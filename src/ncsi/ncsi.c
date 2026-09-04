////////////////////////////////////////////////////////////////////////////////
///
/// @file       ncsi.c
///
/// @brief      NC-SI (DSP0222) device/responder protocol engine.
///
/// Ported from meklort/bcm5719-fw, libs/NCSI/ncsi.c (BSD-3-Clause,
/// Copyright (c) 2018-2019 Evan Lojewski -- see
/// third_party/ncsi/LICENSE for the original license text). That project
/// implements this same command engine for the Broadcom BCM5719's APE
/// coprocessor; this file is a mechanical port to the generic Network.h HAL
/// declared in this directory, for use on a bare-metal / minimal-RTOS
/// RISC-V32 target instead of BCM5719 hardware. Differences from upstream:
///
///   - Per-channel state (NcsiChannelInfo.bits.*, NcsiChannelStatus, etc.)
///     used to live in APE<->main-chip shared memory (port->shm_channel);
///     here it's just NetworkPort_t::state, a plain RAM struct (see
///     Network.h). Field names were kept 1:1 with upstream's SHM bit names
///     so this is a storage-location swap, not a reinterpretation of
///     protocol behavior.
///   - NCSI_TxPacket()/NCSI_TxBePacket() transmitted via BCM5719's APE_PERI
///     hardware TX FIFO; here they call Network_TxFrame() (see Network.h),
///     which you implement against your target's MAC driver.
///   - getLinkStatusHandler() read a Broadcom-specific "Auxiliary Status
///     Summary" PHY register (0x19); here it reads the standard MII BMSR
///     (register 1) link-status bit, so it works with any IEEE 802.3 PHY.
///   - Removed BCM5719 PCI-config-space lookups in getVersionID() (a
///     RISC-V32 NC-SI device is generally not a PCI function); the PCI ID
///     fields come from ncsi_board_config.h instead (default: DSP0222's
///     "not used" sentinel, 0x0000 per clause 8.4.44.4).
///   - Removed APE_aquireLock()/APE_releaseLock() calls around PHY access
///     (that lock arbitrated APE-vs-main-chip access to shared MDIO
///     hardware on BCM5719; add your own locking inside MII_readRegister()
///     if your target's MDIO bus is shared with another driver/core).
///   - Package/Channel ID were hardcoded to 0 upstream (single-package,
///     single-channel BCM5719 firmware). This board is one DSP0222 package
///     with *two* channels, each answered by a separate physical IC running
///     this same firmware image (their RMII/RBT pins are tied together and
///     wired to one BMC sideband port) -- so each instance now reads its own
///     package ID and channel ID at NCSI_init() (gLocalPackageId /
///     gLocalChannelId, via ncsi_board_config.h's
///     NCSI_BoardConfig_ReadPackageID()/ReadChannelID(), both GPIO-backed on
///     this board) and only answers frames addressed to that identity --
///     every other frame on the shared bus (including ones for the other
///     IC's channel) is silently ignored, never answered with an error.
///     gPackageState.port[0] is always this chip's *own* one local channel;
///     it is never indexed by the wire channel number.
///   - handleNCSIFrame() now bounds-checks `command` (a full uint8_t off the
///     wire, 0-255) against gNCSIHandlers[]'s actual size before indexing
///     it. Upstream indexes it unchecked; gNCSIHandlers[] is only sized to
///     its highest designated-initializer entry (0x1A -> 27 elements), so
///     any command past that -- OEM Command (0x50) included -- read past
///     the array as a fabricated ncsi_handler_t and would have called
///     through whatever garbage happened to sit in its `fn` field.
///     Confirmed with AddressSanitizer (global-buffer-overflow at the
///     `handler->fn` dereference) before the fix; see
///     tests/test_ncsi.c's "gNCSIHandlers[] out-of-bounds read" block.
///   - Added optional DSP0222 Checksum computation (upstream always sends
///     the 0/0 "not calculated" sentinel; NCSI_SetComputeChecksum()/
///     NCSI_GetComputeChecksum(), default off -- see
///     NCSI_COMPUTE_CHECKSUM_DEFAULT in ncsi_board_config.h). Wired into
///     every response builder via NCSI_APPLY_CHECKSUM_IF_ENABLED() below;
///     see ncsi_compute_checksum() in types.h for the DSP0222 clause 8.2.2.3
///     algorithm this implements.
///   - Ethernet.h's struct layouts were rewritten directly against the real
///     DSP0222 v1.2.1 spec text (provided for this port) after it turned up
///     three real bugs in the layout this port had inherited from upstream:
///     the NC-SI header being 2 bytes short, spurious reserved gaps in
///     response payloads, and Checksum being incorrectly split into two
///     non-adjacent halves instead of one contiguous 4-byte field. See
///     Ethernet.h's header comment for details. This rewrite also fixed the
///     PCI-ID and Manufacturer-ID "unused" sentinel values (0x0000 and
///     0xFFFFFFFF respectively per clauses 8.4.44.4/8.4.44.5 -- this port
///     previously used 0xFFFF for both, which was wrong for the PCI IDs).
///   - Added DSP0222 Pass-through support (clauses 6.1.11/6.1.12): tracks
///     configured MAC address filters (Set MAC Address) so the
///     BMC-to-network direction can check the DSP0222-mandated "source MAC
///     matches a configured unicast filter" condition; see
///     ncsi_passthrough_tx_from_mc() below and its caller in
///     ncsi_hal_template.c. The network-to-BMC direction
///     (NCSI_handlePassthrough() / Network_PassthroughRxPacket()) existed
///     as scaffolding before this port; its gating condition was corrected
///     to check state.enabled (clause 6.1.12: "after the channel has been
///     enabled") rather than state.ready.
///
/// Known gaps carried over from upstream (see project README before relying
/// on this in production):
///   - VLAN and broadcast/multicast filtering settings are tracked (Set
///     VLAN Filter, Enable/Disable VLAN, Enable/Disable Broadcast Filter
///     all record their configured values now) but not actually enforced
///     against traffic -- this port has no packet-classification path in
///     software; enforcing these is expected to happen in your MAC/PHY
///     driver or hardware filter, informed by the tracked state.
///   - Get Parameters (0x17) is DSP0222-*mandatory* but not implemented
///     (unknownHandler() answers "unsupported") -- unlike the commands
///     below, this one should not ship unimplemented. Its response payload
///     length varies with how many MAC/VLAN filters are supported (clause
///     8.4.48), which is more involved than this port's fixed-size response
///     structs support today.
///   - OEM Command (0x50) and the truly optional Get Controller Packet
///     Statistics / Get NC-SI Statistics / Get Pass-through Statistics /
///     Set NC-SI Flow Control / Enable-Disable Global Multicast Filtering
///     commands are not implemented (unknownHandler() answers them
///     "unsupported") -- DSP0222 does not require these.
///
////////////////////////////////////////////////////////////////////////////////

#include <NCSI.h>
#include <Network.h>
#include <ncsi_board_config.h>

#ifdef NCSI_DEBUG
#include <stdio.h>
#define NCSI_LOG(...) printf(__VA_ARGS__)
#else
#define NCSI_LOG(...) ((void)0)
#endif

/* Local port array size -- always 1: each firmware instance manages
 * exactly one channel (its own, identified by gLocalChannelId below), never
 * "the other IC's" channel. Kept as a named constant (matching upstream's
 * MAX_CHANNELS) rather than a bare 1 for readability at the two use sites
 * below. */
#define MAX_CHANNELS NCSI_MAX_CHANNELS

#define PACKAGE_ID_SHIFT 5
#define CHANNEL_ID_MASK (0x1F)
#define CHANNEL_ID_PACKAGE (0x1F)

/* This firmware instance's own DSP0222 identity -- who it answers as on the
 * shared NC-SI bus. Every frame on the bus reaches every device on it
 * (RMII/RBT is multi-drop), so these gate which frames get a response at
 * all; see the addressing check at the top of handleNCSIFrame(). Read once
 * from board config (GPIO) during NCSI_init(). */
static uint8_t gLocalPackageId;
static uint8_t gLocalChannelId;

/* Whether outgoing responses get a real Checksum computed -- see
 * NCSI_SetComputeChecksum()/NCSI_GetComputeChecksum() (NCSI.h) and
 * ncsi_compute_checksum() (types.h) for what this does and its caveats. */
static bool gNcsiComputeChecksum = NCSI_COMPUTE_CHECKSUM_DEFAULT;

void NCSI_SetComputeChecksum(bool enable)
{
    gNcsiComputeChecksum = enable;
}

bool NCSI_GetComputeChecksum(void)
{
    return gNcsiComputeChecksum;
}

/* Computes and writes obj's checksum[4] field if checksums are currently
 * enabled; no-op otherwise (leaves the caller's existing 0/0/0/0 "not
 * calculated" bytes in place). `structtype` must be one of the Ethernet.h
 * response structs -- anything with ManagmentControllerID and checksum
 * fields. */
#define NCSI_APPLY_CHECKSUM_IF_ENABLED(structtype, obj)                                  \
    do                                                                                   \
    {                                                                                    \
        if (gNcsiComputeChecksum)                                                        \
        {                                                                                \
            uint32_t region_start_ = offsetof(structtype, ManagmentControllerID);        \
            uint32_t checksum_offset_ = offsetof(structtype, checksum);                  \
            ncsi_compute_checksum((uint8_t *)&(obj), region_start_,                      \
                                   checksum_offset_ - region_start_, checksum_offset_);   \
        }                                                                                 \
    } while (0)

/* Response frame - global and usable by one thread at a time only. */
NetworkFrame_t gResponseFrame =
{
    .responsePacket = {
        .DestinationAddress = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .SourceAddress =      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},

        .HeaderRevision = 1,
        .ManagmentControllerID = 0,
        .EtherType = {ETHER_TYPE_NCSI_HI, ETHER_TYPE_NCSI_LO},
        .ChannelID = 0,         /* Filled in by appropriate handler. */
        .ControlPacketType = 0, /* Filled in by appropriate handler. */
        .InstanceID = 0,        /* Filled in by appropriate handler. */
        .reserved_0 = 0,
        .reserved_2 = {0, 0, 0, 0, 0, 0, 0, 0},
        .PayloadLength = {0, 4},

        .ResponseCode = {0, 0}, /* Filled in by appropriate handler. */
        .ReasonCode = {0, 0},   /* Filled in by appropriate handler. */
        .checksum = {0, 0, 0, 0},
    },
};

NetworkFrame_t gLinkStatusResponseFrame =
{
    .linkStatusResponse = {
        .DestinationAddress = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .SourceAddress =      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},

        .HeaderRevision = 1,
        .ManagmentControllerID = 0,
        .EtherType = {ETHER_TYPE_NCSI_HI, ETHER_TYPE_NCSI_LO},
        .ChannelID = 0,         /* Filled in by appropriate handler. */
        .ControlPacketType = CONTROL_PACKET_TYPE_RESPONSE | CONTROL_PACKET_TYPE_GET_LINK_STATUS,
        .InstanceID = 0,        /* Filled in by appropriate handler. */
        .reserved_0 = 0,
        .reserved_2 = {0, 0, 0, 0, 0, 0, 0, 0},
        .PayloadLength = {0, 16},

        .ResponseCode = {0, NCSI_RESPONSE_CODE_COMMAND_COMPLETE},
        .ReasonCode = {0, NCSI_REASON_CODE_NONE},
        .LinkStatus = {0, 0, 0, 0},
        .OtherIndications = {0, 0, 0, 0},
        .OEMLinkStatus = {0, 0, 0, 0},
        .checksum = {0, 0, 0, 0},
    },
};

NetworkFrame_t gCapabilitiesFrame =
{
    .capabilities = {
        .DestinationAddress = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .SourceAddress =      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},

        .HeaderRevision = 1,
        .ManagmentControllerID = 0,
        .EtherType = {ETHER_TYPE_NCSI_HI, ETHER_TYPE_NCSI_LO},
        .ChannelID = 0,         /* Filled in by appropriate handler. */
        .ControlPacketType = CONTROL_PACKET_TYPE_RESPONSE | CONTROL_PACKET_TYPE_GET_CAPABILITIES,
        .InstanceID = 0,        /* Filled in by appropriate handler. */
        .reserved_0 = 0,
        .reserved_2 = {0, 0, 0, 0, 0, 0, 0, 0},
        .PayloadLength = {0, 32},

        .ResponseCode = {0, NCSI_RESPONSE_CODE_COMMAND_COMPLETE},
        .ReasonCode = {0, NCSI_REASON_CODE_NONE},
        .CapabilitiesFlags = {0, 0, 0, 0},
        .BroadcastPacketFilterCapabilities = {0, 0, 0, 0xF}, /* Table 73 bits 0-3: ARP/DHCP client/DHCP server/NetBIOS */
        .MulticastPacketFilterCapabilities = {0, 0, 0, 0},
        .BufferingCapability = {0, 0, 0, 0}, /* 0 = unspecified (clause 8.4.46.4) */
        .AENControlSupport = {0, 0, 0, 0x7}, /* Table 42 bits 0-2: Link Status Change/Config Required/Host NC Driver Status Change */
        .VLANFilterCount = 1,
        .MixedFilterCount = 1,
        .MulticastFilterCount = 1,
        .UnicastFilterCount = 1,
        .reserved_1 = {0, 0},
        .VLANModeSupport = 0x7, /* Table 91 bits 0-2: VLAN only / VLAN+non-VLAN / Any VLAN+non-VLAN */
        .ChannelCount = NCSI_PACKAGE_CHANNEL_COUNT, /* total channels in this *package* (both ICs), not "how many this instance implements" (always 1) */
        .checksum = {0, 0, 0, 0},
    },
};

NetworkFrame_t gVersionFrame =
{
    .version = {
        .DestinationAddress = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .SourceAddress =      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},

        .HeaderRevision = 1,
        .ManagmentControllerID = 0,
        .EtherType = {ETHER_TYPE_NCSI_HI, ETHER_TYPE_NCSI_LO},
        .ChannelID = 0,         /* Filled in by appropriate handler. */
        .ControlPacketType = CONTROL_PACKET_TYPE_RESPONSE | CONTROL_PACKET_TYPE_GET_VERSION_ID,
        .InstanceID = 0,        /* Filled in by appropriate handler. */
        .reserved_0 = 0,
        .reserved_2 = {0, 0, 0, 0, 0, 0, 0, 0},
        .PayloadLength = {0, 40}, /* ResponseCode+ReasonCode(4) + Major/Minor/Update/Alpha1(4) + reserved_1(3) + Alpha2(1) + FirmwareNameString(12) + FirmwareVersion(4) + PCIDID/PCIVID/PCISSID/PCISVID(8) + ManufacturerID(4) = 40; checksum starts at byte 70 (30+40) */

        .ResponseCode = {0, NCSI_RESPONSE_CODE_COMMAND_COMPLETE},
        .ReasonCode = {0, NCSI_REASON_CODE_NONE},

        /* NC-SI Version (clause 8.4.44.1): BCD-encoded with an 0xF-prefixed
         * MS nibble meaning "single digit value". DSP0222 v1.2.1 mandates
         * reporting compatibility as "1.2.0" (0xF1F2F00000) regardless --
         * this is a documentation-only spec revision, not a wire-format
         * change, per the spec's own text. */
        .NCSIMajor = 0xF1,
        .NCSIMinor = 0xF2,
        .NCSIUpdate = 0xF0,
        .NCSIAlpha1 = 0x00,
        .reserved_1 = {0, 0, 0},
        .NCSIAlpha2 = 0x00,

        /* Firmware Name String: left-justified, most-significant byte
         * (leftmost character) first (clause 8.4.44.2). */
        .FirmwareNameString = {'N', 'C', 'S', 'I', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '},

        .FirmwareVersion = {0, NCSI_FW_VERSION_MAJOR, NCSI_FW_VERSION_MINOR, NCSI_FW_VERSION_PATCH},

        /* PCI/Manufacturer identity -- see ncsi_board_config.h to override.
         * "Not used" sentinels per clauses 8.4.44.4 (PCI IDs: 0x0000) and
         * 8.4.44.5 (Manufacturer ID: 0xFFFFFFFF) -- these differ from each
         * other, and this port previously used 0xFFFF for both, which was
         * wrong for the PCI ID fields. */
        .PCIDID = {(uint8_t)(NCSI_PCI_DEVICE_ID >> 8), (uint8_t)NCSI_PCI_DEVICE_ID},
        .PCIVID = {(uint8_t)(NCSI_PCI_VENDOR_ID >> 8), (uint8_t)NCSI_PCI_VENDOR_ID},
        .PCISSID = {(uint8_t)(NCSI_PCI_SUBSYSTEM_DEVICE_ID >> 8), (uint8_t)NCSI_PCI_SUBSYSTEM_DEVICE_ID},
        .PCISVID = {(uint8_t)(NCSI_PCI_SUBSYSTEM_VENDOR_ID >> 8), (uint8_t)NCSI_PCI_SUBSYSTEM_VENDOR_ID},

        .ManufacturerID = {(uint8_t)(NCSI_MANUFACTURER_ID >> 24), (uint8_t)(NCSI_MANUFACTURER_ID >> 16),
                            (uint8_t)(NCSI_MANUFACTURER_ID >> 8), (uint8_t)NCSI_MANUFACTURER_ID},

        .checksum = {0, 0, 0, 0},
    },
};

typedef struct
{
    bool selected;
    NetworkPort_t *port[MAX_CHANNELS];
} package_state_t;

package_state_t gPackageState = {
    .selected = false,
    .port = {
        [0] = NULL,
    },
};

void NCSI_usePort(NetworkPort_t *port)
{
    gPackageState.port[0] = port;
}

void sendNCSIResponse(uint8_t InstanceID, uint8_t channelID, uint16_t controlID, uint16_t response_code, uint16_t reasons_code);
void sendNCSILinkStatusResponse(uint8_t InstanceID, uint8_t channelID, uint32_t LinkStatus, uint32_t OEMLinkStatus, uint32_t OtherIndications);

void resetChannel(void);

typedef struct
{
    bool ignoreInit;
    bool packageCommand;
    int payloadLength;
    void (*fn)(const NetworkFrame_t *);

} ncsi_handler_t;

void unknownHandler(const NetworkFrame_t *frame)
{
    NCSI_LOG("Unhandled Packet Type: %x\n", ncsi_rd16(frame->header.EtherType));
    NCSI_LOG("ManagmentControllerID: %x\n", frame->controlPacket.ManagmentControllerID);
    NCSI_LOG("HeaderRevision: %x\n", frame->controlPacket.HeaderRevision);
    NCSI_LOG("Control Packet Type: %x\n", frame->controlPacket.ControlPacketType);
    NCSI_LOG("Channel ID: %x\n", frame->controlPacket.ChannelID);
    NCSI_LOG("Instance ID: %d\n", frame->controlPacket.InstanceID);
    NCSI_LOG("Payload Length: %d\n", NCSI_PAYLOAD_LENGTH(&frame->controlPacket));

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_UNSUPPORTED, NCSI_REASON_CODE_UNKNOWN_UNSUPPORTED);
}

static void clearInitialStateHandler(const NetworkFrame_t *frame)
{
    /* handleNCSIFrame() has already confirmed this frame is addressed to us
     * (our package, and either our channel or the package-wide indicator)
     * before calling any handler -- no need to re-check the channel here. */
    gPackageState.port[0]->state.ready = true;
    NCSI_LOG("Clear initial state: channel %x\n", frame->controlPacket.ChannelID & CHANNEL_ID_MASK);

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void selectPackageHandler(const NetworkFrame_t *frame)
{
    NCSI_LOG("Package enabled. Arb: %d\n", frame->selectPackage.HardwareArbitartionDisabled & 0x01 ? 0 : 1);
    gPackageState.selected = true;
    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void deselectPackageHandler(const NetworkFrame_t *frame)
{
    NCSI_LOG("Package disabled.\n");
    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
    gPackageState.selected = false;
}

static void enableChannelHandler(const NetworkFrame_t *frame)
{
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    (void)ch; /* only used by NCSI_LOG below */

    NCSI_LOG("Enable Channel: %x\n", ch);
    gPackageState.port[0]->state.enabled = true;

    Network_InitPort(gPackageState.port[0], NCSI_RELOAD_AS_NEEDED);

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void disableChannelHandler(const NetworkFrame_t *frame)
{
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    bool allow_link_down = (frame->disableChannel.ALD & 0x01) != 0;
    (void)ch; (void)allow_link_down; /* only used by NCSI_LOG below; TODO: DSP0222 Table 33 -- if
                             * ALD is set, this channel's external link may be taken down while
                             * disabled (e.g. to save power) as long as no other functionality
                             * (host OS, WoL) needs it; not acted on -- no HAL hook for "take the
                             * link down" exists yet. */

    NCSI_LOG("Disable Channel: %x (ALD=%d)\n", ch, allow_link_down);
    gPackageState.port[0]->state.enabled = false;

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void resetChannelHandler(const NetworkFrame_t *frame)
{
    NCSI_LOG("Reset Channel: %x\n", frame->controlPacket.ChannelID & CHANNEL_ID_MASK);
    resetChannel();

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void enableChannelNetworkTXHandler(const NetworkFrame_t *frame)
{
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    (void)ch; /* only used by NCSI_LOG below */

    NCSI_LOG("Enable Channel Network TX: channel %x\n", ch);
    gPackageState.port[0]->state.tx_passthrough_en = true;

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void disableChannelNetworkTXHandler(const NetworkFrame_t *frame)
{
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    (void)ch; /* only used by NCSI_LOG below */

    NCSI_LOG("Disable Channel Network TX: %x\n", ch);
    gPackageState.port[0]->state.tx_passthrough_en = false;

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void AENEnableHandler(const NetworkFrame_t *frame)
{
    uint32_t AENControl = ncsi_rd32(frame->AENEnable.AENControl);
    NCSI_LOG("AEN Enable: AEN_MC_ID %x\n", frame->AENEnable.AEN_MC_ID);
    NCSI_LOG("AEN Enable: AENControl %x\n", AENControl);

    gPackageState.port[0]->state.aen_mc_id = frame->AENEnable.AEN_MC_ID;
    gPackageState.port[0]->state.aen_control = AENControl;

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void setLinkHandler(const NetworkFrame_t *frame)
{
    uint32_t LinkSettings = ncsi_rd32(frame->setLink.LinkSettings);
    uint32_t OEMLinkSettings = ncsi_rd32(frame->setLink.OEMLinkSettings);
    NCSI_LOG("Set Link: LinkSettings %x\n", LinkSettings);
    NCSI_LOG("Set Link: OEMLinkSettings %x\n", OEMLinkSettings);

    NetworkPort_t *port = gPackageState.port[0];
    port->state.link_settings = LinkSettings;
    port->state.oem_link_settings = OEMLinkSettings;

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void getLinkStatusHandler(const NetworkFrame_t *frame)
{
    NetworkPort_t *port = gPackageState.port[0];
    uint8_t phy = MII_getPhy(port->device);

    uint32_t rx_net = port->state.stat_net_rx;
    uint32_t tx_net = port->state.stat_net_tx;
    uint32_t rx_ncsi = port->state.stat_ncsi_rx;
    uint32_t tx_ncsi = port->state.stat_ncsi_tx;
    (void)rx_net; (void)tx_net; (void)rx_ncsi; (void)tx_ncsi; /* only used by NCSI_LOG below */

    bool phy_link_up;
    int32_t reg = MII_readRegister(port->device, phy, REG_MII_BASIC_STATUS);
    if (reg >= 0)
    {
        phy_link_up = ((uint16_t)reg & MII_BMSR_LINK_STATUS_BIT) != 0;
        port->state.link_status.link_up = phy_link_up;
        port->state.link_status.autoneg_hcd = phy_link_up;
    }
    else
    {
        /* Unable to read status register. Re-use previous cached value. */
        NCSI_LOG("Error determining Link Status [%d]", frame->controlPacket.ChannelID);
        phy_link_up = port->state.link_status.link_up;
    }

    NCSI_LOG("Link Status [%d] %s, NCSI TX/RX 0x%08X/0x%08X Net TX/RX 0x%08X/0x%08X\n", frame->controlPacket.ChannelID,
             phy_link_up ? "up" : "down", tx_ncsi, rx_ncsi, tx_net, rx_net);

    if (!phy_link_up)
    {
        if (!Network_isLinkUp(port))
        {
            NCSI_LOG("Resetting link.\n");
            Network_resetLink(port);
        }
    }

    port->state.link_status.link_up = phy_link_up;
    port->state.link_status.autoneg_enabled = 1;

    /* WARNING -- placeholder Link Status encoding, not yet spec-complete.
     * DSP0222 Table 51's "Link Status" field is: bit 0 link flag, bits 4:1
     * a *4-bit* speed/duplex enumeration, bit 5 autoneg enable, bit 6
     * autoneg complete, bit 7 parallel detection, etc. This only sets a
     * single bit for "hcd" instead of the real 4-bit speed/duplex code,
     * and does not report autoneg_complete or parallel detection at all
     * (no HAL call surfaces them yet). Fill this in against Table 51 --
     * e.g. have MII_readRegister()/your PHY driver resolve actual
     * negotiated speed/duplex and extend NetworkPort_t::state accordingly
     * -- before relying on BMC-side link/speed reporting. */
    uint32_t LinkStatus = port->state.link_status.link_up |
                           (port->state.link_status.autoneg_hcd << 1) |
                           (port->state.link_status.autoneg_enabled << 5) |
                           (port->state.link_status.autoneg_complete << 6);
    uint32_t OEMLinkStatus = 0;
    uint32_t OtherIndications = 0;

    sendNCSILinkStatusResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, LinkStatus, OEMLinkStatus, OtherIndications);
}

static void disableVLANHandler(const NetworkFrame_t *frame)
{
    /* TODO: tracked, not enforced -- see ncsi.c's top-of-file comment. */
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    (void)ch; /* only used by NCSI_LOG below */
    NetworkPort_t *port = gPackageState.port[0];
    port->state.vlan_enabled = false;

    NCSI_LOG("Disable VLAN: channel %x\n", ch);

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void getCapabilities(const NetworkFrame_t *frame)
{
    uint32_t packetSize = MAX(sizeof(gCapabilitiesFrame.capabilities), ETHERNET_FRAME_MIN);

    /* Echo the full ChannelID byte (package bits included), not just the
     * masked channel -- with package IDs no longer hardcoded to 0, dropping
     * them here would send back a response addressed to the wrong package. */
    gCapabilitiesFrame.capabilities.ChannelID = frame->controlPacket.ChannelID;
    gCapabilitiesFrame.capabilities.ControlPacketType = frame->controlPacket.ControlPacketType | CONTROL_PACKET_TYPE_RESPONSE;
    gCapabilitiesFrame.capabilities.InstanceID = frame->controlPacket.InstanceID;
    ncsi_wr16(gCapabilitiesFrame.capabilities.ResponseCode, NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    ncsi_wr16(gCapabilitiesFrame.capabilities.ReasonCode, NCSI_REASON_CODE_NONE);

    NCSI_APPLY_CHECKSUM_IF_ENABLED(CapabilitiesResponsePacket_t, gCapabilitiesFrame.capabilities);
    NCSI_TxPacket((const uint8_t *)&gCapabilitiesFrame, packetSize);
}

static void getVersionID(const NetworkFrame_t *frame)
{
    uint32_t packetSize = MAX(sizeof(gVersionFrame.version), ETHERNET_FRAME_MIN);

    /* See getCapabilities() above: echo the full ChannelID, package bits
     * included. */
    gVersionFrame.version.ChannelID = frame->controlPacket.ChannelID;
    gVersionFrame.version.ControlPacketType = frame->controlPacket.ControlPacketType | CONTROL_PACKET_TYPE_RESPONSE;
    gVersionFrame.version.InstanceID = frame->controlPacket.InstanceID;
    ncsi_wr16(gVersionFrame.version.ResponseCode, NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    ncsi_wr16(gVersionFrame.version.ReasonCode, NCSI_REASON_CODE_NONE);

    NCSI_APPLY_CHECKSUM_IF_ENABLED(VersionResponsePacket_t, gVersionFrame.version);
    NCSI_TxPacket((const uint8_t *)&gVersionFrame, packetSize);
}

static void enableVLANHandler(const NetworkFrame_t *frame)
{
    /* TODO: tracked, not enforced -- see ncsi.c's top-of-file comment. */
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    (void)ch; /* only used by NCSI_LOG below */
    uint8_t mode = frame->enableVLAN.Mode; /* Table 62: 0x01 VLAN only, 0x02 VLAN+non-VLAN, 0x03 Any VLAN+non-VLAN */
    NetworkPort_t *port = gPackageState.port[0];
    port->state.vlan_enabled = (mode != 0);

    NCSI_LOG("Enable VLAN: channel %x mode %x\n", ch, mode);

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void setVLANFilter(const NetworkFrame_t *frame)
{
    /* TODO: tracked, not enforced -- see ncsi.c's top-of-file comment. */
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    (void)ch; /* only used by NCSI_LOG below */
    uint16_t vlan_id = ncsi_rd16(frame->setVLANFilter.VLANID) & 0x0FFFu;
    uint8_t filter_selector = frame->setVLANFilter.FilterSelector;
    (void)filter_selector; /* only used by NCSI_LOG below */
    bool enable = (frame->setVLANFilter.Enable & 0x01) != 0;

    NCSI_LOG("Set VLAN Filter: channel %x filter %u vlan %u enable %d\n", ch, filter_selector, vlan_id, enable);

    /* Table 60: VLAN ID 0 is invalid when enabling a filter. */
    if (enable && vlan_id == 0)
    {
        sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                         NCSI_RESPONSE_CODE_COMMAND_FAILED, 0x0B07 /* VLAN Tag Is Invalid */);
        return;
    }

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void setMACAddressHandler(const NetworkFrame_t *frame)
{
    uint32_t MACNumber = frame->setMACAddr.MACNumber;
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    (void)ch; /* only used by NCSI_LOG below */
    NetworkPort_t *port = gPackageState.port[0];

    const uint8_t *mac = frame->setMACAddr.MACAddress; /* wire order: mac[0]=byte5 (MSB) .. mac[5]=byte0 (LSB) */
    uint8_t enable = SETMAC_ENABLE(frame->setMACAddr.MACInfo);
    uint8_t at = SETMAC_AT(frame->setMACAddr.MACInfo);

    NCSI_LOG("Set MAC: channel %x\n", ch);
    NCSI_LOG("  MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    NCSI_LOG("  Enable: %d\n", enable);
    NCSI_LOG("  AT: %d\n", at);
    NCSI_LOG("  MACNumber: %d\n", frame->setMACAddr.MACNumber);

    /* NC-SI has the mac starting at 1, reindex based at 0. */
    if (MACNumber > 0)
    {
        MACNumber--;
    }

    uint16_t mac_high16 = ((uint16_t)mac[0] << 8) | mac[1];
    uint32_t mac_low32 = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
    Network_SetMACAddr(port, mac_high16, mac_low32, MACNumber, enable ? true : false);

    /* Track the filter locally too (see ncsi_passthrough.c) so the
     * BMC-to-network Pass-through direction can check the DSP0222-mandated
     * "source MAC matches a configured unicast filter" condition without
     * round-tripping through the MAC driver. */
    if (MACNumber < NCSI_MAX_MAC_FILTERS)
    {
        port->state.mac_filters[MACNumber].enabled = (enable != 0);
        port->state.mac_filters[MACNumber].address_type = at;
        for (int i = 0; i < 6; i++)
        {
            port->state.mac_filters[MACNumber].mac[i] = mac[i];
        }
    }
    else
    {
        NCSI_LOG("Set MAC: filter slot %u out of range (max %u)\n", MACNumber, (unsigned)NCSI_MAX_MAC_FILTERS);
    }

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void enableBroadcastFilteringHandler(const NetworkFrame_t *frame)
{
    /* TODO: tracked, not enforced -- see ncsi.c's top-of-file comment. */
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    (void)ch; /* only used by NCSI_LOG below */
    NetworkPort_t *port = gPackageState.port[0];
    port->state.broadcast_filter_settings = ncsi_rd32(frame->enableBroadcastFilter.BroadcastPacketFilterSettings);

    NCSI_LOG("Enable Broadcast Filtering: channel %x settings %x\n", ch, port->state.broadcast_filter_settings);

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

static void disableBroadcastFilteringHandler(const NetworkFrame_t *frame)
{
    int ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    (void)ch; /* only used by NCSI_LOG below */
    NetworkPort_t *port = gPackageState.port[0];
    port->state.broadcast_filter_settings = 0; /* clause 8.4.35: reception of all broadcast frames re-enabled */

    NCSI_LOG("Disable Broadcast Filtering: channel %x\n", ch);

    sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                     NCSI_RESPONSE_CODE_COMMAND_COMPLETE, NCSI_REASON_CODE_NONE);
}

/* CLEAR INITIAL STATE, SELECT PACKAGE, DESELECT PACKAGE, ENABLE CHANNEL, DISABLE CHANNEL, RESET CHANNEL, ENABLE CHANNEL NETWORK TX, DISABLE CHANNEL NETWORK TX,
 * AEN ENABLE, SET LINK;   then you need GET LINK STATUS */
ncsi_handler_t gNCSIHandlers[] = {
    /* Package / Initialization commands */
    [0x00] = { .ignoreInit = true, .packageCommand = false, .payloadLength = 0, .fn = clearInitialStateHandler },
    [0x01] = { .ignoreInit = false, .packageCommand = true, .payloadLength = 4, .fn = selectPackageHandler },
    [0x02] = { .ignoreInit = false, .packageCommand = true, .payloadLength = 0, .fn = deselectPackageHandler },

    /* Channel Specific commands */
    [0x03] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = enableChannelHandler },
    [0x04] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 4, .fn = disableChannelHandler },
    [0x05] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 4, .fn = resetChannelHandler },
    [0x06] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = enableChannelNetworkTXHandler },
    [0x07] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = disableChannelNetworkTXHandler },
    [0x08] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 8, .fn = AENEnableHandler }, /* Conditional */
    [0x09] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 8, .fn = setLinkHandler },
    [0x0A] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = getLinkStatusHandler },
    [0x0B] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 8, .fn = setVLANFilter },
    [0x0C] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 4, .fn = enableVLANHandler },
    [0x0D] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = disableVLANHandler },
    [0x0E] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 8, .fn = setMACAddressHandler },
    [0x10] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 4, .fn = enableBroadcastFilteringHandler },
    [0x11] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = disableBroadcastFilteringHandler },
    [0x12] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 4, .fn = unknownHandler },
    [0x13] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = unknownHandler },
    [0x14] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 4, .fn = unknownHandler }, /* Optional */
    [0x15] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = getVersionID },
    [0x16] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = getCapabilities },
    [0x17] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = unknownHandler },
    [0x18] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = unknownHandler }, /* Optional */
    [0x19] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = unknownHandler }, /* Optional */
    [0x1A] = { .ignoreInit = false, .packageCommand = false, .payloadLength = 0, .fn = unknownHandler }, /* Optional */
};

void handleNCSIFrame(const NetworkFrame_t *frame)
{
    /* RMII/RBT is a multi-drop bus: every device on it -- both ICs of this
     * card's pair, and any other package sharing the bus -- sees every
     * frame. Addressing (package + channel) determines who is allowed to
     * answer; getting this wrong means two responders answering the same
     * BMC query. Checked once, up front, for every command (known or not),
     * rather than per-handler as upstream (single-package, single-channel)
     * did it. */
    uint8_t package = frame->controlPacket.ChannelID >> PACKAGE_ID_SHIFT;
    if (package != gLocalPackageId)
    {
        return; /* not our package -- stay silent */
    }

    uint8_t ch = frame->controlPacket.ChannelID & CHANNEL_ID_MASK;
    bool addressed_to_us = (ch == CHANNEL_ID_PACKAGE) || (ch == gLocalChannelId);
    if (!addressed_to_us)
    {
        /* Same package, but the other IC's channel -- e.g. this OCP NIC 3.0
         * card's channel-1 chip seeing a command addressed to channel 0.
         * Stay silent; the chip that owns that channel will answer. */
        return;
    }

    uint8_t command = frame->controlPacket.ControlPacketType;
    uint16_t payloadLength = NCSI_PAYLOAD_LENGTH(&frame->controlPacket);
    NetworkPort_t *port = gPackageState.port[0];

    /* command is a full uint8_t (0-255) straight off the wire; gNCSIHandlers[]
     * is only sized to its highest designated-initializer index (0x1A) --
     * OEM Command (0x50) alone, let alone a malformed/garbage byte, would
     * index past the end of it. Confirmed with AddressSanitizer
     * (global-buffer-overflow) before this bounds check existed: an
     * out-of-range command would have read whatever bytes happen to sit
     * past the array as a fabricated ncsi_handler_t, including its `fn`
     * pointer, and called through it -- a real memory-safety bug reachable
     * by a single command byte, not just a theoretical one. */
    ncsi_handler_t *handler = (command < ARRAY_ELEMENTS(gNCSIHandlers)) ? &gNCSIHandlers[command] : NULL;

    if (handler && handler->fn)
    {
        if (handler->payloadLength != payloadLength)
        {
            NCSI_LOG("[%x] Unexpected payload length: 0x%04x != 0x%04x\n", command, handler->payloadLength, payloadLength);
            sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                             NCSI_RESPONSE_CODE_COMMAND_FAILED, NCSI_REASON_CODE_INVALID_PAYLOAD_LENGTH);
        }
        else if ((handler->packageCommand && ch == CHANNEL_ID_PACKAGE) || /* Package commands are always accepted. */
                 (handler->ignoreInit))
        {
            NCSI_LOG("[%x] packageCommand/ignore init channel: %d\n", command, ch);

            if (port)
            {
                port->state.stat_ncsi_rx++;
            }
            gPackageState.selected = true;
            handler->fn(frame);
        }
        else if (ch != gLocalChannelId)
        {
            /* A channel-specific command type (not packageCommand, not
             * ignoreInit) sent with the package-wide channel indicator
             * (0x1F) instead of a real channel -- not valid for these. */
            NCSI_LOG("[%x] Channel-specific command sent to package indicator\n", command);
            sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                             NCSI_RESPONSE_CODE_COMMAND_FAILED, NCSI_REASON_CODE_INVALID_PARAM);
        }
        else
        {
            gPackageState.selected = true;
            if (false == port->state.ready)
            {
                NCSI_LOG("[%x] Channel not initialized: %d\n", command, ch);
                sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                                 NCSI_RESPONSE_CODE_COMMAND_FAILED, NCSI_REASON_CODE_INITIALIZATION_REQUIRED);
            }
            else
            {
                port->state.stat_ncsi_rx++;
                handler->fn(frame);
            }
        }
    }
    else
    {
        NCSI_LOG("[%x] Unknown command\n", command);
        sendNCSIResponse(frame->controlPacket.InstanceID, frame->controlPacket.ChannelID, frame->controlPacket.ControlPacketType,
                         NCSI_RESPONSE_CODE_COMMAND_UNSUPPORTED, NCSI_REASON_CODE_UNKNOWN_UNSUPPORTED);
    }
}

/* Each firmware instance manages exactly one local channel (this chip's
 * own), always at gPackageState.port[0] -- there is no "which channel"
 * parameter to take, unlike upstream's multi-channel-per-instance design. */
void resetChannel(void)
{
    NetworkPort_t *port = gPackageState.port[0];

    port->state.ready = false;
    port->state.enabled = false;
    port->state.tx_passthrough_en = false;
    port->state.vlan_enabled = false;
    port->state.broadcast_filter_settings = 0;
    for (unsigned int i = 0; i < NCSI_MAX_MAC_FILTERS; i++)
    {
        port->state.mac_filters[i].enabled = false;
    }
    port->state.stat_ncsi_rx = 0;
    port->state.stat_ncsi_tx = 0;
    port->state.stat_net_rx = 0;
    port->state.stat_net_tx = 0;

    uint8_t phy = MII_getPhy(port->device);
    bool success = MII_reset(port->device, phy);

    if (!success)
    {
        NCSI_LOG("resetChannel: Error writing register.\n");
    }
}

void reloadChannel(reload_type_t reset_phy)
{
    NetworkPort_t *port = gPackageState.port[0];
    port->state.stat_ncsi_rx = 0;
    port->state.stat_ncsi_tx = 0;
    port->state.stat_net_rx = 0;
    port->state.stat_net_tx = 0;

    Network_InitPort(port, reset_phy);
}

static inline bool NCSI_TxPacket_internal(const uint8_t *packet, uint32_t packet_len, bool swap_words)
{
    NetworkPort_t *port = gPackageState.port[0];
    if (!port)
    {
        return false;
    }

    if (!swap_words)
    {
        return Network_TxFrame(port, packet, packet_len);
    }

    /* Upstream's NCSI_TxBePacket() 32-bit-word byte-swap was needed because
     * BCM5719's APE_PERI TX FIFO is word-oriented; a generic MAC driver's
     * Network_TxFrame() just wants the raw frame bytes in transmit order,
     * so this is unlikely to be needed by any caller on this target -- kept
     * only for API parity with upstream. Swaps each full 4-byte group;
     * byte-indexed (not uint32_t-aliased) to stay alignment-safe. */
    uint8_t swapped[256]; /* NC-SI control frames are well under this. */
    if (packet_len > sizeof(swapped))
    {
        return false;
    }

    uint32_t full_words = packet_len / 4;
    for (uint32_t i = 0; i < full_words; i++)
    {
        uint32_t off = i * 4;
        swapped[off + 0] = packet[off + 3];
        swapped[off + 1] = packet[off + 2];
        swapped[off + 2] = packet[off + 1];
        swapped[off + 3] = packet[off + 0];
    }
    for (uint32_t i = full_words * 4; i < packet_len; i++)
    {
        swapped[i] = packet[i];
    }

    return Network_TxFrame(port, swapped, packet_len);
}

void NCSI_TxPacket(const uint8_t *packet, uint32_t packet_len)
{
    if (NCSI_TxPacket_internal(packet, packet_len, false))
    {
        NetworkPort_t *port = gPackageState.port[0];
        port->state.stat_ncsi_tx++;
    }
    else
    {
        NCSI_LOG("Error transmitting NCSI packet.\n");
    }
}

void NCSI_TxBePacket(const uint8_t *packet, uint32_t packet_len)
{
    (void)NCSI_TxPacket_internal(packet, packet_len, true);
}

void sendNCSILinkStatusResponse(uint8_t InstanceID, uint8_t channelID, uint32_t LinkStatus, uint32_t OEMLinkStatus, uint32_t OtherIndications)
{
    uint32_t packetSize = MAX(sizeof(gLinkStatusResponseFrame.linkStatusResponse), ETHERNET_FRAME_MIN);

    gLinkStatusResponseFrame.linkStatusResponse.ChannelID = channelID;
    gLinkStatusResponseFrame.linkStatusResponse.InstanceID = InstanceID;
    ncsi_wr16(gLinkStatusResponseFrame.linkStatusResponse.ResponseCode, NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    ncsi_wr16(gLinkStatusResponseFrame.linkStatusResponse.ReasonCode, NCSI_REASON_CODE_NONE);

    ncsi_wr32(gLinkStatusResponseFrame.linkStatusResponse.LinkStatus, LinkStatus);
    ncsi_wr32(gLinkStatusResponseFrame.linkStatusResponse.OEMLinkStatus, OEMLinkStatus);
    ncsi_wr32(gLinkStatusResponseFrame.linkStatusResponse.OtherIndications, OtherIndications);

    NCSI_APPLY_CHECKSUM_IF_ENABLED(LinkStatusResponsePacketHeader_t, gLinkStatusResponseFrame.linkStatusResponse);
    NCSI_TxPacket((const uint8_t *)&gLinkStatusResponseFrame, packetSize);
}

void sendNCSIResponse(uint8_t InstanceID, uint8_t channelID, uint16_t controlID, uint16_t response_code, uint16_t reasons_code)
{
    uint32_t packetSize = MAX(sizeof(gResponseFrame.responsePacket), ETHERNET_FRAME_MIN);

    gResponseFrame.responsePacket.ChannelID = channelID;
    gResponseFrame.responsePacket.ControlPacketType = controlID | CONTROL_PACKET_TYPE_RESPONSE;
    gResponseFrame.responsePacket.InstanceID = InstanceID;

    ncsi_wr16(gResponseFrame.responsePacket.ResponseCode, response_code);
    ncsi_wr16(gResponseFrame.responsePacket.ReasonCode, reasons_code);

    NCSI_APPLY_CHECKSUM_IF_ENABLED(ResponsePacketHeader_t, gResponseFrame.responsePacket);
    NCSI_TxPacket((const uint8_t *)&gResponseFrame, packetSize);
}

void NCSI_init(void)
{
    gLocalPackageId = NCSI_BoardConfig_ReadPackageID();
    gLocalChannelId = NCSI_BoardConfig_ReadChannelID();
    NCSI_LOG("NC-SI identity: package %u, channel %u\n", gLocalPackageId, gLocalChannelId);

    NCSI_LOG("Resetting channel...\n");
    resetChannel();
}

void NCSI_reload(reload_type_t reset_phy)
{
    reloadChannel(reset_phy);
}

/* --- Pass-through (DSP0222 clauses 6.1.11 / 6.1.12) ----------------------
 * Two independent directions; see each function's comment. */

void NCSI_handlePassthrough(void)
{
    /* Network-to-BMC direction (clause 6.1.12): "after the channel has
     * been enabled, any packet that the Network Controller receives for
     * the Management Controller shall be forwarded to the Management
     * Controller." Gates on state.enabled (Enable Channel received), not
     * just state.ready (Clear Initial State received) -- these are
     * different DSP0222 states and the spec text for this clause
     * specifically says "enabled". */
    for (unsigned int ch = 0; ch < ARRAY_ELEMENTS(gPackageState.port); ch++)
    {
        NetworkPort_t *port = gPackageState.port[ch];

        if (port && port->state.enabled)
        {
            if (!Network_PassthroughRxPacket(port))
            {
                port->state.stat_net_dropped++;
            }
        }
    }
}

bool ncsi_passthrough_tx_from_mc(NetworkPort_t *port, const uint8_t source_mac[6], const uint8_t *frame, uint32_t frame_len)
{
    /* BMC-to-network direction (clause 6.1.11): "Packets not recognized as
     * command packets ... that are received on the Network Controller's
     * NC-SI interface shall be assumed to be Pass-through packets provided
     * that the source MAC Address matches one of the unicast MAC addresses
     * settings (as configured by the Set MAC Address command) ..., and
     * will be forwarded for transmission to the corresponding external
     * network interface if Channel Network TX is enabled." Called from
     * ncsi_on_rx_frame() (ncsi_hal_template.c) for any received frame that
     * isn't an NC-SI control frame. Returns false (frame not forwarded,
     * silently dropped per spec -- Pass-through packets never get an NC-SI
     * response either way) if TX passthrough is disabled or the source MAC
     * doesn't match an enabled unicast filter. */
    if (!port || !port->state.tx_passthrough_en)
    {
        return false;
    }

    bool source_matches = false;
    for (unsigned int i = 0; i < NCSI_MAX_MAC_FILTERS; i++)
    {
        const ncsi_mac_filter_t *filter = &port->state.mac_filters[i];
        if (!filter->enabled || filter->address_type != 0 /* AT: 0 = unicast (Table 68) */)
        {
            continue;
        }
        bool match = true;
        for (int b = 0; b < 6; b++)
        {
            if (filter->mac[b] != source_mac[b])
            {
                match = false;
                break;
            }
        }
        if (match)
        {
            source_matches = true;
            break;
        }
    }

    if (!source_matches)
    {
        return false;
    }

    bool ok = Network_TxFrame(port, frame, frame_len);
    if (ok)
    {
        port->state.stat_net_tx++;
    }
    return ok;
}
