////////////////////////////////////////////////////////////////////////////////
///
/// @file       Network.h
///
/// @brief      Hardware abstraction layer (HAL) that ncsi.c is ported
///             against. Implement every function declared here in your
///             board's Ethernet MAC/PHY/MDIO driver code (see
///             ncsi_hal_template.c for a starting point).
///
/// This is new code written for this port (not from bcm5719-fw): the
/// upstream Network.h lives inside BCM5719's APE firmware tree and isn't
/// public, so its contract had to be reconstructed from how libs/NCSI/ncsi.c
/// calls it. Function names were kept close to upstream for anyone
/// cross-referencing bcm5719-fw; NetworkPort_t's contents are new, replacing
/// upstream's shared-memory (SHM) register channel state with a plain RAM
/// struct, since a RISC-V32 bare-metal/RTOS target has no APE/SHM concept.
///
////////////////////////////////////////////////////////////////////////////////

#ifndef NETWORK_H
#define NETWORK_H

#include <types.h>
#include <Ethernet.h>

/* Number of NC-SI channels THIS FIRMWARE INSTANCE manages locally -- not
 * how many channels exist in the package. Always 1 on this board: each of
 * the two ICs runs this exact same firmware image and each only ever owns
 * ONE channel (0 or 1, read from GPIO20 -- see ncsi_board_config.c). The
 * "second channel" is not a second local port on this chip, it's the
 * *other* physical chip. Do not raise this to 2 to "add the other
 * channel" -- that would make this instance falsely claim ownership of
 * both channels and try to answer commands that are actually the other
 * chip's to answer.
 *
 * (The package-wide total of 2 channels is a *separate* value --
 * NCSI_PACKAGE_CHANNEL_COUNT in ncsi_board_config.h -- used only to fill
 * in the Get Capabilities response's Channel Count field, which reports
 * package topology to the BMC regardless of which chip answers.) */
#define NCSI_MAX_CHANNELS 1

/* --- Per-channel NC-SI state -------------------------------------------
 * Upstream (bcm5719-fw) keeps this in APE<->main-chip shared memory
 * (port->shm_channel->NcsiChannelInfo.bits.*, etc.) because the APE
 * coprocessor and the main BCM5719 firmware are separate CPUs that need to
 * see the same channel state. On a standalone RISC-V32 device there's only
 * one CPU running this stack, so it's just local RAM. Field names mirror
 * the upstream SHM bit names 1:1 so the ncsi.c port is a mechanical
 * find/replace, not a reinterpretation of protocol semantics. */
typedef struct
{
    bool ready;            /* Clear Initial State received (NcsiChannelInfo.bits.Ready) */
    bool enabled;           /* Enable/Disable Channel (NcsiChannelInfo.bits.Enabled) */
    bool tx_passthrough_en; /* Enable/Disable Channel Network TX (NcsiChannelInfo.bits.TXPassthrough) */
    bool vlan_enabled;      /* Enable/Disable VLAN (NcsiChannelInfo.bits.VLAN) */

    uint8_t  aen_mc_id;     /* AEN Enable command's AEN_MC_ID (NcsiChannelMcid) */
    uint32_t aen_control;   /* AEN Enable command's AENControl bitmask (NcsiChannelAen) */

    uint32_t link_settings;      /* Set Link command's LinkSettings (NcsiChannelSetting1) */
    uint32_t oem_link_settings;  /* Set Link command's OEMLinkSettings (NcsiChannelSetting2) */

    /* Cached NC-SI Get Link Status response fields (NcsiChannelStatus).
     * NOTE: verify these bit meanings against DSP0222 Get Link Status
     * (the "Link Status" response field) for your NC-SI revision before
     * shipping -- this struct only mirrors upstream's grouping, it does not
     * re-derive the wire bit positions from the spec. */
    struct
    {
        uint32_t link_up : 1;
        uint32_t autoneg_hcd : 1;         /* upstream: LinkStatus (highest common denominator) */
        uint32_t autoneg_enabled : 1;
        uint32_t autoneg_complete : 1;
    } link_status;

    uint32_t stat_ncsi_rx;
    uint32_t stat_ncsi_tx;
    uint32_t stat_net_rx;
    uint32_t stat_net_tx;
    uint32_t stat_net_dropped;
} ncsi_channel_state_t;

/* One physical NC-SI channel / MAC-PHY pair. `device` is an opaque handle
 * into your own Ethernet/MDIO driver -- ncsi.c never dereferences it
 * directly, it's only ever passed back into MII_*()/Network_*() below. */
typedef struct NetworkPort
{
    void *device;
    ncsi_channel_state_t state;
} NetworkPort_t;

/* --- HAL functions ncsi.c calls -----------------------------------------
 * Implement all of these against your target's real MAC/PHY/MDIO driver. */

/* (Re-)bring up the port after Enable Channel / NCSI_reload(). reset_phy
 * indicates how hard to reset the PHY (see reload_type_t in types.h). */
void Network_InitPort(NetworkPort_t *port, reload_type_t reset_phy);

/* Return true if the PHY currently reports link up. */
bool Network_isLinkUp(NetworkPort_t *port);

/* Force a link renegotiation/reset (called when Get Link Status finds the
 * link down and wants to kick it). */
void Network_resetLink(NetworkPort_t *port);

/* Program one of the NIC's MAC address filter slots.
 *   mac_high16: top 16 bits of the 48-bit MAC address (bits 47:32)
 *   mac_low32:  bottom 32 bits of the 48-bit MAC address (bits 31:0)
 *   index:      0-based MAC filter slot (NC-SI's MACNumber is 1-based;
 *               ncsi.c already re-indexes to 0-based before calling this)
 *   enable:     whether to enable or disable that filter slot
 */
void Network_SetMACAddr(NetworkPort_t *port, uint16_t mac_high16, uint32_t mac_low32,
                         uint32_t index, bool enable);

/* Forward one pending host-network-bound frame through the NC-SI
 * passthrough path (called from NCSI_handlePassthrough(), which you should
 * poll -- or drive from an RX interrupt -- once per received non-control
 * frame while tx_passthrough_en is set for that channel). Return false if
 * the frame had to be dropped (counted in state.stat_net_dropped). */
bool Network_PassthroughRxPacket(NetworkPort_t *port);

/* Transmit one already-built NC-SI frame ('packet'/'packet_len' as handed
 * to NCSI_TxPacket()/NCSI_TxBePacket() in ncsi.c) out the sideband link.
 * Replaces upstream's APE_PERI TX FIFO write loop -- call your MAC driver's
 * raw-frame transmit here. Must not block indefinitely. */
bool Network_TxFrame(NetworkPort_t *port, const uint8_t *frame, uint32_t frame_len);

/* --- MDIO / PHY register access -----------------------------------------
 * 'device' is the same opaque handle stored in NetworkPort_t::device. */

/* Return the MDIO address of the PHY attached to this device. */
uint8_t MII_getPhy(void *device);

/* Read one MDIO/MII register (0..31). Return -1 on failure so callers can
 * fall back to cached state, matching upstream's error handling. */
int32_t MII_readRegister(void *device, uint8_t phy, uint8_t reg);

/* Reset the PHY via MDIO (e.g. set the BMCR reset bit and wait for it to
 * self-clear). Return true on success. */
bool MII_reset(void *device, uint8_t phy);

/* Standard IEEE 802.3 MII/MDIO register numbers used by getLinkStatusHandler
 * in ncsi.c. Upstream reads a Broadcom-specific "Auxiliary Status Summary"
 * extended register instead (0x19) which isn't portable to another vendor's
 * PHY; this port reads the standard Basic Status Register (BMSR, register 1,
 * bit 2 = Link Status) so it works with any IEEE-compliant PHY. If your PHY
 * exposes a richer vendor register you'd rather use (e.g. for autoneg HCD
 * speed/duplex), read it from inside your MII_readRegister()/board glue and
 * feed the result into NetworkPort_t::state.link_status instead. */
#define REG_MII_BASIC_STATUS (1)
#define MII_BMSR_LINK_STATUS_BIT (1u << 2)

#endif /* NETWORK_H */
