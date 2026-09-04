////////////////////////////////////////////////////////////////////////////////
///
/// @file       Ethernet.h
///
/// @brief      NC-SI (DSP0222) wire-format structs.
///
/// Ported from meklort/bcm5719-fw, libs/NCSI/include/Ethernet.h
/// (BSD-3-Clause, Copyright (c) 2018 Evan Lojewski -- see
/// third_party/ncsi/LICENSE), but the field layouts below are a **rewrite
/// against the actual DSP0222 v1.2.1 specification** (DMTF DSP0222,
/// provided directly for this port), not a port of upstream's struct
/// layouts. Field NAMES were kept close to upstream/DSP0222 terminology
/// where practical; byte positions are not upstream's.
///
/// Why this rewrite happened: this port originally inherited upstream's
/// header/payload byte layout (bit-fields fixed to true wire order in an
/// earlier pass -- see git history / CLAUDE.md decision #2). Once the real
/// DSP0222 spec became available, cross-checking against it (clause 8.2.1
/// Table 10, and clause 8.4's per-command tables) found the inherited
/// layout had THREE separate real bugs, not just the earlier-fixed
/// bit-field ordering:
///
///   1. The DSP0222 NC-SI Control Packet header is 16 bytes (Table 10:
///      bytes 0-15 relative to the start of the NC-SI packet, i.e.
///      immediately after the 14-byte Ethernet header) -- the inherited
///      layout's header was only 14 bytes (6 bytes of trailing Reserved
///      short), so every payload field after it was offset by 2 bytes from
///      where DSP0222 actually puts it.
///   2. Response payloads had spurious 2-byte "reserved" gaps between
///      ResponseCode/ReasonCode and other fields that DSP0222 does not
///      define -- e.g. Table 24/25 (Clear Initial State) and every other
///      response's Table put ResponseCode and ReasonCode *immediately*
///      adjacent (bytes 16-19 as one 4-byte group), with no gap.
///   3. Checksum is defined (clause 8.2.2.3, and every per-command Table in
///      8.4) as ONE contiguous 4-byte field placed right after the payload
///      (and its padding to a 32-bit boundary) -- never split into two
///      non-adjacent 2-byte halves. The inherited layout had Checksum_High
///      sitting mid-payload (before ReasonCode) and Checksum_Low near the
///      very end, which does not match any DSP0222 table.
///
/// These were caught by cross-checking specific fields against real
/// captured request frames (tests/valid_commands.c) using the spec's exact
/// byte tables rather than trusting the inherited layout further -- see
/// each struct's comment below for which fields were re-verified this way
/// (AEN Enable's AEN_MC_ID and AEN Control, Set MAC Address's MAC bytes).
///
/// Every field wider than one byte is a plain uint8_t[N] in true wire
/// (MSB-first) order and must be read or written through
/// ncsi_rd16()/ncsi_wr16()/ncsi_rd32()/ncsi_wr32() (see types.h) -- never
/// cast directly to an integer type; C bit-fields and native
/// uint16_t/uint32_t members are both the wrong tool here (bit-field
/// allocation order within a shared storage unit is implementation-defined
/// in C, and a plain multi-byte integer stores its bytes in the host's
/// endianness, not the wire's -- both were verified to silently produce
/// wrong bytes earlier in this port; see git history).
///
////////////////////////////////////////////////////////////////////////////////

#ifndef ETHERNET_H
#define ETHERNET_H

#include <types.h>

#define ETHERNET_FRAME_MIN      (64 - 4) /* Hardware automatically adds FCS (last 4 bytes) */

typedef struct
{
    uint8_t  DestinationAddress[6];
    uint8_t  SourceAddress[6];
    uint8_t  EtherType[2];  /* bytes 12-13, wire/BE order: {0x88, 0xF8} for NC-SI */
    uint8_t  reserved[2];   /* bytes 14-15 */
} __attribute__((packed)) EthernetHeader_t;
_Static_assert(sizeof(EthernetHeader_t) == 16, "sizeof(EthernetHeader_t) must be 16.");
#define ETHER_TYPE_NCSI_HI 0x88
#define ETHER_TYPE_NCSI_LO 0xf8
#define ETHER_TYPE_NCSI 0x88f8 /* logical value; compare via ncsi_rd16(frame->header.EtherType) */
#define ETHERNET_HEADER_OFFSET      0

#define PACKET_OFFSET           (12) /* Ethernet DA+SA; EtherType is part of NCSI_HEADER_FIELDS below */

/* The 16-byte NC-SI Control Packet header (DSP0222 clause 8.2.1, Table 10),
 * repeated inline (not shared via composition) in every payload struct
 * below, matching upstream's flat-field-access style (frame->controlPacket.Foo,
 * not frame->controlPacket.header.Foo). Byte positions verified against
 * Table 10 directly, and cross-checked against real captured request
 * frames for ChannelID/ControlPacketType/InstanceID/PayloadLength (see
 * tests/valid_commands.c, tests/test_ncsi.c). */
#define NCSI_HEADER_FIELDS \
    uint8_t  DestinationAddress[6]; \
    uint8_t  SourceAddress[6]; \
    uint8_t  EtherType[2];              /* bytes 12-13 */ \
    uint8_t  ManagmentControllerID;     /* byte 14. Should be 0 */ \
    uint8_t  HeaderRevision;            /* byte 15. Should be 1 */ \
    uint8_t  reserved_0;                /* byte 16 */ \
    uint8_t  InstanceID;                /* byte 17 */ \
    uint8_t  ControlPacketType;         /* byte 18 */ \
    uint8_t  ChannelID;                 /* byte 19 */ \
    uint8_t  PayloadLength[2];          /* bytes 20-21: Flags(4 bits):PayloadLength(12 bits), BE */ \
    uint8_t  reserved_2[8]              /* bytes 22-29 (Table 10's two 4-byte Reserved rows) */

/* Extract the 12-bit Payload Length out of ControlPacketHeader_t::PayloadLength[2]. */
#define NCSI_PAYLOAD_LENGTH(hdr_ptr) (ncsi_rd16((hdr_ptr)->PayloadLength) & 0x0FFFu)
#define NCSI_SET_PAYLOAD_LENGTH(hdr_ptr, len) ncsi_wr16((hdr_ptr)->PayloadLength, (uint16_t)(len))

typedef struct
{
    NCSI_HEADER_FIELDS;
} __attribute__((packed)) ControlPacketHeader_t;
_Static_assert(sizeof(ControlPacketHeader_t) == 30, "sizeof(ControlPacketHeader_t) must be 30 (16-byte DSP0222 header + 14-byte Ethernet header - 12, i.e. bytes 12..29 inclusive).");
#define CONTROL_PACKET_PAYLOAD_OFFSET   (30)

#define CONTROL_PACKET_TYPE_RESPONSE                    (0x80)
#define CONTROL_PACKET_TYPE_CLEAR_INITIAL_STATE         (0x00)
#define CONTROL_PACKET_TYPE_SELECT_PACKAGE              (0x01)
#define CONTROL_PACKET_TYPE_DESELECT_PACKAGE            (0x02)
#define CONTROL_PACKET_TYPE_ENABLE_CHANNEL              (0x03)
#define CONTROL_PACKET_TYPE_DISABLE_CHANNEL             (0x04)
#define CONTROL_PACKET_TYPE_RESET_CHANNEL               (0x05)
#define CONTROL_PACKET_TYPE_ENABLE_CHANNEL_NETWORK_TX   (0x06)
#define CONTROL_PACKET_TYPE_DISABLE_CHANNEL_NETWORK_TX  (0x07)
#define CONTROL_PACKET_TYPE_AEN_ENABLE                  (0x08)
#define CONTROL_PACKET_TYPE_SET_LINK                    (0x09)
#define CONTROL_PACKET_TYPE_GET_LINK_STATUS             (0x0A)
#define CONTROL_PACKET_TYPE_SET_VLAN_FILTER             (0x0B)
#define CONTROL_PACKET_TYPE_ENABLE_VLAN                 (0x0C)
#define CONTROL_PACKET_TYPE_DISABLE_VLAN                (0x0D)
#define CONTROL_PACKET_TYPE_SET_MAC_ADDRESS             (0x0E)
// 0x0F
#define CONTROL_PACKET_TYPE_ENABLE_BROADCAST_FILTERING  (0x10)
#define CONTROL_PACKET_TYPE_DISABLE_BROADCAST_FILTERING (0x11)
#define CONTROL_PACKET_TYPE_ENABLE_GLOBAL_MULTICAST_FILTERING  (0x12)
#define CONTROL_PACKET_TYPE_DISABLE_GLOBAL_MULTICAST_FILTERING (0x13)
#define CONTROL_PACKET_TYPE_SET_NCSI_FLOW_CONTROL       (0x14)
#define CONTROL_PACKET_TYPE_GET_VERSION_ID              (0x15)
#define CONTROL_PACKET_TYPE_GET_CAPABILITIES            (0x16)
#define CONTROL_PACKET_TYPE_GET_PARAMETERS              (0x17)
#define CONTROL_PACKET_TYPE_GET_CONTROLLER_PACKET_STATS (0x18)
#define CONTROL_PACKET_TYPE_GET_NCSI_STATS              (0x19)
#define CONTROL_PACKET_TYPE_GET_NCSI_PASSTHRU_STATS     (0x1A)
#define CONTROL_PACKET_OEM_COMMAND                      (0x50)

/* --- Request payload structs --------------------------------------------
 * Byte positions taken directly from DSP0222 clause 8.4's per-command
 * tables (cited per struct below), not reverse-engineered. Checksum is
 * always one contiguous 4-byte field (unused/not verified further -- 0/0
 * is DSP0222's valid "not calculated" sentinel, see clause 8.2.2.3, and
 * this port's optional real-checksum path computes it fresh rather than
 * reading these unused command-side checksum bytes). */

/* Table 24 -- Clear Initial State command: header + Checksum only, no
 * payload data. Deselect Package (Table 29), Enable Channel (Table 31),
 * Get Link Status (Table 49), Get Version ID (Table 86), Get Capabilities
 * (Table 88) and Get Parameters (Table 92) share this exact shape. */
typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t checksum[4]; /* bytes 30-33 (unused/not computed on the request side) */
} __attribute__((packed)) ClearInitialState_t;

/* Table 26 -- Select Package command. */
typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t reserved_1[3];               /* bytes 30-32 */
    uint8_t HardwareArbitartionDisabled; /* byte 33: Features Control byte (Table 27) -- bit0 = disable HW arbitration, bit1 = delayed response enable, bits7:2 reserved */
    uint8_t checksum[4];                 /* bytes 34-37 */
} __attribute__((packed)) SelectPackage_t;

/* Table 33 -- Disable Channel command. */
typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t reserved_1[3]; /* bytes 30-32 */
    uint8_t ALD;           /* byte 33: bit0 = Allow Link Down, bits7:1 reserved */
    uint8_t checksum[4];   /* bytes 34-37 */
} __attribute__((packed)) DisableChannel_t;

/* Table 41 -- AEN Enable command. Verified against a real captured request
 * (tests/valid_commands.c aen_enable[]): decodes to AEN_MC_ID=0,
 * AENControl=0x00000007 (plausible "enable Link Status Change + Config
 * Required + Host NC Driver Status Change AEN" per Table 42) at these
 * exact byte positions. */
typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t reserved_1[3];  /* bytes 30-32 */
    uint8_t AEN_MC_ID;      /* byte 33 */
    uint8_t AENControl[4];  /* bytes 34-37 (Table 42 bit definitions) */
    uint8_t checksum[4];    /* bytes 38-41 */
} __attribute__((packed)) AENEnable_t;

/* Table 44 -- Set Link command. LinkSettings/OEMLinkSettings are each one
 * contiguous 32-bit field (Table 45/46 bit definitions), not split. */
typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t LinkSettings[4];    /* bytes 30-33 */
    uint8_t OEMLinkSettings[4]; /* bytes 34-37 */
    uint8_t checksum[4];        /* bytes 38-41 */
} __attribute__((packed)) SetLink_t;

/* Table 56 -- Set VLAN Filter command. */
typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t reserved_1;       /* byte 30 */
    uint8_t UserPriorityCFI;  /* byte 31: bits7:5 User Priority, bit4 CFI, bits3:0 reserved */
    uint8_t VLANID[2];        /* bytes 32-33: 12-bit VLAN ID in the low 12 bits, BE */
    uint8_t vlan_reserved_1;  /* byte 34 -- named apart from NCSI_HEADER_FIELDS's own reserved_2[8], which this struct already inherits */
    uint8_t FilterSelector;   /* byte 35 */
    uint8_t vlan_reserved_2;  /* byte 36 */
    uint8_t Enable;           /* byte 37: bit0 = Enable (E), bits7:1 reserved */
    uint8_t checksum[4];      /* bytes 38-41 */
} __attribute__((packed)) SetVLANFilter_t;

/* Table 61 -- Enable VLAN command. */
typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t reserved_1[3]; /* bytes 30-32 */
    uint8_t Mode;          /* byte 33: Table 62 mode numbers */
    uint8_t checksum[4];   /* bytes 34-37 */
} __attribute__((packed)) EnableVLAN_t;

/* Table 66 -- Set MAC Address command. Verified against a real captured
 * request (tests/valid_commands.c set_mac_addr[]): decodes to MAC
 * 2c:09:4d:00:01:4a, MACNumber=1 (filter slot 1, 1-based per spec), at
 * these exact byte positions. */
typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t MACAddress[6]; /* bytes 30-35, wire order byte5..byte0 (MACAddress[0]=byte5=MSB) */
    uint8_t MACNumber;     /* byte 36: 1-based MAC address filter slot (Table 67) */
    uint8_t MACInfo;       /* byte 37: bits2:0 AT (Table 68), bits6:3 reserved, bit7 E (Table 69) */
    uint8_t checksum[4];   /* bytes 38-41 */
} __attribute__((packed)) SetMACAddr_t;

#define SETMAC_AT(info)     ((info) & 0x07u)
#define SETMAC_RSVD(info)   (((info) >> 3) & 0x0Fu)
#define SETMAC_ENABLE(info) (((info) >> 7) & 0x01u)

/* Table 72 -- Enable Broadcast Filter command. One contiguous 32-bit field
 * (Table 73 bit definitions), not split. */
typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t BroadcastPacketFilterSettings[4]; /* bytes 30-33 */
    uint8_t checksum[4];                      /* bytes 34-37 */
} __attribute__((packed)) EnableBroadcastFilter_t;

/* --- Response payload structs --------------------------------------------
 * Byte positions taken directly from DSP0222 clause 8.4's per-command
 * response tables. ResponseCode/ReasonCode are always the first 4 payload
 * bytes (clause 8.2.4) with no gap between them, and Checksum is always
 * one contiguous 4-byte field placed immediately after the last defined
 * payload field -- neither matches the layout this port used before being
 * checked against the real spec (see this file's header comment). */

/* Table 25 (Clear Initial State) and the identical shape used by Select
 * Package (28), Deselect Package (30), Enable Channel (32), Disable
 * Channel (34), Reset Channel (36), Enable/Disable Channel Network TX
 * (38/40), AEN Enable (43), Set Link (47), Set VLAN Filter (59), Enable/
 * Disable VLAN (63/65), Set MAC Address (70), Enable/Disable Broadcast
 * Filter (74/76) responses: just the generic ResponseCode/ReasonCode
 * payload, no command-specific content. */
typedef struct
{
    NCSI_HEADER_FIELDS;
    uint8_t ResponseCode[2]; /* bytes 30-31 */
    uint8_t ReasonCode[2];   /* bytes 32-33 */
    uint8_t checksum[4];     /* bytes 34-37 */
} __attribute__((packed)) ResponsePacketHeader_t;

/* Table 50 -- Get Link Status response. LinkStatus/OtherIndications/
 * OEMLinkStatus are each one contiguous 32-bit field (Table 51 bit
 * definitions for LinkStatus), not split. */
typedef struct
{
    NCSI_HEADER_FIELDS;
    uint8_t ResponseCode[2];      /* bytes 30-31 */
    uint8_t ReasonCode[2];        /* bytes 32-33 */
    uint8_t LinkStatus[4];        /* bytes 34-37 */
    uint8_t OtherIndications[4];  /* bytes 38-41 */
    uint8_t OEMLinkStatus[4];     /* bytes 42-45 */
    uint8_t checksum[4];          /* bytes 46-49 */
} __attribute__((packed)) LinkStatusResponsePacketHeader_t;

/* Table 89 -- Get Capabilities response. */
typedef struct
{
    NCSI_HEADER_FIELDS;
    uint8_t ResponseCode[2];                    /* bytes 30-31 */
    uint8_t ReasonCode[2];                      /* bytes 32-33 */
    uint8_t CapabilitiesFlags[4];                /* bytes 34-37 (Table 90) */
    uint8_t BroadcastPacketFilterCapabilities[4]; /* bytes 38-41 (Table 73 bit defs) */
    uint8_t MulticastPacketFilterCapabilities[4]; /* bytes 42-45 (Table 78 bit defs) */
    uint8_t BufferingCapability[4];              /* bytes 46-49 */
    uint8_t AENControlSupport[4];                /* bytes 50-53 (Table 42 bit defs) */
    uint8_t VLANFilterCount;                     /* byte 54 */
    uint8_t MixedFilterCount;                    /* byte 55 */
    uint8_t MulticastFilterCount;                /* byte 56 */
    uint8_t UnicastFilterCount;                  /* byte 57 */
    uint8_t reserved_1[2];                       /* bytes 58-59 */
    uint8_t VLANModeSupport;                     /* byte 60 (Table 91) */
    uint8_t ChannelCount;                        /* byte 61 */
    uint8_t checksum[4];                         /* bytes 62-65 */
} __attribute__((packed)) CapabilitiesResponsePacket_t;

/* Table 87 -- Get Version ID response. NCSI Major/Minor/Update are
 * BCD-encoded per clause 8.4.44.1 (e.g. 0xF1/0xF2/0xF0 for "1.2.0"
 * compatibility, the value DSP0222 v1.2.1 mandates -- see
 * ncsi.c's gVersionFrame initializer). Firmware Name String is left-
 * justified, most-significant byte first (clause 8.4.44.2); PCI ID fields
 * default to 0x0000 when unused (clause 8.4.44.4) and Manufacturer ID
 * defaults to 0xFFFFFFFF when unused (clause 8.4.44.5) -- both different
 * defaults than this port originally used, fixed alongside this rewrite. */
typedef struct
{
    NCSI_HEADER_FIELDS;
    uint8_t ResponseCode[2];       /* bytes 30-31 */
    uint8_t ReasonCode[2];         /* bytes 32-33 */
    uint8_t NCSIMajor;             /* byte 34 */
    uint8_t NCSIMinor;             /* byte 35 */
    uint8_t NCSIUpdate;            /* byte 36 */
    uint8_t NCSIAlpha1;            /* byte 37 */
    uint8_t reserved_1[3];         /* bytes 38-40 */
    uint8_t NCSIAlpha2;            /* byte 41 */
    uint8_t FirmwareNameString[12];/* bytes 42-53, left-justified, byte42=leftmost char */
    uint8_t FirmwareVersion[4];    /* bytes 54-57, MS byte first */
    uint8_t PCIDID[2];             /* bytes 58-59 */
    uint8_t PCIVID[2];             /* bytes 60-61 */
    uint8_t PCISSID[2];            /* bytes 62-63 */
    uint8_t PCISVID[2];            /* bytes 64-65 */
    uint8_t ManufacturerID[4];     /* bytes 66-69, IANA Enterprise Number */
    uint8_t checksum[4];           /* bytes 70-73 */
} __attribute__((packed)) VersionResponsePacket_t;

#define CAPABILITIES_HARDWARE_ABSTRACTION   (1 << 0)
#define CAPABILITIES_OS_PRESENCE            (1 << 1)
#define CAPABILITIES_FLOW_CONTROL_RX        (1 << 2)
#define CAPABILITIES_FLOW_CONTROL_TX        (1 << 3)
#define CAPABILITIES_MULTICAST              (1 << 4)

typedef union {
    EthernetHeader_t    header;

    /* Control Packets */
    ControlPacketHeader_t controlPacket;

    ClearInitialState_t clearInitialState;

    SelectPackage_t selectPackage;

    DisableChannel_t disableChannel;

    AENEnable_t AENEnable;

    SetLink_t setLink;

    SetVLANFilter_t setVLANFilter;

    EnableVLAN_t enableVLAN;

    SetMACAddr_t setMACAddr;

    EnableBroadcastFilter_t enableBroadcastFilter;

    /* Response Packets */
    ResponsePacketHeader_t  responsePacket;

    LinkStatusResponsePacketHeader_t    linkStatusResponse;

    CapabilitiesResponsePacket_t capabilities;

    VersionResponsePacket_t version;
} __attribute__((packed)) NetworkFrame_t;

// Ethernet frame must be at least 64 bytes.
_Static_assert(sizeof(NetworkFrame_t) >= ETHERNET_FRAME_MIN, "NetworkFrame_t must be at least ETHERNET_FRAME_MIN bytes");

#endif /* ETHERNET_H */
