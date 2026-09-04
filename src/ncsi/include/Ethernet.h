////////////////////////////////////////////////////////////////////////////////
///
/// @file       Ethernet.h
///
/// @brief      NC-SI (DSP0222) wire-format structs.
///
/// Ported from meklort/bcm5719-fw, libs/NCSI/include/Ethernet.h
/// (BSD-3-Clause, Copyright (c) 2018 Evan Lojewski -- see
/// third_party/ncsi/LICENSE). Field NAMES and per-command field sets are
/// unchanged from upstream. The field *representation* is not: upstream
/// declares every multi-byte header/payload field as a C bit-field (or, for
/// response payloads, a plain native uint16_t), relying on those bytes
/// landing in on-the-wire (MSB-first) order. Building this port against a
/// standard little-endian GCC target and feeding it real captured DSP0222
/// request frames (see tests/valid_commands.c, tests/test_ncsi.c) showed
/// that assumption doesn't hold: e.g. ControlPacketHeader_t's ChannelID,
/// ControlPacketType and InstanceID bit-fields decoded to the wrong bytes
/// entirely (a real captured Select Package command's ControlPacketType
/// read back as 0xed instead of 0x01, crashing command dispatch on an
/// out-of-range handler-table index) -- C bit-field allocation order within
/// a shared storage unit is implementation-defined, and it does not follow
/// wire order here. Every field wider than one byte is therefore declared
/// as a plain uint8_t[N] in true wire (MSB-first) order and must be read or
/// written through ncsi_rd16()/ncsi_wr16()/ncsi_rd32()/ncsi_wr32() (see
/// types.h) -- never cast directly to an integer type. Byte positions for
/// the command-dispatch header (ChannelID/ControlPacketType/InstanceID/
/// PayloadLength/EtherType) and for AEN Enable's AENControl and Set MAC
/// Address's MAC-address bytes were verified this way against real
/// captures (Set MAC Address decoded to a well-formed MAC address; AEN
/// Enable decoded to a plausible AEN control bitmask). The remaining
/// request/response payload fields (Set Link, Get Capabilities response
/// content, Get Version ID response content, Get Link Status response
/// content) follow the same verified pattern but were not independently
/// checked against a real capture (none exists in valid_commands.c) --
/// re-verify those against DSP0222 and/or a real BMC exchange before
/// shipping.
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

#define PACKET_OFFSET           (12)

/* Shared 28-byte NC-SI control-packet header, byte-verified against real
 * captured request frames (see file header comment). Repeated inline (not
 * shared via composition) in every payload struct below, matching
 * upstream's layout so field access stays flat (frame->controlPacket.Foo,
 * not frame->controlPacket.header.Foo). */
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
    uint8_t  PayloadLength[2];          /* bytes 20-21: reserved(4 bits):PayloadLength(12 bits), BE */ \
    uint8_t  reserved_2[2];             /* bytes 22-23 */ \
    uint8_t  reserved_3[4]                /* bytes 24-27 */

/* Extract the 12-bit Payload Length out of ControlPacketHeader_t::PayloadLength[2]. */
#define NCSI_PAYLOAD_LENGTH(hdr_ptr) (ncsi_rd16((hdr_ptr)->PayloadLength) & 0x0FFFu)
#define NCSI_SET_PAYLOAD_LENGTH(hdr_ptr, len) ncsi_wr16((hdr_ptr)->PayloadLength, (uint16_t)(len))

typedef struct
{
    NCSI_HEADER_FIELDS;
} __attribute__((packed)) ControlPacketHeader_t;
_Static_assert(sizeof(ControlPacketHeader_t) == 16 + PACKET_OFFSET, "sizeof(ControlPacketHeader_t) must be 16.");
#define CONTROL_PACKET_PAYLOAD_OFFSET   (PACKET_OFFSET + 16)

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
 * Field order/position within each struct was fixed by reversing upstream's
 * declared order within each shared storage group (the same correction the
 * header above needed) and splitting >1-byte fields into wire-order
 * uint8_t[N]. Verified against real captures for AENEnable_t and
 * SetMACAddr_t (see file header comment); ClearInitialState_t/
 * SelectPackage_t/SetLink_t's *unused* checksum/padding tail fields don't
 * affect correctness either way since this port never computes checksums
 * (0/0 is DSP0222's valid "not calculated" sentinel, same as upstream). */

typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t headerPadding[2]; /* bytes 28-29 (unused) */
    uint8_t checksumHigh[2];  /* bytes 30-31 (unused, checksum not computed) */
    uint8_t checksumLow[4];   /* bytes 32-35 (unused) */
    uint8_t pad[2];           /* bytes 36-37 (unused) */
} __attribute__((packed)) ClearInitialState_t;

typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t headerPadding[2];            /* bytes 28-29 (unused) */
    uint8_t payload_reserved_0;          /* byte 30 */
    uint8_t HardwareArbitartionDisabled; /* byte 31 */
    uint8_t checksumLow[4];              /* bytes 32-35 (unused) */
    uint8_t pad[2];                      /* bytes 36-37 (unused) */
} __attribute__((packed)) SelectPackage_t;

typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t headerPadding[2];   /* bytes 28-29 (unused) */
    uint8_t payload_reserved_0; /* byte 30 */
    uint8_t AEN_MC_ID;          /* byte 31 */
    uint8_t reserved_1[2];      /* bytes 32-33 (unused) */
    uint8_t AENControl_High[2]; /* bytes 34-35 */
    uint8_t AENControl_Low[2];  /* bytes 36-37 */
    uint8_t pad[2];             /* bytes 38-39 (unused) */
} __attribute__((packed)) AENEnable_t;

typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t headerPadding[2];       /* bytes 28-29 (unused) */
    uint8_t LinkSettings_High[2];   /* bytes 30-31 */
    uint8_t LinkSettings_Low[2];    /* bytes 32-33 */
    uint8_t OEMLinkSettings_High[2];/* bytes 34-35 */
    uint8_t OEMLinkSettings_Low[2]; /* bytes 36-37 */
    uint8_t pad[2];                 /* bytes 38-39 (unused) */
} __attribute__((packed)) SetLink_t;

typedef struct {
    NCSI_HEADER_FIELDS;
    uint8_t headerPadding[2]; /* bytes 28-29 (unused) */
    uint8_t MAC54[2];         /* bytes 30-31: top 16 bits of the 48-bit MAC */
    uint8_t MAC32[2];         /* bytes 32-33: middle 16 bits */
    uint8_t MAC10[2];         /* bytes 34-35: bottom 16 bits */
    uint8_t MACNumber;        /* byte 36: 1-based NC-SI MAC filter slot */
    uint8_t MACInfo;          /* byte 37: bit7 Enable, bits6:3 Rsvd, bits2:0 AT */
    uint8_t pad[2];           /* bytes 38-39 (unused) */
} __attribute__((packed)) SetMACAddr_t;

#define SETMAC_AT(info)     ((info) & 0x07u)
#define SETMAC_RSVD(info)   (((info) >> 3) & 0x0Fu)
#define SETMAC_ENABLE(info) (((info) >> 7) & 0x01u)

/* --- Response payload structs --------------------------------------------
 * Every field here past the shared header is a plain (non-bit-field)
 * member in upstream too, so declaration order was already unambiguous;
 * only the type changed (native uint16_t -> uint8_t[2]) since a plain
 * multi-byte integer still stores its bytes in host (little-endian)
 * order, not wire order. Not independently re-verified against a real
 * captured response (no response captures exist in valid_commands.c) --
 * see file header comment. */

typedef struct
{
    NCSI_HEADER_FIELDS;
    uint8_t ResponseCode[2];
    uint8_t reserved_4[2];
    uint8_t Checksum_High[2];
    uint8_t ReasonCode[2];
    uint8_t reserved_5[2];
    uint8_t Checksum_Low[2];
} __attribute__((packed)) ResponsePacketHeader_t;

typedef struct
{
    NCSI_HEADER_FIELDS;
    uint8_t ResponseCode[2];
    uint8_t reserved_4[2];
    uint8_t LinkStatus_High[2];
    uint8_t ReasonCode[2];
    uint8_t OtherIndications_High[2];
    uint8_t LinkStatus_Low[2];
    uint8_t OEMLinkStatus_High[2];
    uint8_t OtherIndications_Low[2];
    uint8_t Checksum_High[2];
    uint8_t OEMLinkStatus_Low[2];
    uint8_t pad[2];
    uint8_t Checksum_Low[2];
} __attribute__((packed)) LinkStatusResponsePacketHeader_t;

typedef struct
{
    NCSI_HEADER_FIELDS;
    uint8_t ResponseCode[2];
    uint8_t reserved_4[2];
    uint8_t Capabilities_High[2];
    uint8_t ReasonCode[2];
    uint8_t BroadcastCapabilities_High[2];
    uint8_t Capabilities_Low[2];
    uint8_t MilticastCapabilities_High[2];
    uint8_t BroadcastCapabilities_Low[2];
    uint8_t BufferingCapabilities_High[2];
    uint8_t MilticastCapabilities_Low[2];
    uint8_t AENControlSupport_High[2];
    uint8_t BufferingCapabilities_Low[2];

    uint8_t  MixedFilterCount;
    uint8_t  VLANFilterCount;
    uint8_t  AENControlSupport_Low[2];

    uint8_t reserved_5[2];
    uint8_t  UnicastFilterCount;
    uint8_t  MulticastFilterCount;

    uint8_t Checksum_High[2];
    uint8_t  ChannelCount;
    uint8_t  VLANModeSupport;

    uint8_t pad[2];
    uint8_t Checksum_Low[2];
} __attribute__((packed)) CapabilitiesResponsePacket_t;

typedef struct
{
    NCSI_HEADER_FIELDS;
    uint8_t ResponseCode[2];
    uint8_t reserved_4[2];

    uint8_t  NCSIMinor;
    uint8_t  NCSIMajor;
    uint8_t ReasonCode[2];

    uint8_t reserved_5[2];
    uint8_t  NCSIAlpha1;
    uint8_t  NCSIUpdate;

    uint8_t name_10;
    uint8_t name_11;

    uint8_t NCSIAlpha2;
    uint8_t reserved_6;

    uint8_t name_6;
    uint8_t name_7;
    uint8_t name_8;
    uint8_t name_9;

    uint8_t name_2;
    uint8_t name_3;
    uint8_t name_4;
    uint8_t name_5;

    uint8_t FWVersion_High[2];
    uint8_t name_0;
    uint8_t name_1;

    uint8_t PCIVendor[2];
    uint8_t FWVersion_Low[2];

    uint8_t PCISubsystemVendor[2];
    uint8_t PCIDevice[2];

    uint8_t ManufacturerID_High[2];
    uint8_t PCISubsystemDevice[2];

    uint8_t Checksum_High[2];
    uint8_t ManufacturerID_Low[2];

    uint8_t pad[2];
    uint8_t Checksum_Low[2];
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

    AENEnable_t AENEnable;

    SetLink_t setLink;

    SetMACAddr_t setMACAddr;

    /* Response Packets */
    ResponsePacketHeader_t  responsePacket;

    LinkStatusResponsePacketHeader_t    linkStatusResponse;

    CapabilitiesResponsePacket_t capabilities;

    VersionResponsePacket_t version;
} __attribute__((packed)) NetworkFrame_t;

// Ethernet frame must be at least 64 bytes.
_Static_assert(sizeof(NetworkFrame_t) >= ETHERNET_FRAME_MIN, "NetworkFrame_t must be at least ETHERNET_FRAME_MIN bytes");

#endif /* ETHERNET_H */
