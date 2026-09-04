////////////////////////////////////////////////////////////////////////////////
///
/// @file       types.h
///
/// @brief      Portable base types/macros for the ported NC-SI stack.
///
/// Ported from meklort/bcm5719-fw (libs/NCSI), BSD-3-Clause. This header
/// replaces the original BCM5719-firmware-internal types.h with a
/// dependency-free version suitable for a bare-metal / minimal-RTOS
/// RISC-V32 target: only <stdint.h>/<stdbool.h>/<stddef.h> are required.
///
////////////////////////////////////////////////////////////////////////////////

#ifndef NCSI_PORT_TYPES_H
#define NCSI_PORT_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef __LITTLE_ENDIAN__
/* ncsi.c / Ethernet.h bitfield layouts are only defined for little-endian
 * targets (as in upstream). RISC-V32 is little-endian by default; if this
 * target is configured big-endian, the ControlPacketHeader_t etc. bitfield
 * layouts in Ethernet.h need a big-endian variant before this will work. */
#define __LITTLE_ENDIAN__ 1
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef DIVIDE_RND_UP
#define DIVIDE_RND_UP(n, d) (((n) + (d) - 1) / (d))
#endif

#ifndef ARRAY_ELEMENTS
#define ARRAY_ELEMENTS(a) (sizeof(a) / sizeof((a)[0]))
#endif

/* Volatile qualifier for hardware-backed state. The original SHM-register
 * based state needed this; plain RAM struct fields generally don't, but the
 * macro is kept so ncsi.c doesn't need to change if a future port moves
 * state back into a hardware-shared region. */
#ifndef VOLATILE
#define VOLATILE volatile
#endif

/* How aggressively NCSI_reload()/Network_InitPort() should re-touch the PHY.
 * Renamed from upstream's unexplained AS_NEEDED/FORCE pair to make the two
 * options self-describing. */
typedef enum
{
    NCSI_RELOAD_AS_NEEDED = 0, /* Only reset the PHY if it looks necessary */
    NCSI_RELOAD_FORCE     = 1, /* Always reset the PHY */
} reload_type_t;

/* Portable byte-swap used by NCSI_TxBePacket(). On a little-endian host,
 * be32toh() is defined as an unconditional 4-byte reversal; avoids
 * depending on <endian.h> (not available on all bare-metal toolchains). */
static inline uint32_t ncsi_be32toh(uint32_t v)
{
    return ((v & 0x000000FFu) << 24) | ((v & 0x0000FF00u) << 8) |
           ((v & 0x00FF0000u) >> 8) | ((v & 0xFF000000u) >> 24);
}
#define be32toh(x) ncsi_be32toh(x)

/* --- Big-endian wire-field helpers --------------------------------------
 * NC-SI (DSP0222), like the Ethernet header it rides on, transmits every
 * multi-byte field MSB-first. C bit-fields are the wrong tool for decoding
 * that: bit-field allocation order *within* a shared storage unit is
 * implementation-defined, and a plain uint16_t/uint32_t struct member
 * holding a "natural" constant stores its bytes in the *host's* endianness,
 * not the wire's. Both were verified to silently produce wrong bytes when
 * this port was built with a standard GCC-target little-endian ABI (see
 * Ethernet.h's header comment for how this was found). Every multi-byte
 * field in Ethernet.h is therefore a plain uint8_t[N] in wire (MSB-first)
 * order; read/write it through these helpers instead of casting it to an
 * integer directly. */
static inline uint16_t ncsi_rd16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

static inline void ncsi_wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static inline uint32_t ncsi_rd32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static inline void ncsi_wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

/* --- Optional DSP0222 Checksum computation -------------------------------
 * DSP0222's Checksum field is normally left at 0/0 ("not calculated" --
 * always valid per clause 8.2.2.3, and what this port sends by default).
 * This helper computes a real one instead, for callers that want it (see
 * NCSI_COMPUTE_CHECKSUM_DEFAULT / NCSI_SetComputeChecksum() in NCSI.h).
 *
 * Algorithm per DSP0222 v1.2.1 clause 8.2.2.3 (verified against the actual
 * spec text, not guessed): "the checksum compensation shall be computed as
 * the 2's complement of the checksum, which shall be computed as the
 * 32-bit unsigned sum of the NC-SI packet header and NC-SI packet payload
 * interpreted as a series of 16-bit unsigned integer values." I.e.: sum
 * the region from the NC-SI header start (ManagmentControllerID --
 * *excluding* the 12-byte Ethernet DA/SA and 2-byte EtherType, which
 * belong to the separate Ethernet framing clause 8.1, not "the NC-SI
 * packet header") through the byte immediately before the Checksum field,
 * as consecutive big-endian *16-bit* words (a trailing odd byte is
 * zero-padded), into a 32-bit accumulator; the stored value is the 2's
 * complement (negation) of that sum. A receiver verifies by summing the
 * same region *including* the checksum field itself; a correctly computed
 * checksum makes that total 0 (verified this way in tests/test_ncsi.c,
 * across all response struct types this port sends).
 *
 * An earlier version of this file assumed the DMTF-unpublished-to-this-
 * project 32-bit-word / non-contiguous-Checksum-field layout used by
 * bcm5719-fw; once the real spec text became available it turned out
 * Checksum is always one contiguous 4-byte field (see Ethernet.h's header
 * comment), which is what this function assumes.
 *
 * `packet`           base of the struct (e.g. &gResponseFrame.responsePacket)
 * `region_start`     byte offset of ManagmentControllerID within `packet`
 *                     (always 14 for every NC-SI header struct here)
 * `region_len`       number of bytes from region_start up to (not
 *                     including) the checksum field
 * `checksum_offset`  byte offset of the 4-byte checksum field within
 *                     `packet` -- always region_start + region_len */
static inline void ncsi_compute_checksum(uint8_t *packet, uint32_t region_start, uint32_t region_len,
                                          uint32_t checksum_offset)
{
    uint32_t end = region_start + region_len;
    uint32_t sum = 0;
    uint32_t i;

    for (i = region_start; i + 2 <= end; i += 2)
    {
        sum += ncsi_rd16(&packet[i]);
    }
    if (i < end)
    {
        /* Odd trailing byte: treat as the high byte of a zero-padded
         * 16-bit word (matches how the final partial payload word is
         * zero-padded to a 32-bit boundary per clause 8.2.2.2). */
        sum += (uint32_t)packet[i] << 8;
    }

    ncsi_wr32(&packet[checksum_offset], (uint32_t)(0u - sum)); /* 2's-complement negation */
}

#endif /* NCSI_PORT_TYPES_H */
