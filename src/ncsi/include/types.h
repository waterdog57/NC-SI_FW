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

#endif /* NCSI_PORT_TYPES_H */
