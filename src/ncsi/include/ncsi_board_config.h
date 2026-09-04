////////////////////////////////////////////////////////////////////////////////
///
/// @file       ncsi_board_config.h
///
/// @brief      ALL board-specific configuration for this NC-SI port lives
///             here: values you fill in directly (compile-time constants)
///             and values read at runtime (function interfaces, implemented
///             in ncsi_board_config.c). This is the one file to edit when
///             bringing this stack up on a new board.
///
/// New code written for this port (not from bcm5719-fw).
///
////////////////////////////////////////////////////////////////////////////////

#ifndef NCSI_BOARD_CONFIG_H
#define NCSI_BOARD_CONFIG_H

#include <types.h>

/* ===========================================================================
 * Runtime-read identity -- implemented in ncsi_board_config.c
 * ==========================================================================*/

/* DSP0222 Package ID (0-7) this firmware image answers to. Read from GPIO
 * strapping: both ICs of one OCP NIC 3.0 card pair are wired to the same
 * package-ID strap signal(s) (they're one package, two channels), so this
 * must return the same value on both chips. Only needs to be unique across
 * *different* card pairs / packages sharing one NC-SI bus. Called once
 * during NCSI_init(). */
uint8_t NCSI_BoardConfig_ReadPackageID(void);

/* DSP0222 Channel ID (0 or 1) this firmware image answers to. On this
 * OCP NIC 3.0 design, GPIO20 is strapped differently on each of the two
 * ICs of a pair -- one reads 0, the other reads 1 -- so each chip's own
 * firmware image reports a different channel here even though it's the
 * exact same binary. Called once during NCSI_init(). */
uint8_t NCSI_BoardConfig_ReadChannelID(void);

/* ===========================================================================
 * Compile-time configuration -- edit these values directly for your board.
 * ==========================================================================*/

/* Total number of NC-SI channels behind this *package* (both ICs combined),
 * reported in the Get Capabilities response's Channel Count field. This is
 * package-wide topology info the BMC needs regardless of which of the two
 * chips answers -- it is NOT how many channels this firmware instance
 * itself implements (that's always exactly 1: this chip's own channel,
 * identified by NCSI_BoardConfig_ReadChannelID() above). */
#ifndef NCSI_PACKAGE_CHANNEL_COUNT
#define NCSI_PACKAGE_CHANNEL_COUNT 2
#endif

/* Get Version ID response fields. */
#ifndef NCSI_FW_VERSION_MAJOR
#define NCSI_FW_VERSION_MAJOR 0
#endif
#ifndef NCSI_FW_VERSION_MINOR
#define NCSI_FW_VERSION_MINOR 1
#endif
#ifndef NCSI_FW_VERSION_PATCH
#define NCSI_FW_VERSION_PATCH 0
#endif

/* DMTF-assigned Manufacturer ID (IANA Enterprise Number), reported as a
 * 32-bit field. Per DSP0222 clause 8.4.44.5, "if the field is unused, the
 * value shall be set to 0xFFFFFFFF" -- 0 is not that sentinel and is not a
 * valid real PEN either, so it's wrong either way; set this to your
 * organization's actual PEN, or leave the sentinel if you don't have one. */
#ifndef NCSI_MANUFACTURER_ID
#define NCSI_MANUFACTURER_ID 0xFFFFFFFF
#endif

/* PCI identity reported in Get Version ID. This device is generally not a
 * PCI function on its own (NC-SI just rides the sideband of a NIC that
 * might be), so these default to DSP0222's "not used" sentinel -- clause
 * 8.4.44.4: "If this field is not used, the values shall be set to all
 * zeros (0x0000)". (Note this sentinel is 0x0000, NOT 0xFFFFFFFF like
 * Manufacturer ID above -- the two fields use different "unused" values;
 * this was gotten backwards, as 0xFFFF for both, in an earlier version of
 * this port before the real DSP0222 spec text was available.) Override if
 * your board has real PCI config space to report. */
#ifndef NCSI_PCI_VENDOR_ID
#define NCSI_PCI_VENDOR_ID 0x0000
#endif
#ifndef NCSI_PCI_DEVICE_ID
#define NCSI_PCI_DEVICE_ID 0x0000
#endif
#ifndef NCSI_PCI_SUBSYSTEM_VENDOR_ID
#define NCSI_PCI_SUBSYSTEM_VENDOR_ID 0x0000
#endif
#ifndef NCSI_PCI_SUBSYSTEM_DEVICE_ID
#define NCSI_PCI_SUBSYSTEM_DEVICE_ID 0x0000
#endif

/* Whether outgoing responses get a real DSP0222 Checksum computed
 * (NCSI_SetComputeChecksum()/NCSI_GetComputeChecksum() in NCSI.h can also
 * flip this at runtime -- this is just the boot-time default). Default
 * OFF: 0/0 ("not calculated") is DSP0222-valid and is what this port has
 * always sent. The computation itself (see ncsi_compute_checksum() in
 * types.h) has been verified against the actual DSP0222 v1.2.1 spec text
 * (clause 8.2.2.3) and a self-consistency test (tests/test_ncsi.c) that
 * checksums each response type and confirms the region sums to zero when
 * the checksum field is included -- still worth confirming against a real
 * BMC exchange before turning this on in production, since no real
 * checksummed capture has been checked against it. */
#ifndef NCSI_COMPUTE_CHECKSUM_DEFAULT
#define NCSI_COMPUTE_CHECKSUM_DEFAULT false
#endif

#endif /* NCSI_BOARD_CONFIG_H */
