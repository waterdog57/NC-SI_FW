////////////////////////////////////////////////////////////////////////////////
///
/// @file       ncsi_board_config.c
///
/// @brief      Fill in the two functions below against your board's actual
///             GPIO peripheral. See ncsi_board_config.h for the compile-time
///             values (version, manufacturer ID, PCI identity, package
///             channel count) that also need setting for your board.
///
/// New code written for this port (not from bcm5719-fw).
///
////////////////////////////////////////////////////////////////////////////////

#include <ncsi_board_config.h>

/* TODO: include your board's GPIO driver header here, e.g.
 *   #include "board_gpio.h"
 */

uint8_t NCSI_BoardConfig_ReadPackageID(void)
{
    /* TODO: read whichever GPIO strap pin(s) encode this package's DSP0222
     * Package ID (3 bits, 0-7) and return the value. Both ICs of one OCP
     * NIC 3.0 card pair must read the same value here (they're wired to
     * the same strap signal(s), being one package with two channels) --
     * this only needs to differ between *separate* card pairs sharing one
     * NC-SI bus.
     *
     * Example, if package ID were 3 individual GPIO pins:
     *   return (uint8_t)((board_gpio_read(GPIO_PKG_ID_BIT0) << 0) |
     *                     (board_gpio_read(GPIO_PKG_ID_BIT1) << 1) |
     *                     (board_gpio_read(GPIO_PKG_ID_BIT2) << 2));
     */
    return 0; /* placeholder */
}

uint8_t NCSI_BoardConfig_ReadChannelID(void)
{
    /* OCP NIC 3.0: this chip's Channel ID is a single GPIO strap (GPIO20)
     * that reads differently on each of the two ICs of a pair -- one is
     * wired to 0, the other to 1. Both chips run the exact same firmware
     * binary; this is the only thing that tells them apart.
     *
     * TODO: replace with your actual GPIO peripheral read call, e.g.:
     *   return (uint8_t)board_gpio_read(20);
     */
    return 0; /* placeholder */
}
