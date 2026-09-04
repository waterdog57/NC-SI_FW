////////////////////////////////////////////////////////////////////////////////
///
/// @file       nc.c
///
/// @brief      Example: wiring this NC-SI stack into a bare-metal project's
///             board init + main loop. Illustrative only -- copy the parts
///             you need into your own startup code, don't build this file
///             as-is (board_init()/interrupt registration are stand-ins for
///             whatever your project already has).
///
/// This shows how the pieces from src/ncsi/ actually fit together at
/// runtime, end to end:
///
///   1. Your board brings up the MAC/PHY (board_init()).
///   2. ncsi_init_and_register_ports() registers this chip's one local
///      NetworkPort_t and reads its package/channel identity (GPIO).
///   3. The RX OK interrupt is connected to NCSI_RxOk_ISR() (descriptor
///      bookkeeping only -- no protocol processing in interrupt context).
///   4. The main loop calls NCSI_PollRx() every iteration; that's where
///      handleNCSIFrame() actually runs, answering the BMC's commands.
///
/// New code written for this port (not from bcm5719-fw) -- a usage example,
/// not a HAL implementation. The actual HAL lives in ncsi_hal_template.c
/// (copy that to ncsi_hal.c and fill in the TODOs); ncsi_board_config.c
/// supplies the GPIO-read package/channel identity this depends on.
///
////////////////////////////////////////////////////////////////////////////////

#include <NCSI.h>
#include <Network.h>

/* Declared in your filled-in ncsi_hal.c (copied from ncsi_hal_template.c):
 *   void ncsi_init_and_register_ports(void);  -- NCSI_usePort() + NCSI_init()
 *   void NCSI_RxOk_ISR(void);                 -- connect to your RX OK vector
 *   void NCSI_PollRx(void);                   -- call every main-loop pass
 */
extern void ncsi_init_and_register_ports(void);
extern void NCSI_RxOk_ISR(void);
extern void NCSI_PollRx(void);

/* TODO: replace with your project's actual board bring-up -- clocks, pin
 * mux, MAC/PHY reset and init, MDIO, DMA descriptor setup, etc. Everything
 * NCSI_init() depends on (Network_InitPort(), MII_getPhy(), MII_reset(),
 * and the GPIO reads in ncsi_board_config.c) must already work by the time
 * ncsi_init_and_register_ports() is called below. */
static void board_init(void)
{
    /* TODO: your board/clock/pin init here. */
    /* TODO: bring up your Ethernet MAC + PHY (MDIO ready, link training
     * can still be in progress -- NC-SI's own Get Link Status handles
     * that). */
    /* TODO: set up your RX DMA descriptor(s) so hardware has somewhere to
     * write received frames before interrupts are enabled. */
}

/* TODO: replace with however your project registers an interrupt handler
 * (a vector table entry, a driver callback registration call, etc.) --
 * this is a placeholder to show *what* gets connected, not *how*. */
static void register_rx_ok_interrupt(void (*handler)(void))
{
    (void)handler;
    /* TODO: e.g. irq_register(IRQ_NCSI_RX_OK, handler);
     *       irq_enable(IRQ_NCSI_RX_OK); */
}

int main(void)
{
    board_init();

    /* Registers this chip's NetworkPort_t and reads its package/channel
     * identity via ncsi_board_config.c's GPIO functions. Must run after
     * board_init() (needs a working MAC/PHY/MDIO underneath it) and before
     * any NC-SI traffic can arrive. */
    ncsi_init_and_register_ports();

    /* From here on, RX OK interrupts just record where the DMA-written
     * frame landed and set a flag (NCSI_RxOk_ISR()) -- handleNCSIFrame()
     * itself only ever runs from the main loop below, in NCSI_PollRx(),
     * per this project's interrupt-context design decision. */
    register_rx_ok_interrupt(NCSI_RxOk_ISR);

    for (;;)
    {
        NCSI_PollRx(); /* processes one pending received NC-SI frame, if any */

        /* TODO: whatever else your firmware's main loop needs to do
         * (other peripherals, watchdog kick, power management, ...). If
         * you're running an RTOS instead of a bare super-loop, call
         * NCSI_PollRx() from a task instead -- see README.md, "Quick
         * start" step 4. */
    }
}
