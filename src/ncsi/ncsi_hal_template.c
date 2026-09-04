////////////////////////////////////////////////////////////////////////////////
///
/// @file       ncsi_hal_template.c
///
/// @brief      Starting point for the Network.h HAL on your RISC-V32 board.
///
/// Copy this file (drop the "_template" suffix) and fill in every TODO
/// against your board's real Ethernet MAC / PHY / MDIO driver. Nothing here
/// is from bcm5719-fw -- it's new glue code for this port. See
/// src/ncsi/include/Network.h for what each function is contractually
/// required to do.
///
////////////////////////////////////////////////////////////////////////////////

#include <Network.h>
#include <Ethernet.h>
#include <NCSI.h>

/* TODO: include your board's MAC/PHY/MDIO driver headers here, e.g.
 *   #include "board_emac.h"
 */

/* This firmware instance manages exactly one local NC-SI channel -- its own.
 * (On this board, two physical ICs each run this same firmware image and
 * together form one DSP0222 package with two channels; each chip only ever
 * knows about its own single channel, identified at runtime by
 * ncsi_board_config.c's NCSI_BoardConfig_ReadChannelID(). Do not raise
 * NCSI_MAX_CHANNELS to 2 to "add the other channel" here -- that channel is
 * a different physical chip, not a second local port on this one.) */
static NetworkPort_t gPorts[NCSI_MAX_CHANNELS];

void Network_InitPort(NetworkPort_t *port, reload_type_t reset_phy)
{
    (void)port;
    (void)reset_phy;
    /* TODO: bring the MAC/PHY link up (or re-check it) here. If
     * reset_phy == NCSI_RELOAD_FORCE, force a PHY reset even if the link
     * currently looks fine (mirrors NCSI_reload()/Reset Channel behavior).
     */
}

bool Network_isLinkUp(NetworkPort_t *port)
{
    (void)port;
    /* TODO: return true if your MAC driver's own link-up flag is set.
     * (getLinkStatusHandler() already reads the PHY directly via
     * MII_readRegister() -- this is a second, independent check it uses to
     * decide whether to call Network_resetLink(), matching upstream.) */
    return false;
}

void Network_resetLink(NetworkPort_t *port)
{
    (void)port;
    /* TODO: kick the link back up, e.g. re-trigger autonegotiation. */
}

void Network_SetMACAddr(NetworkPort_t *port, uint16_t mac_high16, uint32_t mac_low32,
                         uint32_t index, bool enable)
{
    (void)port;
    (void)index;
    uint8_t mac[6] = {
        (uint8_t)(mac_high16 >> 8), (uint8_t)mac_high16,
        (uint8_t)(mac_low32 >> 24), (uint8_t)(mac_low32 >> 16),
        (uint8_t)(mac_low32 >> 8), (uint8_t)mac_low32,
    };
    (void)mac;
    (void)enable;
    /* TODO: program `mac` into MAC address filter slot `index` on your MAC,
     * enabling or disabling that slot per `enable`. */
}

bool Network_PassthroughRxPacket(NetworkPort_t *port)
{
    (void)port;
    /* TODO: if a non-NC-SI-control frame is waiting to be forwarded to the
     * host network side, forward it and return true; return true also if
     * there's simply nothing pending. Return false only when a frame was
     * available but had to be dropped (e.g. no buffer space) -- ncsi.c
     * counts that in state.stat_net_dropped. */
    return true;
}

/* --- TX descriptor / polling-bit kick -----------------------------------
 * This board's NC-SI MAC transmits by descriptor: write the frame's buffer
 * address and length into the descriptor, then set a "polling bit" that
 * tells TX hardware to start fetching from that address on its own.
 * Replace ncsi_tx_descriptor_t's fields and the register placeholders
 * below with your actual layout -- shape only, not real hardware. */
typedef struct
{
    volatile uint32_t buffer_addr; /* TODO: match your descriptor's field */
    volatile uint32_t length;      /* TODO: match your descriptor's field */
    volatile uint32_t status;      /* TODO: TX-done/error bit(s), if present */
} ncsi_tx_descriptor_t;

/* TODO: point this at your controller's actual TX descriptor (a fixed
 * register block, or a ring entry -- adjust Network_TxFrame() below to
 * advance through it if so). */
#define NCSI_TX_DESCRIPTOR ((ncsi_tx_descriptor_t *)0 /* TODO: real address */)

/* TODO: replace with your actual "kick"/doorbell register address and the
 * bit that starts the TX hardware polling the descriptor. */
#define NCSI_TX_POLL_BIT_REG ((volatile uint32_t *)0 /* TODO: real address */)
#define NCSI_TX_POLL_BIT (1u << 0) /* TODO: actual bit position */

/* TODO: replace with your descriptor's real "transmit finished" bit, and
 * pick a timeout that comfortably covers one frame at your link speed. */
#define NCSI_TX_STATUS_DONE (1u << 0)
#define NCSI_TX_TIMEOUT_LOOPS 100000u

bool Network_TxFrame(NetworkPort_t *port, const uint8_t *frame, uint32_t frame_len)
{
    (void)port;
    ncsi_tx_descriptor_t *desc = NCSI_TX_DESCRIPTOR;

    desc->buffer_addr = (uint32_t)(uintptr_t)frame;
    desc->length = frame_len;

    /* TODO: if your descriptor needs an owner/valid bit set before hardware
     * will act on it, do that here, before the doorbell write below. */

    *NCSI_TX_POLL_BIT_REG = NCSI_TX_POLL_BIT; /* kick: TX hw starts fetching from buffer_addr */

    /* Wait for hardware to actually finish reading `frame` before
     * returning. This matters here specifically: ncsi.c's callers (
     * sendNCSIResponse(), getCapabilities(), getVersionID(), ...) all
     * transmit out of a small set of *static* per-response-type buffers
     * (gResponseFrame, gCapabilitiesFrame, gVersionFrame,
     * gLinkStatusResponseFrame) that get reused/overwritten by the next
     * response of that same type. If this returned while the DMA engine
     * is still mid-transfer and a second same-type response got built
     * before the first one finished sending, the buffer would be mutated
     * out from under an in-flight DMA read. A bounded poll here is the
     * simplest correct fix; swap for an interrupt-driven "TX done" wait if
     * busy-polling here is unacceptable for your board. */
    uint32_t timeout = NCSI_TX_TIMEOUT_LOOPS;
    while ((desc->status & NCSI_TX_STATUS_DONE) == 0 && --timeout)
    {
        /* spin */
    }

    /* TODO: acknowledge/clear the TX-done condition in your controller if
     * it needs that before the descriptor can be reused (mirrors the RX OK
     * ISR's interrupt-ack TODO below). */

    return timeout != 0;
}

uint8_t MII_getPhy(void *device)
{
    (void)device;
    /* TODO: return the MDIO address of the PHY behind `device` (0-31). */
    return 0;
}

int32_t MII_readRegister(void *device, uint8_t phy, uint8_t reg)
{
    (void)device;
    (void)phy;
    (void)reg;
    /* TODO: MDIO-read `reg` from `phy` and return its 16-bit value, or -1
     * on failure/timeout. getLinkStatusHandler() reads REG_MII_BASIC_STATUS
     * (standard MII register 1 / BMSR) and checks MII_BMSR_LINK_STATUS_BIT. */
    return -1;
}

bool MII_reset(void *device, uint8_t phy)
{
    (void)device;
    (void)phy;
    /* TODO: MDIO-write the PHY's reset bit (BMCR bit 15) and wait for it to
     * self-clear. Return true on success. */
    return false;
}

/* --- RX frame dispatch ---------------------------------------------------
 * Hands one already-received Ethernet frame to the NC-SI engine if it's a
 * control frame (EtherType 0x88F8), otherwise leaves it for your normal RX
 * path / passthrough. Called from NCSI_PollRx() below, never from ISR
 * context. */
void ncsi_on_rx_frame(const uint8_t *frame, uint32_t frame_len)
{
    if (frame_len < sizeof(ControlPacketHeader_t))
    {
        return;
    }

    const NetworkFrame_t *ncsi_frame = (const NetworkFrame_t *)frame;
    if (ncsi_rd16(ncsi_frame->header.EtherType) == ETHER_TYPE_NCSI)
    {
        handleNCSIFrame(ncsi_frame);
        return;
    }

    /* Not an NC-SI control frame -- DSP0222 clause 6.1.11 (Pass-through,
     * BMC-to-network direction): "Packets not recognized as command
     * packets ... shall be assumed to be Pass-through packets provided
     * that the source MAC Address matches one of the unicast MAC addresses
     * settings ..., and will be forwarded for transmission to the
     * corresponding external network interface if Channel Network TX is
     * enabled." ncsi_passthrough_tx_from_mc() (ncsi.c) checks both
     * conditions and does the forwarding; it silently declines (no NC-SI
     * response either way for Pass-through packets) if either isn't met. */
    ncsi_passthrough_tx_from_mc(&gPorts[0], ncsi_frame->header.SourceAddress, frame, frame_len);
}

/* --- RX OK interrupt / DMA descriptor -----------------------------------
 * This board's NC-SI MAC signals "RX OK" by interrupt; its DMA engine has
 * already written the received frame into the buffer address named by the
 * descriptor and filled in the frame's length there by the time the
 * interrupt fires. Replace ncsi_rx_descriptor_t's fields with your actual
 * descriptor layout (register names/offsets, endianness, any owner/valid
 * bit you must check or clear) -- this is a placeholder shape, not real
 * hardware. The ISR does the minimum possible (copy descriptor fields, set
 * a flag) and returns; handleNCSIFrame() itself runs later, from
 * NCSI_PollRx(), called from your main loop / a task -- not from ISR
 * context. */
typedef struct
{
    volatile uint32_t buffer_addr; /* TODO: match your descriptor's field */
    volatile uint32_t length;      /* TODO: match your descriptor's field */
    volatile uint32_t status;      /* TODO: owner/valid/error bits, if any */
} ncsi_rx_descriptor_t;

/* TODO: point this at your controller's actual RX descriptor (a fixed
 * register block, or an entry in a ring buffer your DMA engine advances
 * through -- adjust gRxPending/NCSI_PollRx() below to walk the ring if so). */
#define NCSI_RX_DESCRIPTOR ((ncsi_rx_descriptor_t *)0 /* TODO: real address */)

static volatile bool gRxPending;
static const uint8_t *gRxFrame;
static uint32_t gRxFrameLen;

/* Connect this to your RX OK interrupt vector. Keep it minimal: no NC-SI
 * protocol processing here, only descriptor bookkeeping -- per this
 * project's design decision, handleNCSIFrame() must never run in ISR
 * context. */
void NCSI_RxOk_ISR(void)
{
    ncsi_rx_descriptor_t *desc = NCSI_RX_DESCRIPTOR;

    /* TODO: if your descriptor has an owner/valid/error bit, check it here
     * before trusting buffer_addr/length. */
    gRxFrame = (const uint8_t *)(uintptr_t)desc->buffer_addr;
    gRxFrameLen = desc->length;
    gRxPending = true;

    /* TODO: acknowledge/clear the RX OK interrupt in your controller here
     * (e.g. write-1-to-clear an interrupt-status register), or the ISR will
     * re-fire immediately on return. */
}

/* Call this every iteration of your main loop (or from a dedicated NC-SI
 * task, if you're running an RTOS). This is where handleNCSIFrame() (via
 * ncsi_on_rx_frame()) actually runs. */
void NCSI_PollRx(void)
{
    if (!gRxPending)
    {
        return;
    }
    gRxPending = false;

    ncsi_on_rx_frame(gRxFrame, gRxFrameLen);

    /* TODO: return the descriptor/buffer to hardware ownership so DMA can
     * reuse it for the next received frame (e.g. set an owner bit back to
     * "device", or advance to the next ring entry). Until this happens the
     * MAC likely can't receive another frame. */
}

/* --- Startup glue -------------------------------------------------------
 * Call this once during board init, after your MAC/PHY driver is ready,
 * before NC-SI traffic can arrive. */
void ncsi_init_and_register_ports(void)
{
    gPorts[0].device = NULL; /* TODO: your MAC/PHY driver handle */
    NCSI_usePort(&gPorts[0]);

    NCSI_init(); /* also reads this chip's package/channel identity -- see
                  * ncsi_board_config.c */
}
