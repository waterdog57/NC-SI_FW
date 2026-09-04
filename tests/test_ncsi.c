////////////////////////////////////////////////////////////////////////////////
///
/// @file       test_ncsi.c
///
/// @brief      Host-side smoke test for the ported NC-SI stack.
///
/// New code written for this port. Feeds handleNCSIFrame() the real
/// on-the-wire request frames captured in valid_commands.c (from
/// meklort/bcm5719-fw, BSD-3-Clause), using a trivial in-RAM stand-in for
/// the Network.h HAL, and checks the responses come back well-formed. This
/// exercises the protocol parsing/response logic on your dev machine
/// without needing RISC-V32 hardware; it does NOT exercise your real
/// MAC/PHY driver (that's what ncsi_hal_template.c is for, once you fill it
/// in for your board).
///
/// Build & run (from repo root):
///   gcc -std=c11 -Isrc/ncsi/include -Itests src/ncsi/ncsi.c
///       tests/valid_commands.c tests/test_ncsi.c -o test_ncsi
///   ./test_ncsi
///
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>

#include <types.h>
#include <Ethernet.h>
#include <NCSI.h>
#include <Network.h>
#include <ncsi_board_config.h>

/* Test vectors from valid_commands.c */
extern uint8_t select_package1[];
extern uint8_t clear_initial_state[];
extern uint8_t disable_vlan[];
extern uint8_t set_mac_addr[];
extern uint8_t enable_bcast_filter[];
extern uint8_t enable_network_tx[];
extern uint8_t enable_channel[];
extern uint8_t aen_enable[];
extern uint8_t get_link_status_ch0[];
extern uint8_t get_link_status_ch1[];
extern uint8_t disable_network_tx[];
extern uint8_t disable_channel[];
extern uint8_t deselect_package[];
extern uint8_t get_capabilities[];
extern uint8_t get_version_id[];

extern uint32_t select_package1_len;
extern uint32_t clear_initial_state_len;
extern uint32_t disable_vlan_len;
extern uint32_t set_mac_addr_len;
extern uint32_t enable_bcast_filter_len;
extern uint32_t enable_network_tx_len;
extern uint32_t enable_channel_len;
extern uint32_t aen_enable_len;
extern uint32_t get_link_status_ch0_len;
extern uint32_t get_link_status_ch1_len;
extern uint32_t disable_network_tx_len;
extern uint32_t disable_channel_len;
extern uint32_t deselect_package_len;
extern uint32_t get_capabilities_len;
extern uint32_t get_version_id_len;

/* --- Minimal ncsi_board_config.h stand-in for host testing --------------
 * Real board_config.c reads GPIO; here it's just whatever the test set. */
static uint8_t gTestPackageId;
static uint8_t gTestChannelId;
uint8_t NCSI_BoardConfig_ReadPackageID(void) { return gTestPackageId; }
uint8_t NCSI_BoardConfig_ReadChannelID(void) { return gTestChannelId; }

/* --- Minimal Network.h HAL stand-in for host testing -------------------- */

static uint8_t gTxBuf[512];
static uint32_t gTxLen;
static bool gTxCalled;

void Network_InitPort(NetworkPort_t *port, reload_type_t reset_phy) { (void)port; (void)reset_phy; }
bool Network_isLinkUp(NetworkPort_t *port) { (void)port; return true; }
void Network_resetLink(NetworkPort_t *port) { (void)port; }
static uint16_t gLastMacHigh;
static uint32_t gLastMacLow;
static uint32_t gLastMacIndex;
static bool gLastMacEnable;
static bool gSetMacCalled;

void Network_SetMACAddr(NetworkPort_t *port, uint16_t h, uint32_t l, uint32_t idx, bool en)
{
    (void)port;
    gLastMacHigh = h;
    gLastMacLow = l;
    gLastMacIndex = idx;
    gLastMacEnable = en;
    gSetMacCalled = true;
}
bool Network_PassthroughRxPacket(NetworkPort_t *port) { (void)port; return true; }

bool Network_TxFrame(NetworkPort_t *port, const uint8_t *frame, uint32_t frame_len)
{
    (void)port;
    if (frame_len > sizeof(gTxBuf))
    {
        frame_len = sizeof(gTxBuf);
    }
    memcpy(gTxBuf, frame, frame_len);
    gTxLen = frame_len;
    gTxCalled = true;
    return true;
}

uint8_t MII_getPhy(void *device) { (void)device; return 1; }

int32_t MII_readRegister(void *device, uint8_t phy, uint8_t reg)
{
    (void)device; (void)phy; (void)reg;
    return (int32_t)MII_BMSR_LINK_STATUS_BIT; /* pretend link is always up */
}

bool MII_reset(void *device, uint8_t phy) { (void)device; (void)phy; return true; }

/* --- Test driver ---------------------------------------------------------
 * Decodes both the request and the response through the same NetworkFrame_t
 * union ncsi.c itself uses, rather than hand-computing byte offsets --
 * that keeps this test honest about the struct layout instead of guessing
 * at it independently. */

static int failures = 0;
#define CHECK(cond, name)                                                    \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            printf("FAIL [%s] %s (%s:%d)\n", name, #cond, __FILE__, __LINE__); \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static void send(const uint8_t *pkt, uint32_t len, const char *name, uint16_t expect_response_code)
{
    (void)len; /* handleNCSIFrame() reads the length from the frame's own PayloadLength field */
    const NetworkFrame_t *req = (const NetworkFrame_t *)pkt;

    gTxCalled = false;
    gTxLen = 0;
    handleNCSIFrame(req);

    CHECK(gTxCalled, name);
    if (!gTxCalled)
    {
        return;
    }

    const NetworkFrame_t *resp = (const NetworkFrame_t *)gTxBuf;

    uint16_t resp_response_code = ncsi_rd16(resp->responsePacket.ResponseCode);
    uint16_t resp_reason_code = ncsi_rd16(resp->responsePacket.ReasonCode);

    CHECK(ncsi_rd16(resp->header.EtherType) == ETHER_TYPE_NCSI, name);
    CHECK((resp->controlPacket.ControlPacketType & CONTROL_PACKET_TYPE_RESPONSE) != 0, name);
    CHECK((resp->controlPacket.ControlPacketType & ~CONTROL_PACKET_TYPE_RESPONSE) ==
              req->controlPacket.ControlPacketType,
          name);
    CHECK(resp->controlPacket.InstanceID == req->controlPacket.InstanceID, name);
    CHECK(resp_response_code == expect_response_code, name);

    printf("PASS [%s] tx_len=%u response_code=0x%04x reason_code=0x%04x\n",
           name, gTxLen, resp_response_code, resp_reason_code);
}

int main(void)
{
    static NetworkPort_t port;
    memset(&port, 0, sizeof(port));
    NCSI_usePort(&port);
    gTestPackageId = 0;
    gTestChannelId = 0; /* every captured vector below targets package 0, channel 0 */
    NCSI_init();

    /* Mirrors the discovery sequence a BMC actually performs. */
    send(select_package1, select_package1_len, "Select Package", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(clear_initial_state, clear_initial_state_len, "Clear Initial State", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(enable_channel, enable_channel_len, "Enable Channel", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(get_version_id, get_version_id_len, "Get Version ID", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(get_capabilities, get_capabilities_len, "Get Capabilities", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(get_link_status_ch0, get_link_status_ch0_len, "Get Link Status", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(set_mac_addr, set_mac_addr_len, "Set MAC Address", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    /* Content check: the real captured request should decode to MAC
     * 2c:09:4d:00:01:4a, filter slot 0 (NC-SI's 1-based MACNumber=1). This
     * is what pinned down SetMACAddr_t's field order/split in Ethernet.h. */
    CHECK(gSetMacCalled, "Set MAC Address (HAL called)");
    CHECK(gLastMacHigh == 0x2c09, "Set MAC Address (top 16 bits)");
    CHECK(gLastMacLow == 0x4d00014au, "Set MAC Address (bottom 32 bits)");
    CHECK(gLastMacIndex == 0, "Set MAC Address (0-based filter slot)");
    printf("PASS [Set MAC Address content] MAC=%02x:%02x:%04x:%04x slot=%u enable=%d\n",
           (gLastMacHigh >> 8) & 0xff, gLastMacHigh & 0xff, (unsigned)(gLastMacLow >> 16),
           (unsigned)(gLastMacLow & 0xffff), gLastMacIndex, gLastMacEnable);

    send(aen_enable, aen_enable_len, "AEN Enable", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    /* Content check: should decode to AENControl=0x7 (link/config/driver
     * status AENs enabled) -- what pinned down AENEnable_t's field order. */
    CHECK(port.state.aen_control == 0x7u, "AEN Enable (AENControl bitmask)");
    printf("PASS [AEN Enable content] AENControl=0x%x AEN_MC_ID=%u\n",
           port.state.aen_control, port.state.aen_mc_id);
    send(enable_network_tx, enable_network_tx_len, "Enable Network TX", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(disable_network_tx, disable_network_tx_len, "Disable Network TX", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(enable_bcast_filter, enable_bcast_filter_len, "Enable Broadcast Filter", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(disable_vlan, disable_vlan_len, "Disable VLAN", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(disable_channel, disable_channel_len, "Disable Channel", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);
    send(deselect_package, deselect_package_len, "Deselect Package", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);

    /* A channel command sent before Clear Initial State must be rejected. */
    NCSI_init(); /* reset state -- ready goes back to false */
    send(get_version_id, get_version_id_len, "Get Version ID (uninitialized)", NCSI_RESPONSE_CODE_COMMAND_FAILED);

    /* --- Package/Channel addressing (OCP NIC 3.0: two ICs, one package,
     * channels 0/1) -- every captured vector below targets package 0,
     * channel 0 or 1. A responder must stay *silent* (no TX at all) for
     * frames addressed to a different package or a different channel --
     * see handleNCSIFrame()'s addressed_to_us check. */
    printf("\n-- Addressing tests --\n");

    /* Wrong package entirely: silence regardless of channel. */
    gTestPackageId = 5;
    gTestChannelId = 0;
    NCSI_init();
    gTxCalled = false;
    handleNCSIFrame((const NetworkFrame_t *)select_package1);
    CHECK(!gTxCalled, "Wrong package (Select Package) -- must stay silent");
    printf("%s [Wrong package] silent=%d\n", gTxCalled ? "FAIL" : "PASS", !gTxCalled);

    /* Right package, but this instance is channel 1 -- a channel-0 frame
     * must be ignored, and a channel-1 frame (a real capture,
     * get_link_status_ch1) must get a response. */
    gTestPackageId = 0;
    gTestChannelId = 1;
    NCSI_init();

    gTxCalled = false;
    handleNCSIFrame((const NetworkFrame_t *)clear_initial_state); /* targets channel 0 */
    CHECK(!gTxCalled, "Wrong channel (channel-0 frame, we are channel 1) -- must stay silent");
    printf("%s [Wrong channel] silent=%d\n", gTxCalled ? "FAIL" : "PASS", !gTxCalled);

    /* Our channel (1), but Clear Initial State was never sent for it in
     * this scenario, so this should be answered (not silence) with
     * INITIALIZATION_REQUIRED -- proving channel 1 is actually recognized
     * as ours, not just coincidentally silent. */
    send(get_link_status_ch1, get_link_status_ch1_len, "Get Link Status ch1 (before Clear Initial State)",
         NCSI_RESPONSE_CODE_COMMAND_FAILED);

    /* Select Package is package-wide and always answered regardless of
     * which channel we are. */
    send(select_package1, select_package1_len, "Select Package (as channel 1)", NCSI_RESPONSE_CODE_COMMAND_COMPLETE);

    /* --- Regression: gNCSIHandlers[] out-of-bounds read ----------------
     * gNCSIHandlers[] is only sized to its highest designated-initializer
     * index (0x1A -> 27 entries), but ControlPacketType is a full uint8_t
     * straight off the wire (0-255). Before this was fixed, indexing it
     * with an out-of-range command (OEM Command, 0x50, or outright garbage)
     * read past the end of the array as a fabricated ncsi_handler_t --
     * confirmed with AddressSanitizer: "global-buffer-overflow ... READ of
     * size 8" at the `handler->fn` dereference, which would have called
     * through whatever garbage pointer happened to sit there. Must now
     * answer COMMAND_UNSUPPORTED, same as any other unimplemented command,
     * not crash / read out of bounds (best checked by simply building this
     * whole test binary with -fsanitize=address,undefined; a plain build
     * can only confirm it *responds*, not that the read was in-bounds). */
    gTestPackageId = 0;
    gTestChannelId = 0;
    NCSI_init();
    {
        uint8_t oem_cmd[64];
        memset(oem_cmd, 0, sizeof(oem_cmd));
        memcpy(oem_cmd, select_package1, select_package1_len); /* real frame, real length -- only mutate the opcode byte below */
        oem_cmd[18] = 0x50; /* OEM Command -- valid DSP0222 opcode, past the table */
        send(oem_cmd, select_package1_len, "OEM Command (0x50, past handler table)", NCSI_RESPONSE_CODE_COMMAND_UNSUPPORTED);

        uint8_t garbage_cmd[64];
        memset(garbage_cmd, 0, sizeof(garbage_cmd));
        memcpy(garbage_cmd, select_package1, select_package1_len);
        /* 0x30: not a real opcode, and deliberately without bit 0x80 set --
         * that bit marks *responses* only, no real request would carry it,
         * and using a value that did (0xFF) confused this test's own
         * request/response echo check above, not the firmware. */
        garbage_cmd[18] = 0x30;
        send(garbage_cmd, select_package1_len, "Garbage command (0x30)", NCSI_RESPONSE_CODE_COMMAND_UNSUPPORTED);
    }

    if (failures)
    {
        printf("\n%d check(s) FAILED\n", failures);
        return 1;
    }
    printf("\nAll checks passed.\n");
    return 0;
}
