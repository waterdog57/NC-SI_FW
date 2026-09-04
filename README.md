# NC-SI (DSP0222) device/responder stack for RISC-V32

This project ports the NC-SI protocol engine from
[`meklort/bcm5719-fw`](https://github.com/meklort/bcm5719-fw)'s `libs/NCSI/`
(BSD-3-Clause) to a generic bare-metal / minimal-RTOS RISC-V32 target. It
implements the **device (NC) side** of NC-SI: your RISC-V32 firmware
*receives and answers* NC-SI control frames (EtherType `0x88F8`) sent by a
BMC's Management Controller (MC), rather than sending them — that's the
opposite, *host* side, which is what Linux's `net/ncsi` and U-Boot's
`drivers/net/phy/ncsi.c` implement and isn't reusable here.

**Board context**: an OCP NIC 3.0 card with two RISC-V32 ICs running this
exact same firmware image. Their RMII/RBT sideband pins are tied together in
hardware and wired to one BMC sideband port — together the two chips form
**one DSP0222 package with two channels**, each chip answering only its own
channel and staying silent for the other's (both chips see every frame on
the shared bus). See "How package/channel addressing works" below.

## Repository layout

- `third_party/ncsi/` — the vendored upstream source, unmodified (required
  by its BSD-3-Clause license). Reference only; don't build against it, it
  depends on BCM5719-only headers not included here.
- `src/ncsi/` — the actual port:
  - `include/types.h` — portable base types + the `ncsi_rd16/wr16/rd32/wr32`
    wire-format accessors (see "Bugs found and fixed" below).
  - `include/Ethernet.h`, `include/NCSI.h` — DSP0222 packet structs and the
    public API, adapted from upstream (see file header comments for exactly
    what changed).
  - `include/Network.h` — the hardware-abstraction contract `ncsi.c` is
    written against (new for this port; upstream's equivalent lives inside
    BCM5719's non-public APE firmware tree).
  - `include/ncsi_board_config.h`, `ncsi_board_config.c` — **the one file to
    edit for your board**: package/channel ID GPIO reads, firmware version,
    manufacturer ID, PCI identity, package channel count.
  - `ncsi.c` — the command engine, ported from upstream's `ncsi.c`.
  - `ncsi_hal_template.c` — starting point for your board's MAC/PHY/MDIO/DMA
    glue, including the TX-descriptor and RX-OK-interrupt patterns. Copy it,
    drop the `_template` suffix, fill in the `TODO`s.
- `src/build.mk` — Makefile fragment so another project can `include` this
  stack directly; see its header comment for usage.
- `examples/nc.c` — illustrative example wiring board init, `ncsi_hal.c`,
  and the main loop together end to end. Not meant to be built as-is.
- `tests/` — `valid_commands.c` (real captured DSP0222 request frames, from
  upstream) and `test_ncsi.c` (a host-buildable smoke test, new for this
  port — no RISC-V hardware needed to run it).
- `CLAUDE.md` — persistent project context/decision log for AI-assisted
  sessions in this repo.

## Quick start: bringing this up on your board

### 1. What to configure

Everything board-specific lives in exactly two files:

```
src/ncsi/ncsi_board_config.c            (runtime GPIO reads)
src/ncsi/include/ncsi_board_config.h    (compile-time constants)
```

**`ncsi_board_config.c`** — two functions to fill in:

- **`NCSI_BoardConfig_ReadPackageID(void)`** — read whichever GPIO pin(s)
  strap this card's DSP0222 Package ID (3 bits, values 0–7). Both ICs of one
  card pair must return the **same** value (they're wired to the same strap
  signal(s) — they're one package). Only needs to be unique across
  *different* card pairs if your system has more than one NC-SI package on
  one bus. Which GPIO pin(s) carry this strap isn't specified yet — fill in
  against your actual schematic.
- **`NCSI_BoardConfig_ReadChannelID(void)`** — read GPIO20. On this design
  GPIO20 is strapped differently on each of the two ICs of a pair — one
  chip reads 0, the other reads 1. That's the *only* thing that
  distinguishes the two otherwise-identical firmware images at runtime.
  Replace the placeholder `return 0;` with your actual GPIO read call for
  pin 20.

Both functions are called exactly once, from `NCSI_init()`, which you call
from `ncsi_hal.c`'s `ncsi_init_and_register_ports()` (see step 5 below).

**`ncsi_board_config.h`** — compile-time values to review and set (each has
a placeholder default):

| Constant | Default | Meaning |
|---|---|---|
| `NCSI_PACKAGE_CHANNEL_COUNT` | `2` | Total channels in this **package** (both ICs combined) — leave at 2 for this board |
| `NCSI_FW_VERSION_MAJOR/MINOR/PATCH` | `0/1/0` | Your firmware's own version, reported in Get Version ID |
| `NCSI_MANUFACTURER_ID` | `0` | Your real DMTF/IANA Enterprise Number — 0 isn't valid, **must** be changed before shipping |
| `NCSI_PCI_VENDOR_ID` / `_DEVICE_ID` / `_SUBSYSTEM_VENDOR_ID` / `_SUBSYSTEM_DEVICE_ID` | `0xFFFF` | DSP0222's "not applicable" sentinel — only set real values if this device has actual PCI config space to report |

Any of these can also be overridden at build time instead of editing the
header, e.g. `-DNCSI_MANUFACTURER_ID=0x12345678`.

### 2. MAC/PHY/MDIO HAL

Copy `src/ncsi/ncsi_hal_template.c` to e.g. `src/ncsi/ncsi_hal.c` and fill
in every `TODO`. This is the file that talks to your actual
MAC/PHY/MDIO/DMA registers — nothing in it is board-specific yet. What each
function must do is documented in `src/ncsi/include/Network.h`; short
version:

| Function | Purpose |
|---|---|
| `Network_InitPort` / `Network_isLinkUp` / `Network_resetLink` | bring up / query / reset the link |
| `Network_SetMACAddr` | program a MAC address filter slot (called from Set MAC Address) |
| `Network_TxFrame` | transmit an NC-SI response frame — see step 3, needs more than a TODO fill-in on this board |
| `MII_getPhy` / `MII_readRegister` / `MII_reset` | MDIO access to the PHY (link-status polling and reset) |
| `Network_PassthroughRxPacket` | forward a non-NC-SI frame toward the host network side, if your design needs that (Enable Channel Network TX support) |

### 3. TX: descriptor + polling bit

This board's NC-SI MAC transmits by descriptor: fill in the buffer address
and length in a descriptor, then set a "polling bit" — at that point TX
hardware fetches from that address on its own and sends it out. Implemented
in `ncsi_hal_template.c`'s `Network_TxFrame()`:

| Symbol | Purpose |
|---|---|
| `ncsi_tx_descriptor_t` | PLACEHOLDER struct (`buffer_addr` / `length` / `status`) — replace with your actual descriptor register layout |
| `NCSI_TX_DESCRIPTOR` | PLACEHOLDER macro pointing at your descriptor's real address — must be replaced |
| `NCSI_TX_POLL_BIT_REG` / `NCSI_TX_POLL_BIT` | PLACEHOLDER doorbell register address + bit that actually kicks TX hardware — must be replaced |
| `NCSI_TX_STATUS_DONE` / `NCSI_TX_TIMEOUT_LOOPS` | PLACEHOLDER "transmit finished" bit + a bounded spin-wait on it before `Network_TxFrame()` returns |

**Why it waits for completion instead of just kicking and returning**:
`ncsi.c` transmits out of a small set of *static* per-response-type buffers
(`gResponseFrame`, `gCapabilitiesFrame`, `gVersionFrame`,
`gLinkStatusResponseFrame`) that get reused/overwritten by the next
response of the same type. If `Network_TxFrame()` returned before hardware
actually finished reading the buffer, and a second same-type response got
built before the first one's DMA finished, the buffer would be mutated out
from under an in-flight transfer — a real race, not hypothetical, given how
few buffers there are and how directly they're reused. The bounded poll in
the template is the simplest correct fix; swap it for an interrupt-driven
"TX done" wait instead if busy-polling inside `Network_TxFrame()` is
unacceptable for your board (e.g. block on a semaphore signaled from a
TX-done ISR if you're running an RTOS).

### 4. RX: OK interrupt + descriptor

This board's NC-SI MAC has an RX OK interrupt: its DMA engine writes a
received frame into the buffer address named by a descriptor, fills in the
frame's length there, then raises the interrupt. Per this project's design
decision, **the ISR does the minimum possible** — it does not call into the
NC-SI protocol engine directly. The actual command processing
(`handleNCSIFrame()`, via `ncsi_on_rx_frame()`) runs later, from your main
loop / a task, never from interrupt context. Structured for you in
`ncsi_hal_template.c`:

| Symbol | Purpose |
|---|---|
| `ncsi_rx_descriptor_t` | PLACEHOLDER struct (`buffer_addr` / `length` / `status`) — replace with your actual descriptor register layout |
| `NCSI_RX_DESCRIPTOR` | PLACEHOLDER macro pointing at your descriptor's real address — currently a null-pointer placeholder, must be replaced |
| `NCSI_RxOk_ISR()` | connect to your RX OK interrupt vector. Reads the descriptor, stashes buffer pointer + length, sets a flag, returns. Also where you acknowledge/clear the interrupt (`TODO` in the code) |
| `NCSI_PollRx()` | call every iteration of your main loop (or from a dedicated task). Checks the ISR's flag and, if set, calls `ncsi_on_rx_frame()` → `handleNCSIFrame()` here, in normal context. Also where you return the descriptor/buffer to hardware ownership so DMA can reuse it (`TODO` in the code) |

If your descriptor is one entry in a ring rather than a single fixed
register block, extend `NCSI_RxOk_ISR()`/`NCSI_PollRx()` to walk the ring
instead of a single `ncsi_rx_descriptor_t` — the flag+buffer/length handoff
pattern stays the same.

### 5. Wire it up

1. Call `ncsi_init_and_register_ports()` (or your own equivalent) once at
   board init, after your MAC/PHY driver is up — see `examples/nc.c` for
   the full ordering against your own `board_init()`.
2. Call `NCSI_PollRx()` every iteration of your main loop (or from a
   dedicated task) — this is where `handleNCSIFrame()` actually runs.

### 6. Build & verify

**6a. Host-side smoke test** (no RISC-V hardware needed — run this first):

```sh
gcc -std=c11 -Wall -Wextra -Isrc/ncsi/include -Itests \
    src/ncsi/ncsi.c tests/valid_commands.c tests/test_ncsi.c -o test_ncsi
./test_ncsi
```

This feeds real captured DSP0222 command frames through the protocol
engine and checks the responses, including package/channel addressing (a
frame for the wrong package or wrong channel must produce **no**
transmission at all). All checks should print `PASS`.

For memory-safety checking beyond `-Wall -Wextra` (this is exactly how the
out-of-bounds bug below was confirmed and re-verified), add
`-fsanitize=address,undefined -g` to that same command — worth running
after any change that touches raw pointer/array/frame-buffer handling in
`ncsi.c`.

**6b. Build for your target**: compile `src/ncsi/ncsi.c`,
`ncsi_board_config.c`, and your filled-in `ncsi_hal.c` together with your
board's startup code and toolchain (`riscv32-*-gcc`). No CMake/build system
is provided beyond `src/build.mk` (a Makefile fragment — see its header
comment) — wire these three `.c` files into whatever build your project
already uses.

**6c. Bring-up against a real BMC**: once both ICs are flashed and wired
up, watch the sideband link (e.g. with a tap + Wireshark, which has an
NC-SI dissector) while the BMC probes. You should see, per channel:

```
Select Package → Clear Initial State → Enable Channel →
Get Version ID / Get Capabilities → Get Link Status →
Set MAC Address → AEN Enable
```

Confirm each IC only answers commands for **its own** channel, and that
both ICs' responses carry the correct (shared) package ID. This is also the
fastest way to confirm responses match DSP0222 field-for-field for the
content fields flagged as not independently verified below.

## How package/channel addressing works

Every frame on the shared RMII/RBT bus reaches **both** ICs (and any other
package sharing the bus). `handleNCSIFrame()` in `src/ncsi/ncsi.c` checks,
for every incoming frame:

- Does the package byte match `NCSI_BoardConfig_ReadPackageID()`'s value?
  If not: silently ignored — not our package at all.
- Is the channel byte either the package-wide indicator (`0x1F`) or does it
  match `NCSI_BoardConfig_ReadChannelID()`'s value? If not: silently
  ignored — it's the *other* chip's channel.

A responder must stay **silent** (never answer with an error) for a frame
that isn't addressed to it — both chips seeing the same frame and only one
answering is required for the protocol to work; two chips both responding
to one BMC query would break it. `ncsi.c` reads both identity values once
in `NCSI_init()` and gates every incoming frame on them
(`handleNCSIFrame()`'s `addressed_to_us` check); `gPackageState.port[0]` is
always this chip's own one local channel, never indexed by the wire channel
number. This logic already handles the two-chip design correctly — you
shouldn't need to touch `handleNCSIFrame()` itself, only the two GPIO read
functions in step 1 above.

## Why this codebase

The device/responder side of NC-SI has almost no open-source coverage —
everything else public (Linux `net/ncsi`, U-Boot `drivers/net/phy/ncsi.c`)
implements the *host/BMC* side, the opposite direction of the protocol.
`bcm5719-fw`'s `libs/NCSI/` is the one maintained open-source implementation
of the responder side found, and it's structurally separate from its
original BCM5719-specific hardware code — a usable porting base rather than
something hardwired into one ASIC.

## Supported commands

From `src/ncsi/ncsi.c`'s `gNCSIHandlers[]` dispatch table. "Stub" means the
command is answered `COMMAND_COMPLETE` but has no real effect yet (noted
`TODO` inline in the handler); "Not implemented" means it falls through to
`unknownHandler()`, which answers `COMMAND_UNSUPPORTED`. Mandatory/Optional
per DSP0222.

| Opcode | Command | DSP0222 | Status |
|---|---|---|---|
| `0x00` | Clear Initial State | Mandatory | Supported |
| `0x01` | Select Package | Mandatory | Supported |
| `0x02` | Deselect Package | Mandatory | Supported |
| `0x03` | Enable Channel | Mandatory | Supported |
| `0x04` | Disable Channel | Mandatory | Supported |
| `0x05` | Reset Channel | Mandatory | Supported |
| `0x06` | Enable Channel Network TX | Mandatory | Supported |
| `0x07` | Disable Channel Network TX | Mandatory | Supported |
| `0x08` | AEN Enable | Mandatory | Supported |
| `0x09` | Set Link | Mandatory | Supported |
| `0x0A` | Get Link Status | Mandatory | Supported |
| `0x0B` | Set VLAN Filter | Conditional (VLAN) | Stub — accepted, no filtering applied |
| `0x0C` | Enable VLAN | Conditional (VLAN) | Stub — accepted, no filtering applied |
| `0x0D` | Disable VLAN | Conditional (VLAN) | Stub — accepted, no filtering applied |
| `0x0E` | Set MAC Address | Mandatory | Supported |
| `0x10` | Enable Broadcast Filtering | Mandatory | Stub — accepted, no filtering applied |
| `0x11` | Disable Broadcast Filtering | Mandatory | Stub — accepted, no filtering applied |
| `0x12` | Enable Global Multicast Filtering | Optional | Not implemented |
| `0x13` | Disable Global Multicast Filtering | Optional | Not implemented |
| `0x14` | Set NC-SI Flow Control | Optional | Not implemented |
| `0x15` | Get Version ID | Mandatory | Supported |
| `0x16` | Get Capabilities | Mandatory | Supported |
| `0x17` | Get Parameters | **Mandatory** | **Not implemented** — see "Known gaps" |
| `0x18` | Get Controller Packet Statistics | Optional | Not implemented |
| `0x19` | Get NC-SI Statistics | Optional | Not implemented |
| `0x1A` | Get NC-SI Pass-through Statistics | Optional | Not implemented |
| `0x50` | OEM Command | Optional | Not implemented |

`0x17` Get Parameters is the one gap in this table that's DSP0222-mandatory
and genuinely missing — see "Known gaps" for why it wasn't attempted here.

## Bugs found and fixed during this port

Building the vendored code and testing it against real captured DSP0222
frames (`tests/valid_commands.c`, from upstream) surfaced three real bugs —
documented here in the order they were found, since each fix depended on
understanding the previous one.

### 1. Wire-format bit-field bug (crash on a real, valid command)

Building the vendored code against a standard little-endian GCC target and
feeding it a real captured **Select Package** request crashed: the C
bit-fields upstream uses for `ControlPacketType`/`ChannelID`/`InstanceID`
decoded to the wrong bytes entirely (`ControlPacketType` read back as `0xed`
instead of `0x01`), indexing the command-handler table out of bounds. Root
cause: bit-field allocation order within a shared storage unit is
implementation-defined in C, and it does not match wire order under a
standard GCC little-endian ABI (upstream's code was presumably only ever
exercised on their specific target compiler and a simulator that never
independently checked individual field values).

**Fix**: every field wider than one byte in `Ethernet.h` is now a plain
`uint8_t[N]` in true wire (MSB-first) order — read/write it via
`ncsi_rd16()`/`ncsi_wr16()`/`ncsi_rd32()`/`ncsi_wr32()` (`types.h`), never by
casting to an integer directly. **This is very likely to bite you too if
you build the pristine `third_party/ncsi/` copy directly on RISC-V32 GCC —
use `src/ncsi/` instead.**

Verified against `tests/valid_commands.c`'s real captures:
- Command dispatch (`ChannelID`/`ControlPacketType`/`InstanceID`/
  `PayloadLength`) — all 14 captured commands now dispatch and respond
  correctly (see `tests/test_ncsi.c`).
- `AENEnable_t` — decodes to `AENControl = 0x7`, a plausible
  "enable the standard AEN set" bitmask.
- `SetMACAddr_t` — decodes to MAC `2c:09:4d:00:01:4a`, filter slot 0; a
  well-formed result that would be an implausible coincidence if the byte
  mapping were wrong.

**Not independently verified this way** (no real captures exist for these):
`SetLink_t`'s `LinkSettings`/`OEMLinkSettings`, and the *content* fields of
the Get Capabilities / Get Version ID / Get Link Status responses
(`CapabilitiesResponsePacket_t`, `VersionResponsePacket_t`,
`LinkStatusResponsePacketHeader_t`). These follow the same verified pattern
(reversed declared order, explicit big-endian reconstruction) but should be
re-checked against DSP0222 and/or a real BMC exchange (step 6c above)
before shipping. `getLinkStatusHandler()`'s Link Status bit composition in
`ncsi.c` is explicitly flagged inline as an incomplete placeholder
(DSP0222's real "Link Status" field has a 3-bit speed/duplex enumeration
this only approximates with one bit).

### 2. Response `ChannelID` silently dropping the package bits

Turned up once package IDs stopped being hardcoded to 0:
`getCapabilities()`/`getVersionID()` built their response's `ChannelID`
field from the *masked* channel number
(`frame->controlPacket.ChannelID & CHANNEL_ID_MASK`) instead of the full
byte, silently dropping the package bits. Invisible while every capture was
package 0 (masking off zero bits changes nothing); a real bug once package
IDs vary — the response would carry the wrong package and a real BMC would
very likely reject or misroute it.

**Fix**: echo `frame->controlPacket.ChannelID` verbatim, matching how every
other response builder in this file already did it.

### 3. Out-of-bounds command-handler-table read (memory safety, not just a wrong value)

`handleNCSIFrame()` indexed `gNCSIHandlers[command]` with no bounds check.
`command` is a full `uint8_t` straight off the wire (0–255), but the array
is only sized to its highest designated-initializer index (`0x1A` → 27
elements) — a single OEM Command (`0x50`) frame, a genuinely valid DSP0222
opcode (not malformed input), indexed 8 bytes past the end of the array,
reading whatever bytes happened to sit there as a fabricated
`ncsi_handler_t` and calling through its garbage `fn` pointer. Confirmed
with AddressSanitizer (`global-buffer-overflow`, `READ of size 8` at the
`handler->fn` dereference) before the fix — this wasn't theoretical. This
bug predates this port; upstream's code has the identical unchecked-index
pattern.

**Fix**: bounds-check `command` against the array's real size
(`ARRAY_ELEMENTS(gNCSIHandlers)`) before indexing it, falling through to the
existing `COMMAND_UNSUPPORTED` path for anything out of range — same
behavior as any other unimplemented command, just no longer able to read
out of bounds to get there. Regression test in `tests/test_ncsi.c`.

## Known gaps

- VLAN handling (Set/Enable/Disable VLAN) is a stub — accepts the command,
  applies no actual filtering.
- Get Parameters (`0x17`) is DSP0222-**mandatory** but not implemented —
  the one gap here that shouldn't ship as-is. Needs its own response struct
  in `Ethernet.h` (not modeled yet) and a real DSP0222 copy to get the byte
  layout right; follow the same capture-verify approach used for the rest
  of this port (see "Bugs found and fixed") rather than guessing.
- OEM Command (`0x50`) and a handful of truly optional commands (statistics
  queries, flow control, global multicast filtering) not implemented.
- Get Capabilities / Get Version ID / Get Link Status response *content*
  fields follow the verified byte-layout pattern but aren't independently
  checked against a real capture — needs a real BMC/Wireshark pass (step
  6c above).
- GPIO pin(s) for Package ID are unspecified — `NCSI_BoardConfig_ReadPackageID()`
  is a `TODO` stub (Channel ID/GPIO20 is filled in with the exact intended
  logic, just needs the real register call).
- `NCSI_RX_DESCRIPTOR`/`ncsi_rx_descriptor_t` and `NCSI_TX_DESCRIPTOR`/
  `ncsi_tx_descriptor_t`/`NCSI_TX_POLL_BIT_REG` in `ncsi_hal_template.c` are
  placeholders — real descriptor/doorbell register layout not yet known.

## License

`third_party/ncsi/` and the parts of `src/ncsi/` derived from it are
BSD-3-Clause, Copyright (c) 2018-2019 Evan Lojewski (see
`third_party/ncsi/LICENSE`). Keep that notice on any file carrying it. New
files written for this port (`Network.h`, `ncsi_board_config.c`/`.h`,
`ncsi_hal_template.c`, `examples/nc.c`, `src/build.mk`,
`tests/test_ncsi.c`, this README) carry no such restriction.
