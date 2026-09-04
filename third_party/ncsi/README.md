# Vendored: bcm5719-fw `libs/NCSI`

Source: https://github.com/meklort/bcm5719-fw, `libs/NCSI/` subtree, fetched
2026-09-03 from the `main` branch.

This is the NC-SI (DSP0222) protocol-handler subset of a clean-room
reimplementation of Broadcom BCM5719 NIC firmware. It's kept here unmodified,
as the original license (`LICENSE`, BSD-3-Clause) requires, and as a
reference to diff the ported version in `src/ncsi/` against upstream.

**Do not build against this copy directly** — it depends on BCM5719-specific
headers (`APE.h`, `APE_APE_PERI.h`, `MII.h`, etc.) that aren't vendored here
and won't exist on any other target. The actual RISC-V32 port lives in
`src/ncsi/` (see the top-level README for what changed and why).
