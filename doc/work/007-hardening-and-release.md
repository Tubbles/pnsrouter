# 007 Hardening and release

Status: todo

## Goal

Confidence and a first crates.io release.

## Tasks

- [x] Fixtures from real LibrePCB boards through the recorder. LibrePCB writes one file per routing session when `LIBREPCB_PNS_RECORD_DIR` names an existing directory, in this crate's own recorded session format. See [../librepcb-integration.md](../librepcb-integration.md) section 3.1.1 and the note there on picking a resolver for the replay.
- [ ] Fuzzing of line chain, collision and hull code (cargo-fuzz), property tests for geometry invariants.
- [x] Performance profiling on a large board, budget tuning. See [../performance.md](../performance.md): the `examples/latency.rs` harness, the numbers, the profile and what the shove budget buys.
- [ ] `cargo doc` clean, README examples, CHANGELOG.
- [ ] 0.1.0 on crates.io.

## Acceptance

Release published, CI green on stable and MSRV.
