# 007 Hardening and release

Status: todo

## Goal

Confidence and a first crates.io release.

## Tasks

- [ ] Fixtures from real LibrePCB boards through the recorder.
- [ ] Fuzzing of line chain, collision and hull code (cargo-fuzz), property tests for geometry invariants.
- [x] Performance profiling on a large board, budget tuning. See [../performance.md](../performance.md): the `examples/latency.rs` harness, the numbers, the profile and what the shove budget buys.
- [ ] `cargo doc` clean, README examples, CHANGELOG.
- [ ] 0.1.0 on crates.io.

## Acceptance

Release published, CI green on stable and MSRV.
