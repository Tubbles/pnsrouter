# pnsrouter: instructions for LLM agents

## What this is

A Rust library implementing an interactive push and shove PCB router, ported at the design level from KiCad's PNS router. First consumer: LibrePCB (local checkout at `~/dev/librepcb`, a fork of LibrePCB/LibrePCB with `upstream` remote). Read PLAN.md and DESIGN.md before touching code.

## Reference sources on this machine

- KiCad, sparse shallow clone: `~/dev/ref/kicad` (pcbnew/router, libs/kimath, qa/tools/pns, qa/tests/pcbnew, qa/data/pcbnew/pns_regressions). Commit 302b2ba1014b2f116ab38d69ffa8c6d1c633ed85 (2026-09-07).
- Horizon EDA, shallow clone: `~/dev/ref/horizon`. `3rd_party/router` is their vendored KiCad 6.0.4 router, `src/router/pns_horizon_iface.cpp` is their glue.
- rnestler/pns-router, the 2018 C++ extraction attempt: `~/dev/ref/pns-router`.
- Digested notes with file:line citations: `doc/reference/kicad/`. Read the relevant note before porting a module; they list dead code not to port and behaviours to reproduce deliberately.

## Conventions

- Formatting: rustfmt with the repository's `rustfmt.toml` (2 spaces, 80 columns, matching LibrePCB's rust-core).
- `#![forbid(unsafe_code)]`, `#![warn(missing_docs)]`, clippy clean with `--all-targets --features fail-on-warnings -- -D warnings`.
- Coordinates are i32 nanometres, products of coordinates are i64, see DESIGN.md. Do not use f64 where KiCad uses integers.
- Determinism: never iterate a HashMap or HashSet, never read the wall clock inside an algorithm, tie-break on (distance, item uid). See DESIGN.md.
- When porting a KiCad routine, cite the KiCad file:line in a doc comment. Record deliberate deviations from KiCad behaviour in `doc/log/`.
- Tests: unit tests next to the code, scenario tests under `tests/`. Mirror KiCad's qa tests where they exist (see doc/reference/kicad/01-geometry.md section 15).
- Commits: imperative title of at most 72 characters, body wrapped at 72 columns, `Assisted-by: Claude:<model-id>` trailer, no dashes as prose punctuation.

## Documents

- `PLAN.md`: milestones. `DESIGN.md`: architecture. `TODO.md`: user inbox. `SUGGESTIONS.md`: ideas found during work.
- `doc/log/YYYY-MM-DD.md`: decision log, write once. `doc/work/`: work items with status (todo, in progress, implemented, verified).
- `work/` and `tmp/` are not committed.

## Commands

    cargo build
    cargo test
    cargo clippy --all-targets --features fail-on-warnings -- -D warnings
    cargo fmt --all --check
    cargo doc --no-deps --document-private-items
