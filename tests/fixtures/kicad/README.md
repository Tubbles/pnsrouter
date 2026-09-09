# KiCad PNS regression fixtures

`pns_regressions/` is a verbatim copy of `qa/data/pcbnew/pns_regressions/` from KiCad, taken at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85`. Nothing in it has been edited, renamed or reformatted.

Upstream: <https://gitlab.com/kicad/code/kicad>

## Licence

The files are part of KiCad and are licensed **GPL-3.0-or-later**, the same licence this crate uses. See the `LICENSE` file at the repository root for the full text.

## What is in it

`boards/` is the shared board pool, ten `.kicad_pcb` files. It is not a test case; KiCad's harness skips it by name and hashes its contents into a lookup table (`qa/tools/pns/qa_pns_regressions_main.cpp:195`).

The eleven sibling directories are the cases. Each holds a `.log` (the recorded input events plus the golden commit result), a `.settings` (the router settings as JSON), usually a `.kicad_pro`, and in two cases extra files: `walk-with-teardrops` ships its own board as `pns-no-hug-2.dump`, and `issue24132-shove-same-net-via` ships a `pns.kicad_dru` with a physical clearance rule.

A case names its board by content hash rather than by file name, so a case cannot be matched to a board without KiCad's `IO_UTILS::fileHashMMH3`.

## How this crate uses it

`tests/support/` reads the boards and the logs into a neutral intermediate representation and `tests/kicad_fixtures.rs` pins what is in them. Replaying the cases against the router is a later step, tracked in `doc/work/005-session-api-and-event-log.md`.
