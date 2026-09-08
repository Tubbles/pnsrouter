# pnsrouter

Interactive push and shove PCB router library, written in Rust. The design follows KiCad's PNS router (`pcbnew/router` in the KiCad source tree), reimplemented as a standalone library without GUI or board file format dependencies, so that other EDA tools can embed it. The first consumer is [LibrePCB](https://librepcb.org).

## Status

Pre-alpha. The repository contains the project plan, the design and reference notes on the KiCad implementation. There is no routing code yet.

## Scope of the first milestone

Interactive single net trace routing with the three interaction modes of the KiCad router (highlight collisions, walk around, shove), via placement with layer switching, 45 degree posture handling and the post route optimizer. Dragging existing traces, differential pairs, length tuning and arc tracks come later. See [PLAN.md](PLAN.md).

## Relationship to KiCad

pnsrouter is a design level port of KiCad's PNS router, which is copyright CERN and the KiCad developers and licensed GPL-3.0-or-later. pnsrouter is therefore also licensed GPL-3.0-or-later. The reference notes under [doc/reference/kicad/](doc/reference/kicad/) cite the KiCad sources by file and line, at KiCad master commit 302b2ba1014b2f116ab38d69ffa8c6d1c633ed85.

## Building

    cargo build
    cargo test
    cargo clippy --all-targets --features fail-on-warnings -- -D warnings
    cargo doc --no-deps --open

Minimum supported Rust version: 1.92, the version pinned by LibrePCB's CI.

## Documentation

- [PLAN.md](PLAN.md): milestones and the end goal.
- [DESIGN.md](DESIGN.md): crate architecture and the decisions behind it.
- [doc/](doc/): decision log, work items and KiCad reference notes.

## License

GPL-3.0-or-later, see [LICENSE](LICENSE).
