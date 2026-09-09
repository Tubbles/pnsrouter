# pnsrouter

Interactive push and shove PCB router library, written in Rust. The design follows KiCad's PNS router (`pcbnew/router` in the KiCad source tree), reimplemented as a standalone library without GUI or board file format dependencies, so that other EDA tools can embed it. The first consumer is [LibrePCB](https://librepcb.org).

## Status

Pre-alpha. Milestones 1 to 6 of [PLAN.md](PLAN.md) are done: the crate routes in all three modes (highlight collisions, walk around, shove) with via placement, layer switching, undo and the optimizer, exposes a host facing session API (`router::Router`) fed by a plain data board snapshot and answering with preview frames and commit diffs, and records sessions for replay. The LibrePCB integration is built and manually tested on a real project, but it lives on a fork branch and is not upstream, see [doc/librepcb-integration.md](doc/librepcb-integration.md). Milestone 7, hardening and the first crates.io release, is in progress, so there is no released version yet. Every ported routine cites its KiCad origin; tolerance sensitive routines were diffed against compiled transcriptions of the C++.

## Using the crate

A host describes its board as plain data, drives a session from the mouse, draws the preview frame every event answers with, and applies the commit diff the session ends on. The `pad` helper below is the only thing elided; the crate documentation has it spelled out.

```rust
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::HostId;
use pnsrouter::node::World;
use pnsrouter::router::{FixOutcome, Router};
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RoutingSettings, Sizes};
use pnsrouter::snapshot::WorldSnapshot;

// Every copper object on the board, carrying the host's own object ids.
let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);
snapshot.items.push(pad(HostId(1), Vec2::new(0, 0)));
snapshot.items.push(pad(HostId(2), Vec2::new(4_000_000, 0)));

let mut router = Router::new(
  &snapshot,
  Box::new(FixedClearance::uniform(100_000)),
  RoutingSettings::default(),
  Sizes::default(),
);

// Press on the first pad, on layer 0.
router
  .start_routing(Vec2::new(0, 0), Some(HostId(1)), 0)
  .expect("the pad is routable");

// One call per mouse motion. The frame is everything the host draws, and
// it replaces the previous one whole.
let frame = router.move_to(Vec2::new(4_000_000, 0), Some(HostId(2)));

// Click on the second pad, forcing the placement to finish there.
if let FixOutcome::Finished(diff) =
  router.fix_route(Vec2::new(4_000_000, 0), Some(HostId(2)), true)
{
  // Apply diff.removed, diff.added and diff.updated in one undo
  // transaction, then report the ids assigned back with
  // Router::assign_host_ids.
}
```

The crate documentation has the whole thing as a compiling example, the four things a host has to provide, and a map of the modules: `cargo doc --no-deps --open`, or [docs.rs/pnsrouter](https://docs.rs/pnsrouter) once 0.1.0 is published.

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

- [CHANGELOG.md](CHANGELOG.md): what each release contains, and what it leaves out.
- [PLAN.md](PLAN.md): milestones and the end goal.
- [DESIGN.md](DESIGN.md): crate architecture and the decisions behind it.
- [doc/](doc/): decision log, work items and KiCad reference notes.

## License

GPL-3.0-or-later, see [LICENSE](LICENSE).
