# Changelog

All notable changes to this crate are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the crate follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

The decision behind any entry is in [doc/log/](doc/log/), one file per day, and the work item it belongs to is in [doc/work/](doc/work/).

## [Unreleased]

### Added

- **Parallel obstacle query**, KiCad's `NODE::NearestObstacle` thread pool (`pcbnew/router/pns_node.cpp:437`) as one `std::thread::scope` per query. `node::World::set_parallelism` and `router::Router::set_parallelism` are the knob; the reduction stays sequential and ranks by (distance, uid), so an answer never depends on the thread count. **The default is 1**: measured on both boards of `doc/performance.md`, spawning threads per query cost more than the geometry it moved. The knob is on the world and not on `settings::RoutingSettings`, so a recorded session's format is unchanged.
- `eventlog::replay_with_parallelism`, for replaying a recording at a chosen thread count.
- **Dragging** (milestone 9): `router::Router::start_dragging` starts a segment, corner or via drag on one object, `move_to` and `fix_route` drive it, and `eventlog::SessionEvent::StartDragging` records it. All three routing modes drag a segment, a corner or a via: highlight collisions, walk around, and shove, the last driving the shove engine with the dragged line or the dragged via as its head. A via takes its fanout with it, pushed clear by `via::via_pushout_force` along the reversed mouse trail. Free angle mode, which `start_dragging` takes as a flag, forces a corner drag that follows the cursor exactly and never walks or shoves. `dragger::Dragger::traces_vias` answers the via half of what a drag is moving. `rules::RuleResolver` is unchanged.
- **`optimizer::EffortFlags::REQUIRE_OBTUSE_ANGLES`**, KiCad's drag only pass (`pcbnew/router/pns_optimizer.h:110`): it adds `optimizer::Constraint::ObtuseOnly`, which refuses any replacement making a right, acute or reversed corner, and runs `optimizer::Optimizer::drag_fix_corners` before every other pass, which replaces the bad corner a drag anchored on with the 45 degree bypass enclosing the least area. A drag asks for it when `settings::RoutingSettings::restrict_angles` is set.
- `mouse_trail::MouseTrailTracer::trail_lead_vector`, the straight line from a gesture's first trail point to its last, which the via drag's force propagation pushes against.
- **Multi drag** (milestone 9): `multi_dragger::MultiDragger`, KiCad's `MULTI_DRAGGER`. Several traces are dragged as a bundle in any of the three modes: the line under the cursor follows it and every other line keeps the perpendicular offset it had, with mark obstacles clipping the lines against each other, walkaround walking the set in both orders and keeping the cheaper one, and shove making every line a head in drag distance order. `router::Router::start_dragging` now picks the algorithm from the shape of the item set the way KiCad does, so more than one segment is a multi drag and a set of nothing but pads is a component drag; `router::StartError::MultiDragUnsupported` is gone. `router::Router::last_committed_leader_segments` and `router::Router::host_of` are how a host puts the user's selection back on the segments the router made. KiCad's corner mode line order is unspecified (its sort key is never assigned there and `std::sort` is unstable), so this port ties on the order the host listed the selection in; that is the only deliberate difference.
- `geometry::line_chain::LineChain::point_along`, the point a given distance along a chain, which the multi drag's line against line clipping searches with. It takes an `i64` where KiCad takes an `int`, so a chain longer than 2.1 metres cannot overflow it.
- **Component drag** (milestone 9): `component_dragger::ComponentDragger`, KiCad's `COMPONENT_DRAGGER`. A set of nothing but pads drags the footprints they belong to: each pad is cloned at the cursor offset, anything running between two dragged pads translates rigidly, and every other attached trace has the corner on its pad dragged to where that pad went. `router::CommitDiff::moved_solids` and `router::PreviewFrame::moved_solids` are how a moved pad reaches the host, one `(host object, offset)` pair each, because KiCad never deletes and re-creates a pad either: `PNS_KICAD_IFACE` intercepts the removal and the addition and moves the pad's footprint instead. `router::RouterState::DragComponent` is the state, and `router::Router::pending_update` reports its delta where KiCad's `GetUpdatedItems` has no branch for it and answers nothing. There is no shove, no walkaround and no optimizer in a component drag: KiCad's does not read the routing mode at all, and neither does this. `router::StartError::ComponentDragUnsupported` is gone with it, since every shape of a non empty item set now has an algorithm; the LibrePCB fork's `rust-core` names that variant and needs the rename at its next submodule bump.
- **Differential pair value layer** (milestone 10, first slice): `diff_pair::DiffPair`, two coupled line chains with their nets, width and gap, measuring `coupled_segment_pairs`, `coupled_length` and `skew`; `diff_pair::DpGateway`, `diff_pair::DpPrimitivePair` and `diff_pair::DpGateways`, the anchor pairs a pair route leaves a pad pair, a via pair, two existing tracks or a bare cursor through, with every KiCad builder and its push order, which is part of the route it picks; and `diff_pair::fit_gateways`, which joins two gateway sets into the two lanes of a route. `diff_pair::GapConstraint` is KiCad's `RANGED_NUM`, and `settings::Sizes::diff_pair_pitch` names the centre to centre spacing so that it cannot be confused with the copper gap, which is the trap `DIFF_PAIR::m_gap` sets by meaning both. Nothing here is a placer yet: `router::Router` is unchanged and a host sees no new session API.
- Ten of the eleven KiCad regression cases now reach KiCad's own counts of added and removed items, up from four, and all eleven leave nothing colliding. The one that misses is a via drag whose recorded golden is empty, which `tests/kicad_replay.rs` documents.

## [0.1.0] - unreleased

First release. A headless interactive router for single net traces, driven entirely through `router::Router`.

### Added

- **Three routing modes**, KiCad's own: highlight collisions, walk around and shove, selected with `settings::RouterMode`.
- **Walkaround**: the head is bent around whatever is in its way, over the hulls of the obstacles rather than over their shapes.
- **Shove**: a rank based loop with a springback stack, handling colliding segments, solids and vias, including pushed vias, fanout drag, the lone via and the tadpole patch. The iteration budget is a state count, never a wall clock.
- **Vias and layer switching**: place a via at the head, toggle it, switch layer, with the layer span taken from the layer pairs a host puts in `settings::Sizes`.
- **Posture**: the 45 degree posture solver from the mouse trail, so the head leaves a corner the way the cursor arrived at it, with a manual flip.
- **Optimizer**: the merge passes and the pad breakout passes run over a committed line, with the effort selectable in `settings::RoutingSettings`.
- **Undo of the last segment** while a placement is running, and a fixed tail behind it.
- **Session facade** `router::Router`: hover, start, move, fix, undo, switch layer, toggle via, flip posture, corner and ortho modes, continue from the end, finish, stop and abort. Every event answers with a whole `PreviewFrame` to draw, and a session ends on a `CommitDiff` of removed, added and updated objects for the host to apply in one undo transaction.
- **Plain data board snapshot** `snapshot::WorldSnapshot`: pads and copper graphics, tracks, vias and holes, with the host's own object ids, so the engine needs no board file format and no callbacks into the host.
- **Rule resolver** `rules::RuleResolver`, the one trait a host implements, with `rules::FixedClearance` for a board that has a single clearance.
- **Recorded session format** `eventlog`: a session records its snapshot, settings, sizes, events and resulting commit as text, and replays against a fresh router. It is how a regression or a host bug report travels, and it is what pins the engine as a pure function of (snapshot, settings, events).
- **KiCad regression replay**: four of the eleven cases of KiCad's `qa/data/pcbnew/pns_regressions` corpus are replayed against this engine and reach KiCad's own counts of added and removed items. The other seven open with a drag event and wait for the dragger.
- **Latency harness** `examples/latency.rs`, which builds a synthetic board of a given size and prints the distribution of per call latency in each mode. The numbers and the profile behind them are in [doc/performance.md](doc/performance.md).
- **Debug hook** `debug::DebugDecorator`, for a host that wants to draw an algorithm's internals.

### Not in this release

Deliberately out of scope, and no code exists for any of them:

- Dragging existing traces and vias, component drag and multi drag.
- Differential pairs.
- Length tuning and meanders.
- Arc tracks. The geometry is polygonal throughout.
- A rule expression language. Clearances come from the host's `rules::RuleResolver` and nothing else; a per pair or conditional rule is the host's to evaluate.

### Notes

- Minimum supported Rust version: 1.92, the version LibrePCB's CI pins. It is treated as part of the public API, so raising it is a breaking change.
- Coordinates are `i32` nanometres, which covers about plus or minus 2.1 metres. No routine in the crate uses floating point where KiCad uses integers.
- The engine is deterministic: it never reads the wall clock, never iterates a hash container, and ties break on item uid.
- No `unsafe` code (`#![forbid(unsafe_code)]`).
- KiCad's regression corpus and the KiCad reference notes are in the git repository but not in the published package, along with the two integration tests that read the corpus. `cargo test` on the published package runs everything that is left.

[Unreleased]: https://github.com/Tubbles/pnsrouter/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/Tubbles/pnsrouter/releases/tag/v0.1.0
