# Plan

## End goal

pnsrouter is a standalone Rust crate on crates.io implementing an interactive push and shove router with the behaviour of KiCad's PNS router, and LibrePCB uses it for interactive trace routing in its board editor. Urban Bruhin (LibrePCB maintainer) stated in June 2025 that he would take care of the LibrePCB integration if somebody sets up and maintains such a library, and that a Rust crate is preferred (https://librepcb.discourse.group/t/13/26 and https://github.com/LibrePCB/LibrePCB/issues/1537). Until then the integration is developed on the user's LibrePCB fork.

## Non-goals for now

Differential pairs, length tuning (meanders), multi drag, component drag, arc tracks, HDI micro via stacks, a custom DRC rule language. The data model must not prevent them, but no code is written for them.

## Milestones

- M0 Scaffold (done 2026-09-07): repository, license, crate skeleton, CI, design and plan, KiCad reference notes.
- M1 Geometry foundation (implemented 2026-09-08): Vec2, Seg, Box2, Direction45, LineChain, Shape enum, collision dispatch, hulls, convex hull, integer math helpers. Tests mirror KiCad's `qa/tests/libs/kimath` where applicable. Exit criterion: every routine used by the router core per doc/reference/kicad/01-geometry.md section 1 exists with tests.
- M2 World model (implemented 2026-09-08): items with generational ids, layer ranges, nets, spatial index, joints, Node with branch and commit, line assembly, nearest obstacle, Line walkaround around a single hull, RuleResolver trait with a fixed clearance implementation. Exit criterion: a world can be built from plain data, queried, branched and committed, with scenario tests for joints and line assembly.
- M3 Walkaround router (implemented 2026-09-08): line placer state machine, mouse trail posture solver, mark obstacles and walkaround modes, optimizer merge passes, fixed tail (undo last segment), via placement and layer switch. Exit criterion: headless scenario tests route between two pads around obstacles on a two layer board and commit segments and vias.
- M4 Shove (implemented 2026-09-09): rank based shove loop, springback stack, segment, solid and via handlers, shove mode in the placer. Exit criterion: scenario tests where existing traces get pushed aside and spring back on retreat.
- M5 Session API and event log (implemented 2026-09-09): the facade used by hosts (start, move, fix, undo, switch layer, toggle via, finish, stop, preview, commit result), the debug decorator trait, the event log recorder and replay for regression fixtures.
- M6 LibrePCB integration (built 2026-09-09, steps 1 to 8 of `doc/librepcb-integration.md` on the fork's `pns-router` branch): Cargo dependency in `libs/librepcb/rust-core`, FFI in rust-core, a new board editor tool state next to the existing draw trace tool, Slint toolbar, undo stack commit. Exit criterion met on 2026-09-09: the user's manual test on a real project found no issues (limited coverage, see doc/work/006).
- M7 Hardening and release: fixtures from real boards, fuzzing of geometry, performance profiling on large boards, 0.1.0 release on crates.io.

- M8 Parallel obstacle query: KiCad's thread pool in `NODE::NearestObstacle` (`pns_node.cpp:437`) ported with `std::thread::scope`, sequential below a candidate count threshold, reduction by (distance, uid) so every answer is identical on any thread count. Exit criterion: the replay fixtures and the KiCad goldens are byte identical with and without threads, and `examples/latency.rs` shows the gain on the 20 000 segment board.
- M9 Dragging: segment, corner and via drag with the shove engine, multi drag, and the helpers deferred for them (rotation, angles, `PointAlong`, line versus line decomposition at the dragger's call sites). Exit criterion: the seven drag cases of the KiCad corpus replay to their goldens, and LibrePCB drags a trace with the select tool.
- M10 Differential pairs: the pair placer, coupling and gap rules, the pair dragger, and the host hooks (`dp_net_pair` and friends) filled in for LibrePCB through a naming convention. Exit criterion: a pair routes in all three modes on a synthetic two layer board in the crate's tests. The LibrePCB side is on hold until LibrePCB has a pair concept (user decision, 2026-09-10).
- M11 Meanders: single trace length tuning, differential pair length tuning and skew tuning, chamfered corners only until arcs exist. Exit criterion: a tuned trace reaches the target length within the tolerance and replays.

Later, on hold by user decision (2026-09-10): arcs (they touch the geometry core everywhere and unlock KiCad's rounded corner modes and rounded meanders; LibrePCB traces have no arcs and upstream rates curved traces low priority) and the LibrePCB side of differential pairs.

## Work tracking

`doc/work/` holds one file per work item with its status. `doc/log/` records decisions.
