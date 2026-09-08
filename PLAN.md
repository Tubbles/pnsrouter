# Plan

## End goal

pnsrouter is a standalone Rust crate on crates.io implementing an interactive push and shove router with the behaviour of KiCad's PNS router, and LibrePCB uses it for interactive trace routing in its board editor. Urban Bruhin (LibrePCB maintainer) stated in June 2025 that he would take care of the LibrePCB integration if somebody sets up and maintains such a library, and that a Rust crate is preferred (https://librepcb.discourse.group/t/13/26 and https://github.com/LibrePCB/LibrePCB/issues/1537). Until then the integration is developed on the user's LibrePCB fork.

## Non-goals for now

Differential pairs, length tuning (meanders), multi drag, component drag, arc tracks, HDI micro via stacks, a custom DRC rule language. The data model must not prevent them, but no code is written for them.

## Milestones

- M0 Scaffold (done 2026-09-07): repository, license, crate skeleton, CI, design and plan, KiCad reference notes.
- M1 Geometry foundation: Vec2, Seg, Box2, Direction45, LineChain, Shape enum, collision dispatch, hulls, convex hull, integer math helpers. Tests mirror KiCad's `qa/tests/libs/kimath` where applicable. Exit criterion: every routine used by the router core per doc/reference/kicad/01-geometry.md section 1 exists with tests.
- M2 World model: items with generational ids, layer ranges, nets, spatial index, joints, Node with branch and commit, line assembly, nearest obstacle, Line walkaround around a single hull, RuleResolver trait with a fixed clearance implementation. Exit criterion: a world can be built from plain data, queried, branched and committed, with scenario tests for joints and line assembly.
- M3 Walkaround router: line placer state machine, mouse trail posture solver, mark obstacles and walkaround modes, optimizer merge passes, fixed tail (undo last segment), via placement and layer switch. Exit criterion: headless scenario tests route between two pads around obstacles on a two layer board and commit segments and vias.
- M4 Shove: rank based shove loop, springback stack, segment, solid and via handlers, shove mode in the placer. Exit criterion: scenario tests where existing traces get pushed aside and spring back on retreat.
- M5 Session API and event log: the facade used by hosts (start, move, fix, undo, switch layer, toggle via, finish, stop, preview, commit result), the debug decorator trait, the event log recorder and replay for regression fixtures.
- M6 LibrePCB integration: Cargo dependency in `libs/librepcb/rust-core`, FFI in rust-core, a new board editor tool state next to the existing draw trace tool, Slint toolbar, undo stack commit. Exit criterion: manual routing in LibrePCB on a real project in all three modes.
- M7 Hardening and release: fixtures from real boards, fuzzing of geometry, performance profiling on large boards, 0.1.0 release on crates.io.

Later: dragging of existing traces and vias (M8), differential pairs, tuning, arcs.

## Work tracking

`doc/work/` holds one file per work item with its status. `doc/log/` records decisions.
