# 009 Dragging

Status: in progress (started 2026-09-10; the reference note, all three drag routines for a segment, a corner and a via, free angle mode and the session facade are in; multi drag and the LibrePCB side are not)

## Goal

Milestone 9: drag existing segments, corners and vias with the shove engine, as KiCad's `DRAGGER` and `MULTI_DRAGGER` do.

## Tasks

- [x] Reference note `doc/reference/kicad/06-dragger.md` (2026-09-10). It found the dragger needs no `EDA_ANGLE` outside arcs and no `RotatePoint`; `PointAlong` and line versus line collisions are multi drag only.
- [x] `Line::drag_segment` with the neighbour snapper, `via_pushout_force` lifted into `src/via.rs`, and `src/dragger.rs` with `start`, the mode decision, `drag` and mark obstacles mode (2026-09-10). Walkaround, shove and via drag are stubs.
- [x] Session facade: `Router::start_dragging`, the drag branches of `move_to`, `fix_route`, `stop_routing`, `abort_routing` and `pending_update`, `SessionEvent::StartDragging` recorded, serialised and replayed, and `EventKind::StartDrag` supported in the KiCad log player (2026-09-10). `optimize_and_update_dragged_line`, `best_anchor_for_point` and `point_has_bad_corner` landed with it; note 06 section 2.14 claims mark obstacles calls the first of them and it does not, so it is unreachable (and carries an `#[expect(dead_code)]`) until the walkaround drag arrives.
- [x] Segment and corner drag in walkaround and shove mode (2026-09-10). `dragWalkaround` and `tryWalkaround` with its length limit factor of 30.0, `dragShove` driving the existing `Shove` through the head protocol alone, and the optimizer's drag only passes behind them: `EffortFlags::REQUIRE_OBTUSE_ANGLES`, `Constraint::ObtuseOnly` and `Optimizer::drag_fix_corners`, which `optimize_and_update_dragged_line` asks for when `restrict_angles` is set. `optimize_and_update_dragged_line` is reachable at last, so its `#[expect(dead_code)]` is gone.
- [x] Via drag (2026-09-10): `findViaFanoutByHandle`, `dragViaMarkObstacles`, `propagateViaForces` over `via_pushout_force`, `dragViaWalkaround` and `dragShove`'s `DM_VIA` case, plus `MouseTrailTracer::trail_lead_vector`, which the force propagation negates. `Traces()` is two vectors here, `Dragger::traces` and `Dragger::traces_vias`, cleared as one. Errata E5, E6, E10, E11 and E13 are transcribed with a comment each; E5 is pinned by a test rather than repaired.
- [x] Free angle mode (2026-09-10). It needed no new code: `startDragSegment`'s second case, `Drag`'s bypass and `Line::drag_corner`'s free angle branch all landed in earlier slices, so this is one end to end test that also pins the forced corner mode and that no shove is built.
- [ ] Multi drag.
- [x] The seven drag cases of the KiCad corpus replay in `tests/kicad_replay.rs` (2026-09-10). All seven leave nothing colliding and six of the seven match their golden. The two harness gaps that held the last two back are closed, and the one case that misses now, `issue23449`, misses because the via drag works.
- [ ] LibrePCB: drag from the select tool through `BoardPnsRouter`, one undo entry per drag.

## Where the seven corpus cases stand

Measured through `Router::pending_update` after the last recorded move, which is what KiCad's harness compares (`pcbnew/router/pns_router.cpp:844`); none of the seven logs holds an `EVT_FIX`. Every one of them replays without a panic, keeps a session running and leaves nothing colliding.

| Case | Mode | Added | Golden added | Removed | Golden removed | Colliding segments left |
| --- | --- | --- | --- | --- | --- | --- |
| `drag-acute-fallback` | walkaround | 11 | 11 | 6 | 6 | 0 |
| `drag-walk-optimize-a` | walkaround | 9 | 9 | 6 | 6 | 0 |
| `drag-walk-optimize-fix-corners` | walkaround | 9 | 9 | 3 | 3 | 0 |
| `walk-with-teardrops` | walkaround | 17 | 17 | 19 | 19 | 0 |
| `simple-drag-shove-singlelayer` | shove | 134 | 134 | 139 | 139 | 0 |
| `walk_drag_seg_against_board_edge` | shove | 3 | 3 | 3 | 3 | 0 |
| `issue23449-shove-lone-via-drag-crash` | shove | 2 | 0 | 1 | 0 | 0 |

Six of the seven goldens agree and their tier 2 tests run, and every collision test runs. The four routing cases are unchanged at their goldens: `backspace1` 2 added and 0 removed, `issue22749` 11 and 5, `issue24132` 4 and 1, `simple-shove-1` 28 and 13.

`drag-acute-fallback` and `walk-with-teardrops` moved to their goldens when the two harness gaps of slice 3 were closed, and neither fix moved any other case:

- the rounded rectangle pad `drag-acute-fallback` detours around is polygonised now, so its hull's diagonals cut the corner at 45 degrees the way `PNS::ConvexHull` does over the polygon `syncPad` builds, and the detour reaches the drag anchor in two segments as KiCad's does rather than in one;
- `KicadRules::clearance_epsilon` answers KiCad's 500 nm rather than zero, so the pair of `walk-with-teardrops` tracks that sit at exactly the net class clearance no longer collides and the drag stops walking round the board outline.

`issue23449-shove-lone-via-drag-crash` is the one that misses, and it misses because the via drag works. Its golden is empty on all three counts, `addedItems`, `removedItems` and `headItems`, and it matched only while a via drag moved nothing; note 06 section 10.2 step 9 said in advance that it would count at tier 2 "only if the crate's drag also ends with an empty delta". All 102 moves succeed here, the shove answering `Ok` with the head via moved, and the last cursor position `(142.4, 80.0)` leaves the via at `(142.352108, 79.897157)`, clear of everything under the case's own rules. An empty delta on KiCad's side is a failure shape: either the last `Drag` failed and the restore re-branched a clean node (`pcbnew/router/pns_dragger.cpp:1039`, where `m_lastDragSolution` is default constructed in `DM_VIA` so the restore adds nothing), or `dragViaWalkaround` found an empty fanout and returned true at `:498`. The divergence is in the shove's handling of a lone "stitching" via head (`pcbnew/router/pns_shove.cpp:1120`), not in the dragger. Its tier 2 test is `#[ignore]`d with the measurement; both tier 1 tests run, and they are the crash guard the case exists for.

Beside the corpus, `tests/dragger.rs` carries the via drag scenarios note 06 asked for, because the one corpus via case is a weak signal either way: a via with two fanout traces dragged in each of the three modes, its commit, and the E5 under-reporting.

## Acceptance

The seven drag goldens match; a trace can be dragged in LibrePCB with a working undo.
