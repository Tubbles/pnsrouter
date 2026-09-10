# 009 Dragging

Status: implemented (2026-09-10), with two exceptions: one corpus golden, `issue23449-shove-lone-via-drag-crash`, still disagrees, and the LibrePCB gestures for multi drag and component drag are not wired up. Everything in the crate is in: the reference note, all three drag routines for a segment, a corner and a via, free angle mode, multi drag, component drag and the session facade. A single trace drag works in LibrePCB.

## Goal

Milestone 9: drag existing segments, corners and vias with the shove engine, as KiCad's `DRAGGER` and `MULTI_DRAGGER` do, and drag footprints by their pads as `COMPONENT_DRAGGER` does.

## Tasks

- [x] Reference note `doc/reference/kicad/06-dragger.md` (2026-09-10). It found the dragger needs no `EDA_ANGLE` outside arcs and no `RotatePoint`; `PointAlong` and line versus line collisions are multi drag only.
- [x] `Line::drag_segment` with the neighbour snapper, `via_pushout_force` lifted into `src/via.rs`, and `src/dragger.rs` with `start`, the mode decision, `drag` and mark obstacles mode (2026-09-10). Walkaround, shove and via drag are stubs.
- [x] Session facade: `Router::start_dragging`, the drag branches of `move_to`, `fix_route`, `stop_routing`, `abort_routing` and `pending_update`, `SessionEvent::StartDragging` recorded, serialised and replayed, and `EventKind::StartDrag` supported in the KiCad log player (2026-09-10). `optimize_and_update_dragged_line`, `best_anchor_for_point` and `point_has_bad_corner` landed with it; note 06 section 2.14 claims mark obstacles calls the first of them and it does not, so it is unreachable (and carries an `#[expect(dead_code)]`) until the walkaround drag arrives.
- [x] Segment and corner drag in walkaround and shove mode (2026-09-10). `dragWalkaround` and `tryWalkaround` with its length limit factor of 30.0, `dragShove` driving the existing `Shove` through the head protocol alone, and the optimizer's drag only passes behind them: `EffortFlags::REQUIRE_OBTUSE_ANGLES`, `Constraint::ObtuseOnly` and `Optimizer::drag_fix_corners`, which `optimize_and_update_dragged_line` asks for when `restrict_angles` is set. `optimize_and_update_dragged_line` is reachable at last, so its `#[expect(dead_code)]` is gone.
- [x] Via drag (2026-09-10): `findViaFanoutByHandle`, `dragViaMarkObstacles`, `propagateViaForces` over `via_pushout_force`, `dragViaWalkaround` and `dragShove`'s `DM_VIA` case, plus `MouseTrailTracer::trail_lead_vector`, which the force propagation negates. `Traces()` is two vectors here, `Dragger::traces` and `Dragger::traces_vias`, cleared as one. Errata E5, E6, E10, E11 and E13 are transcribed with a comment each; E5 is pinned by a test rather than repaired.
- [x] Free angle mode (2026-09-10). It needed no new code: `startDragSegment`'s second case, `Drag`'s bypass and `Line::drag_corner`'s free angle branch all landed in earlier slices, so this is one end to end test that also pins the forced corner mode and that no shove is built.
- [x] Multi drag (2026-09-10). `src/multi_dragger.rs` holds `MULTI_DRAGGER` in full: `MDRAG_LINE`, `Start`'s four phases, `Drag` with `tryPosture` over its three variants, `multidragMarkObstacles` with `clipToOtherLine`, `multidragWalkaround` with its own `tryWalkaround` and its length limit factor of 3.0, `multidragShove` with the head order and the re-add block, `findNewLeaderSegment` and `restoreLeaderSegments`, and the small members. `LineChain::point_along` (`shape_line_chain.cpp:2671`) landed with it, and `World::collide_lines` was already there. `Router::start_dragging` dispatches on the shape of the item set the way `pns_router.cpp:176` does, and `Router::last_committed_leader_segments` and `Router::host_of` are what a host puts the selection back with. Errata E17 to E27 are transcribed with a comment each; E28, the unspecified corner mode line order, is the one deviation and is a tie break on the line index.
- [x] The seven drag cases of the KiCad corpus replay in `tests/kicad_replay.rs` (2026-09-10). All seven leave nothing colliding and six of the seven match their golden. The two harness gaps that held the last two back are closed, and the one case that misses now, `issue23449`, misses because the via drag works.
- [x] Component drag (2026-09-10). `src/component_dragger.rs` holds `COMPONENT_DRAGGER` in full: `Start` with its `addLinked` closure, the two "runs between two dragged pads" cases and the unconnected trace end lookup; `Drag`, which rebuilds its one branch on every mouse move; `FixRoute`, `CurrentNode`, `Traces` and the three stub accessors. `CommitDiff` and `PreviewFrame` each gained a `moved_solids` list of `(HostId, Vec2)`, which is what `PNS_KICAD_IFACE` reconstructs into `m_fpOffsets` and turns into one footprint move (`pns_kicad_iface.cpp:2634`, `:2854`, `:2918`). `RouterState::DragComponent` is the state and `StartError::ComponentDragUnsupported` is gone, since every shape of a non empty item set now has an algorithm. Note 06 section 11 is the reference; errata E29 to E36 are transcribed with a comment each, three of them repaired (E31, E32, E33) and the rest reproduced.
- [x] `Router::pending_update` reports a component drag (2026-09-10). KiCad's `GetUpdatedItems` has no `DRAG_COMPONENT` branch (`pns_router.cpp:839`, `:844`) and answers nothing for one, which is erratum E15. That is the one addition rather than a transcription.
- [ ] LibrePCB: drag from the select tool through `BoardPnsRouter`, one undo entry per drag. A single trace drag works; the multi drag and component drag gestures are not wired up, and a component drag also needs the applier to move a device instance for every entry of `CommitDiff::moved_solids` (`doc/librepcb-integration.md` section 4.8).

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

The corpus says nothing at all about multi drag: no log holds an `EVT_START_MULTIDRAG` (note 05 section 6.9). The harness takes the event now, sharing the branch KiCad's log player shares (`qa/tools/pns/pns_log_player.cpp:155`), so a log recorded from one would replay; nothing was un-ignored by that. `tests/multi_dragger.rs` is the coverage instead: a pair dragged in each of the three modes, a corner mode grab, a bundle with unequal spacing and a determinism check, plus the facade case in `tests/router.rs`.

The corpus says nothing about component drag either, and could not: no log holds a `DRAG_COMPONENT` session and `GetUpdatedItems` would have measured an empty delta if one did (erratum E15). `tests/component_dragger.rs` is the coverage: two pads with a trace on each dragged a millimetre, the same drag in shove mode against a nearby track, a determinism check, a trace running between the two dragged pads, a trace end that merely stops inside a pad, and the facade case ending in a commit whose `moved_solids` holds both pads.

## The multi drag deviation

KiCad sorts the set by `MDRAG_LINE::dragDist` with `std::sort`, which is not stable, and `dragDist` is only ever assigned in the segment branch of `tryPosture` (`pns_multi_dragger.cpp:907`). In corner mode every line therefore compares equal and the walkaround attempt order and the shove head order are unspecified, and both decide the result: the first line walked has the free space, and the first head shoved sets the ranks. `DESIGN.md` section 8 forbids that, so the port ties on `mdragIndex`, the order the host listed the selected items in. Segment mode keeps KiCad's order exactly.

One thing that is not a deviation but is worth writing down: KiCad's `preWalkNode` (`:475`) is a local that is never deleted, so a walkaround multi drag leaks one node per mouse move. The port holds it as a member and drops it at the top of the next drag, which takes `m_lastNode` with it and subsumes the `delete m_lastNode` at `:461`.

## The component drag deviations

Three of note 06's eight component drag errata are repaired rather than reproduced, and each repair is invisible on a well formed board:

- **E31**, KiCad's `std::set<SOLID*>` and `std::set<ITEM*>` iterate in address order, which `DESIGN.md` section 8 forbids. Both are uid ordered vectors here.
- **E32**, `Start` clears none of its three collections and neither does the constructor, so a second start on one instance would accumulate. Latent in KiCad, because `StartDragging` allocates a fresh dragger per gesture; cleared here.
- **E33**, the two "runs between two dragged pads" tests count `SOLID_T` links and then walk every link of the joint. It cannot misfire in KiCad, because the set holds nothing but solids, so the filter is what the count already promised.

The rest are reproduced, E36 being the one that shows in a test: the unconnected trace end block asks for the same net **and** a collision, and a same net pair takes `clearance = -1` (`pcbnew/router/pns_item.cpp:188`) and never collides. What is left reachable is a netless pad with a netless trace end inside it, and any board carrying a user defined physical clearance rule. `tests/component_dragger.rs` exercises the netless case and says so.

## Acceptance

The seven drag goldens match; a trace can be dragged in LibrePCB with a working undo.

Six of the seven match. `issue23449-shove-lone-via-drag-crash` does not, for the reason above: its golden is a failure shape and the divergence is in the shove's handling of a lone stitching via head, not in the dragger. A trace can be dragged in LibrePCB with a working undo; the multi drag and component drag gestures there are the remaining work.
