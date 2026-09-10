# 009 Dragging

Status: in progress (started 2026-09-10; the reference note, all three drag routines for a segment or a corner, and the session facade are in; the via drag and multi drag are not)

## Goal

Milestone 9: drag existing segments, corners and vias with the shove engine, as KiCad's `DRAGGER` and `MULTI_DRAGGER` do.

## Tasks

- [x] Reference note `doc/reference/kicad/06-dragger.md` (2026-09-10). It found the dragger needs no `EDA_ANGLE` outside arcs and no `RotatePoint`; `PointAlong` and line versus line collisions are multi drag only.
- [x] `Line::drag_segment` with the neighbour snapper, `via_pushout_force` lifted into `src/via.rs`, and `src/dragger.rs` with `start`, the mode decision, `drag` and mark obstacles mode (2026-09-10). Walkaround, shove and via drag are stubs.
- [x] Session facade: `Router::start_dragging`, the drag branches of `move_to`, `fix_route`, `stop_routing`, `abort_routing` and `pending_update`, `SessionEvent::StartDragging` recorded, serialised and replayed, and `EventKind::StartDrag` supported in the KiCad log player (2026-09-10). `optimize_and_update_dragged_line`, `best_anchor_for_point` and `point_has_bad_corner` landed with it; note 06 section 2.14 claims mark obstacles calls the first of them and it does not, so it is unreachable (and carries an `#[expect(dead_code)]`) until the walkaround drag arrives.
- [x] Segment and corner drag in walkaround and shove mode (2026-09-10). `dragWalkaround` and `tryWalkaround` with its length limit factor of 30.0, `dragShove` driving the existing `Shove` through the head protocol alone, and the optimizer's drag only passes behind them: `EffortFlags::REQUIRE_OBTUSE_ANGLES`, `Constraint::ObtuseOnly` and `Optimizer::drag_fix_corners`, which `optimize_and_update_dragged_line` asks for when `restrict_angles` is set. `optimize_and_update_dragged_line` is reachable at last, so its `#[expect(dead_code)]` is gone.
- [ ] Via drag: `findViaFanoutByHandle`, `dragViaMarkObstacles`, `propagateViaForces`, `dragViaWalkaround` and `dragShove`'s `DM_VIA` case. All three drag routines have the arm and none of them moves the via.
- [ ] Multi drag.
- [x] The seven drag cases of the KiCad corpus replay in `tests/kicad_replay.rs` (2026-09-10). All seven leave nothing colliding and five of the seven match their golden; the two that do not diverge in the snapshot the test harness builds, not in the drag.
- [ ] LibrePCB: drag from the select tool through `BoardPnsRouter`, one undo entry per drag.

## Where the seven corpus cases stand

Measured through `Router::pending_update` after the last recorded move, which is what KiCad's harness compares (`pcbnew/router/pns_router.cpp:844`); none of the seven logs holds an `EVT_FIX`. Every one of them replays without a panic, keeps a session running and leaves nothing colliding.

| Case | Mode | Added | Golden added | Removed | Golden removed | Colliding segments left |
| --- | --- | --- | --- | --- | --- | --- |
| `drag-acute-fallback` | walkaround | 10 | 11 | 6 | 6 | 0 |
| `drag-walk-optimize-a` | walkaround | 9 | 9 | 6 | 6 | 0 |
| `drag-walk-optimize-fix-corners` | walkaround | 9 | 9 | 3 | 3 | 0 |
| `walk-with-teardrops` | walkaround | 15 | 17 | 19 | 19 | 0 |
| `simple-drag-shove-singlelayer` | shove | 134 | 134 | 139 | 139 | 0 |
| `walk_drag_seg_against_board_edge` | shove | 3 | 3 | 3 | 3 | 0 |
| `issue23449-shove-lone-via-drag-crash` | shove | 0 | 0 | 0 | 0 | 0 |

Five goldens agree and their tier 2 tests run, and every collision test runs. `simple-drag-shove-singlelayer` is the one that says the shove drag works: 134 added and 139 removed across ten nets, all of it the cascade one drag head pushed. `walk_drag_seg_against_board_edge` now agrees vertex for vertex with the log's `addedItems` and not only on counts. `issue23449` still agrees because both sides are empty and the via drag moves nothing, so it is a crash guard and little else.

The two that miss both diverge in the snapshot `tests/support/kicad_snapshot.rs` builds, not in the drag, and both were traced to the walked line before the post drag optimizer ran.

- `drag-acute-fallback` detours around pad 2 of `C1`, a rounded rectangle. The harness maps every rectangular pad to a `Shape::Rect` and drops the corner radius (`rectangle_shape`), and an `SH_RECT` gets an octagonal hull with **no** chamfer (`pcbnew/router/pns_utils.cpp:488`). KiCad's `syncPad` sees a roundrect as a compound of five shapes, takes the polygon branch (`pcbnew/router/pns_kicad_iface.cpp:1733`) and gets `ConvexHull` (`pns_utils.cpp:300`), whose diagonals are pushed inward until they touch the outline. So KiCad's detour column at x = 146.675 mm runs y 104.964172 to 106.035828 where the crate's runs 104.575 to 106.425, the same face cut back by 0.389172 mm at each end, and the crate's line reaches the drag anchor with one segment where KiCad's needs two. Everything from the anchor onward is identical.
- `walk-with-teardrops` has one pair of parallel 0.2 mm tracks at exactly the 0.2 mm net class clearance, `(58.5, 40.118628)` to `(52.581371, 34.2)` against the dragged line. `KicadRules::clearance_epsilon` answers zero, where KiCad's rule resolver answers the board's DRC epsilon (`pcbnew/router/pns_kicad_iface.cpp:338`, subtracted at `:972`), so the pair collides here and not there. The very first move therefore runs a walkaround it should never have needed, and that walk circles the board outline. Setting the epsilon to 500 nm in the harness makes the case match its golden exactly and changes no other case; it is not shipped, because the number needs checking against KiCad's advanced configuration default and it would move the rules under all eleven cases at once.

Both are recorded in `SUGGESTIONS.md`.

## Acceptance

The seven drag goldens match; a trace can be dragged in LibrePCB with a working undo.
