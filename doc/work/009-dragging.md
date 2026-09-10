# 009 Dragging

Status: in progress (started 2026-09-10; the reference note, the dragger core with mark obstacles mode and the session facade are in)

## Goal

Milestone 9: drag existing segments, corners and vias with the shove engine, as KiCad's `DRAGGER` and `MULTI_DRAGGER` do.

## Tasks

- [x] Reference note `doc/reference/kicad/06-dragger.md` (2026-09-10). It found the dragger needs no `EDA_ANGLE` outside arcs and no `RotatePoint`; `PointAlong` and line versus line collisions are multi drag only.
- [x] `Line::drag_segment` with the neighbour snapper, `via_pushout_force` lifted into `src/via.rs`, and `src/dragger.rs` with `start`, the mode decision, `drag` and mark obstacles mode (2026-09-10). Walkaround, shove and via drag are stubs.
- [x] Session facade: `Router::start_dragging`, the drag branches of `move_to`, `fix_route`, `stop_routing`, `abort_routing` and `pending_update`, `SessionEvent::StartDragging` recorded, serialised and replayed, and `EventKind::StartDrag` supported in the KiCad log player (2026-09-10). `optimize_and_update_dragged_line`, `best_anchor_for_point` and `point_has_bad_corner` landed with it; note 06 section 2.14 claims mark obstacles calls the first of them and it does not, so it is unreachable (and carries an `#[expect(dead_code)]`) until the walkaround drag arrives.
- [ ] Segment drag, corner drag, via drag, in walkaround and shove modes.
- [ ] Multi drag.
- [x] The seven drag cases of the KiCad corpus replay in `tests/kicad_replay.rs` (2026-09-10). Their goldens do not match yet, and six of the seven leave the dragged trace lying across something, because the mark obstacles fallback follows the cursor exactly.
- [ ] LibrePCB: drag from the select tool through `BoardPnsRouter`, one undo entry per drag.

## Where the seven corpus cases stand

Measured through `Router::pending_update` after the last recorded move, which is what KiCad's harness compares (`pcbnew/router/pns_router.cpp:844`); none of the seven logs holds an `EVT_FIX`. Every one of them replays without a panic and keeps a session running.

| Case | Mode | Added | Golden added | Removed | Golden removed | Colliding segments left |
| --- | --- | --- | --- | --- | --- | --- |
| `drag-acute-fallback` | walkaround | 7 | 11 | 6 | 6 | 1 |
| `drag-walk-optimize-a` | walkaround | 7 | 9 | 6 | 6 | 1 |
| `drag-walk-optimize-fix-corners` | walkaround | 3 | 9 | 3 | 3 | 2 |
| `walk-with-teardrops` | walkaround | 19 | 17 | 19 | 19 | 3 |
| `simple-drag-shove-singlelayer` | shove | 18 | 134 | 18 | 139 | 1 |
| `walk_drag_seg_against_board_edge` | shove | 3 | 3 | 3 | 3 | 2 |
| `issue23449-shove-lone-via-drag-crash` | shove | 0 | 0 | 0 | 0 | 0 |

Two goldens agree already and their tier 2 tests run. `issue23449` agrees because both sides are empty and the via drag moves nothing yet, so it is a crash guard and little else. `walk_drag_seg_against_board_edge` agrees on counts only: one long segment goes out and the three pieces of the dragged line come back, which is the same arithmetic the shove drag will have to reproduce with different geometry.

The remaining five need steps 6 to 8 of note 06 section 10.2: the walkaround drag, `REQUIRE_OBTUSE_ANGLES` (three of the four walkaround cases set `restrict_angles`) and the shove drag.

## Acceptance

The seven drag goldens match; a trace can be dragged in LibrePCB with a working undo.
