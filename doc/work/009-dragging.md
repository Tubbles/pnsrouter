# 009 Dragging

Status: in progress (started 2026-09-10; the reference note and the dragger core with mark obstacles mode are in)

## Goal

Milestone 9: drag existing segments, corners and vias with the shove engine, as KiCad's `DRAGGER` and `MULTI_DRAGGER` do.

## Tasks

- [x] Reference note `doc/reference/kicad/06-dragger.md` (2026-09-10). It found the dragger needs no `EDA_ANGLE` outside arcs and no `RotatePoint`; `PointAlong` and line versus line collisions are multi drag only.
- [x] `Line::drag_segment` with the neighbour snapper, `via_pushout_force` lifted into `src/via.rs`, and `src/dragger.rs` with `start`, the mode decision, `drag` and mark obstacles mode (2026-09-10). Walkaround, shove and via drag are stubs.
- [ ] Session facade: `start_dragging`, drag `move_to`, fix, with the same preview and commit shapes as routing.
- [ ] Segment drag, corner drag, via drag, in walkaround and shove modes.
- [ ] Multi drag.
- [ ] The seven drag cases of the KiCad corpus un-ignored in `tests/kicad_replay.rs`.
- [ ] LibrePCB: drag from the select tool through `BoardPnsRouter`, one undo entry per drag.

## Acceptance

The seven drag goldens match; a trace can be dragged in LibrePCB with a working undo.
