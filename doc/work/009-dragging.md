# 009 Dragging

Status: todo

## Goal

Milestone 9: drag existing segments, corners and vias with the shove engine, as KiCad's `DRAGGER` and `MULTI_DRAGGER` do.

## Tasks

- [ ] Reference note for `pns_dragger.cpp`, `pns_multi_dragger.cpp` and `pns_drag_algo.h` under `doc/reference/kicad/`, with the helpers they need (`EDA_ANGLE`, `RotatePoint`, `PointAlong`, line versus line collisions).
- [ ] Session facade: `start_dragging`, drag `move_to`, fix, with the same preview and commit shapes as routing.
- [ ] Segment drag, corner drag, via drag, in walkaround and shove modes.
- [ ] Multi drag.
- [ ] The seven drag cases of the KiCad corpus un-ignored in `tests/kicad_replay.rs`.
- [ ] LibrePCB: drag from the select tool through `BoardPnsRouter`, one undo entry per drag.

## Acceptance

The seven drag goldens match; a trace can be dragged in LibrePCB with a working undo.
