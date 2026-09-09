# 006 LibrePCB integration

Status: in progress (started 2026-09-09, steps 1 to 4 of doc/librepcb-integration.md on the fork's pns-router branch)

## Goal

Interactive routing in LibrePCB's board editor using pnsrouter, developed on the user's fork (`~/dev/librepcb`, branch to be created from master).

## Tasks

- [x] Add pnsrouter as a Cargo dependency of `libs/librepcb/rust-core` (git dependency during development, crates.io later).
- [x] Board snapshot over FFI: copper layers, net ids, pads with outlines and clearances, vias, traces, holes, keepout zones, board outline. Planes are not synced (KiCad and Horizon do the same).
- [x] RuleResolver for LibrePCB in rust-core: net class minimum copper clearance, pad copper clearance, board design rules.
- [x] `ffi_pnsrouter_*` session functions in rust-core and the headless C++ wrapper `BoardPnsRouter` in `libs/librepcb/core/project/board/` (step 4 of `doc/librepcb-integration.md`; the snapshot and the wrapper live in core, not the editor library, because neither has a user interface or an undo stack).
- [ ] New tool state `BoardEditorState_DrawTraceInteractive` next to the existing draw trace tool, with a Slint toolbar (mode, layer, width, via size, corner mode).
- [ ] Preview rendering with the existing graphics items, commit through the undo stack as one command group per route, anchor resolution to existing net points, pads and vias at apply time.
- [ ] Manual test on a real project in all three modes, on a two layer and a four layer board.

## Acceptance

Routing works in all three modes with undo, on a two layer and a four layer board, without DRC violations introduced by the router.

## References

doc/reference/kicad/05-host-interface-and-tests.md sections 3, 5 and 7. LibrePCB: `libs/librepcb/rust-core`, `libs/librepcb/editor/project/board/fsm/boardeditorstate_drawtrace.cpp`, the 2018 branch `upstream/add-pns-router` with `pns_librepcb_iface.cpp`.
