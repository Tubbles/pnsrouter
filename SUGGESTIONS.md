# Suggestions

Ideas noticed during work, for the user to pick up or discard.

- Announce the project early in the LibrePCB forum thread (https://librepcb.discourse.group/t/13) and in issue LibrePCB/LibrePCB#1537. Urban Bruhin wrote in June 2025 that he would take care of the LibrePCB integration if somebody sets up and maintains the library, and that a Rust crate on crates.io is preferred. Early feedback on the host API shape in DESIGN.md avoids rework.
- Reserve the crate name on crates.io with a 0.0.1 placeholder release once the skeleton compiles, so the name is not taken while the port is in progress.
- KiCad's `qa/data/pcbnew/pns_regressions` contain recorded interaction logs plus board files. A KiCad board parser is out of scope, but the interaction log format (doc/reference/kicad/03-router-placer-walkaround.md section 7) is worth mirroring so that fixtures can be exchanged later.

## After milestone 6 (2026-09-09)

- Step 9 first: the manual test on a real project decides everything else. The scripts are in the step 6 to 8 sections of `doc/log/2026-09-09.md` and this session's transcript; run the board DRC before and after, in all three modes, on a two and a four layer board.
- If interactive speed is a problem on a real board, the two known costs are the full re-snapshot after every commit (`BoardEditorState_RouteTrace::rebuildRouter`) and the whole frame preview rebuild; `Router::assign_host_ids` over the FFI plus an incremental sync is the designed fix for the first.
- Upstream conversation before any PR: the ten questions in `doc/librepcb-integration.md` section 6, the Slint contract additions of step 8, and the Escape semantics (commit the fixed part, unlike the draw trace tool). LibrePCB's CONTRIBUTING.md requires the PR text to be written by a person and the LLM use declared.
- Crate side, in the order of value: `.kicad_dru` physical clearance rules so the last replay golden matches; the M7 hardening list in `doc/work/007-hardening-and-release.md`; the milestone 8 dragger, which also unlocks the seven drag cases of the KiCad corpus.
- API polish recorded in TODO.md under the step 4 to 8 headings (const pointers in `BoardPnsHostRef`, `BoardPnsCommit::isEmpty`, `BoardPnsNewItem` as a variant, corner mode in the session settings).

## After milestone 9 slice 3 (2026-09-10)

Two gaps in the KiCad replay harness, not in the crate, each worth about one corpus golden. Both were found by comparing the walked line of a drag against the log's `addedItems` and both were confirmed by experiment.

- **`KicadRules::clearance_epsilon` answers zero** (`tests/support/kicad_snapshot.rs`), where `PNS_PCBNEW_RULE_RESOLVER` answers the board's DRC epsilon (`pcbnew/router/pns_kicad_iface.cpp:338`, subtracted at `:972`). A pair sitting at exactly the nominal clearance therefore collides here and not in KiCad. Setting it to 500 nm makes `walk-with-teardrops` match its golden exactly and leaves every other case alone. Before shipping it, check what `ADVANCED_CFG::m_DRCEpsilon` actually defaults to; that file is outside the sparse KiCad checkout.
- **A rounded rectangle pad loses its corner radius** in `rectangle_shape` (`tests/support/kicad_snapshot.rs`), so it becomes a `Shape::Rect` whose hull has no 45 degree chamfer (`pcbnew/router/pns_utils.cpp:488`). KiCad's `syncPad` gives a roundrect the polygon branch (`pcbnew/router/pns_kicad_iface.cpp:1733`) and therefore `ConvexHull` (`pns_utils.cpp:300`), which pushes each diagonal in until it touches the outline. This is what keeps `drag-acute-fallback` one segment short of its golden. The fix is a polygon for a roundrect pad, which needs the arc flattening the harness already has for custom pads.
