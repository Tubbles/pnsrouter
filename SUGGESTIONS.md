# Suggestions

Ideas noticed during work, for the user to pick up or discard.

- Announce the project early in the LibrePCB forum thread (https://librepcb.discourse.group/t/13) and in issue LibrePCB/LibrePCB#1537. Urban Bruhin wrote in June 2025 that he would take care of the LibrePCB integration if somebody sets up and maintains the library, and that a Rust crate on crates.io is preferred. Early feedback on the host API shape in DESIGN.md avoids rework.
- KiCad's `qa/data/pcbnew/pns_regressions` contain recorded interaction logs plus board files. A KiCad board parser is out of scope, but the interaction log format (doc/reference/kicad/03-router-placer-walkaround.md section 7) is worth mirroring so that fixtures can be exchanged later.

## After milestone 6 (2026-09-09)

- If interactive speed is a problem on a real board, the two known costs are the full re-snapshot after every commit (`BoardEditorState_RouteTrace::rebuildRouter`) and the whole frame preview rebuild; `Router::assign_host_ids` over the FFI plus an incremental sync is the designed fix for the first.
- Upstream conversation before any PR: the ten questions in `doc/librepcb-integration.md` section 6, the Slint contract additions of step 8, and the Escape semantics (commit the fixed part, unlike the draw trace tool). LibrePCB's CONTRIBUTING.md requires the PR text to be written by a person and the LLM use declared.

## After milestone 9 slice 4 (2026-09-10)

- One corpus golden disagrees, `issue23449-shove-lone-via-drag-crash`, and the disagreement is in the shove rather than in the dragger: KiCad pushes a via with no fanout onto its line stack as a proxy line (`pcbnew/router/pns_shove.cpp:1120`), and something downstream of that ends the recorded session with an untouched node where this crate ends it with the via moved and legal. Worth an hour with `PNS_DBG` traces on both sides before milestone 11 touches the shove again; the case's own tier 2 test carries the measurement.

## After milestone 9 slice 6, component drag (2026-09-10)

- **KiCad's unconnected trace end handling is nearly dead code** and this crate now reproduces that faithfully; note 06 erratum E36 has the reading. If it ever looks like a bug worth reporting upstream, the one line fix is to drop the same net test at `pcbnew/router/pns_component_dragger.cpp:142` or to give the collide call a search context with `m_differentNetsOnly = false`. Worth raising on the KiCad tracker rather than deviating here.
