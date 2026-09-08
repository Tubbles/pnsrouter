# Suggestions

Ideas noticed during work, for the user to pick up or discard.

- Announce the project early in the LibrePCB forum thread (https://librepcb.discourse.group/t/13) and in issue LibrePCB/LibrePCB#1537. Urban Bruhin wrote in June 2025 that he would take care of the LibrePCB integration if somebody sets up and maintains the library, and that a Rust crate on crates.io is preferred. Early feedback on the host API shape in DESIGN.md avoids rework.
- Reserve the crate name on crates.io with a 0.0.1 placeholder release once the skeleton compiles, so the name is not taken while the port is in progress.
- KiCad's `qa/data/pcbnew/pns_regressions` contain recorded interaction logs plus board files. A KiCad board parser is out of scope, but the interaction log format (doc/reference/kicad/03-router-placer-walkaround.md section 7) is worth mirroring so that fixtures can be exchanged later.
