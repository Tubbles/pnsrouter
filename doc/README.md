# Documentation index

- [../PLAN.md](../PLAN.md): milestones and end goal.
- [../DESIGN.md](../DESIGN.md): crate architecture.
- [log/](log/): decision log, one file per day, write once. Entries carry tags for grepping (for example `#license`, `#architecture`).
- [work/](work/): work items with status. Finished items stay in place.
- [performance.md](performance.md): interactive latency on a large synthetic board, how to reproduce it with `examples/latency.rs`, the profile behind the numbers and the shove budget sweep.
- [librepcb-integration.md](librepcb-integration.md): design note for milestone 6, the LibrePCB board editor integration (FFI marshalling, rule resolver, session driving, commit applier, task breakdown, open questions for upstream).
- [reference/kicad/](reference/kicad/): architecture notes on KiCad's PNS router with file:line citations, produced while studying the source. They are the primary input to DESIGN.md and to each porting work item.
