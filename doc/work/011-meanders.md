# 011 Meanders

Status: in progress (started 2026-09-10 with the reference note)

## Goal

Milestone 11: length tuning of a single trace, of a differential pair, and skew tuning between the legs of a pair, as KiCad's `MEANDER_PLACER`, `DP_MEANDER_PLACER` and `MEANDER_SKEW_PLACER` do.

## Tasks

- [x] Reference note `doc/reference/kicad/08-meanders.md` (2026-09-10). Finding: no `VECTOR2D` port is needed; the turtle only turns by 90 degrees so its direction stays integer, and the three transcendental constants are compile time constants. Tuning in KiCad is a board generator (`PCB_TUNING_PATTERN`), not the router tool.
- [ ] Meander shapes with chamfered corners; the round style is refused at the settings boundary (a `#[non_exhaustive]` corner style with one variant) since drawing chamfers for it would miss the target by about 0.62 radius per meander.
- [ ] Single trace length tuning with target, tolerance, amplitude and spacing settings.
- [ ] Differential pair length tuning keeping the coupling.
- [ ] Skew tuning.
- [ ] LibrePCB: a tuning mode with its settings in the toolbar.

## Acceptance

A tuned trace reaches its target length within the tolerance, in a replayed session.
