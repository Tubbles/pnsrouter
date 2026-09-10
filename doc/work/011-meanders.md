# 011 Meanders

Status: todo

## Goal

Milestone 11: length tuning of a single trace, of a differential pair, and skew tuning between the legs of a pair, as KiCad's `MEANDER_PLACER`, `DP_MEANDER_PLACER` and `MEANDER_SKEW_PLACER` do.

## Tasks

- [ ] Reference note for `pns_meander.cpp` and the three placers, plus the `VECTOR2D` floating point vector algebra they use.
- [ ] Meander shapes with chamfered corners; rounded corners wait for arcs.
- [ ] Single trace length tuning with target, tolerance, amplitude and spacing settings.
- [ ] Differential pair length tuning keeping the coupling.
- [ ] Skew tuning.
- [ ] LibrePCB: a tuning mode with its settings in the toolbar.

## Acceptance

A tuned trace reaches its target length within the tolerance, in a replayed session.
