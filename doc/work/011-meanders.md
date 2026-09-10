# 011 Meanders

Status: implemented on the engine side (2026-09-10); the LibrePCB task is deferred

## Goal

Milestone 11: length tuning of a single trace, of a differential pair, and skew tuning between the legs of a pair, as KiCad's `MEANDER_PLACER`, `DP_MEANDER_PLACER` and `MEANDER_SKEW_PLACER` do.

## Tasks

- [x] Reference note `doc/reference/kicad/08-meanders.md` (2026-09-10). Finding: no `VECTOR2D` port is needed; the turtle only turns by 90 degrees so its direction stays integer, and the three transcendental constants are compile time constants. Tuning in KiCad is a board generator (`PCB_TUNING_PATTERN`), not the router tool.
- [x] Meander shapes with chamfered corners; the round style is refused at the settings boundary (a `#[non_exhaustive]` corner style with one variant) since drawing chamfers for it would miss the target by about 0.62 radius per meander. Steps 1 to 4 of the note's section 13, all in `src/meander.rs` (2026-09-10):
  - [x] Step 1, the settings and status types: `MeanderSettings` behind a constructor that refuses a non positive step (erratum E22) and the round style, `LengthTarget`, `CornerStyle`, `MeanderStyle`, `MeanderSide`, `MeanderType`, `TuningStatus`.
  - [x] Step 2, the shape generator: `MeanderShape` with the integer turtle, the chamfered corner and the five bodies of `genMeanderShape`, borrowing a `MeanderContext` instead of holding a placer back pointer. Pinned against the note's hand computed anchor case and the closed forms of section 2.6.
  - [x] Step 3, the fitting loop: `MeanderedLine` with `meander_segment`, `check_self_intersections` and `MeanderShape::fit` including the two check types. The side flip comes out as a flag rather than writing back into the placer's settings.
  - [x] Step 4, the length arithmetic: `tune_line_length`, `find_amplitude_for_length`, `find_amplitude_binary_search`, `amplitude_step`, `spacing_step`, `clearance`. Errata E8 and E16 are behind named tests so a later fix is a visible test change.
- [x] Step 5, `topology::assemble_tuning_path` and the length metric (2026-09-10). `walk_tuning_path` and `find_lines_from_via` with it, and `topology::path_length` in place of the host hook `ROUTER_IFACE::CalculateRoutedPathLength`: the plain sum of the chain lengths, since this crate has no pad to die length, no via height and no propagation delay. The two in place chain fixups `AssembleTuningPath` ends with, `OptimiseTraceInPad` and `OptimiseTraceInVia`, live in KiCad's board code and are not ported, so a track that overshoots into a pad measures as it is drawn.
- [x] Single trace length tuning with target, tolerance, amplitude and spacing settings (step 6, 2026-09-10). `src/placer/meander_placer.rs`, `Placer::Meander`, `Router::start_tuning` with `PreviewFrame::tuning`, `Router::amplitude_step` and `Router::spacing_step`, and the three recorded events.
- [x] Step 7, `topology::assemble_diff_pair` (2026-09-10). The `SEGMENT*` overload is declared and never defined in KiCad (note 07 erratum E16), so only the live one is ported. It answers the pair **and** the two assembled lines, because this crate's `DiffPair` holds chains only where KiCad's keeps the linked `LINE`s that both tuners walk and remove from. `src/topology.rs`'s "what is not here" list lost its `AssembleDiffPair` entry.
- [x] Differential pair length tuning keeping the coupling (step 8, 2026-09-10). `src/placer/dp_meander_placer.rs`, `Placer::DpMeander`, `Router::start_tuning_diff_pair` and the recorded `SessionEvent::StartTuningDiffPair`.
- [x] Skew tuning and the facade (step 9, 2026-09-10). `src/placer/meander_skew_placer.rs`, `Placer::MeanderSkew`, `Router::start_tuning_skew` and `SessionEvent::StartTuningSkew`. `RouterState` gained `TuneDiffPair` and `TuneSkew`, and `router::TuningInfo` gained the mode, the skew, the skew target and the coupled lane's length.
- [ ] LibrePCB: a tuning mode with its settings in the toolbar. **Deferred**: the pair half has no LibrePCB counterpart while pairs themselves are on hold (user decision, 2026-09-10, `PLAN.md`), and the single trace half needs a tool state and a settings panel. `doc/librepcb-integration.md` section 3.8 records what all three modes would take.

## Acceptance

A tuned trace reaches its target length within the tolerance, in a replayed session.

Met for the single trace half on 2026-09-10 by `tests/meander_placer.rs::a_trace_reaches_a_longer_target` and `a_tuning_session_replays`, and by `eventlog::tests::a_tuning_session_replays_to_the_same_commit`.

Met for the skew half the same day by `tests/meander_skew_placer.rs::the_skew_placer_lengthens_the_shorter_lane`, which closes a two millimetre skew to eight nanometres, and by `a_skew_session_replays`.

Met for the pair half with a **wider window than KiCad's own**, and the reason is a KiCad defect this slice found: `DP_MEANDER_PLACER` fits the first meander of every coupled span from the P lane's vertex rather than from the pair's centreline, so that meander is drawn half a pitch off the two lanes and the tuned length overshoots by one or two jogs of that size. On the section 14.3 board that is 400018 nm against KiCad's 100000 nm tolerance, so `a_pair_reaches_a_longer_target` asks for 600000 nm and `the_first_meander_of_a_span_is_drawn_off_the_two_lanes` pins the overshoot with KiCad's own window. Note 08 erratum E24 records it.

## What the fixture cannot cover

Note 08 section 14.7 asks for this to be said out loud, and it should stay said. There is **no upstream test this port can be checked against**: KiCad's regression corpus holds no tuning case, its log format cannot express one (`ROUTER_MODE` is not in it at all), and its unit test suite never constructs a `MEANDER_SHAPE`. So the fidelity of the meander code rests entirely on the hand computed shape assertions in `src/meander.rs`, taken from note 08 section 2.6's closed forms, and on the errata being reproduced deliberately rather than on any end to end replay. That is a weaker footing than milestones 3, 4 and 9 had.

Four more things the fixture cannot reach: the round corner style and therefore any comparison against KiCad's default output; a tuned stretch that already contains an arc; time domain tuning and net chains, neither of which is ported; and pad to die length, which the crate has no concept of.

Two of the note's own fixture cases turned out to be unreachable and are not written. `an_obstacle_flips_the_initial_side` (section 14.6) cannot fire on the single trace board, because the amplitude scan finds a shorter meander on the same side before it ever needs the other one. And `a_pair_reaches_a_longer_target`'s "status `Tuned`" cannot be had inside KiCad's own 100000 nm window on the pair board, for the erratum E24 reason above; it is written with a 600000 nm window instead.
