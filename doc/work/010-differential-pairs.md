# 010 Differential pairs

Status: in progress (started 2026-09-10 with the reference note). The value layer is in: the coupling geometry, `DiffPair`, the gateway builders and the fit. The placer, the optimizer's pair path and the facade are not.

## Goal

Milestone 10: route and drag differential pairs, as KiCad's `DIFF_PAIR_PLACER` and the pair dragger do.

## Tasks

- [x] Reference note `doc/reference/kicad/07-differential-pairs.md` (2026-09-10).
- [x] Steps 1 to 5 of the note's section 14, the whole value layer, in `src/diff_pair.rs` (2026-09-10). `Seg::t_coef` and `common_parallel_projection`; `GapConstraint`, which is `RANGED_NUM<int>`; `DiffPair` with `coupled_segment_pairs`, both live `coupled_length` forms, `skew`, `set_shape` and its `SetShape( const DIFF_PAIR& )` twin, the end vias and the two line views; `DpGateway`, `DpPrimitivePair` over arena handles rather than owned clones, and `DpGateways` with `build_generic`, `build_entries`, `build_dp_continuation`, `build_from_primitive_pair`, `build_for_cursor` and `filter_by_orientation`; `check_connection_angle`, `check_gap`, `build_initial` and `fit_gateways`. `Sizes::diff_pair_pitch` names the centre to centre quantity so the two meanings of the gap cannot be mixed up again. Errata E4 to E7 and E10 are transcribed with a comment at the line, E1 and E18's dead members are not carried, and E17 cannot arise. `tests/diff_pair.rs` pins the complete ordered gateway list and the fitted chains of five inputs.
- [ ] Pair identification through the host hooks (`RuleResolver::dp_net_pair`, `dp_coupled_net`, `dp_net_polarity`); the test harness answers them from a synthetic board. No LibrePCB convention (on hold, see below).
- [ ] Gap and skew rules through `RuleResolver::constraint` (`DiffPairGap`, `DiffPairSkew`); `MaxUncoupled` never influences routing in KiCad (note 07 section 8), so it is host status only.
- [ ] The optimizer's pair path (step 6): `merge_dp_segments`, `merge_dp_step`, `coupled_bypass`, `verify_dp_bypass`, `find_coupled_vertices`, bounded by `MERGE_PASS_LIMIT` (erratum E14).
- [ ] `topology::simplify_line` (step 7), which `fix_route` needs.
- [ ] The pair placer in all three modes, via placement for pairs (steps 8 to 10).
- [x] The pair dragger: none exists in KiCad at this commit (note 07 section 9); selecting both tracks and using the multi dragger is the answer, so nothing to port.
- [ ] LibrePCB: a pair routing mode in the router tool. On hold (user, 2026-09-10): LibrePCB has no pair concept and no naming convention is to be invented here, so the placer stays engine only until upstream has one; the fixture is a synthetic board in the crate's tests.

## Acceptance

A pair routes in all three modes on a two layer board and replays from a recording.
