# 010 Differential pairs

Status: in progress (started 2026-09-10 with the reference note)

## Goal

Milestone 10: route and drag differential pairs, as KiCad's `DIFF_PAIR_PLACER` and the pair dragger do.

## Tasks

- [x] Reference note `doc/reference/kicad/07-differential-pairs.md` (2026-09-10).
- [ ] Pair identification through the host hooks (`RuleResolver::dp_net_pair`, `dp_coupled_net`, `dp_net_polarity`); the test harness answers them from a synthetic board. No LibrePCB convention (on hold, see below).
- [ ] Gap and skew rules through `RuleResolver::constraint` (`DiffPairGap`, `DiffPairSkew`); `MaxUncoupled` never influences routing in KiCad (note 07 section 8), so it is host status only.
- [ ] The pair placer in all three modes, via placement for pairs.
- [x] The pair dragger: none exists in KiCad at this commit (note 07 section 9); selecting both tracks and using the multi dragger is the answer, so nothing to port.
- [ ] LibrePCB: a pair routing mode in the router tool. On hold (user, 2026-09-10): LibrePCB has no pair concept and no naming convention is to be invented here, so the placer stays engine only until upstream has one; the fixture is a synthetic board in the crate's tests.

## Acceptance

A pair routes in all three modes on a two layer board and replays from a recording.
