# 010 Differential pairs

Status: todo

## Goal

Milestone 10: route and drag differential pairs, as KiCad's `DIFF_PAIR_PLACER` and the pair dragger do.

## Tasks

- [ ] Reference note for `pns_diff_pair.cpp`, `pns_diff_pair_placer.cpp` and the pair drag code.
- [ ] Pair identification through the host hooks (`RuleResolver::dp_net_pair`, `dp_coupled_net`, `dp_net_polarity`); the test harness answers them from a synthetic board. No LibrePCB convention (on hold, see below).
- [ ] Gap and coupling rules through `RuleResolver::constraint` (`DiffPairGap`, `DiffPairMaxUncoupled`).
- [ ] The pair placer in all three modes, via placement for pairs.
- [ ] The pair dragger.
- [ ] LibrePCB: a pair routing mode in the router tool. On hold (user, 2026-09-10): LibrePCB has no pair concept and no naming convention is to be invented here, so the placer stays engine only until upstream has one; the fixture is a synthetic board in the crate's tests.

## Acceptance

A pair routes in all three modes on a two layer board and replays from a recording.
