# 003 Walkaround router

Status: implemented (2026-09-08)

## Goal

Interactive single net routing in mark obstacles and walkaround modes with the optimizer, via placement and undo of the last segment, headless.

## Tasks

- [x] `RoutingSettings` and `Sizes` structs with KiCad's defaults, without the host only snap flags and the source strings.
- [x] `MouseTrailTracer` posture solver with the area comparison heuristic and its thresholds.
- [x] `Walkaround` with the two windings plus shortest policy and iteration limits, without the dead setters.
- [x] `Optimizer`: merge obtuse, merge step, merge full, merge colinear, area and preserve vertex constraints, corner cost. Skip the dead cache and the no-op constraints.
- [x] `LinePlacer` as an enum state machine: start, move, route step with head and tail negotiation, fix route, fixed tail stages, undo, via placement, layer switch, finish, continue from end.
- [x] Scenario tests on a two layer board with pads and existing traces: route around obstacles, place a via, undo, finish on a pad.

## Acceptance

Scenario tests green and deterministic across runs. Committed geometry checked against golden files.

## References

doc/reference/kicad/03-router-placer-walkaround.md sections 3, 4, 5, 9.
