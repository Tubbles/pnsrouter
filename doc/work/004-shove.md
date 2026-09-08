# 004 Shove

Status: in progress (started 2026-09-08)

## Goal

The shove algorithm and shove mode in the placer.

## Tasks

- [x] Phase 1: segment only shove. Hull walk, `check_shove_direction`, `shove_line_to_hull_set`, `shove_obstacle_line`, line stack, main loop with rank based anti ping pong, effects applied by the caller.
- [x] Phase 2: springback stack, root line index, optimizer queue as an insertion ordered map.
- [ ] Phase 3: solids, cluster assembly, walkaround escalation (`TryWalk`).
- [ ] Phase 4: vias, via hulls, lone via shove, via fanout drag, reverse via collision.
- [ ] Shove mode in the placer (`rh_shove_only`) with the node handoff documented in doc/reference/kicad/03 section 9.3.
- [ ] Scenario tests: push a parallel trace aside, retreat and observe springback, shove a via, budget exhaustion returns incomplete.

## Acceptance

Scenario tests green and deterministic, iteration budgets explicit.

## References

doc/reference/kicad/04-shove-and-optimizer.md sections 8 and 9.
