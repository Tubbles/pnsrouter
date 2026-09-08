# 002 World model

Status: in progress (started 2026-09-08)

## Goal

Items, layers, nets, spatial index, joints, the branching node, line assembly, nearest obstacle and the single hull walkaround on a line.

## Tasks

- [x] `LayerRange`, `NetId`, `ItemId` (generational), `Kind` bitmask, marker flags, rank, provenance (board or synthetic).
- [x] `Item` enum bodies: solid, segment, via, hole. Hull builders per body with the degenerate segment handling from `pns_utils.cpp`.
- [x] `Index`: per layer R-trees plus net map plus membership set, cheap clone, removal keyed on the item id and its insertion bbox.
- [x] `JointMap` with joint arena and `JointId`, touch, link, unlink, rebuild, lock.
- [ ] `Node` with add, remove, branch, commit, kill children, max clearance, and the single `visit_candidates` helper that visits branch and root only.
- [x] `collide_simple` with the clearance ladder and the `- 1` at `pns_item.cpp:249`.
- [ ] `Line` with links valid in a node, `LineVia::Owned | Linked`, assemble line, follow line, clip to nearest obstacle.
- [ ] `nearest_obstacle` with deterministic `(distance, uid)` ordering, `Line::walkaround` around one hull.
- [x] `RuleResolver` trait and a fixed clearance implementation for tests.
- [ ] Scenario tests: build a world from plain data, branch, add, remove, commit, assemble lines across joints, query colliding items.

## Acceptance

`cargo test` green, clippy clean. Doc comments cite KiCad file:line.

## References

doc/reference/kicad/02-item-model-and-node.md sections 10 and 11, doc/reference/kicad/01-geometry.md section 10.
