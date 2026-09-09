# 005 Session API and event log

Status: implemented (2026-09-09)

## Goal

The facade a host uses, the preview data it renders, the commit result it applies, and a recorder plus replay for regression fixtures.

## Tasks

- [x] `Router` session type: start routing, move, fix route, undo last segment, switch layer, toggle via, flip posture, toggle corner mode, finish, stop, hover query.
- [x] Preview as returned plain data: head and tail line chains with widths and layers, pending via, violation markers, leading ratline.
- [x] Commit result as returned plain data: added segments and vias, removed items by id, updated items, with the remove plus add fold that preserves host item identity.
- [x] `DebugDecorator` trait with the shove and placer hooks, plus a no-op implementation.
- [x] Event log: record world snapshot, settings and the input event sequence, replay against the engine, compare committed geometry.
- [x] Fixture based regression tests using the recorder.

## Acceptance

A host can drive a full routing session without any callback except the rule resolver.

## References

doc/reference/kicad/05-host-interface-and-tests.md sections 4, 6, 7 and doc/reference/kicad/03 section 1.
