# 008 Parallel obstacle query

Status: todo

## Goal

Port the thread pool of `NODE::NearestObstacle` (`pcbnew/router/pns_node.cpp:437`) without giving up determinism, and measure what it buys.

## Tasks

- [ ] Read how KiCad splits the candidate loop across the pool and what it reduces (note 02, `pns_node.cpp:437` onwards).
- [ ] `std::thread::scope` over chunks of the candidate list, sequential below a threshold of candidates, no new dependency, no `unsafe`.
- [ ] Reduction by (distance, uid) exactly as the sequential path, so the answer does not depend on thread count or scheduling.
- [ ] A test that runs the KiCad goldens and the session fixtures with the thread count forced to 1 and to many and compares the commits byte for byte.
- [ ] `examples/latency.rs` before and after on both boards; `doc/performance.md` updated.

## Acceptance

Same answers on every thread count; the 20 000 segment board's move p95 lower than before; `doc/performance.md` records the numbers.
