# 008 Parallel obstacle query

Status: implemented, and off by default

## Goal

Port the thread pool of `NODE::NearestObstacle` (`pcbnew/router/pns_node.cpp:437`) without giving up determinism, and measure what it buys.

## Tasks

- [x] Read how KiCad splits the candidate loop across the pool and what it reduces (note 02, `pns_node.cpp:437` onwards).
- [x] `std::thread::scope` over chunks of the candidate list, sequential below a threshold of candidates, no new dependency, no `unsafe`.
- [x] Reduction by (distance, uid) exactly as the sequential path, so the answer does not depend on thread count or scheduling.
- [x] A test that runs the KiCad goldens and the session fixtures with the thread count forced to 1 and to many and compares the commits byte for byte.
- [x] `examples/latency.rs` before and after on both boards; `doc/performance.md` updated.

## Acceptance

> Same answers on every thread count; the 20 000 segment board's move p95 lower than before; `doc/performance.md` records the numbers.

The first and the third hold. **The second does not, and it is not going to.** Eight interleaved runs put the 20 000 board's shove p95 at 507.6 ms on one thread and 499.8 ms on eight, which is inside a run to run spread this document already measured at plus or minus 8%. Mark obstacles mode is reproducibly *worse* on threads: three independent batches put its p95 up 13 to 29% and its worst move up 25 to 32%.

The arithmetic behind that is in the parallel obstacle query section of `doc/performance.md`. One `std::thread::scope` costs about 21 microseconds per block, so 42 for the smallest useful split; scanning one obstacle candidate costs about 2.2 microseconds; the queries big enough to be cut into blocks are 5% of them and hold a quarter of a scan that is 4% of the run. There is no threshold on the candidate count that turns that into a win, because the cost of a candidate is proportional to the head's segment count as well and the mode with the cheapest candidates is the one that loses most.

So the split is implemented, tested and documented, and `World::set_parallelism` defaults to 1. The two ways to make it pay are both bigger than this item: a persistent worker pool, which needs either a dependency or `'static` bounds a `World` full of `Rc` cannot give, and a work estimate that counts head segments as well as candidates, which wants a workload to tune against.

## What landed

- `World::set_parallelism`, `World::parallelism`, `World::MAX_USEFUL_PARALLELISM` and `Router::set_parallelism`. The knob is on the world and not on `RoutingSettings`, because settings are serialised into a session recording and a thread count belongs to the machine replaying it.
- `RuleResolver` is unchanged: the parallel section never calls it, and a `Send + Sync` bound would rule out a memoising host resolver for no gain today.
- `eventlog::replay_with_parallelism` and `tests/support/kicad_replay::replay_case_with_parallelism`, for the tests.
- `PNSROUTER_PARALLELISM` in `examples/latency.rs`, read in the example and nowhere else.
- Tests: `tests/parallelism.rs` over every session fixture, `every_replayable_case_replays_the_same_at_every_thread_count` in `tests/kicad_replay.rs`, and two unit tests in `src/node.rs`, `the_candidate_scan_answers_the_same_at_every_block_count` and `a_distance_tie_goes_to_the_lowest_uid_at_every_thread_count`. The recorded sessions never reach the block threshold, so the unit tests are the ones that exercise the split; both test files say so.
