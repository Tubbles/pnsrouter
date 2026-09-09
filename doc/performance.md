# Performance

Interactive latency of a routing session, measured on a synthetic board, with the profile behind it. The work item is the "performance profiling on a large board, budget tuning" task of [work/007-hardening-and-release.md](work/007-hardening-and-release.md).

The number that matters is the wall time of one `Router::move_to`. LibrePCB calls it once per mouse motion event and redraws the `PreviewFrame` it answers with, so a move that takes longer than one 60 Hz frame (16 ms) is lag the user sees.

## How to run it

```
dev/in-container.sh cargo run --release --example latency            # 2000 and 20000 segments
dev/in-container.sh cargo run --release --example latency -- 5000    # one board size
```

The arguments are target track segment counts, default 2000 and 20000. For each of them the example builds a board, then runs one session per routing mode: it starts on a pad, makes 96 moves of half a pitch each along a diagonal that crosses the grid, fixes a corner every 12 moves, and stops. It records the wall time of `Router::new`, `start_routing`, every `move_to`, every `fix_route` and `stop_routing`, and prints p50, p95, max and the number of moves over 16 ms.

`PNSROUTER_SHOVE_ITERATION_LIMIT` overrides `RoutingSettings::shove_iteration_limit` for the run, which is what the budget table below was measured with.

The wall clock is read in the example and nowhere else: `src/` never reads it, per `DESIGN.md` section 8.

### The board

A two layer Manhattan grid, `lines` traces per layer, each broken into `pieces` segments, with `lines = isqrt(2 * target)` and `pieces = lines / 4`, so both the number of traces and the length of one trace grow with the requested size. Horizontal traces sit on layer 0, vertical ones on layer 1, each on its own net, at a pitch of 1 mm with a track width of 0.2 mm and a clearance of 0.2 mm, which leaves a channel wide enough for the routed track. Every row and column has one two cell gap so that a route has somewhere to slip through, and the gap of a row is a fixed stride from the gap of the row before it, so nothing here is random and one segment count always produces one board. Pads and vias sit in the middle of a cell, on their own nets, every tenth and every seventeenth cell. The outline is 32 non routable edge segments.

The 20 000 segment board is 201 mm square with 20 573 items, which is a dense two layer board of a realistic size.

## The machine

- AMD Ryzen 7 2700X, 8 cores and 16 threads, running at about 3.99 GHz.
- Container image `localhost/pnsrouter-dev:latest`, built from `dev/Containerfile` on `docker.io/librepcb/librepcb-dev:ubuntu-24.04-4`, which is Ubuntu 24.04.3 LTS.
- rustc 1.92.0 (ded5c06cf 2025-12-08), stock `--release` profile.

## The numbers

All times in milliseconds. `>16 ms` counts the moves of the 96 that missed a 60 Hz frame. `segs` is how many segments the session committed and `drawn` the total number of preview items it produced, both there to show that the session did the same work in every run.

Board of 2000 target segments: 1890 track segments, 50 pads, 15 vias, 32 edge segments, 1987 items. Snapshot build 0.3 ms.

| mode | `Router::new` | `start_routing` | move p50 | move p95 | move max | >16 ms | `fix_route` max | `stop_routing` | segs | drawn |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| mark obstacles | 6.0 | 0.018 | 0.090 | 0.266 | 0.280 | 0 | 0.224 | 0.044 | 0 | 3558 |
| walkaround | 5.1 | 0.016 | 0.792 | 22.927 | 55.685 | 5 | 0.226 | 0.274 | 44 | 1728 |
| shove | 4.7 | 0.014 | 8.499 | 78.519 | 229.806 | 33 | 0.179 | 2.718 | 84 | 6589 |

Board of 20 000 target segments: 20 000 track segments, 401 pads, 140 vias, 32 edge segments, 20 573 items. Snapshot build 2.7 ms.

| mode | `Router::new` | `start_routing` | move p50 | move p95 | move max | >16 ms | `fix_route` max | `stop_routing` | segs | drawn |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| mark obstacles | 61.9 | 0.015 | 0.089 | 0.319 | 0.357 | 0 | 0.291 | 0.568 | 0 | 3537 |
| walkaround | 54.7 | 0.015 | 2.990 | 66.271 | 97.741 | 19 | 0.155 | 1.116 | 25 | 1188 |
| shove | 55.2 | 0.019 | 56.255 | 523.550 | 833.327 | 80 | 0.202 | 6.376 | 90 | 8882 |

The route is deterministic, the timings are not: over six runs of the 20 000 board the shove p50 ranged from 46 to 56 ms and the max from 800 to 880 ms.

Mark obstacles mode commits nothing because the head is in violation at every fix and `allow_drc_violations` is false, which is KiCad's default. The moves still do the mode's full work.

What the table says:

- The one time costs are fine. Building the snapshot is 2.7 ms and `Router::new`, which is `World::from_snapshot` and the root index build, is 62 ms for 20 573 items. A host that re-syncs the whole board after every commit pays that 62 ms each time, which is the LibrePCB item already in `TODO.md` under step 6.
- `start_routing`, `fix_route` and `stop_routing` are never a latency problem: all of them stay under 7 ms, most under 0.3 ms.
- Mark obstacles mode is flat: 0.09 ms per move on both boards. Ten times the items changes nothing, which is the spatial index doing its job.
- Walkaround mode misses frames on the tail: p50 under 3 ms, p95 over 60 ms on the large board.
- Shove mode is not interactive on the large board: p50 56 ms, and 80 of 96 moves over the frame budget.

## What the cost actually scales with

Not with the number of items. The experiment below was run by editing the example locally and reverting afterwards.

Fixing the trace length at 15 segments per trace while the board keeps growing (so the item count still grows, but every chain the router handles stays the same length):

| traces per layer | items | walkaround p50 | shove p50 |
| --- | --- | --- | --- |
| 31 | 975 | 0.90 | 2.54 |
| 63 | 1987 | 0.76 | 8.84 |
| 126 | 4045 | 0.67 | 21.8 |
| 200 | 6573 | 0.77 | 48.6 |

Walkaround mode goes flat, exactly like mark obstacles: its cost is the number of points in the chains it walks, and nothing else. Shove mode keeps growing at roughly the 1.6th power of the board's linear extent, with the item count held down. What grows there is the shove cascade: a pushed trace spans more of the board, so it collides with more of its neighbours before the shove finds a free lane, and the number of shove iterations rises until it hits the budget.

The samples confirm it. Of the 131 stack samples taken inside `Shove::shove_iteration`, the iteration counter had a median of 89 and a maximum of 249 against a limit of 250, so the expensive moves are the ones that spend the whole budget.

## Where the time goes

`perf` is not in the container image (`which perf` fails), so the profile was taken on the host, where `gdb` is, by attaching to the running release binary in a loop:

```
while kill -0 "$pid"; do gdb -p "$pid" -batch -ex "set pagination off" -ex "bt 80"; done
```

The binary is built in the container and runs unchanged on the host. Debug information came from `CARGO_PROFILE_RELEASE_DEBUG=2` in the environment of the container build, so no `Cargo.toml` change was needed and none was made. 200 samples over repeated 20 000 segment runs, 196 of them inside `Router::move_to`.

Self time by the innermost matching component:

| component | share of `move_to` |
| --- | --- |
| `Line::walkaround` (`src/line.rs:1840`) | 58.5% |
| obstacle queries, collision and spatial index maintenance (`World::nearest_obstacle`, `query_colliding_*`, `Index::add`) | 28.0% |
| node mutation (add, remove and replace line, joints) | 9.5% |
| everything else | 4.0% |

### First cost: `Line::walkaround`

`src/line.rs:1840`. Reached 90 times out of 117 from `Shove::shove_line_to_hull_set` (`src/shove.rs:2115`) and 27 times from `walkaround::process_cluster`. Inside it, as a share of all `move_to` time:

| what | share |
| --- | --- |
| `LineChain::point_on_edge` at `src/line.rs:1889` and `:1911` | 23.5% |
| `LineChain::point_inside` at `src/line.rs:1855` and `:1912` | 8.5% |
| `LineChain::self_intersecting` at `src/line.rs:1867` | 7.0% |
| `LineChain::find` at `src/line.rs:1875` and `:1879` | 5.5% |
| `LineChain::split` at `src/line.rs:1876`, `:1880` and `:1894` | 3.5% |
| growing the per vertex neighbour `Vec` at `src/line.rs:1939` and `:1980` | 3.0% |
| `hull_intersection` at `src/line.rs:1862` | 2.5% |
| `find_vertex` at `src/line.rs:1951`, `:1976` and `:1977` | 2.5% |

`point_on_edge` (`src/geometry/line_chain.rs:2026`) is `edge_containing_point` (`:2047`), a linear scan over every edge of the chain, and the two loops that call it visit every point of the path, so the pair is O(path points times hull points) per walkaround.

**Inherent.** `LINE::Walkaround` (`pcbnew/router/pns_line.cpp:297`) runs the same two loops over the same predicate: the hull splitting loop at `:376` calls `hnew.PointOnEdge( p )` at `:379`, and the classification loop at `:405` calls `PointOnEdge` at `:409` and `PointInside` at `:410`. `SHAPE_LINE_CHAIN_BASE::PointOnEdge` (`libs/kimath/src/geometry/shape_line_chain.cpp:2074`) is `EdgeContainingPoint` (`:2080`), the same linear scan, and KiCad reaches its segments through a virtual `GetSegment`, which the port does not.

The call count is inherent too. `SHOVE::shoveLineToHullSet` tries four winding and traversal combinations (`pcbnew/router/pns_shove.cpp:338`) and walks around every hull of the set inside each of them (`:414`), and `ShoveObstacleLine` wraps that in three hull expansion attempts (`:578`). One shove iteration can therefore be twelve walkarounds per hull, and one move can be 250 shove iterations. The port transcribes all three loops.

### Second cost: the obstacle query

`World::nearest_obstacle` (`src/node.rs:1921`) is 39 of the 56 samples in that row, 19.5% of `move_to`, reached 22 times from `Shove::shove_iteration` and 17 times from `Walkaround::single_step` (`src/walkaround.rs:620`). Most of the rest is the index maintenance of adding and removing the lines a shove replaces. It splits into the broad and narrow phase collision search at `src/node.rs:1930` and the hull building and intersection scan at `:2008`.

**Inherent.** It is a transcription of `NODE::NearestObstacle` (`pcbnew/router/pns_node.cpp:298`), which does the same query, the same `makeHull` per obstacle and the same intersection scan. The broad phase is an R-tree per copper layer and behaves: mark obstacles mode, which uses the same query path, is flat at 0.09 ms per move across a tenfold change in item count. The port is ahead of KiCad in one place here, `World::hull_of` (`src/node.rs:3000`) memoises the hulls, and behind it in another, KiCad runs the per obstacle scan on a thread pool (`pns_node.cpp:437`) which the port leaves sequential for determinism.

## Budget tuning

The tail is made of shove iterations that the shove ends up discarding. Sweeping `RoutingSettings::shove_iteration_limit` on the 20 000 segment board, everything else unchanged:

| limit | move p50 | move p95 | move max | >16 ms | segs | drawn |
| --- | --- | --- | --- | --- | --- | --- |
| 250 (KiCad's default) | 49.5 | 496.8 | 849.8 | 80 | 90 | 8882 |
| 100 | 44.8 | 189.3 | 738.7 | 80 | 90 | 8882 |
| 50 | 38.8 | 101.9 | 136.8 | 81 | 90 | 8882 |
| 25 | 18.6 | 40.2 | 49.5 | 55 | 79 | 7793 |

And on the 2000 segment board:

| limit | move p50 | move p95 | move max | >16 ms | segs | drawn |
| --- | --- | --- | --- | --- | --- | --- |
| 250 | 8.29 | 78.5 | 244.8 | 31 | 84 | 6589 |
| 50 | 7.97 | 71.2 | 98.2 | 33 | 84 | 6589 |

On both boards a limit of 50 leaves the session's outcome unchanged (the same committed segment count and the same total preview item count) while cutting the worst move by a factor of six. A limit of 25 does change the outcome: the shove gives up more often and the route ends up shorter.

The default stays at KiCad's 250. Two synthetic boards are not enough evidence to move a number that decides what the router can push, and the setting is the host's to choose. What the measurement does say is that a host with a latency budget has a cheap knob: on this board everything above 50 iterations was work whose result was thrown away.

KiCad has a second brake that this crate deliberately does not: `shoveMainLoop` tests a 1000 ms wall clock limit alongside the iteration limit (`pcbnew/router/pns_shove.cpp:1890`). `DESIGN.md` section 8 forbids reading a clock inside an algorithm, so the iteration limit is the only budget here. The worst moves measured, 830 to 880 ms, sit just under the point where KiCad would have bailed out on time instead.

## What was changed

Nothing in `src/`. The measurement found no hot spot that a small local change would fix.

One port level difference did turn up and was rejected on the evidence. `World::invalidate_caches` (`src/node.rs:3222`) scans the whole clearance cache and the whole hull cache once per removed item, where KiCad's `ClearCacheForItems` (`pcbnew/router/pns_kicad_iface.cpp:792`) scans them once per deletion batch (`pns_node.cpp:127` and `:1610`). It showed up in 4% of the samples. Replacing the body with a no op, which is the upper bound of any batching fix, moved the 20 000 board shove p50 from 49.7 to 51.0 ms, that is nowhere outside the run to run spread, because dropping the invalidation also lets the caches grow and makes every later lookup slower. Not worth the churn in the node code.

## What is left

- Shove mode on a large board is not interactive and no local fix changes that. The cost is the cascade length, which is the algorithm. The realistic answers are the budget (see above), an early bail out when the cascade is clearly not converging, or the thread pool KiCad uses.
- `Line::walkaround` allocates one `Vec` per graph vertex for its neighbour list, at most three entries each, which is about 3% of `move_to`. An inline three element list would remove it. It needs a hand rolled type, since the crate takes no new dependencies, and a proof that no vertex can ever exceed three neighbours.
- The parallel obstacle scan of `NODE::NearestObstacle` (`pns_node.cpp:437`) is still not ported. It was already listed as deferred in `TODO.md` under milestone 2 with "profile before adding threads". The profile now exists: the scan is inside the 28%, so threading it would buy less than shortening the walkaround work would.
