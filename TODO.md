# TODO

Inbox for new work items. Fully specified items are moved into `doc/work/`.

## Inbox

(empty)

## Possible action items

Deferred during milestone 1 (geometry foundation), each with the place it surfaced:

- Arcs: `SHAPE_ARC`, `ArcHull` (`pns_utils.cpp:71`), the arc reference vector in `LineChain` (planned as `Vec<ArcRef>` parallel to the points, note 01 section 14.3), `SelfIntersectingWithArcs`, the `ROUNDED_45` and `ROUNDED_90` corner modes of `Direction45::build_initial_trace` with the `MIN_PRECISION_IU` re-snap (`direction_45.cpp:200`). Deferred because LibrePCB has no arc traces and milestone 1 routes straight segments only.
- `LineChain::nearest_point` lost KiCad's `aAllowInternalShapePoints` flag because its body is inert without arcs (`shape_line_chain.cpp:2425`). Restore it with the arc vectors; `pns_shove.cpp:359` passes `true` and `pns_helpers.cpp:98` passes `false`.
- Rounded rectangle outline is not polygonised (`src/geometry/shape.rs`, `rect_outline`); the sharp outline is a conservative superset. The router never builds a rounded rect (`pns_utils.cpp:356` leaves the radius at zero). Port `ROUNDRECT::TransformToPolygon` only if a host ever supplies one.
- `Shape::Compound` bounding box passes the clearance down where KiCad drops it (`shape_compound.cpp:78`); if a fixture ever depends on KiCad's under grown box, revisit.
- `LineChain::PointAlong` (`shape_line_chain.cpp:2671`) landed with multi drag on 2026-09-10 as `LineChain::point_along`; its one caller is still `clipToOtherLine` (`pns_multi_dragger.cpp:313`).
- `VECTOR2D` needs no port: the meander turtle only turns by 90 degrees so its direction stays integer, and the only genuine doubles are three compile time constants (note 08 section 8). Closed 2026-09-10.
- `EDA_ANGLE` is used by `SOLID` orientation (`pns_solid.h`) and, in the dragger, only inside `startDragArc` (arcs, on hold); `Seg::angle_degrees` returns bare `f64` degrees. Decide on an angle newtype when the item model needs solid orientation (note 06 section 9 corrects the earlier claim that the dragger needs it).
- `RotatePoint` from `trigo.h` (note 01 section 1.4) is not used by either dragger (note 06 section 9); find its remaining callers when the item model needs rotation.
- The polygon union in `SOLID::Hull` and `HOLE::Hull` (`pns_solid.cpp:55`, `pns_hole.cpp:84`) is replaced by a convex hull of the per primitive hulls per DESIGN.md section 3. Measure against a KiCad fixture with a complex pad once fixtures exist; a tighter union may matter for dense pad rows.
- Exact `isqrt` replaces KiCad's truncated `f64` square root in distances; the two agree below about 95 mm. If a fixture disagrees above that, the distances in `seg.rs`, `line_chain.rs` and `collision.rs` are the place to look.

Deferred during milestone 2 (world model):

- `INDEX::SetDeferred` / `BuildSpatialIndex`, KiCad's bulk load for the initial board sync (`pns_index.cpp:55`, `pns_node.cpp:1257`), is not ported. Measure the root index build on a large board before adding a second insertion path.
- The parallel obstacle scan of `NODE::NearestObstacle` (`pns_node.cpp:437`) is ported as of milestone 8 and defaults to off: the profile said the threads cost more than they save. See `doc/work/008-parallel-obstacle-query.md`.
- `NODE::FixupVirtualVias` (`pns_node.cpp:1282`) is not ported; note 02 records two errata in it (a dead `n_seg >= 3` branch, a `locked_seg` that leaks across joints). Decide with the shove work item.
- Line versus line collisions (`pns_item.cpp:133`) are not supported since lines are never stored; the shove (`pns_shove.cpp:318`, `:481`) and optimizer (`pns_optimizer.cpp:1356`) call sites must decompose one side into segments when they are ported. The multi dragger's (`pns_multi_dragger.cpp:321`) is done, through `World::collide_lines`, which decomposes the obstacle side.
- `check_colliding_items` (`src/node.rs`) takes items only, not lines, for the same reason.
- The via self collision heuristic was retired (log entry of 2026-09-08). Two distinct stored vias at one position with equal padstack, net and drill now collide hole to hole; revert `consider_hole_to_hole` in `src/collide.rs` if a shove fixture disagrees.
- The 90 degree corner mode hull simplification in `NearestObstacle` (`pns_node.cpp:330`) needs the routing settings and lands with the walkaround.

Deferred during milestone 3 (walkaround router):

- `update_leading_ratline` (`src/placer/line_placer.rs`) is a stub: it needs `TOPOLOGY::LeadingRatLine` / `NearestUnconnectedItem` (`pns_topology.cpp`), which land with the session facade.
- The preview via has no hole item, so `via_pushout_force` resolves copper clearances only (`pns_via.cpp:126` gives KiCad's via a hole). Add a synthetic hole to the preview via when hole to copper rules matter for via placement.
- `Sizes::via_layer_range` and `Sizes::layer_top`/`layer_bottom` need the board's copper layer count for through vias; the crate carries none. Give `Sizes` or the world a layer count when the facade is designed.
- The dead pad orientation and last segment postures of `LINE_PLACER::Start` (`pns_line_placer.cpp:1408`, `:1415`) are not computed; wiring them into `SetDefaultDirections` is a routing change that needs a fixture.
- `LINE_PLACER::AbortPlacement` (`pns_line_placer.cpp:2150`) has no caller and is not ported.
- The via pushout keeps KiCad's discarded `force.Resize( threshold )` (`pns_via.cpp:207`); capping the step is a behaviour change to decide with a shove fixture.

Deferred during milestone 4 (shove):

- The via anti snap loop in `src/shove.rs` is bounded at 1000 iterations and returns `Incomplete`; KiCad's is unbounded. Revisit if a fixture needs more.
- `World::collide_lines` (`src/node.rs`) does not decompose a via on the obstacle side; every shove call site keeps the via carrying line on the head side. Needed only if a future caller collides two via ended lines.
- `ShoveDraggingVia` is declared and never defined in KiCad. Decided in milestone 9 (2026-09-10): it does not exist here either. `dragShove`'s `DM_VIA` case leaves the call commented out (`pns_dragger.cpp:919`) and drives the via through `AddHeads( VIA_HANDLE, pos, policy )` and `Run` like any other head, which is what `Dragger::drag_shove` does.
- `reduceSpringback` keeps its bottom frame (`pns_shove.cpp:926`), so the first move's shove is sticky within a session; reproduced, worth a look with a real board fixture.
- Arcs throughout the shove are marked `TODO(arcs)`.

Deferred during the KiCad replay work (2026-09-09):

- Keepout zones are counted and skipped by the snapshot converter (`tests/support/kicad_snapshot.rs`); `Item` carries no keepout mark yet. None of the replayed boards has one.
- A tier 3 comparison (net names and per item geometry against `addedItems`) is cheap once the gaps above close.

Deferred during LibrePCB step 4 (2026-09-09):

- `Router::undo_last_segment` right after `fix_route` answers `None` because the fix clears the head; faithful to KiCad, whose host always moves in between, but a host friendly facade could answer the fixed tail's last point instead. Documented in `BoardPnsRouter::getPreview()` for now.
- Expose `Router::assign_host_ids` over the FFI before step 5: without it a second route in the same session does not recognise the board objects the first commit became.
- Add `clang-format` to `dev/Containerfile` so LibrePCB C++ can be formatted in the container; step 4's files were formatted by hand.
- `dev/librepcb-in-container.sh cargo clippy --lib` on rust-core skips the `ffi` module; document `--features ffi` wherever the check is listed.

Deferred during LibrePCB step 6 (2026-09-09):

- `BoardPnsRouter::undoLastSegment()` returns the leg's start for cursor warping, which `BoardEditorFsmAdapter` cannot do; either add a cursor warp to the adapter or drop the return value.
- The whole board is re-snapshotted after every commit; measure on a large board in step 9 and consider `assign_host_ids` plus an incremental sync.

Deferred during LibrePCB step 7 (2026-09-09):

- The fixed tail renders brighter than the head because `setLighterColorsWithMinAlpha` brightens; decide after the manual test whether the head or the tail should get it (one line in `getMinAlphaOfStyle()`).
- Collision and rat line share the air wire colour; a dedicated registered graphics layer for router collisions would read better.
- Clearance halo (KiCad draws the geometry twice, once inflated) needs a primitive that can lower alpha.
- `BoardPnsPreviewVia::style` beyond Collision is invisible: `PrimitiveCircleGraphicsItem` has no lighter colour mode.
- `BoardPnsViolation::clearance` and `forcedLayer` have no consumer; either draw a marker or drop them.
- `BoardPnsHostRef` needs `operator==` and `qHash` so frames can be diffed as sets; `BoardPnsPreviewItem::layer` can be null for an out of range dense index and is silently skipped.

Deferred during LibrePCB step 8 (2026-09-09):

- Raise the Slint contract additions with upstream before any PR: `EditorTool.route-trace`, `RouterMode`, two `TabAction` values, two `Board2dTabData` properties, one helper. They are additive but `types.slint` is shared with every tab.

Deferred during the latency measurement (2026-09-09):

- Shove mode is not interactive on a large board: median 56 ms per `move_to` and 80 of 96 moves over the 16 ms frame budget on a 20 000 segment board (`doc/performance.md`). The cost is the shove cascade, not the index, and no local fix changes it. The options are the iteration budget, an early bail out when the cascade is not converging, or KiCad's thread pool.
- `Line::walkaround` (`src/line.rs:1840`) allocates one `Vec` per graph vertex for a neighbour list of at most three entries, about 3% of `move_to`. An inline list would remove it, but it needs a hand rolled type (no new dependencies) and a proof that no vertex can exceed three neighbours.
- `RoutingSettings::shove_iteration_limit` stays at KiCad's 250. On both measured boards a limit of 50 gave the same route with a sixth of the worst case move time. Revisit with fixtures from real boards before moving the default, and consider exposing it to LibrePCB.
- `World::invalidate_caches` (`src/node.rs:3222`) scans both caches once per removed item where KiCad batches (`pns_kicad_iface.cpp:792`). Measured at zero benefit when removed entirely, so it is left alone. Revisit only if a profile on a real board disagrees.
- `Router::new` is 62 ms for a 20 573 item board, which is the cost a host pays for every full re-sync. That is the incremental sync item already listed under LibrePCB step 6.

Deferred during the session recording work (2026-09-09):

- A recording carries no rule table, only `max_clearance` and the sizes, so a replay of a board with net class overrides cannot pick the right resolver. Serialising the clearance answers the session actually used (a table by net pair, or the net class values) would make replays exact for any board.

Deferred during the M7 close (2026-09-09):

- cargo-fuzz targets for `SessionRecording::from_text` and for random shape pairs through `collide` (no panic, symmetric answer) need a nightly toolchain; run them as a CI job on nightly rather than in the container, which pins 1.92.0 and has a read only rustup directory.

Requested by the user (2026-09-10):

- As soon as 0.1.0 is on crates.io, switch LibrePCB from the `libs/pnsrouter` submodule to `pnsrouter = "0.1"` in `libs/librepcb/rust-core/Cargo.toml` and remove the submodule.

Deferred during the LibrePCB keepout zones and the milestone 9 to 11 hosts (2026-09-10):

- Zones reach the preview as collision items only; `BoardPnsPreviewStyle::SemiSolid` is still never emitted, and drawing the zone's triangles semi solid while routing is now possible.
- Zones on device footprints are not synced: their layer flags go through the device transform and a host id for one needs a reference `BI_Zone*` cannot carry.
- `CommitDiff::moved_solids` (a component drag) is not applied by `CmdBoardApplyPnsCommit`, so the host refuses a pad drag; applying it means moving the device instance in the same undo group (note section 4.8).
- Multi drag has no LibrePCB gesture: the router tool passes one host id. A selection driven drag from the select tool would give the crate several.
- Differential pairs and length tuning have no LibrePCB UI (on hold with the pair identification); the crate's entries are `start_routing_diff_pair`, `start_tuning`, `start_tuning_diff_pair`, `start_tuning_skew`.
