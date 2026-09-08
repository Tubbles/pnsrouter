# Design

This document describes the architecture of the pnsrouter crate. It is the distilled form of the KiCad reference notes under doc/reference/kicad/, which carry the file and line citations. Every section names the note it is derived from. Decisions and their dates are in doc/log/.

## 1. Purpose and boundaries

The crate is an interactive routing engine: given a snapshot of a board's copper obstacles, a rule oracle, settings, and a stream of user events (start, move, fix, undo, layer switch, via toggle, finish), it produces a preview frame after every event and a commit diff at the end. It is a design level port of KiCad's PNS router (`pcbnew/router`).

The crate does not know about any board file format, any GUI toolkit, any undo stack, or cursor snapping. Those belong to the host. In KiCad terms, the crate is the 60 engine files that Horizon EDA vendored unchanged, and the host is the 4 files Horizon rewrote (`pns_kicad_iface`, `router_tool`, `pns_tool_base`, `router_preview_item`). See note 05 section 5 for that inventory.

The crate has no FFI. LibrePCB's `libs/librepcb/rust-core` crate will depend on pnsrouter and expose C functions through cbindgen, the same way it already wraps the `interactive-html-bom` crate.

## 2. Numeric model

Coordinates are `i32` nanometres, KiCad's envelope of about plus or minus 2.1 metres. Differences of coordinates are computed in `i64` before any product, products and determinants are `i64`, and `rescale` uses `i128` as KiCad's `util.cpp` does. `euclidean_norm` and `resize` reproduce KiCad's 45 degree special cases so that diagonal rounding matches. Rounding is half away from zero with saturation. Integer square root is exact. The `- 1` in the collision clearance (`pns_item.cpp:249`) and every tolerance in note 01 section 13 are transplanted verbatim, because the walkaround's termination depends on hulls and collision predicates agreeing to the nanometre.

Hosts with wider coordinates (LibrePCB uses i64) range check at the snapshot boundary. Source: note 01 sections 2 and 14.1, note 05 section 7.4.

## 3. Geometry layer

Module `geometry` holds pure value types with no knowledge of nets or layers.

- `Vec2` (i32) and `Vec2L` (i64) with dot, cross, norms, `resize`, `kiround`, `rescale`, `isqrt`.
- `Seg` with distance, squared distance, nearest point, intersection, collinearity (determinant `<= 1`), containment (squared tolerance `<= 3`), line projection.
- `Box2` as an `Option` based bounding box with merge, contains, intersects, inflate, center, squared distance.
- `Direction45`: an octant enum plus a 90 degree flag, angle classification as bitflags, `build_initial_trace` for the 45 and 90 degree corner modes, and the `MIN_PRECISION_IU` endpoint re-snap.
- `LineChain`: a polyline with an explicit closed flag and width. `append` suppresses a duplicate of the last point (the placer depends on it), a closed chain has `segment_count == point_count`, `slice` returns a `Result`, `intersect` returns a `Vec` of `Hit::Segment(index) | Hit::Corner(index)`, and both `simplify` (exact colinearity, arc aware later) and `simplify2` (1 nm tolerance) exist because the optimizer's convergence depends on their difference. There is no bounding box cache. Milestone 1 has no arcs; the planned extension is a `Vec<ArcRef>` parallel to the points, see note 01 section 14.3.
- `Shape`: an enum of `Circle`, `Rect` (with the radius field the collision code branches on), `Segment`, `Simple` (a closed `LineChain` newtype), `Compound(Vec<Shape>)`. No polygon set: the router core performs exactly one polygon boolean, a self union for compound pad hulls, which is replaced by a convex hull of the primitive hulls (a conservative superset). Note 01 sections 11 and 14.5.
- `collision`: a `match` over shape pairs computing collision with clearance, actual distance, location, and a minimum translation vector with one documented sign convention (the vector displaces the second argument). Compound shapes flatten before dispatch.
- `hull`: octagonal hull, segment hull with the kink threshold (`clearance / 10`) and its `cl++` and `cl += 2` compensations, convex hull by monotone chain.

Tests mirror KiCad's `qa/tests/libs/kimath` where they exist (note 01 section 15) and add tables for `Direction45` and the collision matrix, which KiCad does not unit test.

## 4. World model

Source: note 02 sections 10 and 11, note 01 section 10.

### 4.1 Items

Every stored object is an `Item` in an arena addressed by a generational `ItemId`. The body is an enum: `Solid`, `Segment`, `Via`, `Hole` (and later `Arc`). An item carries its kind bitmask, net (`Option<NetId>`), `LayerRange` (closed interval over dense copper layer indices), marker flags, rank, a provenance (`Board(HostId)` or `Synthetic`), a `flashed_layers` mask, an optional `hole: ItemId`, and a per world `uid: u64` used for deterministic tie breaking.

There is no owner pointer, no clone virtual, no deferred free pool: a stale `ItemId` fails its generation check instead of dangling.

### 4.2 Lines

`Line` is a transient value, never stored in a node: a `LineChain` plus width, layer, net, an optional via as `LineVia::Owned(Via) | Linked(ItemId)`, the `links: Vec<ItemId>` of the segments it was assembled from, and the `NodeId` those links are valid in. Marking and rank operations take `&mut Node` explicitly instead of hiding mutation behind interior mutability. The explicit `Linked` variant removes KiCad's geometric self collision heuristic for line vias.

### 4.3 Index

`Index` holds one R-tree per copper layer plus a net map and a membership set, all keyed by `ItemId` with the insertion bounding box stored alongside so removal never misses. The root node's index is built once from the snapshot. Branch indexes only ever contain items added during a routing session, so cloning them is cheap. Every query is inflated by the node's `max_clearance`, which the host must not understate (note 05 section 7.2).

### 4.4 Joints

Joints live in their own arena and are addressed by `JointId`, never by address. The map is keyed by `(position, net)` and may hold several joints per key that differ in layer range, exactly like KiCad's multimap, with explicit layer overlap disambiguation. A tombstone is `Option::None`, not a joint with a negative layer range.

### 4.5 Nodes and the two level overlay

`World` owns a node arena. A `Node` has a parent, the root, a depth, its own `Index`, `JointMap`, and an `overrides: IndexSet<ItemId>` of root items shadowed in this branch. Every read path visits exactly the branch and the root through one helper, `visit_candidates`, never the parent chain. `branch()` on a non root node clones the parent's index, joints and overrides. `commit()` folds a branch into the root, `kill_children()` drops a subtree. This is KiCad's design with the raw pointers removed; the eager copy is what keeps queries O(1) in depth.

## 5. Rules and flashing

Source: note 02 section 7 and 10.6, note 05 sections 2, 7.2, 7.3, 7.5.

```rust
pub trait RuleResolver {
  fn clearance(&self, a: ItemRef, b: Option<ItemRef>, use_epsilon: bool) -> Option<i32>;
  fn clearance_epsilon(&self) -> i32 { 0 }
  fn constraint(&self, ty: ConstraintType, a: ItemRef, b: Option<ItemRef>, layer: LayerIndex) -> Option<Constraint>;
  fn is_keepout(&self, obstacle: ItemRef, item: ItemRef) -> Keepout; // None | Present | Enforced
  fn is_drilled_hole(&self, item: ItemRef) -> bool;
  fn is_non_plated_slot(&self, item: ItemRef) -> bool;
  fn net_code(&self, net: NetId) -> i32;
  // diff pair and net tie methods defaulted to "not supported"
}
```

`Option<i32>` replaces the `-1` sentinel, a three state enum replaces the bool plus out parameter. The clearance cache and the hull cache belong to the engine, keyed by `ItemId`, so hosts cannot get invalidation wrong.

`IsFlashedOnLayer` is not a callback. It is a pure function of item and layer with three effects (collision suppression, via hull shrinking to the hole, per layer pad geometry), so it is materialised as the `flashed_layers` mask on the item. For LibrePCB's first integration the mask equals the layer span.

## 6. Algorithms

### 6.1 Walkaround

Source: note 03 section 5. `Line::walkaround` walks a line around one hull. `Walkaround` runs the clockwise, counter clockwise and shortest policies as three state machines with an iteration limit and returns the best result by path length. Winding is hull reversal, which relies on the hull builders always emitting clockwise. The dead setters and the undefined overload are not ported.

### 6.2 Shove

Source: note 04 sections 1 to 3, 8. The entry point is clear heads, add heads, run. Anti ping pong is the rank test (`obstacle.rank > current.rank`), heads start at 100000 and each forward shove assigns `rank - 1`. There is no analytic side decision: `shove_line_to_hull_set` tries the four winding and hull order combinations and accepts the first candidate passing endpoint preservation, direction, self intersection and collision checks, with three hull inflation attempts. The springback stack is a `Vec` of frames over arena nodes. The line stack has three explicit operations (push, push below top, retain not referencing). The optimizer queue is an insertion ordered map keyed by root line identity.

The shove iteration returns an effect list (`ReplaceLine`, `MoveVia`, `PushLine`, `PopLine`, `Fail`) that the caller applies, instead of mutating items while iterating a copy. Budgets are explicit (`max_iterations`, optional deadline that tests never set) and are reset once per run, not once per head.

### 6.3 Optimizer

Source: note 04 section 4. Passes: merge obtuse, merge step, merge full, merge colinear, smart pads, fanout cleanup, with area and preserve vertex constraints and the corner cost estimator. The collision cache, the corner count limit, the vertex range restriction, the 45 degree angle constraint and the length cost are dead in KiCad and are not ported. Corner mode is a parameter, not a global.

### 6.4 Line placer

Source: note 03 sections 3, 4, 9.1 to 9.3. The placer is an enum state machine (`Idle`, `Placing`, `Finished`) with a `Placing` struct holding head, tail, directions, anchors, the `chained` flag (the only flag that gates a layer change), sizes, the posture solver and the fixed tail stages. Every mouse move rebuilds the head from scratch; the tail grows through `merge_head` and shrinks through `reduce_tail`, `handle_pullback` and `handle_self_intersections`, with the step re-run after each shrink. Mode dispatch is a `match` (mark obstacles, walk only, shove only), where shove falls back to walk and both share `rh_walk_base`. `fix_route` writes segments and vias into the node. The placer never owns nodes: it holds `current_node: NodeId` and `last_node: Option<NodeId>` into the world's node arena, and in shove mode the shove hands back the node the placer stands on.

The posture solver (`MouseTrailTracer`) compares the areas enclosed by the straight first and diagonal first candidates against the reversed mouse trail, with the hysteresis and lock thresholds from note 03 section 4.

Inert KiCad behaviours listed in note 03 section 9.6 are decided one by one in the porting work item, and each decision is logged.

## 7. Session API

Source: note 03 section 1, note 05 sections 4 and 7.2.

```rust
pub struct Router { /* world, settings, sizes, placer, shove, caches */ }

impl Router {
  pub fn new(snapshot: WorldSnapshot, rules: Box<dyn RuleResolver>, settings: RoutingSettings) -> Self;
  pub fn hover(&self, at: Vec2, layer: LayerIndex) -> Vec<ItemId>;
  pub fn start_routing(&mut self, at: Vec2, start: Option<ItemId>, layer: LayerIndex, sizes: Sizes) -> Result<PreviewFrame, RouterError>;
  pub fn move_to(&mut self, at: Vec2, end: Option<ItemId>) -> PreviewFrame;
  pub fn fix_route(&mut self, at: Vec2, end: Option<ItemId>, force_finish: bool) -> FixOutcome;
  pub fn undo_last_segment(&mut self) -> Option<Vec2>;
  pub fn switch_layer(&mut self, layer: LayerIndex) -> bool;
  pub fn toggle_via(&mut self);
  pub fn flip_posture(&mut self);
  pub fn finish(&mut self) -> CommitDiff;
  pub fn stop(&mut self) -> CommitDiff;
}
```

Inputs are already snapped points; snapping is host work. `PreviewFrame` is returned whole after every event (items with clearance and style flags, path lines, ratlines, hidden host ids), which matches KiCad's actual semantics where every move erases the view first. `CommitDiff` lists removed host ids, added items, and updated items whose identity is preserved by the engine's remove plus add fold. Escape does not discard the route in KiCad or Horizon; the crate exposes both `finish` and `stop` and lets the host decide.

## 8. Determinism and testing

- No `HashMap` or `HashSet` is ever iterated. Iterated containers are `Vec`, `IndexMap`, `IndexSet`.
- Obstacle candidates are sorted by `(distance, uid)`. Uids come from a per world counter, never from a process global.
- No wall clock inside algorithms. Budgets are iteration counts; a deadline is optional and never set in tests.
- No global router instance. Settings and the rule resolver are passed as a context.
- The engine is a pure function of (snapshot, settings, event sequence). An event log records the snapshot, settings and events, replays them, and compares the commit diff against a golden file. Assertions are tiered: no panic, no clearance violation in the result, exact geometry.
- Unit tests live next to the code, scenario tests under `tests/`, fixtures under `tests/fixtures/`.

## 9. Module layout

```
src/
  lib.rs
  geometry/   vec2, seg, box2, direction45, line_chain, shape, collision, hull, math
  item.rs     ItemId, Item, bodies, LayerRange, NetId, markers
  index.rs    spatial index
  joint.rs    JointMap
  node.rs     World, Node, branch, commit, queries
  line.rs     Line, assemble, walkaround on one hull
  rules.rs    RuleResolver, FixedClearance, caches
  settings.rs RoutingSettings, Sizes, enums
  walkaround.rs
  shove.rs
  optimizer.rs
  placer/     line_placer, mouse_trail, fixed_tail
  router.rs   session facade, PreviewFrame, CommitDiff
  debug.rs    DebugDecorator, event log
  topology.rs cluster assembly (minimal)
```

Modules are added milestone by milestone. A module does not appear in `lib.rs` before it has tests.

## 10. Dependencies

Zero runtime dependencies is the preference (the upstream maintainer asked for minimal dependencies). Candidates worth their cost when the module arrives: `rstar` for the R-tree (pure Rust, insertion bbox kept alongside anyway), `indexmap` for ordered maps, `bitflags`, `smallvec`. No `geo`, no clipper binding, no polygon boolean crate. Dev dependencies for property tests are fine.

## 11. Deviations from KiCad

Summarised here, detailed in the notes:

- Arena ids instead of pointers, no garbage pool, no pointer keyed caches (note 02 section 11 lists every place pointer identity mattered).
- Explicit `Linked` line vias instead of the geometric self collision heuristic.
- Effects instead of in place mutation in the shove loop.
- Deterministic obstacle ordering and iteration budgets.
- No global singleton.
- `Option` and enums instead of `-1` sentinels and out parameters.
- Dead code not ported: listed per note (01 section 14, 02 section 10, 03 section 9.5, 04 section 8.5, 05 section 7.8).
- Known KiCad quirks reproduced or fixed by explicit decision (note 03 section 9.6, note 04 finding 8).

## 12. LibrePCB integration sketch

Source: note 05 sections 5, 7.5 to 7.7. The host side lives in LibrePCB, not in this crate, and is tracked in doc/work/006-librepcb-integration.md.

- A snapshot builder in rust-core walks the board data passed over FFI: traces to segments, vias to via plus hole, pads to one solid per layer plus hole, board outline to a zero width all layer non routable solid, mechanical holes. Planes are not synced, matching KiCad and Horizon; plane fragments are rebuilt after commit.
- The rule resolver is a table lookup over LibrePCB's net class and board rules combined with `max`, seven of KiCad's thirteen constraint types apply, clearance epsilon is zero.
- `max_clearance` is the maximum over the board rule, every net class and every per pad override.
- The commit applier turns the diff into one undo command group, stitching free floating segments into net segments with junctions. Endpoint resolution against existing pads and junctions happens in the apply step, because a deferred command group cannot see its own earlier writes (Horizon's trap, note 05 finding 9).
- Preview rendering uses the existing graphics items; four styles: head, hover, semi solid, collision.
