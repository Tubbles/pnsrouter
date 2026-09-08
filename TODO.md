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
- `LineChain::PointAlong` (`shape_line_chain.cpp:2671`) is only used by the multi dragger (`pns_multi_dragger.cpp:313`), milestone 8.
- `VECTOR2D` and the `f64` vector algebra are used only by the meander placers; port with length tuning.
- `EDA_ANGLE` is used by the dragger and by `SOLID` orientation (`pns_solid.h`); `Seg::angle_degrees` returns bare `f64` degrees. Decide on an angle newtype when the item model needs solid orientation.
- `RotatePoint` from `trigo.h` (note 01 section 1.4) has callers in the router core; find them when the item model or the dragger needs rotation.
- The polygon union in `SOLID::Hull` and `HOLE::Hull` (`pns_solid.cpp:55`, `pns_hole.cpp:84`) is replaced by a convex hull of the per primitive hulls per DESIGN.md section 3. Measure against a KiCad fixture with a complex pad once fixtures exist; a tighter union may matter for dense pad rows.
- Exact `isqrt` replaces KiCad's truncated `f64` square root in distances; the two agree below about 95 mm. If a fixture disagrees above that, the distances in `seg.rs`, `line_chain.rs` and `collision.rs` are the place to look.
