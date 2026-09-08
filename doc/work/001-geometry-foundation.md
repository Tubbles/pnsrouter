# 001 Geometry foundation

Status: implemented (2026-09-08)

## Goal

The integer geometry the router core needs, with KiCad's tolerances, as listed in doc/reference/kicad/01-geometry.md section 1.

## Tasks

- [x] `Vec2` (i32), `Vec2L` (i64 differences), dot, cross, squared norm, `euclidean_norm` with KiCad's 45 degree special cases, `resize`, `kiround`, `rescale` (i128), `isqrt`.
- [x] `Seg`: distance, squared distance, nearest point, intersect, collinear, approx parallel, contains with the `<= 3` squared tolerance, line project and line distance.
- [x] `Box2` as `Option` based type: merge, contains, intersects, inflate, center, squared distance.
- [x] `Direction45`: octant enum, angle classification as bitflags, `build_initial_trace` (45 degree and 90 degree modes), `MIN_PRECISION_IU` endpoint re-snap, left/right/opposite, is diagonal.
- [x] `LineChain` without arcs: append with duplicate suppression, insert, replace, remove, slice as `Result`, split, reverse, simplify and simplify2 (both), nearest point, path length, find, point inside, self intersecting, intersect returning a Vec with `Hit::Segment | Hit::Corner`, closed chains with `segment_count == point_count`.
- [x] `Shape` enum: circle, rect (with radius field), segment, simple (closed line chain newtype), compound. No polygon set.
- [x] Collision dispatch as a match over shape pairs with clearance, actual distance, location, and a minimum translation vector with one documented sign convention.
- [x] Hulls: octagonal hull, segment hull with the kink threshold and its compensations, convex hull (monotone chain).
- [x] Tests mirroring KiCad's `qa/tests/libs/kimath` for line chain, segment, vector2, box2, util, circle, compound collision, nearest points, intersection. New tables for Direction45 and the collision matrix since KiCad has none.

## Acceptance

`cargo test` green, clippy clean, every public item documented, every routine cites its KiCad origin.

## References

doc/reference/kicad/01-geometry.md, especially sections 13 (constants) and 14 (Rust mapping).
