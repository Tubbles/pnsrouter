// SPDX-License-Identifier: GPL-3.0-or-later

//! Pure value geometry, ported from KiCad's `libs/kimath`.
//!
//! The layer knows nothing about nets, layers or items. Coordinates are
//! `i32` nanometres, which covers about plus or minus 2.1 metres, and every
//! product, norm or determinant of two coordinates is computed in `i64` from
//! operands that were widened first. See `DESIGN.md` section 2 and
//! `doc/reference/kicad/01-geometry.md` sections 2 and 14.1.
//!
//! Modules are added as the port progresses. So far:
//!
//! - [`math`]: rounding, rational rescaling, integer square root and the
//!   degree angle with the rotation the arcs need.
//! - [`vec2`]: the two integer vector types.
//! - [`seg`]: line segments, with the distance, intersection and
//!   collinearity tolerances the router depends on.
//! - [`box2`]: axis aligned bounding boxes, always normalised, with `i64`
//!   coordinates so a clearance inflation cannot overflow.
//! - [`direction45`]: the octant directions of the 45 degree routing
//!   regime, their angle classification and the initial trace builder.
//! - [`line_chain`]: polylines with a closed flag and a width, the
//!   container the router's lines, hulls and outlines are all made of,
//!   with the intersection, collision, distance, nearest point,
//!   containment and self intersection queries the walkaround, the shove
//!   and the optimizer ask of them.
//! - [`shape`]: the enum that replaces KiCad's `SHAPE` hierarchy, with
//!   the bounding box, centre and translation each variant needs, and the
//!   accessors the hull builders read off it.
//! - [`collision`]: the dispatch between two shapes, as an exhaustive
//!   `match` over the pair, with one minimum translation vector sign
//!   convention: the vector displaces the second argument.
//! - [`arc`]: circular arcs in KiCad's three point form, with every
//!   derived value computed on demand and none cached, the polyline
//!   approximation the collision layer falls back to, and the point,
//!   segment and nearest point primitives the arc collision rows in
//!   [`collision`] are built from.
//! - [`hull`]: the octagons the walkaround and the shove walk around,
//!   built around a rectangle, a capsule, an arc or a polygon assumed
//!   convex, always clockwise, the monotone chain the item model uses in
//!   place of KiCad's one polygon boolean, plus the filter that turns a
//!   raw chain intersection into the crossings the walkaround can use.

pub mod arc;
pub mod box2;
pub mod collision;
pub mod direction45;
pub mod hull;
pub mod line_chain;
pub mod math;
pub mod seg;
pub mod shape;
pub mod vec2;

pub use arc::{
  ArcCenter, ShapeArc, arc_to_segment_count, calc_arc_center,
  calc_arc_center_f64, calc_arc_center_from_angle,
  circle_to_end_segment_delta_radius,
};
pub use box2::Box2;
pub use collision::{
  ShapeCollision, collide, collide_arc_arc, collide_arc_arc_mtv,
  collide_arc_chain, collide_arc_chain_base, collide_arc_circle,
  collide_arc_circle_mtv, collide_arc_rect, collide_arc_rect_mtv,
  collide_arc_segment, collide_mtv, collide_point, collide_seg, collides,
};
pub use direction45::{AngleType, CornerMode, Direction45, Octant};
pub use hull::{
  ArcHullError, HULL_MARGIN, approximate_segment_as_rect, arc_hull,
  build_hull_for_primitive_shape, convex_hull, hull_intersection,
  monotone_chain_hull, octagonal_hull, segment_hull,
};
pub use line_chain::{
  ArcRef, Collision, Hit, Intersection, LineChain, PointRole, SliceError,
};
pub use seg::{NearestPoints, Seg, SegCollision};
pub use shape::{Shape, ShapeKind, SimplePolygon};
pub use vec2::{Vec2, Vec2L};
