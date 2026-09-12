// SPDX-License-Identifier: GPL-3.0-or-later

//! The shapes the router collides, indexes and builds hulls from.
//!
//! KiCad's `SHAPE` is a virtual hierarchy rooted at
//! `libs/kimath/include/geometry/shape.h:123`, with a runtime type tag
//! (`SHAPE_TYPE`, `shape.h:41`) that the collision dispatcher switches on
//! twice. [`Shape`] replaces it with an enum, as
//! `doc/reference/kicad/01-geometry.md` section 14.2 recommends and
//! `DESIGN.md` section 3 fixes. Three things follow from that:
//!
//! - the collision dispatcher becomes an exhaustive `match` over the pair,
//!   so a missing cell is a compile error rather than a `wxFAIL_MSG` that
//!   returns "no collision" at runtime
//!   (`libs/kimath/src/geometry/shape_collisions.cpp:1300`);
//! - nesting is representable, so [`Shape::Compound`] holding a
//!   [`Shape::Compound`] forces a decision instead of silently asserting
//!   (note 01 section 9.1). The decision is in [`Shape::subshapes`];
//! - `Clone` stops being a virtual that asserts and returns null
//!   (`shape.h:146`).
//!
//! Only the variants the router's world model can contain are here. Left
//! out on purpose:
//!
//! - `SHAPE_POLY_SET` and `SHAPE_ELLIPSE`: the router's world contains
//!   neither (note 01 sections 11 and 7.5). `SH_ELLIPSE` reaches the
//!   router only in the hull dispatcher, where it degrades to its bounding
//!   box (`pcbnew/router/pns_utils.cpp:523`).
//! - `SHAPE_NULL` (`shape_null.h:32`): grepping the router core for
//!   `SHAPE_NULL` finds nothing. The three `SH_NULL` hits under
//!   `pcbnew/router/` are `PNS::SHOVE::SHOVE_STATUS::SH_NULL`
//!   (`pns_shove.h:53`), an unrelated enumerator.
//! - `SH_POLY_SET_TRIANGLE`: zone triangles reach the router as
//!   `SHAPE_SIMPLE` (`pns_kicad_iface.cpp:1933`), which is
//!   [`Shape::Simple`].
//!
//! The corner radius of [`Shape::Rect`] is kept even though the router
//! never sets it, because the collision code branches on it
//! (`shape_collisions.cpp:70`, `:472`, `shape_rect.cpp:28`) and the branch
//! is part of the ported behaviour.

use crate::geometry::arc::ShapeArc;
use crate::geometry::box2::Box2;
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::{Vec2, Vec2L};

/// The shape's runtime type.
///
/// Port of `enum SHAPE_TYPE`,
/// `libs/kimath/include/geometry/shape.h:41`, restricted to the variants
/// [`Shape`] carries. The numeric values are part of KiCad's debug
/// serialisation format (`src/geometry/shape.cpp:43`) and are not
/// reproduced, because this crate has no such format.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum ShapeKind {
  /// An axis aligned rectangle, possibly with rounded corners.
  /// KiCad's `SH_RECT`.
  Rect,
  /// A capsule: a segment with a round cap at each end.
  /// KiCad's `SH_SEGMENT`.
  Segment,
  /// A polyline, open or closed. KiCad's `SH_LINE_CHAIN`.
  LineChain,
  /// A circle. KiCad's `SH_CIRCLE`.
  Circle,
  /// A closed polygon assumed convex. KiCad's `SH_SIMPLE`.
  Simple,
  /// A circular arc of a given width. KiCad's `SH_ARC`.
  Arc,
  /// Several shapes acting as one. KiCad's `SH_COMPOUND`.
  Compound,
}

/// A closed polygon.
///
/// Port of `SHAPE_SIMPLE`,
/// `libs/kimath/include/geometry/shape_simple.h:37`, which is a thin
/// wrapper over a [`LineChain`] that every constructor forces closed
/// (`shape_simple.h:46`, `:53`) and whose `IsClosed` returns true
/// unconditionally (`shape_simple.h:169`). The newtype is what makes that
/// invariant unbreakable here, which matters because the collision code
/// takes a containment shortcut on any closed chain
/// (`shape_collisions.cpp:246`, `:363`, `:483`).
///
/// KiCad's board interface hands the router one of these per pad outline
/// (`pcbnew/router/pns_kicad_iface.cpp:1734`) and per zone triangle
/// (`:1933`). `PNS::ConvexHull` (`pcbnew/router/pns_utils.cpp:300`)
/// assumes the polygon is convex without checking, and so does this type.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct SimplePolygon {
  /// The vertices. Always closed.
  ///
  /// Port of `m_points`,
  /// `libs/kimath/include/geometry/shape_simple.h:198`.
  points: LineChain,
}

impl SimplePolygon {
  /// Build a polygon from a chain, closing it.
  ///
  /// Port of `SHAPE_SIMPLE( const SHAPE_LINE_CHAIN& )`,
  /// `libs/kimath/include/geometry/shape_simple.h:51`, which copies the
  /// chain and then calls `SetClosed( true )` on the copy whatever the
  /// argument said.
  pub fn new(points: LineChain) -> Self {
    let mut points = points;
    points.set_closed(true);

    Self { points }
  }

  /// Build a polygon from its vertices in order.
  ///
  /// The closing vertex is implicit, as it is for any closed
  /// [`LineChain`]: do not repeat the first point at the end.
  pub fn from_points(points: Vec<Vec2>) -> Self {
    Self::new(LineChain::from_points(points, true))
  }

  /// The vertices, as a closed chain.
  ///
  /// Port of `Vertices`,
  /// `libs/kimath/include/geometry/shape_simple.h:120`. This is what the
  /// convex hull builder walks (`pcbnew/router/pns_utils.cpp:306`).
  pub fn vertices(&self) -> &LineChain {
    &self.points
  }

  /// The number of vertices.
  ///
  /// Port of `PointCount`,
  /// `libs/kimath/include/geometry/shape_simple.h:85`.
  pub fn point_count(&self) -> usize {
    self.points.point_count()
  }

  /// The vertex at an index.
  ///
  /// Port of `CPoint`,
  /// `libs/kimath/include/geometry/shape_simple.h:99`, minus the negative
  /// index wrap: indices are explicit here, as note 01 section 14.3 asks.
  ///
  /// # Panics
  ///
  /// When the index is out of range, where KiCad is undefined.
  pub fn point(&self, index: usize) -> Vec2 {
    self.points.point(index)
  }

  /// Add a vertex at the end.
  ///
  /// Port of `Append( const VECTOR2I& )`,
  /// `libs/kimath/include/geometry/shape_simple.h:145`, duplicate
  /// suppression included, since it goes through [`LineChain::append`].
  pub fn append(&mut self, point: Vec2) {
    self.points.append(point);
  }

  /// Translate every vertex.
  ///
  /// Port of `Move`,
  /// `libs/kimath/include/geometry/shape_simple.h:161`.
  pub fn move_by(&mut self, delta: Vec2) {
    self.points.move_by(delta);
  }

  /// The bounding box, grown by a clearance.
  ///
  /// Port of `BBox`,
  /// `libs/kimath/include/geometry/shape_simple.h:74`, which forwards to
  /// the chain. Note that [`LineChain::bbox`] grows by the clearance plus
  /// the **whole** nominal width, not half of it, which is KiCad's
  /// behaviour at `shape_line_chain.h:457`.
  pub fn bbox(&self, clearance: i32) -> Option<Box2> {
    self.points.bbox(clearance)
  }
}

/// A shape in the plane.
///
/// Port of the `SHAPE` hierarchy,
/// `libs/kimath/include/geometry/shape.h:123`, as an enum. See the module
/// documentation for the variants that were left out and why.
///
/// The width folded into [`Shape::Segment`] is a full width: the capsule
/// reaches `width / 2` on each side of the segment, with a round cap at
/// each end (`shape_segment.h:33`). [`Shape::Arc`] carries a full width
/// the same way. The width carried by the chain inside
/// [`Shape::LineChain`] and [`Shape::Simple`] is **ignored** by every
/// collision routine, which is why `PNS::ITEM::collideSimple` folds line
/// widths into the clearance instead (`pcbnew/router/pns_item.cpp:159`).
/// An arc a chain **stores** has width zero, which the collision code
/// asserts (`shape_collisions.cpp:686`).
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Shape {
  /// A circle.
  ///
  /// Port of `SHAPE_CIRCLE`,
  /// `libs/kimath/include/geometry/shape_circle.h:33`, which wraps a
  /// `CIRCLE` (`geometry/circle.h:150`). `PNS::VIA` stores one of these
  /// per layer (`pcbnew/router/pns_via.h:349`).
  Circle {
    /// The centre.
    center: Vec2,
    /// The radius in nanometres.
    radius: i32,
  },
  /// An axis aligned rectangle with an optional corner radius.
  ///
  /// Port of `SHAPE_RECT`,
  /// `libs/kimath/include/geometry/shape_rect.h:34`, whose state is a top
  /// left corner, a width, a height and a corner radius
  /// (`shape_rect.h:249`). The router builds one only in
  /// `PNS::ApproximateSegmentAsRect` (`pcbnew/router/pns_utils.cpp:356`),
  /// which leaves the radius at zero, but the collision code branches on
  /// the radius so the field is kept.
  ///
  /// A negative component in `size` is representable, as it is in KiCad,
  /// and makes the containment test in the rectangle versus circle cell
  /// (`shape_collisions.cpp:98`) never fire. KiCad has a `Normalize`
  /// (`shape_rect.cpp:155`) for that; the router never calls it and it is
  /// not ported.
  Rect {
    /// The corner with the smaller coordinates, KiCad's `m_p0`.
    origin: Vec2,
    /// The width and height, KiCad's `m_w` and `m_h`.
    size: Vec2,
    /// The corner radius, KiCad's `m_radius`.
    radius: i32,
  },
  /// A capsule: a segment with a round cap of `width / 2` at each end.
  ///
  /// Port of `SHAPE_SEGMENT`,
  /// `libs/kimath/include/geometry/shape_segment.h:33`. `PNS::SEGMENT`
  /// holds one by value (`pcbnew/router/pns_segment.h:146`), and so does a
  /// slot shaped hole (`pcbnew/router/pns_hole.cpp:131`).
  Segment {
    /// The spine of the capsule.
    seg: Seg,
    /// The full width in nanometres.
    width: i32,
  },
  /// A closed polygon assumed convex.
  ///
  /// Port of `SHAPE_SIMPLE`,
  /// `libs/kimath/include/geometry/shape_simple.h:37`. See
  /// [`SimplePolygon`].
  Simple(SimplePolygon),
  /// A polyline, open or closed, with no width.
  ///
  /// Port of `SHAPE_LINE_CHAIN`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:77`. `PNS::LINE`
  /// hands one of these straight to the collision code
  /// (`pcbnew/router/pns_line.h:138`), so the router really does collide
  /// bare open chains as shapes.
  LineChain(LineChain),
  /// A circular arc of a given width.
  ///
  /// Port of `SHAPE_ARC`,
  /// `libs/kimath/include/geometry/shape_arc.h:35`. `PNS::ARC` holds one
  /// by value (`pcbnew/router/pns_arc.h:96`) and hands it to the
  /// collision code with the track width still on it, which is why the
  /// arc rows in [`crate::geometry::collision`] apply the half width and
  /// the polyline rows do not.
  ///
  /// The width is a full width, like [`Shape::Segment`]'s: the copper
  /// reaches `width / 2` on each side of the curve.
  Arc(ShapeArc),
  /// Several shapes acting as one.
  ///
  /// Port of `SHAPE_COMPOUND`,
  /// `libs/kimath/include/geometry/shape_compound.h:34`, which represents
  /// a complex pad or a slot hole. The router touches it in exactly two
  /// places, both hull builders (`pcbnew/router/pns_solid.cpp:46`,
  /// `pns_hole.cpp:75`).
  ///
  /// KiCad refuses to nest: `AddShape` flattens any child that reports
  /// indexable subshapes (`shape_compound.h:85`) and the collision
  /// dispatcher has no case for a compound inside a compound
  /// (`shape_collisions.cpp:1337`). Here nesting is representable, and
  /// [`Shape::subshapes`] flattens it.
  Compound(Vec<Shape>),
}

impl Shape {
  /// The smallest distance any point of a shape is meant to be trusted
  /// to, in nanometres.
  ///
  /// Port of `SHAPE::MIN_PRECISION_IU`,
  /// `libs/kimath/include/geometry/shape.h:129`. `CIRCLE::Contains` and
  /// `CIRCLE::IntersectLine` use it as a band tolerance around the
  /// circumference (`libs/kimath/src/geometry/circle.cpp:191`, `:349`,
  /// `:353`), which is how a tangent line is told apart from a secant.
  pub const MIN_PRECISION_IU: i32 = 4;

  /// A circle of a given centre and radius.
  ///
  /// Port of `SHAPE_CIRCLE( const VECTOR2I&, int )`,
  /// `libs/kimath/include/geometry/shape_circle.h:41`.
  pub fn circle(center: Vec2, radius: i32) -> Self {
    Self::Circle { center, radius }
  }

  /// A sharp cornered rectangle from its top left corner and its size.
  ///
  /// Port of `SHAPE_RECT( const VECTOR2I&, int, int )`,
  /// `libs/kimath/include/geometry/shape_rect.h:70`, which leaves the
  /// corner radius at zero like every other constructor except the copy
  /// one.
  pub fn rect(origin: Vec2, size: Vec2) -> Self {
    Self::Rect {
      origin,
      size,
      radius: 0,
    }
  }

  /// A rectangle with rounded corners.
  ///
  /// Port of `SHAPE_RECT` followed by `SetRadius`,
  /// `libs/kimath/include/geometry/shape_rect.h:203`. The router never
  /// builds one; see the note on [`Shape::Rect`] and the deviation
  /// documented on [`crate::geometry::collision`].
  pub fn rounded_rect(origin: Vec2, size: Vec2, radius: i32) -> Self {
    Self::Rect {
      origin,
      size,
      radius,
    }
  }

  /// A capsule of a given spine and width.
  ///
  /// Port of `SHAPE_SEGMENT( const SEG&, int )`,
  /// `libs/kimath/include/geometry/shape_segment.h:46`.
  pub fn segment(seg: Seg, width: i32) -> Self {
    Self::Segment { seg, width }
  }

  /// A closed polygon from a chain, closing the chain.
  ///
  /// Port of `SHAPE_SIMPLE( const SHAPE_LINE_CHAIN& )`,
  /// `libs/kimath/include/geometry/shape_simple.h:51`.
  pub fn simple(points: LineChain) -> Self {
    Self::Simple(SimplePolygon::new(points))
  }

  /// A polyline shape.
  ///
  /// The chain keeps whatever open or closed flag it arrived with, which
  /// is what makes this different from [`Shape::simple`].
  pub fn line_chain(chain: LineChain) -> Self {
    Self::LineChain(chain)
  }

  /// A circular arc shape.
  ///
  /// Port of `SHAPE_ARC( const SHAPE_ARC& )`,
  /// `libs/kimath/include/geometry/shape_arc.h:44`. The width travels
  /// with the arc, so there is nothing to pass beside it.
  pub const fn arc(arc: ShapeArc) -> Self {
    Self::Arc(arc)
  }

  /// Several shapes acting as one.
  ///
  /// Port of `SHAPE_COMPOUND( const std::vector<SHAPE*>& )`,
  /// `libs/kimath/src/geometry/shape_compound.cpp:37`. Unlike
  /// `SHAPE_COMPOUND::AddShape` (`shape_compound.h:79`) this does not
  /// flatten nested compounds on the way in; [`Shape::subshapes`] flattens
  /// them on the way out instead, so a caller that built a tree gets it
  /// back unchanged.
  pub fn compound(shapes: Vec<Shape>) -> Self {
    Self::Compound(shapes)
  }

  /// The runtime type.
  ///
  /// Port of `SHAPE_BASE::Type`,
  /// `libs/kimath/include/geometry/shape.h:93`.
  pub fn kind(&self) -> ShapeKind {
    match self {
      Self::Circle { .. } => ShapeKind::Circle,
      Self::Rect { .. } => ShapeKind::Rect,
      Self::Segment { .. } => ShapeKind::Segment,
      Self::Simple(_) => ShapeKind::Simple,
      Self::LineChain(_) => ShapeKind::LineChain,
      Self::Arc(_) => ShapeKind::Arc,
      Self::Compound(_) => ShapeKind::Compound,
    }
  }

  /// Whether the shape encloses an area rather than being a bare outline.
  ///
  /// Port of the `SHAPE::IsSolid` virtual,
  /// `libs/kimath/include/geometry/shape.h:292`. Every ported variant
  /// returns true (`shape_rect.h:228`, `shape_circle.h:139`,
  /// `shape_segment.h:159`, `shape_simple.h:163`, `shape_arc.h:216`,
  /// `shape_compound.cpp:105`) except [`Shape::LineChain`], which returns
  /// false (`shape_line_chain.h:808`), even when the chain is closed.
  ///
  /// The router core never reads it; `SHAPE::IsSolid` is consulted by the
  /// DRC and the plotters. It is here because it is part of the ported
  /// interface and it is one line.
  pub fn is_solid(&self) -> bool {
    !matches!(self, Self::LineChain(_))
  }

  /// The bounding box, grown by a clearance on every side.
  ///
  /// Port of the `SHAPE::BBox` virtual,
  /// `libs/kimath/include/geometry/shape.h:223`, dispatched per variant:
  ///
  /// - circle: `BOX2I( center - (r + cl), 2 * (r + cl) )`, so the box
  ///   includes the radius (`shape_circle.h:67`);
  /// - rectangle: `BOX2I( p0 - cl, (w + 2 cl, h + 2 cl) )`, where the
  ///   second argument is a size, not a corner (`shape_rect.h:105`). The
  ///   corner radius does not shrink it, exactly as in KiCad;
  /// - capsule: `BOX2I( A, B - A ).Inflate( cl + (width + 1) / 2 )`, so
  ///   the box includes the width, with the half width **rounded up**
  ///   (`shape_segment.h:63`);
  /// - polyline and polygon: the chain's own box, which grows by the
  ///   clearance plus the whole nominal width
  ///   (`shape_line_chain.h:457`, `shape_simple.h:74`);
  /// - arc: the box over the three points and every axis quadrant point
  ///   the sweep crosses, inflated by `kiround(width / 2) + 1` when the
  ///   width is non zero and then by the clearance
  ///   (`shape_arc.cpp:462`). See [`ShapeArc::bbox`];
  /// - compound: the union of the children's boxes
  ///   (`shape_compound.cpp:71`).
  ///
  /// Returns `None` where KiCad returns its uninitialised `BOX2I`: an
  /// empty chain and an empty compound.
  ///
  /// Deviation: `SHAPE_COMPOUND::BBox` ignores its `aClearance` argument
  /// and calls `BBox()` on each child (`shape_compound.cpp:78`, `:81`).
  /// That silently under grows the box, and the spatial index is built out
  /// of these boxes, so a query that trusts it can miss an obstacle. The
  /// clearance is passed down here.
  pub fn bbox(&self, clearance: i32) -> Option<Box2> {
    let clearance = i64::from(clearance);

    match self {
      Self::Circle { center, radius } => {
        let grown = i64::from(*radius) + clearance;
        let corner = Vec2L::new(grown, grown);
        let origin = Vec2L::from(*center) - corner;

        Some(Box2::from_origin_and_size(origin, corner * 2))
      }
      Self::Rect { origin, size, .. } => {
        let corner = Vec2L::new(clearance, clearance);
        let grown_size = Vec2L::from(*size) + corner * 2;

        Some(Box2::from_origin_and_size(
          Vec2L::from(*origin) - corner,
          grown_size,
        ))
      }
      Self::Segment { seg, width } => {
        let half_width = (i64::from(*width) + 1) / 2;

        Some(
          Box2::from_vec2_corners(seg.a, seg.b)
            .inflate_by(clearance + half_width),
        )
      }
      Self::Simple(polygon) => polygon.bbox(saturate_i32(clearance)),
      Self::LineChain(chain) => chain.bbox(saturate_i32(clearance)),
      Self::Arc(arc) => Some(arc.bbox(saturate_i32(clearance))),
      Self::Compound(shapes) => shapes
        .iter()
        .filter_map(|shape| shape.bbox(saturate_i32(clearance)))
        .reduce(Box2::merge),
    }
  }

  /// The centre of the bounding box.
  ///
  /// Port of `SHAPE::Centre`,
  /// `libs/kimath/include/geometry/shape.h:230`, which is
  /// `BBox( 0 ).Centre()` for every ported variant, because none of them
  /// overrides it. `BOX2::Centre` truncates towards the origin
  /// (`math/box2.h:94`), so the centre of a box of odd width sits one
  /// nanometre short. The rectangle versus polyline cell depends on this
  /// value (`shape_collisions.cpp:483`).
  ///
  /// Returns `None` for the shapes that have no bounding box, where KiCad
  /// reads the centre of an uninitialised `BOX2I`.
  pub fn center(&self) -> Option<Vec2> {
    Some(self.bbox(0)?.center().saturating_to_vec2())
  }

  /// Translate the shape.
  ///
  /// Port of the `SHAPE::Move` virtual,
  /// `libs/kimath/include/geometry/shape.h:290` (`shape_circle.h:129`,
  /// `shape_rect.h:206`, `shape_segment.h:170`, `shape_simple.h:161`,
  /// `shape_line_chain.cpp:1128`, `shape_arc.cpp:1079`,
  /// `shape_compound.cpp:85`).
  ///
  /// # Panics
  ///
  /// In a debug build when a coordinate leaves `i32`, because
  /// [`Vec2`] addition panics there. KiCad wraps.
  pub fn move_by(&mut self, delta: Vec2) {
    match self {
      Self::Circle { center, .. } => *center += delta,
      Self::Rect { origin, .. } => *origin += delta,
      Self::Segment { seg, .. } => {
        seg.a += delta;
        seg.b += delta;
      }
      Self::Simple(polygon) => polygon.move_by(delta),
      Self::LineChain(chain) => chain.move_by(delta),
      Self::Arc(arc) => arc.move_by(delta),
      Self::Compound(shapes) => {
        for shape in shapes {
          shape.move_by(delta);
        }
      }
    }
  }

  /// The shape itself, or the leaves of a compound, in order.
  ///
  /// Port of `SHAPE_BASE::GetIndexableSubshapes`,
  /// `libs/kimath/include/geometry/shape_compound.h:148`, extended to
  /// recurse.
  ///
  /// Deliberate extension: KiCad cannot nest compounds. `AddShape`
  /// flattens a compound child on the way in (`shape_compound.h:85`), and
  /// if one gets in anyway the collision dispatcher reaches
  /// `collideSingleShapes`, which has no `SH_COMPOUND` case, asserts in
  /// debug and returns "no collision" in release
  /// (`shape_collisions.cpp:1337`, `:1300`, note 01 section 9.1). Nesting
  /// is representable here, so it is flattened recursively instead. A
  /// silently missed collision is the one outcome that is not acceptable,
  /// because the router would route a trace through an obstacle.
  pub fn subshapes(&self) -> Vec<&Shape> {
    let mut leaves = Vec::new();
    self.collect_subshapes(&mut leaves);

    leaves
  }

  /// The recursion behind [`Shape::subshapes`].
  fn collect_subshapes<'a>(&'a self, leaves: &mut Vec<&'a Shape>) {
    match self {
      Self::Compound(shapes) => {
        for shape in shapes {
          shape.collect_subshapes(leaves);
        }
      }
      other => leaves.push(other),
    }
  }
}

/// The four sharp corners of a rectangle, in the order the collision code
/// walks them.
///
/// Port of the `vts` array at
/// `libs/kimath/src/geometry/shape_collisions.cpp:85` and the `corners`
/// array at `libs/kimath/src/geometry/shape_rect.cpp:58`, which agree:
/// top left, bottom left, bottom right, top right. The closing repeat of
/// the first corner that both arrays carry is left to the caller.
pub(crate) fn rect_corners(origin: Vec2, size: Vec2) -> [Vec2; 4] {
  [
    origin,
    Vec2::new(origin.x, origin.y + size.y),
    Vec2::new(origin.x + size.x, origin.y + size.y),
    Vec2::new(origin.x + size.x, origin.y),
  ]
}

/// The outline of a rectangle as a closed chain.
///
/// Port of `SHAPE_RECT::Outline`,
/// `libs/kimath/src/geometry/shape_rect.cpp:141`, which polygonises the
/// rectangle and returns the first outline. For a sharp cornered
/// rectangle that is the four corners in the order top left, top right,
/// bottom right, bottom left (`shape_rect.cpp:132`), which is the reverse
/// winding of [`rect_corners`].
///
/// Deviation: KiCad routes a rectangle with a non zero corner radius
/// through `ROUNDRECT::TransformToPolygon` (`shape_rect.cpp:126`), which
/// approximates each corner arc to within `ARC_HIGH_DEF`, 5000 nm. That
/// polygoniser is not ported, so the outline here has sharp corners
/// whatever the radius says. The sharp outline **contains** the rounded
/// one, so the collision routines that go through it over report near a
/// corner and never under report, which is the safe direction for a
/// router. Nothing in the router core can reach this: the only
/// `SHAPE_RECT` it builds leaves the radius at zero
/// (`pcbnew/router/pns_utils.cpp:356`).
pub(crate) fn rect_outline(origin: Vec2, size: Vec2) -> LineChain {
  LineChain::from_points(
    vec![
      origin,
      Vec2::new(origin.x + size.x, origin.y),
      Vec2::new(origin.x + size.x, origin.y + size.y),
      Vec2::new(origin.x, origin.y + size.y),
    ],
    true,
  )
}

/// Clamp an `i64` into an `i32`.
///
/// The saturation is this crate's answer to KiCad's silent narrowing
/// (note 01 section 9.6). It only fires on values no board can produce.
fn saturate_i32(value: i64) -> i32 {
  value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::math::Degrees;

  /// Shorthand for a point.
  fn point(x: i32, y: i32) -> Vec2 {
    Vec2::new(x, y)
  }

  /// Shorthand for a box corner.
  fn long(x: i64, y: i64) -> Vec2L {
    Vec2L::new(x, y)
  }

  #[test]
  fn kind_names_every_variant() {
    assert_eq!(Shape::circle(point(0, 0), 10).kind(), ShapeKind::Circle);
    assert_eq!(
      Shape::rect(point(0, 0), point(10, 10)).kind(),
      ShapeKind::Rect
    );
    assert_eq!(
      Shape::segment(Seg::from_coords(0, 0, 10, 0), 4).kind(),
      ShapeKind::Segment
    );
    assert_eq!(
      Shape::simple(LineChain::from_slice(
        &[point(0, 0), point(10, 0), point(10, 10)],
        false
      ))
      .kind(),
      ShapeKind::Simple
    );
    assert_eq!(
      Shape::line_chain(LineChain::from_slice(
        &[point(0, 0), point(10, 0)],
        false
      ))
      .kind(),
      ShapeKind::LineChain
    );
    assert_eq!(
      Shape::arc(ShapeArc::new(point(0, 0), point(5, 5), point(10, 0), 4))
        .kind(),
      ShapeKind::Arc
    );
    assert_eq!(Shape::compound(Vec::new()).kind(), ShapeKind::Compound);
  }

  #[test]
  fn only_the_bare_chain_is_not_solid() {
    assert!(Shape::circle(point(0, 0), 10).is_solid());
    assert!(Shape::rect(point(0, 0), point(10, 10)).is_solid());
    assert!(Shape::segment(Seg::from_coords(0, 0, 10, 0), 4).is_solid());
    assert!(
      Shape::simple(LineChain::from_slice(
        &[point(0, 0), point(10, 0), point(10, 10)],
        true
      ))
      .is_solid()
    );
    assert!(Shape::compound(Vec::new()).is_solid());
    assert!(
      Shape::arc(ShapeArc::new(point(0, 0), point(5, 5), point(10, 0), 4))
        .is_solid()
    );

    let mut closed = LineChain::from_slice(&[point(0, 0), point(10, 0)], true);
    closed.set_closed(true);
    assert!(!Shape::line_chain(closed).is_solid());
  }

  /// `SHAPE_SIMPLE` forces the chain closed whatever the caller said,
  /// `libs/kimath/include/geometry/shape_simple.h:53`.
  #[test]
  fn simple_polygon_is_always_closed() {
    let open = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);
    let polygon = SimplePolygon::new(open);

    assert!(polygon.vertices().is_closed());
    assert_eq!(polygon.point_count(), 2);
    assert_eq!(polygon.vertices().segment_count(), 2);
  }

  /// `SHAPE_CIRCLE::BBox`,
  /// `libs/kimath/include/geometry/shape_circle.h:67`.
  #[test]
  fn circle_bbox_includes_the_radius() {
    let circle = Shape::circle(point(100, 200), 50);

    let plain = circle.bbox(0).unwrap();
    assert_eq!(plain.origin(), long(50, 150));
    assert_eq!(plain.end(), long(150, 250));

    let grown = circle.bbox(10).unwrap();
    assert_eq!(grown.origin(), long(40, 140));
    assert_eq!(grown.end(), long(160, 260));
  }

  /// `SHAPE_RECT::BBox`,
  /// `libs/kimath/include/geometry/shape_rect.h:105`, where the second
  /// argument of the `BOX2I` is a size and not a corner.
  #[test]
  fn rect_bbox_grows_by_the_clearance_on_every_side() {
    let rect = Shape::rect(point(10, 20), point(30, 40));

    let plain = rect.bbox(0).unwrap();
    assert_eq!(plain.origin(), long(10, 20));
    assert_eq!(plain.end(), long(40, 60));

    let grown = rect.bbox(5).unwrap();
    assert_eq!(grown.origin(), long(5, 15));
    assert_eq!(grown.end(), long(45, 65));
  }

  /// `SHAPE_SEGMENT::BBox`,
  /// `libs/kimath/include/geometry/shape_segment.h:63`, which rounds the
  /// half width **up**, unlike the collision code.
  #[test]
  fn segment_bbox_includes_the_width_rounded_up() {
    let capsule = Shape::segment(Seg::from_coords(0, 0, 100, 0), 7);

    let plain = capsule.bbox(0).unwrap();
    assert_eq!(plain.origin(), long(-4, -4));
    assert_eq!(plain.end(), long(104, 4));

    let grown = capsule.bbox(6).unwrap();
    assert_eq!(grown.origin(), long(-10, -10));
    assert_eq!(grown.end(), long(110, 10));
  }

  /// A capsule whose spine runs backwards still has a normalised box.
  #[test]
  fn segment_bbox_normalises_the_spine() {
    let capsule = Shape::segment(Seg::from_coords(100, 50, 0, 0), 0);
    let box2 = capsule.bbox(0).unwrap();

    assert_eq!(box2.origin(), long(0, 0));
    assert_eq!(box2.end(), long(100, 50));
  }

  /// `SHAPE_LINE_CHAIN::BBox`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:457`, grows by the
  /// clearance plus the **whole** width.
  #[test]
  fn chain_bbox_grows_by_the_whole_width() {
    let mut chain = LineChain::from_slice(&[point(0, 0), point(100, 0)], false);
    chain.set_width(10);

    let box2 = Shape::line_chain(chain).bbox(5).unwrap();
    assert_eq!(box2.origin(), long(-15, -15));
    assert_eq!(box2.end(), long(115, 15));
  }

  #[test]
  fn empty_shapes_have_no_bbox() {
    assert!(Shape::compound(Vec::new()).bbox(0).is_none());
    assert!(Shape::line_chain(LineChain::new()).bbox(0).is_none());
    assert!(Shape::compound(Vec::new()).center().is_none());
  }

  /// Deviation from `SHAPE_COMPOUND::BBox`,
  /// `libs/kimath/src/geometry/shape_compound.cpp:78`, which drops the
  /// clearance.
  #[test]
  fn compound_bbox_is_the_union_grown_by_the_clearance() {
    let compound = Shape::compound(vec![
      Shape::circle(point(0, 0), 10),
      Shape::circle(point(100, 0), 10),
    ]);

    let box2 = compound.bbox(5).unwrap();
    assert_eq!(box2.origin(), long(-15, -15));
    assert_eq!(box2.end(), long(115, 15));
  }

  /// `SHAPE::Centre` truncates towards the origin, because `BOX2::Centre`
  /// does (`libs/kimath/include/math/box2.h:94`).
  #[test]
  fn center_truncates_towards_the_origin() {
    let rect = Shape::rect(point(0, 0), point(11, 11));
    assert_eq!(rect.center(), Some(point(5, 5)));

    let circle = Shape::circle(point(-3, 7), 4);
    assert_eq!(circle.center(), Some(point(-3, 7)));
  }

  #[test]
  fn move_by_translates_every_variant() {
    let delta = point(10, -20);

    let mut circle = Shape::circle(point(0, 0), 5);
    circle.move_by(delta);
    assert_eq!(circle, Shape::circle(point(10, -20), 5));

    let mut rect = Shape::rect(point(0, 0), point(4, 4));
    rect.move_by(delta);
    assert_eq!(rect, Shape::rect(point(10, -20), point(4, 4)));

    let mut capsule = Shape::segment(Seg::from_coords(0, 0, 4, 0), 2);
    capsule.move_by(delta);
    assert_eq!(
      capsule,
      Shape::segment(Seg::from_coords(10, -20, 14, -20), 2)
    );

    let mut compound = Shape::compound(vec![Shape::circle(point(0, 0), 5)]);
    compound.move_by(delta);
    assert_eq!(
      compound,
      Shape::compound(vec![Shape::circle(point(10, -20), 5)])
    );

    let mut arc =
      Shape::arc(ShapeArc::new(point(0, 0), point(5, 5), point(10, 0), 4));
    arc.move_by(delta);
    assert_eq!(
      arc,
      Shape::arc(ShapeArc::new(
        point(10, -20),
        point(15, -15),
        point(20, -20),
        4
      ))
    );
  }

  /// The port's own: [`Shape::Arc`] answers the three queries the router
  /// asks of every shape, and they agree with the ones
  /// [`ShapeArc`] answers directly.
  ///
  /// `SHAPE_ARC::BBox` inflates by `kiround( width / 2 ) + 1` whenever
  /// the width is non zero (`shape_arc.cpp:466`), so the box of a wide
  /// arc is one nanometre larger than the copper on every side, and
  /// `SHAPE::Centre` is the truncating centre of that box.
  #[test]
  fn arc_round_trips_through_bbox_center_and_move_by() {
    let quarter = ShapeArc::from_center_start_angle(
      point(0, 0),
      point(1_000_000, 0),
      Degrees::new(90.0),
      200_000,
    );
    let shape = Shape::arc(quarter);

    assert_eq!(shape.bbox(0), Some(quarter.bbox(0)));
    assert_eq!(shape.bbox(50_000), Some(quarter.bbox(50_000)));

    // The quarter crosses no axis quadrant point between its ends, so
    // the box is the three points plus the half width and the `+ 1`.
    let plain = shape.bbox(0).unwrap();
    assert_eq!(plain.origin(), long(-100_001, -100_001));
    assert_eq!(plain.end(), long(1_100_001, 1_100_001));
    assert_eq!(shape.center(), Some(point(500_000, 500_000)));

    // A clearance grows the box by exactly the clearance on each side,
    // and the centre does not move.
    let grown = shape.bbox(50_000).unwrap();
    assert_eq!(grown.origin(), long(-150_001, -150_001));
    assert_eq!(grown.end(), long(1_150_001, 1_150_001));

    // Moving is exact in all three points, and the box follows.
    let delta = point(-250_000, 750_000);
    let mut moved = shape.clone();
    moved.move_by(delta);

    assert_eq!(
      moved,
      Shape::arc(ShapeArc::new(
        quarter.start() + delta,
        quarter.arc_mid() + delta,
        quarter.end() + delta,
        quarter.width()
      ))
    );
    assert_eq!(moved.center(), shape.center().map(|center| center + delta));

    let moved_box = moved.bbox(0).unwrap();
    assert_eq!(moved_box.origin(), plain.origin() + Vec2L::from(delta));
    assert_eq!(moved_box.end(), plain.end() + Vec2L::from(delta));
  }

  /// The deliberate extension over `shape_collisions.cpp:1337`.
  #[test]
  fn subshapes_flattens_nested_compounds() {
    let inner = Shape::compound(vec![
      Shape::circle(point(0, 0), 1),
      Shape::circle(point(1, 0), 1),
    ]);
    let outer = Shape::compound(vec![
      Shape::circle(point(2, 0), 1),
      inner,
      Shape::compound(vec![Shape::compound(vec![Shape::circle(
        point(3, 0),
        1,
      )])]),
    ]);

    let leaves = outer.subshapes();
    assert_eq!(leaves.len(), 4);
    assert_eq!(*leaves[0], Shape::circle(point(2, 0), 1));
    assert_eq!(*leaves[1], Shape::circle(point(0, 0), 1));
    assert_eq!(*leaves[2], Shape::circle(point(1, 0), 1));
    assert_eq!(*leaves[3], Shape::circle(point(3, 0), 1));
  }

  #[test]
  fn subshapes_of_a_leaf_is_the_leaf() {
    let circle = Shape::circle(point(0, 0), 1);
    let leaves = circle.subshapes();

    assert_eq!(leaves.len(), 1);
    assert_eq!(*leaves[0], circle);
  }

  /// Mirrors `qa/tests/libs/kimath/geometry/test_shape_rect_corner.cpp:25`,
  /// the only assertion that file makes. It does not exercise collision.
  #[test]
  fn rect_keeps_its_corner_radius() {
    let rect = Shape::rounded_rect(point(0, 0), point(10, 10), 2);

    match rect {
      Shape::Rect { radius, .. } => assert_eq!(radius, 2),
      other => panic!("expected a rectangle, got {other:?}"),
    }
  }

  /// The corner order the two KiCad arrays agree on.
  #[test]
  fn rect_corners_run_anticlockwise_on_screen() {
    let corners = rect_corners(point(10, 20), point(30, 40));

    assert_eq!(corners[0], point(10, 20));
    assert_eq!(corners[1], point(10, 60));
    assert_eq!(corners[2], point(40, 60));
    assert_eq!(corners[3], point(40, 20));
  }

  /// `SHAPE_RECT::Outline` winds the other way,
  /// `libs/kimath/src/geometry/shape_rect.cpp:132`.
  #[test]
  fn rect_outline_is_closed_and_winds_the_other_way() {
    let outline = rect_outline(point(10, 20), point(30, 40));

    assert!(outline.is_closed());
    assert_eq!(outline.point_count(), 4);
    assert_eq!(outline.segment_count(), 4);
    assert_eq!(outline.point(0), point(10, 20));
    assert_eq!(outline.point(1), point(40, 20));
    assert_eq!(outline.point(2), point(40, 60));
    assert_eq!(outline.point(3), point(10, 60));
  }
}
