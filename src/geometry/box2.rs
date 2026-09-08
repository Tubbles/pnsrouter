// SPDX-License-Identifier: GPL-3.0-or-later

//! Axis aligned bounding boxes.
//!
//! [`Box2`] is the port of KiCad's `BOX2I`,
//! `libs/kimath/include/math/box2.h:927`, the instantiation of
//! `BOX2<Vec>` at `box2.h:39` over `VECTOR2I`.
//!
//! Three things about `BOX2<Vec>` are deliberately not ported, following
//! `DESIGN.md` section 3 and `doc/reference/kicad/01-geometry.md` section
//! 14.1:
//!
//! - KiCad stores an origin and a size (`box2.h:920`), which lets a box
//!   have a **negative** size until somebody calls `Normalize`
//!   (`box2.h:143`). Half the members then handle the negative case and
//!   half quietly do not: `SquaredDistance` (`box2.h:781`) gives wrong
//!   answers on a box nobody normalised. This stores two corners and
//!   normalises in every constructor, so an unnormalised box is not
//!   representable.
//! - KiCad carries an `m_init` flag (`box2.h:923`) that makes an
//!   untouched box absorbing in `Merge` and reportable through
//!   `IsValid`. Here an absent box is `Option<Box2>`, so the tri-state is
//!   gone; a caller accumulating boxes writes
//!   `accumulator = Some(match accumulator { None => box, Some(a) =>
//!   a.merge(box) })`.
//! - Coordinates are `i64` throughout, where KiCad keeps an `i32` origin
//!   and an `i64` size (`box2.h:44`). A box inflated by a clearance or a
//!   query margin then cannot leave the range, which is the whole reason
//!   the index inflates its queries.
//!
//! Because the coordinates are `i64` and the router's are `i32`, the
//! arithmetic here saturates rather than wrapping: an addition that would
//! leave `i64` clamps. KiCad range checks the same additions through
//! `KiCheckedCast` (`box2.h:61`), which clamps and trips an assertion.
//! Nothing on a board comes within nine orders of magnitude of the limit.
//!
//! Members of `BOX2<Vec>` that neither the router core nor
//! `shape_line_chain.cpp` and `shape_collisions.cpp` call are left out:
//! `ByCenter`, `Compute`, `Move` and `Offset`, `GetWithOffset`, every
//! setter, `Intersects( Vec, Vec )`, `Intersects( BOX2, EDA_ANGLE )`,
//! `IntersectsCircle`, `IntersectsCircleEdge`, `GetBoundingBoxRotated`,
//! `Diagonal` and `SquaredDiagonal`, `NearestPoint`, `FarthestPointTo`,
//! `GetSizeMax`, `Format` and `IsValid`. `Normalize` is not a member here
//! because no constructor can produce a box that needs it.

use crate::geometry::math::kiround_i64;
use crate::geometry::vec2::{Vec2, Vec2L};

/// An axis aligned rectangle, always normalised.
///
/// Port of `BOX2<VECTOR2I>`, `libs/kimath/include/math/box2.h:39` and
/// `:927`. The invariant is `min.x <= max.x && min.y <= max.y`, which
/// every constructor establishes and no member can break, so a box always
/// has a non negative width and height. A box that covers a single point
/// has `min == max`.
///
/// Edges count as inside: [`Box2::contains_point`],
/// [`Box2::contains_box`] and [`Box2::intersects`] are all closed
/// interval tests, as KiCad's are (`box2.h:163`, `:333`).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Box2 {
  /// The corner with the smaller coordinates, KiCad's `m_Pos` once it is
  /// normalised (`box2.h:920`).
  min: Vec2L,
  /// The corner with the larger coordinates, KiCad's `GetEnd()`
  /// (`box2.h:209`).
  max: Vec2L,
}

impl Box2 {
  /// A box covering exactly one point.
  ///
  /// Port of `BOX2( const Vec&, const SizeVec& = (0, 0) )`,
  /// `libs/kimath/include/math/box2.h:55` with its default size.
  pub const fn from_point(point: Vec2L) -> Self {
    Self {
      min: point,
      max: point,
    }
  }

  /// The smallest box containing two corners, in either order.
  ///
  /// Port of `BOX2::ByCorners`,
  /// `libs/kimath/include/math/box2.h:67`, which builds a box from a
  /// corner and a possibly negative size and then normalises it
  /// (`box2.h:64`, `:143`).
  pub const fn from_corners(first: Vec2L, second: Vec2L) -> Self {
    Self {
      min: Vec2L::new(min_i64(first.x, second.x), min_i64(first.y, second.y)),
      max: Vec2L::new(max_i64(first.x, second.x), max_i64(first.y, second.y)),
    }
  }

  /// A box from an origin and a size, where a negative size extends
  /// backwards from the origin.
  ///
  /// Port of `BOX2( const Vec&, const SizeVec& )`,
  /// `libs/kimath/include/math/box2.h:55`, whose constructor ends in
  /// `Normalize()`: a negative component moves the origin by that
  /// component and flips its sign, so the box is the same rectangle
  /// either way.
  pub fn from_origin_and_size(origin: Vec2L, size: Vec2L) -> Self {
    Self::from_corners(
      origin,
      Vec2L::new(
        origin.x.saturating_add(size.x),
        origin.y.saturating_add(size.y),
      ),
    )
  }

  /// A box covering exactly one nanometre coordinate point.
  ///
  /// The `i32` shorthand for [`Box2::from_point`]; the router builds
  /// nearly every box out of [`Vec2`] shape coordinates.
  pub fn from_vec2(point: Vec2) -> Self {
    Self::from_point(Vec2L::from(point))
  }

  /// The smallest box containing two nanometre coordinate corners.
  ///
  /// The `i32` shorthand for [`Box2::from_corners`].
  pub fn from_vec2_corners(first: Vec2, second: Vec2) -> Self {
    Self::from_corners(Vec2L::from(first), Vec2L::from(second))
  }

  /// The box that contains every representable nanometre coordinate.
  ///
  /// Port of `BOX2::SetMaximum`,
  /// `libs/kimath/include/math/box2.h:77`, which sets the origin to
  /// `-INT32_MAX` and the size to `2 * INT32_MAX` with the comment "we
  /// want to be able to invert the box, so don't use lowest()". The
  /// router's only use is the initial visible view area
  /// (`pcbnew/router/pns_router.cpp:77`), which the shove intersects with
  /// its changed area (`pcbnew/router/pns_shove.cpp:2040`).
  ///
  /// The bounds are the `i32` ones and not the `i64` ones on purpose:
  /// everything this box is ever compared against is a [`Vec2`]
  /// coordinate, and keeping KiCad's asymmetric range means the
  /// comparisons come out the same. `i32::MIN` itself is one nanometre
  /// outside, exactly as in KiCad.
  pub fn maximum() -> Self {
    let limit = i64::from(i32::MAX);

    Self {
      min: Vec2L::new(-limit, -limit),
      max: Vec2L::new(limit, limit),
    }
  }

  /// The corner with the smaller coordinates.
  ///
  /// Port of `BOX2::GetOrigin` and `GetPosition`,
  /// `libs/kimath/include/math/box2.h:207` and `:208`.
  pub const fn origin(&self) -> Vec2L {
    self.min
  }

  /// The corner with the larger coordinates.
  ///
  /// Port of `BOX2::GetEnd`,
  /// `libs/kimath/include/math/box2.h:209`.
  pub const fn end(&self) -> Vec2L {
    self.max
  }

  /// The width and the height as a vector.
  ///
  /// Port of `BOX2::GetSize`,
  /// `libs/kimath/include/math/box2.h:203`. Both components are non
  /// negative, since the box is normalised.
  pub fn size(&self) -> Vec2L {
    Vec2L::new(self.width(), self.height())
  }

  /// The width, never negative.
  ///
  /// Port of `BOX2::GetWidth`,
  /// `libs/kimath/include/math/box2.h:211`.
  pub const fn width(&self) -> i64 {
    self.max.x.saturating_sub(self.min.x)
  }

  /// The height, never negative.
  ///
  /// Port of `BOX2::GetHeight`,
  /// `libs/kimath/include/math/box2.h:212`.
  pub const fn height(&self) -> i64 {
    self.max.y.saturating_sub(self.min.y)
  }

  /// The smaller `x` coordinate.
  ///
  /// Port of `BOX2::GetLeft`, and of `GetX`,
  /// `libs/kimath/include/math/box2.h:225` and `:204`, which are the same
  /// value on a normalised box.
  pub const fn left(&self) -> i64 {
    self.min.x
  }

  /// The larger `x` coordinate.
  ///
  /// Port of `BOX2::GetRight`,
  /// `libs/kimath/include/math/box2.h:214`.
  pub const fn right(&self) -> i64 {
    self.max.x
  }

  /// The smaller `y` coordinate, which is the upper edge on screen.
  ///
  /// Port of `BOX2::GetTop`, and of `GetY`,
  /// `libs/kimath/include/math/box2.h:226` and `:205`.
  pub const fn top(&self) -> i64 {
    self.min.y
  }

  /// The larger `y` coordinate, which is the lower edge on screen.
  ///
  /// Port of `BOX2::GetBottom`,
  /// `libs/kimath/include/math/box2.h:219`.
  pub const fn bottom(&self) -> i64 {
    self.max.y
  }

  /// The centre point.
  ///
  /// Port of `BOX2::Centre` and `GetCenter`,
  /// `libs/kimath/include/math/box2.h:94` and `:227`, which is
  /// `origin + size / 2` with an integer division that **truncates
  /// towards zero**. It does not round, so the centre of a box of odd
  /// width sits one nanometre towards the origin.
  pub const fn center(&self) -> Vec2L {
    Vec2L::new(
      self.min.x.saturating_add(self.width() / 2),
      self.min.y.saturating_add(self.height() / 2),
    )
  }

  /// The area.
  ///
  /// Port of `BOX2::GetArea`,
  /// `libs/kimath/include/math/box2.h:756`. The cluster walk in
  /// `pcbnew/router/pns_topology.cpp:1096` compares areas to decide
  /// whether a candidate blows the cluster up.
  ///
  /// The area of [`Box2::maximum`] is about twice `i64::MAX`, so this is
  /// the one accessor where the saturation the module documentation
  /// promises actually fires. KiCad wraps there instead.
  pub const fn area(&self) -> i64 {
    self.width().saturating_mul(self.height())
  }

  /// The smallest box containing this one and another.
  ///
  /// Port of `BOX2::Merge( const BOX2& )`,
  /// `libs/kimath/include/math/box2.h:653`, minus the `m_init` branch:
  /// an absent box is `None` here, so there is nothing to absorb.
  pub fn merge(self, other: Box2) -> Self {
    Self {
      min: Vec2L::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
      max: Vec2L::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
    }
  }

  /// The smallest box containing this one and a point.
  ///
  /// Port of `BOX2::Merge( const Vec& )`,
  /// `libs/kimath/include/math/box2.h:687`.
  pub fn merge_point(self, point: Vec2L) -> Self {
    Self {
      min: Vec2L::new(self.min.x.min(point.x), self.min.y.min(point.y)),
      max: Vec2L::new(self.max.x.max(point.x), self.max.y.max(point.y)),
    }
  }

  /// Whether a point lies inside the box, edges included.
  ///
  /// Port of `BOX2::Contains( const Vec& )`,
  /// `libs/kimath/include/math/box2.h:165`. KiCad's negative size
  /// handling there (`box2.h:170`) is unreachable on a normalised box and
  /// is not ported.
  pub const fn contains_point(&self, point: Vec2L) -> bool {
    point.x >= self.min.x
      && point.x <= self.max.x
      && point.y >= self.min.y
      && point.y <= self.max.y
  }

  /// Whether another box lies inside this one, common edges included.
  ///
  /// Port of `BOX2::Contains( const BOX2& )`,
  /// `libs/kimath/include/math/box2.h:198`.
  pub const fn contains_box(&self, other: &Box2) -> bool {
    self.contains_point(other.min) && self.contains_point(other.max)
  }

  /// Whether two boxes share at least one point.
  ///
  /// Port of `BOX2::Intersects( const BOX2& )`,
  /// `libs/kimath/include/math/box2.h:308`, which is a closed interval
  /// test: two boxes that merely touch along an edge do intersect.
  pub const fn intersects(&self, other: &Box2) -> bool {
    max_i64(self.min.x, other.min.x) <= min_i64(self.max.x, other.max.x)
      && max_i64(self.min.y, other.min.y) <= min_i64(self.max.y, other.max.y)
  }

  /// The overlap of two boxes, if they overlap in both directions.
  ///
  /// Port of `BOX2::Intersect`,
  /// `libs/kimath/include/math/box2.h:344`. Note KiCad's emptiness test
  /// there is a **strict** `<` while [`Box2::intersects`] uses `<=`, so
  /// two boxes that touch along an edge report an intersection but have
  /// no intersection box. That asymmetry is kept.
  ///
  /// Deviation: KiCad returns a zero sized box at the origin when there
  /// is no overlap, which is indistinguishable from a genuine overlap at
  /// the origin. This returns `None`.
  pub fn intersection(&self, other: &Box2) -> Option<Box2> {
    let min =
      Vec2L::new(self.min.x.max(other.min.x), self.min.y.max(other.min.y));
    let max =
      Vec2L::new(self.max.x.min(other.max.x), self.max.y.min(other.max.y));

    if min.x < max.x && min.y < max.y {
      Some(Self { min, max })
    } else {
      None
    }
  }

  /// The box grown by `delta_x` on the left and right and by `delta_y` on
  /// the top and bottom.
  ///
  /// Port of `BOX2::Inflate( coord_type, coord_type )` and
  /// `GetInflated`, `libs/kimath/include/math/box2.h:553` and `:633`.
  ///
  /// A negative delta deflates. KiCad refuses to let a deflation invert
  /// the box: when the requested deflation would eat more than the whole
  /// width, the box collapses onto `origin + size / 2` instead, which is
  /// the same truncating half as [`Box2::center`] (`box2.h:557`). The two
  /// axes decide that independently, so a box can collapse in `x` and
  /// still grow in `y`.
  pub fn inflate(self, delta_x: i64, delta_y: i64) -> Self {
    let (min_x, max_x) = inflate_axis(self.min.x, self.max.x, delta_x);
    let (min_y, max_y) = inflate_axis(self.min.y, self.max.y, delta_y);

    Self {
      min: Vec2L::new(min_x, min_y),
      max: Vec2L::new(max_x, max_y),
    }
  }

  /// The box grown by the same delta in both directions.
  ///
  /// Port of `BOX2::Inflate( coord_type )` and `GetInflated`,
  /// `libs/kimath/include/math/box2.h:624` and `:643`.
  pub fn inflate_by(self, delta: i64) -> Self {
    self.inflate(delta, delta)
  }

  /// The squared distance from a point to the box, zero inside it.
  ///
  /// Deviation from `BOX2::SquaredDistance( const Vec& )`,
  /// `libs/kimath/include/math/box2.h:781`. KiCad's expression there is
  /// `max( aP.x < m_Pos.x ? m_Pos.x - aP.x : m_Pos.x - x2, 0 )`: the
  /// second arm subtracts the box's own right edge from its own left
  /// edge, which on a normalised box is never positive, so every point to
  /// the right of or below the box reports distance zero. This computes
  /// the intended `max(min - p, p - max, 0)` per axis instead.
  ///
  /// The router core never calls this, and reproducing a bug in the
  /// primitive the spatial index will be built on buys nothing. See the
  /// module documentation for the negative size mode the same function
  /// also mishandles.
  pub fn squared_distance_to_point(&self, point: Vec2L) -> i64 {
    let x_gap = axis_gap(self.min.x, self.max.x, point.x);
    let y_gap = axis_gap(self.min.y, self.max.y, point.y);

    x_gap
      .saturating_mul(x_gap)
      .saturating_add(y_gap.saturating_mul(y_gap))
  }

  /// The squared distance between two boxes, zero when they overlap.
  ///
  /// Port of `BOX2::SquaredDistance( const BOX2& )`,
  /// `libs/kimath/include/math/box2.h:808`, which handles both directions
  /// correctly.
  pub fn squared_distance_to_box(&self, other: &Box2) -> i64 {
    let x_gap = box_axis_gap(self.min.x, self.max.x, other.min.x, other.max.x);
    let y_gap = box_axis_gap(self.min.y, self.max.y, other.min.y, other.max.y);

    x_gap
      .saturating_mul(x_gap)
      .saturating_add(y_gap.saturating_mul(y_gap))
  }

  /// The distance from a point to the box, rounded.
  ///
  /// Port of `BOX2::Distance( const Vec& )`,
  /// `libs/kimath/include/math/box2.h:792`, which takes a `f64` square
  /// root and rounds it with `KiROUND`. Note this rounds where
  /// [`crate::geometry::seg::Seg::distance_to_point`] truncates; the two
  /// conventions coexist in KiCad and both are kept.
  pub fn distance_to_point(&self, point: Vec2L) -> i64 {
    kiround_i64((self.squared_distance_to_point(point) as f64).sqrt())
  }
}

/// The smaller of two values, as a `const fn` so the constructors can be
/// `const`.
const fn min_i64(first: i64, second: i64) -> i64 {
  if first < second { first } else { second }
}

/// The larger of two values, as a `const fn` so the constructors can be
/// `const`.
const fn max_i64(first: i64, second: i64) -> i64 {
  if first > second { first } else { second }
}

/// One axis of `BOX2::Inflate`, `libs/kimath/include/math/box2.h:555`,
/// restricted to the non negative size branch that a normalised box
/// always takes.
fn inflate_axis(min: i64, max: i64, delta: i64) -> (i64, i64) {
  let size = max.saturating_sub(min);

  if size < delta.saturating_mul(-2) {
    // The deflation would invert the box, so collapse it onto the centre.
    let centre = min.saturating_add(size / 2);

    (centre, centre)
  } else {
    (min.saturating_sub(delta), max.saturating_add(delta))
  }
}

/// The gap between a coordinate and a closed interval, zero inside it.
fn axis_gap(min: i64, max: i64, value: i64) -> i64 {
  if value < min {
    min.saturating_sub(value)
  } else if value > max {
    value.saturating_sub(max)
  } else {
    0
  }
}

/// The gap between two closed intervals, zero when they overlap.
///
/// Port of the two branches per axis of
/// `BOX2::SquaredDistance( const BOX2& )`,
/// `libs/kimath/include/math/box2.h:812`.
fn box_axis_gap(min: i64, max: i64, other_min: i64, other_max: i64) -> i64 {
  if other_max < min {
    min.saturating_sub(other_max)
  } else if other_min > max {
    other_min.saturating_sub(max)
  } else {
    0
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  /// A shorthand for the KiCad test cases, which are all small integers.
  fn point(x: i64, y: i64) -> Vec2L {
    Vec2L::new(x, y)
  }

  /// A box from an origin and a size, the shape most KiCad test cases are
  /// written in.
  fn sized(x: i64, y: i64, width: i64, height: i64) -> Box2 {
    Box2::from_origin_and_size(point(x, y), point(width, height))
  }

  /// `BasicInt`, `qa/tests/libs/kimath/math/test_box2.cpp:43`, with
  /// KiCad's expected values verbatim.
  #[test]
  fn basic_origin_size_and_inflation() {
    let box2 = sized(1, 2, 3, 4);

    assert_eq!(box2.origin(), point(1, 2));
    assert_eq!(box2.size(), point(3, 4));
    assert_eq!(box2, sized(1, 2, 3, 4));

    let inflated = box2.inflate_by(1);
    assert_eq!(inflated.origin(), point(0, 1));
    assert_eq!(inflated.size(), point(5, 6));
  }

  /// The runtime half of `Constexpr`,
  /// `qa/tests/libs/kimath/math/test_box2.cpp:64`. The `GetWithOffset`
  /// step is rewritten as an explicit shifted box, since the offset
  /// member is not ported.
  #[test]
  fn inflation_and_merge_from_the_kicad_constexpr_case() {
    let base = sized(1, 2, 3, 4);
    let inflated = base.inflate_by(1);

    assert_eq!(inflated.origin(), point(0, 1));
    assert_eq!(inflated.size(), point(5, 6));

    let shifted = sized(101, 2, 3, 4);
    let merged = shifted.merge(base);

    assert_eq!(merged.origin(), point(1, 2));
    assert_eq!(merged.size(), point(103, 4));
  }

  /// `ByCorners`, `qa/tests/libs/kimath/math/test_box2.cpp:95`.
  #[test]
  fn by_corners_matches_origin_and_size() {
    assert_eq!(
      Box2::from_corners(point(1, 2), point(3, 4)),
      sized(1, 2, 2, 2)
    );
    // The corners may come in either order, unlike KiCad where an
    // unnormalised box would survive until somebody called Normalize.
    assert_eq!(
      Box2::from_corners(point(3, 4), point(1, 2)),
      sized(1, 2, 2, 2)
    );
  }

  /// A negative size extends backwards from the origin,
  /// `libs/kimath/include/math/box2.h:143`.
  #[test]
  fn a_negative_size_normalises() {
    assert_eq!(sized(10, 10, -4, -6), sized(6, 4, 4, 6));
    assert_eq!(sized(10, 10, -4, 6), sized(6, 10, 4, 6));
    assert_eq!(sized(0, 0, 0, 0), Box2::from_point(point(0, 0)));
  }

  /// The `i32` shorthands agree with the widened constructors.
  #[test]
  fn vec2_shorthands() {
    assert_eq!(
      Box2::from_vec2(Vec2::new(3, 4)),
      Box2::from_point(point(3, 4))
    );
    assert_eq!(
      Box2::from_vec2_corners(Vec2::new(3, 4), Vec2::new(-1, 9)),
      Box2::from_corners(point(-1, 4), point(3, 9))
    );
  }

  /// The maximum box spans the `i32` range without its most negative
  /// value, `libs/kimath/include/math/box2.h:87`.
  #[test]
  fn the_maximum_box_covers_every_nanometre_coordinate() {
    let maximum = Box2::maximum();

    assert_eq!(maximum.left(), i64::from(-i32::MAX));
    assert_eq!(maximum.right(), i64::from(i32::MAX));
    assert_eq!(maximum.width(), 2 * i64::from(i32::MAX));
    assert_eq!(maximum.height(), 2 * i64::from(i32::MAX));
    assert_eq!(maximum.center(), point(0, 0));

    assert!(maximum.contains_point(Vec2L::from(Vec2::new(i32::MAX, 0))));
    assert!(maximum.contains_point(Vec2L::from(Vec2::new(-i32::MAX, 0))));
    // KiCad's comment says it avoids lowest() so the box can be inverted,
    // which leaves i32::MIN one nanometre outside.
    assert!(!maximum.contains_point(Vec2L::from(Vec2::new(i32::MIN, 0))));
    assert!(maximum.contains_box(&sized(-1000, -1000, 2000, 2000)));
  }

  /// The edge accessors and the truncating centre,
  /// `libs/kimath/include/math/box2.h:94`, `:203` to `:227`.
  #[test]
  fn accessors_and_centre() {
    let box2 = sized(1, 2, 3, 4);

    assert_eq!(box2.left(), 1);
    assert_eq!(box2.top(), 2);
    assert_eq!(box2.right(), 4);
    assert_eq!(box2.bottom(), 6);
    assert_eq!(box2.end(), point(4, 6));
    assert_eq!(box2.width(), 3);
    assert_eq!(box2.height(), 4);
    assert_eq!(box2.area(), 12);

    // The centre truncates: 1 + 3 / 2 is 2, not 2.5 rounded to 3.
    assert_eq!(box2.center(), point(2, 4));
    assert_eq!(sized(-4, 0, 3, 0).center(), point(-3, 0));
  }

  /// Merging with a box and with a point,
  /// `libs/kimath/include/math/box2.h:653` and `:687`.
  #[test]
  fn merging() {
    let box2 = sized(0, 0, 10, 10);

    assert_eq!(box2.merge(sized(20, 20, 5, 5)), sized(0, 0, 25, 25));
    assert_eq!(box2.merge(sized(2, 2, 2, 2)), box2);
    assert_eq!(box2.merge(sized(-5, -5, 2, 2)), sized(-5, -5, 15, 15));

    assert_eq!(box2.merge_point(point(5, 5)), box2);
    assert_eq!(box2.merge_point(point(20, 5)), sized(0, 0, 20, 10));
    assert_eq!(box2.merge_point(point(-4, -6)), sized(-4, -6, 14, 16));

    // An accumulator over an Option is what replaces KiCad's m_init.
    let mut accumulator: Option<Box2> = None;

    for corner in [point(3, 4), point(-2, 8), point(5, -1)] {
      accumulator = Some(match accumulator {
        None => Box2::from_point(corner),
        Some(existing) => existing.merge_point(corner),
      });
    }

    assert_eq!(
      accumulator,
      Some(Box2::from_corners(point(-2, -1), point(5, 8)))
    );
  }

  /// Containment is a closed interval test,
  /// `libs/kimath/include/math/box2.h:163`.
  #[test]
  fn containment_counts_the_edges() {
    let box2 = sized(0, 0, 10, 10);

    assert!(box2.contains_point(point(5, 5)));
    assert!(box2.contains_point(point(0, 0)));
    assert!(box2.contains_point(point(10, 10)));
    assert!(box2.contains_point(point(0, 7)));
    assert!(!box2.contains_point(point(-1, 5)));
    assert!(!box2.contains_point(point(11, 5)));
    assert!(!box2.contains_point(point(5, 11)));

    assert!(box2.contains_box(&sized(2, 2, 3, 3)));
    assert!(box2.contains_box(&box2));
    assert!(box2.contains_box(&Box2::from_point(point(10, 0))));
    assert!(!box2.contains_box(&sized(2, 2, 20, 3)));
    assert!(!box2.contains_box(&sized(-1, 0, 3, 3)));
  }

  /// Intersection is closed interval, the intersection box is not,
  /// `libs/kimath/include/math/box2.h:333` against `:361`.
  #[test]
  fn intersects_and_intersection() {
    let box2 = sized(0, 0, 10, 10);

    assert!(box2.intersects(&sized(5, 5, 10, 10)));
    assert!(box2.intersects(&sized(-5, -5, 20, 20)));
    assert!(box2.intersects(&sized(2, 2, 2, 2)));
    assert!(!box2.intersects(&sized(11, 0, 5, 5)));

    assert_eq!(
      box2.intersection(&sized(5, 5, 10, 10)),
      Some(sized(5, 5, 5, 5))
    );
    assert_eq!(box2.intersection(&sized(-5, -5, 20, 20)), Some(box2));
    assert_eq!(
      box2.intersection(&sized(2, 3, 2, 2)),
      Some(sized(2, 3, 2, 2))
    );
    assert_eq!(box2.intersection(&sized(11, 0, 5, 5)), None);

    // Touching along an edge: they intersect, but the overlap is empty
    // and KiCad's strict test rejects it.
    let touching = sized(10, 0, 5, 5);
    assert!(box2.intersects(&touching));
    assert_eq!(box2.intersection(&touching), None);
  }

  /// Inflating and deflating, including past zero,
  /// `libs/kimath/include/math/box2.h:553`.
  #[test]
  fn inflation_and_deflation() {
    let box2 = sized(0, 0, 10, 20);

    assert_eq!(box2.inflate_by(5), sized(-5, -5, 20, 30));
    assert_eq!(box2.inflate(1, 2), sized(-1, -2, 12, 24));
    assert_eq!(box2.inflate_by(0), box2);
    assert_eq!(box2.inflate_by(-3), sized(3, 3, 4, 14));

    // Deflating by exactly half is still a valid inflate, the box
    // collapses to zero width without taking the centre branch.
    assert_eq!(box2.inflate(-5, 0), sized(5, 0, 0, 20));

    // Past half, the axis collapses onto the truncated centre.
    let collapsed = box2.inflate(-6, 0);
    assert_eq!(collapsed.left(), 5);
    assert_eq!(collapsed.width(), 0);
    assert_eq!(collapsed.height(), 20);

    // The two axes decide independently.
    let mixed = sized(0, 0, 3, 100).inflate(-10, 10);
    assert_eq!(mixed.width(), 0);
    assert_eq!(mixed.left(), 1);
    assert_eq!(mixed.height(), 120);
    assert_eq!(mixed.top(), -10);
  }

  /// The distance to a point, in all nine regions around the box.
  #[test]
  fn squared_distance_to_a_point() {
    let box2 = sized(0, 0, 10, 10);

    assert_eq!(box2.squared_distance_to_point(point(5, 5)), 0);
    assert_eq!(box2.squared_distance_to_point(point(0, 0)), 0);
    assert_eq!(box2.squared_distance_to_point(point(10, 10)), 0);

    // Left, right, above, below.
    assert_eq!(box2.squared_distance_to_point(point(-3, 5)), 9);
    assert_eq!(box2.squared_distance_to_point(point(13, 5)), 9);
    assert_eq!(box2.squared_distance_to_point(point(5, -4)), 16);
    assert_eq!(box2.squared_distance_to_point(point(5, 14)), 16);

    // The four diagonal regions.
    assert_eq!(box2.squared_distance_to_point(point(-3, -4)), 25);
    assert_eq!(box2.squared_distance_to_point(point(13, -4)), 25);
    assert_eq!(box2.squared_distance_to_point(point(-3, 14)), 25);
    assert_eq!(box2.squared_distance_to_point(point(13, 14)), 25);

    assert_eq!(box2.distance_to_point(point(13, 14)), 5);
    assert_eq!(box2.distance_to_point(point(5, 5)), 0);
    // The square root rounds rather than truncating: sqrt(9 + 16 + 2).
    assert_eq!(box2.distance_to_point(point(-3, 15)), 6);
  }

  /// The distance between two boxes,
  /// `libs/kimath/include/math/box2.h:808`.
  #[test]
  fn squared_distance_to_a_box() {
    let box2 = sized(0, 0, 10, 10);

    assert_eq!(box2.squared_distance_to_box(&sized(2, 2, 2, 2)), 0);
    assert_eq!(box2.squared_distance_to_box(&sized(5, 5, 20, 20)), 0);
    assert_eq!(box2.squared_distance_to_box(&sized(10, 10, 5, 5)), 0);

    // Three to the right, four below, and both at once.
    assert_eq!(box2.squared_distance_to_box(&sized(13, 0, 5, 5)), 9);
    assert_eq!(box2.squared_distance_to_box(&sized(0, 14, 5, 5)), 16);
    assert_eq!(box2.squared_distance_to_box(&sized(13, 14, 5, 5)), 25);

    // And the same going the other way.
    assert_eq!(box2.squared_distance_to_box(&sized(-8, -9, 5, 5)), 25);
    assert_eq!(
      box2.squared_distance_to_box(&sized(-8, -9, 5, 5)),
      sized(-8, -9, 5, 5).squared_distance_to_box(&box2)
    );
  }

  /// A box inflated by a clearance stays far from the `i64` limits, which
  /// is why the coordinates are widened in the first place.
  #[test]
  fn a_clearance_inflation_cannot_overflow() {
    let full = Box2::from_vec2_corners(
      Vec2::new(i32::MIN, i32::MIN),
      Vec2::new(i32::MAX, i32::MAX),
    );

    // Eight hundred micrometres is the router's query inflation
    // (`pcbnew/router/pns_node.cpp:62`).
    let inflated = full.inflate_by(800_000);

    assert_eq!(inflated.left(), i64::from(i32::MIN) - 800_000);
    assert_eq!(inflated.right(), i64::from(i32::MAX) + 800_000);
    assert!(inflated.contains_box(&full));
  }

  /// Every member stays total on the widest box the type can carry: the
  /// area alone is twice `i64::MAX` and saturates rather than wrapping.
  #[test]
  fn the_maximum_box_saturates_instead_of_overflowing() {
    let maximum = Box2::maximum();
    let far = Vec2L::new(i64::MAX, i64::MAX);

    assert_eq!(maximum.area(), i64::MAX);
    assert_eq!(maximum.squared_distance_to_point(far), i64::MAX);
    assert_eq!(
      maximum.squared_distance_to_box(&Box2::from_point(far)),
      i64::MAX
    );
    assert_eq!(maximum.distance_to_point(far), 3_037_000_500);
    assert_eq!(maximum.inflate_by(i64::MAX).left(), i64::MIN);
    assert_eq!(maximum.inflate_by(i64::MIN).width(), 0);
  }
}
