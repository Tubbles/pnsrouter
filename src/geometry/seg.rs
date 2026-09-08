// SPDX-License-Identifier: GPL-3.0-or-later

//! Line segments.
//!
//! [`Seg`] is the port of KiCad's `SEG`,
//! `libs/kimath/include/geometry/seg.h:37`. The endpoints are public there
//! (`seg.h:43`) and stay public here, and so does the parent shape index
//! that `SHAPE_LINE_CHAIN::Segment` writes
//! (`libs/kimath/src/geometry/shape_line_chain.cpp:1285`).
//!
//! Every predicate carries a tolerance that the router depends on to the
//! nanometre, so they are transplanted verbatim: [`Seg::contains_point`]
//! keeps the squared tolerance of 3 (`seg.cpp:623`), [`Seg::collinear`]
//! keeps the raw determinant test `<= 1` (`seg.h:290`), and
//! [`Seg::APPROX_DISTANCE_THRESHOLD`] keeps the default 1 of
//! `ApproxCollinear` and `ApproxParallel` (`seg.h:293`). See
//! `doc/reference/kicad/01-geometry.md` sections 3 and 13.
//!
//! Deliberate differences from `seg.h` and `seg.cpp`:
//!
//! - Where KiCad adds or subtracts two `VECTOR2I` in 32 bits and lets the
//!   result wrap (`vector2d.h:437`, `:466`), this widens first. Note
//!   `01-geometry.md` section 14.1 asks for exactly that. The two places
//!   that keep the 32 bit arithmetic are [`Seg::length`] and
//!   [`Seg::perpendicular_seg`], which feed the result straight back into
//!   a `Vec2` and so gain nothing from widening.
//! - Products of two widened differences are computed in `i64` exactly as
//!   KiCad computes them, which is exact for coordinates up to about
//!   plus or minus 1.5e9 nanometres and panics in a debug build beyond
//!   that, where KiCad wraps silently. The one exception is
//!   [`Seg::squared_distance_to_point`], which the collision code leans on
//!   and which therefore widens to `i128`.
//! - KiCad's out parameters become return values:
//!   `SEG::NearestPoints` returns [`NearestPoints`] and `SEG::Collide`
//!   returns [`SegCollision`].
//! - `SEG::Angle` returns an `EDA_ANGLE` (`seg.cpp:107`); this returns
//!   plain degrees as `f64`, because `EDA_ANGLE` is not ported. See
//!   `01-geometry.md` section 14.1.
//!
//! Members of `SEG` that neither the router core nor
//! `shape_line_chain.cpp` and `shape_collisions.cpp` call are left out:
//! `IntersectsLine`, `ParallelSeg`, `TCoef`, `CanonicalCoefs` as a public
//! member (it stays private, [`Seg::collinear`] needs it) and `Square`.

use crate::geometry::math::{isqrt, kiround, kiround_i64, rescale, sign};
use crate::geometry::vec2::{Vec2, Vec2L};

/// Clamp an `i128` into the `i64` range.
///
/// The widened intermediates of [`Seg::squared_distance_to_point`] have to
/// come back as the `SEG::ecoord` that KiCad returns
/// (`libs/kimath/include/geometry/seg.h:40`), and clamping is the same
/// choice `rescale` already makes in [`crate::geometry::math`].
fn saturate_i64(value: i128) -> i64 {
  value.clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i64
}

/// Clamp an `i64` into the `i32` range.
///
/// KiCad narrows the `isqrt` results of `SEG::Distance` (`seg.cpp:698`),
/// `SEG::LineDistance` (`seg.cpp:758`) and `SEG::Collide` (`seg.cpp:611`)
/// with a plain `int` cast, which is implementation defined once the root
/// passes `INT32_MAX`. A root that large needs a squared distance above
/// 4.6e18, which only the extreme corners of the coordinate envelope
/// reach.
fn saturate_i32(value: i64) -> i32 {
  value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// The truncating integer square root of a squared distance, narrowed the
/// way KiCad narrows it.
///
/// Port of the `int( isqrt( ... ) )` idiom at
/// `libs/kimath/src/geometry/seg.cpp:700`, `:706` and `:611`.
fn distance_from_squared(squared: i64) -> i32 {
  debug_assert!(squared >= 0, "a squared distance cannot be negative");

  saturate_i32(isqrt(squared.max(0) as u64) as i64)
}

/// The index of the first smallest entry.
///
/// Port of the four way minimum loops at
/// `libs/kimath/src/geometry/seg.cpp:148` and `:202`, which keep the first
/// minimum because they compare with a strict `<`.
fn index_of_minimum(values: &[i64; 4]) -> usize {
  let mut best = 0;

  for (index, value) in values.iter().enumerate() {
    if *value < values[best] {
      best = index;
    }
  }

  best
}

/// The direction of a vector in degrees, with KiCad's exact axis and
/// diagonal cases.
///
/// Port of `EDA_ANGLE( const VECTOR2D& )`,
/// `libs/kimath/include/geometry/eda_angle.h:72`. The special cases exist
/// so that axis aligned and 45 degree vectors land on exact degree values
/// instead of whatever `atan2` rounds to.
fn direction_degrees(vector: Vec2L) -> f64 {
  let x = vector.x as f64;
  let y = vector.y as f64;

  if x == 0.0 && y == 0.0 {
    return 0.0;
  }

  if y == 0.0 {
    return if x >= 0.0 { 0.0 } else { -180.0 };
  }

  if x == 0.0 {
    return if y >= 0.0 { 90.0 } else { -90.0 };
  }

  if x == y {
    return if x >= 0.0 { 45.0 } else { -180.0 + 45.0 };
  }

  if x == -y {
    return if x >= 0.0 { -45.0 } else { 180.0 - 45.0 };
  }

  // KiCad divides the radians by DEGREES_TO_RADIANS rather than
  // multiplying by its reciprocal (`eda_angle.h:51`, `:122`), so do the
  // same division and not `f64::to_degrees`.
  y.atan2(x) / (std::f64::consts::PI / 180.0)
}

/// Fold an angle in degrees into the half open interval `(-180, 180]`.
///
/// Port of `EDA_ANGLE::Normalize180`,
/// `libs/kimath/include/geometry/eda_angle.h:268`.
fn normalize_180(degrees: f64) -> f64 {
  let mut value = degrees;

  while value <= -180.0 {
    value += 360.0;
  }

  while value > 180.0 {
    value -= 360.0;
  }

  value
}

/// The two closest points of a pair of segments and their distance.
///
/// Replaces the three out parameters of `SEG::NearestPoints`,
/// `libs/kimath/include/geometry/seg.h:187`. KiCad's `bool` return is
/// always `true` (`libs/kimath/src/geometry/seg.cpp:212`), so it is
/// dropped.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct NearestPoints {
  /// The point on the segment the method was called on.
  pub on_self: Vec2,
  /// The point on the other segment.
  pub on_other: Vec2,
  /// The squared distance between the two points.
  pub squared_distance: i64,
}

/// The result of a segment against segment clearance test.
///
/// Replaces the `bool` return and the `int* aActual` out parameter of
/// `SEG::Collide`, `libs/kimath/include/geometry/seg.h:247`. KiCad writes
/// `*aActual` even when it returns `false`
/// (`libs/kimath/src/geometry/seg.cpp:616`), unlike the shape level
/// `Collide` family, so [`SegCollision::actual`] is always meaningful.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct SegCollision {
  /// Whether the two segments are closer to each other than the clearance.
  pub collides: bool,
  /// The distance between the two segments, truncated through `isqrt`.
  ///
  /// Zero when the segments touch or cross, and zero when the clearance
  /// was negative and the test was refused.
  pub actual: i32,
}

/// The outcome of the shared intersection primitive.
///
/// KiCad's private `SEG::intersects` (`seg.cpp:308`) takes an optional out
/// pointer and behaves differently depending on whether it is null: only
/// the point computing path range checks the result against the `int32`
/// coordinate range and answers "no intersection" when it does not fit
/// (`seg.cpp:419`). Keeping the two answers apart in the type makes that
/// asymmetry visible instead of accidental.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum Intersection {
  /// The segments do not intersect.
  Missing,
  /// They intersect and the caller did not ask where.
  Present,
  /// They intersect at this point.
  At(Vec2),
}

/// A straight segment between two points in nanometres.
///
/// Port of `SEG`, `libs/kimath/include/geometry/seg.h:37`.
#[derive(Copy, Clone, Debug)]
pub struct Seg {
  /// The start point.
  pub a: Vec2,
  /// The end point.
  pub b: Vec2,
  /// The position of this segment in the shape it was taken from, or
  /// [`Seg::NO_INDEX`] for a segment that stands on its own.
  ///
  /// Port of `SEG::m_index`, `libs/kimath/include/geometry/seg.h:398`.
  /// `SHAPE_LINE_CHAIN::Segment` fills it in
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:1285`) and the
  /// optimizer reads it back when it splices a bypass into a path
  /// (`pcbnew/router/pns_optimizer.cpp:562`, `:795`).
  pub index: i32,
}

impl Seg {
  /// The index of a segment that does not belong to a parent shape.
  ///
  /// Port of the `m_index = -1` that every `SEG` constructor except the
  /// three argument one sets, `libs/kimath/include/geometry/seg.h:53`,
  /// `:63` and `:73`.
  pub const NO_INDEX: i32 = -1;

  /// The default distance threshold of [`Seg::approx_collinear`] and
  /// [`Seg::approx_parallel`], one nanometre.
  ///
  /// Port of the default argument at
  /// `libs/kimath/include/geometry/seg.h:293`. Both methods take the
  /// threshold explicitly here, because Rust has no default arguments.
  pub const APPROX_DISTANCE_THRESHOLD: i32 = 1;

  /// A segment between two points, with no parent shape.
  ///
  /// Port of `SEG( const VECTOR2I&, const VECTOR2I& )`,
  /// `libs/kimath/include/geometry/seg.h:69`.
  pub const fn new(a: Vec2, b: Vec2) -> Self {
    Self {
      a,
      b,
      index: Self::NO_INDEX,
    }
  }

  /// A segment between two coordinate pairs, with no parent shape.
  ///
  /// Port of `SEG( int, int, int, int )`,
  /// `libs/kimath/include/geometry/seg.h:59`.
  pub const fn from_coords(x1: i32, y1: i32, x2: i32, y2: i32) -> Self {
    Self::new(Vec2::new(x1, y1), Vec2::new(x2, y2))
  }

  /// A segment between two points, referenced to a parent shape.
  ///
  /// Port of `SEG( const VECTOR2I&, const VECTOR2I&, int )`,
  /// `libs/kimath/include/geometry/seg.h:83`.
  pub const fn with_index(a: Vec2, b: Vec2, index: i32) -> Self {
    Self { a, b, index }
  }

  /// The length, rounded to the nearest nanometre.
  ///
  /// Port of `SEG::Length`,
  /// `libs/kimath/include/geometry/seg.h:339`, which is the
  /// `EuclideanNorm` of the 32 bit difference `A - B`, exact diagonal case
  /// included. The subtraction is part of the ported expression rather
  /// than a widened difference, so it panics in a debug build for a
  /// segment that spans more than the `i32` range, where KiCad wraps.
  pub fn length(&self) -> i32 {
    (self.a - self.b).euclidean_norm()
  }

  /// The squared length.
  ///
  /// Port of `SEG::SquaredLength`,
  /// `libs/kimath/include/geometry/seg.h:344`. The difference is widened
  /// first, so the result is exact whenever it fits in an `i64`.
  pub fn squared_length(&self) -> i64 {
    self.a.widening_sub(self.b).squared_euclidean_norm()
  }

  /// The segment with its endpoints swapped.
  ///
  /// Port of `SEG::Reversed`,
  /// `libs/kimath/include/geometry/seg.h:369`, which drops the parent
  /// shape index because it builds a fresh two argument `SEG`.
  pub fn reversed(&self) -> Self {
    Self::new(self.b, self.a)
  }

  /// Swap the endpoints in place.
  ///
  /// Port of `SEG::Reverse`,
  /// `libs/kimath/include/geometry/seg.h:364`, which swaps `A` and `B`
  /// and keeps the parent shape index.
  pub fn reverse(&mut self) {
    std::mem::swap(&mut self.a, &mut self.b);
  }

  /// The midpoint.
  ///
  /// Port of `SEG::Center`,
  /// `libs/kimath/include/geometry/seg.h:375`, which is
  /// `A + ( B - A ) / 2`. KiCad's `operator/` takes a `double` and rounds
  /// each component half away from zero (`vector2d.h:524`), so the
  /// midpoint of an odd length segment lands away from `A`, not towards
  /// it. The difference is widened here where KiCad's wraps.
  ///
  /// Neither the router core nor the shape code calls this; it is here
  /// because `SEG` exposes it and a midpoint is easy to get wrong.
  pub fn center(&self) -> Vec2 {
    let difference = self.b.widening_sub(self.a);
    let half = Vec2::new(
      kiround(difference.x as f64 / 2.0),
      kiround(difference.y as f64 / 2.0),
    );

    self.a + half
  }

  /// Which side of the directed line through the endpoints a point lies
  /// on: `-1` left, `0` on the line, `1` right.
  ///
  /// Port of `SEG::Side`,
  /// `libs/kimath/include/geometry/seg.h:139`. Exact in `i64`, since it
  /// is the raw determinant reduced to its sign.
  pub fn side(&self, point: Vec2) -> i32 {
    let determinant = self
      .b
      .widening_sub(self.a)
      .cross(point.widening_sub(self.a));

    sign(determinant)
  }

  /// The squared distance to a point.
  ///
  /// Port of `SEG::SquaredDistance( const VECTOR2I& )`,
  /// `libs/kimath/src/geometry/seg.cpp:710`.
  ///
  /// KiCad computes `ap.Dot( ab )` in `i64` from two differences that are
  /// already `i64` wide. Each product reaches `(2^32)^2` for endpoints at
  /// opposite ends of the coordinate envelope and their sum overflows, so
  /// this widens the projection, the two squared norms and the length to
  /// `i128` and clamps the answer back into the `i64` that `SEG::ecoord`
  /// is. For every input where KiCad does not overflow the two agree
  /// exactly, the interior `f64` correction included.
  ///
  /// The interior case computes `|ap|^2 - e * e / f` in `f64`, rounds it
  /// back with `KiROUND` and reports 0 when the result comes out negative,
  /// which can only be a rounding artefact (`seg.cpp:731`). The upper
  /// guard, which also returns 0 for a value past `i64::MAX`, is KiCad's
  /// and is reproduced as it stands.
  pub fn squared_distance_to_point(&self, point: Vec2) -> i64 {
    let ab_x = i128::from(self.b.x) - i128::from(self.a.x);
    let ab_y = i128::from(self.b.y) - i128::from(self.a.y);
    let ap_x = i128::from(point.x) - i128::from(self.a.x);
    let ap_y = i128::from(point.y) - i128::from(self.a.y);

    let projection = ap_x * ab_x + ap_y * ab_y;
    let ap_squared = ap_x * ap_x + ap_y * ap_y;

    if projection <= 0 {
      return saturate_i64(ap_squared);
    }

    let ab_squared = ab_x * ab_x + ab_y * ab_y;

    if projection >= ab_squared {
      let bp_x = i128::from(point.x) - i128::from(self.b.x);
      let bp_y = i128::from(point.y) - i128::from(self.b.y);

      return saturate_i64(bp_x * bp_x + bp_y * bp_y);
    }

    let corrected = ap_squared as f64
      - (projection as f64) * (projection as f64) / (ab_squared as f64);

    if corrected < 0.0 || corrected > i64::MAX as f64 {
      return 0;
    }

    kiround_i64(corrected)
  }

  /// The squared distance to another segment.
  ///
  /// Port of `SEG::SquaredDistance( const SEG& )`,
  /// `libs/kimath/src/geometry/seg.cpp:76`. Both degenerate inputs are
  /// handled before the intersection test, because the cross product with
  /// a zero vector is zero and would report a false hit (KiCad's comment
  /// at `seg.cpp:78`).
  pub fn squared_distance_to_segment(&self, other: &Seg) -> i64 {
    if self.a == self.b {
      return other.squared_distance_to_point(self.a);
    }

    if other.a == other.b {
      return self.squared_distance_to_point(other.a);
    }

    if self.intersects(other) {
      return 0;
    }

    let candidates = [
      other.nearest_point_to_point(self.a).widening_sub(self.a),
      other.nearest_point_to_point(self.b).widening_sub(self.b),
      self.nearest_point_to_point(other.a).widening_sub(other.a),
      self.nearest_point_to_point(other.b).widening_sub(other.b),
    ];

    let mut minimum = i64::MAX;

    for candidate in candidates {
      minimum = minimum.min(candidate.squared_euclidean_norm());
    }

    minimum
  }

  /// The distance to a point, truncated.
  ///
  /// Port of `SEG::Distance( const VECTOR2I& )`,
  /// `libs/kimath/src/geometry/seg.cpp:704`. The square root truncates,
  /// which is a property of this caller and not of the root: the shape
  /// level collision code rounds instead. See `01-geometry.md` section
  /// 2.2.
  pub fn distance_to_point(&self, point: Vec2) -> i32 {
    distance_from_squared(self.squared_distance_to_point(point))
  }

  /// The distance to another segment, truncated.
  ///
  /// Port of `SEG::Distance( const SEG& )`,
  /// `libs/kimath/src/geometry/seg.cpp:698`.
  pub fn distance_to_segment(&self, other: &Seg) -> i32 {
    distance_from_squared(self.squared_distance_to_segment(other))
  }

  /// The point of this segment that is closest to a given point.
  ///
  /// Port of `SEG::NearestPoint( const VECTOR2I& )`,
  /// `libs/kimath/src/geometry/seg.cpp:629`. A degenerate segment answers
  /// with its start point. The interior case is a `rescale` of the
  /// projection, and the sum is built in `i64` and narrowed through
  /// [`Vec2L::saturating_to_vec2`], which is what KiCad's cross type
  /// constructor does (`vector2d.h:85`).
  pub fn nearest_point_to_point(&self, point: Vec2) -> Vec2 {
    let direction = self.b.widening_sub(self.a);
    let length_squared = direction.squared_euclidean_norm();

    if length_squared == 0 {
      return self.a;
    }

    let projection = direction.dot(point.widening_sub(self.a));

    if projection < 0 {
      return self.a;
    }

    if projection > length_squared {
      return self.b;
    }

    Vec2L::new(
      i64::from(self.a.x) + rescale(projection, direction.x, length_squared),
      i64::from(self.a.y) + rescale(projection, direction.y, length_squared),
    )
    .saturating_to_vec2()
  }

  /// The point of this segment that is closest to any point of another
  /// segment.
  ///
  /// Port of `SEG::NearestPoint( const SEG& )`,
  /// `libs/kimath/src/geometry/seg.cpp:116`. Intersecting segments answer
  /// with the intersection point; otherwise the best of the four endpoint
  /// projections wins, ties going to the earlier candidate.
  pub fn nearest_point_to_segment(&self, other: &Seg) -> Vec2 {
    if let Some(point) = self.intersect(other, false, false) {
      return point;
    }

    let projected = [
      other.nearest_point_to_point(self.a),
      other.nearest_point_to_point(self.b),
      self.nearest_point_to_point(other.a),
      self.nearest_point_to_point(other.b),
    ];

    let candidates = [self.a, self.b, projected[2], projected[3]];
    let distances = [
      projected[0].widening_sub(self.a).squared_euclidean_norm(),
      projected[1].widening_sub(self.b).squared_euclidean_norm(),
      projected[2].widening_sub(other.a).squared_euclidean_norm(),
      projected[3].widening_sub(other.b).squared_euclidean_norm(),
    ];

    candidates[index_of_minimum(&distances)]
  }

  /// The closest pair of points between this segment and another.
  ///
  /// Port of `SEG::NearestPoints`,
  /// `libs/kimath/src/geometry/seg.cpp:158`. The shape level collision
  /// code builds its minimum translation vectors out of this pair
  /// (`libs/kimath/src/geometry/shape_collisions.cpp:613`, `:740`,
  /// `:872`), so the tie breaking order matters: the four candidates are
  /// tried in the order start of self, end of self, start of other, end of
  /// other, and the first minimum wins.
  pub fn nearest_points(&self, other: &Seg) -> NearestPoints {
    if let Some(point) = self.intersect(other, false, false) {
      return NearestPoints {
        on_self: point,
        on_other: point,
        squared_distance: 0,
      };
    }

    let projected = [
      other.nearest_point_to_point(self.a),
      other.nearest_point_to_point(self.b),
      self.nearest_point_to_point(other.a),
      self.nearest_point_to_point(other.b),
    ];

    let on_self = [self.a, self.b, projected[2], projected[3]];
    let on_other = [projected[0], projected[1], other.a, other.b];
    let distances = [
      projected[0].widening_sub(self.a).squared_euclidean_norm(),
      projected[1].widening_sub(self.b).squared_euclidean_norm(),
      projected[2].widening_sub(other.a).squared_euclidean_norm(),
      projected[3].widening_sub(other.b).squared_euclidean_norm(),
    ];

    let best = index_of_minimum(&distances);

    NearestPoints {
      on_self: on_self[best],
      on_other: on_other[best],
      squared_distance: distances[best],
    }
  }

  /// The perpendicular projection of a point onto the infinite line
  /// through the endpoints.
  ///
  /// Port of `SEG::LineProject`,
  /// `libs/kimath/src/geometry/seg.cpp:681`. A degenerate segment answers
  /// with its start point. Unlike [`Seg::nearest_point_to_point`] the
  /// result is not clamped to the segment, so the caller gets a point on
  /// the extension of the line as well.
  pub fn line_project(&self, point: Vec2) -> Vec2 {
    let direction = self.b.widening_sub(self.a);
    let length_squared = direction.dot(direction);

    if length_squared == 0 {
      return self.a;
    }

    let projection = direction.dot(point.widening_sub(self.a));

    Vec2L::new(
      i64::from(self.a.x) + rescale(projection, direction.x, length_squared),
      i64::from(self.a.y) + rescale(projection, direction.y, length_squared),
    )
    .saturating_to_vec2()
  }

  /// The point mirrored across the infinite line through the endpoints.
  ///
  /// Port of `SEG::ReflectPoint`,
  /// `libs/kimath/src/geometry/seg.cpp:660`. A degenerate segment mirrors
  /// a point onto itself. `SHAPE_LINE_CHAIN::Mirror` is the only caller
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:992`).
  pub fn reflect_point(&self, point: Vec2) -> Vec2 {
    let direction = self.b.widening_sub(self.a);
    let length_squared = direction.dot(direction);

    let centre = if length_squared == 0 {
      Vec2L::from(point)
    } else {
      let projection = direction.dot(point.widening_sub(self.a));

      Vec2L::new(
        i64::from(self.a.x) + rescale(projection, direction.x, length_squared),
        i64::from(self.a.y) + rescale(projection, direction.y, length_squared),
      )
    };

    Vec2L::new(
      2 * centre.x - i64::from(point.x),
      2 * centre.y - i64::from(point.y),
    )
    .saturating_to_vec2()
  }

  /// The unsigned distance from a point to the infinite line through the
  /// endpoints, truncated.
  ///
  /// Port of `SEG::LineDistance` with `aDetermineSide == false`,
  /// `libs/kimath/src/geometry/seg.cpp:742`.
  pub fn line_distance(&self, point: Vec2) -> i32 {
    let (_, distance) = self.line_distance_parts(point);

    saturate_i32(distance)
  }

  /// The distance from a point to the infinite line through the
  /// endpoints, negative on the left of the directed line.
  ///
  /// Port of `SEG::LineDistance` with `aDetermineSide == true`,
  /// `libs/kimath/src/geometry/seg.cpp:742`. The multi dragger uses the
  /// sign to keep the dragged lines on the side they started on
  /// (`pcbnew/router/pns_multi_dragger.cpp:849`).
  pub fn line_distance_signed(&self, point: Vec2) -> i32 {
    let (determinant, distance) = self.line_distance_parts(point);

    saturate_i32(i64::from(sign(determinant)) * distance)
  }

  /// The determinant and the truncated distance shared by the two
  /// `LineDistance` flavours, `libs/kimath/src/geometry/seg.cpp:742`.
  fn line_distance_parts(&self, point: Vec2) -> (i64, i64) {
    let p = i64::from(self.a.y) - i64::from(self.b.y);
    let q = i64::from(self.b.x) - i64::from(self.a.x);
    let r = -p * i64::from(self.a.x) - q * i64::from(self.a.y);
    let length_squared = p * p + q * q;
    let determinant = p * i64::from(point.x) + q * i64::from(point.y) + r;

    let distance_squared = if length_squared > 0 {
      rescale(determinant, determinant, length_squared)
    } else {
      0
    };

    (determinant, isqrt(distance_squared.max(0) as u64) as i64)
  }

  /// The smallest angle between two segments, in degrees.
  ///
  /// Port of `SEG::Angle`,
  /// `libs/kimath/src/geometry/seg.cpp:107`. KiCad returns an `EDA_ANGLE`,
  /// which is a `double` in degrees (`eda_angle.h:116`); since `EDA_ANGLE`
  /// itself is not ported the degrees come back bare. The optimizer's area
  /// constraint is the only caller in the router core and only asks
  /// whether the result is horizontal (`pns_optimizer.cpp:232`, `:236`).
  ///
  /// Both directions run through the axis and diagonal special cases of
  /// `EDA_ANGLE( const VECTOR2D& )`, so exact horizontals, verticals and
  /// 45 degree segments give exact degree values.
  ///
  /// The two segments are directed, from `b` towards `a` as KiCad takes
  /// them, so reversing one of them turns the result by 180 degrees. That
  /// is why `EDA_ANGLE::IsHorizontal` accepts both 0 and 180
  /// (`eda_angle.h:142`).
  pub fn angle_degrees(&self, other: &Seg) -> f64 {
    let this_angle =
      normalize_180(direction_degrees(self.a.widening_sub(self.b)));
    let other_angle =
      normalize_180(direction_degrees(other.a.widening_sub(other.b)));

    normalize_180(this_angle - other_angle).abs()
  }

  /// A segment perpendicular to this one, starting at a given point.
  ///
  /// Port of `SEG::PerpendicularSeg`,
  /// `libs/kimath/src/geometry/seg.cpp:520`. Only
  /// [`Seg::approx_perpendicular`] needs it; the slope and the endpoint
  /// stay in 32 bits as they do in KiCad, so a slope that spans more than
  /// the coordinate range panics in a debug build.
  pub fn perpendicular_seg(&self, point: Vec2) -> Self {
    let slope = self.b - self.a;

    Self::new(point, slope.perpendicular() + point)
  }

  /// Whether another segment lies on the same infinite line.
  ///
  /// Port of `SEG::Collinear`,
  /// `libs/kimath/include/geometry/seg.h:282`. The test is `<= 1` on the
  /// raw, un-normalised determinant, so the distance it tolerates shrinks
  /// as the segment gets longer. That is deliberate and the router depends
  /// on it, see `01-geometry.md` section 3.1.
  pub fn collinear(&self, other: &Seg) -> bool {
    let (qa, qb, qc) = self.canonical_coefs();

    let d1 = (i64::from(other.a.x) * qa + i64::from(other.a.y) * qb + qc).abs();
    let d2 = (i64::from(other.b.x) * qa + i64::from(other.b.y) * qb + qc).abs();

    d1 <= 1 && d2 <= 1
  }

  /// The coefficients of the line equation `qa * x + qb * y + qc == 0`.
  ///
  /// Port of `SEG::CanonicalCoefs`,
  /// `libs/kimath/include/geometry/seg.h:269`. Private, because nothing
  /// outside [`Seg::collinear`] calls it.
  fn canonical_coefs(&self) -> (i64, i64, i64) {
    let qa = i64::from(self.a.y) - i64::from(self.b.y);
    let qb = i64::from(self.b.x) - i64::from(self.a.x);
    let qc = -qa * i64::from(self.a.x) - qb * i64::from(self.a.y);

    (qa, qb, qc)
  }

  /// The signed squared distances of the shorter segment's endpoints to
  /// the longer segment's line, or `None` when the longer segment is
  /// degenerate.
  ///
  /// Port of `SEG::mutualDistanceSquared`,
  /// `libs/kimath/src/geometry/seg.cpp:762`.
  fn mutual_distance_squared(&self, other: &Seg) -> Option<(i64, i64)> {
    let (long, short) = if self.squared_length() < other.squared_length() {
      (other, self)
    } else {
      (self, other)
    };

    let p = i64::from(long.a.y) - i64::from(long.b.y);
    let q = i64::from(long.b.x) - i64::from(long.a.x);
    let r = -p * i64::from(long.a.x) - q * i64::from(long.a.y);
    let length_squared = p * p + q * q;

    if length_squared == 0 {
      return None;
    }

    let det1 = p * i64::from(short.a.x) + q * i64::from(short.a.y) + r;
    let det2 = p * i64::from(short.b.x) + q * i64::from(short.b.y) + r;

    Some((
      i64::from(sign(det1)) * rescale(det1, det1, length_squared),
      i64::from(sign(det2)) * rescale(det2, det2, length_squared),
    ))
  }

  /// Whether another segment lies on the same line to within a distance
  /// threshold.
  ///
  /// Port of `SEG::ApproxCollinear`,
  /// `libs/kimath/src/geometry/seg.cpp:791`. Pass
  /// [`Seg::APPROX_DISTANCE_THRESHOLD`] for KiCad's default. Unlike
  /// [`Seg::collinear`] the tolerance is a real distance, because the
  /// determinants are normalised by the segment length.
  pub fn approx_collinear(&self, other: &Seg, distance_threshold: i32) -> bool {
    let threshold_squared =
      i64::from(distance_threshold) * i64::from(distance_threshold);

    match self.mutual_distance_squared(other) {
      None => false,
      Some((d1, d2)) => {
        d1.abs() <= threshold_squared && d2.abs() <= threshold_squared
      }
    }
  }

  /// Whether another segment runs parallel to this one to within a
  /// distance threshold.
  ///
  /// Port of `SEG::ApproxParallel`,
  /// `libs/kimath/src/geometry/seg.cpp:803`. Pass
  /// [`Seg::APPROX_DISTANCE_THRESHOLD`] for KiCad's default. Note this
  /// compares two **signed** squared distances, so it is a test on how
  /// much the far segment tilts over its own length and not a rotation
  /// invariant angular tolerance.
  pub fn approx_parallel(&self, other: &Seg, distance_threshold: i32) -> bool {
    let threshold_squared =
      i64::from(distance_threshold) * i64::from(distance_threshold);

    match self.mutual_distance_squared(other) {
      None => false,
      Some((d1, d2)) => (d1 - d2).abs() <= threshold_squared,
    }
  }

  /// Whether another segment is perpendicular to this one, to within the
  /// default distance threshold.
  ///
  /// Port of `SEG::ApproxPerpendicular`,
  /// `libs/kimath/src/geometry/seg.cpp:815`, which rotates this segment by
  /// 90 degrees about its own start and asks whether the other one is
  /// parallel to that. There is no threshold argument in KiCad either, the
  /// inner call takes the default.
  pub fn approx_perpendicular(&self, other: &Seg) -> bool {
    let perpendicular = self.perpendicular_seg(self.a);

    other.approx_parallel(&perpendicular, Self::APPROX_DISTANCE_THRESHOLD)
  }

  /// Whether a point lies on the segment, to within a squared distance of
  /// 3.
  ///
  /// Port of `SEG::Contains( const VECTOR2I& )`,
  /// `libs/kimath/src/geometry/seg.cpp:623`. The tolerance is hard coded
  /// in KiCad and is roughly 1.7 nanometres, enough to cover the rounding
  /// of a 45 degree corner.
  pub fn contains_point(&self, point: Vec2) -> bool {
    self.squared_distance_to_point(point) <= 3
  }

  /// Whether another segment lies entirely within this one.
  ///
  /// Port of `SEG::Contains( const SEG& )`,
  /// `libs/kimath/include/geometry/seg.h:320`.
  pub fn contains_segment(&self, other: &Seg) -> bool {
    if other.a == other.b {
      return self.contains_point(other.a);
    }

    if !self.collinear(other) {
      return false;
    }

    self.contains_point(other.a) && self.contains_point(other.b)
  }

  /// Whether two collinear segments share more than a bare endpoint.
  ///
  /// Port of `SEG::Overlaps`,
  /// `libs/kimath/include/geometry/seg.h:297`. A degenerate other segment
  /// that sits on one of this segment's endpoints reports `false`, which
  /// is what makes the line placer split a target trace only where it
  /// really runs alongside it (`pcbnew/router/pns_line_placer.cpp:1521`).
  pub fn overlaps(&self, other: &Seg) -> bool {
    if other.a == other.b {
      if self.a == other.a || self.b == other.a {
        return false;
      }

      return self.contains_point(other.a);
    }

    if !self.collinear(other) {
      return false;
    }

    self.contains_point(other.a)
      || self.contains_point(other.b)
      || other.contains_point(self.a)
      || other.contains_point(self.b)
  }

  /// Whether two segments touch or cross.
  ///
  /// Port of `SEG::Intersects`,
  /// `libs/kimath/src/geometry/seg.cpp:436`, which is the private
  /// primitive with no output point. It therefore skips the range check
  /// that [`Seg::intersect`] applies, so it can report an intersection
  /// whose coordinates would not fit in a [`Vec2`].
  pub fn intersects(&self, other: &Seg) -> bool {
    self.intersects_impl(other, false, false, false) != Intersection::Missing
  }

  /// The intersection point of two segments, if there is one.
  ///
  /// Port of `SEG::Intersect`,
  /// `libs/kimath/src/geometry/seg.cpp:442`. With `ignore_endpoints` a
  /// contact that is only an endpoint of both segments does not count.
  /// With `lines` both segments stand for the infinite lines through their
  /// endpoints, which also skips the bounding box rejection.
  ///
  /// Two collinear overlapping segments answer with the midpoint of the
  /// overlap interval, truncated towards zero (`seg.cpp:281`), and two
  /// collinear infinite lines answer with the rounded midpoint between the
  /// two start points (`seg.cpp:366`).
  ///
  /// Returns `None` when the intersection exists but its coordinates
  /// leave the `i32` range (`seg.cpp:423`). That silent failure is
  /// KiCad's and is kept, because the walkaround compares the answer
  /// against hull vertices that are `i32` themselves.
  pub fn intersect(
    &self,
    other: &Seg,
    ignore_endpoints: bool,
    lines: bool,
  ) -> Option<Vec2> {
    match self.intersects_impl(other, ignore_endpoints, lines, true) {
      Intersection::At(point) => Some(point),
      _ => None,
    }
  }

  /// The intersection point of the infinite lines through the two
  /// segments.
  ///
  /// Port of `SEG::IntersectLines`,
  /// `libs/kimath/include/geometry/seg.h:216`. The hull builders lean on
  /// it to intersect offset lines, so it is one of the busiest members of
  /// the type.
  pub fn intersect_lines(&self, other: &Seg) -> Option<Vec2> {
    self.intersect(other, false, true)
  }

  /// The shared intersection primitive.
  ///
  /// Port of `SEG::intersects`,
  /// `libs/kimath/src/geometry/seg.cpp:308`. `want_point` stands for
  /// KiCad's `aPt != nullptr`.
  fn intersects_impl(
    &self,
    other: &Seg,
    ignore_endpoints: bool,
    lines: bool,
    want_point: bool,
  ) -> Intersection {
    // Bounding box rejection, skipped for infinite lines.
    if !lines {
      let this_min_x = self.a.x.min(self.b.x);
      let this_max_x = self.a.x.max(self.b.x);
      let this_min_y = self.a.y.min(self.b.y);
      let this_max_y = self.a.y.max(self.b.y);

      let other_min_x = other.a.x.min(other.b.x);
      let other_max_x = other.a.x.max(other.b.x);
      let other_min_y = other.a.y.min(other.b.y);
      let other_max_y = other.a.y.max(other.b.y);

      if this_max_x < other_min_x
        || other_max_x < this_min_x
        || this_max_y < other_min_y
        || other_max_y < this_min_y
      {
        return Intersection::Missing;
      }
    }

    let dir1 = self.b.widening_sub(self.a);
    let dir2 = other.b.widening_sub(other.a);
    let offset = other.a.widening_sub(self.a);
    let determinant = dir2.cross(dir1);

    if determinant == 0 {
      if dir1.cross(offset) != 0 {
        // Parallel but not collinear.
        return Intersection::Missing;
      }

      if lines {
        if !want_point {
          return Intersection::Present;
        }

        if other.a == other.b {
          return Intersection::At(other.a);
        }

        if self.a == self.b {
          return Intersection::At(self.a);
        }

        // KiCad's `( A + aSeg.A ) / 2` adds in 32 bits and then rounds
        // each component half away from zero (`seg.cpp:366`,
        // `vector2d.h:524`). The sum is widened here, so two collinear
        // lines far out in the coordinate range answer with their real
        // midpoint rather than a wrapped one.
        let midpoint = Vec2L::new(
          kiround_i64(
            (i64::from(self.a.x) + i64::from(other.a.x)) as f64 / 2.0,
          ),
          kiround_i64(
            (i64::from(self.a.y) + i64::from(other.a.y)) as f64 / 2.0,
          ),
        );

        return Intersection::At(midpoint.saturating_to_vec2());
      }

      let use_x_axis = dir1.x.abs() >= dir1.y.abs();

      return self.check_collinear_overlap(
        other,
        use_x_axis,
        ignore_endpoints,
        want_point,
      );
    }

    let param2_num = dir2.cross(offset);
    let param1_num = dir1.cross(offset);

    if !lines {
      let outside = if determinant > 0 {
        param1_num < 0
          || param1_num > determinant
          || param2_num < 0
          || param2_num > determinant
      } else {
        param1_num > 0
          || param1_num < determinant
          || param2_num > 0
          || param2_num < determinant
      };

      if outside {
        return Intersection::Missing;
      }

      if ignore_endpoints
        && (param1_num == 0 || param1_num == determinant)
        && (param2_num == 0 || param2_num == determinant)
      {
        return Intersection::Missing;
      }
    }

    if !want_point {
      return Intersection::Present;
    }

    let x = i64::from(other.a.x) + rescale(param1_num, dir2.x, determinant);
    let y = i64::from(other.a.y) + rescale(param1_num, dir2.y, determinant);

    if x > i64::from(i32::MAX)
      || x < i64::from(i32::MIN)
      || y > i64::from(i32::MAX)
      || y < i64::from(i32::MIN)
    {
      return Intersection::Missing;
    }

    Intersection::At(Vec2::new(x as i32, y as i32))
  }

  /// The overlap of two collinear segments, projected onto one axis.
  ///
  /// Port of `SEG::checkCollinearOverlap`,
  /// `libs/kimath/src/geometry/seg.cpp:216`. The projection axis is the
  /// one along which this segment is longer, so a vertical segment is
  /// compared on `y`.
  ///
  /// KiCad computes the midpoint of the overlap and the second coordinate
  /// in `int`, which overflows for a segment that spans more than half the
  /// coordinate range; both are computed in `i64` here and narrowed at the
  /// end. The midpoint still truncates towards zero rather than rounding,
  /// because that is what an integer division does in C++ and the ported
  /// test cases depend on it.
  fn check_collinear_overlap(
    &self,
    other: &Seg,
    use_x_axis: bool,
    ignore_endpoints: bool,
    want_point: bool,
  ) -> Intersection {
    let (seg1_start, seg1_end, seg2_start, seg2_end, other1_start, other1_end) =
      if use_x_axis {
        (self.a.x, self.b.x, other.a.x, other.b.x, self.a.y, self.b.y)
      } else {
        (self.a.y, self.b.y, other.a.y, other.b.y, self.a.x, self.b.x)
      };

    let seg1_min = seg1_start.min(seg1_end);
    let seg1_max = seg1_start.max(seg1_end);
    let seg2_min = seg2_start.min(seg2_end);
    let seg2_max = seg2_start.max(seg2_end);

    if seg1_max < seg2_min || seg2_max < seg1_min {
      return Intersection::Missing;
    }

    let overlap_start = seg1_min.max(seg2_min);
    let overlap_end = seg1_max.min(seg2_max);

    if ignore_endpoints && overlap_start == overlap_end {
      let touches_first =
        overlap_start == seg1_min || overlap_start == seg1_max;
      let touches_second =
        overlap_start == seg2_min || overlap_start == seg2_max;

      if touches_first && touches_second {
        return Intersection::Missing;
      }
    }

    if !want_point {
      return Intersection::Present;
    }

    let projected = (i64::from(overlap_start) + i64::from(overlap_end)) / 2;

    let other_coordinate = if seg1_end == seg1_start {
      i64::from(other1_start)
    } else {
      i64::from(other1_start)
        + rescale(
          projected - i64::from(seg1_start),
          i64::from(other1_end) - i64::from(other1_start),
          i64::from(seg1_end) - i64::from(seg1_start),
        )
    };

    let point = if use_x_axis {
      Vec2L::new(projected, other_coordinate)
    } else {
      Vec2L::new(other_coordinate, projected)
    };

    Intersection::At(point.saturating_to_vec2())
  }

  /// Whether two segments come closer to each other than a clearance.
  ///
  /// Port of `SEG::Collide`,
  /// `libs/kimath/src/geometry/seg.cpp:538`. A negative clearance is
  /// refused outright. The comparison against the clearance is strict, so
  /// two segments exactly `clearance` apart do not collide, and a distance
  /// of zero always does. Degenerate segments are handled before the
  /// intersection test for the same reason as in
  /// [`Seg::squared_distance_to_segment`].
  ///
  /// The router never calls this directly, it reaches it through the shape
  /// level dispatch, but the `-1` clearance fudge in `pns_item.cpp:249`
  /// only makes sense against this strict comparison.
  pub fn collide(&self, other: &Seg, clearance: i32) -> SegCollision {
    if clearance < 0 {
      return SegCollision {
        collides: false,
        actual: 0,
      };
    }

    if self.a == self.b {
      let distance = other.distance_to_point(self.a);

      return SegCollision {
        collides: distance == 0 || distance < clearance,
        actual: distance,
      };
    }

    if other.a == other.b {
      let distance = self.distance_to_point(other.a);

      return SegCollision {
        collides: distance == 0 || distance < clearance,
        actual: distance,
      };
    }

    if self.intersects(other) {
      return SegCollision {
        collides: true,
        actual: 0,
      };
    }

    let clearance_squared = i64::from(clearance) * i64::from(clearance);
    let mut min_distance_squared = i64::MAX;

    for distance_squared in [
      self.squared_distance_to_point(other.a),
      self.squared_distance_to_point(other.b),
      other.squared_distance_to_point(self.a),
      other.squared_distance_to_point(self.b),
    ] {
      if distance_squared == 0 {
        return SegCollision {
          collides: true,
          actual: 0,
        };
      }

      min_distance_squared = min_distance_squared.min(distance_squared);
    }

    SegCollision {
      collides: min_distance_squared < clearance_squared,
      actual: distance_from_squared(min_distance_squared),
    }
  }
}

/// Port of `SEG::operator==`,
/// `libs/kimath/include/geometry/seg.h:109`, which compares the endpoints
/// and ignores the parent shape index.
impl PartialEq for Seg {
  fn eq(&self, other: &Self) -> bool {
    self.a == other.a && self.b == other.b
  }
}

impl Eq for Seg {}

#[cfg(test)]
mod tests {
  use super::*;

  /// `EndpointCtorMod`,
  /// `qa/tests/libs/kimath/geometry/test_segment.cpp:226`.
  #[test]
  fn endpoints_are_public_and_mutable() {
    let point_a = Vec2::new(10, 20);
    let point_b = Vec2::new(100, 200);

    let mut segment = Seg::new(point_a, point_b);

    assert_eq!(point_a, Vec2::new(10, 20));
    assert_eq!(point_b, Vec2::new(100, 200));

    segment.a += Vec2::new(10, 10);
    segment.b += Vec2::new(100, 100);

    assert_eq!(segment.a, Vec2::new(20, 30));
    assert_eq!(segment.b, Vec2::new(200, 300));
  }

  /// A fresh segment carries no parent shape index, `seg.h:53`.
  #[test]
  fn index_defaults_to_minus_one() {
    assert_eq!(Seg::from_coords(0, 0, 10, 0).index, -1);
    assert_eq!(Seg::from_coords(0, 0, 10, 0).index, Seg::NO_INDEX);
    assert_eq!(
      Seg::with_index(Vec2::new(0, 0), Vec2::new(10, 0), 7).index,
      7
    );
  }

  /// Equality ignores the index and the direction matters, `seg.h:109`.
  #[test]
  fn equality_compares_the_endpoints_only() {
    let plain = Seg::from_coords(0, 0, 10, 0);
    let indexed = Seg::with_index(Vec2::new(0, 0), Vec2::new(10, 0), 3);

    assert_eq!(plain, indexed);
    assert_ne!(plain, plain.reversed());
  }

  /// The distance between two segments is symmetric and agrees with a
  /// zero clearance collision, which is what
  /// `SegDistanceCorrect`/`SegCollideCorrect` check together
  /// (`test_segment.cpp:70`, `:38`).
  fn check_segment_distance(a: &Seg, b: &Seg, expected: i32) {
    assert_eq!(a.distance_to_segment(b), expected, "a to b");
    assert_eq!(b.distance_to_segment(a), expected, "b to a");

    let expect_collision = expected == 0;
    assert_eq!(a.collide(b, 0).collides, expect_collision, "collide a to b");
    assert_eq!(b.collide(a, 0).collides, expect_collision, "collide b to a");
  }

  /// `seg_seg_dist_cases`, `test_segment.cpp:255`, with KiCad's expected
  /// values verbatim.
  #[test]
  fn segment_to_segment_distance() {
    // Parallel, 10 apart.
    check_segment_distance(
      &Seg::from_coords(0, 0, 10, 0),
      &Seg::from_coords(0, 10, 10, 10),
      10,
    );
    // Non-parallel, 10 apart.
    check_segment_distance(
      &Seg::from_coords(0, -5, 10, 0),
      &Seg::from_coords(0, 10, 10, 10),
      10,
    );
    // Co-incident.
    check_segment_distance(
      &Seg::from_coords(0, 0, 30, 0),
      &Seg::from_coords(10, 0, 20, 0),
      0,
    );
    // Crossing.
    check_segment_distance(
      &Seg::from_coords(0, -10, 0, 10),
      &Seg::from_coords(-20, 0, 20, 0),
      0,
    );
    // T-junction.
    check_segment_distance(
      &Seg::from_coords(0, -10, 0, 10),
      &Seg::from_coords(-20, 0, 0, 0),
      0,
    );
    // T-junction (no touch).
    check_segment_distance(
      &Seg::from_coords(0, -10, 0, 10),
      &Seg::from_coords(-20, 0, -2, 0),
      2,
    );
    // Zero-length segment A.
    check_segment_distance(
      &Seg::from_coords(0, 0, 0, 0),
      &Seg::from_coords(10, 0, 20, 0),
      10,
    );
    // Zero-length segment B.
    check_segment_distance(
      &Seg::from_coords(10, 0, 20, 0),
      &Seg::from_coords(0, 0, 0, 0),
      10,
    );
    // Both zero-length.
    check_segment_distance(
      &Seg::from_coords(0, 0, 0, 0),
      &Seg::from_coords(10, 0, 10, 0),
      10,
    );
  }

  /// `seg_vec_dist_cases`, `test_segment.cpp:329`, with KiCad's expected
  /// values verbatim. The squared distance is never negative, which
  /// `SegVecDistanceCorrect` asserts at `test_segment.cpp:107`.
  #[test]
  fn segment_to_point_distance() {
    let cases: [(Seg, Vec2, i32); 8] = [
      // On endpoint.
      (Seg::from_coords(0, 0, 10, 0), Vec2::new(0, 0), 0),
      // On segment.
      (Seg::from_coords(0, 0, 10, 0), Vec2::new(3, 0), 0),
      // At side.
      (Seg::from_coords(0, 0, 10, 0), Vec2::new(3, 2), 2),
      // At end (collinear).
      (Seg::from_coords(0, 0, 10, 0), Vec2::new(12, 0), 2),
      // At end (not collinear), sqrt(200^2 + 200^2) = 282.8 truncated.
      (Seg::from_coords(0, 0, 1000, 0), Vec2::new(1200, 200), 282),
      // Issue 18473, an inside hit with a rounding error.
      (
        Seg::from_coords(187360000, 42510000, 105796472, 42510000),
        Vec2::new(106645000, 42510000),
        0,
      ),
      // Straight line x distance.
      (
        Seg::from_coords(187360000, 42510000, 105796472, 42510000),
        Vec2::new(197360000, 42510000),
        10000000,
      ),
      // Straight line -x distance.
      (
        Seg::from_coords(187360000, 42510000, 105796472, 42510000),
        Vec2::new(104796472, 42510000),
        1000000,
      ),
    ];

    for (segment, point, expected) in cases {
      assert!(segment.squared_distance_to_point(point) >= 0);
      assert_eq!(
        segment.distance_to_point(point),
        expected,
        "{segment:?} to {point:?}"
      );
    }
  }

  /// The distance is exactly zero on both endpoints and grows linearly
  /// past them, which is the boundary between the three branches of
  /// `SEG::SquaredDistance` (`seg.cpp:717`, `:722`).
  #[test]
  fn distance_at_and_past_the_endpoints() {
    let segment = Seg::from_coords(0, 0, 10, 0);

    assert_eq!(segment.distance_to_point(Vec2::new(0, 0)), 0);
    assert_eq!(segment.distance_to_point(Vec2::new(10, 0)), 0);
    assert_eq!(segment.squared_distance_to_point(Vec2::new(0, 0)), 0);
    assert_eq!(segment.squared_distance_to_point(Vec2::new(10, 0)), 0);

    // Just past A, so the `e <= 0` branch.
    assert_eq!(segment.squared_distance_to_point(Vec2::new(-1, 0)), 1);
    assert_eq!(segment.squared_distance_to_point(Vec2::new(-7, 0)), 49);
    // Just past B, so the `e >= f` branch.
    assert_eq!(segment.squared_distance_to_point(Vec2::new(11, 0)), 1);
    assert_eq!(segment.squared_distance_to_point(Vec2::new(17, 0)), 49);
    // Diagonally past B.
    assert_eq!(segment.squared_distance_to_point(Vec2::new(13, 4)), 25);
    assert_eq!(segment.distance_to_point(Vec2::new(13, 4)), 5);
  }

  /// A dot product of two widened differences overflows `i64` once the
  /// coordinates approach plus or minus 1.5e9, so the squared distance
  /// computes in `i128`. Without the widening the projection wraps
  /// negative and the answer comes out as the squared distance to `A`
  /// instead of zero.
  #[test]
  fn squared_distance_widens_past_the_i64_product() {
    const FAR: i32 = 1_500_000_000;

    let segment = Seg::from_coords(-FAR, -FAR, FAR, FAR);
    let corner = Vec2::new(FAR, FAR);

    // The naive projection is 2 * (3e9)^2 = 1.8e19, past i64::MAX.
    let difference = i128::from(FAR) * 2;
    assert!(2 * difference * difference > i128::from(i64::MAX));

    // The point is the far endpoint, so the distance is exactly zero.
    assert_eq!(segment.squared_distance_to_point(corner), 0);
    assert_eq!(segment.distance_to_point(corner), 0);
    assert!(segment.contains_point(corner));

    // And the same through the segment to segment form, whose degenerate
    // shortcut forwards to the point form.
    let point_segment = Seg::new(corner, corner);
    assert_eq!(segment.squared_distance_to_segment(&point_segment), 0);
    assert_eq!(point_segment.squared_distance_to_segment(&segment), 0);

    // A point one nanometre off the far end, on the perpendicular.
    let beside = Vec2::new(FAR - 1, FAR + 1);
    assert_eq!(segment.squared_distance_to_point(beside), 2);
  }

  /// `seg_seg_coll_cases`, `test_segment.cpp:402`, with KiCad's expected
  /// values verbatim. Collision is symmetric, which
  /// `SegCollideCorrect` checks at `test_segment.cpp:38`.
  #[test]
  fn segment_to_segment_collision() {
    let cases: [(Seg, Seg, i32, bool); 17] = [
      // Parallel, 10 apart, 5 / 10 / 11 clear.
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 10, 10, 10),
        5,
        false,
      ),
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 10, 10, 10),
        10,
        false,
      ),
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 10, 10, 10),
        11,
        true,
      ),
      // T-junction, 2 apart, 2 / 3 clear.
      (
        Seg::from_coords(0, -10, 0, 0),
        Seg::from_coords(-20, 0, -2, 0),
        2,
        false,
      ),
      (
        Seg::from_coords(0, -10, 0, 0),
        Seg::from_coords(-20, 0, -2, 0),
        3,
        true,
      ),
      // Zero-length segment A, 10 apart, 0 / 9 / 10 / 11 clear.
      (
        Seg::from_coords(0, 0, 0, 0),
        Seg::from_coords(10, 0, 20, 0),
        0,
        false,
      ),
      (
        Seg::from_coords(0, 0, 0, 0),
        Seg::from_coords(10, 0, 20, 0),
        9,
        false,
      ),
      (
        Seg::from_coords(0, 0, 0, 0),
        Seg::from_coords(10, 0, 20, 0),
        10,
        false,
      ),
      (
        Seg::from_coords(0, 0, 0, 0),
        Seg::from_coords(10, 0, 20, 0),
        11,
        true,
      ),
      // Zero-length segment B, 10 apart.
      (
        Seg::from_coords(10, 0, 20, 0),
        Seg::from_coords(0, 0, 0, 0),
        0,
        false,
      ),
      // Both zero-length, same point.
      (
        Seg::from_coords(5, 5, 5, 5),
        Seg::from_coords(5, 5, 5, 5),
        0,
        true,
      ),
      // Both zero-length, 10 apart.
      (
        Seg::from_coords(0, 0, 0, 0),
        Seg::from_coords(10, 0, 10, 0),
        0,
        false,
      ),
      // Zero-length on segment.
      (
        Seg::from_coords(5, 0, 5, 0),
        Seg::from_coords(0, 0, 10, 0),
        0,
        true,
      ),
      // Zero-length near segment, x overlaps but y differs.
      (
        Seg::from_coords(5, 5, 5, 5),
        Seg::from_coords(0, 0, 10, 0),
        0,
        false,
      ),
      (
        Seg::from_coords(5, 5, 5, 5),
        Seg::from_coords(0, 0, 10, 0),
        4,
        false,
      ),
      (
        Seg::from_coords(5, 5, 5, 5),
        Seg::from_coords(0, 0, 10, 0),
        5,
        false,
      ),
      (
        Seg::from_coords(5, 5, 5, 5),
        Seg::from_coords(0, 0, 10, 0),
        6,
        true,
      ),
    ];

    for (a, b, clearance, expected) in cases {
      assert_eq!(
        a.collide(&b, clearance).collides,
        expected,
        "{a:?} to {b:?} at {clearance}"
      );
      assert_eq!(
        b.collide(&a, clearance).collides,
        expected,
        "{b:?} to {a:?} at {clearance}"
      );
    }
  }

  /// A negative clearance is refused and reports a zero distance,
  /// `seg.cpp:541`.
  #[test]
  fn collide_refuses_a_negative_clearance() {
    let a = Seg::from_coords(0, 0, 10, 0);
    let b = Seg::from_coords(0, 1, 10, 1);

    assert_eq!(
      a.collide(&b, -1),
      SegCollision {
        collides: false,
        actual: 0
      }
    );
  }

  /// The actual distance is written even when the segments do not
  /// collide, `seg.cpp:616`.
  #[test]
  fn collide_reports_the_distance_on_a_miss() {
    let a = Seg::from_coords(0, 0, 10, 0);
    let b = Seg::from_coords(0, 10, 10, 10);

    assert_eq!(
      a.collide(&b, 5),
      SegCollision {
        collides: false,
        actual: 10
      }
    );
    assert_eq!(
      a.collide(&b, 11),
      SegCollision {
        collides: true,
        actual: 10
      }
    );
  }

  /// `seg_vec_collinear_cases`, `test_segment.cpp:547`, with KiCad's
  /// expected values verbatim. Collinearity is symmetric, which
  /// `SegCollinearCorrect` checks at `test_segment.cpp:130`.
  #[test]
  fn segment_collinearity() {
    let cases: [(Seg, Seg, bool); 5] = [
      // coincident
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 0, 10, 0),
        true,
      ),
      // end-to-end
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(10, 0, 20, 0),
        true,
      ),
      // In segment
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(4, 0, 7, 0),
        true,
      ),
      // At side, parallel
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(4, 1, 7, 1),
        false,
      ),
      // crossing
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(5, -5, 5, 5),
        false,
      ),
    ];

    for (a, b, expected) in cases {
      assert_eq!(a.collinear(&b), expected, "{a:?} to {b:?}");
      assert_eq!(b.collinear(&a), expected, "{b:?} to {a:?}");
    }
  }

  /// The collinearity test is `<= 1` on the raw determinant, so it flips
  /// between a determinant of 1 and one of 2 and its effective distance
  /// tolerance shrinks as the segment gets longer (`seg.h:290`).
  #[test]
  fn collinearity_at_determinant_zero_one_and_two() {
    // A unit length reference along x: qa = 0, qb = 1, qc = 0, so the
    // determinant of a point is simply its y coordinate.
    let reference = Seg::from_coords(0, 0, 1, 0);

    let (qa, qb, qc) = reference.canonical_coefs();
    assert_eq!((qa, qb, qc), (0, 1, 0));

    assert!(reference.collinear(&Seg::from_coords(4, 0, 7, 0)));
    assert!(reference.collinear(&Seg::from_coords(4, 1, 7, 1)));
    assert!(!reference.collinear(&Seg::from_coords(4, 2, 7, 2)));

    // One endpoint at determinant 1 and the other at 2 is enough to fail.
    assert!(!reference.collinear(&Seg::from_coords(4, 1, 7, 2)));

    // A segment ten times longer scales every determinant by ten, so the
    // same one nanometre offset no longer passes.
    let long = Seg::from_coords(0, 0, 10, 0);
    assert!(!long.collinear(&Seg::from_coords(4, 1, 7, 1)));
  }

  /// `seg_vec_parallel_cases`, `test_segment.cpp:592`, with KiCad's
  /// expected values verbatim.
  #[test]
  fn segment_parallelism() {
    let cases: [(Seg, Seg, bool); 5] = [
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 0, 10, 0),
        true,
      ),
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(10, 0, 20, 0),
        true,
      ),
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(4, 0, 7, 0),
        true,
      ),
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(4, 1, 7, 1),
        true,
      ),
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(5, -5, 5, 5),
        false,
      ),
    ];

    for (a, b, expected) in cases {
      let threshold = Seg::APPROX_DISTANCE_THRESHOLD;

      assert_eq!(a.approx_parallel(&b, threshold), expected, "{a:?} {b:?}");
      assert_eq!(b.approx_parallel(&a, threshold), expected, "{b:?} {a:?}");
    }
  }

  /// `seg_vec_perpendicular_cases`, `test_segment.cpp:637`, with KiCad's
  /// expected values verbatim.
  #[test]
  fn segment_perpendicularity() {
    let cases: [(Seg, Seg, bool); 9] = [
      // coincident
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 0, 10, 0),
        false,
      ),
      // end-to-end
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(10, 0, 20, 0),
        false,
      ),
      // In segment
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(4, 0, 7, 0),
        false,
      ),
      // At side, parallel
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(4, 1, 7, 1),
        false,
      ),
      // crossing 45 deg
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 0, 5, 5),
        false,
      ),
      // very nearly perpendicular, an error margin of 1 IU is allowed
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 0, 1, 10),
        true,
      ),
      // not really perpendicular
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 0, 3, 10),
        false,
      ),
      // perpendicular
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 0, 0, 10),
        true,
      ),
      // perpendicular not intersecting
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(15, 5, 15, 10),
        true,
      ),
    ];

    for (a, b, expected) in cases {
      assert_eq!(a.approx_perpendicular(&b), expected, "{a:?} {b:?}");
      assert_eq!(b.approx_perpendicular(&a), expected, "{b:?} {a:?}");
    }
  }

  /// `SegCreatePerpendicular`, `test_segment.cpp:761`, restricted to the
  /// cases KiCad drives through `PerpendicularSeg`.
  #[test]
  fn perpendicular_segment_passes_through_the_point() {
    let cases: [(Seg, Vec2); 6] = [
      (Seg::from_coords(0, 0, 10, 0), Vec2::new(0, 0)),
      (Seg::from_coords(0, 0, 10, 0), Vec2::new(5, 0)),
      (Seg::from_coords(0, 0, 10, 0), Vec2::new(20, 20)),
      (Seg::from_coords(0, 0, 0, 10), Vec2::new(0, 0)),
      (Seg::from_coords(0, 0, 0, 10), Vec2::new(0, 5)),
      (Seg::from_coords(0, 0, 0, 10), Vec2::new(20, 20)),
    ];

    for (segment, point) in cases {
      let perpendicular = segment.perpendicular_seg(point);

      assert!(perpendicular.approx_perpendicular(&segment));
      assert!(segment.approx_perpendicular(&perpendicular));
      assert_eq!(perpendicular.distance_to_point(point), 0);
    }
  }

  /// `ApproxCollinear` keeps the sign convention of
  /// `mutualDistanceSquared` and normalises by the longer segment's
  /// length, so a one nanometre offset passes at the default threshold
  /// whatever the length (`seg.cpp:791`).
  #[test]
  fn approximate_collinearity() {
    let threshold = Seg::APPROX_DISTANCE_THRESHOLD;
    let reference = Seg::from_coords(0, 0, 1000, 0);

    assert!(
      reference.approx_collinear(&Seg::from_coords(4, 0, 7, 0), threshold)
    );
    assert!(
      reference.approx_collinear(&Seg::from_coords(4, 1, 7, 1), threshold)
    );
    assert!(
      !reference.approx_collinear(&Seg::from_coords(4, 2, 7, 2), threshold)
    );
    assert!(reference.approx_collinear(&Seg::from_coords(4, 2, 7, 2), 2));

    // A degenerate longer segment has no line to measure against.
    let point = Seg::from_coords(5, 5, 5, 5);
    assert!(!point.approx_collinear(&point, threshold));
    assert!(!point.approx_parallel(&point, threshold));
  }

  /// `LineDistance` and `LineDistanceSided`, `test_segment.cpp:770` and
  /// `:778`.
  #[test]
  fn line_distance_to_a_point() {
    let segment = Seg::from_coords(0, 0, 10, 0);

    assert_eq!(segment.line_distance(Vec2::new(5, 0)), 0);
    assert_eq!(segment.line_distance(Vec2::new(5, 8)), 8);
    assert_eq!(segment.line_distance_signed(Vec2::new(5, 8)), 8);
    assert_eq!(segment.line_distance_signed(Vec2::new(5, -8)), -8);

    // Past the endpoints the infinite line is still the reference.
    assert_eq!(segment.line_distance(Vec2::new(-100, 3)), 3);
    assert_eq!(segment.line_distance(Vec2::new(100, 3)), 3);

    // A degenerate segment has no line, so every distance is zero.
    let point = Seg::from_coords(5, 5, 5, 5);
    assert_eq!(point.line_distance(Vec2::new(0, 0)), 0);
  }

  /// `SEG::Side` is the sign of the determinant, `seg.h:139`. Negative is
  /// left of the directed line, which is above it on screen because `y`
  /// grows downwards.
  #[test]
  fn side_of_the_directed_line() {
    let segment = Seg::from_coords(0, 0, 10, 0);

    assert_eq!(segment.side(Vec2::new(5, -1)), -1);
    assert_eq!(segment.side(Vec2::new(5, 0)), 0);
    assert_eq!(segment.side(Vec2::new(5, 1)), 1);
    assert_eq!(segment.side(Vec2::new(-100, 0)), 0);
    assert_eq!(segment.reversed().side(Vec2::new(5, 1)), -1);
  }

  /// `SEG::Contains( VECTOR2I )` is a squared tolerance of 3, so a point
  /// at squared distance 3 is on the segment and one at 4 is not
  /// (`seg.cpp:623`).
  ///
  /// A squared distance of exactly 3 is unreachable through the two
  /// endpoint branches, since no sum of two integer squares is 3. It is
  /// reachable through the interior branch, which rounds a `f64`
  /// correction back to an integer (`seg.cpp:729`): on the segment
  /// `(0, 0)` to `(1, 3)` the perpendicular squared distance of a point
  /// is `(3x - y)^2 / 10`, so `(2, 1)` sits at 2.5 and rounds to 3 while
  /// `(2, 0)` sits at 3.6 and rounds to 4.
  #[test]
  fn contains_point_at_squared_distance_three_and_four() {
    let steep = Seg::from_coords(0, 0, 1, 3);

    assert_eq!(steep.squared_distance_to_point(Vec2::new(2, 1)), 3);
    assert!(steep.contains_point(Vec2::new(2, 1)));

    assert_eq!(steep.squared_distance_to_point(Vec2::new(2, 0)), 4);
    assert!(!steep.contains_point(Vec2::new(2, 0)));

    // The same boundary on an axis aligned segment, where the reachable
    // values step from 1 to 4.
    let segment = Seg::from_coords(0, 0, 100, 0);

    assert_eq!(segment.squared_distance_to_point(Vec2::new(50, 1)), 1);
    assert!(segment.contains_point(Vec2::new(50, 1)));
    assert_eq!(segment.squared_distance_to_point(Vec2::new(50, 2)), 4);
    assert!(!segment.contains_point(Vec2::new(50, 2)));

    // Diagonally past the end: (1, 1) beyond B is squared distance 2.
    assert_eq!(segment.squared_distance_to_point(Vec2::new(101, 1)), 2);
    assert!(segment.contains_point(Vec2::new(101, 1)));
    assert_eq!(segment.squared_distance_to_point(Vec2::new(102, 0)), 4);
    assert!(!segment.contains_point(Vec2::new(102, 0)));

    // And past the start, through the other endpoint branch.
    assert_eq!(segment.squared_distance_to_point(Vec2::new(-1, -1)), 2);
    assert!(segment.contains_point(Vec2::new(-1, -1)));
  }

  /// `SEG::Contains( SEG )` and `SEG::Overlaps`, `seg.h:320` and `:297`.
  #[test]
  fn contains_and_overlaps_a_segment() {
    let segment = Seg::from_coords(0, 0, 100, 0);

    assert!(segment.contains_segment(&Seg::from_coords(10, 0, 90, 0)));
    assert!(segment.contains_segment(&Seg::from_coords(0, 0, 100, 0)));
    assert!(!segment.contains_segment(&Seg::from_coords(10, 0, 110, 0)));
    assert!(!segment.contains_segment(&Seg::from_coords(10, 5, 90, 5)));
    // A degenerate segment falls back to the point test.
    assert!(segment.contains_segment(&Seg::from_coords(50, 0, 50, 0)));
    assert!(!segment.contains_segment(&Seg::from_coords(50, 5, 50, 5)));

    assert!(segment.overlaps(&Seg::from_coords(50, 0, 150, 0)));
    assert!(segment.overlaps(&Seg::from_coords(-50, 0, 50, 0)));
    assert!(!segment.overlaps(&Seg::from_coords(150, 0, 250, 0)));
    assert!(!segment.overlaps(&Seg::from_coords(50, 5, 150, 5)));
    // A degenerate other segment on an endpoint does not overlap.
    assert!(!segment.overlaps(&Seg::from_coords(0, 0, 0, 0)));
    assert!(!segment.overlaps(&Seg::from_coords(100, 0, 100, 0)));
    assert!(segment.overlaps(&Seg::from_coords(50, 0, 50, 0)));
  }

  /// `seg_intersect_cases`, `test_segment.cpp:800`, with KiCad's expected
  /// values verbatim. The expected point is checked with the tolerance of
  /// 1 that `SegIntersectCorrect` allows (`test_segment.cpp:1029`), and
  /// the cases whose expected point is the default `(0, 0)` only check
  /// whether an intersection was found.
  #[test]
  fn segment_intersection_cases() {
    // segment a, segment b, ignore endpoints, lines, expected point.
    let cases: [(Seg, Seg, bool, bool, Option<Vec2>); 25] = [
      // Crossing at origin.
      (
        Seg::from_coords(-10, 0, 10, 0),
        Seg::from_coords(0, -10, 0, 10),
        false,
        false,
        Some(Vec2::new(0, 0)),
      ),
      // Crossing at (5,5).
      (
        Seg::from_coords(0, 5, 10, 5),
        Seg::from_coords(5, 0, 5, 10),
        false,
        false,
        Some(Vec2::new(5, 5)),
      ),
      // T-junction intersection.
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(5, -5, 5, 0),
        false,
        false,
        Some(Vec2::new(5, 0)),
      ),
      // Parallel segments.
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 5, 10, 5),
        false,
        false,
        None,
      ),
      // Separated segments.
      (
        Seg::from_coords(0, 0, 5, 0),
        Seg::from_coords(10, 0, 15, 0),
        false,
        false,
        None,
      ),
      // Lines would intersect, but segments do not.
      (
        Seg::from_coords(0, 0, 2, 0),
        Seg::from_coords(5, -5, 5, 5),
        false,
        false,
        None,
      ),
      // Endpoint touching, should intersect.
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(10, 0, 20, 0),
        false,
        false,
        Some(Vec2::new(10, 0)),
      ),
      // Endpoint touching, ignore endpoints.
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(10, 0, 20, 0),
        true,
        false,
        None,
      ),
      // Endpoint touching at an angle.
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(10, 0, 15, 5),
        false,
        false,
        Some(Vec2::new(10, 0)),
      ),
      // Collinear overlapping segments, midpoint of the overlap [5,10].
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(5, 0, 15, 0),
        false,
        false,
        Some(Vec2::new(7, 0)),
      ),
      // Collinear non-overlapping segments.
      (
        Seg::from_coords(0, 0, 5, 0),
        Seg::from_coords(10, 0, 15, 0),
        false,
        false,
        None,
      ),
      // Collinear touching at an endpoint.
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(10, 0, 20, 0),
        false,
        false,
        Some(Vec2::new(10, 0)),
      ),
      // Collinear contained segment.
      (
        Seg::from_coords(0, 0, 20, 0),
        Seg::from_coords(5, 0, 15, 0),
        false,
        false,
        Some(Vec2::new(10, 0)),
      ),
      // Collinear vertical overlapping.
      (
        Seg::from_coords(5, 0, 5, 10),
        Seg::from_coords(5, 5, 5, 15),
        false,
        false,
        Some(Vec2::new(5, 7)),
      ),
      // Lines intersect, segments do not.
      (
        Seg::from_coords(0, 0, 2, 0),
        Seg::from_coords(5, -5, 5, 5),
        false,
        true,
        Some(Vec2::new(5, 0)),
      ),
      // Parallel lines (infinite).
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(0, 5, 10, 5),
        false,
        true,
        None,
      ),
      // Collinear lines (infinite), midpoint between the two starts.
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(20, 0, 30, 0),
        false,
        true,
        Some(Vec2::new(10, 0)),
      ),
      // Zero-length segment intersection.
      (
        Seg::from_coords(5, 5, 5, 5),
        Seg::from_coords(0, 5, 10, 5),
        false,
        false,
        Some(Vec2::new(5, 5)),
      ),
      // Both zero-length, same point.
      (
        Seg::from_coords(5, 5, 5, 5),
        Seg::from_coords(5, 5, 5, 5),
        false,
        false,
        Some(Vec2::new(5, 5)),
      ),
      // Both zero-length, different points.
      (
        Seg::from_coords(5, 5, 5, 5),
        Seg::from_coords(10, 10, 10, 10),
        false,
        false,
        None,
      ),
      // 45-degree crossing.
      (
        Seg::from_coords(0, 0, 10, 10),
        Seg::from_coords(0, 10, 10, 0),
        false,
        false,
        Some(Vec2::new(5, 5)),
      ),
      // Arbitrary angle crossing.
      (
        Seg::from_coords(0, 0, 6, 8),
        Seg::from_coords(0, 8, 6, 0),
        false,
        false,
        Some(Vec2::new(3, 4)),
      ),
      // Far apart horizontal segments.
      (
        Seg::from_coords(0, 0, 10, 0),
        Seg::from_coords(100, 0, 110, 0),
        false,
        false,
        None,
      ),
      // Far apart vertical segments.
      (
        Seg::from_coords(0, 0, 0, 10),
        Seg::from_coords(0, 100, 0, 110),
        false,
        false,
        None,
      ),
      // Far apart diagonal segments.
      (
        Seg::from_coords(0, 0, 10, 10),
        Seg::from_coords(100, 100, 110, 110),
        false,
        false,
        None,
      ),
    ];

    for (a, b, ignore_endpoints, lines, expected) in cases {
      let forward = a.intersect(&b, ignore_endpoints, lines);
      let backward = b.intersect(&a, ignore_endpoints, lines);

      assert_eq!(
        forward.is_some(),
        expected.is_some(),
        "{a:?} to {b:?} lines {lines}"
      );
      assert_eq!(
        backward.is_some(),
        expected.is_some(),
        "{b:?} to {a:?} lines {lines}"
      );

      if let Some(point) = expected {
        for found in [forward, backward] {
          let found = found.expect("an intersection was expected");

          assert!(
            (found.x - point.x).abs() <= 1 && (found.y - point.y).abs() <= 1,
            "{a:?} to {b:?}: expected {point:?}, got {found:?}"
          );
        }
      }
    }
  }

  /// Two parallel segments never intersect, whether or not they are read
  /// as infinite lines (`seg.cpp:345`).
  #[test]
  fn intersect_parallel() {
    let a = Seg::from_coords(0, 0, 10, 0);
    let b = Seg::from_coords(0, 5, 10, 5);

    assert_eq!(a.intersect(&b, false, false), None);
    assert_eq!(a.intersect(&b, true, false), None);
    assert_eq!(a.intersect(&b, false, true), None);
    assert!(!a.intersects(&b));

    // Nearly parallel but not quite: a determinant of 1000 - 1000 = 0.
    let steep = Seg::from_coords(0, 0, 1000, 1);
    let beside = Seg::from_coords(0, 1, 1000, 2);
    assert_eq!(steep.intersect(&beside, false, false), None);
  }

  /// Collinear overlapping segments answer with the midpoint of the
  /// overlap interval, truncated towards zero (`seg.cpp:281`).
  #[test]
  fn intersect_collinear_overlapping() {
    let a = Seg::from_coords(0, 0, 10, 0);
    let b = Seg::from_coords(5, 0, 15, 0);

    // The overlap is [5, 10] and (5 + 10) / 2 truncates to 7.
    assert_eq!(a.intersect(&b, false, false), Some(Vec2::new(7, 0)));
    assert_eq!(b.intersect(&a, false, false), Some(Vec2::new(7, 0)));

    // A diagonal keeps its slope through the second coordinate.
    let diagonal_a = Seg::from_coords(0, 0, 10, 10);
    let diagonal_b = Seg::from_coords(5, 5, 15, 15);
    let hit = diagonal_a
      .intersect(&diagonal_b, false, false)
      .expect("collinear diagonals overlap");
    assert_eq!(hit.x, hit.y);
    assert!(hit.x >= 5 && hit.x <= 10);

    // No overlap at all.
    assert_eq!(
      Seg::from_coords(0, 0, 5, 0).intersect(
        &Seg::from_coords(10, 0, 15, 0),
        false,
        false
      ),
      None
    );
  }

  /// Endpoint contacts count unless `ignore_endpoints` is set, both for
  /// crossing segments (`seg.cpp:404`) and for collinear ones
  /// (`seg.cpp:247`).
  #[test]
  fn intersect_touching_at_an_endpoint() {
    let a = Seg::from_coords(0, 0, 10, 0);

    // Collinear, end to end.
    let collinear = Seg::from_coords(10, 0, 20, 0);
    assert_eq!(
      a.intersect(&collinear, false, false),
      Some(Vec2::new(10, 0))
    );
    assert_eq!(a.intersect(&collinear, true, false), None);

    // At an angle, end to end.
    let angled = Seg::from_coords(10, 0, 15, 5);
    assert_eq!(a.intersect(&angled, false, false), Some(Vec2::new(10, 0)));
    assert_eq!(a.intersect(&angled, true, false), None);

    // An endpoint of one landing in the middle of the other is not an
    // endpoint contact of both, so it survives the flag.
    let tee = Seg::from_coords(5, -5, 5, 0);
    assert_eq!(a.intersect(&tee, false, false), Some(Vec2::new(5, 0)));
    assert_eq!(a.intersect(&tee, true, false), Some(Vec2::new(5, 0)));

    // A plain crossing is unaffected.
    let crossing = Seg::from_coords(5, -5, 5, 5);
    assert_eq!(a.intersect(&crossing, true, false), Some(Vec2::new(5, 0)));
  }

  /// `IntersectLargeCoordinates`, `test_segment.cpp:1069`.
  #[test]
  fn intersect_large_coordinates() {
    let a = Seg::from_coords(1000000000, 0, -1000000000, 0);
    let b = Seg::from_coords(0, 1000000000, 0, -1000000000);

    assert_eq!(a.intersect(&b, false, false), Some(Vec2::new(0, 0)));
  }

  /// `IntersectOverflowDetection`, `test_segment.cpp:1082`, which only
  /// asks that the two extreme diagonals do not crash. The determinant is
  /// `2 * i32::MAX^2`, which is 8.5e9 below `i64::MAX`, so `i64` still
  /// carries it and the answer lands within one nanometre of the true
  /// half of the diagonal.
  #[test]
  fn intersect_does_not_overflow_on_the_extreme_diagonals() {
    let a = Seg::from_coords(0, 0, i32::MAX, i32::MAX);
    let b = Seg::from_coords(i32::MAX, 0, 0, i32::MAX);

    let hit = a.intersect(&b, false, false).expect("the diagonals cross");
    let half = i32::MAX / 2;

    assert!((hit.x - half).abs() <= 1, "{hit:?}");
    assert!((hit.y - half).abs() <= 1, "{hit:?}");
  }

  /// `IntersectBoundingBoxOptimization`, `test_segment.cpp:1197`.
  #[test]
  fn intersect_bounding_box_rejection() {
    // Clearly separated, rejected by the bounding boxes.
    assert_eq!(
      Seg::from_coords(0, 0, 10, 10).intersect(
        &Seg::from_coords(100, 100, 110, 110),
        false,
        false
      ),
      None
    );

    // Overlapping bounding boxes but no intersection.
    assert_eq!(
      Seg::from_coords(0, 0, 10, 0).intersect(
        &Seg::from_coords(5, 5, 15, 5),
        false,
        false
      ),
      None
    );

    // Touching bounding boxes and a real intersection.
    assert_eq!(
      Seg::from_coords(0, 0, 10, 10).intersect(
        &Seg::from_coords(10, 0, 0, 10),
        false,
        false
      ),
      Some(Vec2::new(5, 5))
    );
  }

  /// `IntersectPrecisionEdgeCases`, `test_segment.cpp:1101`, and the
  /// acute angle case of `IntersectNumericalStability`
  /// (`test_segment.cpp:1276`).
  #[test]
  fn intersect_precision_edge_cases() {
    let a = Seg::from_coords(0, 0, 1000000, 1);
    let b = Seg::from_coords(500000, -1, 500000, 2);

    let hit = a.intersect(&b, false, false).expect("the segments cross");
    assert_eq!(hit.x, 500000);
    assert!(hit.y >= 0 && hit.y <= 1);

    // Very small segments crossing near their midpoints.
    let small_a = Seg::from_coords(0, 0, 1, 1);
    let small_b = Seg::from_coords(0, 1, 1, 0);
    let small = small_a
      .intersect(&small_b, false, false)
      .expect("the segments cross");
    assert!(small.x >= 0 && small.x <= 1);
    assert!(small.y >= 0 && small.y <= 1);
  }

  /// `IntersectLineVsSegmentMode`, `test_segment.cpp:1224`.
  #[test]
  fn intersect_line_mode_versus_segment_mode() {
    let a = Seg::from_coords(0, 0, 5, 0);
    let b = Seg::from_coords(10, -5, 10, 5);

    assert_eq!(a.intersect(&b, false, false), None);
    assert_eq!(a.intersect_lines(&b), Some(Vec2::new(10, 0)));

    let c = Seg::from_coords(0, 0, 10, 0);
    let d = Seg::from_coords(20, 0, 30, 0);

    assert_eq!(c.intersect(&d, false, false), None);
    assert_eq!(c.intersect_lines(&d), Some(Vec2::new(10, 0)));

    // The collinear line answer is the midpoint between the two start
    // points, rounded half away from zero and computed in `i64` so that
    // it does not wrap the way KiCad's 32 bit sum would.
    // Both start points are near the top of the range, so their 32 bit
    // sum wraps while their midpoint fits comfortably.
    let near = Seg::from_coords(2_000_000_000, 0, 1_000_000_000, 0);
    let far = Seg::from_coords(2_100_000_000, 0, 1_500_000_000, 0);

    assert_eq!(
      near.intersect_lines(&far),
      Some(Vec2::new(2_050_000_000, 0))
    );
    assert_eq!(
      Seg::from_coords(0, 0, 10, 0)
        .intersect_lines(&Seg::from_coords(5, 0, 15, 0)),
      Some(Vec2::new(3, 0))
    );
  }

  /// `IntersectZeroLengthSegments`, `test_segment.cpp:1284`.
  #[test]
  fn intersect_zero_length_segments() {
    let point1 = Vec2::new(5, 5);
    let point2 = Vec2::new(10, 10);

    let point_seg1 = Seg::new(point1, point1);
    let point_seg2 = Seg::new(point2, point2);
    let normal = Seg::from_coords(0, 5, 10, 5);

    assert_eq!(point_seg1.intersect(&normal, false, false), Some(point1));
    assert_eq!(point_seg2.intersect(&normal, false, false), None);
    assert_eq!(
      point_seg1.intersect(&Seg::new(point1, point1), false, false),
      Some(point1)
    );
    assert_eq!(point_seg1.intersect(&point_seg2, false, false), None);

    let line = Seg::from_coords(0, 0, 1, 1);
    let on_line = Seg::from_coords(100, 100, 100, 100);

    assert_eq!(on_line.intersect(&line, false, false), None);
    assert_eq!(on_line.intersect_lines(&line), Some(Vec2::new(100, 100)));
  }

  /// `IntersectCollinearRegressionTests`, `test_segment.cpp:1142`.
  #[test]
  fn intersect_collinear_regressions() {
    let seg1 = Seg::from_coords(0, 5, 10, 5);
    let seg2 = Seg::from_coords(5, 5, 15, 5);
    let hit = seg1.intersect(&seg2, false, false).expect("overlap");
    assert_eq!(hit.y, 5);
    assert!(hit.x >= 5 && hit.x <= 10);

    let seg3 = Seg::from_coords(3, 0, 3, 20);
    let seg4 = Seg::from_coords(3, 5, 3, 15);
    let hit = seg3.intersect(&seg4, false, false).expect("containment");
    assert_eq!(hit.x, 3);
    assert!(hit.y >= 5 && hit.y <= 15);

    let seg7 = Seg::from_coords(0, 0, 5, 0);
    let seg8 = Seg::from_coords(5, 0, 10, 0);
    assert_eq!(seg7.intersect(&seg8, false, false), Some(Vec2::new(5, 0)));
    assert_eq!(seg7.intersect(&seg8, true, false), None);
  }

  /// The nearest point clamps to the endpoints outside the segment and
  /// projects inside it (`seg.cpp:629`), and a degenerate segment answers
  /// with its start point.
  #[test]
  fn nearest_point_to_a_point() {
    let segment = Seg::from_coords(0, 0, 100, 0);

    assert_eq!(
      segment.nearest_point_to_point(Vec2::new(-10, 10)),
      Vec2::new(0, 0)
    );
    assert_eq!(
      segment.nearest_point_to_point(Vec2::new(110, 10)),
      Vec2::new(100, 0)
    );
    assert_eq!(
      segment.nearest_point_to_point(Vec2::new(40, 10)),
      Vec2::new(40, 0)
    );
    assert_eq!(
      segment.nearest_point_to_point(Vec2::new(0, 0)),
      Vec2::new(0, 0)
    );

    let point = Seg::from_coords(5, 5, 5, 5);
    assert_eq!(
      point.nearest_point_to_point(Vec2::new(100, 100)),
      Vec2::new(5, 5)
    );
  }

  /// The nearest point and the nearest pair agree, and an intersection
  /// collapses both to the crossing point (`seg.cpp:116`, `:158`).
  #[test]
  fn nearest_point_and_points_between_segments() {
    let a = Seg::from_coords(0, 0, 10, 0);
    let b = Seg::from_coords(0, 4, 10, 4);

    assert_eq!(a.nearest_point_to_segment(&b), Vec2::new(0, 0));

    let pair = a.nearest_points(&b);
    assert_eq!(pair.squared_distance, 16);
    assert_eq!(pair.on_self, Vec2::new(0, 0));
    assert_eq!(pair.on_other, Vec2::new(0, 4));

    let crossing = Seg::from_coords(5, -5, 5, 5);
    assert_eq!(a.nearest_point_to_segment(&crossing), Vec2::new(5, 0));

    let pair = a.nearest_points(&crossing);
    assert_eq!(pair.squared_distance, 0);
    assert_eq!(pair.on_self, Vec2::new(5, 0));
    assert_eq!(pair.on_other, Vec2::new(5, 0));
  }

  /// The perpendicular projection is not clamped to the segment, unlike
  /// the nearest point (`seg.cpp:681`).
  #[test]
  fn line_projection_leaves_the_segment() {
    let segment = Seg::from_coords(0, 0, 10, 0);

    assert_eq!(segment.line_project(Vec2::new(5, 7)), Vec2::new(5, 0));
    assert_eq!(segment.line_project(Vec2::new(-40, 7)), Vec2::new(-40, 0));
    assert_eq!(segment.line_project(Vec2::new(40, 7)), Vec2::new(40, 0));

    let point = Seg::from_coords(5, 5, 5, 5);
    assert_eq!(point.line_project(Vec2::new(0, 0)), Vec2::new(5, 5));
  }

  /// Reflection across the axis, `seg.cpp:660`.
  #[test]
  fn reflect_point_across_the_segment() {
    let axis = Seg::from_coords(0, 0, 10, 0);

    assert_eq!(axis.reflect_point(Vec2::new(3, 4)), Vec2::new(3, -4));
    assert_eq!(axis.reflect_point(Vec2::new(3, 0)), Vec2::new(3, 0));
    assert_eq!(axis.reflect_point(Vec2::new(-7, -2)), Vec2::new(-7, 2));

    let diagonal = Seg::from_coords(0, 0, 10, 10);
    assert_eq!(diagonal.reflect_point(Vec2::new(4, 0)), Vec2::new(0, 4));

    // A degenerate axis leaves the point where it is.
    let point = Seg::from_coords(5, 5, 5, 5);
    assert_eq!(point.reflect_point(Vec2::new(1, 2)), Vec2::new(1, 2));
  }

  /// Length, squared length, reversal and the midpoint, `seg.h:339`,
  /// `:344`, `:364` and `:375`.
  #[test]
  fn length_reversal_and_centre() {
    let segment = Seg::from_coords(0, 0, 3, 4);

    assert_eq!(segment.length(), 5);
    assert_eq!(segment.squared_length(), 25);
    assert_eq!(Seg::from_coords(0, 0, 7, 7).length(), 10);

    let mut reversible = Seg::with_index(Vec2::new(0, 0), Vec2::new(3, 4), 9);
    reversible.reverse();
    assert_eq!(reversible.a, Vec2::new(3, 4));
    assert_eq!(reversible.b, Vec2::new(0, 0));
    assert_eq!(reversible.index, 9);
    assert_eq!(segment.reversed().index, Seg::NO_INDEX);

    // The midpoint rounds half away from zero, away from A.
    assert_eq!(Seg::from_coords(0, 0, 10, 0).center(), Vec2::new(5, 0));
    assert_eq!(Seg::from_coords(0, 0, 5, 0).center(), Vec2::new(3, 0));
    assert_eq!(Seg::from_coords(0, 0, -5, 0).center(), Vec2::new(-3, 0));
  }

  /// The angle between two segments, in degrees, with the exact axis and
  /// diagonal cases of `EDA_ANGLE( VECTOR2D )` (`seg.cpp:107`).
  #[test]
  fn angle_between_two_segments() {
    let horizontal = Seg::from_coords(0, 0, 10, 0);

    assert_eq!(
      horizontal.angle_degrees(&Seg::from_coords(0, 0, 20, 0)),
      0.0
    );
    assert_eq!(
      horizontal.angle_degrees(&Seg::from_coords(0, 0, 0, 10)),
      90.0
    );
    assert_eq!(
      horizontal.angle_degrees(&Seg::from_coords(0, 0, 5, 5)),
      45.0
    );
    assert_eq!(
      horizontal.angle_degrees(&Seg::from_coords(0, 0, -5, 5)),
      135.0
    );

    // The segments are directed, so reversing one turns the answer by
    // 180 degrees. `EDA_ANGLE::IsHorizontal` accepts both 0 and 180
    // (`eda_angle.h:142`), which is why the optimizer does not care.
    assert_eq!(
      horizontal.angle_degrees(&Seg::from_coords(20, 0, 0, 0)),
      180.0
    );

    // The optimizer's area constraint only asks whether the result is
    // horizontal, so exact zero matters (`pns_optimizer.cpp:232`).
    assert!(horizontal.angle_degrees(&Seg::from_coords(4, 7, 24, 7)) == 0.0);

    let arbitrary = Seg::from_coords(0, 0, 3, 4);
    let angle = arbitrary.angle_degrees(&horizontal);
    assert!((angle - 53.13010235415598).abs() < 1e-9);
  }
}
