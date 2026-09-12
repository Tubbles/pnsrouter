// SPDX-License-Identifier: GPL-3.0-or-later

//! Circular arcs.
//!
//! [`ShapeArc`] is the port of KiCad's `SHAPE_ARC`,
//! `libs/kimath/include/geometry/shape_arc.h:35`. It carries KiCad's three
//! point representation, a start, a mid and an end in nanometres plus a
//! track width, and **caches nothing**: the centre, the radius, both
//! endpoint angles, the sweep, the length and the bounding box are all
//! derived on demand. KiCad caches the first three and then has to decide
//! what happens when they go stale, which it does inconsistently
//! (`doc/reference/kicad/09-arcs.md` erratum E4). Deriving on demand
//! removes that class of bug and is the decision recorded in
//! `doc/work/012-arcs.md`.
//!
//! Why three points and not a centre or an angle: three points are exactly
//! representable in `i32` nanometres, the handedness falls out of one
//! `i64` cross product, and translation, mirroring and quarter turns are
//! exact on all three. A centre over-determines the arc and an angle needs
//! a second unit. Hosts that store a centre (Horizon) or an angle
//! (LibrePCB) convert at the boundary through
//! [`ShapeArc::from_start_end_center`] and
//! [`ShapeArc::from_start_end_angle`], both of which are documented as
//! lossy for exactly that reason. See note 09 sections 7.4 and 11.2.
//!
//! What is exact here and what is not, note 09 section 1.10: the three
//! points, [`ShapeArc::is_ccw`], equality and [`ShapeArc::chord`] are
//! integer and reproducible. [`ShapeArc::center`], [`ShapeArc::radius`],
//! the three angles, [`ShapeArc::length`],
//! [`ShapeArc::slice_contains_point`] and the interior points of
//! [`ShapeArc::convert_to_polyline`] all pass through `f64`. Where KiCad
//! computes in `double` this computes in `f64` and rounds in the same
//! places with the same rounding, so the values agree bit for bit; that is
//! deliberate and is what makes the mirrored `test_shape_arc.cpp` cases
//! meaningful.
//!
//! The centre in particular is **not a continuous function of the three
//! points**: `CalcArcCenter` snaps it to a multiple of 100 nm, or failing
//! that 10 nm, whenever its own propagated uncertainty covers the round
//! value. That is reproduced verbatim, see [`calc_arc_center`] and the
//! decision of 2026-09-12 in `doc/log/`.
//!
//! [`ShapeArc::collide_point`], [`ShapeArc::collide_seg`] and the four
//! `nearest_points` methods are the collision primitives everything above
//! this layer is built from. The pairwise rows that combine an arc with
//! another shape live in [`crate::geometry::collision`]; they are free
//! functions and are not reachable through [`crate::geometry::shape::Shape`]
//! until slice 4 of `doc/work/012-arcs.md` adds the variant.

use std::f64::consts::{FRAC_1_SQRT_2, PI};

use crate::geometry::box2::Box2;
use crate::geometry::collision::{
  ShapeCollision, circle_intersect_seg, circle_seg_collision,
};
use crate::geometry::line_chain::LineChain;
use crate::geometry::math::{
  Degrees, euclidean_norm_f64, kiround, rotate_point, rotate_point_f64, sign,
};
use crate::geometry::seg::{NearestPoints, Seg};
use crate::geometry::shape::rect_outline;
use crate::geometry::vec2::{Vec2, Vec2L};

/// The centre of the circle through three points, and where it came from.
///
/// `CalcArcCenter` never fails: when the three points do not span a
/// triangle it hands back one of three stand ins rather than nothing
/// (`libs/kimath/src/trigo.cpp:386`, `:398`, `:401`). The distinction has
/// to survive, because `GetCentralAngle`'s straight arc branch
/// (`libs/kimath/src/geometry/shape_arc.cpp:982`) depends on a stand in
/// existing and an `Option` here would force every call site to re-derive
/// that. Note 09 section 11.2 asks for exactly this shape.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum ArcCenter {
  /// The circumcentre of three points that span a triangle.
  ///
  /// This includes the two exact special cases KiCad short circuits, the
  /// perpendicular chord pair (`trigo.cpp:414`) and the full turn
  /// (`trigo.cpp:446`), both of which really are the circumcentre. It also
  /// includes the colinear fall through, where KiCad nudges the two
  /// bisector slopes apart by an epsilon and lands somewhere far away
  /// (`trigo.cpp:455`); the predicate that guards against trusting that
  /// one is [`ShapeArc::is_effective_line`], which is exact integer
  /// arithmetic, and it is the predicate KiCad itself uses.
  Circumcentre(Vec2),
  /// A stand in for three points that do not span a triangle: the centroid
  /// of a cluster inside a 5 nanometre box (`trigo.cpp:386`), or a chord
  /// midpoint when a pair of points is within 2 nanometres (`:398`,
  /// `:401`). No circle passes through all three, so nothing derived from
  /// this centre describes a real curve.
  Degenerate(Vec2),
}

impl ArcCenter {
  /// The centre itself, whichever branch produced it.
  ///
  /// KiCad's `GetCenter` (`shape_arc.cpp:952`) returns the point with no
  /// provenance, so this is the accessor that ports a call site verbatim.
  pub const fn point(self) -> Vec2 {
    match self {
      ArcCenter::Circumcentre(point) | ArcCenter::Degenerate(point) => point,
    }
  }

  /// Whether the three points failed to span a triangle.
  pub const fn is_degenerate(self) -> bool {
    matches!(self, ArcCenter::Degenerate(_))
  }
}

/// A circular arc through three points, with a track width.
///
/// Port of `SHAPE_ARC`, `libs/kimath/include/geometry/shape_arc.h:35`,
/// minus the three cached members (`:332` to `:334`). Equality compares
/// the three points and the width, as KiCad's does (`shape_arc.h:299`).
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct ShapeArc {
  /// KiCad's `m_start`, returned by `GetP0` (`shape_arc.h:110`).
  start: Vec2,
  /// KiCad's `m_mid`, returned by `GetArcMid` (`shape_arc.h:112`). It is
  /// a point on the arc between the two endpoints and not necessarily the
  /// halfway point: the three point constructor takes whatever it is
  /// given.
  mid: Vec2,
  /// KiCad's `m_end`, returned by `GetP1` (`shape_arc.h:111`).
  end: Vec2,
  /// The full track width, KiCad's `m_width` (`shape_arc.h:330`). An arc
  /// stored inside a chain carries zero; an arc that is a router item
  /// carries the track width (note 09 section 1.9).
  width: i32,
}

impl ShapeArc {
  /// The polyline accuracy the collision code assumes, 5000 nanometres.
  ///
  /// Port of `DefaultAccuracyForPCB`,
  /// `libs/kimath/include/geometry/shape_arc.h:275`, which returns
  /// `ARC_HIGH_DEF`, `pcbIUScale.mmToIU( 0.005 )` (`include/base_units.h:137`,
  /// `:128`). A chain stores its arcs at a fifth of this, 1000 nm, and
  /// `ArcHull` uses `ARC_LOW_DEF`, 20000 nm (note 09 section 1.5).
  pub const DEFAULT_ACCURACY_FOR_PCB: i32 = 5000;

  /// An arc through three points.
  ///
  /// Port of `SHAPE_ARC( const VECTOR2I&, const VECTOR2I&, const VECTOR2I&, int )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:62`, the canonical
  /// constructor. Nothing is derived here, so unlike KiCad's this cannot
  /// be given a stale centre.
  pub const fn new(start: Vec2, mid: Vec2, end: Vec2, width: i32) -> Self {
    Self {
      start,
      mid,
      end,
      width,
    }
  }

  /// An arc from a centre, a start point and a signed sweep.
  ///
  /// Port of `SHAPE_ARC( const VECTOR2I&, const VECTOR2I&, const EDA_ANGLE&, int )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:41`, which rotates the start
  /// point about the centre by half the sweep and by the whole sweep, both
  /// in `f64`, and rounds each result once.
  ///
  /// **Lossy, and the centre is not stored.** The mid and end points are
  /// rounded to whole nanometres, so [`ShapeArc::center`] recomputes a
  /// centre from the three rounded points and that centre generally
  /// differs from the one passed in. Note 09 section 11.2.
  pub fn from_center_start_angle(
    center: Vec2,
    start: Vec2,
    central_angle: Degrees,
    width: i32,
  ) -> Self {
    let center_f64 = to_f64(center);
    let start_f64 = to_f64(start);
    let mid = rotate_point_f64(start_f64, center_f64, -central_angle / 2.0);
    let end = rotate_point_f64(start_f64, center_f64, -central_angle);

    Self {
      start,
      mid: kiround_f64_pair(mid),
      end: kiround_f64_pair(end),
      width,
    }
  }

  /// An arc from its two endpoints and the angle it sweeps between them.
  ///
  /// Port of `ConstructFromStartEndAngle`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:198`, which places a centre
  /// with [`calc_arc_center_from_angle`], truncates it into `i32` and
  /// rotates the start point about it by half the angle to get the mid
  /// point.
  ///
  /// **Lossy, and the angle is not stored.** Start and end survive
  /// exactly; the mid point is a rounded rotation, and the centre KiCad
  /// used to place it is thrown away immediately, so
  /// [`ShapeArc::central_angle`] recomputed from the three points is not
  /// in general the angle passed in here. A host that keeps an angle of
  /// its own must not rewrite an arc the commit diff did not list as
  /// updated (note 09 section 11.2).
  ///
  /// KiCad takes the width as a `double` and assigns it into an `int`
  /// member (`shape_arc.cpp:204`, erratum E30); the truncation is
  /// unrepresentable here because the parameter is an `i32`.
  pub fn from_start_end_angle(
    start: Vec2,
    end: Vec2,
    angle: Degrees,
    width: i32,
  ) -> Self {
    let center =
      truncate_f64_pair(calc_arc_center_from_angle(start, end, angle));

    Self {
      start,
      mid: rotate_point(start, center, -angle / 2.0),
      end,
      width,
    }
  }

  /// An arc from its two endpoints and a centre, in one of the two
  /// handednesses.
  ///
  /// Port of `ConstructFromStartEndCenter`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:216`, which takes both radial
  /// angles, normalises them, subtracts, forces the difference into
  /// `[0, 360)` or `(-360, 0]` according to `clockwise`, and rotates the
  /// start point about the given centre by half of it.
  ///
  /// **Lossy, and the centre is not stored.** Only the rounded mid point
  /// survives the centre, so [`ShapeArc::center`] recomputes one from the
  /// three points and it generally differs from `center`. This is the
  /// single most surprising property of KiCad's type and the reason its
  /// `amendArc` and `Slice` arc re-cut are lossy (note 09 sections 1.2 and
  /// 2.2). The width is an `i32` here for the reason given on
  /// [`ShapeArc::from_start_end_angle`].
  pub fn from_start_end_center(
    start: Vec2,
    end: Vec2,
    center: Vec2,
    clockwise: bool,
    width: i32,
  ) -> Self {
    let start_line = start.widening_sub(center);
    let end_line = end.widening_sub(center);
    let start_angle =
      Degrees::from_vector(start_line.x as f64, start_line.y as f64)
        .normalized();
    let end_angle =
      Degrees::from_vector(end_line.x as f64, end_line.y as f64).normalized();
    let angle = end_angle - start_angle;
    let angle = if clockwise {
      angle.normalized() - Degrees::FULL_TURN
    } else {
      angle.normalized()
    };

    Self {
      start,
      mid: rotate_point(start, center, -angle / 2.0),
      end,
      width,
    }
  }

  /// The first endpoint.
  ///
  /// Port of `GetP0`, `libs/kimath/include/geometry/shape_arc.h:110`,
  /// which is also what `GetStart` returns (`:203`).
  pub const fn start(self) -> Vec2 {
    self.start
  }

  /// The point between the endpoints that fixes the circle and the
  /// handedness.
  ///
  /// Port of `GetArcMid`,
  /// `libs/kimath/include/geometry/shape_arc.h:112`.
  pub const fn arc_mid(self) -> Vec2 {
    self.mid
  }

  /// The second endpoint.
  ///
  /// Port of `GetP1`, `libs/kimath/include/geometry/shape_arc.h:111`,
  /// which is also what `GetEnd` returns (`:204`).
  pub const fn end(self) -> Vec2 {
    self.end
  }

  /// The full track width.
  ///
  /// Port of `GetWidth`,
  /// `libs/kimath/include/geometry/shape_arc.h:211`.
  pub const fn width(self) -> i32 {
    self.width
  }

  /// Set the full track width.
  ///
  /// Port of `SetWidth`,
  /// `libs/kimath/include/geometry/shape_arc.h:206`. Width is not an input
  /// to any derived value except the bounding box inflation and the
  /// polyline's external radius, so nothing else changes with it.
  pub const fn set_width(&mut self, width: i32) {
    self.width = width;
  }

  /// The straight segment between the two endpoints.
  ///
  /// Port of `GetChord`,
  /// `libs/kimath/include/geometry/shape_arc.h:243`. Exact.
  pub const fn chord(self) -> Seg {
    Seg::new(self.start, self.end)
  }

  /// Whether the arc runs counterclockwise in mathematical coordinates.
  ///
  /// Port of `IsCCW`,
  /// `libs/kimath/include/geometry/shape_arc.h:310`. One `i64` cross
  /// product about the mid point, and the only handedness answer in the
  /// type that does not go through an `f64` (note 09 section 1.1).
  pub fn is_ccw(self) -> bool {
    let from_mid_to_end = self.end.widening_sub(self.mid);
    let from_mid_to_start = self.start.widening_sub(self.mid);

    from_mid_to_end.cross(from_mid_to_start) > 0
  }

  /// Whether the three points are close enough to a straight run that the
  /// arc has no usable circle.
  ///
  /// Port of `IsEffectiveLine`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:248`: the two chords are
  /// approximately collinear to within one nanometre and point the same
  /// way. This is the guard that sends five of the six arc collision cases
  /// down a segment path and that makes [`ShapeArc::central_angle`] report
  /// a zero sweep instead of a fabricated one.
  pub fn is_effective_line(self) -> bool {
    let first = Seg::new(self.start, self.mid);
    let second = Seg::new(self.mid, self.end);

    first.approx_collinear(&second, Seg::APPROX_DISTANCE_THRESHOLD)
      && (first.b - first.a).dot(second.b - second.a) > 0
  }

  /// The centre of the circle through the three points, with its
  /// provenance.
  ///
  /// Port of the `update_values` line that fills `m_center`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:413`. See [`calc_arc_center`]
  /// for the rounding, which is not what a reader expects.
  pub fn arc_center(self) -> ArcCenter {
    calc_arc_center(self.start, self.mid, self.end)
  }

  /// The centre of the circle through the three points.
  ///
  /// Port of `GetCenter`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:952`, which returns the
  /// cached value. Use [`ShapeArc::arc_center`] when the degeneracy
  /// matters.
  pub fn center(self) -> Vec2 {
    self.arc_center().point()
  }

  /// The distance from the centre to the start point.
  ///
  /// Port of the `update_values` line that fills `m_radius`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:414`, read back by
  /// `GetRadius` (`:1007`). There is no integer radius accessor in KiCad
  /// either, so every caller that wants one rounds or truncates itself
  /// (note 09 section 1.4).
  pub fn radius(self) -> f64 {
    self.radius_from(self.center())
  }

  /// The angle from the centre to the start point, in `[0, 360)`.
  ///
  /// Port of `GetStartAngle`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:936`.
  pub fn start_angle(self) -> Degrees {
    self.start_angle_from(self.center())
  }

  /// The angle from the centre to the end point, in `[0, 360)`.
  ///
  /// Port of `GetEndAngle`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:944`.
  pub fn end_angle(self) -> Degrees {
    Self::angle_about(self.end, self.center()).normalized()
  }

  /// The signed sweep, negative when the arc runs clockwise.
  ///
  /// Port of `GetCentralAngle`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:971`, with its three cases:
  /// coincident endpoints are a full turn (`:976`), a straight run sweeps
  /// nothing (`:982`), and otherwise the angular difference about the
  /// centre is pushed into the sign [`ShapeArc::is_ccw`] reports. The
  /// straight run case exists because a straight arc has no circumcircle
  /// and any angle measured about the stand in centre is fabricated;
  /// KiCad's comment at `:979` says so.
  pub fn central_angle(self) -> Degrees {
    self.central_angle_from(self.center())
  }

  /// The length along the curve.
  ///
  /// Port of `GetLength`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:958`. A zero sweep returns
  /// the chord length instead of zero, because a straight arc still has a
  /// run (`:964`).
  pub fn length(self) -> f64 {
    let center = self.center();
    let included_angle = self.central_angle_from(center);

    if included_angle == Degrees::ZERO {
      return f64::from(self.chord().length());
    }

    (self.radius_from(center) * included_angle.as_radians()).abs()
  }

  /// The bounding box, grown by half the width and then by a clearance.
  ///
  /// Port of `BBox`, `libs/kimath/src/geometry/shape_arc.cpp:462`, over
  /// the box `update_values` computes (`:416` to `:458`): the three points
  /// plus every axis quadrant point the sweep crosses. The quadrant walk
  /// is skipped for a radius at or above `INT_MAX / 2`, where the arc is
  /// so flat that the centre cannot be trusted and the three points are
  /// the right answer anyway (`:435`).
  ///
  /// Note the `+ 1` at `:467`: a non zero width inflates by
  /// `kiround(width / 2) + 1`, unconditionally.
  ///
  /// Deviation: the quadrant points are accumulated in `i64`, where KiCad
  /// adds the radius to the centre in `int` and can wrap.
  pub fn bbox(self, clearance: i32) -> Box2 {
    let center = self.center();
    let radius = self.radius_from(center);
    let mut box2 = Box2::from_vec2(self.start)
      .merge_point(Vec2L::from(self.mid))
      .merge_point(Vec2L::from(self.end));

    let start_angle = self.start_angle_from(center);
    let end_angle = start_angle + self.central_angle_from(center);
    // KiCad counts quadrants clockwise, so the two angles are ordered
    // first (`shape_arc.cpp:426`).
    let (start_angle, end_angle) = if start_angle > end_angle {
      (end_angle, start_angle)
    } else {
      (start_angle, end_angle)
    };

    let quadrant_start = (start_angle.as_degrees() / 90.0).ceil() as i32;
    let quadrant_end = (end_angle.as_degrees() / 90.0).floor() as i32;

    if radius < f64::from(i32::MAX) / 2.0 {
      let radius = i64::from(kiround(radius));
      let center_x = i64::from(center.x);
      let center_y = i64::from(center.y);

      for quadrant in quadrant_start..=quadrant_end {
        // C++ and Rust both truncate the remainder towards zero, so the
        // negative arms of KiCad's switch (`:445` to `:448`) are the
        // negative remainders here.
        let quadrant_point = match quadrant % 4 {
          0 => Vec2L::new(center_x + radius, center_y),
          1 | -3 => Vec2L::new(center_x, center_y + radius),
          2 | -2 => Vec2L::new(center_x - radius, center_y),
          _ => Vec2L::new(center_x, center_y - radius),
        };

        box2 = box2.merge_point(quadrant_point);
      }
    }

    if self.width != 0 {
      box2 =
        box2.inflate_by(i64::from(kiround(f64::from(self.width) / 2.0)) + 1);
    }

    if clearance != 0 {
      box2 = box2.inflate_by(i64::from(clearance));
    }

    box2
  }

  /// Whether a point's radial angle lies inside the sweep.
  ///
  /// Port of `sliceContainsPoint`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:1139`. It walks the point's
  /// angle by full turns until it is on the right side of the start angle
  /// and then compares against the end angle. It works entirely in `f64`
  /// and has no tolerance of its own, so a point exactly on an endpoint
  /// radius can fall either way; the callers that care snap to the
  /// endpoints first, as [`ShapeArc::nearest_point`] does.
  pub fn slice_contains_point(self, point: Vec2) -> bool {
    self.slice_contains_point_from(point, self.center())
  }

  /// The point on the arc nearest a given point.
  ///
  /// Port of `NearestPoint`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:476`: the nearest point on
  /// the full circle, snapped to an endpoint when within a squared
  /// distance of 8 (about 2.8 nanometres), then the sweep test, then the
  /// nearer endpoint. The arc's width is not considered, as KiCad's is
  /// not.
  ///
  /// Deviation: KiCad builds a `CIRCLE`, whose radius is an `int`, from a
  /// `double` radius that can exceed `INT_MAX` on a nearly flat arc; the
  /// conversion is undefined there. Here it saturates.
  pub fn nearest_point(self, point: Vec2) -> Vec2 {
    /// KiCad's `s_epsilon`, a squared distance
    /// (`shape_arc.cpp:478`).
    const SNAP_SQUARED_DISTANCE: i64 = 8;

    let center = self.center();
    let nearest = circle_nearest_point(
      center,
      truncate_f64_to_i32(self.radius_from(center)),
      point,
    );

    if nearest.squared_distance(self.start) <= SNAP_SQUARED_DISTANCE {
      return self.start;
    }

    if nearest.squared_distance(self.end) <= SNAP_SQUARED_DISTANCE {
      return self.end;
    }

    if self.slice_contains_point_from(nearest, center) {
      return nearest;
    }

    if point.squared_distance(self.start) <= point.squared_distance(self.end) {
      self.start
    } else {
      self.end
    }
  }

  /// A polyline approximating the arc to within `max_error` nanometres.
  ///
  /// Port of `ConvertToPolyline`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:1013`. The shape of the
  /// approximation is worth stating, because it is not the obvious one:
  /// the segment count is doubled and the loop then takes only the odd
  /// halves (`:1053`, `:1057`), which makes the first and last sub
  /// segments half length. That is what keeps the two exact endpoints on
  /// the arc while the interior points sit on a radius inflated by half
  /// the approximation error (`:1052`), so the error band straddles the
  /// true curve instead of falling entirely outside it.
  ///
  /// Three inputs collapse it to the chord: an external radius below half
  /// the error, a zero sweep, and a mid point closer to the chord than
  /// half the error (`:1032`). The result is then a two point chain, which
  /// `SHAPE_LINE_CHAIN::Append( SHAPE_ARC )` refuses to tag as an arc
  /// (`shape_line_chain.cpp:1622`).
  ///
  /// KiCad's `aActualError` out parameter is not ported; nothing in the
  /// router reads it.
  ///
  /// Deviation: the three `double` to `int` narrowings KiCad performs on
  /// the external radius and the implied full turn segment count saturate
  /// here, where C++ leaves them undefined out of range. Only a nearly
  /// flat arc reaches that, and it takes the chord branch.
  pub fn convert_to_polyline(self, max_error: i32) -> LineChain {
    let center = self.center();
    let mut radius = self.radius_from(center);
    let start_angle = self.start_angle_from(center);
    let central_angle = self.central_angle_from(center);
    let start_to_end = self.chord();
    let half_max_error = f64::max(1.0, f64::from(max_error) / 2.0);
    // The external radius, not the radius: for a small arc with a wide
    // track the difference matters (`shape_arc.cpp:1027`).
    let external_radius = radius + f64::from(self.width) / 2.0;

    let segment_count;
    let effective_error;

    if external_radius < half_max_error
      || central_angle == Degrees::ZERO
      || f64::from(start_to_end.distance_to_point(self.mid)) < half_max_error
    {
      segment_count = 0;
      effective_error = external_radius;
    } else {
      segment_count = arc_to_segment_count(
        truncate_f64_to_i32(external_radius),
        max_error,
        central_angle,
      );

      let full_turn_segments = truncate_f64_to_i32(
        f64::from(segment_count) * 360.0 / central_angle.as_degrees().abs(),
      );

      effective_error = f64::from(circle_to_end_segment_delta_radius(
        truncate_f64_to_i32(external_radius),
        full_turn_segments,
      ));
    }

    radius += effective_error / 2.0;

    let doubled_count = segment_count * 2;
    let mut chain = LineChain::new();

    chain.append(self.start);

    let mut index = 1;

    while index < doubled_count {
      let angle = start_angle
        + central_angle * f64::from(index) / f64::from(doubled_count);
      let x = f64::from(center.x) + radius * angle.cos();
      let y = f64::from(center.y) + radius * angle.sin();

      chain.append(Vec2::new(kiround(x), kiround(y)));
      index += 2;
    }

    chain.append(self.end);
    chain
  }

  /// Translate all three points.
  ///
  /// Port of `Move`, `libs/kimath/src/geometry/shape_arc.cpp:1079`. Exact.
  pub fn move_by(&mut self, delta: Vec2) {
    self.start += delta;
    self.mid += delta;
    self.end += delta;
  }

  /// Reflect all three points in an axis.
  ///
  /// Port of `Mirror( const SEG& )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:1117`. The three points move
  /// together, so the representation stays consistent and the handedness
  /// flips without any reordering. KiCad's other overload, which mirrors
  /// about a horizontal or vertical line through a reference point
  /// (`:1098`), has no router caller and is not ported, matching
  /// [`LineChain::mirror`].
  pub fn mirror(&mut self, axis: &Seg) {
    self.start = axis.reflect_point(self.start);
    self.mid = axis.reflect_point(self.mid);
    self.end = axis.reflect_point(self.end);
  }

  /// Swap the two endpoints in place, leaving the mid point where it is.
  ///
  /// Port of `Reverse`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:1127`. The mid point stays
  /// the mid point, so the curve is unchanged and only the direction of
  /// travel flips.
  ///
  /// Erratum E4 is fixed here by construction. In KiCad this method leaves
  /// the cached centre, radius and bounding box untouched while
  /// `Reversed` (`:1133`) rebuilds the arc from the permuted points and
  /// re-runs `CalcArcCenter`, which through the round number snapping can
  /// land on a different centre; the two are therefore not equivalent, and
  /// `SHAPE_LINE_CHAIN::Reverse` and `NODE::AssembleLine` use one each.
  /// Nothing is cached here, so both give the same arc.
  pub const fn reverse(&mut self) {
    let start = self.start;

    self.start = self.end;
    self.end = start;
  }

  /// A copy with the two endpoints swapped.
  ///
  /// Port of `Reversed`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:1133`. See
  /// [`ShapeArc::reverse`] for why the two agree here and not in KiCad.
  pub const fn reversed(self) -> ShapeArc {
    Self::new(self.end, self.mid, self.start, self.width)
  }

  /// Whether a point comes within a clearance of the arc, and by how
  /// much.
  ///
  /// Port of `Collide( const VECTOR2I&, int, int*, VECTOR2I* )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:844`. The arc's own half
  /// width is part of the test, so a zero width arc measures to its
  /// centre line and a track width arc measures to its edge (`:847`,
  /// `:927`).
  ///
  /// Four stages: a bounding box reject (`:851`), a nearly flat branch
  /// that measures against the two chords instead of a circle (`:859`),
  /// the nearest point on the full circle (`:882`), and an angular test
  /// that falls back to the nearer endpoint when the point lies outside
  /// the sweep (`:896`). The reported location is the point on the arc,
  /// except in the nearly flat branch where it is the point on the nearer
  /// chord.
  ///
  /// Two pieces of arithmetic here are not the obvious ones and are kept
  /// as KiCad has them:
  ///
  /// - the radius is `|center - start|` through the three case
  ///   `EuclideanNorm` (`:858`), not the square root of the squared norm
  ///   that [`ShapeArc::radius`] takes (`:414`). The two disagree in the
  ///   last bit on some inputs, so this routine has its own radius;
  /// - a distance that rounds to zero is recomputed as
  ///   `kiround(radius - sqrt(|point - center|^2))` (`:889`), because
  ///   measuring from the already rounded nearest point would have
  ///   truncated the gap away before the subtraction. KiCad's comment at
  ///   `:887` says exactly that.
  ///
  /// Deviations, all widenings: the clearance and half width sum, the
  /// point to centre difference and the two endpoint distances are taken
  /// in `i64` where KiCad computes them in `int` and can wrap.
  pub fn collide_point(
    self,
    point: Vec2,
    clearance: i32,
  ) -> Option<ShapeCollision> {
    let minimum_distance = i64::from(clearance) + i64::from(self.width) / 2;

    if !self
      .bbox(saturate_i32(minimum_distance))
      .contains_point(Vec2L::from(point))
    {
      return None;
    }

    let center = self.center();
    let radius = self.collide_radius(center);

    // `CIRCLE` stores an `int` radius, so a nearly straight arc is
    // measured against its two chords instead (`shape_arc.cpp:861`).
    if radius >= f64::from(i32::MAX) / 2.0 {
      let first = Seg::new(self.start, self.mid);
      let second = Seg::new(self.mid, self.end);
      let first_distance = first.distance_to_point(point);
      let second_distance = second.distance_to_point(point);
      let distance = first_distance.min(second_distance);

      if i64::from(distance) > minimum_distance {
        return None;
      }

      return Some(ShapeCollision {
        actual: self.gap_from_distance(i64::from(distance)),
        location: if first_distance <= second_distance {
          first.nearest_point_to_point(point)
        } else {
          second.nearest_point_to_point(point)
        },
      });
    }

    let circle_radius = truncate_f64_to_i32(radius);
    let mut nearest = circle_nearest_point_f64(center, circle_radius, point);
    let mut distance = i64::from(kiround(euclidean_norm_f64(
      f64::from(point.x) - nearest.0,
      f64::from(point.y) - nearest.1,
    )));
    let offset = point.widening_sub(center);
    let angle_to_point = Degrees::from_vector(offset.x as f64, offset.y as f64);

    if distance == 0 {
      distance = i64::from(kiround(
        radius - (offset.squared_euclidean_norm() as f64).sqrt(),
      ));
      nearest = rotate_point_f64(
        (
          f64::from(center.x) + f64::from(circle_radius),
          f64::from(center.y),
        ),
        to_f64(center),
        -angle_to_point,
      );
    }

    // A full turn has no outside, so the angular test is skipped for one
    // (`shape_arc.cpp:895`).
    if self.start != self.end {
      let counterclockwise = self.central_angle_from(center) > Degrees::ZERO;
      let start_angle = self.start_angle_from(center);
      let rotated_point_angle =
        (angle_to_point.normalized() - start_angle).normalized();
      let rotated_end_angle =
        (self.end_angle_from(center) - start_angle).normalized();

      if (counterclockwise && rotated_point_angle > rotated_end_angle)
        || (!counterclockwise && rotated_point_angle < rotated_end_angle)
      {
        let to_start =
          saturate_i32(point.widening_sub(self.start).euclidean_norm());
        let to_end =
          saturate_i32(point.widening_sub(self.end).euclidean_norm());

        if to_start < to_end {
          distance = i64::from(to_start);
          nearest = to_f64(self.start);
        } else {
          distance = i64::from(to_end);
          nearest = to_f64(self.end);
        }
      }
    }

    if distance > minimum_distance {
      return None;
    }

    Some(ShapeCollision {
      actual: self.gap_from_distance(distance),
      location: truncate_f64_pair(nearest),
    })
  }

  /// Whether a segment comes within a clearance of the arc, and by how
  /// much.
  ///
  /// Port of `Collide( const SEG&, int, int*, VECTOR2I* )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:257`. This is a candidate
  /// point method and not a closed form: it builds a list of points that
  /// could carry the collision and runs each through
  /// [`ShapeArc::collide_point`]. The candidates are the circle and
  /// segment intersections, the segment's nearest point to the centre,
  /// its nearest points to the two arc endpoints, and the two segment
  /// endpoints (`:318` to `:324`).
  ///
  /// Two branches come first. A nearly straight arc, radius at or above
  /// `INT_MAX / 2`, builds nine candidates from the two chords and the
  /// segment instead (`:264` to `:292`). An arc that sweeps more than a
  /// half turn and whose chord is shorter than the clearance is treated
  /// as a whole circle, with an early miss when both segment endpoints
  /// sit strictly inside `radius - clearance` (`:298` to `:310`).
  ///
  /// # Erratum E2
  ///
  /// The reported gap and location are those of the **last candidate that
  /// collided**, not of the nearest one. KiCad's loop overwrites its two
  /// out parameters on every colliding candidate and only stops early on
  /// an exact touch (`:328` to `:335`), so a caller that asks an arc how
  /// far a segment is gets an arbitrary one of the candidate distances.
  /// That is reproduced here, deliberately; the decision is in
  /// `doc/log/2026-09-12.md` and
  /// `arc_seg_collide_reports_the_last_candidate_erratum_e2` pins it.
  ///
  /// Note that KiCad's loop also returns at the **first** colliding
  /// candidate when the caller passed no `aActual` pointer. That cannot
  /// change the answer, only which candidate's numbers are discarded, so
  /// this form always runs the full loop.
  pub fn collide_seg(
    self,
    seg: &Seg,
    clearance: i32,
  ) -> Option<ShapeCollision> {
    let center = self.center();
    let radius = self.collide_radius(center);

    if radius >= f64::from(i32::MAX) / 2.0 {
      let first = Seg::new(self.start, self.mid);
      let second = Seg::new(self.mid, self.end);

      return self.collide_candidates(
        &[
          seg.nearest_point_to_point(self.start),
          seg.nearest_point_to_point(self.mid),
          seg.nearest_point_to_point(self.end),
          first.nearest_point_to_point(seg.a),
          first.nearest_point_to_point(seg.b),
          second.nearest_point_to_point(seg.a),
          second.nearest_point_to_point(seg.b),
          seg.a,
          seg.b,
        ],
        clearance,
      );
    }

    let circle_radius = truncate_f64_to_i32(radius);

    // An arc with less room left inside it than the clearance collides
    // like the whole circle (`shape_arc.cpp:296`).
    if self.central_angle_from(center).as_degrees() > 180.0
      && self.start.widening_sub(self.end).squared_euclidean_norm()
        < square(i64::from(clearance))
    {
      let to_a = seg.a.widening_sub(center).squared_euclidean_norm();
      let to_b = seg.b.widening_sub(center).squared_euclidean_norm();
      // `SEG::Square` takes an `int`, so the difference truncates
      // (`shape_arc.cpp:303`).
      let inner_radius_squared = square(i64::from(truncate_f64_to_i32(
        radius - f64::from(clearance),
      )));

      if to_a < inner_radius_squared && to_b < inner_radius_squared {
        return None;
      }

      return circle_seg_collision(center, circle_radius, seg, clearance);
    }

    let mut candidates = circle_intersect_seg(center, circle_radius, seg);

    candidates.push(seg.nearest_point_to_point(center));
    candidates.push(seg.nearest_point_to_point(self.start));
    candidates.push(seg.nearest_point_to_point(self.end));
    candidates.push(seg.a);
    candidates.push(seg.b);

    self.collide_candidates(&candidates, clearance)
  }

  /// The nearest points between the arc and a circle.
  ///
  /// Port of `NearestPoints( const SHAPE_CIRCLE&, VECTOR2I&, VECTOR2I&, int64_t& )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:499`. The point on the arc
  /// is pulled in by half the arc's width towards the circle and the
  /// distance is zeroed when it falls inside that half width (`:543` to
  /// `:550`), so the result is an edge to edge answer for a track width
  /// arc and a centre line answer for a zero width one.
  ///
  /// The candidates are the two circle intersections that lie inside the
  /// sweep, then the two arc endpoints and the arc circle's nearest point
  /// to the other centre, each kept only when it lies inside the sweep
  /// (`:513` to `:541`). An arc concentric with the circle and of the
  /// same radius answers its own start point at zero distance (`:501`).
  ///
  /// When **no** candidate lies inside the sweep, KiCad reports the pair
  /// its caller left default constructed, which is the origin twice at
  /// zero distance (`shape_collisions.cpp:612`). That is reproduced,
  /// origin and all, because the collision cell built on top of it then
  /// reports a collision at the board origin and a port that quietly
  /// fixed it would diverge. Both arc endpoints lie inside their own
  /// sweep for every arc the router builds, so the branch needs a
  /// degenerate arc to reach.
  pub fn nearest_points_to_circle(
    self,
    center: Vec2,
    radius: i32,
  ) -> NearestPoints {
    let own_center = self.center();
    let own_radius = self.radius_from(own_center);

    if own_center == center && own_radius == f64::from(radius) {
      return NearestPoints {
        on_self: self.start,
        on_other: self.start,
        squared_distance: 0,
      };
    }

    let own_circle_radius = truncate_f64_to_i32(own_radius);

    for point in
      circle_intersect_circle(own_center, own_circle_radius, center, radius)
    {
      if self.slice_contains_point_from(point, own_center) {
        return NearestPoints {
          on_self: point,
          on_other: point,
          squared_distance: 0,
        };
      }
    }

    let mut nearest = NearestPoints {
      on_self: Vec2::new(0, 0),
      on_other: Vec2::new(0, 0),
      squared_distance: i64::MAX,
    };

    for point in [
      self.start,
      self.end,
      circle_nearest_point(own_center, own_circle_radius, center),
    ] {
      if !self.slice_contains_point_from(point, own_center) {
        continue;
      }

      let on_circle = circle_nearest_point(center, radius, point);
      let squared_distance = point.squared_distance(on_circle);

      if squared_distance < nearest.squared_distance {
        nearest = NearestPoints {
          on_self: point,
          on_other: on_circle,
          squared_distance,
        };
      }
    }

    self.adjusted_for_own_width(nearest)
  }

  /// The nearest points between the arc and a segment.
  ///
  /// Port of `NearestPoints( const SEG&, VECTOR2I&, VECTOR2I&, int64_t& )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:556`, with the same half
  /// width treatment as [`ShapeArc::nearest_points_to_circle`] (`:622`
  /// to `:629`).
  ///
  /// Four candidate families, in KiCad's order: the circle and segment
  /// intersections inside the sweep, which answer zero straight away
  /// (`:563`); the segment endpoints whose radial angle lies inside the
  /// sweep, measured to the circle (`:574`); the two arc endpoints,
  /// measured to the segment (`:590`); and the segment's nearest point to
  /// the centre, measured to the circle (`:604`).
  ///
  /// The last family writes the **segment** point into the arc's slot and
  /// the circle point into the segment's slot (`:612`, `:613`), the
  /// reverse of the other three. That is KiCad's and is kept: the half
  /// width adjustment that follows then moves the point on the segment,
  /// not the one on the arc.
  pub fn nearest_points_to_seg(self, seg: &Seg) -> NearestPoints {
    let center = self.center();
    let circle_radius = truncate_f64_to_i32(self.radius_from(center));

    for point in circle_intersect_seg(center, circle_radius, seg) {
      if self.slice_contains_point_from(point, center) {
        return NearestPoints {
          on_self: point,
          on_other: point,
          squared_distance: 0,
        };
      }
    }

    let mut nearest = NearestPoints {
      on_self: Vec2::new(0, 0),
      on_other: Vec2::new(0, 0),
      squared_distance: i64::MAX,
    };

    for point in [seg.a, seg.b] {
      if !self.slice_contains_point_from(point, center) {
        continue;
      }

      let on_circle = circle_nearest_point(center, circle_radius, point);
      let squared_distance = point.squared_distance(on_circle);

      if squared_distance < nearest.squared_distance {
        nearest = NearestPoints {
          on_self: on_circle,
          on_other: point,
          squared_distance,
        };
      }
    }

    for point in [self.start, self.end] {
      let on_seg = seg.nearest_point_to_point(point);
      let squared_distance = point.squared_distance(on_seg);

      if squared_distance < nearest.squared_distance {
        nearest = NearestPoints {
          on_self: point,
          on_other: on_seg,
          squared_distance,
        };
      }
    }

    let on_seg = seg.nearest_point_to_point(center);

    if self.slice_contains_point_from(on_seg, center) {
      let on_circle = circle_nearest_point(center, circle_radius, on_seg);
      let squared_distance = on_seg.squared_distance(on_circle);

      if squared_distance < nearest.squared_distance {
        nearest = NearestPoints {
          on_self: on_seg,
          on_other: on_circle,
          squared_distance,
        };
      }
    }

    self.adjusted_for_own_width(nearest)
  }

  /// The nearest points between the arc and an axis aligned rectangle.
  ///
  /// Port of `NearestPoints( const SHAPE_RECT&, VECTOR2I&, VECTOR2I&, int64_t& )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:635`, which hands the
  /// rectangle's outline to the generic shape dispatcher
  /// (`shape_nearest_points.cpp:658`) and swaps the two output points
  /// back. The dispatcher walks the outline's segments and takes the
  /// minimum of [`ShapeArc::nearest_points_to_seg`] over them, which is
  /// what this does directly.
  ///
  /// # Erratum E3
  ///
  /// The final line recomputes the squared distance from the two points
  /// (`:644`) and so **throws away the zeroing** that the segment
  /// overload applied when the arc's half width already covered the gap.
  /// The point on the arc has still been pulled in by the half width, so
  /// the result is an edge to edge distance that never clamps to zero:
  /// once the arc's edge crosses the rectangle the distance starts
  /// growing again instead of staying at zero. A wide arc track therefore
  /// under-reports its overlap with a rectangle. Reproduced deliberately,
  /// see `doc/log/2026-09-12.md` and
  /// `arc_nearest_points_to_rect_drops_the_width_zeroing_erratum_e3`.
  ///
  /// Note 09 erratum E3 describes this as the overload ignoring the
  /// width outright. It does apply the half width to the point; what it
  /// loses is only the clamp.
  pub fn nearest_points_to_rect(
    self,
    origin: Vec2,
    size: Vec2,
  ) -> NearestPoints {
    let outline = rect_outline(origin, size);
    let mut on_self = Vec2::new(0, 0);
    let mut on_other = Vec2::new(0, 0);
    let mut best = i64::MAX;

    for index in 0..outline.segment_count() {
      let candidate = self.nearest_points_to_seg(&outline.segment(index));

      if candidate.squared_distance < best {
        best = candidate.squared_distance;
        on_self = candidate.on_self;
        on_other = candidate.on_other;
      }
    }

    NearestPoints {
      on_self,
      on_other,
      squared_distance: on_self.squared_distance(on_other),
    }
  }

  /// The nearest points between two arcs.
  ///
  /// Port of `NearestPoints( const SHAPE_ARC&, VECTOR2I&, VECTOR2I&, int64_t& )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:649`. Both arcs' half widths
  /// are applied at the end, the second one measured from the already
  /// moved first point (`:652` to `:663`).
  ///
  /// The routine works down a ladder and stops at the first rung that
  /// answers:
  ///
  /// 1. the four endpoint pairs, which return immediately on an exact
  ///    touch and **without** the width adjustment (`:686`);
  /// 2. each endpoint of this arc that lies inside the other's sweep,
  ///    against the other's circle (`:692`);
  /// 3. each endpoint of the other arc inside this sweep, against this
  ///    circle (`:710`);
  /// 4. the circle intersections inside both sweeps (`:735`);
  /// 5. the closest pair of points on the two full circles, with a
  ///    separate branch for one circle contained in the other (`:766` to
  ///    `:787`), then the endpoints of each arc against whichever of that
  ///    pair lies inside its own sweep (`:806`, `:820`).
  ///
  /// Rungs 2 and 3 only stop the ladder when the two centres are within
  /// `min(radius) / 1000` of each other or the distance is exactly zero
  /// (`:702`, `:720`). That epsilon exists because the two centres come
  /// out of [`calc_arc_center`] and are not exact; KiCad's comment at
  /// `:678` says so. Concentric arcs never reach rungs 4 and 5, whose
  /// geometry needs two distinct circles (`:731`).
  pub fn nearest_points_to_arc(self, other: ShapeArc) -> NearestPoints {
    let own_center = self.center();
    let other_center = other.center();
    let own_radius = self.radius_from(own_center);
    let other_radius = other.radius_from(other_center);
    let own_circle_radius = truncate_f64_to_i32(own_radius);
    let other_circle_radius = truncate_f64_to_i32(other_radius);
    let center_distance_squared = own_center.squared_distance(other_center);
    let center_epsilon =
      i64::from(kiround(own_radius.min(other_radius) / 1000.0));
    let colocated = center_distance_squared < center_epsilon * center_epsilon;
    let own_ends = [self.start, self.end];
    let other_ends = [other.start, other.end];

    let mut nearest = NearestPoints {
      on_self: Vec2::new(0, 0),
      on_other: Vec2::new(0, 0),
      squared_distance: i64::MAX,
    };

    for own in own_ends {
      for far in other_ends {
        let squared_distance = own.squared_distance(far);

        if squared_distance < nearest.squared_distance {
          nearest = NearestPoints {
            on_self: own,
            on_other: far,
            squared_distance,
          };

          // An exact touch returns before either width is applied
          // (`shape_arc.cpp:686`).
          if nearest.squared_distance == 0 {
            return nearest;
          }
        }
      }
    }

    for own in own_ends {
      if !other.slice_contains_point_from(own, other_center) {
        continue;
      }

      let on_other =
        circle_nearest_point(other_center, other_circle_radius, own);

      nearest = NearestPoints {
        on_self: own,
        on_other,
        squared_distance: own.squared_distance(on_other),
      };

      if colocated || nearest.squared_distance == 0 {
        if nearest.squared_distance != 0 {
          nearest = self.adjusted_for_both_widths(nearest, other.width);
        }

        return nearest;
      }
    }

    for far in other_ends {
      if !self.slice_contains_point_from(far, own_center) {
        continue;
      }

      let on_self = circle_nearest_point(own_center, own_circle_radius, far);

      nearest = NearestPoints {
        on_self,
        on_other: far,
        squared_distance: on_self.squared_distance(far),
      };

      if colocated || nearest.squared_distance == 0 {
        if nearest.squared_distance != 0 {
          nearest = self.adjusted_for_both_widths(nearest, other.width);
        }

        return nearest;
      }
    }

    // The rest needs two distinct circles (`shape_arc.cpp:731`).
    if colocated {
      return nearest;
    }

    for point in circle_intersect_circle(
      own_center,
      own_circle_radius,
      other_center,
      other_circle_radius,
    ) {
      if self.slice_contains_point_from(point, own_center)
        && other.slice_contains_point_from(point, other_center)
      {
        return NearestPoints {
          on_self: point,
          on_other: point,
          squared_distance: 0,
        };
      }
    }

    // For two separate circles the closest pair faces across the line of
    // centres. For one circle inside the other the pair is on the same
    // side, so the outer one takes its nearest point to the inner centre
    // and the inner one its furthest point from the outer centre
    // (`shape_arc.cpp:760` to `:787`).
    let contained = (center_distance_squared as f64)
      < (own_radius - other_radius) * (own_radius - other_radius);
    let (own_point, other_point) = if contained && own_radius > other_radius {
      (
        circle_nearest_point(own_center, own_circle_radius, other_center),
        circle_furthest_point(other_center, other_circle_radius, own_center),
      )
    } else if contained {
      (
        circle_furthest_point(own_center, own_circle_radius, other_center),
        circle_nearest_point(other_center, other_circle_radius, own_center),
      )
    } else {
      (
        circle_nearest_point(own_center, own_circle_radius, other_center),
        circle_nearest_point(other_center, other_circle_radius, own_center),
      )
    };

    let own_in_slice = self.slice_contains_point_from(own_point, own_center);
    let other_in_slice =
      other.slice_contains_point_from(other_point, other_center);

    if own_in_slice && other_in_slice {
      let squared_distance = own_point.squared_distance(other_point);

      if squared_distance < nearest.squared_distance {
        nearest = NearestPoints {
          on_self: own_point,
          on_other: other_point,
          squared_distance,
        };
      }

      return self.adjusted_for_both_widths(nearest, other.width);
    }

    if other_in_slice {
      for own in own_ends {
        let squared_distance = own.squared_distance(other_point);

        if squared_distance < nearest.squared_distance {
          nearest = NearestPoints {
            on_self: own,
            on_other: other_point,
            squared_distance,
          };
        }
      }
    }

    if own_in_slice {
      for far in other_ends {
        let squared_distance = far.squared_distance(own_point);

        if squared_distance < nearest.squared_distance {
          nearest = NearestPoints {
            on_self: own_point,
            on_other: far,
            squared_distance,
          };
        }
      }
    }

    self.adjusted_for_both_widths(nearest, other.width)
  }

  /// The radius [`ShapeArc::collide_point`] and [`ShapeArc::collide_seg`]
  /// measure with.
  ///
  /// `VECTOR2D( center - m_start ).EuclideanNorm()`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:259` and `:858`. The three
  /// case norm of [`euclidean_norm_f64`] is not the same expression as
  /// `sqrt( x * x + y * y )`, which is what `update_values` uses for
  /// `m_radius` (`:414`), so the two can disagree in the last bit and the
  /// collision routines have to use this one.
  fn collide_radius(self, center: Vec2) -> f64 {
    euclidean_norm_f64(
      f64::from(center.x) - f64::from(self.start.x),
      f64::from(center.y) - f64::from(self.start.y),
    )
  }

  /// [`ShapeArc::end_angle`] against a centre the caller already has.
  fn end_angle_from(self, center: Vec2) -> Degrees {
    Self::angle_about(self.end, center).normalized()
  }

  /// `std::max( 0, dist - m_width / 2 )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:874` and `:927`.
  fn gap_from_distance(self, distance: i64) -> i32 {
    saturate_i32((distance - i64::from(self.width) / 2).max(0))
  }

  /// The candidate loop of `Collide( const SEG& )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:328` to `:335`.
  ///
  /// It keeps the **last** candidate that collided, which is erratum E2;
  /// see [`ShapeArc::collide_seg`].
  fn collide_candidates(
    self,
    candidates: &[Vec2],
    clearance: i32,
  ) -> Option<ShapeCollision> {
    let mut collision = None;

    for candidate in candidates {
      let Some(hit) = self.collide_point(*candidate, clearance) else {
        continue;
      };

      collision = Some(hit);

      if hit.actual == 0 {
        break;
      }
    }

    collision
  }

  /// Pull the point on the arc in by half the arc's width and clamp the
  /// distance.
  ///
  /// The tail shared by `NearestPoints( const SHAPE_CIRCLE& )` and
  /// `NearestPoints( const SEG& )`,
  /// `libs/kimath/src/geometry/shape_arc.cpp:543` and `:622`.
  ///
  /// Deviation: the difference of the two points is taken in `i64`, where
  /// KiCad subtracts two `VECTOR2I` and can wrap.
  fn adjusted_for_own_width(self, nearest: NearestPoints) -> NearestPoints {
    let half_width = self.width / 2;
    let direction = nearest
      .on_other
      .widening_sub(nearest.on_self)
      .saturating_to_vec2()
      .resize(half_width);
    let on_self = nearest.on_self + direction;

    NearestPoints {
      on_self,
      on_other: nearest.on_other,
      squared_distance: if nearest.squared_distance
        < square(i64::from(half_width))
      {
        0
      } else {
        on_self.squared_distance(nearest.on_other)
      },
    }
  }

  /// Pull both points in by their own arc's half width and clamp the
  /// distance.
  ///
  /// KiCad's `adjustForArcWidths` lambda,
  /// `libs/kimath/src/geometry/shape_arc.cpp:652` to `:663`. The second
  /// direction is measured from the **already moved** first point, which
  /// is why this cannot be two calls to
  /// [`ShapeArc::adjusted_for_own_width`].
  fn adjusted_for_both_widths(
    self,
    nearest: NearestPoints,
    other_width: i32,
  ) -> NearestPoints {
    let own_half_width = self.width / 2;
    let other_half_width = other_width / 2;
    let on_self = nearest.on_self
      + nearest
        .on_other
        .widening_sub(nearest.on_self)
        .saturating_to_vec2()
        .resize(own_half_width);
    let on_other = nearest.on_other
      + on_self
        .widening_sub(nearest.on_other)
        .saturating_to_vec2()
        .resize(other_half_width);

    NearestPoints {
      on_self,
      on_other,
      squared_distance: if nearest.squared_distance
        < square(i64::from(own_half_width) + i64::from(other_half_width))
      {
        0
      } else {
        on_self.squared_distance(on_other)
      },
    }
  }

  /// [`ShapeArc::radius`] against a centre the caller already has.
  fn radius_from(self, center: Vec2) -> f64 {
    let x = f64::from(self.start.x) - f64::from(center.x);
    let y = f64::from(self.start.y) - f64::from(center.y);

    (x * x + y * y).sqrt()
  }

  /// [`ShapeArc::start_angle`] against a centre the caller already has.
  fn start_angle_from(self, center: Vec2) -> Degrees {
    Self::angle_about(self.start, center).normalized()
  }

  /// [`ShapeArc::central_angle`] against a centre the caller already has.
  fn central_angle_from(self, center: Vec2) -> Degrees {
    if self.start == self.end {
      return Degrees::FULL_TURN;
    }

    if self.is_effective_line() {
      return Degrees::ZERO;
    }

    let angle = Self::angle_about(self.end, center)
      - Self::angle_about(self.start, center);

    // The two endpoints alone leave two arcs on the same circle, so the
    // sign has to come from the mid point (`shape_arc.cpp:992`).
    if self.is_ccw() {
      if angle < Degrees::ZERO {
        return angle + Degrees::FULL_TURN;
      }
    } else if angle > Degrees::ZERO {
      return angle - Degrees::FULL_TURN;
    }

    angle
  }

  /// [`ShapeArc::slice_contains_point`] against a centre the caller
  /// already has.
  fn slice_contains_point_from(self, point: Vec2, center: Vec2) -> bool {
    let start_angle = self.start_angle_from(center);
    let central_angle = self.central_angle_from(center);
    let end_angle = start_angle + central_angle;
    let mut phi = Self::angle_about(point, center).normalized();

    if central_angle >= Degrees::ZERO {
      while phi < start_angle {
        phi = phi + Degrees::FULL_TURN;
      }

      phi >= start_angle && phi <= end_angle
    } else {
      while phi > start_angle {
        phi = phi - Degrees::FULL_TURN;
      }

      phi <= start_angle && phi >= end_angle
    }
  }

  /// The radial angle of a point about a centre, unnormalised.
  ///
  /// KiCad widens the difference to `VECTOR2L` before it builds the
  /// `EDA_ANGLE` (`shape_arc.cpp:938`, `:989`), which is what keeps a
  /// point at one end of the coordinate range and a centre at the other
  /// from wrapping.
  fn angle_about(point: Vec2, center: Vec2) -> Degrees {
    let offset = point.widening_sub(center);

    Degrees::from_vector(offset.x as f64, offset.y as f64)
  }
}

/// The centre of the circle through three integer points.
///
/// Port of `CalcArcCenter( const VECTOR2I&, const VECTOR2I&, const VECTOR2I& )`,
/// `libs/kimath/src/trigo.cpp:562`, which runs the `f64` routine, clamps
/// each coordinate into `[INT_MIN + 100, INT_MAX - 100]` and rounds.
///
/// The `f64` routine snaps the result to a multiple of 100 nanometres, or
/// failing that 10 nanometres, whenever its own propagated uncertainty
/// covers the round value (`trigo.cpp:529` to `:550`). KiCad's comment at
/// `:534` justifies it: "The last step is to find the nice, round numbers
/// near our baseline estimate and see if they are within our uncertainty
/// range. If they are, then we use this round value as the true value.
/// This is justified because ALL values within the uncertainty range are
/// equally true."
///
/// The consequence for a caller is that **the centre is not a continuous
/// function of the three points**: moving an endpoint by one nanometre can
/// move the centre by up to 50, and the radius, the sweep and the length
/// move with it. It is ported verbatim on purpose, so that the crate's
/// arcs stay bit compatible with KiCad's; erratum E1 and the decision of
/// 2026-09-12 in `doc/log/` have the argument.
pub fn calc_arc_center(start: Vec2, mid: Vec2, end: Vec2) -> ArcCenter {
  /// KiCad's clamp margin, `trigo.cpp:571`.
  const CLAMP_MARGIN: f64 = 100.0;

  let (x, y, degenerate) = calc_arc_center_f64_with_degeneracy(
    to_f64(start),
    to_f64(mid),
    to_f64(end),
  );

  let low = f64::from(i32::MIN) + CLAMP_MARGIN;
  let high = f64::from(i32::MAX) - CLAMP_MARGIN;
  let center =
    Vec2::new(kiround(x.clamp(low, high)), kiround(y.clamp(low, high)));

  if degenerate {
    ArcCenter::Degenerate(center)
  } else {
    ArcCenter::Circumcentre(center)
  }
}

/// The centre of the circle through three floating point points.
///
/// Port of `CalcArcCenter( const VECTOR2D&, const VECTOR2D&, const VECTOR2D& )`,
/// `libs/kimath/src/trigo.cpp:371`. See [`calc_arc_center`] for the round
/// number snapping, which is the part of this routine a reader has to know
/// about. This is the entry point KiCad's own unit tests exercise.
pub fn calc_arc_center_f64(
  start: (f64, f64),
  mid: (f64, f64),
  end: (f64, f64),
) -> (f64, f64) {
  let (x, y, _) = calc_arc_center_f64_with_degeneracy(start, mid, end);

  (x, y)
}

/// The centre of the circle through two points that sweeps a given angle
/// between them.
///
/// Port of `CalcArcCenter( const VECTOR2D&, const VECTOR2D&, const EDA_ANGLE& )`,
/// `libs/kimath/src/trigo.cpp:329`, a different algorithm from the three
/// point one: it orients the sweep positive and at most a half turn,
/// computes the radius from the chord and the half angle, and steps from
/// the start point along the chord and then perpendicular to it. A zero
/// half angle sine has no defined centre and falls back to the chord
/// midpoint (`:352`).
///
/// It has **no round number snapping**, so an arc built through
/// [`ShapeArc::from_start_end_angle`] places its centre by one rounding
/// rule and then, through [`ShapeArc::center`], reports one placed by
/// another (note 09 section 1.3).
///
/// The result is a floating point position because that is what KiCad
/// returns; its one caller truncates it into `i32`
/// (`shape_arc.cpp:205`, through the narrowing conversion at
/// `libs/kimath/include/math/vector2d.h:85`).
pub fn calc_arc_center_from_angle(
  start: Vec2,
  end: Vec2,
  angle: Degrees,
) -> (f64, f64) {
  let mut angle = angle;
  let mut start = to_f64(start);
  let mut end = to_f64(end);

  if angle < Degrees::ZERO {
    std::mem::swap(&mut start, &mut end);
    angle = -angle;
  }

  if angle > Degrees::HALF_TURN {
    std::mem::swap(&mut start, &mut end);
    angle = Degrees::FULL_TURN - angle;
  }

  let chord = euclidean_norm_f64(start.0 - end.0, start.1 - end.1);
  let sin_half_angle = (angle / 2.0).sin();

  if sin_half_angle == 0.0 {
    return ((start.0 + end.0) / 2.0, (start.1 + end.1) / 2.0);
  }

  let radius = (chord / 2.0) / sin_half_angle;
  let distance_squared = radius * radius - chord * chord / 4.0;
  let distance = if distance_squared > 0.0 {
    distance_squared.sqrt()
  } else {
    0.0
  };

  let along = end.0 - start.0;
  let across = end.1 - start.1;
  let half_chord = resize_f64(along, across, chord / 2.0);
  // `RotatePoint( vec2, -ANGLE_90 )`, `trigo.cpp:365`.
  let perpendicular = rotate_point_f64(
    resize_f64(along, across, distance),
    (0.0, 0.0),
    -Degrees::QUARTER_TURN,
  );

  (
    start.0 + half_chord.0 + perpendicular.0,
    start.1 + half_chord.1 + perpendicular.1,
  )
}

/// The number of segments needed to approximate an arc to within an error.
///
/// Port of `GetArcToSegmentCount`,
/// `libs/kimath/src/geometry/geometry_utils.cpp:38`. The per segment angle
/// is capped at a eighth of a turn so that a very small radius still gets
/// a recognisable circle, and the count is floored at two "for algorithmic
/// safety".
pub fn arc_to_segment_count(
  radius: i32,
  error_max: i32,
  arc_angle: Degrees,
) -> i32 {
  /// KiCad's `MIN_SEGCOUNT_FOR_CIRCLE`, `geometry_utils.cpp:36`.
  const MIN_SEGMENT_COUNT_FOR_CIRCLE: f64 = 8.0;

  let radius = radius.max(1);
  let error_max = error_max.max(1);
  let relative_error = f64::from(error_max) / f64::from(radius);
  let arc_increment = 180.0 / PI * (1.0 - relative_error).acos() * 2.0;
  // `std::min` keeps the cap when the arc cosine is not a number, and so
  // does `f64::min`.
  let arc_increment = arc_increment.min(360.0 / MIN_SEGMENT_COUNT_FOR_CIRCLE);
  let segment_count = kiround(arc_angle.as_degrees().abs() / arc_increment);

  segment_count.max(2)
}

/// How far the ends of a chord fall outside the circle its middle is
/// tangent to.
///
/// Port of `CircleToEndSegmentDeltaRadius`,
/// `libs/kimath/src/geometry/geometry_utils.cpp:63`. The segment count is
/// floored at three, below which the quantity has no meaning.
pub fn circle_to_end_segment_delta_radius(
  radius: i32,
  segment_count: i32,
) -> i32 {
  let segment_count = if segment_count <= 2 { 3 } else { segment_count };
  let alpha = PI / f64::from(segment_count);

  kiround((f64::from(radius) * (1.0 - 1.0 / alpha.cos())).abs())
}

/// The body of [`calc_arc_center_f64`], with the degeneracy flag
/// [`calc_arc_center`] needs.
///
/// `libs/kimath/src/trigo.cpp:371` to `:559`, transcribed term by term.
/// The parenthesisation of the uncertainty products is KiCad's and is kept
/// exactly, because `a / b * a / b` and `(a / b) * (a / b)` do not agree in
/// the last bit and the comparisons at `:539` and `:545` are strict.
fn calc_arc_center_f64_with_degeneracy(
  start: (f64, f64),
  mid: (f64, f64),
  end: (f64, f64),
) -> (f64, f64, bool) {
  /// Three points inside a box this big are all rounding noise
  /// (`trigo.cpp:375`).
  const CLUSTER_EXTENT: f64 = 5.0;
  /// A pair closer than this is one point (`trigo.cpp:380`).
  const COINCIDENT_RADIUS_SQUARED: f64 = 2.0 * 2.0;

  let min_x = start.0.min(mid.0).min(end.0);
  let max_x = start.0.max(mid.0).max(end.0);
  let min_y = start.1.min(mid.1).min(end.1);
  let max_y = start.1.max(mid.1).max(end.1);

  if max_x - min_x < CLUSTER_EXTENT && max_y - min_y < CLUSTER_EXTENT {
    return (
      (start.0 + mid.0 + end.0) / 3.0,
      (start.1 + mid.1 + end.1) / 3.0,
      true,
    );
  }

  /// `trigo.cpp:392`.
  fn coincident(first: (f64, f64), second: (f64, f64)) -> bool {
    let x = first.0 - second.0;
    let y = first.1 - second.1;

    x * x + y * y < COINCIDENT_RADIUS_SQUARED
  }

  // Two distinct points fall back to the chord midpoint, as the diameter
  // arc paths below do (`trigo.cpp:397`).
  if coincident(start, mid) || coincident(mid, end) {
    return ((start.0 + end.0) / 2.0, (start.1 + end.1) / 2.0, true);
  }

  if coincident(start, end) {
    return ((start.0 + mid.0) / 2.0, (start.1 + mid.1) / 2.0, true);
  }

  let mut y_delta_21 = mid.1 - start.1;
  let mut x_delta_21 = mid.0 - start.0;
  let mut y_delta_32 = end.1 - mid.1;
  let mut x_delta_32 = end.0 - mid.0;

  // The mid point is the halfway point of a quarter turn whose two chords
  // are axis aligned; the centre then lies on the chord from start to end
  // (`trigo.cpp:410`).
  if (x_delta_21 == 0.0 && y_delta_32 == 0.0)
    || (y_delta_21 == 0.0 && x_delta_32 == 0.0)
  {
    return ((start.0 + end.0) / 2.0, (start.1 + end.1) / 2.0, false);
  }

  if x_delta_21 == 0.0 {
    x_delta_21 = f64::EPSILON;
  }

  if x_delta_32 == 0.0 {
    x_delta_32 = -f64::EPSILON;
  }

  let mut a_slope = y_delta_21 / x_delta_21;
  let mut b_slope = y_delta_32 / x_delta_32;

  // The y deltas are guarded after the slopes are taken so that a
  // horizontal chord keeps its exact zero slope while the uncertainty
  // terms below stay finite (`trigo.cpp:432`).
  if y_delta_21 == 0.0 {
    y_delta_21 = f64::EPSILON;
  }

  if y_delta_32 == 0.0 {
    y_delta_32 = f64::EPSILON;
  }

  let d_a_slope =
    a_slope * euclidean_norm_f64(0.5 / y_delta_21, 0.5 / x_delta_21);
  let d_b_slope =
    b_slope * euclidean_norm_f64(0.5 / y_delta_32, 0.5 / x_delta_32);

  if a_slope == b_slope {
    if start == end {
      // A full turn: the centre is halfway between the mid point and
      // either endpoint (`trigo.cpp:446`).
      return ((start.0 + mid.0) / 2.0, (start.1 + mid.1) / 2.0, false);
    }

    // Colinear points put the centre at infinity, so the slopes are
    // nudged apart. KiCad's own warning at `:453` says this induces a
    // small error in the centre.
    a_slope += f64::EPSILON;
    b_slope -= f64::EPSILON;
  }

  // `std::numeric_limits<double>::epsilon()` is too small here and
  // generates false results, so KiCad uses 1e-10 (`trigo.cpp:466`).
  if a_slope == 0.0 {
    a_slope = 1e-10;
  }

  if b_slope == 0.0 {
    b_slope = 1e-10;
  }

  // What follows is the centre from the two bisector slopes together with
  // the error propagated through every term, truncated at the first order
  // of the series and ignoring covariance. All the `d` prefixed values are
  // approximately a standard deviation (`trigo.cpp:476` to `:527`).
  let ab_slope_start_end_y = a_slope * b_slope * (start.1 - end.1);
  let d_ab_slope_start_end_y = ab_slope_start_end_y
    * (d_a_slope / a_slope * d_a_slope / a_slope
      + d_b_slope / b_slope * d_b_slope / b_slope
      + FRAC_1_SQRT_2 / (start.1 - end.1) * FRAC_1_SQRT_2 / (start.1 - end.1))
      .sqrt();

  let b_slope_start_mid_x = b_slope * (start.0 + mid.0);
  let d_b_slope_start_mid_x = b_slope_start_mid_x
    * (d_b_slope / b_slope * d_b_slope / b_slope
      + FRAC_1_SQRT_2 / (start.0 + mid.0) * FRAC_1_SQRT_2 / (start.0 + mid.0))
      .sqrt();

  let a_slope_mid_end_x = a_slope * (mid.0 + end.0);
  let d_a_slope_mid_end_x = a_slope_mid_end_x
    * (d_a_slope / a_slope * d_a_slope / a_slope
      + FRAC_1_SQRT_2 / (mid.0 + end.0) * FRAC_1_SQRT_2 / (mid.0 + end.0))
      .sqrt();

  let twice_ba_slope_diff = 2.0 * (b_slope - a_slope);
  let d_twice_ba_slope_diff =
    2.0 * (d_b_slope * d_b_slope + d_a_slope * d_a_slope).sqrt();

  let center_numerator_x =
    ab_slope_start_end_y + b_slope_start_mid_x - a_slope_mid_end_x;
  let d_center_numerator_x = (d_ab_slope_start_end_y * d_ab_slope_start_end_y
    + d_b_slope_start_mid_x * d_b_slope_start_mid_x
    + d_a_slope_mid_end_x * d_a_slope_mid_end_x)
    .sqrt();

  let center_x = (ab_slope_start_end_y + b_slope_start_mid_x
    - a_slope_mid_end_x)
    / twice_ba_slope_diff;
  let d_center_x = center_x
    * (d_center_numerator_x / center_numerator_x * d_center_numerator_x
      / center_numerator_x
      + d_twice_ba_slope_diff / twice_ba_slope_diff * d_twice_ba_slope_diff
        / twice_ba_slope_diff)
      .sqrt();

  let center_numerator_y = (start.0 + mid.0) / 2.0 - center_x;
  let d_center_numerator_y = (1.0 / 8.0 + d_center_x * d_center_x).sqrt();

  let center_first_term = center_numerator_y / a_slope;
  let d_center_first_term_y = center_first_term
    * (d_center_numerator_y / center_numerator_y * d_center_numerator_y
      / center_numerator_y
      + d_a_slope / a_slope * d_a_slope / a_slope)
      .sqrt();

  let center_y = center_first_term + (start.1 + mid.1) / 2.0;
  let d_center_y =
    (d_center_first_term_y * d_center_first_term_y + 1.0 / 8.0).sqrt();

  let rounded_100_center_x = ((center_x + 50.0) / 100.0).floor() * 100.0;
  let rounded_100_center_y = ((center_y + 50.0) / 100.0).floor() * 100.0;
  let rounded_10_center_x = ((center_x + 5.0) / 10.0).floor() * 10.0;
  let rounded_10_center_y = ((center_y + 5.0) / 10.0).floor() * 10.0;

  if (rounded_100_center_x - center_x).abs() < d_center_x
    && (rounded_100_center_y - center_y).abs() < d_center_y
  {
    (rounded_100_center_x, rounded_100_center_y, false)
  } else if (rounded_10_center_x - center_x).abs() < d_center_x
    && (rounded_10_center_y - center_y).abs() < d_center_y
  {
    (rounded_10_center_x, rounded_10_center_y, false)
  } else {
    (center_x, center_y, false)
  }
}

/// The point on a circle nearest a given floating point position.
///
/// Port of `CIRCLE::NearestPoint( const VECTOR2D& )`,
/// `libs/kimath/src/geometry/circle.cpp:208`, the `double` overload
/// `SHAPE_ARC::Collide( const VECTOR2I& )` uses (`shape_arc.cpp:883`).
/// Nothing is rounded, so the caller decides where the result lands on the
/// nanometre grid; `Collide` truncates it into the reported location.
fn circle_nearest_point_f64(
  center: Vec2,
  radius: i32,
  point: Vec2,
) -> (f64, f64) {
  let mut x = f64::from(point.x) - f64::from(center.x);
  let y = f64::from(point.y) - f64::from(center.y);

  // A point at the centre has no nearest point, so KiCad picks the
  // positive x direction (`circle.cpp:214`).
  if x == 0.0 && y == 0.0 {
    x = 1.0;
  }

  let (resized_x, resized_y) = resize_f64(x, y, f64::from(radius));

  (
    resized_x + f64::from(center.x),
    resized_y + f64::from(center.y),
  )
}

/// The point on a circle furthest from a given point.
///
/// Port of `CIRCLE::FurthestPoint( const VECTOR2I& )`,
/// `libs/kimath/src/geometry/circle.cpp:221`, which is
/// [`circle_nearest_point`] with the difference taken the other way
/// round.
fn circle_furthest_point(center: Vec2, radius: i32, point: Vec2) -> Vec2 {
  let mut offset = center.widening_sub(point).saturating_to_vec2();

  if offset.x == 0 && offset.y == 0 {
    offset = Vec2::new(1, 0);
  }

  offset.resize(radius) + center
}

/// The intersections of two circles.
///
/// Port of `CIRCLE::Intersect( const CIRCLE& )`,
/// `libs/kimath/src/geometry/circle.cpp:243`. The problem is moved to the
/// frame where this circle sits at the origin and the other on the
/// positive x axis, solved there, and rotated back. Concentric circles
/// answer nothing, even when their radii agree and every point is an
/// intersection (`:328`).
///
/// Deviations, both widenings: the rotated solution is built from `i64`
/// coordinates and saturates into `i32`, and the centre is added back in
/// `i64`. KiCad narrows and adds in `int`.
fn circle_intersect_circle(
  first_center: Vec2,
  first_radius: i32,
  second_center: Vec2,
  second_radius: i32,
) -> Vec<Vec2> {
  let center_to_center = second_center.widening_sub(first_center);
  let center_distance = center_to_center.euclidean_norm();
  let first = i64::from(first_radius);
  let second = i64::from(second_radius);

  if center_distance > first + second
    || center_distance < (first - second).abs()
    || center_distance == 0
  {
    return Vec::new();
  }

  let x = ((center_distance * center_distance) + (first * first)
    - (second * second))
    / (2 * center_distance);
  let remainder = (first * first) - (x * x);

  if remainder < 0 {
    return Vec::new();
  }

  // `KiROUND` without an explicit return type narrows to `int` before the
  // `int64_t` assignment at `circle.cpp:340`.
  let y = i64::from(kiround((remainder as f64).sqrt()));
  let rotation =
    Degrees::from_vector(center_to_center.x as f64, center_to_center.y as f64);
  let origin = Vec2::new(0, 0);
  let mut intersections = vec![
    (Vec2L::from(rotate_point(
      Vec2L::new(x, y).saturating_to_vec2(),
      origin,
      -rotation,
    )) + Vec2L::from(first_center))
    .saturating_to_vec2(),
  ];

  if y != 0 {
    intersections.push(
      (Vec2L::from(rotate_point(
        Vec2L::new(x, -y).saturating_to_vec2(),
        origin,
        -rotation,
      )) + Vec2L::from(first_center))
      .saturating_to_vec2(),
    );
  }

  intersections
}

/// A squared length, saturating instead of wrapping.
///
/// Port of `SEG::Square`, `libs/kimath/include/geometry/seg.h:119`, which
/// takes an `int` and multiplies in `i64`. Every caller here squares a
/// clearance or a half width, so the saturation is unreachable on any
/// board; see the note on [`crate::geometry::collision`].
fn square(value: i64) -> i64 {
  value.saturating_mul(value)
}

/// Clamp an `i64` into an `i32`.
fn saturate_i32(value: i64) -> i32 {
  value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// The point on a circle nearest a given point.
///
/// Port of `CIRCLE::NearestPoint( const VECTOR2I& )`,
/// `libs/kimath/src/geometry/circle.cpp:197`. A point at the centre has no
/// nearest point, so KiCad picks the positive x direction arbitrarily so
/// that the answer is always on the circumference.
fn circle_nearest_point(center: Vec2, radius: i32, point: Vec2) -> Vec2 {
  let mut offset = point - center;

  if offset.x == 0 && offset.y == 0 {
    offset.x = 1;
  }

  offset.resize(radius) + center
}

/// A floating point vector scaled to a new length.
///
/// Port of `VECTOR2<double>::Resize`,
/// `libs/kimath/include/math/vector2d.h:388`, the floating point
/// instantiation: no rounding, and the exact diagonal is taken from
/// `sqrt(1/2)` rather than through the rescaling. `rescale` for `double`
/// is the generic template (`libs/kimath/include/math/util.h:135`), a
/// plain `numerator * value / denominator`.
fn resize_f64(x: f64, y: f64, new_length: f64) -> (f64, f64) {
  if x == 0.0 && y == 0.0 {
    return (0.0, 0.0);
  }

  let (new_x, new_y) = if x.abs() == y.abs() {
    let scaled = new_length.abs() * FRAC_1_SQRT_2;

    (scaled, scaled)
  } else {
    let x_squared = x * x;
    let y_squared = y * y;
    let length_squared = x_squared + y_squared;
    let new_length_squared = new_length * new_length;

    (
      (new_length_squared * x_squared / length_squared).sqrt(),
      (new_length_squared * y_squared / length_squared).sqrt(),
    )
  };

  let signed = (
    if x < 0.0 { -new_x } else { new_x },
    if y < 0.0 { -new_y } else { new_y },
  );
  let length_sign = f64::from(sign(new_length));

  (signed.0 * length_sign, signed.1 * length_sign)
}

/// A nanometre point widened for the floating point routines.
fn to_f64(point: Vec2) -> (f64, f64) {
  (f64::from(point.x), f64::from(point.y))
}

/// `KiROUND( const VECTOR2D& )`,
/// `libs/kimath/include/math/vector2d.h:687`.
fn kiround_f64_pair(point: (f64, f64)) -> Vec2 {
  Vec2::new(kiround(point.0), kiround(point.1))
}

/// The `VECTOR2D` to `VECTOR2I` narrowing conversion,
/// `libs/kimath/include/math/vector2d.h:85`: clamp into the integer range
/// and truncate towards zero. Note it truncates where
/// [`kiround_f64_pair`] rounds; `ConstructFromStartEndAngle` takes this
/// path and the three point centre takes the other.
fn truncate_f64_pair(point: (f64, f64)) -> Vec2 {
  Vec2::new(truncate_f64_to_i32(point.0), truncate_f64_to_i32(point.1))
}

/// One coordinate of [`truncate_f64_pair`].
///
/// Rust saturates a float to integer cast, where C++ leaves an out of
/// range conversion undefined. Every arc the router builds is far inside
/// the range; a nearly flat one is not, and this is where it stops.
fn truncate_f64_to_i32(value: f64) -> i32 {
  value as i32
}

#[cfg(test)]
mod tests {
  use super::*;

  /// KiCad's `ARC_PROPERTIES`,
  /// `qa/tests/libs/kimath/geometry/test_shape_arc.cpp:42`, with the
  /// bounding box split into the origin and size `BOX2I` is built from.
  struct ArcProperties {
    /// The expected centre.
    center: Vec2,
    /// The expected start point.
    start: Vec2,
    /// The expected end point.
    end: Vec2,
    /// The expected sweep in degrees.
    central_angle: f64,
    /// The expected start angle in degrees.
    start_angle: f64,
    /// The expected end angle in degrees.
    end_angle: f64,
    /// The expected radius.
    radius: i32,
    /// The expected bounding box origin.
    bbox_origin: Vec2,
    /// The expected bounding box size.
    bbox_size: Vec2,
  }

  /// KiCad's `KI_TEST::IsWithin`,
  /// `qa/qa_utils/include/qa_utils/numeric.h:58`.
  fn is_within(value: f64, nominal: f64, error: f64) -> bool {
    value >= nominal - error && value <= nominal + error
  }

  /// KiCad's `KI_TEST::IsWithinWrapped`,
  /// `qa/qa_utils/include/qa_utils/numeric.h:40`.
  fn is_within_wrapped(
    value: f64,
    nominal: f64,
    wrap: f64,
    error: f64,
  ) -> bool {
    let mut difference = (value - nominal) % wrap;

    if difference > wrap / 2.0 {
      difference -= wrap;
    } else if difference < -wrap / 2.0 {
      difference += wrap;
    }

    difference.abs() <= error
  }

  /// KiCad's `KI_TEST::IsVecWithinTol`,
  /// `qa/qa_utils/include/qa_utils/geometry/geometry.h:51`.
  fn is_vec_within_tolerance(
    value: Vec2,
    expected: Vec2,
    tolerance: i32,
  ) -> bool {
    (i64::from(value.x) - i64::from(expected.x)).abs() <= i64::from(tolerance)
      && (i64::from(value.y) - i64::from(expected.y)).abs()
        <= i64::from(tolerance)
  }

  /// KiCad's `CheckArcGeom`, `test_shape_arc.cpp:60`. `CheckArc`
  /// (`:124`) wraps it once more to exercise `Clone`, which a `Copy`
  /// value type does not need.
  fn check_arc_geom(
    arc: &ShapeArc,
    properties: &ArcProperties,
    synthetic_tolerance: i32,
  ) {
    /// KiCad's `angle_tol_deg`, `test_shape_arc.cpp:63`.
    const ANGLE_TOLERANCE_DEGREES: f64 = 2.0;
    /// KiCad's `pos_tol`, `test_shape_arc.cpp:66`.
    const POSITION_TOLERANCE: i32 = 1;

    assert!(
      is_vec_within_tolerance(arc.end(), properties.end, POSITION_TOLERANCE),
      "end point {:?} against {:?}",
      arc.end(),
      properties.end
    );
    assert!(
      is_vec_within_tolerance(
        arc.center(),
        properties.center,
        synthetic_tolerance
      ),
      "centre {:?} against {:?}",
      arc.center(),
      properties.center
    );
    assert!(
      is_within_wrapped(
        arc.central_angle().as_degrees(),
        properties.central_angle,
        360.0,
        ANGLE_TOLERANCE_DEGREES
      ),
      "central angle {} against {}",
      arc.central_angle().as_degrees(),
      properties.central_angle
    );
    assert!(
      is_within_wrapped(
        arc.start_angle().as_degrees(),
        properties.start_angle,
        360.0,
        ANGLE_TOLERANCE_DEGREES
      ),
      "start angle {} against {}",
      arc.start_angle().as_degrees(),
      properties.start_angle
    );
    assert!(
      is_within_wrapped(
        arc.end_angle().as_degrees(),
        properties.end_angle,
        360.0,
        ANGLE_TOLERANCE_DEGREES
      ),
      "end angle {} against {}",
      arc.end_angle().as_degrees(),
      properties.end_angle
    );
    assert!(
      is_within(
        arc.radius(),
        f64::from(properties.radius),
        f64::from(synthetic_tolerance)
      ),
      "radius {} against {}",
      arc.radius(),
      properties.radius
    );

    // The normalisation contracts of `test_shape_arc.cpp:89`.
    assert!(arc.start_angle().as_degrees() >= 0.0);
    assert!(arc.start_angle().as_degrees() <= 360.0);
    assert!(arc.end_angle().as_degrees() >= 0.0);
    assert!(arc.end_angle().as_degrees() <= 360.0);
    assert!(arc.central_angle().as_degrees() >= -360.0);
    assert!(arc.central_angle().as_degrees() <= 360.0);

    let chord = arc.chord();

    assert!(is_vec_within_tolerance(
      chord.a,
      properties.start,
      POSITION_TOLERANCE
    ));
    assert!(is_vec_within_tolerance(
      chord.b,
      properties.end,
      POSITION_TOLERANCE
    ));

    // `KI_TEST::IsBoxWithinTol` allows twice the tolerance on the size,
    // `qa/qa_utils/include/qa_utils/geometry/geometry.h:62`.
    let bbox = arc.bbox(0);
    let origin = bbox.origin();
    let size = bbox.size();

    assert!(
      (origin.x - i64::from(properties.bbox_origin.x)).abs()
        <= i64::from(POSITION_TOLERANCE)
        && (origin.y - i64::from(properties.bbox_origin.y)).abs()
          <= i64::from(POSITION_TOLERANCE),
      "bbox origin {origin:?} against {:?}",
      properties.bbox_origin
    );
    assert!(
      (size.x - i64::from(properties.bbox_size.x)).abs()
        <= i64::from(POSITION_TOLERANCE) * 2
        && (size.y - i64::from(properties.bbox_size.y)).abs()
          <= i64::from(POSITION_TOLERANCE) * 2,
      "bbox size {size:?} against {:?}",
      properties.bbox_size
    );
  }

  /// `NullCtor`, `test_shape_arc.cpp:146`. A default arc is degenerate and
  /// answers every query without panicking.
  #[test]
  fn null_ctor() {
    let arc = ShapeArc::default();

    assert_eq!(arc.width(), 0);

    check_arc_geom(
      &arc,
      &ArcProperties {
        center: Vec2::new(0, 0),
        start: Vec2::new(0, 0),
        end: Vec2::new(0, 0),
        central_angle: 0.0,
        start_angle: 0.0,
        end_angle: 0.0,
        radius: 0,
        bbox_origin: Vec2::new(0, 0),
        bbox_size: Vec2::new(0, 0),
      },
      1,
    );
  }

  /// `BasicSMEGeom`, `test_shape_arc.cpp:254`, over the table at `:188`.
  #[test]
  fn basic_sme_geom() {
    let cases: [(&str, [Vec2; 3], i32, ArcProperties); 3] = [
      (
        "S(-100,0), M(0,100), E(100,0)",
        [Vec2::new(-100, 0), Vec2::new(0, 100), Vec2::new(100, 0)],
        0,
        ArcProperties {
          center: Vec2::new(0, 0),
          start: Vec2::new(-100, 0),
          end: Vec2::new(100, 0),
          central_angle: 180.0,
          start_angle: 180.0,
          end_angle: 0.0,
          radius: 100,
          bbox_origin: Vec2::new(-100, 0),
          bbox_size: Vec2::new(200, 100),
        },
      ),
      (
        "S(100,0), M(0,100), E(-100,0) (reversed)",
        [Vec2::new(100, 0), Vec2::new(0, 100), Vec2::new(-100, 0)],
        0,
        ArcProperties {
          center: Vec2::new(0, 0),
          start: Vec2::new(100, 0),
          end: Vec2::new(-100, 0),
          central_angle: -180.0,
          start_angle: 0.0,
          end_angle: 180.0,
          radius: 100,
          bbox_origin: Vec2::new(-100, 0),
          bbox_size: Vec2::new(200, 100),
        },
      ),
      (
        // The mid point is nowhere near the halfway point of the sweep;
        // the three point form takes it as given and the result is a
        // 270 degree arc with the bottom right quadrant open.
        "S(100,0), M(-100,0), E(0,100) (bad midpoint)",
        [Vec2::new(100, 0), Vec2::new(-100, 0), Vec2::new(0, 100)],
        0,
        ArcProperties {
          center: Vec2::new(0, 0),
          start: Vec2::new(100, 0),
          end: Vec2::new(0, 100),
          central_angle: -270.0,
          start_angle: 0.0,
          end_angle: 90.0,
          radius: 100,
          bbox_origin: Vec2::new(-100, -100),
          bbox_size: Vec2::new(200, 200),
        },
      ),
    ];

    for (name, points, width, properties) in cases {
      let arc = ShapeArc::new(points[0], points[1], points[2], width);

      println!("case {name}");
      check_arc_geom(&arc, &properties, 1);
    }
  }

  /// `BasicSECGeom`, `test_shape_arc.cpp:620`, over the table at `:610`.
  /// It pins the mid point, which is the only thing
  /// `ConstructFromStartEndCenter` keeps of the centre it was given.
  #[test]
  fn basic_sec_geom() {
    let cases: [(&str, Vec2, Vec2, Vec2, bool, Vec2); 6] = [
      (
        "180 deg, clockwise",
        Vec2::new(100, 0),
        Vec2::new(0, 0),
        Vec2::new(50, 0),
        true,
        Vec2::new(50, -50),
      ),
      (
        "180 deg, anticlockwise",
        Vec2::new(100, 0),
        Vec2::new(0, 0),
        Vec2::new(50, 0),
        false,
        Vec2::new(50, 50),
      ),
      (
        "180 deg flipped, clockwise",
        Vec2::new(0, 0),
        Vec2::new(100, 0),
        Vec2::new(50, 0),
        true,
        Vec2::new(50, 50),
      ),
      (
        "180 deg flipped, anticlockwise",
        Vec2::new(0, 0),
        Vec2::new(100, 0),
        Vec2::new(50, 0),
        false,
        Vec2::new(50, -50),
      ),
      (
        "90 deg, clockwise",
        Vec2::new(-100, 0),
        Vec2::new(0, 100),
        Vec2::new(0, 0),
        true,
        Vec2::new(-71, 71),
      ),
      (
        "90 deg, anticlockwise",
        Vec2::new(-100, 0),
        Vec2::new(0, 100),
        Vec2::new(0, 0),
        false,
        Vec2::new(71, -71),
      ),
    ];

    for (name, start, end, center, clockwise, expected_mid) in cases {
      let arc =
        ShapeArc::from_start_end_center(start, end, center, clockwise, 0);

      assert_eq!(arc.arc_mid(), expected_mid, "case {name}");
    }
  }

  /// `ArePolylineEndPointsNearCircle`, `test_shape_arc.cpp:1073`.
  fn are_polyline_end_points_near_circle(
    polyline: &LineChain,
    center: Vec2,
    radius: i32,
    tolerance: i32,
  ) -> bool {
    (0..polyline.point_count()).all(|index| {
      let distance = (center - polyline.point(index)).euclidean_norm();

      is_within(f64::from(distance), f64::from(radius), f64::from(tolerance))
    })
  }

  /// `ArePolylineMidPointsNearCircle`, `test_shape_arc.cpp:1095`.
  fn are_polyline_mid_points_near_circle(
    polyline: &LineChain,
    center: Vec2,
    radius: i32,
    tolerance: i32,
  ) -> bool {
    (0..polyline.point_count().saturating_sub(1)).all(|index| {
      let mid = (polyline.point(index) + polyline.point(index + 1)) / 2.0;
      let distance = (center - mid).euclidean_norm();

      is_within(f64::from(distance), f64::from(radius), f64::from(tolerance))
    })
  }

  /// `ArcToPolyline`, `test_shape_arc.cpp:1150`, over the table at
  /// `:1111`. The endpoints have to land exactly and every other point
  /// and every segment midpoint within the requested accuracy.
  #[test]
  fn arc_to_polyline() {
    /// KiCad's `accuracy`, `test_shape_arc.cpp:1158`. It notes that
    /// anything near 1 will not work, because the points are integers.
    const ACCURACY: i32 = 100;
    /// KiCad's `epsilon`, `test_shape_arc.cpp:1159`.
    const EPSILON: i32 = 1;

    let cases: [(&str, Vec2, Vec2, f64); 4] = [
      ("Zero rad", Vec2::new(0, 0), Vec2::new(0, 0), 180.0),
      (
        "Semicircle",
        Vec2::new(0, 0),
        Vec2::new(-1_000_000, 0),
        180.0,
      ),
      (
        // Very small circles must not fall apart, and a reversed sweep
        // has to work too.
        "Extremely small semicircle",
        Vec2::new(0, 0),
        Vec2::new(-1000, 0),
        -180.0,
      ),
      (
        "Non-round geometry",
        Vec2::new(0, 0),
        Vec2::new(1_234_567, 0),
        42.22,
      ),
    ];

    for (name, center, start, central_angle) in cases {
      let arc = ShapeArc::from_center_start_angle(
        center,
        start,
        Degrees::new(central_angle),
        0,
      );
      let chain = arc.convert_to_polyline(ACCURACY);

      assert_eq!(chain.point(0), start, "case {name}: start point");
      assert_eq!(
        chain.last_point(),
        Some(arc.end()),
        "case {name}: end point"
      );

      let radius = (center - start).euclidean_norm();

      assert!(
        are_polyline_end_points_near_circle(
          &chain,
          center,
          radius,
          ACCURACY + EPSILON
        ),
        "case {name}: end points"
      );
      assert!(
        are_polyline_mid_points_near_circle(
          &chain,
          center,
          radius,
          ACCURACY + EPSILON
        ),
        "case {name}: mid points"
      );
    }
  }

  /// `DegenerateArcCoincidentPoints`, `test_shape_arc.cpp:1343`. Three
  /// points a nanometre apart must not fabricate a two metre circumcircle,
  /// which is what the drag preview used to draw.
  #[test]
  fn degenerate_arc_coincident_points() {
    let arc = ShapeArc::new(
      Vec2::new(135_674_000, 84_576_744),
      Vec2::new(135_673_999, 84_576_744),
      Vec2::new(135_673_998, 84_576_744),
      100_000,
    );

    assert!(arc.radius() < 10.0, "radius {}", arc.radius());

    let polyline = arc.convert_to_polyline(ShapeArc::DEFAULT_ACCURACY_FOR_PCB);
    let bbox = polyline.bbox(0).expect("a polyline has points");

    assert!(bbox.width() < 1000);
    assert!(bbox.height() < 1000);
    assert!(arc.length() < 1000.0, "length {}", arc.length());
  }

  /// `CollinearArcSweepIsNotAFullTurn`, `test_shape_arc.cpp:1361`. The
  /// coincident start and mid send the centre to the chord midpoint, which
  /// puts start and end antipodal and would read as a clean half turn if
  /// `IsEffectiveLine` did not catch it first.
  #[test]
  fn collinear_arc_sweep_is_not_a_full_turn() {
    let arc = ShapeArc::new(
      Vec2::new(2_275_000, 3_123_714),
      Vec2::new(2_275_000, 3_123_715),
      Vec2::new(2_275_000, 3_123_720),
      127_000,
    );

    assert!(arc.central_angle().as_degrees().abs() < 1.0);

    // The 6 nanometre chord is the whole run.
    assert!(
      (arc.length() - 6.0).abs() / 6.0 < 0.01,
      "length {}",
      arc.length()
    );

    let polyline = arc.convert_to_polyline(ShapeArc::DEFAULT_ACCURACY_FOR_PCB);
    let bbox = polyline.bbox(0).expect("a polyline has points");

    assert!(bbox.width() < 10);
    assert!(bbox.height() < 10);
  }

  /// `CurvedArcsKeepTheirSweep`, `test_shape_arc.cpp:1380`. The straight
  /// run guard must not catch a major arc, which is the only user of the
  /// full turn correction.
  #[test]
  fn curved_arcs_keep_their_sweep() {
    let quarter = ShapeArc::new(
      Vec2::new(1_000_000, 0),
      Vec2::new(707_107, 707_107),
      Vec2::new(0, 1_000_000),
      127_000,
    );

    assert!(
      (quarter.central_angle().as_degrees() - 90.0).abs() / 90.0 < 0.0001,
      "quarter sweep {}",
      quarter.central_angle().as_degrees()
    );

    let major = ShapeArc::new(
      Vec2::new(1_000_000, 0),
      Vec2::new(-1_000_000, 0),
      Vec2::new(0, -1_000_000),
      127_000,
    );

    assert!(
      (major.central_angle().as_degrees() - 270.0).abs() / 270.0 < 0.0001,
      "major sweep {}",
      major.central_angle().as_degrees()
    );
  }

  /// `CalcArcCenterTwoCoincidentStartMid`, `test_shape_arc.cpp:1395`.
  #[test]
  fn calc_arc_center_two_coincident_start_mid() {
    let center = calc_arc_center_f64((0.0, 0.0), (0.0, 0.0), (1000.0, 0.0));

    assert!((center.0 - 500.0).abs() < 1e-9, "x {}", center.0);
    assert!(center.1.abs() < 1e-9, "y {}", center.1);
  }

  /// `CalcArcCenterTwoCoincidentMidEnd`, `test_shape_arc.cpp:1408`.
  #[test]
  fn calc_arc_center_two_coincident_mid_end() {
    let center = calc_arc_center_f64((-1000.0, 0.0), (0.0, 0.0), (0.0, 0.0));

    assert!((center.0 + 500.0).abs() < 1e-9, "x {}", center.0);
    assert!(center.1.abs() < 1e-9, "y {}", center.1);
  }

  /// `CalcArcCenterTwoCoincidentStartEnd`, `test_shape_arc.cpp:1421`.
  /// Coincident endpoints with a distinct mid are a full turn, and the
  /// centre is the midpoint from there to the mid point.
  #[test]
  fn calc_arc_center_two_coincident_start_end() {
    let center = calc_arc_center_f64((0.0, 0.0), (1000.0, 0.0), (0.0, 0.0));

    assert!((center.0 - 500.0).abs() < 1e-9, "x {}", center.0);
    assert!(center.1.abs() < 1e-9, "y {}", center.1);
  }

  /// `CalcArcCenterThreeNearCoincident`, `test_shape_arc.cpp:1436`. Three
  /// near coincident points collapse to the centroid through the bounding
  /// box guard, not through the pairwise one.
  #[test]
  fn calc_arc_center_three_near_coincident() {
    let start = (100.0, 200.0);
    let mid = (101.0, 200.0);
    let end = (100.0, 201.0);
    let center = calc_arc_center_f64(start, mid, end);

    assert!(
      (center.0 - (start.0 + mid.0 + end.0) / 3.0).abs() < 1e-9,
      "x {}",
      center.0
    );
    assert!(
      (center.1 - (start.1 + mid.1 + end.1) / 3.0).abs() < 1e-9,
      "y {}",
      center.1
    );
  }

  /// `CalcArcCenterThinArcNotDegenerate`, `test_shape_arc.cpp:1450`. A
  /// thin arc has a short chord and a small sagitta but is not degenerate,
  /// and a wrongly triggered midpoint guard would shrink its radius by
  /// five times.
  #[test]
  fn calc_arc_center_thin_arc_not_degenerate() {
    let start = (0.0, 0.0);
    let mid = (10.0, 1.0);
    let end = (20.0, 0.0);
    let center = calc_arc_center_f64(start, mid, end);

    let to_start = euclidean_norm_f64(center.0 - start.0, center.1 - start.1);
    let to_mid = euclidean_norm_f64(center.0 - mid.0, center.1 - mid.1);
    let to_end = euclidean_norm_f64(center.0 - end.0, center.1 - end.1);

    assert!((to_start - to_mid).abs() / to_mid < 0.0001);
    assert!((to_mid - to_end).abs() / to_end < 0.0001);

    // The true circumradius is 50.5 nanometres.
    assert!(to_start > 40.0, "radius {to_start}");
  }

  /// `CalcArcCenterFewUnitArcKeepsItsCircumcircle`,
  /// `test_shape_arc.cpp:1473`. A few nanometre arc rounds its own mid
  /// point off the true circle, but the three points still span a healthy
  /// triangle; reading that as a coincident pair collapses the centre onto
  /// the chord and turns a quarter turn into a reflex sweep.
  #[test]
  fn calc_arc_center_few_unit_arc_keeps_its_circumcircle() {
    let arc = ShapeArc::from_center_start_angle(
      Vec2::new(0, 0),
      Vec2::new(5, 0),
      Degrees::QUARTER_TURN,
      0,
    );

    assert!(arc.central_angle().as_degrees().abs() < 180.0);

    // The chord midpoint fallback sits inside the arc it is supposed to
    // circumscribe.
    let center_to_mid = (arc.center() - arc.arc_mid()).euclidean_norm();

    assert!(f64::from(center_to_mid) > arc.radius() / 2.0);
  }

  /// `CalcArcCenterBoardScaleSanity`, `test_shape_arc.cpp:1485`. A board
  /// scale arc must not regress from the pairwise coincidence guards.
  #[test]
  fn calc_arc_center_board_scale_sanity() {
    /// KiCad's `R`, `test_shape_arc.cpp:1487`.
    const R: f64 = 50_000_000.0;

    let center = calc_arc_center_f64((R, 0.0), (0.0, R), (-R, 0.0));

    assert!(center.0.abs() < 1.0, "x {}", center.0);
    assert!(center.1.abs() < 1.0, "y {}", center.1);
  }

  /// The port's own: erratum E4, `Reverse` against `Reversed`.
  ///
  /// In KiCad the two are not equivalent, because `Reverse` leaves the
  /// cached centre, radius and bounding box alone while `Reversed`
  /// rebuilds the arc and re-runs `CalcArcCenter` on the permuted points,
  /// which through the round number snapping can land somewhere else.
  /// Nothing is cached here, so the two agree by construction and
  /// assembling a line in either scan direction gives the same geometry.
  #[test]
  fn reverse_and_reversed_agree_because_nothing_is_cached() {
    let arc = ShapeArc::new(
      Vec2::new(-1_000_000, 0),
      Vec2::new(0, 1_000_000),
      Vec2::new(1_000_000, 0),
      200_000,
    );

    let mut in_place = arc;

    in_place.reverse();

    assert_eq!(in_place, arc.reversed());
    assert_eq!(in_place.center(), arc.reversed().center());
    assert_eq!(in_place.radius(), arc.reversed().radius());
    assert_eq!(in_place.bbox(0), arc.reversed().bbox(0));

    // Reversing twice is the identity, and the curve never moved.
    let mut back = in_place;

    back.reverse();

    assert_eq!(back, arc);
    assert_eq!(in_place.arc_mid(), arc.arc_mid());
    assert_eq!(in_place.is_ccw(), !arc.is_ccw());
  }

  /// The port's own: [`ShapeArc::from_start_end_center`] does not keep the
  /// centre it was given, which is the loss the doc comment warns about.
  ///
  /// The first case is KiCad's own "90 deg, clockwise" row
  /// (`test_shape_arc.cpp:615`), where the half angle rotation lands the
  /// mid point on `(-71, 71)`, two nanometres off the true circle, and the
  /// centre recomputed from the three points comes back one nanometre away
  /// from the origin the caller passed in. KiCad behaves the same way; it
  /// only never asserts on it.
  #[test]
  fn from_start_end_center_loses_the_centre() {
    let arc = ShapeArc::from_start_end_center(
      Vec2::new(-100, 0),
      Vec2::new(0, 100),
      Vec2::new(0, 0),
      true,
      0,
    );

    assert_eq!(arc.arc_mid(), Vec2::new(-71, 71));
    assert_ne!(arc.center(), Vec2::new(0, 0));
    assert_eq!(arc.center(), Vec2::new(-1, 1));
    // Both endpoints do survive exactly, which is what a host can rely on.
    assert_eq!(arc.start(), Vec2::new(-100, 0));
    assert_eq!(arc.end(), Vec2::new(0, 100));

    // A board scale one, where the round number snapping of erratum E1
    // moves the answer further than the rounding of the mid point does:
    // the centre asked for was `(3, 7)` and the centre reported is the
    // nearest multiple of ten.
    let snapped = ShapeArc::from_start_end_center(
      Vec2::new(1_234_567, 89),
      Vec2::new(89, 1_234_567),
      Vec2::new(3, 7),
      false,
      0,
    );

    assert_eq!(snapped.center(), Vec2::new(10, 10));
  }

  /// The port's own: [`ShapeArc::from_start_end_angle`] does not keep the
  /// angle it was given either.
  #[test]
  fn from_start_end_angle_loses_the_angle() {
    let angle = Degrees::new(42.22);
    let arc = ShapeArc::from_start_end_angle(
      Vec2::new(0, 0),
      Vec2::new(1_234_567, 0),
      angle,
      0,
    );

    assert_eq!(arc.start(), Vec2::new(0, 0));
    assert_eq!(arc.end(), Vec2::new(1_234_567, 0));
    assert_ne!(arc.central_angle().as_degrees(), angle.as_degrees());
    // It is close, though: the loss is the rounded mid point, not a
    // different arc.
    assert!(
      (arc.central_angle().as_degrees() - angle.as_degrees()).abs() < 0.01,
      "sweep {}",
      arc.central_angle().as_degrees()
    );
  }

  /// The port's own: [`ShapeArc::move_by`] is exact on all three points
  /// and moves the centre with them.
  #[test]
  fn move_by_translates_all_three_points() {
    let arc = ShapeArc::new(
      Vec2::new(-100_000, 0),
      Vec2::new(0, 100_000),
      Vec2::new(100_000, 0),
      50_000,
    );
    let delta = Vec2::new(1_234_567, -7_654_321);
    let mut moved = arc;

    moved.move_by(delta);

    assert_eq!(moved.start(), arc.start() + delta);
    assert_eq!(moved.arc_mid(), arc.arc_mid() + delta);
    assert_eq!(moved.end(), arc.end() + delta);
    assert_eq!(moved.center(), arc.center() + delta);
    assert_eq!(moved.width(), arc.width());
    assert_eq!(moved.is_ccw(), arc.is_ccw());
  }

  /// The port's own: [`ShapeArc::mirror`] reflects all three points and
  /// flips the handedness without reordering anything.
  #[test]
  fn mirror_reflects_all_three_points() {
    let arc = ShapeArc::new(
      Vec2::new(-100_000, 0),
      Vec2::new(0, 100_000),
      Vec2::new(100_000, 0),
      50_000,
    );
    // The x axis.
    let axis = Seg::new(Vec2::new(-1_000_000, 0), Vec2::new(1_000_000, 0));
    let mut mirrored = arc;

    mirrored.mirror(&axis);

    assert_eq!(mirrored.start(), Vec2::new(-100_000, 0));
    assert_eq!(mirrored.arc_mid(), Vec2::new(0, -100_000));
    assert_eq!(mirrored.end(), Vec2::new(100_000, 0));
    assert_eq!(mirrored.is_ccw(), !arc.is_ccw());
    assert_eq!(mirrored.width(), arc.width());

    // Mirroring twice is the identity.
    mirrored.mirror(&axis);

    assert_eq!(mirrored, arc);
  }

  /// The port's own: the centre carries its provenance, so a caller can
  /// tell a real circumcentre from one of `CalcArcCenter`'s stand ins.
  #[test]
  fn arc_center_reports_its_provenance() {
    let real = ShapeArc::new(
      Vec2::new(-1_000_000, 0),
      Vec2::new(0, 1_000_000),
      Vec2::new(1_000_000, 0),
      0,
    );

    assert!(!real.arc_center().is_degenerate());

    // Three points inside a five nanometre box.
    let cluster =
      ShapeArc::new(Vec2::new(0, 0), Vec2::new(1, 1), Vec2::new(2, 0), 0);

    assert!(cluster.arc_center().is_degenerate());

    // A coincident pair with a distant third point.
    let pair = ShapeArc::new(
      Vec2::new(2_275_000, 3_123_714),
      Vec2::new(2_275_000, 3_123_715),
      Vec2::new(2_275_000, 3_123_720),
      0,
    );

    assert!(pair.arc_center().is_degenerate());
  }

  /// One row of KiCad's `ARC_PT_COLLIDE_CASE`,
  /// `test_shape_arc.cpp:676`, and of `ARC_SEG_COLLIDE_CASE` (`:772`),
  /// which share the centre, start and angle geometry.
  struct ArcCollideCase {
    /// The case name KiCad gives it.
    name: &'static str,
    /// The arc's centre.
    center: Vec2,
    /// The arc's start point.
    start: Vec2,
    /// The sweep in degrees.
    central_angle: f64,
    /// The clearance the collision is tested at.
    clearance: i32,
    /// Whether KiCad reports a collision.
    collides: bool,
    /// The gap KiCad reports, meaningful only when it collides.
    distance: i32,
  }

  /// The two halves of KiCad's `CollidePt` and `CollideSeg` bodies
  /// (`test_shape_arc.cpp:747` and `:817`): the zero width arc reports
  /// the clearance as the gap, and the same arc widened to twice the
  /// clearance reports zero.
  ///
  /// KiCad leaves its `dist` at `-1` when the call answers false, which
  /// an [`Option`] says by being `None`.
  fn check_collide_case(
    case: &ArcCollideCase,
    zero_width: Option<ShapeCollision>,
    wide: Option<ShapeCollision>,
  ) {
    assert_eq!(
      zero_width.is_some(),
      case.collides,
      "{}: collision at the nominal clearance",
      case.name
    );

    if let Some(collision) = zero_width {
      assert_eq!(collision.actual, case.distance, "{}: gap", case.name);
    }

    assert_eq!(
      wide.is_some(),
      case.collides,
      "{}: collision with the width folded in",
      case.name
    );

    if let Some(collision) = wide {
      assert_eq!(collision.actual, 0, "{}: widened gap", case.name);
    }
  }

  /// `CollidePt`, `test_shape_arc.cpp:742`, over the table at `:687`.
  #[test]
  fn collide_pt() {
    let cases: [(ArcCollideCase, Vec2); 41] = [
      (
        arc_case(" 270deg, 0 cl, 0   deg    ", 270.0, 0, true, 0),
        Vec2::new(100, 0),
      ),
      (
        arc_case(" 270deg, 0 cl, 90  deg    ", 270.0, 0, true, 0),
        Vec2::new(0, 100),
      ),
      (
        arc_case(" 270deg, 0 cl, 180 deg    ", 270.0, 0, true, 0),
        Vec2::new(-100, 0),
      ),
      (
        arc_case(" 270deg, 0 cl, 270 deg    ", 270.0, 0, true, 0),
        Vec2::new(0, -100),
      ),
      (
        arc_case(" 270deg, 0 cl, 45  deg    ", 270.0, 0, true, 0),
        Vec2::new(71, 71),
      ),
      (
        arc_case(" 270deg, 0 cl, -45 deg    ", 270.0, 0, false, -1),
        Vec2::new(71, -71),
      ),
      (
        arc_case("-270deg, 0 cl, 0   deg    ", -270.0, 0, true, 0),
        Vec2::new(100, 0),
      ),
      (
        arc_case("-270deg, 0 cl, 90  deg    ", -270.0, 0, true, 0),
        Vec2::new(0, 100),
      ),
      (
        arc_case("-270deg, 0 cl, 180 deg    ", -270.0, 0, true, 0),
        Vec2::new(-100, 0),
      ),
      (
        arc_case("-270deg, 0 cl, 270 deg    ", -270.0, 0, true, 0),
        Vec2::new(0, -100),
      ),
      (
        arc_case("-270deg, 0 cl, 45  deg    ", -270.0, 0, false, -1),
        Vec2::new(71, 71),
      ),
      (
        arc_case("-270deg, 0 cl, -45 deg    ", -270.0, 0, true, 0),
        Vec2::new(71, -71),
      ),
      (
        arc_case(" 270deg, 5 cl, 0   deg, 5 pos X", 270.0, 5, true, 5),
        Vec2::new(105, 0),
      ),
      (
        arc_case(" 270deg, 5 cl, 0  deg, 5 pos Y", 270.0, 5, true, 5),
        Vec2::new(100, -5),
      ),
      (
        arc_case(" 270deg, 5 cl, 90  deg, 5 pos", 270.0, 5, true, 5),
        Vec2::new(0, 105),
      ),
      (
        arc_case(" 270deg, 5 cl, 180 deg, 5 pos", 270.0, 5, true, 5),
        Vec2::new(-105, 0),
      ),
      (
        arc_case(" 270deg, 5 cl, 270 deg, 5 pos", 270.0, 5, true, 5),
        Vec2::new(0, -105),
      ),
      (
        arc_case(" 270deg, 5 cl, 0   deg, 5 neg", 270.0, 5, true, 5),
        Vec2::new(105, 0),
      ),
      (
        arc_case(" 270deg, 5 cl, 90  deg, 5 neg", 270.0, 5, true, 5),
        Vec2::new(0, 105),
      ),
      (
        arc_case(" 270deg, 5 cl, 180 deg, 5 neg", 270.0, 5, true, 5),
        Vec2::new(-105, 0),
      ),
      (
        arc_case(" 270deg, 5 cl, 270 deg, 5 neg", 270.0, 5, true, 5),
        Vec2::new(0, -105),
      ),
      (
        arc_case(" 270deg, 5 cl, 45  deg, 5 pos", 270.0, 5, true, 5),
        Vec2::new(74, 75),
      ),
      (
        arc_case(" 270deg, 5 cl, -45 deg, 5 pos", 270.0, 5, false, -1),
        Vec2::new(74, -75),
      ),
      (
        arc_case(" 270deg, 5 cl, 45  deg, 5 neg", 270.0, 5, true, 5),
        Vec2::new(67, 67),
      ),
      (
        arc_case(" 270deg, 5 cl, -45 deg, 5 neg", 270.0, 5, false, -1),
        Vec2::new(67, -67),
      ),
      (
        arc_case(" 270deg, 4 cl, 0   deg pos", 270.0, 4, false, -1),
        Vec2::new(105, 0),
      ),
      (
        arc_case(" 270deg, 4 cl, 90  deg pos", 270.0, 4, false, -1),
        Vec2::new(0, 105),
      ),
      (
        arc_case(" 270deg, 4 cl, 180 deg pos", 270.0, 4, false, -1),
        Vec2::new(-105, 0),
      ),
      (
        arc_case(" 270deg, 4 cl, 270 deg pos", 270.0, 4, false, -1),
        Vec2::new(0, -105),
      ),
      (
        quarter_case("  90deg, 0 cl,   0 deg    ", 90.0, 0, true, 0),
        Vec2::new(71, -71),
      ),
      (
        quarter_case("  90deg, 0 cl,  45 deg    ", 90.0, 0, true, 0),
        Vec2::new(100, 0),
      ),
      (
        quarter_case("  90deg, 0 cl,  90 deg    ", 90.0, 0, true, 0),
        Vec2::new(71, 71),
      ),
      (
        quarter_case("  90deg, 0 cl, 135 deg    ", 90.0, 0, false, -1),
        Vec2::new(0, -100),
      ),
      (
        quarter_case("  90deg, 0 cl, -45 deg    ", 90.0, 0, false, -1),
        Vec2::new(0, 100),
      ),
      (
        quarter_case(" -90deg, 0 cl,   0 deg    ", -90.0, 0, true, 0),
        Vec2::new(71, -71),
      ),
      (
        quarter_case(" -90deg, 0 cl,  45 deg    ", -90.0, 0, true, 0),
        Vec2::new(100, 0),
      ),
      (
        quarter_case(" -90deg, 0 cl,  90 deg    ", -90.0, 0, true, 0),
        Vec2::new(71, 71),
      ),
      (
        quarter_case(" -90deg, 0 cl, 135 deg    ", -90.0, 0, false, -1),
        Vec2::new(0, -100),
      ),
      (
        quarter_case(" -90deg, 0 cl, -45 deg    ", -90.0, 0, false, -1),
        Vec2::new(0, 100),
      ),
      (
        ArcCollideCase {
          name: "issue 11358 collide",
          center: Vec2::new(119_888_000, 60_452_000),
          start: Vec2::new(120_904_000, 60_452_000),
          central_angle: 360.0,
          clearance: 0,
          collides: true,
          distance: 0,
        },
        Vec2::new(120_395_500, 59_571_830),
      ),
      (
        ArcCollideCase {
          name: "issue 11358 dist",
          center: Vec2::new(119_888_000, 60_452_000),
          start: Vec2::new(120_904_000, 60_452_000),
          central_angle: 360.0,
          clearance: 100,
          collides: true,
          distance: 50,
        },
        Vec2::new(118_872_050, 60_452_000),
      ),
    ];

    for (case, point) in cases {
      let mut arc = ShapeArc::from_center_start_angle(
        case.center,
        case.start,
        Degrees::new(case.central_angle),
        0,
      );
      let zero_width = arc.collide_point(point, case.clearance);

      arc.set_width(case.clearance * 2);

      let wide = arc.collide_point(point, 0);

      check_collide_case(&case, zero_width, wide);
    }
  }

  /// A `270deg` or `-270deg` row of KiCad's `arc_pt_collide_cases`, which
  /// all share the centre `(0, 0)` and the start `(100, 0)`.
  fn arc_case(
    name: &'static str,
    central_angle: f64,
    clearance: i32,
    collides: bool,
    distance: i32,
  ) -> ArcCollideCase {
    ArcCollideCase {
      name,
      center: Vec2::new(0, 0),
      start: Vec2::new(100, 0),
      central_angle,
      clearance,
      collides,
      distance,
    }
  }

  /// A `90deg` row, whose start is the diagonal that matches the sign of
  /// the sweep.
  fn quarter_case(
    name: &'static str,
    central_angle: f64,
    clearance: i32,
    collides: bool,
    distance: i32,
  ) -> ArcCollideCase {
    ArcCollideCase {
      name,
      center: Vec2::new(0, 0),
      start: if central_angle > 0.0 {
        Vec2::new(71, -71)
      } else {
        Vec2::new(71, 71)
      },
      central_angle,
      clearance,
      collides,
      distance,
    }
  }

  /// `CollideSeg`, `test_shape_arc.cpp:799`, over the table at `:783`.
  ///
  /// The third block of KiCad's body (`:838`) checks the reported
  /// location, which is the candidate point the loop last wrote; see
  /// erratum E2 on [`ShapeArc::collide_seg`].
  #[test]
  fn collide_seg() {
    let cases: [(ArcCollideCase, Seg, Vec2); 10] = [
      (
        arc_case("0   deg    ", 270.0, 0, true, 0),
        Seg::new(Vec2::new(100, 0), Vec2::new(50, 0)),
        Vec2::new(100, 0),
      ),
      (
        arc_case("90  deg    ", 270.0, 0, true, 0),
        Seg::new(Vec2::new(0, 100), Vec2::new(0, 50)),
        Vec2::new(0, 100),
      ),
      (
        arc_case("180 deg    ", 270.0, 0, true, 0),
        Seg::new(Vec2::new(-100, 0), Vec2::new(-50, 0)),
        Vec2::new(-100, 0),
      ),
      (
        arc_case("270 deg    ", 270.0, 0, true, 0),
        Seg::new(Vec2::new(0, -100), Vec2::new(0, -50)),
        Vec2::new(0, -100),
      ),
      (
        arc_case("45  deg    ", 270.0, 0, true, 0),
        Seg::new(Vec2::new(71, 71), Vec2::new(35, 35)),
        Vec2::new(70, 70),
      ),
      (
        arc_case("-45 deg    ", 270.0, 0, false, -1),
        Seg::new(Vec2::new(71, -71), Vec2::new(35, -35)),
        Vec2::new(0, 0),
      ),
      (
        quarter_case("seg inside arc start", 90.0, 10, true, 10),
        Seg::new(Vec2::new(90, 0), Vec2::new(-35, 0)),
        Vec2::new(100, 0),
      ),
      (
        quarter_case("seg inside arc end", 90.0, 10, true, 10),
        Seg::new(Vec2::new(-35, 0), Vec2::new(90, 0)),
        Vec2::new(100, 0),
      ),
      (
        ArcCollideCase {
          name: "large diameter arc",
          center: Vec2::new(172_367_922, 82_282_076),
          start: Vec2::new(162_530_000, 92_120_000),
          central_angle: -45.0,
          clearance: 433_300,
          collides: true,
          distance: 433_268,
        },
        Seg::new(
          Vec2::new(162_096_732, 92_331_236),
          Vec2::new(162_096_732, 78_253_268),
        ),
        Vec2::new(162_530_000, 92_120_000),
      ),
      (
        ArcCollideCase {
          name: "upside down collide",
          center: Vec2::new(26_250_000, 16_520_000),
          start: Vec2::new(28_360_000, 16_520_000),
          central_angle: 90.0,
          clearance: 0,
          collides: true,
          distance: 0,
        },
        Seg::new(
          Vec2::new(27_545_249, 18_303_444),
          Vec2::new(27_545_249, 18_114_500),
        ),
        Vec2::new(27_545_249, 18_185_662),
      ),
    ];

    for (case, seg, location) in cases {
      let mut arc = ShapeArc::from_center_start_angle(
        case.center,
        case.start,
        Degrees::new(case.central_angle),
        0,
      );
      let zero_width = arc.collide_seg(&seg, case.clearance);

      if let Some(collision) = zero_width {
        assert_eq!(collision.location, location, "{}: location", case.name);
      }

      arc.set_width(case.clearance * 2);

      let wide = arc.collide_seg(&seg, 0);

      check_collide_case(&case, zero_width, wide);
    }
  }

  /// `CollideNearlyFlatArcDoesNotOverflow`, `test_shape_arc.cpp:1305`.
  /// The radius of this arc, taken from a PADS import crash, is past
  /// `INT_MAX / 2`, which is what the two chord branches of
  /// [`ShapeArc::collide_point`] and [`ShapeArc::collide_seg`] exist for.
  ///
  /// KiCad asserts only that nothing throws. Here the equivalent is that
  /// nothing panics, which in a debug build also covers every integer
  /// overflow on the way.
  #[test]
  fn collide_nearly_flat_arc_does_not_overflow() {
    let arc = ShapeArc::new(
      Vec2::new(68_208_364, -8000),
      Vec2::new(771_364, 500_000),
      Vec2::new(35_224_335, -7999),
      1_270_000,
    );

    assert!(arc.radius() >= f64::from(i32::MAX) / 2.0);

    let point = Vec2::new(35_224_298, -5381);

    arc.collide_point(point, 635_000);
    arc.collide_seg(
      &Seg::new(point, Vec2::new(35_696_364, -32_988_651)),
      635_000,
    );
  }

  /// The port's own: [`ShapeArc::collide_seg`] reports the last candidate
  /// that collided, not the nearest one. Erratum E2, reproduced on
  /// purpose; the decision is in `doc/log/2026-09-12.md`.
  ///
  /// The segment here runs radially away from a quarter circle, so every
  /// candidate KiCad builds collapses onto one of its two endpoints and
  /// the far one is evaluated last.
  #[test]
  fn collide_seg_reports_the_last_candidate_erratum_e2() {
    let arc = ShapeArc::new(
      Vec2::new(1_000_000, 0),
      Vec2::new(707_107, 707_107),
      Vec2::new(0, 1_000_000),
      0,
    );
    let seg =
      Seg::new(Vec2::new(900_000, 900_000), Vec2::new(1_200_000, 1_200_000));
    let clearance = 750_000;

    let near = arc.collide_point(seg.a, clearance).expect("the near end");
    let far = arc.collide_point(seg.b, clearance).expect("the far end");

    assert!(near.actual < far.actual, "{near:?} against {far:?}");

    let reported = arc.collide_seg(&seg, clearance).expect("collides");

    assert_eq!(reported, far);
    assert!(reported.actual > near.actual);
  }

  /// The port's own: [`ShapeArc::nearest_points_to_rect`] loses the half
  /// width clamp that [`ShapeArc::nearest_points_to_seg`] applies.
  /// Erratum E3, reproduced on purpose.
  ///
  /// The arc's edge overlaps the rectangle here, so the segment overload
  /// zeroes the distance while the rectangle overload recomputes it from
  /// the two points and reports the overshoot instead.
  #[test]
  fn nearest_points_to_rect_drops_the_width_clamp_erratum_e3() {
    let points = [
      Vec2::new(1_000_000, 0),
      Vec2::new(707_107, 707_107),
      Vec2::new(0, 1_000_000),
    ];
    let origin = Vec2::new(1_100_000, -50_000);
    let size = Vec2::new(200_000, 100_000);
    // The first segment of the rectangle's outline, which is the one the
    // minimum lands on.
    let side = Seg::new(origin, Vec2::new(origin.x + size.x, origin.y));

    let wide = ShapeArc::new(points[0], points[1], points[2], 400_000);

    // The side is 100 micrometres from the arc's centre line, well inside
    // the 200 micrometre half width, so the segment overload clamps to
    // zero and says the two overlap.
    assert_eq!(wide.nearest_points_to_seg(&side).squared_distance, 0);
    // The rectangle overload recomputes the distance from the two points
    // after the half width has already moved one of them, so it reports
    // the overshoot, about 88 micrometres, instead of the zero.
    assert_eq!(
      wide.nearest_points_to_rect(origin, size).squared_distance,
      7_778_593_474
    );

    // With no width there is nothing to clamp, so the same query answers
    // the true centre line gap of 100 micrometres.
    let thin = ShapeArc::new(points[0], points[1], points[2], 0);

    assert_eq!(
      thin.nearest_points_to_rect(origin, size).squared_distance,
      100_000 * 100_000
    );
  }

  /// The port's own: the four `nearest_points` methods put the point on
  /// the arc in `on_self` and the point on the other shape in
  /// `on_other`, including the rectangle overload, which reaches its
  /// answer through a dispatcher that swaps them twice
  /// (`shape_arc.cpp:642`).
  #[test]
  fn nearest_points_put_the_arc_point_first() {
    let arc = ShapeArc::new(
      Vec2::new(1_000_000, 0),
      Vec2::new(707_107, 707_107),
      Vec2::new(0, 1_000_000),
      0,
    );
    let far = Vec2::new(3_000_000, 0);

    let to_circle = arc.nearest_points_to_circle(far, 100_000);

    assert_eq!(to_circle.on_self, Vec2::new(1_000_000, 0));
    assert_eq!(to_circle.on_other, Vec2::new(2_900_000, 0));

    let to_seg = arc.nearest_points_to_seg(&Seg::new(
      Vec2::new(2_000_000, -1_000_000),
      Vec2::new(2_000_000, 1_000_000),
    ));

    assert_eq!(to_seg.on_self, Vec2::new(1_000_000, 0));
    assert_eq!(to_seg.on_other, Vec2::new(2_000_000, 0));

    let to_rect = arc.nearest_points_to_rect(
      Vec2::new(2_000_000, -100_000),
      Vec2::new(200_000, 200_000),
    );

    assert_eq!(to_rect.on_self, Vec2::new(1_000_000, 0));
    assert_eq!(to_rect.on_other, Vec2::new(2_000_000, 0));

    // Two quarter circles a few millimetres apart: the nearest pair is
    // one arc's endpoint against a point on the other's circle.
    let other = ShapeArc::new(
      Vec2::new(6_000_000, 0),
      Vec2::new(5_707_107, 707_107),
      Vec2::new(5_000_000, 1_000_000),
      0,
    );
    let to_arc = arc.nearest_points_to_arc(other);

    assert_eq!(to_arc.on_other, Vec2::new(5_000_000, 1_000_000));
    assert!(
      (i64::from(to_arc.on_self.euclidean_norm()) - 1_000_000).abs() <= 1,
      "{:?} is not on the arc's circle",
      to_arc.on_self
    );
  }

  /// The port's own: the two helpers the polyline builder rests on,
  /// against values taken straight from their formulas.
  #[test]
  fn segment_count_helpers_match_their_formulas() {
    // A 90 degree sweep of a 1 mm radius at the chain accuracy, 1000 nm.
    assert_eq!(
      arc_to_segment_count(1_000_000, 1000, Degrees::QUARTER_TURN),
      18
    );
    // The per segment angle is capped at an eighth of a turn, so a tiny
    // radius still gets eight segments for a full turn.
    assert_eq!(arc_to_segment_count(1, 1000, Degrees::FULL_TURN), 8);
    // Never fewer than two.
    assert_eq!(arc_to_segment_count(1_000_000, 1000, Degrees::new(1.0)), 2);

    // The segment count floors at three below which the quantity has no
    // meaning, so two and three give the same answer.
    assert_eq!(
      circle_to_end_segment_delta_radius(1_000_000, 2),
      circle_to_end_segment_delta_radius(1_000_000, 3)
    );
    // `|r * (1 - 1 / cos(pi / 8))|` for a 1 mm radius.
    assert_eq!(circle_to_end_segment_delta_radius(1_000_000, 8), 82392);
  }
}
