// SPDX-License-Identifier: GPL-3.0-or-later

//! The octagons the router walks around.
//!
//! A hull is the shape of an obstacle grown by the clearance and by half
//! the width of the line that has to get past it, so that the walkaround
//! can treat the moving line as a zero width polyline and still keep its
//! copper edge clear. Every hull in the router is an octagon, because an
//! octagon is the tightest convex shape whose edges all lie on the 45
//! degree grid the router routes on. All of this is
//! `pcbnew/router/pns_utils.cpp`; see `doc/reference/kicad/01-geometry.md`
//! section 12 and `doc/reference/kicad/03-router-placer-walkaround.md`
//! section 6.
//!
//! # The clockwise invariant
//!
//! Every builder here returns a **clockwise** chain, where clockwise means
//! on screen, with y growing downwards, so [`LineChain::area`] with
//! `absolute` unset comes out positive. `LINE::Walkaround` implements the
//! counter clockwise winding by reversing the hull rather than by walking
//! it differently (`pcbnew/router/pns_line.cpp:397`), so a hull that came
//! out the other way round would silently reverse the meaning of the
//! walkaround's `cw` flag. [`segment_hull`] normalises the winding
//! explicitly, the other two get it from the order they append their
//! vertices in.
//!
//! # What is not here
//!
//! - `ArcHull` (`pns_utils.cpp:71`): milestone 1 routes straight segments
//!   only, so there is no `Shape::Arc` to feed it. It is the one builder
//!   that miters offset lines rather than resizing a direction vector,
//!   and it brings `SHAPE_ARC::ConvertToPolyline` with it.
//! - `ChangedArea` (`pns_utils.cpp:369`, `:389`): a dispatch over item
//!   kinds that forwards to `VIA::ChangedArea` and `LINE::ChangedArea`,
//!   so it belongs to the item model.
//! - `NodeStats` (`pns_utils.cpp:544`): debug drawing.
//! - The compound union in `SOLID::Hull` (`pcbnew/router/pns_solid.cpp:55`)
//!   and `HOLE::Hull` (`pcbnew/router/pns_hole.cpp:84`), which is the
//!   router's only polygon boolean. It lives with the items, not here.
//!   See [`build_hull_for_primitive_shape`] for what the item model has
//!   to do instead.
//! - `PNS::ClipLine` does not exist in this revision. The only `ClipLine`
//!   in the tree is kimath's Cohen Sutherland box clipper
//!   (`libs/kimath/include/geometry/geometry_utils.h:238`), which the
//!   router never calls.

use crate::geometry::line_chain::{Hit, Intersection, LineChain};
use crate::geometry::math::{kiround, sign};
use crate::geometry::seg::Seg;
use crate::geometry::shape::{Shape, SimplePolygon};
use crate::geometry::vec2::{Vec2, Vec2L};
use std::f64::consts::{FRAC_1_SQRT_2, SQRT_2};

/// The slack the router leaves around a joint hull, in nanometres.
///
/// Port of the macro `PNS_HULL_MARGIN`,
/// `pcbnew/router/pns_line.h:45`. That is the definition the router
/// actually uses, at `pcbnew/router/pns_node.cpp:1338` and `:1348` for
/// the width of a joint's hull query and at
/// `pcbnew/router/pns_diff_pair_placer.cpp:247` for the forced clearance
/// of a diff pair shove.
///
/// `pns_utils.h:34` declares `constexpr int HULL_MARGIN = 10` with the
/// same value, and nothing reads it: the macro shadows it everywhere. The
/// one ported here is the macro, and the two agree anyway.
pub const HULL_MARGIN: i32 = 10;

/// The corner cut of an equilateral octagon inscribed in a square, as a
/// fraction of the square's half diagonal.
///
/// The `2.0 * ( 1.0 - M_SQRT1_2 )` that appears at
/// `pcbnew/router/pns_utils.cpp:82`, `:252` and `:501`. The grouping is
/// KiCad's: it multiplies this constant by the radius sum, so the two
/// roundings happen in the same order.
const OCTAGON_CHAMFER_FACTOR: f64 = 2.0 * (1.0 - FRAC_1_SQRT_2);

/// The ratio between an equilateral octagon's side and its apothem.
///
/// The `2.0 / ( 1.0 + M_SQRT2 )` at `pcbnew/router/pns_utils.cpp:86` and
/// `:188`, again with KiCad's grouping.
const OCTAGON_SIDE_RATIO: f64 = 2.0 / (1.0 + SQRT_2);

/// Clamp an `i64` into an `i32`.
///
/// [`crate::geometry::box2::Box2`] carries `i64` coordinates where
/// KiCad's `BOX2I` carries `int`, so the width and height of a bounding
/// box have to be narrowed before they can seed a diagonal. Saturating is
/// this crate's answer to KiCad's silent narrowing; it only fires on
/// boxes no board can produce.
fn saturate_i32(value: i64) -> i32 {
  value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// The rectangle `[origin, origin + size]` inflated by a clearance, with
/// each corner cut back by a chamfer along both axes.
///
/// Port of `PNS::OctagonalHull`,
/// `pcbnew/router/pns_utils.cpp:40`. This is the primitive every other
/// hull of a round or rectangular obstacle reduces to. A chamfer of zero
/// skips the four diagonal vertices (`:49`, `:54`, `:59`, `:64`), so the
/// result degenerates to a plain inflated rectangle.
///
/// The vertex order starts on the left edge just below the top left
/// chamfer and runs clockwise on screen, so [`LineChain::area`] with
/// `absolute` unset is positive for every non degenerate input.
///
/// The chain is closed before the first append, exactly as KiCad closes
/// it at `:45`, so a chamfer that reaches the far side of the rectangle
/// can collapse two vertices into one through [`LineChain::append`]'s
/// duplicate suppression, and an all zero input yields a single point.
///
/// # Panics
///
/// In a debug build when a vertex coordinate leaves `i32`. KiCad computes
/// these in wrapping `int`.
pub fn octagonal_hull(
  origin: Vec2,
  size: Vec2,
  clearance: i32,
  chamfer: i32,
) -> LineChain {
  let mut hull = LineChain::new();
  hull.set_closed(true);

  // KiCad spells each coordinate out in full, left to right, so the
  // intermediate sums are the same ones.
  let left = origin.x - clearance;
  let top = origin.y - clearance;
  let right = origin.x + size.x + clearance;
  let bottom = origin.y + size.y + clearance;

  hull.append(Vec2::new(left, top + chamfer));

  if chamfer != 0 {
    hull.append(Vec2::new(left + chamfer, top));
  }

  hull.append(Vec2::new(right - chamfer, top));

  if chamfer != 0 {
    hull.append(Vec2::new(right, top + chamfer));
  }

  hull.append(Vec2::new(right, bottom - chamfer));

  if chamfer != 0 {
    hull.append(Vec2::new(right - chamfer, bottom));
  }

  hull.append(Vec2::new(left + chamfer, bottom));

  if chamfer != 0 {
    hull.append(Vec2::new(left, bottom - chamfer));
  }

  hull
}

/// Whether a segment counts as lying on the 45 degree grid.
///
/// Port of the file local `IsSegment45Degree`,
/// `pcbnew/router/pns_utils.cpp:157`. The three tests are integer slop and
/// not angles: a segment passes when `|dx| <= 1`, when `|dy| <= 1`, or
/// when `|dx| - |dy|` is in `-1 ..= 1`. A one nanometre wide staircase
/// therefore counts as axis aligned, which is the point: it is the input
/// that [`segment_hull`]'s kink correction exists for.
///
/// # Panics
///
/// In a debug build when the endpoint difference leaves `i32`, where
/// KiCad wraps, and for a coordinate difference of exactly `i32::MIN`,
/// where `std::abs` is undefined in C++.
fn is_segment_45_degree(seg: &Seg) -> bool {
  let direction = seg.b - seg.a;

  if direction.x.abs() <= 1 {
    return true;
  }

  if direction.y.abs() <= 1 {
    return true;
  }

  let delta = direction.x.abs() - direction.y.abs();

  (-1..=1).contains(&delta)
}

/// The octagon around a capsule: a segment of the given full width with a
/// round cap at each end.
///
/// Port of `PNS::SegmentHull`,
/// `pcbnew/router/pns_utils.cpp:181`, the workhorse of the family.
/// `width` is `SHAPE_SEGMENT::GetWidth`, the full width of the capsule,
/// and `walkaround_thickness` is the width of the **moving** line, never
/// of the obstacle.
///
/// The geometry is four resized copies of the direction vector:
///
/// ```text
/// cl  = clearance + walkaround_thickness / 2
/// d   = width / 2 + cl                  (in f64, so a half nanometre survives)
/// dr  = round( d )                      the full offset
/// xr2 = round( OCTAGON_SIDE_RATIO * d / 2 )  the corner cut offset
/// ```
///
/// Note the half width rounding: this function truncates
/// `walkaround_thickness / 2` where `ArcHull` (`:73`) and
/// [`build_hull_for_primitive_shape`] (`:481`) round it up. The
/// difference is one nanometre and it is observable, so it is preserved;
/// see the note on [`build_hull_for_primitive_shape`].
///
/// # The kink correction
///
/// Most of the function straightens a segment that is too short for its
/// direction to be trustworthy (`:207` to `:248`), driven by
/// `kink_threshold = clearance / 10` (`:184`):
///
/// - not on the 45 degree grid and `0 < length <= kink_threshold`: the
///   end point is snapped to an exact diagonal,
///   `b = a + (sgn(w) * ll, sgn(h) * ll)` with `ll = max(|w|, |h|)`;
/// - on the grid and `length <= kink_threshold`, almost vertical
///   (`|w| <= 1`): `w = 0` and `cl += 1`;
/// - the same, almost horizontal (`|h| <= 1`): `h = 0` and `cl += 1`;
/// - the same, almost diagonal (`||w| - |h|| <= 2`): both components take
///   `sgn(.) * max(|w|, |h|)` and `cl += 2`.
///
/// The `cl` bumps are meant to pay for the error the snapping introduces,
/// and in this revision they almost never do: `d`, `dr` and `xr2` are all
/// computed at `:187` to `:190`, **before** the correction runs, so the
/// only path that reads the bumped `cl` is the degenerate one at `:250`,
/// where the corrected segment has collapsed to a point and the hull
/// becomes an octagon around a `width` by `width` square. That is
/// reproduced as it stands, bump placement included, because the
/// walkaround's termination depends on hulls and the collision predicates
/// agreeing to the nanometre.
///
/// # Panics
///
/// In a debug build when a vertex coordinate leaves `i32`, where KiCad
/// wraps.
pub fn segment_hull(
  seg: &Seg,
  width: i32,
  clearance: i32,
  walkaround_thickness: i32,
) -> LineChain {
  let kink_threshold = clearance / 10;

  let mut cl = clearance + walkaround_thickness / 2;
  let d = f64::from(width) / 2.0 + f64::from(cl);
  let x = OCTAGON_SIDE_RATIO * d;
  let dr = kiround(d);
  let xr2 = kiround(x / 2.0);

  let a = seg.a;
  let mut b = seg.b;
  let length = seg.length();
  let mut w = b.x - a.x;
  let mut h = b.y - a.y;

  if a != b {
    if !is_segment_45_degree(seg) {
      if length <= kink_threshold && length > 0 {
        let longest = w.abs().max(h.abs());

        b = a + Vec2::new(sign(w) * longest, sign(h) * longest);
      }
    } else if length <= kink_threshold {
      let delta45 = (w.abs() - h.abs()).abs();

      if w.abs() <= 1 {
        // Almost vertical.
        w = 0;
        cl += 1;
      } else if h.abs() <= 1 {
        // Almost horizontal.
        h = 0;
        cl += 1;
      } else if delta45 <= 2 {
        // Almost 45 degrees.
        let new_w = sign(w) * w.abs().max(h.abs());
        let new_h = sign(h) * w.abs().max(h.abs());

        w = new_w;
        h = new_h;
        cl += 2;
      }

      b.x = a.x + w;
      b.y = a.y + h;
    }
  }

  if a == b {
    let chamfer = kiround(OCTAGON_CHAMFER_FACTOR * d);

    return octagonal_hull(
      a - Vec2::new(width / 2, width / 2),
      Vec2::new(width, width),
      cl,
      chamfer,
    );
  }

  let direction = b - a;
  let p0 = direction.perpendicular().resize(dr);
  let ds = direction.perpendicular().resize(xr2);
  let pd = direction.resize(xr2);
  let dp = direction.resize(dr);

  let mut hull = LineChain::new();
  hull.set_closed(true);

  hull.append(b + p0 + pd);
  hull.append(b + dp + ds);
  hull.append(b + dp - ds);
  hull.append(b - p0 + pd);
  hull.append(a - p0 - pd);
  hull.append(a - dp - ds);
  hull.append(a - dp + ds);
  hull.append(a + p0 - pd);

  // Make sure the hull outline is always clockwise (`:282`).
  if hull.segment(0).side(a) < 0 {
    hull.reversed()
  } else {
    hull
  }
}

/// Slide a diagonal of [`convex_hull`] sideways until it is exactly
/// `clearance` away from the polygon vertex nearest to it.
///
/// Port of the file local `MoveDiagonal`,
/// `pcbnew/router/pns_utils.cpp:289`. "Nearest" is
/// [`LineChain::nearest_point_to_seg`], which measures **vertices**
/// against the infinite line through the diagonal, so a vertex in the
/// middle of an edge that comes closer is not seen. This is the only
/// caller of that routine, and it depends on exactly that behaviour.
///
/// The displacement is `perpendicular(a - b)` resized to
/// `distance - clearance`, and [`Vec2::resize`] reverses the direction
/// for a negative length, so a diagonal that already cuts closer than the
/// clearance is pushed outwards instead.
///
/// Deviation: KiCad reads an uninitialised `dist` when the vertex chain
/// is empty (`libs/kimath/src/geometry/shape_line_chain.cpp:2461` returns
/// before assigning it), which is undefined behaviour. Here an empty
/// chain leaves the diagonal alone. [`convex_hull`] returns before it can
/// happen either way.
fn move_diagonal(diagonal: &mut Seg, vertices: &LineChain, clearance: i32) {
  let Some((_, distance)) = vertices.nearest_point_to_seg(diagonal) else {
    return;
  };

  let move_by = (diagonal.a - diagonal.b)
    .perpendicular()
    .resize(distance - clearance);

  diagonal.a += move_by;
  diagonal.b += move_by;
}

/// The octagon around a polygon that is **assumed** convex.
///
/// Port of `PNS::ConvexHull`,
/// `pcbnew/router/pns_utils.cpp:300`. Despite the name this computes no
/// convex hull: the input is taken to be convex already, as it is for the
/// pad outlines and zone triangles the board interface hands the router
/// (`pcbnew/router/pns_kicad_iface.cpp:1734`, `:1933`). kimath's real
/// monotone chain `BuildConvexHull`
/// (`libs/kimath/src/geometry/convex_hull.cpp:83`) exists and the router
/// never calls it.
///
/// The construction takes the four edges of the polygon's bounding box
/// inflated by the clearance, adds a 45 degree line at each corner, slides
/// each of those inwards with `move_diagonal` until it is exactly
/// `clearance` from the nearest vertex, and reads the eight octagon
/// corners off the pairwise intersections of consecutive lines. The
/// diagonals are seeded with a length of `box.height()` on both axes
/// (`:319`, `:325`, `:331`, `:337`), an implicit assumption that the
/// height is enough to span the box; it does not matter for the result,
/// because the intersections are taken between infinite lines.
///
/// The append order is the same cycle as [`octagonal_hull`]'s, so the
/// result is clockwise on screen. KiCad does not normalise the winding
/// here the way [`segment_hull`] does, and does not need to.
///
/// Returns `None` for an empty polygon, which has no bounding box, and
/// wherever `SEG::IntersectLines` finds no intersection, which is where
/// KiCad dereferences an empty `std::optional` (`:343` and following).
/// The second case needs an intersection whose coordinates leave `i32`:
/// a degenerate input does **not** reach it, because a bounding box of
/// zero height shrinks the four diagonals to points that still sit on
/// their axis lines, and `IntersectLines` answers a degenerate collinear
/// pair with that point (`libs/kimath/src/geometry/seg.cpp:355`). Such an
/// input yields a hull of two or three coincident vertices rather than
/// nothing.
pub fn convex_hull(
  convex: &SimplePolygon,
  clearance: i32,
) -> Option<LineChain> {
  let bounding_box = convex.bbox(clearance)?;

  // KiCad narrows `BOX2I::GetHeight()` to an `int`; this box is `i64`
  // wide, so narrow it the same way and keep the diagonal endpoints in
  // `i64` until they are saturated, where KiCad lets them wrap.
  let height = saturate_i32(bounding_box.height());
  let min = bounding_box.origin().saturating_to_vec2();
  let max = bounding_box.end().saturating_to_vec2();

  // KiCad's names, which read upside down on screen: "top" is the edge at
  // the larger y (`:306`), which is the lower one when y grows downwards.
  let top_line = Seg::new(Vec2::new(min.x, max.y), max);
  let right_line = Seg::new(max, Vec2::new(max.x, min.y));
  let bottom_line = Seg::new(Vec2::new(max.x, min.y), min);
  let left_line = Seg::new(min, Vec2::new(min.x, max.y));

  let vertices = convex.vertices();

  /// The diagonal offset of a corner line, `(height, -height)` and its
  /// three sign variants at `pns_utils.cpp:319`, `:324`, `:331`, `:336`.
  fn offset(corner: Vec2, delta_x: i64, delta_y: i64) -> Vec2 {
    Vec2L::new(i64::from(corner.x) + delta_x, i64::from(corner.y) + delta_y)
      .saturating_to_vec2()
  }

  // Top right diagonal (`:317`).
  let corner = max;
  let mut top_right_line = Seg::new(
    corner,
    offset(corner, i64::from(height), -i64::from(height)),
  );
  move_diagonal(&mut top_right_line, vertices, clearance);

  // Bottom right diagonal (`:323`).
  let corner = Vec2::new(max.x, min.y);
  let mut bottom_right_line =
    Seg::new(offset(corner, i64::from(height), i64::from(height)), corner);
  move_diagonal(&mut bottom_right_line, vertices, clearance);

  // Bottom left diagonal (`:329`).
  let corner = min;
  let mut bottom_left_line = Seg::new(
    corner,
    offset(corner, -i64::from(height), i64::from(height)),
  );
  move_diagonal(&mut bottom_left_line, vertices, clearance);

  // Top left diagonal (`:335`).
  let corner = Vec2::new(min.x, max.y);
  let mut top_left_line = Seg::new(
    offset(corner, -i64::from(height), -i64::from(height)),
    corner,
  );
  move_diagonal(&mut top_left_line, vertices, clearance);

  let mut octagon = LineChain::new();
  octagon.set_closed(true);

  for (first, second) in [
    (&left_line, &bottom_left_line),
    (&bottom_line, &bottom_left_line),
    (&bottom_line, &bottom_right_line),
    (&right_line, &bottom_right_line),
    (&right_line, &top_right_line),
    (&top_line, &top_right_line),
    (&top_line, &top_left_line),
    (&left_line, &top_left_line),
  ] {
    octagon.append(first.intersect_lines(second)?);
  }

  Some(octagon)
}

/// The rectangle that contains a capsule, as a coarse over
/// approximation.
///
/// Port of `PNS::ApproximateSegmentAsRect`,
/// `pcbnew/router/pns_utils.cpp:356`. Both endpoints are inflated by
/// `width / 2` on both axes and the enclosing rectangle is normalised, so
/// the answer is right for an axis aligned capsule and much too large for
/// a diagonal one. The optimizer uses it as a cheap first pass when it
/// looks for a segment to replace (`pcbnew/router/pns_optimizer.cpp:1068`),
/// where over approximating is the safe direction.
///
/// The returned [`Shape`] is always a [`Shape::Rect`] with a zero corner
/// radius, as `SHAPE_RECT`'s four argument constructor leaves it
/// (`libs/kimath/include/geometry/shape_rect.h:61`).
pub fn approximate_segment_as_rect(seg: &Seg, width: i32) -> Shape {
  let delta = Vec2::new(width / 2, width / 2);
  let p0 = seg.a - delta;
  let p1 = seg.b + delta;

  Shape::rect(
    Vec2::new(p0.x.min(p1.x), p0.y.min(p1.y)),
    Vec2::new((p1.x - p0.x).abs(), (p1.y - p0.y).abs()),
  )
}

/// The hull of one primitive shape.
///
/// Port of `PNS::BuildHullForPrimitiveShape`,
/// `pcbnew/router/pns_utils.cpp:478`. `walkaround_thickness` is the width
/// of the moving line, never of the obstacle: the callers pass
/// `aLine.Width()` (`pcbnew/router/pns_walkaround.cpp:158`,
/// `pns_line_placer.cpp:822`, `pns_shove.cpp:1282`).
///
/// | shape | hull | KiCad |
/// |---|---|---|
/// | [`Shape::Rect`] | [`octagonal_hull`] with no chamfer | `:488` |
/// | [`Shape::Circle`] | [`octagonal_hull`] around the `2r` square, chamfered | `:498` |
/// | [`Shape::Segment`] | [`segment_hull`] with the **raw** arguments | `:507` |
/// | [`Shape::Simple`] | [`convex_hull`] with the combined clearance | `:520` |
/// | [`Shape::LineChain`], [`Shape::Compound`] | `None` | `:531` |
///
/// # Two roundings that differ on purpose
///
/// The combined clearance is `clearance + (walkaround_thickness + 1) / 2`,
/// rounded **up** (`:481`), and every branch uses it except the segment
/// one, which forwards the raw `clearance` and `walkaround_thickness` and
/// lets [`segment_hull`] recombine them with `walkaround_thickness / 2`,
/// rounded **down** (`:186`). For an odd line width the hull of a track
/// segment is therefore one nanometre tighter than the hull of a via or a
/// pad at the same clearance. That is note 03 section 9.6 item 9, and it
/// is reproduced rather than fixed.
///
/// The circle's chamfer is `OCTAGON_CHAMFER_FACTOR * (radius + cl)`
/// **truncated** by C++'s implicit `double` to `int` conversion at the
/// call (`:501`), where the same expression inside [`segment_hull`]
/// (`:252`) is rounded through `KiROUND`. Both are kept as they stand.
///
/// # Compounds
///
/// KiCad's dispatcher has no `SH_COMPOUND` case: it asserts and returns
/// an empty chain (`:531`). The compound aware builders are
/// `SOLID::Hull` (`pcbnew/router/pns_solid.cpp:39`) and `HOLE::Hull`
/// (`pcbnew/router/pns_hole.cpp:57`), which unwrap a single element
/// compound and otherwise union the per primitive hulls through a
/// `SHAPE_POLY_SET`. That union is the router's only polygon boolean and
/// it belongs to the item model, so it is not here. The item model gets
/// what it needs from [`Shape::subshapes`], which returns the leaves of a
/// compound in order and the shape itself for anything else: mapping this
/// function over that list reproduces both of KiCad's branches, since a
/// one element list needs no union.
///
/// `SH_ELLIPSE` (`:523`), which degrades to its bounding box, has no
/// counterpart because [`Shape`] has no ellipse: the router's world model
/// never contains one, it only ever arrives through this dispatcher.
pub fn build_hull_for_primitive_shape(
  shape: &Shape,
  clearance: i32,
  walkaround_thickness: i32,
) -> Option<LineChain> {
  let cl = clearance + (walkaround_thickness + 1) / 2;

  match shape {
    Shape::Rect { origin, size, .. } => {
      Some(octagonal_hull(*origin, *size, cl, 0))
    }
    Shape::Circle { center, radius } => {
      let radius = *radius;
      // The implicit `double` to `int` conversion of `:501`, which
      // truncates towards zero.
      let chamfer = (OCTAGON_CHAMFER_FACTOR * f64::from(radius + cl)) as i32;

      Some(octagonal_hull(
        *center - Vec2::new(radius, radius),
        Vec2::new(2 * radius, 2 * radius),
        cl,
        chamfer,
      ))
    }
    Shape::Segment { seg, width } => {
      Some(segment_hull(seg, *width, clearance, walkaround_thickness))
    }
    Shape::Simple(convex) => convex_hull(convex, cl),
    Shape::LineChain(_) | Shape::Compound(_) => None,
  }
}

/// The points where a line crosses a hull, with the merely touching ones
/// filtered out.
///
/// Port of `PNS::HullIntersection`,
/// `pcbnew/router/pns_utils.cpp:395`, the filter between
/// [`LineChain::intersect_chain`] and the walkaround. Without it a line
/// that grazes a hull vertex or runs along a hull edge would seed the
/// walkaround's traversal graph with vertices it cannot leave.
///
/// The rule, per raw intersection:
///
/// - a hit that is a corner of **neither** chain passes unconditionally
///   (`:416`);
/// - otherwise up to two hull segments and up to two neighbouring line
///   points are collected around the hit, and the record is kept only if
///   some segment has some point strictly on its positive side (`:455`).
///   [`Seg::side`] is positive to the right of the directed segment in
///   screen coordinates, which for a clockwise hull is its **inner**
///   side, so the test asks whether the line has anywhere to go into the
///   hull. It is a half plane test per edge and not a containment test,
///   so it only rejects a hit whose neighbouring line points all sit in
///   the outer wedge of the corner itself. That is enough to keep a line
///   that merely grazes a vertex out of the walkaround's traversal
///   graph, which is what the filter is for. Note 03 section 6 describes
///   the sign the other way round; the code is what is reproduced here,
///   and the tests pin it.
///
/// # The `Hit` mapping
///
/// KiCad carries `index_our` plus `is_corner_our` and pairs them with a
/// `valid` flag it writes here and filters on. This crate folds the first
/// pair into [`Hit`] and drops the flag by returning only the records
/// KiCad would have kept, which is what `pcbnew/router/pns_node.cpp:396`
/// and `pcbnew/router/pns_line.cpp:341` see anyway: neither ever meets an
/// invalid record.
///
/// The translation of the index logic:
///
/// - `Hit::Segment(index)` is `is_corner == false`, and `index` is a
///   segment index. `d1` is the single hull segment at `index`; `d2` is
///   the two endpoints of the line segment at `index`.
/// - `Hit::Corner(index)` is `is_corner == true`, and `index` is a
///   **point** index that is already in range, because [`Hit`] folds the
///   `index + 1` of a hit on a segment's far endpoint back into the
///   chain. KiCad has to undo that by hand, `if( p.index_our >=
///   hull.SegmentCount() ) p.index_our -= hull.SegmentCount();` at
///   `:423`, and that statement has no counterpart here. `d1` is the hull
///   segment starting at the corner and the one ending there, the second
///   being KiCad's `CSegment( index_our - 1 )` with its wrap for
///   `index == 0`. `d2` is the neighbouring line point on each side that
///   exists, KiCad's `:440` and `:444` guards.
///
/// Deviation: KiCad copies the record at `:413`, before it wraps
/// `index_our` at `:423`, so the record it pushes keeps the **unwrapped**
/// index, which can equal `SegmentCount()` and index nothing. Here the
/// wrapped point index is what comes out. No consumer reads that field:
/// the node reads `index_their` and the point, the walkaround reads only
/// the point.
///
/// # Order
///
/// The records come out in the order [`LineChain::intersect_chain`]
/// produced them, which is the hull's segments outermost and the line's
/// segments innermost. KiCad's raw order differs in the inner loop, which
/// it walks sorted by minimum x. Neither consumer depends on it: the
/// walkaround splits both chains at every point, which is independent of
/// the order (`pcbnew/router/pns_line.cpp:367`), and the node keeps the
/// record with the shortest path length (`pcbnew/router/pns_node.cpp:400`).
///
/// # Panics
///
/// In a debug build when `hull` is an open chain whose last point is the
/// corner of a hit, because the hull segment starting there does not
/// exist. Every builder in this module returns a closed chain.
pub fn hull_intersection(
  hull: &LineChain,
  line: &LineChain,
) -> Vec<Intersection> {
  if line.point_count() < 2 {
    return Vec::new();
  }

  let segment_count = hull.segment_count();
  let point_count = line.point_count();
  let mut kept = Vec::new();

  // `aExcludeColinearAndTouching` keeps its KiCad default of false, which
  // this crate spells the right way round (`:403`).
  for intersection in hull.intersect_chain(line, true) {
    let Some(theirs) = intersection.theirs else {
      continue;
    };

    if !intersection.ours.is_corner() && !theirs.is_corner() {
      kept.push(intersection);
      continue;
    }

    let mut hull_segments = Vec::with_capacity(2);

    match intersection.ours {
      Hit::Corner(index) => {
        hull_segments.push(hull.segment(index));
        // KiCad's `CSegment( index_our - 1 )`, whose negative index wraps
        // by `SegmentCount()`.
        let previous = if index == 0 {
          segment_count - 1
        } else {
          index - 1
        };
        hull_segments.push(hull.segment(previous));
      }
      Hit::Segment(index) => hull_segments.push(hull.segment(index)),
    }

    let mut line_points = Vec::with_capacity(2);

    match theirs {
      Hit::Corner(index) => {
        if index > 0 {
          line_points.push(line.segment(index - 1).a);
        }

        if index < point_count - 1 {
          line_points.push(line.segment(index).b);
        }
      }
      Hit::Segment(index) => {
        let segment = line.segment(index);

        line_points.push(segment.a);
        line_points.push(segment.b);
      }
    }

    let crosses = hull_segments
      .iter()
      .any(|segment| line_points.iter().any(|point| segment.side(*point) > 0));

    if crosses {
      kept.push(intersection);
    }
  }

  kept
}

#[cfg(test)]
mod tests {
  use super::*;

  /// A chain built from a flat list of coordinates, closed.
  fn closed_chain(coordinates: &[i32]) -> LineChain {
    LineChain::from_points(
      coordinates
        .chunks_exact(2)
        .map(|pair| Vec2::new(pair[0], pair[1]))
        .collect(),
      true,
    )
  }

  /// A chain built from a flat list of coordinates, open.
  fn open_chain(coordinates: &[i32]) -> LineChain {
    LineChain::from_points(
      coordinates
        .chunks_exact(2)
        .map(|pair| Vec2::new(pair[0], pair[1]))
        .collect(),
      false,
    )
  }

  /// A polygon built from a flat list of coordinates.
  fn polygon(coordinates: &[i32]) -> SimplePolygon {
    SimplePolygon::new(closed_chain(coordinates))
  }

  /// The chain's points as a flat list, for comparing against a table.
  fn flatten(chain: &LineChain) -> Vec<i32> {
    chain
      .points()
      .iter()
      .flat_map(|point| [point.x, point.y])
      .collect()
  }

  // -----------------------------------------------------------------
  // OctagonalHull
  // -----------------------------------------------------------------

  #[test]
  fn octagonal_hull_without_a_chamfer_is_an_inflated_rectangle() {
    let hull = octagonal_hull(Vec2::new(10, 20), Vec2::new(100, 50), 5, 0);

    assert!(hull.is_closed());
    assert_eq!(flatten(&hull), vec![5, 15, 115, 15, 115, 75, 5, 75]);
  }

  #[test]
  fn octagonal_hull_with_a_chamfer_has_eight_vertices() {
    let hull = octagonal_hull(Vec2::new(0, 0), Vec2::new(100, 50), 10, 7);

    assert_eq!(
      flatten(&hull),
      vec![
        -10, -3, -3, -10, 103, -10, 110, -3, 110, 53, 103, 60, -3, 60, -10, 53
      ]
    );
  }

  #[test]
  fn octagonal_hull_of_a_zero_size_rectangle_is_a_chamfered_square() {
    let hull = octagonal_hull(Vec2::new(0, 0), Vec2::new(0, 0), 10, 3);

    assert_eq!(
      flatten(&hull),
      vec![
        -10, -7, -7, -10, 7, -10, 10, -7, 10, 7, 7, 10, -7, 10, -10, 7
      ]
    );
  }

  #[test]
  fn octagonal_hull_of_nothing_at_all_collapses_to_one_point() {
    // Every vertex lands on the origin and `append` suppresses the
    // duplicates, exactly as KiCad's does.
    let hull = octagonal_hull(Vec2::new(4, 5), Vec2::new(0, 0), 0, 0);

    assert_eq!(flatten(&hull), vec![4, 5]);
  }

  // -----------------------------------------------------------------
  // IsSegment45Degree
  // -----------------------------------------------------------------

  #[test]
  fn is_segment_45_degree_accepts_one_nanometre_of_slop() {
    let cases = [
      // Exactly axis aligned and exactly diagonal.
      ((100, 0), true),
      ((0, 100), true),
      ((100, 100), true),
      ((100, -100), true),
      // One nanometre off an axis, and two, which is too far.
      ((100, 1), true),
      ((100, 2), false),
      ((1, 100), true),
      ((2, 100), false),
      // One nanometre off the diagonal, and two, which is too far.
      ((100, 99), true),
      ((100, 98), false),
      ((99, -100), true),
      ((98, -100), false),
      // Degenerate.
      ((0, 0), true),
    ];

    for ((dx, dy), expected) in cases {
      let seg = Seg::new(Vec2::new(7, 9), Vec2::new(7 + dx, 9 + dy));

      assert_eq!(
        is_segment_45_degree(&seg),
        expected,
        "difference ({dx}, {dy})"
      );
    }
  }

  // -----------------------------------------------------------------
  // SegmentHull
  // -----------------------------------------------------------------

  #[test]
  fn segment_hull_horizontal() {
    let hull = segment_hull(&Seg::from_coords(0, 0, 1000, 0), 200, 100, 80);

    assert_eq!(
      flatten(&hull),
      vec![
        -99, 240, -240, 99, -240, -99, -99, -240, 1099, -240, 1240, -99, 1240,
        99, 1099, 240
      ]
    );
  }

  #[test]
  fn segment_hull_vertical() {
    let hull = segment_hull(&Seg::from_coords(0, 0, 0, 1000), 200, 100, 80);

    assert_eq!(
      flatten(&hull),
      vec![
        -240, -99, -99, -240, 99, -240, 240, -99, 240, 1099, 99, 1240, -99,
        1240, -240, 1099
      ]
    );
  }

  #[test]
  fn segment_hull_exact_diagonal() {
    let hull = segment_hull(&Seg::from_coords(0, 0, 1000, 1000), 200, 100, 80);

    assert_eq!(
      flatten(&hull),
      vec![
        -240, 100, -240, -100, -100, -240, 100, -240, 1240, 900, 1240, 1100,
        1100, 1240, 900, 1240
      ]
    );
  }

  #[test]
  fn segment_hull_general_angle() {
    // Length 557, kink threshold 10, so nothing is straightened.
    let hull = segment_hull(&Seg::from_coords(0, 0, 300, 470), 200, 100, 80);

    assert_eq!(
      flatten(&hull),
      vec![
        -255, 46, -212, -149, -46, -255, 149, -212, 555, 424, 512, 619, 346,
        725, 151, 682
      ]
    );
  }

  #[test]
  fn segment_hull_zero_length_is_an_octagon_around_a_square() {
    let hull =
      segment_hull(&Seg::from_coords(100, 100, 100, 100), 200, 100, 80);

    assert_eq!(
      flatten(&hull),
      vec![
        -140, 1, 1, -140, 199, -140, 340, 1, 340, 199, 199, 340, 1, 340, -140,
        199
      ]
    );
  }

  #[test]
  fn segment_hull_almost_vertical_kink_snaps_below_the_threshold() {
    // A one nanometre kink off vertical. Length 50, and the threshold is
    // `clearance / 10`, so the correction fires at a clearance of 1000
    // and not at 100. Below the threshold the hull is exactly the hull of
    // the straightened, vertical segment.
    let kinked = Seg::from_coords(0, 0, 1, 50);
    let snapped = Seg::from_coords(0, 0, 0, 50);

    let corrected = segment_hull(&kinked, 200, 1000, 80);

    assert_eq!(
      flatten(&corrected),
      vec![
        -1140, -472, -472, -1140, 472, -1140, 1140, -472, 1140, 522, 472, 1190,
        -472, 1190, -1140, 522
      ]
    );
    assert_eq!(
      flatten(&corrected),
      flatten(&segment_hull(&snapped, 200, 1000, 80))
    );

    // Above the threshold the kink survives, so the hull is tilted.
    let untouched = segment_hull(&kinked, 200, 100, 80);

    assert_eq!(
      flatten(&untouched),
      vec![
        -242, -94, -104, -238, 94, -242, 238, -104, 243, 144, 105, 288, -93,
        292, -237, 154
      ]
    );
    assert_ne!(
      flatten(&untouched),
      flatten(&segment_hull(&snapped, 200, 100, 80))
    );
  }

  #[test]
  fn segment_hull_almost_diagonal_kink_snaps_below_the_threshold() {
    // `|w| - |h|` is one, so the segment counts as 45 degrees and the
    // `delta45 <= 2` branch fires. Length 57, so the threshold is crossed
    // between a clearance of 500 and 600.
    let kinked = Seg::from_coords(0, 0, 40, 41);
    let snapped = Seg::from_coords(0, 0, 41, 41);

    let corrected = segment_hull(&kinked, 200, 600, 80);

    assert_eq!(
      flatten(&corrected),
      vec![
        -740, 306, -740, -306, -306, -740, 306, -740, 781, -265, 781, 347, 347,
        781, -265, 781
      ]
    );
    assert_eq!(
      flatten(&corrected),
      flatten(&segment_hull(&snapped, 200, 600, 80))
    );

    let untouched = segment_hull(&kinked, 200, 500, 80);

    assert_eq!(
      flatten(&untouched),
      vec![
        -643, 257, -637, -273, -257, -643, 273, -637, 683, -216, 677, 314, 297,
        684, -233, 678
      ]
    );
  }

  #[test]
  fn segment_hull_off_grid_kink_snaps_to_an_exact_diagonal() {
    // Not on the 45 degree grid: the whole end point is replaced, and no
    // clearance bump is applied.
    let kinked = Seg::from_coords(0, 0, 40, 10);
    let snapped = Seg::from_coords(0, 0, 40, 40);

    let corrected = segment_hull(&kinked, 200, 500, 80);

    assert_eq!(
      flatten(&corrected),
      vec![
        -640, 266, -640, -266, -266, -640, 266, -640, 680, -226, 680, 306, 306,
        680, -226, 680
      ]
    );
    assert_eq!(
      flatten(&corrected),
      flatten(&segment_hull(&snapped, 200, 500, 80))
    );
  }

  #[test]
  fn segment_hull_clearance_bump_only_reaches_the_degenerate_octagon() {
    // A one nanometre segment on the grid: the correction zeroes the only
    // non zero component, the segment collapses onto its start point, and
    // the `cl++` that `:226` applies is what the octagon is built with.
    // The same input one nanometre above the threshold keeps its length.
    let seg = Seg::from_coords(0, 0, 1, 0);

    let bumped = segment_hull(&seg, 20, 100, 0);
    let unbumped = octagonal_hull(
      Vec2::new(-10, -10),
      Vec2::new(20, 20),
      100,
      kiround(OCTAGON_CHAMFER_FACTOR * 110.0),
    );

    assert_eq!(
      flatten(&bumped),
      flatten(&octagonal_hull(
        Vec2::new(-10, -10),
        Vec2::new(20, 20),
        101,
        kiround(OCTAGON_CHAMFER_FACTOR * 110.0),
      ))
    );
    assert_ne!(flatten(&bumped), flatten(&unbumped));
  }

  // -----------------------------------------------------------------
  // MoveDiagonal
  // -----------------------------------------------------------------

  #[test]
  fn move_diagonal_slides_inwards_to_the_clearance() {
    // A 45 degree line through the origin and a single vertex at (100,
    // 100), whose distance to that line is 100 * sqrt(2) = 141. Asking
    // for a clearance of 41 moves the line 100 nanometres towards the
    // vertex, along (1, 1) normalised.
    let vertices = closed_chain(&[100, 100]);
    let mut diagonal = Seg::from_coords(0, 0, -100, 100);

    move_diagonal(&mut diagonal, &vertices, 41);

    assert_eq!(diagonal.a, Vec2::new(71, 71));
    assert_eq!(diagonal.b, Vec2::new(-29, 171));
  }

  #[test]
  fn move_diagonal_with_a_clearance_past_the_vertex_slides_outwards() {
    // `Vec2::resize` reverses direction for a negative length, so a
    // clearance larger than the distance pushes the line away.
    let vertices = closed_chain(&[100, 100]);
    let mut diagonal = Seg::from_coords(0, 0, -100, 100);

    move_diagonal(&mut diagonal, &vertices, 241);

    assert_eq!(diagonal.a, Vec2::new(-71, -71));
    assert_eq!(diagonal.b, Vec2::new(-171, 29));
  }

  #[test]
  fn move_diagonal_measures_against_the_infinite_line() {
    // The vertex sits a thousand nanometres past the end of the diagonal
    // and only fifty nanometres from the infinite line through it.
    // `NearestPoint( SEG )` measures with `SEG::LineDistance`, so fifty
    // is what the move uses; the distance to the segment itself is
    // twenty times that. `PNS::MoveDiagonal` is that routine's only
    // caller and depends on exactly this.
    let vertices = closed_chain(&[1000, 50]);
    let mut diagonal = Seg::from_coords(0, 0, 10, 0);

    move_diagonal(&mut diagonal, &vertices, 0);

    assert_eq!(diagonal.a, Vec2::new(0, -50));
    assert_eq!(diagonal.b, Vec2::new(10, -50));
  }

  #[test]
  fn move_diagonal_leaves_an_empty_chain_alone() {
    let mut diagonal = Seg::from_coords(0, 0, 100, 100);
    let before = diagonal;

    move_diagonal(&mut diagonal, &LineChain::new(), 10);

    assert_eq!(diagonal, before);
  }

  // -----------------------------------------------------------------
  // ConvexHull
  // -----------------------------------------------------------------

  #[test]
  fn convex_hull_of_a_rectangle() {
    let hull =
      convex_hull(&polygon(&[0, 0, 100, 0, 100, 60, 0, 60]), 10).unwrap();

    assert!(hull.is_closed());
    assert_eq!(
      flatten(&hull),
      vec![
        -10, -4, -4, -10, 104, -10, 110, -4, 110, 64, 104, 70, -4, 70, -10, 64
      ]
    );
  }

  #[test]
  fn convex_hull_of_a_concave_polygon_cuts_the_notch_off() {
    // The input is assumed convex and this one is not, so the top right
    // diagonal is stopped by the reflex vertex at (40, 40) and the hull
    // slices straight across the notch. KiCad has the same behaviour; the
    // router only ever feeds it pad outlines and zone triangles.
    let hull = convex_hull(
      &polygon(&[0, 0, 100, 0, 100, 40, 40, 40, 40, 100, 0, 100]),
      10,
    )
    .unwrap();

    assert_eq!(
      flatten(&hull),
      vec![
        -10, -4, -4, -10, 104, -10, 110, -4, 110, 44, 44, 110, -4, 110, -10,
        104
      ]
    );
  }

  #[test]
  fn convex_hull_scales_with_the_clearance() {
    let hull =
      convex_hull(&polygon(&[0, 0, 100, 0, 100, 60, 0, 60]), 25).unwrap();

    assert_eq!(
      flatten(&hull),
      vec![
        -25, -11, -11, -25, 111, -25, 125, -11, 125, 71, 111, 85, -11, 85, -25,
        71
      ]
    );
  }

  #[test]
  fn convex_hull_of_a_flat_polygon_collapses_to_a_line() {
    // With no clearance the bounding box has zero height, so all four
    // diagonals shrink to points sitting on the box corners. Every pair
    // of lines is then degenerate and collinear, which
    // `SEG::IntersectLines` answers with the degenerate point itself, so
    // the hull comes out as the two box corners with the first one
    // repeated: `append` only suppresses a duplicate of the previous
    // point, and the chain was closed before the first append.
    let hull = convex_hull(&polygon(&[0, 0, 100, 0]), 0).unwrap();

    assert_eq!(flatten(&hull), vec![0, 0, 100, 0, 0, 0]);

    // One nanometre of clearance gives it an area again. The box is only
    // two nanometres tall, so the four diagonals still land on its
    // corners and the octagon collapses back to a rectangle.
    let inflated = convex_hull(&polygon(&[0, 0, 100, 0]), 1).unwrap();

    assert_eq!(flatten(&inflated), vec![-1, -1, 101, -1, 101, 1, -1, 1]);
  }

  #[test]
  fn convex_hull_of_an_empty_polygon_is_none() {
    assert!(convex_hull(&SimplePolygon::default(), 10).is_none());
  }

  // -----------------------------------------------------------------
  // ApproximateSegmentAsRect
  // -----------------------------------------------------------------

  #[test]
  fn approximate_segment_as_rect_covers_the_capsule() {
    let rect = approximate_segment_as_rect(&Seg::from_coords(0, 0, 100, 0), 20);

    assert_eq!(rect, Shape::rect(Vec2::new(-10, -10), Vec2::new(120, 20)));

    // Backwards, so the normalisation has something to do.
    let backwards =
      approximate_segment_as_rect(&Seg::from_coords(100, 0, 0, 0), 20);

    // Both endpoints are inflated outwards, so a reversed segment gives
    // the box `[(10, -10), (90, 10)]`: the two inflations eat into each
    // other instead of adding up.
    assert_eq!(
      backwards,
      Shape::rect(Vec2::new(10, -10), Vec2::new(80, 20))
    );

    // A diagonal is over approximated: the rectangle is the square that
    // contains both inflated endpoints, not the capsule.
    let diagonal =
      approximate_segment_as_rect(&Seg::from_coords(0, 0, 100, 100), 20);

    assert_eq!(
      diagonal,
      Shape::rect(Vec2::new(-10, -10), Vec2::new(120, 120))
    );
  }

  // -----------------------------------------------------------------
  // BuildHullForPrimitiveShape
  // -----------------------------------------------------------------

  #[test]
  fn primitive_hull_of_a_rectangle_is_a_plain_inflated_rectangle() {
    let shape = Shape::rect(Vec2::new(0, 0), Vec2::new(100, 50));
    let hull = build_hull_for_primitive_shape(&shape, 20, 9).unwrap();

    // `cl` is `20 + (9 + 1) / 2`, that is 25, and the chamfer is zero.
    assert_eq!(flatten(&hull), vec![-25, -25, 125, -25, 125, 75, -25, 75]);
    assert_eq!(
      flatten(&hull),
      flatten(&octagonal_hull(Vec2::new(0, 0), Vec2::new(100, 50), 25, 0))
    );
  }

  #[test]
  fn primitive_hull_of_a_circle_truncates_its_chamfer() {
    let shape = Shape::circle(Vec2::new(0, 0), 50);
    let hull = build_hull_for_primitive_shape(&shape, 20, 9).unwrap();

    // The chamfer is `2 * (1 - 1/sqrt(2)) * (50 + 25)`, that is 43.93,
    // truncated by C++'s implicit conversion to 43 and not rounded to 44.
    assert_eq!((OCTAGON_CHAMFER_FACTOR * 75.0) as i32, 43);
    assert_eq!(kiround(OCTAGON_CHAMFER_FACTOR * 75.0), 44);
    assert_eq!(
      flatten(&hull),
      vec![
        -75, -32, -32, -75, 32, -75, 75, -32, 75, 32, 32, 75, -32, 75, -75, 32
      ]
    );
  }

  #[test]
  fn primitive_hull_of_a_segment_gets_the_raw_arguments() {
    let seg = Seg::from_coords(0, 0, 1000, 0);
    let shape = Shape::segment(seg, 200);

    let hull = build_hull_for_primitive_shape(&shape, 20, 9).unwrap();

    // The dispatcher forwards the raw clearance and thickness, so the
    // effective clearance is `20 + 9 / 2 = 24`, one nanometre less than
    // the `20 + (9 + 1) / 2 = 25` every other branch uses. Note 03
    // section 9.6 item 9.
    assert_eq!(flatten(&hull), flatten(&segment_hull(&seg, 200, 20, 9)));
    assert_ne!(flatten(&hull), flatten(&segment_hull(&seg, 200, 25, 0)));
    assert_eq!(
      flatten(&hull),
      vec![
        -51, 124, -124, 51, -124, -51, -51, -124, 1051, -124, 1124, -51, 1124,
        51, 1051, 124
      ]
    );
  }

  #[test]
  fn primitive_hull_of_a_polygon_uses_the_combined_clearance() {
    let convex = polygon(&[0, 0, 100, 0, 100, 60, 0, 60]);
    let shape = Shape::Simple(convex.clone());

    let hull = build_hull_for_primitive_shape(&shape, 20, 9).unwrap();

    assert_eq!(flatten(&hull), flatten(&convex_hull(&convex, 25).unwrap()));
  }

  #[test]
  fn primitive_hull_of_a_line_chain_or_a_compound_is_none() {
    let chain = Shape::line_chain(open_chain(&[0, 0, 100, 0]));

    assert!(build_hull_for_primitive_shape(&chain, 20, 9).is_none());

    // KiCad's dispatcher has no compound case either. The item model
    // unwraps the compound itself and unions the per primitive hulls.
    let compound = Shape::compound(vec![
      Shape::rect(Vec2::new(0, 0), Vec2::new(100, 50)),
      Shape::circle(Vec2::new(200, 0), 50),
    ]);

    assert!(build_hull_for_primitive_shape(&compound, 20, 9).is_none());

    // What the item model gets instead: one hull per leaf, in order.
    let per_leaf: Vec<Option<LineChain>> = compound
      .subshapes()
      .iter()
      .map(|leaf| build_hull_for_primitive_shape(leaf, 20, 9))
      .collect();

    assert_eq!(per_leaf.len(), 2);
    assert!(per_leaf.iter().all(Option::is_some));

    // A single element compound needs no union at all, which is the
    // branch `SOLID::Hull` spells out at `pns_solid.cpp:48`.
    let single =
      Shape::compound(vec![Shape::rect(Vec2::new(0, 0), Vec2::new(100, 50))]);

    assert_eq!(
      single
        .subshapes()
        .iter()
        .map(|leaf| build_hull_for_primitive_shape(leaf, 20, 9))
        .collect::<Vec<_>>(),
      vec![build_hull_for_primitive_shape(
        &Shape::rect(Vec2::new(0, 0), Vec2::new(100, 50)),
        20,
        9
      )]
    );
  }

  // -----------------------------------------------------------------
  // The clockwise invariant
  // -----------------------------------------------------------------

  #[test]
  fn every_hull_kind_winds_clockwise() {
    // `LineChain::area` negates the shoelace sum, so a chain that winds
    // clockwise on screen, with y growing downwards, comes out positive.
    // `LINE::Walkaround` reverses the hull to walk counter clockwise, so
    // this is the invariant the whole walkaround rests on.
    let mut hulls: Vec<(&str, LineChain)> = vec![
      (
        "octagon, no chamfer",
        octagonal_hull(Vec2::new(10, 20), Vec2::new(100, 50), 5, 0),
      ),
      (
        "octagon, chamfered",
        octagonal_hull(Vec2::new(0, 0), Vec2::new(100, 50), 10, 7),
      ),
      (
        "octagon, zero size",
        octagonal_hull(Vec2::new(0, 0), Vec2::new(0, 0), 10, 3),
      ),
      (
        "convex hull, rectangle",
        convex_hull(&polygon(&[0, 0, 100, 0, 100, 60, 0, 60]), 10).unwrap(),
      ),
      (
        "convex hull, concave input",
        convex_hull(
          &polygon(&[0, 0, 100, 0, 100, 40, 40, 40, 40, 100, 0, 100]),
          10,
        )
        .unwrap(),
      ),
      (
        "convex hull, triangle",
        convex_hull(&polygon(&[0, 0, 90, 10, 30, 70]), 15).unwrap(),
      ),
    ];

    // Every direction a segment can take, including the degenerate one,
    // and both sides of the kink threshold.
    for (dx, dy) in [
      (1000, 0),
      (-1000, 0),
      (0, 1000),
      (0, -1000),
      (1000, 1000),
      (-1000, 1000),
      (1000, -1000),
      (-1000, -1000),
      (300, 470),
      (-470, 300),
      (0, 0),
      (1, 50),
      (40, 41),
      (40, 10),
      (1, 0),
    ] {
      for clearance in [0, 100, 1000] {
        let seg = Seg::from_coords(7, 9, 7 + dx, 9 + dy);
        let hull = segment_hull(&seg, 200, clearance, 80);

        hulls.push(("segment hull", hull));
      }
    }

    for (name, hull) in hulls {
      assert!(
        hull.is_closed(),
        "{name} is not closed: {:?}",
        hull.points()
      );
      assert!(
        hull.area(false) > 0.0,
        "{name} does not wind clockwise, area {}: {:?}",
        hull.area(false),
        hull.points()
      );
    }
  }

  // -----------------------------------------------------------------
  // HullIntersection
  // -----------------------------------------------------------------

  /// The intersection points of a filtered hull intersection, flattened.
  fn intersection_points(records: &[Intersection]) -> Vec<i32> {
    records
      .iter()
      .flat_map(|record| [record.point.x, record.point.y])
      .collect()
  }

  #[test]
  fn hull_intersection_of_a_line_crossing_twice() {
    // A square hull from (0, 0) to (100, 100) and a horizontal line
    // straight through it at y = 50. Neither hit is a corner of either
    // chain, so both pass unconditionally.
    let hull = closed_chain(&[0, 0, 100, 0, 100, 100, 0, 100]);
    let line = open_chain(&[-50, 50, 150, 50]);

    let records = hull_intersection(&hull, &line);

    assert_eq!(records.len(), 2);
    assert_eq!(intersection_points(&records), vec![100, 50, 0, 50]);
    assert!(records.iter().all(|record| !record.ours.is_corner()));
    assert!(
      records
        .iter()
        .all(|record| !record.theirs.unwrap().is_corner())
    );
  }

  #[test]
  fn hull_intersection_of_a_line_touching_a_vertex() {
    let hull = closed_chain(&[0, 0, 100, 0, 100, 100, 0, 100]);

    // The line grazes the corner at (0, 0) and both of its neighbouring
    // points sit in the outer wedge of that corner, outside the top edge
    // and outside the left edge alike. No adjacent hull segment sees a
    // line point on its inner side, so every record is dropped.
    let grazing = open_chain(&[-50, -10, 0, 0, -10, -50]);

    assert_eq!(hull.intersect_chain(&grazing, true).len(), 4);
    assert!(hull_intersection(&hull, &grazing).is_empty());

    // The same corner, and the line still stays outside the hull, but
    // its far point is now on the inner side of the **left** edge's
    // line. The half plane test is per edge, so the records survive.
    // This is the filter's real shape, and it is deliberate.
    let outside_but_inside_an_edge = open_chain(&[-50, -50, 0, 0, 50, -50]);
    let records = hull_intersection(&hull, &outside_but_inside_an_edge);

    assert_eq!(records.len(), 4);

    // A line that walks straight through the corner keeps its records
    // too, which is the case the walkaround needs.
    let crossing = open_chain(&[-50, -50, 0, 0, 50, 50]);
    let records = hull_intersection(&hull, &crossing);

    assert_eq!(records.len(), 4);
    assert!(records.iter().all(|record| record.point == Vec2::new(0, 0)));
    assert!(records.iter().all(|record| record.ours.is_corner()));
  }

  #[test]
  fn hull_intersection_of_a_line_running_along_an_edge() {
    let hull = closed_chain(&[0, 0, 100, 0, 100, 100, 0, 100]);

    // The line lies on the hull's top edge and overhangs it at both
    // ends. `intersect_chain` keeps colinear and touching records, so the
    // two hull corners come back, once from the top edge's own colinear
    // branch and once from the side edge that ends there. All four
    // survive the filter, because each corner has a line point on the
    // inner side of one of its two edges.
    let along = open_chain(&[-50, 0, 150, 0]);
    let records = hull_intersection(&hull, &along);

    assert_eq!(
      intersection_points(&records),
      vec![0, 0, 100, 0, 100, 0, 0, 0]
    );
    assert_eq!(
      records.iter().map(|record| record.ours).collect::<Vec<_>>(),
      vec![
        Hit::Corner(0),
        Hit::Corner(1),
        Hit::Corner(1),
        Hit::Corner(0)
      ]
    );
    assert!(
      records
        .iter()
        .all(|record| record.theirs == Some(Hit::Segment(0)))
    );

    // The same edge, but the line stops on it and dives into the hull.
    // The extra crossing at (100, 50) is not a corner of either chain,
    // so it passes unconditionally, and two of the nine raw records are
    // filtered out.
    let leaving = open_chain(&[-50, 0, 100, 0, 100, 50]);

    assert_eq!(hull.intersect_chain(&leaving, true).len(), 9);
    assert_eq!(hull_intersection(&hull, &leaving).len(), 7);
  }

  #[test]
  fn hull_intersection_needs_two_points_on_the_line() {
    let hull = closed_chain(&[0, 0, 100, 0, 100, 100, 0, 100]);

    assert!(hull_intersection(&hull, &open_chain(&[50, 50])).is_empty());
    assert!(hull_intersection(&hull, &LineChain::new()).is_empty());
  }

  #[test]
  fn hull_intersection_corner_indices_are_in_range() {
    // The corner compensation KiCad writes out by hand at
    // `pns_utils.cpp:423` is already in `Hit::Corner`, so every index a
    // record carries indexes its own chain.
    let hull = closed_chain(&[0, 0, 100, 0, 100, 100, 0, 100]);
    let line = open_chain(&[-50, -50, 0, 0, 50, 50]);

    let records = hull_intersection(&hull, &line);

    assert!(!records.is_empty());

    for record in &records {
      match record.ours {
        Hit::Corner(index) => assert!(index < hull.point_count()),
        Hit::Segment(index) => assert!(index < hull.segment_count()),
      }

      match record.theirs.unwrap() {
        Hit::Corner(index) => assert!(index < line.point_count()),
        Hit::Segment(index) => assert!(index < line.segment_count()),
      }
    }
  }

  #[test]
  fn hull_intersection_works_on_a_real_hull() {
    // A track walking into the octagon around a via sized circle.
    let shape = Shape::circle(Vec2::new(0, 0), 300);
    let hull = build_hull_for_primitive_shape(&shape, 200, 250).unwrap();
    let line = open_chain(&[-2000, 0, 2000, 0]);

    let records = hull_intersection(&hull, &line);

    assert_eq!(records.len(), 2);

    for record in &records {
      assert_eq!(record.point.y, 0);
      assert!(record.point.x.abs() > 300);
    }
  }

  // -----------------------------------------------------------------
  // Constants
  // -----------------------------------------------------------------

  #[test]
  fn hull_margin_is_ten_nanometres() {
    assert_eq!(HULL_MARGIN, 10);
  }
}
