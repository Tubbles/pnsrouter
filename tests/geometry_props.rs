// SPDX-License-Identifier: GPL-3.0-or-later

//! Property based tests for the geometry layer.
//!
//! The unit tests next to the code pin KiCad's own expected values case by
//! case. These properties come at the same code from the other side: they
//! state the invariants the router leans on and let `proptest` search for
//! an input that breaks one.
//!
//! Coordinates stay inside a board sized window of plus or minus 100 mm in
//! nanometres, so every product of two coordinates stays far from the
//! limits the geometry widens to. Where a property needs two shapes to
//! actually meet, the window shrinks to plus or minus 0.1 mm, because two
//! random points 200 mm apart never collide.
//!
//! Several properties are deliberately narrower than they look, because
//! the port reproduces KiCad quirks on purpose. Every narrowing carries a
//! comment naming the quirk it steps around.

#![forbid(unsafe_code)]

use pnsrouter::geometry::collision::{collide, collide_mtv, collides};
use pnsrouter::geometry::direction45::{CornerMode, Direction45};
use pnsrouter::geometry::hull::{
  build_hull_for_primitive_shape, hull_intersection, monotone_chain_hull,
};
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use proptest::prelude::*;

// -------------------------------------------------------------------
// Strategies
// -------------------------------------------------------------------

/// Half the side of the board sized coordinate window, 100 mm in
/// nanometres.
const BOARD: i32 = 100_000_000;

/// Half the side of the window the interaction heavy properties use,
/// 0.1 mm in nanometres.
const NEAR: i32 = 100_000;

/// Half the side of the window the nanometre exact predicates use, 10 mm
/// in nanometres.
///
/// `SEG::SquaredDistance` corrects its interior case in `f64`,
/// `|ap|^2 - e * e / f` (`seg.cpp:731`), and the two terms are of the same
/// magnitude. At a coordinate of 100 mm that magnitude is around 1e16,
/// where an `f64` step is already two units, so the answer can miss the
/// squared tolerance of 3 that `SEG::Contains` compares against
/// (`seg.cpp:623`) by more than the tolerance itself. Inside 10 mm the
/// correction is exact to a fraction of a unit, which is what a property
/// about that tolerance needs.
const FINE: i32 = 10_000_000;

/// One nanometre coordinate inside a window.
fn coordinate(limit: i32) -> impl Strategy<Value = i32> {
  -limit..=limit
}

/// A point inside a window.
fn point(limit: i32) -> impl Strategy<Value = Vec2> {
  (coordinate(limit), coordinate(limit)).prop_map(|(x, y)| Vec2::new(x, y))
}

/// A segment whose endpoints lie inside a window. Degenerate segments are
/// allowed on purpose: every predicate handles them explicitly.
fn segment(limit: i32) -> impl Strategy<Value = Seg> {
  (point(limit), point(limit)).prop_map(|(a, b)| Seg::new(a, b))
}

/// A chain of two to twelve points with the given closed flag.
fn chain(limit: i32, closed: bool) -> impl Strategy<Value = LineChain> {
  prop::collection::vec(point(limit), 2..=12)
    .prop_map(move |points| LineChain::from_points(points, closed))
}

/// An open or closed chain of two to twelve points.
fn any_chain(limit: i32) -> impl Strategy<Value = LineChain> {
  prop_oneof![chain(limit, false), chain(limit, true)]
}

/// An open chain whose every leg is horizontal, vertical or an exact 45
/// degrees, built by chaining [`Direction45::build_initial_trace`] over a
/// handful of way points.
fn chain_45(limit: i32) -> impl Strategy<Value = LineChain> {
  prop::collection::vec(point(limit), 2..=6).prop_filter_map(
    "a 45 degree trace of at least two points",
    |waypoints| {
      let mut result = LineChain::new();

      for pair in waypoints.windows(2) {
        let leg = Direction45::default().build_initial_trace(
          pair[0],
          pair[1],
          false,
          CornerMode::Mitered45,
        );

        for vertex in leg {
          result.append(vertex);
        }
      }

      (result.point_count() >= 2).then_some(result)
    },
  )
}

/// A circle with a radius no board would call unreasonable.
fn shape_circle(limit: i32) -> impl Strategy<Value = Shape> {
  (point(limit), 1i32..=50_000)
    .prop_map(|(center, radius)| Shape::circle(center, radius))
}

/// A sharp cornered rectangle of positive size.
fn shape_rect(limit: i32) -> impl Strategy<Value = Shape> {
  (point(limit), 1i32..=100_000, 1i32..=100_000).prop_map(
    |(origin, width, height)| Shape::rect(origin, Vec2::new(width, height)),
  )
}

/// A capsule of **even** width.
///
/// The width is even on purpose. The dispatch file truncates the other
/// operand's half width where `SHAPE_SEGMENT`'s own routines round it up
/// (`shape_collisions.cpp:336` against `shape_segment.h:83`), so a capsule
/// against a capsule is asymmetric by one nanometre for odd widths. That
/// quirk is reproduced by the port and would break the symmetry property
/// below; an even width sidesteps it without hiding it.
fn shape_segment(limit: i32) -> impl Strategy<Value = Shape> {
  (segment(limit), 1i32..=25_000)
    .prop_map(|(seg, half_width)| Shape::segment(seg, 2 * half_width))
}

/// A convex polygon, built as the monotone chain hull of random points so
/// that it really is convex: `PNS::ConvexHull` assumes convexity without
/// checking and so does [`Shape::Simple`].
fn shape_simple(limit: i32) -> impl Strategy<Value = Shape> {
  prop::collection::vec(point(limit), 3..=8).prop_filter_map(
    "a convex hull of at least three vertices",
    |points| {
      let hull = monotone_chain_hull(&points);

      (hull.point_count() >= 3).then(|| Shape::simple(hull))
    },
  )
}

/// One of the four primitive shapes a hull can be built for.
fn shape_leaf(limit: i32) -> impl Strategy<Value = Shape> {
  prop_oneof![
    shape_circle(limit),
    shape_rect(limit),
    shape_segment(limit),
    shape_simple(limit),
  ]
}

/// A primitive shape, or a compound of two of them.
fn shape_any(limit: i32) -> impl Strategy<Value = Shape> {
  prop_oneof![
    4 => shape_leaf(limit),
    1 => (shape_leaf(limit), shape_leaf(limit))
      .prop_map(|(a, b)| Shape::compound(vec![a, b])),
  ]
}

// -------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------

/// Whether a shape is, or contains, a [`Shape::Rect`].
///
/// [`collides`] substitutes an inclusive bounding box test for a pair of
/// sharp cornered rectangles at a clearance of zero
/// (`shape_collisions.cpp:589`), which is the one documented place where
/// it does not agree with [`collide`]. This spots the pairs that reach it.
fn holds_rect(shape: &Shape) -> bool {
  shape
    .subshapes()
    .iter()
    .any(|leaf| matches!(leaf, Shape::Rect { .. }))
}

/// Whether this rectangle contains that circle's centre, inclusively.
///
/// The rectangle versus circle cell reports the distance to the nearest
/// **side** as the gap when the centre is inside (`:135`), instead of
/// zero. KiCad carries that defect and the port reproduces it, so the
/// property about the reported gap has to step around the branch.
fn rect_holds_circle_centre(rect: &Shape, circle: &Shape) -> bool {
  let (
    Shape::Rect {
      origin,
      size,
      radius,
    },
    Shape::Circle { center, .. },
  ) = (rect, circle)
  else {
    return false;
  };

  *radius == 0
    && i64::from(center.x) >= i64::from(origin.x)
    && i64::from(center.x) <= i64::from(origin.x) + i64::from(size.x)
    && i64::from(center.y) >= i64::from(origin.y)
    && i64::from(center.y) <= i64::from(origin.y) + i64::from(size.y)
}

/// Whether any leaf pair of two shapes reaches that containment branch.
fn reaches_rect_circle_containment(first: &Shape, second: &Shape) -> bool {
  let right = second.subshapes();

  first.subshapes().iter().any(|left| {
    right.iter().any(|other| {
      rect_holds_circle_centre(left, other)
        || rect_holds_circle_centre(other, left)
    })
  })
}

/// The points of a shape that a hull has to swallow.
///
/// Vertices for a polygon, the corners for a rectangle, the endpoints for
/// a capsule spine, and the centre plus eight rim samples for a circle.
fn boundary_samples(shape: &Shape) -> Vec<Vec2> {
  match shape {
    Shape::Circle { center, radius } => {
      let mut samples = vec![*center];

      for direction in [
        Vec2::new(1, 0),
        Vec2::new(1, 1),
        Vec2::new(0, 1),
        Vec2::new(-1, 1),
        Vec2::new(-1, 0),
        Vec2::new(-1, -1),
        Vec2::new(0, -1),
        Vec2::new(1, -1),
      ] {
        samples.push(*center + direction.resize(*radius));
      }

      samples
    }
    Shape::Rect { origin, size, .. } => vec![
      *origin,
      *origin + Vec2::new(size.x, 0),
      *origin + *size,
      *origin + Vec2::new(0, size.y),
    ],
    Shape::Segment { seg, .. } => vec![seg.a, seg.b],
    // The three defining points, which are on the curve by
    // construction. Nothing in this file builds an arc yet. The arm is
    // here because the match is exhaustive.
    Shape::Arc(arc) => vec![arc.start(), arc.arc_mid(), arc.end()],
    Shape::Simple(polygon) => polygon.vertices().points().to_vec(),
    Shape::LineChain(chain) => chain.points().to_vec(),
    Shape::Compound(parts) => parts.iter().flat_map(boundary_samples).collect(),
  }
}

/// An even odd ray cast written from scratch, in exact arithmetic.
///
/// This is the reference [`LineChain::point_inside`] is checked against.
/// It reproduces the two boundary rules the port documents, the half open
/// straddle test `( p1.y >= p.y ) != ( p2.y >= p.y )` and the strict side
/// test, but it compares the crossing exactly in `i128` instead of
/// rounding the projection through `rescale`. The two therefore agree for
/// every point that is not within half a nanometre of an edge, which is
/// why the property that uses it keeps its points clear of the outline.
fn reference_point_inside(chain: &LineChain, query: Vec2) -> bool {
  let points = chain.points();

  if !chain.is_closed() || points.len() < 3 {
    return false;
  }

  let mut inside = false;

  for index in 0..points.len() {
    let first = points[index];
    let second = points[(index + 1) % points.len()];
    let denominator = i128::from(second.y) - i128::from(first.y);

    if denominator == 0 {
      continue;
    }

    if (first.y >= query.y) == (second.y >= query.y) {
      continue;
    }

    // `query.x - first.x < ( query.y - first.y ) * dx / dy`, cleared of
    // the division and with the inequality flipped for a negative
    // denominator.
    let numerator = (i128::from(query.y) - i128::from(first.y))
      * (i128::from(second.x) - i128::from(first.x));
    let scaled = (i128::from(query.x) - i128::from(first.x)) * denominator;

    if (denominator > 0 && scaled < numerator)
      || (denominator < 0 && scaled > numerator)
    {
      inside = !inside;
    }
  }

  inside
}

/// Whether `shorter` can be obtained from `longer` by deleting elements
/// that each equal the element kept before them.
///
/// This is the honest reading of "never removes a non duplicate": a
/// deletion is legitimate only when the deleted point repeats the point
/// that survived just ahead of it.
fn removes_only_duplicates(longer: &[Vec2], shorter: &[Vec2]) -> bool {
  let mut kept = 0usize;
  let mut last_kept: Option<Vec2> = None;

  for &candidate in longer {
    if kept < shorter.len() && shorter[kept] == candidate {
      last_kept = Some(candidate);
      kept += 1;
      continue;
    }

    if last_kept != Some(candidate) {
      return false;
    }
  }

  kept == shorter.len()
}

// -------------------------------------------------------------------
// Seg
// -------------------------------------------------------------------

proptest! {
  /// `SEG::Distance( const SEG& )` measures a set, so the operand order
  /// cannot matter. `SegDistanceCorrect` checks it on a table
  /// (`test_segment.cpp:70`); this checks it everywhere.
  #[test]
  fn seg_distance_is_symmetric(
    first in segment(BOARD),
    second in segment(BOARD),
  ) {
    prop_assert_eq!(
      first.distance_to_segment(&second),
      second.distance_to_segment(&first)
    );
    prop_assert_eq!(
      first.squared_distance_to_segment(&second),
      second.squared_distance_to_segment(&first)
    );
  }

  /// `SEG::Collide` compares strictly, `dist < clearance`, on a squared
  /// distance (`seg.cpp:538`), so a gap below the clearance always
  /// collides and a gap the truncated square root reports above
  /// `clearance + 2` never does.
  #[test]
  fn seg_collide_brackets_the_distance(
    first in segment(NEAR),
    second in segment(NEAR),
    clearance in 0i32..=200_000,
  ) {
    let distance = first.distance_to_segment(&second);
    let hit = first.collide(&second, clearance);

    if distance < clearance {
      prop_assert!(hit.collides, "distance {distance} below {clearance}");
    }

    if distance > clearance + 2 {
      prop_assert!(!hit.collides, "distance {distance} above {clearance}");
    }
  }

  /// A reported intersection point lies on both segments, to within the
  /// squared tolerance of 3 that `SEG::Contains` carries
  /// (`seg.cpp:623`). That covers the rounding of the rational crossing
  /// and the truncation of a collinear overlap midpoint (`seg.cpp:281`).
  #[test]
  fn seg_intersection_lies_on_both_segments(
    first in segment(NEAR),
    second in segment(NEAR),
  ) {
    if let Some(crossing) = first.intersect(&second, false, false) {
      prop_assert!(
        first.contains_point(crossing),
        "{crossing:?} off {first:?}"
      );
      prop_assert!(
        second.contains_point(crossing),
        "{crossing:?} off {second:?}"
      );
    }
  }

  /// `SEG::NearestPoint` answers with a point that lies on the segment,
  /// to within the squared tolerance of 3 that `SEG::Contains` carries
  /// (`seg.cpp:623`).
  ///
  /// The window is the 10 mm one, not the board sized one: see [`FINE`]
  /// for why a tolerance of three squared nanometres stops meaning
  /// anything at 100 mm coordinates.
  #[test]
  fn seg_nearest_point_is_on_the_segment(
    seg in segment(FINE),
    query in point(FINE),
  ) {
    let nearest = seg.nearest_point_to_point(query);

    prop_assert!(seg.contains_point(nearest), "{nearest:?} off {seg:?}");
  }

  /// `SEG::NearestPoint` is at least as close to the query as either
  /// endpoint. The one nanometre of slack pays for the `rescale` rounding
  /// of the interior case (`seg.cpp:629`).
  #[test]
  fn seg_nearest_point_beats_both_endpoints(
    seg in segment(BOARD),
    query in point(BOARD),
  ) {
    let nearest = seg.nearest_point_to_point(query);
    let distance = (nearest - query).euclidean_norm();

    prop_assert!(distance <= (seg.a - query).euclidean_norm() + 1);
    prop_assert!(distance <= (seg.b - query).euclidean_norm() + 1);
  }

  /// `SEG::NearestPoints` reports a point on each segment together with
  /// their separation, and the three agree: each point really lies on its
  /// own segment and the reported squared distance is the squared
  /// distance between them (`seg.cpp:158`). The shape level collision
  /// code builds its translation vectors out of this pair, so a stale
  /// field here would be invisible until the shove misbehaved.
  #[test]
  fn seg_nearest_points_sit_on_their_segments(
    first in segment(NEAR),
    second in segment(NEAR),
  ) {
    let pair = first.nearest_points(&second);

    prop_assert!(first.contains_point(pair.on_self));
    prop_assert!(second.contains_point(pair.on_other));
    prop_assert_eq!(
      pair.squared_distance,
      pair.on_self.squared_distance(pair.on_other)
    );
  }
}

// -------------------------------------------------------------------
// LineChain
// -------------------------------------------------------------------

proptest! {
  /// `Append` drops a point equal to the last one
  /// (`shape_line_chain.h:534`). The placer depends on it, so N calls do
  /// not imply N points and no two neighbours are ever equal.
  #[test]
  fn append_never_stores_equal_neighbours(
    points in prop::collection::vec(point(1_000), 1..=30),
  ) {
    let mut result = LineChain::new();

    for candidate in &points {
      result.append(*candidate);
    }

    prop_assert!(result.point_count() <= points.len());

    for pair in result.points().windows(2) {
      prop_assert_ne!(pair[0], pair[1]);
    }
  }

  /// `SegmentCount` is `max( 0, PointCount() - 1 + closed )`
  /// (`shape_line_chain.h:327`), so a closed chain carries as many
  /// segments as points.
  #[test]
  fn segment_count_follows_the_closed_rule(chain in any_chain(BOARD)) {
    let points = chain.point_count();
    let expected = if chain.is_closed() {
      if points == 0 { 0 } else { points }
    } else {
      points.saturating_sub(1)
    };

    prop_assert_eq!(chain.segment_count(), expected);
  }

  /// `Simplify` never adds a vertex, and on an **open** chain it keeps
  /// the first and the last. The closed case is excluded on purpose: the
  /// walk wraps modulo the point count, so a closed chain can simplify
  /// across its seam and lose its first vertex
  /// (`shape_line_chain.cpp:2782`).
  #[test]
  fn simplify_keeps_the_ends_of_an_open_chain(
    chain in chain(NEAR, false),
    tolerance in 0i32..=64,
  ) {
    let original = chain.clone();
    let mut simplified = chain;

    simplified.simplify(tolerance);

    prop_assert!(simplified.point_count() <= original.point_count());
    prop_assert!(simplified.point_count() >= 2);
    prop_assert_eq!(simplified.point(0), original.point(0));
    prop_assert_eq!(simplified.last_point(), original.last_point());
  }

  /// `Simplify2` ignores the closed flag entirely
  /// (`shape_line_chain.cpp:2906`), so it keeps the first and the last
  /// point of any chain and never adds one.
  #[test]
  fn simplify2_keeps_the_ends(
    chain in any_chain(NEAR),
    remove_colinear in any::<bool>(),
  ) {
    let original = chain.clone();
    let mut simplified = chain;

    simplified.simplify2(remove_colinear);

    prop_assert!(simplified.point_count() <= original.point_count());
    prop_assert_eq!(simplified.point(0), original.point(0));
    prop_assert_eq!(simplified.last_point(), original.last_point());
  }

  /// `Reverse` copies and reverses the point vector, keeping the closed
  /// flag and the width (`shape_line_chain.cpp:910`), so it is its own
  /// inverse.
  #[test]
  fn reversing_twice_is_the_identity(chain in any_chain(BOARD)) {
    prop_assert_eq!(chain.reversed().reversed(), chain);
  }

  /// `Slice` hands back exactly the requested closed range of points, as
  /// an open chain of width zero (`shape_line_chain.cpp:1422`).
  #[test]
  fn slice_returns_the_requested_points(
    chain in any_chain(BOARD),
    first in 0usize..12,
    span in 0usize..12,
  ) {
    let count = chain.point_count();
    let start = first % count;
    let end = (start + span).min(count - 1);
    let slice = chain.slice(start, end).expect("in range");

    prop_assert_eq!(slice.points(), &chain.points()[start..=end]);
    prop_assert!(!slice.is_closed());
    prop_assert_eq!(slice.width(), 0);
  }

  /// `Slice` refuses an out of range index and a backwards range, rather
  /// than answering with an empty chain the way KiCad does
  /// (`shape_line_chain.cpp:1422`).
  #[test]
  fn slice_refuses_a_backwards_range(
    chain in any_chain(BOARD),
    pick in 0usize..12,
  ) {
    let count = chain.point_count();

    prop_assert!(chain.slice(count, count).is_err());

    if count >= 2 {
      let end = 1 + pick % (count - 1);

      prop_assert!(chain.slice(end, end - 1).is_err());
    }
  }

  /// `PathLength` of the last point, with the last segment as the hint,
  /// is `Length`: both sum `SEG::Length` over the same segments
  /// (`shape_line_chain.cpp:1952`, `:956`).
  #[test]
  fn path_length_of_the_last_point_is_the_length(
    chain in chain(BOARD, false),
  ) {
    let last_segment = chain.segment_count() - 1;
    let last = chain.last_point().expect("two points at least");

    prop_assert_eq!(
      chain.path_length(last, Some(last_segment)),
      Some(chain.length())
    );
  }

  /// `PointInside` is a crossing number with a half open straddle test
  /// and a strict side test (`shape_line_chain.cpp:1986`). Away from the
  /// outline it has to agree with an independent even odd ray cast in
  /// exact arithmetic.
  ///
  /// Points within two nanometres of the outline are excluded, because
  /// the port rounds the ray crossing through `rescale` while the
  /// reference divides exactly; the two can only disagree inside half a
  /// nanometre of an edge. The polygon is convex here only because that
  /// is what the hull builders produce; the reference handles any closed
  /// chain.
  #[test]
  fn point_inside_agrees_with_an_even_odd_reference(
    points in prop::collection::vec(point(NEAR), 3..=10),
    query in point(NEAR),
  ) {
    let polygon = monotone_chain_hull(&points);

    prop_assume!(polygon.point_count() >= 3);
    prop_assume!(polygon.distance(query, true) > 2);
    prop_assert_eq!(
      polygon.point_inside(query, 0),
      reference_point_inside(&polygon, query)
    );
  }

  /// An accuracy of two or more turns `PointInside` into "inside or on
  /// the edge" (`shape_line_chain.cpp:2065`), so it can only ever answer
  /// yes where the plain test already did.
  #[test]
  fn point_inside_grows_with_the_accuracy(
    points in prop::collection::vec(point(NEAR), 3..=10),
    query in point(NEAR),
  ) {
    let polygon = monotone_chain_hull(&points);

    prop_assume!(polygon.point_count() >= 3);

    if polygon.point_inside(query, 0) {
      prop_assert!(polygon.point_inside(query, 2));
    }
  }

  /// Every record `Intersect` reports names a point that really lies on
  /// both chains, to within the two nanometre band `PointOnEdge` accepts
  /// at an accuracy of one.
  #[test]
  fn intersections_lie_on_both_chains(
    first in any_chain(NEAR),
    second in any_chain(NEAR),
    include_colinear in any::<bool>(),
  ) {
    for record in first.intersect_chain(&second, include_colinear) {
      prop_assert!(
        first.point_on_edge(record.point, 1),
        "{:?} off the first chain",
        record.point
      );
      prop_assert!(
        second.point_on_edge(record.point, 1),
        "{:?} off the second chain",
        record.point
      );
    }
  }

  /// `Intersects` asks whether the full intersection came back empty
  /// (`shape_line_chain.cpp:2610`), and crossing is a symmetric relation,
  /// so the operand order cannot change the answer.
  #[test]
  fn chain_intersection_is_symmetric_as_a_predicate(
    first in any_chain(NEAR),
    second in any_chain(NEAR),
  ) {
    prop_assert_eq!(
      first.intersects_chain(&second),
      second.intersects_chain(&first)
    );
  }

  /// The point set of an intersection does not depend on the operand
  /// order either, whichever way the colinear and touching records are
  /// asked for.
  #[test]
  fn chain_intersection_points_are_symmetric(
    first in any_chain(NEAR),
    second in any_chain(NEAR),
    include_colinear in any::<bool>(),
  ) {
    let mut ours: Vec<Vec2> = first
      .intersect_chain(&second, include_colinear)
      .iter()
      .map(|record| record.point)
      .collect();
    let mut theirs: Vec<Vec2> = second
      .intersect_chain(&first, include_colinear)
      .iter()
      .map(|record| record.point)
      .collect();

    ours.sort_unstable_by_key(|p| (p.x, p.y));
    ours.dedup();
    theirs.sort_unstable_by_key(|p| (p.x, p.y));
    theirs.dedup();

    prop_assert_eq!(ours, theirs);
  }

  /// A chain always meets itself: every one of its own vertices is a
  /// point of the self intersection, with the colinear and touching
  /// records included.
  #[test]
  fn a_chain_intersects_itself_at_every_vertex(
    chain in any_chain(NEAR),
  ) {
    let found: Vec<Vec2> = chain
      .intersect_chain(&chain, true)
      .iter()
      .map(|record| record.point)
      .collect();

    for vertex in chain.points() {
      prop_assert!(found.contains(vertex), "{vertex:?} not reported");
    }
  }

  /// `RemoveDuplicatePoints` only ever deletes a point that repeats the
  /// point kept before it (`shape_line_chain.cpp:2720`). No colinear
  /// vertex is touched and the closed flag survives.
  ///
  /// The coordinate window is 64 nanometres wide here, so that a random
  /// chain really does grow runs of equal points for the routine to
  /// collapse.
  #[test]
  fn remove_duplicate_points_removes_only_duplicates(
    points in prop::collection::vec(point(64), 1..=14),
    closed in any::<bool>(),
  ) {
    let original = LineChain::from_points(points, closed);
    let mut trimmed = original.clone();

    trimmed.remove_duplicate_points();

    prop_assert_eq!(trimmed.is_closed(), original.is_closed());
    prop_assert!(trimmed.point_count() <= original.point_count());
    prop_assert!(
      removes_only_duplicates(original.points(), trimmed.points()),
      "{:?} is not a duplicate only trim of {:?}",
      trimmed.points(),
      original.points()
    );
  }

  /// The absolute form of `Area` is the magnitude of the signed one, and
  /// reversing the walk flips the sign (`shape_line_chain.cpp:2696`). The
  /// nanometre of slack is the `f64` accumulation KiCad's own callers
  /// compare in.
  #[test]
  fn area_is_the_magnitude_of_the_signed_area(
    chain in chain(NEAR, true),
  ) {
    let signed = chain.area(false);

    prop_assert!((chain.area(true) - signed.abs()).abs() <= 1.0);
    prop_assert!((chain.reversed().area(false) + signed).abs() <= 1.0);
  }
}

// -------------------------------------------------------------------
// Direction45
// -------------------------------------------------------------------

proptest! {
  /// `BuildInitialTrace` answers with at most three points, starts at
  /// `p0`, ends at `p1` and lays down nothing but horizontal, vertical
  /// and exact 45 degree legs (`direction_45.cpp:24`). The 90 degree mode
  /// lays down no diagonal at all.
  #[test]
  fn build_initial_trace_stays_on_the_45_degree_grid(
    from in point(BOARD),
    to in point(BOARD),
    start_diagonal in any::<bool>(),
    ninety in any::<bool>(),
  ) {
    let mode = if ninety {
      CornerMode::Mitered90
    } else {
      CornerMode::Mitered45
    };
    let trace = Direction45::default()
      .build_initial_trace(from, to, start_diagonal, mode);

    prop_assert!(!trace.is_empty());
    prop_assert!(trace.len() <= 3);
    prop_assert_eq!(trace[0], from);
    prop_assert_eq!(*trace.last().expect("not empty"), to);

    for leg in trace.windows(2) {
      let delta = leg[1].widening_sub(leg[0]);
      let horizontal = delta.y == 0;
      let vertical = delta.x == 0;
      let diagonal = delta.x.abs() == delta.y.abs();

      prop_assert!(
        horizontal || vertical || diagonal,
        "leg {:?} to {:?} is off the grid",
        leg[0],
        leg[1]
      );

      if ninety {
        prop_assert!(
          horizontal || vertical,
          "leg {:?} to {:?} is diagonal in 90 degree mode",
          leg[0],
          leg[1]
        );
      }
    }
  }

  /// A trace built out of chained `BuildInitialTrace` calls is still on
  /// the grid once `Append` has suppressed the shared way points.
  #[test]
  fn a_chained_45_degree_trace_stays_on_the_grid(
    trace in chain_45(NEAR),
  ) {
    for index in 0..trace.segment_count() {
      let seg = trace.segment(index);
      let delta = seg.b.widening_sub(seg.a);

      prop_assert!(
        delta.x == 0 || delta.y == 0 || delta.x.abs() == delta.y.abs(),
        "segment {seg:?} is off the grid"
      );
    }
  }
}

// -------------------------------------------------------------------
// Collision
// -------------------------------------------------------------------

proptest! {
  /// The boolean entry point answers the same question as the one that
  /// fills in the gap.
  ///
  /// The one documented exception is skipped: a pair of sharp cornered
  /// rectangles at a clearance of zero takes an **inclusive** bounding box
  /// shortcut in the boolean form only (`shape_collisions.cpp:589`), which
  /// is the whole reason [`collides`] exists as a separate entry point.
  #[test]
  fn collides_agrees_with_collide(
    first in shape_any(NEAR),
    second in shape_any(NEAR),
    clearance in 0i32..=200_000,
  ) {
    if clearance == 0 && holds_rect(&first) && holds_rect(&second) {
      return Ok(());
    }

    prop_assert_eq!(
      collides(&first, &second, clearance),
      collide(&first, &second, clearance).is_some()
    );
  }

  /// Colliding is a symmetric relation. The dispatcher reaches half of
  /// its cells by swapping the operands (`shape_collisions.cpp:1107` and
  /// following), so this is the property that keeps the swaps honest.
  #[test]
  fn collides_is_symmetric(
    first in shape_any(NEAR),
    second in shape_any(NEAR),
    clearance in 0i32..=200_000,
  ) {
    prop_assert_eq!(
      collides(&first, &second, clearance),
      collides(&second, &first, clearance)
    );
  }

  /// The reported gap is never as large as the clearance that found it:
  /// the comparison is `dist == 0 || dist < clearance`
  /// (`shape_collisions.cpp:49`).
  ///
  /// One branch is excluded, and it is a KiCad defect the port keeps on
  /// purpose: a circle whose centre lies inside a sharp cornered
  /// rectangle collides, but the gap written is the distance to the
  /// nearest side rather than zero (`:135`). A circle of radius 1 at
  /// `(-21948, 14059)` inside the rectangle at `(-22882, 13125)` of size
  /// `1868 by 77492` reports a gap of 933 at a clearance of zero.
  #[test]
  fn a_reported_gap_stays_under_the_clearance(
    first in shape_any(NEAR),
    second in shape_any(NEAR),
    clearance in 0i32..=200_000,
  ) {
    if reaches_rect_circle_containment(&first, &second) {
      return Ok(());
    }

    if let Some(hit) = collide(&first, &second, clearance) {
      prop_assert!(hit.actual >= 0);
      prop_assert!(
        hit.actual == 0 || hit.actual < clearance,
        "gap {} against clearance {clearance}",
        hit.actual
      );
    }
  }

  /// The translation vector displaces the **second** operand, and
  /// applying it separates the pair.
  ///
  /// Narrowed to the two closed form cells, a circle against a circle and
  /// a circle against a rectangle. The three cells that go through
  /// `pushoutForce` (`shape_collisions.cpp:154`), a circle against a
  /// capsule, a polyline or a polygon, do **not** separate reliably, and
  /// the reason is a KiCad inconsistency the port reproduces: the search
  /// stops when `SEG::Distance( c + f ) >= min_dist` (`:168`) while the
  /// predicate that decides the collision measures to the **rounded**
  /// `SEG::NearestPoint` instead (`shape_circle.h:78`). The two disagree
  /// by up to a nanometre, so the pushout can stop one nanometre short of
  /// clearing the predicate that asked for it.
  ///
  /// A minimal counterexample, in nanometres: a circle of radius 23402 at
  /// `(5328, 1321)` against the open chain
  /// `(50708, -16429) -> (-47671, 33587)` at a clearance of 49615 answers
  /// `(30947, 60857)`, which leaves the moved centre at a true distance of
  /// 73017 from the segment, exactly `min_dist`, but only 73016 from the
  /// rounded nearest point, so it still collides.
  ///
  /// The polyline cells are weaker still: the vector is a single ordered
  /// relaxation pass over the segments with no convergence check
  /// (`:314`), so on a chain of several segments it can leave the shapes
  /// overlapping outright. Both are documented on `collide_mtv`, which
  /// nevertheless claims that applying the vector always separates the
  /// pair; that sentence is the over claim, not the code.
  #[test]
  fn a_non_zero_translation_vector_separates_the_pair(
    center in point(NEAR),
    radius in 1i32..=50_000,
    other in prop_oneof![shape_circle(NEAR), shape_rect(NEAR)],
    clearance in 0i32..=50_000,
  ) {
    let circle = Shape::circle(center, radius);
    let Some(vector) = collide_mtv(&circle, &other, clearance) else {
      return Ok(());
    };

    if vector == Vec2::new(0, 0) {
      return Ok(());
    }

    let mut moved = other.clone();
    moved.move_by(vector);

    prop_assert!(
      !collides(&circle, &moved, clearance),
      "{vector:?} did not separate {circle:?} from {other:?}"
    );
  }

  /// Mirroring the operands negates the translation vector, which is what
  /// makes the one sign convention usable: the caller can pick either
  /// order and know which shape the answer moves.
  #[test]
  fn mirroring_the_operands_negates_the_vector(
    center in point(NEAR),
    radius in 1i32..=50_000,
    other in shape_leaf(NEAR),
    clearance in 0i32..=50_000,
  ) {
    let circle = Shape::circle(center, radius);
    let forward = collide_mtv(&circle, &other, clearance);
    let backward = collide_mtv(&other, &circle, clearance);

    match (forward, backward) {
      (Some(first), Some(second)) => {
        prop_assert_eq!(first, -second);
      }
      (first, second) => prop_assert_eq!(first.is_some(), second.is_some()),
    }
  }
}

// -------------------------------------------------------------------
// Hulls
// -------------------------------------------------------------------

proptest! {
  /// Every hull comes out closed and clockwise on screen, which
  /// `LINE::Walkaround` relies on because it implements the counter
  /// clockwise winding by reversing the hull
  /// (`pcbnew/router/pns_line.cpp:397`).
  #[test]
  fn every_hull_is_closed_and_clockwise(
    shape in shape_leaf(NEAR),
    clearance in 1_000i32..=100_000,
    thickness in 0i32..=100_000,
  ) {
    let Some(hull) =
      build_hull_for_primitive_shape(&shape, clearance, thickness)
    else {
      return Ok(());
    };

    prop_assert!(hull.is_closed());
    prop_assert!(hull.point_count() >= 3);
    prop_assert!(
      hull.area(false) > 0.0,
      "{shape:?} gave a hull of area {}",
      hull.area(false)
    );
  }

  /// A hull swallows the shape it was built for. The samples are the
  /// vertices of a polygon, the corners of a rectangle, the endpoints of
  /// a capsule spine and the centre plus eight rim points of a circle.
  #[test]
  fn every_hull_contains_its_shape(
    shape in shape_leaf(NEAR),
    clearance in 1_000i32..=100_000,
    thickness in 0i32..=100_000,
  ) {
    let Some(hull) =
      build_hull_for_primitive_shape(&shape, clearance, thickness)
    else {
      return Ok(());
    };

    for sample in boundary_samples(&shape) {
      prop_assert!(
        hull.point_inside(sample, 2),
        "{sample:?} of {shape:?} is outside its hull"
      );
    }
  }

  /// A hull grows with the clearance: the hull at `clearance` contains
  /// every vertex of the hull at `clearance - 1000`.
  ///
  /// The capsule is left out, and the exclusion is measured rather than
  /// assumed. `SegmentHull` snaps a segment no longer than
  /// `clearance / 10` onto the 45 degree grid (`pns_utils.cpp:207`), so
  /// two clearances a thousand nanometres apart can straighten the same
  /// spine differently and the tighter hull then escapes the wider one.
  /// The counterexample, in nanometres: the capsule of width 2 on the
  /// spine `(-41266, 11822) -> (-45989, 7607)`, whose length is 6330, at
  /// clearances 62300 and 63300. Only the second one has a kink threshold
  /// the spine reaches, so only the second hull is built around a
  /// diagonal. That is the correction the log of 2026-09-08 records as
  /// reproduced on purpose.
  #[test]
  fn a_hull_grows_with_the_clearance(
    shape in prop_oneof![
      shape_circle(NEAR),
      shape_rect(NEAR),
      shape_simple(NEAR),
    ],
    clearance in 2_000i32..=100_000,
    thickness in 0i32..=100_000,
  ) {
    let inner = build_hull_for_primitive_shape(&shape, clearance - 1000,
      thickness);
    let outer = build_hull_for_primitive_shape(&shape, clearance, thickness);
    let (Some(inner), Some(outer)) = (inner, outer) else {
      return Ok(());
    };

    for vertex in inner.points() {
      prop_assert!(
        outer.point_inside(*vertex, 2),
        "{vertex:?} of the tighter hull escapes the wider one"
      );
    }
  }

  /// `HullIntersection` is a filter over `Intersect`
  /// (`pns_utils.cpp:395`), so it can only ever return records the raw
  /// intersection already produced.
  #[test]
  fn hull_intersection_is_a_subset_of_the_raw_intersection(
    shape in shape_leaf(NEAR),
    line in any_chain(NEAR),
    clearance in 1_000i32..=100_000,
    thickness in 0i32..=100_000,
  ) {
    let Some(hull) =
      build_hull_for_primitive_shape(&shape, clearance, thickness)
    else {
      return Ok(());
    };

    let raw = hull.intersect_chain(&line, true);

    for kept in hull_intersection(&hull, &line) {
      prop_assert!(raw.contains(&kept), "{kept:?} is not a raw record");
    }
  }

  /// The monotone chain hull is convex, closed and clockwise on screen,
  /// and it swallows every point it was built from
  /// (`convex_hull.cpp:83`). The turn test is what makes the convexity a
  /// claim rather than a hope: `Shape::Simple` and `PNS::ConvexHull` both
  /// assume it without checking.
  #[test]
  fn the_monotone_chain_hull_contains_its_points(
    points in prop::collection::vec(point(NEAR), 1..=20),
  ) {
    let hull = monotone_chain_hull(&points);

    prop_assert!(hull.is_closed());

    if hull.point_count() < 3 {
      return Ok(());
    }

    prop_assert!(hull.area(false) > 0.0);

    let vertices = hull.points();

    for index in 0..vertices.len() {
      let first = vertices[index];
      let second = vertices[(index + 1) % vertices.len()];
      let third = vertices[(index + 2) % vertices.len()];
      let turn = second.widening_sub(first).cross(third.widening_sub(first));

      prop_assert!(turn > 0, "the corner at {second:?} is not convex");
    }

    for candidate in &points {
      prop_assert!(
        hull.point_inside(*candidate, 0) || hull.point_on_edge(*candidate, 0),
        "{candidate:?} is outside its own hull"
      );
    }
  }
}
