// SPDX-License-Identifier: GPL-3.0-or-later

//! Collision between two shapes.
//!
//! Port of `libs/kimath/src/geometry/shape_collisions.cpp`, which KiCad
//! reaches through two mutually exclusive entry points
//! (`shape_collisions.cpp:1432` and `:1438`, declared
//! `geometry/shape.h:198` and `:200`): one that fills in a minimum
//! translation vector and one that fills in the actual gap and a point
//! near the collision. You cannot ask for both in the same call, so this
//! module keeps them apart as [`collide_mtv`] and [`collide`], with
//! [`collides`] for the boolean only form the router's fast path uses
//! (`pcbnew/router/pns_item.cpp:280`).
//!
//! # What each form answers
//!
//! | Function | `actual` | `location` | vector |
//! |---|---|---|---|
//! | [`collides`] | not computed | not computed | not computed |
//! | [`collide`] | meaningful | meaningful | not computed |
//! | [`collide_mtv`] | not computed | not computed | meaningful |
//!
//! # The predicate
//!
//! `clearance` is an edge to edge separation in nanometres, added to the
//! shape inflation terms before the geometric test. The comparison is
//! **strict**, `dist_sq == 0 || dist_sq < min_dist_sq`
//! (`shape_collisions.cpp:49`, `:129`), so a gap of exactly `clearance` is
//! not a collision while a gap of exactly zero always is, even at
//! `clearance == 0`. That strictness is the hinge the router's hull
//! geometry hangs on: `PNS::ITEM::collideSimple` subtracts one nanometre
//! from the clearance it passes because "the hulls are built to exactly
//! the clearance distance" (`pns_item.cpp:246`, `:249`), and the
//! walkaround can fail to terminate if the two disagree.
//!
//! `actual` is the edge to edge gap, clamped to at least zero, and is only
//! meaningful when there is a collision. `location` is "a point near the
//! collision" and **which** shape it lies on varies per cell, exactly as
//! it does in KiCad: the circle versus circle cell writes the midpoint of
//! the two centres, on neither shape (`shape_collisions.cpp:55`); the
//! rectangle versus circle cell writes a point on the rectangle
//! (`:132`); the polyline cells write a point on the first chain (`:412`)
//! or a vertex in the containment branches (`:364`, `:369`). No contract
//! is promised beyond "near", because KiCad promises none and the router
//! only feeds it to point queries (`pns_item.cpp:252`, `:255`).
//!
//! # The MTV sign convention
//!
//! **The returned vector is the translation to apply to `b`, the second
//! argument, to separate it from `a`.** One convention, everywhere.
//!
//! KiCad is inconsistent here. Its prevailing convention is the same one
//! (`shape_collisions.cpp:58`, `:142`, `:339`), but the circle versus
//! polyline routine writes the pushout applied to **A** instead
//! (`:323`), and several switch cells swap their operands without
//! negating the vector (`:1140`, `:1143`, `:1173`, `:1179`, `:1186`,
//! `:1207`, `:1210`, `:1213`). The router compensates with a double
//! negation and an in source comment admitting the problem
//! (`pcbnew/router/pns_shove.cpp:1244`). Note 01 section 14.6 asks for one
//! convention, enforced. The cells where KiCad's result had to be negated
//! to get there are listed on [`collide_mtv`].
//!
//! A returned vector of `(0, 0)` means "the shapes collide but this pair
//! cannot produce a translation vector". Only five of the pairs can:
//! circle versus circle, circle versus rectangle, circle versus capsule
//! and circle versus polyline or polygon, in either order. Every other
//! pair asserts the vector away in KiCad (`shape_collisions.cpp:353`,
//! `:471`, `:531`, `:547`, `:563`) and answers zero here. A zero vector
//! also comes out of a genuine degeneracy, two concentric circles, where
//! `Resize` on the zero vector returns zero
//! (`math/vector2d.h:383`); `PNS::VIA::PushoutForce` treats both the same
//! way and gives up (`pcbnew/router/pns_via.cpp:139`, `:172`).
//!
//! # Square roots and widths
//!
//! Two deliberate numeric decisions, both flagged in note 01 section 9.6:
//!
//! - Where KiCad narrows a square root to an integer to produce a
//!   **distance**, `(int) sqrt( dist_sq )`, this uses the exact integer
//!   square root that [`crate::geometry::seg`] and
//!   [`crate::geometry::line_chain`] already use. The two agree for every
//!   distance below about 95 mm, where `f64` still represents the squared
//!   value exactly; above that KiCad's rounding can land one nanometre
//!   higher. Keeping one square root across the crate matters more than
//!   matching KiCad on distances no clearance rule can produce.
//! - Where KiCad uses a square root inside an **MTV magnitude**, it stays
//!   in `f64` and the whole expression is truncated at the end, biases
//!   included. Those expressions are reproduced verbatim in `f64`, because
//!   the `+ 3` and the two `+ 1` are pure fudge whose exact value the
//!   shove's convergence depends on.
//!
//! Inflation sums (`clearance + radius`, `clearance + width / 2`) are
//! computed in `i64` and the products in `i64`. KiCad computes them in
//! `int` and lets them wrap, which `SHAPE::GetClearance` makes reachable
//! by passing `INT_MAX / 2` (`libs/kimath/src/geometry/shape.cpp:94`);
//! in Rust that would panic in a debug build, which is worse. Note 01
//! section 14.6 asks for the widening.
//!
//! Half widths are inconsistent in KiCad and the inconsistency is
//! reproduced: `SHAPE_SEGMENT`'s own routines round the half width **up**,
//! `( width + 1 ) / 2` (`geometry/shape_segment.h:83`, `:90`), while the
//! dispatch file truncates the *other* operand's half width,
//! `width / 2` (`shape_collisions.cpp:336`, `:536`, `:552`, `:571`). A
//! capsule versus capsule collision therefore uses
//! `(width_a + 1) / 2 + clearance + width_b / 2`, which is asymmetric by
//! one nanometre for odd widths.
//!
//! The nominal width carried by a [`LineChain`] takes no part in any of
//! this, which is why `PNS::ITEM::collideSimple` folds line widths into
//! the clearance instead (`pcbnew/router/pns_item.cpp:159`).
//!
//! # Not ported
//!
//! - Every arc and ellipse cell, and the polygon set short circuit
//!   (`shape_collisions.cpp:1050`), because [`Shape`] has no such
//!   variants. The arc rescue block inside the polyline versus polyline
//!   cell (`:426`) is likewise dead here.
//! - `SHAPE_NULL`, which the router never builds. See the
//!   [`crate::geometry::shape`] module documentation.

use crate::geometry::box2::Box2;
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::{Seg, distance_from_squared};
use crate::geometry::shape::{Shape, rect_corners, rect_outline};
use crate::geometry::vec2::{Vec2, Vec2L};

/// A collision, as [`collide`] reports it.
///
/// Port of the `aActual` and `aLocation` out parameters of
/// `SHAPE::Collide`, `libs/kimath/include/geometry/shape.h:200`, which
/// KiCad writes only when the call returns true. Here their presence *is*
/// the collision, so there is no way to read a stale value out of a call
/// that did not collide, which the KiCad unit tests do by accident
/// (`qa/tests/libs/kimath/geometry/test_shape_compound_collision.cpp:87`).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct ShapeCollision {
  /// The edge to edge gap in nanometres, clamped to at least zero.
  ///
  /// Zero means the shapes touch or overlap. A positive value means they
  /// are apart but closer than the clearance.
  pub actual: i32,
  /// A point near the collision.
  ///
  /// Which shape it lies on depends on the pair; see the module
  /// documentation.
  pub location: Vec2,
}

/// What a collision query asks the dispatcher to compute.
///
/// KiCad expresses this with three out parameter pointers, of which
/// `aMTV` is mutually exclusive with the other two through the public API
/// (`libs/kimath/include/geometry/shape.h:198`, `:200`). The three
/// combinations that KiCad can actually be asked for are exactly these
/// three variants, and the cells branch on them the way KiCad branches on
/// its null checks.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum Request {
  /// `aActual`, `aLocation` and `aMTV` all null: only the boolean matters.
  Boolean,
  /// `aActual` and `aLocation` both non null, `aMTV` null.
  Actual,
  /// `aMTV` non null, `aActual` and `aLocation` null.
  Mtv,
}

impl Request {
  /// Whether the caller asked for the gap and the location.
  ///
  /// Stands for KiCad's `aActual` and `aLocation` being non null, which
  /// the dispatch file always tests together as
  /// `aActual || aLocation ? &x : nullptr`
  /// (`libs/kimath/src/geometry/shape_collisions.cpp:270`).
  fn wants_actual(self) -> bool {
    matches!(self, Self::Actual)
  }

  /// Whether the caller asked for a minimum translation vector.
  fn wants_mtv(self) -> bool {
    matches!(self, Self::Mtv)
  }
}

/// Everything a cell can produce, before the entry point drops the fields
/// the caller did not ask for.
///
/// `actual` is filled in only when [`Request::wants_actual`] and `mtv`
/// only when [`Request::wants_mtv`], which is what makes the compound
/// aggregation in [`collide_shapes`] behave like KiCad's, where the
/// corresponding locals stay at their zero initialised values when the
/// pointers are null
/// (`libs/kimath/src/geometry/shape_collisions.cpp:1330`). `location` is
/// filled in opportunistically, wherever the cell had already computed
/// the point for its own use; it is unobservable when the caller did not
/// ask for it, since [`collides`] and [`collide_mtv`] both drop it.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
struct Outcome {
  /// The edge to edge gap, clamped to at least zero.
  actual: i32,
  /// A point near the collision.
  location: Vec2,
  /// The translation that separates the second operand from the first.
  mtv: Vec2,
}

/// A [`Shape::Circle`] taken apart, so the cells take four arguments
/// instead of six.
#[derive(Copy, Clone, Debug)]
struct CircleRef {
  /// The centre.
  center: Vec2,
  /// The radius in nanometres.
  radius: i32,
}

/// A [`Shape::Rect`] taken apart.
#[derive(Copy, Clone, Debug)]
struct RectRef {
  /// The corner with the smaller coordinates.
  origin: Vec2,
  /// The width and height.
  size: Vec2,
  /// The corner radius, which the collision cells branch on.
  radius: i32,
}

/// A [`Shape::Segment`] taken apart.
#[derive(Copy, Clone, Debug)]
struct SegmentRef {
  /// The spine of the capsule.
  seg: Seg,
  /// The full width in nanometres.
  width: i32,
}

// -------------------------------------------------------------------
// Entry points
// -------------------------------------------------------------------

/// Whether two shapes come closer to each other than a clearance.
///
/// Port of `SHAPE::Collide( const SHAPE*, int )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:1438` called with no out
/// parameters. This is the router's fast path
/// (`pcbnew/router/pns_item.cpp:280`).
///
/// It answers the same question as `collide(..).is_some()` and is only
/// separate because one cell, rectangle versus rectangle, substitutes an
/// inclusive bounding box test when the clearance is zero and nothing else
/// was asked for (`shape_collisions.cpp:583`). That shortcut is part of
/// the ported cell, so it needs a caller that can reach it.
pub fn collides(a: &Shape, b: &Shape, clearance: i32) -> bool {
  collide_shapes(a, b, clearance, Request::Boolean).is_some()
}

/// The gap between two shapes and a point near it, when they collide.
///
/// Port of `SHAPE::Collide( const SHAPE*, int, int*, VECTOR2I* )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:1438`, which
/// `PNS::ITEM::collideSimple` uses on its slow path so it can test
/// castellation and net tie exclusions at the collision point
/// (`pcbnew/router/pns_item.cpp:249`).
///
/// The returned `mtv` of the underlying dispatch is not available in this
/// form, because KiCad cannot compute the gap and the translation vector
/// in one call. Use [`collide_mtv`] for that.
pub fn collide(a: &Shape, b: &Shape, clearance: i32) -> Option<ShapeCollision> {
  collide_shapes(a, b, clearance, Request::Actual).map(|outcome| {
    ShapeCollision {
      actual: outcome.actual,
      location: outcome.location,
    }
  })
}

/// The translation that separates `b` from `a`, when they collide.
///
/// Port of `SHAPE::Collide( const SHAPE*, int, VECTOR2I* )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:1432`. The router
/// consumes it in exactly two places, `PNS::VIA::PushoutForce`
/// (`pcbnew/router/pns_via.cpp:126`) and `PNS::SHOVE::onCollidingVia`
/// (`pcbnew/router/pns_shove.cpp:1180`), both of which take the maximum
/// magnitude across the layers a padstack spans.
///
/// `Some(Vec2::new(0, 0))` means the shapes collide but no translation
/// vector is available: either the pair is one KiCad cannot produce one
/// for, or the geometry is degenerate. See the module documentation.
///
/// # The sign
///
/// The vector displaces **`b`**. Applying it to `b` and asking again gives
/// no collision, in every cell that produces a non zero vector. Calling
/// with the operands mirrored gives the negated vector.
///
/// KiCad's own result had to be negated in two cells to get there, the
/// ones where its circle versus polyline routine writes the pushout for
/// `A` instead of `B` (`shape_collisions.cpp:323`) and is reached without
/// an operand swap:
///
/// - circle against polyline (`shape_collisions.cpp:1113`);
/// - circle against polygon (`shape_collisions.cpp:1120`).
///
/// The two mirrored cells, polyline against circle
/// (`shape_collisions.cpp:1143`) and polygon against circle (`:1210`),
/// are left alone: KiCad swaps the operands there **without** negating,
/// which cancels the first mistake and lands on the right sign by
/// accident.
///
/// The magnitude is not a true minimum overlap distance. It is the
/// penetration depth plus the clearance plus a bias, and for the polyline
/// cells it is the result of a single order dependent relaxation pass over
/// the segments, in chain order, capped at five one nanometre correction
/// steps and with no convergence check
/// (`libs/kimath/src/geometry/shape_collisions.cpp:168`, `:314`). A
/// tidier implementation would change how the shove behaves.
pub fn collide_mtv(a: &Shape, b: &Shape, clearance: i32) -> Option<Vec2> {
  collide_shapes(a, b, clearance, Request::Mtv).map(|outcome| outcome.mtv)
}

/// The gap between a shape and a point, when they collide.
///
/// Port of `SHAPE::Collide( const VECTOR2I&, int, int*, VECTOR2I* )`,
/// `libs/kimath/include/geometry/shape.h:179`, whose default body forwards
/// to the segment form with a degenerate segment. Two shapes override it:
/// a capsule measures the point directly (`shape_segment.h:100`) and a
/// polyline or polygon takes the containment shortcut with the clearance
/// as the accuracy (`shape.h:324`,
/// `libs/kimath/src/geometry/shape_line_chain.cpp:426`).
///
/// `PNS::NODE::QueryEdgeExclusions` (`pcbnew/router/pns_node.cpp:801`) and
/// the length tuning fallback (`pcbnew/router/pns_topology.cpp:588`) are
/// the router's callers.
pub fn collide_point(
  shape: &Shape,
  point: Vec2,
  clearance: i32,
) -> Option<ShapeCollision> {
  shape_collide_point(shape, point, clearance, Request::Actual).map(|outcome| {
    ShapeCollision {
      actual: outcome.actual,
      location: outcome.location,
    }
  })
}

/// The gap between a shape and a segment, when they collide.
///
/// Port of the `SHAPE::Collide( const SEG&, int, int*, VECTOR2I* )` pure
/// virtual, `libs/kimath/include/geometry/shape.h:213`. This is the
/// primitive every shape has to implement and most of the pair cells are
/// built out of. The optimizer calls it directly when it checks whether a
/// candidate breakout is swallowed by its pad
/// (`pcbnew/router/pns_optimizer.cpp:1139`).
///
/// Note that the segment is treated as having no width. A capsule needs
/// [`collide`] against a [`Shape::Segment`].
pub fn collide_seg(
  shape: &Shape,
  seg: &Seg,
  clearance: i32,
) -> Option<ShapeCollision> {
  shape_collide_seg(shape, seg, clearance, Request::Actual).map(|outcome| {
    ShapeCollision {
      actual: outcome.actual,
      location: outcome.location,
    }
  })
}

// -------------------------------------------------------------------
// Compound expansion
// -------------------------------------------------------------------

/// The compound aware dispatcher.
///
/// Port of `collideShapes`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:1307`. It expands
/// compound operands into per subshape pairs, keeps the **minimum**
/// `actual` across the sub collisions (`:1342`) and the
/// **maximum magnitude** translation vector (`:1348`), and stops early
/// through `canExit` (`:1315`).
///
/// Deliberate extension: KiCad expands one level. A compound inside a
/// compound reaches `collideSingleShapes`, which has no case for it,
/// asserts in debug and returns "no collision" in release (`:1337`,
/// `:1300`, note 01 section 9.1). [`Shape::subshapes`] flattens
/// recursively instead, so nesting behaves like the flat form KiCad's
/// `AddShape` would have built (`geometry/shape_compound.h:85`). Returning
/// "no collision" was not an option: the router would route a trace
/// straight through a complex pad.
fn collide_shapes(
  a: &Shape,
  b: &Shape,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let compound =
    matches!(a, Shape::Compound(_)) || matches!(b, Shape::Compound(_));

  if !compound {
    return collide_single(a, b, clearance, request);
  }

  let left = a.subshapes();
  let right = b.subshapes();

  let mut colliding = false;
  let mut actual = i32::MAX;
  let mut location = Vec2::new(0, 0);
  let mut mtv = Vec2::new(0, 0);

  'search: for element_a in &left {
    for element_b in &right {
      let Some(outcome) =
        collide_single(element_a, element_b, clearance, request)
      else {
        continue;
      };

      colliding = true;

      if outcome.actual < actual {
        actual = outcome.actual;
        location = outcome.location;
      }

      if request.wants_mtv()
        && outcome.mtv.squared_euclidean_norm() > mtv.squared_euclidean_norm()
      {
        mtv = outcome.mtv;
      }

      // `canExit`, shape_collisions.cpp:1315: never when a translation
      // vector was asked for, and only once the gap has bottomed out at
      // zero when the gap was asked for.
      let can_exit = match request {
        Request::Boolean => true,
        Request::Actual => actual == 0,
        Request::Mtv => false,
      };

      if can_exit {
        break 'search;
      }
    }
  }

  colliding.then_some(Outcome {
    actual,
    location,
    mtv,
  })
}

// -------------------------------------------------------------------
// The pair matrix
// -------------------------------------------------------------------

/// The dispatch matrix.
///
/// Port of `collideSingleShapes`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:1047`, as an exhaustive
/// `match` over the pair. KiCad's two level `switch` (`:1065` to `:1298`)
/// falls through to `wxFAIL_MSG` and returns false for any pair it forgot
/// (`:1300`); here a forgotten pair does not compile.
///
/// Each arm cites the KiCad switch line it came from. Where KiCad reaches
/// a cell by reordering the operands, the reordering is reproduced,
/// together with the negation of the translation vector that
/// `CollCaseReversed` performs (`:906`) and that four of the swapping
/// cells omit.
///
/// # Panics
///
/// When either operand is a [`Shape::Compound`]. [`collide_shapes`]
/// flattens those before calling this, and [`Shape::subshapes`] never
/// yields one.
fn collide_single(
  a: &Shape,
  b: &Shape,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  match (a, b) {
    (Shape::Compound(_), _) | (_, Shape::Compound(_)) => {
      unreachable!("compound shapes are flattened before dispatch")
    }

    // shape_collisions.cpp:1074
    (
      Shape::Rect {
        origin: a_origin,
        size: a_size,
        radius: a_radius,
      },
      Shape::Rect {
        origin: b_origin,
        size: b_size,
        radius: b_radius,
      },
    ) => rect_rect(
      RectRef {
        origin: *a_origin,
        size: *a_size,
        radius: *a_radius,
      },
      RectRef {
        origin: *b_origin,
        size: *b_size,
        radius: *b_radius,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1077
    (
      Shape::Rect {
        origin,
        size,
        radius,
      },
      Shape::Circle { center, radius: r },
    ) => rect_circle(
      RectRef {
        origin: *origin,
        size: *size,
        radius: *radius,
      },
      CircleRef {
        center: *center,
        radius: *r,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1080
    (
      Shape::Rect {
        origin,
        size,
        radius,
      },
      Shape::LineChain(chain),
    ) => rect_chain(
      RectRef {
        origin: *origin,
        size: *size,
        radius: *radius,
      },
      chain,
      clearance,
      request,
    ),

    // shape_collisions.cpp:1083
    (
      Shape::Rect {
        origin,
        size,
        radius,
      },
      Shape::Segment { seg, width },
    ) => rect_segment(
      RectRef {
        origin: *origin,
        size: *size,
        radius: *radius,
      },
      SegmentRef {
        seg: *seg,
        width: *width,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1087
    (
      Shape::Rect {
        origin,
        size,
        radius,
      },
      Shape::Simple(polygon),
    ) => rect_chain(
      RectRef {
        origin: *origin,
        size: *size,
        radius: *radius,
      },
      polygon.vertices(),
      clearance,
      request,
    ),

    // shape_collisions.cpp:1107, reversed operands with the vector
    // negated. The negation is what makes the result displace the
    // rectangle, which is `b` here.
    (
      Shape::Circle { center, radius: r },
      Shape::Rect {
        origin,
        size,
        radius,
      },
    ) => negated_mtv(rect_circle(
      RectRef {
        origin: *origin,
        size: *size,
        radius: *radius,
      },
      CircleRef {
        center: *center,
        radius: *r,
      },
      clearance,
      request,
    )),

    // shape_collisions.cpp:1110
    (
      Shape::Circle {
        center: a_center,
        radius: a_radius,
      },
      Shape::Circle {
        center: b_center,
        radius: b_radius,
      },
    ) => circle_circle(
      CircleRef {
        center: *a_center,
        radius: *a_radius,
      },
      CircleRef {
        center: *b_center,
        radius: *b_radius,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1113. KiCad's cell writes the pushout applied
    // to the circle, which is `a` here (`:323`), so the vector is negated
    // to make it displace `b`.
    (Shape::Circle { center, radius }, Shape::LineChain(chain)) => {
      negated_mtv(circle_chain(
        CircleRef {
          center: *center,
          radius: *radius,
        },
        chain,
        clearance,
        request,
      ))
    }

    // shape_collisions.cpp:1116
    (Shape::Circle { center, radius }, Shape::Segment { seg, width }) => {
      circle_segment(
        CircleRef {
          center: *center,
          radius: *radius,
        },
        SegmentRef {
          seg: *seg,
          width: *width,
        },
        clearance,
        request,
      )
    }

    // shape_collisions.cpp:1120, negated for the same reason as `:1113`.
    (Shape::Circle { center, radius }, Shape::Simple(polygon)) => {
      negated_mtv(circle_chain(
        CircleRef {
          center: *center,
          radius: *radius,
        },
        polygon.vertices(),
        clearance,
        request,
      ))
    }

    // shape_collisions.cpp:1140, operands swapped without a negation. The
    // cell produces no vector, so there is nothing to negate.
    (
      Shape::LineChain(chain),
      Shape::Rect {
        origin,
        size,
        radius,
      },
    ) => rect_chain(
      RectRef {
        origin: *origin,
        size: *size,
        radius: *radius,
      },
      chain,
      clearance,
      request,
    ),

    // shape_collisions.cpp:1143, operands swapped without a negation. Two
    // wrongs make a right: the cell writes the pushout for its own first
    // argument, which the swap has made the circle, and the circle is `b`
    // here, so the sign already matches this module's convention.
    (Shape::LineChain(chain), Shape::Circle { center, radius }) => {
      circle_chain(
        CircleRef {
          center: *center,
          radius: *radius,
        },
        chain,
        clearance,
        request,
      )
    }

    // shape_collisions.cpp:1146
    (Shape::LineChain(a_chain), Shape::LineChain(b_chain)) => {
      chain_chain(a_chain, b_chain, clearance, request)
    }

    // shape_collisions.cpp:1149
    (Shape::LineChain(chain), Shape::Segment { seg, width }) => chain_segment(
      chain,
      SegmentRef {
        seg: *seg,
        width: *width,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1153
    (Shape::LineChain(chain), Shape::Simple(polygon)) => {
      chain_chain(chain, polygon.vertices(), clearance, request)
    }

    // shape_collisions.cpp:1173, operands swapped without a negation.
    (
      Shape::Segment { seg, width },
      Shape::Rect {
        origin,
        size,
        radius,
      },
    ) => rect_segment(
      RectRef {
        origin: *origin,
        size: *size,
        radius: *radius,
      },
      SegmentRef {
        seg: *seg,
        width: *width,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1176, reversed operands with the vector
    // negated, which lands on the circle, which is `b` here.
    (Shape::Segment { seg, width }, Shape::Circle { center, radius }) => {
      negated_mtv(circle_segment(
        CircleRef {
          center: *center,
          radius: *radius,
        },
        SegmentRef {
          seg: *seg,
          width: *width,
        },
        clearance,
        request,
      ))
    }

    // shape_collisions.cpp:1179, operands swapped without a negation.
    (Shape::Segment { seg, width }, Shape::LineChain(chain)) => chain_segment(
      chain,
      SegmentRef {
        seg: *seg,
        width: *width,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1182
    (
      Shape::Segment {
        seg: a_seg,
        width: a_width,
      },
      Shape::Segment {
        seg: b_seg,
        width: b_width,
      },
    ) => segment_segment(
      SegmentRef {
        seg: *a_seg,
        width: *a_width,
      },
      SegmentRef {
        seg: *b_seg,
        width: *b_width,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1186, operands swapped without a negation.
    (Shape::Segment { seg, width }, Shape::Simple(polygon)) => chain_segment(
      polygon.vertices(),
      SegmentRef {
        seg: *seg,
        width: *width,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1207, operands swapped without a negation.
    (
      Shape::Simple(polygon),
      Shape::Rect {
        origin,
        size,
        radius,
      },
    ) => rect_chain(
      RectRef {
        origin: *origin,
        size: *size,
        radius: *radius,
      },
      polygon.vertices(),
      clearance,
      request,
    ),

    // shape_collisions.cpp:1210, operands swapped without a negation, and
    // right for the same accidental reason as `:1143`.
    (Shape::Simple(polygon), Shape::Circle { center, radius }) => circle_chain(
      CircleRef {
        center: *center,
        radius: *radius,
      },
      polygon.vertices(),
      clearance,
      request,
    ),

    // shape_collisions.cpp:1213, operands swapped: the polyline takes the
    // first role and the polygon the second, which decides the order of
    // the two containment shortcuts.
    (Shape::Simple(polygon), Shape::LineChain(chain)) => {
      chain_chain(chain, polygon.vertices(), clearance, request)
    }

    // shape_collisions.cpp:1216
    (Shape::Simple(polygon), Shape::Segment { seg, width }) => chain_segment(
      polygon.vertices(),
      SegmentRef {
        seg: *seg,
        width: *width,
      },
      clearance,
      request,
    ),

    // shape_collisions.cpp:1220
    (Shape::Simple(a_polygon), Shape::Simple(b_polygon)) => chain_chain(
      a_polygon.vertices(),
      b_polygon.vertices(),
      clearance,
      request,
    ),
  }
}

/// Flip the sign of a translation vector.
///
/// Port of the negation `CollCaseReversed` performs,
/// `libs/kimath/src/geometry/shape_collisions.cpp:912`.
fn negated_mtv(outcome: Option<Outcome>) -> Option<Outcome> {
  outcome.map(|outcome| Outcome {
    mtv: -outcome.mtv,
    ..outcome
  })
}

// -------------------------------------------------------------------
// Shape against a bare segment or point
// -------------------------------------------------------------------

/// A shape against a segment of no width.
///
/// Port of the `SHAPE::Collide( const SEG&, ... )` virtual,
/// `libs/kimath/include/geometry/shape.h:213`, dispatched per variant.
fn shape_collide_seg(
  shape: &Shape,
  seg: &Seg,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  match shape {
    Shape::Circle { center, radius } => circle_collide_seg(
      CircleRef {
        center: *center,
        radius: *radius,
      },
      seg,
      clearance,
      request,
    ),
    Shape::Rect {
      origin,
      size,
      radius,
    } => rect_collide_seg(
      RectRef {
        origin: *origin,
        size: *size,
        radius: *radius,
      },
      seg,
      clearance,
      request,
    ),
    Shape::Segment { seg: spine, width } => segment_collide_seg(
      SegmentRef {
        seg: *spine,
        width: *width,
      },
      seg,
      clearance,
      request,
    ),
    Shape::Simple(polygon) => {
      chain_collide_seg(polygon.vertices(), seg, clearance, request)
    }
    Shape::LineChain(chain) => {
      chain_collide_seg(chain, seg, clearance, request)
    }
    Shape::Compound(_) => compound_collide_seg(shape, seg, clearance, request),
  }
}

/// A shape against a point.
///
/// Port of `SHAPE::Collide( const VECTOR2I&, ... )`,
/// `libs/kimath/include/geometry/shape.h:179`. The default body wraps the
/// point in a degenerate segment; a capsule
/// (`geometry/shape_segment.h:100`) and a polyline or polygon
/// (`geometry/shape.h:324`) override it with a direct measurement.
fn shape_collide_point(
  shape: &Shape,
  point: Vec2,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  match shape {
    Shape::Segment { seg: spine, width } => segment_collide_point(
      SegmentRef {
        seg: *spine,
        width: *width,
      },
      point,
      clearance,
      request,
    ),
    Shape::Simple(polygon) => {
      chain_collide_point(polygon.vertices(), point, clearance, request)
    }
    Shape::LineChain(chain) => {
      chain_collide_point(chain, point, clearance, request)
    }
    _ => shape_collide_seg(shape, &Seg::new(point, point), clearance, request),
  }
}

/// A compound against a segment of no width.
///
/// Port of `SHAPE_COMPOUND::Collide( const SEG&, ... )`,
/// `libs/kimath/src/geometry/shape_compound.cpp:109`. This is a different
/// aggregation from the one in [`collide_shapes`]: it keeps the smallest
/// gap and, on a tie, the location closest to the start of the argument
/// segment (`:130`).
fn compound_collide_seg(
  shape: &Shape,
  seg: &Seg,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let mut closest = i32::MAX;
  let mut nearest = Vec2::new(0, 0);

  for item in shape.subshapes() {
    let Some(outcome) = shape_collide_seg(item, seg, clearance, request) else {
      continue;
    };

    if outcome.actual < closest {
      nearest = outcome.location;
      closest = outcome.actual;

      if !request.wants_actual() {
        break;
      }
    } else if request.wants_actual() && outcome.actual == closest {
      let candidate = outcome
        .location
        .widening_sub(seg.a)
        .squared_euclidean_norm();
      let incumbent = nearest.widening_sub(seg.a).squared_euclidean_norm();

      if candidate < incumbent {
        nearest = outcome.location;
      }
    }
  }

  if closest == 0 || closest < clearance {
    return Some(Outcome {
      actual: closest,
      location: nearest,
      mtv: Vec2::new(0, 0),
    });
  }

  None
}

/// A polyline or polygon against a segment of no width.
///
/// Port of `SHAPE_LINE_CHAIN_BASE::Collide( const SEG&, ... )`,
/// `libs/kimath/src/geometry/shape_line_chain.cpp:815`, which
/// [`LineChain::collide_seg`] already carries.
fn chain_collide_seg(
  chain: &LineChain,
  seg: &Seg,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  chain.collide_seg(seg, clearance).map(|hit| Outcome {
    actual: if request.wants_actual() {
      hit.actual
    } else {
      0
    },
    location: hit.location,
    mtv: Vec2::new(0, 0),
  })
}

/// A polyline or polygon against a point.
///
/// Port of `SHAPE_LINE_CHAIN_BASE::Collide( const VECTOR2I&, ... )`,
/// `libs/kimath/src/geometry/shape_line_chain.cpp:426`, which
/// [`LineChain::collide_point`] already carries. Note that it runs the
/// containment test with the **clearance** as the accuracy, unlike the
/// segment form.
fn chain_collide_point(
  chain: &LineChain,
  point: Vec2,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  chain.collide_point(point, clearance).map(|hit| Outcome {
    actual: if request.wants_actual() {
      hit.actual
    } else {
      0
    },
    location: hit.location,
    mtv: Vec2::new(0, 0),
  })
}

/// A circle against a segment of no width.
///
/// Port of `SHAPE_CIRCLE::Collide( const SEG&, ... )`,
/// `libs/kimath/include/geometry/shape_circle.h:73`. The gap is measured
/// from the circumference, so a segment passing through the centre reports
/// zero.
///
/// The location is the segment's nearest point to the centre, except when
/// the centre lies exactly **on** the segment, where it is the first
/// intersection of the circle with the segment (`shape_circle.h:82`); the
/// nearest point would be the centre itself there, which is on neither
/// boundary.
///
/// Deviation, harmless: KiCad computes the intersection whenever a
/// location was asked for and then throws it away unless the distance was
/// zero. It is only computed here in the branch that uses it.
fn circle_collide_seg(
  circle: CircleRef,
  seg: &Seg,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let min_dist = i64::from(clearance) + i64::from(circle.radius);
  let nearest = seg.nearest_point_to_point(circle.center);
  let distance_squared =
    nearest.widening_sub(circle.center).squared_euclidean_norm();

  if distance_squared != 0 && distance_squared >= square(min_dist) {
    return None;
  }

  let mut outcome = Outcome {
    actual: 0,
    location: nearest,
    mtv: Vec2::new(0, 0),
  };

  if request.wants_actual() {
    if distance_squared == 0
      && let Some(first) = circle_intersect_seg(circle, seg).first()
    {
      outcome.location = *first;
    }

    outcome.actual = clamped_gap(
      i64::from(distance_from_squared(distance_squared))
        - i64::from(circle.radius),
    );
  }

  Some(outcome)
}

/// A rectangle against a segment of no width.
///
/// Port of `SHAPE_RECT::Collide( const SEG&, ... )`,
/// `libs/kimath/src/geometry/shape_rect.cpp:27`. A segment with either
/// endpoint inside the rectangle collides at zero straight away
/// (`:35`, `:46`); otherwise the four sides are measured and the closest
/// wins, with ties going to the point nearest the start of the argument
/// (`:78`).
///
/// Note that the gap this cell writes is **not** clamped to zero
/// (`shape_rect.cpp:94`), unlike every other cell. It cannot go negative
/// here, since a squared distance is never negative.
fn rect_collide_seg(
  rect: RectRef,
  seg: &Seg,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  if rect.radius > 0 {
    return chain_collide_seg(
      &rect_outline(rect.origin, rect.size),
      seg,
      clearance,
      request,
    );
  }

  let bbox = Box2::from_origin_and_size(
    Vec2L::from(rect.origin),
    Vec2L::from(rect.size),
  );

  for endpoint in [seg.a, seg.b] {
    if bbox.contains_point(Vec2L::from(endpoint)) {
      return Some(Outcome {
        actual: 0,
        location: endpoint,
        mtv: Vec2::new(0, 0),
      });
    }
  }

  let corners = rect_corners(rect.origin, rect.size);
  let mut closest_squared = i64::MAX;
  let mut nearest = Vec2::new(0, 0);

  for (index, corner) in corners.iter().enumerate() {
    let side = Seg::new(*corner, corners[(index + 1) % 4]);
    let side_squared = side.squared_distance_to_segment(seg);

    if side_squared < closest_squared {
      if request.wants_actual() {
        nearest = side.nearest_point_to_segment(seg);
      }

      closest_squared = side_squared;
    } else if request.wants_actual() && side_squared == closest_squared {
      let candidate = side.nearest_point_to_segment(seg);

      if candidate.widening_sub(seg.a).squared_euclidean_norm()
        < nearest.widening_sub(seg.a).squared_euclidean_norm()
      {
        nearest = candidate;
      }
    }
  }

  if closest_squared != 0 && closest_squared >= square(i64::from(clearance)) {
    return None;
  }

  Some(Outcome {
    actual: if request.wants_actual() {
      distance_from_squared(closest_squared)
    } else {
      0
    },
    location: nearest,
    mtv: Vec2::new(0, 0),
  })
}

/// A capsule against a segment of no width.
///
/// Port of `SHAPE_SEGMENT::Collide( const SEG&, ... )`,
/// `libs/kimath/include/geometry/shape_segment.h:78`. The half width is
/// rounded **up** here, `( width + 1 ) / 2`, which is one nanometre more
/// than the dispatch file uses for the *other* operand's half width; see
/// the module documentation.
fn segment_collide_seg(
  capsule: SegmentRef,
  seg: &Seg,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  if seg.a == seg.b {
    return segment_collide_point(capsule, seg.a, clearance, request);
  }

  let half_width = (i64::from(capsule.width) + 1) / 2;
  let min_dist = half_width + i64::from(clearance);
  let distance_squared = capsule.seg.squared_distance_to_segment(seg);

  if distance_squared != 0 && distance_squared >= square(min_dist) {
    return None;
  }

  Some(Outcome {
    actual: if request.wants_actual() {
      clamped_gap(
        i64::from(distance_from_squared(distance_squared)) - half_width,
      )
    } else {
      0
    },
    location: if request.wants_actual() {
      capsule.seg.nearest_point_to_segment(seg)
    } else {
      Vec2::new(0, 0)
    },
    mtv: Vec2::new(0, 0),
  })
}

/// A capsule against a point.
///
/// Port of `SHAPE_SEGMENT::Collide( const VECTOR2I&, ... )`,
/// `libs/kimath/include/geometry/shape_segment.h:100`, the same routine as
/// [`segment_collide_seg`] measured against a point.
fn segment_collide_point(
  capsule: SegmentRef,
  point: Vec2,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let half_width = (i64::from(capsule.width) + 1) / 2;
  let min_dist = half_width + i64::from(clearance);
  let distance_squared = capsule.seg.squared_distance_to_point(point);

  if distance_squared != 0 && distance_squared >= square(min_dist) {
    return None;
  }

  Some(Outcome {
    actual: if request.wants_actual() {
      clamped_gap(
        i64::from(distance_from_squared(distance_squared)) - half_width,
      )
    } else {
      0
    },
    location: if request.wants_actual() {
      capsule.seg.nearest_point_to_point(point)
    } else {
      Vec2::new(0, 0)
    },
    mtv: Vec2::new(0, 0),
  })
}

// -------------------------------------------------------------------
// The cells
// -------------------------------------------------------------------

/// Circle against circle.
///
/// Port of `Collide( const SHAPE_CIRCLE&, const SHAPE_CIRCLE&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:40`. The location is the
/// midpoint of the two centres, which lies on neither circumference
/// (`:55`).
///
/// The translation vector is the analytic radial one: the centre to centre
/// direction resized to the penetration depth plus the `+ 3` bias that
/// carries an in source `fixme` (`:58`). Concentric circles have a zero
/// direction and therefore an unresolvable collision, which
/// `PNS::VIA::PushoutForce` handles explicitly
/// (`pcbnew/router/pns_via.cpp:139`).
fn circle_circle(
  a: CircleRef,
  b: CircleRef,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let min_dist =
    i64::from(clearance) + i64::from(a.radius) + i64::from(b.radius);
  let delta = b.center.widening_sub(a.center);
  let distance_squared = delta.squared_euclidean_norm();

  if distance_squared != 0 && distance_squared >= square(min_dist) {
    return None;
  }

  let mut outcome = Outcome::default();

  if request.wants_actual() {
    outcome.actual = clamped_gap(
      i64::from(distance_from_squared(distance_squared))
        - i64::from(a.radius)
        - i64::from(b.radius),
    );
    outcome.location = midpoint(a.center, b.center);
  }

  if request.wants_mtv() {
    let magnitude = min_dist as f64 - (distance_squared as f64).sqrt() + 3.0;
    outcome.mtv = delta.saturating_to_vec2().resize(truncate_i32(magnitude));
  }

  Some(outcome)
}

/// Rectangle against circle.
///
/// Port of `Collide( const SHAPE_RECT&, const SHAPE_CIRCLE&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:67`. A rectangle with a
/// corner radius hands the whole thing to its outline and **drops the
/// translation vector request** (`:72`, `:76`), which is reproduced: the
/// vector comes back zero.
///
/// Two behaviours worth naming. The containment test is an inclusive axis
/// aligned one (`:98`), so a centre on the boundary counts as inside. And
/// when the centre is inside, the gap reported is the distance to the
/// nearest **side** rather than zero (`:135`), which is a defect KiCad
/// carries and this reproduces; the collision itself is still detected.
///
/// The translation vector is the analytic nearest side one, with two
/// independent `+ 1` biases (`:142`, `:144`). When a vector is asked for
/// the side scan cannot stop early (`:117`), because the true nearest side
/// decides the direction.
fn rect_circle(
  rect: RectRef,
  circle: CircleRef,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  if rect.radius > 0 {
    // `outline.SHAPE::Collide( &aB, aClearance, aActual, aLocation )` at
    // shape_collisions.cpp:76 re-enters the dispatcher, which routes a
    // polyline against a circle back to the circle cell with the operands
    // swapped (`:1143`). The translation vector pointer is not passed on.
    let inner = if request.wants_mtv() {
      Request::Boolean
    } else {
      request
    };

    return circle_chain(
      circle,
      &rect_outline(rect.origin, rect.size),
      clearance,
      inner,
    )
    .map(|outcome| Outcome {
      mtv: Vec2::new(0, 0),
      ..outcome
    });
  }

  let center = circle.center;
  let min_dist = i64::from(clearance) + i64::from(circle.radius);
  let min_dist_squared = square(min_dist);
  let corners = rect_corners(rect.origin, rect.size);

  let inside = i64::from(center.x) >= i64::from(rect.origin.x)
    && i64::from(center.x) <= i64::from(rect.origin.x) + i64::from(rect.size.x)
    && i64::from(center.y) >= i64::from(rect.origin.y)
    && i64::from(center.y) <= i64::from(rect.origin.y) + i64::from(rect.size.y);

  // shape_collisions.cpp:101.
  if inside && request == Request::Boolean {
    return Some(Outcome::default());
  }

  let mut nearest_squared = i64::MAX;
  let mut nearest = Vec2::new(0, 0);

  for (index, corner) in corners.iter().enumerate() {
    let side = Seg::new(*corner, corners[(index + 1) % 4]);
    let projected = side.nearest_point_to_point(center);
    let side_squared = projected.widening_sub(center).squared_euclidean_norm();

    if side_squared >= nearest_squared {
      continue;
    }

    nearest = projected;
    nearest_squared = side_squared;

    if request.wants_mtv() {
      continue;
    }

    if nearest_squared == 0 {
      break;
    }

    if nearest_squared < min_dist_squared && !request.wants_actual() {
      break;
    }
  }

  if !inside && nearest_squared != 0 && nearest_squared >= min_dist_squared {
    return None;
  }

  let mut outcome = Outcome {
    actual: 0,
    location: nearest,
    mtv: Vec2::new(0, 0),
  };

  if request.wants_actual() {
    outcome.actual = clamped_gap(
      i64::from(distance_from_squared(nearest_squared))
        - i64::from(circle.radius),
    );
  }

  if request.wants_mtv() {
    let delta = center.widening_sub(nearest).saturating_to_vec2();
    let root = (nearest_squared as f64).sqrt();

    outcome.mtv = if inside {
      -delta.resize(truncate_i32((min_dist as f64 + 1.0 + root).abs() + 1.0))
    } else {
      delta.resize(truncate_i32((min_dist as f64 + 1.0 - root).abs() + 1.0))
    };
  }

  Some(outcome)
}

/// The one nanometre step search that pushes a circle off a segment.
///
/// Port of `pushoutForce`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:154`. It resizes the
/// centre to nearest point direction to the penetration depth and then
/// walks the length up in one nanometre steps until the moved centre is at
/// least `min_dist` from the segment, giving up after five steps
/// (`:168`). Giving up returns the last, insufficient, vector rather than
/// nothing.
///
/// The returned vector displaces the **circle**, which is why every caller
/// negates it before handing it out (`:339`).
///
/// Note that the test that decides whether to search at all is
/// `dist < min_dist` on the **rounded** distance (`:165`), not the strict
/// squared comparison the collision predicate uses, so a circle that
/// collides can still get a zero vector back.
///
/// Deviation: KiCad measures `nearest - c` in `i32`, which wraps for
/// points more than 2.1 m apart. This measures it in `i64`. The two agree
/// wherever a collision is possible.
fn pushout_force(circle: CircleRef, seg: &Seg, clearance: i32) -> Vec2 {
  let nearest = seg.nearest_point_to_point(circle.center);
  let distance = nearest.widening_sub(circle.center).euclidean_norm();
  let min_dist = i64::from(clearance) + i64::from(circle.radius);

  if distance >= min_dist {
    return Vec2::new(0, 0);
  }

  let direction = circle.center.widening_sub(nearest).saturating_to_vec2();
  let mut force = Vec2::new(0, 0);

  for correction in 0..5 {
    force = direction.resize(saturate_i32(min_dist - distance + correction));

    if i64::from(seg.distance_to_point(circle.center + force)) >= min_dist {
      break;
    }
  }

  force
}

/// Circle against polyline or polygon.
///
/// Port of
/// `Collide( const SHAPE_CIRCLE&, const SHAPE_LINE_CHAIN_BASE&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:236`. A centre inside a
/// closed chain collides at zero and reports itself as the location
/// (`:246`).
///
/// The translation vector is the iterative pushout described in note 01
/// section 9.4: if the centre started inside the chain it is first thrown
/// out through the nearest segment and one radius past it (`:305`), and
/// then a **single ordered pass** over the segments accumulates
/// [`pushout_force`] contributions, moving the circle as it goes
/// (`:314`). There is no convergence check and the result depends on the
/// segment order, so a tidier implementation would change how the shove
/// behaves.
///
/// The vector this cell produces displaces the **circle**, KiCad's one
/// exception to its own convention (`:323`). The callers in
/// [`collide_single`] fix the sign.
fn circle_chain(
  circle: CircleRef,
  chain: &LineChain,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let mut closest = i32::MAX;
  let mut nearest = Vec2::new(0, 0);
  let mut closest_mtv_seg = None;

  if chain.is_closed() && chain.point_inside(circle.center, 0) {
    nearest = circle.center;
    closest = 0;

    if request.wants_mtv() {
      let mut closest_mtv_dist = i32::MAX;

      for index in 0..chain.segment_count() {
        let distance = chain.segment(index).distance_to_point(circle.center);

        if distance < closest_mtv_dist {
          closest_mtv_dist = distance;
          closest_mtv_seg = Some(index);
        }
      }
    }
  } else {
    for index in 0..chain.segment_count() {
      let segment = chain.segment(index);
      let Some(hit) = circle_collide_seg(circle, &segment, clearance, request)
      else {
        continue;
      };

      if hit.actual < closest {
        nearest = hit.location;
        closest = hit.actual;
      }

      if closest == 0 || !request.wants_actual() {
        break;
      }
    }
  }

  if closest != 0 && closest >= clearance {
    return None;
  }

  let mut outcome = Outcome {
    actual: closest,
    location: nearest,
    mtv: Vec2::new(0, 0),
  };

  if request.wants_mtv() {
    let mut moved = circle;
    let mut total = Vec2::new(0, 0);
    let mut force = Vec2::new(0, 0);

    if let Some(index) = closest_mtv_seg {
      let segment = chain.segment(index);
      let projected = segment.nearest_point_to_point(circle.center);
      let outwards = projected - circle.center;

      force = outwards + outwards.resize(circle.radius);
    }

    moved.center += force;
    total += force;

    for index in 0..chain.segment_count() {
      let force = pushout_force(moved, &chain.segment(index), clearance);

      moved.center += force;
      total += force;
    }

    outcome.mtv = total;
  }

  Some(outcome)
}

/// Circle against capsule.
///
/// Port of `Collide( const SHAPE_CIRCLE&, const SHAPE_SEGMENT&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:333`. The capsule's half
/// width folds into the clearance, **truncating** (`:336`), and comes back
/// off the reported gap afterwards (`:342`).
///
/// The translation vector is the negated [`pushout_force`], so it
/// displaces the capsule, which is the second operand (`:339`).
fn circle_segment(
  circle: CircleRef,
  capsule: SegmentRef,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let half_width = i64::from(capsule.width) / 2;
  let folded = saturate_i32(i64::from(clearance) + half_width);
  let mut outcome = circle_collide_seg(circle, &capsule.seg, folded, request)?;

  if request.wants_mtv() {
    outcome.mtv = -pushout_force(circle, &capsule.seg, folded);
  }

  if request.wants_actual() {
    outcome.actual = clamped_gap(i64::from(outcome.actual) - half_width);
  }

  Some(outcome)
}

/// Polyline or polygon against polyline or polygon.
///
/// Port of `Collide( const SHAPE_LINE_CHAIN_BASE&,
/// const SHAPE_LINE_CHAIN_BASE&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:351`. No translation
/// vector: KiCad asserts the request away (`:353`).
///
/// Order matters. The first containment shortcut asks whether vertex zero
/// of `a` is inside a closed `b` (`:363`), and only then the other way
/// round (`:368`), so the reported location differs between the two
/// orders when both hold.
///
/// The segment lists are sorted by start point before the double loop
/// (`:396`), and the inner `break`s exit only the inner loop (`:412` to
/// `:419`), both of which are reproduced.
///
/// Deviations: KiCad's `std::sort` is not stable, so its order among
/// segments sharing a start point is unspecified; this uses a stable sort
/// so the answer is deterministic, as `DESIGN.md` section 8 requires. The
/// arc rescue block at `:426` is not ported, because a [`LineChain`] here
/// carries no arcs.
fn chain_chain(
  a: &LineChain,
  b: &LineChain,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let mut closest = i32::MAX;
  let mut nearest = Vec2::new(0, 0);

  if b.is_closed() && a.point_count() > 0 && b.point_inside(a.point(0), 0) {
    closest = 0;
    nearest = a.point(0);
  } else if a.is_closed()
    && b.point_count() > 0
    && a.point_inside(b.point(0), 0)
  {
    closest = 0;
    nearest = b.point(0);
  } else {
    let a_segments = sorted_segments(a);
    let b_segments = sorted_segments(b);

    for a_segment in &a_segments {
      for b_segment in &b_segments {
        let hit = a_segment.collide(b_segment, clearance);

        if !hit.collides {
          continue;
        }

        let distance = if request.wants_actual() {
          hit.actual
        } else {
          0
        };

        if distance < closest {
          nearest = a_segment.nearest_point_to_segment(b_segment);
          closest = distance;
        }

        if closest == 0 || !request.wants_actual() {
          break;
        }
      }
    }
  }

  if closest != 0 && closest >= clearance {
    return None;
  }

  Some(Outcome {
    actual: closest,
    location: nearest,
    mtv: Vec2::new(0, 0),
  })
}

/// The segments of a chain, sorted by start point.
///
/// Port of the `seg_sort` lambda and the two `std::sort` calls,
/// `libs/kimath/src/geometry/shape_collisions.cpp:390` to `:397`.
fn sorted_segments(chain: &LineChain) -> Vec<Seg> {
  let mut segments: Vec<Seg> = (0..chain.segment_count())
    .map(|i| chain.segment(i))
    .collect();

  segments.sort_by_key(|segment| (segment.a.x, segment.a.y));
  segments
}

/// Rectangle against polyline or polygon.
///
/// Port of
/// `Collide( const SHAPE_RECT&, const SHAPE_LINE_CHAIN_BASE&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:469`. A rectangle with a
/// corner radius delegates to its outline (`:472`). No translation vector
/// (`:474`).
///
/// The containment shortcut asks whether the rectangle's **centre** is
/// inside a closed chain (`:483`), which means a rectangle whose centre
/// sits outside a chain it swallows is only caught by the side scan.
fn rect_chain(
  rect: RectRef,
  chain: &LineChain,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  if rect.radius > 0 {
    return chain_chain(
      &rect_outline(rect.origin, rect.size),
      chain,
      clearance,
      request,
    );
  }

  let mut closest = i32::MAX;
  let mut nearest = Vec2::new(0, 0);
  let center = rect_center(rect);

  if chain.is_closed() && chain.point_inside(center, 0) {
    nearest = center;
    closest = 0;
  } else {
    for index in 0..chain.segment_count() {
      let segment = chain.segment(index);
      let Some(hit) = rect_collide_seg(rect, &segment, clearance, request)
      else {
        continue;
      };

      if hit.actual < closest {
        nearest = hit.location;
        closest = hit.actual;
      }

      if closest == 0 || !request.wants_actual() {
        break;
      }
    }
  }

  if closest != 0 && closest >= clearance {
    return None;
  }

  Some(Outcome {
    actual: closest,
    location: nearest,
    mtv: Vec2::new(0, 0),
  })
}

/// Capsule against capsule.
///
/// Port of `Collide( const SHAPE_SEGMENT&, const SHAPE_SEGMENT&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:529`. The second
/// capsule's half width folds into the clearance truncating (`:536`) while
/// the first capsule's own routine rounds its half width up, so the
/// effective separation is
/// `(width_a + 1) / 2 + clearance + width_b / 2`.
fn segment_segment(
  a: SegmentRef,
  b: SegmentRef,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let half_width = i64::from(b.width) / 2;
  let folded = saturate_i32(i64::from(clearance) + half_width);
  let mut outcome = segment_collide_seg(a, &b.seg, folded, request)?;

  if request.wants_actual() {
    outcome.actual = clamped_gap(i64::from(outcome.actual) - half_width);
  }

  Some(outcome)
}

/// Polyline or polygon against capsule.
///
/// Port of
/// `Collide( const SHAPE_LINE_CHAIN_BASE&, const SHAPE_SEGMENT&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:545`. The capsule's half
/// width folds into the clearance truncating (`:552`) and comes back off
/// the gap (`:555`). The chain's own nominal width plays no part.
fn chain_segment(
  chain: &LineChain,
  capsule: SegmentRef,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  let half_width = i64::from(capsule.width) / 2;
  let folded = saturate_i32(i64::from(clearance) + half_width);
  let mut outcome = chain_collide_seg(chain, &capsule.seg, folded, request)?;

  if request.wants_actual() {
    outcome.actual = clamped_gap(i64::from(outcome.actual) - half_width);
  }

  Some(outcome)
}

/// Rectangle against capsule.
///
/// Port of `Collide( const SHAPE_RECT&, const SHAPE_SEGMENT&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:561`. A rectangle with a
/// corner radius delegates to its outline (`:564`), which lands in
/// [`chain_segment`].
fn rect_segment(
  rect: RectRef,
  capsule: SegmentRef,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  if rect.radius > 0 {
    return chain_segment(
      &rect_outline(rect.origin, rect.size),
      capsule,
      clearance,
      request,
    );
  }

  let half_width = i64::from(capsule.width) / 2;
  let folded = saturate_i32(i64::from(clearance) + half_width);
  let mut outcome = rect_collide_seg(rect, &capsule.seg, folded, request)?;

  if request.wants_actual() {
    outcome.actual = clamped_gap(i64::from(outcome.actual) - half_width);
  }

  Some(outcome)
}

/// Rectangle against rectangle.
///
/// Port of `Collide( const SHAPE_RECT&, const SHAPE_RECT&, ... )`,
/// `libs/kimath/src/geometry/shape_collisions.cpp:580`. Both outlines go
/// through [`chain_chain`], except in the one case where the clearance is
/// zero, neither rectangle is rounded and nothing but a boolean was asked
/// for: there KiCad substitutes an **inclusive** bounding box overlap test
/// (`:589`), which is why [`collides`] exists as a separate entry point.
fn rect_rect(
  a: RectRef,
  b: RectRef,
  clearance: i32,
  request: Request,
) -> Option<Outcome> {
  if clearance != 0
    || request != Request::Boolean
    || a.radius > 0
    || b.radius > 0
  {
    return chain_chain(
      &rect_outline(a.origin, a.size),
      &rect_outline(b.origin, b.size),
      clearance,
      request,
    );
  }

  let a_box =
    Box2::from_origin_and_size(Vec2L::from(a.origin), Vec2L::from(a.size));
  let b_box =
    Box2::from_origin_and_size(Vec2L::from(b.origin), Vec2L::from(b.size));

  a_box.intersects(&b_box).then(Outcome::default)
}

// -------------------------------------------------------------------
// Small helpers
// -------------------------------------------------------------------

/// The intersections of a circle with an infinite line.
///
/// Port of `CIRCLE::IntersectLine`,
/// `libs/kimath/src/geometry/circle.cpp:322`. The centre is projected onto
/// the line, and the chord half length comes from Pythagoras. A projection
/// within [`Shape::MIN_PRECISION_IU`] of the circumference counts as
/// tangent and answers one point (`:353`).
///
/// Deviation: the chord half length uses this crate's exact integer square
/// root where KiCad truncates an `f64` one (`:363`), as the module
/// documentation explains.
fn circle_intersect_line(circle: CircleRef, line: &Seg) -> Vec<Vec2> {
  let projected = line.line_project(circle.center);
  let center_distance =
    (Vec2L::from(projected) - Vec2L::from(circle.center)).euclidean_norm();
  let radius = i64::from(circle.radius);
  let tolerance = i64::from(Shape::MIN_PRECISION_IU);

  if center_distance > radius + tolerance {
    return Vec::new();
  }

  if center_distance >= radius - tolerance {
    return vec![projected];
  }

  let half_chord =
    distance_from_squared(radius * radius - center_distance * center_distance);
  let along = (line.b - line.a).resize(half_chord);

  vec![along + projected, -along + projected]
}

/// The intersections of a circle with a segment.
///
/// Port of `CIRCLE::Intersect( const SEG& )`,
/// `libs/kimath/src/geometry/circle.cpp:308`, which filters the line
/// intersections by `SEG::Contains`, itself a squared tolerance of three
/// (`seg.cpp:623`).
fn circle_intersect_seg(circle: CircleRef, seg: &Seg) -> Vec<Vec2> {
  circle_intersect_line(circle, seg)
    .into_iter()
    .filter(|point| seg.contains_point(*point))
    .collect()
}

/// The centre of a rectangle.
///
/// Port of `SHAPE::Centre` for a rectangle,
/// `libs/kimath/include/geometry/shape.h:230` through
/// `libs/kimath/include/math/box2.h:94`: the top left corner plus half the
/// size, with an integer division that truncates towards zero.
fn rect_center(rect: RectRef) -> Vec2 {
  Vec2::new(
    saturate_i32(i64::from(rect.origin.x) + i64::from(rect.size.x) / 2),
    saturate_i32(i64::from(rect.origin.y) + i64::from(rect.size.y) / 2),
  )
}

/// The midpoint of two points, rounded half away from zero.
///
/// Port of the `( a + b ) / 2` forms at
/// `libs/kimath/src/geometry/shape_collisions.cpp:55`. `VECTOR2I` has no
/// integer `operator/`, only `operator/( double )`
/// (`math/vector2d.h:524`), so the division goes through `KiROUND` and
/// rounds half away from zero rather than truncating.
///
/// Deviation: KiCad adds the two points in `i32` and can overflow before
/// the divide (note 01 section 9.6). The sum is taken in `i64` here.
fn midpoint(a: Vec2, b: Vec2) -> Vec2 {
  let sum = Vec2L::from(a) + Vec2L::from(b);

  Vec2::new(
    saturate_i32(half_rounded(sum.x)),
    saturate_i32(half_rounded(sum.y)),
  )
}

/// Half of a value, rounded half away from zero.
///
/// The scalar behind [`midpoint`], matching `KiROUND( x / 2.0 )`.
fn half_rounded(value: i64) -> i64 {
  if value >= 0 {
    (value + 1) / 2
  } else {
    (value - 1) / 2
  }
}

/// A squared length, saturating instead of wrapping.
///
/// Port of `SEG::Square`,
/// `libs/kimath/include/geometry/seg.h:119`, which multiplies in `i64`.
///
/// The operands here are sums of at most three `i32` values, so the
/// largest they can reach is about `6.4e9` and the exact square would need
/// one bit more than `i64` offers. Saturating is the right answer there:
/// the value it is compared against is a squared distance built from `i32`
/// coordinates, which cannot exceed `i64::MAX`, so a saturated bound
/// always compares as "further than any distance", which is what an
/// enormous clearance means. No board comes within three orders of
/// magnitude of it.
fn square(value: i64) -> i64 {
  value.saturating_mul(value)
}

/// A gap, clamped to at least zero and narrowed to nanometres.
///
/// Port of the `std::max( 0, ... )` that guards every `aActual` write
/// (`libs/kimath/src/geometry/shape_collisions.cpp:52`, `:135`, `:342`,
/// `:539`).
fn clamped_gap(value: i64) -> i32 {
  saturate_i32(value.max(0))
}

/// Truncate a floating point magnitude towards zero into nanometres.
///
/// Port of the implicit `double` to `int` conversion at every `Resize`
/// call in the dispatch file (`shape_collisions.cpp:58`, `:142`, `:144`).
/// KiCad's conversion is undefined outside the `int` range; Rust's `as`
/// saturates, which is the behaviour to want.
fn truncate_i32(value: f64) -> i32 {
  value as i32
}

/// Clamp an `i64` into an `i32`.
fn saturate_i32(value: i64) -> i32 {
  value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::shape::ShapeKind;

  /// Shorthand for a point.
  fn point(x: i32, y: i32) -> Vec2 {
    Vec2::new(x, y)
  }

  /// An open polyline through the given points.
  fn open_chain(points: &[Vec2]) -> Shape {
    Shape::line_chain(LineChain::from_slice(points, false))
  }

  /// A closed polyline through the given points.
  fn closed_chain(points: &[Vec2]) -> Shape {
    Shape::line_chain(LineChain::from_slice(points, true))
  }

  /// A closed polygon through the given points.
  fn polygon(points: &[Vec2]) -> Shape {
    Shape::simple(LineChain::from_slice(points, true))
  }

  /// A capsule between two points.
  fn capsule(a: Vec2, b: Vec2, width: i32) -> Shape {
    Shape::segment(Seg::new(a, b), width)
  }

  /// The square every polyline and polygon minimum translation vector
  /// test pushes against.
  fn unit_square() -> [Vec2; 4] {
    [point(0, 0), point(100, 0), point(100, 100), point(0, 100)]
  }

  // -----------------------------------------------------------------
  // Mirrors of qa/tests/libs/kimath/geometry/
  // test_shape_line_chain_collision.cpp
  //
  // The three arc cases in that file are not mirrored, because a
  // LineChain here carries no arcs.
  // -----------------------------------------------------------------

  /// `Collide_LineToLine`, test_shape_line_chain_collision.cpp:32.
  #[test]
  fn kicad_collide_line_to_line() {
    let a = open_chain(&[point(0, 0), point(10, 0)]);
    let b = open_chain(&[point(5, 5), point(5, -5)]);

    let hit = collide(&a, &b, 0).expect("expected a collision");
    assert_eq!(hit.actual, 0);
    assert_eq!(hit.location, point(5, 0));
  }

  /// `Collide_WithClearance`, test_shape_line_chain_collision.cpp:86.
  #[test]
  fn kicad_collide_with_clearance() {
    let a = open_chain(&[point(0, 0), point(10, 0)]);
    let b = open_chain(&[point(5, 6), point(-5, 6)]);

    let hit = collide(&a, &b, 7).expect("expected a collision");
    assert_eq!(hit.actual, 6);
    assert_eq!(hit.location, point(0, 0));
  }

  /// `Collide_NoClearance`, test_shape_line_chain_collision.cpp:106.
  #[test]
  fn kicad_collide_no_clearance() {
    let a = open_chain(&[point(0, 0), point(10, 0)]);
    let b = open_chain(&[point(5, 6), point(-5, 6)]);

    assert!(collide(&a, &b, 0).is_none());
    assert!(!collides(&a, &b, 0));
  }

  // -----------------------------------------------------------------
  // Mirror of qa/tests/libs/kimath/geometry/test_shape_compound_collision.cpp
  // -----------------------------------------------------------------

  /// The three compounds of `ShapeCompoundCollisionFixture`,
  /// test_shape_compound_collision.cpp:47.
  fn compound_fixture() -> ([Shape; 2], [Shape; 2], [Shape; 2]) {
    (
      [
        Shape::circle(point(0, 0), 100),
        Shape::circle(point(80, 0), 100),
      ],
      [
        Shape::circle(point(0, 80), 100),
        Shape::circle(point(80, 80), 100),
      ],
      [
        Shape::circle(point(0, 280), 100),
        Shape::circle(point(80, 280), 100),
      ],
    )
  }

  /// `ShapeCompoundCollide`, test_shape_compound_collision.cpp:83.
  ///
  /// The `BOOST_TEST( actual == 0 )` that follows the non colliding call
  /// at `:87` is not mirrored: it reads the out parameter of a call that
  /// returned false, which still holds the value the previous call wrote.
  /// [`ShapeCollision`] makes that unrepresentable.
  #[test]
  fn kicad_shape_compound_collide() {
    let (shapes_a, shapes_b, shapes_c) = compound_fixture();
    let compound_a = Shape::compound(shapes_a.to_vec());
    let compound_b = Shape::compound(shapes_b.to_vec());
    let compound_c = Shape::compound(shapes_c.to_vec());

    let hit = collide(&compound_a, &compound_b, 0).expect("A touches B");
    assert_eq!(hit.actual, 0);

    assert!(collide(&compound_a, &compound_c, 0).is_none());

    let hit = collide(&compound_a, &compound_c, 100).expect("A near C");
    assert_eq!(hit.actual, 80);

    for shape in &shapes_a {
      assert!(collides(shape, &compound_b, 0));
      assert!(collides(&compound_b, shape, 0));
      assert!(!collides(shape, &compound_c, 0));
      assert!(!collides(&compound_c, shape, 0));
    }

    for shape in &shapes_b {
      assert!(collides(shape, &compound_a, 0));
      assert!(collides(&compound_a, shape, 0));
    }

    for shape in &shapes_c {
      assert!(!collides(shape, &compound_a, 0));
      assert!(!collides(&compound_a, shape, 0));

      assert_eq!(
        collide(shape, &compound_a, 100).map(|hit| hit.actual),
        Some(80)
      );
      assert_eq!(
        collide(&compound_a, shape, 100).map(|hit| hit.actual),
        Some(80)
      );
    }
  }

  /// The deliberate extension over `shape_collisions.cpp:1337`: a
  /// compound inside a compound is flattened rather than silently
  /// reported as not colliding.
  #[test]
  fn nested_compounds_still_collide() {
    let inner = Shape::compound(vec![Shape::circle(point(0, 0), 100)]);
    let outer =
      Shape::compound(vec![Shape::circle(point(1_000_000, 0), 10), inner]);
    let probe = Shape::circle(point(0, 80), 100);

    assert!(collides(&outer, &probe, 0));
    assert!(collides(&probe, &outer, 0));
    assert_eq!(collide(&outer, &probe, 0).map(|hit| hit.actual), Some(0));
  }

  // -----------------------------------------------------------------
  // Mirrors of qa/tests/libs/kimath/geometry/test_circle.cpp, the
  // intersection cases that feed SHAPE_CIRCLE::Collide's location.
  //
  // CIRCLE::Contains and CIRCLE::NearestPoint are not mirrored: no
  // collision routine calls them.
  // -----------------------------------------------------------------

  /// `CompareVector2I`, test_circle.cpp:40, which allows each component
  /// to be off by `SHAPE::MIN_PRECISION_IU`.
  fn close_enough(a: Vec2, b: Vec2) -> bool {
    (i64::from(a.x) - i64::from(b.x)).abs()
      <= i64::from(Shape::MIN_PRECISION_IU)
      && (i64::from(a.y) - i64::from(b.y)).abs()
        <= i64::from(Shape::MIN_PRECISION_IU)
  }

  /// `KI_TEST::CheckUnorderedMatches`, as test_circle.cpp uses it.
  fn matches_unordered(expected: &[Vec2], actual: &[Vec2]) -> bool {
    expected.len() == actual.len()
      && expected
        .iter()
        .all(|wanted| actual.iter().any(|found| close_enough(*wanted, *found)))
  }

  /// `intersect_seg_cases`, test_circle.cpp:360.
  #[test]
  fn kicad_circle_intersect_seg() {
    let circle = CircleRef {
      center: point(0, 0),
      radius: 20,
    };

    let cases: [(&str, Seg, Vec<Vec2>); 5] = [
      (
        "two point aligned",
        Seg::from_coords(10, -40, 10, 40),
        vec![point(10, -17), point(10, 17)],
      ),
      (
        "two point angled",
        Seg::from_coords(-20, -40, 20, 40),
        vec![point(8, 17), point(-8, -17)],
      ),
      (
        "tangent",
        Seg::from_coords(20, 0, 20, 40),
        vec![point(20, 0)],
      ),
      (
        "no intersection",
        Seg::from_coords(25, 0, 25, 40),
        Vec::new(),
      ),
      (
        "no intersection: seg end points inside circle",
        Seg::from_coords(0, 10, 0, -10),
        Vec::new(),
      ),
    ];

    for (name, seg, expected) in cases {
      let found = circle_intersect_seg(circle, &seg);
      assert!(
        matches_unordered(&expected, &found),
        "{name}: expected {expected:?}, got {found:?}"
      );
    }
  }

  /// `intersect_line_cases`, test_circle.cpp:419.
  #[test]
  fn kicad_circle_intersect_line() {
    let circle = CircleRef {
      center: point(0, 0),
      radius: 20,
    };

    let cases: [(&str, Seg, Vec<Vec2>); 5] = [
      (
        "two point aligned",
        Seg::from_coords(10, 45, 10, 40),
        vec![point(10, -17), point(10, 17)],
      ),
      (
        "two point angled",
        Seg::from_coords(-20, -40, 20, 40),
        vec![point(8, 17), point(-8, -17)],
      ),
      (
        "tangent",
        Seg::from_coords(20, 0, 20, 40),
        vec![point(20, 0)],
      ),
      (
        "no intersection",
        Seg::from_coords(25, 0, 25, 40),
        Vec::new(),
      ),
      (
        "intersection, seg end points inside circle",
        Seg::from_coords(0, 10, 0, -10),
        vec![point(0, 20), point(0, -20)],
      ),
    ];

    for (name, line, expected) in cases {
      let found = circle_intersect_line(circle, &line);
      assert!(
        matches_unordered(&expected, &found),
        "{name}: expected {expected:?}, got {found:?}"
      );
    }
  }

  /// The location a circle reports when the segment runs through its
  /// centre is an intersection point, not the centre
  /// (`shape_circle.h:82`).
  #[test]
  fn circle_location_on_a_segment_through_the_centre() {
    let circle = Shape::circle(point(0, 0), 20);
    let seg = Seg::from_coords(-40, 0, 40, 0);

    let hit = collide_seg(&circle, &seg, 0).expect("centre is on the seg");
    assert_eq!(hit.actual, 0);
    assert!(hit.location == point(20, 0) || hit.location == point(-20, 0));
  }

  // -----------------------------------------------------------------
  // The coverage table
  // -----------------------------------------------------------------

  /// One shape of every variant, all overlapping the origin.
  fn every_variant() -> [Shape; 6] {
    [
      Shape::circle(point(0, 0), 50),
      Shape::rect(point(-40, -40), point(80, 80)),
      capsule(point(-50, 0), point(50, 0), 10),
      polygon(&[point(-45, -45), point(45, -45), point(0, 45)]),
      open_chain(&[point(-60, 0), point(60, 0)]),
      Shape::compound(vec![
        Shape::circle(point(-30, 0), 10),
        Shape::circle(point(30, 0), 10),
      ]),
    ]
  }

  /// Every ordered pair of variants reaches a dispatch cell.
  ///
  /// The six shapes all overlap the origin, so every one of the thirty
  /// six cells must report a collision. A cell that went missing would
  /// panic on the `unreachable!` in [`collide_single`] or fail here; a
  /// cell wired to the wrong pair would show up as an asymmetry, which
  /// the second half of the test pins down.
  #[test]
  fn every_ordered_pair_collides_when_the_shapes_overlap() {
    let shapes = every_variant();

    for a in &shapes {
      for b in &shapes {
        let label = format!("{:?} against {:?}", a.kind(), b.kind());

        assert!(collides(a, b, 0), "{label}: boolean form");
        assert!(collide(a, b, 0).is_some(), "{label}: actual form");
        assert!(collide_mtv(a, b, 0).is_some(), "{label}: vector form");
        assert!(collides(b, a, 0), "{label}: mirrored");
      }
    }
  }

  /// The same thirty six cells with the operands two millimetres apart.
  #[test]
  fn every_ordered_pair_misses_when_the_shapes_are_apart() {
    let near = every_variant();
    let far = every_variant().map(|shape| {
      let mut moved = shape;
      moved.move_by(point(2_000_000, 0));
      moved
    });

    for a in &near {
      for b in &far {
        let label = format!("{:?} against {:?}", a.kind(), b.kind());

        assert!(!collides(a, b, 1000), "{label}: boolean form");
        assert!(collide(a, b, 1000).is_none(), "{label}: actual form");
        assert!(collide_mtv(a, b, 1000).is_none(), "{label}: vector form");
        assert!(!collides(b, a, 1000), "{label}: mirrored");
      }
    }
  }

  /// Only the circle cells can produce a translation vector. Every other
  /// pair answers zero, which is not the same as answering "no
  /// collision".
  #[test]
  fn cells_without_a_translation_vector_answer_zero() {
    let shapes = every_variant();

    for a in &shapes {
      for b in &shapes {
        let vector = collide_mtv(a, b, 0).expect("the shapes overlap");
        let circles_involved = matches!(a.kind(), ShapeKind::Circle)
          || matches!(b.kind(), ShapeKind::Circle)
          || matches!(a.kind(), ShapeKind::Compound)
          || matches!(b.kind(), ShapeKind::Compound);

        if !circles_involved {
          assert_eq!(
            vector,
            point(0, 0),
            "{:?} against {:?} should have no vector",
            a.kind(),
            b.kind()
          );
        }
      }
    }
  }

  // -----------------------------------------------------------------
  // Minimum translation vector sign and effect, one test per capable
  // cell
  // -----------------------------------------------------------------

  /// Assert the convention on one pair: the vector displaces `b`, moving
  /// `b` by it ends the collision, and the mirrored call negates it.
  fn check_mtv(a: &Shape, b: &Shape, clearance: i32) -> Vec2 {
    let vector = collide_mtv(a, b, clearance).expect("expected a collision");
    assert_ne!(vector, point(0, 0), "expected a usable vector");

    let mut moved = b.clone();
    moved.move_by(vector);
    assert!(
      !collides(a, &moved, clearance),
      "moving b by {vector:?} did not separate the shapes"
    );

    assert_eq!(
      collide_mtv(b, a, clearance),
      Some(-vector),
      "the mirrored call should negate the vector"
    );

    vector
  }

  /// `shape_collisions.cpp:58`, the analytic radial vector with its `+ 3`
  /// bias: `200 - 150 + 3 = 53`.
  #[test]
  fn mtv_circle_against_circle() {
    let a = Shape::circle(point(0, 0), 100);
    let b = Shape::circle(point(0, 150), 100);

    assert_eq!(check_mtv(&a, &b, 0), point(0, 53));
  }

  /// Concentric circles collide but cannot be separated, which is the
  /// case `PNS::VIA::PushoutForce` gives up on
  /// (`pcbnew/router/pns_via.cpp:172`).
  #[test]
  fn mtv_concentric_circles_is_zero() {
    let a = Shape::circle(point(0, 0), 100);
    let b = Shape::circle(point(0, 0), 50);

    assert_eq!(collide_mtv(&a, &b, 0), Some(point(0, 0)));
  }

  /// `shape_collisions.cpp:144`, the analytic nearest side vector with
  /// its two `+ 1` biases: `trunc( |30 + 1 - 20| + 1 ) = 12`.
  #[test]
  fn mtv_rect_against_circle() {
    let a = Shape::rect(point(0, 0), point(100, 100));
    let b = Shape::circle(point(50, 120), 30);

    assert_eq!(check_mtv(&a, &b, 0), point(0, 12));
  }

  /// The mirrored cell, `shape_collisions.cpp:1107`, which reverses the
  /// operands and negates.
  #[test]
  fn mtv_circle_against_rect() {
    let a = Shape::circle(point(50, 120), 30);
    let b = Shape::rect(point(0, 0), point(100, 100));

    assert_eq!(check_mtv(&a, &b, 0), point(0, -12));
  }

  /// A circle whose centre is inside the rectangle pushes out through the
  /// nearest side, `shape_collisions.cpp:142`.
  #[test]
  fn mtv_rect_against_a_circle_inside_it() {
    let a = Shape::rect(point(0, 0), point(100, 100));
    let b = Shape::circle(point(50, 90), 10);

    let vector = collide_mtv(&a, &b, 0).expect("the centre is inside");
    assert!(vector.y > 0, "expected a push through the nearest side");

    let mut moved = b.clone();
    moved.move_by(vector);
    assert!(!collides(&a, &moved, 0));
  }

  /// `shape_collisions.cpp:339`, the negated pushout: one nanometre of
  /// penetration gives one nanometre of vector.
  #[test]
  fn mtv_circle_against_capsule() {
    let a = Shape::circle(point(0, 0), 50);
    let b = capsule(point(0, 59), point(100, 59), 20);

    assert_eq!(check_mtv(&a, &b, 0), point(0, 1));
  }

  /// The mirrored cell, `shape_collisions.cpp:1176`.
  #[test]
  fn mtv_capsule_against_circle() {
    let a = capsule(point(0, 59), point(100, 59), 20);
    let b = Shape::circle(point(0, 0), 50);

    assert_eq!(check_mtv(&a, &b, 0), point(0, -1));
  }

  /// `shape_collisions.cpp:1113`, the cell whose vector KiCad writes for
  /// the wrong operand. The negation happens in [`collide_single`], so
  /// this vector displaces the chain.
  #[test]
  fn mtv_circle_against_chain() {
    let a = Shape::circle(point(50, 110), 20);
    let b = closed_chain(&unit_square());

    assert_eq!(check_mtv(&a, &b, 0), point(0, -10));
  }

  /// `shape_collisions.cpp:1143`, the cell KiCad reaches by swapping the
  /// operands without negating, which lands on the right sign anyway.
  #[test]
  fn mtv_chain_against_circle() {
    let a = closed_chain(&unit_square());
    let b = Shape::circle(point(50, 110), 20);

    assert_eq!(check_mtv(&a, &b, 0), point(0, 10));
  }

  /// `shape_collisions.cpp:1120`, the polygon twin of
  /// [`mtv_circle_against_chain`].
  #[test]
  fn mtv_circle_against_polygon() {
    let a = Shape::circle(point(50, 110), 20);
    let b = polygon(&unit_square());

    assert_eq!(check_mtv(&a, &b, 0), point(0, -10));
  }

  /// `shape_collisions.cpp:1210`, the polygon twin of
  /// [`mtv_chain_against_circle`].
  #[test]
  fn mtv_polygon_against_circle() {
    let a = polygon(&unit_square());
    let b = Shape::circle(point(50, 110), 20);

    assert_eq!(check_mtv(&a, &b, 0), point(0, 10));
  }

  /// A centre inside a closed chain is first thrown out through the
  /// nearest segment and one radius past it,
  /// `shape_collisions.cpp:305`.
  #[test]
  fn mtv_circle_inside_a_closed_chain() {
    let a = Shape::circle(point(50, 90), 10);
    let b = closed_chain(&unit_square());

    let vector = collide_mtv(&a, &b, 0).expect("the centre is inside");
    assert_ne!(vector, point(0, 0));

    let mut moved = b.clone();
    moved.move_by(vector);
    assert!(!collides(&a, &moved, 0));
  }

  /// The compound aggregation keeps the largest vector by magnitude,
  /// `shape_collisions.cpp:1348`.
  #[test]
  fn mtv_compound_keeps_the_largest_vector() {
    let a = Shape::compound(vec![
      Shape::circle(point(0, 0), 100),
      Shape::circle(point(0, 0), 40),
    ]);
    let b = Shape::circle(point(0, 150), 100);

    let deep = collide_mtv(&Shape::circle(point(0, 0), 100), &b, 0);
    assert_eq!(collide_mtv(&a, &b, 0), deep);
  }

  // -----------------------------------------------------------------
  // Boundary cases, one per cell: exactly the clearance apart is not a
  // collision, one nanometre closer is, and overlapping reports a gap of
  // zero.
  // -----------------------------------------------------------------

  /// `shape_collisions.cpp:49`, the strict comparison on two circles.
  #[test]
  fn boundary_circle_against_circle() {
    let a = Shape::circle(point(0, 0), 100);

    let touching = Shape::circle(point(0, 200), 100);
    assert!(
      collide(&a, &touching, 0).is_none(),
      "tangent at zero clearance"
    );

    let overlapping = Shape::circle(point(0, 199), 100);
    assert_eq!(
      collide(&a, &overlapping, 0).map(|hit| hit.actual),
      Some(0),
      "one nanometre of overlap is a gap of zero"
    );

    let exactly = Shape::circle(point(0, 300), 100);
    assert!(
      collide(&a, &exactly, 100).is_none(),
      "exactly the clearance"
    );

    let closer = Shape::circle(point(0, 299), 100);
    assert_eq!(
      collide(&a, &closer, 100).map(|hit| hit.actual),
      Some(99),
      "one nanometre closer"
    );
  }

  /// `shape_collisions.cpp:536`, where the half widths round differently:
  /// `(10 + 1) / 2 + 50 + 20 / 2 = 65`.
  #[test]
  fn boundary_capsule_against_capsule() {
    let a = capsule(point(0, 0), point(100, 0), 10);

    let exactly = capsule(point(0, 65), point(100, 65), 20);
    assert!(collide(&a, &exactly, 50).is_none());

    let closer = capsule(point(0, 64), point(100, 64), 20);
    assert_eq!(collide(&a, &closer, 50).map(|hit| hit.actual), Some(49));

    let crossing = capsule(point(50, -50), point(50, 50), 20);
    assert_eq!(collide(&a, &crossing, 0).map(|hit| hit.actual), Some(0));
  }

  /// `shape_collisions.cpp:412` through `SEG::Collide`.
  #[test]
  fn boundary_chain_against_chain() {
    let a = open_chain(&[point(0, 0), point(100, 0)]);

    let exactly = open_chain(&[point(0, 50), point(100, 50)]);
    assert!(collide(&a, &exactly, 50).is_none());

    let closer = open_chain(&[point(0, 49), point(100, 49)]);
    let hit = collide(&a, &closer, 50).expect("one nanometre closer");
    assert_eq!(hit.actual, 49);
    assert_eq!(hit.location, point(0, 0));
  }

  /// `shape_collisions.cpp:129`, the strict comparison on a rectangle and
  /// a circle.
  #[test]
  fn boundary_rect_against_circle() {
    let a = Shape::rect(point(0, 0), point(100, 100));

    let exactly = Shape::circle(point(50, 150), 20);
    assert!(collide(&a, &exactly, 30).is_none());

    let closer = Shape::circle(point(50, 149), 20);
    let hit = collide(&a, &closer, 30).expect("one nanometre closer");
    assert_eq!(hit.actual, 29);
    assert_eq!(hit.location, point(50, 100));

    let tangent = Shape::circle(point(50, 120), 20);
    assert!(
      collide(&a, &tangent, 0).is_none(),
      "a circle exactly touching a side is not a collision"
    );

    let overlapping = Shape::circle(point(50, 119), 20);
    assert_eq!(collide(&a, &overlapping, 0).map(|hit| hit.actual), Some(0));
  }

  /// The defect at `shape_collisions.cpp:135`: a circle whose centre is
  /// inside the rectangle reports the distance to the nearest side rather
  /// than zero. The collision itself is still found.
  #[test]
  fn rect_against_a_contained_circle_reports_the_side_distance() {
    let a = Shape::rect(point(0, 0), point(100, 100));
    let b = Shape::circle(point(50, 50), 5);

    let hit = collide(&a, &b, 0).expect("the centre is inside");
    assert_eq!(hit.actual, 45, "50 to the nearest side, less the radius");
  }

  /// `shape_collisions.cpp:571`, the truncating half width fold.
  #[test]
  fn boundary_rect_against_capsule() {
    let a = Shape::rect(point(0, 0), point(100, 100));

    let exactly = capsule(point(0, 150), point(100, 150), 20);
    assert!(collide(&a, &exactly, 40).is_none());

    let closer = capsule(point(0, 149), point(100, 149), 20);
    assert_eq!(collide(&a, &closer, 40).map(|hit| hit.actual), Some(39));
  }

  /// `shape_collisions.cpp:483`.
  #[test]
  fn boundary_rect_against_chain() {
    let a = Shape::rect(point(0, 0), point(100, 100));

    let exactly = open_chain(&[point(0, 150), point(100, 150)]);
    assert!(collide(&a, &exactly, 50).is_none());

    let closer = open_chain(&[point(0, 149), point(100, 149)]);
    assert_eq!(collide(&a, &closer, 50).map(|hit| hit.actual), Some(49));
  }

  /// `shape_collisions.cpp:552`.
  #[test]
  fn boundary_chain_against_capsule() {
    let a = open_chain(&[point(0, 0), point(100, 0)]);

    let exactly = capsule(point(0, 50), point(100, 50), 20);
    assert!(collide(&a, &exactly, 40).is_none());

    let closer = capsule(point(0, 49), point(100, 49), 20);
    assert_eq!(collide(&a, &closer, 40).map(|hit| hit.actual), Some(39));
  }

  /// `shape_collisions.cpp:336` and `:342`.
  #[test]
  fn boundary_circle_against_capsule() {
    let a = Shape::circle(point(0, 0), 50);

    let exactly = capsule(point(0, 110), point(100, 110), 20);
    assert!(collide(&a, &exactly, 50).is_none());

    let closer = capsule(point(0, 110), point(100, 110), 20);
    assert_eq!(collide(&a, &closer, 51).map(|hit| hit.actual), Some(50));
  }

  /// `shape_collisions.cpp:269` and the final re test at `:290`.
  #[test]
  fn boundary_circle_against_chain() {
    let a = open_chain(&[point(0, 0), point(100, 0)]);

    let exactly = Shape::circle(point(50, 50), 20);
    assert!(collide(&a, &exactly, 30).is_none());

    let closer = Shape::circle(point(50, 49), 20);
    let hit = collide(&a, &closer, 30).expect("one nanometre closer");
    assert_eq!(hit.actual, 29);
    assert_eq!(hit.location, point(50, 0));
  }

  /// `shape_collisions.cpp:580`. Both outlines go through the polyline
  /// cell whenever anything but a boolean is asked for.
  #[test]
  fn boundary_rect_against_rect() {
    let a = Shape::rect(point(0, 0), point(100, 100));

    let exactly = Shape::rect(point(0, 150), point(100, 100));
    assert!(collide(&a, &exactly, 50).is_none());

    let closer = Shape::rect(point(0, 149), point(100, 100));
    assert_eq!(collide(&a, &closer, 50).map(|hit| hit.actual), Some(49));

    let touching = Shape::rect(point(0, 100), point(100, 100));
    assert_eq!(collide(&a, &touching, 0).map(|hit| hit.actual), Some(0));
  }

  /// The bounding box shortcut at `shape_collisions.cpp:589` is only
  /// reachable through [`collides`], at zero clearance, with both corner
  /// radii zero. It agrees with the outline path on both sides of the
  /// boundary.
  #[test]
  fn rect_against_rect_bounding_box_shortcut() {
    let a = Shape::rect(point(0, 0), point(100, 100));
    let touching = Shape::rect(point(100, 0), point(100, 100));
    let apart = Shape::rect(point(101, 0), point(100, 100));

    assert!(collides(&a, &touching, 0));
    assert!(collide(&a, &touching, 0).is_some());

    assert!(!collides(&a, &apart, 0));
    assert!(collide(&a, &apart, 0).is_none());
  }

  // -----------------------------------------------------------------
  // Containment shortcuts and the rounded rectangle branch
  // -----------------------------------------------------------------

  /// `shape_collisions.cpp:363`: vertex zero of the first chain inside
  /// the closed second one collides at zero and reports itself.
  #[test]
  fn chain_inside_a_closed_chain_collides_at_zero() {
    let inside = open_chain(&[point(40, 50), point(60, 50)]);
    let outline = closed_chain(&unit_square());

    let hit = collide(&inside, &outline, 0).expect("wholly inside");
    assert_eq!(hit.actual, 0);
    assert_eq!(hit.location, point(40, 50));
  }

  /// `shape_collisions.cpp:368`, the other order, which reports the other
  /// chain's vertex zero.
  #[test]
  fn closed_chain_around_a_chain_reports_the_inner_vertex() {
    let inside = open_chain(&[point(40, 50), point(60, 50)]);
    let outline = closed_chain(&unit_square());

    let hit = collide(&outline, &inside, 0).expect("wholly inside");
    assert_eq!(hit.actual, 0);
    assert_eq!(hit.location, point(40, 50));
  }

  /// `shape_collisions.cpp:483`, which tests the rectangle's centre and
  /// not its corners.
  #[test]
  fn rect_inside_a_closed_chain_collides_at_its_centre() {
    let rect = Shape::rect(point(40, 40), point(20, 20));
    let outline = closed_chain(&unit_square());

    let hit = collide(&rect, &outline, 0).expect("wholly inside");
    assert_eq!(hit.actual, 0);
    assert_eq!(hit.location, point(50, 50));
  }

  /// A rectangle with a corner radius hands the whole collision to its
  /// outline (`shape_collisions.cpp:70`, `:472`, `:564`,
  /// `shape_rect.cpp:28`), and the vector request is dropped on the way
  /// (`shape_collisions.cpp:72`).
  ///
  /// The rounded corners themselves are not ported; see the deviation on
  /// `rect_outline`. The outline is the sharp one, so a rounded rectangle
  /// reports the same gap as a sharp one. The reported **location** can
  /// still differ, because `SHAPE_RECT::Collide( const SEG& )` breaks ties
  /// between equidistant sides towards the start of the argument
  /// (`shape_rect.cpp:78`) while the polyline scan it delegates to keeps
  /// the first strict minimum (`shape_line_chain.cpp:838`).
  #[test]
  fn rounded_rect_delegates_to_its_outline() {
    let sharp = Shape::rect(point(0, 0), point(100, 100));
    let rounded = Shape::rounded_rect(point(0, 0), point(100, 100), 20);
    let circle = Shape::circle(point(50, 149), 20);
    let strip = capsule(point(0, 149), point(100, 149), 20);
    let chain = open_chain(&[point(0, 149), point(100, 149)]);

    for (probe, clearance) in [(&circle, 30), (&strip, 40), (&chain, 50)] {
      assert_eq!(
        collide(&rounded, probe, clearance).map(|hit| hit.actual),
        collide(&sharp, probe, clearance).map(|hit| hit.actual),
        "{:?} should report the same gap either way",
        probe.kind()
      );
    }

    assert_eq!(
      collide_mtv(&rounded, &circle, 30),
      Some(point(0, 0)),
      "a rounded rectangle drops the vector request"
    );
    assert!(collide_mtv(&sharp, &circle, 30).is_some());
    assert_ne!(collide_mtv(&sharp, &circle, 30), Some(point(0, 0)));
  }

  /// The polygon variant reaches the same cells as a closed polyline, but
  /// through the `SH_SIMPLE` arms of the switch, so the predicate and the
  /// gap agree.
  ///
  /// The reported location does **not** always agree, because the
  /// polygon against polyline arm swaps its operands
  /// (`shape_collisions.cpp:1213`) while the polyline against polyline
  /// arm does not (`:1146`), and the location lands on whichever chain
  /// took the first role. See
  /// [`polygon_against_polyline_swaps_the_operands`].
  #[test]
  fn polygon_and_closed_chain_agree_on_the_gap() {
    let square_polygon = polygon(&unit_square());
    let square_chain = closed_chain(&unit_square());
    let probes = [
      Shape::circle(point(50, 110), 20),
      Shape::rect(point(40, 105), point(20, 20)),
      capsule(point(0, 110), point(100, 110), 30),
      open_chain(&[point(0, 105), point(100, 105)]),
    ];

    for probe in &probes {
      assert_eq!(
        collide(&square_polygon, probe, 10).map(|hit| hit.actual),
        collide(&square_chain, probe, 10).map(|hit| hit.actual),
        "{:?} in the second role",
        probe.kind()
      );
      assert_eq!(
        collide(probe, &square_polygon, 10).map(|hit| hit.actual),
        collide(probe, &square_chain, 10).map(|hit| hit.actual),
        "{:?} in the first role",
        probe.kind()
      );
    }
  }

  /// `shape_collisions.cpp:1213` hands the polyline the first role and
  /// the polygon the second, whichever way round the caller wrote them,
  /// so the location lands on the polyline both times.
  #[test]
  fn polygon_against_polyline_swaps_the_operands() {
    let square_polygon = polygon(&unit_square());
    let probe = open_chain(&[point(0, 105), point(100, 105)]);

    let forwards = collide(&square_polygon, &probe, 10).expect("in range");
    let backwards = collide(&probe, &square_polygon, 10).expect("in range");

    assert_eq!(forwards, backwards);
    assert_eq!(forwards.actual, 5);
    assert_eq!(forwards.location, point(0, 105));

    let square_chain = closed_chain(&unit_square());
    let unswapped = collide(&square_chain, &probe, 10).expect("in range");
    assert_eq!(unswapped.actual, 5);
    assert_eq!(unswapped.location, point(0, 100));
  }

  /// An open chain is never closed, so a polygon built from the same
  /// points behaves differently: the containment shortcut only exists for
  /// the closed one.
  #[test]
  fn only_a_closed_chain_swallows_a_point() {
    let inner = Shape::circle(point(50, 50), 5);
    let open = open_chain(&unit_square());
    let closed = closed_chain(&unit_square());

    assert!(!collides(&open, &inner, 0));
    assert!(collides(&closed, &inner, 0));
  }

  // -----------------------------------------------------------------
  // The point and segment forms
  // -----------------------------------------------------------------

  /// The default point form wraps the point in a degenerate segment,
  /// `shape.h:179`.
  #[test]
  fn collide_point_against_a_circle() {
    let circle = Shape::circle(point(0, 0), 50);

    let hit = collide_point(&circle, point(30, 0), 0).expect("inside");
    assert_eq!(hit.actual, 0);
    assert_eq!(hit.location, point(30, 0));

    assert!(collide_point(&circle, point(49, 0), 0).is_some());
    assert!(
      collide_point(&circle, point(50, 0), 0).is_none(),
      "a point exactly on the circumference is not a collision"
    );
    assert!(collide_point(&circle, point(60, 0), 11).is_some());
    assert!(collide_point(&circle, point(60, 0), 10).is_none());
  }

  /// A rectangle answers through `shape_rect.cpp:35`, which returns the
  /// point itself as the location.
  #[test]
  fn collide_point_against_a_rect() {
    let rect = Shape::rect(point(0, 0), point(100, 100));

    let hit = collide_point(&rect, point(50, 50), 0).expect("inside");
    assert_eq!(hit.actual, 0);
    assert_eq!(hit.location, point(50, 50));

    assert!(collide_point(&rect, point(100, 50), 0).is_some());
    assert!(collide_point(&rect, point(101, 50), 0).is_none());
    assert!(collide_point(&rect, point(110, 50), 11).is_some());
  }

  /// A capsule overrides the point form with its own half width rounded
  /// up, `shape_segment.h:100`.
  #[test]
  fn collide_point_against_a_capsule() {
    let strip = capsule(point(0, 0), point(100, 0), 10);

    assert!(collide_point(&strip, point(50, 5), 0).is_none());
    let hit = collide_point(&strip, point(50, 4), 0).expect("inside");
    assert_eq!(hit.actual, 0);
    assert_eq!(hit.location, point(50, 0));
  }

  /// A closed chain overrides the point form and runs its containment
  /// test with the clearance as the accuracy,
  /// `shape_line_chain.cpp:429`.
  #[test]
  fn collide_point_against_a_closed_chain() {
    let outline = closed_chain(&unit_square());

    let hit = collide_point(&outline, point(50, 50), 0).expect("inside");
    assert_eq!(hit.actual, 0);
    assert_eq!(hit.location, point(50, 50));

    assert!(collide_point(&outline, point(150, 50), 0).is_none());

    // `PointInside` is called with the clearance as its accuracy
    // (shape_line_chain.cpp:429) and falls through to `PointOnEdge`,
    // whose tolerance is `( accuracy + 1 )` squared
    // (shape_line_chain.cpp:2082). A point within fifty one nanometres of
    // the outline therefore takes the containment shortcut and reports a
    // gap of zero rather than its true distance.
    assert_eq!(
      collide_point(&outline, point(151, 50), 50).map(|hit| hit.actual),
      Some(0)
    );
    assert!(collide_point(&outline, point(152, 50), 50).is_none());
  }

  /// A compound answers through its own aggregation,
  /// `shape_compound.cpp:109`.
  #[test]
  fn collide_point_against_a_compound() {
    let compound = Shape::compound(vec![
      Shape::circle(point(0, 0), 10),
      Shape::circle(point(100, 0), 10),
    ]);

    assert!(collide_point(&compound, point(100, 5), 0).is_some());
    assert!(collide_point(&compound, point(50, 0), 0).is_none());
    assert!(collide_point(&compound, point(50, 0), 41).is_some());
  }

  /// The segment form is what the optimizer calls when it asks whether a
  /// breakout is swallowed by its pad
  /// (`pcbnew/router/pns_optimizer.cpp:1139`).
  #[test]
  fn collide_seg_against_every_variant() {
    let crossing = Seg::from_coords(-200, 0, 200, 0);
    let far = Seg::from_coords(-200, 5000, 200, 5000);

    for shape in &every_variant() {
      assert!(
        collide_seg(shape, &crossing, 0).is_some(),
        "{:?} should meet a segment through the origin",
        shape.kind()
      );
      assert!(
        collide_seg(shape, &far, 0).is_none(),
        "{:?} should miss a distant segment",
        shape.kind()
      );
    }
  }

  /// `shape_rect.cpp:94` writes the gap without clamping it, unlike every
  /// other cell. A squared distance cannot be negative, so it never goes
  /// below zero anyway.
  #[test]
  fn collide_seg_against_a_rect_reports_the_side_distance() {
    let rect = Shape::rect(point(0, 0), point(100, 100));
    let seg = Seg::from_coords(0, 149, 100, 149);

    let hit = collide_seg(&rect, &seg, 50).expect("within the clearance");
    assert_eq!(hit.actual, 49);
  }

  /// The compound segment form breaks ties on the distance to the start
  /// of the argument segment, `shape_compound.cpp:130`.
  #[test]
  fn collide_seg_against_a_compound_breaks_ties_towards_the_start() {
    let compound = Shape::compound(vec![
      Shape::circle(point(200, 0), 10),
      Shape::circle(point(0, 0), 10),
    ]);
    let seg = Seg::from_coords(0, 100, 200, 100);

    let hit = collide_seg(&compound, &seg, 91).expect("both are in range");
    assert_eq!(hit.actual, 90);
    assert_eq!(
      hit.location,
      point(0, 100),
      "the second subshape wins the tie because it is nearer the start"
    );
  }

  // -----------------------------------------------------------------
  // Small helpers
  // -----------------------------------------------------------------

  /// `KiROUND( x / 2.0 )` rounds half away from zero, unlike an integer
  /// division.
  #[test]
  fn midpoint_rounds_half_away_from_zero() {
    assert_eq!(midpoint(point(0, 0), point(3, 3)), point(2, 2));
    assert_eq!(midpoint(point(0, 0), point(-3, -3)), point(-2, -2));
    assert_eq!(midpoint(point(10, 20), point(20, 40)), point(15, 30));
  }

  /// The midpoint of two centres, `shape_collisions.cpp:55`, is on
  /// neither circumference.
  #[test]
  fn circle_against_circle_reports_the_midpoint_of_the_centres() {
    let a = Shape::circle(point(0, 0), 100);
    let b = Shape::circle(point(0, 150), 100);

    let hit = collide(&a, &b, 0).expect("overlapping");
    assert_eq!(hit.location, point(0, 75));
  }

  /// The squared clearance saturates instead of wrapping, so an enormous
  /// clearance still compares as "further than any distance". This is the
  /// path `SHAPE::GetClearance` takes with `INT_MAX / 2`
  /// (`libs/kimath/src/geometry/shape.cpp:94`).
  #[test]
  fn an_enormous_clearance_does_not_overflow() {
    let a = Shape::circle(point(0, 0), i32::MAX);
    let b = Shape::circle(point(1000, 0), i32::MAX);

    assert_eq!(square(3 * i64::from(i32::MAX)), i64::MAX);
    assert!(collides(&a, &b, i32::MAX));
  }

  /// A negative clearance is not guarded at the shape level, note 01
  /// section 9.3, and the router does pass minus one
  /// (`pcbnew/router/pns_item.cpp:249`). It simply shrinks the inflation
  /// sum, so two circles have to overlap by two nanometres rather than
  /// one before they count as colliding.
  #[test]
  fn a_negative_clearance_shrinks_the_inflation() {
    let a = Shape::circle(point(0, 0), 100);

    assert!(collides(&a, &Shape::circle(point(0, 198), 100), -1));
    assert!(!collides(&a, &Shape::circle(point(0, 199), 100), -1));
    assert!(collides(&a, &Shape::circle(point(0, 199), 100), 0));
  }
}
