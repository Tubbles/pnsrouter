// SPDX-License-Identifier: GPL-3.0-or-later

//! Deciding which of the two 45 degree postures the head leaves in.
//!
//! Port of `PNS::MOUSE_TRAIL_TRACER`
//! (`pcbnew/router/pns_mouse_trail_tracer.h:32`,
//! `pcbnew/router/pns_mouse_trail_tracer.cpp`), the posture solver the
//! line placer consults on every mouse move
//! (`pcbnew/router/pns_line_placer.cpp:2041`).
//!
//! # The question it answers
//!
//! Two points that are neither axis aligned nor an exact diagonal are
//! joined by a two segment 45 degree trace in exactly two ways: straight
//! leg first, or diagonal leg first. [`Direction45::build_initial_trace`]
//! builds either on demand, so the whole job of this module is to pick
//! one, and to keep picking the same one while the user is not asking for
//! the other.
//!
//! # How it decides
//!
//! It remembers where the cursor has been since the route started, as a
//! polyline it prunes and simplifies as it grows
//! ([`MouseTrailTracer::add_trail_point`]). Given a cursor position it
//! builds both candidate traces from the trail's first point, closes each
//! one against the reversed trail, and compares the two enclosed areas
//! ([`LineChain::area`]). The candidate that encloses **less** area is
//! the one that follows what the user actually drew.
//!
//! The raw comparison flutters, so three guards sit on top of it: a
//! ratio threshold with a hysteresis band
//! ([`AREA_RATIO_THRESHOLD`], [`AREA_RATIO_EPSILON`]), a minimum trail
//! size below which the area test is not trusted at all
//! ([`MIN_AREA_CUTOFF_DISTANCE_FACTOR`]), and a lock that takes the area
//! comparison's vote away once the cursor is far enough from the start
//! ([`LOCK_DISTANCE_FACTOR`], released again by
//! [`UNLOCK_DISTANCE_FACTOR`]).
//!
//! The lock is worth reading carefully, because its name oversells it.
//! `m_forced` only disables the two ratio branches
//! (`pcbnew/router/pns_mouse_trail_tracer.cpp:169`, `:171`); the
//! assignment below them is not guarded, so a locked tracer still
//! re-answers from the current candidates every move. What is frozen is
//! the **posture family**, straight first or diagonal first, and not the
//! octant: a locked answer of east becomes north as soon as the cursor
//! moves somewhere whose straight first candidate leaves northwards.
//!
//! All four thresholds are relative to
//! [`MouseTrailTracer::set_tolerance`], which the placer sets to the head
//! width (`pcbnew/router/pns_line_placer.cpp:1425`).
//!
//! On top of the area test come two overrides: the direction of the
//! previously fixed segment, which pulls the answer towards the least
//! obtuse continuation, and the user's own posture key
//! ([`MouseTrailTracer::flip_posture`]), which wins over everything for
//! the life of the trail.
//!
//! # What is left out
//!
//! `GetTrailLeadVector` (`pcbnew/router/pns_mouse_trail_tracer.cpp:279`)
//! has no callers in KiCad's tree and is not ported; note 03 section 9.5
//! lists it. There is therefore no lead vector parameter anywhere in this
//! module: `GetPosture` takes the cursor position and nothing else
//! (`pcbnew/router/pns_mouse_trail_tracer.h:50`).
//!
//! # The one place the singleton was reached
//!
//! Both `AddTrailPoint` and `GetPosture` fetch the debug decorator
//! through `ROUTER::GetInstance()->GetInterface()->GetDebugDecorator()`
//! (`:78`, `:112`). That is one of the four uses of the global router
//! note 03 section 9.4 lists, and it is the only ambient state this class
//! touches. Here it arrives as an [`AlgoContext`] parameter, so the
//! tracer is a plain value with no hidden dependency.

use crate::algo_base::AlgoContext;
use crate::geometry::direction45::{AngleType, CornerMode, Direction45};
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::Vec2;

// ---------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------

/// How much better the fit has to be before the posture switches.
///
/// Port of `areaRatioThreshold`,
/// `pcbnew/router/pns_mouse_trail_tracer.cpp:87`. The comparison is on
/// the ratio of the straight first candidate's area to the diagonal
/// first candidate's, so a ratio above the threshold means the straight
/// first trace strays far from the trail and the diagonal one wins.
pub const AREA_RATIO_THRESHOLD: f64 = 1.3;

/// The dead band around [`AREA_RATIO_THRESHOLD`].
///
/// Port of `areaRatioEpsilon`,
/// `pcbnew/router/pns_mouse_trail_tracer.cpp:90`. It widens the switch
/// threshold in both directions, so a ratio between
/// `1 / 1.3 - 0.25` and `1.3 + 0.25` leaves the posture alone.
pub const AREA_RATIO_EPSILON: f64 = 0.25;

/// How long the trail has to be, in tolerances, before the area test is
/// believed.
///
/// Port of `minAreaCutoffDistanceFactor`,
/// `pcbnew/router/pns_mouse_trail_tracer.cpp:93`. Note that this gates
/// the **distance** from the trail's first point to the cursor. The area
/// the trail then has to enclose is not a multiple of the tolerance
/// squared but `tolerance * that distance`
/// (`:152`), which is the area of a corridor one tolerance wide along
/// the straight run: a trail that never leaves such a corridor is too
/// close to a straight line for its shape to mean anything.
pub const MIN_AREA_CUTOFF_DISTANCE_FACTOR: f64 = 6.0;

/// How far from the trail's first point the posture family freezes, in
/// tolerances.
///
/// Port of `lockDistanceFactor`,
/// `pcbnew/router/pns_mouse_trail_tracer.cpp:96`. What the lock stops is
/// the area comparison, not the answer; see the module documentation.
pub const LOCK_DISTANCE_FACTOR: i32 = 30;

/// How close to the trail's first point the freeze is released, in
/// tolerances.
///
/// Port of `unlockDistanceFactor`,
/// `pcbnew/router/pns_mouse_trail_tracer.cpp:99`. Coming back inside
/// this radius also restarts the trail from that first point (`:141` to
/// `:143`), so the user who drags back to where they started gets a
/// clean slate.
pub const UNLOCK_DISTANCE_FACTOR: i32 = 10;

/// How many trailing segments the self pruning scan skips.
///
/// Port of the `SegmentCount() - 2` bound,
/// `pcbnew/router/pns_mouse_trail_tracer.cpp:61`. The new segment always
/// touches the last one and usually nearly touches the one before, so
/// neither may be allowed to prune the trail.
pub const TRAIL_PRUNE_SKIPPED_SEGMENTS: usize = 2;

// ---------------------------------------------------------------------
// MouseTrailTracer
// ---------------------------------------------------------------------

/// The posture solver.
///
/// Port of `PNS::MOUSE_TRAIL_TRACER`,
/// `pcbnew/router/pns_mouse_trail_tracer.h:32`. Feed it cursor positions
/// with [`MouseTrailTracer::add_trail_point`] and ask it for a posture
/// with [`MouseTrailTracer::get_posture`].
///
/// # Clearing does not reset everything
///
/// [`MouseTrailTracer::clear`] empties the trail and releases both locks
/// but deliberately leaves the current direction, the previous segment's
/// direction and the tolerance alone (`:39` to `:44`). That is why every
/// caller in KiCad follows a `Clear()` with
/// [`MouseTrailTracer::set_tolerance`] and
/// [`MouseTrailTracer::set_default_directions`]
/// (`pcbnew/router/pns_line_placer.cpp:1423` to `:1426`, `:1741` to
/// `:1744`, `:1785` to `:1787`).
#[derive(Clone, Debug, Default)]
pub struct MouseTrailTracer {
  /// Where the cursor has been. Port of `m_trail` (`:64`).
  trail: LineChain,
  /// The distance scale every threshold is measured in, the head width.
  /// Port of `m_tolerance` (`:65`).
  tolerance: i32,
  /// The answer. Port of `m_direction` (`:66`).
  direction: Direction45,
  /// The direction of the segment that was fixed last, or undefined.
  /// Port of `m_lastSegDirection` (`:67`).
  last_seg_direction: Direction45,
  /// Whether the answer is frozen. Port of `m_forced` (`:68`).
  forced: bool,
  /// Whether the mouse trail heuristic is switched off. Port of
  /// `m_disableMouse` (`:69`).
  disable_mouse: bool,
  /// Whether the user pressed the posture key. Port of
  /// `m_manuallyForced` (`:70`).
  manually_forced: bool,
}

impl MouseTrailTracer {
  /// A tracer with no trail, no tolerance and an undefined direction.
  ///
  /// Port of the constructor,
  /// `pcbnew/router/pns_mouse_trail_tracer.cpp:28`, which sets the
  /// tolerance to zero, leaves the mouse heuristic enabled and then calls
  /// `Clear()`. Both directions start out default constructed, which is
  /// `DIRECTION_45::UNDEFINED`.
  ///
  /// A tolerance of zero makes every threshold zero, so a fresh tracer
  /// trusts the area test from the first move and locks immediately. The
  /// placer never routes in that state: it sets the tolerance to the head
  /// width right after clearing
  /// (`pcbnew/router/pns_line_placer.cpp:1425`).
  pub fn new() -> Self {
    Self::default()
  }

  /// Forget the trail and release both locks.
  ///
  /// Port of `Clear`,
  /// `pcbnew/router/pns_mouse_trail_tracer.cpp:39`. See the type
  /// documentation for what it deliberately does **not** reset.
  pub fn clear(&mut self) {
    self.forced = false;
    self.manually_forced = false;
    self.trail.clear();
  }

  /// The distance scale the thresholds are measured in.
  ///
  /// Port of `SetTolerance`,
  /// `pcbnew/router/pns_mouse_trail_tracer.h:42`. The placer passes the
  /// head width (`pcbnew/router/pns_line_placer.cpp:1425`, `:1742`).
  pub const fn set_tolerance(&mut self, tolerance: i32) {
    self.tolerance = tolerance;
  }

  /// Seed the current answer and the previous segment's direction.
  ///
  /// Port of `SetDefaultDirections`,
  /// `pcbnew/router/pns_mouse_trail_tracer.h:44`. Pass
  /// `Direction45::default()` for "there is no previous segment", which
  /// is KiCad's `DIRECTION_45::UNDEFINED`.
  pub const fn set_default_directions(
    &mut self,
    initial: Direction45,
    last_segment: Direction45,
  ) {
    self.direction = initial;
    self.last_seg_direction = last_segment;
  }

  /// Switch the mouse trail heuristic off or on.
  ///
  /// Port of `SetMouseDisabled`,
  /// `pcbnew/router/pns_mouse_trail_tracer.h:58`. The placer passes the
  /// negation of [`crate::settings::RoutingSettings::auto_posture`]
  /// (`pcbnew/router/pns_line_placer.cpp:1427`).
  ///
  /// Disabling has two effects and neither of them is "stop computing".
  /// The area derived direction is still computed but never assigned
  /// (`pcbnew/router/pns_mouse_trail_tracer.cpp:176`), and the fallback
  /// for a trail too short to judge answers
  /// `last_seg_direction.right()` instead of `last_seg_direction`
  /// (`:107`), that is, it alternates the posture on every fixed segment.
  pub const fn set_mouse_disabled(&mut self, disabled: bool) {
    self.disable_mouse = disabled;
  }

  /// Whether the user has pressed the posture key on this trail.
  ///
  /// Port of `IsManuallyForced`,
  /// `pcbnew/router/pns_mouse_trail_tracer.h:60`. It is sticky until the
  /// next [`MouseTrailTracer::clear`], and the placer reads it to suppress
  /// the smart pad pass (`pcbnew/router/pns_line_placer.cpp:764`, `:984`),
  /// to suppress the fanout cleanup (`:1045`) and to enable the tail
  /// dropping branch of `buildInitialLine` (`:2053`).
  pub const fn is_manually_forced(&self) -> bool {
    self.manually_forced
  }

  /// Turn the answer by 45 degrees and pin it there.
  ///
  /// Port of `FlipPosture`,
  /// `pcbnew/router/pns_mouse_trail_tracer.cpp:271`. A 45 degree turn on
  /// a direction that is not in 90 degree mode is exactly the straight to
  /// diagonal toggle, so this is the posture switch and not a rotation of
  /// the route.
  ///
  /// It sets both locks: [`MouseTrailTracer::get_posture`] stops
  /// consulting the trail from here until the next
  /// [`MouseTrailTracer::clear`].
  pub fn flip_posture(&mut self) {
    self.direction = self.direction.right();
    self.forced = true;
    self.manually_forced = true;
  }

  /// Record where the cursor is now.
  ///
  /// Port of `AddTrailPoint`,
  /// `pcbnew/router/pns_mouse_trail_tracer.cpp:47`. The trail grows by
  /// one point and then prunes itself: when it has more than two
  /// segments, the new segment is measured against every segment except
  /// the last two, and the first one within [`Self::set_tolerance`] of it
  /// cuts the trail back to that index (`:59` to `:69`). That is the "the
  /// user came back over their own path" detector, and it is why a long
  /// wandering route does not keep an unbounded history.
  ///
  /// The trail is [`LineChain::simplify`]d after every append (`:76`), so
  /// a slow drag along one direction costs one point, not hundreds.
  pub fn add_trail_point(&mut self, context: &AlgoContext<'_>, point: Vec2) {
    if self.trail.segment_count() == 0 {
      // :51
      self.trail.append(point);
    } else {
      // :55
      let new_segment = Seg::new(
        self
          .trail
          .last_point()
          .expect("a chain with a segment has a last point"),
        point,
      );

      // :57
      if self.trail.segment_count() > TRAIL_PRUNE_SKIPPED_SEGMENTS {
        // :59
        let limit = i64::from(self.tolerance) * i64::from(self.tolerance);
        let scanned = self.trail.segment_count() - TRAIL_PRUNE_SKIPPED_SEGMENTS;

        for index in 0..scanned {
          let trail_segment = self.trail.segment(index);

          // :65
          if trail_segment.squared_distance_to_segment(&new_segment) <= limit {
            // :67. The trail is always open, so the range cannot cross a
            // seam and the slice always succeeds.
            if let Ok(sliced) = self.trail.slice(0, index) {
              self.trail = sliced;
            }

            break;
          }
        }
      }

      // :73
      self.trail.append(point);
    }

    // :76
    self.trail.simplify(0);

    // :80
    if context.debug.is_enabled() {
      context.debug.add_shape(&self.trail, "mt-trail");
    }
  }

  /// Which posture the head should leave `at` in.
  ///
  /// Port of `GetPosture`,
  /// `pcbnew/router/pns_mouse_trail_tracer.cpp:84`. It takes the cursor
  /// position and nothing else, and it mutates the tracer: the answer,
  /// the lock and possibly the trail all change as a side effect, which
  /// is why this is `&mut self`.
  ///
  /// The algorithm in order:
  ///
  /// 1. A trail of fewer than two points, or a manual flip, short
  ///    circuits to the stored answer, first nudged towards the previous
  ///    segment (`:101` to `:110`).
  /// 2. Both candidate traces are built from the trail's first point to
  ///    `at`, closed against the reversed trail, and their areas taken
  ///    (`:115` to `:133`).
  /// 3. Coming back within [`UNLOCK_DISTANCE_FACTOR`] tolerances of the
  ///    trail's first point releases the lock and restarts the trail
  ///    (`:137` to `:144`).
  /// 4. The area test is believed only past
  ///    [`MIN_AREA_CUTOFF_DISTANCE_FACTOR`] tolerances and only if the
  ///    trail itself encloses more than `tolerance * distance`
  ///    (`:150` to `:158`).
  /// 5. The ratio decides, with [`AREA_RATIO_THRESHOLD`] and
  ///    [`AREA_RATIO_EPSILON`]; otherwise the stored posture is kept
  ///    (`:169` to `:174`).
  /// 6. The previous segment's direction overrides, preferring the least
  ///    obtuse continuation (`:185` to `:258`).
  /// 7. Past [`LOCK_DISTANCE_FACTOR`] tolerances the posture family
  ///    freezes (`:261` to `:265`); see the module documentation for what
  ///    that does and does not stop changing.
  ///
  /// # Deviations
  ///
  /// The two distance thresholds are computed in `i64` where KiCad
  /// multiplies two `int`s (`:137`, `:261`); a tolerance is a track width
  /// so the product cannot overflow in practice, and widening removes the
  /// question. `CSegment( 0 )` on a candidate with no segment would read
  /// KiCad's out of range fallback `SEG( back, back )`, whose direction is
  /// undefined; that case is spelled out here instead of relying on the
  /// fallback, see `first_segment_direction`.
  pub fn get_posture(
    &mut self,
    context: &AlgoContext<'_>,
    at: Vec2,
  ) -> Direction45 {
    // :101
    if self.trail.point_count() < 2 || self.manually_forced {
      // :106
      if !self.manually_forced && self.last_seg_direction.is_defined() {
        self.direction = if self.disable_mouse {
          self.last_seg_direction.right()
        } else {
          self.last_seg_direction
        };
      }

      return self.direction;
    }

    // :113
    let first = self.trail.point(0);
    let reference_length = f64::from(Seg::new(first, at).length());

    // :115. `DIRECTION_45()` is undefined, so the posture argument is
    // what decides, and the corner mode argument is left at its default
    // (`libs/kimath/include/geometry/direction45.h:236`). The posture
    // solver therefore always reasons in 45 degree corners, whatever the
    // routing settings say.
    let mut straight = LineChain::from_points(
      Direction45::default().build_initial_trace(
        first,
        at,
        false,
        CornerMode::Mitered45,
      ),
      false,
    );

    // :117. The straight candidate is closed before the trail is
    // appended and the diagonal one after; see the note below.
    straight.set_closed(true);
    straight.append_chain(&self.trail.reversed());
    straight.simplify(0);

    if context.debug.is_enabled() {
      context.debug.add_shape(&straight, "mt-straight");
    }

    // :123
    let area_straight = straight.area(true);

    // :125
    let mut diagonal = LineChain::from_points(
      Direction45::default().build_initial_trace(
        first,
        at,
        true,
        CornerMode::Mitered45,
      ),
      false,
    );

    // :126. KiCad appends first and closes afterwards here, the opposite
    // order from the straight candidate. Both orders give the same chain,
    // because the duplicate of the first point that the reversed trail
    // ends with is dropped either by the append (when the chain is
    // already closed) or by the close (when it is not); the asymmetry is
    // reproduced so that the two branches can be diffed against KiCad
    // line by line.
    diagonal.append_chain(&self.trail.reversed());
    diagonal.set_closed(true);
    diagonal.simplify(0);

    if context.debug.is_enabled() {
      context.debug.add_shape(&diagonal, "mt-diag");
    }

    // :132
    let area_diagonal = diagonal.area(true);
    let ratio = area_straight / (area_diagonal + 1.0);

    // :137. The user dragged back to where the trace started: drop the
    // lock and restart the trail there.
    if self.forced
      && reference_length
        < unscale(UNLOCK_DISTANCE_FACTOR, self.tolerance) as f64
    {
      if context.debug.is_enabled() {
        context.debug.message("Posture: Unlocked and reset");
      }

      self.forced = false;
      self.trail.clear();
      self.trail.append(first);
    }

    // :146
    let mut area_ok = false;

    // :150
    if !self.forced
      && reference_length
        > MIN_AREA_CUTOFF_DISTANCE_FACTOR * f64::from(self.tolerance)
    {
      // :152
      let area_cutoff = f64::from(self.tolerance) * reference_length;
      let mut closed_trail = self.trail.clone();

      closed_trail.set_closed(true);

      if closed_trail.area(true) > area_cutoff {
        area_ok = true;
      }
    }

    // :166
    let straight_direction = first_segment_direction(&straight);
    let diagonal_direction = first_segment_direction(&diagonal);

    // :169
    let new_direction = if !self.forced
      && area_ok
      && ratio > AREA_RATIO_THRESHOLD + AREA_RATIO_EPSILON
    {
      diagonal_direction
    } else if !self.forced
      && area_ok
      && ratio < (1.0 / AREA_RATIO_THRESHOLD) - AREA_RATIO_EPSILON
    {
      straight_direction
    } else if self.direction.is_diagonal() {
      diagonal_direction
    } else {
      straight_direction
    };

    // :176. With the mouse heuristic off everything above was computed
    // and is then thrown away.
    if !self.disable_mouse && new_direction != self.direction {
      self.direction = new_direction;
    }

    // :185
    if !self.manually_forced
      && !self.disable_mouse
      && self.last_seg_direction.is_defined()
    {
      self.correct_against_last_segment(straight_direction, diagonal_direction);
    }

    // :261
    if !self.forced
      && reference_length > unscale(LOCK_DISTANCE_FACTOR, self.tolerance) as f64
    {
      if context.debug.is_enabled() {
        context.debug.message("Posture: solution locked");
      }

      self.forced = true;
    }

    self.direction
  }

  /// Pull the answer towards the least obtuse continuation of the
  /// previously fixed segment.
  ///
  /// The `else` ladder of `GetPosture`,
  /// `pcbnew/router/pns_mouse_trail_tracer.cpp:191` to `:257`. A
  /// candidate that already runs in the previous segment's direction wins
  /// outright; otherwise the current answer is judged by the angle it
  /// makes with that segment and swapped for the other candidate when
  /// swapping improves it.
  ///
  /// The three cases are exactly KiCad's: a hairpin
  /// ([`AngleType::HALF_FULL`]) always swaps, an acute corner swaps only
  /// if the other candidate makes a right angle, and a right angle swaps
  /// only if the other candidate makes an obtuse one. A straight or
  /// already obtuse continuation is left alone.
  fn correct_against_last_segment(
    &mut self,
    straight_direction: Direction45,
    diagonal_direction: Direction45,
  ) {
    // :191
    if straight_direction == self.last_seg_direction {
      self.direction = straight_direction;
      return;
    }

    // :201
    if diagonal_direction == self.last_seg_direction {
      self.direction = diagonal_direction;
      return;
    }

    // :225, :241. The candidate that is not the current answer.
    let other = if self.direction.is_diagonal() {
      straight_direction
    } else {
      diagonal_direction
    };
    let angle = self.direction.angle(self.last_seg_direction);

    // :215
    if angle == AngleType::HALF_FULL {
      self.direction = other;
    } else if angle == AngleType::ACUTE {
      // :228
      if other.angle(self.last_seg_direction) == AngleType::RIGHT {
        self.direction = other;
      }
    } else if angle == AngleType::RIGHT {
      // :244
      if other.angle(self.last_seg_direction) == AngleType::OBTUSE {
        self.direction = other;
      }
    }
  }
}

/// The direction of a chain's first segment, undefined when it has none.
///
/// The `DIRECTION_45( straight.CSegment( 0 ) )` of
/// `pcbnew/router/pns_mouse_trail_tracer.cpp:166` and `:167`. KiCad's
/// `CSegment` answers an out of range index with the degenerate
/// `SEG( back, back )` (`libs/kimath/src/geometry/shape_line_chain.cpp:1285`),
/// whose direction is undefined; this crate's
/// [`LineChain::segment`] panics instead, so the empty case is spelled
/// out.
///
/// Both call sites are reached only with a candidate that carries at
/// least two points of trail, so the fallback is unreachable in practice.
fn first_segment_direction(chain: &LineChain) -> Direction45 {
  if chain.segment_count() == 0 {
    return Direction45::default();
  }

  Direction45::from_seg(&chain.segment(0), false)
}

/// A distance threshold in tolerances, widened out of KiCad's `int`.
///
/// The `lockDistanceFactor * m_tolerance` and
/// `unlockDistanceFactor * m_tolerance` products of
/// `pcbnew/router/pns_mouse_trail_tracer.cpp:137` and `:261`, which KiCad
/// computes in `int`.
const fn unscale(factor: i32, tolerance: i32) -> i64 {
  factor as i64 * tolerance as i64
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::debug::NoDebug;
  use crate::geometry::direction45::Octant;
  use crate::rules::FixedClearance;
  use crate::settings::RoutingSettings;

  /// The tolerance every test uses, a plausible track width in
  /// nanometres.
  const TOLERANCE: i32 = 100_000;

  /// Everything an algorithm is handed, with nothing listening.
  struct Fixture {
    /// The rule oracle, unused by the posture solver.
    rules: FixedClearance,
    /// The settings, unused by the posture solver.
    settings: RoutingSettings,
  }

  impl Fixture {
    fn new() -> Self {
      Self {
        rules: FixedClearance::uniform(1000),
        settings: RoutingSettings::default(),
      }
    }

    fn context(&self) -> AlgoContext<'_> {
      AlgoContext::new(&self.rules, &self.settings)
    }
  }

  /// A tracer with the test tolerance and no seeded directions.
  fn tracer() -> MouseTrailTracer {
    let mut tracer = MouseTrailTracer::new();

    tracer.set_tolerance(TOLERANCE);
    tracer
  }

  /// Feed a whole trail in one go.
  fn feed(
    tracer: &mut MouseTrailTracer,
    context: &AlgoContext<'_>,
    points: &[Vec2],
  ) {
    for point in points {
      tracer.add_trail_point(context, *point);
    }
  }

  #[test]
  fn a_fresh_tracer_answers_undefined() {
    let fixture = Fixture::new();
    let mut tracer = tracer();

    assert_eq!(
      tracer.get_posture(&fixture.context(), Vec2::new(1000, 1000)),
      Direction45::default()
    );
  }

  #[test]
  fn a_trail_of_one_point_keeps_the_seeded_direction() {
    let fixture = Fixture::new();
    let mut tracer = tracer();

    tracer.set_default_directions(
      Direction45::from_octant(Octant::N),
      Direction45::default(),
    );
    feed(&mut tracer, &fixture.context(), &[Vec2::new(0, 0)]);

    assert_eq!(
      tracer.get_posture(&fixture.context(), Vec2::new(500_000, -300_000)),
      Direction45::from_octant(Octant::N)
    );
  }

  /// East then north east: the trail hugs the straight first candidate,
  /// so the straight first posture wins and the answer is east.
  ///
  /// The tracer is seeded with a **diagonal** direction on purpose. The
  /// fallback of `:174` would then answer with the diagonal candidate, so
  /// only the area comparison can produce east and the test cannot pass
  /// by accident.
  #[test]
  fn a_straight_then_diagonal_trail_chooses_straight_first() {
    let fixture = Fixture::new();
    let mut tracer = tracer();
    let end = Vec2::new(2_000_000, -1_000_000);

    tracer.set_default_directions(
      Direction45::from_octant(Octant::NE),
      Direction45::default(),
    );
    feed(
      &mut tracer,
      &fixture.context(),
      &[Vec2::new(0, 0), Vec2::new(1_000_000, 0), end],
    );

    assert_eq!(
      tracer.get_posture(&fixture.context(), end),
      Direction45::from_octant(Octant::E)
    );
  }

  /// The mirror image: north east then east, so the diagonal first
  /// posture wins and the answer is north east.
  #[test]
  fn a_diagonal_then_straight_trail_chooses_diagonal_first() {
    let fixture = Fixture::new();
    let mut tracer = tracer();
    let end = Vec2::new(2_000_000, -1_000_000);

    feed(
      &mut tracer,
      &fixture.context(),
      &[Vec2::new(0, 0), Vec2::new(1_000_000, -1_000_000), end],
    );

    assert_eq!(
      tracer.get_posture(&fixture.context(), end),
      Direction45::from_octant(Octant::NE)
    );
  }

  /// The same shape as the straight first case, scaled down below the
  /// minimum area cutoff. The area test is not consulted, so the seeded
  /// diagonal posture survives.
  #[test]
  fn a_tiny_trail_keeps_the_previous_posture() {
    let fixture = Fixture::new();
    let mut tracer = tracer();
    let end = Vec2::new(200_000, -100_000);

    tracer.set_default_directions(
      Direction45::from_octant(Octant::NE),
      Direction45::default(),
    );
    feed(
      &mut tracer,
      &fixture.context(),
      &[Vec2::new(0, 0), Vec2::new(100_000, 0), end],
    );

    // The trail runs 100000 nm east and then a short diagonal, so the
    // reference length stays under 6 * 100000 and the ratio never gets a
    // vote. The answer is the diagonal candidate because the seeded
    // direction is diagonal.
    assert!(tracer.get_posture(&fixture.context(), end).is_diagonal());
  }

  /// Past 30 tolerances the answer freezes into its posture family: the
  /// area comparison stops voting and the fallback of `:174` keeps
  /// answering with the straight candidate for a straight direction and
  /// the diagonal one for a diagonal direction.
  ///
  /// Two tracers see exactly the same trail. The first is asked for a
  /// posture after the opening leg, which is long enough to lock it; the
  /// second is only asked at the end. They disagree, and that
  /// disagreement is the lock.
  #[test]
  fn the_lock_engages_after_a_long_trail() {
    let fixture = Fixture::new();
    let opening = Vec2::new(4_000_000, 0);
    let end = Vec2::new(4_000_000, -8_000_000);

    let mut locked = tracer();
    let mut unlocked = tracer();

    feed(&mut locked, &fixture.context(), &[Vec2::new(0, 0), opening]);

    // 4000000 nm is past 30 * 100000, so this call freezes the answer.
    assert_eq!(
      locked.get_posture(&fixture.context(), opening),
      Direction45::from_octant(Octant::E)
    );

    feed(&mut locked, &fixture.context(), &[end]);
    feed(
      &mut unlocked,
      &fixture.context(),
      &[Vec2::new(0, 0), opening, end],
    );

    // The trail now encloses three times more area against the straight
    // first candidate than against the diagonal one, so an unlocked
    // tracer switches to the diagonal posture and a locked one does not.
    assert_eq!(
      unlocked.get_posture(&fixture.context(), end),
      Direction45::from_octant(Octant::NE)
    );
    assert_eq!(
      locked.get_posture(&fixture.context(), end),
      Direction45::from_octant(Octant::N)
    );
  }

  /// Coming back within 10 tolerances of the start releases the lock and
  /// restarts the trail from that point.
  #[test]
  fn coming_back_to_the_start_releases_the_lock() {
    let fixture = Fixture::new();
    let mut tracer = tracer();
    let far = Vec2::new(8_000_000, -4_000_000);

    feed(
      &mut tracer,
      &fixture.context(),
      &[Vec2::new(0, 0), Vec2::new(4_000_000, 0), far],
    );
    tracer.get_posture(&fixture.context(), far);

    // Back near the origin: the lock goes and the trail is one point
    // again, so the next answer is the short circuit of `:101`.
    tracer.get_posture(&fixture.context(), Vec2::new(100_000, 0));

    // A fresh diagonal trail now wins. While the lock held the fallback
    // would have kept answering with the straight candidate, which for
    // this pair of points is east.
    let end = Vec2::new(2_000_000, -1_000_000);

    feed(
      &mut tracer,
      &fixture.context(),
      &[Vec2::new(1_000_000, -1_000_000), end],
    );

    assert_eq!(
      tracer.get_posture(&fixture.context(), end),
      Direction45::from_octant(Octant::NE)
    );
  }

  /// A manual flip turns the answer by 45 degrees and outranks the trail
  /// from then on.
  #[test]
  fn a_manual_flip_overrides_the_trail() {
    let fixture = Fixture::new();
    let mut tracer = tracer();
    let end = Vec2::new(2_000_000, -1_000_000);

    feed(
      &mut tracer,
      &fixture.context(),
      &[Vec2::new(0, 0), Vec2::new(1_000_000, 0), end],
    );

    let before = tracer.get_posture(&fixture.context(), end);

    assert_eq!(before, Direction45::from_octant(Octant::E));
    assert!(!tracer.is_manually_forced());

    tracer.flip_posture();

    assert!(tracer.is_manually_forced());
    assert_eq!(tracer.get_posture(&fixture.context(), end), before.right());

    // Even a trail that argues hard for the other posture is ignored.
    feed(
      &mut tracer,
      &fixture.context(),
      &[Vec2::new(2_000_000, -2_000_000)],
    );

    assert_eq!(
      tracer.get_posture(&fixture.context(), Vec2::new(3_000_000, -2_500_000)),
      before.right()
    );
  }

  /// `Clear` releases both locks and empties the trail while leaving the
  /// answer, the previous segment and the tolerance alone.
  #[test]
  fn clear_keeps_the_direction_and_the_tolerance() {
    let fixture = Fixture::new();
    let mut tracer = tracer();

    feed(
      &mut tracer,
      &fixture.context(),
      &[Vec2::new(0, 0), Vec2::new(1_000_000, 0)],
    );
    tracer.flip_posture();

    let flipped = tracer.get_posture(&fixture.context(), Vec2::new(0, 0));

    tracer.clear();

    assert!(!tracer.is_manually_forced());
    // The trail is empty, so the short circuit answers with the kept
    // direction.
    assert_eq!(
      tracer.get_posture(&fixture.context(), Vec2::new(5_000_000, 0)),
      flipped
    );
  }

  /// With the mouse heuristic off the previous segment's direction is
  /// turned by 45 degrees on every query, which is "switch posture every
  /// segment".
  #[test]
  fn a_disabled_mouse_alternates_from_the_last_segment() {
    let fixture = Fixture::new();
    let mut tracer = tracer();

    tracer.set_mouse_disabled(true);
    tracer.set_default_directions(
      Direction45::from_octant(Octant::N),
      Direction45::from_octant(Octant::E),
    );

    assert_eq!(
      tracer.get_posture(&fixture.context(), Vec2::new(1_000_000, 0)),
      Direction45::from_octant(Octant::E).right()
    );
  }

  /// The mouse derived direction is computed and thrown away when the
  /// heuristic is off, so a trail that would flip the posture does not.
  #[test]
  fn a_disabled_mouse_ignores_the_trail() {
    let fixture = Fixture::new();
    let mut tracer = tracer();
    let end = Vec2::new(2_000_000, -1_000_000);

    tracer.set_mouse_disabled(true);
    tracer.set_default_directions(
      Direction45::from_octant(Octant::NE),
      Direction45::default(),
    );
    feed(
      &mut tracer,
      &fixture.context(),
      &[Vec2::new(0, 0), Vec2::new(1_000_000, 0), end],
    );

    assert_eq!(
      tracer.get_posture(&fixture.context(), end),
      Direction45::from_octant(Octant::NE)
    );
  }

  /// Walking back over the trail cuts it back rather than letting it
  /// grow, which is what keeps a long route's history bounded.
  #[test]
  fn a_trail_that_doubles_back_prunes_itself() {
    let fixture = Fixture::new();
    let mut tracer = tracer();
    let context = fixture.context();
    let out = [
      Vec2::new(0, 0),
      Vec2::new(1_000_000, 0),
      Vec2::new(1_000_000, -1_000_000),
      Vec2::new(2_000_000, -1_000_000),
    ];

    feed(&mut tracer, &context, &out);

    let grown = tracer.trail.point_count();

    // Retrace the first leg exactly: the new segment lands on segment 0,
    // which is outside the two segment skip, so the trail is cut back.
    tracer.add_trail_point(&context, Vec2::new(500_000, 0));

    assert!(tracer.trail.point_count() < grown);
  }

  #[test]
  fn the_same_input_answers_identically_twice() {
    let fixture = Fixture::new();
    let points = [
      Vec2::new(0, 0),
      Vec2::new(1_500_000, 0),
      Vec2::new(2_500_000, -700_000),
      Vec2::new(3_100_000, -700_000),
    ];
    let query = Vec2::new(3_500_000, -1_200_000);

    let run = || {
      let mut tracer = tracer();

      feed(&mut tracer, &fixture.context(), &points);
      tracer.get_posture(&fixture.context(), query)
    };

    assert_eq!(run(), run());
  }

  /// The debug hook sees the trail and both candidates, and nothing it
  /// does changes the answer.
  #[test]
  fn the_debug_hook_sees_the_trail_and_both_candidates() {
    use std::cell::RefCell;

    #[derive(Default)]
    struct Recorder {
      /// Every shape label, in order.
      shapes: RefCell<Vec<String>>,
    }

    impl crate::debug::DebugDecorator for Recorder {
      fn add_shape(&self, shape: &LineChain, text: &str) {
        let _ = shape;
        self.shapes.borrow_mut().push(text.to_string());
      }
    }

    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let recorder = Recorder::default();
    let traced = AlgoContext::new(&rules, &settings).with_debug(&recorder);
    let silent = AlgoContext::new(&rules, &settings).with_debug(&NoDebug);
    let points = [
      Vec2::new(0, 0),
      Vec2::new(1_000_000, 0),
      Vec2::new(2_000_000, -1_000_000),
    ];
    let query = Vec2::new(2_000_000, -1_000_000);

    let mut traced_tracer = tracer();
    let mut silent_tracer = tracer();

    feed(&mut traced_tracer, &traced, &points);
    feed(&mut silent_tracer, &silent, &points);

    let with_trace = traced_tracer.get_posture(&traced, query);
    let without = silent_tracer.get_posture(&silent, query);

    assert_eq!(with_trace, without);
    assert!(
      recorder
        .shapes
        .borrow()
        .iter()
        .any(|label| label == "mt-trail")
    );
    assert!(
      recorder
        .shapes
        .borrow()
        .iter()
        .any(|label| label == "mt-straight")
    );
    assert!(
      recorder
        .shapes
        .borrow()
        .iter()
        .any(|label| label == "mt-diag")
    );
  }
}
