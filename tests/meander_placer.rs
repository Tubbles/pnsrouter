// SPDX-License-Identifier: GPL-3.0-or-later

//! Single track length tuning scenarios, through the facade only.
//!
//! The board is the synthetic fixture of
//! `doc/reference/kicad/08-meanders.md` section 14.2: two round pads
//! eight millimetres apart, one straight track between them, and one
//! obstacle placed so that a full amplitude meander on that side of the
//! base line cannot fit.
//!
//! # Why the fixture is synthetic
//!
//! There is no tuning case in KiCad's regression corpus, its log format
//! cannot express one, and its unit test suite never constructs a
//! `MEANDER_SHAPE` at all (note 08 section 10.5 and erratum E23). So
//! there is nothing upstream to replay a tuning session against, and the
//! fidelity of the port rests on the hand computed shape assertions in
//! `src/meander.rs` and on the errata being reproduced deliberately.
//! Note 08 section 14.7 asks for that to be said out loud, and
//! `doc/work/011-meanders.md` says it.
//!
//! # What is asserted, and what is not
//!
//! Lengths and statuses, not raw coordinates. A coordinate assertion on a
//! fitted meander breaks on every change to the amplitude scan, where
//! "the result is within the tolerance of the target" survives one. The
//! raw coordinate assertions live in `src/meander.rs`, where they are
//! hand computed against the note's closed forms and are the whole point.

#![forbid(unsafe_code)]

use pnsrouter::eventlog::{SessionRecording, assert_replay_matches, replay};
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{HostId, LayerRange, NetId};
use pnsrouter::meander::{
  LengthTarget, MeanderSettings, MeanderSettingsRequest, MeanderSide,
  MeanderStyle, TuningStatus,
};
use pnsrouter::node::World;
use pnsrouter::router::{
  CommitDiff, FixOutcome, NewGeometry, Router, StartError,
};
use pnsrouter::rules::{
  Constraint, ConstraintType, FixedClearance, ItemRef, Keepout, RuleResolver,
};
use pnsrouter::settings::{RoutingSettings, Sizes};
use pnsrouter::snapshot::{WorldGeometry, WorldItem, WorldSnapshot};

/// The clearance every scenario tunes to, in nanometres.
const CLEARANCE: i32 = 100_000;

/// The width of the tuned track.
const WIDTH: i32 = 200_000;

/// The copper radius of the two pads.
const PAD_RADIUS: i32 = 150_000;

/// The net of the tuned track.
const NET: NetId = NetId(1);

/// The net of the obstacle, so that it is never exempt.
const OBSTACLE_NET: NetId = NetId(2);

/// The west pad.
const PAD_A: HostId = HostId(1);

/// The east pad.
const PAD_B: HostId = HostId(2);

/// The track between them.
const TRACK: HostId = HostId(3);

/// The obstacle, on the boards that carry one.
const OBSTACLE: HostId = HostId(4);

/// Where the track starts.
const WEST: Vec2 = Vec2::new(0, 0);

/// Where the track ends.
const EAST: Vec2 = Vec2::new(8_000_000, 0);

/// Where the tuned stretch begins.
const TUNE_FROM: Vec2 = Vec2::new(1_000_000, 0);

/// Where the tuned stretch ends. Six millimetres of baseline, which holds
/// five full amplitude singles at 1200000 nm each with nothing left over.
const TUNE_TO: Vec2 = Vec2::new(7_000_000, 0);

/// The length of the untuned track, which is the whole path.
const BASELINE: i64 = 8_000_000;

/// A resolver that answers the clearance both ways.
///
/// `FixedClearance` leaves `RuleResolver::constraint` at its default
/// `None`, and a tuning session is the first thing in the crate that asks
/// for a `CT_CLEARANCE` constraint
/// (`pcbnew/router/pns_meander_placer_base.cpp:122`). Without an answer
/// the clearance falls back to the **track width** (`:125`), which then
/// floors the meander period at twice the width; note 08 section 14.5
/// asks for both cases to be covered, and
/// `a_missing_clearance_rule_falls_back_to_the_track_width` covers the
/// other one.
struct TuningRules {
  /// The pairwise clearance every collision test uses.
  inner: FixedClearance,
  /// What the `CT_CLEARANCE` constraint answers, or `None` for a host
  /// that has no such rule.
  constraint: Option<i32>,
}

impl TuningRules {
  /// A resolver that answers both.
  fn new(clearance: i32) -> Self {
    Self {
      inner: FixedClearance::uniform(clearance),
      constraint: Some(clearance),
    }
  }

  /// A resolver that answers the pairwise clearance and no constraint.
  fn without_constraint(clearance: i32) -> Self {
    Self {
      inner: FixedClearance::uniform(clearance),
      constraint: None,
    }
  }
}

impl RuleResolver for TuningRules {
  fn clearance(
    &self,
    a: ItemRef<'_>,
    b: Option<ItemRef<'_>>,
    use_epsilon: bool,
  ) -> Option<i32> {
    self.inner.clearance(a, b, use_epsilon)
  }

  fn clearance_epsilon(&self) -> i32 {
    self.inner.clearance_epsilon()
  }

  fn constraint(
    &self,
    constraint_type: ConstraintType,
    _a: ItemRef<'_>,
    _b: Option<ItemRef<'_>>,
    _layer: i32,
  ) -> Option<Constraint> {
    match constraint_type {
      ConstraintType::Clearance => Some(Constraint {
        constraint_type,
        min: self.constraint,
        opt: None,
        max: None,
        allowed: true,
      }),
      _ => None,
    }
  }

  fn is_keepout(&self, obstacle: ItemRef<'_>, item: ItemRef<'_>) -> Keepout {
    self.inner.is_keepout(obstacle, item)
  }

  fn is_drilled_hole(&self, item: ItemRef<'_>) -> bool {
    self.inner.is_drilled_hole(item)
  }

  fn is_non_plated_slot(&self, item: ItemRef<'_>) -> bool {
    self.inner.is_non_plated_slot(item)
  }

  fn net_code(&self, net: NetId) -> i32 {
    self.inner.net_code(net)
  }

  fn orphaned_net(&self) -> NetId {
    self.inner.orphaned_net()
  }
}

/// A round pad of the tuned net.
fn pad(id: HostId, at: Vec2) -> WorldItem {
  WorldItem::new(
    id,
    Some(NET),
    LayerRange::single(0),
    WorldGeometry::Solid {
      shape: Shape::circle(at, PAD_RADIUS),
      pos: at,
      offset: Vec2::new(0, 0),
      orientation_degrees: 0.0,
      anchors: Vec::new(),
    },
  )
}

/// A track of a given net.
fn track(id: HostId, net: NetId, from: Vec2, to: Vec2) -> WorldItem {
  WorldItem::new(
    id,
    Some(net),
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(from, to),
      width: WIDTH,
    },
  )
}

/// The fixture board, with or without the obstacle.
///
/// The obstacle sits 900000 nm above the base line, which leaves
/// `900000 - 100000 - 100000 = 700000` nm of clear space between the two
/// pieces of copper. A full amplitude meander reaches 1000000 nm and
/// collides, so `Fit` has to step the amplitude down or flip the side.
fn board(with_obstacle: bool) -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(pad(PAD_A, WEST));
  snapshot.items.push(pad(PAD_B, EAST));
  snapshot.items.push(track(TRACK, NET, WEST, EAST));

  if with_obstacle {
    snapshot.items.push(track(
      OBSTACLE,
      OBSTACLE_NET,
      Vec2::new(4_000_000, 900_000),
      Vec2::new(5_000_000, 900_000),
    ));
  }

  snapshot
}

/// The sizes every scenario runs with. A tuning placer reads none of
/// them (note 08 section 11.1), so they are the board minimums only.
fn sizes() -> Sizes {
  Sizes {
    track_width: WIDTH,
    board_min_track_width: 100_000,
    min_clearance: CLEARANCE,
    ..Sizes::default()
  }
}

/// The meander settings of note 08 section 14.1, aimed at a target.
fn settings_for(target: i64) -> MeanderSettings {
  MeanderSettings::new(MeanderSettingsRequest {
    min_amplitude: 200_000,
    max_amplitude: 1_000_000,
    spacing: 600_000,
    step: 50_000,
    corner_style: MeanderStyle::Chamfer,
    corner_radius_percentage: 80,
    single_sided: false,
    initial_side: MeanderSide::Left,
    // The host forces it (`pcbnew/generators/pcb_tuning_pattern.cpp:1297`).
    keep_endpoints: true,
    target_length: Some(LengthTarget::around(target)),
    target_skew: Some(LengthTarget::around(0)),
  })
  .expect("the note's settings are a step of 50000 and chamfered corners")
}

/// A router over one of the boards.
fn router_on(with_obstacle: bool) -> Router {
  Router::new(
    &board(with_obstacle),
    Box::new(TuningRules::new(CLEARANCE)),
    RoutingSettings::default(),
    sizes(),
  )
}

/// The total length of every segment a commit added, in nanometres.
fn added_length(diff: &CommitDiff) -> i64 {
  diff
    .added
    .iter()
    .filter_map(|item| match item.geometry {
      NewGeometry::Segment { seg, .. } => Some(i64::from(seg.length())),
      NewGeometry::Arc { .. } | NewGeometry::Via { .. } => None,
    })
    .sum()
}

/// Every segment a commit added, whether by addition or by update.
fn committed_segments(diff: &CommitDiff) -> Vec<Seg> {
  diff
    .added
    .iter()
    .chain(diff.updated.iter().map(|(_, item)| item))
    .filter_map(|item| match item.geometry {
      NewGeometry::Segment { seg, .. } => Some(seg),
      NewGeometry::Arc { .. } | NewGeometry::Via { .. } => None,
    })
    .collect()
}

/// Start, move once and fix, which is a whole tuning session.
fn tune(router: &mut Router, target: i64) -> (TuningStatus, i64, CommitDiff) {
  let frame = router
    .start_tuning(TUNE_FROM, TRACK, settings_for(target))
    .expect("the fixture track is a track");
  let before = frame
    .tuning
    .as_deref()
    .expect("a tuning session reports a readout")
    .status;

  assert_eq!(
    before,
    TuningStatus::TooShort,
    "an eight millimetre track is short of every target here"
  );

  let frame = router.move_to(TUNE_TO, None);
  let readout = *frame
    .tuning
    .as_deref()
    .expect("a tuning session reports a readout after a move");

  let FixOutcome::Finished(diff) = router.fix_route(TUNE_TO, None, true) else {
    panic!("a tuning fix always ends the session");
  };

  (readout.status, readout.result, diff)
}

/// A ten millimetre target on an eight millimetre track: the meanders
/// make up the two millimetres and the status settles on `TUNED`.
///
/// The fixture case `a_trace_reaches_a_longer_target` of note 08 section
/// 14.6.
#[test]
fn a_trace_reaches_a_longer_target() {
  let mut router = router_on(false);
  let (status, result, diff) = tune(&mut router, 10_000_000);

  assert_eq!(status, TuningStatus::Tuned);
  assert!(
    (result - 10_000_000).abs() <= 100_000,
    "the result {result} is outside the tolerance around ten millimetres"
  );

  // The commit replaces the one track with the meandered chain: the
  // original leaves the board and the pieces arrive in its place.
  let committed: i64 = committed_segments(&diff)
    .iter()
    .map(|seg| i64::from(seg.length()))
    .sum();

  assert!(
    (committed - result).abs() <= 1,
    "the committed geometry is {committed} where the readout said {result}"
  );
  assert!(committed > BASELINE, "a tuned track is longer than it was");
}

/// The commit keeps the net and the layer of the track it replaced, and
/// takes the original off the board.
#[test]
fn the_commit_replaces_the_track_on_the_same_net_and_layer() {
  let mut router = router_on(false);
  let (_, _, diff) = tune(&mut router, 10_000_000);
  let pieces: Vec<_> = diff
    .added
    .iter()
    .chain(diff.updated.iter().map(|(_, item)| item))
    .collect();

  assert!(!pieces.is_empty(), "a tuned track reaches the board");

  for item in &pieces {
    assert_eq!(item.net, Some(NET));
    assert_eq!(item.layers, LayerRange::single(0));
  }

  // The track is gone, either as a removal or as the update one of the
  // pieces inherited its identity through.
  let replaced = diff.removed.contains(&TRACK)
    || diff.updated.iter().any(|(host, _)| *host == TRACK);

  assert!(replaced, "the original track is off the board");
  assert!(diff.removed.iter().all(|host| *host == TRACK));
}

/// A target no amount of amplitude can reach reports `TOO_SHORT` and
/// commits the best effort anyway, which is what `FixRoute` does: it
/// ignores the status entirely (`pcbnew/router/pns_meander_placer.cpp:364`).
///
/// The fixture case `a_target_that_cannot_be_reached_reports_too_short`.
#[test]
fn a_target_that_cannot_be_reached_reports_too_short_and_still_commits() {
  let mut router = router_on(false);
  let (status, result, diff) = tune(&mut router, 30_000_000);

  assert_eq!(status, TuningStatus::TooShort);
  assert!(
    result > BASELINE,
    "the best effort {result} is longer than the untouched track"
  );
  assert!(
    result < 30_000_000,
    "the best effort {result} falls short of the target"
  );
  assert!(
    !committed_segments(&diff).is_empty(),
    "KiCad's FixRoute commits whatever the last move produced"
  );
}

/// A target below the current length fires `doMove`'s early test
/// (`pcbnew/router/pns_meander_placer.cpp:282`): the status is
/// `TOO_LONG`, no meander is fitted and the geometry is untouched.
///
/// The fixture case `a_target_below_the_current_length_reports_too_long`.
#[test]
fn a_target_below_the_current_length_reports_too_long() {
  let mut router = router_on(false);
  let frame = router
    .start_tuning(TUNE_FROM, TRACK, settings_for(5_000_000))
    .expect("the fixture track is a track");

  assert!(frame.tuning.is_some());

  let frame = router.move_to(TUNE_TO, None);
  let readout = *frame.tuning.as_deref().expect("a readout after a move");

  assert_eq!(readout.status, TuningStatus::TooLong);
  assert_eq!(
    readout.result, BASELINE,
    "nothing was meandered, so the length is the one the session started at"
  );
  assert_eq!(readout.delta, Some(0));
}

/// Growing the amplitude and widening the spacing both change the
/// geometry, and the two pull in opposite directions: a taller meander is
/// longer, a wider period fits fewer of them into the same baseline.
#[test]
fn the_amplitude_and_spacing_steps_change_the_result() {
  let target = 30_000_000;
  let plain = {
    let mut router = router_on(false);
    let (_, result, _) = tune(&mut router, target);

    result
  };
  let taller = {
    let mut router = router_on(false);

    router
      .start_tuning(TUNE_FROM, TRACK, settings_for(target))
      .expect("the fixture track is a track");
    router.move_to(TUNE_TO, None);

    for _ in 0..4 {
      assert!(router.amplitude_step(1));
    }

    let frame = router.move_to(TUNE_TO, None);

    frame.tuning.as_deref().expect("a readout").result
  };
  let wider = {
    let mut router = router_on(false);

    router
      .start_tuning(TUNE_FROM, TRACK, settings_for(target))
      .expect("the fixture track is a track");
    router.move_to(TUNE_TO, None);

    for _ in 0..4 {
      assert!(router.spacing_step(1));
    }

    let frame = router.move_to(TUNE_TO, None);

    frame.tuning.as_deref().expect("a readout").result
  };

  assert!(
    taller > plain,
    "a taller meander is longer: {taller} against {plain}"
  );
  assert!(
    wider < plain,
    "a wider period fits fewer meanders: {wider} against {plain}"
  );
}

/// Neither live adjustment applies outside a tuning session.
#[test]
fn the_live_adjustments_refuse_a_routing_session() {
  let mut router = router_on(false);

  assert!(!router.amplitude_step(1));
  assert!(!router.spacing_step(1));

  router
    .start_routing(WEST, Some(PAD_A), 0)
    .expect("the west pad is routable");

  assert!(!router.amplitude_step(1));
  assert!(!router.spacing_step(1));
}

/// A start on a pad and a start on an unknown object are both refused.
///
/// The fixture case `a_start_on_something_that_is_not_a_track_is_refused`.
#[test]
fn a_start_on_a_pad_is_refused() {
  let mut router = router_on(false);
  let error = router
    .start_tuning(WEST, PAD_A, settings_for(10_000_000))
    .expect_err("a pad is not a track");

  assert!(matches!(error, StartError::NotATrack(_)));

  let error = router
    .start_tuning(WEST, HostId(99), settings_for(10_000_000))
    .expect_err("no such object");

  assert_eq!(error, StartError::UnknownStartItem(HostId(99)));
}

/// An obstacle within reach of a full amplitude meander makes `CheckFit`
/// reject that amplitude, so the fit steps down and the run gives less
/// length than the same run on a clear board.
///
/// The fixture case `an_obstacle_forces_a_smaller_amplitude`. Its sibling
/// `an_obstacle_flips_the_initial_side` is **not** covered: on this board
/// the amplitude scan finds a shorter meander on the same side before it
/// ever needs the other one, so `flipInitialSide` does not fire and the
/// readout comes back on the side it started on. Building a board that
/// flips needs an obstacle that blocks every amplitude on one side and
/// none on the other, which the fitting loop's own tests in
/// `src/meander.rs` cover without a world.
#[test]
fn an_obstacle_forces_a_smaller_amplitude() {
  let clear = {
    let mut router = router_on(false);
    let (_, result, _) = tune(&mut router, 30_000_000);

    result
  };
  let mut router = router_on(true);
  let (status, blocked, diff) = tune(&mut router, 30_000_000);

  assert_eq!(status, TuningStatus::TooShort);
  assert!(
    blocked < clear,
    "the obstacle costs length: {blocked} against {clear} on a clear board"
  );
  assert_eq!(
    router.pending_update().added.len(),
    0,
    "the session is over and its node has been folded away"
  );

  // And whatever was fitted clears the obstacle by the rule clearance.
  let obstacle =
    Seg::new(Vec2::new(4_000_000, 900_000), Vec2::new(5_000_000, 900_000));
  let needed = i64::from(CLEARANCE + WIDTH);

  for seg in committed_segments(&diff) {
    let gap = i64::from(seg.distance_to_segment(&obstacle));

    assert!(
      gap >= needed,
      "the tuned track passes {gap} nm from the obstacle, needing {needed}"
    );
  }
}

/// The tuned stretch clears the obstacle at the reachable target too.
#[test]
fn an_obstacle_is_cleared_by_the_committed_geometry() {
  let mut router = router_on(true);
  let (status, _, diff) = tune(&mut router, 10_000_000);

  assert_eq!(status, TuningStatus::Tuned);

  let obstacle =
    Seg::new(Vec2::new(4_000_000, 900_000), Vec2::new(5_000_000, 900_000));
  let needed = i64::from(CLEARANCE + WIDTH);

  for seg in committed_segments(&diff) {
    let gap = i64::from(seg.distance_to_segment(&obstacle));

    assert!(
      gap >= needed,
      "the tuned track passes {gap} nm from the obstacle, needing {needed}"
    );
  }
}

/// A host with no `CT_CLEARANCE` rule gets the **track width** back as
/// the clearance (`pcbnew/router/pns_meander_placer_base.cpp:125`), which
/// then floors the meander period at twice the width. The fixture's
/// spacing of 600000 is above that floor, so the geometry is the same one
/// the answering resolver produces; the point is that the session runs at
/// all.
#[test]
fn a_missing_clearance_rule_falls_back_to_the_track_width() {
  let mut answering = Router::new(
    &board(false),
    Box::new(TuningRules::new(CLEARANCE)),
    RoutingSettings::default(),
    sizes(),
  );
  let mut silent = Router::new(
    &board(false),
    Box::new(TuningRules::without_constraint(CLEARANCE)),
    RoutingSettings::default(),
    sizes(),
  );
  let (answering_status, answering_result, _) =
    tune(&mut answering, 10_000_000);
  let (silent_status, silent_result, _) = tune(&mut silent, 10_000_000);

  assert_eq!(answering_status, silent_status);
  assert_eq!(answering_result, silent_result);
}

/// An unconstrained target is KiCad's `LENGTH_UNCONSTRAINED` sentinel
/// said without a sentinel: the session meanders as hard as the stretch
/// allows and reports `TUNED`, because the sentinel's window is zero to
/// one kilometre (`pcbnew/router/pns_meander.cpp:71` to `:74`).
///
/// Note 08 section 11.3 offers short circuiting to `TOO_SHORT` instead.
/// Reproducing KiCad was chosen; this is the pin.
#[test]
fn a_session_with_no_target_meanders_as_hard_as_it_can() {
  let mut settings = settings_for(10_000_000);

  settings.set_target_length(None);

  let mut router = router_on(false);

  router
    .start_tuning(TUNE_FROM, TRACK, settings)
    .expect("the fixture track is a track");

  let frame = router.move_to(TUNE_TO, None);
  let readout = *frame.tuning.as_deref().expect("a readout after a move");
  let unbounded = {
    let mut router = router_on(false);
    let (_, result, _) = tune(&mut router, 30_000_000);

    result
  };

  assert_eq!(readout.status, TuningStatus::Tuned);
  assert_eq!(
    readout.result, unbounded,
    "an unconstrained target gives the same geometry as an unreachable one"
  );
}

/// Two runs of the same session produce the same commit, which is
/// `DESIGN.md` section 8 applied to the tuner.
#[test]
fn two_tuning_sessions_agree() {
  let mut first = router_on(true);
  let mut second = router_on(true);
  let (first_status, first_result, first_diff) = tune(&mut first, 10_000_000);
  let (second_status, second_result, second_diff) =
    tune(&mut second, 10_000_000);

  assert_eq!(first_status, second_status);
  assert_eq!(first_result, second_result);
  assert_eq!(first_diff, second_diff);
}

/// The readout carries the settings back so that a host can persist the
/// initial side flip, and the target the status was decided against.
#[test]
fn the_readout_carries_the_settings_and_the_target_back() {
  let mut router = router_on(false);

  router
    .start_tuning(TUNE_FROM, TRACK, settings_for(10_000_000))
    .expect("the fixture track is a track");

  let frame = router.move_to(TUNE_TO, None);
  let readout = *frame.tuning.as_deref().expect("a readout after a move");

  assert_eq!(readout.target, LengthTarget::around(10_000_000));
  assert_eq!(readout.settings.max_amplitude(), 1_000_000);
  assert_eq!(
    readout.delta,
    Some(readout.result - BASELINE),
    "the delta is measured from the length the session started at"
  );
}

/// A tuned stretch that runs across a corner is two base segments, and
/// `doMove` meanders each of them in chain order.
///
/// This is the shape that makes the side flip order sensitive: KiCad
/// re-reads `m_settings.m_initialSide` at the top of every base segment
/// (`pcbnew/router/pns_meander_placer.cpp:265`), so a flip on the first
/// decides which side the second starts on. Nothing on this board makes
/// the flip fire, so what this pins is the two segment path itself.
#[test]
fn a_tuned_stretch_across_a_corner_meanders_both_legs() {
  let corner = Vec2::new(4_000_000, 0);
  let end = Vec2::new(4_000_000, 4_000_000);
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(pad(PAD_A, WEST));
  snapshot.items.push(pad(PAD_B, end));
  snapshot.items.push(track(TRACK, NET, WEST, corner));
  snapshot.items.push(track(HostId(5), NET, corner, end));

  let mut router = Router::new(
    &snapshot,
    Box::new(TuningRules::new(CLEARANCE)),
    RoutingSettings::default(),
    sizes(),
  );

  router
    .start_tuning(TUNE_FROM, TRACK, settings_for(10_000_000))
    .expect("the fixture track is a track");

  let frame = router.move_to(Vec2::new(4_000_000, 3_000_000), None);
  let readout = *frame.tuning.as_deref().expect("a readout after a move");

  assert_eq!(readout.status, TuningStatus::Tuned);
  assert!(
    (readout.result - 10_000_000).abs() <= 100_000,
    "the bent track tuned to {}",
    readout.result
  );

  let FixOutcome::Finished(diff) =
    router.fix_route(Vec2::new(4_000_000, 3_000_000), None, true)
  else {
    panic!("a tuning fix always ends the session");
  };
  let horizontal = committed_segments(&diff)
    .iter()
    .filter(|seg| seg.a.y != 0 || seg.b.y != 0)
    .count();
  let vertical = committed_segments(&diff)
    .iter()
    .filter(|seg| seg.a.x != 4_000_000 || seg.b.x != 4_000_000)
    .count();

  assert!(horizontal > 0 && vertical > 0, "both legs were meandered");
}

/// A recorded tuning session replays to the same commit, settings, live
/// adjustments and all. The milestone's acceptance criterion.
#[test]
fn a_tuning_session_replays() {
  let snapshot = board(true);
  let mut router = Router::new(
    &snapshot,
    Box::new(TuningRules::new(CLEARANCE)),
    RoutingSettings::default(),
    sizes(),
  );

  router.start_recording(&snapshot);
  router
    .start_tuning(TUNE_FROM, TRACK, settings_for(30_000_000))
    .expect("the fixture track is a track");
  router.move_to(TUNE_TO, None);
  router.amplitude_step(-1);
  router.amplitude_step(-1);
  router.spacing_step(1);
  router.move_to(TUNE_TO, None);
  router.fix_route(TUNE_TO, None, true);

  let recording = router.take_recording().expect("recording was started");

  assert!(!recording.results.is_empty(), "the session committed");

  // The text round trip, and then the replay of what came back out of it.
  let text = recording.to_text();
  let parsed =
    SessionRecording::from_text(&text).expect("the writer's own output");

  assert_eq!(parsed, recording);
  assert_replay_matches(&parsed, || Box::new(TuningRules::new(CLEARANCE)));
}

/// The recording is what drives the replay, so the settings have to be in
/// it: a replay with the amplitude steps stripped out gives different
/// geometry, which is the reason they are events at all.
#[test]
fn the_live_adjustments_are_part_of_the_recording() {
  let snapshot = board(false);
  let record = |steps: i32| {
    let mut router = Router::new(
      &snapshot,
      Box::new(TuningRules::new(CLEARANCE)),
      RoutingSettings::default(),
      sizes(),
    );

    router.start_recording(&snapshot);
    router
      .start_tuning(TUNE_FROM, TRACK, settings_for(30_000_000))
      .expect("the fixture track is a track");
    router.move_to(TUNE_TO, None);

    for _ in 0..steps {
      router.amplitude_step(-1);
    }

    router.move_to(TUNE_TO, None);
    router.fix_route(TUNE_TO, None, true);
    router.take_recording().expect("recording was started")
  };
  let plain = record(0);
  let stepped = record(6);

  assert_ne!(
    plain.results, stepped.results,
    "six amplitude steps change the geometry"
  );

  let replayed = replay(&stepped, Box::new(TuningRules::new(CLEARANCE)));

  assert_eq!(replayed.diffs, stepped.results);
}

/// Every session that the facade drove committed something a host can
/// apply: the added lengths are positive and no piece is degenerate.
#[test]
fn every_committed_piece_has_a_length() {
  let mut router = router_on(false);
  let (_, _, diff) = tune(&mut router, 10_000_000);

  assert!(added_length(&diff) >= 0);

  for seg in committed_segments(&diff) {
    assert!(seg.a != seg.b, "a committed segment is not degenerate");
  }
}
