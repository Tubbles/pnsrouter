// SPDX-License-Identifier: GPL-3.0-or-later

//! Differential pair skew tuning scenarios, through the facade only.
//!
//! The board is the synthetic fixture of
//! `doc/reference/kicad/08-meanders.md` section 14.4: two lanes that are
//! coupled from `x = 2 mm` onwards and unequal overall, because the N
//! lane takes a rectangular detour near the start that adds exactly two
//! millimetres outside the tuned range. P is 8 mm, N is 10 mm.
//!
//! # The placer does not pick a lane
//!
//! It meanders whichever lane the user clicked and only measures the
//! other one (note 08 section 7.1). So the fixture has to click the
//! **shorter** lane to get anywhere, and clicking the longer one is a
//! case of its own: it reports "too long" and changes nothing. There is
//! no automatic choice anywhere in KiCad's class and none here.
//!
//! # Why the fixture is synthetic
//!
//! There is no tuning case in KiCad's regression corpus and its log
//! format cannot express one (note 08 section 10.5), so there is nothing
//! upstream to replay a skew session against.

#![forbid(unsafe_code)]

use pnsrouter::eventlog::{SessionRecording, assert_replay_matches};
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{HostId, LayerRange, NetId};
use pnsrouter::meander::{
  LengthTarget, MeanderSettings, MeanderSettingsRequest, MeanderSide,
  MeanderStyle, TuningStatus,
};
use pnsrouter::node::World;
use pnsrouter::placer::TuningMode;
use pnsrouter::router::{
  CommitDiff, FixOutcome, NewGeometry, Router, StartError,
};
use pnsrouter::rules::{
  Constraint, ConstraintType, CoupledNets, FixedClearance, ItemRef, Keepout,
  RuleResolver,
};
use pnsrouter::settings::{RoutingSettings, Sizes};
use pnsrouter::snapshot::{WorldGeometry, WorldItem, WorldSnapshot};

/// The clearance every scenario tunes to, in nanometres.
const CLEARANCE: i32 = 100_000;

/// The width of one lane.
const WIDTH: i32 = 200_000;

/// The copper gap between the two lanes, edge to edge.
const GAP: i32 = 200_000;

/// The centre to centre spacing of the two lanes.
const PITCH: i32 = WIDTH + GAP;

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 150_000;

/// The positive half of the pair, the **shorter** lane.
const NET_P: NetId = NetId(1);

/// The negative half, the longer one.
const NET_N: NetId = NetId(2);

/// A net the resolver knows nothing about.
const LONE_NET: NetId = NetId(3);

/// The pad the P lane starts on.
const PAD_A_P: HostId = HostId(1);

/// The pad the N lane starts on.
const PAD_A_N: HostId = HostId(2);

/// The pad the P lane ends on.
const PAD_B_P: HostId = HostId(3);

/// The pad the N lane ends on.
const PAD_B_N: HostId = HostId(4);

/// The P lane, one straight segment.
const TRACK_P: HostId = HostId(5);

/// The last segment of the N lane, the one that runs beside P.
const TRACK_N: HostId = HostId(9);

/// A track on a net with no coupled partner.
const LONE_TRACK: HostId = HostId(10);

/// Where the P lane runs.
const P_Y: i32 = -PITCH / 2;

/// Where the N lane runs, once it comes back from its detour.
const N_Y: i32 = PITCH / 2;

/// Where the lanes end.
const EAST: i32 = 8_000_000;

/// Where the detour rejoins the N lane.
const DETOUR_END: i32 = 2_000_000;

/// How far the detour reaches.
const DETOUR_Y: i32 = 1_200_000;

/// Where the tuned stretch begins, in x. Past the detour, so that the two
/// lanes are coupled where the user clicks.
const TUNE_FROM_X: i32 = 3_000_000;

/// Where the tuned stretch ends, in x. Four millimetres of baseline.
const TUNE_TO_X: i32 = 7_000_000;

/// The length of the P lane, the shorter one.
const LENGTH_P: i64 = 8_000_000;

/// The length of the N lane, one millimetre up, two across and one back
/// down on top of the six that remain.
const LENGTH_N: i64 = 10_000_000;

/// A resolver that says which two nets are coupled and answers the
/// clearance constraint.
///
/// The same one `tests/dp_meander_placer.rs` uses, for the same reason: a
/// tuning session is the one thing in the crate that asks for a
/// `CT_CLEARANCE` constraint
/// (`pcbnew/router/pns_meander_placer_base.cpp:122`).
struct PairTuningRules {
  /// The pair hooks, and every rule that is not about pairs.
  inner: CoupledNets,
}

impl PairTuningRules {
  /// A resolver over one clearance.
  fn new(clearance: i32) -> Self {
    Self {
      inner: CoupledNets::new(FixedClearance::uniform(clearance), NET_P, NET_N),
    }
  }
}

impl RuleResolver for PairTuningRules {
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
        min: Some(CLEARANCE),
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

  fn dp_coupled_net(&self, net: NetId) -> Option<NetId> {
    self.inner.dp_coupled_net(net)
  }

  fn dp_net_polarity(&self, net: NetId) -> i32 {
    self.inner.dp_net_polarity(net)
  }

  fn dp_net_pair(&self, item: ItemRef<'_>) -> Option<(NetId, NetId)> {
    self.inner.dp_net_pair(item)
  }
}

/// A round pad of one of the nets.
fn pad(id: HostId, at: Vec2, net: NetId) -> WorldItem {
  WorldItem::new(
    id,
    Some(net),
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

/// The unequal lane board of note 08 section 14.4.
fn board() -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(pad(PAD_A_P, Vec2::new(0, P_Y), NET_P));
  snapshot.items.push(pad(PAD_A_N, Vec2::new(0, N_Y), NET_N));
  snapshot
    .items
    .push(pad(PAD_B_P, Vec2::new(EAST, P_Y), NET_P));
  snapshot
    .items
    .push(pad(PAD_B_N, Vec2::new(EAST, N_Y), NET_N));
  snapshot.items.push(track(
    TRACK_P,
    NET_P,
    Vec2::new(0, P_Y),
    Vec2::new(EAST, P_Y),
  ));
  // The detour, which adds exactly two millimetres outside the tuned
  // range: one up, two across and one back down.
  snapshot.items.push(track(
    HostId(6),
    NET_N,
    Vec2::new(0, N_Y),
    Vec2::new(0, DETOUR_Y),
  ));
  snapshot.items.push(track(
    HostId(7),
    NET_N,
    Vec2::new(0, DETOUR_Y),
    Vec2::new(DETOUR_END, DETOUR_Y),
  ));
  snapshot.items.push(track(
    HostId(8),
    NET_N,
    Vec2::new(DETOUR_END, DETOUR_Y),
    Vec2::new(DETOUR_END, N_Y),
  ));
  snapshot.items.push(track(
    TRACK_N,
    NET_N,
    Vec2::new(DETOUR_END, N_Y),
    Vec2::new(EAST, N_Y),
  ));

  snapshot
}

/// A board with one track on a net the resolver knows nothing about.
fn lone_board() -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(track(
    LONE_TRACK,
    LONE_NET,
    Vec2::new(0, 4_000_000),
    Vec2::new(EAST, 4_000_000),
  ));

  snapshot
}

/// The sizes every scenario runs with.
fn sizes() -> Sizes {
  Sizes {
    track_width: WIDTH,
    board_min_track_width: 100_000,
    min_clearance: CLEARANCE,
    diff_pair_width: WIDTH,
    diff_pair_gap: GAP,
    ..Sizes::default()
  }
}

/// The meander settings of note 08 section 14.1, aimed at a skew.
fn settings_for(skew: i64) -> MeanderSettings {
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
    // A skew session never reads it: the target `doMove` gets is the
    // coupled lane's length plus the skew window
    // (`pcbnew/router/pns_meander_skew_placer.cpp:234`).
    target_length: None,
    target_skew: Some(LengthTarget::around(skew)),
  })
  .expect("the note's settings are a step of 50000 and chamfered corners")
}

/// A router over the unequal lane board.
fn router() -> Router {
  Router::new(
    &board(),
    Box::new(PairTuningRules::new(CLEARANCE)),
    RoutingSettings::default(),
    sizes(),
  )
}

/// Every segment a commit added, whether by addition or by update.
fn committed_segments(diff: &CommitDiff) -> Vec<(Option<NetId>, Seg)> {
  diff
    .added
    .iter()
    .chain(diff.updated.iter().map(|(_, item)| item))
    .filter_map(|item| match item.geometry {
      NewGeometry::Segment { seg, .. } => Some((item.net, seg)),
      NewGeometry::Via { .. } => None,
    })
    .collect()
}

/// The total copper length one net got out of a commit.
fn committed_length(diff: &CommitDiff, net: NetId) -> i64 {
  committed_segments(diff)
    .iter()
    .filter(|(item_net, _)| *item_net == Some(net))
    .map(|(_, seg)| i64::from(seg.length()))
    .sum()
}

/// Click the P lane and tune it to a skew.
fn tune_p(router: &mut Router, skew: i64) -> (TuningStatus, i64, CommitDiff) {
  let frame = router
    .start_tuning_skew(Vec2::new(TUNE_FROM_X, P_Y), TRACK_P, settings_for(skew))
    .expect("the P lane is half of a differential pair");
  let readout = *frame
    .tuning
    .as_deref()
    .expect("a tuning session reports a readout");

  assert_eq!(readout.mode, TuningMode::PairSkew);
  assert_eq!(
    readout.result,
    LENGTH_P - LENGTH_N,
    "the session starts two millimetres short on the P lane"
  );

  let frame = router.move_to(Vec2::new(TUNE_TO_X, P_Y), None);
  let readout = *frame.tuning.as_deref().expect("a readout after a move");

  let FixOutcome::Finished(diff) =
    router.fix_route(Vec2::new(TUNE_TO_X, P_Y), None, true)
  else {
    panic!("a tuning fix always ends the session");
  };

  (readout.status, readout.result, diff)
}

/// Click the shorter lane and ask for zero skew: the meanders make up the
/// two millimetres and the two lanes come out the same length.
///
/// The fixture case `the_skew_placer_lengthens_the_shorter_lane` of note
/// 08 section 14.6.
#[test]
fn the_skew_placer_lengthens_the_shorter_lane() {
  let mut router = router();
  let (status, skew, diff) = tune_p(&mut router, 0);

  assert_eq!(status, TuningStatus::Tuned);
  assert!(
    skew.abs() <= 100_000,
    "the skew {skew} is outside KiCad's own tolerance around zero"
  );

  // The P lane is the one that grew; the N lane was never removed from
  // the branch and never touched
  // (`pcbnew/router/pns_meander_skew_placer.cpp:128`).
  let length_p = committed_length(&diff, NET_P);

  assert!(
    (length_p - LENGTH_N).abs() <= 100_000,
    "the tuned P lane is {length_p} where the N lane is {LENGTH_N}"
  );
  assert_eq!(
    committed_length(&diff, NET_N),
    0,
    "the coupled lane is only measured, never meandered"
  );
  assert!(
    diff.removed.iter().all(|host| *host == TRACK_P)
      && diff.updated.iter().all(|(host, _)| *host == TRACK_P),
    "only the clicked lane leaves and returns"
  );
}

/// Click the **longer** lane and ask for zero skew: the target lands
/// below the current length, `doMove`'s early test fires and nothing is
/// meandered.
///
/// The fixture case `the_skew_placer_reports_too_long_on_the_longer_lane`,
/// and note 08 section 7.1's "it does not pick a lane" as a test.
#[test]
fn the_skew_placer_reports_too_long_on_the_longer_lane() {
  let mut router = router();
  let frame = router
    .start_tuning_skew(Vec2::new(TUNE_FROM_X, N_Y), TRACK_N, settings_for(0))
    .expect("the N lane is half of a differential pair");
  let readout = *frame.tuning.as_deref().expect("a readout");

  assert_eq!(
    readout.result,
    LENGTH_N - LENGTH_P,
    "the active lane is the longer one, so the skew starts positive"
  );

  let frame = router.move_to(Vec2::new(TUNE_TO_X, N_Y), None);
  let readout = *frame.tuning.as_deref().expect("a readout after a move");

  assert_eq!(readout.status, TuningStatus::TooLong);
  assert_eq!(
    readout.result,
    LENGTH_N - LENGTH_P,
    "nothing was meandered, so the skew is the one it started at"
  );
}

/// A skew target other than zero: the active lane is driven to the
/// coupled lane's length plus the target.
#[test]
fn a_skew_target_other_than_zero_is_reached() {
  let mut router = router();
  let (status, skew, diff) = tune_p(&mut router, 1_000_000);

  assert_eq!(status, TuningStatus::Tuned);
  assert!(
    (skew - 1_000_000).abs() <= 100_000,
    "the skew {skew} is outside the tolerance around one millimetre"
  );
  assert!(
    (committed_length(&diff, NET_P) - (LENGTH_N + 1_000_000)).abs() <= 100_000,
    "the P lane overshoots the N lane by the requested skew"
  );
}

/// A negative skew target asks the active lane to stay shorter, which on
/// this board it already is by two millimetres, so the run is short of
/// the target rather than past it.
#[test]
fn a_negative_skew_target_is_still_measured_against_the_coupled_lane() {
  let mut router = router();
  let (status, skew, _) = tune_p(&mut router, -1_000_000);

  assert_eq!(status, TuningStatus::Tuned);
  assert!(
    (skew + 1_000_000).abs() <= 100_000,
    "the skew {skew} is outside the tolerance around minus one millimetre"
  );
}

/// A start on a track whose net has no coupled partner is refused with
/// the skew tuner's own message, which is not the length tuner's.
#[test]
fn a_start_on_an_uncoupled_net_is_refused() {
  let mut router = Router::new(
    &lone_board(),
    Box::new(PairTuningRules::new(CLEARANCE)),
    RoutingSettings::default(),
    sizes(),
  );
  let error = router
    .start_tuning_skew(
      Vec2::new(1_000_000, 4_000_000),
      LONE_TRACK,
      settings_for(0),
    )
    .expect_err("the lone track has no coupled net");

  assert_eq!(error, StartError::NotADiffPairForSkew);
}

/// A start on a pad and a start on an unknown object are both refused.
#[test]
fn a_start_on_a_pad_is_refused() {
  let mut router = router();
  let error = router
    .start_tuning_skew(Vec2::new(0, P_Y), PAD_A_P, settings_for(0))
    .expect_err("a pad is not a track");

  assert!(matches!(error, StartError::NotATrack(_)));

  let error = router
    .start_tuning_skew(Vec2::new(TUNE_FROM_X, P_Y), HostId(99), settings_for(0))
    .expect_err("no such object");

  assert_eq!(error, StartError::UnknownStartItem(HostId(99)));
}

/// The readout names the skew mode, carries the skew under its own name
/// and reports the length it was measured against.
#[test]
fn the_readout_names_the_skew_mode() {
  let mut router = router();

  router
    .start_tuning_skew(Vec2::new(TUNE_FROM_X, P_Y), TRACK_P, settings_for(0))
    .expect("the P lane is half of a differential pair");

  let frame = router.move_to(Vec2::new(TUNE_TO_X, P_Y), None);
  let readout = *frame.tuning.as_deref().expect("a readout after a move");

  assert_eq!(readout.mode, TuningMode::PairSkew);
  assert_eq!(
    readout.skew,
    Some(readout.result),
    "`TuningLengthResult` is the skew in this mode"
  );
  assert_eq!(readout.skew_target, Some(LengthTarget::around(0)));
  assert_eq!(readout.coupled_length, Some(LENGTH_N));
  // The window `doMove` compared against is the coupled lane's length
  // plus the skew window, which is neither setting on its own.
  assert_eq!(
    readout.target,
    LengthTarget::explicit(LENGTH_N - 100_000, LENGTH_N, LENGTH_N + 100_000)
  );
}

/// The two live adjustments reach a skew session.
#[test]
fn the_amplitude_and_spacing_steps_reach_a_skew_session() {
  let mut router = router();

  router
    .start_tuning_skew(Vec2::new(TUNE_FROM_X, P_Y), TRACK_P, settings_for(0))
    .expect("the P lane is half of a differential pair");

  assert!(router.amplitude_step(1));
  assert!(router.spacing_step(1));

  let frame = router.move_to(Vec2::new(TUNE_TO_X, P_Y), None);
  let readout = *frame.tuning.as_deref().expect("a readout after a move");

  assert_eq!(readout.settings.max_amplitude(), 1_050_000);
  assert_eq!(readout.settings.spacing(), 650_000);
}

/// Two runs of the same session produce the same commit, which is
/// `DESIGN.md` section 8 applied to the skew tuner.
#[test]
fn two_skew_sessions_agree() {
  let mut first = router();
  let mut second = router();
  let (first_status, first_skew, first_diff) = tune_p(&mut first, 0);
  let (second_status, second_skew, second_diff) = tune_p(&mut second, 0);

  assert_eq!(first_status, second_status);
  assert_eq!(first_skew, second_skew);
  assert_eq!(first_diff, second_diff);
}

/// A recorded skew session replays to the same commit, settings, live
/// adjustments and all.
#[test]
fn a_skew_session_replays() {
  let snapshot = board();
  let mut router = Router::new(
    &snapshot,
    Box::new(PairTuningRules::new(CLEARANCE)),
    RoutingSettings::default(),
    sizes(),
  );

  router.start_recording(&snapshot);
  router
    .start_tuning_skew(Vec2::new(TUNE_FROM_X, P_Y), TRACK_P, settings_for(0))
    .expect("the P lane is half of a differential pair");
  router.move_to(Vec2::new(TUNE_TO_X, P_Y), None);
  router.amplitude_step(-1);
  router.spacing_step(1);
  router.move_to(Vec2::new(TUNE_TO_X, P_Y), None);
  router.fix_route(Vec2::new(TUNE_TO_X, P_Y), None, true);

  let recording = router.take_recording().expect("recording was started");

  assert!(!recording.results.is_empty(), "the session committed");

  let text = recording.to_text();

  assert!(
    text.contains("event start-tuning-skew"),
    "the skew start has an event of its own"
  );

  let parsed =
    SessionRecording::from_text(&text).expect("the writer's own output");

  assert_eq!(parsed, recording);
  assert_replay_matches(&parsed, || Box::new(PairTuningRules::new(CLEARANCE)));
}
