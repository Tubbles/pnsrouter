// SPDX-License-Identifier: GPL-3.0-or-later

//! Differential pair placement scenarios, through the facade only.
//!
//! The board is the synthetic fixture of
//! `doc/reference/kicad/07-differential-pairs.md` section 15: two nets,
//! two pairs of round pads a pitch apart, and an obstacle. KiCad's
//! regression corpus has no differential pair case at all, and its log
//! format cannot express one (note 07 section 15 and note 05 section
//! 6.2), so there is nothing to replay a pair session against; the
//! fidelity of the port rests on the gateway and candidate assertions of
//! `tests/diff_pair.rs` and on the behaviour pinned here.
//!
//! Two obstacles rather than the note's one. The crossing track is what
//! walkaround mode has to get around and what mark obstacles mode reports
//! a collision on; the parallel track beside the P lane is what shove
//! mode pushes, because a track that crosses the route at a right angle
//! has nowhere to be pushed to.
//!
//! Coupling is asserted through [`DiffPair::coupled_length`] and
//! [`DiffPair::skew`] over the committed geometry rather than through raw
//! coordinates, as note 07 section 15.3 asks: a coordinate assertion on a
//! 45 degree router breaks on every gateway ordering change, where "the
//! two lanes are coupled over at least six millimetres" survives.

#![forbid(unsafe_code)]

use pnsrouter::diff_pair::DiffPair;
use pnsrouter::geometry::arc::ShapeArc;
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{HostId, LayerRange, NetId};
use pnsrouter::node::World;
use pnsrouter::placer::diff_pair_placer::DiffPairPlacer;
use pnsrouter::router::{
  CommitDiff, FixOutcome, NewGeometry, Router, StartError,
};
use pnsrouter::rules::{CoupledNets, FixedClearance};
use pnsrouter::settings::{RouterMode, RoutingSettings, Sizes};
use pnsrouter::snapshot::{WorldGeometry, WorldItem, WorldSnapshot};

/// The clearance every scenario routes to, in nanometres.
///
/// Below [`GAP`], because the two lanes are two nets and collide
/// normally: nothing in the router exempts P from N (note 07 section
/// 8.3).
const CLEARANCE: i32 = 100_000;

/// The width of one lane.
const WIDTH: i32 = 200_000;

/// The copper gap between the two lanes, edge to edge.
const GAP: i32 = 200_000;

/// The centre to centre spacing of the two lanes, `Sizes::diff_pair_pitch`.
const PITCH: i32 = WIDTH + GAP;

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 150_000;

/// The copper diameter of a via.
const VIA_DIAMETER: i32 = 600_000;

/// The drill of a via.
const VIA_DRILL: i32 = 300_000;

/// The positive half of the pair.
const NET_P: NetId = NetId(1);

/// The negative half.
const NET_N: NetId = NetId(2);

/// The net of both obstacles, so that neither is ever exempt.
const OBSTACLE_NET: NetId = NetId(3);

/// The pad the P lane starts on.
const START_P: HostId = HostId(1);

/// The pad the N lane starts on.
const START_N: HostId = HostId(2);

/// The pad the P lane ends on.
const TARGET_P: HostId = HostId(3);

/// The pad the N lane ends on.
const TARGET_N: HostId = HostId(4);

/// The obstacle, whichever kind the board was built with.
const OBSTACLE: HostId = HostId(5);

/// A pad on the P net on the second layer, whose coupled net has nothing
/// on that layer.
const LONE_P: HostId = HostId(6);

/// Where the P lane starts.
const START_P_AT: Vec2 = Vec2::new(0, -PITCH / 2);

/// Where the N lane starts.
const START_N_AT: Vec2 = Vec2::new(0, PITCH / 2);

/// Where the P lane ends.
const TARGET_P_AT: Vec2 = Vec2::new(8_000_000, -PITCH / 2);

/// Where the N lane ends.
const TARGET_N_AT: Vec2 = Vec2::new(8_000_000, PITCH / 2);

/// Where the lone second layer pad sits.
const LONE_P_AT: Vec2 = Vec2::new(0, -4_000_000);

/// Which obstacle a board carries.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum Obstacle {
  /// None: the pair runs straight from one pad pair to the other.
  None,
  /// A track crossing the route, which the pair has to get around.
  Crossing,
  /// A track running alongside the P lane, which the pair can push.
  Parallel,
}

/// A round pad on one copper layer.
fn pad(id: HostId, at: Vec2, layer: i32, net: NetId) -> WorldItem {
  WorldItem::new(
    id,
    Some(net),
    LayerRange::single(layer),
    WorldGeometry::Solid {
      shape: Shape::circle(at, PAD_RADIUS),
      pos: at,
      offset: Vec2::new(0, 0),
      orientation_degrees: 0.0,
      anchors: Vec::new(),
    },
  )
}

/// A track of the obstacle net.
fn track(id: HostId, from: Vec2, to: Vec2) -> WorldItem {
  WorldItem::new(
    id,
    Some(OBSTACLE_NET),
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(from, to),
      width: WIDTH,
    },
  )
}

/// The fixture board, as a host would hand it over.
///
/// The two pad pairs are exactly [`PITCH`] apart, which is what
/// `ROUTER::isStartingPointRoutable`'s gap tolerance check would measure
/// if it applied (it only fires for a track start,
/// `pcbnew/router/pns_router.cpp:363`) and what
/// `check_diagonal_alignment` needs to pass on the vertical.
fn board(obstacle: Obstacle) -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(pad(START_P, START_P_AT, 0, NET_P));
  snapshot.items.push(pad(START_N, START_N_AT, 0, NET_N));
  snapshot.items.push(pad(TARGET_P, TARGET_P_AT, 0, NET_P));
  snapshot.items.push(pad(TARGET_N, TARGET_N_AT, 0, NET_N));
  snapshot.items.push(pad(LONE_P, LONE_P_AT, 1, NET_P));

  match obstacle {
    Obstacle::None => {}
    Obstacle::Crossing => snapshot.items.push(track(
      OBSTACLE,
      Vec2::new(4_000_000, -3_000_000),
      Vec2::new(4_000_000, 3_000_000),
    )),
    Obstacle::Parallel => snapshot.items.push(track(
      OBSTACLE,
      Vec2::new(2_000_000, -450_000),
      Vec2::new(6_000_000, -450_000),
    )),
  }

  snapshot
}

/// The sizes every scenario places with.
fn sizes() -> Sizes {
  let mut sizes = Sizes {
    track_width: WIDTH,
    board_min_track_width: 100_000,
    min_clearance: CLEARANCE,
    diff_pair_width: WIDTH,
    diff_pair_gap: GAP,
    diff_pair_via_gap: GAP,
    diff_pair_via_gap_same_as_trace_gap: false,
    via_diameter: VIA_DIAMETER,
    via_drill: VIA_DRILL,
    ..Sizes::default()
  };

  sizes.add_layer_pair(0, 1);
  sizes
}

/// The rule oracle every scenario uses: one clearance, one pair.
fn rules() -> CoupledNets {
  CoupledNets::new(FixedClearance::uniform(CLEARANCE), NET_P, NET_N)
}

/// A router over one of the boards, in one routing mode.
fn router_in(mode: RouterMode, obstacle: Obstacle) -> Router {
  let settings = RoutingSettings {
    mode,
    ..RoutingSettings::default()
  };

  Router::new(&board(obstacle), Box::new(rules()), settings, sizes())
}

/// Every segment a commit created on one net, as a chain from a point.
///
/// The commit lists segments in the order they were stored, which is not
/// the order they run in, so this walks them end to end from the lane's
/// start pad. A segment that does not join the chain is left out, which
/// no scenario here produces and which an assertion on the point count
/// would catch.
fn lane(diff: &CommitDiff, net: NetId, start: Vec2) -> LineChain {
  let mut segments: Vec<Seg> = diff
    .added
    .iter()
    .filter(|item| item.net == Some(net))
    .filter_map(|item| match item.geometry {
      NewGeometry::Segment { seg, .. } => Some(seg),
      NewGeometry::Arc { .. } | NewGeometry::Via { .. } => None,
    })
    .collect();
  let mut chain = LineChain::new();

  chain.append(start);

  while let Some(position) = segments
    .iter()
    .position(|seg| seg.a == chain.last_point().unwrap_or(start))
    .or_else(|| {
      segments
        .iter()
        .position(|seg| seg.b == chain.last_point().unwrap_or(start))
    })
  {
    let seg = segments.remove(position);
    let next = if seg.a == chain.last_point().unwrap_or(start) {
      seg.b
    } else {
      seg.a
    };

    chain.append(next);
  }

  chain
}

/// The committed pair, measured the way the engine measures one.
fn committed_pair(diff: &CommitDiff) -> DiffPair {
  let mut pair = DiffPair::from_chains(
    lane(diff, NET_P, START_P_AT),
    lane(diff, NET_N, START_N_AT),
    0,
  );

  pair.set_width(WIDTH);
  pair.set_gap(GAP);
  pair
}

/// How many segments a diff creates on one net.
fn added_segments(diff: &CommitDiff, net: NetId) -> usize {
  diff
    .added
    .iter()
    .filter(|item| item.net == Some(net))
    .filter(|item| matches!(item.geometry, NewGeometry::Segment { .. }))
    .count()
}

/// Every via a diff creates, with the net it carries.
fn added_vias(diff: &CommitDiff) -> Vec<(Option<NetId>, Vec2, i32)> {
  diff
    .added
    .iter()
    .filter_map(|item| match item.geometry {
      NewGeometry::Via { pos, diameter, .. } => Some((item.net, pos, diameter)),
      NewGeometry::Arc { .. } | NewGeometry::Segment { .. } => None,
    })
    .collect()
}

/// Route from the start pad pair to the target pad pair and commit.
///
/// The gesture a host performs: press on one half of the start pair, one
/// move onto the target pair, click there. The cursor lands on the P pad,
/// which is enough for the placer to snap the whole pair to it
/// (`pcbnew/router/pns_diff_pair_placer.cpp:691`).
fn route_across(router: &mut Router) -> CommitDiff {
  router
    .start_routing_diff_pair(START_P_AT, Some(START_P), 0)
    .expect("the start pad pair is routable");
  router.move_to(TARGET_P_AT, Some(TARGET_P));

  match router.fix_route(TARGET_P_AT, Some(TARGET_P), false) {
    FixOutcome::Finished(diff) => diff,
    FixOutcome::Continue(_) => panic!("a fix on the target pair finishes"),
  }
}

#[test]
fn a_pair_routes_straight_between_two_pad_pairs() {
  let mut router = router_in(RouterMode::MarkObstacles, Obstacle::None);
  let diff = route_across(&mut router);
  let pair = committed_pair(&diff);

  assert!(
    added_segments(&diff, NET_P) >= 1 && added_segments(&diff, NET_N) >= 1,
    "both lanes reach the board: {diff:?}"
  );

  // The lanes run the whole eight millimetres side by side, so almost all
  // of that length is coupled and neither lane is longer than the other.
  assert!(
    pair.coupled_length() >= 7_000_000,
    "coupled over {} nm",
    pair.coupled_length()
  );
  assert!(pair.skew().abs() < 100_000, "skew {} nm", pair.skew());

  // The gap is the configured one: every P point sits half a pitch below
  // the centre line and every N point half a pitch above it.
  for index in 0..pair.chain_p().point_count() {
    assert_eq!(pair.chain_p().point(index).y, -PITCH / 2);
  }

  for index in 0..pair.chain_n().point_count() {
    assert_eq!(pair.chain_n().point(index).y, PITCH / 2);
  }
}

#[test]
fn a_pair_walks_around_an_obstacle_and_stays_coupled() {
  let mut router = router_in(RouterMode::Walkaround, Obstacle::Crossing);
  let diff = route_across(&mut router);
  let pair = committed_pair(&diff);

  assert!(
    added_segments(&diff, NET_P) >= 2 && added_segments(&diff, NET_N) >= 2,
    "the detour needs corners on both lanes: {diff:?}"
  );

  // The obstacle is not moved: walkaround mode goes round it.
  assert!(diff.updated.is_empty(), "{diff:?}");
  assert!(diff.removed.is_empty(), "{diff:?}");

  // The straight runs before and after the obstacle are still coupled.
  assert!(
    pair.coupled_length() >= 4_000_000,
    "coupled over {} nm",
    pair.coupled_length()
  );
}

#[test]
fn a_pair_shoves_a_track_out_of_its_way() {
  let mut router = router_in(RouterMode::Shove, Obstacle::Parallel);
  let diff = route_across(&mut router);
  let pair = committed_pair(&diff);

  // The obstacle keeps its host object and comes back moved, which is
  // what the remove plus add fold of `CommitRouting` reports as an
  // update.
  assert!(
    diff.updated.iter().any(|(host, _)| *host == OBSTACLE),
    "the parallel track was pushed: {diff:?}"
  );

  assert!(
    pair.coupled_length() >= 6_000_000,
    "coupled over {} nm",
    pair.coupled_length()
  );
}

#[test]
fn mark_obstacles_mode_refuses_to_commit_a_colliding_pair() {
  let mut router = router_in(RouterMode::MarkObstacles, Obstacle::Crossing);

  router
    .start_routing_diff_pair(START_P_AT, Some(START_P), 0)
    .expect("the start pad pair is routable");

  let frame = router.move_to(TARGET_P_AT, Some(TARGET_P));

  assert!(
    !frame.violations.is_empty(),
    "the crossing track is reported: {frame:?}"
  );

  // `rhMarkObstacles` leaves `m_fitOk` false, which is the gate at
  // `pcbnew/router/pns_diff_pair_placer.cpp:810`.
  assert!(matches!(
    router.fix_route(TARGET_P_AT, Some(TARGET_P), false),
    FixOutcome::Continue(_)
  ));
}

#[test]
fn a_pair_places_two_vias_and_carries_on_below() {
  let mut router = router_in(RouterMode::Walkaround, Obstacle::None);

  router
    .start_routing_diff_pair(START_P_AT, Some(START_P), 0)
    .expect("the start pad pair is routable");

  let half_way = Vec2::new(4_000_000, 0);

  router.move_to(half_way, None);

  assert!(router.toggle_via_placement(), "the via is armed");
  assert!(router.placing_via());

  let frame = router.move_to(half_way, None);

  assert!(frame.via.is_some() && frame.via_n.is_some(), "{frame:?}");

  // The fix writes both vias and starts a new leg, so the layer may
  // change (`pcbnew/router/pns_diff_pair_placer.cpp:840`, `:448`).
  assert!(matches!(
    router.fix_route(half_way, None, false),
    FixOutcome::Continue(_)
  ));
  assert!(router.switch_layer(1), "a via pair unpins the layer");

  router.move_to(TARGET_P_AT, None);

  let diff = match router.fix_route(TARGET_P_AT, None, true) {
    FixOutcome::Finished(diff) => diff,
    FixOutcome::Continue(_) => panic!("a forced fix finishes the route"),
  };
  let vias = added_vias(&diff);

  assert_eq!(vias.len(), 2, "one via per lane: {diff:?}");

  let (net_a, pos_a, diameter) = vias[0];
  let (net_b, pos_b, _) = vias[1];

  assert_ne!(net_a, net_b, "one via per net");

  // "A via gap apart" is copper edge to copper edge, which is what
  // `SIZES_SETTINGS::EffectiveDiffPairViaGap` measures.
  let centres = (pos_a - pos_b).euclidean_norm();

  assert!(
    centres - diameter >= sizes().effective_diff_pair_via_gap(),
    "vias {centres} nm apart, diameter {diameter}"
  );

  // Both layers carry copper afterwards.
  let layers: Vec<i32> =
    diff.added.iter().map(|item| item.layers.start()).collect();

  assert!(layers.contains(&0) && layers.contains(&1), "{layers:?}");
}

#[test]
fn a_pair_start_needs_a_start_item() {
  let mut router = router_in(RouterMode::Walkaround, Obstacle::None);

  assert_eq!(
    router.start_routing_diff_pair(START_P_AT, None, 0),
    Err(StartError::PairNeedsStartItem)
  );
}

#[test]
fn a_start_on_a_net_that_is_not_half_of_a_pair_is_refused() {
  let mut router = router_in(RouterMode::Walkaround, Obstacle::Crossing);

  assert_eq!(
    router.start_routing_diff_pair(Vec2::new(4_000_000, 0), Some(OBSTACLE), 0),
    Err(StartError::NotADiffPair)
  );
}

#[test]
fn a_start_whose_coupled_net_has_no_partner_is_refused() {
  let mut router = router_in(RouterMode::Walkaround, Obstacle::None);

  // Every object on the coupled net is on the other layer, and the layer
  // equality test of `FindDpPrimitivePair` (`:570`) applies to pads.
  assert_eq!(
    router.start_routing_diff_pair(LONE_P_AT, Some(LONE_P), 1),
    Err(StartError::NoCoupledStartItem(NET_N))
  );
}

#[test]
fn a_pair_gap_below_the_board_minimum_clearance_is_refused() {
  let settings = RoutingSettings {
    mode: RouterMode::Walkaround,
    ..RoutingSettings::default()
  };
  let mut sizes = sizes();

  sizes.min_clearance = GAP + 1;

  let mut router =
    Router::new(&board(Obstacle::None), Box::new(rules()), settings, sizes);

  assert_eq!(
    router.start_routing_diff_pair(START_P_AT, Some(START_P), 0),
    Err(StartError::PairGapBelowMinClearance)
  );
}

#[test]
fn the_same_pair_session_answers_the_same_thing_twice() {
  let first =
    route_across(&mut router_in(RouterMode::Walkaround, Obstacle::Crossing));
  let second =
    route_across(&mut router_in(RouterMode::Walkaround, Obstacle::Crossing));

  assert_eq!(first, second);
}

#[test]
fn a_pair_session_commits_both_lanes_through_the_facade() {
  let mut router = router_in(RouterMode::Walkaround, Obstacle::Crossing);

  router
    .start_routing_diff_pair(START_P_AT, Some(START_P), 0)
    .expect("the start pad pair is routable");

  // Two legs: one click before the obstacle, one on the target pair.
  router.move_to(Vec2::new(2_000_000, 0), None);

  assert!(matches!(
    router.fix_route(Vec2::new(2_000_000, 0), None, false),
    FixOutcome::Continue(_)
  ));

  router.move_to(TARGET_P_AT, Some(TARGET_P));

  let diff = match router.fix_route(TARGET_P_AT, Some(TARGET_P), false) {
    FixOutcome::Finished(diff) => diff,
    FixOutcome::Continue(_) => panic!("a fix on the target pair finishes"),
  };

  assert!(
    added_segments(&diff, NET_P) >= 2,
    "the P lane: {} segments",
    added_segments(&diff, NET_P)
  );
  assert!(
    added_segments(&diff, NET_N) >= 2,
    "the N lane: {} segments",
    added_segments(&diff, NET_N)
  );
  assert!(
    added_segments(&diff, NET_P) + added_segments(&diff, NET_N) >= 4,
    "{diff:?}"
  );
  assert_eq!(
    router.current_nets(),
    pnsrouter::router::RoutedNets::None,
    "the session ended"
  );
  assert_eq!(
    TARGET_N_AT,
    Vec2::new(8_000_000, PITCH / 2),
    "the N lane's target is where the fixture put it"
  );
  assert!(START_N == HostId(2) && TARGET_N == HostId(4));
}

// ---------------------------------------------------------------------
// getDanglingAnchor's arc arm
// ---------------------------------------------------------------------

/// `getDanglingAnchor`'s `ARC_T` case,
/// `pcbnew/router/pns_diff_pair_placer.cpp:479`: an arc answers with
/// whichever of its two endpoints sits on a joint with exactly one link,
/// that is, with the end nothing else connects to. An arc joined at both
/// ends answers nothing, which is what makes the placer tell the user to
/// click at the end of an existing pair.
#[test]
fn a_dangling_arc_answers_with_its_free_end() {
  // A quarter turn about `(1000000, 0)`, from the origin heading north
  // round to `(1000000, 1000000)`, with a straight run joined to its
  // **end** only.
  let bend = ShapeArc::new(
    Vec2::new(0, 0),
    Vec2::new(292_893, 707_107),
    Vec2::new(1_000_000, 1_000_000),
    WIDTH,
  );
  let free_end = Vec2::new(3_000_000, 1_000_000);
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(WorldItem::new(
    HostId(1),
    Some(NET_P),
    LayerRange::single(0),
    WorldGeometry::Arc {
      start: bend.start(),
      mid: bend.arc_mid(),
      end: bend.end(),
      width: WIDTH,
    },
  ));
  snapshot.items.push(WorldItem::new(
    HostId(2),
    Some(NET_P),
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(bend.end(), free_end),
      width: WIDTH,
    },
  ));

  let (world, index) = World::from_snapshot(&snapshot);
  let root = world.root();
  let resolve = |host: u64| {
    *index
      .items_of(HostId(host))
      .first()
      .expect("every snapshot item was stored")
  };
  let arc = resolve(1);
  let run = resolve(2);

  // The arc's start is free, its end is shared with the straight run.
  assert_eq!(
    DiffPairPlacer::dangling_anchor(&world, root, arc),
    Some(bend.start())
  );

  // The straight run's own free end is the other one, which is the
  // `SEGMENT_T` case the arc arm is the twin of.
  assert_eq!(
    DiffPairPlacer::dangling_anchor(&world, root, run),
    Some(free_end)
  );

  // An arc joined at both ends answers nothing.
  let mut closed = snapshot;

  closed.items.push(WorldItem::new(
    HostId(3),
    Some(NET_P),
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(Vec2::new(0, -1_000_000), bend.start()),
      width: WIDTH,
    },
  ));

  let (world, index) = World::from_snapshot(&closed);
  let root = world.root();
  let arc = *index
    .items_of(HostId(1))
    .first()
    .expect("every snapshot item was stored");

  assert_eq!(DiffPairPlacer::dangling_anchor(&world, root, arc), None);
}

// ---------------------------------------------------------------------
// Legs after a fix
// ---------------------------------------------------------------------

/// The lane width of the board a user reported duplicated legs on
/// (`TODO.md`, 2026-09-24).
const REPORTED_WIDTH: i32 = 125_000;

/// The copper gap of that board.
const REPORTED_GAP: i32 = 180_000;

/// The clearance of that board.
const REPORTED_CLEARANCE: i32 = 150_000;

/// The copper radius of that board's pads, wider than half a pitch, so
/// the first leg has to fan out of them.
const REPORTED_PAD_RADIUS: i32 = 250_000;

/// The four pad centres of that board: the start pair, then the target
/// pair four millimetres further south and twelve east.
const REPORTED_PADS: [Vec2; 4] = [
  Vec2::new(0, -600_000),
  Vec2::new(0, 600_000),
  Vec2::new(12_000_000, 4_400_000),
  Vec2::new(12_000_000, 5_600_000),
];

/// A router over the reported board, in one routing mode.
fn reported_router(mode: RouterMode) -> Router {
  let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);
  let hosts = [START_P, START_N, TARGET_P, TARGET_N];
  let nets = [NET_P, NET_N, NET_P, NET_N];

  for index in 0..4 {
    let at = REPORTED_PADS[index];

    snapshot.items.push(WorldItem::new(
      hosts[index],
      Some(nets[index]),
      LayerRange::single(0),
      WorldGeometry::Solid {
        shape: Shape::circle(at, REPORTED_PAD_RADIUS),
        pos: at,
        offset: Vec2::new(0, 0),
        orientation_degrees: 0.0,
        anchors: Vec::new(),
      },
    ));
  }

  let mut sizes = Sizes {
    track_width: REPORTED_WIDTH,
    board_min_track_width: 100_000,
    min_clearance: REPORTED_CLEARANCE,
    diff_pair_width: REPORTED_WIDTH,
    diff_pair_gap: REPORTED_GAP,
    diff_pair_via_gap: REPORTED_GAP,
    via_diameter: VIA_DIAMETER,
    via_drill: VIA_DRILL,
    ..Sizes::default()
  };

  sizes.add_layer_pair(0, 1);

  let settings = RoutingSettings {
    mode,
    ..RoutingSettings::default()
  };
  let rules =
    CoupledNets::new(FixedClearance::uniform(REPORTED_CLEARANCE), NET_P, NET_N);

  Router::new(&snapshot, Box::new(rules), settings, sizes)
}

/// Every segment a diff creates on one net.
fn added_segs(diff: &CommitDiff, net: NetId) -> Vec<Seg> {
  diff
    .added
    .iter()
    .filter(|item| item.net == Some(net))
    .filter_map(|item| match item.geometry {
      NewGeometry::Segment { seg, .. } => Some(seg),
      NewGeometry::Arc { .. } | NewGeometry::Via { .. } => None,
    })
    .collect()
}

/// Whether two segments lie on one line and share a stretch of it.
fn overlap(first: Seg, second: Seg) -> bool {
  let direction = first.b - first.a;
  let cross = |point: Vec2| {
    let offset = point - first.a;

    i64::from(direction.x) * i64::from(offset.y)
      - i64::from(direction.y) * i64::from(offset.x)
  };

  if cross(second.a) != 0 || cross(second.b) != 0 {
    return false;
  }

  let along = |point: Vec2| {
    let offset = point - first.a;

    i64::from(direction.x) * i64::from(offset.x)
      + i64::from(direction.y) * i64::from(offset.y)
  };
  let length = along(first.b);
  let (low, high) = (
    along(second.a).min(along(second.b)),
    along(second.a).max(along(second.b)),
  );

  low.max(0) < high.min(length)
}

/// Assert that one lane of a commit is one continuous chain from pad to
/// pad, with every segment used once and no two segments overlapping.
fn assert_lane_once(diff: &CommitDiff, net: NetId, from: Vec2, to: Vec2) {
  let segs = added_segs(diff, net);

  for (index, first) in segs.iter().enumerate() {
    for second in &segs[index + 1..] {
      assert!(
        !overlap(*first, *second),
        "{net:?} holds {first:?} and {second:?}, which overlap"
      );
    }
  }

  let chain = lane(diff, net, from);

  assert_eq!(
    chain.point_count(),
    segs.len() + 1,
    "{net:?} does not chain from {from:?}: {segs:?}"
  );
  assert_eq!(chain.last_point(), Some(to), "{net:?} ends elsewhere");
}

/// A router over the fixture's pads and sizes with the target pair moved
/// to the side, facing the start pair across a diagonal.
///
/// The pad pairs are the fixture's, so their lanes leave and arrive
/// horizontally, and the target pair sits so that a pair fixed on the
/// straight line out of the start has to turn by 45 degrees at once.
fn offset_router(mode: RouterMode) -> Router {
  let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(pad(START_P, START_P_AT, 0, NET_P));
  snapshot.items.push(pad(START_N, START_N_AT, 0, NET_N));
  snapshot.items.push(pad(TARGET_P, OFFSET_PADS[2], 0, NET_P));
  snapshot.items.push(pad(TARGET_N, OFFSET_PADS[3], 0, NET_N));

  let settings = RoutingSettings {
    mode,
    ..RoutingSettings::default()
  };

  Router::new(&snapshot, Box::new(rules()), settings, sizes())
}

/// The pad centres of [`offset_router`]'s board, start pair first.
const OFFSET_PADS: [Vec2; 4] = [
  START_P_AT,
  START_N_AT,
  Vec2::new(7_500_000, 4_000_000 - PITCH / 2),
  Vec2::new(7_500_000, 4_000_000 + PITCH / 2),
];

/// Start on the first pad, fix at every waypoint, then fix on the target
/// pair.
fn route_through(
  mut router: Router,
  pads: [Vec2; 4],
  waypoints: &[Vec2],
) -> CommitDiff {
  router
    .start_routing_diff_pair(pads[0], Some(START_P), 0)
    .expect("the start pad pair is routable");

  for waypoint in waypoints {
    router.move_to(*waypoint, None);

    assert!(
      matches!(
        router.fix_route(*waypoint, None, false),
        FixOutcome::Continue(_)
      ),
      "a fix at {waypoint:?} carries on"
    );
  }

  router.move_to(pads[2], Some(TARGET_P));

  match router.fix_route(pads[2], Some(TARGET_P), false) {
    FixOutcome::Finished(diff) => diff,
    FixOutcome::Continue(_) => {
      panic!("the fix on the target pair did not finish")
    }
  }
}

/// The reported session, with intermediate fixes on the straight run out
/// of the start pair and one more where the pair turns by more than 45
/// degrees. Before the fix the move after that turn failed, erratum E12
/// answered it with the leg just fixed, and the next fix wrote that leg a
/// second time, on top of the segment it had been merged into; the
/// target leg then started from the stale end. See
/// `doc/log/2026-09-24.md`.
#[test]
fn a_pair_with_intermediate_fixes_commits_each_lane_once() {
  let waypoints = [
    Vec2::new(2_000_000, 0),
    Vec2::new(4_000_000, 0),
    Vec2::new(6_000_000, 2_500_000),
  ];

  for mode in [
    RouterMode::MarkObstacles,
    RouterMode::Walkaround,
    RouterMode::Shove,
  ] {
    let diff = route_through(reported_router(mode), REPORTED_PADS, &waypoints);

    assert_lane_once(&diff, NET_P, REPORTED_PADS[0], REPORTED_PADS[2]);
    assert_lane_once(&diff, NET_N, REPORTED_PADS[1], REPORTED_PADS[3]);
  }
}

/// A pair that runs straight out of its pads, is fixed once and then has
/// to turn by 45 degrees at once to reach its target pair, in all three
/// modes. The turn needs the angled continuation gateways, which
/// KiCad's sine step never lets fit, so before the fix the target leg
/// failed and its fix committed the straight leg a second time instead.
#[test]
fn a_straight_pair_with_one_intermediate_fix_commits_each_lane_once() {
  for mode in [
    RouterMode::MarkObstacles,
    RouterMode::Walkaround,
    RouterMode::Shove,
  ] {
    let diff = route_through(
      offset_router(mode),
      OFFSET_PADS,
      &[Vec2::new(3_000_000, 0)],
    );

    assert_lane_once(&diff, NET_P, OFFSET_PADS[0], OFFSET_PADS[2]);
    assert_lane_once(&diff, NET_N, OFFSET_PADS[1], OFFSET_PADS[3]);
  }
}

/// The first move after a fix routes from where the fixed leg ends, even
/// when it has to turn at once, rather than answering with the leg just
/// fixed.
#[test]
fn the_move_after_a_fix_continues_from_the_fixed_leg() {
  let mut router = reported_router(RouterMode::Walkaround);
  let fixed_at = Vec2::new(4_000_000, 0);
  let cursor = Vec2::new(6_000_000, 2_500_000);

  router
    .start_routing_diff_pair(REPORTED_PADS[0], Some(START_P), 0)
    .expect("the start pad pair is routable");
  router.move_to(fixed_at, None);

  assert!(matches!(
    router.fix_route(fixed_at, None, false),
    FixOutcome::Continue(_)
  ));

  let frame = router.move_to(cursor, None);
  let head = |net: NetId| {
    frame
      .items
      .iter()
      .find(|item| {
        item.net == Some(net)
          && item.style == pnsrouter::router::PreviewStyle::Head
      })
      .map(|item| item.chain.clone())
      .unwrap_or_else(|| panic!("no {net:?} head: {frame:?}"))
  };
  let half_pitch = (REPORTED_WIDTH + REPORTED_GAP) / 2;

  for (net, start) in [
    (NET_P, Vec2::new(4_000_000, -half_pitch)),
    (NET_N, Vec2::new(4_000_000, half_pitch)),
  ] {
    let chain = head(net);
    let end = chain.last_point().expect("a head has points");

    assert_eq!(chain.point(0), start, "{net:?} head: {chain:?}");
    assert!(
      (end - cursor).euclidean_norm() <= half_pitch * 2,
      "{net:?} head ends at {end:?}, away from the cursor"
    );
  }
}

/// A move after a fix that no gateway can fit answers no head rather
/// than the leg just fixed, and a fix there writes nothing, where
/// erratum E12's sticky success carried across the fix used to write
/// the fixed leg a second time, on top of the segment it had been merged
/// into.
#[test]
fn a_fix_after_a_move_that_cannot_fit_writes_nothing() {
  for mode in [
    RouterMode::MarkObstacles,
    RouterMode::Walkaround,
    RouterMode::Shove,
  ] {
    let mut router = reported_router(mode);

    router
      .start_routing_diff_pair(REPORTED_PADS[0], Some(START_P), 0)
      .expect("the start pad pair is routable");

    for waypoint in [Vec2::new(2_000_000, 0), Vec2::new(4_000_000, 0)] {
      router.move_to(waypoint, None);

      assert!(matches!(
        router.fix_route(waypoint, None, false),
        FixOutcome::Continue(_)
      ));
    }

    // Behind the fixed end: a pair cannot turn back on itself.
    let behind = Vec2::new(1_000_000, 3_000_000);
    let frame = router.move_to(behind, None);

    assert!(
      frame.items.iter().all(|item| {
        item.style != pnsrouter::router::PreviewStyle::Head
          || item.chain.segment_count() == 0
      }),
      "{mode:?}: a stale head {frame:?}"
    );
    assert!(matches!(
      router.fix_route(behind, None, false),
      FixOutcome::Continue(_)
    ));

    router.move_to(REPORTED_PADS[2], Some(TARGET_P));

    let FixOutcome::Finished(diff) =
      router.fix_route(REPORTED_PADS[2], Some(TARGET_P), false)
    else {
      panic!("{mode:?}: the fix on the target pair did not finish");
    };

    assert_lane_once(&diff, NET_P, REPORTED_PADS[0], REPORTED_PADS[2]);
    assert_lane_once(&diff, NET_N, REPORTED_PADS[1], REPORTED_PADS[3]);
  }
}
