// SPDX-License-Identifier: GPL-3.0-or-later

//! Routing sessions driven through the facade only.
//!
//! `doc/work/005-session-api-and-event-log.md` asks that a host be able
//! to drive a whole session without any callback except the rule
//! resolver. Every scenario here therefore builds a
//! [`WorldSnapshot`] of plain data, hands it to a [`Router`], and speaks
//! only in [`HostId`]s, [`PreviewFrame`]s and [`CommitDiff`]s; nothing
//! reaches into the world or the placer except to check what a commit
//! actually left behind.
//!
//! The board is the two layer fixture of `tests/placer.rs` re expressed
//! as a snapshot, with an existing track and an existing via added so
//! that the sync of `src/snapshot.rs` is exercised on every item kind a
//! LibrePCB board holds.

#![forbid(unsafe_code)]

use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{HostId, Kind, LayerRange, NetId, ViaType};
use pnsrouter::node::World;
use pnsrouter::router::{
  CommitDiff, FixOutcome, NewGeometry, PreviewFrame, PreviewStyle, Router,
  RouterState, StartError,
};
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RouterMode, RoutingSettings, Sizes};
use pnsrouter::snapshot::{WorldGeometry, WorldItem, WorldSnapshot};

/// The clearance every scenario routes to, in nanometres.
const CLEARANCE: i32 = 100_000;

/// The width of the routed track.
const TRACK_WIDTH: i32 = 200_000;

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 400_000;

/// The net the start and target pads are on.
const TRACE_NET: Option<NetId> = Some(NetId(1));

/// The net of the obstacle pad, so that it is never exempt.
const OBSTACLE_NET: Option<NetId> = Some(NetId(2));

/// The net of the track a session may start in the middle of.
const SPLIT_NET: Option<NetId> = Some(NetId(4));

/// The pad the route starts on, layer 0.
const START_PAD: HostId = HostId(1);

/// The pad in the way, layer 0.
const OBSTACLE_PAD: HostId = HostId(2);

/// The pad the route ends on, layer 0.
const TARGET_PAD: HostId = HostId(3);

/// An existing track a session can start in the middle of, layer 0.
const EXISTING_TRACK: HostId = HostId(4);

/// The pad the two layer route ends on, layer 1.
const DEEP_PAD: HostId = HostId(5);

/// An existing via, well away from everything else.
const EXISTING_VIA: HostId = HostId(6);

/// Where the route starts.
const START: Vec2 = Vec2::new(0, 0);

/// The pad the route has to get past.
const OBSTACLE: Vec2 = Vec2::new(2_000_000, 0);

/// Where the single layer route ends.
const TARGET: Vec2 = Vec2::new(4_000_000, 0);

/// Where the layer change happens, clear of everything on both layers.
const VIA_POINT: Vec2 = Vec2::new(0, -3_000_000);

/// Where the two layer route ends, on layer 1.
const DEEP_TARGET: Vec2 = Vec2::new(3_000_000, -3_000_000);

/// One end of [`EXISTING_TRACK`].
const TRACK_A: Vec2 = Vec2::new(0, 3_000_000);

/// The other end of [`EXISTING_TRACK`].
const TRACK_B: Vec2 = Vec2::new(4_000_000, 3_000_000);

/// The middle of [`EXISTING_TRACK`], which a start splits it at.
const TRACK_MIDDLE: Vec2 = Vec2::new(2_000_000, 3_000_000);

/// A round pad on one copper layer.
fn pad(id: HostId, at: Vec2, layer: i32, net: Option<NetId>) -> WorldItem {
  WorldItem::new(
    id,
    net,
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

/// The fixture board, as a host would hand it over.
fn board() -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(pad(START_PAD, START, 0, TRACE_NET));
  snapshot
    .items
    .push(pad(OBSTACLE_PAD, OBSTACLE, 0, OBSTACLE_NET));
  snapshot.items.push(pad(TARGET_PAD, TARGET, 0, TRACE_NET));
  snapshot
    .items
    .push(pad(DEEP_PAD, DEEP_TARGET, 1, TRACE_NET));
  snapshot.items.push(WorldItem::new(
    EXISTING_TRACK,
    SPLIT_NET,
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(TRACK_A, TRACK_B),
      width: TRACK_WIDTH,
    },
  ));
  snapshot.items.push(WorldItem::new(
    EXISTING_VIA,
    Some(NetId(6)),
    LayerRange::new(0, 1),
    WorldGeometry::Via {
      pos: Vec2::new(0, -9_000_000),
      diameter: 600_000,
      drill: 300_000,
      via_type: ViaType::Through,
      is_free: false,
    },
  ));

  snapshot
}

/// The sizes every scenario places with.
///
/// The layer pair is what `Sizes::via_layer_range` answers from, so a via
/// placed here spans both copper layers.
fn sizes() -> Sizes {
  let mut sizes = Sizes {
    track_width: TRACK_WIDTH,
    board_min_track_width: 100_000,
    via_diameter: 600_000,
    via_drill: 300_000,
    ..Sizes::default()
  };

  sizes.add_layer_pair(0, 1);
  sizes
}

/// A router over the fixture, in one routing mode.
fn router_in(mode: RouterMode) -> Router {
  let settings = RoutingSettings {
    mode,
    ..RoutingSettings::default()
  };

  Router::new(
    &board(),
    Box::new(FixedClearance::uniform(CLEARANCE)),
    settings,
    sizes(),
  )
}

/// The default router, in walkaround mode.
fn router() -> Router {
  router_in(RouterMode::Walkaround)
}

/// How many segments a diff creates.
fn added_segments(diff: &CommitDiff) -> usize {
  diff
    .added
    .iter()
    .filter(|item| matches!(item.geometry, NewGeometry::Segment { .. }))
    .count()
}

/// How many vias a diff creates.
fn added_vias(diff: &CommitDiff) -> usize {
  diff
    .added
    .iter()
    .filter(|item| matches!(item.geometry, NewGeometry::Via { .. }))
    .count()
}

/// Every segment of one net stored in the committed board.
fn committed_segments(router: &Router, net: Option<NetId>) -> usize {
  let world = router.world();

  world
    .all_items_in_net(world.root(), net, Kind::SEGMENT)
    .len()
}

/// The head of a frame, which is the route being drawn.
fn head_of(frame: &PreviewFrame) -> Option<&pnsrouter::router::PreviewItem> {
  frame
    .items
    .iter()
    .find(|item| item.style == PreviewStyle::Head)
}

/// Route from the start pad to the target pad and finish there.
///
/// The shared body of the scenarios that only need a committed route.
fn route_across(router: &mut Router) -> CommitDiff {
  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(TARGET, Some(TARGET_PAD));

  match router.fix_route(TARGET, Some(TARGET_PAD), false) {
    FixOutcome::Finished(diff) => diff,
    FixOutcome::Continue(_) => panic!("the fix on the target pad continued"),
  }
}

// ---------------------------------------------------------------------
// The session
// ---------------------------------------------------------------------

#[test]
fn hover_names_the_host_object_under_the_cursor() {
  let router = router();

  assert_eq!(router.hover(START, Some(0)), vec![START_PAD]);
  assert_eq!(router.hover(TRACK_MIDDLE, Some(0)), vec![EXISTING_TRACK]);

  // The layer filter is the tool's, so a layer 0 query does not see the
  // layer 1 pad and a query with no layer does.
  assert!(router.hover(DEEP_TARGET, Some(0)).is_empty());
  assert_eq!(router.hover(DEEP_TARGET, None), vec![DEEP_PAD]);

  // Empty board is empty.
  assert!(router.hover(Vec2::new(50_000_000, 0), None).is_empty());
}

#[test]
fn a_start_on_a_pad_puts_the_session_into_route_track() {
  let mut router = router();

  assert_eq!(router.state(), RouterState::Idle);
  assert!(!router.routing_in_progress());
  assert!(
    router
      .is_starting_point_routable(START, Some(START_PAD), 0)
      .is_ok()
  );

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");

  assert_eq!(router.state(), RouterState::RouteTrack);
  assert!(router.routing_in_progress());
  assert_eq!(router.current_layer(), Some(0));
  assert_eq!(router.current_net(), TRACE_NET);
  assert!(!router.placing_via());

  // A second start is refused rather than silently replacing the first.
  assert_eq!(
    router.start_routing(START, Some(START_PAD), 0),
    Err(StartError::AlreadyRouting)
  );
}

#[test]
fn shove_mode_routes_and_commits_through_the_facade() {
  let mut router = router_in(RouterMode::Shove);

  assert_eq!(router.settings().mode, RouterMode::Shove);

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("shove mode starts a session like the other modes");
  assert_eq!(router.state(), RouterState::RouteTrack);
  router.abort_routing();
  assert_eq!(router.state(), RouterState::Idle);

  let mut router = router_in(RouterMode::Shove);
  let diff = route_across(&mut router);

  assert!(added_segments(&diff) > 0);
  assert_eq!(router.state(), RouterState::Idle);
  assert_eq!(
    committed_segments(&router, TRACE_NET),
    added_segments(&diff)
  );
}

#[test]
fn a_move_draws_a_head_that_walks_around_the_pad_in_the_way() {
  let mut router = router();

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");

  let frame = router.move_to(TARGET, Some(TARGET_PAD));
  let head = head_of(&frame).expect("the move drew no head");

  assert_eq!(head.width, TRACK_WIDTH);
  assert_eq!(head.layer, 0);
  assert_eq!(head.net, TRACE_NET);
  assert_eq!(head.clearance, Some(CLEARANCE));
  assert_eq!(head.chain.point(0), START);
  assert!(
    head.chain.points().iter().any(|point| point.y != 0),
    "the head went straight through the obstacle instead of around it"
  );

  // Nothing is armed and nothing collides, so the frame is otherwise
  // empty.
  assert!(frame.via.is_none());
  assert!(frame.violations.is_empty());
  assert!(frame.hidden.is_empty());
}

#[test]
fn a_fix_continues_and_then_finishes_on_the_far_pad() {
  let mut router = router();
  let corner = Vec2::new(1_000_000, -1_500_000);

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(corner, None);

  // No end item, so the placement carries on.
  let FixOutcome::Continue(frame) = router.fix_route(corner, None, false)
  else {
    panic!("an intermediate fix finished the session");
  };

  assert_eq!(router.state(), RouterState::RouteTrack);
  assert!(
    frame
      .items
      .iter()
      .any(|item| item.style == PreviewStyle::Tail),
    "the fixed leg did not reach the preview"
  );

  router.move_to(TARGET, Some(TARGET_PAD));

  let FixOutcome::Finished(diff) =
    router.fix_route(TARGET, Some(TARGET_PAD), false)
  else {
    panic!("the fix on the target pad did not finish");
  };

  assert_eq!(router.state(), RouterState::Idle);
  assert!(!router.routing_in_progress());
  assert!(added_segments(&diff) >= 2, "{diff:?}");
  assert_eq!(added_vias(&diff), 0);
  assert!(diff.removed.is_empty());
  assert!(diff.updated.is_empty());

  for item in &diff.added {
    assert_eq!(item.net, TRACE_NET);
    assert_eq!(item.layers, LayerRange::single(0));
    assert_eq!(item.source, None, "a routed segment has no source object");
  }

  // The commit really reached the board.
  assert_eq!(committed_segments(&router, TRACE_NET), diff.added.len());
}

#[test]
fn an_armed_via_reaches_the_preview_and_then_the_diff() {
  let mut router = router();

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(VIA_POINT, None);

  assert!(router.toggle_via_placement());
  assert!(router.placing_via());

  // The via is materialised on the next move, not by the toggle.
  let frame = router.move_to(VIA_POINT, None);
  let via = frame.via.expect("the armed via did not reach the preview");

  assert_eq!(via.pos, VIA_POINT);
  assert_eq!(via.drill, 300_000);
  assert_eq!(via.layers, LayerRange::new(0, 1));
  assert_eq!(via.style, PreviewStyle::Head);

  // The fix stores it and frees the layer, so the rest of the route can
  // run on layer 1.
  let FixOutcome::Continue(_) = router.fix_route(VIA_POINT, None, false) else {
    panic!("an intermediate fix finished the session");
  };

  assert!(!router.placing_via());
  assert!(router.switch_layer(1));
  assert_eq!(router.current_layer(), Some(1));

  router.move_to(DEEP_TARGET, Some(DEEP_PAD));

  let FixOutcome::Finished(diff) =
    router.fix_route(DEEP_TARGET, Some(DEEP_PAD), false)
  else {
    panic!("the fix on the layer 1 pad did not finish");
  };

  assert_eq!(added_vias(&diff), 1, "{diff:?}");
  assert!(added_segments(&diff) >= 2, "{diff:?}");

  let Some(NewGeometry::Via { pos, diameter, .. }) = diff
    .added
    .iter()
    .find(|item| matches!(item.geometry, NewGeometry::Via { .. }))
    .map(|item| item.geometry)
  else {
    panic!("the via left the diff");
  };

  assert_eq!(pos, VIA_POINT);
  assert_eq!(diameter, 600_000);
}

#[test]
fn a_layer_switch_is_refused_outside_the_board_and_without_a_via() {
  let mut router = router();

  // Nothing is being routed yet.
  assert!(!router.switch_layer(1));

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");

  assert_eq!(router.copper_layer_count(), 2);
  assert!(!router.switch_layer(2), "a layer the board does not have");
  assert!(!router.switch_layer(-1));

  // The start pad only exists on layer 0, so the placement may not walk
  // off it without a via.
  assert!(!router.switch_layer(1));
  assert_eq!(router.current_layer(), Some(0));

  // A placement that started on nothing may change layer freely, until a
  // fix without a via chains it.
  let mut floating = router;

  floating.abort_routing();
  floating
    .start_routing(Vec2::new(0, -6_000_000), None, 0)
    .expect("empty board is routable");

  assert!(floating.switch_layer(1));
  assert_eq!(floating.current_layer(), Some(1));
  assert!(floating.switch_layer(0));

  let corner = Vec2::new(1_000_000, -7_500_000);

  floating.move_to(corner, None);
  floating.fix_route(corner, None, false);

  assert!(!floating.switch_layer(1));
  assert_eq!(floating.current_layer(), Some(0));
}

// ---------------------------------------------------------------------
// The commit
// ---------------------------------------------------------------------

#[test]
fn starting_in_the_middle_of_a_track_folds_a_removal_into_an_update() {
  let mut router = router();
  let corner = Vec2::new(4_000_000, 5_000_000);

  router
    .start_routing(TRACK_MIDDLE, Some(EXISTING_TRACK), 0)
    .expect("a track is a routable start");
  router.move_to(corner, None);

  let FixOutcome::Finished(diff) = router.fix_route(corner, None, true) else {
    panic!("a forced finish did not finish");
  };

  // The split removed the original track and added its two halves. The
  // fold turns the first half into an update of the host object, so
  // nothing is reported as removed at all.
  assert!(
    diff.removed.is_empty(),
    "the split was reported as a removal: {diff:?}"
  );
  assert_eq!(diff.updated.len(), 1, "{diff:?}");
  assert_eq!(diff.updated[0].0, EXISTING_TRACK);
  assert_eq!(diff.updated[0].1.source, Some(EXISTING_TRACK));
  assert_eq!(diff.updated[0].1.net, SPLIT_NET);

  // The other half is a plain addition that remembers where it came
  // from, which is what a host inherits per object attributes through.
  let inherited = diff
    .added
    .iter()
    .filter(|item| item.source == Some(EXISTING_TRACK))
    .count();

  assert_eq!(inherited, 1, "{diff:?}");
  assert!(added_segments(&diff) > inherited, "nothing was routed");

  // The updated half kept the host object's identity in the map.
  assert!(!router.host_index().items_of(EXISTING_TRACK).is_empty());
}

#[test]
fn the_host_reports_its_own_ids_back_for_the_additions() {
  let mut router = router();
  let diff = route_across(&mut router);
  let assignments: Vec<(usize, HostId)> = (0..diff.added.len())
    .map(|index| (index, HostId(100 + index as u64)))
    .collect();

  assert_eq!(router.assign_host_ids(&assignments), diff.added.len());

  for (_, host) in &assignments {
    assert_eq!(
      router.host_index().items_of(*host).len(),
      1,
      "{host:?} did not reach the map"
    );
  }

  // An index past the end of the diff is ignored rather than panicking.
  assert_eq!(router.assign_host_ids(&[(999, HostId(999))]), 0);
}

#[test]
fn stopping_commits_and_aborting_discards() {
  let corner = Vec2::new(1_000_000, -1_500_000);
  let mut committed = router();

  committed
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  committed.move_to(corner, None);
  committed.fix_route(corner, None, false);

  let diff = committed.stop_routing();

  assert_eq!(committed.state(), RouterState::Idle);
  assert!(added_segments(&diff) >= 1, "{diff:?}");
  assert_eq!(committed_segments(&committed, TRACE_NET), diff.added.len());

  let mut discarded = router();

  discarded
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  discarded.move_to(corner, None);
  discarded.fix_route(corner, None, false);
  discarded.abort_routing();

  assert_eq!(discarded.state(), RouterState::Idle);
  assert_eq!(committed_segments(&discarded, TRACE_NET), 0);
  assert_eq!(
    committed_segments(&discarded, SPLIT_NET),
    1,
    "the board's own track did not survive the abort"
  );
}

#[test]
fn stopping_a_session_that_placed_nothing_changes_nothing() {
  let mut router = router();

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(TARGET, None);

  let diff = router.stop_routing();

  assert_eq!(diff, CommitDiff::default());
  assert_eq!(committed_segments(&router, TRACE_NET), 0);
}

#[test]
fn undoing_gives_the_cursor_back_to_where_the_leg_began() {
  let mut router = router();
  let corner = Vec2::new(1_000_000, -1_500_000);

  // Nothing is being routed yet.
  assert_eq!(router.undo_last_segment(), None);

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(corner, None);
  router.fix_route(corner, None, false);

  let next = Vec2::new(1_000_000, -3_000_000);

  router.move_to(next, None);

  let back = router.undo_last_segment().expect("the undo found a stage");

  assert_ne!(back, next, "the undo answered with the live cursor");
  assert_eq!(router.state(), RouterState::RouteTrack);

  // The undone leg is gone from the world the next fix would commit.
  let diff = router.stop_routing();

  assert_eq!(added_segments(&diff), 0, "{diff:?}");
}

// ---------------------------------------------------------------------
// The preview
// ---------------------------------------------------------------------

#[test]
fn mark_obstacles_mode_reports_the_pad_the_head_runs_into() {
  let mut router = router_in(RouterMode::MarkObstacles);

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");

  let frame = router.move_to(TARGET, Some(TARGET_PAD));
  let head = head_of(&frame).expect("the move drew no head");

  // The head goes straight at the obstacle rather than around it.
  assert!(
    head.chain.points().iter().all(|point| point.y == 0),
    "mark obstacles mode walked around the pad"
  );

  let marker = frame
    .violations
    .iter()
    .find(|marker| marker.host == Some(OBSTACLE_PAD))
    .expect("the obstacle pad was not marked");

  assert_eq!(marker.clearance, CLEARANCE);
  assert!(marker.hide_original, "a plain pad is not a compound object");
  assert_eq!(
    marker.forced_layer, None,
    "a single layer pad keeps its own layer"
  );

  // A colliding route cannot be fixed while rule violations are refused.
  assert!(matches!(
    router.fix_route(TARGET, Some(TARGET_PAD), false),
    FixOutcome::Continue(_)
  ));
  assert_eq!(router.state(), RouterState::RouteTrack);
}

#[test]
fn the_corner_mode_cycles_between_the_two_mitered_modes() {
  use pnsrouter::geometry::direction45::CornerMode;

  let mut router = router();

  assert_eq!(router.settings().corner_mode, CornerMode::Mitered45);
  router.toggle_corner_mode();
  assert_eq!(router.settings().corner_mode, CornerMode::Mitered90);
  router.toggle_corner_mode();
  assert_eq!(router.settings().corner_mode, CornerMode::Mitered45);
}

#[test]
fn the_small_commands_refuse_an_idle_router() {
  let mut router = router();

  assert!(!router.toggle_via_placement());
  assert!(!router.switch_layer(1));
  assert!(!router.placing_via());
  assert_eq!(router.current_layer(), None);
  assert_eq!(router.current_net(), None);
  assert!(router.finish().is_none());
  assert!(router.continue_from_end().is_none());
  assert_eq!(router.move_to(TARGET, None), PreviewFrame::default());

  // These four answer nothing at all, so the test is that they are
  // callable while idle without panicking.
  router.flip_posture();
  router.set_ortho_mode(true);
  router.set_sizes(sizes());
  router.abort_routing();
}

// ---------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------

#[test]
fn two_identical_sessions_produce_the_same_diff_and_the_same_frames() {
  let run = || {
    let mut router = router();
    let mut frames = Vec::new();

    frames.push(
      router
        .start_routing(START, Some(START_PAD), 0)
        .expect("the start pad is routable"),
    );
    frames.push(router.move_to(Vec2::new(1_000_000, -1_500_000), None));

    if let FixOutcome::Continue(frame) =
      router.fix_route(Vec2::new(1_000_000, -1_500_000), None, false)
    {
      frames.push(frame);
    }

    frames.push(router.move_to(TARGET, Some(TARGET_PAD)));

    let FixOutcome::Finished(diff) =
      router.fix_route(TARGET, Some(TARGET_PAD), false)
    else {
      panic!("the fix on the target pad did not finish");
    };

    (frames, diff)
  };

  let (first_frames, first_diff) = run();
  let (second_frames, second_diff) = run();

  assert_eq!(first_diff, second_diff);
  assert_eq!(first_frames, second_frames);
  assert!(added_segments(&first_diff) >= 2, "{first_diff:?}");
}
