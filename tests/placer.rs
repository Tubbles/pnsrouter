// SPDX-License-Identifier: GPL-3.0-or-later

//! Line placer scenarios on a two layer board, through the public API
//! only.
//!
//! `doc/work/003-walkaround-router.md` asks for the placer to be driven
//! over a board with pads and existing traces rather than only in unit
//! tests. The board is built out of plain data the way `tests/world.rs`
//! and `tests/walkaround.rs` build theirs, and every scenario that fixes
//! a route asserts that what reached the node is actually clear, through
//! `World::check_colliding_line`.
//!
//! Vias, the fixed tail and undo are part 2 of the line placer work item
//! and are not exercised here.

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::collide::CollisionSearchOptions;
use pnsrouter::geometry::direction45::{CornerMode, Direction45};
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{ItemBody, ItemId, Kind, LayerRange, NetId, Solid};
use pnsrouter::line::Line;
use pnsrouter::node::{NodeId, World};
use pnsrouter::placer::line_placer::{LinePlacer, PlacerState};
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RouterMode, RoutingSettings, Sizes};

/// The clearance every scenario routes to, in nanometres.
const CLEARANCE: i32 = 100_000;

/// The width of the routed track.
const TRACK_WIDTH: i32 = 200_000;

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 400_000;

/// The net the start and target pads are on, which is the net the placer
/// routes.
const TRACE_NET: Option<NetId> = Some(NetId(1));

/// The net of the obstacle pad, so that it is never exempt from the
/// routed track.
const OBSTACLE_NET: Option<NetId> = Some(NetId(2));

/// The net of the pad on the second layer.
const TOP_NET: Option<NetId> = Some(NetId(3));

/// Where the routed track starts.
const START: Vec2 = Vec2::new(0, 0);

/// The pad the track has to get past.
const OBSTACLE: Vec2 = Vec2::new(2_000_000, 0);

/// Where the routed track ends.
const TARGET: Vec2 = Vec2::new(4_000_000, 0);

/// The only pad on the second layer.
const TOP: Vec2 = Vec2::new(0, -2_000_000);

/// How far a track's centreline has to stay from a pad's centre: the
/// pad's copper, the clearance and half the track's width.
const KEEP_OUT: i32 = PAD_RADIUS + CLEARANCE + TRACK_WIDTH / 2;

/// The pads of the fixture board.
struct Board {
  /// The pad the route starts on, on the routed net.
  start_pad: ItemId,
  /// The pad in the way, on another net.
  obstacle_pad: ItemId,
  /// The pad the route ends on, on the routed net.
  target_pad: ItemId,
}

/// Turn the plain data above into a world.
fn build() -> (World, Board) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let add_pad = |world: &mut World, at: Vec2, layer: i32, net| {
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(net);

    world.add_solid(root, item, None)
  };

  let start_pad = add_pad(&mut world, START, 0, TRACE_NET);
  let obstacle_pad = add_pad(&mut world, OBSTACLE, 0, OBSTACLE_NET);
  let target_pad = add_pad(&mut world, TARGET, 0, TRACE_NET);

  add_pad(&mut world, TOP, 1, TOP_NET);

  (
    world,
    Board {
      start_pad,
      obstacle_pad,
      target_pad,
    },
  )
}

/// The rule oracle every scenario uses.
fn rules() -> FixedClearance {
  FixedClearance::uniform(CLEARANCE)
}

/// The sizes every scenario places with.
fn sizes() -> Sizes {
  Sizes {
    track_width: TRACK_WIDTH,
    ..Sizes::default()
  }
}

/// Settings in one routing mode, with everything else at KiCad's
/// defaults.
fn settings_for(mode: RouterMode) -> RoutingSettings {
  RoutingSettings {
    mode,
    ..RoutingSettings::default()
  }
}

/// A placer over the root of a world.
fn placer_for(world: &World, settings: &RoutingSettings) -> LinePlacer {
  LinePlacer::new(world, world.root(), settings, sizes())
}

/// Every segment of one net in a node, as a line that can be collision
/// tested.
fn stored_segments(
  world: &World,
  node: NodeId,
  net: Option<NetId>,
) -> Vec<Line> {
  world
    .all_items_in_net(node, net, Kind::SEGMENT)
    .into_iter()
    .filter_map(|id| Line::from_segment(world, node, id))
    .collect()
}

/// Fail when anything the placer stored is not clear.
fn assert_stored_segments_are_clear(
  world: &World,
  node: NodeId,
  net: Option<NetId>,
  rules: &FixedClearance,
) {
  let segments = stored_segments(world, node, net);

  assert!(!segments.is_empty(), "nothing was stored");

  for line in segments {
    assert!(
      world
        .check_colliding_line(
          node,
          &line,
          rules,
          &CollisionSearchOptions::default()
        )
        .is_none(),
      "a fixed segment from {:?} to {:?} collides",
      line.point(0),
      line.last_point()
    );
  }
}

/// Whether every corner of a chain sits on the 45 degree grid.
fn is_on_the_grid(line: &Line) -> bool {
  (0..line.segment_count()).all(|index| {
    let segment = line.segment(index);
    let delta = segment.b - segment.a;

    delta.x == 0 || delta.y == 0 || delta.x.abs() == delta.y.abs()
  })
}

/// The squared distance from a point to the obstacle pad's centre.
fn squared_distance_to_obstacle(point: Vec2) -> i64 {
  let delta = point - OBSTACLE;

  i64::from(delta.x) * i64::from(delta.x)
    + i64::from(delta.y) * i64::from(delta.y)
}

#[test]
fn a_head_over_empty_space_follows_the_cursor_on_the_grid() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let cursor = Vec2::new(1_200_000, -1_500_000);

  assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
  assert!(placer.move_to(&mut world, &context, cursor, None));

  let trace = placer.trace().expect("a placement is running");

  assert_eq!(placer.current_end(), Some(cursor));
  assert_eq!(trace.last_point(), Some(cursor));
  assert_eq!(trace.point(0), START);
  assert!(is_on_the_grid(&trace), "the head left the 45 degree grid");
  // A 45 degree posture over a delta that is neither axis aligned nor
  // exactly diagonal needs one corner.
  assert_eq!(trace.segment_count(), 2);
}

#[test]
fn a_walkaround_head_gets_past_a_pad_and_the_fix_is_clear() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);

  assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
  assert!(placer.move_to(&mut world, &context, TARGET, None));

  let trace = placer.trace().expect("a placement is running");

  assert!(is_on_the_grid(&trace), "the walk left the 45 degree grid");
  assert!(
    trace.shape().points().iter().any(|point| point.y != 0),
    "the head went straight through the obstacle instead of around it"
  );

  for index in 0..trace.point_count() {
    assert!(
      squared_distance_to_obstacle(trace.point(index))
        >= i64::from(KEEP_OUT) * i64::from(KEEP_OUT),
      "a corner of the walk sits inside the obstacle's keep out"
    );
  }

  // Fixing on the target pad ends the placement, because the pad is on
  // the routed net.
  assert!(placer.fix_route(
    &mut world,
    &context,
    TARGET,
    Some(board.target_pad),
    false
  ));
  assert!(matches!(
    placer.state(),
    PlacerState::Finished {
      placed_anything: true
    }
  ));

  let node = placer.last_node().expect("the fix wrote into a branch");

  assert_stored_segments_are_clear(&world, node, TRACE_NET, &rules);
}

#[test]
fn an_intermediate_fix_starts_the_next_leg_and_chains_the_layer() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let corner = Vec2::new(1_000_000, -1_000_000);

  assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
  assert!(placer.move_to(&mut world, &context, corner, None));

  // No end item, so the fix is intermediate and the placement continues.
  assert!(!placer.fix_route(&mut world, &context, corner, None, false));
  assert!(placer.has_placed_anything());
  assert!(matches!(placer.state(), PlacerState::Placing(_)));

  let restarted = placer
    .current_start()
    .expect("the placement continues from the fix");

  assert_ne!(restarted, START, "the next leg starts where the fix ended");
  assert_stored_segments_are_clear(
    &world,
    placer.current_node(false),
    TRACE_NET,
    &rules,
  );

  // A layer change is refused for the rest of a chained placement,
  // because nothing placed a via.
  assert!(!placer.set_layer(&mut world, &context, 1));
  assert_eq!(placer.current_layer(), Some(0));

  // The next leg keeps routing from the new start.
  let next = Vec2::new(1_000_000, -2_500_000);

  assert!(placer.move_to(&mut world, &context, next, None));
  assert_eq!(placer.current_end(), Some(next));
  assert_eq!(
    placer.trace().expect("a placement is running").point(0),
    restarted
  );
}

#[test]
fn mark_obstacles_mode_runs_the_head_into_the_pad() {
  let (mut world, board) = build();
  let rules = rules();
  let walk_settings = settings_for(RouterMode::Walkaround);
  let mark_settings = settings_for(RouterMode::MarkObstacles);

  let marked = {
    let context = AlgoContext::new(&rules, &mark_settings);
    let mut placer = placer_for(&world, &mark_settings);

    assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
    assert!(placer.move_to(&mut world, &context, TARGET, None));

    placer.trace().expect("a placement is running")
  };

  // The head goes straight at the obstacle rather than around it.
  assert_eq!(marked.point(0), START);
  assert!(
    marked.shape().points().iter().all(|point| point.y == 0),
    "mark obstacles mode walked around the pad"
  );

  assert!(
    world
      .check_colliding_line(
        world.root(),
        &marked,
        &rules,
        &CollisionSearchOptions::default()
      )
      .and_then(|obstacle| obstacle.item)
      == Some(board.obstacle_pad),
    "the straight head does not report the pad it runs into"
  );

  // The same move in walkaround mode is clear.
  let walked = {
    let context = AlgoContext::new(&rules, &walk_settings);
    let mut placer = placer_for(&world, &walk_settings);

    assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
    assert!(placer.move_to(&mut world, &context, TARGET, None));

    placer.trace().expect("a placement is running")
  };

  assert!(
    world
      .check_colliding_line(
        world.root(),
        &walked,
        &rules,
        &CollisionSearchOptions::default()
      )
      .is_none(),
    "the walkaround head still collides"
  );

  // A colliding trace cannot be fixed while rule violations are refused.
  let context = AlgoContext::new(&rules, &mark_settings);
  let mut placer = placer_for(&world, &mark_settings);

  assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
  assert!(placer.move_to(&mut world, &context, TARGET, None));
  assert!(!placer.fix_route(
    &mut world,
    &context,
    TARGET,
    Some(board.target_pad),
    false
  ));
}

#[test]
fn fix_all_segments_decides_whether_the_last_leg_stays_rubber_banded() {
  // A cursor that is neither axis aligned nor exactly diagonal, so that
  // the head has a bend and there is a last leg to leave unfixed.
  let corner = Vec2::new(1_200_000, -1_500_000);
  let mut ends = Vec::new();

  for fix_all in [true, false] {
    let (mut world, board) = build();
    let rules = rules();
    let settings = RoutingSettings {
      fix_all_segments: fix_all,
      ..settings_for(RouterMode::Walkaround)
    };
    let context = AlgoContext::new(&rules, &settings);
    let mut placer = placer_for(&world, &settings);

    assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
    assert!(placer.move_to(&mut world, &context, corner, None));

    let trace = placer.trace().expect("a placement is running");
    let segment_count = trace.segment_count();

    assert!(segment_count > 1, "the scenario needs a bend to fix");
    assert!(!placer.fix_route(&mut world, &context, corner, None, false));

    let node = placer.current_node(false);
    let stored = stored_segments(&world, node, TRACE_NET).len();

    ends.push((
      placer.current_start().expect("the placement continues"),
      stored,
      segment_count,
    ));
  }

  let (all_start, all_stored, all_segments) = ends[0];
  let (last_start, last_stored, last_segments) = ends[1];

  // With the setting on, every segment reaches the node and the next leg
  // starts at the cursor.
  assert_eq!(all_start, corner);
  assert_eq!(all_stored, all_segments);

  // With it off, the last segment stays under the cursor.
  assert_ne!(last_start, corner);
  assert_eq!(last_stored, last_segments - 1);
}

#[test]
fn a_layer_change_without_a_start_item_resets_the_preview() {
  let (mut world, _board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let start = Vec2::new(3_000_000, -3_000_000);
  let cursor = Vec2::new(4_000_000, -4_000_000);

  assert!(placer.start(&mut world, &context, start, None));
  assert!(placer.move_to(&mut world, &context, cursor, None));
  assert_eq!(placer.current_layer(), Some(0));

  // No start item, so any layer is allowed, and the preview comes back on
  // the new one.
  assert!(placer.set_layer(&mut world, &context, 1));
  assert_eq!(placer.current_layer(), Some(1));
  assert_eq!(
    placer.head().and_then(Line::last_point),
    Some(cursor),
    "the preview was not regenerated on the new layer"
  );
  assert_eq!(placer.head().map(Line::layer), Some(1));
}

#[test]
fn flipping_the_posture_moves_the_first_corner() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let cursor = Vec2::new(1_200_000, -1_500_000);

  assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
  assert!(placer.move_to(&mut world, &context, cursor, None));

  let before = placer.trace().expect("a placement is running").point(1);

  placer.flip_posture();

  assert!(placer.move_to(&mut world, &context, cursor, None));

  let after = placer.trace().expect("a placement is running").point(1);

  assert_ne!(before, after, "the posture flip did not move the corner");

  // Both postures reach the cursor on the grid.
  let trace = placer.trace().expect("a placement is running");

  assert_eq!(trace.last_point(), Some(cursor));
  assert!(is_on_the_grid(&trace));
}

#[test]
fn ortho_mode_gives_one_straight_leg() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let cursor = Vec2::new(1_200_000, -1_500_000);

  assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));

  placer.set_ortho_mode(true);

  assert!(placer.move_to(&mut world, &context, cursor, None));

  let trace = placer.trace().expect("a placement is running");

  assert_eq!(
    trace.segment_count(),
    1,
    "ortho mode left more than one leg"
  );

  let segment = trace.segment(0);

  assert_eq!(segment.a, START);
  assert!(
    Direction45::from_seg(&segment, false).is_defined(),
    "the ortho leg is not on the 45 degree grid"
  );
  assert!(is_on_the_grid(&trace));
}

#[test]
fn a_ninety_degree_corner_mode_keeps_the_head_axis_aligned() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = RoutingSettings {
    corner_mode: CornerMode::Mitered90,
    ..settings_for(RouterMode::Walkaround)
  };
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let cursor = Vec2::new(1_200_000, -1_500_000);

  assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
  assert!(placer.move_to(&mut world, &context, cursor, None));

  let trace = placer.trace().expect("a placement is running");

  for index in 0..trace.segment_count() {
    let segment = trace.segment(index);
    let delta = segment.b - segment.a;

    assert!(
      delta.x == 0 || delta.y == 0,
      "a 90 degree corner mode produced a diagonal leg"
    );
  }
}

#[test]
fn committing_moves_the_segments_into_the_root_and_drops_the_branches() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let root = world.root();

  assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));
  assert!(placer.move_to(&mut world, &context, TARGET, None));
  assert!(placer.fix_route(
    &mut world,
    &context,
    TARGET,
    Some(board.target_pad),
    false
  ));

  let branch = placer.current_node(false);
  let scratch = placer.last_node().expect("the fix wrote into a branch");
  let expected = stored_segments(&world, scratch, TRACE_NET).len();

  assert!(expected > 0);
  assert!(placer.commit_placement(&mut world));

  // The branches are gone and the segments are in the root.
  assert!(world.node(branch).is_none());
  assert!(world.node(scratch).is_none());
  assert_eq!(placer.current_node(false), root);
  assert_eq!(stored_segments(&world, root, TRACE_NET).len(), expected);
  assert_stored_segments_are_clear(&world, root, TRACE_NET, &rules);
}

#[test]
fn two_identical_runs_produce_the_same_geometry() {
  let cursors = [
    Vec2::new(800_000, -400_000),
    Vec2::new(1_800_000, -900_000),
    Vec2::new(3_000_000, -600_000),
    TARGET,
  ];
  let mut runs = Vec::new();

  for _ in 0..2 {
    let (mut world, board) = build();
    let rules = rules();
    let settings = settings_for(RouterMode::Walkaround);
    let context = AlgoContext::new(&rules, &settings);
    let mut placer = placer_for(&world, &settings);
    let root = world.root();

    assert!(placer.start(&mut world, &context, START, Some(board.start_pad)));

    for cursor in cursors {
      assert!(placer.move_to(&mut world, &context, cursor, None));
    }

    assert!(placer.fix_route(
      &mut world,
      &context,
      TARGET,
      Some(board.target_pad),
      false
    ));
    assert!(placer.commit_placement(&mut world));

    let mut committed: Vec<(Vec2, Vec2)> =
      stored_segments(&world, root, TRACE_NET)
        .iter()
        .map(|line| (line.point(0), line.point(1)))
        .collect();

    committed.sort_by_key(|(from, to)| (from.x, from.y, to.x, to.y));
    runs.push(committed);
  }

  assert_eq!(runs[0], runs[1], "two identical runs disagreed");
  assert!(!runs[0].is_empty());
}

#[test]
fn a_track_under_the_cursor_is_split_at_the_start_point() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);

  // A track of the routed net, well clear of everything else, to start in
  // the middle of.
  let seg = pnsrouter::geometry::seg::Seg::new(
    Vec2::new(0, -3_000_000),
    Vec2::new(4_000_000, -3_000_000),
  );
  let body = ItemBody::Segment(pnsrouter::item::Segment::new(seg, TRACK_WIDTH));
  let root = world.root();
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::single(0));
  item.set_net(TRACE_NET);

  let track = world
    .add_segment(root, item, false)
    .expect("the track is neither degenerate nor redundant");
  let middle = Vec2::new(2_000_000, -3_000_000);
  let mut placer = placer_for(&world, &settings);

  assert!(placer.start(&mut world, &context, middle, Some(track)));

  let node = placer.current_node(false);

  assert!(
    world.find_joint(node, middle, 0, TRACE_NET).is_some(),
    "the start point did not become a joint"
  );
  assert_eq!(placer.current_net(), TRACE_NET);
  // The original track is shadowed in the branch and the two halves take
  // its place.
  assert_eq!(
    world
      .all_items_in_net(node, TRACE_NET, Kind::SEGMENT)
      .into_iter()
      .filter(|id| *id != track)
      .count(),
    2
  );

  // The board's own pads are untouched by the split.
  assert!(world.item(board.obstacle_pad).is_some());
  assert!(world.item(board.start_pad).is_some());
  assert!(world.item(board.target_pad).is_some());
}
