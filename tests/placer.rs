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
//! The via scenarios and the undo scenarios run over a second fixture,
//! [`build_two_layer`], whose pads sit on both copper layers so that a
//! via has something to connect and something to be pushed out of.

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
use pnsrouter::rules::{FixedClearance, ItemRef};
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
///
/// The layer pair is what `Sizes::via_layer_range` answers from, so a via
/// placed here spans both copper layers of the fixtures.
fn sizes() -> Sizes {
  let mut sizes = Sizes {
    track_width: TRACK_WIDTH,
    ..Sizes::default()
  };

  sizes.add_layer_pair(0, 1);
  sizes
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

// ---------------------------------------------------------------------
// The two layer fixture
// ---------------------------------------------------------------------

/// The copper radius of a via placed by these scenarios.
const VIA_RADIUS: i32 = 300_000;

/// How far a via's centre has to stay from a pad's centre it may not
/// touch: the pad's copper, the clearance and the via's own copper.
const VIA_KEEP_OUT: i32 = PAD_RADIUS + CLEARANCE + VIA_RADIUS;

/// Where the two layer route starts, on a pad of layer 0.
const DEEP_START: Vec2 = Vec2::new(0, 0);

/// The first thing in the way, on layer 0.
const FIRST_OBSTACLE: Vec2 = Vec2::new(2_000_000, 0);

/// The second thing in the way, on layer 0.
const SECOND_OBSTACLE: Vec2 = Vec2::new(4_000_000, 0);

/// Where the layer change happens, clear of everything on both layers.
const VIA_POINT: Vec2 = Vec2::new(6_000_000, 0);

/// The third thing in the way, on layer 1 only.
const THIRD_OBSTACLE: Vec2 = Vec2::new(8_000_000, 0);

/// Where the two layer route ends, on a pad of layer 1.
const DEEP_TARGET: Vec2 = Vec2::new(10_000_000, 0);

/// A pad on layer 1 that only a via can collide with, well away from the
/// route above.
const BURIED_PAD: Vec2 = Vec2::new(3_000_000, -4_000_000);

/// A cursor position that overlaps [`BURIED_PAD`] without sitting on its
/// centre.
///
/// Two concentric circles have no minimum translation vector, so a via
/// dropped exactly on a round pad's centre cannot be pushed anywhere;
/// `Via::pushout_force` answers `None` for that pair and the placer would
/// report the via as unplaceable rather than move it.
const OVER_BURIED_PAD: Vec2 = Vec2::new(3_200_000, -4_000_000);

/// The pads of the two layer fixture.
struct TwoLayerBoard {
  /// The pad the route starts on, layer 0, routed net.
  start_pad: ItemId,
  /// The pad the route ends on, layer 1, routed net.
  target_pad: ItemId,
  /// A pad of another net on layer 1 that a layer 0 track walks straight
  /// past and a via cannot.
  buried_pad: ItemId,
}

/// A board with copper on both layers and three obstacles.
///
/// Layer 0 carries the start pad and two obstacles between it and
/// [`VIA_POINT`]; layer 1 carries one obstacle between [`VIA_POINT`] and
/// the target pad, plus [`BURIED_PAD`], which is the one a via pushout
/// has to notice.
fn build_two_layer() -> (World, TwoLayerBoard) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let add_pad = |world: &mut World, at: Vec2, layer: i32, net| {
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(net);

    world.add_solid(root, item, None)
  };

  let start_pad = add_pad(&mut world, DEEP_START, 0, TRACE_NET);

  add_pad(&mut world, FIRST_OBSTACLE, 0, OBSTACLE_NET);
  add_pad(&mut world, SECOND_OBSTACLE, 0, OBSTACLE_NET);
  add_pad(&mut world, THIRD_OBSTACLE, 1, OBSTACLE_NET);

  let buried_pad = add_pad(&mut world, BURIED_PAD, 1, OBSTACLE_NET);
  let target_pad = add_pad(&mut world, DEEP_TARGET, 1, TRACE_NET);

  (
    world,
    TwoLayerBoard {
      start_pad,
      target_pad,
      buried_pad,
    },
  )
}

/// Every via of one net in a node.
fn stored_vias(world: &World, node: NodeId, net: Option<NetId>) -> Vec<ItemId> {
  world.all_items_in_net(node, net, Kind::VIA)
}

/// Fail when a stored item collides with anything in its node.
fn assert_item_is_clear(
  world: &World,
  node: NodeId,
  id: ItemId,
  rules: &FixedClearance,
) {
  let item = world.item(id).expect("the item is still in the arena");

  assert!(
    world
      .check_colliding(
        node,
        ItemRef::stored(id, item),
        rules,
        &CollisionSearchOptions::default()
      )
      .is_none(),
    "a stored item collides"
  );
}

/// Route from the start pad to [`VIA_POINT`] with a via armed, and fix
/// there.
///
/// The shared prelude of the via scenarios: it leaves the placer running,
/// on layer 0, with one leg fixed and a via at its end.
fn route_to_the_via_point(
  world: &mut World,
  context: &AlgoContext<'_>,
  placer: &mut LinePlacer,
  board: &TwoLayerBoard,
) {
  assert!(placer.start(world, context, DEEP_START, Some(board.start_pad)));
  assert!(placer.move_to(world, context, VIA_POINT, None));
  assert!(placer.toggle_via(true));
  assert!(placer.is_placing_via());
  assert!(placer.move_to(world, context, VIA_POINT, None));

  // An intermediate fix: no end item, so the placement carries on.
  assert!(!placer.fix_route(world, context, VIA_POINT, None, false));
}

#[test]
fn a_via_is_stored_at_the_fix_and_frees_the_layer() {
  let (mut world, board) = build_two_layer();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);

  route_to_the_via_point(&mut world, &context, &mut placer, &board);

  let node = placer.current_node(false);
  let vias = stored_vias(&world, node, TRACE_NET);

  assert_eq!(vias.len(), 1, "the fix did not store exactly one via");

  let via = world.item(vias[0]).expect("the via is in the arena");

  // The via spans both layers, which is what makes the layer change
  // legal, and it sits where the leg ended.
  assert!(via.layers().contains(0) && via.layers().contains(1));
  assert_eq!(via.anchor(0), VIA_POINT);
  assert_item_is_clear(&world, node, vias[0], &rules);

  // A fix that ended on a via leaves the placement unchained, so the
  // layer may still change, which a fix without one refuses.
  assert!(!placer.is_placing_via(), "the via flag survived the fix");
  assert!(placer.set_layer(&mut world, &context, 1));
  assert_eq!(placer.current_layer(), Some(1));
  assert_eq!(placer.head().map(Line::layer), Some(1));
}

#[test]
fn a_layer_change_is_refused_after_a_fix_without_a_via() {
  let (mut world, board) = build_two_layer();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let corner = Vec2::new(1_000_000, -1_500_000);

  assert!(placer.start(
    &mut world,
    &context,
    DEEP_START,
    Some(board.start_pad)
  ));
  assert!(placer.move_to(&mut world, &context, corner, None));
  assert!(!placer.fix_route(&mut world, &context, corner, None, false));

  // No via at the fix, so the layer is pinned for the rest of the
  // placement.
  assert!(!placer.set_layer(&mut world, &context, 1));
  assert_eq!(placer.current_layer(), Some(0));
}

#[test]
fn a_via_over_a_pad_of_another_layer_is_pushed_clear() {
  let (mut world, board) = build_two_layer();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);

  assert!(placer.start(
    &mut world,
    &context,
    DEEP_START,
    Some(board.start_pad)
  ));
  assert!(placer.toggle_via(true));
  assert!(placer.move_to(&mut world, &context, OVER_BURIED_PAD, None));

  let head = placer.head().expect("a placement is running");

  assert!(head.ends_with_via(), "the head carries no via");

  let via_pos = head.via_pos(&world).expect("the via has a position");
  let node = placer.current_node(false);

  // The track itself runs on layer 0 and never sees the pad, so nothing
  // but the via can have moved the end of the trace.
  assert_ne!(
    via_pos, OVER_BURIED_PAD,
    "the via stayed on top of a pad of the other layer"
  );

  let delta = via_pos - BURIED_PAD;

  assert!(
    i64::from(delta.x) * i64::from(delta.x)
      + i64::from(delta.y) * i64::from(delta.y)
      >= i64::from(VIA_KEEP_OUT) * i64::from(VIA_KEEP_OUT),
    "the pushed out via is still inside the pad's keep out"
  );
  assert!(
    world
      .check_colliding(
        node,
        head.via_item(&world).expect("the head has a via"),
        &rules,
        &CollisionSearchOptions::default()
      )
      .is_none(),
    "the pushed out via still collides"
  );
  assert!(world.item(board.buried_pad).is_some());
}

#[test]
fn a_via_only_commit_stores_the_via_and_ends_the_placement() {
  let (mut world, board) = build_two_layer();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let root = world.root();

  assert!(placer.start(
    &mut world,
    &context,
    DEEP_START,
    Some(board.start_pad)
  ));
  assert!(placer.toggle_via(true));

  // The cursor never leaves the start point, so the trace is a via and
  // nothing else.
  assert!(placer.move_to(&mut world, &context, DEEP_START, None));

  let trace = placer.trace().expect("a placement is running");

  assert_eq!(trace.segment_count(), 0, "the trace grew a segment");
  assert!(trace.ends_with_via());
  assert_eq!(placer.current_end(), Some(DEEP_START));

  // The via only commit ends the placement, where a fix on an empty
  // trace without a via would answer false.
  assert!(placer.fix_route(&mut world, &context, DEEP_START, None, false));
  assert!(matches!(
    placer.state(),
    PlacerState::Finished {
      placed_anything: true
    }
  ));
  assert!(placer.commit_placement(&mut world));

  let vias = stored_vias(&world, root, TRACE_NET);

  assert_eq!(vias.len(), 1, "the via only commit stored no via");
  assert_eq!(
    world
      .item(vias[0])
      .expect("the via is in the arena")
      .anchor(0),
    DEEP_START
  );
}

// ---------------------------------------------------------------------
// Undo
// ---------------------------------------------------------------------

/// Route the two layer board as far as a second fix, leaving three
/// stages on the fixed tail.
///
/// Leg one runs on layer 0 and ends on a via at [`VIA_POINT`], the layer
/// changes, and leg two runs on layer 1 as far as `corner`.
fn route_two_legs(
  world: &mut World,
  context: &AlgoContext<'_>,
  placer: &mut LinePlacer,
  board: &TwoLayerBoard,
  corner: Vec2,
) {
  route_to_the_via_point(world, context, placer, board);

  assert!(placer.set_layer(world, context, 1));
  assert!(placer.move_to(world, context, corner, None));
  assert!(!placer.fix_route(world, context, corner, None, false));
}

#[test]
fn an_undo_after_two_fixes_comes_back_to_the_second_leg() {
  let (mut world, board) = build_two_layer();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let corner = Vec2::new(7_000_000, -1_000_000);

  route_two_legs(&mut world, &context, &mut placer, &board, corner);

  let after_two = placer.current_node(false);
  let segments_after_two = stored_segments(&world, after_two, TRACE_NET).len();

  assert!(segments_after_two > 1, "the scenario needs two legs");
  assert_eq!(placer.current_layer(), Some(1));

  // A move so that the head has something to report back.
  assert!(placer.move_to(&mut world, &context, DEEP_TARGET, None));

  let back = placer.undo_last_segment(&mut world, &context);

  assert!(back.is_some(), "the undo reported no point");

  // The second leg started at the via, on layer 1.
  assert_eq!(placer.current_start(), Some(VIA_POINT));
  assert_eq!(placer.current_layer(), Some(1));

  let after_undo = placer.current_node(false);
  let segments_after_undo = stored_segments(&world, after_undo, TRACE_NET);

  assert!(
    segments_after_undo.len() < segments_after_two,
    "the undo left the second leg's segments in the node"
  );
  assert!(
    segments_after_undo.iter().all(|line| line.layer() == 0),
    "a segment of the undone layer 1 leg survived"
  );

  // The via of the first fix is still there: it belongs to the leg that
  // is still fixed.
  assert_eq!(stored_vias(&world, after_undo, TRACE_NET).len(), 1);
}

#[test]
fn undoing_down_to_the_first_stage_empties_the_placement() {
  let (mut world, board) = build_two_layer();
  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let corner = Vec2::new(7_000_000, -1_000_000);

  route_two_legs(&mut world, &context, &mut placer, &board, corner);

  // Back over the second leg.
  placer.undo_last_segment(&mut world, &context);

  // Back over the first leg, which restores layer 0 and the via that was
  // armed when it was fixed.
  placer.undo_last_segment(&mut world, &context);

  assert_eq!(placer.current_start(), Some(DEEP_START));
  assert_eq!(placer.current_layer(), Some(0));
  assert!(
    placer.is_placing_via(),
    "the via flag of the first fix was not restored"
  );

  let node = placer.current_node(false);

  assert!(stored_segments(&world, node, TRACE_NET).is_empty());
  assert!(stored_vias(&world, node, TRACE_NET).is_empty());

  // The bottom stage is handed out for ever, so further undos are safe
  // and keep answering with the point the placement started at.
  for _ in 0..2 {
    placer.undo_last_segment(&mut world, &context);

    assert_eq!(placer.current_start(), Some(DEEP_START));
    assert_eq!(placer.current_layer(), Some(0));
  }

  // The placement is still live and can be routed again.
  assert!(placer.move_to(&mut world, &context, VIA_POINT, None));
  assert_eq!(
    placer.trace().expect("a placement is running").point(0),
    DEEP_START
  );
}

// ---------------------------------------------------------------------
// The milestone 3 exit criterion
// ---------------------------------------------------------------------

/// Where the cursor stops on layer 0 before the via, per mode.
///
/// Walkaround mode is handed the via point directly and is expected to
/// find its own way past the two pads. Mark obstacles mode does not route
/// around anything by design, so it gets the corners a user would click,
/// which is the whole difference between the two modes.
fn stops_before_the_via(mode: RouterMode) -> Vec<Vec2> {
  match mode {
    RouterMode::MarkObstacles => vec![
      Vec2::new(1_000_000, -1_000_000),
      Vec2::new(5_000_000, -1_000_000),
      VIA_POINT,
    ],
    _ => vec![VIA_POINT],
  }
}

/// Where the cursor stops on layer 1 after the via, per mode.
fn stops_after_the_via(mode: RouterMode) -> Vec<Vec2> {
  match mode {
    RouterMode::MarkObstacles => vec![
      Vec2::new(7_000_000, -1_000_000),
      Vec2::new(9_000_000, -1_000_000),
      DEEP_TARGET,
    ],
    _ => vec![DEEP_TARGET],
  }
}

/// Route the whole two layer board and commit.
///
/// Start on a pad of layer 0, get past two obstacles, place a via, change
/// layer, get past a third obstacle and finish on a pad of the same net.
/// The last fix is forced, which is what a host does when the user
/// double clicks rather than landing exactly on the target.
fn route_across_the_board(mode: RouterMode) -> World {
  let (mut world, board) = build_two_layer();
  let rules = rules();
  let settings = RoutingSettings {
    fix_all_segments: true,
    ..settings_for(mode)
  };
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);
  let before = stops_before_the_via(mode);
  let after = stops_after_the_via(mode);

  assert!(placer.start(
    &mut world,
    &context,
    DEEP_START,
    Some(board.start_pad)
  ));

  // Layer 0, up to the point the via goes.
  for (index, stop) in before.iter().enumerate() {
    let last = index + 1 == before.len();

    if last {
      assert!(placer.toggle_via(true));
    }

    assert!(placer.move_to(&mut world, &context, *stop, None));
    assert!(
      !placer.fix_route(&mut world, &context, *stop, None, false),
      "an intermediate fix ended the placement"
    );
  }

  // The via at the end of the last leg is what unpins the layer.
  assert!(
    placer.set_layer(&mut world, &context, 1),
    "the layer change was refused although the fix left a via"
  );
  assert_eq!(placer.current_layer(), Some(1));

  // Layer 1, up to the target pad.
  for (index, stop) in after.iter().enumerate() {
    let last = index + 1 == after.len();
    let end_item = last.then_some(board.target_pad);

    assert!(placer.move_to(&mut world, &context, *stop, end_item));

    let finished =
      placer.fix_route(&mut world, &context, *stop, end_item, last);

    assert_eq!(finished, last, "the placement ended at the wrong stop");
  }

  assert!(placer.has_placed_anything());
  assert!(placer.commit_placement(&mut world));

  world
}

/// Fail when a corner of any of the lines sits inside a pad's keep out.
fn assert_corners_are_clear_of(
  lines: &[&Line],
  obstacle: Vec2,
  mode: RouterMode,
) {
  for line in lines {
    for index in 0..line.point_count() {
      let delta = line.point(index) - obstacle;
      let squared = i64::from(delta.x) * i64::from(delta.x)
        + i64::from(delta.y) * i64::from(delta.y);

      assert!(
        squared >= i64::from(KEEP_OUT) * i64::from(KEEP_OUT),
        "{mode:?}: a corner sits inside the keep out of ({}, {})",
        obstacle.x,
        obstacle.y
      );
    }
  }
}

/// Whether any of the lines starts or ends at a point.
fn touches(lines: &[&Line], point: Vec2) -> bool {
  lines
    .iter()
    .any(|line| line.point(0) == point || line.last_point() == Some(point))
}

/// The committed route as plain comparable data.
fn committed_route(world: &World) -> (Vec<(Vec2, Vec2, i32)>, Vec<Vec2>) {
  let root = world.root();
  let mut segments: Vec<(Vec2, Vec2, i32)> =
    stored_segments(world, root, TRACE_NET)
      .iter()
      .map(|line| (line.point(0), line.point(1), line.layer()))
      .collect();
  let mut vias: Vec<Vec2> = stored_vias(world, root, TRACE_NET)
    .into_iter()
    .filter_map(|id| world.item(id).map(|item| item.anchor(0)))
    .collect();

  segments
    .sort_by_key(|(from, to, layer)| (*layer, from.x, from.y, to.x, to.y));
  vias.sort_by_key(|point| (point.x, point.y));

  (segments, vias)
}

#[test]
fn a_two_layer_route_reaches_the_far_pad_through_a_via() {
  for mode in [RouterMode::Walkaround, RouterMode::MarkObstacles] {
    let world = route_across_the_board(mode);
    let rules = rules();
    let root = world.root();
    let segments = stored_segments(&world, root, TRACE_NET);
    let vias = stored_vias(&world, root, TRACE_NET);

    assert_eq!(vias.len(), 1, "{mode:?}: the route has no single via");

    let via = world.item(vias[0]).expect("the via is in the arena");
    let via_pos = via.anchor(0);

    assert!(
      via.layers().contains(0) && via.layers().contains(1),
      "{mode:?}: the via does not join the two layers"
    );

    let bottom: Vec<&Line> =
      segments.iter().filter(|line| line.layer() == 0).collect();
    let top: Vec<&Line> =
      segments.iter().filter(|line| line.layer() == 1).collect();

    assert!(
      !bottom.is_empty(),
      "{mode:?}: nothing was routed on layer 0"
    );
    assert!(!top.is_empty(), "{mode:?}: nothing was routed on layer 1");

    // The route runs pad to via to pad.
    assert!(
      touches(&bottom, DEEP_START),
      "{mode:?}: the route does not start on the start pad"
    );
    assert!(
      touches(&bottom, via_pos),
      "{mode:?}: the layer 0 leg does not reach the via"
    );
    assert!(
      touches(&top, via_pos),
      "{mode:?}: the layer 1 leg does not leave the via"
    );
    assert!(
      touches(&top, DEEP_TARGET),
      "{mode:?}: the route does not end on the target pad"
    );

    // The detour actually happened: every corner of the layer 0 leg is
    // outside the keep out of both pads it had to get past, and the same
    // on layer 1 for the third one.
    for obstacle in [FIRST_OBSTACLE, SECOND_OBSTACLE] {
      assert_corners_are_clear_of(&bottom, obstacle, mode);
    }

    assert_corners_are_clear_of(&top, THIRD_OBSTACLE, mode);
    assert_stored_segments_are_clear(&world, root, TRACE_NET, &rules);
    assert_item_is_clear(&world, root, vias[0], &rules);
  }
}

#[test]
fn the_two_layer_route_is_the_same_twice() {
  for mode in [RouterMode::Walkaround, RouterMode::MarkObstacles] {
    let first = committed_route(&route_across_the_board(mode));
    let second = committed_route(&route_across_the_board(mode));

    assert_eq!(first, second, "{mode:?}: two identical runs disagreed");
    assert!(!first.0.is_empty());
    assert_eq!(first.1.len(), 1);
  }
}

// ---------------------------------------------------------------------
// The milestone 4 exit criterion
// ---------------------------------------------------------------------

/// The radius of the two pads the shove route runs between.
///
/// Deliberately small: the pads have to stay clear of tracks that the
/// **head** is close enough to push, and a pad is never shoved, so a
/// route that starts inside a violation could never be fixed.
const SHOVE_PAD_RADIUS: i32 = 50_000;

/// Where the shove route starts.
const SHOVE_START: Vec2 = Vec2::new(0, 0);

/// Where the shove route ends.
const SHOVE_TARGET: Vec2 = Vec2::new(8_000_000, 0);

/// A cursor short of everything the route has to push, which is what the
/// springback step retreats to.
const SHOVE_RETREAT: Vec2 = Vec2::new(2_000_000, 0);

/// The net of the track running nearest the route.
const NEAR_TRACK_NET: Option<NetId> = Some(NetId(4));

/// The net of the track beyond it, which only moves once the near one has
/// been pushed into it.
const FAR_TRACK_NET: Option<NetId> = Some(NetId(5));

/// The net of the via the route pushes aside.
const SHOVE_VIA_NET: Option<NetId> = Some(NetId(6));

/// The net of the track on layer 1, which nothing may disturb.
const OTHER_LAYER_TRACK_NET: Option<NetId> = Some(NetId(7));

/// The corners of the track running nearest the route.
const NEAR_TRACK: [Vec2; 2] = [
  Vec2::new(-1_000_000, 260_000),
  Vec2::new(9_000_000, 260_000),
];

/// The corners of the track beyond it.
const FAR_TRACK: [Vec2; 2] = [
  Vec2::new(-1_500_000, 560_000),
  Vec2::new(9_500_000, 560_000),
];

/// The corners of the track on layer 1.
const OTHER_LAYER_TRACK: [Vec2; 2] =
  [Vec2::new(0, -2_500_000), Vec2::new(8_000_000, -2_500_000)];

/// Where the via the route has to push sits before anything happens.
const SHOVE_VIA: Vec2 = Vec2::new(4_000_000, -300_000);

/// The diameter of that via.
const SHOVE_VIA_DIAMETER: i32 = 400_000;

/// Its drill.
const SHOVE_VIA_DRILL: i32 = 200_000;

/// The board the shove scenarios run on.
struct ShoveBoard {
  /// The pad the route starts on.
  start_pad: ItemId,
  /// The pad the route ends on.
  target_pad: ItemId,
}

/// A two layer board with two parallel tracks and a via in the way.
///
/// The tracks reach past both ends of the route, so their own endpoints
/// are outside everything the head can push against; that is what lets
/// the shove bend them without having to move an endpoint, which
/// `shoveLineToHullSet` refuses to do
/// (`pcbnew/router/pns_shove.cpp:455`).
fn build_shove_board() -> (World, ShoveBoard) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let add_pad = |world: &mut World, at: Vec2, net| {
    let body =
      ItemBody::Solid(Solid::new(Shape::circle(at, SHOVE_PAD_RADIUS), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);

    world.add_solid(root, item, None)
  };

  let start_pad = add_pad(&mut world, SHOVE_START, TRACE_NET);
  let target_pad = add_pad(&mut world, SHOVE_TARGET, TRACE_NET);

  let add_track = |world: &mut World, ends: [Vec2; 2], layer, net| {
    let body = ItemBody::Segment(pnsrouter::item::Segment::new(
      pnsrouter::geometry::seg::Seg::new(ends[0], ends[1]),
      TRACK_WIDTH,
    ));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(net);
    world
      .add_segment(root, item, false)
      .expect("a board track is neither degenerate nor redundant");
  };

  add_track(&mut world, NEAR_TRACK, 0, NEAR_TRACK_NET);
  add_track(&mut world, FAR_TRACK, 0, FAR_TRACK_NET);
  add_track(&mut world, OTHER_LAYER_TRACK, 1, OTHER_LAYER_TRACK_NET);

  let body = ItemBody::Via(pnsrouter::item::Via::new(
    SHOVE_VIA,
    SHOVE_VIA_DIAMETER,
    SHOVE_VIA_DRILL,
    pnsrouter::item::ViaType::Through,
  ));
  let mut via = world.make_item(body);

  via.set_layers_and_flash_all(LayerRange::new(0, 1));
  via.set_net(SHOVE_VIA_NET);
  world.add_via(root, via);

  (
    world,
    ShoveBoard {
      start_pad,
      target_pad,
    },
  )
}

/// The corners of one net's track in a node.
fn track_shape(world: &World, node: NodeId, net: Option<NetId>) -> Vec<Vec2> {
  let segments = world.all_items_in_net(node, net, Kind::SEGMENT);
  let first = *segments.first().expect("the net has at least one segment");

  world
    .assemble_line(node, first, None, false, false, true)
    .shape()
    .points()
    .to_vec()
}

/// Where one net's via sits in a node.
fn via_position(
  world: &World,
  node: NodeId,
  net: Option<NetId>,
) -> Option<Vec2> {
  let vias = world.all_items_in_net(node, net, Kind::VIA);

  world.item(*vias.first()?).map(|item| item.anchor(0))
}

/// Every net the shove board holds.
const SHOVE_BOARD_NETS: [Option<NetId>; 5] = [
  TRACE_NET,
  NEAR_TRACK_NET,
  FAR_TRACK_NET,
  SHOVE_VIA_NET,
  OTHER_LAYER_TRACK_NET,
];

/// Fail when anything a node holds collides with anything else.
fn assert_shove_node_is_clear(
  world: &World,
  node: NodeId,
  rules: &FixedClearance,
) {
  let options = CollisionSearchOptions::default();

  for net in SHOVE_BOARD_NETS {
    for id in world.all_items_in_net(node, net, Kind::SEGMENT) {
      let line = Line::from_segment(world, node, id)
        .expect("the node listed the segment");

      assert!(
        world
          .check_colliding_line(node, &line, rules, &options)
          .is_none(),
        "a track on net {net:?} collides"
      );
    }

    for id in world.all_items_in_net(node, net, Kind::VIA) {
      let item = world.item(id).expect("the node listed the via");
      let line = Line::from_linked_via(id, item);

      assert!(
        world
          .check_colliding_line(node, &line, rules, &options)
          .is_none(),
        "a via on net {net:?} collides"
      );
    }
  }
}

/// Route the shove board end to end and commit, answering the world.
///
/// The move to [`SHOVE_RETREAT`] in the middle is the springback step:
/// the frame the long move pushed is no longer in any head's way, so
/// `reduceSpringback` drops it and the board comes back
/// (`pcbnew/router/pns_shove.cpp:924`).
fn route_the_shove_board() -> World {
  let (mut world, board) = build_shove_board();
  let rules = rules();
  let settings = RoutingSettings {
    fix_all_segments: true,
    ..settings_for(RouterMode::Shove)
  };
  let context = AlgoContext::new(&rules, &settings);
  let mut placer = placer_for(&world, &settings);

  assert!(placer.start(
    &mut world,
    &context,
    SHOVE_START,
    Some(board.start_pad)
  ));

  // A first short move, so that the springback stack has a frame below
  // the one the long move pushes: `reduceSpringback` never drops its
  // bottom frame (`pns_shove.cpp:926`), so what springback can undo is
  // everything above this state and not the state itself.
  assert!(placer.move_to(&mut world, &context, SHOVE_RETREAT, None));

  let short = placer.current_node(false);
  let near_short = track_shape(&world, short, NEAR_TRACK_NET);
  let far_short = track_shape(&world, short, FAR_TRACK_NET);

  assert_eq!(
    via_position(&world, short, SHOVE_VIA_NET),
    Some(SHOVE_VIA),
    "the short move should not have reached the via"
  );

  // The long move pushes both tracks and the via aside.
  assert!(placer.move_to(&mut world, &context, SHOVE_TARGET, None));

  let pushed = placer.current_node(false);
  let near_pushed = track_shape(&world, pushed, NEAR_TRACK_NET);
  let via_pushed = via_position(&world, pushed, SHOVE_VIA_NET);

  assert_ne!(near_pushed, near_short, "the near track never moved");
  assert_ne!(via_pushed, Some(SHOVE_VIA), "the via never moved");
  assert_ne!(
    track_shape(&world, pushed, FAR_TRACK_NET),
    far_short,
    "the far track never moved"
  );
  assert_shove_node_is_clear(&world, pushed, &rules);

  // The cursor retreats: the frame the long move pushed stands in
  // nothing's way any more, so it is dropped and the board comes back to
  // exactly what the short move had left.
  assert!(placer.move_to(&mut world, &context, SHOVE_RETREAT, None));

  let sprung = placer.current_node(false);

  assert_eq!(
    via_position(&world, sprung, SHOVE_VIA_NET),
    Some(SHOVE_VIA),
    "the via did not spring back"
  );
  assert_eq!(
    track_shape(&world, sprung, FAR_TRACK_NET),
    far_short,
    "the far track did not spring back"
  );
  assert_eq!(
    track_shape(&world, sprung, NEAR_TRACK_NET),
    near_short,
    "the near track did not spring back"
  );

  // And then the route is finished for real.
  assert!(placer.move_to(
    &mut world,
    &context,
    SHOVE_TARGET,
    Some(board.target_pad)
  ));
  assert!(placer.fix_route(
    &mut world,
    &context,
    SHOVE_TARGET,
    Some(board.target_pad),
    true
  ));
  assert!(placer.has_placed_anything());
  assert!(placer.commit_placement(&mut world));

  world
}

/// The committed board as plain comparable data.
fn committed_shove_board(world: &World) -> Vec<(Option<NetId>, Vec<Vec2>)> {
  let root = world.root();
  let mut rows: Vec<(Option<NetId>, Vec<Vec2>)> = Vec::new();

  for net in SHOVE_BOARD_NETS {
    let mut segments: Vec<Vec2> = world
      .all_items_in_net(root, net, Kind::SEGMENT)
      .into_iter()
      .filter_map(|id| Line::from_segment(world, root, id))
      .flat_map(|line| [line.point(0), line.point(1)])
      .collect();

    segments.sort_by_key(|point| (point.x, point.y));
    rows.push((net, segments));

    let mut vias: Vec<Vec2> = world
      .all_items_in_net(root, net, Kind::VIA)
      .into_iter()
      .filter_map(|id| world.item(id).map(|item| item.anchor(0)))
      .collect();

    vias.sort_by_key(|point| (point.x, point.y));
    rows.push((net, vias));
  }

  rows
}

#[test]
fn a_shove_route_pushes_two_tracks_and_a_via_and_commits_clear() {
  let world = route_the_shove_board();
  let rules = rules();
  let root = world.root();

  // The route reached the far pad.
  let routed = stored_segments(&world, root, TRACE_NET);

  assert!(!routed.is_empty(), "nothing was committed");
  assert!(
    touches(&routed.iter().collect::<Vec<_>>(), SHOVE_START),
    "the committed route does not start on the start pad"
  );
  assert!(
    touches(&routed.iter().collect::<Vec<_>>(), SHOVE_TARGET),
    "the committed route does not reach the target pad"
  );

  // The obstacles ended up somewhere else, and everything is clear.
  assert_ne!(
    track_shape(&world, root, NEAR_TRACK_NET),
    NEAR_TRACK.to_vec()
  );
  assert_ne!(track_shape(&world, root, FAR_TRACK_NET), FAR_TRACK.to_vec());
  assert_ne!(via_position(&world, root, SHOVE_VIA_NET), Some(SHOVE_VIA));
  // The layer 1 track was never in anything's way.
  assert_eq!(
    track_shape(&world, root, OTHER_LAYER_TRACK_NET),
    OTHER_LAYER_TRACK.to_vec()
  );
  assert_shove_node_is_clear(&world, root, &rules);
}

#[test]
fn the_shove_route_is_the_same_twice() {
  let first = committed_shove_board(&route_the_shove_board());
  let second = committed_shove_board(&route_the_shove_board());

  assert_eq!(first, second, "two identical shove runs disagreed");
}
