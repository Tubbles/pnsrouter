// SPDX-License-Identifier: GPL-3.0-or-later

//! Dragger scenarios on a two layer board, through the public API only.
//!
//! `doc/work/009-dragging.md` and note
//! `doc/reference/kicad/06-dragger.md` section 10.2 step 3 ask for the
//! mark obstacles drag to be exercised over a board with a real stored
//! trace rather than only in unit tests. The board is built out of plain
//! data the way `tests/placer.rs` and `tests/shove.rs` build theirs; the
//! integration tests here each own their fixture, because
//! `tests/support/` is the KiCad fixture reader and nothing else.
//!
//! All three drag routines are here, for a segment, a corner and a via,
//! so a scenario picks the [`RouterMode`] its assertion is about. Free
//! angle mode has its own scenario at the end, because it bypasses the
//! mode entirely (note 06 section 2.17).

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::dragger::{DragMode, Dragger};
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{
  ItemBody, ItemId, LayerRange, MarkerFlags, NetId, Segment, Solid, Via,
  ViaType,
};
use pnsrouter::node::{NodeId, World};
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RouterMode, RoutingSettings};

/// The clearance every scenario drags to, in nanometres.
const CLEARANCE: i32 = 100_000;

/// The width of the trace every scenario drags.
const TRACK_WIDTH: i32 = 200_000;

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 400_000;

/// The net the dragged trace is on.
const TRACE_NET: Option<NetId> = Some(NetId(1));

/// The net of everything the trace is not allowed to touch.
const OBSTACLE_NET: Option<NetId> = Some(NetId(2));

/// The corners of the dragged trace, in order.
///
/// Three segments, all due east, with a middle one long enough to survive
/// a sideways drag without collapsing into a corner.
const TRACE: [Vec2; 4] = [
  Vec2::new(0, 0),
  Vec2::new(1_000_000, 0),
  Vec2::new(3_000_000, 0),
  Vec2::new(4_000_000, 0),
];

/// Where the pad the drag can run into sits, on layer 0.
const OBSTACLE: Vec2 = Vec2::new(2_000_000, -1_500_000);

/// Where the via a via drag starts on sits, well clear of the trace.
const VIA: Vec2 = Vec2::new(8_000_000, 0);

/// The diameter of that via.
const VIA_DIAMETER: i32 = 600_000;

/// Its drill.
const VIA_DRILL: i32 = 300_000;

/// Where the pad a drag cannot start on sits, on layer 0.
const PAD: Vec2 = Vec2::new(10_000_000, 0);

/// What the fixture board holds, by handle.
struct Board {
  /// The three segments of the trace, in [`TRACE`] order.
  trace: [ItemId; 3],
  /// The through via, on the obstacle net.
  via: ItemId,
  /// A pad, which is what a drag has to refuse to start on.
  pad: ItemId,
}

/// A two layer board with one three segment trace, one pad in the way,
/// one through via and one pad on nothing in particular.
fn build() -> (World, Board) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let mut trace = Vec::new();

  for pair in TRACE.windows(2) {
    let seg = Seg::new(pair[0], pair[1]);
    let body = ItemBody::Segment(Segment::new(seg, TRACK_WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(TRACE_NET);

    trace.push(
      world
        .add_segment(root, item, false)
        .expect("a board track is neither degenerate nor redundant"),
    );
  }

  let add_pad = |world: &mut World, at: Vec2, layer: i32, net| {
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(net);

    world.add_solid(root, item, None)
  };

  add_pad(&mut world, OBSTACLE, 0, OBSTACLE_NET);

  let pad = add_pad(&mut world, PAD, 0, OBSTACLE_NET);
  let body =
    ItemBody::Via(Via::new(VIA, VIA_DIAMETER, VIA_DRILL, ViaType::Through));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::new(0, 1));
  item.set_net(OBSTACLE_NET);

  let via = world.add_via(root, item);

  (
    world,
    Board {
      trace: [trace[0], trace[1], trace[2]],
      via,
      pad,
    },
  )
}

/// The rule oracle every scenario uses.
fn rules() -> FixedClearance {
  FixedClearance::uniform(CLEARANCE)
}

/// Settings in one routing mode, with everything else at KiCad's
/// defaults, which includes `smooth_dragged_segments`.
fn settings_for(mode: RouterMode) -> RoutingSettings {
  RoutingSettings {
    mode,
    ..RoutingSettings::default()
  }
}

/// The segments of a node's delta, as geometry, in the order
/// `World::get_updated_items` answers.
fn delta_segments(world: &World, node: NodeId) -> (Vec<Seg>, Vec<Seg>) {
  let (added, removed) = world.get_updated_items(node);
  let seg_of = |ids: Vec<ItemId>| -> Vec<Seg> {
    ids
      .into_iter()
      .filter_map(|id| match world.item(id)?.body() {
        ItemBody::Segment(body) => Some(body.seg()),
        _ => None,
      })
      .collect()
  };

  (seg_of(added), seg_of(removed))
}

/// Whether every segment of a chain of points sits on the 45 degree grid.
fn is_on_the_grid(points: &[Vec2]) -> bool {
  points.windows(2).all(|pair| {
    let delta = pair[1] - pair[0];

    delta.x == 0 || delta.y == 0 || delta.x.abs() == delta.y.abs()
  })
}

// ---------------------------------------------------------------------
// The mode decision
// ---------------------------------------------------------------------

#[test]
fn a_click_in_the_middle_of_a_segment_starts_a_segment_drag() {
  // `pns_dragger.cpp:128`: neither end is within `width / 2`, so the
  // `else` at `:148` wins.
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = Dragger::new(&world, world.root());

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(2_000_000, 0),
    board.trace[1]
  ));
  assert_eq!(dragger.mode(), DragMode::Segment);
  assert_eq!(dragger.original_line().point_count(), TRACE.len());
}

#[test]
fn a_click_on_a_corner_starts_a_corner_drag() {
  // `:128` again, from the other side: the click is exactly on the far
  // end of the first segment, so `:132` moves the index onto that point.
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = Dragger::new(&world, world.root());

  assert!(dragger.start(&mut world, &context, TRACE[1], board.trace[0]));
  assert_eq!(dragger.mode(), DragMode::Corner);
}

#[test]
fn a_start_on_a_via_reports_the_via_mode() {
  // `:349` to `startDragVia` (`:257`), which cannot fail.
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = Dragger::new(&world, world.root());

  assert!(dragger.start(&mut world, &context, VIA, board.via));
  assert_eq!(dragger.mode(), DragMode::Via);
  assert_eq!(dragger.current_nets(), OBSTACLE_NET);
}

#[test]
fn a_start_on_a_pad_refuses() {
  // A `SOLID_T` start item falls into the `default:` at `:355`, which is
  // how a single pad handed to a `DRAGGER` bows out.
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = Dragger::new(&world, world.root());

  assert!(!dragger.start(&mut world, &context, PAD, board.pad));
  assert_eq!(dragger.current_node(), world.root());
  assert!(dragger.traces().is_empty());
}

#[test]
fn a_start_on_an_item_that_is_gone_refuses() {
  // KiCad refuses an empty primitive set at `:306`; a stale handle is
  // the same nothing, and C++ would have dereferenced it.
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();

  world.remove(root, board.trace[1]);

  let mut dragger = Dragger::new(&world, root);

  assert!(!dragger.start(
    &mut world,
    &context,
    Vec2::new(2_000_000, 0),
    board.trace[1]
  ));
}

#[test]
fn a_start_on_a_locked_segment_clears_the_lock_instead_of_refusing() {
  // `startItem->Unmark( MK_LOCKED )` at `:331` mutates the world item and
  // is never undone. Refusing a locked item is the host's job: KiCad
  // shows the confirmation dialog in `ROUTER_TOOL::performDragging`
  // (`pcbnew/router/router_tool.cpp:2525`) before it gets here.
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();

  world
    .item_mut(board.trace[1])
    .expect("the segment is stored")
    .mark(MarkerFlags::LOCKED);

  let mut dragger = Dragger::new(&world, root);

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(2_000_000, 0),
    board.trace[1]
  ));
  assert!(
    !world
      .item(board.trace[1])
      .expect("the segment is still stored")
      .is_locked(),
    "the drag left the lock in place"
  );
}

// ---------------------------------------------------------------------
// The mark obstacles drag
// ---------------------------------------------------------------------

#[test]
fn a_segment_drag_moves_the_middle_segment_and_reports_the_node_delta() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(2_000_000, 0),
    board.trace[1]
  ));
  // Before the first drag `CurrentNode()` is the untouched board
  // (`:1052`).
  assert_eq!(dragger.current_node(), root);

  assert!(dragger.drag(&mut world, &context, Vec2::new(2_000_000, -500_000)));

  let node = dragger.current_node();

  assert_ne!(node, root);

  let (added, removed) = delta_segments(&world, node);

  // The whole assembled line goes, links and all, and the re-dragged one
  // takes its place: `m_lastNode->Remove( origLine ); Add( dragged )`
  // (`:409`, `:410`).
  assert_eq!(
    removed,
    vec![
      Seg::new(TRACE[0], TRACE[1]),
      Seg::new(TRACE[1], TRACE[2]),
      Seg::new(TRACE[2], TRACE[3]),
    ]
  );
  assert_eq!(
    added,
    vec![
      Seg::new(Vec2::new(0, 0), Vec2::new(1_000_000, 0)),
      Seg::new(Vec2::new(1_000_000, 0), Vec2::new(1_500_000, -500_000)),
      Seg::new(
        Vec2::new(1_500_000, -500_000),
        Vec2::new(2_500_000, -500_000)
      ),
      Seg::new(Vec2::new(2_500_000, -500_000), Vec2::new(3_000_000, 0)),
      Seg::new(Vec2::new(3_000_000, 0), Vec2::new(4_000_000, 0)),
    ]
  );

  // `Traces()` is the one dragged line (`:413`), and the drag is clear of
  // the obstacle pad, so `m_dragStatus` is true (`:446`).
  assert_eq!(dragger.traces().len(), 1);
  assert_eq!(dragger.traces()[0].point(0), TRACE[0]);
  assert_eq!(dragger.traces()[0].last_point(), Some(TRACE[3]));
  assert!(is_on_the_grid(dragger.traces()[0].shape().points()));
  assert_eq!(dragger.force_mark_obstacles_mode(), (false, true));
}

#[test]
fn a_segment_drag_into_a_pad_still_succeeds_but_reports_the_collision() {
  // `dragMarkObstacles` returns true unconditionally (`:448`); the
  // collision reaches the host through `m_dragStatus` (`:446`), which is
  // what "mark obstacles" means.
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(2_000_000, 0),
    board.trace[1]
  ));
  assert!(dragger.drag(
    &mut world,
    &context,
    Vec2::new(2_000_000, OBSTACLE.y + 200_000)
  ));
  assert_eq!(dragger.force_mark_obstacles_mode(), (false, false));

  // A colliding mark obstacles drag refuses to commit, and `force_commit`
  // does not help, because the forced fallback never latched: note 06
  // erratum E7.
  assert!(!dragger.fix_route(&mut world, &context, true));
}

#[test]
fn a_corner_drag_moves_one_corner_and_leaves_the_two_ends_alone() {
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);
  let target = Vec2::new(1_200_000, -800_000);

  assert!(dragger.start(&mut world, &context, TRACE[1], board.trace[0]));
  assert_eq!(dragger.mode(), DragMode::Corner);
  assert!(dragger.drag(&mut world, &context, target));

  let dragged = &dragger.traces()[0];
  let points = dragged.shape().points();

  assert_eq!(points.first(), Some(&TRACE[0]));
  assert_eq!(points.last(), Some(&TRACE[3]));
  assert!(is_on_the_grid(points));
  assert!(
    points.iter().any(|point| point.y == target.y),
    "no corner reached the height the drag asked for: {points:?}"
  );

  let (added, removed) = delta_segments(&world, dragger.current_node());

  assert_eq!(removed.len(), 3);
  assert!(
    added.len() > 3,
    "the corner drag added {} segments",
    added.len()
  );
}

#[test]
fn a_clear_drag_commits_and_leaves_the_new_geometry_in_the_root() {
  // `FixRoute`'s `m_dragStatus` branch (`:963`), which is
  // `Router()->CommitRouting( node )` and here `World::commit`.
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(2_000_000, 0),
    board.trace[1]
  ));
  assert!(dragger.drag(&mut world, &context, Vec2::new(2_000_000, -500_000)));
  assert!(dragger.fix_route(&mut world, &context, false));

  let stored: Vec<Seg> = world
    .all_items_in_net(root, TRACE_NET, pnsrouter::item::Kind::SEGMENT)
    .into_iter()
    .filter_map(|id| match world.item(id)?.body() {
      ItemBody::Segment(body) => Some(body.seg()),
      _ => None,
    })
    .collect();

  assert_eq!(stored.len(), 5, "the committed trace is {stored:?}");
  assert!(
    stored
      .iter()
      .any(|seg| seg.a.y == -500_000 && seg.b.y == -500_000),
    "the dragged segment did not reach the root"
  );
}

#[test]
fn two_identical_drags_answer_identically() {
  // `DESIGN.md` section 8: the engine is a pure function of the world,
  // the settings and the events.
  let run = || {
    let (mut world, board) = build();
    let rules = rules();
    let settings = settings_for(RouterMode::MarkObstacles);
    let context = AlgoContext::new(&rules, &settings);
    let mut dragger = Dragger::new(&world, world.root());

    dragger.start(
      &mut world,
      &context,
      Vec2::new(2_000_000, 0),
      board.trace[1],
    );

    for step in 1..6 {
      dragger.drag(&mut world, &context, Vec2::new(2_000_000, -100_000 * step));
    }

    let node = dragger.current_node();

    (
      delta_segments(&world, node),
      dragger.force_mark_obstacles_mode(),
    )
  };

  assert_eq!(run(), run());
}
// ---------------------------------------------------------------------
// The via drag
// ---------------------------------------------------------------------

/// Where the via a fanout drag starts on sits.
const FANOUT_VIA: Vec2 = Vec2::new(0, 0);

/// The net the fanout via and both of its traces are on.
const FANOUT_NET: Option<NetId> = Some(NetId(3));

/// The corners of the trace that leaves the fanout via eastwards on layer
/// 0.
const FANOUT_EAST: [Vec2; 3] = [
  Vec2::new(0, 0),
  Vec2::new(2_000_000, 0),
  Vec2::new(4_000_000, 0),
];

/// The corners of the trace that leaves it northwards on layer 1.
const FANOUT_NORTH: [Vec2; 3] = [
  Vec2::new(0, 0),
  Vec2::new(0, -2_000_000),
  Vec2::new(0, -4_000_000),
];

/// Add one track segment to the root.
fn add_track(
  world: &mut World,
  seg: Seg,
  layer: i32,
  net: Option<NetId>,
) -> ItemId {
  let root = world.root();
  let body = ItemBody::Segment(Segment::new(seg, TRACK_WIDTH));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::single(layer));
  item.set_net(net);

  world
    .add_segment(root, item, false)
    .expect("a test track is neither degenerate nor redundant")
}

/// Add one round pad to the root.
fn add_pad(
  world: &mut World,
  at: Vec2,
  radius: i32,
  layer: i32,
  net: Option<NetId>,
) -> ItemId {
  let root = world.root();
  let body = ItemBody::Solid(Solid::new(Shape::circle(at, radius), at));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::single(layer));
  item.set_net(net);

  world.add_solid(root, item, None)
}

/// A two layer board with one through via and one trace leaving it on
/// each layer.
///
/// The joint at the via therefore has three links, a via and two
/// segments, which is what `findViaFanoutByHandle`
/// (`pcbnew/router/pns_dragger.cpp:267`) walks and what stops
/// `AssembleLine` from running one trace into the other.
fn build_via_fanout() -> (World, ItemId) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let body = ItemBody::Via(Via::new(
    FANOUT_VIA,
    VIA_DIAMETER,
    VIA_DRILL,
    ViaType::Through,
  ));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::new(0, 1));
  item.set_net(FANOUT_NET);

  let via = world.add_via(root, item);

  for pair in FANOUT_EAST.windows(2) {
    add_track(&mut world, Seg::new(pair[0], pair[1]), 0, FANOUT_NET);
  }

  for pair in FANOUT_NORTH.windows(2) {
    add_track(&mut world, Seg::new(pair[0], pair[1]), 1, FANOUT_NET);
  }

  (world, via)
}

/// Where a via item sits, by handle.
fn via_position(world: &World, id: ItemId) -> Option<Vec2> {
  match world.item(id)?.body() {
    ItemBody::Via(body) => Some(body.pos()),
    _ => None,
  }
}

/// The positions of the vias a node's delta adds and removes.
fn delta_vias(world: &World, node: NodeId) -> (Vec<Vec2>, Vec<Vec2>) {
  let (added, removed) = world.get_updated_items(node);
  let vias = |ids: Vec<ItemId>| -> Vec<Vec2> {
    ids
      .into_iter()
      .filter_map(|id| via_position(world, id))
      .collect()
  };

  (vias(added), vias(removed))
}

/// The trace of `traces()` that starts nearest a point.
fn trace_from(dragger: &Dragger, at: Vec2) -> &pnsrouter::line::Line {
  dragger
    .traces()
    .iter()
    .find(|line| line.point(0) == at)
    .unwrap_or_else(|| {
      panic!(
        "no dragged trace starts at {at:?}; they start at {:?}",
        dragger
          .traces()
          .iter()
          .map(|line| line.point(0))
          .collect::<Vec<_>>()
      )
    })
}

#[test]
fn a_via_drag_in_mark_obstacles_mode_moves_the_via_and_both_trace_ends() {
  // `dragViaMarkObstacles` (`pns_dragger.cpp:452`): no forces and no
  // walkaround, every fanout line's near corner follows the cursor and
  // the via is replaced by a copy at the cursor.
  let (mut world, via) = build_via_fanout();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);
  let target = Vec2::new(1_000_000, -1_000_000);

  assert!(dragger.start(&mut world, &context, FANOUT_VIA, via));
  assert_eq!(dragger.mode(), DragMode::Via);
  assert!(dragger.drag(&mut world, &context, target));

  let node = dragger.current_node();

  // The via itself, `:478` to `:484`. One entry, because
  // `findViaFanoutByHandle` keeps at most one via per joint (`:293`).
  assert_eq!(dragger.traces_vias().len(), 1);
  assert_eq!(via_position(&world, dragger.traces_vias()[0]), Some(target));

  let (added_vias, removed_vias) = delta_vias(&world, node);

  assert_eq!(added_vias, vec![target]);
  assert_eq!(removed_vias, vec![FANOUT_VIA]);

  // Both traces, `:462` to `:474`. Each starts at the cursor because
  // `findViaFanoutByHandle` reversed it to start at the via (`:286`), and
  // each keeps its far end.
  assert_eq!(dragger.traces().len(), 2);

  let east = trace_from(&dragger, target);

  assert_eq!(east.layer(), 0);
  assert_eq!(east.last_point(), Some(FANOUT_EAST[2]));

  let north = dragger
    .traces()
    .iter()
    .find(|line| line.layer() == 1)
    .expect("the northward trace was dragged too");

  assert_eq!(north.point(0), target);
  assert_eq!(north.last_point(), Some(FANOUT_NORTH[2]));

  for line in dragger.traces() {
    assert!(
      is_on_the_grid(line.shape().points()),
      "a dragged fanout line left the 45 degree regime: {:?}",
      line.shape().points()
    );
  }
}

#[test]
fn a_via_drag_in_walkaround_mode_bends_a_fanout_trace_around_a_pad() {
  // `dragViaWalkaround` (`:492`): the via first, through
  // `propagateViaForces`, then every fanout line, and the ones that
  // collide go through `tryWalkaround` and the post drag optimizer.
  let (mut world, via) = build_via_fanout();

  // On layer 0 only, so it is the eastward trace that has to bend.
  add_pad(
    &mut world,
    Vec2::new(1_400_000, -600_000),
    500_000,
    0,
    OBSTACLE_NET,
  );

  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);
  let target = Vec2::new(0, -1_400_000);

  assert!(dragger.start(&mut world, &context, FANOUT_VIA, via));
  assert!(dragger.drag(&mut world, &context, target));

  let node = dragger.current_node();
  let (added_vias, removed_vias) = delta_vias(&world, node);

  // The force propagation had nothing to push against, so the via
  // reached the cursor exactly (`:519`, `:523`).
  assert_eq!(added_vias, vec![target]);
  assert_eq!(removed_vias, vec![FANOUT_VIA]);

  // The eastward trace now starts at the via's new position and reaches
  // its old far end without touching the pad.
  let (added, _) = delta_segments(&world, node);

  assert!(
    added.iter().any(|seg| seg.a == target || seg.b == target),
    "nothing in the delta starts at the via's new position: {added:?}"
  );
  assert!(
    added.len() > FANOUT_EAST.len() + FANOUT_NORTH.len() - 2,
    "the walk added no detour: {added:?}"
  );
}

#[test]
fn a_via_drag_in_walkaround_mode_under_reports_its_traces() {
  // Note 06 erratum E5, `pns_dragger.cpp:617`.
  //
  // `dragViaWalkaround` adds the dragged via to the set at `:511` and
  // every clear fanout line at `:557`, and then the **first** line that
  // needs a walkaround reaches `optimizeAndUpdateDraggedLine`, which
  // clears the whole set and puts only its own optimized line back
  // (`:617`, `:618`). So `Traces()` reports one line and no via at all,
  // although the via moved and two lines were dragged, which is what
  // makes `ROUTER::markViolations` stop skipping the dragged via
  // (`pcbnew/router/pns_router.cpp:726`) and draw it as colliding with
  // itself. Transcribed rather than repaired: the note's proposal, to add
  // rather than replace, is a behaviour change with no fixture behind it.
  let (mut world, via) = build_via_fanout();

  add_pad(
    &mut world,
    Vec2::new(1_400_000, -600_000),
    500_000,
    0,
    OBSTACLE_NET,
  );

  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);
  let target = Vec2::new(0, -1_400_000);

  assert!(dragger.start(&mut world, &context, FANOUT_VIA, via));
  assert!(dragger.drag(&mut world, &context, target));

  // The node still holds all of it.
  let (added_vias, _) = delta_vias(&world, dragger.current_node());

  assert_eq!(added_vias, vec![target]);

  // The set does not. The joint's links are in insertion order, so the
  // fanout is the via, then the eastward trace, then the northward one:
  // the via and the walked line are added first and the optimizer wipes
  // both, and only the trace dragged **after** it survives beside the
  // optimized one.
  assert!(
    dragger.traces_vias().is_empty(),
    "the dragged via survived the optimizer, so erratum E5 is gone"
  );
  assert_eq!(
    dragger.traces().len(),
    2,
    "the set holds neither everything that moved nor only the optimized \
     line: {:?}",
    dragger
      .traces()
      .iter()
      .map(|line| line.point(0))
      .collect::<Vec<_>>()
  );
}

#[test]
fn a_via_drag_in_shove_mode_pushes_a_track_out_of_the_way() {
  // `dragShove`'s `DM_VIA` case (`:908`): the via goes in as a shove head
  // under `SHP_SHOVE` alone and the engine does the rest.
  let (mut world, via) = build_via_fanout();
  // A track on another net across the via's path, close enough that the
  // via cannot sit where the cursor asks without moving it.
  let pushed = add_track(
    &mut world,
    Seg::new(
      Vec2::new(-2_000_000, -1_400_000),
      Vec2::new(2_000_000, -1_400_000),
    ),
    0,
    OBSTACLE_NET,
  );
  let rules = rules();
  let settings = settings_for(RouterMode::Shove);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);
  let target = Vec2::new(0, -1_200_000);

  assert!(dragger.start(&mut world, &context, FANOUT_VIA, via));
  assert!(dragger.drag(&mut world, &context, target));

  let node = dragger.current_node();
  let (added_vias, removed_vias) = delta_vias(&world, node);

  assert_eq!(removed_vias, vec![FANOUT_VIA]);
  assert_eq!(added_vias.len(), 1);
  assert_ne!(added_vias[0], FANOUT_VIA);

  // A successful via shove clears the set and puts nothing back
  // (`:941`, and `:946` adds nothing), so the answer is read off the
  // node.
  assert!(dragger.traces().is_empty());
  assert!(dragger.traces_vias().is_empty());

  let (_, removed) = delta_segments(&world, node);
  let pushed_seg = match world.item(pushed).map(pnsrouter::item::Item::body) {
    Some(ItemBody::Segment(body)) => body.seg(),
    _ => panic!("the obstacle track is a segment"),
  };

  assert!(
    removed.contains(&pushed_seg),
    "the shove left the obstacle track where it was: {removed:?}"
  );
}

#[test]
fn a_via_drag_commits_the_moved_via_and_its_fanout() {
  // `FixRoute`'s `m_dragStatus` branch (`:963`) does not care which of
  // the three gestures produced the node, so a via drag commits through
  // exactly the same path a segment drag does.
  let (mut world, via) = build_via_fanout();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);
  let target = Vec2::new(1_000_000, -1_000_000);

  assert!(dragger.start(&mut world, &context, FANOUT_VIA, via));
  assert!(dragger.drag(&mut world, &context, target));
  assert!(dragger.fix_route(&mut world, &context, false));

  // Exactly one via on the net is left in the root, at the cursor.
  let moved = world
    .all_items_in_net(root, FANOUT_NET, pnsrouter::item::Kind::VIA)
    .into_iter()
    .filter_map(|id| via_position(&world, id))
    .collect::<Vec<_>>();

  assert_eq!(moved, vec![target]);

  // And both traces reach it.
  let ends: Vec<Seg> = world
    .all_items_in_net(root, FANOUT_NET, pnsrouter::item::Kind::SEGMENT)
    .into_iter()
    .filter_map(|id| match world.item(id)?.body() {
      ItemBody::Segment(body) => Some(body.seg()),
      _ => None,
    })
    .filter(|seg| seg.a == target || seg.b == target)
    .collect();

  assert_eq!(
    ends.len(),
    2,
    "the committed board has {} track ends at the via: {ends:?}",
    ends.len()
  );
}

// ---------------------------------------------------------------------
// Free angle mode
// ---------------------------------------------------------------------

#[test]
fn a_free_angle_drag_is_a_corner_drag_that_follows_the_cursor_exactly() {
  // Note 06 section 2.17, end to end: `Start` latches the flag and does
  // **not** build a shove (`:314`, `:323`), `startDragSegment` forces
  // `DM_CORNER` even for a mid segment click (`:135` to `:145`), `Drag`
  // routes to `dragMarkObstacles` before the mode switch is reached
  // (`:1005`), and `dragMarkObstacles` passes the flag to `DragCorner`
  // (`:407`), which is `dragCornerFree`: set the point, simplify, no 45
  // degree rebuild and no optimizer.
  //
  // The routing mode is [`RouterMode::Shove`] here on purpose: free angle
  // has to win over it.
  let (mut world, board) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::Shove);
  let context = AlgoContext::new(&rules, &settings);
  let root = world.root();
  let mut dragger = Dragger::new(&world, root);
  let target = Vec2::new(2_100_000, -700_000);

  dragger.set_free_angle_mode(true);

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(2_000_000, 0),
    board.trace[1]
  ));
  // A mid segment click is a corner drag in free angle mode, where the
  // same click without it is a segment drag.
  assert_eq!(dragger.mode(), DragMode::Corner);

  assert!(dragger.drag(&mut world, &context, target));

  let points = dragger.traces()[0].shape().points().to_vec();

  assert_eq!(points.first(), Some(&TRACE[0]));
  assert_eq!(points.last(), Some(&TRACE[3]));
  assert!(
    points.contains(&target),
    "the drag did not put a corner on the cursor: {points:?}"
  );
  assert!(
    !is_on_the_grid(&points),
    "a free angle drag stayed on the 45 degree grid: {points:?}"
  );

  // The same click and the same cursor without free angle mode stays on
  // the grid and never puts a corner on the cursor.
  let (mut world, board) = build();
  let mut dragger = Dragger::new(&world, world.root());

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(2_000_000, 0),
    board.trace[1]
  ));
  assert_eq!(dragger.mode(), DragMode::Segment);
  assert!(dragger.drag(&mut world, &context, target));
  assert!(is_on_the_grid(dragger.traces()[0].shape().points()));
}
