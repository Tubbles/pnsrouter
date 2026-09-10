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
//! Only the mark obstacles path exists yet, so every scenario runs in
//! [`RouterMode::MarkObstacles`]. The walkaround and shove drags fall
//! back to it, which is why no scenario here sets another mode: it would
//! assert on a stub.

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
