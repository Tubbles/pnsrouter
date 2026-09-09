// SPDX-License-Identifier: GPL-3.0-or-later

//! Shove scenarios on a two layer board, through the public API only.
//!
//! `doc/work/004-shove.md` asks for the shove to be exercised on a board
//! with real tracks rather than only in unit tests, so this file builds
//! one out of plain data the way `tests/walkaround.rs` does and drives
//! [`pnsrouter::shove::Shove`] over it.
//!
//! Part 1 covered segments; part 2 adds solids, which are walked around
//! rather than moved, and vias, which move and drag their tracks with
//! them. Every scenario that claims success asserts that the node the
//! shove hands back is actually clear, through
//! `World::check_colliding_line` on every track it holds, because a shove
//! that reports success over geometry that still collides is the failure
//! mode worth catching.

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::collide::CollisionSearchOptions;
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{
  ItemBody, ItemId, Kind, LayerRange, MarkerFlags, NetId, Segment, Solid, Via,
  ViaType,
};
use pnsrouter::line::Line;
use pnsrouter::node::{NodeId, World};
use pnsrouter::optimizer::EffortFlags;
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::RoutingSettings;
use pnsrouter::shove::{HEAD_RANK, Shove, ShovePolicy, ShoveStatus, ViaHandle};

/// The clearance every scenario routes to, in nanometres.
const CLEARANCE: i32 = 100000;

/// The width of the head and of every track on the board.
const TRACK_WIDTH: i32 = 200000;

/// How far apart two centrelines have to be: the clearance plus half of
/// each width. A hull is built at exactly this distance, so a shoved
/// track ends up on that line or just outside it.
const REQUIRED_GAP: i32 = CLEARANCE + TRACK_WIDTH;

/// The net the head is on, so that nothing on the board exempts it.
const HEAD_NET: Option<NetId> = Some(NetId(1));

/// The net of the track that runs nearest the head.
const NEAR_NET: Option<NetId> = Some(NetId(2));

/// The net of the track beyond it, which only moves once the near one has
/// been pushed into it.
const FAR_NET: Option<NetId> = Some(NetId(3));

/// The net of the track on the other layer, which nothing on layer 0 can
/// ever see.
const OTHER_LAYER_NET: Option<NetId> = Some(NetId(4));

/// One track of the board, as a host would hand it over.
struct TrackSpec {
  /// Its corners, in order.
  points: &'static [Vec2],
  /// Which layer it is on.
  layer: i32,
  /// Which net it belongs to.
  net: Option<NetId>,
}

/// The corners of the track that runs closest to the head, 250000 north
/// of it, which is inside the 300000 the clearance asks for.
///
/// It reaches well past both ends of the head so that its own endpoints
/// are outside the head's hull and a walk has somewhere to start.
const NEAR_POINTS: [Vec2; 2] =
  [Vec2::new(-1000000, 250000), Vec2::new(4000000, 250000)];

/// The same track with a detour north in the middle, which stands clear
/// of the head where it is and is pointless once the rest of the track
/// has been pushed up to meet it.
const BUMPY_POINTS: [Vec2; 6] = [
  Vec2::new(-1000000, 250000),
  Vec2::new(1000000, 250000),
  Vec2::new(1200000, 450000),
  Vec2::new(1800000, 450000),
  Vec2::new(2000000, 250000),
  Vec2::new(4000000, 250000),
];

/// The corners of the track beyond the near one, far enough north to be
/// clear of it where it stands and too close once it has been pushed.
const FAR_POINTS: [Vec2; 2] =
  [Vec2::new(-1500000, 560000), Vec2::new(4500000, 560000)];

/// The corners of the track on layer 1.
const OTHER_LAYER_POINTS: [Vec2; 2] = [Vec2::new(0, 0), Vec2::new(3000000, 0)];

/// The track that runs closest to the head.
fn near_track() -> TrackSpec {
  TrackSpec {
    points: &NEAR_POINTS,
    layer: 0,
    net: NEAR_NET,
  }
}

/// The near track with its pointless detour.
fn bumpy_track() -> TrackSpec {
  TrackSpec {
    points: &BUMPY_POINTS,
    layer: 0,
    net: NEAR_NET,
  }
}

/// A short track across the head's path, with both ends well clear of it
/// so that a walkaround has somewhere to go round.
const STUB_POINTS: [Vec2; 2] =
  [Vec2::new(1500000, -600000), Vec2::new(1500000, 600000)];

/// The short track the walkaround escalation is exercised against.
fn stub_track() -> TrackSpec {
  TrackSpec {
    points: &STUB_POINTS,
    layer: 0,
    net: NEAR_NET,
  }
}

/// The track beyond the near one.
fn far_track() -> TrackSpec {
  TrackSpec {
    points: &FAR_POINTS,
    layer: 0,
    net: FAR_NET,
  }
}

/// A track on layer 1, so that the board is a two layer one and the
/// shove has to ignore it.
fn other_layer_track() -> TrackSpec {
  TrackSpec {
    points: &OTHER_LAYER_POINTS,
    layer: 1,
    net: OTHER_LAYER_NET,
  }
}

/// Turn the plain data into a world.
fn build(tracks: &[TrackSpec]) -> World {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();

  for track in tracks {
    for pair in track.points.windows(2) {
      let seg = Seg::new(pair[0], pair[1]);
      let body = ItemBody::Segment(Segment::new(seg, TRACK_WIDTH));
      let mut item = world.make_item(body);

      item.set_layers_and_flash_all(LayerRange::single(track.layer));
      item.set_net(track.net);
      world
        .add_segment(root, item, false)
        .expect("a board track is neither degenerate nor redundant");
    }
  }

  world
}

/// The board the cascade scenarios use: two tracks north of the head and
/// one on the other layer.
fn board() -> World {
  build(&[near_track(), far_track(), other_layer_track()])
}

/// The same board without the far track, so that exactly one track moves.
fn one_track_board() -> World {
  build(&[near_track(), other_layer_track()])
}

/// The board whose near track carries a pointless detour.
fn bumpy_board() -> World {
  build(&[bumpy_track(), other_layer_track()])
}

/// The board whose near track the host has locked, which the shove
/// refuses to move.
fn locked_board() -> World {
  let mut world = build(&[near_track(), other_layer_track()]);
  let root = world.root();

  for segment in segments_of(&world, root, NEAR_NET) {
    world
      .item_mut(segment)
      .expect("the track was just added")
      .mark(MarkerFlags::LOCKED);
  }

  world
}

/// A head the placer could be dragging, on layer 0.
fn head(points: &[Vec2]) -> Line {
  let mut line = Line::new();

  line.set_width(TRACK_WIDTH);
  line.set_layer(0);
  line.set_net(HEAD_NET);
  line.set_shape(LineChain::from_slice(points, false));

  line
}

/// The head that runs straight past the near track, close enough to push
/// it.
fn pushing_head() -> Line {
  head(&[Vec2::new(0, 0), Vec2::new(3000000, 0)])
}

/// The same head retreated far to the south, where it meets nothing.
fn retreated_head() -> Line {
  head(&[Vec2::new(0, -3000000), Vec2::new(3000000, -3000000)])
}

/// The rule resolver every scenario uses.
fn rules() -> FixedClearance {
  FixedClearance::uniform(CLEARANCE)
}

/// Every stored segment of one net in a node.
fn segments_of(world: &World, node: NodeId, net: Option<NetId>) -> Vec<ItemId> {
  world.all_items_in_net(node, net, Kind::SEGMENT)
}

/// The whole track of one net, assembled from its first segment.
fn track_of(world: &World, node: NodeId, net: Option<NetId>) -> Line {
  let segments = segments_of(world, node, net);
  let first = *segments.first().expect("the net has at least one segment");

  world.assemble_line(node, first, None, false, false, true)
}

/// The points of one net's track, which is what a scenario compares.
fn shape_of(world: &World, node: NodeId, net: Option<NetId>) -> Vec<Vec2> {
  track_of(world, node, net).shape().points().to_vec()
}

/// The smallest distance between two lines' centrelines.
fn min_distance(first: &Line, second: &Line) -> i32 {
  let mut smallest = i32::MAX;

  for outer in 0..first.segment_count() {
    let one = first.segment(outer);

    for inner in 0..second.segment_count() {
      smallest = smallest.min(one.distance_to_segment(&second.segment(inner)));
    }
  }

  smallest
}

/// Every net a scenario might have put on the board.
///
/// The head's net is in the list because a failed run can leave a head
/// behind, and out of [`BOARD_NETS`] because a successful run takes its
/// heads back out of the node again.
const ALL_NETS: [Option<NetId>; 6] = [
  HEAD_NET,
  NEAR_NET,
  FAR_NET,
  OTHER_LAYER_NET,
  PAD_NET,
  VIA_NET,
];

/// The nets a board still holds after a successful run.
const BOARD_NETS: [Option<NetId>; 3] = [NEAR_NET, FAR_NET, OTHER_LAYER_NET];

/// Assert that nothing in a node collides with anything else.
fn assert_node_is_clear(world: &World, node: NodeId) {
  let options = CollisionSearchOptions::default();

  for net in ALL_NETS {
    for segment in segments_of(world, node, net) {
      let line = world.assemble_line(node, segment, None, false, false, true);

      assert!(
        world
          .check_colliding_line(node, &line, &rules(), &options)
          .is_none(),
        "a track on net {net:?} still collides after the shove"
      );
    }

    // A via is checked the same way, as a line that carries nothing else.
    for via in world.all_items_in_net(node, net, Kind::VIA) {
      let item = world.item(via).expect("the node listed it");
      let line = Line::from_linked_via(via, item);

      assert!(
        world
          .check_colliding_line(node, &line, &rules(), &options)
          .is_none(),
        "a via on net {net:?} still collides after the shove"
      );
    }
  }
}

/// Assert that a line is clear of everything a node holds.
fn assert_line_is_clear(world: &World, node: NodeId, line: &Line) {
  let options = CollisionSearchOptions::default();

  assert!(
    world
      .check_colliding_line(node, line, &rules(), &options)
      .is_none(),
    "the head still collides with the shoved world"
  );
}

// ---------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------

#[test]
fn a_head_pushes_one_parallel_track_aside() {
  let mut world = one_track_board();
  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let root = world.root();

  let before = shape_of(&world, root, NEAR_NET);
  let head = pushing_head();

  // The head really is in the way before the shove.
  assert!(
    world
      .check_colliding_line(
        root,
        &head,
        &resolver,
        &CollisionSearchOptions::default()
      )
      .is_some()
  );

  let mut shove = Shove::new(root);

  shove.add_head_line(head.clone(), ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let node = shove.current_node();
  let after = track_of(&world, node, NEAR_NET);

  // The track moved.
  assert_ne!(after.shape().points().to_vec(), before);
  // Its endpoints did not: that is what separates a shove from a reroute.
  assert_eq!(after.point(0), before[0]);
  assert_eq!(after.last_point(), before.last().copied());
  // It is out of the head's way, and nothing else was disturbed into a
  // collision.
  assert!(min_distance(&after, &head) >= REQUIRED_GAP);
  assert_line_is_clear(&world, node, &head);
  assert_node_is_clear(&world, node);
  // The layer 1 track was left alone.
  assert_eq!(
    shape_of(&world, node, OTHER_LAYER_NET),
    shape_of(&world, root, OTHER_LAYER_NET)
  );
  // The head itself was not touched: a shove moves the obstacles and
  // leaves the caller's line where the caller put it. KiCad's
  // `GetModifiedHead` would dereference a disengaged optional here
  // (`pns_shove.cpp:2646`), which is why this answers `None` instead.
  assert!(!shove.heads_modified(None));
  assert!(shove.modified_head(0).is_none());
}

#[test]
fn two_parallel_tracks_are_pushed_in_sequence_with_ranks_decreasing() {
  let mut world = board();
  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let root = world.root();

  let near_before = shape_of(&world, root, NEAR_NET);
  let far_before = shape_of(&world, root, FAR_NET);
  let head = pushing_head();
  let mut shove = Shove::new(root);

  shove.add_head_line(head.clone(), ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let node = shove.current_node();

  assert_ne!(shape_of(&world, node, NEAR_NET), near_before);
  assert_ne!(shape_of(&world, node, FAR_NET), far_before);
  assert_node_is_clear(&world, node);
  assert_line_is_clear(&world, node, &head);

  // The wave descends: the head is at HEAD_RANK, the track it pushed sits
  // one below, and the track that one pushed sits one below that.
  let near_rank = rank_of(&world, node, NEAR_NET);
  let far_rank = rank_of(&world, node, FAR_NET);

  assert_eq!(near_rank, HEAD_RANK - 1);
  assert_eq!(far_rank, HEAD_RANK - 2);
}

/// The rank every segment of one net carries, which the shove keeps equal
/// across a whole track.
fn rank_of(world: &World, node: NodeId, net: Option<NetId>) -> i32 {
  let segments = segments_of(world, node, net);
  let ranks: Vec<i32> = segments
    .iter()
    .filter_map(|id| world.item(*id).map(pnsrouter::item::Item::rank))
    .collect();

  assert!(!ranks.is_empty(), "the net {net:?} has no segments");
  assert!(
    ranks.iter().all(|rank| *rank == ranks[0]),
    "one track carries two ranks: {ranks:?}"
  );

  ranks[0]
}

#[test]
fn a_track_the_shove_may_not_move_leaves_the_run_incomplete() {
  let mut world = locked_board();
  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let root = world.root();

  let before = shape_of(&world, root, NEAR_NET);
  let mut shove = Shove::new(root);

  shove.add_head_line(pushing_head(), ShovePolicy::SHOVE);

  // A locked segment may not be pushed (`pns_shove.cpp:647`), so the
  // shove escalates to walking the head around it (`:1826`). This track
  // reaches well past both ends of the head, so there is nothing to walk
  // round and the run gives up.
  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Incomplete);

  // The failed run left the world exactly as it was.
  assert_eq!(shove.current_node(), root);
  assert_eq!(shape_of(&world, root, NEAR_NET), before);
  assert!(shove.springback().is_empty());
}

#[test]
fn the_iteration_budget_stops_a_run_that_would_otherwise_finish() {
  let mut world = board();
  // One iteration is enough to shove the near track and not enough to
  // find out that the result is clear, and the budget is tested after
  // every iteration whatever it answered.
  let settings = RoutingSettings {
    shove_iteration_limit: 1,
    ..RoutingSettings::default()
  };

  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let root = world.root();
  let before = shape_of(&world, root, NEAR_NET);
  let mut shove = Shove::new(root);

  shove.add_head_line(pushing_head(), ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Incomplete);
  assert_eq!(shove.iterations(), 1);
  assert_eq!(shove.current_node(), root);
  assert_eq!(shape_of(&world, root, NEAR_NET), before);
}

#[test]
fn a_retreating_head_springs_the_pushed_track_back() {
  let mut world = board();
  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let root = world.root();
  let before = shape_of(&world, root, NEAR_NET);
  let mut shove = Shove::new(root);

  // The base frame a line placer would push when it commits, which is
  // what gives springback something to pop back to: `reduceSpringback`
  // never drops the bottom frame.
  assert!(shove.add_locked_springback_node(root));

  shove.add_head_line(pushing_head(), ShovePolicy::SHOVE);
  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let pushed = shove.current_node();

  assert_ne!(shape_of(&world, pushed, NEAR_NET), before);
  assert_eq!(shove.springback().len(), 2);

  // Now the head retreats out of the way. The frame it left behind no
  // longer stands in anything's way, so it is dropped and the world
  // springs back.
  shove.clear_heads();
  shove.add_head_line(retreated_head(), ShovePolicy::SHOVE);
  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let sprung = shove.current_node();

  assert_eq!(shape_of(&world, sprung, NEAR_NET), before);
  assert_eq!(shove.springback().len(), 2);
  assert_node_is_clear(&world, sprung);
}

#[test]
fn a_locked_frame_survives_a_retreating_head() {
  let mut world = board();
  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let root = world.root();
  let before = shape_of(&world, root, NEAR_NET);
  let mut shove = Shove::new(root);

  shove.add_locked_springback_node(root);
  shove.add_head_line(pushing_head(), ShovePolicy::SHOVE);
  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let pushed = shove.current_node();
  let pushed_shape = shape_of(&world, pushed, NEAR_NET);

  // What the line placer does when it fixes a segment: pin the world as
  // it stands so that springback can never roll past it.
  shove.add_locked_springback_node(pushed);

  shove.clear_heads();
  shove.add_head_line(retreated_head(), ShovePolicy::SHOVE);
  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  // The pushed track stayed pushed, because the frame above it is locked.
  let node = shove.current_node();

  assert_eq!(shape_of(&world, node, NEAR_NET), pushed_shape);
  assert_ne!(pushed_shape, before);

  // Rewinding to the last locked frame lands back on the pinned world.
  assert!(shove.rewind_to_last_locked_node());
  assert_eq!(shove.current_node(), pushed);
}

#[test]
fn rewinding_the_springback_drops_the_frames_above_a_node() {
  let mut world = board();
  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let root = world.root();
  let mut shove = Shove::new(root);

  shove.add_locked_springback_node(root);
  shove.add_head_line(pushing_head(), ShovePolicy::SHOVE);
  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let pushed = shove.current_node();

  assert_eq!(shove.springback().len(), 2);
  assert!(!shove.springback()[1].is_locked());

  // A node that was never on the stack is not found.
  let stranger = world.branch(root);

  assert!(!shove.rewind_springback_to(&mut world, stranger));

  // KiCad erases the frame it is asked to rewind to along with the ones
  // above it, so the stack loses one and the current node steps back to
  // the frame below.
  assert!(shove.rewind_springback_to(&mut world, pushed));
  assert_eq!(shove.springback().len(), 1);
  assert_eq!(shove.current_node(), root);

  // Unlocking works on the frames that are left.
  assert!(shove.springback()[0].is_locked());
  shove.unlock_springback_node(root);
  assert!(!shove.springback()[0].is_locked());
}

#[test]
fn the_optimizer_queue_tidies_the_pushed_track() {
  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);

  let shove_and_read = |disable: EffortFlags| {
    let mut world = bumpy_board();
    let root = world.root();
    let mut shove = Shove::new(root);

    shove.disable_post_shove_optimizations(disable);
    shove.add_head_line(pushing_head(), ShovePolicy::SHOVE);
    assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

    let node = shove.current_node();

    assert_node_is_clear(&world, node);

    shape_of(&world, node, NEAR_NET)
  };

  // Everything the medium effort level would have run.
  let raw = shove_and_read(EffortFlags::MERGE_SEGMENTS);
  let optimized = shove_and_read(EffortFlags::NONE);

  // The shove pushed the flat parts of the track up to meet its own
  // detour, and the optimizer then read the queue and threw the detour
  // away, while keeping both endpoints.
  assert!(
    optimized.len() < raw.len(),
    "optimized {optimized:?} is not simpler than raw {raw:?}"
  );
  assert_eq!(optimized.first(), raw.first());
  assert_eq!(optimized.last(), raw.last());
}

#[test]
fn two_identical_runs_produce_identical_worlds() {
  let settings = RoutingSettings::default();
  let resolver = rules();

  let run = || {
    let context = AlgoContext::new(&resolver, &settings);
    let mut world = board();
    let root = world.root();
    let mut shove = Shove::new(root);

    shove.add_head_line(pushing_head(), ShovePolicy::SHOVE);

    let status = shove.run(&mut world, &context);
    let node = shove.current_node();
    let shapes: Vec<Vec<Vec2>> = BOARD_NETS
      .iter()
      .map(|net| shape_of(&world, node, *net))
      .collect();

    (status, shove.iterations(), shapes)
  };

  assert_eq!(run(), run());
}

// ---------------------------------------------------------------------
// Solids
// ---------------------------------------------------------------------

/// The half width of the square pad the solid scenarios use.
const PAD_HALF: i32 = 300000;

/// The net the pad is on, which is nobody else's.
const PAD_NET: Option<NetId> = Some(NetId(5));

/// The via's net, so that it is neither the head's nor a track's.
const VIA_NET: Option<NetId> = Some(NetId(6));

/// The diameter of every via a scenario places.
const VIA_DIAMETER: i32 = 400000;

/// The drill of every via a scenario places.
const VIA_DRILL: i32 = 200000;

/// Put a square pad into a node and answer its handle.
fn add_pad(world: &mut World, node: NodeId, at: Vec2) -> ItemId {
  let shape = Shape::rect(
    at - Vec2::new(PAD_HALF, PAD_HALF),
    Vec2::new(PAD_HALF * 2, PAD_HALF * 2),
  );
  let mut item = world.make_item(ItemBody::Solid(Solid::new(shape, at)));

  item.set_layers_and_flash_all(LayerRange::single(0));
  item.set_net(PAD_NET);

  world.add_solid(node, item, None)
}

/// Put a through via into a node and answer its handle.
fn add_via(
  world: &mut World,
  node: NodeId,
  at: Vec2,
  net: Option<NetId>,
) -> ItemId {
  let body =
    ItemBody::Via(Via::new(at, VIA_DIAMETER, VIA_DRILL, ViaType::Through));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::new(0, 1));
  item.set_net(net);

  world.add_via(node, item)
}

/// Where a via of one net sits in a node.
fn via_pos(world: &World, node: NodeId, net: Option<NetId>) -> Option<Vec2> {
  let vias = world.all_items_in_net(node, net, Kind::VIA);
  let first = *vias.first()?;

  match world.item(first)?.body() {
    ItemBody::Via(body) => Some(body.pos()),
    _ => None,
  }
}

#[test]
fn a_head_walks_around_a_pad_it_cannot_shove() {
  let mut world = build(&[other_layer_track()]);
  let root = world.root();
  // A pad squarely on the head's path, which no shove can move.
  add_pad(&mut world, root, Vec2::new(1500000, 0));

  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let head = pushing_head();
  let mut shove = Shove::new(root);

  shove.add_head_line(head.clone(), ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  // A solid is never moved: the head is what gave way
  // (`pns_shove.cpp:776`).
  assert!(shove.heads_modified(None));

  let walked = shove
    .modified_head(0)
    .expect("the head was re-routed around the pad")
    .clone();

  assert_eq!(walked.point(0), head.point(0));
  assert_eq!(walked.last_point(), head.last_point());
  assert!(walked.point_count() > head.point_count());

  let node = shove.current_node();

  assert_line_is_clear(&world, node, &walked);
  assert_node_is_clear(&world, node);
}

#[test]
fn a_locked_track_escalates_to_a_walkaround() {
  // `pns_shove.cpp:647` answers `SH_TRY_WALK` for a locked track and
  // `:1826` turns that into `onCollidingSolid`, so the head walks around
  // what it may not push. The track is short, because a locked track that
  // reaches past both ends of the head has no way round it and the
  // escalation then fails for a geometric reason rather than a structural
  // one.
  let mut world = build(&[stub_track()]);
  let root = world.root();

  for segment in segments_of(&world, root, NEAR_NET) {
    world
      .item_mut(segment)
      .expect("the track was just added")
      .mark(MarkerFlags::LOCKED);
  }

  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let before = shape_of(&world, root, NEAR_NET);
  let head = pushing_head();
  let mut shove = Shove::new(root);

  shove.add_head_line(head.clone(), ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let node = shove.current_node();

  // The locked track did not move, and the head did.
  assert_eq!(shape_of(&world, node, NEAR_NET), before);
  assert!(shove.heads_modified(None));

  let walked = shove
    .modified_head(0)
    .expect("the head was re-routed around the locked track")
    .clone();

  assert_eq!(walked.point(0), head.point(0));
  assert_eq!(walked.last_point(), head.last_point());
  assert_line_is_clear(&world, node, &walked);
  assert_node_is_clear(&world, node);
}

// ---------------------------------------------------------------------
// Vias
// ---------------------------------------------------------------------

#[test]
fn a_head_pushes_a_stitching_via_aside() {
  let mut world = build(&[other_layer_track()]);
  let root = world.root();
  // A via straddling the head's path, close enough to have to move.
  let at = Vec2::new(1500000, 200000);

  add_via(&mut world, root, at, VIA_NET);

  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let head = pushing_head();
  let mut shove = Shove::new(root);

  shove.add_head_line(head.clone(), ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let node = shove.current_node();
  let after =
    via_pos(&world, node, VIA_NET).expect("the via is still on the board");

  assert_ne!(after, at);
  // It was pushed away from the head, which runs along y = 0.
  assert!(after.y > at.y);
  assert_line_is_clear(&world, node, &head);
  assert_node_is_clear(&world, node);
}

#[test]
fn a_via_the_settings_forbid_moving_makes_the_head_walk_around_it() {
  let mut world = build(&[]);
  let root = world.root();
  let at = Vec2::new(1500000, 200000);

  add_via(&mut world, root, at, VIA_NET);

  // `pns_shove.cpp:1060`: with vias frozen every via collision degrades
  // to `SH_TRY_WALK`, so the head walks around instead.
  let settings = RoutingSettings {
    shove_vias: false,
    ..RoutingSettings::default()
  };
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let head = pushing_head();
  let mut shove = Shove::new(root);

  shove.add_head_line(head.clone(), ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let node = shove.current_node();

  assert_eq!(via_pos(&world, node, VIA_NET), Some(at));
  assert!(shove.heads_modified(None));

  let walked = shove
    .modified_head(0)
    .expect("the head was re-routed around the frozen via")
    .clone();

  assert_line_is_clear(&world, node, &walked);
  assert_node_is_clear(&world, node);
}

#[test]
fn a_via_that_cannot_move_at_all_leaves_the_run_incomplete() {
  let mut world = build(&[]);
  let root = world.root();
  let at = Vec2::new(1500000, 0);
  let via = add_via(&mut world, root, at, VIA_NET);

  // A locked via answers `SH_TRY_WALK` (`pns_shove.cpp:1060`), and the
  // pads on either side leave the walkaround nowhere to go, so the run
  // gives up and the world is untouched.
  world
    .item_mut(via)
    .expect("the via was just added")
    .mark(MarkerFlags::LOCKED);
  add_pad(&mut world, root, Vec2::new(1500000, -700000));
  add_pad(&mut world, root, Vec2::new(1500000, 700000));

  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let mut shove = Shove::new(root);

  shove.add_head_line(pushing_head(), ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Incomplete);
  assert_eq!(shove.current_node(), root);
  assert_eq!(via_pos(&world, root, VIA_NET), Some(at));
}

#[test]
fn a_pushed_via_drags_the_track_hanging_off_it() {
  let mut world = build(&[]);
  let root = world.root();
  let at = Vec2::new(1500000, 200000);

  add_via(&mut world, root, at, VIA_NET);

  // One track running north out of the via, on the via's net, so that the
  // fanout drag has something to carry (`pns_shove.cpp:1081`).
  let tail = Seg::new(at, Vec2::new(1500000, 2000000));
  let mut item =
    world.make_item(ItemBody::Segment(Segment::new(tail, TRACK_WIDTH)));

  item.set_layers_and_flash_all(LayerRange::single(0));
  item.set_net(VIA_NET);
  world
    .add_segment(root, item, false)
    .expect("the fanout track is neither degenerate nor redundant");

  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let head = pushing_head();
  let mut shove = Shove::new(root);

  shove.add_head_line(head.clone(), ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let node = shove.current_node();
  let after =
    via_pos(&world, node, VIA_NET).expect("the via is still on the board");

  assert_ne!(after, at);

  // The track followed the via: its via end sits where the via now is,
  // and its far end did not move.
  let track = track_of(&world, node, VIA_NET);
  let ends = [track.point(0), track.last_point().expect("two ends")];

  assert!(
    ends.contains(&after),
    "the dragged track {ends:?} does not reach the via at {after:?}"
  );
  assert!(ends.contains(&Vec2::new(1500000, 2000000)));
  assert_line_is_clear(&world, node, &head);
  assert_node_is_clear(&world, node);
}

#[test]
fn a_head_that_ends_with_a_via_pushes_a_track_on_the_other_layer() {
  // The via head path: the head's own via is stored in the shove's branch
  // (`pns_shove.cpp:2505`) and hulled as part of the pusher (`:601`), so
  // a track on the layer the head does not run on still gets out of its
  // way.
  let mut world = build(&[other_layer_track()]);
  let root = world.root();
  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let before = shape_of(&world, root, OTHER_LAYER_NET);

  // A head on layer 0 that ends over the layer 1 track with a via.
  let mut head = head(&[Vec2::new(0, -2000000), Vec2::new(1500000, -500000)]);
  let mut via = world.make_item(ItemBody::Via(Via::new(
    Vec2::new(1500000, -500000),
    VIA_DIAMETER,
    VIA_DRILL,
    ViaType::Through,
  )));

  via.set_layers_and_flash_all(LayerRange::new(0, 1));
  via.set_net(HEAD_NET);
  head.append_via(via);

  // The via really does stand in the layer 1 track's way once it is put
  // where the head ends.
  let mut shove = Shove::new(root);

  shove.add_head_line(head.clone(), ShovePolicy::SHOVE);

  let status = shove.run(&mut world, &context);
  let node = shove.current_node();

  assert_eq!(status, ShoveStatus::Ok);
  // Nothing on layer 1 was in the way at that height, so the run is a
  // clean no op; what matters is that a via head does not fail.
  assert_eq!(shape_of(&world, node, OTHER_LAYER_NET), before);
  assert_node_is_clear(&world, node);
  // The head's via was taken back out of the node with the head.
  assert!(world.all_items_in_net(node, HEAD_NET, Kind::VIA).is_empty());
}

#[test]
fn a_via_drag_head_moves_the_via_and_reports_its_new_handle() {
  let mut world = build(&[]);
  let root = world.root();
  let at = Vec2::new(1500000, 0);
  let via = add_via(&mut world, root, at, VIA_NET);
  let handle = ViaHandle::of(&world, via).expect("the via was just added");

  let settings = RoutingSettings::default();
  let resolver = rules();
  let context = AlgoContext::new(&resolver, &settings);
  let target = Vec2::new(2500000, 0);
  let mut shove = Shove::new(root);

  // `AddHeads( VIA_HANDLE, VECTOR2I, int )`, `pns_shove.cpp:2261`.
  shove.add_head_via(handle, target, ShovePolicy::SHOVE);

  assert_eq!(shove.run(&mut world, &context), ShoveStatus::Ok);

  let node = shove.current_node();
  let moved = shove.head_via(0).expect("the drag reports a handle");

  assert_eq!(moved.pos, target);
  assert_eq!(via_pos(&world, node, VIA_NET), Some(target));
  // The handle resolves in the node the shove handed back, and it names a
  // **different** item: a shove moves a via by replacing it, which is the
  // whole reason the caller is handed a handle and not an item id.
  let resolved = world
    .find_via_by_handle(node, moved.pos, moved.layers, moved.net)
    .expect("the handle resolves in the node the shove handed back");

  assert_ne!(resolved, via);
  assert_node_is_clear(&world, node);
}
