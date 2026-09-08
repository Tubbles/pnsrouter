// SPDX-License-Identifier: GPL-3.0-or-later

//! Connectivity queries over a two layer board, through the public API
//! only.
//!
//! The counterpart of `tests/world.rs` and `tests/placer.rs` for
//! `src/topology.rs`: the board is plain data at the top, everything the
//! scenarios need is reachable from outside the crate, and no test only
//! helper exists.
//!
//! The board carries three nets, one per question the module answers:
//!
//! - [`TRACE_NET`] is a run of track hanging off a pad, with a lone pad
//!   to reach and a second, unreachable cluster somewhere else on the
//!   same net. It drives [`nearest_unconnected_item`],
//!   [`leading_rat_line`] and [`connected_joints`].
//! - [`CLOSED_NET`] is two pads joined by one segment and nothing else,
//!   which is what "the net is fully routed" looks like.
//! - [`VIA_NET`] is a run that changes layer halfway, for
//!   [`assemble_trivial_path`].

#![forbid(unsafe_code)]

use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{
  ItemBody, ItemId, Kind, LayerRange, NetId, Segment, Solid, Via, ViaType,
};
use pnsrouter::line::Line;
use pnsrouter::node::{JointRef, World};
use pnsrouter::rules::FixedClearance;
use pnsrouter::topology::{
  assemble_trivial_path, connected_joints, leading_rat_line,
  nearest_unconnected_item,
};

/// The net the routed track and its targets sit on.
const TRACE_NET: Option<NetId> = Some(NetId(1));

/// The net that is already completely routed.
const CLOSED_NET: Option<NetId> = Some(NetId(2));

/// The net whose run changes layer through a via.
const VIA_NET: Option<NetId> = Some(NetId(3));

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 200_000;

/// The width of every track.
const TRACK_WIDTH: i32 = 200_000;

/// The clearance the rule oracle answers with. Nothing here collides, so
/// the value only has to be a legal one.
const CLEARANCE: i32 = 100_000;

/// The pad the routed run hangs off.
const WEST_PAD: Vec2 = Vec2::new(0, 0);

/// The corner in the middle of the routed run.
const WEST_CORNER: Vec2 = Vec2::new(500_000, 0);

/// The loose end of the routed run, where a head would carry on from.
const WEST_END: Vec2 = Vec2::new(1_000_000, 0);

/// The lone pad on [`TRACE_NET`], the thing the run still has to reach.
const EAST_PAD: Vec2 = Vec2::new(4_000_000, 0);

/// The pad of the second [`TRACE_NET`] cluster, far enough away that it
/// never wins a distance comparison.
const FAR_PAD: Vec2 = Vec2::new(10_000_000, 0);

/// The loose end of the second cluster's stub.
const FAR_END: Vec2 = Vec2::new(10_500_000, 0);

/// The only [`TRACE_NET`] pad on the second copper layer.
const TOP_PAD: Vec2 = Vec2::new(0, -6_000_000);

/// The west pad of the fully routed net.
const CLOSED_WEST: Vec2 = Vec2::new(0, 3_000_000);

/// The east pad of the fully routed net.
const CLOSED_EAST: Vec2 = Vec2::new(1_000_000, 3_000_000);

/// Where the run that changes layer starts, on layer zero.
const VIA_RUN_START: Vec2 = Vec2::new(0, -3_000_000);

/// The corner in the middle of that run's layer zero half.
const VIA_RUN_CORNER: Vec2 = Vec2::new(500_000, -3_000_000);

/// Where that run changes layer.
const VIA_POS: Vec2 = Vec2::new(1_000_000, -3_000_000);

/// Where the run ends, on layer one.
const VIA_RUN_END: Vec2 = Vec2::new(1_500_000, -3_000_000);

/// The handles a scenario needs to name.
struct Board {
  /// The pad the routed run hangs off, on [`TRACE_NET`].
  west_pad: ItemId,
  /// The lone pad the routed run has to reach.
  east_pad: ItemId,
  /// The first segment of the routed run.
  west_track: ItemId,
  /// The first segment of the run that changes layer.
  via_run_track: ItemId,
}

/// A pad, as a host would hand it over.
fn add_pad(
  world: &mut World,
  at: Vec2,
  layer: i32,
  net: Option<NetId>,
) -> ItemId {
  let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
  let mut item = world.make_item(body);
  let root = world.root();

  item.set_layers_and_flash_all(LayerRange::single(layer));
  item.set_net(net);

  world.add_solid(root, item, None)
}

/// A track segment.
fn add_track(
  world: &mut World,
  from: Vec2,
  to: Vec2,
  layer: i32,
  net: Option<NetId>,
) -> ItemId {
  let body = ItemBody::Segment(Segment::new(Seg::new(from, to), TRACK_WIDTH));
  let mut item = world.make_item(body);
  let root = world.root();

  item.set_layers_and_flash_all(LayerRange::single(layer));
  item.set_net(net);

  world
    .add_segment(root, item, false)
    .expect("no track of the board is degenerate or redundant")
}

/// A through via spanning both copper layers.
fn add_via(world: &mut World, at: Vec2, net: Option<NetId>) -> ItemId {
  let body =
    ItemBody::Via(Via::new(at, PAD_RADIUS * 2, PAD_RADIUS, ViaType::Through));
  let mut item = world.make_item(body);
  let root = world.root();

  item.set_layers_and_flash_all(LayerRange::new(0, 1));
  item.set_net(net);

  world.add_via(root, item)
}

/// Turn the plain data above into a world.
fn build() -> (World, Board) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);

  // The routed net: a pad, two segments hanging off it, the pad the run
  // has to reach, a second cluster on the same net and one pad on the
  // second copper layer.
  let west_pad = add_pad(&mut world, WEST_PAD, 0, TRACE_NET);
  let west_track = add_track(&mut world, WEST_PAD, WEST_CORNER, 0, TRACE_NET);

  add_track(&mut world, WEST_CORNER, WEST_END, 0, TRACE_NET);

  let east_pad = add_pad(&mut world, EAST_PAD, 0, TRACE_NET);

  add_pad(&mut world, FAR_PAD, 0, TRACE_NET);
  add_track(&mut world, FAR_PAD, FAR_END, 0, TRACE_NET);
  add_pad(&mut world, TOP_PAD, 1, TRACE_NET);

  // The fully routed net.
  add_pad(&mut world, CLOSED_WEST, 0, CLOSED_NET);
  add_track(&mut world, CLOSED_WEST, CLOSED_EAST, 0, CLOSED_NET);
  add_pad(&mut world, CLOSED_EAST, 0, CLOSED_NET);

  // The net that changes layer halfway.
  add_pad(&mut world, VIA_RUN_START, 0, VIA_NET);

  let via_run_track =
    add_track(&mut world, VIA_RUN_START, VIA_RUN_CORNER, 0, VIA_NET);

  add_track(&mut world, VIA_RUN_CORNER, VIA_POS, 0, VIA_NET);
  add_via(&mut world, VIA_POS, VIA_NET);
  add_track(&mut world, VIA_POS, VIA_RUN_END, 1, VIA_NET);
  add_pad(&mut world, VIA_RUN_END, 1, VIA_NET);

  (
    world,
    Board {
      west_pad,
      east_pad,
      west_track,
      via_run_track,
    },
  )
}

/// The joint at a point on a net's first copper layer.
fn joint_at(
  world: &World,
  at: Vec2,
  layer: i32,
  net: Option<NetId>,
) -> JointRef {
  world
    .find_joint(world.root(), at, layer, net)
    .expect("the fixture puts a joint at every point a scenario names")
}

/// Where a joint sits.
fn joint_pos(world: &World, reference: JointRef) -> Vec2 {
  world
    .joint(reference)
    .expect("a joint a query answered with is live")
    .pos()
}

/// A head the placer could be dragging: on [`TRACE_NET`], on layer zero,
/// carrying on eastwards from the loose end of the routed run.
fn head(points: &[Vec2]) -> Line {
  let mut line = Line::new();

  line.set_width(TRACK_WIDTH);
  line.set_layer(0);
  line.set_net(TRACE_NET);
  line.set_shape(LineChain::from_slice(points, false));

  line
}

// ---------------------------------------------------------------------
// ConnectedJoints
// ---------------------------------------------------------------------

#[test]
fn connected_joints_walks_one_cluster_and_leaves_the_rest_of_the_net_alone() {
  let (world, _board) = build();
  let start = joint_at(&world, WEST_PAD, 0, TRACE_NET);

  let mut reached: Vec<Vec2> = connected_joints(&world, world.root(), start)
    .into_iter()
    .map(|reference| joint_pos(&world, reference))
    .collect();

  reached.sort_by_key(|point| (point.x, point.y));

  // The whole run, and nothing else: the walk follows segment links, so
  // it stops at the pad it started on and never crosses to the second
  // cluster of the same net.
  assert_eq!(reached, [WEST_PAD, WEST_CORNER, WEST_END]);

  // The second cluster is reachable from its own pad and only from
  // there.
  let far = joint_at(&world, FAR_PAD, 0, TRACE_NET);
  let mut from_far: Vec<Vec2> = connected_joints(&world, world.root(), far)
    .into_iter()
    .map(|reference| joint_pos(&world, reference))
    .collect();

  from_far.sort_by_key(|point| (point.x, point.y));

  assert_eq!(from_far, [FAR_PAD, FAR_END]);
}

#[test]
fn connected_joints_crosses_a_via_onto_the_other_layer() {
  let (world, _board) = build();
  let start = joint_at(&world, VIA_RUN_START, 0, VIA_NET);

  let mut reached: Vec<Vec2> = connected_joints(&world, world.root(), start)
    .into_iter()
    .map(|reference| joint_pos(&world, reference))
    .collect();

  reached.sort_by_key(|point| (point.x, point.y));

  // The via is not an edge of the walk, but it merged the joints of the
  // two layers it spans into one, so the layer one half of the run hangs
  // off a joint the layer zero half already reached.
  assert_eq!(
    reached,
    [VIA_RUN_START, VIA_RUN_CORNER, VIA_POS, VIA_RUN_END]
  );
}

// ---------------------------------------------------------------------
// NearestUnconnectedItem
// ---------------------------------------------------------------------

#[test]
fn the_nearest_unconnected_item_is_the_lone_pad_and_not_the_run_s_own_pad() {
  let (world, board) = build();
  let start = joint_at(&world, WEST_END, 0, TRACE_NET);

  let found = nearest_unconnected_item(&world, world.root(), start, Kind::ANY)
    .expect("the east pad is on the net and out of reach");

  // The west pad is 1 mm away and the east pad 3 mm, so the west pad
  // would win if the walk through the run's own segments had not
  // excluded it.
  assert_eq!(found.item, board.east_pad);
  assert_ne!(found.item, board.west_pad);

  let anchor = world
    .item(found.item)
    .expect("the answer names a live item")
    .anchor(found.anchor);

  assert_eq!(anchor, EAST_PAD);
}

#[test]
fn a_fully_connected_net_has_no_unconnected_item() {
  let (world, _board) = build();
  let start = joint_at(&world, CLOSED_WEST, 0, CLOSED_NET);

  assert!(
    nearest_unconnected_item(&world, world.root(), start, Kind::ANY).is_none()
  );
}

#[test]
fn the_kind_mask_narrows_what_may_answer() {
  let (world, board) = build();
  let start = joint_at(&world, WEST_END, 0, TRACE_NET);

  // With solids excluded the only candidates left on the net are the
  // second cluster's stub segment and the pads it hangs off, so the
  // answer moves from the lone pad to that stub.
  let found =
    nearest_unconnected_item(&world, world.root(), start, Kind::SEGMENT)
      .expect("the second cluster's stub is a segment on the net");

  assert_ne!(found.item, board.east_pad);

  let anchor = world
    .item(found.item)
    .expect("the answer names a live item")
    .anchor(found.anchor);

  assert_eq!(anchor, FAR_PAD);
}

// ---------------------------------------------------------------------
// LeadingRatLine
// ---------------------------------------------------------------------

#[test]
fn the_leading_rat_line_runs_from_the_head_s_end_to_the_lone_pad() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let root = world.root();
  let track = head(&[WEST_END, Vec2::new(1_500_000, 0)]);

  let rat_line = leading_rat_line(&mut world, root, &rules, &track)
    .expect("the east pad is still out of reach");

  assert_eq!(rat_line.point_count(), 2);
  assert_eq!(rat_line.point(0), Vec2::new(1_500_000, 0));
  assert_eq!(rat_line.point(1), EAST_PAD);
}

#[test]
fn the_leading_rat_line_ignores_what_the_head_already_reaches() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let root = world.root();

  // A head running back westwards along its own run. Its end is nearer
  // to the west pad than to anything else, and the west pad is reachable
  // through the head's own segment, so the answer must still be the east
  // pad.
  let track = head(&[WEST_END, Vec2::new(300_000, 0)]);

  let rat_line = leading_rat_line(&mut world, root, &rules, &track)
    .expect("the east pad is still out of reach");

  assert_eq!(rat_line.point(0), Vec2::new(300_000, 0));
  assert_eq!(rat_line.point(1), EAST_PAD);
}

#[test]
fn a_head_that_has_arrived_gets_a_rat_line_of_one_point() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let root = world.root();

  // The head lands on the pad it was looking for, so the joint under its
  // end already has something else on it and the answer is that joint's
  // own position. Both ends of the chain are then the same point and
  // `LineChain::append` swallows the second one, which is the degenerate
  // case `topology::leading_rat_line` documents.
  let track = head(&[WEST_END, EAST_PAD]);

  let rat_line = leading_rat_line(&mut world, root, &rules, &track)
    .expect("the head's end sits on the east pad");

  assert_eq!(rat_line.point_count(), 1);
  assert_eq!(rat_line.point(0), EAST_PAD);
}

#[test]
fn a_fully_connected_net_has_no_leading_rat_line() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let root = world.root();
  let mut track = head(&[CLOSED_WEST, Vec2::new(500_000, 3_000_000)]);

  track.set_net(CLOSED_NET);

  assert!(leading_rat_line(&mut world, root, &rules, &track).is_none());
}

#[test]
fn the_scratch_node_the_rat_line_needs_leaves_nothing_behind() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let root = world.root();
  let before = world.all_items_in_net(root, TRACE_NET, Kind::ANY);
  let track = head(&[WEST_END, Vec2::new(1_500_000, 0)]);

  leading_rat_line(&mut world, root, &rules, &track)
    .expect("the east pad is still out of reach");

  assert_eq!(world.all_items_in_net(root, TRACE_NET, Kind::ANY), before);
  assert!(!world.node(root).expect("the root is live").has_children());
}

// ---------------------------------------------------------------------
// AssembleTrivialPath
// ---------------------------------------------------------------------

#[test]
fn a_trivial_path_reaches_the_joint_on_each_side_of_a_segment() {
  let (world, board) = build();
  let path =
    assemble_trivial_path(&world, world.root(), board.west_track, false);
  let (left, right) = path
    .terminals
    .expect("a run with two ends has two terminal joints");

  let mut ends = [joint_pos(&world, left), joint_pos(&world, right)];

  ends.sort_by_key(|point| (point.x, point.y));

  // The run ends on the pad at one side and on its loose end at the
  // other, and neither branch had anywhere else to go.
  assert_eq!(ends, [WEST_PAD, WEST_END]);
  assert_eq!(path.items.len(), 1);
}

#[test]
fn a_trivial_path_carries_on_through_a_via_onto_the_other_layer() {
  let (world, board) = build();
  let path =
    assemble_trivial_path(&world, world.root(), board.via_run_track, false);
  let (left, right) = path
    .terminals
    .expect("a run with two ends has two terminal joints");

  let mut ends = [joint_pos(&world, left), joint_pos(&world, right)];

  ends.sort_by_key(|point| (point.x, point.y));

  assert_eq!(ends, [VIA_RUN_START, VIA_RUN_END]);

  // The seed line, the via it stopped at and the layer one run beyond
  // it.
  assert_eq!(path.items.len(), 3);
}

#[test]
fn a_trivial_path_from_a_pad_is_empty() {
  let (world, board) = build();
  let path = assemble_trivial_path(&world, world.root(), board.west_pad, false);

  assert!(path.items.is_empty());
  assert!(path.terminals.is_none());
}

// ---------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------

/// Everything one run of the queries answered, as plain numbers.
#[derive(PartialEq, Eq, Debug)]
struct Report {
  /// The joints [`connected_joints`] reached from the west pad.
  reached: Vec<Vec2>,
  /// The anchor [`nearest_unconnected_item`] settled on.
  nearest: Vec2,
  /// The chain [`leading_rat_line`] built.
  rat_line: Vec<Vec2>,
  /// The terminals of the run that changes layer.
  terminals: Vec<Vec2>,
}

/// Build a world from scratch and ask it everything.
fn run() -> Report {
  let (mut world, board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let root = world.root();

  let reached =
    connected_joints(&world, root, joint_at(&world, WEST_PAD, 0, TRACE_NET))
      .into_iter()
      .map(|reference| joint_pos(&world, reference))
      .collect();

  let start = joint_at(&world, WEST_END, 0, TRACE_NET);
  let found = nearest_unconnected_item(&world, root, start, Kind::ANY)
    .expect("the east pad is still out of reach");
  let nearest = world
    .item(found.item)
    .expect("the answer names a live item")
    .anchor(found.anchor);

  let path = assemble_trivial_path(&world, root, board.via_run_track, false);
  let (left, right) = path
    .terminals
    .expect("a run with two ends has two terminal joints");
  let terminals = vec![joint_pos(&world, left), joint_pos(&world, right)];

  let track = head(&[WEST_END, Vec2::new(1_500_000, 0)]);
  let chain = leading_rat_line(&mut world, root, &rules, &track)
    .expect("the east pad is still out of reach");
  let rat_line = (0..chain.point_count()).map(|i| chain.point(i)).collect();

  Report {
    reached,
    nearest,
    rat_line,
    terminals,
  }
}

#[test]
fn two_identical_runs_answer_identically() {
  assert_eq!(run(), run());
}
