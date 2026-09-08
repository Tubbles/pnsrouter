// SPDX-License-Identifier: GPL-3.0-or-later

//! The milestone 2 exit test: a whole routing world driven through the
//! public API only.
//!
//! `doc/work/002-world-model.md` asks for one scenario that builds a
//! world from plain data, branches it, adds and removes items, commits,
//! assembles lines across joints and queries what collides. It is here
//! rather than next to the code because that is the point: everything it
//! needs has to be reachable from outside the crate, with no access to a
//! private field and no test only helper.
//!
//! The plain data at the top is the shape a host snapshot takes
//! (`DESIGN.md` section 12): rows of numbers, no engine types.

#![forbid(unsafe_code)]

use pnsrouter::collide::CollisionSearchOptions;
use pnsrouter::geometry::direction45::CornerMode;
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{
  Hole, ItemBody, ItemId, Kind, LayerRange, NetId, Segment, Solid, Via, ViaType,
};
use pnsrouter::line::Line;
use pnsrouter::node::{NodeId, World};
use pnsrouter::rules::FixedClearance;

/// The clearance the whole scenario routes to, in nanometres.
const CLEARANCE: i32 = 2000;

/// The net the board is wired on.
const SIGNAL: Option<NetId> = Some(NetId(1));

/// The net the head being routed is on, so that nothing on the board
/// exempts it.
const HEAD_NET: Option<NetId> = Some(NetId(2));

/// The width of every track on the board.
const TRACK_WIDTH: i32 = 1000;

/// A pad of the board, as a host would hand it over: position, layer,
/// copper radius, drill radius.
const PADS: [(i32, i32, i32, i32, i32); 3] = [
  (0, 0, 0, 3000, 500),
  (300000, 0, 0, 3000, 500),
  (300000, 100000, 1, 3000, 500),
];

/// A track of the board: both endpoints and the layer.
const TRACKS: [(i32, i32, i32, i32, i32); 3] = [
  (0, 0, 100000, 0, 0),
  (100000, 0, 200000, 0, 0),
  (200000, 0, 300000, 0, 0),
];

/// The via of the board: position, copper diameter, drill diameter.
const VIA: (i32, i32, i32, i32) = (300000, 0, 3000, 1000);

/// Everything the builder stored, in the order the plain data lists it.
struct Board {
  /// The three pads.
  pads: Vec<ItemId>,
  /// The three track segments, west to east.
  tracks: Vec<ItemId>,
  /// The via at the east pad.
  via: ItemId,
}

/// Turn the plain data into a world.
fn build() -> (World, Board) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let mut pads = Vec::new();
  let mut tracks = Vec::new();

  for (x, y, layer, radius, drill) in PADS {
    let at = Vec2::new(x, y);
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, radius), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(SIGNAL);
    pads.push(world.add_solid(root, item, Some(Hole::circular(at, drill))));
  }

  for (from_x, from_y, to_x, to_y, layer) in TRACKS {
    let seg = Seg::new(Vec2::new(from_x, from_y), Vec2::new(to_x, to_y));
    let body = ItemBody::Segment(Segment::new(seg, TRACK_WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(SIGNAL);
    tracks.push(
      world
        .add_segment(root, item, false)
        .expect("no track of the board is degenerate or redundant"),
    );
  }

  let (x, y, diameter, drill) = VIA;
  let at = Vec2::new(x, y);
  let body = ItemBody::Via(Via::new(at, diameter, drill, ViaType::Through));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::new(0, 1));
  item.set_net(SIGNAL);
  let via = world.add_via(root, item);

  (world, Board { pads, tracks, via })
}

/// A head line the placer could be dragging: on layer 0, on
/// [`HEAD_NET`], with the board's track width.
fn head_line(points: &[Vec2]) -> Line {
  let mut line = Line::new();

  line.set_width(TRACK_WIDTH);
  line.set_layer(0);
  line.set_net(HEAD_NET);
  line.set_shape(LineChain::from_slice(points, false));

  line
}

/// Every item of the signal net a node can see, in uid order.
fn signal_items(world: &World, node: NodeId) -> Vec<ItemId> {
  world.all_items_in_net(node, SIGNAL, Kind::ANY)
}

/// The links of the joint at a point, or an empty list when there is
/// none.
fn joint_links(world: &World, node: NodeId, at: Vec2) -> Vec<ItemId> {
  world
    .find_joint(node, at, 0, SIGNAL)
    .and_then(|joint| world.joint(joint))
    .map_or_else(Vec::new, |joint| joint.links().to_vec())
}

#[test]
fn a_world_survives_a_branch_a_commit_and_every_query_in_between() {
  let (mut world, board) = build();
  let root = world.root();
  let rules = FixedClearance::uniform(CLEARANCE);
  let options = CollisionSearchOptions::default();

  // --- the world the plain data describes -------------------------
  //
  // Three pads with a hole each, three tracks and a via with a hole:
  // eleven items, all on the signal net.
  assert_eq!(signal_items(&world, root).len(), 11);

  // The west pad and the first track meet at the origin.
  assert_eq!(
    joint_links(&world, root, Vec2::new(0, 0)),
    [board.pads[0], board.tracks[0]]
  );
  // The east end carries the pad, the last track and the via, which is
  // what makes it a non trivial joint and ends any line there. The
  // order is the order they were added in, which is the plain data's.
  assert_eq!(
    joint_links(&world, root, Vec2::new(300000, 0)),
    [board.pads[1], board.tracks[2], board.via]
  );

  // --- assembling a line across the plain joints ------------------
  //
  // The two interior joints are trivial, so the run of three segments
  // is one line from pad to pad.
  let mut origin = usize::MAX;
  let assembled = world.assemble_line(
    root,
    board.tracks[1],
    Some(&mut origin),
    false,
    false,
    true,
  );

  assert_eq!(
    assembled.shape().points(),
    [
      Vec2::new(0, 0),
      Vec2::new(100000, 0),
      Vec2::new(200000, 0),
      Vec2::new(300000, 0),
    ]
  );
  assert_eq!(assembled.links(), board.tracks.as_slice());
  assert_eq!(assembled.links_valid_in(), Some(root));
  assert_eq!(origin, 1);
  assert!(assembled.is_linked_checked());
  // A via at the end is not picked up; the placer attaches one itself.
  assert!(!assembled.ends_with_via());

  // --- the branch --------------------------------------------------
  let branch = world.branch(root);

  // A detour north of the straight run, added as a line, which is how
  // a placer fixes a route.
  let mut detour = head_line(&[
    Vec2::new(100000, 0),
    Vec2::new(150000, 50000),
    Vec2::new(200000, 0),
  ]);
  detour.set_net(SIGNAL);
  let added = world.add_line(branch, &mut detour, false);

  assert_eq!(added.len(), 2);
  assert_eq!(detour.links_valid_in(), Some(branch));

  // And the middle of the straight run goes away.
  world.remove(branch, board.tracks[1]);

  let (branch_added, branch_removed) = world.get_updated_items(branch);

  assert_eq!(branch_added, added);
  assert_eq!(branch_removed, [board.tracks[1]]);

  // The root is untouched: removing one of its items from a branch
  // only shadows it.
  assert_eq!(signal_items(&world, root).len(), 11);
  assert!(world.item(board.tracks[1]).is_some());
  assert_eq!(signal_items(&world, branch).len(), 12);

  // The line the branch assembles now runs over the detour.
  let rerouted =
    world.assemble_line(branch, board.tracks[0], None, false, false, true);

  assert_eq!(
    rerouted.shape().points(),
    [
      Vec2::new(0, 0),
      Vec2::new(100000, 0),
      Vec2::new(150000, 50000),
      Vec2::new(200000, 0),
      Vec2::new(300000, 0),
    ]
  );

  // --- what a head runs into ---------------------------------------
  //
  // A head crossing the detour's northern corner, on its own net.
  let head = head_line(&[Vec2::new(150000, -50000), Vec2::new(150000, 150000)]);

  let obstacles: Vec<ItemId> = world
    .query_colliding_line(branch, &head, &rules, &options)
    .into_iter()
    .filter_map(|found| found.item)
    .collect();

  assert_eq!(obstacles, added);
  assert!(
    world
      .check_colliding_line(branch, &head, &rules, &options)
      .is_some()
  );

  // The same head in the root, where the detour does not exist and the
  // straight run does, meets the middle segment instead.
  let in_root: Vec<ItemId> = world
    .query_colliding_line(root, &head, &rules, &options)
    .into_iter()
    .filter_map(|found| found.item)
    .collect();

  assert_eq!(in_root, [board.tracks[1]]);

  let nearest = world
    .nearest_obstacle(branch, &head, &rules, &options, CornerMode::Mitered45)
    .expect("the head crosses the detour");

  assert!(added.contains(&nearest.item.expect("a live obstacle")));
  assert!(nearest.found_intersection);
  assert!(nearest.dist_first > 0);
  assert!(
    nearest
      .hull
      .as_ref()
      .expect("a live obstacle has a hull")
      .point_on_edge(nearest.ip_first, 0)
  );

  // --- the commit --------------------------------------------------
  world.commit(branch);

  // The branch is gone, and the root holds what it held.
  assert!(world.node(branch).is_none());
  assert_eq!(signal_items(&world, root).len(), 12);
  assert!(world.item(board.tracks[1]).is_none());

  for item in &added {
    assert!(world.item(*item).is_some());
  }

  // The joints moved with the items: the western interior joint now
  // links the first track and the first leg of the detour.
  assert_eq!(
    joint_links(&world, root, Vec2::new(100000, 0)),
    [board.tracks[0], added[0]]
  );
  assert_eq!(
    joint_links(&world, root, Vec2::new(150000, 50000)),
    [added[0], added[1]]
  );

  // And the root assembles the rerouted line, across joints that only
  // exist because of the commit.
  let committed = world.assemble_line(root, added[0], None, false, false, true);

  assert_eq!(committed.shape().points(), rerouted.shape().points());
  assert_eq!(
    committed.links(),
    [board.tracks[0], added[0], added[1], board.tracks[2]]
  );

  // The head still finds the detour, now in the root.
  let after: Vec<ItemId> = world
    .query_colliding_line(root, &head, &rules, &options)
    .into_iter()
    .filter_map(|found| found.item)
    .collect();

  assert_eq!(after, added);
}

/// The whole scenario is a pure function of its plain data, down to the
/// order of every list it returns (`DESIGN.md` section 8).
#[test]
fn the_scenario_answers_identically_twice() {
  /// Everything one run produces.
  #[derive(PartialEq, Eq, Debug)]
  struct Answers {
    /// The signal net in the committed root.
    items: Vec<ItemId>,
    /// The points of the line the root assembles afterwards.
    points: Vec<Vec2>,
    /// Its links.
    links: Vec<ItemId>,
    /// What the head collides with.
    obstacles: Vec<Option<ItemId>>,
    /// Which of them it reaches first, and where.
    nearest: Option<(Option<ItemId>, i64, Vec2)>,
  }

  fn run() -> Answers {
    let (mut world, board) = build();
    let root = world.root();
    let rules = FixedClearance::uniform(CLEARANCE);
    let options = CollisionSearchOptions::default();
    let branch = world.branch(root);

    let mut detour = head_line(&[
      Vec2::new(100000, 0),
      Vec2::new(150000, 50000),
      Vec2::new(200000, 0),
    ]);
    detour.set_net(SIGNAL);
    let added = world.add_line(branch, &mut detour, false);

    world.remove(branch, board.tracks[1]);
    world.commit(branch);

    let head =
      head_line(&[Vec2::new(150000, -50000), Vec2::new(150000, 150000)]);
    let assembled =
      world.assemble_line(root, added[0], None, false, false, true);
    let nearest = world
      .nearest_obstacle(root, &head, &rules, &options, CornerMode::Mitered45)
      .map(|found| (found.item, found.dist_first, found.ip_first));

    Answers {
      items: signal_items(&world, root),
      points: assembled.shape().points().to_vec(),
      links: assembled.links().to_vec(),
      obstacles: world
        .query_colliding_line(root, &head, &rules, &options)
        .into_iter()
        .map(|found| found.item)
        .collect(),
      nearest,
    }
  }

  assert_eq!(run(), run());
}
