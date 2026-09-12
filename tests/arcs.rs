// SPDX-License-Identifier: GPL-3.0-or-later

//! The slice 5 exit test of `doc/work/012-arcs.md`: an arc track through
//! the world, driven through the public API only.
//!
//! An arc is an obstacle and a link like any other from here on, and that
//! claim is only worth something if it holds from outside the crate: a
//! host builds the snapshot, the world stores the arc, a query finds it,
//! a line assembly picks it up whole, a removal forgets it everywhere,
//! and a recording of the same board reads back byte for byte. No
//! algorithm above the world is involved yet; the placer and the shove
//! meet arcs in slices 6 and 7.

#![forbid(unsafe_code)]

use pnsrouter::collide::CollisionSearchOptions;
use pnsrouter::eventlog::SessionRecording;
use pnsrouter::geometry::arc::ShapeArc;
use pnsrouter::geometry::direction45::CornerMode;
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{HostId, ItemBody, ItemId, Kind, LayerRange, NetId};
use pnsrouter::line::Line;
use pnsrouter::node::World;
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RoutingSettings, Sizes};
use pnsrouter::snapshot::{WorldGeometry, WorldItem, WorldSnapshot};

/// The clearance the scenario queries at, in nanometres.
const CLEARANCE: i32 = 2000;

/// The net the board is wired on.
const SIGNAL: Option<NetId> = Some(NetId(1));

/// The net the probing line is on, so that nothing on the board exempts
/// it.
const PROBE_NET: Option<NetId> = Some(NetId(2));

/// The width of every track on the board.
const TRACK_WIDTH: i32 = 1000;

/// Where the straight run into the arc starts.
const WEST: Vec2 = Vec2::new(-100_000, 0);

/// Where the straight run out of the arc ends.
const EAST: Vec2 = Vec2::new(200_000, 100_000);

/// The arc track: a quarter turn about `(100000, 0)` of radius 100000,
/// from the origin up to `(100000, 100000)`.
const ARC: ShapeArc = ShapeArc::new(
  Vec2::new(0, 0),
  Vec2::new(29289, 70711),
  Vec2::new(100_000, 100_000),
  TRACK_WIDTH,
);

/// What the builder stored, in snapshot order.
struct Board {
  /// The straight run into the arc.
  west: ItemId,
  /// The arc itself.
  arc: ItemId,
  /// The straight run out of the arc.
  east: ItemId,
}

/// The board as plain data, the way a host hands it over.
fn snapshot() -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(WorldItem::new(
    HostId(1),
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(WEST, ARC.start()),
      width: TRACK_WIDTH,
    },
  ));
  snapshot.items.push(WorldItem::new(
    HostId(2),
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Arc {
      start: ARC.start(),
      mid: ARC.arc_mid(),
      end: ARC.end(),
      width: TRACK_WIDTH,
    },
  ));
  snapshot.items.push(WorldItem::new(
    HostId(3),
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(ARC.end(), EAST),
      width: TRACK_WIDTH,
    },
  ));

  snapshot
}

/// The world the snapshot describes, with the three handles.
fn build() -> (World, Board) {
  let snapshot = snapshot();
  let (world, index) = World::from_snapshot(&snapshot);
  let resolve = |host: u64| {
    *index
      .items_of(HostId(host))
      .first()
      .expect("every snapshot item was stored")
  };

  let board = Board {
    west: resolve(1),
    arc: resolve(2),
    east: resolve(3),
  };

  (world, board)
}

/// A line the router could be moving, on layer 0 and on [`PROBE_NET`].
fn probe(points: &[Vec2]) -> Line {
  let mut line = Line::new();

  line.set_width(TRACK_WIDTH);
  line.set_layer(0);
  line.set_net(PROBE_NET);
  line.set_shape(LineChain::from_slice(points, false));

  line
}

/// The links of the joint at a point, or an empty list when there is
/// none.
fn joint_links(world: &World, at: Vec2) -> Vec<ItemId> {
  let root = world.root();

  world
    .find_joint(root, at, 0, SIGNAL)
    .and_then(|joint| world.joint(joint))
    .map_or_else(Vec::new, |joint| joint.links().to_vec())
}

#[test]
fn an_arc_track_is_an_obstacle_a_link_and_a_recordable_item() {
  let (mut world, board) = build();
  let root = world.root();
  let rules = FixedClearance::uniform(CLEARANCE);
  let options = CollisionSearchOptions::default();

  // --- the world the plain data describes -------------------------
  assert_eq!(
    world.all_items_in_net(root, SIGNAL, Kind::ARC),
    vec![board.arc]
  );

  let ItemBody::Arc(body) = world.item(board.arc).expect("live").body() else {
    panic!("the middle item is an arc");
  };

  assert_eq!(body.arc(), ARC);
  assert_eq!(body.anchor(0), ARC.start());
  assert_eq!(body.anchor(1), ARC.end());

  // Both of its anchors made a joint, each shared with the straight run
  // that meets it there.
  assert_eq!(
    joint_links(&world, ARC.start()),
    [board.west, board.arc],
    "the west joint holds the segment and the arc"
  );
  assert_eq!(
    joint_links(&world, ARC.end()),
    [board.arc, board.east],
    "the east joint holds the arc and the segment"
  );

  // --- the arc as an obstacle --------------------------------------
  //
  // A line crossing the middle of the curve, well away from either
  // straight run, has to answer the arc and nothing else.
  let crossing = probe(&[Vec2::new(0, 80000), Vec2::new(80000, 80000)]);
  let found = world
    .nearest_obstacle(root, &crossing, &rules, &options, CornerMode::Mitered45)
    .expect("the line crosses the arc");

  assert_eq!(found.item, Some(board.arc));
  assert!(found.found_intersection);

  let hull = found.hull.as_ref().expect("a live obstacle has a hull");

  assert!(hull.point_on_edge(found.ip_first, 0));
  // The hull is a real boundary and not the empty chain an arc whose
  // hull could not be built would answer with.
  assert!(hull.point_count() > 2);

  // A line on the other side of the board meets nothing.
  let clear = probe(&[Vec2::new(-50000, 200_000), Vec2::new(50000, 200_000)]);

  assert!(
    world
      .nearest_obstacle(root, &clear, &rules, &options, CornerMode::Mitered45)
      .is_none()
  );

  // --- the arc as a link -------------------------------------------
  //
  // Seeded at either straight run, the assembled line runs the whole way
  // and carries the arc with its three points intact.
  for (seed, first, last) in
    [(board.west, WEST, EAST), (board.east, WEST, EAST)]
  {
    let line = world.assemble_line(root, seed, None, false, false, true);

    assert_eq!(line.links(), [board.west, board.arc, board.east]);
    assert_eq!(line.point(0), first);
    assert_eq!(line.last_point(), Some(last));

    let arcs: Vec<ShapeArc> =
      line.shape().live_arcs().map(|(_, arc)| *arc).collect();

    assert_eq!(arcs.len(), 1, "the line carries one arc");
    // A chain's copy of an arc always has width zero
    // (`shape_line_chain.cpp:1624`), so only the points are compared.
    assert_eq!(arcs[0].start(), ARC.start());
    assert_eq!(arcs[0].arc_mid(), ARC.arc_mid());
    assert_eq!(arcs[0].end(), ARC.end());

    // The arc contributes its approximation and no corner of its own, so
    // the chain is longer than the three shapes it is made of, and the
    // one link per shape invariant (note 02 section 2.4) still holds.
    assert!(line.point_count() > 4);
    assert_eq!(line.shape_count(), 3);
    assert!(line.is_linked_checked());
  }

  // --- removing it -------------------------------------------------
  world.remove(root, board.arc);

  assert!(world.item(board.arc).is_none());
  assert!(
    world.all_items_in_net(root, SIGNAL, Kind::ARC).is_empty(),
    "the index forgot the arc"
  );
  assert_eq!(
    joint_links(&world, ARC.start()),
    [board.west],
    "the west joint kept only the segment"
  );
  assert_eq!(
    joint_links(&world, ARC.end()),
    [board.east],
    "the east joint kept only the segment"
  );

  // What is left is two separate lines, neither of which reaches the
  // other any more.
  let west_line =
    world.assemble_line(root, board.west, None, false, false, true);

  assert_eq!(west_line.links(), [board.west]);
  assert_eq!(west_line.shape().arc_count(), 0);
}

#[test]
fn a_recording_of_an_arc_board_round_trips_through_its_text_form() {
  let recording = SessionRecording::new(
    snapshot(),
    RoutingSettings::default(),
    Sizes::default(),
  );

  let text = recording.to_text();

  assert!(
    text
      .lines()
      .any(|line| line.contains(" arc 0 0 29289 70711")),
    "the arc geometry is written in KiCad's log form:\n{text}"
  );

  let parsed =
    SessionRecording::from_text(&text).expect("the recording reads back");

  assert_eq!(parsed.snapshot, recording.snapshot);
  assert_eq!(parsed.to_text(), text);

  // And the world built from the parsed snapshot is the same world.
  let (original, _) = World::from_snapshot(&recording.snapshot);
  let (replayed, _) = World::from_snapshot(&parsed.snapshot);
  let root = original.root();

  assert_eq!(
    original.all_items_in_net(root, SIGNAL, Kind::ARC).len(),
    replayed
      .all_items_in_net(replayed.root(), SIGNAL, Kind::ARC)
      .len()
  );
}
