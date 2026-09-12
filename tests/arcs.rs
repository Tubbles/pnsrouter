// SPDX-License-Identifier: GPL-3.0-or-later

//! The slice 5 exit test of `doc/work/012-arcs.md`: an arc track through
//! the world, driven through the public API only.
//!
//! An arc is an obstacle and a link like any other from here on, and that
//! claim is only worth something if it holds from outside the crate: a
//! host builds the snapshot, the world stores the arc, a query finds it,
//! a line assembly picks it up whole, a removal forgets it everywhere,
//! and a recording of the same board reads back byte for byte.
//!
//! The second half is the slice 6 exit test: the placer in a rounded
//! corner mode routes past an obstacle, the preview it hands the host
//! carries an arc, and the commit is pinned as it stands until slice 7
//! teaches `fix_route` to emit one.

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::collide::CollisionSearchOptions;
use pnsrouter::eventlog::SessionRecording;
use pnsrouter::geometry::arc::ShapeArc;
use pnsrouter::geometry::direction45::CornerMode;
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{
  HostId, ItemBody, ItemId, Kind, LayerRange, NetId, Solid,
};
use pnsrouter::line::Line;
use pnsrouter::node::World;
use pnsrouter::placer::line_placer::LinePlacer;
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RouterMode, RoutingSettings, Sizes};
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

// =====================================================================
// Slice 6: the placer produces arcs
// =====================================================================
//
// A second fixture, three pads in a row with the middle one on another
// net, routed in the two rounded corner modes. It is the slice 6 exit
// test of `doc/work/012-arcs.md`: `build_initial_trace` is the only
// producer of arcs in the router, and this is where its output has to
// survive the walkaround, the optimizer and the commit.

/// The clearance the rounded scenarios route to.
const ROUTE_CLEARANCE: i32 = 100_000;

/// The width of the routed track.
const ROUTE_WIDTH: i32 = 200_000;

/// The copper radius of every pad of the rounded scenarios.
const PAD_RADIUS: i32 = 400_000;

/// The net the routed track is on.
const ROUTE_NET: Option<NetId> = Some(NetId(11));

/// The net of the pad in the way, so that it is never exempt.
const IN_THE_WAY_NET: Option<NetId> = Some(NetId(12));

/// Where the routed track starts.
const ROUTE_START: Vec2 = Vec2::new(0, 0);

/// The pad the route has to get past.
const IN_THE_WAY: Vec2 = Vec2::new(2_000_000, 500_000);

/// Where the routed track ends. Neither axis aligned nor diagonal from
/// [`ROUTE_START`], so `build_initial_trace` has a corner to round.
const ROUTE_TARGET: Vec2 = Vec2::new(4_000_000, 2_000_000);

/// How far a track's centreline has to stay from a pad's centre.
const KEEP_OUT: i32 = PAD_RADIUS + ROUTE_CLEARANCE + ROUTE_WIDTH / 2;

/// The three pad board, with the start and target handles.
fn rounded_board() -> (World, ItemId, ItemId) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let add_pad = |world: &mut World, at: Vec2, net| {
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);

    world.add_solid(root, item, None)
  };

  let start_pad = add_pad(&mut world, ROUTE_START, ROUTE_NET);

  add_pad(&mut world, IN_THE_WAY, IN_THE_WAY_NET);

  let target_pad = add_pad(&mut world, ROUTE_TARGET, ROUTE_NET);

  (world, start_pad, target_pad)
}

/// Walkaround settings in one corner mode.
fn rounded_settings(corner_mode: CornerMode) -> RoutingSettings {
  RoutingSettings {
    mode: RouterMode::Walkaround,
    corner_mode,
    ..RoutingSettings::default()
  }
}

/// The sizes the rounded scenarios place with.
fn rounded_sizes() -> Sizes {
  let mut sizes = Sizes {
    track_width: ROUTE_WIDTH,
    ..Sizes::default()
  };

  sizes.add_layer_pair(0, 1);
  sizes
}

/// The squared distance from a point to the pad in the way.
fn squared_distance_to_the_obstacle(point: Vec2) -> i64 {
  let delta = point - IN_THE_WAY;

  i64::from(delta.x) * i64::from(delta.x)
    + i64::from(delta.y) * i64::from(delta.y)
}

/// Route the scenario in one corner mode and hand back the placer, the
/// world and the preview trace.
fn route_rounded(corner_mode: CornerMode) -> (World, LinePlacer, ItemId, Line) {
  let (mut world, start_pad, target_pad) = rounded_board();
  let rules = FixedClearance::uniform(ROUTE_CLEARANCE);
  let settings = rounded_settings(corner_mode);
  let context = AlgoContext::new(&rules, &settings);
  let mut placer =
    LinePlacer::new(&world, world.root(), &settings, rounded_sizes());

  assert!(placer.start(&mut world, &context, ROUTE_START, Some(start_pad)));
  assert!(placer.move_to(&mut world, &context, ROUTE_TARGET, None));

  let trace = placer.trace().expect("a placement is running");

  (world, placer, target_pad, trace)
}

#[test]
fn a_rounded_walkaround_carries_an_arc_and_clears_the_obstacle() {
  for corner_mode in [CornerMode::Rounded45, CornerMode::Rounded90] {
    let (world, _placer, _target_pad, trace) = route_rounded(corner_mode);
    let chain = trace.shape();

    // The whole point of the slice: the preview the host draws holds a
    // real arc, not a polyline that looks like one.
    assert!(
      chain.arc_count() >= 1,
      "{corner_mode:?} produced no arc: {:?}",
      chain.points()
    );

    // Every arc's endpoints are exact vertices of the chain, so a host
    // that draws the arcs and a host that draws the polyline agree at
    // the joins. `live_arcs` skips an arc no vertex refers to, which is
    // what erratum E13 is about.
    for (arc_index, arc) in chain.live_arcs() {
      let start = (0..chain.point_count())
        .find(|&vertex| {
          chain.is_arc_start(vertex)
            && chain.arc_index(vertex) == Some(arc_index)
        })
        .expect("a live arc has a first vertex");
      let end = (start..chain.point_count())
        .find(|&vertex| chain.is_arc_end(vertex))
        .expect("an arc that starts has an end");

      assert_eq!(chain.point(start), arc.start(), "{corner_mode:?}");
      assert_eq!(chain.point(end), arc.end(), "{corner_mode:?}");
      assert!(end > start + 1, "{corner_mode:?} arc of one chord");
      assert_ne!(arc.start(), arc.end(), "{corner_mode:?}");
      assert_ne!(arc.start(), arc.arc_mid(), "{corner_mode:?}");
    }

    // The route runs from the start pad to the cursor and stays out of
    // the pad in the way, arc and all.
    assert_eq!(chain.point(0), ROUTE_START, "{corner_mode:?}");
    assert_eq!(chain.last_point(), Some(ROUTE_TARGET), "{corner_mode:?}");

    for index in 0..chain.point_count() {
      assert!(
        squared_distance_to_the_obstacle(chain.point(index))
          >= i64::from(KEEP_OUT) * i64::from(KEEP_OUT),
        "{corner_mode:?} passes through the obstacle's keep out at {:?}",
        chain.point(index)
      );
    }

    let _ = world;
  }
}

#[test]
fn fix_route_commits_an_arc_as_its_chords_until_slice_7() {
  // Interim behaviour, pinned so that slice 7's change shows up in this
  // test's diff. `LINE_PLACER::FixRoute` walks the trace segment by
  // index (`pcbnew/router/pns_line_placer.cpp:1669`), and slice 6 stops
  // short of porting KiCad's arc emission loop, so an arc reaches the
  // node as the straight chords of its own approximation, one `SEGMENT`
  // each. Slice 7 replaces this with one `ARC` per arc and erratum E24
  // fixed; when it does, the two assertions below flip.
  for corner_mode in [CornerMode::Rounded45, CornerMode::Rounded90] {
    let (mut world, mut placer, target_pad, trace) = route_rounded(corner_mode);
    let rules = FixedClearance::uniform(ROUTE_CLEARANCE);
    let settings = rounded_settings(corner_mode);
    let context = AlgoContext::new(&rules, &settings);
    let arcs_in_the_preview = trace.shape().arc_count();
    let chords = trace.segment_count();
    let shapes = trace.shape().shape_count();

    assert!(arcs_in_the_preview >= 1, "{corner_mode:?}");
    assert!(placer.fix_route(
      &mut world,
      &context,
      ROUTE_TARGET,
      Some(target_pad),
      false
    ));

    let node = placer.last_node().expect("the fix wrote into a branch");
    let committed_arcs = world.all_items_in_net(node, ROUTE_NET, Kind::ARC);
    let committed_segments =
      world.all_items_in_net(node, ROUTE_NET, Kind::SEGMENT);

    // Interim: no arc reaches the node, and the approximation chords are
    // there one by one instead. The count is not exactly the chord count
    // because `simplifyNewLine` merges the collinear pairs it finds at
    // the joints afterwards (`pcbnew/router/pns_line_placer.cpp:1894`).
    assert!(
      committed_arcs.is_empty(),
      "{corner_mode:?} committed an arc, which is slice 7's job"
    );
    assert!(
      committed_segments.len() > shapes,
      "{corner_mode:?} committed {} segments for {shapes} shapes, so the \
       arc did not reach the node whole",
      committed_segments.len()
    );
    assert!(
      committed_segments.len() <= chords,
      "{corner_mode:?} committed more segments than the trace had chords"
    );

    // What must hold either way: the commit is clear.
    for id in committed_segments {
      let line = Line::from_segment(&world, node, id)
        .expect("a stored segment assembles");

      assert!(
        world
          .check_colliding_line(
            node,
            &line,
            &rules,
            &CollisionSearchOptions::default()
          )
          .is_none(),
        "{corner_mode:?} committed a colliding segment from {:?} to {:?}",
        line.point(0),
        line.last_point()
      );
    }
  }
}
