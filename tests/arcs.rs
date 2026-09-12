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
//! The second part is the slice 6 exit test: the placer in a rounded
//! corner mode routes past an obstacle and the preview it hands the host
//! carries an arc.
//!
//! The third is the slice 7 exit test: the commit path. A rounded route
//! reaches the board as one `ARC` per fillet rather than as the chords
//! of its approximation, an existing track is shoved out of its way, an
//! arc track is not (the whole of the erratum E27 reachability finding),
//! the straight between two arcs survives the commit (erratum E24), and
//! a session that committed an arc replays from its text form to the
//! same commit.

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::collide::CollisionSearchOptions;
use pnsrouter::eventlog::{SessionRecording, replay};
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
use pnsrouter::router::{CommitDiff, FixOutcome, NewGeometry, Router};
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

// =====================================================================
// Slice 7: the commit path and the shove
// =====================================================================
//
// The slice 7 exit tests of `doc/work/012-arcs.md`. Everything below
// drives the facade, so what is checked is what a host sees: an arc in
// the commit diff, an arc item in the committed board, and an existing
// track pushed out of the way to make room for it.

/// The pad a slice 7 route starts on.
const SHOVE_START_PAD: HostId = HostId(21);

/// The pad a slice 7 route ends on.
const SHOVE_TARGET_PAD: HostId = HostId(22);

/// The straight track a rounded shove has to push aside.
const SHOVE_TRACK: HostId = HostId(23);

/// Where a slice 7 route starts.
const SHOVE_START: Vec2 = Vec2::new(0, 0);

/// Where a slice 7 route ends. Neither axis aligned nor diagonal from
/// [`SHOVE_START`], so the trace has a corner to round.
const SHOVE_TARGET: Vec2 = Vec2::new(6_000_000, 3_000_000);

/// One end of the track in the way, which the route crosses.
const SHOVE_TRACK_A: Vec2 = Vec2::new(3_000_000, -2_000_000);

/// The other end of it.
const SHOVE_TRACK_B: Vec2 = Vec2::new(3_000_000, 5_000_000);

/// A board with two pads and one straight track lying across the route.
fn shove_board() -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);
  let pad = |id: HostId, at: Vec2| {
    WorldItem::new(
      id,
      ROUTE_NET,
      LayerRange::single(0),
      WorldGeometry::Solid {
        shape: Shape::circle(at, PAD_RADIUS),
        pos: at,
        offset: Vec2::new(0, 0),
        orientation_degrees: 0.0,
        anchors: vec![at],
      },
    )
  };

  snapshot.items.push(pad(SHOVE_START_PAD, SHOVE_START));
  snapshot.items.push(pad(SHOVE_TARGET_PAD, SHOVE_TARGET));
  snapshot.items.push(WorldItem::new(
    SHOVE_TRACK,
    IN_THE_WAY_NET,
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(SHOVE_TRACK_A, SHOVE_TRACK_B),
      width: ROUTE_WIDTH,
    },
  ));

  snapshot
}

/// A router over a snapshot, in shove mode and one corner mode.
fn shove_router(snapshot: &WorldSnapshot, corner_mode: CornerMode) -> Router {
  let settings = RoutingSettings {
    mode: RouterMode::Shove,
    corner_mode,
    ..RoutingSettings::default()
  };

  Router::new(
    snapshot,
    Box::new(FixedClearance::uniform(ROUTE_CLEARANCE)),
    settings,
    rounded_sizes(),
  )
}

/// Route [`SHOVE_START`] to [`SHOVE_TARGET`] and finish there.
fn route_shove_scenario(router: &mut Router) -> CommitDiff {
  router
    .start_routing(SHOVE_START, Some(SHOVE_START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(SHOVE_TARGET, Some(SHOVE_TARGET_PAD));

  match router.fix_route(SHOVE_TARGET, Some(SHOVE_TARGET_PAD), false) {
    FixOutcome::Finished(diff) => diff,
    FixOutcome::Continue(_) => panic!("a fix on the target pad finishes"),
  }
}

/// Every geometry a commit diff adds on one net.
fn added_on(diff: &CommitDiff, net: Option<NetId>) -> Vec<NewGeometry> {
  diff
    .added
    .iter()
    .filter(|item| item.net == net)
    .map(|item| item.geometry)
    .collect()
}

/// Every arc item of one net stored in a world.
fn stored_arcs(world: &World, net: Option<NetId>) -> Vec<ShapeArc> {
  world
    .all_items_in_net(world.root(), net, Kind::ARC)
    .into_iter()
    .filter_map(|id| match world.item(id)?.body() {
      ItemBody::Arc(body) => Some(body.arc()),
      _ => None,
    })
    .collect()
}

#[test]
fn a_rounded_shove_commits_an_arc_and_pushes_a_track_aside() {
  let snapshot = shove_board();
  let mut router = shove_router(&snapshot, CornerMode::Rounded45);
  let diff = route_shove_scenario(&mut router);

  // The route is one corner, so `build_initial_trace` gives it one
  // fillet, and `fix_route` commits that fillet as **one** `ARC` rather
  // than as the segments of its own approximation. The three points are
  // pinned as exact integers, because an arc a nanometre away is a
  // different arc and nothing downstream would notice the difference.
  let routed = added_on(&diff, ROUTE_NET);
  let fillet = NewGeometry::Arc {
    start: SHOVE_START,
    mid: Vec2::new(2_771_639, 551_313),
    end: Vec2::new(5_121_321, 2_121_320),
    width: ROUTE_WIDTH,
  };

  assert_eq!(
    routed,
    vec![
      fillet,
      NewGeometry::Segment {
        seg: Seg::new(Vec2::new(5_121_321, 2_121_320), SHOVE_TARGET),
        width: ROUTE_WIDTH,
      },
    ],
    "the rounded route did not commit as one arc and one segment"
  );

  // The same arc is in the committed board, as an `ItemBody::Arc` and
  // not as a polyline of it.
  let world = router.world();

  assert_eq!(
    stored_arcs(world, ROUTE_NET),
    vec![ShapeArc::new(
      SHOVE_START,
      Vec2::new(2_771_639, 551_313),
      Vec2::new(5_121_321, 2_121_320),
      ROUTE_WIDTH,
    )],
    "the committed board does not hold the fillet as an arc"
  );

  // And the straight track that was lying across the route has been
  // pushed out of the way: the host is told to rewrite it, and what it
  // becomes is shorter than what it was, the rest of the detour arriving
  // as fresh items on the same net.
  let (_, shoved) = diff
    .updated
    .iter()
    .find(|(host, _)| *host == SHOVE_TRACK)
    .expect("the shove rewrote the track in the way");

  let NewGeometry::Segment { seg, .. } = shoved.geometry else {
    panic!("a shoved straight track stays a straight track");
  };

  assert_eq!(seg.a, SHOVE_TRACK_A, "a shove may not move a fixed end");
  assert_ne!(
    seg.b, SHOVE_TRACK_B,
    "the track in the way was left where it was"
  );
  assert!(
    !added_on(&diff, IN_THE_WAY_NET).is_empty(),
    "the detour the shove walked reached no items"
  );
  assert!(diff.removed.is_empty(), "{:?}", diff.removed);
}

/// The arc track a rounded shove meets but cannot see.
const ARC_IN_THE_WAY: ShapeArc = ShapeArc::new(
  Vec2::new(2_500_000, -2_000_000),
  Vec2::new(3_500_000, 1_500_000),
  Vec2::new(2_500_000, 5_000_000),
  ROUTE_WIDTH,
);

/// [`shove_board`] with the straight track replaced by an arc track.
fn arc_obstacle_board() -> WorldSnapshot {
  let mut snapshot = shove_board();

  snapshot.items.pop();
  snapshot.items.push(WorldItem::new(
    SHOVE_TRACK,
    IN_THE_WAY_NET,
    LayerRange::single(0),
    WorldGeometry::Arc {
      start: ARC_IN_THE_WAY.start(),
      mid: ARC_IN_THE_WAY.arc_mid(),
      end: ARC_IN_THE_WAY.end(),
      width: ROUTE_WIDTH,
    },
  ));

  snapshot
}

#[test]
fn a_rounded_shove_cannot_see_an_arc_track_erratum_e27() {
  // The decision this test names: KiCad's shove never asks for `ARC_T`.
  // `shoveIteration`'s search runs one pass per kind over
  // `{ SOLID_T, VIA_T, SEGMENT_T, HOLE_T }`
  // (`pcbnew/router/pns_shove.cpp:1650`), `ITEM::OfKind` is a bitwise
  // `( aKindMask & m_kind ) != 0` (`pcbnew/router/pns_item.h:181`), and
  // `SEGMENT_T` (8) and `ARC_T` (16) are different bits, so
  // `NODE::NearestObstacle`'s visitor drops every arc at
  // `pcbnew/router/pns_node.cpp:243`. The port reproduces that, so an arc
  // track is invisible to the shove and the route is committed straight
  // through it. See `doc/log/2026-09-12.md`.
  let snapshot = arc_obstacle_board();
  let mut router = shove_router(&snapshot, CornerMode::Rounded45);
  let diff = route_shove_scenario(&mut router);

  // Nothing happened to the arc: the host is told to change nothing on
  // its net, and the board still holds it point for point.
  assert!(
    !diff.updated.iter().any(|(host, _)| *host == SHOVE_TRACK),
    "the shove rewrote an arc track it cannot see"
  );
  assert!(diff.removed.is_empty(), "{:?}", diff.removed);
  assert!(added_on(&diff, IN_THE_WAY_NET).is_empty());
  assert_eq!(
    stored_arcs(router.world(), IN_THE_WAY_NET),
    vec![ARC_IN_THE_WAY]
  );

  // And the route went where it would have gone on an empty board, which
  // is through the arc. `LINE_PLACER::FixRoute`'s collision gate lets it
  // through because in shove mode that gate blocks on solids alone
  // (`pcbnew/router/pns_line_placer.cpp:1596`), so the overlap reaches
  // the board and only the host's own violation list sees it.
  let world = router.world();
  let root = world.root();
  let rules = FixedClearance::uniform(ROUTE_CLEARANCE);
  let seed = *world
    .all_items_in_net(root, ROUTE_NET, Kind::ARC)
    .first()
    .expect("the rounded route committed an arc");
  let route = world.assemble_line(root, seed, None, false, false, true);
  let hit = world
    .check_colliding_line(
      root,
      &route,
      &rules,
      &CollisionSearchOptions::default(),
    )
    .and_then(|obstacle| obstacle.item)
    .and_then(|id| world.item(id));

  assert!(
    hit.is_some_and(|item| item.of_kind(Kind::ARC)),
    "the committed route does not overlap the arc, so the shove saw it \
     after all and this test no longer names the decision"
  );

  // The contrast, which is what makes the decision a shove one rather
  // than a world one: the **walkaround** does see arcs, because
  // `NearestObstacle` is asked with no kind filter there, so the same
  // board routed in walkaround mode commits nothing that overlaps it.
  let settings = RoutingSettings {
    mode: RouterMode::Walkaround,
    corner_mode: CornerMode::Rounded45,
    ..RoutingSettings::default()
  };
  let mut walker = Router::new(
    &snapshot,
    Box::new(FixedClearance::uniform(ROUTE_CLEARANCE)),
    settings,
    rounded_sizes(),
  );

  route_shove_scenario(&mut walker);

  let world = walker.world();
  let root = world.root();
  let seed = *world
    .all_items_in_net(root, ROUTE_NET, Kind::SEGMENT | Kind::ARC)
    .first()
    .expect("the walkaround committed something");
  let route = world.assemble_line(root, seed, None, false, false, true);

  assert!(
    world
      .check_colliding_line(
        root,
        &route,
        &rules,
        &CollisionSearchOptions::default()
      )
      .is_none(),
    "the walkaround committed a route that overlaps the arc"
  );
}

/// The pads the erratum E24 scenario has to walk around, and their net.
const E24_PADS: [Vec2; 3] = [
  Vec2::new(2_000_000, 500_000),
  Vec2::new(7_000_000, 4_000_000),
  Vec2::new(14_000_000, 8_000_000),
];

/// The cursor positions the erratum E24 scenario moves through.
const E24_CURSORS: [Vec2; 2] = [
  Vec2::new(2_000_000, 1_000_000),
  Vec2::new(4_000_000, 2_000_000),
];

/// The kind of every shape of a chain, in chain order.
fn shape_kinds(chain: &LineChain) -> Vec<Kind> {
  let mut kinds = Vec::new();
  let mut index = Some(0usize);

  while let Some(vertex) = index {
    if vertex + 1 >= chain.point_count() {
      break;
    }

    kinds.push(if chain.is_arc_segment(vertex) {
      Kind::ARC
    } else {
      Kind::SEGMENT
    });
    index = chain.next_shape(vertex);
  }

  kinds
}

#[test]
fn fix_route_commits_the_straight_between_two_arcs_erratum_e24() {
  // KiCad's emission loop decides per vertex with `ArcIndex( i ) < 0`
  // (`pcbnew/router/pns_line_placer.cpp:1671`), which is true at an
  // arc's **last** point even when the shape leaving it is straight. The
  // else branch then runs, `arcIndex == lastArc` holds, and the
  // `continue` at `:1688` swallows that straight segment; the rescue at
  // `:1673` only covers the case where it is the last shape of the
  // trace. So a chain shaped arc, straight, arc commits the two arcs and
  // loses the straight between them, leaving a gap in the track.
  //
  // The fix is to ask `IsArcSegment`, "does the shape leaving this
  // vertex curve", which is what the loop is about. This scenario builds
  // exactly that chain: `build_initial_trace` yields at most two shapes,
  // so the only way to it is `mergeHead`'s `tail.Append( head )`
  // (`:371`), which the walkaround reaches after two cursor moves past a
  // row of pads.
  let rules = FixedClearance::uniform(ROUTE_CLEARANCE);
  let settings = RoutingSettings {
    mode: RouterMode::Walkaround,
    corner_mode: CornerMode::Rounded45,
    ..RoutingSettings::default()
  };
  let context = AlgoContext::new(&rules, &settings);
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();

  for at in E24_PADS {
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(IN_THE_WAY_NET);
    world.add_solid(root, item, None);
  }

  let mut placer = LinePlacer::new(&world, root, &settings, rounded_sizes());

  assert!(placer.start(&mut world, &context, ROUTE_START, None));

  for cursor in E24_CURSORS {
    placer.move_to(&mut world, &context, cursor, None);
  }

  let trace = placer.trace().expect("a placement is running");
  let kinds = shape_kinds(trace.shape());
  let first_point = trace.point(0);
  let last_point = trace.last_point().expect("a trace has an end");
  let net = placer.current_net();

  // The scenario is only worth anything if it built the erratum's own
  // shape: an arc, then at least one straight, then another arc.
  assert_eq!(
    kinds,
    vec![
      Kind::ARC,
      Kind::SEGMENT,
      Kind::SEGMENT,
      Kind::ARC,
      Kind::SEGMENT
    ],
    "the scenario stopped producing the arc, straight, arc chain"
  );
  assert!(
    trace.shape().is_arc_end(12) && !trace.shape().is_arc_segment(12),
    "vertex 12 is no longer the arc end a straight leaves, which is the \
     exact vertex erratum E24 loses"
  );

  let last_cursor = *E24_CURSORS.last().expect("the cursor list is not empty");

  assert!(placer.fix_route(&mut world, &context, last_cursor, None, true));

  let node = placer.last_node().expect("the fix wrote into a branch");
  let committed = world.all_items_in_net(node, net, Kind::SEGMENT | Kind::ARC);

  assert_eq!(
    committed.len(),
    kinds.len(),
    "the commit has one item per shape"
  );

  // Assembled from either end, the committed track runs the whole way
  // and its links are the trace's shapes, in order and one for one.
  // Under the erratum the second link is missing, the two arcs are not
  // joined, and the assembly stops at the gap.
  let line = world.assemble_line(
    node,
    *committed.first().expect("the commit stored something"),
    None,
    false,
    false,
    true,
  );

  assert_eq!(line.point(0), first_point);
  assert_eq!(line.last_point(), Some(last_point));
  assert_eq!(
    line
      .links()
      .iter()
      .filter_map(|id| world.item(*id).map(pnsrouter::item::Item::kind))
      .collect::<Vec<Kind>>(),
    kinds
  );
  assert!(line.is_linked_checked());
}

#[test]
fn a_rounded_session_that_commits_an_arc_replays_to_the_same_commit() {
  let snapshot = shove_board();
  let mut router = shove_router(&snapshot, CornerMode::Rounded45);

  router.start_recording(&snapshot);

  let committed = route_shove_scenario(&mut router);
  let recording = router.take_recording().expect("a recording was running");
  let text = recording.to_text();

  // The commit carried an arc, so the recording has to have written one,
  // in the same `{ start, mid, end, width }` token form the snapshot side
  // uses and KiCad's own router log writes
  // (`pcbnew/router/pns_logger.cpp:245`).
  assert!(!added_on(&committed, ROUTE_NET).is_empty());
  assert!(
    text
      .lines()
      .any(|line| line.contains("arc 0 0 2771639 551313")),
    "the committed arc is not in the recording:\n{text}"
  );

  // The text form is an exact round trip, and replaying the parsed
  // recording reproduces the commit item for item, arc included.
  let read = SessionRecording::from_text(&text).expect(&text);

  assert_eq!(read, recording, "{text}");
  assert_eq!(read.to_text(), text);
  assert_eq!(
    replay(&read, Box::new(FixedClearance::uniform(ROUTE_CLEARANCE))).diffs,
    vec![committed]
  );
}
