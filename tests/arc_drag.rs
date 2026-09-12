// SPDX-License-Identifier: GPL-3.0-or-later

//! Dragging an arc track, through the public API only.
//!
//! The slice 8 exit test of `doc/work/012-arcs.md`. `DRAGGER::startDragArc`
//! (`pcbnew/router/pns_dragger.cpp:155`) and `LINE::DragArc`
//! (`pcbnew/router/pns_line.cpp:911`) are the two routines under test, and
//! KiCad has no test of either: its regression corpus contains no case
//! that touches an arc at all (note 09 section 8.3), and none of its four
//! PNS unit tests constructs one. So the contract here is the port's own.
//!
//! What each scenario asserts is that the drag **moves** the arc and
//! leaves it an arc: three exact integer points that a host can store, a
//! chord the cursor pulled towards itself, and neighbours that still meet
//! it end to end.
//!
//! The board is the one `tests/arcs.rs` uses, a straight run into a
//! quarter turn and a straight run out of it, because the interesting
//! cases are all about what the arc's two neighbours do.

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::dragger::{DragMode, Dragger};
use pnsrouter::geometry::arc::ShapeArc;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{HostId, ItemBody, ItemId, Kind, LayerRange, NetId};
use pnsrouter::node::{NodeId, World};
use pnsrouter::router::{FixOutcome, NewGeometry, Router};
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RouterMode, RoutingSettings, Sizes};
use pnsrouter::snapshot::{WorldGeometry, WorldItem, WorldSnapshot};

/// The clearance every scenario drags to, in nanometres.
const CLEARANCE: i32 = 100_000;

/// The width of every track on the board.
const TRACK_WIDTH: i32 = 200_000;

/// The net every track is on.
const SIGNAL: Option<NetId> = Some(NetId(1));

/// Where the straight run into the arc starts.
const WEST: Vec2 = Vec2::new(-2_000_000, 0);

/// Where the straight run out of the arc ends.
const EAST: Vec2 = Vec2::new(2_000_000, 3_000_000);

/// The arc track: a quarter turn about `(0, 1000000)` of radius one
/// millimetre, from the origin heading east round to
/// `(1000000, 1000000)` heading north.
const ARC: ShapeArc = ShapeArc::new(
  Vec2::new(0, 0),
  Vec2::new(707_107, 292_893),
  Vec2::new(1_000_000, 1_000_000),
  TRACK_WIDTH,
);

/// The straight run into the arc.
const WEST_TRACK: HostId = HostId(1);

/// The arc.
const ARC_TRACK: HostId = HostId(2);

/// The straight run out of the arc.
const EAST_TRACK: HostId = HostId(3);

/// The board as plain data, the way a host hands it over.
fn snapshot() -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(WorldItem::new(
    WEST_TRACK,
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(WEST, ARC.start()),
      width: TRACK_WIDTH,
    },
  ));
  snapshot.items.push(WorldItem::new(
    ARC_TRACK,
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
    EAST_TRACK,
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(ARC.end(), EAST),
      width: TRACK_WIDTH,
    },
  ));

  snapshot
}

/// The world the snapshot describes, and the arc's handle in it.
fn build() -> (World, ItemId) {
  let snapshot = snapshot();
  let (world, index) = World::from_snapshot(&snapshot);
  let arc = *index
    .items_of(ARC_TRACK)
    .first()
    .expect("every snapshot item was stored");

  (world, arc)
}

/// The same arc as a chain stores it, with no width.
///
/// `SHAPE_LINE_CHAIN::Append( const SHAPE_ARC& )` zeroes the copy it
/// keeps (`libs/kimath/src/geometry/shape_line_chain.cpp:1625`).
fn stored(arc: ShapeArc) -> ShapeArc {
  ShapeArc::new(arc.start(), arc.arc_mid(), arc.end(), 0)
}

/// The rule oracle every scenario uses.
fn rules() -> FixedClearance {
  FixedClearance::uniform(CLEARANCE)
}

/// Settings in one routing mode, everything else at KiCad's defaults.
fn settings_for(mode: RouterMode) -> RoutingSettings {
  RoutingSettings {
    mode,
    ..RoutingSettings::default()
  }
}

/// Every arc a node's delta added.
///
/// More than one is normal after a walkaround or a shove drag:
/// `optimizeAndUpdateDraggedLine` splits the chain at the vertex it wants
/// preserved (`pcbnew/router/pns_dragger.cpp:594`), and a vertex that sits
/// on an arc splits the arc in two
/// (`libs/kimath/src/geometry/shape_line_chain.cpp`, `splitArc`). The two
/// halves are still two pieces of one circle.
fn added_arcs(world: &World, node: NodeId) -> Vec<ShapeArc> {
  let (added, _) = world.get_updated_items(node);

  added
    .into_iter()
    .filter_map(|id| match world.item(id)?.body() {
      ItemBody::Arc(body) => Some(body.arc()),
      _ => None,
    })
    .collect()
}

/// Every straight segment a node's delta added.
fn added_segments(world: &World, node: NodeId) -> Vec<Seg> {
  let (added, _) = world.get_updated_items(node);

  added
    .into_iter()
    .filter_map(|id| match world.item(id)?.body() {
      ItemBody::Segment(body) => Some(body.seg()),
      _ => None,
    })
    .collect()
}

/// Start a drag on the arc and move the cursor once.
fn drag_the_arc(
  world: &mut World,
  context: &AlgoContext<'_>,
  arc: ItemId,
  at: Vec2,
) -> (Dragger, bool) {
  let mut dragger = Dragger::new(world, world.root());

  assert!(
    dragger.start(world, context, ARC.arc_mid(), arc),
    "a quarter turn is draggable"
  );
  assert_eq!(dragger.mode(), DragMode::Arc);

  let moved = dragger.drag(world, context, at);

  (dragger, moved)
}

// ---------------------------------------------------------------------
// The mode decision
// ---------------------------------------------------------------------

/// `DRAGGER::Start`'s `ARC_T` case (`pcbnew/router/pns_dragger.cpp:351`)
/// dispatches to `startDragArc`, which sets `DM_ARC` (`:251`) and
/// assembles the whole run the arc belongs to.
#[test]
fn a_click_on_an_arc_starts_an_arc_drag() {
  let (mut world, arc) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = Dragger::new(&world, world.root());

  assert!(dragger.start(&mut world, &context, ARC.arc_mid(), arc));
  assert_eq!(dragger.mode(), DragMode::Arc);

  // West, the arc's approximation and east, in one line.
  let line = dragger.original_line();

  assert_eq!(line.shape().arc_count(), 1);
  assert_eq!(line.shape().arc(0), Some(stored(ARC)));
  assert_eq!(line.shape().points().first(), Some(&WEST));
  assert_eq!(line.shape().last_point(), Some(EAST));
}

/// `startDragArc`'s first act is to refuse an arc of half a turn or more
/// (`:161`), because the two tangents of such an arc meet on the wrong
/// side of it and the tangent circle construction has no solution.
#[test]
fn an_arc_of_half_a_turn_or_more_is_refused() {
  let mut snapshot = snapshot();
  let half = ShapeArc::new(
    Vec2::new(5_000_000, 0),
    Vec2::new(6_000_000, 1_000_000),
    Vec2::new(7_000_000, 0),
    TRACK_WIDTH,
  );

  snapshot.items.push(WorldItem::new(
    HostId(4),
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Arc {
      start: half.start(),
      mid: half.arc_mid(),
      end: half.end(),
      width: TRACK_WIDTH,
    },
  ));

  let (mut world, index) = World::from_snapshot(&snapshot);
  let id = *index
    .items_of(HostId(4))
    .first()
    .expect("every snapshot item was stored");

  assert!(
    (half.central_angle().as_degrees().abs() - 180.0).abs() < 1.0,
    "the fixture arc is a half turn"
  );

  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = Dragger::new(&world, world.root());

  assert!(!dragger.start(&mut world, &context, half.arc_mid(), id));
}

// ---------------------------------------------------------------------
// The three modes
// ---------------------------------------------------------------------

/// The arc drag in all three routing modes.
///
/// The arm the mode picks is `dragMarkObstacles`' `DM_ARC` (`:418`),
/// `dragWalkaround`'s (`:762`) or `dragShove`'s (`:865`); all three call
/// `LINE::DragArc` and then put the result in the node the same way, so
/// on a board with nothing in the way all three have to reach a committed
/// geometry that still holds one arc.
#[test]
fn an_arc_drags_in_all_three_modes() {
  for mode in [
    RouterMode::MarkObstacles,
    RouterMode::Walkaround,
    RouterMode::Shove,
  ] {
    let (mut world, arc) = build();
    let rules = rules();
    let settings = settings_for(mode);
    let context = AlgoContext::new(&rules, &settings);
    // Pull the arc's bulge out towards the corner the two tangents meet
    // at, which is the only region a smaller tangent circle can reach:
    // every inscribed circle smaller than the one the arc already is lies
    // between it and that corner.
    let target = Vec2::new(900_000, 300_000);
    let (mut dragger, moved) = drag_the_arc(&mut world, &context, arc, target);

    assert!(moved, "{mode:?}: the drag succeeded");

    let node = dragger
      .fix_route_node(&mut world, &context, true)
      .unwrap_or_else(|| panic!("{mode:?}: a successful drag commits"));
    let dragged = added_arcs(&world, node);

    assert!(!dragged.is_empty(), "{mode:?}: an arc reaches the board");

    let mut ends: Vec<Vec2> = Vec::new();

    for arc in &dragged {
      // Still an arc, and still on integer nanometres: the three points
      // are what a host stores.
      assert_ne!(*arc, stored(ARC), "{mode:?}: the arc moved");
      assert_ne!(arc.start(), arc.end(), "{mode:?}: the arc is not flat");

      // The cursor pulled the bulge out towards the corner the two
      // tangents meet at, so every piece of the new arc is on a smaller
      // circle than the old one.
      assert!(
        arc.radius() < ARC.radius(),
        "{mode:?}: {} against {}",
        arc.radius(),
        ARC.radius()
      );

      ends.push(arc.start());
      ends.push(arc.end());
    }

    // And the route is still one unbroken path from `WEST` to `EAST`
    // through the arc: every point where two pieces meet is used twice,
    // and only the two board ends are used once. The dragged arc's own
    // endpoints moved, so its neighbours had to be rebuilt to reach them.

    for seg in added_segments(&world, node) {
      ends.push(seg.a);
      ends.push(seg.b);
    }

    let mut loose: Vec<Vec2> = ends
      .iter()
      .filter(|point| {
        ends.iter().filter(|other| other == point).count() % 2 == 1
      })
      .copied()
      .collect();

    loose.sort_by_key(|point| (point.x, point.y));
    loose.dedup();

    assert_eq!(loose, vec![WEST, EAST], "{mode:?}: the route is unbroken");
  }
}

/// A drag that pulls the cursor onto the corner the two tangents meet at
/// collapses the arc: its new chord is shorter than
/// `ADVANCED_CFG::m_MaxTrackLengthToKeep`, so `DragArc` splices the chain
/// without it (`pcbnew/router/pns_line.cpp:1089` to `:1102`) and the
/// route keeps its corner as a plain vertex. KiCad's own comment at
/// `pns_dragger.cpp:426` calls that the intended outcome.
#[test]
fn a_collapsed_arc_drag_drops_the_arc() {
  let (mut world, arc) = build();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  // The two tangents meet at `(1000000, 0)`; ask for a point a hair
  // inside it, so the circle through it still exists and its chord comes
  // out under the 500 nm limit.
  let target = Vec2::new(999_929, 71);
  let (mut dragger, moved) = drag_the_arc(&mut world, &context, arc, target);

  assert!(moved);

  let node = dragger
    .fix_route_node(&mut world, &context, true)
    .expect("a mark obstacles drag always commits");

  assert!(
    added_arcs(&world, node).is_empty(),
    "no arc reaches the board"
  );
  assert!(
    !added_segments(&world, node).is_empty(),
    "the route is still there, as straight segments"
  );
}

// ---------------------------------------------------------------------
// The tangent stubs, erratum E12
// ---------------------------------------------------------------------

/// An arc with a free end gets a tangent stub segment in the pre drag
/// node so the drag has something to hinge on (`:198` to `:244`), and
/// those stubs are invisible to the collision probe, which tests against
/// the **root** (erratum E12 of note 06).
///
/// A lone arc on an otherwise empty board is the case: both of its ends
/// are free, so both stubs are built, and the assembled line is four
/// points where the world's own is two plus the approximation.
#[test]
fn an_isolated_arc_gets_tangent_stubs_erratum_e12() {
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(WorldItem::new(
    ARC_TRACK,
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Arc {
      start: ARC.start(),
      mid: ARC.arc_mid(),
      end: ARC.end(),
      width: TRACK_WIDTH,
    },
  ));

  let (mut world, index) = World::from_snapshot(&snapshot);
  let arc = *index
    .items_of(ARC_TRACK)
    .first()
    .expect("every snapshot item was stored");
  let root = world.root();

  // The root holds the arc and nothing else.
  assert_eq!(world.all_items_in_net(root, SIGNAL, Kind::ANY), vec![arc]);

  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = Dragger::new(&world, root);

  assert!(dragger.start(&mut world, &context, ARC.arc_mid(), arc));

  // The assembled line runs stub, arc, stub, so it starts and ends off
  // the arc's own endpoints by the stub length.
  let line = dragger.original_line();

  assert_eq!(line.shape().arc_count(), 1);
  assert_ne!(line.shape().points().first(), Some(&ARC.start()));
  assert_ne!(line.shape().last_point(), Some(ARC.end()));

  // Each stub is a quarter micrometre, half of
  // `m_MaxTrackLengthToKeep * IU_PER_MM`, and tangent to the arc: it
  // leaves the endpoint at a right angle to the radius.
  let first = line.shape().points()[0];
  let stub = first - ARC.start();

  assert_eq!(stub.euclidean_norm(), 250);
  assert_eq!(
    stub.dot(ARC.start() - ARC.center()),
    0,
    "the stub is perpendicular to the radius"
  );

  // Erratum E12: the stubs went into the pre drag node, and the root the
  // walkaround probe tests against still holds one item.
  assert_eq!(world.all_items_in_net(root, SIGNAL, Kind::ANY), vec![arc]);
}

// ---------------------------------------------------------------------
// The component drag
// ---------------------------------------------------------------------

/// A component drag of a pad with an arc track attached moves the arc.
///
/// `COMPONENT_DRAGGER`'s `ARC_T` case
/// (`pcbnew/router/pns_component_dragger.cpp:210`) clones the arc, calls
/// `SHAPE_ARC::Move` on the clone and puts it back, which is the fixed
/// item path: a track that runs between two dragged pads travels with
/// them rigidly rather than being re-routed.
#[test]
fn a_component_drag_moves_an_arc_between_two_pads() {
  let pad_radius = 300_000;
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);
  let pad = |id: HostId, at: Vec2| {
    WorldItem::new(
      id,
      SIGNAL,
      LayerRange::single(0),
      WorldGeometry::Solid {
        shape: Shape::circle(at, pad_radius),
        pos: at,
        offset: Vec2::new(0, 0),
        orientation_degrees: 0.0,
        anchors: Vec::new(),
      },
    )
  };

  snapshot.items.push(pad(HostId(10), ARC.start()));
  snapshot.items.push(pad(HostId(11), ARC.end()));
  snapshot.items.push(WorldItem::new(
    ARC_TRACK,
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Arc {
      start: ARC.start(),
      mid: ARC.arc_mid(),
      end: ARC.end(),
      width: TRACK_WIDTH,
    },
  ));

  let mut router = Router::new(
    &snapshot,
    Box::new(rules()),
    settings_for(RouterMode::Shove),
    Sizes::default(),
  );
  let delta = Vec2::new(1_500_000, -500_000);

  router
    .start_dragging(ARC.start(), &[HostId(10), HostId(11)], false)
    .expect("two pads start a component drag");
  router.move_to(ARC.start() + delta, None);

  let FixOutcome::Finished(diff) =
    router.fix_route(ARC.start() + delta, None, true)
  else {
    panic!("a component drag commits");
  };
  let moved = diff
    .added
    .iter()
    .chain(diff.updated.iter().map(|(_, item)| item))
    .find_map(|item| match item.geometry {
      NewGeometry::Arc {
        start, mid, end, ..
      } => Some((start, mid, end)),
      _ => None,
    })
    .expect("the arc reaches the commit");

  assert_eq!(moved.0, ARC.start() + delta);
  assert_eq!(moved.1, ARC.arc_mid() + delta);
  assert_eq!(moved.2, ARC.end() + delta);
}

// ---------------------------------------------------------------------
// Through the facade
// ---------------------------------------------------------------------

/// The facade recognises an arc under the cursor and hands it to the
/// single dragger.
///
/// `ROUTER::StartDragging` counts `SEGMENT_T | ARC_T` to choose between
/// the single and the multi dragger (`pcbnew/router/pns_router.cpp:182`),
/// so a set holding one arc reaches `DRAGGER::Start` and its `ARC_T`
/// case. The commit that comes back is what a host applies.
#[test]
fn the_facade_drags_an_arc_and_commits_it() {
  let snapshot = snapshot();
  let mut router = Router::new(
    &snapshot,
    Box::new(rules()),
    settings_for(RouterMode::Shove),
    Sizes::default(),
  );
  let target = Vec2::new(900_000, 300_000);

  router
    .start_dragging(ARC.arc_mid(), &[ARC_TRACK], false)
    .expect("an arc is draggable");
  router.move_to(target, None);

  let FixOutcome::Finished(diff) = router.fix_route(target, None, true) else {
    panic!("a drag fix always finishes");
  };
  let arcs: Vec<_> = diff
    .added
    .iter()
    .chain(diff.updated.iter().map(|(_, item)| item))
    .filter_map(|item| match item.geometry {
      NewGeometry::Arc {
        start, mid, end, ..
      } => Some(ShapeArc::new(start, mid, end, 0)),
      _ => None,
    })
    .collect();

  assert!(!arcs.is_empty(), "the commit carries the dragged arc");

  for arc in &arcs {
    assert_ne!(*arc, stored(ARC), "the arc moved");
    assert!(arc.radius() < ARC.radius());
  }

  // The board's own arc left, either as a removal or as the update one of
  // the new pieces inherited its identity through.
  let replaced = diff.removed.contains(&ARC_TRACK)
    || diff.updated.iter().any(|(host, _)| *host == ARC_TRACK);

  assert!(replaced, "the original arc is off the board");
}

// ---------------------------------------------------------------------
// Which way an arc can be dragged
// ---------------------------------------------------------------------

/// An arc whose two neighbours are both tangent to it can be **grown**,
/// where one whose neighbour is not tangent can only shrink.
///
/// `DragArc` builds the largest circle inscribed in the angle between the
/// two tangent constraints and then clamps the cursor out of it
/// (`pcbnew/router/pns_line.cpp:1036`, `:1070`), so that maximal circle is
/// the ceiling on the drag. Where one side falls back to the arc's **own**
/// tangent stub, which is the `(intersection, endpoint)` segment at
/// `:1002`, that stub's far end is the arc's own endpoint and the maximal
/// circle comes out as the circle the arc is already on: the drag can then
/// only pull the bulge towards the corner. Two long tangent neighbours
/// lift the ceiling to whichever of them is shorter.
#[test]
fn an_arc_between_two_tangent_neighbours_can_be_grown() {
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);
  // The tangent at the arc's end points due north, so put the east run
  // there and both neighbours become tangent constraints.
  let north = Vec2::new(1_000_000, 3_000_000);

  snapshot.items.push(WorldItem::new(
    WEST_TRACK,
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(WEST, ARC.start()),
      width: TRACK_WIDTH,
    },
  ));
  snapshot.items.push(WorldItem::new(
    ARC_TRACK,
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
    EAST_TRACK,
    SIGNAL,
    LayerRange::single(0),
    WorldGeometry::Segment {
      seg: Seg::new(ARC.end(), north),
      width: TRACK_WIDTH,
    },
  ));

  let (mut world, index) = World::from_snapshot(&snapshot);
  let arc = *index
    .items_of(ARC_TRACK)
    .first()
    .expect("every snapshot item was stored");
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  // Away from the corner the two tangents meet at, which is the
  // direction a larger inscribed circle lies in.
  let target = Vec2::new(200_000, 400_000);
  let (mut dragger, moved) = drag_the_arc(&mut world, &context, arc, target);

  assert!(moved);

  let node = dragger
    .fix_route_node(&mut world, &context, true)
    .expect("a mark obstacles drag always commits");
  let dragged = added_arcs(&world, node);

  assert!(!dragged.is_empty(), "an arc reaches the board");

  for arc in &dragged {
    assert!(
      arc.radius() > ARC.radius(),
      "{} against {}",
      arc.radius(),
      ARC.radius()
    );
  }
}
