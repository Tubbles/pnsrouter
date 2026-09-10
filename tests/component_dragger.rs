// SPDX-License-Identifier: GPL-3.0-or-later

//! Component drag scenarios on a one layer board, through the public API.
//!
//! `doc/work/009-dragging.md` and note
//! `doc/reference/kicad/06-dragger.md` section 11 ask for
//! [`ComponentDragger`] to be exercised over a board with real stored
//! pads and traces rather than only in unit tests. The board is built out
//! of plain data the way `tests/dragger.rs` and
//! `tests/multi_dragger.rs` build theirs, because `tests/support/` is the
//! KiCad fixture reader and nothing else.
//!
//! The corpus says nothing at all about component drag: no log in
//! KiCad's `pns_regressions` holds a `DRAG_COMPONENT` session, and
//! `ROUTER::GetUpdatedItems` could not have measured one anyway (note 06
//! erratum E15). These scenarios are the coverage instead.

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::component_dragger::ComponentDragger;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{
  HostId, ItemBody, ItemId, LayerRange, NetId, Segment, Solid,
};
use pnsrouter::node::{NodeId, World};
use pnsrouter::router::{FixOutcome, Router, RouterState};
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RouterMode, RoutingSettings, Sizes};
use pnsrouter::snapshot::{WorldGeometry, WorldItem, WorldSnapshot};

/// The clearance every scenario drags to, in nanometres.
const CLEARANCE: i32 = 100_000;

/// The width of every trace.
const TRACK_WIDTH: i32 = 200_000;

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 400_000;

/// The net of the first pad and the trace on it.
const NET_A: Option<NetId> = Some(NetId(1));

/// The net of the second pad and the trace on it.
const NET_B: Option<NetId> = Some(NetId(2));

/// The net of the track a shove would have to move, if there were one.
const OBSTACLE_NET: Option<NetId> = Some(NetId(90));

/// Where the first pad of the footprint sits.
const PAD_A_AT: Vec2 = Vec2::new(0, 0);

/// Where the second pad sits, three millimetres to the right.
const PAD_B_AT: Vec2 = Vec2::new(3_000_000, 0);

/// The far end of both traces, six millimetres below their pads.
const TRACE_BOTTOM: i32 = -6_000_000;

/// The x of the track that sits in the way of the dragged component.
///
/// Half a millimetre past the second pad's new right hand edge, so that
/// the pair is clear before the drag and touching after it.
const OBSTACLE_X: i32 = 4_500_000;

/// Where the cursor grabs the component, between the two pads.
const GRAB: Vec2 = Vec2::new(1_500_000, 0);

/// How far the component is dragged: one millimetre to the right.
const OFFSET: Vec2 = Vec2::new(1_000_000, 0);

/// The host object the first pad came from.
const HOST_PAD_A: HostId = HostId(1);

/// The host object the second pad came from.
const HOST_PAD_B: HostId = HostId(2);

/// The host object the first trace came from.
const HOST_TRACE_A: HostId = HostId(3);

/// The host object the second trace came from.
const HOST_TRACE_B: HostId = HostId(4);

/// A board built by hand, with the handles a scenario drags by.
struct Fixture {
  /// The two pads of the component, in host id order.
  pads: [ItemId; 2],
  /// The trace hanging off each pad, in the same order.
  traces: [ItemId; 2],
}

/// A round pad on layer 0.
fn add_pad(world: &mut World, at: Vec2, net: Option<NetId>) -> ItemId {
  let root = world.root();
  let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::single(0));
  item.set_net(net);

  world.add_solid(root, item, None)
}

/// A straight track on layer 0.
fn add_track(
  world: &mut World,
  from: Vec2,
  to: Vec2,
  net: Option<NetId>,
) -> ItemId {
  let root = world.root();
  let body = ItemBody::Segment(Segment::new(Seg::new(from, to), TRACK_WIDTH));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::single(0));
  item.set_net(net);

  world
    .add_segment(root, item, false)
    .expect("a board track is neither degenerate nor redundant")
}

/// Two pads with one trace hanging off each, and optionally a track in
/// the way of where the drag is going.
fn build(with_obstacle: bool) -> (World, Fixture) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let pad_a = add_pad(&mut world, PAD_A_AT, NET_A);
  let pad_b = add_pad(&mut world, PAD_B_AT, NET_B);
  let trace_a = add_track(
    &mut world,
    PAD_A_AT,
    Vec2::new(PAD_A_AT.x, TRACE_BOTTOM),
    NET_A,
  );
  let trace_b = add_track(
    &mut world,
    PAD_B_AT,
    Vec2::new(PAD_B_AT.x, TRACE_BOTTOM),
    NET_B,
  );

  if with_obstacle {
    add_track(
      &mut world,
      Vec2::new(OBSTACLE_X, TRACE_BOTTOM),
      Vec2::new(OBSTACLE_X, 2_000_000),
      OBSTACLE_NET,
    );
  }

  (
    world,
    Fixture {
      pads: [pad_a, pad_b],
      traces: [trace_a, trace_b],
    },
  )
}

/// The rule oracle every scenario uses.
fn rules() -> FixedClearance {
  FixedClearance::uniform(CLEARANCE)
}

/// Settings in one routing mode, with everything else at KiCad's
/// defaults.
fn settings_for(mode: RouterMode) -> RoutingSettings {
  RoutingSettings {
    mode,
    ..RoutingSettings::default()
  }
}

/// Where a solid sits in a node, by handle.
fn solid_pos(world: &World, id: ItemId) -> Option<Vec2> {
  match world.item(id)?.body() {
    ItemBody::Solid(solid) => Some(solid.pos()),
    _ => None,
  }
}

/// The segments of a node's delta, as geometry, in the order
/// [`World::get_updated_items`] answers.
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

/// Drag the two pads by [`OFFSET`] in one routing mode.
fn drag_in(mode: RouterMode, with_obstacle: bool) -> (World, ComponentDragger) {
  let (mut world, fixture) = build(with_obstacle);
  let root = world.root();
  let rules = rules();
  let settings = settings_for(mode);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = ComponentDragger::new(&world, root);

  assert!(
    dragger.start(&world, &context, GRAB, &fixture.pads),
    "a set of two pads starts a component drag"
  );
  assert!(
    dragger.drag(&mut world, GRAB + OFFSET),
    "COMPONENT_DRAGGER::Drag always answers true"
  );

  (world, dragger)
}

// ---------------------------------------------------------------------
// Mark obstacles mode
// ---------------------------------------------------------------------

#[test]
fn both_pads_move_by_the_cursor_offset_and_both_traces_follow() {
  // `pcbnew/router/pns_component_dragger.cpp:170` moves each pad by
  // `aP - m_p0`, and `:233` drags the corner of each attached line to
  // where that pad's end went.
  let (mut world, fixture) = build(false);
  let root = world.root();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = ComponentDragger::new(&world, root);

  assert!(dragger.start(&world, &context, GRAB, &fixture.pads));
  // `CurrentNode()` is the untouched board before the first drag
  // (`:266`).
  assert_eq!(dragger.current_node(), root);
  assert!(dragger.traces().is_empty());
  assert!(dragger.moved_solids().is_empty());

  assert!(dragger.drag(&mut world, GRAB + OFFSET));

  let node = dragger.current_node();

  assert_ne!(node, root);

  // Both pads moved, and both report the cursor's own displacement.
  let moved = dragger.moved_solids().to_vec();

  assert_eq!(moved.len(), 2, "{moved:?}");

  for (index, entry) in moved.iter().enumerate() {
    assert_eq!(entry.before, fixture.pads[index], "{moved:?}");
    assert_eq!(entry.offset, OFFSET, "{moved:?}");
    assert_eq!(
      solid_pos(&world, entry.after),
      solid_pos(&world, entry.before).map(|before| before + OFFSET),
      "{moved:?}"
    );
  }

  // Both traces were re-dragged, and each one starts on the pad it hangs
  // off at the pad's new position.
  let traces = dragger.traces();

  assert_eq!(traces.len(), 2, "{traces:?}");

  for line in traces {
    let head = line.point(0);
    let tail = line.last_point().expect("a dragged line has points");

    assert!(
      head == PAD_A_AT + OFFSET || head == PAD_B_AT + OFFSET,
      "a dragged trace does not start on a moved pad: {line:?}"
    );
    // The far end never moves: only one corner of the line is dragged.
    assert_eq!(tail.y, TRACE_BOTTOM, "{line:?}");
  }

  // The node delta replaces both original traces.
  let (added, removed) = delta_segments(&world, node);

  assert!(!added.is_empty(), "{added:?}");
  assert_eq!(removed.len(), 2, "{removed:?}");

  // Nothing the drag put down collides, so a fix commits.
  assert_eq!(
    dragger.fix_route_node(&world, &context, false),
    Some(node),
    "a clear component drag refused to commit"
  );
  assert!(dragger.fix_route(&mut world, &context, false));

  // After the commit the moved pads are still where the drag put them.
  for entry in &moved {
    let at = solid_pos(&world, entry.after);

    assert!(
      at == Some(PAD_A_AT + OFFSET) || at == Some(PAD_B_AT + OFFSET),
      "a committed pad is not where the drag left it: {at:?}"
    );
  }

  // The traces the scenario built are the ones the delta replaced.
  assert_eq!(fixture.traces.len(), 2);
}

// ---------------------------------------------------------------------
// Shove mode
// ---------------------------------------------------------------------

#[test]
fn shove_mode_shoves_nothing_and_a_touched_track_refuses_the_commit() {
  // Note 06 erratum E30: `COMPONENT_DRAGGER` reads no routing mode, builds
  // no `SHOVE` and calls no `OPTIMIZER`, so a component drag in shove
  // mode is the same drag as in mark obstacles mode. The one thing the
  // obstacle changes is the answer of `FixRoute` (`:253`).
  let (mut world, marked) = drag_in(RouterMode::MarkObstacles, true);
  let (shoved_world, shoved) = drag_in(RouterMode::Shove, true);

  let marked_delta = delta_segments(&world, marked.current_node());
  let shoved_delta = delta_segments(&shoved_world, shoved.current_node());

  assert_eq!(
    marked_delta, shoved_delta,
    "shove mode produced a different component drag"
  );
  assert_eq!(marked.moved_solids(), shoved.moved_solids());

  // The nearby track is not in the delta at all: nothing pushed it.
  let (added, removed) = delta_segments(&shoved_world, shoved.current_node());
  let obstacle = Seg::new(
    Vec2::new(OBSTACLE_X, TRACE_BOTTOM),
    Vec2::new(OBSTACLE_X, 2_000_000),
  );

  assert!(!removed.contains(&obstacle), "{removed:?}");
  assert!(
    added.iter().all(|seg| seg.a.x != OBSTACLE_X),
    "the obstacle track was moved: {added:?}"
  );

  // The moved pad now touches it, so the fix refuses unless it is forced.
  let rules = rules();
  let settings = settings_for(RouterMode::Shove);
  let context = AlgoContext::new(&rules, &settings);

  assert_eq!(marked.fix_route_node(&world, &context, false), None);
  assert_eq!(
    marked.fix_route_node(&world, &context, true),
    Some(marked.current_node()),
    "a forced component drag commit was refused"
  );
  assert!(marked.fix_route(&mut world, &context, true));
}

// ---------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------

#[test]
fn two_identical_component_drags_answer_the_same_thing() {
  // `DESIGN.md` section 8. KiCad's `m_solids` and `m_fixedItems` are
  // pointer ordered `std::set`s (note 06 erratum E31); this orders both
  // by uid, so two runs of one gesture cannot disagree.
  let first = drag_in(RouterMode::MarkObstacles, true);
  let second = drag_in(RouterMode::MarkObstacles, true);

  assert_eq!(
    delta_segments(&first.0, first.1.current_node()),
    delta_segments(&second.0, second.1.current_node())
  );
  assert_eq!(first.1.moved_solids(), second.1.moved_solids());

  let shapes = |dragger: &ComponentDragger| -> Vec<Vec<Vec2>> {
    dragger
      .traces()
      .iter()
      .map(|line| line.shape().points().to_vec())
      .collect()
  };

  assert_eq!(shapes(&first.1), shapes(&second.1));
}

// ---------------------------------------------------------------------
// A trace between two dragged pads
// ---------------------------------------------------------------------

#[test]
fn a_trace_that_runs_between_two_dragged_pads_translates_rigidly() {
  // `pcbnew/router/pns_component_dragger.cpp:71` to `:81` catches one
  // segment whose far anchor carries a dragged pad and moves it whole
  // (`:202`) instead of dragging one of its corners.
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let pad_a = add_pad(&mut world, PAD_A_AT, NET_A);
  let pad_b = add_pad(&mut world, PAD_B_AT, NET_A);

  add_track(&mut world, PAD_A_AT, PAD_B_AT, NET_A);

  let root = world.root();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = ComponentDragger::new(&world, root);

  assert!(dragger.start(&world, &context, GRAB, &[pad_a, pad_b]));
  assert!(dragger.drag(&mut world, GRAB + OFFSET));

  // Nothing was re-dragged: the bridge is a fixed item, not a
  // connection.
  assert!(dragger.traces().is_empty(), "{:?}", dragger.traces());

  let (added, removed) = delta_segments(&world, dragger.current_node());

  assert_eq!(removed, vec![Seg::new(PAD_A_AT, PAD_B_AT)]);
  assert_eq!(
    added,
    vec![Seg::new(PAD_A_AT + OFFSET, PAD_B_AT + OFFSET)],
    "the bridge did not translate rigidly"
  );
}

#[test]
fn a_run_of_segments_between_two_dragged_pads_is_not_cloned_twice() {
  // `:98` to `:109` puts **every** link of the assembled line into
  // `m_fixedItems`, and the run is reached once from each pad, because
  // `seenItems` de-duplicates the seed segment and not the line. KiCad's
  // `std::set` swallows the second insertion; a vector has to be
  // de-duplicated or the drag clones each segment twice.
  const JUNCTION: Vec2 = Vec2::new(1_500_000, -1_000_000);

  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let pad_a = add_pad(&mut world, PAD_A_AT, NET_A);
  let pad_b = add_pad(&mut world, PAD_B_AT, NET_A);

  add_track(&mut world, PAD_A_AT, JUNCTION, NET_A);
  add_track(&mut world, JUNCTION, PAD_B_AT, NET_A);

  let root = world.root();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = ComponentDragger::new(&world, root);

  assert!(dragger.start(&world, &context, GRAB, &[pad_a, pad_b]));
  assert!(dragger.drag(&mut world, GRAB + OFFSET));

  assert!(dragger.traces().is_empty(), "{:?}", dragger.traces());

  let (added, removed) = delta_segments(&world, dragger.current_node());

  assert_eq!(removed.len(), 2, "{removed:?}");
  assert_eq!(added.len(), 2, "the run was cloned twice: {added:?}");
  assert_eq!(
    added,
    vec![
      Seg::new(PAD_A_AT + OFFSET, JUNCTION + OFFSET),
      Seg::new(JUNCTION + OFFSET, PAD_B_AT + OFFSET),
    ]
  );
}

// ---------------------------------------------------------------------
// A trace end that is not jointed to the pad
// ---------------------------------------------------------------------

#[test]
fn a_trace_that_merely_ends_inside_a_pad_is_dragged_along_with_it() {
  // `pcbnew/router/pns_component_dragger.cpp:137` to `:148`: a joint in
  // the pad's copper box, on the pad's net and layers, with exactly one
  // link that actually touches the pad, is dragged along as if it were
  // connected, at the offset it had from the pad centre.
  //
  // Both are netless, which is what makes the block reachable at all:
  // it asks for the same net **and** a collision, and a same net pair
  // gets no clearance and therefore never collides
  // (`pcbnew/router/pns_item.cpp:188`). Two null nets are not "the same
  // net" to that test, so they do collide. Note 06 erratum E36.
  const STUB_END: Vec2 = Vec2::new(200_000, 0);

  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let pad = add_pad(&mut world, PAD_A_AT, None);

  add_track(
    &mut world,
    STUB_END,
    Vec2::new(STUB_END.x, TRACE_BOTTOM),
    None,
  );
  let root = world.root();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = ComponentDragger::new(&world, root);

  assert!(dragger.start(&world, &context, PAD_A_AT, &[pad]));
  assert!(dragger.drag(&mut world, PAD_A_AT + OFFSET));

  let traces = dragger.traces();

  assert_eq!(traces.len(), 1, "the stub was not picked up: {traces:?}");
  // The end that was inside the pad kept its offset from the pad centre.
  assert_eq!(traces[0].point(0), STUB_END + OFFSET, "{traces:?}");
  assert_eq!(
    traces[0].last_point(),
    Some(Vec2::new(STUB_END.x, TRACE_BOTTOM)),
    "{traces:?}"
  );
}

// ---------------------------------------------------------------------
// The facade
// ---------------------------------------------------------------------

/// The fixture board as a host would hand it over.
fn snapshot() -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(1, World::DEFAULT_MAX_CLEARANCE);
  let pad = |id: HostId, at: Vec2, net: Option<NetId>| {
    WorldItem::new(
      id,
      net,
      LayerRange::single(0),
      WorldGeometry::Solid {
        shape: Shape::circle(at, PAD_RADIUS),
        pos: at,
        offset: Vec2::new(0, 0),
        orientation_degrees: 0.0,
        anchors: Vec::new(),
      },
    )
  };
  let track = |id: HostId, from: Vec2, to: Vec2, net: Option<NetId>| {
    WorldItem::new(
      id,
      net,
      LayerRange::single(0),
      WorldGeometry::Segment {
        seg: Seg::new(from, to),
        width: TRACK_WIDTH,
      },
    )
  };

  snapshot.items.push(pad(HOST_PAD_A, PAD_A_AT, NET_A));
  snapshot.items.push(pad(HOST_PAD_B, PAD_B_AT, NET_B));
  snapshot.items.push(track(
    HOST_TRACE_A,
    PAD_A_AT,
    Vec2::new(PAD_A_AT.x, TRACE_BOTTOM),
    NET_A,
  ));
  snapshot.items.push(track(
    HOST_TRACE_B,
    PAD_B_AT,
    Vec2::new(PAD_B_AT.x, TRACE_BOTTOM),
    NET_B,
  ));

  snapshot
}

#[test]
fn the_facade_drags_two_pads_and_commits_the_offset_for_both() {
  // `pcbnew/router/pns_router.cpp:176`: a set of nothing but solids is a
  // component drag, whatever the drag mode said. The commit is the pair
  // `PNS_KICAD_IFACE` turns into `m_fpOffsets`
  // (`pcbnew/router/pns_kicad_iface.cpp:2634`, `:2854`, `:2918`).
  let snapshot = snapshot();
  let mut router = Router::new(
    &snapshot,
    Box::new(rules()),
    settings_for(RouterMode::Shove),
    Sizes {
      track_width: TRACK_WIDTH,
      board_min_track_width: TRACK_WIDTH / 2,
      ..Sizes::default()
    },
  );

  router
    .start_dragging(GRAB, &[HOST_PAD_A, HOST_PAD_B], false)
    .expect("a set of two pads is a component drag");

  assert_eq!(router.state(), RouterState::DragComponent);

  let frame = router.move_to(GRAB + OFFSET, None);

  // The preview says both pads moved, and by how far.
  assert_eq!(
    frame.moved_solids,
    vec![(HOST_PAD_A, OFFSET), (HOST_PAD_B, OFFSET)],
    "{frame:?}"
  );
  // `updateView` hides the originals as it does for anything the drag
  // took out of the board (`pcbnew/router/pns_router.cpp:772`).
  assert!(frame.hidden.contains(&HOST_PAD_A), "{frame:?}");
  assert!(frame.hidden.contains(&HOST_PAD_B), "{frame:?}");

  // The `DRAG_COMPONENT` branch KiCad's `GetUpdatedItems` does not have
  // (note 06 erratum E15) reports the delta rather than nothing.
  let pending = router.pending_update();

  assert!(pending.node.is_some(), "{pending:?}");
  assert!(pending.removed.contains(&HOST_TRACE_A), "{pending:?}");
  assert!(pending.removed.contains(&HOST_TRACE_B), "{pending:?}");

  let FixOutcome::Finished(diff) = router.fix_route(GRAB + OFFSET, None, false)
  else {
    panic!("a clear component drag refused to commit");
  };

  assert_eq!(router.state(), RouterState::Idle);
  assert_eq!(
    diff.moved_solids,
    vec![(HOST_PAD_A, OFFSET), (HOST_PAD_B, OFFSET)],
    "{diff:?}"
  );
  // The pads are not deleted and re-created: `moved_solids` is the only
  // place they appear. The two traces became two segments each, so the
  // remove plus add fold keeps each host object's identity on one of the
  // halves (`pcbnew/router/pns_router.cpp:874`) and nothing is removed
  // outright.
  let updated: Vec<HostId> =
    diff.updated.iter().map(|(host, _)| *host).collect();

  assert!(diff.removed.is_empty(), "{diff:?}");
  assert_eq!(updated, vec![HOST_TRACE_A, HOST_TRACE_B], "{diff:?}");
  assert_eq!(diff.added.len(), 2, "{diff:?}");
}
