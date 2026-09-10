// SPDX-License-Identifier: GPL-3.0-or-later

//! Recording, storing and replaying whole routing sessions.
//!
//! `DESIGN.md` section 8 asks for the engine to be a pure function of
//! (snapshot, settings, event sequence), checked by recording a session,
//! replaying it and comparing the commit against a golden. This is that
//! check, on the two layer board `tests/router.rs` and `tests/placer.rs`
//! route on.
//!
//! The golden itself is `tests/fixtures/sessions/two_layer_via.txt`;
//! `tests/fixtures/sessions/README.md` says how it is regenerated and why
//! a change to it is a change of behaviour.

#![forbid(unsafe_code)]

use std::path::PathBuf;

use pnsrouter::eventlog::{
  ParseError, Recorder, SessionEvent, SessionRecording,
  assert_replay_is_collision_free, assert_replay_matches, replay,
};
use pnsrouter::geometry::direction45::CornerMode;
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::{Shape, SimplePolygon};
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{HostId, LayerMask, LayerRange, NetId, ViaType};
use pnsrouter::node::World;
use pnsrouter::router::{CommitDiff, FixOutcome, NewGeometry, NewItem, Router};
use pnsrouter::rules::{FixedClearance, RuleResolver};
use pnsrouter::settings::{
  OptimizerEffort, RouterMode, RoutingSettings, Sizes,
};
use pnsrouter::snapshot::{
  WorldGeometry, WorldItem, WorldItemFlags, WorldSnapshot,
};

/// The clearance every scenario routes to, in nanometres.
const CLEARANCE: i32 = 100_000;

/// The width of the routed track.
const TRACK_WIDTH: i32 = 200_000;

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 400_000;

/// The net the start and target pads are on.
const TRACE_NET: Option<NetId> = Some(NetId(1));

/// The net of the obstacle pad, so that it is never exempt.
const OBSTACLE_NET: Option<NetId> = Some(NetId(2));

/// The pad the route starts on, layer 0.
const START_PAD: HostId = HostId(1);

/// The pad in the way, layer 0.
const OBSTACLE_PAD: HostId = HostId(2);

/// The pad the route ends on, layer 0.
const TARGET_PAD: HostId = HostId(3);

/// The pad the two layer route ends on, layer 1.
const DEEP_PAD: HostId = HostId(5);

/// Where the route starts.
const START: Vec2 = Vec2::new(0, 0);

/// The pad the route has to get past.
const OBSTACLE: Vec2 = Vec2::new(2_000_000, 0);

/// Where the single layer route ends.
const TARGET: Vec2 = Vec2::new(4_000_000, 0);

/// Where the layer change happens, clear of everything on both layers.
const VIA_POINT: Vec2 = Vec2::new(0, -3_000_000);

/// The first corner, fixed and then taken back by the undo.
const CORNER: Vec2 = Vec2::new(-1_000_000, -1_000_000);

/// Where the cursor was when the undo took the corner back.
///
/// A move has to come between the fix and the undo. `UnfixRoute` answers
/// with the head's first point and only when the head has one
/// (`pcbnew/router/pns_line_placer.cpp:1766`), and a fix leaves the head
/// empty, so an undo straight after a fix takes the stage back and still
/// answers [`None`]. That is also the real sequence: the cursor moves on
/// before the user presses backspace.
const UNDONE_REACH: Vec2 = Vec2::new(-1_000_000, -2_000_000);

/// Where the two layer route ends, on layer 1.
const DEEP_TARGET: Vec2 = Vec2::new(3_000_000, -3_000_000);

/// The golden recording every fixture test reads.
const FIXTURE: &str = "two_layer_via.txt";

/// A round pad on one copper layer.
fn pad(id: HostId, at: Vec2, layer: i32, net: Option<NetId>) -> WorldItem {
  WorldItem::new(
    id,
    net,
    LayerRange::single(layer),
    WorldGeometry::Solid {
      shape: Shape::circle(at, PAD_RADIUS),
      pos: at,
      offset: Vec2::new(0, 0),
      orientation_degrees: 0.0,
      anchors: Vec::new(),
    },
  )
}

/// The fixture board, as a host would hand it over.
fn board() -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);

  snapshot.items.push(pad(START_PAD, START, 0, TRACE_NET));
  snapshot
    .items
    .push(pad(OBSTACLE_PAD, OBSTACLE, 0, OBSTACLE_NET));
  snapshot.items.push(pad(TARGET_PAD, TARGET, 0, TRACE_NET));
  snapshot
    .items
    .push(pad(DEEP_PAD, DEEP_TARGET, 1, TRACE_NET));

  snapshot
}

/// The three host objects the drag session's trace is made of.
const TRACE_SEGMENTS: [HostId; 3] = [HostId(10), HostId(11), HostId(12)];

/// The corners of that trace, in order.
///
/// Three segments due east on layer 0, with a middle one long enough to
/// survive a sideways drag without collapsing into a corner.
const DRAGGED_TRACE: [Vec2; 4] = [
  Vec2::new(0, 3_000_000),
  Vec2::new(1_000_000, 3_000_000),
  Vec2::new(3_000_000, 3_000_000),
  Vec2::new(4_000_000, 3_000_000),
];

/// Where the drag grabs the trace: the middle of its middle segment, more
/// than half a track width from either end, so `startDragSegment`
/// (`pcbnew/router/pns_dragger.cpp:128`) resolves it to a segment drag.
const DRAG_GRAB: Vec2 = Vec2::new(2_000_000, 3_000_000);

/// Where the drag lets go, clear of everything on the board.
const DRAG_RELEASE: Vec2 = Vec2::new(2_000_000, 2_000_000);

/// A board with one three segment trace and nothing else.
///
/// Separate from [`board`] on purpose: the golden fixture
/// `two_layer_via.txt` carries [`board`] inside it, so adding an item
/// there would rewrite a stored golden.
fn drag_board() -> WorldSnapshot {
  let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);

  for (host, pair) in TRACE_SEGMENTS.iter().zip(DRAGGED_TRACE.windows(2)) {
    snapshot.items.push(WorldItem::new(
      *host,
      TRACE_NET,
      LayerRange::single(0),
      WorldGeometry::Segment {
        seg: Seg::new(pair[0], pair[1]),
        width: TRACK_WIDTH,
      },
    ));
  }

  snapshot
}

/// The sizes every scenario places with.
fn sizes() -> Sizes {
  let mut sizes = Sizes {
    track_width: TRACK_WIDTH,
    board_min_track_width: 100_000,
    via_diameter: 600_000,
    via_drill: 300_000,
    ..Sizes::default()
  };

  sizes.add_layer_pair(0, 1);
  sizes
}

/// The settings every scenario routes with.
fn settings() -> RoutingSettings {
  RoutingSettings {
    mode: RouterMode::Walkaround,
    ..RoutingSettings::default()
  }
}

/// The rule oracle, as a fresh box every time it is asked for.
fn rules() -> Box<dyn RuleResolver> {
  Box::new(FixedClearance::uniform(CLEARANCE))
}

/// A router over the fixture board, already recording.
fn recording_router() -> Router {
  let snapshot = board();
  let mut router = Router::new(&snapshot, rules(), settings(), sizes());

  router.start_recording(&snapshot);
  router
}

/// Where a session fixture lives.
fn fixture_path(name: &str) -> PathBuf {
  PathBuf::from(env!("CARGO_MANIFEST_DIR"))
    .join("tests/fixtures/sessions")
    .join(name)
}

/// The two layer session the golden fixture holds.
///
/// A route out of the start pad, a via, a leg on layer 1, one fix undone
/// again, and a terminal fix on the pad on the far layer. That is the
/// "route, via, undo, finish" of
/// `doc/work/005-session-api-and-event-log.md`.
fn record_two_layer_via() -> SessionRecording {
  let mut router = recording_router();

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(CORNER, None);

  let FixOutcome::Continue(_) = router.fix_route(CORNER, None, false) else {
    panic!("the corner fix finished the session");
  };

  router.move_to(UNDONE_REACH, None);

  assert_eq!(router.undo_last_segment(), Some(CORNER));

  router.move_to(VIA_POINT, None);

  assert!(router.toggle_via_placement());

  router.move_to(VIA_POINT, None);

  let FixOutcome::Continue(_) = router.fix_route(VIA_POINT, None, false) else {
    panic!("the fix that places the via finished the session");
  };

  assert!(router.switch_layer(1));

  router.move_to(DEEP_TARGET, Some(DEEP_PAD));

  let FixOutcome::Finished(diff) =
    router.fix_route(DEEP_TARGET, Some(DEEP_PAD), false)
  else {
    panic!("the fix on the layer 1 pad did not finish");
  };

  assert_eq!(
    diff
      .added
      .iter()
      .filter(|item| matches!(item.geometry, NewGeometry::Via { .. }))
      .count(),
    1,
    "the session did not commit its via: {diff:?}"
  );

  router.take_recording().expect("the recorder was installed")
}

// ---------------------------------------------------------------------
// Recording and replaying
// ---------------------------------------------------------------------

#[test]
fn a_recorded_session_replays_to_the_commit_it_made() {
  let recording = record_two_layer_via();

  assert_eq!(recording.results.len(), 1);
  assert_replay_matches(&recording, rules);
}

/// A drag replays to the commit it made, and survives the text form.
///
/// The drag half of `DESIGN.md` section 8: `start_dragging`, two moves
/// and a fix are inputs like any other, so a recorded drag is a pure
/// function of (snapshot, settings, events) too. It runs in
/// [`RouterMode::MarkObstacles`], which is the only drag path
/// `src/dragger.rs` has: the walkaround and the shove drags fall back to
/// it, so asking for one of those would assert on a stub.
#[test]
fn a_recorded_drag_replays_to_the_commit_it_made() {
  let snapshot = drag_board();
  let drag_settings = RoutingSettings {
    mode: RouterMode::MarkObstacles,
    ..RoutingSettings::default()
  };
  let mut router = Router::new(&snapshot, rules(), drag_settings, sizes());

  router.start_recording(&snapshot);
  router
    .start_dragging(DRAG_GRAB, &[TRACE_SEGMENTS[1]], false)
    .expect("the middle segment of a trace is draggable");
  router.move_to(Vec2::new(2_000_000, 2_500_000), None);
  router.move_to(DRAG_RELEASE, None);

  let FixOutcome::Finished(diff) = router.fix_route(DRAG_RELEASE, None, false)
  else {
    panic!("a clear drag refused to commit");
  };

  // The whole assembled line goes out and the dragged one comes back, so
  // the commit accounts for all three host objects. One of them survives
  // as an update, because the remove plus add fold pairs on
  // `GetSourceItem` and every segment `World::add_line` creates inherits
  // the one source the assembled line carried
  // (`pcbnew/router/pns_router.cpp:874`).
  assert_eq!(
    diff.removed.len() + diff.updated.len(),
    TRACE_SEGMENTS.len(),
    "the commit is {diff:?}"
  );
  assert_eq!(diff.updated.len(), 1, "the commit is {diff:?}");
  assert!(
    diff.added.len() + diff.updated.len() > TRACE_SEGMENTS.len(),
    "the sideways drag added no corner: {diff:?}"
  );

  let recording = router.take_recording().expect("a recorder is installed");

  assert_eq!(
    recording.events.first(),
    Some(&SessionEvent::StartDragging {
      at: DRAG_GRAB,
      items: vec![TRACE_SEGMENTS[1]],
      free_angle: false,
    })
  );
  assert_eq!(recording.results.len(), 1);

  let text = recording.to_text();

  assert_eq!(
    SessionRecording::from_text(&text),
    Ok(recording.clone()),
    "{text}"
  );
  assert_replay_matches(&recording, rules);
}

#[test]
fn the_recording_holds_every_call_that_changed_the_state() {
  let recording = record_two_layer_via();

  assert_eq!(
    recording.events,
    vec![
      SessionEvent::StartRouting {
        at: START,
        start: Some(START_PAD),
        layer: 0,
      },
      SessionEvent::MoveTo {
        at: CORNER,
        end: None,
      },
      SessionEvent::FixRoute {
        at: CORNER,
        end: None,
        force_finish: false,
      },
      SessionEvent::MoveTo {
        at: UNDONE_REACH,
        end: None,
      },
      SessionEvent::UndoLastSegment,
      SessionEvent::MoveTo {
        at: VIA_POINT,
        end: None,
      },
      SessionEvent::ToggleViaPlacement,
      SessionEvent::MoveTo {
        at: VIA_POINT,
        end: None,
      },
      SessionEvent::FixRoute {
        at: VIA_POINT,
        end: None,
        force_finish: false,
      },
      SessionEvent::SwitchLayer { layer: 1 },
      SessionEvent::MoveTo {
        at: DEEP_TARGET,
        end: Some(DEEP_PAD),
      },
      SessionEvent::FixRoute {
        at: DEEP_TARGET,
        end: Some(DEEP_PAD),
        force_finish: false,
      },
    ]
  );
}

#[test]
fn the_commit_a_terminal_fix_reaches_is_recorded_once() {
  let recording = record_two_layer_via();
  let outcome = replay(&recording, rules());

  assert_eq!(outcome.diffs.len(), 1);
  assert_eq!(outcome.events_count, recording.events.len());
  assert!(outcome.frames_count > 0);
}

#[test]
fn replaying_twice_produces_identical_commits() {
  let recording = record_two_layer_via();
  let first = replay(&recording, rules());
  let second = replay(&recording, rules());

  assert_eq!(first.diffs, second.diffs);
  assert_eq!(first.frames_count, second.frames_count);
}

#[test]
fn a_replay_leaves_no_clearance_violation_behind() {
  assert_replay_is_collision_free(&record_two_layer_via(), rules);
}

#[test]
fn a_wider_clearance_still_commits_a_clear_result() {
  // The tier a host with its own rules holds itself to: the geometry is
  // free to change, the answer is not free to collide.
  let recording = record_two_layer_via();

  assert_replay_is_collision_free(&recording, || {
    Box::new(FixedClearance::uniform(CLEARANCE * 2))
  });
}

#[test]
fn a_finish_and_a_continue_are_recorded_as_one_event_each() {
  let mut router = recording_router();

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(TARGET, Some(TARGET_PAD));
  router.finish();

  let recording = router.take_recording().expect("a recorder is installed");

  assert_eq!(
    recording.events,
    vec![
      SessionEvent::StartRouting {
        at: START,
        start: Some(START_PAD),
        layer: 0,
      },
      SessionEvent::MoveTo {
        at: TARGET,
        end: Some(TARGET_PAD),
      },
      SessionEvent::Finish,
    ],
    "the moves and the fix a finish drives leaked into the log"
  );
  assert_replay_matches(&recording, rules);
}

#[test]
fn an_abort_and_a_stop_are_both_recorded() {
  let mut router = recording_router();

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(Vec2::new(1_000_000, -1_000_000), None);
  router.abort_routing();
  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable again");
  router.move_to(Vec2::new(1_000_000, -1_000_000), None);
  router.fix_route(Vec2::new(1_000_000, -1_000_000), None, false);
  router.stop_routing();

  let recording = router.take_recording().expect("a recorder is installed");

  assert!(recording.events.contains(&SessionEvent::AbortRouting));
  assert!(recording.events.contains(&SessionEvent::StopRouting));
  assert_eq!(recording.results.len(), 1, "an abort committed something");
  assert_replay_matches(&recording, rules);
}

#[test]
fn the_ids_a_host_gives_the_additions_are_recorded() {
  let mut router = recording_router();

  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.move_to(TARGET, Some(TARGET_PAD));

  let FixOutcome::Finished(diff) =
    router.fix_route(TARGET, Some(TARGET_PAD), false)
  else {
    panic!("the fix on the target pad did not finish");
  };
  let ids: Vec<(usize, HostId)> = (0..diff.added.len())
    .map(|index| {
      (
        index,
        HostId(100 + u64::try_from(index).expect("a small index")),
      )
    })
    .collect();

  assert_eq!(router.assign_host_ids(&ids), ids.len());

  let recording = router.take_recording().expect("a recorder is installed");

  assert_eq!(
    recording.events.last(),
    Some(&SessionEvent::AssignHostIds { ids })
  );
  assert_replay_matches(&recording, rules);
}

#[test]
fn a_settings_and_a_sizes_change_travel_with_the_session() {
  let mut router = recording_router();
  let mut wider = sizes();

  wider.track_width = 300_000;

  router.set_sizes(wider.clone());
  router.set_settings(RoutingSettings {
    mode: RouterMode::MarkObstacles,
    ..settings()
  });
  router.toggle_corner_mode();
  router
    .start_routing(START, Some(START_PAD), 0)
    .expect("the start pad is routable");
  router.set_ortho_mode(true);
  router.flip_posture();
  router.move_to(Vec2::new(1_000_000, 0), None);

  let recording = router.take_recording().expect("a recorder is installed");
  let text = recording.to_text();
  let read = SessionRecording::from_text(&text)
    .expect("a recording this crate wrote does not parse");

  assert_eq!(read, recording);
  assert!(matches!(
    read.events.first(),
    Some(SessionEvent::SetSizes { sizes }) if sizes.track_width == 300_000
  ));
  assert!(matches!(
    read.events.get(1),
    Some(SessionEvent::SetSettings { settings })
      if settings.mode == RouterMode::MarkObstacles
  ));
  assert_replay_matches(&read, rules);
}

// ---------------------------------------------------------------------
// The text format
// ---------------------------------------------------------------------

#[test]
fn a_session_survives_the_trip_through_text() {
  let recording = record_two_layer_via();
  let text = recording.to_text();
  let read = SessionRecording::from_text(&text)
    .expect("a recording this crate wrote does not parse");

  assert_eq!(read, recording);
  assert_eq!(read.to_text(), text, "writing is not idempotent");
  assert_replay_matches(&read, rules);
}

#[test]
fn every_settings_field_survives_the_trip_through_text() {
  // Every field away from its default, so a field the writer forgot
  // comes back as the default and fails the comparison.
  let settings = RoutingSettings {
    mode: RouterMode::MarkObstacles,
    optimizer_effort: OptimizerEffort::Full,
    shove_vias: false,
    remove_loops: false,
    smart_pads: false,
    follow_mouse: false,
    start_diagonal: true,
    jump_over_obstacles: true,
    smooth_dragged_segments: false,
    allow_drc_violations: true,
    free_angle_mode: true,
    optimize_entire_dragged_track: true,
    auto_posture: false,
    fix_all_segments: false,
    restrict_angles: true,
    corner_mode: CornerMode::Mitered90,
    walkaround_iteration_limit: 41,
    shove_iteration_limit: 251,
    shove_time_limit_ms: 1001,
    via_force_prop_iteration_limit: 42,
    walkaround_hug_length_threshold: 1.234_567_890_123_456_7,
  };
  let recording = SessionRecording {
    events: vec![SessionEvent::SetSettings { settings }],
    ..SessionRecording::new(board(), settings, sizes())
  };
  let read = SessionRecording::from_text(&recording.to_text())
    .expect("the settings do not parse");

  assert_eq!(read.settings, settings);
  assert_eq!(read.events, recording.events);
  assert_ne!(read.settings, RoutingSettings::default());
}

#[test]
fn every_sizes_field_survives_the_trip_through_text() {
  let mut sizes = Sizes {
    clearance: 1,
    min_clearance: 2,
    track_width: 3,
    track_width_is_explicit: false,
    board_min_track_width: 4,
    via_type: ViaType::Buried,
    via_diameter: 5,
    via_drill: 6,
    diff_pair_width: 7,
    diff_pair_gap: 8,
    diff_pair_via_gap: 9,
    diff_pair_via_gap_same_as_trace_gap: false,
    hole_to_hole: 10,
    diff_pair_hole_to_hole: 11,
    diff_pair_copper_to_hole: 12,
    ..Sizes::default()
  };

  sizes.add_layer_pair(0, 3);
  sizes.add_layer_pair(1, 2);

  let recording = SessionRecording {
    events: vec![SessionEvent::SetSizes {
      sizes: sizes.clone(),
    }],
    ..SessionRecording::new(board(), settings(), sizes.clone())
  };
  let read = SessionRecording::from_text(&recording.to_text())
    .expect("the sizes do not parse");

  assert_eq!(read.sizes, sizes);
  assert_eq!(read.events, recording.events);
  assert_ne!(read.sizes, Sizes::default());
}

#[test]
fn every_geometry_and_shape_variant_survives_the_trip_through_text() {
  let mut chain = LineChain::from_points(
    vec![Vec2::new(0, 0), Vec2::new(10, 0), Vec2::new(10, 10)],
    false,
  );

  chain.set_width(7);

  let shapes = vec![
    Shape::circle(Vec2::new(1, 2), 3),
    Shape::Rect {
      origin: Vec2::new(4, 5),
      size: Vec2::new(6, 7),
      radius: 8,
    },
    Shape::segment(Seg::with_index(Vec2::new(9, 10), Vec2::new(11, 12), 3), 13),
    Shape::Simple(SimplePolygon::from_points(vec![
      Vec2::new(0, 0),
      Vec2::new(100, 0),
      Vec2::new(100, 100),
    ])),
    Shape::line_chain(chain),
    Shape::Compound(vec![
      Shape::circle(Vec2::new(0, 0), 1),
      Shape::Compound(vec![Shape::circle(Vec2::new(2, 2), 3)]),
    ]),
  ];
  let mut snapshot = WorldSnapshot::new(4, 900_000);

  snapshot.edge_exclusions = shapes.clone();

  for (index, shape) in shapes.iter().enumerate() {
    let id = HostId(u64::try_from(index).expect("a small index"));
    let mut item = WorldItem::new(
      id,
      Some(NetId(3)),
      LayerRange::new(0, 2),
      WorldGeometry::Solid {
        shape: shape.clone(),
        pos: Vec2::new(-1, -2),
        offset: Vec2::new(-3, -4),
        orientation_degrees: 33.75,
        anchors: vec![Vec2::new(5, 6), Vec2::new(7, 8)],
      },
    );

    item.hole = Some(shape.clone());
    item.flags = WorldItemFlags {
      locked: true,
      routable: false,
      free_pad: true,
      compound_primitive: true,
    };
    item.flashed_layers = Some(LayerMask::NONE.with(0).with(2));
    snapshot.items.push(item);
  }

  snapshot.items.push(WorldItem::new(
    HostId(20),
    None,
    LayerRange::single(1),
    WorldGeometry::Segment {
      seg: Seg::new(Vec2::new(0, 0), Vec2::new(1_000, 2_000)),
      width: 250_000,
    },
  ));
  snapshot.items.push(WorldItem::new(
    HostId(21),
    Some(NetId(9)),
    LayerRange::new(0, 3),
    WorldGeometry::Via {
      pos: Vec2::new(-5, -6),
      diameter: 600_000,
      drill: 300_000,
      via_type: ViaType::MicroVia,
      is_free: true,
    },
  ));
  snapshot.items.push(WorldItem::new(
    HostId(22),
    None,
    LayerRange::new(0, 3),
    WorldGeometry::Hole {
      shape: Shape::circle(Vec2::new(1, 1), 150_000),
    },
  ));

  let recording = SessionRecording {
    results: vec![CommitDiff {
      removed: vec![HostId(20)],
      added: vec![NewItem {
        geometry: NewGeometry::Via {
          pos: Vec2::new(1, 2),
          diameter: 3,
          drill: 4,
          via_type: ViaType::Blind,
        },
        net: Some(NetId(1)),
        layers: LayerRange::new(0, 1),
        source: None,
      }],
      updated: vec![(
        HostId(21),
        NewItem {
          geometry: NewGeometry::Segment {
            seg: Seg::with_index(Vec2::new(0, 0), Vec2::new(5, 5), 2),
            width: 6,
          },
          net: None,
          layers: LayerRange::single(2),
          source: Some(HostId(21)),
        },
      )],
    }],
    ..SessionRecording::new(snapshot, settings(), sizes())
  };
  let read = SessionRecording::from_text(&recording.to_text())
    .expect("the geometry does not parse");

  assert_eq!(read, recording);
}

#[test]
fn a_parse_error_names_the_line_it_is_on() {
  let text = "pnsrouter-session 1\nsnapshot 2 1000\nitem 1 - 0 0 0 1 0 0 \
              flash-default no-drill banana\n";

  assert_eq!(
    SessionRecording::from_text(text),
    Err(ParseError {
      line: 3,
      message: "`banana` is not an item geometry".to_string(),
    })
  );
  assert_eq!(
    SessionRecording::from_text(text)
      .expect_err("the geometry is not a fruit")
      .to_string(),
    "line 3: `banana` is not an item geometry"
  );
}

#[test]
fn the_reader_refuses_what_it_cannot_understand() {
  let cases = [
    ("", 0),
    ("snapshot 2 1000\n", 1),
    ("pnsrouter-session 99\n", 1),
    ("pnsrouter-session 1\n", 1),
    ("pnsrouter-session 1\nsnapshot 2 1000\nfnord 1\n", 3),
    (
      "pnsrouter-session 1\nsnapshot 2 1000\nsettings 0 fnord 1\n",
      3,
    ),
    ("pnsrouter-session 1\nsnapshot 2 1000\nsizes 0 fnord 1\n", 3),
    ("pnsrouter-session 1\nsnapshot 2 1000\nevent fnord\n", 3),
    (
      "pnsrouter-session 1\nsnapshot 2 1000\nevent set-sizes 4\n",
      3,
    ),
    ("pnsrouter-session 1\nsnapshot 2 1000\nremoved 1\n", 3),
    ("pnsrouter-session 1\nsnapshot 2 1000\nsnapshot 2 1000\n", 3),
    ("pnsrouter-session 1\nsnapshot 2 1000 3\n", 2),
    ("pnsrouter-session 1\nsnapshot 2\n", 2),
    (
      "pnsrouter-session 1\nexclusion compound 9 circle 0 0 1\n",
      2,
    ),
  ];

  for (text, line) in cases {
    let error = SessionRecording::from_text(text)
      .expect_err(&format!("`{text}` was accepted"));

    assert_eq!(error.line, line, "`{text}` reported the wrong line");
  }
}

#[test]
fn comments_and_blank_lines_are_ignored() {
  let text = "# a comment\n\n  \npnsrouter-session 1 # and a trailing one\n\
              \nsnapshot 2 1000\n";
  let read =
    SessionRecording::from_text(text).expect("comments are not records");

  assert_eq!(read.snapshot.copper_layer_count, 2);
  assert_eq!(read.snapshot.max_clearance, 1000);
  assert!(read.events.is_empty());
  assert!(read.results.is_empty());
}

// ---------------------------------------------------------------------
// The golden fixture
// ---------------------------------------------------------------------

#[test]
fn the_golden_fixture_replays_to_the_commit_stored_in_it() {
  let text = std::fs::read_to_string(fixture_path(FIXTURE))
    .expect("tests/fixtures/sessions/two_layer_via.txt is missing");
  let recording =
    SessionRecording::from_text(&text).expect("the fixture does not parse");

  assert_replay_matches(&recording, rules);
  assert_replay_is_collision_free(&recording, rules);
}

#[test]
fn the_golden_fixture_is_the_session_this_crate_records_today() {
  let text = std::fs::read_to_string(fixture_path(FIXTURE))
    .expect("tests/fixtures/sessions/two_layer_via.txt is missing");

  assert_eq!(
    record_two_layer_via().to_text(),
    text,
    "the session this crate records is no longer the one in \
     tests/fixtures/sessions/{FIXTURE}. That is a change of behaviour: \
     read tests/fixtures/sessions/README.md before regenerating it."
  );
}

/// Rewrite the golden fixture from what this crate does today.
///
/// Ignored, so it only ever runs when a person asks for it by name:
///
/// ```text
/// dev/in-container.sh cargo test --test eventlog -- --ignored \
///   regenerate_the_golden_fixture
/// ```
///
/// The resulting diff is the behaviour change, and reviewing it is the
/// point of the exercise. See `tests/fixtures/sessions/README.md`.
#[test]
#[ignore = "rewrites a golden fixture, run it deliberately"]
fn regenerate_the_golden_fixture() {
  let recording = record_two_layer_via();

  std::fs::write(fixture_path(FIXTURE), recording.to_text())
    .expect("the fixture directory is not writable");
}

// ---------------------------------------------------------------------
// The recorder on its own
// ---------------------------------------------------------------------

#[test]
fn a_recorder_can_be_installed_and_taken_back_by_hand() {
  let snapshot = board();
  let mut router = Router::new(&snapshot, rules(), settings(), sizes());

  assert!(router.recorder().is_none());
  assert!(router.take_recording().is_none());

  router.set_recorder(Some(Recorder::new(
    snapshot.clone(),
    settings(),
    sizes(),
  )));
  router.flip_posture();

  assert_eq!(
    router
      .recorder()
      .map(|recorder| recorder.recording().events.len()),
    Some(1)
  );

  let recording = router.take_recording().expect("a recorder is installed");

  assert!(router.recorder().is_none());
  assert_eq!(recording.snapshot, snapshot);
  assert_eq!(recording.events, vec![SessionEvent::FlipPosture]);
}
