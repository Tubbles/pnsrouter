// SPDX-License-Identifier: GPL-3.0-or-later

//! Driving a [`Router`] from a recorded KiCad routing session.
//!
//! This is the Rust counterpart of `PNS_LOG_PLAYER::ReplayLog`
//! (`qa/tools/pns/pns_log_player.cpp:91`). The shape of the loop is
//! KiCad's, event for event:
//!
//! ```text
//! items        = log->ItemsById( evt )               // KIID lookup
//! ritem        = world->FindItemByParent( items[0] )
//! routingLayer = ritem ? ritem->Layers().Start() : evt.layer
//! EVT_START_ROUTE     -> ImportSizes; UpdateSizes; StartRouting
//! EVT_FIX             -> FixRoute( p, ritem, false, false )
//! EVT_UNFIX           -> UndoLastSegment()
//! EVT_MOVE            -> Move( p, ritem )
//! EVT_TOGGLE_VIA      -> ToggleViaPlacement()
//! EVT_START_DRAG      -> StartDragging( p, ritems, 0 )
//! ```
//!
//! Three of those differ here.
//!
//! - **The KIID lookup is two steps, not one.** KiCad resolves a KIID to
//!   a `BOARD_ITEM*` and then that to a `PNS::ITEM*` (`:116`). Here the
//!   first step is [`HostMap`] and the second is the router's own
//!   [`pnsrouter::snapshot::HostIndex`], which the facade hides: a
//!   [`pnsrouter::item::HostId`] is all it wants.
//! - **The routing layer comes off the board object, not off the engine
//!   item.** `ritem->Layers().Start()` (`:118`) is the same number as the
//!   copper layer the reader recorded for that object, because the layer
//!   span of every snapshot item is built from exactly that field; taking
//!   it from the board keeps the resolution out of the facade.
//! - **Dragging is refused.** The crate has no dragger yet
//!   (`PLAN.md` milestone 8), so `EVT_START_DRAG` and
//!   `EVT_START_MULTIDRAG` become
//!   [`EventOutcome::Unsupported`] and the session never starts. Seven of
//!   the eleven cases in the corpus are drags.
//!
//! `EVT_FIX` passes `force_finish = false`, which is KiCad's third
//! argument at `:188`, so a fix is terminal only when the placer says the
//! route reached its target.
//!
//! # Where the golden is measured
//!
//! KiCad's harness never commits. It reads
//! `ROUTER::GetUpdatedItems` off the live placer node
//! (`qa/tools/pns/pns_log_player.cpp:63`) and compares that against the
//! `addedItems` and `removedItems` stored in the log
//! (`qa/tools/pns/qa_pns_regressions_main.cpp:81`). This does the same
//! through [`Router::pending_update`] **before** calling
//! [`Router::stop_routing`], because in shove mode the commit first
//! rewinds to the last locked node
//! (`pcbnew/router/pns_line_placer.cpp:1812`) and would measure a
//! different, smaller delta. The commit still happens afterwards, so that
//! the report can also say what a host would have applied.

use std::fmt;
use std::path::{Path, PathBuf};

use pnsrouter::collide::CollisionSearchOptions;
use pnsrouter::eventlog::SessionRecording;
use pnsrouter::geometry::direction45::CornerMode;
use pnsrouter::item::{ItemId, Kind, NetId};
use pnsrouter::line::Line;
use pnsrouter::router::{CommitDiff, FixOutcome, Router};
use pnsrouter::settings::{
  OptimizerEffort, RouterMode, RoutingSettings, Sizes,
};

use super::kicad_dru;
use super::kicad_pcb::{KicadBoard, read_board};
use super::kicad_snapshot::{HostMap, KicadRules, snapshot_from_board};
use super::pns_log::{
  self, CornerMode as LogCornerMode, EventKind, LogEvent, LogFile, LogSettings,
  RegressionCase, RoutingMode,
};

// ---------------------------------------------------------------------
// Loading a case
// ---------------------------------------------------------------------

/// What went wrong while loading a case, and which file it was.
#[derive(Debug, Clone)]
pub struct CaseError {
  /// The file that could not be read or understood.
  pub path: PathBuf,
  /// What was wrong with it.
  pub message: String,
}

impl fmt::Display for CaseError {
  fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(formatter, "{}: {}", self.path.display(), self.message)
  }
}

impl std::error::Error for CaseError {}

/// One regression case with every file it needs already read.
///
/// The sidecars are the ones `PNS_LOG_FILE::Load`
/// (`qa/tools/pns/pns_log_file.cpp:484`) derives from the log path by
/// substituting the extension. The board is **not** one of them: a log
/// names its board by content hash, so the caller supplies the path; see
/// the doc comment of `tests/kicad_replay.rs` for how that mapping was
/// found without KiCad's hash function.
#[derive(Debug, Clone)]
pub struct LoadedCase {
  /// The case directory's name, which is the name KiCad registers the
  /// test under (`qa/tools/pns/qa_pns_regressions_main.cpp:210`).
  pub name: String,
  /// The board file the case routes on, for test messages.
  pub board_source: String,
  /// The board itself.
  pub board: KicadBoard,
  /// The recorded events and the golden result.
  pub log: LogFile,
  /// The `.settings` sidecar, or KiCad's defaults when there is none.
  pub settings: LogSettings,
  /// The rules of the `.kicad_pro`, resolved against the board's nets.
  pub rules: KicadRules,
}

impl LoadedCase {
  /// Read a case and the board it routes on.
  ///
  /// # Errors
  ///
  /// When a file is missing or does not parse.
  pub fn load(
    case: &RegressionCase,
    board_path: &Path,
  ) -> Result<Self, CaseError> {
    let board_source = board_path
      .file_name()
      .map_or_else(String::new, |name| name.to_string_lossy().into_owned());
    let board =
      read_board(&board_source, &read_file(board_path)?).map_err(|error| {
        CaseError {
          path: board_path.to_path_buf(),
          message: error.to_string(),
        }
      })?;
    let log =
      pns_log::read_log(&read_file(&case.log_path)?).map_err(|error| {
        CaseError {
          path: case.log_path.clone(),
          message: error.to_string(),
        }
      })?;
    let settings = match &case.settings_path {
      None => LogSettings::default(),
      Some(path) => {
        pns_log::read_settings(&read_file(path)?).map_err(|error| {
          CaseError {
            path: path.clone(),
            message: error.to_string(),
          }
        })?
      }
    };
    let project_path = case.log_path.with_extension("kicad_pro");
    let project = if project_path.exists() {
      Some(read_file(&project_path)?)
    } else {
      None
    };
    let mut rules = KicadRules::from_project(project.as_deref(), &board)
      .map_err(|error| CaseError {
        path: project_path,
        message: error.to_string(),
      })?;

    // `PNS_LOG_FILE::Load` derives the rules path from the log path and
    // gives it to the DRC engine when it exists
    // (`qa/tools/pns/pns_log_file.cpp:551`).
    if let Some(path) = &case.design_rules_path {
      let design_rules =
        kicad_dru::parse(&read_file(path)?).map_err(|error| CaseError {
          path: path.clone(),
          message: error.to_string(),
        })?;

      rules.set_design_rules(design_rules);
    }

    Ok(Self {
      name: case.name.clone(),
      board_source,
      board,
      log,
      settings,
      rules,
    })
  }

  /// Whether every event of the log is one this harness can replay.
  ///
  /// False for the seven drag cases; see the module documentation.
  pub fn is_replayable(&self) -> bool {
    self
      .log
      .events
      .iter()
      .all(|event| supported(event.kind).is_some())
  }
}

/// Read a file, naming it in the error.
fn read_file(path: &Path) -> Result<String, CaseError> {
  std::fs::read_to_string(path).map_err(|error| CaseError {
    path: path.to_path_buf(),
    message: error.to_string(),
  })
}

// ---------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------

/// The router settings of a `.settings` sidecar, and what was dropped.
///
/// Port of the assignment KiCad makes by handing the parsed
/// `PNS::ROUTING_SETTINGS` straight to `LoadSettings`
/// (`qa/tools/pns/pns_log_player.cpp:98`). The four host only or dead
/// keys the crate does not carry (`suggest_finish`, `snap_to_tracks`,
/// `snap_to_pads`, and the log's own `shove_time_limit`, which is read by
/// nothing because the engine has no clock) are dropped silently; a
/// corner mode the crate cannot draw is dropped loudly, into
/// [`MappedSettings::unsupported`].
#[derive(Debug, Clone, PartialEq)]
pub struct MappedSettings {
  /// What the router is built with.
  pub settings: RoutingSettings,
  /// Settings the crate cannot honour, in the order they were found.
  pub unsupported: Vec<String>,
}

/// Map a `.settings` sidecar onto [`RoutingSettings`].
pub fn map_settings(log: &LogSettings) -> MappedSettings {
  let mut unsupported = Vec::new();
  let corner_mode = match log.corner_mode {
    LogCornerMode::Mitered45 => CornerMode::Mitered45,
    LogCornerMode::Mitered90 => CornerMode::Mitered90,
    // The two rounded modes need an arc body, `DESIGN.md` section 3;
    // `Router::toggle_corner_mode` documents the same gap.
    LogCornerMode::Rounded45 => {
      unsupported.push("corner_mode rounded 45".to_string());
      CornerMode::Mitered45
    }
    LogCornerMode::Rounded90 => {
      unsupported.push("corner_mode rounded 90".to_string());
      CornerMode::Mitered90
    }
  };

  if log.free_angle_mode {
    // `LINE_PLACER` has no free angle path in this port.
    unsupported.push("free_angle_mode".to_string());
  }

  let settings = RoutingSettings {
    mode: match log.mode {
      RoutingMode::MarkObstacles => RouterMode::MarkObstacles,
      RoutingMode::Shove => RouterMode::Shove,
      RoutingMode::Walkaround => RouterMode::Walkaround,
    },
    optimizer_effort: match log.effort {
      pns_log::OptimizationEffort::Low => OptimizerEffort::Low,
      pns_log::OptimizationEffort::Medium => OptimizerEffort::Medium,
      pns_log::OptimizationEffort::Full => OptimizerEffort::Full,
    },
    shove_vias: log.shove_vias,
    remove_loops: log.remove_loops,
    smart_pads: log.smart_pads,
    follow_mouse: log.follow_mouse,
    start_diagonal: log.start_diagonal,
    jump_over_obstacles: log.jump_over_obstacles,
    smooth_dragged_segments: log.smooth_dragged_segments,
    allow_drc_violations: log.can_violate_drc,
    free_angle_mode: log.free_angle_mode,
    optimize_entire_dragged_track: log.optimize_dragged_track,
    auto_posture: log.auto_posture,
    fix_all_segments: log.fix_all_segments,
    restrict_angles: log.restrict_angles,
    corner_mode,
    walkaround_iteration_limit: clamp_u32(log.walkaround_iteration_limit),
    shove_iteration_limit: clamp_u32(log.shove_iteration_limit),
    shove_time_limit_ms: clamp_u32(log.shove_time_limit),
    via_force_prop_iteration_limit: clamp_u32(
      log.via_force_prop_iteration_limit,
    ),
    walkaround_hug_length_threshold: log.walkaround_hug_length_threshold,
  };

  MappedSettings {
    settings,
    unsupported,
  }
}

/// Narrow a budget from the log's `i64` into the settings' `u32`.
fn clamp_u32(value: i64) -> u32 {
  u32::try_from(value.max(0)).unwrap_or(u32::MAX)
}

// ---------------------------------------------------------------------
// The report
// ---------------------------------------------------------------------

/// What one recorded event did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EventOutcome {
  /// `EVT_START_ROUTE` started a placement on this copper layer.
  Started {
    /// The layer the placement runs on.
    layer: i32,
  },
  /// `EVT_START_ROUTE` was refused. The text is
  /// [`pnsrouter::router::StartError`] as it came back.
  StartRefused {
    /// Why the facade said no.
    reason: String,
  },
  /// `EVT_MOVE`.
  Moved,
  /// `EVT_FIX`.
  Fixed {
    /// Whether the fix ended the session and committed.
    finished: bool,
  },
  /// `EVT_UNFIX`.
  Unfixed {
    /// Whether the placer handed back a point to warp the cursor to,
    /// which it does not when there is nothing left to undo.
    warped: bool,
  },
  /// `EVT_TOGGLE_VIA`.
  ViaToggled {
    /// Whether the placer honoured the request.
    honoured: bool,
  },
  /// An event kind this harness cannot replay.
  Unsupported {
    /// Which kind.
    kind: EventKind,
  },
  /// The event arrived with no session running, so nothing happened.
  ///
  /// Every event after a refused start lands here, which is also what
  /// KiCad's player does: it keeps feeding a router whose state is
  /// `IDLE` and every facade method takes its `default:` branch.
  Ignored,
}

/// How many of each kind a session added.
///
/// KiCad compares the added set with `comparePnsItems`
/// (`qa/tools/pns/pns_log_file.cpp:320`), which is a multiset equality
/// over kind, net, layers and per kind geometry. Only the counts are
/// compared here, so the breakdown is what makes a mismatch readable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AddedCounts {
  /// Track segments.
  pub segments: usize,
  /// Vias.
  pub vias: usize,
  /// The holes those vias drilled, which the node delta reports as items
  /// of their own.
  pub holes: usize,
  /// Anything else, which a single track placer should never produce.
  pub other: usize,
}

impl AddedCounts {
  /// Every item, whatever its kind.
  pub const fn total(&self) -> usize {
    self.segments + self.vias + self.holes + self.other
  }
}

impl fmt::Display for AddedCounts {
  fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      formatter,
      "{} ({} segment, {} via, {} hole, {} other)",
      self.total(),
      self.segments,
      self.vias,
      self.holes,
      self.other
    )
  }
}

/// Everything one replay produced.
#[derive(Debug, Clone)]
pub struct ReplayReport {
  /// The case directory's name.
  pub case: String,
  /// The board file it routed on.
  pub board_source: String,
  /// The settings the router was built with, and what was dropped.
  pub settings: MappedSettings,
  /// The sizes `ImportSizes` produced for the start item.
  pub sizes: Sizes,
  /// One entry per recorded event, in order.
  pub outcomes: Vec<EventOutcome>,
  /// The distinct event kinds the harness refused, in first seen order.
  pub unsupported_events: Vec<EventKind>,
  /// The board objects the snapshot described.
  pub host_map: HostMap,
  /// What the live session had added when the last event was fed in.
  pub measured_added: AddedCounts,
  /// How many board objects it had removed.
  pub measured_removed: usize,
  /// `addedItems` of the log, the golden.
  pub golden_added: usize,
  /// `removedItems` of the log, the golden.
  pub golden_removed: usize,
  /// Segments of the live session that collide with something, described
  /// by their endpoints.
  pub pending_collisions: Vec<String>,
  /// The same check over the root once the session has committed.
  pub committed_collisions: Vec<String>,
  /// What a host would have applied.
  pub commit: CommitDiff,
  /// The session as the event log recorder saw it.
  pub recording: SessionRecording,
}

impl ReplayReport {
  /// Whether nothing the session left behind collides.
  pub fn is_collision_free(&self) -> bool {
    self.pending_collisions.is_empty() && self.committed_collisions.is_empty()
  }

  /// Whether the measured counts equal the log's goldens.
  pub fn matches_golden(&self) -> bool {
    self.measured_added.total() == self.golden_added
      && self.measured_removed == self.golden_removed
  }

  /// A one line comparison against the golden, for an assertion message.
  pub fn golden_summary(&self) -> String {
    format!(
      "{}: measured added {} versus golden {}, measured removed {} versus \
       golden {}",
      self.case,
      self.measured_added,
      self.golden_added,
      self.measured_removed,
      self.golden_removed
    )
  }

  /// How many events came back with each broad outcome, for a message.
  pub fn outcome_summary(&self) -> String {
    let mut started = 0;
    let mut refused = 0;
    let mut moved = 0;
    let mut fixed = 0;
    let mut finished = 0;
    let mut unfixed = 0;
    let mut toggled = 0;
    let mut ignored = 0;
    let mut unsupported = 0;

    for outcome in &self.outcomes {
      match outcome {
        EventOutcome::Started { .. } => started += 1,
        EventOutcome::StartRefused { .. } => refused += 1,
        EventOutcome::Moved => moved += 1,
        EventOutcome::Fixed { finished: done } => {
          fixed += 1;
          finished += usize::from(*done);
        }
        EventOutcome::Unfixed { .. } => unfixed += 1,
        EventOutcome::ViaToggled { .. } => toggled += 1,
        EventOutcome::Unsupported { .. } => unsupported += 1,
        EventOutcome::Ignored => ignored += 1,
      }
    }

    format!(
      "{started} started, {refused} refused, {moved} moved, {fixed} fixed \
       ({finished} terminal), {unfixed} unfixed, {toggled} via toggles, \
       {ignored} ignored, {unsupported} unsupported"
    )
  }
}

// ---------------------------------------------------------------------
// The replay
// ---------------------------------------------------------------------

/// Replay one case against a fresh router.
///
/// Nothing is caught: a panic in the engine is a failed test, which is
/// the tier the whole corpus is meant to hold
/// (note 05 section 6.10, tier a).
///
/// The router is built with the case's own settings and with the sizes
/// [`KicadRules::import_sizes`] computes for the net of the first session
/// starting event's item, which is where KiCad's player calls
/// `ImportSizes` (`qa/tools/pns/pns_log_player.cpp:135`).
pub fn replay_case(case: &LoadedCase) -> ReplayReport {
  replay_case_with_parallelism(case, None)
}

/// [`replay_case`] with the obstacle query pinned to a thread count.
///
/// The thread count must not change what a case replays to; see
/// [`pnsrouter::node::World::set_parallelism`]. `None` leaves the world
/// at its default, which is one thread.
pub fn replay_case_with_parallelism(
  case: &LoadedCase,
  threads: Option<usize>,
) -> ReplayReport {
  let (snapshot, host_map) = snapshot_from_board(&case.board, &case.rules);
  let settings = map_settings(&case.settings);
  let start_net = start_item_net(case);
  let sizes = case
    .rules
    .import_sizes(start_net, snapshot.copper_layer_count);
  let mut router = Router::new(
    &snapshot,
    Box::new(case.rules.clone()),
    settings.settings,
    sizes.clone(),
  );

  if let Some(threads) = threads {
    router.set_parallelism(threads);
  }

  router.start_recording(&snapshot);

  let mut outcomes = Vec::with_capacity(case.log.events.len());
  let mut unsupported_events: Vec<EventKind> = Vec::new();

  for event in &case.log.events {
    let outcome = replay_event(
      &mut router,
      case,
      &host_map,
      event,
      &mut unsupported_events,
    );

    outcomes.push(outcome);
  }

  // KiCad measures here, on the live node, and never commits; see the
  // module documentation.
  let pending = router.pending_update();
  let measured_added = count_added(&router, &pending.added);
  let measured_removed = pending.removed.len();
  let pending_collisions = pending.node.map_or_else(Vec::new, |node| {
    colliding_segments(&router, case, node, &pending.added)
  });
  let commit = router.stop_routing();
  let committed_collisions = committed_collisions(&router, case, &commit);
  let recording = router
    .take_recording()
    .expect("the recorder was installed before the first event");

  ReplayReport {
    case: case.name.clone(),
    board_source: case.board_source.clone(),
    settings,
    sizes,
    outcomes,
    unsupported_events,
    host_map,
    measured_added,
    measured_removed,
    golden_added: case.log.added_items.len(),
    golden_removed: case.log.removed_items.len(),
    pending_collisions,
    committed_collisions,
    commit,
    recording,
  }
}

/// Feed one recorded event to the router.
///
/// The dispatch of `PNS_LOG_PLAYER::ReplayLog`'s switch
/// (`qa/tools/pns/pns_log_player.cpp:129` to `:234`).
fn replay_event(
  router: &mut Router,
  case: &LoadedCase,
  host_map: &HostMap,
  event: &LogEvent,
  unsupported_events: &mut Vec<EventKind>,
) -> EventOutcome {
  // :113, the first uuid is the one every single item branch reads.
  let item = event
    .uuids
    .first()
    .and_then(|uuid| host_map.host_of_uuid(uuid).map(|host| (host, uuid)));
  let host = item.map(|(host, _)| host);
  // :118
  let layer = item
    .and_then(|(_, uuid)| board_layer(&case.board, uuid))
    .unwrap_or_else(|| i32::try_from(event.layer).unwrap_or(0));
  let at = pnsrouter::geometry::vec2::Vec2::new(
    clamp_i32(event.position.x),
    clamp_i32(event.position.y),
  );

  if supported(event.kind).is_none() {
    if !unsupported_events.contains(&event.kind) {
      unsupported_events.push(event.kind);
    }

    return EventOutcome::Unsupported { kind: event.kind };
  }

  match event.kind {
    // :130
    EventKind::StartRoute => match router.start_routing(at, host, layer) {
      Ok(_frame) => EventOutcome::Started { layer },
      Err(error) => EventOutcome::StartRefused {
        reason: format!("{error:?}"),
      },
    },
    // :203
    EventKind::Move => {
      if router.routing_in_progress() {
        router.move_to(at, host);

        EventOutcome::Moved
      } else {
        EventOutcome::Ignored
      }
    }
    // :184, whose third argument is KiCad's `aForceFinish`.
    EventKind::Fix => {
      if router.routing_in_progress() {
        EventOutcome::Fixed {
          finished: matches!(
            router.fix_route(at, host, false),
            FixOutcome::Finished(_)
          ),
        }
      } else {
        EventOutcome::Ignored
      }
    }
    // :194
    EventKind::Unfix => {
      if router.routing_in_progress() {
        EventOutcome::Unfixed {
          warped: router.undo_last_segment().is_some(),
        }
      } else {
        EventOutcome::Ignored
      }
    }
    // :222
    EventKind::ToggleVia => {
      if router.routing_in_progress() {
        EventOutcome::ViaToggled {
          honoured: router.toggle_via_placement(),
        }
      } else {
        EventOutcome::Ignored
      }
    }
    // Refused above.
    EventKind::StartDrag | EventKind::StartMultiDrag | EventKind::Abort => {
      EventOutcome::Unsupported { kind: event.kind }
    }
  }
}

/// Whether this harness can replay an event kind at all.
///
/// [`None`] for the two drag events, which need milestone 8, and for
/// `EVT_ABORT`, which KiCad declares and never emits
/// (`pcbnew/router/pns_logger.h:65`) and whose meaning is therefore
/// unpinned.
pub const fn supported(kind: EventKind) -> Option<EventKind> {
  match kind {
    EventKind::StartRoute
    | EventKind::Move
    | EventKind::Fix
    | EventKind::Unfix
    | EventKind::ToggleVia => Some(kind),
    EventKind::StartDrag | EventKind::StartMultiDrag | EventKind::Abort => None,
  }
}

/// The net of the item the first session starting event names.
///
/// What `ImportSizes` reads off `aStartItem`
/// (`pcbnew/router/pns_kicad_iface.cpp:1128`). [`None`] when the session
/// started in free space, which `simple-shove-1` does, and then
/// `ImportSizes` falls through to the board's current values, that is to
/// the default net class.
fn start_item_net(case: &LoadedCase) -> Option<NetId> {
  let event = case.log.events.iter().find(|event| {
    matches!(
      event.kind,
      EventKind::StartRoute | EventKind::StartDrag | EventKind::StartMultiDrag
    )
  })?;
  let uuid = event.uuids.first()?;

  board_net(&case.board, uuid)
}

/// The copper layer a board object starts on.
///
/// The number `ritem->Layers().Start()` would answer
/// (`qa/tools/pns/pns_log_player.cpp:118`), read off the board rather
/// than off the engine: a track and a via take the layer the reader
/// recorded, a through hole pad starts on layer 0 because its span is the
/// whole stack, and a surface mount pad starts on its one copper layer.
fn board_layer(board: &KicadBoard, uuid: &str) -> Option<i32> {
  if let Some(segment) = board.segments.iter().find(|item| item.uuid == uuid) {
    return i32::try_from(segment.copper_layer).ok();
  }

  if let Some(via) = board.vias.iter().find(|item| item.uuid == uuid) {
    return i32::try_from(via.copper_layer_top).ok();
  }

  if let Some(pad) = board.pads.iter().find(|item| item.uuid == uuid) {
    return match pad.kind {
      super::kicad_pcb::PadKind::ThroughHole
      | super::kicad_pcb::PadKind::NonPlatedThroughHole => Some(0),
      _ => pad
        .copper_layers
        .first()
        .and_then(|layer| i32::try_from(*layer).ok()),
    };
  }

  None
}

/// The net of a board object, as the snapshot numbered it.
fn board_net(board: &KicadBoard, uuid: &str) -> Option<NetId> {
  let index = if let Some(segment) =
    board.segments.iter().find(|item| item.uuid == uuid)
  {
    segment.net
  } else if let Some(via) = board.vias.iter().find(|item| item.uuid == uuid) {
    via.net
  } else if let Some(pad) = board.pads.iter().find(|item| item.uuid == uuid) {
    pad.net
  } else {
    return None;
  };

  Some(NetId(u32::try_from(index.unwrap_or(0)).unwrap_or(0)))
}

/// Sort the live delta by item kind.
fn count_added(router: &Router, added: &[ItemId]) -> AddedCounts {
  let mut counts = AddedCounts::default();

  for id in added {
    let Some(item) = router.world().item(*id) else {
      counts.other += 1;

      continue;
    };

    if item.of_kind(Kind::SEGMENT) {
      counts.segments += 1;
    } else if item.of_kind(Kind::VIA) {
      counts.vias += 1;
    } else if item.of_kind(Kind::HOLE) {
      counts.holes += 1;
    } else {
      counts.other += 1;
    }
  }

  counts
}

/// Every segment of a delta that collides with something in its node.
///
/// The `LINE_T` branch of `NODE::CheckColliding`
/// (`pcbnew/router/pns_node.cpp:507`) run over one segment at a time,
/// which is the same query
/// [`pnsrouter::eventlog::assert_replay_is_collision_free`] makes over a
/// committed board.
fn colliding_segments(
  router: &Router,
  case: &LoadedCase,
  node: pnsrouter::node::NodeId,
  added: &[ItemId],
) -> Vec<String> {
  let world = router.world();
  let rules = &case.rules;
  let mut found = Vec::new();

  for id in added {
    if !world
      .item(*id)
      .is_some_and(|item| item.of_kind(Kind::SEGMENT))
    {
      continue;
    }

    let Some(line) = Line::from_segment(world, node, *id) else {
      continue;
    };

    if let Some(obstacle) = world.check_colliding_line(
      node,
      &line,
      rules,
      &CollisionSearchOptions::default(),
    ) {
      found.push(format!(
        "a segment from {:?} to {:?} on {:?} collides with item {:?}",
        line.point(0),
        line.last_point(),
        line.net(),
        obstacle.item
      ));
    }
  }

  found
}

/// The same check over the committed board.
///
/// The recipe of
/// [`pnsrouter::eventlog::assert_replay_is_collision_free`]: every
/// segment of every net the commit touched, checked against the root. A
/// board may well arrive with violations the session never went near, so
/// the nets come from the diff and not from the whole board.
fn committed_collisions(
  router: &Router,
  case: &LoadedCase,
  commit: &CommitDiff,
) -> Vec<String> {
  let world = router.world();
  let root = world.root();
  let rules = &case.rules;
  let mut nets: Vec<Option<NetId>> = Vec::new();

  for item in commit
    .added
    .iter()
    .chain(commit.updated.iter().map(|(_, item)| item))
  {
    if !nets.contains(&item.net) {
      nets.push(item.net);
    }
  }

  nets.sort_unstable();

  let mut found = Vec::new();

  for net in nets {
    for id in world.all_items_in_net(root, net, Kind::SEGMENT) {
      let Some(line) = Line::from_segment(world, root, id) else {
        continue;
      };

      if let Some(obstacle) = world.check_colliding_line(
        root,
        &line,
        rules,
        &CollisionSearchOptions::default(),
      ) {
        found.push(format!(
          "a committed segment from {:?} to {:?} on {net:?} collides with \
           item {:?}",
          line.point(0),
          line.last_point(),
          obstacle.item
        ));
      }
    }
  }

  found
}

/// Narrow a log coordinate into the engine's `i32` nanometres.
fn clamp_i32(value: i64) -> i32 {
  value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// The board objects a case's log refers to that the snapshot never
/// described.
///
/// A non empty answer means the case was matched to the wrong board, or
/// that the conversion dropped an object the session needs; either way
/// the replay would start in the wrong place, so a test can say so
/// instead of silently routing from nothing.
pub fn unresolved_uuids(case: &LoadedCase, host_map: &HostMap) -> Vec<String> {
  let mut missing = Vec::new();

  for event in &case.log.events {
    for uuid in &event.uuids {
      if host_map.host_of_uuid(uuid).is_none() && !missing.contains(uuid) {
        missing.push(uuid.clone());
      }
    }
  }

  missing
}
