// SPDX-License-Identifier: GPL-3.0-or-later

//! Recording and replaying routing sessions.
//!
//! `DESIGN.md` section 8 asks for the engine to be a pure function of
//! (snapshot, settings, event sequence). This module is that sentence
//! made executable: a [`Recorder`] the [`Router`] writes every state
//! changing call into, a [`SessionRecording`] holding the inputs and the
//! commits they produced, a [`replay`] that drives a fresh router from
//! one, and a line oriented text form so a recording can be stored as a
//! fixture and read in a review.
//!
//! It is the counterpart of KiCad's `PNS::LOGGER`
//! (`pcbnew/router/pns_logger.h:48`) and of the replay harness around it
//! (`qa/tools/pns/pns_log_player.cpp:91`), with the same central idea:
//! **geometry is not logged, only the user's input events**, and replay
//! reconstructs the geometry by running the real engine. Note 03 section
//! 7.1 is the digest of the original.
//!
//! # Where this departs from KiCad's logger, and why
//!
//! - **The board travels with the log.** KiCad identifies its board by a
//!   content hash and looks it up in a directory of `.kicad_pcb` files
//!   (`qa/tools/pns/pns_log_file.cpp:716`), which makes a missing board
//!   report success and skip
//!   (`qa/tools/pns/qa_pns_regressions_main.cpp:101`, trap 1 of note 05
//!   section 6.6). A [`SessionRecording`] carries its own
//!   [`WorldSnapshot`], so there is nothing to look up and nothing to
//!   silently skip.
//! - **Every event is recorded.** KiCad has eight event types and emits
//!   seven (`pcbnew/router/pns_logger.h:60`); a layer switch, a posture
//!   flip, a corner mode change and a size change never reach its log at
//!   all, so a session that used them cannot be replayed. Every facade
//!   method that changes state is a [`SessionEvent`] here. The one that
//!   is not is [`Router::settings_mut`]; see [`SessionEvent::SetSettings`].
//! - **The sizes are read back.** `LOGGER::ParseEventFromJSON`
//!   (`pcbnew/router/pns_logger.cpp:297`) writes a `sizes` block on every
//!   event and reads none of them back, re deriving them from the live
//!   board instead (`qa/tools/pns/pns_log_player.cpp:135`), so the field
//!   is decorative. Here [`SessionEvent::SetSizes`] is an input like any
//!   other.
//! - **The golden is the commit diff, and it is compared structurally.**
//!   `COMMIT_STATE::Compare` (`qa/tools/pns/pns_log_file.cpp:411`) is set
//!   equality over removed ids and multiset equality over added items;
//!   [`assert_replay_matches`] compares [`CommitDiff`] values in order,
//!   which is stricter and possible because the engine is deterministic.
//!   Note 05 section 6.10 recommendation 5 says not to keep the golden in
//!   the input file; it is kept here anyway, because a recording is one
//!   reviewable artefact and a split would make the two halves drift.
//!   The safeguard is procedural: see `tests/fixtures/sessions/README.md`.
//!
//! # Tiers
//!
//! Note 05 section 6.10 asks for tiered assertions, because two correct
//! push and shove implementations agree on topology long before they
//! agree on vertices:
//!
//! 1. [`replay`] alone: the session terminates and does not panic.
//! 2. [`assert_replay_is_collision_free`]: every segment the replay
//!    committed is clear under the resolver it was replayed with. This is
//!    the tier a host with its own rules can hold itself to, because a
//!    different clearance changes the geometry but must never leave a
//!    violation behind.
//! 3. [`assert_replay_matches`]: the commit diffs equal the recorded
//!    ones, vertex for vertex, and a second replay equals the first.
//!
//! # The text format
//!
//! One record per line, `#` starts a comment, blank lines are ignored,
//! and every field is an integer except the two `f64`s the engine itself
//! carries ([`RoutingSettings::walkaround_hug_length_threshold`] and a
//! solid's orientation), which are written with Rust's shortest round
//! tripping form. Variable length parts are prefixed by their count, so a
//! whole record parses out of one line's token stream.
//!
//! ```text
//! pnsrouter-session 1
//! snapshot 2 1000000
//! item 1 1 0 0 0 1 0 0 flash-default no-drill solid circle 0 0 400000 0 0 0 0 0.0 0
//! settings 0 mode walkaround
//! sizes 0 track-width 200000
//! event start-routing 0 0 1 0
//! event move-to 4000000 0 3
//! commit
//! added segment 0 0 4000000 0 -1 200000 1 0 0 -
//! ```
//!
//! The grammar in full is on [`SessionRecording::to_text`]. It is a
//! deliberate echo of the legacy whitespace format KiCad still parses as
//! a fallback (`pcbnew/router/pns_logger.cpp:274`,
//! `qa/tools/pns/pns_log_file.cpp:662`), which is line oriented for the
//! same reason: a diff of two runs is readable.

use std::fmt;

use crate::collide::CollisionSearchOptions;
use crate::geometry::direction45::CornerMode;
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::shape::{Shape, SimplePolygon};
use crate::geometry::vec2::Vec2;
use crate::item::{HostId, Kind, LayerMask, LayerRange, NetId, ViaType};
use crate::line::Line;
use crate::router::{
  CommitDiff, ContinueOutcome, FixOutcome, NewGeometry, NewItem, Router,
};
use crate::rules::RuleResolver;
use crate::settings::{OptimizerEffort, RouterMode, RoutingSettings, Sizes};
use crate::snapshot::{
  WorldGeometry, WorldItem, WorldItemFlags, WorldSnapshot,
};

/// The version stamped into the first line of a recording.
///
/// Bumped whenever the grammar changes in a way an older reader would
/// misparse. [`SessionRecording::from_text`] refuses anything else rather
/// than guessing, which is the opposite of KiCad's loader trying the JSON
/// parser and falling back to the legacy one
/// (`qa/tools/pns/pns_log_file.cpp:571`).
pub const FORMAT_VERSION: u32 = 1;

// ---------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------

/// One call a host made on a [`Router`] that changed its state.
///
/// Port of `LOGGER::EVENT_ENTRY` (`pcbnew/router/pns_logger.h:71`),
/// widened from KiCad's five fields to one variant per facade method. The
/// queries are absent because they change nothing a later event can
/// observe: [`Router::hover`] and
/// [`Router::is_starting_point_routable`] take `&self`, and
/// [`Router::nearest_ratsnest_anchor`] takes `&mut self` only to branch a
/// scratch node it drops again (`pcbnew/router/pns_topology.cpp:116`).
///
/// That scratch branch does consume uids from the world's counter, which
/// a replay that skips the query would not. It is harmless because a
/// missing block of uids shifts every later uid by the same amount and
/// the `(distance, uid)` tie break of `DESIGN.md` section 8 only reads
/// their order, and it does not arise for the two callers inside the
/// facade, which are [`SessionEvent::Finish`] and
/// [`SessionEvent::ContinueFromEnd`] and are replayed whole.
#[derive(Clone, PartialEq, Debug)]
pub enum SessionEvent {
  /// [`Router::start_routing`]. `EVT_START_ROUTE`,
  /// `pcbnew/router/pns_router.cpp:479`.
  StartRouting {
    /// The snapped point the route starts at.
    at: Vec2,
    /// The object it starts on, if any.
    start: Option<HostId>,
    /// The copper layer to route on.
    layer: i32,
  },

  /// [`Router::move_to`]. `EVT_MOVE`,
  /// `pcbnew/router/pns_router.cpp:497`.
  MoveTo {
    /// The snapped cursor point.
    at: Vec2,
    /// The object under the cursor, if any.
    end: Option<HostId>,
  },

  /// [`Router::fix_route`]. `EVT_FIX`,
  /// `pcbnew/router/pns_router.cpp:920`.
  FixRoute {
    /// The snapped point to pin the route at.
    at: Vec2,
    /// The object under the cursor, if any.
    end: Option<HostId>,
    /// Whether the fix ends the session wherever it lands.
    force_finish: bool,
  },

  /// [`Router::finish`]. KiCad's `ROUTER::Finish`
  /// (`pcbnew/router/pns_router.cpp:569`) is not logged at all.
  ///
  /// The moves and the fix it drives are not recorded separately: the
  /// routine picks its own anchor, so replaying it whole is what
  /// reproduces the session.
  Finish,

  /// [`Router::continue_from_end`], likewise unlogged in KiCad
  /// (`pcbnew/router/pns_router.cpp:617`).
  ContinueFromEnd,

  /// [`Router::undo_last_segment`]. `EVT_UNFIX`,
  /// `pcbnew/router/pns_router.cpp:952`, the one event KiCad records with
  /// no payload at all.
  UndoLastSegment,

  /// [`Router::switch_layer`]. Not logged by KiCad, which is why a
  /// session that changed layer cannot be replayed there.
  SwitchLayer {
    /// The copper layer to move to.
    layer: i32,
  },

  /// [`Router::toggle_via_placement`]. `EVT_TOGGLE_VIA`,
  /// `pcbnew/router/pns_router.cpp:1027`.
  ToggleViaPlacement,

  /// [`Router::flip_posture`]. Not logged by KiCad.
  FlipPosture,

  /// [`Router::toggle_corner_mode`]. Not logged by KiCad, although it
  /// writes through to the settings the replay is driven with.
  ToggleCornerMode,

  /// [`Router::set_ortho_mode`]. Not logged by KiCad.
  SetOrthoMode {
    /// Whether the head is forced to a single segment.
    ortho: bool,
  },

  /// [`Router::set_sizes`]. KiCad stamps a `SIZES_SETTINGS` on every
  /// event and reads none back (`pcbnew/router/pns_logger.cpp:297`).
  SetSizes {
    /// The geometry the next route is placed with.
    sizes: Sizes,
  },

  /// [`Router::set_settings`].
  ///
  /// [`Router::settings_mut`] is the one state changing facade method
  /// that cannot be recorded: it hands out a mutable reference and the
  /// router never learns what was done with it, exactly as KiCad's
  /// `Settings()` does (`pcbnew/router/pns_router.h:227`). A host that
  /// wants its settings changes in the log goes through
  /// [`Router::set_settings`] instead.
  SetSettings {
    /// The whole settings block, because a partial diff would need a
    /// field name in the format and would rot when a field is added.
    settings: RoutingSettings,
  },

  /// [`Router::stop_routing`]. KiCad's host calls `CommitRouting` and
  /// `StopRouting` without logging either.
  StopRouting,

  /// [`Router::abort_routing`]. `EVT_ABORT` exists in KiCad's enum
  /// (`pcbnew/router/pns_logger.h:65`) and nothing emits it.
  AbortRouting,

  /// [`Router::assign_host_ids`], the step KiCad has no counterpart for
  /// because its interface writes into the board as it goes
  /// (`pcbnew/router/pns_kicad_iface.cpp:2775`).
  AssignHostIds {
    /// Index into the last [`CommitDiff::added`] and the id the host gave
    /// that entry.
    ids: Vec<(usize, HostId)>,
  },
}

// ---------------------------------------------------------------------
// The recording
// ---------------------------------------------------------------------

/// Everything one routing session depended on, and what it produced.
///
/// Port of `LOGGER::LOG_DATA` (`pcbnew/router/pns_logger.h:94`) with the
/// board folded in. The four input fields are the whole of the engine's
/// input, so two replays of one recording are two runs of the same pure
/// function; [`SessionRecording::results`] is what that function returned
/// on the day the recording was made.
#[derive(Clone, PartialEq, Debug)]
pub struct SessionRecording {
  /// The board the session ran on.
  pub snapshot: WorldSnapshot,
  /// The settings the router was built with.
  pub settings: RoutingSettings,
  /// The sizes the router was built with.
  pub sizes: Sizes,
  /// Every state changing call, in order.
  pub events: Vec<SessionEvent>,
  /// One entry per commit the session made, in order.
  ///
  /// A commit is what [`Router::stop_routing`] returns, whether the host
  /// called it, a terminal [`Router::fix_route`] reached it, or
  /// [`Router::continue_from_end`] passed through it. This is the golden
  /// of note 05 section 6.6, kept beside its input rather than in a
  /// second file; see the module documentation.
  pub results: Vec<CommitDiff>,
}

impl SessionRecording {
  /// An empty recording over a board.
  pub fn new(
    snapshot: WorldSnapshot,
    settings: RoutingSettings,
    sizes: Sizes,
  ) -> Self {
    Self {
      snapshot,
      settings,
      sizes,
      events: Vec::new(),
      results: Vec::new(),
    }
  }
}

/// The sink a [`Router`] writes its session into.
///
/// Port of `PNS::LOGGER` (`pcbnew/router/pns_logger.h:48`), which is also
/// a plain accumulator with no opinions. Install one with
/// [`Router::set_recorder`] or [`Router::start_recording`] and take the
/// result back with [`Router::take_recording`].
///
/// KiCad clears its log at the top of `StartRouting`
/// (`pcbnew/router/pns_router.cpp:478`), so an in memory log there only
/// ever holds one placement. This one accumulates until it is taken,
/// because a host session is several placements and the commits between
/// them are the interesting part.
#[derive(Clone, PartialEq, Debug)]
pub struct Recorder {
  /// What has been recorded so far.
  recording: SessionRecording,
}

impl Recorder {
  /// A recorder over a board, the settings and the sizes a router was
  /// built with.
  ///
  /// The three inputs have to be the ones the router actually holds, or a
  /// replay starts from a different state than the session did.
  /// [`Router::start_recording`] takes them off the router itself and
  /// cannot get them wrong.
  pub fn new(
    snapshot: WorldSnapshot,
    settings: RoutingSettings,
    sizes: Sizes,
  ) -> Self {
    Self {
      recording: SessionRecording::new(snapshot, settings, sizes),
    }
  }

  /// Append one event.
  pub fn push_event(&mut self, event: SessionEvent) {
    self.recording.events.push(event);
  }

  /// Append one commit result.
  pub fn push_commit(&mut self, diff: &CommitDiff) {
    self.recording.results.push(diff.clone());
  }

  /// What has been recorded so far.
  pub const fn recording(&self) -> &SessionRecording {
    &self.recording
  }

  /// Take the recording out, consuming the recorder.
  pub fn into_recording(self) -> SessionRecording {
    self.recording
  }
}

// ---------------------------------------------------------------------
// Replay
// ---------------------------------------------------------------------

/// What a replay produced.
///
/// The counterpart of `PNS_LOG_PLAYER::GetRouterUpdatedItems`
/// (`qa/tools/pns/pns_log_player.cpp:63`), which builds a `COMMIT_STATE`
/// out of the router after the events have been fed in.
pub struct ReplayOutcome {
  /// One entry per commit, to compare against
  /// [`SessionRecording::results`].
  pub diffs: Vec<CommitDiff>,
  /// How many events answered with a [`crate::router::PreviewFrame`].
  ///
  /// One per successful start, per move, per non terminal fix, per finish
  /// that did not commit, and per continue that restarted. It counts
  /// events and not frames, so the moves a finish drives inside itself
  /// are one, the same way they are one event.
  ///
  /// KiCad's player discards its frames entirely, with the comment
  /// "fixme: update the state with the head trace"
  /// (`qa/tools/pns/pns_log_player.cpp:83`); the count is kept here so a
  /// test can pin that a session drew as much as it did.
  pub frames_count: usize,
  /// How many events were fed in, which is
  /// `recording.events.len()`.
  pub events_count: usize,
  /// The router as the last event left it.
  ///
  /// Kept so that a caller can inspect the committed world, which is what
  /// [`assert_replay_is_collision_free`] does.
  pub router: Router,
}

/// Drive a fresh router from a recording.
///
/// Port of `PNS_LOG_PLAYER::ReplayLog`
/// (`qa/tools/pns/pns_log_player.cpp:91`): build the world, apply the
/// recorded settings, and map every event onto the facade call it came
/// from. Nothing is resolved by lookup the way KiCad resolves a `KIID`
/// through `NODE::FindItemByParent` (`:116`), because a
/// [`HostId`] is already the host's own handle and the snapshot travels
/// with the events.
///
/// The resolver is the caller's: replaying with the rules the session was
/// recorded under reproduces its geometry, and replaying with different
/// rules is the tier 2 check of the module documentation.
pub fn replay(
  recording: &SessionRecording,
  resolver: Box<dyn RuleResolver>,
) -> ReplayOutcome {
  replay_on(recording, resolver, None)
}

/// [`replay`] with the obstacle query pinned to a thread count.
///
/// The thread count is not part of a recording, and it must not change
/// what a replay answers; see [`crate::node::World::set_parallelism`].
/// This exists so that `tests/parallelism.rs` can replay the same
/// recording on one thread and on many and compare the commits.
pub fn replay_with_parallelism(
  recording: &SessionRecording,
  resolver: Box<dyn RuleResolver>,
  threads: usize,
) -> ReplayOutcome {
  replay_on(recording, resolver, Some(threads))
}

/// The body of [`replay`] and [`replay_with_parallelism`].
fn replay_on(
  recording: &SessionRecording,
  resolver: Box<dyn RuleResolver>,
  threads: Option<usize>,
) -> ReplayOutcome {
  let mut router = Router::new(
    &recording.snapshot,
    resolver,
    recording.settings,
    recording.sizes.clone(),
  );

  if let Some(threads) = threads {
    router.set_parallelism(threads);
  }

  let mut diffs = Vec::new();
  let mut frames_count = 0;

  for event in &recording.events {
    apply(&mut router, event, &mut diffs, &mut frames_count);
  }

  ReplayOutcome {
    diffs,
    frames_count,
    events_count: recording.events.len(),
    router,
  }
}

/// Feed one event to a router, collecting what it answered.
///
/// The `switch` of `qa/tools/pns/pns_log_player.cpp:105` to `:237`.
fn apply(
  router: &mut Router,
  event: &SessionEvent,
  diffs: &mut Vec<CommitDiff>,
  frames_count: &mut usize,
) {
  match event {
    SessionEvent::StartRouting { at, start, layer } => {
      if router.start_routing(*at, *start, *layer).is_ok() {
        *frames_count += 1;
      }
    }
    SessionEvent::MoveTo { at, end } => {
      router.move_to(*at, *end);
      *frames_count += 1;
    }
    SessionEvent::FixRoute {
      at,
      end,
      force_finish,
    } => match router.fix_route(*at, *end, *force_finish) {
      FixOutcome::Continue(_) => *frames_count += 1,
      FixOutcome::Finished(diff) => diffs.push(diff),
    },
    SessionEvent::Finish => match router.finish() {
      Some(FixOutcome::Continue(_)) => *frames_count += 1,
      Some(FixOutcome::Finished(diff)) => diffs.push(diff),
      None => {}
    },
    SessionEvent::ContinueFromEnd => {
      if let Some(ContinueOutcome { diff, frame, .. }) =
        router.continue_from_end()
      {
        diffs.push(diff);

        if frame.is_some() {
          *frames_count += 1;
        }
      }
    }
    SessionEvent::UndoLastSegment => {
      router.undo_last_segment();
    }
    SessionEvent::SwitchLayer { layer } => {
      router.switch_layer(*layer);
    }
    SessionEvent::ToggleViaPlacement => {
      router.toggle_via_placement();
    }
    SessionEvent::FlipPosture => router.flip_posture(),
    SessionEvent::ToggleCornerMode => router.toggle_corner_mode(),
    SessionEvent::SetOrthoMode { ortho } => router.set_ortho_mode(*ortho),
    SessionEvent::SetSizes { sizes } => router.set_sizes(sizes.clone()),
    SessionEvent::SetSettings { settings } => router.set_settings(*settings),
    SessionEvent::StopRouting => diffs.push(router.stop_routing()),
    SessionEvent::AbortRouting => router.abort_routing(),
    SessionEvent::AssignHostIds { ids } => {
      router.assign_host_ids(ids);
    }
  }
}

/// Tier 3: the replay reproduces the recorded commits, twice.
///
/// The strict tier of note 05 section 6.10: exact geometry, which is
/// stronger than `COMMIT_STATE::Compare`
/// (`qa/tools/pns/pns_log_file.cpp:411`) because the engine is
/// deterministic and the diffs come out in a fixed order rather than as
/// sets.
///
/// The second replay is the determinism guard of `DESIGN.md` section 8: a
/// run that reads a hash map's order or a clock passes the first
/// comparison and fails this one.
///
/// The resolver arrives as a factory because two routers are built and a
/// `Box<dyn RuleResolver>` cannot be cloned.
///
/// # Panics
///
/// When the replayed commits differ from the recorded ones, or when two
/// replays of the same recording differ.
pub fn assert_replay_matches<Rules>(
  recording: &SessionRecording,
  resolver: Rules,
) where
  Rules: Fn() -> Box<dyn RuleResolver>,
{
  let first = replay(recording, resolver());

  assert_eq!(
    first.diffs.len(),
    recording.results.len(),
    "the replay made {} commits and the recording holds {}",
    first.diffs.len(),
    recording.results.len()
  );

  for (index, (replayed, recorded)) in
    first.diffs.iter().zip(&recording.results).enumerate()
  {
    assert_eq!(
      replayed, recorded,
      "commit {index} of the replay differs from the recording"
    );
  }

  let second = replay(recording, resolver());

  assert_eq!(
    second.diffs, first.diffs,
    "two replays of one recording produced different commits"
  );
  assert_eq!(
    second.frames_count, first.frames_count,
    "two replays of one recording produced different frame counts"
  );
}

/// Tier 2: the replay leaves no clearance violation behind.
///
/// The tier a host with its own rules holds itself to. Different
/// clearances give different geometry, so [`assert_replay_matches`] does
/// not apply, but the answer must still be clear: every segment on a net
/// the session committed to is checked against the committed world with
/// [`crate::node::World::check_colliding_line`], the `LINE_T` branch of
/// `NODE::CheckColliding` (`pcbnew/router/pns_node.cpp:507`).
///
/// The nets come from the diffs rather than from the whole board, because
/// a board may well arrive with violations the session never touched.
///
/// # Panics
///
/// When a segment the replay committed collides with anything.
pub fn assert_replay_is_collision_free<Rules>(
  recording: &SessionRecording,
  resolver: Rules,
) where
  Rules: Fn() -> Box<dyn RuleResolver>,
{
  let outcome = replay(recording, resolver());
  let rules = resolver();
  let world = outcome.router.world();
  let root = world.root();

  for net in committed_nets(&outcome.diffs) {
    for id in world.all_items_in_net(root, net, Kind::SEGMENT) {
      let Some(line) = Line::from_segment(world, root, id) else {
        continue;
      };

      assert!(
        world
          .check_colliding_line(
            root,
            &line,
            rules.as_ref(),
            &CollisionSearchOptions::default()
          )
          .is_none(),
        "a committed segment from {:?} to {:?} on {net:?} collides",
        line.point(0),
        line.last_point()
      );
    }
  }
}

/// Every net a set of commits created or rewrote an item on, in order.
///
/// A sorted [`Vec`] and not a set, because `DESIGN.md` section 8 forbids
/// iterating a hash container.
fn committed_nets(diffs: &[CommitDiff]) -> Vec<Option<NetId>> {
  let mut nets: Vec<Option<NetId>> = Vec::new();

  for diff in diffs {
    let added = diff.added.iter();
    let updated = diff.updated.iter().map(|(_, item)| item);

    for item in added.chain(updated) {
      if !nets.contains(&item.net) {
        nets.push(item.net);
      }
    }
  }

  nets.sort_unstable();
  nets
}

// ---------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------

/// Append one record and its newline.
///
/// Used instead of `write!` so that no call site has to discard a
/// `fmt::Result` that writing to a [`String`] cannot produce.
fn line(out: &mut String, record: &str) {
  out.push_str(record);
  out.push('\n');
}

/// `-` for absent, the number otherwise.
///
/// The one sentinel the format has. KiCad writes an empty `uuids` array
/// for the same case (`qa/data/pcbnew/pns_regressions`, case
/// `simple-shove-1`).
fn optional(value: Option<u64>) -> String {
  value.map_or_else(|| "-".to_string(), |value| value.to_string())
}

/// `<x> <y>`.
fn vec2_text(at: Vec2) -> String {
  format!("{} {}", at.x, at.y)
}

/// `<ax> <ay> <bx> <by> <index>`.
///
/// The parent shape index is carried because [`Seg::index`] is part of
/// the value (`libs/kimath/include/geometry/seg.h:398`) and the optimizer
/// reads it back (`pcbnew/router/pns_optimizer.cpp:562`).
fn seg_text(seg: Seg) -> String {
  format!("{} {} {}", vec2_text(seg.a), vec2_text(seg.b), seg.index)
}

/// `chain <closed> <width> <count> <x> <y>...`.
fn chain_text(chain: &LineChain) -> String {
  let mut text = format!(
    "chain {} {} {}",
    u8::from(chain.is_closed()),
    chain.width(),
    chain.point_count()
  );

  for point in chain.points() {
    text.push(' ');
    text.push_str(&vec2_text(*point));
  }

  text
}

/// One shape, in prefix form so that it parses out of a token stream.
///
/// The six variants of [`Shape`] against the three
/// `formatShapeAsJSON` writes (`pcbnew/router/pns_logger.cpp:231`), which
/// emits `null` for everything else. A snapshot is the engine's input and
/// not a debug dump, so nothing may be dropped here.
fn shape_text(shape: &Shape) -> String {
  match shape {
    Shape::Circle { center, radius } => {
      format!("circle {} {radius}", vec2_text(*center))
    }
    Shape::Rect {
      origin,
      size,
      radius,
    } => format!("rect {} {} {radius}", vec2_text(*origin), vec2_text(*size)),
    Shape::Segment { seg, width } => {
      format!("segment {} {width}", seg_text(*seg))
    }
    Shape::Simple(polygon) => {
      format!("simple {}", chain_text(polygon.vertices()))
    }
    Shape::LineChain(chain) => format!("line-chain {}", chain_text(chain)),
    Shape::Compound(shapes) => {
      let mut text = format!("compound {}", shapes.len());

      for member in shapes {
        text.push(' ');
        text.push_str(&shape_text(member));
      }

      text
    }
  }
}

/// The word for a via type.
fn via_type_text(via_type: ViaType) -> &'static str {
  match via_type {
    ViaType::NotDefined => "not-defined",
    ViaType::MicroVia => "micro",
    ViaType::Blind => "blind",
    ViaType::Buried => "buried",
    ViaType::Through => "through",
  }
}

/// One snapshot geometry, in prefix form.
fn world_geometry_text(geometry: &WorldGeometry) -> String {
  match geometry {
    WorldGeometry::Segment { seg, width } => {
      format!("segment {} {width}", seg_text(*seg))
    }
    WorldGeometry::Via {
      pos,
      diameter,
      drill,
      via_type,
      is_free,
    } => format!(
      "via {} {diameter} {drill} {} {}",
      vec2_text(*pos),
      via_type_text(*via_type),
      u8::from(*is_free)
    ),
    WorldGeometry::Solid {
      shape,
      pos,
      offset,
      orientation_degrees,
      anchors,
    } => {
      let mut text = format!(
        "solid {} {} {} {orientation_degrees:?} {}",
        shape_text(shape),
        vec2_text(*pos),
        vec2_text(*offset),
        anchors.len()
      );

      for anchor in anchors {
        text.push(' ');
        text.push_str(&vec2_text(*anchor));
      }

      text
    }
    WorldGeometry::Hole { shape } => format!("hole {}", shape_text(shape)),
  }
}

/// `flash-default`, or `flash <count> <layer>...`.
///
/// [`WorldItem::flashed_layers`] is [`None`] for "the mask follows the
/// layers", which is not the same as an empty mask, so the two cases have
/// two spellings.
fn flash_text(mask: Option<LayerMask>) -> String {
  let Some(mask) = mask else {
    return "flash-default".to_string();
  };
  let layers: Vec<i32> = (0..=LayerMask::MAX_LAYER)
    .filter(|layer| mask.is_flashed_on(*layer))
    .collect();
  let mut text = format!("flash {}", layers.len());

  for layer in layers {
    text.push(' ');
    text.push_str(&layer.to_string());
  }

  text
}

/// One snapshot item, on one line.
fn item_text(item: &WorldItem) -> String {
  let drill = item.hole.as_ref().map_or_else(
    || "no-drill".to_string(),
    |hole| format!("drill {}", shape_text(hole)),
  );

  format!(
    "item {} {} {} {} {} {} {} {} {} {drill} {}",
    item.id.0,
    optional(item.net.map(|net| u64::from(net.0))),
    item.layers.start(),
    item.layers.end(),
    u8::from(item.flags.locked),
    u8::from(item.flags.routable),
    u8::from(item.flags.free_pad),
    u8::from(item.flags.compound_primitive),
    flash_text(item.flashed_layers),
    world_geometry_text(&item.geometry)
  )
}

/// One committed item: its geometry, then its net, layers and source.
fn new_item_text(item: &NewItem) -> String {
  let geometry = match item.geometry {
    NewGeometry::Segment { seg, width } => {
      format!("segment {} {width}", seg_text(seg))
    }
    NewGeometry::Via {
      pos,
      diameter,
      drill,
      via_type,
    } => format!(
      "via {} {diameter} {drill} {}",
      vec2_text(pos),
      via_type_text(via_type)
    ),
  };

  format!(
    "{geometry} {} {} {} {}",
    optional(item.net.map(|net| u64::from(net.0))),
    item.layers.start(),
    item.layers.end(),
    optional(item.source.map(|host| host.0))
  )
}

/// Every `settings <block> <key> <value>` line of one block.
fn settings_lines(out: &mut String, block: usize, settings: &RoutingSettings) {
  let mode = match settings.mode {
    RouterMode::MarkObstacles => "mark-obstacles",
    RouterMode::Shove => "shove",
    RouterMode::Walkaround => "walkaround",
  };
  let effort = match settings.optimizer_effort {
    OptimizerEffort::Low => "low",
    OptimizerEffort::Medium => "medium",
    OptimizerEffort::Full => "full",
  };
  let corner_mode = match settings.corner_mode {
    CornerMode::Mitered45 => "mitered-45",
    CornerMode::Mitered90 => "mitered-90",
  };
  let flags = [
    ("shove-vias", settings.shove_vias),
    ("remove-loops", settings.remove_loops),
    ("smart-pads", settings.smart_pads),
    ("follow-mouse", settings.follow_mouse),
    ("start-diagonal", settings.start_diagonal),
    ("jump-over-obstacles", settings.jump_over_obstacles),
    ("smooth-dragged-segments", settings.smooth_dragged_segments),
    ("allow-drc-violations", settings.allow_drc_violations),
    ("free-angle-mode", settings.free_angle_mode),
    (
      "optimize-entire-dragged-track",
      settings.optimize_entire_dragged_track,
    ),
    ("auto-posture", settings.auto_posture),
    ("fix-all-segments", settings.fix_all_segments),
    ("restrict-angles", settings.restrict_angles),
  ];
  let counts = [
    (
      "walkaround-iteration-limit",
      settings.walkaround_iteration_limit,
    ),
    ("shove-iteration-limit", settings.shove_iteration_limit),
    ("shove-time-limit-ms", settings.shove_time_limit_ms),
    (
      "via-force-prop-iteration-limit",
      settings.via_force_prop_iteration_limit,
    ),
  ];

  line(out, &format!("settings {block} mode {mode}"));
  line(out, &format!("settings {block} effort {effort}"));

  for (key, value) in flags {
    line(out, &format!("settings {block} {key} {}", u8::from(value)));
  }

  line(out, &format!("settings {block} corner-mode {corner_mode}"));

  for (key, value) in counts {
    line(out, &format!("settings {block} {key} {value}"));
  }

  line(
    out,
    &format!(
      "settings {block} walkaround-hug-length-threshold {:?}",
      settings.walkaround_hug_length_threshold
    ),
  );
}

/// Every `sizes <block> <key> <value>` line of one block.
fn sizes_lines(out: &mut String, block: usize, sizes: &Sizes) {
  let lengths = [
    ("clearance", sizes.clearance),
    ("min-clearance", sizes.min_clearance),
    ("track-width", sizes.track_width),
    ("board-min-track-width", sizes.board_min_track_width),
    ("via-diameter", sizes.via_diameter),
    ("via-drill", sizes.via_drill),
    ("diff-pair-width", sizes.diff_pair_width),
    ("diff-pair-gap", sizes.diff_pair_gap),
    ("diff-pair-via-gap", sizes.diff_pair_via_gap),
    ("hole-to-hole", sizes.hole_to_hole),
    ("diff-pair-hole-to-hole", sizes.diff_pair_hole_to_hole),
    ("diff-pair-copper-to-hole", sizes.diff_pair_copper_to_hole),
  ];
  let flags = [
    ("track-width-is-explicit", sizes.track_width_is_explicit),
    (
      "diff-pair-via-gap-same-as-trace-gap",
      sizes.diff_pair_via_gap_same_as_trace_gap,
    ),
  ];

  for (key, value) in lengths {
    line(out, &format!("sizes {block} {key} {value}"));
  }

  for (key, value) in flags {
    line(out, &format!("sizes {block} {key} {}", u8::from(value)));
  }

  line(
    out,
    &format!("sizes {block} via-type {}", via_type_text(sizes.via_type)),
  );

  for (first, second) in &sizes.layer_pairs {
    line(out, &format!("sizes {block} layer-pair {first} {second}"));
  }
}

/// One `event` line.
///
/// `sizes_block` and `settings_block` are the indices the next
/// [`SessionEvent::SetSizes`] and [`SessionEvent::SetSettings`] were
/// written under; the caller advances them in event order, which is the
/// order [`SessionRecording::to_text`] emitted the blocks in.
fn event_text(
  event: &SessionEvent,
  sizes_block: usize,
  settings_block: usize,
) -> String {
  match event {
    SessionEvent::StartRouting { at, start, layer } => format!(
      "event start-routing {} {} {layer}",
      vec2_text(*at),
      optional(start.map(|host| host.0))
    ),
    SessionEvent::MoveTo { at, end } => format!(
      "event move-to {} {}",
      vec2_text(*at),
      optional(end.map(|host| host.0))
    ),
    SessionEvent::FixRoute {
      at,
      end,
      force_finish,
    } => format!(
      "event fix-route {} {} {}",
      vec2_text(*at),
      optional(end.map(|host| host.0)),
      u8::from(*force_finish)
    ),
    SessionEvent::Finish => "event finish".to_string(),
    SessionEvent::ContinueFromEnd => "event continue-from-end".to_string(),
    SessionEvent::UndoLastSegment => "event undo-last-segment".to_string(),
    SessionEvent::SwitchLayer { layer } => {
      format!("event switch-layer {layer}")
    }
    SessionEvent::ToggleViaPlacement => {
      "event toggle-via-placement".to_string()
    }
    SessionEvent::FlipPosture => "event flip-posture".to_string(),
    SessionEvent::ToggleCornerMode => "event toggle-corner-mode".to_string(),
    SessionEvent::SetOrthoMode { ortho } => {
      format!("event set-ortho-mode {}", u8::from(*ortho))
    }
    SessionEvent::SetSizes { .. } => format!("event set-sizes {sizes_block}"),
    SessionEvent::SetSettings { .. } => {
      format!("event set-settings {settings_block}")
    }
    SessionEvent::StopRouting => "event stop-routing".to_string(),
    SessionEvent::AbortRouting => "event abort-routing".to_string(),
    SessionEvent::AssignHostIds { ids } => {
      let mut text = format!("event assign-host-ids {}", ids.len());

      for (index, host) in ids {
        text.push_str(&format!(" {index} {}", host.0));
      }

      text
    }
  }
}

impl SessionRecording {
  /// The recording as text.
  ///
  /// # The grammar
  ///
  /// A record is one line. `#` starts a comment that runs to the end of
  /// the line, and a blank line is ignored. Tokens are separated by
  /// whitespace; there are no strings and no escapes. Variable length
  /// runs are prefixed by their count, so every record parses out of one
  /// token stream. `-` is the only sentinel and means "absent".
  ///
  /// ```text
  /// pnsrouter-session <version>
  /// snapshot <copper-layer-count> <max-clearance>
  /// exclusion <shape>
  /// item <host> <net> <layer-start> <layer-end> <locked> <routable>
  ///      <free-pad> <compound> <flash> <drill> <geometry>
  /// settings <block> <key> <value>
  /// sizes <block> <key> <value>
  /// event <name> <fields...>
  /// commit
  /// removed <host>
  /// added <new-item>
  /// updated <host> <new-item>
  /// ```
  ///
  /// `<flash>` is `flash-default` for [`WorldItem::flashed_layers`] of
  /// [`None`] and `flash <count> <layer>...` otherwise. `<drill>` is
  /// `no-drill` or `drill <shape>`. A `<geometry>` is one of
  /// `segment <seg> <width>`, `via <x> <y> <diameter> <drill> <via-type>
  /// <is-free>`, `solid <shape> <pos> <offset> <orientation>
  /// <anchor-count> <anchor>...` or `hole <shape>`, and a `<seg>` is
  /// `<ax> <ay> <bx> <by> <parent-index>`.
  ///
  /// A `<shape>` is `circle <x> <y> <radius>`, `rect <x> <y> <w> <h>
  /// <corner-radius>`, `segment <seg> <width>`, `simple <chain>`,
  /// `line-chain <chain>` or `compound <count> <shape>...`, and a
  /// `<chain>` is `chain <closed> <width> <count> <x> <y>...`.
  ///
  /// A `<new-item>` is `<segment|via geometry> <net> <layer-start>
  /// <layer-end> <source-host>`. A `<pos>`, an `<offset>` and an
  /// `<anchor>` are each an `<x> <y>` pair.
  ///
  /// The events, one line each:
  ///
  /// ```text
  /// start-routing <x> <y> <start-host> <layer>
  /// move-to <x> <y> <end-host>
  /// fix-route <x> <y> <end-host> <force-finish>
  /// finish
  /// continue-from-end
  /// undo-last-segment
  /// switch-layer <layer>
  /// toggle-via-placement
  /// flip-posture
  /// toggle-corner-mode
  /// set-ortho-mode <ortho>
  /// set-sizes <block>
  /// set-settings <block>
  /// stop-routing
  /// abort-routing
  /// assign-host-ids <count> <added-index> <host>...
  /// ```
  ///
  /// Block `0` of `settings` and of `sizes` is what the router was built
  /// with; higher blocks are the payloads of the `set-settings` and
  /// `set-sizes` events, numbered in the order those events appear. A key
  /// a block does not mention keeps its default, so a hand written
  /// fixture may list only what it cares about; the writer always lists
  /// everything.
  ///
  /// Booleans are `0` and `1`, enums are words, and every number is an
  /// integer except a solid's orientation in degrees and
  /// [`RoutingSettings::walkaround_hug_length_threshold`], the two `f64`s
  /// the engine itself carries (`DESIGN.md` section 8). Both are written
  /// with Rust's shortest form that reads back to the same bits.
  pub fn to_text(&self) -> String {
    let mut out = String::new();
    let mut sizes_blocks: Vec<&Sizes> = vec![&self.sizes];
    let mut settings_blocks: Vec<&RoutingSettings> = vec![&self.settings];

    for event in &self.events {
      match event {
        SessionEvent::SetSizes { sizes } => sizes_blocks.push(sizes),
        SessionEvent::SetSettings { settings } => {
          settings_blocks.push(settings);
        }
        _ => {}
      }
    }

    line(
      &mut out,
      "# A pnsrouter session recording. The grammar is on",
    );
    line(
      &mut out,
      "# `SessionRecording::to_text` in src/eventlog.rs;",
    );
    line(
      &mut out,
      "# regenerating a fixture is a deliberate act, see",
    );
    line(&mut out, "# tests/fixtures/sessions/README.md.");
    line(&mut out, &format!("pnsrouter-session {FORMAT_VERSION}"));
    line(&mut out, "");
    line(&mut out, "# snapshot <copper-layer-count> <max-clearance>");
    line(
      &mut out,
      &format!(
        "snapshot {} {}",
        self.snapshot.copper_layer_count, self.snapshot.max_clearance
      ),
    );

    for shape in &self.snapshot.edge_exclusions {
      line(&mut out, &format!("exclusion {}", shape_text(shape)));
    }

    if !self.snapshot.items.is_empty() {
      line(&mut out, "");
      line(
        &mut out,
        "# item <host> <net> <layers> <locked> <routable> <free-pad> \
         <compound> <flash> <drill> <geometry>",
      );

      for item in &self.snapshot.items {
        line(&mut out, &item_text(item));
      }
    }

    for (block, settings) in settings_blocks.iter().enumerate() {
      line(&mut out, "");
      settings_lines(&mut out, block, settings);
    }

    for (block, sizes) in sizes_blocks.iter().enumerate() {
      line(&mut out, "");
      sizes_lines(&mut out, block, sizes);
    }

    if !self.events.is_empty() {
      let mut sizes_block = 0;
      let mut settings_block = 0;

      line(&mut out, "");
      line(&mut out, "# event <name> <fields>");

      for event in &self.events {
        match event {
          SessionEvent::SetSizes { .. } => sizes_block += 1,
          SessionEvent::SetSettings { .. } => settings_block += 1,
          _ => {}
        }

        line(&mut out, &event_text(event, sizes_block, settings_block));
      }
    }

    for diff in &self.results {
      line(&mut out, "");
      line(
        &mut out,
        "# commit, then its removed, added and updated items",
      );
      line(&mut out, "commit");

      for host in &diff.removed {
        line(&mut out, &format!("removed {}", host.0));
      }

      for item in &diff.added {
        line(&mut out, &format!("added {}", new_item_text(item)));
      }

      for (host, item) in &diff.updated {
        line(
          &mut out,
          &format!("updated {} {}", host.0, new_item_text(item)),
        );
      }
    }

    out
  }

  /// Read a recording back.
  ///
  /// # Errors
  ///
  /// [`ParseError`], which names the one based line the offending token
  /// is on. Unknown keys, unknown words and trailing tokens are all
  /// errors, so a typo surfaces instead of silently keeping a default.
  pub fn from_text(text: &str) -> Result<Self, ParseError> {
    parse_recording(text)
  }
}

// ---------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------

/// What went wrong while reading a recording, and where.
///
/// The line number is what makes a hand edited fixture usable: KiCad's
/// own legacy parser (`qa/tools/pns/pns_log_file.cpp:662`) reports
/// nothing at all and simply stops.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct ParseError {
  /// One based line the offending token is on.
  pub line: u32,
  /// What the reader expected, in prose.
  pub message: String,
}

impl ParseError {
  /// An error against one line.
  pub fn new(line: u32, message: impl Into<String>) -> Self {
    Self {
      line,
      message: message.into(),
    }
  }
}

impl fmt::Display for ParseError {
  fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(formatter, "line {}: {}", self.line, self.message)
  }
}

impl std::error::Error for ParseError {}

/// A cursor over one line's tokens.
struct Tokens<'a> {
  /// One based number of the line the tokens came from.
  line: u32,
  /// The tokens, comment already stripped.
  words: Vec<&'a str>,
  /// How many have been taken.
  taken: usize,
}

impl<'a> Tokens<'a> {
  /// Split one line into tokens, dropping a `#` comment.
  fn new(line: u32, text: &'a str) -> Self {
    let body = text.split('#').next().unwrap_or("");

    Self {
      line,
      words: body.split_whitespace().collect(),
      taken: 0,
    }
  }

  /// Whether the line held nothing but whitespace and a comment.
  fn is_blank(&self) -> bool {
    self.words.is_empty()
  }

  /// How many tokens are left.
  fn remaining(&self) -> usize {
    self.words.len() - self.taken
  }

  /// An error against this line.
  fn error(&self, message: impl Into<String>) -> ParseError {
    ParseError::new(self.line, message)
  }

  /// The next token.
  fn word(&mut self, what: &str) -> Result<&'a str, ParseError> {
    let word = self
      .words
      .get(self.taken)
      .ok_or_else(|| self.error(format!("expected {what}, the line ended")))?;

    self.taken += 1;

    Ok(word)
  }

  /// The next token as a number.
  fn number<Value>(&mut self, what: &str) -> Result<Value, ParseError>
  where
    Value: std::str::FromStr,
  {
    let line = self.line;
    let word = self.word(what)?;

    word
      .parse::<Value>()
      .map_err(|_| ParseError::new(line, format!("`{word}` is not {what}")))
  }

  /// The next token as `0` or `1`.
  fn flag(&mut self, what: &str) -> Result<bool, ParseError> {
    match self.word(what)? {
      "0" => Ok(false),
      "1" => Ok(true),
      other => {
        Err(self.error(format!("{what} is `0` or `1`, found `{other}`")))
      }
    }
  }

  /// The next token as a count of the records that follow on this line.
  ///
  /// `tokens_each` is the smallest number of tokens one of those records
  /// takes, so the count is bounded by the tokens actually left: a wrong
  /// count in a hand edited file is an error and never an allocation.
  fn count(
    &mut self,
    what: &str,
    tokens_each: usize,
  ) -> Result<usize, ParseError> {
    let count: usize = self.number(what)?;

    if count.saturating_mul(tokens_each) > self.remaining() {
      return Err(self.error(format!(
        "{what} is {count} but only {} tokens follow",
        self.remaining()
      )));
    }

    Ok(count)
  }

  /// The next token as an index into the settings or sizes blocks.
  fn block(&mut self, what: &str) -> Result<usize, ParseError> {
    let block: usize = self.number(what)?;

    if block > MAX_BLOCK {
      return Err(self.error(format!("{what} {block} is out of range")));
    }

    Ok(block)
  }

  /// `<x> <y>`.
  fn vec2(&mut self, what: &str) -> Result<Vec2, ParseError> {
    Ok(Vec2::new(self.number(what)?, self.number(what)?))
  }

  /// `<ax> <ay> <bx> <by> <parent-index>`.
  fn seg(&mut self) -> Result<Seg, ParseError> {
    let a = self.vec2("a segment end")?;
    let b = self.vec2("a segment end")?;
    let index = self.number("a parent shape index")?;

    Ok(Seg::with_index(a, b, index))
  }

  /// `-`, or a host id.
  fn optional_host(&mut self) -> Result<Option<HostId>, ParseError> {
    let line = self.line;
    let word = self.word("a host id or `-`")?;

    if word == "-" {
      return Ok(None);
    }

    word
      .parse::<u64>()
      .map(HostId)
      .map(Some)
      .map_err(|_| ParseError::new(line, format!("`{word}` is not a host id")))
  }

  /// `-`, or a net id.
  fn optional_net(&mut self) -> Result<Option<NetId>, ParseError> {
    let line = self.line;
    let word = self.word("a net id or `-`")?;

    if word == "-" {
      return Ok(None);
    }

    word
      .parse::<u32>()
      .map(NetId)
      .map(Some)
      .map_err(|_| ParseError::new(line, format!("`{word}` is not a net id")))
  }

  /// `<layer-start> <layer-end>`.
  fn layers(&mut self) -> Result<LayerRange, ParseError> {
    let start = self.number("a layer index")?;
    let end = self.number("a layer index")?;

    Ok(LayerRange::new(start, end))
  }

  /// One via type word.
  fn via_type(&mut self) -> Result<ViaType, ParseError> {
    match self.word("a via type")? {
      "not-defined" => Ok(ViaType::NotDefined),
      "micro" => Ok(ViaType::MicroVia),
      "blind" => Ok(ViaType::Blind),
      "buried" => Ok(ViaType::Buried),
      "through" => Ok(ViaType::Through),
      other => Err(self.error(format!("`{other}` is not a via type"))),
    }
  }

  /// `chain <closed> <width> <count> <x> <y>...`.
  fn chain(&mut self) -> Result<LineChain, ParseError> {
    match self.word("a line chain")? {
      "chain" => {}
      other => {
        return Err(self.error(format!("expected `chain`, found `{other}`")));
      }
    }

    let closed = self.flag("a closed flag")?;
    let width: i32 = self.number("a chain width")?;
    let count = self.count("a point count", 2)?;
    let mut points = Vec::with_capacity(count);

    for _ in 0..count {
      points.push(self.vec2("a chain point")?);
    }

    let mut chain = LineChain::from_points(points, closed);

    chain.set_width(width);

    Ok(chain)
  }

  /// One shape, in the prefix form [`shape_text`] writes.
  fn shape(&mut self) -> Result<Shape, ParseError> {
    match self.word("a shape")? {
      "circle" => Ok(Shape::circle(
        self.vec2("a circle centre")?,
        self.number("a radius")?,
      )),
      "rect" => {
        let origin = self.vec2("a rectangle corner")?;
        let size = self.vec2("a rectangle size")?;

        Ok(Shape::Rect {
          origin,
          size,
          radius: self.number("a corner radius")?,
        })
      }
      "segment" => {
        let seg = self.seg()?;

        Ok(Shape::segment(seg, self.number("a width")?))
      }
      "simple" => Ok(Shape::Simple(SimplePolygon::new(self.chain()?))),
      "line-chain" => Ok(Shape::line_chain(self.chain()?)),
      "compound" => {
        let count = self.count("a shape count", 2)?;
        let mut shapes = Vec::with_capacity(count);

        for _ in 0..count {
          shapes.push(self.shape()?);
        }

        Ok(Shape::Compound(shapes))
      }
      other => Err(self.error(format!("`{other}` is not a shape"))),
    }
  }

  /// `flash-default`, or `flash <count> <layer>...`.
  fn flashed_layers(&mut self) -> Result<Option<LayerMask>, ParseError> {
    match self.word("a flashing mask")? {
      "flash-default" => Ok(None),
      "flash" => {
        let count = self.count("a layer count", 1)?;
        let mut mask = LayerMask::NONE;

        for _ in 0..count {
          mask = mask.with(self.number("a layer index")?);
        }

        Ok(Some(mask))
      }
      other => Err(self.error(format!(
        "expected `flash-default` or `flash`, found `{other}`"
      ))),
    }
  }

  /// `no-drill`, or `drill <shape>`.
  fn drill(&mut self) -> Result<Option<Shape>, ParseError> {
    match self.word("a drilled hole")? {
      "no-drill" => Ok(None),
      "drill" => Ok(Some(self.shape()?)),
      other => Err(
        self.error(format!("expected `no-drill` or `drill`, found `{other}`")),
      ),
    }
  }

  /// One snapshot geometry.
  fn world_geometry(&mut self) -> Result<WorldGeometry, ParseError> {
    match self.word("an item geometry")? {
      "segment" => {
        let seg = self.seg()?;

        Ok(WorldGeometry::Segment {
          seg,
          width: self.number("a track width")?,
        })
      }
      "via" => {
        let pos = self.vec2("a via centre")?;
        let diameter = self.number("a via diameter")?;
        let drill = self.number("a drill diameter")?;
        let via_type = self.via_type()?;

        Ok(WorldGeometry::Via {
          pos,
          diameter,
          drill,
          via_type,
          is_free: self.flag("a free via flag")?,
        })
      }
      "solid" => {
        let shape = self.shape()?;
        let pos = self.vec2("a solid position")?;
        let offset = self.vec2("a copper offset")?;
        let orientation_degrees = self.number("an orientation")?;
        let count = self.count("an anchor count", 2)?;
        let mut anchors = Vec::with_capacity(count);

        for _ in 0..count {
          anchors.push(self.vec2("an anchor")?);
        }

        Ok(WorldGeometry::Solid {
          shape,
          pos,
          offset,
          orientation_degrees,
          anchors,
        })
      }
      "hole" => Ok(WorldGeometry::Hole {
        shape: self.shape()?,
      }),
      other => Err(self.error(format!("`{other}` is not an item geometry"))),
    }
  }

  /// One committed item.
  fn new_item(&mut self) -> Result<NewItem, ParseError> {
    let geometry = match self.word("a committed geometry")? {
      "segment" => {
        let seg = self.seg()?;

        NewGeometry::Segment {
          seg,
          width: self.number("a track width")?,
        }
      }
      "via" => {
        let pos = self.vec2("a via centre")?;
        let diameter = self.number("a via diameter")?;
        let drill = self.number("a drill diameter")?;

        NewGeometry::Via {
          pos,
          diameter,
          drill,
          via_type: self.via_type()?,
        }
      }
      other => {
        return Err(
          self.error(format!("`{other}` is not a committed geometry")),
        );
      }
    };
    let net = self.optional_net()?;
    let layers = self.layers()?;

    Ok(NewItem {
      geometry,
      net,
      layers,
      source: self.optional_host()?,
    })
  }

  /// Refuse a line that has tokens left over.
  fn end(&self) -> Result<(), ParseError> {
    if self.remaining() == 0 {
      return Ok(());
    }

    Err(self.error(format!(
      "{} unexpected token(s) after the record",
      self.remaining()
    )))
  }
}

/// The largest settings or sizes block index a file may name.
///
/// A bound rather than an unbounded [`Vec`] growth, so that a mistyped
/// index is an error and never an allocation.
const MAX_BLOCK: usize = 4095;

/// Which of the two block tables an event referred to.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum BlockKind {
  /// A [`Sizes`] block, named by `event set-sizes`.
  Sizes,
  /// A [`RoutingSettings`] block, named by `event set-settings`.
  Settings,
}

/// An event whose payload is still a block reference.
///
/// Blocks may be written after the events that name them, so the events
/// are parsed with a placeholder payload and patched once the whole file
/// has been read.
struct PendingBlock {
  /// Which event holds the placeholder.
  event: usize,
  /// Which table the block is in.
  kind: BlockKind,
  /// The block index the event named.
  block: usize,
  /// The line the event is on, for the error when the block is missing.
  line: u32,
}

/// One parsed `event` record.
enum ParsedEvent {
  /// An event with its whole payload.
  Ready(SessionEvent),
  /// `set-sizes` or `set-settings`, with the block it named.
  Block(BlockKind, usize),
}

/// The block at an index, created from its defaults when it is new.
fn block_at<Value>(blocks: &mut Vec<Option<Value>>, index: usize) -> &mut Value
where
  Value: Clone + Default,
{
  if blocks.len() <= index {
    blocks.resize(index + 1, None);
  }

  blocks[index].get_or_insert_with(Value::default)
}

/// Apply one `settings <block> <key> <value>` record.
fn parse_settings_key(
  tokens: &mut Tokens<'_>,
  settings: &mut RoutingSettings,
) -> Result<(), ParseError> {
  match tokens.word("a settings key")? {
    "mode" => {
      settings.mode = match tokens.word("a routing mode")? {
        "mark-obstacles" => RouterMode::MarkObstacles,
        "shove" => RouterMode::Shove,
        "walkaround" => RouterMode::Walkaround,
        other => {
          return Err(tokens.error(format!("`{other}` is not a mode")));
        }
      };
    }
    "effort" => {
      settings.optimizer_effort = match tokens.word("an optimizer effort")? {
        "low" => OptimizerEffort::Low,
        "medium" => OptimizerEffort::Medium,
        "full" => OptimizerEffort::Full,
        other => {
          return Err(tokens.error(format!("`{other}` is not an effort")));
        }
      };
    }
    "corner-mode" => {
      settings.corner_mode = match tokens.word("a corner mode")? {
        "mitered-45" => CornerMode::Mitered45,
        "mitered-90" => CornerMode::Mitered90,
        other => {
          return Err(tokens.error(format!("`{other}` is not a corner mode")));
        }
      };
    }
    "shove-vias" => settings.shove_vias = tokens.flag("a setting")?,
    "remove-loops" => settings.remove_loops = tokens.flag("a setting")?,
    "smart-pads" => settings.smart_pads = tokens.flag("a setting")?,
    "follow-mouse" => settings.follow_mouse = tokens.flag("a setting")?,
    "start-diagonal" => settings.start_diagonal = tokens.flag("a setting")?,
    "jump-over-obstacles" => {
      settings.jump_over_obstacles = tokens.flag("a setting")?;
    }
    "smooth-dragged-segments" => {
      settings.smooth_dragged_segments = tokens.flag("a setting")?;
    }
    "allow-drc-violations" => {
      settings.allow_drc_violations = tokens.flag("a setting")?;
    }
    "free-angle-mode" => settings.free_angle_mode = tokens.flag("a setting")?,
    "optimize-entire-dragged-track" => {
      settings.optimize_entire_dragged_track = tokens.flag("a setting")?;
    }
    "auto-posture" => settings.auto_posture = tokens.flag("a setting")?,
    "fix-all-segments" => {
      settings.fix_all_segments = tokens.flag("a setting")?;
    }
    "restrict-angles" => settings.restrict_angles = tokens.flag("a setting")?,
    "walkaround-iteration-limit" => {
      settings.walkaround_iteration_limit = tokens.number("a limit")?;
    }
    "shove-iteration-limit" => {
      settings.shove_iteration_limit = tokens.number("a limit")?;
    }
    "shove-time-limit-ms" => {
      settings.shove_time_limit_ms = tokens.number("a limit")?;
    }
    "via-force-prop-iteration-limit" => {
      settings.via_force_prop_iteration_limit = tokens.number("a limit")?;
    }
    "walkaround-hug-length-threshold" => {
      settings.walkaround_hug_length_threshold =
        tokens.number("a threshold")?;
    }
    other => {
      return Err(tokens.error(format!("`{other}` is not a settings key")));
    }
  }

  Ok(())
}

/// Apply one `sizes <block> <key> <value>` record.
fn parse_sizes_key(
  tokens: &mut Tokens<'_>,
  sizes: &mut Sizes,
) -> Result<(), ParseError> {
  match tokens.word("a sizes key")? {
    "clearance" => sizes.clearance = tokens.number("a length")?,
    "min-clearance" => sizes.min_clearance = tokens.number("a length")?,
    "track-width" => sizes.track_width = tokens.number("a length")?,
    "board-min-track-width" => {
      sizes.board_min_track_width = tokens.number("a length")?;
    }
    "via-diameter" => sizes.via_diameter = tokens.number("a length")?,
    "via-drill" => sizes.via_drill = tokens.number("a length")?,
    "diff-pair-width" => sizes.diff_pair_width = tokens.number("a length")?,
    "diff-pair-gap" => sizes.diff_pair_gap = tokens.number("a length")?,
    "diff-pair-via-gap" => {
      sizes.diff_pair_via_gap = tokens.number("a length")?;
    }
    "hole-to-hole" => sizes.hole_to_hole = tokens.number("a length")?,
    "diff-pair-hole-to-hole" => {
      sizes.diff_pair_hole_to_hole = tokens.number("a length")?;
    }
    "diff-pair-copper-to-hole" => {
      sizes.diff_pair_copper_to_hole = tokens.number("a length")?;
    }
    "track-width-is-explicit" => {
      sizes.track_width_is_explicit = tokens.flag("a setting")?;
    }
    "diff-pair-via-gap-same-as-trace-gap" => {
      sizes.diff_pair_via_gap_same_as_trace_gap = tokens.flag("a setting")?;
    }
    "via-type" => sizes.via_type = tokens.via_type()?,
    "layer-pair" => {
      // Inserted rather than added, because
      // [`Sizes::add_layer_pair`] normalises and writes the reverse
      // direction too (`pcbnew/router/pns_sizes_settings.cpp:36`) and a
      // read back has to reproduce the map it was written from, whatever
      // a host put in it.
      let first = tokens.number("a layer index")?;
      let second = tokens.number("a layer index")?;

      sizes.layer_pairs.insert(first, second);
    }
    other => {
      return Err(tokens.error(format!("`{other}` is not a sizes key")));
    }
  }

  Ok(())
}

/// Read one `event` record, leaving a block reference unresolved.
fn parse_event(tokens: &mut Tokens<'_>) -> Result<ParsedEvent, ParseError> {
  let event = match tokens.word("an event name")? {
    "start-routing" => {
      let at = tokens.vec2("a point")?;
      let start = tokens.optional_host()?;

      SessionEvent::StartRouting {
        at,
        start,
        layer: tokens.number("a layer index")?,
      }
    }
    "move-to" => {
      let at = tokens.vec2("a point")?;

      SessionEvent::MoveTo {
        at,
        end: tokens.optional_host()?,
      }
    }
    "fix-route" => {
      let at = tokens.vec2("a point")?;
      let end = tokens.optional_host()?;

      SessionEvent::FixRoute {
        at,
        end,
        force_finish: tokens.flag("a force finish flag")?,
      }
    }
    "finish" => SessionEvent::Finish,
    "continue-from-end" => SessionEvent::ContinueFromEnd,
    "undo-last-segment" => SessionEvent::UndoLastSegment,
    "switch-layer" => SessionEvent::SwitchLayer {
      layer: tokens.number("a layer index")?,
    },
    "toggle-via-placement" => SessionEvent::ToggleViaPlacement,
    "flip-posture" => SessionEvent::FlipPosture,
    "toggle-corner-mode" => SessionEvent::ToggleCornerMode,
    "set-ortho-mode" => SessionEvent::SetOrthoMode {
      ortho: tokens.flag("an ortho flag")?,
    },
    "set-sizes" => {
      let block = tokens.block("a sizes block")?;

      return Ok(ParsedEvent::Block(BlockKind::Sizes, block));
    }
    "set-settings" => {
      let block = tokens.block("a settings block")?;

      return Ok(ParsedEvent::Block(BlockKind::Settings, block));
    }
    "stop-routing" => SessionEvent::StopRouting,
    "abort-routing" => SessionEvent::AbortRouting,
    "assign-host-ids" => {
      let count = tokens.count("an id count", 2)?;
      let mut ids = Vec::with_capacity(count);

      for _ in 0..count {
        let index = tokens.number("an added item index")?;
        let host: u64 = tokens.number("a host id")?;

        ids.push((index, HostId(host)));
      }

      SessionEvent::AssignHostIds { ids }
    }
    other => {
      return Err(tokens.error(format!("`{other}` is not an event")));
    }
  };

  Ok(ParsedEvent::Ready(event))
}

/// Read one `item` record.
fn parse_item(tokens: &mut Tokens<'_>) -> Result<WorldItem, ParseError> {
  let id: u64 = tokens.number("a host id")?;
  let net = tokens.optional_net()?;
  let layers = tokens.layers()?;
  let flags = WorldItemFlags {
    locked: tokens.flag("a locked flag")?,
    routable: tokens.flag("a routable flag")?,
    free_pad: tokens.flag("a free pad flag")?,
    compound_primitive: tokens.flag("a compound flag")?,
  };
  let flashed_layers = tokens.flashed_layers()?;
  let hole = tokens.drill()?;

  Ok(WorldItem {
    id: HostId(id),
    net,
    layers,
    geometry: tokens.world_geometry()?,
    hole,
    flags,
    flashed_layers,
  })
}

/// The body of [`SessionRecording::from_text`].
fn parse_recording(text: &str) -> Result<SessionRecording, ParseError> {
  let mut header_seen = false;
  let mut snapshot_header: Option<(u8, i32)> = None;
  let mut items: Vec<WorldItem> = Vec::new();
  let mut exclusions: Vec<Shape> = Vec::new();
  let mut settings_blocks: Vec<Option<RoutingSettings>> = Vec::new();
  let mut sizes_blocks: Vec<Option<Sizes>> = Vec::new();
  let mut events: Vec<SessionEvent> = Vec::new();
  let mut pending: Vec<PendingBlock> = Vec::new();
  let mut results: Vec<CommitDiff> = Vec::new();
  let mut last_line = 0;

  for (index, record) in text.lines().enumerate() {
    let number = u32::try_from(index).unwrap_or(u32::MAX).saturating_add(1);
    let mut tokens = Tokens::new(number, record);

    last_line = number;

    if tokens.is_blank() {
      continue;
    }

    let keyword = tokens.word("a record keyword")?;

    if !header_seen {
      if keyword != "pnsrouter-session" {
        return Err(
          tokens
            .error("the first record must be `pnsrouter-session <version>`"),
        );
      }

      let version: u32 = tokens.number("a format version")?;

      if version != FORMAT_VERSION {
        return Err(tokens.error(format!(
          "format {version} is not the {FORMAT_VERSION} this reader knows"
        )));
      }

      header_seen = true;
      tokens.end()?;

      continue;
    }

    match keyword {
      "pnsrouter-session" => {
        return Err(tokens.error("a second header record"));
      }
      "snapshot" => {
        if snapshot_header.is_some() {
          return Err(tokens.error("a second `snapshot` record"));
        }

        let count = tokens.number("a copper layer count")?;

        snapshot_header = Some((count, tokens.number("a clearance")?));
      }
      "exclusion" => exclusions.push(tokens.shape()?),
      "item" => items.push(parse_item(&mut tokens)?),
      "settings" => {
        let block = tokens.block("a settings block")?;

        parse_settings_key(&mut tokens, block_at(&mut settings_blocks, block))?;
      }
      "sizes" => {
        let block = tokens.block("a sizes block")?;

        parse_sizes_key(&mut tokens, block_at(&mut sizes_blocks, block))?;
      }
      "event" => match parse_event(&mut tokens)? {
        ParsedEvent::Ready(event) => events.push(event),
        ParsedEvent::Block(kind, block) => {
          pending.push(PendingBlock {
            event: events.len(),
            kind,
            block,
            line: number,
          });
          events.push(match kind {
            BlockKind::Sizes => SessionEvent::SetSizes {
              sizes: Sizes::default(),
            },
            BlockKind::Settings => SessionEvent::SetSettings {
              settings: RoutingSettings::default(),
            },
          });
        }
      },
      "commit" => results.push(CommitDiff::default()),
      "removed" => {
        let host: u64 = tokens.number("a host id")?;

        commit_at(&mut results, &tokens)?.removed.push(HostId(host));
      }
      "added" => {
        let item = tokens.new_item()?;

        commit_at(&mut results, &tokens)?.added.push(item);
      }
      "updated" => {
        let host: u64 = tokens.number("a host id")?;
        let item = tokens.new_item()?;

        commit_at(&mut results, &tokens)?
          .updated
          .push((HostId(host), item));
      }
      other => {
        return Err(tokens.error(format!("`{other}` is not a record keyword")));
      }
    }

    tokens.end()?;
  }

  if !header_seen {
    return Err(ParseError::new(last_line, "the file holds no records"));
  }

  for entry in pending {
    resolve_block(&mut events, &entry, &settings_blocks, &sizes_blocks)?;
  }

  let (copper_layer_count, max_clearance) = snapshot_header
    .ok_or_else(|| ParseError::new(last_line, "no `snapshot` record"))?;

  Ok(SessionRecording {
    snapshot: WorldSnapshot {
      copper_layer_count,
      max_clearance,
      items,
      edge_exclusions: exclusions,
    },
    settings: settings_blocks
      .first()
      .and_then(Option::as_ref)
      .copied()
      .unwrap_or_default(),
    sizes: sizes_blocks
      .first()
      .and_then(Option::as_ref)
      .cloned()
      .unwrap_or_default(),
    events,
    results,
  })
}

/// The commit a `removed`, `added` or `updated` line belongs to.
fn commit_at<'a>(
  results: &'a mut [CommitDiff],
  tokens: &Tokens<'_>,
) -> Result<&'a mut CommitDiff, ParseError> {
  results
    .last_mut()
    .ok_or_else(|| tokens.error("this record needs a `commit` before it"))
}

/// Fill one event's placeholder payload in from its block.
fn resolve_block(
  events: &mut [SessionEvent],
  entry: &PendingBlock,
  settings_blocks: &[Option<RoutingSettings>],
  sizes_blocks: &[Option<Sizes>],
) -> Result<(), ParseError> {
  match entry.kind {
    BlockKind::Sizes => {
      let block = sizes_blocks
        .get(entry.block)
        .and_then(Option::as_ref)
        .ok_or_else(|| {
          ParseError::new(
            entry.line,
            format!("no `sizes {}` block is defined", entry.block),
          )
        })?;

      if let Some(SessionEvent::SetSizes { sizes }) =
        events.get_mut(entry.event)
      {
        *sizes = block.clone();
      }
    }
    BlockKind::Settings => {
      let block = settings_blocks
        .get(entry.block)
        .and_then(Option::as_ref)
        .ok_or_else(|| {
          ParseError::new(
            entry.line,
            format!("no `settings {}` block is defined", entry.block),
          )
        })?;

      if let Some(SessionEvent::SetSettings { settings }) =
        events.get_mut(entry.event)
      {
        *settings = *block;
      }
    }
  }

  Ok(())
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::item::LayerRange;

  /// A recording over an empty two layer board.
  fn empty() -> SessionRecording {
    SessionRecording::new(
      WorldSnapshot::new(2, 800_000),
      RoutingSettings::default(),
      Sizes::default(),
    )
  }

  /// One committed segment on a net.
  fn committed(net: Option<NetId>) -> NewItem {
    NewItem {
      geometry: NewGeometry::Segment {
        seg: Seg::new(Vec2::new(0, 0), Vec2::new(1, 1)),
        width: 2,
      },
      net,
      layers: LayerRange::single(0),
      source: None,
    }
  }

  #[test]
  fn an_empty_recording_survives_the_trip_through_text() {
    let recording = empty();
    let text = recording.to_text();

    assert_eq!(SessionRecording::from_text(&text), Ok(recording), "{text}");
  }

  #[test]
  fn a_block_may_be_written_after_the_event_that_names_it() {
    let text = "pnsrouter-session 1\nsnapshot 2 800000\n\
                event set-sizes 1\nsizes 1 track-width 123\n";
    let read =
      SessionRecording::from_text(text).expect("a late block is legal");

    assert!(matches!(
      read.events.first(),
      Some(SessionEvent::SetSizes { sizes }) if sizes.track_width == 123
    ));
  }

  #[test]
  fn a_count_wider_than_its_line_is_refused_before_it_allocates() {
    let error = SessionRecording::from_text(
      "pnsrouter-session 1\nsnapshot 2 800000\n\
       exclusion line-chain chain 0 0 4000000000 1 2\n",
    )
    .expect_err("a count of four billion points was accepted");

    assert_eq!(error.line, 3);
  }

  #[test]
  fn a_block_index_out_of_range_is_refused() {
    let error = SessionRecording::from_text(
      "pnsrouter-session 1\nsnapshot 2 800000\nsizes 99999 clearance 1\n",
    )
    .expect_err("an unbounded block index was accepted");

    assert_eq!(error.line, 3);
  }

  #[test]
  fn the_nets_of_a_commit_come_out_ordered_and_deduplicated() {
    let diffs = vec![
      CommitDiff {
        removed: Vec::new(),
        added: vec![committed(Some(NetId(7))), committed(None)],
        updated: vec![(HostId(1), committed(Some(NetId(2))))],
      },
      CommitDiff {
        added: vec![committed(Some(NetId(7)))],
        ..CommitDiff::default()
      },
    ];

    assert_eq!(
      committed_nets(&diffs),
      vec![None, Some(NetId(2)), Some(NetId(7))]
    );
  }

  #[test]
  fn a_trailing_token_is_refused() {
    let error = SessionRecording::from_text(
      "pnsrouter-session 1\nsnapshot 2 800000\nevent finish now\n",
    )
    .expect_err("a trailing token was accepted");

    assert_eq!(error.line, 3);
    assert!(error.message.contains("unexpected"), "{error}");
  }
}
