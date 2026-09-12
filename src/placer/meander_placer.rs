// SPDX-License-Identifier: GPL-3.0-or-later

//! Length tuning of a single track.
//!
//! Port of `PNS::MEANDER_PLACER` (`pcbnew/router/pns_meander_placer.h:45`,
//! `pcbnew/router/pns_meander_placer.cpp`) and of the half of
//! `PNS::MEANDER_PLACER_BASE` it needs. The reference note is
//! `doc/reference/kicad/08-meanders.md` sections 4 and 5, and the value
//! layer underneath it is [`crate::meander`].
//!
//! # What a tuning session is
//!
//! Click a track, and the placer branches the world, assembles the whole
//! run of copper the track belongs to ([`topology::assemble_tuning_path`])
//! and measures it. Every move then cuts the assembled line into three at
//! the click point and the cursor, fills the middle with meanders, shrinks
//! them until the total lands on the target, and glues the three parts
//! back together. The fix writes that shape into the branch and the
//! session commits.
//!
//! There is **no walkaround, no shove, no optimizer and no via** anywhere
//! in this placer, and none of [`crate::settings::RoutingSettings`] is
//! read by any of it. The only rule query a tuning session makes is for
//! the clearance (`pcbnew/router/pns_meander_placer_base.cpp:122`).
//!
//! # What it does not implement
//!
//! `UnfixRoute`, `ToggleVia`, `IsPlacingVia`, `SetLayer`, `FlipPosture`,
//! `UpdateSizes`, `SetOrthoMode` and `GetModifiedNets` are all left at
//! `PLACEMENT_ALGO`'s defaults by all three meander placers (note 08
//! section 5.8). So during a tuning session backspace does nothing, the
//! via key does nothing, the layer keys do nothing, posture does nothing,
//! and the sizes the router imports at `StartRouting` are discarded.
//!
//! # Deviations, each documented at its line
//!
//! - The clearance is resolved **once per session** into a
//!   [`MeanderContext`], where KiCad resolves it inside
//!   `MEANDER_SHAPE::spacing()` and therefore several times per candidate
//!   amplitude per meander, rebuilding the placer's whole trace each time
//!   (erratum E11). The results are identical: a [`RuleResolver`] belongs
//!   to the [`crate::router::Router`] for the session's whole life and
//!   nothing in a move calls back into the host.
//! - `doMove`'s early "too long" test reads its own `aTargetMax` argument
//!   rather than `m_settings.m_targetLength.Max()` (erratum E9). The two
//!   are the same value for this placer; they differ for the skew placer.
//! - Erratum E10's `width + spacing` self intersection clearance and
//!   erratum E18's duplicated corner shapes are transcribed as they stand.
//! - `FixRoute` leaves the tuned trace in the branch and the facade
//!   commits it once, where KiCad's `CommitPlacement` commits through the
//!   router from inside the placer; see [`MeanderPlacer::fix_route`].

use crate::algo_base::AlgoContext;
use crate::collide::CollisionSearchOptions;
use crate::geometry::arc::ShapeArc;
use crate::geometry::line_chain::LineChain;
use crate::geometry::vec2::Vec2;
use crate::item::{Item, ItemBody, ItemId, Kind, NetId};
use crate::line::Line;
use crate::meander::{
  LengthTarget, MeanderContext, MeanderSettings, MeanderShape, MeanderSide,
  MeanderType, MeanderedLine, TuningStatus, amplitude_step, clearance,
  spacing_step, tune_line_length,
};
use crate::node::{NodeId, World};
use crate::rules::{ItemRef, RuleResolver};
use crate::topology::{self, PathItem};

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

/// The uid of the throwaway item the clearance query is asked about.
///
/// The same convention as `src/shove.rs` and
/// [`crate::placer::diff_pair_placer`]: a [`Line`] has no [`Item`] of its
/// own, so [`Line::rule_item`] builds one per query and it never enters
/// the arena.
const PROBE_UID: u64 = u64::MAX;

// ---------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------

/// Why a tuning session could not start.
///
/// KiCad has one message for the first two,
/// "Please select a track whose length you want to tune."
/// (`pcbnew/router/pns_meander_placer.cpp:73`), and no answer at all for
/// the third; see [`TuningError::NoTuningPath`].
/// [`crate::router::StartError`] carries them on to a host.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum TuningError {
  /// The host named no object to tune.
  ///
  /// The `!aStartItem` half of `pcbnew/router/pns_meander_placer.cpp:71`.
  /// Unlike a single track placement, a tuning session cannot start in
  /// the middle of nowhere: there is nothing to lengthen.
  NeedsStartItem,

  /// The object under the cursor is not a track.
  ///
  /// The `!OfKind( SEGMENT_T | ARC_T )` half of `:71`. A pad, a via or a
  /// hole is refused.
  NotATrack(ItemId),

  /// The topology walk found no copper to measure.
  ///
  /// KiCad has no such refusal: `AssembleTuningPath` can answer an empty
  /// `ITEM_SET` (`pcbnew/router/pns_topology.cpp:844`) and `Start` goes on
  /// regardless, leaving `origPathLength()` at zero so that every target
  /// reads as too short. The case cannot arise from a segment start there
  /// or here, since the seed line is always in the answer; it is a
  /// refusal rather than a silent zero so that a host that reaches it
  /// learns why instead of seeing a tuner that will not tune.
  NoTuningPath,

  /// The track is not half of a differential pair.
  ///
  /// "Unable to find complementary differential pair net for length
  /// tuning..." (`pcbnew/router/pns_dp_meander_placer.cpp:107`), which is
  /// [`crate::topology::assemble_diff_pair`] answering [`None`]: the
  /// resolver named no coupled net, or nothing on that net runs alongside
  /// the clicked track.
  NotADiffPairForTuning,

  /// The same refusal from the skew tuner.
  ///
  /// "...for skew tuning..."
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:79`). A separate variant
  /// because KiCad words the two differently and a host shows the message
  /// the mode calls for.
  NotADiffPairForSkew,

  /// One lane of the recovered pair holds no segment.
  ///
  /// The unreported `return false` of
  /// `pcbnew/router/pns_dp_meander_placer.cpp:117` and
  /// `pns_meander_skew_placer.cpp:88`, which KiCad leaves with no failure
  /// reason at all, so its host shows an empty status bar. It cannot
  /// arise from a successful pair assembly, since both lines were
  /// assembled from real segments; it is a refusal rather than a silent
  /// false so that a host that reaches it learns why.
  PairLaneHasNoSegments,
}

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

/// Where a [`MeanderPlacer`] is in its lifecycle.
///
/// KiCad has no such enum: its placer is newed by `StartRouting` and
/// deleted by `StopRouting` (`pcbnew/router/pns_router.cpp:451`, `:983`),
/// so "not started" and "finished" are both "no placer". The facade keeps
/// its placer until the session ends, which is what
/// [`crate::placer::diff_pair_placer::DpPlacerState`] answers too.
#[derive(Clone, Debug)]
pub enum MeanderPlacerState {
  /// Nothing has been started.
  Idle,
  /// A track is being tuned.
  Tuning(Box<MeanderTuning>),
  /// The tuned shape has been fixed into the branch.
  Finished {
    /// What [`MeanderPlacer::has_placed_anything`] answers afterwards.
    placed_anything: bool,
    /// What [`MeanderPlacer::current_layer`] answers afterwards.
    layer: i32,
  },
}

impl MeanderPlacerState {
  /// The tuning state, when there is one.
  pub fn tuning(&self) -> Option<&MeanderTuning> {
    match self {
      MeanderPlacerState::Tuning(tuning) => Some(tuning),
      _ => None,
    }
  }

  /// The tuning state, for a routine that changes it.
  pub fn tuning_mut(&mut self) -> Option<&mut MeanderTuning> {
    match self {
      MeanderPlacerState::Tuning(tuning) => Some(tuning),
      _ => None,
    }
  }
}

/// Everything one tuning session carries.
///
/// The members of `MEANDER_PLACER` (`pcbnew/router/pns_meander_placer.h:117`
/// to `:141`) plus the four of `MEANDER_PLACER_BASE` this placer reads
/// (`pns_meander_placer_base.h:166`, `:177`, `:180`, `:188`). The time
/// domain half, the net chain half and the pad to die half are not here;
/// note 08 sections 4.3 and 4.4 say why.
#[derive(Clone, Debug)]
pub struct MeanderTuning {
  /// `m_initialSegment` (`:139`), the track that was clicked.
  initial_segment: ItemId,
  /// `m_currentStart` (`:117`), the snapped point the tuned stretch
  /// begins at.
  current_start: Vec2,
  /// `m_currentEnd` (`pns_meander_placer_base.h:186`), written once to
  /// the origin by `Start` (`:105`) and never again. Erratum E1,
  /// transcribed so that `CurrentEnd()` answers what KiCad's does.
  current_end: Vec2,
  /// `m_world` (`pns_meander_placer_base.h:177`), the branch the placer
  /// owns and removed the origin line from.
  world_node: NodeId,
  /// `m_currentNode` (`:119`), the scratch branch of the last move.
  current_node: Option<NodeId>,
  /// `m_originLine` (`:121`), the assembled track being tuned, unlinked
  /// because `Start` removed it from [`MeanderTuning::world_node`].
  origin_line: Line,
  /// `m_tunedPath` (`:125`), the run of copper the length is measured
  /// over. Its chains keep the **pre meander** geometry for the whole
  /// session; nothing ever writes the meanders back into it, which is
  /// what makes `origPathLength()` a constant after `Start`.
  tuned_path: Vec<PathItem>,
  /// `m_startPad_n` (`pns_meander_placer_base.h:190`), for a host that
  /// wants the terminal. Its pad to die length is not read; see
  /// [`topology::path_length`].
  start_pad: Option<ItemId>,
  /// `m_endPad_n` (`:191`).
  end_pad: Option<ItemId>,
  /// `m_currentTrace` (`:123`), the rebuilt line handed to the host.
  current_trace: Line,
  /// `m_finalShape` (`:127`), pre plus tuned plus post.
  final_shape: LineChain,
  /// `m_result` (`:129`), the meanders the last move fitted.
  result: MeanderedLine,
  /// `m_lastLength` (`:135`).
  last_length: i64,
  /// `m_lastStatus` (`:137`).
  last_status: TuningStatus,
  /// `m_currentWidth` (`pns_meander_placer_base.h:180`).
  current_width: i32,
  /// `m_baselineLength` (`:166`), the path length captured at `Start`.
  baseline_length: i64,
  /// What `MEANDER_PLACER_BASE::Clearance()`
  /// (`pns_meander_placer_base.cpp:114`) answers, resolved once; see the
  /// module documentation.
  clearance: i32,
  /// What `CurrentLayer()` reads off the clicked segment
  /// (`pcbnew/router/pns_meander_placer.cpp:437`), captured at `Start`
  /// because the enum's accessor has no world to read it from.
  layer: i32,
  /// `m_originLine.Net()`, which is `CurrentNets()`
  /// (`pcbnew/router/pns_meander_placer.h:86`).
  net: Option<NetId>,
}

// ---------------------------------------------------------------------
// The placer
// ---------------------------------------------------------------------

/// The single track length tuner.
///
/// Port of `MEANDER_PLACER`, `pcbnew/router/pns_meander_placer.h:45`.
#[derive(Clone, Debug)]
pub struct MeanderPlacer {
  /// Where the placer is in its lifecycle.
  state: MeanderPlacerState,
  /// The node every session branches from, KiCad's
  /// `Router()->GetWorld()` (`pcbnew/router/pns_meander_placer.cpp:81`).
  root_node: NodeId,
  /// `m_settings` (`pcbnew/router/pns_meander_placer_base.h:183`). It
  /// lives outside the session state because
  /// [`MeanderPlacer::amplitude_step`] and
  /// [`MeanderPlacer::spacing_step`] are the host's live adjustments and
  /// KiCad's host applies them through the placer between moves
  /// (`pcbnew/generators/pcb_tuning_pattern.cpp:2764`, `:2785`).
  settings: MeanderSettings,
  /// The node [`MeanderPlacer::fix_route`] wrote the tuned trace into.
  ///
  /// KiCad has no such member: `FixRoute` commits through the router from
  /// inside the placer (`pcbnew/router/pns_meander_placer.cpp:371`). Here
  /// the facade owns the commit, so the node it should fold waits here;
  /// see [`MeanderPlacer::fix_route`].
  fixed_node: Option<NodeId>,
}

impl MeanderPlacer {
  /// An idle tuner over one node.
  ///
  /// The constructor, `pcbnew/router/pns_meander_placer.cpp:39`, which
  /// zeroes the last length and the pad to die lengths and starts the
  /// status at `TOO_SHORT` (`:48`).
  ///
  /// # Panics
  ///
  /// When `node` is not a live node of `world`.
  #[must_use]
  pub fn new(world: &World, node: NodeId, settings: MeanderSettings) -> Self {
    assert!(
      world.node(node).is_some(),
      "a placer needs a live node to branch from"
    );

    Self {
      state: MeanderPlacerState::Idle,
      root_node: node,
      settings,
      fixed_node: None,
    }
  }

  // -----------------------------------------------------------------
  // Accessors
  // -----------------------------------------------------------------

  /// The lifecycle state.
  pub const fn state(&self) -> &MeanderPlacerState {
    &self.state
  }

  /// The node every session branches from.
  ///
  /// `Router()->GetWorld()` (`pcbnew/router/pns_meander_placer.cpp:81`).
  /// [`crate::placer::meander_skew_placer::MeanderSkewPlacer`] reads it
  /// so that it can recover the pair before this placer takes the clicked
  /// lane out of its own branch; see that type's `start`.
  pub const fn root_node(&self) -> NodeId {
    self.root_node
  }

  /// The dimensions the meanders are drawn to.
  ///
  /// Port of `MeanderSettings()`,
  /// `pcbnew/router/pns_meander_placer_base.cpp:301`.
  pub const fn meander_settings(&self) -> &MeanderSettings {
    &self.settings
  }

  /// Replace the dimensions the meanders are drawn to.
  ///
  /// Port of `UpdateSettings`,
  /// `pcbnew/router/pns_meander_placer_base.cpp:131`, which KiCad's host
  /// calls before every `Move` (`pcbnew/generators/pcb_tuning_pattern.cpp:1298`)
  /// and reads straight back afterwards (`:1319`) to recover the initial
  /// side flip. Here the flip travels out on
  /// [`crate::router::TuningInfo::settings`] instead.
  pub const fn set_settings(&mut self, settings: MeanderSettings) {
    self.settings = settings;
  }

  /// Nudge the meander amplitude by one step.
  ///
  /// Port of `AmplitudeStep`,
  /// `pcbnew/router/pns_meander_placer_base.cpp:96`. The host re-runs the
  /// move afterwards (`pcbnew/generators/pcb_tuning_pattern.cpp:2786`);
  /// nothing is re-fitted here either, so the caller moves again.
  pub fn amplitude_step(&mut self, sign: i32) {
    amplitude_step(&mut self.settings, sign);
  }

  /// Nudge the meander spacing by one step.
  ///
  /// Port of `SpacingStep`,
  /// `pcbnew/router/pns_meander_placer_base.cpp:105`. Its floor is
  /// `m_currentWidth + Clearance()`, both of which are the session's; an
  /// idle placer has neither, so the floor is zero and only the step
  /// applies.
  pub fn spacing_step(&mut self, sign: i32) {
    let (width, clearance) = self
      .state
      .tuning()
      .map_or((0, 0), |tuning| (tuning.current_width, tuning.clearance));

    spacing_step(&mut self.settings, sign, width, clearance);
  }

  /// The line the host draws.
  ///
  /// Port of `Traces()`, `pcbnew/router/pns_meander_placer.cpp:414`,
  /// which **assigns** `m_currentTrace` before answering. This is a pure
  /// query: the assignment only matters in KiCad because `Clearance()`
  /// reads `Traces().CItems().front()` on every amplitude trial (erratum
  /// E11), and the clearance is resolved once per session here.
  ///
  /// Empty before the first move, because the final shape is, and empty
  /// once the session has finished, because KiCad's placer is destroyed
  /// at that point.
  pub fn traces(&self) -> Vec<Line> {
    self.state.tuning().map_or_else(Vec::new, |tuning| {
      vec![Line::with_chain(
        &tuning.origin_line,
        tuning.final_shape.clone(),
      )]
    })
  }

  /// The run of copper the tuned length is measured over.
  ///
  /// Port of `TunedPath()`, `pcbnew/router/pns_meander_placer.cpp:420`,
  /// which the host draws as a highlight
  /// (`pcbnew/generators/pcb_tuning_pattern.cpp:2014`).
  pub fn tuned_path(&self) -> &[PathItem] {
    self
      .state
      .tuning()
      .map_or(&[][..], |tuning| tuning.tuned_path.as_slice())
  }

  /// The track that was clicked.
  ///
  /// `m_initialSegment` (`pcbnew/router/pns_meander_placer.h:139`), which
  /// KiCad reads for `CurrentLayer()` (`:437`) and for nothing else. The
  /// layer is captured at `Start` here, because the enum's accessor has
  /// no world to resolve a handle against, so this is the handle itself
  /// for a host that wants to know what it is tuning.
  pub fn initial_segment(&self) -> Option<ItemId> {
    self.state.tuning().map(|tuning| tuning.initial_segment)
  }

  /// The two terminal pads the topology walk stopped on.
  ///
  /// `m_startPad_n` and `m_endPad_n`
  /// (`pcbnew/router/pns_meander_placer_base.h:190`, `:191`), which KiCad
  /// reads a pad to die length off; see [`topology::path_length`] for why
  /// nothing here does.
  pub fn terminal_pads(&self) -> (Option<ItemId>, Option<ItemId>) {
    self
      .state
      .tuning()
      .map_or((None, None), |tuning| (tuning.start_pad, tuning.end_pad))
  }

  /// Where the tuned stretch begins.
  ///
  /// Port of `CurrentStart()`,
  /// `pcbnew/router/pns_meander_placer.cpp:425`.
  pub fn current_start(&self) -> Option<Vec2> {
    self.state.tuning().map(|tuning| tuning.current_start)
  }

  /// Where the tuned stretch ends, which is always the origin.
  ///
  /// Port of `CurrentEnd()`,
  /// `pcbnew/router/pns_meander_placer.cpp:430`, whose `m_currentEnd` is
  /// written once by `Start` and never again (erratum E1). It is not the
  /// cursor and it is not the far end of the tuned stretch.
  pub fn current_end(&self) -> Option<Vec2> {
    self.state.tuning().map(|tuning| tuning.current_end)
  }

  /// The net being tuned.
  ///
  /// Port of `CurrentNets()`,
  /// `pcbnew/router/pns_meander_placer.h:86`, a one element vector
  /// holding the origin line's net.
  pub fn current_nets(&self) -> Option<NetId> {
    self.state.tuning().and_then(|tuning| tuning.net)
  }

  /// The layer being tuned on.
  ///
  /// Port of `CurrentLayer()`,
  /// `pcbnew/router/pns_meander_placer.cpp:435`, which reads the clicked
  /// segment's first layer.
  pub const fn current_layer(&self) -> Option<i32> {
    match &self.state {
      MeanderPlacerState::Idle => None,
      MeanderPlacerState::Tuning(tuning) => Some(tuning.layer),
      MeanderPlacerState::Finished { layer, .. } => Some(*layer),
    }
  }

  /// The most recent world state.
  ///
  /// Port of `CurrentNode( bool )`,
  /// `pcbnew/router/pns_meander_placer.cpp:60`: the scratch branch when
  /// there is one, the placer's own branch otherwise. The `aLoopsRemoved`
  /// argument is ignored there, so it is not in the signature.
  pub fn current_node(&self) -> NodeId {
    match &self.state {
      MeanderPlacerState::Idle => self.root_node,
      MeanderPlacerState::Tuning(tuning) => {
        tuning.current_node.unwrap_or(tuning.world_node)
      }
      MeanderPlacerState::Finished { .. } => {
        self.fixed_node.unwrap_or(self.root_node)
      }
    }
  }

  /// The scratch branch of the last move, when there is one.
  pub fn last_node(&self) -> Option<NodeId> {
    self.state.tuning().and_then(|tuning| tuning.current_node)
  }

  /// The node a commit should fold into the board, if any.
  ///
  /// KiCad has no such accessor: `FixRoute` commits `m_currentNode`
  /// through the router and nulls it
  /// (`pcbnew/router/pns_meander_placer.cpp:371`, `:390`). Answering
  /// [`None`] until a fix has happened is what keeps a session that only
  /// moved from committing the removal of the track it was going to tune.
  pub const fn fixed_node(&self) -> Option<NodeId> {
    self.fixed_node
  }

  /// Whether anything has reached a node.
  ///
  /// Port of `HasPlacedAnything`,
  /// `pcbnew/router/pns_meander_placer.cpp:384`, which is
  /// `m_currentTrace.SegmentCount() > 0`. `m_currentTrace` is only ever
  /// written by `FixRoute` and by `Traces()`, so it answers false until
  /// one of the two has run.
  pub fn has_placed_anything(&self) -> bool {
    match &self.state {
      MeanderPlacerState::Idle => false,
      MeanderPlacerState::Tuning(tuning) => {
        tuning.current_trace.segment_count() > 0
      }
      MeanderPlacerState::Finished {
        placed_anything, ..
      } => *placed_anything,
    }
  }

  /// How the tuned line stands against its target.
  ///
  /// Port of `TuningStatus()`,
  /// `pcbnew/router/pns_meander_placer.cpp:459`.
  pub fn tuning_status(&self) -> Option<TuningStatus> {
    self.state.tuning().map(|tuning| tuning.last_status)
  }

  /// The length the last move produced.
  ///
  /// Port of `TuningLengthResult()`,
  /// `pcbnew/router/pns_meander_placer.cpp:441`: the last measured
  /// length, or the untouched path length when no move has produced one.
  /// Zero doubles as "no move yet" there, exactly as it does here.
  pub fn tuning_length_result(&self) -> Option<i64> {
    self.state.tuning().map(|tuning| {
      if tuning.last_length == 0 {
        // :446. `origPathLength()` measures `m_tunedPath`, whose chains
        // are never rewritten, so it is the baseline captured at `Start`.
        tuning.baseline_length
      } else {
        tuning.last_length
      }
    })
  }

  /// How far the tuned line has moved from where it started.
  ///
  /// Port of `TuningLengthDelta()`
  /// (`pcbnew/router/pns_meander_placer_base.h:70`), gated by
  /// `HasBaseline()` (`:68`), which without the delay half is
  /// "the baseline length is not zero".
  pub fn tuning_length_delta(&self) -> Option<i64> {
    let tuning = self.state.tuning()?;

    if tuning.baseline_length == 0 {
      return None;
    }

    Some(self.tuning_length_result()? - tuning.baseline_length)
  }

  /// The length the last move measured, before the fallback
  /// [`MeanderPlacer::tuning_length_result`] applies.
  ///
  /// `m_lastLength` (`pcbnew/router/pns_meander_placer.h:135`), raw. The
  /// skew placer reads it that way, because its own `TuningLengthResult`
  /// is a skew (`pcbnew/router/pns_meander_skew_placer.cpp:242`) and
  /// never goes through the "zero means no move yet" fallback.
  pub fn last_length(&self) -> Option<i64> {
    self.state.tuning().map(|tuning| tuning.last_length)
  }

  /// Seed the length the readout starts from.
  ///
  /// `MEANDER_SKEW_PLACER::Start` writes `m_lastLength` itself
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:152`, `:160`), where
  /// `MEANDER_PLACER::Start` leaves the constructor's zero (`:46`). That
  /// is one assignment across an inheritance boundary in KiCad and one
  /// call across a module boundary here.
  pub fn set_last_length(&mut self, length: i64) {
    if let Some(tuning) = self.state.tuning_mut() {
      tuning.last_length = length;
    }
  }

  // -----------------------------------------------------------------
  // Start
  // -----------------------------------------------------------------

  /// Begin tuning the track under a point.
  ///
  /// Port of `Start`, `pcbnew/router/pns_meander_placer.cpp:69`: snap the
  /// click onto the clicked segment, branch the world, assemble the
  /// track, walk the whole run of copper it belongs to, take the track
  /// out of the branch and measure the baseline.
  ///
  /// The track is removed from the branch (`:102`) and **nothing else on
  /// the path is**, which is what makes [`MeanderPlacer::check_fit`]
  /// collide the candidate meanders against everything on the board
  /// except the track being tuned.
  ///
  /// `initChainExtras` (`:113`) and `calculateTimeDomainTargets` (`:115`)
  /// have no counterpart: note 08 section 4.4 works out that both
  /// collapse to a constant zero without a net chain concept, and the
  /// time domain half needs a stackup model.
  ///
  /// # Errors
  ///
  /// [`TuningError`].
  pub fn start(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    start_item: Option<ItemId>,
  ) -> Result<(), TuningError> {
    // :71
    let start_item = start_item.ok_or(TuningError::NeedsStartItem)?;
    let item = world
      .item(start_item)
      .ok_or(TuningError::NotATrack(start_item))?;

    if !item.of_kind(Kind::SEGMENT | Kind::ARC) {
      return Err(TuningError::NotATrack(start_item));
    }

    // :79
    let current_start = snapped_start_point(item, at);
    let layer = item.layers().start();

    // :81
    let world_node = world.branch(self.root_node);

    // :82. `AssembleLine( m_initialSegment )` takes every default, so
    // locked joints do not stop it and locked segments are not followed.
    let mut origin_line =
      world.assemble_line(world_node, start_item, None, false, false, true);

    // :85
    let path = topology::assemble_tuning_path(
      world,
      world_node,
      context.resolver,
      start_item,
    );

    if path.items.is_empty() {
      return Err(TuningError::NoTuningPath);
    }

    // :102. Only the tuned track leaves the branch.
    world.remove_line(world_node, &mut origin_line);

    // :104
    let current_width = origin_line.width();
    let net = origin_line.net();

    // :110. `origPathLength()` (`:121`) minus the pad to die and net
    // chain terms, neither of which this crate has.
    let baseline_length = topology::path_length(&path.items);

    // `MEANDER_PLACER_BASE::Clearance()`
    // (`pcbnew/router/pns_meander_placer_base.cpp:114`), resolved once;
    // see the module documentation. The item is the track being tuned,
    // which is what KiCad's `Traces().CItems().front()` amounts to once
    // the trace has been rebuilt from it.
    let probe = origin_line.rule_item(world, PROBE_UID);
    let clearance = clearance(
      context.resolver,
      ItemRef::unstored(&probe),
      layer,
      current_width,
    );

    self.fixed_node = None;
    self.state = MeanderPlacerState::Tuning(Box::new(MeanderTuning {
      initial_segment: start_item,
      current_start,
      // :105
      current_end: Vec2::new(0, 0),
      world_node,
      // :78
      current_node: None,
      origin_line,
      tuned_path: path.items,
      start_pad: path.start_pad,
      end_pad: path.end_pad,
      current_trace: Line::new(),
      final_shape: LineChain::new(),
      result: MeanderedLine::new(current_width, false),
      // :46, :48
      last_length: 0,
      last_status: TuningStatus::TooShort,
      current_width,
      baseline_length,
      clearance,
      layer,
      net,
    }));

    Ok(())
  }

  // -----------------------------------------------------------------
  // Move
  // -----------------------------------------------------------------

  /// Re-meander the stretch between the click and a point.
  ///
  /// Port of `Move`, `pcbnew/router/pns_meander_placer.cpp:188`.
  /// Everything before its call to `doMove` is net chain arithmetic that
  /// collapses to nothing here (note 08 section 4.4), so this is the
  /// target lookup and the forward.
  ///
  /// # An unconstrained target
  ///
  /// [`MeanderSettings::target_length`] of [`None`] is KiCad's
  /// `LENGTH_UNCONSTRAINED` said without a sentinel, and it is treated as
  /// that sentinel: optimum one kilometre, minimum zero, maximum one
  /// kilometre (`pcbnew/router/pns_meander.cpp:71` to `:74`). So an
  /// unconstrained session meanders as hard as the stretch allows and
  /// reports [`TuningStatus::Tuned`], which is what KiCad does. Note 08
  /// section 11.3 offers short circuiting to
  /// [`TuningStatus::TooShort`] instead; reproducing KiCad was chosen,
  /// and `a_session_with_no_target_meanders_as_hard_as_it_can` pins it.
  ///
  /// The end item is ignored, as it is in KiCad (`:188`): there is
  /// nothing to snap a tuned stretch onto.
  pub fn move_to(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    _end_item: Option<ItemId>,
  ) -> bool {
    let target = self
      .settings
      .target_length()
      .unwrap_or_else(LengthTarget::unconstrained);

    // :223
    self.do_move(world, context, at, target.opt, target.min, target.max)
  }

  /// Cut the line into three, meander the middle, glue it back together.
  ///
  /// Port of `doMove`, `pcbnew/router/pns_meander_placer.cpp:228`, the
  /// one routine the milestone is about.
  ///
  /// It is public because
  /// [`crate::placer::meander_skew_placer::MeanderSkewPlacer`] reuses it
  /// whole and changes only the three targets it is given
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:234`); KiCad spells that
  /// reuse as inheritance, and this is what `protected` amounts to across
  /// two modules. Nothing else should call it: the target a single track
  /// session tunes to is [`MeanderPlacer::move_to`]'s job to look up.
  ///
  /// There is no incremental state:
  /// the tuned stretch is re-meandered from scratch on every move
  /// (`:243`).
  ///
  /// The arc branch at `:249`, which passes an existing arc through as an
  /// `MT_CORNER` shape carrying it, has no counterpart while
  /// [`LineChain`] cannot hold an arc.
  ///
  /// `m_lastLength` is built by subtraction and then addition (`:288`,
  /// `:323`): the whole path length minus the straight stretch about to
  /// be replaced, then plus the meandered stretch. That works because the
  /// path being measured still holds the pre meander geometry: nothing
  /// ever writes the meanders back into `m_tunedPath`, which is what
  /// makes `origPathLength()` a constant after `Start`.
  pub fn do_move(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    target_length: i64,
    target_min: i64,
    target_max: i64,
  ) -> bool {
    let Some(tuning) = self.state.tuning_mut() else {
      return false;
    };

    // :231
    if tuning.current_start == at {
      return false;
    }

    // :234, :237
    if let Some(node) = tuning.current_node.take() {
      world.drop_node(node);
    }

    let current_node = world.branch(tuning.world_node);

    tuning.current_node = Some(current_node);

    // :241
    let Some((mut pre, mut tuned, mut post)) = tuning
      .origin_line
      .shape()
      .split_three_way(tuning.current_start, at)
    else {
      return false;
    };

    // The pieces the fitting loop borrows. The origin line is cloned
    // because the fit check needs it while the meander list is being
    // built; one clone per move is the price of keeping the shape
    // generator free of a back pointer into the placer (note 08 section
    // 11.4).
    let origin_line = tuning.origin_line.clone();
    let width = origin_line.width();
    let clearance = tuning.clearance;
    let path_length = topology::path_length(&tuning.tuned_path);
    let settings = self.settings;

    // :243 to :245
    let mut result = MeanderedLine::new(width, false);

    result.set_width(width);
    result.set_baseline_offset(0);

    // KiCad re-reads `m_settings.m_initialSide` at the top of **every**
    // base segment (`pns_meander_placer.cpp:265`) and `flipInitialSide`
    // writes into it from inside `MeanderSegment` (`pns_meander.cpp:282`),
    // so a flip on one base segment decides which side the next one
    // starts on. That is note 08 section 11.6's order sensitivity, and it
    // is why the side is a local the loop updates rather than a value
    // read once: the shape generator here reports the flip instead of
    // performing it, but the placer has to apply it at the same point.
    let mut initial_side = settings.initial_side();
    let mut flipped = false;
    let mut last_length = path_length;
    let mut last_status = TuningStatus::Tuned;

    {
      let node_world: &World = world;
      let resolver = context.resolver;
      let check = |shape: &MeanderShape, placed: &MeanderedLine| -> bool {
        check_fit(
          node_world,
          current_node,
          resolver,
          &origin_line,
          &settings,
          shape,
          placed,
        )
      };
      let meander_context =
        MeanderContext::new(&settings, width, clearance, &check);

      // :247. A `while` loop rather than a `for`, because the arc
      // passthrough below sets the next index itself.
      let mut index = 0;

      while index < tuned.segment_count() {
        // :249 to :260. An arc the tuned stretch already contained is
        // carried through whole, as one pass through marker, and the
        // walk resumes at the next shape.
        //
        // KiCad sets `i = tuned.NextShape( i )` and then `continue`s,
        // which runs the `for` loop's own `i++` on top of the answer, so
        // the shape after every arc is never meandered and never emitted
        // as a corner. That is erratum E29, and it is **fixed** here: the
        // walk resumes at exactly the shape `next_shape` named. Nothing
        // in KiCad's corpus can see the difference, no regression case
        // containing an arc existing (note 09 section 8.3).
        if tuned.is_arc_segment(index) {
          if let Some(arc) =
            tuned.arc_index(index).and_then(|which| tuned.arc(which))
          {
            // :252. KiCad's second lane is the default argument, a
            // default constructed `SHAPE_ARC` (`pns_meander.h:515`);
            // nothing reads it for a single track, `m_dual` being false.
            result.add_arc(&arc, &ShapeArc::default());
          }

          // :254. `NextShape` answering `None` is KiCad's `-1`, the
          // last shape, and ends the walk.
          match tuned.next_shape(index) {
            Some(next) => index = next,
            None => break,
          }

          continue;
        }

        let seg = tuned.segment(index);

        // :262 to :268
        let side = match initial_side {
          // `MEANDER_SIDE_DEFAULT`, the follow the cursor branch.
          MeanderSide::Default => seg.side(at) < 0,
          // `m_initialSide < 0`, so left is true and right is false.
          MeanderSide::Left => true,
          MeanderSide::Right => false,
        };

        // :270 to :272. Erratum E18: `MeanderSegment` adds the same two
        // corners itself (`pns_meander.cpp:263`, `:407`), so a base
        // segment ends up with four corner shapes where two would do.
        // They are harmless, because a corner shape is a one point chain
        // and `Append` drops a repeat, and they are transcribed.
        result.add_corner(seg.a, Vec2::new(0, 0));

        let outcome = result.meander_segment(&meander_context, seg, side);

        if outcome.initial_side_flipped {
          // `pns_meander.cpp:286`. `MeanderSide::Default` negates to
          // itself, so a stretch following the cursor never changes side
          // here, which is KiCad's `-0 == 0`.
          initial_side = initial_side.flipped();
          flipped = !flipped;
        }

        result.add_corner(seg.b, Vec2::new(0, 0));

        index += 1;
      }

      // :275 to :280
      let line_length = path_length;

      // :282. Erratum E9: KiCad reads `m_settings.m_targetLength.Max()`
      // here and its own `aTargetMax` argument at `:332`. `Move` passes
      // the one as the other for this placer, so the two agree; they do
      // not for the skew placer, and the argument is the one that was
      // meant.
      if line_length > target_max {
        last_status = TuningStatus::TooLong;
      } else {
        // :288
        last_length = line_length - tuned.length();

        // :298
        tune_line_length(
          &meander_context,
          &mut result,
          target_length - line_length,
        );
      }
    }

    // :311
    if last_status != TuningStatus::TooLong {
      // :313 to :321
      tuned.clear();

      for meander in result.meanders() {
        if meander.meander_type() != MeanderType::Empty {
          tuned.append_chain(meander.cline(0));
        }
      }

      // :323
      last_length += tuned.length();

      // :332 to :337
      last_status = TuningStatus::for_length(
        last_length,
        &LengthTarget::explicit(target_min, target_length, target_max),
      );
    }

    // :340 to :358. `keepEndpoints` changes only where `Simplify` runs:
    // per part when set, over the concatenation when not. The host forces
    // it true (`pcbnew/generators/pcb_tuning_pattern.cpp:1297`) so that
    // the vertices at the two seams survive and it can tell which
    // segments belong to the pattern.
    let mut final_shape = LineChain::new();

    if settings.keep_endpoints() {
      pre.simplify(0);
      tuned.simplify(0);
      post.simplify(0);
    }

    final_shape.append_chain(&pre);
    final_shape.append_chain(&tuned);
    final_shape.append_chain(&post);

    if !settings.keep_endpoints() {
      final_shape.simplify(0);
    }

    if flipped {
      // The parity of the flips carried into the placer's own settings,
      // which is what KiCad's `UpdateSettings` round trip leaves behind
      // and what the host reads back to persist
      // (`pcbnew/generators/pcb_tuning_pattern.cpp:1319`).
      self.settings.flip_initial_side();
    }

    let Some(tuning) = self.state.tuning_mut() else {
      return false;
    };

    tuning.result = result;
    tuning.final_shape = final_shape;
    tuning.last_length = last_length;
    tuning.last_status = last_status;

    true
  }

  /// Whether a candidate meander may be placed.
  ///
  /// Port of `CheckFit`, `pcbnew/router/pns_meander_placer.cpp:400`.
  /// Public because it is the fit check the shape generator is handed and
  /// because a reader looking for KiCad's member should find it under its
  /// own name; `MeanderPlacer::do_move` installs it as a closure.
  ///
  /// The collision test runs against the scratch branch, which has the
  /// tuned track removed and nothing else, so a candidate is checked
  /// against every other object on the board.
  ///
  /// The self intersection clearance is `width + spacing` (`:408`), which
  /// is not a design rule clearance at all but a heuristic, and it scales
  /// with the user's spacing setting so that raising the spacing makes
  /// the test stricter. The pair placer uses four widths instead
  /// (`pcbnew/router/pns_dp_meander_placer.cpp:620`). Erratum E10, both
  /// halves transcribed as they stand.
  pub fn check_fit(
    &self,
    world: &World,
    resolver: &dyn RuleResolver,
    shape: &MeanderShape,
    placed: &MeanderedLine,
  ) -> bool {
    let Some(tuning) = self.state.tuning() else {
      return false;
    };
    let Some(node) = tuning.current_node else {
      return false;
    };

    check_fit(
      world,
      node,
      resolver,
      &tuning.origin_line,
      &self.settings,
      shape,
      placed,
    )
  }

  // -----------------------------------------------------------------
  // Fix, commit, abort
  // -----------------------------------------------------------------

  /// Write the tuned shape into the branch and end the session.
  ///
  /// Port of `FixRoute`, `pcbnew/router/pns_meander_placer.cpp:364`,
  /// which ignores both its point and its end item and fixes whatever the
  /// last `doMove` produced. A session that never moved has no scratch
  /// branch and answers false (`:366`), which is also the null check
  /// `DP_MEANDER_PLACER::FixRoute` is missing (erratum E13).
  ///
  /// # Deviation
  ///
  /// KiCad ends with `CommitPlacement()`, which commits the scratch
  /// branch through the router (`:371`, `:390`). Here the node is
  /// remembered in [`MeanderPlacer::fixed_node`] and the facade folds it,
  /// so that the [`crate::router::CommitDiff`] can be built before the
  /// commit takes the removed items out of the arena. That is the same
  /// arrangement [`crate::placer::diff_pair_placer::DiffPairPlacer`] uses
  /// and the reason a session that only moved commits nothing.
  pub fn fix_route(
    &mut self,
    world: &mut World,
    _at: Vec2,
    _end_item: Option<ItemId>,
    _force_finish: bool,
  ) -> bool {
    let Some(tuning) = self.state.tuning_mut() else {
      return false;
    };

    // :366
    let Some(node) = tuning.current_node else {
      return false;
    };

    // :369, :370
    let mut trace =
      Line::with_chain(&tuning.origin_line, tuning.final_shape.clone());

    world.add_line(node, &mut trace, false);

    let placed_anything = trace.segment_count() > 0;
    let layer = tuning.layer;

    tuning.current_trace = trace;
    self.fixed_node = Some(node);
    self.state = MeanderPlacerState::Finished {
      placed_anything,
      layer,
    };

    true
  }

  /// Fold the fixed branch into the board.
  ///
  /// Port of `CommitPlacement`,
  /// `pcbnew/router/pns_meander_placer.cpp:390`. A session that never
  /// fixed anything commits nothing, which is what its null
  /// `m_currentNode` means.
  pub fn commit_placement(&mut self, world: &mut World) -> bool {
    if let Some(node) = self.fixed_node.take() {
      world.commit(node);
    }

    true
  }

  /// Throw the session away.
  ///
  /// Port of `AbortPlacement`,
  /// `pcbnew/router/pns_meander_placer.cpp:377`, which kills the
  /// placement world's children. It does not reset anything there,
  /// because the router deletes the placer instead
  /// (`pcbnew/router/pns_router.cpp:967`); the state is reset here, since
  /// the facade keeps its placer until the session ends. The track the
  /// session removed from its own branch comes back with the branch.
  pub fn abort_placement(&mut self, world: &mut World) {
    if let Some(tuning) = self.state.tuning() {
      world.kill_children(tuning.world_node);
    }

    self.fixed_node = None;
    self.state = MeanderPlacerState::Idle;
  }

  // -----------------------------------------------------------------
  // The commands a meander placer does not implement
  // -----------------------------------------------------------------

  /// Roll the tuning back one step. Always [`None`].
  ///
  /// `UnfixRoute` is not overridden by any of the three meander placers,
  /// so `PLACEMENT_ALGO::UnfixRoute` answers `std::nullopt`
  /// (`pcbnew/router/pns_placement_algo.h:81`) and backspace does nothing
  /// during a tuning session. Note 08 section 5.8.
  pub const fn undo_last_segment(&self) -> Option<Vec2> {
    None
  }

  /// Arm a via. Always false.
  ///
  /// `ToggleVia` is not overridden
  /// (`pcbnew/router/pns_placement_algo.h:90`); there is no via anywhere
  /// in a meander placer. Note 08 section 5.8.
  pub const fn toggle_via(&self, _enabled: bool) -> bool {
    false
  }

  /// Whether a via is pending. Always false.
  ///
  /// `IsPlacingVia` is not overridden
  /// (`pcbnew/router/pns_placement_algo.h:104`).
  pub const fn is_placing_via(&self) -> bool {
    false
  }

  /// Change layer. Always false.
  ///
  /// `SetLayer` is not overridden
  /// (`pcbnew/router/pns_placement_algo.h:114`), so the layer keys do
  /// nothing during a tuning session and the layer the router sets before
  /// `Start` is discarded.
  pub const fn set_layer(&self, _layer: i32) -> bool {
    false
  }

  /// Flip the posture. Does nothing.
  ///
  /// `FlipPosture` is not overridden
  /// (`pcbnew/router/pns_placement_algo.h:167`).
  pub const fn flip_posture(&self) {}
}

// ---------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------

/// Where a click on a track begins the tuned stretch.
///
/// Port of `HELPERS::GetSnappedStartPoint`,
/// `pcbnew/router/pns_helpers.cpp:187`: the nearest point of a segment,
/// and for an arc the nearer of its two anchors. So a click anywhere on
/// an arc begins the tuned stretch at one of its ends and never inside
/// it, which is what keeps the arc whole for `do_move`'s passthrough.
pub(crate) fn snapped_start_point(item: &Item, at: Vec2) -> Vec2 {
  // :190
  if let ItemBody::Segment(segment) = item.body() {
    return segment.seg().nearest_point_to_point(at);
  }

  // :199 to :207, the arc branch. KiCad asserts the item is an arc
  // (`:197`); anything else with two anchors answers the same way here
  // and anything with fewer answers the click.
  if item.anchor_count() < 2 {
    return at;
  }

  let first = item.anchor(0);
  let second = item.anchor(1);

  if (first - at).squared_euclidean_norm()
    <= (second - at).squared_euclidean_norm()
  {
    first
  } else {
    second
  }
}

/// The body of [`MeanderPlacer::check_fit`], as a free function so that
/// [`MeanderPlacer::do_move`] can install it as a closure while it holds
/// the meander list.
fn check_fit(
  world: &World,
  node: NodeId,
  resolver: &dyn RuleResolver,
  origin_line: &Line,
  settings: &MeanderSettings,
  shape: &MeanderShape,
  placed: &MeanderedLine,
) -> bool {
  // :402
  let line = Line::with_chain(origin_line, shape.cline(0).clone());

  // :404. `NODE::CheckColliding( const ITEM* )`
  // (`pcbnew/router/pns_node.h:342`) takes the default kind mask and a
  // limit of one, the answer being a yes or a no.
  let options = CollisionSearchOptions {
    limit_count: Some(1),
    ..CollisionSearchOptions::default()
  };

  if world
    .check_colliding_line(node, &line, resolver, &options)
    .is_some()
  {
    return false;
  }

  // :407, :408. Erratum E10; KiCad computes the sum in an `int`.
  let clearance = shape.width().saturating_add(settings.spacing());

  // :410
  placed.check_self_intersections(shape, clearance)
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::seg::Seg;
  use crate::geometry::shape::Shape;
  use crate::item::{ItemBody, LayerRange, NetId, Segment, Solid};
  use crate::meander::{LengthTarget, MeanderSettingsRequest};
  use crate::rules::FixedClearance;
  use crate::settings::RoutingSettings;

  /// The width of the fixture track.
  const WIDTH: i32 = 200_000;

  /// A world holding one straight track between two pads, and the
  /// track's handle.
  fn fixture() -> (World, ItemId) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let net = Some(NetId(1));
    let pad_at = |world: &mut World, at: Vec2| {
      let body = ItemBody::Solid(Solid::new(Shape::circle(at, 150_000), at));
      let mut item = world.make_item(body);

      item.set_layers_and_flash_all(LayerRange::single(0));
      item.set_net(net);
      world.add_solid(root, item, None);
    };

    pad_at(&mut world, Vec2::new(0, 0));
    pad_at(&mut world, Vec2::new(8_000_000, 0));

    let body = ItemBody::Segment(Segment::new(
      Seg::new(Vec2::new(0, 0), Vec2::new(8_000_000, 0)),
      WIDTH,
    ));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);

    let track = world
      .add_segment(root, item, false)
      .expect("the fixture track is neither degenerate nor redundant");

    (world, track)
  }

  /// The note's settings, aimed at a target.
  fn settings(target: i64) -> MeanderSettings {
    MeanderSettings::new(MeanderSettingsRequest {
      keep_endpoints: true,
      target_length: Some(LengthTarget::around(target)),
      ..MeanderSettingsRequest::default()
    })
    .expect("the default request is chamfered and has a positive step")
  }

  #[test]
  fn a_start_needs_a_track() {
    let (mut world, track) = fixture();
    let root = world.root();
    let rules = FixedClearance::uniform(100_000);
    let routing = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &routing);
    let mut placer = MeanderPlacer::new(&world, root, settings(10_000_000));

    assert_eq!(
      placer.start(&mut world, &context, Vec2::new(0, 0), None),
      Err(TuningError::NeedsStartItem)
    );
    assert!(
      placer
        .start(&mut world, &context, Vec2::new(1_000_000, 0), Some(track))
        .is_ok()
    );
    assert_eq!(placer.initial_segment(), Some(track));
    assert_eq!(placer.current_layer(), Some(0));
    assert_eq!(placer.current_start(), Some(Vec2::new(1_000_000, 0)));
    // `m_currentEnd` is written once to the origin and never again;
    // erratum E1.
    assert_eq!(placer.current_end(), Some(Vec2::new(0, 0)));
    assert_eq!(placer.tuning_status(), Some(TuningStatus::TooShort));
    assert_eq!(placer.tuning_length_result(), Some(8_000_000));
    assert_eq!(placer.tuning_length_delta(), Some(0));
    assert_eq!(placer.tuned_path().len(), 1);
    assert!(placer.terminal_pads().0.is_some());
    assert!(placer.terminal_pads().1.is_some());
  }

  /// `CheckFit` answers false before a move, because there is no scratch
  /// branch to collide against, and accepts the shapes the move itself
  /// fitted afterwards.
  #[test]
  fn check_fit_needs_a_move_and_accepts_what_that_move_fitted() {
    let (mut world, track) = fixture();
    let root = world.root();
    let rules = FixedClearance::uniform(100_000);
    let routing = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &routing);
    let mut placer = MeanderPlacer::new(&world, root, settings(10_000_000));

    placer
      .start(&mut world, &context, Vec2::new(1_000_000, 0), Some(track))
      .expect("the fixture track is a track");

    let empty = MeanderedLine::new(WIDTH, false);
    let probe = MeanderShape::new(WIDTH, false);

    assert!(
      !placer.check_fit(&world, &rules, &probe, &empty),
      "a session that has not moved has no node to test against"
    );

    assert!(placer.move_to(
      &mut world,
      &context,
      Vec2::new(7_000_000, 0),
      None
    ));

    let fitted: Vec<MeanderShape> = placer
      .state
      .tuning()
      .expect("the session is running")
      .result
      .meanders()
      .iter()
      .filter(|meander| meander.meander_type() != MeanderType::Corner)
      .cloned()
      .collect();

    assert!(!fitted.is_empty(), "the move fitted something");

    for shape in &fitted {
      assert!(
        placer.check_fit(&world, &rules, shape, &empty),
        "a shape the fitting loop accepted still fits against an empty line"
      );
    }
  }

  /// The two live adjustments move the settings, and the spacing floor is
  /// the tuned track's width plus its clearance.
  #[test]
  fn the_live_adjustments_read_the_sessions_width_and_clearance() {
    let (mut world, track) = fixture();
    let root = world.root();
    let rules = FixedClearance::uniform(100_000);
    let routing = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &routing);
    let mut placer = MeanderPlacer::new(&world, root, settings(10_000_000));

    placer
      .start(&mut world, &context, Vec2::new(1_000_000, 0), Some(track))
      .expect("the fixture track is a track");

    let step = placer.meander_settings().step();
    let before = placer.meander_settings().max_amplitude();

    placer.amplitude_step(1);

    assert_eq!(placer.meander_settings().max_amplitude(), before + step);

    // `FixedClearance` answers no `CT_CLEARANCE` constraint, so the
    // clearance falls back to the track width
    // (`pcbnew/router/pns_meander_placer_base.cpp:125`) and the spacing
    // floor is two widths.
    for _ in 0..100 {
      placer.spacing_step(-1);
    }

    assert_eq!(placer.meander_settings().spacing(), WIDTH + WIDTH);
  }
}
