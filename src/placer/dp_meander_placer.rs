// SPDX-License-Identifier: GPL-3.0-or-later

//! Length tuning of a differential pair, keeping the coupling.
//!
//! Port of `PNS::DP_MEANDER_PLACER` (`pcbnew/router/pns_dp_meander_placer.h:48`,
//! `pcbnew/router/pns_dp_meander_placer.cpp`). The reference note is
//! `doc/reference/kicad/08-meanders.md` section 6, and the value layer
//! underneath it is [`crate::meander`].
//!
//! It does **not** derive from `MEANDER_PLACER` in KiCad and does not wrap
//! [`crate::placer::meander_placer::MeanderPlacer`] here: it derives from
//! `MEANDER_PLACER_BASE` and duplicates a good deal of the single track
//! placer, `doMove` included, so there is nothing to share.
//!
//! # How the coupling survives the meandering
//!
//! One number, the baseline offset (`pns_dp_meander_placer.cpp:313`).
//! Every meander is fitted against the **centreline** of the two lanes,
//! [`baseline_segment`], and the shape generator draws chain 0 at
//! `+offset` from it and chain 1 at `-offset` (`pns_meander.cpp:795`,
//! `:796`), where the offset is half the pitch. Everything else follows
//! from that single value, inside [`crate::meander`]: the period grows by
//! twice the offset so the two lanes do not touch on the straights, the
//! corner radius floor grows by the offset so the inner lane's corner
//! never inverts, the minimum amplitude grows by it so the excursion is
//! at least as tall as the pair is wide, and the two corner radii differ
//! by exactly the pitch, which is what keeps the gap constant around a
//! corner. Nothing checks the gap afterwards: there is no `checkGap` here
//! as there is in the pair router, the coupling is a property of the
//! construction.
//!
//! # What the pair length is
//!
//! `origPathLength()` (`:180`) is the **longer** of the two lanes, so
//! tuning a pair shortens nothing and only ever adds to whichever lane is
//! being meandered. [`DpMeanderPlacer::origin_path_length`].
//!
//! # Deviations, each documented at its line
//!
//! - The clearance is resolved once per session rather than inside
//!   `spacing()` (erratum E11), as it is for the single track placer.
//! - Erratum E10's four width self intersection clearance is transcribed
//!   as it stands, and it is a different heuristic from the single
//!   placer's `width + spacing`.
//! - `FixRoute` dereferences `m_currentNode` with no null check
//!   (erratum E13). It cannot arise here: a session that never moved has
//!   no scratch branch and [`DpMeanderPlacer::fix_route`] answers false.
//! - Erratum E21's single sign decision is reproduced: the baseline
//!   offset's sign is decided from the **first** coupled span alone, so a
//!   pair that swaps sides part way along the tuned stretch gets the
//!   wrong sign for the rest of it.
//! - Erratum E20's first bail out is transcribed as it stands, reporting
//!   [`TuningStatus::TooShort`] where the second one reports honestly.
//! - The four way arc walk of `addCornersUntilIndex` (`:392` to `:441`)
//!   is ported whole, including the forward hunt that pairs an arc on one
//!   lane with an arc on the other.
//! - `m_coupledSegments`, `m_tunedPath`, `totalLength`, `meanderSegment`,
//!   `setWorld`, `release` and `Trace` are not ported (erratum E12).

use crate::algo_base::AlgoContext;
use crate::collide::CollisionSearchOptions;
use crate::diff_pair::{CoupledSegments, DiffPair};
use crate::geometry::arc::ShapeArc;
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::Vec2;
use crate::item::{ItemId, Kind, NetId};
use crate::line::Line;
use crate::meander::{
  LengthTarget, MeanderContext, MeanderSettings, MeanderShape, MeanderSide,
  MeanderType, MeanderedLine, TuningStatus, amplitude_step, clearance,
  spacing_step, tune_line_length,
};
use crate::node::{NodeId, World};
use crate::placer::meander_placer::{TuningError, snapped_start_point};
use crate::rules::{ItemRef, RuleResolver};
use crate::topology::{self, PathItem};

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

/// The uid of the throwaway item the clearance query is asked about.
///
/// The same convention as [`crate::placer::meander_placer`]: a [`Line`]
/// has no [`crate::item::Item`] of its own, so [`Line::rule_item`] builds
/// one per query and it never enters the arena.
const PROBE_UID: u64 = u64::MAX;

// ---------------------------------------------------------------------
// The baseline
// ---------------------------------------------------------------------

/// The centreline of one coupled span.
///
/// Port of `baselineSegment`,
/// `pcbnew/router/pns_dp_meander_placer.cpp:196`: the midpoints of the
/// two clipped segments' ends. It is the segment the meanders are fitted
/// against, so the pair's footprint is centred on it and neither lane is
/// the reference.
///
/// KiCad's `/ 2` is `VECTOR2I::operator/( int )`, which truncates each
/// component toward zero, and its addition is plain `int` addition that
/// overflows for coordinates past a gigametre. The sum is widened here
/// and the division still truncates toward zero, so the two agree on
/// every input KiCad does not overflow on.
#[must_use]
pub fn baseline_segment(coupled: &CoupledSegments) -> Seg {
  Seg::new(
    midpoint(coupled.coupled_p.a, coupled.coupled_n.a),
    midpoint(coupled.coupled_p.b, coupled.coupled_n.b),
  )
}

/// Which lane sits on the positive side of the centreline.
///
/// Port of `pairOrientation`,
/// `pcbnew/router/pns_dp_meander_placer.cpp:205`, whose answer negates
/// the baseline offset (`:315`) and so decides which lane the generator
/// draws as chain 0.
#[must_use]
pub fn pair_orientation(coupled: &CoupledSegments) -> bool {
  // :207
  let midpoint = midpoint(coupled.coupled_p.a, coupled.coupled_n.a);

  // :211
  coupled.coupled_p.side(midpoint) > 0
}

/// The integer midpoint of two points, truncating toward zero.
fn midpoint(a: Vec2, b: Vec2) -> Vec2 {
  let x = (i64::from(a.x) + i64::from(b.x)) / 2;
  let y = (i64::from(a.y) + i64::from(b.y)) / 2;

  Vec2::new(
    i32::try_from(x).unwrap_or(i32::MAX),
    i32::try_from(y).unwrap_or(i32::MAX),
  )
}

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

/// Where a [`DpMeanderPlacer`] is in its lifecycle.
///
/// The same three states as
/// [`crate::placer::meander_placer::MeanderPlacerState`], and for the
/// same reason: KiCad's placer is newed and deleted by the router, where
/// the facade keeps its placer until the session ends.
#[derive(Clone, Debug)]
pub enum DpMeanderPlacerState {
  /// Nothing has been started.
  Idle,
  /// A pair is being tuned.
  Tuning(Box<DpMeanderTuning>),
  /// The tuned shapes have been fixed into the branch.
  Finished {
    /// What [`DpMeanderPlacer::has_placed_anything`] answers afterwards.
    placed_anything: bool,
    /// What [`DpMeanderPlacer::current_layer`] answers afterwards.
    layer: i32,
    /// What [`DpMeanderPlacer::current_nets`] answers afterwards.
    nets: (Option<NetId>, Option<NetId>),
  },
}

impl DpMeanderPlacerState {
  /// The tuning state, when there is one.
  pub fn tuning(&self) -> Option<&DpMeanderTuning> {
    match self {
      DpMeanderPlacerState::Tuning(tuning) => Some(tuning),
      _ => None,
    }
  }

  /// The tuning state, for a routine that changes it.
  pub fn tuning_mut(&mut self) -> Option<&mut DpMeanderTuning> {
    match self {
      DpMeanderPlacerState::Tuning(tuning) => Some(tuning),
      _ => None,
    }
  }
}

/// Everything one pair tuning session carries.
///
/// The live members of `DP_MEANDER_PLACER`
/// (`pcbnew/router/pns_dp_meander_placer.h:142` to `:165`) plus the four
/// of `MEANDER_PLACER_BASE` this placer reads. The time domain half, the
/// net chain half, the pad to die half and the seven dead declarations of
/// erratum E12 are not here.
#[derive(Clone, Debug)]
pub struct DpMeanderTuning {
  /// `m_initialSegment` (`:163`), the track that was clicked.
  initial_segment: ItemId,
  /// `m_currentStart` (`:142`), the snapped point the tuned stretch
  /// begins at.
  current_start: Vec2,
  /// `m_currentEnd` (`pns_meander_placer_base.h:186`), which this placer
  /// never assigns at all, so `CurrentEnd()` (`:660`) answers the origin
  /// for the life of the session. Erratum E1.
  current_end: Vec2,
  /// `m_world` (`pns_meander_placer_base.h:177`), the branch the placer
  /// owns and removed both lanes from.
  world_node: NodeId,
  /// `m_currentNode` (`:143`), the scratch branch of the last move.
  current_node: Option<NodeId>,
  /// `m_originPair` (`:145`), the pair as it was found on the board.
  origin_pair: DiffPair,
  /// The P lane as assembled, unlinked because `Start` removed it from
  /// [`DpMeanderTuning::world_node`] (`:156`). KiCad reaches the same
  /// object through `m_originPair.PLine()`, which answers the linked
  /// original while it is linked (`pns_diff_pair.h:488`); this crate's
  /// [`DiffPair`] holds chains only, so the line is kept beside it.
  origin_line_p: Line,
  /// The N lane as assembled (`:157`).
  origin_line_n: Line,
  /// `m_tunedPathN` followed by `m_tunedPathP`, which is the order
  /// `TunedPath()` answers in (`:640`). One vector rather than two,
  /// because the only readers are the length of each half and the
  /// highlight the host draws over both.
  tuned_path: Vec<PathItem>,
  /// Where the P half of [`DpMeanderTuning::tuned_path`] begins.
  tuned_path_p_start: usize,
  /// `m_startPad_p` (`pns_meander_placer_base.h:192`). Its pad to die
  /// length is not read; see [`topology::path_length`].
  start_pad_p: Option<ItemId>,
  /// `m_endPad_p` (`:193`).
  end_pad_p: Option<ItemId>,
  /// `m_startPad_n` (`:190`).
  start_pad_n: Option<ItemId>,
  /// `m_endPad_n` (`:191`).
  end_pad_n: Option<ItemId>,
  /// `m_finalShapeP` (`:152`), pre plus tuned plus post on the P lane.
  final_shape_p: LineChain,
  /// `m_finalShapeN` (`:153`).
  final_shape_n: LineChain,
  /// `m_result` (`:154`), the meanders the last move fitted. Dual, so
  /// every shape carries both lanes.
  result: MeanderedLine,
  /// `m_lastLength` (`:158`).
  last_length: i64,
  /// `m_lastStatus` (`:161`).
  last_status: TuningStatus,
  /// `m_currentWidth` (`pns_meander_placer_base.h:180`), one lane's.
  current_width: i32,
  /// `m_baselineLength` (`:166`), the pair length captured at `Start`.
  baseline_length: i64,
  /// What `MEANDER_PLACER_BASE::Clearance()`
  /// (`pns_meander_placer_base.cpp:114`) answers, resolved once; see the
  /// module documentation.
  clearance: i32,
  /// What `CurrentLayer()` reads off the clicked segment (`:666`).
  layer: i32,
  /// `m_originPair.NetP()` and `NetN()`, which `CurrentNets()` answers
  /// with in that order (`:696`).
  nets: (Option<NetId>, Option<NetId>),
}

// ---------------------------------------------------------------------
// The placer
// ---------------------------------------------------------------------

/// The differential pair length tuner.
///
/// Port of `DP_MEANDER_PLACER`,
/// `pcbnew/router/pns_dp_meander_placer.h:48`.
#[derive(Clone, Debug)]
pub struct DpMeanderPlacer {
  /// Where the placer is in its lifecycle.
  state: DpMeanderPlacerState,
  /// The node every session branches from, KiCad's
  /// `Router()->GetWorld()` (`pcbnew/router/pns_dp_meander_placer.cpp:101`).
  root_node: NodeId,
  /// `m_settings` (`pcbnew/router/pns_meander_placer_base.h:183`).
  settings: MeanderSettings,
  /// The fallback gap when the pair's own could not be measured.
  ///
  /// `Router()->Sizes().DiffPairGap()` (`:115`). It is the **only** thing
  /// either pair tuner reads out of [`crate::settings::Sizes`], and
  /// `UpdateSizes` is a no op on every meander placer, so it is captured
  /// at construction and never refreshed, exactly as KiCad's read of a
  /// router field that its host stopped writing once tuning started.
  diff_pair_gap: i32,
  /// The node [`DpMeanderPlacer::fix_route`] wrote both lanes into.
  ///
  /// KiCad has no such member: `FixRoute` commits through the router from
  /// inside the placer (`:579`). Here the facade owns the commit, and
  /// answering [`None`] until a fix has happened is what keeps a session
  /// that only moved from committing the removal of the two lanes it was
  /// going to tune.
  fixed_node: Option<NodeId>,
}

impl DpMeanderPlacer {
  /// An idle pair tuner over one node.
  ///
  /// The constructor, `pcbnew/router/pns_dp_meander_placer.cpp:41`, which
  /// zeroes the last length and the four pad to die values and starts the
  /// status at `TOO_SHORT` (`:57`).
  ///
  /// # Panics
  ///
  /// When `node` is not a live node of `world`.
  #[must_use]
  pub fn new(
    world: &World,
    node: NodeId,
    settings: MeanderSettings,
    diff_pair_gap: i32,
  ) -> Self {
    assert!(
      world.node(node).is_some(),
      "a placer needs a live node to branch from"
    );

    Self {
      state: DpMeanderPlacerState::Idle,
      root_node: node,
      settings,
      diff_pair_gap,
      fixed_node: None,
    }
  }

  // -----------------------------------------------------------------
  // Accessors
  // -----------------------------------------------------------------

  /// The lifecycle state.
  pub const fn state(&self) -> &DpMeanderPlacerState {
    &self.state
  }

  /// The pair as it was found on the board.
  ///
  /// Port of `GetOriginPair()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:74`.
  pub fn origin_pair(&self) -> Option<&DiffPair> {
    self.state.tuning().map(|tuning| &tuning.origin_pair)
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
  /// `pcbnew/router/pns_meander_placer_base.cpp:131`.
  pub const fn set_settings(&mut self, settings: MeanderSettings) {
    self.settings = settings;
  }

  /// Nudge the meander amplitude by one step.
  ///
  /// Port of `AmplitudeStep`,
  /// `pcbnew/router/pns_meander_placer_base.cpp:96`. The caller re-runs
  /// the move, as KiCad's host does.
  pub fn amplitude_step(&mut self, sign: i32) {
    amplitude_step(&mut self.settings, sign);
  }

  /// Nudge the meander spacing by one step.
  ///
  /// Port of `SpacingStep`,
  /// `pcbnew/router/pns_meander_placer_base.cpp:105`, whose floor is the
  /// session's width plus its clearance.
  pub fn spacing_step(&mut self, sign: i32) {
    let (width, clearance) = self
      .state
      .tuning()
      .map_or((0, 0), |tuning| (tuning.current_width, tuning.clearance));

    spacing_step(&mut self.settings, sign, width, clearance);
  }

  /// The two lines the host draws, P first.
  ///
  /// Port of `Traces()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:626`, which **assigns**
  /// `m_currentTraceP` and `m_currentTraceN` before answering. This is a
  /// pure query, for the same reason the single placer's is: the
  /// assignment only matters in KiCad because `Clearance()` reads the
  /// first trace back on every amplitude trial (erratum E11).
  pub fn traces(&self) -> Vec<Line> {
    self.state.tuning().map_or_else(Vec::new, |tuning| {
      vec![
        Line::with_chain(&tuning.origin_line_p, tuning.final_shape_p.clone()),
        Line::with_chain(&tuning.origin_line_n, tuning.final_shape_n.clone()),
      ]
    })
  }

  /// The run of copper the tuned length is measured over, both lanes.
  ///
  /// Port of `TunedPath()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:640`, which answers **N's
  /// items and then P's**, in that order.
  pub fn tuned_path(&self) -> &[PathItem] {
    self
      .state
      .tuning()
      .map_or(&[][..], |tuning| tuning.tuned_path.as_slice())
  }

  /// The track that was clicked.
  ///
  /// `m_initialSegment` (`pcbnew/router/pns_dp_meander_placer.h:163`),
  /// which KiCad reads for `CurrentLayer()` (`:666`) and nothing else.
  pub fn initial_segment(&self) -> Option<ItemId> {
    self.state.tuning().map(|tuning| tuning.initial_segment)
  }

  /// The four terminal pads the two topology walks stopped on, P's pair
  /// first.
  ///
  /// `m_startPad_p`, `m_endPad_p`, `m_startPad_n` and `m_endPad_n`
  /// (`pcbnew/router/pns_meander_placer_base.h:190` to `:193`), which
  /// KiCad reads a pad to die length off; see [`topology::path_length`]
  /// for why nothing here does.
  pub fn terminal_pads(&self) -> [Option<ItemId>; 4] {
    self.state.tuning().map_or([None; 4], |tuning| {
      [
        tuning.start_pad_p,
        tuning.end_pad_p,
        tuning.start_pad_n,
        tuning.end_pad_n,
      ]
    })
  }

  /// Where the tuned stretch begins.
  ///
  /// Port of `CurrentStart()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:654`.
  pub fn current_start(&self) -> Option<Vec2> {
    self.state.tuning().map(|tuning| tuning.current_start)
  }

  /// Where the tuned stretch ends, which is always the origin.
  ///
  /// Port of `CurrentEnd()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:660`, whose `m_currentEnd`
  /// this placer never writes at all. Erratum E1.
  pub fn current_end(&self) -> Option<Vec2> {
    self.state.tuning().map(|tuning| tuning.current_end)
  }

  /// The two nets being tuned, P first.
  ///
  /// Port of `CurrentNets()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:696`.
  pub const fn current_nets(&self) -> Option<(Option<NetId>, Option<NetId>)> {
    match &self.state {
      DpMeanderPlacerState::Idle => None,
      DpMeanderPlacerState::Tuning(tuning) => Some(tuning.nets),
      DpMeanderPlacerState::Finished { nets, .. } => Some(*nets),
    }
  }

  /// The layer being tuned on.
  ///
  /// Port of `CurrentLayer()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:666`, which reads the
  /// clicked segment's first layer.
  pub const fn current_layer(&self) -> Option<i32> {
    match &self.state {
      DpMeanderPlacerState::Idle => None,
      DpMeanderPlacerState::Tuning(tuning) => Some(tuning.layer),
      DpMeanderPlacerState::Finished { layer, .. } => Some(*layer),
    }
  }

  /// The most recent world state.
  ///
  /// Port of `CurrentNode( bool )`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:80`: the scratch branch
  /// when there is one, the placer's own branch otherwise. The
  /// `aLoopsRemoved` argument is ignored there too.
  pub fn current_node(&self) -> NodeId {
    match &self.state {
      DpMeanderPlacerState::Idle => self.root_node,
      DpMeanderPlacerState::Tuning(tuning) => {
        tuning.current_node.unwrap_or(tuning.world_node)
      }
      DpMeanderPlacerState::Finished { .. } => {
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
  /// [`None`] until a fix has happened, which is what keeps a session
  /// that only moved from committing the removal of the two lanes it took
  /// out of its own branch (`:156`, `:157`) and put nothing back.
  pub const fn fixed_node(&self) -> Option<NodeId> {
    self.fixed_node
  }

  /// Whether anything has reached a node.
  ///
  /// Port of `HasPlacedAnything`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:592`, which tests the
  /// **origin** pair's segment counts and is therefore true from the
  /// first successful `Start` onwards, unlike the single placer's, which
  /// tests the trace it built.
  pub fn has_placed_anything(&self) -> bool {
    match &self.state {
      DpMeanderPlacerState::Idle => false,
      DpMeanderPlacerState::Tuning(tuning) => {
        tuning.origin_pair.chain_p().segment_count() > 0
          || tuning.origin_pair.chain_n().segment_count() > 0
      }
      DpMeanderPlacerState::Finished {
        placed_anything, ..
      } => *placed_anything,
    }
  }

  /// How the tuned pair stands against its target.
  ///
  /// Port of `TuningStatus()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:690`.
  pub fn tuning_status(&self) -> Option<TuningStatus> {
    self.state.tuning().map(|tuning| tuning.last_status)
  }

  /// The pair length the last move produced.
  ///
  /// Port of `TuningLengthResult()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:672`: the last measured
  /// length, or the untouched pair length when no move has produced one.
  /// Zero doubles as "no move yet" there, exactly as it does here.
  pub fn tuning_length_result(&self) -> Option<i64> {
    self.state.tuning().map(|tuning| {
      if tuning.last_length == 0 {
        tuning.baseline_length
      } else {
        tuning.last_length
      }
    })
  }

  /// How far that has moved from the length the session started at.
  ///
  /// Port of `TuningLengthDelta()`
  /// (`pcbnew/router/pns_meander_placer_base.h:70`), gated by
  /// `HasBaseline()` (`:68`), which without the delay half is "the
  /// baseline length is not zero".
  pub fn tuning_length_delta(&self) -> Option<i64> {
    let tuning = self.state.tuning()?;

    if tuning.baseline_length == 0 {
      return None;
    }

    Some(self.tuning_length_result()? - tuning.baseline_length)
  }

  /// The pair length, which is the **longer** lane.
  ///
  /// Port of `origPathLength()`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:180`, minus the pad to die
  /// term this crate has no concept of. Taking the maximum is why tuning
  /// a pair never shortens anything.
  pub fn origin_path_length(&self) -> Option<i64> {
    self.state.tuning().map(DpMeanderTuning::origin_path_length)
  }

  // -----------------------------------------------------------------
  // Start
  // -----------------------------------------------------------------

  /// Begin tuning the pair the track under a point belongs to.
  ///
  /// Port of `Start`, `pcbnew/router/pns_dp_meander_placer.cpp:89`: snap
  /// the click onto the clicked segment, branch the world, recover the
  /// pair with [`topology::assemble_diff_pair`], walk each lane's run of
  /// copper, take **both** lanes out of the branch and measure the
  /// baseline.
  ///
  /// Both lanes are removed (`:156`, `:157`), so a meander on one lane
  /// can collide with nothing of the other and
  /// [`DpMeanderPlacer::check_fit`] tests each candidate against
  /// everything else on the board.
  ///
  /// `initChainExtras` (`:167`) and `calculateTimeDomainTargets` (`:169`)
  /// have no counterpart; note 08 section 4.4 works out that both
  /// collapse to a constant zero without a net chain concept.
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
    // :91
    let start_item = start_item.ok_or(TuningError::NeedsStartItem)?;
    let item = world
      .item(start_item)
      .ok_or(TuningError::NotATrack(start_item))?;

    if !item.of_kind(Kind::SEGMENT | Kind::ARC) {
      return Err(TuningError::NotATrack(start_item));
    }

    // :99
    let current_start = snapped_start_point(item, at);
    let layer = item.layers().start();

    // :101
    let world_node = world.branch(self.root_node);

    // :105
    let assembled = topology::assemble_diff_pair(
      world,
      world_node,
      context.resolver,
      start_item,
    )
    .ok_or(TuningError::NotADiffPairForTuning)?;

    let mut origin_pair = assembled.pair;
    let mut origin_line_p = assembled.line_p;
    let mut origin_line_n = assembled.line_n;

    // :114. The measured gap is negative when nothing could be measured.
    if origin_pair.gap() < 0 {
      origin_pair.set_gap(self.diff_pair_gap);
    }

    // :117
    let (Some(seed_p), Some(seed_n)) = (
      origin_line_p.links().first().copied(),
      origin_line_n.links().first().copied(),
    ) else {
      return Err(TuningError::PairLaneHasNoSegments);
    };

    // :120, :138
    let path_p = topology::assemble_tuning_path(
      world,
      world_node,
      context.resolver,
      seed_p,
    );
    let path_n = topology::assemble_tuning_path(
      world,
      world_node,
      context.resolver,
      seed_n,
    );

    // :156, :157. Both lanes leave the branch.
    world.remove_line(world_node, &mut origin_line_p);
    world.remove_line(world_node, &mut origin_line_n);

    // :159
    let current_width = origin_pair.width();
    let nets = origin_pair.nets();

    // :164. `origPathLength()` is the longer of the two lanes.
    let length_p = topology::path_length(&path_p.items);
    let length_n = topology::path_length(&path_n.items);
    let baseline_length = length_p.max(length_n);

    // `MEANDER_PLACER_BASE::Clearance()`
    // (`pcbnew/router/pns_meander_placer_base.cpp:114`), resolved once.
    // The item is `Traces().CItems().front()`, which for this placer is
    // the P lane (`:628`, `:633`).
    let probe = origin_line_p.rule_item(world, PROBE_UID);
    let clearance = clearance(
      context.resolver,
      ItemRef::unstored(&probe),
      layer,
      current_width,
    );

    // `TunedPath()` answers N's items and then P's (`:640`).
    let tuned_path_p_start = path_n.items.len();
    let mut tuned_path = path_n.items;

    tuned_path.extend(path_p.items);

    self.fixed_node = None;
    self.state = DpMeanderPlacerState::Tuning(Box::new(DpMeanderTuning {
      initial_segment: start_item,
      current_start,
      // Never assigned by this placer; erratum E1.
      current_end: Vec2::new(0, 0),
      world_node,
      // :98
      current_node: None,
      origin_pair,
      origin_line_p,
      origin_line_n,
      tuned_path,
      tuned_path_p_start,
      start_pad_p: path_p.start_pad,
      end_pad_p: path_p.end_pad,
      start_pad_n: path_n.start_pad,
      end_pad_n: path_n.end_pad,
      final_shape_p: LineChain::new(),
      final_shape_n: LineChain::new(),
      result: MeanderedLine::new(current_width, true),
      // :55, :57
      last_length: 0,
      last_status: TuningStatus::TooShort,
      current_width,
      baseline_length,
      clearance,
      layer,
      nets,
    }));

    Ok(())
  }

  // -----------------------------------------------------------------
  // Move
  // -----------------------------------------------------------------

  /// Re-meander the coupled stretch between the click and a point.
  ///
  /// Port of `Move`, `pcbnew/router/pns_dp_meander_placer.cpp:215`.
  /// Everything before `:249` is net chain arithmetic that collapses to
  /// nothing here (note 08 section 4.4), and there is no `doMove` split:
  /// this placer compares against `m_settings.m_targetLength` directly
  /// rather than against arguments, because nothing ever hands it a
  /// different window.
  ///
  /// The end item is ignored, as it is in KiCad: there is nothing to snap
  /// a tuned stretch onto.
  ///
  /// # An unconstrained target
  ///
  /// [`MeanderSettings::target_length`] of [`None`] resolves to KiCad's
  /// `LENGTH_UNCONSTRAINED` triple, exactly as it does for the single
  /// track placer: minimum zero, optimum and maximum one kilometre
  /// (`pcbnew/router/pns_meander.cpp:71` to `:74`), so an unconstrained
  /// session meanders as hard as the coupled stretch allows.
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
    let settings = self.settings;

    let Some(tuning) = self.state.tuning_mut() else {
      return false;
    };

    // :249
    if tuning.current_start == at {
      return false;
    }

    // :254, :257
    if let Some(node) = tuning.current_node.take() {
      world.drop_node(node);
    }

    let current_node = world.branch(tuning.world_node);

    tuning.current_node = Some(current_node);

    // :262, :263
    let split_p = tuning
      .origin_pair
      .chain_p()
      .split_three_way(tuning.current_start, at);
    let split_n = tuning
      .origin_pair
      .chain_n()
      .split_three_way(tuning.current_start, at);
    let (
      Some((pre_p, mut tuned_p, post_p)),
      Some((pre_n, mut tuned_n, post_n)),
    ) = (split_p, split_n)
    else {
      return tuning.bail_out_too_short();
    };

    // :265, :266
    tuned_p.simplify(0);
    tuned_n.simplify(0);

    // :270. Erratum E20: this bail out forces `TOO_SHORT` where the one
    // eight lines below reports honestly, and neither writes the delay.
    // Both are transcribed as they stand. It is the guard added for
    // KiCad issue 22041, for split points too close together or outside
    // the chain; the crate reaches it through a `None` from
    // [`LineChain::split_three_way`] as well.
    if tuned_p.point_count() == 0 || tuned_n.point_count() == 0 {
      return tuning.bail_out_too_short();
    }

    // :291 to :295
    let mut tuned = tuning.origin_pair.clone();

    tuned.set_shape(tuned_p.clone(), tuned_n.clone(), false);

    let coupled = tuned.coupled_segment_pairs();

    // :297. The cursor has not reached a coupled stretch yet, so the
    // original geometry is kept rather than letting the pair vanish.
    let Some(first_span) = coupled.first() else {
      tuning.final_shape_p = tuning.origin_pair.chain_p().clone();
      tuning.final_shape_n = tuning.origin_pair.chain_n().clone();
      tuning.last_length = tuning.origin_path_length();
      tuning.last_status =
        TuningStatus::for_length(tuning.last_length, &target);

      return false;
    };

    // :310, :311
    let mut result = MeanderedLine::new(tuned.width(), true);

    result.set_width(tuned.width());

    // :313 to :318. Erratum E21: the sign is decided from the first
    // coupled span alone, so a pair that crosses over inside the tuned
    // stretch gets the wrong sign for every span after the crossing and
    // the two lanes' meanders land on top of each other.
    let mut offset = (tuned.gap() + tuned.width()) / 2;

    if pair_orientation(first_span) {
      offset = -offset;
    }

    result.set_baseline_offset(offset);

    let origin_line_p = tuning.origin_line_p.clone();
    let origin_line_n = tuning.origin_line_n.clone();
    let width = tuning.current_width;
    let clearance = tuning.clearance;
    let dp_length = tuning.origin_path_length();

    // KiCad re-reads `m_settings.m_initialSide` at the top of every
    // coupled span (`:456`) and `flipInitialSide` writes into it from
    // inside `MeanderSegment` (`pns_meander.cpp:282`), so a flip on one
    // span decides which side the next one starts on. The side is a local
    // the loop updates for that reason, and only the parity reaches the
    // placer's own settings; note 08 section 11.6.
    let mut initial_side = settings.initial_side();
    let mut flipped = false;
    let mut last_length = dp_length;
    let mut last_status = TuningStatus::Tuned;

    {
      let node_world: &World = world;
      let resolver = context.resolver;
      let check = |shape: &MeanderShape, placed: &MeanderedLine| -> bool {
        check_fit(
          node_world,
          current_node,
          resolver,
          &origin_line_p,
          &origin_line_n,
          shape,
          placed,
        )
      };
      let meander_context =
        MeanderContext::new(&settings, width, clearance, &check);
      let mut cursor = CornerCursor::new();

      // :451
      for span in &coupled {
        let base = baseline_segment(span);

        // :454 to :459
        let side = match initial_side {
          MeanderSide::Default => base.side(at) < 0,
          MeanderSide::Left => true,
          MeanderSide::Right => false,
        };

        // :463
        cursor.add_corners_until(
          &mut result,
          &tuned_p,
          &tuned_n,
          span.index_p,
          span.index_n,
        );

        // :465
        let outcome = result.meander_segment(&meander_context, base, side);

        if outcome.initial_side_flipped {
          initial_side = initial_side.flipped();
          flipped = !flipped;
        }
      }

      // :468
      cursor.add_corners_until(
        &mut result,
        &tuned_p,
        &tuned_n,
        tuned_p.point_count().saturating_sub(1),
        tuned_n.point_count().saturating_sub(1),
      );

      // :470
      if let (Some(last_p), Some(last_n)) =
        (tuned_p.last_point(), tuned_n.last_point())
      {
        result.add_corner(last_p, last_n);
      }

      // :477
      if dp_length > target.max {
        last_status = TuningStatus::TooLong;
      } else {
        // :485
        last_length = dp_length - tuned_p.length().max(tuned_n.length());

        // :499
        tune_line_length(&meander_context, &mut result, target.opt - dp_length);
      }
    }

    // :502
    if last_status != TuningStatus::TooLong {
      // :504 to :514
      tuned_p.clear();
      tuned_n.clear();

      for meander in result.meanders() {
        if meander.meander_type() != MeanderType::Empty {
          tuned_p.append_chain(meander.cline(0));
          tuned_n.append_chain(meander.cline(1));
        }
      }

      // :516
      last_length += tuned_p.length().max(tuned_n.length());

      // :530
      last_status = TuningStatus::for_length(last_length, &target);
    }

    // :533 to :565. `keepEndpoints` changes only where `Simplify` runs.
    let final_shape_p = glue(&settings, pre_p, tuned_p, post_p);
    let final_shape_n = glue(&settings, pre_n, tuned_n, post_n);

    if flipped {
      self.settings.flip_initial_side();
    }

    let Some(tuning) = self.state.tuning_mut() else {
      return false;
    };

    tuning.result = result;
    tuning.final_shape_p = final_shape_p;
    tuning.final_shape_n = final_shape_n;
    tuning.last_length = last_length;
    tuning.last_status = last_status;

    true
  }

  /// Whether a candidate meander may be placed.
  ///
  /// Port of `CheckFit`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:608`. Public because it is
  /// the fit check the shape generator is handed and because a reader
  /// looking for KiCad's member should find it under its own name;
  /// [`DpMeanderPlacer::move_to`] installs it as a closure.
  ///
  /// Both lanes are tested against the scratch branch (`:613`, `:616`),
  /// which has both of them removed and nothing else. The self
  /// intersection clearance is four widths (`:620`), a bare heuristic
  /// with no design rule behind it and a different one from the single
  /// placer's `width + spacing`; erratum E10, both halves transcribed as
  /// they stand.
  ///
  /// `CheckSelfIntersections` then looks at chain 0 only
  /// (`pns_meander.cpp:714`), so the N lane is never tested against
  /// earlier meanders; erratum E7, transcribed in [`crate::meander`].
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
      &tuning.origin_line_p,
      &tuning.origin_line_n,
      shape,
      placed,
    )
  }

  // -----------------------------------------------------------------
  // Fix, commit, abort
  // -----------------------------------------------------------------

  /// Write both tuned lanes into the branch and end the session.
  ///
  /// Port of `FixRoute`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:571`, which ignores its
  /// point, its end item and the tuning status entirely and fixes
  /// whatever the last `Move` produced.
  ///
  /// # Erratum E13
  ///
  /// KiCad dereferences `m_currentNode` at `:576` with no null check
  /// where the single placer guards the same dereference
  /// (`pns_meander_placer.cpp:366`), so a host that fixes without a
  /// successful move crashes. Answering false is what the guard would
  /// have done.
  pub fn fix_route(
    &mut self,
    world: &mut World,
    _at: Vec2,
    _end_item: Option<ItemId>,
    _force_finish: bool,
  ) -> bool {
    let placed_anything = self.has_placed_anything();
    let Some(tuning) = self.state.tuning_mut() else {
      return false;
    };
    let Some(node) = tuning.current_node else {
      return false;
    };

    // :573 to :577
    let mut trace_p =
      Line::with_chain(&tuning.origin_line_p, tuning.final_shape_p.clone());
    let mut trace_n =
      Line::with_chain(&tuning.origin_line_n, tuning.final_shape_n.clone());

    world.add_line(node, &mut trace_p, false);
    world.add_line(node, &mut trace_n, false);

    let layer = tuning.layer;
    let nets = tuning.nets;

    self.fixed_node = Some(node);
    self.state = DpMeanderPlacerState::Finished {
      placed_anything,
      layer,
      nets,
    };

    true
  }

  /// Fold the fixed branch into the board.
  ///
  /// Port of `CommitPlacement`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:598`. A session that never
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
  /// `pcbnew/router/pns_dp_meander_placer.cpp:585`, which kills the
  /// placement world's children. The two lanes the session removed from
  /// its own branch come back with the branch.
  pub fn abort_placement(&mut self, world: &mut World) {
    if let Some(tuning) = self.state.tuning() {
      world.kill_children(tuning.world_node);
    }

    self.fixed_node = None;
    self.state = DpMeanderPlacerState::Idle;
  }

  // -----------------------------------------------------------------
  // The commands a meander placer does not implement
  // -----------------------------------------------------------------

  /// Roll the tuning back one step. Always [`None`].
  ///
  /// `UnfixRoute` is not overridden by any of the three meander placers
  /// (`pcbnew/router/pns_placement_algo.h:81`), so backspace does nothing
  /// during a tuning session. Note 08 section 5.8.
  pub const fn undo_last_segment(&self) -> Option<Vec2> {
    None
  }

  /// Arm a via. Always false.
  ///
  /// `ToggleVia` is not overridden
  /// (`pcbnew/router/pns_placement_algo.h:90`).
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
  /// (`pcbnew/router/pns_placement_algo.h:114`).
  pub const fn set_layer(&self, _layer: i32) -> bool {
    false
  }

  /// Flip the posture. Does nothing.
  ///
  /// `FlipPosture` is not overridden
  /// (`pcbnew/router/pns_placement_algo.h:167`).
  pub const fn flip_posture(&self) {}
}

impl DpMeanderTuning {
  /// The pair length, the longer of the two lanes.
  ///
  /// `origPathLength()`, `pcbnew/router/pns_dp_meander_placer.cpp:180`.
  fn origin_path_length(&self) -> i64 {
    let (path_n, path_p) = self.tuned_path.split_at(self.tuned_path_p_start);

    topology::path_length(path_p).max(topology::path_length(path_n))
  }

  /// The `:270` bail out: keep the original geometry and report
  /// [`TuningStatus::TooShort`] whatever the truth is.
  ///
  /// Erratum E20. `Move` answers false afterwards, so the caller returns
  /// it straight back.
  fn bail_out_too_short(&mut self) -> bool {
    self.final_shape_p = self.origin_pair.chain_p().clone();
    self.final_shape_n = self.origin_pair.chain_n().clone();
    self.last_length = self.origin_path_length();
    self.last_status = TuningStatus::TooShort;

    false
  }
}

// ---------------------------------------------------------------------
// The corner walk
// ---------------------------------------------------------------------

/// The two cursors `addCornersUntilIndex` walks the tuned chains with.
///
/// `curIndexP` and `curIndexN`, `pcbnew/router/pns_dp_meander_placer.cpp:376`.
/// KiCad carries them as `int`s and uses `-1` for "past the end", which is
/// what `SHAPE_LINE_CHAIN::NextShape` answers at the last segment
/// (`libs/kimath/src/geometry/shape_line_chain.cpp:1313`); an [`Option`]
/// says the same without the sentinel.
struct CornerCursor {
  /// Which segment of the P chain the walk stands on.
  index_p: Option<usize>,
  /// Which segment of the N chain the walk stands on.
  index_n: Option<usize>,
}

impl CornerCursor {
  /// Both cursors at the first segment.
  const fn new() -> Self {
    Self {
      index_p: Some(0),
      index_n: Some(0),
    }
  }

  /// Carry the uncoupled stretch before a coupled span through unchanged.
  ///
  /// Port of `addCornersUntilIndex`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:378`: one
  /// `AddCorner( p, n )` per pair of aligned vertices, both cursors
  /// advancing together, until both have passed their stop index. That is
  /// what keeps the two lanes' vertices paired up across the parts of the
  /// tuned stretch no meander is fitted into.
  ///
  /// # The four way branch
  ///
  /// `:392` to `:441` splits on which side stands on an arc. Neither does:
  /// one corner. Both do: one `AddArc`, and the two arcs travel together.
  /// Exactly one does: the corner goes out first and then the walk **hunts
  /// forward on the other lane alone** until it finds an arc to pair the
  /// first one with, emitting a corner per straight shape it passes and
  /// repeating the arc bearing lane's own start point for each of them.
  /// That hunt is the whole reason the routine exists: two lanes of a pair
  /// can have their arcs at different indices, and the pairing has to be
  /// arc to arc or the meander list goes out of step.
  ///
  /// The hunt can also run out without finding an arc, in which case the
  /// outer loop simply carries on with the lane cursor left where the hunt
  /// abandoned it.
  ///
  /// # The exhausted cursor
  ///
  /// KiCad calls `getItem` on both chains before testing either cursor
  /// (`:389`, `:390`), so when one side has run out and the other has not
  /// it reads `GetSegment( -1 )`, which
  /// `SHAPE_LINE_CHAIN::Segment` resolves as an index from the back
  /// (`libs/kimath/include/geometry/shape_line_chain.h:381`) and therefore
  /// answers the **last** segment. `IsArcSegment( -1 )` is false, the
  /// `size_t` wrap landing back on the bound check
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:3246`), so an
  /// exhausted cursor never reads as an arc. The exhausted side's corner
  /// point is its last segment's start, over and over, until the other
  /// side catches up. That is transcribed rather than tidied, and it can
  /// only arise when the two lanes have different shape counts inside the
  /// tuned stretch.
  fn add_corners_until(
    &mut self,
    result: &mut MeanderedLine,
    tuned_p: &LineChain,
    tuned_n: &LineChain,
    last_index_p: usize,
    last_index_n: usize,
  ) {
    loop {
      // :382 to :387. `checkIndex` writes its flag through a reference,
      // so a hunt below leaves the flag it walked on behind for the
      // cursor advance at `:443` to read. Both are mutable here for the
      // same reason.
      let mut p_ok = self.index_p.is_some_and(|index| index <= last_index_p);
      let mut n_ok = self.index_n.is_some_and(|index| index <= last_index_n);

      if !p_ok && !n_ok {
        break;
      }

      // :389, :390
      let (Some(p_item), Some(n_item)) = (
        get_item(tuned_p, self.index_p),
        get_item(tuned_n, self.index_n),
      ) else {
        break;
      };

      match (p_item.arc, n_item.arc) {
        // :392
        (None, None) => result.add_corner(p_item.start, n_item.start),
        // :396
        (Some(p_arc), Some(n_arc)) => result.add_arc(&p_arc, &n_arc),
        // :400. P is on an arc, N is not: hunt forward on N.
        (Some(p_arc), None) => {
          result.add_corner(p_item.start, n_item.start);

          while n_ok {
            self.index_n = chain_next_shape(tuned_n, self.index_n);

            let Some(found) = get_item(tuned_n, self.index_n) else {
              break;
            };

            if let Some(n_arc) = found.arc {
              result.add_arc(&p_arc, &n_arc);
              break;
            }

            result.add_corner(p_item.start, found.start);
            n_ok = self.index_n.is_some_and(|index| index <= last_index_n);
          }
        }
        // :420. The mirror image, hunting forward on P.
        (None, Some(n_arc)) => {
          result.add_corner(p_item.start, n_item.start);

          while p_ok {
            self.index_p = chain_next_shape(tuned_p, self.index_p);

            let Some(found) = get_item(tuned_p, self.index_p) else {
              break;
            };

            if let Some(p_arc) = found.arc {
              result.add_arc(&p_arc, &n_arc);
              break;
            }

            result.add_corner(found.start, n_item.start);
            p_ok = self.index_p.is_some_and(|index| index <= last_index_p);
          }
        }
      }

      // :443 to :447
      if p_ok {
        self.index_p = chain_next_shape(tuned_p, self.index_p);
      }

      if n_ok {
        self.index_n = chain_next_shape(tuned_n, self.index_n);
      }
    }
  }
}

/// One shape of one lane, as the corner walk sees it.
///
/// Port of `GET_ITEM_RET` and the `getItem` lambda,
/// `pcbnew/router/pns_dp_meander_placer.cpp:340` and `:347`. KiCad carries
/// the end point too and reads it nowhere, so it is not here.
struct WalkItem {
  /// The arc this shape belongs to, or [`None`] for a straight segment.
  arc: Option<ShapeArc>,
  /// `startPt`: the arc's start, or the segment's.
  start: Vec2,
}

/// The shape a cursor stands on.
///
/// `getItem`, `pcbnew/router/pns_dp_meander_placer.cpp:347`. A cursor past
/// the end, KiCad's `-1`, reads the **last** segment and never reads as an
/// arc; [`CornerCursor::add_corners_until`] documents why.
fn get_item(chain: &LineChain, index: Option<usize>) -> Option<WalkItem> {
  // :352
  if let Some(index) = index
    && chain.is_arc_segment(index)
    && let Some(arc) = chain.arc_index(index).and_then(|which| chain.arc(which))
  {
    return Some(WalkItem {
      arc: Some(arc),
      start: arc.start(),
    });
  }

  // :361
  Some(WalkItem {
    arc: None,
    start: segment_start(chain, index)?,
  })
}

/// The start point of a chain's segment, with KiCad's index from the back.
///
/// `SHAPE_LINE_CHAIN::Segment( int )`,
/// `libs/kimath/include/geometry/shape_line_chain.h:381`: a negative index
/// counts from the end, so [`None`], KiCad's `-1`, reads the last segment.
fn segment_start(chain: &LineChain, index: Option<usize>) -> Option<Vec2> {
  let count = chain.segment_count();

  if count == 0 {
    return None;
  }

  let index = index.unwrap_or(count - 1).min(count - 1);

  Some(chain.segment(index).a)
}

/// The index of the next shape, or [`None`] at the last one.
///
/// `SHAPE_LINE_CHAIN::NextShape`,
/// `libs/kimath/src/geometry/shape_line_chain.cpp:1302`, with KiCad's
/// `-1` spelled as [`None`] on both sides. An already exhausted cursor
/// stays exhausted, where KiCad would call `NextShape( -1 )` and read
/// `m_shapes` out of bounds.
fn chain_next_shape(chain: &LineChain, index: Option<usize>) -> Option<usize> {
  chain.next_shape(index?)
}

// ---------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------

/// Glue the three parts of one lane back into a chain.
///
/// `pcbnew/router/pns_dp_meander_placer.cpp:536` to `:565`.
/// `keepEndpoints` changes only where `Simplify` runs: per part when set,
/// over the concatenation when not. The host forces it true
/// (`pcbnew/generators/pcb_tuning_pattern.cpp:1297`) so that the vertices
/// at the two seams survive and it can tell which segments belong to the
/// pattern.
fn glue(
  settings: &MeanderSettings,
  mut pre: LineChain,
  mut tuned: LineChain,
  mut post: LineChain,
) -> LineChain {
  let mut chain = LineChain::new();

  if settings.keep_endpoints() {
    pre.simplify(0);
    tuned.simplify(0);
    post.simplify(0);
  }

  chain.append_chain(&pre);
  chain.append_chain(&tuned);
  chain.append_chain(&post);

  if !settings.keep_endpoints() {
    chain.simplify(0);
  }

  chain
}

/// The body of [`DpMeanderPlacer::check_fit`], as a free function so that
/// [`DpMeanderPlacer::move_to`] can install it as a closure while it holds
/// the meander list.
fn check_fit(
  world: &World,
  node: NodeId,
  resolver: &dyn RuleResolver,
  origin_line_p: &Line,
  origin_line_n: &Line,
  shape: &MeanderShape,
  placed: &MeanderedLine,
) -> bool {
  // :610, :611
  let lane_p = Line::with_chain(origin_line_p, shape.cline(0).clone());
  let lane_n = Line::with_chain(origin_line_n, shape.cline(1).clone());

  // :613, :616. `NODE::CheckColliding( const ITEM* )` takes the default
  // kind mask and a limit of one, the answer being a yes or a no.
  let options = CollisionSearchOptions {
    limit_count: Some(1),
    ..CollisionSearchOptions::default()
  };

  for lane in [&lane_p, &lane_n] {
    if world
      .check_colliding_line(node, lane, resolver, &options)
      .is_some()
    {
      return false;
    }
  }

  // :619, :620. Erratum E10; KiCad computes the sum in an `int`.
  let clearance = shape.width().saturating_mul(4);

  // :622
  placed.check_self_intersections(shape, clearance)
}
