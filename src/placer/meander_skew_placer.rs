// SPDX-License-Identifier: GPL-3.0-or-later

//! Skew tuning between the two lanes of a differential pair.
//!
//! Port of `PNS::MEANDER_SKEW_PLACER`
//! (`pcbnew/router/pns_meander_skew_placer.h:39`,
//! `pcbnew/router/pns_meander_skew_placer.cpp`). The reference note is
//! `doc/reference/kicad/08-meanders.md` section 7.
//!
//! # It does not pick a lane
//!
//! The name suggests the tuner finds the shorter lane and lengthens it.
//! It does not. `Start` assembles the line of the item the **user
//! clicked** (`:72`), removes that line from its branch (`:128`), and
//! every subsequent move meanders it; the other lane is only measured.
//! Click the longer lane and ask for zero skew and the target lands below
//! the current length, every meander is emptied on the first pass of
//! `tuneLineLength`, and the move reports [`TuningStatus::TooLong`]. The
//! user's remedy is to click the other lane, and there is no automatic
//! choice anywhere in the class.
//!
//! # What changes against the single track placer
//!
//! Only the target. `MEANDER_SKEW_PLACER` derives from `MEANDER_PLACER`
//! and reuses its `doMove` whole (`:234`), handing it "the other lane's
//! length, plus the requested skew" in place of a length target. That
//! inheritance is composition here:
//! [`MeanderSkewPlacer::inner`] is a
//! [`crate::placer::meander_placer::MeanderPlacer`] and
//! [`crate::placer::meander_placer::MeanderPlacer::do_move`] is the one
//! method reached across the boundary.
//!
//! Two accessors are overridden rather than inherited. `origPathLength()`
//! (`:177`) answers the **active** lane's length alone, and
//! `TuningLengthResult()` (`:240`) answers the **skew** rather than a
//! length, which is why the host labels the readout "current skew" in
//! this mode (`pcbnew/generators/pcb_tuning_pattern.cpp:2147`).
//!
//! # Deviations, each documented at its line
//!
//! - Erratum E9: `doMove`'s early "too long" test reads its own
//!   `aTargetMax` argument, where KiCad reads
//!   `m_settings.m_targetLength.Max()` and so never fires in skew mode,
//!   the default target length being one kilometre. The port's early test
//!   therefore **can** fire where KiCad's cannot; the erratum asks for
//!   exactly that and calls KiCad's behaviour correct by accident.
//! - Erratum E14: `Start` writes `m_tunedPath` twice, once from
//!   `AssembleTrivialPath` (`:75`) and once from the matching
//!   `AssembleTuningPath` result (`:155`). Only the second is ported, and
//!   it is the only call to `AssembleTrivialPath` in any tuning code.
//! - The pair is recovered before the single track half starts, where
//!   KiCad does both against one branch; see [`MeanderSkewPlacer::start`].
//! - `GetSignalAggregate` (`:146`) and `calculateTimeDomainTargets`
//!   (`:252`) have no counterpart: both are the net chain and time domain
//!   halves note 08 section 4.4 works out to a constant zero here.

use crate::algo_base::AlgoContext;
use crate::diff_pair::DiffPair;
use crate::geometry::vec2::Vec2;
use crate::item::{ItemId, Kind, NetId};
use crate::line::Line;
use crate::meander::{
  LengthTarget, MeanderSettings, MeanderShape, MeanderedLine, TuningStatus,
};
use crate::node::{NodeId, World};
use crate::placer::meander_placer::{MeanderPlacer, TuningError};
use crate::rules::RuleResolver;
use crate::topology::{self, PathItem};

/// The skew a session asks for when the settings name none.
///
/// `MEANDER_SETTINGS::SKEW_UNCONSTRAINED`
/// (`pcbnew/router/pns_meander.cpp:37`) is `std::numeric_limits<int>::max()`,
/// and `SetTargetSkew` widens it to a window of zero to that (`:186` to
/// `:190`). [`crate::meander::MeanderSettings::target_skew`] of [`None`]
/// is that sentinel said without a sentinel, and it resolves back to it
/// here, so an unconstrained skew session meanders the active lane as hard
/// as the stretch allows and reports [`TuningStatus::TooShort`].
const SKEW_UNCONSTRAINED: i64 = i32::MAX as i64;

/// What a skew tuning session carries beyond the single track state.
///
/// The five members `MEANDER_SKEW_PLACER` adds to `MEANDER_PLACER`
/// (`pcbnew/router/pns_meander_skew_placer.h:66` to `:80`), minus the
/// delay half and minus the four pad to die values this crate has no
/// concept of.
#[derive(Clone, Debug)]
pub struct SkewTuning {
  /// `m_originPair` (`:66`), the pair the clicked lane belongs to.
  origin_pair: DiffPair,
  /// `m_coupledLength` (`:73`), the **other** lane's total length, which
  /// the target is measured against.
  coupled_length: i64,
  /// `pIsActive` (`:137`), whether the clicked lane is the P half.
  p_is_active: bool,
  /// `CurrentNets()` (`pcbnew/router/pns_meander_skew_placer.h:60`), the
  /// **active** net first and the coupled one second. The order matters
  /// in KiCad because `chainNarrowingOffset` looks up `nets[0]`'s board
  /// length; it is kept because a host reading the readout wants to know
  /// which lane it is looking at.
  nets: (Option<NetId>, Option<NetId>),
  /// The length window the last move actually compared against, which is
  /// the coupled length plus the skew window (`:234` to `:236`).
  ///
  /// KiCad has no such member: its host never asks what `doMove` was
  /// given. [`crate::router::TuningInfo::target`] does.
  target: LengthTarget,
}

/// The skew tuner.
///
/// Port of `MEANDER_SKEW_PLACER`,
/// `pcbnew/router/pns_meander_skew_placer.h:39`.
#[derive(Clone, Debug)]
pub struct MeanderSkewPlacer {
  /// The single track placer this one is, in KiCad, a subclass of.
  inner: MeanderPlacer,
  /// The pair half of the session, once one is running.
  skew: Option<Box<SkewTuning>>,
  /// The fallback gap when the pair's own could not be measured.
  ///
  /// `Router()->Sizes().DiffPairGap()`
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:86`).
  diff_pair_gap: i32,
}

impl MeanderSkewPlacer {
  /// An idle skew tuner over one node.
  ///
  /// The constructor, `pcbnew/router/pns_meander_skew_placer.cpp:41`,
  /// which zeroes the coupled length and the four pad to die values.
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
    Self {
      inner: MeanderPlacer::new(world, node, settings),
      skew: None,
      diff_pair_gap,
    }
  }

  // -----------------------------------------------------------------
  // Accessors
  // -----------------------------------------------------------------

  /// The single track placer underneath.
  pub const fn inner(&self) -> &MeanderPlacer {
    &self.inner
  }

  /// The pair the clicked lane belongs to.
  ///
  /// `m_originPair` (`pcbnew/router/pns_meander_skew_placer.h:66`), which
  /// KiCad has no accessor for.
  pub fn origin_pair(&self) -> Option<&DiffPair> {
    self.skew.as_deref().map(|skew| &skew.origin_pair)
  }

  /// The dimensions the meanders are drawn to.
  pub const fn meander_settings(&self) -> &MeanderSettings {
    self.inner.meander_settings()
  }

  /// Replace the dimensions the meanders are drawn to.
  pub const fn set_settings(&mut self, settings: MeanderSettings) {
    self.inner.set_settings(settings);
  }

  /// Nudge the meander amplitude by one step.
  pub fn amplitude_step(&mut self, sign: i32) {
    self.inner.amplitude_step(sign);
  }

  /// Nudge the meander spacing by one step.
  pub fn spacing_step(&mut self, sign: i32) {
    self.inner.spacing_step(sign);
  }

  /// The line the host draws, which is the active lane's.
  pub fn traces(&self) -> Vec<Line> {
    self.inner.traces()
  }

  /// The run of copper the active lane's length is measured over.
  ///
  /// `TunedPath()` answers `m_tunedPath`, which `Start` overwrites with
  /// whichever of the two lanes was clicked
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:155`, `:163`); erratum
  /// E14's dead first assignment is not ported.
  pub fn tuned_path(&self) -> &[PathItem] {
    self.inner.tuned_path()
  }

  /// The track that was clicked.
  pub fn initial_segment(&self) -> Option<ItemId> {
    self.inner.initial_segment()
  }

  /// Where the tuned stretch begins.
  pub fn current_start(&self) -> Option<Vec2> {
    self.inner.current_start()
  }

  /// Where the tuned stretch ends, which is always the origin.
  ///
  /// `m_currentEnd` is written once to `(0,0)`
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:131`) and never again;
  /// erratum E1.
  pub fn current_end(&self) -> Option<Vec2> {
    self.inner.current_end()
  }

  /// The two nets, the **active** lane first.
  ///
  /// Port of `CurrentNets()`,
  /// `pcbnew/router/pns_meander_skew_placer.h:60`, whose comment at `:62`
  /// says the order matters.
  pub fn current_nets(&self) -> Option<(Option<NetId>, Option<NetId>)> {
    self.skew.as_deref().map(|skew| skew.nets)
  }

  /// The layer being tuned on.
  pub const fn current_layer(&self) -> Option<i32> {
    self.inner.current_layer()
  }

  /// The most recent world state.
  pub fn current_node(&self) -> NodeId {
    self.inner.current_node()
  }

  /// The scratch branch of the last move, when there is one.
  pub fn last_node(&self) -> Option<NodeId> {
    self.inner.last_node()
  }

  /// The node a commit should fold into the board, if any.
  pub const fn fixed_node(&self) -> Option<NodeId> {
    self.inner.fixed_node()
  }

  /// Whether anything has reached a node.
  pub fn has_placed_anything(&self) -> bool {
    self.inner.has_placed_anything()
  }

  /// How the active lane stands against the target skew.
  pub fn tuning_status(&self) -> Option<TuningStatus> {
    self.inner.tuning_status()
  }

  /// The skew between the two lanes, which is what this placer reports.
  ///
  /// Port of `CurrentSkew()`,
  /// `pcbnew/router/pns_meander_skew_placer.cpp:195`, and of
  /// `TuningLengthResult()` (`:240`), which are the same expression:
  /// `m_lastLength - m_coupledLength`. It is a **skew** and not a length,
  /// which is the one thing a host has to know about this mode.
  pub fn current_skew(&self) -> Option<i64> {
    let skew = self.skew.as_deref()?;

    Some(self.inner.last_length()? - skew.coupled_length)
  }

  /// The skew, under the name the placer interface uses.
  ///
  /// `TuningLengthResult()` overridden at
  /// `pcbnew/router/pns_meander_skew_placer.cpp:240`. Note it does
  /// **not** go through `MEANDER_PLACER`'s "zero means no move yet"
  /// fallback (`pns_meander_placer.cpp:443`): `Start` seeds
  /// `m_lastLength` with the active lane's length (`:152`, `:160`), so
  /// the skew is right from the first frame.
  pub fn tuning_length_result(&self) -> Option<i64> {
    self.current_skew()
  }

  /// How far the readout has moved from where the session started.
  ///
  /// `TuningLengthDelta()` (`pcbnew/router/pns_meander_placer_base.h:70`)
  /// is not virtual and subtracts `m_baselineLength` from whatever
  /// `TuningLengthResult()` answers. In this mode that is a **skew minus
  /// a length**, which is not a meaningful quantity; it is transcribed
  /// because it is what KiCad's host reads, and a host that shows a delta
  /// in skew mode should not.
  pub fn tuning_length_delta(&self) -> Option<i64> {
    let baseline = self.origin_path_length()?;

    if baseline == 0 {
      return None;
    }

    Some(self.tuning_length_result()? - baseline)
  }

  /// The other lane's total length.
  ///
  /// `m_coupledLength`
  /// (`pcbnew/router/pns_meander_skew_placer.h:73`), which the target is
  /// measured against and which a host needs to make sense of the skew.
  pub fn coupled_length(&self) -> Option<i64> {
    self.skew.as_deref().map(|skew| skew.coupled_length)
  }

  /// The active lane's length, which is what this placer's baseline is.
  ///
  /// Port of `origPathLength()`,
  /// `pcbnew/router/pns_meander_skew_placer.cpp:177`, which overrides the
  /// base and answers the clicked lane alone. It is the value
  /// [`crate::placer::meander_placer::MeanderPlacer::start`] already
  /// measured, because the path it walked is the active lane's.
  pub fn origin_path_length(&self) -> Option<i64> {
    Some(topology::path_length(self.inner.tuned_path()))
  }

  /// The length window the last move compared against.
  ///
  /// The coupled length plus the skew window
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:234` to `:236`), which
  /// is what `doMove` was given and is not the same thing as
  /// [`crate::meander::MeanderSettings::target_skew`].
  pub fn target(&self) -> Option<LengthTarget> {
    self.skew.as_deref().map(|skew| skew.target)
  }

  // -----------------------------------------------------------------
  // Start
  // -----------------------------------------------------------------

  /// Begin skew tuning the lane under a point.
  ///
  /// Port of `Start`,
  /// `pcbnew/router/pns_meander_skew_placer.cpp:59`.
  ///
  /// # Why the pair is recovered first
  ///
  /// KiCad branches the world once and does everything against that
  /// branch, assembling the pair (`:77`) and both lanes' tuning paths
  /// (`:92`, `:110`) before it removes the clicked lane (`:128`). The
  /// single track half of this port owns its branch and removes the
  /// clicked lane at the end of its own `Start`, so the pair is recovered
  /// against the node that branch is taken **from**, before it exists. A
  /// fresh branch answers exactly what its parent does, so the two agree;
  /// what they cannot do is run in KiCad's order.
  ///
  /// # Errors
  ///
  /// [`TuningError`]. The clicked object not being a track is
  /// [`TuningError::NotATrack`], which KiCad words differently here
  /// ("Please select a differential pair track you want to tune.",
  /// `:63`) but tests identically (`:61`).
  pub fn start(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    start_item: Option<ItemId>,
  ) -> Result<(), TuningError> {
    // :61
    let start_item = start_item.ok_or(TuningError::NeedsStartItem)?;
    let item = world
      .item(start_item)
      .ok_or(TuningError::NotATrack(start_item))?;

    if !item.of_kind(Kind::SEGMENT | Kind::ARC) {
      return Err(TuningError::NotATrack(start_item));
    }

    let active_net = item.net();
    let root = self.inner.root_node();

    // :77
    let assembled =
      topology::assemble_diff_pair(world, root, context.resolver, start_item)
        .ok_or(TuningError::NotADiffPairForSkew)?;
    let mut origin_pair = assembled.pair;

    // :85
    if origin_pair.gap() < 0 {
      origin_pair.set_gap(self.diff_pair_gap);
    }

    // :88
    let (Some(seed_p), Some(seed_n)) = (
      assembled.line_p.links().first().copied(),
      assembled.line_n.links().first().copied(),
    ) else {
      return Err(TuningError::PairLaneHasNoSegments);
    };

    // :92, :110
    let path_p =
      topology::assemble_tuning_path(world, root, context.resolver, seed_p);
    let path_n =
      topology::assemble_tuning_path(world, root, context.resolver, seed_n);
    let length_p = topology::path_length(&path_p.items);
    let length_n = topology::path_length(&path_n.items);

    // :137
    let (net_p, net_n) = origin_pair.nets();
    let p_is_active = net_p == active_net;

    // :149 to :164. `GetSignalAggregate`'s two extra terms are zero.
    let (coupled_length, last_length, nets) = if p_is_active {
      (length_n, length_p, (net_p, net_n))
    } else {
      (length_p, length_n, (net_n, net_p))
    };

    // :69 to :75, :128 to :134. The single track half branches the world,
    // assembles the clicked lane and its tuning path, removes it and
    // measures the baseline, which is `origPathLength()` here.
    self.inner.start(world, context, at, Some(start_item))?;

    // :152, :160
    self.inner.set_last_length(last_length);
    self.skew = Some(Box::new(SkewTuning {
      origin_pair,
      coupled_length,
      p_is_active,
      nets,
      target: skew_window(self.inner.meander_settings(), coupled_length),
    }));

    Ok(())
  }

  /// Whether the clicked lane is the P half.
  ///
  /// `pIsActive` (`pcbnew/router/pns_meander_skew_placer.cpp:137`), which
  /// decides which lane is meandered and which is only measured.
  pub fn p_is_active(&self) -> Option<bool> {
    self.skew.as_deref().map(|skew| skew.p_is_active)
  }

  // -----------------------------------------------------------------
  // Move
  // -----------------------------------------------------------------

  /// Re-meander the active lane between the click and a point.
  ///
  /// Port of `Move`, `pcbnew/router/pns_meander_skew_placer.cpp:201`,
  /// which is the two highlight loops, the chain narrowing offset and one
  /// call to the inherited `doMove` with a target of "the other lane's
  /// length, plus the requested skew" (`:234`).
  ///
  /// `chainNarrowingOffset()` is a constant zero without a net chain
  /// concept, so it is not subtracted.
  pub fn move_to(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    _end_item: Option<ItemId>,
  ) -> bool {
    let Some(skew) = self.skew.as_deref_mut() else {
      return false;
    };
    let target =
      skew_window(self.inner.meander_settings(), skew.coupled_length);

    skew.target = target;

    // :234
    self
      .inner
      .do_move(world, context, at, target.opt, target.min, target.max)
  }

  /// Whether a candidate meander may be placed.
  ///
  /// `CheckFit` is not overridden, so this is
  /// `MEANDER_PLACER::CheckFit` (`pcbnew/router/pns_meander_placer.cpp:400`)
  /// with erratum E10's `width + spacing` self intersection clearance.
  pub fn check_fit(
    &self,
    world: &World,
    resolver: &dyn RuleResolver,
    shape: &MeanderShape,
    placed: &MeanderedLine,
  ) -> bool {
    self.inner.check_fit(world, resolver, shape, placed)
  }

  // -----------------------------------------------------------------
  // Fix, commit, abort
  // -----------------------------------------------------------------

  /// Write the tuned lane into the branch and end the session.
  ///
  /// `FixRoute` is not overridden, so this is
  /// `MEANDER_PLACER::FixRoute` (`pcbnew/router/pns_meander_placer.cpp:364`)
  /// with its null check. Only the active lane is written: the coupled
  /// one was never removed and was never touched.
  pub fn fix_route(
    &mut self,
    world: &mut World,
    at: Vec2,
    end_item: Option<ItemId>,
    force_finish: bool,
  ) -> bool {
    self.inner.fix_route(world, at, end_item, force_finish)
  }

  /// Fold the fixed branch into the board.
  pub fn commit_placement(&mut self, world: &mut World) -> bool {
    self.inner.commit_placement(world)
  }

  /// Throw the session away.
  pub fn abort_placement(&mut self, world: &mut World) {
    self.inner.abort_placement(world);
    self.skew = None;
  }

  // -----------------------------------------------------------------
  // The commands a meander placer does not implement
  // -----------------------------------------------------------------

  /// Roll the tuning back one step. Always [`None`].
  pub const fn undo_last_segment(&self) -> Option<Vec2> {
    self.inner.undo_last_segment()
  }

  /// Arm a via. Always false.
  pub const fn toggle_via(&self, enabled: bool) -> bool {
    self.inner.toggle_via(enabled)
  }

  /// Whether a via is pending. Always false.
  pub const fn is_placing_via(&self) -> bool {
    self.inner.is_placing_via()
  }

  /// Change layer. Always false.
  pub const fn set_layer(&self, layer: i32) -> bool {
    self.inner.set_layer(layer)
  }

  /// Flip the posture. Does nothing.
  pub const fn flip_posture(&self) {
    self.inner.flip_posture();
  }
}

/// The length window `doMove` is handed in skew mode.
///
/// `pcbnew/router/pns_meander_skew_placer.cpp:234` to `:236`: the coupled
/// lane's length plus each of the three skew bounds. A [`None`] skew
/// target resolves to [`SKEW_UNCONSTRAINED`]'s window, zero to
/// `i32::MAX`.
fn skew_window(
  settings: &MeanderSettings,
  coupled_length: i64,
) -> LengthTarget {
  let skew = settings.target_skew().unwrap_or(LengthTarget {
    min: 0,
    opt: SKEW_UNCONSTRAINED,
    max: SKEW_UNCONSTRAINED,
  });

  LengthTarget {
    min: coupled_length + skew.min,
    opt: coupled_length + skew.opt,
    max: coupled_length + skew.max,
  }
}
