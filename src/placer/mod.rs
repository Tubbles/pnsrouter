// SPDX-License-Identifier: GPL-3.0-or-later

//! The interactive placement algorithms.
//!
//! KiCad's `PLACEMENT_ALGO` (`pcbnew/router/pns_placement_algo.h:44`) is
//! the abstract base the router holds one of: the single track placer,
//! the differential pair placer and the three meander placers. Three of
//! the five are ported, [`line_placer::LinePlacer`],
//! [`diff_pair_placer::DiffPairPlacer`] and
//! [`meander_placer::MeanderPlacer`], and none implements a trait.
//! Note 03 section 9.2 asks for that: the set of placers is closed by the
//! router mode, so [`Placer`], an enum wrapping them, is a better fit than
//! a trait object, and it removes the downcast KiCad's `Finish` and
//! `ContinueFromEnd` perform on `Traces()[0]`
//! (`pcbnew/router/pns_router.cpp:579`, `:624`), which for a pair would
//! silently mean the P lane.
//!
//! [`fixed_tail`] is the undo stack of note 03 section 3.1, which
//! [`line_placer::LinePlacer::undo_last_segment`] rolls a placement back
//! through. Neither a pair nor a tuning session has such a stack; see
//! [`diff_pair_placer::DiffPairPlacer::undo_last_segment`] and
//! [`meander_placer::MeanderPlacer::undo_last_segment`].
//!
//! # What only a meander placer answers
//!
//! [`Placer::tuning_status`], [`Placer::tuning_length_result`] and
//! [`Placer::tuned_path`] are the readout a tuning session exists for, and
//! they answer [`None`] for the two routing placers. So do
//! [`Placer::meander_settings`] and the two live adjustments
//! [`Placer::amplitude_step`] and [`Placer::spacing_step`]. Note 08
//! section 11.4.

use crate::algo_base::AlgoContext;
use crate::geometry::line_chain::LineChain;
use crate::geometry::vec2::Vec2;
use crate::item::{ItemId, NetId};
use crate::line::Line;
use crate::meander::{MeanderSettings, TuningStatus};
use crate::node::{NodeId, World};
use crate::settings::Sizes;
use crate::topology::PathItem;

pub mod diff_pair_placer;
pub mod fixed_tail;
pub mod line_placer;
pub mod meander_placer;

/// Which placement algorithm a session is running.
///
/// Port of the `m_placer` slot (`pcbnew/router/pns_router.h:283`), a
/// `std::unique_ptr<PLACEMENT_ALGO>`, with the same reasoning behind the
/// enum as `crate::router::ActiveDragger` has for the draggers.
///
/// Both variants are boxed because both carry a [`crate::shove::Shove`],
/// so an unboxed enum would be as large as the bigger of them wherever a
/// [`crate::router::Router`] is moved.
///
/// The methods below are the part of `PLACEMENT_ALGO`
/// (`pcbnew/router/pns_placement_algo.h:44`) the facade actually calls.
/// Where the two placers disagree, the difference is on the method.
#[derive(Clone, Debug)]
pub enum Placer {
  /// One track. `LINE_PLACER`.
  Line(Box<line_placer::LinePlacer>),
  /// Two coupled tracks. `DIFF_PAIR_PLACER`.
  DiffPair(Box<diff_pair_placer::DiffPairPlacer>),
  /// One track, lengthened to a target. `MEANDER_PLACER`.
  ///
  /// `DP_MEANDER_PLACER` and `MEANDER_SKEW_PLACER` are the two variants
  /// still missing from KiCad's set of five; they arrive with the second
  /// half of milestone 11.
  Meander(Box<meander_placer::MeanderPlacer>),
}

impl Placer {
  /// Whether this is a differential pair placement.
  ///
  /// KiCad answers the same question with `ROUTER::Mode()`
  /// (`pcbnew/router/pns_router.h:171`), a field its host sets before
  /// `StartRouting`; the placer that was built carries it here.
  pub const fn is_diff_pair(&self) -> bool {
    matches!(self, Placer::DiffPair(_))
  }

  /// Whether this is a length tuning session.
  ///
  /// KiCad's three tuning modes (`pcbnew/router/pns_router.h:70` to
  /// `:72`) picked the placer; the placer that was built carries the
  /// answer here.
  pub const fn is_tuning(&self) -> bool {
    matches!(self, Placer::Meander(_))
  }

  /// `PLACEMENT_ALGO::Move` (`pcbnew/router/pns_placement_algo.h:59`).
  pub fn move_to(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    end_item: Option<ItemId>,
  ) -> bool {
    match self {
      Placer::Line(placer) => placer.move_to(world, context, at, end_item),
      Placer::DiffPair(placer) => placer.move_to(world, context, at, end_item),
      Placer::Meander(placer) => placer.move_to(world, context, at, end_item),
    }
  }

  /// `PLACEMENT_ALGO::FixRoute`
  /// (`pcbnew/router/pns_placement_algo.h:72`). True means the session is
  /// over.
  ///
  /// The pair placer reads neither the point nor the end item: its
  /// `FixRoute` fixes whatever the last `Move` left in the trace
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:808`).
  pub fn fix_route(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    end_item: Option<ItemId>,
    force_finish: bool,
  ) -> bool {
    match self {
      Placer::Line(placer) => {
        placer.fix_route(world, context, at, end_item, force_finish)
      }
      Placer::DiffPair(placer) => {
        placer.fix_route(world, context, force_finish)
      }
      // The meander placer reads neither the point nor the end item
      // either, and fixes whatever the last move produced
      // (`pcbnew/router/pns_meander_placer.cpp:364`).
      Placer::Meander(placer) => {
        placer.fix_route(world, at, end_item, force_finish)
      }
    }
  }

  /// `PLACEMENT_ALGO::UnfixRoute`
  /// (`pcbnew/router/pns_placement_algo.h:81`). The pair placer does not
  /// override it, so backspace does nothing during pair placement.
  pub fn undo_last_segment(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
  ) -> Option<Vec2> {
    match self {
      Placer::Line(placer) => placer.undo_last_segment(world, context),
      Placer::DiffPair(placer) => placer.undo_last_segment(),
      Placer::Meander(placer) => placer.undo_last_segment(),
    }
  }

  /// `PLACEMENT_ALGO::SetLayer`
  /// (`pcbnew/router/pns_placement_algo.h:96`).
  pub fn set_layer(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    layer: i32,
  ) -> bool {
    match self {
      Placer::Line(placer) => placer.set_layer(world, context, layer),
      Placer::DiffPair(placer) => placer.set_layer(world, context, layer),
      Placer::Meander(placer) => placer.set_layer(layer),
    }
  }

  /// `PLACEMENT_ALGO::ToggleVia`
  /// (`pcbnew/router/pns_placement_algo.h:90`).
  ///
  /// The pair placer re-runs the move so that the two vias appear in the
  /// preview at once (`pcbnew/router/pns_diff_pair_placer.cpp:98`); the
  /// single placer does not, so its host has to move first.
  pub fn toggle_via(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    enabled: bool,
  ) -> bool {
    match self {
      Placer::Line(placer) => placer.toggle_via(enabled),
      Placer::DiffPair(placer) => placer.toggle_via(world, context, enabled),
      Placer::Meander(placer) => placer.toggle_via(enabled),
    }
  }

  /// `PLACEMENT_ALGO::FlipPosture`
  /// (`pcbnew/router/pns_placement_algo.h:150`).
  pub fn flip_posture(&mut self, world: &mut World, context: &AlgoContext<'_>) {
    match self {
      Placer::Line(placer) => placer.flip_posture(),
      Placer::DiffPair(placer) => placer.flip_posture(world, context),
      Placer::Meander(placer) => placer.flip_posture(),
    }
  }

  /// `PLACEMENT_ALGO::SetOrthoMode`
  /// (`pcbnew/router/pns_placement_algo.h:143`).
  ///
  /// The pair placer stores the flag in `m_orthoMode` and nothing reads
  /// it (erratum E13), so the only effect there is the redundant `Move`
  /// its setter runs. Neither the flag nor the move is ported.
  pub fn set_ortho_mode(&mut self, ortho: bool) {
    match self {
      Placer::Line(placer) => placer.set_ortho_mode(ortho),
      Placer::DiffPair(_) | Placer::Meander(_) => {}
    }
  }

  /// `PLACEMENT_ALGO::UpdateSizes`
  /// (`pcbnew/router/pns_placement_algo.h:104`).
  pub fn update_sizes(&mut self, sizes: Sizes) {
    match self {
      Placer::Line(_) | Placer::Meander(_) => {}
      Placer::DiffPair(placer) => placer.update_sizes(sizes),
    }
  }

  /// `PLACEMENT_ALGO::CommitPlacement`
  /// (`pcbnew/router/pns_placement_algo.h:129`).
  pub fn commit_placement(&mut self, world: &mut World) -> bool {
    match self {
      Placer::Line(placer) => placer.commit_placement(world),
      Placer::DiffPair(placer) => placer.commit_placement(world),
      Placer::Meander(placer) => placer.commit_placement(world),
    }
  }

  /// `PLACEMENT_ALGO::Traces` (`pcbnew/router/pns_placement_algo.h:110`),
  /// which is one line for a single track and two for a pair, P first.
  pub fn traces(&self) -> Vec<Line> {
    match self {
      Placer::Line(placer) => placer.traces(),
      Placer::DiffPair(placer) => placer.traces(),
      Placer::Meander(placer) => placer.traces(),
    }
  }

  /// The first trace, which is the P lane for a pair.
  ///
  /// The `dynamic_cast<LINE*>( placer->Traces()[0] )` KiCad's `Finish`,
  /// `ContinueFromEnd` and `GetNearestRatnestAnchor` perform
  /// (`pcbnew/router/pns_router.cpp:579`, `:624`, `:530`), which for a
  /// pair aims the whole placement at the P net's nearest unconnected
  /// anchor and lets the N lane follow.
  pub fn trace(&self) -> Option<Line> {
    match self {
      Placer::Line(placer) => placer.trace(),
      Placer::DiffPair(placer) => placer.traces().into_iter().next(),
      Placer::Meander(placer) => placer.traces().into_iter().next(),
    }
  }

  /// `PLACEMENT_ALGO::CurrentNode`
  /// (`pcbnew/router/pns_placement_algo.h:120`).
  pub fn current_node(&self, loops_removed: bool) -> NodeId {
    match self {
      Placer::Line(placer) => placer.current_node(loops_removed),
      // The pair placer ignores the flag, as the single placer does
      // (`pcbnew/router/pns_diff_pair_placer.cpp:428`).
      Placer::DiffPair(placer) => placer.current_node(),
      // The meander placer ignores it too
      // (`pcbnew/router/pns_meander_placer.cpp:60`).
      Placer::Meander(placer) => placer.current_node(),
    }
  }

  /// The scratch branch of the last move, when there is one.
  pub fn last_node(&self) -> Option<NodeId> {
    match self {
      Placer::Line(placer) => placer.last_node(),
      Placer::DiffPair(placer) => placer.last_node(),
      Placer::Meander(placer) => placer.last_node(),
    }
  }

  /// Which node a commit should fold into the board, if any.
  ///
  /// The guard of `ROUTER::CommitRouting()`
  /// (`pcbnew/router/pns_router.cpp:864`) for the single placer, and the
  /// pair placer's `m_lastFixNode` (`:857`) for the pair one, which is
  /// null until a leg is fixed. Answering [`None`] is what keeps an
  /// abandoned pair placement from committing whatever its shove pushed
  /// aside.
  pub fn commit_node(&self) -> Option<NodeId> {
    match self {
      Placer::Line(placer) => placer
        .has_placed_anything()
        .then(|| placer.last_node())
        .flatten(),
      Placer::DiffPair(placer) => placer.fixed_node(),
      // The same answer for the same reason: a tuning session that only
      // moved has taken the track it was going to tune out of its own
      // branch and put nothing back, so committing that branch would
      // delete the track (`pcbnew/router/pns_meander_placer.cpp:102`).
      Placer::Meander(placer) => placer.fixed_node(),
    }
  }

  /// `PLACEMENT_ALGO::CurrentStart`
  /// (`pcbnew/router/pns_placement_algo.h:113`).
  pub fn current_start(&self) -> Option<Vec2> {
    match self {
      Placer::Line(placer) => placer.current_start(),
      Placer::DiffPair(placer) => placer.current_start(),
      Placer::Meander(placer) => placer.current_start(),
    }
  }

  /// `PLACEMENT_ALGO::CurrentEnd`
  /// (`pcbnew/router/pns_placement_algo.h:116`).
  pub fn current_end(&self) -> Option<Vec2> {
    match self {
      Placer::Line(placer) => placer.current_end(),
      Placer::DiffPair(placer) => placer.current_end(),
      // Always the origin, never the cursor; erratum E1.
      Placer::Meander(placer) => placer.current_end(),
    }
  }

  /// The first of `PLACEMENT_ALGO::CurrentNets`
  /// (`pcbnew/router/pns_placement_algo.h:117`), which is the P net for a
  /// pair.
  pub fn current_net(&self) -> Option<NetId> {
    match self {
      Placer::Line(placer) => placer.current_net(),
      Placer::DiffPair(placer) => placer.current_nets().and_then(|nets| nets.0),
      Placer::Meander(placer) => placer.current_nets(),
    }
  }

  /// The second of `CurrentNets`, which only a pair has.
  pub fn current_net_n(&self) -> Option<NetId> {
    match self {
      Placer::Line(_) | Placer::Meander(_) => None,
      Placer::DiffPair(placer) => placer.current_nets().and_then(|nets| nets.1),
    }
  }

  /// `PLACEMENT_ALGO::CurrentLayer`
  /// (`pcbnew/router/pns_placement_algo.h:119`).
  pub fn current_layer(&self) -> Option<i32> {
    match self {
      Placer::Line(placer) => placer.current_layer(),
      Placer::DiffPair(placer) => placer.current_layer(),
      Placer::Meander(placer) => placer.current_layer(),
    }
  }

  /// `PLACEMENT_ALGO::IsPlacingVia`
  /// (`pcbnew/router/pns_placement_algo.h:100`).
  pub fn is_placing_via(&self) -> bool {
    match self {
      Placer::Line(placer) => placer.is_placing_via(),
      Placer::DiffPair(placer) => placer.is_placing_via(),
      Placer::Meander(placer) => placer.is_placing_via(),
    }
  }

  /// `PLACEMENT_ALGO::HasPlacedAnything`
  /// (`pcbnew/router/pns_placement_algo.h:126`).
  pub fn has_placed_anything(&self) -> bool {
    match self {
      Placer::Line(placer) => placer.has_placed_anything(),
      Placer::DiffPair(placer) => placer.has_placed_anything(),
      Placer::Meander(placer) => placer.has_placed_anything(),
    }
  }

  /// The rat line from the end of the first trace to what it has not
  /// reached yet.
  pub fn leading_rat_line(&self) -> Option<&LineChain> {
    match self {
      Placer::Line(placer) => placer.leading_rat_line(),
      Placer::DiffPair(placer) => placer.leading_rat_line(),
      // A meander placer never draws one: it is not going anywhere.
      Placer::Meander(_) => None,
    }
  }

  /// The same for the N lane, which only a pair has.
  ///
  /// `updateLeadingRatLine` draws one per lane
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:914`).
  pub fn leading_rat_line_n(&self) -> Option<&LineChain> {
    match self {
      Placer::Line(_) | Placer::Meander(_) => None,
      Placer::DiffPair(placer) => placer.leading_rat_line_n(),
    }
  }

  // -----------------------------------------------------------------
  // Length tuning
  // -----------------------------------------------------------------

  /// `MEANDER_PLACER_BASE::TuningStatus`
  /// (`pcbnew/router/pns_meander_placer_base.h:76`), which only a tuning
  /// session has.
  pub fn tuning_status(&self) -> Option<TuningStatus> {
    match self {
      Placer::Line(_) | Placer::DiffPair(_) => None,
      Placer::Meander(placer) => placer.tuning_status(),
    }
  }

  /// `MEANDER_PLACER_BASE::TuningLengthResult` (`:61`), a length here and
  /// a skew for the skew placer that arrives with the next slice.
  pub fn tuning_length_result(&self) -> Option<i64> {
    match self {
      Placer::Line(_) | Placer::DiffPair(_) => None,
      Placer::Meander(placer) => placer.tuning_length_result(),
    }
  }

  /// `MEANDER_PLACER_BASE::TuningLengthDelta` (`:70`), [`None`] when
  /// `HasBaseline()` (`:68`) is false.
  pub fn tuning_length_delta(&self) -> Option<i64> {
    match self {
      Placer::Line(_) | Placer::DiffPair(_) => None,
      Placer::Meander(placer) => placer.tuning_length_delta(),
    }
  }

  /// `MEANDER_PLACER_BASE::TunedPath` (`:125`), the run of copper the
  /// tuned length is measured over, which the host draws as a highlight.
  pub fn tuned_path(&self) -> Option<&[PathItem]> {
    match self {
      Placer::Line(_) | Placer::DiffPair(_) => None,
      Placer::Meander(placer) => Some(placer.tuned_path()),
    }
  }

  /// `MEANDER_PLACER_BASE::MeanderSettings`
  /// (`pcbnew/router/pns_meander_placer_base.cpp:301`).
  pub const fn meander_settings(&self) -> Option<&MeanderSettings> {
    match self {
      Placer::Line(_) | Placer::DiffPair(_) => None,
      Placer::Meander(placer) => Some(placer.meander_settings()),
    }
  }

  /// `MEANDER_PLACER_BASE::UpdateSettings` (`:131`). Ignored by the two
  /// routing placers, which have no meander settings.
  pub const fn update_meander_settings(&mut self, settings: MeanderSettings) {
    match self {
      Placer::Line(_) | Placer::DiffPair(_) => {}
      Placer::Meander(placer) => placer.set_settings(settings),
    }
  }

  /// `MEANDER_PLACER_BASE::AmplitudeStep` (`:96`).
  pub fn amplitude_step(&mut self, sign: i32) {
    match self {
      Placer::Line(_) | Placer::DiffPair(_) => {}
      Placer::Meander(placer) => placer.amplitude_step(sign),
    }
  }

  /// `MEANDER_PLACER_BASE::SpacingStep` (`:105`).
  pub fn spacing_step(&mut self, sign: i32) {
    match self {
      Placer::Line(_) | Placer::DiffPair(_) => {}
      Placer::Meander(placer) => placer.spacing_step(sign),
    }
  }
}
