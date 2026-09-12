// SPDX-License-Identifier: GPL-3.0-or-later

//! Interactive placement of a differential pair.
//!
//! Port of `PNS::DIFF_PAIR_PLACER`
//! (`pcbnew/router/pns_diff_pair_placer.h:52`,
//! `pcbnew/router/pns_diff_pair_placer.cpp`), the state machine behind
//! "click one half of a pair, drag, click again". The reference note is
//! `doc/reference/kicad/07-differential-pairs.md` section 5, and the
//! value layer underneath it is [`crate::diff_pair`].
//!
//! # How it differs from the single track placer
//!
//! [`crate::placer::line_placer::LinePlacer`] is the model for the state
//! and for the node handling, but the pair placer is much smaller: there
//! is no tail, no head, no mouse trail solver, no fixed tail stack and no
//! direction member (note 07 section 5.1). Every move rebuilds the whole
//! pair from the two ends: [`DpPlacing::route_head`] asks
//! [`crate::diff_pair::DpGateways`] for the exits of the pair's start and
//! of whatever is under the cursor, and [`crate::diff_pair::fit_gateways`]
//! joins them. Posture is one boolean and a projection test rather than a
//! trail with hysteresis.
//!
//! `UnfixRoute` is **not overridden** in KiCad
//! (`pcbnew/router/pns_placement_algo.h:81` answers `std::nullopt`), so
//! backspace does nothing during pair placement there and
//! [`DiffPairPlacer::undo_last_segment`] answers [`None`] here.
//!
//! # The three modes
//!
//! [`DpPlacing::rh_mark_obstacles`] fits and collision tests.
//! [`DpPlacing::rh_walk_only`] fits and then walks one lane around each
//! obstacle while shoving the other lane along at the pair's own gap,
//! which is what keeps the two coupled ([`DpPlacing::attempt_walk`]).
//! [`DpPlacing::rh_shove_only`] does the same against solids only and
//! then puts both lanes into the shove as two heads. There is no fallback
//! from shove to walk, where the single placer has one, and the pair
//! placer never locks a springback frame (note 07 section 11.2).
//!
//! # Deviations, each documented at its line
//!
//! - Erratum E3: `attemptWalk`'s winding flag is never read, so
//!   `tryWalkDp`'s four attempts are two distinct computations run twice.
//!   Two run here, which keeps the tie break (the first attempt wins) and
//!   halves the work.
//! - Erratum E8: the coupled item search tie breaks on `(distance, uid)`
//!   rather than on allocation order.
//! - Errata E9, E11, E12: transcribed as they stand.
//! - Errata E2 and E13: `setInitialDirection` is declared and never
//!   defined, and `m_orthoMode` is written and never read; neither is
//!   ported, so [`DiffPairPlacer`] has no ortho mode command.
//! - Every leg is committed to the board by KiCad's `FixRoute`
//!   (`pcbnew/router/pns_diff_pair_placer.cpp:863`); here a fixed leg
//!   stays in the node it was written into and the whole session is
//!   committed once, which is what the facade's
//!   [`crate::router::CommitDiff`] contract needs. See
//!   [`DiffPairPlacer::fix_route`].

use std::collections::BTreeSet;

use crate::algo_base::AlgoContext;
use crate::collide::CollisionSearchOptions;
use crate::diff_pair::{
  DiffPair, DpGateways, DpPrimitive, DpPrimitivePair, fit_gateways,
};
use crate::geometry::direction45::{AngleType, Direction45};
use crate::geometry::hull::HULL_MARGIN;
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::Vec2;
use crate::item::{Item, ItemBody, ItemId, Kind, LayerRange, NetId, Via};
use crate::line::Line;
use crate::node::{NodeId, World};
use crate::optimizer::Optimizer;
use crate::rules::{ItemRef, RuleResolver};
use crate::settings::{RouterMode, Sizes};
use crate::shove::{Shove, ShovePolicy, ShoveStatus};
use crate::topology;
use crate::via::move_via_by;
use crate::walkaround::{WalkPolicy, Walkaround, WalkaroundStatus};

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

/// The uid of the throwaway item a clearance query is asked about.
///
/// The same convention as `src/shove.rs`: a [`Line`] has no [`Item`] of
/// its own, so [`Line::rule_item`] builds one per query and it never
/// enters the arena.
const PROBE_UID: u64 = u64::MAX;

/// How far [`DpPlacing::propagate_dp_head_forces`] walks the virtual
/// head.
///
/// `maxIter`, `pcbnew/router/pns_diff_pair_placer.cpp:150`.
pub const HEAD_PUSHOUT_ITERATIONS: u32 = 40;

/// How many walk plus shove rounds [`DpPlacing::attempt_walk`] runs.
///
/// The `while( iter < 3 )` of
/// `pcbnew/router/pns_diff_pair_placer.cpp:268`.
pub const ATTEMPT_WALK_ROUNDS: u32 = 3;

/// How many lane orders [`DpPlacing::try_walk_dp`] tries.
///
/// KiCad's loop runs four times (`:284`) but the second bit only feeds
/// `aWindCw`, which `attemptWalk` never reads, so attempts 2 and 3
/// repeat attempts 0 and 1 and lose the strict `score < bestScore`
/// comparison to them. Erratum E3.
pub const WALK_ATTEMPTS: u32 = 2;

/// What one nanometre of skew costs against one nanometre of coupling in
/// [`DpPlacing::try_walk_dp`]'s score.
///
/// The `fabs( skew ) * 3.0` of
/// `pcbnew/router/pns_diff_pair_placer.cpp:297`.
pub const WALK_SKEW_WEIGHT: i64 = 3;

// ---------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------

/// Why a pair could not be identified at a point.
///
/// The three failures of `FindDpPrimitivePair`
/// (`pcbnew/router/pns_diff_pair_placer.cpp:526`, `:543`, `:598`), which
/// KiCad reports as translated strings through
/// `ROUTER::SetFailureReason`. [`crate::router::StartError`] carries them
/// on to a host.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum PairError {
  /// The rule resolver does not know the item as half of a pair.
  ///
  /// `:526`, "Unable to find complementary differential pair nets."
  NotADiffPair,

  /// The item has no free end to start from.
  ///
  /// `:543`, "Can't find a suitable starting point.". Only a segment or
  /// an arc can fail this way: a pad and a via are anchored at their
  /// centre whatever else touches them.
  NoDanglingAnchor,

  /// Nothing on the coupled net matches the item.
  ///
  /// `:598`, "Can't find a suitable starting point for coupled net". The
  /// candidate has to be the same kind of object, has to have a free end
  /// of its own, and, for a pad or a via, has to span the same layers.
  NoCoupledItem(NetId),
}

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

/// Where a pair placer is in its lifecycle.
///
/// KiCad has one boolean, `m_idle`
/// (`pcbnew/router/pns_diff_pair_placer.h:280`), plus the nullness of
/// three node pointers. This is the same split
/// [`crate::placer::line_placer::PlacerState`] makes, for the reason note
/// 03 section 9.1 gives: several of the combinations those flags can take
/// are unreachable.
///
/// `m_state` (`RT_START`, `RT_ROUTE`, `RT_FINISH`, `:223`) has no
/// counterpart: the constructor writes it and nothing ever reads it
/// (note 07 section 5.1).
#[derive(Clone, Debug)]
pub enum DpPlacerState {
  /// Nothing is being placed. The layer a placement would start on is
  /// carried, which is what `SetLayer`'s idle branch stores (`:439`).
  Idle {
    /// The copper layer [`DiffPairPlacer::start`] would place on.
    layer: i32,
  },
  /// A pair is being placed.
  Placing(Box<DpPlacing>),
  /// The session ended with a terminal fix.
  Finished {
    /// Whether anything reached a node, for
    /// [`DiffPairPlacer::has_placed_anything`].
    placed_anything: bool,
  },
}

impl DpPlacerState {
  /// The live placement, when there is one.
  pub fn placing(&self) -> Option<&DpPlacing> {
    match self {
      DpPlacerState::Placing(placing) => Some(placing),
      DpPlacerState::Idle { .. } | DpPlacerState::Finished { .. } => None,
    }
  }

  /// The live placement, mutably.
  pub fn placing_mut(&mut self) -> Option<&mut DpPlacing> {
    match self {
      DpPlacerState::Placing(placing) => Some(placing),
      DpPlacerState::Idle { .. } | DpPlacerState::Finished { .. } => None,
    }
  }
}

/// The live state of one pair placement.
///
/// The members of `DIFF_PAIR_PLACER` that only mean anything while
/// `m_idle` is false (`pcbnew/router/pns_diff_pair_placer.h:217` to
/// `:281`). The eight dead ones note 07 section 5.1 lists are not
/// carried: `m_state`, `m_iteration`, `m_p_start`, `m_startsOnVia`,
/// `m_orthoMode`, `m_viaDiameter`, `m_viaDrill` and `m_currentWidth`.
#[derive(Clone, Debug)]
pub struct DpPlacing {
  /// Whether a fixed leg pinned the layer. Port of
  /// `m_chainedPlacement` (`:225`), read by
  /// [`DiffPairPlacer::set_layer`].
  chained: bool,
  /// The posture of this leg. Port of `m_startDiagonal` (`:227`), which
  /// every gateway call takes and which `FlipPosture` toggles.
  start_diagonal: bool,
  /// Whether the last route produced a collision free pair. Port of
  /// `m_fitOk` (`:228`), the gate on [`DiffPairPlacer::fix_route`].
  fit_ok: bool,
  /// The positive net. Port of `m_netP` (`:230`).
  net_p: Option<NetId>,
  /// The negative net. Port of `m_netN`.
  net_n: Option<NetId>,
  /// The pair the session started on. Port of `m_start` (`:232`).
  start: DpPrimitivePair,
  /// The pair this leg starts from. Port of `m_prevPair` (`:233`), an
  /// `std::optional` there too; [`DpPlacing::route_head`] seeds it from
  /// [`DpPlacing::start`] and [`DiffPairPlacer::fix_route`] advances it.
  prev_pair: Option<DpPrimitivePair>,
  /// Whether the last route ended on a target pair rather than on the
  /// bare cursor. Port of `m_snapOnTarget` (`:272`), which decides
  /// whether a fix ends the session.
  snap_on_target: bool,
  /// Where this leg starts. Port of `m_currentStart` (`:274`).
  current_start: Vec2,
  /// Where the last move put the cursor. Port of `m_currentEnd`.
  current_end: Vec2,
  /// The pair being placed. Port of `m_currentTrace` (`:275`).
  current_trace: DiffPair,
  /// Whether any fit has succeeded during this leg. Port of
  /// `m_currentTraceOk` (`:276`); see erratum E12 on
  /// [`DpPlacing::route_head`].
  current_trace_ok: bool,
  /// What the cursor is over. Port of `m_currentEndItem` (`:278`).
  current_end_item: Option<ItemId>,
  /// Whether the next fix places a pair of vias. Port of `m_placingVia`
  /// (`:257`).
  placing_via: bool,
  /// The copper layer being routed on. Port of `m_currentLayer`
  /// (`:268`).
  layer: i32,
  /// The geometry this placement uses. Port of `m_sizes` (`:254`).
  sizes: Sizes,
  /// Whether a leg has been fixed. Port of `m_hasFixedAnything` (`:281`),
  /// which gates the connected track width preservation of
  /// `UpdateSizes`.
  has_fixed_anything: bool,
}

// ---------------------------------------------------------------------
// The routing core
// ---------------------------------------------------------------------

impl DpPlacing {
  /// The centre to centre spacing of the two lanes.
  ///
  /// Port of `gap()`, `pcbnew/router/pns_diff_pair_placer.cpp:616`, which
  /// is `DiffPairGap() + DiffPairWidth()`. It is called a pitch
  /// everywhere in [`crate::diff_pair`], because KiCad's name collides
  /// with the edge to edge gap; see that module's documentation.
  pub const fn pitch(&self) -> i32 {
    self.sizes.diff_pair_pitch()
  }

  /// The copper gap a pair of vias needs.
  ///
  /// Port of `viaGap()`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:610`, which is
  /// [`Sizes::effective_diff_pair_via_gap`].
  pub const fn via_gap(&self) -> i32 {
    self.sizes.effective_diff_pair_via_gap()
  }

  /// The geometry this placement uses.
  pub const fn sizes(&self) -> &Sizes {
    &self.sizes
  }

  /// The pair being placed.
  pub const fn current_trace(&self) -> &DiffPair {
    &self.current_trace
  }

  /// One via of the pair, at a point.
  ///
  /// Port of `makeVia`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:74`, which is the line
  /// placer's with the pair's net. The span comes off
  /// [`Sizes::via_layer_range`] and falls back to the layer being routed,
  /// exactly as [`crate::placer::line_placer::Placing::make_via`] does.
  pub fn make_via(
    &self,
    world: &mut World,
    at: Vec2,
    net: Option<NetId>,
  ) -> Item {
    let layers = self
      .sizes
      .via_layer_range()
      .unwrap_or_else(|| LayerRange::single(self.layer));
    let body = ItemBody::Via(Via::new(
      at,
      self.sizes.via_diameter,
      self.sizes.via_drill,
      self.sizes.via_type,
    ));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(layers);
    item.set_net(net);

    item
  }

  /// Walk the cursor out of whatever it is standing in.
  ///
  /// Port of `propagateDpHeadForces`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:118`. The pair is
  /// approximated by one round virtual via as wide as the whole pair,
  /// `gap + 2 * width` for tracks (`:129`) or
  /// `via gap + 2 * via diameter` for a via pair (`:124`), which the
  /// comment at `:144` admits to. In mark obstacles mode the loop is
  /// skipped and the cursor is used as it stands (`:134`), which is what
  /// makes that mode's preview follow the mouse exactly.
  ///
  /// The clearance at `:164` is resolved between the obstacle and the
  /// pair's **P line**, not between the obstacle and the virtual via,
  /// because a via's resolved clearance to an item can differ from the
  /// pair's and it is the pair's that has to be respected (`:148`).
  ///
  /// # Erratum E9, transcribed
  ///
  /// `force` is declared outside the loop (`:153`) and only replaced by a
  /// longer one (`:173`), so it is a running maximum over every obstacle
  /// and every layer, never reset. The virtual via is displaced by that
  /// running maximum on every colliding round (`:180`), so after several
  /// rounds it sits at `at` plus the **sum** of the running maxima, while
  /// the answer is `at + force`, a single application of the last one
  /// (`:192`). The two therefore disagree for any move where more than
  /// one obstacle pushes. KiCad's `totalForce` accumulates the
  /// displacements and is never read, so it is dropped.
  ///
  /// One narrow difference: KiCad's `collided` is the `|=` of what each
  /// layer's `Collide` answered, and a collision whose translation vector
  /// comes out zero still sets it. [`crate::item::Via::pushout_force`]
  /// answers [`None`] for a zero vector, so such a round counts as no
  /// collision here. It can only change the answer when the loop runs out
  /// of budget on that very round.
  pub fn propagate_dp_head_forces(
    &self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    at: Vec2,
  ) -> Option<Vec2> {
    // :120
    let mut virt_head = self.make_via(world, at, None);
    let diameter = if self.placing_via {
      // :124
      self.via_gap() + 2 * self.sizes.via_diameter
    } else {
      // :128, :129
      virt_head.set_layers_and_flash_all(LayerRange::single(self.layer));
      self.sizes.diff_pair_gap + 2 * self.sizes.diff_pair_width
    };

    let layers = virt_head.layers();

    if let ItemBody::Via(body) = virt_head.body_mut() {
      body.set_stack_mode(crate::item::StackMode::Normal);
      body.set_diameter(layers, Via::ALL_LAYERS, diameter);
    }

    // :132 to :141
    let solids_only = match context.settings.mode {
      // :134
      RouterMode::MarkObstacles => return Some(at),
      // :139
      RouterMode::Walkaround => false,
      RouterMode::Shove => true,
    };

    let options = CollisionSearchOptions {
      limit_count: Some(1),
      kind_mask: if solids_only { Kind::SOLID } else { Kind::ANY },
      ..CollisionSearchOptions::default()
    };
    let p_line = self.current_trace.p_line();
    let probe = p_line.rule_item(world, PROBE_UID);

    // :150 to :154
    let mut iteration = 0;
    let mut collided = false;
    let mut force = Vec2::new(0, 0);
    let mut handled: BTreeSet<ItemId> = BTreeSet::new();

    // :156
    while iteration < HEAD_PUSHOUT_ITERATIONS {
      // :158
      let obstacle = world.check_colliding(
        node,
        ItemRef::unstored(&virt_head),
        context.resolver,
        &options,
      );

      // :161
      let Some(id) = obstacle.and_then(|obstacle| obstacle.item) else {
        break;
      };

      if handled.contains(&id) {
        break;
      }

      let Some(other) = world.item(id) else {
        break;
      };

      // :164
      let clearance = context
        .resolver
        .clearance(
          ItemRef::stored(id, other),
          Some(ItemRef::unstored(&probe)),
          false,
        )
        .unwrap_or(-1);

      // :166 to :175. The per layer loop, with its own running maximum,
      // is `Via::pushout_force`.
      let layer_force = match virt_head.body() {
        ItemBody::Via(body) => {
          body.pushout_force(virt_head.layers(), other, clearance)
        }
        _ => None,
      };

      collided = layer_force.is_some();

      if let Some(layer_force) = layer_force
        && layer_force.squared_euclidean_norm() > force.squared_euclidean_norm()
      {
        force = layer_force;
      }

      // :177 to :181
      if collided {
        move_via_by(&mut virt_head, force);
      }

      // :183
      handled.insert(id);
      iteration += 1;
    }

    // :188 to :194
    if collided && iteration == HEAD_PUSHOUT_ITERATIONS {
      return None;
    }

    Some(at + force)
  }

  /// Fit a pair between where this leg starts and where the cursor is.
  ///
  /// Port of `routeHead`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:677`, the core of the
  /// placer. The entry gateways come from the pair this leg starts on;
  /// the target gateways come either from a pair the cursor is over, in
  /// which case the route snaps to it and the next fix ends the session,
  /// or from the cursor itself.
  ///
  /// The `lead_dist` test at `:715` is the pair's whole posture rule.
  /// `fp_proj` is the cursor projected onto the line through the start
  /// pair's midpoint along its direction of travel. A cursor further off
  /// that line than half the pitch may turn the pair (`:717`); a cursor
  /// close to it builds the gateways at the projection and then drops
  /// every gateway whose anchor line runs **along** the direction of
  /// travel (`:723`), which keeps the pair straight and ends it as near
  /// the cursor as the 45 degree regime allows.
  ///
  /// `set_fit_vias` reaches the target set only (`:712`), and only on the
  /// cursor branch, so placing a via while snapped to a target pair
  /// builds that target's gateways at the trace pitch and never expands
  /// them for the vias. The vias are appended anyway (`:745`).
  ///
  /// # Erratum E12, transcribed
  ///
  /// The failure return is `m_currentTraceOk` (`:756`): once any fit has
  /// succeeded during this leg, a later failed fit still reports success
  /// and leaves the **previous** shape in the trace, which all three mode
  /// routines then collision test, walk or shove. The visible effect is
  /// that the preview freezes at the last routable position instead of
  /// disappearing.
  ///
  /// The other half of E12 comes with it: `set_gap(pitch)` at `:731` runs
  /// unconditionally and `set_gap(edge to edge)` at `:741` only on
  /// success, so on the sticky path the trace's gap is the centre to
  /// centre pitch, which [`DpPlacing::attempt_walk`] then hands the shove
  /// as a copper gap and which [`DiffPair::coupled_segment_pairs`] then
  /// matches against as one. The lanes come out a width further apart
  /// than intended and nothing is reported as coupled until the next size
  /// event rewrites the gap.
  pub fn route_head(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    at: Vec2,
  ) -> bool {
    // :679
    self.fit_ok = false;

    let pitch = self.pitch();

    // :681, :682
    let mut entry = DpGateways::new(pitch);
    let mut target = DpGateways::new(pitch);

    // :684
    let prev_pair = *self.prev_pair.get_or_insert(self.start);

    // :687. KiCad ignores the failure, and a set with no gateway simply
    // fits nothing (erratum E10).
    let _ =
      entry.build_from_primitive_pair(world, &prev_pair, self.start_diagonal);

    // :691
    match DiffPairPlacer::find_dp_primitive_pair(
      world,
      context.resolver,
      node,
      self.current_end_item,
    ) {
      Ok(found) => {
        // :693, :694
        let _ =
          target.build_from_primitive_pair(world, &found, self.start_diagonal);
        self.snap_on_target = true;
      }
      Err(_) => {
        // :700
        let Some(forced) =
          self.propagate_dp_head_forces(world, context, node, at)
        else {
          return false;
        };

        // :704
        let Some(orientation) = prev_pair.cursor_orientation(world, forced)
        else {
          return false;
        };

        // :706
        let along = Seg::new(
          orientation.midpoint,
          orientation.midpoint + orientation.direction,
        );
        let projected = along.line_project(forced);

        // :710
        let lead_distance = (projected - forced).euclidean_norm();

        // :712
        target.set_fit_vias(
          self.placing_via,
          self.sizes.via_diameter,
          Some(self.via_gap()),
        );

        // :715
        if lead_distance > pitch / 2 {
          // :717
          target.build_for_cursor(forced);
        } else {
          // :723, :724
          target.build_for_cursor(projected);
          target.filter_by_orientation(
            AngleType::STRAIGHT | AngleType::HALF_FULL,
            Direction45::from_vector(orientation.direction, false),
          );
        }

        // :728
        self.snap_on_target = false;
      }
    }

    // :731, :732
    self.current_trace.set_gap(pitch);
    self.current_trace.set_layer(self.layer);

    // :734
    let Some(fitted) =
      fit_gateways(pitch, &entry, &target, self.start_diagonal)
    else {
      // :756, erratum E12.
      return self.current_trace_ok;
    };

    // The two writes `FitGateways` performs on the caller's pair
    // (`pcbnew/router/pns_diff_pair.cpp:373`, `:374`).
    self.current_trace.set_gap(pitch);
    self.current_trace.set_shape_from(&fitted);

    // :738 to :741
    self.current_trace_ok = true;
    self.current_trace.set_nets(self.net_p, self.net_n);
    self.current_trace.set_width(self.sizes.diff_pair_width);
    self.current_trace.set_gap(self.sizes.diff_pair_gap);

    // :743 to :751
    if self.placing_via {
      let (Some(last_p), Some(last_n)) = (
        self.current_trace.chain_p().last_point(),
        self.current_trace.chain_n().last_point(),
      ) else {
        return true;
      };
      let via_p = self.make_via(world, last_p, self.net_p);
      let via_n = self.make_via(world, last_n, self.net_n);

      self.current_trace.append_vias(via_p, via_n);
    } else {
      self.current_trace.remove_vias();
    }

    true
  }

  /// Walk one lane around what is in its way and shove the other after
  /// it.
  ///
  /// Port of `attemptWalk`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:200`. One lane is walked
  /// around its obstacles with the shortest policy, then the other lane
  /// is shoved away from the walked one at
  /// `ForceClearance( true, gap - 2 * HULL_MARGIN )` (`:247`), which is
  /// the whole coupling mechanism: the shove is told to keep the pair's
  /// own copper gap between the two lanes whatever the rule resolver
  /// would say about two different nets. Then the roles swap and the
  /// round repeats, at most [`ATTEMPT_WALK_ROUNDS`] times.
  ///
  /// The lane that does not collide is skipped without spending a round
  /// (`:230` to `:236`): the roles swap and the loop restarts. That
  /// cannot spin, because the swap is its own inverse and the second
  /// visit takes the other branch, but the loop is bounded here anyway.
  ///
  /// `aWindCw` is never read in KiCad's body and no winding preference is
  /// expressible through the one policy the walkaround is given, so the
  /// parameter is not ported. Erratum E3.
  ///
  /// The pair's gap has to be the edge to edge one for the forced
  /// clearance to mean anything, which it is after a successful
  /// [`DpPlacing::route_head`] and is not after a failed one; see
  /// erratum E12 there.
  pub fn attempt_walk(
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    current: &DiffPair,
    p_first: bool,
    solids_only: bool,
  ) -> Option<DiffPair> {
    // :203 to :207
    let mut walkaround = Walkaround::new(node, context.settings);

    walkaround.set_solids_only(solids_only);
    walkaround.set_iteration_limit(context.settings.walkaround_iteration_limit);
    walkaround.set_allowed_policies(&[WalkPolicy::Shortest]);

    // :209
    let mut shove = Shove::new(node);

    // :212, :216, :218, :220
    let mut walk = current.clone();
    let mut cur = current.clone();
    let mut current_is_p = p_first;
    let mut iteration = 0;
    let options = CollisionSearchOptions {
      limit_count: Some(1),
      kind_mask: if solids_only { Kind::SOLID } else { Kind::ANY },
      ..CollisionSearchOptions::default()
    };

    // :222. The bound is KiCad's `while( iter < 3 )` plus one pass per
    // possible lane swap, which the argument above makes at most one.
    for _ in 0..=(ATTEMPT_WALK_ROUNDS * 2) {
      if iteration >= ATTEMPT_WALK_ROUNDS {
        break;
      }

      // :224, :225
      let (pre_walk, pre_shove) = if current_is_p {
        (cur.p_line(), cur.n_line())
      } else {
        (cur.n_line(), cur.p_line())
      };

      // :228
      if world
        .check_colliding_line(node, &pre_walk, context.resolver, &options)
        .is_none()
      {
        // :230
        current_is_p = !current_is_p;

        // :232
        if world
          .check_colliding_line(node, &pre_shove, context.resolver, &options)
          .is_none()
        {
          break;
        }

        // :235, which does not spend a round.
        continue;
      }

      // :238 to :243
      let result = walkaround.route(world, context, &pre_walk);

      if result.status(WalkPolicy::Shortest) != WalkaroundStatus::Done {
        return None;
      }

      let mut post_walk = result.into_line(WalkPolicy::Shortest);

      // :247
      shove.set_force_clearance(Some(cur.gap() - 2 * HULL_MARGIN));

      // :251
      let mut post_shove = shove
        .shove_obstacle_line(world, context, node, &post_walk, &pre_shove)?;

      // :256, :257
      post_walk.chain_mut().simplify(0);
      post_shove.chain_mut().simplify(0);

      // :259
      cur.set_shape(
        post_walk.shape().clone(),
        post_shove.shape().clone(),
        !current_is_p,
      );

      // :261
      current_is_p = !current_is_p;

      // :263
      if world
        .check_colliding_line(node, &post_shove, context.resolver, &options)
        .is_none()
      {
        break;
      }

      iteration += 1;
    }

    // :270
    if iteration >= ATTEMPT_WALK_ROUNDS {
      return None;
    }

    // :273
    walk.set_shape(cur.chain_p().clone(), cur.chain_n().clone(), false);

    Some(walk)
  }

  /// Walk the pair both ways round and keep the better answer.
  ///
  /// Port of `tryWalkDp`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:279`. Each attempt runs on
  /// its own branch, so a failed walk leaves nothing behind, and the
  /// winner is optimized with [`Optimizer::optimize_diff_pair`] on a
  /// default optimizer over the routing node; no effort flag reaches the
  /// pair path (note 07 section 6).
  ///
  /// `aNode` is never read in KiCad's body, which goes through
  /// `m_currentNode` (`:287`, `:311`); both call sites pass exactly that,
  /// so the parameter is not ported. Erratum E3.
  ///
  /// # Erratum E11, transcribed
  ///
  /// Two defects in nine lines, and both are reproduced.
  ///
  /// The final test is `if( bestScore > 0.0 )` against a score that
  /// starts at `1e14` (`:282`, `:309`), so it passes even when every
  /// attempt failed: a default constructed pair with two empty chains is
  /// written into the caller's pair and the answer is `true`. Walk mode
  /// therefore reports success on a pair it could not route, and the
  /// preview and the return of `Move` both lie. Nothing wrong is
  /// committed, because `FixRoute` refuses a pair with an empty lane
  /// (`:813`).
  ///
  /// The score is `1 + coupled length + 3 * |skew|` and the comparison is
  /// `<` (`:294` to `:299`), so among successful attempts the one with
  /// the **least** coupling wins. Every other coupled length comparison
  /// in KiCad's tree maximises. KiCad's arithmetic is in `double`s over
  /// values that are whole nanometres, so an `i64` here is the same
  /// number and the same ordering.
  pub fn try_walk_dp(
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    pair: &mut DiffPair,
    solids_only: bool,
  ) -> bool {
    let mut best: Option<DiffPair> = None;
    let mut best_score = i64::MAX;

    // :284
    for attempt in 0..WALK_ATTEMPTS {
      // :287
      let scratch = world.branch(node);
      // :289
      let p_first = attempt & 1 != 0;

      // :292
      if let Some(candidate) = DpPlacing::attempt_walk(
        world,
        context,
        scratch,
        pair,
        p_first,
        solids_only,
      ) {
        // :294 to :297
        let score = 1
          + candidate.coupled_length()
          + WALK_SKEW_WEIGHT * candidate.skew().abs();

        // :299
        if score < best_score {
          best_score = score;
          best = Some(candidate);
        }
      }

      // :306
      world.drop_node(scratch);
    }

    // :309, always taken; erratum E11.
    let best = best.unwrap_or_default();
    let optimizer = Optimizer::new(node);

    // :313, :314
    pair.set_shape_from(&best);
    optimizer.optimize_diff_pair(world, context, pair);

    true
  }

  /// The mark obstacles mode routine.
  ///
  /// Port of `rhMarkObstacles`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:104`: fit, then test both
  /// lanes against the node. It answers **false** on a collision, where
  /// the single track placer answers true and lets the host paint the
  /// violations; the difference that matters is the gate on
  /// [`DiffPairPlacer::fix_route`], which refuses to commit a colliding
  /// pair unless rule violations are allowed.
  pub fn rh_mark_obstacles(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    at: Vec2,
  ) -> bool {
    if !self.route_head(world, context, node, at) {
      return false;
    }

    let options = CollisionSearchOptions {
      limit_count: Some(1),
      ..CollisionSearchOptions::default()
    };
    let collides = |line: &Line| {
      world
        .check_colliding_line(node, line, context.resolver, &options)
        .is_some()
    };

    self.fit_ok = !collides(&self.current_trace.p_line())
      && !collides(&self.current_trace.n_line());

    self.fit_ok
  }

  /// The walkaround mode routine.
  ///
  /// Port of `rhWalkOnly`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:323`: fit, then walk around
  /// everything, solids and tracks alike. Given erratum E11 the answer is
  /// always true once the fit succeeded.
  pub fn rh_walk_only(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    at: Vec2,
  ) -> bool {
    if !self.route_head(world, context, node, at) {
      return false;
    }

    let mut trace = std::mem::take(&mut self.current_trace);

    self.fit_ok =
      DpPlacing::try_walk_dp(world, context, node, &mut trace, false);
    self.current_trace = trace;

    self.fit_ok
  }

  /// The shove mode routine.
  ///
  /// Port of `rhShoveOnly`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:352`: fit, walk around the
  /// solids the shove cannot move, then put both lanes into the shove as
  /// two heads in P then N order and read the modified heads back by
  /// index. The placer never calls `SetDefaultShovePolicy`, so both lane
  /// endpoints stay pinned, and it never locks a springback frame, so a
  /// fixed leg drops the shove's history entirely (note 07 section 11.2).
  ///
  /// There is **no fallback to walk mode** on failure, where
  /// `LINE_PLACER::rhShoveOnly` ends with `return rhWalkOnly( aP )`.
  ///
  /// The `else` branch at `:397` is dropped rather than transcribed: it
  /// is labelled "bring back previous state" and assigns the pre shove
  /// copies of the two lanes back into a trace the shove never touched,
  /// so it restores what is already there.
  ///
  /// The answer carries the node the shove wants the placer to stand on,
  /// which is the same handoff note 03 section 9.3 describes for the
  /// single placer.
  pub fn rh_shove_only(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    shove: &mut Option<Shove>,
    node: NodeId,
    at: Vec2,
  ) -> (bool, NodeId) {
    // :354
    let mut node = shove.as_ref().map_or(node, Shove::current_node);

    // :356, :358
    let ok = self.route_head(world, context, node, at);

    self.fit_ok = false;

    // :360
    if !ok {
      return (false, node);
    }

    // :363
    let mut trace = std::mem::take(&mut self.current_trace);
    let walked = DpPlacing::try_walk_dp(world, context, node, &mut trace, true);

    self.current_trace = trace;

    if !walked {
      return (false, node);
    }

    // :366
    let mut line_p = self.current_trace.p_line();
    let mut line_n = self.current_trace.n_line();

    let Some(shove) = shove.as_mut() else {
      return (false, node);
    };

    // :370 to :372
    shove.clear_heads();
    shove.add_head_line(line_p.clone(), ShovePolicy::SHOVE);
    shove.add_head_line(line_n.clone(), ShovePolicy::SHOVE);

    // :374, :376
    let status = shove.run(world, context);

    node = shove.current_node();

    if status != ShoveStatus::Ok {
      return (false, node);
    }

    // :382 to :386
    if shove.heads_modified(Some(0))
      && let Some(modified) = shove.modified_head(0)
    {
      line_p = modified.clone();
    }

    if shove.heads_modified(Some(1))
      && let Some(modified) = shove.modified_head(1)
    {
      line_n = modified.clone();
    }

    // :389
    self.current_trace.set_shape(
      line_p.shape().clone(),
      line_n.shape().clone(),
      false,
    );

    // :391
    let options = CollisionSearchOptions {
      limit_count: Some(1),
      ..CollisionSearchOptions::default()
    };
    let collides = |line: &Line| {
      world
        .check_colliding_line(node, line, context.resolver, &options)
        .is_some()
    };

    if !collides(&line_p) && !collides(&line_n) {
      self.fit_ok = true;
    }

    (self.fit_ok, node)
  }

  /// Route the pair in whichever mode the settings ask for.
  ///
  /// Port of `route`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:334`, as a `match` rather
  /// than a `switch` with a `default: return false`, which note 03
  /// section 9.2 asks for: the mode set is closed.
  pub fn route(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    shove: &mut Option<Shove>,
    node: NodeId,
    at: Vec2,
  ) -> (bool, NodeId) {
    match context.settings.mode {
      RouterMode::MarkObstacles => {
        (self.rh_mark_obstacles(world, context, node, at), node)
      }
      RouterMode::Walkaround => {
        (self.rh_walk_only(world, context, node, at), node)
      }
      RouterMode::Shove => self.rh_shove_only(world, context, shove, node, at),
    }
  }
}

// ---------------------------------------------------------------------
// DiffPairPlacer
// ---------------------------------------------------------------------

/// The differential pair placement algorithm.
///
/// Port of `PNS::DIFF_PAIR_PLACER`
/// (`pcbnew/router/pns_diff_pair_placer.h:52`). See the module
/// documentation for the shape of the state and for the deviations.
#[derive(Clone, Debug)]
pub struct DiffPairPlacer {
  /// Where the placer is in its lifecycle.
  state: DpPlacerState,
  /// The node every placement branches from, KiCad's
  /// `Router()->GetWorld()` (`:663`).
  root_node: NodeId,
  /// The branch the placement runs on. Port of `m_world` (`:239`).
  world_node: NodeId,
  /// The node the algorithms route against. Port of `m_currentNode`
  /// (`:248`).
  current_node: NodeId,
  /// The per move scratch branch. Port of `m_lastNode` (`:251`).
  last_node: Option<NodeId>,
  /// The node the last fixed leg was written into. Port of
  /// `m_lastFixNode` (`:252`), which KiCad commits inside `FixRoute` and
  /// then nulls; see [`DiffPairPlacer::fix_route`] for why it outlives
  /// the fix here.
  fixed_node: Option<NodeId>,
  /// The geometry the next placement uses. Port of `m_sizes` (`:254`).
  sizes: Sizes,
  /// The posture carried across legs and across sessions. Port of
  /// `m_initialDiagonal` (`:226`), which `initPlacement` copies into the
  /// leg's own posture (`:661`) and a fix derives from the committed
  /// shape (`:817`).
  initial_diagonal: bool,
  /// The shove engine shove mode routes through. Port of `m_shove`
  /// (`:245`).
  shove: Option<Shove>,
  /// The rat line of the P lane, from
  /// [`DiffPairPlacer::update_leading_rat_line`].
  leading_rat_line_p: Option<LineChain>,
  /// The rat line of the N lane.
  leading_rat_line_n: Option<LineChain>,
}

impl DiffPairPlacer {
  /// An idle placer over one node.
  ///
  /// The constructor, `pcbnew/router/pns_diff_pair_placer.cpp:33`, plus
  /// the `SetLayer` the router performs before `Start`
  /// (`pcbnew/router/pns_router.cpp:468`); the layer starts at zero as
  /// `m_currentLayer` does (`:53`).
  ///
  /// # Panics
  ///
  /// When `node` is not a live node of `world`.
  pub fn new(world: &World, node: NodeId, sizes: Sizes) -> Self {
    assert!(
      world.node(node).is_some(),
      "a placer needs a live node to branch from"
    );

    Self {
      state: DpPlacerState::Idle { layer: 0 },
      root_node: node,
      world_node: node,
      current_node: node,
      last_node: None,
      fixed_node: None,
      sizes,
      initial_diagonal: false,
      shove: None,
      leading_rat_line_p: None,
      leading_rat_line_n: None,
    }
  }

  // -----------------------------------------------------------------
  // Accessors
  // -----------------------------------------------------------------

  /// The lifecycle state.
  pub const fn state(&self) -> &DpPlacerState {
    &self.state
  }

  /// The pair being placed, when one is.
  pub fn current_trace(&self) -> Option<&DiffPair> {
    self.state.placing().map(DpPlacing::current_trace)
  }

  /// Both lanes of the pair being placed.
  ///
  /// Port of `Traces()`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:408`, which answers an
  /// `ITEM_SET` of pointers into the trace's two cached `LINE`s, in P
  /// then N order. They are owned values here, as
  /// [`crate::placer::line_placer::LinePlacer::traces`] documents.
  pub fn traces(&self) -> Vec<Line> {
    self
      .state
      .placing()
      .map(|placing| {
        vec![
          placing.current_trace.p_line(),
          placing.current_trace.n_line(),
        ]
      })
      .unwrap_or_default()
  }

  /// Where the current leg starts. Port of `CurrentStart()`,
  /// `pcbnew/router/pns_diff_pair_placer.h:130`.
  pub fn current_start(&self) -> Option<Vec2> {
    self.state.placing().map(|placing| placing.current_start)
  }

  /// Where the last move put the cursor. Port of `CurrentEnd()`,
  /// `pcbnew/router/pns_diff_pair_placer.h:135`.
  pub fn current_end(&self) -> Option<Vec2> {
    self.state.placing().map(|placing| placing.current_end)
  }

  /// The two nets being routed.
  ///
  /// Port of `CurrentNets()`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:927`, which pushes `m_netP`
  /// then `m_netN`. `GetModifiedNets` (`:907`) answers the same two.
  pub fn current_nets(&self) -> Option<(Option<NetId>, Option<NetId>)> {
    self
      .state
      .placing()
      .map(|placing| (placing.net_p, placing.net_n))
  }

  /// The layer being routed on. Port of `CurrentLayer()`,
  /// `pcbnew/router/pns_diff_pair_placer.h:145`.
  pub const fn current_layer(&self) -> Option<i32> {
    match &self.state {
      DpPlacerState::Idle { layer } => Some(*layer),
      DpPlacerState::Placing(placing) => Some(placing.layer),
      DpPlacerState::Finished { .. } => None,
    }
  }

  /// The most recent world state.
  ///
  /// Port of `CurrentNode( bool )`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:428`: the scratch branch
  /// when there is one, the routing node otherwise. The `aLoopsRemoved`
  /// parameter is ignored there, as it is in the single placer, so it is
  /// not in the signature.
  pub fn current_node(&self) -> NodeId {
    self.last_node.unwrap_or(self.current_node)
  }

  /// The scratch branch of the last move, when there is one.
  pub const fn last_node(&self) -> Option<NodeId> {
    self.last_node
  }

  /// The node holding every leg this session has fixed.
  ///
  /// KiCad has no such accessor: its `FixRoute` commits each leg through
  /// the router as it goes (`:863`) and nulls `m_lastFixNode`. Here the
  /// legs pile up in one node and the facade commits it once; see
  /// [`DiffPairPlacer::fix_route`].
  pub const fn fixed_node(&self) -> Option<NodeId> {
    self.fixed_node
  }

  /// Whether a via pair is pending. Port of `IsPlacingVia()`,
  /// `pcbnew/router/pns_diff_pair_placer.h:160`.
  pub fn is_placing_via(&self) -> bool {
    self
      .state
      .placing()
      .is_some_and(|placing| placing.placing_via)
  }

  /// Whether anything has reached a node.
  ///
  /// Port of `HasPlacedAnything`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:889`, which is
  /// `CP().SegmentCount() > 0 || CN().SegmentCount() > 0`, an `or` over a
  /// pair that is only meaningful with both.
  pub fn has_placed_anything(&self) -> bool {
    match &self.state {
      DpPlacerState::Idle { .. } => false,
      DpPlacerState::Placing(placing) => {
        placing.current_trace.chain_p().segment_count() > 0
          || placing.current_trace.chain_n().segment_count() > 0
      }
      DpPlacerState::Finished { placed_anything } => *placed_anything,
    }
  }

  /// The rat line from the end of the P lane to what it has not reached.
  ///
  /// See [`crate::placer::line_placer::LinePlacer::leading_rat_line`] for
  /// what the two answers mean.
  pub const fn leading_rat_line(&self) -> Option<&LineChain> {
    self.leading_rat_line_p.as_ref()
  }

  /// The same for the N lane.
  pub const fn leading_rat_line_n(&self) -> Option<&LineChain> {
    self.leading_rat_line_n.as_ref()
  }

  /// The geometry the next placement uses.
  pub const fn sizes(&self) -> &Sizes {
    &self.sizes
  }

  // -----------------------------------------------------------------
  // Finding a pair
  // -----------------------------------------------------------------

  /// Which end of an object a route may start from.
  ///
  /// Port of the free function `getDanglingAnchor`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:462`. A pad and a via answer
  /// with their centre whatever else touches them; a segment answers with
  /// whichever of its two ends carries a joint with exactly one link,
  /// that is, with an end nothing else connects to, which is why KiCad's
  /// error message tells the user to click at the end of an existing
  /// pair.
  ///
  /// The `LINE_T` branch (`:466`) has no counterpart: a
  /// [`crate::line::Line`] is a value here and never an arena item.
  ///
  /// The `ARC_T` branch (`:479`) is the `SEGMENT_T` branch below it with
  /// the arc's two endpoints in place of the segment's, so both are one
  /// arm here. It answers the arc's own `GetP0` or `GetP1` rather than the
  /// anchor it tested, which is the same point: an arc's two anchors are
  /// its two endpoints (`pcbnew/router/pns_arc.h:100`).
  pub fn dangling_anchor(
    world: &World,
    node: NodeId,
    id: ItemId,
  ) -> Option<Vec2> {
    let item = world.item(id)?;

    // :475
    if item.of_kind(Kind::VIA | Kind::SOLID) {
      return Some(item.anchor(0));
    }

    // :479, :493
    let ends = match item.body() {
      ItemBody::Segment(body) => (body.seg().a, body.seg().b),
      ItemBody::Arc(body) => (body.arc().start(), body.arc().end()),
      _ => return None,
    };
    let layer = item.layers().start();
    let net = item.net();
    let single_link = |at: Vec2| {
      world
        .find_joint(node, at, layer, net)
        .and_then(|joint| world.joint(joint))
        .is_some_and(|joint| joint.link_count(world.items(), Kind::ANY) == 1)
    };

    if single_link(item.anchor(0)) {
      Some(ends.0)
    } else if single_link(item.anchor(1)) {
      Some(ends.1)
    } else {
      None
    }
  }

  /// The two objects a pair route starts or ends on, at a point.
  ///
  /// Port of `FindDpPrimitivePair`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:515`, a public static in
  /// KiCad too so that the router's start gate can call it before any
  /// placer exists (`pcbnew/router/pns_router.cpp:352`). The rule
  /// resolver names the two nets, the item names which half it is, and
  /// the nearest object of the coupled net that matches it becomes the
  /// other half.
  ///
  /// Three properties of the search a port has to keep. The **kind
  /// equality** test (`:559`) means a pad can only pair with a pad and a
  /// segment only with a segment, which is what keeps
  /// `BuildFromPrimitivePair`'s unhandled mixed case unreachable
  /// (erratum E10). The **layer equality** test (`:570`) applies to pads
  /// and vias only, so two segments on different layers can pair. And the
  /// answer is always normalised so that the P half is the positive net,
  /// through the `refNet != netP` test at `:580`, which is why the placer
  /// needs no `dp_net_polarity`.
  ///
  /// KiCad's `aP` argument is never read in the body, so it is not in the
  /// signature.
  ///
  /// # Erratum E8
  ///
  /// KiCad scans a `std::set<ITEM*>` with a strict `dist < bestDist`, so
  /// two candidates exactly equidistant from the reference anchor are
  /// resolved by allocation address. [`World::all_items_in_net`] answers
  /// in uid order and the comparison stays strict, so the tie goes to the
  /// smaller uid, which is `DESIGN.md` section 8's rule.
  ///
  /// # Errors
  ///
  /// The three of [`PairError`].
  pub fn find_dp_primitive_pair(
    world: &World,
    resolver: &dyn RuleResolver,
    node: NodeId,
    item: Option<ItemId>,
  ) -> Result<DpPrimitivePair, PairError> {
    // :520
    let reference = item
      .and_then(|id| world.item(id).map(|item| (id, item)))
      .ok_or(PairError::NotADiffPair)?;
    let (reference_id, reference_item) = reference;
    let (net_p, net_n) = resolver
      .dp_net_pair(ItemRef::stored(reference_id, reference_item))
      .ok_or(PairError::NotADiffPair)?;

    // :533, :534
    let reference_net = reference_item.net();
    let coupled_net = if reference_net == Some(net_p) {
      net_n
    } else {
      net_p
    };

    // :536
    let reference_anchor = Self::dangling_anchor(world, node, reference_id)
      .ok_or(PairError::NoDanglingAnchor)?;

    // :553
    let candidates = world.all_items_in_net(node, Some(coupled_net), Kind::ANY);
    let mut best: Option<(i64, DpPrimitivePair)> = None;

    // :557
    for candidate_id in candidates {
      let Some(candidate) = world.item(candidate_id) else {
        continue;
      };

      // :559
      if candidate.kind() != reference_item.kind() {
        continue;
      }

      // :561
      let Some(anchor) = Self::dangling_anchor(world, node, candidate_id)
      else {
        continue;
      };

      // :566
      let distance = (anchor - reference_anchor).squared_euclidean_norm();

      // :570
      if candidate.of_kind(Kind::SOLID | Kind::VIA)
        && candidate.layers() != reference_item.layers()
      {
        continue;
      }

      // :575, strict, so the first candidate in uid order keeps a tie.
      if best.is_some_and(|(seen, _)| distance >= seen) {
        continue;
      }

      // :580 to :591
      let found = if reference_net != Some(net_p) {
        DpPrimitivePair::from_primitives(
          DpPrimitive::Stored(candidate_id),
          DpPrimitive::Stored(reference_id),
          anchor,
          reference_anchor,
        )
      } else {
        DpPrimitivePair::from_primitives(
          DpPrimitive::Stored(reference_id),
          DpPrimitive::Stored(candidate_id),
          reference_anchor,
          anchor,
        )
      };

      best = Some((distance, found));
    }

    // :595
    best
      .map(|(_, pair)| pair)
      .ok_or(PairError::NoCoupledItem(coupled_net))
  }

  // -----------------------------------------------------------------
  // Start
  // -----------------------------------------------------------------

  /// Begin placing a pair.
  ///
  /// Port of `Start`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:622`. The pair the session
  /// starts on is found on the **root**, before the branch exists
  /// (`:627`), so its anchors describe the board as the user sees it.
  ///
  /// Unlike `LINE_PLACER::Start` there is no snapping and no splitting: a
  /// pair started in the middle of two existing tracks does not break
  /// them.
  ///
  /// # Errors
  ///
  /// Whatever [`DiffPairPlacer::find_dp_primitive_pair`] refuses.
  pub fn start(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    start_item: Option<ItemId>,
  ) -> Result<(), PairError> {
    let DpPlacerState::Idle { layer } = self.state else {
      return Err(PairError::NotADiffPair);
    };

    // :626, :627, :631
    let start = Self::find_dp_primitive_pair(
      world,
      context.resolver,
      self.root_node,
      start_item,
    )?;

    // :637, :638
    let net_p = start
      .prim_p()
      .stored()
      .and_then(|id| world.item(id))
      .and_then(Item::net);
    let net_n = start
      .prim_n()
      .stored()
      .and_then(|id| world.item(id))
      .and_then(Item::net);

    // :640 to :648
    let mut current_trace = DiffPair::new();

    current_trace.set_nets(net_p, net_n);
    self.fixed_node = None;

    let placing = DpPlacing {
      chained: false,
      start_diagonal: self.initial_diagonal,
      fit_ok: false,
      net_p,
      net_n,
      start,
      prev_pair: None,
      snap_on_target: false,
      current_start: at,
      current_end: at,
      current_trace,
      current_trace_ok: false,
      current_end_item: None,
      placing_via: false,
      layer,
      sizes: self.sizes.clone(),
      has_fixed_anything: false,
    };

    self.state = DpPlacerState::Placing(Box::new(placing));

    // :650
    self.init_placement(world, self.root_node, true);

    Ok(())
  }

  /// Branch a fresh world for a leg.
  ///
  /// Port of `initPlacement`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:656`: reset the per leg
  /// state, kill the branches hanging off the placement root, branch a
  /// new one and build a shove over a branch of it.
  ///
  /// # Deviation
  ///
  /// KiCad always branches the **router's root** and always kills its
  /// children, because a fixed leg has already been committed into that
  /// root by the time `initPlacement` runs again (`:863`). The legs are
  /// not committed here until the session ends, so the second and later
  /// legs branch the node the last fix wrote into and its children are
  /// the ones that are killed. Branching the root instead would destroy
  /// the fixed geometry.
  fn init_placement(&mut self, world: &mut World, from: NodeId, kill: bool) {
    // :658 to :661
    if let Some(placing) = self.state.placing_mut() {
      placing.current_end_item = None;
      placing.start_diagonal = self.initial_diagonal;
    }

    // :665, :666
    if kill {
      world.kill_children(from);
    }

    let branch = world.branch(from);

    // :668 to :673
    self.world_node = branch;
    self.current_node = branch;
    self.last_node = None;
    self.shove = Some(Shove::new(world.branch(branch)));
  }

  // -----------------------------------------------------------------
  // Move
  // -----------------------------------------------------------------

  /// Re-route the pair to a point.
  ///
  /// Port of `Move`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:760`: drop the scratch
  /// branch, route from scratch, branch a new scratch node and recompute
  /// the two rat lines. There is no tail to reduce and no mouse trail to
  /// feed, which is why it is so much shorter than the single placer's.
  ///
  /// The answer is what the mode routine said, which is "the pair fits
  /// here". An idle placer answers false, where KiCad dereferences a null
  /// node (`:771`); its router gates the call on the session state.
  pub fn move_to(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    end_item: Option<ItemId>,
  ) -> bool {
    if self.state.placing().is_none() {
      return false;
    }

    // :765
    if let Some(last) = self.last_node.take() {
      world.drop_node(last);
    }

    let mut shove = self.shove.take();
    let node = self.current_node;
    let (routed, node) = {
      let placing = self
        .state
        .placing_mut()
        .expect("the state was checked above");

      // :762, :763
      placing.current_end_item = end_item;
      placing.fit_ok = false;

      // :768
      let answer = placing.route(world, context, &mut shove, node, at);

      // :774
      placing.current_end = at;

      answer
    };

    self.shove = shove;

    // :770, :771
    self.current_node = node;
    self.last_node = Some(world.branch(node));

    // :776
    self.update_leading_rat_line(world, context.resolver);

    routed
  }

  // -----------------------------------------------------------------
  // Fix
  // -----------------------------------------------------------------

  /// Pin the pair down and either finish or start another leg.
  ///
  /// Port of `FixRoute`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:808`. The answer is the
  /// single placer's contract: `true` means the session is over, `false`
  /// means a leg was written and another one begins.
  ///
  /// The last segment of each lane is dropped unless this fix ends the
  /// route, so the next leg starts from a corner the user has committed
  /// to, but the guard at `:827` needs **both** lanes to have more than
  /// one segment, so a pair where one lane came out with a single segment
  /// keeps its last segment on both.
  ///
  /// # Deviation: the leg is not committed here
  ///
  /// KiCad ends every fix with `CommitPlacement()` (`:863`), which folds
  /// the leg into the board through the router and then branches the root
  /// again. The facade here answers a host with one
  /// [`crate::router::CommitDiff`] at the end of a session
  /// ([`crate::router::FixOutcome`] has no room for a diff on the
  /// "carry on" arm), so a fixed leg stays in the node it was written
  /// into, the next leg branches that node, and
  /// [`DiffPairPlacer::commit_placement`] folds the lot in one go. The
  /// geometry a session produces is the same; what differs is when the
  /// host sees it.
  ///
  /// The shove is thrown away and rebuilt over the placement world
  /// (`:861`) rather than rewound, with a comment there admitting the
  /// memory management is the reason. Since the pair placer never locks a
  /// springback frame there is nothing to rewind to, so the two are the
  /// same in effect: a fixed leg drops the shove's history.
  pub fn fix_route(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    force_finish: bool,
  ) -> bool {
    let Some(last) = self.last_node else {
      return false;
    };

    let Some(placing) = self.state.placing_mut() else {
      return false;
    };

    // :810
    if !placing.fit_ok && !context.settings.allow_drc_violations() {
      return false;
    }

    // :813
    if placing.current_trace.chain_p().segment_count() < 1
      || placing.current_trace.chain_n().segment_count() < 1
    {
      return false;
    }

    // :816, :817
    if placing.current_trace.chain_p().segment_count() > 1 {
      let chain = placing.current_trace.chain_p();
      let before_last = chain.segment(chain.segment_count() - 2);

      self.initial_diagonal =
        !Direction45::from_seg(&before_last, false).is_diagonal();
    }

    // :821 to :834
    if !placing.snap_on_target
      && !placing.current_trace.ends_with_vias()
      && !force_finish
      && !context.settings.fix_all_segments
    {
      let mut chain_p = placing.current_trace.chain_p().clone();
      let mut chain_n = placing.current_trace.chain_n().clone();

      // :827
      if chain_p.segment_count() > 1 && chain_n.segment_count() > 1 {
        chain_p.remove(chain_p.point_count() - 1);
        chain_n.remove(chain_n.point_count() - 1);
      }

      placing.current_trace.set_shape(chain_p, chain_n, false);
    }

    // :836 to :845
    let ends_with_vias = placing.current_trace.ends_with_vias();

    if ends_with_vias {
      if let (Some(via_p), Some(via_n)) = (
        placing.current_trace.via_p().cloned(),
        placing.current_trace.via_n().cloned(),
      ) {
        world.add_via(last, via_p);
        world.add_via(last, via_n);
      }

      placing.chained = false;
    } else {
      placing.chained = !placing.snap_on_target && !force_finish;
    }

    // :847 to :854. The two vias are already in the node from `:838`;
    // neither `NODE::Add( LINE& )` (`pcbnew/router/pns_node.cpp:683`) nor
    // [`World::add_line`] looks at a line's via, so nothing is stored
    // twice.
    let mut line_p = placing.current_trace.p_line();
    let mut line_n = placing.current_trace.n_line();

    world.add_line(last, &mut line_p, false);
    world.add_line(last, &mut line_n, false);

    topology::simplify_line(world, last, &line_p);
    topology::simplify_line(world, last, &line_n);

    // :856
    placing.prev_pair = placing.current_trace.ending_primitives();

    let snap_on_target = placing.snap_on_target;

    // :857
    self.fixed_node = Some(last);

    // :864
    placing.placing_via = false;

    // :867 to :875. Note what neither branch touches: `m_currentStart`
    // keeps the point the session started at for the whole session, and
    // `m_currentTraceOk` keeps whatever the last fit left it at, so
    // erratum E12's sticky success survives a fixed leg.
    if snap_on_target || force_finish {
      self.state = DpPlacerState::Finished {
        placed_anything: true,
      };
      self.last_node = Some(last);

      return true;
    }

    placing.has_fixed_anything = true;

    // :861 and :874. The shove is rebuilt by `init_placement`, over the
    // node the leg was written into rather than over the root.
    self.init_placement(world, last, false);

    false
  }

  /// Roll the placement back to the previous fix.
  ///
  /// KiCad does **not** override `UnfixRoute` in the pair placer, so
  /// `PLACEMENT_ALGO::UnfixRoute` answers `std::nullopt`
  /// (`pcbnew/router/pns_placement_algo.h:81`) and
  /// `ROUTER::UndoLastSegment` logs the event and does nothing
  /// (`pcbnew/router/pns_router.cpp:946`). Backspace therefore does
  /// nothing during pair placement, there and here.
  pub const fn undo_last_segment(&self) -> Option<Vec2> {
    None
  }

  // -----------------------------------------------------------------
  // The small commands
  // -----------------------------------------------------------------

  /// Arm or disarm the pair of vias the next fix would place.
  ///
  /// Port of `ToggleVia`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:93`, which sets the flag and
  /// re-runs the move so that the preview shows the vias, and always
  /// answers true. The re-run passes a **null** end item, so a via
  /// toggled while the cursor is over a target pair loses the snap for
  /// that frame; that is KiCad's and it is transcribed.
  pub fn toggle_via(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    enabled: bool,
  ) -> bool {
    let Some(placing) = self.state.placing_mut() else {
      return false;
    };

    // :95
    placing.placing_via = enabled;

    let at = placing.current_end;

    // :98
    self.move_to(world, context, at, None);

    true
  }

  /// Turn the pair's first corner the other way.
  ///
  /// Port of `FlipPosture`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:419`, which toggles the
  /// leg's posture and re-runs the move. There is no posture solver to
  /// resync, which is the whole difference from the single placer's.
  pub fn flip_posture(&mut self, world: &mut World, context: &AlgoContext<'_>) {
    let Some(placing) = self.state.placing_mut() else {
      return;
    };

    placing.start_diagonal = !placing.start_diagonal;

    let at = placing.current_end;

    self.move_to(world, context, at, None);
  }

  /// Change the routing layer.
  ///
  /// Port of `SetLayer`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:437`. An idle placer records
  /// the layer; a chained placement refuses (`:444`); otherwise the layer
  /// may change only when this leg starts from a cursor position with no
  /// object behind it, or from a **via pair** that reaches the requested
  /// layer (`:448`). That is the pair analogue of the single placer's
  /// rule: "switch layers" and "leave a via behind" are one decision made
  /// at the fix.
  ///
  /// The leg is then restarted from the pair it began on, which is what
  /// `m_start = *m_prevPair` plus `initPlacement` do at `:451`.
  pub fn set_layer(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    layer: i32,
  ) -> bool {
    let at = {
      let placing = match &mut self.state {
        // :439
        DpPlacerState::Idle { layer: current } => {
          *current = layer;
          return true;
        }
        DpPlacerState::Finished { .. } => return false,
        DpPlacerState::Placing(placing) => placing.as_mut(),
      };

      // :444
      if placing.chained {
        return false;
      }

      let Some(prev_pair) = placing.prev_pair else {
        return false;
      };

      // :448
      let allowed = match prev_pair.prim_p() {
        DpPrimitive::None => true,
        primitive => {
          primitive.of_kind(world, Kind::VIA)
            && primitive
              .layers(world)
              .is_some_and(|layers| layers.contains(layer))
        }
      };

      if !allowed {
        return false;
      }

      // :450, :451
      placing.layer = layer;
      placing.start = prev_pair;

      placing.current_end
    };

    // :452, :453
    let from = self.current_node;

    self.init_placement(world, from, false);
    self.move_to(world, context, at, None);

    true
  }

  /// Push new sizes into a running placement.
  ///
  /// Port of `UpdateSizes`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:782`. The guard at `:793` is
  /// the pair analogue of the single placer's: in "use connected track
  /// width" mode, once a leg has been fixed the inherited width must not
  /// revert to the netclass value.
  ///
  /// Note what `:797` does as a side effect: it rewrites the trace's gap
  /// with the edge to edge value, which closes the window erratum E12
  /// opens on [`DpPlacing::route_head`]'s sticky path.
  pub fn update_sizes(&mut self, sizes: Sizes) {
    // :784, :786
    let previous_width = self.sizes.diff_pair_width;

    self.sizes = sizes;

    let Some(placing) = self.state.placing_mut() else {
      return;
    };

    placing.sizes = self.sizes.clone();

    // :793
    if !placing.sizes.track_width_is_explicit && placing.has_fixed_anything {
      placing.sizes.diff_pair_width = previous_width;
      self.sizes.diff_pair_width = previous_width;
    }

    // :796, :797
    placing
      .current_trace
      .set_width(placing.sizes.diff_pair_width);
    placing.current_trace.set_gap(placing.sizes.diff_pair_gap);

    // :799 to :803
    if placing.current_trace.ends_with_vias() {
      placing
        .current_trace
        .set_via_diameter(placing.sizes.via_diameter);
      placing.current_trace.set_via_drill(placing.sizes.via_drill);
    }
  }

  // -----------------------------------------------------------------
  // Ending
  // -----------------------------------------------------------------

  /// Fold everything this session fixed into the board.
  ///
  /// Port of `CommitPlacement`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:895`, which commits
  /// `m_lastFixNode` through the router and forgets its three nodes. The
  /// node committed here is the one every fixed leg was written into; see
  /// [`DiffPairPlacer::fix_route`] for why the legs are still there.
  ///
  /// A session that fixed nothing commits nothing, which is what KiCad's
  /// null `m_lastFixNode` means. It matters in shove mode: the shove's
  /// node holds everything it pushed aside, and none of that belongs on
  /// the board until a leg is fixed on top of it.
  pub fn commit_placement(&mut self, world: &mut World) -> bool {
    if let Some(node) = self.fixed_node.take() {
      world.commit(node);
    }

    self.last_node = None;
    self.current_node = self.root_node;
    self.world_node = self.root_node;

    true
  }

  /// Throw the placement away.
  ///
  /// Port of `AbortPlacement`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:881`, which kills the
  /// placement world's children and nulls the scratch node. It does not
  /// set `m_idle` there, so an aborted placer goes on claiming to be
  /// busy; the router destroys it instead
  /// (`pcbnew/router/pns_router.cpp:967`). The state is reset here, since
  /// the facade keeps its placer until the session ends.
  pub fn abort_placement(&mut self, world: &mut World) {
    let layer = self.current_layer().unwrap_or(0);

    world.kill_children(self.world_node);

    self.last_node = None;
    self.fixed_node = None;
    self.state = DpPlacerState::Idle { layer };
  }

  /// Recompute both lanes' rat lines.
  ///
  /// Port of `updateLeadingRatLine`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:914`, which runs one
  /// `TOPOLOGY` over the scratch branch and draws a rat line per lane
  /// with that lane's own net. The engine draws nothing, so the two
  /// chains are stored for the facade to read.
  fn update_leading_rat_line(
    &mut self,
    world: &mut World,
    resolver: &dyn RuleResolver,
  ) {
    self.leading_rat_line_p = None;
    self.leading_rat_line_n = None;

    let Some(node) = self.last_node else {
      return;
    };
    let Some(placing) = self.state.placing() else {
      return;
    };
    let line_p = placing.current_trace.p_line();
    let line_n = placing.current_trace.n_line();

    self.leading_rat_line_p =
      topology::leading_rat_line(world, node, resolver, &line_p);
    self.leading_rat_line_n =
      topology::leading_rat_line(world, node, resolver, &line_n);
  }
}
