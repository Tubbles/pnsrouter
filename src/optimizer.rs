// SPDX-License-Identifier: GPL-3.0-or-later

//! Making a routed line shorter and less cornery after the fact.
//!
//! Port of `PNS::OPTIMIZER` (`pcbnew/router/pns_optimizer.h:94`,
//! `pcbnew/router/pns_optimizer.cpp`). Every head routine hands its
//! result here before the user sees it, and the shove runs the whole
//! queue through it after a successful push
//! (`pcbnew/router/pns_shove.cpp:2022`).
//!
//! # The passes
//!
//! [`Optimizer::optimize`] is a fixed sequence of independent passes,
//! each gated on a bit of [`EffortFlags`] and each OR-ing into the
//! answer:
//!
//! - [`Optimizer::merge_full`] ([`EffortFlags::MERGE_SEGMENTS`]) tries to
//!   replace a whole span of the line with one of the two canonical two
//!   segment 45 degree bypasses, coarse to fine.
//! - [`Optimizer::merge_obtuse`] ([`EffortFlags::MERGE_OBTUSE`]) folds a
//!   run of obtuse corners into the single corner where the two outer
//!   segments' infinite lines cross.
//! - [`Optimizer::merge_colinear`] ([`EffortFlags::MERGE_COLINEAR`])
//!   drops the shared vertex of two collinear segments.
//! - [`Optimizer::run_smart_pads`] ([`EffortFlags::SMART_PADS`]) redraws
//!   the first few segments at each end so that the trace leaves its pad
//!   along one of that pad's [`compute_breakouts`].
//! - [`Optimizer::fanout_cleanup`] ([`EffortFlags::FANOUT_CLEANUP`])
//!   redraws a very short pad to pad connection as a plain two segment
//!   trace.
//!
//! The last two are the only passes that read the world's connectivity
//! rather than only its obstacles: both start from
//! [`find_pad_or_via`], a joint lookup at each endpoint.
//!
//! Everything a pass proposes is checked against the live node
//! ([`Optimizer::check_colliding`]) and against the caller's constraints
//! ([`Optimizer::check_constraints`]), except [`Optimizer::merge_obtuse`],
//! which checks collision only. That asymmetry is KiCad's and it has a
//! consequence worth knowing: the shove's lowest effort level uses
//! [`EffortFlags::MERGE_OBTUSE`] alone
//! (`pcbnew/router/pns_shove.cpp:2046`) while always asking for
//! [`EffortFlags::RESTRICT_AREA`] (`:2072`), so on that setting the area
//! restriction is silently not enforced.
//!
//! # What decides
//!
//! [`CostEstimator::corner_cost`] and nothing else. It is a pure lookup
//! on the angle class of each corner, and a collinear joint still costs
//! something, so the estimator prefers fewer vertices even when the shape
//! is unchanged. Length enters exactly one live decision in KiCad's
//! tree, the breakout tie break of [`Optimizer::smart_pads_single`].
//!
//! # What is not here
//!
//! - **The collision cache.** `m_cache`, `m_cacheTags`, `cacheAdd`,
//!   `ClearCache`, `MaxCachedItems` and the `CACHE_VISITOR` are all dead
//!   in KiCad: nothing populates the cache, the visitor is constructed
//!   and discarded, and `ClearCache( true )` would erase from a map while
//!   iterating it (note 04 section 4.3). Every candidate is checked
//!   against the live node here, exactly as KiCad actually does.
//! - **Breakouts for a compound pad.** KiCad's `computeBreakouts` has no
//!   case for `SH_COMPOUND` (`pcbnew/router/pns_optimizer.cpp:1078`), so
//!   a complex pad offers no exits and gets no smart connection. See
//!   [`compute_breakouts`].
//! - **The two no op constraints.** `RESTRICT_VERTEX_RANGE_CONSTRAINT`
//!   returns true unconditionally and ignores both its bounds
//!   (`pcbnew/router/pns_optimizer.cpp:279`,
//!   `pcbnew/router/pns_optimizer.h:318`), and
//!   `CORNER_COUNT_LIMIT_CONSTRAINT` computes a corner count and then
//!   returns true on both branches (`:287` to `:303`). Their flags are
//!   accepted and add no constraint.
//! - **`ANGLE_CONSTRAINT_45`**, declared with no definition anywhere
//!   (`pcbnew/router/pns_optimizer.h:244`).
//! - **`COST_ESTIMATOR::Add` / `Remove` / `Replace` / `IsBetter`** and
//!   the whole `m_lengthCost` running total, which nothing calls
//!   (note 04 section 4.9).
//! - **`Tighten` and its helpers**, which nothing calls.
//! - **`checkDpColliding`** (`pcbnew/router/pns_optimizer.cpp:1426`),
//!   which has no caller either and whose two lines
//!   [`Optimizer::verify_dp_bypass`] does inline; note 07 erratum E15.
//!
//! # The differential pair path
//!
//! [`Optimizer::optimize_diff_pair`] is a second entry point with
//! nothing in common with the one above: one pass,
//! [`Optimizer::merge_dp_segments`], no effort flags, no constraints and
//! no cost estimator. What decides there is coupled length, and the
//! reference note is `doc/reference/kicad/07-differential-pairs.md`
//! section 6.
//!
//! # The drag only pass
//!
//! [`EffortFlags::REQUIRE_OBTUSE_ANGLES`] selects two things at once,
//! [`Constraint::ObtuseOnly`] and [`Optimizer::drag_fix_corners`], and
//! only a drag ever asks for it: `DRAGGER::optimizeAndUpdateDraggedLine`
//! under `GetRestrictAngles()` (`pcbnew/router/pns_dragger.cpp:583`) and
//! `ROUTER_TOOL::OptimizeSelected` for the same reason. It is the one
//! pass that runs **before** [`Optimizer::merge_full`]
//! (`pcbnew/router/pns_optimizer.cpp:709`), because it rewrites the
//! corner the drag anchored on and every later pass then works on the
//! rewritten line.
//!
//! # No singleton
//!
//! `mergeStep` reads the corner mode from
//! `ROUTER::GetInstance()->Settings()` (`pcbnew/router/pns_optimizer.cpp:858`)
//! and so does `fanoutCleanup` (`:1278`). Both become
//! [`crate::settings::RoutingSettings::corner_mode`] off the
//! [`AlgoContext`], which is what note 03 section 9.4 asks for.
//!
//! # Caller contract
//!
//! [`Optimizer::check_colliding`] wraps a candidate path in an unlinked
//! [`Line`] and asks the node about it, so the node will report the
//! **original** line's own stored segments as obstacles if they are still
//! there. Every caller in KiCad either removes the line from the node
//! first (`pcbnew/router/pns_dragger.cpp:830`,
//! `pcbnew/router/router_tool.cpp:2328`) or optimizes inside a branch
//! where the line was never added. The same applies here.

use std::ops::{BitAnd, BitOr, BitOrAssign};

use crate::algo_base::AlgoContext;
use crate::collide::CollisionSearchOptions;
use crate::diff_pair::DiffPair;
use crate::geometry::box2::Box2;
use crate::geometry::collision::collide_seg;
use crate::geometry::direction45::{AngleType, CornerMode, Direction45};
use crate::geometry::hull::approximate_segment_as_rect;
use crate::geometry::line_chain::LineChain;
use crate::geometry::math::kiround;
use crate::geometry::seg::Seg;
use crate::geometry::shape::Shape;
use crate::geometry::vec2::{Vec2, Vec2L};
use crate::item::{Item, ItemBody, ItemId, Kind, NetId, Solid};
use crate::line::Line;
use crate::node::{NodeId, World};

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

/// The cost of a collinear joint. Port of
/// `pcbnew/router/pns_optimizer.cpp:51`.
pub const COST_STRAIGHT: i32 = 5;

/// The cost of a 45 degree corner. Port of
/// `pcbnew/router/pns_optimizer.cpp:50`.
pub const COST_OBTUSE: i32 = 10;

/// The cost of a right angle corner. Port of
/// `pcbnew/router/pns_optimizer.cpp:53`.
pub const COST_RIGHT: i32 = 30;

/// The cost of a 135 degree corner. Port of
/// `pcbnew/router/pns_optimizer.cpp:52`.
pub const COST_ACUTE: i32 = 50;

/// The cost of a 180 degree hairpin. Port of
/// `pcbnew/router/pns_optimizer.cpp:54`.
pub const COST_HALF_FULL: i32 = 60;

/// The cost of a corner that is not on the 45 degree grid at all. Port of
/// `pcbnew/router/pns_optimizer.cpp:55`.
pub const COST_UNDEFINED: i32 = 100;

/// How close the preserved vertex has to be to a segment to count as
/// lying on it, as a squared distance.
///
/// Port of the `dist <= 1` tests of
/// `pcbnew/router/pns_optimizer.cpp:257` and `:271`. It is squared
/// nanometres, so this is "on the segment, give or take rounding".
pub const PRESERVE_VERTEX_TOLERANCE: i64 = 1;

/// How many postures a bypass candidate is built in.
///
/// Port of the `i < 2` loop of `pcbnew/router/pns_optimizer.cpp:881`:
/// straight leg first, then diagonal leg first.
pub const POSTURE_COUNT: usize = 2;

/// A safety net on the coarse to fine loops of [`Optimizer::merge_full`]
/// and [`Optimizer::merge_obtuse`].
///
/// KiCad has no such cap (`pcbnew/router/pns_optimizer.cpp:522`, `:601`)
/// and does not need one: a successful [`Optimizer::merge_step`] strictly
/// lowers the corner cost, a successful obtuse merge strictly lowers the
/// segment count, and a failure lowers the step, so both loops are
/// finite. `DESIGN.md` section 8 nevertheless asks for explicit budgets,
/// so the bound is written down rather than argued. It is deliberately
/// far above anything a real line can reach and is not expected to fire.
pub const MERGE_PASS_LIMIT: usize = 100_000;

// ---------------------------------------------------------------------
// Effort flags
// ---------------------------------------------------------------------

/// Which passes and constraints one [`Optimizer::optimize`] call runs.
///
/// Port of `OPTIMIZER::OptimizationEffort`,
/// `pcbnew/router/pns_optimizer.h:97`. The values are KiCad's, so a host
/// that stores an integer from KiCad's settings or passes one across an
/// FFI boundary can hand it straight to [`EffortFlags::from_bits`].
///
/// This is a newtype rather than a `bitflags` crate type because the
/// crate carries no runtime dependencies, see `DESIGN.md` section 10; it
/// follows [`crate::item::Kind`].
///
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct EffortFlags(u32);

impl EffortFlags {
  /// Do nothing at all. Not one of KiCad's names; it is the zero of the
  /// mask algebra and what [`Optimizer::set_effort_level`] is passed to
  /// disable every pass.
  pub const NONE: EffortFlags = EffortFlags(0);

  /// Run [`Optimizer::merge_full`]. Port of `MERGE_SEGMENTS = 0x01`,
  /// `pcbnew/router/pns_optimizer.h:99`.
  ///
  /// The workhorse: the line placer, the dragger, the router tool and the
  /// shove at medium and full effort all ask for this and often nothing
  /// else.
  pub const MERGE_SEGMENTS: EffortFlags = EffortFlags(0x001);

  /// Reroute the exits from the pads at each end. Port of
  /// `SMART_PADS = 0x02`, `pcbnew/router/pns_optimizer.h:100`, which
  /// selects [`Optimizer::run_smart_pads`].
  ///
  /// That pass reports "changed" unconditionally over a line of three
  /// points or more, so [`Optimizer::optimize`] does too whenever this
  /// bit is set; see [`Optimizer::run_smart_pads`] for why that is
  /// reproduced and who it bites.
  ///
  /// The line placer and the shove only ask for it in the 45 degree
  /// corner modes (`pcbnew/router/pns_line_placer.cpp:762`,
  /// `pcbnew/router/pns_shove.cpp:2079`), because the connections the
  /// pass builds are always mitered at 45 degrees whatever the mode.
  pub const SMART_PADS: EffortFlags = EffortFlags(0x002);

  /// Run [`Optimizer::merge_obtuse`]. Port of `MERGE_OBTUSE = 0x04`,
  /// `pcbnew/router/pns_optimizer.h:101`.
  pub const MERGE_OBTUSE: EffortFlags = EffortFlags(0x004);

  /// Redraw a very short pad to pad connection as a plain L. Port of
  /// `FANOUT_CLEANUP = 0x08`, `pcbnew/router/pns_optimizer.h:102`, which
  /// selects [`Optimizer::fanout_cleanup`].
  ///
  /// The line placer asks for it on its own, as the whole effort level of
  /// `optimizeTailHeadTransition` (`pcbnew/router/pns_line_placer.cpp:1046`),
  /// with a note that it can override the user's posture choice.
  pub const FANOUT_CLEANUP: EffortFlags = EffortFlags(0x008);

  /// Refuse a bypass that would swallow another net's pad. Port of
  /// `KEEP_TOPOLOGY = 0x10`, `pcbnew/router/pns_optimizer.h:103`.
  ///
  /// Nothing in KiCad's tree sets it, but the constraint behind it is
  /// real and is ported; see [`Constraint::KeepTopology`].
  pub const KEEP_TOPOLOGY: EffortFlags = EffortFlags(0x010);

  /// Refuse a bypass that would move the vertex
  /// [`Optimizer::set_preserve_vertex`] named. Port of
  /// `PRESERVE_VERTEX = 0x20`, `pcbnew/router/pns_optimizer.h:104`.
  pub const PRESERVE_VERTEX: EffortFlags = EffortFlags(0x020);

  /// Confine the optimization to a range of vertices. Port of
  /// `RESTRICT_VERTEX_RANGE = 0x40`,
  /// `pcbnew/router/pns_optimizer.h:105`.
  ///
  /// **Inert.** KiCad's constraint returns true unconditionally and its
  /// constructor drops both bounds, so the flag has never done anything;
  /// see [`Optimizer::set_restrict_vertex_range`].
  pub const RESTRICT_VERTEX_RANGE: EffortFlags = EffortFlags(0x040);

  /// Run [`Optimizer::merge_colinear`]. Port of
  /// `MERGE_COLINEAR = 0x80`, `pcbnew/router/pns_optimizer.h:106`.
  pub const MERGE_COLINEAR: EffortFlags = EffortFlags(0x080);

  /// Confine the optimization to a box. Port of
  /// `RESTRICT_AREA = 0x100`, `pcbnew/router/pns_optimizer.h:107`.
  ///
  /// The shove uses it to keep off screen geometry still
  /// (`pcbnew/router/pns_shove.cpp:2072`) and the dragger to keep the
  /// untouched part of a dragged track still
  /// (`pcbnew/router/pns_dragger.cpp:607`).
  pub const RESTRICT_AREA: EffortFlags = EffortFlags(0x100);

  /// Refuse a bypass that would take the corner count out of range. Port
  /// of `LIMIT_CORNER_COUNT = 0x200`,
  /// `pcbnew/router/pns_optimizer.h:108`.
  ///
  /// **Inert.** KiCad's constraint counts the corners and then returns
  /// true on both branches, with a "fixme: something fishy with the max
  /// corneriness limit" at `pcbnew/router/pns_optimizer.cpp:299`, and it
  /// never even stores the maximum. The flag is accepted and adds no
  /// constraint. The shove sets it on every optimize
  /// (`pcbnew/router/pns_shove.cpp:2064`) and the dragger takes it away
  /// again for via drags with a comment calling that a hack
  /// (`pcbnew/router/pns_dragger.cpp:912`); neither changes an answer.
  pub const LIMIT_CORNER_COUNT: EffortFlags = EffortFlags(0x200);

  /// Keep every corner obtuse or straight, and fix the one the drag
  /// anchored on. Port of `REQUIRE_OBTUSE_ANGLES = 0x400`,
  /// `pcbnew/router/pns_optimizer.h:110`, whose comment reads "Try to
  /// prevent 90-degree or acute corners in a drag".
  ///
  /// It selects [`Constraint::ObtuseOnly`] (`:703`) and
  /// [`Optimizer::drag_fix_corners`] (`:709`) together; there is no way
  /// to ask for one and not the other. Its two callers in KiCad's tree
  /// are both drags, and both gate it on `GetRestrictAngles()`
  /// (`pcbnew/router/pns_dragger.cpp:583`), which
  /// [`crate::settings::RoutingSettings::restrict_angles`] is.
  pub const REQUIRE_OBTUSE_ANGLES: EffortFlags = EffortFlags(0x400);

  /// The raw bits, in KiCad's numbering.
  pub const fn bits(self) -> u32 {
    self.0
  }

  /// A flag set from KiCad's integer.
  ///
  /// Every bit is kept, including ones this crate does not know, so that
  /// a host can round trip a stored effort level.
  pub const fn from_bits(bits: u32) -> Self {
    Self(bits)
  }

  /// Whether every bit of `other` is set here.
  ///
  /// The `m_effortLevel & FLAG` tests of
  /// `pcbnew/router/pns_optimizer.cpp:663` onwards, which are single bit
  /// tests at every call site.
  pub const fn contains(self, other: EffortFlags) -> bool {
    (self.0 & other.0) == other.0
  }

  /// Whether any bit of `other` is set here.
  pub const fn intersects(self, other: EffortFlags) -> bool {
    (self.0 & other.0) != 0
  }
}

/// Port of the `|` that builds an effort level, for instance
/// `pcbnew/router/pns_shove.cpp:2064`.
impl BitOr for EffortFlags {
  type Output = EffortFlags;

  fn bitor(self, other: EffortFlags) -> EffortFlags {
    EffortFlags(self.0 | other.0)
  }
}

/// Port of the `|=` of `pcbnew/router/pns_optimizer.h:141`, which is what
/// [`Optimizer::add_effort_level`] and the three setters do.
impl BitOrAssign for EffortFlags {
  fn bitor_assign(&mut self, other: EffortFlags) {
    self.0 |= other.0;
  }
}

/// Port of the `&` that masks an effort level, for instance
/// `pcbnew/router/pns_shove.cpp:2086`.
impl BitAnd for EffortFlags {
  type Output = EffortFlags;

  fn bitand(self, other: EffortFlags) -> EffortFlags {
    EffortFlags(self.0 & other.0)
  }
}

// ---------------------------------------------------------------------
// Cost estimator
// ---------------------------------------------------------------------

/// What a shape costs, in corners.
///
/// Port of `PNS::COST_ESTIMATOR`,
/// `pcbnew/router/pns_optimizer.h:48`, reduced to its three static
/// functions. The running totals (`Add`, `Remove`, `Replace`,
/// `m_lengthCost`, `m_cornerCost`) and `IsBetter` have no callers in
/// KiCad's tree and are not ported: `mergeStep` compares raw corner costs
/// (`pcbnew/router/pns_optimizer.cpp:902`) and `smartPadsSingle` does its
/// own comparison (`:1207`).
///
/// This is a namespace, not a value; it has no state to hold.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct CostEstimator;

impl CostEstimator {
  /// What the corner between two consecutive segments costs.
  ///
  /// Port of `COST_ESTIMATOR::CornerCost( const SEG&, const SEG& )`,
  /// `pcbnew/router/pns_optimizer.cpp:44`, a pure lookup on
  /// [`Direction45::angle`].
  ///
  /// The directions are built with the 90 degree flag off, KiCad's
  /// default for the one argument `DIRECTION_45( const SEG& )`
  /// constructor, so a right angle costs [`COST_RIGHT`] even when the
  /// board is being routed in a 90 degree corner mode.
  ///
  /// A collinear joint still costs [`COST_STRAIGHT`] rather than nothing.
  /// That is deliberate: it is what makes the estimator prefer fewer
  /// vertices over the same shape drawn with more of them.
  pub fn corner_cost(first: &Seg, second: &Seg) -> i32 {
    let angle = Direction45::from_seg(first, false)
      .angle(Direction45::from_seg(second, false));

    if angle == AngleType::OBTUSE {
      COST_OBTUSE
    } else if angle == AngleType::STRAIGHT {
      COST_STRAIGHT
    } else if angle == AngleType::ACUTE {
      COST_ACUTE
    } else if angle == AngleType::RIGHT {
      COST_RIGHT
    } else if angle == AngleType::HALF_FULL {
      COST_HALF_FULL
    } else {
      COST_UNDEFINED
    }
  }

  /// What every corner of a chain costs, summed.
  ///
  /// Port of `COST_ESTIMATOR::CornerCost( const SHAPE_LINE_CHAIN& )`,
  /// `pcbnew/router/pns_optimizer.cpp:60`. A chain of fewer than two
  /// segments has no corner and costs nothing.
  pub fn corner_cost_of_chain(chain: &LineChain) -> i32 {
    let segment_count = chain.segment_count();
    let mut total = 0;

    if segment_count < 2 {
      return 0;
    }

    for index in 0..segment_count - 1 {
      total +=
        Self::corner_cost(&chain.segment(index), &chain.segment(index + 1));
    }

    total
  }

  /// What every corner of a line costs, summed.
  ///
  /// Port of `COST_ESTIMATOR::CornerCost( const LINE& )`,
  /// `pcbnew/router/pns_optimizer.cpp:71`.
  pub fn corner_cost_of_line(line: &Line) -> i32 {
    Self::corner_cost_of_chain(line.shape())
  }
}

// ---------------------------------------------------------------------
// Constraints
// ---------------------------------------------------------------------

/// The replacement a constraint is asked to judge.
///
/// The five arguments `OPT_CONSTRAINT::Check` takes
/// (`pcbnew/router/pns_optimizer.h:232`), bundled so that they travel
/// together and are named at the call site.
///
/// `vertex1` and `vertex2` are **point** indices into `current_path`: the
/// inclusive start and the exclusive end of the span being replaced, as
/// `mergeStep` passes them (`pcbnew/router/pns_optimizer.cpp:890`). Note
/// that the splice `mergeStep` then performs uses a different, one point
/// shorter range; see [`Optimizer::merge_step`].
#[derive(Copy, Clone, Debug)]
pub struct Candidate<'a> {
  /// KiCad's `aVertex1`.
  pub vertex1: usize,
  /// KiCad's `aVertex2`.
  pub vertex2: usize,
  /// KiCad's `aOriginLine`, the line being optimized.
  ///
  /// It is **not** the same as [`Candidate::current_path`]: a pass
  /// rewrites the path while the line keeps the shape it started the pass
  /// with, and [`Constraint::KeepTopology`] slices the line rather than
  /// the path (`pcbnew/router/pns_optimizer.cpp:430`).
  pub origin_line: &'a Line,
  /// KiCad's `aCurrentPath`, the path as the passes have it so far.
  pub current_path: &'a LineChain,
  /// KiCad's `aReplacement`, what the span would become.
  pub replacement: &'a LineChain,
}

/// A veto on a proposed replacement.
///
/// The `OPT_CONSTRAINT` hierarchy (`pcbnew/router/pns_optimizer.h:219`)
/// as one enum. There is no dynamic dispatch to earn here: the three
/// live checks are pure predicates over the same five arguments,
/// [`Optimizer::optimize`] is the only place that builds them, and the
/// `GetPriority` / `SetPriority` pair (`:236`) is never read.
///
/// Three of KiCad's subclasses are absent and one is deferred; see the
/// module documentation.
#[derive(Clone, PartialEq, Debug)]
pub enum Constraint {
  /// Both ends of the replaced span have to stay inside a box.
  ///
  /// Port of `AREA_CONSTRAINT`,
  /// `pcbnew/router/pns_optimizer.h:266`, checked at
  /// `pcbnew/router/pns_optimizer.cpp:217`. The two asymmetric escape
  /// hatches below the plain "both inside" test let a span leave the box
  /// at one end as long as the replacement's outermost segment stays
  /// parallel to the one it replaces, which is how a drag can shorten a
  /// track that runs out of the changed area.
  Area {
    /// KiCad's `m_allowedArea`.
    area: Box2,
    /// KiCad's `aAllowedAreaStrict`, which its constructor drops on the
    /// floor (`pcbnew/router/pns_optimizer.h:269`). It is carried here
    /// and never read, exactly as there; see
    /// [`Optimizer::set_restrict_area`].
    strict: bool,
  },
  /// One particular point has to survive the replacement.
  ///
  /// Port of `PRESERVE_VERTEX_CONSTRAINT`,
  /// `pcbnew/router/pns_optimizer.h:298`, checked at
  /// `pcbnew/router/pns_optimizer.cpp:247`. The test is on the point
  /// lying on the geometry, not on it being a vertex: a span that does
  /// not pass within [`PRESERVE_VERTEX_TOLERANCE`] of it is free to
  /// change, and one that does may only be replaced by a path that also
  /// passes within that tolerance.
  PreserveVertex {
    /// KiCad's `m_v`, the point the dragger anchored on.
    vertex: Vec2,
  },
  /// A bypass may not swallow another net's pad.
  ///
  /// Port of `KEEP_TOPOLOGY_CONSTRAINT`,
  /// `pcbnew/router/pns_optimizer.h:284`, checked at
  /// `pcbnew/router/pns_optimizer.cpp:426`. It closes the polygon between
  /// the original span and the replacement and refuses the replacement if
  /// any foreign solid's joint falls strictly inside it, which is the
  /// geometric way of saying "do not hop over a pad".
  ///
  /// KiCad's own comment on the implementation is "fixme: this is a
  /// remarkably shitty implementation" (`:432`); it is transcribed as it
  /// stands, including the crossing number routine it uses instead of
  /// [`LineChain::point_inside`], because the two disagree on boundary
  /// cases.
  KeepTopology,
  /// Every corner the replacement makes has to be obtuse or straight.
  ///
  /// Port of `OBTUSE_ONLY_CONSTRAINT`,
  /// `pcbnew/router/pns_optimizer.h:350`, checked at
  /// `pcbnew/router/pns_optimizer.cpp:305`. It looks at the corners
  /// **inside** the replacement and at the two seams where it joins the
  /// path on either side, and refuses the lot unless every one of them is
  /// obtuse or straight.
  ///
  /// KiCad's constructor takes the node and never reads it, so there is
  /// nothing to carry here.
  ObtuseOnly,
}

impl Constraint {
  /// Whether this constraint allows the replacement.
  ///
  /// Port of `OPT_CONSTRAINT::Check`,
  /// `pcbnew/router/pns_optimizer.h:232`, whose five arguments are
  /// [`Candidate`].
  pub fn check(
    &self,
    world: &World,
    node: NodeId,
    candidate: &Candidate<'_>,
  ) -> bool {
    match self {
      Constraint::Area { area, strict } => {
        // `strict` is KiCad's dead `aAllowedAreaStrict`.
        let _ = strict;
        check_area(*area, candidate)
      }
      Constraint::PreserveVertex { vertex } => {
        check_preserve_vertex(*vertex, candidate)
      }
      Constraint::KeepTopology => check_keep_topology(world, node, candidate),
      Constraint::ObtuseOnly => check_obtuse_only(candidate),
    }
  }
}

/// The mask a corner has to fall in for [`Constraint::ObtuseOnly`].
///
/// The `ANG_OBTUSE | ANG_STRAIGHT` of `isAngleOk`
/// (`pcbnew/router/pns_optimizer.cpp:316`). [`AngleType::RIGHT`] is
/// **not** in it, and `DIRECTION_45( seg )` is built with the default
/// `a90 = false`, so a 90 degree corner classifies as
/// [`AngleType::RIGHT`] and is refused.
const OBTUSE_OR_STRAIGHT: AngleType =
  AngleType::OBTUSE.union(AngleType::STRAIGHT);

/// Whether the corner between two segments is obtuse or straight.
///
/// Port of the `isAngleOk` lambda,
/// `pcbnew/router/pns_optimizer.cpp:311` to `:317`.
fn is_angle_ok(first: &Seg, second: &Seg) -> bool {
  Direction45::from_seg(first, false)
    .angle(Direction45::from_seg(second, false))
    .intersects(OBTUSE_OR_STRAIGHT)
}

/// The body of [`Constraint::ObtuseOnly`].
///
/// Port of `OBTUSE_ONLY_CONSTRAINT::Check`,
/// `pcbnew/router/pns_optimizer.cpp:305`.
///
/// The `vertex1 - 1 < path_segments` half of the first seam test has no
/// counterpart in KiCad, which reads the segment unchecked. It cannot
/// fire for a candidate [`Optimizer::merge_step`] builds, where `vertex1`
/// is a segment index of the path, and it keeps a candidate a caller
/// builds by hand from panicking on [`LineChain::segment`].
fn check_obtuse_only(candidate: &Candidate<'_>) -> bool {
  let Candidate {
    vertex1,
    vertex2,
    current_path,
    replacement,
    ..
  } = *candidate;

  // :319. A replacement of no segments has no corner to judge.
  let replacement_segments = replacement.segment_count();

  if replacement_segments < 1 {
    return true;
  }

  // :322. Every corner inside the replacement.
  for index in 0..replacement_segments - 1 {
    if !is_angle_ok(
      &replacement.segment(index),
      &replacement.segment(index + 1),
    ) {
      return false;
    }
  }

  let path_segments = current_path.segment_count();
  // :329, :330
  let first = replacement.segment(0);
  let last = replacement.segment(replacement_segments - 1);

  // :333. The seam with what comes before the span.
  if vertex1 > 0
    && vertex1 - 1 < path_segments
    && !is_angle_ok(&current_path.segment(vertex1 - 1), &first)
  {
    return false;
  }

  // :339. The seam with what comes after it.
  if vertex2 < path_segments
    && !is_angle_ok(&last, &current_path.segment(vertex2))
  {
    return false;
  }

  true
}

/// The body of [`Constraint::Area`].
///
/// Port of `AREA_CONSTRAINT::Check`,
/// `pcbnew/router/pns_optimizer.cpp:217`.
///
/// The bounds test at the top has no counterpart in KiCad, which reads
/// the two points unchecked. It cannot fire for a candidate
/// [`Optimizer::merge_step`] builds, since `vertex2` is at most the last
/// point index there, and it refuses rather than reading out of range for
/// one a caller builds by hand.
fn check_area(area: Box2, candidate: &Candidate<'_>) -> bool {
  let Candidate {
    vertex1,
    vertex2,
    current_path,
    replacement,
    ..
  } = *candidate;

  if vertex1 >= current_path.point_count()
    || vertex2 >= current_path.point_count()
    || replacement.segment_count() == 0
  {
    return false;
  }

  // :221
  let first = current_path.point(vertex1);
  let second = current_path.point(vertex2);
  let first_in = area.contains_point(Vec2L::from(first));
  let second_in = area.contains_point(Vec2L::from(second));

  // :227
  if first_in && second_in {
    return true;
  }

  // :230. Leaving the box at the start is allowed when the next vertex is
  // back inside and the replacement's first segment runs parallel to the
  // segment it replaces.
  if vertex1 < current_path.point_count() - 1
    && !first_in
    && second_in
    && area.contains_point(Vec2L::from(current_path.point(vertex1 + 1)))
  {
    return is_horizontal(
      replacement
        .segment(0)
        .angle_degrees(&current_path.segment(vertex1)),
    );
  }

  // :234. The mirror image at the far end.
  if first_in
    && !second_in
    && vertex2 > 0
    && area.contains_point(Vec2L::from(current_path.point(vertex2 - 1)))
  {
    return is_horizontal(
      replacement
        .segment(replacement.segment_count() - 1)
        .angle_degrees(&current_path.segment(vertex2 - 1)),
    );
  }

  // :243
  false
}

/// The body of [`Constraint::PreserveVertex`].
///
/// Port of `PRESERVE_VERTEX_CONSTRAINT::Check`,
/// `pcbnew/router/pns_optimizer.cpp:247`.
///
/// The clamp of the scan's upper bound to the segment count has no
/// counterpart in KiCad, for the same reason as in [`check_area`]: it
/// cannot fire for a candidate [`Optimizer::merge_step`] builds.
fn check_preserve_vertex(vertex: Vec2, candidate: &Candidate<'_>) -> bool {
  let Candidate {
    vertex1,
    vertex2,
    current_path,
    replacement,
    ..
  } = *candidate;
  let mut covered = false;
  let last_segment = current_path.segment_count();

  // :253. Does the span being replaced pass through the preserved point?
  for index in vertex1..vertex2.min(last_segment) {
    if current_path
      .segment(index)
      .squared_distance_to_point(vertex)
      <= PRESERVE_VERTEX_TOLERANCE
    {
      covered = true;
      break;
    }
  }

  // :264. It does not, so the replacement cannot lose it.
  if !covered {
    return true;
  }

  // :267. It does, so the replacement has to pass through it too.
  for index in 0..replacement.segment_count() {
    if replacement.segment(index).squared_distance_to_point(vertex)
      <= PRESERVE_VERTEX_TOLERANCE
    {
      return true;
    }
  }

  false
}

/// The body of [`Constraint::KeepTopology`].
///
/// Port of `KEEP_TOPOLOGY_CONSTRAINT::Check`,
/// `pcbnew/router/pns_optimizer.cpp:426`.
///
/// KiCad slices the **origin** line over indices that came from the
/// current path (`:430`), which can be out of range once an earlier merge
/// has shortened the path; its `Slice` answers that with an empty chain
/// and the check then passes. That is reproduced through the `Err` arm.
fn check_keep_topology(
  world: &World,
  node: NodeId,
  candidate: &Candidate<'_>,
) -> bool {
  let Candidate {
    vertex1,
    vertex2,
    origin_line,
    replacement,
    ..
  } = *candidate;

  // :430
  let Ok(mut enclosure) = origin_line.shape().slice(vertex1, vertex2) else {
    return true;
  };

  // :433
  enclosure.append_chain(&replacement.reversed());
  enclosure.set_closed(true);

  // :436
  let Some(bbox) = enclosure.bbox(0) else {
    return true;
  };

  // :439. No net filter, as KiCad has none here.
  let joints =
    world.query_joints(node, bbox, None, origin_line.layers(), Kind::SOLID);

  if joints.is_empty() {
    return true;
  }

  for reference in joints {
    let Some(joint) = world.joint(reference) else {
      continue;
    };

    // :446
    if joint.net() == origin_line.net() {
      continue;
    }

    if point_inside2(&enclosure, joint.pos()) {
      // :453. A joint that is one of the polygon's own vertices is not
      // enclosed by it, whatever the crossing number says.
      let false_positive =
        enclosure.points().iter().any(|point| *point == joint.pos());

      if !false_positive {
        return false;
      }
    }
  }

  true
}

/// Whether a point is inside a closed chain, boundary included.
///
/// Port of the file local `pointInside2`,
/// `pcbnew/router/pns_optimizer.cpp:360`. It is **not**
/// [`LineChain::point_inside`]: this one answers true for a point on the
/// boundary and the chain method answers false, and KiCad's own comment
/// at `:352` says the two have never been reconciled. The optimizer's
/// answer depends on the difference, so the routine is transcribed rather
/// than redirected.
///
/// Deviation: the two cross products are `double` in KiCad (`:391`,
/// `:407`) and `i64` here. Both operands are coordinate differences, so
/// the product fits and the integer form is exact where KiCad's loses
/// precision beyond 2^53.
fn point_inside2(chain: &LineChain, point: Vec2) -> bool {
  // :362
  if !chain.is_closed() || chain.segment_count() < 3 {
    return false;
  }

  let count = chain.point_count();
  let mut result = 0;
  let mut current = chain.point(0);

  // :370
  for index in 1..=count {
    let next = if index == count {
      chain.point(0)
    } else {
      chain.point(index)
    };

    // :374
    if next.y == point.y
      && (next.x == point.x
        || (current.y == point.y
          && ((next.x > point.x) == (current.x < point.x))))
    {
      return true;
    }

    // :381
    if (current.y < point.y) != (next.y < point.y) {
      let crosses = if current.x >= point.x {
        if next.x > point.x {
          result = 1 - result;
          false
        } else {
          true
        }
      } else {
        next.x > point.x
      };

      if crosses {
        // :391, :407
        let determinant = (i64::from(current.x) - i64::from(point.x))
          * (i64::from(next.y) - i64::from(point.y))
          - (i64::from(next.x) - i64::from(point.x))
            * (i64::from(current.y) - i64::from(point.y));

        if determinant == 0 {
          return true;
        }

        if (determinant > 0) == (next.y > current.y) {
          result = 1 - result;
        }
      }
    }

    current = next;
  }

  result > 0
}

/// Whether an angle between two segments is zero or 180 degrees.
///
/// Port of `EDA_ANGLE::IsHorizontal`,
/// `libs/kimath/include/geometry/eda_angle.h:142`, applied to what
/// [`Seg::angle_degrees`] returns. The name is KiCad's and it is
/// misleading: the two segments are not horizontal, they are parallel.
fn is_horizontal(degrees: f64) -> bool {
  degrees == 0.0 || degrees == 180.0
}

/// A chain segment's parent shape index as a `usize`.
///
/// The `s1.Index()` and `s2.Index()` of
/// `pcbnew/router/pns_optimizer.cpp:562` and `:896`.
/// [`LineChain::segment`] always fills the index in, so the sentinel
/// [`Seg::NO_INDEX`] cannot arrive here.
fn shape_index(seg: &Seg) -> usize {
  debug_assert!(
    seg.index >= 0,
    "a segment taken from a chain always carries its index"
  );

  seg.index.max(0) as usize
}

// ---------------------------------------------------------------------
// Pads and breakouts
// ---------------------------------------------------------------------

/// The exits a pad or via offers, in the order they are tried.
///
/// Port of the `BREAKOUT_LIST` typedef,
/// `pcbnew/router/pns_optimizer.h:161`. Each chain starts at the pad's
/// centre and ends where the trace is allowed to leave it, so the first
/// point of every entry is the same and the list is ordered: the
/// selection in [`Optimizer::smart_pads_single`] breaks a cost tie on
/// breakout length, and among breakouts of equal length the earlier entry
/// wins, so the order below is part of the answer.
pub type BreakoutList = Vec<LineChain>;

/// The corners smart pads refuses to build.
///
/// Port of the `ForbiddenAngles` local of
/// `pcbnew/router/pns_optimizer.cpp:1114`. A breakout to connection
/// corner in this set is dropped, and so is a whole candidate that
/// contains one anywhere.
pub const FORBIDDEN_ANGLES: AngleType = AngleType::ACUTE
  .union(AngleType::RIGHT)
  .union(AngleType::HALF_FULL)
  .union(AngleType::UNDEFINED);

/// How far along the line from a pad smart pads is allowed to rewrite.
///
/// Port of the `3` of `pcbnew/router/pns_optimizer.cpp:1133` and `:1246`,
/// a point index and not a segment count.
pub const SMART_PADS_MAX_VERTEX: isize = 3;

/// How many track widths long a line may be and still be a fanout.
///
/// Port of the `aLine->Width() * 10` threshold of
/// `pcbnew/router/pns_optimizer.cpp:1285`.
pub const FANOUT_CLEANUP_WIDTH_FACTOR: i64 = 10;

/// How many rays [`circle_breakouts`] casts.
///
/// Port of the `angle < ANGLE_360; angle += ANGLE_45` loop of
/// `pcbnew/router/pns_optimizer.cpp:924`, which is eight steps whether or
/// not diagonals were asked for.
pub const CIRCLE_BREAKOUT_COUNT: usize = 8;

/// How much longer than half the longer bounding box side a
/// [`custom_breakouts`] ray is cast, in nanometres.
///
/// Port of the `+ 5` of `pcbnew/router/pns_optimizer.cpp:951`, whose
/// comment says the ray "must be large enough to guarantee intersecting
/// the convex polygon". It is not: half a side falls short of half a
/// diagonal, so on an axis aligned square the four diagonal rays stop
/// inside the polygon. See [`custom_breakouts`].
pub const CUSTOM_BREAKOUT_RAY_MARGIN: i64 = 5;

/// A full turn in degrees, the bound of every breakout loop.
const FULL_TURN_DEGREES: f64 = 360.0;

/// The angular step between two breakouts when diagonals are permitted.
const DIAGONAL_STEP_DEGREES: f64 = 45.0;

/// The angular step between two breakouts when they are not.
const ORTHOGONAL_STEP_DEGREES: f64 = 90.0;

/// An angle folded into `[0, 360)` degrees.
///
/// Port of `EDA_ANGLE::Normalize`,
/// `libs/kimath/include/geometry/eda_angle.h:229`, whose first loop tests
/// against `-0.0`. That comparison is the same as one against `0.0` in
/// IEEE arithmetic, so a negative zero survives it, which is what makes
/// `RotatePoint` take its "no rotation" branch for `-ANGLE_0`.
fn normalize_degrees(degrees: f64) -> f64 {
  let mut value = degrees;

  while value < -0.0 {
    value += FULL_TURN_DEGREES;
  }

  while value >= FULL_TURN_DEGREES {
    value -= FULL_TURN_DEGREES;
  }

  value
}

/// The sine of an angle in degrees.
///
/// Port of `EDA_ANGLE::Sin`,
/// `libs/kimath/include/geometry/eda_angle.h:178`. The eight multiples of
/// 45 degrees are answered from a table so that they are exact; anything
/// else goes through `sin`, and it goes through it on the **unnormalized**
/// value, because KiCad normalizes a copy for the table lookup and then
/// calls `AsRadians()` on `this` (`:193`). Every angle the breakout
/// builders use is in the table, so the fallback is unreachable from this
/// module.
fn angle_sin(degrees: f64) -> f64 {
  let normalized = normalize_degrees(degrees);

  if normalized == 0.0 || normalized == 180.0 {
    0.0
  } else if normalized == 45.0 || normalized == 135.0 {
    std::f64::consts::FRAC_1_SQRT_2
  } else if normalized == 225.0 || normalized == 315.0 {
    -std::f64::consts::FRAC_1_SQRT_2
  } else if normalized == 90.0 {
    1.0
  } else if normalized == 270.0 {
    -1.0
  } else {
    degrees.to_radians().sin()
  }
}

/// The cosine of an angle in degrees.
///
/// Port of `EDA_ANGLE::Cos`,
/// `libs/kimath/include/geometry/eda_angle.h:197`; see [`angle_sin`] for
/// the table and for the unnormalized fallback.
fn angle_cos(degrees: f64) -> f64 {
  let normalized = normalize_degrees(degrees);

  if normalized == 0.0 {
    1.0
  } else if normalized == 180.0 {
    -1.0
  } else if normalized == 90.0 || normalized == 270.0 {
    0.0
  } else if normalized == 45.0 || normalized == 315.0 {
    std::f64::consts::FRAC_1_SQRT_2
  } else if normalized == 135.0 || normalized == 225.0 {
    -std::f64::consts::FRAC_1_SQRT_2
  } else {
    degrees.to_radians().cos()
  }
}

/// A point turned about the origin.
///
/// Port of `RotatePoint( int*, int*, const EDA_ANGLE& )`,
/// `libs/kimath/src/trigo.cpp:225`, including its four exact quadrant
/// cases. The rotation is clockwise in a y down coordinate system, which
/// is why the breakout builders pass a negated angle.
///
/// Deviation: none in value. KiCad multiplies an `int` by a `double` and
/// rounds with `KiROUND`; this widens to `f64` and rounds with
/// [`kiround`], which is the same operation. The crate otherwise avoids
/// `f64` where KiCad uses integers (`DESIGN.md` section 2), but here
/// KiCad's own arithmetic is floating point and reproducing the exact
/// breakout coordinates requires reproducing it.
fn rotate_point(point: Vec2, degrees: f64) -> Vec2 {
  let normalized = normalize_degrees(degrees);

  // :233. The cheap exact cases.
  if normalized == 0.0 {
    return point;
  }

  if normalized == 90.0 {
    return Vec2::new(point.y, -point.x);
  }

  if normalized == 180.0 {
    return Vec2::new(-point.x, -point.y);
  }

  if normalized == 270.0 {
    return Vec2::new(-point.y, point.x);
  }

  // :250
  let sinus = angle_sin(degrees);
  let cosinus = angle_cos(degrees);

  Vec2::new(
    kiround(f64::from(point.y) * sinus + f64::from(point.x) * cosinus),
    kiround(f64::from(point.y) * cosinus - f64::from(point.x) * sinus),
  )
}

/// A point turned about a centre.
///
/// Port of `RotatePoint( VECTOR2I&, const VECTOR2I&, const EDA_ANGLE& )`,
/// `libs/kimath/include/trigo.h:88`, which translates, rotates and
/// translates back.
fn rotate_point_about(point: Vec2, center: Vec2, degrees: f64) -> Vec2 {
  center + rotate_point(point - center, degrees)
}

/// The pad or via a line ends on, if there is one.
///
/// Port of `OPTIMIZER::findPadOrVia`,
/// `pcbnew/router/pns_optimizer.cpp:1093`: the joint at the point, then
/// the **first** link of that joint that is a via or a solid.
///
/// # The link order matters
///
/// Note 04 section 9 item 7 flags this as one of the places KiCad's
/// answer depends on container order: a point carrying both a pad and a
/// via resolves to whichever was linked first, and the two give different
/// breakout lists, so the whole smart pads pass can turn on it. KiCad's
/// `JOINT::LinkList` is a `std::vector<ITEM*>` in insertion order
/// (`pcbnew/router/pns_joint.h:303`) whose insertion order is inherited
/// from a hash set walk in `NODE::Commit`
/// (`pcbnew/router/pns_node.cpp:1630`), so it is stable within a session
/// and arbitrary across builds. Here [`crate::joint::Joint::links`] is a
/// `Vec` in insertion order too, and the order the world inserts in is
/// the caller's, so the answer is reproducible; `DESIGN.md` section 8
/// asks for exactly that.
///
/// # Deviation
///
/// KiCad's signature is `findPadOrVia( int aLayer, NET_HANDLE aNet, const
/// VECTOR2I& aP )` and it reads the node off `m_world`. The world and the
/// node are explicit here because this is a free function rather than a
/// method, and the net is explicit for the same reason it is there:
/// [`World::find_joint`] keys joints by position **and** net, so it
/// cannot be dropped.
pub fn find_pad_or_via(
  world: &World,
  node: NodeId,
  layer: i32,
  net: Option<NetId>,
  position: Vec2,
) -> Option<ItemId> {
  // :1095
  let reference = world.find_joint(node, position, layer, net)?;
  let joint = world.joint(reference)?;

  // :1100
  joint.links().iter().copied().find(|id| {
    world
      .item(*id)
      .is_some_and(|item| item.of_kind(Kind::VIA | Kind::SOLID))
  })
}

/// The eight exits of a round pad or a via.
///
/// Port of `OPTIMIZER::circleBreakouts`,
/// `pcbnew/router/pns_optimizer.cpp:919`. Eight rays from the centre at
/// 45 degree steps, each of length `radius * sqrt(2)` truncated to a
/// whole nanometre, so the four axis aligned ones end outside the copper
/// and the four diagonal ones end on the corner of the bounding box, at
/// distance `radius` in each axis.
///
/// `permit_diagonal` is accepted and **ignored**, as it is in KiCad
/// (`:924` has no branch on it), so a round pad always offers all eight.
/// `width` is ignored too: a circle's breakout length does not depend on
/// the track.
///
/// A shape that is not a [`Shape::Circle`] gives an empty list, where
/// KiCad `static_cast`s whatever it was handed.
pub fn circle_breakouts(
  width: i32,
  shape: &Shape,
  permit_diagonal: bool,
) -> BreakoutList {
  // Both are KiCad's unused parameters; see the doc comment.
  let _ = (width, permit_diagonal);

  let Shape::Circle { center, radius } = shape else {
    return BreakoutList::new();
  };

  // :929. `VECTOR2I( double, int )` truncates towards zero.
  let ray =
    Vec2::new((f64::from(*radius) * std::f64::consts::SQRT_2) as i32, 0);

  (0..CIRCLE_BREAKOUT_COUNT)
    .map(|step| {
      // :924, :931. KiCad accumulates `angle += ANGLE_45`; the multiple is
      // the same value in binary floating point and does not drift.
      let degrees = DIAGONAL_STEP_DEGREES * step as f64;

      LineChain::from_slice(
        &[*center, *center + rotate_point(ray, -degrees)],
        false,
      )
    })
    .collect()
}

/// The four or eight exits of a rectangular pad.
///
/// Port of `OPTIMIZER::rectBreakouts`,
/// `pcbnew/router/pns_optimizer.cpp:988`.
///
/// The four orthogonal exits are single segments from the centre to
/// `size / 2 + width` along each axis, so they clear the copper by a full
/// track width. They come first and in the order east, west, south,
/// north (`+x`, `-x`, `+y`, `-y`).
///
/// The four diagonal exits are two segment chains: first a run of
/// `d_offset` along the pad's long axis, then a 45 degree leg of
/// `width + min(size.x, size.y) / 2` in each axis. `d_offset` is half the
/// difference between the two sides, so on an oblong pad the diagonals
/// leave from the ends of the long axis rather than from the middle,
/// which is what the tie break in [`Optimizer::smart_pads_single`] is
/// there to prefer. On a square pad `d_offset` is zero and the first
/// point is repeated, exactly as in KiCad.
///
/// The two branches at `:1008` and `:1021` emit the same four diagonals
/// in **different orders**; that is transcribed rather than tidied,
/// because the order decides which of two equal cost, equal length
/// candidates wins.
///
/// # Orientation
///
/// A [`Shape::Rect`] is axis aligned by construction, here and in KiCad
/// (`libs/kimath/include/geometry/shape_rect.h:34`), and this routine
/// reads only its origin and size. [`crate::item::Solid`] carries an
/// orientation (`pcbnew/router/pns_solid.h:162`) and **nothing here reads
/// it**: a rotated rectangular pad reaches the router as a
/// [`Shape::Simple`] polygon and goes to [`custom_breakouts`] instead.
/// The only pad property the smart pads pass reads off the solid is the
/// offset, at `pcbnew/router/pns_optimizer.cpp:1123`.
///
/// A shape that is not a [`Shape::Rect`] gives an empty list.
pub fn rect_breakouts(
  width: i32,
  shape: &Shape,
  permit_diagonal: bool,
) -> BreakoutList {
  let Shape::Rect { origin, size, .. } = shape else {
    return BreakoutList::new();
  };

  // :991
  let size = *size;
  let center = *origin + Vec2::new(size.x / 2, size.y / 2);

  // :998. Half the difference between the sides, along the longer one.
  let offset = Vec2::new(
    if size.x > size.y {
      (size.x - size.y) / 2
    } else {
      0
    },
    if size.x < size.y {
      (size.y - size.x) / 2
    } else {
      0
    },
  );

  // :1003
  let vertical = Vec2::new(0, size.y / 2 + width);
  let horizontal = Vec2::new(size.x / 2 + width, 0);

  let mut breakouts = BreakoutList::with_capacity(12);

  // :1006
  breakouts.push(LineChain::from_slice(&[center, center + horizontal], false));
  breakouts.push(LineChain::from_slice(&[center, center - horizontal], false));
  breakouts.push(LineChain::from_slice(&[center, center + vertical], false));
  breakouts.push(LineChain::from_slice(&[center, center - vertical], false));

  if !permit_diagonal {
    return breakouts;
  }

  // :1013
  let leg = width + size.x.min(size.y) / 2;
  let mut diagonal = |corner: Vec2, tip: Vec2| {
    breakouts.push(LineChain::from_slice(
      &[center, corner, corner + tip],
      false,
    ));
  };

  if size.x >= size.y {
    // :1017
    diagonal(center + offset, Vec2::new(leg, leg));
    diagonal(center + offset, Vec2::new(leg, -leg));
    diagonal(center - offset, Vec2::new(-leg, leg));
    diagonal(center - offset, Vec2::new(-leg, -leg));
  } else {
    // :1029, the same four with the middle two swapped.
    diagonal(center + offset, Vec2::new(leg, leg));
    diagonal(center - offset, Vec2::new(leg, -leg));
    diagonal(center + offset, Vec2::new(-leg, leg));
    diagonal(center - offset, Vec2::new(-leg, -leg));
  }

  breakouts
}

/// The exits of a polygonal pad.
///
/// Port of `OPTIMIZER::customBreakouts`,
/// `pcbnew/router/pns_optimizer.cpp:942`. A ray is cast from the pad's
/// **position** every 45 degrees, or every 90 when `permit_diagonal` is
/// false, and the first point where it crosses the polygon boundary
/// becomes the breakout endpoint. A ray that misses contributes nothing,
/// so the list can be shorter than eight or four; KiCad's comment at
/// `:963` says it does not believe a miss can happen.
///
/// It can. The ray is only `max(bbox side) / 2 + 5` long
/// ([`CUSTOM_BREAKOUT_RAY_MARGIN`]), which is half a side and not half a
/// diagonal, so on an axis aligned square pad the four diagonal rays end
/// inside the copper and that pad silently offers four exits instead of
/// eight. Reproduced, and pinned by a test.
///
/// KiCad has three endpoint formulas here, two of them commented out
/// (`:967`, `:970`): a fraction of the centre to edge distance and a
/// fixed 0.1 mm stand off. The live one puts the breakout exactly on the
/// polygon edge and that is the one ported.
///
/// `width` is accepted and unused, as in KiCad.
///
/// The pad's centre is [`crate::item::Solid::pos`], which is **not**
/// necessarily inside the polygon and is not the polygon's centroid;
/// KiCad reads the same field (`:948`).
///
/// An item that is not a solid carrying a [`Shape::Simple`] gives an
/// empty list.
pub fn custom_breakouts(
  width: i32,
  item: &Item,
  permit_diagonal: bool,
) -> BreakoutList {
  // KiCad's unused parameter.
  let _ = width;

  let ItemBody::Solid(solid) = item.body() else {
    return BreakoutList::new();
  };
  let Some(Shape::Simple(convex)) = solid.shape() else {
    return BreakoutList::new();
  };
  let Some(bbox) = convex.bbox(0) else {
    return BreakoutList::new();
  };

  // :948
  let center = solid.pos();

  // :951
  let length = i32::try_from(
    bbox.width().max(bbox.height()) / 2 + CUSTOM_BREAKOUT_RAY_MARGIN,
  )
  .unwrap_or(i32::MAX);

  // :952
  let (step, count) = if permit_diagonal {
    (DIAGONAL_STEP_DEGREES, CIRCLE_BREAKOUT_COUNT)
  } else {
    (ORTHOGONAL_STEP_DEGREES, CIRCLE_BREAKOUT_COUNT / 2)
  };

  let mut breakouts = BreakoutList::new();

  for index in 0..count {
    let degrees = step * index as f64;

    // :956
    let tip =
      rotate_point_about(center + Vec2::new(length, 0), center, -degrees);

    // :961
    let intersections = convex.vertices().intersect_seg(&Seg::new(center, tip));

    // :966. Entry zero is the nearest to the ray's origin.
    if let Some(first) = intersections.first() {
      breakouts.push(LineChain::from_slice(&[center, first.point], false));
    }
  }

  breakouts
}

/// Every exit a pad or via offers.
///
/// Port of `OPTIMIZER::computeBreakouts`,
/// `pcbnew/router/pns_optimizer.cpp:1044`, the dispatcher on the item's
/// kind and then on its shape:
///
/// | item | shape | generator |
/// | --- | --- | --- |
/// | via | any | [`circle_breakouts`] on the layer 0 padstack circle |
/// | solid | [`Shape::Rect`] | [`rect_breakouts`] |
/// | solid | [`Shape::Segment`] | [`rect_breakouts`] over [`approximate_segment_as_rect`] |
/// | solid | [`Shape::Circle`] | [`circle_breakouts`] |
/// | solid | [`Shape::Simple`] | [`custom_breakouts`] |
///
/// Everything else, a [`Shape::Compound`] complex pad included, gives an
/// empty list and therefore no smart exit at all. That is KiCad's
/// `default: break` at `:1078` and it is deliberate here: inventing a
/// breakout set for a compound would change routing answers against
/// KiCad with no fixture to justify it.
///
/// The via branch asks the padstack for layer 0, carrying KiCad's
/// "TODO(JE) padstacks -- computeBreakouts needs to have a layer
/// argument" at `:1052` with it. It is unreachable from
/// [`Optimizer::run_smart_pads`], which refuses vias one step earlier
/// (`:1128`), and reachable from a caller that asks directly.
pub fn compute_breakouts(
  width: i32,
  item: &Item,
  permit_diagonal: bool,
) -> BreakoutList {
  match item.body() {
    // :1049
    ItemBody::Via(via) => {
      circle_breakouts(width, &via.shape(item.layers(), 0), permit_diagonal)
    }
    // :1056
    ItemBody::Solid(solid) => match solid.shape() {
      // :1062
      Some(shape @ Shape::Rect { .. }) => {
        rect_breakouts(width, shape, permit_diagonal)
      }
      // :1065
      Some(Shape::Segment {
        seg,
        width: segment_width,
      }) => rect_breakouts(
        width,
        &approximate_segment_as_rect(seg, *segment_width),
        permit_diagonal,
      ),
      // :1072
      Some(shape @ Shape::Circle { .. }) => {
        circle_breakouts(width, shape, permit_diagonal)
      }
      // :1075
      Some(Shape::Simple(_)) => custom_breakouts(width, item, permit_diagonal),
      // :1078
      _ => BreakoutList::new(),
    },
    // :1086
    _ => BreakoutList::new(),
  }
}

// ---------------------------------------------------------------------
// Optimizer
// ---------------------------------------------------------------------

/// The passes and their constraints over one node.
///
/// Port of `PNS::OPTIMIZER`, `pcbnew/router/pns_optimizer.h:94`. Build
/// one over the node the candidates are checked against, set the effort
/// level and whatever constraints the caller wants, then call
/// [`Optimizer::optimize`].
///
/// # What it holds and what it is handed
///
/// The node is stored, because it is what `SetWorld`
/// (`pcbnew/router/pns_optimizer.h:124`) replaces and it names the branch
/// every collision query goes to. The rule resolver, the settings and the
/// debug hook arrive per call in an [`AlgoContext`] instead of through a
/// router singleton; see `DESIGN.md` section 8 and
/// [`crate::algo_base`].
#[derive(Clone, Debug)]
pub struct Optimizer {
  /// The branch every candidate is checked against. Port of `m_world`
  /// (`pcbnew/router/pns_optimizer.h:208`).
  node: NodeId,
  /// Which kinds of item may be an obstacle. Port of
  /// `m_collisionKindMask` (`:209`).
  ///
  /// **Never read**, as in KiCad: its only consumer there is the
  /// `CACHE_VISITOR` that is constructed and discarded
  /// (`pcbnew/router/pns_optimizer.cpp:476`). See
  /// [`Optimizer::set_collision_mask`].
  collision_mask: Kind,
  /// Which passes and constraints run. Port of `m_effortLevel` (`:210`).
  effort: EffortFlags,
  /// The point [`EffortFlags::PRESERVE_VERTEX`] protects. Port of
  /// `m_preservedVertex` (`:212`).
  preserved_vertex: Vec2,
  /// The box [`EffortFlags::RESTRICT_AREA`] confines the work to. Port of
  /// `m_restrictArea` (`:214`), whose default constructed `BOX2I` is
  /// [`None`] here.
  restrict_area: Option<Box2>,
  /// KiCad's `m_restrictAreaIsStrict` (`:215`), which nothing reads.
  restrict_area_is_strict: bool,
  /// The constraints the current effort level asked for. Port of
  /// `m_constraints` (`:205`).
  constraints: Vec<Constraint>,
}

impl Optimizer {
  /// An optimizer over one branch, with KiCad's defaults.
  ///
  /// Port of the constructor,
  /// `pcbnew/router/pns_optimizer.cpp:113`: every kind may be an
  /// obstacle, the effort level starts at
  /// [`EffortFlags::MERGE_SEGMENTS`] and the area restriction is not
  /// strict.
  pub fn new(node: NodeId) -> Self {
    Self {
      node,
      collision_mask: Kind::ANY,
      effort: EffortFlags::MERGE_SEGMENTS,
      preserved_vertex: Vec2::new(0, 0),
      restrict_area: None,
      restrict_area_is_strict: false,
      constraints: Vec::new(),
    }
  }

  /// The branch the collision queries run against.
  pub const fn world(&self) -> NodeId {
    self.node
  }

  /// Optimize against a different branch.
  ///
  /// Port of `SetWorld`, `pcbnew/router/pns_optimizer.h:124`.
  pub const fn set_world(&mut self, node: NodeId) {
    self.node = node;
  }

  /// The effort level.
  pub const fn effort_level(&self) -> EffortFlags {
    self.effort
  }

  /// Replace the effort level.
  ///
  /// Port of `SetEffortLevel`,
  /// `pcbnew/router/pns_optimizer.h:133`.
  pub const fn set_effort_level(&mut self, effort: EffortFlags) {
    self.effort = effort;
  }

  /// Add to the effort level.
  ///
  /// KiCad has no such method; its callers write
  /// `optFlags |= OPTIMIZER::LIMIT_CORNER_COUNT`
  /// (`pcbnew/router/pns_shove.cpp:2064`) on a local integer and set the
  /// whole thing at the end. This is the same thing spelled on the
  /// optimizer, and it is what the three `SetXxx` methods below do
  /// internally, exactly as `pcbnew/router/pns_optimizer.h:141` does.
  pub fn add_effort_level(&mut self, effort: EffortFlags) {
    self.effort |= effort;
  }

  /// Which kinds of item may stop a candidate.
  ///
  /// Port of `SetCollisionMask`,
  /// `pcbnew/router/pns_optimizer.h:128`.
  ///
  /// **The mask changes nothing**, here or in KiCad. Its only reader
  /// there is the `CACHE_VISITOR` that `checkColliding` builds and throws
  /// away (`pcbnew/router/pns_optimizer.cpp:476`), so the actual query
  /// runs with the node's default kind mask. The setter is kept because
  /// both callers use it (`pcbnew/router/pns_shove.cpp:2087` with
  /// `ITEM::ANY_T`, `pcbnew/router/pns_optimizer.cpp:1263` with `-1`) and
  /// because a port that silently started honouring it would answer
  /// differently from KiCad. Honouring it is a one line change in
  /// [`Optimizer::check_colliding`] if a fixture ever wants it.
  pub const fn set_collision_mask(&mut self, mask: Kind) {
    self.collision_mask = mask;
  }

  /// The kind mask [`Optimizer::set_collision_mask`] stored.
  pub const fn collision_mask(&self) -> Kind {
    self.collision_mask
  }

  /// Confine the work to a box, and turn
  /// [`EffortFlags::RESTRICT_AREA`] on.
  ///
  /// Port of `SetRestrictArea`,
  /// `pcbnew/router/pns_optimizer.h:151`, which also sets the flag.
  ///
  /// `strict` is KiCad's `aAllowedAreaStrict`, defaulted to `true` there.
  /// **It does nothing.** `AREA_CONSTRAINT`'s constructor takes the
  /// argument and does not store it (`pcbnew/router/pns_optimizer.h:269`),
  /// so the constraint always runs in its lenient form, with the two
  /// escape hatches described on [`Constraint::Area`]. The two callers
  /// disagree on the value they pass, the shove `false`
  /// (`pcbnew/router/pns_shove.cpp:2073`) and the dragger the `true`
  /// default (`pcbnew/router/pns_dragger.cpp:607`), and get the same
  /// behaviour.
  ///
  /// The decision here is to reproduce that rather than to invent a
  /// meaning for the flag: inventing one would change routing answers
  /// against KiCad with no fixture to justify the change, and the two
  /// live callers would start disagreeing. The flag is carried on
  /// [`Constraint::Area`] so that the day a fixture asks for strict
  /// behaviour, there is one place to put it.
  pub fn set_restrict_area(&mut self, area: Box2, strict: bool) {
    self.restrict_area = Some(area);
    self.restrict_area_is_strict = strict;
    self.effort |= EffortFlags::RESTRICT_AREA;
  }

  /// The box [`Optimizer::set_restrict_area`] stored.
  pub const fn restrict_area(&self) -> Option<Box2> {
    self.restrict_area
  }

  /// Protect one point, and turn [`EffortFlags::PRESERVE_VERTEX`] on.
  ///
  /// Port of `SetPreserveVertex`,
  /// `pcbnew/router/pns_optimizer.h:138`, which also sets the flag. The
  /// dragger passes the point being dragged, or the nearest good corner
  /// to it (`pcbnew/router/pns_dragger.cpp:593`).
  pub fn set_preserve_vertex(&mut self, vertex: Vec2) {
    self.preserved_vertex = vertex;
    self.effort |= EffortFlags::PRESERVE_VERTEX;
  }

  /// The point [`Optimizer::set_preserve_vertex`] stored.
  pub const fn preserved_vertex(&self) -> Vec2 {
    self.preserved_vertex
  }

  /// Confine the work to a range of vertices, and turn
  /// [`EffortFlags::RESTRICT_VERTEX_RANGE`] on.
  ///
  /// Port of `SetRestrictVertexRange`,
  /// `pcbnew/router/pns_optimizer.h:144`.
  ///
  /// **This is a no operation and so is KiCad's.**
  /// `RESTRICT_VERTEX_RANGE_CONSTRAINT`'s constructor ignores both bounds
  /// (`pcbnew/router/pns_optimizer.h:318`) and its `Check` returns true
  /// unconditionally (`pcbnew/router/pns_optimizer.cpp:279`), so setting
  /// the range has never restricted anything. Nothing in KiCad's tree
  /// calls it either. The method exists so that a host porting code that
  /// calls it does not have to guess whether the call mattered: it did
  /// not.
  ///
  /// The bounds are not even stored, because storing them would suggest
  /// they are read. The flag is set, as KiCad sets it, so that an effort
  /// level round trips.
  pub fn set_restrict_vertex_range(&mut self, start: usize, end: usize) {
    let _ = (start, end);
    self.effort |= EffortFlags::RESTRICT_VERTEX_RANGE;
  }

  /// Drop every constraint the last [`Optimizer::optimize`] built.
  ///
  /// KiCad has no such method: it deletes the constraints in its
  /// destructor and **never** clears them between calls
  /// (`pcbnew/router/pns_optimizer.cpp:122`), so an optimizer reused over
  /// a queue accumulates one duplicate set per call.
  /// `SHOVE::runOptimizer` does exactly that
  /// (`pcbnew/router/pns_shove.cpp:2024`, one optimizer for the whole
  /// queue and two passes over it).
  ///
  /// Deviation: [`Optimizer::optimize`] calls this first, so the
  /// constraints are rebuilt per call and never accumulate. Note 04
  /// section 4.2 says not to replicate the accumulation and explains why
  /// it is safe to drop: the constraints are pure predicates, so a
  /// conjunction of duplicates is the conjunction of one of each. What is
  /// saved is O(calls) work per candidate.
  pub fn clear_constraints(&mut self) {
    self.constraints.clear();
  }

  /// The constraints the last [`Optimizer::optimize`] built.
  pub fn constraints(&self) -> &[Constraint] {
    &self.constraints
  }

  // -----------------------------------------------------------------
  // The driver
  // -----------------------------------------------------------------

  /// Run every pass the effort level asks for.
  ///
  /// Port of `OPTIMIZER::Optimize( const LINE*, LINE*, LINE* )`,
  /// `pcbnew/router/pns_optimizer.cpp:652`. `result` receives a copy of
  /// `line` with its links cleared and is then rewritten by the passes;
  /// the answer is whether any pass changed anything.
  ///
  /// KiCad's `aResult` is a nullable out pointer whose null case returns
  /// false immediately (`:654`); here it is a plain `&mut`, so that case
  /// cannot arise.
  ///
  /// `root` is KiCad's `aRoot`, the pre optimization line the dragger
  /// passes so that `LIMIT_CORNER_COUNT` has a baseline to compare
  /// against (`pcbnew/router/pns_dragger.cpp:612`). That constraint is a
  /// no operation in KiCad and is not ported, so the argument is accepted
  /// and unused; it is kept in the signature because the shove and the
  /// dragger both pass one and because the constraint may yet be given a
  /// working maximum.
  ///
  /// # Order
  ///
  /// The constraints are assembled first, in KiCad's order, then the
  /// passes run in KiCad's order. Both orders are observable: a pass sees
  /// every constraint, and an earlier pass changes what a later one is
  /// given.
  ///
  /// # Arcs
  ///
  /// KiCad skips four of the six passes when the line contains an arc
  /// (`:660`, `:713`, `:717`, `:724`, `:728`). There is no arc type in
  /// this crate yet (`DESIGN.md` section 3), so the guard is documented
  /// rather than written; it comes back with the arcs.
  pub fn optimize(
    &mut self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &Line,
    result: &mut Line,
    root: Option<&Line>,
  ) -> bool {
    // Only `LIMIT_CORNER_COUNT` reads it, and that constraint is inert.
    let _ = root;

    // :657
    *result = line.clone();
    result.clear_links();

    // See the deviation on `clear_constraints`.
    self.clear_constraints();

    let mut changed = false;

    // :663. `LIMIT_CORNER_COUNT` adds no constraint here: KiCad's returns
    // true on both branches.

    // :676
    if self.effort.contains(EffortFlags::PRESERVE_VERTEX) {
      self.constraints.push(Constraint::PreserveVertex {
        vertex: self.preserved_vertex,
      });
    }

    // :682. `RESTRICT_VERTEX_RANGE` adds no constraint here either.

    // :689
    if self.effort.contains(EffortFlags::RESTRICT_AREA)
      && let Some(area) = self.restrict_area
    {
      self.constraints.push(Constraint::Area {
        area,
        strict: self.restrict_area_is_strict,
      });
    }

    // :697
    if self.effort.contains(EffortFlags::KEEP_TOPOLOGY) {
      self.constraints.push(Constraint::KeepTopology);
    }

    // :703
    if self.effort.contains(EffortFlags::REQUIRE_OBTUSE_ANGLES) {
      self.constraints.push(Constraint::ObtuseOnly);
    }

    // :709. The one pass that runs before `mergeFull`, so that everything
    // after it works on the line with its bad corner already bypassed.
    if self.effort.contains(EffortFlags::REQUIRE_OBTUSE_ANGLES) {
      changed |= self.drag_fix_corners(world, context, result);
    }

    // :713
    if self.effort.contains(EffortFlags::MERGE_SEGMENTS) {
      changed |= self.merge_full(world, context, result);
    }

    // :717
    if self.effort.contains(EffortFlags::MERGE_OBTUSE) {
      changed |= self.merge_obtuse(world, context, result);
    }

    // :720
    if self.effort.contains(EffortFlags::MERGE_COLINEAR) {
      changed |= self.merge_colinear(result);
    }

    // :724. This one reports "changed" for any line of three points or
    // more, whether or not it moved anything; see
    // `Optimizer::run_smart_pads`.
    if self.effort.contains(EffortFlags::SMART_PADS) {
      changed |= self.run_smart_pads(world, context, result);
    }

    // :728
    if self.effort.contains(EffortFlags::FANOUT_CLEANUP) {
      changed |= self.fanout_cleanup(world, context, result);
    }

    changed
  }

  /// Optimize one line in place, without setting an optimizer up.
  ///
  /// Port of the static convenience overload
  /// `OPTIMIZER::Optimize( LINE*, int, NODE*, const VECTOR2I& )`,
  /// `pcbnew/router/pns_optimizer.cpp:1258`, which is what the line
  /// placer and the router tool use. It builds a throwaway optimizer, and
  /// sets the preserved vertex only when the effort level asks for it,
  /// because [`Optimizer::set_preserve_vertex`] would otherwise turn the
  /// flag on by itself.
  ///
  /// KiCad's collision mask of `-1` is [`Kind::ANY`] here; it changes
  /// nothing either way, see [`Optimizer::set_collision_mask`].
  pub fn optimize_line(
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    line: &mut Line,
    effort: EffortFlags,
    origin: Vec2,
  ) -> bool {
    let mut optimizer = Optimizer::new(node);

    optimizer.set_effort_level(effort);
    optimizer.set_collision_mask(Kind::ANY);

    // :1265
    if effort.contains(EffortFlags::PRESERVE_VERTEX) {
      optimizer.set_preserve_vertex(origin);
    }

    // :1268. KiCad optimizes a copy into the caller's line.
    let input = line.clone();

    optimizer.optimize(world, context, &input, line, None)
  }

  // -----------------------------------------------------------------
  // Collision and constraints
  // -----------------------------------------------------------------

  /// Whether a candidate line hits anything in the node.
  ///
  /// Port of `OPTIMIZER::checkColliding( ITEM*, bool )`,
  /// `pcbnew/router/pns_optimizer.cpp:474`, narrowed to the one kind of
  /// item it is ever passed. KiCad's `aUpdateCache` argument is unused
  /// there and the `CACHE_VISITOR` it builds is discarded, so all that is
  /// left is the query itself.
  ///
  /// It forwards to [`World::check_colliding_line`] with the options
  /// KiCad's `NODE::CheckColliding( const ITEM* )` overload sets
  /// (`pcbnew/router/pns_node.h:342`): the default kind mask and a limit
  /// of one, since the answer is only a yes or no.
  ///
  /// See the module documentation for why the line must not be in the
  /// node.
  pub fn check_colliding(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &Line,
  ) -> bool {
    let options = CollisionSearchOptions {
      limit_count: Some(1),
      ..CollisionSearchOptions::default()
    };

    world
      .check_colliding_line(self.node, line, context.resolver, &options)
      .is_some()
  }

  /// Whether a candidate path hits anything in the node.
  ///
  /// Port of `OPTIMIZER::checkColliding( LINE*, const SHAPE_LINE_CHAIN& )`,
  /// `pcbnew/router/pns_optimizer.cpp:502`, which wraps the path in a
  /// temporary line carrying the original's width, layer and net and
  /// forwards to [`Optimizer::check_colliding`].
  pub fn check_colliding_path(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &Line,
    path: &LineChain,
  ) -> bool {
    let candidate = Line::with_chain(line, path.clone());

    self.check_colliding(world, context, &candidate)
  }

  /// Whether every constraint allows a replacement.
  ///
  /// Port of `OPTIMIZER::checkConstraints`,
  /// `pcbnew/router/pns_optimizer.cpp:488`, a plain conjunction. An
  /// optimizer with no constraints allows everything.
  pub fn check_constraints(
    &self,
    world: &World,
    candidate: &Candidate<'_>,
  ) -> bool {
    self
      .constraints
      .iter()
      .all(|constraint| constraint.check(world, self.node, candidate))
  }

  // -----------------------------------------------------------------
  // The passes
  // -----------------------------------------------------------------

  /// Replace one bad corner with the 45 degree bypass that encloses the
  /// least area.
  ///
  /// Port of `OPTIMIZER::dragFixCorner`,
  /// `pcbnew/router/pns_optimizer.cpp:738`. `vertex_index` names the
  /// corner, which is bad when the two segments meeting there make an
  /// angle of [`AngleType::RIGHT`], [`AngleType::ACUTE`] or
  /// [`AngleType::HALF_FULL`]; anything else is left alone.
  ///
  /// The bypass runs from the far end of the segment before the corner to
  /// the far end of the segment after it, in each of the two postures.
  /// One that collides is dropped, and of the survivors the winner is the
  /// one whose closed loop against the original corner has the smallest
  /// area. That comparison is the only floating point in the pass, over
  /// at most two candidates, so it cannot make the answer depend on
  /// anything but the geometry.
  ///
  /// The corner mode is **not** consulted: `DIRECTION_45()` is default
  /// constructed at `:767`, so the bypass is always mitered at 45
  /// degrees, where [`Optimizer::merge_step`] passes the setting through
  /// (`:883`).
  ///
  /// The arc guard of `:745` has no counterpart until arcs land.
  pub fn drag_fix_corner(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &mut Line,
    vertex_index: usize,
  ) -> bool {
    // :742
    if vertex_index == 0 || vertex_index + 1 >= line.shape().point_count() {
      return false;
    }

    // :748, :749
    let before = line.shape().segment(vertex_index - 1);
    let after = line.shape().segment(vertex_index);
    let corner = line.shape().point(vertex_index);

    // :751, :753
    let angle = Direction45::from_seg(&before, false)
      .angle(Direction45::from_seg(&after, false));

    if angle != AngleType::RIGHT
      && angle != AngleType::ACUTE
      && angle != AngleType::HALF_FULL
    {
      return false;
    }

    // :756 to :760. A corner outside the restricted area is not this
    // drag's business.
    if self.effort.contains(EffortFlags::RESTRICT_AREA)
      && let Some(area) = self.restrict_area
      && !area.contains_point(Vec2L::from(corner))
    {
      return false;
    }

    let mut best_bypass: Option<LineChain> = None;
    let mut best_area = f64::MAX;

    // :765. Posture 0 is straight leg first, posture 1 diagonal first,
    // the same way round as in `merge_step`.
    for posture in 0..POSTURE_COUNT {
      let bypass = LineChain::from_points(
        Direction45::default().build_initial_trace(
          before.a,
          after.b,
          posture == 1,
          CornerMode::Mitered45,
        ),
        false,
      );

      // :769
      if bypass.segment_count() < 1 {
        continue;
      }

      // :772
      if self.check_colliding_path(world, context, line, &bypass) {
        continue;
      }

      // :775 to :783. The closed loop between the corner and the bypass,
      // whose area says how much the bypass cuts off.
      let mut enclosed = LineChain::new();

      enclosed.append(before.a);
      enclosed.append(corner);
      enclosed.append(after.b);

      for index in (0..bypass.point_count()).rev() {
        enclosed.append(bypass.point(index));
      }

      enclosed.set_closed(true);

      // :785
      let area = enclosed.area(true);

      if area < best_area {
        best_area = area;
        best_bypass = Some(bypass);
      }
    }

    // :792
    let Some(best_bypass) = best_bypass else {
      return false;
    };

    // :795, :796. `s1.Index()` is `vertex_index - 1` and `s2.Index()` is
    // `vertex_index`, and `Replace` takes point indices.
    let path = line.chain_mut();

    path.replace_with_chain(vertex_index - 1, vertex_index, &best_bypass);
    path.simplify2(true);

    true
  }

  /// Fix the corner the drag anchored on, and its neighbours.
  ///
  /// Port of `OPTIMIZER::dragFixCorners`,
  /// `pcbnew/router/pns_optimizer.cpp:801`, the pass
  /// [`EffortFlags::REQUIRE_OBTUSE_ANGLES`] selects. The anchor is the
  /// preserved vertex when there is one and the line's last point
  /// otherwise (`:807`), which is the point the drag pinned; the pass
  /// only ever touches the corner there.
  ///
  /// When the anchor sits in the middle of a straight run the corner on
  /// **either** side of it is fixed, because a straight anchor is not
  /// itself a corner and the drag's bad corner is then one vertex away
  /// (`:830` to `:839`). The re-`Find` between the two calls is not
  /// redundant: the first fix rewrites the chain, so the anchor's index
  /// moves.
  ///
  /// The internal `REQUIRE_OBTUSE_ANGLES` test at `:803` duplicates the
  /// call site's guard at `:709` and is transcribed rather than dropped,
  /// so that a caller reaching the pass directly answers as KiCad does.
  ///
  /// The arc guard of `:814` has no counterpart until arcs land.
  pub fn drag_fix_corners(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &mut Line,
  ) -> bool {
    // :803
    if !self.effort.contains(EffortFlags::REQUIRE_OBTUSE_ANGLES) {
      return false;
    }

    // :807. KiCad reads `CLastPoint()` unchecked, which is out of range
    // for an empty chain; there is nothing to anchor on then.
    let anchor = if self.effort.contains(EffortFlags::PRESERVE_VERTEX) {
      self.preserved_vertex
    } else {
      match line.shape().last_point() {
        Some(point) => point,
        None => return false,
      }
    };

    // :809, :811. An anchor that is not a vertex, or is the first one,
    // has no corner before it to fix.
    let Some(anchor_index) = line.shape().find(anchor, 0) else {
      return false;
    };

    if anchor_index == 0 {
      return false;
    }

    // :819. The anchor is the last point, so the only corner it has is
    // the one before it. KiCad re-tests `anchorIdx > 0` at `:821`, which
    // `:811` has already decided.
    if anchor_index + 1 >= line.shape().point_count() {
      return self.drag_fix_corner(world, context, line, anchor_index - 1);
    }

    // :827
    let before = line.shape().segment(anchor_index - 1);
    let after = line.shape().segment(anchor_index);
    let angle = Direction45::from_seg(&before, false)
      .angle(Direction45::from_seg(&after, false));

    // :830
    if angle != AngleType::STRAIGHT {
      // :842
      return self.drag_fix_corner(world, context, line, anchor_index);
    }

    // :832
    let mut changed =
      self.drag_fix_corner(world, context, line, anchor_index - 1);

    // :834, :835. The fix above moved the anchor, so its index is looked
    // up again.
    line.chain_mut().split(anchor);

    let Some(anchor_index) = line.shape().find(anchor, 0) else {
      return changed;
    };

    // :837, :838
    if anchor_index > 0 && anchor_index + 1 < line.shape().point_count() {
      changed |= self.drag_fix_corner(world, context, line, anchor_index + 1);
    }

    changed
  }

  /// Replace spans of the line with two segment bypasses, coarse to
  /// fine.
  ///
  /// Port of `OPTIMIZER::mergeFull`,
  /// `pcbnew/router/pns_optimizer.cpp:587`. It starts by trying to bypass
  /// the longest possible span and only shortens the span when nothing at
  /// that length works, so a line is straightened globally before it is
  /// tidied locally. [`Optimizer::merge_step`] returns after its first
  /// success, which is why the outer loop retries at the same span length
  /// after every hit.
  ///
  /// # What the answer means, and does not
  ///
  /// The answer is `current_path.SegmentCount() < segs_pre` (`:623`) and
  /// nothing else, so it is a segment count comparison and not "did the
  /// shape change". Two things follow, and both are reproduced.
  ///
  /// A pass that rewrote the line without shortening it reports `false`.
  /// That is easy to hit: a right angle traded for two 45 degree corners
  /// keeps the count. A caller that only writes the result back when the
  /// answer is `true`, as `SHOVE::runOptimizer` does
  /// (`pcbnew/router/pns_shove.cpp:2121`), silently drops such an
  /// improvement.
  ///
  /// And `segs_pre` and the initial step are read **before** the
  /// [`LineChain::simplify2`] at `:594`, so a line that only needed
  /// simplifying, with no bypass found at all, reports `true`. It did
  /// change, because the simplify wrote through to the line, so the
  /// report is not wrong; it is just not what the pass is named after.
  ///
  /// KiCad ends with `aLine->SetShape( current_path )` (`:621`) where
  /// [`Optimizer::merge_obtuse`] assigns the chain directly. The two are
  /// equivalent here, since the path is a copy of a chain that already
  /// carries the line's width, but both are transcribed as they stand.
  pub fn merge_full(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &mut Line,
  ) -> bool {
    // :590
    let mut step = line.segment_count() as isize - 1;
    let segments_before = line.segment_count();

    // :594
    line.chain_mut().simplify2(true);

    // :596
    if step < 0 {
      return false;
    }

    // :599
    let mut current_path = line.shape().clone();

    for _ in 0..MERGE_PASS_LIMIT {
      // :604
      let max_step = current_path.segment_count() as isize - 2;

      if step > max_step {
        step = max_step;
      }

      // :609
      if step < 1 {
        break;
      }

      // :612
      let found =
        self.merge_step(world, context, line, &mut current_path, step as usize);

      if !found {
        step -= 1;
      }

      // :617
      if step == 0 {
        break;
      }
    }

    let segments_after = current_path.segment_count();

    // :621
    line.set_shape(current_path);

    segments_after < segments_before
  }

  /// Try to bypass one span of a given length, anywhere along the path.
  ///
  /// Port of `OPTIMIZER::mergeStep`,
  /// `pcbnew/router/pns_optimizer.cpp:849`. For each span of `step`
  /// segments it builds the two canonical 45 degree connections between
  /// the span's outer endpoints with
  /// [`Direction45::build_initial_trace`], keeps the ones that neither
  /// collide nor break a constraint, and takes the cheaper of those if it
  /// also beats the path it would replace. It returns after the first
  /// span that yields a winner, having written that winner into
  /// `current_path`.
  ///
  /// The corner mode comes from
  /// [`crate::settings::RoutingSettings::corner_mode`] off `context`,
  /// where KiCad reads it off the router singleton (`:858`).
  ///
  /// # Two quirks
  ///
  /// `orig_start` and `orig_end` (`:861`, `:862`) are computed from the
  /// original line's first and last segments and never read. They are not
  /// ported; they are the only thing the `is90mode` local at `:859` is
  /// used for besides the corner mode it already carries.
  ///
  /// `n_segs` is reassigned from the path's segment count immediately
  /// before the successful return (`:909`), and the function returns on
  /// the next line, so the store is dead. Not ported. The same dead store
  /// appears in [`Optimizer::merge_obtuse`] at `:565`, where the `break`
  /// that follows it leaves the loop that reads it.
  ///
  /// # The index the replacement uses
  ///
  /// The constraints are checked over the point range
  /// `[n, n + step + 1]` (`:890`) while the replacement is spliced over
  /// `[n, n + step]` (`:896`), one point short at the far end. The bypass
  /// therefore ends on a duplicate of the point that follows the span,
  /// and the [`LineChain::simplify2`] at `:897` removes it. Reproduced,
  /// because the constraint range and the splice range are both
  /// observable.
  pub fn merge_step(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &Line,
    current_path: &mut LineChain,
    step: usize,
  ) -> bool {
    // :851
    let segment_count = current_path.segment_count();
    let cost_before = CostEstimator::corner_cost_of_chain(current_path);

    // :855
    if line.segment_count() < 2 {
      return false;
    }

    // :858
    let corner_mode = context.settings.corner_mode;

    // :865
    for start in 0..segment_count.saturating_sub(step) {
      // :868. The arc guard has no counterpart until arcs land.
      let first = current_path.segment(start);
      let second = current_path.segment(start + step);
      let mut candidates: [Option<LineChain>; POSTURE_COUNT] = [None, None];
      let mut costs = [i32::MAX; POSTURE_COUNT];

      // :881. Posture 0 is straight leg first, posture 1 diagonal first.
      for posture in 0..POSTURE_COUNT {
        // :883
        let bypass = LineChain::from_points(
          Direction45::default().build_initial_trace(
            first.a,
            second.b,
            posture == 1,
            corner_mode,
          ),
          false,
        );

        // :888
        if self.check_colliding_path(world, context, line, &bypass) {
          continue;
        }

        let candidate_span = Candidate {
          vertex1: start,
          vertex2: start + step + 1,
          origin_line: line,
          current_path,
          replacement: &bypass,
        };

        if !self.check_constraints(world, &candidate_span) {
          continue;
        }

        // :895
        let mut candidate = current_path.clone();

        candidate.replace_with_chain(
          shape_index(&first),
          shape_index(&second),
          &bypass,
        );
        candidate.simplify2(true);
        costs[posture] = CostEstimator::corner_cost_of_chain(&candidate);
        candidates[posture] = Some(candidate);
      }

      // :902
      let picked = if costs[0] < cost_before && costs[0] < costs[1] {
        candidates[0].take()
      } else if costs[1] < cost_before {
        candidates[1].take()
      } else {
        None
      };

      // :907
      if let Some(path) = picked {
        *current_path = path;
        return true;
      }
    }

    false
  }

  /// Fold runs of obtuse corners into the corner where their outer
  /// segments cross.
  ///
  /// Port of `OPTIMIZER::mergeObtuse`,
  /// `pcbnew/router/pns_optimizer.cpp:510`. Same coarse to fine skeleton
  /// as [`Optimizer::merge_full`], but the candidate is not a generated
  /// bypass: it is the intersection of the two spanning segments'
  /// infinite lines, kept only when both halves of the new corner are
  /// still obtuse.
  ///
  /// # It ignores the constraints
  ///
  /// This pass calls [`Optimizer::check_colliding`] and **not**
  /// [`Optimizer::check_constraints`] (`:560`), so
  /// [`EffortFlags::RESTRICT_AREA`], [`EffortFlags::PRESERVE_VERTEX`] and
  /// [`EffortFlags::KEEP_TOPOLOGY`] do not restrain it. That is KiCad's
  /// behaviour and it is reproduced; see the module documentation for
  /// where it bites.
  ///
  /// # Deviation
  ///
  /// KiCad dereferences `s1.IntersectLines( s2 )` without checking the
  /// optional (`:546`). Two obtuse related segments in the 45 degree
  /// world are never parallel, so it holds there, but a caller can hand
  /// this an off grid line. A span with no intersection is skipped here
  /// instead of read through an empty optional.
  pub fn merge_obtuse(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &mut Line,
  ) -> bool {
    // :514
    let mut step = line.point_count() as isize - 3;
    let segments_before = line.segment_count();

    // :517
    if step < 0 {
      return false;
    }

    // :520
    let mut current_path = line.shape().clone();

    for _ in 0..MERGE_PASS_LIMIT {
      // :524
      let segment_count = current_path.segment_count() as isize;
      let max_step = segment_count - 2;

      if step > max_step {
        step = max_step;
      }

      // :530
      if step < 2 {
        *line.chain_mut() = current_path;

        return line.segment_count() < segments_before;
      }

      let mut found = false;

      // :538
      for start in 0..(segment_count - step).max(0) {
        let first = current_path.segment(start as usize);
        let second = current_path.segment((start + step) as usize);

        // :544
        if !Direction45::from_seg(&first, false)
          .is_obtuse(Direction45::from_seg(&second, false))
        {
          continue;
        }

        // :546
        let Some(crossing) = first.intersect_lines(&second) else {
          continue;
        };
        let first_optimized = Seg::new(first.a, crossing);
        let second_optimized = Seg::new(crossing, second.b);

        // :551
        if !Direction45::from_seg(&first_optimized, false)
          .is_obtuse(Direction45::from_seg(&second_optimized, false))
        {
          continue;
        }

        // :553
        let optimized_path = LineChain::from_slice(
          &[first_optimized.a, first_optimized.b, second_optimized.b],
          false,
        );
        let optimized_track = Line::with_chain(line, optimized_path);

        // :560
        if !self.check_colliding(world, context, &optimized_track) {
          // :562
          current_path.replace(
            shape_index(&first) + 1,
            shape_index(&second),
            crossing,
          );
          found = true;
          break;
        }
      }

      // :573
      if !found {
        if step <= 2 {
          *line.chain_mut() = current_path;

          return line.segment_count() < segments_before;
        }

        step -= 1;
      }
    }

    *line.chain_mut() = current_path;

    line.segment_count() < segments_before
  }

  /// Drop the shared vertex of two collinear segments.
  ///
  /// Port of `OPTIMIZER::mergeColinear`,
  /// `pcbnew/router/pns_optimizer.cpp:627`. One forward pass, no
  /// collision check at all: removing a collinear vertex cannot change
  /// the geometry, only how many points describe it.
  ///
  /// # Two things the loop does not do
  ///
  /// The loop bound is re-evaluated every iteration and the index is
  /// **not** rewound after a removal, so a removal that creates a new
  /// collinear pair behind the cursor is missed; a second
  /// [`Optimizer::optimize`] call catches it. And a zero length segment
  /// is skipped rather than removed (`:639`), which in KiCad is an
  /// artefact of abutting arcs.
  ///
  /// The `!line.IsPtOnArc( segIdx + 1 )` guard at `:642` is always true
  /// without arcs and is not written out.
  pub fn merge_colinear(&self, line: &mut Line) -> bool {
    let chain = line.chain_mut();
    let segments_before = chain.segment_count();
    let mut index = 0;

    // :633
    while index + 1 < chain.segment_count() {
      let first = chain.segment(index);
      let second = chain.segment(index + 1);

      // :639
      if first.squared_length() != 0
        && second.squared_length() != 0
        && first.collinear(&second)
      {
        // :644
        chain.remove(index + 1);
      }

      index += 1;
    }

    chain.segment_count() < segments_before
  }

  // -----------------------------------------------------------------
  // The differential pair path
  // -----------------------------------------------------------------

  /// Make a differential pair shorter without decoupling it.
  ///
  /// Port of `OPTIMIZER::Optimize( DIFF_PAIR* )`,
  /// `pcbnew/router/pns_optimizer.cpp:1538`, which forwards to
  /// [`Optimizer::merge_dp_segments`] and does nothing else. The pair
  /// placer is its only caller, on a default constructed optimizer over
  /// the node it is routing in (`pcbnew/router/pns_diff_pair_placer.cpp:311`).
  ///
  /// # None of the effort flags reach this
  ///
  /// `m_effortLevel` is not read anywhere on the pair path and none of
  /// the helpers below calls `checkConstraints`, so every bit of
  /// [`EffortFlags`] is inert here, and so is
  /// [`Optimizer::set_collision_mask`]: the collision queries run with
  /// the node's default kind mask, exactly as
  /// [`Optimizer::check_colliding`] already does. That is note 07
  /// section 6's answer to "which optimizer flags are differential pair
  /// specific": none, in either direction. The pair path also never
  /// populates KiCad's item cache, which is dead anyway (see the module
  /// documentation).
  ///
  /// # What it does not do
  ///
  /// Nothing here rebuilds the pair's vias, nets, width or gap; only the
  /// two chains change, through [`crate::diff_pair::DiffPair::set_shape`].
  pub fn optimize_diff_pair(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    pair: &mut DiffPair,
  ) -> bool {
    // :1540
    self.merge_dp_segments(world, context, pair)
  }

  /// Run [`Optimizer::merge_dp_step`] over both lanes, coarse to fine.
  ///
  /// Port of `OPTIMIZER::mergeDpSegments`,
  /// `pcbnew/router/pns_optimizer.cpp:1497`. The same descending span
  /// skeleton as [`Optimizer::merge_full`], run independently on the two
  /// lanes: each lane keeps its own span length, a lane that found a
  /// merge keeps its length for the next pass, and a pass in which
  /// neither lane found anything shortens **both**.
  ///
  /// The answer is a constant `true` (`:1534`), which is what
  /// `OPTIMIZER::Optimize( DIFF_PAIR* )` hands its caller. It is not a
  /// "something changed" flag and must not be read as one.
  ///
  /// # Deviation: the loop is bounded, erratum E14
  ///
  /// KiCad's loop is `while( 1 )` and leaves only through
  /// `step_p < 1 && step_n < 1` (`:1516`), while the two counters fall
  /// only in the pass where neither lane merged (`:1528`). A
  /// [`Optimizer::merge_dp_step`] that keeps succeeding without shortening
  /// either chain therefore spins forever; the single line path is bounded
  /// by [`MERGE_PASS_LIMIT`] and this one is not. Here it is bounded by
  /// the same constant, through the private `merge_dp_passes` the
  /// pass limit test reads.
  ///
  /// The other half of E14 is transcribed rather than fixed: a span
  /// length of exactly one satisfies neither the `step > 1` guards
  /// (`:1522`, `:1525`) nor the `step < 1` exit, so it costs one pass per
  /// lane and then decrements both counters. Removing it would change
  /// how many passes a pair takes, which is observable through the bound.
  pub fn merge_dp_segments(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    pair: &mut DiffPair,
  ) -> bool {
    self.merge_dp_passes(world, context, pair, MERGE_PASS_LIMIT);

    // :1534
    true
  }

  /// The bounded body of [`Optimizer::merge_dp_segments`], answering how
  /// many passes it took.
  ///
  /// Split out so that the bound erratum E14 asks for is a value a test
  /// can read rather than an argument about termination. `limit` is
  /// [`MERGE_PASS_LIMIT`] on every path but the test's.
  fn merge_dp_passes(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    pair: &mut DiffPair,
    limit: usize,
  ) -> usize {
    // :1499, :1500
    let mut step_p = pair.chain_p().segment_count() as isize - 2;
    let mut step_n = pair.chain_n().segment_count() as isize - 2;
    let mut passes = 0;

    // :1502, bounded.
    while passes < limit {
      passes += 1;

      // :1504 to :1508
      let max_step_p = pair.chain_p().segment_count() as isize - 2;
      let max_step_n = pair.chain_n().segment_count() as isize - 2;

      // :1510, :1513
      step_p = step_p.min(max_step_p);
      step_n = step_n.min(max_step_n);

      // :1516
      if step_p < 1 && step_n < 1 {
        break;
      }

      // :1519, :1520
      let mut found_p = false;
      let mut found_n = false;

      // :1522
      if step_p > 1 {
        found_p =
          self.merge_dp_step(world, context, pair, true, step_p as usize);
      }

      // :1525
      if step_n > 1 {
        found_n =
          self.merge_dp_step(world, context, pair, false, step_n as usize);
      }

      // :1528. A span length of one lands here every time; see the
      // deviation on `Optimizer::merge_dp_segments`.
      if !found_n && !found_p {
        step_n -= 1;
        step_p -= 1;
      }
    }

    passes
  }

  /// Try to bypass one span of one lane, anywhere along it.
  ///
  /// Port of `OPTIMIZER::mergeDpStep`,
  /// `pcbnew/router/pns_optimizer.cpp:1434`. For each span of `step`
  /// segments whose outer segments meet at an obtuse angle it builds the
  /// canonical 45 degree connection between the span's outer endpoints,
  /// then asks [`Optimizer::coupled_bypass`] for a matching bypass on the
  /// other lane. `try_p` selects which lane is the reference: the P lane
  /// with it set, the N lane without.
  ///
  /// # What decides
  ///
  /// The coupled length may drop by at most a tenth of what it was. The
  /// budget is computed once from the pre merge coupled length (`:1444`,
  /// with KiCad's "fixme: come up with something more intelligent here"
  /// left where it stands) and both deltas are
  /// `new - old + budget >= 0`. Length, corner cost and the constraints
  /// play no part; [`CostEstimator`] is not consulted anywhere on this
  /// path.
  ///
  /// # Two lanes, two outcomes
  ///
  /// When [`Optimizer::coupled_bypass`] succeeds, both lanes change. When
  /// it fails, the reference lane may still be rewritten on its own,
  /// under the stricter `delta_uncoupled` test and a direct
  /// [`Optimizer::verify_dp_bypass`] (`:1480`); the coupled lane then
  /// keeps its shape apart from the [`LineChain::simplify2`] at `:1483`,
  /// which is why KiCad's `coupledPath` is a copy taken at `:1439` and
  /// why it is a copy here.
  ///
  /// # The index the replacement uses
  ///
  /// `Replace( s1.Index(), s2.Index(), bypass )` (`:1463`) splices over
  /// the span's own segment indices while the bypass ends on the point
  /// **after** the span, so the result carries a duplicate of that point
  /// until the [`LineChain::simplify2`] at `:1473`. The same one point
  /// offset appears in [`Optimizer::merge_step`], and it is reproduced
  /// for the same reason: both ranges are observable.
  pub fn merge_dp_step(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    pair: &mut DiffPair,
    try_p: bool,
    step: usize,
  ) -> bool {
    // :1438, :1439
    let current_path = if try_p {
      pair.chain_p().clone()
    } else {
      pair.chain_n().clone()
    };
    let mut coupled_path = if try_p {
      pair.chain_n().clone()
    } else {
      pair.chain_p().clone()
    };

    // :1441
    let span_end = current_path.segment_count() as isize - 1;

    // :1443, :1444
    let coupled_before =
      pair.coupled_length_of_chains(&current_path, &coupled_path);
    let budget = coupled_before / 10;

    // :1436, :1446
    let mut index: isize = 1;

    while index < span_end - step as isize {
      let start = index as usize;

      // :1448, :1449
      let first = current_path.segment(start);
      let second = current_path.segment(start + step);

      // :1451, :1452
      let first_direction = Direction45::from_seg(&first, false);
      let second_direction = Direction45::from_seg(&second, false);

      // :1454
      if first_direction.is_obtuse(second_direction) {
        // :1456
        let bypass = LineChain::from_points(
          Direction45::default().build_initial_trace(
            first.a,
            second.b,
            first_direction.is_diagonal(),
            CornerMode::Mitered45,
          ),
          false,
        );

        // :1462, :1463
        let mut new_reference = current_path.clone();

        new_reference.replace_with_chain(
          shape_index(&first),
          shape_index(&second),
          &bypass,
        );

        // :1465
        let delta_uncoupled = pair
          .coupled_length_of_chains(&new_reference, &coupled_path)
          - coupled_before
          + budget;

        // :1467
        if let Some(mut new_coupled) = self.coupled_bypass(
          world,
          context,
          pair,
          try_p,
          &new_reference,
          &bypass,
          &coupled_path,
        ) {
          // :1469
          let delta_coupled = pair
            .coupled_length_of_chains(&new_reference, &new_coupled)
            - coupled_before
            + budget;

          // :1471
          if delta_coupled >= 0 {
            // :1473, :1474
            new_reference.simplify2(true);
            new_coupled.simplify2(true);

            // :1476
            pair.set_shape(new_reference, new_coupled, !try_p);

            return true;
          }
        } else if delta_uncoupled >= 0
          && self.verify_dp_bypass(
            world,
            context,
            pair,
            try_p,
            &new_reference,
            &coupled_path,
          )
        {
          // :1482, :1483
          new_reference.simplify2(true);
          coupled_path.simplify2(true);

          // :1485
          pair.set_shape(new_reference, coupled_path, !try_p);

          return true;
        }
      }

      // :1490
      index += 1;
    }

    false
  }

  /// Find the bypass on the coupled lane that keeps the most coupling.
  ///
  /// Port of `coupledBypass`, `pcbnew/router/pns_optimizer.cpp:1369`. It
  /// starts from every vertex of the coupled lane that
  /// [`find_coupled_vertices`] found opposite the reference bypass's
  /// first vertex, runs a 45 degree connection from there to every
  /// interior vertex of the lane more than one index away, and keeps the
  /// candidate with the greatest coupled length that
  /// [`Optimizer::verify_dp_bypass`] accepts. [`None`] is KiCad's `false`.
  ///
  /// # A segment index used as a point index
  ///
  /// [`find_coupled_vertices`] answers with **segment** indices (`:1340`)
  /// and this reads them as **point** indices (`:1392`, `:1400`). Segment
  /// `i` starts at point `i`, so what the code means by it is "the start
  /// point of that segment", and the conflation is harmless because a
  /// segment index is always a valid point index. It is transcribed as it
  /// stands.
  ///
  /// # Deviations
  ///
  /// KiCad collects the start indices into `int vStartIdx[1024]` with the
  /// comment "fixme: possible overflow" and no bound
  /// (`:1373`); a lane with more than 1024 parallel segments at the gap
  /// walks off the end of it. Here it is a [`Vec`] and the question does
  /// not arise.
  ///
  /// KiCad reads `aRefBypass.CPoint( 0 )` and `aRefBypass.CSegment( 0 )`
  /// unguarded (`:1374`). A bypass between two coincident points is a one
  /// point chain with no segment, which [`LineChain::segment`] refuses, so
  /// that case answers [`None`] here instead of reading out of bounds.
  #[allow(clippy::too_many_arguments)]
  pub fn coupled_bypass(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    pair: &DiffPair,
    reference_is_p: bool,
    reference: &LineChain,
    reference_bypass: &LineChain,
    coupled: &LineChain,
  ) -> Option<LineChain> {
    if reference_bypass.segment_count() == 0 {
      return None;
    }

    let opening = reference_bypass.segment(0);

    // :1374
    let starts =
      find_coupled_vertices(reference_bypass.point(0), opening, coupled, pair);

    // :1377
    let direction = Direction45::from_seg(&opening, false);

    // :1379, :1380
    let mut best_length: i64 = -1;
    let mut best: Option<LineChain> = None;

    // :1384
    for start in starts {
      // :1386
      for end in 1..coupled.point_count().saturating_sub(1) {
        // :1388, :1390
        if start.abs_diff(end) <= 1 {
          continue;
        }

        // :1392, :1393
        let bypass = LineChain::from_points(
          direction.build_initial_trace(
            coupled.point(start),
            coupled.point(end),
            direction.is_diagonal(),
            CornerMode::Mitered45,
          ),
          false,
        );

        // :1396
        let coupled_length = pair.coupled_length_of_chains(reference, &bypass);

        // :1398 to :1406. The reversal is what keeps the lane's own
        // direction when the search runs backwards along it.
        let mut candidate = coupled.clone();

        if start < end {
          candidate.replace_with_chain(start, end, &bypass);
        } else {
          candidate.replace_with_chain(end, start, &bypass.reversed());
        }

        // :1408. The score is read before the verification, so a
        // candidate that scores no better is never checked against the
        // node.
        if coupled_length > best_length
          && self.verify_dp_bypass(
            world,
            context,
            pair,
            reference_is_p,
            reference,
            &candidate,
          )
        {
          // :1411 to :1413
          best_length = coupled_length;
          best = Some(candidate);
        }
      }
    }

    // :1419, :1422
    best
  }

  /// Whether two candidate lane shapes clear each other and the node.
  ///
  /// Port of `verifyDpBypass`,
  /// `pcbnew/router/pns_optimizer.cpp:1350`. The two chains are wrapped
  /// in lines built from the pair's own lanes, so they carry the pair's
  /// width, net and layer, and then three questions are asked: do the two
  /// lanes hit each other, does the reference lane hit the node, does the
  /// coupled lane hit the node.
  ///
  /// # The lane to lane test is an ordinary clearance test
  ///
  /// It goes through the rule resolver like any other pair of items, not
  /// through the forced pair gap `attemptWalk` installs
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:247`). So the optimizer will
  /// happily bring the two lanes closer together than the pair gap as
  /// long as the netclass clearance allows it. The only thing pushing
  /// back is the coupled length, and only as a score; see
  /// [`Optimizer::merge_dp_step`].
  ///
  /// # The end vias are dropped
  ///
  /// `PLine()` and `NLine()` attach the pair's end via when it has one
  /// (`pns_diff_pair.h:491`, through `updateLine` at `:545`), and the
  /// `LINE( const LINE&, const SHAPE_LINE_CHAIN& )` constructor that
  /// wraps them here sets `m_via = nullptr` (`pns_line.h:90`). So a pair
  /// that ends with vias is verified without them and a bypass whose end
  /// via would collide is accepted. [`Line::with_chain`] drops the via
  /// for the same reason, so the behaviour comes for free; note 07
  /// section 6.3 does not mention it.
  ///
  /// # The caller contract of the module documentation applies
  ///
  /// Both candidates are checked against the node with
  /// [`Optimizer::check_colliding`], so the pair's own segments must not
  /// be in it. `OPTIMIZER::Optimize( DIFF_PAIR* )` is called on a pair
  /// that has not been fixed yet, which is how KiCad satisfies that.
  ///
  /// `checkDpColliding` (`:1426`) does the same node query in two lines
  /// and has no caller anywhere in KiCad's tree; erratum E15, not ported.
  pub fn verify_dp_bypass(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    pair: &DiffPair,
    reference_is_p: bool,
    new_reference: &LineChain,
    new_coupled: &LineChain,
  ) -> bool {
    // :1353, :1354
    let reference_base = if reference_is_p {
      pair.p_line()
    } else {
      pair.n_line()
    };
    let coupled_base = if reference_is_p {
      pair.n_line()
    } else {
      pair.p_line()
    };
    let reference_line =
      Line::with_chain(&reference_base, new_reference.clone());
    let coupled_line = Line::with_chain(&coupled_base, new_coupled.clone());

    let options = CollisionSearchOptions {
      limit_count: Some(1),
      ..CollisionSearchOptions::default()
    };

    // :1356. `refLine.Collide( &coupledLine, aNode, refLine.Layer() )`
    // has the reference lane as `this` and the coupled one as the head,
    // which is the way round `World::collide_lines` takes them. The layer
    // argument has no counterpart: both lines already carry the pair's
    // layer.
    if world
      .collide_lines(&reference_line, &coupled_line, context.resolver, &options)
      .is_some()
    {
      return false;
    }

    // :1359
    if self.check_colliding(world, context, &reference_line) {
      return false;
    }

    // :1362
    if self.check_colliding(world, context, &coupled_line) {
      return false;
    }

    // :1365
    true
  }

  /// Redraw the exit from one pad so that the trace leaves it cleanly.
  ///
  /// Port of `OPTIMIZER::smartPadsSingle`,
  /// `pcbnew/router/pns_optimizer.cpp:1110`. It builds every combination
  /// of a breakout from the pad and a two segment connection from that
  /// breakout back onto the line, keeps the ones that have no forbidden
  /// corner anywhere, and picks the cheapest of those that does not
  /// collide. The answer is the point index the winner rejoined the line
  /// at, which [`Optimizer::run_smart_pads`] spends out of the budget for
  /// the other end, or [`None`] for KiCad's `-1`.
  ///
  /// `at_end` is KiCad's `aEnd`: with it set the line is reversed first,
  /// so that "the pad" is always at point zero, and the winner is
  /// reversed back before it is stored.
  ///
  /// `end_vertex` is KiCad's `aEndVertex` and is signed because
  /// [`Optimizer::run_smart_pads`] can compute a negative budget for the
  /// far end; the loop then simply does not run.
  ///
  /// # What it refuses outright
  ///
  /// A pad whose copper is offset from its centre (`:1123`), because
  /// every breakout starts at the centre and would begin outside the
  /// copper. And a via (`:1128`), with KiCad's reason transcribed: vias
  /// are round, so the eight breakouts are indistinguishable and the pass
  /// would only destroy a deliberate via exit posture. That is why
  /// [`compute_breakouts`]'s via branch is unreachable from here.
  ///
  /// # The tie break
  ///
  /// The baseline cost is the cost of the line the user drew (`:1193`),
  /// so a candidate has to be strictly better to win. Equal cost
  /// candidates are then ordered by **breakout** length, longest first
  /// (`:1207`), which on an oblong pad picks the exit that runs along the
  /// pad's long axis before leaving. KiCad's comment at `:1188` explains
  /// it: a track should not leave an oblong pad from its short side and
  /// then run alongside it.
  ///
  /// # Two quirks reproduced
  ///
  /// The connection is built with `diag == 0` as the diagonal first flag
  /// (`:1151`), so posture 0 is the diagonal one here where it is the
  /// straight one in [`Optimizer::merge_step`] and in
  /// [`Optimizer::fanout_cleanup`]. And the connection is built with no
  /// corner mode argument, so it always uses
  /// [`crate::geometry::direction45::CornerMode::Mitered45`] even when
  /// the board is being routed at 90 degrees; the line placer works
  /// around that by not asking for [`EffortFlags::SMART_PADS`] in the 90
  /// degree modes at all (`pcbnew/router/pns_line_placer.cpp:762`).
  ///
  /// # Deviation
  ///
  /// KiCad widens the running longest breakout with
  /// `std::max<int>( len, max_length )` (`:1213`), narrowing a
  /// `long long` to `int` on the way in. The comparison one line above is
  /// done at full width, so the narrowing only matters for a breakout
  /// longer than `i32::MAX` nanometres, that is two metres. This keeps
  /// the full width.
  pub fn smart_pads_single(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &mut Line,
    pad: ItemId,
    at_end: bool,
    end_vertex: isize,
  ) -> Option<usize> {
    let pad_item = world.item(pad)?;
    let solid = match pad_item.body() {
      ItemBody::Solid(solid) => Some(solid),
      _ => None,
    };

    // :1123
    if let Some(solid) = solid
      && solid.offset() != Vec2::new(0, 0)
    {
      return None;
    }

    // :1128
    if pad_item.of_kind(Kind::VIA) {
      return None;
    }

    // :1131
    let breakouts = compute_breakouts(line.width(), pad_item, true);

    // :1132. With `at_end` the pad is put at point zero.
    let path = if at_end {
      line.shape().reversed()
    } else {
      line.shape().clone()
    };

    // :1133
    let last_vertex = end_vertex
      .min(SMART_PADS_MAX_VERTEX.min(path.point_count() as isize - 1));

    // Every accepted rewrite: where it rejoined, how long its breakout
    // was, and the whole candidate in the line's own direction.
    let mut variants: Vec<(usize, i64, LineChain)> = Vec::new();
    let path_length = path.length();

    // :1136. Point zero is the pad connection itself, so start at one.
    for vertex in 1..=last_vertex.max(0) {
      let vertex = vertex as usize;

      // :1139. When the span from the pad out to this vertex does not
      // even touch the copper, the line has already left the pad and
      // there is nothing to redraw from here.
      if let Some(shape) = solid.and_then(Solid::shape)
        && collide_seg(
          shape,
          &Seg::new(path.point(0), path.point(vertex)),
          line.width() / 2,
        )
        .is_none()
      {
        continue;
      }

      for breakout in &breakouts {
        // :1153. The direction the breakout leaves in.
        let Some(last) = breakout.segment_count().checked_sub(1) else {
          continue;
        };
        let Some(exit) = breakout.last_point() else {
          continue;
        };
        let breakout_direction =
          Direction45::from_seg(&breakout.segment(last), false);

        for posture in 0..POSTURE_COUNT {
          // :1150. Note the inverted posture flag, see the doc comment.
          let connection = LineChain::from_points(
            Direction45::default().build_initial_trace(
              exit,
              path.point(vertex),
              posture == 0,
              CornerMode::Mitered45,
            ),
            false,
          );

          // :1155
          if connection.segment_count() == 0 {
            continue;
          }

          // :1159
          if breakout_direction
            .angle(Direction45::from_seg(&connection.segment(0), false))
            .intersects(FORBIDDEN_ANGLES)
          {
            continue;
          }

          // :1164. A breakout longer than the whole line is not an exit.
          let breakout_length = breakout.length();

          if breakout_length > path_length {
            continue;
          }

          // :1166
          let mut candidate = breakout.clone();

          candidate.append_chain(&connection);

          for index in vertex + 1..path.point_count() {
            candidate.append(path.point(index));
          }

          // :1172
          if Line::with_chain(line, candidate.clone())
            .count_corners(FORBIDDEN_ANGLES)
            != 0
          {
            continue;
          }

          // :1177
          let mut stored = if at_end {
            candidate.reversed()
          } else {
            candidate
          };

          stored.simplify2(true);
          variants.push((vertex, breakout_length, stored));
        }
      }
    }

    // :1193. The line the user drew is the baseline to beat.
    let mut min_cost = CostEstimator::corner_cost_of_line(line);
    let mut max_length: i64 = 0;
    let mut best: Option<(usize, LineChain)> = None;

    // :1199
    for (vertex, breakout_length, candidate) in &variants {
      let cost = CostEstimator::corner_cost_of_chain(candidate);

      // :1205
      if self.check_colliding_path(world, context, line, candidate) {
        continue;
      }

      // :1207
      if cost < min_cost || (cost == min_cost && *breakout_length > max_length)
      {
        best = Some((*vertex, candidate.clone()));

        if cost <= min_cost {
          max_length = max_length.max(*breakout_length);
        }

        min_cost = min_cost.min(cost);
      }
    }

    // :1221
    let (vertex, shape) = best?;

    line.set_shape(shape);

    Some(vertex)
  }

  /// Redraw the exits from the pads at both ends of a line.
  ///
  /// Port of `OPTIMIZER::runSmartPads`,
  /// `pcbnew/router/pns_optimizer.cpp:1231`. The start pad is rewritten
  /// first, with a budget of [`SMART_PADS_MAX_VERTEX`] points; the end
  /// pad then gets whatever is left of the line, minus what the start
  /// pass consumed.
  ///
  /// # It always reports "changed"
  ///
  /// The `return true` at `:1254` is unconditional: as long as the line
  /// has three points, this pass reports that it changed something even
  /// when both ends had no pad and nothing was touched. It is transcribed
  /// as it stands, so [`Optimizer::optimize`] answers `true` for any
  /// effort level containing [`EffortFlags::SMART_PADS`] over a line of
  /// three points or more, exactly as KiCad's `rv |= runSmartPads` does.
  /// That matters to callers that only write the result back on a `true`
  /// answer, such as `SHOVE::runOptimizer`
  /// (`pcbnew/router/pns_shove.cpp:2121`): asking for this flag makes
  /// them always write back.
  ///
  /// # The budget is read after the first pass
  ///
  /// KiCad holds `line` as a reference into the line's own chain
  /// (`:1233`), so the `line.PointCount()` at `:1250` is the count
  /// **after** the start pass rewrote it, not the one the pass started
  /// with. That is reproduced by reading [`Line::point_count`] again, and
  /// it is why the budget can come out negative and why
  /// [`Optimizer::smart_pads_single`] takes a signed one.
  pub fn run_smart_pads(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &mut Line,
  ) -> bool {
    // :1235
    if line.point_count() < 3 {
      return false;
    }

    let start_point = line.point(0);
    let Some(end_point) = line.last_point() else {
      return false;
    };

    // :1240
    let start_pad =
      find_pad_or_via(world, self.node, line.layer(), line.net(), start_point);
    let end_pad =
      find_pad_or_via(world, self.node, line.layer(), line.net(), end_point);

    // :1244
    let mut spent = None;

    // :1245
    if let Some(pad) = start_pad {
      spent = self.smart_pads_single(
        world,
        context,
        line,
        pad,
        false,
        SMART_PADS_MAX_VERTEX,
      );
    }

    // :1248
    if let Some(pad) = end_pad {
      let remaining = line.point_count() as isize - 1;
      let budget = match spent {
        None => remaining,
        Some(vertex) => remaining - vertex as isize,
      };

      self.smart_pads_single(world, context, line, pad, true, budget);
    }

    // :1252
    line.chain_mut().simplify2(true);

    // :1254
    true
  }

  /// Redraw a very short pad to pad or pad to via connection as a plain
  /// two segment trace.
  ///
  /// Port of `OPTIMIZER::fanoutCleanup`,
  /// `pcbnew/router/pns_optimizer.cpp:1273`. When both ends of a line sit
  /// on a pad or a via, or the far end carries the line's own via, and
  /// the line is shorter than [`FANOUT_CLEANUP_WIDTH_FACTOR`] track
  /// widths, the whole thing is replaced by whichever of the two
  /// [`Direction45::build_initial_trace`] postures does not collide. This
  /// is the "two pads next to each other, just draw the L" cleanup, and
  /// it is why the line placer notes that the flag can override the
  /// user's posture choice (`pcbnew/router/pns_line_placer.cpp:1044`).
  ///
  /// The corner mode comes from
  /// [`crate::settings::RoutingSettings::corner_mode`] off `context`,
  /// where KiCad reads it off the router singleton at `:1278`; that
  /// singleton use is the one note 04 section 9.3 names as making the
  /// routine untestable in C++.
  ///
  /// # The start end asymmetry
  ///
  /// A missing pad at the **start** ends the pass (`:1288`), while a
  /// missing one at the end falls back to [`Line::ends_with_via`]
  /// (`:1300`). So a line that starts in open copper is never cleaned up
  /// even if it ends on a pad. That is KiCad's and it is reproduced.
  ///
  /// `start_match` is a second kind test on an item
  /// [`find_pad_or_via`] already filtered by kind, so it is always true;
  /// it is written out because KiCad writes it out.
  ///
  /// # Deviation
  ///
  /// KiCad holds both the length and the threshold in `int` (`:1285`,
  /// `:1286`), which wraps for a line longer than two metres. Both are
  /// `i64` here.
  pub fn fanout_cleanup(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &mut Line,
  ) -> bool {
    // :1275
    if line.point_count() < 3 {
      return false;
    }

    // :1278
    let corner_mode = context.settings.corner_mode;

    // :1280
    let start_point = line.point(0);
    let Some(end_point) = line.last_point() else {
      return false;
    };

    let start_pad =
      find_pad_or_via(world, self.node, line.layer(), line.net(), start_point);
    let end_pad =
      find_pad_or_via(world, self.node, line.layer(), line.net(), end_point);

    // :1285
    let threshold = i64::from(line.width()) * FANOUT_CLEANUP_WIDTH_FACTOR;
    let length = line.shape().length();

    // :1288
    let Some(start_pad) = start_pad else {
      return false;
    };

    // :1291
    let start_match = world
      .item(start_pad)
      .is_some_and(|item| item.of_kind(Kind::VIA | Kind::SOLID));

    // :1294
    let end_match = match end_pad {
      Some(pad) => world
        .item(pad)
        .is_some_and(|item| item.of_kind(Kind::VIA | Kind::SOLID)),
      None => line.ends_with_via(),
    };

    // :1303
    if !(start_match && end_match && length < threshold) {
      return false;
    }

    // :1305. Posture 0 is straight leg first, posture 1 diagonal first,
    // the same way round as in `merge_step` and the other way round from
    // `smart_pads_single`.
    for posture in 0..POSTURE_COUNT {
      let candidate = LineChain::from_points(
        Direction45::default().build_initial_trace(
          start_point,
          end_point,
          posture == 1,
          corner_mode,
        ),
        false,
      );
      let replacement = Line::with_chain(line, candidate);

      // :1311
      if !self.check_colliding(world, context, &replacement) {
        line.set_shape(replacement.shape().clone());

        return true;
      }
    }

    false
  }
}

// ---------------------------------------------------------------------
// The differential pair path
// ---------------------------------------------------------------------

/// Every segment of one lane that faces a vertex of the other at the
/// pair's gap.
///
/// Port of `findCoupledVertices`,
/// `pcbnew/router/pns_optimizer.cpp:1323`, a free function in KiCad too.
/// `vertex` and `original` are the first point and the first segment of
/// the reference lane's bypass, and the answer is the **segment** indices
/// of `coupled` that are [`Seg::approx_parallel`] to `original` and whose
/// line passes the pair's gap constraint at `vertex`.
///
/// This is the only reader of
/// [`crate::diff_pair::DiffPair::gap_constraint`] outside
/// `pns_diff_pair.cpp`, and like the two coupled length routines it wants
/// the **edge to edge** value: the pair's width is subtracted from the
/// centre to centre distance before the constraint is asked. See the
/// `diff_pair` module documentation on the two meanings of the gap.
///
/// # It does not take the absolute value
///
/// `CoupledSegmentPairs` and `CoupledLength` both wrap the distance in
/// `std::abs` (`pns_diff_pair.cpp:855`, `:881`); this one does not
/// (`:1335`). It matters for a lane narrower than the pair's own width,
/// where the difference goes negative and no positive gap constraint can
/// match it. Transcribed as it stands.
///
/// KiCad projects the vertex onto every segment before testing whether
/// that segment is parallel at all (`:1331`); the projection is moved
/// under the test here, which changes nothing and saves the work.
pub fn find_coupled_vertices(
  vertex: Vec2,
  original: Seg,
  coupled: &LineChain,
  pair: &DiffPair,
) -> Vec<usize> {
  let mut indices = Vec::new();

  // :1328
  for index in 0..coupled.segment_count() {
    let segment = coupled.segment(index);

    // :1333
    if !segment.approx_parallel(&original, Seg::APPROX_DISTANCE_THRESHOLD) {
      continue;
    }

    // :1331, :1335
    let projected = segment.line_project(vertex);
    let distance = i64::from((projected - vertex).euclidean_norm())
      - i64::from(pair.width());

    // :1338
    if pair.gap_constraint().matches(distance) {
      // :1340, :1341
      indices.push(index);
    }
  }

  // :1346
  indices
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::item::{LayerRange, Segment, Via, ViaType};
  use crate::rules::FixedClearance;
  use crate::settings::RoutingSettings;

  /// A two point segment at the given coordinates.
  fn seg(x1: i32, y1: i32, x2: i32, y2: i32) -> Seg {
    Seg::from_coords(x1, y1, x2, y2)
  }

  #[test]
  fn the_effort_flags_carry_kicads_values() {
    assert_eq!(EffortFlags::MERGE_SEGMENTS.bits(), 0x001);
    assert_eq!(EffortFlags::SMART_PADS.bits(), 0x002);
    assert_eq!(EffortFlags::MERGE_OBTUSE.bits(), 0x004);
    assert_eq!(EffortFlags::FANOUT_CLEANUP.bits(), 0x008);
    assert_eq!(EffortFlags::KEEP_TOPOLOGY.bits(), 0x010);
    assert_eq!(EffortFlags::PRESERVE_VERTEX.bits(), 0x020);
    assert_eq!(EffortFlags::RESTRICT_VERTEX_RANGE.bits(), 0x040);
    assert_eq!(EffortFlags::MERGE_COLINEAR.bits(), 0x080);
    assert_eq!(EffortFlags::RESTRICT_AREA.bits(), 0x100);
    assert_eq!(EffortFlags::LIMIT_CORNER_COUNT.bits(), 0x200);
  }

  #[test]
  fn an_effort_level_round_trips_through_its_bits() {
    let effort = EffortFlags::MERGE_SEGMENTS | EffortFlags::MERGE_COLINEAR;

    assert_eq!(EffortFlags::from_bits(effort.bits()), effort);
    assert!(effort.contains(EffortFlags::MERGE_SEGMENTS));
    assert!(!effort.contains(EffortFlags::MERGE_OBTUSE));
    assert!(effort.intersects(EffortFlags::MERGE_COLINEAR));

    // An unknown bit survives the round trip, which is what lets a host
    // hand KiCad's `REQUIRE_OBTUSE_ANGLES` straight through.
    assert_eq!(EffortFlags::from_bits(0x400).bits(), 0x400);
  }

  #[test]
  fn the_corner_cost_table_is_kicads() {
    let east = seg(0, 0, 100, 0);

    // Straight on: the same direction again.
    assert_eq!(
      CostEstimator::corner_cost(&east, &seg(100, 0, 200, 0)),
      COST_STRAIGHT
    );
    // 45 degrees.
    assert_eq!(
      CostEstimator::corner_cost(&east, &seg(100, 0, 200, -100)),
      COST_OBTUSE
    );
    // 90 degrees.
    assert_eq!(
      CostEstimator::corner_cost(&east, &seg(100, 0, 100, -100)),
      COST_RIGHT
    );
    // 135 degrees.
    assert_eq!(
      CostEstimator::corner_cost(&east, &seg(100, 0, 0, -100)),
      COST_ACUTE
    );
    // A hairpin.
    assert_eq!(
      CostEstimator::corner_cost(&east, &seg(100, 0, 0, 0)),
      COST_HALF_FULL
    );
    // Off the grid, and the degenerate segment that gives an undefined
    // direction.
    assert_eq!(
      CostEstimator::corner_cost(&east, &seg(100, 0, 100, 0)),
      COST_UNDEFINED
    );
  }

  #[test]
  fn a_chains_cost_is_the_sum_over_its_corners() {
    // A staircase: east, north, east. Two right angle corners.
    let chain = LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(100, 0),
        Vec2::new(100, -100),
        Vec2::new(200, -100),
      ],
      false,
    );

    assert_eq!(CostEstimator::corner_cost_of_chain(&chain), COST_RIGHT * 2);

    // Fewer than two segments has no corner.
    let short =
      LineChain::from_slice(&[Vec2::new(0, 0), Vec2::new(1, 0)], false);

    assert_eq!(CostEstimator::corner_cost_of_chain(&short), 0);
  }

  #[test]
  fn merge_colinear_joins_two_collinear_segments() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let optimizer = Optimizer::new(root);
    let mut line = Line::new();

    line.set_shape(LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(100, 0), Vec2::new(200, 0)],
      false,
    ));

    assert!(optimizer.merge_colinear(&mut line));
    assert_eq!(line.point_count(), 2);
    assert_eq!(line.point(1), Vec2::new(200, 0));
  }

  #[test]
  fn merge_colinear_leaves_a_real_corner_alone() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let optimizer = Optimizer::new(root);
    let mut line = Line::new();

    line.set_shape(LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(100, 0), Vec2::new(100, -100)],
      false,
    ));

    assert!(!optimizer.merge_colinear(&mut line));
    assert_eq!(line.point_count(), 3);
  }

  #[test]
  fn the_area_constraint_allows_a_span_inside_the_box() {
    let path = LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(100, 0), Vec2::new(100, -100)],
      false,
    );
    let replacement =
      LineChain::from_slice(&[Vec2::new(0, 0), Vec2::new(100, -100)], false);
    let box_inside =
      Box2::from_corners(Vec2L::new(-1000, -1000), Vec2L::new(1000, 1000));
    let origin = Line::new();
    let candidate = Candidate {
      vertex1: 0,
      vertex2: 2,
      origin_line: &origin,
      current_path: &path,
      replacement: &replacement,
    };

    assert!(check_area(box_inside, &candidate));

    // A box that holds neither endpoint refuses.
    let box_elsewhere = Box2::from_corners(
      Vec2L::new(10_000, 10_000),
      Vec2L::new(20_000, 20_000),
    );

    assert!(!check_area(box_elsewhere, &candidate));
  }

  #[test]
  fn the_preserve_vertex_constraint_only_bites_on_a_covered_point() {
    let path = LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(100, 0), Vec2::new(100, -100)],
      false,
    );
    let replacement =
      LineChain::from_slice(&[Vec2::new(0, 0), Vec2::new(100, -100)], false);
    let origin = Line::new();
    let candidate = Candidate {
      vertex1: 0,
      vertex2: 2,
      origin_line: &origin,
      current_path: &path,
      replacement: &replacement,
    };

    // The corner lies on the span and not on the replacement.
    assert!(!check_preserve_vertex(Vec2::new(100, 0), &candidate));

    // A point nowhere near the span does not constrain anything.
    assert!(check_preserve_vertex(Vec2::new(5000, 5000), &candidate));

    // A point the replacement also passes through is fine.
    assert!(check_preserve_vertex(Vec2::new(0, 0), &candidate));
  }

  /// A bypass whose own two segments meet at an acute corner is refused,
  /// and the obtuse bypass over the same span is taken.
  ///
  /// The path is a straight run along x, so the two seams at `:333` and
  /// `:339` are not what decides either answer; only the corner inside
  /// the replacement is.
  #[test]
  fn the_obtuse_only_constraint_judges_the_corner_inside_a_replacement() {
    let path = LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(1000, 0),
        Vec2::new(2000, 0),
        Vec2::new(3000, 0),
      ],
      false,
    );
    let origin = Line::new();

    // Out along a 45 degree diagonal and straight back: the two legs meet
    // at 45 degrees, which is `ANG_ACUTE`.
    let acute = LineChain::from_slice(
      &[
        Vec2::new(1000, 0),
        Vec2::new(1500, -500),
        Vec2::new(2000, 0),
      ],
      false,
    );

    assert!(!check_obtuse_only(&Candidate {
      vertex1: 1,
      vertex2: 2,
      origin_line: &origin,
      current_path: &path,
      replacement: &acute,
    }));

    // The same span bridged by a straight leg and a 45 degree one, which
    // meet at 135 degrees: `ANG_OBTUSE`.
    let obtuse = LineChain::from_slice(
      &[
        Vec2::new(1000, 0),
        Vec2::new(1500, 0),
        Vec2::new(2000, -500),
      ],
      false,
    );

    assert!(check_obtuse_only(&Candidate {
      vertex1: 1,
      vertex2: 2,
      origin_line: &origin,
      current_path: &path,
      replacement: &obtuse,
    }));
  }

  /// The two seams where a replacement joins the path are judged as well,
  /// and a right angle is refused there.
  ///
  /// The replacement is a single segment, so the loop at `:322` runs zero
  /// times and only `:333` and `:339` can answer.
  #[test]
  fn the_obtuse_only_constraint_judges_both_seams() {
    let origin = Line::new();
    // A path that turns through a right angle at (1000, 0).
    let path = LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(1000, 0), Vec2::new(1000, 1000)],
      false,
    );
    // Replacing the second segment by itself leaves that right angle as
    // the seam at `aVertex1 - 1`.
    let same = LineChain::from_slice(
      &[Vec2::new(1000, 0), Vec2::new(1000, 1000)],
      false,
    );

    assert!(!check_obtuse_only(&Candidate {
      vertex1: 1,
      vertex2: 2,
      origin_line: &origin,
      current_path: &path,
      replacement: &same,
    }));

    // The mirror image: the seam at `aVertex2` is the right angle.
    assert!(!check_obtuse_only(&Candidate {
      vertex1: 0,
      vertex2: 1,
      origin_line: &origin,
      current_path: &path,
      replacement: &LineChain::from_slice(
        &[Vec2::new(0, 0), Vec2::new(1000, 0)],
        false,
      ),
    }));

    // A span at the very start of the path has no seam before it, and a
    // span running to the very end has none after it, so a lone segment
    // covering the whole path is always allowed.
    let straight =
      LineChain::from_slice(&[Vec2::new(0, 0), Vec2::new(2000, 0)], false);

    assert!(check_obtuse_only(&Candidate {
      vertex1: 0,
      vertex2: 2,
      origin_line: &origin,
      current_path: &straight,
      replacement: &straight,
    }));
  }

  /// An empty replacement is allowed, which is `:319`.
  #[test]
  fn the_obtuse_only_constraint_allows_an_empty_replacement() {
    let origin = Line::new();
    let path = LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(1000, 0), Vec2::new(1000, 1000)],
      false,
    );
    let empty = LineChain::new();

    assert!(check_obtuse_only(&Candidate {
      vertex1: 1,
      vertex2: 2,
      origin_line: &origin,
      current_path: &path,
      replacement: &empty,
    }));
  }

  /// The flag reaches the constraint list through
  /// [`Optimizer::optimize`]'s assembly at `:703`.
  #[test]
  fn require_obtuse_angles_adds_the_obtuse_only_constraint() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let resolver = FixedClearance::uniform(100);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&resolver, &settings);
    let mut optimizer = Optimizer::new(root);

    optimizer.set_effort_level(EffortFlags::REQUIRE_OBTUSE_ANGLES);

    let line = Line::new();
    let mut result = Line::new();

    optimizer.optimize(&world, &context, &line, &mut result, None);

    assert_eq!(optimizer.constraints(), &[Constraint::ObtuseOnly]);
  }

  #[test]
  fn point_inside2_counts_the_boundary_as_inside() {
    let square = LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(100, 0),
        Vec2::new(100, 100),
        Vec2::new(0, 100),
      ],
      true,
    );

    assert!(point_inside2(&square, Vec2::new(50, 50)));
    assert!(point_inside2(&square, Vec2::new(0, 50)));
    assert!(point_inside2(&square, Vec2::new(0, 0)));
    assert!(!point_inside2(&square, Vec2::new(150, 50)));

    // An open chain has no inside.
    let mut open = square.clone();

    open.set_closed(false);
    assert!(!point_inside2(&open, Vec2::new(50, 50)));
  }

  #[test]
  fn the_restrict_vertex_range_setter_only_sets_its_flag() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let mut optimizer = Optimizer::new(root);

    optimizer.set_effort_level(EffortFlags::NONE);
    optimizer.set_restrict_vertex_range(1, 4);

    assert_eq!(optimizer.effort_level(), EffortFlags::RESTRICT_VERTEX_RANGE);
    assert!(optimizer.constraints().is_empty());
  }

  // -----------------------------------------------------------------
  // Pads and breakouts
  // -----------------------------------------------------------------

  /// A pad centred on a point, on one layer and one net.
  fn pad(world: &mut World, shape: Shape, at: Vec2, layer: i32) -> ItemId {
    let body = ItemBody::Solid(Solid::new(shape, at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(Some(NetId(1)));

    let root = world.root();

    world.add_solid(root, item, None)
  }

  /// The endpoints of a breakout list, which is what the hand computed
  /// tables below pin.
  fn ends(breakouts: &BreakoutList) -> Vec<Vec2> {
    breakouts
      .iter()
      .map(|chain| chain.last_point().expect("a breakout has points"))
      .collect()
  }

  #[test]
  fn rotate_point_reproduces_kicads_four_exact_quadrants() {
    let point = Vec2::new(1000, 0);

    assert_eq!(rotate_point(point, 0.0), Vec2::new(1000, 0));
    assert_eq!(rotate_point(point, -0.0), Vec2::new(1000, 0));
    assert_eq!(rotate_point(point, 90.0), Vec2::new(0, -1000));
    assert_eq!(rotate_point(point, 180.0), Vec2::new(-1000, 0));
    assert_eq!(rotate_point(point, 270.0), Vec2::new(0, 1000));
    // A full turn normalizes back onto the "no rotation" branch.
    assert_eq!(rotate_point(point, 360.0), Vec2::new(1000, 0));
    // And the 45 degree cases go through the exact sine table.
    assert_eq!(rotate_point(point, -45.0), Vec2::new(707, 707));
    assert_eq!(rotate_point(point, 45.0), Vec2::new(707, -707));
  }

  #[test]
  fn rotate_point_about_a_centre_translates_first() {
    let center = Vec2::new(5000, 5000);
    let point = Vec2::new(6000, 5000);

    assert_eq!(
      rotate_point_about(point, center, -90.0),
      Vec2::new(5000, 6000)
    );
    assert_eq!(
      rotate_point_about(point, center, 180.0),
      Vec2::new(4000, 5000)
    );
  }

  /// The eight rays of a circle of radius 100000, hand computed from
  /// `pcbnew/router/pns_optimizer.cpp:929`: the ray is
  /// `trunc(100000 * sqrt(2)) = 141421` long, so the four axis aligned
  /// exits end at 141421 and the four diagonal ones at
  /// `round(141421 / sqrt(2)) = 100000` in each axis, which is the corner
  /// of the circle's bounding box.
  #[test]
  fn circle_breakouts_are_eight_rays_of_radius_root_two() {
    let shape = Shape::circle(Vec2::new(0, 0), 100_000);
    let breakouts = circle_breakouts(100_000, &shape, true);

    assert_eq!(breakouts.len(), CIRCLE_BREAKOUT_COUNT);
    assert_eq!(
      ends(&breakouts),
      vec![
        Vec2::new(141_421, 0),
        Vec2::new(100_000, 100_000),
        Vec2::new(0, 141_421),
        Vec2::new(-100_000, 100_000),
        Vec2::new(-141_421, 0),
        Vec2::new(-100_000, -100_000),
        Vec2::new(0, -141_421),
        Vec2::new(100_000, -100_000),
      ]
    );

    // Every one of them starts at the centre and is a single segment.
    for breakout in &breakouts {
      assert_eq!(breakout.point(0), Vec2::new(0, 0));
      assert_eq!(breakout.segment_count(), 1);
    }
  }

  /// `circleBreakouts` has no branch on `aPermitDiagonal`
  /// (`pcbnew/router/pns_optimizer.cpp:924`), so a round pad always
  /// offers all eight.
  #[test]
  fn circle_breakouts_ignore_the_diagonal_flag() {
    let shape = Shape::circle(Vec2::new(0, 0), 100_000);

    assert_eq!(
      circle_breakouts(100_000, &shape, false),
      circle_breakouts(100_000, &shape, true)
    );
  }

  /// A 600000 by 200000 pad centred on the origin with a 100000 wide
  /// track, hand computed from `pcbnew/router/pns_optimizer.cpp:1003`.
  ///
  /// The orthogonal exits are `size / 2 + width` long, so 400000 along x
  /// and 200000 along y. The diagonals first run `d_offset = 200000`
  /// along the long axis and then turn 45 degrees for
  /// `width + min(size) / 2 = 200000` in each axis.
  #[test]
  fn rect_breakouts_are_four_axis_exits_and_four_diagonals() {
    let shape =
      Shape::rect(Vec2::new(-300_000, -100_000), Vec2::new(600_000, 200_000));
    let breakouts = rect_breakouts(100_000, &shape, true);

    assert_eq!(breakouts.len(), 8);
    assert_eq!(
      ends(&breakouts),
      vec![
        Vec2::new(400_000, 0),
        Vec2::new(-400_000, 0),
        Vec2::new(0, 200_000),
        Vec2::new(0, -200_000),
        Vec2::new(400_000, 200_000),
        Vec2::new(400_000, -200_000),
        Vec2::new(-400_000, 200_000),
        Vec2::new(-400_000, -200_000),
      ]
    );

    // The four orthogonal exits are single segments from the centre and
    // the four diagonals are two, with the elbow out along the long axis.
    for breakout in &breakouts[0..4] {
      assert_eq!(breakout.point(0), Vec2::new(0, 0));
      assert_eq!(breakout.segment_count(), 1);
    }

    for breakout in &breakouts[4..8] {
      assert_eq!(breakout.point(0), Vec2::new(0, 0));
      assert_eq!(breakout.segment_count(), 2);
      assert_eq!(breakout.point(1).x.abs(), 200_000);
      assert_eq!(breakout.point(1).y, 0);
    }
  }

  /// The tall branch at `pcbnew/router/pns_optimizer.cpp:1029` emits the
  /// same four diagonals as the wide one with the middle two swapped, and
  /// the elbow runs along y rather than x. The order is observable
  /// because it breaks a tie between two equal cost, equal length
  /// candidates.
  #[test]
  fn rect_breakouts_swap_two_diagonals_on_a_tall_pad() {
    let shape =
      Shape::rect(Vec2::new(-100_000, -300_000), Vec2::new(200_000, 600_000));
    let breakouts = rect_breakouts(100_000, &shape, true);

    assert_eq!(
      ends(&breakouts),
      vec![
        Vec2::new(200_000, 0),
        Vec2::new(-200_000, 0),
        Vec2::new(0, 400_000),
        Vec2::new(0, -400_000),
        Vec2::new(200_000, 400_000),
        Vec2::new(200_000, -400_000),
        Vec2::new(-200_000, 400_000),
        Vec2::new(-200_000, -400_000),
      ]
    );

    for breakout in &breakouts[4..8] {
      assert_eq!(breakout.point(1).x, 0);
      assert_eq!(breakout.point(1).y.abs(), 200_000);
    }
  }

  #[test]
  fn rect_breakouts_drop_the_diagonals_when_they_are_not_permitted() {
    let shape =
      Shape::rect(Vec2::new(-300_000, -100_000), Vec2::new(600_000, 200_000));
    let breakouts = rect_breakouts(100_000, &shape, false);

    assert_eq!(breakouts.len(), 4);
    assert_eq!(
      ends(&breakouts),
      vec![
        Vec2::new(400_000, 0),
        Vec2::new(-400_000, 0),
        Vec2::new(0, 200_000),
        Vec2::new(0, -200_000),
      ]
    );
  }

  /// A diamond of half diagonal 200000 centred on the origin. Every ray
  /// is `max(bbox side) / 2 + 5 = 200005` long, which reaches the four
  /// vertices and crosses the four edges half way, so all eight land on
  /// the outline.
  #[test]
  fn custom_breakouts_land_on_the_polygon_edge() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let diamond = Shape::simple(LineChain::from_slice(
      &[
        Vec2::new(0, -200_000),
        Vec2::new(200_000, 0),
        Vec2::new(0, 200_000),
        Vec2::new(-200_000, 0),
      ],
      true,
    ));
    let id = pad(&mut world, diamond, Vec2::new(0, 0), 0);
    let item = world.item(id).expect("the pad was just added");
    let breakouts = custom_breakouts(100_000, item, true);

    assert_eq!(
      ends(&breakouts),
      vec![
        Vec2::new(200_000, 0),
        Vec2::new(100_000, 100_000),
        Vec2::new(0, 200_000),
        Vec2::new(-100_000, 100_000),
        Vec2::new(-200_000, 0),
        Vec2::new(-100_000, -100_000),
        Vec2::new(0, -200_000),
        Vec2::new(100_000, -100_000),
      ]
    );

    // Without diagonals the step is 90 degrees, so only the four
    // vertices survive.
    assert_eq!(
      ends(&custom_breakouts(100_000, item, false)),
      vec![
        Vec2::new(200_000, 0),
        Vec2::new(0, 200_000),
        Vec2::new(-200_000, 0),
        Vec2::new(0, -200_000),
      ]
    );
  }

  /// KiCad's ray length is half the **bounding box side**, not half its
  /// diagonal (`pcbnew/router/pns_optimizer.cpp:951`), so on an axis
  /// aligned square the four diagonal rays stop inside the polygon and
  /// contribute nothing, whatever the comment at `:950` says. Pinned
  /// because it silently halves the exits of a square polygonal pad.
  #[test]
  fn custom_breakouts_diagonal_rays_fall_short_on_a_square() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let square = Shape::simple(LineChain::from_slice(
      &[
        Vec2::new(-200_000, -200_000),
        Vec2::new(200_000, -200_000),
        Vec2::new(200_000, 200_000),
        Vec2::new(-200_000, 200_000),
      ],
      true,
    ));
    let id = pad(&mut world, square, Vec2::new(0, 0), 0);
    let item = world.item(id).expect("the pad was just added");

    assert_eq!(
      ends(&custom_breakouts(100_000, item, true)),
      vec![
        Vec2::new(200_000, 0),
        Vec2::new(0, 200_000),
        Vec2::new(-200_000, 0),
        Vec2::new(0, -200_000),
      ]
    );
  }

  #[test]
  fn compute_breakouts_dispatches_on_the_body_and_the_shape() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    // A via: the circle of its layer 0 padstack diameter.
    let body = ItemBody::Via(Via::new(
      Vec2::new(0, 0),
      200_000,
      100_000,
      ViaType::Through,
    ));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item.set_net(Some(NetId(1)));

    let via = world.add_via(root, item);
    let via_item = world.item(via).expect("the via was just added");

    assert_eq!(
      ends(&compute_breakouts(100_000, via_item, true)),
      ends(&circle_breakouts(
        100_000,
        &Shape::circle(Vec2::new(0, 0), 100_000),
        true
      ))
    );

    // A capsule shaped pad goes through `ApproximateSegmentAsRect`, and
    // this one approximates to exactly the rectangle above.
    let capsule =
      Shape::segment(Seg::from_coords(-200_000, 0, 200_000, 0), 200_000);
    let oblong = pad(&mut world, capsule, Vec2::new(0, 0), 0);
    let oblong_item = world.item(oblong).expect("the pad was just added");
    let rect =
      Shape::rect(Vec2::new(-300_000, -100_000), Vec2::new(600_000, 200_000));

    assert_eq!(
      compute_breakouts(100_000, oblong_item, true),
      rect_breakouts(100_000, &rect, true)
    );

    // A compound pad has no case in KiCad's switch and gets no exits.
    let compound = Shape::compound(vec![
      Shape::rect(Vec2::new(-100_000, -100_000), Vec2::new(200_000, 200_000)),
      Shape::circle(Vec2::new(100_000, 0), 100_000),
    ]);
    let complex = pad(&mut world, compound, Vec2::new(2_000_000, 0), 0);
    let complex_item = world.item(complex).expect("the pad was just added");

    assert!(compute_breakouts(100_000, complex_item, true).is_empty());
  }

  #[test]
  fn find_pad_or_via_answers_the_first_link_of_the_right_kind() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let at = Vec2::new(1_000_000, 0);
    let id = pad(&mut world, Shape::circle(at, 100_000), at, 0);

    assert_eq!(
      find_pad_or_via(&world, root, 0, Some(NetId(1)), at),
      Some(id)
    );

    // The wrong layer, the wrong net and a bare point all answer nothing.
    assert_eq!(find_pad_or_via(&world, root, 1, Some(NetId(1)), at), None);
    assert_eq!(find_pad_or_via(&world, root, 0, Some(NetId(2)), at), None);
    assert_eq!(
      find_pad_or_via(&world, root, 0, Some(NetId(1)), Vec2::new(0, 0)),
      None
    );

    // A joint that holds only track ends is not a pad.
    let seg = Seg::from_coords(0, 0, 500_000, 0);
    let body = ItemBody::Segment(Segment::new(seg, 100_000));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(Some(NetId(1)));
    world
      .add_segment(root, item, false)
      .expect("the segment is neither degenerate nor redundant");

    assert_eq!(
      find_pad_or_via(&world, root, 0, Some(NetId(1)), Vec2::new(0, 0)),
      None
    );
  }

  #[test]
  fn the_forbidden_angle_mask_is_kicads() {
    assert!(FORBIDDEN_ANGLES.contains(AngleType::ACUTE));
    assert!(FORBIDDEN_ANGLES.contains(AngleType::RIGHT));
    assert!(FORBIDDEN_ANGLES.contains(AngleType::HALF_FULL));
    assert!(FORBIDDEN_ANGLES.contains(AngleType::UNDEFINED));
    assert!(!FORBIDDEN_ANGLES.intersects(AngleType::OBTUSE));
    assert!(!FORBIDDEN_ANGLES.intersects(AngleType::STRAIGHT));
  }

  /// A via is refused one step before the breakouts are even built
  /// (`pcbnew/router/pns_optimizer.cpp:1128`), so a line leaving a via
  /// keeps the exit posture the placer gave it.
  #[test]
  fn smart_pads_single_refuses_a_via() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let at = Vec2::new(0, 0);
    let body = ItemBody::Via(Via::new(at, 200_000, 100_000, ViaType::Through));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item.set_net(Some(NetId(1)));

    let via = world.add_via(root, item);
    let rules = FixedClearance::uniform(50_000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let optimizer = Optimizer::new(root);
    let mut line = Line::new();

    line.set_width(100_000);
    line.set_layer(0);
    line.set_net(Some(NetId(1)));
    line.set_shape(LineChain::from_slice(
      &[at, Vec2::new(0, 600_000), Vec2::new(1_000_000, 600_000)],
      false,
    ));

    let before = line.shape().clone();

    assert_eq!(
      optimizer.smart_pads_single(&world, &context, &mut line, via, false, 3),
      None
    );
    assert_eq!(*line.shape(), before);
  }

  /// An offset pad is refused for the same reason a via is: every
  /// breakout starts at the centre, which is not where the copper is
  /// (`pcbnew/router/pns_optimizer.cpp:1123`).
  #[test]
  fn smart_pads_single_refuses_an_offset_pad() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let at = Vec2::new(0, 0);
    let mut solid = Solid::new(Shape::circle(at, 100_000), at);

    solid.set_offset(Vec2::new(10_000, 0));

    let mut item = world.make_item(ItemBody::Solid(solid));

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(Some(NetId(1)));

    let id = world.add_solid(root, item, None);
    let rules = FixedClearance::uniform(50_000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let optimizer = Optimizer::new(root);
    let mut line = Line::new();

    line.set_width(100_000);
    line.set_layer(0);
    line.set_net(Some(NetId(1)));
    line.set_shape(LineChain::from_slice(
      &[at, Vec2::new(0, 600_000), Vec2::new(1_000_000, 600_000)],
      false,
    ));

    assert_eq!(
      optimizer.smart_pads_single(&world, &context, &mut line, id, false, 3),
      None
    );
  }

  /// A line of two points has nothing between its ends to rewrite, so
  /// both pad passes decline it (`pcbnew/router/pns_optimizer.cpp:1235`,
  /// `:1275`).
  #[test]
  fn the_pad_passes_decline_a_two_point_line() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(50_000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let optimizer = Optimizer::new(root);
    let mut line = Line::new();

    line.set_width(100_000);
    line.set_shape(LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(1_000_000, 0)],
      false,
    ));

    assert!(!optimizer.run_smart_pads(&world, &context, &mut line));
    assert!(!optimizer.fanout_cleanup(&world, &context, &mut line));
  }

  /// The `return true` at `pcbnew/router/pns_optimizer.cpp:1254` is
  /// unconditional, so a line whose ends carry no pad at all still
  /// reports "changed".
  #[test]
  fn run_smart_pads_reports_changed_even_with_nothing_to_do() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(50_000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let optimizer = Optimizer::new(root);
    let mut line = Line::new();

    line.set_width(100_000);
    line.set_shape(LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(0, 600_000),
        Vec2::new(1_000_000, 600_000),
      ],
      false,
    ));

    let before = line.shape().clone();

    assert!(optimizer.run_smart_pads(&world, &context, &mut line));
    assert_eq!(*line.shape(), before);
  }

  // -----------------------------------------------------------------
  // The differential pair path
  // -----------------------------------------------------------------

  /// The width of one lane of the pair fixtures.
  const DP_WIDTH: i32 = 100_000;

  /// The edge to edge gap of the pair fixtures, which with
  /// [`DP_WIDTH`] puts the two centre lines 500000 apart.
  const DP_GAP: i32 = 400_000;

  /// The clearance the pair fixtures are routed under. It has to sit
  /// below the 253553 the two lanes leave each other along their 45
  /// degree legs, which is what a gap measured across a diagonal comes
  /// to.
  const DP_CLEARANCE: i32 = 100_000;

  /// A pair with the same detour on both lanes, at the gap along its
  /// straight runs.
  ///
  /// The P lane runs east at `y = 0`, spikes out to `(2e6, -1e6)` and
  /// comes back, runs east again and leaves on a diagonal. The N lane is
  /// the same chain moved 500000 along y, so the two straight runs are
  /// coupled and the four diagonals are not: a diagonal moved along y by
  /// the pitch is only `pitch / sqrt(2)` from its twin.
  fn spiked_pair() -> DiffPair {
    let p = LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(1_000_000, 0),
        Vec2::new(2_000_000, -1_000_000),
        Vec2::new(3_000_000, 0),
        Vec2::new(4_000_000, 0),
        Vec2::new(5_000_000, -1_000_000),
      ],
      false,
    );
    let n = LineChain::from_points(
      p.points()
        .iter()
        .map(|point| Vec2::new(point.x, point.y + 500_000))
        .collect(),
      false,
    );
    let mut pair = DiffPair::from_chains(p, n, DP_GAP);

    pair.set_width(DP_WIDTH);
    pair.set_gap(DP_GAP);
    pair.set_nets(Some(NetId(1)), Some(NetId(2)));
    pair.set_layer(0);

    pair
  }

  /// A round solid on a third net, on the pair fixtures' layer.
  fn dp_obstacle(world: &mut World, at: Vec2, radius: i32) -> ItemId {
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, radius), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(Some(NetId(3)));

    let root = world.root();

    world.add_solid(root, item, None)
  }

  /// Both lanes lose their spike in one pass, and the pair comes out
  /// more coupled than it went in.
  ///
  /// The reference lane's span is `[s1, s3]`, whose outer segments are a
  /// 45 degree leg and an east leg and therefore obtuse. Its bypass runs
  /// straight from `(1e6, 0)` to `(4e6, 0)`, which is
  /// `coupledBypass`'s cue to straighten the N lane over the same run;
  /// the winner there is the candidate that reaches furthest, because the
  /// score is coupled length and nothing else.
  #[test]
  fn a_spike_on_both_lanes_is_merged_and_the_pair_stays_coupled() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let resolver = FixedClearance::uniform(DP_CLEARANCE);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&resolver, &settings);
    let optimizer = Optimizer::new(root);
    let mut pair = spiked_pair();

    // The two straight runs, 1e6 and 1e6 of them.
    assert_eq!(pair.coupled_length(), 2_000_000);

    assert!(optimizer.optimize_diff_pair(&world, &context, &mut pair));

    assert_eq!(
      pair.chain_p().points(),
      &[
        Vec2::new(0, 0),
        Vec2::new(4_000_000, 0),
        Vec2::new(5_000_000, -1_000_000),
      ]
    );
    assert_eq!(
      pair.chain_n().points(),
      &[
        Vec2::new(0, 500_000),
        Vec2::new(4_000_000, 500_000),
        Vec2::new(5_000_000, -500_000),
      ]
    );

    // The whole 4e6 of straight run is coupled now, where two thirds of
    // it used to be spent on the detour.
    assert_eq!(pair.coupled_length(), 4_000_000);
  }

  /// The same pair, with a solid on each of the two straight runs the
  /// bypasses want.
  ///
  /// Both sit at `x = 2e6`, where the detour has taken the two lanes
  /// well out of the way: the nearer one is 353553 from the N lane and
  /// 707107 from the P lane, the further one 707107 and 1060660, so
  /// neither lane touches either before the merge. They are on the two
  /// straight lines the lanes would be rewritten onto, so every
  /// candidate fails [`Optimizer::verify_dp_bypass`] and the pair comes
  /// out untouched.
  ///
  /// Both solids are needed. With only the one on the P run, the N lane
  /// still straightens on its own: its bypass is clear, and
  /// [`Optimizer::coupled_bypass`] then wins with the candidate that
  /// leaves the P lane exactly as it is.
  #[test]
  fn a_bypass_that_would_hit_an_obstacle_is_rejected() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let resolver = FixedClearance::uniform(DP_CLEARANCE);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&resolver, &settings);
    let optimizer = Optimizer::new(root);
    let mut pair = spiked_pair();
    let before = pair.clone();

    dp_obstacle(&mut world, Vec2::new(2_000_000, 0), 150_000);
    dp_obstacle(&mut world, Vec2::new(2_000_000, 500_000), 150_000);

    // Neither lane touches the two solids where they are now.
    assert!(!optimizer.check_colliding(&world, &context, &pair.p_line()));
    assert!(!optimizer.check_colliding(&world, &context, &pair.n_line()));

    optimizer.optimize_diff_pair(&world, &context, &mut pair);

    assert_eq!(pair.chain_p().points(), before.chain_p().points());
    assert_eq!(pair.chain_n().points(), before.chain_n().points());
  }

  /// A merge that would cost more coupling than the budget allows is
  /// refused even though nothing is in the way.
  ///
  /// The P lane's only obtuse span is the one that holds its whole
  /// coupled run, so bypassing it lifts that run away from the N lane
  /// altogether: the coupled length would fall from 2e6 to zero, where
  /// the budget of a tenth allows it to fall by 2e5. `coupledBypass` has
  /// nothing to offer either, because the straightened P lane is 1.4e6
  /// from the N lane and no vertex of it matches the gap constraint.
  #[test]
  fn a_bypass_that_would_decouple_the_pair_is_rejected() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let resolver = FixedClearance::uniform(DP_CLEARANCE);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&resolver, &settings);
    let optimizer = Optimizer::new(root);
    let p = LineChain::from_slice(
      &[
        Vec2::new(-1_000_000, 1_000_000),
        Vec2::new(0, 1_000_000),
        Vec2::new(1_000_000, 0),
        Vec2::new(3_000_000, 0),
        Vec2::new(4_000_000, 1_000_000),
        Vec2::new(5_000_000, 1_000_000),
        Vec2::new(6_000_000, 2_000_000),
      ],
      false,
    );
    let n = LineChain::from_slice(
      &[
        Vec2::new(-1_000_000, -500_000),
        Vec2::new(6_000_000, -500_000),
      ],
      false,
    );
    let mut pair = DiffPair::from_chains(p, n, DP_GAP);

    pair.set_width(DP_WIDTH);
    pair.set_gap(DP_GAP);
    pair.set_nets(Some(NetId(1)), Some(NetId(2)));
    pair.set_layer(0);

    let before = pair.clone();

    assert_eq!(pair.coupled_length(), 2_000_000);

    optimizer.optimize_diff_pair(&world, &context, &mut pair);

    assert_eq!(pair.chain_p().points(), before.chain_p().points());
    assert_eq!(pair.chain_n().points(), before.chain_n().points());
    assert_eq!(pair.coupled_length(), 2_000_000);
  }

  /// The bound erratum E14 asks for actually stops the loop.
  ///
  /// A right angle staircase has no obtuse span at any length, so no
  /// pass ever merges anything and the span counters fall by one per
  /// pass: eighteen passes to walk a twenty segment lane down to a span
  /// of one, and a nineteenth to leave through the exit test. Held to
  /// three passes the loop stops after three, which is the behaviour
  /// KiCad's `while( 1 )` cannot express.
  #[test]
  fn the_merge_pass_limit_bounds_the_pair_loop() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let resolver = FixedClearance::uniform(DP_CLEARANCE);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&resolver, &settings);
    let optimizer = Optimizer::new(root);
    let mut points = vec![Vec2::new(0, 0)];

    for step in 0..10 {
      let x = (step + 1) * 100_000;
      let y = -step * 100_000;

      points.push(Vec2::new(x, y));
      points.push(Vec2::new(x, y - 100_000));
    }

    let p = LineChain::from_points(points, false);
    let n = LineChain::from_points(
      p.points()
        .iter()
        .map(|point| Vec2::new(point.x, point.y + 500_000))
        .collect(),
      false,
    );

    assert_eq!(p.segment_count(), 20);

    let mut pair = DiffPair::from_chains(p, n, DP_GAP);

    pair.set_width(DP_WIDTH);
    pair.set_gap(DP_GAP);
    pair.set_nets(Some(NetId(1)), Some(NetId(2)));
    pair.set_layer(0);

    let mut bounded = pair.clone();

    assert_eq!(
      optimizer.merge_dp_passes(&world, &context, &mut pair, MERGE_PASS_LIMIT),
      19
    );
    assert_eq!(
      optimizer.merge_dp_passes(&world, &context, &mut bounded, 3),
      3
    );

    // Neither run changed the geometry, so the two answers differ only in
    // how long they took to say so.
    assert_eq!(pair.chain_p().points(), bounded.chain_p().points());
  }

  /// [`find_coupled_vertices`] answers with segment indices, and it
  /// measures edge to edge.
  #[test]
  fn find_coupled_vertices_picks_the_parallel_segments_at_the_gap() {
    let pair = spiked_pair();
    // The reference lane's bypass: straight east along `y = 0`.
    let bypass = LineChain::from_slice(
      &[Vec2::new(1_000_000, 0), Vec2::new(4_000_000, 0)],
      false,
    );
    let found = find_coupled_vertices(
      bypass.point(0),
      bypass.segment(0),
      pair.chain_n(),
      &pair,
    );

    // The N lane's two east segments, and neither of its four diagonals.
    assert_eq!(found, vec![0, 3]);

    // A pair whose constraint holds the centre to centre pitch instead
    // matches nothing, which is the trap the two gap meanings set.
    let mut pitch_pair = spiked_pair();

    pitch_pair.set_gap(500_000);

    assert!(
      find_coupled_vertices(
        bypass.point(0),
        bypass.segment(0),
        pitch_pair.chain_n(),
        &pitch_pair,
      )
      .is_empty()
    );
  }
}
