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
//! is unchanged. Length is not part of any live decision in KiCad's tree.
//!
//! # What is not here
//!
//! - **The collision cache.** `m_cache`, `m_cacheTags`, `cacheAdd`,
//!   `ClearCache`, `MaxCachedItems` and the `CACHE_VISITOR` are all dead
//!   in KiCad: nothing populates the cache, the visitor is constructed
//!   and discarded, and `ClearCache( true )` would erase from a map while
//!   iterating it (note 04 section 4.3). Every candidate is checked
//!   against the live node here, exactly as KiCad actually does.
//! - **The pad passes.** `runSmartPads`, `smartPadsSingle`, the four
//!   breakout generators, `fanoutCleanup` and `findPadOrVia` are the next
//!   work item; note 04 section 8.5 puts them in phase 5. The two flags
//!   that select them, [`EffortFlags::SMART_PADS`] and
//!   [`EffortFlags::FANOUT_CLEANUP`], are accepted and skipped so that a
//!   caller passing them still gets the other passes.
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
//! - **The diff pair path**, `mergeDpSegments`, `mergeDpStep`,
//!   `coupledBypass` and `verifyDpBypass`, together with `Tighten` and
//!   its helpers, which nothing calls.
//! - **`dragFixCorners` / `dragFixCorner` and `OBTUSE_ONLY_CONSTRAINT`**,
//!   selected by `REQUIRE_OBTUSE_ANGLES`. That flag is a dragger feature
//!   and arrives with the dragger, so it has no [`EffortFlags`] constant
//!   here.
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
use crate::geometry::box2::Box2;
use crate::geometry::direction45::{AngleType, Direction45};
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::{Vec2, Vec2L};
use crate::item::Kind;
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
/// `REQUIRE_OBTUSE_ANGLES` (`0x400`) has no constant here: it selects
/// `dragFixCorners` and `OBTUSE_ONLY_CONSTRAINT`, which are a dragger
/// feature and are not ported yet. A host passing that bit through
/// [`EffortFlags::from_bits`] keeps it and it is ignored.
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
  /// `SMART_PADS = 0x02`, `pcbnew/router/pns_optimizer.h:100`.
  ///
  /// **Not implemented yet.** The pass is the next work item; the flag is
  /// accepted and skipped, so a caller that asks for it still gets every
  /// other pass. KiCad's `runSmartPads` always reports "changed"
  /// (note 04 section 4.7), so an [`Optimizer::optimize`] that is given
  /// only this flag answers `false` here where KiCad would answer `true`.
  pub const SMART_PADS: EffortFlags = EffortFlags(0x002);

  /// Run [`Optimizer::merge_obtuse`]. Port of `MERGE_OBTUSE = 0x04`,
  /// `pcbnew/router/pns_optimizer.h:101`.
  pub const MERGE_OBTUSE: EffortFlags = EffortFlags(0x004);

  /// Redraw a very short pad to pad connection as a plain L. Port of
  /// `FANOUT_CLEANUP = 0x08`, `pcbnew/router/pns_optimizer.h:102`.
  ///
  /// **Not implemented yet**, like [`EffortFlags::SMART_PADS`].
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
    }
  }
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

    // :709. `REQUIRE_OBTUSE_ANGLES` and `dragFixCorners` arrive with the
    // dragger.

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

    // :724 and :728. `runSmartPads` and `fanoutCleanup` are the next work
    // item; their flags are accepted and skipped.

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
}

#[cfg(test)]
mod tests {
  use super::*;

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
}
