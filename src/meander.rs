// SPDX-License-Identifier: GPL-3.0-or-later

//! Meanders: the shape generator, the fitting loop and the length
//! arithmetic length tuning is built out of.
//!
//! A port of `pcbnew/router/pns_meander.h`, `pcbnew/router/pns_meander.cpp`
//! and the arithmetic half of `pcbnew/router/pns_meander_placer_base.cpp`.
//! Nothing here needs a [`crate::node::World`], a
//! [`crate::rules::RuleResolver`] or a placer: a meander is a pure
//! function of a base segment, a [`MeanderSettings`] and the three numbers
//! a [`MeanderContext`] carries, which is what makes the whole module unit
//! testable and is the one structural change from KiCad, where
//! `MEANDER_SHAPE` holds a `MEANDER_PLACER_BASE*` back pointer
//! (`pns_meander.h:417`).
//!
//! The reference note is `doc/reference/kicad/08-meanders.md`, sections 1
//! to 4, 8, 9, 11 and errata E1, E3, E4 to E8, E10, E15 to E18 and E22.
//!
//! # What is implemented
//!
//! - [`MeanderSettings`] with KiCad's defaults, [`LengthTarget`] in place
//!   of `MINOPTMAX<long long>` and its one kilometre sentinel,
//!   [`CornerStyle`], [`MeanderSide`], [`MeanderType`] and
//!   [`TuningStatus`].
//! - [`MeanderShape`]: the turtle, the chamfered corner, the five bodies
//!   of `genMeanderShape`, the amplitude search of `Fit`, and the
//!   measurements `tuneLineLength` compares meanders by.
//! - [`MeanderedLine`]: the ordered list of shapes covering one base
//!   segment, the fitting loop `MeanderSegment`, and the self
//!   intersection test both placers' `CheckFit` ends on.
//! - The shared placer arithmetic: [`tune_line_length`],
//!   [`find_amplitude_for_length`], [`find_amplitude_binary_search`],
//!   [`amplitude_step`], [`spacing_step`] and [`clearance`].
//!
//! # What is not
//!
//! - The three placers, which are a later slice. Everything a placer adds
//!   on top of this module is state: the world branch, the assembled
//!   path, the cut and reassembly of the original line, and the status
//!   readout.
//! - Arcs. `MEANDER_STYLE_ROUND` builds the only `SHAPE_ARC` in the
//!   router (`pns_meander.cpp:496`) and `MakeArc`, `AddArc`,
//!   `AddArcAndPt` and `AddPtAndArc` carry a pre existing arc through a
//!   tuned stretch. [`CornerStyle`] has one variant and
//!   [`MeanderSettings::new`] refuses [`MeanderStyle::Round`], so no
//!   caller can reach a code path that would need one. Note 08 section
//!   9.3 works out that this costs no behaviour the crate could otherwise
//!   exhibit, because a [`crate::geometry::line_chain::LineChain`] cannot
//!   hold an arc in the first place.
//! - `MT_ARC` as a meander type: `MakeArc` sets `MT_CORNER`, so nothing
//!   in KiCad's tree ever produces it (erratum E4).
//! - The time domain and net chain halves of `MEANDER_SETTINGS`
//!   (`m_signalExtraLength`, `m_targetLengthDelay`, `m_isTimeDomain`,
//!   `m_netClass` and six more) and of `MEANDER_PLACER_BASE`
//!   (`initChainExtras`, `chainNarrowingOffset`), which have no
//!   counterpart in this crate or in LibrePCB. Note 08 section 4.4.
//! - The dead members erratum E1 collects: `m_lenPadToDie`,
//!   `m_lengthTolerance`, `MEANDER_SHAPE::m_baseIndex` with its two
//!   accessors, `MEANDER_PLACER_BASE::m_currentEnd`, and the unused
//!   `SHAPE_LINE_CHAIN lc` of `MeanderSegment`.
//!
//! # Errata reproduced on purpose
//!
//! Each of these is transcribed as KiCad wrote it, with a comment at the
//! line naming the erratum, because the rest of the milestone measures
//! against this behaviour and a later fix should be a visible test change.
//!
//! - **E3**: [`MeanderShape::min_amplitude`]'s chamfer correction is
//!   `tan( 1 - tan( 22.5 deg ) )`, a typo for `1 - tan( 22.5 deg )`.
//! - **E6**: the skip advance of [`MeanderedLine::meander_segment`] is
//!   `spacing + step`, because the corner radius term it subtracts is
//!   always zero.
//! - **E7**: [`MeanderedLine::check_self_intersections`] tests chain `0`
//!   only, so a dual meander's N lane is never checked.
//! - **E8**: [`find_amplitude_for_length`]'s fast path measures the
//!   minimum amplitude and returns a different one.
//! - **E10**: the two ad hoc self intersection clearances live in the
//!   placers; this module takes the clearance as an argument, as KiCad
//!   does.
//! - **E15**: the two `(int)` casts of a fractional scalar,
//!   [`MeanderShape::corner_radius`]'s floor and
//!   [`MeanderShape::min_amplitude`]'s correction, truncate toward zero
//!   rather than rounding. The turtle's own position does **not**: see
//!   the deviation below.
//! - **E16**: [`find_amplitude_binary_search`] returns an amplitude it
//!   never measured when the interval has collapsed.
//! - **E17**: `Fit`'s check path assigns `m_baseSeg` twice; only the
//!   surviving assignment is transcribed.
//! - **E18**: the duplicate corner shapes a placer's `doMove` adds are a
//!   placer concern; [`MeanderedLine::meander_segment`] adds its own two
//!   for a non dual line exactly where KiCad does.
//!
//! **E22 is the one erratum that is fixed rather than reproduced.** `Fit`'s
//! amplitude loop decrements by `m_step` and never terminates when the
//! step is zero (`pns_meander.cpp:789`). [`MeanderSettings::new`] refuses
//! a step that is not positive, so the loop cannot be entered with one.
//!
//! # The one deviation: the turtle rounds where KiCad truncates
//!
//! KiCad's turtle carries its position in a `VECTOR2D` and truncates each
//! coordinate toward zero when it reaches the chain
//! (`pns_meander.cpp:486`, `:513`, `:515`, `:518`, and the implicit
//! `VECTOR2D` to `VECTOR2I` conversions at `:625`, `:650`, `:652`,
//! `:674`). This port carries it as a [`crate::geometry::vec2::Vec2`] and
//! rounds at every step through [`crate::geometry::vec2::Vec2::resize`],
//! which is what `DESIGN.md` section 2 asks for. The two agree exactly on
//! an axis aligned or 45 degree base segment, which is every trace KiCad's
//! or LibrePCB's 45 degree router can produce, and differ by at most one
//! nanometre per turtle step otherwise. Note 08 section 8.3 recommends
//! this and `doc/log/2026-09-10.md` records it.
//!
//! # The three constants that need an `f64`
//!
//! [`TAN_22_5`], [`ONE_MINUS_TAN_22_5`] and
//! [`TAN_ONE_MINUS_TAN_22_5`]. They are the only floating point in the
//! module, they are transcendental, and they are written as literals
//! because `f64::tan` is not a `const fn`; `the_chamfer_constants_are_the_
//! ones_kicad_computes` pins each one bit for bit against the expression
//! KiCad writes.

use std::fmt;

use crate::geometry::line_chain::LineChain;
use crate::geometry::math::kiround;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::Vec2;
use crate::rules::{ConstraintType, ItemRef, RuleResolver};

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

/// `tan( 22.5 degrees )`, which is `sqrt( 2 ) - 1`.
///
/// Port of the `tan( DEG2RAD( 22.5 ) )` of `makeMiterShape`'s dual
/// correction, `pcbnew/router/pns_meander.cpp:507`. Written as the literal
/// KiCad's own expression evaluates to rather than as `SQRT_2 - 1.0`,
/// which is one unit in the last place away.
pub const TAN_22_5: f64 = 0.414_213_562_373_095_03;

/// `1 - tan( 22.5 degrees )`, which is `2 - sqrt( 2 )`.
///
/// Port of the corner radius floor's chamfer factor,
/// `pcbnew/router/pns_meander.cpp:439`. One unit in the last place away
/// from `2.0 - SQRT_2`, so it is written out.
pub const ONE_MINUS_TAN_22_5: f64 = 0.585_786_437_626_905;

/// `tan( 1 - tan( 22.5 degrees ) )`, the tangent of a length in radians.
///
/// Port of `MinAmplitude`'s chamfer correction,
/// `pcbnew/router/pns_meander.cpp:421`, which is erratum E3: the
/// expression written correctly eighteen lines down is
/// [`ONE_MINUS_TAN_22_5`]. The two differ by 13.3 percent, so a host that
/// lowers [`MeanderSettings::min_amplitude`] below the correction gets a
/// taller minimum meander than KiCad's authors intended. Reproduced,
/// because the rest of milestone 11 measures against it.
pub const TAN_ONE_MINUS_TAN_22_5: f64 = 0.663_470_255_400_132_7;

/// The half width the setters pad an optimum length with, 0.1 millimetre.
///
/// Port of `MEANDER_SETTINGS::DEFAULT_LENGTH_TOLERANCE`,
/// `pcbnew/router/pns_meander.cpp:31`, which is `pcbIUScale.mmToIU( 0.1 )`.
pub const DEFAULT_LENGTH_TOLERANCE: i64 = 100_000;

/// The "no length was asked for" sentinel, one kilometre.
///
/// Port of `MEANDER_SETTINGS::LENGTH_UNCONSTRAINED`,
/// `pcbnew/router/pns_meander.cpp:32`, which is `1000000 * IU_PER_MM`.
///
/// It is a sentinel and not a clamp: KiCad happily tries to reach it and
/// settles on [`TuningStatus::TooShort`] when the baseline runs out. This
/// crate says "unconstrained" with `Option::None` instead
/// (`DESIGN.md` section 11), and keeps the constant so that a host
/// translating KiCad's settings has something to compare against.
pub const LENGTH_UNCONSTRAINED: i64 = 1_000_000_000_000;

/// How close to the requested length the amplitude search settles for.
///
/// Port of `LENGTH_TARGET_TOLERANCE`,
/// `pcbnew/router/pns_meander_placer_base.cpp:32`, a namespace scope
/// `const int` with no declaration in any header.
pub const LENGTH_TARGET_TOLERANCE: i64 = 20;

/// The shortest move the turtle makes; anything smaller is dropped.
///
/// Port of the guard at `pcbnew/router/pns_meander.cpp:543`, commented
/// there as "very small segments cause problems". It also swallows the
/// negative lengths that `amplitude - 2 * corner_radius + |offset|` can
/// produce.
const MIN_TURTLE_STEP: i32 = 5;

// ---------------------------------------------------------------------
// The small value types
// ---------------------------------------------------------------------

/// A length the tuner aims for, with the window it accepts.
///
/// Replaces `MINOPTMAX<long long int>` plus [`LENGTH_UNCONSTRAINED`]
/// (`pcbnew/router/pns_meander.h:125`). KiCad's setters always fill all
/// three values, so there is nothing here for the `HasMin` / `HasOpt` /
/// `HasMax` predicates of `libs/core/include/core/minoptmax.h:29` to do;
/// the three `Option`s that [`crate::rules::Constraint`] carries are for
/// a design rule that arrives half filled, which is a different thing.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct LengthTarget {
  /// The shortest accepted length. Below it the status is
  /// [`TuningStatus::TooShort`].
  pub min: i64,
  /// The length the meanders aim for.
  pub opt: i64,
  /// The longest accepted length. Above it the status is
  /// [`TuningStatus::TooLong`].
  pub max: i64,
}

impl LengthTarget {
  /// An optimum padded by [`DEFAULT_LENGTH_TOLERANCE`] on both sides.
  ///
  /// Port of `MEANDER_SETTINGS::SetTargetLength( long long int )`,
  /// `pcbnew/router/pns_meander.cpp:67`. The [`LENGTH_UNCONSTRAINED`]
  /// case is KiCad's own branch at `:71`: minimum zero, maximum the
  /// sentinel itself, so that nothing is ever "too long".
  #[must_use]
  pub const fn around(opt: i64) -> Self {
    if opt == LENGTH_UNCONSTRAINED {
      // :73, :74
      return Self {
        min: 0,
        opt,
        max: opt,
      };
    }

    // :78, :79
    Self {
      min: opt - DEFAULT_LENGTH_TOLERANCE,
      opt,
      max: opt + DEFAULT_LENGTH_TOLERANCE,
    }
  }

  /// The three values a design rule supplied.
  ///
  /// Port of `SetTargetLength( const MINOPTMAX<int>& )`,
  /// `pcbnew/router/pns_meander.cpp:84`, which starts from the scalar
  /// form and then overwrites whichever of the two bounds the constraint
  /// carries.
  #[must_use]
  pub const fn explicit(min: i64, opt: i64, max: i64) -> Self {
    Self { min, opt, max }
  }

  /// The target `MEANDER_SETTINGS`'s constructor installs.
  ///
  /// Port of `SetTargetLength( LENGTH_UNCONSTRAINED )`,
  /// `pcbnew/router/pns_meander.cpp:49`. Prefer `Option::None` on
  /// [`MeanderSettings::target_length`]; this exists so a host can round
  /// trip KiCad's own value.
  #[must_use]
  pub const fn unconstrained() -> Self {
    Self::around(LENGTH_UNCONSTRAINED)
  }
}

/// The shape of a meander's corners.
///
/// Port of `MEANDER_STYLE` (`pcbnew/router/pns_meander.h:53`), restricted
/// to what this crate can draw. KiCad's `MEANDER_STYLE_ROUND` (`:54`) is a
/// 90 degree `SHAPE_ARC` built in `makeMiterShape`
/// (`pns_meander.cpp:496`), and arcs are on hold (`PLAN.md`), so this enum
/// has one variant. Adding `Round` is the change that lands with them, and
/// `#[non_exhaustive]` keeps that from breaking a host that matches on it.
///
/// The round style is refused rather than silently drawn as a chamfer
/// because the two differ in length by `radius * (pi / 2 - sqrt( 2 ))` per
/// corner, so substituting one for the other would make a "tuned" trace
/// miss its target by roughly `0.62 * radius` per meander. Say so with
/// [`MeanderStyle`] and let [`MeanderSettings::new`] answer
/// [`MeanderSettingsError::RoundCornersUnsupported`].
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
#[non_exhaustive]
pub enum CornerStyle {
  /// A 45 degree chord across the corner. `MEANDER_STYLE_CHAMFER`
  /// (`pcbnew/router/pns_meander.h:55`).
  #[default]
  Chamfer,
}

/// The corner style a host asks for, which is KiCad's enum in full.
///
/// Port of `MEANDER_STYLE`, `pcbnew/router/pns_meander.h:53`, with
/// KiCad's discriminants so a host bridging to a stored "rounded" flag
/// (`pcbnew/generators/pcb_tuning_pattern.cpp:1802`) has somewhere to put
/// it. It exists separately from [`CornerStyle`] because a value has to be
/// expressible before it can be refused: [`MeanderSettings::new`] is the
/// boundary that turns the supported half of this into a [`CornerStyle`]
/// and answers [`MeanderSettingsError::RoundCornersUnsupported`] for the
/// other half.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
#[repr(i32)]
pub enum MeanderStyle {
  /// A 90 degree arc. `MEANDER_STYLE_ROUND` (`:54`), which is KiCad's own
  /// default (`pns_meander.cpp:56`) and is not implemented here.
  Round = 1,
  /// A 45 degree chord. `MEANDER_STYLE_CHAMFER` (`:55`).
  #[default]
  Chamfer = 2,
}

/// Which side of the base line a tuned stretch starts its meanders on.
///
/// Port of `MEANDER_SIDE`, `pcbnew/router/pns_meander.h:59`, with KiCad's
/// discriminants, because [`MeanderSide::flipped`] is a negation and
/// [`MeanderSide::Default`] is the value that a negation leaves alone.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
#[repr(i32)]
pub enum MeanderSide {
  /// `MEANDER_SIDE_LEFT`, the value `MEANDER_SETTINGS`'s constructor
  /// installs (`pns_meander.cpp:59`).
  #[default]
  Left = -1,
  /// `MEANDER_SIDE_DEFAULT`, which selects the "follow the cursor" branch
  /// in both placers (`pns_meander_placer.cpp:266`). Neither KiCad's
  /// settings constructor nor its host ever leaves the value here, so it
  /// is only reachable from a host that asks for it.
  Default = 0,
  /// `MEANDER_SIDE_RIGHT`.
  Right = 1,
}

impl MeanderSide {
  /// The side with its sign flipped.
  ///
  /// Port of `settings.m_initialSide = (PNS::MEANDER_SIDE) -settings.m_initialSide`
  /// inside `MeanderSegment`'s `flipInitialSide` lambda,
  /// `pcbnew/router/pns_meander.cpp:286`. `-0 == 0`, so
  /// [`MeanderSide::Default`] never flips, which is KiCad's behaviour and
  /// not an oversight of this port.
  #[must_use]
  pub const fn flipped(self) -> Self {
    match self {
      Self::Left => Self::Right,
      Self::Default => Self::Default,
      Self::Right => Self::Left,
    }
  }
}

/// Which of the shapes a meander is.
///
/// Port of `MEANDER_TYPE`, `pcbnew/router/pns_meander.h:40`. KiCad has
/// nine members; `MT_ARC` (`:48`) is not here, because `MakeArc` assigns
/// `MT_CORNER` instead (`pns_meander.cpp:918`) and nothing anywhere in
/// KiCad's tree ever produces it. Erratum E4.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum MeanderType {
  /// `_|^|_`, a meander that starts and ends on the base line.
  /// `MT_SINGLE`.
  Single,
  /// `_|^|`, the opening shape of a turning run. `MT_START`.
  Start,
  /// `|^|_`, the closing shape of a turning run. `MT_FINISH`.
  Finish,
  /// `|^|` or `|_|`, one shape in the middle of a turning run. `MT_TURN`.
  Turn,
  /// Ask whether a [`MeanderType::Start`] and then a
  /// [`MeanderType::Turn`] would fit, without producing either.
  /// `MT_CHECK_START`.
  CheckStart,
  /// Ask whether a [`MeanderType::Turn`] and then a
  /// [`MeanderType::Finish`] would fit. `MT_CHECK_FINISH`.
  CheckFinish,
  /// A corner of the line being tuned, carried through the meander list
  /// so the reassembly keeps it. `MT_CORNER`.
  Corner,
  /// A straight bypass of the baseline this meander used to consume.
  /// `MT_EMPTY`.
  Empty,
}

/// How a tuned line stands against its target.
///
/// Port of `MEANDER_PLACER_BASE::TUNING_STATUS`,
/// `pcbnew/router/pns_meander_placer_base.h:49`.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum TuningStatus {
  /// `TOO_SHORT`. The meanders ran out of baseline before the target.
  TooShort,
  /// `TOO_LONG`. The line is longer than the window allows and no
  /// meander can shorten it.
  TooLong,
  /// `TUNED`. Inside the window.
  Tuned,
}

impl TuningStatus {
  /// The status a length gives against a target.
  ///
  /// Port of the three inline branches every placer ends its move with,
  /// `pcbnew/router/pns_meander_placer.cpp:332` to `:337` and
  /// `pcbnew/router/pns_dp_meander_placer.cpp:283` to `:288`. Note the
  /// order: too long wins over too short, which only matters for an
  /// inverted window a host could build with
  /// [`LengthTarget::explicit`].
  #[must_use]
  pub const fn for_length(length: i64, target: &LengthTarget) -> Self {
    if length > target.max {
      Self::TooLong
    } else if length < target.min {
      Self::TooShort
    } else {
      Self::Tuned
    }
  }
}

// ---------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------

/// What [`MeanderSettings::new`] can refuse.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum MeanderSettingsError {
  /// The step is zero or negative.
  ///
  /// `Fit`'s amplitude loop decrements by it
  /// (`pcbnew/router/pns_meander.cpp:789`) and `MeanderSegment` compares
  /// its remaining baseline against it twice (`:317`, `:385`), so a zero
  /// step turns two loop exits into none and the fit never terminates.
  /// Nothing in KiCad's tree can set it to zero, which is why it is a
  /// latent hazard there and an error here. Erratum E22.
  NonPositiveStep,
  /// The round corner style was asked for.
  ///
  /// It needs a `SHAPE_ARC` (`pcbnew/router/pns_meander.cpp:496`) and
  /// arcs are on hold. See [`CornerStyle`] for why silently drawing
  /// chamfers instead is not an option.
  RoundCornersUnsupported,
}

impl fmt::Display for MeanderSettingsError {
  fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::NonPositiveStep => {
        formatter.write_str("the meander amplitude step must be positive")
      }
      Self::RoundCornersUnsupported => {
        formatter.write_str("rounded meander corners need arcs")
      }
    }
  }
}

impl std::error::Error for MeanderSettingsError {}

/// The settings a host hands to [`MeanderSettings::new`].
///
/// Plain data with public fields, so a host can build one field by field;
/// [`MeanderSettings`] itself keeps its fields private because two of them
/// carry an invariant. [`Default`] is KiCad's constructor
/// (`pcbnew/router/pns_meander.cpp:42` to `:63`) with one deliberate
/// change: the corner style is [`MeanderStyle::Chamfer`] where KiCad's is
/// [`MeanderStyle::Round`] (`:56`), so that the default request is one
/// this crate can honour.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct MeanderSettingsRequest {
  /// `m_minAmplitude` (`pcbnew/router/pns_meander.h:101`), default
  /// 200000.
  pub min_amplitude: i32,
  /// `m_maxAmplitude` (`:104`), default 1000000.
  pub max_amplitude: i32,
  /// `m_spacing` (`:107`), default 600000.
  pub spacing: i32,
  /// `m_step` (`:110`), default 50000. Must be positive, erratum E22.
  pub step: i32,
  /// `m_cornerStyle` (`:145`). See [`MeanderStyle`].
  pub corner_style: MeanderStyle,
  /// `m_cornerRadiusPercentage` (`:148`), default 80. It is a percentage
  /// of the **half** period, so 100 means a corner radius of exactly half
  /// the spacing, which is the most that fits.
  pub corner_radius_percentage: i32,
  /// `m_singleSided` (`:151`), default false. When set, the fitting loop
  /// never opens a turning run and lays down a plain sequence of
  /// [`MeanderType::Single`] shapes.
  pub single_sided: bool,
  /// `m_initialSide` (`:154`), default [`MeanderSide::Left`].
  pub initial_side: MeanderSide,
  /// `m_keepEndpoints` (`:160`), default false, forced true by KiCad's
  /// host. Only the placers' reassembly reads it
  /// (`pcbnew/router/pns_meander_placer.cpp:342`).
  pub keep_endpoints: bool,
  /// `m_targetLength` (`:125`). `None` is KiCad's
  /// [`LENGTH_UNCONSTRAINED`] said without a sentinel.
  pub target_length: Option<LengthTarget>,
  /// `m_targetSkew` (`:137`), default optimum zero with a
  /// [`DEFAULT_LENGTH_TOLERANCE`] window either side
  /// (`pns_meander.cpp:61`).
  pub target_skew: Option<LengthTarget>,
}

impl Default for MeanderSettingsRequest {
  fn default() -> Self {
    Self {
      min_amplitude: 200_000,
      max_amplitude: 1_000_000,
      spacing: 600_000,
      step: 50_000,
      corner_style: MeanderStyle::Chamfer,
      corner_radius_percentage: 80,
      single_sided: false,
      initial_side: MeanderSide::Left,
      keep_endpoints: false,
      target_length: None,
      target_skew: Some(LengthTarget::around(0)),
    }
  }
}

/// The dimensions the meander generator works to.
///
/// Port of `MEANDER_SETTINGS`, `pcbnew/router/pns_meander.h:69`, minus the
/// nine time domain and net chain fields this crate has no concept of and
/// the two that are dead in KiCad itself (erratum E1). See note 08 section
/// 1.4 for the accounting.
///
/// The fields are private because two of them carry invariants that the
/// rest of the module relies on: the step is positive and the corner style
/// is one this crate can draw. Build one with [`MeanderSettings::new`],
/// which is the boundary those are checked at.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct MeanderSettings {
  /// `m_minAmplitude` (`pcbnew/router/pns_meander.h:101`).
  min_amplitude: i32,
  /// `m_maxAmplitude` (`:104`).
  max_amplitude: i32,
  /// `m_spacing` (`:107`).
  spacing: i32,
  /// `m_step` (`:110`), always positive.
  step: i32,
  /// `m_cornerStyle` (`:145`).
  corner_style: CornerStyle,
  /// `m_cornerRadiusPercentage` (`:148`).
  corner_radius_percentage: i32,
  /// `m_singleSided` (`:151`).
  single_sided: bool,
  /// `m_initialSide` (`:154`).
  initial_side: MeanderSide,
  /// `m_keepEndpoints` (`:160`).
  keep_endpoints: bool,
  /// `m_targetLength` (`:125`).
  target_length: Option<LengthTarget>,
  /// `m_targetSkew` (`:137`).
  target_skew: Option<LengthTarget>,
}

impl Default for MeanderSettings {
  /// KiCad's defaults, `pcbnew/router/pns_meander.cpp:42` to `:63`, with
  /// the corner style changed to the one that is implemented.
  fn default() -> Self {
    Self {
      min_amplitude: 200_000,
      max_amplitude: 1_000_000,
      spacing: 600_000,
      step: 50_000,
      corner_style: CornerStyle::Chamfer,
      corner_radius_percentage: 80,
      single_sided: false,
      initial_side: MeanderSide::Left,
      keep_endpoints: false,
      target_length: None,
      target_skew: Some(LengthTarget::around(0)),
    }
  }
}

impl MeanderSettings {
  /// Check a request and turn it into settings.
  ///
  /// The two things it refuses are a step that is not positive, which
  /// would make `Fit`'s amplitude loop spin forever (erratum E22,
  /// `pcbnew/router/pns_meander.cpp:789`), and the round corner style,
  /// which needs an arc (`:496`). Everything else KiCad accepts, this
  /// accepts: a negative amplitude, a corner radius percentage over 100
  /// and an inverted length window all have defined behaviour further
  /// down and none of them can make a loop run away.
  ///
  /// # Errors
  ///
  /// [`MeanderSettingsError::NonPositiveStep`] and
  /// [`MeanderSettingsError::RoundCornersUnsupported`].
  pub fn new(
    request: MeanderSettingsRequest,
  ) -> Result<Self, MeanderSettingsError> {
    if request.step <= 0 {
      return Err(MeanderSettingsError::NonPositiveStep);
    }

    let corner_style = match request.corner_style {
      MeanderStyle::Chamfer => CornerStyle::Chamfer,
      MeanderStyle::Round => {
        return Err(MeanderSettingsError::RoundCornersUnsupported);
      }
    };

    Ok(Self {
      min_amplitude: request.min_amplitude,
      max_amplitude: request.max_amplitude,
      spacing: request.spacing,
      step: request.step,
      corner_style,
      corner_radius_percentage: request.corner_radius_percentage,
      single_sided: request.single_sided,
      initial_side: request.initial_side,
      keep_endpoints: request.keep_endpoints,
      target_length: request.target_length,
      target_skew: request.target_skew,
    })
  }

  /// The smallest excursion a meander may have. `m_minAmplitude`.
  #[must_use]
  pub const fn min_amplitude(&self) -> i32 {
    self.min_amplitude
  }

  /// The largest excursion `Fit` starts its scan at. `m_maxAmplitude`.
  #[must_use]
  pub const fn max_amplitude(&self) -> i32 {
    self.max_amplitude
  }

  /// The meander period the user asked for. `m_spacing`.
  #[must_use]
  pub const fn spacing(&self) -> i32 {
    self.spacing
  }

  /// The amplitude scan's decrement, always positive. `m_step`.
  #[must_use]
  pub const fn step(&self) -> i32 {
    self.step
  }

  /// The corner shape. `m_cornerStyle`.
  #[must_use]
  pub const fn corner_style(&self) -> CornerStyle {
    self.corner_style
  }

  /// The corner radius as a percentage of the half period.
  /// `m_cornerRadiusPercentage`.
  #[must_use]
  pub const fn corner_radius_percentage(&self) -> i32 {
    self.corner_radius_percentage
  }

  /// Whether a run may only meander to one side. `m_singleSided`.
  #[must_use]
  pub const fn single_sided(&self) -> bool {
    self.single_sided
  }

  /// The side the next tuned stretch starts on. `m_initialSide`.
  #[must_use]
  pub const fn initial_side(&self) -> MeanderSide {
    self.initial_side
  }

  /// Whether the placers' reassembly keeps the original endpoints.
  /// `m_keepEndpoints`.
  #[must_use]
  pub const fn keep_endpoints(&self) -> bool {
    self.keep_endpoints
  }

  /// The length being tuned to, or `None` for unconstrained.
  /// `m_targetLength`.
  #[must_use]
  pub const fn target_length(&self) -> Option<LengthTarget> {
    self.target_length
  }

  /// The skew being tuned to, or `None` for unconstrained.
  /// `m_targetSkew`.
  #[must_use]
  pub const fn target_skew(&self) -> Option<LengthTarget> {
    self.target_skew
  }

  /// Set the largest excursion `Fit` starts its scan at.
  ///
  /// The mutator [`amplitude_step`] needs; it cannot break either
  /// invariant, so it is infallible.
  pub const fn set_max_amplitude(&mut self, amplitude: i32) {
    self.max_amplitude = amplitude;
  }

  /// Set the meander period.
  ///
  /// The mutator [`spacing_step`] needs.
  pub const fn set_spacing(&mut self, spacing: i32) {
    self.spacing = spacing;
  }

  /// Set the length being tuned to.
  pub const fn set_target_length(&mut self, target: Option<LengthTarget>) {
    self.target_length = target;
  }

  /// Set the skew being tuned to.
  pub const fn set_target_skew(&mut self, target: Option<LengthTarget>) {
    self.target_skew = target;
  }

  /// Flip the side the next tuned stretch starts on.
  ///
  /// Port of `MeanderSegment`'s `flipInitialSide` lambda,
  /// `pcbnew/router/pns_meander.cpp:282` to `:288`, which reads the
  /// placer's settings, negates the side and writes the whole struct back
  /// through `UpdateSettings` (`pns_meander_placer_base.cpp:131`).
  ///
  /// In this port the generator does not reach back into anything: the
  /// flip comes out of [`MeanderedLine::meander_segment`] as a flag and
  /// the caller applies it here. Note 08 section 11.4 asks for that, and
  /// it is sound because nothing inside the fitting loop reads
  /// [`MeanderSettings::initial_side`]: the flip only changes where the
  /// **next** move starts.
  pub const fn flip_initial_side(&mut self) {
    self.initial_side = self.initial_side.flipped();
  }
}

// ---------------------------------------------------------------------
// The context the generator borrows
// ---------------------------------------------------------------------

/// The always accepting fit check [`MeanderContext::accepting_every_fit`]
/// installs, as a `static` so a borrow of it lives long enough.
static ACCEPT_EVERY_FIT: fn(&MeanderShape, &MeanderedLine) -> bool =
  accept_every_fit;

/// The body behind [`ACCEPT_EVERY_FIT`].
fn accept_every_fit(_shape: &MeanderShape, _placed: &MeanderedLine) -> bool {
  true
}

/// What the shape generator is handed instead of a back pointer.
///
/// KiCad's `MEANDER_SHAPE::m_placer` (`pcbnew/router/pns_meander.h:417`)
/// exists to reach exactly three things: `MeanderSettings()`
/// (`pns_meander.cpp:242`), `Clearance()` (`:460`) and `CheckFit()`
/// (`:815`). This carries all three, so the generator is a pure function
/// of its arguments and needs no world, no node and no router.
///
/// The clearance is resolved **once per move** here, where KiCad resolves
/// it once per `spacing()` call, which is several times per candidate
/// amplitude per meander and drags a rebuild of the placer's whole trace
/// behind it (erratum E11). The results are identical as long as the
/// host's rules do not change during a move, which they cannot: nothing
/// in a move calls back into the host.
pub struct MeanderContext<'a> {
  /// `MEANDER_PLACER_BASE::m_settings`
  /// (`pcbnew/router/pns_meander_placer_base.h:183`).
  settings: &'a MeanderSettings,
  /// `MEANDER_PLACER_BASE::m_currentWidth` (`:180`), the width of the
  /// track being tuned.
  width: i32,
  /// What `MEANDER_PLACER_BASE::Clearance()`
  /// (`pns_meander_placer_base.cpp:114`) answers, resolved once. Build it
  /// with [`clearance`].
  clearance: i32,
  /// `MEANDER_PLACER_BASE::CheckFit`
  /// (`pcbnew/router/pns_meander_placer_base.h:120`, overridden at
  /// `pns_meander_placer.cpp:400` and `pns_dp_meander_placer.cpp:608`).
  /// It is handed the candidate and everything already placed, which is
  /// what both overrides read: the candidate goes against the node and
  /// against the meanders already in the line.
  check_fit: &'a dyn Fn(&MeanderShape, &MeanderedLine) -> bool,
}

impl<'a> MeanderContext<'a> {
  /// A context with a real fit check.
  #[must_use]
  pub fn new(
    settings: &'a MeanderSettings,
    width: i32,
    clearance: i32,
    check_fit: &'a dyn Fn(&MeanderShape, &MeanderedLine) -> bool,
  ) -> Self {
    Self {
      settings,
      width,
      clearance,
      check_fit,
    }
  }

  /// A context whose fit check accepts every candidate.
  ///
  /// The base implementation of `MEANDER_PLACER_BASE::CheckFit` answers
  /// false (`pcbnew/router/pns_meander_placer_base.h:120`), so a placer
  /// that forgets to override it fits nothing at all. This is the
  /// opposite, and it is what makes the shape generator and the fitting
  /// loop testable without a world.
  #[must_use]
  pub fn accepting_every_fit(
    settings: &'a MeanderSettings,
    width: i32,
    clearance: i32,
  ) -> Self {
    Self {
      settings,
      width,
      clearance,
      check_fit: &ACCEPT_EVERY_FIT,
    }
  }

  /// The settings every dimension is derived from.
  #[must_use]
  pub const fn settings(&self) -> &MeanderSettings {
    self.settings
  }

  /// The width of the track being tuned.
  #[must_use]
  pub const fn width(&self) -> i32 {
    self.width
  }

  /// The copper to copper clearance in force.
  #[must_use]
  pub const fn clearance(&self) -> i32 {
    self.clearance
  }

  /// Ask the fit check whether a candidate may be placed.
  #[must_use]
  pub fn check_fit(
    &self,
    shape: &MeanderShape,
    placed: &MeanderedLine,
  ) -> bool {
    (self.check_fit)(shape, placed)
  }
}

// ---------------------------------------------------------------------
// The turtle
// ---------------------------------------------------------------------

/// One of the two rotations the turtle ever performs.
///
/// `MEANDER_SHAPE::turn` takes an `EDA_ANGLE`
/// (`pcbnew/router/pns_meander.cpp:551`) and the only arguments anywhere
/// in the tree are `ANGLE_90` and `-ANGLE_90` (`:561`, `:569`, `:643`,
/// `:661`). `RotatePoint` normalises first and then takes an exact branch
/// for each (`libs/kimath/src/trigo.cpp:305`, `:313`), so no trigonometry
/// ever runs and the turtle's direction stays an integer vector. That is
/// the whole reason this module needs no floating point vector type.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum QuarterTurn {
  /// `ANGLE_90`, which `RotatePoint` maps to `(y, -x)`
  /// (`libs/kimath/src/trigo.cpp:305`).
  Plus90,
  /// `-ANGLE_90`, normalised to 270 and mapped to `(-y, x)` (`:313`).
  Minus90,
}

/// The Logo style turtle `genMeanderShape` draws each body with.
///
/// Port of the five private members and the four methods at
/// `pcbnew/router/pns_meander.cpp:530` to `:580`. KiCad keeps the position,
/// the direction and a raw `SHAPE_LINE_CHAIN*` on the shape itself and
/// nulls the pointer afterwards to avoid dangling (`:688`); here the
/// turtle owns its chain and hands it back, so there is nothing to dangle.
///
/// It also carries the three shape fields `makeMiterShape` reads, because
/// `genMeanderShape` has already written the last of them by the time the
/// turtle runs (`:610`).
struct Turtle {
  /// `m_currentTarget` (`pns_meander.h:462`), owned rather than borrowed.
  chain: LineChain,
  /// `m_currentDir` (`:456`). Always the initial base segment vector with
  /// its components permuted and signed.
  direction: Vec2,
  /// `m_currentPos` (`:459`).
  position: Vec2,
  /// `MEANDER_SHAPE::m_dual` (`:420`), read by `makeMiterShape` at
  /// `pns_meander.cpp:506`.
  dual: bool,
  /// `MEANDER_SHAPE::m_baselineOffset` (`:429`), read at `:507`.
  baseline_offset: i32,
  /// `MEANDER_SHAPE::m_meanCornerRadius` (`:432`), read at `:506`.
  mean_corner_radius: i32,
}

impl Turtle {
  /// Put the turtle down at a point, facing a direction.
  ///
  /// Port of `start`, `pcbnew/router/pns_meander.cpp:530`. KiCad clears
  /// the target chain first, which is why `MT_FINISH` and `MT_TURN` can
  /// call it a second time to throw away the point `genMeanderShape`
  /// already put down (`:642`, `:660`).
  fn start(
    position: Vec2,
    direction: Vec2,
    dual: bool,
    baseline_offset: i32,
    mean_corner_radius: i32,
  ) -> Self {
    let mut chain = LineChain::new();
    chain.append(position);

    Self {
      chain,
      direction,
      position,
      dual,
      baseline_offset,
      mean_corner_radius,
    }
  }

  /// Walk forward, dropping anything shorter than five nanometres.
  ///
  /// Port of `forward`, `pcbnew/router/pns_meander.cpp:540`. The guard at
  /// `:543` also swallows the negative lengths
  /// `amplitude - 2 * corner_radius + |offset|` can produce.
  fn forward(&mut self, length: i32) {
    // :543
    if length < MIN_TURTLE_STEP {
      return;
    }

    // :546. KiCad accumulates in a `VECTOR2D` and truncates on append;
    // this rounds through `Vec2::resize` at every step. See the module
    // documentation and `doc/log/2026-09-10.md`.
    self.position += self.direction.resize(length);
    self.chain.append(self.position);
  }

  /// Turn a quarter circle.
  ///
  /// Port of `turn`, `pcbnew/router/pns_meander.cpp:551`. Written as a
  /// component permutation because that is exactly what `RotatePoint`
  /// does for these two angles; `Vec2::perpendicular` is already `(-y, x)`,
  /// which is the `-ANGLE_90` case.
  fn turn(&mut self, quarter: QuarterTurn) {
    self.direction = match quarter {
      // (y, -x), which is the negation of (-y, x).
      QuarterTurn::Plus90 => -self.direction.perpendicular(),
      QuarterTurn::Minus90 => self.direction.perpendicular(),
    };
  }

  /// Round the corner and turn.
  ///
  /// Port of `miter`, `pcbnew/router/pns_meander.cpp:557`. Note the order:
  /// the position is moved to the corner's far end **before** the corner
  /// chain is appended (`:571`), and the seam duplicate the append would
  /// otherwise leave is dropped by
  /// [`crate::geometry::line_chain::LineChain::append_chain`].
  fn miter(&mut self, radius: i32, side: bool) {
    // :559
    if radius <= 0 {
      self.turn(if side {
        QuarterTurn::Plus90
      } else {
        QuarterTurn::Minus90
      });
      return;
    }

    // :565
    let direction = self.direction.resize(radius);
    let corner = self.make_miter_shape(self.position, direction, side);

    // :568. The corner chain always holds at least the point it started
    // from, so the fallback is unreachable.
    if let Some(last) = corner.last_point() {
      self.position = last;
    }

    self.turn(if side {
      QuarterTurn::Plus90
    } else {
      QuarterTurn::Minus90
    });

    self.chain.append_chain(&corner);
  }

  /// Out, across and back: the excursion every meander is built from.
  ///
  /// Port of `uShape`, `pcbnew/router/pns_meander.cpp:575`.
  fn u_shape(&mut self, sides: i32, corner: i32, top: i32) {
    self.forward(sides);
    self.miter(corner, true);
    self.forward(top);
    self.miter(corner, true);
    self.forward(sides);
  }

  /// The chain that replaces one right angle.
  ///
  /// Port of `makeMiterShape`, `pcbnew/router/pns_meander.cpp:470`, the
  /// only function in the router that branches on the corner style and the
  /// only place a `SHAPE_ARC` is constructed. There is no arc branch here:
  /// [`CornerStyle`] cannot name the round style, so `:491` to `:499` has
  /// no reachable input.
  ///
  /// For a single track the correction is zero, the two inner appends
  /// collapse onto the endpoints, and the result is exactly two points: a
  /// 45 degree chord of length `radius * sqrt( 2 )` across a corner whose
  /// apex would have been at `p + dir`. For a dual meander the correction
  /// pulls the chamfer's start back along the direction of travel and
  /// pushes its middle out sideways by `2 * |offset| * tan( 22.5 deg )`,
  /// so the two lanes stay a constant gap apart around the corner, and the
  /// `radius > mean_corner_radius` guard at `:506` applies it to the outer
  /// lane only.
  fn make_miter_shape(
    &self,
    p: Vec2,
    direction: Vec2,
    side: bool,
  ) -> LineChain {
    let mut chain = LineChain::new();

    // :475. KiCad tests `EuclideanNorm() == 0.0f`; the norm of a non zero
    // integer vector is never zero, so this is the same test.
    if direction == Vec2::new(0, 0) {
      chain.append(p);
      return chain;
    }

    // :481, :482
    let direction_u = direction;
    let direction_v = direction.perpendicular();
    let sign = if side { -1 } else { 1 };

    // :484
    let end_point = p + direction_u + direction_v * sign;

    // :486
    chain.append(p);

    // :503 to :507. `radius` is only ever compared, so it stays an
    // integer.
    let radius = direction.euclidean_norm();
    let mut correction = 0.0_f64;

    if self.dual && i64::from(radius) > i64::from(self.mean_corner_radius) {
      correction =
        -2.0 * (i64::from(self.baseline_offset).abs() as f64) * TAN_22_5;
    }

    // :509, :510. KiCad passes the fractional correction straight into
    // `VECTOR2D::Resize`; `Vec2::resize` takes an integer, so it is
    // rounded here, which moves the chamfer by under a nanometre. Note 08
    // section 8.2 offers exactly this choice.
    let correction = kiround(correction);
    let direction_cu = direction_u.resize(correction);
    let direction_cv = direction_v.resize(correction);

    // :513
    chain.append(p - direction_cu);
    // :515
    chain.append(p + direction_u + (direction_v + direction_cv) * sign);
    // :518
    chain.append(end_point);

    chain
  }
}

// ---------------------------------------------------------------------
// MeanderShape
// ---------------------------------------------------------------------

/// One meander: a type, an amplitude, and the chains it generates.
///
/// Port of `MEANDER_SHAPE`, `pcbnew/router/pns_meander.h:172`. It is a
/// value in KiCad too, copied all over `tuneLineLength`
/// (`pns_meander_placer_base.cpp:213`), even though `MEANDERED_LINE` stores
/// raw pointers to it; here it is a value everywhere and
/// [`MeanderedLine`] owns a `Vec` of them.
///
/// Four of KiCad's eighteen members are the turtle's scratch state and
/// live on the private turtle instead; `m_baseIndex` and its two
/// accessors are
/// written in two places and read nowhere, so they are not carried
/// (erratum E1).
#[derive(Clone, Debug)]
pub struct MeanderShape {
  /// `m_type` (`pcbnew/router/pns_meander.h:414`).
  meander_type: MeanderType,
  /// `m_dual` (`:420`). Two chains rather than one.
  dual: bool,
  /// `m_width` (`:423`).
  width: i32,
  /// `m_amplitude` (`:426`).
  amplitude: i32,
  /// `m_baselineOffset` (`:429`). Half the pair pitch, signed; zero for a
  /// single track.
  baseline_offset: i32,
  /// `m_meanCornerRadius` (`:432`), an **output** of
  /// [`MeanderShape::gen_meander_shape`] (`pns_meander.cpp:610`) read back
  /// by [`MeanderShape::fit`] (`:812`) and by
  /// the turtle's `make_miter_shape` (`:506`).
  mean_corner_radius: i32,
  /// `m_targetBaseLen` (`:435`). When non zero it widens the flat top so
  /// a resized meander keeps its baseline footprint.
  target_base_len: i32,
  /// `m_p0` (`:438`), where the meander starts on the base segment.
  p0: Vec2,
  /// `m_baseSeg` (`:441`), the whole segment being meandered. KiCad's
  /// `BaseSegment()` accessor does **not** return this one; see
  /// [`MeanderShape::base_segment`].
  base_seg: Seg,
  /// `m_clippedBaseSeg` (`:444`), the part of the base segment this
  /// meander consumes.
  clipped_base_seg: Seg,
  /// `m_side` (`:447`). True means mirrored across the base line.
  side: bool,
  /// `m_shapes[2]` (`:450`). The second is used only when dual.
  shapes: [LineChain; 2],
}

impl MeanderShape {
  /// An empty meander of a given width.
  ///
  /// Port of the constructor at `pcbnew/router/pns_meander.h:181`, minus
  /// the placer back pointer. Everything else is zeroed there too.
  #[must_use]
  pub fn new(width: i32, dual: bool) -> Self {
    Self {
      meander_type: MeanderType::Single,
      dual,
      width,
      amplitude: 0,
      baseline_offset: 0,
      mean_corner_radius: 0,
      target_base_len: 0,
      p0: Vec2::new(0, 0),
      base_seg: Seg::new(Vec2::new(0, 0), Vec2::new(0, 0)),
      clipped_base_seg: Seg::new(Vec2::new(0, 0), Vec2::new(0, 0)),
      side: false,
      shapes: [LineChain::new(), LineChain::new()],
    }
  }

  // -------------------------------------------------------------------
  // Accessors and mutators
  // -------------------------------------------------------------------

  /// Which shape this is. `Type()` (`pcbnew/router/pns_meander.h:208`).
  #[must_use]
  pub const fn meander_type(&self) -> MeanderType {
    self.meander_type
  }

  /// Change the shape without regenerating it.
  ///
  /// Port of `SetType` (`pcbnew/router/pns_meander.h:200`). Every caller
  /// in `tuneLineLength` follows it with
  /// [`MeanderShape::recalculate`] (`pns_meander_placer_base.cpp:222`).
  pub const fn set_type(&mut self, meander_type: MeanderType) {
    self.meander_type = meander_type;
  }

  /// The excursion height. `Amplitude()` (`:232`).
  #[must_use]
  pub const fn amplitude(&self) -> i32 {
    self.amplitude
  }

  /// The track width. `Width()` (`:355`).
  #[must_use]
  pub const fn width(&self) -> i32 {
    self.width
  }

  /// Whether this meander carries two chains. `IsDual()` (`:271`).
  #[must_use]
  pub const fn is_dual(&self) -> bool {
    self.dual
  }

  /// Which side of the base line the meander is on. `Side()` (`:279`).
  #[must_use]
  pub const fn side(&self) -> bool {
    self.side
  }

  /// Half the pair pitch, signed. `m_baselineOffset` (`:429`).
  #[must_use]
  pub const fn baseline_offset(&self) -> i32 {
    self.baseline_offset
  }

  /// Set half the pair pitch. `SetBaselineOffset` (`:366`).
  pub const fn set_baseline_offset(&mut self, offset: i32) {
    self.baseline_offset = offset;
  }

  /// The baseline footprint a resize has to preserve.
  /// `SetTargetBaselineLength` (`:377`).
  pub const fn set_target_baseline_length(&mut self, length: i32) {
    self.target_base_len = length;
  }

  /// The corner radius [`MeanderShape::gen_meander_shape`] actually used.
  ///
  /// `m_meanCornerRadius` (`pcbnew/router/pns_meander.h:432`), written at
  /// `pns_meander.cpp:610` after the three clamps and read back by
  /// [`MeanderShape::fit`]'s rejection test at `:812`.
  #[must_use]
  pub const fn mean_corner_radius(&self) -> i32 {
    self.mean_corner_radius
  }

  /// The part of the base segment this meander consumes.
  ///
  /// Port of `BaseSegment()` (`pcbnew/router/pns_meander.h:322`), which
  /// despite its name returns `m_clippedBaseSeg` and not `m_baseSeg`. Both
  /// callers depend on that: `MEANDERED_LINE::AddMeander` marches along it
  /// (`pns_meander.cpp:930`) and `CheckSelfIntersections` compares two of
  /// them for parallelism (`:707`).
  #[must_use]
  pub const fn base_segment(&self) -> Seg {
    self.clipped_base_seg
  }

  /// Where the next meander starts. `End()`
  /// (`pcbnew/router/pns_meander.h:287`).
  #[must_use]
  pub const fn end(&self) -> Vec2 {
    self.clipped_base_seg.b
  }

  /// One of the generated chains.
  ///
  /// Port of `CLine( int )` (`pcbnew/router/pns_meander.h:295`). Index `1`
  /// is meaningful only for a dual meander.
  ///
  /// # Panics
  ///
  /// When `index` is not 0 or 1, where KiCad reads out of bounds.
  #[must_use]
  pub fn cline(&self, index: usize) -> &LineChain {
    &self.shapes[index]
  }

  // -------------------------------------------------------------------
  // The three derived dimensions
  // -------------------------------------------------------------------

  /// The meander period, floored by what physically fits.
  ///
  /// Port of `spacing()`, `pcbnew/router/pns_meander.cpp:456`. In KiCad
  /// this reaches through the placer into the rule resolver on every call
  /// and is on the hot path (erratum E11); here the clearance was resolved
  /// once into the context.
  #[must_use]
  pub fn spacing(&self, context: &MeanderContext<'_>) -> i32 {
    let floor = if self.dual {
      // :464
      i64::from(self.width)
        + i64::from(context.clearance())
        + 2 * i64::from(self.baseline_offset).abs()
    } else {
      // :460
      i64::from(self.width) + i64::from(context.clearance())
    };

    saturate_i32(floor.max(i64::from(context.settings().spacing())))
  }

  /// The corner radius before [`MeanderShape::gen_meander_shape`]'s own
  /// clamps.
  ///
  /// Port of `cornerRadius()`, `pcbnew/router/pns_meander.cpp:429`. The
  /// `/ 200` at `:450` is deliberate: the percentage is of the **half**
  /// period, so 100 percent is a radius of exactly half the spacing, the
  /// most that fits.
  ///
  /// The `maxCr < minCr` branch at `:445` is a `wxCHECK2_MSG`, which
  /// returns the maximum and logs; there is nothing to log here.
  #[must_use]
  pub fn corner_radius(&self, context: &MeanderContext<'_>) -> i32 {
    // :431
    if self.amplitude == 0 {
      return 0;
    }

    let offset = i64::from(self.baseline_offset).abs();
    let spacing = i64::from(self.spacing(context));

    // :439. The integer division happens first, then the multiplication
    // by the constant, then the truncation toward zero of the assignment
    // to `int`. Erratum E15.
    let minimum =
      offset + (f64::from(self.width / 2) * ONE_MINUS_TAN_22_5) as i64;

    // :441 to :443
    let maximum = ((i64::from(self.amplitude) + offset) / 2).min(spacing / 2);

    // :445
    if maximum < minimum {
      return saturate_i32(maximum);
    }

    // :450
    let optimum =
      spacing * i64::from(context.settings().corner_radius_percentage()) / 200;

    // :452
    saturate_i32(optimum.clamp(minimum, maximum))
  }

  /// The shortest excursion this meander may be resized to.
  ///
  /// Port of `MinAmplitude()`, `pcbnew/router/pns_meander.cpp:411`. The
  /// chamfer correction at `:421` is `tan( 1 - tan( 22.5 deg ) )` where
  /// the same expression written correctly eighteen lines down is
  /// `1 - tan( 22.5 deg )`, so the correction is 13.3 percent larger than
  /// intended. Erratum E3, reproduced; it is masked whenever
  /// [`MeanderSettings::min_amplitude`] dominates, which is the default
  /// case, and the host copies the same typo into its selection outline
  /// (`pcbnew/generators/pcb_tuning_pattern.cpp:1626`).
  #[must_use]
  pub fn min_amplitude(&self, context: &MeanderContext<'_>) -> i32 {
    // :413
    let minimum = i64::from(context.settings().min_amplitude());

    // :421, truncated toward zero by the assignment to `int`, erratum
    // E15. E3 is the constant itself.
    let correction = (f64::from(self.width) * TAN_ONE_MINUS_TAN_22_5) as i64;

    // :422
    saturate_i32(
      minimum.max(i64::from(self.baseline_offset).abs() + correction),
    )
  }

  /// The largest excursion [`MeanderShape::fit`] starts its scan at.
  ///
  /// Port of `maxAmpl` in `Fit`, `pcbnew/router/pns_meander.cpp:781`,
  /// which is the setting floored by [`MeanderShape::min_amplitude`] so
  /// that the scan always runs at least once.
  #[must_use]
  pub fn max_amplitude(&self, context: &MeanderContext<'_>) -> i32 {
    context
      .settings()
      .max_amplitude()
      .max(self.min_amplitude(context))
  }

  // -------------------------------------------------------------------
  // The generator
  // -------------------------------------------------------------------

  /// Draw one meander with the turtle.
  ///
  /// Port of `genMeanderShape`, `pcbnew/router/pns_meander.cpp:585`. The
  /// direction is the **whole base segment vector**, not a unit vector;
  /// every use goes through `Resize`, so its magnitude does not matter.
  ///
  /// It writes [`MeanderShape::mean_corner_radius`] as a side effect
  /// (`:610`), which is what `Fit`'s rejection test and the dual chamfer
  /// correction read afterwards.
  ///
  /// [`MeanderType::CheckStart`], [`MeanderType::CheckFinish`] and
  /// [`MeanderType::Corner`] fall through KiCad's `default:` at `:677` and
  /// produce a chain holding only the start point.
  pub fn gen_meander_shape(
    &mut self,
    context: &MeanderContext<'_>,
    p: Vec2,
    direction: Vec2,
    side: bool,
    meander_type: MeanderType,
    baseline_offset: i32,
  ) -> LineChain {
    // :589 to :593
    let mut corner_radius = i64::from(self.corner_radius(context));
    let spacing = i64::from(self.spacing(context));
    let amplitude = i64::from(self.amplitude);
    let target_base_len = i64::from(self.target_base_len);

    // :595
    let offset = if side {
      -i64::from(baseline_offset)
    } else {
      i64::from(baseline_offset)
    };

    // :598, :599
    let direction_u_b = direction.resize(saturate_i32(offset));
    let direction_v_b = direction_u_b.perpendicular();

    // :601 to :608
    if 2 * corner_radius > amplitude + offset.abs() {
      corner_radius = (amplitude + offset.abs()) / 2;
    }

    if 2 * corner_radius > spacing {
      corner_radius = spacing / 2;
    }

    if corner_radius - offset < 0 {
      corner_radius = offset;
    }

    // :610
    self.mean_corner_radius = saturate_i32(corner_radius);

    // :612 to :616
    let s_corner = saturate_i32(corner_radius - offset);
    let u_corner = saturate_i32(corner_radius + offset);
    let start_side = saturate_i32(amplitude - 2 * corner_radius + offset.abs());
    let turn_side = saturate_i32(amplitude - corner_radius);
    let mut top = spacing - 2 * corner_radius;

    let corner_radius = saturate_i32(corner_radius);

    // :620
    let mut turtle = Turtle::start(
      p + direction_v_b,
      direction,
      self.dual,
      self.baseline_offset,
      self.mean_corner_radius,
    );

    match meander_type {
      MeanderType::Empty => {
        // :625. The clipped base segment, translated.
        turtle.chain.append(p + direction_v_b + direction);
      }

      MeanderType::Start => {
        // :629
        if target_base_len != 0 {
          top = top.max(
            target_base_len - i64::from(s_corner) - 2 * i64::from(u_corner)
              + offset,
          );
        }

        // :631 to :635
        turtle.miter(s_corner, false);
        turtle.u_shape(start_side, u_corner, saturate_i32(top));
        turtle.forward(s_corner.min(u_corner));
        turtle.forward(saturate_i32(offset.abs()));
      }

      MeanderType::Finish => {
        // :640
        if target_base_len != 0 {
          top = top.max(target_base_len - i64::from(corner_radius) - spacing);
        }

        // :642, :643
        turtle = Turtle::start(
          p - direction_u_b,
          direction,
          self.dual,
          self.baseline_offset,
          self.mean_corner_radius,
        );
        turtle.turn(QuarterTurn::Minus90);

        // :645 to :649
        turtle.forward(s_corner.min(u_corner));
        turtle.forward(saturate_i32(offset.abs()));
        turtle.u_shape(start_side, u_corner, saturate_i32(top));
        turtle.miter(s_corner, false);

        // :650 to :653
        let reach = if target_base_len >= spacing + i64::from(corner_radius) {
          target_base_len
        } else {
          2 * spacing - i64::from(corner_radius)
        };

        turtle
          .chain
          .append(p + direction_v_b + direction.resize(saturate_i32(reach)));
      }

      MeanderType::Turn => {
        // :658
        if target_base_len != 0 {
          top = top.max(target_base_len - 2 * i64::from(u_corner) + 2 * offset);
        }

        // :660, :661
        turtle = Turtle::start(
          p - direction_u_b,
          direction,
          self.dual,
          self.baseline_offset,
          self.mean_corner_radius,
        );
        turtle.turn(QuarterTurn::Minus90);

        // :662 to :665
        turtle.forward(saturate_i32(offset.abs()));
        turtle.u_shape(turn_side, u_corner, saturate_i32(top));
        turtle.forward(saturate_i32(offset.abs()));
      }

      MeanderType::Single => {
        // :669
        if target_base_len != 0 {
          top = top.max(
            (target_base_len
              - 2 * i64::from(s_corner)
              - 2 * i64::from(u_corner))
              / 2,
          );
        }

        // :671 to :674
        turtle.miter(s_corner, false);
        turtle.u_shape(start_side, u_corner, saturate_i32(top));
        turtle.miter(s_corner, false);
        turtle.chain.append(
          p + direction_v_b + direction.resize(saturate_i32(2 * spacing)),
        );
      }

      // :677. The two check types and the corner type produce nothing but
      // the start point.
      MeanderType::CheckStart
      | MeanderType::CheckFinish
      | MeanderType::Corner => {}
    }

    let mut chain = turtle.chain;

    // :681 to :686
    if side {
      chain.mirror(&Seg::new(p, p + direction));
    }

    chain
  }

  // -------------------------------------------------------------------
  // Fitting
  // -------------------------------------------------------------------

  /// Find the tallest amplitude at which this meander fits.
  ///
  /// Port of `Fit`, `pcbnew/router/pns_meander.cpp:723`, the only entry
  /// point that produces a fitted meander.
  ///
  /// [`MeanderType::CheckStart`] and [`MeanderType::CheckFinish`] are a
  /// two step lookahead: "can I fit a start here **and** a turn after it".
  /// On success the shape becomes the first of the two primitives and the
  /// second is thrown away, which is what lets
  /// [`MeanderedLine::meander_segment`] decide between opening a turning
  /// run and placing an isolated single.
  ///
  /// The scan is linear, downwards from
  /// [`MeanderShape::max_amplitude`] in [`MeanderSettings::step`]
  /// decrements, and it stops at the first amplitude that clears both the
  /// corner radius floor and the context's fit check. The order is part of
  /// the answer, not a search strategy: a meander is always as tall as it
  /// can be at fitting time, and [`tune_line_length`] shrinks it
  /// afterwards.
  ///
  /// The corner radius rejection at `:812` compares against `width / 2`
  /// while [`MeanderShape::corner_radius`]'s own chamfer floor is
  /// `width / 2 * 0.5857864`, strictly below it, so a chamfered meander
  /// whose optimum radius is clamped down to that floor is rejected here.
  /// The two floors disagree by design; the comment at `:810` cites KiCad
  /// issue 8629 and the intent is visual.
  pub fn fit(
    &mut self,
    context: &MeanderContext<'_>,
    placed: &MeanderedLine,
    meander_type: MeanderType,
    seg: Seg,
    p: Vec2,
    side: bool,
  ) -> bool {
    // :731 to :742
    let primitives = match meander_type {
      MeanderType::CheckStart => Some((MeanderType::Start, MeanderType::Turn)),
      MeanderType::CheckFinish => {
        Some((MeanderType::Turn, MeanderType::Finish))
      }
      _ => None,
    };

    if let Some((first, second)) = primitives {
      // :746 to :751
      let mut m1 = Self::new(self.width, self.dual);
      let mut m2 = Self::new(self.width, self.dual);
      m1.set_baseline_offset(self.baseline_offset);
      m2.set_baseline_offset(self.baseline_offset);

      // :752 to :757
      let fits_first = m1.fit(context, placed, first, seg, p, side);
      let fits_second =
        fits_first && m2.fit(context, placed, second, seg, m1.end(), !side);

      if !(fits_first && fits_second) {
        // :774
        return false;
      }

      // :760 to :772. The `m_baseSeg = aSeg` at :763 is dead, because
      // :768 overwrites it with `m1.m_baseSeg`, which `m1` was fitted
      // against the same segment for. Erratum E17.
      self.meander_type = first;
      self.shapes[0] = m1.shapes[0].clone();
      self.shapes[1] = m1.shapes[1].clone();
      self.p0 = p;
      self.side = side;
      self.amplitude = m1.amplitude;
      self.dual = m1.dual;
      self.base_seg = m1.base_seg;
      self.update_base_segment();
      self.baseline_offset = m1.baseline_offset;

      return true;
    }

    // :780, :781
    let min_amplitude = self.min_amplitude(context);
    let max_amplitude = self.max_amplitude(context);

    // :787. Deliberately not the same floor as `corner_radius`'s.
    let min_corner_radius = self.width / 2;

    let step = context.settings().step();
    let direction = seg.b - seg.a;
    let offset = self.baseline_offset;
    let mut amplitude = max_amplitude;

    // :789. The step is positive because `MeanderSettings::new` refuses a
    // step that is not, which is erratum E22 closed at the boundary.
    loop {
      if amplitude < min_amplitude {
        return false;
      }

      // :791
      self.amplitude = amplitude;

      if self.dual {
        // :795, :796
        let lane_p = self.gen_meander_shape(
          context,
          p,
          direction,
          side,
          meander_type,
          offset,
        );
        let lane_n = self.gen_meander_shape(
          context,
          p,
          direction,
          side,
          meander_type,
          -offset,
        );
        self.shapes[0] = lane_p;
        self.shapes[1] = lane_n;
      } else {
        // :800
        let lane =
          self.gen_meander_shape(context, p, direction, side, meander_type, 0);
        self.shapes[0] = lane;
      }

      // :803 to :808
      self.meander_type = meander_type;
      self.base_seg = seg;
      self.p0 = p;
      self.side = side;
      self.update_base_segment();

      // :812
      if self.mean_corner_radius >= min_corner_radius
        && context.check_fit(self, placed)
      {
        // :815
        return true;
      }

      match amplitude.checked_sub(step) {
        Some(next) => amplitude = next,
        None => return false,
      }
    }
  }

  // -------------------------------------------------------------------
  // The mutators
  // -------------------------------------------------------------------

  /// Regenerate the chains from the current type and amplitude.
  ///
  /// Port of `Recalculate`, `pcbnew/router/pns_meander.cpp:823`. Note it
  /// uses the **unclipped** base segment as its direction (`:825`), where
  /// [`MeanderShape::make_empty`] uses the clipped one.
  pub fn recalculate(&mut self, context: &MeanderContext<'_>) {
    let p0 = self.p0;
    let direction = self.base_seg.b - self.base_seg.a;
    let side = self.side;
    let meander_type = self.meander_type;
    let offset = if self.dual { self.baseline_offset } else { 0 };

    let lane_p = self.gen_meander_shape(
      context,
      p0,
      direction,
      side,
      meander_type,
      offset,
    );
    self.shapes[0] = lane_p;

    if self.dual {
      let lane_n = self.gen_meander_shape(
        context,
        p0,
        direction,
        side,
        meander_type,
        -self.baseline_offset,
      );
      self.shapes[1] = lane_n;
    }

    self.update_base_segment();
  }

  /// Change the amplitude and regenerate.
  ///
  /// Port of `Resize`, `pcbnew/router/pns_meander.cpp:836`. A negative
  /// amplitude is ignored outright (`:838`), and the new amplitude is
  /// floored by [`MeanderShape::min_amplitude`] (`:844`, KiCad issue
  /// 8629), which is why the amplitude search papers over its zero
  /// sentinel with a `max` at the call site.
  pub fn resize(&mut self, context: &MeanderContext<'_>, amplitude: i32) {
    // :838
    if amplitude < 0 {
      return;
    }

    // :844
    self.amplitude = amplitude.max(self.min_amplitude(context));
    self.recalculate(context);
  }

  /// Replace the meander with a straight bypass of the baseline it
  /// consumed.
  ///
  /// Port of `MakeEmpty`, `pcbnew/router/pns_meander.cpp:850`. Two details
  /// carry the behaviour: the clipped base segment is updated **before**
  /// the regeneration rather than after (`:852`), and the direction is the
  /// clipped segment rather than the whole one (`:854`). Together they
  /// mean an emptied meander keeps exactly the footprint it had, which is
  /// what `tuneLineLength` needs so the survivors do not slide together.
  pub fn make_empty(&mut self, context: &MeanderContext<'_>) {
    // :852
    self.update_base_segment();

    // :854
    let direction = self.clipped_base_seg.b - self.clipped_base_seg.a;
    let p0 = self.p0;
    let side = self.side;

    // :856, :857
    self.meander_type = MeanderType::Empty;
    self.amplitude = 0;

    let offset = if self.dual { self.baseline_offset } else { 0 };
    let lane_p = self.gen_meander_shape(
      context,
      p0,
      direction,
      side,
      MeanderType::Empty,
      offset,
    );
    self.shapes[0] = lane_p;

    if self.dual {
      let lane_n = self.gen_meander_shape(
        context,
        p0,
        direction,
        side,
        MeanderType::Empty,
        -self.baseline_offset,
      );
      self.shapes[1] = lane_n;
    }
  }

  /// Make this a one point marker for a corner of the line being tuned.
  ///
  /// Port of `MakeCorner`, `pcbnew/router/pns_meander.cpp:904`. The
  /// clipped base segment becomes degenerate, so the corner contributes
  /// zero baseline and zero length to every sum in
  /// [`tune_line_length`].
  ///
  /// KiCad's `MakeArc` (`:916`) is the same routine for a pre existing
  /// arc, and it sets `MT_CORNER` rather than `MT_ARC`, which is why
  /// `MT_ARC` is not a [`MeanderType`] here (erratum E4). Arcs are on
  /// hold, so there is nothing to port.
  pub fn make_corner(&mut self, p1: Vec2, p2: Vec2) {
    self.meander_type = MeanderType::Corner;
    self.shapes[0].clear();
    self.shapes[1].clear();
    self.shapes[0].append(p1);
    self.shapes[1].append(p2);
    self.clipped_base_seg = Seg::new(p1, p1);
  }

  /// Recompute which part of the base segment the chains project onto.
  ///
  /// Port of `updateBaseSegment`, `pcbnew/router/pns_meander.cpp:967`. For
  /// a dual meander the two chains' endpoints are averaged first, so the
  /// clipped segment tracks the pair's centreline.
  pub fn update_base_segment(&mut self) {
    // KiCad indexes the chains unguarded and would read out of bounds for
    // an empty one; every caller there has just generated them.
    let (Some(first_p), Some(last_p)) = (
      self.shapes[0].points().first().copied(),
      self.shapes[0].last_point(),
    ) else {
      return;
    };

    let (start, end) = if self.dual {
      // :971 to :976
      let (Some(first_n), Some(last_n)) = (
        self.shapes[1].points().first().copied(),
        self.shapes[1].last_point(),
      ) else {
        return;
      };

      (midpoint(first_p, first_n), midpoint(last_p, last_n))
    } else {
      // :979 to :982
      (first_p, last_p)
    };

    self.clipped_base_seg = Seg::new(
      self.base_seg.line_project(start),
      self.base_seg.line_project(end),
    );
  }

  // -------------------------------------------------------------------
  // The measurements
  // -------------------------------------------------------------------

  /// How much base line this meander consumes.
  ///
  /// Port of `BaselineLength()`, `pcbnew/router/pns_meander.cpp:944`,
  /// documented in the header as "the minimum tuned length".
  #[must_use]
  pub fn baseline_length(&self) -> i32 {
    self.clipped_base_seg.length()
  }

  /// How long the generated chain is.
  ///
  /// Port of `CurrentLength()`, `pcbnew/router/pns_meander.cpp:950`, which
  /// is `CLine( 0 ).Length()`: a per segment sum of rounded lengths, so it
  /// is not the length of the exact polyline. That is what every
  /// comparison in [`tune_line_length`] is made in, so it is reproduced.
  #[must_use]
  pub fn meander_length(&self) -> i64 {
    self.shapes[0].length()
  }

  /// The shortest this meander can be made without changing its
  /// footprint.
  ///
  /// Port of `MinTunableLength()`, `pcbnew/router/pns_meander.cpp:956`. It
  /// is what [`tune_line_length`] decides whether a meander is worth
  /// keeping by.
  #[must_use]
  pub fn min_tunable_length(&self, context: &MeanderContext<'_>) -> i64 {
    let mut copy = self.clone();

    copy.set_target_baseline_length(self.baseline_length());
    copy.resize(context, copy.min_amplitude(context));

    copy.meander_length()
  }
}

// ---------------------------------------------------------------------
// MeanderedLine
// ---------------------------------------------------------------------

/// What [`MeanderedLine::meander_segment`] answers.
///
/// KiCad's `MeanderSegment` writes its side flip straight back into the
/// placer's settings through `UpdateSettings`
/// (`pcbnew/router/pns_meander.cpp:287`), which is the one place in the
/// router where the shape generator mutates its placer. Note 08 section
/// 11.4 asks for it as a return value instead, so the generator stays a
/// pure function of its context.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct MeanderSegmentResult {
  /// Whether the caller should call
  /// [`MeanderSettings::flip_initial_side`].
  ///
  /// KiCad's lambda can fire more than once in one loop and each firing
  /// negates the side, so what matters is the parity; this is that
  /// parity. Nothing inside the loop reads
  /// [`MeanderSettings::initial_side`], so applying the flip after the
  /// loop instead of during it changes nothing.
  pub initial_side_flipped: bool,
}

/// An ordered list of meanders covering one stretch of a line.
///
/// Port of `MEANDERED_LINE`, `pcbnew/router/pns_meander.h:469`. KiCad
/// stores `MEANDER_SHAPE*` and news and deletes them (`pns_meander.cpp:297`,
/// `:938`), which is what its move assignment operator and its `Clear()`
/// exist for; a `Vec` of values needs neither.
#[derive(Clone, Debug)]
pub struct MeanderedLine {
  /// `m_last` (`pcbnew/router/pns_meander.h:611`), the point the next
  /// meander starts from.
  last: Vec2,
  /// `m_meanders` (`:613`), owned by value here.
  meanders: Vec<MeanderShape>,
  /// `m_dual` (`:615`).
  dual: bool,
  /// `m_width` (`:617`).
  width: i32,
  /// `m_baselineOffset` (`:618`).
  baseline_offset: i32,
}

impl MeanderedLine {
  /// An empty line of a given width.
  ///
  /// Port of the constructor at `pcbnew/router/pns_meander.h:475`, minus
  /// the placer back pointer.
  #[must_use]
  pub const fn new(width: i32, dual: bool) -> Self {
    Self {
      last: Vec2::new(0, 0),
      meanders: Vec::new(),
      dual,
      width,
      baseline_offset: 0,
    }
  }

  /// The meanders, in the order they were placed.
  ///
  /// Port of `Meanders()`, `pcbnew/router/pns_meander.h:583`.
  #[must_use]
  pub fn meanders(&self) -> &[MeanderShape] {
    &self.meanders
  }

  /// The meanders, for a pass that resizes them.
  ///
  /// This is what [`tune_line_length`] walks; KiCad gets the same access
  /// out of a `std::vector<MEANDER_SHAPE*>`.
  pub fn meanders_mut(&mut self) -> &mut [MeanderShape] {
    &mut self.meanders
  }

  /// Drop every meander.
  ///
  /// Port of `Clear()`, `pcbnew/router/pns_meander.cpp:936`, which in
  /// KiCad also has to `delete` each one.
  pub fn clear(&mut self) {
    self.meanders.clear();
  }

  /// The width of the line being meandered.
  ///
  /// Port of `Width()`, `pcbnew/router/pns_meander.h:568`.
  #[must_use]
  pub const fn width(&self) -> i32 {
    self.width
  }

  /// Set the width of the line being meandered.
  ///
  /// Port of `SetWidth`, `pcbnew/router/pns_meander.h:560`. It does not
  /// touch the meanders already placed, which is KiCad's behaviour too.
  pub const fn set_width(&mut self, width: i32) {
    self.width = width;
  }

  /// Half the pair pitch every meander is built at.
  ///
  /// Port of `BaselineOffset()`, `pcbnew/router/pns_meander.h:603`.
  #[must_use]
  pub const fn baseline_offset(&self) -> i32 {
    self.baseline_offset
  }

  /// Set half the pair pitch.
  ///
  /// Port of `SetBaselineOffset`, `pcbnew/router/pns_meander.h:595`.
  pub const fn set_baseline_offset(&mut self, offset: i32) {
    self.baseline_offset = offset;
  }

  /// Where the next meander starts.
  ///
  /// KiCad has no accessor for `m_last`; this is here because the fitting
  /// loop's progress is the only thing a test can watch.
  #[must_use]
  pub const fn last(&self) -> Vec2 {
    self.last
  }

  /// Add a one point marker for a corner of the line being tuned.
  ///
  /// Port of `AddCorner`, `pcbnew/router/pns_meander.cpp:866`. It advances
  /// the cursor to the point it was given, where
  /// [`MeanderedLine::add_meander`] advances it to the far end of the
  /// clipped base segment instead; that asymmetry is what keeps the
  /// meanders marching along the base line rather than along the
  /// meandered path.
  pub fn add_corner(&mut self, a: Vec2, b: Vec2) {
    let mut meander = MeanderShape::new(self.width, self.dual);

    meander.make_corner(a, b);
    self.last = a;
    self.meanders.push(meander);
  }

  /// Add a fitted meander.
  ///
  /// Port of `AddMeander`, `pcbnew/router/pns_meander.cpp:928`.
  pub fn add_meander(&mut self, shape: MeanderShape) {
    self.last = shape.base_segment().b;
    self.meanders.push(shape);
  }

  /// Whether a candidate stays clear of everything already placed.
  ///
  /// Port of `CheckSelfIntersections`, `pcbnew/router/pns_meander.cpp:695`,
  /// which both placers' `CheckFit` ends on.
  ///
  /// The parallel skip at `:707` is what makes it cheap: meanders on the
  /// same base segment are all parallel to each other and are skipped
  /// wholesale, so only meanders from **other** base segments of the same
  /// line are actually tested.
  ///
  /// Erratum E7: chain `0` is the only one tested, both as the candidate
  /// and as the obstacle, so a dual meander's N lane is never checked
  /// against anything. Transcribed as it stands. Note 08 argues it is
  /// hard to construct a case where the N lane self intersects without
  /// the P lane doing so, since the two are a fixed offset apart and the
  /// offset is baked into `spacing()` and `cornerRadius()`, but it is
  /// still a hole.
  #[must_use]
  pub fn check_self_intersections(
    &self,
    shape: &MeanderShape,
    clearance: i32,
  ) -> bool {
    // :697. Backwards, as KiCad does; the answer is a bool, so the order
    // only decides which collision would be reported first.
    for meander in self.meanders.iter().rev() {
      // :701
      if matches!(
        meander.meander_type(),
        MeanderType::Empty | MeanderType::Corner
      ) {
        continue;
      }

      // :707
      if shape.base_segment().approx_parallel(
        &meander.base_segment(),
        Seg::APPROX_DISTANCE_THRESHOLD,
      ) {
        continue;
      }

      // :710 to :716. Erratum E7: chain 0 on both sides.
      let obstacle = meander.cline(0);

      for index in (0..obstacle.segment_count()).rev() {
        if shape
          .cline(0)
          .collide_seg(&obstacle.segment(index), clearance)
          .is_some()
        {
          return false;
        }
      }
    }

    true
  }

  /// Fill one base segment with as many meanders as fit.
  ///
  /// Port of `MeanderSegment`, `pcbnew/router/pns_meander.cpp:252`, the
  /// heart of the milestone.
  ///
  /// The three arms of its `if` chain are mutually exclusive. With
  /// [`MeanderSettings::single_sided`] set the first two are unreachable
  /// and a run is a plain sequence of [`MeanderType::Single`] shapes with
  /// no turning; otherwise the first arm handles a long stretch (more
  /// than three periods left), the second closes an open turning run when
  /// the stretch got short, and the third places an isolated single on a
  /// medium stretch.
  ///
  /// Every quantity KiCad computes here in a `double` is integer valued:
  /// `SEG::Length` and `VECTOR2I::EuclideanNorm` both return `int`, and
  /// `3.0 * thr` and `thr * 2.0` are exact for any board sized value. Note
  /// 08 section 8.2.
  pub fn meander_segment(
    &mut self,
    context: &MeanderContext<'_>,
    base: Seg,
    side: bool,
  ) -> MeanderSegmentResult {
    // :254, :258, :260
    let base_length = i64::from(base.length());
    let single_sided = context.settings().single_sided();
    let direction = base.b - base.a;
    let step = i64::from(context.settings().step());

    let mut side = side;
    let mut initial_side_flipped = false;

    // :263
    if !self.dual {
      self.add_corner(base.a, Vec2::new(0, 0));
    }

    // :265 to :268. The `m_last = aBase.A` at :268 is redundant, because
    // `AddCorner` already set it; erratum E1.
    let mut turning = false;
    let mut started = false;

    loop {
      // :272 to :275
      let mut candidate = MeanderShape::new(self.width, self.dual);
      candidate.set_baseline_offset(self.baseline_offset);

      // :277
      let threshold = i64::from(candidate.spacing(context));

      let mut failed = false;

      // :280
      let mut remaining =
        base_length - i64::from((self.last - base.a).euclidean_norm());

      // :317
      if remaining < step {
        break;
      }

      // :320
      if !single_sided && remaining > 3 * threshold {
        if turning {
          // :346
          if candidate.fit(
            context,
            self,
            MeanderType::CheckFinish,
            base,
            self.last,
            side,
          ) {
            // :350 to :354
            candidate.fit(
              context,
              self,
              MeanderType::Turn,
              base,
              self.last,
              side,
            );
            self.add_meander(candidate);
            side = !side;
            started = true;
          } else {
            // :357 to :361
            candidate.fit(
              context,
              self,
              MeanderType::Finish,
              base,
              self.last,
              side,
            );
            started = false;
            self.add_meander(candidate);
            turning = false;
          }
        } else {
          // :324 to :339
          for attempt in 0..2 {
            let check_side = if attempt == 0 { side } else { !side };

            if candidate.fit(
              context,
              self,
              MeanderType::CheckStart,
              base,
              self.last,
              check_side,
            ) {
              // :330
              if !started && check_side != side {
                initial_side_flipped = !initial_side_flipped;
              }

              turning = true;
              self.add_meander(candidate.clone());
              side = !check_side;
              started = true;
              break;
            }
          }

          // :342
          if !turning {
            let outcome = self.add_single_if_fits(
              context,
              &mut candidate,
              base,
              side,
              started,
              single_sided,
            );
            failed = outcome.failed;
            side = outcome.side;
            started = outcome.started;
            initial_side_flipped ^= outcome.flip_initial_side;
          }
        }
      } else if !single_sided && started {
        // :366 to :371
        if candidate.fit(
          context,
          self,
          MeanderType::Finish,
          base,
          self.last,
          side,
        ) {
          self.add_meander(candidate);
        }

        break;
      } else if !turning && remaining > threshold * 2 {
        // :376
        let outcome = self.add_single_if_fits(
          context,
          &mut candidate,
          base,
          side,
          started,
          single_sided,
        );
        failed = outcome.failed;
        side = outcome.side;
        started = outcome.started;
        initial_side_flipped ^= outcome.flip_initial_side;
      } else {
        // :380
        failed = true;
      }

      // :383, :385
      remaining =
        base_length - i64::from((self.last - base.a).euclidean_norm());

      if remaining < step {
        break;
      }

      // :388
      if failed {
        // :390 to :394. `tmp` is a fresh shape whose amplitude is zero,
        // and `cornerRadius()` answers zero for that, so the
        // `- 2 * tmp.cornerRadius()` term of KiCad's expression is always
        // zero: the advance is `spacing() + m_step`. Erratum E6.
        let mut skip = MeanderShape::new(self.width, self.dual);
        skip.set_baseline_offset(self.baseline_offset);

        let next = i64::from(skip.spacing(context)) + step;

        // :395
        let point = self.last + direction.resize(saturate_i32(next));

        // :397
        if base.contains_point(point) && !self.dual {
          self.add_corner(point, Vec2::new(0, 0));
        } else {
          break;
        }
      }
    }

    // :407
    if !self.dual {
      self.add_corner(base.b, Vec2::new(0, 0));
    }

    MeanderSegmentResult {
      initial_side_flipped,
    }
  }

  /// Try to place one isolated meander, on the current side and then on
  /// the other.
  ///
  /// Port of the `addSingleIfFits` lambda,
  /// `pcbnew/router/pns_meander.cpp:290` to `:314`. It is a method rather
  /// than a closure because it needs `&mut self` while the caller holds
  /// the candidate.
  fn add_single_if_fits(
    &mut self,
    context: &MeanderContext<'_>,
    candidate: &mut MeanderShape,
    base: Seg,
    side: bool,
    started: bool,
    single_sided: bool,
  ) -> AddSingleOutcome {
    let mut outcome = AddSingleOutcome {
      // :292
      failed: true,
      side,
      started,
      flip_initial_side: false,
    };

    // :294
    if candidate.fit(context, self, MeanderType::Single, base, self.last, side)
    {
      // :296 to :299
      self.add_meander(candidate.clone());
      outcome.failed = false;
      outcome.started = false;
    }

    // :302
    if outcome.failed && !single_sided {
      // :304
      if candidate.fit(
        context,
        self,
        MeanderType::Single,
        base,
        self.last,
        !side,
      ) {
        // :306 to :312
        if !started {
          outcome.flip_initial_side = true;
        }

        self.add_meander(candidate.clone());
        outcome.failed = false;
        outcome.started = false;
        outcome.side = !side;
      }
    }

    outcome
  }
}

/// What [`MeanderedLine::add_single_if_fits`] changes in its caller.
///
/// KiCad's lambda captures `fail`, `side` and `started` by reference and
/// writes all three (`pcbnew/router/pns_meander.cpp:292`, `:299`, `:311`),
/// plus the placer's initial side through `flipInitialSide` (`:307`).
struct AddSingleOutcome {
  /// KiCad's `fail`.
  failed: bool,
  /// KiCad's `side`.
  side: bool,
  /// KiCad's `started`.
  started: bool,
  /// Whether `flipInitialSide` fired.
  flip_initial_side: bool,
}

// ---------------------------------------------------------------------
// The shared placer arithmetic
// ---------------------------------------------------------------------

/// Shrink and delete meanders until the line lands on its target.
///
/// Port of `MEANDER_PLACER_BASE::tuneLineLength`,
/// `pcbnew/router/pns_meander_placer_base.cpp:203`. It is handed a fitted
/// line whose meanders are all at the maximum amplitude
/// [`MeanderShape::fit`] gave them, and the number of nanometres the line
/// still has to gain.
///
/// Three passes.
///
/// 1. Truncate the run: walk the meanders accumulating what they give at
///    full amplitude, and at the first one that would overshoot, turn it
///    into an end shape and empty everything after it. The accumulators
///    are updated **after** the mutation, so an emptied meander
///    contributes zero from that point on, and the `finished` flag never
///    resets.
/// 2. Measure what the survivors actually give.
/// 3. Take an equal share of the excess off each survivor, through
///    [`find_amplitude_for_length`].
///
/// The early return in pass 2 is the "still too short even at full
/// amplitude" case: nothing is shrunk and the status ends up
/// [`TuningStatus::TooShort`]. The opposite case, a negative elongation
/// because the line is already too long, is decided before this is ever
/// called (`pns_meander_placer.cpp:282`).
///
/// `SetTargetBaselineLength` before the resize at `:288` is what keeps a
/// shrunk meander from also shrinking its footprint: the flat top widens
/// to compensate for the shorter sides, so the meanders do not slide
/// together and leave a gap at the end of the tuned stretch.
///
/// The `MT_ARC` skips at `:211`, `:261` and `:276` are dead in KiCad
/// (erratum E4) and are not transcribed.
pub fn tune_line_length(
  context: &MeanderContext<'_>,
  tuned: &mut MeanderedLine,
  elongation: i64,
) {
  let mut max_elongation: i64 = 0;
  let mut min_elongation: i64 = 0;
  let mut finished = false;

  // :209
  for meander in tuned.meanders_mut() {
    // :211
    if meander.meander_type() == MeanderType::Corner {
      continue;
    }

    // :216 to :219
    let end_type = match meander.meander_type() {
      MeanderType::Start | MeanderType::Single => MeanderType::Single,
      _ => MeanderType::Finish,
    };

    // :213, :221, :222
    let mut end = meander.clone();
    end.set_type(end_type);
    end.recalculate(context);

    // :224
    let max_end_elongation =
      end.meander_length() - i64::from(end.baseline_length());

    // :226
    if max_elongation + max_end_elongation > elongation {
      if finished {
        // :247
        meander.make_empty(context);
      } else {
        // :230, :231
        meander.set_type(end_type);
        meander.recalculate(context);

        // :233
        if end_type == MeanderType::Single {
          // :236, :237
          let end_min_elongation = meander.min_tunable_length(context)
            - i64::from(meander.baseline_length());

          // :239, :240
          if min_elongation + end_min_elongation >= elongation {
            meander.make_empty(context);
          }
        }

        finished = true;
      }
    }

    // :251, :252
    max_elongation +=
      meander.meander_length() - i64::from(meander.baseline_length());
    min_elongation += meander.min_tunable_length(context)
      - i64::from(meander.baseline_length());
  }

  // :256 to :266
  let mut remaining_elongation = elongation;
  let mut meander_count: i32 = 0;

  for meander in tuned.meanders() {
    if is_tunable(meander.meander_type()) {
      remaining_elongation -=
        meander.meander_length() - i64::from(meander.baseline_length());
      meander_count += 1;
    }
  }

  // :268 to :272
  let mut length_reduction_left = -remaining_elongation;
  let mut meanders_left = meander_count;

  if length_reduction_left < 0 || meanders_left == 0 {
    return;
  }

  // :274
  for meander in tuned.meanders_mut() {
    if !is_tunable(meander.meander_type()) {
      continue;
    }

    // :278 to :281
    let reduction_here = length_reduction_left / i64::from(meanders_left);
    let initial_length = meander.meander_length();
    let min_amplitude = meander.min_amplitude(context);

    // :282
    let amplitude = find_amplitude_for_length(
      context,
      meander,
      initial_length - reduction_here,
      min_amplitude,
      meander.amplitude(),
    );

    // :285, :286. The `max` papers over the amplitude search's use of
    // zero as both "not found" and a legal amplitude.
    let amplitude = amplitude.max(min_amplitude);

    // :288, :289
    meander.set_target_baseline_length(meander.baseline_length());
    meander.resize(context, amplitude);

    // :291 to :295
    length_reduction_left -= initial_length - meander.meander_length();
    meanders_left -= 1;

    if meanders_left == 0 {
      break;
    }
  }
}

/// Whether a meander is one [`tune_line_length`]'s later passes count.
///
/// Port of the `Type() != MT_CORNER && Type() != MT_ARC && Type() != MT_EMPTY`
/// test at `pcbnew/router/pns_meander_placer_base.cpp:261` and `:276`,
/// minus the `MT_ARC` half that is dead (erratum E4).
const fn is_tunable(meander_type: MeanderType) -> bool {
  !matches!(meander_type, MeanderType::Corner | MeanderType::Empty)
}

/// Find the amplitude at which a meander is a given length.
///
/// Port of `findAmplitudeForLength`,
/// `pcbnew/router/pns_meander_placer_base.cpp:181`, a free function with
/// external linkage and no declaration in any header, so nothing outside
/// that translation unit can call it in KiCad.
///
/// The initial guess is `amplitude - delta / 2`, the right first order
/// estimate because a meander's elongation is `2 * amplitude` plus a
/// corner correction.
///
/// **Erratum E8, reproduced.** The fast path resizes the working copy to
/// the **minimum** amplitude (`:192`) and then returns the initial guess
/// (`:195`), which the guard at `:190` has already established is a
/// different number except by coincidence. So the fast path fires only
/// when the minimum amplitude is already within
/// [`LENGTH_TARGET_TOLERANCE`] of the target, and then answers with an
/// amplitude that was never measured. The intent is plainly
/// `copy.Resize( initialGuess )`.
///
/// KiCad's target length is an `int` and its caller passes a `long long`;
/// the implicit narrowing is not reproduced, because it can only bite for
/// a length over 2.1 metres.
#[must_use]
pub fn find_amplitude_for_length(
  context: &MeanderContext<'_>,
  meander: &MeanderShape,
  target_length: i64,
  min_amplitude: i32,
  max_amplitude: i32,
) -> i32 {
  // :183, :186
  let mut copy = meander.clone();
  copy.set_target_baseline_length(meander.baseline_length());

  // :188
  let initial_guess = i64::from(meander.amplitude())
    - (meander.meander_length() - target_length) / 2;

  // :190. The bounds are two `i32`s, so the guess fits an `i32` exactly
  // when the guard can hold.
  if let Ok(guess) = i32::try_from(initial_guess)
    && guess >= min_amplitude
    && guess <= max_amplitude
  {
    // :192. Erratum E8: this measures `min_amplitude`, and :195 returns
    // `guess`.
    copy.resize(context, min_amplitude);

    // :194
    if (copy.meander_length() - target_length).abs() < LENGTH_TARGET_TOLERANCE {
      // :195
      return guess;
    }
  }

  // :199
  find_amplitude_binary_search(
    context,
    &mut copy,
    target_length,
    min_amplitude,
    max_amplitude,
  )
}

/// Bisect an amplitude interval for the one that hits a length.
///
/// Port of `findAmplitudeBinarySearch`,
/// `pcbnew/router/pns_meander_placer_base.cpp:137`. Zero doubles as "not
/// found" and as a legal amplitude, which the caller papers over with a
/// `max` against the minimum amplitude (`:285`).
///
/// **Erratum E16, reproduced.** The base case at `:139` returns `max_amplitude`
/// with no length test at all, so a narrow interval bottoms out immediately
/// and reports success with an amplitude that can miss the target by far
/// more than [`LENGTH_TARGET_TOLERANCE`].
///
/// The recursion tries the left half before the right and returns the
/// first non zero answer (`:164` to `:172`), so the traversal order is
/// part of the result and is reproduced exactly.
///
/// It terminates for every input the tuner produces. The interval halves
/// each level, and the one case that could recurse on itself, an interval
/// two nanometres wide whose midpoint equals its lower bound, cannot get
/// past the two bracket tests at `:148` and `:151`: an amplitude change of
/// one nanometre moves the length by about two, so one of the two errors
/// is always inside the twenty nanometre tolerance.
///
/// The `int minLen = aCopy.CurrentLength()` narrowings at `:143` and
/// `:146` are not reproduced; they can only bite for a length over 2.1
/// metres.
#[must_use]
pub fn find_amplitude_binary_search(
  context: &MeanderContext<'_>,
  copy: &mut MeanderShape,
  target_length: i64,
  min_amplitude: i32,
  max_amplitude: i32,
) -> i32 {
  // :139. Erratum E16.
  if min_amplitude == max_amplitude {
    return max_amplitude;
  }

  // :142 to :146
  copy.resize(context, min_amplitude);
  let min_length = copy.meander_length();

  copy.resize(context, max_amplitude);
  let max_length = copy.meander_length();

  // :148, :151
  if min_length > target_length || max_length < target_length {
    return 0;
  }

  // :154, :155
  let min_error = min_length - target_length;
  let max_error = max_length - target_length;

  // :157 to :162
  if min_error.abs() < LENGTH_TARGET_TOLERANCE
    || max_error.abs() < LENGTH_TARGET_TOLERANCE
  {
    return if min_error.abs() < max_error.abs() {
      min_amplitude
    } else {
      max_amplitude
    };
  }

  // KiCad's `( minAmp + maxAmp ) / 2` overflows an `int` for two large
  // amplitudes; the midpoint of two `i32`s always fits one, so the
  // fallback is unreachable.
  let middle =
    i32::try_from((i64::from(min_amplitude) + i64::from(max_amplitude)) / 2)
      .unwrap_or(max_amplitude);

  // :164 to :169
  let left = find_amplitude_binary_search(
    context,
    copy,
    target_length,
    min_amplitude,
    middle,
  );

  if left != 0 {
    return left;
  }

  // :170 to :175
  let right = find_amplitude_binary_search(
    context,
    copy,
    target_length,
    middle,
    max_amplitude,
  );

  if right != 0 {
    return right;
  }

  // :178
  0
}

/// Nudge the maximum amplitude by one step.
///
/// Port of `MEANDER_PLACER_BASE::AmplitudeStep`,
/// `pcbnew/router/pns_meander_placer_base.cpp:96`, the live adjustment the
/// host binds to a key (`pcbnew/generators/pcb_tuning_pattern.cpp:2764`).
///
/// KiCad computes the sum in an `int` and lets it overflow; this computes
/// it in an `i64` and saturates, which is the only difference and cannot
/// be reached with a board sized amplitude.
pub fn amplitude_step(settings: &mut MeanderSettings, sign: i32) {
  // :98
  let stepped = i64::from(settings.max_amplitude())
    + i64::from(sign) * i64::from(settings.step());

  // :99, :101
  settings.set_max_amplitude(saturate_i32(
    stepped.max(i64::from(settings.min_amplitude())),
  ));
}

/// Nudge the spacing by one step.
///
/// Port of `MEANDER_PLACER_BASE::SpacingStep`,
/// `pcbnew/router/pns_meander_placer_base.cpp:105`. The floor is the width
/// plus the clearance, which is the same quantity
/// [`MeanderShape::spacing`] floors the period by.
pub fn spacing_step(
  settings: &mut MeanderSettings,
  sign: i32,
  current_width: i32,
  clearance: i32,
) {
  // :107
  let stepped = i64::from(settings.spacing())
    + i64::from(sign) * i64::from(settings.step());

  // :108, :110
  settings.set_spacing(saturate_i32(
    stepped.max(i64::from(current_width) + i64::from(clearance)),
  ));
}

/// The copper to copper clearance a tuned track works to.
///
/// Port of `MEANDER_PLACER_BASE::Clearance`,
/// `pcbnew/router/pns_meander_placer_base.cpp:114`. KiCad picks the first
/// item of the placer's trace on the assumption that every track in the
/// tuned path is on the same net class and therefore has the same
/// clearance (`:116` to `:119`); the caller here passes whichever item it
/// wants that assumption made about.
///
/// The fallback when the host has no minimum clearance rule is the **track
/// width** (`:125`, a `wxCHECK_MSG`), which then floors
/// [`MeanderShape::spacing`] at twice the width. That is a surprising
/// default and it is reproduced deliberately.
///
/// This is called once per move here. KiCad calls it from `spacing()`,
/// which runs from `cornerRadius()`, from `genMeanderShape()` and twice
/// per iteration of the fitting loop, and each call rebuilds the placer's
/// whole trace before issuing an uncached rule query (erratum E11).
#[must_use]
pub fn clearance(
  resolver: &dyn RuleResolver,
  item: ItemRef<'_>,
  layer: i32,
  current_width: i32,
) -> i32 {
  // :122
  resolver
    .constraint(ConstraintType::Clearance, item, None, layer)
    // :125, :127
    .and_then(|constraint| constraint.min)
    .unwrap_or(current_width)
}

// ---------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------

/// Clamp an `i64` into an `i32`.
///
/// Every quantity in this module is board sized, so this never fires; it
/// exists because KiCad computes the same expressions in `int` and lets
/// them overflow, and a debug build here would panic instead.
const fn saturate_i32(value: i64) -> i32 {
  if value > i32::MAX as i64 {
    i32::MAX
  } else if value < i32::MIN as i64 {
    i32::MIN
  } else {
    value as i32
  }
}

/// The midpoint of two points, truncating toward zero per component.
///
/// Port of `( a + b ) / 2` on two `VECTOR2I`s as `updateBaseSegment` writes
/// it (`pcbnew/router/pns_meander.cpp:971`, `:972`). The sum is taken in
/// `i64` so two board sized points cannot overflow before the halving,
/// which they can in KiCad.
fn midpoint(a: Vec2, b: Vec2) -> Vec2 {
  Vec2::new(
    saturate_i32((i64::from(a.x) + i64::from(b.x)) / 2),
    saturate_i32((i64::from(a.y) + i64::from(b.y)) / 2),
  )
}

#[cfg(test)]
mod tests {
  use std::f64::consts::{PI, SQRT_2};

  use super::*;
  use crate::item::{Item, ItemBody, LayerRange, Segment};
  use crate::rules::{Constraint, FixedClearance, Keepout};

  /// The width of the track every worked example in note 08 section 2.6
  /// is drawn for.
  const NOTE_WIDTH: i32 = 200_000;

  /// The clearance those examples resolve to.
  const NOTE_CLEARANCE: i32 = 100_000;

  /// Settings with KiCad's defaults and one amplitude and spacing chosen,
  /// so that `fit`'s downward scan lands on the amplitude a test names.
  fn settings_at(max_amplitude: i32, spacing: i32) -> MeanderSettings {
    MeanderSettings::new(MeanderSettingsRequest {
      max_amplitude,
      spacing,
      ..MeanderSettingsRequest::default()
    })
    .expect("the defaults with one amplitude and spacing are valid")
  }

  /// `d` of note 08 section 2.6: the length of one chamfered corner, which
  /// is `KiROUND( c * sqrt( 2 ) )`.
  fn chamfer_chord(corner_radius: i32) -> i64 {
    i64::from(kiround(f64::from(corner_radius) * SQRT_2))
  }

  /// Fit one meander of a given type on a base segment along `+x` from
  /// the origin, with the accepting fit check.
  fn fit_along_x(
    context: &MeanderContext<'_>,
    meander_type: MeanderType,
  ) -> MeanderShape {
    let placed = MeanderedLine::new(NOTE_WIDTH, false);
    let base = Seg::new(Vec2::new(0, 0), Vec2::new(10_000_000, 0));
    let mut shape = MeanderShape::new(NOTE_WIDTH, false);

    assert!(
      shape.fit(context, &placed, meander_type, base, Vec2::new(0, 0), false),
      "the accepting check fits every type at the maximum amplitude"
    );

    shape
  }

  /// The total elongation a meandered line gives over its baseline.
  fn total_elongation(line: &MeanderedLine) -> i64 {
    line
      .meanders()
      .iter()
      .map(|meander| {
        meander.meander_length() - i64::from(meander.baseline_length())
      })
      .sum()
  }

  // -----------------------------------------------------------------
  // Settings
  // -----------------------------------------------------------------

  /// Every default of `MEANDER_SETTINGS`, pinned against the constructor
  /// at `pcbnew/router/pns_meander.cpp:42` to `:63`. The four KiCad's own
  /// suite asserts are the first four
  /// (`qa/tests/pcbnew/test_meander_corner_radius.cpp:43` to `:52`).
  #[test]
  fn meander_settings_defaults_are_kicads() {
    let settings = MeanderSettings::default();

    assert_eq!(settings.min_amplitude(), 200_000);
    assert_eq!(settings.max_amplitude(), 1_000_000);
    assert_eq!(settings.spacing(), 600_000);
    assert_eq!(settings.step(), 50_000);
    assert_eq!(settings.corner_radius_percentage(), 80);
    assert!(!settings.single_sided());
    assert_eq!(settings.initial_side(), MeanderSide::Left);
    assert!(!settings.keep_endpoints());
    assert_eq!(settings.target_length(), None);
    assert_eq!(settings.target_skew(), Some(LengthTarget::around(0)));

    // The one deliberate difference: KiCad's default corner style is
    // `MEANDER_STYLE_ROUND` (`:56`), which needs an arc.
    assert_eq!(settings.corner_style(), CornerStyle::Chamfer);
    assert_eq!(
      MeanderSettings::new(MeanderSettingsRequest::default()),
      Ok(MeanderSettings::default())
    );
  }

  /// Erratum E22: `Fit`'s amplitude loop decrements by the step
  /// (`pcbnew/router/pns_meander.cpp:789`) and `MeanderSegment` breaks on
  /// it twice (`:317`, `:385`), so a step of zero is a loop that never
  /// ends. It is refused at the boundary instead.
  #[test]
  fn the_settings_constructor_refuses_a_step_that_is_not_positive() {
    for step in [0, -1, -50_000] {
      assert_eq!(
        MeanderSettings::new(MeanderSettingsRequest {
          step,
          ..MeanderSettingsRequest::default()
        }),
        Err(MeanderSettingsError::NonPositiveStep),
      );
    }

    assert!(
      MeanderSettings::new(MeanderSettingsRequest {
        step: 1,
        ..MeanderSettingsRequest::default()
      })
      .is_ok()
    );
  }

  /// The round style needs a `SHAPE_ARC` (`pns_meander.cpp:496`), and
  /// silently drawing chamfers instead would miss the target by about
  /// `0.62 * radius` per meander, so it is refused rather than
  /// substituted.
  #[test]
  fn the_settings_constructor_refuses_the_round_corner_style() {
    assert_eq!(
      MeanderSettings::new(MeanderSettingsRequest {
        corner_style: MeanderStyle::Round,
        ..MeanderSettingsRequest::default()
      }),
      Err(MeanderSettingsError::RoundCornersUnsupported),
    );
  }

  /// `SetTargetLength( long long int )`,
  /// `pcbnew/router/pns_meander.cpp:67`, pads by
  /// [`DEFAULT_LENGTH_TOLERANCE`] except for the sentinel, which gets a
  /// minimum of zero and a maximum of itself (`:73`, `:74`).
  #[test]
  fn a_length_target_is_padded_by_a_tenth_of_a_millimetre() {
    let target = LengthTarget::around(50_000_000);

    assert_eq!(target.min, 49_900_000);
    assert_eq!(target.opt, 50_000_000);
    assert_eq!(target.max, 50_100_000);

    let unconstrained = LengthTarget::unconstrained();

    assert_eq!(unconstrained.min, 0);
    assert_eq!(unconstrained.opt, LENGTH_UNCONSTRAINED);
    assert_eq!(unconstrained.max, LENGTH_UNCONSTRAINED);
  }

  /// The three inline branches every placer's move ends on,
  /// `pcbnew/router/pns_meander_placer.cpp:332` to `:337`.
  #[test]
  fn the_tuning_status_is_the_window_test() {
    let target = LengthTarget::around(10_000_000);

    assert_eq!(
      TuningStatus::for_length(10_000_000, &target),
      TuningStatus::Tuned
    );
    assert_eq!(
      TuningStatus::for_length(9_900_000, &target),
      TuningStatus::Tuned
    );
    assert_eq!(
      TuningStatus::for_length(9_899_999, &target),
      TuningStatus::TooShort
    );
    assert_eq!(
      TuningStatus::for_length(10_100_001, &target),
      TuningStatus::TooLong
    );
  }

  /// `MEANDER_SIDE` is `-1`, `0`, `1` and the flip is a negation
  /// (`pcbnew/router/pns_meander.cpp:286`), so the default side never
  /// flips.
  #[test]
  fn flipping_the_initial_side_is_a_negation() {
    assert_eq!(MeanderSide::Left.flipped(), MeanderSide::Right);
    assert_eq!(MeanderSide::Right.flipped(), MeanderSide::Left);
    assert_eq!(MeanderSide::Default.flipped(), MeanderSide::Default);

    let mut settings = MeanderSettings::default();
    settings.flip_initial_side();
    assert_eq!(settings.initial_side(), MeanderSide::Right);
    settings.flip_initial_side();
    assert_eq!(settings.initial_side(), MeanderSide::Left);
  }

  // -----------------------------------------------------------------
  // The three constants
  // -----------------------------------------------------------------

  /// The three transcendental constants, bit for bit against the
  /// expressions KiCad writes. `DEG2RAD` is `deg * M_PI / 180.0`
  /// (`libs/kimath/include/trigo.h:172`), and the association matters:
  /// `f64::to_radians` multiplies by `PI / 180` instead and lands one
  /// unit in the last place away.
  #[test]
  fn the_chamfer_constants_are_the_ones_kicad_computes() {
    let tan_22_5 = (22.5 * PI / 180.0).tan();

    assert_eq!(TAN_22_5.to_bits(), tan_22_5.to_bits());
    assert_eq!(ONE_MINUS_TAN_22_5.to_bits(), (1.0 - tan_22_5).to_bits());
    assert_eq!(
      TAN_ONE_MINUS_TAN_22_5.to_bits(),
      (1.0 - tan_22_5).tan().to_bits()
    );

    // The closed forms note 08 section 8.2 offers are each one unit in
    // the last place away, which is why the literals are written out.
    assert_ne!(TAN_22_5.to_bits(), (SQRT_2 - 1.0).to_bits());
    assert_ne!(ONE_MINUS_TAN_22_5.to_bits(), (2.0 - SQRT_2).to_bits());
  }

  // -----------------------------------------------------------------
  // The turtle
  // -----------------------------------------------------------------

  /// The turtle's direction is a component permutation with signs of the
  /// vector it started on, for any sequence of turns, because
  /// `RotatePoint` takes an exact branch for both angles
  /// (`libs/kimath/src/trigo.cpp:305`, `:313`). That is the whole reason
  /// this module needs no floating point vector type.
  #[test]
  fn the_turtle_direction_stays_exact_through_any_sequence_of_turns() {
    let start = Vec2::new(1_234_567, -765_432);

    for sequence in 0_u32..64 {
      for length in 0..6 {
        let mut turtle = Turtle::start(Vec2::new(0, 0), start, false, 0, 0);
        let mut net: i32 = 0;

        for index in 0..length {
          if sequence & (1 << index) == 0 {
            turtle.turn(QuarterTurn::Plus90);
            net += 1;
          } else {
            turtle.turn(QuarterTurn::Minus90);
            net -= 1;
          }
        }

        let expected = match net.rem_euclid(4) {
          0 => start,
          1 => Vec2::new(start.y, -start.x),
          2 => Vec2::new(-start.x, -start.y),
          _ => Vec2::new(-start.y, start.x),
        };

        assert_eq!(
          turtle.direction, expected,
          "sequence {sequence:#08b} of {length} turns"
        );
      }
    }
  }

  // -----------------------------------------------------------------
  // The derived dimensions
  // -----------------------------------------------------------------

  /// Erratum E3: `MinAmplitude`'s chamfer correction is
  /// `tan( 1 - tan( 22.5 deg ) )` (`pcbnew/router/pns_meander.cpp:421`)
  /// where the same expression written correctly eighteen lines down is
  /// `1 - tan( 22.5 deg )` (`:439`). For a 200000 nm track that is 132694
  /// instead of 117157, 13.3 percent too large. It is masked whenever the
  /// setting dominates, which is the default case.
  #[test]
  fn min_amplitude_keeps_kicads_chamfer_typo() {
    let masked = settings_at(1_000_000, 600_000);
    let context =
      MeanderContext::accepting_every_fit(&masked, NOTE_WIDTH, NOTE_CLEARANCE);
    let shape = MeanderShape::new(NOTE_WIDTH, false);

    // The default minimum amplitude of 200000 dominates the correction.
    assert_eq!(shape.min_amplitude(&context), 200_000);

    let lowered = MeanderSettings::new(MeanderSettingsRequest {
      min_amplitude: 100_000,
      ..MeanderSettingsRequest::default()
    })
    .expect("a lower minimum amplitude is valid");
    let context =
      MeanderContext::accepting_every_fit(&lowered, NOTE_WIDTH, NOTE_CLEARANCE);

    // KiCad's typo, truncated toward zero by the assignment to `int`.
    assert_eq!(shape.min_amplitude(&context), 132_694);

    // What the correctly written factor would have given.
    assert_eq!((f64::from(NOTE_WIDTH) * ONE_MINUS_TAN_22_5) as i32, 117_157);

    // A dual meander adds the baseline offset on top (`:422`).
    let mut dual = MeanderShape::new(NOTE_WIDTH, true);
    dual.set_baseline_offset(300_000);
    assert_eq!(dual.min_amplitude(&context), 432_694);
  }

  /// `cornerRadius()`, `pcbnew/router/pns_meander.cpp:429`, hand
  /// evaluated for the note's worked case and for both clamps.
  #[test]
  fn the_corner_radius_is_the_clamped_percentage_of_the_half_period() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );

    // The anchor case: optimum 600000 * 80 / 200 inside a window of
    // 58578 to min( 500000, 300000 ).
    let anchor = fit_along_x(&context, MeanderType::Single);
    assert_eq!(anchor.corner_radius(&context), 240_000);

    // A small amplitude is clamped by ( amplitude + |offset| ) / 2.
    let small = settings_at(300_000, 600_000);
    let context =
      MeanderContext::accepting_every_fit(&small, NOTE_WIDTH, NOTE_CLEARANCE);
    let shape = fit_along_x(&context, MeanderType::Single);
    assert_eq!(shape.amplitude(), 300_000);
    assert_eq!(shape.corner_radius(&context), 150_000);

    // A narrow spacing is clamped by spacing / 2, and the spacing itself
    // is floored by width plus clearance (`:460`).
    let narrow = settings_at(1_000_000, 200_000);
    let context =
      MeanderContext::accepting_every_fit(&narrow, NOTE_WIDTH, NOTE_CLEARANCE);
    let shape = fit_along_x(&context, MeanderType::Single);
    assert_eq!(shape.spacing(&context), 300_000);
    assert_eq!(shape.corner_radius(&context), 120_000);
  }

  // -----------------------------------------------------------------
  // The anchor case
  // -----------------------------------------------------------------

  /// The concrete case of note 08 section 2.6, hand computed there and
  /// transcribed here as literals: width 200000, clearance 100000,
  /// spacing 600000, corner radius 80 percent, amplitude 1000000,
  /// chamfered corners.
  ///
  /// Every number below comes from the note, not from this port. It is
  /// the anchor for every other geometry assertion in the module, and
  /// KiCad has no test of its own to mirror (erratum E23).
  #[test]
  fn the_anchor_case_of_the_note_is_reproduced_exactly() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let shape = fit_along_x(&context, MeanderType::Single);

    assert_eq!(shape.spacing(&context), 600_000);
    assert_eq!(shape.amplitude(), 1_000_000);
    assert_eq!(shape.corner_radius(&context), 240_000);
    assert_eq!(shape.mean_corner_radius(), 240_000);

    assert_eq!(
      shape.cline(0).points(),
      &[
        Vec2::new(0, 0),
        Vec2::new(240_000, 240_000),
        Vec2::new(240_000, 760_000),
        Vec2::new(480_000, 1_000_000),
        Vec2::new(600_000, 1_000_000),
        Vec2::new(840_000, 760_000),
        Vec2::new(840_000, 240_000),
        Vec2::new(1_080_000, 0),
        Vec2::new(1_200_000, 0),
      ]
    );

    assert_eq!(chamfer_chord(240_000), 339_411);
    assert_eq!(shape.meander_length(), 2_637_644);
    assert_eq!(shape.baseline_length(), 1_200_000);
    assert_eq!(
      shape.meander_length() - i64::from(shape.baseline_length()),
      1_437_644
    );
  }

  // -----------------------------------------------------------------
  // The five shapes, against the closed forms
  // -----------------------------------------------------------------

  /// The amplitude and spacing combinations the closed form tests run on,
  /// with the corner radius each produces. Every corner radius here is
  /// hand evaluated from `cornerRadius()`'s clamp arithmetic: the
  /// optimum is `spacing * 80 / 200` and the window is 58578 to
  /// `min( amplitude / 2, spacing / 2 )`.
  const CLOSED_FORM_CASES: [(i32, i32, i32); 5] = [
    (1_000_000, 600_000, 240_000),
    (800_000, 600_000, 240_000),
    (600_000, 600_000, 240_000),
    (500_000, 600_000, 240_000),
    (1_000_000, 800_000, 320_000),
  ];

  /// `MT_SINGLE` against note 08 section 2.6: nine points, a current
  /// length of `4d + 2(A - 2c) + 2(s - 2c)` and a baseline of `2s`.
  #[test]
  fn mt_single_matches_the_closed_form() {
    for (amplitude, spacing, corner) in CLOSED_FORM_CASES {
      let settings = settings_at(amplitude, spacing);
      let context = MeanderContext::accepting_every_fit(
        &settings,
        NOTE_WIDTH,
        NOTE_CLEARANCE,
      );
      let shape = fit_along_x(&context, MeanderType::Single);
      let (a, s, c) = (amplitude, spacing, corner);

      assert_eq!(shape.mean_corner_radius(), c, "amplitude {a} spacing {s}");
      assert_eq!(
        shape.cline(0).points(),
        &[
          Vec2::new(0, 0),
          Vec2::new(c, c),
          Vec2::new(c, a - c),
          Vec2::new(2 * c, a),
          Vec2::new(s, a),
          Vec2::new(s + c, a - c),
          Vec2::new(s + c, c),
          Vec2::new(s + 2 * c, 0),
          Vec2::new(2 * s, 0),
        ],
        "amplitude {a} spacing {s}"
      );

      let d = chamfer_chord(c);
      assert_eq!(
        shape.meander_length(),
        4 * d + 2 * i64::from(a - 2 * c) + 2 * i64::from(s - 2 * c),
        "amplitude {a} spacing {s}"
      );
      assert_eq!(shape.baseline_length(), 2 * s, "amplitude {a} spacing {s}");
    }
  }

  /// `MT_START` against note 08 section 2.6: eight points, a current
  /// length of `3d + 2(A - 2c) + (s - 2c) + c` and a baseline of `s + c`.
  #[test]
  fn mt_start_matches_the_closed_form() {
    for (amplitude, spacing, corner) in CLOSED_FORM_CASES {
      let settings = settings_at(amplitude, spacing);
      let context = MeanderContext::accepting_every_fit(
        &settings,
        NOTE_WIDTH,
        NOTE_CLEARANCE,
      );
      let shape = fit_along_x(&context, MeanderType::Start);
      let (a, s, c) = (amplitude, spacing, corner);

      assert_eq!(
        shape.cline(0).points(),
        &[
          Vec2::new(0, 0),
          Vec2::new(c, c),
          Vec2::new(c, a - c),
          Vec2::new(2 * c, a),
          Vec2::new(s, a),
          Vec2::new(s + c, a - c),
          Vec2::new(s + c, c),
          Vec2::new(s + c, 0),
        ],
        "amplitude {a} spacing {s}"
      );

      let d = chamfer_chord(c);
      assert_eq!(
        shape.meander_length(),
        3 * d + 2 * i64::from(a - 2 * c) + i64::from(s - 2 * c) + i64::from(c),
        "amplitude {a} spacing {s}"
      );
      assert_eq!(shape.baseline_length(), s + c, "amplitude {a} spacing {s}");
    }
  }

  /// `MT_TURN` against note 08 section 2.6: six points, a current length
  /// of `2d + 2(A - c) + (s - 2c)` and a baseline of `s`.
  #[test]
  fn mt_turn_matches_the_closed_form() {
    for (amplitude, spacing, corner) in CLOSED_FORM_CASES {
      let settings = settings_at(amplitude, spacing);
      let context = MeanderContext::accepting_every_fit(
        &settings,
        NOTE_WIDTH,
        NOTE_CLEARANCE,
      );
      let shape = fit_along_x(&context, MeanderType::Turn);
      let (a, s, c) = (amplitude, spacing, corner);

      assert_eq!(
        shape.cline(0).points(),
        &[
          Vec2::new(0, 0),
          Vec2::new(0, a - c),
          Vec2::new(c, a),
          Vec2::new(s - c, a),
          Vec2::new(s, a - c),
          Vec2::new(s, 0),
        ],
        "amplitude {a} spacing {s}"
      );

      let d = chamfer_chord(c);
      assert_eq!(
        shape.meander_length(),
        2 * d + 2 * i64::from(a - c) + i64::from(s - 2 * c),
        "amplitude {a} spacing {s}"
      );
      assert_eq!(shape.baseline_length(), s, "amplitude {a} spacing {s}");
    }
  }

  /// `MT_FINISH` against note 08 section 2.6: nine points, a current
  /// length of `3d + c + 2(A - 2c) + 2(s - 2c)` and a baseline of
  /// `2s - c`.
  #[test]
  fn mt_finish_matches_the_closed_form() {
    for (amplitude, spacing, corner) in CLOSED_FORM_CASES {
      let settings = settings_at(amplitude, spacing);
      let context = MeanderContext::accepting_every_fit(
        &settings,
        NOTE_WIDTH,
        NOTE_CLEARANCE,
      );
      let shape = fit_along_x(&context, MeanderType::Finish);
      let (a, s, c) = (amplitude, spacing, corner);

      assert_eq!(
        shape.cline(0).points(),
        &[
          Vec2::new(0, 0),
          Vec2::new(0, c),
          Vec2::new(0, a - c),
          Vec2::new(c, a),
          Vec2::new(s - c, a),
          Vec2::new(s, a - c),
          Vec2::new(s, c),
          Vec2::new(s + c, 0),
          Vec2::new(2 * s - c, 0),
        ],
        "amplitude {a} spacing {s}"
      );

      let d = chamfer_chord(c);
      assert_eq!(
        shape.meander_length(),
        3 * d
          + i64::from(c)
          + 2 * i64::from(a - 2 * c)
          + 2 * i64::from(s - 2 * c),
        "amplitude {a} spacing {s}"
      );
      assert_eq!(
        shape.baseline_length(),
        2 * s - c,
        "amplitude {a} spacing {s}"
      );
    }
  }

  /// `MT_EMPTY` is the clipped base segment itself, so its elongation is
  /// zero and its footprint is the one it had (`pns_meander.cpp:850`).
  #[test]
  fn make_empty_leaves_a_straight_bypass_of_the_same_baseline() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let mut shape = fit_along_x(&context, MeanderType::Single);
    let baseline = shape.baseline_length();

    shape.make_empty(&context);

    assert_eq!(shape.meander_type(), MeanderType::Empty);
    assert_eq!(shape.amplitude(), 0);
    assert_eq!(shape.baseline_length(), baseline);
    assert_eq!(
      shape.cline(0).points(),
      &[Vec2::new(0, 0), Vec2::new(1_200_000, 0)]
    );
    assert_eq!(
      shape.meander_length() - i64::from(shape.baseline_length()),
      0
    );
  }

  /// Erratum E15 and the deviation `doc/log/2026-09-10.md` records: KiCad
  /// truncates every generated coordinate toward zero, so a meander to
  /// the left of the origin is not the mirror image of the same meander to
  /// its right. This port carries the turtle in integers and rounds
  /// through `Vec2::resize`, so on an axis aligned base segment the two
  /// **are** exact mirror images.
  #[test]
  fn the_same_meander_left_and_right_of_the_origin_is_a_mirror_image() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let placed = MeanderedLine::new(NOTE_WIDTH, false);

    let mut right = MeanderShape::new(NOTE_WIDTH, false);
    assert!(right.fit(
      &context,
      &placed,
      MeanderType::Single,
      Seg::new(Vec2::new(0, 0), Vec2::new(10_000_000, 0)),
      Vec2::new(0, 0),
      false
    ));

    let mut left = MeanderShape::new(NOTE_WIDTH, false);
    assert!(left.fit(
      &context,
      &placed,
      MeanderType::Single,
      Seg::new(Vec2::new(0, 0), Vec2::new(-10_000_000, 0)),
      Vec2::new(0, 0),
      false
    ));

    let mirrored: Vec<Vec2> = right
      .cline(0)
      .points()
      .iter()
      .map(|point| Vec2::new(-point.x, -point.y))
      .collect();

    assert_eq!(left.cline(0).points(), mirrored.as_slice());
    assert_eq!(left.meander_length(), right.meander_length());
  }

  // -----------------------------------------------------------------
  // The fitting loop
  // -----------------------------------------------------------------

  /// A 10 mm base segment with the note's settings and the accepting fit
  /// check: the loop opens a turning run, keeps turning while three
  /// periods are left, and closes it.
  #[test]
  fn the_fitting_loop_fills_a_ten_millimetre_base_segment() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let base = Seg::new(Vec2::new(0, 0), Vec2::new(10_000_000, 0));
    let mut line = MeanderedLine::new(NOTE_WIDTH, false);

    let result = line.meander_segment(&context, base, false);

    assert!(!result.initial_side_flipped);

    let types: Vec<MeanderType> = line
      .meanders()
      .iter()
      .map(MeanderShape::meander_type)
      .collect();

    // `MeanderSegment` brackets the run with the base segment's own two
    // endpoints (`pns_meander.cpp:263`, `:407`).
    assert_eq!(types.first(), Some(&MeanderType::Corner));
    assert_eq!(types.last(), Some(&MeanderType::Corner));

    let run: Vec<MeanderType> = types
      .iter()
      .copied()
      .filter(|meander_type| *meander_type != MeanderType::Corner)
      .collect();

    assert_eq!(run.first(), Some(&MeanderType::Start));
    assert_eq!(run.last(), Some(&MeanderType::Finish));
    assert!(
      run[1..run.len() - 1]
        .iter()
        .all(|meander_type| *meander_type == MeanderType::Turn),
      "a turning run is start, turns, finish: {run:?}"
    );

    // The sides alternate through the run, and every meander sits inside
    // the base segment.
    let run_shapes: Vec<&MeanderShape> = line
      .meanders()
      .iter()
      .filter(|meander| meander.meander_type() != MeanderType::Corner)
      .collect();

    for pair in run_shapes.windows(2) {
      assert_ne!(pair[0].side(), pair[1].side());
    }

    for meander in &run_shapes {
      assert!(base.contains_point(meander.base_segment().a));
      assert!(base.contains_point(meander.base_segment().b));
    }

    // Every meander comes out of `Fit` at the tallest amplitude that
    // fits, which with an accepting check is the setting itself.
    assert!(
      run_shapes
        .iter()
        .all(|meander| meander.amplitude() == 1_000_000)
    );

    let consumed: i64 = line
      .meanders()
      .iter()
      .map(|meander| i64::from(meander.baseline_length()))
      .sum();

    assert!(consumed <= i64::from(base.length()));
    assert!(total_elongation(&line) > 0);
  }

  /// A single sided run never turns: the first two arms of the `if` chain
  /// are unreachable and the third lays down `MT_SINGLE` shapes
  /// (`pns_meander.cpp:320`, `:364`, `:374`).
  #[test]
  fn a_single_sided_run_is_all_singles_on_one_side() {
    let settings = MeanderSettings::new(MeanderSettingsRequest {
      single_sided: true,
      ..MeanderSettingsRequest::default()
    })
    .expect("single sided is valid");
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let base = Seg::new(Vec2::new(0, 0), Vec2::new(10_000_000, 0));
    let mut line = MeanderedLine::new(NOTE_WIDTH, false);

    let result = line.meander_segment(&context, base, false);

    assert!(!result.initial_side_flipped);

    let run: Vec<&MeanderShape> = line
      .meanders()
      .iter()
      .filter(|meander| meander.meander_type() != MeanderType::Corner)
      .collect();

    assert!(!run.is_empty());
    assert!(
      run
        .iter()
        .all(|meander| meander.meander_type() == MeanderType::Single)
    );
    assert!(run.iter().all(|meander| !meander.side()));
  }

  /// Erratum E6: the skip advance is `spacing() + m_step`, because the
  /// corner radius it subtracts is always zero
  /// (`pns_meander.cpp:394`, the fresh shape's amplitude is zero and
  /// `cornerRadius()` answers zero for that at `:431`).
  #[test]
  fn erratum_e6_the_skip_advance_has_no_corner_radius_term() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let skip = MeanderShape::new(NOTE_WIDTH, false);

    assert_eq!(skip.amplitude(), 0);
    assert_eq!(skip.corner_radius(&context), 0);
    assert_eq!(
      skip.spacing(&context) + settings.step(),
      600_000 + 50_000,
      "the advance is the period plus one step"
    );
  }

  /// Two identical runs produce identical geometry, point for point. The
  /// three order sensitive places note 08 section 11.6 lists are the side
  /// flip, the downward amplitude scan and the left before right
  /// recursion of the amplitude search; none of them reads anything this
  /// module does not carry explicitly.
  #[test]
  fn two_identical_runs_produce_identical_chains() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let base = Seg::new(
      Vec2::new(-3_000_000, 1_000_000),
      Vec2::new(7_000_000, 1_000_000),
    );

    let run = || {
      let mut line = MeanderedLine::new(NOTE_WIDTH, false);
      let flipped = line.meander_segment(&context, base, false);
      let half = total_elongation(&line) / 2;
      tune_line_length(&context, &mut line, half);

      let points: Vec<Vec<Vec2>> = line
        .meanders()
        .iter()
        .map(|meander| meander.cline(0).points().to_vec())
        .collect();

      (flipped, points)
    };

    assert_eq!(run(), run());
  }

  /// The amplitude scan is a linear walk downwards in `m_step`
  /// decrements that stops at the first amplitude the fit check accepts
  /// (`pns_meander.cpp:789`). The order is the answer, not a search
  /// strategy, so a check with a ceiling lands on the first multiple of
  /// the step at or below it.
  #[test]
  fn the_amplitude_scan_walks_down_in_steps_until_the_check_accepts() {
    let settings = settings_at(1_000_000, 600_000);

    for (ceiling, expected) in [(700_000, 700_000), (690_000, 650_000)] {
      let check =
        |shape: &MeanderShape, _: &MeanderedLine| shape.amplitude() <= ceiling;
      let context =
        MeanderContext::new(&settings, NOTE_WIDTH, NOTE_CLEARANCE, &check);

      assert_eq!(
        fit_along_x(&context, MeanderType::Single).amplitude(),
        expected
      );
    }

    // A check that accepts nothing fits nothing, which is the base
    // implementation's behaviour (`pns_meander_placer_base.h:120`).
    let refusing = |_: &MeanderShape, _: &MeanderedLine| false;
    let context =
      MeanderContext::new(&settings, NOTE_WIDTH, NOTE_CLEARANCE, &refusing);
    let placed = MeanderedLine::new(NOTE_WIDTH, false);
    let mut shape = MeanderShape::new(NOTE_WIDTH, false);

    assert!(!shape.fit(
      &context,
      &placed,
      MeanderType::Single,
      Seg::new(Vec2::new(0, 0), Vec2::new(10_000_000, 0)),
      Vec2::new(0, 0),
      false
    ));
  }

  /// `CheckSelfIntersections`, `pns_meander.cpp:695`. The parallel skip at
  /// `:707` is what makes it cheap: two meanders on the same base segment
  /// overlap completely and are still accepted, so only meanders from
  /// other base segments of the same line are ever measured.
  #[test]
  fn check_self_intersections_skips_parallel_and_empty_meanders() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let placed = MeanderedLine::new(NOTE_WIDTH, false);

    let along_x = fit_along_x(&context, MeanderType::Single);

    let mut along_y = MeanderShape::new(NOTE_WIDTH, false);
    assert!(along_y.fit(
      &context,
      &placed,
      MeanderType::Single,
      Seg::new(Vec2::new(600_000, 0), Vec2::new(600_000, 10_000_000)),
      Vec2::new(600_000, 0),
      false
    ));

    let mut line = MeanderedLine::new(NOTE_WIDTH, false);
    line.add_meander(along_x.clone());

    // Parallel: the same base segment, so skipped whatever the
    // clearance.
    assert!(line.check_self_intersections(&along_x, 2_000_000));

    // Crossing: measured, and at that clearance it collides.
    assert!(!line.check_self_intersections(&along_y, 2_000_000));

    // A corner and an emptied meander are skipped outright (`:701`).
    let mut skipped = MeanderedLine::new(NOTE_WIDTH, false);
    skipped.add_corner(Vec2::new(600_000, 0), Vec2::new(0, 0));

    let mut emptied = along_x;
    emptied.make_empty(&context);
    skipped.add_meander(emptied);

    assert!(skipped.check_self_intersections(&along_y, 2_000_000));
  }

  // -----------------------------------------------------------------
  // The length arithmetic
  // -----------------------------------------------------------------

  /// `tuneLineLength` lands on the requested elongation, on a straight
  /// 10 mm chain meandered with the accepting fit check.
  #[test]
  fn tune_line_length_reaches_a_target_within_tolerance() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let base = Seg::new(Vec2::new(0, 0), Vec2::new(10_000_000, 0));

    let mut line = MeanderedLine::new(NOTE_WIDTH, false);
    line.meander_segment(&context, base, false);

    let available = total_elongation(&line);
    assert!(available > 0);

    for numerator in 1..=9 {
      let mut tuned = line.clone();
      let target = available * numerator / 10;

      tune_line_length(&context, &mut tuned, target);

      let achieved = total_elongation(&tuned);
      let error = (achieved - target).abs();

      // KiCad's own acceptance window, which is what "tuned" means.
      assert!(
        error <= DEFAULT_LENGTH_TOLERANCE,
        "asked for {target}, got {achieved}"
      );

      // And it is far tighter than that in practice: the worst of these
      // nine misses by eleven nanometres, because only the last survivor
      // absorbs what the equal share left over.
      assert!(error <= 1_000, "asked for {target}, got {achieved}");
    }
  }

  /// The two ends of the range. Asking for more than the meanders can
  /// give leaves every one of them at full amplitude, which is the "still
  /// too short" case that returns early at
  /// `pns_meander_placer_base.cpp:271`; asking for nothing empties the
  /// lot.
  #[test]
  fn tune_line_length_saturates_at_both_ends() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let base = Seg::new(Vec2::new(0, 0), Vec2::new(10_000_000, 0));

    let mut line = MeanderedLine::new(NOTE_WIDTH, false);
    line.meander_segment(&context, base, false);

    let available = total_elongation(&line);

    let mut greedy = line.clone();
    tune_line_length(&context, &mut greedy, available * 10);

    assert_eq!(total_elongation(&greedy), available);
    assert!(
      greedy
        .meanders()
        .iter()
        .filter(|meander| meander.meander_type() != MeanderType::Corner)
        .all(|meander| meander.amplitude() == 1_000_000)
    );

    let mut none = line.clone();
    tune_line_length(&context, &mut none, 0);

    assert_eq!(total_elongation(&none), 0);
    assert!(none.meanders().iter().all(|meander| matches!(
      meander.meander_type(),
      MeanderType::Corner | MeanderType::Empty
    )));
  }

  /// Erratum E8: the fast path of `findAmplitudeForLength` resizes the
  /// working copy to the **minimum** amplitude
  /// (`pns_meander_placer_base.cpp:192`) and then returns the initial
  /// guess (`:195`), so it answers with an amplitude it never measured.
  ///
  /// The numbers are hand computed from note 08 section 2.6. The anchor
  /// meander is 2637644 long at amplitude 1000000 over a baseline of
  /// 1200000; at the minimum amplitude of 200000, with that baseline
  /// preserved, its corner radius clamps to 100000 and the seven point
  /// chain is `4 * KiROUND( 100000 * sqrt( 2 ) ) + 800000 = 1365684` long.
  /// The initial guess is therefore
  /// `1000000 - ( 2637644 - 1365684 ) / 2 = 364020`.
  #[test]
  fn erratum_e8_the_fast_path_returns_an_amplitude_it_did_not_measure() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let meander = fit_along_x(&context, MeanderType::Single);

    let target = 1_365_684;

    // The amplitude the fast path actually measures.
    let mut measured = meander.clone();
    measured.set_target_baseline_length(meander.baseline_length());
    measured.resize(&context, 200_000);
    assert_eq!(measured.meander_length(), target);

    let answer =
      find_amplitude_for_length(&context, &meander, target, 200_000, 1_000_000);

    assert_eq!(answer, 364_020);
    assert_ne!(answer, 200_000);

    // And the amplitude it answered with misses the target by far more
    // than the twenty nanometre tolerance.
    let mut returned = meander.clone();
    returned.set_target_baseline_length(meander.baseline_length());
    returned.resize(&context, answer);

    assert!(
      (returned.meander_length() - target).abs() > LENGTH_TARGET_TOLERANCE
    );
  }

  /// Erratum E16: the base case of `findAmplitudeBinarySearch` returns
  /// `maxAmp` with no length test at all
  /// (`pns_meander_placer_base.cpp:139`), so a collapsed interval reports
  /// success with an amplitude that achieves nothing.
  #[test]
  fn erratum_e16_a_collapsed_interval_answers_without_measuring() {
    let settings = settings_at(1_000_000, 600_000);
    let context = MeanderContext::accepting_every_fit(
      &settings,
      NOTE_WIDTH,
      NOTE_CLEARANCE,
    );
    let meander = fit_along_x(&context, MeanderType::Single);
    let mut copy = meander.clone();

    // A length no amplitude of this meander can reach.
    let unreachable = 1;
    let answer = find_amplitude_binary_search(
      &context,
      &mut copy,
      unreachable,
      500_000,
      500_000,
    );

    assert_eq!(answer, 500_000);

    let mut probe = meander.clone();
    probe.resize(&context, answer);
    assert!(probe.meander_length() > 1_000_000);

    // With the interval one wide the bracket tests at `:148` and `:151`
    // catch it, and the answer is the "not found" zero the caller papers
    // over with a `max`.
    let mut copy = meander.clone();
    assert_eq!(
      find_amplitude_binary_search(
        &context,
        &mut copy,
        unreachable,
        500_000,
        500_001,
      ),
      0
    );
  }

  // -----------------------------------------------------------------
  // The placer base's small members
  // -----------------------------------------------------------------

  /// `AmplitudeStep` and `SpacingStep`,
  /// `pcbnew/router/pns_meander_placer_base.cpp:96` and `:105`, each
  /// floored by its own quantity.
  #[test]
  fn the_two_live_adjustments_are_floored() {
    let mut settings = MeanderSettings::default();

    amplitude_step(&mut settings, 1);
    assert_eq!(settings.max_amplitude(), 1_050_000);

    for _ in 0..100 {
      amplitude_step(&mut settings, -1);
    }
    assert_eq!(settings.max_amplitude(), settings.min_amplitude());

    spacing_step(&mut settings, 1, NOTE_WIDTH, NOTE_CLEARANCE);
    assert_eq!(settings.spacing(), 650_000);

    for _ in 0..100 {
      spacing_step(&mut settings, -1, NOTE_WIDTH, NOTE_CLEARANCE);
    }
    assert_eq!(settings.spacing(), NOTE_WIDTH + NOTE_CLEARANCE);
  }

  /// `Clearance()`, `pcbnew/router/pns_meander_placer_base.cpp:114`, with
  /// its surprising `wxCHECK_MSG` fallback: a host with no minimum
  /// clearance rule gets the **track width** back, which then floors the
  /// meander period at twice the width.
  #[test]
  fn the_clearance_falls_back_to_the_track_width() {
    let mut item = Item::new(
      1,
      ItemBody::Segment(Segment::new(
        Seg::new(Vec2::new(0, 0), Vec2::new(1_000_000, 0)),
        NOTE_WIDTH,
      )),
    );
    item.set_layers_and_flash_all(LayerRange::single(0));

    // A resolver that answers no constraint at all, which is
    // `RuleResolver::constraint`'s default body.
    let silent = FixedClearance::uniform(NOTE_CLEARANCE);
    assert_eq!(
      clearance(&silent, ItemRef::unstored(&item), 0, NOTE_WIDTH),
      NOTE_WIDTH
    );

    // A resolver that does answer.
    struct Answering(i32);

    impl RuleResolver for Answering {
      fn clearance(
        &self,
        _a: ItemRef<'_>,
        _b: Option<ItemRef<'_>>,
        _use_epsilon: bool,
      ) -> Option<i32> {
        Some(self.0)
      }

      fn constraint(
        &self,
        constraint_type: ConstraintType,
        _a: ItemRef<'_>,
        _b: Option<ItemRef<'_>>,
        _layer: i32,
      ) -> Option<Constraint> {
        Some(Constraint {
          constraint_type,
          min: Some(self.0),
          opt: None,
          max: None,
          allowed: true,
        })
      }

      fn is_keepout(
        &self,
        _obstacle: ItemRef<'_>,
        _item: ItemRef<'_>,
      ) -> Keepout {
        Keepout::None
      }

      fn is_drilled_hole(&self, _item: ItemRef<'_>) -> bool {
        false
      }

      fn is_non_plated_slot(&self, _item: ItemRef<'_>) -> bool {
        false
      }

      fn net_code(&self, net: crate::item::NetId) -> i32 {
        net.0 as i32
      }

      fn orphaned_net(&self) -> crate::item::NetId {
        crate::item::NetId(0)
      }
    }

    assert_eq!(
      clearance(&Answering(123_456), ItemRef::unstored(&item), 0, NOTE_WIDTH),
      123_456
    );
  }
}
