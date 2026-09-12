// SPDX-License-Identifier: GPL-3.0-or-later

//! The routing session facade a host drives (KiCad's `PNS::ROUTER`).
//!
//! Port of `PNS::ROUTER` (`pcbnew/router/pns_router.h:153`) narrowed to
//! routing and dragging. The facade owns the world, the placer, the rule
//! oracle and the settings, and turns a stream of user events into a
//! [`PreviewFrame`] after every event and a [`CommitDiff`] at the end.
//! That is `DESIGN.md` section 7 and note 05 section 7.2: both directions
//! of the host contract become returned values instead of callbacks,
//! because both were already whole frame replacements pretending to be
//! incremental (`EraseView` clears the view at the top of every move,
//! `pcbnew/router/pns_router.cpp:791`).
//!
//! # What is not here
//!
//! - **The singleton.** KiCad's file static `theRouter`
//!   (`pcbnew/router/pns_router.cpp:58`) and `ROUTER::GetInstance()` have
//!   no counterpart; the ambient state travels as
//!   [`crate::algo_base::AlgoContext`] (`DESIGN.md` section 8).
//! - **The three tuning modes.** The placer set is closed by the mode
//!   (note 03 section 9.2), so the missing modes are missing
//!   [`crate::placer::Placer`] variants and not missing trait
//!   implementations. The two that are here are the single track placer
//!   and the differential pair placer, through [`Router::start_routing`]
//!   and [`Router::start_routing_diff_pair`]. All three of KiCad's drag
//!   algorithms are here too, through [`Router::start_dragging`],
//!   [`RouterState::DragSegment`] and [`RouterState::DragComponent`]:
//!   [`crate::dragger`], [`crate::multi_dragger`] and
//!   [`crate::component_dragger`].
//! - **A router mode.** `ROUTER::SetMode` (`:1094`) is a field KiCad's
//!   host sets before `StartRouting`, and every one of its readers is
//!   either a start gate or a placer choice; both are entry points here,
//!   so there is nothing left for the field to decide. Which one is
//!   running is [`Router::current_nets`].
//! - **`SetIterLimit` and `GetIterLimit`**
//!   (`pcbnew/router/pns_router.h:224`), dead API read by nobody in
//!   KiCad's tree; the real budgets live in
//!   [`crate::settings::RoutingSettings`].
//! - **`BreakSegmentOrArc`** (`:1106`), which is not part of a routing
//!   session: it is a standalone board edit that builds a throwaway
//!   placer to reach `SplitAdjacentSegments`. A host reaches the same
//!   routine through
//!   [`crate::placer::line_placer::LinePlacer::split_adjacent_segments`].
//! - **`ClearViewDecorations`**, `EraseView` and the rest of the drawing
//!   half of `ROUTER_IFACE`. Every frame is complete, so there is nothing
//!   to erase.
//! - **`GetModifiedNets` and `UpdateNet`** (`:971`). KiCad pushes the
//!   touched nets to the host so it can rebuild the ratsnest; a host here
//!   reads the nets off [`CommitDiff`], which names every item it has to
//!   apply anyway.
//!
//! # Escape does not discard
//!
//! Neither KiCad nor Horizon EDA throws a route away when the user
//! presses escape: `ROUTER::CommitRouting()` commits and then stops
//! (`pcbnew/router/pns_router.cpp:958`). `DESIGN.md` section 7 asks for
//! both behaviours to be reachable, so [`Router::stop_routing`] is that
//! commit and [`Router::abort_routing`] is the discard, and the host
//! decides which key does what.

use crate::algo_base::AlgoContext;
use crate::collide::CollisionSearchOptions;
use crate::component_dragger::{ComponentDragger, MovedSolid};
use crate::debug::{DebugDecorator, NoDebug};
use crate::dragger::Dragger;
use crate::eventlog::{Recorder, SessionEvent, SessionRecording};
use crate::geometry::direction45::CornerMode;
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::Vec2;
use crate::item::{
  HostId, Item, ItemBody, ItemId, Kind, LayerRange, MarkerFlags, NetId,
  Provenance, ViaType,
};
use crate::line::Line;
use crate::meander::{LengthTarget, MeanderSettings, TuningStatus};
use crate::multi_dragger::MultiDragger;
use crate::node::{NodeId, World};
use crate::placer::diff_pair_placer::{DiffPairPlacer, PairError};
use crate::placer::dp_meander_placer::DpMeanderPlacer;
use crate::placer::line_placer::LinePlacer;
use crate::placer::meander_placer::{MeanderPlacer, TuningError};
use crate::placer::meander_skew_placer::MeanderSkewPlacer;
use crate::placer::{Placer, TuningMode};
use crate::rules::{ItemRef, RuleResolver};
use crate::settings::{RoutingSettings, Sizes};
use crate::snapshot::{HostIndex, WorldSnapshot};
use crate::topology;

/// The uid of the throwaway item a clearance query is asked about.
///
/// A line has no [`Item`] of its own, so [`Line::rule_item`] builds one
/// per query. It never enters the arena and never takes part in a
/// `(distance, uid)` tie break, so it does not consume a real uid; the
/// world's counter stays a function of what was actually stored, which is
/// what makes a replay reproducible (`DESIGN.md` section 8).
const PROBE_UID: u64 = u64::MAX;

// ---------------------------------------------------------------------
// The state machine
// ---------------------------------------------------------------------

/// Which algorithm the facade is running.
///
/// Port of `ROUTER::RouterState`, `pcbnew/router/pns_router.h:157`.
/// `RoutingInProgress()` is `m_state != IDLE`
/// (`pcbnew/router/pns_router.cpp:120`); there is no separate committing
/// state, the transition back to [`RouterState::Idle`] happens inside
/// [`Router::stop_routing`].
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum RouterState {
  /// Nothing is being routed. `IDLE`.
  #[default]
  Idle,
  /// A single track placement is running. `ROUTE_TRACK`.
  RouteTrack,

  /// A single track is being length tuned.
  ///
  /// KiCad has no such state: `ROUTER::StartRouting` puts every mode into
  /// `ROUTE_TRACK` and the three tuning modes are told apart by
  /// `ROUTER::Mode()` alone (`pcbnew/router/pns_router.cpp:451` to
  /// `:483`), so its `Move`, `FixRoute` and `GetUpdatedItems` all take
  /// the placer branch whichever placer it is. The state is separate here
  /// because the facade has separate entry points and because the
  /// commands have to be told apart in both directions:
  /// [`Router::amplitude_step`] and [`Router::spacing_step`] refuse a
  /// routing session, and [`Router::undo_last_segment`],
  /// [`Router::switch_layer`], [`Router::toggle_via_placement`],
  /// [`Router::flip_posture`], [`Router::finish`] and
  /// [`Router::continue_from_end`] refuse a tuning one. KiCad reaches the
  /// same place for the first four by way of a placer that overrides none
  /// of them (note 08 section 5.8); the last two it does not gate at all,
  /// and `Finish` on a tuning session would drive it at a ratsnest anchor
  /// that means nothing to a tuner.
  ///
  /// [`Router::move_to`], [`Router::fix_route`],
  /// [`Router::pending_update`], [`Router::stop_routing`] and
  /// [`Router::abort_routing`] all take the placer branch, exactly as
  /// they do for [`RouterState::RouteTrack`].
  TuneSingle,

  /// Both lanes of a differential pair are being length tuned.
  ///
  /// Everything [`RouterState::TuneSingle`] says applies, with
  /// `PNS_MODE_TUNE_DIFF_PAIR` in place of `PNS_MODE_TUNE_SINGLE`.
  TuneDiffPair,

  /// One lane of a differential pair is being skew tuned.
  ///
  /// Everything [`RouterState::TuneSingle`] says applies, with
  /// `PNS_MODE_TUNE_DIFF_PAIR_SKEW` in place of `PNS_MODE_TUNE_SINGLE`.
  /// The readout is a **skew** and not a length; see
  /// [`TuningInfo::mode`].
  TuneSkew,
  /// An existing track, corner or via is being dragged. `DRAG_SEGMENT`.
  ///
  /// Entered by [`Router::start_dragging`]
  /// (`pcbnew/router/pns_router.cpp:185`, `:190`) for a set that reaches
  /// [`crate::dragger::Dragger`] or
  /// [`crate::multi_dragger::MultiDragger`].
  DragSegment,

  /// One or more footprints are being dragged by their pads.
  /// `DRAG_COMPONENT`.
  ///
  /// Entered by [`Router::start_dragging`]
  /// (`pcbnew/router/pns_router.cpp:179`) for a set of nothing but
  /// solids, which reaches [`crate::component_dragger::ComponentDragger`].
  ///
  /// KiCad's `ROUTER::GetUpdatedItems` has no branch for this state
  /// (`:839` handles `ROUTE_TRACK`, `:844` handles `DRAG_SEGMENT`,
  /// nothing handles `DRAG_COMPONENT`), so a component drag reports an
  /// empty delta to anything that asks. That is note 06 erratum E15 and
  /// it is the one thing [`Router::pending_update`] adds rather than
  /// transcribes.
  DragComponent,
}

impl RouterState {
  /// Whether one of the three length tuning modes is running.
  ///
  /// KiCad asks the same question with `ROUTER::Mode()`
  /// (`pcbnew/router/pns_router.h:171`), because all three modes leave it
  /// in `ROUTE_TRACK`; the state carries the answer here.
  pub const fn is_tuning(self) -> bool {
    matches!(
      self,
      RouterState::TuneSingle
        | RouterState::TuneDiffPair
        | RouterState::TuneSkew
    )
  }
}

/// Why a routing session refused to start.
///
/// Replaces `ROUTER::SetFailureReason` and `FailureReason()`
/// (`pcbnew/router/pns_router.h:245`), a translated string the host reads
/// back and shows in its status bar. An enum instead of a string, because
/// the crate has no user interface and no translation catalogue; a host
/// maps each variant to its own message.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum StartError {
  /// A session is already running. KiCad has no such check: its host
  /// never starts twice, and `ContinueFromEnd` commits before restarting
  /// (`pcbnew/router/pns_router.cpp:633`).
  AlreadyRouting,

  /// The host named a start object the snapshot does not describe.
  UnknownStartItem(HostId),

  /// Something under the start point may not be routed from.
  ///
  /// Port of the per type messages of `isStartingPointRoutable`
  /// (`pcbnew/router/pns_router.cpp:264`, `:277`, `:290`): a non plated
  /// hole, a rule area that disallows tracks, a text item. The engine
  /// cannot tell those apart, because it sees a solid with
  /// [`crate::snapshot::WorldItemFlags::routable`] cleared and not a
  /// board object, so the item is named instead of the reason.
  NotRoutable(ItemId),

  /// A track of the minimum width would already violate a rule here.
  ///
  /// Port of "The routing start point violates DRC."
  /// (`pcbnew/router/pns_router.cpp:336`). The probe is run twice, at the
  /// requested track width and then at the board minimum, because a
  /// collision that is only caused by the width should not stop the user
  /// from starting and fixing the width afterwards (`:322`).
  StartPointViolatesRules,

  /// The placer refused to start.
  ///
  /// Port of the `else` branch of `StartRouting`
  /// (`pcbnew/router/pns_router.cpp:485`).
  PlacerRefused,

  /// A drag was asked for with no object to drag.
  ///
  /// Port of `if( aStartItems.Empty() ) return false`
  /// (`pcbnew/router/pns_router.cpp:171`), the first thing
  /// `StartDragging` tests.
  NothingToDrag,

  /// A differential pair placement was asked for with no start object.
  ///
  /// Port of "Cannot start a differential pair in the middle of nowhere."
  /// (`pcbnew/router/pns_router.cpp:345`). A pair placement **requires**
  /// a start object, where a single track does not: the engine has no
  /// other way to learn which two nets are being routed.
  PairNeedsStartItem,

  /// The rule resolver does not know the start object as half of a pair.
  ///
  /// Port of "Unable to find complementary differential pair nets..."
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:526`), which for KiCad
  /// means the two net names do not differ in a `P`/`N` or `+`/`-`
  /// suffix. This crate has no net names and invents no convention: the
  /// host answers [`crate::rules::RuleResolver::dp_net_pair`], and a host
  /// with no pair concept answers [`None`] and reaches this.
  NotADiffPair,

  /// The start object has no free end to start a pair from.
  ///
  /// Port of "Can't find a suitable starting point.  If starting from an
  /// existing differential pair make sure you are at the end."
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:543`).
  NoDanglingAnchor,

  /// Nothing on the coupled net can be paired with the start object.
  ///
  /// Port of "Can't find a suitable starting point for coupled net"
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:598`). The candidate has to
  /// be the same kind of object, has to have a free end of its own, and,
  /// for a pad or a via, has to span the same layers.
  NoCoupledStartItem(NetId),

  /// The configured pair gap is below the board's minimum clearance.
  ///
  /// Port of "Diff pair gap is less than board minimum clearance."
  /// (`pcbnew/router/pns_router.cpp:231`). It is the only consistency
  /// check KiCad makes between the pair gap, which is geometry, and the
  /// clearance rules, which are what the two lanes are then tested
  /// against; note 07 section 8.3 spells out what happens without it.
  PairGapBelowMinClearance,

  /// The pair of tracks under the cursor is not spaced like the
  /// configured pair.
  ///
  /// Port of "The differential pair gap at the start point does not match
  /// the configured gap." (`pcbnew/router/pns_router.cpp:373`), inside a
  /// ten percent tolerance. It applies only when starting from a segment
  /// or an arc: the spacing of two pads or two vias is fixed by their
  /// placement and not by routing rules (`:359`).
  PairGapMismatch,

  /// A tuning session was asked for with no object to tune.
  ///
  /// The `!aStartItem` half of "Please select a track whose length you
  /// want to tune." (`pcbnew/router/pns_meander_placer.cpp:71`). A
  /// tuning session **requires** an object where a single track
  /// placement does not: there is nothing to lengthen otherwise.
  TuningNeedsStartItem,

  /// The object a tuning session was asked to tune is not a track.
  ///
  /// The `!OfKind( SEGMENT_T | ARC_T )` half of the same message
  /// (`pcbnew/router/pns_meander_placer.cpp:71`). A pad, a via or a hole
  /// is refused.
  NotATrack(ItemId),

  /// The topology walk found no copper for a tuning session to measure.
  ///
  /// KiCad has no such refusal; see
  /// [`crate::placer::meander_placer::TuningError::NoTuningPath`] for why
  /// it is one here.
  NoTuningPath,

  /// The track a pair length tuning session was asked to tune is not
  /// half of a differential pair.
  ///
  /// Port of "Unable to find complementary differential pair net for
  /// length tuning. Make sure the names of the nets belonging to a
  /// differential pair end with either _N/_P or +/-."
  /// (`pcbnew/router/pns_dp_meander_placer.cpp:107`). This crate has no
  /// net names and invents no convention: the host answers
  /// [`crate::rules::RuleResolver::dp_coupled_net`], and a host with no
  /// pair concept answers [`None`] and reaches this.
  NotADiffPairForTuning,

  /// The same, from a skew tuning session.
  ///
  /// Port of "...for skew tuning..."
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:79`), a separate variant
  /// because KiCad words the two differently and a host shows the message
  /// the mode calls for.
  NotADiffPairForSkew,

  /// One lane of the recovered pair holds no segment.
  ///
  /// The unreported `return false` of
  /// `pcbnew/router/pns_dp_meander_placer.cpp:117` and
  /// `pns_meander_skew_placer.cpp:88`, which leaves KiCad's status bar
  /// empty. It cannot arise from a successful pair assembly; see
  /// [`crate::placer::meander_placer::TuningError::PairLaneHasNoSegments`].
  PairLaneHasNoSegments,

  /// The object under the drag is not something a drag can move.
  ///
  /// Port of the `default:` of `DRAGGER::Start`
  /// (`pcbnew/router/pns_dragger.cpp:355`), which refuses a solid, a hole
  /// and anything else that is neither a segment, an arc nor a via, and
  /// of `startDragArc`'s refusal, which has no counterpart while this
  /// crate has no arcs.
  NotDraggable(ItemId),
}

/// The six ways a tuning session refuses to start, as a host sees them.
impl From<TuningError> for StartError {
  fn from(error: TuningError) -> Self {
    match error {
      TuningError::NeedsStartItem => StartError::TuningNeedsStartItem,
      TuningError::NotATrack(item) => StartError::NotATrack(item),
      TuningError::NoTuningPath => StartError::NoTuningPath,
      TuningError::NotADiffPairForTuning => StartError::NotADiffPairForTuning,
      TuningError::NotADiffPairForSkew => StartError::NotADiffPairForSkew,
      TuningError::PairLaneHasNoSegments => StartError::PairLaneHasNoSegments,
    }
  }
}

/// The three ways pair identification fails, as a host sees them.
impl From<PairError> for StartError {
  fn from(error: PairError) -> Self {
    match error {
      PairError::NotADiffPair => StartError::NotADiffPair,
      PairError::NoDanglingAnchor => StartError::NoDanglingAnchor,
      PairError::NoCoupledItem(net) => StartError::NoCoupledStartItem(net),
    }
  }
}

// ---------------------------------------------------------------------
// The preview
// ---------------------------------------------------------------------

/// How a host should draw one preview element.
///
/// Port of the four flags of `router_preview_item.h:50` to `:53`, which
/// are what `ROUTER_PREVIEW_ITEM` turns into a colour and a width
/// (`router_preview_item.cpp:90`, `:212`, `:608`).
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum PreviewStyle {
  /// The track being routed right now. `PNS_HEAD_TRACE`, which
  /// `movePlacing` sets on every trace and on its via
  /// (`pcbnew/router/pns_router.cpp:804`, `:821`).
  #[default]
  Head,

  /// Geometry this session has already fixed into the speculative node.
  ///
  /// KiCad draws these with no flag at all, from `updateView`'s added
  /// loop (`pcbnew/router/pns_router.cpp:766`). They are the settled part
  /// of the route plus, when the session started or ended in the middle
  /// of a track, the halves that track was split into.
  Tail,

  /// The item under the cursor. `PNS_HOVER_ITEM`.
  ///
  /// Set by the host tool and never by the engine, which does not know
  /// where the cursor is between events. It is in the enum so that a host
  /// has one vocabulary for its preview layer.
  Hover,

  /// One primitive of a rule area. `PNS_SEMI_SOLID`
  /// (`pcbnew/router/pns_kicad_iface.cpp:2488`).
  ///
  /// Nothing emits it yet, because rule areas are not synced; see the
  /// [`crate::snapshot`] module documentation.
  SemiSolid,

  /// Something a violation was found on. `PNS_COLLISION`, which KiCad
  /// derives from the `MK_VIOLATION` marker
  /// (`router_preview_item.cpp:211`).
  Collision,
}

/// One polyline of a preview frame.
///
/// Port of one `ROUTER_IFACE::DisplayItem` call
/// (`pcbnew/router/pns_router.h:105`) for something track shaped.
#[derive(Clone, PartialEq, Debug)]
pub struct PreviewItem {
  /// The centre line.
  pub chain: LineChain,
  /// The full width in nanometres.
  pub width: i32,
  /// The copper layer to draw on.
  pub layer: i32,
  /// The net, for highlighting.
  pub net: Option<NetId>,
  /// How to draw it.
  pub style: PreviewStyle,
  /// The clearance outline the host draws around it, when a rule applies.
  ///
  /// KiCad's `DisplayItem` takes `aClearance` from
  /// `GetRuleResolver()->Clearance( item, nullptr )`, the one sided query
  /// (`pcbnew/router/pns_router.cpp:769`, `:801`).
  pub clearance: Option<i32>,
}

/// The via of a preview frame.
///
/// Port of the `DisplayItem( &l->Via(), ... )` call of `movePlacing`
/// (`pcbnew/router/pns_router.cpp:821`).
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct PreviewVia {
  /// The centre.
  pub pos: Vec2,
  /// The copper diameter in nanometres.
  pub diameter: i32,
  /// The drill diameter in nanometres.
  pub drill: i32,
  /// The copper layers it spans.
  pub layers: LayerRange,
  /// The net.
  pub net: Option<NetId>,
  /// How to draw it.
  pub style: PreviewStyle,
  /// The clearance outline the host draws around it.
  ///
  /// Port of `pcbnew/router/pns_router.cpp:808` to `:819`: the copper
  /// clearance, widened to the hole clearance minus the annular ring
  /// whenever the hole rule reaches further out than the copper rule
  /// does.
  pub clearance: Option<i32>,
}

/// One obstacle the current route runs into.
///
/// Port of the `updateItem` lambda of `ROUTER::markViolations`
/// (`pcbnew/router/pns_router.cpp:672` to `:699`), which draws a clone of
/// the obstacle with the resolved clearance and sometimes hides the
/// original.
///
/// # Deviation
///
/// KiCad ORs `MK_VIOLATION` into the obstacle's marker bits at `:729`.
/// The only reader of that bit in its whole tree is the preview item
/// constructor (`router_preview_item.cpp:211`), which turns it into the
/// collision colour, and `NODE::ClearRanks` clears it again when the
/// session stops. That colour is [`PreviewStyle::Collision`] here, so the
/// marker is not written: a preview then leaves no trace in the world and
/// [`Router::move_to`] cannot change what a later shove sees.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct ViolationMarker {
  /// The obstacle, as the engine knows it.
  pub item: ItemId,
  /// The obstacle, as the host knows it. [`None`] for something this
  /// session created and the host has no id for yet.
  pub host: Option<HostId>,
  /// The clearance that was asked for and not met.
  pub clearance: i32,
  /// The layer to draw the obstacle on instead of its own.
  ///
  /// Port of `pcbnew/router/pns_router.cpp:682`: a multilayer obstacle
  /// hit by a single layer route is drawn on the route's layer, so that
  /// the violation appears where the user is working, unless the obstacle
  /// has per layer shapes and the substitution would draw the wrong one.
  pub forced_layer: Option<i32>,
  /// Whether the host should hide the obstacle's normal rendering while
  /// this marker is drawn.
  ///
  /// False for one primitive of a compound object, "we are only
  /// highlighting one (or more) of several primitives so we do not want
  /// all the other parts of the object to disappear" (`:688`).
  pub hide_original: bool,
}

/// Everything a host has to draw after one event.
///
/// Port of the whole view decoration half of `ROUTER_IFACE`, gathered
/// into one value as note 05 section 7.2 proposes. The frame is complete:
/// a host replaces what it drew last time rather than merging, which is
/// what KiCad's `EraseView` at the top of every move already amounts to
/// (`pcbnew/router/pns_router.cpp:791`).
#[derive(Clone, PartialEq, Debug, Default)]
pub struct PreviewFrame {
  /// The track shaped geometry, the route first and the settled
  /// geometry after it.
  pub items: Vec<PreviewItem>,

  /// The via the next fix would place, when one is pending.
  ///
  /// For a differential pair this is the P lane's; the N lane's is
  /// [`PreviewFrame::via_n`]. `movePlacing` draws one per trace
  /// (`pcbnew/router/pns_router.cpp:806`), which for a pair is two.
  pub via: Option<PreviewVia>,

  /// The second via of a pending differential pair via placement.
  ///
  /// Always [`None`] while a single track is being routed.
  pub via_n: Option<PreviewVia>,

  /// Vias this session has already fixed into the speculative node.
  ///
  /// The via half of `updateView`'s added loop
  /// (`pcbnew/router/pns_router.cpp:766`), which [`PreviewFrame::via`]
  /// cannot carry because there is at most one pending via and there may
  /// be several fixed ones.
  pub fixed_vias: Vec<PreviewVia>,

  /// The rat line from the end of the route to what it still has to
  /// reach.
  ///
  /// Port of `DisplayRatline` as driven by
  /// `LINE_PLACER::updateLeadingRatLine`
  /// (`pcbnew/router/pns_line_placer.cpp:2021`), which the placer stores
  /// rather than draws; see
  /// [`crate::placer::line_placer::LinePlacer::leading_rat_line`].
  pub ratline: Option<LineChain>,

  /// The same for the N lane of a differential pair.
  ///
  /// `DIFF_PAIR_PLACER::updateLeadingRatLine` draws one rat line per lane
  /// with that lane's own net
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:914`). Always [`None`]
  /// while a single track is being routed.
  pub ratline_n: Option<LineChain>,

  /// Every obstacle the route runs into.
  pub violations: Vec<ViolationMarker>,

  /// Board objects the host must stop drawing.
  ///
  /// Port of `updateView`'s removed loop, which calls `HideItem` for
  /// every item the speculative node takes out of the board
  /// (`pcbnew/router/pns_router.cpp:772`): the loops the new route made
  /// redundant, and the track a session started or ended in the middle
  /// of. Items the engine itself created and then dropped are not listed,
  /// because the host only ever drew them from an earlier frame and every
  /// frame is complete.
  ///
  /// A component drag's pads are in here too, and are also in
  /// [`PreviewFrame::moved_solids`] with the offset to draw them at.
  pub hidden: Vec<HostId>,

  /// The tuning readout, when the session is a length tuning session.
  ///
  /// KiCad's host reads `TuningStatus()` and `TuningLengthResult()` off
  /// the placer after every `Move`
  /// (`pcbnew/generators/pcb_tuning_pattern.cpp:1321`, `:1322`) and
  /// composes them into the string it shows (`:1351`). Always [`None`]
  /// while anything else is running.
  ///
  /// Boxed because a frame is returned by value from every
  /// [`Router::move_to`] and a [`TuningInfo`] carries a whole
  /// [`MeanderSettings`], which no routing or dragging frame has any use
  /// for.
  pub tuning: Option<Box<TuningInfo>>,

  /// Board objects the host must draw at an offset instead of where they
  /// are.
  ///
  /// The preview counterpart of [`CommitDiff::moved_solids`], and empty
  /// for everything except a component drag.
  ///
  /// KiCad splits this between the router and the tool: `updateView`
  /// hides the old solid and draws the cloned one
  /// (`pcbnew/router/pns_router.cpp:766`, `:772`), while
  /// `ROUTER_TOOL::InlineDrag` clones the footprint's graphics, its non
  /// copper pads, its reference and its value, translates each by the
  /// cursor offset and hides the originals
  /// (`pcbnew/router/router_tool.cpp:3076` to `:3113`), under the comment
  /// "Pads with copper or holes are handled by the router" (`:3100`).
  /// There is no geometry here to draw a pad's copper with and no reason
  /// to invent one, since the host already owns the object; the offset is
  /// what it needs and it is the same offset the tool uses for everything
  /// else the component carries.
  pub moved_solids: Vec<(HostId, Vec2)>,
}

/// What a host shows a user during a length tuning session.
///
/// The four values `PCB_TUNING_PATTERN::Update` reads back off the placer
/// (`pcbnew/generators/pcb_tuning_pattern.cpp:1319` to `:1322`) plus the
/// target they are read against, which KiCad's host already has because
/// it is the one that set it.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct TuningInfo {
  /// How the tuned line stands against its target. `TuningStatus()`
  /// (`pcbnew/router/pns_meander_placer_base.h:76`), which the host turns
  /// into "too long", "too short" or "tuned" (`pcb_tuning_pattern.cpp:1328`).
  pub status: TuningStatus,

  /// Which of the three tuning modes produced this readout.
  ///
  /// KiCad's host knows it because it set `ROUTER::SetMode` itself
  /// (`pcbnew/generators/pcb_tuning_pattern.cpp:1285`) and branches on it
  /// to label the number (`:2147`). It travels with the readout here so
  /// that a host that only holds the frame can label it too.
  pub mode: TuningMode,

  /// The length the last move produced, in nanometres.
  ///
  /// `TuningLengthResult()` (`pns_meander_placer_base.h:61`). It is a
  /// **skew** rather than a length in [`TuningMode::PairSkew`], where the
  /// skew placer overrides the accessor
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:240`); the same number
  /// is in [`TuningInfo::skew`] under its own name for that mode.
  pub result: i64,

  /// How far that has moved from the length the session started at.
  ///
  /// `TuningLengthDelta()` (`:70`), [`None`] when `HasBaseline()` (`:68`)
  /// is false, which without the delay half means the path measured zero.
  pub delta: Option<i64>,

  /// The window [`TuningInfo::status`] was decided against.
  ///
  /// This is what `doMove` actually compared, which is not the same thing
  /// as [`MeanderSettings::target_length`]: an unconstrained target is
  /// resolved to KiCad's `LENGTH_UNCONSTRAINED` triple before the
  /// comparison; see [`crate::placer::meander_placer::MeanderPlacer::move_to`].
  pub target: LengthTarget,

  /// The settings after the move, so that a host can persist the initial
  /// side flip.
  ///
  /// KiCad's host reads the whole settings block back for exactly this
  /// reason (`pcbnew/generators/pcb_tuning_pattern.cpp:1319`), because
  /// `flipInitialSide` writes into it from inside the shape generator
  /// (`pcbnew/router/pns_meander.cpp:287`).
  pub settings: MeanderSettings,

  /// The difference in length between the two lanes, when the session
  /// measures one.
  ///
  /// `MEANDER_SKEW_PLACER::CurrentSkew`
  /// (`pcbnew/router/pns_meander_skew_placer.cpp:195`), which is the same
  /// number [`TuningInfo::result`] carries in [`TuningMode::PairSkew`].
  /// It is here under its own name so that a host can show it without
  /// knowing about the override.
  ///
  /// [`None`] in the other two modes. The pair **length** tuner has no
  /// skew concept: it tunes the longer lane's length
  /// (`pns_dp_meander_placer.cpp:180`) and never compares the two, so
  /// there is no number of KiCad's to report.
  pub skew: Option<i64>,

  /// The skew window the session was aimed at.
  ///
  /// [`MeanderSettings::target_skew`], resolved the way the skew placer
  /// resolves it, and [`None`] outside [`TuningMode::PairSkew`] where
  /// nothing reads the field. [`TuningInfo::target`] is the **length**
  /// window the status was actually decided against, which in skew mode
  /// is this window plus the coupled lane's length.
  pub skew_target: Option<LengthTarget>,

  /// The coupled lane's total length, in [`TuningMode::PairSkew`].
  ///
  /// `m_coupledLength` (`pcbnew/router/pns_meander_skew_placer.h:73`),
  /// which is what the skew is measured against and what makes the two
  /// numbers a host shows add up. [`None`] in the other two modes.
  pub coupled_length: Option<i64>,
}

// ---------------------------------------------------------------------
// The commit
// ---------------------------------------------------------------------

/// The geometry of an item a routing session produced.
///
/// A single track placer emits segments and vias and nothing else
/// (`pcbnew/router/pns_line_placer.cpp:1669`, `:1704`).
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum NewGeometry {
  /// A straight track.
  Segment {
    /// The centre line.
    seg: Seg,
    /// The full width in nanometres.
    width: i32,
  },
  /// A via, with the hole it needs drilling.
  Via {
    /// The centre.
    pos: Vec2,
    /// The copper diameter in nanometres.
    diameter: i32,
    /// The drill diameter in nanometres.
    drill: i32,
    /// Through, blind, buried or micro.
    via_type: ViaType,
  },
}

/// One item a host has to create or update.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct NewItem {
  /// What it looks like.
  pub geometry: NewGeometry,
  /// The net it belongs to.
  pub net: Option<NetId>,
  /// The copper layers it occupies.
  pub layers: LayerRange,
  /// The host object it descends from, when there is one.
  ///
  /// Port of `ITEM::GetSourceItem` (`pcbnew/router/pns_item.h:202`).
  /// KiCad reads it at commit time to inherit solder mask settings, via
  /// tenting and group membership from the object a new item came out of
  /// (`pcbnew/router/pns_kicad_iface.cpp:2775`, `:2838`, `:2875`), and
  /// note 05 section 7.2 asks for the same field here. It is set for the
  /// halves an existing track was split into, and unset for a freshly
  /// routed segment.
  pub source: Option<HostId>,
}

/// Why a commit could not describe one item to the host.
///
/// [`NewGeometry`] covers what a single track placer emits, and nothing
/// else; an item of any other shape that reaches the commit is reported
/// here instead of being dropped. Read it with
/// [`Router::last_commit_error`].
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum CommitError {
  /// An arc reached the added or the updated side of a commit.
  ///
  /// [`NewGeometry`] gains its arc variant in slice 8 of
  /// `doc/work/012-arcs.md`, together with the host boundary that can
  /// apply one. Until then a commit that would have to emit an arc lists
  /// the **removal** of whatever the arc replaced and leaves the arc
  /// itself out, so a host that ignores this error applies a diff that is
  /// short of one track rather than one that is wrong.
  ///
  /// It cannot arise before slice 7: nothing in the crate produces an arc
  /// item except a host snapshot, and a snapshot item that is never
  /// touched is in neither list.
  ArcNotRepresentable {
    /// The world unique number of the item, which is what
    /// [`crate::item::Item::uid`] answers and what the debug output of a
    /// failing session names.
    uid: u64,
  },
  /// The commit reached a body no [`NewGeometry`] describes and none ever
  /// will: a solid or a hole.
  ///
  /// The commit's own kind filter drops both before the lists are built,
  /// so this is the arm that says so rather than one a caller can
  /// observe.
  NotCommittable {
    /// The item's world unique number.
    uid: u64,
  },
}

/// What one routing session changed.
///
/// Port of `ROUTER::CommitRouting( NODE* )`
/// (`pcbnew/router/pns_router.cpp:862`), whose three `ROUTER_IFACE` calls
/// become three lists a host applies inside one undo transaction, plus
/// the fourth thing `PNS_KICAD_IFACE` reconstructs out of those calls for
/// a component drag. Note 05 section 7.2 asks for exactly this shape and
/// names the fourth list `moved_pads`.
#[derive(Clone, PartialEq, Debug, Default)]
pub struct CommitDiff {
  /// Objects the host must delete.
  pub removed: Vec<HostId>,
  /// Objects the host must create. It assigns their ids and reports them
  /// back with [`Router::assign_host_ids`].
  pub added: Vec<NewItem>,
  /// Objects the host must rewrite in place, keeping their identity.
  ///
  /// The result of the remove plus add fold; see [`CommitDiff::removed`]
  /// and the port note on [`Router::stop_routing`].
  pub updated: Vec<(HostId, NewItem)>,
  /// Pads a component drag moved, and how far each one moved.
  ///
  /// Empty for everything except
  /// [`crate::component_dragger::ComponentDragger`], so every other
  /// session and every stored fixture is unaffected.
  ///
  /// A component drag reports the pad it moved as an ordinary removal
  /// plus addition, and `PNS_KICAD_IFACE` intercepts both:
  /// `RemoveItem` records the old position and returns without touching
  /// the commit (`pcbnew/router/pns_kicad_iface.cpp:2629` to `:2636`),
  /// `createBoardItem` records the new position and returns `nullptr`
  /// with the comment "Don't add to commit; we'll add the parent
  /// footprints when processing the m_fpOffsets" (`:2849` to `:2856`),
  /// and `Commit()` then moves each pad's **footprint** by the
  /// difference, once per footprint (`:2918` to `:2933`). So the board
  /// never sees a pad deleted and re-created; it sees one component
  /// moved.
  ///
  /// This is that pair as a value: the pad's own host id and the offset
  /// it moved by. A host resolves the id to whatever owns the pad, a
  /// footprint in KiCad and a device instance in LibrePCB, and moves that
  /// object by the offset, de-duplicating owners as `processedFootprints`
  /// does. One entry per host object, in the drag's own uid order; a pad
  /// that became several solids, one per padstack layer, still appears
  /// once, because all of them move by the same vector.
  pub moved_solids: Vec<(HostId, Vec2)>,
}

/// What a running session has changed so far, before any commit.
///
/// Port of `ROUTER::GetUpdatedItems`
/// (`pcbnew/router/pns_router.cpp:832`), the read only view of the
/// placer's speculative node that KiCad's regression harness compares
/// against the golden stored in a `pns.log`
/// (`qa/tools/pns/pns_log_player.cpp:63`).
///
/// It is deliberately **not** a [`CommitDiff`]. A commit folds a removal
/// and an addition that share a host object into an update, drops the
/// holes a via drilled, and in shove mode rewinds to the last locked node
/// first (`pcbnew/router/pns_line_placer.cpp:1810`); none of that has
/// happened yet here. A [`CommitDiff`] is what a host applies, this is
/// what a harness measures.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct PendingUpdate {
  /// Board objects the session has taken out of the world so far.
  ///
  /// Only objects the snapshot named: an item the session created and
  /// then removed again has no host id, which is the `item->Parent()`
  /// filter of `qa/tools/pns/pns_log_player.cpp:70`.
  pub removed: Vec<HostId>,
  /// Every item the session has put in, in uid order.
  ///
  /// Segments, vias and the holes those vias drilled, because the node
  /// delta does not filter by kind.
  pub added: Vec<ItemId>,
  /// The speculative node the two lists were read from.
  ///
  /// KiCad's `GetUpdatedItems` keeps this to itself; it is returned here
  /// so that a harness can run a collision query against the world the
  /// session is standing in rather than against the board it started
  /// from. [`None`] when nothing is being routed.
  pub node: Option<NodeId>,
}

/// What happened to a fix.
///
/// KiCad's `ROUTER::FixRoute` answers with the placer's bool, which is
/// `true` for "the route reached its target" and `false` for both "one
/// more corner was pinned, keep going" and "the fix was refused"
/// (`pcbnew/router/pns_line_placer.cpp:1750`, `:1587`). Its host cannot
/// tell the last two apart and does not need to; this splits the terminal
/// case out, because that is the one where the facade commits.
// `LineChain` grew two vectors with the arcs (work item 012 slice 3), which
// put this enum over clippy's size difference threshold. Boxing the large
// variant would move a hot path's payload to the heap to satisfy a lint
// about a value that is built once per call, so the lint is turned off
// here instead.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, PartialEq, Debug)]
pub enum FixOutcome {
  /// The placement continues. The frame is the state after the fix, which
  /// KiCad's host immediately follows with a fresh `Move`.
  ///
  /// A refused fix, which is what the collision gate of
  /// `LINE_PLACER::FixRoute` produces when the route is not clear and
  /// rule violations are not allowed (`:1587`), also lands here with the
  /// frame unchanged.
  Continue(PreviewFrame),
  /// The route reached its target and was committed.
  Finished(CommitDiff),
}

/// What [`Router::continue_from_end`] left behind.
///
/// KiCad's `ContinueFromEnd` answers with a bool and hands the new start
/// object back through an out parameter (`pcbnew/router/pns_router.cpp`
/// `:645`); its commit needs no return value because the interface has
/// already written the edits into the board. Here the commit is a value,
/// so it travels with the rest.
#[derive(Clone, PartialEq, Debug)]
pub struct ContinueOutcome {
  /// What the finished half of the route changed. A host applies it
  /// whether or not the restart succeeded.
  pub diff: CommitDiff,
  /// The preview of the new placement after it was moved back towards
  /// where the user was. [`None`] when the restart at the anchor was
  /// refused, which leaves the router idle.
  pub frame: Option<PreviewFrame>,
  /// The object the new placement started on.
  pub start: Option<HostId>,
}

/// Where a route could be carried on to.
///
/// Port of the three out parameters of `ROUTER::GetNearestRatnestAnchor`
/// (`pcbnew/router/pns_router.cpp:518`).
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct RatsnestAnchor {
  /// The point to route to.
  pub at: Vec2,
  /// The layers the object at that point spans.
  pub layers: LayerRange,
  /// The object itself, when the anchor came from one.
  pub item: Option<ItemId>,
}

/// What a session is routing.
///
/// Port of `ROUTER::GetCurrentNets` (`pcbnew/router/pns_router.cpp:1031`),
/// a `std::vector<NET_HANDLE>` whose length is the only thing that says
/// whether a pair is being placed. `DESIGN.md` section 11 prefers an enum
/// over a length, so the two cases are named.
///
/// The nets are optional because a route need not have one: a track
/// started in empty space takes
/// [`crate::rules::RuleResolver::orphaned_net`].
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum RoutedNets {
  /// Nothing is being routed.
  #[default]
  None,
  /// One track, or one dragged object.
  Single(Option<NetId>),
  /// A differential pair, the positive half first.
  Pair(Option<NetId>, Option<NetId>),
}

impl RoutedNets {
  /// The first net, which is what `CurrentNets()[0]` means to the three
  /// places in KiCad's router that read only one
  /// (`pcbnew/router/pns_router.cpp:549`, `:1033`).
  pub const fn first(self) -> Option<NetId> {
    match self {
      RoutedNets::None => None,
      RoutedNets::Single(net) | RoutedNets::Pair(net, _) => net,
    }
  }
}

// ---------------------------------------------------------------------
// The facade
// ---------------------------------------------------------------------

/// One interactive routing session over one board.
///
/// See the module documentation for the port's boundaries. A host builds
/// one from a [`WorldSnapshot`], drives it with the event methods, and
/// applies the [`CommitDiff`] the session ends with.
pub struct Router {
  /// The board and every speculative branch over it.
  world: World,
  /// The two way map between the host's ids and the arena's.
  index: HostIndex,
  /// The persistent settings. Port of `m_settings`, which KiCad holds as
  /// a pointer into an object the host owns
  /// (`pcbnew/router/pns_router.h:294`).
  settings: RoutingSettings,
  /// The geometry the next route is placed with. Port of `m_sizes`
  /// (`:295`).
  sizes: Sizes,
  /// The rule oracle. KiCad reaches it through the interface and the
  /// node (`pcbnew/router/pns_router.h:212`); it is owned here, because
  /// nothing else may hold it while the engine runs.
  resolver: Box<dyn RuleResolver>,
  /// Where an algorithm draws its own internals.
  debug: Box<dyn DebugDecorator>,
  /// The single track placer, alive exactly in
  /// [`RouterState::RouteTrack`]. Port of `m_placer` (`:283`).
  ///
  /// KiCad's `ROUTE_TRACK` covers a differential pair too: its
  /// `RouterState` has no pair state and the mode is a separate field
  /// (`pcbnew/router/pns_router.h:69`). So does this, and
  /// [`crate::placer::Placer`] is what says which of the two is running.
  placer: Option<Placer>,
  /// The dragger, alive exactly in [`RouterState::DragSegment`]. Port of
  /// `m_dragger` (`:284`), which KiCad holds as a
  /// `std::unique_ptr<DRAG_ALGO>` over three implementations. See
  /// [`ActiveDragger`] for why the two that are in scope are an enum and
  /// not a trait object.
  dragger: Option<ActiveDragger>,
  /// The segments the last multi drag move handed back for the host to
  /// re-select. Port of `m_leaderSegments`
  /// (`pcbnew/router/pns_router.h:284`), cleared by
  /// [`Router::start_dragging`] (`:168`) and refilled by every drag move
  /// (`:663`). A single drag never contributes, because
  /// `DRAG_ALGO::GetLastCommittedLeaderSegments`'s default body answers
  /// an empty vector (`pcbnew/router/pns_drag_algo.h:125`).
  leader_segments: Vec<ItemId>,
  /// Which algorithm is running. Port of `m_state` (`:279`).
  state: RouterState,
  /// How many copper layers the board has, from the snapshot.
  copper_layer_count: u8,
  /// The arena handles behind the last [`CommitDiff::added`], in that
  /// vector's order, so that [`Router::assign_host_ids`] can find them.
  committed: Vec<ItemId>,
  /// Where the session is written down, when one is being recorded.
  ///
  /// Port of `m_logger` (`pcbnew/router/pns_router.h:293`), which KiCad
  /// allocates only behind an advanced configuration flag
  /// (`pcbnew/router/pns_router.cpp:69`). See [`crate::eventlog`].
  recorder: Option<Recorder>,
  /// How deep inside a composite facade method the session is.
  ///
  /// [`Router::finish`] and [`Router::continue_from_end`] pick their own
  /// arguments and drive the smaller methods themselves, so each is one
  /// event and the events those calls would record are suppressed. A
  /// commit is recorded whatever the depth, because it is a result and
  /// not an input.
  recording_depth: u32,
  /// What the last commit could not describe, if anything.
  ///
  /// KiCad has no counterpart: `ROUTER::CommitRouting` hands every non
  /// virtual item to `ROUTER_IFACE::AddItem`
  /// (`pcbnew/router/pns_router.cpp:896`), and `createBoardItem`'s
  /// `default:` answers `nullptr` for a kind it cannot build
  /// (`pcbnew/router/pns_kicad_iface.cpp:2858`), which drops it in
  /// silence. Here the drop is reported instead, see [`CommitError`]. Set
  /// by every commit, including the ones that describe everything, so it
  /// always names the most recent one.
  last_commit_error: Option<CommitError>,
}

impl Router {
  // -----------------------------------------------------------------
  // Construction and configuration
  // -----------------------------------------------------------------

  /// A session over a board.
  ///
  /// Port of the constructor (`pcbnew/router/pns_router.cpp:60`) followed
  /// by `SetInterface`, `LoadSettings`, `UpdateSizes` and `SyncWorld`,
  /// which is the order `PNS::TOOL_BASE::Reset` uses
  /// (`pcbnew/router/pns_tool_base.cpp:74`). The state starts
  /// [`RouterState::Idle`] and the debug hook starts at
  /// [`NoDebug`].
  pub fn new(
    snapshot: &WorldSnapshot,
    resolver: Box<dyn RuleResolver>,
    settings: RoutingSettings,
    sizes: Sizes,
  ) -> Self {
    let (world, index) = World::from_snapshot(snapshot);

    Self {
      world,
      index,
      settings,
      sizes,
      resolver,
      debug: Box::new(NoDebug),
      placer: None,
      dragger: None,
      leader_segments: Vec::new(),
      state: RouterState::Idle,
      copper_layer_count: snapshot.copper_layer_count,
      committed: Vec::new(),
      recorder: None,
      recording_depth: 0,
      last_commit_error: None,
    }
  }

  /// What the last commit could not describe to the host, if anything.
  ///
  /// [`None`] when the last commit described everything it listed, which
  /// is every commit a board without arc tracks can produce. See
  /// [`CommitError`] for the one case that is reachable today and for
  /// what the diff holds when it fires.
  pub const fn last_commit_error(&self) -> Option<CommitError> {
    self.last_commit_error
  }

  /// Install a trace hook.
  ///
  /// Port of `SetDebugDecorator` as `StartRouting` pushes it into the
  /// placer (`pcbnew/router/pns_router.cpp:470`). Nothing an
  /// implementation does may change an answer; see
  /// [`DebugDecorator`].
  pub fn set_debug(&mut self, debug: Box<dyn DebugDecorator>) {
    self.debug = debug;
  }

  /// Install or remove the session recorder.
  ///
  /// The counterpart of KiCad's `m_logger`
  /// (`pcbnew/router/pns_router.h:293`), which the constructor allocates
  /// only when an advanced configuration flag is set
  /// (`pcbnew/router/pns_router.cpp:69`) and which no setter reaches; the
  /// `SetLogger` in that tree is `ALGO_BASE`'s
  /// (`pcbnew/router/pns_algo_base.h:65`), for handing the pointer down.
  ///
  /// The recorder has to have been built over the same board, settings
  /// and sizes this router holds, or a replay starts somewhere else;
  /// [`Router::start_recording`] cannot get that wrong.
  pub fn set_recorder(&mut self, recorder: Option<Recorder>) {
    self.recorder = recorder;
  }

  /// Start recording this session over the board it was built from.
  ///
  /// The snapshot is the one [`Router::new`] was given: the engine keeps
  /// no copy of it, because it is the host's value and the world is
  /// derived from it.
  pub fn start_recording(&mut self, snapshot: &WorldSnapshot) {
    self.recorder = Some(Recorder::new(
      snapshot.clone(),
      self.settings,
      self.sizes.clone(),
    ));
  }

  /// What has been recorded so far, if anything.
  ///
  /// Port of `Logger()`, `pcbnew/router/pns_router.h:213`.
  pub const fn recorder(&self) -> Option<&Recorder> {
    self.recorder.as_ref()
  }

  /// Take the recording out and stop recording.
  pub fn take_recording(&mut self) -> Option<SessionRecording> {
    self.recorder.take().map(Recorder::into_recording)
  }

  /// Append one event, unless a composite method is driving this call.
  fn record(&mut self, event: SessionEvent) {
    if self.recording_depth == 0
      && let Some(recorder) = self.recorder.as_mut()
    {
      recorder.push_event(event);
    }
  }

  /// Append one commit result, whatever is driving the call.
  fn record_commit(&mut self, diff: &CommitDiff) {
    if let Some(recorder) = self.recorder.as_mut() {
      recorder.push_commit(diff);
    }
  }

  /// Stop recording the events of the methods this one drives.
  fn suspend_recording(&mut self) {
    self.recording_depth = self.recording_depth.saturating_add(1);
  }

  /// Undo one [`Router::suspend_recording`].
  fn resume_recording(&mut self) {
    self.recording_depth = self.recording_depth.saturating_sub(1);
  }

  /// The persistent settings. Port of `Settings()`,
  /// `pcbnew/router/pns_router.h:222`.
  pub const fn settings(&self) -> &RoutingSettings {
    &self.settings
  }

  /// The persistent settings, to change.
  ///
  /// KiCad's `Settings()` hands out a mutable reference to an object the
  /// host owns and edits behind the router's back, which is how
  /// `ToggleCornerMode` writes through it (`:1069`). A change takes
  /// effect on the next event, because every algorithm reads the settings
  /// through the context it is handed.
  pub const fn settings_mut(&mut self) -> &mut RoutingSettings {
    &mut self.settings
  }

  /// The geometry the next route is placed with. Port of `Sizes()`,
  /// `pcbnew/router/pns_router.h:243`.
  pub const fn sizes(&self) -> &Sizes {
    &self.sizes
  }

  /// Change the geometry the next route is placed with.
  ///
  /// Port of `UpdateSizes` (`pcbnew/router/pns_router.cpp:781`), which
  /// also pushes the new sizes into a running placer; that is how KiCad's
  /// width and via size actions take effect mid route.
  ///
  /// Only the differential pair placer takes them
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:782`). [`LinePlacer`] has
  /// no such entry point yet, so a running single track placement keeps
  /// the sizes it started with and a host applies a width change by
  /// fixing and starting a new leg; that is a milestone 5 follow up,
  /// tracked in `doc/work/005-session-api-and-event-log.md`.
  pub fn set_sizes(&mut self, sizes: Sizes) {
    self.record(SessionEvent::SetSizes {
      sizes: sizes.clone(),
    });
    self.sizes = sizes.clone();

    if let Some(placer) = self.placer.as_mut() {
      placer.update_sizes(sizes);
    }
  }

  /// Replace the persistent settings.
  ///
  /// The recordable form of [`Router::settings_mut`], which hands out a
  /// mutable reference and never learns what was done with it. KiCad has
  /// only the reference (`pcbnew/router/pns_router.h:227`); a host that
  /// wants its settings changes to survive into a replay goes through
  /// this instead. A change takes effect on the next event, because every
  /// algorithm reads the settings through the context it is handed.
  pub fn set_settings(&mut self, settings: RoutingSettings) {
    self.record(SessionEvent::SetSettings { settings });
    self.settings = settings;
  }

  /// The board and its speculative branches.
  ///
  /// Port of `GetWorld()`, `pcbnew/router/pns_router.h:196`, which hands
  /// out the root. The whole arena is reachable here, so a host can
  /// inspect what a session has built before committing it.
  pub const fn world(&self) -> &World {
    &self.world
  }

  /// How many threads the obstacle query may use.
  ///
  /// Forwards to [`World::set_parallelism`]. It is here and not on
  /// [`RoutingSettings`] because a session recording serialises the
  /// settings and a thread count belongs to the machine, not to the
  /// session; the router answers the same whatever it is set to.
  pub const fn set_parallelism(&mut self, threads: usize) {
    self.world.set_parallelism(threads);
  }

  /// The map between the host's ids and the engine's handles.
  pub const fn host_index(&self) -> &HostIndex {
    &self.index
  }

  /// How many copper layers the board has, from the snapshot.
  pub const fn copper_layer_count(&self) -> u8 {
    self.copper_layer_count
  }

  /// Which algorithm is running. Port of `GetState()`,
  /// `pcbnew/router/pns_router.h:172`.
  pub const fn state(&self) -> RouterState {
    self.state
  }

  /// Whether a session is running. Port of `RoutingInProgress()`,
  /// `pcbnew/router/pns_router.cpp:120`.
  pub fn routing_in_progress(&self) -> bool {
    self.state != RouterState::Idle
  }

  /// The layer being routed or dragged on. Port of `GetCurrentLayer()`,
  /// `pcbnew/router/pns_router.cpp:1041`, whose `-1` for "neither a
  /// placer nor a dragger" becomes [`None`]. The placer answers first
  /// there and here, although the two are never both alive.
  ///
  /// The dragger's answer is `m_draggedLine.Layer()`, which is wrong in
  /// [`crate::dragger::DragMode::Via`] and unreachable in KiCad; see
  /// [`Dragger::current_layer`].
  pub fn current_layer(&self) -> Option<i32> {
    if let Some(placer) = self.placer.as_ref() {
      return placer.current_layer();
    }

    self.dragger.as_ref().map(ActiveDragger::current_layer)
  }

  /// The net being routed or dragged. Port of `GetCurrentNets()`,
  /// `pcbnew/router/pns_router.cpp:1031`, which wraps the placer's or the
  /// dragger's one net in a vector.
  pub fn current_net(&self) -> Option<NetId> {
    if let Some(placer) = self.placer.as_ref() {
      return placer.current_net();
    }

    self.dragger.as_ref().and_then(ActiveDragger::current_net)
  }

  /// Every net the session is routing.
  ///
  /// Port of `ROUTER::GetCurrentNets`
  /// (`pcbnew/router/pns_router.cpp:1031`). A differential pair answers
  /// with both halves, P first; everything else answers with the one net
  /// [`Router::current_net`] gives.
  pub fn current_nets(&self) -> RoutedNets {
    if let Some(placer) = self.placer.as_ref() {
      return if placer.routes_two_nets() {
        RoutedNets::Pair(placer.current_net(), placer.current_net_n())
      } else {
        RoutedNets::Single(placer.current_net())
      };
    }

    match self.dragger.as_ref() {
      Some(dragger) => RoutedNets::Single(dragger.current_net()),
      None => RoutedNets::None,
    }
  }

  /// Whether the next fix would place a via. Port of `IsPlacingVia()`,
  /// `pcbnew/router/pns_router.cpp:1059`.
  pub fn placing_via(&self) -> bool {
    self.placer.as_ref().is_some_and(Placer::is_placing_via)
  }

  // -----------------------------------------------------------------
  // Queries
  // -----------------------------------------------------------------

  /// Every host object under a point.
  ///
  /// Port of `ROUTER::QueryHoverItems( aP, 0 )`
  /// (`pcbnew/router/pns_router.cpp:126`), the exact hit test, against
  /// the node the placer stands on rather than the committed board
  /// (`:128`), so that hovering while routing sees the speculative world.
  ///
  /// `layer` is the filter `PNS::TOOL_BASE::pickSingleItem` applies to
  /// the winner (`pcbnew/router/pns_tool_base.cpp:238`); [`None`] is its
  /// "any layer", which the tool passes while a via is pending
  /// (`:398`). One host object may appear once even though it became
  /// several items.
  ///
  /// The slop radius overload (`pcbnew/router/pns_router.cpp:135`) and
  /// the five priority slots that rank the hits (note 05 section 4.2) are
  /// host work: they depend on the grid step, on layer visibility and on
  /// the high contrast display mode, none of which the engine knows.
  pub fn hover(&self, at: Vec2, layer: Option<i32>) -> Vec<HostId> {
    let node = self.query_node();
    let mut found: Vec<HostId> = Vec::new();

    for id in self.world.hit_test(node, at) {
      let Some(item) = self.world.item(id) else {
        continue;
      };

      if !layer.is_none_or(|layer| item.layers().contains(layer)) {
        continue;
      }

      if let Some(host) = self.index.host_of(id)
        && !found.contains(&host)
      {
        found.push(host);
      }
    }

    found
  }

  /// Whether a route may start here.
  ///
  /// Port of `ROUTER::isStartingPointRoutable`
  /// (`pcbnew/router/pns_router.cpp:222`), the gate in front of
  /// [`Router::start_routing`], for single track mode. It is skipped
  /// entirely when the settings allow rule violations (`:224`), and it
  /// probes twice, at the configured track width and then at the board
  /// minimum, because a collision caused only by the width should not
  /// stop the user from starting (`:322`).
  ///
  /// # Deviation
  ///
  /// KiCad skips candidates that live on `Edge_Cuts`, with the comment
  /// "Edge cuts are put on all layers, but they are not really on all
  /// layers" (`:242`). The engine has no board layers, so the board
  /// outline is a non routable solid like any other and a start point
  /// exactly on the outline curve reports
  /// [`StartError::NotRoutable`] where KiCad would have carried on to the
  /// collision probe. Every other case is unchanged, because the loop
  /// clears its failure as soon as one routable item is found (`:248`).
  ///
  /// The differential pair half of the routine (`:227` and `:340` to
  /// `:427`) is [`Router::is_starting_point_routable_diff_pair`], which
  /// shares the per item scan and nothing else.
  pub fn is_starting_point_routable(
    &self,
    at: Vec2,
    start: Option<HostId>,
    layer: i32,
  ) -> Result<(), StartError> {
    // :224
    if self.settings.allow_drc_violations() {
      return Ok(());
    }

    let root = self.world.root();
    let start_item = match start {
      None => None,
      Some(host) => Some(
        self
          .resolve_host_item(root, at, host, Some(layer))
          .ok_or(StartError::UnknownStartItem(host))?,
      ),
    };

    self.check_hovered_items_routable(at, layer)?;

    // :306. The degenerate two point line KiCad probes with; the second
    // append is the allow duplicate one (`:310`).
    let mut chain = LineChain::new();

    chain.append(at);
    chain.append_allow_duplicate(at);

    let mut probe = Line::new();

    probe.set_shape(chain);
    probe.set_layer(layer);
    probe.set_net(
      start_item.and_then(|id| self.world.item(id).and_then(Item::net)),
    );
    probe.set_width(self.sizes.track_width);

    if self.probe_collides(&probe) {
      // :324
      probe.set_width(self.sizes.board_min_track_width);

      if self.probe_collides(&probe) {
        return Err(StartError::StartPointViolatesRules);
      }
    }

    Ok(())
  }

  /// The per item half of the start gate, shared by both modes.
  ///
  /// `pcbnew/router/pns_router.cpp:238` to `:302`: every object under the
  /// point that reaches the layer is examined, and one routable object is
  /// enough to clear all of them, however many unroutable ones the point
  /// also lands on.
  ///
  /// # Errors
  ///
  /// [`StartError::NotRoutable`], naming the last unroutable object.
  fn check_hovered_items_routable(
    &self,
    at: Vec2,
    layer: i32,
  ) -> Result<(), StartError> {
    let root = self.world.root();
    // :238
    let mut failure = None;

    for id in self.world.hit_test(root, at) {
      let Some(item) = self.world.item(id) else {
        continue;
      };

      // :245
      if !item.layers().contains(layer) {
        continue;
      }

      // :248
      if item.is_routable() {
        failure = None;
        break;
      }

      failure = Some(StartError::NotRoutable(id));
    }

    match failure {
      Some(failure) => Err(failure),
      None => Ok(()),
    }
  }

  /// Whether a differential pair may be started at a point.
  ///
  /// Port of the `PNS_MODE_ROUTE_DIFF_PAIR` half of
  /// `isStartingPointRoutable` (`pcbnew/router/pns_router.cpp:227` and
  /// `:341` to `:427`), which is much larger than the single track one.
  /// Three gates are pair specific: the configured gap has to reach the
  /// board minimum clearance, a start object is required, and a start on
  /// an existing pair of tracks has to find them spaced like the
  /// configured pair to within a tenth. Then two degenerate probe lines,
  /// one per anchor, are tested exactly as the single track gate tests
  /// its one.
  ///
  /// # Errors
  ///
  /// [`StartError::PairGapBelowMinClearance`],
  /// [`StartError::PairNeedsStartItem`],
  /// [`StartError::UnknownStartItem`], the three
  /// [`crate::placer::diff_pair_placer::PairError`] variants,
  /// [`StartError::PairGapMismatch`] and
  /// [`StartError::StartPointViolatesRules`].
  pub fn is_starting_point_routable_diff_pair(
    &self,
    at: Vec2,
    start: Option<HostId>,
    layer: i32,
  ) -> Result<(), StartError> {
    // :224
    if self.settings.allow_drc_violations() {
      return Ok(());
    }

    // :229
    if self.sizes.diff_pair_gap < self.sizes.min_clearance {
      return Err(StartError::PairGapBelowMinClearance);
    }

    self.check_hovered_items_routable(at, layer)?;

    // :343
    let host = start.ok_or(StartError::PairNeedsStartItem)?;
    let root = self.world.root();
    let start_item = self
      .resolve_host_item(root, at, host, Some(layer))
      .ok_or(StartError::UnknownStartItem(host))?;

    // :352
    let pair = DiffPairPlacer::find_dp_primitive_pair(
      &self.world,
      self.resolver.as_ref(),
      root,
      Some(start_item),
    )
    .map_err(StartError::from)?;

    // :363
    let starts_on_track = self
      .world
      .item(start_item)
      .is_some_and(|item| item.of_kind(Kind::SEGMENT | Kind::ARC));

    if starts_on_track {
      // :365 to :375
      let actual = (pair.anchor_p() - pair.anchor_n()).euclidean_norm();
      let configured = self.sizes.diff_pair_pitch();
      let tolerance = configured / 10;

      if (actual - configured).abs() > tolerance {
        return Err(StartError::PairGapMismatch);
      }
    }

    // :382 to :401
    let net_of = |primitive: crate::diff_pair::DpPrimitive| {
      primitive
        .stored()
        .and_then(|id| self.world.item(id))
        .and_then(Item::net)
    };
    let probe_of = |at: Vec2, net: Option<NetId>, width: i32| {
      let mut chain = LineChain::new();

      chain.append(at);
      chain.append_allow_duplicate(at);

      let mut probe = Line::new();

      probe.set_shape(chain);
      probe.set_layer(layer);
      probe.set_net(net);
      probe.set_width(width);
      probe
    };
    let net_p = net_of(pair.prim_p());
    let net_n = net_of(pair.prim_n());
    let collides = |width: i32| {
      self.probe_collides(&probe_of(pair.anchor_n(), net_n, width))
        || self.probe_collides(&probe_of(pair.anchor_p(), net_p, width))
    };

    // :403
    if collides(self.sizes.diff_pair_width) {
      // :408, the same "do not stop the user over a width they can fix
      // later" relaxation the single track gate makes.
      if collides(self.sizes.board_min_track_width) {
        return Err(StartError::StartPointViolatesRules);
      }
    }

    Ok(())
  }

  /// Where the route could be carried on to.
  ///
  /// Port of `ROUTER::GetNearestRatnestAnchor`
  /// (`pcbnew/router/pns_router.cpp:518`): the anchor nearest the drawn
  /// line when the user has drawn one, and otherwise the nearest
  /// unconnected item seen from the joint the placement started on.
  ///
  /// The query runs against the scratch branch, `CurrentNode( true )`
  /// (`:534`), so the loops the last move removed are not in the way.
  ///
  /// # Deviation
  ///
  /// KiCad's first guard is `GetCurrentNets().empty()` (`:521`), which is
  /// only ever true when there is no placer at all: `CurrentNets()` wraps
  /// the placer's one net in a vector even when that net is null
  /// (`pcbnew/router/pns_line_placer.h:200`). The net is therefore only
  /// required on the second branch, where KiCad reads `CurrentNets()[0]`
  /// to find the joint (`:549`). A netless route falls out of the first
  /// branch anyway, because
  /// [`crate::topology::nearest_unconnected_anchor_point`] refuses a net
  /// code of zero or below.
  ///
  /// It needs `&mut self` because the anchor query branches a scratch
  /// node to lay the track into (`pcbnew/router/pns_topology.cpp:116`).
  pub fn nearest_ratsnest_anchor(&mut self) -> Option<RatsnestAnchor> {
    // :523
    let placer = self.placer.as_ref()?;
    let trace = placer.trace()?;
    let node = placer.current_node(true);
    let start = placer.current_start()?;
    let layer = placer.current_layer()?;
    let net = placer.current_net();

    // :539
    if trace.segment_count() > 0 {
      let anchor = topology::nearest_unconnected_anchor_point(
        &mut self.world,
        node,
        self.resolver.as_ref(),
        &trace,
      )?;

      return Some(RatsnestAnchor {
        at: anchor.point,
        layers: anchor.layers,
        item: Some(anchor.item),
      });
    }

    // :546. The placement has not drawn anything yet, so the anchor is
    // measured from the joint it started on.
    let joint = self.world.find_joint(node, start, layer, net)?;
    let found =
      topology::nearest_unconnected_item(&self.world, node, joint, Kind::ANY)?;
    let item = self.world.item(found.item)?;

    Some(RatsnestAnchor {
      at: item.anchor(found.anchor),
      layers: item.layers(),
      item: Some(found.item),
    })
  }

  // -----------------------------------------------------------------
  // The session
  // -----------------------------------------------------------------

  /// Begin routing a track.
  ///
  /// Port of `ROUTER::StartRouting` (`pcbnew/router/pns_router.cpp:434`)
  /// for `PNS_MODE_ROUTE_SINGLE`: the gate runs, a placer is built, the
  /// sizes, the layer and the debug hook go in, and the placer starts.
  ///
  /// KiCad's host follows a successful start with a `Move`, so the frame
  /// returned here is the state of a placement that has not been moved
  /// yet: the route is a single point and there is nothing to draw but
  /// whatever splitting the start left in the node.
  pub fn start_routing(
    &mut self,
    at: Vec2,
    start: Option<HostId>,
    layer: i32,
  ) -> Result<PreviewFrame, StartError> {
    self.record(SessionEvent::StartRouting { at, start, layer });

    if self.routing_in_progress() {
      return Err(StartError::AlreadyRouting);
    }

    let root = self.world.root();
    let start_item = match start {
      None => None,
      Some(host) => Some(
        self
          .resolve_host_item(root, at, host, Some(layer))
          .ok_or(StartError::UnknownStartItem(host))?,
      ),
    };

    self.is_starting_point_routable(at, start, layer)?;

    // :443 to :471, in KiCad's order.
    let mut placer =
      LinePlacer::new(&self.world, root, &self.settings, self.sizes.clone());
    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    placer.set_layer(&mut self.world, &context, layer);

    // :473
    if !placer.start(&mut self.world, &context, at, start_item) {
      return Err(StartError::PlacerRefused);
    }

    self.placer = Some(Placer::Line(Box::new(placer)));
    self.state = RouterState::RouteTrack;

    Ok(self.frame())
  }

  /// Begin routing a differential pair.
  ///
  /// Port of `ROUTER::StartRouting` (`pcbnew/router/pns_router.cpp:434`)
  /// for `PNS_MODE_ROUTE_DIFF_PAIR`. KiCad reaches the same function
  /// through a mode its host set earlier with `SetMode` (`:1094`); note
  /// 07 section 12.4 recommends a second entry point instead, because the
  /// preconditions really are different (a start object is required) and
  /// because a mode field would make [`Router::start_routing`] fallible
  /// in a way its signature does not show. The recording gains one
  /// variant rather than a mode event.
  ///
  /// Which two nets are routed is entirely the host's answer, through
  /// [`crate::rules::RuleResolver::dp_net_pair`] on the start object.
  /// KiCad's own answer is a net **name** convention
  /// (`BOARD::MatchDpSuffix`, `pcbnew/board.cpp:2780`); this crate has no
  /// net names and invents no convention.
  ///
  /// # Errors
  ///
  /// [`StartError::AlreadyRouting`], everything
  /// [`Router::is_starting_point_routable_diff_pair`] refuses, and
  /// [`StartError::PlacerRefused`].
  pub fn start_routing_diff_pair(
    &mut self,
    at: Vec2,
    start: Option<HostId>,
    layer: i32,
  ) -> Result<PreviewFrame, StartError> {
    self.record(SessionEvent::StartRoutingDiffPair { at, start, layer });

    if self.routing_in_progress() {
      return Err(StartError::AlreadyRouting);
    }

    self.is_starting_point_routable_diff_pair(at, start, layer)?;

    let root = self.world.root();
    let host = start.ok_or(StartError::PairNeedsStartItem)?;
    let start_item = self
      .resolve_host_item(root, at, host, Some(layer))
      .ok_or(StartError::UnknownStartItem(host))?;

    // :443 to :471, in KiCad's order.
    let mut placer = DiffPairPlacer::new(&self.world, root, self.sizes.clone());
    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    placer.set_layer(&mut self.world, &context, layer);

    // :473
    placer.start(&mut self.world, &context, at, Some(start_item))?;

    self.placer = Some(Placer::DiffPair(Box::new(placer)));
    self.state = RouterState::RouteTrack;

    Ok(self.frame())
  }

  /// Begin length tuning the track under a point.
  ///
  /// Port of `ROUTER::StartRouting` (`pcbnew/router/pns_router.cpp:434`)
  /// for `PNS_MODE_TUNE_SINGLE`, which news a `MEANDER_PLACER` (`:453`)
  /// and then runs the same four calls it runs for a routing placer
  /// (`:467` to `:470`). Two of those four, `UpdateSizes` and `SetLayer`,
  /// are no ops on a meander placer, so the sizes and the layer the host
  /// just imported are dropped on the floor there; neither is in this
  /// signature for that reason.
  ///
  /// A third entry point rather than a mode field, for the reason note 07
  /// section 12.4 gives and [`Router::start_routing_diff_pair`] repeats:
  /// the preconditions really are different. A tuning session needs the
  /// object to tune and no layer at all, because `CurrentLayer()` is read
  /// off the clicked segment (`pcbnew/router/pns_meander_placer.cpp:435`).
  ///
  /// The settings arrive here rather than through a setter because KiCad's
  /// host pushes a whole `MEANDER_SETTINGS` into the placer before every
  /// move (`pcbnew/generators/pcb_tuning_pattern.cpp:1298`) and there is
  /// no sensible default for the one field that matters, the target
  /// length. [`Router::amplitude_step`] and [`Router::spacing_step`] are
  /// the two live adjustments the host binds to keys; the flip the shape
  /// generator makes to the initial side comes back on
  /// [`TuningInfo::settings`].
  ///
  /// # No start gate
  ///
  /// `isStartingPointRoutable` (`pcbnew/router/pns_router.cpp:294`) has
  /// no arm for any of the three tuning modes, so none of the refusals
  /// [`Router::is_starting_point_routable`] makes applies here. The only
  /// gate is the placer's own `Start`.
  ///
  /// # Errors
  ///
  /// [`StartError::AlreadyRouting`],
  /// [`StartError::UnknownStartItem`] for an object the snapshot does not
  /// describe, and everything
  /// [`crate::placer::meander_placer::TuningError`] carries.
  pub fn start_tuning(
    &mut self,
    at: Vec2,
    item: HostId,
    settings: MeanderSettings,
  ) -> Result<PreviewFrame, StartError> {
    self.record(SessionEvent::StartTuning { at, item, settings });

    if self.routing_in_progress() {
      return Err(StartError::AlreadyRouting);
    }

    let root = self.world.root();
    let start_item = self
      .resolve_host_item(root, at, item, None)
      .ok_or(StartError::UnknownStartItem(item))?;

    // :453
    let mut placer = MeanderPlacer::new(&self.world, root, settings);
    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    // :473
    placer.start(&mut self.world, &context, at, Some(start_item))?;

    self.placer = Some(Placer::Meander(Box::new(placer)));
    self.state = RouterState::TuneSingle;

    Ok(self.frame())
  }

  /// Begin length tuning the differential pair a track belongs to.
  ///
  /// Port of `ROUTER::StartRouting` (`pcbnew/router/pns_router.cpp:434`)
  /// for `PNS_MODE_TUNE_DIFF_PAIR`, which news a `DP_MEANDER_PLACER`
  /// (`:457`). Everything [`Router::start_tuning`] says about the
  /// signature holds: no layer, no sizes, no start gate, the settings in
  /// the call.
  ///
  /// Which two nets are coupled is entirely the host's answer, through
  /// [`crate::rules::RuleResolver::dp_coupled_net`] and
  /// [`crate::rules::RuleResolver::dp_net_polarity`], exactly as it is
  /// for [`Router::start_routing_diff_pair`]. A host that answers neither
  /// sees [`StartError::NotADiffPairForTuning`].
  ///
  /// # Why a third entry point and not a mode argument
  ///
  /// Note 08 section 11.5 recommends one entry point with a mode, on the
  /// grounds that all three tuning modes take the same arguments. Three
  /// separate entries are taken instead, for the reason note 07 section
  /// 12.4 gave for pairs and [`Router::start_routing_diff_pair`] repeats:
  /// the refusals differ (only the two pair modes can answer
  /// [`StartError::NotADiffPairForTuning`] or
  /// [`StartError::NotADiffPairForSkew`]), the recording then carries one
  /// event per mode so a reader is never in doubt about which placer ran,
  /// and [`Router::start_tuning`] keeps the shape it shipped with.
  ///
  /// # Errors
  ///
  /// [`StartError::AlreadyRouting`], [`StartError::UnknownStartItem`],
  /// and everything
  /// [`crate::placer::meander_placer::TuningError`] carries.
  pub fn start_tuning_diff_pair(
    &mut self,
    at: Vec2,
    item: HostId,
    settings: MeanderSettings,
  ) -> Result<PreviewFrame, StartError> {
    self.record(SessionEvent::StartTuningDiffPair { at, item, settings });

    if self.routing_in_progress() {
      return Err(StartError::AlreadyRouting);
    }

    let root = self.world.root();
    let start_item = self
      .resolve_host_item(root, at, item, None)
      .ok_or(StartError::UnknownStartItem(item))?;

    // :457
    let mut placer = DpMeanderPlacer::new(
      &self.world,
      root,
      settings,
      self.sizes.diff_pair_gap,
    );
    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    // :473
    placer.start(&mut self.world, &context, at, Some(start_item))?;

    self.placer = Some(Placer::DpMeander(Box::new(placer)));
    self.state = RouterState::TuneDiffPair;

    Ok(self.frame())
  }

  /// Begin skew tuning the lane of a differential pair under a point.
  ///
  /// Port of `ROUTER::StartRouting` (`pcbnew/router/pns_router.cpp:434`)
  /// for `PNS_MODE_TUNE_DIFF_PAIR_SKEW`, which news a
  /// `MEANDER_SKEW_PLACER` (`:461`).
  ///
  /// # It meanders the lane that was clicked
  ///
  /// There is no automatic choice of the shorter lane anywhere in
  /// KiCad's placer, and none here: the clicked lane is the one that gets
  /// meandered and the other one is only measured. Clicking the **longer**
  /// lane and asking for zero skew therefore reports
  /// [`crate::meander::TuningStatus::TooLong`] and changes nothing; the
  /// user's remedy is to click the other lane. Note 08 section 7.1.
  ///
  /// The readout is a skew: [`TuningInfo::result`] and
  /// [`TuningInfo::skew`] are the same number, and
  /// [`TuningInfo::mode`] is what tells a host to label it as one.
  ///
  /// # Errors
  ///
  /// [`StartError::AlreadyRouting`], [`StartError::UnknownStartItem`],
  /// and everything
  /// [`crate::placer::meander_placer::TuningError`] carries, with
  /// [`StartError::NotADiffPairForSkew`] in place of
  /// [`StartError::NotADiffPairForTuning`].
  pub fn start_tuning_skew(
    &mut self,
    at: Vec2,
    item: HostId,
    settings: MeanderSettings,
  ) -> Result<PreviewFrame, StartError> {
    self.record(SessionEvent::StartTuningSkew { at, item, settings });

    if self.routing_in_progress() {
      return Err(StartError::AlreadyRouting);
    }

    let root = self.world.root();
    let start_item = self
      .resolve_host_item(root, at, item, None)
      .ok_or(StartError::UnknownStartItem(item))?;

    // :461
    let mut placer = MeanderSkewPlacer::new(
      &self.world,
      root,
      settings,
      self.sizes.diff_pair_gap,
    );
    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    // :473
    placer.start(&mut self.world, &context, at, Some(start_item))?;

    self.placer = Some(Placer::MeanderSkew(Box::new(placer)));
    self.state = RouterState::TuneSkew;

    Ok(self.frame())
  }

  /// Nudge the meander amplitude by one step and re-run the last move.
  ///
  /// Port of `PCB_ACTIONS::amplIncrease` and `amplDecrease`
  /// (`pcbnew/generators/pcb_tuning_pattern.cpp:2785`), which call
  /// `MEANDER_PLACER_BASE::AmplitudeStep( +/-1 )`
  /// (`pcbnew/router/pns_meander_placer_base.cpp:96`), copy the new
  /// maximum amplitude back onto the board item and then re-run `Update`,
  /// which is a fresh `Move`.
  ///
  /// The re-run is the caller's here: this changes the settings and
  /// answers whether it did, and the host follows with a
  /// [`Router::move_to`] at the same point. `sign` is a direction and not
  /// a distance; the distance is [`MeanderSettings::step`].
  ///
  /// False when no tuning session is running, which is every routing and
  /// dragging state.
  pub fn amplitude_step(&mut self, sign: i32) -> bool {
    self.record(SessionEvent::AmplitudeStep { sign });

    if !self.state.is_tuning() {
      return false;
    }

    let Some(placer) = self.placer.as_mut() else {
      return false;
    };

    placer.amplitude_step(sign);

    true
  }

  /// Nudge the meander spacing by one step and re-run the last move.
  ///
  /// Port of `PCB_ACTIONS::spacingIncrease` and `spacingDecrease`
  /// (`pcbnew/generators/pcb_tuning_pattern.cpp:2764`), which call
  /// `MEANDER_PLACER_BASE::SpacingStep( +/-1 )`
  /// (`pcbnew/router/pns_meander_placer_base.cpp:105`). See
  /// [`Router::amplitude_step`] for the shape of both.
  ///
  /// The new spacing is floored by the tuned track's width plus its
  /// clearance, so a decrease can be refused by the floor and still
  /// answer true: the answer is "a tuning session took this", not "the
  /// value changed".
  pub fn spacing_step(&mut self, sign: i32) -> bool {
    self.record(SessionEvent::SpacingStep { sign });

    if !self.state.is_tuning() {
      return false;
    }

    let Some(placer) = self.placer.as_mut() else {
      return false;
    };

    placer.spacing_step(sign);

    true
  }

  /// Begin dragging an existing object.
  ///
  /// Port of `ROUTER::StartDragging( aP, ITEM_SET, aDragMode )`
  /// (`pcbnew/router/pns_router.cpp:166`). The single item overload
  /// (`:159`) is a one line forward to this one, and the default drag
  /// mode differs between the two declarations without either mattering,
  /// so there is one method here.
  ///
  /// # Why `free_angle` and not a mode mask
  ///
  /// `DRAGGER::Start` reads exactly one bit out of KiCad's mask,
  /// `DM_FREE_ANGLE` (`pcbnew/router/pns_dragger.cpp:314`), and then
  /// `startDragSegment` and `startDragVia` overwrite `m_mode` from the
  /// clicked item's kind and the click position. `DM_CORNER`,
  /// `DM_SEGMENT`, `DM_VIA` and `DM_ARC` in the request are therefore
  /// never consulted, which is note 06 erratum E2 and which is why the
  /// QA log player passes a plain `0`
  /// (`qa/tools/pns/pns_log_player.cpp:169`). Read the mode back off
  /// [`Dragger::mode`] instead.
  ///
  /// # Which algorithm the item set picks
  ///
  /// KiCad dispatches on the **shape of the set** and not on the mode
  /// (`:176` to `:190`), and the three tests are in this order:
  ///
  /// 1. every item is a solid, which is a component drag and reaches
  ///    [`ComponentDragger`], leaving the router in
  ///    [`RouterState::DragComponent`];
  /// 2. more than one item is a segment or an arc, which is a multi drag
  ///    and reaches [`MultiDragger`];
  /// 3. anything else reaches [`Dragger`], which then reads
  ///    `aPrimitives[0]` and nothing else
  ///    (`pcbnew/router/pns_dragger.cpp:309`). So a segment plus a via is
  ///    a single drag of the segment, and the via is ignored.
  ///
  /// The kind test at `:182` is over a **mask**, so two segments, two
  /// arcs or one of each all reach the multi dragger. Every shape of a
  /// non empty set therefore has an algorithm, which is why there is no
  /// "unsupported set" error.
  ///
  /// `free_angle` reaches the single dragger only.
  /// `MULTI_DRAGGER::SetMode` has an empty body
  /// (`pcbnew/router/pns_multi_dragger.cpp:284`) and
  /// `COMPONENT_DRAGGER` does not override `SetMode` at all, so it takes
  /// `DRAG_ALGO`'s empty default (`pcbnew/router/pns_drag_algo.h:119`);
  /// that is note 06 errata E2 and E21.
  ///
  /// `GetRuleResolver()->ClearCaches()` (`:174`) has no counterpart: a
  /// [`RuleResolver`] here owns whatever caching it does and the facade
  /// never invalidates it.
  ///
  /// KiCad's host follows a successful start with a `Move`, so the frame
  /// returned here is the state of a drag that has not moved yet:
  /// `CurrentNode()` is still the untouched board
  /// (`pcbnew/router/pns_dragger.cpp:1052`) and the frame is empty.
  ///
  /// # Errors
  ///
  /// [`StartError::AlreadyRouting`], [`StartError::NothingToDrag`],
  /// [`StartError::UnknownStartItem`] for an object the snapshot does not
  /// describe, and [`StartError::NotDraggable`] for one the dragger
  /// refuses.
  pub fn start_dragging(
    &mut self,
    at: Vec2,
    items: &[HostId],
    free_angle: bool,
  ) -> Result<PreviewFrame, StartError> {
    self.record(SessionEvent::StartDragging {
      at,
      items: items.to_vec(),
      free_angle,
    });

    if self.routing_in_progress() {
      return Err(StartError::AlreadyRouting);
    }

    // :168
    self.leader_segments.clear();

    // :171
    if items.is_empty() {
      return Err(StartError::NothingToDrag);
    }

    let root = self.world.root();
    // A drag names no layer, so any item of that host object will do; the
    // dragger reads the kind and the geometry off it.
    let mut resolved = Vec::with_capacity(items.len());

    for host in items {
      resolved.push(
        self
          .resolve_host_item(root, at, *host, None)
          .ok_or(StartError::UnknownStartItem(*host))?,
      );
    }

    let count_of = |mask: Kind| {
      resolved
        .iter()
        .filter(|id| {
          self.world.item(**id).is_some_and(|item| item.of_kind(mask))
        })
        .count()
    };

    let solids = count_of(Kind::SOLID);
    let tracks = count_of(Kind::SEGMENT | Kind::ARC);
    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };
    // :187, :194. Every dragger branches from the **root**, not from a
    // branch of it, unlike the placer.
    let started = if solids == resolved.len() {
      // :176. All solids wins over the segment count below it.
      let mut dragger = ComponentDragger::new(&self.world, root);
      let started = dragger.start(&self.world, &context, at, &resolved);

      self.dragger = Some(ActiveDragger::Component(Box::new(dragger)));
      self.state = RouterState::DragComponent;
      started
    } else if tracks > 1 {
      // :182
      let mut dragger = MultiDragger::new(&self.world, root);
      let started = dragger.start(&mut self.world, &context, at, &resolved);

      self.dragger = Some(ActiveDragger::Multi(Box::new(dragger)));
      self.state = RouterState::DragSegment;
      started
    } else {
      // :187
      let mut dragger = Dragger::new(&self.world, root);

      // :193
      dragger.set_free_angle_mode(free_angle);

      // :209. `aPrimitives[0]` and nothing else.
      let started = dragger.start(&mut self.world, &context, at, resolved[0]);

      self.dragger = Some(ActiveDragger::Single(Box::new(dragger)));
      self.state = RouterState::DragSegment;
      started
    };

    if !started {
      // :213. KiCad drops the dragger and goes back to `IDLE`.
      self.dragger = None;
      self.state = RouterState::Idle;

      return Err(StartError::NotDraggable(resolved[0]));
    }

    Ok(self.frame())
  }

  /// The segments a host should put a multi drag's selection back on.
  ///
  /// Port of `ROUTER::GetLastCommittedLeaderSegments`
  /// (`pcbnew/router/pns_router.cpp:940`), which
  /// `ROUTER_TOOL::performDragging` reads after the fix and turns into a
  /// selection of `lseg->Parent()`
  /// (`pcbnew/router/router_tool.cpp:3164`, `:3238`).
  ///
  /// A multi drag deletes the segments the user had selected and makes
  /// new ones, so this is how the host learns which new ones stand for
  /// the old selection. Only [`MultiDragger`] contributes; a single drag
  /// always answers with nothing, as KiCad's default body does
  /// (`pcbnew/router/pns_drag_algo.h:125`).
  ///
  /// The ids name items of the drag's own node while the gesture runs.
  /// [`Router::fix_route`] folds that node into the board rather than
  /// rebuilding it, so the same ids stay valid afterwards and
  /// [`Router::host_of`] answers for them once the host has reported its
  /// own ids back through [`Router::assign_host_ids`].
  pub fn last_committed_leader_segments(&self) -> &[ItemId] {
    &self.leader_segments
  }

  /// Which host object an engine item belongs to.
  ///
  /// Port of `ITEM::Parent()` (`pcbnew/router/pns_item.h:196`) as far as
  /// this crate has one: an item the snapshot described answers the id it
  /// came in with, and an item this session created answers only after
  /// [`Router::assign_host_ids`] has been told what the host called it.
  ///
  /// It is what turns [`Router::last_committed_leader_segments`] into a
  /// selection.
  pub fn host_of(&self, item: ItemId) -> Option<HostId> {
    self.index.host_of(item)
  }

  /// Move the end of the route.
  ///
  /// Port of `ROUTER::Move` (`pcbnew/router/pns_router.cpp:494`) through
  /// its `movePlacing` branch (`:789`): the view is erased, the placer
  /// reroutes, the route and its via are drawn as the head trace, and
  /// `updateView` adds the node delta and the violation markers on top.
  /// All of that is the returned frame.
  ///
  /// `at` is already snapped: snapping is host work and every one of
  /// KiCad's own call sites passes the snapped point (note 05 section
  /// 4.5). An idle router answers with an empty frame, which is the
  /// `default:` branch of `ROUTER::Move` (`:508`).
  ///
  /// # The drag branch
  ///
  /// `ROUTER::Move` sends [`RouterState::DragSegment`] to `moveDragging`
  /// (`:504`, `:656`), which erases the view, drags, and calls
  /// `updateView( m_dragger->CurrentNode(), dragged, true )`. Two things
  /// differ from the placing branch and both survive into the frame:
  /// `moveDragging` never sets `PNS_HEAD_TRACE`, so the dragged geometry
  /// reaches the host as an ordinary added item of the node delta rather
  /// than as [`PreviewStyle::Head`], and `aEndItem` is accepted and never
  /// used (note 06 erratum E14). The argument stays in the signature
  /// because one facade method serves both states and the event log
  /// records it either way.
  pub fn move_to(&mut self, at: Vec2, end: Option<HostId>) -> PreviewFrame {
    self.record(SessionEvent::MoveTo { at, end });

    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    match self.state {
      // :508
      RouterState::Idle => return PreviewFrame::default(),
      // :501. A tuning session takes the same branch, exactly as
      // `ROUTER::Move` does (`:502`).
      RouterState::RouteTrack
      | RouterState::TuneSingle
      | RouterState::TuneDiffPair
      | RouterState::TuneSkew => {
        let end_item = self.resolve_end_item(at, end);

        if let Some(placer) = self.placer.as_mut() {
          placer.move_to(&mut self.world, &context, at, end_item);
        }
      }
      // :504, :506, :658. `ROUTER::Move` routes both drag states to
      // `moveDragging`.
      RouterState::DragSegment | RouterState::DragComponent => {
        if let Some(dragger) = self.dragger.as_mut() {
          dragger.drag(&mut self.world, &context, at);
        }

        // :663. Only a multi drag ever answers with anything.
        let leaders = self.dragger.as_ref().map_or_else(Vec::new, |dragger| {
          dragger.last_committed_leader_segments().to_vec()
        });

        self.leader_segments = leaders;
      }
    }

    self.frame()
  }

  /// Pin the route down at a point.
  ///
  /// Port of `ROUTER::FixRoute` (`pcbnew/router/pns_router.cpp:915`)
  /// through both of its live branches, plus the `CommitRouting()` its
  /// host performs when the placer reports that the route reached its
  /// target (note 03 section 1.6).
  ///
  /// An idle router answers `Continue` with an empty frame, which is the
  /// `default:` branch of `ROUTER::FixRoute` (`:933`).
  ///
  /// # `force_finish` carries KiCad's two flags, because they never meet
  ///
  /// KiCad's signature is
  /// `FixRoute( aP, aEndItem, aForceFinish, aForceCommit )` and the
  /// switch drops one of the two on each branch: the placer is given
  /// `aForceFinish` and never sees `aForceCommit` (`:926`), the dragger is
  /// given `aForceCommit` and never sees `aP`, `aEndItem` or
  /// `aForceFinish` (`:930`). Note 06 section 9.4 suggests a fourth
  /// argument; one flag is taken here instead, so that every routing call
  /// site keeps its three arguments and the recorded
  /// [`SessionEvent::FixRoute`] and every stored fixture keep their
  /// shape. There is no state in which both meanings apply, so nothing is
  /// conflated.
  ///
  /// For the placer it is `aForceFinish`, which makes the fix terminal
  /// wherever it lands (`pcbnew/router/pns_line_placer.cpp:1647`). For
  /// the dragger it is `aForceCommit`, the Ctrl+click that commits a
  /// drag the rules refuse, which
  /// [`crate::dragger::Dragger::fix_route_node`] honours **only** in
  /// forced mark obstacles mode (note 06 erratum E7).
  ///
  /// # A drag fix is always terminal
  ///
  /// `DRAGGER::FixRoute` either commits or refuses; there is no "one more
  /// corner, keep going" for a drag. So the drag branch answers
  /// [`FixOutcome::Finished`] with the commit, or
  /// [`FixOutcome::Continue`] with the unchanged frame when the fix was
  /// refused and the gesture is still running.
  pub fn fix_route(
    &mut self,
    at: Vec2,
    end: Option<HostId>,
    force_finish: bool,
  ) -> FixOutcome {
    self.record(SessionEvent::FixRoute {
      at,
      end,
      force_finish,
    });

    match self.state {
      // :933
      RouterState::Idle => FixOutcome::Continue(PreviewFrame::default()),
      // :922. All three tuning modes take it too, because KiCad leaves
      // every one of them in `ROUTE_TRACK`.
      RouterState::RouteTrack
      | RouterState::TuneSingle
      | RouterState::TuneDiffPair
      | RouterState::TuneSkew => self.fix_placement(at, end, force_finish),
      // :928, :929. Both drag states reach the dragger's `FixRoute`.
      RouterState::DragSegment | RouterState::DragComponent => {
        self.fix_drag(force_finish)
      }
    }
  }

  /// The `ROUTE_TRACK` branch of [`Router::fix_route`].
  fn fix_placement(
    &mut self,
    at: Vec2,
    end: Option<HostId>,
    force_finish: bool,
  ) -> FixOutcome {
    let end_item = self.resolve_end_item(at, end);
    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };
    let mut reached = false;

    if let Some(placer) = self.placer.as_mut() {
      reached =
        placer.fix_route(&mut self.world, &context, at, end_item, force_finish);
    }

    if reached {
      // The fix is the event; the commit it reaches is not a second one.
      self.suspend_recording();

      let diff = self.stop_routing();

      self.resume_recording();

      FixOutcome::Finished(diff)
    } else {
      FixOutcome::Continue(self.frame())
    }
  }

  /// The `DRAG_SEGMENT` branch of [`Router::fix_route`].
  ///
  /// `DRAGGER::FixRoute` (`pcbnew/router/pns_dragger.cpp:957`) commits
  /// through `Router()->CommitRouting( node )`; here the dragger answers
  /// which node it would commit and the facade does the committing, so
  /// that the [`CommitDiff`] can be built before the node is folded away.
  fn fix_drag(&mut self, force_commit: bool) -> FixOutcome {
    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };
    let mut node = None;

    if let Some(dragger) = self.dragger.as_mut() {
      node = dragger.fix_route_node(&mut self.world, &context, force_commit);
    }

    match node {
      Some(node) => FixOutcome::Finished(self.commit_drag(node)),
      None => FixOutcome::Continue(self.frame()),
    }
  }

  /// Route the rest of the way and finish.
  ///
  /// Port of `ROUTER::Finish` (`pcbnew/router/pns_router.cpp:569`): move
  /// at the nearest unconnected anchor up to five times, stopping early
  /// once the route's end stops changing, and fix there when the end
  /// settled on the anchor and the layers overlap. The retry loop exists
  /// because one move may get partially stuck and a second attempt from
  /// the new state can get further.
  ///
  /// [`None`] when nothing is being routed, when nothing unconnected is
  /// left to reach, or when the route did not settle on the anchor.
  ///
  /// # Deviation
  ///
  /// The anchor reaches [`Router::fix_route`] as a host id, so an anchor
  /// on something this session itself created is passed as no end item at
  /// all and the fix is not terminal. KiCad hands the item pointer
  /// straight back (`:608`) and has no such gap; it closes as soon as the
  /// host reports its ids back through [`Router::assign_host_ids`].
  pub fn finish(&mut self) -> Option<FixOutcome> {
    self.record(SessionEvent::Finish);

    if self.state != RouterState::RouteTrack {
      return None;
    }

    let anchor = self.nearest_ratsnest_anchor()?;
    let host = anchor.item.and_then(|id| self.index.host_of(id));

    // :594. Five tries, stopping as soon as the end stops moving.
    let mut settled = self.placer.as_ref().and_then(Placer::current_end);

    // This routine picks its own anchor, so it is one event and the moves
    // and the fix it drives are not recorded on their own.
    self.suspend_recording();

    for _ in 0..5 {
      settled = self.placer.as_ref().and_then(Placer::current_end);
      self.move_to(anchor.at, host);

      if self.placer.as_ref().and_then(Placer::current_end) == settled {
        break;
      }
    }

    // :603
    let overlaps = self
      .current_layer()
      .is_some_and(|layer| anchor.layers.contains(layer));

    if settled != Some(anchor.at) || !overlaps {
      self.resume_recording();

      return None;
    }

    let outcome = self.fix_route(anchor.at, host, false);

    self.resume_recording();

    Some(outcome)
  }

  /// Commit what is routed and start again from the far end.
  ///
  /// Port of `ROUTER::ContinueFromEnd`
  /// (`pcbnew/router/pns_router.cpp:617`): remember the layer and the
  /// end, find the far anchor, commit and stop, start again at the anchor
  /// on the current layer when its layers reach it and on its own first
  /// layer otherwise, then move back towards where the user was.
  ///
  /// [`None`] when nothing is being routed and when nothing unconnected
  /// is left, in which case nothing has been committed either. Once the
  /// anchor is known the commit happens whatever comes next, because
  /// KiCad commits before it knows whether the restart will succeed
  /// (`:633`); a [`ContinueOutcome`] whose
  /// [`ContinueOutcome::frame`] is [`None`] is that case, and the host
  /// still has to apply the diff.
  pub fn continue_from_end(&mut self) -> Option<ContinueOutcome> {
    self.record(SessionEvent::ContinueFromEnd);

    if self.state != RouterState::RouteTrack {
      return None;
    }

    let anchor = self.nearest_ratsnest_anchor()?;
    let host = anchor.item.and_then(|id| self.index.host_of(id));
    let layer = self.current_layer()?;
    let end = self.placer.as_ref().and_then(Placer::current_end)?;

    // This routine picks its own anchor, so it is one event and the
    // commit, the restart and the move it drives are not recorded on
    // their own.
    self.suspend_recording();

    // :633
    let diff = self.stop_routing();

    // :636
    let next_layer = if anchor.layers.contains(layer) {
      layer
    } else {
      anchor.layers.start()
    };

    let restarted = self.start_routing(anchor.at, host, next_layer).is_ok();
    let outcome = ContinueOutcome {
      diff,
      // :643
      frame: restarted.then(|| self.move_to(end, None)),
      start: host,
    };

    self.resume_recording();

    Some(outcome)
  }

  /// Undo the last fix.
  ///
  /// Port of `ROUTER::UndoLastSegment`
  /// (`pcbnew/router/pns_router.cpp:946`), which forwards to
  /// `LINE_PLACER::UnfixRoute`. The answer is where the undone leg began,
  /// which a host uses to warp the cursor back there
  /// (`pcbnew/router/router_tool.cpp:1865`).
  ///
  /// KiCad dereferences its placer after checking only
  /// `RoutingInProgress()`, so a dragging session would crash there; this
  /// answers [`None`].
  pub fn undo_last_segment(&mut self) -> Option<Vec2> {
    self.record(SessionEvent::UndoLastSegment);

    if self.state != RouterState::RouteTrack {
      return None;
    }

    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    self
      .placer
      .as_mut()?
      .undo_last_segment(&mut self.world, &context)
  }

  // -----------------------------------------------------------------
  // Small commands
  // -----------------------------------------------------------------

  /// Move the route to another copper layer.
  ///
  /// Port of `ROUTER::SwitchLayer` (`pcbnew/router/pns_router.cpp:1010`),
  /// which forwards to the placer only while routing. The placer refuses
  /// once the placement is chained, that is once a fix ended a leg
  /// without leaving a via behind
  /// (`pcbnew/router/pns_line_placer.cpp:1354`); a host that wants
  /// KiCad's key binding toggles a via, fixes, and then switches
  /// (`pcbnew/router/pns_router.cpp:1010`).
  ///
  /// # Deviation
  ///
  /// A layer outside the board is refused here. KiCad has no such check
  /// because its host only ever offers layers the board has, and
  /// [`crate::snapshot::WorldSnapshot::copper_layer_count`] is what makes
  /// the check possible at all.
  pub fn switch_layer(&mut self, layer: i32) -> bool {
    self.record(SessionEvent::SwitchLayer { layer });

    if self.state != RouterState::RouteTrack {
      return false;
    }

    if layer < 0 || layer >= i32::from(self.copper_layer_count) {
      return false;
    }

    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };
    let Some(placer) = self.placer.as_mut() else {
      return false;
    };

    placer.set_layer(&mut self.world, &context, layer)
  }

  /// Arm or disarm the via the next fix would place.
  ///
  /// Port of `ROUTER::ToggleViaPlacement`
  /// (`pcbnew/router/pns_router.cpp:1019`), which reads the current state
  /// and inverts it. The answer is whether the request was honoured, not
  /// the new state; read that back with [`Router::placing_via`].
  ///
  /// The via is only materialised on the next move
  /// (`pcbnew/router/pns_line_placer.cpp:2105`), so a host has to move
  /// before the preview shows it.
  pub fn toggle_via_placement(&mut self) -> bool {
    self.record(SessionEvent::ToggleViaPlacement);

    if self.state != RouterState::RouteTrack {
      return false;
    }

    let Some(placer) = self.placer.as_mut() else {
      return false;
    };
    let armed = !placer.is_placing_via();

    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    placer.toggle_via(&mut self.world, &context, armed)
  }

  /// Turn the route's first corner the other way.
  ///
  /// Port of `ROUTER::FlipPosture`
  /// (`pcbnew/router/pns_router.cpp:1001`), which forwards to the placer
  /// only while routing.
  pub fn flip_posture(&mut self) {
    self.record(SessionEvent::FlipPosture);

    if self.state != RouterState::RouteTrack {
      return;
    }

    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    if let Some(placer) = self.placer.as_mut() {
      placer.flip_posture(&mut self.world, &context);
    }
  }

  /// Cycle the corner mode.
  ///
  /// Port of `ROUTER::ToggleCornerMode`
  /// (`pcbnew/router/pns_router.cpp:1069`), which writes the new mode
  /// back into the settings object. The cycle is KiCad's,
  /// `MITERED_45 -> ROUNDED_45 -> MITERED_90 -> ROUNDED_90 ->
  /// MITERED_45` (`:1075` to `:1078`).
  ///
  /// A host that cannot store an arc track has to keep the user off this
  /// cycle, or refuse the two rounded modes where it applies the
  /// settings; see [`CornerMode::is_rounded`].
  pub fn toggle_corner_mode(&mut self) {
    self.record(SessionEvent::ToggleCornerMode);

    self.settings.corner_mode = match self.settings.corner_mode {
      CornerMode::Mitered45 => CornerMode::Rounded45,
      CornerMode::Rounded45 => CornerMode::Mitered90,
      CornerMode::Mitered90 => CornerMode::Rounded90,
      CornerMode::Rounded90 => CornerMode::Mitered45,
    };
  }

  /// Force the head to a single segment.
  ///
  /// Port of `ROUTER::SetOrthoMode`
  /// (`pcbnew/router/pns_router.cpp:1085`), which forwards whenever a
  /// placer exists rather than testing the state.
  pub fn set_ortho_mode(&mut self, ortho: bool) {
    self.record(SessionEvent::SetOrthoMode { ortho });

    if let Some(placer) = self.placer.as_mut() {
      placer.set_ortho_mode(ortho);
    }
  }

  // -----------------------------------------------------------------
  // Ending a session
  // -----------------------------------------------------------------

  /// What the session has changed so far, without committing it.
  ///
  /// Port of `ROUTER::GetUpdatedItems`
  /// (`pcbnew/router/pns_router.cpp:832`): the delta of the node the
  /// placer stands on with loops removed, `CurrentNode( true )` (`:839`),
  /// against the board. An idle router answers with nothing.
  ///
  /// KiCad's third out parameter, the cloned head items, is not returned:
  /// its only consumer deletes them immediately with the comment "fixme:
  /// update the state with the head trace (not supported in current
  /// testsuite)" (`qa/tools/pns/pns_log_player.cpp:83`), and a host that
  /// wants the head already has it in the [`PreviewFrame`].
  ///
  /// The drag branch is `:844`: the node is `m_dragger->CurrentNode()`,
  /// with no "loops removed" flag to pass and no committed node in sight.
  /// That makes this what the KiCad regression corpus compares a drag
  /// case against, because none of the seven drag logs holds an
  /// `EVT_FIX`; see note 06 section 10.1.
  ///
  /// # The one addition
  ///
  /// [`RouterState::DragComponent`] takes the same branch, which KiCad
  /// does **not** have: its `switch` handles `ROUTE_TRACK` (`:839`) and
  /// `DRAG_SEGMENT` (`:844`) and falls through for `DRAG_COMPONENT`, so
  /// a component drag reports an empty delta to everything that asks,
  /// the QA log player included. That is note 06 erratum E15, and it is
  /// repaired rather than reproduced: an empty answer here is not a
  /// behaviour a host could want, and the branch it belongs in is
  /// already written.
  ///
  /// See [`PendingUpdate`] for why this is not a [`CommitDiff`].
  pub fn pending_update(&self) -> PendingUpdate {
    let node = match self.state {
      RouterState::Idle => None,
      // :839
      RouterState::RouteTrack
      | RouterState::TuneSingle
      | RouterState::TuneDiffPair
      | RouterState::TuneSkew => {
        self.placer.as_ref().map(|placer| placer.current_node(true))
      }
      // :844, plus the `DRAG_COMPONENT` branch KiCad lacks.
      RouterState::DragSegment | RouterState::DragComponent => {
        self.dragger.as_ref().map(ActiveDragger::current_node)
      }
    };
    let Some(node) = node else {
      return PendingUpdate::default();
    };
    let (added, removed) = self.world.get_updated_items(node);

    PendingUpdate {
      removed: removed
        .into_iter()
        .filter_map(|id| self.index.host_of(id))
        .collect(),
      added,
      node: Some(node),
    }
  }

  /// Commit what was routed and end the session.
  ///
  /// Port of the no argument `ROUTER::CommitRouting`
  /// (`pcbnew/router/pns_router.cpp:958`), which is
  /// `m_placer->CommitPlacement()` followed by `StopRouting()`, and of
  /// `CommitRouting( NODE* )` (`:862`), which is where the diff is built.
  ///
  /// # The remove plus add fold
  ///
  /// `pcbnew/router/pns_router.cpp:877` to `:892`: a removed item that
  /// shares a host object with an added item is not a removal followed by
  /// an addition, it is an update, and reporting it as one is what keeps
  /// the host object's identity, its uuid and its per object attributes
  /// across a route. The one path that produces such a pair today is
  /// `SplitAdjacentSegments`
  /// (`pcbnew/router/pns_line_placer.cpp:1287`): starting or ending a
  /// route in the middle of a track removes that track and adds the two
  /// halves it was cut into.
  ///
  /// KiCad matches on `ITEM::Parent()`, the one to one mapping to a host
  /// object. This matches on [`Item::source`], `GetSourceItem`, because
  /// the halves inherit the source and are deliberately left with no
  /// parent of their own: a half is not the track, it is one piece of it.
  /// The two fields agree for every board item
  /// ([`Item::set_provenance`] writes both), so the fold finds the same
  /// pairs, and the surviving half is the first one in uid order where
  /// KiCad's is the first in address order.
  ///
  /// A host applies the three lists in one undo transaction and reports
  /// the ids it gave the additions back through
  /// [`Router::assign_host_ids`].
  ///
  /// `StopRouting` also pushes the touched nets to the host so it can
  /// rebuild the ratsnest (`:971`); a host here reads them off the diff.
  ///
  /// # A drag commits nothing here
  ///
  /// `ROUTER::CommitRouting()` commits only inside
  /// `if( m_state == ROUTE_TRACK )` (`:960`) and `StopRouting` itself
  /// only tears the session down (`:967`), so a drag that was never fixed
  /// is discarded whichever of the two the host calls. That is what
  /// [`Router::fix_route`] is for: it is the only path that commits a
  /// drag, exactly as `DRAGGER::FixRoute` is in KiCad. So this answers an
  /// empty [`CommitDiff`] for a drag and drops the dragger, and
  /// [`Router::abort_routing`] does the same thing without the recorded
  /// event.
  pub fn stop_routing(&mut self) -> CommitDiff {
    self.record(SessionEvent::StopRouting);

    // :960. `build_commit_plan` answers an empty plan without a placer,
    // which is the drag and the idle case both.
    let plan = self.build_commit_plan();

    // :959, the placer's own commit, which folds its scratch branch into
    // the root through `World::commit`.
    if let Some(placer) = self.placer.as_mut() {
      placer.commit_placement(&mut self.world);
    }

    self.apply_commit_plan(plan)
  }

  /// Fold a drag's node into the board and end the session.
  ///
  /// The `Router()->CommitRouting( node )` of `DRAGGER::FixRoute`
  /// (`pcbnew/router/pns_dragger.cpp:965`), which is
  /// `CommitRouting( NODE* )` (`pcbnew/router/pns_router.cpp:862`) over
  /// the dragger's node and then, in the host,
  /// `ROUTER_TOOL::performDragging`'s `StopRouting`
  /// (`pcbnew/router/router_tool.cpp:2591`).
  ///
  /// The plan is built before [`World::commit`] runs, because the commit
  /// takes the removed items out of the arena and their handles go stale.
  fn commit_drag(&mut self, node: NodeId) -> CommitDiff {
    let plan = self.commit_plan_for(node);

    self.world.commit(node);

    self.apply_commit_plan(plan)
  }

  /// Fix the host map up after a commit and end the session.
  ///
  /// The tail both [`Router::stop_routing`] and [`Router::commit_drag`]
  /// end with, once the node they were committing has been folded into
  /// the board.
  fn apply_commit_plan(&mut self, plan: CommitPlan) -> CommitDiff {
    self.last_commit_error = plan.error;

    // The removed items are out of the board now, and each updated one
    // has taken over the identity of the item it replaced.
    for id in plan.removed {
      self.index.remove_item(id);
    }

    for (position, id) in plan.updated.iter().enumerate() {
      let Some((host, _)) = plan.diff.updated.get(position) else {
        continue;
      };

      if let Some(item) = self.world.item_mut(*id) {
        item.set_provenance(Provenance::Board(*host));
      }

      self.index.insert(*host, *id);
    }

    // A moved pad keeps its host object: the old solid is gone from the
    // board and the clone stands for the same pad at its new position.
    for (host, moved) in plan.moved {
      self.index.remove_item(moved.before);
      self.index.insert(host, moved.after);
    }

    self.committed = plan.added;
    self.finish_session();
    self.record_commit(&plan.diff);

    plan.diff
  }

  /// Throw the session away.
  ///
  /// `LINE_PLACER::AbortPlacement`
  /// (`pcbnew/router/pns_line_placer.cpp:2150`) followed by the second
  /// half of `ROUTER::StopRouting` (`pcbnew/router/pns_router.cpp:967`),
  /// with the commit left out. `ROUTER` never calls `AbortPlacement` in
  /// this revision, because escape commits there; see the module
  /// documentation.
  ///
  /// Dropping the root's children takes every speculative branch with it,
  /// and every item those branches own, so nothing the session built
  /// survives.
  pub fn abort_routing(&mut self) {
    self.record(SessionEvent::AbortRouting);
    self.finish_session();
  }

  /// Tell the engine which ids the host gave the last commit's additions.
  ///
  /// The engine cannot name a segment it has just created: only the host
  /// knows what id its own board object got. Each pair is an index into
  /// the [`CommitDiff::added`] the last [`Router::stop_routing`] returned
  /// and the id that entry was applied as. The item then has a
  /// [`Provenance::Board`] like any other board item, so a later session
  /// can remove or update it and the fold above can preserve it.
  ///
  /// KiCad has no counterpart because its interface writes into the board
  /// as it goes and reads the new `BOARD_ITEM*` straight back
  /// (`pcbnew/router/pns_kicad_iface.cpp:2775`). Returning a diff instead
  /// buys the engine's purity at the price of this one extra step.
  ///
  /// Returns how many pairs were applied; an index past the end of the
  /// last diff is ignored.
  pub fn assign_host_ids(&mut self, ids: &[(usize, HostId)]) -> usize {
    self.record(SessionEvent::AssignHostIds { ids: ids.to_vec() });

    let mut applied = 0;

    for (index, host) in ids {
      let Some(id) = self.committed.get(*index).copied() else {
        continue;
      };

      let Some(item) = self.world.item_mut(id) else {
        continue;
      };

      item.set_provenance(Provenance::Board(*host));
      self.index.insert(*host, id);
      applied += 1;
    }

    applied
  }

  // -----------------------------------------------------------------
  // Internals
  // -----------------------------------------------------------------

  /// The node a query runs against.
  ///
  /// Port of `pcbnew/router/pns_router.cpp:128`: the placer's node while
  /// routing, so a hover sees the speculative world, and the root
  /// otherwise. `PLACEMENT_ALGO::CurrentNode` defaults to "loops not
  /// removed" (`pcbnew/router/pns_placement_algo.h:120`).
  fn query_node(&self) -> NodeId {
    self
      .placer
      .as_ref()
      .map_or_else(|| self.world.root(), |placer| placer.current_node(false))
  }

  /// Whether the start point probe hits anything.
  ///
  /// `m_world->CheckColliding( &dummyStartLine, ITEM::ANY_T )`
  /// (`pcbnew/router/pns_router.cpp:320`), which is the kind masked
  /// overload with a limit of one (`pcbnew/router/pns_node.cpp:493`).
  fn probe_collides(&self, probe: &Line) -> bool {
    let options = CollisionSearchOptions {
      kind_mask: Kind::ANY,
      limit_count: Some(1),
      ..CollisionSearchOptions::default()
    };

    self
      .world
      .check_colliding_line(
        self.world.root(),
        probe,
        self.resolver.as_ref(),
        &options,
      )
      .is_some()
  }

  /// Which engine item a host id means at a point.
  ///
  /// One host object may have become several items, one per padstack
  /// layer or one per stroke; see [`crate::snapshot::WorldItem::id`]. The
  /// one meant is the one the point actually lands on and whose layers
  /// reach `layer`, and failing that the first of that host's items that
  /// reaches the layer, in the order the snapshot listed them.
  fn resolve_host_item(
    &self,
    node: NodeId,
    at: Vec2,
    host: HostId,
    layer: Option<i32>,
  ) -> Option<ItemId> {
    let candidates = self.index.items_of(host);

    if candidates.is_empty() {
      return None;
    }

    let reaches = |id: ItemId| {
      layer.is_none_or(|layer| {
        self
          .world
          .item(id)
          .is_some_and(|item| item.layers().contains(layer))
      })
    };

    for id in self.world.hit_test(node, at) {
      if candidates.contains(&id) && reaches(id) {
        return Some(id);
      }
    }

    candidates.iter().copied().find(|id| reaches(*id))
  }

  /// Which engine item the end of a move means.
  ///
  /// The layer rule is `PNS::TOOL_BASE::updateEndItem`'s
  /// (`pcbnew/router/pns_tool_base.cpp:398`): the routed layer normally,
  /// and any layer while a via is pending, because the via is what makes
  /// an object on another layer a legitimate end point.
  fn resolve_end_item(&self, at: Vec2, end: Option<HostId>) -> Option<ItemId> {
    let host = end?;
    let layer = if self.placing_via() {
      None
    } else {
      self.current_layer()
    };

    self.resolve_host_item(self.query_node(), at, host, layer)
  }

  /// Whether an item of the node delta can be described as a
  /// [`NewItem`].
  ///
  /// A single track placer stores segments and vias
  /// (`pcbnew/router/pns_line_placer.cpp:1669`, `:1704`), and a via's
  /// hole rides along with it, so anything else in the delta belongs to
  /// the board and not to the session. Virtual items are never reported
  /// to the host (`pcbnew/router/pns_router.cpp:894`).
  ///
  /// `Kind::ARC` is in the mask even though [`NewGeometry`] cannot
  /// describe an arc yet. KiCad filters the three lists on
  /// `IsVirtual()` and on nothing else (`pcbnew/router/pns_router.cpp:891`,
  /// `:897`), and a shove that pushes an existing arc track aside removes
  /// it from slice 7 of `doc/work/012-arcs.md` on; leaving the arc out of
  /// the mask would make that removal vanish silently instead of reaching
  /// the host. What an arc cannot do is come back on the added side, and
  /// [`Router::new_item`] says so with [`CommitError`].
  fn is_committable(&self, id: ItemId) -> bool {
    self.world.item(id).is_some_and(|item| {
      item.of_kind(Kind::SEGMENT | Kind::ARC | Kind::VIA) && !item.is_virtual()
    })
  }

  /// One item of a commit, as a host has to build it.
  ///
  /// The error arm is the whole of the crate's answer to "what does the
  /// commit do with an arc", see [`CommitError::ArcNotRepresentable`].
  fn new_item(&self, id: ItemId) -> Result<NewItem, CommitError> {
    let Some(item) = self.world.item(id) else {
      return Err(CommitError::NotCommittable { uid: 0 });
    };

    let geometry = match item.body() {
      ItemBody::Segment(body) => NewGeometry::Segment {
        seg: body.seg(),
        width: body.width(),
      },
      ItemBody::Via(body) => NewGeometry::Via {
        pos: body.pos(),
        diameter: body.diameter(item.layers(), item.layers().start()),
        drill: body.drill(),
        via_type: body.via_type(),
      },
      ItemBody::Arc(_) => {
        return Err(CommitError::ArcNotRepresentable { uid: item.uid() });
      }
      ItemBody::Solid(_) | ItemBody::Hole(_) => {
        return Err(CommitError::NotCommittable { uid: item.uid() });
      }
    };

    Ok(NewItem {
      geometry,
      net: item.net(),
      layers: item.layers(),
      source: item.source(),
    })
  }

  /// The three lists of `ROUTER::CommitRouting( NODE* )` and the arena
  /// handles behind them.
  ///
  /// `pcbnew/router/pns_router.cpp:862` to `:900`, including the remove
  /// plus add fold documented on [`Router::stop_routing`]. Nothing is
  /// applied here: the caller has to build the whole plan before
  /// committing, because the commit takes the removed items out of the
  /// arena and their handles go stale.
  fn build_commit_plan(&self) -> CommitPlan {
    // :864. The guard is on the state, so a drag always commits and a
    // placement that placed nothing never does.
    let Some(placer) = self.placer.as_ref() else {
      return CommitPlan::default();
    };

    let Some(node) = placer.commit_node() else {
      return CommitPlan::default();
    };

    self.commit_plan_for(node)
  }

  /// [`Router::build_commit_plan`] over a node the caller names.
  ///
  /// The body of `CommitRouting( NODE* )` from `:867` on, which is the
  /// half that does not look at the state. [`Router::commit_drag`] uses
  /// it directly, because the dragger's node is not the placer's.
  fn commit_plan_for(&self, node: NodeId) -> CommitPlan {
    let mut plan = CommitPlan::default();

    // A component drag's pads are not committable items and never reach
    // the three lists below; see [`CommitDiff::moved_solids`].
    for moved in self
      .dragger
      .as_ref()
      .map_or(&[][..], ActiveDragger::moved_solids)
    {
      let Some(host) = self.index.host_of(moved.before) else {
        continue;
      };

      plan.moved.push((host, *moved));

      // `Commit()` de-duplicates by footprint (`:2925`); this
      // de-duplicates by host object, which is one step closer in.
      if !plan.diff.moved_solids.iter().any(|(seen, _)| *seen == host) {
        plan.diff.moved_solids.push((host, moved.offset));
      }
    }

    let (added, removed) = self.world.get_updated_items(node);
    let mut pool: Vec<ItemId> = added
      .into_iter()
      .filter(|id| self.is_committable(*id))
      .collect();

    // :869
    for id in removed {
      if !self.is_committable(id) {
        continue;
      }

      let source = self.world.item(id).and_then(Item::source);

      // :874. The added item that descends from the same host object is
      // an update of it and not a fresh addition.
      if let Some(source) = source
        && let Some(position) = pool.iter().position(|candidate| {
          self.world.item(*candidate).and_then(Item::source) == Some(source)
        })
      {
        let replacement = pool.remove(position);

        match self.new_item(replacement) {
          Ok(new_item) => {
            plan.diff.updated.push((source, new_item));
            plan.updated.push(replacement);

            continue;
          }
          // The update could not be described, so the host is told to
          // delete the old object and the error says what it is missing.
          Err(error) => {
            plan.error.get_or_insert(error);
          }
        }
      }

      // :890
      if let Some(host) = self.index.host_of(id) {
        plan.diff.removed.push(host);
        plan.removed.push(id);
      }
    }

    // :895
    for id in pool {
      match self.new_item(id) {
        Ok(new_item) => {
          plan.diff.added.push(new_item);
          plan.added.push(id);
        }
        Err(error) => {
          plan.error.get_or_insert(error);
        }
      }
    }

    plan
  }

  /// The preview of whatever algorithm is running.
  fn frame(&self) -> PreviewFrame {
    if let Some(dragger) = self.dragger.as_ref() {
      return drag_preview_frame(
        &self.world,
        &self.index,
        dragger,
        self.resolver.as_ref(),
      );
    }

    self
      .placer
      .as_ref()
      .map_or_else(PreviewFrame::default, |placer| {
        preview_frame(&self.world, &self.index, placer, self.resolver.as_ref())
      })
  }

  /// The second half of `ROUTER::StopRouting`,
  /// `pcbnew/router/pns_router.cpp:977`: drop the algorithms, go idle,
  /// and clean the root of everything the session left on it.
  fn finish_session(&mut self) {
    let root = self.world.root();

    self.placer = None;
    // :985
    self.dragger = None;
    self.state = RouterState::Idle;
    self.world.kill_children(root);
    self
      .world
      .clear_ranks(root, MarkerFlags::CLEARED_BY_CLEAR_RANKS);
  }
}

/// Which drag algorithm a session is running.
///
/// Port of the `m_dragger` slot (`pcbnew/router/pns_router.h:284`), a
/// `std::unique_ptr<DRAG_ALGO>` over three implementations.
/// `DESIGN.md` section 11 and note 06 section 9.3 both ask for enum
/// dispatch here rather than a trait: `DRAG_ALGO` is a C++ virtual base
/// and all three of its implementations are in this crate, so a trait
/// would be a vtable nothing outside needs. There is deliberately no
/// `src/drag_algo.rs`.
///
/// All three are boxed because two of them carry a
/// [`crate::shove::Shove`], so an unboxed enum would be as large as the
/// biggest of them wherever a [`Router`] is moved.
enum ActiveDragger {
  /// One segment, one corner or one via of one trace. `DRAGGER`.
  Single(Box<Dragger>),
  /// Several traces at once. `MULTI_DRAGGER`.
  Multi(Box<MultiDragger>),
  /// One or more footprints, by their pads. `COMPONENT_DRAGGER`.
  Component(Box<ComponentDragger>),
}

impl ActiveDragger {
  /// `DRAG_ALGO::Drag` (`pcbnew/router/pns_drag_algo.h:80`). The answer
  /// is "this position has a valid solution", which neither the facade
  /// nor `ROUTER::moveDragging` reads.
  fn drag(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
  ) -> bool {
    match self {
      ActiveDragger::Single(dragger) => dragger.drag(world, context, at),
      ActiveDragger::Multi(dragger) => dragger.drag(world, context, at),
      // A component drag reads no settings and runs no collision query,
      // so it needs no context (note 06 erratum E30).
      ActiveDragger::Component(dragger) => dragger.drag(world, at),
    }
  }

  /// `DRAG_ALGO::CurrentNode` (`pcbnew/router/pns_drag_algo.h:96`).
  fn current_node(&self) -> NodeId {
    match self {
      ActiveDragger::Single(dragger) => dragger.current_node(),
      ActiveDragger::Multi(dragger) => dragger.current_node(),
      ActiveDragger::Component(dragger) => dragger.current_node(),
    }
  }

  /// `DRAG_ALGO::CurrentLayer` (`pcbnew/router/pns_drag_algo.h:110`).
  /// All three implementations are unreachable in KiCad and each is
  /// unhelpful in its own way; see [`Dragger::current_layer`],
  /// [`MultiDragger::current_layer`] and
  /// [`ComponentDragger::current_layer`].
  fn current_layer(&self) -> i32 {
    match self {
      ActiveDragger::Single(dragger) => dragger.current_layer(),
      ActiveDragger::Multi(dragger) => dragger.current_layer(),
      ActiveDragger::Component(dragger) => dragger.current_layer(),
    }
  }

  /// The first of `DRAG_ALGO::CurrentNets`
  /// (`pcbnew/router/pns_drag_algo.h:103`). A single drag has exactly
  /// one net; a multi drag may have one per line and the facade's
  /// [`Router::current_net`] answers with one, so this is the first in
  /// [`MultiDragger::current_nets`] order.
  fn current_net(&self) -> Option<NetId> {
    match self {
      ActiveDragger::Single(dragger) => dragger.current_nets(),
      ActiveDragger::Multi(dragger) => dragger.current_nets().first().copied(),
      // Always nothing: `COMPONENT_DRAGGER::CurrentNets` answers an
      // empty vector (`pcbnew/router/pns_component_dragger.h:85`).
      ActiveDragger::Component(dragger) => {
        dragger.current_nets().first().copied()
      }
    }
  }

  /// `DRAG_ALGO::Traces` (`pcbnew/router/pns_drag_algo.h:117`), the lines
  /// the drag is moving.
  fn traces(&self) -> &[Line] {
    match self {
      ActiveDragger::Single(dragger) => dragger.traces(),
      ActiveDragger::Multi(dragger) => dragger.traces(),
      ActiveDragger::Component(dragger) => dragger.traces(),
    }
  }

  /// `DRAG_ALGO::GetLastCommittedLeaderSegments`
  /// (`pcbnew/router/pns_drag_algo.h:125`), whose default body answers an
  /// empty vector and which only `MULTI_DRAGGER` overrides.
  fn last_committed_leader_segments(&self) -> &[ItemId] {
    match self {
      ActiveDragger::Single(_) | ActiveDragger::Component(_) => &[],
      ActiveDragger::Multi(dragger) => dragger.last_committed_leader_segments(),
    }
  }

  /// The pads a drag has moved, which only a component drag ever has.
  ///
  /// There is no `DRAG_ALGO` method behind this: KiCad's interface
  /// reconstructs the same information from the removal and addition of
  /// each solid (`pcbnew/router/pns_kicad_iface.cpp:2634`, `:2854`). See
  /// [`CommitDiff::moved_solids`].
  fn moved_solids(&self) -> &[MovedSolid] {
    match self {
      ActiveDragger::Single(_) | ActiveDragger::Multi(_) => &[],
      ActiveDragger::Component(dragger) => dragger.moved_solids(),
    }
  }

  /// Which node a fix would commit, or [`None`] when it refuses.
  ///
  /// The decision half of `DRAG_ALGO::FixRoute`
  /// (`pcbnew/router/pns_drag_algo.h:89`). `force_commit` reaches the
  /// single dragger in forced mark obstacles mode only (note 06 erratum
  /// E7) and the component dragger always; the multi dragger ignores it,
  /// which is note 06 erratum E24.
  fn fix_route_node(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    force_commit: bool,
  ) -> Option<NodeId> {
    match self {
      ActiveDragger::Single(dragger) => {
        dragger.fix_route_node(world, context, force_commit)
      }
      ActiveDragger::Multi(dragger) => {
        dragger.fix_route_node(context, force_commit)
      }
      ActiveDragger::Component(dragger) => {
        dragger.fix_route_node(world, context, force_commit)
      }
    }
  }
}

/// A commit that has been computed but not applied.
///
/// [`CommitDiff`] is what the host sees; the three vectors beside it are
/// the arena handles the same entries came from, in the same order, which
/// is what lets [`Router::stop_routing`] fix the host map up afterwards
/// and what [`Router::assign_host_ids`] indexes into.
#[derive(Default)]
struct CommitPlan {
  /// What the host has to apply.
  diff: CommitDiff,
  /// The items behind [`CommitDiff::added`].
  added: Vec<ItemId>,
  /// The items behind [`CommitDiff::updated`].
  updated: Vec<ItemId>,
  /// The items behind [`CommitDiff::removed`].
  removed: Vec<ItemId>,
  /// The solids a component drag moved, with the host object each one
  /// belongs to.
  ///
  /// Deliberately **not** positional against
  /// [`CommitDiff::moved_solids`]: that has one entry per host object
  /// and this has one per solid, because a pad on several padstack
  /// layers is several solids that all move together.
  moved: Vec<(HostId, MovedSolid)>,
  /// The first item the commit could not describe, if there was one.
  ///
  /// It travels on the plan rather than on [`CommitDiff`] so that the
  /// recorded text format, and therefore every stored session fixture,
  /// is untouched; [`Router::last_commit_error`] is where a host reads
  /// it.
  error: Option<CommitError>,
}

/// Build the frame a host draws after one event.
///
/// The union of `ROUTER::movePlacing` (`pcbnew/router/pns_router.cpp:789`
/// from `:794`), `ROUTER::updateView` (`:745`) and
/// `ROUTER::markViolations` (`:670`). It is a free function so that it can
/// borrow the world, the map, the placer and the oracle at once.
fn preview_frame(
  world: &World,
  index: &HostIndex,
  placer: &Placer,
  resolver: &dyn RuleResolver,
) -> PreviewFrame {
  let node = placer.current_node(true);
  let mut frame = PreviewFrame {
    ratline: placer.leading_rat_line().cloned(),
    ratline_n: placer.leading_rat_line_n().cloned(),
    tuning: tuning_info(placer),
    ..PreviewFrame::default()
  };

  // :796, the route itself and the via it ends with. The loop runs twice
  // for a differential pair, P then N, and each lane draws its own via.
  for (lane, line) in placer.traces().into_iter().enumerate() {
    if line.segment_count() > 0 {
      frame.items.push(PreviewItem {
        chain: line.shape().clone(),
        width: line.width(),
        layer: line.layer(),
        net: line.net(),
        style: PreviewStyle::Head,
        clearance: line_clearance(world, resolver, &line),
      });
    }

    if let Some(via) = preview_via(world, resolver, &line) {
      if lane == 0 {
        frame.via = Some(via);
      } else {
        frame.via_n = Some(via);
      }
    }

    // :700, the violations this line runs into.
    frame
      .violations
      .extend(violations_of(world, index, node, resolver, &line));
  }

  append_node_delta(&mut frame, world, index, node, resolver);

  frame
}

/// The tuning readout of a placer, when it is a tuning placer.
///
/// The four reads `PCB_TUNING_PATTERN::Update` makes off the placer after
/// every `Move` (`pcbnew/generators/pcb_tuning_pattern.cpp:1319` to
/// `:1322`), gathered into one value. [`None`] for the two routing
/// placers, which answer [`None`] to every one of them.
fn tuning_info(placer: &Placer) -> Option<Box<TuningInfo>> {
  let settings = *placer.meander_settings()?;
  let mode = placer.tuning_mode()?;
  let skew = placer.tuning_skew();

  Some(Box::new(TuningInfo {
    mode,
    status: placer.tuning_status()?,
    result: placer.tuning_length_result()?,
    delta: placer.tuning_length_delta(),
    // The window the status was decided against; see [`TuningInfo::target`].
    target: placer
      .tuning_target()
      .unwrap_or_else(LengthTarget::unconstrained),
    settings,
    skew,
    // Only the skew mode reads either, so only it reports them.
    skew_target: match mode {
      TuningMode::PairSkew => settings.target_skew(),
      TuningMode::SingleLength | TuningMode::PairLength => None,
    },
    coupled_length: placer.tuning_coupled_length(),
  }))
}

/// Build the frame a host draws after one drag event.
///
/// `ROUTER::moveDragging` (`pcbnew/router/pns_router.cpp:656`) is three
/// lines: erase the view, drag, and
/// `updateView( m_dragger->CurrentNode(), m_dragger->Traces(), true )`.
/// So the frame is [`append_node_delta`] over the drag node plus the
/// violations of `markViolations` over the dragged geometry, and nothing
/// else. Two consequences of that, both deliberate:
///
/// - the dragged line is drawn from the node delta as
///   [`PreviewStyle::Tail`] and not as [`PreviewStyle::Head`], because
///   `moveDragging` sets no `PNS_HEAD_TRACE` where `movePlacing` sets it
///   on every trace (`:804`). Drawing it from
///   [`Dragger::traces`] as well would draw it twice, since
///   `dragMarkObstacles` adds it to the drag node
///   (`pcbnew/router/pns_dragger.cpp:410`);
/// - there is no [`PreviewFrame::ratline`] and no
///   [`PreviewFrame::via`]: a drag has no head to attach and no pending
///   via, and `ROUTER::StopRouting` never even updates the ratsnest after
///   one (note 06 erratum E16).
///
/// `markViolations`'s dragged item filter (`:724`) has nothing to skip
/// here. KiCad walks the node's own changed items and skips the ones the
/// drag is moving, which for a via drag includes the dragged via itself
/// (`:511`); this walks [`Dragger::traces`] and asks what each **line**
/// runs into, so a dragged via is never a candidate to begin with and
/// reaches the host as an ordinary added item of the node delta. A host
/// that wants KiCad's skip list has [`Dragger::traces_vias`].
fn drag_preview_frame(
  world: &World,
  index: &HostIndex,
  dragger: &ActiveDragger,
  resolver: &dyn RuleResolver,
) -> PreviewFrame {
  let node = dragger.current_node();
  let mut frame = PreviewFrame::default();

  // :700, the violations the dragged geometry runs into.
  for line in dragger.traces() {
    frame
      .violations
      .extend(violations_of(world, index, node, resolver, line));
  }

  append_node_delta(&mut frame, world, index, node, resolver);

  // The pads a component drag is moving. `append_node_delta` has just
  // put each of them in `hidden`, because the drag node took the
  // original out of the board; see [`PreviewFrame::moved_solids`].
  for moved in dragger.moved_solids() {
    let Some(host) = index.host_of(moved.before) else {
      continue;
    };

    if !frame.moved_solids.iter().any(|(seen, _)| *seen == host) {
      frame.moved_solids.push((host, moved.offset));
    }
  }

  frame
}

/// Add a node's delta to a frame.
///
/// The body of `ROUTER::updateView` after `markViolations`
/// (`pcbnew/router/pns_router.cpp:765` to `:773`): every added item is
/// drawn with its own clearance, and every removed one is hidden. It is
/// shared by the placing and the dragging frames because `updateView` is.
fn append_node_delta(
  frame: &mut PreviewFrame,
  world: &World,
  index: &HostIndex,
  node: NodeId,
  resolver: &dyn RuleResolver,
) {
  let (added, removed) = world.get_updated_items(node);

  for id in added {
    let Some(item) = world.item(id) else {
      continue;
    };

    if item.is_virtual() {
      continue;
    }

    let clearance = resolver.clearance(ItemRef::stored(id, item), None, true);

    match item.body() {
      ItemBody::Segment(body) => frame.items.push(PreviewItem {
        chain: LineChain::from_seg(&body.seg()),
        width: body.width(),
        layer: item.layer(),
        net: item.net(),
        style: PreviewStyle::Tail,
        clearance,
      }),
      ItemBody::Via(body) => frame.fixed_vias.push(PreviewVia {
        pos: body.pos(),
        diameter: body.diameter(item.layers(), item.layers().start()),
        drill: body.drill(),
        layers: item.layers(),
        net: item.net(),
        style: PreviewStyle::Tail,
        clearance,
      }),
      // An arc draws as the one arc chain it is. `PreviewItem::chain`
      // carries a [`LineChain`], which has held arcs since work item 012
      // slice 3, so a host that flattens the chain draws the
      // approximation and a host that reads its arcs draws the curve;
      // KiCad's preview reads them (`router_preview_item.cpp:278`).
      ItemBody::Arc(body) => {
        let mut chain = LineChain::new();

        chain.append_arc(&body.arc(), LineChain::ARC_POLYGONIZATION_MAX_ERROR);

        frame.items.push(PreviewItem {
          chain,
          width: body.width(),
          layer: item.layer(),
          net: item.net(),
          style: PreviewStyle::Tail,
          clearance,
        });
      }
      ItemBody::Solid(_) | ItemBody::Hole(_) => {}
    }
  }

  // :772, what the host must stop drawing.
  for id in removed {
    if let Some(host) = index.host_of(id) {
      frame.hidden.push(host);
    }
  }
}

/// The clearance outline a host draws around a route.
///
/// `GetRuleResolver()->Clearance( item, nullptr )`
/// (`pcbnew/router/pns_router.cpp:801`). A line has no stored item, so
/// the throwaway one [`Line::rule_item`] builds stands in for it.
fn line_clearance(
  world: &World,
  resolver: &dyn RuleResolver,
  line: &Line,
) -> Option<i32> {
  let item = line.rule_item(world, PROBE_UID);

  resolver.clearance(ItemRef::unstored(&item), None, true)
}

/// The via a route ends with, when it has one.
///
/// Port of `pcbnew/router/pns_router.cpp:806` to `:822`, including the
/// clearance widening: a hole whose own rule reaches further out than the
/// copper's does, once the annular ring is taken off, decides the outline
/// the host draws. The preview via the placer builds owns no hole (note
/// 03, the milestone 3 log), so the widening only fires for a via the
/// route adopted from the node.
fn preview_via(
  world: &World,
  resolver: &dyn RuleResolver,
  line: &Line,
) -> Option<PreviewVia> {
  let via_ref = line.via_item(world)?;
  let item = via_ref.item();
  let ItemBody::Via(body) = item.body() else {
    return None;
  };

  let mut clearance = resolver.clearance(via_ref, None, true);

  if let Some(hole_id) = item.hole()
    && let Some(hole) = world.item(hole_id)
  {
    let hole_clearance =
      resolver.clearance(ItemRef::stored(hole_id, hole), None, true);

    if let Some(hole_clearance) = hole_clearance {
      // :816
      let annular =
        (body.diameter(item.layers(), line.layer()) - body.drill()).max(0) / 2;
      let excess = hole_clearance - annular;

      if clearance.is_none_or(|copper| excess > copper) {
        clearance = Some(excess);
      }
    }
  }

  Some(PreviewVia {
    pos: body.pos(),
    diameter: body.diameter(item.layers(), line.layer()),
    drill: body.drill(),
    layers: item.layers(),
    net: item.net(),
    style: PreviewStyle::Head,
    clearance,
  })
}

/// Every obstacle one route line runs into.
///
/// Port of the body of `ROUTER::markViolations`
/// (`pcbnew/router/pns_router.cpp:700` to `:742`). One query covers the
/// whole line and its via, because
/// [`World::query_colliding_line`] runs KiCad's per segment loop and the
/// extra via query (`pcbnew/router/pns_node.cpp:302` to `:316`) against
/// one obstacle set.
///
/// The dragged item filter (`:724`) has nothing to skip while routing.
/// `LINE::GetBlockingObstacle` (`:738`) is queried, although only
/// `rhMarkObstacles` writes that field and in this revision it only ever
/// writes nothing (note 03 section 1.4).
fn violations_of(
  world: &World,
  index: &HostIndex,
  node: NodeId,
  resolver: &dyn RuleResolver,
  line: &Line,
) -> Vec<ViolationMarker> {
  let obstacles = world.query_colliding_line(
    node,
    line,
    resolver,
    &CollisionSearchOptions::default(),
  );
  let mut markers: Vec<ViolationMarker> = obstacles
    .iter()
    .filter_map(|obstacle| {
      let id = obstacle.item?;

      Some(marker_for(world, index, line, id, obstacle.clearance))
    })
    .collect();

  // :738
  if let Some(id) = line.blocking_obstacle()
    && !markers.iter().any(|marker| marker.item == id)
  {
    let clearance = world
      .item(id)
      .and_then(|item| {
        let probe = line.rule_item(world, PROBE_UID);

        resolver.clearance(
          ItemRef::unstored(&probe),
          Some(ItemRef::stored(id, item)),
          true,
        )
      })
      .unwrap_or(0);

    markers.push(marker_for(world, index, line, id, clearance));
  }

  markers
}

/// One entry of [`violations_of`].
///
/// The `updateItem` lambda of `ROUTER::markViolations`
/// (`pcbnew/router/pns_router.cpp:672`), minus the marker write; see
/// [`ViolationMarker`].
fn marker_for(
  world: &World,
  index: &HostIndex,
  line: &Line,
  id: ItemId,
  clearance: i32,
) -> ViolationMarker {
  let item = world.item(id);
  // :682. The route is always single layer, which is the `!currentItem->
  // Layers().IsMultilayer()` half of KiCad's test.
  let forced_layer = item.and_then(|item| {
    (item.layers().is_multilayer() && !item.has_unique_shape_layers())
      .then_some(line.layer())
  });

  ViolationMarker {
    item: id,
    host: index.host_of(id),
    clearance,
    forced_layer,
    // :686
    hide_original: !item.is_some_and(Item::is_compound_shape_primitive),
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::shape::Shape;
  use crate::rules::FixedClearance;
  use crate::snapshot::{WorldGeometry, WorldItem};

  /// Two pads on one layer, far enough apart for a straight run.
  fn board() -> WorldSnapshot {
    let mut snapshot = WorldSnapshot::new(2, 1_000_000);

    for (id, at) in [(1_u64, Vec2::new(0, 0)), (2, Vec2::new(4_000_000, 0))] {
      snapshot.items.push(WorldItem::new(
        HostId(id),
        Some(NetId(1)),
        LayerRange::single(0),
        WorldGeometry::Solid {
          shape: Shape::circle(at, 400_000),
          pos: at,
          offset: Vec2::new(0, 0),
          orientation_degrees: 0.0,
          anchors: Vec::new(),
        },
      ));
    }

    snapshot
  }

  /// A session over that board, already started on the left pad.
  fn started() -> Router {
    let mut router = Router::new(
      &board(),
      Box::new(FixedClearance::uniform(100_000)),
      RoutingSettings::default(),
      Sizes {
        track_width: 200_000,
        ..Sizes::default()
      },
    );

    router
      .start_routing(Vec2::new(0, 0), Some(HostId(1)), 0)
      .expect("the left pad is a routable start");
    router
  }

  #[test]
  fn an_idle_router_has_nothing_pending() {
    let router = Router::new(
      &board(),
      Box::new(FixedClearance::uniform(100_000)),
      RoutingSettings::default(),
      Sizes::default(),
    );

    assert_eq!(router.pending_update(), PendingUpdate::default());
  }

  #[test]
  fn a_fixed_leg_is_pending_before_it_is_committed() {
    let mut router = started();

    router.move_to(Vec2::new(2_000_000, 0), None);
    router.fix_route(Vec2::new(2_000_000, 0), None, false);

    let pending = router.pending_update();

    assert!(
      !pending.added.is_empty(),
      "a fixed leg left nothing in the node delta"
    );
    assert!(
      pending.removed.is_empty(),
      "a route through empty space removed a board object"
    );

    for id in &pending.added {
      assert!(
        router
          .world()
          .item(*id)
          .is_some_and(|item| item.of_kind(Kind::SEGMENT | Kind::VIA)),
        "the delta holds something a single track placer cannot make"
      );
    }

    // The commit reports the same work through the host facing shape.
    let committed = pending.added.len();
    let diff = router.stop_routing();

    assert_eq!(diff.added.len() + diff.updated.len(), committed);
    assert_eq!(router.pending_update(), PendingUpdate::default());
  }
}
