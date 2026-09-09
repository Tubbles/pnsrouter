// SPDX-License-Identifier: GPL-3.0-or-later

//! The routing session facade a host drives (KiCad's `PNS::ROUTER`).
//!
//! Port of `PNS::ROUTER` (`pcbnew/router/pns_router.h:153`) narrowed to
//! single track routing. The facade owns the world, the placer, the rule
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
//! - **Dragging, the component dragger, differential pairs and the three
//!   tuning modes.** [`RouterState::DragSegment`] is declared so the
//!   state machine has KiCad's shape, and nothing ever enters it. The
//!   placer set is closed by the mode (note 03 section 9.2), so the
//!   missing modes are missing variants and not missing trait
//!   implementations.
//! - **`GetLastCommittedLeaderSegments`**
//!   (`pcbnew/router/pns_router.cpp:940`). Note 03 section 1.5 shows it
//!   is populated only by `MULTI_DRAGGER` and that `LINE_PLACER` never
//!   contributes to it, so a single track facade would always answer with
//!   an empty vector.
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
use crate::debug::{DebugDecorator, NoDebug};
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
use crate::node::{NodeId, World};
use crate::placer::line_placer::LinePlacer;
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
/// Port of `ROUTER::RouterState`, `pcbnew/router/pns_router.h:157`, minus
/// `DRAG_COMPONENT`. `RoutingInProgress()` is `m_state != IDLE`
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
  /// An existing track is being dragged. `DRAG_SEGMENT`.
  ///
  /// Declared so the state machine has KiCad's shape and never entered:
  /// the dragger is out of scope for this milestone (`PLAN.md`, M8).
  DragSegment,
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
  pub via: Option<PreviewVia>,

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
  pub hidden: Vec<HostId>,
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

/// What one routing session changed.
///
/// Port of `ROUTER::CommitRouting( NODE* )`
/// (`pcbnew/router/pns_router.cpp:862`), whose three `ROUTER_IFACE` calls
/// become three lists a host applies inside one undo transaction. Note 05
/// section 7.2 asks for exactly this shape, minus `moved_pads`, which
/// only the component dragger produces.
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
  placer: Option<LinePlacer>,
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
      state: RouterState::Idle,
      copper_layer_count: snapshot.copper_layer_count,
      committed: Vec::new(),
      recorder: None,
      recording_depth: 0,
    }
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
  /// Port of `UpdateSizes` (`pcbnew/router/pns_router.cpp:781`) minus its
  /// second half: KiCad also pushes the new sizes into a running placer,
  /// which is how its width and via size actions take effect mid route.
  /// [`LinePlacer`] has no such entry point yet, so a running placement
  /// keeps the sizes it started with and a host applies a width change by
  /// fixing and starting a new leg. Widening the placer is a milestone 5
  /// follow up, tracked in `doc/work/005-session-api-and-event-log.md`.
  pub fn set_sizes(&mut self, sizes: Sizes) {
    self.record(SessionEvent::SetSizes {
      sizes: sizes.clone(),
    });
    self.sizes = sizes;
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

  /// The layer being routed on. Port of `GetCurrentLayer()`,
  /// `pcbnew/router/pns_router.cpp:1041`, whose `-1` for "no placer"
  /// becomes [`None`].
  pub fn current_layer(&self) -> Option<i32> {
    self.placer.as_ref().and_then(LinePlacer::current_layer)
  }

  /// The net being routed. Port of `GetCurrentNets()`,
  /// `pcbnew/router/pns_router.cpp:1031`, which wraps the placer's one
  /// net in a vector.
  pub fn current_net(&self) -> Option<NetId> {
    self.placer.as_ref().and_then(LinePlacer::current_net)
  }

  /// Whether the next fix would place a via. Port of `IsPlacingVia()`,
  /// `pcbnew/router/pns_router.cpp:1059`.
  pub fn placing_via(&self) -> bool {
    self.placer.as_ref().is_some_and(LinePlacer::is_placing_via)
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
  /// The differential pair half of the routine (`:340` to `:427`) is out
  /// of scope; see `PLAN.md`.
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

    if let Some(failure) = failure {
      return Err(failure);
    }

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

    self.placer = Some(placer);
    self.state = RouterState::RouteTrack;

    Ok(self.frame())
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
  pub fn move_to(&mut self, at: Vec2, end: Option<HostId>) -> PreviewFrame {
    self.record(SessionEvent::MoveTo { at, end });

    if self.state != RouterState::RouteTrack {
      return PreviewFrame::default();
    }

    let end_item = self.resolve_end_item(at, end);
    let context = AlgoContext {
      resolver: self.resolver.as_ref(),
      settings: &self.settings,
      debug: self.debug.as_ref(),
    };

    if let Some(placer) = self.placer.as_mut() {
      placer.move_to(&mut self.world, &context, at, end_item);
    }

    self.frame()
  }

  /// Pin the route down at a point.
  ///
  /// Port of `ROUTER::FixRoute` (`pcbnew/router/pns_router.cpp:915`)
  /// through its `ROUTE_TRACK` branch, plus the `CommitRouting()` its
  /// host performs when the placer reports that the route reached its
  /// target (note 03 section 1.6). `force_finish` is KiCad's
  /// `aForceFinish`, which makes the fix terminal wherever it lands
  /// (`pcbnew/router/pns_line_placer.cpp:1647`); `aForceCommit` is a
  /// dragger only parameter the placer never sees (`:918`).
  ///
  /// An idle router answers `Continue` with an empty frame, which is the
  /// `default:` branch of `ROUTER::FixRoute` (`:928`).
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

    if self.state != RouterState::RouteTrack {
      return FixOutcome::Continue(PreviewFrame::default());
    }

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
    let mut settled = self.placer.as_ref().and_then(LinePlacer::current_end);

    // This routine picks its own anchor, so it is one event and the moves
    // and the fix it drives are not recorded on their own.
    self.suspend_recording();

    for _ in 0..5 {
      settled = self.placer.as_ref().and_then(LinePlacer::current_end);
      self.move_to(anchor.at, host);

      if self.placer.as_ref().and_then(LinePlacer::current_end) == settled {
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
    let end = self.placer.as_ref().and_then(LinePlacer::current_end)?;

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

    placer.toggle_via(armed)
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

    if let Some(placer) = self.placer.as_mut() {
      placer.flip_posture();
    }
  }

  /// Cycle the corner mode.
  ///
  /// Port of `ROUTER::ToggleCornerMode`
  /// (`pcbnew/router/pns_router.cpp:1069`), which writes the new mode
  /// back into the settings object. KiCad's cycle is
  /// `MITERED_45 -> ROUNDED_45 -> MITERED_90 -> ROUNDED_90 ->
  /// MITERED_45`; the two rounded modes need an arc body
  /// (`DESIGN.md` section 3), so this crate has the two mitered ones and
  /// the cycle is between them.
  pub fn toggle_corner_mode(&mut self) {
    self.record(SessionEvent::ToggleCornerMode);

    self.settings.corner_mode = match self.settings.corner_mode {
      CornerMode::Mitered45 => CornerMode::Mitered90,
      CornerMode::Mitered90 => CornerMode::Mitered45,
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
  /// See [`PendingUpdate`] for why this is not a [`CommitDiff`].
  pub fn pending_update(&self) -> PendingUpdate {
    let Some(placer) = self.placer.as_ref() else {
      return PendingUpdate::default();
    };
    // :839
    let node = placer.current_node(true);
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

  /// `StopRouting` also pushes the touched nets to the host so it can
  /// rebuild the ratsnest (`:971`); a host here reads them off the diff.
  pub fn stop_routing(&mut self) -> CommitDiff {
    self.record(SessionEvent::StopRouting);

    let plan = self.build_commit_plan();

    // :959, the placer's own commit, which folds its scratch branch into
    // the root through `World::commit`.
    if let Some(placer) = self.placer.as_mut() {
      placer.commit_placement(&mut self.world);
    }

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
  fn is_committable(&self, id: ItemId) -> bool {
    self.world.item(id).is_some_and(|item| {
      item.of_kind(Kind::SEGMENT | Kind::VIA) && !item.is_virtual()
    })
  }

  /// One item of a commit, as a host has to build it.
  fn new_item(&self, id: ItemId) -> Option<NewItem> {
    let item = self.world.item(id)?;
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
      ItemBody::Solid(_) | ItemBody::Hole(_) => return None,
    };

    Some(NewItem {
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
    let mut plan = CommitPlan::default();

    // :864
    let Some(placer) = self.placer.as_ref() else {
      return plan;
    };

    if !placer.has_placed_anything() {
      return plan;
    }

    let Some(node) = placer.last_node() else {
      return plan;
    };

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

        if let Some(new_item) = self.new_item(replacement) {
          plan.diff.updated.push((source, new_item));
          plan.updated.push(replacement);

          continue;
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
      if let Some(new_item) = self.new_item(id) {
        plan.diff.added.push(new_item);
        plan.added.push(id);
      }
    }

    plan
  }

  /// The preview of the placer's current state.
  fn frame(&self) -> PreviewFrame {
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
    self.state = RouterState::Idle;
    self.world.kill_children(root);
    self
      .world
      .clear_ranks(root, MarkerFlags::CLEARED_BY_CLEAR_RANKS);
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
  placer: &LinePlacer,
  resolver: &dyn RuleResolver,
) -> PreviewFrame {
  let node = placer.current_node(true);
  let mut frame = PreviewFrame {
    ratline: placer.leading_rat_line().cloned(),
    ..PreviewFrame::default()
  };

  // :796, the route itself and the via it ends with.
  for line in placer.traces() {
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
      frame.via = Some(via);
    }

    // :700, the violations this line runs into.
    frame
      .violations
      .extend(violations_of(world, index, node, resolver, &line));
  }

  // :765, the node delta.
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
      ItemBody::Solid(_) | ItemBody::Hole(_) => {}
    }
  }

  // :772, what the host must stop drawing.
  for id in removed {
    if let Some(host) = index.host_of(id) {
      frame.hidden.push(host);
    }
  }

  frame
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
