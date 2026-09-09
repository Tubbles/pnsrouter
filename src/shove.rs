// SPDX-License-Identifier: GPL-3.0-or-later

//! Pushing what is in the way out of the way.
//!
//! Port of `PNS::SHOVE` (`pcbnew/router/pns_shove.h:46`,
//! `pcbnew/router/pns_shove.cpp`), the third and last of the head
//! routines. Where [`crate::walkaround::Walkaround`] bends the line being
//! routed around whatever it meets, this moves the obstacles instead and
//! leaves the head where the user put it.
//!
//! # The shape of one run
//!
//! The entry point is three calls, [`Shove::clear_heads`],
//! [`Shove::add_head_line`] and [`Shove::run`]
//! (`pcbnew/router/pns_shove.h:80` to `:84`). One run branches a fresh
//! node off the world, adds every head to it, and then resolves
//! collisions one at a time until nothing collides or a budget runs out.
//!
//! Each collision is resolved by re walking the **obstacle** around a set
//! of hulls built along the head, which is what
//! [`Shove::shove_obstacle_line`] does. There is no analytic "left or
//! right" decision anywhere: `Shove::shove_line_to_hull_set` tries the
//! two windings and the two hull orders and takes the first candidate
//! that keeps the obstacle's endpoints, points away from the pusher, does
//! not intersect itself and does not collide (`DESIGN.md` section 6.2).
//!
//! What stops the shove wave from oscillating is the rank. A head starts
//! at [`HEAD_RANK`] and every forward shove hands the pushed line
//! `rank - 1`, so ranks decrease outward; when the current line meets
//! something that outranks it, the roles are reversed and the current
//! line yields instead (`pcbnew/router/pns_shove.cpp:1717`).
//!
//! # What this revision covers
//!
//! Phases 1 to 4 of note 04 section 8.5: the hull walk, the line stack,
//! the springback stack, the root line index, the optimizer queue, the
//! main loop, solids through the walkaround escalation
//! (`Shove::on_colliding_solid`) and vias in all three of their roles
//! (pusher, pushee and head). Arcs are phase 5 and every arc site
//! carries a `TODO(arcs)`.
//!
//! # Solids and the walkaround escalation
//!
//! A solid can never be shoved, so the roles swap: the **current** line
//! is re-routed around it. [`ShoveStatus::TryWalk`] is the answer that
//! asks for that, and `Shove::shove_iteration` turns it into a call to
//! `Shove::on_colliding_solid`
//! (`pcbnew/router/pns_shove.cpp:1825`). The walk runs over a
//! [`World::assemble_cluster`] of everything topologically attached to
//! the obstacle, so a pad in a row of touching pads is walked around as
//! a group. Its first attempt ranks the result at
//! `rank + WALKAROUND_RANK_PROMOTION`, which turns a line that had to
//! give way into a pusher for every later iteration.
//!
//! # Vias
//!
//! Three interactions, note 04 section 2.6:
//!
//! - a via as the **pusher**, when the current line's segments are on
//!   another layer and only its via reaches the obstacle:
//!   `Shove::shove_line_from_lone_via`;
//! - a via as the **pushee**: `Shove::on_colliding_via` computes a
//!   minimum translation vector and `Shove::push_or_shove_via` moves the
//!   via and drags every track attached to its joint along with it;
//! - via against via, per shape layer, which takes priority over a line
//!   collision when both are present.
//!
//! [`crate::settings::RoutingSettings::shove_vias`] switches the middle
//! one off, and then every via collision degrades to
//! [`ShoveStatus::TryWalk`] and therefore to the walkaround.
//!
//! # Effects instead of in place mutation
//!
//! Note 04 section 8.3 asks for the single biggest structural change of
//! the port, and this is it. KiCad's `shoveIteration` takes a **copy** of
//! the line stack's top (`pcbnew/router/pns_shove.cpp:1635`) and then
//! hands a pointer to it to handlers that replace items in the node,
//! which invalidates the copy's links while the caller still holds it.
//!
//! Here `Shove::shove_iteration` returns a [`Vec`] of [`ShoveEffect`]
//! and `Shove::shove_main_loop` applies them. No handler calls a node
//! mutator, so nothing a handler holds can go stale under it. Handlers
//! still take `&mut World`, because [`World::nearest_obstacle`] and the
//! clearance and hull caches behind it need it; the invariant is about
//! **items**, not about the caches, and it is stated on every handler.
//!
//! # Deliberate deviations
//!
//! - The iteration budget is reset once per [`Shove::run`], not once per
//!   head as in KiCad (`pcbnew/router/pns_shove.cpp:1893`, reached from
//!   `:2547`), which note 04 section 8.4 asks for: KiCad's total work
//!   scales with the head count in a way the caller cannot see.
//! - There is no wall clock. KiCad's second budget is a `TIME_LIMIT` over
//!   `wxGetLocalTimeMillis` (`pcbnew/router/time_limit.cpp:87`), which
//!   `DESIGN.md` section 8 rules out; [`crate::settings::RoutingSettings`]
//!   carries `shove_time_limit_ms` as data and nothing reads it.
//! - The root line index is keyed by [`crate::item::Item::uid`], a per
//!   world counter, where KiCad keys on `LINKED_ITEM::UNIQ_ID` from a
//!   process global one (note 04 section 9 item 10). It is a
//!   [`BTreeMap`], so the erases and lookups are ordered, and it is never
//!   iterated either way.
//! - The optimizer queue is keyed by root line identity rather than
//!   scanned by link (`DESIGN.md` section 6.2, note 04 section 8.2). See
//!   `OptimizerQueue::insert` for where that answers differently.
//!
//! # Not ported
//!
//! - `sanityCheck` (`pcbnew/router/pns_shove.cpp:186`) has no callers in
//!   KiCad's tree; the invariant it documents is enforced for real inside
//!   `Shove::shove_line_to_hull_set`.
//! - The free `removeHead` (`:2270`) has no callers either.
//! - `m_draggedVia` (`:203`) is never assigned, so the dead branch at
//!   `:1897` is not ported.
//! - `m_multiLineMode` (`:2401`) is only read inside `#if 0` blocks
//!   (`:753`, `:855`), and `m_restrictSpringbackTagId` (`:207`) is
//!   written once and never read. Both are dropped.
//! - `SHOVE_STATUS`'s `SH_NULL` and `SH_HEAD_MODIFIED` are dead (note 04
//!   section 1.1), so [`ShoveStatus`] has three variants.
//! - The `SHP_IGNORE` collision filter (`:1654` to `:1676`) is not built.
//!   Nothing in KiCad's tree ever sets that policy bit, so the filter
//!   always answers true; the bit is kept on [`ShovePolicy`] for the day
//!   a caller wants it.
//! - `SHP_WALK_FORWARD` and `SHP_WALK_BACK` are declared and never tested
//!   anywhere (note 04 section 1.1), so they are not on [`ShovePolicy`].
//! - The dead `SPRINGBACK_TAG` fields `m_length`, `m_p` and `m_seq`
//!   (`pcbnew/router/pns_shove.h:203`, `:205`, `:208`) are not on
//!   [`SpringbackFrame`].
//! - `formatPolicy` (`:2617`) and the stale namespace scope copy of
//!   `SHOVE_POLICY` (`:2607`) are debug scaffolding with two copy paste
//!   bugs in them.
//! - `ShoveDraggingVia` (`pcbnew/router/pns_shove.h:86`) is declared and
//!   never defined; its one would be caller is commented out
//!   (`pcbnew/router/pns_dragger.cpp:919`), which runs an ordinary
//!   [`Shove::run`] over a via head instead. `walkaroundLoneVia` is not
//!   in this revision at all; the routine that walks an obstacle around a
//!   lone via is `shoveLineFromLoneVia` (`:278`), which is ported.

use std::collections::BTreeMap;

use crate::algo_base::AlgoContext;
use crate::collide::{CollisionSearchOptions, LineHead, collide_line_items};
use crate::geometry::box2::Box2;
use crate::geometry::direction45::CornerMode;
use crate::geometry::line_chain::{LineChain, PointInsideTracker};
use crate::geometry::shape::Shape;
use crate::geometry::vec2::Vec2;
use crate::item::{
  Item, ItemBody, ItemId, Kind, LayerRange, MarkerFlags, NetId,
};
use crate::line::{Line, LineVia};
use crate::node::{NodeId, World};
use crate::optimizer::{EffortFlags, Optimizer};
use crate::rules::ItemRef;
use crate::settings::OptimizerEffort;
use crate::walkaround::{WalkPolicy, Walkaround, WalkaroundStatus};

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

/// The rank a head is routed at.
///
/// Port of the `100000` at `pcbnew/router/pns_shove.cpp:2501`. Every
/// forward shove hands out `rank - 1`, so this is the ceiling the wave
/// descends from and it has to be far above anything a run can reach.
pub const HEAD_RANK: i32 = 100000;

/// How much the hulls grow after a failed shove attempt, in nanometres.
///
/// Port of `cHullFailureExpansionFactor`,
/// `pcbnew/router/pns_shove.cpp:524`, applied at `:628`.
const HULL_FAILURE_EXPANSION: i32 = 1000;

/// How many times [`Shove::shove_obstacle_line`] inflates the hulls.
///
/// Port of the loop bound at `pcbnew/router/pns_shove.cpp:578`.
const HULL_EXPANSION_ATTEMPTS: i32 = 3;

/// How close an obstacle endpoint has to be to a hull to be snapped onto
/// it, in nanometres.
///
/// Port of `c_ENDPOINT_ON_HULL_THRESHOLD`,
/// `pcbnew/router/pns_shove.cpp:332`.
const ENDPOINT_ON_HULL_THRESHOLD: i64 = 1000;

/// The uid every unstored probe item is built with.
///
/// Uids order the obstacle candidates (`DESIGN.md` section 8) and nothing
/// ever sorts an unstored item, so one value serves them all. It matches
/// the private constant `src/node.rs` uses for the same purpose.
const PROBE_UID: u64 = u64::MAX;

/// How far above its own rank a walked around line is promoted.
///
/// Port of the `currentRank + 10000` at
/// `pcbnew/router/pns_shove.cpp:832`, the "jump the queue" trick: a line
/// that had to walk around something immovable is lifted far above
/// everything else, so later iterations treat it as a pusher rather than
/// as a pushee. [`crate::settings::RoutingSettings::jump_over_obstacles`]
/// switches it off, and then the ordinary `rank - 1` applies.
const WALKAROUND_RANK_PROMOTION: i32 = 10000;

/// How many times `Shove::on_colliding_solid` walks the current line.
///
/// Port of the loop bound at `pcbnew/router/pns_shove.cpp:827`. The two
/// attempts differ only in the rank they would assign; see
/// [`WALKAROUND_RANK_PROMOTION`].
const SOLID_WALKAROUND_ATTEMPTS: i32 = 2;

/// How far a cluster around a solid may grow before it is abandoned.
///
/// The `10.0` of `TOPOLOGY::AssembleCluster`'s call at
/// `pcbnew/router/pns_shove.cpp:804`, a ratio against the seed item's own
/// area.
const CLUSTER_AREA_EXPANSION_LIMIT: f64 = 10.0;

/// How far a via is nudged per anti snap step, in nanometres.
///
/// The `aForce.Resize( 2 )` of `pushOrShoveVia`,
/// `pcbnew/router/pns_shove.cpp:1074`: a via must not land exactly on an
/// existing joint, so it is walked along the force until it does not.
const VIA_ANTI_SNAP_NUDGE: i32 = 2;

/// How many anti snap steps one via push may take.
///
/// # Deviation
///
/// KiCad's loop at `pcbnew/router/pns_shove.cpp:1067` has no bound at
/// all; a dense enough run of joints along the force direction would spin
/// it forever. A budget is the crate's answer to an unbounded loop
/// (`DESIGN.md` section 8), and a via that has stepped this far has moved
/// well past where the geometry asked it to go, so giving up is the
/// honest answer. Exceeding it is [`ShoveStatus::Incomplete`].
const VIA_ANTI_SNAP_LIMIT: u32 = 1000;

/// The slack added to a fanout width before it is used as a diameter.
///
/// The `+ 1` of `fixupViaCollisions`, `pcbnew/router/pns_shove.cpp:1553`
/// and `:1592`, which makes the inflated scratch via strictly wider than
/// the track it stands for.
const FANOUT_WIDTH_SLACK: i32 = 1;

/// The kinds `Shove::shove_iteration` looks for, in KiCad's order.
///
/// Port of the search order at `pcbnew/router/pns_shove.cpp:1650`. It is
/// a priority ladder: solids first because they can never be shoved and
/// therefore have to be walked around before anything else is disturbed,
/// then vias, then segments, then holes.
const OBSTACLE_SEARCH_ORDER: [Kind; 4] =
  [Kind::SOLID, Kind::VIA, Kind::SEGMENT, Kind::HOLE];

// ---------------------------------------------------------------------
// Via handles
// ---------------------------------------------------------------------

/// Where a via is, in a form that survives the via being replaced.
///
/// Port of `VIA_HANDLE`, `pcbnew/router/pns_via.h:45`. Every shove that
/// moves a via **replaces** the item, so a caller that held the old
/// handle would be looking at something the node no longer has; a
/// position, a layer range and a net can be looked up again in whatever
/// node the caller now stands on, which is what
/// [`World::find_via_by_handle`] does.
///
/// KiCad's `valid` flag is an [`Option`] at every use site here.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct ViaHandle {
  /// Where the via sits. Port of `pos`.
  pub pos: Vec2,
  /// The layers it spans. Port of `layers`.
  pub layers: LayerRange,
  /// The net it is on. Port of `net`.
  pub net: Option<NetId>,
}

impl ViaHandle {
  /// The handle of a stored via.
  ///
  /// Port of `VIA::MakeHandle`, `pcbnew/router/pns_via.cpp:310`.
  /// `None` when the handle is stale or does not name a via.
  pub fn of(world: &World, via: ItemId) -> Option<Self> {
    let item = world.item(via)?;
    let ItemBody::Via(body) = item.body() else {
      return None;
    };

    Some(Self {
      pos: body.pos(),
      layers: item.layers(),
      net: item.net(),
    })
  }
}

// ---------------------------------------------------------------------
// Status and policy
// ---------------------------------------------------------------------

/// How far one shove got.
///
/// Port of `SHOVE_STATUS`, `pcbnew/router/pns_shove.h:50`, without the
/// two dead members: `SH_NULL` is only ever the initial value of a local
/// (`pcbnew/router/pns_shove.cpp:1637`) and `SH_HEAD_MODIFIED` is only
/// ever compared against, never assigned (`:2557`). Note 04 section 8.3
/// suggests a `Result` over this; there is no failure here that is not
/// also an answer the caller acts on, so the three live states are one
/// enum and no case is an error.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum ShoveStatus {
  /// Every collision was resolved. `SH_OK`.
  Ok,
  /// The shove gave up: a line could not be pushed anywhere, a stack push
  /// was refused, or the iteration budget ran out. `SH_INCOMPLETE`.
  ///
  /// The caller's world is unchanged; [`Shove::run`] throws its branch
  /// away and steps back to the node it started from.
  Incomplete,
  /// This obstacle cannot be shoved, so the current line has to walk
  /// around it instead. `SH_TRY_WALK`.
  ///
  /// A handler answer, never a [`Shove::run`] answer:
  /// `Shove::shove_iteration` converts it into a call to
  /// `Shove::on_colliding_solid` (`pcbnew/router/pns_shove.cpp:1825`),
  /// so the current line walks around whatever refused to move.
  TryWalk,
}

/// What may be done to one line.
///
/// Port of `SHOVE_POLICY`, `pcbnew/router/pns_shove.h:59`, a bit mask
/// carried per root line. `SHP_WALK_FORWARD` and `SHP_WALK_BACK` are
/// declared in KiCad and tested nowhere, so they are not here.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct ShovePolicy(u32);

impl ShovePolicy {
  /// No bits at all. Port of `SHP_DEFAULT = 0`.
  pub const DEFAULT: ShovePolicy = ShovePolicy(0);

  /// This line may be shoved. Port of `SHP_SHOVE = 0x1`.
  ///
  /// Documentation only: it is the constructor's default
  /// (`pcbnew/router/pns_shove.cpp:209`) and nothing ever tests it.
  pub const SHOVE: ShovePolicy = ShovePolicy(0x1);

  /// Leave this item out of the collision search. Port of
  /// `SHP_IGNORE = 0x8`.
  ///
  /// **Not honoured.** KiCad reads it in one place, the obstacle filter
  /// at `pcbnew/router/pns_shove.cpp:1663`, and nothing in its tree ever
  /// sets it, so that filter always answers true. The bit is kept so a
  /// caller can ask for the behaviour, and the filter is written the day
  /// one does.
  pub const IGNORE: ShovePolicy = ShovePolicy(0x8);

  /// Keep this line out of the post shove optimization. Port of
  /// `SHP_DONT_OPTIMIZE = 0x10`, read at
  /// `pcbnew/router/pns_shove.cpp:2107`.
  ///
  /// The multi dragger is its only user, which is why multi drag results
  /// are not re-optimized (`pcbnew/router/pns_multi_dragger.cpp:653`).
  pub const DONT_OPTIMIZE: ShovePolicy = ShovePolicy(0x10);

  /// Leave this head's endpoints free to slide. Port of
  /// `SHP_DONT_LOCK_ENDPOINTS = 0x20`, read at
  /// `pcbnew/router/pns_shove.cpp:2489`.
  ///
  /// Both draggers set it, because a dragged line's ends are supposed to
  /// move (`pcbnew/router/pns_dragger.cpp:832`).
  pub const DONT_LOCK_ENDPOINTS: ShovePolicy = ShovePolicy(0x20);

  /// The head's **first** point is the pusher, not its last. Port of
  /// `SHP_REVERSED = 0x40`, read at `pcbnew/router/pns_shove.cpp:253`.
  ///
  /// The external hint the archaeology comment at `:234` asks for: an
  /// open curve has no orientation, so corner dragging has to say which
  /// end is pushing (`pcbnew/router/pns_dragger.cpp:836`).
  pub const REVERSED: ShovePolicy = ShovePolicy(0x40);

  /// The raw bits, in KiCad's numbering.
  pub const fn bits(self) -> u32 {
    self.0
  }

  /// A policy from KiCad's integer, keeping every bit.
  pub const fn from_bits(bits: u32) -> Self {
    Self(bits)
  }

  /// Whether every bit of `other` is set here.
  pub const fn contains(self, other: ShovePolicy) -> bool {
    (self.0 & other.0) == other.0
  }
}

impl std::ops::BitOr for ShovePolicy {
  type Output = ShovePolicy;

  fn bitor(self, other: ShovePolicy) -> ShovePolicy {
    ShovePolicy(self.0 | other.0)
  }
}

// ---------------------------------------------------------------------
// The iteration budget
// ---------------------------------------------------------------------

/// How much work one [`Shove::run`] may do.
///
/// The replacement for KiCad's pair of budgets at
/// `pcbnew/router/pns_shove.cpp:1890`: the iteration count is kept and
/// the `TIME_LIMIT` beside it is dropped, because a wall clock inside an
/// algorithm makes the answer depend on the machine (`DESIGN.md`
/// section 8, note 04 section 8.4).
///
/// # The reset point
///
/// KiCad builds both budgets inside `shoveMainLoop`, which runs once per
/// head (`:1893`, called from `:2547`), so a five head drag gets five
/// times the allowance. This one is built once in [`Shove::run`] and
/// carried across the heads, which is what note 04 section 8.4 asks for:
/// the total work must be something the caller can predict.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct IterationBudget {
  /// The ceiling, `ROUTING_SETTINGS::ShoveIterationLimit()`
  /// (`pcbnew/router/pns_routing_settings.cpp:43`, default 250).
  max_iterations: u32,
  /// How many iterations have run. Port of `m_iter`.
  used: u32,
}

impl IterationBudget {
  /// A fresh budget.
  pub const fn new(max_iterations: u32) -> Self {
    Self {
      max_iterations,
      used: 0,
    }
  }

  /// How many iterations have run so far.
  pub const fn used(&self) -> u32 {
    self.used
  }

  /// The ceiling this budget was built with.
  pub const fn max_iterations(&self) -> u32 {
    self.max_iterations
  }

  /// Charge one iteration.
  pub const fn spend(&mut self) {
    self.used = self.used.saturating_add(1);
  }

  /// Whether no more iterations may run.
  ///
  /// Port of the `m_iter >= iterLimit` test at
  /// `pcbnew/router/pns_shove.cpp:1913`, which runs **after** the
  /// iteration and unconditionally, so a run that finishes on its very
  /// last allowed iteration still reports
  /// [`ShoveStatus::Incomplete`]. A caller therefore has to allow
  /// strictly more iterations than the work needs.
  pub const fn is_exhausted(&self) -> bool {
    self.used >= self.max_iterations
  }
}

// ---------------------------------------------------------------------
// The springback stack
// ---------------------------------------------------------------------

/// One committed shove result: the world after that shove.
///
/// Port of `SPRINGBACK_TAG`, `pcbnew/router/pns_shove.h:194`, without its
/// three dead fields (`m_length` and `m_p` are never written, `m_seq` is
/// written at `pcbnew/router/pns_shove.cpp:1026` and never read).
///
/// The stack of these is what lets a shove be **undone** when the mouse
/// moves back, rather than recomputed: [`Shove::run`] drops the frames
/// whose world no longer collides with the new heads and branches from
/// what is left.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct SpringbackFrame {
  /// The branch this frame owns. Port of `m_node`.
  node: NodeId,
  /// Everything this frame and the frames below it changed. Port of
  /// `m_affectedArea`, which the optimizer's area restriction is built
  /// from.
  affected_area: Option<Box2>,
  /// Whether springback may drop this frame. Port of `m_locked`, set
  /// only through [`Shove::add_locked_springback_node`].
  locked: bool,
  /// Where each head's via stood when this frame was pushed. Port of
  /// `m_draggedVias` (`pcbnew/router/pns_shove.h:204`), written at
  /// `pcbnew/router/pns_shove.cpp:1006` and read back by
  /// `reduceSpringback` at `:958` so that popping a frame puts the
  /// caller's via handle back where that frame found it.
  ///
  /// One slot per head, in head order; [`None`] for a head that is a
  /// line rather than a via.
  dragged_vias: Vec<Option<ViaHandle>>,
}

impl SpringbackFrame {
  /// The branch this frame owns.
  pub const fn node(&self) -> NodeId {
    self.node
  }

  /// Where each head's via stood when this frame was pushed.
  pub fn dragged_vias(&self) -> &[Option<ViaHandle>] {
    &self.dragged_vias
  }

  /// Everything this frame and the frames below it changed.
  pub const fn affected_area(&self) -> Option<Box2> {
    self.affected_area
  }

  /// Whether springback may drop this frame.
  pub const fn is_locked(&self) -> bool {
    self.locked
  }
}

// ---------------------------------------------------------------------
// The root line index
// ---------------------------------------------------------------------

/// A handle into `RootLineIndex`.
///
/// The replacement for KiCad's `ROOT_LINE_ENTRY*`
/// (`pcbnew/router/pns_shove.h:282`). Several uids alias one entry, which
/// is exactly why KiCad keeps ownership outside the index: "UID entries
/// may alias the same history entry, so ownership lives outside the
/// index" (`pcbnew/router/pns_shove.h:280`). An index into the owning
/// [`Vec`] aliases just as well and cannot double free.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct RootLineId(usize);

/// What is remembered about one track across a whole shove session.
///
/// Port of `ROOT_LINE_ENTRY`, `pcbnew/router/pns_shove.h:118`.
#[derive(Clone, PartialEq, Debug, Default)]
struct RootLineEntry {
  /// The shape the track had before this session touched it. Port of
  /// `rootLine`.
  ///
  /// The comment at `pcbnew/router/pns_shove.cpp:117` says what it is
  /// for: the optimizer can be told not to make a line worse than it
  /// originally was. `None` for an entry created from a bare item, which
  /// is the via case (`:2003`).
  ///
  /// Its links are cleared, where KiCad keeps the ones the line had and
  /// lets them dangle once the node drops the items. Nothing reads them:
  /// the entry is found through the **index**, never by scanning this
  /// line, and the optimizer only ever compares its geometry.
  root_line: Option<Line>,
  /// The shape the track has now. Port of `newLine`, written by
  /// `replaceLine` (`pcbnew/router/pns_shove.cpp:163`).
  new_line: Option<Line>,
  /// What may be done to this track. Port of `policy`.
  policy: ShovePolicy,
  /// Whether this track is one of the caller's heads. Port of `isHead`,
  /// read by the optimizer (`:2109`) and by `removeHeads` (`:2288`).
  is_head: bool,
  /// The via this entry stood for before the session moved it. Port of
  /// `oldVia`, written by `Run` for a via head
  /// (`pcbnew/router/pns_shove.cpp:2453`).
  old_via: Option<ItemId>,
  /// The via this entry stands for now. Port of `newVia`, written by
  /// `replaceItems` when a via moves (`pcbnew/router/pns_shove.cpp:67`)
  /// and read by `reconstructHeads` (`:2404`).
  new_via: Option<ItemId>,
}

/// Every track this shove session has touched, by uid.
///
/// Port of the pair `m_rootLineHistoryEntries` / `m_rootLineHistory`
/// (`pcbnew/router/pns_shove.h:281`, `:282`). The owning vector grows
/// monotonically for the lifetime of the session, in KiCad as here: only
/// the **index** is pruned (`pruneRootLines`,
/// `pcbnew/router/pns_shove.cpp:901`), never the entries. An interactive
/// router builds one shove per drag, so it is bounded in practice.
#[derive(Clone, PartialEq, Debug, Default)]
struct RootLineIndex {
  /// The entries themselves, owned here and referred to by index.
  entries: Vec<RootLineEntry>,
  /// Which entry each item uid names.
  ///
  /// KiCad's is an `unordered_map` that is only ever point looked up and
  /// single key erased (note 04 section 9 item 8), so it is never
  /// iterated and a [`BTreeMap`] costs nothing and keeps the crate's
  /// no hash iteration rule trivially true.
  by_uid: BTreeMap<u64, RootLineId>,
}

impl RootLineIndex {
  /// A new entry, indexed under every uid given.
  ///
  /// Port of `allocRootLine`, `pcbnew/router/pns_shove.cpp:1942`, plus
  /// the indexing loop its two callers run afterwards (`:137`, `:1993`).
  fn alloc(
    &mut self,
    root_line: Option<Line>,
    policy: ShovePolicy,
    uids: &[u64],
  ) -> RootLineId {
    let id = RootLineId(self.entries.len());

    self.entries.push(RootLineEntry {
      root_line,
      new_line: None,
      policy,
      is_head: false,
      old_via: None,
      new_via: None,
    });

    for uid in uids {
      self.by_uid.insert(*uid, id);
    }

    id
  }

  /// The entry one uid names.
  ///
  /// Port of `findRootLine( const LINKED_ITEM* )`,
  /// `pcbnew/router/pns_shove.cpp:1964`.
  fn find_by_uid(&self, uid: u64) -> Option<RootLineId> {
    self.by_uid.get(&uid).copied()
  }

  /// The entry the first indexed uid of a set names.
  ///
  /// Port of `findRootLine( const LINE& )`,
  /// `pcbnew/router/pns_shove.cpp:1951`, which scans the line's links and
  /// answers on the first hit. Link order is insertion order, so the
  /// answer is deterministic (note 04 section 9 item 8).
  fn find(&self, uids: &[u64]) -> Option<RootLineId> {
    uids.iter().find_map(|uid| self.find_by_uid(*uid))
  }

  /// Point a set of uids at an entry.
  ///
  /// The `for( LINKED_ITEM* link : aNew.Links() ) m_rootLineHistory[...]`
  /// of `replaceLine` (`pcbnew/router/pns_shove.cpp:157`), which note 04
  /// section 8.3 item 3 asks for as one atomic operation rather than as a
  /// loop the caller writes while still holding the old line.
  fn bind(&mut self, id: RootLineId, uids: &[u64]) {
    for uid in uids {
      self.by_uid.insert(*uid, id);
    }
  }

  /// Drop one uid from the index, keeping its entry.
  ///
  /// The single key erase of `pruneRootLines`,
  /// `pcbnew/router/pns_shove.cpp:914`.
  fn forget(&mut self, uid: u64) {
    self.by_uid.remove(&uid);
  }

  /// Read one entry.
  fn entry(&self, id: RootLineId) -> &RootLineEntry {
    &self.entries[id.0]
  }

  /// Write one entry.
  fn entry_mut(&mut self, id: RootLineId) -> &mut RootLineEntry {
    &mut self.entries[id.0]
  }
}

// ---------------------------------------------------------------------
// The line stack
// ---------------------------------------------------------------------

/// What happens to one stacked line when an item it references is about
/// to be replaced.
///
/// Note 04 section 8.2 asks for this to be an explicit enum so that the
/// rule is testable on its own, rather than the three way branch buried
/// in `unwindLineStack` (`pcbnew/router/pns_shove.cpp:1397`).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum UnwindAction {
  /// The line does not reference the item; leave it alone.
  Keep,
  /// The line references the item; take it off the stack.
  Drop,
  /// The line references the item and ends with a via; reduce it to the
  /// via alone and keep it.
  ///
  /// The note to self at `pcbnew/router/pns_shove.cpp:1397` says why:
  /// "if we have a tadpole in the stack, keep track of the via even if
  /// the parent line has been deleted, otherwise the via will be ignored
  /// in the case of collisions with tracks on another layer". A stub is
  /// [`Line::clear_links`], an empty chain and one
  /// [`Line::link_via`].
  DegradeToViaStub,
}

/// What one stacked line should do about an item being replaced.
///
/// The predicate of `unwindLineStack`,
/// `pcbnew/router/pns_shove.cpp:1392`, as a pure function.
/// `item_is_via` is KiCad's `aSeg->OfKind( ITEM::VIA_T )`: unwinding a
/// **via** drops every line that references it, because there is no via
/// left to degrade to.
pub fn unwind_action(
  line: &Line,
  item: ItemId,
  item_is_via: bool,
) -> UnwindAction {
  if !line.contains_link(item) {
    return UnwindAction::Keep;
  }

  // :1397
  if line.ends_with_via() && !item_is_via {
    return UnwindAction::DegradeToViaStub;
  }

  UnwindAction::Drop
}

/// Reduce a line to the via it ends with.
///
/// The `ClearLinks(); Line().Clear(); LinkVia( via )` of
/// `unwindLineStack` (`pcbnew/router/pns_shove.cpp:1414`). KiCad scans
/// the line's links for a via and keeps the **last** one; a line whose
/// via is owned rather than linked has no via link, and KiCad then leaves
/// the line untouched (`:1412` guards the whole rewrite), which is
/// reproduced by answering false.
fn degrade_to_via_stub(world: &World, line: &mut Line) -> bool {
  let via = line.links().iter().rev().copied().find(|link| {
    world
      .item(*link)
      .is_some_and(|item| item.of_kind(Kind::VIA))
  });

  let Some(via) = via else {
    return false;
  };
  let Some(pos) = world.item(via).and_then(|item| match item.body() {
    ItemBody::Via(body) => Some(body.pos()),
    _ => None,
  }) else {
    return false;
  };

  line.clear_links();
  line.chain_mut().clear();
  line.remove_via();
  line.link_via(via, pos);

  true
}

/// The lines whose collisions still have to be resolved.
///
/// Port of `m_lineStack` (`pcbnew/router/pns_shove.h:276`), which is a
/// stack in name only: KiCad pops the back (`:1695`), inserts one slot
/// below the top (`:1465`) and erases from the middle (`:1421`). Note 04
/// section 8.2 asks for exactly those three operations to be named, and
/// they are [`LineStack::push`], [`LineStack::push_below_top`] and
/// [`LineStack::unwind`].
#[derive(Clone, PartialEq, Debug, Default)]
pub struct LineStack {
  /// The lines, bottom first.
  lines: Vec<Line>,
}

impl LineStack {
  /// An empty stack.
  pub const fn new() -> Self {
    Self { lines: Vec::new() }
  }

  /// Whether there is nothing left to resolve.
  pub fn is_empty(&self) -> bool {
    self.lines.is_empty()
  }

  /// How many lines are waiting.
  pub fn len(&self) -> usize {
    self.lines.len()
  }

  /// The line the next iteration works on, the top of the stack.
  ///
  /// `m_lineStack.back()`, `pcbnew/router/pns_shove.cpp:1635`.
  pub fn last(&self) -> Option<&Line> {
    self.lines.last()
  }

  /// The line at the **bottom** of the stack.
  ///
  /// `m_lineStack.front()`, which `onCollidingSolid` reads as "the last
  /// line" (`pcbnew/router/pns_shove.cpp:865`). That looks like a latent
  /// bug and it is the shipped behaviour, so the accessor is named for
  /// what it does rather than for what its one caller calls it.
  pub fn first(&self) -> Option<&Line> {
    self.lines.first()
  }

  /// Put a line on top.
  pub fn push(&mut self, line: Line) {
    self.lines.push(line);
  }

  /// Put a line one slot below the top, so the current line stays on top.
  ///
  /// Port of the `aKeepCurrentOnTop` branch of `pushLineStack`,
  /// `pcbnew/router/pns_shove.cpp:1465`. On an empty stack it is a plain
  /// [`LineStack::push`], as in KiCad.
  pub fn push_below_top(&mut self, line: Line) {
    if self.lines.is_empty() {
      self.lines.push(line);
    } else {
      self.lines.insert(self.lines.len() - 1, line);
    }
  }

  /// Take the top line off.
  pub fn pop(&mut self) -> Option<Line> {
    self.lines.pop()
  }

  /// Take every line that references an item off the stack, or reduce it
  /// to its via.
  ///
  /// The line stack half of `unwindLineStack`,
  /// `pcbnew/router/pns_shove.cpp:1390`, decided per line by
  /// [`unwind_action`] and carried out by `degrade_to_via_stub`.
  ///
  /// It is not a `retain`: the third answer keeps the line and rewrites
  /// it, so the loop has to be able to touch what it walks over.
  pub fn unwind(&mut self, world: &World, item: ItemId) {
    let item_is_via =
      world.item(item).is_some_and(|item| item.of_kind(Kind::VIA));
    let mut index = 0;

    while index < self.lines.len() {
      match unwind_action(&self.lines[index], item, item_is_via) {
        UnwindAction::Keep => index += 1,
        UnwindAction::Drop => {
          self.lines.remove(index);
        }
        UnwindAction::DegradeToViaStub => {
          degrade_to_via_stub(world, &mut self.lines[index]);
          index += 1;
        }
      }
    }
  }

  /// Forget everything.
  pub fn clear(&mut self) {
    self.lines.clear();
  }
}

// ---------------------------------------------------------------------
// The optimizer queue
// ---------------------------------------------------------------------

/// Every line this run touched, waiting to be optimized.
///
/// Port of `m_optimizerQueue` (`pcbnew/router/pns_shove.h:277`), which is
/// not a queue either: it is a set like vector kept deduplicated, and
/// `DESIGN.md` section 6.2 asks for it to be an insertion ordered map
/// keyed by root line identity. The insertion order is observable,
/// because `Shove::run_optimizer` reverses it between passes
/// (`pcbnew/router/pns_shove.cpp:2093`).
///
/// The asymmetry with the line stack is deliberate and is what feeds the
/// optimizer at all: a line that comes clean is popped from the stack
/// with a raw pop that leaves it **in** this queue (`:1695`), while a
/// line that is unwound or explicitly popped leaves the queue as well.
#[derive(Clone, PartialEq, Debug, Default)]
struct OptimizerQueue {
  /// The queued lines with the root line each one descends from, in
  /// insertion order.
  entries: Vec<(RootLineId, Line)>,
}

impl OptimizerQueue {
  /// An empty queue.
  const fn new() -> Self {
    Self {
      entries: Vec::new(),
    }
  }

  /// Forget everything.
  fn clear(&mut self) {
    self.entries.clear();
  }

  /// How many lines are queued.
  fn len(&self) -> usize {
    self.entries.len()
  }

  /// Add a line, replacing whatever stood for the same track.
  ///
  /// Port of the `pruneLineFromOptimizerQueue( aL ); push_back( aL )`
  /// pair inside `pushLineStack`
  /// (`pcbnew/router/pns_shove.cpp:1475`). The entry moves to the end,
  /// which is KiCad's erase then append and **not** what an ordered map
  /// insert would do; the order decides which line the optimizer sees
  /// first, so it is reproduced.
  ///
  /// # Where this answers differently from KiCad
  ///
  /// KiCad's prune matches on shared links
  /// (`pcbnew/router/pns_shove.cpp:1482`). A line that has just been
  /// replaced carries **fresh** links, so that prune finds nothing and
  /// the stale entry survives; it is `unwindLineStack` that removes it,
  /// one call earlier (`:672`). Keying on the root line makes the replace
  /// unconditional, so an entry KiCad could keep in a path that forgot to
  /// unwind is dropped here. That can only ever drop a line whose items
  /// are already gone from the node.
  fn insert(&mut self, key: RootLineId, line: Line) {
    self.entries.retain(|(existing, _)| *existing != key);
    self.entries.push((key, line));
  }

  /// Drop the entry that stands for one track.
  ///
  /// The root line keyed form of `pruneLineFromOptimizerQueue`,
  /// `pcbnew/router/pns_shove.cpp:1482`, which `popLineStack` calls
  /// without re-adding (`:1511`).
  fn prune(&mut self, key: RootLineId) {
    self.entries.retain(|(existing, _)| *existing != key);
  }

  /// Drop every entry that references an item.
  ///
  /// The optimizer queue half of `unwindLineStack`,
  /// `pcbnew/router/pns_shove.cpp:1430`, which unlike the line stack half
  /// has no via exception: it erases unconditionally for a non via link
  /// and leaves everything alone for a via one.
  fn retain_not_referencing(&mut self, item: ItemId, item_is_via: bool) {
    if item_is_via {
      return;
    }

    self.entries.retain(|(_, line)| !line.contains_link(item));
  }

  /// Reverse the order, as each optimizer pass does.
  ///
  /// `std::reverse( m_optimizerQueue... )`,
  /// `pcbnew/router/pns_shove.cpp:2093`, so pass 0 runs newest first and
  /// pass 1 oldest first.
  fn reverse(&mut self) {
    self.entries.reverse();
  }

  /// The widest queued line, which the optimizer's area is inflated by.
  ///
  /// `pcbnew/router/pns_shove.cpp:2034`.
  fn max_width(&self) -> i32 {
    self
      .entries
      .iter()
      .map(|(_, line)| line.width())
      .max()
      .unwrap_or(0)
  }
}

// ---------------------------------------------------------------------
// Heads
// ---------------------------------------------------------------------

/// One line the caller wants routed, and what became of it.
///
/// Port of `HEAD_LINE_ENTRY`, `pcbnew/router/pns_shove.h:133`. A head is
/// either a line or a via being dragged, which is why `orig_head` is an
/// [`Option`] exactly as KiCad's is: `reconstructHeads` branches on it
/// (`pcbnew/router/pns_shove.cpp:2306`).
#[derive(Clone, PartialEq, Debug)]
struct HeadEntry {
  /// The line as the caller handed it over, and then as the node holds
  /// it. Port of `origHead`.
  ///
  /// KiCad clears the links of its copy in the constructor
  /// (`pcbnew/router/pns_shove.h:139`) and [`Shove::add_head_line`] does
  /// the same; [`Shove::run`] then adds it to the branch, which links it.
  /// [`None`] for a via head.
  orig_head: Option<Line>,
  /// What the shove made of it. Port of `newHead`, filled in by
  /// `reconstructHeads` (`pcbnew/router/pns_shove.cpp:2316`).
  new_head: Option<Line>,
  /// Where the dragged via is now. Port of `theVia`, which
  /// `reduceSpringback` also restores from a popped frame
  /// (`pcbnew/router/pns_shove.cpp:961`).
  the_via: Option<ViaHandle>,
  /// Where it was before this run. Port of `prevVia`, which the failure
  /// path of `Run` restores (`pcbnew/router/pns_shove.cpp:2578`).
  prev_via: Option<ViaHandle>,
  /// The via as this run's branch holds it. Port of `draggedVia`, written
  /// by `Run` (`pcbnew/router/pns_shove.cpp:2455`).
  dragged_via: Option<ItemId>,
  /// Where the caller wants the via. Port of `viaNewPos`.
  via_new_pos: Option<Vec2>,
  /// The root entry `Shove::run` created for this head.
  ///
  /// KiCad has no field for it: `reconstructHeads` calls
  /// `findRootLine( *headEntry.origHead )`
  /// (`pcbnew/router/pns_shove.cpp:2309`), which reads uids off the
  /// original head's links. By then the shove may have replaced the head
  /// and the node may have dropped those items, so KiCad is reading uids
  /// out of freed memory; here the handles simply stop resolving and the
  /// lookup finds nothing. The entry is therefore remembered when it is
  /// created, which is the same entry KiCad's lookup is trying to reach:
  /// `replaceLine` only ever re-points **new** links at it and never
  /// erases the old ones (`:157`).
  root_entry: Option<RootLineId>,
  /// Whether the shove moved it. Port of `geometryModified`.
  geometry_modified: bool,
  /// What may be done to it. Port of `policy`.
  policy: ShovePolicy,
}

// ---------------------------------------------------------------------
// Effects
// ---------------------------------------------------------------------

/// Whether the line an effect produced goes on the stack, and where.
///
/// The `pushLineStack` call every handler makes right after its
/// `replaceLine` (`pcbnew/router/pns_shove.cpp:676`, `:766`). It is part
/// of [`ShoveEffect::ReplaceLine`] rather than an effect of its own for
/// one reason: the new line's links do not exist until the replace has
/// been applied, so a separate push effect would carry a line with stale
/// links and `Shove::push_line_stack`'s "segments but no links" guard
/// would refuse it.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum PushMode {
  /// Do not stack the new line.
  None,
  /// Stack it on top, the ordinary case.
  Top,
  /// Stack it one slot below the top, KiCad's `aKeepCurrentOnTop`.
  ///
  /// `pushOrShoveVia`'s commented out `pushLineStack( lp.second, true )`
  /// at `pcbnew/router/pns_shove.cpp:1155` is the one site that ever
  /// asked for it; the shipped call passes false and its "WHY?" comment
  /// records the doubt. Nothing in this crate asks for it either, and it
  /// stays because [`LineStack::push_below_top`] is a named stack
  /// operation note 04 section 8.2 wants tested.
  BelowTop,
  /// Take the current top off the stack first, then stack the new line
  /// on top.
  ///
  /// The `popLineStack(); pushLineStack( walkaroundLine )` pair of
  /// `onCollidingSolid` (`pcbnew/router/pns_shove.cpp:892` and `:894`).
  /// It cannot be spelled as two effects: the line to push is the one
  /// the replace has just produced, so its links do not exist until the
  /// replace has been applied.
  PopThenTop,
}

/// What one [`ShoveEffect::ReplaceLine`] does.
///
/// The arguments of `replaceLine` (`pcbnew/router/pns_shove.cpp:83`)
/// together with the `SetRank` and `pushLineStack` its callers wrap it
/// in, which always travel together.
#[derive(Clone, PartialEq, Debug)]
pub struct LineReplacement {
  /// The line the node holds now. Its links say what to remove.
  pub old: Line,
  /// The line to put in its place.
  pub new: Line,
  /// The rank the new line and its stored segments carry.
  ///
  /// Applied **before** the replace, which is what `onCollidingSegment`
  /// does (`:669`) so that `NODE::Add` copies it onto each fresh segment
  /// (`pcbnew/router/pns_segment.h:70`). `onCollidingLine` sets it after
  /// instead (`:761`), which reaches the same items through
  /// `LINE::SetRank`'s propagation to links
  /// (`pcbnew/router/pns_line.cpp:1439`), so the two orders write the
  /// same rank to the same places and one of them serves both.
  pub rank: i32,
  /// Whether the change counts towards the area the optimizer is allowed
  /// to work in. KiCad's `aIncludeInChangedArea`.
  pub include_in_changed_area: bool,
  /// Whether the node may reuse an existing identical segment. KiCad's
  /// `aAllowRedundantSegments`.
  pub allow_redundant: bool,
  /// Where the new line goes on the stack.
  pub push: PushMode,
}

/// What is pushing a via out of the way.
///
/// The two branches of `onCollidingVia`'s `aCurrent`
/// (`pcbnew/router/pns_shove.cpp:1188` and `:1233`), which is an `ITEM*`
/// there and is only ever a `LINE` or a `SOLID`. The solid case is
/// reached from one place, `onCollidingSolid`'s role swap at `:800`.
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum ViaPusher<'a> {
  /// A line, KiCad's `LINE_T` branch.
  Line(&'a Line),
  /// A stored solid, KiCad's `SOLID_T` branch.
  Solid(ItemId),
}

/// What one [`ShoveEffect::MoveVia`] does.
///
/// The whole body of `pushOrShoveVia` from `SetRank` onwards
/// (`pcbnew/router/pns_shove.cpp:1107` to `:1166`), as data. It is one
/// effect rather than a via effect plus a line effect per branch for the
/// reason the module documentation gives: the dragged lines have to
/// **link** the via that the move produces, and that handle does not
/// exist until the move has been applied.
#[derive(Clone, PartialEq, Debug)]
pub struct ViaMovement {
  /// The via the node holds now.
  pub old_via: ItemId,
  /// The via to put in its place, already at its new position and
  /// carrying the old one's marker.
  pub new_via: Box<Item>,
  /// Every track attached to the via's joint, as it is and as it should
  /// be after being dragged.
  ///
  /// The `draggedLines` of `pcbnew/router/pns_shove.cpp:1081`, each pair
  /// reversed so that the via end is the **last** point (`:1096`).
  pub dragged: Vec<(Line, Line)>,
  /// The rank the via and every dragged line carry. KiCad's `aNewRank`.
  pub rank: i32,
  /// Whether the stacks are left alone. KiCad's `aDontUnwindStack`,
  /// which only the via drag head passes true (`:2459`).
  pub dont_unwind_stack: bool,
}

/// One change `Shove::shove_iteration` decided on.
///
/// The effect list of note 04 section 8.3. A handler computes what should
/// happen and says so; `Shove::shove_main_loop` is the only thing that
/// touches the node or the stacks. See the module documentation for why.
///
/// The list is applied in order and only when the iteration succeeded: a
/// failed iteration means [`Shove::run`] throws the whole branch away, so
/// applying half of it would be work with no observer. KiCad has no
/// choice about that, and does apply the first half.
#[derive(Clone, PartialEq, Debug)]
pub enum ShoveEffect {
  /// Take every line that references this item off both stacks, because
  /// the item is about to stop existing.
  ///
  /// Port of `unwindLineStack( const LINKED_ITEM* )`,
  /// `pcbnew/router/pns_shove.cpp:1390`. KiCad's `unwindLineStack( const
  /// ITEM* )` overload (`:1440`) is the same thing once per link of a
  /// line, which a handler spells as several of these.
  Unwind(ItemId),
  /// Swap a line in the node for a new one, rank it, and stack it.
  ///
  /// Port of `replaceLine` (`pcbnew/router/pns_shove.cpp:83`) plus the
  /// `SetRank` and `pushLineStack` around it. Boxed because it carries
  /// two whole lines and the other effects carry a handle.
  ReplaceLine(Box<LineReplacement>),
  /// Stack a line the node already holds.
  ///
  /// The `pushLineStack( revLine )` of the reverse collision branch,
  /// `pcbnew/router/pns_shove.cpp:1782`.
  PushLine {
    /// The line, which must already be linked into the node. Boxed for
    /// the same reason [`ShoveEffect::ReplaceLine`] is.
    line: Box<Line>,
    /// KiCad's `aKeepCurrentOnTop`.
    below_top: bool,
  },
  /// Take the top line off the stack **and** out of the optimizer queue.
  ///
  /// Port of `popLineStack`, `pcbnew/router/pns_shove.cpp:1509`.
  PopLine,
  /// Take the top line off the stack and leave it in the optimizer queue.
  ///
  /// Port of the raw `m_lineStack.pop_back()` at
  /// `pcbnew/router/pns_shove.cpp:1695`, the one an iteration that found
  /// no obstacle performs. Note 04 section 1.3 calls the asymmetry
  /// essential: this is how a line that has come clean reaches the
  /// optimizer at all.
  RetireLine,
  /// Move a via and drag every track attached to it along.
  ///
  /// Port of the mutating half of `pushOrShoveVia`,
  /// `pcbnew/router/pns_shove.cpp:1107` to `:1166`. See [`ViaMovement`].
  MoveVia(Box<ViaMovement>),
}

// ---------------------------------------------------------------------
// Shove
// ---------------------------------------------------------------------

/// The push and shove algorithm.
///
/// Port of `PNS::SHOVE`, `pcbnew/router/pns_shove.h:46`. See the module
/// documentation for the shape of one run and for what this revision
/// covers.
///
/// It owns no node. `root` and [`Shove::current_node`] are handles into
/// the caller's [`World`], which every method takes as a parameter, so
/// that a shove and a placer can stand on the same arena at once
/// (`DESIGN.md` section 6.4).
///
/// [`Clone`] is derived because the line placer holds one and is
/// cloneable itself; a cloned shove points at the same nodes, so only one
/// of the two copies may be run.
#[derive(Clone, Debug)]
pub struct Shove {
  /// The node the session started from. Port of `m_root`.
  root: NodeId,
  /// The node the caller should read now. Port of `m_currentNode`.
  current_node: NodeId,
  /// One frame per committed shove result, oldest first. Port of
  /// `m_nodeStack`.
  stack: Vec<SpringbackFrame>,
  /// The work stack of the current run. Port of `m_lineStack`.
  line_stack: LineStack,
  /// Every line this run touched. Port of `m_optimizerQueue`.
  optimizer_queue: OptimizerQueue,
  /// The caller's heads for this run. Port of `m_headLines`.
  heads: Vec<HeadEntry>,
  /// Every track this session has touched. Port of the
  /// `m_rootLineHistoryEntries` and `m_rootLineHistory` pair.
  root_lines: RootLineIndex,
  /// A frame springback may not drop. Port of
  /// `m_springbackDoNotTouchNode`.
  pinned: Option<NodeId>,
  /// A clearance that overrides every rule. Port of `m_forceClearance`,
  /// whose `-1` sentinel is [`None`] here.
  force_clearance: Option<i32>,
  /// Optimizer passes the caller has switched off. Port of
  /// `m_optFlagDisableMask`.
  optimizer_disable_mask: EffortFlags,
  /// Everything the current run changed. Port of `m_affectedArea`.
  affected_area: Option<Box2>,
  /// What the user can see, which the optimizer's area is clipped to.
  ///
  /// The one consumer of `ALGO_BASE::VisibleViewArea`
  /// (`pcbnew/router/pns_algo_base.h:83`) is `runOptimizer`
  /// (`pcbnew/router/pns_shove.cpp:2040`), so it lives here rather than
  /// on [`AlgoContext`] (see `src/algo_base.rs`). [`None`] is a headless
  /// caller, and then nothing is clipped.
  visible_view_area: Option<Box2>,
  /// Whether any head came out different. Port of `m_headsModified`.
  heads_modified: bool,
  /// How many iterations the last run used. Port of `m_iter`, which
  /// KiCad reads in its summary message (`:2557`).
  iterations: u32,
}

impl Shove {
  // -----------------------------------------------------------------
  // Construction and the caller's handles
  // -----------------------------------------------------------------

  /// A shove session over one node.
  ///
  /// Port of the constructor, `pcbnew/router/pns_shove.cpp:193`, which
  /// sets both the root and the current node to the world it is given.
  /// The debug decorator it installs there arrives per call through
  /// [`AlgoContext`] instead, and the default policy it sets is
  /// documented as not ported in the module documentation.
  pub fn new(root: NodeId) -> Self {
    Self {
      root,
      current_node: root,
      stack: Vec::new(),
      line_stack: LineStack::new(),
      optimizer_queue: OptimizerQueue::new(),
      heads: Vec::new(),
      root_lines: RootLineIndex::default(),
      pinned: None,
      force_clearance: None,
      optimizer_disable_mask: EffortFlags::NONE,
      affected_area: None,
      visible_view_area: None,
      heads_modified: false,
      iterations: 0,
    }
  }

  /// The node the session started from.
  pub const fn root(&self) -> NodeId {
    self.root
  }

  /// The node the caller should read.
  ///
  /// Port of `CurrentNode`, `pcbnew/router/pns_shove.cpp:2132`, whose
  /// `m_currentNode ? m_currentNode : m_root` cannot arise here because a
  /// [`NodeId`] is not nullable. Note that it does **not** consult the
  /// springback stack; KiCad's alternative is commented out at `:2134`.
  pub const fn current_node(&self) -> NodeId {
    self.current_node
  }

  /// The springback stack, oldest frame first.
  pub fn springback(&self) -> &[SpringbackFrame] {
    &self.stack
  }

  /// How many iterations the last [`Shove::run`] used.
  pub const fn iterations(&self) -> u32 {
    self.iterations
  }

  /// Override every clearance rule with one number.
  ///
  /// Port of `ForceClearance`, `pcbnew/router/pns_shove.h:91`, whose
  /// `bool` plus `-1` pair is one [`Option`] here. Its only user in
  /// KiCad's tree is the differential pair placer, which sets it so that
  /// the two members of a pair are shoved to exactly the coupled gap
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:247`).
  pub const fn set_force_clearance(&mut self, clearance: Option<i32>) {
    self.force_clearance = clearance;
  }

  /// Switch optimizer passes off for the post shove optimization.
  ///
  /// Port of `DisablePostShoveOptimizations`,
  /// `pcbnew/router/pns_shove.cpp:2217`, whose mask is subtracted from
  /// the effort level at `:2086`.
  pub const fn disable_post_shove_optimizations(&mut self, mask: EffortFlags) {
    self.optimizer_disable_mask = mask;
  }

  /// Pin a node so that springback may never drop it.
  ///
  /// Port of `SetSpringbackDoNotTouchNode`,
  /// `pcbnew/router/pns_shove.cpp:2223`. The comment at `:930` records
  /// why it exists: the line placer hands the shove an end item owned by
  /// a node in the stack, and dropping that node left the placer using
  /// freed memory. With arena handles the failure mode is a clean
  /// [`None`] instead of a crash, but the pin is still needed for the end
  /// item to stay resolvable at all (note 04 section 8.1).
  pub const fn set_pinned_node(&mut self, node: Option<NodeId>) {
    self.pinned = node;
  }

  /// What the user can see, which the post shove optimization is clipped
  /// to.
  ///
  /// `pcbnew/router/pns_shove.cpp:2040`. [`None`] leaves the optimizer
  /// free over everything the shove touched, which is the headless case.
  pub const fn set_visible_view_area(&mut self, area: Option<Box2>) {
    self.visible_view_area = area;
  }

  // -----------------------------------------------------------------
  // Heads
  // -----------------------------------------------------------------

  /// Forget the heads of the previous run.
  ///
  /// Port of `ClearHeads`, `pcbnew/router/pns_shove.cpp:2248`.
  pub fn clear_heads(&mut self) {
    self.heads.clear();
  }

  /// Add one line for the next run to route.
  ///
  /// Port of `AddHeads( const LINE&, int )`,
  /// `pcbnew/router/pns_shove.cpp:2254`. The line's links are cleared, as
  /// the entry's constructor does (`pcbnew/router/pns_shove.h:139`),
  /// because [`Shove::run`] adds it to a fresh branch and the links it
  /// arrives with belong to some other node.
  ///
  /// # Deviation
  ///
  /// KiCad also calls `SetShovePolicy( aHead, aPolicy )` here (`:2257`).
  /// That call reaches `touchRootLine` on the **caller's** line, whose
  /// links are either absent or stale, so it allocates a root entry that
  /// is indexed under nothing and can never be found again; the policy
  /// that is actually read is the one [`Shove::run`] writes onto the
  /// head's real root entry (`:2515`). The dead allocation is not
  /// reproduced.
  pub fn add_head_line(&mut self, head: Line, policy: ShovePolicy) {
    let mut orig_head = head;

    orig_head.clear_links();

    self.heads.push(HeadEntry {
      orig_head: Some(orig_head),
      new_head: None,
      the_via: None,
      prev_via: None,
      dragged_via: None,
      via_new_pos: None,
      root_entry: None,
      geometry_modified: false,
      policy,
    });
  }

  /// Add one via for the next run to drag to a new position.
  ///
  /// Port of `AddHeads( VIA_HANDLE, VECTOR2I, int )`,
  /// `pcbnew/router/pns_shove.cpp:2261`. Note that it does **not** call
  /// `SetShovePolicy`, where the line overload does; the policy is
  /// carried on the entry and reaches the via's root entry through
  /// `Shove::add_via_head_to_node`.
  ///
  /// The via is named by a [`ViaHandle`] rather than by an [`ItemId`]
  /// because [`Shove::run`] branches a fresh node and looks the via up
  /// again there: the handle the caller holds names an item of some other
  /// node.
  pub fn add_head_via(
    &mut self,
    via: ViaHandle,
    new_pos: Vec2,
    policy: ShovePolicy,
  ) {
    self.heads.push(HeadEntry {
      orig_head: None,
      new_head: None,
      the_via: Some(via),
      prev_via: Some(via),
      dragged_via: None,
      via_new_pos: Some(new_pos),
      root_entry: None,
      geometry_modified: false,
      policy,
    });
  }

  /// Where a via head stands now.
  ///
  /// The `theVia` of one head entry
  /// (`pcbnew/router/pns_shove.h:141`), which is what a via dragger reads
  /// back after a run: the shove replaces the via item, so the caller's
  /// old handle names nothing.
  pub fn head_via(&self, index: usize) -> Option<ViaHandle> {
    self.heads.get(index)?.the_via
  }

  /// Whether the shove moved a head, or any head at all.
  ///
  /// Port of `HeadsModified`, `pcbnew/router/pns_shove.cpp:2638`, whose
  /// `aIndex < 0` default is [`None`] here.
  pub fn heads_modified(&self, index: Option<usize>) -> bool {
    match index {
      None => self.heads_modified,
      Some(index) => self
        .heads
        .get(index)
        .is_some_and(|head| head.geometry_modified),
    }
  }

  /// The shape a head came out with.
  ///
  /// Port of `GetModifiedHead`, `pcbnew/router/pns_shove.cpp:2646`, which
  /// dereferences its optional without checking that it is engaged. This
  /// answers [`None`] for the same case: a head the shove never touched
  /// has no new shape, and the caller keeps the one it handed in.
  pub fn modified_head(&self, index: usize) -> Option<&Line> {
    self.heads.get(index)?.new_head.as_ref()
  }

  // -----------------------------------------------------------------
  // The springback stack
  // -----------------------------------------------------------------

  /// Push a frame that springback may never drop.
  ///
  /// Port of `AddLockedSpringbackNode`,
  /// `pcbnew/router/pns_shove.cpp:2138`, which pushes a tag with no
  /// affected area and no via snapshot. The line placer calls it whenever
  /// it fixes a segment, so that springback can never roll the world back
  /// past a committed decision (`pcbnew/router/pns_line_placer.cpp:1626`).
  ///
  /// Always answers true, as KiCad's does.
  pub fn add_locked_springback_node(&mut self, node: NodeId) -> bool {
    self.stack.push(SpringbackFrame {
      node,
      affected_area: None,
      locked: true,
      dragged_vias: Vec::new(),
    });

    true
  }

  /// Let springback drop a frame again.
  ///
  /// Port of `UnlockSpringbackNode`,
  /// `pcbnew/router/pns_shove.cpp:2200`, a linear scan that clears the
  /// flag on the first match and stops.
  pub fn unlock_springback_node(&mut self, node: NodeId) {
    if let Some(frame) = self.stack.iter_mut().find(|frame| frame.node == node)
    {
      frame.locked = false;
    }
  }

  /// Roll the world back to a frame, dropping it and everything above.
  ///
  /// Port of `RewindSpringbackTo`, `pcbnew/router/pns_shove.cpp:2152`.
  /// Answers false when the node is not on the stack.
  ///
  /// Two things are worth knowing, and both are KiCad's. The frame that
  /// is rewound **to** is erased along with the ones above it (`:2175`),
  /// so [`Shove::current_node`] ends up at the frame below it, or at the
  /// root. And the node itself is not dropped: only its children are
  /// (`:2174`), which is what takes the erased frames' nodes with it.
  /// That is why the line placer's `RewindSpringbackTo` followed by
  /// `UnlockSpringbackNode` on the same node (`:1789`) unlocks nothing.
  pub fn rewind_springback_to(
    &mut self,
    world: &mut World,
    node: NodeId,
  ) -> bool {
    let Some(at) = self.stack.iter().position(|frame| frame.node == node)
    else {
      return false;
    };

    world.kill_children(node);
    self.stack.truncate(at);
    self.current_node = self.stack.last().map_or(self.root, |frame| frame.node);

    true
  }

  /// Roll the world back to the newest locked frame.
  ///
  /// Port of `RewindToLastLockedNode`,
  /// `pcbnew/router/pns_shove.cpp:2186`, which pops until the top is
  /// locked or one frame is left, and answers whether that top is
  /// actually locked. An empty stack answers false and changes nothing.
  ///
  /// The popped frames' nodes are not dropped, in KiCad as here: they are
  /// children of the surviving one and go with it.
  pub fn rewind_to_last_locked_node(&mut self) -> bool {
    let Some(frame) = self.stack.last() else {
      return false;
    };

    let mut locked = frame.locked;

    while !locked && self.stack.len() > 1 {
      self.stack.pop();
      locked = self.stack.last().is_some_and(|frame| frame.locked);
    }

    if let Some(frame) = self.stack.last() {
      self.current_node = frame.node;

      return frame.locked;
    }

    false
  }

  /// Record the world after a shove.
  ///
  /// Port of `pushSpringback`, `pcbnew/router/pns_shove.cpp:983`. The new
  /// frame's area is the union of the area handed in and the previous
  /// frame's, so the optimizer's restriction covers everything the whole
  /// session has moved. Always answers true, as KiCad's does.
  ///
  /// The per head via snapshot (`:1006`) is why KiCad's doc comment at
  /// `:2568` insists this runs **after** `reconstructHeads`: it stores
  /// the handles that call has just refreshed, so that popping the frame
  /// puts the caller's vias back where this frame found them.
  fn push_springback(&mut self, node: NodeId, affected_area: Option<Box2>) {
    let previous = self.stack.last().and_then(|frame| frame.affected_area);

    self.stack.push(SpringbackFrame {
      node,
      affected_area: match (previous, affected_area) {
        (Some(previous), Some(area)) => Some(previous.merge(area)),
        (Some(previous), None) => Some(previous),
        (None, area) => area,
      },
      locked: false,
      // :989 and :994.
      dragged_vias: self.heads.iter().map(|head| head.the_via).collect(),
    });
  }

  /// Drop the frames whose world no longer stands in the heads' way.
  ///
  /// Port of `reduceSpringback`, `pcbnew/router/pns_shove.cpp:924`, and
  /// the whole point of the stack: a shove is undone by discarding a
  /// frame rather than by being recomputed. Answers the node the next
  /// branch should come off.
  ///
  /// The bottom frame is never dropped, because the loop runs while more
  /// than one frame is left (`:926`). A session whose stack holds exactly
  /// one frame therefore keeps that shove until something else rewinds
  /// it; that is KiCad's behaviour and the line placer works around it by
  /// pushing a locked frame whenever it commits
  /// (`pcbnew/router/pns_line_placer.cpp:1626`).
  ///
  /// The surviving frame's via snapshot is copied back onto the heads
  /// (`:958`), which is how a via drag that the mouse retreated from ends
  /// up reporting the via where the frame below left it rather than where
  /// the dropped frame had pushed it.
  fn reduce_springback(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    head_set: &[Line],
  ) -> NodeId {
    while self.stack.len() > 1 {
      let Some(frame) = self.stack.last() else {
        break;
      };

      // :932. The pin the line placer sets around its end item.
      if Some(frame.node) == self.pinned {
        break;
      }

      if frame.locked
        || self.node_collides_with_heads(world, context, frame.node, head_set)
      {
        break;
      }

      let node = frame.node;

      // :941 and :944.
      self.prune_root_lines(world, node);
      world.drop_node(node);
      self.stack.pop();
    }

    // :952
    let Some(frame) = self.stack.last() else {
      return self.root;
    };

    // :958
    let restored: Vec<Option<ViaHandle>> = frame.dragged_vias.clone();

    for (index, via) in restored.into_iter().enumerate() {
      let (Some(via), Some(head)) = (via, self.heads.get_mut(index)) else {
        continue;
      };

      head.the_via = Some(via);
      head.prev_via = Some(via);
      head.geometry_modified = true;
    }

    self.stack.last().map_or(self.root, |frame| frame.node)
  }

  /// Whether any head still collides with what a frame's world holds.
  ///
  /// The `spTag.m_node->CheckColliding( aHeadSet )` of `reduceSpringback`
  /// (`pcbnew/router/pns_shove.cpp:935`), which is `NODE::CheckColliding(
  /// const ITEM_SET& )` (`pcbnew/router/pns_node.cpp:478`): one query per
  /// member, stopping at the first hit, with a limit count of one.
  fn node_collides_with_heads(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    head_set: &[Line],
  ) -> bool {
    let options = CollisionSearchOptions {
      limit_count: Some(1),
      ..CollisionSearchOptions::default()
    };

    head_set.iter().any(|head| {
      world
        .check_colliding_line(node, head, context.resolver, &options)
        .is_some()
    })
  }

  // -----------------------------------------------------------------
  // The root line index
  // -----------------------------------------------------------------

  /// The entry that stands for a line's track, if there is one.
  ///
  /// Port of `findRootLine( const LINE& )`,
  /// `pcbnew/router/pns_shove.cpp:1951`.
  fn find_root_line(&self, world: &World, line: &Line) -> Option<RootLineId> {
    self.root_lines.find(&uids_of(world, line))
  }

  /// The entry that stands for a line's track, creating one if needed.
  ///
  /// Port of `touchRootLine( const LINE& )`,
  /// `pcbnew/router/pns_shove.cpp:1975`, which creates the entry with a
  /// clone of the line as its root shape and indexes it under every one
  /// of the line's links.
  ///
  /// A line with no links gets an entry indexed under nothing, exactly as
  /// in KiCad. It is still a usable key for the optimizer queue, it is
  /// just one nothing else can find again.
  fn touch_root_line(&mut self, world: &World, line: &Line) -> RootLineId {
    let uids = uids_of(world, line);

    if let Some(id) = self.root_lines.find(&uids) {
      return id;
    }

    self.root_lines.alloc(
      Some(root_shape_of(line)),
      ShovePolicy::DEFAULT,
      &uids,
    )
  }

  /// The entry that stands for one stored item, creating one if needed.
  ///
  /// Port of `touchRootLine( const LINKED_ITEM* )`,
  /// `pcbnew/router/pns_shove.cpp:2003`, which creates the entry with
  /// **no** root shape: a via has no pre shove polyline to compare
  /// against, only a position.
  fn touch_root_line_of_item(
    &mut self,
    world: &World,
    item: ItemId,
  ) -> RootLineId {
    let Some(uid) = world.item(item).map(Item::uid) else {
      return self.root_lines.alloc(None, ShovePolicy::DEFAULT, &[]);
    };

    if let Some(id) = self.root_lines.find_by_uid(uid) {
      return id;
    }

    self.root_lines.alloc(None, ShovePolicy::DEFAULT, &[uid])
  }

  /// Drop the index entries of everything a node added.
  ///
  /// Port of `pruneRootLines`, `pcbnew/router/pns_shove.cpp:901`, called
  /// on both paths that destroy a node: the springback pop (`:941`) and
  /// the failure path of [`Shove::run`] (`:2598`). It touches the index
  /// only, never the entries, so an entry an aliasing uid still names
  /// stays reachable.
  fn prune_root_lines(&mut self, world: &World, node: NodeId) {
    let (added, _removed) = world.get_updated_items(node);

    for item in added {
      let Some(stored) = world.item(item) else {
        continue;
      };

      if stored.of_kind(Kind::LINKED_ITEM_MASK) {
        self.root_lines.forget(stored.uid());
      }
    }
  }

  // -----------------------------------------------------------------
  // The stacks
  // -----------------------------------------------------------------

  /// Stack a line and refresh its optimizer queue entry.
  ///
  /// Port of `pushLineStack`, `pcbnew/router/pns_shove.cpp:1456`.
  /// Answers false for a line that has segments but no links, which is
  /// the guard that turns "we built a line the node never took" into
  /// [`ShoveStatus::Incomplete`] rather than into a later surprise.
  fn push_line_stack(
    &mut self,
    world: &World,
    line: Line,
    keep_current_on_top: bool,
  ) -> bool {
    // :1458
    if !line.is_linked() && line.segment_count() != 0 {
      return false;
    }

    let key = self.touch_root_line(world, &line);

    if keep_current_on_top {
      self.line_stack.push_below_top(line.clone());
    } else {
      self.line_stack.push(line.clone());
    }

    // :1475
    self.optimizer_queue.insert(key, line);

    true
  }

  /// Take the top line off the stack and out of the optimizer queue.
  ///
  /// Port of `popLineStack`, `pcbnew/router/pns_shove.cpp:1509`.
  fn pop_line_stack(&mut self, world: &World) {
    let Some(line) = self.line_stack.pop() else {
      return;
    };

    if let Some(key) = self.find_root_line(world, &line) {
      self.optimizer_queue.prune(key);
    }
  }

  /// Take every line that references an item off both stacks.
  ///
  /// Port of `unwindLineStack( const LINKED_ITEM* )`,
  /// `pcbnew/router/pns_shove.cpp:1390`.
  fn unwind_line_stack(&mut self, world: &World, item: ItemId) {
    let is_via = world.item(item).is_some_and(|item| item.of_kind(Kind::VIA));

    self.line_stack.unwind(world, item);
    self.optimizer_queue.retain_not_referencing(item, is_via);
  }

  /// Take every line that references any link of a line off both stacks.
  ///
  /// Port of the `LINE_T` branch of `unwindLineStack( const ITEM* )`,
  /// `pcbnew/router/pns_shove.cpp:1447`, which is the per link overload
  /// once per link.
  fn unwind_line(&mut self, world: &World, line: &Line) {
    for link in line.links().to_vec() {
      self.unwind_line_stack(world, link);
    }
  }

  // -----------------------------------------------------------------
  // The node
  // -----------------------------------------------------------------

  /// Swap a line in the node for another, and keep the root line index
  /// pointing at the right entry.
  ///
  /// Port of `replaceLine`, `pcbnew/router/pns_shove.cpp:83`, which is
  /// both the node mutator and the bookkeeper: it finds the entry the old
  /// line already belongs to, or creates one from the old line, and then
  /// re-points every one of the new line's links at that same entry so
  /// that the pre shove shape survives any number of further shoves.
  ///
  /// The via unlink at `:99` is what stops a shove from deleting a via:
  /// [`World::remove_line`] takes every link out of the node, via links
  /// included, so the via has to leave the old line's link list before
  /// the replace runs.
  fn replace_line(
    &mut self,
    world: &mut World,
    node: NodeId,
    old: &mut Line,
    new: &mut Line,
    include_in_changed_area: bool,
    allow_redundant: bool,
  ) -> RootLineId {
    // :85
    if include_in_changed_area && let Some(area) = old.changed_area(new) {
      self.affected_area = Some(match self.affected_area {
        Some(previous) => previous.merge(area),
        None => area,
      });
    }

    // :99. The via link comes off the old line so that the replace does
    // not take the via out of the node with the segments.
    if old.ends_with_via()
      && let Some(via) = old.links().iter().copied().find(|link| {
        world
          .item(*link)
          .is_some_and(|item| item.of_kind(Kind::VIA))
      })
    {
      old.unlink(via);
    }

    // :124. The old line's uids have to be read before the replace, which
    // clears its links.
    let old_uids = uids_of(world, old);
    let entry = match self.root_lines.find(&old_uids) {
      Some(entry) => entry,
      // :137
      None => self.root_lines.alloc(
        Some(root_shape_of(old)),
        ShovePolicy::DEFAULT,
        &old_uids,
      ),
    };

    // :151
    world.replace_line(node, old, new, allow_redundant);

    // :157, as one operation over the new links (note 04 section 8.3).
    self.root_lines.bind(entry, &uids_of(world, new));
    // :163
    self.root_lines.entry_mut(entry).new_line = Some(new.clone());

    entry
  }

  // -----------------------------------------------------------------
  // Geometry
  // -----------------------------------------------------------------

  /// The clearance two items need from each other.
  ///
  /// Port of `getClearance`, `pcbnew/router/pns_shove.cpp:169`. The
  /// forced clearance short circuits the resolver entirely, which is the
  /// whole reason the function exists, and the two hole terms at `:177`
  /// and `:180` raise the answer to whatever the holes need: a via has to
  /// be hulled so that the hull clears the **hole** as well as the pad.
  ///
  /// `-1` is KiCad's "these two can never collide" arriving in an `int`
  /// and being used as a length regardless; the hull it sizes is only
  /// ever compared with other hulls.
  fn clearance_between_items(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    first: ItemRef<'_>,
    second: ItemRef<'_>,
  ) -> i32 {
    if let Some(forced) = self.force_clearance {
      return forced;
    }

    let ask = |a: ItemRef<'_>, b: ItemRef<'_>| {
      context.resolver.clearance(a, Some(b), false).unwrap_or(-1)
    };
    let mut clearance = ask(first, second);

    // :176
    if let Some(hole) = first.item().hole()
      && let Some(stored) = world.item(hole)
    {
      clearance = clearance.max(ask(ItemRef::stored(hole, stored), second));
    }

    // :179
    if let Some(hole) = second.item().hole()
      && let Some(stored) = world.item(hole)
    {
      clearance = clearance.max(ask(first, ItemRef::stored(hole, stored)));
    }

    clearance
  }

  /// The clearance two lines need from each other.
  ///
  /// [`Shove::clearance_between_items`] over the two rule probes, which
  /// is the `getClearance( &aCurLine, &obstacleLine )` of
  /// `ShoveObstacleLine` (`pcbnew/router/pns_shove.cpp:570`). A line owns
  /// no hole, so neither hole term can contribute.
  fn clearance_between_lines(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    first: &Line,
    second: &Line,
  ) -> i32 {
    let first_item = first.rule_item(world, PROBE_UID);
    let second_item = second.rule_item(world, PROBE_UID);

    self.clearance_between_items(
      world,
      context,
      ItemRef::unstored(&first_item),
      ItemRef::unstored(&second_item),
    )
  }

  /// The clearance a via needs from a line, with the hole as the binding
  /// constraint when it is one.
  ///
  /// The `:604` to `:612` block of `ShoveObstacleLine`, repeated at
  /// `:286` to `:290` inside `shoveLineFromLoneVia`: if the hole
  /// clearance plus the hole radius reaches further than the pad
  /// clearance plus the pad radius, the pad clearance is rewritten so
  /// that a hull built around the **pad** still clears the hole.
  ///
  /// `via` is an [`ItemRef`] rather than an [`ItemId`] because the pusher's
  /// via may be one the line owns and no node holds.
  fn via_clearance_against_line(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    via: ItemRef<'_>,
    obstacle: &Line,
  ) -> i32 {
    let probe = obstacle.rule_item(world, PROBE_UID);
    let mut clearance = self.clearance_between_items(
      world,
      context,
      via,
      ItemRef::unstored(&probe),
    );

    let ItemBody::Via(body) = via.item().body() else {
      return clearance;
    };

    let Some((hole_id, hole)) = via
      .item()
      .hole()
      .and_then(|id| world.item(id).map(|item| (id, item)))
    else {
      return clearance;
    };

    let hole_clearance = context
      .resolver
      .clearance(
        ItemRef::stored(hole_id, hole),
        Some(ItemRef::unstored(&probe)),
        false,
      )
      .unwrap_or(-1);
    let diameter = body.diameter(via.item().layers(), obstacle.layer());

    // :609
    if hole_clearance + body.drill() / 2 > clearance + diameter / 2 {
      clearance = hole_clearance + body.drill() / 2 - diameter / 2;
    }

    clearance
  }

  /// Whether a candidate was shoved the right way.
  ///
  /// Port of `checkShoveDirection`,
  /// `pcbnew/router/pns_shove.cpp:243`. It closes a region out of the
  /// obstacle's old shape and its new one walked backwards, and asks
  /// whether the pusher's reference point ends up inside it. Inside means
  /// the obstacle wrapped **around** the pusher, which is the wrong way.
  ///
  /// The three comment archaeology blocks at `:234` admit this is a
  /// heuristic: an open curve has no orientation, so which end of the
  /// pusher counts as the pusher had to become an external hint. That
  /// hint is [`ShovePolicy::REVERSED`], which corner dragging sets.
  fn check_shove_direction(
    &self,
    world: &World,
    cur_line: &Line,
    obstacle_line: &Line,
    shoved_line: &Line,
  ) -> bool {
    // :248. A lone via has no points at all, so its centre is the
    // reference; `Shove::on_reverse_colliding_via` builds exactly that
    // shape at `:1348`.
    let lone_via = cur_line.point_count() == 0 && cur_line.ends_with_via();

    let mut reference = if lone_via {
      match cur_line.via_pos(world) {
        Some(pos) => pos,
        None => return false,
      }
    } else if cur_line.point_count() > 0 {
      cur_line.point(0)
    } else {
      return false;
    };

    // :253
    if !lone_via
      && self.find_root_line(world, cur_line).is_some_and(|root| {
        self
          .root_lines
          .entry(root)
          .policy
          .contains(ShovePolicy::REVERSED)
      })
      && let Some(last) = cur_line.last_point()
    {
      reference = last;
    }

    let mut checker = PointInsideTracker::new(reference);

    checker.add_polyline(obstacle_line.shape());
    checker.add_polyline(&shoved_line.shape().reversed());

    !checker.is_inside()
  }

  /// Re-walk an obstacle around a set of hulls.
  ///
  /// Port of `shoveLineToHullSet`,
  /// `pcbnew/router/pns_shove.cpp:328`, where the "which way round"
  /// decision actually happens. There is no analytic answer, so it is a
  /// brute force search over four combinations, in this order: clockwise,
  /// counter clockwise, then both again with the hull list walked back to
  /// front (`:338`). Hull order matters because walking around A and then
  /// B is not the same as B and then A when the two overlap, which is
  /// what a via grid or a row of pads produces.
  ///
  /// A candidate has to survive four independent checks: it keeps the
  /// obstacle's first and last points (`:432`), it points away from the
  /// pusher (`:466`), it does not intersect itself (`:474`), and it does
  /// not collide with the pusher (`:481`). The first that survives all
  /// four wins.
  ///
  /// # Deviation
  ///
  /// KiCad writes a direction rejected candidate into its out parameter
  /// before moving on (`:470`), so a caller that ignores the return value
  /// can read a shape that failed. `onCollidingArc` is exactly such a
  /// caller (note 04 section 1.7); it is `TODO(arcs)`, and every caller
  /// here reads the shape only on success, so the wart has no
  /// counterpart.
  fn shove_line_to_hull_set(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    cur_line: &Line,
    obstacle_line: &Line,
    hulls: &[LineChain],
    adjust: EndpointAdjustment,
  ) -> Option<LineChain> {
    // :338
    for attempt in 0..4 {
      let invert_traversal = attempt >= 2;
      let clockwise = attempt % 2 == 1;
      let mut walked = obstacle_line.clone();

      // :347. Let an endpoint that all but sits on a hull move onto it,
      // which is what makes the third expansion attempt worth running.
      if adjust.any() && walked.segment_count() >= 1 {
        let start =
          nearest_hull_point(hulls, invert_traversal, walked.point(0));
        let end = walked
          .last_point()
          .and_then(|last| nearest_hull_point(hulls, invert_traversal, last));

        // :385. The end first, so that the insert at the start does not
        // move the index the append would have used.
        if adjust.end
          && let Some(point) = end
        {
          walked.chain_mut().append(point);
        }

        if adjust.start
          && let Some(point) = start
        {
          walked.chain_mut().insert(0, point);
        }
      }

      let obstacle_chain = walked.shape().clone();
      let mut path = walked.shape().clone();
      let mut walk_failed = false;

      // :406
      for index in 0..hulls.len() {
        let hull = &hulls[if invert_traversal {
          hulls.len() - 1 - index
        } else {
          index
        }];

        // :414
        let Some(next) = walked.walkaround(hull, clockwise) else {
          walk_failed = true;
          break;
        };

        path = next;
        // :425
        path.simplify2(true);
        walked.set_shape(path.clone());
      }

      if walk_failed {
        continue;
      }

      // KiCad reads `CPoint( 0 )` below without checking; an empty walk
      // result would be a panic here, so it is rejected instead.
      if path.point_count() == 0 || obstacle_chain.point_count() == 0 {
        continue;
      }

      // :432 and :440, the first and the last vertex at which the walked
      // path and the obstacle disagree.
      let leading = (0..path.point_count().min(obstacle_chain.point_count()))
        .find(|index| path.point(*index) != obstacle_chain.point(*index));
      let trailing = (0..path.point_count().min(obstacle_chain.point_count()))
        .find(|offset| {
          path.point(path.point_count() - 1 - offset)
            != obstacle_chain.point(obstacle_chain.point_count() - 1 - offset)
        });

      // :448
      if (leading.is_none() || trailing.is_none())
        && !path.compare_geometry(&obstacle_chain)
      {
        continue;
      }

      // :455
      if path.last_point() != obstacle_chain.last_point()
        || path.point(0) != obstacle_chain.point(0)
      {
        continue;
      }

      // :466
      if !self.check_shove_direction(world, cur_line, obstacle_line, &walked) {
        continue;
      }

      // :474
      if path.self_intersecting().is_some() {
        continue;
      }

      // :481
      if world
        .collide_lines(
          &walked,
          cur_line,
          context.resolver,
          &CollisionSearchOptions::default(),
        )
        .is_some()
      {
        continue;
      }

      // :503
      return Some(path);
    }

    None
  }

  /// Push one line away from another by the clearance.
  ///
  /// Port of `ShoveObstacleLine`,
  /// `pcbnew/router/pns_shove.cpp:521`. It builds one hull per segment of
  /// the **pusher**, at the clearance plus half the obstacle's width so
  /// that re-walking the obstacle's centreline around them puts the
  /// obstacle's edge exactly at the clearance (the convention is spelled
  /// out at `:565`), and then hands the set to
  /// `Shove::shove_line_to_hull_set`.
  ///
  /// Three attempts: plain, hulls inflated by
  /// `HULL_FAILURE_EXPANSION`, and inflated twice with permission to
  /// move the obstacle's endpoints, but only endpoints that do not carry
  /// a via (`:617`).
  ///
  /// Public because it is public in KiCad, where the differential pair
  /// placer uses it as a geometric primitive outside any run
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:251`).
  ///
  /// `TODO(arcs)`: the arc clearance bump at `:593`, whose accumulation
  /// across several arc segments note 04 section 2.3 flags as probably
  /// unintended.
  pub fn shove_obstacle_line(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    cur_line: &Line,
    obstacle_line: &Line,
  ) -> Option<Line> {
    // :531. Whether either end of the obstacle is pinned by a via.
    let mut via_on_start = false;
    let mut via_on_end = false;

    if obstacle_line.point_count() >= 2 {
      let layer = obstacle_line.layer();
      let net = obstacle_line.net();

      via_on_start =
        joint_has_via(world, node, obstacle_line.point(0), layer, net);
      via_on_end = obstacle_line
        .last_point()
        .is_some_and(|last| joint_has_via(world, node, last, layer, net));
    }

    // :542
    let mut result = obstacle_line.clone();

    result.clear_links();

    // :548. The obstacle's own via is taken off for the walk and put back
    // afterwards, because the hull set is about the pusher's shape only.
    let mut obstacle = obstacle_line.clone();
    let obstacle_via = obstacle.via().cloned();

    if obstacle_via.is_some() {
      obstacle.remove_via();
    }

    // :545 and :558. Only the pusher's via reaches the obstacle's layer,
    // so the hull set collapses to that one via hull.
    let pusher_has_via = cur_line.ends_with_via();

    if pusher_has_via
      && (!cur_line.layers().overlaps(obstacle.layers())
        || cur_line.segment_count() == 0)
    {
      let shape =
        self.shove_line_from_lone_via(world, context, cur_line, &obstacle)?;

      result.set_shape(shape);

      return Some(result);
    }

    // :570
    let clearance =
      self.clearance_between_lines(world, context, cur_line, &obstacle);
    let mut extra_expansion = 0;

    // :578
    for attempt in 0..HULL_EXPANSION_ATTEMPTS {
      let mut hulls = Vec::with_capacity(cur_line.segment_count());

      // :583
      for index in 0..cur_line.segment_count() {
        let segment = cur_line.segment_item(world, index, PROBE_UID);

        // :596
        hulls.push(segment.hull(
          clearance + extra_expansion,
          obstacle.width(),
          obstacle.layer(),
        ));
      }

      // :601. The pusher's own via, hulled last so that the hull set
      // stays ordered along the pusher.
      if pusher_has_via && let Some(via) = cur_line.via_item(world) {
        let via_clearance =
          self.via_clearance_against_line(world, context, via, &obstacle);

        hulls.push(via.item().hull(
          via_clearance,
          obstacle.width(),
          obstacle.layer(),
        ));
      }

      // :617
      let adjust = EndpointAdjustment {
        start: attempt >= 2 && !via_on_start,
        end: attempt >= 2 && !via_on_end,
      };

      if let Some(shape) = self.shove_line_to_hull_set(
        world, context, cur_line, &obstacle, &hulls, adjust,
      ) {
        result.set_shape(shape);

        // :622
        if let Some(via) = obstacle_via {
          restore_via(&mut result, via);
        }

        return Some(result);
      }

      // :628
      extra_expansion += HULL_FAILURE_EXPANSION;
    }

    None
  }

  /// Push a line away from a via that is the only thing in its way.
  ///
  /// Port of `shoveLineFromLoneVia`,
  /// `pcbnew/router/pns_shove.cpp:278`, used when the pusher's segments
  /// are on another layer or it has none, so only its via matters. One
  /// hull, walked both ways, clockwise preferred unless
  /// `Shove::check_shove_direction` rejects it, then the same endpoint
  /// and collision checks the hull set path applies.
  ///
  /// Answers the walked chain, or [`None`] when either walk failed or a
  /// check rejected the result.
  ///
  /// # A KiCad wart, reproduced
  ///
  /// `ShoveObstacleLine` re-appends the obstacle's own via only on its
  /// **other** branch (`:622`), so an obstacle that ends with a via loses
  /// it on this path. That is the shipped behaviour and the caller keeps
  /// it.
  fn shove_line_from_lone_via(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    cur_line: &Line,
    obstacle_line: &Line,
  ) -> Option<LineChain> {
    let via = cur_line.via_item(world)?;
    // :286 to :290, the hole as the binding constraint.
    let clearance =
      self.via_clearance_against_line(world, context, via, obstacle_line);
    // :292
    let hull =
      via
        .item()
        .hull(clearance, obstacle_line.width(), cur_line.layer());

    // :296 and :299.
    let clockwise = obstacle_line.walkaround(&hull, true)?;
    let counter_clockwise = obstacle_line.walkaround(&hull, false)?;

    // :302. Clockwise unless it went the wrong way.
    let mut candidate = obstacle_line.clone();

    candidate.set_shape(clockwise);

    if !self.check_shove_direction(world, cur_line, obstacle_line, &candidate) {
      candidate.set_shape(counter_clockwise);
    }

    // :309
    if candidate.point_count() < 2 {
      return None;
    }

    // :312 and :315.
    if candidate.last_point() != obstacle_line.last_point()
      || candidate.point(0) != obstacle_line.point(0)
    {
      return None;
    }

    // :318
    if world
      .collide_lines(
        &candidate,
        cur_line,
        context.resolver,
        &CollisionSearchOptions::default(),
      )
      .is_some()
    {
      return None;
    }

    Some(candidate.shape().clone())
  }

  // -----------------------------------------------------------------
  // The per obstacle handlers
  // -----------------------------------------------------------------

  /// Resolve a collision with a stored segment by pushing its line aside.
  ///
  /// Port of `onCollidingSegment`,
  /// `pcbnew/router/pns_shove.cpp:639`. The obstacle's whole line is
  /// assembled first, so the shove moves a track rather than a fragment
  /// of one, and the assembly stops at locked joints, which is how the
  /// head's pinned endpoints restrain the algorithm at all (note 04
  /// section 1.7, `assembleLine`).
  ///
  /// # Deviation: the pre shove cleanup is folded in
  ///
  /// KiCad's `assembleLine( ..., aPreCleanup = true )` runs
  /// `preShoveCleanup` (`:2368`), which simplifies the assembled line and
  /// **replaces it in the node** before the shove even starts, creating
  /// the root entry from the raw line and immediately overwriting its new
  /// shape. Here the simplification is applied to the line the shove
  /// reasons about, while the replace effect still carries the raw
  /// assembled line as the thing to remove.
  ///
  /// The reachable end state is identical: the node ends up holding the
  /// shoved geometry, the root entry is still a clone of the raw line and
  /// its new shape is still the shoved one. The reason to fold them is
  /// the effect list: the intermediate line's links do not exist until
  /// the first replace has been applied, so an effect list that carried
  /// it would carry stale handles. The two paths differ only in the
  /// changed area contributed by a failed shove, and a failed shove
  /// throws the whole branch away.
  fn on_colliding_segment(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    current: &Line,
    obstacle_segment: ItemId,
  ) -> (ShoveStatus, Vec<ShoveEffect>) {
    // :643
    let obstacle_line =
      world.assemble_line(node, obstacle_segment, None, true, false, true);

    // :2374, the folded pre shove cleanup.
    let mut cleaned = obstacle_line.shape().clone();
    let before = cleaned.point_count();

    cleaned.simplify2(true);

    let mut shove_input = obstacle_line.clone();

    if cleaned.point_count() != before {
      shove_input.set_shape(cleaned);
    }

    // :647
    if shove_input.has_locked_segments(world) {
      trace(context, || "shove: try walk (locked segments)".to_string());

      return (ShoveStatus::TryWalk, Vec::new());
    }

    // :653
    let Some(mut shoved) =
      self.shove_obstacle_line(world, context, node, current, &shove_input)
    else {
      return (ShoveStatus::Incomplete, Vec::new());
    };

    // :669
    let rank = current.rank(world) - 1;

    // :670
    shoved.chain_mut().simplify2(true);

    // :672
    let mut effects: Vec<ShoveEffect> = obstacle_line
      .links()
      .iter()
      .map(|link| ShoveEffect::Unwind(*link))
      .collect();

    // :674 and :676
    effects.push(ShoveEffect::ReplaceLine(Box::new(LineReplacement {
      old: obstacle_line,
      new: shoved,
      rank,
      include_in_changed_area: true,
      allow_redundant: false,
      push: PushMode::Top,
    })));

    (ShoveStatus::Ok, effects)
  }

  /// Resolve a collision with a line that has already been assembled.
  ///
  /// Port of `onCollidingLine`, `pcbnew/router/pns_shove.cpp:738`, the
  /// variant the reverse collision paths use, where the roles are the
  /// other way round: `current` is the pusher and `obstacle` is the line
  /// that yields, which in that branch is the line the iteration started
  /// on.
  fn on_colliding_line(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    current: &Line,
    obstacle: &Line,
    next_rank: i32,
  ) -> (ShoveStatus, Vec<ShoveEffect>) {
    let Some(shoved) =
      self.shove_obstacle_line(world, context, node, current, obstacle)
    else {
      return (ShoveStatus::Incomplete, Vec::new());
    };

    // :759, :761 and :764.
    (
      ShoveStatus::Ok,
      vec![ShoveEffect::ReplaceLine(Box::new(LineReplacement {
        old: obstacle.clone(),
        new: shoved,
        rank: next_rank,
        include_in_changed_area: true,
        allow_redundant: false,
        push: PushMode::Top,
      }))],
    )
  }

  /// Resolve a collision with something that cannot be shoved by walking
  /// the current line around it.
  ///
  /// Port of `onCollidingSolid`,
  /// `pcbnew/router/pns_shove.cpp:776`, the escape hatch of the whole
  /// algorithm: a pad, a board edge or a via the settings forbid moving
  /// never yields, so the **current** line is re-routed instead.
  ///
  /// The walk runs over a [`World::assemble_cluster`] around the
  /// obstacle, not over the obstacle alone, so a pad in a row of touching
  /// pads is cleared in one go rather than one pad per iteration (note 04
  /// section 2.5).
  ///
  /// Two attempts that differ only in the rank the result would carry:
  /// the first promotes it by [`WALKAROUND_RANK_PROMOTION`] unless
  /// [`crate::settings::RoutingSettings::jump_over_obstacles`] is on, the
  /// second demotes it the ordinary way. KiCad re-routes on both, which
  /// is wasted work on the second attempt because nothing between them
  /// changes; it is reproduced because the acceptance test below can
  /// answer differently in principle and the cost is one walk.
  ///
  /// # Where the answer comes from
  ///
  /// The walked line is accepted when it does not collide with the line
  /// at the **bottom** of the stack, or when it does and pushing that
  /// line out of the way is feasible (`:863`). KiCad reads
  /// `m_lineStack.front()` and calls it "the last line"; an empty stack
  /// leaves `success` false, so the handler answers
  /// [`ShoveStatus::Incomplete`] (`:885`). Both are reproduced.
  ///
  /// `world` is `&mut` because the walkaround needs the hull cache, not
  /// because this touches an item; see the module documentation.
  fn on_colliding_solid(
    &self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    current: &Line,
    obstacle: ItemId,
    max_fanout_width: i32,
  ) -> (ShoveStatus, Vec<ShoveEffect>) {
    // :780. A head that ends with a via meets the solid with the via, not
    // with the track, so the roles swap and the solid becomes the pusher.
    if current.ends_with_via() {
      let Some(via) = self.via_at_line_end(world, node, current) else {
        return (ShoveStatus::Incomplete, Vec::new());
      };

      let collides = match (world.item(via), world.item(obstacle)) {
        (Some(via_item), Some(obstacle_item)) => crate::collide::collide_items(
          world.items(),
          ItemRef::stored(obstacle, obstacle_item),
          ItemRef::stored(via, via_item),
          context.resolver,
          &CollisionSearchOptions::default(),
        )
        .is_some(),
        _ => false,
      };

      // :799
      if collides {
        let next_rank =
          world.item(obstacle).map_or(-1, crate::item::Item::rank) - 1;

        return self.on_colliding_via(
          world,
          context,
          node,
          &ViaPusher::Solid(obstacle),
          via,
          max_fanout_width,
          next_rank,
        );
      }
    }

    // :804
    let cluster = world.assemble_cluster(
      node,
      obstacle,
      current.layers().start(),
      Some(CLUSTER_AREA_EXPANSION_LIMIT),
      None,
      context.resolver,
    );

    // :811 to :818.
    let mut walkaround = Walkaround::new(node, context.settings);

    walkaround.set_solids_only(false);
    walkaround.restrict_to_cluster(world, true, &cluster);
    walkaround.set_allowed_policies(&[WalkPolicy::Shortest]);
    walkaround.set_iteration_limit(context.settings.walkaround_iteration_limit);

    let current_rank = current.rank(world);
    let mut accepted: Option<(Line, i32)> = None;

    // :827
    for attempt in 0..SOLID_WALKAROUND_ATTEMPTS {
      // :829
      let next_rank = if attempt == 1 || context.settings.jump_over_obstacles {
        current_rank - 1
      } else {
        current_rank + WALKAROUND_RANK_PROMOTION
      };

      // :834
      let result = walkaround.route(world, context, current);

      if result.status(WalkPolicy::Shortest) != WalkaroundStatus::Done {
        continue;
      }

      let mut walked = result.into_line(WalkPolicy::Shortest);

      // :841 to :843.
      walked.clear_links();
      walked.unmark(world, MarkerFlags::ALL);
      walked.chain_mut().simplify2(true);

      // :845
      if walked.has_loops() {
        continue;
      }

      // :861. An empty stack never sets `success`.
      let Some(last_line) = self.line_stack.first().cloned() else {
        continue;
      };

      // :865, `lastLine.Collide( &walkaroundLine )`: the bottom line is
      // KiCad's `this` and the walked line is its `aHead`, which is the
      // way round that lets the walked line's own via take part.
      let collides = world
        .collide_lines(
          &last_line,
          &walked,
          context.resolver,
          &CollisionSearchOptions::default(),
        )
        .is_some();

      // :869. The probe only asks whether the bottom line could be
      // pushed out of the way; its result is thrown away.
      if !collides
        || self
          .shove_obstacle_line(world, context, node, &walked, &last_line)
          .is_some()
      {
        accepted = Some((walked, next_rank));
        break;
      }
    }

    // :885
    let Some((walked, next_rank)) = accepted else {
      return (ShoveStatus::Incomplete, Vec::new());
    };

    // :888 to :894.
    (
      ShoveStatus::Ok,
      vec![ShoveEffect::ReplaceLine(Box::new(LineReplacement {
        old: current.clone(),
        new: walked,
        rank: next_rank,
        include_in_changed_area: true,
        allow_redundant: false,
        push: PushMode::PopThenTop,
      }))],
    )
  }

  // -----------------------------------------------------------------
  // Vias
  // -----------------------------------------------------------------

  /// The stored via at a line's via end.
  ///
  /// The joint lookup of `onCollidingSolid`
  /// (`pcbnew/router/pns_shove.cpp:784` to `:795`): the line's via may be
  /// one it owns, and what the shove has to move is the real item the
  /// node holds at that point.
  fn via_at_line_end(
    &self,
    world: &World,
    node: NodeId,
    line: &Line,
  ) -> Option<ItemId> {
    let pos = line.via_pos(world)?;
    let reference = world.find_joint(node, pos, line.layer(), line.net())?;

    world.joint(reference)?.via(world.items())
  }

  /// Move a via out of the way, dragging every track attached to it.
  ///
  /// Port of `onCollidingVia`,
  /// `pcbnew/router/pns_shove.cpp:1175`: it computes a minimum
  /// translation vector and hands it to [`Shove::push_or_shove_via`].
  ///
  /// Three sources of a vector, in KiCad's priority order (`:1246`): via
  /// against via beats line against via beats solid against via. KiCad
  /// negates the winner twice, once at `:1247` with a "we may have a sign
  /// issue" comment and once at `:1255`, so the via ends up pushed by
  /// `+mtv`; the two negations cancel and are not reproduced.
  ///
  /// `max_fanout_width` is `OBSTACLE::m_maxFanoutWidth`, which
  /// [`Shove::fixup_via_collisions`] fills in: the obstacle via is
  /// inflated to the width of the widest track hanging off it before the
  /// vector is measured, so that the vector is large enough to clear
  /// those tracks too (`:1194`).
  #[allow(clippy::too_many_arguments)]
  fn on_colliding_via(
    &self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    current: &ViaPusher<'_>,
    obstacle_via: ItemId,
    max_fanout_width: i32,
    next_rank: i32,
  ) -> (ShoveStatus, Vec<ShoveEffect>) {
    let Some(force) = self.via_pushout_vector(
      world,
      context,
      current,
      obstacle_via,
      max_fanout_width,
    ) else {
      return (ShoveStatus::Incomplete, Vec::new());
    };

    // :1255
    self.push_or_shove_via(
      world,
      context,
      node,
      obstacle_via,
      force,
      next_rank,
      false,
    )
  }

  /// The vector [`Shove::on_colliding_via`] pushes a via by.
  ///
  /// The measuring half of `onCollidingVia`,
  /// `pcbnew/router/pns_shove.cpp:1179` to `:1253`, as its own function
  /// so that the mover can stay readable. `None` names a stale handle,
  /// which KiCad cannot represent; a pair that simply does not collide
  /// comes back as a zero vector, which
  /// [`Shove::push_or_shove_via`] answers [`ShoveStatus::Ok`] to
  /// (`:1051`).
  fn via_pushout_vector(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    current: &ViaPusher<'_>,
    obstacle_via: ItemId,
    max_fanout_width: i32,
  ) -> Option<Vec2> {
    let stored = world.item(obstacle_via)?;
    let ItemBody::Via(body) = stored.body() else {
      return None;
    };

    match current {
      ViaPusher::Line(line) => {
        let layer = line.layer();
        let layers = stored.layers();

        // :1179
        let probe = line.rule_item(world, PROBE_UID);
        let clearance = self.clearance_between_items(
          world,
          context,
          ItemRef::unstored(&probe),
          ItemRef::stored(obstacle_via, stored),
        );

        // :1194. The scratch copy the fanout width inflates.
        let mut scratch = stored.clone();

        if max_fanout_width > 0
          && max_fanout_width > body.diameter(layers, layer)
          && let ItemBody::Via(scratch_body) = scratch.body_mut()
        {
          scratch_body.set_diameter(
            layers,
            crate::item::Via::ALL_LAYERS,
            max_fanout_width,
          );
        }

        // :1211. `Collide( CIRCLE, LINE_CHAIN, .., &mtvLine )` writes the
        // force on the **via**; the operands are the other way round here
        // because `collide_mtv` displaces its second argument, which is
        // the same cell without the negation (`src/geometry/collision.rs`).
        let chain = Shape::LineChain(line.shape().clone());
        let line_mtv = scratch.shape(layer).and_then(|shape| {
          crate::geometry::collision::collide_mtv(
            &chain,
            &shape,
            clearance + line.width() / 2,
          )
        });

        // :1216. Via against via takes priority, and the largest per
        // layer vector wins.
        let mut via_mtv: Option<Vec2> = None;

        if let Some(via) = line.via_item(world) {
          let via_clearance = self.clearance_between_items(
            world,
            context,
            via,
            ItemRef::unstored(&scratch),
          );

          for layer in via.item().relevant_shape_layers(&scratch) {
            let (Some(mine), Some(theirs)) =
              (via.item().shape(layer), scratch.shape(layer))
            else {
              continue;
            };

            // The vector displaces the second operand, which is the
            // obstacle via; see `collide_mtv`'s sign convention.
            let Some(mtv) = crate::geometry::collision::collide_mtv(
              &mine,
              &theirs,
              via_clearance,
            ) else {
              continue;
            };

            if via_mtv.is_none_or(|best: Vec2| {
              mtv.squared_euclidean_norm() > best.squared_euclidean_norm()
            }) {
              via_mtv = Some(mtv);
            }
          }
        }

        // :1246
        Some(via_mtv.or(line_mtv).unwrap_or(Vec2::new(0, 0)))
      }
      ViaPusher::Solid(solid) => {
        let pusher = world.item(*solid)?;
        let clearance = self.clearance_between_items(
          world,
          context,
          ItemRef::stored(*solid, pusher),
          ItemRef::stored(obstacle_via, stored),
        );

        // :1236, with the layer KiCad's `Shape( -1 )` names.
        let layer = crate::item::Via::ALL_LAYERS;
        let (Some(pusher_shape), Some(via_shape)) =
          (pusher.shape(layer), stored.shape(layer))
        else {
          return Some(Vec2::new(0, 0));
        };

        Some(
          crate::geometry::collision::collide_mtv(
            &pusher_shape,
            &via_shape,
            clearance,
          )
          .unwrap_or(Vec2::new(0, 0)),
        )
      }
    }
  }

  /// Work out how to move a via and what to drag with it.
  ///
  /// Port of `pushOrShoveVia`,
  /// `pcbnew/router/pns_shove.cpp:1041`, up to but not including its
  /// mutations, which are [`ShoveEffect::MoveVia`].
  ///
  /// The three refusals are KiCad's, in KiCad's order: no joint at the
  /// via centre is [`ShoveStatus::Incomplete`] (`:1054`), a locked via or
  /// [`crate::settings::RoutingSettings::shove_vias`] switched off is
  /// [`ShoveStatus::TryWalk`] so the caller walks around it instead
  /// (`:1060`), and a locked **joint** is
  /// [`ShoveStatus::Incomplete`] (`:1063`).
  ///
  /// The fanout is every segment at the via's joint, each assembled into
  /// a whole line and reversed if needed so that the via end is last, then
  /// corner dragged to the via's new position. A locked segment anywhere
  /// in that fanout is [`ShoveStatus::TryWalk`] (`:1091`).
  #[allow(clippy::too_many_arguments)]
  fn push_or_shove_via(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    via: ItemId,
    force: Vec2,
    new_rank: i32,
    dont_unwind_stack: bool,
  ) -> (ShoveStatus, Vec<ShoveEffect>) {
    // :1051
    if force == Vec2::new(0, 0) {
      return (ShoveStatus::Ok, Vec::new());
    }

    let Some(stored) = world.item(via) else {
      return (ShoveStatus::Incomplete, Vec::new());
    };
    let ItemBody::Via(body) = stored.body() else {
      return (ShoveStatus::Incomplete, Vec::new());
    };

    let origin = body.pos();
    let layer = stored.layer();
    let net = stored.net();

    // :1054
    let Some(reference) = world.find_joint(node, origin, layer, net) else {
      trace(context, || {
        "shove: weird, cannot find the centre of via joint".to_string()
      });

      return (ShoveStatus::Incomplete, Vec::new());
    };
    let Some(joint) = world.joint(reference) else {
      return (ShoveStatus::Incomplete, Vec::new());
    };

    // :1060
    if !context.settings.shove_vias || stored.is_locked() {
      return (ShoveStatus::TryWalk, Vec::new());
    }

    // :1063
    if joint.is_locked() {
      return (ShoveStatus::Incomplete, Vec::new());
    }

    // :1067. The via must not land on an existing joint.
    let mut pushed_pos = origin + force;
    let nudge = force.resize(VIA_ANTI_SNAP_NUDGE);
    let mut steps = 0;

    while world.find_joint(node, pushed_pos, layer, net).is_some() {
      if steps >= VIA_ANTI_SNAP_LIMIT {
        return (ShoveStatus::Incomplete, Vec::new());
      }

      pushed_pos += nudge;
      steps += 1;
    }

    // :1077, `Clone( *aVia )` followed by `SetPos`. KiCad's via copy
    // constructor gives the copy a **fresh** hole rather than aliasing
    // the original's (`pcbnew/router/pns_via.h:126` and `:143`), and so
    // must this: a hole is a separate arena item here, and letting the
    // new via keep the old handle would re home the original's hole into
    // whatever branch the copy is added to, so dropping that branch would
    // take the original's drill with it.
    let mut pushed = stored.clone();

    pushed.set_hole(None);

    if let ItemBody::Via(body) = pushed.body_mut() {
      body.set_pos(pushed_pos);
    }

    // :1107
    pushed.set_rank(new_rank);

    // :1081. Joint link order is insertion order, which is deterministic
    // here because every writer of it is (note 04 section 9 item 6).
    let mut dragged: Vec<(Line, Line)> = Vec::new();

    for link in joint.links().to_vec() {
      let Some(item) = world.item(link) else {
        continue;
      };

      if !item.of_kind(Kind::SEGMENT | Kind::ARC) {
        continue;
      }

      let mut before = world.assemble_line(node, link, None, true, false, true);

      // :1091
      if before.has_locked_segments(world) {
        return (ShoveStatus::TryWalk, Vec::new());
      }

      // :1096. KiCad asserts the via is at one end of the line; a line
      // that does not touch the via at all is skipped here instead.
      let corner = match before.shape().find(origin, 0) {
        Some(0) => {
          before.reverse();

          before.point_count() - 1
        }
        Some(index) if index == before.point_count() - 1 => index,
        _ => continue,
      };

      // :1099
      let mut after = before.clone();

      after.clear_links();
      after.drag_corner(pushed_pos, corner);
      after.chain_mut().simplify2(true);

      dragged.push((before, after));
    }

    (
      ShoveStatus::Ok,
      vec![ShoveEffect::MoveVia(Box::new(ViaMovement {
        old_via: via,
        new_via: Box::new(pushed),
        dragged,
        rank: new_rank,
        dont_unwind_stack,
      }))],
    )
  }

  /// Yield to a via that this run already shoved.
  ///
  /// Port of `onReverseCollidingVia`,
  /// `pcbnew/router/pns_shove.cpp:1267`. The current line has run into a
  /// via with a **higher** rank, so the earlier decision wins and the
  /// current line is the one that moves.
  ///
  /// It is shoved once per track attached to the via, against a synthetic
  /// pusher made of that track plus the via, so that the results compose
  /// (`:1315`). A via with nothing attached gets a degenerate pusher that
  /// is the via alone (`:1348`).
  ///
  /// A via to via collision escalates back to
  /// [`Shove::on_colliding_via`] instead (`:1301`), because two vias
  /// cannot both stay where they are.
  fn on_reverse_colliding_via(
    &self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    current: &Line,
    obstacle_via: ItemId,
    max_fanout_width: i32,
  ) -> (ShoveStatus, Vec<ShoveEffect>) {
    let Some(stored) = world.item(obstacle_via) else {
      return (ShoveStatus::Incomplete, Vec::new());
    };

    // :1271. KiCad also fetches the obstacle's hull and runs an inside
    // test purely to draw it (`:1282`); nothing reads the answer.
    if current.ends_with_via()
      && let Some(via) = current.via_item(world)
    {
      let clearance = self.clearance_between_items(
        world,
        context,
        via,
        ItemRef::stored(obstacle_via, stored),
      );
      let mut collides = false;

      for layer in via.item().relevant_shape_layers(stored) {
        let (Some(mine), Some(theirs)) =
          (via.item().shape(layer), stored.shape(layer))
        else {
          continue;
        };

        collides |=
          crate::geometry::collision::collides(&mine, &theirs, clearance);
      }

      // :1299
      if collides {
        let next_rank = current.rank(world) - 1;

        return self.on_colliding_via(
          world,
          context,
          node,
          &ViaPusher::Line(current),
          obstacle_via,
          max_fanout_width,
          next_rank,
        );
      }
    }

    let Some(pos) =
      world.item(obstacle_via).and_then(|item| match item.body() {
        ItemBody::Via(body) => Some(body.pos()),
        _ => None,
      })
    else {
      return (ShoveStatus::Incomplete, Vec::new());
    };
    let net = world.item(obstacle_via).and_then(Item::net);
    let layer = world.item(obstacle_via).map_or(0, Item::layer);

    // :1308 and :1312.
    let mut carried = current.clone();

    carried.clear_links();
    carried.remove_via();

    let mut shoved = current.clone();

    shoved.clear_links();

    // :1313
    let mut effects: Vec<ShoveEffect> = current
      .links()
      .iter()
      .map(|link| ShoveEffect::Unwind(*link))
      .collect();

    // :1315
    let links = world
      .find_joint(node, pos, layer, net)
      .and_then(|reference| world.joint(reference))
      .map_or_else(Vec::new, |joint| joint.links().to_vec());
    let mut pushers = 0;

    for link in links {
      let Some(item) = world.item(link) else {
        continue;
      };

      if !item.of_kind(Kind::SEGMENT | Kind::ARC)
        || !item.layers().overlaps(current.layers())
      {
        continue;
      }

      // :1322
      let mut pusher = world.assemble_line(node, link, None, true, false, true);

      pusher.link_via(obstacle_via, pos);

      // :1326
      let Some(result) =
        self.shove_obstacle_line(world, context, node, &pusher, &carried)
      else {
        return (ShoveStatus::Incomplete, Vec::new());
      };

      // :1344
      carried.set_shape(result.shape().clone());
      shoved = result;
      pushers += 1;
    }

    // :1348. A stitching via with nothing attached.
    if pushers == 0 {
      let mut pusher = current.clone();

      pusher.chain_mut().clear();
      pusher.clear_links();
      pusher.remove_via();
      pusher.link_via(obstacle_via, pos);

      let Some(result) =
        self.shove_obstacle_line(world, context, node, &pusher, current)
      else {
        return (ShoveStatus::Incomplete, Vec::new());
      };

      carried.set_shape(result.shape().clone());
      shoved = result;
    }

    shoved.clear_links();

    // :1368
    if let Some(via) = current.via() {
      restore_via(&mut shoved, via.clone());
    }

    // :1377. The rank is captured before the replace, and KiCad writes it
    // back after the push; the effect carries it into the replace, which
    // reaches the same items through `LINE::SetRank`'s propagation.
    let current_rank = current.rank(world);

    // :1378 and :1379.
    effects.extend(
      current
        .links()
        .iter()
        .map(|link| ShoveEffect::Unwind(*link)),
    );
    effects.push(ShoveEffect::ReplaceLine(Box::new(LineReplacement {
      old: current.clone(),
      new: shoved,
      rank: current_rank,
      include_in_changed_area: true,
      allow_redundant: false,
      push: PushMode::Top,
    })));

    (ShoveStatus::Ok, effects)
  }

  /// Rewrite an obstacle so that a via is shoved rather than the track
  /// hanging off it.
  ///
  /// Port of `fixupViaCollisions`,
  /// `pcbnew/router/pns_shove.cpp:1517`, a pre-pass on every iteration
  /// (`:1700`) whose whole job is to keep the force propagation
  /// assumption true: a track end never moves on its own, it is only ever
  /// dragged by a via.
  ///
  /// Two cases. The obstacle is a via, and then the widest track attached
  /// to it is reported so that
  /// [`Shove::on_colliding_via`] can inflate the via before it measures
  /// (`:1527`). Or the obstacle is a segment with a via on one end that
  /// is **narrower** than the segment, and then the obstacle is rewritten
  /// to be that via (`:1564`), because shoving the segment would leave
  /// the via behind.
  ///
  /// Answers the obstacle to act on and its fanout width, where KiCad
  /// writes both back into the `OBSTACLE` it was handed.
  fn fixup_via_collisions(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    current: &Line,
    obstacle: ItemId,
  ) -> (ItemId, i32) {
    let layer = current.layer();
    let Some(item) = world.item(obstacle) else {
      return (obstacle, 0);
    };

    // :1527
    if let ItemBody::Via(body) = item.body() {
      let Some(reference) =
        world.find_joint(node, body.pos(), item.layer(), item.net())
      else {
        return (obstacle, 0);
      };
      let Some(joint) = world.joint(reference) else {
        return (obstacle, 0);
      };
      let widest = joint
        .links()
        .iter()
        .filter_map(|link| match world.item(*link)?.body() {
          ItemBody::Segment(segment) => Some(segment.width()),
          _ => None,
        })
        .max()
        .unwrap_or(0);

      // :1551
      if widest > 0 && widest >= body.diameter(item.layers(), layer) {
        return (obstacle, widest + FANOUT_WIDTH_SLACK);
      }

      return (obstacle, 0);
    }

    // :1563
    let ItemBody::Segment(segment) = item.body() else {
      return (obstacle, 0);
    };

    let seg = segment.seg();
    let width = segment.width();
    let segment_layer = item.layer();
    let ends = [seg.a, seg.b];

    for end in ends {
      // :1570. KiCad reads `ja->Via()` without checking that the joint
      // exists; a missing joint is skipped here.
      let Some(via) = world
        .find_joint(node, end, segment_layer, item.net())
        .and_then(|reference| world.joint(reference))
        .and_then(|joint| joint.via(world.items()))
      else {
        continue;
      };
      let Some(stored) = world.item(via) else {
        continue;
      };
      let ItemBody::Via(body) = stored.body() else {
        continue;
      };

      // :1580. A via wider than the track needs no help.
      if body.diameter(stored.layers(), segment_layer) > width {
        continue;
      }

      // :1583
      let mut test = stored.clone();

      if let ItemBody::Via(body) = test.body_mut() {
        body.set_diameter(stored.layers(), crate::item::Via::ALL_LAYERS, width);
      }

      let probe = current.rule_item(world, PROBE_UID);
      let head = LineHead::new(
        ItemRef::unstored(&probe),
        current,
        current.via_item(world),
      );

      // :1587
      if collide_line_items(
        world.items(),
        ItemRef::unstored(&test),
        &head,
        context.resolver,
        &CollisionSearchOptions::default(),
      )
      .is_some()
      {
        // :1591
        return (via, width + FANOUT_WIDTH_SLACK);
      }
    }

    (obstacle, 0)
  }

  /// Re-attach a via that earlier shoving detached from a track.
  ///
  /// Port of `patchTadpoleVia`,
  /// `pcbnew/router/pns_shove.cpp:1599`, called on both reverse collision
  /// branches (`:1727`, `:1762`). When the current line's last point sits
  /// on a joint that carries a colliding via and the line does not
  /// already end with one, the via is linked in so that the pair behaves
  /// as a unit again.
  ///
  /// KiCad's version always returns false and both call sites ignore the
  /// answer; this returns whether it linked anything, which the caller
  /// also ignores and a test can read.
  ///
  /// KiCad mutates the local copy of the stack top, not the stacked line,
  /// so the patch lives exactly as long as the iteration does. That is
  /// reproduced: `current` here is the iteration's own copy.
  fn patch_tadpole_via(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    current: &mut Line,
  ) -> bool {
    // :1601
    let Some(last) = current.last_point() else {
      return false;
    };

    let Some(via) = world
      .find_joint(node, last, current.layer(), current.net())
      .and_then(|reference| world.joint(reference))
      .and_then(|joint| joint.via(world.items()))
    else {
      return false;
    };

    if current.ends_with_via() {
      return false;
    }

    let Some(stored) = world.item(via) else {
      return false;
    };

    // :1620
    let colliding = world
      .check_colliding(
        node,
        ItemRef::stored(via, stored),
        context.resolver,
        &CollisionSearchOptions {
          limit_count: Some(1),
          ..CollisionSearchOptions::default()
        },
      )
      .is_some();

    if !colliding {
      return false;
    }

    let ItemBody::Via(body) = stored.body() else {
      return false;
    };
    let pos = body.pos();

    // :1624
    current.link_via(via, pos);

    true
  }

  // -----------------------------------------------------------------
  // The main loop
  // -----------------------------------------------------------------

  /// Resolve the next collision, as a list of changes to apply.
  ///
  /// Port of `shoveIteration`,
  /// `pcbnew/router/pns_shove.cpp:1633`. It reads the top of the line
  /// stack, looks for the nearest obstacle in the priority order
  /// [`OBSTACLE_SEARCH_ORDER`] gives, and dispatches on what it found and
  /// on how that obstacle is ranked.
  ///
  /// It mutates no item and neither stack: everything it decides comes
  /// back as [`ShoveEffect`]s for `Shove::shove_main_loop` to apply.
  /// `world` is still `&mut` because the obstacle search fills the
  /// clearance and hull caches behind it.
  ///
  /// # The anti ping pong test
  ///
  /// `:1717`. When the obstacle is not a solid, is ranked, and outranks
  /// the current line, it is something this run has already shoved, so
  /// the earlier decision wins and the **current** line yields instead.
  /// Since every forward shove hands out `rank - 1`, ranks strictly
  /// decrease outward from [`HEAD_RANK`] and a cycle would need an item
  /// to outrank its own pusher.
  ///
  /// # The escalation ladder
  ///
  /// A handler that answers [`ShoveStatus::TryWalk`] means "this cannot
  /// be moved", and every such answer is turned into
  /// `Shove::on_colliding_solid` here (`:1826`, `:1841`, `:1855`), so
  /// a locked track, a locked via and a via the settings forbid moving
  /// all end up with the current line walking around them.
  ///
  /// `TODO(arcs)`: the arc branches at `:1793` and `:1833`.
  fn shove_iteration(
    &self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    iteration: u32,
  ) -> (ShoveStatus, Vec<ShoveEffect>) {
    // :1635, a copy of the stack top.
    let Some(current) = self.line_stack.last().cloned() else {
      return (ShoveStatus::Ok, Vec::new());
    };

    // :1650
    let mut nearest = None;

    for kind in OBSTACLE_SEARCH_ORDER {
      let options = CollisionSearchOptions {
        kind_mask: kind,
        ..CollisionSearchOptions::default()
      };

      // Not built: the `SHP_IGNORE` filter at `:1654`, which nothing in
      // KiCad's tree ever switches on; see [`ShovePolicy::IGNORE`].
      nearest = world.nearest_obstacle(
        node,
        &current,
        context.resolver,
        &options,
        context.settings.corner_mode,
      );

      if nearest.is_some() {
        break;
      }
    }

    // :1691. Nothing in the way: the line retires into the optimizer
    // queue, which is the asymmetry note 04 section 1.3 calls essential.
    let Some(nearest) = nearest else {
      return (ShoveStatus::Ok, vec![ShoveEffect::RetireLine]);
    };

    let Some(found) = nearest.item else {
      return (ShoveStatus::Incomplete, Vec::new());
    };

    // :1698. The pre-pass that may rewrite the obstacle from a segment to
    // the via that holds its end.
    let (obstacle_item, max_fanout_width) =
      self.fixup_via_collisions(world, context, node, &current, found);

    let Some((kind, obstacle_rank)) = world
      .item(obstacle_item)
      .map(|item| (item.kind(), item.rank()))
    else {
      return (ShoveStatus::Incomplete, Vec::new());
    };

    trace(context, || {
      format!(
        "shove: iter {iteration} obstacle {kind:?} rank {obstacle_rank} current rank {}",
        current.rank(world)
      )
    });

    // :1715
    let mut effects = vec![ShoveEffect::Unwind(obstacle_item)];

    // :1717
    let reverse = !kind.of_kind(Kind::SOLID)
      && obstacle_rank >= 0
      && obstacle_rank > current.rank(world);

    if reverse {
      return self.reverse_collision(
        world,
        context,
        node,
        current,
        obstacle_item,
        kind,
        max_fanout_width,
        effects,
      );
    }

    // :1820. A lower ranking obstacle, or a solid.
    let (status, pushed) = if kind.of_kind(Kind::SEGMENT) {
      // :1823
      self.on_colliding_segment(world, context, node, &current, obstacle_item)
    } else if kind.of_kind(Kind::VIA) {
      // :1847
      let next_rank = current.rank(world) - 1;

      self.on_colliding_via(
        world,
        context,
        node,
        &ViaPusher::Line(&current),
        obstacle_item,
        max_fanout_width,
        next_rank,
      )
    } else if kind.of_kind(Kind::SOLID | Kind::HOLE) {
      // :1859
      self.on_colliding_solid(
        world,
        context,
        node,
        &current,
        obstacle_item,
        max_fanout_width,
      )
    } else {
      // :1870, KiCad's `default: break`, which leaves `st` at `SH_NULL`
      // and lets the loop treat the iteration as a failure.
      (ShoveStatus::Incomplete, Vec::new())
    };

    // :1826, :1841 and :1855. Whatever refused to move is walked around.
    if status == ShoveStatus::TryWalk {
      let (status, pushed) = self.on_colliding_solid(
        world,
        context,
        node,
        &current,
        obstacle_item,
        max_fanout_width,
      );

      effects.extend(pushed);

      return (status, effects);
    }

    effects.extend(pushed);

    (status, effects)
  }

  /// The branch `Shove::shove_iteration` takes when the obstacle
  /// outranks the current line.
  ///
  /// `pcbnew/router/pns_shove.cpp:1721` to `:1815`, split out so that the
  /// iteration stays readable. The rule is "the earlier decision wins",
  /// so the current line is the one that yields.
  #[allow(clippy::too_many_arguments)]
  fn reverse_collision(
    &self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    current: Line,
    obstacle_item: ItemId,
    kind: Kind,
    max_fanout_width: i32,
    mut effects: Vec<ShoveEffect>,
  ) -> (ShoveStatus, Vec<ShoveEffect>) {
    let mut current = current;

    if kind.of_kind(Kind::VIA) {
      // :1727
      self.patch_tadpole_via(world, context, node, &mut current);

      // :1730. Two vias cannot both stay put, so a real via to via
      // collision is a forward shove of the obstacle after all.
      let via_to_via = current.via_item(world).is_some_and(|via| {
        world.item(obstacle_item).is_some_and(|obstacle| {
          crate::collide::collide_items(
            world.items(),
            ItemRef::stored(obstacle_item, obstacle),
            via,
            context.resolver,
            &CollisionSearchOptions::default(),
          )
          .is_some()
        })
      });

      let (status, pushed) = if via_to_via {
        // :1737
        let next_rank = world.item(obstacle_item).map_or(-1, Item::rank) + 1;

        self.on_colliding_via(
          world,
          context,
          node,
          &ViaPusher::Line(&current),
          obstacle_item,
          max_fanout_width,
          next_rank,
        )
      } else {
        // :1743
        self.on_reverse_colliding_via(
          world,
          context,
          node,
          &current,
          obstacle_item,
          max_fanout_width,
        )
      };

      effects.extend(pushed);

      return (status, effects);
    }

    if !kind.of_kind(Kind::SEGMENT) {
      // :1810, KiCad's `assert( false )` for a reverse collision with
      // anything else. `TODO(arcs)`: the arc branch at `:1793`.
      return (ShoveStatus::Incomplete, Vec::new());
    }

    // :1753. The obstacle's line becomes the pusher.
    let reverse_line =
      world.assemble_line(node, obstacle_item, None, true, false, true);

    // :1758, :1759 and :1760.
    effects.push(ShoveEffect::PopLine);
    effects.extend(
      reverse_line
        .links()
        .iter()
        .map(|link| ShoveEffect::Unwind(*link)),
    );
    self.patch_tadpole_via(world, context, node, &mut current);

    // :1762. A head whose via is what actually touches the segment shoves
    // the via that segment hangs off, not the line.
    let via_hits_segment = current.via_item(world).is_some_and(|via| {
      world.item(obstacle_item).is_some_and(|obstacle| {
        crate::collide::collide_items(
          world.items(),
          ItemRef::stored(obstacle_item, obstacle),
          via,
          context.resolver,
          &CollisionSearchOptions::default(),
        )
        .is_some()
      })
    });

    let (status, pushed) = if via_hits_segment {
      // :1770. KiCad looks the via up again by handle, and answers
      // `SH_INCOMPLETE` when the node no longer has it.
      match self.via_at_line_end(world, node, &current) {
        Some(via) => {
          let next_rank = reverse_line.rank(world) + 1;

          self.on_colliding_via(
            world,
            context,
            node,
            &ViaPusher::Line(&reverse_line),
            via,
            max_fanout_width,
            next_rank,
          )
        }
        None => (ShoveStatus::Incomplete, Vec::new()),
      }
    } else {
      // :1779
      let next_rank = reverse_line.rank(world) + 1;

      self.on_colliding_line(
        world,
        context,
        node,
        &reverse_line,
        &current,
        next_rank,
      )
    };

    effects.extend(pushed);

    // :1782. KiCad pushes the reverse line whatever the shove answered;
    // a failed iteration discards the whole branch, so the difference
    // is unobservable.
    effects.push(ShoveEffect::PushLine {
      line: Box::new(reverse_line),
      below_top: false,
    });

    (status, effects)
  }

  /// Apply what one iteration decided.
  ///
  /// The other half of note 04 section 8.3: every node and stack mutation
  /// of a KiCad shove iteration happens here, in the order the handler
  /// listed them, with nothing else holding a line meanwhile. Answers
  /// false when a stack push was refused, which is KiCad's
  /// `SH_INCOMPLETE` from `pushLineStack`.
  fn apply_effects(
    &mut self,
    world: &mut World,
    node: NodeId,
    effects: Vec<ShoveEffect>,
  ) -> bool {
    for effect in effects {
      match effect {
        ShoveEffect::Unwind(item) => self.unwind_line_stack(world, item),
        ShoveEffect::PopLine => self.pop_line_stack(world),
        ShoveEffect::RetireLine => {
          self.line_stack.pop();
        }
        ShoveEffect::PushLine { line, below_top } => {
          if !self.push_line_stack(world, *line, below_top) {
            return false;
          }
        }
        ShoveEffect::ReplaceLine(replacement) => {
          let LineReplacement {
            mut old,
            mut new,
            rank,
            include_in_changed_area,
            allow_redundant,
            push,
          } = *replacement;

          // The rank goes on before the replace so that `NODE::Add`
          // stamps it onto every fresh segment; see
          // [`ShoveEffect::ReplaceLine::rank`].
          new.set_rank(world, rank);
          self.replace_line(
            world,
            node,
            &mut old,
            &mut new,
            include_in_changed_area,
            allow_redundant,
          );

          let stacked = match push {
            PushMode::None => true,
            PushMode::Top => self.push_line_stack(world, new, false),
            PushMode::BelowTop => self.push_line_stack(world, new, true),
            PushMode::PopThenTop => {
              self.pop_line_stack(world);

              self.push_line_stack(world, new, false)
            }
          };

          if !stacked {
            return false;
          }
        }
        ShoveEffect::MoveVia(movement) => {
          if !self.apply_via_movement(world, node, *movement) {
            return false;
          }
        }
      }
    }

    true
  }

  /// Carry out one [`ShoveEffect::MoveVia`].
  ///
  /// The mutating half of `pushOrShoveVia`,
  /// `pcbnew/router/pns_shove.cpp:1115` to `:1166`, in KiCad's order:
  /// unwind the old via, replace it, push a proxy line when nothing hangs
  /// off it, and then per fanout branch unwind, replace, link the new
  /// via, unwind again, rank and stack.
  ///
  /// Answers false for a refused stack push, which is KiCad's
  /// `SH_INCOMPLETE` (`:1125`, `:1156`).
  fn apply_via_movement(
    &mut self,
    world: &mut World,
    node: NodeId,
    movement: ViaMovement,
  ) -> bool {
    let ViaMovement {
      old_via,
      new_via,
      dragged,
      rank,
      dont_unwind_stack,
    } = movement;

    // :1115
    if !dont_unwind_stack {
      self.unwind_line_stack(world, old_via);
    }

    // :1118, `replaceItems`.
    let Some(pushed) = self.replace_via(world, node, old_via, *new_via) else {
      return false;
    };

    // :1120. A stitching via with no fanout would otherwise be forgotten
    // by the main loop, so it goes on the stack as a line of its own.
    if dragged.is_empty() {
      let Some(item) = world.item(pushed) else {
        return false;
      };
      let proxy = Line::from_linked_via(pushed, item);

      return self.push_line_stack(world, proxy, false);
    }

    // :1129
    for (mut before, mut after) in dragged {
      if !dont_unwind_stack {
        self.unwind_line(world, &before);
      }

      // :1160. A branch that collapsed to nothing is simply removed.
      if after.segment_count() == 0 {
        world.remove_line(node, &mut before);

        continue;
      }

      // :1137
      after.clear_links();

      let entry =
        self.replace_line(world, node, &mut before, &mut after, true, true);

      // :1139
      let pos = after.last_point().unwrap_or_default();

      after.link_via(pushed, pos);

      if !dont_unwind_stack {
        self.unwind_line(world, &after);
      }

      // :1145
      after.set_rank(world, rank);
      // :1150, "fixme: it's inelegant".
      self.root_lines.entry_mut(entry).new_line = Some(after.clone());

      // :1156
      if !self.push_line_stack(world, after, false) {
        return false;
      }
    }

    true
  }

  /// Swap one via in the node for another, and keep the root line index
  /// pointing at the right entry.
  ///
  /// Port of `replaceItems`, `pcbnew/router/pns_shove.cpp:52`, for the
  /// only pair of items that ever reaches it. `None` for a stale handle,
  /// which KiCad cannot represent.
  fn replace_via(
    &mut self,
    world: &mut World,
    node: NodeId,
    old: ItemId,
    new_via: Item,
  ) -> Option<ItemId> {
    // :54, `ChangedArea( VIA*, VIA* )` (`pcbnew/router/pns_via.cpp:291`):
    // a via that did not move contributes nothing.
    let moved = world
      .item(old)
      .is_some_and(|stored| via_pos_of(stored) != via_pos_of(&new_via));

    if moved
      && let Some(area) = world
        .item(old)
        .and_then(|stored| stored.bbox(0))
        .zip(new_via.bbox(0))
        .map(|(before, after)| before.merge(after))
    {
      self.affected_area = Some(match self.affected_area {
        Some(previous) => previous.merge(area),
        None => area,
      });
    }

    // :63
    let old_uid = world.item(old)?.uid();
    let entry = match self.root_lines.find_by_uid(old_uid) {
      Some(entry) => entry,
      None => self
        .root_lines
        .alloc(None, ShovePolicy::DEFAULT, &[old_uid]),
    };

    // :76. `World::add_via` drills a hole for a via that carries none,
    // which is what `Shove::push_or_shove_via` leaves it needing.

    let pushed = world.replace(node, old, new_via);
    let new_uid = world.item(pushed)?.uid();

    // :66 and :79.
    self.root_lines.entry_mut(entry).new_via = Some(pushed);
    self.root_lines.bind(entry, &[new_uid]);

    Some(pushed)
  }

  /// Resolve collisions until there are none left or the budget runs out.
  ///
  /// Port of `shoveMainLoop`, `pcbnew/router/pns_shove.cpp:1880`.
  ///
  /// The affected area is reset here, which means **per head**: KiCad
  /// calls this once per head from `Run` (`:2547`), so the area the
  /// springback frame ends up recording is the last head's alone. That is
  /// reproduced. The budget is not reset here; see [`IterationBudget`].
  ///
  /// The budget test runs after the iteration and unconditionally, so a
  /// run that finishes on its last allowed iteration still answers
  /// [`ShoveStatus::Incomplete`] (`:1913`).
  fn shove_main_loop(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    budget: &mut IterationBudget,
  ) -> ShoveStatus {
    // :1884
    self.affected_area = None;

    while !self.line_stack.is_empty() {
      let (status, effects) =
        self.shove_iteration(world, context, node, budget.used());
      let status = if status == ShoveStatus::Ok
        && !self.apply_effects(world, node, effects)
      {
        ShoveStatus::Incomplete
      } else {
        status
      };

      budget.spend();

      // :1913
      if status != ShoveStatus::Ok || budget.is_exhausted() {
        return ShoveStatus::Incomplete;
      }
    }

    ShoveStatus::Ok
  }

  // -----------------------------------------------------------------
  // Run
  // -----------------------------------------------------------------

  /// Route every head that was added, pushing whatever is in the way.
  ///
  /// Port of `Run`, `pcbnew/router/pns_shove.cpp:2397`. In order: drop
  /// the springback frames the new heads no longer need, branch a fresh
  /// node off what is left, and then, per head, clear the ranks, add the
  /// head to the node and run the main loop over it. On success the
  /// result is optimized, the heads are read back and taken out of the
  /// node again, and the node becomes a springback frame. On failure the
  /// branch is thrown away whole and [`Shove::current_node`] steps back
  /// to the node the run started from.
  ///
  /// The ranks are cleared **again** at the top of every head iteration
  /// (`:2440`), so the ranks one head assigned do not carry into the
  /// next; only the geometry in the node does. That is KiCad's, and it is
  /// the one budget related thing that is still per head here (note 04
  /// section 9 item 3, `DESIGN.md` section 6.2).
  ///
  /// The heads are scaffolding: they are added to the node so that the
  /// shove has something to push with, and removed again at `:2566`
  /// before the node is handed back.
  pub fn run(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
  ) -> ShoveStatus {
    // :2401
    self.heads_modified = false;
    self.line_stack.clear();
    self.optimizer_queue.clear();

    // :2413. A via head contributes the via it names, looked up in the
    // node the session currently stands on, as a line that carries
    // nothing but that via.
    let node_before = self.current_node;
    let head_set: Vec<Line> = self
      .heads
      .iter()
      .filter_map(|head| match (&head.orig_head, head.the_via) {
        (Some(line), _) => Some(line.clone()),
        (None, Some(handle)) => {
          let via = world.find_via_by_handle(
            node_before,
            handle.pos,
            handle.layers,
            handle.net,
          )?;

          world.item(via).map(|item| Line::from_linked_via(via, item))
        }
        (None, None) => None,
      })
      .collect();

    // :2429
    let parent = self.reduce_springback(world, context, &head_set);
    // :2430
    let current = world.branch(parent);

    self.current_node = current;
    world.clear_ranks(current, MarkerFlags::CLEARED_BY_CLEAR_RANKS);

    let mut budget =
      IterationBudget::new(context.settings.shove_iteration_limit);
    let mut status = ShoveStatus::Ok;

    // :2438
    for index in 0..self.heads.len() {
      // :2440
      world.clear_ranks(current, MarkerFlags::CLEARED_BY_CLEAR_RANKS);

      // :2442
      status = if self.heads[index].orig_head.is_some() {
        self.add_head_to_node(world, current, index)
      } else {
        self.add_via_head_to_node(world, context, current, index)
      };

      if status != ShoveStatus::Ok {
        break;
      }

      // :2547
      status = self.shove_main_loop(world, context, current, &mut budget);

      if status != ShoveStatus::Ok {
        break;
      }
    }

    self.iterations = budget.used();

    trace(context, || {
      format!(
        "shove: {status:?} after {} iterations, {} heads",
        self.iterations,
        self.heads.len()
      )
    });

    if status == ShoveStatus::Ok {
      // :2563
      self.run_optimizer(world, context, current);
      // :2565
      self.reconstruct_heads(world);
      // :2566
      self.remove_heads(world, current);
      // :2569
      self.push_springback(current, self.affected_area);
    } else {
      // :2595. KiCad has to clear the stacks before it deletes the node,
      // because the lines in them hold raw pointers into it. Here they
      // hold generational handles that would simply stop resolving, and
      // they are cleared anyway: a stale line is still nonsense.
      // :2578. A via head goes back to where the run found it, and the
      // caller is told the handle changed so that it re-reads it.
      for head in &mut self.heads {
        if let Some(previous) = head.prev_via {
          head.the_via = Some(previous);
          head.geometry_modified = true;
          self.heads_modified = true;
        }
      }

      self.line_stack.clear();
      self.optimizer_queue.clear();
      // :2598
      self.prune_root_lines(world, current);
      // :2600
      world.drop_node(current);
      self.current_node = parent;
    }

    status
  }

  /// Put one head into the branch and onto the line stack.
  ///
  /// The line head half of `Run`'s per head body,
  /// `pcbnew/router/pns_shove.cpp:2462` to `:2545`.
  ///
  /// The endpoint locks at `:2489` are what actually restrain the shove:
  /// a locked joint terminates line assembly
  /// (`World::assemble_line`'s `stop_at_locked_joints`), so no obstacle
  /// line ever grows past a pinned head endpoint and the "endpoints must
  /// not move" checks inside `Shove::shove_line_to_hull_set` then hold
  /// on their own.
  ///
  /// A head that ends with a via has that via **stored** here (`:2505`):
  /// the caller's copy is one the line owns and no node holds, and the
  /// shove needs a real item so that later iterations can collide with
  /// it, rank it and, if something outranks it, move it. Both the
  /// original and the working copy are re-linked to the stored one, and
  /// its uid is indexed under the head's root entry (`:2518`) so that
  /// `Shove::remove_heads` takes it back out again.
  fn add_head_to_node(
    &mut self,
    world: &mut World,
    node: NodeId,
    index: usize,
  ) -> ShoveStatus {
    {
      let Some(entry) = self.heads[index].orig_head.as_mut() else {
        return ShoveStatus::Incomplete;
      };

      debug_assert!(
        !entry.is_linked(),
        "a head arrives with its links cleared (pns_shove.cpp:2464)"
      );

      // :2465
      world.add_line(node, entry, true);
    }

    let policy = self.heads[index].policy;
    let mut head = match self.heads[index].orig_head.clone() {
      Some(head) => head,
      None => return ShoveStatus::Incomplete,
    };

    // :2481
    if head.segment_count() == 0 && !head.ends_with_via() {
      return ShoveStatus::Incomplete;
    }

    // :2489. KiCad passes the head itself, which `LockJoint` reads only
    // for its layers and its net; the first link carries both.
    if !policy.contains(ShovePolicy::DONT_LOCK_ENDPOINTS)
      && let Some(anchor) = head.links().first().copied()
    {
      if head.point_count() > 0 {
        world.lock_joint(node, head.point(0), anchor, true);
      }

      if !head.ends_with_via()
        && let Some(last) = head.last_point()
      {
        world.lock_joint(node, last, anchor, true);
      }
    }

    // :2501
    head.set_rank(world, HEAD_RANK);

    // :2503. The head's own via becomes an item of the node, ranked with
    // the head and linked from both copies.
    let mut head_via = None;

    if let Some(LineVia::Owned(via)) = head.via().cloned() {
      let mut stored = via;
      let uid = world.next_uid();

      stored.set_uid(uid);
      stored.set_rank(HEAD_RANK);

      let ItemBody::Via(body) = stored.body() else {
        return ShoveStatus::Incomplete;
      };
      let pos = body.pos();
      let id = world.add_via(node, stored);

      head.remove_via();
      head.link_via(id, pos);

      if let Some(orig) = self.heads[index].orig_head.as_mut() {
        orig.remove_via();
        orig.link_via(id, pos);
      }

      head_via = Some(id);
    }

    let orig_head = match self.heads[index].orig_head.clone() {
      Some(head) => head,
      None => return ShoveStatus::Incomplete,
    };

    // :2512
    let root = self.touch_root_line(world, &orig_head);
    let entry = self.root_lines.entry_mut(root);

    entry.is_head = true;
    entry.root_line = Some(root_shape_of(&orig_head));
    entry.policy = policy;
    self.heads[index].root_entry = Some(root);

    // :2518
    if let Some(via) = head_via
      && let Some(uid) = world.item(via).map(Item::uid)
    {
      self.root_lines.bind(root, &[uid]);
    }

    // :2540
    if self.push_line_stack(world, head, false) {
      ShoveStatus::Ok
    } else {
      ShoveStatus::Incomplete
    }
  }

  /// Put one via drag head into the branch and shove it to its new
  /// position.
  ///
  /// The via head half of `Run`'s per head body,
  /// `pcbnew/router/pns_shove.cpp:2445` to `:2461`. Unlike a line head,
  /// nothing is added: the via already exists in the node and the head is
  /// the request to move it, so the body is a
  /// [`Shove::push_or_shove_via`] at rank `0` with the stacks left alone
  /// (`:2459`).
  ///
  /// Rank `0` is what makes the wave descend the right way: every track
  /// the drag pushes gets `-1` and so on, and the dragged via itself is
  /// below every line head at [`HEAD_RANK`].
  fn add_via_head_to_node(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    index: usize,
  ) -> ShoveStatus {
    let (Some(handle), Some(new_pos)) =
      (self.heads[index].the_via, self.heads[index].via_new_pos)
    else {
      return ShoveStatus::Incomplete;
    };

    // :2447
    let Some(via) =
      world.find_via_by_handle(node, handle.pos, handle.layers, handle.net)
    else {
      return ShoveStatus::Incomplete;
    };

    // :2453
    let root = self.touch_root_line_of_item(world, via);

    self.root_lines.entry_mut(root).old_via = Some(via);
    self.root_lines.entry_mut(root).policy = self.heads[index].policy;
    self.heads[index].dragged_via = Some(via);

    let force = new_pos - handle.pos;

    // :2459
    let (status, effects) =
      self.push_or_shove_via(world, context, node, via, force, 0, true);

    if status != ShoveStatus::Ok {
      return status;
    }

    if self.apply_effects(world, node, effects) {
      ShoveStatus::Ok
    } else {
      ShoveStatus::Incomplete
    }
  }

  /// Read back what became of each head.
  ///
  /// Port of `reconstructHeads`,
  /// `pcbnew/router/pns_shove.cpp:2296`, whose `aShoveFailed` parameter
  /// its body never reads and which its only live caller passes false
  /// (`:2565`), so it is not a parameter here.
  ///
  /// KiCad asserts that every head still has a root entry and then
  /// dereferences it, which a release build turns into a null
  /// dereference; a head whose entry has gone is skipped here.
  fn reconstruct_heads(&mut self, world: &World) {
    for index in 0..self.heads.len() {
      match self.heads[index].orig_head.clone() {
        Some(orig_head) => {
          // :2309, through the entry the head was given rather than
          // through its links; see [`HeadEntry::root_entry`].
          let Some(root) = self.heads[index]
            .root_entry
            .or_else(|| self.find_root_line(world, &orig_head))
          else {
            continue;
          };
          let entry = self.root_lines.entry(root);

          // :2316
          if let Some(new_line) = entry.new_line.clone() {
            let modified = entry
              .root_line
              .as_ref()
              .is_none_or(|root_line| !new_line.compare_geometry(root_line));

            self.heads[index].new_head = Some(new_line);
            self.heads[index].geometry_modified = modified;
          }
        }
        // :2400. A via head reports where its via ended up, which is the
        // one thing a via dragger reads back.
        None => {
          let Some(dragged) = self.heads[index].dragged_via else {
            continue;
          };
          let Some(uid) = world.item(dragged).map(Item::uid) else {
            continue;
          };
          let Some(root) = self.root_lines.find_by_uid(uid) else {
            continue;
          };
          let entry = self.root_lines.entry(root);

          // :2404 and :2417.
          if let Some(new_via) = entry.new_via {
            self.heads[index].the_via = ViaHandle::of(world, new_via);
            self.heads[index].geometry_modified = true;
          } else if let Some(old_via) = entry.old_via {
            self.heads[index].the_via = ViaHandle::of(world, old_via);
          }
        }
      }

      // :2360
      self.heads_modified |= self.heads[index].geometry_modified;
    }
  }

  /// Take the heads back out of the node.
  ///
  /// Port of `removeHeads`, `pcbnew/router/pns_shove.cpp:2279`. A head is
  /// only in the node so that the shove has something to push with; the
  /// caller owns its shape and gets it back through
  /// [`Shove::modified_head`].
  fn remove_heads(&mut self, world: &mut World, node: NodeId) {
    let (added, _removed) = world.get_updated_items(node);

    for item in added {
      let Some(uid) = world.item(item).map(Item::uid) else {
        continue;
      };

      if self
        .root_lines
        .find_by_uid(uid)
        .is_some_and(|root| self.root_lines.entry(root).is_head)
      {
        world.remove(node, item);
      }
    }
  }

  // -----------------------------------------------------------------
  // The post shove optimization
  // -----------------------------------------------------------------

  /// Everything this session has moved.
  ///
  /// Port of `totalAffectedArea`,
  /// `pcbnew/router/pns_shove.cpp:1926`: the surviving springback frame's
  /// area merged with the current run's.
  fn total_affected_area(&self) -> Option<Box2> {
    let frame = self.stack.last().and_then(|frame| frame.affected_area);

    match (frame, self.affected_area) {
      (Some(frame), Some(current)) => Some(frame.merge(current)),
      (Some(frame), None) => Some(frame),
      (None, current) => current,
    }
  }

  /// Tidy up every line the run touched.
  ///
  /// Port of `runOptimizer`, `pcbnew/router/pns_shove.cpp:2022`. The
  /// queue is walked `n_passes` times, reversed before each pass so that
  /// pass 0 runs newest first and pass 1 oldest first (`:2093`), and each
  /// line that the optimizer improved is replaced in the node and written
  /// back into the queue so that its links stay current.
  ///
  /// Heads are skipped because the caller owns a head's shape (`:2109`),
  /// and so is anything whose policy says
  /// [`ShovePolicy::DONT_OPTIMIZE`] (`:2107`).
  ///
  /// # Deviations
  ///
  /// `LIMIT_CORNER_COUNT`, which KiCad adds to every shove optimization
  /// (`:2064`), is inert: its constraint counts the corners and then
  /// answers true on both branches. `src/optimizer.rs` documents that and
  /// does not build the constraint, so the flag is not set here either.
  ///
  /// KiCad's replace at `:2122` passes four arguments to a five parameter
  /// function, so `aAllowRedundantSegments` receives a node pointer,
  /// which is non null and therefore true, and the node parameter stays
  /// null. Note 04 section 5.1 says not to copy that. The arguments are
  /// spelled out here with the behaviour KiCad actually gets: redundant
  /// segments allowed, into the node that was passed in.
  ///
  /// The area is clipped to [`Shove::set_visible_view_area`] when there
  /// is one. KiCad always clips, because its `VisibleViewArea` always
  /// answers; a headless caller has no viewport, and then nothing is
  /// clipped and the optimizer is free over everything the shove touched.
  fn run_optimizer(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
  ) {
    let mut optimizer = Optimizer::new(node);

    // :2030 and :2034.
    let mut area = self.total_affected_area();
    let max_width = self.optimizer_queue.max_width();

    // :2037
    if let Some(inflated) = area {
      let inflated = inflated.inflate_by(i64::from(max_width));

      area = match self.visible_view_area {
        Some(view) => inflated.intersection(&view),
        None => Some(inflated),
      };
    }

    // :2043
    let (mut flags, passes) = match context.settings.optimizer_effort {
      OptimizerEffort::Low => (EffortFlags::MERGE_OBTUSE, 1),
      // The two are identical in this revision (`:2050` and `:2055`).
      OptimizerEffort::Medium | OptimizerEffort::Full => {
        (EffortFlags::MERGE_SEGMENTS, 2)
      }
    };

    // :2066
    if let Some(area) = area {
      flags |= EffortFlags::RESTRICT_AREA;
      optimizer.set_restrict_area(area, false);
    }

    // :2079. KiCad also allows `ROUNDED_45`, which this crate's
    // `CornerMode` does not have; the pass builds 45 degree connections
    // whatever the mode, which is why the 90 degree modes are excluded.
    if context.settings.smart_pads
      && context.settings.corner_mode == CornerMode::Mitered45
    {
      flags |= EffortFlags::SMART_PADS;
    }

    // :2086
    optimizer.set_effort_level(EffortFlags::from_bits(
      flags.bits() & !self.optimizer_disable_mask.bits(),
    ));
    optimizer.set_collision_mask(Kind::ANY);

    for _pass in 0..passes {
      // :2093
      self.optimizer_queue.reverse();

      for index in 0..self.optimizer_queue.len() {
        let (root, mut line) = {
          let entry = &self.optimizer_queue.entries[index];

          (entry.0, entry.1.clone())
        };
        let (policy, is_head, root_line) = {
          let entry = self.root_lines.entry(root);

          (entry.policy, entry.is_head, entry.root_line.clone())
        };

        // :2107 and :2109.
        if policy.contains(ShovePolicy::DONT_OPTIMIZE) || is_head {
          continue;
        }

        let mut optimized = Line::new();

        // :2118
        if optimizer.optimize(
          world,
          context,
          &line,
          &mut optimized,
          root_line.as_ref(),
        ) {
          // :2122
          self.replace_line(
            world,
            node,
            &mut line,
            &mut optimized,
            false,
            true,
          );
          // :2123
          self.optimizer_queue.entries[index].1 = optimized;
        }
      }
    }
  }
}

// ---------------------------------------------------------------------
// Free helpers
// ---------------------------------------------------------------------

/// Which of an obstacle's endpoints may be moved onto a hull.
///
/// The `aPermitAdjustingStart` and `aPermitAdjustingEnd` pair of
/// `shoveLineToHullSet` (`pcbnew/router/pns_shove.h:215`), as one value
/// so that the function stays inside a readable argument count.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct EndpointAdjustment {
  /// Whether the obstacle's first point may move.
  pub start: bool,
  /// Whether the obstacle's last point may move.
  pub end: bool,
}

impl EndpointAdjustment {
  /// Whether either endpoint may move.
  ///
  /// KiCad's `permitAdjustingEndpoints`,
  /// `pcbnew/router/pns_shove.cpp:334`.
  pub const fn any(self) -> bool {
    self.start || self.end
  }
}

/// The uids of a line's links, in link order.
///
/// The root line index is keyed by uid because KiCad's is
/// (`pcbnew/router/pns_shove.h:282`), and because a uid survives the
/// arena slot being reused where an [`ItemId`] deliberately does not. A
/// link the arena has forgotten contributes nothing.
fn uids_of(world: &World, line: &Line) -> Vec<u64> {
  line
    .links()
    .iter()
    .filter_map(|link| world.item(*link).map(Item::uid))
    .collect()
}

/// A line's shape, kept as the pre shove baseline.
///
/// The `aOld.Clone()` of `allocRootLine`
/// (`pcbnew/router/pns_shove.cpp:137`) with the links dropped. KiCad
/// keeps them and lets them dangle once the node drops the items; nothing
/// reads them, because a root entry is always reached through the index
/// and the optimizer compares only geometry.
fn root_shape_of(line: &Line) -> Line {
  let mut root = line.clone();

  root.clear_links();

  root
}

/// Whether the joint at a point carries a via.
///
/// The `jtStart->Via() != nullptr` of `ShoveObstacleLine`
/// (`pcbnew/router/pns_shove.cpp:537`), which decides whether that
/// endpoint may be moved onto a hull. A point with no joint at all
/// answers false, where KiCad leaves its flag at its initial false for
/// the same reason.
fn joint_has_via(
  world: &World,
  node: NodeId,
  at: Vec2,
  layer: i32,
  net: Option<NetId>,
) -> bool {
  let Some(reference) = world.find_joint(node, at, layer, net) else {
    return false;
  };
  let Some(joint) = world.joint(reference) else {
    return false;
  };

  joint.links().iter().any(|link| {
    world
      .item(*link)
      .is_some_and(|item| item.of_kind(Kind::VIA))
  })
}

/// The point on the nearest hull an endpoint could be snapped to.
///
/// Port of the `minDistP` lambda of `shoveLineToHullSet`,
/// `pcbnew/router/pns_shove.cpp:349`. A point inside a hull counts as
/// distance zero, and only a hull within
/// [`ENDPOINT_ON_HULL_THRESHOLD`] is a candidate at all.
///
/// KiCad returns the point and the distance and lets the caller test the
/// distance again; the threshold is applied once here and the answer is
/// [`None`] when nothing is close enough, which is the same set of
/// outcomes. Its `reject` flag is initialised to false and never set
/// (`:365`), so the branch it guards is not reproduced.
fn nearest_hull_point(
  hulls: &[LineChain],
  invert_traversal: bool,
  reference: Vec2,
) -> Option<Vec2> {
  let mut best: Option<(i64, Vec2)> = None;

  for index in 0..hulls.len() {
    let hull = &hulls[if invert_traversal {
      hulls.len() - 1 - index
    } else {
      index
    }];
    let Some(point) = hull.nearest_point(reference) else {
      continue;
    };
    let distance = if hull.point_inside(reference, 0) {
      0
    } else {
      point.widening_sub(reference).euclidean_norm()
    };

    if distance < ENDPOINT_ON_HULL_THRESHOLD
      && best.is_none_or(|(closest, _)| distance < closest)
    {
      best = Some((distance, point));
    }
  }

  best.map(|(_, point)| point)
}

/// Where a via item sits, or the origin for anything else.
///
/// `VIA::Pos` (`pcbnew/router/pns_via.h:203`) read off an [`Item`], which
/// `Shove::replace_via` needs to reproduce `VIA::ChangedArea`'s "did it
/// actually move" test (`pcbnew/router/pns_via.cpp:293`).
fn via_pos_of(item: &Item) -> Vec2 {
  match item.body() {
    ItemBody::Via(body) => body.pos(),
    _ => Vec2::new(0, 0),
  }
}

/// Put an obstacle's own via back after the hull walk.
///
/// The `aResultLine.AppendVia( *obsVia )` of `ShoveObstacleLine`
/// (`pcbnew/router/pns_shove.cpp:623`).
///
/// # Deviation
///
/// KiCad's `AppendVia` always clones into a via the line **owns**
/// (`pcbnew/router/pns_line.cpp:1421`), because a `LINE`'s via is a
/// pointer whose ownership it recovers by asking. This crate names the
/// two cases (`src/line.rs`), so a via that lives in the node stays
/// linked to it rather than being copied out of it, which is what the
/// shove wants: the via is a real item the node holds and the shove is
/// not trying to make a second one.
///
/// `World::assemble_line` never attaches a via, so the obstacle side only
/// ever carries one when a caller put it there:
/// `Shove::on_reverse_colliding_via` does (`:1368`), and so does the
/// head, whose via is linked by `Shove::add_head_to_node`.
fn restore_via(line: &mut Line, via: LineVia) {
  match via {
    LineVia::Owned(item) => line.append_via(item),
    LineVia::Linked(id) => {
      if let Some(last) = line.last_point() {
        line.link_via(id, last);
      }
    }
  }
}

/// Send one line of prose to the debug decorator, if anything is
/// listening.
///
/// The `PNS_DBG( Dbg(), Message, ... )` macro
/// (`pcbnew/router/pns_debug_decorator.h:127`), whose whole point is that
/// a headless run never formats the string.
fn trace(context: &AlgoContext<'_>, message: impl FnOnce() -> String) {
  if context.debug.is_enabled() {
    context.debug.message(&message());
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::seg::Seg;
  use crate::item::{ItemBody, LayerRange, NetId, Segment};

  /// The width every test track has.
  const WIDTH: i32 = 200000;

  /// A world holding `count` separate segments, and their handles.
  fn world_with_segments(count: i32) -> (World, Vec<ItemId>) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let mut items = Vec::new();

    for index in 0..count {
      let y = i64::from(index) * 1000000;
      let seg = Seg::new(Vec2::new(0, y as i32), Vec2::new(1000000, y as i32));
      let mut item =
        world.make_item(ItemBody::Segment(Segment::new(seg, WIDTH)));

      item.set_layers_and_flash_all(LayerRange::single(0));
      item.set_net(Some(NetId(index as u32 + 1)));
      items.push(
        world
          .add_segment(root, item, false)
          .expect("a test segment is neither degenerate nor redundant"),
      );
    }

    (world, items)
  }

  /// A via of the test width at a point, stored in a node.
  fn add_test_via(world: &mut World, node: NodeId, at: Vec2) -> ItemId {
    let uid = world.next_uid();
    let mut item = Item::new(
      uid,
      ItemBody::Via(crate::item::Via::new(
        at,
        WIDTH * 2,
        WIDTH,
        crate::item::ViaType::Through,
      )),
    );

    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item.set_net(Some(NetId(99)));

    world.add_via(node, item)
  }

  /// A line of one point that links one stored item, which is enough for
  /// the stack rules and never enough to be walked.
  fn line_linking(item: ItemId) -> Line {
    let mut line = Line::new();

    line.set_width(WIDTH);
    line.link(item);

    line
  }

  #[test]
  fn a_budget_is_spent_and_then_exhausted() {
    let mut budget = IterationBudget::new(2);

    assert_eq!(budget.max_iterations(), 2);
    assert!(!budget.is_exhausted());

    budget.spend();
    assert_eq!(budget.used(), 1);
    assert!(!budget.is_exhausted());

    budget.spend();
    assert!(budget.is_exhausted());
  }

  #[test]
  fn a_zero_budget_is_exhausted_from_the_start() {
    // KiCad's test is `m_iter >= iterLimit` after the first iteration, so
    // a limit of zero stops a run after exactly one iteration.
    assert!(IterationBudget::new(0).is_exhausted());
  }

  #[test]
  fn a_policy_is_a_bit_mask() {
    let policy = ShovePolicy::SHOVE | ShovePolicy::DONT_LOCK_ENDPOINTS;

    assert!(policy.contains(ShovePolicy::SHOVE));
    assert!(policy.contains(ShovePolicy::DONT_LOCK_ENDPOINTS));
    assert!(!policy.contains(ShovePolicy::REVERSED));
    assert!(ShovePolicy::DEFAULT.contains(ShovePolicy::DEFAULT));
    assert_eq!(ShovePolicy::from_bits(policy.bits()), policy);
  }

  #[test]
  fn push_below_top_keeps_the_current_line_on_top() {
    let (_world, items) = world_with_segments(3);
    let mut stack = LineStack::new();

    // On an empty stack it is a plain push (`pns_shove.cpp:1465`).
    stack.push_below_top(line_linking(items[0]));
    assert_eq!(stack.len(), 1);

    stack.push(line_linking(items[1]));
    stack.push_below_top(line_linking(items[2]));

    assert_eq!(stack.len(), 3);
    assert!(
      stack
        .last()
        .is_some_and(|line| line.contains_link(items[1]))
    );
    assert!(
      stack
        .first()
        .is_some_and(|line| line.contains_link(items[0]))
    );
  }

  #[test]
  fn unwinding_drops_exactly_the_lines_that_reference_the_item() {
    let (world, items) = world_with_segments(2);
    let mut stack = LineStack::new();

    stack.push(line_linking(items[0]));
    stack.push(line_linking(items[1]));

    assert_eq!(
      unwind_action(&line_linking(items[0]), items[0], false),
      UnwindAction::Drop
    );
    assert_eq!(
      unwind_action(&line_linking(items[0]), items[1], false),
      UnwindAction::Keep
    );

    stack.unwind(&world, items[0]);

    assert_eq!(stack.len(), 1);
    assert!(
      stack
        .last()
        .is_some_and(|line| line.contains_link(items[1]))
    );
  }

  #[test]
  fn unwinding_a_segment_degrades_a_tadpole_to_its_via() {
    let (mut world, items) = world_with_segments(1);
    let root = world.root();
    let via = add_test_via(&mut world, root, Vec2::new(1000000, 0));
    let mut tadpole = line_linking(items[0]);

    tadpole.set_shape(LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(1000000, 0)],
      false,
    ));
    tadpole.link_via(via, Vec2::new(1000000, 0));

    // `pns_shove.cpp:1397`: a line that ends with a via keeps the via
    // when one of its segments is unwound, so cross layer collisions
    // still see it.
    assert_eq!(
      unwind_action(&tadpole, items[0], false),
      UnwindAction::DegradeToViaStub
    );
    // Unwinding the via itself has nothing left to degrade to.
    assert_eq!(unwind_action(&tadpole, via, true), UnwindAction::Drop);

    let mut stack = LineStack::new();

    stack.push(tadpole);
    stack.unwind(&world, items[0]);

    let stub = stack.last().expect("the tadpole was kept as a stub");

    assert_eq!(stub.point_count(), 0);
    assert!(stub.ends_with_via());
    assert!(stub.contains_link(via));
    assert!(!stub.contains_link(items[0]));

    stack.unwind(&world, via);
    assert!(stack.is_empty());
  }

  #[test]
  fn a_via_handle_survives_the_item_being_replaced() {
    let (mut world, _items) = world_with_segments(1);
    let root = world.root();
    let at = Vec2::new(500000, 500000);
    let via = add_test_via(&mut world, root, at);
    let handle = ViaHandle::of(&world, via).expect("the via was just added");

    assert_eq!(handle.pos, at);
    assert_eq!(
      world.find_via_by_handle(root, handle.pos, handle.layers, handle.net),
      Some(via)
    );

    // A handle for a via that is not there answers nothing.
    assert_eq!(
      world.find_via_by_handle(
        root,
        Vec2::new(0, 900000),
        handle.layers,
        handle.net
      ),
      None
    );
  }

  #[test]
  fn the_optimizer_queue_replaces_an_entry_and_moves_it_to_the_end() {
    let (_world, items) = world_with_segments(2);
    let mut queue = OptimizerQueue::new();
    let first = RootLineId(0);
    let second = RootLineId(1);

    queue.insert(first, line_linking(items[0]));
    queue.insert(second, line_linking(items[1]));

    let mut replacement = line_linking(items[0]);

    replacement.set_width(WIDTH * 2);
    queue.insert(first, replacement);

    // KiCad erases and appends rather than updating in place, and the
    // order decides what the optimizer sees first.
    assert_eq!(queue.len(), 2);
    assert_eq!(queue.entries[0].0, second);
    assert_eq!(queue.entries[1].0, first);
    assert_eq!(queue.max_width(), WIDTH * 2);

    queue.reverse();
    assert_eq!(queue.entries[0].0, first);

    queue.prune(first);
    assert_eq!(queue.len(), 1);
    assert_eq!(queue.entries[0].0, second);
  }

  #[test]
  fn the_optimizer_queue_keeps_everything_when_a_via_is_unwound() {
    let (_world, items) = world_with_segments(1);
    let mut queue = OptimizerQueue::new();

    queue.insert(RootLineId(0), line_linking(items[0]));

    // `pns_shove.cpp:1430` erases for a non via link and leaves the queue
    // alone for a via one.
    queue.retain_not_referencing(items[0], true);
    assert_eq!(queue.len(), 1);

    queue.retain_not_referencing(items[0], false);
    assert_eq!(queue.len(), 0);
  }

  #[test]
  fn the_root_line_index_aliases_several_uids_onto_one_entry() {
    let mut index = RootLineIndex::default();
    let entry = index.alloc(None, ShovePolicy::SHOVE, &[1, 2]);

    assert_eq!(index.find_by_uid(1), Some(entry));
    assert_eq!(index.find_by_uid(2), Some(entry));
    assert_eq!(index.find(&[9, 2]), Some(entry));
    assert_eq!(index.find(&[9]), None);
    assert_eq!(index.entry(entry).policy, ShovePolicy::SHOVE);

    // What `replaceLine` does with the new links (`:157`).
    index.bind(entry, &[3]);
    assert_eq!(index.find_by_uid(3), Some(entry));

    // What `pruneRootLines` does, which touches the index only (`:914`).
    index.forget(1);
    assert_eq!(index.find_by_uid(1), None);
    assert_eq!(index.find_by_uid(2), Some(entry));

    index.entry_mut(entry).is_head = true;
    assert!(index.entry(entry).is_head);
  }

  #[test]
  fn a_fresh_shove_stands_on_its_root() {
    let (world, _items) = world_with_segments(1);
    let shove = Shove::new(world.root());

    assert_eq!(shove.root(), world.root());
    assert_eq!(shove.current_node(), world.root());
    assert!(shove.springback().is_empty());
    assert_eq!(shove.iterations(), 0);
    assert!(!shove.heads_modified(None));
    assert!(!shove.heads_modified(Some(0)));
    assert!(shove.modified_head(0).is_none());
  }

  #[test]
  fn clearing_the_heads_forgets_them() {
    let (world, _items) = world_with_segments(1);
    let mut shove = Shove::new(world.root());
    let mut head = Line::new();

    head.set_width(WIDTH);
    head.set_shape(LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(1000, 0)],
      false,
    ));
    shove.add_head_line(head, ShovePolicy::SHOVE);

    assert!(!shove.heads_modified(Some(0)));

    shove.clear_heads();

    // Nothing to answer about any more.
    assert!(shove.modified_head(0).is_none());
  }

  #[test]
  fn rewinding_an_empty_springback_stack_answers_false() {
    let (mut world, _items) = world_with_segments(1);
    let root = world.root();
    let mut shove = Shove::new(root);

    assert!(!shove.rewind_to_last_locked_node());
    assert!(!shove.rewind_springback_to(&mut world, root));
  }

  #[test]
  fn an_endpoint_adjustment_knows_whether_anything_may_move() {
    assert!(!EndpointAdjustment::default().any());
    assert!(
      EndpointAdjustment {
        start: true,
        end: false
      }
      .any()
    );
  }
}
