// SPDX-License-Identifier: GPL-3.0-or-later

//! Dragging an existing segment, corner or via.
//!
//! Port of `PNS::DRAGGER` (`pcbnew/router/pns_dragger.h:47`,
//! `pcbnew/router/pns_dragger.cpp`). Every `// :NNNN` citation in this
//! module is a line of `pcbnew/router/pns_dragger.cpp` unless another
//! file is named. The reference note is
//! `doc/reference/kicad/06-dragger.md`, whose section 10.2 sets the order
//! this is being built in.
//!
//! # What is implemented
//!
//! Steps 1 to 10 of that order, which is every `DRAGGER` routine a board
//! without arcs can reach:
//!
//! - [`Dragger::start`] with the mode decision of `startDragSegment`
//!   (`:118`) and `startDragVia` (`:257`);
//! - [`Dragger::drag`]'s dispatch, its first drag fallback and its
//!   restore branch (`:998`);
//! - `dragMarkObstacles` (`:381`), which is [`Dragger::drag`] in
//!   [`RouterMode::MarkObstacles`], in free angle mode, and after any
//!   first drag failure;
//! - `dragWalkaround` (`:709`) and `tryWalkaround` (`:685`), the
//!   [`RouterMode::Walkaround`] path;
//! - `dragShove` (`:802`), the [`RouterMode::Shove`] path, which drives
//!   the [`Shove`] [`Dragger::start`] built through the head protocol
//!   alone;
//! - the via drag: `findViaFanoutByHandle` (`:267`),
//!   `dragViaMarkObstacles` (`:452`), `propagateViaForces` (`:62`),
//!   `dragViaWalkaround` (`:492`) and `dragShove`'s `DM_VIA` case
//!   (`:908`);
//! - `optimizeAndUpdateDraggedLine` (`:569`), `bestAnchorForPoint`
//!   (`:639`) and `pointHasBadCorner` (`:622`), where the walkaround and
//!   the shove both end;
//! - free angle mode end to end (note 06 section 2.17), which is
//!   [`Dragger::set_free_angle_mode`], `startDragSegment`'s second case
//!   and [`Dragger::drag`]'s bypass, and which never builds a shove;
//! - [`Dragger::traces`], [`Dragger::traces_vias`],
//!   [`Dragger::current_node`], [`Dragger::current_nets`],
//!   [`Dragger::force_mark_obstacles_mode`], [`Dragger::fix_route_node`]
//!   and [`Dragger::fix_route`].
//!
//! # Where a drag ends
//!
//! Note 06 section 2.14 opens with "`optimizeAndUpdateDraggedLine` is
//! where every successful drag ends, in all three modes". That is wrong
//! against the source: its five call sites are `:553` (`dragViaWalkaround`),
//! `:757` and `:786` (`dragWalkaround`) and `:857` and `:900`
//! (`dragShove`). `dragMarkObstacles` (`:381` to `:449`) never optimizes,
//! which the note itself says two sections later and which is what makes
//! a mark obstacles drag follow the cursor exactly.
//!
//! # What is not ported at all
//!
//! - `startDragArc` (`:155`) and the `DM_ARC` cases, because this crate
//!   has no arcs (`PLAN.md`).
//! - `checkVirtualVia` (`:81`). It looks for a via that
//!   `NODE::FixupVirtualVias` planted, and this crate has no virtual vias
//!   (`src/node.rs`, `src/snapshot.rs`), so a click near a width change
//!   becomes a corner drag here where KiCad gives a via drag. Note 06
//!   section 9.5 records the decision that has to be taken before the via
//!   drag is finished.
//! - `COMPONENT_DRAGGER`, which moves footprints rather than traces and
//!   which `PLAN.md` has as the tail of milestone 9. `MULTI_DRAGGER` is
//!   [`crate::multi_dragger`]. There is deliberately no `DRAG_ALGO` trait
//!   over the two: `DESIGN.md` section 11 and note 06 section 9.3 both
//!   ask for enum dispatch in the facade instead, which is
//!   `router::ActiveDragger`.
//! - The dead members note 06 section 8.1 names: `m_origViaConnections`
//!   (`pcbnew/router/pns_dragger.h:166`), never read or written anywhere
//!   in KiCad's tree, and `GetLastDragSolution` (`:108`), which has no
//!   caller. `m_lastDragSolution` itself is kept, because
//!   [`Dragger::drag`]'s restore branch reads it.
//! - `SetDefaultShovePolicy( SHP_SHOVE )` (`:328`), a no operation in
//!   this revision of KiCad (note 06 erratum E3); `src/shove.rs` has no
//!   equivalent.
//!
//! # The node tree
//!
//! ```text
//! world root                       handed in to Dragger::new
//!  +-- pre_drag_node                world.branch(root), once, in start
//!       |   Shove::new stands here when the mode is Shove
//!       +-- last_node                rebuilt on every drag
//! ```
//!
//! `CurrentNode()` is `m_lastNode ? m_lastNode : m_world` (`:1052`), so
//! before the first [`Dragger::drag`] the dragger reports the untouched
//! board.

use crate::algo_base::AlgoContext;
use crate::collide::CollisionSearchOptions;
use crate::geometry::box2::Box2;
use crate::geometry::direction45::{AngleType, Direction45};
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::Vec2;
use crate::item::{Item, ItemBody, ItemId, Kind, MarkerFlags, NetId};
use crate::line::Line;
use crate::mouse_trail::MouseTrailTracer;
use crate::node::{NodeId, World};
use crate::optimizer::{EffortFlags, Optimizer};
use crate::rules::ItemRef;
use crate::settings::RouterMode;
use crate::shove::{Shove, ShovePolicy, ShoveStatus, ViaHandle};
use crate::via::{move_via_by, move_via_to, via_pushout_force};
use crate::walkaround::{WalkPolicy, Walkaround, WalkaroundStatus};

// ---------------------------------------------------------------------
// The mode
// ---------------------------------------------------------------------

/// Which gesture a drag turned out to be.
///
/// Port of `DRAG_MODE` (`pcbnew/router/pns_router.h:75`), reduced to what
/// a [`Dragger`] can actually answer. KiCad's is a bit mask on the way
/// in and a single value on the way out, and note 06 erratum E2 shows the
/// mask half is write only except for `DM_FREE_ANGLE`: `Start` reads that
/// one bit (`:314`) and then `startDragSegment` and `startDragVia`
/// overwrite `m_mode` outright (`:130`, `:144`, `:148`, `:262`). So the
/// caller cannot request a mode, and the free angle bit travels
/// separately as [`Dragger::set_free_angle_mode`].
///
/// `DM_ARC` has no variant, because this crate has no arcs, and
/// `DM_COMPONENT` never reaches a dragger at all
/// (`pcbnew/router/pns_router.cpp:176`).
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub enum DragMode {
  /// One corner of a line moves. `DM_CORNER = 0x01`.
  Corner,
  /// One segment of a line moves sideways. `DM_SEGMENT = 0x02`, and the
  /// value the constructor seeds `m_mode` with (`:44`).
  #[default]
  Segment,
  /// A via moves, taking its fanout with it. `DM_VIA = 0x04`.
  Via,
}

// ---------------------------------------------------------------------
// The snap threshold divisor
// ---------------------------------------------------------------------

/// The divisor `dragMarkObstacles` and `dragWalkaround` apply to the
/// dragged line's width to get their corner snap threshold.
///
/// `width / 4` at `:398` and again at `:727`. `dragShove` uses
/// `width / 2` instead (`:818`), and all three sites carry the same
/// `//TODO: Make threshold configurable` comment, so the difference reads
/// like a slip. Note 06 section 8.5 lists it under "do not fix it",
/// because a KiCad regression golden can see it, which is why
/// [`Dragger::snap_threshold`] takes the divisor rather than hard coding
/// one; see [`SHOVE_SNAP_DIVISOR`].
const MARK_OBSTACLES_SNAP_DIVISOR: i32 = 4;

/// The divisor `dragShove` applies instead.
///
/// `width / 2` at `:818`, under the same
/// `//TODO: Make threshold configurable` comment as the other two sites.
/// See [`MARK_OBSTACLES_SNAP_DIVISOR`] for why the difference is kept.
const SHOVE_SNAP_DIVISOR: i32 = 2;

// ---------------------------------------------------------------------
// The walkaround budget
// ---------------------------------------------------------------------

/// How far a walkaround drag may stray before it gives up.
///
/// `walkaround.SetLengthLimit( true, 30.0 )` (`:692`). The walk is
/// abandoned once the routed length exceeds this many times the direct
/// length (`pcbnew/router/pns_walkaround.cpp:344`), so a single drag is
/// allowed an enormous detour: this is **twenty times** the placer's
/// [`crate::walkaround::DEFAULT_LENGTH_EXPANSION_FACTOR`] and ten times
/// `MULTI_DRAGGER::tryWalkaround`'s 3.0
/// (`pcbnew/router/pns_multi_dragger.cpp:391`). It is the number that
/// lets `walk-with-teardrops` and `walk_drag_seg_against_board_edge`
/// finish their walks at all.
const DRAG_WALKAROUND_LENGTH_LIMIT_FACTOR: f64 = 30.0;

// ---------------------------------------------------------------------
// The dragger
// ---------------------------------------------------------------------

/// A drag in progress.
///
/// Port of `DRAGGER` (`pcbnew/router/pns_dragger.h:47`). One is built per
/// gesture: [`Dragger::start`] decides what the click grabbed,
/// [`Dragger::drag`] answers every mouse move and [`Dragger::fix_route`]
/// commits or refuses.
///
/// Like every other algorithm here it holds [`NodeId`]s rather than owned
/// nodes, and takes the [`World`] and the [`AlgoContext`] per call.
/// KiCad's destructor is empty (`:57`) for the same reason: the branch
/// tree belongs to the world.
pub struct Dragger {
  /// The node the drag branches from, KiCad's `m_world`
  /// (`pcbnew/router/pns_drag_algo.h:128`).
  ///
  /// `ROUTER::StartDragging` hands the dragger the router's **root**
  /// (`pcbnew/router/pns_router.cpp:194`), not a branch, unlike the line
  /// placer.
  world_node: NodeId,
  /// The via as it was when the drag started. Port of `m_initialVia`
  /// (`pcbnew/router/pns_dragger.h:154`).
  ///
  /// `dragViaMarkObstacles` and `dragViaWalkaround` look the fanout up
  /// from this every time (`:438`, `:792`), so those two paths always
  /// re-derive from the original position.
  initial_via: Option<ViaHandle>,
  /// The via where the last successful shove left it. Port of
  /// `m_draggedVia` (`:155`).
  dragged_via: Option<ViaHandle>,
  /// The answer of the last [`Dragger::drag`]. Port of `m_lastNode`
  /// (`:157`).
  last_node: Option<NodeId>,
  /// The branch of [`Dragger::world_node`] made once by
  /// [`Dragger::start`]. Port of `m_preDragNode` (`:158`, `:321`).
  pre_drag_node: Option<NodeId>,
  /// What the click turned out to grab. Port of `m_mode` (`:159`).
  mode: DragMode,
  /// The **original** assembled line, with its links. Port of
  /// `m_draggedLine` (`:160`). Every drag starts from this; it is never
  /// re-dragged.
  dragged_line: Line,
  /// The last successful post shove, post optimize line. Port of
  /// `m_lastDragSolution` (`:161`).
  ///
  /// Written by `Dragger::start_drag_segment` with the original line
  /// (`:123`) and then only by the shove path (`:858`, `:901`), which is
  /// note 06 erratum E4: [`Dragger::drag`]'s restore branch re-adds it
  /// after any failed non first drag, so in walkaround mode a failed drag
  /// snaps the trace back to its original shape rather than to the last
  /// good one.
  last_drag_solution: Line,
  /// The shove engine, allocated only in [`RouterMode::Shove`] and only
  /// outside free angle mode. Port of `m_shove` (`:162`, `:323`).
  shove: Option<Shove>,
  /// A **segment** index in [`DragMode::Segment`] and a **point** index
  /// in [`DragMode::Corner`]. Port of `m_draggedSegmentIndex` (`:163`).
  ///
  /// The two meanings share one field in KiCad too. Segment `i` spans
  /// points `i` and `i + 1`, so the un-incremented index names the
  /// segment's near end and the `++` at `:132` names its far end.
  dragged_segment_index: usize,
  /// Whether the last drag position is legal. Port of `m_dragStatus`
  /// (`:164`).
  drag_status: bool,
  /// [`crate::settings::RoutingSettings::mode`] sampled once by
  /// [`Dragger::start`]. Port of `m_currentMode` (`:165`, `:313`), so
  /// changing the routing mode mid drag has no effect.
  current_mode: RouterMode,
  /// The last point that produced a solution. Port of `m_lastValidPoint`
  /// (`:167`), which KiCad declares as a `VECTOR2D` although all three
  /// of its writes are integer and its only read feeds
  /// `Drag( const VECTOR2I& )` (note 06 erratum E8).
  last_valid_point: Vec2,
  /// What [`Dragger::traces`] answers. Port of `m_draggedItems` (`:170`).
  ///
  /// KiCad's `ITEM_SET` holds a dragged `VIA` as well as the lines
  /// (`:511`), which is [`Dragger::dragged_vias`] here; the mark
  /// obstacles path for a segment or a corner only ever puts one line in
  /// it (`:413`).
  dragged_items: Vec<Line>,
  /// The via half of what [`Dragger::traces`] answers, as
  /// [`Dragger::traces_vias`]. The other half of `m_draggedItems`
  /// (`:170`).
  ///
  /// KiCad's `ITEM_SET` holds lines and items in one vector, so a via
  /// drag's dragged via (`:511`) and its dragged lines (`:557`) share a
  /// set and are cleared together. A [`Line`] is not an
  /// [`crate::item::Item`] here, so the set is two vectors that
  /// [`Dragger::clear_dragged_items`] always empties as one.
  dragged_vias: Vec<ItemId>,
  /// Whether the drag ignores the 45 degree regime. Port of
  /// `m_freeAngleMode` (`:173`), set from the request mask at `:314` and
  /// here by [`Dragger::set_free_angle_mode`].
  free_angle_mode: bool,
  /// Whether the drag has fallen back to highlighting. Port of
  /// `m_forceMarkObstaclesMode` (`:174`).
  ///
  /// Latched true when the very **first** drag fails (`:1029`) and never
  /// cleared, so a drag that starts on top of an obstacle spends the rest
  /// of its life in highlight mode even after the cursor moves somewhere
  /// legal. The host reads it back through
  /// [`Dragger::force_mark_obstacles_mode`] and offers a forced commit.
  force_mark_obstacles_mode: bool,
  /// The trail every drag point is added to. Port of
  /// `m_mouseTrailTracer` (`:175`, fed at `:1000`).
  ///
  /// Read for exactly one thing, the lead vector in `propagateViaForces`
  /// (`:67`), which is step 9. The posture half of the tracer is unused
  /// by the dragger.
  mouse_trail: MouseTrailTracer,
}

impl Dragger {
  // -----------------------------------------------------------------
  // Construction
  // -----------------------------------------------------------------

  /// A dragger over one node.
  ///
  /// The constructor (`:41`) plus `SetWorld`
  /// (`pcbnew/router/pns_drag_algo.h:61`), which
  /// `ROUTER::StartDragging` calls with the router's root
  /// (`pcbnew/router/pns_router.cpp:194`).
  ///
  /// # Panics
  ///
  /// When `node` is not a live node of `world`, which is the same guard
  /// [`crate::placer::line_placer::LinePlacer::new`] applies.
  pub fn new(world: &World, node: NodeId) -> Self {
    assert!(
      world.node(node).is_some(),
      "a dragger needs a live node to branch from"
    );

    Self {
      world_node: node,
      initial_via: None,
      dragged_via: None,
      last_node: None,
      pre_drag_node: None,
      // :44
      mode: DragMode::Segment,
      dragged_line: Line::new(),
      last_drag_solution: Line::new(),
      shove: None,
      dragged_segment_index: 0,
      drag_status: false,
      // :45
      current_mode: RouterMode::MarkObstacles,
      last_valid_point: Vec2::new(0, 0),
      dragged_items: Vec::new(),
      dragged_vias: Vec::new(),
      free_angle_mode: false,
      force_mark_obstacles_mode: false,
      mouse_trail: MouseTrailTracer::new(),
    }
  }

  /// Ask for a free angle drag before starting one.
  ///
  /// `SetMode` (`:255`) narrowed to the one bit `Start` reads out of the
  /// request mask, `DM_FREE_ANGLE` (`:314`); see [`DragMode`] for why the
  /// other bits are not an input. It has to be called before
  /// [`Dragger::start`], as KiCad's `SetMode` is
  /// (`pcbnew/router/pns_router.cpp:193`).
  pub const fn set_free_angle_mode(&mut self, free_angle: bool) {
    self.free_angle_mode = free_angle;
  }

  // -----------------------------------------------------------------
  // Accessors
  // -----------------------------------------------------------------

  /// What the click turned out to grab.
  ///
  /// `Mode()` (`:366`). Note 06 erratum E1 records that KiCad's virtual
  /// has no caller anywhere in its tree; here it is how a host, and the
  /// tests, read which of the three gestures a press resolved to.
  pub const fn mode(&self) -> DragMode {
    self.mode
  }

  /// The node holding everything the drag has changed.
  ///
  /// Port of `CurrentNode` (`:1052`): the last drag's node, or the world
  /// the dragger was given when no drag has happened yet.
  pub fn current_node(&self) -> NodeId {
    self.last_node.unwrap_or(self.world_node)
  }

  /// The lines the drag is moving.
  ///
  /// Port of `Traces` (`:1058`). KiCad returns its `ITEM_SET` by value
  /// and the set holds raw pointers whose lifetime is the node's; a
  /// borrow of owned [`Line`] values has neither hazard.
  pub fn traces(&self) -> &[Line] {
    &self.dragged_items
  }

  /// The vias the drag is moving, as they stand in
  /// [`Dragger::current_node`].
  ///
  /// The other half of `Traces` (`:1058`): KiCad's `ITEM_SET` holds a
  /// dragged `VIA` beside the lines (`:478` to `:484`, `:508` to `:511`),
  /// and only a via drag ever puts one there. `ROUTER::markViolations`
  /// reads the set to skip what the drag is moving
  /// (`pcbnew/router/pns_router.cpp:726`), which is the one place the
  /// distinction is observable.
  ///
  /// At most one entry today, because `findViaFanoutByHandle` keeps at
  /// most one via per joint (`:293`).
  pub fn traces_vias(&self) -> &[ItemId] {
    &self.dragged_vias
  }

  /// The net the drag is on.
  ///
  /// Port of `CurrentNets` (`:372`), which answers the dragged via's net
  /// in [`DragMode::Via`] and the dragged line's otherwise. KiCad returns
  /// a one element vector; the one element is enough.
  pub fn current_nets(&self) -> Option<NetId> {
    if self.mode == DragMode::Via {
      self.dragged_via.and_then(|via| via.net)
    } else {
      self.dragged_line.net()
    }
  }

  /// The layer the drag is on.
  ///
  /// Port of `CurrentLayer` (`pcbnew/router/pns_dragger.h:98`), which
  /// answers `m_draggedLine.Layer()` even in [`DragMode::Via`], where
  /// that line was never assigned. Note 06 section 1.5 shows the accessor
  /// is unreachable in KiCad, so the wrong answer is latent there and is
  /// reproduced here rather than quietly repaired.
  pub const fn current_layer(&self) -> i32 {
    self.dragged_line.layer()
  }

  /// The original line the drag started from.
  ///
  /// Port of `GetOriginalLine` (`pcbnew/router/pns_dragger.h:103`). Its
  /// one caller is `TOOL_BASE::checkSnap`
  /// (`pcbnew/router/pns_tool_base.cpp:308`), which refuses to snap the
  /// cursor to a segment of the line being dragged; a host that snaps
  /// needs the same guard or the drag will snap to itself.
  pub const fn original_line(&self) -> &Line {
    &self.dragged_line
  }

  /// Whether the drag has fallen back to highlighting, and whether the
  /// last position was legal.
  ///
  /// Port of `GetForceMarkObstaclesMode`
  /// (`pcbnew/router/pns_dragger.h:124`), which answers both at once, the
  /// second through an out parameter. The pair is `(fallen back, last
  /// position legal)`.
  pub const fn force_mark_obstacles_mode(&self) -> (bool, bool) {
    (self.force_mark_obstacles_mode, self.drag_status)
  }

  // -----------------------------------------------------------------
  // Start
  // -----------------------------------------------------------------

  /// Begin a drag on an item at a point.
  ///
  /// Port of `Start` (`:304`). KiCad takes an `ITEM_SET` and reads
  /// `aPrimitives[0]` and nothing else (`:309`), so a single item is the
  /// whole of what a `DRAGGER` uses; the set overload exists for
  /// `MULTI_DRAGGER` and `COMPONENT_DRAGGER`, which
  /// `ROUTER::StartDragging` picks from the shape of the set rather than
  /// from a mode (`pcbnew/router/pns_router.cpp:176`).
  ///
  /// False means the drag never started: the item is gone, or it is a
  /// solid, a hole or anything else the `default:` at `:355` refuses. A
  /// `SOLID` refusal is how a single pad handed to a `DRAGGER` bows out.
  ///
  /// # The lock is cleared, not honoured
  ///
  /// `startItem->Unmark( MK_LOCKED )` (`:331`) **mutates the world item**
  /// and is never undone. It is how the dragger overrides a lock the host
  /// already decided to override: `ROUTER_TOOL::performDragging` puts the
  /// confirmation dialog in front of it
  /// (`pcbnew/router/router_tool.cpp:2525`). So a locked segment starts a
  /// drag here and comes out unlocked; refusing it is the host's job, not
  /// this one's.
  pub fn start(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    item: ItemId,
  ) -> bool {
    // :306. An item the arena no longer knows is KiCad's empty set.
    let Some(start_item) = world.item(item) else {
      return false;
    };
    let kind = start_item.kind();

    // :311 to :316
    self.last_node = None;
    self.clear_dragged_items();
    self.current_mode = context.settings.mode;
    self.force_mark_obstacles_mode = false;
    self.last_valid_point = at;

    // :318, :319
    self.mouse_trail.clear();
    self.mouse_trail.add_trail_point(context, at);

    // :321
    let pre_drag = world.branch(self.world_node);

    self.pre_drag_node = Some(pre_drag);

    // :323. The shove stands on the pre drag node itself, not on a
    // branch of it, which is where the dragger differs from the placer.
    self.shove =
      if self.current_mode == RouterMode::Shove && !self.free_angle_mode {
        Some(Shove::new(pre_drag))
      } else {
        None
      };

    // :331
    if let Some(start_item) = world.item_mut(item) {
      start_item.unmark(MarkerFlags::LOCKED);
    }

    // :336. `checkVirtualVia` would sit in the segment arm; see the
    // module documentation for why it does not.
    match kind {
      Kind::SEGMENT => self.start_drag_segment(world, at, item),
      Kind::VIA => self.start_drag_via(world, item),
      // :352 would be `startDragArc`, and :355 refuses everything else.
      _ => false,
    }
  }

  /// Assemble the clicked segment's line and decide between a segment
  /// drag and a corner drag.
  ///
  /// Port of `startDragSegment` (`:118`), the routine that makes the mode
  /// decision. The click is a **corner** drag when it lands within half a
  /// track width of either end of the clicked segment, and a **segment**
  /// drag otherwise; free angle mode forces a corner drag whatever the
  /// click was (`:135`), which is why [`Line::drag_segment`] needs no
  /// free angle form.
  ///
  /// Three spellings of the same threshold live in KiCad's tree and note
  /// 06 erratum E9 keeps them apart: `checkVirtualVia` tests
  /// `dist <= w2` (`:90`, `:94`), this tests `dist < w2` (`:128`), and
  /// `TOOL_BASE::snapToItem` tests squared values with `<`
  /// (`pcbnew/router/pns_tool_base.cpp:496`). A click at exactly `w / 2`
  /// from an endpoint is therefore a segment drag and never a corner
  /// drag.
  ///
  /// The line is assembled from [`Dragger::world_node`], the root
  /// (`:122`), not from the branch [`Dragger::start`] has just made.
  fn start_drag_segment(
    &mut self,
    world: &World,
    at: Vec2,
    segment: ItemId,
  ) -> bool {
    let Some(item) = world.item(segment) else {
      return false;
    };
    let ItemBody::Segment(body) = item.body() else {
      return false;
    };

    // :120
    let half_width = body.width() / 2;
    let seg = body.seg();
    let mut index = 0;

    // :122
    self.dragged_line = world.assemble_line(
      self.world_node,
      segment,
      Some(&mut index),
      false,
      false,
      true,
    );
    // :123
    self.last_drag_solution = self.dragged_line.clone();

    // :125, :126
    let distance_a = (at - seg.a).euclidean_norm();
    let distance_b = (at - seg.b).euclidean_norm();

    if distance_a < half_width || distance_b < half_width {
      // :128
      self.mode = DragMode::Corner;

      // :132. The index becomes a point index, and `<=` puts a click
      // equidistant from both ends on the segment's far end.
      if distance_b <= distance_a {
        index += 1;
      }
    } else if self.free_angle_mode {
      // :135. A mid segment click in free angle mode is a corner drag
      // too, with two guards the branch above does not have. The arc
      // test at `:139` has nothing to match here.
      if distance_b < distance_a && index + 2 < self.dragged_line.point_count()
      {
        index += 1;
      }

      // :144
      self.mode = DragMode::Corner;
    } else {
      // :148
      self.mode = DragMode::Segment;
    }

    self.dragged_segment_index = index;

    true
  }

  /// Remember the via a drag grabbed.
  ///
  /// Port of `startDragVia` (`:257`), which is three assignments and
  /// cannot fail. It stores handles rather than an [`ItemId`] because a
  /// shove **replaces** a via item wholesale, so an id would go stale;
  /// [`World::find_via_by_handle`] resolves the handle again in whatever
  /// node the caller now stands on.
  ///
  /// [`Dragger::initial_via`] is what `dragViaMarkObstacles` (`:452`) and
  /// `dragViaWalkaround` (`:492`) look the fanout up from every time, so
  /// those two always re-derive from the position the drag started at.
  /// [`Dragger::dragged_via`] is the one `dragShove` (`:908`) tracks as
  /// the shove moves it, and the one the shove path's walkaround
  /// fallback uses (`:945`).
  ///
  /// KiCad returns true unconditionally; this returns false for a handle
  /// that cannot be built, which needs the item to have stopped being a
  /// via between the two lookups and cannot happen through
  /// [`Dragger::start`].
  fn start_drag_via(&mut self, world: &World, via: ItemId) -> bool {
    // :259, :260
    let Some(handle) = ViaHandle::of(world, via) else {
      return false;
    };

    self.initial_via = Some(handle);
    self.dragged_via = Some(handle);
    // :262
    self.mode = DragMode::Via;

    true
  }

  // -----------------------------------------------------------------
  // Drag
  // -----------------------------------------------------------------

  /// Move the drag to a point.
  ///
  /// Port of `Drag` (`:998`). The answer is "this position has a valid
  /// solution", not "something happened"
  /// (`pcbnew/router/pns_drag_algo.h:80`).
  ///
  /// Four behaviours of the state machine, all deliberate:
  ///
  /// 1. free angle mode and the forced fallback both bypass the mode
  ///    switch entirely (`:1005`), so a free angle drag never walks
  ///    around and never shoves;
  /// 2. [`Dragger::force_mark_obstacles_mode`] latches on the **first**
  ///    failure (`:1029`) and is never cleared;
  /// 3. `dragMarkObstacles` returns true unconditionally (`:448`), so the
  ///    first drag fallback always succeeds and the restore branch below
  ///    it is reachable only from the walkaround and the shove;
  /// 4. the restore re-adds `Dragger::last_drag_solution`, which only
  ///    the shove path ever updates, and never restores the via (note 06
  ///    erratum E4).
  ///
  pub fn drag(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
  ) -> bool {
    // :1000
    self.mouse_trail.add_trail_point(context, at);

    // :1002
    let first_drag = self.last_node.is_none();

    // :1005
    let mut ok = if self.free_angle_mode || self.force_mark_obstacles_mode {
      self.drag_mark_obstacles(world, context, at)
    } else {
      // :1011
      match self.current_mode {
        RouterMode::MarkObstacles => {
          self.drag_mark_obstacles(world, context, at)
        }
        RouterMode::Shove => self.drag_shove(world, context, at),
        RouterMode::Walkaround => self.drag_walkaround(world, context, at),
      }
    };

    if ok {
      // :1022
      self.last_valid_point = at;
    } else if first_drag {
      // :1029. The first collision resolution failed: fall back to
      // highlighting, and stay there.
      self.force_mark_obstacles_mode = true;
      ok = self.drag_mark_obstacles(world, context, at);

      if ok {
        self.last_valid_point = at;
      }
    } else if let Some(last) = self.last_node {
      self.restore_last_solution(world, last);
    }

    ok
  }

  /// Throw the failed drag away and put the last solution back.
  ///
  /// The `else if( m_lastNode )` arm of `Drag` (`:1036` to `:1045`): the
  /// failed node is replaced by a fresh branch of its own parent and
  /// `Dragger::last_drag_solution` is added to it.
  ///
  /// KiCad drops the failed node with `delete`, which releases the whole
  /// subtree; [`World::drop_node`] is the same operation. The branch is
  /// taken **before** the drop, because the parent handle is read off the
  /// node that is about to go.
  fn restore_last_solution(&mut self, world: &mut World, last: NodeId) {
    // :1039
    let Some(parent) = world.parent(last) else {
      return;
    };
    let restored = world.branch(parent);

    world.drop_node(last);

    self.last_node = Some(restored);
    self.clear_dragged_items();

    // :1043, :1044
    self.last_drag_solution.clear_links();
    world.add_line(restored, &mut self.last_drag_solution, false);
  }

  /// Move the drag and let whatever it collides with be highlighted.
  ///
  /// Port of `dragMarkObstacles` (`:381`), the simplest of the three drag
  /// routines and the one the other two fall back to. It rebuilds the
  /// drag node from scratch, re-drags the **original** line to the new
  /// point, swaps the two in the node and then only **reports** whether
  /// the result collides.
  ///
  /// Always answers true (`:448`), which is what makes the forced mark
  /// obstacles fallback of [`Dragger::drag`] terminal.
  ///
  /// # The threshold is `width / 4` here
  ///
  /// `Settings().SmoothDraggedSegments() ? width / 4 : 0` (`:398`). The
  /// walkaround path uses the same quarter (`:727`) and the shove path
  /// uses a **half** (`:818`), both under an identical
  /// `//TODO: Make threshold configurable` comment. Note 06 section 8.5
  /// keeps them apart on purpose: it looks like a slip but a golden can
  /// see the difference, so it is not normalised away. See
  /// [`Dragger::snap_threshold`].
  ///
  /// # The link ordering
  ///
  /// `dragged.ClearLinks()` runs **after** `origLine` was copied and
  /// **before** the drag (`:402`), so the removal at `:409` uses
  /// `origLine`, which kept its links, and the addition at `:410` builds
  /// fresh ones. Reordering it would make [`World::remove_line`] fall
  /// back to geometry, which is not the same thing.
  fn drag_mark_obstacles(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
  ) -> bool {
    // :384
    if let Some(last) = self.last_node.take() {
      world.drop_node(last);
    }

    // :390. KiCad dereferences `m_preDragNode` unchecked, because
    // `Start` always assigns it (`:321`) and `ROUTER::Move` cannot reach
    // a dragger that never started. A drag before any start answers
    // KiCad's unconditional true with `m_dragStatus` still false, so a
    // fix would refuse.
    let Some(pre_drag) = self.pre_drag_node else {
      return true;
    };
    let last = world.branch(pre_drag);

    self.last_node = Some(last);

    match self.mode {
      // :394
      DragMode::Segment | DragMode::Corner => {
        // :398
        let threshold =
          self.snap_threshold(context, MARK_OBSTACLES_SNAP_DIVISOR);
        // :399, :400
        let mut original = self.dragged_line.clone();
        let mut dragged = self.dragged_line.clone();

        // :401, :402
        dragged.set_snap_threshold(threshold);
        dragged.clear_links();

        if self.mode == DragMode::Segment {
          // :405
          dragged.drag_segment(at, self.dragged_segment_index);
        } else {
          // :407
          dragged.drag_corner(
            at,
            self.dragged_segment_index,
            self.free_angle_mode,
            Direction45::default(),
          );
        }

        // :409, :410
        world.remove_line(last, &mut original);
        world.add_line(last, &mut dragged, false);

        // :412, :413
        self.clear_dragged_items();
        self.dragged_items.push(dragged);
      }
      DragMode::Via => {
        // :437, :438. The answer is discarded, exactly as KiCad discards
        // it: the status below is the collision test and nothing else.
        if let Some(handle) = self.initial_via {
          self.drag_via_mark_obstacles(world, handle, last, at);
        }
      }
    }

    // :443 to :446
    self.drag_status = context.settings.allow_drc_violations()
      || !self.dragged_items_collide(world, context, last);

    // :448
    true
  }

  /// Whether anything the drag is moving collides in a node.
  ///
  /// The `m_lastNode->CheckColliding( m_draggedItems )` of `:446`, which
  /// is the `ITEM_SET` overload (`pcbnew/router/pns_node.cpp:478`): a
  /// plain loop that stops at the first member with an obstacle. A
  /// [`Line`] is not an [`crate::item::Item`] here, so the loop runs over
  /// [`World::check_colliding_line`] instead of
  /// [`World::check_colliding_items`], which is the decomposition
  /// `src/node.rs` names this call site as the reason for.
  fn dragged_items_collide(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
  ) -> bool {
    let options = CollisionSearchOptions {
      limit_count: Some(1),
      ..CollisionSearchOptions::default()
    };

    self
      .dragged_items
      .iter()
      .any(|line| line_collides(world, context, node, line))
      || self.dragged_vias.iter().any(|id| {
        world.item(*id).is_some_and(|item| {
          world
            .check_colliding(
              node,
              ItemRef::stored(*id, item),
              context.resolver,
              &options,
            )
            .is_some()
        })
      })
  }

  /// Move the drag and bend it around what is in the way.
  ///
  /// Port of `dragWalkaround` (`:709`). It drags the **original** line to
  /// the new point exactly as the mark obstacles path does, then, only if
  /// that shape runs into something, hands it to [`try_walkaround`] and
  /// keeps the walked answer. Either way the result goes through
  /// [`Dragger::optimize_and_update_dragged_line`], which is what makes a
  /// walkaround drag come out tidier than a mark obstacles one.
  ///
  /// Unlike `dragMarkObstacles` this can fail, and its failure is what
  /// makes [`Dragger::drag`]'s first drag fallback and restore branch
  /// reachable at all.
  ///
  /// # The threshold is `width / 4` here too
  ///
  /// `:727`, the same quarter as `dragMarkObstacles` and not the half
  /// `dragShove` uses; see [`MARK_OBSTACLES_SNAP_DIVISOR`].
  ///
  /// # Two things not to tidy up
  ///
  /// `DragCorner( aP, idx )` at `:737` omits the free angle argument
  /// where `dragMarkObstacles` passes `m_freeAngleMode` (`:407`). It
  /// cannot matter: free angle mode is routed to `dragMarkObstacles`
  /// before the mode switch is reached (`:1005`), so this routine never
  /// runs with it set. The `false` below is that omission, spelled out.
  ///
  /// The collision probe at `:739` is against `m_world`, the **root**,
  /// not against the branch this routine has just made. At that moment
  /// the branch is a fresh copy of the pre drag node and still holds the
  /// original line, so for a segment drag the two answer the same; note
  /// 06 section 8.5 lists the difference under "do not fix it".
  fn drag_walkaround(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
  ) -> bool {
    let mut ok = false;

    // :714
    if let Some(last) = self.last_node.take() {
      world.drop_node(last);
    }

    // :720. KiCad dereferences `m_preDragNode` unchecked; see
    // `Dragger::drag_mark_obstacles` for why it is always there.
    let Some(pre_drag) = self.pre_drag_node else {
      self.drag_status = false;

      return false;
    };
    let last = world.branch(pre_drag);

    self.last_node = Some(last);

    match self.mode {
      // :724
      DragMode::Segment | DragMode::Corner => {
        // :727
        let threshold =
          self.snap_threshold(context, MARK_OBSTACLES_SNAP_DIVISOR);
        // :728 to :730
        let mut original = self.dragged_line.clone();
        let mut dragged = self.dragged_line.clone();

        dragged.set_snap_threshold(threshold);

        if self.mode == DragMode::Segment {
          // :735
          dragged.drag_segment(at, self.dragged_segment_index);
        } else {
          // :737
          dragged.drag_corner(
            at,
            self.dragged_segment_index,
            false,
            Direction45::default(),
          );
        }

        // :739
        let mut walked =
          if line_collides(world, context, self.world_node, &dragged) {
            match try_walkaround(world, context, last, &dragged) {
              Some(walked) => {
                ok = true;

                walked
              }
              // KiCad leaves `aWalk` assigned to `aOrig` on the way out of
              // `tryWalkaround` (`:695`), which is what the point count
              // test below then sees.
              None => dragged,
            }
          } else {
            // :745, :746
            ok = true;

            dragged
          };

        // :749
        if walked.shape().point_count() < 2 {
          ok = false;
        }

        if ok {
          // :756. `NODE::Remove( LINE& )` clears the line's links on the
          // way out, so `original` reaches the optimizer as geometry
          // alone, which is all `changed_area` reads off it.
          world.remove_line(last, &mut original);
          // :757
          self.optimize_and_update_dragged_line(
            world,
            context,
            &mut walked,
            &original,
            at,
          );
        }
      }
      DragMode::Via => {
        // :791, :792. Note that the fanout is looked up from
        // [`Dragger::initial_via`], the position the drag started at, and
        // not from wherever a previous shove left the via: only
        // `dragShove` tracks the via as it moves.
        ok = self.initial_via.is_some_and(|handle| {
          self.drag_via_walkaround(world, context, handle, last, at)
        });
      }
    }

    // :796
    self.drag_status = ok;

    ok
  }

  /// Move the drag and push what is in the way aside.
  ///
  /// Port of `dragShove` (`:802`), the segment and corner cases. The
  /// dragged line becomes a shove **head** and the engine does the rest;
  /// note 06 section 4 confirms [`Shove`] needs nothing new for it.
  ///
  /// Four things decide the behaviour.
  ///
  /// 1. **The threshold is `width / 2`** (`:818`), not the quarter the
  ///    other two paths use. See [`SHOVE_SNAP_DIVISOR`].
  /// 2. **The removal at `:830` uses the links, not the geometry.**
  ///    `draggedPreShove` has already been re-shaped by
  ///    [`Line::drag_segment`], which does not touch the links, so
  ///    [`World::remove_line`] takes the **original** segments out of the
  ///    shove's node. `MULTI_DRAGGER` spells the same trick out in a
  ///    comment (`pcbnew/router/pns_multi_dragger.cpp:731`); note 06
  ///    section 8.5 lists it under "do not fix it".
  /// 3. **[`ShovePolicy::REVERSED`] for a corner 0 drag** (`:836`). An
  ///    open line has no orientation, and the shove's endpoint locking
  ///    works from the far end, so dragging the line's **first** point
  ///    has to say which end is pushing.
  ///    [`ShovePolicy::DONT_LOCK_ENDPOINTS`] is set for every segment and
  ///    corner drag, because both ends of a dragged line may legitimately
  ///    move.
  /// 4. **The branch is taken whether or not the run succeeded** (`:851`).
  ///    On failure the shove has already rewound its own stack, so
  ///    [`Shove::current_node`] is the pre run state and the branch is a
  ///    clean copy of it.
  ///
  /// The optimizer's root line is [`Dragger::dragged_line`], the original
  /// (`:857`), not the pre shove one.
  ///
  /// # `last_drag_solution` is written only here
  ///
  /// `:858`, and nowhere else outside the arc branch. Note 06 erratum E4:
  /// [`Dragger::drag`]'s restore branch reads it after **any** failed non
  /// first drag, so in walkaround mode it is still the original line from
  /// `startDragSegment` (`:123`) and a failed walkaround drag snaps the
  /// trace back to where it started rather than to the last good
  /// position. That is transcribed rather than repaired: making the
  /// walkaround path write it too would change what a failed drag looks
  /// like, with no fixture asking for the change.
  fn drag_shove(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
  ) -> bool {
    // :805
    if let Some(last) = self.last_node.take() {
      world.drop_node(last);
    }

    match self.mode {
      // :813
      DragMode::Segment | DragMode::Corner => {
        // :818
        let threshold = self.snap_threshold(context, SHOVE_SNAP_DIVISOR);
        // :819, :820
        let mut pre_shove = self.dragged_line.clone();

        pre_shove.set_snap_threshold(threshold);

        if self.mode == DragMode::Segment {
          // :823
          pre_shove.drag_segment(at, self.dragged_segment_index);
        } else {
          // :825
          pre_shove.drag_corner(
            at,
            self.dragged_segment_index,
            false,
            Direction45::default(),
          );
        }

        // :832, :836, :837
        let mut policy = ShovePolicy::SHOVE | ShovePolicy::DONT_LOCK_ENDPOINTS;

        if self.mode == DragMode::Corner && self.dragged_segment_index == 0 {
          policy = policy | ShovePolicy::REVERSED;
        }

        // KiCad dereferences `m_shove` unchecked, and `Start` allocates
        // it for exactly the state that reaches here (`:323`).
        let Some(shove) = self.shove.as_mut() else {
          self.drag_status = false;

          return false;
        };

        // :827, :830
        let pre_shove_node = shove.current_node();

        world.remove_line(pre_shove_node, &mut pre_shove);

        // :839 to :841
        shove.clear_heads();
        shove.add_head_line(pre_shove.clone(), policy);

        let ok = shove.run(world, context) == ShoveStatus::Ok;
        // :843
        let mut post_shove = pre_shove;

        // :847, :848
        if ok
          && shove.heads_modified(None)
          && let Some(head) = shove.modified_head(0)
        {
          post_shove = head.clone();
        }

        // :851
        let shove_node = shove.current_node();
        let last = world.branch(shove_node);

        self.last_node = Some(last);

        if ok {
          // :855, :856. Redundant with `:573` and `:574`, and
          // transcribed anyway.
          post_shove.clear_links();
          post_shove.unmark(world, MarkerFlags::ALL);

          // :857
          let original = self.dragged_line.clone();

          self.optimize_and_update_dragged_line(
            world,
            context,
            &mut post_shove,
            &original,
            at,
          );

          // :858
          self.last_drag_solution = post_shove;
        }

        // :861
        self.drag_status = ok;
      }
      // :908
      DragMode::Via => {
        let (Some(shove), Some(handle)) =
          (self.shove.as_mut(), self.dragged_via)
        else {
          self.drag_status = false;

          return false;
        };

        // :914, the "hack that disables it, before I figure out a more
        // reliable solution" of KiCad's own comment. It is called on
        // every via drag move and never undone.
        shove.disable_post_shove_optimizations(EffortFlags::LIMIT_CORNER_COUNT);

        // :916, :917. A via head takes [`ShovePolicy::SHOVE`] alone,
        // where a line head takes `DONT_LOCK_ENDPOINTS` with it.
        shove.clear_heads();
        shove.add_head_via(handle, at, ShovePolicy::SHOVE);

        // :919
        let status = shove.run(world, context);

        // :924 to :936. The test is on `HeadsModified()` alone and not
        // on the status, which is safe both ways: a failed run puts the
        // head via back where it found it and reports the handle as
        // modified anyway (`pcbnew/router/pns_shove.cpp:2587`), so this
        // re-reads the same position, and a successful one hands back
        // where the via really went. Either way the walkaround fallback
        // below starts from the handle the shove last confirmed, which
        // is why `dragShove` tracks the via and the other two routines
        // do not.
        if shove.heads_modified(None)
          && let Some(moved) = shove.head_via(0)
        {
          self.dragged_via = Some(moved);
        }

        // :939
        let shove_node = shove.current_node();
        let last = world.branch(shove_node);

        self.last_node = Some(last);
        // :941
        self.clear_dragged_items();

        // :944, :945
        self.drag_status = if status == ShoveStatus::Ok {
          true
        } else {
          self.dragged_via.is_some_and(|handle| {
            self.drag_via_walkaround(world, context, handle, last, at)
          })
        };
      }
    }

    // :953
    self.drag_status
  }

  /// Optimize the line a drag produced and put it in the drag node.
  ///
  /// Port of `optimizeAndUpdateDraggedLine` (`:569`), the tail every
  /// walkaround and shove drag ends with. `original` is the line the drag
  /// started from and `at` is the cursor point.
  ///
  /// Four things it does that are easy to get wrong.
  ///
  /// 1. The effort level is [`EffortFlags::MERGE_SEGMENTS`] plus
  ///    [`EffortFlags::MERGE_COLINEAR`] when
  ///    [`crate::settings::RoutingSettings::smooth_dragged_segments`] is
  ///    set (`:580`), plus [`EffortFlags::REQUIRE_OBTUSE_ANGLES`] when
  ///    [`crate::settings::RoutingSettings::restrict_angles`] is (`:583`).
  ///    The second is the whole of what
  ///    [`Optimizer::drag_fix_corners`] and [`crate::optimizer::Constraint::ObtuseOnly`]
  ///    are for, and three of the seven corpus drag cases set it.
  /// 2. [`Optimizer::set_preserve_vertex`] turns
  ///    [`EffortFlags::PRESERVE_VERTEX`] on by itself, in KiCad as here,
  ///    which is why the effort level above never names it.
  /// 3. The preserved vertex is the cursor point when the line already
  ///    passes through it and [`best_anchor_for_point`] otherwise
  ///    (`:590`), and the chain is split there (`:594`) so the vertex
  ///    exists to be preserved.
  /// 4. With
  ///    [`crate::settings::RoutingSettings::optimize_entire_dragged_track`]
  ///    false the optimizer is confined to
  ///    [`Line::changed_area`], and to a **zero size** box at the cursor
  ///    when there is no changed area, whose comment says "No valid area
  ///    yet? set to minimum to disable optimization" (`:603`).
  ///
  /// # Erratum E5 is transcribed, not repaired
  ///
  /// `:617` clears `m_draggedItems` and `:618` puts the optimized line in
  /// alone. Note 06 erratum E5 shows what that costs a via drag:
  /// `dragViaWalkaround` adds the dragged via (`:511`) and every clear
  /// fanout line (`:557`) first, and the first line that needs a
  /// walkaround wipes all of it. The note proposes adding rather than
  /// replacing and clearing at the top of `dragViaWalkaround` instead.
  /// That repair belongs to step 9, where the via path is written and its
  /// only observable consequence, [`Dragger::traces`] under-reporting,
  /// can be tested; until then this is KiCad's line for line.
  fn optimize_and_update_dragged_line(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    dragged: &mut Line,
    original: &Line,
    at: Vec2,
  ) {
    let Some(last) = self.last_node else {
      return;
    };

    // :573, :574. The links are gone before the unmark, so the unmark
    // only ever clears the line's own bits.
    dragged.clear_links();
    dragged.unmark(world, MarkerFlags::ALL);

    // :576
    let mut optimizer = Optimizer::new(last);
    // :578
    let mut effort = EffortFlags::MERGE_SEGMENTS;

    // :580
    if context.settings.smooth_dragged_segments {
      effort |= EffortFlags::MERGE_COLINEAR;
    }

    // :583
    if context.settings.restrict_angles {
      effort |= EffortFlags::REQUIRE_OBTUSE_ANGLES;
    }

    optimizer.set_effort_level(effort);

    // :588, :590
    let anchor = if dragged.shape().find(at, 0).is_some() {
      at
    } else {
      best_anchor_for_point(dragged.shape(), at)
    };

    // :593, :594
    optimizer.set_preserve_vertex(anchor);
    dragged.chain_mut().split(anchor);

    // :598
    if !context.settings.optimize_entire_dragged_track {
      // :600, :603
      let area = dragged
        .changed_area(original)
        .unwrap_or_else(|| Box2::from_vec2(at));

      // :607. KiCad passes `SetRestrictArea`'s `true` default, which
      // does nothing either way; see `Optimizer::set_restrict_area`.
      optimizer.set_restrict_area(area, true);
    }

    // :612
    let mut post_opt = Line::new();

    optimizer.optimize(world, context, dragged, &mut post_opt, Some(original));

    // :613. The caller's line takes the optimized shape without the
    // links the addition below hands out.
    *dragged = post_opt.clone();

    // :616 to :618
    world.add_line(last, &mut post_opt, false);
    self.clear_dragged_items();
    self.dragged_items.push(post_opt);
  }

  // -----------------------------------------------------------------
  // The via drag
  // -----------------------------------------------------------------

  /// Move a via to the cursor and take its fanout with it, reporting
  /// whatever that runs into.
  ///
  /// Port of `dragViaMarkObstacles` (`:452`). No forces and no
  /// walkaround: every line attached at the via has its near corner
  /// dragged to the cursor and the via itself is replaced by a copy at
  /// the cursor.
  ///
  /// KiCad returns true unconditionally once past the empty fanout guard
  /// (`:458`), and its one caller in this mode discards the answer
  /// anyway (`:438`); the collision test that follows the call is where a
  /// via drag's status comes from.
  ///
  /// # `node` is only read from
  ///
  /// The fanout is looked up in `node` and every mutation goes to
  /// [`Dragger::last_node`] (`:473`, `:474`, `:483`, `:484`), which note
  /// 06 erratum E13 records: both call sites pass `m_lastNode` for
  /// `aNode`, so the parameter is redundant. It is kept, because keeping
  /// it is what makes the two nodes visible at the call site.
  fn drag_via_mark_obstacles(
    &mut self,
    world: &mut World,
    handle: ViaHandle,
    node: NodeId,
    at: Vec2,
  ) -> bool {
    // :454
    self.clear_dragged_items();

    // :456
    let fanout = find_via_fanout_by_handle(world, node, handle);

    // :458
    if fanout.is_empty() {
      return true;
    }

    let Some(last) = self.last_node else {
      return true;
    };

    for item in fanout {
      match item {
        ViaFanoutItem::Line(line) => {
          // :462, :463
          let line = *line;
          let mut original = line.clone();
          let mut dragged = line;
          // :468. `CLine().Find( aHandle.pos )` answers `-1` for a line
          // that does not pass through the handle and KiCad feeds that
          // straight to `DragCorner`; `findViaFanoutByHandle` has just
          // reversed every line so that the via is point 0, so only a
          // stale handle could miss and such a line is skipped here.
          let Some(corner) = original.shape().find(handle.pos, 0) else {
            continue;
          };

          dragged.drag_corner(
            at,
            corner,
            self.free_angle_mode,
            Direction45::default(),
          );
          // :469
          dragged.clear_links();

          // :471
          self.dragged_items.push(dragged.clone());

          // :473, :474. The removal uses `original`, which kept its
          // links.
          world.remove_line(last, &mut original);
          world.add_line(last, &mut dragged, false);
        }
        ViaFanoutItem::Via(via) => {
          // :478, :480
          let Some(moved) = cloned_via_at(world, via, at) else {
            continue;
          };

          // :483, :484
          world.remove(last, via);

          let id = world.add_via(last, moved);

          // :481. KiCad adds the raw pointer to the set before the node
          // takes ownership of it; an arena id only exists after the
          // addition, so the push moves down here. Same item either way.
          self.dragged_vias.push(id);
        }
      }
    }

    true
  }

  /// Push a via out of what it collides with, along the drag's own lead.
  ///
  /// Port of `propagateViaForces` (`:62`), which is
  /// [`via_pushout_force`] with two arguments filled in: the direction is
  /// the **negated** trail lead vector (`:67`), so a via the barycentric
  /// force cannot free is pushed back the way the cursor came, and the
  /// budget is
  /// [`crate::settings::RoutingSettings::via_force_prop_iteration_limit`]
  /// (`:69`).
  ///
  /// `via` is a copy that is not in any node yet; on success it is moved
  /// by the force (`:73`).
  ///
  /// KiCad's signature takes a `std::set<VIA*>&` and reads only
  /// `*vias.begin()`, and its one call site builds a set of exactly one
  /// (`:513` to `:515`). Note 06 erratum E10: that is dead generality,
  /// and iterating a set of pointers is address ordered, which
  /// `DESIGN.md` section 8 forbids. It is one via here.
  fn propagate_via_forces(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    via: &mut Item,
  ) -> bool {
    // :67
    let lead = -self.mouse_trail.trail_lead_vector();
    // :69
    let limit = context.settings.via_force_prop_iteration_limit;

    // :71
    let Some(force) =
      via_pushout_force(world, context, node, via, lead, Kind::ANY, limit)
    else {
      // :77
      return false;
    };

    // :73
    move_via_by(via, force);

    true
  }

  /// Move a via to the cursor, push it clear of what is there, and bend
  /// its fanout around whatever the new geometry runs into.
  ///
  /// Port of `dragViaWalkaround` (`:492`), which is two passes over the
  /// fanout: the via first, because every line has to be dragged to where
  /// the via really ended up, and then the lines.
  ///
  /// # Three things transcribed rather than repaired
  ///
  /// - **The via is removed from the drag node before the force
  ///   propagation and only put back if it succeeds** (`:517`, `:525`),
  ///   so the `false` at `:530` leaves a node without the via that is
  ///   being dragged. Note 06 erratum E11. Nothing sees it, because
  ///   [`Dragger::drag`]'s restore branch re-branches from the parent and
  ///   throws this node away.
  /// - **[`Dragger::optimize_and_update_dragged_line`] clears the dragged
  ///   set** (`:617`), so the first fanout line that needs a walkaround
  ///   wipes out the via added at `:511` and every earlier line. Note 06
  ///   erratum E5: `Traces()` then under-reports and
  ///   `ROUTER::markViolations` stops skipping the dragged via, marking
  ///   it as colliding with itself. The note proposes adding rather than
  ///   replacing; that is a behaviour change with no fixture behind it,
  ///   so this reproduces KiCad and `tests/dragger.rs` pins the result.
  /// - **The optimizer is anchored on the cursor, not on where the via
  ///   ended up** (`:553` against `:541`). When the force propagation
  ///   moved the via, the cursor is not on the line at all, so
  ///   [`best_anchor_for_point`] silently substitutes the nearest good
  ///   vertex. Note 06 erratum E6.
  ///
  /// `LINE walkLine( *l )` at `:539` is a dead initialization, because
  /// `tryWalkaround` assigns over it first thing (`:695`); there is no
  /// counterpart here.
  fn drag_via_walkaround(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    handle: ViaHandle,
    node: NodeId,
    at: Vec2,
  ) -> bool {
    // :494
    self.clear_dragged_items();

    // :496
    let fanout = find_via_fanout_by_handle(world, node, handle);

    // :498
    if fanout.is_empty() {
      return true;
    }

    let Some(last) = self.last_node else {
      return false;
    };
    let mut via_prop_ok = false;
    let mut via_target_pos = at;

    // :504
    for item in &fanout {
      let ViaFanoutItem::Via(via) = item else {
        continue;
      };
      // :508, :510
      let Some(mut dragged_via) = cloned_via_at(world, *via, at) else {
        continue;
      };

      // :517
      world.remove(last, *via);

      // :519
      if self.propagate_via_forces(world, context, last, &mut dragged_via) {
        // :523
        via_target_pos = via_position(&dragged_via).unwrap_or(at);
        via_prop_ok = true;

        // :525
        let id = world.add_via(last, dragged_via);

        // :511, see `Dragger::drag_via_mark_obstacles` for why the push
        // is here rather than before the force propagation.
        self.dragged_vias.push(id);
      }
    }

    // :530
    if !via_prop_ok {
      return false;
    }

    // :533
    for item in fanout {
      let ViaFanoutItem::Line(line) = item else {
        continue;
      };
      let line = *line;
      let mut original = line.clone();
      let mut dragged = line;
      let Some(corner) = original.shape().find(handle.pos, 0) else {
        continue;
      };

      // :541. The line goes to where the via really is, not to the
      // cursor.
      dragged.drag_corner(
        via_target_pos,
        corner,
        self.free_angle_mode,
        Direction45::default(),
      );
      // :543
      dragged.clear_links();

      // :545. Against the root, as everywhere else in this file.
      if line_collides(world, context, self.world_node, &dragged) {
        // :547 to :550
        let Some(mut walked) = try_walkaround(world, context, last, &dragged)
        else {
          return false;
        };

        // :552
        world.remove_line(last, &mut original);
        // :553, erratum E6: the anchor is `at`, not `via_target_pos`.
        self.optimize_and_update_dragged_line(
          world,
          context,
          &mut walked,
          &original,
          at,
        );
      } else {
        // :557 to :560
        self.dragged_items.push(dragged.clone());

        world.remove_line(last, &mut original);
        world.add_line(last, &mut dragged, false);
      }
    }

    true
  }

  /// Empty the set [`Dragger::traces`] and [`Dragger::traces_vias`]
  /// answer from.
  ///
  /// Every `m_draggedItems.Clear()` in the file (`:312`, `:412`, `:454`,
  /// `:494`, `:617`, `:941`, `:1042`). The two vectors are one `ITEM_SET`
  /// in KiCad and are never cleared apart.
  fn clear_dragged_items(&mut self) {
    self.dragged_items.clear();
    self.dragged_vias.clear();
  }

  /// How far a dragged corner may snap onto the line's own neighbours.
  ///
  /// The `Settings().SmoothDraggedSegments() ? width / N : 0` the three
  /// drag routines each compute for themselves (`:398`, `:727`, `:818`).
  /// `divisor` is the one thing that differs between them:
  /// [`MARK_OBSTACLES_SNAP_DIVISOR`] for the mark obstacles and
  /// walkaround paths, `2` for the shove.
  fn snap_threshold(&self, context: &AlgoContext<'_>, divisor: i32) -> i32 {
    if context.settings.smooth_dragged_segments {
      self.dragged_line.width() / divisor
    } else {
      0
    }
  }

  // -----------------------------------------------------------------
  // Fix
  // -----------------------------------------------------------------

  /// Commit the drag, or refuse.
  ///
  /// Port of `FixRoute` (`:957`). `ROUTER::FixRoute` drops the point and
  /// the end item on this branch, because the dragger already knows where
  /// it is (`pcbnew/router/pns_router.cpp:928`), so `force_commit` is the
  /// only argument.
  ///
  /// `force_commit` is honoured **only** in forced mark obstacles mode
  /// (`:968` to `:977`), which is note 06 erratum E7: in plain
  /// [`RouterMode::MarkObstacles`] with a colliding drag and
  /// [`crate::settings::RoutingSettings::allow_drc_violations`] false,
  /// `Dragger::drag_status` is false and the fallback never latched, so
  /// a forced commit falls into the re-drag branch instead of committing.
  ///
  /// The re-drag at `:983` is a full [`Dragger::drag`] call, so it goes
  /// through the walkaround or the shove again, can itself fail, and
  /// appends another point to the mouse trail.
  ///
  /// KiCad's `FixRoute` reaches back into `ROUTER::CommitRouting( node )`
  /// through the dragger's router pointer. There is no such pointer here
  /// (`DESIGN.md` section 8), and the facade has to build its host facing
  /// diff **before** the node is folded away, so the routine is split:
  /// [`Dragger::fix_route_node`] is everything up to the commit and
  /// answers which node to commit, and this is the whole of it for a
  /// caller that only wants the world updated.
  pub fn fix_route(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    force_commit: bool,
  ) -> bool {
    match self.fix_route_node(world, context, force_commit) {
      Some(node) => {
        world.commit(node);

        true
      }
      None => false,
    }
  }

  /// Which node a fix would commit, or [`None`] when it refuses.
  ///
  /// The decision half of [`Dragger::fix_route`], which is
  /// `DRAGGER::FixRoute` (`:957`) with the three
  /// `Router()->CommitRouting( node )` calls replaced by the node they
  /// were given. Nothing is committed here, so a caller may still read
  /// the delta off the answer.
  ///
  /// The re-drag of the last branch (`:983`) does happen, because it is
  /// what makes the answer legal; only the commit is deferred.
  pub fn fix_route_node(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    force_commit: bool,
  ) -> Option<NodeId> {
    let node = self.current_node();

    // :963
    if self.drag_status {
      return Some(node);
    }

    // :968
    if self.force_mark_obstacles_mode {
      return force_commit.then_some(node);
    }

    // :983. Everything already committed is legal even when the current
    // cursor solution is not, so the drag is re-run at the last point
    // that worked.
    let at = self.last_valid_point;

    self.drag(world, context, at);

    self.drag_status.then(|| self.current_node())
  }
}

// ---------------------------------------------------------------------
// The walkaround
// ---------------------------------------------------------------------

/// Whether a line runs into anything in a node.
///
/// The `NODE::CheckColliding( const ITEM* )` overload
/// (`pcbnew/router/pns_node.h:342`), which the dragger asks twice: of the
/// root, about a freshly dragged line (`:739`, `:545`), and of the drag
/// node, about everything the drag is moving (`:446`). KiCad's builds its
/// own options with `m_limitCount = 1`
/// (`pcbnew/router/pns_node.cpp:494`), since the answer is a yes or a no.
fn line_collides(
  world: &World,
  context: &AlgoContext<'_>,
  node: NodeId,
  line: &Line,
) -> bool {
  let options = CollisionSearchOptions {
    limit_count: Some(1),
    ..CollisionSearchOptions::default()
  };

  world
    .check_colliding_line(node, line, context.resolver, &options)
    .is_some()
}

/// Walk one line around whatever is in its way, or fail.
///
/// Port of `tryWalkaround` (`:685`), four setters and one call.
/// [`None`] is KiCad's false, where the caller keeps the unwalked line.
///
/// Only [`WalkPolicy::Shortest`] runs (`:693`), so there is no best of
/// three to pick from, and the length limit is
/// [`DRAG_WALKAROUND_LENGTH_LIMIT_FACTOR`] rather than the default
/// (`:692`). `SetSolidsOnly( false )` (`:688`) lets the walk go around
/// tracks as well as pads, and `SetIterationLimit` (`:691`) re-imposes
/// the setting [`Walkaround::new`] has already installed, exactly as the
/// shove's own call does.
fn try_walkaround(
  world: &mut World,
  context: &AlgoContext<'_>,
  node: NodeId,
  original: &Line,
) -> Option<Line> {
  // :687
  let mut walkaround = Walkaround::new(node, context.settings);

  // :688
  walkaround.set_solids_only(false);
  // :691
  walkaround.set_iteration_limit(context.settings.walkaround_iteration_limit);
  // :692
  walkaround.set_length_limit(true, DRAG_WALKAROUND_LENGTH_LIMIT_FACTOR);
  // :693
  walkaround.set_allowed_policies(&[WalkPolicy::Shortest]);

  // :695, :697
  let result = walkaround.route(world, context, original);

  // :699 to :704
  (result.status(WalkPolicy::Shortest) == WalkaroundStatus::Done)
    .then(|| result.into_line(WalkPolicy::Shortest))
}

// ---------------------------------------------------------------------
// The via fanout
// ---------------------------------------------------------------------

/// One thing attached at the joint a via sits on.
///
/// The members of the `ITEM_SET` `findViaFanoutByHandle` answers with
/// (`:267`), which holds assembled `LINE`s by value and at most one
/// `VIA` by pointer. A [`Line`] is not an [`crate::item::Item`] here, so
/// the two cases are an enum rather than one heterogeneous set.
#[derive(Debug)]
enum ViaFanoutItem {
  /// A trivially connected line, reversed so that it **starts** at the
  /// via (`:286`). Boxed because a [`Line`] is forty times the size of
  /// an [`ItemId`] and the fanout of a via is mostly lines.
  Line(Box<Line>),
  /// The via itself, as it stands in the node the fanout was read from.
  Via(ItemId),
}

/// Everything attached at a via's joint.
///
/// Port of `findViaFanoutByHandle` (`:267`). The handle names a position,
/// a layer range and a net rather than an item, because a shove replaces
/// a via wholesale and an id would go stale.
///
/// # Every line starts at the via
///
/// `if( segIndex != 0 ) l.Reverse()` (`:286`) is the invariant both
/// callers depend on: it is what lets them say
/// `origLine.CLine().Find( aHandle.pos )` and get a corner index that
/// [`Line::drag_corner`] can use. The condition is on the **seed
/// segment's** index within the assembled line and not on where the via
/// is, which is the same thing only because the seed is linked to this
/// joint; it is transcribed rather than reasoned about.
///
/// # One via
///
/// The `foundVia` guard (`:291`) means a stacked pair at one joint
/// contributes a single via. KiCad walks the joint's link vector, which
/// is insertion ordered there and here, so which one that is is
/// deterministic on both sides.
fn find_via_fanout_by_handle(
  world: &World,
  node: NodeId,
  handle: ViaHandle,
) -> Vec<ViaFanoutItem> {
  // :271
  let Some(reference) =
    world.find_joint(node, handle.pos, handle.layers.start(), handle.net)
  else {
    // :273
    return Vec::new();
  };
  let Some(joint) = world.joint(reference) else {
    return Vec::new();
  };
  let mut fanout = Vec::new();
  let mut found_via = false;

  // :278
  for link in joint.links() {
    let Some(item) = world.item(*link) else {
      continue;
    };

    if item.of_kind(Kind::SEGMENT | Kind::ARC) {
      let mut index = 0;
      // :284
      let mut line =
        world.assemble_line(node, *link, Some(&mut index), false, false, true);

      // :286
      if index != 0 {
        line.reverse();
      }

      // :288
      fanout.push(ViaFanoutItem::Line(Box::new(line)));
    } else if item.of_kind(Kind::VIA) && !found_via {
      // :293, :294
      fanout.push(ViaFanoutItem::Via(*link));
      found_via = true;
    }
  }

  fanout
}

/// A copy of a stored via, moved to a point and ready to be added.
///
/// The `Clone( *via ); nvia->SetPos( aP )` pair of `:478` and `:480`, and
/// again of `:508` and `:510`. KiCad's via copy constructor gives the
/// copy a **fresh** hole rather than aliasing the original's
/// (`pcbnew/router/pns_via.h:126`), and so must this: a hole is a
/// separate arena item, and letting the copy keep the original's handle
/// would re home that hole into whatever node the copy lands in, so
/// dropping the node would take the original's drill with it.
/// [`World::add_via`] drills the replacement.
///
/// [`None`] for a handle the node no longer knows, and for one that does
/// not name a via.
fn cloned_via_at(world: &World, via: ItemId, at: Vec2) -> Option<Item> {
  let stored = world.item(via)?;

  if !matches!(stored.body(), ItemBody::Via(_)) {
    return None;
  }

  let mut moved = stored.clone();

  moved.set_hole(None);
  move_via_to(&mut moved, at);

  Some(moved)
}

/// Where a via item sits.
///
/// The `draggedVia->Pos()` of `:523`, read off an item that is not in a
/// node yet, so [`World::find_via_by_handle`] cannot answer it.
fn via_position(item: &Item) -> Option<Vec2> {
  match item.body() {
    ItemBody::Via(body) => Some(body.pos()),
    _ => None,
  }
}

// ---------------------------------------------------------------------
// The post drag anchor
// ---------------------------------------------------------------------

/// Whether the two segments meeting at a vertex make a corner the post
/// drag optimizer must not be asked to preserve.
///
/// Port of `pointHasBadCorner` (`:622`). A corner is bad when the angle
/// between the two directions is acute, right or a full reversal; an
/// endpoint is never bad (`:624`).
///
/// `DIRECTION_45( seg )` is built with the default `a90 = false` (`:629`,
/// `:630`), so a 90 degree corner classifies as
/// [`AngleType::RIGHT`] and not as straight.
fn point_has_bad_corner(chain: &LineChain, vertex_index: usize) -> bool {
  // :624
  if vertex_index == 0 || vertex_index + 1 >= chain.point_count() {
    return false;
  }

  // :626, :627
  let before =
    Seg::new(chain.point(vertex_index - 1), chain.point(vertex_index));
  let after =
    Seg::new(chain.point(vertex_index), chain.point(vertex_index + 1));

  // :633
  Direction45::from_seg(&before, false)
    .angle(Direction45::from_seg(&after, false))
    .intersects(AngleType::ACUTE | AngleType::RIGHT | AngleType::HALF_FULL)
}

/// The vertex the post drag optimizer should preserve for a cursor point
/// that is not on the line.
///
/// Port of `bestAnchorForPoint` (`:639`): the nearest point of the chain,
/// unless that lands on a vertex with a bad corner, in which case the
/// walk steps outwards one vertex at a time and takes the first good one,
/// preferring the nearer of the left and the right candidate.
///
/// Two details decide the answer. The comparison at `:664` is on
/// **squared** distances, so no rounding is involved, and a tie goes to
/// the right candidate, because the left one only wins on a strict `<`.
/// When the walk finds nothing, the bad nearest point is used after all
/// (`:681`).
fn best_anchor_for_point(chain: &LineChain, at: Vec2) -> Vec2 {
  // :641, :642. KiCad takes the default `aAllowInternalShapePoints` here,
  // which is `true`.
  let Some(nearest) = chain.nearest_point(at, true) else {
    return at;
  };
  let Some(vertex_index) = chain.find(nearest, 0) else {
    return nearest;
  };

  // :644
  if !point_has_bad_corner(chain, vertex_index) {
    return nearest;
  }

  let point_count = chain.point_count();

  // :650
  for offset in 1..point_count {
    // :652, :655
    let right = (vertex_index + offset < point_count
      && !point_has_bad_corner(chain, vertex_index + offset))
    .then(|| chain.point(vertex_index + offset));

    // :658
    if let Some(left_index) = vertex_index.checked_sub(offset)
      && !point_has_bad_corner(chain, left_index)
    {
      let left = chain.point(left_index);

      // :664
      if right.is_none_or(|right| {
        (left - at).squared_euclidean_norm()
          < (right - at).squared_euclidean_norm()
      }) {
        return left;
      }
    }

    // :672
    if let Some(right) = right {
      return right;
    }
  }

  // :681
  nearest
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::rules::FixedClearance;
  use crate::settings::RoutingSettings;

  /// A dragger whose dragged line has a width, which is all
  /// [`Dragger::snap_threshold`] reads off it.
  fn dragger_with_a_track_width(world: &World, width: i32) -> Dragger {
    let mut dragger = Dragger::new(world, world.root());

    dragger.dragged_line.set_width(width);

    dragger
  }

  #[test]
  fn the_two_snap_threshold_divisors_stay_apart() {
    // `pns_dragger.cpp:398` and `:727` against `:818`; note 06 section
    // 8.5 says not to normalise them.
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let dragger = dragger_with_a_track_width(&world, 200_000);
    let rules = FixedClearance::uniform(0);
    let settings = RoutingSettings {
      smooth_dragged_segments: true,
      ..RoutingSettings::default()
    };
    let context = AlgoContext::new(&rules, &settings);

    assert_eq!(
      dragger.snap_threshold(&context, MARK_OBSTACLES_SNAP_DIVISOR),
      50_000
    );
    // `:818`, the shove's `width / 2`.
    assert_eq!(
      dragger.snap_threshold(&context, SHOVE_SNAP_DIVISOR),
      100_000
    );
  }

  #[test]
  fn an_unsmoothed_drag_has_no_snap_threshold() {
    // The `: 0` half of `:398`.
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let dragger = dragger_with_a_track_width(&world, 200_000);
    let rules = FixedClearance::uniform(0);
    let settings = RoutingSettings {
      smooth_dragged_segments: false,
      ..RoutingSettings::default()
    };
    let context = AlgoContext::new(&rules, &settings);

    assert_eq!(
      dragger.snap_threshold(&context, MARK_OBSTACLES_SNAP_DIVISOR),
      0
    );
  }

  /// A chain out of its points.
  fn chain_of(points: &[Vec2]) -> LineChain {
    let mut chain = LineChain::new();

    for point in points {
      chain.append(*point);
    }

    chain
  }

  /// A right angled elbow with an obtuse corner past it: `A` and `D` are
  /// endpoints, `B` is the bad corner and `C` is a good one.
  fn elbow() -> LineChain {
    chain_of(&[
      Vec2::new(0, 0),
      Vec2::new(200_000, 0),
      Vec2::new(200_000, 1_000_000),
      Vec2::new(1_200_000, 2_000_000),
    ])
  }

  #[test]
  fn an_endpoint_is_never_a_bad_corner() {
    // `:624`, which is what stops the walk of `bestAnchorForPoint` from
    // running off either end.
    let chain = elbow();

    assert!(!point_has_bad_corner(&chain, 0));
    assert!(!point_has_bad_corner(&chain, chain.point_count() - 1));
    assert!(!point_has_bad_corner(&chain, chain.point_count()));
  }

  #[test]
  fn a_right_angle_is_a_bad_corner_and_an_obtuse_one_is_not() {
    // `DIRECTION_45( seg )` takes the default `a90 = false` (`:629`), so
    // 90 degrees classifies as `ANG_RIGHT` and lands in the mask at
    // `:633`; one octant apart is `ANG_OBTUSE` and does not.
    let chain = elbow();

    assert!(point_has_bad_corner(&chain, 1));
    assert!(!point_has_bad_corner(&chain, 2));
  }

  #[test]
  fn a_straight_run_is_not_a_bad_corner_and_a_reversal_is() {
    // `ANG_STRAIGHT` is outside the mask, `ANG_HALF_FULL` inside it.
    let straight = chain_of(&[
      Vec2::new(0, 0),
      Vec2::new(1_000_000, 0),
      Vec2::new(2_000_000, 0),
    ]);
    let reversal = chain_of(&[
      Vec2::new(0, 0),
      Vec2::new(1_000_000, 0),
      Vec2::new(500_000, 0),
    ]);

    assert!(!point_has_bad_corner(&straight, 1));
    assert!(point_has_bad_corner(&reversal, 1));
  }

  #[test]
  fn an_anchor_on_a_good_line_is_just_the_nearest_point() {
    // The early return at `:644`, taken both when the nearest point is
    // not a vertex at all and when it is a good one.
    let chain = chain_of(&[
      Vec2::new(0, 0),
      Vec2::new(1_000_000, 0),
      Vec2::new(2_000_000, 1_000_000),
    ]);

    assert_eq!(
      best_anchor_for_point(&chain, Vec2::new(500_000, 400_000)),
      Vec2::new(500_000, 0)
    );
    assert_eq!(
      best_anchor_for_point(&chain, Vec2::new(1_100_000, -100_000)),
      Vec2::new(1_000_000, 0)
    );
  }

  #[test]
  fn an_anchor_on_a_bad_corner_walks_out_to_the_nearer_good_vertex() {
    // `:658` to `:670`: the left candidate wins on a strict `<` of the
    // squared distances, so a nearer left vertex is taken.
    let chain = elbow();
    let at = Vec2::new(300_000, -100_000);

    // The nearest point is the bad corner itself.
    assert_eq!(chain.nearest_point(at, true), Some(Vec2::new(200_000, 0)));
    assert_eq!(best_anchor_for_point(&chain, at), Vec2::new(0, 0));
  }

  #[test]
  fn an_anchor_walk_that_ties_goes_to_the_right_candidate() {
    // `:664` compares **squared** distances with `<`, so an exact tie
    // falls through to the right candidate at `:672`.
    let chain = chain_of(&[
      Vec2::new(0, 0),
      Vec2::new(1_000_000, 0),
      Vec2::new(1_000_000, 1_000_000),
      Vec2::new(2_000_000, 2_000_000),
    ]);
    let at = Vec2::new(1_100_000, -100_000);

    assert_eq!(chain.nearest_point(at, true), Some(Vec2::new(1_000_000, 0)));
    assert!(point_has_bad_corner(&chain, 1));
    assert_eq!(
      best_anchor_for_point(&chain, at),
      Vec2::new(1_000_000, 1_000_000)
    );
  }

  #[test]
  fn a_fresh_dragger_reports_the_constructors_mode() {
    // `:44` seeds `m_mode = DM_SEGMENT`, which `Start` always overwrites.
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let dragger = Dragger::new(&world, root);

    assert_eq!(dragger.mode(), DragMode::Segment);
    assert_eq!(dragger.current_node(), root);
    assert!(dragger.traces().is_empty());
    assert_eq!(dragger.force_mark_obstacles_mode(), (false, false));
  }
}
