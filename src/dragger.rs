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
//! Steps 1 to 3 of that order, which is the mark obstacles path end to
//! end:
//!
//! - [`Dragger::start`] with the mode decision of `startDragSegment`
//!   (`:118`) and the state half of `startDragVia` (`:257`);
//! - [`Dragger::drag`]'s dispatch, its first drag fallback and its
//!   restore branch (`:998`);
//! - `dragMarkObstacles` (`:381`) for the segment and corner cases,
//!   which is [`Dragger::drag`] in [`RouterMode::MarkObstacles`], in free
//!   angle mode, and after any first drag failure;
//! - [`Dragger::traces`], [`Dragger::current_node`],
//!   [`Dragger::current_nets`], [`Dragger::force_mark_obstacles_mode`]
//!   and [`Dragger::fix_route`] as far as mark obstacles needs it.
//!
//! # What is not implemented yet
//!
//! - `dragWalkaround` (`:709`) and `tryWalkaround` (`:685`), step 6.
//!   `Dragger::drag_walkaround` is a stub that falls back to mark
//!   obstacles.
//! - `dragShove` (`:802`), step 8. `Dragger::drag_shove` is the same
//!   kind of stub. The [`Shove`] itself is already constructed by
//!   [`Dragger::start`], because that is where KiCad builds it (`:325`)
//!   and the node tree depends on it.
//! - `optimizeAndUpdateDraggedLine` (`:569`), `bestAnchorForPoint`
//!   (`:639`) and `pointHasBadCorner` (`:622`), step 5. Nothing in the
//!   mark obstacles path reaches them: `dragMarkObstacles` is the one
//!   drag routine that never optimizes.
//! - Everything about the via drag except the handles `startDragVia`
//!   stores: `findViaFanoutByHandle` (`:267`), `dragViaMarkObstacles`
//!   (`:452`), `dragViaWalkaround` (`:492`) and `propagateViaForces`
//!   (`:62`), step 9.
//! - The session facade (`ROUTER::StartDragging`,
//!   `pcbnew/router/pns_router.cpp:166`), the event log and the KiCad
//!   corpus replay, step 4. Nothing in `src/router.rs` reaches a
//!   [`Dragger`] yet.
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
//! - `MULTI_DRAGGER` (`pcbnew/router/pns_multi_dragger.cpp`) and
//!   `COMPONENT_DRAGGER`, both out of scope. There is deliberately no
//!   `DRAG_ALGO` trait either: `DESIGN.md` section 11 and note 06 section
//!   9.3 both ask for enum dispatch in the facade rather than a trait
//!   with one live implementation.
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
use crate::geometry::direction45::Direction45;
use crate::geometry::vec2::Vec2;
use crate::item::{ItemBody, ItemId, Kind, MarkerFlags, NetId};
use crate::line::Line;
use crate::mouse_trail::MouseTrailTracer;
use crate::node::{NodeId, World};
use crate::settings::RouterMode;
use crate::shove::{Shove, ViaHandle};

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
/// one. Step 8 passes `2` here.
const MARK_OBSTACLES_SNAP_DIVISOR: i32 = 4;

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
  /// KiCad's `ITEM_SET` can hold a dragged `VIA` as well as the lines
  /// (`:511`), which is what the via drag of step 9 will need; the mark
  /// obstacles path for a segment or a corner only ever puts one line in
  /// it (`:413`).
  dragged_items: Vec<Line>,
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
    self.dragged_items.clear();
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
  /// The routines that make the via actually move,
  /// `dragViaMarkObstacles` (`:452`), `dragViaWalkaround` (`:492`) and
  /// `dragShove`'s `DM_VIA` case (`:908`), are step 9 of note 06 section
  /// 10.2 and are not here yet, so a via drag starts, reports
  /// [`DragMode::Via`] and then moves nothing.
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
  /// Until steps 6 and 8 land, `Dragger::drag_walkaround` and
  /// `Dragger::drag_shove` fall back to mark obstacles, which always
  /// succeeds, so points 2 to 4 cannot be observed yet.
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
    self.dragged_items.clear();

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
        self.dragged_items.clear();
        self.dragged_items.push(dragged);
      }
      DragMode::Via => {
        // :437. TODO(step 9): `dragViaMarkObstacles( m_initialVia,
        // m_lastNode, aP )`, `pcbnew/router/pns_dragger.cpp:452`, with
        // `findViaFanoutByHandle` behind it. Until then a via drag
        // rebuilds an untouched node and reports no dragged items, which
        // is what `:458` does for an empty fanout anyway.
        self.dragged_items.clear();
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
    // `CheckColliding( const ITEM_SET& )` builds its own options with
    // `m_limitCount = 1` (`pcbnew/router/pns_node.cpp:494`).
    let options = CollisionSearchOptions {
      limit_count: Some(1),
      ..CollisionSearchOptions::default()
    };

    self.dragged_items.iter().any(|line| {
      world
        .check_colliding_line(node, line, context.resolver, &options)
        .is_some()
    })
  }

  /// Move the drag and bend it around what is in the way.
  ///
  /// TODO(step 6): port `dragWalkaround` (`:709`) and `tryWalkaround`
  /// (`:685`), whose length limit factor is **30.0** (`:692`), twenty
  /// times the placer's. Note 06 section 10.2 step 6 has the detail; it
  /// unlocks the `walk-with-teardrops` corpus case.
  ///
  /// Until then a walkaround drag is a mark obstacles drag, which is
  /// exactly what `Drag`'s own fallback would produce for it on the first
  /// failure (`:1029`).
  fn drag_walkaround(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
  ) -> bool {
    self.drag_mark_obstacles(world, context, at)
  }

  /// Move the drag and push what is in the way aside.
  ///
  /// TODO(step 8): port `dragShove` (`:802`), which needs the
  /// `preShoveNode->Remove( draggedPreShove )` ordering (`:830`),
  /// `SHP_SHOVE | SHP_DONT_LOCK_ENDPOINTS` plus `SHP_REVERSED` for a
  /// corner 0 drag (`:832`, `:837`), the `width / 2` snap threshold and
  /// `Dragger::last_drag_solution` (`:858`). Note 06 section 4 confirms
  /// [`Shove`] needs nothing new for it.
  ///
  /// Until then a shove drag is a mark obstacles drag. The [`Shove`] is
  /// still built by [`Dragger::start`], because it owns a node in the
  /// tree either way.
  fn drag_shove(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
  ) -> bool {
    self.drag_mark_obstacles(world, context, at)
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
  /// This calls [`World::commit`] where KiCad calls
  /// `ROUTER::CommitRouting`; the host half of that, the removed plus
  /// added fold that preserves object identity, belongs to the session
  /// facade and arrives with step 4.
  pub fn fix_route(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    force_commit: bool,
  ) -> bool {
    let node = self.current_node();

    // :963
    if self.drag_status {
      world.commit(node);

      return true;
    }

    // :968
    if self.force_mark_obstacles_mode {
      if force_commit {
        world.commit(node);

        return true;
      }

      return false;
    }

    // :983. Everything already committed is legal even when the current
    // cursor solution is not, so the drag is re-run at the last point
    // that worked.
    let at = self.last_valid_point;

    self.drag(world, context, at);

    if self.drag_status {
      world.commit(self.current_node());

      return true;
    }

    false
  }
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
    // `:818`, the shove's `width / 2`, which step 8 will pass.
    assert_eq!(dragger.snap_threshold(&context, 2), 100_000);
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
