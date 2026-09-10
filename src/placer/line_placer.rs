// SPDX-License-Identifier: GPL-3.0-or-later

//! Interactive placement of a single track.
//!
//! Port of `PNS::LINE_PLACER` (`pcbnew/router/pns_line_placer.h:113`,
//! `pcbnew/router/pns_line_placer.cpp`), the state machine behind "click
//! a start point, drag, click again". Note 03 sections 3 and 9.1 to 9.3
//! describe the shape this port takes and `DESIGN.md` section 6.4
//! summarises it.
//!
//! # The two level state
//!
//! KiCad encodes the lifecycle in four booleans plus the nullness of two
//! node pointers, and several combinations of those are unreachable. This
//! port splits the lifecycle out into [`PlacerState`] and keeps the live
//! geometry in [`Placing`], as note 03 section 9.1 recommends. `m_idle`
//! disappears into the outer enum, `m_placementCorrect` becomes
//! `Placing::placement_correct` plus [`PlacerState::Finished`], and
//! `m_p_start` becomes the derived [`Placing::p_start`].
//!
//! # Head and tail
//!
//! The head runs from the tail's last point to the cursor and is rebuilt
//! from scratch on every move. The tail is the part collisions have
//! already settled; it grows through [`Placing::merge_head`] and shrinks
//! through [`Placing::reduce_tail`], [`Placing::handle_pullback`] and
//! [`Placing::handle_self_intersections`], and every shrink makes
//! [`Placing::route_step`] run another pass so the head is rebuilt
//! against the shortened tail. "Fixed" in the tail's sense means fixed
//! *within this placement*: only [`LinePlacer::fix_route`] writes items
//! into a node.
//!
//! # Nodes
//!
//! The placer never owns a node. It holds three handles into the world's
//! arena, exactly KiCad's three pointers: the branch the placement runs
//! on (`m_world`), the node the algorithms route against
//! (`m_currentNode`) and a per move scratch branch that carries the
//! removed loops and the split end item (`m_lastNode`). The scratch
//! branch is dropped and rebuilt inside every
//! [`LinePlacer::move_to`], which is KiCad's `delete m_lastNode`
//! (`pcbnew/router/pns_line_placer.cpp:1492`) made explicit as note 03
//! section 9.3 asks.
//!
//! # Vias and the fixed tail
//!
//! A via is never a separate command. [`LinePlacer::toggle_via`] only
//! arms a flag, and the via itself is built, pushed out of whatever it
//! lands in and attached to the head inside
//! [`Placing::build_initial_line`] and [`Placing::rh_walk_only`], so
//! every mouse move re decides where it can sit. Only
//! [`LinePlacer::fix_route`] writes one into a node.
//!
//! The fixed tail ([`crate::placer::fixed_tail::FixedTail`]) is the undo
//! stack: [`LinePlacer::start`] pushes one stage, every intermediate fix
//! pushes another, and [`LinePlacer::undo_last_segment`] pops one and
//! rolls the placement back to it, node included.
//!
//! # What is not here yet
//!
//! `ContinueFromEnd` and `Finish` are host level commands that live on
//! KiCad's router rather than on its placer
//! (`pcbnew/router/pns_router.cpp:579`, `:624`), so they arrive with the
//! session facade of milestone 5.
//!
//! # Shove mode
//!
//! The placer owns a [`crate::shove::Shove`] built over a branch of the
//! placement node ([`LinePlacer::start`],
//! `pcbnew/router/pns_line_placer.cpp:1478`), and shove mode routes its
//! head through [`Placing::rh_shove_only`]. The shove owns a stack of
//! branches, so the node the placer routes against is the shove's rather
//! than its own from the first move onwards; that handoff is note 03
//! section 9.3 and it is why [`Placing::route_head`] answers with a node
//! as well as with two lines. Every fix pins the world it wrote into
//! ([`crate::shove::Shove::add_locked_springback_node`]), an undo rewinds
//! the springback to the stage it restores, and the commit takes the last
//! locked frame.

use std::collections::BTreeSet;

use crate::algo_base::AlgoContext;
use crate::collide::CollisionSearchOptions;
use crate::geometry::direction45::{AngleType, CornerMode, Direction45};
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::Vec2;
use crate::item::{
  Item, ItemBody, ItemId, Kind, LayerRange, NetId, Segment, Via,
};
use crate::line::{Line, LineVia};
use crate::mouse_trail::MouseTrailTracer;
use crate::node::{JointRef, NodeId, World};
use crate::optimizer::{EffortFlags, Optimizer};
use crate::placer::fixed_tail::FixedTail;
use crate::rules::RuleResolver;
use crate::settings::{OptimizerEffort, RouterMode, RoutingSettings, Sizes};
use crate::shove::{Shove, ShovePolicy, ShoveStatus};
use crate::topology;
use crate::via::{move_via_by, move_via_to, via_pushout_force};
use crate::walkaround::{WalkPolicy, Walkaround, WalkaroundStatus};

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

/// How many tail segments [`Placing::optimize_tail_head_transition`]
/// looks back over.
///
/// `tailLookbackSegments`, `pcbnew/router/pns_line_placer.cpp:1061`.
pub const TAIL_LOOKBACK_SEGMENTS: usize = 3;

/// How many head points that same routine folds into the candidate.
///
/// The `end = std::min( 2, head.PointCount() - 1 )` of
/// `pcbnew/router/pns_line_placer.cpp:1074`.
pub const HEAD_SLICE_LENGTH: usize = 2;

/// The fewest head shapes [`Placing::merge_head`] considers established.
///
/// The `n_head < 3` gate of `pcbnew/router/pns_line_placer.cpp:335`,
/// which is why the head normally keeps two shapes live under the cursor.
pub const MIN_HEAD_SHAPES: usize = 3;

/// The fewest tail shapes
/// [`Placing::optimize_tail_head_transition`] works on.
///
/// `pcbnew/router/pns_line_placer.cpp:1068`.
pub const MIN_TAIL_SHAPES: usize = 3;

/// The fewest tail segments [`Placing::reduce_tail`] works on.
///
/// `pcbnew/router/pns_line_placer.cpp:267`.
pub const MIN_REDUCE_TAIL_SEGMENTS: usize = 2;

/// The self intersection index below which the whole trace restarts.
///
/// The `n < 2` cut off of `pcbnew/router/pns_line_placer.cpp:151`.
pub const SELF_INTERSECTION_RESTART_INDEX: usize = 2;

/// How many rounds [`Placing::rh_walk_base`] runs while placing a via.
///
/// The `round < 2` of `pcbnew/router/pns_line_placer.cpp:713`. Without a
/// via the loop runs exactly once; with one it walks again from the point
/// the first walk ended at, so the via is pushed out of the way of what
/// the walk actually produced rather than of the straight guess.
pub const WALK_ROUNDS_WITH_VIA: u32 = 2;

/// How many leads [`Placing::build_initial_line`] tries to push a via
/// along.
///
/// The `attempt < 2` of `pcbnew/router/pns_line_placer.cpp:2117`: first
/// the vector from the leg's start to the cursor, then the vector from
/// the previous cursor position, which is roughly "back the way the
/// mouse came".
pub const VIA_PUSHOUT_ATTEMPTS: u32 = 2;

/// How much longer than the direct path a complete walkaround may be.
///
/// The `2.0 *` of `pcbnew/router/pns_line_placer.cpp:592`, on top of
/// [`RoutingSettings::walkaround_hug_length_threshold`]. A full
/// walkaround up to three times the direct length is therefore preferred
/// over a partial hug at the default threshold of 1.5.
pub const COMPLETE_WALK_THRESHOLD_FACTOR: f64 = 2.0;

/// The corner angles [`Placing::merge_head`] refuses to move into the
/// tail.
///
/// `ForbiddenAngles`, `pcbnew/router/pns_line_placer.cpp:325`.
pub const FORBIDDEN_ANGLES: AngleType = AngleType::ACUTE
  .union(AngleType::HALF_FULL)
  .union(AngleType::UNDEFINED);

/// A safety bound on the re run loop of [`Placing::route_step`].
///
/// # Deviation
///
/// KiCad has no such bound (`pcbnew/router/pns_line_placer.cpp:1128`):
/// its loop trusts that every `n_iter++` follows a strict shortening of
/// the tail. That holds for the shrinking routines themselves, but the
/// head routine that runs on the next pass may hand back a longer tail,
/// so the argument is empirical rather than structural. `DESIGN.md`
/// section 8 asks for explicit iteration budgets instead of wall clock
/// budgets, so the loop gets one. It is far above any pass count a real
/// route reaches, so nothing observable changes.
pub const ROUTE_STEP_ITERATION_LIMIT: usize = 64;

/// The clearance query options KiCad's yes or no collision overload
/// builds.
///
/// `NODE::CheckColliding( const ITEM*, int aKindMask )`
/// (`pcbnew/router/pns_node.h:342`) sets the kind mask from its argument
/// and a limit of one, since the answer is only whether anything was hit.
fn collision_options(kind_mask: Kind) -> CollisionSearchOptions<'static> {
  CollisionSearchOptions {
    limit_count: Some(1),
    kind_mask,
    ..CollisionSearchOptions::default()
  }
}

// ---------------------------------------------------------------------
// PlacerState
// ---------------------------------------------------------------------

/// Where a [`LinePlacer`] is in its lifecycle.
///
/// The outer half of the two level split note 03 section 9.1 recommends.
/// KiCad spells the same three states as `m_idle` plus
/// `m_placementCorrect` plus the nullness of `m_currentNode`.
#[derive(Clone, Debug)]
pub enum PlacerState {
  /// Nothing is being placed. KiCad's `m_idle == true` before the first
  /// `Start` (`pcbnew/router/pns_line_placer.cpp:50`).
  Idle {
    /// The layer a placement started here would run on. KiCad's
    /// `m_currentLayer`, which `ROUTER::StartRouting` writes through
    /// `SetLayer` before `Start` (`pcbnew/router/pns_router.cpp:468`).
    layer: i32,
  },
  /// A track is being placed.
  ///
  /// The payload is boxed because it is two lines, a posture solver and
  /// a copy of the sizes, some three orders of magnitude larger than the
  /// other two variants; an idle placer would otherwise carry all of it.
  /// There is one placer per session, so the indirection costs one
  /// allocation per `Start`.
  Placing(Box<Placing>),
  /// The placement ended, either at a real end point or through a via
  /// only commit. KiCad's `m_idle == true` again
  /// (`pcbnew/router/pns_line_placer.cpp:1631`, `:1752`).
  Finished {
    /// Whether anything reached a node. Port of the answer
    /// `HasPlacedAnything` gives (`:1804`).
    placed_anything: bool,
  },
}

impl PlacerState {
  /// The live placement, when there is one.
  pub fn placing(&self) -> Option<&Placing> {
    match self {
      PlacerState::Placing(placing) => Some(placing.as_ref()),
      _ => None,
    }
  }

  /// The live placement, mutably.
  pub fn placing_mut(&mut self) -> Option<&mut Placing> {
    match self {
      PlacerState::Placing(placing) => Some(placing.as_mut()),
      _ => None,
    }
  }
}

// ---------------------------------------------------------------------
// Placing
// ---------------------------------------------------------------------

/// The live geometry of one placement.
///
/// The inner half of note 03 section 9.1's split. Every field is one of
/// `LINE_PLACER`'s (`pcbnew/router/pns_line_placer.h:375` to `:416`)
/// except that `m_p_start` is derived by [`Placing::p_start`] and
/// `m_currentTrace`, a cache KiCad's `Traces()` writes and `FlipPosture`
/// reads, is recomputed instead; see [`LinePlacer::flip_posture`].
#[derive(Clone, Debug)]
pub struct Placing {
  /// The volatile part of the track, from [`Placing::p_start`] to the
  /// cursor. Port of `m_head` (`:378`).
  head: Line,
  /// The part collisions have settled. Port of `m_tail` (`:381`).
  tail: Line,
  /// The direction the head is expected to leave the tail with. Port of
  /// `m_direction` (`:375`).
  direction: Direction45,
  /// The direction a fresh trace starts in. Port of
  /// `m_initial_direction` (`:376`).
  initial_direction: Direction45,
  /// The previous cursor position. Port of `m_last_p_end` (`:388`),
  /// which only the second via pushout attempt reads (`:2122`).
  last_p_end: Option<Vec2>,
  /// Where the current leg starts. Port of `m_currentStart` (`:404`).
  current_start: Vec2,
  /// Where the head currently ends, which is not the cursor when
  /// something is in the way. Port of `m_currentEnd` (`:403`).
  current_end: Vec2,
  /// The start point of the last fix. Port of `m_fixStart` (`:386`).
  fix_start: Vec2,
  /// The item under the cursor when the placement started. Port of
  /// `m_startItem` (`:407`).
  start_item: Option<ItemId>,
  /// The item under the cursor now. Port of `m_endItem` (`:408`).
  end_item: Option<ItemId>,
  /// Whether the next fix places a via. Port of `m_placingVia` (`:398`).
  ///
  /// Only [`LinePlacer::toggle_via`] and a fix write it. It gates the via
  /// branch of [`Placing::build_initial_line`], the second walk round of
  /// [`Placing::rh_walk_base`] and two of the rules in
  /// [`LinePlacer::fix_route`].
  placing_via: bool,
  /// Whether the head is forced to a single 90 or 45 degree segment.
  /// Port of `m_orthoMode` (`:412`).
  ortho: bool,
  /// Whether the last fix left no via, which pins the layer. Port of
  /// `m_chainedPlacement` (`:411`). Note 03 section 9.1 calls this the
  /// one KiCad flag that must not be simplified away.
  chained: bool,
  /// Whether a fix has succeeded. Port of `m_placementCorrect` (`:413`).
  placement_correct: bool,
  /// The net being routed. Port of `m_currentNet` (`:400`).
  net: Option<NetId>,
  /// The layer being routed on. Port of `m_currentLayer` (`:401`).
  layer: i32,
  /// The geometry the track is placed with. Port of `m_sizes` (`:396`).
  sizes: Sizes,
  /// The posture solver. Port of `m_mouseTrailTracer` (`:416`).
  posture: MouseTrailTracer,
  /// The undo stack. Port of `m_fixedTail` (`:415`).
  ///
  /// [`LinePlacer::start`] pushes one stage and every intermediate fix
  /// pushes another; [`LinePlacer::undo_last_segment`] pops them.
  fixed_tail: FixedTail,
}

impl Placing {
  /// The boundary between the tail and the head.
  ///
  /// Port of `updatePStart`,
  /// `pcbnew/router/pns_line_placer.cpp:1102`: the tail's last point, or
  /// `Placing::current_start` when the tail is empty.
  ///
  /// # Deviation
  ///
  /// KiCad stores this in `m_p_start` and refreshes it at two points
  /// inside `routeStep` (`:1142`, `:1161`). Note 03 section 9.1 asks for
  /// a method instead, because the rule is total. The one place KiCad
  /// writes `m_p_start` from something other than the tail is
  /// `buildInitialLine`'s manually forced branch (`:2063`), which sets it
  /// to the tail's **first** point and then clears the tail; the tail's
  /// first point is always `Placing::current_start`, because every
  /// routine that shortens the tail cuts from its end, so dropping that
  /// write changes nothing.
  pub fn p_start(&self) -> Vec2 {
    self.tail.last_point().unwrap_or(self.current_start)
  }

  /// The geometry this placement uses.
  ///
  /// Port of `m_sizes`, `pcbnew/router/pns_line_placer.h:396`. The track
  /// width is pushed into the head and the tail by
  /// [`LinePlacer::start`]; the via diameter, the drill, the type and the
  /// layer pair are read on every via by [`Placing::make_via`].
  pub const fn sizes(&self) -> &Sizes {
    &self.sizes
  }

  /// The via a fix would place at a point.
  ///
  /// Port of `makeVia`, `pcbnew/router/pns_line_placer.cpp:76`. The
  /// diameter, the drill and the type come off [`Placing::sizes`]; the
  /// layer span comes off [`Sizes::via_layer_range`], the port of the one
  /// non virtual host helper (`pcbnew/router/pns_router.h:144`), and
  /// falls back to the layer being routed when the sizes carry no layer
  /// pair.
  ///
  /// KiCad's `makeVia` leaves the net null and lets each of its three
  /// callers set it (`:1506`, `:2106`) or lets `LINE::AppendVia` do it
  /// (`pcbnew/router/pns_line.cpp:1424`). It is set here as well, so that
  /// the via only commit of [`LinePlacer::fix_route`], which stores a via
  /// no line ever adopted, still puts it on the routed net.
  pub fn make_via(&self, world: &mut World, at: Vec2) -> Item {
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
    item.set_net(self.net);

    item
  }

  /// The whole routed line, tail followed by head.
  ///
  /// Port of `Trace()`, `pcbnew/router/pns_line_placer.cpp:1233`. The
  /// concatenation is only simplified when it has more than two points
  /// (`:1241`), so that the zero length feedback line `routeStep` leaves
  /// behind survives to the caller. The result inherits the head's width,
  /// layers, net and via.
  pub fn trace(&self) -> Line {
    let mut chain = self.tail.shape().clone();

    chain.append_chain(self.head.shape());

    // :1241
    if chain.point_count() > 2 {
      chain.simplify(0);
    }

    let mut line = self.head.clone();

    line.set_shape(chain);
    line
  }

  /// Point the trace in a new direction.
  ///
  /// Port of `setInitialDirection`,
  /// `pcbnew/router/pns_line_placer.cpp:95`, which also writes the
  /// current direction while the tail is empty.
  fn set_initial_direction(&mut self, direction: Direction45) {
    self.initial_direction = direction;

    if self.tail.segment_count() == 0 {
      self.direction = direction;
    }
  }

  // -----------------------------------------------------------------
  // Tail management primitives
  // -----------------------------------------------------------------

  /// Cut the tail back to where the head crosses it.
  ///
  /// Port of `handleSelfIntersections`,
  /// `pcbnew/router/pns_line_placer.cpp:104`. Three outcomes: a head that
  /// starts where the tail does is a completely new trace and the tail
  /// goes (`:118`); a crossing on the first or second tail segment
  /// restarts the whole trace (`:151`); anything else clips the tail at
  /// the crossing and adopts the direction of the segment before it
  /// (`:163`). A crossing exactly at the junction of head and tail is the
  /// normal case and is ignored (`:146`).
  ///
  /// Returns whether the line changed.
  pub fn handle_self_intersections(&mut self) -> bool {
    // :111
    if self.tail.point_count() < 2 {
      return false;
    }

    // :114
    if self.head.point_count() < 2 {
      return false;
    }

    // :118
    if self.tail.point(0) == self.head.point(0) {
      self.direction = self.initial_direction;
      self.tail.chain_mut().clear();
      return true;
    }

    // :125. KiCad's default argument means "include collinear and
    // touching"; see `LineChain::intersect_chain`.
    let hits = self.tail.shape().intersect_chain(self.head.shape(), true);

    // :128
    if hits.is_empty() {
      return false;
    }

    let mut index = usize::MAX;
    let mut point = Vec2::new(0, 0);

    // :136, the crossing closest to the beginning of the tail.
    for hit in &hits {
      if hit.ours.index() < index {
        index = hit.ours.index();
        point = hit.point;
      }
    }

    // :146, the point where the head and the tail meet is the normal
    // junction and not a crossing.
    if point == self.head.point(0) || Some(point) == self.tail.last_point() {
      return false;
    }

    if index < SELF_INTERSECTION_RESTART_INDEX {
      // :151
      self.direction = self.initial_direction;
      self.tail.chain_mut().clear();
      self.head.chain_mut().clear();

      return true;
    }

    // :163
    let last = self.tail.segment(index - 1);

    self.direction = Direction45::from_seg(&last, false);

    let point_count = self.tail.point_count();

    self.tail.chain_mut().remove_range(index, point_count - 1);

    true
  }

  /// Give the tail back a segment when the head doubles back on it.
  ///
  /// Port of `handlePullback`,
  /// `pcbnew/router/pns_line_placer.cpp:173`. When the first head
  /// direction meets the last tail direction at a right or acute angle
  /// (`:218`), the last tail shape is dropped and the routing direction
  /// becomes that of the dropped segment, in the hope that the next pass
  /// produces a cleaner head.
  ///
  /// `pullback_1`, the "the computed head does not follow the current
  /// direction" case, is hard disabled at `:213` and is not ported; note
  /// 03 section 9.5 lists it.
  ///
  /// Returns whether the line changed.
  pub fn handle_pullback(&mut self) -> bool {
    // :178
    if self.head.point_count() < 2 {
      return false;
    }

    let point_count = self.tail.point_count();

    // :183
    if point_count == 0 {
      return false;
    }

    // :187, a one point tail is simply dropped.
    if point_count == 1 {
      self.tail.chain_mut().clear();
      return true;
    }

    let first_head = Direction45::from_seg(&self.head.segment(0), false);
    let last_segment_index = point_count - 2;
    let last_tail =
      Direction45::from_seg(&self.tail.segment(last_segment_index), false);
    let angle = first_head.angle(last_tail);

    // :218
    if angle != AngleType::RIGHT && angle != AngleType::ACUTE {
      return false;
    }

    // :224
    self.direction =
      Direction45::from_seg(&self.tail.segment(last_segment_index), false);

    // :241. `RemoveShape( -1 )` on an arc free chain is the removal of
    // the last point.
    self.tail.chain_mut().remove(point_count - 1);

    // :246
    if self.tail.segment_count() == 0 {
      self.direction = self.initial_direction;
    }

    true
  }

  /// Replace the last tail segments with a straight run to the cursor.
  ///
  /// Port of `reduceTail`,
  /// `pcbnew/router/pns_line_placer.cpp:256`. It scans the tail
  /// backwards, building the trace each segment would be replaced by, and
  /// **breaks** on the first replacement that collides (`:291`) while
  /// recording a candidate only when the replacement leaves in the same
  /// direction as the segment it replaces (`:294`). The last candidate
  /// recorded, that is the earliest one, wins.
  ///
  /// The replacement is built in
  /// [`crate::geometry::direction45::CornerMode::Mitered45`] whatever the
  /// settings say, because KiCad calls `BuildInitialTrace( s.A, aEnd )`
  /// with both default arguments (`:284`).
  ///
  /// # Decision on note 03 section 9.6 item 8
  ///
  /// `reducedLine` (`:305`) is built out of the winning candidate and
  /// then never used. It is not ported: it has no side effect, so
  /// dropping it cannot change an answer, and reproducing dead work would
  /// only invite a later reader to wire it up.
  ///
  /// Returns whether the line changed.
  pub fn reduce_tail(
    &mut self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    end: Vec2,
  ) -> bool {
    // :263
    if self.head.segment_count() < 1 {
      return false;
    }

    // :267
    if self.tail.segment_count() < MIN_REDUCE_TAIL_SEGMENTS {
      return false;
    }

    let mut new_direction = Direction45::default();
    let mut reduce_index: Option<usize> = None;
    let options = collision_options(Kind::ANY);

    // :277
    for index in (0..self.tail.segment_count()).rev() {
      let segment = self.tail.segment(index);
      let direction = Direction45::from_seg(&segment, false);
      let replacement = LineChain::from_points(
        direction.build_initial_trace(
          segment.a,
          end,
          false,
          CornerMode::Mitered45,
        ),
        false,
      );

      // :286
      if replacement.segment_count() < 1 {
        continue;
      }

      let candidate = Line::with_chain(&self.tail, replacement.clone());

      // :291
      if world
        .check_colliding_line(node, &candidate, context.resolver, &options)
        .is_some()
      {
        break;
      }

      // :294
      if Direction45::from_seg(&replacement.segment(0), false) == direction {
        new_direction = direction;
        reduce_index = Some(index);
      }
    }

    // :302
    if let Some(index) = reduce_index {
      let point_count = self.tail.point_count();

      self.direction = new_direction;
      self
        .tail
        .chain_mut()
        .remove_range(index + 1, point_count - 1);
      self.head.chain_mut().clear();

      return true;
    }

    // :313
    if self.tail.segment_count() == 0 {
      self.direction = self.initial_direction;
    }

    false
  }

  /// Move an established head into the tail.
  ///
  /// Port of `mergeHead`,
  /// `pcbnew/router/pns_line_placer.cpp:320`. The head has to have grown
  /// to at least [`MIN_HEAD_SHAPES`] shapes, to continue the tail without
  /// a gap, and to contain no forbidden corner either inside itself or at
  /// the junction with the tail.
  ///
  /// Returns whether the line changed.
  pub fn merge_head(&mut self) -> bool {
    // :329
    self.head.chain_mut().simplify(0);
    self.tail.chain_mut().simplify(0);

    let head_shapes = self.head.shape_count();
    let tail_shapes = self.tail.shape_count();

    // :335
    if head_shapes < MIN_HEAD_SHAPES {
      return false;
    }

    // :341
    if tail_shapes > 0 && Some(self.head.point(0)) != self.tail.last_point() {
      return false;
    }

    // :347
    if self.head.count_corners(FORBIDDEN_ANGLES) != 0 {
      return false;
    }

    let head_direction = Direction45::from_seg(&self.head.segment(0), false);

    // :357
    if tail_shapes > 0 {
      let last = self.tail.segment(self.tail.segment_count() - 1);
      let tail_direction = Direction45::from_seg(&last, false);

      if head_direction
        .angle(tail_direction)
        .intersects(FORBIDDEN_ANGLES)
      {
        return false;
      }
    }

    // :371
    let head_chain = self.head.shape().clone();

    self.tail.chain_mut().append_chain(&head_chain);
    self.tail.chain_mut().simplify(0);

    // :377
    let last = self.tail.segment(self.tail.segment_count() - 1);

    self.direction = Direction45::from_seg(&last, false);

    // :382
    self.head.chain_mut().clear();

    // :387
    self.head.chain_mut().simplify(0);
    self.tail.chain_mut().simplify(0);

    true
  }

  /// Merge the last tail corners with the first head ones.
  ///
  /// Port of `optimizeTailHeadTransition`,
  /// `pcbnew/router/pns_line_placer.cpp:1038`, which `routeStep` tries
  /// before [`Placing::merge_head`] and which suppresses the merge when
  /// it succeeds.
  ///
  /// The fanout cleanup branch (`:1046`) can override the user's posture,
  /// which is why it is skipped while the posture is manually forced.
  ///
  /// # Decision on note 03 section 9.6 item 8
  ///
  /// The dead local `tmp` at `:1088` is not ported, for the same reason
  /// as `reduceTail`'s `reducedLine`: it has no side effect.
  ///
  /// Returns whether the line changed.
  pub fn optimize_tail_head_transition(
    &mut self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
  ) -> bool {
    let mut candidate = self.trace();

    // :1045
    if !self.posture.is_manually_forced()
      && Optimizer::optimize_line(
        world,
        context,
        node,
        &mut candidate,
        EffortFlags::FANOUT_CLEANUP,
        Vec2::new(0, 0),
      )
    {
      // :1048
      if candidate.segment_count() < 1 {
        return false;
      }

      self.direction = Direction45::from_seg(&candidate.segment(0), false);
      self.head = candidate;
      self.tail.chain_mut().clear();

      return true;
    }

    // :1066
    let threshold = self.tail.point_count().min(TAIL_LOOKBACK_SEGMENTS + 1);

    // :1068
    if self.tail.shape_count() < MIN_TAIL_SHAPES {
      return false;
    }

    let tail_point_count = self.tail.point_count();
    let Ok(mut optimized) = self
      .tail
      .shape()
      .slice(tail_point_count - threshold, tail_point_count - 1)
    else {
      return false;
    };

    // :1074. KiCad's `head.Slice( 0, min( 2, PointCount() - 1 ) )` reads
    // an out of range index on an empty head and answers with an empty
    // chain; nothing is appended in that case here either.
    if self.head.point_count() > 0 {
      let end = HEAD_SLICE_LENGTH.min(self.head.point_count() - 1);

      if let Ok(head_part) = self.head.shape().slice(0, end) {
        optimized.append_chain(&head_part);
      }
    }

    // :1078
    let mut new_head = Line::with_chain(&self.tail, optimized);

    // :1086
    if !Optimizer::optimize_line(
      world,
      context,
      node,
      &mut new_head,
      EffortFlags::MERGE_SEGMENTS,
      Vec2::new(0, 0),
    ) {
      return false;
    }

    // :1090
    self.head.chain_mut().clear();

    let replacement = new_head.shape().clone();

    self.tail.chain_mut().replace_with_chain(
      tail_point_count - threshold,
      tail_point_count - 1,
      &replacement,
    );
    self.tail.chain_mut().simplify(0);

    // :1094
    let last = new_head.segment(new_head.segment_count() - 1);

    self.direction = Direction45::from_seg(&last, false);

    true
  }

  // -----------------------------------------------------------------
  // The head routines
  // -----------------------------------------------------------------

  /// Turn the posture into geometry.
  ///
  /// Port of `buildInitialLine`,
  /// `pcbnew/router/pns_line_placer.cpp:2038`. The head is the trace the
  /// posture solver, or the current routing direction, draws from
  /// [`Placing::p_start`] to `at`.
  ///
  /// Note the asymmetry KiCad has and this keeps: the guessed posture is
  /// only consulted while the tail is empty (`:2081`), and the current
  /// routing direction takes over once there is one.
  ///
  /// # The via
  ///
  /// While a via is armed ([`LinePlacer::is_placing_via`]) and
  /// `force_no_via` is not set, the head is given a via at `at` (`:2102`
  /// to `:2140`). Mark obstacles mode attaches it as it is and lets the
  /// user see the collision; the other two modes push it clear with the
  /// port of `VIA::PushoutForce`
  /// (`pcbnew/router/pns_via.cpp:143`), retrace the head to the pushed
  /// out position and attach it there. Two leads are tried, the leg's own direction and the
  /// direction the cursor came from, and a via that neither lead can free
  /// makes the routine answer false. See [`Placing::rh_walk_base`] for
  /// why `force_no_via` never actually arrives set.
  ///
  /// Note the asymmetry KiCad has here too: the retrace after a pushout
  /// always uses the guessed posture (`:2127`), where the plain trace
  /// above uses the current routing direction as soon as there is a tail.
  ///
  /// Returns KiCad's `aViaOk`, false only when a via could not be pushed
  /// out of the way.
  #[allow(clippy::too_many_arguments)]
  pub fn build_initial_line(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    at: Vec2,
    head: &mut Line,
    mode: RouterMode,
    force_no_via: bool,
  ) -> bool {
    let guessed_direction = self.posture.get_posture(context, at);
    let mut corner_mode = context.settings.corner_mode;

    // :2048, rounded corners make no sense on a single ortho segment.
    if self.ortho {
      corner_mode = CornerMode::Mitered45;
    }

    // :2053. Dropping a one segment tail rather than inferring a
    // direction from it is what makes posture switching deterministic in
    // walkaround and shove modes.
    if self.posture.is_manually_forced()
      && self.tail.segment_count() == 1
      && self.head.segment_count() > 0
    {
      let matches = Direction45::from_seg(&self.tail.segment(0), false)
        == Direction45::from_seg(&self.head.segment(0), false);

      if matches || self.head.segment_count() == 1 {
        // :2063. `m_p_start = m_tail.CPoint( 0 )` is implied; see
        // `Placing::p_start`.
        self.tail.clear();
      }
    }

    let p_start = self.p_start();
    let mut chain = LineChain::new();

    // :2069
    if p_start != at {
      if context.settings.free_angle_mode
        && context.settings.mode == RouterMode::MarkObstacles
      {
        // :2077
        chain = LineChain::from_slice(&[p_start, at], false);
      } else if self.tail.point_count() == 0 {
        // :2082
        chain = LineChain::from_points(
          guessed_direction.build_initial_trace(
            p_start,
            at,
            false,
            corner_mode,
          ),
          false,
        );
      } else {
        // :2084
        chain = LineChain::from_points(
          self
            .direction
            .build_initial_trace(p_start, at, false, corner_mode),
          false,
        );
      }

      // :2087, collapse the two segment bend to one orthogonal segment.
      if chain.segment_count() > 1 && self.ortho {
        let projected = chain
          .segment(0)
          .line_project(chain.last_point().unwrap_or(at));
        let point_count = chain.point_count();

        chain.remove(point_count - 1);
        chain.set_point(1, projected);
      }
    }

    // :2096
    head.set_layer(self.layer);
    head.set_shape(chain);

    // :2102
    if !self.placing_via || force_no_via {
      return true;
    }

    // :2105
    let mut via = self.make_via(world, at);

    // :2106
    via.set_net(head.net());

    // :2108, the manual mode shows the via where the user asked for it,
    // collision and all.
    if mode == RouterMode::MarkObstacles {
      head.append_via(via);

      return true;
    }

    // :2114
    let collision_mask = if mode == RouterMode::Walkaround {
      Kind::ANY
    } else {
      Kind::SOLID
    };
    let iteration_limit = context.settings.via_force_prop_iteration_limit;

    // :2117
    for attempt in 0..VIA_PUSHOUT_ATTEMPTS {
      // :2119
      let mut lead = at - p_start;

      // :2122
      if attempt == 1
        && let Some(previous) = self.last_p_end
      {
        lead = at - previous;
      }

      let Some(force) = via_pushout_force(
        world,
        context,
        node,
        &via,
        lead,
        collision_mask,
        iteration_limit,
      ) else {
        continue;
      };

      // :2127
      let corrected = LineChain::from_points(
        guessed_direction.build_initial_trace(
          p_start,
          at + force,
          false,
          corner_mode,
        ),
        false,
      );

      // :2128. `LINE( aHead, line )` keeps the width, layers, net and
      // rank of the head and nulls its via
      // (`pcbnew/router/pns_line.h:81`), which is why the via is
      // attached after the retrace and not before it.
      *head = Line::with_chain(head, corrected);

      // :2130
      move_via_by(&mut via, force);

      // :2132
      head.append_via(via);

      return true;
    }

    // :2140
    false
  }

  /// Walk the tail and head around everything in the way.
  ///
  /// Port of `rhWalkBase`,
  /// `pcbnew/router/pns_line_placer.cpp:549`, the driver both the
  /// walkaround and the shove head routines share. It walks
  /// `tail ++ head` rather than the head alone, for the reason the long
  /// comment at `:775` gives: with the clearance epsilon in place, a head
  /// computed on its own can be collision free and still start inside a
  /// hull it has yet to process, which breaks the walkaround's
  /// precondition.
  ///
  /// A complete walkaround is accepted when it is shorter than
  /// [`COMPLETE_WALK_THRESHOLD_FACTOR`] times
  /// [`RoutingSettings::walkaround_hug_length_threshold`] times the
  /// direct path; otherwise each candidate is clipped to the point
  /// nearest the cursor by [`Placing::cursor_dist_minimum`] and the
  /// nearer of the two wins.
  ///
  /// # Decision on note 03 section 9.6 item 1
  ///
  /// KiCad increments `round` before testing `round == 0` (`:576` to
  /// `:578`), so the `aForceNoVia` argument of `buildInitialLine` is
  /// **always false** and the apparent intent, "do not place a via on the
  /// first round", never happens. The code is reproduced, not the intent:
  /// a regression suite built from KiCad logs pins the code, and changing
  /// it would move the via on every second round. The argument is passed
  /// through so that part 2 inherits the same expression.
  ///
  /// # The via
  ///
  /// While a via is being placed the loop runs twice (`:713`): the second
  /// round rebuilds the head, and with it the via, from the point the
  /// first walk actually ended at. Afterwards the via the last
  /// `buildInitialLine` produced is moved to the end of the walked line
  /// (`:716` to `:721`), because the walk itself never carries one: the
  /// line handed to the walkaround is `tail ++ l1.CLine()`, a chain
  /// without a via, so a via is never an obstacle the walk avoids.
  ///
  /// Returns the walked line and KiCad's `aViaOk`. `None` covers both of
  /// KiCad's false returns: no usable candidate (`:709`) and a walked
  /// line that ends with a via the pushout could not free (`:733`).
  pub fn rh_walk_base(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    at: Vec2,
    collision_mask: Kind,
    mode: RouterMode,
  ) -> Option<(Line, bool)> {
    let mut walk_full = self.head.clone();
    let mut walk_point = at;
    let mut walkaround = Walkaround::new(node, context.settings);

    // :562
    walkaround.set_solids_only(false);
    walkaround.set_iteration_limit(context.settings.walkaround_iteration_limit);
    walkaround.set_allowed_kinds(collision_mask);
    walkaround.set_allowed_policies(&[
      WalkPolicy::CounterClockwise,
      WalkPolicy::Clockwise,
    ]);

    let mut round: u32 = 0;
    let mut via_ok;
    // :553. KiCad declares `l1` outside the loop, which is what lets the
    // via transfer below read the via of the **last** round.
    let mut l1 = self.head.clone();

    loop {
      l1.clear();

      // :576, the increment that makes the `round == 0` test below dead.
      round += 1;

      via_ok = self.build_initial_line(
        world,
        context,
        node,
        walk_point,
        &mut l1,
        mode,
        round == 0,
      );

      // :584
      let mut initial_track = self.tail.clone();
      let l1_chain = l1.shape().clone();

      initial_track.chain_mut().append_chain(&l1_chain);
      initial_track.chain_mut().simplify(0);

      // :589
      let initial_length = initial_track.shape().length() as f64;
      let hug_threshold =
        initial_length * context.settings.walkaround_hug_length_threshold;
      let hug_threshold_complete = COMPLETE_WALK_THRESHOLD_FACTOR
        * initial_length
        * context.settings.walkaround_hug_length_threshold;

      // :594
      let result = walkaround.route(world, context, &initial_track);
      let mut best_line: Option<Line> = None;
      let mut clockwise = result.line(WalkPolicy::Clockwise).clone();
      let mut counter_clockwise =
        result.line(WalkPolicy::CounterClockwise).clone();

      // :605
      let mut length_cw =
        if result.status(WalkPolicy::Clockwise) != WalkaroundStatus::Stuck {
          clockwise.shape().length()
        } else {
          i64::from(i32::MAX)
        };
      let mut length_ccw = if result.status(WalkPolicy::CounterClockwise)
        != WalkaroundStatus::Stuck
      {
        counter_clockwise.shape().length()
      } else {
        i64::from(i32::MAX)
      };

      // :611
      if result.status(WalkPolicy::Clockwise) == WalkaroundStatus::Done {
        self.optimize_walk_candidate(
          world,
          context,
          node,
          collision_mask,
          &mut clockwise,
        );

        length_cw = clockwise.shape().length();
        best_line = Some(clockwise.clone());
      }

      // :631
      if result.status(WalkPolicy::CounterClockwise) == WalkaroundStatus::Done {
        self.optimize_walk_candidate(
          world,
          context,
          node,
          collision_mask,
          &mut counter_clockwise,
        );

        length_ccw = counter_clockwise.shape().length();

        if length_ccw < length_cw {
          best_line = Some(counter_clockwise.clone());
        }
      }

      // :653
      let best_length = length_cw.min(length_ccw);

      if (best_length as f64) < hug_threshold_complete
        && let Some(best) = best_line
      {
        let shape = best.shape().clone();

        walk_full.set_shape(shape);
        walk_point = walk_full.last_point().unwrap_or(walk_point);
      } else {
        // :663, the hug fallback.
        let mut clipped_cw = None;
        let mut clipped_ccw = None;

        if result.status(WalkPolicy::Clockwise) != WalkaroundStatus::Stuck {
          clipped_cw = self.cursor_dist_minimum(
            world,
            context,
            node,
            clockwise.shape(),
            at,
            hug_threshold,
          );
        }

        if result.status(WalkPolicy::CounterClockwise)
          != WalkaroundStatus::Stuck
        {
          clipped_ccw = self.cursor_dist_minimum(
            world,
            context,
            node,
            counter_clockwise.shape(),
            at,
            hug_threshold,
          );
        }

        let distance_cw = clipped_cw.as_ref().map_or(i32::MAX, |chain| {
          chain
            .last_point()
            .map_or(i32::MAX, |point| (at - point).euclidean_norm())
        });
        let distance_ccw = clipped_ccw.as_ref().map_or(i32::MAX, |chain| {
          chain
            .last_point()
            .map_or(i32::MAX, |point| (at - point).euclidean_norm())
        });

        // :696 to :709. A clockwise clip with the smaller distance wins,
        // otherwise the counter clockwise one, and neither means the walk
        // failed. `distance_cw` is `i32::MAX` whenever `clipped_cw` is
        // absent, so the comparison never selects a missing clip.
        let chain = if distance_cw < distance_ccw {
          clipped_cw
        } else {
          clipped_ccw
        }?;

        walk_point = chain.last_point().unwrap_or(walk_point);
        walk_full.set_shape(chain);
      }

      // :713
      if round >= WALK_ROUNDS_WITH_VIA || !self.placing_via {
        break;
      }
    }

    // :716. KiCad reads `walkFull.CLastPoint()` unchecked; a walked line
    // with no points has nowhere to put a via, so the transfer is
    // skipped rather than reading out of range.
    if let Some(LineVia::Owned(item)) = l1.via()
      && let Some(last) = walk_full.last_point()
    {
      let mut moved = item.clone();

      move_via_to(&mut moved, last);
      walk_full.append_via(moved);
    }

    // :733
    if walk_full.ends_with_via() && !via_ok {
      return None;
    }

    Some((walk_full, via_ok))
  }

  /// Run the per candidate optimization of `rhWalkBase`.
  ///
  /// `pcbnew/router/pns_line_placer.cpp:617` to `:624`, which each of the
  /// two winding branches performs identically: merge the whole
  /// candidate, split it back into a head and a tail, merge the head
  /// alone under the collision mask, and stitch the two together again.
  ///
  /// # Decision on note 03 section 9.6 item 4
  ///
  /// `splitHeadTail` never answers false (`:917`), so KiCad's
  /// `if( splitHeadTail( ... ) )` here is unconditional. The port makes
  /// that structural by returning the two lines rather than a boolean,
  /// which is what note 03 section 9.2 asks for.
  fn optimize_walk_candidate(
    &self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    collision_mask: Kind,
    candidate: &mut Line,
  ) {
    // :617
    Optimizer::optimize_line(
      world,
      context,
      node,
      candidate,
      EffortFlags::MERGE_SEGMENTS,
      Vec2::new(0, 0),
    );

    let (mut head, tail) = split_head_tail(candidate, &self.tail);
    let mut optimizer = Optimizer::new(node);

    optimizer.set_effort_level(EffortFlags::MERGE_SEGMENTS);
    optimizer.set_collision_mask(collision_mask);

    let input = head.clone();

    optimizer.optimize(world, context, &input, &mut head, None);

    // :622
    let mut chain = tail.shape().clone();

    chain.append_chain(head.shape());
    candidate.set_shape(chain);
  }

  /// The walkaround mode head routine.
  ///
  /// Port of `rhWalkOnly`,
  /// `pcbnew/router/pns_line_placer.cpp:737`. Smart pads is suppressed in
  /// the 90 degree corner modes, "incompatible with 90-degree mode for
  /// now" (`:761`), and while the posture is manually forced; the pad
  /// passes of `src/optimizer.rs` build their breakouts in 45 degree
  /// corners only, which is the same guard seen from the other side.
  ///
  /// The via at `:792` is a **fresh** one at the new head's last point
  /// rather than the one [`Placing::rh_walk_base`] carried:
  /// [`split_head_tail`] drops the via whenever the old tail survived,
  /// and the two positions agree anyway because the head ends where the
  /// walk did.
  pub fn rh_walk_only(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    at: Vec2,
  ) -> Option<(Line, Line)> {
    // :744
    let (walk_full, via_ok) = self.rh_walk_base(
      world,
      context,
      node,
      at,
      Kind::ANY,
      RouterMode::Walkaround,
    )?;
    let effort = self.optimizer_effort(context);

    // :769
    if world
      .check_colliding_line(
        node,
        &walk_full,
        context.resolver,
        &collision_options(Kind::ANY),
      )
      .is_some()
    {
      return None;
    }

    // :789
    let (mut new_head, new_tail) = split_head_tail(&walk_full, &self.tail);

    // :792
    if self.placing_via
      && via_ok
      && let Some(last) = new_head.last_point()
    {
      let via = self.make_via(world, last);

      new_head.append_via(via);
    }

    // :799
    Optimizer::optimize_line(
      world,
      context,
      node,
      &mut new_head,
      effort,
      Vec2::new(0, 0),
    );

    Some((new_head, new_tail))
  }

  /// The shove mode head routine.
  ///
  /// Port of `rhShoveOnly`,
  /// `pcbnew/router/pns_line_placer.cpp:921`. The head is first walked
  /// around **solids only**, because a pad can never be shoved, and the
  /// result is handed to the shove as its single head. On failure the
  /// whole move degrades to the walkaround (`:1013`), which is why shove
  /// mode never gets stuck where walkaround mode would not.
  ///
  /// # The node handoff
  ///
  /// Note 03 section 9.3. The shove owns a stack of branches and hands
  /// back whichever one the caller should stand on now, and KiCad reads
  /// that twice: once before the run (`:930`), because springback may
  /// already have dropped frames on a previous move, and once after
  /// (`:963`), because a successful run has just pushed a new one. Both
  /// reads happen here and the second node travels back to
  /// [`LinePlacer::move_to`] with the two lines.
  ///
  /// The end item is pinned before the run (`:935`): the placer holds an
  /// item that lives in one of the shove's nodes, and springback dropping
  /// that node left KiCad using freed memory. With no end item the pin is
  /// cleared again (`:944`), so springback can roll back past a frame an
  /// earlier obstacle touch pinned.
  ///
  /// # Deviation: the via after the split
  ///
  /// KiCad re-appends `newHead.Via()` at `:1003`, which is a pointer into
  /// the node `removeHeads` has just emptied. Here the via is rebuilt at
  /// the shoved head's last point instead, which is the same geometry
  /// with no stale handle: a head is the pusher at
  /// [`crate::shove::HEAD_RANK`], so nothing outranks it and the shove
  /// leaves its via where the placer put it.
  pub fn rh_shove_only(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    shove: &mut Shove,
    node: NodeId,
    at: Vec2,
  ) -> Option<(Line, Line, NodeId)> {
    // :927. Solids only: the shove moves everything else itself.
    let (walk_solids, via_ok) = self.rh_walk_base(
      world,
      context,
      node,
      at,
      Kind::SOLID,
      RouterMode::Shove,
    )?;

    // :930. The first of the two node handoffs (note 03 section 9.3):
    // springback may have dropped frames during an earlier move, so the
    // placer is re-pointed before the run rather than only after it.
    // Nothing between here and `:963` reads it, so all it really asserts
    // is that the shove is standing somewhere live.
    let mut node = shove.current_node();

    debug_assert!(
      world.node(node).is_some(),
      "the shove stands on a live node"
    );

    // :935 and :944.
    shove.set_pinned_node(self.end_item.and_then(|id| world.home_of(id)));

    let mut new_head = walk_solids;

    // :952
    if self.placing_via
      && via_ok
      && let Some(last) = new_head.last_point()
    {
      let via = self.make_via(world, last);

      new_head.append_via(via);
    }

    // :958
    shove.clear_heads();
    shove.add_head_line(new_head.clone(), ShovePolicy::SHOVE);

    let shove_ok = shove.run(world, context) == ShoveStatus::Ok;

    // :963
    node = shove.current_node();

    // :967 to :987, the same expression `rh_walk_only` uses.
    let effort = self.optimizer_effort(context);

    // :1013
    if !shove_ok {
      return self
        .rh_walk_only(world, context, node, at)
        .map(|(head, tail)| (head, tail, node));
    }

    // :991
    if shove.heads_modified(None)
      && let Some(modified) = shove.modified_head(0)
    {
      new_head = modified.clone();
    }

    // :1000
    let (mut head, tail) = split_head_tail(&new_head, &self.tail);

    // :1003, see the deviation above.
    if new_head.ends_with_via()
      && let Some(last) = new_head.last_point()
    {
      let via = self.make_via(world, last);

      head.append_via(via);
    }

    // :1005
    Optimizer::optimize_line(
      world,
      context,
      node,
      &mut head,
      effort,
      Vec2::new(0, 0),
    );

    Some((head, tail, node))
  }

  /// The mark obstacles mode head routine.
  ///
  /// Port of `rhMarkObstacles`,
  /// `pcbnew/router/pns_line_placer.cpp:808`. The head runs straight to
  /// the cursor and, when something is in the way and the cursor is
  /// within half a track width of that obstacle's hull, snaps to the
  /// hull. The comment at `:815` explains the snap: it lets a user route
  /// as tightly as possible without turning on shove or walkaround. The
  /// routine never fails, so mark obstacles mode never gets stuck.
  ///
  /// A via needs no handling of its own here: `buildInitialLine` attaches
  /// one directly in this mode (`:2108`), collision and all, and the
  /// second call below rebuilds both the trace and the via at the snapped
  /// point.
  ///
  /// The blocking obstacle of the head is cleared (`:811`) and never set:
  /// the only code that would set it is the `#if 0` "stop at first
  /// obstacle" sketch at `:840`, which note 03 section 9.5 lists as not
  /// to be ported. A host that wants the obstacle painted queries the
  /// node, which is what KiCad's `markViolations` does.
  pub fn rh_mark_obstacles(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    at: Vec2,
  ) -> Option<(Line, Line)> {
    // :810. KiCad builds into `m_head` itself, which note 03 section 9.2
    // calls an inconsistency worth removing; the head is built into a
    // copy here and handed back like every other mode's.
    let mut head = self.head.clone();

    self.build_initial_line(
      world,
      context,
      node,
      at,
      &mut head,
      RouterMode::MarkObstacles,
      false,
    );

    // :811
    head.set_blocking_obstacle(None);

    // :813
    let obstacle = world.nearest_obstacle(
      node,
      &head,
      context.resolver,
      &CollisionSearchOptions::default(),
      context.settings.corner_mode,
    );

    if let Some(obstacle) = obstacle
      && let Some(item) = obstacle.item
    {
      // :820
      let clearance =
        world.clearance_for_line(item, &head, false, context.resolver);
      let hull = world.hull_of(item, clearance, head.width(), head.layer());

      if let Some(hull) = hull {
        // :827
        let nearest = if context.settings.corner_mode == CornerMode::Mitered90 {
          hull.bbox(0).map(|bbox| {
            let clamped =
              bbox.nearest_point(crate::geometry::vec2::Vec2L::from(at));

            Vec2::new(clamped.x as i32, clamped.y as i32)
          })
        } else {
          hull.nearest_point(at)
        };

        // :832
        if let Some(nearest) = nearest
          && (nearest - at).euclidean_norm() < head.width() / 2
        {
          self.build_initial_line(
            world,
            context,
            node,
            nearest,
            &mut head,
            RouterMode::MarkObstacles,
            false,
          );
        }
      }
    }

    // :853
    Some((head, self.tail.clone()))
  }

  /// Compute the head for one cursor position.
  ///
  /// Port of `routeHead`,
  /// `pcbnew/router/pns_line_placer.cpp:1020`, a plain dispatch on the
  /// routing mode. Note 03 section 9.2 asks for a `match` rather than a
  /// trait object, because the three routines are not independent: shove
  /// falls back to walkaround and both share
  /// [`Placing::rh_walk_base`].
  ///
  /// Returns the new head, the new tail and the node the placer should
  /// route against from here on, where KiCad returns a boolean, writes
  /// two out parameters that are only valid when it is true, and lets
  /// `rhShoveOnly` assign `m_currentNode` behind the caller's back
  /// (note 03 section 9.3). Only shove mode ever answers with a different
  /// node.
  ///
  /// A shove mode call with no shove engine falls back to the walkaround,
  /// which is what a host that switched into shove mode without starting
  /// a placement would get.
  pub fn route_head(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    shove: &mut Option<Shove>,
    node: NodeId,
    at: Vec2,
  ) -> Option<(Line, Line, NodeId)> {
    match context.settings.mode {
      RouterMode::MarkObstacles => self
        .rh_mark_obstacles(world, context, node, at)
        .map(|(head, tail)| (head, tail, node)),
      RouterMode::Walkaround => self
        .rh_walk_only(world, context, node, at)
        .map(|(head, tail)| (head, tail, node)),
      RouterMode::Shove => match shove.as_mut() {
        Some(shove) => self.rh_shove_only(world, context, shove, node, at),
        None => self
          .rh_walk_only(world, context, node, at)
          .map(|(head, tail)| (head, tail, node)),
      },
    }
  }

  /// The optimizer effort the two walk routines ask for.
  ///
  /// `pcbnew/router/pns_line_placer.cpp:747` to `:767`, repeated verbatim
  /// at `:965` to `:987`.
  fn optimizer_effort(&self, context: &AlgoContext<'_>) -> EffortFlags {
    let mut effort = match context.settings.optimizer_effort {
      OptimizerEffort::Low => EffortFlags::NONE,
      OptimizerEffort::Medium | OptimizerEffort::Full => {
        EffortFlags::MERGE_SEGMENTS
      }
    };

    // :762. KiCad tests `MITERED_45 || ROUNDED_45`; this crate has no
    // rounded modes, so the 45 degree family is one variant.
    if context.settings.smart_pads
      && context.settings.corner_mode == CornerMode::Mitered45
      && !self.posture.is_manually_forced()
    {
      effort |= EffortFlags::SMART_PADS;
    }

    effort
  }

  // -----------------------------------------------------------------
  // Clipping a walk candidate to the cursor
  // -----------------------------------------------------------------

  /// Clip a chain at a point and check that the result is clear.
  ///
  /// Port of `clipAndCheckCollisions`,
  /// `pcbnew/router/pns_line_placer.cpp:394`. The candidate is rejected
  /// when it is shorter than the best one so far, and when the clipped
  /// prefix collides; on success the threshold rises, which is what makes
  /// the fallback loop of [`Placing::cursor_dist_minimum`] keep the
  /// longest clear prefix.
  ///
  /// # Deviation
  ///
  /// KiCad holds the length and the threshold in an `int`
  /// (`:406`). Both are path lengths, which
  /// [`crate::geometry::line_chain::LineChain::length`] computes in
  /// `i64`, so they stay `i64` here rather than being truncated on a
  /// board wider than two metres of accumulated path.
  pub fn clip_and_check_collisions(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    at: Vec2,
    chain: &LineChain,
    threshold_dist: &mut i64,
  ) -> Option<LineChain> {
    let mut split = chain.clone();

    // :398
    let index = split.split(at)?;
    let clipped = split.slice(0, index).ok()?;
    let distance = clipped.length();
    let mut accepted = true;

    // :411
    if distance < *threshold_dist {
      accepted = false;
    }

    // :414. The collision test runs whatever the length test said, as it
    // does in KiCad.
    let probe = Line::with_chain(&self.head, clipped.clone());

    if world
      .check_colliding_line(
        node,
        &probe,
        context.resolver,
        &collision_options(Kind::ANY),
      )
      .is_some()
    {
      accepted = false;
    }

    if !accepted {
      return None;
    }

    *threshold_dist = distance;

    Some(clipped)
  }

  /// Clip a walk candidate at the point closest to the cursor.
  ///
  /// Port of `cursorDistMinimum`,
  /// `pcbnew/router/pns_line_placer.cpp:429`. It builds the list of
  /// candidate cut points, every segment start plus the projection of the
  /// cursor onto each segment, stopping once the accumulated length
  /// passes the threshold, and clips at the one nearest the cursor. When
  /// that fails, every candidate is tried and the longest clear prefix
  /// wins.
  ///
  /// The local minimum search of `:485` to `:511` is dead code: `:515`
  /// overrides its result with `-1` under the comment "I didn't make my
  /// mind yet if local or global minimum feels better". Note 03 section
  /// 9.5 lists it as not to be ported, so only the global minimum is
  /// here.
  pub fn cursor_dist_minimum(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    chain: &LineChain,
    cursor: Vec2,
    length_threshold: f64,
  ) -> Option<LineChain> {
    // :435
    if chain.point_count() == 0 {
      return None;
    }

    let mut last_point = chain.last_point()?;
    let mut accumulated: i64 = 0;
    let mut distances: Vec<i32> = Vec::new();
    let mut points: Vec<Vec2> = Vec::new();

    // :443
    for index in 0..chain.segment_count() {
      let segment = chain.segment(index);

      distances.push((cursor - segment.a).euclidean_norm());
      points.push(segment.a);

      let nearest = segment.nearest_point_to_point(cursor);

      if nearest != segment.a && nearest != segment.b {
        distances.push((nearest - cursor).euclidean_norm());
        points.push(nearest);
      }

      accumulated += i64::from(segment.length());

      // :459
      if accumulated as f64 > length_threshold {
        last_point = segment.b;
        break;
      }
    }

    distances.push((cursor - last_point).euclidean_norm());
    points.push(last_point);

    // :474, the global minimum, which `:515` makes the only one.
    let mut preferred = 0usize;
    let mut best = i32::MAX;

    for (index, distance) in distances.iter().enumerate() {
      if *distance < best {
        best = *distance;
        preferred = index;
      }
    }

    let mut threshold_dist: i64 = 0;

    // :529
    if let Some(clipped) = self.clip_and_check_collisions(
      world,
      context,
      node,
      points[preferred],
      chain,
      &mut threshold_dist,
    ) {
      return Some(clipped);
    }

    // :532, the fallback keeps the longest clear prefix.
    threshold_dist = 0;

    let mut result = None;

    for point in &points {
      if let Some(clipped) = self.clip_and_check_collisions(
        world,
        context,
        node,
        *point,
        chain,
        &mut threshold_dist,
      ) {
        result = Some(clipped);
      }
    }

    result
  }

  // -----------------------------------------------------------------
  // The step
  // -----------------------------------------------------------------

  /// One routing pass towards the cursor.
  ///
  /// Port of `routeStep`,
  /// `pcbnew/router/pns_line_placer.cpp:1110`. Every mutation of the tail
  /// by [`Placing::handle_self_intersections`] or
  /// [`Placing::handle_pullback`] adds another pass, so the head is
  /// recomputed against the shortened tail; see
  /// [`ROUTE_STEP_ITERATION_LIMIT`] for the one deviation.
  ///
  /// The zero length line the failure path leaves behind (`:1152`) is
  /// deliberate user feedback rather than a geometric result, which is
  /// also why [`Placing::trace`] does not simplify a two point chain.
  pub fn route_step(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    shove: &mut Option<Shove>,
    node: NodeId,
    at: Vec2,
  ) -> NodeId {
    let mut fail = false;
    let mut go_back = false;
    let mut iterations = 1usize;
    let mut index = 0usize;
    let mut node = node;

    while index < iterations && index < ROUTE_STEP_ITERATION_LIMIT {
      let previous_tail = self.tail.clone();
      let previous_head = self.head.clone();

      // :1134
      if !go_back && context.settings.follow_mouse() {
        self.reduce_tail(world, context, node, at);
      }

      go_back = false;

      match self.route_head(world, context, shove, node, at) {
        Some((new_head, new_tail, routed_node)) => {
          // :1170
          self.head = new_head;
          self.tail = new_tail;
          // The shove hands back the branch it wants the placer to stand
          // on; every other mode answers with the node it was given.
          node = routed_node;
        }
        None => {
          // :1146
          self.tail = previous_tail;
          self.head = previous_head;

          // :1152
          if self.tail.point_count() == 0 {
            let start = self.p_start();

            self.tail.chain_mut().append(start);
            self.tail.chain_mut().append_allow_duplicate(start);
          }

          fail = true;
        }
      }

      // :1165
      if fail {
        break;
      }

      // :1173
      if self.handle_self_intersections() {
        iterations += 1;
        go_back = true;
      }

      // :1184
      if !go_back && self.handle_pullback() {
        iterations += 1;
        self.head.clear();
        go_back = true;
      }

      index += 1;
    }

    // :1198
    if !fail
      && context.settings.follow_mouse()
      && !self.optimize_tail_head_transition(world, context, node)
    {
      self.merge_head();
    }

    // :1216
    self.last_p_end = Some(at);

    node
  }

  /// Route to the cursor and say whether the head got there.
  ///
  /// Port of `route`, `pcbnew/router/pns_line_placer.cpp:1222`. Its
  /// docstring promises repeated steps due to mouse smoothing; the
  /// implementation calls [`Placing::route_step`] exactly once and this
  /// does the same.
  ///
  /// Answers whether the head reached the cursor **and** the node the
  /// placer should route against from here on, which shove mode changes;
  /// see [`Placing::route_head`].
  pub fn route(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    shove: &mut Option<Shove>,
    node: NodeId,
    at: Vec2,
  ) -> (bool, NodeId) {
    let node = self.route_step(world, context, shove, node, at);

    if self.head.point_count() == 0 {
      return (false, node);
    }

    (self.head.last_point() == Some(at), node)
  }
}

/// Put the via a trace ends with into a node.
///
/// The `Clone( pl.Via() ); newVia->ResetUid(); m_lastNode->Add(...)` of
/// `pcbnew/router/pns_line_placer.cpp:1621` and `:1704`. `ResetUid`
/// becomes a fresh uid from the world: the preview via's uid belongs to
/// the item the placer was dragging, and the stored one is a new object
/// whose uid orders it against the rest of the node (`DESIGN.md`
/// section 8).
///
/// `None` when the trace ends with no via, and when the via it ends with
/// already lives in a node, which the placer's head never produces.
fn store_trace_via(
  world: &mut World,
  node: NodeId,
  trace: &Line,
) -> Option<ItemId> {
  let LineVia::Owned(item) = trace.via()? else {
    return None;
  };
  let mut stored = item.clone();
  let uid = world.next_uid();

  stored.set_uid(uid);

  Some(world.add_via(node, stored))
}

// ---------------------------------------------------------------------
// splitHeadTail
// ---------------------------------------------------------------------

/// Rebuild the head and tail boundary after a walk produced a whole line.
///
/// Port of `splitHeadTail`,
/// `pcbnew/router/pns_line_placer.cpp:860`. The split point is the first
/// point of the old tail that the new line does not carry, which is as
/// close as possible to the old head's first point without being inside
/// an obstacle hull; the comment at `:775` explains why that matters.
///
/// # Decision on note 03 section 9.6 item 4
///
/// KiCad's version always returns true (`:917`), so its three callers
/// guard on a condition that can never fail (`:619`, `:639`, `:789`, and
/// `:1000` in the shove path). The port returns the pair directly, which
/// removes the unreachable branches rather than transcribing them. That
/// is a change of shape, not of behaviour.
///
/// It is a free function rather than a method because it reads nothing
/// off the placer; KiCad's is a member only by habit.
pub fn split_head_tail(new_line: &Line, old_tail: &Line) -> (Line, Line) {
  // :863. Both start as copies of the old tail, so they inherit its
  // width, layers and net.
  let mut new_tail = old_tail.clone();
  let mut new_head = old_tail.clone();
  let mut line = new_line.clone();

  new_tail.remove_via();
  new_head.clear();

  let point_count = line.point_count();

  // :874
  if point_count > 1 && old_tail.point_count() > 1 {
    let tail_last = old_tail.last_point().unwrap_or_default();

    // :876
    if line.shape().point_on_edge(tail_last, 0) {
      line.chain_mut().split(tail_last);
    }

    // :881, the first old tail point the new line does not carry.
    let mut index = old_tail.point_count();
    let mut found = false;

    for candidate in 0..old_tail.point_count() {
      if line.shape().find(old_tail.point(candidate), 0).is_none() {
        index = candidate;
        found = true;
        break;
      }
    }

    // :890
    if !found {
      index -= 1;
    }

    // :894
    if index >= line.point_count() {
      index = line.point_count() - 1;
    }

    // :899
    if index == 0 {
      new_tail.chain_mut().clear();
    } else if let Ok(chain) = line.shape().slice(0, index) {
      new_tail.set_shape(chain);
    }

    if let Ok(chain) = line.shape().slice(index, line.point_count() - 1) {
      new_head.set_shape(chain);
    }
  } else {
    // :908. The head becomes the new line whole, attributes included.
    new_tail.chain_mut().clear();
    new_head = line;
  }

  (new_head, new_tail)
}

// ---------------------------------------------------------------------
// LinePlacer
// ---------------------------------------------------------------------

/// The single track placement algorithm.
///
/// Port of `PNS::LINE_PLACER` (`pcbnew/router/pns_line_placer.h:113`).
/// See the module documentation for the shape of the state and for what
/// is left to later milestones.
///
/// The methods below are the port of `PLACEMENT_ALGO`
/// (`pcbnew/router/pns_placement_algo.h:44`) as a plain `impl`; note 03
/// section 9.2 says the abstraction over the five placers belongs in the
/// session facade, as an enum, and not in a trait here.
#[derive(Clone, Debug)]
pub struct LinePlacer {
  /// Where the placer is in its lifecycle.
  state: PlacerState,
  /// The node every placement branches from. KiCad reaches it as
  /// `Router()->GetWorld()` (`pcbnew/router/pns_line_placer.cpp:1461`).
  root_node: NodeId,
  /// The branch the placement runs on. Port of `m_world` (`:384`), read
  /// by the collision gate of `FixRoute` (`:1589`).
  world_node: NodeId,
  /// The node the algorithms route against. Port of `m_currentNode`
  /// (`:392`).
  current_node: NodeId,
  /// The per move scratch branch that carries the removed loops and the
  /// split end item. Port of `m_lastNode` (`:393`).
  last_node: Option<NodeId>,
  /// The geometry a placement is started with. Port of `m_sizes`
  /// (`:396`).
  sizes: Sizes,
  /// The direction a placement starts in before one is running.
  ///
  /// KiCad's constructor seeds `m_initial_direction` with north
  /// (`pcbnew/router/pns_line_placer.cpp:46`) and `Start` overwrites it
  /// from [`RoutingSettings::initial_direction`] (`:1393`). The seed is
  /// taken from the settings here, which is the same value in the only
  /// case anything reads it.
  initial_direction: Direction45,
  /// The last rat line [`LinePlacer::update_leading_ratline`] computed.
  ///
  /// KiCad has no field for this: it hands the chain straight to
  /// `ROUTER_IFACE::DisplayRatline`
  /// (`pcbnew/router/pns_line_placer.cpp:2028`) and forgets it. The
  /// engine draws nothing here, so the answer is kept for the facade to
  /// read through [`LinePlacer::leading_rat_line`].
  leading_rat_line: Option<LineChain>,
  /// The shove engine shove mode routes through. Port of `m_shove`
  /// (`pcbnew/router/pns_line_placer.h:344`), built over a branch of the
  /// placement node by [`LinePlacer::start`]
  /// (`pcbnew/router/pns_line_placer.cpp:1478`).
  ///
  /// [`None`] before the first start, where KiCad's unique pointer is
  /// null for the same window. Every springback call site guards on it,
  /// because a host may call [`LinePlacer::fix_route`] in a mode that
  /// never built one.
  shove: Option<Shove>,
  /// Whether the last event ran in [`RouterMode::Shove`].
  ///
  /// KiCad reads `Settings().Mode()` wherever it needs this, including
  /// inside `CommitPlacement` (`pcbnew/router/pns_line_placer.cpp:1812`).
  /// [`LinePlacer::commit_placement`] takes no settings, so the mode of
  /// the last [`LinePlacer::start`], [`LinePlacer::move_to`] or
  /// [`LinePlacer::fix_route`] is remembered instead. A host that changed
  /// the mode between its last event and the commit would see KiCad's
  /// answer and this one differ; nothing else observes the mode in that
  /// window.
  shove_mode: bool,
}

impl LinePlacer {
  /// An idle placer over one node.
  ///
  /// Port of the constructor,
  /// `pcbnew/router/pns_line_placer.cpp:43`. `node` is the world the
  /// placement will branch from, KiCad's `Router()->GetWorld()`, and
  /// `sizes` is what `ROUTER::StartRouting` pushes in through
  /// `UpdateSizes` before `Start` (`pcbnew/router/pns_router.cpp:467`).
  ///
  /// The layer starts at zero, as KiCad's `m_currentLayer` does (`:56`);
  /// a host sets it with [`LinePlacer::set_layer`] before starting, which
  /// is the order `ROUTER::StartRouting` uses (`:468`).
  ///
  /// # Panics
  ///
  /// When `node` is not a live node of `world`.
  pub fn new(
    world: &World,
    node: NodeId,
    settings: &RoutingSettings,
    sizes: Sizes,
  ) -> Self {
    assert!(
      world.node(node).is_some(),
      "a placer needs a live node to branch from"
    );

    Self {
      state: PlacerState::Idle { layer: 0 },
      root_node: node,
      world_node: node,
      current_node: node,
      last_node: None,
      sizes,
      initial_direction: settings.initial_direction(),
      leading_rat_line: None,
      shove: None,
      shove_mode: settings.mode == RouterMode::Shove,
    }
  }

  // -----------------------------------------------------------------
  // Accessors
  // -----------------------------------------------------------------

  /// The lifecycle state.
  pub const fn state(&self) -> &PlacerState {
    &self.state
  }

  /// The volatile part of the track. Port of `Head()`,
  /// `pcbnew/router/pns_line_placer.h:162`.
  pub fn head(&self) -> Option<&Line> {
    self.state.placing().map(|placing| &placing.head)
  }

  /// The settled part of the track. Port of `Tail()`,
  /// `pcbnew/router/pns_line_placer.h:168`.
  pub fn tail(&self) -> Option<&Line> {
    self.state.placing().map(|placing| &placing.tail)
  }

  /// The complete routed line. Port of `Trace()`,
  /// `pcbnew/router/pns_line_placer.cpp:1233`.
  pub fn trace(&self) -> Option<Line> {
    self.state.placing().map(Placing::trace)
  }

  /// The rat line from the end of the trace to the nearest thing on the
  /// routed net it has not reached yet.
  ///
  /// What KiCad passes to `ROUTER_IFACE::DisplayRatline`
  /// (`pcbnew/router/pns_line_placer.cpp:2028`), recomputed by every
  /// [`LinePlacer::move_to`] and [`None`] before the first one. KiCad has
  /// no accessor because it pushes the chain at the host instead.
  ///
  /// Two answers a facade has to tell apart. [`None`] means there is
  /// nothing left to reach, or the net has no net code
  /// (`pcbnew/router/pns_topology.cpp:123`). A chain of a **single**
  /// point means the trace's end already touches its target, which is the
  /// degenerate two identical points case
  /// [`crate::topology::leading_rat_line`] documents; neither should be
  /// drawn.
  pub fn leading_rat_line(&self) -> Option<&LineChain> {
    self.leading_rat_line.as_ref()
  }

  /// Every routed line, which is one for a single track placer.
  ///
  /// Port of `Traces()`,
  /// `pcbnew/router/pns_line_placer.cpp:1255`.
  ///
  /// # Deviation
  ///
  /// KiCad caches the trace in `m_currentTrace` and hands out an
  /// `ITEM_SET` pointing into that field, so the returned set is
  /// invalidated by the next call and `FlipPosture` and `UpdateSizes`
  /// read the cache behind the caller's back. This returns owned lines
  /// and no cache; see [`LinePlacer::flip_posture`] for the one place
  /// that changes an answer.
  pub fn traces(&self) -> Vec<Line> {
    self.trace().into_iter().collect()
  }

  /// Where the current leg starts. Port of `CurrentStart()`,
  /// `pcbnew/router/pns_line_placer.h:183`.
  ///
  /// `None` when nothing is being placed, where KiCad answers with a
  /// stale value; its host drops the placer at the end of a session
  /// (`pcbnew/router/pns_router.cpp:441`) and never asks.
  pub fn current_start(&self) -> Option<Vec2> {
    self.state.placing().map(|placing| placing.current_start)
  }

  /// Where the head ends, which is not the cursor when something is in
  /// the way. Port of `CurrentEnd()`,
  /// `pcbnew/router/pns_line_placer.h:192`.
  ///
  /// A trace that is nothing but a via has no points, so
  /// [`LinePlacer::move_to`] leaves this at `Placing::p_start`
  /// (`pcbnew/router/pns_line_placer.cpp:1529`), which is where that via
  /// sits.
  pub fn current_end(&self) -> Option<Vec2> {
    self.state.placing().map(|placing| placing.current_end)
  }

  /// The net being routed. Port of `CurrentNets()`,
  /// `pcbnew/router/pns_line_placer.h:200`, which wraps the one net in a
  /// vector.
  pub fn current_net(&self) -> Option<NetId> {
    self.state.placing().and_then(|placing| placing.net)
  }

  /// The layer being routed on. Port of `CurrentLayer()`,
  /// `pcbnew/router/pns_line_placer.h:208`.
  ///
  /// `None` after a terminal fix, for the reason
  /// [`LinePlacer::current_start`] gives.
  pub fn current_layer(&self) -> Option<i32> {
    match &self.state {
      PlacerState::Idle { layer } => Some(*layer),
      PlacerState::Placing(placing) => Some(placing.layer),
      PlacerState::Finished { .. } => None,
    }
  }

  /// The most recent world state.
  ///
  /// Port of `CurrentNode( bool aLoopsRemoved )`,
  /// `pcbnew/router/pns_line_placer.cpp:1278`: the scratch branch when
  /// the caller wants the loops removed and there is one, the routing
  /// node otherwise.
  ///
  /// KiCad answers with a null pointer after [`LinePlacer::commit_placement`];
  /// this answers with the node the placer was built on, since the field
  /// is not optional (`DESIGN.md` section 6.4).
  pub fn current_node(&self, loops_removed: bool) -> NodeId {
    if loops_removed && let Some(last) = self.last_node {
      return last;
    }

    self.current_node
  }

  /// The scratch branch of the last move, when there is one.
  pub const fn last_node(&self) -> Option<NodeId> {
    self.last_node
  }

  /// Whether a via is pending. Port of `IsPlacingVia()`,
  /// `pcbnew/router/pns_line_placer.h:233`.
  ///
  /// "Pending" and not "present": the flag says the next fix will place
  /// one, and whether the head actually carries one right now is
  /// `head().is_some_and(Line::ends_with_via)`, which the pushout can
  /// refuse. An idle or finished placer answers false, where KiCad
  /// answers with a stale flag.
  pub fn is_placing_via(&self) -> bool {
    self
      .state
      .placing()
      .is_some_and(|placing| placing.placing_via)
  }

  /// Whether anything has reached a node.
  ///
  /// Port of `HasPlacedAnything`,
  /// `pcbnew/router/pns_line_placer.cpp:1804`, which is
  /// `m_placementCorrect || m_fixedTail.StageCount() > 1`. `Start` pushes
  /// one stage, so the second half means "at least one fix happened".
  ///
  /// # Deviation
  ///
  /// Note 03 section 9.1 suggests collapsing both halves into one
  /// computed answer. They are not the same predicate once
  /// [`LinePlacer::undo_last_segment`] exists: an undo pops a stage but
  /// deliberately leaves `Placing::placement_correct` alone, so the field
  /// is kept and the expression stays KiCad's.
  pub fn has_placed_anything(&self) -> bool {
    match &self.state {
      PlacerState::Idle { .. } => false,
      PlacerState::Placing(placing) => {
        placing.placement_correct || placing.fixed_tail.stage_count() > 1
      }
      PlacerState::Finished { placed_anything } => *placed_anything,
    }
  }

  // -----------------------------------------------------------------
  // Small commands
  // -----------------------------------------------------------------

  /// Force the head to a single 90 or 45 degree segment.
  ///
  /// Port of `SetOrthoMode`,
  /// `pcbnew/router/pns_line_placer.cpp:2032`, which sets the flag and
  /// nothing else; the effect is entirely inside
  /// [`Placing::build_initial_line`].
  pub fn set_ortho_mode(&mut self, ortho: bool) {
    if let Some(placing) = self.state.placing_mut() {
      placing.ortho = ortho;
    }
  }

  /// Toggle the posture of the head between straight first and diagonal
  /// first.
  ///
  /// Port of `FlipPosture`,
  /// `pcbnew/router/pns_line_placer.cpp:1262`. The current trace's own
  /// first direction is copied into the posture solver before the flip
  /// (`:1269`, with the comment naming issue 12369), because the placer
  /// may have rerouted since the solver last spoke and a flip against a
  /// stale direction turns the wrong way.
  ///
  /// # Deviation
  ///
  /// KiCad reads `m_currentTrace`, the cache `Traces()` writes, so the
  /// resync uses whatever the last `Traces()` call saw. The trace is
  /// recomputed here instead. Its host calls `Traces()` after every move
  /// (`pcbnew/router/pns_router.cpp:494`), so the two agree except when a
  /// flip arrives before the first trace was ever asked for, where
  /// KiCad's cache is empty and skips the resync.
  pub fn flip_posture(&mut self) {
    let Some(placing) = self.state.placing_mut() else {
      return;
    };
    let trace = placing.trace();

    // :1267
    if !placing.posture.is_manually_forced() && trace.segment_count() > 0 {
      let first = Direction45::from_seg(&trace.segment(0), false);

      placing
        .posture
        .set_default_directions(first, Direction45::default());
    }

    placing.posture.flip_posture();
  }

  /// Enable or disable a via at the end of the trace.
  ///
  /// Port of `ToggleVia`, `pcbnew/router/pns_line_placer.cpp:84`, which
  /// sets the flag and, when disabling, drops the via the head is
  /// carrying. It never builds one: the via is materialised inside
  /// [`Placing::build_initial_line`] (`:2105`) and
  /// [`Placing::rh_walk_only`] (`:792`) on the next move, so the caller
  /// has to move before the change is visible in the preview. It touches
  /// nothing else, the posture solver included.
  ///
  /// # Deviation
  ///
  /// KiCad always answers true and writes `m_placingVia` even on an idle
  /// placer. The flag lives inside [`Placing`] here, so there is nowhere
  /// to record it before a placement starts; an idle or finished placer
  /// answers false instead, which tells a host the request was not
  /// honoured. `ROUTER::ToggleViaPlacement`
  /// (`pcbnew/router/pns_router.cpp:1019`) only calls it while routing, so
  /// the case does not arise there.
  pub fn toggle_via(&mut self, enabled: bool) -> bool {
    let Some(placing) = self.state.placing_mut() else {
      return false;
    };

    // :86
    placing.placing_via = enabled;

    // :88
    if !enabled {
      placing.head.remove_via();
    }

    true
  }

  /// Change the routing layer.
  ///
  /// Port of `SetLayer`,
  /// `pcbnew/router/pns_line_placer.cpp:1347`, which has three branches:
  /// an idle placer just records the layer; a chained placement refuses,
  /// which is the whole point of `Placing::chained`; otherwise the
  /// layer may change only when there is no start item, or the start item
  /// is a via or a solid that reaches the requested layer, and the live
  /// state is reset and the preview regenerated on the new layer.
  ///
  /// There is no via handling in this method, and there is none in
  /// KiCad's either. A layer change mid placement is legal exactly while
  /// `Placing::chained` is false, and the only thing that clears that
  /// flag after a fix is having ended the fixed leg with a via
  /// (`:1725`), so "switch layers" and "leave a via behind" are one
  /// decision made at the fix and not two. A host that wants KiCad's key
  /// binding drives [`LinePlacer::toggle_via`], then
  /// [`LinePlacer::fix_route`], then this
  /// (`pcbnew/router/pns_router.cpp:1010`).
  ///
  /// The layer pair of [`Sizes`] does not gate the change either: it only
  /// decides the span of the via a fix places, through
  /// [`Sizes::via_layer_range`].
  ///
  /// A finished placer refuses, where KiCad's idle flag lets it record a
  /// layer for a next placement on the same object; its router builds a
  /// fresh placer per session instead
  /// (`pcbnew/router/pns_router.cpp:441`).
  pub fn set_layer(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    layer: i32,
  ) -> bool {
    let current_end = {
      let placing = match &mut self.state {
        // :1349
        PlacerState::Idle { layer: current } => {
          *current = layer;
          return true;
        }
        PlacerState::Finished { .. } => return false,
        PlacerState::Placing(placing) => placing.as_mut(),
      };

      // :1354
      if placing.chained {
        return false;
      }

      // :1358
      let allowed = match placing.start_item {
        None => true,
        Some(id) => world.item(id).is_some_and(|item| {
          item.of_kind(Kind::VIA | Kind::SOLID) && item.layers().contains(layer)
        }),
      };

      if !allowed {
        return false;
      }

      // :1362
      placing.layer = layer;
      placing.direction = placing.initial_direction;
      placing.posture.clear();
      placing.head.chain_mut().clear();
      placing.tail.chain_mut().clear();
      placing.head.remove_via();
      placing.tail.remove_via();
      placing.head.set_layer(layer);
      placing.tail.set_layer(layer);

      placing.current_end
    };

    // :1372
    self.move_to(world, context, current_end, None);

    true
  }

  // -----------------------------------------------------------------
  // Splitting an item under the cursor
  // -----------------------------------------------------------------

  /// Split a track at a point so that a joint exists there.
  ///
  /// Port of `SplitAdjacentSegments`,
  /// `pcbnew/router/pns_line_placer.cpp:1287`, the "start in the middle
  /// of a track" mechanism: the segment is replaced by two halves meeting
  /// at `at`, both added with redundancy allowed. It refuses when a joint
  /// with at least one link already exists there, which is the "the click
  /// landed exactly on an existing endpoint" case.
  ///
  /// `SplitAdjacentArcs` (`:1315`) is the same routine over
  /// `SHAPE_ARC::ConstructFromStartEndCenter`; there is no arc body in
  /// this crate yet (`DESIGN.md` section 3), so it arrives with the arcs.
  ///
  /// Returns whether the split happened.
  pub fn split_adjacent_segments(
    world: &mut World,
    node: NodeId,
    item: Option<ItemId>,
    at: Vec2,
  ) -> bool {
    // :1289
    let Some(id) = item else {
      return false;
    };

    let Some(stored) = world.item(id) else {
      return false;
    };

    // :1292
    if !stored.of_kind(Kind::SEGMENT) {
      return false;
    }

    let ItemBody::Segment(body) = stored.body() else {
      return false;
    };

    let seg = body.seg();
    let width = body.width();
    let layers = stored.layers();
    let net = stored.net();
    let source = stored.source();
    let flashed = stored.flashed_layers();

    // :1295
    if let Some(joint) = world.find_joint(node, at, layers.start(), net)
      && world
        .joint(joint)
        .is_some_and(|joint| joint.link_count(world.items(), Kind::ANY) >= 1)
    {
      return false;
    }

    // :1302
    let mut halves = [
      world
        .make_item(ItemBody::Segment(Segment::new(Seg::new(seg.a, at), width))),
      world
        .make_item(ItemBody::Segment(Segment::new(Seg::new(at, seg.b), width))),
    ];

    for half in &mut halves {
      half.set_layers(layers);
      half.set_flashed_layers(flashed);
      half.set_net(net);
      half.set_source(source);
    }

    let [first, second] = halves;

    // :1307
    world.remove(node, id);
    world.add_segment(node, first, true);
    world.add_segment(node, second, true);

    true
  }

  // -----------------------------------------------------------------
  // Start
  // -----------------------------------------------------------------

  /// Begin placing a track.
  ///
  /// Port of `Start`,
  /// `pcbnew/router/pns_line_placer.cpp:1380`, together with
  /// `initPlacement` (`:1442`), which it is the only caller of. The world
  /// is branched, a track under the cursor is split so that the new trace
  /// has a joint to attach to, and the posture solver is seeded.
  ///
  /// The net is the start item's, which may itself be `None` for a non
  /// conductive start item, and for a track that starts on nothing it is
  /// [`RuleResolver::orphaned_net`] (`:1386`). That net is what makes the
  /// head of a free space route and its own already fixed tail "same net"
  /// (`pcbnew/router/pns_item.cpp:188`), so a shove can push an obstacle
  /// there instead of the placer falling back to the walkaround, while a
  /// null net would collide with everything.
  ///
  /// # Decision on note 03 section 9.6 item 2
  ///
  /// KiCad computes two posture seeds it then does not use: `initialDir`
  /// from the start pad's orientation (`:1415`) and `lastSegDir` from the
  /// segment the click landed on (`:1408`). Neither reaches
  /// `SetDefaultDirections`, which is passed `m_initial_direction` and a
  /// hard coded undefined direction (`:1426`); `initialDir` only ever
  /// reaches a debug message. The observable behaviour is reproduced and
  /// the two dead computations are not ported, because computing a value
  /// that is thrown away invites a later reader to wire it up and change
  /// the routing. Wiring them up is a behaviour change that belongs in a
  /// fixture backed decision of its own, not in a port.
  ///
  /// The first fixed tail stage (`:1436`) is pushed at the end, which is
  /// what makes `HasPlacedAnything`'s `StageCount() > 1` mean "a fix
  /// happened" and what gives [`LinePlacer::undo_last_segment`] a floor
  /// to stop at.
  ///
  /// The shove engine is built here over a **branch** of the placement
  /// node (`:1478`), not over the node itself, so that everything it
  /// pushes lives above the world the placer stands on and a springback
  /// rewind can throw it all away.
  ///
  /// Returns false when a placement is already running, where KiCad's
  /// `Start` always answers true; its router builds a fresh placer per
  /// session (`pcbnew/router/pns_router.cpp:441`), so the case cannot
  /// arise there. The gate that refuses an unroutable start point is
  /// `ROUTER::isStartingPointRoutable`, not the placer.
  pub fn start(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    start_item: Option<ItemId>,
  ) -> bool {
    let PlacerState::Idle { layer } = self.state else {
      return false;
    };

    // :1386
    let net = match start_item {
      Some(id) => world.item(id).and_then(Item::net),
      None => Some(context.resolver.orphaned_net()),
    };

    // :1393
    self.initial_direction = context.settings.initial_direction();

    // initPlacement, :1461 to :1476.
    world.kill_children(self.root_node);

    let branch = world.branch(self.root_node);

    Self::split_adjacent_segments(world, branch, start_item, at);

    self.world_node = branch;
    self.current_node = branch;
    self.last_node = None;

    // :1446 to :1455
    let mut head = Line::new();
    let mut tail = Line::new();

    for line in [&mut head, &mut tail] {
      line.set_net(net);
      line.set_layer(layer);
      line.set_width(self.sizes.track_width);
    }

    let mut placing = Placing {
      head,
      tail,
      direction: self.initial_direction,
      initial_direction: self.initial_direction,
      last_p_end: None,
      current_start: at,
      current_end: at,
      fix_start: at,
      start_item,
      end_item: None,
      placing_via: false,
      ortho: false,
      chained: false,
      placement_correct: false,
      net,
      layer,
      sizes: self.sizes.clone(),
      posture: MouseTrailTracer::new(),
      fixed_tail: FixedTail::new(),
    };

    // :1423 to :1427, in KiCad's order.
    placing.posture.clear();
    placing.posture.add_trail_point(context, at);
    placing.posture.set_tolerance(placing.head.width());
    placing.posture.set_default_directions(
      placing.initial_direction,
      Direction45::default(),
    );
    placing
      .posture
      .set_mouse_disabled(!context.settings.auto_posture);

    // :1478. The shove stands on a **branch** of the placement node, so
    // that everything it pushes lives above the world the placer routes
    // against and a springback rewind can throw it all away.
    self.shove = Some(Shove::new(world.branch(branch)));
    self.shove_mode = context.settings.mode == RouterMode::Shove;

    // :1436. The stage records the placement node, not the shove's:
    // `undo_last_segment` asks the shove to rewind to it and takes the
    // false answer for the first stage, which is never on the stack.
    placing.fixed_tail.add_stage(
      placing.fix_start,
      placing.layer,
      placing.placing_via,
      placing.direction,
      branch,
    );

    self.state = PlacerState::Placing(Box::new(placing));

    true
  }

  // -----------------------------------------------------------------
  // Move
  // -----------------------------------------------------------------

  /// Move the end of the trace to a point.
  ///
  /// Port of `Move`,
  /// `pcbnew/router/pns_line_placer.cpp:1482`. The scratch branch is
  /// dropped and rebuilt, the head is rerouted, the end point is snapped
  /// onto a collinear target segment when the head reached the cursor,
  /// and the loops the new trace made redundant are removed from the
  /// scratch branch.
  ///
  /// The `eiDepth` guard (`:1537`) stops the split from writing into a
  /// node shallower than the one the end item lives in, that is, into a
  /// node the end item does not exist in.
  ///
  /// The zero length via fallback at `:1505` covers the user pressing the
  /// via key without moving the mouse: the lead vector is then zero, the
  /// pushout of [`Placing::build_initial_line`] cannot resolve anything,
  /// and the head would carry no via for the next fix to commit.
  ///
  /// `updateLeadingRatLine` (`:1549`) fills
  /// [`LinePlacer::leading_rat_line`] instead of drawing, and runs on the
  /// scratch branch this function has just rebuilt.
  ///
  /// Always answers true while a placement is running, as KiCad does; the
  /// "did the head reach the cursor" answer is internal and a host reads
  /// it indirectly through [`LinePlacer::current_end`].
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

    self.shove_mode = context.settings.mode == RouterMode::Shove;

    // :1487
    let end_item_depth = end_item
      .and_then(|id| world.home_of(id))
      .and_then(|node| world.depth(node));

    // :1490
    if let Some(last) = self.last_node.take() {
      world.drop_node(last);
    }

    let mut node = self.current_node;
    let net = self.current_net();
    let mut current;
    let mut split_point;
    let reaches_end;

    {
      let placing = self
        .state
        .placing_mut()
        .expect("the state was checked above");

      placing.end_item = end_item;

      // :1498. In shove mode the run hands back the branch the shove
      // wants the placer to stand on, which is KiCad's two assignments to
      // `m_currentNode` inside `rhShoveOnly` (note 03 section 9.3).
      let (reached, routed_node) =
        placing.route(world, context, &mut self.shove, node, at);

      reaches_end = reached;
      node = routed_node;

      // :1505
      if placing.placing_via
        && at == placing.p_start()
        && !placing.head.ends_with_via()
      {
        let fallback = placing.make_via(world, at);

        placing.head.append_via(fallback);
      }

      // :1512
      current = placing.trace();
      split_point = current.last_point().unwrap_or_else(|| placing.p_start());

      // :1516
      if reaches_end
        && let Some(id) = end_item
        && current.segment_count() > 0
        && let Some(item) = world.item(id)
        && let ItemBody::Segment(body) = item.body()
      {
        let last_seg = current.segment(current.segment_count() - 1);
        let target = body.seg();

        if last_seg.collinear(&target) && target.overlaps(&last_seg) {
          split_point = target.nearest_point_to_point(last_seg.a);

          let last_index = current.point_count() - 1;

          current.chain_mut().set_point(last_index, split_point);

          let head_last = placing.head.point_count() - 1;

          placing.head.chain_mut().set_point(head_last, split_point);
        }
      }

      // :1529
      placing.current_end = if current.point_count() == 0 {
        placing.p_start()
      } else {
        split_point
      };
    }

    // :1534
    let last = world.branch(node);

    self.current_node = node;
    self.last_node = Some(last);

    // :1537
    if reaches_end
      && let Some(depth) = end_item_depth
      && let Some(id) = end_item
      && world.depth(node).is_some_and(|own| own >= depth)
      && current.segment_count() > 0
    {
      // :1542
      if world.item(id).and_then(Item::net) == net {
        Self::split_adjacent_segments(world, last, Some(id), split_point);
      }

      // :1545
      if context.settings.remove_loops {
        Self::remove_loops(world, last, &mut current);
      }
    }

    // :1549
    self.update_leading_ratline(world, context.resolver);

    // :1550
    if let Some(placing) = self.state.placing_mut() {
      placing.posture.add_trail_point(context, at);
    }

    true
  }

  // -----------------------------------------------------------------
  // Fix
  // -----------------------------------------------------------------

  /// Whether a trace may be written into a node.
  ///
  /// The collision gate of `FixRoute`,
  /// `pcbnew/router/pns_line_placer.cpp:1587` to `:1599`. Collisions
  /// prevent a fix unless the user allowed rule violations, which is only
  /// possible in mark obstacles mode
  /// ([`RoutingSettings::allow_drc_violations`]). Collisions can happen
  /// in walkaround and shove modes too, for instance when the start of
  /// the trace is already too wide for where it starts.
  ///
  /// The shove relaxation at `:1596`, where only a solid counts as a
  /// blocker, is a documented workaround for the shove node reporting
  /// collisions against objects it shoved itself; it is reproduced so
  /// that milestone 4 inherits it.
  ///
  /// KiCad has no function of this name; the gate is inline there.
  fn check_obstacles(
    world: &World,
    context: &AlgoContext<'_>,
    node: NodeId,
    trace: &Line,
  ) -> bool {
    // :1587
    if context.settings.allow_drc_violations() {
      return true;
    }

    let Some(obstacle) = world.check_colliding_line(
      node,
      trace,
      context.resolver,
      &CollisionSearchOptions::default(),
    ) else {
      return true;
    };

    // :1596
    if context.settings.mode != RouterMode::Shove {
      return false;
    }

    !obstacle
      .item
      .and_then(|id| world.item(id))
      .is_some_and(|item| item.of_kind(Kind::SOLID))
  }

  /// Write the trace into the scratch branch.
  ///
  /// Port of `FixRoute`,
  /// `pcbnew/router/pns_line_placer.cpp:1555`, the only place geometry
  /// becomes items in a node.
  ///
  /// The "fix all segments" setting lives entirely in two expressions
  /// (`:1659` and `:1718`): with it off, and with no via and no real end,
  /// the last segment is left unfixed and the next leg starts at the
  /// point before it, so the final leg stays rubber banded under the
  /// cursor. With it on, everything is emitted and the next leg starts at
  /// the trace's end.
  ///
  /// A fix is a "real end" when the end item is on the routed net, or
  /// when the caller forces it; that is the answer returned, and a host
  /// uses it to decide whether the interactive loop is over.
  ///
  /// # The via
  ///
  /// A trace of no segments that ends with a via is the via only commit
  /// (`:1616` to `:1633`): the via is stored on its own and the placement
  /// ends. Otherwise the via, when there is one, is stored after the
  /// segments (`:1704`), and it changes three of the rules around it: the
  /// last segment is always emitted (`:1659`), the next leg starts at the
  /// trace's end rather than one point back (`:1717`), and
  /// `Placing::chained` stays false so the layer may still change
  /// (`:1725`).
  ///
  /// `TODO(arcs)`: the arc branches at `:1650` and `:1671`, together with
  /// the "rollback is broken for arcs" override that forces `fix_all` on
  /// (`:1652`).
  ///
  /// The collision gate runs against the shove's node in shove mode and
  /// against the placement branch otherwise (`:1589`), because in shove
  /// mode the world the user is looking at is the one the shove built.
  pub fn fix_route(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    end_item: Option<ItemId>,
    force_finish: bool,
  ) -> bool {
    let _ = at;

    if self.state.placing().is_none() {
      return false;
    }

    let world_node = self.world_node;
    let Some(last) = self.last_node else {
      // KiCad dereferences `m_lastNode` unchecked here; it is never null
      // in practice because `Move` rebuilds it before every fix.
      return false;
    };

    // :1557
    let fix_all = context.settings.fix_all_segments;
    let mut trace;
    let net;
    let layer;
    let placing_via;

    {
      let placing = self
        .state
        .placing_mut()
        .expect("the state was checked above");

      trace = placing.trace();

      // :1562, net adoption in mark obstacles mode: whichever side is
      // netless takes the other's net.
      if context.settings.mode == RouterMode::MarkObstacles
        && let Some(id) = end_item
      {
        let own_code =
          placing.net.map_or(0, |net| context.resolver.net_code(net));
        let other = world.item(id).and_then(Item::net);
        let other_code = other.map_or(0, |net| context.resolver.net_code(net));

        if own_code <= 0 {
          placing.net = other;
          trace.set_net(other);
        } else if other_code <= 0
          && let Some(item) = world.item_mut(id)
        {
          item.set_net(placing.net);
        }
      }

      net = placing.net;
      layer = placing.layer;
      placing_via = placing.placing_via;
    }

    self.shove_mode = context.settings.mode == RouterMode::Shove;

    // :1589
    let check_node = if context.settings.mode == RouterMode::Shove {
      self
        .shove
        .as_ref()
        .map_or(world_node, crate::shove::Shove::current_node)
    } else {
      world_node
    };

    // :1587
    if !Self::check_obstacles(world, context, check_node, &trace) {
      return false;
    }

    // :1603
    if trace.segment_count() == 0 {
      // :1605, a final simplification of whatever was stored last.
      let (added, _removed) = world.get_updated_items(last);

      if let Some(id) = added.last().copied()
        && world
          .item(id)
          .is_some_and(|item| item.of_kind(Kind::SEGMENT))
      {
        Self::simplify_new_line(world, context, last, id);
      }

      // :1617
      if !trace.ends_with_via() {
        return false;
      }

      // :1623
      store_trace_via(world, last, &trace);

      // :1626. The via only commit pins the world it wrote the via into,
      // so springback can never roll back past it.
      if let Some(shove) = self.shove.as_mut() {
        shove.add_locked_springback_node(last);
      }

      // :1629. KiCad nulls `m_currentNode` and keeps `m_lastNode`, so a
      // following `CommitPlacement` still folds the via into the board;
      // the node handles are not optional here, so only the state
      // changes.
      self.state = PlacerState::Finished {
        placed_anything: true,
      };

      return true;
    }

    // :1636
    let last_point = trace.last_point().unwrap_or(at);
    let pre_last_point = if trace.point_count() > 2 {
      trace.point(trace.point_count() - 2)
    } else {
      last_point
    };

    // :1642
    let mut real_end = end_item.is_some()
      && net.is_some()
      && net == end_item.and_then(|id| world.item(id).and_then(Item::net));

    if force_finish {
      real_end = true;
    }

    // :1650. There are no arcs yet, so the "rollback is broken for arcs"
    // override that forces `fix_all` on cannot fire; it comes back with
    // the arc body.

    // :1654
    let direction_segment = if !fix_all && trace.segment_count() > 1 {
      trace.segment(trace.segment_count() - 2)
    } else {
      trace.segment(trace.segment_count() - 1)
    };
    let last_direction = Direction45::from_seg(&direction_segment, false);

    // :1659. A pending via pins the last segment: the via sits at the
    // trace's end, so leaving that segment rubber banded would leave the
    // via hanging off nothing.
    let last_vertex = if real_end || placing_via || fix_all {
      trace.segment_count()
    } else {
      1.max(trace.segment_count() - 1)
    };

    // :1669
    let mut last_item = None;

    for index in 0..last_vertex {
      let mut item = world.make_item(ItemBody::Segment(Segment::new(
        trace.segment(index),
        trace.width(),
      )));

      item.set_net(net);
      item.set_layers_and_flash_all(LayerRange::single(layer));

      last_item = world.add_segment(last, item, false);
    }

    // :1704
    store_trace_via(world, last, &trace);

    // :1712
    if let Some(id) = last_item {
      Self::simplify_new_line(world, context, last, id);
    }

    if real_end {
      // :1750
      if let Some(shove) = self.shove.as_mut() {
        shove.add_locked_springback_node(last);
      }

      self.state = PlacerState::Finished {
        placed_anything: true,
      };

      return true;
    }

    // :1715, the intermediate click.
    let next_start = if placing_via || fix_all {
      last_point
    } else {
      pre_last_point
    };
    // :1720 records the node the placement stood on **before** this fix,
    // which is the node an undo has to come back to.
    let stage_node = self.current_node;
    let ends_with_via = trace.ends_with_via();
    let next_node = world.branch(last);

    self.current_node = last;
    self.last_node = Some(next_node);

    // :1737. Every fixed leg pins the world it was written into.
    if let Some(shove) = self.shove.as_mut() {
      shove.add_locked_springback_node(last);
    }

    let placing = self
      .state
      .placing_mut()
      .expect("the state was checked above");

    // :1717
    placing.set_initial_direction(last_direction);
    placing.current_start = next_start;

    // :1720. Every field is read before the resets below overwrite it.
    placing.fixed_tail.add_stage(
      placing.fix_start,
      placing.layer,
      placing.placing_via,
      placing.direction,
      stage_node,
    );

    placing.fix_start = next_start;
    placing.start_item = None;
    placing.placing_via = false;

    // :1725. Without a via the layer is pinned from here on; with one the
    // trace can carry on wherever the via reaches.
    placing.chained = !ends_with_via;

    placing.direction = placing.initial_direction;
    placing.head.clear();
    placing.tail.clear();

    // :1739. A fix that ended on a via gives the posture solver nothing
    // to continue from, because the next leg leaves the via in whatever
    // direction the user takes it.
    let last_seg_dir = if ends_with_via {
      Direction45::default()
    } else {
      last_direction
    };

    // :1741
    placing.posture.clear();
    placing.posture.set_tolerance(placing.head.width());
    placing.posture.add_trail_point(context, next_start);
    placing
      .posture
      .set_default_directions(last_seg_dir, last_seg_dir);

    placing.placement_correct = true;

    false
  }

  // -----------------------------------------------------------------
  // Undo
  // -----------------------------------------------------------------

  /// Roll the placement back to the previous fix.
  ///
  /// Port of `UnfixRoute`,
  /// `pcbnew/router/pns_line_placer.cpp:1759`. One stage comes off the
  /// fixed tail and the whole live state is restored from it: the start
  /// point, the layer, the routing direction, the via flag and the node
  /// the placement stood on before the fix that stage recorded. Because
  /// [`FixedTail::pop_stage`] never removes the bottom stage, undoing
  /// past the first fix keeps answering with the point the placement
  /// started at instead of failing.
  ///
  /// The answer is the head's first point **before** the undo, which
  /// KiCad's host uses to put the cursor back where the undone leg began
  /// (note 03 section 3.11).
  ///
  /// # Deviation
  ///
  /// KiCad leaves the nodes above the restored one alive and lets the
  /// parent's destructor collect them. This crate owns its node tree
  /// explicitly (note 03 section 9.3), so the subtree above the stage's
  /// node is dropped before the fresh scratch branch is taken. Nothing
  /// observable changes: those nodes were already unreachable from the
  /// restored one.
  ///
  /// Two of KiCad's fields are deliberately **not** restored, because
  /// KiCad does not restore them either: `m_chainedPlacement` and
  /// `m_placementCorrect` keep whatever the last fix left them at. That
  /// means an undo back to the very start still reports
  /// [`LinePlacer::has_placed_anything`] as true, which is what lets a
  /// host tell "nothing was ever placed" from "everything was undone".
  ///
  /// # The springback rewind
  ///
  /// `:1789`. The stage's node is rewound to and then unlocked, which
  /// unlocks nothing: `RewindSpringbackTo` **erases** the frame it is
  /// asked to rewind to (`pcbnew/router/pns_shove.cpp:2175`), so the
  /// following `UnlockSpringbackNode` scans a stack the node has already
  /// left. Both calls are reproduced because the second is harmless and
  /// leaving it out would hide the wart. In shove mode the placer then
  /// takes the shove's node instead of the stage's and drops its subtree
  /// (`:1792`).
  ///
  /// `None` when nothing is being placed.
  pub fn undo_last_segment(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
  ) -> Option<Vec2> {
    let (stage, head_start) = {
      let placing = self.state.placing_mut()?;
      // :1764
      let stage = placing.fixed_tail.pop_stage()?;

      // :1767
      let head_start =
        (placing.head.point_count() > 0).then(|| placing.head.point(0));

      // :1770
      placing.head.chain_mut().clear();
      placing.tail.chain_mut().clear();
      placing.start_item = None;

      // :1773. `m_p_start` is derived here, and an empty tail makes it
      // `current_start`, so writing that one covers all three of KiCad's
      // assignments.
      placing.current_start = stage.pos;
      placing.fix_start = stage.pos;
      placing.direction = stage.direction;
      placing.placing_via = stage.placing_via;
      placing.layer = stage.layer;

      // :1780
      placing.head.set_layer(stage.layer);
      placing.tail.set_layer(stage.layer);
      placing.head.remove_via();
      placing.tail.remove_via();

      // :1785
      placing.posture.clear();
      placing
        .posture
        .set_default_directions(placing.initial_direction, stage.direction);
      placing.posture.add_trail_point(context, stage.pos);

      (stage, head_start)
    };

    // :1777
    self.current_node = stage.node;

    // :1789
    if let Some(shove) = self.shove.as_mut() {
      shove.rewind_springback_to(world, stage.node);
      shove.unlock_springback_node(stage.node);

      // :1792
      if context.settings.mode == RouterMode::Shove {
        self.current_node = shove.current_node();
      }
    }

    // :1794, and the deviation above.
    world.kill_children(self.current_node);

    // :1798
    self.last_node = Some(world.branch(self.current_node));

    head_start
  }

  /// The rat line from the end of the trace to what it still has to
  /// reach.
  ///
  /// Port of `updateLeadingRatLine`,
  /// `pcbnew/router/pns_line_placer.cpp:2021`, whose body is
  /// `TOPOLOGY( m_lastNode ).LeadingRatLine( &Trace(), ratLine )` followed
  /// by a call into the host's `DisplayRatline`.
  ///
  /// `LeadingRatLine` is [`crate::topology::leading_rat_line`] and the
  /// `DisplayRatline` call becomes a field: this crate draws nothing, so
  /// the chain is stored for the facade to hand on, and
  /// [`LinePlacer::leading_rat_line`] reads it back.
  ///
  /// Storing [`None`] when the query fails is what a host sees in KiCad
  /// too. KiCad only calls `DisplayRatline` on a true answer, but its
  /// preview items are freed on every frame
  /// (`PNS_KICAD_IFACE::EraseView`, `pcbnew/router/pns_kicad_iface.cpp`
  /// `:2456`), so a rat line that is not redrawn is a rat line that is
  /// gone.
  ///
  /// The query runs on `m_lastNode`, the scratch branch
  /// [`LinePlacer::move_to`] has just rebuilt, and not on the routing
  /// node, so the loops that move removed and the split end item are part
  /// of the connectivity it reasons about.
  fn update_leading_ratline(
    &mut self,
    world: &mut World,
    resolver: &dyn RuleResolver,
  ) {
    self.leading_rat_line = None;

    let Some(trace) = self.trace() else {
      return;
    };
    let Some(node) = self.last_node else {
      return;
    };

    self.leading_rat_line =
      topology::leading_rat_line(world, node, resolver, &trace);
  }

  /// Fold the placed geometry into the root.
  ///
  /// Port of `CommitPlacement`,
  /// `pcbnew/router/pns_line_placer.cpp:1810`, which hands the scratch
  /// branch to `ROUTER::CommitRouting` and then forgets both nodes. The
  /// commit itself is [`World::commit`], which also drops every branch of
  /// the root, so the placer's nodes are gone afterwards.
  ///
  /// A via needs nothing of its own here. Both places that store one,
  /// the via only commit and the append after the segments, put it in the
  /// scratch branch (`:1621`, `:1704`), and [`World::commit`] folds the
  /// branch whole, hole included.
  ///
  /// In shove mode the node that is committed is the shove's last locked
  /// springback frame (`:1814`), not the placer's scratch branch: every
  /// fix pins the world it wrote into, so the last locked frame is the
  /// world that holds every fixed leg together with everything the shove
  /// moved to make room for them.
  ///
  /// `AbortPlacement` (`pcbnew/router/pns_line_placer.cpp:2150`) is not
  /// ported: note 03 section 9.5 lists it among the routines with no
  /// caller in this revision, and its body is
  /// `m_world->KillChildren(); m_lastNode = nullptr;`, which a host can
  /// spell for itself.
  ///
  /// KiCad leaves both node pointers null; this leaves the placer
  /// pointing at the root, see [`LinePlacer::current_node`].
  pub fn commit_placement(&mut self, world: &mut World) -> bool {
    // :1812
    if self.shove_mode
      && let Some(shove) = self.shove.as_mut()
    {
      shove.rewind_to_last_locked_node();

      let node = shove.current_node();

      world.kill_children(node);
      self.last_node = Some(node);
    }

    if let Some(last) = self.last_node {
      world.commit(last);
    }

    self.last_node = None;
    self.current_node = self.root_node;
    self.world_node = self.root_node;

    true
  }

  // -----------------------------------------------------------------
  // Node hygiene
  // -----------------------------------------------------------------

  /// Delete the traces the new one made redundant.
  ///
  /// Port of `removeLoops`,
  /// `pcbnew/router/pns_line_placer.cpp:1828`, called from
  /// [`LinePlacer::move_to`] and not from the fix. The new trace is added
  /// to the scratch branch, every other line running between the same two
  /// joints is marked for erasure unless it is locked, and the new trace
  /// is taken out again.
  ///
  /// The locked track guard at `:1864` is what keeps a locked parallel
  /// route from being silently deleted as a redundant loop.
  fn remove_loops(world: &mut World, node: NodeId, latest: &mut Line) {
    // :1830
    if latest.segment_count() == 0 {
      return;
    }

    // :1833
    if Some(latest.point(0)) == latest.last_point() {
      return;
    }

    let mut to_erase: BTreeSet<ItemId> = BTreeSet::new();

    latest.clear_links();
    world.add_line(node, latest, true);

    let links: Vec<ItemId> = latest.links().to_vec();

    for link in links {
      // :1843
      let our_line = world.assemble_line(node, link, None, false, false, true);
      let Some(first) = our_line.shape().point_count().checked_sub(1) else {
        continue;
      };
      let ends = world.find_line_ends(
        node,
        our_line.point(0),
        our_line.point(first),
        our_line.layers(),
        our_line.net(),
      );

      let Some((mut start, mut end)) = ends else {
        continue;
      };

      // :1849
      if start == end
        && let Some(latest_last) = latest.last_point()
        && let Some(pair) = world.find_line_ends(
          node,
          latest.point(0),
          latest_last,
          latest.layers(),
          latest.net(),
        )
      {
        start = pair.0;
        end = pair.1;
      }

      // :1852
      for line in world.find_lines_between_joints(node, start, end) {
        if line.contains_link(link) || line.segment_count() == 0 {
          continue;
        }

        // :1864
        if line.has_locked_segments(world) {
          continue;
        }

        for owned in line.links() {
          to_erase.insert(*owned);
        }
      }
    }

    // :1887
    for id in to_erase {
      world.remove(node, id);
    }

    world.remove_line(node, latest);
  }

  /// Clean up and re optimize the line a fix just stored.
  ///
  /// Port of `simplifyNewLine`,
  /// `pcbnew/router/pns_line_placer.cpp:1894`, which runs in two phases.
  ///
  /// Phase one walks every segment the branch added and, for each of its
  /// joints that is not a line corner, removes a collinear stub of the
  /// same width whose far joint has a single link. The comment at `:1898`
  /// says why: such a segment blocks line assembly and the optimizer
  /// cannot clean it up.
  ///
  /// Phase two assembles the line, merges its collinear segments,
  /// simplifies the chain and stores the result when either step changed
  /// something.
  fn simplify_new_line(
    world: &mut World,
    context: &AlgoContext<'_>,
    node: NodeId,
    latest: ItemId,
  ) {
    // :1901
    let (added, _removed) = world.get_updated_items(node);
    let mut cleanup: BTreeSet<ItemId> = BTreeSet::new();

    // :1959
    for id in &added {
      let Some(item) = world.item(*id) else {
        continue;
      };

      if !item.of_kind(Kind::SEGMENT) || cleanup.contains(id) {
        continue;
      }

      let first = item.anchor(0);
      let second = item.anchor(1);
      let layer = item.layers().start();
      let net = item.net();
      let first_joint = world.find_joint(node, first, layer, net);
      let second_joint = world.find_joint(node, second, layer, net);

      process_stub_joint(world, node, first_joint, *id, &mut cleanup);
      process_stub_joint(world, node, second_joint, *id, &mut cleanup);
    }

    // :1971
    for id in cleanup {
      world.remove(node, id);
    }

    // :1976
    let mut original =
      world.assemble_line(node, latest, None, false, false, false);
    let mut candidate = original.clone();
    let optimized = Optimizer::optimize_line(
      world,
      context,
      node,
      &mut candidate,
      EffortFlags::MERGE_COLINEAR,
      Vec2::new(0, 0),
    );
    let mut simplified = candidate.shape().clone();

    simplified.simplify(0);

    // :1985
    if optimized || simplified.point_count() != candidate.point_count() {
      world.remove_line(node, &mut original);
      candidate.set_shape(simplified);
      candidate.clear_links();
      world.add_line(node, &mut candidate, false);
    }
  }
}

/// Queue the collinear stubs hanging off one joint for removal.
///
/// The `processJoint` lambda of `simplifyNewLine`,
/// `pcbnew/router/pns_line_placer.cpp:1906`. A joint that is a line
/// corner is left alone; otherwise every neighbouring track of the same
/// width and overlapping layers that contains, or is contained in, the
/// reference segment and whose far joint has exactly one link is queued.
/// The second case queues the reference segment itself and stops looking.
fn process_stub_joint(
  world: &World,
  node: NodeId,
  joint: Option<JointRef>,
  item_id: ItemId,
  cleanup: &mut BTreeSet<ItemId>,
) {
  // :1909
  let Some(joint_ref) = joint else {
    return;
  };
  let Some(joint) = world.joint(joint_ref) else {
    return;
  };

  if joint.is_line_corner(world.items(), false) {
    return;
  }

  let Some(item) = world.item(item_id) else {
    return;
  };
  let ItemBody::Segment(body) = item.body() else {
    return;
  };

  let reference = body.seg();
  let width = body.width();
  let layers = item.layers();
  let item_first = item.anchor(0);
  let item_second = item.anchor(1);
  let item_layer = layers.start();
  let item_net = item.net();
  let links: Vec<ItemId> = joint.links().to_vec();

  for neighbour in links {
    // :1918
    if neighbour == item_id {
      continue;
    }

    let Some(other) = world.item(neighbour) else {
      continue;
    };

    if !other.of_kind(Kind::SEGMENT | Kind::ARC)
      || !other.layers().overlaps(layers)
    {
      continue;
    }

    let ItemBody::Segment(other_body) = other.body() else {
      continue;
    };

    // :1925
    if other_body.width() != width {
      continue;
    }

    let test = other_body.seg();
    let other_first = other.anchor(0);
    let other_second = other.anchor(1);
    let other_layer = other.layers().start();
    let other_net = other.net();

    // :1933
    if reference.contains_segment(&test) {
      let near = world.find_joint(node, other_first, other_layer, other_net);
      let far = world.find_joint(node, other_second, other_layer, other_net);

      if stub_is_loose(world, joint_ref, near, far) {
        cleanup.insert(neighbour);
      }
    } else if test.contains_segment(&reference) {
      // :1944
      let near = world.find_joint(node, item_first, item_layer, item_net);
      let far = world.find_joint(node, item_second, item_layer, item_net);

      if stub_is_loose(world, joint_ref, near, far) {
        cleanup.insert(item_id);
        return;
      }
    }
  }
}

/// Whether one end of a segment is the joint under test and the other end
/// is a dead end.
///
/// The `( nA == aJoint && nB->LinkCount() == 1 ) || ( nB == aJoint &&
/// nA->LinkCount() == 1 )` test of
/// `pcbnew/router/pns_line_placer.cpp:1938` and `:1949`.
fn stub_is_loose(
  world: &World,
  joint: JointRef,
  first: Option<JointRef>,
  second: Option<JointRef>,
) -> bool {
  let link_count = |end: Option<JointRef>| {
    end
      .and_then(|end| world.joint(end))
      .map_or(0, |end| end.link_count(world.items(), Kind::ANY))
  };

  (first == Some(joint) && link_count(second) == 1)
    || (second == Some(joint) && link_count(first) == 1)
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::shape::Shape;
  use crate::item::Solid;
  use crate::rules::FixedClearance;

  /// The track width every unit test builds lines with.
  const WIDTH: i32 = 100_000;

  /// A placing state over an empty world, for the pure helpers.
  fn placing_at(start: Vec2) -> Placing {
    let mut head = Line::new();
    let mut tail = Line::new();

    for line in [&mut head, &mut tail] {
      line.set_width(WIDTH);
      line.set_layer(0);
    }

    Placing {
      head,
      tail,
      direction: Direction45::default(),
      initial_direction: Direction45::default(),
      last_p_end: None,
      current_start: start,
      current_end: start,
      fix_start: start,
      start_item: None,
      end_item: None,
      placing_via: false,
      ortho: false,
      chained: false,
      placement_correct: false,
      net: None,
      layer: 0,
      sizes: Sizes::default(),
      posture: MouseTrailTracer::new(),
      fixed_tail: FixedTail::new(),
    }
  }

  /// A line of the given points with the test width.
  fn line_of(points: &[Vec2]) -> Line {
    let mut line = Line::new();

    line.set_width(WIDTH);
    line.set_layer(0);
    line.set_shape(LineChain::from_slice(points, false));

    line
  }

  /// Shorthand for a point.
  fn at(x: i32, y: i32) -> Vec2 {
    Vec2::new(x, y)
  }

  #[test]
  fn p_start_follows_the_tail_and_falls_back_to_the_start() {
    let mut placing = placing_at(at(0, 0));

    assert_eq!(placing.p_start(), at(0, 0));

    placing.tail = line_of(&[at(0, 0), at(100, 0)]);

    assert_eq!(placing.p_start(), at(100, 0));
  }

  #[test]
  fn split_head_tail_cuts_at_the_first_moved_tail_point() {
    // The walk moved the tail's second point, so the boundary lands on
    // index 1 and both sides come out of the walked line rather than out
    // of the old tail.
    let old_tail = line_of(&[at(0, 0), at(1000, 0), at(2000, 0)]);
    let walked = line_of(&[at(0, 0), at(1000, 1000), at(2000, 1000)]);
    let (head, tail) = split_head_tail(&walked, &old_tail);

    assert_eq!(tail.shape().points(), &[at(0, 0), at(1000, 1000)]);
    assert_eq!(head.shape().points(), &[at(1000, 1000), at(2000, 1000)]);
  }

  #[test]
  fn split_head_tail_leaves_a_one_point_head_when_only_the_end_moved() {
    // Only the tail's last point moved, so the boundary is its index and
    // the whole walked line becomes the tail; the head is the single
    // point they meet at. That is what keeps the head anchored where the
    // walk left it rather than inside an obstacle hull.
    let old_tail = line_of(&[at(0, 0), at(1000, 0), at(2000, 0)]);
    let walked = line_of(&[at(0, 0), at(1000, 0), at(2000, 1000)]);
    let (head, tail) = split_head_tail(&walked, &old_tail);

    assert_eq!(
      tail.shape().points(),
      &[at(0, 0), at(1000, 0), at(2000, 1000)]
    );
    assert_eq!(head.shape().points(), &[at(2000, 1000)]);
  }

  #[test]
  fn split_head_tail_keeps_the_whole_line_as_head_without_a_tail() {
    let old_tail = line_of(&[]);
    let walked = line_of(&[at(0, 0), at(1000, 1000)]);
    let (head, tail) = split_head_tail(&walked, &old_tail);

    assert_eq!(tail.point_count(), 0);
    assert_eq!(head.shape().points(), &[at(0, 0), at(1000, 1000)]);
  }

  #[test]
  fn split_head_tail_backs_up_one_point_when_the_tail_survived_whole() {
    // Every old tail point is still on the new line, so KiCad's `i--`
    // (`:891`) puts the boundary on the tail's last point.
    let old_tail = line_of(&[at(0, 0), at(1000, 0)]);
    let walked = line_of(&[at(0, 0), at(1000, 0), at(2000, 0)]);
    let (head, tail) = split_head_tail(&walked, &old_tail);

    assert_eq!(tail.shape().points(), &[at(0, 0), at(1000, 0)]);
    assert_eq!(head.shape().points(), &[at(1000, 0), at(2000, 0)]);
  }

  #[test]
  fn merge_head_refuses_a_head_of_fewer_than_three_shapes() {
    let mut placing = placing_at(at(0, 0));

    placing.head = line_of(&[at(0, 0), at(1000, 0), at(2000, 1000)]);

    assert!(!placing.merge_head());
    assert_eq!(placing.tail.point_count(), 0);
  }

  #[test]
  fn merge_head_moves_a_three_shape_head_into_the_tail() {
    let mut placing = placing_at(at(0, 0));

    placing.head =
      line_of(&[at(0, 0), at(1000, 0), at(2000, 1000), at(3000, 1000)]);

    assert!(placing.merge_head());
    assert_eq!(placing.head.point_count(), 0);
    assert_eq!(placing.tail.last_point(), Some(at(3000, 1000)));
    // The direction becomes that of the last tail segment, due east.
    assert_eq!(
      placing.direction,
      Direction45::from_seg(&Seg::new(at(2000, 1000), at(3000, 1000)), false)
    );
  }

  #[test]
  fn merge_head_refuses_an_acute_corner() {
    let mut placing = placing_at(at(0, 0));

    // East, then back north west, which is an acute corner.
    placing.head =
      line_of(&[at(0, 0), at(2000, 0), at(1000, -1000), at(1000, -2000)]);

    assert!(!placing.merge_head());
  }

  #[test]
  fn merge_head_refuses_a_head_that_does_not_continue_the_tail() {
    let mut placing = placing_at(at(0, 0));

    placing.tail = line_of(&[at(0, 0), at(1000, 0)]);
    placing.head = line_of(&[
      at(5000, 5000),
      at(6000, 5000),
      at(7000, 6000),
      at(8000, 6000),
    ]);

    assert!(!placing.merge_head());
  }

  #[test]
  fn handle_self_intersections_clears_a_tail_the_head_restarts() {
    let mut placing = placing_at(at(0, 0));

    placing.initial_direction =
      Direction45::from_seg(&Seg::new(at(0, 0), at(0, -1000)), false);
    placing.tail = line_of(&[at(0, 0), at(1000, 0)]);
    placing.head = line_of(&[at(0, 0), at(0, -1000)]);

    assert!(placing.handle_self_intersections());
    assert_eq!(placing.tail.point_count(), 0);
    assert_eq!(placing.direction, placing.initial_direction);
  }

  #[test]
  fn handle_self_intersections_ignores_the_normal_junction() {
    let mut placing = placing_at(at(0, 0));

    placing.tail = line_of(&[at(0, 0), at(1000, 0), at(2000, 0)]);
    placing.head = line_of(&[at(2000, 0), at(3000, 0)]);

    assert!(!placing.handle_self_intersections());
    assert_eq!(placing.tail.point_count(), 3);
  }

  #[test]
  fn handle_pullback_drops_the_last_tail_shape_on_an_acute_corner() {
    let mut placing = placing_at(at(0, 0));

    placing.tail = line_of(&[at(0, 0), at(1000, 0), at(2000, 0)]);
    // Straight back west, a half full angle is not enough; use a right
    // angle turn to the north instead, which is what pullback catches.
    placing.head = line_of(&[at(2000, 0), at(2000, -1000)]);

    assert!(placing.handle_pullback());
    assert_eq!(placing.tail.shape().points(), &[at(0, 0), at(1000, 0)]);
  }

  #[test]
  fn handle_pullback_leaves_an_obtuse_corner_alone() {
    let mut placing = placing_at(at(0, 0));

    placing.tail = line_of(&[at(0, 0), at(1000, 0), at(2000, 0)]);
    placing.head = line_of(&[at(2000, 0), at(3000, -1000)]);

    assert!(!placing.handle_pullback());
    assert_eq!(placing.tail.point_count(), 3);
  }

  #[test]
  fn handle_pullback_clears_a_one_point_tail() {
    let mut placing = placing_at(at(0, 0));

    placing.tail = line_of(&[at(0, 0)]);
    placing.head = line_of(&[at(0, 0), at(1000, 0)]);

    assert!(placing.handle_pullback());
    assert_eq!(placing.tail.point_count(), 0);
  }

  #[test]
  fn reduce_tail_replaces_the_last_segments_over_an_empty_world() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let mut placing = placing_at(at(0, 0));

    // A staircase east, which one straight run plus one diagonal can
    // replace from its very first corner.
    placing.tail =
      line_of(&[at(0, 0), at(1000, 0), at(2000, 1000), at(3000, 1000)]);
    placing.head = line_of(&[at(3000, 1000), at(4000, 2000)]);

    assert!(placing.reduce_tail(&world, &context, root, at(4000, 2000)));
    assert_eq!(placing.head.point_count(), 0);
    assert!(placing.tail.point_count() < 4);
  }

  #[test]
  fn reduce_tail_refuses_a_tail_of_one_segment() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let mut placing = placing_at(at(0, 0));

    placing.tail = line_of(&[at(0, 0), at(1000, 0)]);
    placing.head = line_of(&[at(1000, 0), at(2000, 0)]);

    assert!(!placing.reduce_tail(&world, &context, root, at(2000, 0)));
  }

  #[test]
  fn cursor_dist_minimum_clips_a_line_that_overshoots_the_cursor() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let placing = placing_at(at(0, 0));
    let chain = LineChain::from_slice(
      &[at(0, 0), at(1000, 0), at(2000, 0), at(3000, 0)],
      false,
    );

    let clipped = placing
      .cursor_dist_minimum(&world, &context, root, &chain, at(1000, 0), 1.0e9)
      .expect("an empty world clips at the point nearest the cursor");

    assert_eq!(clipped.last_point(), Some(at(1000, 0)));
  }

  #[test]
  fn cursor_dist_minimum_answers_nothing_for_an_empty_chain() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let placing = placing_at(at(0, 0));

    assert!(
      placing
        .cursor_dist_minimum(
          &world,
          &context,
          root,
          &LineChain::new(),
          at(0, 0),
          1.0e9,
        )
        .is_none()
    );
  }

  #[test]
  fn split_adjacent_segments_makes_a_joint_in_the_middle_of_a_track() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let body =
      ItemBody::Segment(Segment::new(Seg::new(at(0, 0), at(4000, 0)), WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(Some(NetId(1)));

    let id = world
      .add_segment(root, item, false)
      .expect("the track is neither degenerate nor redundant");
    let branch = world.branch(root);

    assert!(LinePlacer::split_adjacent_segments(
      &mut world,
      branch,
      Some(id),
      at(2000, 0)
    ));
    assert!(
      world
        .find_joint(branch, at(2000, 0), 0, Some(NetId(1)))
        .is_some()
    );

    // A second split at the same point finds the joint and refuses.
    assert!(!LinePlacer::split_adjacent_segments(
      &mut world,
      branch,
      Some(id),
      at(2000, 0)
    ));
  }

  #[test]
  fn a_fresh_placer_is_idle_and_places_nothing() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let settings = RoutingSettings::default();
    let placer = LinePlacer::new(&world, root, &settings, Sizes::default());

    assert!(matches!(placer.state(), PlacerState::Idle { layer: 0 }));
    assert!(!placer.has_placed_anything());
    assert!(!placer.is_placing_via());
    assert!(placer.head().is_none());
    assert!(placer.traces().is_empty());
  }

  #[test]
  fn undoing_restores_the_direction_and_the_via_flag_of_the_stage() {
    // Two legs at right angles over an empty world, so that the fixes
    // leave `direction` and `initial_direction` pointing somewhere other
    // than where `start` put them, and the undo has something to
    // restore.
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let mut placer = LinePlacer::new(&world, root, &settings, Sizes::default());
    let seeded = settings.initial_direction();

    assert!(placer.start(&mut world, &context, at(0, 0), None));

    // The first stage carries exactly what `start` set up.
    let placing = placer.state.placing().expect("a placement is running");

    assert_eq!(placing.fixed_tail.stage_count(), 1);
    assert_eq!(placing.direction, seeded);
    assert!(!placing.placing_via);

    // Leg one runs east, leg two south, and the second fix arms a via so
    // that the stage it pushes differs from the first one in every
    // field an undo restores. Neither leg runs the way the seed points,
    // which is what keeps the assertion below meaningful.
    assert!(placer.move_to(&mut world, &context, at(4_000_000, 0), None));
    assert!(!placer.fix_route(
      &mut world,
      &context,
      at(4_000_000, 0),
      None,
      false
    ));
    assert!(placer.toggle_via(true));
    assert!(placer.move_to(
      &mut world,
      &context,
      at(4_000_000, 4_000_000),
      None
    ));
    assert!(!placer.fix_route(
      &mut world,
      &context,
      at(4_000_000, 4_000_000),
      None,
      false
    ));

    let placing = placer.state.placing().expect("a placement is running");

    assert_eq!(placing.fixed_tail.stage_count(), 3);
    // Both fixes rewrote the initial direction from the leg they ended.
    assert_ne!(placing.initial_direction, seeded);

    // Three undos come back past both fixes to the stage `start` pushed,
    // whose contents are known exactly: two pops take the two fixes off
    // and the third hands out the bottom stage without removing it.
    for _ in 0..3 {
      placer.undo_last_segment(&mut world, &context);
    }

    let placing = placer.state.placing().expect("a placement is running");

    assert_eq!(placing.direction, seeded, "the direction was not restored");
    assert_eq!(placing.current_start, at(0, 0));
    assert_eq!(placing.fix_start, at(0, 0));
    assert_eq!(placing.layer, 0);
    assert!(!placing.placing_via, "the via flag was not restored");
    assert_eq!(placing.head.point_count(), 0);
    assert_eq!(placing.tail.point_count(), 0);
    // The bottom stage stays, so `has_placed_anything` still remembers
    // that something was fixed, which KiCad's undo does not clear either.
    assert_eq!(placing.fixed_tail.stage_count(), 1);
    assert!(placer.has_placed_anything());
  }

  #[test]
  fn the_stage_before_a_via_fix_remembers_the_armed_via() {
    // The stage pushed by a fix records the via flag as it was at the
    // fix, so undoing back over a via fix re arms the via.
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let mut placer = LinePlacer::new(&world, root, &settings, Sizes::default());

    assert!(placer.start(&mut world, &context, at(0, 0), None));
    assert!(placer.toggle_via(true));
    assert!(placer.move_to(&mut world, &context, at(4_000_000, 0), None));
    assert!(placer.head().is_some_and(Line::ends_with_via));
    assert!(!placer.fix_route(
      &mut world,
      &context,
      at(4_000_000, 0),
      None,
      false
    ));

    // The fix consumed the via, so the layer is free and the flag is
    // down.
    assert!(!placer.is_placing_via());
    assert!(placer.set_layer(&mut world, &context, 1));
    assert_eq!(placer.current_layer(), Some(1));

    // Undoing back over that fix restores the armed via and the layer it
    // was armed on. The answer is `None` because `set_layer` regenerated
    // the preview at the point the leg starts, which leaves no head.
    assert!(placer.undo_last_segment(&mut world, &context).is_none());
    assert!(placer.is_placing_via());
    assert_eq!(placer.current_layer(), Some(0));
    assert_eq!(placer.current_start(), Some(at(0, 0)));
  }

  #[test]
  fn a_pad_under_the_cursor_gives_the_trace_its_net() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let body =
      ItemBody::Solid(Solid::new(Shape::circle(at(0, 0), 200_000), at(0, 0)));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(Some(NetId(7)));

    let pad = world.add_solid(root, item, None);
    let mut placer = LinePlacer::new(&world, root, &settings, Sizes::default());

    assert!(placer.start(&mut world, &context, at(0, 0), Some(pad)));
    assert_eq!(placer.current_net(), Some(NetId(7)));
    assert_eq!(placer.current_start(), Some(at(0, 0)));
    assert_eq!(placer.current_layer(), Some(0));
  }

  #[test]
  fn a_start_on_nothing_takes_the_resolvers_orphaned_net() {
    // :1386. KiCad asks the host for a real handle here rather than
    // leaving the net null, so that the head of a free space route is
    // "same net" with the tail it has already fixed; the scenario that
    // depends on it is
    // `tests/placer.rs::a_route_started_in_free_space_shoves_past_its_own_fixed_tail`.
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let mut placer = LinePlacer::new(&world, root, &settings, Sizes::default());

    assert!(placer.start(&mut world, &context, at(0, 0), None));
    assert_eq!(placer.current_net(), Some(rules.orphaned_net()));
    assert!(
      rules.net_code(rules.orphaned_net()) <= 0,
      "the orphan has to read as `no net` to the topology code"
    );
  }
}
