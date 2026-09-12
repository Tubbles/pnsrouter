// SPDX-License-Identifier: GPL-3.0-or-later

//! The transient line: a run of segments seen as one polyline.
//!
//! Port of `PNS::LINE` (`pcbnew/router/pns_line.h:61`), which KiCad
//! documents as "a track on a PCB, connecting two non-trivial joints
//! (that is, vias, pads, junctions between multiple traces or two traces
//! different widths and combinations of these). PNS_LINEs are NOT stored
//! in the model (NODE). Instead, they are assembled on-the-fly"
//! (`pcbnew/router/pns_line.h:47`).
//!
//! Everything the router does to copper it does through a [`Line`]: the
//! placer drags one, the walkaround bends one around a hull, the shove
//! pushes one aside and the optimizer shortens one. [`World::add_line`]
//! and [`World::remove_line`] are the only two places where a line turns
//! back into stored [`crate::item::Item`]s.
//!
//! # A value type, not an item
//!
//! In C++ a `LINE` is an `ITEM` with `Kind() == LINE_T`, which is what
//! lets `NODE::CheckColliding` take one through the generic item path
//! (`pcbnew/router/pns_node.cpp:505`). Here it is a plain value that is
//! never in the arena, so it has no [`crate::item::ItemId`] and never
//! appears in a spatial index. [`crate::item::Kind::LINE`] still exists,
//! because it is a bit of the kind masks the collision queries take, but
//! nothing in a world ever carries it, so a mask containing it simply
//! never matches.
//!
//! What replaces the `ITEM` inheritance is [`Line::segment_item`]: the
//! exact counterpart of `SEGMENT( const LINE&, const SEG& )`
//! (`pcbnew/router/pns_segment.h:59`), the throwaway segment KiCad builds
//! per chain segment whenever it wants to ask the node a question about a
//! line (`pcbnew/router/pns_node.cpp:310`, `:518`).
//! [`World::query_colliding_line`] and [`World::check_colliding_line`]
//! are that loop, plus one query for the via; see [`Line::via_item`].
//! Where the whole line has to answer at once, as the head of one
//! collision test, [`Line::rule_item`] and
//! [`crate::collide::LineHead`] take over.
//!
//! # Links, and what `links_valid_in` guarantees
//!
//! A link is a handle to a stored segment the line was assembled from, or
//! that [`World::add_line`] created for it. KiCad's are raw
//! `LINKED_ITEM*` into some node's index, valid only while that node is
//! alive and has not dropped them, with no back pointer and no
//! invalidation (note 02 section 2.3).
//!
//! [`Line::links_valid_in`] records **which node the handles were
//! resolved against**, which is what `SetOwner( this )` at
//! `pcbnew/router/pns_node.cpp:1152` is really for. It is provenance, not
//! liveness: nothing updates it when items leave that node, and a link
//! whose item has been removed simply goes stale. That is safe here in a
//! way it is not in C++, because a stale [`crate::item::ItemId`] fails
//! its generation check and every consumer in this module skips it
//! instead of dereferencing it. KiCad needs the root's `m_garbageItems`
//! pool for exactly this and still cannot tell a stale link from a live
//! one (note 02 section 11 entry 16).
//!
//! So the rule for callers: a link may be dead, and the answer to "what
//! is this line's rank" or "is any of it locked" is taken over the links
//! that still resolve. `Line::is_linked_checked` is the invariant KiCad
//! states as `IsLinked() && LinkCount() == ShapeCount()`
//! (`pcbnew/router/pns_line.h:125`), kept here as a `debug_assert!`
//! helper on the functions that consume links.
//!
//! # The via at the end
//!
//! A line may carry a via at its **last** point, and that via is part of
//! its collision footprint (note 02 section 2.11). C++ has one `VIA*`
//! that is sometimes owned and sometimes borrowed, told apart by
//! `m_via->BelongsTo( this )` (`pcbnew/router/pns_line.cpp:56`). This
//! crate makes the fork explicit as [`LineVia`], which is what `DESIGN.md`
//! sections 4.2 and 11 ask for.
//!
//! The payoff is in `src/collide.rs`. `shouldWeConsiderHoleCollisions`
//! carries a geometric heuristic (`pcbnew/router/pns_item.cpp:65`) that
//! prunes a via's hole against a hole that merely looks identical: same
//! position, same padstack, same net, same drill. It exists only because
//! a `LINE` holds a **copy** of a via that is also in the node, so the
//! two have different addresses and the honest identity test at `:71`
//! misses. With [`LineVia::Linked`] the two sides are literally the same
//! arena item and `parent_item != parent_head` catches it, so the
//! heuristic has nothing left to do for lines.
//!
//! It is gone. [`Line::via_item`] hands [`LineVia::Linked`] through as a
//! **stored** [`crate::rules::ItemRef`], so the identity tests prune both
//! the via pass and the hole pass, and [`LineVia::Owned`] as an unstored
//! one that owns no hole at all, because a hole is a separate arena item
//! and nothing drills one until [`World::add_via`] stores the via. So a
//! line never reaches the hole to hole branch the heuristic guarded. The
//! fixtures are in `src/node.rs`; `src/collide.rs` documents the one
//! answer that changed.
//!
//! # What is not ported
//!
//! - The four output `Walkaround( obstacle, pre, walk, post, cw )`
//!   overload (`pcbnew/router/pns_line.h:187`) is declared and never
//!   defined; calling it would not link.
//! - `ShowLinks` (`pcbnew/router/pns_line.h:193` and
//!   `pcbnew/router/pns_link_holder.h:100`), whose body is `#if 0` in one
//!   place and missing in the other.
//! - `DragArc` (`pcbnew/router/pns_line.cpp:911` to `:1141`), which needs
//!   `SHAPE_ARC`, `CIRCLE::ConstructFromTanTanPt` and `CalcArcMid`, none
//!   of which this crate has. The rest of the drag primitives are here:
//!   [`Line::drag_corner`] is `DragCorner` with both of its private
//!   halves, [`Line::drag_segment`] is `DragSegment`, and the two
//!   snappers are `Line::snap_dragged_corner` and
//!   `Line::snap_to_neighbour_segments`, both private.
//! - `dragSegmentFree` (`pcbnew/router/pns_line.h:265`), declared and
//!   never defined anywhere in the tree. `DragSegment`'s free angle
//!   branch is `assert( false )` (`pcbnew/router/pns_line.cpp:900`),
//!   which is why [`Line::drag_segment`] takes no free angle flag.
//! - `ClipToNearestObstacle` (`:679`). It has no caller anywhere in
//!   `pcbnew/router`, only its definition and its declaration, so it is
//!   left out even now that [`World::nearest_obstacle`] exists.
//! - `FindSegment( const SEGMENT* )` (`:1668`), which has no caller in
//!   the tree.
//! - `restoreUntouchedArcs` (`:255`), the arc splice at the end of the
//!   walkaround. It returns immediately when the input has no arcs
//!   (`:257`), and this crate has no arcs, so [`Line::walkaround`] omits
//!   the call rather than porting a routine that could not be exercised.
//! - `Is45Degree` does not exist on `LINE`; the only such predicate is
//!   the file local `IsSegment45Degree` in `pns_utils.cpp`, which
//!   `src/geometry/hull.rs` already has.
//! - No `remove_duplicate_points` or `simplify` wrapper. The router calls
//!   those on the chain through `Line()`
//!   (`pcbnew/router/pns_walkaround.cpp:171`,
//!   `pcbnew/router/pns_node.cpp:1204`), which is
//!   [`Line::chain_mut`] here.

use crate::geometry::box2::Box2;
use crate::geometry::direction45::{AngleType, CornerMode, Direction45};
use crate::geometry::hull::hull_intersection;
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::Vec2;
use crate::item::{
  HostId, Item, ItemBody, ItemId, LayerRange, MarkerFlags, NetId, Segment,
};
use crate::node::{NodeId, World};
use crate::rules::ItemRef;

/// The dummy width a fresh line starts with, in nanometres.
///
/// Port of the `m_width = 1` initialiser with its "Dummy value" comment,
/// `pcbnew/router/pns_line.h:71`. Every real line gets its width from the
/// segment it was assembled from or from the placer's sizes.
pub const DUMMY_WIDTH: i32 = 1;

/// The escape hatch of the walkaround's graph search.
///
/// Port of `int iterLimit = 1000`, `pcbnew/router/pns_line.cpp:498`,
/// whose comment says it out loud: "I'm not 100% sure this algorithm
/// doesn't have bugs that may cause it to freeze, so here's a temporary
/// iteration limit".
pub const WALKAROUND_ITERATION_LIMIT: u32 = 1000;

// ---------------------------------------------------------------------
// LineVia
// ---------------------------------------------------------------------

/// The via at a line's last point.
///
/// Replaces the single `VIA* m_via` (`pcbnew/router/pns_line.h:287`)
/// whose ownership KiCad recovers by asking `m_via->BelongsTo( this )`
/// (`pcbnew/router/pns_line.cpp:56`, `:78`, `:1651`). See the module
/// documentation for why the distinction is worth a type.
// `LineChain` grew two vectors with the arcs (work item 012 slice 3), which
// put this enum over clippy's size difference threshold. Boxing the large
// variant would move a hot path's payload to the heap to satisfy a lint
// about a value that is built once per call, so the lint is turned off
// here instead.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, PartialEq, Debug)]
pub enum LineVia {
  /// A via the line owns, which is in no node.
  ///
  /// `AppendVia` builds this one: it clones the via it is given and
  /// adopts the clone (`pcbnew/router/pns_line.cpp:1421`). The body is an
  /// [`Item`] rather than a bare [`crate::item::Via`] so that
  /// [`Line::via_item`] can hand the collision code a complete
  /// [`ItemRef`], which needs the net, the layer range and the flashing
  /// mask as well as the geometry.
  Owned(Item),
  /// A via that lives in a node, which the line only points at.
  ///
  /// `LinkVia` builds this one: it aliases the pointer and registers the
  /// via as a link, so `NODE::Remove( LINE& )` takes it out with the
  /// segments (`pcbnew/router/pns_line.cpp:1434`,
  /// `pcbnew/router/pns_node.cpp:1065`). `LINE( VIA* )`
  /// (`pcbnew/router/pns_line.h:96`) also produces this form, without the
  /// link.
  Linked(ItemId),
}

impl LineVia {
  /// The handle of a via that lives in a node, `None` for an owned one.
  pub const fn id(&self) -> Option<ItemId> {
    match self {
      LineVia::Owned(_) => None,
      LineVia::Linked(id) => Some(*id),
    }
  }

  /// Whether the line owns the via and would have to store it itself.
  pub const fn is_owned(&self) -> bool {
    matches!(self, LineVia::Owned(_))
  }
}

// ---------------------------------------------------------------------
// Line
// ---------------------------------------------------------------------

/// A run of segments seen as one polyline.
///
/// Port of `PNS::LINE`'s state (`pcbnew/router/pns_line.h:281` to `:288`
/// plus the `ITEM` and `LINK_HOLDER` fields it inherits). See the module
/// documentation for the three deliberate departures: it is a value and
/// not an item, its links carry the node they were resolved in, and its
/// via says whether it is owned.
#[derive(PartialEq, Debug)]
pub struct Line {
  /// The geometry. Port of `m_line`, `pcbnew/router/pns_line.h:281`.
  chain: LineChain,
  /// The track width in nanometres. Port of `m_width`, `:282`.
  ///
  /// Kept in step with the chain's own width, which is what `SetWidth`
  /// and `SetShape` arrange (`:131`, `:155`).
  width: i32,
  /// The layers the line spans. Port of `ITEM::m_layers`,
  /// `pcbnew/router/pns_item.h:319`.
  ///
  /// A `LINE` is not single layer in KiCad: `AssembleLine` copies the
  /// whole range off the seed segment (`pcbnew/router/pns_node.cpp:1148`)
  /// and `LINE( VIA* )` copies a via's range (`pcbnew/router/pns_line.h:104`),
  /// while `ITEM::Layer()` is just `Layers().Start()` (`pns_item.h:216`),
  /// which is what the placer and the walkaround read.
  layers: LayerRange,
  /// The net, or `None` for an unconnected line. Port of `ITEM::m_net`,
  /// `pcbnew/router/pns_item.h:318`.
  net: Option<NetId>,
  /// The via at the last point, if any. Port of `m_via`, `:287`.
  via: Option<LineVia>,
  /// The stored items this line was assembled from, in chain order.
  ///
  /// Port of `LINK_HOLDER::m_links`,
  /// `pcbnew/router/pns_link_holder.h:125`.
  links: Vec<ItemId>,
  /// The node [`Line::links`] were resolved against.
  ///
  /// Replaces the `SetOwner( this )` that `AssembleLine` performs on a
  /// line the node does not own (`pcbnew/router/pns_node.cpp:1152`). See
  /// the module documentation for what it does and does not guarantee.
  links_valid_in: Option<NodeId>,
  /// The obstacle that stopped this line, for mark obstacle mode.
  ///
  /// Port of `m_blockingObstacle`, `pcbnew/router/pns_line.h:288`. The
  /// router reads it when it paints the violation
  /// (`pcbnew/router/pns_router.cpp:738`).
  blocking_obstacle: Option<ItemId>,
  /// The line's own marks. Port of `ITEM::m_marker`,
  /// `pcbnew/router/pns_item.h:327`.
  ///
  /// [`Line::marker`] ORs the links' marks on top of this, and
  /// [`Line::unmark`] zeroes it, exactly as `pcbnew/router/pns_line.cpp:189`
  /// does.
  marker: MarkerFlags,
  /// The line's own shove rank. Port of `ITEM::m_rank`,
  /// `pcbnew/router/pns_item.h:328`.
  ///
  /// Only read when the line has no links; see [`Line::rank`].
  rank: i32,
  /// The window within which a dragged corner snaps, in nanometres.
  ///
  /// Port of `m_snapThreshhold` (KiCad's spelling),
  /// `pcbnew/router/pns_line.h:285`. Nothing here reads it, because the
  /// two heuristics that do belong to the dragger, but every copy path
  /// propagates it and `DRAGGER::startDragSegment` sets it
  /// (`pcbnew/router/pns_dragger.cpp:401`), so a line that loses it on a
  /// copy would drag differently.
  snap_threshold: i32,
  /// The host object this line came from, if any.
  ///
  /// Port of `ITEM::m_sourceItem`, `pcbnew/router/pns_item.h:322`.
  /// `AssembleLine` copies it off the seed segment
  /// (`pcbnew/router/pns_node.cpp:1151`) and `SEGMENT( const LINE&, const
  /// SEG& )` copies it back onto every segment the line is stored as
  /// (`pcbnew/router/pns_segment.h:64`), which is how the commit diff
  /// keeps a host's identity across a reroute.
  source: Option<HostId>,
}

impl Default for Line {
  fn default() -> Self {
    Self::new()
  }
}

impl Clone for Line {
  /// Copy the line, cloning an owned via and aliasing a linked one.
  ///
  /// Port of `LINE::LINE( const LINE& )`,
  /// `pcbnew/router/pns_line.cpp:42`, and of the identical body of
  /// `operator=` (`:83`) and the move assignment (`:123`). The via rule
  /// is the whole point:
  ///
  /// - an owned via is deep copied and the copy is **re netted to the
  ///   copy's net** (`:58` to `:60`), so the two lines never share it;
  /// - a linked via is aliased (`:64`), so both lines point at the same
  ///   item in the node, which is what makes a linked via survive a copy
  ///   of the line the shove is carrying around.
  ///
  /// Links, marks, rank, the blocking obstacle and the snap threshold are
  /// all copied, which is `copyLinks` (`:72`) plus the field by field
  /// assignments at `:68` to `:70`. `m_owner` is the one thing KiCad
  /// drops; [`Line::links_valid_in`] is copied instead, because it names
  /// the node the copied links point into and the copy's links are the
  /// same handles.
  fn clone(&self) -> Self {
    let via = match &self.via {
      None => None,
      Some(LineVia::Linked(id)) => Some(LineVia::Linked(*id)),
      Some(LineVia::Owned(item)) => {
        let mut copy = item.clone();
        copy.set_net(self.net);

        Some(LineVia::Owned(copy))
      }
    };

    Self {
      chain: self.chain.clone(),
      width: self.width,
      layers: self.layers,
      net: self.net,
      via,
      links: self.links.clone(),
      links_valid_in: self.links_valid_in,
      blocking_obstacle: self.blocking_obstacle,
      marker: self.marker,
      rank: self.rank,
      snap_threshold: self.snap_threshold,
      source: self.source,
    }
  }
}

impl Line {
  // -----------------------------------------------------------------
  // Construction
  // -----------------------------------------------------------------

  /// An empty line with KiCad's dummy defaults.
  ///
  /// Port of `LINE()`, `pcbnew/router/pns_line.h:67`: width
  /// [`DUMMY_WIDTH`], no snap threshold, no via, no blocking obstacle,
  /// and the `ITEM` defaults of no net, undefined layers, no marks and
  /// [`Item::UNASSIGNED_RANK`].
  pub fn new() -> Self {
    Self {
      chain: LineChain::new(),
      width: DUMMY_WIDTH,
      layers: LayerRange::UNDEFINED,
      net: None,
      via: None,
      links: Vec::new(),
      links_valid_in: None,
      blocking_obstacle: None,
      marker: MarkerFlags::NONE,
      rank: Item::UNASSIGNED_RANK,
      snap_threshold: 0,
      source: None,
    }
  }

  /// A line with another line's properties and a new shape.
  ///
  /// Port of `LINE( const LINE& aBase, const SHAPE_LINE_CHAIN& aLine )`,
  /// `pcbnew/router/pns_line.h:81`, which the optimizer uses to propose a
  /// replacement geometry (`pcbnew/router/pns_optimizer.cpp:1309`).
  ///
  /// The via is dropped (`:90`), but the **links are kept**: the
  /// constructor delegates to `LINK_HOLDER( aBase )`, whose implicit copy
  /// constructor copies `m_links`. That is easy to miss and it matters,
  /// because the result is a line whose links no longer match its shape,
  /// so [`Line::is_linked_checked`] fails for it. KiCad's callers either
  /// clear the links straight away or hand the line to
  /// [`World::replace_line`], which removes by link and adds by shape.
  /// The behaviour is reproduced rather than tidied.
  pub fn with_chain(base: &Line, chain: LineChain) -> Self {
    let mut line = Self {
      chain,
      width: base.width,
      layers: base.layers,
      net: base.net,
      via: None,
      links: base.links.clone(),
      links_valid_in: base.links_valid_in,
      blocking_obstacle: None,
      marker: base.marker,
      rank: base.rank,
      snap_threshold: base.snap_threshold,
      source: base.source,
    };

    line.chain.set_width(line.width);
    line
  }

  /// The one segment line of a stored segment.
  ///
  /// KiCad has no such constructor; the placer and the shove always go
  /// through [`World::assemble_line`], and the inverse direction is
  /// `SEGMENT( const LINE&, const SEG& )`
  /// (`pcbnew/router/pns_segment.h:59`), which this mirrors field for
  /// field: width, layers, net, marks, rank and the source host object,
  /// plus the two point chain `SEGMENT::CLine` builds
  /// (`pcbnew/router/pns_segment.h:104`).
  ///
  /// The segment is linked and [`Line::links_valid_in`] is set to `node`,
  /// so the result is a line of one shape and one link, which is what
  /// [`Line::is_linked_checked`] wants. Answers `None` for a handle the
  /// arena does not know or for an item that is not a segment.
  pub fn from_segment(
    world: &World,
    node: NodeId,
    segment: ItemId,
  ) -> Option<Line> {
    let item = world.item(segment)?;

    let ItemBody::Segment(body) = item.body() else {
      return None;
    };

    let mut line = Self::new();

    line.set_width(body.width());
    line.layers = item.layers();
    line.net = item.net();
    line.marker = item.marker();
    line.rank = item.rank();
    line.source = item.source();
    line.set_shape(body.line());
    line.links.push(segment);
    line.links_valid_in = Some(node);

    Some(line)
  }

  /// The degenerate line of a lone stitching via.
  ///
  /// Port of `LINE( VIA* aVia )`, `pcbnew/router/pns_line.h:96`, which
  /// takes the via's diameter as the line's width and copies its net,
  /// layers and rank. The chain stays empty, so the line has no points at
  /// all; the shove tests for exactly that shape
  /// (`pcbnew/router/pns_shove.cpp:248`).
  ///
  /// KiCad stores the pointer without claiming ownership (note 02
  /// section 2.11), which is [`LineVia::Linked`] here. It does **not**
  /// register the via as a link, so the line stays unlinked and
  /// [`World::remove_line`] would not remove the via.
  pub fn from_linked_via(via: ItemId, item: &Item) -> Line {
    let mut line = Self::new();
    let layers = item.layers();

    if let ItemBody::Via(body) = item.body() {
      line.set_width(body.diameter(layers, layers.start()));
    }

    line.net = item.net();
    line.layers = layers;
    line.rank = item.rank();
    line.via = Some(LineVia::Linked(via));

    line
  }

  // -----------------------------------------------------------------
  // Shape
  // -----------------------------------------------------------------

  /// The geometry.
  ///
  /// Port of `CLine` and of `Shape( int )`, which returns the same chain
  /// for every layer (`pcbnew/router/pns_line.h:138`, `:142`).
  pub const fn shape(&self) -> &LineChain {
    &self.chain
  }

  /// The geometry, mutably.
  ///
  /// Port of `Line()`, `pcbnew/router/pns_line.h:141`. This is how the
  /// router calls the chain's own simplifiers on a line, for instance
  /// `line.Line().Simplify2()` at
  /// `pcbnew/router/pns_walkaround.cpp:171` and
  /// `pl.Line().RemoveDuplicatePoints()` at
  /// `pcbnew/router/pns_node.cpp:1204`.
  ///
  /// Writing through it does not touch [`Line::links`], so the caller
  /// takes on the invariant [`Line::is_linked_checked`] states.
  pub const fn chain_mut(&mut self) -> &mut LineChain {
    &mut self.chain
  }

  /// Replace the geometry, re applying the line's width to it.
  ///
  /// Port of `SetShape`, `pcbnew/router/pns_line.h:131`.
  pub fn set_shape(&mut self, chain: LineChain) {
    self.chain = chain;
    self.chain.set_width(self.width);
  }

  /// The number of vertices. Port of `PointCount`,
  /// `pcbnew/router/pns_line.h:145`.
  pub fn point_count(&self) -> usize {
    self.chain.point_count()
  }

  /// The number of segments. Port of `SegmentCount`,
  /// `pcbnew/router/pns_line.h:144`.
  pub fn segment_count(&self) -> usize {
    self.chain.segment_count()
  }

  /// The number of straight segments plus whole arcs.
  ///
  /// Port of `ShapeCount`, `pcbnew/router/pns_line.h:147`, the count that
  /// is meant to match [`Line::link_count`] (note 02 section 2.4). With
  /// no arcs in the crate it equals [`Line::segment_count`]; it exists
  /// under its own name so that the invariant reads the way KiCad states
  /// it and so that the arc work has one place to change.
  pub fn shape_count(&self) -> usize {
    self.chain.segment_count()
  }

  /// One vertex. Port of `CPoint`, `pcbnew/router/pns_line.h:150`.
  ///
  /// # Panics
  ///
  /// When `index` is not a point of the chain, where KiCad's `CPoint`
  /// wraps once and then indexes out of bounds; see
  /// [`LineChain::point`].
  pub fn point(&self, index: usize) -> Vec2 {
    self.chain.point(index)
  }

  /// The last vertex, or `None` for an empty line.
  ///
  /// Port of `CLastPoint`, `pcbnew/router/pns_line.h:151`, which is
  /// undefined on an empty chain.
  pub fn last_point(&self) -> Option<Vec2> {
    self.chain.last_point()
  }

  /// One segment. Port of `CSegment`, `pcbnew/router/pns_line.h:152`.
  ///
  /// # Panics
  ///
  /// When `index` is not a segment of the chain.
  pub fn segment(&self, index: usize) -> Seg {
    self.chain.segment(index)
  }

  /// Drop the links, the via and the geometry.
  ///
  /// Port of `Clear`, `pcbnew/router/pns_line.cpp:1637`, which is
  /// `ClearLinks(); RemoveVia(); m_line.Clear();`. The width, the layers,
  /// the net and the marks survive, as they do in C++.
  pub fn clear(&mut self) {
    self.clear_links();
    self.remove_via();
    self.chain.clear();
  }

  // -----------------------------------------------------------------
  // Links
  // -----------------------------------------------------------------

  /// The stored items this line was assembled from.
  ///
  /// Port of `LINK_HOLDER::Links`,
  /// `pcbnew/router/pns_link_holder.h:66`. Some of them may be stale; see
  /// the module documentation.
  pub fn links(&self) -> &[ItemId] {
    &self.links
  }

  /// How many links the line holds. Port of `LinkCount`,
  /// `pcbnew/router/pns_link_holder.h:95`.
  pub fn link_count(&self) -> usize {
    self.links.len()
  }

  /// The node the links were resolved against, if any.
  ///
  /// See the module documentation for what this does and does not
  /// guarantee.
  pub const fn links_valid_in(&self) -> Option<NodeId> {
    self.links_valid_in
  }

  /// Record which node the links were resolved against.
  ///
  /// The counterpart of the `SetOwner( this )` that
  /// `NODE::AssembleLine` performs (`pcbnew/router/pns_node.cpp:1152`).
  /// [`World::assemble_line`] and [`World::add_line`] call it themselves;
  /// a caller that builds a link set by hand has to.
  pub const fn set_links_valid_in(&mut self, node: Option<NodeId>) {
    self.links_valid_in = node;
  }

  /// Whether the line came out of a node.
  ///
  /// Port of `IsLinked`, `pcbnew/router/pns_link_holder.h:69`, which is
  /// "the link vector is not empty".
  pub fn is_linked(&self) -> bool {
    !self.links.is_empty()
  }

  /// The consistency invariant: linked, with one link per shape.
  ///
  /// Port of `IsLinkedChecked`, `pcbnew/router/pns_line.h:125`. Note 02
  /// section 10.3 asks for it to be kept as a `debug_assert!` on every
  /// function that consumes links, which [`World::remove_line`] and
  /// [`World::replace_line`] do.
  pub fn is_linked_checked(&self) -> bool {
    self.is_linked() && self.link_count() == self.shape_count()
  }

  /// Whether an item is one of the line's links.
  ///
  /// Port of `ContainsLink`, `pcbnew/router/pns_link_holder.h:75`, a
  /// linear search. `NODE::Add( LINE& )` uses it to avoid double linking
  /// a reused segment (`pcbnew/router/pns_node.cpp:722`).
  pub fn contains_link(&self, item: ItemId) -> bool {
    self.links.contains(&item)
  }

  /// One link by index, counting from the end for a negative index.
  ///
  /// Port of `GetLink`, `pcbnew/router/pns_link_holder.h:80`, whose
  /// negative index wraps once. The placer reads link `s`
  /// (`pcbnew/router/pns_line_placer.cpp:1842`) and the multi dragger
  /// reads link `-1` (`pcbnew/router/pns_multi_dragger.cpp:442`). KiCad
  /// indexes out of bounds for anything further out; this answers `None`.
  pub fn link_at(&self, index: isize) -> Option<ItemId> {
    let count = self.links.len() as isize;
    let resolved = if index < 0 { index + count } else { index };

    if resolved < 0 || resolved >= count {
      return None;
    }

    self.links.get(resolved as usize).copied()
  }

  /// Register an item as part of this line.
  ///
  /// Port of `Link`, `pcbnew/router/pns_link_holder.h:46`, which refuses
  /// a duplicate and logs a debug warning. The refusal is what
  /// `NODE::Add( LINE& )` relies on when it re links a redundant segment
  /// (note 02 section 11 entry 9).
  ///
  /// The caller is responsible for [`Line::links_valid_in`]; the two
  /// functions that build a complete link set,
  /// [`World::assemble_line`] and [`World::add_line`], set it themselves.
  pub fn link(&mut self, item: ItemId) {
    if self.links.contains(&item) {
      return;
    }

    self.links.push(item);
  }

  /// Take an item out of the link list.
  ///
  /// Port of `Unlink`, `pcbnew/router/pns_link_holder.h:57`, whose
  /// `wxCHECK_MSG` guard turns "not linked" into a no op.
  pub fn unlink(&mut self, item: ItemId) {
    self.links.retain(|link| *link != item);
  }

  /// Detach the line from its node.
  ///
  /// Port of `ClearLinks`, `pcbnew/router/pns_link_holder.h:89`, which
  /// empties the vector without touching the items.
  /// [`Line::links_valid_in`] goes with it, since there are no handles
  /// left for it to qualify.
  pub fn clear_links(&mut self) {
    self.links.clear();
    self.links_valid_in = None;
  }

  // -----------------------------------------------------------------
  // Marks and rank
  // -----------------------------------------------------------------

  /// The line's own marks, without the links'.
  ///
  /// This is the bare `m_marker` field. Nothing in KiCad reads it on its
  /// own, because `LINE::Marker` shadows the base accessor; it is exposed
  /// here so that a test can tell the two apart.
  pub const fn own_marker(&self) -> MarkerFlags {
    self.marker
  }

  /// The line's marks, ORed with every link's.
  ///
  /// Port of `LINE::Marker`, `pcbnew/router/pns_line.cpp:193`. A link the
  /// arena no longer knows contributes nothing.
  pub fn marker(&self, world: &World) -> MarkerFlags {
    let mut marker = self.marker;

    for link in &self.links {
      if let Some(item) = world.item(*link) {
        marker |= item.marker();
      }
    }

    marker
  }

  /// Overwrite the marks on the line and on every link.
  ///
  /// Port of `LINE::Mark`, `pcbnew/router/pns_line.cpp:174`, which
  /// **assigns** rather than ORs, on the line and on each linked item.
  ///
  /// Deviation from note 02 section 10.3, which sketches `fn mark(&self,
  /// world: &mut World, ...)`. The `&self` there comes from KiCad's
  /// `const` method over a `mutable int m_marker`; since the line is a
  /// value here and the field is genuinely written, the signature says
  /// so. What the note is really after, that the fan out over the links
  /// cannot hide behind a shared reference, is what the `&mut World`
  /// gives.
  pub fn mark(&mut self, world: &mut World, marker: MarkerFlags) {
    self.marker = marker;

    for link in &self.links {
      if let Some(item) = world.item_mut(*link) {
        item.mark(marker);
      }
    }
  }

  /// Clear the masked marks on every link, then zero the line's own.
  ///
  /// Port of `LINE::Unmark`, `pcbnew/router/pns_line.cpp:184`. Note the
  /// asymmetry it reproduces: the links are cleared by `mask`, but the
  /// line's own marks are set to nothing whatever the mask was (`:189`).
  /// KiCad's default argument is `-1`, which is [`MarkerFlags::ALL`].
  pub fn unmark(&mut self, world: &mut World, mask: MarkerFlags) {
    for link in &self.links {
      if let Some(item) = world.item_mut(*link) {
        item.unmark(mask);
      }
    }

    self.marker = MarkerFlags::NONE;
  }

  /// Whether the line is locked.
  ///
  /// Port of `ITEM::IsLocked`, `pcbnew/router/pns_item.h:278`, which is
  /// `Marker() & MK_LOCKED` and therefore goes through the virtual
  /// [`Line::marker`], links included.
  pub fn is_locked(&self, world: &World) -> bool {
    self.marker(world).intersects(MarkerFlags::LOCKED)
  }

  /// Whether any linked item is locked.
  ///
  /// Port of `HasLockedSegments`, `pcbnew/router/pns_line.cpp:1626`. The
  /// shove refuses to push such a line aside
  /// (`pcbnew/router/pns_shove.cpp:647`). Unlike [`Line::is_locked`] this
  /// ignores the line's own marks.
  pub fn has_locked_segments(&self, world: &World) -> bool {
    self
      .links
      .iter()
      .any(|link| world.item(*link).is_some_and(Item::is_locked))
  }

  /// The line's own rank, without the links'.
  ///
  /// The bare `m_rank` field, which [`Line::rank`] only falls back to.
  pub const fn own_rank(&self) -> i32 {
    self.rank
  }

  /// The shove priority: the smallest rank over the links.
  ///
  /// Port of `LINE::Rank`, `pcbnew/router/pns_line.cpp:1449`. A linked
  /// line answers with the minimum over its links, an unlinked one with
  /// its own field, and `INT_MAX` (no link had a rank, or every link is
  /// stale) becomes [`Item::UNASSIGNED_RANK`].
  ///
  /// The shove's anti ping pong test is a comparison of two of these
  /// (`DESIGN.md` section 6.2), so the minimum is what makes a line as
  /// weak as its weakest segment.
  pub fn rank(&self, world: &World) -> i32 {
    if !self.is_linked() {
      return self.rank;
    }

    let mut min_rank = i32::MAX;

    for link in &self.links {
      if let Some(item) = world.item(*link) {
        min_rank = min_rank.min(item.rank());
      }
    }

    if min_rank == i32::MAX {
      Item::UNASSIGNED_RANK
    } else {
      min_rank
    }
  }

  /// Set the rank on the line and on every link.
  ///
  /// Port of `LINE::SetRank`, `pcbnew/router/pns_line.cpp:1439`.
  pub fn set_rank(&mut self, world: &mut World, rank: i32) {
    self.rank = rank;

    for link in &self.links {
      if let Some(item) = world.item_mut(*link) {
        item.set_rank(rank);
      }
    }
  }

  // -----------------------------------------------------------------
  // The via at the end
  // -----------------------------------------------------------------

  /// Whether a via sits at the line's last point.
  ///
  /// Port of `EndsWithVia`, `pcbnew/router/pns_line.h:195`.
  pub const fn ends_with_via(&self) -> bool {
    self.via.is_some()
  }

  /// The via at the last point, owned or linked.
  ///
  /// Port of `Via`, `pcbnew/router/pns_line.h:203`, which dereferences a
  /// null pointer when there is none.
  pub const fn via(&self) -> Option<&LineVia> {
    self.via.as_ref()
  }

  /// The via as the collision code wants it.
  ///
  /// A [`LineVia::Linked`] comes back as a **stored** reference, so that
  /// the hole pruning in `src/collide.rs` recognises it as the very item
  /// it is testing against and the geometric heuristic never has to fire.
  /// A [`LineVia::Owned`] comes back unstored, which is the case the
  /// heuristic still covers. See the module documentation.
  ///
  /// `None` when the line has no via, and when a linked via's handle has
  /// gone stale.
  pub fn via_item<'a>(&'a self, world: &'a World) -> Option<ItemRef<'a>> {
    match self.via.as_ref()? {
      LineVia::Owned(item) => Some(ItemRef::unstored(item)),
      LineVia::Linked(id) => {
        world.item(*id).map(|item| ItemRef::stored(*id, item))
      }
    }
  }

  /// Where the via sits.
  ///
  /// `VIA::Pos` (`pcbnew/router/pns_via.h:203`) read through whichever
  /// form the line holds. `None` when there is no via, when its handle is
  /// stale, or when the item is not a via.
  pub fn via_pos(&self, world: &World) -> Option<Vec2> {
    let item = self.via_item(world)?;

    match item.item().body() {
      ItemBody::Via(via) => Some(via.pos()),
      _ => None,
    }
  }

  /// Adopt a via at the line's last point.
  ///
  /// Port of `AppendVia`, `pcbnew/router/pns_line.cpp:1414`. The line is
  /// reversed first if the via sits at point 0, so that the via is always
  /// at the **last** point; then the via is cloned, adopted and re netted
  /// to the line's net.
  ///
  /// The clone is the caller's copy here: the [`Item`] is taken by value,
  /// which is the `aVia.Clone()` at `:1421`. KiCad's version leaks any
  /// via the line already owned, since it overwrites `m_via` without
  /// deleting it; here the old one is simply dropped.
  ///
  /// # Panics
  ///
  /// When the item is not a via, where KiCad's signature would not
  /// compile.
  pub fn append_via(&mut self, via: Item) {
    let ItemBody::Via(body) = via.body() else {
      panic!("append_via needs a via body");
    };

    let pos = body.pos();

    if self.chain.point_count() > 1 && pos == self.chain.point(0) {
      self.reverse();
    }

    let mut owned = via;
    owned.set_net(self.net);

    self.via = Some(LineVia::Owned(owned));
  }

  /// Point at a via that lives in a node, and link it.
  ///
  /// Port of `LinkVia`, `pcbnew/router/pns_line.cpp:1427`: the same
  /// reversal as [`Line::append_via`], then the pointer is aliased and
  /// the via is registered as a link, which is what makes
  /// [`World::remove_line`] take it out with the segments.
  ///
  /// Deviation: KiCad reads `aVia->Pos()` off the pointer it is handed.
  /// The position is a parameter here so that the call does not depend on
  /// the arena still knowing the handle, and so that it cannot silently
  /// skip the reversal for a stale one.
  pub fn link_via(&mut self, via: ItemId, pos: Vec2) {
    if self.chain.point_count() > 1 && pos == self.chain.point(0) {
      self.reverse();
    }

    self.via = Some(LineVia::Linked(via));
    self.link(via);
  }

  /// Drop the via.
  ///
  /// Port of `RemoveVia`, `pcbnew/router/pns_line.cpp:1645`: unlink it if
  /// it was linked, free it if it was owned, then null the field. Freeing
  /// is the drop of the owned [`Item`]; a linked via stays in its node.
  pub fn remove_via(&mut self) {
    if let Some(LineVia::Linked(id)) = self.via
      && self.contains_link(id)
    {
      self.unlink(id);
    }

    self.via = None;
  }

  /// Set the diameter of an owned via on every layer.
  ///
  /// Port of `SetViaDiameter`, `pcbnew/router/pns_line.h:206`, which the
  /// placer calls with the session's via size
  /// (`pcbnew/router/pns_line_placer.cpp:2014`). KiCad forces a complex
  /// padstack down to [`crate::item::StackMode::Normal`] with a warning
  /// before writing, which is reproduced.
  ///
  /// Deviation: a [`LineVia::Linked`] via is left alone. It is indexed in
  /// a node, and note 02 section 3.16 lists mutating an indexed item's
  /// geometry among the illegal mutations, because the spatial index
  /// keyed the insertion bounding box and could no longer remove it.
  /// KiCad writes through the pointer regardless. No caller does that:
  /// the placer's head always owns its via.
  pub fn set_via_diameter(&mut self, diameter: i32) {
    let Some(LineVia::Owned(item)) = self.via.as_mut() else {
      return;
    };

    let layers = item.layers();

    if let ItemBody::Via(via) = item.body_mut() {
      via.set_stack_mode(crate::item::StackMode::Normal);
      via.set_diameter(layers, crate::item::Via::ALL_LAYERS, diameter);
    }
  }

  /// Set the drill of an owned via.
  ///
  /// Port of `SetViaDrill`, `pcbnew/router/pns_line.h:215`. The same
  /// deviation as [`Line::set_via_diameter`] applies, and the hole the
  /// via owns is a separate arena item that the caller has to resize with
  /// it once the via is stored.
  pub fn set_via_drill(&mut self, drill: i32) {
    let Some(LineVia::Owned(item)) = self.via.as_mut() else {
      return;
    };

    if let ItemBody::Via(via) = item.body_mut() {
      via.set_drill(drill);
    }
  }

  // -----------------------------------------------------------------
  // Geometry
  // -----------------------------------------------------------------

  /// Reverse the point order and the link order together.
  ///
  /// Port of `Reverse`, `pcbnew/router/pns_line.cpp:1406`, which reverses
  /// both vectors so that they stay in correspondence. The via is not
  /// touched, which is why [`Line::append_via`] and [`Line::link_via`]
  /// reverse **before** attaching one: the invariant is that the via sits
  /// at the last point, and a reversal after the fact would break it.
  pub fn reverse(&mut self) {
    self.chain.reverse();
    self.links.reverse();
  }

  /// Move one corner of the line.
  ///
  /// Port of `DragCorner` (`pcbnew/router/pns_line.cpp:884`), the
  /// dispatcher: `free_angle` picks `dragCornerFree` (`:857`) over
  /// `dragCorner45` (`:823`).
  ///
  /// The signature carries KiCad's two default arguments rather than
  /// hiding them behind a second entry point, because all four call
  /// shapes are live in the tree: the shove's via fanout drag
  /// (`pcbnew/router/pns_shove.cpp:1101`) takes both defaults, the
  /// dragger passes `m_freeAngleMode` (`pcbnew/router/pns_dragger.cpp:407`)
  /// and the multi dragger passes a preferred ending direction
  /// (`pcbnew/router/pns_multi_dragger.cpp:863`).
  ///
  /// `index` is a **point** index. An index past the last point leaves
  /// the line alone, where KiCad would read out of range.
  pub fn drag_corner(
    &mut self,
    at: Vec2,
    index: usize,
    free_angle: bool,
    preferred_ending_direction: Direction45,
  ) {
    if index >= self.chain.point_count() {
      return;
    }

    if free_angle {
      self.drag_corner_free(at, index);
    } else {
      self.drag_corner_45(at, index, preferred_ending_direction);
    }
  }

  /// Move one corner and rebuild the 45 degree geometry around it.
  ///
  /// Port of `dragCorner45`, `pcbnew/router/pns_line.cpp:823`. The three
  /// cases: dragging the first point rebuilds the reversed chain,
  /// dragging the last point rebuilds the chain as it stands, and
  /// dragging a middle corner rebuilds both halves and joins them. The
  /// comment at `:844`, "fixme: awkward behaviour for outwards drags", is
  /// the known weakness of the middle case.
  fn drag_corner_45(
    &mut self,
    at: Vec2,
    index: usize,
    preferred_ending_direction: Direction45,
  ) {
    let width = self.chain.width();
    let snapped = self.snap_dragged_corner(at, index);
    let last = self.chain.point_count() - 1;

    // :830
    let mut path = if index == 0 {
      let mut dragged = drag_corner_internal(
        &self.chain.reversed(),
        snapped,
        preferred_ending_direction,
      );

      dragged.reverse();

      dragged
    } else if index == self.chain.segment_count() {
      // :834
      drag_corner_internal(&self.chain, snapped, preferred_ending_direction)
    } else {
      // :845
      let head = self.chain.slice(0, index).unwrap_or_default();
      let tail = self.chain.slice(index, last).unwrap_or_default().reversed();
      let mut first =
        drag_corner_internal(&head, snapped, preferred_ending_direction);
      let mut second =
        drag_corner_internal(&tail, snapped, preferred_ending_direction);

      second.reverse();
      first.append_chain(&second);

      first
    };

    // :851
    path.simplify(0);
    path.set_width(width);

    self.chain = path;
  }

  /// Move one corner to exactly where it was asked to go.
  ///
  /// Port of `dragCornerFree`, `pcbnew/router/pns_line.cpp:857`, which is
  /// the whole of the router's free angle mode: set the point and
  /// simplify. The arc vertex insertion at `:863` to `:878` has nothing
  /// to do here, because this crate has no arcs.
  ///
  /// `Simplify()` and not `Simplify2()`, so this is
  /// [`LineChain::simplify`] with a zero tolerance
  /// (`pcbnew/router/pns_line.cpp:881`).
  fn drag_corner_free(&mut self, at: Vec2, index: usize) {
    // :880
    self.chain.set_point(index, at);
    self.chain.simplify(0);
  }

  /// Pull a dragged corner onto the intersection of two of the line's own
  /// segments, when one is close enough.
  ///
  /// Port of `snapDraggedCorner`,
  /// `pcbnew/router/pns_line.cpp:1143`. It looks at the four segments
  /// around the dragged corner, intersects every obtuse pair of them as
  /// **lines** rather than as segments, and takes the nearest
  /// intersection within [`Line::snap_threshold`].
  ///
  /// A line whose threshold is zero, which is every line this crate
  /// builds unless a host sets one, short circuits to the point it was
  /// given (`:1153`).
  fn snap_dragged_corner(&self, at: Vec2, index: usize) -> Vec2 {
    if self.snap_threshold <= 0 {
      return at;
    }

    let segment_count = self.chain.segment_count();

    if segment_count == 0 {
      return at;
    }

    // :1146
    let start = index.saturating_sub(2);
    let end = (index + 2).min(segment_count - 1);
    let mut best: Option<(i32, Vec2)> = None;

    for outer in start..=end {
      let a = self.chain.segment(outer);

      for inner in start..outer {
        let b = self.chain.segment(inner);

        // :1163
        if !Direction45::from_seg(&a, false)
          .is_obtuse(Direction45::from_seg(&b, false))
        {
          continue;
        }

        let Some(point) = a.intersect_lines(&b) else {
          continue;
        };
        let distance = (point - at).euclidean_norm();

        if distance < self.snap_threshold
          && best.is_none_or(|(closest, _)| distance < closest)
        {
          best = Some((distance, point));
        }
      }
    }

    best.map_or(at, |(_, point)| point)
  }

  /// Pull a dragged segment onto the supporting line of a parallel
  /// neighbour two positions away, when one is close enough.
  ///
  /// Port of `snapToNeighbourSegments`,
  /// `pcbnew/router/pns_line.cpp:1185`. A different rule from
  /// [`Line::snap_dragged_corner`]: only the segments at `index - 2` and
  /// `index + 2` are candidates, only when their direction is exactly the
  /// dragged segment's, and the measure is the distance from `at` to
  /// their **supporting line**. The nearer of the two wins, ties going to
  /// the earlier one, and a tie in `snap_d` keeps the first because the
  /// test is `<` and not `<=` (`:1220`).
  ///
  /// The answer is a **point**, `s.A` of the winning neighbour
  /// (`:1201`, `:1211`), not a distance. [`Line::drag_segment`] turns it
  /// into `SEG( target, target + drag_dir )` (`:1330`), so only its
  /// projection onto the perpendicular of the drag direction matters.
  ///
  /// The early return is `== 0` here and `<= 0` in
  /// [`Line::snap_dragged_corner`] (`:1192` against `:1153`), so a
  /// negative threshold makes the two snappers disagree. That is KiCad's
  /// and it is transcribed rather than normalised; nothing in the router
  /// ever sets a negative threshold.
  fn snap_to_neighbour_segments(&self, at: Vec2, index: usize) -> Vec2 {
    // :1192
    if self.snap_threshold == 0 {
      return at;
    }

    let segment_count = self.chain.segment_count();

    if index >= segment_count {
      return at;
    }

    let drag_direction =
      Direction45::from_seg(&self.chain.segment(index), false);
    let mut candidates: [Option<(i32, Vec2)>; 2] = [None, None];

    // :1195
    if index >= 2 {
      let previous = self.chain.segment(index - 2);

      if Direction45::from_seg(&previous, false) == drag_direction {
        candidates[0] = Some((previous.line_distance(at), previous.a));
      }
    }

    // :1205
    if index + 2 < segment_count {
      let next = self.chain.segment(index + 2);

      if Direction45::from_seg(&next, false) == drag_direction {
        candidates[1] = Some((next.line_distance(at), next.a));
      }
    }

    // :1216
    let mut best = at;
    let mut best_distance = i32::MAX;

    for candidate in candidates.into_iter().flatten() {
      let (distance, point) = candidate;

      if distance < best_distance && distance <= self.snap_threshold {
        best_distance = distance;
        best = point;
      }
    }

    best
  }

  /// Move one segment of the line sideways and rebuild the two corners
  /// around it.
  ///
  /// Port of `DragSegment` (`pcbnew/router/pns_line.cpp:898`) and the
  /// `dragSegment45` behind it (`:1230`). There is no free angle form:
  /// KiCad's `DragSegment( aP, aIndex, true )` is `assert( false )`
  /// (`:900` to `:903`) and the `dragSegmentFree` it would call
  /// (`pcbnew/router/pns_line.h:265`) is declared and never defined, so
  /// the dragger routes a free angle click to a corner drag instead
  /// (`pcbnew/router/pns_dragger.cpp:135` to `:145`).
  ///
  /// The routine works on a **copy** of the chain into which zero length
  /// segments are inserted so that the dragged segment always has a
  /// previous and a next neighbour to bend. Four candidate paths are
  /// built, one per pair of the two 45 degree ways out at each end, and
  /// the shortest wins. The result is spliced back into the original
  /// chain by the **original** index, which is what the three `Replace`
  /// cases at `:1389` to `:1395` are doing.
  ///
  /// `index` is a **segment** index. KiCad asserts `aIndex <
  /// PointCount()` (`:1235`) and then reads `CSegment( index + 1 )`, so an
  /// index that is not a segment index is out of range there; here it
  /// leaves the line alone.
  ///
  /// The `m_line.PointCount() == 1` branch at `:1387` is not ported: it
  /// is unreachable behind the same assertion.
  pub fn drag_segment(&mut self, at: Vec2, index: usize) {
    if index >= self.chain.segment_count() {
      return;
    }

    // :1231
    let mut path = self.chain.clone();
    // :1240
    let target = self.snap_to_neighbour_segments(at, index);
    let mut cursor = index;

    // :1247. Guarantee a previous segment. Without arcs the only reason
    // to pad is a drag of the very first segment, so the
    // `index > 0 ? index + 1 : 0` insertion point is always zero.
    if cursor == 0 {
      path.insert(0, path.point(0));
      cursor += 1;
    }

    // :1253. Guarantee a next segment. The `IsPtOnArc` alternative at
    // `:1257` has nothing to match without arcs.
    if cursor + 1 == path.segment_count() {
      let last = path.point(path.point_count() - 1);

      path.insert(path.point_count() - 1, last);
    }

    let drag_direction = Direction45::from_seg(&path.segment(cursor), false);
    let mut direction_prev =
      Direction45::from_seg(&path.segment(cursor - 1), false);
    let mut direction_next =
      Direction45::from_seg(&path.segment(cursor + 1), false);

    // :1271. A neighbour running the same way as the dragged segment
    // cannot bend, so it is split into a zero length stub that can.
    if direction_prev == drag_direction {
      direction_prev = direction_prev.left();
      path.insert(cursor, path.point(cursor));
      cursor += 1;
    } else if !direction_prev.is_defined() {
      // :1277
      direction_prev = drag_direction.left();
    }

    // :1282
    if direction_next == drag_direction {
      direction_next = direction_next.right();
      path.insert(cursor + 1, path.point(cursor + 1));
    } else if !direction_next.is_defined() {
      // :1287
      direction_next = drag_direction.right();
    }

    // :1292
    let previous = path.segment(cursor - 1);
    let next = path.segment(cursor + 1);
    let dragged = path.segment(cursor);

    // :1296. Two guide lines at each end: the two 45 degree ways out.
    // The tests are on the **original** index, so a drag of an end
    // segment lets that end swing freely.
    let guide_a = if index == 0 {
      [
        Seg::new(dragged.a, dragged.a + drag_direction.right().to_vector()),
        Seg::new(dragged.a, dragged.a + drag_direction.left().to_vector()),
      ]
    } else if direction_prev
      .angle(drag_direction)
      .intersects(AngleType::OBTUSE.union(AngleType::HALF_FULL))
    {
      // :1306
      [
        Seg::new(previous.a, previous.a + drag_direction.left().to_vector()),
        Seg::new(previous.a, previous.a + drag_direction.right().to_vector()),
      ]
    } else {
      // :1310
      let guide = Seg::new(dragged.a, dragged.a + direction_prev.to_vector());

      [guide, guide]
    };

    // :1313
    let guide_b = if index + 1 == self.chain.segment_count() {
      [
        Seg::new(dragged.b, dragged.b + drag_direction.right().to_vector()),
        Seg::new(dragged.b, dragged.b + drag_direction.left().to_vector()),
      ]
    } else if direction_next
      .angle(drag_direction)
      .intersects(AngleType::OBTUSE.union(AngleType::HALF_FULL))
    {
      [
        Seg::new(next.b, next.b + drag_direction.left().to_vector()),
        Seg::new(next.b, next.b + drag_direction.right().to_vector()),
      ]
    } else {
      let guide = Seg::new(dragged.b, dragged.b + direction_next.to_vector());

      [guide, guide]
    };

    // :1330. The supporting line the dragged segment has to end up on.
    let current = Seg::new(target, target + drag_direction.to_vector());
    let mut best: Option<(i64, LineChain)> = None;

    // :1335
    for first in &guide_a {
      for second in &guide_b {
        let (Some(near), Some(far)) = (
          current.intersect_lines(first),
          current.intersect_lines(second),
        ) else {
          continue;
        };

        // `SEG s2( *ip1, *ip2 )` at `:1349` is never read; it is not
        // transcribed.
        let entry = Seg::new(previous.a, near);
        let exit = Seg::new(far, next.b);
        let mut candidate = LineChain::new();

        if let Some(point) = entry.intersect(&next, false, false) {
          // :1353. The new segment's line crosses the far neighbour, so
          // the far corner disappears.
          candidate.append(entry.a);
          candidate.append(point);
          candidate.append(next.b);
        } else if let Some(point) = exit.intersect(&previous, false, false) {
          // :1359
          candidate.append(previous.a);
          candidate.append(point);
          candidate.append(exit.b);
        } else if let Some(point) = entry.intersect(&exit, false, false) {
          // :1365
          candidate.append(previous.a);
          candidate.append(point);
          candidate.append(next.b);
        } else {
          // :1373
          candidate.append(previous.a);
          candidate.append(near);
          candidate.append(far);
          candidate.append(next.b);
        }

        // :1381
        let length = candidate.length();

        if best.as_ref().is_none_or(|(shortest, _)| length < *shortest) {
          best = Some((length, candidate));
        }
      }
    }

    let best = best.map_or_else(LineChain::new, |(_, chain)| chain);
    let last = self.chain.segment_count() - 1;

    // :1389. The splice indexes the original chain, not `path`: `best`
    // spans `previous.a` to `next.b`, which are the same two points as
    // `index` and `index + 1` here whenever a pad was inserted at that
    // end.
    if index == 0 {
      self.chain.replace_with_chain(0, 1, &best);
    } else if index == last {
      // :1392, KiCad's `Replace( -2, -1, best )`.
      let point_count = self.chain.point_count();

      self
        .chain
        .replace_with_chain(point_count - 2, point_count - 1, &best);
    } else {
      self.chain.replace_with_chain(index, index + 1, &best);
    }

    // :1396
    self.chain.simplify(0);
  }

  /// How many corners of the given kinds the line turns.
  ///
  /// Port of `CountCorners`, `pcbnew/router/pns_line.cpp:218`: every
  /// consecutive pair of segments whose [`Direction45::angle`] matches
  /// the mask. The placer refuses a head with a forbidden corner
  /// (`pcbnew/router/pns_line_placer.cpp:347`) and the optimizer counts
  /// obtuse ones (`pcbnew/router/pns_optimizer.cpp:666`).
  ///
  /// The directions are built with `a90` false, KiCad's default for
  /// `DIRECTION_45( const SEG& )`
  /// (`libs/kimath/include/geometry/direction45.h:103`), so a 90 degree
  /// corner classifies as [`AngleType::RIGHT`] and not as a straight run.
  pub fn count_corners(&self, angles: AngleType) -> usize {
    let segment_count = self.chain.segment_count();
    let mut count = 0;

    if segment_count < 2 {
      return 0;
    }

    for index in 0..segment_count - 1 {
      let first = Direction45::from_seg(&self.chain.segment(index), false);
      let second = Direction45::from_seg(&self.chain.segment(index + 1), false);

      if first.angle(second).intersects(angles) {
        count += 1;
      }
    }

    count
  }

  /// Whether the line visits any point twice.
  ///
  /// Port of `HasLoops`, `pcbnew/router/pns_line.cpp:1516`, the same
  /// O(n squared) scan over vertex pairs at least two apart. The shove
  /// throws away a walkaround result that has one
  /// (`pcbnew/router/pns_shove.cpp:845`).
  pub fn has_loops(&self) -> bool {
    let point_count = self.chain.point_count();

    for first in 0..point_count {
      for second in first + 2..point_count {
        if self.chain.point(first) == self.chain.point(second) {
          return true;
        }
      }
    }

    false
  }

  /// Whether two lines have the same geometry.
  ///
  /// Port of `CompareGeometry`, `pcbnew/router/pns_line.cpp:1400`, which
  /// delegates to the chain.
  pub fn compare_geometry(&self, other: &Line) -> bool {
    self.chain.compare_geometry(&other.chain)
  }

  /// Cut the line down to a range of vertices, links included.
  ///
  /// Port of `ClipVertexRange`, `pcbnew/router/pns_line.cpp:1469`. The
  /// chain is sliced and the link vector is rotated and truncated to the
  /// matching sub range, walking shapes through `SHAPE_LINE_CHAIN::NextShape`
  /// so that an arc counts once. Its documented precondition is that the
  /// range came from joints, so it never cuts inside an arc (`:1471`).
  ///
  /// Two details of the C++ are reproduced rather than tidied. The walk
  /// stops at `i >= aEnd - 1`, one shape short of the range's end, which
  /// is what makes `lastLink` name the last link **inside** the range.
  /// And `std::rotate` is called with `m_links.begin() + lastLink` as its
  /// end (`:1508`), not with `m_links.end()`, so links past `lastLink`
  /// take no part in the rotation and are then dropped by the resize.
  ///
  /// Its only caller is [`World::find_lines_between_joints`]
  /// (`pcbnew/router/pns_node.cpp:1247`); the multi dragger is the other
  /// one and is milestone 9, last of it.
  ///
  /// # Panics
  ///
  /// When the range is not a valid slice of the chain, where KiCad's
  /// `Slice` answers with an empty chain.
  pub fn clip_vertex_range(&mut self, start: usize, end: usize) {
    let point_count = self.chain.point_count();
    let mut first_link = 0usize;
    let mut last_link = self.links.len().saturating_sub(1);
    let mut link_index = 0usize;
    let mut index: isize = 0;

    while index >= 0 && (index as usize) < point_count {
      let point = index as usize;

      if point <= start {
        first_link = link_index;
      }

      // KiCad's `i >= aEnd - 1` in signed arithmetic. `end` is a point
      // index, so `point + 1 >= end` says the same thing without the
      // underflow that `end == 0` would cause here.
      if point + 1 >= end || link_index >= last_link {
        last_link = link_index;
        break;
      }

      link_index += 1;
      index = next_shape(point_count, point);
    }

    self.chain = self
      .chain
      .slice(start, end)
      .expect("clip_vertex_range needs a valid vertex range");
    self.chain.set_width(self.width);

    if !self.is_linked() {
      return;
    }

    debug_assert!(
      last_link >= first_link,
      "clip_vertex_range produced an inverted link range"
    );

    if first_link <= last_link && last_link <= self.links.len() {
      self.links[..last_link].rotate_left(first_link);
      self.links.truncate(last_link - first_link + 1);
    }
  }

  /// The area that differs between two versions of a line.
  ///
  /// Port of `ChangedArea`, `pcbnew/router/pns_line.cpp:1545`: both
  /// chains are simplified, the common prefix and the common suffix are
  /// found, and the box covers every vertex in between on either side,
  /// inflated by the larger of the two widths. The shove accumulates
  /// these to bound the region it hands to the optimizer
  /// (`pcbnew/router/pns_shove.cpp:87`).
  ///
  /// One subtlety of the prefix scan is reproduced: a vertex that differs
  /// but still lies **on** this line's segment at the same index does not
  /// start the changed region (`:1574`), so a line that merely gained a
  /// vertex on a straight run reports no change there.
  pub fn changed_area(&self, other: &Line) -> Option<Box2> {
    let mut mine = self.chain.clone();
    mine.simplify(0);
    let mut theirs = other.chain.clone();
    theirs.simplify(0);

    let count_mine = mine.point_count();
    let count_theirs = theirs.point_count();

    if count_mine == 0 || count_theirs == 0 {
      return None;
    }

    let common = count_mine.min(count_theirs);
    let mut start = None;
    let mut end_mine = None;
    let mut end_theirs = None;

    for index in 0..common {
      let first = mine.point(index);
      let second = theirs.point(index);

      if first == second {
        continue;
      }

      if index != common - 1 && mine.segment(index).contains_point(second) {
        continue;
      }

      start = Some(index);
      break;
    }

    for index in 0..common {
      let first = mine.point(count_mine - 1 - index);
      let second = theirs.point(count_theirs - 1 - index);

      if first != second {
        end_mine = Some(count_mine - 1 - index);
        end_theirs = Some(count_theirs - 1 - index);
        break;
      }
    }

    let start = start.unwrap_or(common);
    let end_mine = end_mine.unwrap_or(count_mine - 1);
    let end_theirs = end_theirs.unwrap_or(count_theirs - 1);

    let mut area: Option<Box2> = None;

    for index in start..=end_mine {
      area = Some(match area {
        None => Box2::from_vec2(mine.point(index)),
        Some(box2) => box2.merge_point(mine.point(index).into()),
      });
    }

    for index in start..=end_theirs {
      area = Some(match area {
        None => Box2::from_vec2(theirs.point(index)),
        Some(box2) => box2.merge_point(theirs.point(index).into()),
      });
    }

    area.map(|box2| box2.inflate_by(i64::from(self.width.max(other.width))))
  }

  /// The bounding box of the geometry, grown by a clearance.
  ///
  /// KiCad reaches this through `ITEM::Shape( -1 )->BBox( aClearance )`
  /// (`pcbnew/router/pns_line.h:138`), which is the chain's own box. Note
  /// that [`LineChain::bbox`] also grows by the chain's width, which is
  /// the line's width, so the result already covers the copper.
  pub fn bbox(&self, clearance: i32) -> Option<Box2> {
    self.chain.bbox(clearance)
  }

  /// One end of the line.
  ///
  /// Port of `Anchor`, `pcbnew/router/pns_line.h:221`: anchor 0 is the
  /// first point, any other index is the last. An empty line answers
  /// `(0, 0)`, which is KiCad's default constructed `VECTOR2I`.
  pub fn anchor(&self, n: usize) -> Vec2 {
    if self.chain.point_count() < 1 {
      return Vec2::new(0, 0);
    }

    if n == 0 {
      self.chain.point(0)
    } else {
      self.chain.point(self.chain.point_count() - 1)
    }
  }

  /// How many ends the line has.
  ///
  /// Port of `AnchorCount`, `pcbnew/router/pns_line.h:229`: two once
  /// there are two points, otherwise the point count, so an empty line
  /// has none.
  pub fn anchor_count(&self) -> usize {
    if self.chain.point_count() >= 2 {
      2
    } else {
      self.chain.point_count()
    }
  }

  // -----------------------------------------------------------------
  // Properties
  // -----------------------------------------------------------------

  /// The track width. Port of `Width`, `pcbnew/router/pns_line.h:162`.
  pub const fn width(&self) -> i32 {
    self.width
  }

  /// Set the track width on the line and on its chain.
  ///
  /// Port of `SetWidth`, `pcbnew/router/pns_line.h:155`, which writes
  /// both.
  pub fn set_width(&mut self, width: i32) {
    self.width = width;
    self.chain.set_width(width);
  }

  /// The layers the line spans. Port of `ITEM::Layers`,
  /// `pcbnew/router/pns_item.h:212`.
  pub const fn layers(&self) -> LayerRange {
    self.layers
  }

  /// Set the layer range. Port of `ITEM::SetLayers`,
  /// `pcbnew/router/pns_item.h:213`.
  pub const fn set_layers(&mut self, layers: LayerRange) {
    self.layers = layers;
  }

  /// The first layer. Port of `ITEM::Layer`,
  /// `pcbnew/router/pns_item.h:216`, which is `Layers().Start()`. This is
  /// what the placer and the walkaround mean by "the line's layer".
  pub const fn layer(&self) -> i32 {
    self.layers.start()
  }

  /// Collapse the line onto one layer. Port of `ITEM::SetLayer`,
  /// `pcbnew/router/pns_item.h:215`.
  pub const fn set_layer(&mut self, layer: i32) {
    self.layers = LayerRange::single(layer);
  }

  /// The net. Port of `ITEM::Net`, `pcbnew/router/pns_item.h:210`.
  pub const fn net(&self) -> Option<NetId> {
    self.net
  }

  /// Set the net. Port of `ITEM::SetNet`,
  /// `pcbnew/router/pns_item.h:209`.
  ///
  /// The via is not re netted here, which is KiCad's behaviour too: only
  /// [`Line::append_via`] and the copy paths do that (`:60`, `:1423`).
  pub const fn set_net(&mut self, net: Option<NetId>) {
    self.net = net;
  }

  /// The corner snapping window. Port of `GetSnapThreshhold`,
  /// `pcbnew/router/pns_line.h:257`.
  pub const fn snap_threshold(&self) -> i32 {
    self.snap_threshold
  }

  /// Set the corner snapping window. Port of `SetSnapThreshhold`,
  /// `pcbnew/router/pns_line.h:252`.
  pub const fn set_snap_threshold(&mut self, threshold: i32) {
    self.snap_threshold = threshold;
  }

  /// What stopped this line, in mark obstacle mode.
  ///
  /// Port of `GetBlockingObstacle`, `pcbnew/router/pns_line.h:235`.
  pub const fn blocking_obstacle(&self) -> Option<ItemId> {
    self.blocking_obstacle
  }

  /// Record what stopped this line.
  ///
  /// Port of `SetBlockingObstacle`, `pcbnew/router/pns_line.h:234`.
  pub const fn set_blocking_obstacle(&mut self, obstacle: Option<ItemId>) {
    self.blocking_obstacle = obstacle;
  }

  /// The host object this line came from.
  ///
  /// Port of `ITEM::GetSourceItem`, `pcbnew/router/pns_item.h:202`.
  pub const fn source(&self) -> Option<HostId> {
    self.source
  }

  /// Set the host object this line came from.
  ///
  /// Port of `ITEM::SetSourceItem`, `pcbnew/router/pns_item.h:201`.
  pub const fn set_source(&mut self, source: Option<HostId>) {
    self.source = source;
  }

  // -----------------------------------------------------------------
  // Adapters
  // -----------------------------------------------------------------

  /// One chain segment as a throwaway item.
  ///
  /// Port of `SEGMENT( const LINE& aParentLine, const SEG& aSeg )`,
  /// `pcbnew/router/pns_segment.h:59`, which copies the line's width,
  /// net, layers, marks, rank and source host object onto a fresh
  /// segment with a null parent.
  ///
  /// This is the adapter that stands in for the `ITEM` base class a
  /// `LINE` has in C++ and this crate's [`Line`] does not. Every question
  /// the node is asked about a line goes through it, one segment at a
  /// time: `NODE::NearestObstacle` (`pcbnew/router/pns_node.cpp:310`),
  /// `NODE::CheckColliding` (`:518`) and [`World::add_line`], which is
  /// the only one of the three that stores what it builds.
  ///
  /// The `uid` is the caller's, because uids come from the world's
  /// counter (`DESIGN.md` section 8): pass [`World::next_uid`] for an
  /// item that will be stored, and anything at all for a probe that will
  /// not, since nothing sorts by the uid of an unstored item. The world
  /// is needed for the two properties `SEGMENT`'s constructor reads
  /// through virtual accessors, the marks and the rank, both of which a
  /// linked line takes from its links.
  ///
  /// # Panics
  ///
  /// When `index` is not a segment of the chain.
  pub fn segment_item(&self, world: &World, index: usize, uid: u64) -> Item {
    self.item_over(world, self.chain.segment(index), uid)
  }

  /// The whole line as one throwaway item for the rule queries.
  ///
  /// `ITEM::collideSimple` hands the `LINE*` it was given straight to
  /// `IsKeepout`, `IsNetTieExclusion` and `NODE::GetClearance`
  /// (`pcbnew/router/pns_item.cpp:198`, `:220`, `:254`), and
  /// `NODE::NearestObstacle` does the same when it asks for the clearance
  /// that sizes a hull (`pcbnew/router/pns_node.cpp:360`). A `LINE` is an
  /// `ITEM` there and a [`Line`] is not one here, so the resolver is
  /// handed this stand in instead.
  ///
  /// It carries every property a rule can read, which is what
  /// [`Line::segment_item`] copies as well: the width, the net, the
  /// layers, the marks, the rank and the source host object. Its
  /// **geometry is a placeholder**, the straight segment from the first
  /// point to the last, and no caller may use it: the shape the collision
  /// test uses for a line is the chain, which
  /// [`crate::collide::LineHead`] carries separately.
  ///
  /// See [`Line::segment_item`] for what `uid` is for.
  pub fn rule_item(&self, world: &World, uid: u64) -> Item {
    let first = if self.chain.point_count() > 0 {
      self.chain.point(0)
    } else {
      Vec2::new(0, 0)
    };
    let last = self.last_point().unwrap_or(first);

    self.item_over(world, Seg::new(first, last), uid)
  }

  /// The body of both throwaway item builders.
  ///
  /// The constructor `SEGMENT( const LINE& aParentLine, const SEG& aSeg )`
  /// (`pcbnew/router/pns_segment.h:59`) with the segment as a parameter,
  /// so that [`Line::segment_item`] can pass one of the chain's and
  /// [`Line::rule_item`] a placeholder.
  fn item_over(&self, world: &World, seg: Seg, uid: u64) -> Item {
    let body = Segment::new(seg, self.width);
    let mut item = Item::new(uid, ItemBody::Segment(body));

    item.set_layers_and_flash_all(self.layers);
    item.set_net(self.net);
    // `aParentLine.Marker()` and `aParentLine.Rank()` at
    // `pcbnew/router/pns_segment.h:67` and `:68` are the virtual ones, so
    // a linked line hands its links' marks and its smallest rank on.
    item.mark(self.marker(world));
    item.set_rank(self.rank(world));
    item.set_source(self.source);

    item
  }
}

// ---------------------------------------------------------------------
// Walkaround
// ---------------------------------------------------------------------

/// Where a vertex of the walkaround graph sits relative to the hull.
///
/// Port of the function local `enum VERTEX_TYPE { INSIDE, OUTSIDE,
/// ON_EDGE }`, `pcbnew/router/pns_line.cpp:317`.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum VertexType {
  /// Strictly inside the hull, so the walk may never stand here.
  Inside,
  /// Strictly outside the hull.
  Outside,
  /// On the hull's outline, which every hull vertex is by construction.
  OnEdge,
}

/// One node of the directed graph the walkaround searches.
///
/// Port of the function local `struct VERTEX`,
/// `pcbnew/router/pns_line.cpp:321`, with one change: KiCad's
/// `std::vector<VERTEX*> neighbours` points into the `vts` vector itself,
/// which is why `vts.reserve` at `:402` is a memory safety requirement
/// and not an optimisation (note 02 section 11 entry 13). Here the
/// neighbours are indices, so the reserve is only a reserve.
struct WalkVertex {
  /// Inside, outside or on the hull's outline. Port of `type`, `:324`.
  vertex_type: VertexType,
  /// Whether the vertex came from the hull. Port of `isHull`, `:326`.
  is_hull: bool,
  /// Where the vertex is. Port of `pos`, `:328`.
  pos: Vec2,
  /// Indices of the vertices reachable from here. Port of `neighbours`,
  /// `:330`.
  neighbours: Vec<usize>,
  /// The vertex's index in the split path, or `-1`. Port of `indexp`,
  /// `:332`.
  ///
  /// The `-1` is kept as a value rather than becoming an `Option`,
  /// because [`are_neighbours`] compares the raw numbers and its answer
  /// for `-1` is part of the behaviour; see there.
  index_p: i32,
  /// The vertex's index in the split hull, or `-1`. Port of `indexh`,
  /// `:334`.
  index_h: i32,
  /// Whether the search has stood here. Port of `visited`, `:336`.
  visited: bool,
}

/// Whether two path indices are adjacent along the path.
///
/// Port of the file local `areNeighbours`,
/// `pcbnew/router/pns_line.cpp:239`. It is asymmetric and it is fed `-1`
/// for vertices that are not on the path at all, which is why the
/// sentinel survives the port: with `x == -1` the second test reads `x <
/// max - 1 && x + 1 == y`, so a hull only vertex counts as a neighbour of
/// **path vertex 0**. No call site can reach that combination, because a
/// path vertex that has a hull only neighbour is itself a hull vertex and
/// therefore classified `ON_EDGE` rather than `OUTSIDE`, but the
/// transcription keeps the rule instead of the reasoning.
const fn are_neighbours(x: i32, y: i32, max: i32) -> bool {
  if x > 0 && x - 1 == y {
    return true;
  }

  if x < max - 1 && x + 1 == y {
    return true;
  }

  false
}

/// The point index the next shape of an open arcless chain starts at.
///
/// Port of `SHAPE_LINE_CHAIN::NextShape`,
/// `libs/kimath/src/geometry/shape_line_chain.cpp:1302`, reduced to the
/// case this crate can produce: with no arcs every shape is one segment,
/// so the answer is the next point index, except that KiCad refuses to
/// wrap (`:1313`) and answers `-1` one point before the end of an open
/// chain (`:1318`). The `-1` is kept as the return value because
/// [`Line::clip_vertex_range`] uses it as its loop's stop condition.
const fn next_shape(point_count: usize, index: usize) -> isize {
  if point_count == 0 || index + 1 >= point_count {
    return -1;
  }

  // `aPointIndex == lastIndex - 1` on an open chain.
  if index + 2 == point_count {
    return -1;
  }

  (index + 1) as isize
}

/// The first vertex at a position.
///
/// Port of the `findVertex` lambda, `pcbnew/router/pns_line.cpp:347`, a
/// linear scan that answers with the **first** match, so a path that
/// visits one position twice only ever resolves to its first visit.
fn find_vertex(vertices: &[WalkVertex], pos: Vec2) -> Option<usize> {
  vertices.iter().position(|vertex| vertex.pos == pos)
}

/// Rebuild the tail of a chain so that it ends at a point.
///
/// Port of `dragCornerInternal`, `pcbnew/router/pns_line.cpp:722`. It
/// walks the chain backwards looking for the last corner from which a 45
/// degree trace to the new point ends in `preferred_ending_direction`,
/// or leaves in the same direction as the segment it replaces, or turns
/// obtusely away from the segment before it; the chain up to that corner
/// is kept and the trace is appended.
///
/// KiCad builds both start postures at `:757` and classifies them at
/// `:763`, but `BuildInitialTrace` overrides the posture whenever the
/// direction it is called on is defined
/// (`libs/kimath/src/geometry/direction_45.cpp` and
/// [`Direction45::build_initial_trace`]), so the two candidates are the
/// same chain. The loop is transcribed as it stands rather than halved,
/// because that is where a later corner mode would make them differ
/// again.
///
/// `preferred_ending_direction` is an undefined [`Direction45`] for every
/// caller but `MULTI_DRAGGER` (`pcbnew/router/pns_multi_dragger.cpp:863`),
/// and the block at `:768` is skipped for it.
///
/// Deviation: KiCad's preferred direction block indexes `paths[j]` with
/// `j < dirCount`, where `dirCount` counts only the paths that produced a
/// segment (`:759` to `:766`), so a first posture with no segments would
/// make it read a chain it then calls `CSegment( -1 )` on. Both postures
/// are empty only when the corner is already at `aP`, which the callers
/// never ask for; the candidates are compacted here, which is the same
/// answer without the out of range read.
///
/// The fallback at `:813` starts the trace at the chain's **first**
/// point, throwing the whole chain away; that looks like a bug and it is
/// the shipped behaviour.
fn drag_corner_internal(
  origin: &LineChain,
  at: Vec2,
  preferred_ending_direction: Direction45,
) -> LineChain {
  let trace = |from: Vec2, diagonal: bool| {
    LineChain::from_points(
      Direction45::default().build_initial_trace(
        from,
        at,
        diagonal,
        CornerMode::Mitered45,
      ),
      false,
    )
  };

  // :729. KiCad asserts a non empty chain and then reads point 0.
  if origin.point_count() == 0 {
    return LineChain::new();
  }

  // :731
  if origin.point_count() == 1 {
    return trace(origin.point(0), false);
  }

  // :735
  if origin.segment_count() == 1 {
    let direction = Direction45::from_seg(&origin.segment(0), false);

    return trace(origin.point(0), direction.is_diagonal());
  }

  // :745. `d` is assigned 1 unconditionally one line below its
  // initialiser, so the loop always starts at the last segment.
  let mut picked: Option<(usize, LineChain)> = None;

  for index in (0..origin.segment_count()).rev() {
    let d_start = Direction45::from_seg(&origin.segment(index), false);
    let p_start = origin.point(index);
    let d_prev = if index > 0 {
      Direction45::from_seg(&origin.segment(index - 1), false)
    } else {
      Direction45::default()
    };

    // :755
    let mut candidates: Vec<(Direction45, LineChain)> = Vec::new();

    for posture in 0..2 {
      let path = trace_from(&d_start, p_start, at, posture == 1);

      if path.segment_count() < 1 {
        continue;
      }

      candidates.push((Direction45::from_seg(&path.segment(0), false), path));
    }

    // :768. The candidate that arrives in the direction asked for.
    if preferred_ending_direction.is_defined()
      && let Some((_, path)) = candidates.iter().find(|(_, path)| {
        Direction45::from_seg(&path.segment(path.segment_count() - 1), false)
          == preferred_ending_direction
      })
    {
      picked = Some((index, path.clone()));

      break;
    }

    // :784. The candidate that leaves the way the replaced segment did.
    if let Some((_, path)) = candidates
      .iter()
      .find(|(direction, _)| *direction == d_start)
    {
      picked = Some((index, path.clone()));

      break;
    }

    // :797. Otherwise the one that turns obtusely off the segment before.
    if let Some((_, path)) = candidates
      .iter()
      .find(|(direction, _)| direction.is_obtuse(d_prev))
    {
      picked = Some((index, path.clone()));

      break;
    }
  }

  // :806
  if let Some((index, tail)) = picked {
    let mut path = origin.slice(0, index).unwrap_or_default();

    path.append_chain(&tail);

    return path;
  }

  // :813
  let last = origin.point_count() - 1;
  let direction = Direction45::from_vector(
    origin.point(last) - origin.point(last - 1),
    false,
  );

  trace(origin.point(0), direction.is_diagonal())
}

/// One `BuildInitialTrace` call of [`drag_corner_internal`].
///
/// `pcbnew/router/pns_line.cpp:757`, on the direction of the segment
/// being replaced rather than on a default constructed one.
fn trace_from(
  direction: &Direction45,
  from: Vec2,
  to: Vec2,
  diagonal: bool,
) -> LineChain {
  LineChain::from_points(
    direction.build_initial_trace(from, to, diagonal, CornerMode::Mitered45),
    false,
  )
}

impl Line {
  /// Bend the line around one hull.
  ///
  /// Port of `LINE::Walkaround( const SHAPE_LINE_CHAIN& aObstacle,
  /// SHAPE_LINE_CHAIN& aPath, bool aCw )`,
  /// `pcbnew/router/pns_line.cpp:297`. It builds a directed graph over
  /// the union of the path's vertices and the hull's, then walks it from
  /// the path's first point to its last, leaving the path for the hull
  /// whenever the path would enter the obstacle. The result is the new
  /// path; `None` is KiCad's `false`, which makes the calling policy go
  /// `ST_STUCK` (note 03 section 5.3).
  ///
  /// `clockwise` selects the winding, and it is realised by **reversing
  /// the hull** rather than by walking it differently (`:399`). That
  /// relies on every hull builder emitting a clockwise chain, which
  /// `src/geometry/hull.rs` states as an invariant.
  ///
  /// # The precondition
  ///
  /// The path's first point must not be strictly inside the hull
  /// (`:308`), with "on the outline" explicitly allowed because,
  /// as the comment says, treating it as inside "triggers many
  /// unroutable corner cases". This is the precondition
  /// `LINE_PLACER::splitHeadTail` exists to maintain (note 03 section
  /// 5.4 item 1).
  ///
  /// # The five ways it fails
  ///
  /// A line of no segments (`:301`), a first point inside the hull
  /// (`:314`), the iteration limit (`:508`), no usable neighbour at an
  /// `OUTSIDE` vertex (`:556`), and a null next vertex (`:652`), which is
  /// also what an `INSIDE` vertex produces because neither branch of the
  /// dispatch selects anything for one.
  ///
  /// # Deviations
  ///
  /// The arc splice `restoreUntouchedArcs( out, pnew )` (`:661`) is left
  /// out: it returns immediately when the input has no arcs and this
  /// crate has none.
  ///
  /// The second of the three `ON_EDGE` neighbour tiers (`:578`) is dead
  /// code, in KiCad as here: it asks for a vertex that carries a hull
  /// index without being marked as a hull vertex, and the graph never
  /// builds one. It is transcribed anyway, and the differential harness
  /// confirms it never fires.
  ///
  /// The squared distance at `:630` is reproduced **with its truncation**.
  /// KiCad assigns `VECTOR2I::SquaredEuclideanNorm`, a 64 bit value, to
  /// an `int` and compares it against an `int lastDst`, so any distance
  /// past about 46 millimetres wraps. That decides when the walk stops
  /// orbiting an obstacle and projects onto the current hull edge, so it
  /// is a behaviour and not a detail; the differential harness would
  /// disagree without it.
  pub fn walkaround(
    &self,
    obstacle: &LineChain,
    clockwise: bool,
  ) -> Option<LineChain> {
    let line = &self.chain;

    // :301
    if line.segment_count() < 1 {
      return None;
    }

    // :308. Inside but not on the outline.
    let first_point = line.point(0);

    if obstacle.point_inside(first_point, 0)
      && !obstacle.point_on_edge(first_point, 0)
    {
      return None;
    }

    // :341
    let intersections = hull_intersection(obstacle, line);
    let mut pnew = line.clone();
    let mut hnew = obstacle.clone();

    // :360, the corner case for loopy tracks.
    if let Some(self_intersection) = pnew.self_intersecting()
      && Some(self_intersection.point) != pnew.last_point()
    {
      pnew.split(self_intersection.point);
    }

    // :367
    for intersection in &intersections {
      if pnew.find(intersection.point, 1).is_none() {
        pnew.split(intersection.point);
      }

      if hnew.find(intersection.point, 1).is_none() {
        hnew.split(intersection.point);
      }
    }

    // :376, split the hull wherever a path vertex sits on one of its
    // edges without being one of its vertices.
    for index in 0..pnew.point_count() {
      let point = pnew.point(index);

      if !hnew.point_on_edge(point, 0) {
        continue;
      }

      if hnew.find(point, 0).is_none() {
        hnew.split(point);
      }
    }

    // :399
    if !clockwise {
      hnew = hnew.reversed();
    }

    let path_count = pnew.point_count();
    let hull_count = hnew.point_count();
    let mut vertices: Vec<WalkVertex> =
      Vec::with_capacity(2 * (hull_count + path_count));

    // :405, classify every path vertex.
    for index in 0..path_count {
      let point = pnew.point(index);
      let on_edge = hnew.point_on_edge(point, 0);
      let inside = hnew.point_inside(point, 0);

      let vertex_type = if inside && !on_edge {
        VertexType::Inside
      } else if on_edge {
        VertexType::OnEdge
      } else {
        VertexType::Outside
      };

      vertices.push(WalkVertex {
        vertex_type,
        is_hull: false,
        pos: point,
        neighbours: Vec::new(),
        index_p: index as i32,
        index_h: -1,
        visited: false,
      });
    }

    // :430 and :436, the successor and the predecessor along the path.
    // KiCad walks the path twice, once for each direction; one pass
    // gives every vertex the same two neighbours in the same order,
    // which is what the search's first match depends on.
    for (index, vertex) in vertices.iter_mut().enumerate().take(path_count) {
      if index + 1 < path_count {
        vertex.neighbours.push(index + 1);
      }

      if index > 0 {
        vertex.neighbours.push(index - 1);
      }
    }

    // :442, merge the hull vertices in, reusing a coincident path vertex.
    for index in 0..hull_count {
      let point = hnew.point(index);

      match find_vertex(&vertices, point) {
        Some(existing) => {
          vertices[existing].is_hull = true;
          vertices[existing].index_h = index as i32;
        }
        None => vertices.push(WalkVertex {
          vertex_type: VertexType::OnEdge,
          is_hull: true,
          pos: point,
          neighbours: Vec::new(),
          index_p: -1,
          index_h: index as i32,
          visited: false,
        }),
      }
    }

    // :466, link each hull vertex to the next one around the hull. KiCad
    // reads `hnew.CPoint( i + 1 )`, whose index wraps once.
    for index in 0..hull_count {
      let next_index = if index + 1 == hull_count {
        0
      } else {
        index + 1
      };
      let current = find_vertex(&vertices, hnew.point(index));
      let next = find_vertex(&vertices, hnew.point(next_index));

      if let (Some(current), Some(next)) = (current, next) {
        vertices[current].neighbours.push(next);
      }
    }

    // :478, is the cursor sitting inside this obstacle?
    let last_point = line.point(line.point_count() - 1);
    let in_last = obstacle.point_inside(last_point, 0)
      && !obstacle.point_on_edge(last_point, 0);

    self.walk_graph(&mut vertices, path_count, hull_count, in_last, last_point)
  }

  /// The graph search half of [`Line::walkaround`].
  ///
  /// Port of `pcbnew/router/pns_line.cpp:494` to `:664`, split out only
  /// because the setup and the search have nothing to say to each other.
  /// `vertices` is the graph, `path_count` and `hull_count` are the point
  /// counts of the split path and the split hull, and `last_point` is the
  /// **original** line's last point, which is what `CLastPoint()` reads
  /// inside the loop.
  fn walk_graph(
    &self,
    vertices: &mut [WalkVertex],
    path_count: usize,
    hull_count: usize,
    in_last: bool,
    last_point: Vec2,
  ) -> Option<LineChain> {
    if vertices.is_empty() {
      return None;
    }

    let path_count = path_count as i32;
    let target = path_count - 1;
    let mut current = 0usize;
    let mut previous: Option<usize> = None;
    let mut out = LineChain::new();
    let mut iterations = WALKAROUND_ITERATION_LIMIT;
    let mut append_last = true;
    // :480, `int lastDst = INT_MAX`.
    let mut last_distance = i32::MAX;

    'walk: while vertices[current].index_p != target {
      iterations -= 1;

      // :507
      if iterations == 0 {
        return None;
      }

      // :510, a loop in the graph.
      if vertices[current].visited {
        break;
      }

      out.append(vertices[current].pos);

      let mut next: Option<usize> = None;

      match vertices[current].vertex_type {
        VertexType::Outside => {
          // :527. KiCad appends the position a second time here; the
          // chain suppresses a duplicate of its last point, so this is
          // the no op its own comment at :658 half expects.
          out.append(vertices[current].pos);

          let mut fallback: Option<usize> = None;

          for index in 0..vertices[current].neighbours.len() {
            let candidate = vertices[current].neighbours[index];

            if !are_neighbours(
              vertices[candidate].index_p,
              vertices[current].index_p,
              path_count,
            ) || vertices[candidate].vertex_type == VertexType::Inside
            {
              continue;
            }

            if !vertices[candidate].visited {
              next = Some(candidate);
              break;
            }

            if Some(candidate) != previous {
              fallback = Some(candidate);
            }
          }

          if next.is_none() {
            next = fallback;
          }

          // :551, "such a vertex must always be present, if not, bummer".
          next?;
        }
        VertexType::OnEdge => {
          // :562, first tier: any unvisited vertex outside the hull.
          for index in 0..vertices[current].neighbours.len() {
            let candidate = vertices[current].neighbours[index];

            if vertices[candidate].vertex_type == VertexType::Outside
              && !vertices[candidate].visited
            {
              next = Some(candidate);
              break;
            }
          }

          let next_hull_index = if hull_count == 0 {
            -1
          } else {
            (vertices[current].index_h + 1) % hull_count as i32
          };

          // :578, second tier: the next path vertex on the hull's
          // outline that is not itself a hull vertex. This can never
          // select anything, in KiCad or here: a vertex only receives an
          // `index_h` when it is marked `is_hull`, and the tier asks for
          // a vertex that has the one without the other. It is kept so
          // that the transcription stays line for line; the differential
          // harness confirms it never fires.
          if next.is_none() {
            for index in 0..vertices[current].neighbours.len() {
              let candidate = vertices[current].neighbours[index];

              if vertices[candidate].vertex_type == VertexType::OnEdge
                && !vertices[candidate].is_hull
                && are_neighbours(
                  vertices[candidate].index_p,
                  vertices[current].index_p,
                  path_count,
                )
                && vertices[candidate].index_h == next_hull_index
              {
                next = Some(candidate);
                break;
              }
            }
          }

          // :595, third tier: the next vertex round the hull, whatever
          // it is. Reaching this starts another lap, so every hull
          // vertex is un visited again (:614).
          if next.is_none() {
            for index in 0..vertices[current].neighbours.len() {
              let candidate = vertices[current].neighbours[index];

              if vertices[candidate].vertex_type == VertexType::OnEdge
                && vertices[candidate].index_h == next_hull_index
              {
                next = Some(candidate);
                break;
              }
            }

            if next.is_some() {
              for vertex in vertices.iter_mut() {
                if vertex.is_hull {
                  vertex.visited = false;
                }
              }
            }

            // :628. The cursor is inside this obstacle and another lap
            // would only orbit it, so as soon as the candidate stops
            // getting closer, project the cursor onto the edge being
            // walked and stop there.
            if in_last && let Some(candidate) = next {
              // :630, truncated to `int` exactly as KiCad's is; see the
              // deviation note on `Line::walkaround`.
              let distance =
                vertices[candidate].pos.squared_distance(last_point) as i32;

              if distance < last_distance {
                last_distance = distance;
              } else {
                let edge =
                  Seg::new(vertices[current].pos, vertices[candidate].pos);

                out.append(edge.nearest_point_to_point(last_point));
                append_last = false;
                break 'walk;
              }
            }
          }
        }
        // An `INSIDE` vertex matches neither branch, so nothing is
        // selected and the null check below ends the walk (:651).
        VertexType::Inside => {}
      }

      vertices[current].visited = true;
      previous = Some(current);
      current = next?;
    }

    // :655
    if append_last {
      out.append(vertices[current].pos);
    }

    // :660. `restoreUntouchedArcs` would follow; see the deviation note.
    out.simplify2(false);

    Some(out)
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::hull::octagonal_hull;
  use crate::item::{ItemBody, LayerRange, Segment, Via, ViaType};
  use crate::node::World;

  /// The net every fixture item is on.
  const NET: Option<NetId> = Some(NetId(7));

  /// The layer every fixture item is on.
  const LAYER: i32 = 0;

  /// The width every fixture segment has.
  const WIDTH: i32 = 1000;

  /// A world holding two segments that meet at `(100000, 0)`.
  ///
  /// The joint between them is a line corner, so
  /// [`World::assemble_line`] from either one produces a line of two
  /// segments; the tests here only need the two handles.
  fn two_segments() -> (World, ItemId, ItemId) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    let first =
      stored_segment(&mut world, Vec2::new(0, 0), Vec2::new(100000, 0));
    let second =
      stored_segment(&mut world, Vec2::new(100000, 0), Vec2::new(200000, 0));

    let first = world
      .add_segment(root, first, false)
      .expect("the first segment is neither degenerate nor redundant");
    let second = world
      .add_segment(root, second, false)
      .expect("the second segment is neither degenerate nor redundant");

    (world, first, second)
  }

  /// One unstored segment item with the fixture's properties.
  fn stored_segment(world: &mut World, from: Vec2, to: Vec2) -> Item {
    let body = ItemBody::Segment(Segment::new(Seg::new(from, to), WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(LAYER));
    item.set_net(NET);

    item
  }

  /// An unstored through via item at a position.
  fn via_item(world: &mut World, at: Vec2) -> Item {
    let body = ItemBody::Via(Via::new(at, 3000, 1000, ViaType::Through));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::new(0, 1));

    item
  }

  /// A line over a hand built chain, with the fixture's properties.
  fn line_over(points: &[Vec2]) -> Line {
    let mut line = Line::new();

    line.set_width(WIDTH);
    line.set_layer(LAYER);
    line.set_net(NET);
    line.set_shape(LineChain::from_slice(points, false));

    line
  }

  /// The octagon every walkaround test walks around.
  ///
  /// A hull centred on the origin, spanning `-10000` to `10000` on both
  /// axes with a 4000 nm chamfer, built by the same routine the router
  /// uses, so it is closed and clockwise.
  fn octagon() -> LineChain {
    octagonal_hull(Vec2::new(-10000, -10000), Vec2::new(20000, 20000), 0, 4000)
  }

  // -----------------------------------------------------------------
  // Construction and copying
  // -----------------------------------------------------------------

  #[test]
  fn a_line_built_from_a_segment_carries_its_properties_and_links_it() {
    let (world, first, _) = two_segments();
    let root = world.root();

    let line = Line::from_segment(&world, root, first)
      .expect("the handle names a stored segment");

    assert_eq!(line.width(), WIDTH);
    assert_eq!(line.net(), NET);
    assert_eq!(line.layers(), LayerRange::single(LAYER));
    assert_eq!(line.point_count(), 2);
    assert_eq!(line.point(0), Vec2::new(0, 0));
    assert_eq!(line.point(1), Vec2::new(100000, 0));
    assert_eq!(line.links(), [first]);
    assert_eq!(line.links_valid_in(), Some(root));
    assert!(line.is_linked_checked());
    assert_eq!(line.shape().width(), WIDTH);
  }

  #[test]
  fn a_line_over_another_lines_shape_keeps_its_links_and_drops_its_via() {
    let (mut world, first, _) = two_segments();
    let root = world.root();
    let via = via_item(&mut world, Vec2::new(100000, 0));

    let mut base = Line::from_segment(&world, root, first)
      .expect("the handle names a stored segment");
    base.append_via(via);

    let replacement = Line::with_chain(
      &base,
      LineChain::from_slice(
        &[
          Vec2::new(0, 0),
          Vec2::new(50000, 50000),
          Vec2::new(100000, 0),
        ],
        false,
      ),
    );

    // `LINK_HOLDER( aBase )` copies the links, `:90` nulls the via.
    assert_eq!(replacement.links(), [first]);
    assert!(!replacement.ends_with_via());
    assert_eq!(replacement.width(), WIDTH);
    assert_eq!(replacement.net(), NET);
    // Two shapes against one link, which is the invariant this
    // constructor cannot keep.
    assert!(!replacement.is_linked_checked());
  }

  #[test]
  fn a_lone_via_becomes_a_line_of_no_points() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let mut item = via_item(&mut world, Vec2::new(1000, 2000));
    item.set_net(NET);
    item.set_rank(4);
    let stored = world.add_via(root, item);

    let item = world.item(stored).expect("the via was just added");
    let line = Line::from_linked_via(stored, item);

    assert_eq!(line.point_count(), 0);
    assert_eq!(line.width(), 3000);
    assert_eq!(line.net(), NET);
    assert_eq!(line.layers(), LayerRange::new(0, 1));
    assert_eq!(line.own_rank(), 4);
    assert_eq!(line.via().and_then(LineVia::id), Some(stored));
    // `LINE( VIA* )` stores the pointer without linking it.
    assert!(!line.is_linked());
  }

  #[test]
  fn cloning_deep_copies_an_owned_via_and_re_nets_it() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let via = via_item(&mut world, Vec2::new(100000, 0));

    let mut line = line_over(&[Vec2::new(0, 0), Vec2::new(100000, 0)]);
    line.set_net(None);
    line.append_via(via);
    // The net changes after the via was adopted, so only the copy's via
    // can pick it up; that is `pcbnew/router/pns_line.cpp:60`.
    line.set_net(NET);

    let copy = line.clone();

    let Some(LineVia::Owned(original)) = line.via() else {
      panic!("the line owns its via");
    };
    let Some(LineVia::Owned(copied)) = copy.via() else {
      panic!("the copy owns a via of its own");
    };

    assert_eq!(original.net(), None);
    assert_eq!(copied.net(), NET);

    // The copy is deep: writing through one does not reach the other.
    let mut copy = copy;
    copy.set_via_diameter(9000);

    let Some(LineVia::Owned(copied)) = copy.via() else {
      panic!("the copy still owns a via");
    };
    let ItemBody::Via(copied) = copied.body() else {
      panic!("an owned line via is a via");
    };
    let Some(LineVia::Owned(original)) = line.via() else {
      panic!("the line still owns its via");
    };
    let ItemBody::Via(original) = original.body() else {
      panic!("an owned line via is a via");
    };

    assert_eq!(copied.diameter(LayerRange::new(0, 1), 0), 9000);
    assert_eq!(original.diameter(LayerRange::new(0, 1), 0), 3000);
  }

  #[test]
  fn cloning_aliases_a_linked_via() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let item = via_item(&mut world, Vec2::new(100000, 0));
    let stored = world.add_via(root, item);

    let mut line = line_over(&[Vec2::new(0, 0), Vec2::new(100000, 0)]);
    line.link_via(stored, Vec2::new(100000, 0));

    let copy = line.clone();

    assert_eq!(copy.via().and_then(LineVia::id), Some(stored));
    assert_eq!(line.via().and_then(LineVia::id), Some(stored));
    assert!(copy.contains_link(stored));
  }

  // -----------------------------------------------------------------
  // Links
  // -----------------------------------------------------------------

  #[test]
  fn linking_refuses_a_duplicate_and_unlinking_is_forgiving() {
    let (world, first, second) = two_segments();
    let root = world.root();

    let mut line = Line::from_segment(&world, root, first)
      .expect("the handle names a stored segment");

    line.link(first);
    assert_eq!(line.link_count(), 1);

    line.link(second);
    assert_eq!(line.links(), [first, second]);
    assert_eq!(line.link_at(0), Some(first));
    assert_eq!(line.link_at(-1), Some(second));
    assert_eq!(line.link_at(-3), None);
    assert_eq!(line.link_at(2), None);

    line.unlink(second);
    assert_eq!(line.links(), [first]);

    // Unlinking something that is not there is a no op, which is what
    // the `wxCHECK_MSG` guard makes of it.
    line.unlink(second);
    assert_eq!(line.links(), [first]);

    line.clear_links();
    assert!(!line.is_linked());
    assert_eq!(line.links_valid_in(), None);
  }

  #[test]
  fn a_stale_link_is_skipped_rather_than_trusted() {
    let (mut world, first, second) = two_segments();
    let root = world.root();

    let mut line = Line::from_segment(&world, root, first)
      .expect("the handle names a stored segment");
    line.link(second);

    if let Some(item) = world.item_mut(second) {
      item.set_rank(3);
    }

    world.remove(root, first);

    // The handle is still in the link list, and the arena no longer
    // knows it, so it contributes nothing.
    assert_eq!(line.link_count(), 2);
    assert_eq!(line.rank(&world), 3);
    assert!(!line.has_locked_segments(&world));
  }

  // -----------------------------------------------------------------
  // Marks and rank
  // -----------------------------------------------------------------

  #[test]
  fn marking_fans_out_over_the_links_and_unmarking_zeroes_the_line() {
    let (mut world, first, second) = two_segments();
    let root = world.root();

    let mut line = Line::from_segment(&world, root, first)
      .expect("the handle names a stored segment");
    line.link(second);

    line.mark(&mut world, MarkerFlags::HEAD);

    assert_eq!(line.own_marker(), MarkerFlags::HEAD);
    for link in [first, second] {
      let item = world.item(link).expect("the segment is still stored");
      assert_eq!(item.marker(), MarkerFlags::HEAD);
    }

    // `Marker()` ORs the links' marks on top of the line's own.
    if let Some(item) = world.item_mut(second) {
      item.mark(MarkerFlags::VIOLATION);
    }

    assert_eq!(
      line.marker(&world),
      MarkerFlags::HEAD | MarkerFlags::VIOLATION
    );

    line.unmark(&mut world, MarkerFlags::VIOLATION);

    // The links lose only the masked bit, the line loses everything.
    assert_eq!(line.own_marker(), MarkerFlags::NONE);
    let item = world.item(first).expect("the segment is still stored");
    assert_eq!(item.marker(), MarkerFlags::HEAD);
    assert_eq!(line.marker(&world), MarkerFlags::HEAD);
  }

  #[test]
  fn the_rank_is_the_smallest_over_the_links() {
    let (mut world, first, second) = two_segments();
    let root = world.root();

    let mut line = Line::from_segment(&world, root, first)
      .expect("the handle names a stored segment");
    line.link(second);

    line.set_rank(&mut world, 5);
    assert_eq!(line.rank(&world), 5);

    if let Some(item) = world.item_mut(second) {
      item.set_rank(3);
    }

    assert_eq!(line.rank(&world), 3);
    // The line's own field is untouched by the links.
    assert_eq!(line.own_rank(), 5);

    // An unlinked line answers with its own field.
    let mut loose = line_over(&[Vec2::new(0, 0), Vec2::new(1000, 0)]);
    assert_eq!(loose.rank(&world), Item::UNASSIGNED_RANK);
    loose.set_rank(&mut world, 8);
    assert_eq!(loose.rank(&world), 8);
  }

  #[test]
  fn a_locked_link_makes_the_line_report_locked_segments() {
    let (mut world, first, second) = two_segments();
    let root = world.root();

    let mut line = Line::from_segment(&world, root, first)
      .expect("the handle names a stored segment");
    line.link(second);

    assert!(!line.has_locked_segments(&world));
    assert!(!line.is_locked(&world));

    if let Some(item) = world.item_mut(second) {
      item.mark(MarkerFlags::LOCKED);
    }

    assert!(line.has_locked_segments(&world));
    // `IsLocked` goes through `Marker()`, so a locked link locks the line.
    assert!(line.is_locked(&world));
  }

  // -----------------------------------------------------------------
  // The via and the reversal
  // -----------------------------------------------------------------

  #[test]
  fn appending_a_via_at_the_first_point_reverses_the_line() {
    let (world, first, second) = two_segments();
    let root = world.root();
    let mut world = world;
    let via = via_item(&mut world, Vec2::new(0, 0));

    let mut line = Line::from_segment(&world, root, first)
      .expect("the handle names a stored segment");
    line.link(second);
    line.chain_mut().append(Vec2::new(200000, 0));

    assert_eq!(line.point(0), Vec2::new(0, 0));

    line.append_via(via);

    // The via is at the last point, and the links reversed with the
    // chain so that they stay in correspondence.
    assert_eq!(line.last_point(), Some(Vec2::new(0, 0)));
    assert_eq!(line.via_pos(&world), Some(Vec2::new(0, 0)));
    assert_eq!(line.links(), [second, first]);
    assert_eq!(line.point(0), Vec2::new(200000, 0));
  }

  #[test]
  fn a_via_at_the_last_point_leaves_the_line_alone() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let via = via_item(&mut world, Vec2::new(100000, 0));

    let mut line = line_over(&[Vec2::new(0, 0), Vec2::new(100000, 0)]);
    line.append_via(via);

    assert_eq!(line.point(0), Vec2::new(0, 0));
    assert!(line.ends_with_via());
    assert_eq!(line.via_pos(&world), Some(Vec2::new(100000, 0)));
  }

  #[test]
  fn a_linked_via_is_unlinked_again_when_it_is_removed() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let item = via_item(&mut world, Vec2::new(100000, 0));
    let stored = world.add_via(root, item);

    let mut line = line_over(&[Vec2::new(0, 0), Vec2::new(100000, 0)]);
    line.link_via(stored, Vec2::new(100000, 0));

    assert!(line.contains_link(stored));

    line.remove_via();

    assert!(!line.ends_with_via());
    assert!(!line.contains_link(stored));
    // The via itself is still in the node.
    assert!(world.item(stored).is_some());
  }

  #[test]
  fn the_via_reaches_the_collision_code_as_stored_or_unstored() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let item = via_item(&mut world, Vec2::new(100000, 0));
    let stored = world.add_via(root, item);

    let mut linked = line_over(&[Vec2::new(0, 0), Vec2::new(100000, 0)]);
    linked.link_via(stored, Vec2::new(100000, 0));

    let reference = linked.via_item(&world).expect("the via is stored");
    assert_eq!(reference.id(), Some(stored));

    let owned_via = via_item(&mut world, Vec2::new(100000, 0));
    let mut owned = line_over(&[Vec2::new(0, 0), Vec2::new(100000, 0)]);
    owned.append_via(owned_via);

    let reference = owned.via_item(&world).expect("the via is owned");
    assert_eq!(reference.id(), None);
  }

  // -----------------------------------------------------------------
  // Geometry
  // -----------------------------------------------------------------

  #[test]
  fn count_corners_classifies_a_hand_built_45_degree_chain() {
    let line = line_over(&[
      Vec2::new(0, 0),
      Vec2::new(100000, 0),
      Vec2::new(200000, 100000),
      Vec2::new(300000, 100000),
    ]);

    assert_eq!(line.count_corners(AngleType::OBTUSE), 2);
    assert_eq!(line.count_corners(AngleType::RIGHT), 0);
    assert_eq!(line.count_corners(AngleType::STRAIGHT), 0);

    let square = line_over(&[
      Vec2::new(0, 0),
      Vec2::new(100000, 0),
      Vec2::new(100000, 100000),
    ]);

    assert_eq!(square.count_corners(AngleType::RIGHT), 1);
    assert_eq!(
      square.count_corners(AngleType::RIGHT | AngleType::OBTUSE),
      1
    );

    let straight =
      line_over(&[Vec2::new(0, 0), Vec2::new(1000, 0), Vec2::new(2000, 0)]);
    assert_eq!(straight.count_corners(AngleType::STRAIGHT), 1);
  }

  #[test]
  fn has_loops_finds_a_repeated_vertex() {
    let open = line_over(&[
      Vec2::new(0, 0),
      Vec2::new(100000, 0),
      Vec2::new(100000, 100000),
    ]);
    assert!(!open.has_loops());

    let looped = line_over(&[
      Vec2::new(0, 0),
      Vec2::new(100000, 0),
      Vec2::new(100000, 100000),
      Vec2::new(0, 0),
    ]);
    assert!(looped.has_loops());
  }

  #[test]
  fn anchors_are_the_two_ends() {
    let line =
      line_over(&[Vec2::new(0, 0), Vec2::new(100000, 0), Vec2::new(200000, 0)]);

    assert_eq!(line.anchor_count(), 2);
    assert_eq!(line.anchor(0), Vec2::new(0, 0));
    assert_eq!(line.anchor(1), Vec2::new(200000, 0));

    let empty = Line::new();
    assert_eq!(empty.anchor_count(), 0);
    assert_eq!(empty.anchor(0), Vec2::new(0, 0));
  }

  #[test]
  fn changed_area_covers_only_the_part_that_moved() {
    let first =
      line_over(&[Vec2::new(0, 0), Vec2::new(100000, 0), Vec2::new(200000, 0)]);
    let second = line_over(&[
      Vec2::new(0, 0),
      Vec2::new(100000, 50000),
      Vec2::new(200000, 0),
    ]);

    assert_eq!(first.changed_area(&first), None);

    // `Simplify()` runs first, so the straight line collapses to its two
    // ends and only the vertex that moved is left to cover.
    let area = first.changed_area(&second).expect("the middle moved");

    assert!(area.contains_point(Vec2::new(100000, 50000).into()));
    assert!(!area.contains_point(Vec2::new(0, 0).into()));
    assert_eq!(area.top(), 50000 - i64::from(WIDTH));
    assert_eq!(area.bottom(), 50000 + i64::from(WIDTH));
  }

  // -----------------------------------------------------------------
  // Walkaround
  // -----------------------------------------------------------------

  /// Whether no point of a walk is strictly inside the hull.
  fn stays_outside(hull: &LineChain, walk: &LineChain) -> bool {
    (0..walk.point_count()).all(|index| {
      let point = walk.point(index);

      !hull.point_inside(point, 0) || hull.point_on_edge(point, 0)
    })
  }

  #[test]
  fn a_line_that_misses_the_hull_comes_back_unchanged() {
    let hull = octagon();
    let line = line_over(&[Vec2::new(-30000, 50000), Vec2::new(30000, 50000)]);

    for clockwise in [true, false] {
      let walk = line
        .walkaround(&hull, clockwise)
        .expect("a line clear of the hull always walks");

      assert_eq!(walk.points(), line.shape().points());
    }
  }

  #[test]
  fn a_line_across_the_hull_walks_round_it_in_both_windings() {
    let hull = octagon();
    let line = line_over(&[Vec2::new(-30000, 0), Vec2::new(30000, 0)]);

    let clockwise = line
      .walkaround(&hull, true)
      .expect("the clockwise walk succeeds");
    let counter = line
      .walkaround(&hull, false)
      .expect("the counter clockwise walk succeeds");

    for walk in [&clockwise, &counter] {
      assert_eq!(walk.point(0), Vec2::new(-30000, 0));
      assert_eq!(walk.last_point(), Some(Vec2::new(30000, 0)));
      assert!(walk.point_count() > 2, "the walk has to detour");
      assert!(stays_outside(&hull, walk));
    }

    // The two windings go round opposite sides of the obstacle.
    let clockwise_below =
      (0..clockwise.point_count()).any(|index| clockwise.point(index).y > 0);
    let counter_below =
      (0..counter.point_count()).any(|index| counter.point(index).y > 0);

    assert_ne!(clockwise_below, counter_below);
  }

  #[test]
  fn a_line_through_a_hull_vertex_still_walks() {
    let hull = octagon();
    let corner = hull.point(0);
    let line = line_over(&[
      Vec2::new(corner.x - 20000, corner.y),
      Vec2::new(corner.x + 20000, corner.y),
    ]);

    for clockwise in [true, false] {
      let walk = line
        .walkaround(&hull, clockwise)
        .expect("touching a vertex is not a failure");

      assert_eq!(walk.point(0), line.point(0));
      assert!(stays_outside(&hull, &walk));
    }
  }

  #[test]
  fn a_line_that_starts_inside_the_hull_refuses_to_walk() {
    let hull = octagon();
    let line = line_over(&[Vec2::new(0, 0), Vec2::new(30000, 0)]);

    assert!(hull.point_inside(Vec2::new(0, 0), 0));
    assert!(!hull.point_on_edge(Vec2::new(0, 0), 0));

    assert_eq!(line.walkaround(&hull, true), None);
    assert_eq!(line.walkaround(&hull, false), None);
  }

  #[test]
  fn a_line_of_no_segments_refuses_to_walk() {
    let hull = octagon();
    let empty = Line::new();
    let single = line_over(&[Vec2::new(-30000, 0)]);

    assert_eq!(empty.walkaround(&hull, true), None);
    assert_eq!(single.walkaround(&hull, true), None);
  }

  #[test]
  fn a_line_ending_inside_the_hull_stops_on_its_boundary() {
    let hull = octagon();
    let line = line_over(&[Vec2::new(-30000, 0), Vec2::new(0, 0)]);

    let walk = line
      .walkaround(&hull, true)
      .expect("a cursor inside the obstacle still produces a path");

    assert_eq!(walk.point(0), Vec2::new(-30000, 0));
    assert!(stays_outside(&hull, &walk));
    // It does not reach the requested endpoint, which is what makes the
    // calling policy report `ST_ALMOST_DONE`.
    assert_ne!(walk.last_point(), Some(Vec2::new(0, 0)));
  }

  #[test]
  fn every_answer_is_the_same_across_two_identical_runs() {
    let hull = octagon();
    let cases = [
      line_over(&[Vec2::new(-30000, 0), Vec2::new(30000, 0)]),
      line_over(&[Vec2::new(-30000, -30000), Vec2::new(30000, 30000)]),
      line_over(&[
        Vec2::new(-30000, 5000),
        Vec2::new(0, 5000),
        Vec2::new(30000, -20000),
      ]),
      line_over(&[Vec2::new(-30000, 0), Vec2::new(0, 0)]),
    ];

    for line in &cases {
      for clockwise in [true, false] {
        let first = line.walkaround(&hull, clockwise);
        let second = line.walkaround(&hull, clockwise);

        assert_eq!(first, second);
      }
    }
  }

  #[test]
  fn the_rule_item_speaks_for_the_whole_line() {
    let (world, first, _) = two_segments();
    let root = world.root();
    let line = world.assemble_line(root, first, None, false, false, true);
    let stand_in = line.rule_item(&world, 11);
    let segment = line.segment_item(&world, 0, 12);

    // Everything a rule can read matches the line, and matches what one
    // of its segments would have said.
    assert_eq!(stand_in.uid(), 11);
    assert_eq!(stand_in.net(), line.net());
    assert_eq!(stand_in.layers(), line.layers());
    assert_eq!(stand_in.net(), segment.net());
    assert_eq!(stand_in.layers(), segment.layers());
    assert_eq!(stand_in.rank(), segment.rank());
    assert_eq!(stand_in.marker(), segment.marker());
    // It owns nothing, so it can never prune a hole by parentage.
    assert_eq!(stand_in.hole(), None);
    assert_eq!(stand_in.parent_pad_via(), None);

    // Its geometry is the placeholder the documentation warns about: the
    // straight segment from the first point to the last, which is not
    // the chain.
    let ItemBody::Segment(body) = stand_in.body() else {
      panic!("the stand in is a segment");
    };

    assert_eq!(body.seg(), Seg::new(line.point(0), Vec2::new(200000, 0)));
    assert_eq!(body.width(), line.width());
  }

  #[test]
  fn the_rule_item_of_an_empty_line_is_degenerate() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let line = Line::new();
    let stand_in = line.rule_item(&world, 0);

    let ItemBody::Segment(body) = stand_in.body() else {
      panic!("the stand in is a segment");
    };

    assert_eq!(body.seg(), Seg::new(Vec2::new(0, 0), Vec2::new(0, 0)));
  }

  /// An L shaped chain: east then north east, which is what the fanout of
  /// a via looks like once it has been reversed so the via end is last.
  fn dragged_line() -> Line {
    let mut line = Line::new();

    line.set_width(WIDTH);
    line.set_shape(LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(100000, 0),
        Vec2::new(200000, 100000),
      ],
      false,
    ));

    line
  }

  #[test]
  fn dragging_the_last_corner_keeps_the_first_point_and_the_grid() {
    let mut line = dragged_line();
    let target = Vec2::new(250000, 200000);

    line.drag_corner(
      target,
      line.point_count() - 1,
      false,
      Direction45::default(),
    );

    assert_eq!(line.point(0), Vec2::new(0, 0));
    assert_eq!(line.last_point(), Some(target));
    assert_eq!(line.width(), WIDTH);

    for index in 0..line.segment_count() {
      let seg = line.segment(index);
      let delta = seg.b - seg.a;

      assert!(
        delta.x == 0 || delta.y == 0 || delta.x.abs() == delta.y.abs(),
        "segment {index} left the 45 degree grid"
      );
    }
  }

  #[test]
  fn dragging_the_first_corner_keeps_the_last_point() {
    let mut line = dragged_line();
    let target = Vec2::new(-100000, -100000);

    line.drag_corner(target, 0, false, Direction45::default());

    assert_eq!(line.point(0), target);
    assert_eq!(line.last_point(), Some(Vec2::new(200000, 100000)));
  }

  #[test]
  fn dragging_a_corner_past_the_end_leaves_the_line_alone() {
    let mut line = dragged_line();
    let before = line.shape().points().to_vec();

    line.drag_corner(
      Vec2::new(1, 1),
      line.point_count(),
      false,
      Direction45::default(),
    );

    assert_eq!(line.shape().points().to_vec(), before);
  }

  #[test]
  fn a_snap_threshold_of_zero_leaves_the_dragged_point_where_it_was() {
    // `pns_line.cpp:1153`, the short circuit every line this crate builds
    // takes unless a host sets a threshold.
    let line = dragged_line();
    let at = Vec2::new(123, 456);

    assert_eq!(line.snap_dragged_corner(at, 1), at);
  }

  /// A straight run of three horizontal segments, one nanometre grid
  /// aligned, whose middle segment is long enough to survive a sideways
  /// drag without collapsing into a corner.
  fn straight_three_segment_line() -> Line {
    let mut line = Line::new();

    line.set_width(WIDTH);
    line.set_shape(LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(1_000_000, 0),
        Vec2::new(3_000_000, 0),
        Vec2::new(4_000_000, 0),
      ],
      false,
    ));

    line
  }

  /// Whether every segment of a line sits on the 45 degree grid.
  fn is_on_the_grid(line: &Line) -> bool {
    (0..line.segment_count()).all(|index| {
      let delta = line.segment(index).b - line.segment(index).a;

      delta.x == 0 || delta.y == 0 || delta.x.abs() == delta.y.abs()
    })
  }

  #[test]
  fn dragging_the_middle_segment_sideways_keeps_both_ends() {
    let mut line = straight_three_segment_line();

    line.drag_segment(Vec2::new(2_000_000, -500_000), 1);

    assert_eq!(line.point(0), Vec2::new(0, 0));
    assert_eq!(line.last_point(), Some(Vec2::new(4_000_000, 0)));
    assert_eq!(line.width(), WIDTH);
  }

  #[test]
  fn a_dragged_middle_segment_moves_to_the_cursor_and_keeps_45_degrees() {
    let mut line = straight_three_segment_line();

    line.drag_segment(Vec2::new(2_000_000, -500_000), 1);

    assert!(is_on_the_grid(&line), "the drag left the 45 degree grid");

    // The two zero length neighbours `dragSegment45` inserts at `:1271`
    // and `:1282` become the two 45 degree ramps down to the new height,
    // so the dragged segment is the only horizontal run at that height.
    let moved: Vec<Seg> = (0..line.segment_count())
      .map(|index| line.segment(index))
      .filter(|seg| seg.a.y == -500_000 && seg.b.y == -500_000)
      .collect();

    assert_eq!(moved.len(), 1, "expected exactly one segment to have moved");
    assert!(
      moved[0].a.x < moved[0].b.x && moved[0].length() > 0,
      "the dragged segment collapsed"
    );
  }

  #[test]
  fn dragging_a_segment_index_past_the_end_leaves_the_line_alone() {
    let mut line = straight_three_segment_line();
    let before = line.shape().points().to_vec();

    line.drag_segment(Vec2::new(1, 1), line.segment_count());

    assert_eq!(line.shape().points().to_vec(), before);
  }

  /// A chain whose segment 1 and segment 3 are both due east, which is
  /// what `snapToNeighbourSegments` looks for at `index + 2`
  /// (`pns_line.cpp:1205`).
  fn line_with_a_parallel_neighbour() -> Line {
    let mut line = Line::new();

    line.set_width(WIDTH);
    line.set_shape(LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(1_000_000, 1_000_000),
        Vec2::new(3_000_000, 1_000_000),
        Vec2::new(4_000_000, 0),
        Vec2::new(6_000_000, 0),
      ],
      false,
    ));

    line
  }

  #[test]
  fn the_neighbour_snapper_short_circuits_on_a_zero_threshold() {
    // `pns_line.cpp:1192`, which is `== 0` where the corner snapper is
    // `<= 0` (`:1153`).
    let line = line_with_a_parallel_neighbour();
    let at = Vec2::new(2_000_000, -30_000);

    assert_eq!(line.snap_threshold(), 0);
    assert_eq!(line.snap_to_neighbour_segments(at, 1), at);
  }

  #[test]
  fn the_neighbour_snapper_takes_a_distance_equal_to_the_threshold() {
    // `:1220` is `snap_d[i] <= m_snapThreshhold`, so the boundary snaps.
    let mut line = line_with_a_parallel_neighbour();

    line.set_snap_threshold(30_000);

    let at = Vec2::new(2_000_000, -30_000);

    assert_eq!(
      line.snap_to_neighbour_segments(at, 1),
      Vec2::new(4_000_000, 0),
      "the answer is `s.A` of the neighbour, not a projection of `at`"
    );
  }

  #[test]
  fn the_neighbour_snapper_refuses_one_nanometre_beyond_the_threshold() {
    let mut line = line_with_a_parallel_neighbour();

    line.set_snap_threshold(29_999);

    let at = Vec2::new(2_000_000, -30_000);

    assert_eq!(line.snap_to_neighbour_segments(at, 1), at);
  }

  #[test]
  fn the_neighbour_snapper_ignores_a_neighbour_of_another_direction() {
    // `:1207` compares the two directions for equality, so the segment
    // two positions away only counts when it is exactly parallel.
    let mut line = Line::new();

    line.set_width(WIDTH);
    line.set_shape(LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(1_000_000, 1_000_000),
        Vec2::new(3_000_000, 1_000_000),
        Vec2::new(4_000_000, 0),
        Vec2::new(4_000_000, -2_000_000),
      ],
      false,
    ));
    line.set_snap_threshold(2_000_000);

    let at = Vec2::new(2_000_000, -30_000);

    assert_eq!(line.snap_to_neighbour_segments(at, 1), at);
  }

  #[test]
  fn a_free_angle_corner_drag_puts_the_point_exactly_where_it_was_asked() {
    // `dragCornerFree`, `pns_line.cpp:857`: no 45 degree rebuild at all.
    let mut line = dragged_line();
    let target = Vec2::new(123_456, -654_321);

    line.drag_corner(target, 1, true, Direction45::default());

    assert_eq!(line.point(0), Vec2::new(0, 0));
    assert_eq!(line.point(1), target);
    assert_eq!(line.last_point(), Some(Vec2::new(200000, 100000)));
  }
}
