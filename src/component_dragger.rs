// SPDX-License-Identifier: GPL-3.0-or-later

//! Dragging a footprint and the traces hanging off its pads.
//!
//! Port of `PNS::COMPONENT_DRAGGER`
//! (`pcbnew/router/pns_component_dragger.h:39`,
//! `pcbnew/router/pns_component_dragger.cpp`). Every `// :NNNN` citation
//! in this module is a line of `pcbnew/router/pns_component_dragger.cpp`
//! unless another file is named. The reference note is
//! `doc/reference/kicad/06-dragger.md` section 11.
//!
//! The mechanism in one sentence: **clone every selected pad at the
//! cursor offset, move rigidly anything that runs between two selected
//! pads, and drag one corner of every other attached trace to where its
//! pad end went.**
//!
//! # What is transcribed
//!
//! The whole file, which is short:
//!
//! - [`ComponentDragger::start`] (`:48`) with its `addLinked` closure
//!   (`:58`), the two "runs between two dragged pads" cases and the
//!   unconnected trace end lookup;
//! - [`ComponentDragger::drag`] (`:156`), which rebuilds its one branch
//!   from scratch on every mouse move;
//! - [`ComponentDragger::fix_route`] and
//!   [`ComponentDragger::fix_route_node`] (`:247`);
//! - [`ComponentDragger::current_node`] (`:264`),
//!   [`ComponentDragger::traces`] and
//!   [`ComponentDragger::traces_items`] (`:270`),
//!   [`ComponentDragger::current_nets`] and
//!   [`ComponentDragger::current_layer`]
//!   (`pcbnew/router/pns_component_dragger.h:85`, `:96`) and
//!   [`ComponentDragger::force_mark_obstacles_mode`] (`:113`).
//!
//! # What a component drag is not
//!
//! There is no shove, no walkaround, no optimizer and no mode. The
//! routing mode is never read, so a component drag behaves like
//! [`crate::settings::RouterMode::MarkObstacles`] whatever the session is
//! set to, and the only collision test in the whole algorithm is the one
//! [`ComponentDragger::fix_route_node`] runs at commit time. The dead
//! `class OPTIMIZER;` forward declaration at
//! `pcbnew/router/pns_component_dragger.h:32` is the trace of an
//! intention that was never carried out (note 06 erratum E30).
//!
//! [`ComponentDragger::drag`] re-derives everything from what
//! [`ComponentDragger::start`] recorded rather than from the previous
//! drag, so a component drag never accumulates and never needs a restore
//! branch.
//!
//! # The errata, one by one
//!
//! Note 06 section 11.6 lists eight, E29 to E36. Four are reproduced,
//! three are repaired because `DESIGN.md` forbids them or because the
//! repair costs nothing, and one has no counterpart:
//!
//! - **E29**, `m_dragStatus` never assigned after the constructor:
//!   reproduced, see [`ComponentDragger::force_mark_obstacles_mode`];
//! - **E30**, no failure path and no mode: reproduced, see
//!   [`ComponentDragger::drag`];
//! - **E31**, the two `std::set` of raw pointers: **repaired**.
//!   `ComponentDragger::solids` and `ComponentDragger::fixed_items`
//!   are uid ordered, which `DESIGN.md` section 8 requires;
//! - **E32**, `Start` not clearing its three collections: **repaired**,
//!   [`ComponentDragger::start`] clears them, so a second start on one
//!   instance behaves;
//! - **E33**, counting solid links and then iterating every link:
//!   **repaired**, `links_hold_a_dragged_pad` filters by
//!   [`Kind::SOLID`], which is what the count promised;
//! - **E34**, `Find` answering `-1` and `DragCorner`'s `wxCHECK_RET`
//!   swallowing it: reproduced as an [`Option`], see
//!   [`ComponentDragger::drag`];
//! - **E35**, the `wxASSERT` `jSearch` rests on: reproduced as a
//!   `debug_assert!`;
//! - **E36**, the unconnected trace end block asking for the same net and
//!   a collision at once, which a same net pair can never give:
//!   reproduced, see [`ComponentDragger::start`].
//!
//! # The node tree
//!
//! ```text
//! world root                  handed in to ComponentDragger::new
//!  +-- current_node            world.branch(root), rebuilt on every drag
//! ```
//!
//! One level, where [`crate::dragger::Dragger`] has two, because there is
//! no shove to stand a pre drag node on. `CurrentNode()` is
//! `m_currentNode ? m_currentNode : m_world` (`:266`), so before the
//! first drag the dragger reports the untouched board.

use crate::algo_base::AlgoContext;
use crate::collide::{CollisionSearchOptions, collide_items};
use crate::geometry::direction45::Direction45;
use crate::geometry::vec2::Vec2;
use crate::item::{Item, ItemBody, ItemId, Kind, MarkerFlags, NetId};
use crate::line::Line;
use crate::node::{JointRef, NodeId, World};
use crate::rules::ItemRef;

// ---------------------------------------------------------------------
// The records
// ---------------------------------------------------------------------

/// One trace that hangs off a dragged pad and gets one corner moved.
///
/// Port of `COMPONENT_DRAGGER::DRAGGED_CONNECTION`
/// (`pcbnew/router/pns_component_dragger.h:120`).
#[derive(Clone, PartialEq, Debug)]
struct DraggedConnection {
  /// The line as the board has it, with its links. Port of `origLine`
  /// (`:122`). Every drag starts from this; it is never re-dragged.
  orig_line: Line,
  /// The pad the line hangs off. Port of `attachedPad` (`:123`).
  attached_pad: ItemId,
  /// Where the moving end is now. Port of `p_orig` (`:124`), written by
  /// [`ComponentDragger::drag`] alone (`:184`).
  p_orig: Vec2,
  /// Where the moving end has to go. Port of `p_next` (`:124`), written
  /// by [`ComponentDragger::drag`] alone (`:185`).
  p_next: Vec2,
  /// The distance from the pad centre to the moving end. Port of
  /// `offset` (`:125`).
  ///
  /// Zero for a trace that is jointed to the pad, and the real distance
  /// for one that merely ends inside it; see
  /// [`ComponentDragger::start`].
  offset: Vec2,
}

/// One pad a component drag is moving.
///
/// There is no KiCad counterpart on the dragger side: the board sees a
/// removal and an addition like any other item, and
/// `PNS_KICAD_IFACE::RemoveItem` / `createBoardItem` intercept the pair
/// into `m_fpOffsets` (`pcbnew/router/pns_kicad_iface.cpp:2634`,
/// `:2854`) so that `Commit()` can move the pad's **footprint** instead
/// (`:2918`). This is the value that pair carries, computed where it is
/// known rather than reconstructed from the diff.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct MovedSolid {
  /// The solid as the board has it, in the world the drag branched from.
  pub before: ItemId,
  /// The solid the drag put in its place, in
  /// [`ComponentDragger::current_node`].
  pub after: ItemId,
  /// How far it moved, which is the cursor's own displacement.
  pub offset: Vec2,
}

// ---------------------------------------------------------------------
// The dragger
// ---------------------------------------------------------------------

/// A drag of one or more footprints, by their pads.
///
/// Port of `COMPONENT_DRAGGER`
/// (`pcbnew/router/pns_component_dragger.h:39`). One is built per
/// gesture, exactly like [`crate::dragger::Dragger`] and
/// [`crate::multi_dragger::MultiDragger`].
///
/// `ROUTER::StartDragging` picks it from the **shape of the item set**
/// and not from a drag mode (`pcbnew/router/pns_router.cpp:176`): a set
/// of nothing but solids is a component drag, and that test wins over the
/// multi drag test below it.
pub struct ComponentDragger {
  /// The node the drag branches from, KiCad's `m_world`
  /// (`pcbnew/router/pns_drag_algo.h:128`), which
  /// `ROUTER::StartDragging` fills with the router's **root**
  /// (`pcbnew/router/pns_router.cpp:194`).
  world_node: NodeId,
  /// The pads being dragged. `m_solids` (`.h:128`).
  ///
  /// Uid ordered, where KiCad's `std::set<SOLID*>` is address ordered
  /// (note 06 erratum E31).
  solids: Vec<ItemId>,
  /// Segments that move rigidly with the pads. `m_fixedItems` (`.h:129`).
  ///
  /// Uid ordered, for the same reason. KiCad's set can also hold arcs
  /// (`:210`); this crate has none, so anything that is not a segment is
  /// skipped where KiCad reaches its `wxFAIL_MSG` (`:224`).
  fixed_items: Vec<ItemId>,
  /// The traces that get one corner dragged. `m_conns` (`.h:130`).
  conns: Vec<DraggedConnection>,
  /// Whether the last drag position is legal. `m_dragStatus` (`.h:132`).
  ///
  /// Never written after construction, which is note 06 erratum E29 and
  /// is reproduced; see [`ComponentDragger::force_mark_obstacles_mode`].
  drag_status: bool,
  /// The line half of what [`ComponentDragger::traces`] answers, the
  /// re-dragged fanout. Part of `m_draggedItems` (`.h:133`).
  dragged_items: Vec<Line>,
  /// The stored item half of `m_draggedItems`: the cloned pads and the
  /// cloned rigid segments, as [`ComponentDragger::traces_items`].
  ///
  /// KiCad's `ITEM_SET` holds lines and items in one vector; a [`Line`]
  /// is not an [`Item`] here, so the set is two vectors that
  /// [`ComponentDragger::clear_dragged_items`] always empties as one.
  /// This is the same split [`crate::dragger::Dragger::traces_vias`]
  /// makes.
  dragged_solids: Vec<ItemId>,
  /// One entry per cloned pad, as [`ComponentDragger::moved_solids`].
  moved_solids: Vec<MovedSolid>,
  /// The primitives as they arrived. `m_initialDraggedItems` (`.h:134`),
  /// removed from the fresh branch at the top of every drag (`:163`).
  initial_dragged_items: Vec<ItemId>,
  /// The one branch, rebuilt on every drag. `m_currentNode` (`.h:135`).
  current_node: Option<NodeId>,
  /// Where the gesture started. `m_p0` (`.h:136`).
  p0: Vec2,
}

impl ComponentDragger {
  // -----------------------------------------------------------------
  // Construction
  // -----------------------------------------------------------------

  /// A component dragger over one node.
  ///
  /// The constructor (`:35`) plus `SetWorld`
  /// (`pcbnew/router/pns_drag_algo.h:61`).
  ///
  /// # Panics
  ///
  /// When `node` is not a live node of `world`, which is the same guard
  /// [`crate::dragger::Dragger::new`] applies.
  pub fn new(world: &World, node: NodeId) -> Self {
    assert!(
      world.node(node).is_some(),
      "a component dragger needs a live node to branch from"
    );

    Self {
      world_node: node,
      solids: Vec::new(),
      fixed_items: Vec::new(),
      conns: Vec::new(),
      // :38
      drag_status: false,
      dragged_items: Vec::new(),
      dragged_solids: Vec::new(),
      moved_solids: Vec::new(),
      initial_dragged_items: Vec::new(),
      // :39
      current_node: None,
      p0: Vec2::new(0, 0),
    }
  }

  // -----------------------------------------------------------------
  // Accessors
  // -----------------------------------------------------------------

  /// The node holding everything the drag has changed.
  ///
  /// Port of `CurrentNode` (`:264`): the current branch, or the world the
  /// dragger was given when no drag has happened yet.
  pub fn current_node(&self) -> NodeId {
    self.current_node.unwrap_or(self.world_node)
  }

  /// The traces the drag is re-shaping.
  ///
  /// The line half of `Traces()` (`:270`). KiCad copies each line into
  /// its `ITEM_SET` **before** the node links it (`:236` before `:240`,
  /// and `ITEM_SET::Add( const LINE& )` clones,
  /// `pcbnew/router/pns_itemset.cpp:36`), so what a caller reads back is
  /// an unlinked snapshot. The same is true here.
  pub fn traces(&self) -> &[Line] {
    &self.dragged_items
  }

  /// The stored items the drag is moving: the cloned pads first, then
  /// the segments that move rigidly with them.
  ///
  /// The other half of `Traces()` (`:270`). `ROUTER::markViolations`
  /// reads the whole set to skip what the drag is moving
  /// (`pcbnew/router/pns_router.cpp:726`), which is the one place the
  /// distinction is observable.
  pub fn traces_items(&self) -> &[ItemId] {
    &self.dragged_solids
  }

  /// The pads the drag has moved, and by how far.
  ///
  /// What `PNS_KICAD_IFACE` reconstructs from the removal and addition
  /// pair into `m_fpOffsets` (`pcbnew/router/pns_kicad_iface.cpp:2634`,
  /// `:2854`); see [`MovedSolid`]. Empty before the first
  /// [`ComponentDragger::drag`], and in `ComponentDragger::solids`
  /// order, which is uid order.
  pub fn moved_solids(&self) -> &[MovedSolid] {
    &self.moved_solids
  }

  /// The nets the drag is on.
  ///
  /// Port of `CurrentNets`
  /// (`pcbnew/router/pns_component_dragger.h:85`), which answers an
  /// empty vector with the comment "Currently unused for component
  /// dragging". Reproduced: a component drag can touch any number of
  /// nets and KiCad does not try to name them.
  pub fn current_nets(&self) -> Vec<NetId> {
    Vec::new()
  }

  /// The layer the drag is on.
  ///
  /// Port of `CurrentLayer`
  /// (`pcbnew/router/pns_component_dragger.h:96`), which answers
  /// `UNDEFINED_LAYER` with the same "currently unused" comment. Note 06
  /// section 1.5 shows the accessor is unreachable in KiCad, since
  /// `ROUTER::GetCurrentLayer`'s only callers are in
  /// `pns_dp_meander_placer.cpp`.
  pub const fn current_layer(&self) -> i32 {
    UNDEFINED_LAYER
  }

  /// Whether the drag has fallen back to highlighting, and whether the
  /// last position is legal.
  ///
  /// Port of `GetForceMarkObstaclesMode`
  /// (`pcbnew/router/pns_component_dragger.h:113`), which writes
  /// `m_dragStatus` out and returns false. Note 06 erratum E29:
  /// `m_dragStatus` is assigned once, in the constructor, and never
  /// again, so this always answers `(false, false)` and the host never
  /// shows its "Ctrl+click to commit anyway" hint for a component drag
  /// even though [`ComponentDragger::fix_route_node`] will refuse for
  /// exactly that reason. Reproduced rather than repaired, because
  /// repairing it would mean running a collision query per mouse move
  /// that KiCad does not run.
  pub const fn force_mark_obstacles_mode(&self) -> (bool, bool) {
    (false, self.drag_status)
  }

  // -----------------------------------------------------------------
  // Start
  // -----------------------------------------------------------------

  /// Work out what the selected pads drag along with them.
  ///
  /// Port of `Start` (`:48`). It classifies and it never fails: KiCad's
  /// only `return` is `true` at `:152`, and this answers false only for
  /// the empty set, which `ROUTER::StartDragging` has already refused
  /// (`pcbnew/router/pns_router.cpp:171`).
  ///
  /// Anything that is not a solid is skipped (`:117`) rather than
  /// refused, because the host's set is not only pads: a footprint's
  /// copper zones and its board outline, margin and copper graphics all
  /// reach the router as solids too
  /// (`pcbnew/router/router_tool.cpp:2881`, `:2887`).
  ///
  /// A pad that is not routable still moves (`:124` only skips the
  /// connection search), so an unplated hole or a mask only pad is
  /// cloned at the new position and drags nothing.
  ///
  /// The `extraJoints` lookup (`:137` to `:149`) is the whole of the
  /// "the trace is not connected to the pad" handling: an end that stops
  /// inside the pad's copper, on the pad's net and layers, with exactly
  /// one link, and that actually touches the pad, is dragged along as if
  /// it were jointed to it, at the distance it had from the pad centre.
  /// Note 06 erratum E36 records how narrow "actually touches" is.
  ///
  /// Note 06 erratum E32: KiCad clears none of its three collections
  /// here and neither does its constructor, so a second start on one
  /// instance would accumulate. That is repaired, since it costs three
  /// lines and nothing depends on the accumulation.
  pub fn start(
    &mut self,
    world: &World,
    context: &AlgoContext<'_>,
    at: Vec2,
    items: &[ItemId],
  ) -> bool {
    // :52 to :54
    self.current_node = None;
    self.initial_dragged_items = items.to_vec();
    self.p0 = at;

    // Not KiCad's; see the doc comment, erratum E32.
    self.solids.clear();
    self.fixed_items.clear();
    self.conns.clear();
    self.clear_dragged_items();

    if items.is_empty() {
      return false;
    }

    // `seenItems` (`:56`), an `unordered_set` KiCad only ever asks
    // `count`, so its order is not observable. It de-duplicates the seed
    // segment and not the assembled line, which is why the same line can
    // be reached again from the pad at its other end; the second visit
    // lands in the second special case below and is idempotent.
    let mut seen: Vec<ItemId> = Vec::new();

    // :115
    for id in items {
      let Some(item) = world.item(*id) else {
        continue;
      };

      // :117
      if !item.of_kind(Kind::SOLID) {
        continue;
      }

      // :122
      self.solids.push(*id);

      // :124
      if !item.is_routable() {
        continue;
      }

      let ItemBody::Solid(solid) = item.body() else {
        continue;
      };
      let pos = solid.pos();

      // :127. The two argument `FindJoint` overload,
      // `FindJoint( aPos, aItem->Layers().Start(), aItem->Net() )`
      // (`pcbnew/router/pns_node.h:478`). KiCad dereferences the answer
      // unchecked at `:129`; what makes that safe is `:124` above,
      // because `NODE::addSolid` links a joint only for a routable solid
      // (`pcbnew/router/pns_node.cpp:609`).
      let Some(reference) =
        world.find_joint(self.world_node, pos, item.layer(), item.net())
      else {
        continue;
      };
      let Some(joint) = world.joint(reference) else {
        continue;
      };
      let net = joint.net();
      let links: Vec<ItemId> = joint.links().to_vec();

      // :129 to :133
      for link in links {
        if world
          .item(link)
          .is_some_and(|item| item.of_kind(Kind::SEGMENT | Kind::ARC))
        {
          self.add_linked(
            world, items, &mut seen, *id, reference, pos, link, ZERO,
          );
        }
      }

      // :137. Every joint inside the pad's bare copper box, on the pad's
      // layers, that a segment or an arc hangs off. `SOLID::Hull` is
      // called with all three defaults
      // (`pcbnew/router/pns_solid.h:110`), so the box is the copper's.
      let Some(bbox) = item.hull(0, 0, item.layer()).bbox(0) else {
        continue;
      };
      let extra = world.query_joints(
        self.world_node,
        bbox,
        None,
        item.layers(),
        Kind::SEGMENT | Kind::ARC,
      );

      // :140
      for candidate in extra {
        let Some(other) = world.joint(candidate) else {
          continue;
        };

        // :142. `LinkCount()` takes the default mask, so a dangling end
        // that also carries a via is not picked up.
        if other.net() != net || other.links().len() != 1 {
          continue;
        }

        let Some(link) = other.links().first().copied() else {
          continue;
        };
        let other_pos = other.pos();

        // :146. The trace end has to actually touch the pad; a joint
        // that merely sits in the bounding box is not connected to it.
        // Note 06 erratum E36: this and the net test above contradict
        // each other for an ordinary net, because a same net pair gets no
        // clearance and therefore never collides
        // (`pcbnew/router/pns_item.cpp:188`). What is left reachable is a
        // netless pad with a netless end inside it, and any board with a
        // user defined physical clearance rule. Reproduced as it stands.
        if !items_collide(world, context, link, *id) {
          continue;
        }

        // :147. The offset is what keeps the trace end in the same place
        // relative to the pad as the pad moves.
        self.add_linked(
          world,
          items,
          &mut seen,
          *id,
          candidate,
          other_pos,
          link,
          other_pos - pos,
        );
      }
    }

    // KiCad's two collections are `std::set`s, so they are ordered and
    // de-duplicated by construction (`.h:128`, `:129`). Both properties
    // matter: the order is note 06 erratum E31, and the duplicates are
    // real, because a run of segments between two dragged pads is
    // reached once from each end and the second visit inserts every one
    // of its links again. Without the `dedup` a rigid segment would be
    // cloned twice into the drag node.
    self.solids.sort_by_key(|id| uid_of(world, *id));
    self.solids.dedup();
    self.fixed_items.sort_by_key(|id| uid_of(world, *id));
    self.fixed_items.dedup();

    // :152
    true
  }

  /// Classify one segment that hangs off a dragged pad.
  ///
  /// Port of the `addLinked` closure (`:58` to `:113`), which is where
  /// the two "runs between two dragged pads" cases live. Both are the
  /// same idea at two granularities: the first catches one segment whose
  /// far anchor carries a dragged pad, the second catches a run of
  /// segments whose far **joint** does, and the second puts **every**
  /// link of the run into [`ComponentDragger::fixed_items`] rather than
  /// just the seed. A trace between two dragged pads therefore
  /// translates rigidly instead of being re-shaped, which is the only
  /// way to keep it straight when both of its ends move by the same
  /// vector.
  ///
  /// `joint_pos` is the position of the joint `segment` was reached
  /// through, which is the pad's own position for a jointed trace and
  /// the dangling end's for one that merely stops inside the pad.
  #[expect(
    clippy::too_many_arguments,
    reason = "KiCad's closure captures four of these; naming them is \
              clearer than a context struct used once"
  )]
  fn add_linked(
    &mut self,
    world: &World,
    primitives: &[ItemId],
    seen: &mut Vec<ItemId>,
    solid: ItemId,
    joint: JointRef,
    joint_pos: Vec2,
    segment: ItemId,
    offset: Vec2,
  ) {
    // :61, :64
    if seen.contains(&segment) {
      return;
    }

    seen.push(segment);

    let Some(item) = world.item(segment) else {
      return;
    };
    let layer = item.layer();
    let net = item.net();

    // :67. Segments that go directly between two linked pads are
    // special cased.
    let other_end = if joint_pos == item.anchor(0) {
      item.anchor(1)
    } else {
      item.anchor(0)
    };

    // :69 to :81
    if let Some(other) =
      world.find_joint(self.world_node, other_end, layer, net)
      && links_hold_a_dragged_pad(world, other, primitives)
    {
      // :77, :78
      self.fixed_items.push(segment);

      return;
    }

    // :86 to :88
    let conn = DraggedConnection {
      // The out parameter KiCad passes here, `segIndex` (`:83`), is
      // never read; it exists only because it has no default.
      orig_line: world.assemble_line(
        self.world_node,
        segment,
        None,
        false,
        false,
        true,
      ),
      attached_pad: solid,
      p_orig: ZERO,
      p_next: ZERO,
      offset,
    };

    // :90 to :96. Lines that go directly between two linked pads are
    // also special cased.
    let chain = conn.orig_line.shape();
    let Some(first) = chain.points().first().copied() else {
      return;
    };
    let Some(last) = chain.points().last().copied() else {
      return;
    };
    let joint_a = world.find_joint(self.world_node, first, layer, net);
    let joint_b = world.find_joint(self.world_node, last, layer, net);
    let at_a = joint_a == Some(joint);
    let at_b = joint_b == Some(joint);

    // :95. In a release build KiCad's `wxASSERT` is a no operation and
    // the `?:` below silently picks `jA`; the assumption holds because
    // `AssembleLine` stops at a pad, so one end of the assembled line is
    // always the joint the seed hangs off (note 06 erratum E35).
    debug_assert!(
      at_a || at_b,
      "an assembled fanout line ends on the joint it was seeded from"
    );

    // :96
    let search = if at_a { joint_b } else { joint_a };

    // :98 to :109
    if let Some(search) = search
      && links_hold_a_dragged_pad(world, search, primitives)
    {
      // :104, :105
      self.fixed_items.extend_from_slice(conn.orig_line.links());

      return;
    }

    // :112
    self.conns.push(conn);
  }

  // -----------------------------------------------------------------
  // Drag
  // -----------------------------------------------------------------

  /// Move the whole component to a point.
  ///
  /// Port of `Drag` (`:156`). It kills every branch of the world, makes
  /// one of its own, takes the selected pads out of it, and then puts
  /// back a translated clone of each pad, a translated clone of each
  /// rigid segment and a re-dragged copy of each attached trace.
  ///
  /// Note 06 erratum E30: this always answers true (`:243`), tests no
  /// collisions, keeps no last good solution and reads no routing mode.
  /// There is nothing for [`crate::dragger::Dragger`]'s first drag
  /// fallback or restore branch to be the counterpart of, and
  /// [`ComponentDragger::fix_route_node`] is the only place a component
  /// drag looks at collisions at all.
  ///
  /// Everything is re-derived from what [`ComponentDragger::start`]
  /// recorded, so the drag is a pure function of the cursor and never
  /// accumulates.
  pub fn drag(&mut self, world: &mut World, at: Vec2) -> bool {
    // :160, :161
    world.kill_children(self.world_node);

    let node = world.branch(self.world_node);

    self.current_node = Some(node);

    // :163, :164
    for id in &self.initial_dragged_items {
      world.remove(node, *id);
    }

    // :166
    self.clear_dragged_items();

    let delta = at - self.p0;

    // :168
    for index in 0..self.solids.len() {
      let id = self.solids[index];
      let Some(item) = world.item(id) else {
        continue;
      };
      let ItemBody::Solid(solid) = item.body() else {
        continue;
      };
      // :170
      let pos = solid.pos();
      let p_next = pos + delta;
      let routable = item.is_routable();

      // :171, :172, :175
      let Some(clone) = move_solid(world, node, id, p_next) else {
        continue;
      };

      // :174
      self.dragged_solids.push(clone);
      self.moved_solids.push(MovedSolid {
        before: id,
        after: clone,
        offset: delta,
      });

      // :177
      if !routable {
        continue;
      }

      // :180 to :186
      for conn in &mut self.conns {
        if conn.attached_pad == id {
          conn.p_orig = pos + conn.offset;
          conn.p_next = p_next + conn.offset;
        }
      }
    }

    // :190
    for index in 0..self.fixed_items.len() {
      let id = self.fixed_items[index];

      if let Some(clone) = move_fixed_item(world, node, id, delta) {
        // :204
        self.dragged_solids.push(clone);
      }
    }

    // :228
    for index in 0..self.conns.len() {
      let mut dragged = self.conns[index].orig_line.clone();
      let p_orig = self.conns[index].p_orig;
      let p_next = self.conns[index].p_next;

      // :231. `LINE::Unmark` runs while the copy still holds the
      // original's links and clears the mask on every one of them
      // (`pcbnew/router/pns_line.cpp:184`), so the board's own segments
      // lose their marker bits too. The order is KiCad's and is kept.
      dragged.unmark(world, MarkerFlags::ALL);

      // :232
      dragged.clear_links();

      // :233. `Find` answers `-1` for a point that is not a vertex and
      // `LINE::DragCorner` swallows that with `wxCHECK_RET`
      // (`pcbnew/router/pns_line.cpp:886`), leaving the line unchanged;
      // that is note 06 erratum E34 and this is the same behaviour
      // spelled as an `Option`.
      if let Some(corner) = dragged.shape().find(p_orig, 0) {
        dragged.drag_corner(p_next, corner, false, Direction45::default());
      }

      // :236. The copy goes into the set **before** the node links the
      // real line, so what `Traces()` answers has no links.
      self.dragged_items.push(dragged.clone());

      // :238, :239. The removal uses a copy that kept its links, where
      // the line being added had them cleared at `:232`; the links, not
      // the geometry, are what a line removal goes by.
      let mut original = self.conns[index].orig_line.clone();

      world.remove_line(node, &mut original);

      // :240
      world.add_line(node, &mut dragged, false);
    }

    // :243
    true
  }

  /// Empty both halves of `m_draggedItems` (`:166`) as one.
  fn clear_dragged_items(&mut self) {
    self.dragged_items.clear();
    self.dragged_solids.clear();
    self.moved_solids.clear();
  }

  // -----------------------------------------------------------------
  // Fix
  // -----------------------------------------------------------------

  /// Commit the drag, or refuse.
  ///
  /// Port of `FixRoute` (`:247`). Unlike `DRAGGER::FixRoute`, which
  /// honours its force flag only in forced mark obstacles mode (note 06
  /// erratum E7), this honours it always, because there is no fallback
  /// state to hide it in and no re-drag branch to fall into: a refused
  /// fix simply answers false and the host's gesture has already ended.
  ///
  /// The collision test is the whole of `m_draggedItems`, cloned pads,
  /// cloned rigid segments and re-dragged lines together, and it is the
  /// **only** one a component drag runs.
  pub fn fix_route(
    &self,
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
  /// The decision half of [`ComponentDragger::fix_route`], which is
  /// `FixRoute` (`:247`) with `Router()->CommitRouting( node )` replaced
  /// by the node it was given. Nothing is committed here, so a caller may
  /// still read the delta off the answer.
  pub fn fix_route_node(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    force_commit: bool,
  ) -> Option<NodeId> {
    let node = self.current_node?;

    // :253
    let allowed = context.settings.allow_drc_violations()
      || force_commit
      || !self.dragged_items_collide(world, context, node);

    allowed.then_some(node)
  }

  /// Whether anything the drag is moving collides in a node.
  ///
  /// The `node->CheckColliding( m_draggedItems )` of `:253`, which is the
  /// `ITEM_SET` overload (`pcbnew/router/pns_node.cpp:478`): a plain loop
  /// that stops at the first member with an obstacle. A [`Line`] is not
  /// an [`Item`] here, so the loop runs over
  /// [`World::check_colliding_line`] for the lines and over
  /// [`World::check_colliding`] for the cloned pads and segments, which
  /// is the decomposition `src/node.rs` names this shape of call site as
  /// the reason for.
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

    self.dragged_solids.iter().any(|id| {
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
    }) || self.dragged_items.iter().any(|line| {
      world
        .check_colliding_line(node, line, context.resolver, &options)
        .is_some()
    })
  }
}

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

/// KiCad's `UNDEFINED_LAYER`, the answer of
/// [`ComponentDragger::current_layer`].
///
/// `layer_ids.h` spells it `-1`; the constant is repeated here rather
/// than pulled into [`crate::item`], because nothing else in the crate
/// has a use for it.
const UNDEFINED_LAYER: i32 = -1;

/// The zero vector, KiCad's `VECTOR2I aOffset = {}` default (`:59`).
const ZERO: Vec2 = Vec2::new(0, 0);

/// The uid of a stored item, and zero for a handle the arena has lost.
///
/// The sort key that replaces KiCad's address ordered `std::set`s; see
/// note 06 erratum E31 and `DESIGN.md` section 8.
fn uid_of(world: &World, id: ItemId) -> u64 {
  world.item(id).map_or(0, Item::uid)
}

/// Whether any solid linked to a joint is one of the dragged primitives.
///
/// The two `LinkCount( SOLID_T )` plus `LinkList()` loops of `:71` to
/// `:81` and `:98` to `:109`, which are the same test twice.
///
/// Note 06 erratum E33: KiCad counts solid links and then walks **every**
/// link, so the `Contains` test is offered segments and vias as
/// candidates too. It cannot misfire in KiCad, because a set that reaches
/// `COMPONENT_DRAGGER` holds nothing but solids
/// (`pcbnew/router/pns_router.cpp:176`), but the filter is what the count
/// promised was there, so it is applied.
fn links_hold_a_dragged_pad(
  world: &World,
  joint: JointRef,
  primitives: &[ItemId],
) -> bool {
  let Some(joint) = world.joint(joint) else {
    return false;
  };

  joint.links().iter().any(|link| {
    world
      .item(*link)
      .is_some_and(|item| item.of_kind(Kind::SOLID))
      && primitives.contains(link)
  })
}

/// Clone a pad at a new position and put the clone in a node.
///
/// `s->Clone()`, `SetPos` and `Add` (`:171`, `:172`, `:175`).
///
/// KiCad's copy constructor deep copies the hole
/// (`pcbnew/router/pns_solid.h:66`) and `SOLID::SetPos` moves it with the
/// copper (`pcbnew/router/pns_solid.cpp:87`). A hole is a separate arena
/// item here, so the clone's stale hole handle is cleared and a fresh
/// hole, translated by the same delta, is handed to
/// [`World::add_solid`]; without that the two pads would share one drill.
///
/// The clone keeps the original's uid, which is this crate's one rule for
/// [`Item`] (`src/item.rs`), and its provenance, so it still answers for
/// the same host object.
fn move_solid(
  world: &mut World,
  node: NodeId,
  id: ItemId,
  at: Vec2,
) -> Option<ItemId> {
  let hole_id = world.item(id)?.hole();
  let mut clone = world.item(id)?.clone();
  let hole = hole_id.and_then(|hole| match world.item(hole)?.body() {
    ItemBody::Hole(hole) => Some(hole.clone()),
    _ => None,
  });
  let ItemBody::Solid(solid) = clone.body_mut() else {
    return None;
  };
  let delta = solid.set_pos(at);

  clone.set_hole(None);

  let hole = hole.map(|mut hole| {
    hole.move_by(delta);
    hole
  });

  Some(world.add_solid(node, clone, hole))
}

/// Translate a segment that runs between two dragged pads.
///
/// The `SEGMENT_T` case of the `m_fixedItems` loop (`:196` to `:207`):
/// remove the original from the branch, clone it, move both ends and add
/// the clone. KiCad's `ARC_T` case (`:210`) has no counterpart, and its
/// `wxFAIL_MSG` for anything else (`:224`) is this [`None`].
///
/// The clone is built before the removal, where KiCad reads the original
/// afterwards. In a branch a removal only shadows, so the two orders
/// agree; taking the copy first is what keeps the handle honest.
fn move_fixed_item(
  world: &mut World,
  node: NodeId,
  id: ItemId,
  delta: Vec2,
) -> Option<ItemId> {
  let mut clone = world.item(id)?.clone();
  let ItemBody::Segment(body) = clone.body_mut() else {
    return None;
  };
  let seg = body.seg();

  // :202
  body.set_ends(seg.a + delta, seg.b + delta);

  // :192
  world.remove(node, id);

  // :205
  world.add_segment(node, clone, false)
}

/// Whether two stored items collide under the session's rules.
///
/// `li->Collide( solid, m_world, solid->Layer() )` (`:146`). The layer
/// argument has no counterpart: [`collide_items`] loops over the
/// candidate's relevant shape layers itself, which `src/collide.rs`
/// documents as the reason its signature takes no layer.
fn items_collide(
  world: &World,
  context: &AlgoContext<'_>,
  item: ItemId,
  head: ItemId,
) -> bool {
  let (Some(item_ref), Some(head_ref)) = (world.item(item), world.item(head))
  else {
    return false;
  };

  collide_items(
    world.items(),
    ItemRef::stored(item, item_ref),
    ItemRef::stored(head, head_ref),
    context.resolver,
    &CollisionSearchOptions::default(),
  )
  .is_some()
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::seg::Seg;
  use crate::geometry::shape::Shape;
  use crate::item::{Hole, LayerRange, Segment, Solid};
  use crate::rules::FixedClearance;
  use crate::settings::RoutingSettings;

  /// The net every fixture item is on.
  const NET: Option<NetId> = Some(NetId(1));

  /// The copper radius of a fixture pad.
  const PAD_RADIUS: i32 = 400_000;

  /// A round pad on layer 0, optionally drilled.
  fn add_pad(world: &mut World, at: Vec2, drill: Option<i32>) -> ItemId {
    let root = world.root();
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(NET);

    let hole = drill.map(|drill| Hole::circular(at, drill / 2));

    world.add_solid(root, item, hole)
  }

  /// A straight track on layer 0.
  fn add_track(world: &mut World, from: Vec2, to: Vec2) -> ItemId {
    let root = world.root();
    let body = ItemBody::Segment(Segment::new(Seg::new(from, to), 200_000));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(NET);

    world
      .add_segment(root, item, false)
      .expect("a fresh track is neither degenerate nor redundant")
  }

  /// The centre of a stored solid.
  fn solid_pos(world: &World, id: ItemId) -> Option<Vec2> {
    match world.item(id)?.body() {
      ItemBody::Solid(solid) => Some(solid.pos()),
      _ => None,
    }
  }

  /// The spine of a stored segment.
  fn segment_seg(world: &World, id: ItemId) -> Option<Seg> {
    match world.item(id)?.body() {
      ItemBody::Segment(body) => Some(body.seg()),
      _ => None,
    }
  }

  #[test]
  fn a_fresh_dragger_reports_the_untouched_board_and_nothing_else() {
    // `CurrentNode()` is `m_currentNode ? m_currentNode : m_world`
    // (`:266`), and the three stubs of
    // `pcbnew/router/pns_component_dragger.h:85`, `:96` and `:113`.
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let dragger = ComponentDragger::new(&world, root);

    assert_eq!(dragger.current_node(), root);
    assert!(dragger.traces().is_empty());
    assert!(dragger.traces_items().is_empty());
    assert!(dragger.moved_solids().is_empty());
    assert!(dragger.current_nets().is_empty());
    assert_eq!(dragger.current_layer(), UNDEFINED_LAYER);
    // Erratum E29: `m_dragStatus` is never assigned after the
    // constructor, so both halves are always false.
    assert_eq!(dragger.force_mark_obstacles_mode(), (false, false));
  }

  #[test]
  fn an_empty_selection_refuses_to_start() {
    // KiCad's `Start` cannot see one: `ROUTER::StartDragging` refuses an
    // empty set first (`pcbnew/router/pns_router.cpp:171`). The guard is
    // here so that a direct caller gets an answer rather than a drag that
    // moves nothing.
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(100_000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let mut dragger = ComponentDragger::new(&world, root);

    assert!(!dragger.start(&world, &context, Vec2::new(0, 0), &[]));
  }

  #[test]
  fn a_moved_pad_takes_a_hole_of_its_own_with_it() {
    // `SOLID`'s copy constructor deep copies the hole
    // (`pcbnew/router/pns_solid.h:66`) and `SetPos` moves it with the
    // copper (`pcbnew/router/pns_solid.cpp:87`). The hole is a separate
    // arena item here, so `move_solid` has to build a fresh one or the
    // two pads would share a drill.
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let pad = add_pad(&mut world, Vec2::new(0, 0), Some(300_000));
    let node = world.branch(root);
    let original_hole = world.item(pad).and_then(Item::hole);

    assert!(original_hole.is_some());

    let clone = move_solid(&mut world, node, pad, Vec2::new(1_000_000, 0))
      .expect("a pad clones");
    let clone_hole = world.item(clone).and_then(Item::hole);

    assert_eq!(solid_pos(&world, clone), Some(Vec2::new(1_000_000, 0)));
    assert_eq!(solid_pos(&world, pad), Some(Vec2::new(0, 0)));
    assert_ne!(clone_hole, original_hole, "the clone reused the pad's hole");
    assert!(clone_hole.is_some(), "the clone lost its drill");
    // The clone keeps the identity of the pad it came from, which is what
    // lets a commit report it as one host object that moved.
    assert_eq!(
      world.item(clone).and_then(Item::source),
      world.item(pad).and_then(Item::source)
    );
    assert_eq!(
      world.item(clone).map(Item::uid),
      world.item(pad).map(Item::uid)
    );
  }

  #[test]
  fn a_pad_with_no_hole_clones_without_gaining_one() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let pad = add_pad(&mut world, Vec2::new(0, 0), None);
    let node = world.branch(root);
    let clone =
      move_solid(&mut world, node, pad, Vec2::new(0, 500_000)).expect("clones");

    assert_eq!(world.item(clone).and_then(Item::hole), None);
    assert_eq!(solid_pos(&world, clone), Some(Vec2::new(0, 500_000)));
  }

  #[test]
  fn a_rigid_segment_translates_and_leaves_the_original_shadowed() {
    // The `SEGMENT_T` case of `:196` to `:207`.
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let track = add_track(&mut world, Vec2::new(0, 0), Vec2::new(1_000_000, 0));
    let node = world.branch(root);
    let delta = Vec2::new(0, 2_000_000);
    let clone = move_fixed_item(&mut world, node, track, delta)
      .expect("a segment translates");

    assert_eq!(
      segment_seg(&world, clone),
      Some(Seg::new(
        Vec2::new(0, 2_000_000),
        Vec2::new(1_000_000, 2_000_000)
      ))
    );

    let (added, removed) = world.get_updated_items(node);

    assert_eq!(added, vec![clone]);
    assert_eq!(removed, vec![track]);
  }

  #[test]
  fn a_pad_is_not_a_rigid_segment() {
    // KiCad reaches `wxFAIL_MSG` for anything that is not a segment or an
    // arc (`:224`); this answers `None` instead.
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let pad = add_pad(&mut world, Vec2::new(0, 0), None);
    let node = world.branch(root);

    assert_eq!(
      move_fixed_item(&mut world, node, pad, Vec2::new(1, 1)),
      None
    );
  }

  #[test]
  fn a_joint_answers_for_the_dragged_pads_and_for_nothing_else() {
    // The two `LinkCount( SOLID_T )` plus `LinkList()` tests of `:71` and
    // `:98`, with erratum E33's missing kind filter applied: only a solid
    // link can make this true.
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let pad = add_pad(&mut world, Vec2::new(0, 0), None);
    let track = add_track(&mut world, Vec2::new(0, 0), Vec2::new(2_000_000, 0));
    let at_pad = world
      .find_joint(root, Vec2::new(0, 0), 0, NET)
      .expect("the pad and the track share a joint");
    let far_end = world
      .find_joint(root, Vec2::new(2_000_000, 0), 0, NET)
      .expect("the track's far end has a joint");

    assert!(links_hold_a_dragged_pad(&world, at_pad, &[pad]));
    assert!(!links_hold_a_dragged_pad(&world, far_end, &[pad]));
    // The track is linked to the near joint, but it is not a solid, so
    // naming it as a primitive changes nothing.
    assert!(!links_hold_a_dragged_pad(&world, at_pad, &[track]));
  }

  #[test]
  fn the_uid_sort_key_orders_two_pads_and_survives_a_lost_handle() {
    // The replacement for KiCad's address ordered `std::set<SOLID*>`
    // (note 06 erratum E31): a uid is monotonic in creation order, so the
    // sort is the order the world was built in and not the allocator's.
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let first = add_pad(&mut world, Vec2::new(0, 0), None);
    let second = add_pad(&mut world, Vec2::new(4_000_000, 0), None);

    assert!(uid_of(&world, first) < uid_of(&world, second));

    let mut ids = vec![second, first];

    ids.sort_by_key(|id| uid_of(&world, *id));

    assert_eq!(ids, vec![first, second]);

    // A handle the arena no longer knows sorts first rather than
    // panicking, which is the same shape `World::uid_of` has.
    let stale = {
      let mut scratch = World::new(World::DEFAULT_MAX_CLEARANCE);
      let root = scratch.root();
      let doomed = add_pad(&mut scratch, Vec2::new(0, 0), None);

      scratch.remove(root, doomed);
      doomed
    };

    assert_eq!(uid_of(&world, stale), 0);
  }
}
