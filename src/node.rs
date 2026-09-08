// SPDX-License-Identifier: GPL-3.0-or-later

//! The branching world: nodes, their overlay, and every query over it.
//!
//! Port of `PNS::NODE` (`pcbnew/router/pns_node.h:232`), the container
//! that holds the router's items, indexes them, keeps their connectivity
//! graph and can be branched, committed and thrown away cheaply. The
//! router uses that branching for shove springback and for the preview
//! frame it rebuilds on every mouse move.
//!
//! # The two level overlay
//!
//! A [`Node`] is either the root or a branch. `NODE::Branch`
//! (`pcbnew/router/pns_node.cpp:157`) copies the parent's index, joints
//! and overrides **only when the parent is not the root**, so a branch of
//! the root starts empty and a deeper branch has already inherited
//! everything its ancestors held. The invariant that falls out is the one
//! every read path depends on and that `DESIGN.md` section 4.5 makes
//! binding:
//!
//! > a node's own state, plus the root's state, minus the node's
//! > overrides, is the complete world.
//!
//! So no query ever walks the parent chain. `World::visit_candidates` is
//! the single helper `doc/reference/kicad/02-item-model-and-node.md`
//! section 10.4 asks for: it visits the node's index and then the root's,
//! filtered through the node's [`Node::overrides`], and nothing else.
//! KiCad open codes that pattern in five places
//! (`pcbnew/router/pns_node.cpp:284`, `:581`, `:1655`, `:1784`, `:1366`),
//! which is how two of them ended up with dead filters; see
//! [`World::query_joints`] and [`World::hit_test`].
//!
//! # Ownership, and what replaces the garbage pool
//!
//! KiCad's items carry an owner pointer and `doRemove` parks a removed
//! item in the root's `m_garbageItems` with a null owner
//! (`pcbnew/router/pns_node.cpp:840`) so that a `LINE` still holding a
//! link to it does not dangle. It is freed later by `releaseGarbage`
//! (`:1592`), and only if the root does not own it again by then.
//!
//! There is no garbage pool here. Ownership is "the node an item was added
//! in", recorded as [`Node::home`], and the rule is:
//!
//! - an item added in a node and removed **in that same node** leaves the
//!   arena immediately, together with the hole it owns;
//! - an item added in one node and removed from a descendant branch stays
//!   in the arena: the branch only de indexes it, or shadows it if it
//!   belongs to the root;
//! - a root item shadowed by a branch's [`Node::overrides`] stays in the
//!   arena until the override is committed, because until then the root
//!   still holds it;
//! - killing a node drops every item homed in it.
//!
//! The consequence is that an [`ItemId`] can go stale, which is exactly
//! what the generational arena makes safe: the lookup answers `None`
//! instead of dereferencing freed memory (`DESIGN.md` section 4.1). Every
//! query in this module tolerates a stale handle by skipping it, so a
//! `Line` holding links into a node that has moved on degrades to "those
//! links no longer resolve" rather than to undefined behaviour.
//!
//! # Determinism
//!
//! Every list this module returns has a defined order, where KiCad's come
//! out of `std::set<ITEM*>`, `std::unordered_set<ITEM*>` and
//! `std::set<OBSTACLE>` keyed on addresses (`DESIGN.md` section 8,
//! `doc/reference/kicad/02-item-model-and-node.md` section 11 entries 2, 3
//! and 15). Obstacles come back sorted by `(item uid, head uid)` and
//! deduplicated by item; item lists come back sorted by uid. The
//! `(distance, uid)` order KiCad's `NearestObstacle` needs is a different
//! order and belongs to that function, which arrives with `Line`.
//!
//! # Members not ported
//!
//! - `Add( LINE& )`, `AssembleLine`, `followLine`, `NearestObstacle`,
//!   `FindLinesBetweenJoints`, the `LINE` overloads of `Remove`,
//!   `Replace`, `QueryColliding` and `CheckColliding`, and
//!   `CheckColliding( const ITEM_SET& )`. All of them need `Line`, which
//!   is the next work item.
//! - `FindItemByParent` / `FindItemsByParent`
//!   (`pcbnew/router/pns_node.cpp:1815`, `:1836`), which resolve a host
//!   object back to an item. That is host bookkeeping: the commit diff
//!   hands [`crate::item::HostId`]s back and the host keeps its own map.
//! - `FindViaByHandle` (`pcbnew/router/pns_node.h:530`), part of the
//!   pointer free via identity the dragger uses.
//! - `FixupVirtualVias` (`pcbnew/router/pns_node.cpp:1270`). Note 02
//!   section 3.15 records it as partly dead: its `n_seg >= 3` trigger can
//!   never fire because `n_seg` is never incremented, and `is_locked` and
//!   `locked_seg` leak across joints. It only matters once the shove
//!   propagates forces through virtual vias, so it is left for the shove
//!   work item to port deliberately, with those errata decided one by one.
//! - `BeginBulkAdd` / `FinalizeBulkAdd` (`:1257`), the deferred index
//!   build, which `src/index.rs` does not have either.
//! - `Dump` (`pcbnew/router/pns_node.h:444`), whose body is `#if 0`.
//! - `SetRuleResolver` / `GetRuleResolver`. The resolver is not world
//!   state: every query takes `&dyn RuleResolver` as a parameter, so that
//!   there is no global and no hidden context (`DESIGN.md` section 8).
//!
//! `AddEdgeExclusion` and `QueryEdgeExclusions`
//! (`pcbnew/router/pns_node.cpp:795`, `:801`) **are** here, small as they
//! are, because `crate::collide` cannot finish the castellation test
//! without them; see [`World::query_edge_exclusions`] for how the
//! collision ladder is meant to reach them.

use std::collections::{BTreeMap, BTreeSet};
use std::rc::Rc;

use crate::arena::{Arena, ArenaId};
use crate::collide::{CollisionSearchOptions, Obstacle, collide_into};
use crate::geometry::box2::Box2;
use crate::geometry::collision;
use crate::geometry::line_chain::LineChain;
use crate::geometry::shape::Shape;
use crate::geometry::vec2::Vec2;
use crate::index::Index;
use crate::item::{
  Hole, Item, ItemBody, ItemId, Kind, LayerRange, MarkerFlags, NetId,
  UidCounter,
};
use crate::joint::{Joint, JointId, JointMap};
use crate::rules::{ItemRef, RuleResolver};

// ---------------------------------------------------------------------
// Handles
// ---------------------------------------------------------------------

/// A handle to a [`Node`] in a [`World`].
///
/// This is the `NODE*` KiCad passes around
/// (`pcbnew/router/pns_node.h:593`), with the lifetime hazards of a raw
/// pointer removed: a handle to a node that has been killed fails its
/// generation check instead of dangling.
pub type NodeId = ArenaId<Node>;

/// A joint, named together with the node whose map holds it.
///
/// `NODE::FindJoint` can answer from the node's own map or from the
/// root's (`pcbnew/router/pns_node.cpp:1366`), and a [`JointId`] alone
/// cannot say which arena it belongs to; `src/joint.rs` says so in its
/// module documentation. So the two travel together.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct JointRef {
  /// The node whose [`JointMap`] holds the joint.
  pub node: NodeId,
  /// The joint.
  pub joint: JointId,
}

// ---------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------

/// One version of the routing world.
///
/// Port of `PNS::NODE`'s state (`pcbnew/router/pns_node.h:592` to
/// `:610`). What is missing from that list is deliberate: the rule
/// resolver is a query parameter (see the module documentation), and
/// `m_garbageItems` is replaced by the ownership rule described there.
#[derive(Clone, Debug)]
pub struct Node {
  /// The node this one was branched from. Port of `m_parent`; `None` on
  /// the root, which is KiCad's `isRoot()`
  /// (`pcbnew/router/pns_node.h:568`).
  parent: Option<NodeId>,
  /// The root of the hierarchy. Port of `m_root`.
  ///
  /// `None` on the root itself, where KiCad writes `m_root = this`
  /// (`pcbnew/router/pns_node.cpp:59`). An arena handle does not exist
  /// before the insertion that mints it returns, so a node cannot be
  /// built already holding its own; `None` says "myself" and
  /// [`World::root_of`] resolves it.
  root: Option<NodeId>,
  /// How many ancestors this node has. Port of `m_depth`
  /// (`pcbnew/router/pns_node.cpp:163`).
  depth: u32,
  /// The live branches of this node. Port of `m_children`.
  children: Vec<NodeId>,
  /// The items added in this node and, for a branch of a branch, those
  /// inherited from the parent. Port of `m_index`.
  index: Index,
  /// The connectivity graph of this node. Port of `m_joints`.
  joints: JointMap,
  /// The root items this branch shadows. Port of `m_override`.
  ///
  /// Only ever root items, which is the `BelongsTo( m_root )` test in
  /// `doRemove` (`pcbnew/router/pns_node.cpp:815`). Ordered, where
  /// KiCad's is a `std::unordered_set<ITEM*>` whose iteration order
  /// reaches the commit diff.
  overrides: BTreeSet<ItemId>,
  /// The broad phase inflation radius. Port of `m_maxClearance`.
  max_clearance: i32,
  /// The items this node owns, that is the ones that were added in it.
  ///
  /// This replaces `OWNABLE_ITEM::m_owner`
  /// (`pcbnew/router/pns_item.h:72`) and the three questions
  /// `BelongsTo` answers with it. An item is homed in exactly one node,
  /// and `Branch` does **not** copy this set even when it copies the
  /// index: a branch of a branch sees its parent's items but does not own
  /// them, exactly as a copied `INDEX` does not change any owner pointer.
  home: BTreeSet<ItemId>,
  /// The castellation exclusion zones. Port of `m_edgeExclusions`
  /// (`pcbnew/router/pns_node.h:606`).
  edge_exclusions: Vec<Shape>,
}

impl Node {
  /// The node this one was branched from, `None` on the root.
  ///
  /// Port of `GetParent`, `pcbnew/router/pns_node.h:507`.
  pub const fn parent(&self) -> Option<NodeId> {
    self.parent
  }

  /// Whether this is the root. Port of `isRoot`,
  /// `pcbnew/router/pns_node.h:568`.
  pub const fn is_root(&self) -> bool {
    self.parent.is_none()
  }

  /// How many ancestors this node has. Port of `Depth`,
  /// `pcbnew/router/pns_node.h:303`.
  pub const fn depth(&self) -> u32 {
    self.depth
  }

  /// The live branches of this node.
  pub fn children(&self) -> &[NodeId] {
    &self.children
  }

  /// Whether this node has live branches. Port of `HasChildren`,
  /// `pcbnew/router/pns_node.h:502`.
  pub fn has_children(&self) -> bool {
    !self.children.is_empty()
  }

  /// The spatial index of this node alone.
  pub const fn index(&self) -> &Index {
    &self.index
  }

  /// The joints of this node alone. Port of the reader behind
  /// `JointCount`, `pcbnew/router/pns_node.h:297`.
  pub const fn joints(&self) -> &JointMap {
    &self.joints
  }

  /// The root items this branch shadows. Port of `GetOverrides`,
  /// `pcbnew/router/pns_node.h:525`.
  pub const fn overrides(&self) -> &BTreeSet<ItemId> {
    &self.overrides
  }

  /// Whether a root item is shadowed here. Port of `Overrides`,
  /// `pcbnew/router/pns_node.h:513`.
  pub fn is_overridden(&self, id: ItemId) -> bool {
    self.overrides.contains(&id)
  }

  /// The broad phase inflation radius. Port of `GetMaxClearance`,
  /// `pcbnew/router/pns_node.h:263`.
  pub const fn max_clearance(&self) -> i32 {
    self.max_clearance
  }

  /// The items added in this node. See the field documentation.
  pub const fn home(&self) -> &BTreeSet<ItemId> {
    &self.home
  }
}

// ---------------------------------------------------------------------
// The candidate query
// ---------------------------------------------------------------------

/// What `World::visit_candidates` is searching for.
///
/// The two shapes of `INDEX::Query` the router uses: one that walks a
/// query item's own layers (`pcbnew/router/pns_index.h:171`) and one that
/// treats every layer as colliding (`:191`).
enum Candidates<'a> {
  /// Every item that could collide with this one, layer by layer.
  Item(&'a Item),
  /// Every item on any layer whose box meets this one.
  Box(Box2),
}

impl Candidates<'_> {
  /// Run one index's worth of the query.
  ///
  /// Returns whether the search may continue, that is whether the visitor
  /// never answered `false`. `src/index.rs` honours that contract, where
  /// KiCad's R-tree carries on into sibling children; see the note there.
  fn run<V>(&self, index: &Index, max_clearance: i32, visitor: &mut V) -> bool
  where
    V: FnMut(ItemId, i32) -> bool,
  {
    let mut keep_going = true;

    {
      let mut watched = |id: ItemId, layer: i32| {
        let answer = visitor(id, layer);

        if !answer {
          keep_going = false;
        }

        answer
      };

      match self {
        Self::Item(item) => {
          index.query_item(item, max_clearance, &mut watched);
        }
        Self::Box(bbox) => {
          index.visit_all_layers(
            bbox.inflate_by(i64::from(max_clearance)),
            &mut watched,
          );
        }
      }
    }

    keep_going
  }
}

// ---------------------------------------------------------------------
// World
// ---------------------------------------------------------------------

/// The item arena, the node arena and the root.
///
/// KiCad has no such type: a `NODE` owns its items outright and reaches
/// its siblings through raw pointers. Arena handles need the arenas to be
/// reachable from one place, which is what
/// `doc/reference/kicad/02-item-model-and-node.md` section 10.2 asks for,
/// and it is also where the caches that KiCad hangs off the rule resolver
/// live (`DESIGN.md` section 5).
#[derive(Debug)]
pub struct World {
  /// Every item of every node.
  items: Arena<Item>,
  /// Every live node.
  nodes: Arena<Node>,
  /// The root node.
  root: NodeId,
  /// The source of item uids.
  ///
  /// One per world, never a process global, so that the `(distance, uid)`
  /// and `(item uid, head uid)` tie breaks are reproducible
  /// (`DESIGN.md` section 8, [`UidCounter`]).
  uids: UidCounter,
  /// The clearance cache. See [`World::clearance_between`].
  clearances: BTreeMap<(ItemId, ItemId, bool), Option<i32>>,
  /// The hull cache. See [`World::hull_of`].
  hulls: BTreeMap<(ItemId, i32, i32, i32), Rc<LineChain>>,
}

impl World {
  /// KiCad's default broad phase inflation radius, in nanometres.
  ///
  /// Port of `m_maxClearance = 800000` (`pcbnew/router/pns_node.cpp:62`),
  /// whose comment is "fixme: depends on how thick traces are". A host
  /// must pass at least the largest clearance any of its rules can
  /// return, or the broad phase drops candidates the narrow phase never
  /// sees; see `src/index.rs`.
  pub const DEFAULT_MAX_CLEARANCE: i32 = 800000;

  /// A world holding one empty root node.
  ///
  /// Port of `NODE::NODE`, `pcbnew/router/pns_node.cpp:57`: depth zero,
  /// itself as root, an empty index.
  pub fn new(max_clearance: i32) -> Self {
    let mut nodes = Arena::new();
    let root = nodes.insert(Node {
      parent: None,
      root: None,
      depth: 0,
      children: Vec::new(),
      index: Index::new(),
      joints: JointMap::new(),
      overrides: BTreeSet::new(),
      max_clearance,
      home: BTreeSet::new(),
      edge_exclusions: Vec::new(),
    });

    Self {
      items: Arena::new(),
      nodes,
      root,
      uids: UidCounter::new(),
      clearances: BTreeMap::new(),
      hulls: BTreeMap::new(),
    }
  }

  // -----------------------------------------------------------------
  // Nodes
  // -----------------------------------------------------------------

  /// The root node.
  pub const fn root(&self) -> NodeId {
    self.root
  }

  /// Borrow a node.
  pub fn node(&self, node: NodeId) -> Option<&Node> {
    self.nodes.get(node)
  }

  /// The number of ancestors a node has. Port of `Depth`,
  /// `pcbnew/router/pns_node.h:303`.
  pub fn depth(&self, node: NodeId) -> Option<u32> {
    Some(self.nodes.get(node)?.depth)
  }

  /// Whether a node is the root. Port of `isRoot`,
  /// `pcbnew/router/pns_node.h:568`. A stale handle is not the root.
  pub fn is_root(&self, node: NodeId) -> bool {
    self.nodes.get(node).is_some_and(Node::is_root)
  }

  /// The node a node was branched from. Port of `GetParent`,
  /// `pcbnew/router/pns_node.h:507`.
  pub fn parent(&self, node: NodeId) -> Option<NodeId> {
    self.nodes.get(node)?.parent
  }

  /// The root of a node's hierarchy, which for the root is itself.
  ///
  /// Port of reading `m_root`. `None` only for a stale handle.
  pub fn root_of(&self, node: NodeId) -> Option<NodeId> {
    Some(self.nodes.get(node)?.root.unwrap_or(node))
  }

  /// The broad phase inflation radius of a node.
  ///
  /// Port of `GetMaxClearance`, `pcbnew/router/pns_node.h:263`.
  pub fn max_clearance(&self, node: NodeId) -> Option<i32> {
    Some(self.nodes.get(node)?.max_clearance)
  }

  /// Set the broad phase inflation radius of a node.
  ///
  /// Port of `SetMaxClearance`, `pcbnew/router/pns_node.h:280`. It must
  /// dominate every clearance the rule resolver can return, or the search
  /// is unsound; `Branch` propagates it
  /// (`pcbnew/router/pns_node.cpp:167`) and this does not reach the nodes
  /// that were branched before the call.
  pub fn set_max_clearance(&mut self, node: NodeId, max_clearance: i32) {
    if let Some(node) = self.nodes.get_mut(node) {
      node.max_clearance = max_clearance;
    }
  }

  /// A new branch of a node.
  ///
  /// Port of `NODE::Branch`, `pcbnew/router/pns_node.cpp:157`. The child
  /// gets the parent's depth plus one, the parent's maximum clearance,
  /// and the root of the hierarchy. It copies the parent's index, joints
  /// and overrides **only when the parent is not the root** (`:171`),
  /// which is what keeps the two level invariant; see the module
  /// documentation. It never copies [`Node::home`]: a copied index does
  /// not change who owns an item, exactly as `INDEX::Clone` does not
  /// touch any owner pointer.
  ///
  /// # Panics
  ///
  /// On a stale node handle, which means the caller kept a handle to a
  /// node that was committed or killed.
  pub fn branch(&mut self, node: NodeId) -> NodeId {
    let parent = self
      .nodes
      .get(node)
      .expect("cannot branch from a node that is no longer live");
    let parent_is_root = parent.is_root();

    let child = Node {
      parent: Some(node),
      root: Some(if parent_is_root {
        node
      } else {
        parent.root.unwrap_or(node)
      }),
      depth: parent.depth + 1,
      children: Vec::new(),
      index: if parent_is_root {
        Index::new()
      } else {
        parent.index.clone()
      },
      joints: if parent_is_root {
        JointMap::new()
      } else {
        parent.joints.clone()
      },
      overrides: if parent_is_root {
        BTreeSet::new()
      } else {
        parent.overrides.clone()
      },
      max_clearance: parent.max_clearance,
      home: BTreeSet::new(),
      edge_exclusions: parent.edge_exclusions.clone(),
    };

    let child_id = self.nodes.insert(child);

    if let Some(parent) = self.nodes.get_mut(node) {
      parent.children.push(child_id);
    }

    child_id
  }

  /// Destroy every branch of a node, and everything they own.
  ///
  /// Port of `KillChildren` and the `releaseChildren` behind it
  /// (`pcbnew/router/pns_node.cpp:1647`, `:1579`). Every item homed in a
  /// dropped node leaves the arena, which is what `~NODE` does for the
  /// items it owns (`:91` to `:131`), and every handle into the dropped
  /// subtree goes stale.
  pub fn kill_children(&mut self, node: NodeId) {
    let Some(live) = self.nodes.get(node) else {
      return;
    };

    for child in live.children.clone() {
      self.release_node(child);
    }

    if let Some(live) = self.nodes.get_mut(node) {
      live.children.clear();
    }
  }

  /// Fold a branch into the root and destroy the branch hierarchy.
  ///
  /// Port of `NODE::Commit`, `pcbnew/router/pns_node.cpp:1622`, called on
  /// the root with a branch as its argument
  /// (`pcbnew/router/pns_router.cpp:911`). Committing the root is a no op
  /// (`:1624`). In order:
  ///
  /// 1. every override becomes a removal from the root (`:1627`);
  /// 2. every item in the branch's index gets its rank reset to
  ///    [`Item::UNASSIGNED_RANK`] (`:1637`) and every marker bit cleared
  ///    (`:1638`, `Unmark()`'s default argument is `-1`), and is added to
  ///    the root, which re homes it and its hole (`:1639`);
  /// 3. the root's children are released (`:1642`).
  ///
  /// Three things worth knowing. There is no ordering guarantee between
  /// the removals and the additions beyond "all removals first". The
  /// third step kills **every** branch of the root, not only the one that
  /// was committed, because `releaseChildren` runs on the node `Commit`
  /// was called on. And the addition dispatch is the raw one, so a
  /// freestanding hole that a branch added is silently dropped, since
  /// `add` treats `HOLE_T` as "added by its parent"
  /// (`pcbnew/router/pns_node.cpp:676`); a hole that has a parent is
  /// carried over by that parent.
  ///
  /// For a deep branch, the index also holds the items of the
  /// intermediate branches it was cloned from, so those are committed
  /// too. That is KiCad's behaviour and note 02 section 3.11 records it.
  pub fn commit(&mut self, branch: NodeId) {
    let Some(node) = self.nodes.get(branch) else {
      return;
    };

    // :1624
    if node.is_root() {
      return;
    }

    let root = node.root.unwrap_or(self.root);
    let removed: Vec<ItemId> = node.overrides.iter().copied().collect();
    let added: Vec<ItemId> = node.index.items().collect();

    // :1627
    for id in removed {
      self.remove(root, id);
    }

    for id in added {
      if let Some(item) = self.items.get_mut(id) {
        // :1637, :1638
        item.set_rank(Item::UNASSIGNED_RANK);
        item.unmark(MarkerFlags::ALL);
      }

      // :1639. Re homing the item to the root is what `SetOwner( this )`
      // inside `add` does, and it is what keeps the subtree release below
      // from taking the item with it.
      self.do_add(root, id);
    }

    // :1642
    self.kill_children(root);
  }

  /// Drop one node and its whole subtree.
  ///
  /// The recursive half of `releaseChildren`,
  /// `pcbnew/router/pns_node.cpp:1579`, plus the item deletion `~NODE`
  /// performs for the items it owns (`:91`). A hole is homed in the same
  /// node as its parent, so it is covered by the same loop, where KiCad
  /// has to special case it (`:99` to `:114`).
  fn release_node(&mut self, node: NodeId) {
    let Some(live) = self.nodes.get(node) else {
      return;
    };

    for child in live.children.clone() {
      self.release_node(child);
    }

    let Some(dropped) = self.nodes.remove(node) else {
      return;
    };

    for id in dropped.home {
      self.items.remove(id);
      self.invalidate_caches(id);
    }
  }

  // -----------------------------------------------------------------
  // Items
  // -----------------------------------------------------------------

  /// The item arena, for the routines that take one.
  ///
  /// [`crate::collide::collide_items`] and the joint predicates need it
  /// to follow [`Item::hole`] and [`Item::parent_pad_via`].
  pub const fn items(&self) -> &Arena<Item> {
    &self.items
  }

  /// Borrow an item. `None` for a handle that has gone stale.
  pub fn item(&self, id: ItemId) -> Option<&Item> {
    self.items.get(id)
  }

  /// Borrow an item mutably.
  ///
  /// # Mutating an indexed item
  ///
  /// Changing the geometry, the layers or the net of an item that is in a
  /// node's index corrupts that index: the entry was keyed on the box the
  /// item had when it was inserted, so the query that should find it may
  /// not (`src/index.rs`, and note 02 section 3.16's list of illegal
  /// mutations). Use [`World::replace`], which re indexes. The marker
  /// bits and the rank are safe to change in place, and the shove does
  /// exactly that.
  ///
  /// The caches are not invalidated here, for the same reason: a
  /// mutation that would change a hull or a clearance is one that has to
  /// go through [`World::replace`] anyway.
  pub fn item_mut(&mut self, id: ItemId) -> Option<&mut Item> {
    self.items.get_mut(id)
  }

  /// A fresh uid from this world's counter.
  ///
  /// Port of `LINKED_ITEM::genNextUid`,
  /// `pcbnew/router/pns_item.cpp:362`, with the process global removed;
  /// see [`UidCounter`].
  pub fn next_uid(&mut self) -> u64 {
    self.uids.next_uid()
  }

  /// A fresh item, with a uid from this world.
  ///
  /// The caller fills in the net, the layers and the flags before handing
  /// it to one of the `add` methods, the way KiCad's host does between
  /// `new SEGMENT` and `NODE::Add`.
  pub fn make_item(&mut self, body: ItemBody) -> Item {
    let uid = self.uids.next_uid();

    Item::new(uid, body)
  }

  /// Which node an item was added in, if any.
  ///
  /// Replaces `OWNABLE_ITEM::Owner`, `pcbnew/router/pns_item.h:72`. It
  /// walks the nodes, which is cheap because a routing session has a
  /// handful of them, where KiCad reads one pointer.
  pub fn home_of(&self, id: ItemId) -> Option<NodeId> {
    self
      .nodes
      .iter()
      .find(|(_, node)| node.home.contains(&id))
      .map(|(node, _)| node)
  }

  // -----------------------------------------------------------------
  // Add
  // -----------------------------------------------------------------

  /// Add an item to a node, dispatching on its body.
  ///
  /// Port of the private `NODE::add`, `pcbnew/router/pns_node.cpp:658`,
  /// which KiCad also exposes as `AddRaw`
  /// (`pcbnew/router/pns_node.h:520`). It performs none of the checks the
  /// typed entry points perform, so a zero length or redundant segment
  /// goes in; use [`World::add_segment`] for those.
  ///
  /// A via with no hole handle yet is given one, because a `VIA` always
  /// has a hole in KiCad (`pcbnew/router/pns_via.h:342` returns true
  /// unconditionally, and the constructor drills it at `:117`).
  ///
  /// # Deviation: a freestanding hole
  ///
  /// KiCad's dispatcher ignores `HOLE_T` outright, with the comment
  /// "added by parent VIA_T or SOLID_T (pad)" (`:676`), which leaks the
  /// pointer it was handed. A hole passed here is indexed instead, which
  /// is what [`World::add_hole`] does and what a mechanical hole needs.
  /// The commit path keeps KiCad's no op, see [`World::commit`].
  pub fn add(&mut self, node: NodeId, item: Item) -> ItemId {
    match item.body() {
      ItemBody::Solid(_) => self.add_solid(node, item, None),
      ItemBody::Segment(_) => {
        let id = self.items.insert(item);
        self.do_add_segment(node, id);

        id
      }
      ItemBody::Via(_) => self.add_via(node, item),
      ItemBody::Hole(_) => self.add_hole(node, item),
    }
  }

  /// Add a pad or another fixed obstacle, with the hole it was drilled
  /// with.
  ///
  /// Port of `NODE::Add( unique_ptr<SOLID> )` and the `addSolid` behind
  /// it (`pcbnew/router/pns_node.cpp:617`, `:601`): the hole goes in
  /// first, then a joint is linked **only if the solid is routable**
  /// (`:609`), then the solid is indexed.
  ///
  /// The hole is created here rather than being passed in already
  /// stored, because the two point at each other and neither handle
  /// exists before the other is inserted. It inherits the solid's layers
  /// and net, which is what `SOLID::SetHole`
  /// (`pcbnew/router/pns_solid.h:150`) and `HOLE::Net`
  /// (`pcbnew/router/pns_hole.h:56`) arrange in KiCad.
  ///
  /// # Panics
  ///
  /// When the item is not a solid.
  pub fn add_solid(
    &mut self,
    node: NodeId,
    solid: Item,
    hole: Option<Hole>,
  ) -> ItemId {
    assert!(
      matches!(solid.body(), ItemBody::Solid(_)),
      "add_solid needs a solid body"
    );

    let id = self.insert_with_hole(solid, hole);
    self.do_add_solid(node, id);

    id
  }

  /// Add a track.
  ///
  /// Port of `NODE::Add( unique_ptr<SEGMENT>, bool )`,
  /// `pcbnew/router/pns_node.cpp:747`, which rejects a segment whose two
  /// ends are the same point (`:749`) and, unless `allow_redundant`, one
  /// that duplicates a segment already linked to the joint at its start
  /// (`:756`). A rejected segment is not stored at all, where KiCad drops
  /// the `unique_ptr` it was handed.
  ///
  /// The joints at both endpoints are linked by `addSegment` (`:736`).
  ///
  /// # Panics
  ///
  /// When the item is not a segment.
  pub fn add_segment(
    &mut self,
    node: NodeId,
    segment: Item,
    allow_redundant: bool,
  ) -> Option<ItemId> {
    let ItemBody::Segment(body) = segment.body() else {
      panic!("add_segment needs a segment body");
    };

    let seg = body.seg();

    // :749
    if seg.a == seg.b {
      return None;
    }

    // :756
    if !allow_redundant
      && self
        .find_redundant_segment(
          node,
          seg.a,
          seg.b,
          segment.layers(),
          segment.net(),
        )
        .is_some()
    {
      return None;
    }

    let id = self.items.insert(segment);
    self.do_add_segment(node, id);

    Some(id)
  }

  /// Add a via, drilling its hole.
  ///
  /// Port of `NODE::Add( unique_ptr<VIA> )` and the `addVia` behind it
  /// (`pcbnew/router/pns_node.cpp:652`, `:624`): the hole goes in first,
  /// then one joint is linked at the via's position over the via's
  /// **full layer range**, which is what binds the layers together and
  /// what [`World::remove`] has to split again, then the via is indexed.
  ///
  /// The hole is a circle of half the drill, as `VIA`'s constructor makes
  /// it (`pcbnew/router/pns_via.h:117`,
  /// `HOLE::MakeCircularHole`), and it takes the via's layer range
  /// (`VIA::SetHoleLayers`, `pcbnew/router/pns_via.cpp:87`) and net. An
  /// item that already carries a hole handle keeps it, which is how a
  /// committed via is re added.
  ///
  /// # Panics
  ///
  /// When the item is not a via.
  pub fn add_via(&mut self, node: NodeId, via: Item) -> ItemId {
    let ItemBody::Via(body) = via.body() else {
      panic!("add_via needs a via body");
    };

    let hole = if via.hole().is_some() {
      None
    } else {
      Some(Hole::circular(body.pos(), body.drill() / 2))
    };

    let id = self.insert_with_hole(via, hole);
    self.do_add_via(node, id);

    id
  }

  /// Add a hole on its own.
  ///
  /// Port of `NODE::addHole`, `pcbnew/router/pns_node.cpp:642`: no joint,
  /// index only. The commented out `linkJoint` at `:644` asks "do we need
  /// holes in the connection graph?"; the answer in this revision is no.
  ///
  /// # Panics
  ///
  /// When the item is not a hole.
  pub fn add_hole(&mut self, node: NodeId, hole: Item) -> ItemId {
    assert!(
      matches!(hole.body(), ItemBody::Hole(_)),
      "add_hole needs a hole body"
    );

    let id = self.items.insert(hole);
    self.do_add_hole(node, id);

    id
  }

  /// Take an item out and put a new one in its place.
  ///
  /// Port of `NODE::Replace( ITEM*, unique_ptr<ITEM> )`,
  /// `pcbnew/router/pns_node.cpp:951`, which is literally a removal
  /// followed by the raw addition, so none of the segment checks apply.
  /// The old handle goes stale if the node owned the item; see the module
  /// documentation.
  pub fn replace(
    &mut self,
    node: NodeId,
    old: ItemId,
    new_item: Item,
  ) -> ItemId {
    self.remove(node, old);

    self.add(node, new_item)
  }

  /// Store an item together with the hole it owns, linked both ways.
  ///
  /// The arena counterpart of `ITEM::SetHole`
  /// (`pcbnew/router/pns_via.h:328`), which sets the hole's parent, its
  /// owner and its layers in one go. Neither handle exists before the
  /// other is inserted, so the parent is stored first and patched.
  fn insert_with_hole(&mut self, item: Item, hole: Option<Hole>) -> ItemId {
    let layers = item.layers();
    let net = item.net();
    let id = self.items.insert(item);

    let Some(hole) = hole else {
      return id;
    };

    let uid = self.uids.next_uid();
    let mut drilled = Item::new(uid, ItemBody::Hole(hole));
    drilled.set_layers_and_flash_all(layers);
    drilled.set_net(net);
    drilled.set_parent_pad_via(Some(id));

    let hole_id = self.items.insert(drilled);

    if let Some(parent) = self.items.get_mut(id) {
      parent.set_hole(Some(hole_id));
    }

    id
  }

  /// The raw addition dispatcher.
  ///
  /// Port of the private `NODE::add`, `pcbnew/router/pns_node.cpp:658`,
  /// on an item that is already in the arena. This is the form
  /// [`World::commit`] uses, so it keeps KiCad's `HOLE_T` no op at
  /// `:676`.
  fn do_add(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    match item.body() {
      ItemBody::Solid(_) => self.do_add_solid(node, id),
      ItemBody::Segment(_) => self.do_add_segment(node, id),
      ItemBody::Via(_) => self.do_add_via(node, id),
      // :676, "added by parent VIA_T or SOLID_T (pad)".
      ItemBody::Hole(_) => {}
    }
  }

  /// Port of `NODE::addSolid`, `pcbnew/router/pns_node.cpp:601`.
  fn do_add_solid(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    let ItemBody::Solid(solid) = item.body() else {
      return;
    };

    let pos = solid.pos();
    let layers = item.layers();
    let net = item.net();
    let routable = item.is_routable();
    let hole = item.hole();

    if let Some(hole) = hole {
      self.do_add_hole(node, hole);
    }

    // :609. A non routable solid gets no joint, so nothing can start or
    // end a trace on it.
    if routable {
      self.with_joints(node, |joints, root, _| {
        joints.link_joint(pos, layers, net, id, root);
      });
    }

    self.index_add(node, id);
  }

  /// Port of `NODE::addVia`, `pcbnew/router/pns_node.cpp:624`.
  fn do_add_via(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    let ItemBody::Via(via) = item.body() else {
      return;
    };

    let pos = via.pos();
    let layers = item.layers();
    let net = item.net();
    let hole = item.hole();

    if let Some(hole) = hole {
      self.do_add_hole(node, hole);
    }

    // :635. One joint over the whole layer range, which is what binds the
    // layers together.
    self.with_joints(node, |joints, root, _| {
      joints.link_joint(pos, layers, net, id, root);
    });

    self.index_add(node, id);
  }

  /// Port of `NODE::addSegment`, `pcbnew/router/pns_node.cpp:736`.
  fn do_add_segment(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    let ItemBody::Segment(segment) = item.body() else {
      return;
    };

    let seg = segment.seg();
    let layers = item.layers();
    let net = item.net();

    self.with_joints(node, |joints, root, _| {
      joints.link_joint(seg.a, layers, net, id, root);
      joints.link_joint(seg.b, layers, net, id, root);
    });

    self.index_add(node, id);
  }

  /// Port of `NODE::addHole`, `pcbnew/router/pns_node.cpp:642`.
  fn do_add_hole(&mut self, node: NodeId, id: ItemId) {
    self.index_add(node, id);
  }

  /// Index an item in a node and give the node ownership of it.
  ///
  /// The `SetOwner( this ); m_index->Add( aItem )` pair every `addXxx`
  /// helper ends with (`pcbnew/router/pns_node.cpp:613`, `:637`, `:648`,
  /// `:743`).
  fn index_add(&mut self, node: NodeId, id: ItemId) {
    if let Some(previous) = self.home_of(id)
      && previous != node
      && let Some(previous) = self.nodes.get_mut(previous)
    {
      previous.home.remove(&id);
    }

    let items = &self.items;

    if let Some(live) = self.nodes.get_mut(node) {
      live.index.add(items, id);
      live.home.insert(id);
    }
  }

  /// A segment already linked to the joint at `a` with the same ends.
  ///
  /// Port of `NODE::findRedundantSegment`,
  /// `pcbnew/router/pns_node.cpp:1707`. The endpoints may be the other
  /// way round, and the layer test is on the range's start only, which is
  /// KiCad's.
  fn find_redundant_segment(
    &self,
    node: NodeId,
    a: Vec2,
    b: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
  ) -> Option<ItemId> {
    let start = self.find_joint(node, a, layers.start(), net)?;
    let joint = self.joint(start)?;

    joint
      .links()
      .iter()
      .copied()
      .find(|id| self.is_same_segment(*id, a, b, layers))
  }

  /// Whether a stored item is a segment with those ends on that layer.
  ///
  /// The body of the loop in `findRedundantSegment`,
  /// `pcbnew/router/pns_node.cpp:1715` to `:1726`.
  fn is_same_segment(
    &self,
    id: ItemId,
    a: Vec2,
    b: Vec2,
    layers: LayerRange,
  ) -> bool {
    let Some(item) = self.items.get(id) else {
      return false;
    };

    let ItemBody::Segment(segment) = item.body() else {
      return false;
    };

    let seg = segment.seg();

    item.layers().start() == layers.start()
      && ((a == seg.a && b == seg.b) || (a == seg.b && b == seg.a))
  }

  // -----------------------------------------------------------------
  // Remove
  // -----------------------------------------------------------------

  /// Take an item out of a node.
  ///
  /// Port of `NODE::Remove( ITEM* )`, `pcbnew/router/pns_node.cpp:998`,
  /// which dispatches on the kind. Each typed remover tears the joint
  /// structure down first and then calls `doRemove`:
  ///
  /// - a segment unlinks the joints at both of its ends
  ///   (`removeSegmentIndex`, `:856`);
  /// - a via finds the joint at its position and rebuilds it, splitting
  ///   the layers it bound back apart (`removeViaIndex`, `:931`);
  /// - a routable solid does the same, and a non routable one is skipped
  ///   entirely because it never had a joint (`removeSolidIndex`, `:939`);
  /// - a hole is a no op, because `Remove( ITEM* )`'s switch has no
  ///   `HOLE_T` case and falls through to `default: break` (`:1047`).
  ///   Holes leave with their parent, whose `doRemove` de indexes them.
  ///
  /// The `Remove( solid->Hole() )` calls at `:1010` and `:1036` are that
  /// same no op, so they are not reproduced; the `SetOwner` that follows
  /// each of them is the ownership hand off that
  /// `World::do_remove` performs anyway.
  pub fn remove(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    match item.body() {
      ItemBody::Solid(_) => {
        self.remove_solid_index(node, id);
        self.do_remove(node, id);
      }
      ItemBody::Segment(_) => {
        self.remove_segment_index(node, id);
        self.do_remove(node, id);
      }
      ItemBody::Via(_) => {
        self.remove_via_index(node, id);
        self.do_remove(node, id);
      }
      ItemBody::Hole(_) => {}
    }
  }

  /// Remove every locally indexed item carrying a marker bit.
  ///
  /// Port of `NODE::RemoveByMarker`,
  /// `pcbnew/router/pns_node.cpp:1694`, which collects first and removes
  /// afterwards because the removal mutates the index it is walking.
  pub fn remove_by_marker(&mut self, node: NodeId, marker: MarkerFlags) {
    let Some(live) = self.nodes.get(node) else {
      return;
    };

    let doomed: Vec<ItemId> = live
      .index
      .items()
      .filter(|id| {
        self
          .items
          .get(*id)
          .is_some_and(|item| item.marker().intersects(marker))
      })
      .collect();

    for id in doomed {
      self.remove(node, id);
    }
  }

  /// Port of `NODE::removeSegmentIndex`,
  /// `pcbnew/router/pns_node.cpp:856`.
  fn remove_segment_index(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    let ItemBody::Segment(segment) = item.body() else {
      return;
    };

    let seg = segment.seg();
    let layers = item.layers();
    let net = item.net();

    self.with_joints(node, |joints, root, _| {
      joints.unlink_joint(seg.a, layers, net, id, root);
      joints.unlink_joint(seg.b, layers, net, id, root);
    });
  }

  /// Port of `NODE::removeViaIndex`, `pcbnew/router/pns_node.cpp:931`.
  fn remove_via_index(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    let ItemBody::Via(via) = item.body() else {
      return;
    };

    let pos = via.pos();
    let layers = item.layers();
    let net = item.net();

    self.rebuild_joint_of(node, id, pos, layers, net);
  }

  /// Port of `NODE::removeSolidIndex`,
  /// `pcbnew/router/pns_node.cpp:939`.
  fn remove_solid_index(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    let ItemBody::Solid(solid) = item.body() else {
      return;
    };

    // :941. A non routable solid never had a joint.
    if !item.is_routable() {
      return;
    }

    let pos = solid.pos();
    let layers = item.layers();
    let net = item.net();

    self.rebuild_joint_of(node, id, pos, layers, net);
  }

  /// Find the joint an item sits on and split it back into per layer
  /// joints.
  ///
  /// The shared body of `removeViaIndex` and `removeSolidIndex`
  /// (`pcbnew/router/pns_node.cpp:933`, `:946`), which both call
  /// `FindJoint( pos, layers.Start(), net )` and hand the result to
  /// `rebuildJoint`. That joint may live in the root's map, which is why
  /// only its position and its link list are passed on; `src/joint.rs`
  /// explains that at [`JointMap::rebuild_joint`].
  ///
  /// KiCad asserts that the joint exists (`:934`, `:947`). A missing
  /// joint is ignored here, because a non routable solid and an item that
  /// was never added both reach this legitimately.
  fn rebuild_joint_of(
    &mut self,
    node: NodeId,
    id: ItemId,
    pos: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
  ) {
    let Some(found) = self.find_joint(node, pos, layers.start(), net) else {
      return;
    };

    let Some(joint) = self.joint(found) else {
      return;
    };

    let joint_pos = joint.pos();
    let links = joint.links().to_vec();

    self.with_joints(node, |joints, root, items| {
      joints.rebuild_joint(items, joint_pos, &links, id, root);
    });
  }

  /// Record a root item as shadowed by this branch.
  ///
  /// Case one of `doRemove`, `pcbnew/router/pns_node.cpp:815` to `:820`.
  /// The root keeps the item, so it stays in the arena and stays visible
  /// from every other branch. The hole goes into the set as well, which
  /// is what makes the commit remove both.
  fn shadow_in_branch(
    &mut self,
    node: NodeId,
    id: ItemId,
    hole: Option<ItemId>,
  ) {
    let Some(live) = self.nodes.get_mut(node) else {
      return;
    };

    live.overrides.insert(id);

    if let Some(hole) = hole {
      live.overrides.insert(hole);
    }
  }

  /// Take an item and its hole out of one node's index.
  ///
  /// Case two of `doRemove`, `pcbnew/router/pns_node.cpp:825` to `:834`.
  /// Answers KiCad's `holeRemoved`, whose "fixme: better logic, I do not
  /// like this" at `:811` is about the third case having to know whether
  /// this one already ran.
  fn deindex(
    &mut self,
    node: NodeId,
    id: ItemId,
    hole: Option<ItemId>,
  ) -> bool {
    let Some(live) = self.nodes.get_mut(node) else {
      return false;
    };

    live.index.remove(id);

    let Some(hole) = hole else {
      return false;
    };

    live.index.remove(hole);

    true
  }

  /// Shadow, de index or drop an item, depending on who owns it.
  ///
  /// Port of `NODE::doRemove`, `pcbnew/router/pns_node.cpp:809`, the
  /// heart of the copy on write model:
  ///
  /// - a **root** item removed from a **branch** is shadowed, not
  ///   touched: it and its hole go into [`Node::overrides`] (`:815` to
  ///   `:820`). The root keeps them, so both stay in the arena and stay
  ///   visible from every other branch;
  /// - anything else is taken out of this node's index, and so is its
  ///   hole (`:825` to `:834`);
  /// - if the item was **added in this node**, it and its hole leave the
  ///   arena (`:837` to `:852`). KiCad parks them in the root's garbage
  ///   pool with a null owner and frees them later, which exists only so
  ///   that stale `LINE` links stay dereferenceable; a generational
  ///   handle makes that unnecessary (note 02 section 11 entry 16). The
  ///   consequence is documented on the module: such an [`ItemId`] is
  ///   stale from here on, and every query treats it as absent.
  fn do_remove(&mut self, node: NodeId, id: ItemId) {
    let Some(root) = self.root_of(node) else {
      return;
    };

    let is_root = node == root;
    let home = self.home_of(id);
    let hole = self.items.get(id).and_then(Item::hole);
    let mut hole_removed = false;

    if home == Some(root) && !is_root {
      // :815
      self.shadow_in_branch(node, id, hole);
    } else if home != Some(root) || is_root {
      // :825
      hole_removed = self.deindex(node, id, hole);
    }

    // :837
    if home != Some(node) {
      return;
    }

    if let Some(hole) = hole
      && !hole_removed
      && let Some(live) = self.nodes.get_mut(node)
    {
      // :847. The hole is not indexed by the node in KiCad's ownership
      // model but by its parent, so it has to come out separately.
      live.index.remove(hole);
    }

    if let Some(live) = self.nodes.get_mut(node) {
      live.home.remove(&id);

      if let Some(hole) = hole {
        live.home.remove(&hole);
      }
    }

    self.items.remove(id);
    self.invalidate_caches(id);

    // :850. The hole reverts to its parent, which here means it goes
    // wherever the parent goes.
    if let Some(hole) = hole {
      self.items.remove(hole);
      self.invalidate_caches(hole);
    }
  }

  // -----------------------------------------------------------------
  // Queries
  // -----------------------------------------------------------------

  /// Visit the candidates of a query: this node's index, then the root's
  /// filtered through this node's overrides.
  ///
  /// This is the single helper `DESIGN.md` section 4.4 and note 02
  /// section 10.4 ask for. Every read path goes through it, and no read
  /// path ever walks the parent chain, because
  /// `NODE::Branch` has already copied whatever an intermediate branch
  /// held (`pcbnew/router/pns_node.cpp:171`).
  ///
  /// It is the shared shape of `QueryColliding` (`:284`), `HitTest`
  /// (`:581`), `AllItemsInNet` (`:1655`) and `QueryJoints` (`:1784`).
  /// Returning `false` from the visitor stops the whole search, the local
  /// pass and the root pass alike.
  fn visit_candidates<V>(
    &self,
    node: NodeId,
    query: &Candidates<'_>,
    mut visitor: V,
  ) where
    V: FnMut(ItemId, i32) -> bool,
  {
    let Some(live) = self.nodes.get(node) else {
      return;
    };

    let max_clearance = live.max_clearance;

    if !query.run(&live.index, max_clearance, &mut visitor) {
      return;
    }

    let Some(root) = live.root else {
      return;
    };

    let Some(root) = self.nodes.get(root) else {
      return;
    };

    query.run(&root.index, max_clearance, &mut |id, layer| {
      // :288, the override test `OBSTACLE_VISITOR::visit` performs
      // (`pcbnew/router/pns_node.cpp:215`), hoisted into the one helper.
      if live.overrides.contains(&id) {
        return true;
      }

      visitor(id, layer)
    });
  }

  /// Every obstacle a head item meets in this node.
  ///
  /// Port of `NODE::QueryColliding` (`pcbnew/router/pns_node.cpp:267`)
  /// and the `DEFAULT_OBSTACLE_VISITOR` it drives (`:241`). The visitor's
  /// filters, in KiCad's order: the kind mask (`:243`), the self identity
  /// test (`:247`), the user filter (`:249`, not ported, see
  /// [`CollisionSearchOptions`]), the override test (`:252`, inside
  /// `World::visit_candidates`), then the item level collision through
  /// [`collide_into`], then the limit (`:259`).
  ///
  /// A virtual head collides with nothing and returns immediately
  /// (`:272`, "by default, virtual items cannot collide").
  ///
  /// # Ordering
  ///
  /// KiCad accumulates into a `std::set<OBSTACLE>` ordered by the pair of
  /// **addresses** `(m_head, m_item)` (`pcbnew/router/pns_node.h:103`),
  /// which is not reproducible across runs. Note 02 section 11 entry 4
  /// observes that the head is a stack temporary that is reused every
  /// iteration, so the deduplication degenerates to "unique by item",
  /// which is what the algorithm wants. That is what this does: one
  /// obstacle per item, the first one found, sorted by
  /// `(item uid, head uid)`. An unstored or stale handle sorts last.
  ///
  /// The `(distance, uid)` order belongs to `NearestObstacle`, which
  /// needs `Line`.
  ///
  /// # The limit
  ///
  /// `m_limitCount` is honoured against a search that really stops, where
  /// KiCad's R-tree keeps visiting siblings after the visitor asks it not
  /// to and its obstacle set can therefore come back longer than the
  /// limit (`src/index.rs`). The limit is also re tested when a candidate
  /// is offered, which is what makes the root pass a no op once it has
  /// been reached; KiCad expresses the same thing as an explicit test
  /// before starting that pass (`:283`).
  pub fn query_colliding(
    &self,
    node: NodeId,
    head: ItemRef<'_>,
    resolver: &dyn RuleResolver,
    options: &CollisionSearchOptions,
  ) -> Vec<Obstacle> {
    // :272
    if head.item().is_virtual() {
      return Vec::new();
    }

    let items = &self.items;
    let limit = options.limit_count.filter(|count| *count > 0);
    let mut obstacles: BTreeMap<Option<ItemId>, Obstacle> = BTreeMap::new();
    let mut scratch: Vec<Obstacle> = Vec::new();

    self.visit_candidates(node, &Candidates::Item(head.item()), |id, layer| {
      if limit.is_some_and(|limit| obstacles.len() >= limit) {
        return false;
      }

      let Some(candidate) = items.get(id) else {
        return true;
      };

      // :243
      if !candidate.of_kind(options.kind_mask) {
        return true;
      }

      let candidate = ItemRef::stored(id, candidate);

      // :247. Collisions with self are not a thing.
      if candidate.is_same_as(head) {
        return true;
      }

      scratch.clear();

      let found = collide_into(
        items,
        candidate,
        head,
        layer,
        resolver,
        options,
        &mut scratch,
      );

      for obstacle in scratch.drain(..) {
        obstacles.entry(obstacle.item).or_insert(obstacle);
      }

      // :255. A candidate that answered "no collision" does not reach the
      // limit test, even when its hole recursion left something in the
      // set.
      if !found {
        return true;
      }

      // :259
      limit.is_none_or(|limit| obstacles.len() < limit)
    });

    let mut found: Vec<Obstacle> = obstacles.into_values().collect();
    found.sort_by_key(|obstacle| {
      (self.uid_of(obstacle.item), self.uid_of(obstacle.head))
    });

    found
  }

  /// The first obstacle a head item meets, or `None`.
  ///
  /// Port of `NODE::CheckColliding( const ITEM*, const
  /// COLLISION_SEARCH_OPTIONS& )`, `pcbnew/router/pns_node.cpp:502`,
  /// minus its `LINE_T` branch, which needs `Line`. KiCad answers with
  /// `*obs.begin()`, the address smallest obstacle, which note 02 section
  /// 11 entry 3 calls arbitrary; this answers the first in the
  /// deterministic order [`World::query_colliding`] establishes.
  ///
  /// The `CheckColliding( item, kindMask )` overload (`:493`) is this one
  /// with `kind_mask` set and `limit_count` at `Some(1)`.
  pub fn check_colliding(
    &self,
    node: NodeId,
    head: ItemRef<'_>,
    resolver: &dyn RuleResolver,
    options: &CollisionSearchOptions,
  ) -> Option<Obstacle> {
    self
      .query_colliding(node, head, resolver, options)
      .into_iter()
      .next()
  }

  /// Every item whose shape contains a point.
  ///
  /// Port of `NODE::HitTest`, `pcbnew/router/pns_node.cpp:572`, which
  /// treats the point as a circle of radius zero ("fixme: we treat a
  /// point as an infinitely small circle, this is inefficient", `:576`)
  /// and tests `aItem->Shape( -1 )->Collide( &cp, 0 )` (`:563`). The `-1`
  /// carries KiCad's own "TODO(JE) padstacks, this may not work": a via
  /// with a complex padstack has no shape there and KiCad dereferences
  /// the null. An item with no shape on that layer is skipped here.
  ///
  /// KiCad filters the root's hits through `Overrides` **after** the
  /// query (`:590`), because the second visitor never gets a `SetWorld`
  /// call and the one at `:586` is dead. The filter is inside
  /// `World::visit_candidates` here, which is the same answer.
  ///
  /// The result is deduplicated, which matters because an item is visited
  /// once per layer, and ordered by uid, where KiCad's `ITEM_SET` keeps
  /// R-tree traversal order and its duplicates.
  pub fn hit_test(&self, node: NodeId, point: Vec2) -> Vec<ItemId> {
    let probe = Shape::circle(point, 0);
    let items = &self.items;
    let mut hits: BTreeSet<ItemId> = BTreeSet::new();

    self.visit_candidates(node, &Candidates::Box(Box2::from_vec2(point)), {
      |id, _layer| {
        if let Some(item) = items.get(id)
          && let Some(shape) = item.shape(-1)
          && collision::collides(&shape, &probe, 0)
        {
          hits.insert(id);
        }

        true
      }
    });

    let mut found: Vec<ItemId> = hits.into_iter().collect();
    found.sort_by_key(|id| self.uid_of(Some(*id)));

    found
  }

  /// Every routable item of one net, of the kinds asked for.
  ///
  /// Port of `NODE::AllItemsInNet`,
  /// `pcbnew/router/pns_node.cpp:1653`: this node's net bucket, then the
  /// root's filtered through the overrides, both filtered by the kind
  /// mask and by [`Item::is_routable`]. KiCad collects into a
  /// `std::set<ITEM*>`, so the result is deduplicated in address order;
  /// this one is in uid order.
  pub fn all_items_in_net(
    &self,
    node: NodeId,
    net: Option<NetId>,
    kind_mask: Kind,
  ) -> Vec<ItemId> {
    let Some(live) = self.nodes.get(node) else {
      return Vec::new();
    };

    let mut found: BTreeSet<ItemId> = BTreeSet::new();

    for id in live.index.items_in_net(net) {
      if self.is_routable_of_kind(*id, kind_mask) {
        found.insert(*id);
      }
    }

    if let Some(root) = live.root
      && let Some(root) = self.nodes.get(root)
    {
      for id in root.index.items_in_net(net) {
        if !live.overrides.contains(id)
          && self.is_routable_of_kind(*id, kind_mask)
        {
          found.insert(*id);
        }
      }
    }

    let mut found: Vec<ItemId> = found.into_iter().collect();
    found.sort_by_key(|id| self.uid_of(Some(*id)));

    found
  }

  /// The `OfKind( aKindMask ) && IsRoutable()` pair of `AllItemsInNet`,
  /// `pcbnew/router/pns_node.cpp:1661`.
  fn is_routable_of_kind(&self, id: ItemId, kind_mask: Kind) -> bool {
    self
      .items
      .get(id)
      .is_some_and(|item| item.of_kind(kind_mask) && item.is_routable())
  }

  /// The joint at a position, on a net, covering a layer.
  ///
  /// Port of `NODE::FindJoint`, `pcbnew/router/pns_node.cpp:1704`. It
  /// looks in this node's map and falls through to the root's **only when
  /// the key is absent** here. A key that is present but holds nothing
  /// covering the layer answers "no joint" without consulting the root:
  /// that is the tombstone a branch plants when it deliberately removes a
  /// joint (`:911`), which `src/joint.rs` stores as a present key with an
  /// empty list rather than as a joint with a negative layer range.
  pub fn find_joint(
    &self,
    node: NodeId,
    pos: Vec2,
    layer: i32,
    net: Option<NetId>,
  ) -> Option<JointRef> {
    let live = self.nodes.get(node)?;

    if let Some(joint) = live.joints.find_joint(pos, layer, net) {
      return Some(JointRef { node, joint });
    }

    if live.joints.contains_key(pos, net) {
      return None;
    }

    let root = live.root?;
    let joint = self.nodes.get(root)?.joints.find_joint(pos, layer, net)?;

    Some(JointRef { node: root, joint })
  }

  /// Borrow a joint one of the two maps answered with.
  pub fn joint(&self, joint: JointRef) -> Option<&Joint> {
    self.nodes.get(joint.node)?.joints.get(joint.joint)
  }

  /// The joints at the two ends of a run of segments.
  ///
  /// Port of `NODE::FindLineEnds`,
  /// `pcbnew/router/pns_node.cpp:1216`, which is two [`World::find_joint`]
  /// calls at the first and last point with the line's layer range start
  /// and net (`pcbnew/router/pns_node.h:478`). KiCad dereferences both
  /// results unchecked; this answers `None` when either end has no joint.
  /// The signature takes the four properties directly because `Line` does
  /// not exist yet.
  pub fn find_line_ends(
    &self,
    node: NodeId,
    first_point: Vec2,
    last_point: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
  ) -> Option<(JointRef, JointRef)> {
    let start = self.find_joint(node, first_point, layers.start(), net)?;
    let end = self.find_joint(node, last_point, layers.start(), net)?;

    Some((start, end))
  }

  /// Every joint in a box, over a layer range, with a link of a kind.
  ///
  /// Port of `NODE::QueryJoints`,
  /// `pcbnew/router/pns_node.cpp:1777`: this node's map, then the root's.
  ///
  /// KiCad filters the root's joints through `Overrides( &j.second )`
  /// (`:1801`), which passes a `JOINT*` to a test against a set of
  /// `ITEM*` and therefore never matches anything. The filter is
  /// reproduced as what it actually is, which is no filter at all; note
  /// 02 section 3.7 calls it out.
  ///
  /// `net` is an addition on the map side: `None` accepts every net, as
  /// KiCad does. The result is ordered by joint position and net, then by
  /// which map answered, where KiCad's is in hash order.
  pub fn query_joints(
    &self,
    node: NodeId,
    bbox: Box2,
    net: Option<Option<NetId>>,
    layers: LayerRange,
    kind_mask: Kind,
  ) -> Vec<JointRef> {
    let Some(live) = self.nodes.get(node) else {
      return Vec::new();
    };

    let mut found: Vec<JointRef> = live
      .joints
      .query_joints(&self.items, bbox, net, layers, kind_mask)
      .into_iter()
      .map(|joint| JointRef { node, joint })
      .collect();

    if let Some(root) = live.root
      && let Some(live_root) = self.nodes.get(root)
    {
      found.extend(
        live_root
          .joints
          .query_joints(&self.items, bbox, net, layers, kind_mask)
          .into_iter()
          .map(|joint| JointRef { node: root, joint }),
      );
    }

    found
  }

  /// Pin or unpin the joint an item sits on.
  ///
  /// Port of `NODE::LockJoint`, `pcbnew/router/pns_node.cpp:1734`, which
  /// reads the item's layers and net to say which joint is meant and goes
  /// through `touchJoint`, so a position that has no joint yet gets one.
  /// The shove pins the head endpoints with it
  /// (`pcbnew/router/pns_shove.cpp:2492`).
  pub fn lock_joint(
    &mut self,
    node: NodeId,
    pos: Vec2,
    item: ItemId,
    lock: bool,
  ) {
    let Some(item) = self.items.get(item) else {
      return;
    };

    let layers = item.layers();
    let net = item.net();

    self.with_joints(node, |joints, root, _| {
      joints.lock_joint(pos, layers, net, lock, root);
    });
  }

  /// Reset the rank of every locally indexed item and clear marker bits.
  ///
  /// Port of `NODE::ClearRanks`,
  /// `pcbnew/router/pns_node.cpp:1682`, whose default mask is
  /// [`MarkerFlags::CLEARED_BY_CLEAR_RANKS`]
  /// (`pcbnew/router/pns_node.h:494`). It walks the local index only, so
  /// a branch does not touch the root's items.
  pub fn clear_ranks(&mut self, node: NodeId, marker_mask: MarkerFlags) {
    let Some(live) = self.nodes.get(node) else {
      return;
    };

    let indexed: Vec<ItemId> = live.index.items().collect();

    for id in indexed {
      if let Some(item) = self.items.get_mut(id) {
        item.set_rank(Item::UNASSIGNED_RANK);
        item.mark(item.marker().remove(marker_mask));
      }
    }
  }

  /// What a branch changed, as the host has to apply it.
  ///
  /// Port of `NODE::GetUpdatedItems`,
  /// `pcbnew/router/pns_node.cpp:1560`: the overrides are the removals
  /// and the whole local index is the additions. A root answers with two
  /// empty lists (`:1562`).
  ///
  /// The tuple is `(added, removed)`, where KiCad's out parameters are in
  /// the other order. Both lists are in uid order, where KiCad's come out
  /// of an `unordered_set` and reach the commit diff in address order.
  ///
  /// For a deep branch the additions include the items of the
  /// intermediate branches its index was cloned from, exactly as in
  /// KiCad.
  pub fn get_updated_items(&self, node: NodeId) -> (Vec<ItemId>, Vec<ItemId>) {
    let Some(live) = self.nodes.get(node) else {
      return (Vec::new(), Vec::new());
    };

    // :1562
    if live.is_root() {
      return (Vec::new(), Vec::new());
    }

    let mut added: Vec<ItemId> = live.index.items().collect();
    let mut removed: Vec<ItemId> = live.overrides.iter().copied().collect();

    added.sort_by_key(|id| self.uid_of(Some(*id)));
    removed.sort_by_key(|id| self.uid_of(Some(*id)));

    (added, removed)
  }

  /// Register a castellation exclusion zone.
  ///
  /// Port of `NODE::AddEdgeExclusion`,
  /// `pcbnew/router/pns_node.cpp:795`. `Branch` does not copy
  /// `m_edgeExclusions` in KiCad, which makes every branch answer "not
  /// excluded"; the copy is made here, because an exclusion is a property
  /// of the board and not of a routing attempt.
  pub fn add_edge_exclusion(&mut self, node: NodeId, shape: Shape) {
    if let Some(live) = self.nodes.get_mut(node) {
      live.edge_exclusions.push(shape);
    }
  }

  /// Whether a position lies inside a castellation exclusion zone.
  ///
  /// Port of `NODE::QueryEdgeExclusions`,
  /// `pcbnew/router/pns_node.cpp:801`. This is the second half of the
  /// castellation test `crate::collide` cannot finish on its own
  /// (`pcbnew/router/pns_item.cpp:251`): when an obstacle comes back with
  /// a [`Obstacle::detail`], its `location` is what to pass here, and a
  /// `true` answer means KiCad would have suppressed that collision.
  pub fn query_edge_exclusions(&self, node: NodeId, pos: Vec2) -> bool {
    let Some(live) = self.nodes.get(node) else {
      return false;
    };

    live
      .edge_exclusions
      .iter()
      .any(|shape| collision::collide_point(shape, pos, 0).is_some())
  }

  /// An item's uid, for the deterministic orders.
  ///
  /// An unstored or stale handle answers [`u64::MAX`], so it sorts last
  /// rather than colliding with the uid of a live item.
  fn uid_of(&self, id: Option<ItemId>) -> u64 {
    id.and_then(|id| self.items.get(id))
      .map_or(u64::MAX, Item::uid)
  }

  /// Run something on a node's joint map with the root's map alongside.
  ///
  /// Every write on a [`JointMap`] takes `root: Option<&JointMap>`,
  /// because `touchJoint` copies the root's joints into the branch before
  /// mutating them (`pcbnew/router/pns_node.cpp:1406`). Both maps live in
  /// the same arena, so the node's map is lifted out for the duration of
  /// the call and put back afterwards. `None` is passed for the root
  /// itself, which is KiCad's `isRoot()` test as data.
  fn with_joints<R>(
    &mut self,
    node: NodeId,
    action: impl FnOnce(&mut JointMap, Option<&JointMap>, &Arena<Item>) -> R,
  ) -> Option<R> {
    let root = self.nodes.get(node)?.root;
    let mut joints = std::mem::take(&mut self.nodes.get_mut(node)?.joints);

    let result = {
      let root = root.and_then(|root| self.nodes.get(root)).map(Node::joints);

      action(&mut joints, root, &self.items)
    };

    if let Some(live) = self.nodes.get_mut(node) {
      live.joints = joints;
    }

    Some(result)
  }

  // -----------------------------------------------------------------
  // Caches
  // -----------------------------------------------------------------

  /// The clearance between two stored items, cached.
  ///
  /// Port of `NODE::GetClearance` (`pcbnew/router/pns_node.cpp:143`) with
  /// the cache KiCad's reference resolver keeps
  /// (`pcbnew/router/pns_kicad_iface.cpp:92`) folded in, which
  /// `DESIGN.md` section 5 puts in the engine so that a host cannot get
  /// the invalidation wrong. A virtual item is given a clearance of zero
  /// rather than being exempted (`:148`); KiCad's "no resolver, return
  /// 100000" case (`:145`) cannot happen, because the resolver is a
  /// parameter and not an optional pointer.
  ///
  /// The key is the two handles in [`ItemId`] order plus `use_epsilon`,
  /// which is KiCad's canonical ordering by address
  /// (`pcbnew/router/pns_kicad_iface.cpp:99`) with the address hazard
  /// gone: a freed and reallocated slot has a different generation, so it
  /// cannot inherit a stale entry the way KiCad's can. Entries are
  /// dropped when an item leaves the arena or is replaced.
  ///
  /// The answer is **not** per layer: KiCad's resolver takes the maximum
  /// over the shared layers and its cache key has no layer either, which
  /// [`RuleResolver::clearance`] documents.
  ///
  /// `None` for a stale handle, and for a pair the resolver exempts.
  pub fn clearance_between(
    &mut self,
    first: ItemId,
    second: ItemId,
    use_epsilon: bool,
    resolver: &dyn RuleResolver,
  ) -> Option<i32> {
    let key = if first <= second {
      (first, second, use_epsilon)
    } else {
      (second, first, use_epsilon)
    };

    if let Some(cached) = self.clearances.get(&key) {
      return *cached;
    }

    let (Some(a), Some(b)) = (self.items.get(first), self.items.get(second))
    else {
      return None;
    };

    let a = ItemRef::stored(first, a);
    let b = ItemRef::stored(second, b);

    // :148
    let clearance = if a.item().is_virtual() || b.item().is_virtual() {
      Some(0)
    } else {
      resolver.clearance(a, Some(b), use_epsilon)
    };

    self.clearances.insert(key, clearance);

    clearance
  }

  /// The hull of a stored item, cached.
  ///
  /// Port of `RULE_RESOLVER::HullCache`
  /// (`pcbnew/router/pns_kicad_iface.cpp:222`), which keys on the item
  /// pointer, the clearance, the walkaround thickness and the layer, and
  /// returns a reference into its own map that the next insertion can
  /// invalidate (note 02 section 11 entry 6). An [`Rc`] removes that
  /// hazard and lets the walkaround hold on to a hull across further
  /// queries.
  ///
  /// Note that the two callers use different keys for the same geometry:
  /// `NearestObstacle` folds the line's half width into the clearance and
  /// passes a thickness of zero (`pcbnew/router/pns_node.cpp:361`) while
  /// `WALKAROUND::processCluster` passes the line width as the thickness
  /// (`pcbnew/router/pns_walkaround.cpp:157`). Both end up at the same
  /// hull and at two cache entries; the port keeps the two call shapes,
  /// so it keeps the two entries.
  ///
  /// `None` for a stale handle.
  pub fn hull_of(
    &mut self,
    id: ItemId,
    clearance: i32,
    walkaround_thickness: i32,
    layer: i32,
  ) -> Option<Rc<LineChain>> {
    let key = (id, clearance, walkaround_thickness, layer);

    if let Some(cached) = self.hulls.get(&key) {
      return Some(Rc::clone(cached));
    }

    let hull = Rc::new(self.items.get(id)?.hull(
      clearance,
      walkaround_thickness,
      layer,
    ));

    self.hulls.insert(key, Rc::clone(&hull));

    Some(hull)
  }

  /// Drop every cache entry that mentions an item.
  ///
  /// Port of `RULE_RESOLVER::ClearCacheForItems`
  /// (`pcbnew/router/pns_node.cpp:127`, `:1610`), which exists precisely
  /// because a freed and reallocated item can land on an address a cache
  /// still knows. Called when an item leaves the arena.
  fn invalidate_caches(&mut self, id: ItemId) {
    self.clearances.retain(|key, _| key.0 != id && key.1 != id);
    self.hulls.retain(|key, _| key.0 != id);
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::seg::Seg;
  use crate::item::{Segment, Solid, Via, ViaType};
  use crate::rules::FixedClearance;

  /// The clearance every scenario test uses.
  const CLEARANCE: i32 = 2000;

  /// The net every item of the fixture is on.
  const NET: Option<NetId> = Some(NetId(1));

  /// The net the probes are on, so that they are never exempt.
  const OTHER_NET: Option<NetId> = Some(NetId(2));

  /// Where the track's two segments meet, and where the via sits.
  const MIDDLE: Vec2 = Vec2::new(100000, 0);

  /// The handles of the fixture world.
  ///
  /// A two layer board: a pad on layer 0 at the origin, a pad on layer 1
  /// at `(200000, 0)`, a through via halfway between them, and a track of
  /// one segment per layer joining the three.
  struct Fixture {
    /// The pad on layer 0, at the origin.
    pad_bottom: ItemId,
    /// The hole of the pad on layer 0.
    pad_bottom_hole: ItemId,
    /// The pad on layer 1.
    pad_top: ItemId,
    /// The hole of the pad on layer 1.
    pad_top_hole: ItemId,
    /// The through via at [`MIDDLE`].
    via: ItemId,
    /// The hole of the via.
    via_hole: ItemId,
    /// The layer 0 segment, from the origin to [`MIDDLE`].
    lower: ItemId,
    /// The layer 1 segment, from [`MIDDLE`] to the top pad.
    upper: ItemId,
  }

  /// A round pad of radius 2000 on one layer, routable, on [`NET`].
  fn pad(world: &mut World, at: Vec2, layer: i32) -> Item {
    let shape = Shape::circle(at, 2000);
    let mut item = world.make_item(ItemBody::Solid(Solid::new(shape, at)));
    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(NET);

    item
  }

  /// A track of width 1000 on one layer.
  fn track(
    world: &mut World,
    from: Vec2,
    to: Vec2,
    layer: i32,
    net: Option<NetId>,
  ) -> Item {
    let body = ItemBody::Segment(Segment::new(Seg::new(from, to), 1000));
    let mut item = world.make_item(body);
    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(net);

    item
  }

  /// The fixture, built into a fresh world.
  ///
  /// The order matters: the via is added **before** the two segments, so
  /// that it is first in the link list of the joint at [`MIDDLE`], which
  /// is what lets removing it split that joint back into one joint per
  /// layer; see `JointMap::rebuild_joint`.
  fn fixture() -> (World, Fixture) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    let bottom = pad(&mut world, Vec2::new(0, 0), 0);
    let pad_bottom =
      world.add_solid(root, bottom, Some(Hole::circular(Vec2::new(0, 0), 500)));
    let pad_bottom_hole = world
      .item(pad_bottom)
      .and_then(Item::hole)
      .expect("the pad was drilled");

    let top = pad(&mut world, Vec2::new(200000, 0), 1);
    let pad_top = world.add_solid(
      root,
      top,
      Some(Hole::circular(Vec2::new(200000, 0), 500)),
    );
    let pad_top_hole = world
      .item(pad_top)
      .and_then(Item::hole)
      .expect("the pad was drilled");

    let body = ItemBody::Via(Via::new(MIDDLE, 3000, 1000, ViaType::Through));
    let mut item = world.make_item(body);
    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item.set_net(NET);
    let via = world.add_via(root, item);
    let via_hole = world
      .item(via)
      .and_then(Item::hole)
      .expect("a via is drilled");

    let lower_item = track(&mut world, Vec2::new(0, 0), MIDDLE, 0, NET);
    let lower = world
      .add_segment(root, lower_item, false)
      .expect("the lower segment is neither degenerate nor redundant");

    let upper_item = track(&mut world, MIDDLE, Vec2::new(200000, 0), 1, NET);
    let upper = world
      .add_segment(root, upper_item, false)
      .expect("the upper segment is neither degenerate nor redundant");

    (
      world,
      Fixture {
        pad_bottom,
        pad_bottom_hole,
        pad_top,
        pad_top_hole,
        via,
        via_hole,
        lower,
        upper,
      },
    )
  }

  /// An unstored probe track, the thing a placer drags.
  fn probe(from: Vec2, to: Vec2, layer: i32) -> Item {
    let body = ItemBody::Segment(Segment::new(Seg::new(from, to), 1000));
    let mut item = Item::new(u64::MAX, body);
    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(OTHER_NET);

    item
  }

  /// The items a probe collides with in a node, in the order reported.
  fn obstacles(
    world: &World,
    node: NodeId,
    head: &Item,
    options: &CollisionSearchOptions,
  ) -> Vec<ItemId> {
    world
      .query_colliding(
        node,
        ItemRef::unstored(head),
        &FixedClearance::uniform(CLEARANCE),
        options,
      )
      .into_iter()
      .filter_map(|obstacle| obstacle.item)
      .collect()
  }

  // -----------------------------------------------------------------
  // Joints
  // -----------------------------------------------------------------

  #[test]
  fn the_fixture_links_a_joint_at_every_anchor() {
    let (world, ids) = fixture();
    let root = world.root();

    let start = world
      .find_joint(root, Vec2::new(0, 0), 0, NET)
      .expect("the pad and the lower segment meet at the origin");
    let start = world.joint(start).expect("the joint was just found");

    assert_eq!(start.links(), [ids.pad_bottom, ids.lower]);
    assert_eq!(start.layers(), LayerRange::single(0));

    let end = world
      .find_joint(root, Vec2::new(200000, 0), 1, NET)
      .expect("the pad and the upper segment meet at the far end");
    let end = world.joint(end).expect("the joint was just found");

    assert_eq!(end.links(), [ids.pad_top, ids.upper]);
  }

  /// The via binds the two layers into one joint, so the same joint
  /// answers on both.
  #[test]
  fn the_via_joint_covers_both_layers() {
    let (world, ids) = fixture();
    let root = world.root();

    let bottom = world
      .find_joint(root, MIDDLE, 0, NET)
      .expect("the via is on layer 0");
    let top = world
      .find_joint(root, MIDDLE, 1, NET)
      .expect("the via is on layer 1");

    assert_eq!(bottom, top);

    let joint = world.joint(bottom).expect("the joint was just found");

    assert_eq!(joint.links(), [ids.via, ids.lower, ids.upper]);
    assert_eq!(joint.layers(), LayerRange::new(0, 1));
    assert!(joint.is_non_fanout_via(world.items()));
  }

  #[test]
  fn find_line_ends_answers_with_both_joints_of_a_segment() {
    let (world, _) = fixture();
    let root = world.root();

    let ends = world.find_line_ends(
      root,
      Vec2::new(0, 0),
      MIDDLE,
      LayerRange::single(0),
      NET,
    );
    let (start, end) = ends.expect("both ends carry a joint");

    assert_ne!(start, end);
    assert_eq!(world.joint(start).map(Joint::pos), Some(Vec2::new(0, 0)));
    assert_eq!(world.joint(end).map(Joint::pos), Some(MIDDLE));
  }

  /// Port test of `NODE::removeViaIndex` plus `rebuildJoint`
  /// (`pcbnew/router/pns_node.cpp:931`, `:870`): the joint that bound the
  /// two layers together comes apart into one joint per layer, each
  /// holding the segment that lives there.
  #[test]
  fn removing_a_via_splits_its_joint_per_layer() {
    let (mut world, ids) = fixture();
    let root = world.root();

    world.remove(root, ids.via);

    let bottom = world
      .find_joint(root, MIDDLE, 0, NET)
      .expect("the lower segment still ends here");
    let top = world
      .find_joint(root, MIDDLE, 1, NET)
      .expect("the upper segment still ends here");

    assert_ne!(bottom, top);
    assert_eq!(
      world.joint(bottom).map(Joint::links),
      Some([ids.lower].as_slice())
    );
    assert_eq!(
      world.joint(top).map(Joint::links),
      Some([ids.upper].as_slice())
    );

    // The via and the hole it owned are gone from the arena, because the
    // root is where they were added.
    assert!(world.item(ids.via).is_none());
    assert!(world.item(ids.via_hole).is_none());
  }

  // -----------------------------------------------------------------
  // Branching
  // -----------------------------------------------------------------

  #[test]
  fn a_branch_of_the_root_starts_empty_and_defers_to_it() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    assert_eq!(world.depth(branch), Some(1));
    assert_eq!(world.root_of(branch), Some(root));
    assert!(!world.is_root(branch));
    assert_eq!(world.parent(branch), Some(root));

    let live = world.node(branch).expect("the branch is live");

    assert!(live.index().is_empty());
    assert!(live.joints().is_empty());
    assert!(live.overrides().is_empty());
    assert_eq!(live.max_clearance(), World::DEFAULT_MAX_CLEARANCE);

    // The root's items are still visible from it. The holes are in the
    // net bucket too, because `INDEX::Add` reads the virtual `Net()`
    // that a hole forwards to its parent
    // (`pcbnew/router/pns_index.cpp:50`, `pcbnew/router/pns_hole.h:56`).
    assert_eq!(
      world.all_items_in_net(branch, NET, Kind::ANY),
      vec![
        ids.pad_bottom,
        ids.pad_bottom_hole,
        ids.pad_top,
        ids.pad_top_hole,
        ids.via,
        ids.via_hole,
        ids.lower,
        ids.upper,
      ]
    );
  }

  #[test]
  fn a_query_from_a_branch_sees_root_items_and_branch_items() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    let extra = track(
      &mut world,
      Vec2::new(40000, 10000),
      Vec2::new(60000, 10000),
      0,
      Some(NetId(3)),
    );
    let extra = world
      .add_segment(branch, extra, false)
      .expect("the branch segment is fine");

    let head = probe(Vec2::new(50000, -50000), Vec2::new(50000, 50000), 0);
    let options = CollisionSearchOptions::default();

    assert_eq!(
      obstacles(&world, branch, &head, &options),
      vec![ids.lower, extra]
    );
    // The root knows nothing of the branch's segment.
    assert_eq!(obstacles(&world, root, &head, &options), vec![ids.lower]);
  }

  /// Port test of `doRemove`'s first case
  /// (`pcbnew/router/pns_node.cpp:815`): a root item removed from a
  /// branch is shadowed there and untouched everywhere else.
  #[test]
  fn removing_a_root_item_in_a_branch_only_shadows_it() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    world.remove(branch, ids.lower);

    assert!(
      world
        .node(branch)
        .expect("the branch is live")
        .is_overridden(ids.lower)
    );
    // Still in the arena, still owned by the root, still visible there.
    assert!(world.item(ids.lower).is_some());
    assert_eq!(world.home_of(ids.lower), Some(root));

    let head = probe(Vec2::new(50000, -50000), Vec2::new(50000, 50000), 0);
    let options = CollisionSearchOptions::default();

    assert!(obstacles(&world, branch, &head, &options).is_empty());
    assert_eq!(obstacles(&world, root, &head, &options), vec![ids.lower]);
    assert!(world.hit_test(branch, Vec2::new(50000, 0)).is_empty());
    assert_eq!(world.hit_test(root, Vec2::new(50000, 0)), vec![ids.lower]);
  }

  /// Removing a pad shadows its hole as well
  /// (`pcbnew/router/pns_node.cpp:819`), so the hole does not survive the
  /// pad in the branch's view.
  #[test]
  fn shadowing_a_pad_shadows_the_hole_it_owns() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    world.remove(branch, ids.pad_bottom);

    let live = world.node(branch).expect("the branch is live");

    assert!(live.is_overridden(ids.pad_bottom));
    assert!(live.is_overridden(ids.pad_bottom_hole));
    assert!(world.hit_test(branch, Vec2::new(0, 1000)).is_empty());
    assert_eq!(
      world.hit_test(root, Vec2::new(0, 1000)),
      vec![ids.pad_bottom]
    );
  }

  #[test]
  fn a_branch_of_a_branch_clones_the_overlay() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let first = world.branch(root);

    let extra = track(
      &mut world,
      MIDDLE,
      Vec2::new(100000, 50000),
      0,
      Some(NetId(3)),
    );
    let extra = world
      .add_segment(first, extra, false)
      .expect("the branch segment is fine");
    world.remove(first, ids.lower);

    let second = world.branch(first);
    let live = world.node(second).expect("the branch is live");

    assert_eq!(world.depth(second), Some(2));
    assert_eq!(world.root_of(second), Some(root));
    assert!(live.index().contains(extra));
    assert!(live.is_overridden(ids.lower));
    // The copied index does not move ownership.
    assert!(live.home().is_empty());
    assert_eq!(world.home_of(extra), Some(first));
  }

  #[test]
  fn kill_children_drops_a_branch_and_the_items_it_owns() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let first = world.branch(root);
    let extra = track(
      &mut world,
      MIDDLE,
      Vec2::new(100000, 50000),
      0,
      Some(NetId(3)),
    );
    let extra = world
      .add_segment(first, extra, false)
      .expect("the branch segment is fine");
    let second = world.branch(first);

    world.kill_children(root);

    assert!(world.node(first).is_none());
    assert!(world.node(second).is_none());
    assert!(world.item(extra).is_none());
    assert!(
      world
        .node(root)
        .expect("the root survives")
        .children()
        .is_empty()
    );
    // The root's own world is untouched.
    assert!(world.item(ids.lower).is_some());
    assert_eq!(
      world.all_items_in_net(root, NET, Kind::SEGMENT | Kind::VIA),
      vec![ids.via, ids.lower, ids.upper]
    );
  }

  // -----------------------------------------------------------------
  // Commit
  // -----------------------------------------------------------------

  #[test]
  fn commit_folds_both_the_addition_and_the_removal_into_the_root() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    let extra = track(
      &mut world,
      Vec2::new(40000, 10000),
      Vec2::new(60000, 10000),
      0,
      Some(NetId(3)),
    );
    let extra = world
      .add_segment(branch, extra, false)
      .expect("the branch segment is fine");
    world.remove(branch, ids.lower);

    world.commit(branch);

    assert!(world.node(branch).is_none());
    assert!(world.item(ids.lower).is_none());
    assert!(world.item(extra).is_some());
    assert_eq!(world.home_of(extra), Some(root));

    let head = probe(Vec2::new(50000, -50000), Vec2::new(50000, 50000), 0);
    let options = CollisionSearchOptions::default();

    assert_eq!(obstacles(&world, root, &head, &options), vec![extra]);
    assert_eq!(
      world.all_items_in_net(root, Some(NetId(3)), Kind::ANY),
      vec![extra]
    );
    // The committed segment brought its joints with it.
    assert!(
      world
        .find_joint(root, Vec2::new(40000, 10000), 0, Some(NetId(3)))
        .is_some()
    );
  }

  /// Port test of `pcbnew/router/pns_node.cpp:1637` and `:1638`. Note
  /// that `Unmark()`'s default argument clears **every** bit, the lock
  /// included.
  #[test]
  fn commit_resets_the_rank_and_clears_every_marker() {
    let (mut world, _) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    let extra = track(
      &mut world,
      Vec2::new(40000, 10000),
      Vec2::new(60000, 10000),
      0,
      Some(NetId(3)),
    );
    let extra = world
      .add_segment(branch, extra, false)
      .expect("the branch segment is fine");

    if let Some(item) = world.item_mut(extra) {
      item.set_rank(7);
      item.mark(MarkerFlags::HEAD | MarkerFlags::LOCKED);
    }

    world.commit(branch);

    let item = world.item(extra).expect("the segment was committed");

    assert_eq!(item.rank(), Item::UNASSIGNED_RANK);
    assert!(item.marker().is_empty());
  }

  #[test]
  fn get_updated_items_reports_the_branch_delta() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    assert_eq!(world.get_updated_items(root), (Vec::new(), Vec::new()));

    let extra = track(
      &mut world,
      Vec2::new(40000, 10000),
      Vec2::new(60000, 10000),
      0,
      Some(NetId(3)),
    );
    let extra = world
      .add_segment(branch, extra, false)
      .expect("the branch segment is fine");
    world.remove(branch, ids.pad_bottom);

    let (added, removed) = world.get_updated_items(branch);

    assert_eq!(added, vec![extra]);
    assert_eq!(removed, vec![ids.pad_bottom, ids.pad_bottom_hole]);
  }

  // -----------------------------------------------------------------
  // Collision queries
  // -----------------------------------------------------------------

  #[test]
  fn check_colliding_answers_only_for_a_probe_that_is_too_close() {
    let (world, ids) = fixture();
    let root = world.root();
    let rules = FixedClearance::uniform(CLEARANCE);
    let options = CollisionSearchOptions::default();

    let crossing = probe(Vec2::new(50000, -50000), Vec2::new(50000, 50000), 0);
    let found = world
      .check_colliding(root, ItemRef::unstored(&crossing), &rules, &options)
      .expect("the probe crosses the lower segment");

    assert_eq!(found.item, Some(ids.lower));
    assert_eq!(found.head, None);
    assert_eq!(found.clearance, CLEARANCE);

    // 20000 nm above the track's spine, so 19000 nm of copper to copper
    // gap against a clearance of 2000.
    let clear = probe(Vec2::new(50000, 20000), Vec2::new(50000, 50000), 0);

    assert!(
      world
        .check_colliding(root, ItemRef::unstored(&clear), &rules, &options)
        .is_none()
    );
  }

  /// A probe on layer 1 cannot see the layer 0 track, which is the layer
  /// filtering the per layer sub indices perform.
  #[test]
  fn a_probe_on_another_layer_finds_nothing() {
    let (world, _) = fixture();
    let root = world.root();
    let options = CollisionSearchOptions::default();
    let head = probe(Vec2::new(50000, -50000), Vec2::new(50000, 50000), 1);

    assert!(obstacles(&world, root, &head, &options).is_empty());
  }

  #[test]
  fn the_limit_count_stops_the_search() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    let extra = track(
      &mut world,
      Vec2::new(40000, 10000),
      Vec2::new(60000, 10000),
      0,
      Some(NetId(3)),
    );
    let extra = world
      .add_segment(branch, extra, false)
      .expect("the branch segment is fine");

    let head = probe(Vec2::new(50000, -50000), Vec2::new(50000, 50000), 0);
    let unlimited = CollisionSearchOptions::default();
    let limited = CollisionSearchOptions {
      limit_count: Some(1),
      ..CollisionSearchOptions::default()
    };

    assert_eq!(
      obstacles(&world, branch, &head, &unlimited),
      vec![ids.lower, extra]
    );
    // The local pass runs first and stops there, so the root is never
    // searched.
    assert_eq!(obstacles(&world, branch, &head, &limited), vec![extra]);
  }

  #[test]
  fn the_kind_mask_filters_the_candidates() {
    let (world, ids) = fixture();
    let root = world.root();
    let options = CollisionSearchOptions {
      kind_mask: Kind::SOLID,
      ..CollisionSearchOptions::default()
    };
    let head = probe(Vec2::new(0, 0), Vec2::new(0, 30000), 0);

    // The track on the same layer is filtered out. The pad's hole is
    // **not**, even though it is not a solid: the mask filters the
    // candidates the index offers (`pcbnew/router/pns_node.cpp:243`) and
    // the hole is reported by the accumulating collision test recursing
    // into the candidate's hole (`pcbnew/router/pns_item.cpp:154`).
    assert_eq!(
      obstacles(&world, root, &head, &options),
      vec![ids.pad_bottom, ids.pad_bottom_hole]
    );
  }

  #[test]
  fn a_virtual_head_collides_with_nothing() {
    let (world, _) = fixture();
    let root = world.root();
    let options = CollisionSearchOptions::default();
    let mut head = probe(Vec2::new(50000, -50000), Vec2::new(50000, 50000), 0);
    head.set_is_virtual(true);

    assert!(obstacles(&world, root, &head, &options).is_empty());
  }

  // -----------------------------------------------------------------
  // Hit test and the other read paths
  // -----------------------------------------------------------------

  #[test]
  fn hit_test_answers_at_a_pad_and_not_in_empty_space() {
    let (world, ids) = fixture();
    let root = world.root();

    // Inside the pad's copper, outside its hole and clear of the track.
    assert_eq!(
      world.hit_test(root, Vec2::new(0, 1000)),
      vec![ids.pad_bottom]
    );
    // Dead centre, where the pad, its hole and the track all sit.
    assert_eq!(
      world.hit_test(root, Vec2::new(0, 0)),
      vec![ids.pad_bottom, ids.pad_bottom_hole, ids.lower]
    );
    assert!(world.hit_test(root, Vec2::new(50000, 50000)).is_empty());
  }

  #[test]
  fn all_items_in_net_skips_the_other_nets_and_the_unroutable() {
    let (mut world, ids) = fixture();
    let root = world.root();

    assert_eq!(
      world.all_items_in_net(root, NET, Kind::SEGMENT),
      vec![ids.lower, ids.upper]
    );
    assert!(
      world
        .all_items_in_net(root, OTHER_NET, Kind::ANY)
        .is_empty()
    );

    if let Some(item) = world.item_mut(ids.pad_top) {
      item.set_routable(false);
    }

    // The unroutable pad drops out, its hole does not: a hole carries its
    // own routable flag, and nothing clears it.
    assert_eq!(
      world.all_items_in_net(root, NET, Kind::SOLID | Kind::HOLE),
      vec![
        ids.pad_bottom,
        ids.pad_bottom_hole,
        ids.pad_top_hole,
        ids.via_hole,
      ]
    );
  }

  #[test]
  fn query_joints_combines_the_branch_and_the_root() {
    let (mut world, _) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    let extra = track(
      &mut world,
      MIDDLE,
      Vec2::new(100000, 50000),
      0,
      Some(NetId(3)),
    );
    world
      .add_segment(branch, extra, false)
      .expect("the branch segment is fine");

    let box_all = Box2::from_vec2_corners(
      Vec2::new(-10000, -10000),
      Vec2::new(300000, 100000),
    );
    let found = world.query_joints(
      branch,
      box_all,
      None,
      LayerRange::new(0, 1),
      Kind::ANY,
    );

    let positions: Vec<Vec2> = found
      .iter()
      .filter_map(|joint| world.joint(*joint).map(Joint::pos))
      .collect();

    // The branch's own two joints first, in position order, then the
    // root's three.
    assert_eq!(
      positions,
      vec![
        MIDDLE,
        Vec2::new(100000, 50000),
        Vec2::new(0, 0),
        MIDDLE,
        Vec2::new(200000, 0),
      ]
    );
  }

  #[test]
  fn clear_ranks_and_remove_by_marker_walk_the_local_index_only() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let branch = world.branch(root);

    let extra = track(
      &mut world,
      Vec2::new(40000, 10000),
      Vec2::new(60000, 10000),
      0,
      Some(NetId(3)),
    );
    let extra = world
      .add_segment(branch, extra, false)
      .expect("the branch segment is fine");

    if let Some(item) = world.item_mut(ids.lower) {
      item.set_rank(4);
      item.mark(MarkerFlags::VIOLATION);
    }

    if let Some(item) = world.item_mut(extra) {
      item.set_rank(9);
      item.mark(MarkerFlags::VIOLATION | MarkerFlags::LOCKED);
    }

    world.clear_ranks(branch, MarkerFlags::CLEARED_BY_CLEAR_RANKS);

    // The root's item is untouched, the branch's is reset and keeps the
    // bits outside the mask.
    assert_eq!(world.item(ids.lower).map(Item::rank), Some(4));
    assert_eq!(
      world.item(extra).map(Item::marker),
      Some(MarkerFlags::LOCKED)
    );
    assert_eq!(world.item(extra).map(Item::rank), Some(-1));

    if let Some(item) = world.item_mut(extra) {
      item.mark(MarkerFlags::VIOLATION);
    }

    world.remove_by_marker(branch, MarkerFlags::VIOLATION);

    assert!(world.item(extra).is_none());
    assert!(world.item(ids.lower).is_some());
  }

  #[test]
  fn a_zero_length_or_redundant_segment_is_refused() {
    let (mut world, ids) = fixture();
    let root = world.root();

    let degenerate = track(&mut world, MIDDLE, MIDDLE, 0, Some(NetId(3)));

    assert!(world.add_segment(root, degenerate, false).is_none());

    let duplicate = track(&mut world, MIDDLE, Vec2::new(0, 0), 0, NET);

    assert!(world.add_segment(root, duplicate, false).is_none());

    let duplicate = track(&mut world, MIDDLE, Vec2::new(0, 0), 0, NET);
    let allowed = world
      .add_segment(root, duplicate, true)
      .expect("the redundancy check was waived");

    assert_ne!(allowed, ids.lower);
  }

  #[test]
  fn replace_swaps_the_item_and_reindexes_it() {
    let (mut world, ids) = fixture();
    let root = world.root();

    let moved = track(
      &mut world,
      Vec2::new(0, 30000),
      Vec2::new(100000, 30000),
      0,
      NET,
    );
    let moved = world.replace(root, ids.lower, moved);

    assert!(world.item(ids.lower).is_none());
    assert_eq!(world.hit_test(root, Vec2::new(50000, 0)), Vec::new());
    assert_eq!(world.hit_test(root, Vec2::new(50000, 30000)), vec![moved]);
  }

  // -----------------------------------------------------------------
  // Caches
  // -----------------------------------------------------------------

  #[test]
  fn the_caches_answer_twice_and_forget_a_removed_item() {
    let (mut world, ids) = fixture();
    let root = world.root();
    let rules = FixedClearance::uniform(CLEARANCE);

    let other = track(
      &mut world,
      Vec2::new(40000, 10000),
      Vec2::new(60000, 10000),
      0,
      Some(NetId(3)),
    );
    let other = world
      .add_segment(root, other, false)
      .expect("the extra segment is fine");

    let first = world.clearance_between(ids.lower, other, false, &rules);
    // The key is canonically ordered, so the mirrored question is the
    // same entry (`pcbnew/router/pns_kicad_iface.cpp:99`).
    let again = world.clearance_between(other, ids.lower, false, &rules);

    assert_eq!(first, Some(CLEARANCE));
    assert_eq!(again, first);
    // Two items of one net can never collide, which is the resolver's
    // negative answer and not a missing entry.
    assert_eq!(
      world.clearance_between(ids.lower, ids.upper, false, &rules),
      None
    );

    let hull = world
      .hull_of(ids.lower, CLEARANCE, 0, 0)
      .expect("a track has a hull");
    let cached = world.hull_of(ids.lower, CLEARANCE, 0, 0).expect("cached");

    assert!(Rc::ptr_eq(&hull, &cached));
    assert!(hull.is_closed());

    world.remove(root, ids.lower);

    assert!(world.hull_of(ids.lower, CLEARANCE, 0, 0).is_none());
    assert!(
      world
        .clearance_between(ids.lower, other, false, &rules)
        .is_none()
    );
  }

  /// A virtual item is given a clearance of zero rather than being
  /// exempted, `pcbnew/router/pns_node.cpp:148`.
  #[test]
  fn a_virtual_item_gets_a_clearance_of_zero() {
    let (mut world, ids) = fixture();
    let rules = FixedClearance::uniform(CLEARANCE);

    if let Some(item) = world.item_mut(ids.upper) {
      item.set_is_virtual(true);
    }

    assert_eq!(
      world.clearance_between(ids.lower, ids.upper, false, &rules),
      Some(0)
    );
  }

  // -----------------------------------------------------------------
  // Edge exclusions
  // -----------------------------------------------------------------

  #[test]
  fn an_edge_exclusion_answers_for_the_positions_inside_it() {
    let (mut world, _) = fixture();
    let root = world.root();

    world.add_edge_exclusion(root, Shape::circle(Vec2::new(0, 0), 1000));

    assert!(world.query_edge_exclusions(root, Vec2::new(0, 500)));
    assert!(!world.query_edge_exclusions(root, Vec2::new(0, 5000)));

    // A branch inherits them, where KiCad's `Branch` does not copy the
    // vector at all.
    let branch = world.branch(root);

    assert!(world.query_edge_exclusions(branch, Vec2::new(0, 500)));
  }

  // -----------------------------------------------------------------
  // Determinism
  // -----------------------------------------------------------------

  /// `DESIGN.md` section 8: two identical runs answer identically, down
  /// to the order of every list.
  #[test]
  fn every_answer_is_the_same_across_two_identical_runs() {
    /// Everything one run answers, so that two runs can be compared as
    /// one value.
    #[derive(PartialEq, Eq, Debug)]
    struct Answers {
      /// What a probe collides with in the branch.
      obstacles: Vec<ItemId>,
      /// What sits under the via's position.
      hits: Vec<ItemId>,
      /// The net the fixture is routed on, as the branch sees it.
      net: Vec<ItemId>,
      /// The branch delta, added then removed.
      updated: (Vec<ItemId>, Vec<ItemId>),
      /// Where the joints of the branch and the root are.
      joints: Vec<Vec2>,
    }

    /// One run's worth of answers.
    fn run() -> Answers {
      let (mut world, ids) = fixture();
      let root = world.root();
      let branch = world.branch(root);

      let extra = track(
        &mut world,
        Vec2::new(40000, 10000),
        Vec2::new(60000, 10000),
        0,
        Some(NetId(3)),
      );
      world
        .add_segment(branch, extra, false)
        .expect("the branch segment is fine");
      world.remove(branch, ids.pad_bottom);

      let head = probe(Vec2::new(50000, -50000), Vec2::new(50000, 50000), 0);
      let options = CollisionSearchOptions::default();

      let box_all = Box2::from_vec2_corners(
        Vec2::new(-10000, -10000),
        Vec2::new(300000, 100000),
      );
      let joints = world
        .query_joints(branch, box_all, None, LayerRange::new(0, 1), Kind::ANY)
        .into_iter()
        .filter_map(|joint| world.joint(joint).map(Joint::pos))
        .collect();

      Answers {
        obstacles: obstacles(&world, branch, &head, &options),
        hits: world.hit_test(branch, MIDDLE),
        net: world.all_items_in_net(branch, NET, Kind::ANY),
        updated: world.get_updated_items(branch),
        joints,
      }
    }

    assert_eq!(run(), run());
  }
}
