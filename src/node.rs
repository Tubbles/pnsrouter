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
//! deduplicated by item; item lists come back sorted by uid.
//! [`World::nearest_obstacle`] scans that same order and breaks a
//! distance tie on it, which is the `(distance, uid)` key `DESIGN.md`
//! section 8 asks for.
//!
//! # Members not ported
//!
//! - `LINE::ClipToNearestObstacle` (`pcbnew/router/pns_line.cpp:679`).
//!   It is the one consumer of [`World::nearest_obstacle`] with no caller
//!   at all in this KiCad revision: a grep of `pcbnew/router` finds its
//!   definition and its declaration and nothing else. Port it when
//!   something needs it.
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
//! - `FindLineEnds`'s use inside `FindLinesBetweenJoints`
//!   (`pcbnew/router/pns_node.cpp:1237`), whose two outputs are never
//!   read. The public [`World::find_line_ends`] is here.
//!
//! `AddEdgeExclusion` and `QueryEdgeExclusions`
//! (`pcbnew/router/pns_node.cpp:795`, `:801`) **are** here, small as they
//! are, because `crate::collide` cannot finish the castellation test
//! without them; see [`World::query_edge_exclusions`] for how the
//! collision ladder is meant to reach them.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::rc::Rc;

use crate::arena::{Arena, ArenaId};
use crate::collide::{
  CollisionSearchOptions, LineHead, Obstacle, collide_into, collide_line_items,
};
use crate::geometry::arc::ShapeArc;
use crate::geometry::box2::Box2;
use crate::geometry::collision;
use crate::geometry::direction45::CornerMode;
use crate::geometry::hull::hull_intersection;
use crate::geometry::line_chain::LineChain;
use crate::geometry::shape::Shape;
use crate::geometry::vec2::Vec2;
use crate::index::Index;
use crate::item::{
  Arc, Hole, Item, ItemBody, ItemId, Kind, LayerRange, MarkerFlags, NetId,
  UidCounter,
};
use crate::joint::{Joint, JointId, JointMap};
use crate::line::Line;
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
  /// How many threads [`World::nearest_obstacle`] may scan candidates
  /// on. See [`World::set_parallelism`].
  parallelism: usize,
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
      parallelism: 1,
    }
  }

  /// The largest thread count worth asking [`World::set_parallelism`]
  /// for.
  ///
  /// KiCad submits the scan to a process wide pool and so inherits its
  /// size (`pcbnew/router/pns_node.cpp:437`). There is no pool here, one
  /// [`std::thread::scope`] per query instead, and a scope of `n` blocks
  /// costs about 21 microseconds per block in the container
  /// `doc/performance.md` describes: 42 for two, 169 for eight. One
  /// candidate costs about 2.2 microseconds to scan. Past eight blocks
  /// the dispatch is the query.
  pub const MAX_USEFUL_PARALLELISM: usize = 8;

  /// How many threads [`World::nearest_obstacle`] may scan candidates on.
  ///
  /// One means sequential, and one is the **default**: measured on both
  /// boards of `doc/performance.md`, spawning threads per query made
  /// every tail metric worse and nothing better, because a
  /// [`std::thread::scope`] costs more than the geometry it moves. The
  /// parallel obstacle query section there has the numbers. A host on
  /// hardware where thread creation is cheaper, or on boards denser than
  /// those two, can turn it on with
  /// `world.set_parallelism(std::thread::available_parallelism().map_or(1, NonZero::get))`,
  /// and should measure before it does. Zero is read as one, and there
  /// is no point going above [`World::MAX_USEFUL_PARALLELISM`].
  ///
  /// The knob is here and not on [`crate::settings::RoutingSettings`] on
  /// purpose: settings are serialised into a session recording
  /// (`crate::eventlog`) and a thread count is a property of the machine
  /// replaying it, not of the session. The answer does not depend on it,
  /// which is what `tests/parallelism.rs` pins.
  pub const fn set_parallelism(&mut self, threads: usize) {
    self.parallelism = if threads == 0 { 1 } else { threads };
  }

  /// How many threads the obstacle scan may use. See
  /// [`World::set_parallelism`].
  pub const fn parallelism(&self) -> usize {
    self.parallelism
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

  /// Drop one node, its whole subtree, and everything they own.
  ///
  /// The single node form of [`World::kill_children`]: it releases the
  /// node itself as well as its children and unlinks it from its parent's
  /// child list. C++ spells it `delete aNode`, whose destructor performs
  /// the same subtree release (`pcbnew/router/pns_node.cpp:91`); the line
  /// placer does exactly that to its scratch branch once per mouse move
  /// (`pcbnew/router/pns_line_placer.cpp:1492`), and note 03 section 9.3
  /// asks for it under the name `drop_subtree`. Dropping only the
  /// children would take the placer's siblings with it, which is why
  /// [`World::kill_children`] is not enough.
  ///
  /// Dropping a root is refused: a world without a root has no meaning,
  /// and KiCad never deletes its own.
  pub fn drop_node(&mut self, node: NodeId) {
    let Some(live) = self.nodes.get(node) else {
      return;
    };

    if live.is_root() {
      return;
    }

    let parent = live.parent;

    self.release_node(node);

    if let Some(parent) = parent
      && let Some(live) = self.nodes.get_mut(parent)
    {
      live.children.retain(|child| *child != node);
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
      // `:662`, the `ARC_T` case, which like `SEGMENT_T` goes straight to
      // the private adder and performs none of `Add( unique_ptr<ARC> )`'s
      // redundancy check.
      ItemBody::Arc(_) => {
        let id = self.items.insert(item);
        self.do_add_arc(node, id);

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

  /// Add a curved track.
  ///
  /// Port of `NODE::Add( unique_ptr<ARC>, bool )`,
  /// `pcbnew/router/pns_node.cpp:776`, which refuses an arc that
  /// duplicates one already linked to the joint at its start unless
  /// `allow_redundant` is set (`:780`), and otherwise calls `addArc`
  /// (`:765`), which links the joints at both anchors and indexes it.
  ///
  /// # Erratum E36, reproduced: no degenerate check
  ///
  /// `Add( SEGMENT )` refuses a segment whose two ends coincide
  /// (`:749`); this overload has no counterpart, so an arc whose three
  /// points coincide goes in. That is defined behaviour rather than a
  /// crash, so the milestone rule says reproduce it, and the consequence
  /// is smaller than the erratum's text suggests: the two `linkJoint`
  /// calls land on one position and [`Joint::link`] refuses the second
  /// (`pcbnew/router/pns_joint.h:213`), so the joint holds the arc once
  /// and [`World::remove`] takes it out again cleanly. What is left is an
  /// arc in the index with a point sized bounding box, which collides
  /// with whatever comes within a clearance of that point. See
  /// `doc/log/2026-09-12.md`.
  ///
  /// # Panics
  ///
  /// When the item is not an arc.
  pub fn add_arc(
    &mut self,
    node: NodeId,
    arc: Item,
    allow_redundant: bool,
  ) -> Option<ItemId> {
    let ItemBody::Arc(body) = arc.body() else {
      panic!("add_arc needs an arc body");
    };

    let shape = body.arc();

    // :780
    if !allow_redundant
      && self
        .find_redundant_arc(node, shape, arc.layers(), arc.net())
        .is_some()
    {
      return None;
    }

    let id = self.items.insert(arc);
    self.do_add_arc(node, id);

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
      ItemBody::Arc(_) => self.do_add_arc(node, id),
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

  /// Port of `NODE::addArc`, `pcbnew/router/pns_node.cpp:765`, which is
  /// `addSegment` with the two anchors in place of the two segment ends.
  fn do_add_arc(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    let ItemBody::Arc(arc) = item.body() else {
      return;
    };

    let (start, end) = (arc.anchor(0), arc.anchor(1));
    let layers = item.layers();
    let net = item.net();

    self.with_joints(node, |joints, root, _| {
      joints.link_joint(start, layers, net, id, root);
      joints.link_joint(end, layers, net, id, root);
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

  /// An arc already linked to the joint at the start with the same
  /// geometry.
  ///
  /// Port of `NODE::findRedundantArc`,
  /// `pcbnew/router/pns_node.cpp:1742`. The endpoints may be the other
  /// way round and the layer test is on the range's start only, both as
  /// [`World::find_redundant_segment`]'s are.
  ///
  /// # Erratum E8, fixed: the mid point is compared too
  ///
  /// KiCad compares the two anchors and nothing else (`:1760`), so an arc
  /// bulging the other way between the same two endpoints reads as
  /// redundant and is silently reused in its place. Two arcs that differ
  /// only in their mid point are two different tracks and a board can
  /// hold both, so this compares the mid point as well, reversing it with
  /// the endpoints. `doc/work/012-arcs.md` asks for the fix; the test
  /// naming it is `find_redundant_arc_compares_the_mid_point_erratum_e8`.
  fn find_redundant_arc(
    &self,
    node: NodeId,
    arc: ShapeArc,
    layers: LayerRange,
    net: Option<NetId>,
  ) -> Option<ItemId> {
    let start = self.find_joint(node, arc.start(), layers.start(), net)?;
    let joint = self.joint(start)?;

    joint
      .links()
      .iter()
      .copied()
      .find(|id| self.is_same_arc(*id, arc, layers))
  }

  /// Whether a stored item is an arc with that geometry on that layer.
  ///
  /// The body of the loop in `findRedundantArc`,
  /// `pcbnew/router/pns_node.cpp:1752` to `:1763`, with erratum E8's mid
  /// point added; see [`World::find_redundant_arc`].
  fn is_same_arc(&self, id: ItemId, arc: ShapeArc, layers: LayerRange) -> bool {
    let Some(item) = self.items.get(id) else {
      return false;
    };

    let ItemBody::Arc(stored) = item.body() else {
      return false;
    };

    let stored = stored.arc();

    item.layers().start() == layers.start()
      && (same_arc_geometry(arc, stored)
        || same_arc_geometry(arc, stored.reversed()))
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
      // `:1002`, the `ARC_T` case, which is `removeArcIndex` plus the
      // common tail (`:991`).
      ItemBody::Arc(_) => {
        self.remove_arc_index(node, id);
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

  /// Port of `NODE::removeArcIndex`,
  /// `pcbnew/router/pns_node.cpp:863`, which is `removeSegmentIndex`
  /// with the two anchors in place of the two segment ends.
  fn remove_arc_index(&mut self, node: NodeId, id: ItemId) {
    let Some(item) = self.items.get(id) else {
      return;
    };

    let ItemBody::Arc(arc) = item.body() else {
      return;
    };

    let (start, end) = (arc.anchor(0), arc.anchor(1));
    let layers = item.layers();
    let net = item.net();

    self.with_joints(node, |joints, root, _| {
      joints.unlink_joint(start, layers, net, id, root);
      joints.unlink_joint(end, layers, net, id, root);
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
  /// test (`:247`), the user filter (`:249`, which is
  /// [`CollisionSearchOptions::restricted_set`] here), the override test
  /// (`:252`, inside `World::visit_candidates`), then the item level
  /// collision through [`collide_into`], then the limit (`:259`).
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
    let mut obstacles = BTreeMap::new();

    self.query_colliding_into(node, head, resolver, options, &mut obstacles);

    self.sort_obstacles(obstacles)
  }

  /// One query's worth of obstacles, merged into a set the caller owns.
  ///
  /// `NODE::QueryColliding` writes into the `OBSTACLES&` it is handed
  /// (`pcbnew/router/pns_node.cpp:267`), which is what lets the line
  /// paths run one query per segment against **one** set: the limit at
  /// `:259` counts what every earlier segment already found, and a query
  /// that starts with a full set stops at once. Keeping that shape is why
  /// the set is a parameter here and the sort is the caller's.
  ///
  /// The key is the obstacle's item, which is KiCad's deduplication once
  /// the dangling head pointer is taken into account (note 02 section 11
  /// entry 4), and the first obstacle found for an item wins.
  fn query_colliding_into(
    &self,
    node: NodeId,
    head: ItemRef<'_>,
    resolver: &dyn RuleResolver,
    options: &CollisionSearchOptions,
    obstacles: &mut BTreeMap<Option<ItemId>, Obstacle>,
  ) {
    // :272
    if head.item().is_virtual() {
      return;
    }

    let items = &self.items;
    let limit = options.limit_count.filter(|count| *count > 0);
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

      // :249, the caller's filter. See
      // `CollisionSearchOptions::restricted_set` for why an empty set is
      // not spelled `None`.
      if options.restricted_set.is_some_and(|set| !set.contains(&id)) {
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
  }

  /// A merged obstacle set in this crate's deterministic order.
  ///
  /// The `(item uid, head uid)` order [`World::query_colliding`]
  /// documents, replacing the address order of KiCad's
  /// `std::set<OBSTACLE>` (`pcbnew/router/pns_node.h:103`). An unstored
  /// or stale handle sorts last.
  fn sort_obstacles(
    &self,
    obstacles: BTreeMap<Option<ItemId>, Obstacle>,
  ) -> Vec<Obstacle> {
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

  // -----------------------------------------------------------------
  // The line queries
  // -----------------------------------------------------------------

  /// Every obstacle a line meets in this node.
  ///
  /// The query loop of `NODE::NearestObstacle`,
  /// `pcbnew/router/pns_node.cpp:302` to `:316`: one
  /// [`World::query_colliding`] per segment of the chain, with the
  /// segment turned into an unstored `SEGMENT` by [`Line::segment_item`]
  /// (KiCad's stack temporary at `:310`), then one more for the via when
  /// the line ends with one (`:315`). Every query writes into the same
  /// set, so [`CollisionSearchOptions::limit_count`] counts the whole
  /// line and not each segment, and the result is deduplicated by item
  /// and ordered like [`World::query_colliding`]'s.
  ///
  /// KiCad has no `QueryColliding( const LINE& )` overload; this is the
  /// loop its two line callers open code, factored out so that
  /// [`World::check_colliding_line`] and [`World::nearest_obstacle`]
  /// cannot drift apart.
  ///
  /// A line's own stored segments are candidates like any other, because
  /// the identity test at `pcbnew/router/pns_item.cpp:119` compares the
  /// probe with the candidate and a probe is never a stored item (note 02
  /// section 11 entry 12). They are exempt through the same net rung of
  /// the ladder instead.
  pub fn query_colliding_line(
    &self,
    node: NodeId,
    line: &Line,
    resolver: &dyn RuleResolver,
    options: &CollisionSearchOptions,
  ) -> Vec<Obstacle> {
    let mut obstacles = BTreeMap::new();

    self.collect_line_obstacles(
      node,
      line,
      resolver,
      options,
      &mut obstacles,
      false,
    );

    self.sort_obstacles(obstacles)
  }

  /// The first obstacle a line meets, or `None`.
  ///
  /// Port of the `LINE_T` branch of `NODE::CheckColliding( const ITEM*,
  /// const COLLISION_SEARCH_OPTIONS& )`,
  /// `pcbnew/router/pns_node.cpp:507` to `:531`, which answers as soon as
  /// one segment's query has put anything in the set rather than
  /// finishing the line.
  ///
  /// KiCad then returns `*obs.begin()`, the address smallest member of
  /// what has accumulated so far; this returns the first in the
  /// deterministic order [`World::query_colliding`] establishes, over the
  /// same accumulated set.
  pub fn check_colliding_line(
    &self,
    node: NodeId,
    line: &Line,
    resolver: &dyn RuleResolver,
    options: &CollisionSearchOptions,
  ) -> Option<Obstacle> {
    let mut obstacles = BTreeMap::new();

    self.collect_line_obstacles(
      node,
      line,
      resolver,
      options,
      &mut obstacles,
      true,
    );

    self.sort_obstacles(obstacles).into_iter().next()
  }

  /// The first obstacle any of a set of items meets, or `None`.
  ///
  /// Port of `NODE::CheckColliding( const ITEM_SET&, int )`,
  /// `pcbnew/router/pns_node.cpp:478`, a plain loop that stops at the
  /// first item with an obstacle. The order of `items` decides which one
  /// that is, and it is the caller's `Vec`, where KiCad's `ITEM_SET`
  /// keeps insertion order too.
  ///
  /// KiCad's overload builds the options itself, `m_kindMask` from its
  /// argument and `m_limitCount = 1` (`:494`); both are the caller's here,
  /// so that the whole set can be queried under one policy.
  ///
  /// An `ITEM_SET` can also hold `LINE`s, which KiCad's overload
  /// decomposes through the same `CheckColliding`. A [`Line`] is not an
  /// item here, so a caller with lines in its set loops over
  /// [`World::check_colliding_line`] itself. The dragger is the one
  /// caller that mixes the two (`pcbnew/router/pns_dragger.cpp:446`) and
  /// `src/dragger.rs` does exactly that loop rather than widening this
  /// signature.
  pub fn check_colliding_items(
    &self,
    node: NodeId,
    items: &[ItemRef<'_>],
    resolver: &dyn RuleResolver,
    options: &CollisionSearchOptions,
  ) -> Option<Obstacle> {
    items
      .iter()
      .find_map(|item| self.check_colliding(node, *item, resolver, options))
  }

  /// Whether two lines collide, neither of them stored.
  ///
  /// Port of the `LINE::Collide( const LINE*, const NODE*, int )` call
  /// the shove makes at `pcbnew/router/pns_shove.cpp:481` to decide
  /// whether a freshly walked candidate still touches the line that is
  /// pushing it. In KiCad a `LINE` is an `ITEM`, so that is one
  /// `collideSimple` call with a line on both sides.
  ///
  /// A [`Line`] is not an [`Item`] here and `src/collide.rs` only accepts
  /// a line on the head side (note 03 section 6, and the milestone 2
  /// entry in `doc/log/2026-09-08.md`: "the shove, optimizer and multi
  /// dragger call sites that collide two lines will decompose one side
  /// into segments"). This is that decomposition: `obstacle` becomes one
  /// unstored [`Line::segment_item`] per segment, which is the same
  /// geometry, because a chain tested with half its width folded into the
  /// clearance and a run of segments each carrying its width in its shape
  /// cover the same area.
  ///
  /// `obstacle` maps to KiCad's `this` and `head` to its `aHead`, so the
  /// roles, and with them the head side via handling of
  /// `pcbnew/router/pns_item.cpp:140`, are the way round the shove asks
  /// for.
  ///
  /// No node is consulted: KiCad passes one only so that `collideSimple`
  /// can reach the rule resolver, which arrives here as a parameter. The
  /// layer is not a parameter either, for the reason `src/collide.rs`
  /// gives: the loop runs over the candidate's relevant shape layers
  /// rather than over the one layer KiCad's caller happens to name.
  ///
  /// `TODO(part 2)`: a via on the **obstacle** side
  /// (`pcbnew/router/pns_item.cpp:132`) is not decomposed, because
  /// `ShoveObstacleLine` strips the obstacle's via before it walks
  /// (`pcbnew/router/pns_shove.cpp:548`) and nothing else calls this yet.
  /// A via on the `head` side is handled, through [`Line::via_item`].
  pub fn collide_lines(
    &self,
    obstacle: &Line,
    head: &Line,
    resolver: &dyn RuleResolver,
    options: &CollisionSearchOptions,
  ) -> Option<Obstacle> {
    let probe = head.rule_item(self, PROBE_UID);
    let line_head =
      LineHead::new(ItemRef::unstored(&probe), head, head.via_item(self));

    (0..obstacle.shape().segment_count()).find_map(|index| {
      let segment = obstacle.segment_item(self, index, PROBE_UID);

      collide_line_items(
        &self.items,
        ItemRef::unstored(&segment),
        &line_head,
        resolver,
        options,
      )
    })
  }

  /// The query loop [`World::query_colliding_line`] and
  /// [`World::check_colliding_line`] share.
  ///
  /// `stop_when_found` is the difference between the two KiCad call
  /// sites: `NearestObstacle` runs every segment (`:302`), `CheckColliding`
  /// returns at the first non empty set (`:521`).
  fn collect_line_obstacles(
    &self,
    node: NodeId,
    line: &Line,
    resolver: &dyn RuleResolver,
    options: &CollisionSearchOptions,
    obstacles: &mut BTreeMap<Option<ItemId>, Obstacle>,
    stop_when_found: bool,
  ) {
    for index in 0..line.shape().segment_count() {
      let probe = line.segment_item(self, index, PROBE_UID);

      self.query_colliding_into(
        node,
        ItemRef::unstored(&probe),
        resolver,
        options,
        obstacles,
      );

      if stop_when_found && !obstacles.is_empty() {
        return;
      }
    }

    // :314. A via at the end is part of the line's footprint.
    if let Some(via) = line.via_item(self) {
      self.query_colliding_into(node, via, resolver, options, obstacles);
    }
  }

  /// The obstacle a line runs into first, with the geometry that says
  /// where.
  ///
  /// Port of `NODE::NearestObstacle`, `pcbnew/router/pns_node.cpp:298`.
  /// [`World::query_colliding_line`] answers *what* collides; this answers
  /// *which comes first along the line*, by building each obstacle's hull
  /// at the clearance the line needs from it, intersecting that hull with
  /// the line's chain, and measuring the path length from the line's start
  /// to each intersection.
  ///
  /// # Why it takes `&mut self`
  ///
  /// [`World::hull_of`] memoises. KiCad has the same constraint and spells
  /// it as an explicit sequential phase before it goes parallel, because
  /// neither its clearance cache nor its hull cache is thread safe
  /// (`:346`). Note 02 section 10.7 predicted that `&mut` would force the
  /// same split; it does.
  ///
  /// # Deterministic order
  ///
  /// Three of KiCad's pointer order dependencies live in this function
  /// (note 04 section 9 items 2, 3 and 4). All three are answered by
  /// scanning the candidates in `(item uid)` order, which
  /// [`World::query_colliding_line`] already returns them in:
  ///
  /// - the winner scan uses a strict `<` on the distance, so a tie goes
  ///   to the smaller item uid where KiCad's goes to the lower address;
  /// - the zero distance early break (`:466`) stops at the smallest uid
  ///   among the obstacles the line touches. It is an optimisation and
  ///   nothing else: no later candidate can beat a distance of zero under
  ///   a strict `<`;
  /// - the no intersection fallback (`:471`) picks the smallest item uid
  ///   rather than the lowest address. See
  ///   [`NearestObstacle::found_intersection`] for what that result means.
  ///
  /// # The corner mode
  ///
  /// `corner_mode` is the `makeHull` lambda's only input (`:330`): in the
  /// two 90 degree modes every hull is replaced by its axis aligned
  /// bounding box, so that a head made of horizontal and vertical
  /// segments is measured against a boundary of the same kind. KiCad
  /// reads it off the router singleton at `:301`; it is a parameter here
  /// for the reason `DESIGN.md` section 8 gives, and the walkaround
  /// passes the same value into [`simplified_hull`] for the hull it walks
  /// around.
  ///
  /// # The threads
  ///
  /// The per candidate scan can run on [`std::thread::scope`] threads,
  /// which is KiCad's thread pool at `:437` without a pool. It does not
  /// unless a host asks for it: [`World::set_parallelism`] is the knob
  /// and its default is one, for the reason recorded there. The private
  /// `scan_candidates` says how the blocks are cut and why the answer
  /// cannot depend on how many there are.
  ///
  /// Only the scan is threaded. The query, the clearances and the hulls
  /// stay on the calling thread, the first two because they read the
  /// arena through `&self` while the third needs `&mut self`, and all
  /// three because that is the split KiCad makes for the same reason
  /// (`:346`). Nothing in this function draws to a
  /// [`crate::debug::DebugDecorator`], so there is no trace call to keep
  /// out of the threads.
  ///
  /// # Not ported
  ///
  /// KiCad copies each hull into an owned `SHAPE_LINE_CHAIN` before it
  /// goes parallel (`:346` to `:376`). The copy is not needed here: the
  /// private `HullRefs` borrows the chains out of the [`Rc`]s the cache
  /// handed back, and the borrow checker is what makes that safe.
  pub fn nearest_obstacle(
    &mut self,
    node: NodeId,
    line: &Line,
    resolver: &dyn RuleResolver,
    options: &CollisionSearchOptions,
    corner_mode: CornerMode,
  ) -> Option<NearestObstacle> {
    // :302 to :318
    let obstacles = self.query_colliding_line(node, line, resolver, options);

    // :319
    if obstacles.is_empty() {
      return None;
    }

    let layer = line.layer();
    let use_epsilon = options.use_clearance_epsilon;
    let probe = line.rule_item(self, PROBE_UID);
    let mut clearances: Vec<(i32, Option<i32>)> =
      Vec::with_capacity(obstacles.len());

    {
      let head = ItemRef::unstored(&probe);
      let via = line.via_item(self);
      // :369. `VIA::Diameter( aLine->Layer() ) / 2`.
      let via_radius = via.and_then(|via| via_radius_on(via.item(), layer));

      for obstacle in &obstacles {
        let Some(item) = obstacle
          .item
          .and_then(|id| Some(ItemRef::stored(id, self.items.get(id)?)))
        else {
          clearances.push((0, None));
          continue;
        };

        // :360. Half the line's width folded into the clearance, with a
        // walkaround thickness of zero, which is the other half of the
        // same sum inside the hull builders.
        let line_clearance =
          clearance_of(resolver, item, head, use_epsilon) + line.width() / 2;
        // :366
        let via_clearance = match (via, via_radius) {
          (Some(via), Some(radius)) => {
            Some(clearance_of(resolver, item, via, use_epsilon) + radius)
          }
          _ => None,
        };

        clearances.push((line_clearance, via_clearance));
      }
    }

    // :361, :371. The hulls, which is the phase that needs `&mut self`.
    let mut hulls: Vec<ObstacleHulls> = Vec::with_capacity(obstacles.len());

    for (obstacle, (line_clearance, via_clearance)) in
      obstacles.iter().zip(&clearances)
    {
      let Some(id) = obstacle.item else {
        hulls.push(ObstacleHulls::default());
        continue;
      };

      // :330, `makeHull` applied to both of them.
      hulls.push(ObstacleHulls {
        line: self
          .hull_of(id, *line_clearance, 0, layer)
          .map(|hull| simplified_hull(hull, corner_mode)),
        via: via_clearance
          .and_then(|clearance| self.hull_of(id, clearance, 0, layer))
          .map(|hull| simplified_hull(hull, corner_mode)),
      });
    }

    // :376. The hulls, borrowed out of the reference counts the phase
    // above holds, so that the scan below can cross a thread boundary.
    let borrowed: Vec<HullRefs<'_>> = hulls
      .iter()
      .map(|candidate| HullRefs {
        line: candidate.line.as_deref(),
        via: candidate.via.as_deref(),
      })
      .collect();

    // :385 to :450, the per obstacle intersection scan, on this thread
    // or on several.
    let path = line.shape();
    let scan = scan_candidates(&borrowed, path, self.parallelism);

    // :456 to :468, the winner scan, always on this thread and always in
    // candidate order, which is item uid order.
    let mut best: Option<(i64, Vec2, usize)> = None;

    for (index, nearest) in scan.iter().enumerate() {
      // :460
      if let Some((distance, point)) = *nearest
        && best.is_none_or(|(closest, _, _)| distance < closest)
      {
        best = Some((distance, point, index));

        // :466
        if distance == 0 {
          break;
        }
      }
    }

    // :471. Nothing intersected the path at all. KiCad overwrites its
    // `INT_MAX` sentinel with the whole of `obstacles[0]`, whose
    // `m_distFirst` is the zero `collideSimple` wrote
    // (`pcbnew/router/pns_item.cpp:264`) and whose `m_ipFirst` is a
    // default `VECTOR2I`, so a caller cannot tell this case from a line
    // that starts exactly on a hull.
    let (distance, point, index, found_intersection) = match best {
      Some((distance, point, index)) => (distance, point, index, true),
      None => (0, Vec2::new(0, 0), 0, false),
    };

    Some(NearestObstacle {
      item: obstacles[index].item,
      clearance: obstacles[index].clearance,
      ip_first: point,
      dist_first: distance,
      pos: Vec2::new(0, 0),
      max_fanout_width: 0,
      hull: hulls[index].line.clone(),
      found_intersection,
    })
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

  /// The via at a position, named by what survives the via being
  /// replaced.
  ///
  /// Port of `NODE::FindViaByHandle`,
  /// `pcbnew/router/pns_node.cpp:1850`: the joint at the handle's
  /// position on the handle's first layer, then the first of its links
  /// that is a via on the right net with an overlapping layer range.
  ///
  /// The shove needs it because moving a via **replaces** the item, so a
  /// caller that held the old [`ItemId`] would be looking at something
  /// the node no longer has; a position plus a layer range plus a net can
  /// be resolved again in whatever branch the caller now stands on.
  ///
  /// Link order is insertion order, so the answer is deterministic where
  /// KiCad's is the order its index happened to build the joint in.
  pub fn find_via_by_handle(
    &self,
    node: NodeId,
    pos: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
  ) -> Option<ItemId> {
    let reference = self.find_joint(node, pos, layers.start(), net)?;
    let joint = self.joint(reference)?;

    joint.links().iter().copied().find(|link| {
      self.items.get(*link).is_some_and(|item| {
        item.of_kind(Kind::VIA)
          && item.net() == net
          && item.layers().overlaps(layers)
      })
    })
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

  // -----------------------------------------------------------------
  // Lines
  // -----------------------------------------------------------------

  /// Store a line's geometry as segments and link them back into it.
  ///
  /// Port of `NODE::Add( LINE&, bool )`,
  /// `pcbnew/router/pns_node.cpp:683`. Every segment of the chain either
  /// reuses an identical segment already at those coordinates, on that
  /// layer and on that net, or becomes a fresh stored segment; either way
  /// the handle is registered as a link of `line`, so that
  /// [`World::remove_line`] can undo the whole thing. A zero length
  /// segment is skipped (`:714`).
  ///
  /// The new segments are added with `allow_redundant` forced to true
  /// (`:729`), because the redundancy question was already answered by
  /// the lookup above; only the caller's `allow_redundant` decides
  /// whether that lookup happens at all.
  ///
  /// The via is **not** added. That asymmetry is KiCad's and it is load
  /// bearing for the placer: placing a via is a separate
  /// [`World::add_via`] plus [`Line::link_via`]
  /// (`pcbnew/router/pns_shove.cpp:2503`), while
  /// [`World::remove_line`] does take a linked via out (note 02 section
  /// 2.3).
  ///
  /// # Erratum E22, fixed: one loop in chain order
  ///
  /// KiCad runs **two** loops, every arc of `m_arcs` first (`:689`) and
  /// then every non arc segment in chain order (`:707`), so
  /// `LINE::m_links` of a line that holds both comes out in arcs first
  /// order. [`Line::clip_vertex_range`] walks shapes with
  /// [`LineChain::next_shape`] and counts link indices in lockstep, which
  /// is only correct for chain order, so a line built here and clipped
  /// afterwards would map the wrong links. This walks the chain once
  /// instead and emits each shape, straight or curved, where it sits.
  /// `doc/work/012-arcs.md` asks for the fix; the test naming it is
  /// `add_line_links_in_chain_order_erratum_e22`.
  ///
  /// The hazard is latent in KiCad because [`World::assemble_line`] does
  /// link in chain order (`:1188`) and that is the path the shove and the
  /// dragger clip; the fix makes the two producers agree.
  ///
  /// Returns the handles that were linked, in chain order.
  ///
  /// # The shared segment
  ///
  /// The comment at `:721`, "another line could be referencing this
  /// segment too :(", guards the reuse path: the segment found by the
  /// lookup may already be a link of some **other** live line. KiCad only
  /// protects against linking it twice into *this* line; nothing stops
  /// two lines from sharing it.
  ///
  /// What that means for [`Line::links_valid_in`]: it stays honest,
  /// because both lines name the same node and the handle really is
  /// valid there. What it does not do is tell either line that the other
  /// exists. Remove one of them and the shared segment leaves the node,
  /// so the other line keeps a handle whose generation check now fails.
  /// That is the stale link the module documentation of `src/line.rs`
  /// describes, and it is why every consumer skips a link it cannot
  /// resolve instead of trusting the count.
  ///
  /// # Panics
  ///
  /// In a debug build when the line is already linked, which is KiCad's
  /// `assert( !aLine.IsLinked() )` at `:685`.
  pub fn add_line(
    &mut self,
    node: NodeId,
    line: &mut Line,
    allow_redundant: bool,
  ) -> Vec<ItemId> {
    debug_assert!(
      !line.is_linked(),
      "add_line needs a line that is not linked yet"
    );

    let mut added = Vec::new();
    let layers = line.layers();
    let net = line.net();
    let width = line.width();
    // The arc the previous chain segment belonged to, so that the several
    // approximation segments of one arc emit one item.
    let mut open_arc: Option<usize> = None;

    for index in 0..line.shape().segment_count() {
      // :689, the arc loop, moved into chain order; see the port note.
      if line.shape().is_arc_segment(index) {
        let slot = line.shape().arc_index(index);

        if slot == open_arc {
          continue;
        }

        open_arc = slot;

        let Some(mut shape) = slot.and_then(|slot| line.shape().arc(slot))
        else {
          continue;
        };

        // `ARC( const LINE&, const SHAPE_ARC& )`,
        // `pcbnew/router/pns_arc.h:61`: the three points of the stored
        // arc with the **line's** width, the chain's copy carrying none
        // (`shape_line_chain.cpp:1624`).
        shape.set_width(width);

        // :694
        let reused = if allow_redundant {
          None
        } else {
          self.find_redundant_arc(node, shape, layers, net)
        };

        if let Some(existing) = reused {
          // :697. Unlike the segment loop below, KiCad's arc loop has no
          // `ContainsLink` guard; reproduced.
          line.link(existing);
          added.push(existing);

          continue;
        }

        let uid = self.next_uid();
        let item = self.arc_item_for_line(line, shape, uid);

        if let Some(stored) = self.add_arc(node, item, true) {
          line.link(stored);
          added.push(stored);
        }

        continue;
      }

      open_arc = None;

      let seg = line.shape().segment(index);

      // :714
      if seg.a == seg.b {
        continue;
      }

      let reused = if allow_redundant {
        None
      } else {
        self.find_redundant_segment(node, seg.a, seg.b, layers, net)
      };

      if let Some(existing) = reused {
        // :721, "another line could be referencing this segment too :("
        if !line.contains_link(existing) {
          line.link(existing);
          added.push(existing);
        }

        continue;
      }

      let uid = self.next_uid();
      let item = line.segment_item(self, index, uid);

      if let Some(stored) = self.add_segment(node, item, true) {
        line.link(stored);
        added.push(stored);
      }
    }

    if !added.is_empty() {
      line.set_links_valid_in(Some(node));
    }

    added
  }

  /// One arc of a line as a throwaway [`Item`] the node can store.
  ///
  /// Port of `ARC( const LINE& aParentLine, const SHAPE_ARC& aArc )`,
  /// `pcbnew/router/pns_arc.h:61`, which copies the net, the layers, the
  /// marker and the rank from the line. It lives here rather than beside
  /// [`Line::segment_item`] because [`crate::line`] is not part of work
  /// item 012 slice 5; the two build the same properties, and if a third
  /// body ever needs the same treatment they should be folded together.
  ///
  /// The `uid` rule is [`Line::segment_item`]'s: the caller takes one
  /// from [`World::next_uid`] so that the item carries a world unique
  /// number even before it is stored.
  fn arc_item_for_line(&self, line: &Line, arc: ShapeArc, uid: u64) -> Item {
    let mut item = Item::new(uid, ItemBody::Arc(Arc::new(arc)));

    item.set_layers_and_flash_all(line.layers());
    item.set_net(line.net());
    item.mark(line.marker(self));
    item.set_rank(line.rank(self));
    item.set_source(line.source());

    item
  }

  /// Take a line's stored segments, and its linked via, out of a node.
  ///
  /// Port of `NODE::Remove( LINE& )`,
  /// `pcbnew/router/pns_node.cpp:1054`, whose own comment says why it is
  /// not a typed remover: "LINE does not have a separate remover, as
  /// LINEs are never truly a member of the tree" (`:1056`). It removes
  /// every link that is a segment, an arc or a via, then detaches the
  /// line.
  ///
  /// A link of any other kind is ignored, because KiCad's dispatch is a
  /// chain of three `OfKind` tests with no `else`. A link the arena no
  /// longer knows is ignored too, which C++ cannot do.
  ///
  /// # Deviation: the line is mutated
  ///
  /// KiCad ends with `aLine.SetOwner( nullptr ); aLine.ClearLinks()`
  /// (`:1069`), which is the only signal a caller gets that the line's
  /// handles are gone. Keeping it means the line has to be taken by
  /// `&mut`, which is what `LINE&` is in C++ anyway.
  ///
  /// # Why there is no `is_linked_checked` assertion here
  ///
  /// Note 02 section 10.3 suggests asserting `IsLinkedChecked` on every
  /// function that consumes links. It does not hold at this one: a via
  /// attached with [`Line::link_via`] is a link with no shape behind it,
  /// so a head that carries one has `link_count == shape_count + 1`
  /// (`pcbnew/router/pns_shove.cpp:2508` builds exactly that and then
  /// removes the line). [`Line::is_linked_checked`] stays available for
  /// the callers where it does hold.
  pub fn remove_line(&mut self, node: NodeId, line: &mut Line) {
    let links: Vec<ItemId> = line.links().to_vec();

    for link in links {
      let Some(item) = self.items.get(link) else {
        continue;
      };

      if item.of_kind(Kind::SEGMENT | Kind::ARC | Kind::VIA) {
        self.remove(node, link);
      }
    }

    line.clear_links();
  }

  /// Swap one line's geometry for another's.
  ///
  /// Port of `NODE::Replace( LINE&, LINE&, bool )`,
  /// `pcbnew/router/pns_node.cpp:958`, which is literally
  /// [`World::remove_line`] followed by [`World::add_line`]. Both lines
  /// are mutated: the old one loses its links, the new one gains them.
  ///
  /// Returns what [`World::add_line`] returned.
  pub fn replace_line(
    &mut self,
    node: NodeId,
    old: &mut Line,
    new_line: &mut Line,
    allow_redundant: bool,
  ) -> Vec<ItemId> {
    self.remove_line(node, old);

    self.add_line(node, new_line, allow_redundant)
  }

  /// Walk the joint graph both ways from one segment and build a line.
  ///
  /// Port of `NODE::AssembleLine`,
  /// `pcbnew/router/pns_node.cpp:1132`. The line runs from one non
  /// trivial joint to the next: [`Joint::next_segment`] decides what
  /// counts as a continuation, so a pad, a via, a fan out or a width
  /// change ends it.
  ///
  /// - `origin_segment_index` receives the **point** index at which
  ///   `segment` was appended (`:1195`), clamped to the last segment
  ///   index afterwards (`:1207`). KiCad's own TODO there admits the
  ///   index is not maintained under simplification, and the clamp is the
  ///   patch. The value is left untouched when the seed was never
  ///   appended, which is KiCad's behaviour for an `int*` it does not
  ///   write.
  /// - `stop_at_locked_joints` ends the line at a joint the shove has
  ///   pinned (`:1116`).
  /// - `follow_locked_segments` is handed to [`Joint::next_segment`],
  ///   where it means "a locked segment still continues the line, and a
  ///   virtual via here does not end it".
  /// - `allow_segment_size_mismatch` lets the line cross a width change
  ///   (`:1123`). KiCad defaults it to true
  ///   (`pcbnew/router/pns_node.h:441`); the topology and the placer pass
  ///   false (`pcbnew/router/pns_topology.cpp:55`,
  ///   `pcbnew/router/pns_line_placer.cpp:1976`), so it is a parameter
  ///   here rather than a constant.
  ///
  /// A via at the end of the run is **not** picked up: `followLine` stops
  /// at it and nothing attaches it afterwards, so an assembled line never
  /// has [`Line::ends_with_via`] set. The placer and the shove attach one
  /// themselves when they need it.
  ///
  /// # Deviations
  ///
  /// KiCad allocates three `std::array`s of `1024 * 16 + 1` entries on
  /// the stack, 128 KiB per call, and grows into them from the middle
  /// (`:1135` to `:1145`). This uses one [`VecDeque`] of triples, with
  /// the backward walk pushing to the front and the forward walk to the
  /// back, which produces the same order with no fixed limit; the
  /// `aLimit` checks at `:1118` therefore have no counterpart. The third
  /// member of each triple is KiCad's parallel `arcReversed` array.
  ///
  /// # Arcs, and why erratum E4 is moot here
  ///
  /// An arc link contributes **no** plain corner point of its own
  /// (`:1175`): its whole polyline comes from `Append( SHAPE_ARC )` at
  /// `:1185`, which is why an arc in the middle of a line pushes every
  /// later point index along by the arc's approximation point count, and
  /// why `origin_segment_index` needs the clamp below.
  ///
  /// KiCad appends `sa->Reversed()` for an arc the walk reached from the
  /// far end. `Reversed()` rebuilds the arc from the permuted points and
  /// re runs `CalcArcCenter`, which through erratum E1's round number
  /// snapping can land on a different centre than `Reverse()` would have
  /// left in place, so assembling one physical line in the two scan
  /// directions can produce two arcs with different derived geometry;
  /// that is erratum E4. It cannot happen here: [`ShapeArc`] caches
  /// nothing, every derived quantity is computed on demand, and
  /// [`ShapeArc::reverse`] and [`ShapeArc::reversed`] give the same three
  /// points. The in place reverse is used because it says what is meant.
  ///
  /// The `aSegments[aPos] = nullptr` a guard hit writes (`:1110`) is not
  /// reproduced: `aPos` has just been stepped past the range the
  /// assembly loop reads, in both directions, so the write can never be
  /// seen.
  ///
  /// The final `wxASSERT_MSG( pl.SegmentCount() != 0 )` (`:1210`) is a
  /// `debug_assert!`. An empty line comes back for a stale handle, where
  /// KiCad would have dereferenced it.
  pub fn assemble_line(
    &self,
    node: NodeId,
    segment: ItemId,
    origin_segment_index: Option<&mut usize>,
    stop_at_locked_joints: bool,
    follow_locked_segments: bool,
    allow_segment_size_mismatch: bool,
  ) -> Line {
    let mut line = Line::new();

    let Some(seed) = self.items.get(segment) else {
      return line;
    };

    // The seed is a `LINKED_ITEM*` in KiCad, so an arc seeds a line as
    // readily as a segment; a body with no width is not one.
    let Some(width) = width_of(seed) else {
      return line;
    };

    // :1147 to :1152
    line.set_width(width);
    line.set_layers(seed.layers());
    line.set_net(seed.net());
    line.set_source(seed.source());

    let mut corners: VecDeque<(Vec2, ItemId, bool)> = VecDeque::new();
    let options = FollowOptions {
      stop_at_locked_joints,
      follow_locked_segments,
      allow_segment_size_mismatch,
    };

    // :1154, backwards from the seed, pushing to the front so that the
    // deque ends up in chain order.
    let guard_hit =
      self.follow_line(node, segment, false, &mut corners, options);

    // :1157
    if !guard_hit {
      self.follow_line(node, segment, true, &mut corners, options);
    }

    let mut previous: Option<ItemId> = None;
    let mut origin_point: Option<usize> = None;

    for (corner, link, arc_reversed) in &corners {
      let arc = self.items.get(*link).and_then(|item| match item.body() {
        ItemBody::Arc(arc) => Some(arc.arc()),
        _ => None,
      });

      // :1175. Only a link that is not an arc contributes its corner.
      if arc.is_none() {
        line.chain_mut().append(*corner);
      }

      if previous != Some(*link) {
        // :1180 to :1185
        if let Some(mut shape) = arc {
          if *arc_reversed {
            shape.reverse();
          }

          line
            .chain_mut()
            .append_arc(&shape, LineChain::ARC_POLYGONIZATION_MAX_ERROR);
        }

        line.link(*link);

        // :1191, "latter condition to avoid loops".
        if *link == segment && origin_point.is_none() {
          origin_point = Some(line.point_count().saturating_sub(1));
        }
      }

      previous = Some(*link);
    }

    if line.is_linked() {
      line.set_links_valid_in(Some(node));
    }

    // :1203, "do NOT remove colinear segments here!"
    line.chain_mut().remove_duplicate_points();

    if let Some(slot) = origin_segment_index {
      if let Some(point) = origin_point {
        *slot = point;
      }

      // :1207
      if *slot >= line.segment_count() {
        *slot = line.segment_count().saturating_sub(1);
      }
    }

    debug_assert!(
      line.segment_count() != 0,
      "assembled line should never be empty"
    );

    line
  }

  /// Walk the joint graph in one direction, collecting corners.
  ///
  /// Port of `NODE::followLine`, `pcbnew/router/pns_node.cpp:1074`.
  /// `scan_forward` is KiCad's `aScanDirection`: the anchor index the
  /// walk advances towards, and the direction the corners are written
  /// in. Returns `aGuardHit`, which says the walk came back to where it
  /// started and the line is a closed loop, so the caller must not walk
  /// the other way as well.
  ///
  /// The `prevReversed` flip (`:1127`) is what keeps the walk going when
  /// a segment is stored back to front: the next anchor to look at is
  /// `aScanDirection ^ prevReversed`, not `aScanDirection`.
  ///
  /// The third member of each corner is KiCad's `aArcReversed` (`:1092`
  /// to `:1103`): true when the walk reached an arc from the anchor the
  /// scan direction does not expect, so that [`World::assemble_line`]
  /// appends the arc the way the line travels. It is false for every
  /// other kind, as KiCad's is.
  fn follow_line(
    &self,
    node: NodeId,
    start: ItemId,
    scan_forward: bool,
    corners: &mut VecDeque<(Vec2, ItemId, bool)>,
    options: FollowOptions,
  ) -> bool {
    let anchor_index = usize::from(scan_forward);

    let Some(seed) = self.items.get(start) else {
      return false;
    };

    // :1081 and :1082
    let guard = seed.anchor(anchor_index);
    let start_width = width_of(seed);

    let mut current = start;
    let mut previous_reversed = false;
    let mut count: u32 = 0;

    while let Some(item) = self.items.get(current) {
      // :1086
      let position =
        item.anchor(usize::from(scan_forward != previous_reversed));
      let Some(joint) =
        self.find_joint(node, position, item.layers().start(), item.net())
      else {
        break;
      };

      let Some(joint) = self.joint(joint) else {
        break;
      };

      // :1096 to :1103. An arc reached from anchor 0 while scanning
      // forward, or from anchor 1 while scanning backward, runs against
      // the direction of travel.
      let arc_reversed = matches!(item.body(), ItemBody::Arc(_))
        && joint.pos() == item.anchor(usize::from(!scan_forward));

      // :1092
      if scan_forward {
        corners.push_back((joint.pos(), current, arc_reversed));
      } else {
        corners.push_front((joint.pos(), current, arc_reversed));
      }

      // :1107, the loop detector.
      if count > 0 && guard == position {
        return true;
      }

      // :1116
      if options.stop_at_locked_joints && joint.is_locked() {
        break;
      }

      // :1121
      let Some(next) = joint.next_segment(
        &self.items,
        current,
        options.follow_locked_segments,
      ) else {
        break;
      };

      // :1123
      if !options.allow_segment_size_mismatch
        && self.items.get(next).and_then(width_of) != start_width
      {
        break;
      }

      // :1127
      previous_reversed = self
        .items
        .get(next)
        .is_some_and(|item| joint.pos() == item.anchor(anchor_index));
      current = next;
      count = count.saturating_add(1);
    }

    false
  }

  /// Every line that runs from one joint to another, clipped to it.
  ///
  /// Port of `NODE::FindLinesBetweenJoints`,
  /// `pcbnew/router/pns_node.cpp:1223`. It assembles a line from each
  /// track linked to `first`, drops the ones whose layers do not overlap
  /// `second`, and clips what is left to the vertex range between the two
  /// joint positions. The placer uses it to find and remove loops
  /// (`pcbnew/router/pns_line_placer.cpp:1852`).
  ///
  /// KiCad's `FindLineEnds( line, j_start, j_end )` call at `:1237` is
  /// not reproduced: neither output is ever read, and the routine
  /// dereferences `FindJoint` without a null check.
  ///
  /// The `-1` that `Find` answers with survives as a signed comparison,
  /// because KiCad swaps the two indices **before** it tests them
  /// (`:1242` against `:1245`), so a line that contains only one of the
  /// two positions is discarded whichever end it was.
  ///
  /// The `int` return value is dropped: it is always zero (`:1253`).
  pub fn find_lines_between_joints(
    &self,
    node: NodeId,
    first: JointRef,
    second: JointRef,
  ) -> Vec<Line> {
    let (Some(first_joint), Some(second_joint)) =
      (self.joint(first), self.joint(second))
    else {
      return Vec::new();
    };

    let mut lines = Vec::new();

    for link in first_joint.links() {
      if !self.is_of_kind(*link, Kind::SEGMENT | Kind::ARC) {
        continue;
      }

      let mut line = self.assemble_line(node, *link, None, false, false, true);

      if !line.layers().overlaps(second_joint.layers()) {
        continue;
      }

      let mut start = index_or_missing(line.shape().find(first_joint.pos(), 0));
      let mut end = index_or_missing(line.shape().find(second_joint.pos(), 0));

      if end < start {
        std::mem::swap(&mut start, &mut end);
      }

      if start >= 0 && end >= 0 {
        line.clip_vertex_range(start as usize, end as usize);
        lines.push(line);
      }
    }

    lines
  }

  /// Whether a stored item is of one of the given kinds.
  ///
  /// The `item->Kind() == ITEM::SEGMENT_T || item->Kind() == ITEM::ARC_T`
  /// test of `FindLinesBetweenJoints`, `pcbnew/router/pns_node.cpp:1227`,
  /// with a stale handle answering no.
  fn is_of_kind(&self, id: ItemId, mask: Kind) -> bool {
    self.items.get(id).is_some_and(|item| item.of_kind(mask))
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

  /// The clearance a stored item needs from a line.
  ///
  /// `NODE::GetClearance( const ITEM*, const ITEM*, bool )`
  /// (`pcbnew/router/pns_node.cpp:143`) for the one call shape that
  /// cannot go through [`World::clearance_between`], because a [`Line`]
  /// has no [`ItemId`] to key the cache on:
  /// `WALKAROUND::processCluster` asks for it once per cluster member
  /// (`pcbnew/router/pns_walkaround.cpp:156`) with the epsilon turned
  /// off, because the hull it sizes has to be the strict rule.
  ///
  /// KiCad caches this one too, on the address of a stack temporary; the
  /// answer here is recomputed instead, which is the same number.
  ///
  /// `-1` for a pair the resolver exempts and for a stale handle, which
  /// is KiCad's "these two can never collide" arriving in an `int`. The
  /// hull builders take it as it is, as the private `clearance_of` this
  /// forwards to documents.
  pub fn clearance_for_line(
    &self,
    item: ItemId,
    line: &Line,
    use_epsilon: bool,
    resolver: &dyn RuleResolver,
  ) -> i32 {
    let Some(stored) = self.items.get(item) else {
      return -1;
    };

    let probe = line.rule_item(self, PROBE_UID);

    clearance_of(
      resolver,
      ItemRef::stored(item, stored),
      ItemRef::unstored(&probe),
      use_epsilon,
    )
  }

  /// Everything that touches an item, transitively.
  ///
  /// Port of `TOPOLOGY::AssembleCluster`,
  /// `pcbnew/router/pns_topology.cpp:1187`. It is a breadth first walk
  /// from `start` over [`World::query_colliding`] with the clearance
  /// forced to zero and the same net exemption turned off, so that what
  /// it finds is what physically touches, and it is what lets the
  /// walkaround clear a whole pad row or via group in one pass rather
  /// than one obstacle per iteration.
  ///
  /// It lives on [`World`] rather than in a topology module because
  /// KiCad's `TOPOLOGY` is a one field wrapper over a node
  /// (`pcbnew/router/pns_topology.h:54`) and this is the only member of
  /// it the walkaround needs. `DESIGN.md` section 9 gives cluster
  /// assembly its own module; it should move there when the rest of
  /// `TOPOLOGY` arrives.
  ///
  /// The four parameters are KiCad's. `layer` restricts the walk to items
  /// that reach the line's layer, `area_expansion_limit` is the ratio at
  /// which a growing cluster is abandoned ([`None`] for KiCad's `0.0`,
  /// which disables the test; the walkaround passes that and the shove
  /// passes `10.0`), and `excluded_net` drops the routed net's own
  /// copper, which is how a head does not gather the track it is
  /// extending.
  ///
  /// # Order
  ///
  /// The result is in discovery order, which is deterministic because
  /// [`World::query_colliding`] is: KiCad's is the address order of its
  /// obstacle set (note 02 section 11 entry 3). The membership set is a
  /// [`BTreeSet`] where KiCad's is an `unordered_set`, which changes
  /// nothing because neither is iterated.
  pub fn assemble_cluster(
    &self,
    node: NodeId,
    start: ItemId,
    layer: i32,
    area_expansion_limit: Option<f64>,
    excluded_net: Option<NetId>,
    resolver: &dyn RuleResolver,
  ) -> Vec<ItemId> {
    let mut cluster: Vec<ItemId> = Vec::new();
    let Some(seed) = self.items.get(start) else {
      return cluster;
    };

    // :1192 to :1195. Touching, not "closer than a rule allows".
    let options = CollisionSearchOptions {
      different_nets_only: false,
      override_clearance: Some(0),
      ..CollisionSearchOptions::default()
    };

    // :1199 to :1201
    let Some(mut cluster_bbox) =
      seed.shape(layer).and_then(|shape| shape.bbox(0))
    else {
      return cluster;
    };
    let initial_area = cluster_bbox.area();
    let mut pending: VecDeque<ItemId> = VecDeque::new();
    let mut processed: BTreeSet<ItemId> = BTreeSet::new();

    pending.push_back(start);

    // :1203
    while let Some(top) = pending.pop_front() {
      // :1210. The seed is the only item that can reach here unprocessed,
      // because every other one is inserted as it is queued.
      if !processed.contains(&top) {
        cluster.push(top);
      }

      processed.insert(top);

      let Some(item) = self.items.get(top) else {
        continue;
      };

      let obstacles = self.query_colliding(
        node,
        ItemRef::stored(top, item),
        resolver,
        &options,
      );

      for obstacle in obstacles {
        let Some(id) = obstacle.item else {
          continue;
        };
        let Some(found) = self.items.get(id) else {
          continue;
        };
        let Some(item) = self.items.get(top) else {
          continue;
        };

        // :1221. Two tracks of different nets crossing are not one
        // cluster, however close they run.
        if found.net() != item.net()
          && found.of_kind(Kind::SEGMENT)
          && item.of_kind(Kind::SEGMENT)
        {
          continue;
        }

        // :1226
        if excluded_net.is_some() && found.net() == excluded_net {
          continue;
        }

        let overlaps = found.layers().overlaps(LayerRange::single(layer));

        // :1229. A track contributes the box of the whole line it belongs
        // to, not of the one segment that was hit.
        let grown = if found.of_kind(Kind::SEGMENT | Kind::ARC) && overlaps {
          self
            .assemble_line(node, id, None, false, false, true)
            .shape()
            .bbox(0)
        } else {
          found.shape(layer).and_then(|shape| shape.bbox(0))
        };

        if let Some(grown) = grown {
          cluster_bbox = cluster_bbox.merge(grown);
        }

        // :1239 to :1243. The `+ 1` is KiCad's guard against a zero area
        // seed, and the limit is off when it is not positive.
        let area_ratio =
          cluster_bbox.area() as f64 / (initial_area as f64 + 1.0);

        if area_expansion_limit
          .is_some_and(|limit| limit > 0.0 && area_ratio > limit)
        {
          break;
        }

        // :1245
        if !processed.contains(&id)
          && overlaps
          && !found.marker().intersects(MarkerFlags::HEAD)
        {
          processed.insert(id);
          cluster.push(id);
          pending.push_back(id);
        }
      }
    }

    cluster
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

// ---------------------------------------------------------------------
// The nearest obstacle
// ---------------------------------------------------------------------

/// The uid a line's throwaway probe items carry.
///
/// [`Line::segment_item`] and [`Line::rule_item`] take a uid because a
/// stored item's has to come from the world's counter (`DESIGN.md`
/// section 8). A probe is never stored, so nothing can sort by its uid;
/// `World::uid_of` answers `u64::MAX` for the missing handle anyway, so
/// this is the same number by a different route.
const PROBE_UID: u64 = u64::MAX;

/// A hull as the corner mode wants it seen.
///
/// Port of the `makeHull` lambda of `NODE::NearestObstacle`
/// (`pcbnew/router/pns_node.cpp:330` to `:344`) and of the identical
/// block in `WALKAROUND::processCluster`
/// (`pcbnew/router/pns_walkaround.cpp:162` to `:173`). In the two 90
/// degree corner modes the hull is replaced by its axis aligned bounding
/// box, in the corner order left top, right top, right bottom, left
/// bottom, which is clockwise on screen and therefore keeps the winding
/// invariant `crate::geometry::hull` states. In every other mode the hull
/// is returned untouched, which is why this takes and returns an [`Rc`]:
/// the common path costs a reference count and no copy.
///
/// # Deviation: the box is closed
///
/// KiCad builds the box into a default constructed
/// `SHAPE_LINE_CHAIN` and appends four points without ever calling
/// `SetClosed( true )`, so the chain it hands on is **open**. That is not
/// a cosmetic difference. `SHAPE_LINE_CHAIN::PointInside` returns false
/// for any open chain (`libs/kimath/src/geometry/shape_line_chain.cpp:1994`),
/// so an open box hull has no inside: `LINE::Walkaround` classifies every
/// path vertex as outside, never leaves the path, and the walkaround is a
/// no op in the two 90 degree corner modes, running to its iteration
/// limit and reporting almost done. Every real hull builder closes its
/// chain (`pcbnew/router/pns_utils.cpp:45`, `:91`, `:270`, `:341`) and
/// `LINE::Walkaround` documents that a hull is a closed clockwise polygon
/// by construction (`pcbnew/router/pns_line.cpp:397`), so the open chain
/// is a KiCad defect and not a behaviour to pin. This port closes it, and
/// the 90 degree walkaround therefore does bend around obstacles where
/// KiCad's does not.
///
/// A hull of fewer than two points has no bounding box; it is returned as
/// it is, which is what KiCad's `BBox()` of an empty chain would produce
/// anyway.
pub fn simplified_hull(
  hull: Rc<LineChain>,
  corner_mode: CornerMode,
) -> Rc<LineChain> {
  // :326. KiCad tests `MITERED_90 || ROUNDED_90`; this crate has no
  // rounded modes yet, see `CornerMode`.
  if corner_mode != CornerMode::Mitered90 {
    return hull;
  }

  let Some(bbox) = hull.bbox(0) else {
    return hull;
  };

  let left = bbox.left() as i32;
  let right = bbox.right() as i32;
  let top = bbox.top() as i32;
  let bottom = bbox.bottom() as i32;

  // :336 to :339
  let mut box_hull = LineChain::from_slice(
    &[
      Vec2::new(left, top),
      Vec2::new(right, top),
      Vec2::new(right, bottom),
      Vec2::new(left, bottom),
    ],
    true,
  );

  box_hull.set_width(hull.width());

  Rc::new(box_hull)
}

/// What [`World::nearest_obstacle`] found, and where.
///
/// Port of the `OBSTACLE` that `NODE::NearestObstacle` returns
/// (`pcbnew/router/pns_node.h:88`), restricted to the fields something
/// reads and extended with the hull, so that the walkaround does not have
/// to rebuild it.
///
/// Two of KiCad's fields are not here. `m_head` points at the stack
/// `SEGMENT` the query loop built and dangles the moment the loop ends;
/// nothing in the tree reads it (note 02 section 11 entry 4). `m_pos` is
/// [`NearestObstacle::pos`], kept because the struct is a port, and it is
/// never written anywhere in KiCad's tree either.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct NearestObstacle {
  /// The item that collides. Port of `m_item`. `None` only for an
  /// obstacle the arena has forgotten between the query and the scan.
  pub item: Option<ItemId>,
  /// The clearance the ladder applied when the collision was found. Port
  /// of `m_clearance`, written by `collideSimple`
  /// (`pcbnew/router/pns_item.cpp:263`), which is **not** the clearance
  /// the hull was built at: that one folds in half the line's width and
  /// is asked for again with the line rather than one of its segments.
  pub clearance: i32,
  /// Where the line first enters the obstacle's hull. Port of
  /// `m_ipFirst` (`pcbnew/router/pns_node.cpp:464`).
  pub ip_first: Vec2,
  /// The path length from the line's first point to
  /// [`NearestObstacle::ip_first`]. Port of `m_distFirst` (`:463`),
  /// widened to `i64` as [`LineChain::path_length`] is.
  pub dist_first: i64,
  /// Port of `m_pos`, which no code in KiCad's tree writes. Always the
  /// origin here, so that a consumer that reads it gets the same nothing.
  pub pos: Vec2,
  /// The widest track fanning out of an obstacle via. Port of
  /// `m_maxFanoutWidth`, which `collideSimple` zeroes
  /// (`pcbnew/router/pns_item.cpp:264`) and only the shove fills in
  /// (`pcbnew/router/pns_shove.cpp:1553`), so it is zero until the shove
  /// lands.
  pub max_fanout_width: i32,
  /// The obstacle's hull at the clearance this line needs from it, with a
  /// walkaround thickness of zero, exactly the chain the scan intersected
  /// (`pcbnew/router/pns_node.cpp:361`).
  ///
  /// KiCad keeps no hull and its callers rebuild one; note that
  /// `WALKAROUND::processCluster` rebuilds it under a **different** key,
  /// the line's width as the thickness rather than half of it as
  /// clearance (`pcbnew/router/pns_walkaround.cpp:157`), so this is the
  /// hull for the distance measurement and not a drop in replacement for
  /// the walkaround's.
  ///
  /// It is the line hull even when the via hull is what produced the
  /// winning intersection, because that is the only one a consumer has a
  /// use for. `None` only for an obstacle whose handle went stale.
  pub hull: Option<Rc<LineChain>>,
  /// Whether the winner was chosen by geometry or by the fallback.
  ///
  /// Not a KiCad field. `false` means no obstacle's hull met the line at
  /// all and `:471` picked the first candidate in the deterministic
  /// order, in which case [`NearestObstacle::dist_first`] and
  /// [`NearestObstacle::ip_first`] carry the zeroes `collideSimple` left
  /// and mean nothing. Note 02 section 3.9 calls that fallback a
  /// correctness wart: the item it names may not block the path at all.
  /// A consumer that wants KiCad's behaviour to the letter ignores this
  /// field, since KiCad cannot tell the two cases apart.
  pub found_intersection: bool,
}

/// The two hulls one obstacle contributes to the distance scan.
///
/// `hullData[i]`, `pcbnew/router/pns_node.cpp:350`. KiCad owns a copy of
/// each chain there, precisely so that the parallel phase cannot race the
/// cache it came from (`:346`); [`Rc`] shares them instead.
#[derive(Clone, Default)]
struct ObstacleHulls {
  /// The hull at the line's clearance (`:361`).
  line: Option<Rc<LineChain>>,
  /// The hull at the via's clearance, when the line ends with one
  /// (`:371`).
  via: Option<Rc<LineChain>>,
}

/// One obstacle's two hulls, borrowed out of the cache.
///
/// The parallel scan below reads hulls and nothing else, and
/// [`ObstacleHulls`] cannot cross a thread boundary because [`Rc`] is
/// neither [`Send`] nor [`Sync`]. Borrowing the chains out of the
/// reference counts leaves the counts on the calling thread, where the
/// sequential hull phase put them, and hands the threads a plain shared
/// reference to immutable geometry. It is the cheap half of the copy
/// KiCad makes for the same reason (`pcbnew/router/pns_node.cpp:346`).
#[derive(Copy, Clone, Default)]
struct HullRefs<'hulls> {
  /// The hull at the line's clearance (`:361`).
  line: Option<&'hulls LineChain>,
  /// The hull at the via's clearance (`:371`).
  via: Option<&'hulls LineChain>,
}

/// Where one obstacle's hulls first cross a line, and how far along it.
///
/// The `ObstacleResult` of `pcbnew/router/pns_node.cpp:377`, whose
/// `INT_MAX` sentinel for "this obstacle does not cross the path at all"
/// is [`None`] here. The distance is a path length in nanometres, so it
/// is an `i64` where KiCad's is an `int`.
type Crossing = Option<(i64, Vec2)>;

/// How many obstacle candidates make one parallel block worth dispatching.
///
/// Port of `MIN_OBSTACLES_PER_BLOCK`, `pcbnew/router/pns_node.cpp:425`,
/// whose value is 8. It is 8 there because the work goes to a pool that
/// is already running and only has to be woken, which KiCad's own comment
/// puts at 5 to 20 microseconds. This crate takes no dependency, so there
/// is no pool: a block costs a whole thread, and a scope of `n` blocks
/// measures at about 21 microseconds per block in the container
/// `doc/performance.md` describes, against 2.2 microseconds to scan one
/// candidate. 32 candidates is about 70 microseconds of geometry per
/// block against 21 of dispatch, and below that ratio the threads cost
/// more than they save. Even above it they mostly do, which is why
/// [`World::set_parallelism`] defaults to one; the parallel obstacle
/// query section of `doc/performance.md` has the measurement.
const MIN_CANDIDATES_PER_BLOCK: usize = 32;

/// One candidate's [`Crossing`].
///
/// Port of the `processObstacle` lambda,
/// `pcbnew/router/pns_node.cpp:385` to `:425`: intersect both hulls with
/// the path and keep the crossing with the smallest path length.
///
/// It is a free function taking only shared references because it is the
/// body both the sequential and the threaded scan run, so neither can
/// drift from the other.
fn nearest_crossing(hulls: HullRefs<'_>, path: &LineChain) -> Crossing {
  let mut nearest: Crossing = None;

  for hull in [hulls.line, hulls.via].into_iter().flatten() {
    for crossing in hull_intersection(hull, path) {
      let Some(theirs) = crossing.theirs else {
        continue;
      };
      // :400. `index_their` is the segment hint.
      let Some(distance) =
        path.path_length(crossing.point, Some(theirs.index()))
      else {
        continue;
      };

      if nearest.is_none_or(|(closest, _)| distance < closest) {
        nearest = Some((distance, crossing.point));
      }
    }
  }

  nearest
}

/// [`nearest_crossing`] for every candidate, in candidate order.
///
/// Port of `pcbnew/router/pns_node.cpp:425` to `:450`: `numBlocks` is
/// `numObstacles / MIN_OBSTACLES_PER_BLOCK` there too, and a query below
/// the threshold runs the loop on the calling thread.
///
/// # Why the answer cannot depend on `parallelism`
///
/// `results[i]` is a pure function of `hulls[i]` and `path`, both shared
/// and immutable, so a block boundary can fall anywhere. Nothing is
/// reduced here: the winner scan runs afterwards, on the calling thread,
/// over this vector in candidate order, which is item uid order. That is
/// the whole of the determinism argument, and `tests/parallelism.rs`
/// checks it end to end.
///
/// # Deviation: no early exit in the scan
///
/// [`World::nearest_obstacle`] used to fuse this loop with the winner
/// scan, so a candidate at distance zero ended both. Splitting them to
/// get the blocks costs that, exactly as it costs KiCad. It was measured
/// on the boards of `doc/performance.md` before the split: the break
/// fired late enough that 97.5% of the candidates were scanned anyway.
fn scan_candidates(
  hulls: &[HullRefs<'_>],
  path: &LineChain,
  parallelism: usize,
) -> Vec<Crossing> {
  // :443. `std::max( 1, numObstacles / MIN_OBSTACLES_PER_BLOCK )`, with
  // the pool's size as the second cap, which KiCad gets from the pool
  // itself.
  let blocks = (hulls.len() / MIN_CANDIDATES_PER_BLOCK).min(parallelism);

  if blocks < 2 {
    // :447. The sequential fallback.
    return hulls
      .iter()
      .map(|candidate| nearest_crossing(*candidate, path))
      .collect();
  }

  let block_size = hulls.len().div_ceil(blocks);
  let mut results: Vec<Crossing> = vec![None; hulls.len()];
  let mut work: Vec<_> = hulls
    .chunks(block_size)
    .zip(results.chunks_mut(block_size))
    .collect();
  // The calling thread takes one block instead of blocking on the others,
  // so a query of `blocks` blocks spawns `blocks - 1` threads.
  let ours = work.pop();

  std::thread::scope(|scope| {
    for (candidates, into) in work {
      scope.spawn(move || fill_block(candidates, into, path));
    }

    if let Some((candidates, into)) = ours {
      fill_block(candidates, into, path);
    }
  });

  results
}

/// [`nearest_crossing`] over one block of candidates.
fn fill_block(
  candidates: &[HullRefs<'_>],
  into: &mut [Crossing],
  path: &LineChain,
) {
  for (candidate, result) in candidates.iter().zip(into) {
    *result = nearest_crossing(*candidate, path);
  }
}

/// The clearance between two items, uncached.
///
/// Port of `NODE::GetClearance`, `pcbnew/router/pns_node.cpp:143`, for the
/// one caller that cannot use [`World::clearance_between`]: a line is not
/// in the arena, so there is no [`ItemId`] to key a cache entry on. KiCad
/// caches it by pointer and reaches this path with a `LINE*`
/// (`pcbnew/router/pns_node.cpp:360`), so it caches on an address that
/// belongs to a stack temporary.
///
/// `-1` is KiCad's "these two can never collide" sentinel arriving in an
/// `int` and being added to a half width regardless (`:361`), which is
/// reproduced rather than tidied because the hull it sizes is only ever
/// compared with other hulls.
fn clearance_of(
  resolver: &dyn RuleResolver,
  item: ItemRef<'_>,
  head: ItemRef<'_>,
  use_epsilon: bool,
) -> i32 {
  // :148
  if item.item().is_virtual() || head.item().is_virtual() {
    return 0;
  }

  resolver
    .clearance(item, Some(head), use_epsilon)
    .unwrap_or(-1)
}

/// Half a via's copper diameter on a layer.
///
/// The `via.Diameter( aLine->Layer() ) / 2` at
/// `pcbnew/router/pns_node.cpp:369`. `None` when the item is not a via,
/// which KiCad's typed `const VIA&` makes unrepresentable.
fn via_radius_on(item: &Item, layer: i32) -> Option<i32> {
  match item.body() {
    ItemBody::Via(via) => Some(via.diameter(item.layers(), layer) / 2),
    _ => None,
  }
}

// ---------------------------------------------------------------------
// Line assembly helpers
// ---------------------------------------------------------------------

/// The three option flags `followLine` forwards to the joint graph.
///
/// The tail of `NODE::followLine`'s parameter list,
/// `pcbnew/router/pns_node.cpp:1076`. They travel together because they
/// are decided once per [`World::assemble_line`] call and never change
/// between the backward and the forward walk.
#[derive(Copy, Clone, Debug)]
struct FollowOptions {
  /// Stop at a joint the shove has pinned (`:1116`).
  stop_at_locked_joints: bool,
  /// Treat a locked segment as a continuation, and ignore a virtual via
  /// (`pcbnew/router/pns_joint.h:261`).
  follow_locked_segments: bool,
  /// Let the line cross a width change (`:1123`).
  allow_segment_size_mismatch: bool,
}

/// The width of a linked item.
///
/// Port of `LINKED_ITEM::Width`,
/// `pcbnew/router/pns_linked_item.h:53`, which is pure virtual and
/// implemented by `SEGMENT`, `ARC` and `VIA`. `followLine` compares two
/// of these to decide whether a width change ends the line
/// (`pcbnew/router/pns_node.cpp:1123`).
///
/// `None` for a body that has no width, which in this crate is a solid or
/// a hole. Neither can be a link, so the comparison never sees one.
fn width_of(item: &Item) -> Option<i32> {
  match item.body() {
    ItemBody::Segment(segment) => Some(segment.width()),
    ItemBody::Arc(arc) => Some(arc.width()),
    ItemBody::Via(via) => {
      Some(via.diameter(item.layers(), item.layers().start()))
    }
    _ => None,
  }
}

/// Whether two arcs describe the same curve travelled the same way.
///
/// The three point comparison [`World::find_redundant_arc`] needs.
/// KiCad's `findRedundantArc` compares only the two anchors
/// (`pcbnew/router/pns_node.cpp:1760`), which is erratum E8. The mid
/// point is here because two arcs between the same endpoints that bulge
/// opposite ways are two different tracks.
///
/// The width is deliberately **not** compared, because KiCad's endpoint
/// test does not compare it either: a redundant arc is one that occupies
/// the same place on the same layer, and the caller that reuses it wants
/// whatever is already there.
fn same_arc_geometry(first: ShapeArc, second: ShapeArc) -> bool {
  first.start() == second.start()
    && first.arc_mid() == second.arc_mid()
    && first.end() == second.end()
}

/// A found point index as KiCad's signed one, with `-1` for not found.
///
/// `SHAPE_LINE_CHAIN::Find` answers `-1`
/// (`libs/kimath/src/geometry/shape_line_chain.cpp:1237`), and
/// `FindLinesBetweenJoints` swaps its two results before it tests them
/// for `>= 0` (`pcbnew/router/pns_node.cpp:1242`), so the sentinel has to
/// survive the swap for the port to answer the same way.
fn index_or_missing(index: Option<usize>) -> isize {
  index.map_or(-1, |value| value as isize)
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::collide::{LineHead, collide_line_into, collide_line_items};
  use crate::geometry::seg::Seg;
  use crate::item::{Segment, Solid, Via, ViaType};
  use crate::rules::FixedClearance;

  /// The clearance every scenario test uses.
  const CLEARANCE: i32 = 2000;

  /// The corner mode every scenario test routes in, which is KiCad's
  /// default (`pcbnew/router/pns_routing_settings.cpp:53`) and the one
  /// that leaves a hull alone; see [`simplified_hull`].
  const CORNERS: CornerMode = CornerMode::Mitered45;

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

  // -----------------------------------------------------------------
  // Lines
  // -----------------------------------------------------------------

  /// Where the run of three segments starts, and its three corners.
  const TRACK_POINTS: [Vec2; 4] = [
    Vec2::new(0, 0),
    Vec2::new(100000, 0),
    Vec2::new(200000, 0),
    Vec2::new(300000, 0),
  ];

  /// The handles of the line fixture.
  ///
  /// Three segments in a row on layer 0, `a` then `b` then `c`, with line
  /// corners between them, plus two more tracks leaving the far end so
  /// that the joint there is a fan out and ends the line.
  struct LineFixture {
    /// The first segment, from the origin.
    a: ItemId,
    /// The middle segment.
    b: ItemId,
    /// The last segment of the run.
    c: ItemId,
    /// One of the two branches at the far end.
    branch_up: ItemId,
  }

  /// Add a width 1000 track on layer 0 to a node.
  fn add_track(
    world: &mut World,
    node: NodeId,
    from: Vec2,
    to: Vec2,
  ) -> ItemId {
    let item = track(world, from, to, 0, NET);

    world
      .add_segment(node, item, false)
      .expect("the track is neither degenerate nor redundant")
  }

  /// The line fixture, built into a fresh world.
  fn line_fixture() -> (World, LineFixture) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    let a = add_track(&mut world, root, TRACK_POINTS[0], TRACK_POINTS[1]);
    let b = add_track(&mut world, root, TRACK_POINTS[1], TRACK_POINTS[2]);
    let c = add_track(&mut world, root, TRACK_POINTS[2], TRACK_POINTS[3]);
    let branch_up =
      add_track(&mut world, root, TRACK_POINTS[3], Vec2::new(300000, 100000));
    add_track(
      &mut world,
      root,
      TRACK_POINTS[3],
      Vec2::new(300000, -100000),
    );

    (world, LineFixture { a, b, c, branch_up })
  }

  /// A loose line over a chain, with the fixture's width, layer and net.
  fn loose_line(points: &[Vec2]) -> Line {
    let mut line = Line::new();

    line.set_width(1000);
    line.set_layer(0);
    line.set_net(NET);
    line.set_shape(LineChain::from_slice(points, false));

    line
  }

  /// The points of a line, for comparing against a literal.
  fn points_of(line: &Line) -> Vec<Vec2> {
    line.shape().points().to_vec()
  }

  #[test]
  fn assemble_line_runs_between_the_two_non_trivial_joints() {
    let (world, ids) = line_fixture();
    let root = world.root();

    let line = world.assemble_line(root, ids.b, None, false, false, true);

    assert_eq!(points_of(&line), TRACK_POINTS);
    assert_eq!(line.links(), [ids.a, ids.b, ids.c]);
    assert_eq!(line.links_valid_in(), Some(root));
    assert!(line.is_linked_checked());
    assert_eq!(line.width(), 1000);
    assert_eq!(line.net(), NET);
    assert_eq!(line.layers(), LayerRange::single(0));
    // `AssembleLine` never attaches the via or the fan out it stopped at.
    assert!(!line.ends_with_via());
  }

  #[test]
  fn assemble_line_stops_at_a_locked_joint_when_asked() {
    let (mut world, ids) = line_fixture();
    let root = world.root();

    world.lock_joint(root, TRACK_POINTS[2], ids.c, true);

    let stopped = world.assemble_line(root, ids.a, None, true, false, true);

    assert_eq!(points_of(&stopped), TRACK_POINTS[..3]);
    assert_eq!(stopped.links(), [ids.a, ids.b]);

    // The flag is what stops it; without it the lock is invisible here.
    let full = world.assemble_line(root, ids.a, None, false, false, true);

    assert_eq!(points_of(&full), TRACK_POINTS);
    assert_eq!(full.links(), [ids.a, ids.b, ids.c]);
  }

  #[test]
  fn assemble_line_stops_at_a_via_and_does_not_pick_it_up() {
    let (mut world, ids) = line_fixture();
    let root = world.root();

    let body =
      ItemBody::Via(Via::new(TRACK_POINTS[2], 3000, 1000, ViaType::Through));
    let mut item = world.make_item(body);
    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item.set_net(NET);
    let via = world.add_via(root, item);

    let line = world.assemble_line(root, ids.a, None, false, false, true);

    assert_eq!(points_of(&line), TRACK_POINTS[..3]);
    assert_eq!(line.links(), [ids.a, ids.b]);
    assert!(!line.ends_with_via());
    assert!(!line.contains_link(via));
  }

  #[test]
  fn assemble_line_stops_at_a_width_change_only_when_asked() {
    let (mut world, ids) = line_fixture();
    let root = world.root();

    if let Some(item) = world.item_mut(ids.c)
      && let ItemBody::Segment(segment) = item.body_mut()
    {
      segment.set_width(2000);
    }

    let crossing = world.assemble_line(root, ids.a, None, false, false, true);
    assert_eq!(crossing.links(), [ids.a, ids.b, ids.c]);

    let stopped = world.assemble_line(root, ids.a, None, false, false, false);
    assert_eq!(stopped.links(), [ids.a, ids.b]);
  }

  #[test]
  fn the_origin_segment_index_names_the_seed_segment() {
    let (world, ids) = line_fixture();
    let root = world.root();

    for (seed, expected) in [(ids.a, 0), (ids.b, 1), (ids.c, 2)] {
      let mut origin = usize::MAX;
      let line =
        world.assemble_line(root, seed, Some(&mut origin), false, false, true);

      assert_eq!(origin, expected);
      assert_eq!(line.segment(origin), line.shape().segment(expected));
    }

    // A handle the arena does not know leaves the caller's value alone
    // and answers with an empty line, where KiCad would dereference it.
    let mut world = world;
    world.remove(root, ids.branch_up);

    let mut origin = 7;
    let empty = world.assemble_line(
      root,
      ids.branch_up,
      Some(&mut origin),
      false,
      false,
      true,
    );

    assert_eq!(origin, 7);
    assert_eq!(empty.point_count(), 0);
  }

  #[test]
  fn add_line_stores_every_segment_and_links_it() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let mut line = loose_line(&TRACK_POINTS[..3]);

    let added = world.add_line(root, &mut line, false);

    assert_eq!(added.len(), 2);
    assert_eq!(line.links(), added);
    assert_eq!(line.links_valid_in(), Some(root));
    assert!(line.is_linked_checked());

    for id in &added {
      let item = world.item(*id).expect("the segment was stored");

      assert_eq!(item.net(), NET);
      assert_eq!(item.layers(), LayerRange::single(0));
      assert!(matches!(item.body(), ItemBody::Segment(_)));
    }

    // Both endpoints and the corner carry a joint now.
    for point in &TRACK_POINTS[..3] {
      assert!(world.find_joint(root, *point, 0, NET).is_some());
    }
  }

  #[test]
  fn add_line_reuses_a_segment_that_is_already_there() {
    let (mut world, ids) = line_fixture();
    let root = world.root();

    let mut reusing = loose_line(&[TRACK_POINTS[0], TRACK_POINTS[1]]);
    let added = world.add_line(root, &mut reusing, false);

    assert_eq!(added, [ids.a]);
    assert_eq!(reusing.links(), [ids.a]);

    // With `allow_redundant` the lookup never happens and a second,
    // geometrically identical segment goes in.
    let mut duplicating = loose_line(&[TRACK_POINTS[0], TRACK_POINTS[1]]);
    let added = world.add_line(root, &mut duplicating, true);

    assert_eq!(added.len(), 1);
    assert_ne!(added[0], ids.a);
    assert!(world.item(added[0]).is_some());
  }

  #[test]
  fn add_line_skips_a_zero_length_segment() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    let mut chain = LineChain::new();
    chain.append(TRACK_POINTS[0]);
    chain.append_allow_duplicate(TRACK_POINTS[0]);
    chain.append(TRACK_POINTS[1]);

    let mut line = loose_line(&[]);
    line.set_shape(chain);

    let added = world.add_line(root, &mut line, false);

    assert_eq!(added.len(), 1);
    assert_eq!(line.link_count(), 1);
    // Two shapes, one link: the invariant cannot hold for this input.
    assert!(!line.is_linked_checked());
  }

  #[test]
  fn remove_line_takes_the_segments_and_a_linked_via_out() {
    let (mut world, ids) = line_fixture();
    let root = world.root();

    let body =
      ItemBody::Via(Via::new(TRACK_POINTS[0], 3000, 1000, ViaType::Through));
    let mut item = world.make_item(body);
    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item.set_net(NET);
    let via = world.add_via(root, item);

    let mut line = world.assemble_line(root, ids.b, None, false, false, true);
    line.link_via(via, TRACK_POINTS[0]);

    // The via sits at point 0, so linking it reversed the line.
    assert_eq!(line.last_point(), Some(TRACK_POINTS[0]));

    world.remove_line(root, &mut line);

    assert!(!line.is_linked());
    assert_eq!(line.links_valid_in(), None);

    for id in [ids.a, ids.b, ids.c, via] {
      assert!(world.item(id).is_none(), "the item left the arena");
    }

    // The fan out at the far end is untouched.
    assert!(world.item(ids.branch_up).is_some());
    assert!(world.find_joint(root, TRACK_POINTS[1], 0, NET).is_none());
  }

  #[test]
  fn replace_line_swaps_the_geometry_and_moves_the_links() {
    let (mut world, ids) = line_fixture();
    let root = world.root();

    let mut old = world.assemble_line(root, ids.a, None, false, false, true);
    let mut new_line = loose_line(&[
      TRACK_POINTS[0],
      Vec2::new(150000, 150000),
      TRACK_POINTS[3],
    ]);

    let added = world.replace_line(root, &mut old, &mut new_line, false);

    assert!(!old.is_linked());
    assert_eq!(added.len(), 2);
    assert_eq!(new_line.links(), added);

    for id in [ids.a, ids.b, ids.c] {
      assert!(world.item(id).is_none());
    }

    assert!(
      world
        .find_joint(root, Vec2::new(150000, 150000), 0, NET)
        .is_some()
    );
    // The fan out still holds the joint at the far end.
    assert!(world.find_joint(root, TRACK_POINTS[3], 0, NET).is_some());
  }

  #[test]
  fn find_lines_between_joints_clips_to_the_vertex_range() {
    let (world, ids) = line_fixture();
    let root = world.root();

    let first = world
      .find_joint(root, TRACK_POINTS[0], 0, NET)
      .expect("the run starts at a joint");
    let second = world
      .find_joint(root, TRACK_POINTS[2], 0, NET)
      .expect("the middle corner is a joint");

    let lines = world.find_lines_between_joints(root, first, second);

    assert_eq!(lines.len(), 1);
    assert_eq!(points_of(&lines[0]), TRACK_POINTS[..3]);
    assert_eq!(lines[0].links(), [ids.a, ids.b]);

    // A joint the line does not reach is discarded, whichever end it is.
    let elsewhere = world
      .find_joint(root, Vec2::new(300000, 100000), 0, NET)
      .expect("the branch ends at a joint");

    assert!(
      world
        .find_lines_between_joints(root, first, elsewhere)
        .is_empty()
    );
  }

  // -----------------------------------------------------------------
  // Arcs
  // -----------------------------------------------------------------

  /// A quarter turn from the origin, bulging below the chord.
  ///
  /// Centre `(100000, 0)`, radius 100000, so every one of the three
  /// points is on the circle to within a nanometre and the arc is an
  /// ordinary track corner rather than a near degenerate one.
  const QUARTER: ShapeArc = ShapeArc::new(
    Vec2::new(0, 0),
    Vec2::new(29289, 70711),
    Vec2::new(100_000, 100_000),
    1000,
  );

  /// The second quarter turn of the arc, straight, arc chain, starting
  /// where the straight run ends.
  const SECOND_QUARTER: ShapeArc = ShapeArc::new(
    Vec2::new(200_000, 100_000),
    Vec2::new(229_289, 170_711),
    Vec2::new(300_000, 200_000),
    1000,
  );

  /// A width 1000 arc track on layer 0, not yet stored.
  fn arc_track(world: &mut World, arc: ShapeArc, net: Option<NetId>) -> Item {
    let mut item = world.make_item(ItemBody::Arc(Arc::new(arc)));

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);

    item
  }

  /// Store a width 1000 arc track on layer 0, refusing a redundant one.
  fn add_arc_track(world: &mut World, node: NodeId, arc: ShapeArc) -> ItemId {
    let item = arc_track(world, arc, NET);

    world
      .add_arc(node, item, false)
      .expect("the arc is not redundant")
  }

  /// The three points of an arc, for comparing against a literal.
  ///
  /// An arc stored in a [`LineChain`] always has width zero
  /// (`shape_line_chain.cpp:1624`), so the width is left out of every
  /// comparison against the width 1000 track it came from.
  fn arc_points(arc: ShapeArc) -> [Vec2; 3] {
    [arc.start(), arc.arc_mid(), arc.end()]
  }

  /// The three points of the one arc a line carries.
  fn arc_of(line: &Line) -> ShapeArc {
    let arcs: Vec<ShapeArc> =
      line.shape().live_arcs().map(|(_, arc)| *arc).collect();

    assert_eq!(arcs.len(), 1, "the line carries exactly one arc");

    arcs[0]
  }

  /// Erratum E8, fixed: two arcs between the same endpoints that bulge
  /// opposite ways are two tracks, not one.
  #[test]
  fn find_redundant_arc_compares_the_mid_point_erratum_e8() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    let stored = add_arc_track(&mut world, root, QUARTER);

    // The same three points again is redundant, which is the behaviour
    // the mid point comparison must not break.
    let duplicate = arc_track(&mut world, QUARTER, NET);
    assert!(world.add_arc(root, duplicate, false).is_none());

    // So is the same curve travelled the other way, which is what
    // KiCad's "in either order" endpoint test is for (`:1760`).
    let backwards = arc_track(&mut world, QUARTER.reversed(), NET);
    assert!(world.add_arc(root, backwards, false).is_none());

    // The other bulge between the same two endpoints is not. KiCad
    // reuses the stored arc here and the board loses a track.
    let mirrored = ShapeArc::new(
      QUARTER.start(),
      Vec2::new(70711, 29289),
      QUARTER.end(),
      1000,
    );
    let other = arc_track(&mut world, mirrored, NET);
    let other = world
      .add_arc(root, other, false)
      .expect("the opposite bulge is a different track");

    assert_ne!(other, stored);

    // Both are linked to the joint at the shared start point.
    let joint = world
      .find_joint(root, QUARTER.start(), 0, NET)
      .expect("the two arcs meet at a joint");
    let links = world.joint(joint).expect("the joint is live").links();

    assert_eq!(links, [stored, other]);
  }

  /// Erratum E36, reproduced: `Add( ARC )` has no degenerate check.
  ///
  /// `Add( SEGMENT )` refuses a segment whose ends coincide (`:749`) and
  /// the arc overload has no counterpart, so a degenerate arc goes in.
  /// What that costs is small and is asserted here: one joint holding the
  /// arc once, because [`Joint::link`] refuses the second link at the
  /// same position, and a point sized entry in the index.
  #[test]
  fn add_arc_accepts_a_degenerate_arc_erratum_e36() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    let at = Vec2::new(50000, 50000);
    let degenerate = ShapeArc::new(at, at, at, 1000);
    let item = arc_track(&mut world, degenerate, NET);
    let stored = world
      .add_arc(root, item, false)
      .expect("a degenerate arc is accepted where a degenerate segment is not");

    // A degenerate segment at the same place is refused, which is the
    // asymmetry the erratum names.
    let flat = track(&mut world, at, at, 0, NET);
    assert!(world.add_segment(root, flat, false).is_none());

    let joint = world
      .find_joint(root, at, 0, NET)
      .expect("the two anchors made one joint");

    assert_eq!(world.joint(joint).expect("live").links(), [stored]);
    assert_eq!(world.hit_test(root, at), vec![stored]);

    // And it comes out again cleanly, one unlink per anchor against one
    // link.
    world.remove(root, stored);

    assert!(world.item(stored).is_none());
    assert!(world.find_joint(root, at, 0, NET).is_none());
  }

  /// Erratum E22, fixed: an arc, straight, arc line links in chain order.
  ///
  /// KiCad runs the arc loop first and the segment loop second, so the
  /// links come out arc, arc, segment and `LINE::ClipVertexRange`, which
  /// counts links in shape order, maps the wrong one.
  #[test]
  fn add_line_links_in_chain_order_erratum_e22() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    let mut chain = LineChain::new();
    chain.append_arc(&QUARTER, LineChain::ARC_POLYGONIZATION_MAX_ERROR);
    chain.append(Vec2::new(200_000, 100_000));
    chain.append_arc(&SECOND_QUARTER, LineChain::ARC_POLYGONIZATION_MAX_ERROR);

    let mut line = loose_line(&[]);
    line.set_shape(chain);

    let added = world.add_line(root, &mut line, true);

    let kinds: Vec<Kind> = added
      .iter()
      .filter_map(|id| world.item(*id).map(Item::kind))
      .collect();

    assert_eq!(kinds, [Kind::ARC, Kind::SEGMENT, Kind::ARC]);
    assert_eq!(line.links(), added);

    // Each arc item carries the chain's three points with the line's
    // width, which the chain's own copy does not hold
    // (`shape_line_chain.cpp:1624`).
    let stored: Vec<ShapeArc> = added
      .iter()
      .filter_map(|id| match world.item(*id)?.body() {
        ItemBody::Arc(arc) => Some(arc.arc()),
        _ => None,
      })
      .collect();

    assert_eq!(stored, [QUARTER, SECOND_QUARTER]);
    assert!(stored.iter().all(|arc| arc.width() == 1000));

    // The straight item is the one segment between the two curves.
    let segment = added[1];
    let ItemBody::Segment(body) = world.item(segment).expect("live").body()
    else {
      panic!("the middle link is a segment");
    };

    assert_eq!(body.seg().a, QUARTER.end());
    assert_eq!(body.seg().b, SECOND_QUARTER.start());
  }

  /// The fixture `assemble_line` walks over an arc.
  ///
  /// A straight run into the arc's start, the arc, a straight run out of
  /// its end, and a fan out at each far end so that the line stops there.
  /// `reversed` stores the arc pointing against the direction of travel,
  /// which is what sets `followLine`'s `aArcReversed` (`:1096`).
  ///
  /// The outbound segment is deliberately stored **from** the far fan out
  /// **to** the arc's end. An assembled line runs in the direction the
  /// seed's own two anchors point, so a fixture whose segments all point
  /// the same way is walked the same way whichever of them seeds it; this
  /// one is walked in one direction from `first` and in the other from
  /// `last`, which is the only way to reach `aArcReversed` from both
  /// sides.
  fn arc_line_fixture(reversed: bool) -> (World, ItemId, ItemId) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    let before = Vec2::new(-100_000, 0);
    let after = Vec2::new(200_000, 100_000);

    let first = add_track(&mut world, root, before, QUARTER.start());
    let arc = if reversed {
      QUARTER.reversed()
    } else {
      QUARTER
    };

    add_arc_track(&mut world, root, arc);

    let last = add_track(&mut world, root, after, QUARTER.end());

    // Fan outs, so that neither end is a line corner.
    add_track(&mut world, root, before, Vec2::new(-100_000, 100_000));
    add_track(&mut world, root, before, Vec2::new(-100_000, -100_000));
    add_track(&mut world, root, after, Vec2::new(300_000, 100_000));
    add_track(&mut world, root, after, Vec2::new(200_000, 200_000));

    (world, first, last)
  }

  /// An arc stored the way the line travels comes back unchanged.
  #[test]
  fn assemble_line_walks_through_an_arc_stored_forwards() {
    let (world, first, _) = arc_line_fixture(false);
    let root = world.root();

    let line = world.assemble_line(root, first, None, false, false, true);

    assert_eq!(line.point(0), Vec2::new(-100_000, 0));
    assert_eq!(line.last_point(), Some(Vec2::new(200_000, 100_000)));
    assert_eq!(arc_points(arc_of(&line)), arc_points(QUARTER));
    assert_eq!(line.link_count(), 3);
  }

  /// An arc stored against the direction of travel is reversed on the
  /// way in, so the assembled chain still runs start to end.
  #[test]
  fn assemble_line_walks_through_an_arc_stored_backwards() {
    let (world, first, _) = arc_line_fixture(true);
    let root = world.root();

    let line = world.assemble_line(root, first, None, false, false, true);

    assert_eq!(line.point(0), Vec2::new(-100_000, 0));
    assert_eq!(line.last_point(), Some(Vec2::new(200_000, 100_000)));
    assert_eq!(arc_points(arc_of(&line)), arc_points(QUARTER));
  }

  /// Seeding the walk from the other end gives the same physical line
  /// the other way round, arc included.
  ///
  /// This is where KiCad's erratum E4 lives: it appends `Reversed()`,
  /// which re runs `CalcArcCenter` on the permuted points and can land on
  /// a different centre, so the two directions can disagree about the
  /// same curve. Nothing is cached here, so they cannot.
  #[test]
  fn assemble_line_from_the_other_end_reverses_the_arc_and_nothing_else() {
    let (world, first, last) = arc_line_fixture(false);
    let root = world.root();

    let forwards = world.assemble_line(root, first, None, false, false, true);
    let backwards = world.assemble_line(root, last, None, false, false, true);

    assert_eq!(backwards.point(0), Vec2::new(200_000, 100_000));
    assert_eq!(backwards.last_point(), Some(Vec2::new(-100_000, 0)));

    assert_eq!(
      arc_points(arc_of(&backwards)),
      arc_points(QUARTER.reversed())
    );

    let mut reversed_points = points_of(&backwards);
    reversed_points.reverse();

    assert_eq!(reversed_points, points_of(&forwards));
    assert_eq!(backwards.link_count(), forwards.link_count());
  }

  #[test]
  fn every_line_answer_is_the_same_across_two_identical_runs() {
    /// What one run of the line operations produces.
    #[derive(PartialEq, Debug)]
    struct Answers {
      /// The points of the assembled line.
      assembled: Vec<Vec2>,
      /// Its links.
      links: Vec<ItemId>,
      /// The origin index of the seed segment.
      origin: usize,
      /// What `add_line` stored for a fresh line.
      added: Vec<ItemId>,
      /// The clipped lines between the two joints.
      between: Vec<Vec<Vec2>>,
    }

    fn run() -> Answers {
      let (mut world, ids) = line_fixture();
      let root = world.root();
      let mut origin = usize::MAX;
      let assembled =
        world.assemble_line(root, ids.b, Some(&mut origin), false, false, true);

      let first = world
        .find_joint(root, TRACK_POINTS[0], 0, NET)
        .expect("the run starts at a joint");
      let second = world
        .find_joint(root, TRACK_POINTS[2], 0, NET)
        .expect("the middle corner is a joint");
      let between = world
        .find_lines_between_joints(root, first, second)
        .iter()
        .map(points_of)
        .collect();

      let mut fresh = loose_line(&[
        Vec2::new(0, 400000),
        Vec2::new(100000, 400000),
        Vec2::new(200000, 300000),
      ]);
      let added = world.add_line(root, &mut fresh, false);

      Answers {
        assembled: points_of(&assembled),
        links: assembled.links().to_vec(),
        origin,
        added,
        between,
      }
    }

    assert_eq!(run(), run());
  }

  // -----------------------------------------------------------------
  // The line queries and the nearest obstacle
  // -----------------------------------------------------------------

  /// Where the near obstacle of the crossing fixture stands.
  const NEAR_X: i32 = 100000;

  /// Where the far obstacle of the crossing fixture stands.
  const FAR_X: i32 = 200000;

  /// How far the head line of the crossing fixture runs.
  const HEAD_END: i32 = 300000;

  /// The resolver every line query test uses.
  fn rules() -> FixedClearance {
    FixedClearance::uniform(CLEARANCE)
  }

  /// A head line the placer could be dragging: width 1000, layer 0, on
  /// [`OTHER_NET`] so that nothing in a fixture exempts it.
  fn head_line(points: &[Vec2]) -> Line {
    let mut line = Line::new();

    line.set_width(1000);
    line.set_layer(0);
    line.set_net(OTHER_NET);
    line.set_shape(LineChain::from_slice(points, false));

    line
  }

  /// A vertical track on layer 0, 50 millimetres either side of the x
  /// axis, which a head line along that axis has to cross.
  fn crossing_track(world: &mut World, node: NodeId, x: i32) -> ItemId {
    let item = track(world, Vec2::new(x, -50000), Vec2::new(x, 50000), 0, NET);

    world
      .add_segment(node, item, false)
      .expect("the crossing track is neither degenerate nor redundant")
  }

  /// The head line of the crossing fixture, from the origin along the x
  /// axis past both tracks.
  fn crossing_head() -> Line {
    head_line(&[Vec2::new(0, 0), Vec2::new(HEAD_END, 0)])
  }

  #[test]
  fn nearest_obstacle_answers_nothing_when_the_line_is_clear() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    crossing_track(&mut world, root, NEAR_X);

    let clear = head_line(&[Vec2::new(0, 400000), Vec2::new(HEAD_END, 400000)]);
    let options = CollisionSearchOptions::default();

    assert!(
      world
        .nearest_obstacle(root, &clear, &rules(), &options, CORNERS)
        .is_none()
    );
  }

  #[test]
  fn nearest_obstacle_measures_where_the_line_meets_the_hull() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let near = crossing_track(&mut world, root, NEAR_X);
    let head = crossing_head();
    let options = CollisionSearchOptions::default();

    let found = world
      .nearest_obstacle(root, &head, &rules(), &options, CORNERS)
      .expect("the line crosses the track");

    assert_eq!(found.item, Some(near));
    assert!(found.found_intersection);
    // The hull reaches back towards the line's start, so the line enters
    // it well before it reaches the track itself.
    assert!(found.dist_first > 0);
    assert!(found.dist_first < i64::from(NEAR_X));
    assert_eq!(found.ip_first.y, 0);

    let hull = found.hull.as_ref().expect("a live obstacle has a hull");

    assert!(hull.point_on_edge(found.ip_first, 0));
    // The two extras KiCad never fills in here.
    assert_eq!(found.pos, Vec2::new(0, 0));
    assert_eq!(found.max_fanout_width, 0);
  }

  #[test]
  fn nearest_obstacle_picks_the_nearer_of_two_tracks() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let near = crossing_track(&mut world, root, NEAR_X);
    let far = crossing_track(&mut world, root, FAR_X);
    let head = crossing_head();
    let options = CollisionSearchOptions::default();

    let every = world.query_colliding_line(root, &head, &rules(), &options);

    assert_eq!(
      every
        .iter()
        .filter_map(|found| found.item)
        .collect::<Vec<_>>(),
      [near, far]
    );

    let found = world
      .nearest_obstacle(root, &head, &rules(), &options, CORNERS)
      .expect("the line crosses both tracks");

    assert_eq!(found.item, Some(near));
    assert!(found.dist_first < i64::from(FAR_X));
  }

  /// The far track is added **first**, so it holds the smaller uid. The
  /// nearer one still wins, which is what makes the scan a distance test
  /// and not an order test.
  #[test]
  fn the_nearer_track_wins_even_with_the_larger_uid() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let far = crossing_track(&mut world, root, FAR_X);
    let near = crossing_track(&mut world, root, NEAR_X);
    let head = crossing_head();
    let options = CollisionSearchOptions::default();

    assert!(world.item(far).map(Item::uid) < world.item(near).map(Item::uid));

    let found = world
      .nearest_obstacle(root, &head, &rules(), &options, CORNERS)
      .expect("the line crosses both tracks");

    assert_eq!(found.item, Some(near));
  }

  /// `DESIGN.md` section 8 and note 04 section 9 item 2: obstacles at one
  /// distance are separated by the item uid, where KiCad's are separated
  /// by an address.
  ///
  /// The two tracks are mirror images across the head line's axis, so
  /// their hulls meet the line at the same point and the geometry cannot
  /// decide between them. Whichever was added first wins, in both
  /// orders.
  #[test]
  fn a_distance_tie_is_broken_by_the_item_uid() {
    /// One world, with the upper or the lower track added first, and the
    /// handle of the one added first.
    fn run(above_first: bool) -> (ItemId, NearestObstacle) {
      let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
      let root = world.root();
      let above = (Vec2::new(NEAR_X, 1000), Vec2::new(NEAR_X, 50000));
      let below = (Vec2::new(NEAR_X, -50000), Vec2::new(NEAR_X, -1000));
      let (first, second) = if above_first {
        (above, below)
      } else {
        (below, above)
      };

      let item = track(&mut world, first.0, first.1, 0, NET);
      let first = world
        .add_segment(root, item, false)
        .expect("the first mirrored track is fine");
      let item = track(&mut world, second.0, second.1, 0, NET);
      world
        .add_segment(root, item, false)
        .expect("the second mirrored track is fine");

      let head = crossing_head();
      let found = world
        .nearest_obstacle(
          root,
          &head,
          &rules(),
          &CollisionSearchOptions::default(),
          CORNERS,
        )
        .expect("the line crosses both hulls");

      (first, found)
    }

    let (above_first, from_above) = run(true);
    let (below_first, from_below) = run(false);

    assert_eq!(from_above.dist_first, from_below.dist_first);
    assert_eq!(from_above.ip_first, from_below.ip_first);
    assert_eq!(from_above.item, Some(above_first));
    assert_eq!(from_below.item, Some(below_first));
  }

  /// How many tracks the tie fixture stacks at one place.
  ///
  /// More than twice [`MIN_CANDIDATES_PER_BLOCK`], so the query splits
  /// into at least two blocks at any parallelism above one and the tie is
  /// actually resolved across a thread boundary rather than inside one
  /// block.
  const TIED_TRACKS: usize = 2 * MIN_CANDIDATES_PER_BLOCK + 1;

  /// A stack of tracks the head line runs into all at once.
  ///
  /// Every one of them is the same segment on the same layer at the same
  /// place, at the same width, so the resolver gives every one the same
  /// clearance and [`World::hull_of`] builds every one the same hull:
  /// the head enters all of them at one path length. Only the net
  /// differs, which is what stops [`World::add_segment`] rejecting the
  /// second one as redundant, and none of those nets is the head's, so
  /// nothing is exempt.
  ///
  /// The handles come back in creation order, which is uid order, so the
  /// first is the answer a `(distance, uid)` tie break has to give.
  fn tied_obstacle_stack() -> (World, NodeId, Vec<ItemId>) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let mut tracks = Vec::with_capacity(TIED_TRACKS);

    for index in 0..TIED_TRACKS {
      let net = Some(NetId(100 + index as u32));
      let item = track(
        &mut world,
        Vec2::new(NEAR_X, -50000),
        Vec2::new(NEAR_X, 50000),
        0,
        net,
      );

      tracks.push(
        world
          .add_segment(root, item, false)
          .expect("a track on its own net is not redundant"),
      );
    }

    (world, root, tracks)
  }

  /// How many hulls the block split fixture holds.
  ///
  /// Enough for six blocks at [`MIN_CANDIDATES_PER_BLOCK`], and not a
  /// multiple of it, so the last block is short and the chunking has to
  /// handle a remainder.
  const SPLIT_HULLS: usize = 6 * MIN_CANDIDATES_PER_BLOCK + 7;

  /// A closed square of side `2 * radius` centred on `(x, 0)`.
  ///
  /// A stand in for a hull: [`nearest_crossing`] only ever reads a hull
  /// as a chain, so a square is as good as an octagon and says exactly
  /// where it crosses a path along the x axis.
  fn square_hull(x: i32, radius: i32) -> LineChain {
    LineChain::from_slice(
      &[
        Vec2::new(x - radius, -radius),
        Vec2::new(x + radius, -radius),
        Vec2::new(x + radius, radius),
        Vec2::new(x - radius, radius),
      ],
      true,
    )
  }

  /// The candidate scan gives one answer whatever the block count is.
  ///
  /// The direct test of the split
  /// [`World::nearest_obstacle`] relies on: the whole result vector, not
  /// just the winner, has to be the same at every parallelism, because
  /// the winner scan that reads it runs afterwards and reads all of it.
  /// The hulls are at increasing distances so that every entry is a
  /// different number and a block that landed in the wrong place could
  /// not go unnoticed.
  #[test]
  fn the_candidate_scan_answers_the_same_at_every_block_count() {
    let hulls: Vec<LineChain> = (0..SPLIT_HULLS)
      .map(|index| square_hull(100000 + 10000 * index as i32, 4000))
      .collect();
    let borrowed: Vec<HullRefs<'_>> = hulls
      .iter()
      .map(|hull| HullRefs {
        line: Some(hull),
        via: None,
      })
      .collect();
    let path = LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(100000 + 10000 * SPLIT_HULLS as i32, 0),
      ],
      false,
    );
    let sequential = scan_candidates(&borrowed, &path, 1);

    // Every hull really is crossed, and at its own distance, so the
    // comparison below has something to compare.
    assert_eq!(sequential.len(), SPLIT_HULLS);
    assert!(sequential.iter().all(Option::is_some));
    assert!(
      sequential
        .windows(2)
        .all(|pair| pair[0].unwrap().0 < pair[1].unwrap().0)
    );

    for threads in 2..=16 {
      assert_eq!(
        scan_candidates(&borrowed, &path, threads),
        sequential,
        "the scan answers differently on {threads} thread(s)"
      );
    }
  }

  /// The same tie break, over a stack big enough to be cut into blocks.
  ///
  /// [`a_distance_tie_is_broken_by_the_item_uid`] pins the rule on two
  /// candidates, which is one block at any thread count. This one pins
  /// that the rule survives the split: the winner is the lowest uid
  /// whether the scan ran on one thread or on several, and the distance
  /// and the crossing point it reports are the same either way.
  #[test]
  fn a_distance_tie_goes_to_the_lowest_uid_at_every_thread_count() {
    let (mut world, root, tracks) = tied_obstacle_stack();
    let head = crossing_head();
    let options = CollisionSearchOptions::default();

    // The fixture ties, and it does not tie at zero: a candidate at
    // distance zero would end the winner scan at the first index and the
    // test would pass without ever comparing anything.
    let mut crossings = Vec::with_capacity(tracks.len());

    for id in &tracks {
      let clearance =
        world.clearance_for_line(*id, &head, true, &rules()) + head.width() / 2;
      let hull = world
        .hull_of(*id, clearance, 0, 0)
        .expect("a live track has a hull");

      crossings.push(nearest_crossing(
        HullRefs {
          line: Some(hull.as_ref()),
          via: None,
        },
        head.shape(),
      ));
    }

    let tie = crossings[0].expect("the head crosses the stack");

    assert!(tie.0 > 0, "the fixture ties at distance zero");
    assert!(
      crossings.iter().all(|crossing| *crossing == Some(tie)),
      "the fixture does not tie: {crossings:?}"
    );

    // The whole stack really is one query's worth of candidates, so the
    // block split below is not over an empty list.
    assert_eq!(
      world
        .query_colliding_line(root, &head, &rules(), &options)
        .len(),
      TIED_TRACKS
    );

    for threads in [1, 2, 3, 8, 64] {
      world.set_parallelism(threads);

      let found = world
        .nearest_obstacle(root, &head, &rules(), &options, CORNERS)
        .expect("the head crosses the stack");

      assert_eq!(
        found.item,
        Some(tracks[0]),
        "with {threads} thread(s) the tie went to uid {:?} and not to the \
         lowest, {:?}",
        world.uid_of(found.item),
        world.uid_of(Some(tracks[0]))
      );
      assert_eq!(found.dist_first, tie.0);
      assert_eq!(found.ip_first, tie.1);
    }
  }

  /// Note 04 section 9 item 4: when no hull meets the line, KiCad falls
  /// back to the address first obstacle. Here it is the uid first one,
  /// and the distance it reports is the zero `collideSimple` left behind.
  #[test]
  fn a_line_inside_a_hull_falls_back_to_the_first_obstacle() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let item = pad(&mut world, Vec2::new(0, 0), 0);
    let disc = world.add_solid(root, item, None);
    let options = CollisionSearchOptions::default();

    // Both points sit well inside the pad's hull, so the hull and the
    // line never cross.
    let head = head_line(&[Vec2::new(0, 0), Vec2::new(1000, 0)]);
    let found = world
      .nearest_obstacle(root, &head, &rules(), &options, CORNERS)
      .expect("the line is inside the pad's clearance");

    assert_eq!(found.item, Some(disc));
    assert!(!found.found_intersection);
    assert_eq!(found.dist_first, 0);
    assert_eq!(found.ip_first, Vec2::new(0, 0));
  }

  /// The zero distance case, which is also the early break at
  /// `pcbnew/router/pns_node.cpp:466`. The break cannot change an answer,
  /// since nothing beats zero under a strict `<`; what this pins is that
  /// a line starting on a hull measures zero and still names its
  /// obstacle.
  #[test]
  fn a_line_starting_on_a_hull_is_at_distance_zero() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let item = pad(&mut world, Vec2::new(0, 0), 0);
    let disc = world.add_solid(root, item, None);
    crossing_track(&mut world, root, NEAR_X);
    let options = CollisionSearchOptions::default();

    // The clearance `nearest_obstacle` builds the hull at: the rule plus
    // half the line's width, with a walkaround thickness of zero.
    let hull = world
      .hull_of(disc, CLEARANCE + 500, 0, 0)
      .expect("the pad is live");
    let start = hull.point(0);
    let head = head_line(&[start, Vec2::new(0, 0)]);

    let found = world
      .nearest_obstacle(root, &head, &rules(), &options, CORNERS)
      .expect("the line starts on the pad's hull");

    assert_eq!(found.item, Some(disc));
    assert!(found.found_intersection);
    assert_eq!(found.dist_first, 0);
    assert_eq!(found.ip_first, start);
  }

  #[test]
  fn a_line_with_an_owned_via_collides_through_the_via() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let options = CollisionSearchOptions::default();

    // A pad on the far layer, which the line's own segment can never
    // reach.
    let item = pad(&mut world, Vec2::new(NEAR_X, 0), 1);
    let target = world.add_solid(root, item, None);

    let body = ItemBody::Via(Via::new(
      Vec2::new(NEAR_X, 0),
      3000,
      1000,
      ViaType::Through,
    ));
    let mut via = world.make_item(body);
    via.set_layers_and_flash_all(LayerRange::new(0, 1));

    let mut head = head_line(&[Vec2::new(0, 0), Vec2::new(NEAR_X, 0)]);
    head.append_via(via);

    // The segment on its own finds nothing: it is on layer 0 and the pad
    // is on layer 1.
    let seg = head.segment_item(&world, 0, u64::MAX);

    assert!(
      world
        .check_colliding(root, ItemRef::unstored(&seg), &rules(), &options)
        .is_none()
    );

    // The line finds it, because the via is queried as well.
    let found = world.query_colliding_line(root, &head, &rules(), &options);

    assert_eq!(
      found
        .iter()
        .filter_map(|found| found.item)
        .collect::<Vec<_>>(),
      [target]
    );
    assert!(
      world
        .check_colliding_line(root, &head, &rules(), &options)
        .is_some()
    );
  }

  #[test]
  fn check_colliding_line_agrees_with_check_colliding_on_one_segment() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    crossing_track(&mut world, root, NEAR_X);
    let options = CollisionSearchOptions::default();

    let head = crossing_head();
    let seg = head.segment_item(&world, 0, u64::MAX);

    let from_line = world
      .check_colliding_line(root, &head, &rules(), &options)
      .expect("the line crosses the track");
    let from_item = world
      .check_colliding(root, ItemRef::unstored(&seg), &rules(), &options)
      .expect("so does the one segment it is made of");

    assert_eq!(from_line.item, from_item.item);
    assert_eq!(from_line.clearance, from_item.clearance);
    assert_eq!(from_line.detail, from_item.detail);
  }

  /// A line on [`NET`], so that it is not exempt from a head on
  /// [`OTHER_NET`].
  fn board_line(points: &[Vec2]) -> Line {
    let mut line = head_line(points);

    line.set_net(NET);
    line
  }

  #[test]
  fn collide_lines_finds_two_parallel_lines_that_are_too_close() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let head = crossing_head();
    // Half a width each plus the clearance is 3000, so 2000 apart is a
    // collision and 10000 apart is not.
    let near = board_line(&[Vec2::new(0, 2000), Vec2::new(HEAD_END, 2000)]);
    let far = board_line(&[Vec2::new(0, 10000), Vec2::new(HEAD_END, 10000)]);
    let options = CollisionSearchOptions::default();

    assert!(
      world
        .collide_lines(&near, &head, &rules(), &options)
        .is_some()
    );
    assert!(
      world
        .collide_lines(&far, &head, &rules(), &options)
        .is_none()
    );
  }

  #[test]
  fn collide_lines_answers_the_same_as_one_decomposed_segment() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let head = crossing_head();
    let near = board_line(&[Vec2::new(0, 2000), Vec2::new(HEAD_END, 2000)]);
    let options = CollisionSearchOptions::default();

    let probe = head.rule_item(&world, PROBE_UID);
    let line_head =
      LineHead::new(ItemRef::unstored(&probe), &head, head.via_item(&world));
    let segment = near.segment_item(&world, 0, PROBE_UID);

    let from_lines = world
      .collide_lines(&near, &head, &rules(), &options)
      .expect("the two lines run 2000 apart");
    let from_segment = collide_line_items(
      &world.items,
      ItemRef::unstored(&segment),
      &line_head,
      &rules(),
      &options,
    )
    .expect("so does the one segment the obstacle is made of");

    assert_eq!(from_lines.clearance, from_segment.clearance);
    assert_eq!(from_lines.detail, from_segment.detail);
  }

  #[test]
  fn collide_lines_exempts_two_lines_of_one_net() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let mut head = crossing_head();

    head.set_net(NET);

    let near = board_line(&[Vec2::new(0, 2000), Vec2::new(HEAD_END, 2000)]);
    let options = CollisionSearchOptions::default();

    assert!(
      world
        .collide_lines(&near, &head, &rules(), &options)
        .is_none()
    );
  }

  #[test]
  fn check_colliding_items_stops_at_the_first_colliding_item() {
    let (world, ids) = fixture();
    let root = world.root();
    let options = CollisionSearchOptions::default();

    let clear = probe(Vec2::new(0, 400000), Vec2::new(1000, 400000), 0);
    let over_the_via =
      probe(Vec2::new(MIDDLE.x, -50000), Vec2::new(MIDDLE.x, 50000), 0);
    let set = [ItemRef::unstored(&clear), ItemRef::unstored(&over_the_via)];

    let found = world
      .check_colliding_items(root, &set, &rules(), &options)
      .expect("the second probe runs over the via");

    assert_eq!(found.item, Some(ids.via));
    assert!(
      world
        .check_colliding_items(root, &set[..1], &rules(), &options)
        .is_none()
    );
    assert!(
      world
        .check_colliding_items(root, &[], &rules(), &options)
        .is_none()
    );
  }

  // -----------------------------------------------------------------
  // The retired via hole heuristic
  // -----------------------------------------------------------------

  /// A world holding one through via at the origin, its hole, and a line
  /// that ends on it.
  ///
  /// This is the fixture the 2026-09-08 log asks for: the input KiCad's
  /// geometric heuristic (`pcbnew/router/pns_item.cpp:65`) exists to
  /// answer, a line whose via is the node's via.
  fn via_line_fixture() -> (World, ItemId, ItemId, Line) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    let body =
      ItemBody::Via(Via::new(Vec2::new(0, 0), 3000, 1000, ViaType::Through));
    let mut item = world.make_item(body);
    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item.set_net(NET);
    let stored = world.add_via(root, item);
    let hole = world
      .item(stored)
      .and_then(Item::hole)
      .expect("a via is drilled");

    let mut line = Line::new();
    line.set_width(1000);
    line.set_layer(0);
    line.set_net(NET);
    line.set_shape(LineChain::from_slice(
      &[Vec2::new(-50000, 0), Vec2::new(0, 0)],
      false,
    ));

    (world, stored, hole, line)
  }

  /// Collide a whole line against one stored item through the line aware
  /// entry point, the way the shove and the optimizer will.
  fn collide_line_with(
    world: &World,
    line: &Line,
    obstacle: ItemId,
  ) -> Option<Obstacle> {
    let item = world.item(obstacle).expect("the obstacle is live");
    let seg = line.rule_item(world, u64::MAX);
    let head =
      LineHead::new(ItemRef::unstored(&seg), line, line.via_item(world));

    collide_line_items(
      &world.items,
      ItemRef::stored(obstacle, item),
      &head,
      &rules(),
      &CollisionSearchOptions::default(),
    )
  }

  /// A line whose via **is** the node's via reports nothing, against the
  /// via and against its hole, because both sides are one arena item.
  #[test]
  fn a_linked_line_via_is_pruned_by_identity() {
    let (world, stored, hole, mut line) = via_line_fixture();
    line.link_via(stored, Vec2::new(0, 0));

    assert_eq!(collide_line_with(&world, &line, stored), None);
    assert_eq!(collide_line_with(&world, &line, hole), None);

    // And through the node, which is the path the placer takes.
    let root = world.root();

    assert!(
      world
        .check_colliding_line(
          root,
          &line,
          &rules(),
          &CollisionSearchOptions::default()
        )
        .is_none()
    );
  }

  /// The case the heuristic was written for: a line carrying its **own**
  /// via that duplicates one already in the node, built the way the
  /// placer builds one, from scratch.
  ///
  /// KiCad's `LINE::AppendVia` clones the via it is given and
  /// `VIA::Clone` drills the clone a hole of its own
  /// (`pcbnew/router/pns_via.cpp:278`), so there the two holes are
  /// distinct objects and only the geometric test keeps them apart. Here
  /// a hole is a separate arena item that nothing creates until
  /// [`World::add_via`] stores the via, so an owned via has no hole and
  /// the hole to hole branch is never entered at all. What is left, the
  /// copper pair, is exempt for being on one net, exactly as KiCad's is.
  #[test]
  fn an_owned_line_via_built_from_scratch_has_no_hole_to_prune() {
    let (world, stored, hole, mut line) = via_line_fixture();
    let original = world.item(stored).expect("the stored via is live");
    let mut duplicate = Item::new(u64::MAX, original.body().clone());
    duplicate.set_layers_and_flash_all(original.layers());
    duplicate.set_net(original.net());

    // Geometrically indistinguishable, which is exactly what the retired
    // heuristic tested for.
    assert_eq!(duplicate.body(), original.body());
    assert_eq!(duplicate.net(), original.net());
    assert_eq!(duplicate.layers(), original.layers());

    line.append_via(duplicate);

    let owned = line.via_item(&world).expect("the line ends with a via");

    // The premise: no second hole exists for a heuristic to prune.
    assert_eq!(owned.id(), None);
    assert_eq!(owned.item().hole(), None);

    assert_eq!(collide_line_with(&world, &line, stored), None);
    assert_eq!(collide_line_with(&world, &line, hole), None);

    let root = world.root();

    assert!(
      world
        .check_colliding_line(
          root,
          &line,
          &rules(),
          &CollisionSearchOptions::default()
        )
        .is_none()
    );
  }

  /// The other way to build an owned via, cloning the stored [`Item`]
  /// whole. That copies the handle of the hole rather than the hole, so
  /// the two sides share **one** arena hole and the parent identity test
  /// at `pcbnew/router/pns_item.cpp:75` prunes it. KiCad's clone cannot
  /// reach this state: a copied `HOLE*` would be double freed, which is
  /// why `VIA::Clone` drills a new one and why the heuristic had to exist
  /// there.
  #[test]
  fn an_owned_line_via_cloned_whole_shares_the_stored_hole() {
    let (world, stored, hole, mut line) = via_line_fixture();
    let duplicate = world.item(stored).expect("the stored via is live").clone();

    line.append_via(duplicate);

    let owned = line.via_item(&world).expect("the line ends with a via");

    assert_eq!(owned.id(), None);
    assert_eq!(owned.item().hole(), Some(hole));

    assert_eq!(collide_line_with(&world, &line, stored), None);
    assert_eq!(collide_line_with(&world, &line, hole), None);
  }

  /// The same owned via on another net does collide, so the tests above
  /// are not passing for want of a collision path.
  ///
  /// Both passes fire: the line's own chain ends on the via, and the
  /// via pass reports the head line's via against the stored one. The
  /// second is the one worth naming, because
  /// `line->Via().collideSimple( this, ... )`
  /// (`pcbnew/router/pns_item.cpp:141`) puts the via in the obstacle
  /// position and this item in the head position, so its obstacle names
  /// the two the other way round.
  #[test]
  fn an_owned_line_via_on_another_net_collides_with_the_stored_via() {
    let (world, stored, _, mut line) = via_line_fixture();
    let duplicate = world.item(stored).expect("the stored via is live").clone();

    line.set_net(OTHER_NET);
    line.append_via(duplicate);

    let found = collide_line_with(&world, &line, stored)
      .expect("two vias of different nets on one spot collide");

    assert_eq!(found.item, Some(stored));

    // The accumulating form keeps both, which is where the via pass
    // becomes visible.
    let item = world.item(stored).expect("the stored via is live");
    let seg = line.rule_item(&world, u64::MAX);
    let head =
      LineHead::new(ItemRef::unstored(&seg), &line, line.via_item(&world));
    let mut every = Vec::new();

    collide_line_into(
      &world.items,
      ItemRef::stored(stored, item),
      &head,
      -1,
      &rules(),
      &CollisionSearchOptions::default(),
      &mut every,
    );

    assert!(
      every
        .iter()
        .any(|found| found.item.is_none() && found.head == Some(stored)),
      "the via pass reports the head line's via as the obstacle"
    );
  }

  #[test]
  fn every_line_query_answer_is_the_same_across_two_identical_runs() {
    /// What one run of the line queries produces.
    #[derive(PartialEq, Eq, Debug)]
    struct Answers {
      /// Everything the head line collides with.
      obstacles: Vec<Option<ItemId>>,
      /// The first obstacle the early returning form reports.
      first: Option<Option<ItemId>>,
      /// The nearest obstacle, hull and all.
      nearest: Option<NearestObstacle>,
    }

    fn run() -> Answers {
      let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
      let root = world.root();
      crossing_track(&mut world, root, NEAR_X);
      crossing_track(&mut world, root, FAR_X);

      let item = pad(&mut world, Vec2::new(HEAD_END, 0), 0);
      let centre = Vec2::new(HEAD_END, 0);
      world.add_solid(root, item, Some(Hole::circular(centre, 500)));

      let head = crossing_head();
      let options = CollisionSearchOptions::default();

      Answers {
        obstacles: world
          .query_colliding_line(root, &head, &rules(), &options)
          .into_iter()
          .map(|found| found.item)
          .collect(),
        first: world
          .check_colliding_line(root, &head, &rules(), &options)
          .map(|found| found.item),
        nearest: world.nearest_obstacle(
          root,
          &head,
          &rules(),
          &options,
          CORNERS,
        ),
      }
    }

    assert_eq!(run(), run());
  }

  // -----------------------------------------------------------------
  // The hull simplification, the filter and the cluster
  // -----------------------------------------------------------------

  #[test]
  fn a_45_degree_corner_mode_leaves_a_hull_alone() {
    let (mut world, ids) = fixture();
    let hull = world
      .hull_of(ids.pad_bottom, CLEARANCE, 1000, 0)
      .expect("the pad is live");
    let same = simplified_hull(Rc::clone(&hull), CornerMode::Mitered45);

    assert!(
      Rc::ptr_eq(&hull, &same),
      "no copy is made and none is needed"
    );
  }

  #[test]
  fn a_90_degree_corner_mode_squares_a_hull_off() {
    let (mut world, ids) = fixture();
    let hull = world
      .hull_of(ids.pad_bottom, CLEARANCE, 1000, 0)
      .expect("the pad is live");
    let squared = simplified_hull(Rc::clone(&hull), CornerMode::Mitered90);
    let bbox = hull.bbox(0).expect("an octagon has a bounding box");

    // The four corners, in the order `makeHull` appends them
    // (`pcbnew/router/pns_node.cpp:336`).
    assert_eq!(
      squared.points(),
      [
        Vec2::new(bbox.left() as i32, bbox.top() as i32),
        Vec2::new(bbox.right() as i32, bbox.top() as i32),
        Vec2::new(bbox.right() as i32, bbox.bottom() as i32),
        Vec2::new(bbox.left() as i32, bbox.bottom() as i32),
      ]
    );
    // The deviation this port makes: KiCad leaves the chain open, which
    // gives it no inside at all. See `simplified_hull`.
    assert!(squared.is_closed());
    let centre = bbox.center();
    assert!(
      squared.point_inside(Vec2::new(centre.x as i32, centre.y as i32), 0)
    );
    // The box contains the octagon it came from.
    assert_eq!(squared.bbox(0), Some(bbox));
  }

  #[test]
  fn the_nearest_obstacle_measures_against_the_squared_hull() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    // A diagonal track, so that its hull and the box around that hull
    // are visibly different shapes; a vertical one is close enough to
    // its own box that a head along the x axis enters both at the same
    // point.
    let item = track(
      &mut world,
      Vec2::new(NEAR_X - 50000, -50000),
      Vec2::new(NEAR_X + 50000, 50000),
      0,
      NET,
    );
    let obstacle = world
      .add_segment(root, item, false)
      .expect("the diagonal track is neither degenerate nor redundant");
    let head = crossing_head();
    let options = CollisionSearchOptions::default();

    let mitered = world
      .nearest_obstacle(root, &head, &rules(), &options, CornerMode::Mitered45)
      .expect("the head crosses the track");
    let squared = world
      .nearest_obstacle(root, &head, &rules(), &options, CornerMode::Mitered90)
      .expect("the head crosses the track");

    assert_eq!(mitered.item, Some(obstacle));
    assert_eq!(squared.item, Some(obstacle));
    // A bounding box reaches further along the head than the hull it
    // encloses, so the line enters it sooner and the reported distance
    // is smaller.
    assert!(squared.dist_first < mitered.dist_first);
    // And what comes back is the box, not the hull it was made from.
    let boxed = squared.hull.as_ref().expect("a live obstacle has a hull");
    let hull = mitered.hull.as_ref().expect("a live obstacle has a hull");

    assert_eq!(boxed.point_count(), 4);
    assert!(hull.point_count() > 4);
    assert_eq!(boxed.bbox(0), hull.bbox(0));
  }

  #[test]
  fn a_restricted_search_only_reports_what_the_filter_allows() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let near = crossing_track(&mut world, root, NEAR_X);
    let far = crossing_track(&mut world, root, FAR_X);
    let head = crossing_head();

    let unfiltered = CollisionSearchOptions::default();
    let mut allowed = BTreeSet::new();
    allowed.insert(far);

    let restricted = CollisionSearchOptions {
      restricted_set: Some(&allowed),
      ..CollisionSearchOptions::default()
    };

    let all: Vec<Option<ItemId>> = world
      .query_colliding_line(root, &head, &rules(), &unfiltered)
      .into_iter()
      .map(|found| found.item)
      .collect();
    let some: Vec<Option<ItemId>> = world
      .query_colliding_line(root, &head, &rules(), &restricted)
      .into_iter()
      .map(|found| found.item)
      .collect();

    assert_eq!(all, [Some(near), Some(far)]);
    assert_eq!(some, [Some(far)]);

    // And the nearest obstacle is the nearest **allowed** one, not the
    // nearest one filtered out afterwards.
    let nearest = world
      .nearest_obstacle(root, &head, &rules(), &restricted, CORNERS)
      .expect("the far track is still in the way");

    assert_eq!(nearest.item, Some(far));

    // An empty set excludes everything, which is why the walkaround
    // spells "no restriction" as `None`.
    let empty = BTreeSet::new();
    let nothing = CollisionSearchOptions {
      restricted_set: Some(&empty),
      ..CollisionSearchOptions::default()
    };

    assert!(
      world
        .query_colliding_line(root, &head, &rules(), &nothing)
        .is_empty()
    );
  }

  #[test]
  fn a_cluster_gathers_what_touches_and_stops_there() {
    let (world, ids) = fixture();
    let root = world.root();

    // The via at the middle touches both segments and, through them, the
    // two pads. Every one of them is on `NET`, so a head on another net
    // excludes nothing.
    let whole = world.assemble_cluster(root, ids.via, 0, None, None, &rules());

    assert_eq!(whole[0], ids.via, "the seed comes first");
    assert!(whole.contains(&ids.lower));
    assert!(whole.contains(&ids.pad_bottom));
    // The upper segment and the top pad are on layer 1, so a cluster
    // assembled for a layer 0 head does not take them.
    assert!(!whole.contains(&ids.upper));
    assert!(!whole.contains(&ids.pad_top));

    // Excluding the net the cluster is on leaves only the seed.
    let excluded =
      world.assemble_cluster(root, ids.via, 0, None, NET, &rules());

    assert_eq!(excluded, [ids.via]);

    // A lone item is its own cluster.
    let alone =
      world.assemble_cluster(root, ids.pad_top, 1, None, None, &rules());

    assert_eq!(alone[0], ids.pad_top);

    // A stale handle has no shape and therefore no cluster.
    let mut copy = world;
    let root = copy.root();
    copy.remove(root, ids.via);
    assert!(
      copy
        .assemble_cluster(root, ids.via, 0, None, None, &rules())
        .is_empty()
    );
  }

  #[test]
  fn a_cluster_is_abandoned_when_it_grows_too_far() {
    let (world, ids) = fixture();
    let root = world.root();

    // The seed is a via of diameter 3000, so any neighbour blows the
    // bounding box up by far more than this ratio and the walk stops
    // after the round that found it.
    let tight =
      world.assemble_cluster(root, ids.via, 0, Some(1.0), None, &rules());
    let loose = world.assemble_cluster(root, ids.via, 0, None, None, &rules());

    assert!(tight.len() < loose.len());
    assert_eq!(tight[0], ids.via);
  }

  #[test]
  fn the_clearance_for_a_line_is_the_rule_and_nothing_else() {
    let (world, ids) = fixture();
    let head = head_line(&[Vec2::new(0, 0), Vec2::new(HEAD_END, 0)]);

    assert_eq!(
      world.clearance_for_line(ids.pad_bottom, &head, false, &rules()),
      CLEARANCE
    );

    // The epsilon comes off a positive answer when it is asked for.
    let slack = FixedClearance {
      clearance_epsilon: 10,
      ..FixedClearance::uniform(CLEARANCE)
    };

    assert_eq!(
      world.clearance_for_line(ids.pad_bottom, &head, true, &slack),
      CLEARANCE - 10
    );

    // A stale handle is KiCad's "these two can never collide".
    let mut copy = world;
    let root = copy.root();
    copy.remove(root, ids.pad_bottom);
    assert_eq!(
      copy.clearance_for_line(ids.pad_bottom, &head, false, &rules()),
      -1
    );
  }

  /// `drop_node` releases one branch and leaves its siblings alone, which
  /// is what the line placer's per move scratch branch needs
  /// (`pcbnew/router/pns_line_placer.cpp:1492`).
  #[test]
  fn dropping_one_node_leaves_its_siblings_alone() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let first = world.branch(root);
    let second = world.branch(root);
    let grandchild = world.branch(second);

    world.drop_node(second);

    assert!(world.node(first).is_some());
    assert!(world.node(second).is_none());
    assert!(world.node(grandchild).is_none());
    assert_eq!(
      world.node(root).expect("the root is live").children(),
      &[first]
    );

    // The root is never dropped.
    world.drop_node(root);
    assert!(world.node(root).is_some());
  }

  #[test]
  fn a_via_is_found_again_from_its_position_layers_and_net() {
    let (world, items) = fixture();
    let root = world.root();
    let via = world.item(items.via).expect("the fixture built one");
    let layers = via.layers();
    let net = via.net();

    assert_eq!(
      world.find_via_by_handle(root, MIDDLE, layers, net),
      Some(items.via)
    );

    // A handle that names another position, another net or layers the
    // via does not reach finds nothing.
    assert_eq!(
      world.find_via_by_handle(root, Vec2::new(0, 0), layers, net),
      None
    );
    assert_eq!(
      world.find_via_by_handle(root, MIDDLE, layers, OTHER_NET),
      None
    );
    assert_eq!(
      world.find_via_by_handle(root, MIDDLE, LayerRange::single(9), net),
      None
    );
  }
}
