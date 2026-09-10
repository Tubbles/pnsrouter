// SPDX-License-Identifier: GPL-3.0-or-later

//! Connectivity queries over a node (KiCad's `PNS::TOPOLOGY`).
//!
//! Port of `pcbnew/router/pns_topology.cpp`, restricted to what single
//! line routing needs. Note 02 section 6 lists every entry point and
//! section 6.4 says which of them the basic placer actually reaches.
//!
//! # Why there is no `Topology` type
//!
//! KiCad's `TOPOLOGY` is a one field wrapper over a borrowed `NODE*`
//! (`pcbnew/router/pns_topology.h:142`) with a constructor and an empty
//! destructor (`:54`). It carries no state between calls, so every entry
//! point here is a free function taking the world and the [`NodeId`] that
//! pair would have held. That also lets the two functions which need to
//! branch a scratch node ask for `&mut World` while the rest stay read
//! only, which a single borrowed handle could not express.
//!
//! # Net codes
//!
//! `NearestUnconnectedAnchorPoint` refuses a joint whose net code is zero
//! or below (`pcbnew/router/pns_topology.cpp:123`). A net code is not a
//! property of a [`crate::item::NetId`], it is what the host's
//! [`RuleResolver::net_code`] answers, so every function that needs the
//! test takes the resolver where KiCad reads it off the node.
//!
//! # What is not here
//!
//! - `ShortestConnectionLength`, declared at
//!   `pcbnew/router/pns_topology.h:70` and never defined.
//! - `AssembleCluster` (`:1187`), which is [`World::assemble_cluster`]:
//!   it lives on the world because the walkaround calls it on every
//!   iteration and it is a spatial flood fill rather than a connectivity
//!   query.

use std::collections::{BTreeSet, VecDeque};

use crate::collide::CollisionSearchOptions;
use crate::diff_pair::{DiffPair, common_parallel_projection};
use crate::geometry::collision;
use crate::geometry::line_chain::LineChain;
use crate::geometry::shape::Shape;
use crate::geometry::vec2::Vec2;
use crate::item::{Item, ItemBody, ItemId, Kind, LayerRange};
use crate::line::Line;
use crate::node::{JointRef, NodeId, World};
use crate::rules::{ItemRef, RuleResolver};

/// How many depth first states [`follow_branch`] may expand.
///
/// KiCad bounds the same search with a wall clock timeout read from
/// `ADVANCED_CFG::m_FollowBranchTimeout`
/// (`pcbnew/router/pns_topology.cpp:237`, tested at `:270`). `DESIGN.md`
/// section 8 forbids reading the clock inside an algorithm, so the budget
/// is an iteration count instead.
///
/// A backstop is genuinely needed: the search enumerates every simple
/// path out of the start joint, because the shared `visited` set is only
/// ever read inside the loop and never added to, and the per path
/// `visited_joints` set only stops a path from folding back on itself. On
/// a dense mesh that is exponential.
const FOLLOW_BRANCH_STATE_BUDGET: usize = 10_000;

// ---------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------

/// The item and the anchor [`nearest_unconnected_item`] settled on.
///
/// KiCad returns the item and writes the anchor index through an
/// `int* aAnchor` out parameter (`pcbnew/router/pns_topology.h:60`),
/// which every caller passes; the two travel together here.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct NearestUnconnected {
  /// The item the rat line should point at.
  pub item: ItemId,
  /// Which of that item's anchors is the closest one.
  pub anchor: usize,
}

/// Where the leading rat line ends.
///
/// The three out parameters of `NearestUnconnectedAnchorPoint`
/// (`pcbnew/router/pns_topology.cpp:107`) as one value.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct AnchorPoint {
  /// The point to draw to.
  pub point: Vec2,
  /// The layers the thing at that point spans. The rat line itself does
  /// not care about layers (`:171`); the router's finish command does.
  pub layers: LayerRange,
  /// What sits at that point.
  pub item: ItemId,
}

/// One entry of a trivial path.
///
/// KiCad collects the path into an `ITEM_SET`, which mixes borrowed
/// stored items with the `LINE` copies its `Add( const LINE& )` overload
/// takes ownership of (`pcbnew/router/pns_itemset.h:125`). Lines are
/// values in this crate and are never stored in a node, so the two cases
/// are an enum rather than a type erased pointer. The line is boxed
/// because a [`Line`] is two orders of magnitude larger than an
/// [`ItemId`] and the enum would otherwise cost that much per via.
#[derive(Clone, Debug)]
pub enum PathItem {
  /// A run of segments assembled into one line.
  Line(Box<Line>),
  /// A via joining two runs.
  Via(ItemId),
}

/// What [`assemble_trivial_path`] walked.
#[derive(Clone, Debug)]
pub struct TrivialPath {
  /// The path from one terminal to the other, in order.
  pub items: Vec<PathItem>,
  /// The joints the two ends stopped at, KiCad's `aTerminalJoints`
  /// (`pcbnew/router/pns_topology.h:80`). `None` when the path is empty.
  pub terminals: Option<(JointRef, JointRef)>,
}

/// The best branch [`follow_branch`] found out of one joint.
///
/// Port of `TOPOLOGY::PATH_RESULT`, `pcbnew/router/pns_topology.h:108`.
struct PathResult {
  /// The items along that branch.
  items: Vec<PathItem>,
  /// The joint the branch ended on.
  end: JointRef,
  /// How long the branch is, in nanometres. KiCad keeps this in an `int`
  /// while adding `SHAPE_LINE_CHAIN::Length()`, which is an `int64_t`.
  length: i64,
}

/// One frame of [`follow_branch`]'s explicit stack.
///
/// Port of the local `STATE` struct,
/// `pcbnew/router/pns_topology.cpp:242`, minus its `ITEM* via` member:
/// that field is written at `:332` and read nowhere, the via that matters
/// having already been pushed onto `pathItems` at `:336`.
struct BranchState {
  /// Where this frame stands.
  joint: JointRef,
  /// The item the walk arrived on, which it must not walk back along.
  prev: Option<ItemId>,
  /// What the path has collected so far.
  path_items: Vec<PathItem>,
  /// How long that is.
  path_length: i64,
  /// The joints this path has already stood on, so it cannot loop.
  visited_joints: BTreeSet<JointRef>,
}

// ---------------------------------------------------------------------
// Joint lookup helpers
// ---------------------------------------------------------------------

/// The joint at a point, on an item's first layer and net.
///
/// Port of the `FindJoint( const VECTOR2I&, const ITEM* )` overload,
/// `pcbnew/router/pns_node.h:478`, which is the three argument form fed
/// with `aItem->Layers().Start()` and `aItem->Net()`.
fn find_joint_of_item(
  world: &World,
  node: NodeId,
  pos: Vec2,
  item: &Item,
) -> Option<JointRef> {
  world.find_joint(node, pos, item.layers().start(), item.net())
}

/// The joint at a point, on a line's first layer and net.
///
/// The same `pcbnew/router/pns_node.h:478` overload, reached with a
/// `LINE*` (`pcbnew/router/pns_topology.cpp:316`, `:384`, `:385`). A
/// [`Line`] is not an [`Item`] in this crate, so it needs its own
/// spelling.
fn find_joint_of_line(
  world: &World,
  node: NodeId,
  pos: Vec2,
  line: &Line,
) -> Option<JointRef> {
  world.find_joint(node, pos, line.layers().start(), line.net())
}

// ---------------------------------------------------------------------
// SimplifyLine
// ---------------------------------------------------------------------

/// Drop the redundant vertices of a stored track, in place in the node.
///
/// Port of `TOPOLOGY::SimplifyLine`,
/// `pcbnew/router/pns_topology.cpp:49`. `line` is only ever read for its
/// first link: the geometry that is simplified is whatever
/// [`World::assemble_line`] finds from that link, which may be longer
/// than `line` itself if the track continues past it. The answer is
/// whether anything was rewritten.
///
/// Its only callers in KiCad are the two lines of
/// `DIFF_PAIR_PLACER::FixRoute` that tidy the two lanes after they have
/// been committed (`pcbnew/router/pns_diff_pair_placer.cpp:853`, `:854`),
/// which is why it arrives with milestone 10 and not earlier.
///
/// # Simplify, not Simplify2
///
/// `SHAPE_LINE_CHAIN::Simplify()` with its default tolerance of zero
/// ([`LineChain::simplify`]), not [`LineChain::simplify2`]. The two
/// differ on what counts as collinear and on whether a closed chain may
/// lose its first vertex; see [`LineChain::simplify`].
///
/// # The node is rewritten by remove and add
///
/// The assembled track leaves the node and a copy with the simplified
/// shape goes back in, so every segment of the track is a new item with a
/// new [`crate::item::ItemId`] afterwards and any handle a caller was
/// holding is stale. KiCad has the same property; it is only less visible
/// there because the items are pointers into a pool.
///
/// `NODE::Remove( LINE& )` clears the line's links before
/// `LINE lnew( l )` copies it (`:62`, `:63`), so the copy that is added
/// back is unlinked, which is what [`World::add_line`] requires.
pub fn simplify_line(world: &mut World, node: NodeId, line: &Line) -> bool {
  // :51
  if !line.is_linked() || line.segment_count() == 0 {
    return false;
  }

  // :54. KiCad's `GetLink( 0 )` indexes the vector directly; the line is
  // linked, so this cannot in fact answer `None`.
  let Some(root) = line.link_at(0) else {
    return false;
  };

  // :55, :56
  let mut assembled =
    world.assemble_line(node, root, None, false, false, false);
  let mut simplified = assembled.shape().clone();

  // :58
  simplified.simplify(0);

  // :60
  if simplified.point_count() == assembled.point_count() {
    return false;
  }

  // :62
  world.remove_line(node, &mut assembled);

  // :63, :64
  let mut replacement = assembled.clone();

  replacement.set_shape(simplified);

  // :65
  world.add_line(node, &mut replacement, false);

  // :66
  true
}

// ---------------------------------------------------------------------
// ConnectedJoints
// ---------------------------------------------------------------------

/// Every joint reachable from one joint through tracks.
///
/// Port of `TOPOLOGY::ConnectedJoints`,
/// `pcbnew/router/pns_topology.cpp:73`: a breadth first walk that follows
/// [`Kind::SEGMENT`] and [`Kind::ARC`] links only, so neither a pad nor a
/// via is ever an edge of the walk and two runs that merely share a net
/// stay apart.
///
/// A run that changes layer is still walked whole, because a through via
/// merges the joints of the layers it spans into one and the far layer's
/// segments hang off that same joint.
///
/// # Identity
///
/// KiCad mixes two notions of "the same joint" in six lines. It picks the
/// far end of a segment with `*a == *current`, which is
/// `JOINT::operator==` and compares position and net only
/// (`pcbnew/router/pns_joint.h:325`), and it deduplicates with a
/// `std::set<const JOINT*>`, which compares addresses. Note 02 section
/// 10.5 records why an address is not an identity here at all: a joint is
/// erased and reinserted on every merge, and the same logical joint has
/// one address in a branch and another in the root.
///
/// Both halves are reproduced honestly. The far end test is
/// [`crate::joint::Joint::same_key_as`], the port of that `operator==`,
/// and membership is a set of [`JointRef`], which names the map a joint
/// lives in as well as its arena slot and stays valid across the walk.
///
/// The result is in [`JointRef`] order, where KiCad's is in address
/// order; nothing downstream may depend on either, and this one at least
/// repeats.
pub fn connected_joints(
  world: &World,
  node: NodeId,
  start: JointRef,
) -> Vec<JointRef> {
  let mut queue: VecDeque<JointRef> = VecDeque::new();
  let mut processed: BTreeSet<JointRef> = BTreeSet::new();

  // :78, :79
  queue.push_back(start);
  processed.insert(start);

  // :81
  while let Some(current) = queue.pop_front() {
    let Some(current_joint) = world.joint(current) else {
      continue;
    };

    // :86
    for id in current_joint.links() {
      let Some(item) = world.item(*id) else {
        continue;
      };

      // :88
      if !item.of_kind(Kind::SEGMENT | Kind::ARC) {
        continue;
      }

      // :90 to :92. KiCad dereferences `a` unchecked. When this node
      // has no joint at that anchor, the test below is false, so `next`
      // becomes `a` itself and the link is skipped.
      let a = find_joint_of_item(world, node, item.anchor(0), item);
      let b = find_joint_of_item(world, node, item.anchor(1), item);

      let a_is_current = a
        .and_then(|joint| world.joint(joint))
        .is_some_and(|joint| joint.same_key_as(current_joint));

      let Some(next) = (if a_is_current { b } else { a }) else {
        continue;
      };

      // :94 to :98
      if processed.insert(next) {
        queue.push_back(next);
      }
    }
  }

  processed.into_iter().collect()
}

// ---------------------------------------------------------------------
// NearestUnconnectedItem
// ---------------------------------------------------------------------

/// The closest thing on the joint's net that the joint cannot reach.
///
/// Port of `TOPOLOGY::NearestUnconnectedItem`,
/// `pcbnew/router/pns_topology.cpp:185`: take every routable item of the
/// joint's net, drop everything linked to a joint [`connected_joints`]
/// reaches, and keep the anchor closest to the start joint by Euclidean
/// distance.
///
/// `kind_mask` is KiCad's `aKindMask`, defaulted to `ITEM::ANY_T` at
/// `pcbnew/router/pns_topology.h:60`; both callers use the default.
///
/// # Order
///
/// KiCad's candidate set is a `std::set<ITEM*>` and is therefore scanned
/// in address order, so which of several equidistant anchors wins is
/// whatever the allocator decided. [`World::all_items_in_net`] answers in
/// uid order and the comparison stays KiCad's strict `<`, so the winner
/// of a tie is the item with the smallest uid and its lowest numbered
/// anchor.
pub fn nearest_unconnected_item(
  world: &World,
  node: NodeId,
  start: JointRef,
  kind_mask: Kind,
) -> Option<NearestUnconnected> {
  let start_joint = world.joint(start)?;
  let start_pos = start_joint.pos();

  // :189
  let mut disconnected =
    world.all_items_in_net(node, start_joint.net(), Kind::ANY);

  // :191 to :198
  let mut connected: BTreeSet<ItemId> = BTreeSet::new();

  for reachable in connected_joints(world, node, start) {
    if let Some(joint) = world.joint(reachable) {
      connected.extend(joint.links().iter().copied());
    }
  }

  disconnected.retain(|id| !connected.contains(id));

  // :200 to :222
  let mut best_distance = i32::MAX;
  let mut best: Option<NearestUnconnected> = None;

  for id in disconnected {
    let Some(item) = world.item(id) else {
      continue;
    };

    if !item.of_kind(kind_mask) {
      continue;
    }

    for anchor in 0..item.anchor_count() {
      let distance = (item.anchor(anchor) - start_pos).euclidean_norm();

      if distance < best_distance {
        best_distance = distance;
        best = Some(NearestUnconnected { item: id, anchor });
      }
    }
  }

  best
}

// ---------------------------------------------------------------------
// NearestUnconnectedAnchorPoint and LeadingRatLine
// ---------------------------------------------------------------------

/// Where a track being routed still has to get to.
///
/// Port of `TOPOLOGY::NearestUnconnectedAnchorPoint`,
/// `pcbnew/router/pns_topology.cpp:107`. It branches a scratch node, adds
/// an unlinked copy of the track to it so that the track's own geometry
/// joins the connectivity graph, and looks at the joint under the track's
/// last point:
///
/// - when something other than the track is already linked there, that is
///   the answer and the point is the joint itself (`:142`);
/// - otherwise [`nearest_unconnected_item`] answers from the scratch
///   node, so everything the track already reaches is excluded (`:150`).
///
/// The scratch node is dropped before returning, which is KiCad's
/// `std::unique_ptr<NODE> tmpNode` going out of scope at `:165`. That is
/// why the first branch skips a link belonging to the scratch node
/// (`:134`, "tmpNode's own track is freed on return, skip it to avoid a
/// dangling anchor item") and why the second branch is checked the same
/// way here: an [`ItemId`] minted in the scratch node fails its
/// generation check afterwards, and a caller cannot tell that apart from
/// a plain removal. The fallback cannot in fact produce one, because
/// every item the scratch node holds is a segment of the track and is
/// therefore reachable from the start joint, but the guard costs one
/// lookup and removes the question.
///
/// `None` for an empty track, for a track whose end has no joint, for a
/// net code of zero or below (`:123`) and when nothing unconnected is
/// left.
pub fn nearest_unconnected_anchor_point(
  world: &mut World,
  node: NodeId,
  resolver: &dyn RuleResolver,
  track: &Line,
) -> Option<AnchorPoint> {
  // :113
  if track.point_count() == 0 {
    return None;
  }

  // :116 to :119
  let scratch = world.branch(node);
  let mut copy = track.clone();

  copy.clear_links();
  world.add_line(scratch, &mut copy, false);

  let found = anchor_point_in(world, scratch, resolver, &copy);

  world.drop_node(scratch);

  found
}

/// The body of [`nearest_unconnected_anchor_point`], with the scratch
/// node already built and the track already in it.
///
/// Split out so that the scratch node is dropped on every path, which is
/// what KiCad's `unique_ptr` does for it.
fn anchor_point_in(
  world: &World,
  scratch: NodeId,
  resolver: &dyn RuleResolver,
  track: &Line,
) -> Option<AnchorPoint> {
  // :121
  let last = track.last_point()?;
  let reference = find_joint_of_line(world, scratch, last, track)?;
  let joint = world.joint(reference)?;

  // :123. A net code of zero or below means "no net", and a joint with no
  // net at all is the same answer one step earlier.
  let net = joint.net()?;

  if resolver.net_code(net) <= 0 {
    return None;
  }

  // :128. Two links without a via, three with one. Neither KiCad's
  // `NODE::Add( LINE& )` nor [`World::add_line`] stores the line's via,
  // so a track that ends with one ends with a via that is already
  // stored, already linked to this joint and already counted; the
  // threshold rises by one so that it is not mistaken for something the
  // track has arrived at.
  let threshold = if track.ends_with_via() { 3 } else { 2 };
  let mut connected = None;

  if joint.link_count(world.items(), Kind::ANY) >= threshold {
    // :132 to :139
    for id in joint.links() {
      if world.home_of(*id) != Some(scratch) {
        connected = Some(*id);
        break;
      }
    }
  }

  // :142 to :147
  if let Some(item) = connected {
    return Some(AnchorPoint {
      point: joint.pos(),
      layers: joint.layers(),
      item,
    });
  }

  // :150 to :160
  let found = nearest_unconnected_item(world, scratch, reference, Kind::ANY)?;

  if world.home_of(found.item) == Some(scratch) {
    return None;
  }

  let item = world.item(found.item)?;

  Some(AnchorPoint {
    point: item.anchor(found.anchor),
    layers: item.layers(),
    item: found.item,
  })
}

/// The rat line a host draws from the end of a track to its target.
///
/// Port of `TOPOLOGY::LeadingRatLine`,
/// `pcbnew/router/pns_topology.cpp:168`: two points, the track's last
/// point and whatever [`nearest_unconnected_anchor_point`] found.
///
/// # The degenerate answer
///
/// When the track's end already touches something, that function answers
/// with the joint's own position (`:144`), which is the track's last
/// point. Both `SHAPE_LINE_CHAIN::Append` and [`LineChain::append`]
/// swallow a repeat of the last point, so the chain then holds a single
/// point and describes a rat line of zero length. That is KiCad's
/// behaviour, and a host should read it as "nothing left to reach" rather
/// than as a line to draw.
pub fn leading_rat_line(
  world: &mut World,
  node: NodeId,
  resolver: &dyn RuleResolver,
  track: &Line,
) -> Option<LineChain> {
  // :175
  let anchor = nearest_unconnected_anchor_point(world, node, resolver, track)?;
  let last = track.last_point()?;

  // :178 to :180
  let mut rat_line = LineChain::new();

  rat_line.append(last);
  rat_line.append(anchor.point);

  Some(rat_line)
}

// ---------------------------------------------------------------------
// AssembleTrivialPath
// ---------------------------------------------------------------------

/// The empty answer, KiCad's `return ITEM_SET()`
/// (`pcbnew/router/pns_topology.cpp:481`, `:505`).
fn no_trivial_path() -> TrivialPath {
  TrivialPath {
    items: Vec::new(),
    terminals: None,
  }
}

/// Walk a whole run of copper outwards from one item.
///
/// Port of `TOPOLOGY::AssembleTrivialPath`,
/// `pcbnew/router/pns_topology.cpp:461`. The start item is resolved to a
/// seed segment, that segment is assembled into a line, and each end of
/// that line is extended along the longest branch the port of KiCad's
/// `followBranch` (`:228`) can find.
///
/// Starting from a via only works when the via has exactly one track in
/// and one track out (`IsNonFanoutVia`, `:478`); otherwise the answer is
/// empty, because "the trivial path through this via" is not defined.
/// Starting from a pad is not supported at all, which is KiCad's `seg`
/// staying null at `:502`.
///
/// `follow_locked_segments` is KiCad's `aFollowLockedSegments`, passed
/// through to [`World::assemble_line`] at every step.
///
/// The path runs left terminal to right terminal with the seed line in
/// the middle. Note that the left half comes out reversed with respect to
/// the walk that produced it, because KiCad prepends each of its items in
/// turn (`:417`).
pub fn assemble_trivial_path(
  world: &World,
  node: NodeId,
  start: ItemId,
  follow_locked_segments: bool,
) -> TrivialPath {
  let Some(item) = world.item(start) else {
    return no_trivial_path();
  };

  // :472 to :500
  let seed = if item.kind() == Kind::VIA {
    let ItemBody::Via(via) = item.body() else {
      return no_trivial_path();
    };

    // :476. KiCad dereferences the joint unchecked.
    let Some(reference) = find_joint_of_item(world, node, via.pos(), item)
    else {
      return no_trivial_path();
    };
    let Some(joint) = world.joint(reference) else {
      return no_trivial_path();
    };

    // :478
    if !joint.is_non_fanout_via(world.items()) {
      return no_trivial_path();
    }

    // :486 to :494
    joint.links().iter().copied().find(|id| {
      world
        .item(*id)
        .is_some_and(|link| link.of_kind(Kind::SEGMENT | Kind::ARC))
    })
  } else if item.of_kind(Kind::SEGMENT | Kind::ARC) {
    Some(start)
  } else {
    None
  };

  // :502
  let Some(seed) = seed else {
    return no_trivial_path();
  };

  // :510
  let line =
    world.assemble_line(node, seed, None, false, follow_locked_segments, true);

  follow_trivial_path(world, node, &line, follow_locked_segments)
}

/// Extend an assembled line out of both of its ends.
///
/// Port of `TOPOLOGY::followTrivialPath`,
/// `pcbnew/router/pns_topology.cpp:363`. The `visited` set is seeded with
/// the seed line's own links (`:381`) so that neither branch walks back
/// into it, and the seed line's first and last links are handed to the
/// two [`follow_branch`] calls as the item each arrived on (`:389`,
/// `:395`).
///
/// `visited` is a [`BTreeSet`] where KiCad's is a `std::set<ITEM*>`;
/// neither is ever iterated, so the only thing that changes is that the
/// membership test is by handle rather than by address.
fn follow_trivial_path(
  world: &World,
  node: NodeId,
  line: &Line,
  follow_locked_segments: bool,
) -> TrivialPath {
  // :376, :377
  let mut items: Vec<PathItem> = vec![PathItem::Line(Box::new(line.clone()))];

  // :379 to :382
  let visited: BTreeSet<ItemId> = line.links().iter().copied().collect();

  // :384, :385. KiCad dereferences both joints unchecked at `:388`.
  let Some(last) = line.last_point() else {
    return no_trivial_path();
  };
  let Some(joint_a) = find_joint_of_line(world, node, line.point(0), line)
  else {
    return no_trivial_path();
  };
  let Some(joint_b) = find_joint_of_line(world, node, last, line) else {
    return no_trivial_path();
  };

  // :389
  let left = follow_branch(
    world,
    node,
    joint_a,
    line.link_at(0),
    &visited,
    follow_locked_segments,
  );

  // :395
  let right = follow_branch(
    world,
    node,
    joint_b,
    line.link_at(-1),
    &visited,
    follow_locked_segments,
  );

  // :415 to :426. `ITEM_SET::Prepend` in a loop, so the left branch ends
  // up reversed ahead of the seed line.
  for item in left.items {
    items.insert(0, item);
  }

  // :429 to :440
  items.extend(right.items);

  TrivialPath {
    items,
    terminals: Some((left.end, right.end)),
  }
}

/// The longest simple path out of one joint.
///
/// Port of `TOPOLOGY::followBranch`,
/// `pcbnew/router/pns_topology.cpp:228`, an explicit stack depth first
/// search that keeps the longest path it reaches a dead end on (`:344`).
/// Each frame owns a copy of the path collected so far and of the joints
/// that path has stood on, exactly as KiCad's does (`:328`).
///
/// # Deviations
///
/// The wall clock timeout (`:237`, `:270`) becomes
/// [`FOLLOW_BRANCH_STATE_BUDGET`]; see there. The `STATE::via` member is
/// dropped, being written and never read.
///
/// # KiCad behaviour worth knowing
///
/// `aVisited` is read but never added to inside this function, so the
/// same via can be pushed onto the path several times, once per branch
/// that passes through its joint, and once more from the other side's
/// call. The search also keeps the **longest** branch and not the
/// shortest, because its purpose is length tuning.
fn follow_branch(
  world: &World,
  node: NodeId,
  start_joint: JointRef,
  prev: Option<ItemId>,
  visited: &BTreeSet<ItemId>,
  follow_locked_segments: bool,
) -> PathResult {
  // :234, :235
  let mut best = PathResult {
    items: Vec::new(),
    end: start_joint,
    length: 0,
  };

  // :255 to :262
  let mut stack: Vec<BranchState> = vec![BranchState {
    joint: start_joint,
    prev,
    path_items: Vec::new(),
    path_length: 0,
    visited_joints: BTreeSet::from([start_joint]),
  }];
  let mut budget = FOLLOW_BRANCH_STATE_BUDGET;

  // :264
  loop {
    // :266 to :276
    if budget == 0 {
      break;
    }

    let Some(current) = stack.pop() else {
      break;
    };

    budget -= 1;

    let Some(joint) = world.joint(current.joint) else {
      continue;
    };
    let joint_pos = joint.pos();

    // :285 to :294
    let via = joint.links().iter().copied().find(|id| {
      !visited.contains(id)
        && world.item(*id).is_some_and(|item| item.of_kind(Kind::VIA))
    });

    // :297 to :341
    let mut found_branch = false;

    for id in joint.links() {
      let Some(item) = world.item(*id) else {
        continue;
      };

      // :301 to :308
      if !item.of_kind(Kind::SEGMENT | Kind::ARC) {
        continue;
      }

      if Some(*id) == current.prev || visited.contains(id) {
        continue;
      }

      // :310
      let mut line = world.assemble_line(
        node,
        *id,
        None,
        false,
        follow_locked_segments,
        true,
      );

      // :313
      if line.point_count() == 0 {
        continue;
      }

      if line.point(0) != joint_pos {
        line.reverse();
      }

      // :316
      let Some(line_last) = line.last_point() else {
        continue;
      };
      let Some(next_joint) = find_joint_of_line(world, node, line_last, &line)
      else {
        continue;
      };

      // :319
      if current.visited_joints.contains(&next_joint) {
        continue;
      }

      // :322
      found_branch = true;

      // :325 to :340
      let mut visited_joints = current.visited_joints.clone();

      visited_joints.insert(next_joint);

      let mut path_items = current.path_items.clone();

      if let Some(via) = via {
        path_items.push(PathItem::Via(via));
      }

      let path_length = current.path_length + line.shape().length();
      let next_prev = line.link_at(-1);

      path_items.push(PathItem::Line(Box::new(line)));

      stack.push(BranchState {
        joint: next_joint,
        prev: next_prev,
        path_items,
        path_length,
        visited_joints,
      });
    }

    // :344 to :352
    if !found_branch && current.path_length > best.length {
      best.length = current.path_length;
      best.end = current.joint;
      best.items = current.path_items;
    }
  }

  best
}

// ---------------------------------------------------------------------
// AssembleTuningPath
// ---------------------------------------------------------------------

/// How many walk states [`walk_tuning_path`] may expand.
///
/// KiCad bounds the same search with the wall clock timeout
/// [`FOLLOW_BRANCH_STATE_BUDGET`] also replaces
/// (`pcbnew/router/pns_topology.cpp:617`, tested at `:640`), and for the
/// same reason: the walk enumerates every simple path out of the seed
/// line, so a dense mesh is exponential. `DESIGN.md` section 8 forbids
/// reading the clock inside an algorithm, so the budget is a state count.
const WALK_TUNING_PATH_STATE_BUDGET: usize = 10_000;

/// What one [`walk_tuning_path`] call found.
///
/// Port of `TOPOLOGY::WALK_RESULT`, `pcbnew/router/pns_topology.h:117`.
/// Its constructor starts the length at **minus one** (`:125`), not at
/// zero, so a branch that walked nowhere still replaces the initial value
/// and clears the pad. `WalkResult::nothing` reproduces that.
#[derive(Clone, Debug)]
pub struct WalkResult {
  /// The lines and vias along the branch, in walk order. `m_items`
  /// (`:119`).
  pub items: Vec<PathItem>,
  /// The pad the branch ended on. `m_endPad` (`:120`).
  pub end_pad: Option<ItemId>,
  /// How long the branch is, in nanometres. `m_length` (`:121`).
  pub length: i64,
}

impl WalkResult {
  /// The value KiCad's constructor installs, the negative length
  /// included.
  const fn nothing() -> Self {
    Self {
      items: Vec::new(),
      end_pad: None,
      // :125
      length: -1,
    }
  }
}

/// One frame of [`walk_tuning_path`]'s explicit stack.
///
/// Port of the local `STATE` struct,
/// `pcbnew/router/pns_topology.cpp:620`.
struct WalkState {
  /// The point the walk stands on.
  endpoint: Vec2,
  /// What the path has collected so far.
  path_items: Vec<PathItem>,
  /// How long that is.
  path_length: i64,
  /// Everything this path has already consumed, so it cannot fold back.
  visited: BTreeSet<ItemId>,
}

/// What [`assemble_tuning_path`] walked.
///
/// KiCad returns the `ITEM_SET` and writes the two terminal pads through
/// the `SOLID** aStartPad` and `SOLID** aEndPad` out parameters
/// (`pcbnew/router/pns_topology.h:78`), which its one caller passes
/// (`pcbnew/router/pns_meander_placer.cpp:85`); the three travel together
/// here.
#[derive(Clone, Debug, Default)]
pub struct TuningPath {
  /// The path from one terminal to the other, in order, with the seed
  /// line in the middle.
  pub items: Vec<PathItem>,
  /// The pad the leftward walk ended on, KiCad's `*aStartPad`.
  pub start_pad: Option<ItemId>,
  /// The pad the rightward walk ended on, KiCad's `*aEndPad`.
  pub end_pad: Option<ItemId>,
}

/// The total copper length of a path, in nanometres.
///
/// The replacement for `MEANDER_PLACER_BASE::lineLength`
/// (`pcbnew/router/pns_meander_placer_base.cpp:307`), which forwards to
/// the host hook `ROUTER_IFACE::CalculateRoutedPathLength`
/// (`pcbnew/router/pns_router.h:124`). This crate has no such interface,
/// and note 08 section 11.2 item 3 asks for a free function over the item
/// set instead: the sum of every line's chain length, and nothing for a
/// via. KiCad's own `if( aItems.Empty() ) return 0` (`:309`) is the empty
/// sum.
///
/// Four terms of KiCad's number are deliberately absent, and each of them
/// is genuinely zero here rather than approximated:
///
/// - **pad to die length.** `Start` adds `SOLID::GetPadToDie` for each
///   terminal pad (`pcbnew/router/pns_meander_placer.cpp:92`, `:98`).
///   Nothing on this crate's [`Item`] carries one, so the term is zero.
/// - **via height.** KiCad's length calculator adds the stackup distance
///   a via travels. A via is a point here.
/// - **the pad entry and via entry fixups.** `AssembleTuningPath` runs
///   `OptimiseTraceInPad` and `OptimiseTraceInVia` over the path's chains
///   in place (`pcbnew/router/pns_topology.cpp:928`, `:1010`), which is
///   why the host is then told not to repeat them
///   (`pcbnew/router/pns_kicad_iface.cpp:3183`). Both live in KiCad's
///   board code and not in the router; see [`assemble_tuning_path`].
/// - **propagation delay**, the whole `lineDelay` half
///   (`pcbnew/router/pns_meander_placer_base.cpp:317`), which needs a
///   stackup model.
///
/// A host that wants any of them folds its own term into the target it
/// asks for. `doc/log/2026-09-10.md` records the decision.
#[must_use]
pub fn path_length(items: &[PathItem]) -> i64 {
  items
    .iter()
    .map(|item| match item {
      PathItem::Line(line) => line.shape().length(),
      // A via is a point, so it contributes nothing.
      PathItem::Via(_) => 0,
    })
    .sum()
}

/// Every line that leaves a via, for a walk to continue along.
///
/// Port of `TOPOLOGY::findLinesFromVia`,
/// `pcbnew/router/pns_topology.cpp:536`. It is a spatial query rather
/// than a joint lookup because a track may end **inside** a via's pad
/// without reaching its centre, in which case there is no joint to walk
/// through; that is the case `OptimiseTraceInVia` exists to measure.
///
/// The anchor test at `:571` has two branches in KiCad, one that asks the
/// board's `LENGTH_DELAY_CALCULATION::IsPointInsideViaPad` (`:583`) and a
/// fallback that tests the router's own via shape (`:589`). Only the
/// fallback is ported: the first needs a `PCB_VIA` and a board layer,
/// neither of which exists here.
///
/// `visited` keeps the walk from turning round, and the local `assembled`
/// set keeps two links of the same run from producing the same line twice
/// (`:567`, `:600`).
fn find_lines_from_via(
  world: &World,
  node: NodeId,
  resolver: &dyn RuleResolver,
  via: ItemId,
  visited: &BTreeSet<ItemId>,
) -> Vec<Line> {
  let mut result = Vec::new();

  let Some(via_item) = world.item(via) else {
    return result;
  };
  let net = via_item.net();

  // :540 to :546
  let options = CollisionSearchOptions {
    different_nets_only: false,
    override_clearance: Some(0),
    kind_mask: Kind::SEGMENT | Kind::ARC,
    ..CollisionSearchOptions::default()
  };
  let obstacles = world.query_colliding(
    node,
    ItemRef::stored(via, via_item),
    resolver,
    &options,
  );

  // :588. The shape the fallback anchor test runs against.
  let via_shape = via_item.shape(via_item.layers().start());
  let mut assembled: BTreeSet<ItemId> = BTreeSet::new();

  // :557
  for obstacle in obstacles {
    let Some(candidate) = obstacle.item else {
      continue;
    };
    let Some(item) = world.item(candidate) else {
      continue;
    };

    // :559
    if item.net() != net {
      continue;
    }

    // :564, :567
    if visited.contains(&candidate) || assembled.contains(&candidate) {
      continue;
    }

    // :571 to :596. At least one anchor has to sit inside the via's pad.
    let Some(shape) = via_shape.as_ref() else {
      continue;
    };
    let inside = (0..item.anchor_count()).any(|index| {
      collision::collides(
        shape.as_ref(),
        &Shape::circle(item.anchor(index), 0),
        0,
      )
    });

    if !inside {
      continue;
    }

    // :598
    let line = world.assemble_line(node, candidate, None, false, true, true);

    // :600 to :603
    assembled.extend(line.links().iter().copied());
    result.push(line);
  }

  result
}

/// One continuation of a walk, as a fresh stack frame.
///
/// The body shared by `pcbnew/router/pns_topology.cpp:686` to `:703` and
/// `:729` to `:752`: pick the far end of the continuation, extend the
/// path with it, add its length, and mark every one of its links
/// consumed. `via` is the via to push ahead of the line, which only the
/// via branch has (`:736`).
fn walk_forward(
  current: &WalkState,
  line: Line,
  via: Option<ItemId>,
) -> Option<WalkState> {
  let first = (line.point_count() > 0).then(|| line.point(0))?;
  let last = line.last_point()?;
  let endpoint = current.endpoint;

  // :690, :731. The nearer end is where the walk came in, so the far end
  // is where it goes next. A tie sends the walk to the last point.
  let start_near = (first - endpoint).squared_euclidean_norm()
    <= (last - endpoint).squared_euclidean_norm();
  let mut visited = current.visited.clone();

  visited.extend(line.links().iter().copied());

  let mut path_items = current.path_items.clone();

  if let Some(via) = via {
    path_items.push(PathItem::Via(via));
  }

  // :745. `OptimiseTraceInVia` is not ported, so a continuation measures
  // as it is drawn; see [`path_length`].
  let path_length = current.path_length + line.shape().length();

  path_items.push(PathItem::Line(Box::new(line)));

  Some(WalkState {
    endpoint: if start_near { last } else { first },
    path_items,
    path_length,
    visited,
  })
}

/// Walk out of one end of a line, through pads and vias, keeping the
/// longest branch.
///
/// Port of `TOPOLOGY::walkTuningPath`,
/// `pcbnew/router/pns_topology.cpp:611`. It is the length tuning
/// counterpart of `follow_branch` and differs from it in three ways,
/// all of which follow from what a tuned length is:
///
/// - it walks **through** a pad instead of stopping at one (`:676`), so a
///   signal that passes through a series component is tuned end to end,
///   and it records the pad it passed as the branch's terminal (`:669`);
/// - it walks through a via by way of `find_lines_from_via` rather than
///   through the via's joint, so a track that stops inside a via's pad is
///   still followed (`:722`);
/// - the visited set is **per path** and grows as the walk goes (`:676`,
///   `:701`), where `followBranch`'s is shared and never grows, so this
///   walk cannot pass the same copper twice.
///
/// The wall clock timeout at `:633` becomes
/// `WALK_TUNING_PATH_STATE_BUDGET`.
pub fn walk_tuning_path(
  world: &World,
  node: NodeId,
  resolver: &dyn RuleResolver,
  start_line: &Line,
  start_from_back: bool,
  visited: &BTreeSet<ItemId>,
) -> WalkResult {
  let mut best = WalkResult::nothing();
  let net = start_line.net();

  // :629
  let Some(endpoint) = (if start_from_back {
    start_line.last_point()
  } else {
    (start_line.point_count() > 0).then(|| start_line.point(0))
  }) else {
    return best;
  };

  // :630 to :634
  let mut stack = vec![WalkState {
    endpoint,
    path_items: Vec::new(),
    path_length: 0,
    visited: visited.clone(),
  }];
  let mut budget = WALK_TUNING_PATH_STATE_BUDGET;

  // :636
  while let Some(mut current) = stack.pop() {
    // :638 to :646
    if budget == 0 {
      break;
    }

    budget -= 1;

    // :651
    let hits = world.hit_test(node, current.endpoint);

    // :655 to :663
    let pad = hits.iter().copied().find(|id| {
      world.item(*id).is_some_and(|item| {
        item.of_kind(Kind::SOLID)
          && item.net() == net
          && !current.visited.contains(id)
      })
    });

    if let Some(pad) = pad {
      // :667 to :673
      if current.path_length > best.length {
        best.length = current.path_length;
        best.items.clone_from(&current.path_items);
        best.end_pad = Some(pad);
      }

      // :676, "continue through an in-line pad so tuning spans the whole
      // net".
      current.visited.insert(pad);

      // :678 to :703
      for id in &hits {
        let Some(item) = world.item(*id) else {
          continue;
        };

        // :680 to :684
        if !item.of_kind(Kind::SEGMENT | Kind::ARC) {
          continue;
        }

        if item.net() != net || current.visited.contains(id) {
          continue;
        }

        // :686
        let line = world.assemble_line(node, *id, None, false, true, true);

        if let Some(next) = walk_forward(&current, line, None) {
          stack.push(next);
        }
      }

      continue;
    }

    // :708 to :716
    let via = hits.iter().copied().find(|id| {
      world.item(*id).is_some_and(|item| {
        item.of_kind(Kind::VIA)
          && item.net() == net
          && !item.is_virtual()
          && !current.visited.contains(id)
      })
    });

    let Some(via) = via else {
      // :764 to :770
      if current.path_length > best.length {
        best.length = current.path_length;
        best.items = current.path_items;
        best.end_pad = None;
      }

      continue;
    };

    // :720
    current.visited.insert(via);

    // :722
    let continuations =
      find_lines_from_via(world, node, resolver, via, &current.visited);

    // :755 to :761
    if continuations.is_empty() {
      if current.path_length > best.length {
        best.length = current.path_length;
        best.items = current.path_items;
        best.items.push(PathItem::Via(via));
        best.end_pad = None;
      }

      continue;
    }

    // :724 to :753
    for line in continuations {
      if let Some(next) = walk_forward(&current, line, Some(via)) {
        stack.push(next);
      }
    }
  }

  best
}

/// The longest run of copper the tuner may lengthen, out of one item.
///
/// Port of `TOPOLOGY::AssembleTuningPath`,
/// `pcbnew/router/pns_topology.cpp:787`, the path
/// `MEANDER_PLACER::Start` measures the tuned length over
/// (`pcbnew/router/pns_meander_placer.cpp:85`).
///
/// It differs from [`assemble_trivial_path`] in exactly the two ways
/// length tuning needs: [`walk_tuning_path`] walks **through** a pad
/// rather than stopping at one, and each end takes the longest branch a
/// junction offers rather than refusing to guess.
///
/// A via start is resolved the same way twice over: through the via's
/// joint when it is a non fanout via (`:800`), and through
/// `find_lines_from_via` otherwise (`:819`). Anything that is neither a
/// via nor a segment answers an empty path, which is KiCad's `seg`
/// staying null at `:842`.
///
/// # The two in place fixups are not ported
///
/// KiCad finishes by rewriting the path's own chains: `OptimiseTraceInPad`
/// on every line touching a terminal or an intermediate pad (`:928`,
/// `:974`) and `OptimiseTraceInVia` on the lines either side of every via
/// (`:1010`). Both are `LENGTH_DELAY_CALCULATION` members that live in
/// KiCad's board code, outside the router, and both need a `PAD` or a
/// `PCB_VIA` and a board layer. Note 08 section 11.2 item 1 offers the
/// choice of leaving the chains alone or exposing the fixup as a host
/// hook; the chains are left alone, so a track that overshoots into a pad
/// measures as it is drawn. See [`path_length`], where the same decision
/// is recorded from the length side.
pub fn assemble_tuning_path(
  world: &World,
  node: NodeId,
  resolver: &dyn RuleResolver,
  start: ItemId,
) -> TuningPath {
  let Some(item) = world.item(start) else {
    return TuningPath::default();
  };

  // :795 to :840
  let seed = if item.kind() == Kind::VIA {
    let ItemBody::Via(via) = item.body() else {
      return TuningPath::default();
    };

    // :800 to :814
    let from_joint = find_joint_of_item(world, node, via.pos(), item)
      .and_then(|reference| world.joint(reference))
      .filter(|joint| joint.is_non_fanout_via(world.items()))
      .and_then(|joint| {
        joint.links().iter().copied().find(|id| {
          world
            .item(*id)
            .is_some_and(|link| link.of_kind(Kind::SEGMENT | Kind::ARC))
        })
      });

    // :816 to :835
    match from_joint {
      Some(seed) => Some(seed),
      None => {
        find_lines_from_via(world, node, resolver, start, &BTreeSet::new())
          .first()
          .and_then(|line| {
            line.links().iter().copied().find(|id| {
              world
                .item(*id)
                .is_some_and(|link| link.of_kind(Kind::SEGMENT | Kind::ARC))
            })
          })
      }
    }
  } else if item.of_kind(Kind::SEGMENT | Kind::ARC) {
    // :838
    Some(start)
  } else {
    None
  };

  // :842
  let Some(seed) = seed else {
    return TuningPath::default();
  };

  // :848
  let line = world.assemble_line(node, seed, None, false, true, true);

  // :853 to :856
  let visited: BTreeSet<ItemId> = line.links().iter().copied().collect();

  // :860, :865
  let left = walk_tuning_path(world, node, resolver, &line, false, &visited);
  let right = walk_tuning_path(world, node, resolver, &line, true, &visited);

  // :867 to :876. `ITEM_SET::Prepend` in a loop, so the left branch ends
  // up reversed ahead of the seed line, exactly as in
  // [`follow_trivial_path`].
  let mut items: Vec<PathItem> = Vec::new();

  for item in left.items {
    items.insert(0, item);
  }

  items.push(PathItem::Line(Box::new(line)));
  items.extend(right.items);

  // :881 to :908. KiCad also checks that the solid's parent really is a
  // `PCB_PAD_T` before naming it a terminal, because a `SOLID` can stand
  // for any board object with copper; every solid here is one the host
  // named, so there is nothing to check.
  TuningPath {
    items,
    start_pad: left.end_pad,
    end_pad: right.end_pad,
  }
}

// ---------------------------------------------------------------------
// AssembleDiffPair
// ---------------------------------------------------------------------

/// How far two segments may tilt against each other and still be paired.
///
/// `DP_PARALLELITY_THRESHOLD` (`pcbnew/router/pns_topology.h:106`), read
/// at `pcbnew/router/pns_topology.cpp:1088`. KiCad asks the same question
/// with three different thresholds; the other two are named on
/// `COUPLED_SEGMENTS_PARALLELISM_THRESHOLD` in [`crate::diff_pair`].
const DP_PARALLELITY_THRESHOLD: i32 = 5;

/// A differential pair recovered from copper already on the board.
///
/// KiCad writes the pair through a `DIFF_PAIR&` out parameter and answers
/// a `bool` (`pcbnew/router/pns_topology.h:101`); the pair travels as the
/// return value here.
///
/// The two lines come with it because they are the only way back to the
/// items the pair was assembled from. KiCad's `DIFF_PAIR` keeps them as
/// `m_line_p` and `m_line_n` (`pcbnew/router/pns_diff_pair.h:560`), which
/// the `DIFF_PAIR( const LINE&, const LINE& )` constructor fills and
/// `PLine()` hands back unchanged while they are linked (`:488`). This
/// crate's [`DiffPair`] holds chains only and rebuilds its two line views
/// on demand, so the linked originals would be lost. Both tuning placers
/// need them twice over: `AssembleTuningPath` starts from `PLine()`'s
/// first link (`pcbnew/router/pns_dp_meander_placer.cpp:120`) and `Start`
/// removes both lines from its branch by link (`:156`).
#[derive(Clone, Debug)]
pub struct AssembledDiffPair {
  /// The pair, with its measured gap, its width and its layers.
  pub pair: DiffPair,
  /// The P lane as assembled, still linked to the items it came from.
  pub line_p: Line,
  /// The N lane as assembled.
  pub line_n: Line,
}

/// What [`find_coupled_item`] has settled on so far.
///
/// The four locals of `AssembleDiffPair`'s `findNItem` lambda
/// (`pcbnew/router/pns_topology.cpp:1063` to `:1069`), which is called
/// twice and has to carry its best answer across both calls.
struct CoupledSearch {
  /// `minDist_sq` (`:1065`), the smallest segment to segment distance
  /// any accepted candidate had.
  min_dist_sq: i64,
  /// `minDistTarget_sq` (`:1066`), the smallest distance from a candidate
  /// to the centre of the clicked item.
  min_dist_target_sq: i64,
  /// `refItem` (`:1063`), the item on the clicked net that was paired.
  ref_item: Option<ItemId>,
  /// `coupledItem` (`:1064`), its partner on the coupled net.
  coupled_item: Option<ItemId>,
}

/// One pass of `findNItem` over the coupled net's candidates.
///
/// `pcbnew/router/pns_topology.cpp:1071` to `:1128`. A candidate has to be
/// the same kind of object, the same width, parallel within
/// [`DP_PARALLELITY_THRESHOLD`] and overlapping in the common parallel
/// projection; among those, the one nearest `p_id` wins, tie broken by
/// distance to `target_point`.
///
/// # Two things the transcription cannot keep
///
/// The arc arm (`:1098` to `:1112`) pairs two arcs by their centres and
/// scores them by the difference of their radii. [`LineChain`] cannot hold
/// an arc, so there is nothing to pair; the kind test above it already
/// refuses everything that is not a segment.
///
/// The tie break reads `n_item->Shape( -1 )->SquaredDistance( target )`
/// (`:1117`), which for a `SHAPE_SEGMENT` falls through to
/// `SHAPE::SquaredDistance` (`libs/kimath/src/geometry/shape.cpp:111`):
/// it polygonises the segment's copper and measures to that outline. The
/// centre line's squared distance is used instead. Every candidate that
/// reaches the test has the same width, so subtracting the same half
/// width from each cannot reorder them and squaring is monotone; the two
/// therefore choose the same candidate except when the target point lies
/// inside the copper of two of them at once, where KiCad's outline
/// distance is zero for both and this one still separates them.
fn find_coupled_item(
  world: &World,
  search: &mut CoupledSearch,
  n_items: &[ItemId],
  p_id: ItemId,
  target_point: Vec2,
) {
  let Some(p_item) = world.item(p_id) else {
    return;
  };

  for &n_id in n_items {
    let Some(n_item) = world.item(n_id) else {
      continue;
    };

    // :1077
    if n_item.kind() != p_item.kind() {
      continue;
    }

    // :1080
    let (ItemBody::Segment(p_segment), ItemBody::Segment(n_segment)) =
      (p_item.body(), n_item.body())
    else {
      continue;
    };

    // :1085
    if n_segment.width() != p_segment.width() {
      continue;
    }

    // :1088
    if !p_segment
      .seg()
      .approx_parallel(&n_segment.seg(), DP_PARALLELITY_THRESHOLD)
    {
      continue;
    }

    // :1093
    if common_parallel_projection(p_segment.seg(), n_segment.seg()).is_none() {
      continue;
    }

    // :1096
    let dist_sq = n_segment
      .seg()
      .squared_distance_to_segment(&p_segment.seg());

    // :1115. `<=` here and `<` on the tie break, so a later candidate at
    // the same distance only wins by being nearer the click.
    if dist_sq <= search.min_dist_sq {
      let dist_target_sq =
        n_segment.seg().squared_distance_to_point(target_point);

      if dist_target_sq < search.min_dist_target_sq {
        search.min_dist_target_sq = dist_target_sq;
        search.min_dist_sq = dist_sq;
        search.ref_item = Some(p_id);
        search.coupled_item = Some(n_id);
      }
    }
  }
}

/// Recover the differential pair a clicked track belongs to.
///
/// Port of `TOPOLOGY::AssembleDiffPair`,
/// `pcbnew/router/pns_topology.cpp:1036`. Note 07 section 12.2 deferred
/// it because milestone 10's placer never needed it; both pair length
/// tuners start with it and cannot start without it.
///
/// The host says which two nets are coupled, through
/// [`RuleResolver::dp_coupled_net`], and which of the two is the positive
/// half, through [`RuleResolver::dp_net_polarity`]: a negative polarity on
/// the clicked net swaps the two assembled lines so that
/// [`AssembledDiffPair::line_p`] really is P (`:1160`). A resolver that
/// answers neither, which is the default, makes this answer [`None`] and
/// a pair tuning session refuse to start.
///
/// # The gap is measured, not configured
///
/// `:1163` to `:1177` recovers the gap from the geometry: the
/// perpendicular distance between the two paired items, minus one lane's
/// width, so the answer is the **copper edge to edge** gap of the pair as
/// built rather than the one any rule asked for. It is negative when
/// nothing could be measured, which is what makes both placers fall back
/// to [`crate::settings::Sizes::diff_pair_gap`]
/// (`pcbnew/router/pns_dp_meander_placer.cpp:114`).
///
/// # The layer test is equality, not overlap
///
/// `:1052` and `:1061` compare `item->Layers() == startItem->Layers()`,
/// exact equality of the range. A coupled track on a layer range that
/// merely overlaps the clicked one is not a candidate.
///
/// # Determinism
///
/// KiCad collects the coupled net's items into a `std::set<ITEM*>`
/// (`:1057`) and the fallback links into another (`:1134`), so both are
/// iterated in **address** order. The candidate loop is order sensitive,
/// because a candidate that fails the tie break does not update
/// `minDist_sq` and therefore does not narrow the search for the ones
/// after it. Here [`World::all_items_in_net`] answers in uid order and the
/// fallback set is a [`BTreeSet`], so the answer is reproducible; note 07
/// line 1822 flags this as the same class of problem `dist_sq <= minDist_sq`
/// already has.
///
/// # Erratum: `pItems` is built and never read
///
/// `:1048` declares it, `:1053` fills it with the clicked line's same
/// layer segments, and nothing anywhere in the function reads it: the
/// search runs over `startItem` (`:1130`) and then over the items joined
/// to it (`:1153`), never over the rest of the clicked line. So a click in
/// the middle of a bent lane pairs only the segment under the cursor and
/// its immediate neighbours, which is what the fallback exists for. The
/// dead vector is not ported; note 08's errata list does not carry it.
#[must_use]
pub fn assemble_diff_pair(
  world: &World,
  node: NodeId,
  resolver: &dyn RuleResolver,
  start: ItemId,
) -> Option<AssembledDiffPair> {
  let item = world.item(start)?;

  // :1038, :1039
  let ref_net = item.net()?;
  let coupled_net = resolver.dp_coupled_net(ref_net)?;

  // :1040, the `dynamic_cast<LINKED_ITEM*>`: a segment, an arc or a via.
  if !item.of_kind(Kind::SEGMENT | Kind::ARC | Kind::VIA) {
    return None;
  }

  // :1045. Every argument is spelled out because the last one differs
  // from the crate's default: a segment of a different width does not
  // continue the line here.
  let line_p = world.assemble_line(node, start, None, false, false, false);

  // :1059 to :1066
  let layers = item.layers();
  let n_items: Vec<ItemId> = world
    .all_items_in_net(node, Some(coupled_net), Kind::SEGMENT | Kind::ARC)
    .into_iter()
    .filter(|id| {
      world
        .item(*id)
        .is_some_and(|candidate| candidate.layers() == layers)
    })
    .collect();

  // :1069. KiCad reads the centre of an uninitialised `BOX2I` when the
  // shape has none; every kind that reaches here has one.
  let target_point = item.shape(-1)?.center()?;
  let mut search = CoupledSearch {
    min_dist_sq: i64::MAX,
    min_dist_target_sq: i64::MAX,
    ref_item: None,
    coupled_item: None,
  };

  // :1130
  find_coupled_item(world, &mut search, &n_items, start, target_point);

  // :1132 to :1153. Nothing under the cursor could be paired, so try
  // whatever is joined to it at either end.
  if search.coupled_item.is_none() {
    let mut links_to_test: BTreeSet<ItemId> = BTreeSet::new();

    for index in 0..item.anchor_count() {
      let Some(reference) =
        find_joint_of_item(world, node, item.anchor(index), item)
      else {
        continue;
      };
      let Some(joint) = world.joint(reference) else {
        continue;
      };

      for &link in joint.links() {
        if link != start {
          links_to_test.insert(link);
        }
      }
    }

    for link in links_to_test {
      find_coupled_item(world, &mut search, &n_items, link, target_point);
    }
  }

  // :1155
  let ref_id = search.ref_item?;
  let coupled_id = search.coupled_item?;

  // :1158
  let line_n = world.assemble_line(node, coupled_id, None, false, false, false);

  // :1160
  let (line_p, line_n) = if resolver.dp_net_polarity(ref_net) < 0 {
    (line_n, line_p)
  } else {
    (line_p, line_n)
  };

  // :1163 to :1177. The arc arm needs two `SHAPE_ARC` radii and has
  // nothing to read here.
  let gap = gap_between(world, ref_id, coupled_id, line_p.width());

  // :1179 to :1182
  let mut pair = DiffPair::from_lines(&line_p, &line_n, 0);

  pair.set_width(line_p.width());
  pair.set_layers(line_p.layers());
  pair.set_gap(gap);

  Some(AssembledDiffPair {
    pair,
    line_p,
    line_n,
  })
}

/// The copper gap between the two items a pair was recognised by.
///
/// `pcbnew/router/pns_topology.cpp:1165` to `:1170`: the cross product of
/// the reference segment's direction with the displacement between the
/// two far anchors, divided by the reference length, which is the
/// perpendicular distance between the two centre lines, minus one lane's
/// width. `-1` when the reference is not a segment (`:1163`), which is
/// KiCad's "no gap could be measured".
fn gap_between(
  world: &World,
  ref_id: ItemId,
  coupled_id: ItemId,
  width: i32,
) -> i32 {
  let (Some(reference), Some(coupled)) =
    (world.item(ref_id), world.item(coupled_id))
  else {
    return -1;
  };

  if reference.kind() != Kind::SEGMENT {
    return -1;
  }

  // :1167
  let direction = reference.anchor(1) - reference.anchor(0);
  let displacement = reference.anchor(1) - coupled.anchor(1);
  let length = i64::from(direction.euclidean_norm());

  if length == 0 {
    return -1;
  }

  // :1170. KiCad narrows the quotient to an `int` before the subtraction.
  let distance = (direction.cross(displacement) / length).abs();

  i32::try_from(distance).unwrap_or(i32::MAX) - width
}

// ---------------------------------------------------------------------
// ConnectedItems
// ---------------------------------------------------------------------

/// Every item connected to a joint. Always empty.
///
/// Port of `TOPOLOGY::ConnectedItems`, whose two overloads
/// (`pcbnew/router/pns_topology.cpp:1021` and `:1027`, one taking a
/// `const JOINT*` and one an `ITEM*`) both consist of
/// `return ITEM_SET();`. They are declared at
/// `pcbnew/router/pns_topology.h:68` and `:69`, they have no caller in
/// KiCad's tree, and note 02 section 6.1 records them as stubs.
///
/// The stub is reproduced rather than left out so that a reader who goes
/// looking for the KiCad member finds the answer here instead of
/// inventing a plausible one. Anything that needs real connectivity wants
/// [`connected_joints`] or [`assemble_trivial_path`].
pub fn connected_items(_start: JointRef, _kind_mask: Kind) -> Vec<ItemId> {
  Vec::new()
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::seg::Seg;
  use crate::geometry::shape::Shape;
  use crate::item::{NetId, Segment, Solid, Via, ViaType};
  use crate::rules::{CoupledNets, FixedClearance};

  /// The net every fixture item sits on.
  const NET: Option<NetId> = Some(NetId(1));

  /// The width of every fixture track.
  const WIDTH: i32 = 1000;

  /// A pad on layer zero.
  fn add_pad(world: &mut World, node: NodeId, at: Vec2) -> ItemId {
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, 2000), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(NET);

    world.add_solid(node, item, None)
  }

  /// A track on layer zero.
  fn add_track(
    world: &mut World,
    node: NodeId,
    from: Vec2,
    to: Vec2,
  ) -> ItemId {
    let body = ItemBody::Segment(Segment::new(Seg::new(from, to), WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(NET);

    world
      .add_segment(node, item, false)
      .expect("the fixture track is neither degenerate nor redundant")
  }

  #[test]
  fn connected_joints_stops_at_a_pad_and_walks_the_whole_run() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    add_pad(&mut world, root, Vec2::new(0, 0));
    add_track(&mut world, root, Vec2::new(0, 0), Vec2::new(10_000, 0));
    add_track(&mut world, root, Vec2::new(10_000, 0), Vec2::new(20_000, 0));
    add_pad(&mut world, root, Vec2::new(20_000, 0));

    let start = world
      .find_joint(root, Vec2::new(0, 0), 0, NET)
      .expect("the west pad sits on a joint");

    assert_eq!(connected_joints(&world, root, start).len(), 3);
  }

  #[test]
  fn nearest_unconnected_item_ignores_what_the_joint_reaches() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    add_pad(&mut world, root, Vec2::new(0, 0));
    add_track(&mut world, root, Vec2::new(0, 0), Vec2::new(10_000, 0));

    let far = add_pad(&mut world, root, Vec2::new(50_000, 0));
    let start = world
      .find_joint(root, Vec2::new(0, 0), 0, NET)
      .expect("the west pad sits on a joint");

    let found = nearest_unconnected_item(&world, root, start, Kind::ANY)
      .expect("the far pad is on the net and unreachable");

    assert_eq!(found.item, far);
  }

  #[test]
  fn connected_items_is_a_stub() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    add_pad(&mut world, root, Vec2::new(0, 0));

    let start = world
      .find_joint(root, Vec2::new(0, 0), 0, NET)
      .expect("the pad sits on a joint");

    assert!(connected_items(start, Kind::ANY).is_empty());
  }

  #[test]
  fn a_leading_rat_line_points_at_the_only_unconnected_pad() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(100);

    add_pad(&mut world, root, Vec2::new(0, 0));
    add_pad(&mut world, root, Vec2::new(50_000, 0));

    let mut track = Line::new();

    track.set_width(WIDTH);
    track.set_layer(0);
    track.set_net(NET);
    track.set_shape(LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(10_000, 0)],
      false,
    ));

    let rat_line = leading_rat_line(&mut world, root, &rules, &track)
      .expect("the east pad is unconnected");

    assert_eq!(rat_line.point_count(), 2);
    assert_eq!(rat_line.point(0), Vec2::new(10_000, 0));
    assert_eq!(rat_line.point(1), Vec2::new(50_000, 0));
  }

  #[test]
  fn a_trivial_path_from_a_segment_names_both_terminals() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();

    add_pad(&mut world, root, Vec2::new(0, 0));

    let first =
      add_track(&mut world, root, Vec2::new(0, 0), Vec2::new(10_000, 0));

    add_track(&mut world, root, Vec2::new(10_000, 0), Vec2::new(20_000, 0));
    add_pad(&mut world, root, Vec2::new(20_000, 0));

    let path = assemble_trivial_path(&world, root, first, false);
    let (left, right) =
      path.terminals.expect("a two segment run has two terminals");

    let position = |reference: JointRef| {
      world
        .joint(reference)
        .expect("a terminal joint is live")
        .pos()
    };

    assert_eq!(position(left), Vec2::new(0, 0));
    assert_eq!(position(right), Vec2::new(20_000, 0));
  }

  /// A track with a collinear vertex loses it, in the node and not only
  /// in the caller's copy.
  ///
  /// The three fixture segments meet at `(10000, 0)` and `(20000, 0)`;
  /// only the first of those two joints is redundant, because the run
  /// turns at the second.
  #[test]
  fn simplify_line_drops_a_collinear_vertex_from_the_node() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let first =
      add_track(&mut world, root, Vec2::new(0, 0), Vec2::new(10_000, 0));

    add_track(&mut world, root, Vec2::new(10_000, 0), Vec2::new(20_000, 0));
    add_track(
      &mut world,
      root,
      Vec2::new(20_000, 0),
      Vec2::new(30_000, 10_000),
    );

    let line = world.assemble_line(root, first, None, false, false, false);

    assert_eq!(line.point_count(), 4);

    assert!(simplify_line(&mut world, root, &line));

    // The redundant joint is gone from the node and the corner is not.
    assert!(
      world
        .find_joint(root, Vec2::new(10_000, 0), 0, NET)
        .is_none()
    );
    assert!(
      world
        .find_joint(root, Vec2::new(20_000, 0), 0, NET)
        .is_some()
    );

    let start = world
      .find_joint(root, Vec2::new(0, 0), 0, NET)
      .expect("the west end still sits on a joint");
    let link = *world
      .joint(start)
      .expect("the joint is live")
      .links()
      .first()
      .expect("the joint holds the first segment of the run");
    let rebuilt = world.assemble_line(root, link, None, false, false, false);

    assert_eq!(rebuilt.point_count(), 3);
    assert!(!rebuilt.shape().points().contains(&Vec2::new(10_000, 0)));

    // And a run with nothing left to drop answers false, leaving the
    // node alone.
    assert!(!simplify_line(&mut world, root, &rebuilt));
    assert_eq!(
      world
        .assemble_line(root, link, None, false, false, false)
        .point_count(),
      3
    );
  }

  /// The two guards of `pns_topology.cpp:51`.
  #[test]
  fn simplify_line_refuses_a_line_that_is_not_in_the_node() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let mut loose = Line::new();

    loose.set_shape(LineChain::from_slice(
      &[Vec2::new(0, 0), Vec2::new(10_000, 0), Vec2::new(20_000, 0)],
      false,
    ));

    // Linked to nothing, so there is no root segment to assemble from.
    assert!(!simplify_line(&mut world, root, &loose));

    // And a linked line with no segment at all is refused by the second
    // half of the same guard.
    let pad = add_pad(&mut world, root, Vec2::new(0, 0));
    let mut empty = Line::new();

    empty.link(pad);

    assert!(!simplify_line(&mut world, root, &empty));
  }

  /// A track on one layer, of the fixture net.
  fn add_track_on(
    world: &mut World,
    node: NodeId,
    layer: i32,
    from: Vec2,
    to: Vec2,
  ) -> ItemId {
    let body = ItemBody::Segment(Segment::new(Seg::new(from, to), WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(NET);

    world
      .add_segment(node, item, false)
      .expect("the fixture track is neither degenerate nor redundant")
  }

  /// A through via of the fixture net, spanning layers zero and one.
  fn add_via(world: &mut World, node: NodeId, at: Vec2) -> ItemId {
    let body = ItemBody::Via(Via::new(at, 6000, 3000, ViaType::Through));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item.set_net(NET);

    world.add_via(node, item)
  }

  /// Every line of a path, in order, as its chain.
  fn path_lines(path: &TuningPath) -> Vec<LineChain> {
    path
      .items
      .iter()
      .filter_map(|item| match item {
        PathItem::Line(line) => Some(line.shape().clone()),
        PathItem::Via(_) => None,
      })
      .collect()
  }

  /// `walkTuningPath` walks **through** a pad where `followBranch` stops
  /// at one (`pcbnew/router/pns_topology.cpp:676`), so the path spans a
  /// net that runs through a series component.
  #[test]
  fn a_tuning_path_runs_through_an_intermediate_pad_to_both_terminals() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(100);

    let west = add_pad(&mut world, root, Vec2::new(0, 0));
    let seed =
      add_track(&mut world, root, Vec2::new(0, 0), Vec2::new(3_000_000, 0));

    add_pad(&mut world, root, Vec2::new(3_000_000, 0));
    add_track(
      &mut world,
      root,
      Vec2::new(3_000_000, 0),
      Vec2::new(8_000_000, 0),
    );

    let east = add_pad(&mut world, root, Vec2::new(8_000_000, 0));
    let path = assemble_tuning_path(&world, root, &rules, seed);

    assert_eq!(path.start_pad, Some(west));
    assert_eq!(path.end_pad, Some(east));
    assert_eq!(path_lines(&path).len(), 2);
    assert_eq!(path_length(&path.items), 8_000_000);
  }

  /// The depth first search keeps the **longest** branch (`:667`), which
  /// is what makes this a tuning walk and not a shortest connection walk.
  #[test]
  fn a_tuning_path_takes_the_longer_branch_out_of_a_pad() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(100);

    let seed =
      add_track(&mut world, root, Vec2::new(0, 0), Vec2::new(3_000_000, 0));

    add_pad(&mut world, root, Vec2::new(3_000_000, 0));

    // The short branch, and the long one, in that insertion order so
    // that the answer cannot come from the order the hits arrive in.
    add_track(
      &mut world,
      root,
      Vec2::new(3_000_000, 0),
      Vec2::new(4_000_000, 0),
    );
    add_track(
      &mut world,
      root,
      Vec2::new(3_000_000, 0),
      Vec2::new(3_000_000, 5_000_000),
    );

    let path = assemble_tuning_path(&world, root, &rules, seed);

    assert_eq!(path_length(&path.items), 3_000_000 + 5_000_000);
  }

  /// A via is walked through by way of `findLinesFromVia` (`:722`), and
  /// both the via and the run on the far side end up in the path.
  #[test]
  fn a_tuning_path_crosses_a_via_and_keeps_it() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(100);
    let corner = Vec2::new(3_000_000, 0);

    let seed = add_track_on(&mut world, root, 0, Vec2::new(0, 0), corner);
    let via = add_via(&mut world, root, corner);

    add_track_on(&mut world, root, 1, corner, Vec2::new(7_000_000, 0));

    let path = assemble_tuning_path(&world, root, &rules, seed);
    let vias: Vec<ItemId> = path
      .items
      .iter()
      .filter_map(|item| match item {
        PathItem::Via(id) => Some(*id),
        PathItem::Line(_) => None,
      })
      .collect();

    assert_eq!(vias, vec![via]);
    assert_eq!(path_lines(&path).len(), 2);
    assert_eq!(path_length(&path.items), 3_000_000 + 4_000_000);
  }

  /// The via itself adds nothing to the length: it is a point here, where
  /// KiCad's host adds the stackup distance. See [`path_length`].
  #[test]
  fn a_via_contributes_no_length() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let via = add_via(&mut world, root, Vec2::new(0, 0));

    assert_eq!(path_length(&[PathItem::Via(via)]), 0);
    assert_eq!(path_length(&[]), 0);
  }

  /// `AssembleTuningPath`'s `seg` stays null for anything that is neither
  /// a via nor a segment (`:842`), and the answer is an empty set.
  #[test]
  fn a_tuning_path_from_a_pad_is_empty() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(100);
    let pad = add_pad(&mut world, root, Vec2::new(0, 0));

    add_track(&mut world, root, Vec2::new(0, 0), Vec2::new(3_000_000, 0));

    let path = assemble_tuning_path(&world, root, &rules, pad);

    assert!(path.items.is_empty());
    assert!(path.start_pad.is_none());
    assert!(path.end_pad.is_none());
  }

  /// One walk of a dead end answers KiCad's initial `WALK_RESULT`, whose
  /// length is minus one (`pcbnew/router/pns_topology.h:125`) until the
  /// zero length dead end replaces it.
  #[test]
  fn a_walk_out_of_a_dead_end_finds_nothing_but_reports_zero() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(100);
    let seed =
      add_track(&mut world, root, Vec2::new(0, 0), Vec2::new(3_000_000, 0));
    let line = world.assemble_line(root, seed, None, false, true, true);
    let visited: BTreeSet<ItemId> = line.links().iter().copied().collect();
    let walk = walk_tuning_path(&world, root, &rules, &line, true, &visited);

    assert_eq!(walk.length, 0);
    assert!(walk.items.is_empty());
    assert!(walk.end_pad.is_none());
  }

  // -----------------------------------------------------------------
  // AssembleDiffPair
  // -----------------------------------------------------------------

  /// The positive half of the pair fixture.
  const PAIR_NET_P: Option<NetId> = Some(NetId(1));

  /// The negative half.
  const PAIR_NET_N: Option<NetId> = Some(NetId(2));

  /// The width of one lane of the pair fixture.
  const LANE_WIDTH: i32 = 200_000;

  /// The centre to centre spacing of the two lanes.
  const LANE_PITCH: i32 = 400_000;

  /// A track of a chosen net and the lane width.
  fn add_lane(
    world: &mut World,
    node: NodeId,
    net: Option<NetId>,
    from: Vec2,
    to: Vec2,
  ) -> ItemId {
    let body = ItemBody::Segment(Segment::new(Seg::new(from, to), LANE_WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);

    world
      .add_segment(node, item, false)
      .expect("a fixture lane is neither degenerate nor redundant")
  }

  /// Note 08 section 14.3's pair board: two straight lanes a pitch apart,
  /// eight millimetres long, and the handles of the P and N tracks.
  fn pair_board() -> (World, ItemId, ItemId) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let track_p = add_lane(
      &mut world,
      root,
      PAIR_NET_P,
      Vec2::new(0, -LANE_PITCH / 2),
      Vec2::new(8_000_000, -LANE_PITCH / 2),
    );
    let track_n = add_lane(
      &mut world,
      root,
      PAIR_NET_N,
      Vec2::new(0, LANE_PITCH / 2),
      Vec2::new(8_000_000, LANE_PITCH / 2),
    );

    (world, track_p, track_n)
  }

  /// The resolver that says which two nets are coupled.
  fn pair_rules() -> CoupledNets {
    CoupledNets::new(FixedClearance::uniform(100_000), NetId(1), NetId(2))
  }

  /// A click on either lane recovers both lines, P first whichever was
  /// clicked, with the measured gap and the clicked lane's layers.
  #[test]
  fn a_click_on_either_lane_of_a_pair_recovers_both() {
    let (world, track_p, track_n) = pair_board();
    let root = world.root();
    let rules = pair_rules();

    for start in [track_p, track_n] {
      let found = assemble_diff_pair(&world, root, &rules, start)
        .expect("the two lanes are a pair");

      assert_eq!(found.pair.nets(), (PAIR_NET_P, PAIR_NET_N));
      assert_eq!(found.line_p.net(), PAIR_NET_P);
      assert_eq!(found.line_n.net(), PAIR_NET_N);
      assert_eq!(found.pair.width(), LANE_WIDTH);
      assert_eq!(found.pair.layers(), LayerRange::single(0));
      // The gap is measured, not configured: the pitch minus one width.
      assert_eq!(found.pair.gap(), LANE_PITCH - LANE_WIDTH);
      assert_eq!(found.pair.chain_p().point(0).y, -LANE_PITCH / 2);
      assert_eq!(found.pair.chain_n().point(0).y, LANE_PITCH / 2);
      // Both lines carry the links the placers remove and walk from.
      assert_eq!(found.line_p.links(), &[track_p]);
      assert_eq!(found.line_n.links(), &[track_n]);
    }
  }

  /// A resolver with no pair concept, which is the default, answers
  /// nothing and no pair is recovered.
  #[test]
  fn a_net_with_no_coupled_partner_is_not_a_pair() {
    let (world, track_p, _) = pair_board();
    let root = world.root();
    let rules = FixedClearance::uniform(100_000);

    assert!(assemble_diff_pair(&world, root, &rules, track_p).is_none());
  }

  /// A pad is not a `LINKED_ITEM`, so it cannot start the search
  /// (`pcbnew/router/pns_topology.cpp:1040`).
  #[test]
  fn a_pad_cannot_start_a_pair() {
    let (mut world, _, _) = pair_board();
    let root = world.root();
    let rules = pair_rules();
    let pad = add_pad(&mut world, root, Vec2::new(0, -LANE_PITCH / 2));

    assert!(assemble_diff_pair(&world, root, &rules, pad).is_none());
  }

  /// A click on a stub that is not parallel to anything on the coupled
  /// net falls through to the joined items (`:1132`), which is the only
  /// path that reaches a segment other than the clicked one: `pItems` is
  /// built and never read.
  #[test]
  fn a_click_on_an_unpairable_stub_falls_back_to_the_joined_items() {
    let (mut world, track_p, track_n) = pair_board();
    let root = world.root();
    let rules = pair_rules();
    let stub = add_lane(
      &mut world,
      root,
      PAIR_NET_P,
      Vec2::new(0, -LANE_PITCH / 2),
      Vec2::new(0, -1_200_000),
    );

    let found = assemble_diff_pair(&world, root, &rules, stub)
      .expect("the stub's neighbour is half of a pair");

    assert_eq!(found.pair.gap(), LANE_PITCH - LANE_WIDTH);
    assert_eq!(found.line_n.links(), &[track_n]);
    // The clicked line is the whole run the stub belongs to, stub first,
    // because `AssembleLine` follows through the joint of degree two.
    assert!(found.line_p.contains_link(track_p));
    assert!(found.line_p.contains_link(stub));
  }

  /// A coupled track on a different layer is not a candidate: the test
  /// is exact equality of the layer range (`:1052`, `:1061`).
  #[test]
  fn a_coupled_track_on_another_layer_is_not_a_candidate() {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = pair_rules();
    let track_p = add_lane(
      &mut world,
      root,
      PAIR_NET_P,
      Vec2::new(0, -LANE_PITCH / 2),
      Vec2::new(8_000_000, -LANE_PITCH / 2),
    );
    let body = ItemBody::Segment(Segment::new(
      Seg::new(
        Vec2::new(0, LANE_PITCH / 2),
        Vec2::new(8_000_000, LANE_PITCH / 2),
      ),
      LANE_WIDTH,
    ));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(1));
    item.set_net(PAIR_NET_N);
    world
      .add_segment(root, item, false)
      .expect("the second layer lane is not redundant");

    assert!(assemble_diff_pair(&world, root, &rules, track_p).is_none());
  }
}
