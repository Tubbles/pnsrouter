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
//! - `AssembleTuningPath`, `walkTuningPath` and `findLinesFromVia`
//!   (`:787`, `:611`, `:536`): length tuning only.
//! - `AssembleDiffPair` and the `DP_PARALLELITY_THRESHOLD` machinery
//!   (`:1036`): differential pairs only.
//! - `ShortestConnectionLength`, declared at
//!   `pcbnew/router/pns_topology.h:70` and never defined.
//! - `AssembleCluster` (`:1187`), which is [`World::assemble_cluster`]:
//!   it lives on the world because the walkaround calls it on every
//!   iteration and it is a spatial flood fill rather than a connectivity
//!   query.

use std::collections::{BTreeSet, VecDeque};

use crate::geometry::line_chain::LineChain;
use crate::geometry::vec2::Vec2;
use crate::item::{Item, ItemBody, ItemId, Kind, LayerRange};
use crate::line::Line;
use crate::node::{JointRef, NodeId, World};
use crate::rules::RuleResolver;

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
  use crate::item::{NetId, Segment, Solid};
  use crate::rules::FixedClearance;

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
}
