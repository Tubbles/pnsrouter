// SPDX-License-Identifier: GPL-3.0-or-later

//! Joints: the connectivity graph laid over the items.
//!
//! Port of `PNS::JOINT` (`pcbnew/router/pns_joint.h:42`) and of the joint
//! half of `PNS::NODE` (`pcbnew/router/pns_node.cpp`), which is the only
//! place joints are created, merged, linked and torn down.
//!
//! A joint is a position, on a net, over a run of layers, that binds a
//! number of items together. The router walks it to assemble a line out of
//! segments (`pcbnew/router/pns_node.cpp:1074`), to decide where a line
//! ends, and to recognise the shapes it has names for: a line corner, a
//! stitching via, a width change.
//!
//! # Identity
//!
//! KiCad keeps joints by value in an `unordered_multimap` and identifies
//! them by the address of the mapped value
//! (`doc/reference/kicad/02-item-model-and-node.md` sections 10.5 and 11
//! entry 7). That address is not stable: `touchJoint` erases the joints it
//! merges and inserts a fresh one (`pcbnew/router/pns_node.cpp:1432`,
//! `:1439`), and a branch's copy of a root joint is a different object
//! from the root's (`:1411`). Here a joint lives in an arena and is named
//! by a [`JointId`], and [`JointMap::touch_joint`] **keeps** the id of the
//! first joint it merges into rather than making a new one. That is the
//! deliberate deviation: an id obtained before a merge still names the
//! joint that absorbed the others, where the C++ pointer would dangle. The
//! ids of the absorbed joints do go stale, because those joints are gone.
//!
//! # Several joints per key
//!
//! The key is position and net only, never layers
//! (`JOINT::HASH_TAG`, `pcbnew/router/pns_joint.h:47`). Several joints can
//! share a key and differ in layer range, which is what makes a via joint
//! splittable back into per layer joints when the via is removed
//! ([`JointMap::rebuild_joint`]). The disambiguation is explicit here,
//! where KiCad walks the multimap's bucket chain past `equal_range` into
//! unrelated entries and filters them by position and net afterwards
//! (`pcbnew/router/pns_node.cpp:1374` to `:1380`).
//!
//! # The tombstone
//!
//! A branch has to be able to say "there is deliberately nothing at this
//! position", or [`JointMap::find_joint`] would fall through to the root
//! and resurrect a joint the branch removed. KiCad writes that as a joint
//! whose layer range is `(-1, -1)`, which no query can ever overlap
//! (`pcbnew/router/pns_node.cpp:913`, `pcbnew/router/pns_layerset.h:69`).
//! Here it is the **absence of a joint** under a key that is present in
//! the map, as `DESIGN.md` section 4.4 asks. So:
//!
//! - `by_key` never holds an empty entry by accident. An empty entry is
//!   the tombstone and nothing else.
//! - [`JointMap::contains_key`] is what the node consults, and it answers
//!   true for a tombstone, exactly as KiCad's `m_joints.find( tag ) !=
//!   end()` does.
//!
//! The second place KiCad writes a negative range is `JOINT::Unlink`
//! (`pcbnew/router/pns_joint.h:229`): a joint that loses its last link
//! keeps its slot in the map with an undefined range, and is invisible
//! from then on ("fixme: remove dangling joints",
//! `pcbnew/router/pns_node.cpp:1466`). That one **is** reproduced as is,
//! because the joint object survives and so does its id.
//!
//! # Two levels, and what this module does not do
//!
//! `NODE::FindJoint` looks in the node's own map and, **only if the key is
//! absent there**, in the root's (`pcbnew/router/pns_node.cpp:1366` to
//! `:1372`). A key that is present but holds nothing overlapping the
//! queried layer answers "no joint" without consulting the root; that is
//! the whole point of the tombstone. [`JointMap`] is one map, so the node
//! composes the two levels itself:
//!
//! ```text
//! branch.find_joint(pos, layer, net).or_else(|| {
//!   if branch.contains_key(pos, net) { None } else { root.find_joint(..) }
//! })
//! ```
//!
//! The returned [`JointId`] then belongs to whichever map answered, which
//! is why [`JointMap::find_joint`] does not take the root: two maps have
//! two arenas. The write path is different: [`JointMap::touch_joint`] does
//! take the root, because it has to copy the root's joints into the branch
//! before mutating them, and the copies get local ids
//! (`pcbnew/router/pns_node.cpp:1404` to `:1412`).
//!
//! # Members not ported
//!
//! - `JOINT::Clone` (`pcbnew/router/pns_joint.h:90`), which asserts and
//!   returns null.
//! - `JOINT::Dump` (`pcbnew/router/pns_node.cpp:1443`), a logging helper.
//! - `JOINT::Links` and `CLinks` (`pcbnew/router/pns_joint.h:308`,
//!   `:313`), which hand out KiCad's `ITEM_SET` so that callers can filter
//!   it in place. [`Joint::links`] is the read only slice and the
//!   mutations go through [`Joint::link`] and [`Joint::unlink`].
//! - `JOINT::operator==` (`pcbnew/router/pns_joint.h:325`), which compares
//!   position and net and ignores everything else. It is
//!   [`Joint::same_key_as`] here, so that the derived `PartialEq` can mean
//!   what it says.
//! - `JOINT_TAG_HASH` (`pcbnew/router/pns_joint.h:53`). The map is
//!   ordered, so there is nothing to hash (`DESIGN.md` section 8).

use std::cmp::Ordering;
use std::collections::BTreeMap;

use crate::arena::{Arena, ArenaId};
use crate::geometry::box2::Box2;
use crate::geometry::vec2::{Vec2, Vec2L};
use crate::item::{Item, ItemBody, ItemId, Kind, LayerRange, NetId};

// ---------------------------------------------------------------------
// JointId and JointKey
// ---------------------------------------------------------------------

/// A handle to a [`Joint`] in a [`JointMap`]'s arena.
///
/// This replaces the `const JOINT*` that KiCad passes around. See the
/// module documentation for what it guarantees that an address does not.
pub type JointId = ArenaId<Joint>;

/// What a joint is keyed on: where it is and which net it belongs to.
///
/// Port of `JOINT::HASH_TAG`, `pcbnew/router/pns_joint.h:47`. The layer
/// range is deliberately **not** part of the key, despite the comment
/// above the struct saying joints are hashed by layers as well.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct JointKey {
  /// The position. Port of `HASH_TAG::pos`.
  pub pos: Vec2,
  /// The net, or `None` for no net. Port of `HASH_TAG::net`.
  pub net: Option<NetId>,
}

impl JointKey {
  /// A key.
  pub const fn new(pos: Vec2, net: Option<NetId>) -> Self {
    Self { pos, net }
  }
}

/// Keys order by position, lexicographically, then by net.
///
/// KiCad has no order on the tag at all: the container is an
/// `unordered_multimap` and the tag has a hash
/// (`pcbnew/router/pns_joint.h:53`). An order is needed here because the
/// map is a [`BTreeMap`], and `VECTOR2I` has two contradictory ones to
/// choose from: `operator<` compares squared magnitudes
/// (`libs/kimath/include/math/vector2d.h:517`) while
/// `std::less<VECTOR2I>`, which is what its `std::map` keys use, is
/// lexicographic in `x` then `y`
/// (`libs/kimath/src/math/vector2.cpp:22`). This is the lexicographic
/// one, spelled out rather than derived, as the `Vec2` decision in
/// `doc/log/2026-09-08.md` requires.
impl Ord for JointKey {
  fn cmp(&self, other: &Self) -> Ordering {
    (self.pos.x, self.pos.y, self.net).cmp(&(
      other.pos.x,
      other.pos.y,
      other.net,
    ))
  }
}

impl PartialOrd for JointKey {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.cmp(other))
  }
}

// ---------------------------------------------------------------------
// Joint
// ---------------------------------------------------------------------

/// A point where items of one net meet.
///
/// Port of `PNS::JOINT`, `pcbnew/router/pns_joint.h:42`. KiCad derives it
/// from `ITEM` so that it can carry a layer range and be passed to
/// `LayersOverlap`; here it is a plain value, because nothing else about
/// an item applies to it.
///
/// The derived `PartialEq` compares every field. KiCad's
/// `JOINT::operator==` (`:325`) compares only position and net, which is
/// [`Joint::same_key_as`].
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Joint {
  /// Where it is and whose net it is on. Port of `m_tag`, `:364`.
  key: JointKey,
  /// The layers it binds. Port of `ITEM::m_layers` as a `JOINT` uses it,
  /// `:75`. It is the hull of the layer ranges of everything linked, and
  /// [`LayerRange::UNDEFINED`] once the last link goes (`:229`).
  layers: LayerRange,
  /// What is linked here, in link order. Port of `m_linkedItems`,
  /// `:367`, whose `ITEM_SET` is a `std::vector<ITEM*>` underneath
  /// (`pcbnew/router/pns_itemset.h:190`).
  links: Vec<ItemId>,
  /// Whether the shove has pinned this joint. Port of `m_locked`,
  /// `:370`.
  locked: bool,
}

impl Joint {
  /// A joint with no links.
  ///
  /// Port of `JOINT( const VECTOR2I&, const PNS_LAYER_RANGE&, NET_HANDLE )`,
  /// `pcbnew/router/pns_joint.h:70`.
  pub const fn new(pos: Vec2, layers: LayerRange, net: Option<NetId>) -> Self {
    Self {
      key: JointKey { pos, net },
      layers,
      links: Vec::new(),
      locked: false,
    }
  }

  /// The key. Port of `Tag`, `pcbnew/router/pns_joint.h:288`.
  pub const fn key(&self) -> JointKey {
    self.key
  }

  /// Where the joint is. Port of `Pos`,
  /// `pcbnew/router/pns_joint.h:293`.
  pub const fn pos(&self) -> Vec2 {
    self.key.pos
  }

  /// The net. Port of `JOINT::Net`,
  /// `pcbnew/router/pns_joint.h:298`.
  pub const fn net(&self) -> Option<NetId> {
    self.key.net
  }

  /// The layers the joint binds. Port of `ITEM::Layers` on a joint.
  pub const fn layers(&self) -> LayerRange {
    self.layers
  }

  /// What is linked here, in link order.
  ///
  /// Port of `LinkList`, `pcbnew/router/pns_joint.h:303`.
  pub fn links(&self) -> &[ItemId] {
    &self.links
  }

  /// How many links match a kind mask.
  ///
  /// Port of `LinkCount`, `pcbnew/router/pns_joint.h:318`, which forwards
  /// to `ITEM_SET::Count` (`pcbnew/router/pns_itemset.h:72`). [`Kind::ANY`]
  /// is KiCad's `-1` and answers the link count without looking at
  /// anything, which is why a stale link still counts there and not under
  /// a narrower mask.
  pub fn link_count(&self, arena: &Arena<Item>, mask: Kind) -> usize {
    if mask == Kind::ANY {
      return self.links.len();
    }

    self
      .links
      .iter()
      .filter(|id| arena.get(**id).is_some_and(|item| item.of_kind(mask)))
      .count()
  }

  /// Link an item, unless it is already linked.
  ///
  /// Port of `JOINT::Link`, `pcbnew/router/pns_joint.h:215`, whose
  /// duplicate test is a linear pointer search
  /// (`pcbnew/router/pns_itemset.h:161`).
  pub fn link(&mut self, item: ItemId) {
    if self.links.contains(&item) {
      return;
    }

    self.links.push(item);
  }

  /// Unlink an item, and report whether the joint is now dangling.
  ///
  /// Port of `JOINT::Unlink`, `pcbnew/router/pns_joint.h:225`. Losing the
  /// last link resets the layer range to `(-1, -1)`, which is what makes
  /// the joint invisible to every later query while its entry stays in
  /// the map; see the module documentation. `ITEM_SET::Erase`
  /// (`pcbnew/router/pns_itemset.h:166`) removes the first occurrence, and
  /// [`Joint::link`] never makes a second one.
  pub fn unlink(&mut self, item: ItemId) -> bool {
    if let Some(index) = self.links.iter().position(|id| *id == item) {
      self.links.remove(index);
    }

    if self.links.is_empty() {
      self.layers = LayerRange::UNDEFINED;
    }

    self.links.is_empty()
  }

  /// Whether the joint has the same position and net as another.
  ///
  /// Port of `JOINT::operator==`, `pcbnew/router/pns_joint.h:325`, which
  /// ignores layers and links.
  pub fn same_key_as(&self, other: &Joint) -> bool {
    self.key == other.key
  }

  /// Whether the two joints are the same position, the same net and share
  /// a layer.
  ///
  /// Port of `Overlaps`, `pcbnew/router/pns_joint.h:346`, which the note
  /// calls the real identity test.
  pub fn overlaps(&self, other: &Joint) -> bool {
    self.key == other.key && self.layers.overlaps(other.layers)
  }

  /// Absorb another joint.
  ///
  /// Port of `Merge`, `pcbnew/router/pns_joint.h:330`: a no op unless
  /// [`Joint::overlaps`] holds, then the hull of the layer ranges, the
  /// `or` of the locked flags, and every link appended.
  ///
  /// The append is `ITEM_SET::Add` (`pcbnew/router/pns_itemset.h:139`),
  /// which does **not** deduplicate, where [`Joint::link`] does. Two
  /// joints under one key that share a link would therefore leave a
  /// duplicate. Reproduced, because reaching that state means two joints
  /// that overlap were not merged when they should have been, and hiding
  /// it would hide the real fault.
  pub fn merge(&mut self, other: &Joint) {
    if !self.overlaps(other) {
      return;
    }

    self.layers.merge(other.layers);

    if other.locked {
      self.locked = true;
    }

    self.links.extend_from_slice(&other.links);
  }

  /// Whether the joint connects exactly two tracks of the same width.
  ///
  /// Port of `IsLineCorner`, `pcbnew/router/pns_joint.h:101`. Two
  /// branches:
  ///
  /// - exactly two links, both tracks: trivial unless one of them is
  ///   locked and `allow_locked_segs` is false, and the widths have to
  ///   match;
  /// - more than two links of which exactly two are tracks: only ever
  ///   trivial when `allow_locked_segs` is set, and then only if every
  ///   other link is a virtual via. KiCad's comment at `:119` explains
  ///   why that case exists at all: it adds a virtual via to each end of
  ///   every locked segment, so a joint between two locked segments
  ///   carries several of them.
  ///
  /// A link with no width answers "no width" and therefore never matches;
  /// KiCad compares `LINKED_ITEM::Width` there, which a segment and an
  /// arc both have and a via and a pad do not.
  pub fn is_line_corner(
    &self,
    arena: &Arena<Item>,
    allow_locked_segs: bool,
  ) -> bool {
    let tracks = self.link_count(arena, Kind::SEGMENT | Kind::ARC);

    if self.links.len() == 2 && tracks == 2 {
      let (Some(first), Some(second)) =
        (arena.get(self.links[0]), arena.get(self.links[1]))
      else {
        return false;
      };

      if !allow_locked_segs && (first.is_locked() || second.is_locked()) {
        return false;
      }

      return same_width(first, second);
    }

    if self.links.len() > 2 && tracks == 2 {
      if !allow_locked_segs {
        return false;
      }

      let mut first: Option<&Item> = None;
      let mut second: Option<&Item> = None;

      for id in &self.links {
        let Some(item) = arena.get(*id) else {
          continue;
        };

        if item.is_virtual() {
          continue;
        }

        if item.kind() == Kind::SEGMENT || item.kind() == Kind::ARC {
          if first.is_none() {
            first = Some(item);
          } else {
            second = Some(item);
          }
        } else {
          return false;
        }
      }

      if let (Some(first), Some(second)) = (first, second) {
        return same_width(first, second);
      }
    }

    false
  }

  /// Whether the joint is a via with exactly one track coming in and one
  /// going out.
  ///
  /// Port of `IsNonFanoutVia`, `pcbnew/router/pns_joint.h:149`. Virtual
  /// items do not count towards any of the three totals.
  pub fn is_non_fanout_via(&self, arena: &Arena<Item>) -> bool {
    let mut vias = 0;
    let mut tracks = 0;
    let mut real = 0;

    for id in &self.links {
      let Some(item) = arena.get(*id) else {
        continue;
      };

      if item.is_virtual() {
        continue;
      }

      if item.kind() == Kind::VIA {
        vias += 1;
      } else if item.kind() == Kind::SEGMENT || item.kind() == Kind::ARC {
        tracks += 1;
      }

      real += 1;
    }

    real == 3 && vias == 1 && tracks == 2
  }

  /// Whether the joint is a via with nothing else on it.
  ///
  /// Port of `IsStitchingVia`, `pcbnew/router/pns_joint.h:171`. Note that
  /// this one counts virtual items, where
  /// [`Joint::is_non_fanout_via`] does not.
  pub fn is_stitching_via(&self, arena: &Arena<Item>) -> bool {
    self.links.len() == 1 && self.link_count(arena, Kind::VIA) == 1
  }

  /// Whether the joint is the loose end of a single track.
  ///
  /// Port of `IsTrivialEndpoint`, `pcbnew/router/pns_joint.h:176`,
  /// including its own "fixme: Arcs & trivial endpoint vias" at `:178`:
  /// the mask is [`Kind::SEGMENT`] alone, so an arc end is not one.
  pub fn is_trivial_endpoint(&self, arena: &Arena<Item>) -> bool {
    self.links.len() == 1 && self.link_count(arena, Kind::SEGMENT) == 1
  }

  /// Whether the joint is where a track changes width.
  ///
  /// Port of `IsTraceWidthChange`, `pcbnew/router/pns_joint.h:183`. A via
  /// anywhere on the joint disqualifies it, virtual items are skipped, and
  /// the count that gates the whole test is over [`Kind::SEGMENT`] only
  /// while the loop that picks the two also accepts an arc.
  pub fn is_trace_width_change(&self, arena: &Arena<Item>) -> bool {
    if self.link_count(arena, Kind::SEGMENT) != 2 {
      return false;
    }

    let mut first: Option<&Item> = None;
    let mut second: Option<&Item> = None;

    for id in &self.links {
      let Some(item) = arena.get(*id) else {
        continue;
      };

      if item.is_virtual() {
        continue;
      }

      if item.kind() == Kind::VIA {
        return false;
      }

      if item.kind() == Kind::SEGMENT || item.kind() == Kind::ARC {
        if first.is_none() {
          first = Some(item);
        } else {
          second = Some(item);
        }
      }
    }

    // KiCad's `wxCHECK( seg1 && seg2, false )` at `:209`.
    let (Some(first), Some(second)) = (first, second) else {
      return false;
    };

    !same_width(first, second)
  }

  /// The track to continue along, or `None` at the end of a line.
  ///
  /// Port of `NextSegment`, `pcbnew/router/pns_joint.h:235`. The rules,
  /// in the order KiCad applies them:
  ///
  /// - a track of the same net whose layers overlap is a candidate;
  /// - a **second** candidate ends the line, and that test runs before the
  ///   locked test, so a locked third branch still terminates the line;
  /// - a locked track is only a candidate when `allow_locked_segs` is set;
  /// - a pad or a via ends the line, except a virtual via when
  ///   `allow_locked_segs` is set (`:261`).
  ///
  /// `current` is compared by handle, where KiCad compares addresses
  /// (`:246`).
  pub fn next_segment(
    &self,
    arena: &Arena<Item>,
    current: ItemId,
    allow_locked_segs: bool,
  ) -> Option<ItemId> {
    let current_item = arena.get(current)?;
    let mut other = None;

    for id in &self.links {
      if *id == current {
        continue;
      }

      let Some(item) = arena.get(*id) else {
        continue;
      };

      if item.of_kind(Kind::SEGMENT | Kind::ARC) {
        if item.net() == current_item.net()
          && item.layers().overlaps(current_item.layers())
        {
          if other.is_some() {
            return None;
          }

          if !item.is_locked() || allow_locked_segs {
            other = Some(*id);
          }
        }
      } else if item.of_kind(Kind::SOLID | Kind::VIA) {
        if item.kind() == Kind::VIA && item.is_virtual() && allow_locked_segs {
          continue;
        }

        return None;
      }
    }

    other
  }

  /// The first via linked here.
  ///
  /// Port of `JOINT::Via`, `pcbnew/router/pns_joint.h:275`.
  pub fn via(&self, arena: &Arena<Item>) -> Option<ItemId> {
    self
      .links
      .iter()
      .find(|id| arena.get(**id).is_some_and(|item| item.of_kind(Kind::VIA)))
      .copied()
  }

  /// Whether the shove has pinned this joint.
  ///
  /// Port of `IsLocked`, `pcbnew/router/pns_joint.h:357`.
  pub const fn is_locked(&self) -> bool {
    self.locked
  }

  /// Pin or unpin the joint.
  ///
  /// Port of `Lock`, `pcbnew/router/pns_joint.h:352`, whose default
  /// argument is `true`.
  pub const fn lock(&mut self, lock: bool) {
    self.locked = lock;
  }
}

/// Whether two linked items are tracks of the same width.
///
/// The `seg1->Width() == seg2->Width()` of `IsLineCorner`
/// (`pcbnew/router/pns_joint.h:112`) and the `!=` of
/// `IsTraceWidthChange` (`:211`), which read `LINKED_ITEM::Width`
/// (`pcbnew/router/pns_linked_item.h:39`). A segment and an arc both
/// have one, and a joint where the two meet at the same width is an
/// ordinary line corner: there is no joint level notion of tangency
/// anywhere in KiCad (note 09 section 4.4). Every other body answers
/// false, which is how a via or a pad fails the test.
fn same_width(first: &Item, second: &Item) -> bool {
  let width = |item: &Item| match item.body() {
    ItemBody::Segment(segment) => Some(segment.width()),
    ItemBody::Arc(arc) => Some(arc.width()),
    _ => None,
  };

  match (width(first), width(second)) {
    (Some(first), Some(second)) => first == second,
    _ => false,
  }
}

// ---------------------------------------------------------------------
// JointMap
// ---------------------------------------------------------------------

/// Every joint of one node.
///
/// Port of `NODE::m_joints`, the
/// `std::unordered_multimap<JOINT::HASH_TAG, JOINT, JOINT_TAG_HASH>` at
/// `pcbnew/router/pns_node.h:589`, together with the `NODE` members that
/// are the only ways to touch it.
///
/// The multimap becomes a key to list of ids map plus an arena: the
/// several joints one key can hold are the list, and the arena is what
/// gives them stable identities. See the module documentation for the
/// tombstone invariant and for how the node combines a branch map with the
/// root's.
#[derive(Clone, Debug, Default)]
pub struct JointMap {
  /// The joints under each key, in the order they were created.
  ///
  /// An entry with an empty list is the tombstone, and the only way an
  /// empty list can arise.
  by_key: BTreeMap<JointKey, Vec<JointId>>,
  /// Where the joints live.
  arena: Arena<Joint>,
}

impl JointMap {
  /// An empty map.
  pub fn new() -> Self {
    Self::default()
  }

  /// How many joints the map holds, tombstones excluded.
  ///
  /// A joint that lost its last link still counts, because it is still
  /// there; see the module documentation.
  pub fn len(&self) -> usize {
    self.arena.len()
  }

  /// Whether the map holds no joint.
  pub fn is_empty(&self) -> bool {
    self.arena.is_empty()
  }

  /// Borrow a joint.
  pub fn get(&self, id: JointId) -> Option<&Joint> {
    self.arena.get(id)
  }

  /// Borrow a joint mutably.
  pub fn get_mut(&mut self, id: JointId) -> Option<&mut Joint> {
    self.arena.get_mut(id)
  }

  /// Every joint, in arena order.
  ///
  /// The order is reproducible but has no geometric meaning; use
  /// [`JointMap::query_joints`], which walks the keys in position order,
  /// when the order is part of the answer.
  pub fn joints(&self) -> impl Iterator<Item = (JointId, &Joint)> {
    self.arena.iter()
  }

  /// Whether the map has an entry for a key, tombstone included.
  ///
  /// This is KiCad's `m_joints.find( tag ) != m_joints.end()`, the test
  /// that decides whether `NODE::FindJoint` falls through to the root
  /// (`pcbnew/router/pns_node.cpp:1368`) and whether `rebuildJoint` has to
  /// plant a tombstone (`:911`).
  pub fn contains_key(&self, pos: Vec2, net: Option<NetId>) -> bool {
    self.by_key.contains_key(&JointKey { pos, net })
  }

  /// The joints under one key, in creation order.
  pub fn joints_at(&self, pos: Vec2, net: Option<NetId>) -> &[JointId] {
    self
      .by_key
      .get(&JointKey { pos, net })
      .map_or(&[], Vec::as_slice)
  }

  /// The joint at a position, on a net, that covers a layer.
  ///
  /// Port of the local half of `NODE::FindJoint`,
  /// `pcbnew/router/pns_node.cpp:1359`. KiCad walks the multimap from the
  /// first entry with the tag to the end of the container, testing
  /// position, net and `Layers().Overlaps( aLayer )` on each; the walk
  /// past `equal_range` is what
  /// `doc/reference/kicad/02-item-model-and-node.md` section 10.5 asks to
  /// replace with an explicit lookup, which is what this is.
  ///
  /// `layer` is a single layer, not a range: that is KiCad's signature,
  /// and its `FindJoint( pos, item )` overload
  /// (`pcbnew/router/pns_node.h:478`) passes `item->Layers().Start()`.
  ///
  /// A joint that lost its last link has an undefined range and can never
  /// be the answer (`pcbnew/router/pns_layerset.h:69`).
  pub fn find_joint(
    &self,
    pos: Vec2,
    layer: i32,
    net: Option<NetId>,
  ) -> Option<JointId> {
    self
      .by_key
      .get(&JointKey { pos, net })?
      .iter()
      .copied()
      .find(|id| {
        self
          .arena
          .get(*id)
          .is_some_and(|joint| joint.layers.contains(layer))
      })
  }

  /// The joint at a position, creating or growing it as needed.
  ///
  /// Port of `NODE::touchJoint`, `pcbnew/router/pns_node.cpp:1393`, the
  /// only way a joint is ever created or modified. Three steps:
  ///
  /// 1. if this map has no entry for the key and `root` is given, copy
  ///    every joint the root has under that key into this map (`:1406` to
  ///    `:1412`). The copies get fresh local ids, as KiCad's copies get
  ///    fresh addresses;
  /// 2. merge every local joint under the key whose layers overlap
  ///    `layers` (`:1415` to `:1439`);
  /// 3. return the joint.
  ///
  /// `root` is the root node's map when this map belongs to a branch, and
  /// `None` when this map **is** the root's. It is KiCad's `isRoot()` test
  /// as data.
  ///
  /// # Deviation: the surviving id
  ///
  /// KiCad builds a fresh `JOINT` from `aLayers`, merges every overlapping
  /// joint into it, erases them all and inserts the new one, so every
  /// address under the key changes. Here the **first** overlapping joint
  /// absorbs the others and keeps its [`JointId`]; only the absorbed ones
  /// go stale. The resulting joint is the same value either way: the link
  /// order is the concatenation in bucket order, the layer range is the
  /// hull of `layers` and every merged range, and the locked flag is their
  /// `or`. The key cannot change, because a merge only ever happens among
  /// joints that already share it.
  ///
  /// KiCad reruns the merge loop until a pass finds nothing, but its test
  /// is against the `aLayers` argument and not against the growing joint,
  /// so a single pass finds the same set.
  pub fn touch_joint(
    &mut self,
    pos: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
    root: Option<&JointMap>,
  ) -> JointId {
    let key = JointKey { pos, net };

    if !self.by_key.contains_key(&key)
      && let Some(root) = root
    {
      let copied: Vec<JointId> = root
        .joints_at(pos, net)
        .iter()
        .filter_map(|id| root.arena.get(*id).cloned())
        .map(|joint| self.arena.insert(joint))
        .collect();

      if !copied.is_empty() {
        self.by_key.insert(key, copied);
      }
    }

    let under_key: Vec<JointId> =
      self.by_key.get(&key).cloned().unwrap_or_default();
    let mut overlapping = Vec::new();

    for id in under_key {
      let overlaps = self
        .arena
        .get(id)
        .is_some_and(|joint| layers.overlaps(joint.layers));

      if overlaps {
        overlapping.push(id);
      }
    }

    let Some((&survivor, absorbed)) = overlapping.split_first() else {
      let id = self.arena.insert(Joint::new(pos, layers, net));
      self.by_key.entry(key).or_default().push(id);

      return id;
    };

    let absorbed = absorbed.to_vec();

    if let Some(joint) = self.arena.get_mut(survivor) {
      joint.layers.merge(layers);
    }

    for id in &absorbed {
      let Some(other) = self.arena.remove(*id) else {
        continue;
      };

      if let Some(joint) = self.arena.get_mut(survivor) {
        joint.merge(&other);
      }
    }

    if let Some(bucket) = self.by_key.get_mut(&key) {
      bucket.retain(|id| !absorbed.contains(id));
    }

    survivor
  }

  /// Link an item to the joint at a position.
  ///
  /// Port of `NODE::linkJoint`, `pcbnew/router/pns_node.cpp:1454`. The
  /// caller is expected to have added the item to the index already, which
  /// is the order every `addXxx` helper uses
  /// (`pcbnew/router/pns_node.cpp:601` to `:649`); nothing here reads the
  /// item, so the order only matters for the node's own invariants.
  ///
  /// Returns the joint, where KiCad returns nothing.
  pub fn link_joint(
    &mut self,
    pos: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
    item: ItemId,
    root: Option<&JointMap>,
  ) -> JointId {
    let id = self.touch_joint(pos, layers, net, root);

    if let Some(joint) = self.arena.get_mut(id) {
      joint.link(item);
    }

    id
  }

  /// Unlink an item from the joint at a position.
  ///
  /// Port of `NODE::unlinkJoint`, `pcbnew/router/pns_node.cpp:1463`,
  /// including the fact that it goes through `touchJoint` first, so
  /// unlinking from a position that has no joint yet **creates** one and
  /// immediately leaves it dangling. Returns whether the joint is now
  /// dangling, which KiCad computes and discards.
  pub fn unlink_joint(
    &mut self,
    pos: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
    item: ItemId,
    root: Option<&JointMap>,
  ) -> bool {
    let id = self.touch_joint(pos, layers, net, root);

    self
      .arena
      .get_mut(id)
      .is_some_and(|joint| joint.unlink(item))
  }

  /// Pin or unpin the joint at a position.
  ///
  /// Port of `NODE::LockJoint`, `pcbnew/router/pns_node.cpp:1386`, which
  /// takes the item whose layers and net say which joint is meant; those
  /// two are passed directly here. The shove pins the head endpoints with
  /// it (`pcbnew/router/pns_shove.cpp:2492`).
  pub fn lock_joint(
    &mut self,
    pos: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
    lock: bool,
    root: Option<&JointMap>,
  ) -> JointId {
    let id = self.touch_joint(pos, layers, net, root);

    if let Some(joint) = self.arena.get_mut(id) {
      joint.lock(lock);
    }

    id
  }

  /// Split a joint back into per layer joints after an item leaves it.
  ///
  /// Port of `NODE::rebuildJoint`, `pcbnew/router/pns_node.cpp:870`.
  /// Removing a via or a pad, which bound several layers into one joint,
  /// has to leave the remaining links on their own layers. KiCad takes the
  /// lazy route and says so at `:876`: erase every joint under the tag
  /// that the item's layers overlap, then relink every former link except
  /// the item itself.
  ///
  /// The tag's net is the **item's**, not the joint's (`:881`), and the
  /// position is the joint's (`:884`).
  ///
  /// When the erasure leaves the key with nothing and this map belongs to
  /// a branch, a tombstone is planted (`:911` to `:915`) and the item's own
  /// link is then **not** unlinked (`:926`), because unlinking would go
  /// through [`JointMap::touch_joint`] and undo the tombstone.
  ///
  /// The joints erased here leave the arena, so their [`JointId`]s go
  /// stale; that is exactly what happens to KiCad's addresses.
  ///
  /// # The link order decides the outcome
  ///
  /// The relinking walks `links` in order, and the removed item's own
  /// entry is what leaves a dangling joint behind that the later links
  /// cannot merge into. So the split only comes out per layer when the
  /// removed item is **first** in the link list, which is how a via ends
  /// up there: `addVia` links the joint before any track reaches it
  /// (`pcbnew/router/pns_node.cpp:635`). With the item last, the joints
  /// rebuilt for the earlier links are merged back together by the final
  /// `unlinkJoint` and the layers stay bound. KiCad has the same order
  /// dependence and it is reproduced rather than fixed.
  ///
  /// # Why the joint arrives as a position and a link list
  ///
  /// KiCad passes a `const JOINT*` that `NODE::FindJoint` may have found
  /// in **either** this node's map or the root's
  /// (`pcbnew/router/pns_node.cpp:933`, `:1365`), and only its position
  /// and its link list are read. A [`JointId`] cannot say which arena it
  /// belongs to, so the two values are passed instead;
  /// [`JointMap::rebuild_joint_at`] is the shorthand for the local case.
  pub fn rebuild_joint(
    &mut self,
    arena: &Arena<Item>,
    pos: Vec2,
    links: &[ItemId],
    item: ItemId,
    root: Option<&JointMap>,
  ) {
    let Some(item_ref) = arena.get(item) else {
      return;
    };

    let net = item_ref.net();
    let item_layers = item_ref.layers();
    let key = JointKey { pos, net };

    while let Some(bucket) = self.by_key.get(&key) {
      let found = bucket.iter().copied().enumerate().find(|(_, id)| {
        self
          .arena
          .get(*id)
          .is_some_and(|other| item_layers.overlaps(other.layers))
      });

      let Some((index, found)) = found else {
        break;
      };

      let empty = {
        let bucket = self
          .by_key
          .get_mut(&key)
          .expect("the entry was just looked up");
        bucket.remove(index);

        bucket.is_empty()
      };

      if empty {
        self.by_key.remove(&key);
      }

      self.arena.remove(found);
    }

    let mut completely_erased = false;

    if root.is_some() && !self.by_key.contains_key(&key) {
      self.by_key.insert(key, Vec::new());
      completely_erased = true;
    }

    for link in links {
      if *link != item {
        let Some(link_item) = arena.get(*link) else {
          continue;
        };

        let link_layers = link_item.layers();
        self.link_joint(pos, link_layers, net, *link, root);
      } else if !completely_erased {
        self.unlink_joint(pos, item_layers, net, item, root);
      }
    }
  }

  /// [`JointMap::rebuild_joint`] on a joint of this map.
  ///
  /// The shorthand for the case where `NODE::FindJoint` answered from the
  /// node's own map. A [`JointId`] this map does not know is ignored,
  /// which is KiCad's `if( !aJoint ) return` at
  /// `pcbnew/router/pns_node.cpp:872`.
  pub fn rebuild_joint_at(
    &mut self,
    arena: &Arena<Item>,
    joint: JointId,
    item: ItemId,
    root: Option<&JointMap>,
  ) {
    let Some(found) = self.arena.get(joint) else {
      return;
    };

    let pos = found.key.pos;
    let links = found.links.clone();

    self.rebuild_joint(arena, pos, &links, item, root);
  }

  /// Every joint in a box, on a net, over a layer range, with a link of a
  /// given kind.
  ///
  /// Port of `NODE::QueryJoints`, `pcbnew/router/pns_node.cpp:1777`, minus
  /// the root pass, which is the node's to do. The tests are KiCad's:
  /// the layer ranges have to overlap, the box has to contain the joint's
  /// position, and the joint has to have at least one link matching the
  /// kind mask.
  ///
  /// `net` is an addition: `None` accepts every net, as KiCad does, and
  /// `Some(net)` restricts the walk to one key range. KiCad's two callers
  /// (`pcbnew/router/pns_optimizer.cpp:439`,
  /// `pcbnew/router/pns_component_dragger.cpp:137`) both want every net.
  ///
  /// The result is ordered by joint position, lexicographically, then by
  /// net, where KiCad's is in hash order.
  pub fn query_joints(
    &self,
    arena: &Arena<Item>,
    bbox: Box2,
    net: Option<Option<NetId>>,
    layers: LayerRange,
    kind_mask: Kind,
  ) -> Vec<JointId> {
    let mut found = Vec::new();

    for (key, ids) in &self.by_key {
      if let Some(net) = net
        && key.net != net
      {
        continue;
      }

      if !bbox.contains_point(Vec2L::from(key.pos)) {
        continue;
      }

      for id in ids {
        let Some(joint) = self.arena.get(*id) else {
          continue;
        };

        if !joint.layers.overlaps(layers) {
          continue;
        }

        if joint.link_count(arena, kind_mask) == 0 {
          continue;
        }

        found.push(*id);
      }
    }

    found
  }

  /// The joints a line starts and ends on.
  ///
  /// Port of `NODE::FindLineEnds`, `pcbnew/router/pns_node.cpp:1216`,
  /// which is two `FindJoint` calls at the line's first and last point
  /// with the line's own layers and net, through the `FindJoint( pos,
  /// item )` overload that takes the layer range's start
  /// (`pcbnew/router/pns_node.h:478`).
  ///
  /// KiCad dereferences both results without a check (`:1218`, `:1219`),
  /// so a line whose ends have no joint crashes it. This answers `None`.
  /// `Line` does not exist yet, so the line's four properties are passed
  /// directly; the signature becomes `(&Line)` when it does.
  pub fn find_line_ends(
    &self,
    first_point: Vec2,
    last_point: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
  ) -> Option<(JointId, JointId)> {
    let start = self.find_joint(first_point, layers.start(), net)?;
    let end = self.find_joint(last_point, layers.start(), net)?;

    Some((start, end))
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  use crate::geometry::seg::Seg;
  use crate::geometry::shape::Shape;
  use crate::item::{MarkerFlags, Segment, Solid, Via, ViaType};

  /// The net every item in these tests is on unless it says otherwise.
  const NET: Option<NetId> = Some(NetId(1));

  /// A track on one layer, of a given width.
  fn add_segment(
    arena: &mut Arena<Item>,
    a: Vec2,
    b: Vec2,
    width: i32,
    layer: i32,
  ) -> ItemId {
    let mut item =
      Item::new(0, ItemBody::Segment(Segment::new(Seg::new(a, b), width)));
    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(NET);

    arena.insert(item)
  }

  /// A via spanning a layer range.
  fn add_via(arena: &mut Arena<Item>, pos: Vec2, layers: LayerRange) -> ItemId {
    let via = Via::new(pos, 600, 300, ViaType::Through);
    let mut item = Item::new(0, ItemBody::Via(via));
    item.set_layers_and_flash_all(layers);
    item.set_net(NET);

    arena.insert(item)
  }

  /// A pad on one layer.
  fn add_solid(arena: &mut Arena<Item>, pos: Vec2, layer: i32) -> ItemId {
    let shape = Shape::Circle {
      center: pos,
      radius: 500,
    };
    let mut item = Item::new(0, ItemBody::Solid(Solid::new(shape, pos)));
    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(NET);

    arena.insert(item)
  }

  /// A joint built by hand, for the classification predicates.
  fn joint_with(pos: Vec2, layers: LayerRange, links: &[ItemId]) -> Joint {
    let mut joint = Joint::new(pos, layers, NET);

    for link in links {
      joint.link(*link);
    }

    joint
  }

  /// The box around a point, of a given half size.
  fn around(point: Vec2, half: i32) -> Box2 {
    Box2::from_vec2_corners(
      Vec2::new(point.x - half, point.y - half),
      Vec2::new(point.x + half, point.y + half),
    )
  }

  #[test]
  fn keys_order_by_position_lexicographically_then_by_net() {
    let far = JointKey::new(Vec2::new(0, 1000), None);
    let near = JointKey::new(Vec2::new(1, 0), None);

    // The lexicographic order puts the point with the smaller x first,
    // where an order by squared magnitude would not.
    assert!(far < near);

    let no_net = JointKey::new(Vec2::new(0, 0), None);
    let first_net = JointKey::new(Vec2::new(0, 0), Some(NetId(1)));
    let second_net = JointKey::new(Vec2::new(0, 0), Some(NetId(2)));

    assert!(no_net < first_net);
    assert!(first_net < second_net);
  }

  #[test]
  fn touch_joint_creates_a_joint_and_link_joint_fills_it() {
    let mut arena = Arena::new();
    let track =
      add_segment(&mut arena, Vec2::new(0, 0), Vec2::new(1000, 0), 200, 0);
    let mut joints = JointMap::new();

    let id = joints.link_joint(
      Vec2::new(0, 0),
      LayerRange::single(0),
      NET,
      track,
      None,
    );

    assert_eq!(joints.len(), 1);
    assert!(joints.contains_key(Vec2::new(0, 0), NET));
    assert_eq!(joints.find_joint(Vec2::new(0, 0), 0, NET), Some(id));
    assert_eq!(joints.find_joint(Vec2::new(0, 0), 1, NET), None);
    assert_eq!(joints.find_joint(Vec2::new(1, 0), 0, NET), None);
    assert_eq!(joints.find_joint(Vec2::new(0, 0), 0, Some(NetId(2))), None);

    let joint = joints.get(id).expect("the joint is live");

    assert_eq!(joint.pos(), Vec2::new(0, 0));
    assert_eq!(joint.net(), NET);
    assert_eq!(joint.layers(), LayerRange::single(0));
    assert_eq!(joint.links(), [track]);
    assert!(!joint.is_locked());
  }

  /// The deviation from KiCad this module exists for: the joint that
  /// absorbs the others keeps its handle.
  #[test]
  fn touch_joint_merges_overlapping_ranges_and_keeps_the_id() {
    let mut arena = Arena::new();
    let lower =
      add_segment(&mut arena, Vec2::new(0, 0), Vec2::new(1000, 0), 200, 0);
    let upper =
      add_segment(&mut arena, Vec2::new(0, 0), Vec2::new(0, 1000), 200, 2);
    let bridge = add_via(&mut arena, Vec2::new(0, 0), LayerRange::new(0, 2));
    let pos = Vec2::new(0, 0);
    let mut joints = JointMap::new();

    let first = joints.link_joint(pos, LayerRange::new(0, 1), NET, lower, None);
    let second =
      joints.link_joint(pos, LayerRange::new(1, 2), NET, upper, None);

    assert_eq!(second, first, "the overlapping ranges are one joint");
    assert_eq!(joints.len(), 1);
    assert_eq!(
      joints.get(first).expect("live").layers(),
      LayerRange::new(0, 2)
    );
    assert_eq!(joints.get(first).expect("live").links(), [lower, upper]);

    // A disjoint range is a second joint under the same key.
    let apart =
      joints.link_joint(pos, LayerRange::single(6), NET, bridge, None);

    assert_ne!(apart, first);
    assert_eq!(joints.joints_at(pos, NET), [first, apart]);
    assert_eq!(joints.find_joint(pos, 1, NET), Some(first));
    assert_eq!(joints.find_joint(pos, 6, NET), Some(apart));

    // A range spanning both folds them together, and the first one wins.
    let merged = joints.touch_joint(pos, LayerRange::new(0, 6), NET, None);

    assert_eq!(merged, first);
    assert_eq!(joints.len(), 1);
    assert_eq!(joints.joints_at(pos, NET), [first]);
    assert_eq!(
      joints.get(first).expect("live").layers(),
      LayerRange::new(0, 6)
    );
    assert_eq!(
      joints.get(first).expect("live").links(),
      [lower, upper, bridge]
    );
    assert_eq!(joints.get(apart), None, "the absorbed joint is gone");
  }

  /// `JOINT::Unlink` resets the layer range instead of deleting the
  /// joint, so the position stays occupied and stays invisible.
  #[test]
  fn unlinking_the_last_link_leaves_a_dangling_joint() {
    let mut arena = Arena::new();
    let track =
      add_segment(&mut arena, Vec2::new(0, 0), Vec2::new(1000, 0), 200, 0);
    let pos = Vec2::new(0, 0);
    let mut joints = JointMap::new();

    let id = joints.link_joint(pos, LayerRange::single(0), NET, track, None);
    let dangling =
      joints.unlink_joint(pos, LayerRange::single(0), NET, track, None);

    assert!(dangling);
    assert_eq!(joints.len(), 1, "the joint is emptied, never deleted");
    assert!(joints.contains_key(pos, NET), "so the key stays occupied");
    assert_eq!(joints.find_joint(pos, 0, NET), None, "and stays invisible");
    assert_eq!(
      joints.get(id).expect("live").layers(),
      LayerRange::UNDEFINED
    );
    assert!(joints.get(id).expect("live").links().is_empty());
  }

  /// The whole point of `rebuildJoint`: a via bound three layers into one
  /// joint, and removing it has to hand each track its own joint back.
  #[test]
  fn rebuild_joint_splits_a_via_joint_into_per_layer_joints() {
    let mut arena = Arena::new();
    let pos = Vec2::new(0, 0);
    let layers = LayerRange::new(0, 2);
    let via = add_via(&mut arena, pos, layers);
    let bottom = add_segment(&mut arena, pos, Vec2::new(1000, 0), 200, 0);
    let top = add_segment(&mut arena, pos, Vec2::new(0, 1000), 200, 2);
    let mut joints = JointMap::new();

    // The via is linked first, which is the order `addVia` produces and
    // the order the split depends on.
    let bound = joints.link_joint(pos, layers, NET, via, None);
    joints.link_joint(pos, LayerRange::single(0), NET, bottom, None);
    joints.link_joint(pos, LayerRange::single(2), NET, top, None);

    assert_eq!(joints.len(), 1);
    assert_eq!(joints.get(bound).expect("live").links(), [via, bottom, top]);

    joints.rebuild_joint_at(&arena, bound, via, None);

    let on_bottom =
      joints.find_joint(pos, 0, NET).expect("layer 0 has a joint");
    let on_top = joints.find_joint(pos, 2, NET).expect("layer 2 has a joint");

    assert_ne!(on_bottom, on_top);
    assert_eq!(joints.get(on_bottom).expect("live").links(), [bottom]);
    assert_eq!(joints.get(on_top).expect("live").links(), [top]);
    assert_eq!(
      joints.get(on_bottom).expect("live").layers(),
      LayerRange::single(0)
    );
    assert_eq!(
      joints.find_joint(pos, 1, NET),
      None,
      "nothing binds layer 1"
    );
    assert_eq!(joints.get(bound), None, "the bound joint is gone");
  }

  /// A branch that removes the root's only joint at a position has to
  /// leave a mark, or the node's lookup falls through to the root and
  /// resurrects it.
  #[test]
  fn rebuild_joint_plants_a_tombstone_in_a_branch() {
    let mut arena = Arena::new();
    let pos = Vec2::new(0, 0);
    let layers = LayerRange::new(0, 2);
    let via = add_via(&mut arena, pos, layers);
    let mut root = JointMap::new();
    let in_root = root.link_joint(pos, layers, NET, via, None);

    let mut branch = JointMap::new();
    let links = root.get(in_root).expect("live").links().to_vec();
    branch.rebuild_joint(&arena, pos, &links, via, Some(&root));

    assert!(branch.is_empty(), "the branch holds no joint at all");
    assert!(
      branch.contains_key(pos, NET),
      "but the key is occupied, so the node must not consult the root"
    );
    assert_eq!(branch.find_joint(pos, 0, NET), None);
    assert_eq!(
      root.find_joint(pos, 0, NET),
      Some(in_root),
      "and the root is untouched"
    );

    // Touching the tombstone key does not copy the root's joint in.
    let fresh =
      branch.touch_joint(pos, LayerRange::single(0), NET, Some(&root));

    assert!(branch.get(fresh).expect("live").links().is_empty());
  }

  #[test]
  fn touch_joint_copies_the_root_joints_into_a_branch() {
    let mut arena = Arena::new();
    let pos = Vec2::new(0, 0);
    let track = add_segment(&mut arena, pos, Vec2::new(1000, 0), 200, 0);
    let other = add_segment(&mut arena, pos, Vec2::new(0, 1000), 200, 0);
    let mut root = JointMap::new();
    let in_root = root.link_joint(pos, LayerRange::single(0), NET, track, None);

    let mut branch = JointMap::new();
    let in_branch =
      branch.link_joint(pos, LayerRange::single(0), NET, other, Some(&root));

    // A `JointId` only means anything in the map it came from, so the two
    // handles are not compared: the copy is a joint of the branch's own
    // arena and may well carry the same numbers as the root's.
    assert_eq!(branch.len(), 1);
    assert_eq!(
      branch.get(in_branch).expect("live").links(),
      [track, other],
      "the copy carries the root's links"
    );
    assert_eq!(
      root.get(in_root).expect("live").links(),
      [track],
      "while the root is untouched"
    );
    assert_eq!(root.len(), 1);
  }

  #[test]
  fn lock_joint_pins_the_joint() {
    let mut arena = Arena::new();
    let pos = Vec2::new(0, 0);
    let track = add_segment(&mut arena, pos, Vec2::new(1000, 0), 200, 0);
    let mut joints = JointMap::new();
    let id = joints.link_joint(pos, LayerRange::single(0), NET, track, None);

    assert!(!joints.get(id).expect("live").is_locked());

    let locked = joints.lock_joint(pos, LayerRange::single(0), NET, true, None);

    assert_eq!(locked, id);
    assert!(joints.get(id).expect("live").is_locked());

    joints.lock_joint(pos, LayerRange::single(0), NET, false, None);

    assert!(!joints.get(id).expect("live").is_locked());
  }

  /// The locked flag survives a merge, because `JOINT::Merge` ors it.
  #[test]
  fn a_merge_keeps_the_locked_flag_of_either_side() {
    let mut arena = Arena::new();
    let pos = Vec2::new(0, 0);
    let lower = add_segment(&mut arena, pos, Vec2::new(1000, 0), 200, 0);
    let upper = add_segment(&mut arena, pos, Vec2::new(0, 1000), 200, 4);
    let mut joints = JointMap::new();

    joints.link_joint(pos, LayerRange::single(0), NET, lower, None);
    joints.link_joint(pos, LayerRange::single(4), NET, upper, None);

    let pinned = joints.lock_joint(pos, LayerRange::single(4), NET, true, None);
    let merged = joints.touch_joint(pos, LayerRange::new(0, 4), NET, None);

    assert_ne!(merged, pinned, "the first joint under the key survives");
    assert!(joints.get(merged).expect("live").is_locked());
    assert_eq!(joints.get(merged).expect("live").links(), [lower, upper]);
  }

  #[test]
  fn line_corner_via_and_width_change_predicates() {
    let mut arena = Arena::new();
    let pos = Vec2::new(1000, 0);
    let left = add_segment(&mut arena, Vec2::new(0, 0), pos, 200, 0);
    let right = add_segment(&mut arena, pos, Vec2::new(2000, 0), 200, 0);
    let narrow = add_segment(&mut arena, pos, Vec2::new(1000, 1000), 100, 0);
    let via = add_via(&mut arena, pos, LayerRange::new(0, 2));
    let pad = add_solid(&mut arena, pos, 0);

    let corner = joint_with(pos, LayerRange::single(0), &[left, right]);

    assert!(corner.is_line_corner(&arena, false));
    assert!(!corner.is_stitching_via(&arena));
    assert!(!corner.is_non_fanout_via(&arena));
    assert!(!corner.is_trace_width_change(&arena));
    assert_eq!(corner.next_segment(&arena, left, false), Some(right));
    assert_eq!(corner.via(&arena), None);
    assert_eq!(corner.link_count(&arena, Kind::ANY), 2);
    assert_eq!(corner.link_count(&arena, Kind::SEGMENT), 2);
    assert_eq!(corner.link_count(&arena, Kind::VIA), 0);

    let change = joint_with(pos, LayerRange::single(0), &[left, narrow]);

    assert!(!change.is_line_corner(&arena, false));
    assert!(change.is_trace_width_change(&arena));

    let fanout = joint_with(pos, LayerRange::single(0), &[left, right, narrow]);

    assert!(!fanout.is_line_corner(&arena, false));
    assert_eq!(fanout.next_segment(&arena, left, false), None);

    let stitching = joint_with(pos, LayerRange::new(0, 2), &[via]);

    assert!(stitching.is_stitching_via(&arena));
    assert!(!stitching.is_non_fanout_via(&arena));
    assert_eq!(stitching.via(&arena), Some(via));

    let fanned = joint_with(pos, LayerRange::new(0, 2), &[via, left, right]);

    assert!(fanned.is_non_fanout_via(&arena));
    assert!(!fanned.is_stitching_via(&arena));
    assert_eq!(
      fanned.next_segment(&arena, left, false),
      None,
      "a via ends the line"
    );

    let endpoint = joint_with(pos, LayerRange::single(0), &[left]);

    assert!(endpoint.is_trivial_endpoint(&arena));

    let on_a_pad = joint_with(pos, LayerRange::single(0), &[left, pad]);

    assert!(!on_a_pad.is_line_corner(&arena, false));
    assert_eq!(on_a_pad.next_segment(&arena, left, false), None);
  }

  /// A locked track is not a trivial corner, and the second branch of
  /// `IsLineCorner` only opens when locked segments are allowed and every
  /// other link is a virtual via.
  #[test]
  fn locked_tracks_and_virtual_vias_in_a_line_corner() {
    let mut arena = Arena::new();
    let pos = Vec2::new(1000, 0);
    let left = add_segment(&mut arena, Vec2::new(0, 0), pos, 200, 0);
    let right = add_segment(&mut arena, pos, Vec2::new(2000, 0), 200, 0);
    let virtual_via = add_via(&mut arena, pos, LayerRange::new(0, 2));
    let real_via = add_via(&mut arena, pos, LayerRange::new(0, 2));

    arena.get_mut(left).expect("live").mark(MarkerFlags::LOCKED);
    arena
      .get_mut(virtual_via)
      .expect("live")
      .set_is_virtual(true);

    let corner = joint_with(pos, LayerRange::single(0), &[left, right]);

    assert!(!corner.is_line_corner(&arena, false));
    assert!(corner.is_line_corner(&arena, true));
    assert_eq!(
      corner.next_segment(&arena, right, false),
      None,
      "a locked continuation is not one unless it is allowed"
    );
    assert_eq!(corner.next_segment(&arena, right, true), Some(left));

    let with_vvia =
      joint_with(pos, LayerRange::single(0), &[left, right, virtual_via]);

    assert!(!with_vvia.is_line_corner(&arena, false));
    assert!(with_vvia.is_line_corner(&arena, true));
    assert_eq!(
      with_vvia.next_segment(&arena, right, true),
      Some(left),
      "a virtual via does not end the line when locked segments are allowed"
    );
    assert_eq!(
      with_vvia.next_segment(&arena, right, false),
      None,
      "and does end it otherwise"
    );

    let with_real_via =
      joint_with(pos, LayerRange::single(0), &[left, right, real_via]);

    assert!(!with_real_via.is_line_corner(&arena, true));
  }

  #[test]
  fn query_joints_filters_by_box_net_layers_and_kind() {
    let mut arena = Arena::new();
    let here = Vec2::new(0, 0);
    let far = Vec2::new(50_000, 0);
    let track = add_segment(&mut arena, here, Vec2::new(1000, 0), 200, 0);
    let elsewhere = add_segment(&mut arena, far, Vec2::new(51_000, 0), 200, 0);
    let via = add_via(&mut arena, here, LayerRange::new(0, 2));

    let mut other_net = Item::new(
      0,
      ItemBody::Segment(Segment::new(Seg::new(here, Vec2::new(0, 1000)), 200)),
    );
    other_net.set_layers_and_flash_all(LayerRange::single(0));
    other_net.set_net(Some(NetId(2)));
    let stranger = arena.insert(other_net);

    let mut joints = JointMap::new();
    let on_net =
      joints.link_joint(here, LayerRange::single(0), NET, track, None);
    let with_via =
      joints.link_joint(here, LayerRange::new(0, 2), NET, via, None);
    let away =
      joints.link_joint(far, LayerRange::single(0), NET, elsewhere, None);
    let foreign = joints.link_joint(
      here,
      LayerRange::single(0),
      Some(NetId(2)),
      stranger,
      None,
    );

    assert_eq!(on_net, with_via, "the via joint absorbed the track joint");

    let everything = around(Vec2::new(25_000, 0), 30_000);
    let all_layers = LayerRange::new(0, 8);

    assert_eq!(
      joints.query_joints(&arena, everything, None, all_layers, Kind::ANY),
      vec![on_net, foreign, away],
      "ordered by position, then by net"
    );
    assert_eq!(
      joints.query_joints(&arena, everything, Some(NET), all_layers, Kind::ANY),
      vec![on_net, away]
    );
    assert_eq!(
      joints.query_joints(
        &arena,
        around(here, 10),
        None,
        all_layers,
        Kind::ANY
      ),
      vec![on_net, foreign],
      "the box test is on the joint position"
    );
    assert_eq!(
      joints.query_joints(&arena, everything, None, all_layers, Kind::VIA),
      vec![on_net],
      "only one joint has a via link"
    );
    assert_eq!(
      joints.query_joints(
        &arena,
        everything,
        None,
        LayerRange::single(2),
        Kind::ANY
      ),
      vec![on_net],
      "only the via joint reaches layer 2"
    );
  }

  #[test]
  fn find_line_ends_and_next_segment_walk_a_chain() {
    let mut arena = Arena::new();
    let start = Vec2::new(0, 0);
    let middle = Vec2::new(1000, 0);
    let bend = Vec2::new(2000, 0);
    let end = Vec2::new(3000, 0);
    let layers = LayerRange::single(0);

    let first = add_segment(&mut arena, start, middle, 200, 0);
    let second = add_segment(&mut arena, middle, bend, 200, 0);
    let third = add_segment(&mut arena, bend, end, 200, 0);

    let mut joints = JointMap::new();

    for (item, a, b) in [
      (first, start, middle),
      (second, middle, bend),
      (third, bend, end),
    ] {
      joints.link_joint(a, layers, NET, item, None);
      joints.link_joint(b, layers, NET, item, None);
    }

    let (head, tail) = joints
      .find_line_ends(start, end, layers, NET)
      .expect("both ends have a joint");

    assert_eq!(joints.get(head).expect("live").links(), [first]);
    assert_eq!(joints.get(tail).expect("live").links(), [third]);
    assert!(joints.get(head).expect("live").is_trivial_endpoint(&arena));

    let corner = joints
      .find_joint(middle, 0, NET)
      .expect("the corner has a joint");

    assert!(
      joints
        .get(corner)
        .expect("live")
        .is_line_corner(&arena, false)
    );
    assert_eq!(
      joints
        .get(corner)
        .expect("live")
        .next_segment(&arena, first, false),
      Some(second)
    );
    assert_eq!(
      joints
        .get(head)
        .expect("live")
        .next_segment(&arena, first, false),
      None,
      "the loose end has nothing to continue with"
    );
    assert_eq!(
      joints.find_line_ends(start, Vec2::new(9, 9), layers, NET),
      None,
      "a point with no joint answers None where KiCad crashes"
    );
  }
}
