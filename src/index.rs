// SPDX-License-Identifier: GPL-3.0-or-later

//! The broad phase: one R-tree per copper layer.
//!
//! Port of `PNS::INDEX` (`pcbnew/router/pns_index.h:46`), which is a thin
//! layer over one `SHAPE_INDEX<ITEM*>` per layer
//! (`libs/kimath/include/geometry/shape_index.h:109`) plus a net map and a
//! membership set. Every collision search and every hit test starts here
//! (`pcbnew/router/pns_node.cpp:285`, `:291`, `:581`, `:588`).
//!
//! # Which box is stored and which box is inflated
//!
//! This is the part of KiCad's design that is easiest to get wrong, so it
//! is spelled out:
//!
//! - **insert time**: `SHAPE_INDEX::Add` (`shape_index.h:213`) stores
//!   `boundingBox( item, layer )`, which is `item->Shape( layer )->BBox()`
//!   (`shape_index.h:43`, `:58`). No clearance at all. A shape's own box
//!   already covers its width, so a track's box is its capsule and a via's
//!   box is the circle it has **on that layer**.
//! - **query time**: `SHAPE_INDEX::Query` (`shape_index.h:321`) takes the
//!   query shape's box and inflates it by `aMinDistance`
//!   (`shape_index.h:323` and `:324`) before searching. Every caller in the
//!   router passes the node's `m_maxClearance`
//!   (`pcbnew/router/pns_node.cpp:285`, `:291`, `:581`, `:588`), whose
//!   default is 800000 nm (`pcbnew/router/pns_node.cpp:62`).
//!
//! So the index stores un inflated boxes and the **query** carries the
//! whole clearance budget. That makes `max_clearance` a soundness
//! parameter: a value smaller than the largest clearance a rule can return
//! silently drops candidates before the narrow phase ever sees them
//! (`doc/reference/kicad/02-item-model-and-node.md` section 3.12).
//!
//! # Removal keys on the stored box
//!
//! `SHAPE_INDEX::Remove` (`shape_index.h:241`) recomputes the box from the
//! item's **current** shape, while the tree entry was keyed on the box at
//! insert time. An item whose shape moved between the two calls can
//! therefore not be found, and KiCad's copy on write tree has no full scan
//! fallback, so the removal silently leaves a stale pointer behind
//! (`doc/reference/kicad/01-geometry.md` section 10.4). [`Index`] stores
//! the inserted boxes alongside the handle and removes with those, so a
//! mutated item still comes out. That is the one behavioural fix in this
//! module, and `DESIGN.md` section 4.3 asks for it.
//!
//! # Determinism
//!
//! [`Index::items`] and [`Index::items_in_net`] have a defined order,
//! where KiCad iterates an `unordered_set` and a `std::list` of raw
//! pointers. The order in which [`Index::query_item`] and its siblings
//! hand candidates to the visitor is the R-tree's traversal order: it is
//! reproducible for a given sequence of insertions and removals, but it
//! carries no geometric meaning, so nothing here or downstream may depend
//! on it. The node sorts obstacle candidates by `(distance, uid)`
//! afterwards (`DESIGN.md` section 8). Tests in this module sort before
//! comparing for the same reason.
//!
//! # What is not ported
//!
//! - `SetDeferred` / `BuildSpatialIndex` (`pcbnew/router/pns_index.cpp:55`,
//!   `:61`), the bulk load path `NODE::BeginBulkAdd` uses
//!   (`pcbnew/router/pns_node.cpp:1257`). It is a build time optimisation
//!   for the initial board sync and adds a second, differently ordered
//!   insertion path; it can come back with a measurement behind it.
//! - `SHAPE_INDEX::Accept` and `Reindex` (`shape_index.h:264`, `:299`),
//!   which nothing in the router calls.
//! - The copy on write machinery of `COW_RTREE`
//!   (`libs/kimath/include/geometry/rtree/dynamic_rtree_cow.h:33`). Here
//!   `Clone` copies. `DESIGN.md` section 4.3 and 4.5 make that affordable:
//!   the root index is built once and never cloned, and a branch index
//!   only ever holds the handful of items added during one routing
//!   session.

use std::collections::BTreeMap;

use rstar::{AABB, RTree, RTreeObject};

use crate::arena::Arena;
use crate::geometry::box2::Box2;
use crate::item::{Item, ItemId, LayerRange, NetId};

// ---------------------------------------------------------------------
// The R-tree payload
// ---------------------------------------------------------------------

/// What one layer's R-tree holds.
///
/// KiCad stores a bare `ITEM*` and recomputes the box whenever it needs
/// one (`pcbnew/router/pns_index.h:50`). The box travels with the handle
/// here so that removal cannot miss; see the module documentation.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
struct Entry {
  /// The item this entry stands for.
  id: ItemId,
  /// The box the item had on this layer when it was inserted.
  bbox: Box2,
}

impl RTreeObject for Entry {
  type Envelope = AABB<[i64; 2]>;

  fn envelope(&self) -> Self::Envelope {
    envelope_of(self.bbox)
  }
}

/// The R-tree envelope of a box.
///
/// [`Box2`] is `i64` throughout (`geometry::box2`), so a box inflated by a
/// clearance cannot leave the coordinate range, and `rstar` is happy with
/// any signed integer scalar.
fn envelope_of(bbox: Box2) -> AABB<[i64; 2]> {
  AABB::from_corners([bbox.left(), bbox.top()], [bbox.right(), bbox.bottom()])
}

// ---------------------------------------------------------------------
// The insertion record
// ---------------------------------------------------------------------

/// What [`Index::add`] recorded about one item.
///
/// Everything [`Index::remove`] needs to undo the insertion exactly, even
/// if the item has been mutated or dropped from the arena since.
#[derive(Clone, PartialEq, Debug)]
struct Inserted {
  /// The layer range at insertion time, KiCad's `aItem->Layers()` read
  /// again at removal time (`pcbnew/router/pns_index.cpp:88`).
  layers: LayerRange,
  /// The net at insertion time, KiCad's `aItem->Net()` read again at
  /// removal time (`pcbnew/router/pns_index.cpp:98`).
  net: Option<NetId>,
  /// The box inserted into each layer's tree, `layers.start()` first.
  ///
  /// `None` for a layer the item has no shape on, which is the one case
  /// KiCad cannot survive: `boundingBox` dereferences the null shape
  /// (`shape_index.h:60`).
  boxes: Vec<Option<Box2>>,
}

// ---------------------------------------------------------------------
// Index
// ---------------------------------------------------------------------

/// A spatial index over the items of one node.
///
/// Port of `PNS::INDEX`, `pcbnew/router/pns_index.h:46`. The three
/// containers mirror KiCad's `m_subIndices`, `m_netMap` and `m_allItems`
/// (`pcbnew/router/pns_index.h:154` to `:156`).
#[derive(Clone, Debug, Default)]
pub struct Index {
  /// One R-tree per copper layer, the layer index being the position.
  ///
  /// Port of `std::deque<std::unique_ptr<ITEM_SHAPE_INDEX>> m_subIndices`
  /// (`pcbnew/router/pns_index.h:154`), grown on demand to the end of the
  /// range being inserted (`pcbnew/router/pns_index.cpp:33` to `:39`). An
  /// item spanning `n` layers sits in `n` trees, which is why a through
  /// via is expensive and why a query has to fold duplicate hits.
  trees: Vec<RTree<Entry>>,
  /// The items of each net, in insertion order.
  ///
  /// Port of `std::map<NET_HANDLE, NET_ITEMS_LIST> m_netMap`
  /// (`pcbnew/router/pns_index.h:155`). KiCad never inserts an item whose
  /// net handle is null (`pcbnew/router/pns_index.cpp:50`), so the `None`
  /// key is representable here but never populated and
  /// [`Index::items_in_net`] answers empty for it, exactly as
  /// `GetItemsForNet( nullptr )` returns null
  /// (`doc/reference/kicad/02-item-model-and-node.md` section 5.3).
  net_map: BTreeMap<Option<NetId>, Vec<ItemId>>,
  /// Every indexed item, with the record of what was inserted for it.
  ///
  /// Port of `std::unordered_set<ITEM*> m_allItems`
  /// (`pcbnew/router/pns_index.h:156`), which backs `Contains`, `Size`
  /// and iteration. It is a map rather than a set because the insertion
  /// boxes have to live somewhere, and its keys are the membership set.
  /// A [`BTreeMap`] iterates in [`ItemId`] order, where KiCad's hash set
  /// iterates in address order (`DESIGN.md` section 8).
  items: BTreeMap<ItemId, Inserted>,
}

impl Index {
  /// An empty index.
  ///
  /// Port of `INDEX::INDEX`, `pcbnew/router/pns_index.h:53`, minus the
  /// deferred flag.
  pub fn new() -> Self {
    Self::default()
  }

  /// Insert an item into every layer of its range.
  ///
  /// Port of `INDEX::Add`, `pcbnew/router/pns_index.cpp:28`: grow the per
  /// layer trees to the end of the range, insert the item's box on each
  /// layer of the range, register it, and append it to its net bucket
  /// unless it has no net.
  ///
  /// Three deviations, all documented at their source:
  ///
  /// - a layer the item has no shape on is skipped, where KiCad
  ///   dereferences the null shape (`shape_index.h:60`);
  /// - an item that is already indexed is **not** inserted a second time.
  ///   KiCad would put a second entry in every tree while its membership
  ///   set kept one, so the following removal would leave the extra
  ///   entries behind. A debug build trips an assertion.
  /// - a stale handle, and an undefined layer range, are ignored. KiCad
  ///   asserts on the range (`pcbnew/router/pns_index.cpp:31`) and cannot
  ///   express the first case.
  pub fn add(&mut self, arena: &Arena<Item>, id: ItemId) {
    let Some(item) = arena.get(id) else {
      debug_assert!(false, "indexing a stale item handle");

      return;
    };

    let layers = item.layers();

    debug_assert!(
      layers.is_defined(),
      "an item with an undefined layer range cannot be indexed"
    );

    if !layers.is_defined() {
      return;
    }

    debug_assert!(
      !self.items.contains_key(&id),
      "the item is already in this index"
    );

    if self.items.contains_key(&id) {
      return;
    }

    self.grow_to(layers.end());

    let mut boxes = Vec::new();

    for layer in layers.start()..=layers.end() {
      let bbox = item.shape(layer).and_then(|shape| shape.bbox(0));

      if let (Some(bbox), Some(tree)) = (bbox, self.tree_mut(layer)) {
        tree.insert(Entry { id, bbox });
      }

      boxes.push(bbox);
    }

    let net = item.net();

    self.items.insert(id, Inserted { layers, net, boxes });

    if net.is_some() {
      self.net_map.entry(net).or_default().push(id);
    }
  }

  /// Take an item out of every layer it was inserted on.
  ///
  /// Port of `INDEX::Remove`, `pcbnew/router/pns_index.cpp:86`. It keys on
  /// the layer range, the net and the boxes recorded at insert time rather
  /// than reading them off the item again, so an item that was mutated, or
  /// that has already left the arena, still comes out; see the module
  /// documentation. An item that is not in this index is ignored, which is
  /// what KiCad's "the deque is too short" early return amounts to
  /// (`pcbnew/router/pns_index.cpp:91`).
  pub fn remove(&mut self, id: ItemId) {
    let Some(inserted) = self.items.remove(&id) else {
      return;
    };

    for (offset, bbox) in inserted.boxes.iter().enumerate() {
      let Some(bbox) = *bbox else {
        continue;
      };

      let Ok(offset) = i32::try_from(offset) else {
        continue;
      };

      if let Some(tree) = self.tree_mut(inserted.layers.start() + offset) {
        tree.remove(&Entry { id, bbox });
      }
    }

    if let Some(bucket) = self.net_map.get_mut(&inserted.net) {
      bucket.retain(|entry| *entry != id);
    }
  }

  /// Swap one item for another.
  ///
  /// Port of `INDEX::Replace`, `pcbnew/router/pns_index.cpp:105`, which is
  /// literally a removal followed by an insertion. The two handles may
  /// name the same item, in which case this reindexes it with its current
  /// geometry.
  pub fn replace(
    &mut self,
    old_id: ItemId,
    new_id: ItemId,
    arena: &Arena<Item>,
  ) {
    self.remove(old_id);
    self.add(arena, new_id);
  }

  /// Whether the item is in this index.
  ///
  /// Port of `INDEX::Contains`, `pcbnew/router/pns_index.h:136`.
  pub fn contains(&self, id: ItemId) -> bool {
    self.items.contains_key(&id)
  }

  /// How many items are indexed.
  ///
  /// Port of `INDEX::Size`, `pcbnew/router/pns_index.h:144`. An item on
  /// several layers counts once, as it does in KiCad's membership set.
  pub fn len(&self) -> usize {
    self.items.len()
  }

  /// Whether nothing is indexed.
  pub fn is_empty(&self) -> bool {
    self.items.is_empty()
  }

  /// Every indexed item, in [`ItemId`] order.
  ///
  /// Port of `INDEX::begin` and `INDEX::end`
  /// (`pcbnew/router/pns_index.h:146`), which walk an `unordered_set` and
  /// therefore hand out addresses in an order that changes between runs.
  /// `NODE::GetUpdatedItems` and `NODE::Commit`
  /// (`pcbnew/router/pns_node.cpp:1560`, `:1622`) both iterate it, so the
  /// order reaches the commit diff and has to be reproducible.
  pub fn items(&self) -> impl Iterator<Item = ItemId> + '_ {
    self.items.keys().copied()
  }

  /// The items of one net, in insertion order.
  ///
  /// Port of `INDEX::GetItemsForNet`,
  /// `pcbnew/router/pns_index.cpp:112`, which returns null for a net that
  /// has no bucket; an empty slice says the same thing. Items with no net
  /// are never in a bucket, so `items_in_net( None )` is always empty; see
  /// the field documentation.
  pub fn items_in_net(&self, net: Option<NetId>) -> &[ItemId] {
    self.net_map.get(&net).map_or(&[], Vec::as_slice)
  }

  /// The items on one layer whose stored box meets a query box.
  ///
  /// The collecting form of [`Index::visit_box`], for callers that want
  /// the candidates rather than a visitor. The box arrives **already
  /// inflated** by the node's maximum clearance; see the module
  /// documentation. The order is the R-tree's and carries no meaning.
  pub fn query(&self, layer: i32, bbox: Box2) -> Vec<ItemId> {
    let mut found = Vec::new();

    self.visit_box(layer, bbox, |id, _| {
      found.push(id);

      true
    });

    found
  }

  /// Visit every item on one layer whose stored box meets a query box.
  ///
  /// This is the primitive the other two query forms are built from, and
  /// the port of `SHAPE_INDEX::Query`
  /// (`libs/kimath/include/geometry/shape_index.h:321`) minus the
  /// inflation, which the caller has already applied.
  ///
  /// The visitor is handed the item and the layer the sub index stands
  /// for, which is what KiCad's `LAYER_CONTEXT_SETTER`
  /// (`pcbnew/router/pns_node.h:214`, applied at
  /// `pcbnew/router/pns_index.h:167`) stamps onto the visitor so that
  /// `collideSimple` learns its `aLayer` argument. Returning `false` stops
  /// the search.
  ///
  /// The return value counts the candidates handed to the visitor,
  /// including the one that stopped it, which is KiCad's count as well
  /// (`libs/kimath/include/geometry/rtree/dynamic_rtree_cow.h:809`: the
  /// increment sits before the visitor call). It is a box hit count, not a
  /// collision count, because the exact test lives in the visitor.
  ///
  /// # Deviation: `false` really does stop the search
  ///
  /// `COW_RTREE::Search` documents the same contract
  /// (`dynamic_rtree_cow.h:198`) but does not honour it: `searchImpl`
  /// returns out of the current **leaf** only (`:812`) and the recursion
  /// above it carries on into the sibling children (`:821`), so the
  /// visitor is called again. `DEFAULT_OBSTACLE_VISITOR`
  /// (`pcbnew/router/pns_node.cpp:241`) survives that because it retests
  /// its limit on every call, but it has already inserted the obstacle by
  /// then (`:256`, `:259`), so KiCad's obstacle set can come back longer
  /// than `m_limitCount`. Here the walk stops on the first `false`, which
  /// is what the contract says and what the node should be written
  /// against.
  pub fn visit_box<V>(&self, layer: i32, bbox: Box2, mut visitor: V) -> usize
  where
    V: FnMut(ItemId, i32) -> bool,
  {
    self.visit_layer(layer, bbox, &mut visitor).0
  }

  /// Visit every item on **every** layer whose stored box meets a query
  /// box.
  ///
  /// Port of `INDEX::Query( const SHAPE*, int, Visitor& )`,
  /// `pcbnew/router/pns_index.h:191`, which "treats all layers as
  /// colliding" (`:115`). Its one caller is `NODE::HitTest`
  /// (`pcbnew/router/pns_node.cpp:581`, `:588`), which hit tests a point
  /// with no layer context. An item on several layers is visited once per
  /// layer, so the caller has to fold duplicates.
  pub fn visit_all_layers<V>(&self, bbox: Box2, mut visitor: V) -> usize
  where
    V: FnMut(ItemId, i32) -> bool,
  {
    let mut total = 0;

    for layer in 0..self.trees.len() {
      let Ok(layer) = i32::try_from(layer) else {
        break;
      };

      let (count, keep_going) = self.visit_layer(layer, bbox, &mut visitor);
      total += count;

      if !keep_going {
        break;
      }
    }

    total
  }

  /// Visit every item that could collide with `item`, layer by layer.
  ///
  /// Port of `INDEX::Query( const ITEM*, int, Visitor& )`,
  /// `pcbnew/router/pns_index.h:171`: walk the query item's own layer
  /// range, take its shape on each of those layers, and search **only**
  /// that layer's tree with it. Layer filtering is therefore implicit in
  /// which trees are visited, and a padstack via searches each layer with
  /// the diameter it actually has there. A layer the item has no shape on
  /// is skipped (`pcbnew/router/pns_index.h:184`).
  ///
  /// `max_clearance` is KiCad's `aMinDistance`, the node's
  /// `m_maxClearance` (`pcbnew/router/pns_node.cpp:285`). The query box is
  /// the shape's own box grown by it, which is
  /// `shape_index.h:323` and `:324` spelled out.
  ///
  /// The item does not have to be in any arena: the line placer collides a
  /// head segment that lives on the stack, and KiCad does the same with a
  /// local `SEGMENT` (`pcbnew/router/pns_node.cpp:310`).
  pub fn query_item<V>(
    &self,
    item: &Item,
    max_clearance: i32,
    mut visitor: V,
  ) -> usize
  where
    V: FnMut(ItemId, i32) -> bool,
  {
    let layers = item.layers();
    let mut total = 0;

    if !layers.is_defined() {
      return 0;
    }

    for layer in layers.start()..=layers.end() {
      let Some(bbox) = item.shape(layer).and_then(|shape| shape.bbox(0)) else {
        continue;
      };

      let bbox = bbox.inflate_by(i64::from(max_clearance));
      let (count, keep_going) = self.visit_layer(layer, bbox, &mut visitor);
      total += count;

      if !keep_going {
        break;
      }
    }

    total
  }

  /// One layer's worth of a query.
  ///
  /// Returns how many candidates were handed to the visitor and whether
  /// the search may continue. Port of `INDEX::querySingle`,
  /// `pcbnew/router/pns_index.h:162`.
  fn visit_layer<V>(
    &self,
    layer: i32,
    bbox: Box2,
    visitor: &mut V,
  ) -> (usize, bool)
  where
    V: FnMut(ItemId, i32) -> bool,
  {
    let Some(tree) = self.tree(layer) else {
      return (0, true);
    };

    let mut count = 0;

    for entry in tree.locate_in_envelope_intersecting(envelope_of(bbox)) {
      count += 1;

      if !visitor(entry.id, layer) {
        return (count, false);
      }
    }

    (count, true)
  }

  /// The tree of one layer, if that layer has one.
  fn tree(&self, layer: i32) -> Option<&RTree<Entry>> {
    self.trees.get(usize::try_from(layer).ok()?)
  }

  /// The tree of one layer, mutably.
  fn tree_mut(&mut self, layer: i32) -> Option<&mut RTree<Entry>> {
    self.trees.get_mut(usize::try_from(layer).ok()?)
  }

  /// Grow the tree vector so that `layer` has one.
  ///
  /// Port of the lazy deque growth at
  /// `pcbnew/router/pns_index.cpp:33` to `:39`. A negative layer cannot be
  /// grown to, and neither KiCad's deque nor this vector is ever shrunk:
  /// a branch that inherits a long vector keeps it.
  ///
  /// One tree per layer of the range means an item whose range ends far
  /// past the board's layer count allocates that many trees, exactly as
  /// KiCad's deque does. The widest range this crate can build is
  /// [`LayerRange::all`], `(0, 256)`.
  fn grow_to(&mut self, layer: i32) {
    let Ok(layer) = usize::try_from(layer) else {
      return;
    };

    while self.trees.len() <= layer {
      self.trees.push(RTree::new());
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  use crate::geometry::seg::Seg;
  use crate::geometry::vec2::Vec2;
  use crate::item::{ItemBody, Segment, Solid, StackMode, Via, ViaType};

  /// A track on one layer.
  fn segment(
    a: Vec2,
    b: Vec2,
    width: i32,
    layer: i32,
    net: Option<NetId>,
  ) -> Item {
    let mut item =
      Item::new(0, ItemBody::Segment(Segment::new(Seg::new(a, b), width)));
    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(net);

    item
  }

  /// A via of one diameter, spanning a layer range.
  fn through_via(
    pos: Vec2,
    diameter: i32,
    layers: LayerRange,
    net: Option<NetId>,
  ) -> Item {
    let via = Via::new(pos, diameter, diameter / 2, ViaType::Through);
    let mut item = Item::new(0, ItemBody::Via(via));
    item.set_layers_and_flash_all(layers);
    item.set_net(net);

    item
  }

  /// The square box of a given half size around a point.
  fn around(point: Vec2, half: i32) -> Box2 {
    Box2::from_vec2_corners(
      Vec2::new(point.x - half, point.y - half),
      Vec2::new(point.x + half, point.y + half),
    )
  }

  /// The hits of a box query, in a comparable order.
  fn sorted_query(index: &Index, layer: i32, bbox: Box2) -> Vec<ItemId> {
    let mut found = index.query(layer, bbox);
    found.sort_unstable();

    found
  }

  #[test]
  fn add_registers_an_item_on_its_own_layer_only() {
    let mut arena = Arena::new();
    let first = arena.insert(segment(
      Vec2::new(0, 0),
      Vec2::new(1000, 0),
      200,
      0,
      Some(NetId(1)),
    ));
    let second = arena.insert(segment(
      Vec2::new(0, 0),
      Vec2::new(1000, 0),
      200,
      2,
      Some(NetId(1)),
    ));
    let mut index = Index::new();

    assert!(index.is_empty());

    index.add(&arena, first);
    index.add(&arena, second);

    assert!(index.contains(first));
    assert!(index.contains(second));
    assert_eq!(index.len(), 2);
    assert_eq!(index.items().collect::<Vec<_>>(), vec![first, second]);

    let probe = around(Vec2::new(500, 0), 10);

    assert_eq!(sorted_query(&index, 0, probe), vec![first]);
    assert!(sorted_query(&index, 1, probe).is_empty());
    assert_eq!(sorted_query(&index, 2, probe), vec![second]);
    assert!(sorted_query(&index, 9, probe).is_empty());
  }

  #[test]
  fn remove_takes_the_item_out_of_every_layer() {
    let mut arena = Arena::new();
    let id = arena.insert(through_via(
      Vec2::new(0, 0),
      600,
      LayerRange::new(0, 3),
      Some(NetId(1)),
    ));
    let mut index = Index::new();
    index.add(&arena, id);

    let probe = around(Vec2::new(0, 0), 10);

    for layer in 0..=3 {
      assert_eq!(
        sorted_query(&index, layer, probe),
        vec![id],
        "layer {layer}"
      );
    }

    assert!(sorted_query(&index, 4, probe).is_empty());
    assert_eq!(index.len(), 1);

    index.remove(id);

    assert!(!index.contains(id));
    assert_eq!(index.len(), 0);

    for layer in 0..=3 {
      assert!(
        sorted_query(&index, layer, probe).is_empty(),
        "layer {layer}"
      );
    }
  }

  /// A padstack via has a different box on every layer, and the index has
  /// to store the one that layer's tree was searched with.
  #[test]
  fn a_padstack_via_is_indexed_with_its_per_layer_shape() {
    let layers = LayerRange::new(0, 2);
    let mut via = Via::new(Vec2::new(0, 0), 200, 100, ViaType::Through);
    via.set_stack_mode(StackMode::Custom);
    via.set_diameter(layers, 2, 2000);

    let mut item = Item::new(0, ItemBody::Via(via));
    item.set_layers_and_flash_all(layers);
    item.set_net(Some(NetId(1)));

    let mut arena = Arena::new();
    let id = arena.insert(item);
    let mut index = Index::new();
    index.add(&arena, id);

    // 600 nm out: inside the 2000 nm pad of layer 2, outside the 200 nm
    // pad of layers 0 and 1.
    let probe = around(Vec2::new(600, 0), 10);

    assert!(sorted_query(&index, 0, probe).is_empty());
    assert!(sorted_query(&index, 1, probe).is_empty());
    assert_eq!(sorted_query(&index, 2, probe), vec![id]);

    index.remove(id);

    assert!(sorted_query(&index, 2, probe).is_empty());
  }

  /// The hazard of `doc/reference/kicad/01-geometry.md` section 10.4:
  /// KiCad recomputes the box from the item's current shape and silently
  /// fails to find the entry. This keys on the box it inserted.
  #[test]
  fn remove_succeeds_after_the_shape_has_moved() {
    let mut arena = Arena::new();
    let id = arena.insert(segment(
      Vec2::new(0, 0),
      Vec2::new(1000, 0),
      200,
      0,
      Some(NetId(1)),
    ));
    let mut index = Index::new();
    index.add(&arena, id);

    if let ItemBody::Segment(body) =
      arena.get_mut(id).expect("the item is live").body_mut()
    {
      body.set_ends(Vec2::new(500_000, 500_000), Vec2::new(501_000, 500_000));
    }

    index.remove(id);

    assert!(!index.contains(id));
    assert_eq!(index.len(), 0);
    assert!(
      sorted_query(&index, 0, around(Vec2::new(500, 0), 10)).is_empty(),
      "the entry at the old box is gone"
    );
    assert!(
      sorted_query(&index, 0, around(Vec2::new(500_500, 500_000), 10))
        .is_empty(),
      "and nothing was left at the new one either"
    );
  }

  #[test]
  fn the_net_map_lists_each_net_and_omits_the_null_net() {
    let mut arena = Arena::new();
    let first = arena.insert(segment(
      Vec2::new(0, 0),
      Vec2::new(1000, 0),
      200,
      0,
      Some(NetId(7)),
    ));
    let second = arena.insert(segment(
      Vec2::new(0, 500),
      Vec2::new(1000, 500),
      200,
      0,
      Some(NetId(7)),
    ));
    let orphan = arena.insert(segment(
      Vec2::new(0, 1000),
      Vec2::new(1000, 1000),
      200,
      0,
      None,
    ));
    let mut index = Index::new();
    index.add(&arena, first);
    index.add(&arena, second);
    index.add(&arena, orphan);

    assert_eq!(index.items_in_net(Some(NetId(7))), [first, second]);
    assert!(index.items_in_net(Some(NetId(8))).is_empty());
    assert!(
      index.items_in_net(None).is_empty(),
      "KiCad never puts a netless item in a bucket"
    );
    assert!(
      index.contains(orphan),
      "but it is still a member and still in the trees"
    );
    assert_eq!(
      sorted_query(&index, 0, around(Vec2::new(500, 1000), 10)),
      vec![orphan]
    );

    index.remove(first);

    assert_eq!(index.items_in_net(Some(NetId(7))), [second]);
  }

  /// Both the index and [`Box2::intersects`] are closed interval tests,
  /// so a query box that only touches an item's box is a hit.
  #[test]
  fn a_query_box_that_touches_the_item_box_hits() {
    let mut arena = Arena::new();
    let id = arena.insert(segment(
      Vec2::new(0, 0),
      Vec2::new(1000, 0),
      200,
      0,
      Some(NetId(1)),
    ));
    let mut index = Index::new();
    index.add(&arena, id);

    // The segment's box reaches x = 1100, half the width past its end.
    assert_eq!(
      sorted_query(&index, 0, around(Vec2::new(1200, 0), 100)),
      vec![id]
    );
    assert!(sorted_query(&index, 0, around(Vec2::new(1200, 0), 99)).is_empty());
  }

  #[test]
  fn query_item_visits_each_layer_with_that_layer() {
    let mut arena = Arena::new();
    let mut index = Index::new();
    let mut on_layer = Vec::new();

    for layer in 0..3 {
      let id = arena.insert(segment(
        Vec2::new(-500, 0),
        Vec2::new(500, 0),
        200,
        layer,
        Some(NetId(1)),
      ));
      index.add(&arena, id);
      on_layer.push(id);
    }

    let elsewhere = arena.insert(segment(
      Vec2::new(-500, 90_000),
      Vec2::new(500, 90_000),
      200,
      1,
      Some(NetId(1)),
    ));
    index.add(&arena, elsewhere);

    let probe =
      through_via(Vec2::new(0, 0), 600, LayerRange::new(0, 2), Some(NetId(2)));
    let mut seen = Vec::new();
    let count = index.query_item(&probe, 0, |id, layer| {
      seen.push((id, layer));

      true
    });

    seen.sort_unstable();

    assert_eq!(count, 3);
    assert_eq!(
      seen,
      vec![(on_layer[0], 0), (on_layer[1], 1), (on_layer[2], 2)]
    );
  }

  #[test]
  fn query_item_inflates_the_query_box_by_the_max_clearance() {
    let mut arena = Arena::new();
    let id = arena.insert(segment(
      Vec2::new(2000, 0),
      Vec2::new(3000, 0),
      200,
      0,
      Some(NetId(1)),
    ));
    let mut index = Index::new();
    index.add(&arena, id);

    // The via's box reaches x = 300, the track's starts at x = 1900.
    let probe =
      through_via(Vec2::new(0, 0), 600, LayerRange::single(0), Some(NetId(2)));

    let hits = |max_clearance| {
      let mut count = 0;
      index.query_item(&probe, max_clearance, |_, _| {
        count += 1;

        true
      });

      count
    };

    assert_eq!(hits(1599), 0);
    assert_eq!(hits(1600), 1);
  }

  #[test]
  fn a_visitor_that_answers_false_stops_the_search() {
    let mut arena = Arena::new();
    let mut index = Index::new();

    for offset in 0..4 {
      let id = arena.insert(segment(
        Vec2::new(0, offset * 10),
        Vec2::new(1000, offset * 10),
        200,
        0,
        Some(NetId(1)),
      ));
      index.add(&arena, id);
    }

    let mut seen = 0;
    let count = index.visit_box(0, around(Vec2::new(500, 0), 10), |_, _| {
      seen += 1;

      false
    });

    assert_eq!(seen, 1);
    assert_eq!(count, 1, "the candidate that stopped the search is counted");
  }

  #[test]
  fn visit_all_layers_reports_a_multilayer_item_once_per_layer() {
    let mut arena = Arena::new();
    let via = arena.insert(through_via(
      Vec2::new(0, 0),
      600,
      LayerRange::new(0, 2),
      Some(NetId(1)),
    ));
    let track = arena.insert(segment(
      Vec2::new(-500, 0),
      Vec2::new(500, 0),
      200,
      1,
      Some(NetId(1)),
    ));
    let mut index = Index::new();
    index.add(&arena, via);
    index.add(&arena, track);

    let mut seen = Vec::new();
    let count =
      index.visit_all_layers(around(Vec2::new(0, 0), 10), |id, layer| {
        seen.push((id, layer));

        true
      });

    seen.sort_unstable();

    assert_eq!(count, 4);
    assert_eq!(seen, vec![(via, 0), (via, 1), (via, 2), (track, 1)]);
  }

  #[test]
  fn replace_swaps_one_item_for_another() {
    let mut arena = Arena::new();
    let old = arena.insert(segment(
      Vec2::new(0, 0),
      Vec2::new(1000, 0),
      200,
      0,
      Some(NetId(1)),
    ));
    let new = arena.insert(segment(
      Vec2::new(0, 5000),
      Vec2::new(1000, 5000),
      200,
      0,
      Some(NetId(2)),
    ));
    let mut index = Index::new();
    index.add(&arena, old);

    index.replace(old, new, &arena);

    assert!(!index.contains(old));
    assert!(index.contains(new));
    assert_eq!(index.len(), 1);
    assert!(index.items_in_net(Some(NetId(1))).is_empty());
    assert_eq!(index.items_in_net(Some(NetId(2))), [new]);
    assert!(sorted_query(&index, 0, around(Vec2::new(500, 0), 10)).is_empty());
    assert_eq!(
      sorted_query(&index, 0, around(Vec2::new(500, 5000), 10)),
      vec![new]
    );
  }

  /// A solid with no shape is registered and searchable by net, but it is
  /// in no tree. KiCad dereferences the null shape instead.
  #[test]
  fn an_item_with_no_shape_is_registered_but_not_in_a_tree() {
    let mut item = Item::new(0, ItemBody::Solid(Solid::default()));
    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(Some(NetId(3)));

    let mut arena = Arena::new();
    let id = arena.insert(item);
    let mut index = Index::new();
    index.add(&arena, id);

    assert!(index.contains(id));
    assert_eq!(index.items_in_net(Some(NetId(3))), [id]);
    assert!(index.query(0, Box2::maximum()).is_empty());

    index.remove(id);

    assert!(!index.contains(id));
  }

  #[test]
  fn a_clone_is_independent() {
    let mut arena = Arena::new();
    let shared = arena.insert(segment(
      Vec2::new(0, 0),
      Vec2::new(1000, 0),
      200,
      0,
      Some(NetId(1)),
    ));
    let extra = arena.insert(segment(
      Vec2::new(0, 5000),
      Vec2::new(1000, 5000),
      200,
      0,
      Some(NetId(1)),
    ));
    let mut index = Index::new();
    index.add(&arena, shared);

    let mut branch = index.clone();
    branch.add(&arena, extra);
    branch.remove(shared);

    assert_eq!(index.len(), 1);
    assert!(index.contains(shared));
    assert!(!index.contains(extra));
    assert_eq!(index.items_in_net(Some(NetId(1))), [shared]);
    assert_eq!(
      sorted_query(&index, 0, around(Vec2::new(500, 0), 10)),
      vec![shared]
    );

    assert_eq!(branch.len(), 1);
    assert!(branch.contains(extra));
    assert!(!branch.contains(shared));
    assert_eq!(branch.items_in_net(Some(NetId(1))), [extra]);
    assert!(sorted_query(&branch, 0, around(Vec2::new(500, 0), 10)).is_empty());
  }
}
