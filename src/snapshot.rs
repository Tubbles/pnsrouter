// SPDX-License-Identifier: GPL-3.0-or-later

//! The plain data a host hands the engine to describe a board.
//!
//! Replaces KiCad's `ROUTER_IFACE::SyncWorld`
//! (`pcbnew/router/pns_router.cpp:95`, implemented at
//! `pcbnew/router/pns_kicad_iface.cpp:2292`), which is a push style
//! traversal calling back into a half built [`World`] between
//! `BeginBulkAdd` and `FinalizeBulkAdd`. Note 05 section 7.2 asks for a
//! value the host builds and hands over instead: the world becomes
//! constructible away from the host's own data structures, it is
//! serialisable for the replay harness of `DESIGN.md` section 8, and the
//! "the host called `Add` after `FinalizeBulkAdd`" family of bugs cannot
//! be written.
//!
//! [`World::from_snapshot`] is the whole of the port. It returns the
//! world and a [`HostIndex`], the two way map between the host's own ids
//! and the arena handles the engine works with.
//!
//! # What a host must put in, and what it must leave out
//!
//! Note 05 section 3 walks KiCad's `sync*` helpers object by object; note
//! 05 section 7.6 maps LibrePCB's board objects onto them. The rules that
//! matter for the snapshot's semantics:
//!
//! - **Tracks** become [`WorldGeometry::Segment`], added with the allow
//!   duplicate flag (`pcbnew/router/pns_kicad_iface.cpp:2432`), so a
//!   board with two coincident tracks keeps both.
//! - **Vias** become [`WorldGeometry::Via`] plus a hole drilled from the
//!   via's drill diameter (`:1863`). The hole is a first class item and
//!   not an attribute, because the collision ladder recurses into hole
//!   versus item and hole versus hole explicitly
//!   (`pcbnew/router/pns_item.cpp:146`).
//! - **Pads** become one [`WorldGeometry::Solid`] per distinct padstack
//!   layer (`:1743`), all of them sharing one [`WorldItem::id`], plus a
//!   hole when the pad is drilled (`:1710`). A non plated pad is not
//!   routable (`:1672`), which is what makes
//!   [`crate::router::Router::is_starting_point_routable`] refuse it.
//! - **The board outline** becomes a solid spanning every copper layer,
//!   not routable, of zero width (`:2059` to `:2079`). Its clearance
//!   comes from the edge rule and not from the shape, which is why the
//!   width is dropped and not merely made small.
//! - **Mechanical holes** become [`WorldGeometry::Hole`], which is
//!   `NODE::addHole` (`pcbnew/router/pns_node.cpp:642`): indexed, no
//!   joint, no copper.
//! - **Castellated pad holes** go in [`WorldSnapshot::edge_exclusions`],
//!   the port of `AddEdgeExclusion` (`:2369`).
//!
//! Deliberately **not** synced, matching KiCad and Horizon EDA:
//!
//! - **Filled copper zones and planes.** `syncZone`
//!   (`pcbnew/router/pns_kicad_iface.cpp:1894`) returns immediately
//!   unless the zone is a rule area with keepout parameters, so a pour is
//!   invisible to the router and the host refills it after the commit.
//!   Note 05 section 7.6 says the same for LibrePCB's `BI_Plane`.
//! - **Airwires and ratlines.** The engine computes its own.
//! - **Anything on a non copper layer** other than the board outline and
//!   the margin: silkscreen, solder mask, courtyards, documentation
//!   (note 05 section 3.7). Courtyard driven rules reach the engine
//!   through [`crate::rules::RuleResolver::constraint`] instead.
//! - **Thermal reliefs and zone to pad connection detail.**
//! - **Rule areas**, until a host implements
//!   [`crate::rules::RuleResolver::is_keepout`]. They would arrive as one
//!   non routable compound primitive solid per triangle of the outline
//!   (`:1921`).
//!
//! # Two limits of this revision
//!
//! A drilled solid's hole takes the solid's own layer span, because that
//! is what [`World::add_solid`] gives it. KiCad spans a pad's hole over
//! the whole copper stack whatever the pad's copper span is
//! (`pcbnew/router/pns_kicad_iface.cpp:1710`). A host that needs the
//! wider span emits the hole as its own [`WorldGeometry::Hole`] item.
//!
//! There is no arc body in the crate yet (`DESIGN.md` section 3), so a
//! curved track cannot be described. It arrives with the arcs.

use std::collections::BTreeMap;

use crate::geometry::seg::Seg;
use crate::geometry::shape::Shape;
use crate::geometry::vec2::Vec2;
use crate::item::{
  Hole, HostId, ItemBody, ItemFlags, ItemId, LayerMask, LayerRange,
  MarkerFlags, NetId, Provenance, Segment, Solid, Via, ViaType,
};
use crate::node::World;

// ---------------------------------------------------------------------
// The snapshot
// ---------------------------------------------------------------------

/// Everything the engine knows about a board.
///
/// The replacement for `ROUTER_IFACE::SyncWorld` proposed in note 05
/// section 7.2. Every field is plain data: a host builds one, hands it to
/// [`World::from_snapshot`], and is not called back again except through
/// [`crate::rules::RuleResolver`].
#[derive(Clone, PartialEq, Debug, Default)]
pub struct WorldSnapshot {
  /// How many copper layers the board has.
  ///
  /// Layer indices are dense and zero based, `0 .. copper_layer_count`,
  /// which is KiCad's PNS layer numbering (note 05 section 1.7). KiCad
  /// keeps the count on the host side and the engine never sees it, which
  /// is why `PNS::VIA` cannot tell its own bottom layer from the board's
  /// (`pcbnew/router/pns_kicad_iface.cpp:1797`). It is carried here so
  /// that [`crate::router::Router::switch_layer`] can refuse a layer the
  /// board does not have.
  pub copper_layer_count: u8,

  /// The broad phase inflation radius, in nanometres.
  ///
  /// Port of `NODE::m_maxClearance` (`pcbnew/router/pns_node.cpp:62`).
  /// Every spatial query is inflated by it, so a host that understates it
  /// loses candidates the narrow phase never sees and the router silently
  /// routes through them. It must dominate every clearance the rule
  /// resolver can return: for LibrePCB that is the maximum over the board
  /// rule, every net class rule and every per pad override (note 05
  /// section 7.7).
  pub max_clearance: i32,

  /// Every obstacle, in any order.
  ///
  /// The order does not change the routing: the engine sorts its own
  /// candidates by `(distance, uid)` (`DESIGN.md` section 8), and uids
  /// are handed out in this vector's order, so one snapshot always
  /// produces one world.
  pub items: Vec<WorldItem>,

  /// Regions where a collision with the board outline is forgiven.
  ///
  /// Port of `NODE::AddEdgeExclusion`
  /// (`pcbnew/router/pns_node.cpp:795`), whose only producer is the
  /// castellated pad case of the sync
  /// (`pcbnew/router/pns_kicad_iface.cpp:2369`) and whose only consumer
  /// is the edge cuts rung of the collision ladder
  /// (`pcbnew/router/pns_item.cpp:251`).
  pub edge_exclusions: Vec<Shape>,
}

impl WorldSnapshot {
  /// An empty board of a given size and clearance envelope.
  pub fn new(copper_layer_count: u8, max_clearance: i32) -> Self {
    Self {
      copper_layer_count,
      max_clearance,
      items: Vec::new(),
      edge_exclusions: Vec::new(),
    }
  }
}

/// One obstacle of a board.
///
/// Port of the `WorldItem` proposal of note 05 section 7.2. That sketch's
/// `kind` field is folded into [`WorldItem::geometry`], because the kind
/// and the geometry cannot disagree that way.
#[derive(Clone, PartialEq, Debug)]
pub struct WorldItem {
  /// The host's own handle for the object this came from.
  ///
  /// Port of `ITEM::m_parent`, the `BOARD_ITEM*`
  /// (`pcbnew/router/pns_item.h:317`), as an opaque number. It is what
  /// [`crate::router::CommitDiff`] speaks in, and what preserves an
  /// object's identity across a route.
  ///
  /// **Several items may share one id.** A pad with a custom padstack
  /// becomes one solid per distinct layer
  /// (`pcbnew/router/pns_kicad_iface.cpp:1743`) and a stroke text becomes
  /// one solid per stroke, all carrying the id of the one host object.
  /// [`HostIndex`] is one to many in that direction for exactly this
  /// reason.
  pub id: HostId,

  /// The net, or [`None`] for something that has no net at all.
  ///
  /// A netless obstacle is not the same as a freshly started track: null
  /// means "clearance always applies", where a track that has not reached
  /// anything yet carries a net whose
  /// [`crate::rules::RuleResolver::net_code`] is not positive (note 05
  /// section 3.8).
  pub net: Option<NetId>,

  /// The closed interval of copper layers the item occupies.
  pub layers: LayerRange,

  /// The copper, or the drilled shape for [`WorldGeometry::Hole`].
  pub geometry: WorldGeometry,

  /// The shape drilled through a solid, when it has one.
  ///
  /// Only [`WorldGeometry::Solid`] reads it. A via drills its own hole
  /// from its drill diameter, as `VIA`'s constructor does
  /// (`pcbnew/router/pns_via.h:117`), and a [`WorldGeometry::Hole`] is
  /// already the hole. A round hole is a [`Shape::Circle`], a slot a
  /// [`Shape::Segment`] (`pcbnew/router/pns_hole.cpp:131`).
  pub hole: Option<Shape>,

  /// The four booleans the engine branches on.
  pub flags: WorldItemFlags,

  /// Which layers the copper is actually present on.
  ///
  /// Materialises `ROUTER_IFACE::IsFlashedOnLayer`, which note 05 section
  /// 7.3 argues must not stay a callback: it is a pure function of the
  /// item and a layer with three effects. It suppresses a collision
  /// outright (`pcbnew/router/pns_item.cpp:206`), it shrinks a via's
  /// walkaround hull to the hole (`pcbnew/router/pns_via.cpp:243`), and
  /// it is what makes the "always flashed geometry, decide per layer
  /// later" design of `syncPad` work at all
  /// (`pcbnew/router/pns_kicad_iface.cpp:1716`).
  ///
  /// [`None`] means the mask equals [`WorldItem::layers`], which is the
  /// answer for LibrePCB's first integration and for every board with no
  /// removed annular rings. An explicit empty mask would make the item
  /// collide with nothing, so the two cases are kept visibly apart.
  pub flashed_layers: Option<LayerMask>,
}

impl WorldItem {
  /// An item with no hole, default flags, and a flashing mask that
  /// follows the layers.
  pub fn new(
    id: HostId,
    net: Option<NetId>,
    layers: LayerRange,
    geometry: WorldGeometry,
  ) -> Self {
    Self {
      id,
      net,
      layers,
      geometry,
      hole: None,
      flags: WorldItemFlags::default(),
      flashed_layers: None,
    }
  }
}

/// The geometry of one snapshot item, which fixes its kind.
///
/// The four bodies of [`ItemBody`] a host can supply. There is no line
/// variant because a line is never stored (`DESIGN.md` section 4.2), and
/// no arc variant because there is no arc body yet.
#[derive(Clone, PartialEq, Debug)]
pub enum WorldGeometry {
  /// A straight track. Port of `syncTrack`,
  /// `pcbnew/router/pns_kicad_iface.cpp:1749`.
  Segment {
    /// The centre line.
    seg: Seg,
    /// The full track width in nanometres.
    width: i32,
  },

  /// A plated through, blind or buried via. Port of `syncVia`,
  /// `pcbnew/router/pns_kicad_iface.cpp:1792`.
  ///
  /// The padstack is uniform. KiCad carries a diameter per layer
  /// (`pcbnew/router/pns_via.h:348`); this revision carries one, which is
  /// `STACK_MODE::NORMAL`, the mode KiCad itself forces on every blind
  /// and buried via (`:1797`).
  Via {
    /// The centre.
    pos: Vec2,
    /// The copper diameter in nanometres.
    diameter: i32,
    /// The drill diameter in nanometres. The hole is a circle of half
    /// this, drilled by [`World::add_via`].
    drill: i32,
    /// Through, blind, buried or micro.
    via_type: ViaType,
    /// Whether the via has no net yet. Port of `m_isFree`,
    /// `pcbnew/router/pns_via.h:355`.
    is_free: bool,
  },

  /// A pad, a board outline, a copper graphic, a rule area primitive.
  /// Port of `syncPad` and `syncGraphicalItem`,
  /// `pcbnew/router/pns_kicad_iface.cpp:1619` and `:2047`.
  Solid {
    /// The copper, in board coordinates.
    shape: Shape,
    /// The pad centre a trace snaps to, which for a pad is the shape
    /// position minus the rotated offset (`:1700`).
    pos: Vec2,
    /// The offset of the copper from the centre, read by the optimizer
    /// when it decides whether a breakout may start at the centre
    /// (`pcbnew/router/pns_optimizer.cpp:1123`).
    offset: Vec2,
    /// The pad rotation in degrees.
    orientation_degrees: f64,
    /// The connection points a trace may start on, empty for "the centre
    /// is the only anchor". Only copper graphics populate it in KiCad
    /// (`pcbnew/router/pns_kicad_iface.cpp:2081`).
    anchors: Vec<Vec2>,
  },

  /// A hole with no copper of its own, such as a board mounting hole.
  /// Port of `NODE::addHole`, `pcbnew/router/pns_node.cpp:642`.
  Hole {
    /// The drilled shape.
    shape: Shape,
  },
}

/// The booleans a snapshot item carries.
///
/// Three of them are [`ItemFlags`], the port of `ITEM`'s own booleans;
/// the fourth is the `MK_LOCKED` marker, which KiCad keeps in the marker
/// bitmask rather than beside them (`pcbnew/router/pns_item.h:45`). They
/// are gathered here because from a host's point of view they are one
/// group of four answers about one object.
///
/// `ITEM::m_isVirtual` is deliberately absent: a virtual via is something
/// the engine makes up (`NODE::FixupVirtualVias`), never something a
/// board holds.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct WorldItemFlags {
  /// Whether the user pinned this object in place.
  ///
  /// Port of `MK_LOCKED` (`pcbnew/router/pns_item.h:45`). KiCad sets it
  /// from the board item's own lock flag and from membership in a
  /// generator group that is not being edited
  /// (`pcbnew/router/pns_kicad_iface.cpp:1757`), which is how teardrops
  /// stay put while routing next to them. The placer's loop removal
  /// refuses to delete a locked track
  /// (`pcbnew/router/pns_line_placer.cpp:1864`).
  pub locked: bool,

  /// Whether a trace may start or end on this object.
  ///
  /// Port of `m_routable` (`pcbnew/router/pns_item.h:283`). A non
  /// routable solid gets no joint (`pcbnew/router/pns_node.cpp:612`) and
  /// is skipped by `NODE::AllItemsInNet`. Clear it for board outlines,
  /// graphics, text on copper and non plated pads.
  pub routable: bool,

  /// Whether this is a pad on a pin with no internal connection.
  ///
  /// Port of `m_isFreePad` (`pcbnew/router/pns_item.h:286`). The
  /// clearance ladder exempts such a pad entirely
  /// (`pcbnew/router/pns_item.cpp:193`).
  pub free_pad: bool,

  /// Whether this is one piece of a host object that became several
  /// items.
  ///
  /// Port of `m_isCompoundShapePrimitive`
  /// (`pcbnew/router/pns_item.h:300`). A violation on one piece must not
  /// make the whole object vanish from the preview, which is what
  /// [`crate::router::ViolationMarker::hide_original`] carries
  /// (`pcbnew/router/pns_router.cpp:688`).
  pub compound_primitive: bool,
}

impl Default for WorldItemFlags {
  /// KiCad's defaults: unlocked, routable, not a free pad, not a
  /// compound primitive (`pcbnew/router/pns_item.h:116`).
  fn default() -> Self {
    Self {
      locked: false,
      routable: true,
      free_pad: false,
      compound_primitive: false,
    }
  }
}

// ---------------------------------------------------------------------
// The host index
// ---------------------------------------------------------------------

/// The two way map between a host's ids and the engine's handles.
///
/// A host speaks [`HostId`]; the engine speaks [`ItemId`]. Note 05
/// section 7.2 asks for a generational key rather than a pointer, because
/// KiCad's clearance cache is keyed by raw `PNS::ITEM*`
/// (`pcbnew/router/pns_kicad_iface.cpp:92`) and needs explicit
/// invalidation on every node update to keep a reused address from
/// aliasing a stale entry. Both sides of this map are generational, so
/// that hazard cannot be written.
///
/// The host to engine direction is one to many, because one pad becomes
/// one solid per padstack layer; see [`WorldItem::id`]. Both maps are
/// [`BTreeMap`]s, so every iteration order is the id order and nothing
/// here can make a session non reproducible (`DESIGN.md` section 8).
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct HostIndex {
  /// Every engine item one host object became, in insertion order.
  by_host: BTreeMap<HostId, Vec<ItemId>>,
  /// The host object an engine item came from.
  by_item: BTreeMap<ItemId, HostId>,
}

impl HostIndex {
  /// An empty map.
  pub fn new() -> Self {
    Self::default()
  }

  /// Record that `item` came from `host`.
  ///
  /// An item belongs to one host, so recording it twice moves it to the
  /// second host and leaves the first host's list alone. The router only
  /// ever calls this for a freshly created item, so the case does not
  /// arise there.
  pub fn insert(&mut self, host: HostId, item: ItemId) {
    self.by_host.entry(host).or_default().push(item);
    self.by_item.insert(item, host);
  }

  /// Every engine item one host object became.
  ///
  /// Empty for a host object the snapshot never mentioned.
  pub fn items_of(&self, host: HostId) -> &[ItemId] {
    self.by_host.get(&host).map_or(&[], Vec::as_slice)
  }

  /// The host object an engine item came from, if any.
  ///
  /// [`None`] for an item the engine made up, which is every segment and
  /// via a routing session produces until the host has assigned ids to
  /// them through [`crate::router::Router::assign_host_ids`].
  pub fn host_of(&self, item: ItemId) -> Option<HostId> {
    self.by_item.get(&item).copied()
  }

  /// Every host object in the map, in id order.
  pub fn hosts(&self) -> impl Iterator<Item = HostId> + '_ {
    self.by_host.keys().copied()
  }

  /// How many engine items are mapped.
  pub fn len(&self) -> usize {
    self.by_item.len()
  }

  /// Whether nothing is mapped.
  pub fn is_empty(&self) -> bool {
    self.by_item.is_empty()
  }

  /// Forget one engine item.
  ///
  /// Used when a commit removes an item from the world, so that a later
  /// handle cannot resolve through a dead entry.
  pub fn remove_item(&mut self, item: ItemId) {
    if let Some(host) = self.by_item.remove(&item)
      && let Some(items) = self.by_host.get_mut(&host)
    {
      items.retain(|stored| *stored != item);

      if items.is_empty() {
        self.by_host.remove(&host);
      }
    }
  }
}

// ---------------------------------------------------------------------
// Building a world
// ---------------------------------------------------------------------

impl World {
  /// Build a world out of a host's snapshot.
  ///
  /// Port of `ROUTER::SyncWorld` (`pcbnew/router/pns_router.cpp:95`) and
  /// of the `sync*` helpers' choice of what becomes which item
  /// (`pcbnew/router/pns_kicad_iface.cpp:2292`), as a pure function of
  /// plain data. See the module documentation for what a host must put in
  /// and what it must leave out.
  ///
  /// Everything lands in the root node. Segments are added with the allow
  /// duplicate flag, as KiCad's sync does (`:2432`), so two coincident
  /// tracks both survive; a segment whose two ends are the same point is
  /// still refused by [`World::add_segment`]
  /// (`pcbnew/router/pns_node.cpp:749`) and is then absent from the
  /// returned [`HostIndex`].
  ///
  /// `FixupVirtualVias` (`pcbnew/router/pns_node.cpp:1697`), the last
  /// step of KiCad's sync, is not ported: it exists to give the length
  /// tuner an anchor at every multilayer joint, which is out of scope
  /// (`PLAN.md`), and a virtual via collides with nothing anyway
  /// (`pcbnew/router/pns_node.cpp:273`).
  pub fn from_snapshot(snapshot: &WorldSnapshot) -> (Self, HostIndex) {
    let mut world = Self::new(snapshot.max_clearance);
    let root = world.root();
    let mut index = HostIndex::new();

    for shape in &snapshot.edge_exclusions {
      world.add_edge_exclusion(root, shape.clone());
    }

    for entry in &snapshot.items {
      if let Some(id) = world.add_snapshot_item(entry) {
        index.insert(entry.id, id);
      }
    }

    (world, index)
  }

  /// Store one snapshot item in the root node.
  ///
  /// The body of [`World::from_snapshot`]'s loop, split out so that the
  /// four `add_*` calls sit next to the bodies they take. [`None`] when
  /// the world refused the item, which only [`World::add_segment`] does.
  fn add_snapshot_item(&mut self, entry: &WorldItem) -> Option<ItemId> {
    let root = self.root();
    let body = match &entry.geometry {
      WorldGeometry::Segment { seg, width } => {
        ItemBody::Segment(Segment::new(*seg, *width))
      }
      WorldGeometry::Via {
        pos,
        diameter,
        drill,
        via_type,
        is_free,
      } => {
        let mut via = Via::new(*pos, *diameter, *drill, *via_type);

        via.set_is_free(*is_free);

        ItemBody::Via(via)
      }
      WorldGeometry::Solid {
        shape,
        pos,
        offset,
        orientation_degrees,
        anchors,
      } => {
        let mut solid = Solid::new(shape.clone(), *pos);

        solid.set_offset(*offset);
        solid.set_orientation_degrees(*orientation_degrees);
        solid.set_anchor_points(anchors.clone());

        ItemBody::Solid(solid)
      }
      WorldGeometry::Hole { shape } => ItemBody::Hole(Hole::new(shape.clone())),
    };

    let mut item = self.make_item(body);

    item.set_net(entry.net);
    item.set_layers_and_flash_all(entry.layers);

    if let Some(mask) = entry.flashed_layers {
      item.set_flashed_layers(mask);
    }

    item.set_provenance(Provenance::Board(entry.id));
    item.set_flags(ItemFlags {
      routable: entry.flags.routable,
      is_virtual: false,
      is_free_pad: entry.flags.free_pad,
      is_compound_shape_primitive: entry.flags.compound_primitive,
    });

    if entry.flags.locked {
      item.mark(MarkerFlags::LOCKED);
    }

    Some(match &entry.geometry {
      // :2432, the trailing "allow duplicate" flag.
      WorldGeometry::Segment { .. } => self.add_segment(root, item, true)?,
      WorldGeometry::Via { .. } => self.add_via(root, item),
      WorldGeometry::Solid { .. } => {
        let hole = entry.hole.clone().map(Hole::new);

        self.add_solid(root, item, hole)
      }
      WorldGeometry::Hole { .. } => self.add_hole(root, item),
    })
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::item::{Item, Kind};

  /// A round pad on one layer.
  fn pad(
    id: u64,
    at: Vec2,
    layers: LayerRange,
    net: Option<NetId>,
  ) -> WorldItem {
    WorldItem::new(
      HostId(id),
      net,
      layers,
      WorldGeometry::Solid {
        shape: Shape::circle(at, 400_000),
        pos: at,
        offset: Vec2::new(0, 0),
        orientation_degrees: 0.0,
        anchors: Vec::new(),
      },
    )
  }

  #[test]
  fn a_snapshot_becomes_a_world_and_a_two_way_map() {
    let mut snapshot = WorldSnapshot::new(2, 800_000);

    snapshot.items.push(pad(
      1,
      Vec2::new(0, 0),
      LayerRange::single(0),
      Some(NetId(1)),
    ));
    snapshot.items.push(WorldItem::new(
      HostId(2),
      Some(NetId(1)),
      LayerRange::single(0),
      WorldGeometry::Segment {
        seg: Seg::new(Vec2::new(0, 0), Vec2::new(1_000_000, 0)),
        width: 200_000,
      },
    ));

    let (world, index) = World::from_snapshot(&snapshot);
    let root = world.root();

    assert_eq!(index.len(), 2);
    assert_eq!(index.items_of(HostId(1)).len(), 1);

    let pad_id = index.items_of(HostId(1))[0];

    assert_eq!(index.host_of(pad_id), Some(HostId(1)));
    assert!(
      world
        .item(pad_id)
        .is_some_and(|item| item.of_kind(Kind::SOLID))
    );
    assert_eq!(world.hit_test(root, Vec2::new(0, 0)).len(), 2);
  }

  #[test]
  fn one_host_object_may_become_several_items() {
    let mut snapshot = WorldSnapshot::new(4, 800_000);

    for layer in 0..3 {
      snapshot.items.push(pad(
        7,
        Vec2::new(0, 0),
        LayerRange::single(layer),
        Some(NetId(1)),
      ));
    }

    let (_world, index) = World::from_snapshot(&snapshot);

    assert_eq!(index.items_of(HostId(7)).len(), 3);
    assert_eq!(index.hosts().collect::<Vec<_>>(), vec![HostId(7)]);
  }

  #[test]
  fn a_via_drills_its_own_hole_and_a_pad_takes_the_one_it_is_given() {
    let mut snapshot = WorldSnapshot::new(2, 800_000);
    let mut drilled = pad(1, Vec2::new(0, 0), LayerRange::new(0, 1), None);

    drilled.hole = Some(Shape::circle(Vec2::new(0, 0), 150_000));
    snapshot.items.push(drilled);
    snapshot.items.push(WorldItem::new(
      HostId(2),
      Some(NetId(3)),
      LayerRange::new(0, 1),
      WorldGeometry::Via {
        pos: Vec2::new(2_000_000, 0),
        diameter: 600_000,
        drill: 300_000,
        via_type: ViaType::Through,
        is_free: false,
      },
    ));

    let (world, index) = World::from_snapshot(&snapshot);

    for host in [HostId(1), HostId(2)] {
      let id = index.items_of(host)[0];
      let hole = world.item(id).and_then(Item::hole);

      assert!(hole.is_some(), "{host:?} was not drilled");
      assert!(
        world
          .item(hole.expect("just checked"))
          .is_some_and(|item| item.of_kind(Kind::HOLE))
      );
    }
  }

  #[test]
  fn the_flags_and_the_flashing_mask_reach_the_item() {
    let mut snapshot = WorldSnapshot::new(2, 800_000);
    let mut outline = pad(9, Vec2::new(0, 0), LayerRange::new(0, 1), None);

    outline.flags = WorldItemFlags {
      locked: true,
      routable: false,
      free_pad: false,
      compound_primitive: true,
    };
    outline.flashed_layers = Some(LayerMask::NONE.with(0));
    snapshot.items.push(outline);

    let (world, index) = World::from_snapshot(&snapshot);
    let item = world
      .item(index.items_of(HostId(9))[0])
      .expect("the solid was stored");

    assert!(item.is_locked());
    assert!(!item.is_routable());
    assert!(item.is_compound_shape_primitive());
    assert!(item.is_flashed_on(0));
    assert!(!item.is_flashed_on(1));
    assert_eq!(item.host_id(), Some(HostId(9)));
    assert_eq!(item.source(), Some(HostId(9)));
  }

  #[test]
  fn a_degenerate_segment_is_refused_and_never_reaches_the_map() {
    let mut snapshot = WorldSnapshot::new(2, 800_000);

    snapshot.items.push(WorldItem::new(
      HostId(1),
      None,
      LayerRange::single(0),
      WorldGeometry::Segment {
        seg: Seg::new(Vec2::new(0, 0), Vec2::new(0, 0)),
        width: 200_000,
      },
    ));

    let (_world, index) = World::from_snapshot(&snapshot);

    assert!(index.is_empty());
  }

  #[test]
  fn an_edge_exclusion_survives_the_transfer() {
    let mut snapshot = WorldSnapshot::new(2, 800_000);

    snapshot
      .edge_exclusions
      .push(Shape::circle(Vec2::new(1_000, 0), 5_000));

    let (world, _index) = World::from_snapshot(&snapshot);

    assert!(world.query_edge_exclusions(world.root(), Vec2::new(1_000, 0)));
    assert!(!world.query_edge_exclusions(world.root(), Vec2::new(500_000, 0)));
  }

  #[test]
  fn forgetting_an_item_forgets_its_host_when_it_was_the_last_one() {
    let mut snapshot = WorldSnapshot::new(2, 800_000);

    snapshot
      .items
      .push(pad(1, Vec2::new(0, 0), LayerRange::single(0), None));
    snapshot
      .items
      .push(pad(1, Vec2::new(0, 0), LayerRange::single(1), None));

    let (_world, mut index) = World::from_snapshot(&snapshot);
    let first = index.items_of(HostId(1))[0];

    assert_eq!(index.items_of(HostId(1)).len(), 2);

    index.remove_item(first);
    assert_eq!(index.items_of(HostId(1)).len(), 1);
    assert_eq!(index.host_of(first), None);

    let second = index.items_of(HostId(1))[0];

    index.remove_item(second);
    assert!(index.is_empty());
    assert_eq!(index.hosts().count(), 0);
  }
}
