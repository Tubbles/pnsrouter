// SPDX-License-Identifier: GPL-3.0-or-later

//! The objects a routing world is made of.
//!
//! Port of `PNS::ITEM` (`pcbnew/router/pns_item.h:97`) and its four
//! concrete subclasses that milestone 1 needs, `SOLID`
//! (`pcbnew/router/pns_solid.h:36`), `SEGMENT`
//! (`pcbnew/router/pns_segment.h:38`), `VIA`
//! (`pcbnew/router/pns_via.h:60`) and `HOLE`
//! (`pcbnew/router/pns_hole.h:33`), plus `ARC`
//! (`pcbnew/router/pns_arc.h:37`) and the value types they all carry.
//!
//! # Shape of the port
//!
//! KiCad's item hierarchy is a virtual class tree with a runtime kind tag
//! that the router switches on anyway. Here [`Item`] is one struct with an
//! [`ItemBody`] enum, as `doc/reference/kicad/02-item-model-and-node.md`
//! section 10.1 asks and `DESIGN.md` section 4.1 fixes. What follows from
//! that:
//!
//! - `dynamic_cast`, `dyn_cast` and every `Kind()` switch become a `match`
//!   on [`ItemBody`];
//! - the virtual `Clone` becomes a derived `Clone`, with one uid rule for
//!   every body instead of KiCad's three (see [`Item::uid`]);
//! - `mutable int m_marker` with `const` `Mark()` becomes a plain
//!   `&mut self` setter, so the interior mutability disappears;
//! - `OWNABLE_ITEM::m_owner` is not data here. It answered three questions
//!   in KiCad: who frees this (the arena's job), which node do a line's
//!   links point into (a `NodeId` on `Line`, milestone 2), and is this a
//!   synthesised item ([`Provenance`]).
//!
//! # What an item does not carry
//!
//! The collision test, `ITEM::Collide` and `ITEM::collideSimple`
//! (`pcbnew/router/pns_item.cpp:104`), is not here: it needs the rule
//! resolver and the node, so it belongs to the node module. Everything it
//! reads off an item is exposed: [`Item::layers`], [`Item::net`],
//! [`Item::shape`], [`Item::relevant_shape_layers`], [`Item::is_free_pad`],
//! [`Item::hole`], [`Item::parent_pad_via`] and [`Item::flashed_layers`].

use std::borrow::Cow;
use std::f64::consts::FRAC_1_SQRT_2;
use std::ops::{BitOr, BitOrAssign};

use crate::arena::ArenaId;
use crate::geometry::arc::ShapeArc;
use crate::geometry::box2::Box2;
use crate::geometry::collision::collide_mtv;
use crate::geometry::hull::{
  arc_hull, build_hull_for_primitive_shape, monotone_chain_hull,
  octagonal_hull, segment_hull,
};
use crate::geometry::line_chain::LineChain;
use crate::geometry::seg::Seg;
use crate::geometry::shape::Shape;
use crate::geometry::vec2::Vec2;

/// A handle to an [`Item`] in the world's arena.
///
/// This replaces every `PNS::ITEM*`. It is
/// [`ArenaId`], which is the `{ index: u32, generation: u32 }` pair of
/// `doc/reference/kicad/02-item-model-and-node.md` section 10.1 plus a
/// zero sized type tag, so an item handle cannot be passed where a joint
/// or node handle is expected.
pub type ItemId = ArenaId<Item>;

/// The half of the equilateral octagon chamfer factor that `VIA::Hull`
/// and `HOLE::Hull` spell out.
///
/// Both write `( 2 * cl + width ) * ( 1.0 - M_SQRT1_2 )`
/// (`pcbnew/router/pns_via.cpp:249`, `pcbnew/router/pns_hole.cpp:71`),
/// where [`crate::geometry::hull`] groups the same quantity as
/// `2.0 * ( 1.0 - M_SQRT1_2 )` times a radius sum
/// (`pcbnew/router/pns_utils.cpp:82`). The two groupings agree in real
/// arithmetic and can differ in the last bit of an `f64`, so each site
/// keeps the grouping KiCad wrote there.
const OCTAGON_CHAMFER_HALF_FACTOR: f64 = 1.0 - FRAC_1_SQRT_2;

// ---------------------------------------------------------------------
// Nets and host identity
// ---------------------------------------------------------------------

/// A net, as the host numbers them.
///
/// Port of `typedef void* NET_HANDLE` (`pcbnew/router/pns_item.h:55`).
/// KiCad passes a `NETINFO_ITEM*` through it and the router never
/// dereferences it: it compares handles, hashes them, and uses them as map
/// keys. A `u32` newtype does all of that and is deterministic, which a
/// pointer is not (`DESIGN.md` section 8).
///
/// "No net" is `Option::None`, not a reserved value, and it means one
/// narrow thing: a **non conductive obstacle**, whose clearance always
/// applies and which is never "same net" with anything, not even with
/// another netless item (`pcbnew/router/pns_item.cpp:188` needs both
/// handles equal *and* the head's handle non null). KiCad reaches it for
/// text, dimensions and rule areas; note 05 section 1.8 has the list.
///
/// Two neighbouring cases are **not** `None`:
///
/// - Copper with no net of its own carries a host id anyway. KiCad hands
///   the board's unconnected `NETINFO_ITEM` straight through
///   (`pcbnew/router/pns_kicad_iface.cpp:1691`), so two unconnected pads
///   compare as the same net; a `None` there would make them collide.
/// - A route started in free space carries
///   [`crate::rules::RuleResolver::orphaned_net`], the port of
///   `GetOrphanedNetHandle` (`pcbnew/router/pns_kicad_iface.cpp:3020`).
///   Its net code is not positive, so the topology code still reads it as
///   "no net", but it is a real handle, so the head of such a route does
///   not collide with the tail it already fixed.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct NetId(pub u32);

/// The host's identity for one of its own board objects.
///
/// Port of the `BOARD_ITEM*` that `ITEM::m_parent` and
/// `ITEM::m_sourceItem` hold (`pcbnew/router/pns_item.h:317`, `:319`).
/// The crate never looks inside it; it hands it back in the commit diff so
/// the host can find the object again.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct HostId(pub u64);

// ---------------------------------------------------------------------
// Kind
// ---------------------------------------------------------------------

/// What an item is, as a bitmask.
///
/// Port of `ITEM::PnsKind`, `pcbnew/router/pns_item.h:101`. The values are
/// powers of two because the router tests them with `&`, which is
/// [`Kind::of_kind`].
///
/// A stored item carries exactly one bit. The multi bit values,
/// [`Kind::ANY`] and [`Kind::LINKED_ITEM_MASK`], are masks that only ever
/// appear as a query argument.
///
/// This is a newtype rather than a `bitflags` crate type because the crate
/// carries no runtime dependencies, see `DESIGN.md` section 10.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct Kind(u32);

impl Kind {
  /// No kind at all. Port of `INVALID_T`, `pcbnew/router/pns_item.h:104`.
  ///
  /// No item ever carries it; it is the zero of the mask algebra.
  pub const INVALID: Kind = Kind(0);

  /// A pad, a board outline or any other fixed obstacle.
  /// Port of `SOLID_T`, `pcbnew/router/pns_item.h:105`.
  pub const SOLID: Kind = Kind(1);

  /// A transient view over a run of segments. Port of `LINE_T`,
  /// `pcbnew/router/pns_item.h:106`.
  ///
  /// A line is never stored in a node (`DESIGN.md` section 4.2), so no
  /// [`Item`] carries this bit. It exists because the collision search
  /// masks are written in terms of it.
  pub const LINE: Kind = Kind(2);

  /// A connectivity node of the graph. Port of `JOINT_T`,
  /// `pcbnew/router/pns_item.h:107`.
  ///
  /// Joints live in their own arena (`DESIGN.md` section 4.4), so no
  /// [`Item`] carries this bit either.
  pub const JOINT: Kind = Kind(4);

  /// A straight track. Port of `SEGMENT_T`,
  /// `pcbnew/router/pns_item.h:108`.
  pub const SEGMENT: Kind = Kind(8);

  /// A curved track. Port of `ARC_T`, `pcbnew/router/pns_item.h:109`.
  ///
  /// The body is [`Arc`]; the bit already had KiCad's numeric value
  /// before the body arrived, so no mask in the crate changed with it.
  pub const ARC: Kind = Kind(16);

  /// A via. Port of `VIA_T`, `pcbnew/router/pns_item.h:110`.
  pub const VIA: Kind = Kind(32);

  /// A differential pair. Port of `DIFF_PAIR_T`,
  /// `pcbnew/router/pns_item.h:111`. Out of scope, see `PLAN.md`.
  pub const DIFF_PAIR: Kind = Kind(64);

  /// A drilled hole or a slot. Port of `HOLE_T`,
  /// `pcbnew/router/pns_item.h:112`.
  pub const HOLE: Kind = Kind(128);

  /// Every kind. Port of `ANY_T`, `pcbnew/router/pns_item.h:112`.
  ///
  /// KiCad spells it `0xffff` and not `-1`, and its callers pass `-1`
  /// interchangeably (`pcbnew/router/pns_node.h:119`). Both are the same
  /// mask as far as [`Kind::of_kind`] is concerned, and only this one is
  /// representable here.
  pub const ANY: Kind = Kind(0xffff);

  /// The mask the shove uses to pick out items that can carry a uid.
  ///
  /// Port of `LINKED_ITEM_MASK_T`, `pcbnew/router/pns_item.h:113`. Its one
  /// caller is `pcbnew/router/pns_shove.cpp:910`.
  ///
  /// The name is a leftover: it includes [`Kind::SOLID`] and
  /// [`Kind::HOLE`] even though neither `SOLID` nor `HOLE` derives from
  /// `LINKED_ITEM`, so it must not be used to decide whether a downcast is
  /// safe.
  pub const LINKED_ITEM_MASK: Kind = Kind(1 | 8 | 16 | 32 | 128);

  /// The raw bits.
  pub const fn bits(self) -> u32 {
    self.0
  }

  /// Whether this kind matches a mask.
  ///
  /// Port of `ITEM::OfKind`, `pcbnew/router/pns_item.h:181`, which is
  /// `( aKindMask & m_kind ) != 0`.
  pub const fn of_kind(self, mask: Kind) -> bool {
    (mask.0 & self.0) != 0
  }
}

/// Port of the `|` that builds the kind masks, for instance
/// `pcbnew/router/pns_item.h:113`.
impl BitOr for Kind {
  type Output = Kind;

  fn bitor(self, other: Kind) -> Kind {
    Kind(self.0 | other.0)
  }
}

// ---------------------------------------------------------------------
// Marker flags
// ---------------------------------------------------------------------

/// The transient marks an item can carry.
///
/// Port of `enum LineMarker`, `pcbnew/router/pns_item.h:42`. Bits
/// `1 << 1` and `1 << 2` are unused in this revision and there is no
/// `MK_HOLE`.
///
/// A newtype rather than a `bitflags` crate type, for the reason given on
/// [`Kind`].
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct MarkerFlags(u32);

impl MarkerFlags {
  /// No marks.
  pub const NONE: MarkerFlags = MarkerFlags(0);

  /// This line is the routing head.
  ///
  /// Port of `MK_HEAD`, `pcbnew/router/pns_item.h:43`. The only place that
  /// sets it is the propagation from an already marked line
  /// (`pcbnew/router/pns_shove.cpp:853`); the one site that would seed it
  /// is commented out (`pns_shove.cpp:2500`).
  pub const HEAD: MarkerFlags = MarkerFlags(1 << 0);

  /// A design rule violation, drawn red in the preview.
  ///
  /// Port of `MK_VIOLATION`, `pcbnew/router/pns_item.h:44`, set by
  /// `ROUTER::markViolations` (`pcbnew/router/pns_router.cpp:729`).
  pub const VIOLATION: MarkerFlags = MarkerFlags(1 << 3);

  /// The user locked this item, or it belongs to a generator that is not
  /// being edited.
  ///
  /// Port of `MK_LOCKED`, `pcbnew/router/pns_item.h:45`. The host sets it
  /// during the world sync and the dragger clears it on the item being
  /// dragged (`pcbnew/router/pns_dragger.cpp:331`).
  pub const LOCKED: MarkerFlags = MarkerFlags(1 << 4);

  /// Declared by KiCad and never read or written anywhere in its tree.
  ///
  /// Port of `MK_DP_COUPLED`, `pcbnew/router/pns_item.h:46`. Kept so that
  /// [`MarkerFlags::ALL`] has KiCad's value and so that a future
  /// differential pair port does not silently reuse the bit for something
  /// else. Nothing in this crate sets it either.
  pub const DP_COUPLED: MarkerFlags = MarkerFlags(1 << 5);

  /// Every declared bit.
  ///
  /// This is what `ITEM::Unmark`'s default argument of `-1` clears
  /// (`pcbnew/router/pns_item.h:262`). KiCad's `-1` also clears the two
  /// undeclared bits, which nothing ever sets.
  pub const ALL: MarkerFlags = MarkerFlags(
    Self::HEAD.0 | Self::VIOLATION.0 | Self::LOCKED.0 | Self::DP_COUPLED.0,
  );

  /// The mask `NODE::ClearRanks` clears by default.
  ///
  /// Port of the default argument of
  /// `ClearRanks( int aMarkerMask = MK_HEAD | MK_VIOLATION )`,
  /// `pcbnew/router/pns_node.h:494`.
  pub const CLEARED_BY_CLEAR_RANKS: MarkerFlags =
    MarkerFlags(Self::HEAD.0 | Self::VIOLATION.0);

  /// The raw bits.
  pub const fn bits(self) -> u32 {
    self.0
  }

  /// Whether no bit is set.
  pub const fn is_empty(self) -> bool {
    self.0 == 0
  }

  /// Whether every bit of `other` is set here.
  pub const fn contains(self, other: MarkerFlags) -> bool {
    (self.0 & other.0) == other.0
  }

  /// Whether any bit is set in both.
  ///
  /// This is the port of KiCad's `Marker() & MK_...` test, for instance
  /// `ITEM::IsLocked` (`pcbnew/router/pns_item.h:278`).
  pub const fn intersects(self, other: MarkerFlags) -> bool {
    (self.0 & other.0) != 0
  }

  /// The bits of `other` cleared.
  ///
  /// Port of the `m_marker &= ~aMarker` in `ITEM::Unmark`,
  /// `pcbnew/router/pns_item.h:262`.
  pub const fn remove(self, other: MarkerFlags) -> MarkerFlags {
    MarkerFlags(self.0 & !other.0)
  }
}

impl BitOr for MarkerFlags {
  type Output = MarkerFlags;

  fn bitor(self, other: MarkerFlags) -> MarkerFlags {
    MarkerFlags(self.0 | other.0)
  }
}

impl BitOrAssign for MarkerFlags {
  fn bitor_assign(&mut self, other: MarkerFlags) {
    self.0 |= other.0;
  }
}

// ---------------------------------------------------------------------
// Layers
// ---------------------------------------------------------------------

/// A contiguous run of copper layers, as a closed interval.
///
/// Port of `PNS_LAYER_RANGE`, `pcbnew/router/pns_layerset.h:31`. Layer
/// indices are dense and zero based, 0 being the top copper layer; the
/// mapping from the host's own layer identifiers is host work (note 05
/// section 1.7).
///
/// # Why this is not an `Option`
///
/// `(-1, -1)` is KiCad's undefined range, [`LayerRange::UNDEFINED`], and
/// the router depends on more than "there is no range here":
///
/// - [`LayerRange::overlaps`] answers false whenever **any** endpoint is
///   negative (`:69`), which is what makes a dangling joint, whose range
///   is reset to `(-1, -1)` on its last unlink
///   (`pcbnew/router/pns_joint.h:229`), invisible to `FindJoint`;
/// - [`LayerRange::merge`] treats an undefined range as absorbing (`:98`);
/// - [`LayerRange::intersection`] has an asymmetric special case for a
///   negative end (`:118`) that yields the **other** range's end.
///
/// An `Option<(i32, i32)>` cannot express the third rule, so the sentinel
/// is kept as a documented value.
///
/// # `start <= end` is not an invariant
///
/// [`LayerRange::new`] swaps its arguments when they arrive the wrong way
/// round (`:41`), so a range built through it is ordered. But
/// [`LayerRange::intersection`] assigns the two endpoints separately and
/// can produce `start > end` for two disjoint ranges, exactly as KiCad's
/// does. Nothing normalises that afterwards.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct LayerRange {
  /// The first layer of the interval. Port of `m_start`, `:145`.
  start: i32,
  /// The last layer of the interval, included. Port of `m_end`, `:146`.
  end: i32,
}

impl LayerRange {
  /// The undefined range.
  ///
  /// Port of the default constructor, `pcbnew/router/pns_layerset.h:34`,
  /// which is `(-1, -1)`.
  pub const UNDEFINED: LayerRange = LayerRange { start: -1, end: -1 };

  /// The interval `[start, end]`, swapping the endpoints if they arrive
  /// the wrong way round.
  ///
  /// Port of `PNS_LAYER_RANGE( int, int )`,
  /// `pcbnew/router/pns_layerset.h:39`. The swap is why
  /// `pcbnew/router/pns_kicad_iface.cpp:1660` has to guard against a
  /// degenerate inverted pad range: `(1, 0)` becomes `(0, 1)` and would
  /// index the pad on both outer layers.
  pub fn new(start: i32, end: i32) -> Self {
    if start > end {
      Self {
        start: end,
        end: start,
      }
    } else {
      Self { start, end }
    }
  }

  /// The one layer interval `[layer, layer]`.
  ///
  /// Port of `PNS_LAYER_RANGE( int )`,
  /// `pcbnew/router/pns_layerset.h:48`, which is also what
  /// `ITEM::SetLayer` builds (`pcbnew/router/pns_item.h:215`).
  pub const fn single(layer: i32) -> Self {
    Self {
      start: layer,
      end: layer,
    }
  }

  /// Every layer a board can have.
  ///
  /// Port of `PNS_LAYER_RANGE::All`,
  /// `pcbnew/router/pns_layerset.h:129`, hardcoded as `(0, 256)` with a
  /// "fixme: use layer IDs header" comment. Reproduced verbatim, bound
  /// included.
  pub const fn all() -> Self {
    Self { start: 0, end: 256 }
  }

  /// The first layer. Port of `Start`,
  /// `pcbnew/router/pns_layerset.h:86`.
  pub const fn start(self) -> i32 {
    self.start
  }

  /// The last layer, included. Port of `End`,
  /// `pcbnew/router/pns_layerset.h:91`.
  pub const fn end(self) -> i32 {
    self.end
  }

  /// Whether the range spans more than one layer.
  ///
  /// Port of `IsMultilayer`, `pcbnew/router/pns_layerset.h:81`, which is
  /// `m_start != m_end` and therefore answers **true** for an inverted
  /// range and false for [`LayerRange::UNDEFINED`].
  pub const fn is_multilayer(self) -> bool {
    self.start != self.end
  }

  /// Whether both endpoints are non negative.
  ///
  /// KiCad has no such predicate; it repeats the test inline at
  /// `pcbnew/router/pns_layerset.h:69`, `:76` and `:98`. Named here
  /// because three different rules turn on it.
  pub const fn is_defined(self) -> bool {
    self.start >= 0 && self.end >= 0
  }

  /// Whether the two intervals share a layer.
  ///
  /// Port of `Overlaps( const PNS_LAYER_RANGE& )`,
  /// `pcbnew/router/pns_layerset.h:67`. Any negative endpoint on either
  /// side answers false before the interval test runs.
  pub const fn overlaps(self, other: LayerRange) -> bool {
    if !self.is_defined() || !other.is_defined() {
      return false;
    }

    self.end >= other.start && self.start <= other.end
  }

  /// Whether the interval holds a layer.
  ///
  /// Port of `Overlaps( const int )`,
  /// `pcbnew/router/pns_layerset.h:74`, named for what it does. A
  /// negative layer, or a negative endpoint, answers false.
  pub const fn contains(self, layer: i32) -> bool {
    if !self.is_defined() || layer < 0 {
      return false;
    }

    layer >= self.start && layer <= self.end
  }

  /// Grow the interval to cover the other one.
  ///
  /// Port of `Merge`, `pcbnew/router/pns_layerset.h:96`. An undefined
  /// range is absorbing: merging into one copies the other wholesale,
  /// undefined or not. Otherwise this is a hull and not a union, so the
  /// result can cover layers neither input had.
  pub fn merge(&mut self, other: LayerRange) {
    if !self.is_defined() {
      *self = other;

      return;
    }

    if other.start < self.start {
      self.start = other.start;
    }

    if other.end > self.end {
      self.end = other.end;
    }
  }

  /// The layers the two intervals share.
  ///
  /// Port of `Intersection`, `pcbnew/router/pns_layerset.h:112`. Two
  /// quirks are reproduced:
  ///
  /// - the start is a plain `max`, negative endpoints included, so an
  ///   undefined operand does not make the result undefined;
  /// - a negative end on either side yields the **other** side's end
  ///   (`:118`), rather than an empty result.
  ///
  /// Two disjoint ranges give `start > end`, which is not a range
  /// [`LayerRange::new`] could have built. KiCad has the same hole.
  pub fn intersection(self, other: LayerRange) -> LayerRange {
    let end = if self.end < 0 {
      other.end
    } else if other.end < 0 {
      self.end
    } else {
      self.end.min(other.end)
    };

    LayerRange {
      start: self.start.max(other.start),
      end,
    }
  }
}

impl Default for LayerRange {
  fn default() -> Self {
    Self::UNDEFINED
  }
}

/// The set of copper layers an item's copper is actually present on.
///
/// This is the materialised form of KiCad's `IsFlashedOnLayer` host
/// callback (`pcbnew/router/pns_kicad_iface.cpp:2172`). Note 05 section
/// 7.3 and `DESIGN.md` section 5 both say it must not be a callback: it is
/// a pure function of the item and a layer with three effects, so it lives
/// on the item as data.
///
/// The three effects, all of which the port has to honour:
///
/// - a non flashed item does not collide on that layer at all, the
///   `clearance = -1` at `pcbnew/router/pns_item.cpp:206` and `:210`
///   (node module, next task);
/// - a via that is not flashed on a layer has a hull the size of its
///   **hole** rather than its pad (`pcbnew/router/pns_via.cpp:243`), which
///   is [`Via::hull`];
/// - it is what lets a host sync one always flashed pad geometry and
///   decide per layer afterwards
///   (`pcbnew/router/pns_kicad_iface.cpp:1716`).
///
/// # The 64 layer limit
///
/// One bit per copper layer in a `u64`, so layers `0 ..= 63` are
/// representable. That is twice KiCad's 32 copper layer maximum. A layer
/// outside that range answers "flashed", which is the conservative
/// direction in both places the mask is read: it keeps the collision and
/// keeps the larger hull. A debug build asserts instead, so a host that
/// outgrows the limit finds out in its tests.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct LayerMask(u64);

impl LayerMask {
  /// The highest representable layer index, included.
  pub const MAX_LAYER: i32 = 63;

  /// No layer.
  pub const NONE: LayerMask = LayerMask(0);

  /// Every representable layer.
  pub const ALL: LayerMask = LayerMask(u64::MAX);

  /// The mask that equals a layer span.
  ///
  /// This is the "for LibrePCB's first integration the mask equals the
  /// layer span" case of `DESIGN.md` section 5, and the fallback KiCad
  /// itself uses for a router created via with no board parent
  /// (`pcbnew/router/pns_kicad_iface.cpp:2201`). An undefined range gives
  /// [`LayerMask::NONE`]; layers past [`LayerMask::MAX_LAYER`] are
  /// dropped, and [`LayerMask::is_flashed_on`] answers for them anyway.
  pub fn from_layer_range(layers: LayerRange) -> Self {
    if !layers.is_defined() || layers.start() > layers.end() {
      return Self::NONE;
    }

    let mut mask = 0u64;
    let last = layers.end().min(Self::MAX_LAYER);

    for layer in layers.start()..=last {
      mask |= 1u64 << layer;
    }

    Self(mask)
  }

  /// The raw bits.
  pub const fn bits(self) -> u64 {
    self.0
  }

  /// Add a layer to the mask.
  ///
  /// A layer outside `0 ..= 63` is ignored.
  pub const fn with(self, layer: i32) -> Self {
    if layer < 0 || layer > Self::MAX_LAYER {
      return self;
    }

    Self(self.0 | (1u64 << layer))
  }

  /// Remove a layer from the mask.
  ///
  /// A layer outside `0 ..= 63` is ignored.
  pub const fn without(self, layer: i32) -> Self {
    if layer < 0 || layer > Self::MAX_LAYER {
      return self;
    }

    Self(self.0 & !(1u64 << layer))
  }

  /// Whether the item has copper on that layer.
  ///
  /// Port of `ROUTER_IFACE::IsFlashedOnLayer( const ITEM*, int )`,
  /// `pcbnew/router/pns_kicad_iface.cpp:2168`, including its short circuit
  /// at `:2172`: a negative layer means "no layer context, assume
  /// flashed". A layer past [`LayerMask::MAX_LAYER`] answers the same way,
  /// which is the conservative direction; see the type documentation.
  pub fn is_flashed_on(self, layer: i32) -> bool {
    if layer < 0 {
      return true;
    }

    debug_assert!(
      layer <= Self::MAX_LAYER,
      "layer {layer} is past the flashing mask's 64 layer limit"
    );

    if layer > Self::MAX_LAYER {
      return true;
    }

    (self.0 & (1u64 << layer)) != 0
  }

  /// Whether the item has copper on **any** layer of an interval.
  ///
  /// Port of the layer range overload,
  /// `ROUTER_IFACE::IsFlashedOnLayer( const ITEM*, const PNS_LAYER_RANGE& )`,
  /// `pcbnew/router/pns_kicad_iface.cpp:2204`, which loops the interval
  /// and answers true on the first flashed layer (`:2216`). It is the one
  /// the collision ladder asks (`pcbnew/router/pns_item.cpp:206`,
  /// `:210`), where the per layer form is what a hull asks.
  ///
  /// The three edge cases match [`LayerMask::is_flashed_on`]: an empty
  /// interval answers false, which is KiCad's `test.Start() <= test.End()`
  /// fallback at `:2257`; a negative start means "no layer context, assume
  /// flashed"; and an interval reaching past [`LayerMask::MAX_LAYER`]
  /// answers true, the conservative direction.
  pub fn is_flashed_on_any(self, layers: LayerRange) -> bool {
    if layers.start() > layers.end() {
      return false;
    }

    if layers.start() < 0 {
      return true;
    }

    if layers.end() > Self::MAX_LAYER {
      debug_assert!(
        false,
        "layer {} is past the flashing mask's 64 layer limit",
        layers.end()
      );

      return true;
    }

    let width = layers.end() - layers.start() + 1;
    let span = if width > Self::MAX_LAYER {
      u64::MAX
    } else {
      ((1u64 << width) - 1) << layers.start()
    };

    (self.0 & span) != 0
  }
}

// ---------------------------------------------------------------------
// Provenance, flags and uids
// ---------------------------------------------------------------------

/// Where an item came from.
///
/// Replaces two KiCad mechanisms at once, as
/// `doc/reference/kicad/02-item-model-and-node.md` section 10.1 asks:
///
/// - `ITEM::m_parent`, the `BOARD_ITEM*` that means "there is a one to one
///   mapping between this router item and that host object"
///   (`pcbnew/router/pns_item.h:317`). A null parent is
///   [`Provenance::Synthetic`];
/// - the `bothOwned` test in KiCad's reference clearance resolver
///   (`pcbnew/router/pns_kicad_iface.cpp:868`), which asks the same
///   question through `m_owner` and is the only reader of that field the
///   port has to replace.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum Provenance {
  /// The item mirrors one host object, one to one.
  Board(HostId),
  /// The router made this item up: a head segment, a preview via, a
  /// virtual via anchor.
  #[default]
  Synthetic,
}

/// The booleans `ITEM` carries.
///
/// Port of `m_routable`, `m_isVirtual`, `m_isFreePad` and
/// `m_isCompoundShapePrimitive` (`pcbnew/router/pns_item.h:300`).
///
/// # `m_movable` is not here
///
/// KiCad's fifth boolean, `m_movable` (`pcbnew/router/pns_item.h:325`), is
/// dead: `SOLID` clears it (`pcbnew/router/pns_solid.h:46`), every copy
/// constructor and `Clone` propagates it, and nothing in the tree ever
/// reads it. There is not even an accessor. It is not ported.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct ItemFlags {
  /// Whether a trace may start or end on this item.
  ///
  /// Port of `m_routable` (`pcbnew/router/pns_item.h:283`). A non routable
  /// solid gets no joint (`pcbnew/router/pns_node.cpp:612`) and is skipped
  /// by `NODE::AllItemsInNet`. The host clears it for board outlines,
  /// graphics and text on copper.
  pub routable: bool,

  /// Whether this is a virtual via, KiCad's `VVIA`.
  ///
  /// Port of `m_isVirtual` (`pcbnew/router/pns_item.h:295`). A virtual
  /// item never collides (`pcbnew/router/pns_node.cpp:273`), gets a
  /// clearance of zero (`:148`) and is never reported to the host
  /// (`pcbnew/router/pns_router.cpp:894`).
  pub is_virtual: bool,

  /// Whether this is a pad on a not internally connected pin.
  ///
  /// Port of `m_isFreePad` (`pcbnew/router/pns_item.h:286`). Such a pad
  /// has no net until it is used, so the clearance ladder skips it
  /// (`pcbnew/router/pns_item.cpp:193`).
  pub is_free_pad: bool,

  /// Whether this item is one primitive of a decomposed compound pad.
  ///
  /// Port of `m_isCompoundShapePrimitive`
  /// (`pcbnew/router/pns_item.h:300`), read by `ROUTER::markViolations`
  /// (`pcbnew/router/pns_router.cpp:688`) so that a violation on one
  /// primitive is drawn on the whole pad.
  pub is_compound_shape_primitive: bool,
}

impl Default for ItemFlags {
  /// The defaults of `ITEM( PnsKind )`,
  /// `pcbnew/router/pns_item.h:116`: routable, not virtual, not a free
  /// pad, not a compound primitive.
  fn default() -> Self {
    Self {
      routable: true,
      is_virtual: false,
      is_free_pad: false,
      is_compound_shape_primitive: false,
    }
  }
}

/// The source of [`Item::uid`] values.
///
/// Port of `LINKED_ITEM::genNextUid`,
/// `pcbnew/router/pns_item.cpp:362`, which is a function local static with
/// a "fixme: make atomic" comment, meaning KiCad's uids are process
/// global and depend on how many routing sessions ran before.
///
/// `DESIGN.md` section 8 forbids that: obstacle candidates are tie broken
/// on `(distance, uid)`, so a process global counter would make the router
/// non reproducible. The counter therefore lives in the world, one per
/// routing session, and this type is what the world will hold. Until
/// `node.rs` exists, tests own one directly.
#[derive(Clone, Debug, Default)]
pub struct UidCounter {
  /// The next value to hand out.
  next: u64,
}

impl UidCounter {
  /// A counter starting at zero, as KiCad's static does
  /// (`pcbnew/router/pns_item.cpp:364`).
  pub const fn new() -> Self {
    Self { next: 0 }
  }

  /// The next uid.
  ///
  /// # Panics
  ///
  /// Never in practice: it would take `u64::MAX` items in one session.
  pub fn next_uid(&mut self) -> u64 {
    let uid = self.next;
    self.next = self.next.checked_add(1).expect("uid counter overflowed");

    uid
  }
}

// ---------------------------------------------------------------------
// Segment
// ---------------------------------------------------------------------

/// A straight track of a given width.
///
/// Port of `PNS::SEGMENT`, `pcbnew/router/pns_segment.h:38`. KiCad stores
/// one `SHAPE_SEGMENT` and reads the spine and the width back out of it
/// (`pns_segment.h:146`); the two fields are kept apart here and the
/// capsule is built on demand by [`Segment::shape`], which costs nothing.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Segment {
  /// The spine. Port of `m_seg.GetSeg()`,
  /// `pcbnew/router/pns_segment.h:101`.
  seg: Seg,
  /// The full track width in nanometres, half of it on each side of the
  /// spine. Port of `m_seg.GetWidth()`,
  /// `pcbnew/router/pns_segment.h:96`.
  width: i32,
}

impl Segment {
  /// A segment of a given spine and width.
  ///
  /// Port of `SEGMENT( const SHAPE_SEGMENT&, NET_HANDLE )`,
  /// `pcbnew/router/pns_segment.h:52`, without the net, which lives on
  /// [`Item`].
  pub const fn new(seg: Seg, width: i32) -> Self {
    Self { seg, width }
  }

  /// The spine. Port of `Seg`, `pcbnew/router/pns_segment.h:101`.
  pub const fn seg(&self) -> Seg {
    self.seg
  }

  /// The full width. Port of `Width`,
  /// `pcbnew/router/pns_segment.h:96`.
  pub const fn width(&self) -> i32 {
    self.width
  }

  /// Set the full width. Port of `SetWidth`,
  /// `pcbnew/router/pns_segment.h:91`.
  pub const fn set_width(&mut self, width: i32) {
    self.width = width;
  }

  /// Move both endpoints.
  ///
  /// Port of `SetEnds`, `pcbnew/router/pns_segment.h:111`, used by the
  /// line placer when it splits a segment at a fix point
  /// (`pcbnew/router/pns_line_placer.cpp:1304`).
  pub const fn set_ends(&mut self, a: Vec2, b: Vec2) {
    self.seg = Seg::new(a, b);
  }

  /// Swap the two endpoints.
  ///
  /// Port of `SwapEnds`, `pcbnew/router/pns_segment.h:116`. Nothing in
  /// KiCad's tree calls it; it is here because the port is of the type and
  /// not of its live call sites, and because a joint walk that has to
  /// orient a segment is the obvious future caller.
  pub const fn swap_ends(&mut self) {
    self.seg = Seg::new(self.seg.b, self.seg.a);
  }

  /// The spine as a two point chain.
  ///
  /// Port of `CLine`, `pcbnew/router/pns_segment.h:106`.
  pub fn line(&self) -> LineChain {
    LineChain::from_slice(&[self.seg.a, self.seg.b], false)
  }

  /// The capsule the collision code sees.
  ///
  /// Port of `Shape`, `pcbnew/router/pns_segment.h:86`, which ignores the
  /// layer.
  pub const fn shape(&self) -> Shape {
    Shape::Segment {
      seg: self.seg,
      width: self.width,
    }
  }

  /// The walkaround boundary.
  ///
  /// Port of `SEGMENT::Hull`, `pcbnew/router/pns_line.cpp:668`, which
  /// forwards to `PNS::SegmentHull` (`pcbnew/router/pns_utils.cpp:181`)
  /// with the raw clearance and walkaround thickness. That builder is
  /// where the near degenerate segment corrections live, and it is the one
  /// hull in the family that truncates the half thickness rather than
  /// rounding it up; see [`segment_hull`].
  pub fn hull(&self, clearance: i32, walkaround_thickness: i32) -> LineChain {
    segment_hull(&self.seg, self.width, clearance, walkaround_thickness)
  }

  /// One of the two endpoints.
  ///
  /// Port of `Anchor`, `pcbnew/router/pns_segment.h:125`, which returns
  /// `A` for `n == 0` and `B` for **every** other index.
  pub const fn anchor(&self, n: usize) -> Vec2 {
    if n == 0 { self.seg.a } else { self.seg.b }
  }

  /// Two. Port of `AnchorCount`,
  /// `pcbnew/router/pns_segment.h:133`.
  pub const fn anchor_count(&self) -> usize {
    2
  }
}

// ---------------------------------------------------------------------
// Arc
// ---------------------------------------------------------------------

/// A curved track of a given width.
///
/// Port of `PNS::ARC`, `pcbnew/router/pns_arc.h:37`, whose one data
/// member is a `SHAPE_ARC` (`:119`). The width lives inside the arc here
/// too, because [`ShapeArc`] carries one and KiCad's `SetWidth` and
/// `Width` forward straight to it (`:83`, `:88`); that is the one place
/// where this body differs in shape from [`Segment`], which keeps its
/// width beside the spine.
///
/// Not ported: `CLine()` (`:93`), which returns a 1000 nm polygonisation
/// and has no caller anywhere in KiCad's tree (erratum E20). The three
/// copy constructors at `:51` and `:61` are the callers' job here: an
/// arena [`Item`] carries the net, the layers, the marker and the rank,
/// and every site that builds an arc from a parent sets them itself.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Arc {
  /// The curve. Port of `m_arc`, `pcbnew/router/pns_arc.h:119`.
  arc: ShapeArc,
}

impl Arc {
  /// An arc of a given curve.
  ///
  /// Port of `ARC( const SHAPE_ARC&, NET_HANDLE )`,
  /// `pcbnew/router/pns_arc.h:44`, without the net, which lives on
  /// [`Item`].
  pub const fn new(arc: ShapeArc) -> Self {
    Self { arc }
  }

  /// The curve. Port of `CArc`, `pcbnew/router/pns_arc.h:116`.
  pub const fn arc(&self) -> ShapeArc {
    self.arc
  }

  /// Replace the curve. Port of the mutable `Arc()` accessor,
  /// `pcbnew/router/pns_arc.h:115`, which is how every caller that
  /// rewrites the geometry reaches it.
  pub const fn set_arc(&mut self, arc: ShapeArc) {
    self.arc = arc;
  }

  /// The full width. Port of `Width`,
  /// `pcbnew/router/pns_arc.h:88`.
  pub const fn width(&self) -> i32 {
    self.arc.width()
  }

  /// Set the full width. Port of `SetWidth`,
  /// `pcbnew/router/pns_arc.h:83`.
  pub const fn set_width(&mut self, width: i32) {
    self.arc.set_width(width);
  }

  /// The curve the collision code sees.
  ///
  /// Port of `Shape`, `pcbnew/router/pns_arc.h:78`, which ignores the
  /// layer and hands out the member itself.
  pub const fn shape(&self) -> Shape {
    Shape::Arc(self.arc)
  }

  /// The walkaround boundary.
  ///
  /// Port of `ARC::Hull`, `pcbnew/router/pns_arc.cpp:28`, which forwards
  /// to `PNS::ArcHull` (`pcbnew/router/pns_utils.cpp:71`) with the raw
  /// clearance and walkaround thickness and ignores the layer.
  ///
  /// [`arc_hull`] answers a [`Result`] where KiCad dereferences two empty
  /// optionals (erratum E21), and this is the one caller that has to
  /// decide what an error means. **An error becomes an empty chain**,
  /// which is what a hull of an item with no shape already is: [`Item::hull`]
  /// has no way to say "no hull", `LINE::Walkaround` treats a hull it
  /// cannot intersect as an obstacle it does not have to avoid
  /// (`pcbnew/router/pns_line.cpp:404`), and answering a wrong hull would
  /// be worse than answering none. KiCad cannot reach the case at all:
  /// the collinear mitre needs a polygonisation accuracy no caller
  /// passes, and the degenerate arc needs three coincident points **and**
  /// a combined clearance of zero, see the slice 4 entry of
  /// `doc/log/2026-09-12.md`. The world refuses neither, so the case is
  /// reachable in principle and has to answer something.
  pub fn hull(&self, clearance: i32, walkaround_thickness: i32) -> LineChain {
    arc_hull(&self.arc, clearance, walkaround_thickness).unwrap_or_default()
  }

  /// One of the two endpoints.
  ///
  /// Port of `Anchor`, `pcbnew/router/pns_arc.h:100`, which returns
  /// `GetP0()` for `n == 0` and `GetP1()` for **every** other index, as
  /// [`Segment::anchor`] does.
  pub const fn anchor(&self, n: usize) -> Vec2 {
    if n == 0 {
      self.arc.start()
    } else {
      self.arc.end()
    }
  }

  /// Two. Port of `AnchorCount`,
  /// `pcbnew/router/pns_arc.h:108`.
  pub const fn anchor_count(&self) -> usize {
    2
  }

  /// The area two revisions of one arc cover between them.
  ///
  /// Port of `ARC::ChangedArea`, `pcbnew/router/pns_arc.cpp:51`, the
  /// union of the two bounding boxes. KiCad's `OPT_BOX2I` is always
  /// engaged, so there is nothing to make optional; [`Box2`] is the
  /// crate's own possibly empty box and an arc always has one.
  ///
  /// The only caller is the shove's changed area accumulation, which
  /// reaches arcs in slice 7 of `doc/work/012-arcs.md`.
  pub fn changed_area(&self, other: &Arc) -> Box2 {
    self.arc.bbox(0).merge(other.arc.bbox(0))
  }
}

// ---------------------------------------------------------------------
// Via
// ---------------------------------------------------------------------

/// How many distinct copper diameters a via's padstack has.
///
/// Port of `VIA::STACK_MODE`, `pcbnew/router/pns_via.h:63`.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum StackMode {
  /// One diameter on every layer.
  #[default]
  Normal,
  /// Three diameters: the range's start layer, the inner layers and the
  /// range's end layer.
  ///
  /// "Front" and "back" here mean the via's own first and last layer, not
  /// the board's, so KiCad refuses to use this mode for blind and buried
  /// vias (`pcbnew/router/pns_via.cpp:100`).
  FrontInnerBack,
  /// A diameter per layer.
  Custom,
}

/// What kind of hole a via has.
///
/// Port of `enum class VIATYPE`, `pcbnew/pcb_track_types.h:36`, values
/// included. The router core only ever passes it through: the only test on
/// it outside the host is `ROUTER::SyncWorld`'s through hole check
/// (`pcbnew/router/pns_router.h:146`).
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum ViaType {
  /// Not yet used. `VIATYPE::NOT_DEFINED`.
  NotDefined = 0,
  /// From an outer layer to its neighbouring inner layer.
  /// `VIATYPE::MICROVIA`.
  MicroVia = 1,
  /// From an outer layer to an inner one. `VIATYPE::BLIND`.
  Blind = 2,
  /// Between two inner layers. `VIATYPE::BURIED`.
  Buried = 3,
  /// All the way through the board. `VIATYPE::THROUGH`.
  #[default]
  Through = 4,
}

/// A via.
///
/// Port of `PNS::VIA`, `pcbnew/router/pns_via.h:60`.
///
/// # What this revision stores, and what is kept
///
/// KiCad holds two parallel maps, `m_diameters` and `m_shapes`, keyed by
/// the padstack layer key. The second is derived: every mutator keeps
/// `m_shapes[layer]` a circle of `m_diameters[layer] / 2` centred on
/// `m_pos` (`pcbnew/router/pns_via.h:240`, `:242`, `:213`). Only the
/// diameters are stored here and [`Via::shape`] builds the circle, which
/// removes the one way for the two to drift apart. KiCad's default
/// constructor is exactly that drift: it fills `m_diameters` and leaves
/// `m_shapes` empty, so `Shape()` fails its `wxCHECK`
/// (`pcbnew/router/pns_via.h:305`).
///
/// Left out, with reasons:
///
/// - `m_holeLayers` (`pcbnew/router/pns_via.h:357`) only ever forwards to
///   `m_hole->SetLayers` (`pcbnew/router/pns_via.cpp:92`). The hole is a
///   separate arena [`Item`] here and carries its own [`LayerRange`].
/// - `m_unconnectedLayerMode` (`:354`) feeds `ConnectsLayer`
///   (`pcbnew/router/pns_via.cpp:78`), which the router core never calls:
///   its only readers are the host's flashing predicate
///   (`pcbnew/router/pns_kicad_iface.cpp:2199`) and its commit path
///   (`:2250`). Flashing is [`Item::flashed_layers`] here, so the mode has
///   nowhere to be read.
/// - `m_secondaryDrill`, `m_secondaryHoleLayers`, `m_primaryPostMachining`
///   and `m_secondaryPostMachining` (`:359` to `:363`) are written by the
///   host sync and read back by the host commit, and by nothing else. They
///   are host state that happens to be parked on the router's via.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Via {
  /// The centre. Port of `m_pos`, `pcbnew/router/pns_via.h:352`.
  pos: Vec2,
  /// How many distinct diameters the padstack has. Port of `m_stackMode`,
  /// `pcbnew/router/pns_via.h:345`.
  stack_mode: StackMode,
  /// The copper diameter per padstack layer key, sorted by key.
  ///
  /// Port of `m_diameters`, `pcbnew/router/pns_via.h:348`, which is a
  /// `std::map<int, int>`. A sorted vector keeps the iteration order
  /// deterministic without a dependency (`DESIGN.md` sections 8 and 10)
  /// and is faster for the handful of entries a padstack has.
  diameters: Vec<(i32, i32)>,
  /// The drill diameter. Port of `m_drill`,
  /// `pcbnew/router/pns_via.h:351`.
  drill: i32,
  /// Port of `m_viaType`, `pcbnew/router/pns_via.h:353`.
  via_type: ViaType,
  /// Whether the via has no net yet. Port of `m_isFree`,
  /// `pcbnew/router/pns_via.h:355`.
  is_free: bool,
}

impl Via {
  /// The padstack key of the "same on every layer" diameter.
  ///
  /// Port of `VIA::ALL_LAYERS`, `pcbnew/router/pns_via.h:78`. It
  /// deliberately collides with the real layer index 0.
  pub const ALL_LAYERS: i32 = 0;

  /// The padstack key of the inner layers' diameter.
  ///
  /// Port of `VIA::INNER_LAYERS`, `pcbnew/router/pns_via.h:79`. It
  /// deliberately collides with the real layer index 1.
  pub const INNER_LAYERS: i32 = 1;

  /// A via of one diameter on every layer.
  ///
  /// Port of `VIA( const VECTOR2I&, const PNS_LAYER_RANGE&, int, int,
  /// NET_HANDLE, VIATYPE )`, `pcbnew/router/pns_via.h:100`, minus the net
  /// and the layer range, which live on [`Item`], and minus the hole,
  /// which is a separate arena item the world adds alongside
  /// (`pcbnew/router/pns_node.cpp:626`).
  pub fn new(pos: Vec2, diameter: i32, drill: i32, via_type: ViaType) -> Self {
    Self {
      pos,
      stack_mode: StackMode::Normal,
      diameters: vec![(Self::ALL_LAYERS, diameter)],
      drill,
      via_type,
      is_free: false,
    }
  }

  /// The centre. Port of `Pos`, `pcbnew/router/pns_via.h:206`.
  pub const fn pos(&self) -> Vec2 {
    self.pos
  }

  /// Move the via.
  ///
  /// Port of `SetPos`, `pcbnew/router/pns_via.h:208`, minus the two
  /// things KiCad does inline because it owns them: the per layer circles
  /// are derived here, and the hole is a separate arena item, so the
  /// caller has to follow this with `hole.set_center( via.pos() )`.
  pub const fn set_pos(&mut self, pos: Vec2) {
    self.pos = pos;
  }

  /// Port of `StackMode`, `pcbnew/router/pns_via.h:196`.
  pub const fn stack_mode(&self) -> StackMode {
    self.stack_mode
  }

  /// Port of `SetStackMode`, `pcbnew/router/pns_via.cpp:96`, minus the
  /// assertion that `FRONT_INNER_BACK` is never used on a blind or buried
  /// via (`:100`), which is a host obligation and cannot be checked here
  /// because the layer range lives on [`Item`].
  pub const fn set_stack_mode(&mut self, stack_mode: StackMode) {
    self.stack_mode = stack_mode;
  }

  /// Port of `ViaType`, `pcbnew/router/pns_via.h:219`.
  pub const fn via_type(&self) -> ViaType {
    self.via_type
  }

  /// Port of `SetViaType`, `pcbnew/router/pns_via.h:220`.
  pub const fn set_via_type(&mut self, via_type: ViaType) {
    self.via_type = via_type;
  }

  /// The drill diameter. Port of `Drill`,
  /// `pcbnew/router/pns_via.h:247`.
  pub const fn drill(&self) -> i32 {
    self.drill
  }

  /// Set the drill diameter.
  ///
  /// Port of `SetDrill`, `pcbnew/router/pns_via.h:249`. KiCad also
  /// resizes the hole it owns to `m_drill / 2`; here the hole is a
  /// separate arena item, so the caller has to follow this with
  /// `hole.set_radius( via.drill() / 2 )`.
  pub const fn set_drill(&mut self, drill: i32) {
    self.drill = drill;
  }

  /// Whether the via has no net yet.
  ///
  /// Port of `IsFree`, `pcbnew/router/pns_via.h:294`. Only the host reads
  /// it in KiCad (`pcbnew/router/pns_kicad_iface.cpp:2701`), and it is
  /// carried so that the commit diff can hand it back.
  pub const fn is_free(&self) -> bool {
    self.is_free
  }

  /// Port of `SetIsFree`, `pcbnew/router/pns_via.h:295`.
  pub const fn set_is_free(&mut self, is_free: bool) {
    self.is_free = is_free;
  }

  /// The padstack key a layer resolves to.
  ///
  /// Port of `EffectiveLayer`, `pcbnew/router/pns_via.cpp:33`. `layers` is
  /// KiCad's `m_layers`, which lives on [`Item`] here.
  ///
  /// - [`StackMode::Normal`]: always [`Via::ALL_LAYERS`];
  /// - [`StackMode::FrontInnerBack`]: the layer itself at either end of
  ///   the range, otherwise the first inner layer if there is one, else
  ///   the start layer;
  /// - [`StackMode::Custom`]: the layer itself if the range holds it, else
  ///   the start layer.
  pub fn effective_layer(&self, layers: LayerRange, layer: i32) -> i32 {
    match self.stack_mode {
      StackMode::Normal => Self::ALL_LAYERS,
      StackMode::FrontInnerBack => {
        if layer == layers.start() || layer == layers.end() {
          return layer;
        }

        if layers.start() + 1 < layers.end() {
          return layers.start() + 1;
        }

        layers.start()
      }
      StackMode::Custom => {
        if layers.contains(layer) {
          layer
        } else {
          layers.start()
        }
      }
    }
  }

  /// The layers on which the via has a distinct shape.
  ///
  /// Port of `VIA::UniqueShapeLayers`,
  /// `pcbnew/router/pns_via.cpp:56`. The [`StackMode::FrontInnerBack`]
  /// answer can hold duplicates when the range ends on layer 0 or 1,
  /// exactly as KiCad's does;
  /// [`Item::relevant_shape_layers`] is where they are folded away.
  pub fn unique_shape_layers(&self, layers: LayerRange) -> Vec<i32> {
    match self.stack_mode {
      StackMode::Normal => vec![Self::ALL_LAYERS],
      StackMode::FrontInnerBack => {
        vec![Self::ALL_LAYERS, Self::INNER_LAYERS, layers.end()]
      }
      StackMode::Custom => (layers.start()..=layers.end()).collect(),
    }
  }

  /// The copper diameter on a layer.
  ///
  /// Port of `Diameter`, `pcbnew/router/pns_via.h:227`, including its
  /// `wxCHECK` fallback to `m_diameters.begin()->second` when the padstack
  /// has no entry for the resolved key. `std::map` is ordered, so that
  /// fallback is the entry with the smallest key, which is what the sorted
  /// vector's first element is.
  ///
  /// # Panics
  ///
  /// When the padstack is empty, where KiCad dereferences `end()`.
  /// [`Via::new`] always seeds one entry, so this needs a caller that
  /// removed it.
  pub fn diameter(&self, layers: LayerRange, layer: i32) -> i32 {
    let key = self.effective_layer(layers, layer);

    self
      .diameters
      .iter()
      .find(|(entry, _)| *entry == key)
      .or_else(|| self.diameters.first())
      .expect("a via padstack must hold at least one diameter")
      .1
  }

  /// Set the copper diameter on a layer.
  ///
  /// Port of `SetDiameter`, `pcbnew/router/pns_via.h:234`, minus the
  /// mirrored circle, which is derived here.
  pub fn set_diameter(
    &mut self,
    layers: LayerRange,
    layer: i32,
    diameter: i32,
  ) {
    let key = self.effective_layer(layers, layer);

    match self
      .diameters
      .binary_search_by_key(&key, |(entry, _)| *entry)
    {
      Ok(index) => self.diameters[index].1 = diameter,
      Err(index) => self.diameters.insert(index, (key, diameter)),
    }
  }

  /// Whether two vias have the same padstack.
  ///
  /// Port of `PadstackMatches`, `pcbnew/router/pns_via.cpp:108`. Its only
  /// caller in KiCad is the hole to hole pruning heuristic
  /// (`pcbnew/router/pns_item.cpp:66`), which recognises a line's copy of
  /// a via already in the node and which `src/collide.rs` retired, so
  /// nothing in this crate calls it either. It stays because it is a
  /// member of `VIA` and costs nothing.
  ///
  /// Deviation: KiCad's `std::equal( myLayers.begin(), myLayers.end(),
  /// otherLayers.begin() )` (`:113`) walks the other vector to the first
  /// one's length, which reads past its end when it is shorter. The
  /// lengths are compared first here.
  pub fn padstack_matches(
    &self,
    layers: LayerRange,
    other: &Via,
    other_layers: LayerRange,
  ) -> bool {
    let mine = self.unique_shape_layers(layers);
    let theirs = other.unique_shape_layers(other_layers);

    if mine != theirs {
      return false;
    }

    mine.iter().all(|layer| {
      self.diameter(layers, *layer) == other.diameter(other_layers, *layer)
    })
  }

  /// The copper circle on a layer.
  ///
  /// Port of `VIA::Shape`, `pcbnew/router/pns_via.h:302`, built from the
  /// diameter rather than read out of the mirrored map; see the type
  /// documentation. KiCad's `wxCHECK` returning a null shape
  /// (`:305`) has no counterpart, because [`Via::diameter`] always
  /// answers.
  pub fn shape(&self, layers: LayerRange, layer: i32) -> Shape {
    Shape::Circle {
      center: self.pos,
      radius: self.diameter(layers, layer) / 2,
    }
  }

  /// The walkaround boundary on a layer.
  ///
  /// Port of `VIA::Hull`, `pcbnew/router/pns_via.cpp:235`.
  ///
  /// `flashed` is `IsFlashedOnLayer( this, aLayer )`
  /// (`pcbnew/router/pns_via.cpp:243`), which [`Item::hull`] reads off
  /// [`Item::flashed_layers`]. When it is false the hull shrinks from the
  /// copper diameter to the **hole** diameter, so a via with a removed
  /// annular ring is an obstacle of a different size rather than no
  /// obstacle at all (note 05 section 7.3).
  ///
  /// KiCad reads the hole diameter as `m_hole->Radius() * 2` (`:244`).
  /// The hole is a separate arena item here, so `self.drill` is used
  /// instead: every path that sets one sets the other, `MakeCircularHole(
  /// m_pos, m_drill / 2, .. )` at `pcbnew/router/pns_via.h:97`, `:117`,
  /// `:143`, `:179` and `pcbnew/router/pns_via.cpp:278`, `SetDrill` at
  /// `pcbnew/router/pns_via.h:249`, and the host sync at
  /// `pcbnew/router/pns_kicad_iface.cpp:1863`. The `* 2` of a `/ 2`
  /// rounds an odd drill down, which is reproduced.
  ///
  /// The clearance sum truncates the half thickness, `aClearance +
  /// aWalkaroundThickness / 2` (`:240`), where
  /// [`build_hull_for_primitive_shape`] rounds it up. The chamfer is
  /// `( 2 * cl + width ) * ( 1.0 - M_SQRT1_2 )` (`:249`) truncated by
  /// C++'s implicit conversion at the call. That grouping is KiCad's own
  /// and is not the one [`crate::geometry::hull`] uses for the same
  /// quantity, so it is kept here rather than shared.
  ///
  /// KiCad asserts that a complex padstack is never hulled without a
  /// layer context (`:237`). That is reproduced as a debug assertion.
  pub fn hull(
    &self,
    layers: LayerRange,
    clearance: i32,
    walkaround_thickness: i32,
    layer: i32,
    flashed: bool,
  ) -> LineChain {
    debug_assert!(
      layer >= 0 || self.stack_mode == StackMode::Normal,
      "a complex via stack cannot be hulled without a layer context"
    );

    let cl = clearance + walkaround_thickness / 2;
    let mut width = self.diameter(layers, layer);

    if !flashed {
      width = (self.drill / 2) * 2;
    }

    let chamfer =
      (f64::from(2 * cl + width) * OCTAGON_CHAMFER_HALF_FACTOR) as i32;

    octagonal_hull(
      self.pos - Vec2::new(width / 2, width / 2),
      Vec2::new(width, width),
      cl,
      chamfer,
    )
  }

  /// The translation that pushes this via out of another item.
  ///
  /// Port of `VIA::PushoutForce( NODE*, const ITEM*, VECTOR2I& )`,
  /// `pcbnew/router/pns_via.cpp:126`, the two argument overload. It takes
  /// the largest minimum translation vector over the layers the two items
  /// can have distinct shapes on, and reports failure when that vector is
  /// zero, which happens both when nothing collides and when the colliding
  /// pair is one no cell can produce a vector for.
  ///
  /// `clearance` is KiCad's `aNode->GetClearance( this, aOther, false )`
  /// (`:128`). Resolving it needs the rule resolver and the node, so it is
  /// a parameter here.
  ///
  /// # The sign
  ///
  /// KiCad asks `aOther->Shape( layer )->Collide( Shape( layer ), .. )`
  /// (`:133`), so the obstacle is the first operand and the via is the
  /// second. [`collide_mtv`] displaces its second argument, so the result
  /// displaces the via, which is what the caller applies with
  /// `mv.SetPos( mv.Pos() + force )` (`pcbnew/router/pns_via.cpp:216`).
  /// The two cells this crate had to negate to reach one convention are
  /// the ones KiCad reaches **without** an operand swap, and a via is
  /// always the second operand here, so no cell changes meaning.
  ///
  /// # Two literal transcriptions
  ///
  /// KiCad declares `elementForce` outside the loop (`:129`) and
  /// `SHAPE::Collide` leaves the vector untouched when there is no
  /// collision (`libs/kimath/src/geometry/shape_collisions.cpp:1420`), so
  /// a layer that misses leaves the previous layer's vector in place. With
  /// the strict `>` at `:135` that cannot change the answer, but it is
  /// transcribed as it stands.
  ///
  /// KiCad dereferences `aOther->Shape( layer )` without a null check,
  /// which a via whose padstack has no entry for that layer can make null
  /// (`pcbnew/router/pns_via.h:305`). A missing shape is skipped here.
  ///
  /// # Not here
  ///
  /// The five argument overload (`pcbnew/router/pns_via.cpp:143`), which
  /// iterates this one against a node, caps each step at
  /// `Diameter( EffectiveLayer( 0 ) ) / 4` and switches to the lead vector
  /// past half its iteration budget, needs `NODE::CheckColliding`. It
  /// belongs to the shove and dragger task.
  pub fn pushout_force(
    &self,
    layers: LayerRange,
    other: &Item,
    clearance: i32,
  ) -> Option<Vec2> {
    let mut force = Vec2::new(0, 0);
    let mut element_force = Vec2::new(0, 0);

    let relevant = union_of_shape_layers(
      &self.unique_shape_layers(layers),
      true,
      &other.unique_shape_layers(),
      other.has_unique_shape_layers(),
    );

    for layer in relevant {
      let Some(other_shape) = other.shape(layer) else {
        continue;
      };

      if let Some(mtv) =
        collide_mtv(&other_shape, &self.shape(layers, layer), clearance)
      {
        element_force = mtv;
      }

      if element_force.squared_euclidean_norm() > force.squared_euclidean_norm()
      {
        force = element_force;
      }
    }

    if force == Vec2::new(0, 0) {
      None
    } else {
      Some(force)
    }
  }

  /// The centre, whatever the index.
  ///
  /// Port of `VIA::Anchor`, `pcbnew/router/pns_via.h:314`.
  pub const fn anchor(&self, _n: usize) -> Vec2 {
    self.pos
  }

  /// One. Port of `VIA::AnchorCount`,
  /// `pcbnew/router/pns_via.h:319`.
  pub const fn anchor_count(&self) -> usize {
    1
  }
}

// ---------------------------------------------------------------------
// Solid
// ---------------------------------------------------------------------

/// A pad, a board outline, or any other obstacle the router cannot move.
///
/// Port of `PNS::SOLID`, `pcbnew/router/pns_solid.h:36`.
///
/// Left out, with reasons:
///
/// - `m_padToDie` and `m_padToDieDelay` (`pcbnew/router/pns_solid.h:160`)
///   are read only by the meander placers
///   (`pcbnew/router/pns_meander_placer.cpp:107` and siblings). Length
///   tuning is a non goal, see `PLAN.md`.
/// - `EDA_ANGLE m_orientation` (`:162`) becomes a plain `f64` of degrees.
///   `EDA_ANGLE` is not ported (see the milestone 1 log entry), and the
///   router's one reader immediately calls `.AsDegrees()`
///   (`pcbnew/router/pns_line_placer.cpp:1415`), where the optimizer's
///   breakout code uses the pad's offset rather than its angle.
#[derive(Clone, PartialEq, Debug, Default)]
pub struct Solid {
  /// The copper. Port of `m_shape`,
  /// `pcbnew/router/pns_solid.h:158`, which KiCad allows to be null and
  /// checks for in `SOLID::Hull` (`pcbnew/router/pns_solid.cpp:41`).
  shape: Option<Shape>,
  /// The pad centre. Port of `m_pos`,
  /// `pcbnew/router/pns_solid.h:157`.
  pos: Vec2,
  /// The offset of the copper from the pad centre. Port of `m_offset`,
  /// `pcbnew/router/pns_solid.h:159`, read by the optimizer when it
  /// decides whether a breakout may start at the centre
  /// (`pcbnew/router/pns_optimizer.cpp:1123`).
  offset: Vec2,
  /// The pad rotation in degrees. Port of `m_orientation`,
  /// `pcbnew/router/pns_solid.h:162`; see the type documentation.
  orientation_degrees: f64,
  /// The connection points a trace may start on. Port of
  /// `m_anchorPoints`, `pcbnew/router/pns_solid.h:164`. Empty means "the
  /// centre is the only anchor".
  anchor_points: Vec<Vec2>,
}

impl Solid {
  /// A solid of a given copper shape, centred on a point.
  ///
  /// Port of the default `SOLID()` (`pcbnew/router/pns_solid.h:39`)
  /// followed by `SetShape` and `SetPos`, which is how the host builds
  /// every one of them (`pcbnew/router/pns_kicad_iface.cpp:1673` and
  /// following).
  pub fn new(shape: Shape, pos: Vec2) -> Self {
    Self {
      shape: Some(shape),
      pos,
      offset: Vec2::new(0, 0),
      orientation_degrees: 0.0,
      anchor_points: Vec::new(),
    }
  }

  /// The copper. Port of `SOLID::Shape`,
  /// `pcbnew/router/pns_solid.h:107`, which ignores the layer.
  pub const fn shape(&self) -> Option<&Shape> {
    self.shape.as_ref()
  }

  /// Replace the copper. Port of `SetShape`,
  /// `pcbnew/router/pns_solid.h:113`.
  pub fn set_shape(&mut self, shape: Option<Shape>) {
    self.shape = shape;
  }

  /// The pad centre. Port of `Pos`,
  /// `pcbnew/router/pns_solid.h:119`.
  pub const fn pos(&self) -> Vec2 {
    self.pos
  }

  /// Move the solid, copper and all.
  ///
  /// Port of `SOLID::SetPos`, `pcbnew/router/pns_solid.cpp:81`. KiCad also
  /// moves the hole it owns (`:89`); the hole is a separate arena item
  /// here, so the translation is returned and the caller applies it to the
  /// hole item with [`Hole::move_by`].
  pub fn set_pos(&mut self, center: Vec2) -> Vec2 {
    let delta = center - self.pos;

    if let Some(shape) = self.shape.as_mut() {
      shape.move_by(delta);
    }

    self.pos = center;

    delta
  }

  /// The offset of the copper from the pad centre. Port of `Offset`,
  /// `pcbnew/router/pns_solid.h:135`.
  pub const fn offset(&self) -> Vec2 {
    self.offset
  }

  /// Port of `SetOffset`, `pcbnew/router/pns_solid.h:136`.
  pub const fn set_offset(&mut self, offset: Vec2) {
    self.offset = offset;
  }

  /// The pad rotation in degrees.
  ///
  /// Port of `GetOrientation`, `pcbnew/router/pns_solid.h:138`, as the
  /// `f64` its one router side reader asks for
  /// (`pcbnew/router/pns_line_placer.cpp:1415`).
  pub const fn orientation_degrees(&self) -> f64 {
    self.orientation_degrees
  }

  /// Port of `SetOrientation`, `pcbnew/router/pns_solid.h:139`.
  pub const fn set_orientation_degrees(&mut self, degrees: f64) {
    self.orientation_degrees = degrees;
  }

  /// The connection points. Port of `AnchorPoints`,
  /// `pcbnew/router/pns_solid.h:132`.
  pub fn anchor_points(&self) -> &[Vec2] {
    &self.anchor_points
  }

  /// Port of `SetAnchorPoints`, `pcbnew/router/pns_solid.h:133`.
  pub fn set_anchor_points(&mut self, points: Vec<Vec2>) {
    self.anchor_points = points;
  }

  /// One connection point.
  ///
  /// Port of `SOLID::Anchor`, `pcbnew/router/pns_solid.cpp:95`, which
  /// answers the centre when there are no anchor points.
  ///
  /// # Panics
  ///
  /// When the index is out of range, where KiCad indexes its vector
  /// unchecked.
  pub fn anchor(&self, n: usize) -> Vec2 {
    if self.anchor_points.is_empty() {
      self.pos
    } else {
      self.anchor_points[n]
    }
  }

  /// How many connection points there are.
  ///
  /// Port of `SOLID::AnchorCount`,
  /// `pcbnew/router/pns_solid.cpp:100`, which answers one when there are
  /// no anchor points.
  pub fn anchor_count(&self) -> usize {
    if self.anchor_points.is_empty() {
      1
    } else {
      self.anchor_points.len()
    }
  }

  /// The walkaround boundary.
  ///
  /// Port of `SOLID::Hull`, `pcbnew/router/pns_solid.cpp:39`, with the
  /// polygon union replaced; see [`compound_hull`].
  pub fn hull(&self, clearance: i32, walkaround_thickness: i32) -> LineChain {
    let Some(shape) = self.shape.as_ref() else {
      return LineChain::new();
    };

    compound_hull(shape, clearance, walkaround_thickness)
  }
}

// ---------------------------------------------------------------------
// Hole
// ---------------------------------------------------------------------

/// A drilled hole or a slot.
///
/// Port of `PNS::HOLE`, `pcbnew/router/pns_hole.h:33`. A hole is a first
/// class item that lives in the spatial index on its own
/// (`pcbnew/router/pns_node.cpp:603`), which is why the parent pad or via
/// is a back reference on [`Item`] and not a field here.
///
/// KiCad allows `m_holeShape` to be null and checks for it in `HOLE::Hull`
/// (`pcbnew/router/pns_hole.cpp:59`). The only way to build one is
/// `HOLE( const ITEM& )` (`pcbnew/router/pns_hole.h:43`), which leaves the
/// pointer uninitialised rather than null and which nothing calls. The
/// shape is not optional here.
#[derive(Clone, PartialEq, Debug)]
pub struct Hole {
  /// The drilled shape, a circle for a round hole and a capsule for a
  /// slot. Port of `m_holeShape`, `pcbnew/router/pns_hole.h:94`.
  shape: Shape,
}

impl Hole {
  /// A round hole.
  ///
  /// Port of `HOLE::MakeCircularHole`,
  /// `pcbnew/router/pns_hole.cpp:131`, minus the layer range, which lives
  /// on [`Item`].
  pub const fn circular(center: Vec2, radius: i32) -> Self {
    Self {
      shape: Shape::Circle { center, radius },
    }
  }

  /// A hole of an arbitrary shape.
  ///
  /// Port of `HOLE( SHAPE* )`, `pcbnew/router/pns_hole.h:36`. A slot is a
  /// [`Shape::Segment`] (`pcbnew/router/pns_hole.cpp:131` is the round
  /// case; the host builds the slot case at
  /// `pcbnew/router/pns_kicad_iface.cpp:1729`).
  pub const fn new(shape: Shape) -> Self {
    Self { shape }
  }

  /// The drilled shape. Port of `HOLE::Shape`,
  /// `pcbnew/router/pns_hole.h:69`, which ignores the layer.
  pub const fn shape(&self) -> &Shape {
    &self.shape
  }

  /// Replace the drilled shape.
  ///
  /// KiCad has no setter; it constructs a fresh `HOLE` instead. This is
  /// the arena's equivalent.
  pub fn set_shape(&mut self, shape: Shape) {
    self.shape = shape;
  }

  /// The radius of a round hole.
  ///
  /// Port of `HOLE::Radius`, `pcbnew/router/pns_hole.cpp:103`, whose
  /// `assert( Type() == SH_CIRCLE )` becomes a `None`.
  pub const fn radius(&self) -> Option<i32> {
    match self.shape {
      Shape::Circle { radius, .. } => Some(radius),
      _ => None,
    }
  }

  /// Move a round hole's centre.
  ///
  /// Port of `HOLE::SetCenter`, `pcbnew/router/pns_hole.cpp:111`, called
  /// by `VIA::SetPos` (`pcbnew/router/pns_via.h:216`).
  ///
  /// # Panics
  ///
  /// When the hole is not round, where KiCad asserts.
  pub fn set_center(&mut self, center: Vec2) {
    match &mut self.shape {
      Shape::Circle { center: own, .. } => *own = center,
      other => {
        panic!("a hole of shape {:?} has no centre to set", other.kind())
      }
    }
  }

  /// Resize a round hole.
  ///
  /// Port of `HOLE::SetRadius`, `pcbnew/router/pns_hole.cpp:118`, called
  /// by `VIA::SetDrill` (`pcbnew/router/pns_via.h:254`).
  ///
  /// # Panics
  ///
  /// When the hole is not round, where KiCad asserts.
  pub fn set_radius(&mut self, radius: i32) {
    match &mut self.shape {
      Shape::Circle { radius: own, .. } => *own = radius,
      other => {
        panic!("a hole of shape {:?} has no radius to set", other.kind())
      }
    }
  }

  /// Translate the hole.
  ///
  /// Port of `HOLE::Move`, `pcbnew/router/pns_hole.cpp:125`, called by
  /// `SOLID::SetPos` (`pcbnew/router/pns_solid.cpp:89`).
  pub fn move_by(&mut self, delta: Vec2) {
    self.shape.move_by(delta);
  }

  /// The walkaround boundary.
  ///
  /// Port of `HOLE::Hull`, `pcbnew/router/pns_hole.cpp:57`. A round hole
  /// takes the same octagon as [`Via::hull`], with the same truncated
  /// half thickness and the same chamfer grouping (`:65`, `:71`), which is
  /// **not** what [`build_hull_for_primitive_shape`] would build for the
  /// same circle: that one rounds the half thickness up and groups the
  /// chamfer the other way, so the two differ by a nanometre for an odd
  /// walkaround thickness. Everything else goes through
  /// [`compound_hull`].
  pub fn hull(&self, clearance: i32, walkaround_thickness: i32) -> LineChain {
    if let Shape::Circle { center, radius } = self.shape {
      let cl = clearance + walkaround_thickness / 2;
      let width = radius * 2;
      let chamfer =
        (f64::from(2 * cl + width) * OCTAGON_CHAMFER_HALF_FACTOR) as i32;

      return octagonal_hull(
        center - Vec2::new(width / 2, width / 2),
        Vec2::new(width, width),
        cl,
        chamfer,
      );
    }

    compound_hull(&self.shape, clearance, walkaround_thickness)
  }
}

/// The walkaround boundary of a shape that may be a compound.
///
/// The shared body of `SOLID::Hull` (`pcbnew/router/pns_solid.cpp:39`) and
/// the non circular half of `HOLE::Hull`
/// (`pcbnew/router/pns_hole.cpp:73`), which are the same code twice.
///
/// KiCad unwraps a one element compound and otherwise unions the per
/// primitive hulls through a `SHAPE_POLY_SET` and takes outline zero
/// (`pns_solid.cpp:55`). That union is the router's only polygon boolean.
/// `DESIGN.md` section 3 replaces it with the convex hull of the union of
/// the per primitive hulls' vertices, [`monotone_chain_hull`], which is a
/// conservative superset: it can only make an obstacle larger, never
/// smaller, and it stays convex, which is what `LINE::Walkaround` assumes
/// of every hull (`pcbnew/router/pns_line.cpp:397`).
///
/// [`Shape::subshapes`] flattens a compound and answers with the shape
/// itself for anything else, so one call covers both of KiCad's branches.
/// Two edges KiCad has no answer for:
///
/// - an empty compound gives an empty chain, where KiCad reads
///   `Outline( 0 )` of an empty polygon set;
/// - a primitive [`build_hull_for_primitive_shape`] cannot hull, a bare
///   chain or a nested compound, is skipped, where KiCad adds an empty
///   outline to the set.
pub fn compound_hull(
  shape: &Shape,
  clearance: i32,
  walkaround_thickness: i32,
) -> LineChain {
  let leaves = shape.subshapes();

  if leaves.len() == 1 {
    return build_hull_for_primitive_shape(
      leaves[0],
      clearance,
      walkaround_thickness,
    )
    .unwrap_or_default();
  }

  let mut vertices = Vec::new();

  for leaf in leaves {
    let Some(hull) =
      build_hull_for_primitive_shape(leaf, clearance, walkaround_thickness)
    else {
      continue;
    };

    vertices.extend_from_slice(hull.points());
  }

  monotone_chain_hull(&vertices)
}

// ---------------------------------------------------------------------
// The body
// ---------------------------------------------------------------------

/// What kind of thing an item is, and its geometry.
///
/// Replaces KiCad's `ITEM` subclass hierarchy
/// (`pcbnew/router/pns_item.h:97` and its five descendants), as
/// `doc/reference/kicad/02-item-model-and-node.md` section 10.8 asks.
/// `LINE` never joins it: a line is a transient value and is never stored
/// (`DESIGN.md` section 4.2).
#[derive(Clone, PartialEq, Debug)]
pub enum ItemBody {
  /// A pad or another fixed obstacle. `PNS::SOLID`.
  Solid(Solid),
  /// A straight track. `PNS::SEGMENT`.
  Segment(Segment),
  /// A curved track. `PNS::ARC`.
  Arc(Arc),
  /// A via. `PNS::VIA`.
  Via(Via),
  /// A drilled hole or a slot. `PNS::HOLE`.
  Hole(Hole),
}

impl ItemBody {
  /// The one kind bit this body carries.
  ///
  /// Port of `ITEM::Kind`, `pcbnew/router/pns_item.h:173`, which reads the
  /// tag each constructor passed up.
  pub const fn kind(&self) -> Kind {
    match self {
      Self::Solid(_) => Kind::SOLID,
      Self::Segment(_) => Kind::SEGMENT,
      Self::Arc(_) => Kind::ARC,
      Self::Via(_) => Kind::VIA,
      Self::Hole(_) => Kind::HOLE,
    }
  }
}

// ---------------------------------------------------------------------
// Item
// ---------------------------------------------------------------------

/// One object in a routing world.
///
/// Port of `PNS::ITEM`, `pcbnew/router/pns_item.h:97`, flattened; see the
/// module documentation. The item does not know its own [`ItemId`]: the
/// arena owns that, and handing it out with every borrow would be the
/// pointer identity this port set out to remove.
#[derive(Clone, PartialEq, Debug)]
pub struct Item {
  /// The one kind bit, mirroring [`ItemBody::kind`]. Port of `m_kind`,
  /// `pcbnew/router/pns_item.h:316`.
  ///
  /// Stored rather than derived because [`Item::of_kind`] is on the
  /// collision search's hot path and KiCad stores it too.
  kind: Kind,
  /// The geometry and the per kind data.
  body: ItemBody,
  /// The net, or `None` for no net. Port of `m_net`,
  /// `pcbnew/router/pns_item.h:326`.
  net: Option<NetId>,
  /// The copper layers spanned. Port of `m_layers`,
  /// `pcbnew/router/pns_item.h:323`.
  layers: LayerRange,
  /// The transient marks. Port of `m_marker`,
  /// `pcbnew/router/pns_item.h:327`, minus the `mutable`.
  marker: MarkerFlags,
  /// The shove priority. Port of `m_rank`,
  /// `pcbnew/router/pns_item.h:328`.
  rank: i32,
  /// Where the item came from. Replaces `m_parent` and the `m_owner`
  /// based "is this synthesised" test; see [`Provenance`].
  provenance: Provenance,
  /// The host object this item descends from when the mapping is not one
  /// to one. Port of `m_sourceItem`,
  /// `pcbnew/router/pns_item.h:319`.
  source: Option<HostId>,
  /// The layers the item actually has copper on. See [`LayerMask`].
  flashed_layers: LayerMask,
  /// The hole this pad or via owns, if the world added one. Port of
  /// `VIA::m_hole` (`pcbnew/router/pns_via.h:356`) and `SOLID::m_hole`
  /// (`pcbnew/router/pns_solid.h:163`).
  hole: Option<ItemId>,
  /// The pad or via this hole belongs to. Port of
  /// `HOLE::m_parentPadVia`, `pcbnew/router/pns_hole.h:95`.
  parent_pad_via: Option<ItemId>,
  /// The booleans. See [`ItemFlags`].
  flags: ItemFlags,
  /// The identity the shove correlates an item by across branches.
  ///
  /// Port of `LINKED_ITEM::m_uid`,
  /// `pcbnew/router/pns_linked_item.h:64`, which the shove uses to find
  /// the root line an item descends from
  /// (`pcbnew/router/pns_shove.cpp:126`) and which `DESIGN.md` section 8
  /// also uses as the deterministic tie break on obstacle candidates.
  ///
  /// Every item carries one here, where KiCad only gives them to
  /// `LINKED_ITEM`s, that is to everything except `SOLID` and `HOLE`. One
  /// rule is cheaper than three and the tie break needs it on every
  /// candidate.
  ///
  /// # Clone
  ///
  /// A cloned item keeps its uid. KiCad has three rules for that:
  /// `SEGMENT::Clone` keeps it, `ARC::Clone` allocates a new one, and
  /// `VIA::Clone` keeps it with a "fixme: oop" comment
  /// (`pcbnew/router/pns_via.cpp:257`). `doc/reference/kicad/02-item-model-and-node.md`
  /// section 10.1 asks for one rule, documented, and keeping the uid is
  /// the one the shove depends on. A caller that wants a fresh identity
  /// says so, which is KiCad's `ResetUid`
  /// (`pcbnew/router/pns_linked_item.h:46`, called at
  /// `pcbnew/router/pns_line_placer.cpp:1624`), and is
  /// [`Item::set_uid`] here.
  uid: u64,
}

impl Item {
  /// The rank of an item the shove has not touched.
  ///
  /// Port of the `m_rank = -1` initialiser,
  /// `pcbnew/router/pns_item.h:125`, which `NODE::ClearRanks`
  /// (`pcbnew/router/pns_node.cpp:1682`) and `NODE::Commit` (`:1637`) also
  /// reset to.
  pub const UNASSIGNED_RANK: i32 = -1;

  /// A fresh item with KiCad's zero initialised defaults.
  ///
  /// Port of `ITEM( PnsKind )`, `pcbnew/router/pns_item.h:116`: no net,
  /// undefined layers, no marks, [`Item::UNASSIGNED_RANK`], routable, not
  /// virtual, not a free pad, not a compound primitive. The flashing mask
  /// starts empty, because the layer range does too; a host that sets the
  /// layers has to set the mask as well, which
  /// [`Item::set_layers_and_flash_all`] does in one step.
  pub fn new(uid: u64, body: ItemBody) -> Self {
    Self {
      kind: body.kind(),
      body,
      net: None,
      layers: LayerRange::UNDEFINED,
      marker: MarkerFlags::NONE,
      rank: Self::UNASSIGNED_RANK,
      provenance: Provenance::Synthetic,
      source: None,
      flashed_layers: LayerMask::NONE,
      hole: None,
      parent_pad_via: None,
      flags: ItemFlags::default(),
      uid,
    }
  }

  /// The shove's cross branch identity. See the field documentation.
  pub const fn uid(&self) -> u64 {
    self.uid
  }

  /// Give the item a fresh identity.
  ///
  /// Port of `LINKED_ITEM::ResetUid`,
  /// `pcbnew/router/pns_linked_item.h:46`, with the counter passed in
  /// rather than reached for globally.
  pub const fn set_uid(&mut self, uid: u64) {
    self.uid = uid;
  }

  /// The one kind bit. Port of `Kind`,
  /// `pcbnew/router/pns_item.h:173`.
  pub const fn kind(&self) -> Kind {
    self.kind
  }

  /// Whether the item matches a kind mask. Port of `OfKind`,
  /// `pcbnew/router/pns_item.h:181`.
  pub const fn of_kind(&self, mask: Kind) -> bool {
    self.kind.of_kind(mask)
  }

  /// The geometry and the per kind data.
  pub const fn body(&self) -> &ItemBody {
    &self.body
  }

  /// The geometry and the per kind data, mutably.
  pub const fn body_mut(&mut self) -> &mut ItemBody {
    &mut self.body
  }

  /// The net, or `None`. Port of `Net`,
  /// `pcbnew/router/pns_item.h:210`.
  ///
  /// Deviation: `HOLE::Net` is virtual and forwards to the parent pad or
  /// via when there is one (`pcbnew/router/pns_hole.h:56`). That lookup
  /// needs the arena, so it lives with the node; a hole's own net is
  /// whatever the world set when it added the hole.
  pub const fn net(&self) -> Option<NetId> {
    self.net
  }

  /// Port of `SetNet`, `pcbnew/router/pns_item.h:209`.
  pub const fn set_net(&mut self, net: Option<NetId>) {
    self.net = net;
  }

  /// The copper layers spanned. Port of `Layers`,
  /// `pcbnew/router/pns_item.h:212`.
  pub const fn layers(&self) -> LayerRange {
    self.layers
  }

  /// Port of `SetLayers`, `pcbnew/router/pns_item.h:213`.
  pub const fn set_layers(&mut self, layers: LayerRange) {
    self.layers = layers;
  }

  /// Collapse the item onto one layer.
  ///
  /// Port of `SetLayer`, `pcbnew/router/pns_item.h:215`.
  pub const fn set_layer(&mut self, layer: i32) {
    self.layers = LayerRange::single(layer);
  }

  /// Set the layers and flash the item on all of them.
  ///
  /// Not a KiCad routine: it is the "for LibrePCB's first integration the
  /// mask equals the layer span" shorthand of `DESIGN.md` section 5, and
  /// the fallback KiCad's own flashing predicate lands on for a router
  /// created via (`pcbnew/router/pns_kicad_iface.cpp:2201`).
  pub fn set_layers_and_flash_all(&mut self, layers: LayerRange) {
    self.layers = layers;
    self.flashed_layers = LayerMask::from_layer_range(layers);
  }

  /// The item's layer, for the callers that treat it as single layer.
  ///
  /// Port of `Layer`, `pcbnew/router/pns_item.h:216`, which is
  /// `Layers().Start()`.
  pub const fn layer(&self) -> i32 {
    self.layers.start()
  }

  /// Whether the two items share a layer. Port of `LayersOverlap`,
  /// `pcbnew/router/pns_item.h:221`.
  pub const fn layers_overlap(&self, other: &Item) -> bool {
    self.layers.overlaps(other.layers)
  }

  /// The layers the item has copper on. See [`LayerMask`].
  pub const fn flashed_layers(&self) -> LayerMask {
    self.flashed_layers
  }

  /// Port of nothing: the host sets this where KiCad answers a callback.
  pub const fn set_flashed_layers(&mut self, mask: LayerMask) {
    self.flashed_layers = mask;
  }

  /// Whether the item has copper on a layer.
  ///
  /// Port of `ROUTER_IFACE::IsFlashedOnLayer( const ITEM*, int )`,
  /// `pcbnew/router/pns_kicad_iface.cpp:2168`. See [`LayerMask`] for the
  /// three places the answer matters.
  pub fn is_flashed_on(&self, layer: i32) -> bool {
    self.flashed_layers.is_flashed_on(layer)
  }

  /// Whether the item has copper anywhere in the layers it shares with an
  /// interval.
  ///
  /// Port of the layer range overload of `IsFlashedOnLayer`,
  /// `pcbnew/router/pns_kicad_iface.cpp:2204`, whose first line is the
  /// intersection with the item's own range (`:2206`). This is the form
  /// the clearance ladder asks, as `IsFlashedOnLayer( this,
  /// aHead->Layers() )` (`pcbnew/router/pns_item.cpp:206`).
  pub fn is_flashed_on_any(&self, layers: LayerRange) -> bool {
    self
      .flashed_layers
      .is_flashed_on_any(self.layers.intersection(layers))
  }

  /// The transient marks. Port of `Marker`,
  /// `pcbnew/router/pns_item.h:263`.
  pub const fn marker(&self) -> MarkerFlags {
    self.marker
  }

  /// Overwrite the marks.
  ///
  /// Port of `Mark`, `pcbnew/router/pns_item.h:261`, which **assigns**
  /// rather than sets bits. KiCad's is `const` over a `mutable` field; the
  /// interior mutability disappears here
  /// (`doc/reference/kicad/02-item-model-and-node.md` section 10.8).
  pub const fn mark(&mut self, marker: MarkerFlags) {
    self.marker = marker;
  }

  /// Clear the masked marks.
  ///
  /// Port of `Unmark`, `pcbnew/router/pns_item.h:262`, whose default
  /// argument of `-1` is [`MarkerFlags::ALL`] here.
  pub const fn unmark(&mut self, mask: MarkerFlags) {
    self.marker = self.marker.remove(mask);
  }

  /// Whether the item is locked. Port of `IsLocked`,
  /// `pcbnew/router/pns_item.h:278`.
  pub const fn is_locked(&self) -> bool {
    self.marker.intersects(MarkerFlags::LOCKED)
  }

  /// The shove priority. Port of `Rank`,
  /// `pcbnew/router/pns_item.h:266`.
  pub const fn rank(&self) -> i32 {
    self.rank
  }

  /// Port of `SetRank`, `pcbnew/router/pns_item.h:265`.
  pub const fn set_rank(&mut self, rank: i32) {
    self.rank = rank;
  }

  /// Where the item came from. See [`Provenance`].
  pub const fn provenance(&self) -> Provenance {
    self.provenance
  }

  /// Set where the item came from.
  ///
  /// Port of `SetParent`, `pcbnew/router/pns_item.h:191`, including its
  /// side effect: a non null parent also becomes the source item
  /// (`:196`).
  pub const fn set_provenance(&mut self, provenance: Provenance) {
    self.provenance = provenance;

    if let Provenance::Board(host) = provenance {
      self.source = Some(host);
    }
  }

  /// The host object this item maps to one to one, if any.
  ///
  /// Port of `Parent`, `pcbnew/router/pns_item.h:199`.
  pub const fn host_id(&self) -> Option<HostId> {
    match self.provenance {
      Provenance::Board(host) => Some(host),
      Provenance::Synthetic => None,
    }
  }

  /// The host object this item descends from.
  ///
  /// Port of `GetSourceItem`, `pcbnew/router/pns_item.h:202`. It differs
  /// from [`Item::host_id`] where the mapping is not one to one: dragging
  /// one track produces several segments, none of which is that track
  /// (`pcbnew/router/pns_segment.h:64`), and `NODE::AssembleLine` does the
  /// mirror image for the line it builds
  /// (`pcbnew/router/pns_node.cpp:1150`). Note 05 has it feeding attribute
  /// inheritance at commit time
  /// (`pcbnew/router/pns_kicad_iface.cpp:2775`), which is why it is kept
  /// even though nothing in milestone 1 reads it.
  pub const fn source(&self) -> Option<HostId> {
    self.source
  }

  /// Port of `SetSourceItem`, `pcbnew/router/pns_item.h:201`.
  pub const fn set_source(&mut self, source: Option<HostId>) {
    self.source = source;
  }

  /// The hole this pad or via owns.
  ///
  /// Port of `Hole`, `pcbnew/router/pns_item.h:304`. A hole is indexed on
  /// its own (`pcbnew/router/pns_node.cpp:603`), so this is a handle and
  /// not ownership.
  pub const fn hole(&self) -> Option<ItemId> {
    self.hole
  }

  /// Port of `SetHole`, `pcbnew/router/pns_item.h:305`, minus the
  /// ownership transfer: the arena owns every item.
  pub const fn set_hole(&mut self, hole: Option<ItemId>) {
    self.hole = hole;
  }

  /// Whether the item has a hole.
  ///
  /// Port of `HasHole`, `pcbnew/router/pns_item.h:303`. KiCad answers per
  /// class: `VIA::HasHole` is unconditionally true
  /// (`pcbnew/router/pns_via.h:339`), `SOLID::HasHole` tests the pointer
  /// (`pcbnew/router/pns_solid.h:153`) and `VVIA::HasHole` is false
  /// (`pcbnew/router/pns_via.h:376`). All three become "is there a hole
  /// item", which the world sets when it adds one
  /// (`pcbnew/router/pns_node.cpp:626`).
  pub const fn has_hole(&self) -> bool {
    self.hole.is_some()
  }

  /// The pad or via this hole belongs to.
  ///
  /// Port of `ParentPadVia`, `pcbnew/router/pns_item.h:293`, which is
  /// virtual and only `HOLE` overrides
  /// (`pcbnew/router/pns_hole.h:72`).
  pub const fn parent_pad_via(&self) -> Option<ItemId> {
    self.parent_pad_via
  }

  /// Port of `HOLE::SetParentPadVia`,
  /// `pcbnew/router/pns_hole.h:71`.
  pub const fn set_parent_pad_via(&mut self, parent: Option<ItemId>) {
    self.parent_pad_via = parent;
  }

  /// The booleans. See [`ItemFlags`].
  pub const fn flags(&self) -> ItemFlags {
    self.flags
  }

  /// Replace the booleans wholesale. See [`ItemFlags`].
  pub const fn set_flags(&mut self, flags: ItemFlags) {
    self.flags = flags;
  }

  /// Whether a trace may start or end here. Port of `IsRoutable`,
  /// `pcbnew/router/pns_item.h:284`.
  pub const fn is_routable(&self) -> bool {
    self.flags.routable
  }

  /// Port of `SetRoutable`, `pcbnew/router/pns_item.h:283`.
  pub const fn set_routable(&mut self, routable: bool) {
    self.flags.routable = routable;
  }

  /// Whether this is a virtual via. Port of `IsVirtual`,
  /// `pcbnew/router/pns_item.h:295`.
  pub const fn is_virtual(&self) -> bool {
    self.flags.is_virtual
  }

  /// Mark the item virtual.
  ///
  /// KiCad has no setter: `VVIA`'s constructor writes the field directly
  /// (`pcbnew/router/pns_via.h:373`). `NODE::FixupVirtualVias`
  /// (`pcbnew/router/pns_node.cpp:1330`) is the one place that builds
  /// them, and it will use this.
  pub const fn set_is_virtual(&mut self, is_virtual: bool) {
    self.flags.is_virtual = is_virtual;
  }

  /// Whether this is a pad on a not internally connected pin.
  ///
  /// Port of `IsFreePad`, `pcbnew/router/pns_item.h:288`, minus the
  /// second half: KiCad also answers true when the parent pad or via is a
  /// free pad (`:261`). That lookup needs the arena, so the clearance
  /// ladder in the node module has to fold in
  /// `parent_pad_via`'s flag itself.
  pub const fn is_free_pad(&self) -> bool {
    self.flags.is_free_pad
  }

  /// Port of `SetIsFreePad`, `pcbnew/router/pns_item.h:286`.
  pub const fn set_is_free_pad(&mut self, is_free_pad: bool) {
    self.flags.is_free_pad = is_free_pad;
  }

  /// Whether this item is one primitive of a decomposed compound pad.
  ///
  /// Port of `IsCompoundShapePrimitive`,
  /// `pcbnew/router/pns_item.h:301`.
  pub const fn is_compound_shape_primitive(&self) -> bool {
    self.flags.is_compound_shape_primitive
  }

  /// Port of `SetIsCompoundShapePrimitive`,
  /// `pcbnew/router/pns_item.h:300`, which in KiCad can only ever set the
  /// flag.
  pub const fn set_is_compound_shape_primitive(&mut self, primitive: bool) {
    self.flags.is_compound_shape_primitive = primitive;
  }

  /// The layers on which the item has a distinct shape.
  ///
  /// Port of `UniqueShapeLayers`, `pcbnew/router/pns_item.h:250`, whose
  /// default is `{ -1 }` and which only `VIA` overrides
  /// (`pcbnew/router/pns_via.cpp:56`). A `-1` means "one shape, no layer
  /// context".
  pub fn unique_shape_layers(&self) -> Vec<i32> {
    match &self.body {
      ItemBody::Via(via) => via.unique_shape_layers(self.layers),
      _ => vec![-1],
    }
  }

  /// Whether [`Item::unique_shape_layers`] says anything useful.
  ///
  /// Port of `HasUniqueShapeLayers`,
  /// `pcbnew/router/pns_item.h:252`, false except for `VIA`
  /// (`pcbnew/router/pns_via.h:204`).
  pub const fn has_unique_shape_layers(&self) -> bool {
    matches!(self.body, ItemBody::Via(_))
  }

  /// The layers a collision between these two items has to be tested on.
  ///
  /// Port of `RelevantShapeLayers`,
  /// `pcbnew/router/pns_item.cpp:83`: `{ -1 }` when neither side has per
  /// layer shapes, else the set union of both sides' unique shape layers.
  /// KiCad's `std::set_union` into a `std::set` sorts and deduplicates
  /// whatever it is given, which is what this does directly.
  ///
  /// KiCad's own TODO at `:91` notes that this over tests when a via meets
  /// a track, because the track's layers are not masked off. Reproduced.
  pub fn relevant_shape_layers(&self, other: &Item) -> Vec<i32> {
    union_of_shape_layers(
      &self.unique_shape_layers(),
      self.has_unique_shape_layers(),
      &other.unique_shape_layers(),
      other.has_unique_shape_layers(),
    )
  }

  /// The geometry the collision code sees on a layer.
  ///
  /// Port of `Shape( int aLayer )`, `pcbnew/router/pns_item.h:242`. Every
  /// body but the via ignores the layer. A via builds its circle from the
  /// padstack diameter, so it is the one case that has to allocate a
  /// value; the shapes that can be large, a pad outline or a compound, are
  /// borrowed.
  ///
  /// `None` is KiCad's null return: a `SOLID` with no shape
  /// (`pcbnew/router/pns_solid.h:107`).
  pub fn shape(&self, layer: i32) -> Option<Cow<'_, Shape>> {
    match &self.body {
      ItemBody::Solid(solid) => solid.shape().map(Cow::Borrowed),
      ItemBody::Segment(segment) => Some(Cow::Owned(segment.shape())),
      ItemBody::Arc(arc) => Some(Cow::Owned(arc.shape())),
      ItemBody::Via(via) => Some(Cow::Owned(via.shape(self.layers, layer))),
      ItemBody::Hole(hole) => Some(Cow::Borrowed(hole.shape())),
    }
  }

  /// The walkaround boundary on a layer.
  ///
  /// Port of `Hull( aClearance, aWalkaroundThickness, aLayer )`,
  /// `pcbnew/router/pns_item.h:164`, dispatched per body. The result is a
  /// closed, convex, clockwise chain; `LINE::Walkaround` implements the
  /// counter clockwise winding by reversing it
  /// (`pcbnew/router/pns_line.cpp:397`), so the winding is part of the
  /// contract.
  ///
  /// `aWalkaroundThickness` is the width of the **moving** line, never of
  /// this item.
  ///
  /// Only the via reads the layer, and it reads it twice: to pick the
  /// padstack diameter and to ask whether the via is flashed there. The
  /// second answer comes from [`Item::flashed_layers`], where KiCad calls
  /// back into the host (`pcbnew/router/pns_via.cpp:243`).
  pub fn hull(
    &self,
    clearance: i32,
    walkaround_thickness: i32,
    layer: i32,
  ) -> LineChain {
    match &self.body {
      ItemBody::Solid(solid) => solid.hull(clearance, walkaround_thickness),
      ItemBody::Segment(segment) => {
        segment.hull(clearance, walkaround_thickness)
      }
      ItemBody::Arc(arc) => arc.hull(clearance, walkaround_thickness),
      ItemBody::Via(via) => via.hull(
        self.layers,
        clearance,
        walkaround_thickness,
        layer,
        self.is_flashed_on(layer),
      ),
      ItemBody::Hole(hole) => hole.hull(clearance, walkaround_thickness),
    }
  }

  /// The bounding box of every shape the item has, grown by a clearance.
  ///
  /// KiCad has no `ITEM::BBox`. Its spatial index asks each shape
  /// (`pcbnew/router/pns_index.cpp:33`) and `VIA::ChangedArea` merges the
  /// per layer boxes (`pcbnew/router/pns_via.cpp:297`); this is that merge
  /// for every body. The clearance is a parameter because the index has to
  /// store the box it inserted with, inflated by the node's maximum
  /// clearance (`DESIGN.md` section 4.3).
  ///
  /// `None` where no shape has a box: a solid with no shape, an empty
  /// compound, an empty chain.
  pub fn bbox(&self, clearance: i32) -> Option<Box2> {
    self
      .unique_shape_layers()
      .into_iter()
      .filter_map(|layer| self.shape(layer)?.bbox(clearance))
      .reduce(Box2::merge)
  }

  /// One connection point.
  ///
  /// Port of `Anchor`, `pcbnew/router/pns_item.h:268`, whose default is
  /// the origin and which `SEGMENT`, `VIA` and `SOLID` override. A hole
  /// keeps the default, so it answers `(0, 0)`.
  ///
  /// # Panics
  ///
  /// For a solid, when the index is past its anchor point list; see
  /// [`Solid::anchor`].
  pub fn anchor(&self, n: usize) -> Vec2 {
    match &self.body {
      ItemBody::Solid(solid) => solid.anchor(n),
      ItemBody::Segment(segment) => segment.anchor(n),
      ItemBody::Arc(arc) => arc.anchor(n),
      ItemBody::Via(via) => via.anchor(n),
      ItemBody::Hole(_) => Vec2::new(0, 0),
    }
  }

  /// How many connection points the item has.
  ///
  /// Port of `AnchorCount`, `pcbnew/router/pns_item.h:273`, whose default
  /// is zero. A hole keeps the default.
  pub fn anchor_count(&self) -> usize {
    match &self.body {
      ItemBody::Solid(solid) => solid.anchor_count(),
      ItemBody::Segment(segment) => segment.anchor_count(),
      ItemBody::Arc(arc) => arc.anchor_count(),
      ItemBody::Via(via) => via.anchor_count(),
      ItemBody::Hole(_) => 0,
    }
  }
}

/// The shared body of [`Item::relevant_shape_layers`].
///
/// Port of `ITEM::RelevantShapeLayers`,
/// `pcbnew/router/pns_item.cpp:83`. It is a free function because
/// [`Via::pushout_force`] needs it before an [`Item`] exists for the via
/// side.
fn union_of_shape_layers(
  mine: &[i32],
  mine_unique: bool,
  theirs: &[i32],
  theirs_unique: bool,
) -> Vec<i32> {
  if !mine_unique && !theirs_unique {
    return vec![-1];
  }

  let mut union: Vec<i32> = mine.iter().chain(theirs).copied().collect();
  union.sort_unstable();
  union.dedup();

  union
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::line_chain::LineChain;

  /// Shorthand for a point.
  fn point(x: i32, y: i32) -> Vec2 {
    Vec2::new(x, y)
  }

  /// A chain's points as a flat list, for comparing against a table.
  fn flatten(chain: &LineChain) -> Vec<i32> {
    chain
      .points()
      .iter()
      .flat_map(|point| [point.x, point.y])
      .collect()
  }

  /// An item of a body, on one layer, flashed there.
  fn item_on(layer: i32, body: ItemBody) -> Item {
    let mut item = Item::new(0, body);
    item.set_layers_and_flash_all(LayerRange::single(layer));

    item
  }

  // -----------------------------------------------------------------
  // LayerRange
  // -----------------------------------------------------------------

  #[test]
  fn layer_range_default_is_undefined() {
    assert_eq!(LayerRange::default(), LayerRange::UNDEFINED);
    assert_eq!(LayerRange::UNDEFINED.start(), -1);
    assert_eq!(LayerRange::UNDEFINED.end(), -1);
    assert!(!LayerRange::UNDEFINED.is_defined());
  }

  #[test]
  fn layer_range_new_swaps_an_inverted_pair() {
    assert_eq!(LayerRange::new(3, 1), LayerRange::new(1, 3));
    assert_eq!(LayerRange::new(1, 0).start(), 0);
    assert_eq!(LayerRange::new(1, 0).end(), 1);
  }

  #[test]
  fn layer_range_single_and_all() {
    assert_eq!(LayerRange::single(2).start(), 2);
    assert_eq!(LayerRange::single(2).end(), 2);
    assert_eq!(LayerRange::all().start(), 0);
    assert_eq!(LayerRange::all().end(), 256);
  }

  #[test]
  fn layer_range_is_multilayer_ignores_validity() {
    assert!(LayerRange::new(0, 3).is_multilayer());
    assert!(!LayerRange::single(3).is_multilayer());
    assert!(!LayerRange::UNDEFINED.is_multilayer());
  }

  #[test]
  fn layer_range_overlaps_is_closed_interval_overlap() {
    assert!(LayerRange::new(0, 3).overlaps(LayerRange::new(3, 5)));
    assert!(LayerRange::new(0, 3).overlaps(LayerRange::new(1, 2)));
    assert!(!LayerRange::new(0, 3).overlaps(LayerRange::new(4, 5)));
    assert!(LayerRange::single(2).overlaps(LayerRange::single(2)));
  }

  /// This is what makes a dangling joint invisible to `FindJoint`
  /// (`pcbnew/router/pns_joint.h:229`).
  #[test]
  fn layer_range_overlaps_is_false_for_any_negative_endpoint() {
    assert!(!LayerRange::UNDEFINED.overlaps(LayerRange::new(0, 3)));
    assert!(!LayerRange::new(0, 3).overlaps(LayerRange::UNDEFINED));
    assert!(!LayerRange::UNDEFINED.overlaps(LayerRange::UNDEFINED));
    assert!(!LayerRange::UNDEFINED.overlaps(LayerRange::all()));
  }

  #[test]
  fn layer_range_contains_a_layer() {
    let range = LayerRange::new(1, 3);

    assert!(!range.contains(0));
    assert!(range.contains(1));
    assert!(range.contains(3));
    assert!(!range.contains(4));
    assert!(!range.contains(-1));
    assert!(!LayerRange::UNDEFINED.contains(0));
  }

  #[test]
  fn layer_range_merge_absorbs_into_an_undefined_range() {
    let mut range = LayerRange::UNDEFINED;
    range.merge(LayerRange::new(2, 4));

    assert_eq!(range, LayerRange::new(2, 4));

    let mut undefined = LayerRange::UNDEFINED;
    undefined.merge(LayerRange::UNDEFINED);

    assert_eq!(undefined, LayerRange::UNDEFINED);
  }

  /// Merge is a hull and not a union, so it can cover layers neither
  /// input had.
  #[test]
  fn layer_range_merge_is_a_hull() {
    let mut range = LayerRange::new(0, 1);
    range.merge(LayerRange::new(5, 6));

    assert_eq!(range, LayerRange::new(0, 6));

    let mut inside = LayerRange::new(0, 6);
    inside.merge(LayerRange::new(2, 3));

    assert_eq!(inside, LayerRange::new(0, 6));
  }

  /// Merging an undefined range into a defined one drags the start down
  /// to `-1`, because the absorbing branch only fires on the receiver.
  #[test]
  fn layer_range_merge_with_an_undefined_operand_poisons_the_start() {
    let mut range = LayerRange::new(2, 4);
    range.merge(LayerRange::UNDEFINED);

    assert_eq!(range.start(), -1);
    assert_eq!(range.end(), 4);
    assert!(!range.is_defined());
  }

  #[test]
  fn layer_range_intersection_of_two_overlapping_ranges() {
    let overlap = LayerRange::new(0, 3).intersection(LayerRange::new(2, 5));

    assert_eq!(overlap.start(), 2);
    assert_eq!(overlap.end(), 3);
  }

  /// Two disjoint ranges give `start > end`, which
  /// [`LayerRange::new`] would have swapped. KiCad has the same hole.
  #[test]
  fn layer_range_intersection_can_invert() {
    let overlap = LayerRange::new(0, 2).intersection(LayerRange::new(5, 7));

    assert_eq!(overlap.start(), 5);
    assert_eq!(overlap.end(), 2);
  }

  /// The asymmetric special case at `pcbnew/router/pns_layerset.h:118`:
  /// a negative end takes the other side's end.
  #[test]
  fn layer_range_intersection_with_an_undefined_operand() {
    let from_left = LayerRange::UNDEFINED.intersection(LayerRange::new(3, 5));
    assert_eq!(from_left.start(), 3);
    assert_eq!(from_left.end(), 5);

    let from_right = LayerRange::new(3, 5).intersection(LayerRange::UNDEFINED);
    assert_eq!(from_right.start(), 3);
    assert_eq!(from_right.end(), 5);

    let both = LayerRange::UNDEFINED.intersection(LayerRange::UNDEFINED);
    assert_eq!(both, LayerRange::UNDEFINED);
  }

  // -----------------------------------------------------------------
  // Kind
  // -----------------------------------------------------------------

  #[test]
  fn kind_values_match_kicad() {
    assert_eq!(Kind::INVALID.bits(), 0);
    assert_eq!(Kind::SOLID.bits(), 1);
    assert_eq!(Kind::LINE.bits(), 2);
    assert_eq!(Kind::JOINT.bits(), 4);
    assert_eq!(Kind::SEGMENT.bits(), 8);
    assert_eq!(Kind::ARC.bits(), 16);
    assert_eq!(Kind::VIA.bits(), 32);
    assert_eq!(Kind::DIFF_PAIR.bits(), 64);
    assert_eq!(Kind::HOLE.bits(), 128);
    assert_eq!(Kind::ANY.bits(), 0xffff);
    assert_eq!(Kind::LINKED_ITEM_MASK.bits(), 1 + 8 + 16 + 32 + 128);
  }

  #[test]
  fn of_kind_is_a_bit_test() {
    assert!(Kind::SEGMENT.of_kind(Kind::SEGMENT | Kind::VIA));
    assert!(Kind::VIA.of_kind(Kind::SEGMENT | Kind::VIA));
    assert!(!Kind::SOLID.of_kind(Kind::SEGMENT | Kind::VIA));
    assert!(Kind::SOLID.of_kind(Kind::ANY));
    assert!(Kind::HOLE.of_kind(Kind::ANY));
    assert!(!Kind::SOLID.of_kind(Kind::INVALID));
  }

  /// The mask includes solids and holes even though neither is a
  /// `LINKED_ITEM` in KiCad.
  #[test]
  fn linked_item_mask_covers_solids_and_holes() {
    assert!(Kind::SOLID.of_kind(Kind::LINKED_ITEM_MASK));
    assert!(Kind::HOLE.of_kind(Kind::LINKED_ITEM_MASK));
    assert!(Kind::SEGMENT.of_kind(Kind::LINKED_ITEM_MASK));
    assert!(Kind::VIA.of_kind(Kind::LINKED_ITEM_MASK));
    assert!(!Kind::LINE.of_kind(Kind::LINKED_ITEM_MASK));
    assert!(!Kind::JOINT.of_kind(Kind::LINKED_ITEM_MASK));
  }

  // -----------------------------------------------------------------
  // MarkerFlags
  // -----------------------------------------------------------------

  #[test]
  fn marker_values_match_kicad() {
    assert_eq!(MarkerFlags::NONE.bits(), 0);
    assert_eq!(MarkerFlags::HEAD.bits(), 1);
    assert_eq!(MarkerFlags::VIOLATION.bits(), 8);
    assert_eq!(MarkerFlags::LOCKED.bits(), 16);
    assert_eq!(MarkerFlags::DP_COUPLED.bits(), 32);
    assert_eq!(MarkerFlags::ALL.bits(), 1 + 8 + 16 + 32);
    assert_eq!(MarkerFlags::CLEARED_BY_CLEAR_RANKS.bits(), 1 + 8);
  }

  #[test]
  fn marker_bit_algebra() {
    let both = MarkerFlags::HEAD | MarkerFlags::LOCKED;

    assert!(both.contains(MarkerFlags::HEAD));
    assert!(both.contains(MarkerFlags::LOCKED));
    assert!(!both.contains(MarkerFlags::VIOLATION));
    assert!(both.contains(MarkerFlags::HEAD | MarkerFlags::LOCKED));

    assert!(both.intersects(MarkerFlags::HEAD | MarkerFlags::VIOLATION));
    assert!(!both.intersects(MarkerFlags::VIOLATION));

    assert_eq!(both.remove(MarkerFlags::HEAD), MarkerFlags::LOCKED);
    assert_eq!(both.remove(MarkerFlags::ALL), MarkerFlags::NONE);
    assert!(MarkerFlags::NONE.is_empty());
    assert!(!both.is_empty());

    let mut accumulating = MarkerFlags::NONE;
    accumulating |= MarkerFlags::VIOLATION;

    assert_eq!(accumulating, MarkerFlags::VIOLATION);
  }

  /// `Mark` assigns and `Unmark` clears, which is what makes
  /// `NODE::ClearRanks`'s masked reset work.
  #[test]
  fn mark_assigns_and_unmark_clears() {
    let mut item = item_on(
      0,
      ItemBody::Via(Via::new(point(0, 0), 600, 300, ViaType::Through)),
    );

    item.mark(MarkerFlags::LOCKED | MarkerFlags::VIOLATION);
    assert!(item.is_locked());

    item.mark(MarkerFlags::HEAD);
    assert_eq!(item.marker(), MarkerFlags::HEAD);
    assert!(!item.is_locked());

    item.mark(MarkerFlags::HEAD | MarkerFlags::LOCKED);
    item.unmark(MarkerFlags::CLEARED_BY_CLEAR_RANKS);
    assert_eq!(item.marker(), MarkerFlags::LOCKED);

    item.unmark(MarkerFlags::ALL);
    assert_eq!(item.marker(), MarkerFlags::NONE);
  }

  // -----------------------------------------------------------------
  // LayerMask
  // -----------------------------------------------------------------

  #[test]
  fn layer_mask_from_a_span() {
    let mask = LayerMask::from_layer_range(LayerRange::new(1, 3));

    assert!(!mask.is_flashed_on(0));
    assert!(mask.is_flashed_on(1));
    assert!(mask.is_flashed_on(2));
    assert!(mask.is_flashed_on(3));
    assert!(!mask.is_flashed_on(4));
    assert_eq!(mask.bits(), 0b1110);
  }

  #[test]
  fn layer_mask_of_an_undefined_range_is_empty() {
    assert_eq!(
      LayerMask::from_layer_range(LayerRange::UNDEFINED),
      LayerMask::NONE
    );
  }

  /// `LayerRange::all()` reaches layer 256; the mask stops at 63.
  #[test]
  fn layer_mask_saturates_at_the_limit() {
    let mask = LayerMask::from_layer_range(LayerRange::all());

    assert_eq!(mask, LayerMask::ALL);
    assert!(mask.is_flashed_on(LayerMask::MAX_LAYER));
  }

  /// KiCad's single layer overload short circuits to true for a negative
  /// layer, which means "no layer context, assume flashed".
  #[test]
  fn layer_mask_assumes_flashed_without_a_layer_context() {
    assert!(LayerMask::NONE.is_flashed_on(-1));
    assert!(LayerMask::NONE.is_flashed_on(-5));
    assert!(!LayerMask::NONE.is_flashed_on(0));
  }

  #[test]
  fn layer_mask_with_and_without() {
    let mask = LayerMask::NONE.with(3).with(5);

    assert!(mask.is_flashed_on(3));
    assert!(mask.is_flashed_on(5));
    assert!(!mask.is_flashed_on(4));
    assert!(!mask.with(-1).with(64).without(-1).is_flashed_on(0));
    assert!(!mask.without(3).is_flashed_on(3));
  }

  #[test]
  fn layer_mask_over_an_interval_answers_on_the_first_flashed_layer() {
    let mask = LayerMask::NONE.with(3).with(5);

    assert!(mask.is_flashed_on_any(LayerRange::new(0, 3)));
    assert!(mask.is_flashed_on_any(LayerRange::new(5, 9)));
    assert!(mask.is_flashed_on_any(LayerRange::new(3, 5)));
    assert!(!mask.is_flashed_on_any(LayerRange::new(0, 2)));
    assert!(!mask.is_flashed_on_any(LayerRange::new(6, 9)));
    assert!(!mask.is_flashed_on_any(LayerRange::single(4)));
  }

  #[test]
  fn layer_mask_over_an_empty_interval_is_not_flashed() {
    let mask = LayerMask::ALL;

    // What `LayerRange::intersection` produces for two disjoint ranges.
    assert!(!mask.is_flashed_on_any(LayerRange { start: 5, end: 2 }));
  }

  #[test]
  fn layer_mask_over_an_interval_without_a_layer_context_is_flashed() {
    assert!(LayerMask::NONE.is_flashed_on_any(LayerRange::new(-1, 3)));
  }

  #[test]
  fn layer_mask_over_the_widest_interval_does_not_overflow() {
    assert!(
      LayerMask::NONE
        .with(63)
        .is_flashed_on_any(LayerRange::new(0, 63))
    );
    assert!(!LayerMask::NONE.is_flashed_on_any(LayerRange::new(0, 63)));
    assert!(
      LayerMask::NONE
        .with(0)
        .is_flashed_on_any(LayerRange::new(0, 63))
    );
  }

  #[test]
  fn an_item_intersects_the_interval_with_its_own_layers() {
    let mut item = item_on(2, ItemBody::Hole(Hole::circular(point(0, 0), 100)));
    item.set_layers_and_flash_all(LayerRange::new(2, 4));

    assert!(item.is_flashed_on_any(LayerRange::new(0, 2)));
    assert!(item.is_flashed_on_any(LayerRange::all()));
    assert!(!item.is_flashed_on_any(LayerRange::new(0, 1)));

    item.set_flashed_layers(LayerMask::NONE.with(4));

    assert!(!item.is_flashed_on_any(LayerRange::new(0, 3)));
    assert!(item.is_flashed_on_any(LayerRange::new(3, 6)));
  }

  // -----------------------------------------------------------------
  // Item plumbing
  // -----------------------------------------------------------------

  #[test]
  fn a_fresh_item_has_kicads_defaults() {
    let item = Item::new(
      7,
      ItemBody::Segment(Segment::new(Seg::from_coords(0, 0, 10, 0), 200)),
    );

    assert_eq!(item.uid(), 7);
    assert_eq!(item.kind(), Kind::SEGMENT);
    assert_eq!(item.net(), None);
    assert_eq!(item.layers(), LayerRange::UNDEFINED);
    assert_eq!(item.marker(), MarkerFlags::NONE);
    assert_eq!(item.rank(), Item::UNASSIGNED_RANK);
    assert_eq!(item.provenance(), Provenance::Synthetic);
    assert_eq!(item.source(), None);
    assert_eq!(item.flashed_layers(), LayerMask::NONE);
    assert_eq!(item.hole(), None);
    assert_eq!(item.parent_pad_via(), None);
    assert!(item.is_routable());
    assert!(!item.is_virtual());
    assert!(!item.is_free_pad());
    assert!(!item.is_compound_shape_primitive());
  }

  #[test]
  fn every_body_reports_its_kind() {
    assert_eq!(
      ItemBody::Solid(Solid::new(Shape::circle(point(0, 0), 5), point(0, 0)))
        .kind(),
      Kind::SOLID
    );
    assert_eq!(
      ItemBody::Segment(Segment::new(Seg::from_coords(0, 0, 1, 0), 1)).kind(),
      Kind::SEGMENT
    );
    assert_eq!(
      ItemBody::Via(Via::new(point(0, 0), 600, 300, ViaType::Through)).kind(),
      Kind::VIA
    );
    assert_eq!(
      ItemBody::Hole(Hole::circular(point(0, 0), 150)).kind(),
      Kind::HOLE
    );
  }

  /// `SetParent` also assigns the source item when the parent is not
  /// null (`pcbnew/router/pns_item.h:196`).
  #[test]
  fn setting_a_board_provenance_also_sets_the_source() {
    let mut item = item_on(
      0,
      ItemBody::Segment(Segment::new(Seg::from_coords(0, 0, 10, 0), 200)),
    );

    item.set_provenance(Provenance::Board(HostId(42)));

    assert_eq!(item.host_id(), Some(HostId(42)));
    assert_eq!(item.source(), Some(HostId(42)));

    item.set_provenance(Provenance::Synthetic);

    assert_eq!(item.host_id(), None);
    assert_eq!(item.source(), Some(HostId(42)));
  }

  #[test]
  fn set_layer_collapses_the_range_and_layer_reads_the_start() {
    let mut item = item_on(
      0,
      ItemBody::Segment(Segment::new(Seg::from_coords(0, 0, 10, 0), 200)),
    );

    item.set_layers(LayerRange::new(1, 4));
    assert_eq!(item.layer(), 1);

    item.set_layer(2);
    assert_eq!(item.layers(), LayerRange::single(2));
    assert_eq!(item.layer(), 2);
  }

  #[test]
  fn layers_overlap_forwards_to_the_range() {
    let mut first = item_on(
      0,
      ItemBody::Segment(Segment::new(Seg::from_coords(0, 0, 10, 0), 200)),
    );
    let mut second = first.clone();

    first.set_layers(LayerRange::new(0, 1));
    second.set_layers(LayerRange::new(1, 2));
    assert!(first.layers_overlap(&second));

    second.set_layers(LayerRange::new(2, 3));
    assert!(!first.layers_overlap(&second));
  }

  #[test]
  fn hole_and_parent_handles_round_trip() {
    let mut arena: crate::arena::Arena<Item> = crate::arena::Arena::new();
    let hole_id = arena
      .insert(item_on(0, ItemBody::Hole(Hole::circular(point(0, 0), 150))));
    let via_id = arena.insert(item_on(
      1,
      ItemBody::Via(Via::new(point(0, 0), 600, 300, ViaType::Through)),
    ));

    arena.get_mut(via_id).unwrap().set_hole(Some(hole_id));
    arena
      .get_mut(hole_id)
      .unwrap()
      .set_parent_pad_via(Some(via_id));

    assert!(arena.get(via_id).unwrap().has_hole());
    assert_eq!(arena.get(via_id).unwrap().hole(), Some(hole_id));
    assert_eq!(arena.get(hole_id).unwrap().parent_pad_via(), Some(via_id));
    assert!(!arena.get(hole_id).unwrap().has_hole());
  }

  #[test]
  fn unique_and_relevant_shape_layers() {
    let segment = item_on(
      0,
      ItemBody::Segment(Segment::new(Seg::from_coords(0, 0, 10, 0), 200)),
    );
    let mut via = Item::new(
      1,
      ItemBody::Via(Via::new(point(0, 0), 600, 300, ViaType::Through)),
    );
    via.set_layers_and_flash_all(LayerRange::new(0, 3));

    assert_eq!(segment.unique_shape_layers(), vec![-1]);
    assert!(!segment.has_unique_shape_layers());
    assert_eq!(via.unique_shape_layers(), vec![Via::ALL_LAYERS]);
    assert!(via.has_unique_shape_layers());

    // Neither side has per layer shapes: the short circuit.
    assert_eq!(segment.relevant_shape_layers(&segment), vec![-1]);
    // One side does: the union, sorted and deduplicated.
    assert_eq!(via.relevant_shape_layers(&segment), vec![-1, 0]);
    assert_eq!(segment.relevant_shape_layers(&via), vec![-1, 0]);
  }

  // -----------------------------------------------------------------
  // Segment
  // -----------------------------------------------------------------

  #[test]
  fn segment_shape_bbox_and_anchors() {
    let segment = Segment::new(Seg::from_coords(0, 0, 100, 0), 21);
    let item = item_on(0, ItemBody::Segment(segment));

    assert_eq!(
      item.shape(-1).unwrap().into_owned(),
      Shape::segment(Seg::from_coords(0, 0, 100, 0), 21)
    );

    // The capsule's box grows by the clearance plus the half width,
    // rounded up (`shape_segment.h:63`).
    let bbox = item.bbox(5).unwrap();
    assert_eq!(bbox.left(), -16);
    assert_eq!(bbox.right(), 116);
    assert_eq!(bbox.top(), -16);
    assert_eq!(bbox.bottom(), 16);

    assert_eq!(item.anchor_count(), 2);
    assert_eq!(item.anchor(0), point(0, 0));
    assert_eq!(item.anchor(1), point(100, 0));
    // KiCad answers B for every index but zero.
    assert_eq!(item.anchor(7), point(100, 0));
  }

  #[test]
  fn segment_set_ends_and_swap_ends() {
    let mut segment = Segment::new(Seg::from_coords(0, 0, 100, 0), 21);

    segment.set_ends(point(1, 2), point(3, 4));
    assert_eq!(segment.seg().a, point(1, 2));
    assert_eq!(segment.seg().b, point(3, 4));

    segment.swap_ends();
    assert_eq!(segment.seg().a, point(3, 4));
    assert_eq!(segment.seg().b, point(1, 2));

    segment.set_width(50);
    assert_eq!(segment.width(), 50);

    assert_eq!(flatten(&segment.line()), vec![3, 4, 1, 2]);
  }

  /// The hull is exactly what the geometry layer's builder produces from
  /// the raw arguments, half thickness truncation included.
  #[test]
  fn segment_hull_is_the_geometry_layers_segment_hull() {
    let seg = Seg::from_coords(0, 0, 1000, 0);
    let segment = Segment::new(seg, 200);
    let item = item_on(0, ItemBody::Segment(segment));

    assert_eq!(
      flatten(&item.hull(100, 51, -1)),
      flatten(&segment_hull(&seg, 200, 100, 51))
    );
    // The layer is ignored.
    assert_eq!(
      flatten(&item.hull(100, 51, 0)),
      flatten(&item.hull(100, 51, 3))
    );
  }

  // -----------------------------------------------------------------
  // Arc
  // -----------------------------------------------------------------

  /// A quarter turn about `(1000, 0)` of radius 1000, width 200.
  fn quarter() -> ShapeArc {
    ShapeArc::new(point(0, 0), point(293, 707), point(1000, 1000), 200)
  }

  #[test]
  fn arc_forwards_its_shape_width_and_anchors() {
    let mut arc = Arc::new(quarter());

    assert_eq!(arc.shape(), Shape::Arc(quarter()));
    assert_eq!(arc.width(), 200);
    assert_eq!(arc.anchor(0), point(0, 0));
    // Every index but zero is the far end, as `Segment::anchor` is.
    assert_eq!(arc.anchor(1), point(1000, 1000));
    assert_eq!(arc.anchor(7), point(1000, 1000));
    assert_eq!(arc.anchor_count(), 2);

    arc.set_width(50);
    assert_eq!(arc.width(), 50);
    assert_eq!(arc.arc().width(), 50);

    let moved = ShapeArc::new(point(0, 0), point(293, 707), point(0, 2000), 50);
    arc.set_arc(moved);
    assert_eq!(arc.arc(), moved);
  }

  /// The hull is exactly what the geometry layer's builder produces from
  /// the raw arguments.
  #[test]
  fn arc_hull_is_the_geometry_layers_arc_hull() {
    let arc = quarter();
    let item = item_on(0, ItemBody::Arc(Arc::new(arc)));

    assert_eq!(
      flatten(&item.hull(100, 51, -1)),
      flatten(&arc_hull(&arc, 100, 51).expect("a quarter turn hulls"))
    );
    // The layer is ignored.
    assert_eq!(
      flatten(&item.hull(100, 51, 0)),
      flatten(&item.hull(100, 51, 3))
    );
  }

  /// The one case [`arc_hull`] cannot answer becomes an empty chain.
  ///
  /// Erratum E21's guard needs three coincident points **and** a combined
  /// clearance of zero, because any clearance at all sends a chord of
  /// zero down the whole circle branch instead. KiCad dereferences an
  /// empty optional here; the item answers the hull of an item with no
  /// geometry, which is what [`Arc::hull`] documents.
  #[test]
  fn an_arc_whose_hull_cannot_be_built_answers_an_empty_chain() {
    let at = point(500, 500);
    let degenerate = ShapeArc::new(at, at, at, 200);
    let item = item_on(0, ItemBody::Arc(Arc::new(degenerate)));

    assert!(arc_hull(&degenerate, 0, 0).is_err());
    assert_eq!(item.hull(0, 0, -1).point_count(), 0);

    // Any clearance at all takes the whole circle branch and answers an
    // octagon, so the empty chain is as unreachable here as it is in
    // KiCad.
    assert!(item.hull(1, 0, -1).point_count() > 2);
  }

  #[test]
  fn arc_changed_area_is_the_union_of_the_two_boxes() {
    let first = Arc::new(quarter());
    let second = Arc::new(ShapeArc::new(
      point(0, 0),
      point(293, -707),
      point(1000, -1000),
      200,
    ));

    let area = first.changed_area(&second);

    assert_eq!(area, first.arc().bbox(0).merge(second.arc().bbox(0)));
    assert_eq!(first.changed_area(&first), first.arc().bbox(0));
  }

  // -----------------------------------------------------------------
  // Via
  // -----------------------------------------------------------------

  #[test]
  fn via_shape_is_a_circle_of_half_the_diameter() {
    let item = item_on(
      0,
      ItemBody::Via(Via::new(point(50, 60), 601, 300, ViaType::Through)),
    );

    assert_eq!(
      item.shape(-1).unwrap().into_owned(),
      Shape::circle(point(50, 60), 300)
    );
  }

  #[test]
  fn via_effective_layer_per_stack_mode() {
    let layers = LayerRange::new(0, 3);
    let mut via = Via::new(point(0, 0), 600, 300, ViaType::Through);

    assert_eq!(via.effective_layer(layers, 2), Via::ALL_LAYERS);
    assert_eq!(via.effective_layer(layers, -1), Via::ALL_LAYERS);

    via.set_stack_mode(StackMode::FrontInnerBack);
    assert_eq!(via.effective_layer(layers, 0), 0);
    assert_eq!(via.effective_layer(layers, 3), 3);
    assert_eq!(via.effective_layer(layers, 2), 1);
    // A two layer via has no inner layer, so it falls back to the start.
    assert_eq!(via.effective_layer(LayerRange::new(0, 1), 5), 0);

    via.set_stack_mode(StackMode::Custom);
    assert_eq!(via.effective_layer(layers, 2), 2);
    assert_eq!(via.effective_layer(layers, 9), 0);
  }

  #[test]
  fn via_unique_shape_layers_per_stack_mode() {
    let layers = LayerRange::new(0, 3);
    let mut via = Via::new(point(0, 0), 600, 300, ViaType::Through);

    assert_eq!(via.unique_shape_layers(layers), vec![0]);

    via.set_stack_mode(StackMode::FrontInnerBack);
    assert_eq!(via.unique_shape_layers(layers), vec![0, 1, 3]);

    via.set_stack_mode(StackMode::Custom);
    assert_eq!(via.unique_shape_layers(layers), vec![0, 1, 2, 3]);
  }

  #[test]
  fn via_diameters_are_per_padstack_key() {
    let layers = LayerRange::new(0, 3);
    let mut via = Via::new(point(0, 0), 600, 300, ViaType::Through);
    via.set_stack_mode(StackMode::FrontInnerBack);

    via.set_diameter(layers, 0, 700);
    via.set_diameter(layers, 2, 500);
    via.set_diameter(layers, 3, 800);

    assert_eq!(via.diameter(layers, 0), 700);
    assert_eq!(via.diameter(layers, 1), 500);
    assert_eq!(via.diameter(layers, 2), 500);
    assert_eq!(via.diameter(layers, 3), 800);
  }

  /// The `wxCHECK` fallback: an unset padstack key answers with the entry
  /// of the smallest key.
  #[test]
  fn via_diameter_falls_back_to_the_first_padstack_entry() {
    let layers = LayerRange::new(0, 3);
    let mut via = Via::new(point(0, 0), 600, 300, ViaType::Through);
    via.set_stack_mode(StackMode::Custom);

    via.set_diameter(layers, 1, 700);

    // Key 0 is the seed from `Via::new`, key 2 was never set.
    assert_eq!(via.diameter(layers, 0), 600);
    assert_eq!(via.diameter(layers, 1), 700);
    assert_eq!(via.diameter(layers, 2), 600);
  }

  #[test]
  fn via_padstack_matches_compares_layers_then_diameters() {
    let layers = LayerRange::new(0, 3);
    let first = Via::new(point(0, 0), 600, 300, ViaType::Through);
    let mut second = Via::new(point(10, 10), 600, 300, ViaType::Through);

    assert!(first.padstack_matches(layers, &second, layers));

    second.set_diameter(layers, 0, 700);
    assert!(!first.padstack_matches(layers, &second, layers));

    second.set_diameter(layers, 0, 600);
    second.set_stack_mode(StackMode::Custom);
    assert!(!first.padstack_matches(layers, &second, layers));
  }

  #[test]
  fn via_pos_drill_and_free_flag_round_trip() {
    let mut via = Via::new(point(0, 0), 600, 300, ViaType::Through);

    via.set_pos(point(5, 6));
    assert_eq!(via.pos(), point(5, 6));
    assert_eq!(via.anchor(0), point(5, 6));
    assert_eq!(via.anchor(3), point(5, 6));
    assert_eq!(via.anchor_count(), 1);

    via.set_drill(250);
    assert_eq!(via.drill(), 250);

    assert!(!via.is_free());
    via.set_is_free(true);
    assert!(via.is_free());

    via.set_via_type(ViaType::Blind);
    assert_eq!(via.via_type(), ViaType::Blind);
    assert_eq!(via.stack_mode(), StackMode::Normal);
  }

  /// `VIA::Hull` is an octagon around the copper square, with the half
  /// thickness truncated and the chamfer truncated from KiCad's own
  /// grouping.
  #[test]
  fn via_hull_on_a_flashed_layer_uses_the_copper_diameter() {
    let item = item_on(
      0,
      ItemBody::Via(Via::new(point(0, 0), 600, 300, ViaType::Through)),
    );

    // cl = 100 + 51 / 2 = 125, width = 600.
    // chamfer = trunc( 850 * ( 1 - 1 / sqrt 2 ) ) = trunc( 248.99.. ) = 248.
    let cl = 125;
    let width = 600;
    let chamfer = (f64::from(2 * cl + width) * (1.0 - FRAC_1_SQRT_2)) as i32;
    assert_eq!(chamfer, 248);

    assert_eq!(
      flatten(&item.hull(100, 51, 0)),
      flatten(&octagonal_hull(
        point(-300, -300),
        point(600, 600),
        cl,
        chamfer
      ))
    );
  }

  /// A via that is not flashed on the layer is an obstacle the size of
  /// its hole, not no obstacle at all
  /// (`pcbnew/router/pns_via.cpp:243`).
  #[test]
  fn via_hull_on_an_unflashed_layer_shrinks_to_the_hole() {
    let mut item = Item::new(
      0,
      ItemBody::Via(Via::new(point(0, 0), 600, 301, ViaType::Through)),
    );
    item.set_layers(LayerRange::new(0, 3));
    item.set_flashed_layers(LayerMask::NONE.with(0).with(3));

    let flashed = item.hull(100, 51, 0);
    let unflashed = item.hull(100, 51, 1);

    assert_ne!(flatten(&flashed), flatten(&unflashed));

    // The odd drill halves down: 301 / 2 * 2 == 300.
    let cl = 125;
    let width = 300;
    let chamfer = (f64::from(2 * cl + width) * (1.0 - FRAC_1_SQRT_2)) as i32;

    assert_eq!(
      flatten(&unflashed),
      flatten(&octagonal_hull(
        point(-150, -150),
        point(300, 300),
        cl,
        chamfer
      ))
    );
  }

  /// A negative layer means "no layer context", which the flashing mask
  /// answers "flashed" to, so the hull stays at the copper diameter.
  #[test]
  fn via_hull_without_a_layer_context_stays_flashed() {
    let mut item = Item::new(
      0,
      ItemBody::Via(Via::new(point(0, 0), 600, 300, ViaType::Through)),
    );
    item.set_layers(LayerRange::new(0, 3));
    item.set_flashed_layers(LayerMask::NONE);

    assert_eq!(flatten(&item.hull(100, 51, -1)), {
      let mut flashed = item.clone();
      flashed.set_flashed_layers(LayerMask::ALL);

      flatten(&flashed.hull(100, 51, 0))
    });
  }

  #[test]
  fn via_bbox_covers_the_copper() {
    let item = item_on(
      0,
      ItemBody::Via(Via::new(point(100, 200), 600, 300, ViaType::Through)),
    );
    let bbox = item.bbox(10).unwrap();

    assert_eq!(bbox.left(), 100 - 300 - 10);
    assert_eq!(bbox.right(), 100 + 300 + 10);
    assert_eq!(bbox.top(), 200 - 300 - 10);
    assert_eq!(bbox.bottom(), 200 + 300 + 10);
  }

  // -----------------------------------------------------------------
  // Via::PushoutForce
  // -----------------------------------------------------------------

  /// Two concentric vias have no direction to be pushed in, which is the
  /// degeneracy `pcbnew/router/pns_via.cpp:172` gives up on.
  #[test]
  fn pushout_force_of_two_concentric_vias_is_none() {
    let moving = Via::new(point(0, 0), 600, 300, ViaType::Through);
    let obstacle = item_on(
      0,
      ItemBody::Via(Via::new(point(0, 0), 600, 300, ViaType::Through)),
    );

    assert_eq!(
      moving.pushout_force(LayerRange::single(0), &obstacle, 100),
      None
    );
  }

  #[test]
  fn pushout_force_against_another_via_separates_them() {
    let layers = LayerRange::single(0);
    let moving = Via::new(point(0, 0), 600, 300, ViaType::Through);
    let obstacle = item_on(
      0,
      ItemBody::Via(Via::new(point(400, 0), 600, 300, ViaType::Through)),
    );

    let force = moving.pushout_force(layers, &obstacle, 100).unwrap();

    // The obstacle sits to the right, so the via is pushed left.
    assert!(force.x < 0, "{force:?}");
    assert_eq!(force.y, 0);

    // Applying the force clears the collision.
    let mut moved = moving.clone();
    moved.set_pos(moving.pos() + force);

    assert_eq!(moved.pushout_force(layers, &obstacle, 100), None);
  }

  #[test]
  fn pushout_force_against_a_segment_separates_them() {
    let layers = LayerRange::single(0);
    let moving = Via::new(point(0, 0), 600, 300, ViaType::Through);
    let obstacle = item_on(
      0,
      ItemBody::Segment(Segment::new(
        Seg::from_coords(-1000, 350, 1000, 350),
        200,
      )),
    );

    let force = moving.pushout_force(layers, &obstacle, 100).unwrap();

    // The track runs below the via, so the via is pushed up.
    assert_eq!(force.x, 0);
    assert!(force.y < 0, "{force:?}");

    let mut moved = moving.clone();
    moved.set_pos(moving.pos() + force);

    assert_eq!(moved.pushout_force(layers, &obstacle, 100), None);
  }

  #[test]
  fn pushout_force_of_a_via_far_away_is_none() {
    let layers = LayerRange::single(0);
    let moving = Via::new(point(0, 0), 600, 300, ViaType::Through);
    let obstacle = item_on(
      0,
      ItemBody::Via(Via::new(point(100_000, 0), 600, 300, ViaType::Through)),
    );

    assert_eq!(moving.pushout_force(layers, &obstacle, 100), None);
  }

  /// A padstack with different diameters per layer is tested on every
  /// relevant layer and the largest vector wins
  /// (`pcbnew/router/pns_via.cpp:135`).
  #[test]
  fn pushout_force_takes_the_largest_vector_over_the_layers() {
    let layers = LayerRange::new(0, 3);
    let mut moving = Via::new(point(0, 0), 400, 200, ViaType::Through);
    moving.set_stack_mode(StackMode::Custom);
    moving.set_diameter(layers, 0, 400);
    moving.set_diameter(layers, 1, 400);
    moving.set_diameter(layers, 2, 1200);
    moving.set_diameter(layers, 3, 400);

    let obstacle = item_on(
      0,
      ItemBody::Via(Via::new(point(700, 0), 400, 200, ViaType::Through)),
    );

    let narrow_only = {
      let mut via = moving.clone();
      via.set_diameter(layers, 2, 400);

      via.pushout_force(layers, &obstacle, 0)
    };
    let with_the_wide_layer = moving.pushout_force(layers, &obstacle, 0);

    assert_eq!(narrow_only, None);
    assert!(with_the_wide_layer.unwrap().x < 0);
  }

  // -----------------------------------------------------------------
  // Solid
  // -----------------------------------------------------------------

  #[test]
  fn solid_hull_of_one_primitive_is_the_primitive_hull() {
    let shape = Shape::rect(point(-500, -250), point(1000, 500));
    let item =
      item_on(0, ItemBody::Solid(Solid::new(shape.clone(), point(0, 0))));

    assert_eq!(
      flatten(&item.hull(100, 51, -1)),
      flatten(&build_hull_for_primitive_shape(&shape, 100, 51).unwrap())
    );
  }

  /// A compound of one primitive unwraps, exactly as KiCad's
  /// `cmpnd->Shapes().size() == 1` branch does
  /// (`pcbnew/router/pns_solid.cpp:48`).
  #[test]
  fn solid_hull_of_a_one_element_compound_unwraps() {
    let primitive = Shape::circle(point(0, 0), 300);
    let item = item_on(
      0,
      ItemBody::Solid(Solid::new(
        Shape::compound(vec![primitive.clone()]),
        point(0, 0),
      )),
    );

    assert_eq!(
      flatten(&item.hull(100, 51, -1)),
      flatten(&build_hull_for_primitive_shape(&primitive, 100, 51).unwrap())
    );
  }

  /// The convex hull replacement for KiCad's polygon union has to contain
  /// both primitive hulls, which is what makes it a conservative
  /// superset.
  #[test]
  fn solid_hull_of_a_two_element_compound_contains_both_primitive_hulls() {
    let left = Shape::circle(point(-1000, 0), 300);
    let right = Shape::circle(point(1000, 0), 300);
    let item = item_on(
      0,
      ItemBody::Solid(Solid::new(
        Shape::compound(vec![left.clone(), right.clone()]),
        point(0, 0),
      )),
    );

    let hull = item.hull(100, 50, -1);

    assert!(hull.is_closed());
    assert!(hull.area(false) > 0.0, "the hull must stay clockwise");

    for primitive in [&left, &right] {
      let primitive_hull =
        build_hull_for_primitive_shape(primitive, 100, 50).unwrap();

      for index in 0..primitive_hull.point_count() {
        let vertex = primitive_hull.point(index);

        assert!(
          hull.point_inside(vertex, 0) || hull.point_on_edge(vertex, 0),
          "{vertex:?} of the primitive hull left outside the compound hull"
        );
      }
    }
  }

  #[test]
  fn solid_with_no_shape_has_no_hull_no_shape_and_no_bbox() {
    let mut solid = Solid::new(Shape::circle(point(0, 0), 300), point(0, 0));
    solid.set_shape(None);

    let item = item_on(0, ItemBody::Solid(solid));

    assert_eq!(item.hull(100, 50, -1).point_count(), 0);
    assert!(item.shape(-1).is_none());
    assert_eq!(item.bbox(0), None);
  }

  #[test]
  fn solid_anchors_default_to_the_centre() {
    let mut solid = Solid::new(Shape::circle(point(7, 8), 300), point(7, 8));

    assert_eq!(solid.anchor_count(), 1);
    assert_eq!(solid.anchor(0), point(7, 8));

    solid.set_anchor_points(vec![point(1, 1), point(2, 2)]);
    assert_eq!(solid.anchor_count(), 2);
    assert_eq!(solid.anchor(1), point(2, 2));
    assert_eq!(solid.anchor_points(), &[point(1, 1), point(2, 2)]);
  }

  #[test]
  fn solid_set_pos_moves_the_copper_and_reports_the_translation() {
    let mut solid = Solid::new(Shape::circle(point(0, 0), 300), point(0, 0));

    let delta = solid.set_pos(point(100, -50));

    assert_eq!(delta, point(100, -50));
    assert_eq!(solid.pos(), point(100, -50));
    assert_eq!(solid.shape(), Some(&Shape::circle(point(100, -50), 300)));
  }

  #[test]
  fn solid_offset_and_orientation_round_trip() {
    let mut solid = Solid::new(Shape::circle(point(0, 0), 300), point(0, 0));

    assert_eq!(solid.offset(), point(0, 0));
    assert_eq!(solid.orientation_degrees(), 0.0);

    solid.set_offset(point(10, 20));
    solid.set_orientation_degrees(90.0);

    assert_eq!(solid.offset(), point(10, 20));
    assert_eq!(solid.orientation_degrees(), 90.0);
  }

  // -----------------------------------------------------------------
  // Hole
  // -----------------------------------------------------------------

  #[test]
  fn hole_shape_radius_and_moves() {
    let mut hole = Hole::circular(point(10, 20), 150);

    assert_eq!(hole.radius(), Some(150));
    assert_eq!(hole.shape(), &Shape::circle(point(10, 20), 150));

    hole.set_center(point(0, 0));
    hole.set_radius(200);
    assert_eq!(hole.shape(), &Shape::circle(point(0, 0), 200));

    hole.move_by(point(5, -5));
    assert_eq!(hole.shape(), &Shape::circle(point(5, -5), 200));

    let slot = Hole::new(Shape::segment(Seg::from_coords(0, 0, 100, 0), 60));
    assert_eq!(slot.radius(), None);
  }

  /// A round hole's octagon uses `VIA::Hull`'s truncated half thickness
  /// and chamfer grouping, which differs from what the geometry layer's
  /// primitive builder makes of the same circle for an odd thickness.
  #[test]
  fn hole_hull_of_a_circle_differs_from_the_primitive_hull() {
    let hole = Hole::circular(point(0, 0), 150);
    let item = item_on(0, ItemBody::Hole(hole.clone()));

    let cl = 100 + 51 / 2;
    let width = 300;
    let chamfer = (f64::from(2 * cl + width) * (1.0 - FRAC_1_SQRT_2)) as i32;

    assert_eq!(
      flatten(&item.hull(100, 51, -1)),
      flatten(&octagonal_hull(
        point(-150, -150),
        point(300, 300),
        cl,
        chamfer
      ))
    );

    // The primitive builder rounds the half thickness up instead.
    assert_ne!(
      flatten(&item.hull(100, 51, -1)),
      flatten(&build_hull_for_primitive_shape(hole.shape(), 100, 51).unwrap())
    );
    // For an even thickness the two agree.
    assert_eq!(
      flatten(&item.hull(100, 50, -1)),
      flatten(&build_hull_for_primitive_shape(hole.shape(), 100, 50).unwrap())
    );
  }

  /// A slot hole is a capsule, so it goes through the compound path and
  /// lands on the segment hull.
  #[test]
  fn hole_hull_of_a_slot_is_the_segment_hull() {
    let seg = Seg::from_coords(0, 0, 500, 0);
    let hole = Hole::new(Shape::segment(seg, 60));
    let item = item_on(0, ItemBody::Hole(hole));

    assert_eq!(
      flatten(&item.hull(100, 51, -1)),
      flatten(&segment_hull(&seg, 60, 100, 51))
    );
  }

  #[test]
  fn hole_has_no_anchors() {
    let item = item_on(0, ItemBody::Hole(Hole::circular(point(10, 20), 150)));

    assert_eq!(item.anchor_count(), 0);
    assert_eq!(item.anchor(0), point(0, 0));
  }

  #[test]
  #[should_panic(expected = "no centre to set")]
  fn setting_the_centre_of_a_slot_panics() {
    let mut slot =
      Hole::new(Shape::segment(Seg::from_coords(0, 0, 100, 0), 60));
    slot.set_center(point(1, 1));
  }

  #[test]
  #[should_panic(expected = "no radius to set")]
  fn setting_the_radius_of_a_slot_panics() {
    let mut slot =
      Hole::new(Shape::segment(Seg::from_coords(0, 0, 100, 0), 60));
    slot.set_radius(10);
  }

  // -----------------------------------------------------------------
  // The uid counter
  // -----------------------------------------------------------------

  #[test]
  fn the_uid_counter_starts_at_zero_and_never_repeats() {
    let mut counter = UidCounter::new();

    assert_eq!(counter.next_uid(), 0);
    assert_eq!(counter.next_uid(), 1);
    assert_eq!(counter.next_uid(), 2);

    // A second world starts over, which is the whole point of not having
    // a process global counter.
    let mut other = UidCounter::new();
    assert_eq!(other.next_uid(), 0);
  }

  /// One clone rule for every body: the uid survives, and a caller that
  /// wants a fresh identity asks for one.
  #[test]
  fn a_cloned_item_keeps_its_uid_until_it_is_reset() {
    let mut counter = UidCounter::new();
    let item = Item::new(
      counter.next_uid(),
      ItemBody::Via(Via::new(point(0, 0), 600, 300, ViaType::Through)),
    );

    let mut copy = item.clone();
    assert_eq!(copy.uid(), item.uid());

    copy.set_uid(counter.next_uid());
    assert_ne!(copy.uid(), item.uid());
  }
}
