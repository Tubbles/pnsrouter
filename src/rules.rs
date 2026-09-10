// SPDX-License-Identifier: GPL-3.0-or-later

//! The design rule oracle, and a fixed clearance implementation of it.
//!
//! Port of `PNS::RULE_RESOLVER` (`pcbnew/router/pns_node.h:139`) together
//! with the two value types it hands back, `CONSTRAINT`
//! (`pcbnew/router/pns_node.h:73`) and `CONSTRAINT_TYPE`
//! (`pcbnew/router/pns_node.h:51`). Everything the router knows about
//! design rules arrives through this one trait, which is why note 05
//! section 2 calls it the part of the host contract an integrator is most
//! likely to get wrong.
//!
//! # Shape of the port
//!
//! Three deliberate changes, all asked for by `DESIGN.md` section 5 and
//! note 05 section 7.2:
//!
//! - [`RuleResolver::clearance`] answers `Option<i32>` where KiCad answers
//!   `int` with `-1` meaning "these two can never collide"
//!   (`pcbnew/router/pns_kicad_iface.cpp:968`). Every caller of the
//!   sentinel had to remember it; `None` cannot be added to a distance by
//!   accident.
//! - [`RuleResolver::is_keepout`] answers [`Keepout`] where KiCad answers
//!   a `bool` plus a `bool*` out parameter
//!   (`pcbnew/router/pns_node.h:165`). The out parameter is written only
//!   on a true return (`pcbnew/router/pns_kicad_iface.cpp:424`), so only
//!   three of the four combinations exist, and the short circuiting `||`
//!   at `pcbnew/router/pns_item.cpp:198` makes the fourth a latent bug.
//! - flashing is not here at all. `IsFlashedOnLayer` is a pure function of
//!   an item and a layer, so it is data on the item
//!   ([`crate::item::LayerMask`]) rather than a callback; note 05 section
//!   7.3 has the three effects that depend on it.
//!
//! # Why this module owns no cache
//!
//! KiCad's resolver carries three caches: `m_clearanceCache` keyed by the
//! two item **addresses** (`pcbnew/router/pns_kicad_iface.cpp:314`),
//! `m_tempClearanceCache` keyed by the properties of two router
//! temporaries (`:315`), and the hull cache keyed by
//! `(item address, clearance, thickness, layer)` (`:222`). All three are
//! pure memoisation of pure functions, and all three live on the **host**
//! side of the contract, which forces the host to implement
//! `ClearCaches`, `ClearCacheForItems` and `ClearTemporaryCaches`
//! correctly or silently serve a clearance computed for a freed item that
//! a new item now shares an address with
//! (`pcbnew/router/pns_router.cpp:766` is the invalidation call that
//! exists only because of that hazard).
//!
//! Note 05 section 7.2 and `DESIGN.md` section 5 both put the caches on
//! the engine instead. They belong to the node module (`src/node.rs`,
//! `DESIGN.md` section 9), because that is where the arena, the
//! [`crate::item::ItemId`] generations that make a stale key detectable,
//! and the item lifetime events that would invalidate an entry all live.
//! A host therefore implements a pure function and cannot get
//! invalidation wrong, and the three `Clear*` methods disappear from the
//! contract entirely. Nothing in this module memoises anything.
//!
//! # Not ported
//!
//! - `NetName` (`pcbnew/router/pns_node.h:152`), which only feeds display
//!   strings and `ITEM::Format` (`pcbnew/router/pns_item.cpp:347`). The
//!   engine never branches on it.
//! - `ClearCaches`, `ClearCacheForItems`, `ClearTemporaryCaches`
//!   (`pcbnew/router/pns_node.h:170` to `:172`), for the reason above.
//! - `HullCache` (`pcbnew/router/pns_node.h:176`), likewise. Its default
//!   implementation returns a reference to a function local static and is
//!   neither reentrant nor thread safe; note 02 section 7.5 calls it a
//!   correctness trap.

use crate::geometry::vec2::Vec2;
use crate::item::{Item, ItemId, Kind, NetId};

// ---------------------------------------------------------------------
// ItemRef
// ---------------------------------------------------------------------

/// One side of a rule query: an item, plus its identity when it has one.
///
/// Replaces the `const ITEM*` every `RULE_RESOLVER` method takes
/// (`pcbnew/router/pns_node.h:144` and following). A borrow carries
/// everything a rule needs to read; the [`ItemId`] is carried alongside
/// because a cache has to key on identity and an address is not one
/// (note 02 section 11).
///
/// `None` for the id means the item is not in any arena. The router
/// collides stack allocated items against stored ones all the time: the
/// line placer's head segment is one, and KiCad does the same thing with
/// a local `SEGMENT` (`pcbnew/router/pns_line_placer.cpp:1050`). Such an
/// item has no identity to cache on and can never be the same object as a
/// stored one.
#[derive(Copy, Clone, Debug)]
pub struct ItemRef<'a> {
  /// The item's handle, when it lives in an arena.
  id: Option<ItemId>,
  /// The item itself.
  item: &'a Item,
}

impl<'a> ItemRef<'a> {
  /// A reference to an item that lives in an arena.
  pub const fn stored(id: ItemId, item: &'a Item) -> Self {
    Self { id: Some(id), item }
  }

  /// A reference to an item that is not stored anywhere.
  ///
  /// The head the placer is dragging, a preview via, a probe segment in a
  /// test.
  pub const fn unstored(item: &'a Item) -> Self {
    Self { id: None, item }
  }

  /// The item's handle, or `None` when it is not stored.
  pub const fn id(self) -> Option<ItemId> {
    self.id
  }

  /// The item.
  pub const fn item(self) -> &'a Item {
    self.item
  }

  /// Whether the two references name the same object.
  ///
  /// Port of the `this == aHead` pointer comparison at
  /// `pcbnew/router/pns_item.cpp:119`. Two stored items are the same when
  /// their handles match, which the generation check makes exact; two
  /// borrows of one unstored item are the same when they point at the
  /// same place, which is the C++ test itself.
  pub fn is_same_as(self, other: ItemRef<'_>) -> bool {
    if let (Some(mine), Some(theirs)) = (self.id, other.id) {
      return mine == theirs;
    }

    std::ptr::eq(self.item, other.item)
  }
}

// ---------------------------------------------------------------------
// Keepout
// ---------------------------------------------------------------------

/// What a keepout rule says about one item meeting one obstacle.
///
/// Port of the return value and the `bool* aEnforce` out parameter of
/// `IsKeepout` (`pcbnew/router/pns_node.h:165`), whose consumer is
/// `pcbnew/router/pns_item.cpp:198`:
///
/// ```text
/// IsKeepout returns false            -> Keepout::None      (not a keepout)
/// IsKeepout returns true,  enforce=false -> Keepout::Present  (clearance -1)
/// IsKeepout returns true,  enforce=true  -> Keepout::Enforced (clearance 0)
/// ```
///
/// The fourth combination, a false return with `aEnforce` written, is
/// unrepresentable here. KiCad's reference implementation never produces
/// it (`pcbnew/router/pns_kicad_iface.cpp:424` writes the out parameter
/// only on the true path), but nothing in its type system says so.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum Keepout {
  /// The obstacle is not a keepout at all. The clearance ladder carries
  /// on to the next test.
  #[default]
  None,
  /// The obstacle is a keepout, but its rules do not exclude this item.
  /// The pair never collides.
  Present,
  /// The obstacle is a keepout whose rules exclude this item. The pair
  /// collides at the exact boundary, with no clearance at all.
  Enforced,
}

// ---------------------------------------------------------------------
// Constraints
// ---------------------------------------------------------------------

/// The kinds of design rule the router can ask for.
///
/// Port of `CONSTRAINT_TYPE`, `pcbnew/router/pns_node.h:51`, with KiCad's
/// discriminants so that a host bridging to a C++ rule engine can cast.
///
/// Note 05 section 7.5 works out that seven of the thirteen apply to
/// LibrePCB: [`ConstraintType::Clearance`], [`ConstraintType::Width`],
/// [`ConstraintType::ViaDiameter`], [`ConstraintType::ViaHole`],
/// [`ConstraintType::HoleClearance`], [`ConstraintType::EdgeClearance`]
/// and [`ConstraintType::HoleToHole`]. The rest are differential pair,
/// length tuning and physical clearance concepts it does not have.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
#[repr(i32)]
pub enum ConstraintType {
  /// Copper to copper clearance. `CT_CLEARANCE`.
  Clearance = 1,
  /// The gap inside a differential pair. `CT_DIFF_PAIR_GAP`.
  DiffPairGap = 2,
  /// A tuned length. `CT_LENGTH`.
  Length = 3,
  /// Track width. `CT_WIDTH`.
  Width = 4,
  /// Via copper diameter. `CT_VIA_DIAMETER`.
  ViaDiameter = 5,
  /// Via drill diameter. `CT_VIA_HOLE`.
  ViaHole = 6,
  /// Hole to copper clearance. `CT_HOLE_CLEARANCE`.
  HoleClearance = 7,
  /// Copper to board edge clearance. `CT_EDGE_CLEARANCE`.
  EdgeClearance = 8,
  /// Hole to hole clearance. `CT_HOLE_TO_HOLE`.
  HoleToHole = 9,
  /// The allowed length difference inside a differential pair.
  /// `CT_DIFF_PAIR_SKEW`.
  DiffPairSkew = 10,
  /// The longest uncoupled run a differential pair may have.
  /// `CT_MAX_UNCOUPLED`.
  MaxUncoupled = 11,
  /// A net blind physical clearance. `CT_PHYSICAL_CLEARANCE`.
  PhysicalClearance = 12,
  /// A net blind physical hole clearance.
  /// `CT_PHYSICAL_HOLE_CLEARANCE`.
  PhysicalHoleClearance = 13,
}

/// One design rule value.
///
/// Port of `CONSTRAINT`, `pcbnew/router/pns_node.h:73`. KiCad carries a
/// `MINOPTMAX<int>` plus the rule name, the from and to names and a time
/// domain flag; the three names and the flag only ever reach a status bar
/// message, so they are not ported. `MINOPTMAX` becomes three
/// independent `Option`s, which is what its `HasMin`, `HasOpt` and
/// `HasMax` predicates test.
///
/// The clearance path reads `m_Value.Min()` and nothing else
/// (`pcbnew/router/pns_kicad_iface.cpp:912` and following); `ImportSizes`
/// is the one caller that reads `Opt` as well.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Constraint {
  /// Which rule this is.
  pub constraint_type: ConstraintType,
  /// The smallest permitted value, the only one the clearance path reads.
  pub min: Option<i32>,
  /// The preferred value, which `ImportSizes` reads for track and via
  /// sizes.
  pub opt: Option<i32>,
  /// The largest permitted value.
  pub max: Option<i32>,
  /// Whether the rule permits the thing at all. Port of `m_Allowed`.
  pub allowed: bool,
}

// ---------------------------------------------------------------------
// The trait
// ---------------------------------------------------------------------

/// The design rule oracle.
///
/// Port of `PNS::RULE_RESOLVER`, `pcbnew/router/pns_node.h:139`. See the
/// module documentation for the three changes from the C++ shape and for
/// why the caches are not here.
///
/// A method with a default answers "this host does not have that
/// concept", which is what note 05 section 2.11 lists as the minimal
/// subset for a host with plain net class rules. The five required
/// methods are the ones no default can guess.
pub trait RuleResolver {
  /// The clearance required between two items, in nanometres.
  ///
  /// Port of `Clearance`, `pcbnew/router/pns_node.h:144`. `None` is
  /// KiCad's negative return, "these two can never collide".
  ///
  /// The answer is the worst case over the layers the two items share,
  /// not a per layer value: KiCad's implementation loops over the
  /// intersected layer range taking a maximum
  /// (`pcbnew/router/pns_kicad_iface.cpp:905`) and its cache key does not
  /// mention a layer (note 05 section 2.2).
  ///
  /// `b` is `None` for the one sided queries that ask what an item needs
  /// against anything at all.
  ///
  /// `use_epsilon` asks for [`RuleResolver::clearance_epsilon`] to be
  /// subtracted from a positive answer. The router turns it off where it
  /// needs the strict rule: `VIA::PushoutForce`
  /// (`pcbnew/router/pns_via.cpp:158`) and the walkaround's hull query
  /// (`pcbnew/router/pns_walkaround.cpp:156`).
  fn clearance(
    &self,
    a: ItemRef<'_>,
    b: Option<ItemRef<'_>>,
    use_epsilon: bool,
  ) -> Option<i32>;

  /// The slack subtracted from a positive clearance.
  ///
  /// Port of `ClearanceEpsilon`, `pcbnew/router/pns_node.h:174`. It stops
  /// geometry sitting exactly at the rule limit from reading as a
  /// violation. KiCad takes it from the board's DRC epsilon
  /// (`pcbnew/router/pns_kicad_iface.cpp:338`); note 05 section 7.5 says
  /// LibrePCB has no such notion and should leave it at zero, which makes
  /// the router marginally stricter, the safe direction.
  fn clearance_epsilon(&self) -> i32 {
    0
  }

  /// Whether the host has a net blind physical clearance rule.
  ///
  /// Port of `HasUserDefinedPhysicalConstraint`,
  /// `pcbnew/router/pns_node.h:145`. When it answers true the collision
  /// ladder may not take its same net and free pad short circuits,
  /// because a physical rule applies regardless of nets
  /// (`pcbnew/router/pns_item.cpp:127`, `:188`, `:193`).
  ///
  /// KiCad memoises it because it runs in the collision inner loop and
  /// otherwise walks the whole DRC rule map
  /// (`pcbnew/router/pns_kicad_iface.cpp:310`). A host should make it a
  /// field read.
  fn has_user_defined_physical_constraint(&self) -> bool {
    false
  }

  /// A design rule of a given kind, if the host has one.
  ///
  /// Port of `QueryConstraint`, `pcbnew/router/pns_node.h:167`.
  ///
  /// Defaulted to "no rule" because no part of the routing core calls it:
  /// its only two callers in KiCad's tree are the length tuning placer
  /// (`pcbnew/router/pns_meander_placer_base.cpp:122`) and the host tool
  /// reading a differential pair's uncoupled limit
  /// (`pcbnew/router/router_tool.cpp:3475`), neither of which is in
  /// milestone 1. A host that implements nothing here still routes; it
  /// only loses length tuning.
  fn constraint(
    &self,
    _constraint_type: ConstraintType,
    _a: ItemRef<'_>,
    _b: Option<ItemRef<'_>>,
    _layer: i32,
  ) -> Option<Constraint> {
    None
  }

  /// Whether an obstacle is a keepout, and whether it excludes this item.
  ///
  /// Port of `IsKeepout`, `pcbnew/router/pns_node.h:165`. See [`Keepout`]
  /// for the mapping from KiCad's two return values.
  fn is_keepout(&self, obstacle: ItemRef<'_>, item: ItemRef<'_>) -> Keepout;

  /// Whether the item is a plated drilled hole.
  ///
  /// Port of `IsDrilledHole`, `pcbnew/router/pns_node.h:158`. It picks
  /// hole to hole clearance over hole to copper clearance
  /// (`pcbnew/router/pns_kicad_iface.cpp:908`). KiCad's implementation
  /// requires the item to be of kind hole and then resolves the parent
  /// through `ParentPadVia` when the hole has no board item of its own
  /// (`pcbnew/router/pns_kicad_iface.cpp:471`).
  fn is_drilled_hole(&self, item: ItemRef<'_>) -> bool;

  /// Whether the item is a non plated slot.
  ///
  /// Port of `IsNonPlatedSlot`, `pcbnew/router/pns_node.h:159`, defined
  /// narrowly as a hole whose parent pad is NPTH **and** whose drill is
  /// not round (`pcbnew/router/pns_kicad_iface.cpp:494`). It enables the
  /// castellation path in the collision ladder
  /// (`pcbnew/router/pns_item.cpp:230`), which is the slow path that
  /// needs the collision position.
  fn is_non_plated_slot(&self, item: ItemRef<'_>) -> bool;

  /// A small integer identifier for a net.
  ///
  /// Port of `NetCode`, `pcbnew/router/pns_node.h:151`. Values of zero or
  /// less are treated as "no net" by the topology code
  /// (`pcbnew/router/pns_topology.cpp:123`), so a host whose own net
  /// numbering starts at zero has to offset it.
  fn net_code(&self, net: NetId) -> i32;

  /// The net a route started in free space is placed on.
  ///
  /// Port of `ROUTER_IFACE::GetOrphanedNetHandle`
  /// (`pcbnew/router/pns_router.h:118`), whose KiCad implementation hands
  /// back the process wide `NETINFO_LIST::OrphanedItem()`
  /// (`pcbnew/router/pns_kicad_iface.cpp:3020`). It has exactly one
  /// caller, `LINE_PLACER::Start`
  /// (`pcbnew/router/pns_line_placer.cpp:1386`), which reads
  /// `aStartItem ? aStartItem->Net() : GetOrphanedNetHandle()`.
  ///
  /// # Contract
  ///
  /// - The value is **stable** for the life of the resolver. KiCad's is a
  ///   function local static, so every free space route in a session
  ///   shares one handle.
  /// - [`RuleResolver::net_code`] of it is **zero or less**, so the
  ///   topology code reads it as "no net"
  ///   (`pcbnew/router/pns_topology.cpp:123`) and the Mark Obstacles net
  ///   adoption at `pcbnew/router/pns_line_placer.cpp:1571` lets the end
  ///   item's net win. KiCad's orphan carries `NETINFO_LIST::UNCONNECTED`,
  ///   which is zero (`pcbnew/netinfo.h:272`,
  ///   `pcbnew/netinfo_list.cpp:315`).
  /// - It **differs from every net a snapshot item carries**. KiCad's
  ///   `OrphanedItem()` is a separate `NETINFO_ITEM` from the board's own
  ///   unconnected net even though the two share a net code, and the
  ///   router compares handles rather than codes, so a free space head
  ///   still keeps clearance from unconnected copper
  ///   (`pcbnew/router/pns_item.cpp:188`).
  ///
  /// The first two make the head of a free space route and its own
  /// already fixed tail "same net", which is what lets a shove push an
  /// obstacle instead of the placer falling back to the walkaround; the
  /// third stops that exemption from spreading to the board.
  ///
  /// # Why it lives on this trait
  ///
  /// KiCad hangs it off `ROUTER_IFACE`, reached through
  /// `Router()->GetInterface()`. This crate has no router singleton
  /// (`DESIGN.md` section 8), and the uses of `ROUTER::GetInstance()`
  /// became [`crate::algo_base::AlgoContext`] fields instead. The
  /// resolver is the one of those the host supplies, so the host's net
  /// vocabulary is asked for here rather than through a second trait.
  fn orphaned_net(&self) -> NetId;

  /// Whether the item belongs to a net tie footprint.
  ///
  /// Port of `IsInNetTie`, `pcbnew/router/pns_node.h:154`. A true answer
  /// forces the collision ladder onto the slow path that computes the
  /// collision position (`pcbnew/router/pns_item.cpp:232`), because a net
  /// tie exclusion depends on where the collision is.
  ///
  /// Defaulted to false: note 05 section 2.4 records that a host without
  /// net ties answers false here and gets the fast path everywhere.
  fn is_in_net_tie(&self, _item: ItemRef<'_>) -> bool {
    false
  }

  /// Whether one particular collision is forgiven by a net tie.
  ///
  /// Port of `IsNetTieExclusion`, `pcbnew/router/pns_node.h:155`.
  /// Defaulted to false, as above.
  fn is_net_tie_exclusion(
    &self,
    _item: ItemRef<'_>,
    _collision_position: Vec2,
    _colliding_item: ItemRef<'_>,
  ) -> bool {
    false
  }

  /// The other half of a differential pair's net.
  ///
  /// Port of `DpCoupledNet`, `pcbnew/router/pns_node.h:147`. Defaulted to
  /// "not supported": differential pair placement then fails cleanly with
  /// "cannot start a differential pair"
  /// (`pcbnew/router/pns_router.cpp:352`) rather than misbehaving, which
  /// is what note 05 section 2.7 asks for.
  fn dp_coupled_net(&self, _net: NetId) -> Option<NetId> {
    None
  }

  /// Which half of a differential pair a net is.
  ///
  /// Port of `DpNetPolarity`, `pcbnew/router/pns_node.h:148`: positive
  /// for P, negative for N, and `AssembleDiffPair` swaps the two lines
  /// when it is negative (`pcbnew/router/pns_topology.cpp:1160`).
  /// Defaulted to zero, "not supported".
  fn dp_net_polarity(&self, _net: NetId) -> i32 {
    0
  }

  /// The positive and negative nets of the pair an item belongs to.
  ///
  /// Port of `DpNetPair`, `pcbnew/router/pns_node.h:149`. Defaulted to
  /// "not supported".
  fn dp_net_pair(&self, _item: ItemRef<'_>) -> Option<(NetId, NetId)> {
    None
  }
}

// ---------------------------------------------------------------------
// FixedClearance
// ---------------------------------------------------------------------

/// A resolver of three fixed numbers.
///
/// It is what the collision tests need and what note 05 section 2.11
/// calls the minimal subset for a host whose rules are a table lookup.
/// The three fields are exactly the branches the clearance ladder of
/// `PNS_PCBNEW_RULE_RESOLVER::Clearance`
/// (`pcbnew/router/pns_kicad_iface.cpp:865`) can take once the rules that
/// LibrePCB does not have are removed, in the arrangement note 05 section
/// 7.5 works out:
///
/// ```text
/// both sides drilled holes        -> hole_to_hole_clearance
/// exactly one side a hole,
///   and the nets differ           -> hole_clearance
/// neither side a hole,
///   nets differ, no free pad      -> clearance
/// ```
///
/// The three are combined by `max`, not by first match, because KiCad's
/// loop takes a maximum over every rule that applies and a plated hole
/// deliberately gets both its hole rule and the copper rule ("No 'else';
/// plated holes get both HOLE_CLEARANCE and CLEARANCE",
/// `pcbnew/router/pns_kicad_iface.cpp:928`).
///
/// # Deviations from the KiCad resolver
///
/// - "Copper" means "not a hole" here. KiCad's `isCopper`
///   (`pcbnew/router/pns_kicad_iface.cpp:435`) asks the parent board item
///   and therefore answers **true** for a hole the router synthesised,
///   which has no parent. There is no board item to ask here, and note 05
///   section 7.5 models holes as their own case anyway.
/// - Edge clearance and the two physical clearance rules have no field.
///   A host that needs them implements [`RuleResolver`] itself; this type
///   is deliberately the simplest thing that exercises every branch of
///   the ladder.
/// - [`FixedClearance::is_drilled_hole`] answers "every hole is a plated
///   drilled hole", because there is no plating data to consult.
/// - [`FixedClearance::is_non_plated_slot`] answers false, so the
///   castellation path is never taken.
/// - A hole answers for its own net and its own free pad flag, where
///   `HOLE::Net` (`pcbnew/router/pns_hole.h:56`) and `ITEM::IsFreePad`
///   (`pcbnew/router/pns_item.h:288`) both forward to the parent pad or
///   via. Following that handle needs the arena, which a resolver does
///   not have. It costs nothing in practice: the collision ladder in
///   [`crate::collide`] applies both rules with the parent folded in
///   before it ever asks a resolver, and the world sets a hole's net when
///   it adds one (see `Item::net`). A host that wants the parent's answer
///   inside its own rules has to carry the link on its side.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct FixedClearance {
  /// Copper to copper, between items of different nets.
  pub clearance: i32,
  /// Hole to copper, when the nets differ. `None` means the host has no
  /// such rule, so a hole against copper falls back to whatever else
  /// applies.
  pub hole_clearance: Option<i32>,
  /// Hole to hole, whatever the nets. `None` means the host has no such
  /// rule.
  pub hole_to_hole_clearance: Option<i32>,
  /// The slack subtracted from a positive answer. See
  /// [`RuleResolver::clearance_epsilon`].
  pub clearance_epsilon: i32,
}

impl FixedClearance {
  /// One clearance for copper, holes included, and no epsilon.
  pub const fn uniform(clearance: i32) -> Self {
    Self {
      clearance,
      hole_clearance: Some(clearance),
      hole_to_hole_clearance: Some(clearance),
      clearance_epsilon: 0,
    }
  }
}

impl RuleResolver for FixedClearance {
  /// The ladder of `PNS_PCBNEW_RULE_RESOLVER::Clearance`,
  /// `pcbnew/router/pns_kicad_iface.cpp:865`, with a constant in place of
  /// each `QueryConstraint` call and with the layer loop collapsed: every
  /// rule here is the same on every layer, so the maximum over the shared
  /// layers is the maximum over one of them.
  ///
  /// The two tails are reproduced exactly: a same net or free pad pair
  /// whose rules all came out zero answers `None`, KiCad's `-1` at
  /// `:968`, and the epsilon is subtracted from a positive answer and
  /// clamped at zero (`:971`).
  fn clearance(
    &self,
    a: ItemRef<'_>,
    b: Option<ItemRef<'_>>,
    use_epsilon: bool,
  ) -> Option<i32> {
    let a_is_hole = self.is_drilled_hole(a);
    let b_is_hole = b.is_some_and(|b| self.is_drilled_hole(b));

    // `sameNet` at :902. A null net is never the same as anything, not
    // even as another null net.
    let same_net = b.is_some_and(|b| {
      a.item().net().is_some() && a.item().net() == b.item().net()
    });
    // `freePad` at :903, on the item's own flag only; see the type
    // documentation for why a hole's parent cannot be consulted here.
    let free_pad =
      b.is_some_and(|b| a.item().is_free_pad() || b.item().is_free_pad());

    let mut result = 0;

    if a_is_hole && b_is_hole {
      // :908
      result = result.max(self.hole_to_hole_clearance.unwrap_or(0));
    } else if (a_is_hole || b_is_hole) && !same_net {
      // :916
      result = result.max(self.hole_clearance.unwrap_or(0));
    }

    // :929, an independent `if` and not an `else`, so KiCad's own
    // resolver lets a plated hole pick up the copper rule on top of its
    // hole rule. Here "copper" excludes holes, so the two are in fact
    // exclusive; that deviation is on the type documentation.
    if !a_is_hole && !b_is_hole && !same_net && !free_pad {
      result = result.max(self.clearance);
    }

    if (same_net || free_pad) && result == 0 {
      // :968
      return None;
    }

    if use_epsilon && result > 0 {
      // :971
      result = (result - self.clearance_epsilon).max(0);
    }

    Some(result)
  }

  fn clearance_epsilon(&self) -> i32 {
    self.clearance_epsilon
  }

  /// Never a keepout: this resolver has no zones.
  fn is_keepout(&self, _obstacle: ItemRef<'_>, _item: ItemRef<'_>) -> Keepout {
    Keepout::None
  }

  /// Every hole. See the type documentation.
  fn is_drilled_hole(&self, item: ItemRef<'_>) -> bool {
    item.item().kind() == Kind::HOLE
  }

  /// Never. See the type documentation.
  fn is_non_plated_slot(&self, _item: ItemRef<'_>) -> bool {
    false
  }

  /// The net's own number, saturated.
  ///
  /// A host whose nets start at zero has to offset them, because zero and
  /// below mean "no net" to the topology code; see
  /// [`RuleResolver::net_code`].
  fn net_code(&self, net: NetId) -> i32 {
    i32::try_from(net.0).unwrap_or(i32::MAX)
  }

  /// `NetId(0)`, whose [`RuleResolver::net_code`] above is already zero.
  ///
  /// This resolver's convention is that a real net numbers from one, so
  /// net zero is free to be the orphan and no special case is needed. A
  /// host that numbers its own nets from zero has to offset them anyway,
  /// which is the same requirement [`RuleResolver::net_code`] states, and
  /// once it has, nothing in a snapshot carries `NetId(0)`. KiCad's
  /// orphan carries the same code, `NETINFO_LIST::UNCONNECTED`
  /// (`pcbnew/netinfo.h:272`).
  fn orphaned_net(&self) -> NetId {
    NetId(0)
  }
}

// ---------------------------------------------------------------------
// CoupledNets
// ---------------------------------------------------------------------

/// A [`FixedClearance`] that also knows about one differential pair.
///
/// KiCad's answer to "which two nets form a pair" is a net **name**
/// convention and nothing else: `BOARD::MatchDpSuffix`
/// (`pcbnew/board.cpp:2780`) walks a net name backwards over digits and
/// underscores and pairs it with the name that differs in the one
/// `P`/`N` or `+`/`-` character it stops at. There is no pair object on a
/// KiCad board and no pair property on a net.
///
/// This crate has no net names, so it cannot have that convention, and
/// note 07 section 15.2 says why none is invented: the engine's contract
/// is that the **host** says which nets are coupled, through the three
/// hooks below. This is the smallest host that says so, and it is what
/// the crate's own differential pair tests answer with. A real host
/// answers from whatever its board model calls a pair.
///
/// It answers the pair hooks and forwards everything else, so the
/// clearance the two lanes are tested against is the ordinary one: there
/// is no suppression of P against N collisions anywhere in the router,
/// and note 07 section 8.3 spells out the consequence, that a clearance
/// above the configured pair gap makes every pair collide with itself.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct CoupledNets {
  /// The clearances, and every rule that is not about pairs.
  pub inner: FixedClearance,
  /// The positive half.
  pub net_p: NetId,
  /// The negative half.
  pub net_n: NetId,
}

impl CoupledNets {
  /// One pair over one clearance.
  pub const fn new(inner: FixedClearance, net_p: NetId, net_n: NetId) -> Self {
    Self {
      inner,
      net_p,
      net_n,
    }
  }
}

impl RuleResolver for CoupledNets {
  fn clearance(
    &self,
    a: ItemRef<'_>,
    b: Option<ItemRef<'_>>,
    use_epsilon: bool,
  ) -> Option<i32> {
    self.inner.clearance(a, b, use_epsilon)
  }

  fn clearance_epsilon(&self) -> i32 {
    self.inner.clearance_epsilon()
  }

  fn is_keepout(&self, obstacle: ItemRef<'_>, item: ItemRef<'_>) -> Keepout {
    self.inner.is_keepout(obstacle, item)
  }

  fn is_drilled_hole(&self, item: ItemRef<'_>) -> bool {
    self.inner.is_drilled_hole(item)
  }

  fn is_non_plated_slot(&self, item: ItemRef<'_>) -> bool {
    self.inner.is_non_plated_slot(item)
  }

  fn net_code(&self, net: NetId) -> i32 {
    self.inner.net_code(net)
  }

  fn orphaned_net(&self) -> NetId {
    self.inner.orphaned_net()
  }

  /// The other half, `BOARD::DpCoupledNet` (`pcbnew/board.cpp:2828`).
  fn dp_coupled_net(&self, net: NetId) -> Option<NetId> {
    if net == self.net_p {
      Some(self.net_n)
    } else if net == self.net_n {
      Some(self.net_p)
    } else {
      None
    }
  }

  /// The sign `MatchDpSuffix` returns: `1` for the positive half, `-1`
  /// for the negative one, `0` for a net that is not part of a pair.
  fn dp_net_polarity(&self, net: NetId) -> i32 {
    if net == self.net_p {
      1
    } else if net == self.net_n {
      -1
    } else {
      0
    }
  }

  /// Both halves, positive first, whichever half the item belongs to.
  ///
  /// That normalisation is `PNS_PCBNEW_RULE_RESOLVER::DpNetPair`'s
  /// (`pcbnew/router/pns_kicad_iface.cpp:1376`) and it is why the pair
  /// placer needs no [`RuleResolver::dp_net_polarity`].
  fn dp_net_pair(&self, item: ItemRef<'_>) -> Option<(NetId, NetId)> {
    let net = item.item().net()?;

    (net == self.net_p || net == self.net_n).then_some((self.net_p, self.net_n))
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::shape::Shape;
  use crate::geometry::vec2::Vec2;
  use crate::item::{Hole, Item, ItemBody, LayerRange, Solid};

  /// A pad on one layer, on a net.
  fn pad(uid: u64, net: Option<NetId>) -> Item {
    let shape = Shape::Circle {
      center: Vec2::new(0, 0),
      radius: 1000,
    };
    let mut item =
      Item::new(uid, ItemBody::Solid(Solid::new(shape, shape_pos())));
    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);

    item
  }

  /// The centre every test shape sits on.
  fn shape_pos() -> Vec2 {
    Vec2::new(0, 0)
  }

  /// A hole on one layer, on a net.
  fn hole(uid: u64, net: Option<NetId>) -> Item {
    let mut item =
      Item::new(uid, ItemBody::Hole(Hole::circular(shape_pos(), 500)));
    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);

    item
  }

  #[test]
  fn uniform_uses_one_number_everywhere() {
    let rules = FixedClearance::uniform(2000);

    assert_eq!(rules.clearance, 2000);
    assert_eq!(rules.hole_clearance, Some(2000));
    assert_eq!(rules.hole_to_hole_clearance, Some(2000));
    assert_eq!(rules.clearance_epsilon(), 0);
  }

  #[test]
  fn copper_to_copper_on_different_nets_takes_the_copper_rule() {
    let rules = FixedClearance::uniform(2000);
    let a = pad(0, Some(NetId(1)));
    let b = pad(1, Some(NetId(2)));

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&a),
        Some(ItemRef::unstored(&b)),
        false
      ),
      Some(2000)
    );
  }

  #[test]
  fn copper_on_the_same_net_never_collides() {
    let rules = FixedClearance::uniform(2000);
    let a = pad(0, Some(NetId(1)));
    let b = pad(1, Some(NetId(1)));

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&a),
        Some(ItemRef::unstored(&b)),
        false
      ),
      None
    );
  }

  #[test]
  fn two_items_without_a_net_are_not_on_the_same_net() {
    let rules = FixedClearance::uniform(2000);
    let a = pad(0, None);
    let b = pad(1, None);

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&a),
        Some(ItemRef::unstored(&b)),
        false
      ),
      Some(2000)
    );
  }

  #[test]
  fn a_free_pad_never_collides() {
    let rules = FixedClearance::uniform(2000);
    let mut a = pad(0, Some(NetId(1)));
    a.set_is_free_pad(true);
    let b = pad(1, Some(NetId(2)));

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&a),
        Some(ItemRef::unstored(&b)),
        false
      ),
      None
    );
  }

  #[test]
  fn a_hole_against_copper_takes_the_hole_rule() {
    let rules = FixedClearance {
      clearance: 2000,
      hole_clearance: Some(3000),
      hole_to_hole_clearance: Some(4000),
      clearance_epsilon: 0,
    };
    let a = hole(0, Some(NetId(1)));
    let b = pad(1, Some(NetId(2)));

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&a),
        Some(ItemRef::unstored(&b)),
        false
      ),
      Some(3000)
    );
  }

  #[test]
  fn a_hole_against_copper_on_the_same_net_drops_the_hole_rule() {
    let rules = FixedClearance {
      clearance: 2000,
      hole_clearance: Some(3000),
      hole_to_hole_clearance: Some(4000),
      clearance_epsilon: 0,
    };
    let a = hole(0, Some(NetId(1)));
    let b = pad(1, Some(NetId(1)));

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&a),
        Some(ItemRef::unstored(&b)),
        false
      ),
      None
    );
  }

  #[test]
  fn two_holes_take_the_hole_to_hole_rule_whatever_the_nets() {
    let rules = FixedClearance {
      clearance: 2000,
      hole_clearance: Some(3000),
      hole_to_hole_clearance: Some(4000),
      clearance_epsilon: 0,
    };
    let a = hole(0, Some(NetId(1)));
    let b = hole(1, Some(NetId(1)));

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&a),
        Some(ItemRef::unstored(&b)),
        false
      ),
      Some(4000)
    );
  }

  #[test]
  fn a_missing_hole_rule_contributes_nothing() {
    let rules = FixedClearance {
      clearance: 2000,
      hole_clearance: None,
      hole_to_hole_clearance: None,
      clearance_epsilon: 0,
    };
    let a = hole(0, Some(NetId(1)));
    let b = hole(1, Some(NetId(2)));

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&a),
        Some(ItemRef::unstored(&b)),
        false
      ),
      Some(0)
    );
  }

  #[test]
  fn the_epsilon_is_subtracted_only_when_asked_for() {
    let rules = FixedClearance {
      clearance: 2000,
      hole_clearance: None,
      hole_to_hole_clearance: None,
      clearance_epsilon: 150,
    };
    let a = pad(0, Some(NetId(1)));
    let b = pad(1, Some(NetId(2)));

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&a),
        Some(ItemRef::unstored(&b)),
        false
      ),
      Some(2000)
    );
    assert_eq!(
      rules.clearance(ItemRef::unstored(&a), Some(ItemRef::unstored(&b)), true),
      Some(1850)
    );
  }

  #[test]
  fn the_epsilon_never_pushes_a_clearance_below_zero() {
    let rules = FixedClearance {
      clearance: 100,
      hole_clearance: None,
      hole_to_hole_clearance: None,
      clearance_epsilon: 5000,
    };
    let a = pad(0, Some(NetId(1)));
    let b = pad(1, Some(NetId(2)));

    assert_eq!(
      rules.clearance(ItemRef::unstored(&a), Some(ItemRef::unstored(&b)), true),
      Some(0)
    );
  }

  #[test]
  fn a_one_sided_query_has_no_net_and_no_free_pad_short_circuit() {
    let rules = FixedClearance::uniform(2000);
    let mut a = pad(0, Some(NetId(1)));
    a.set_is_free_pad(true);

    assert_eq!(
      rules.clearance(ItemRef::unstored(&a), None, false),
      Some(2000)
    );
  }

  #[test]
  fn the_defaulted_methods_answer_not_supported() {
    let rules = FixedClearance::uniform(2000);
    let a = pad(0, Some(NetId(1)));
    let reference = ItemRef::unstored(&a);

    assert!(!rules.has_user_defined_physical_constraint());
    assert_eq!(
      rules.constraint(ConstraintType::Clearance, reference, None, 0),
      None
    );
    assert!(!rules.is_in_net_tie(reference));
    assert!(!rules.is_net_tie_exclusion(reference, Vec2::new(0, 0), reference));
    assert_eq!(rules.dp_coupled_net(NetId(1)), None);
    assert_eq!(rules.dp_net_polarity(NetId(1)), 0);
    assert_eq!(rules.dp_net_pair(reference), None);
    assert_eq!(rules.is_keepout(reference, reference), Keepout::None);
    assert!(!rules.is_non_plated_slot(reference));
    assert_eq!(rules.net_code(NetId(7)), 7);
  }

  #[test]
  fn only_a_hole_is_a_drilled_hole() {
    let rules = FixedClearance::uniform(2000);
    let a = pad(0, None);
    let b = hole(1, None);

    assert!(!rules.is_drilled_hole(ItemRef::unstored(&a)));
    assert!(rules.is_drilled_hole(ItemRef::unstored(&b)));
  }

  #[test]
  fn an_unstored_reference_is_only_the_same_as_itself() {
    let a = pad(0, None);
    let b = pad(1, None);

    assert!(ItemRef::unstored(&a).is_same_as(ItemRef::unstored(&a)));
    assert!(!ItemRef::unstored(&a).is_same_as(ItemRef::unstored(&b)));
  }
}
