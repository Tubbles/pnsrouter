// SPDX-License-Identifier: GPL-3.0-or-later

//! The item level collision test.
//!
//! Port of `PNS::ITEM::Collide` (`pcbnew/router/pns_item.cpp:305`), the
//! private `ITEM::collideSimple` that does all of its work
//! (`pcbnew/router/pns_item.cpp:104`) and the self collision pruning of
//! `shouldWeConsiderHoleCollisions` (`pcbnew/router/pns_item.cpp:38`).
//!
//! Where [`crate::geometry::collision`] answers "are these two shapes
//! closer than this many nanometres", this module answers "do these two
//! **items** violate a design rule": it resolves the clearance through
//! the [`RuleResolver`], skips the pairs that are exempt, and expands a
//! pad or via into its hole. Read the first argument as the candidate
//! obstacle and the second as the head, the thing being routed, which is
//! how KiCad names them.
//!
//! # The clearance ladder
//!
//! `collideSimple` resolves the clearance through a chain of `else if`s
//! (`pcbnew/router/pns_item.cpp:188` to `:222`), first match wins, and a
//! negative result means "these two can never collide". The chain is
//! transcribed in `resolve_clearance` with a line citation per rung. It
//! is the single place a routing engine decides what counts as a
//! violation, so every rung matters and none of them may be reordered.
//!
//! # The extra nanometre
//!
//! The shape test is asked for `clearance - 1` and not for `clearance`
//! (`pcbnew/router/pns_item.cpp:249`, `:280`), because "the hulls are
//! built to exactly the clearance distance, so we need to allow for no
//! collision when exactly at the clearance distance". Combined with the
//! strict comparison in the shape layer, the effect is that two items
//! whose edges are `clearance - 1` nanometres apart do **not** collide
//! and two items `clearance - 2` apart do. The walkaround's termination
//! depends on the hull builders and this predicate agreeing to the
//! nanometre (`DESIGN.md` section 2), so the `- 1` is transplanted rather
//! than tidied.
//!
//! # The two shapes of the head
//!
//! KiCad's `LINE` is an `ITEM`, so `collideSimple` reaches it through the
//! same pointer as everything else and recovers it with two `dyn_cast`s
//! (`pcbnew/router/pns_item.cpp:133`, `:139`). A [`Line`] is a value here
//! and not an [`Item`] (`DESIGN.md` section 4.2), so the head side is a
//! private `Head` enum instead: either an ordinary [`ItemRef`] or a
//! [`LineHead`], which carries the three things the test reads off a
//! `LINE`. Both go through one ladder, and a line head adds exactly what
//! KiCad's two `LINE` branches add:
//!
//! - **The via pass** (`:139`). A line that ends with a via collides
//!   through that via as well, as a separate call with the via in the
//!   obstacle position and this item in the head position, which is the
//!   argument order `line->Via().collideSimple( this, ... )` produces.
//! - **Half the line width** (`:159`), folded into the clearance the
//!   shape test is asked for, because the shape routines ignore the width
//!   a chain carries. The truncation of `Width() / 2` is KiCad's.
//!
//! The obstacle side is never a line: lines are never stored, so nothing
//! a spatial index hands back can be one. KiCad's `:133` branch, the
//! mirror image, therefore has no counterpart here; its callers are the
//! shove, the optimizer and the multi dragger, which pass a `LINE` as the
//! **receiver** of `ITEM::Collide` and arrive with milestone 3.
//!
//! # What is deliberately not here
//!
//! - **The castellation exclusion** (`pcbnew/router/pns_item.cpp:251`).
//!   Its two halves are a host question, "is this item on the board
//!   edge", and a node query, `NODE::QueryEdgeExclusions`
//!   (`pcbnew/router/pns_node.cpp:801`), which needs the edge exclusion
//!   shapes the snapshot carries. Only the
//!   [`RuleResolver::is_non_plated_slot`] half of the trigger at `:229`
//!   is here, and it still selects the slow path, so the obstacle comes
//!   back with the collision position the node needs to finish the test.
//!   Until the node module exists, such a collision is reported where
//!   KiCad might suppress it.
//!
//! # Both forms of the search
//!
//! Both forms of the search are here. [`collide_items`] is the early
//! returning one, which stops at the first hit and loops the layers
//! itself. [`collide_into`] is the accumulating one KiCad reaches with a
//! `COLLISION_SEARCH_CONTEXT` (`pcbnew/router/pns_item.cpp:257`, `:282`):
//! it takes one layer, appends every obstacle it finds and answers only
//! whether it found any, which is what `NODE::QueryColliding` needs.
//!
//! # The geometric self collision heuristic, and why it is gone
//!
//! `shouldWeConsiderHoleCollisions` used to prune a via's hole against a
//! geometrically identical copy of itself (`pcbnew/router/pns_item.cpp:65`
//! to `:69`): same position, same padstack, same net, same drill. The
//! comment above it says why it is a heuristic and not an identity test:
//! a `LINE` carries a **copy** of its via (`VIA::Clone`,
//! `pcbnew/router/pns_via.cpp:278`, gives the copy a hole of its own), so
//! checking a line against a node that already holds that via would
//! otherwise report the via's hole colliding with itself.
//!
//! That reason does not exist here, which is what `DESIGN.md` sections
//! 4.2 and 11 predicted and what the fixtures in this module and in
//! `src/node.rs` now show:
//!
//! - a [`crate::line::LineVia::Linked`] via is literally the same arena
//!   item as the node's, so the self test at `:119` prunes the via pass
//!   and the parent test at `:77` prunes the hole pass;
//! - a [`crate::line::LineVia::Owned`] via owns no hole, because a hole
//!   is a separate arena item and an unstored via has nothing in the
//!   arena. `World::add_via` drills one when the via is stored, and not
//!   before. So the hole to hole branch is never even entered for a line,
//!   and the copper pass is exempted by the same net rung of the ladder,
//!   exactly as KiCad's is.
//!
//! Retiring it changes one answer, and only one: two **distinct stored**
//! vias at the same position, with the same padstack, net and drill, now
//! report the hole to hole collision that KiCad suppresses. That is not a
//! self collision at all, it is two real objects sharing one hole, and
//! KiCad only suppresses it because an address cannot tell it apart from
//! the line copy case. The deviation is recorded here and in the log.
//!
//! # Which layers are tested
//!
//! KiCad gets a layer context from the spatial index: `INDEX::Query`
//! loops over the query item's layers and hands each sub index's layer
//! down to the visitor (`pcbnew/router/pns_index.h:172`), which passes it
//! to `Collide` (`pcbnew/router/pns_node.cpp:256`). There is no index
//! yet, so [`collide_items`] loops over
//! [`Item::relevant_shape_layers`] instead, which is the same idiom
//! `VIA::PushoutForce` uses when it has no index either
//! (`pcbnew/router/pns_via.cpp:131`). That is `{ -1 }`, a single test
//! with no layer context, unless one of the two is a via with a padstack.
//! It over tests exactly as KiCad's own TODO at
//! `pcbnew/router/pns_item.cpp:91` describes, and never under tests.
//!
//! # A quirk that survives the port
//!
//! The hole recursions run **before** the layer overlap test
//! (`pcbnew/router/pns_item.cpp:146` to `:168`) and their result is held
//! in a local. Three later exits return `false` outright and throw that
//! local away: the layer overlap test at `:168`, the missing shape test
//! at `:238` and the net tie exclusion at `:255`. So an item whose hole
//! collides with the head can still answer "no collision" because the
//! item's own copper does not share a layer with the head. Reproduced as
//! is, and marked at each of the three sites.

use std::borrow::Cow;
use std::collections::BTreeSet;

use crate::arena::Arena;
use crate::geometry::collision::{self, ShapeCollision};
use crate::geometry::shape::Shape;
use crate::item::{Item, ItemId, Kind, NetId};
use crate::line::Line;
use crate::rules::{ItemRef, Keepout, RuleResolver};

// ---------------------------------------------------------------------
// Options and results
// ---------------------------------------------------------------------

/// How a collision search is to be run.
///
/// Port of `COLLISION_SEARCH_OPTIONS`,
/// `pcbnew/router/pns_node.h:113`, with KiCad's `-1` sentinels turned
/// into `Option` and its kind sentinel into [`Kind::ANY`].
///
/// The item level test reads three of the fields:
/// [`CollisionSearchOptions::different_nets_only`]
/// (`pcbnew/router/pns_item.cpp:179`),
/// [`CollisionSearchOptions::override_clearance`] (`:214`) and
/// [`CollisionSearchOptions::use_clearance_epsilon`] (`:220`). The other
/// three are read by the obstacle visitor one level up
/// (`pcbnew/router/pns_node.cpp:243`, `:249`, `:259`) and are carried
/// here so that the node module does not have to invent a second options
/// struct.
///
/// `m_layer` (`pcbnew/router/pns_node.h:122`) is not ported: it is dead
/// in this revision, declared, defaulted to `-1`, and read nowhere in
/// KiCad's tree.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct CollisionSearchOptions<'a> {
  /// Whether items of the same net are exempt.
  ///
  /// Port of `m_differentNetsOnly`, default true
  /// (`pcbnew/router/pns_node.h:116`). The hole to hole case forces it
  /// off, because a hole to hole rule has nothing to do with nets
  /// (`pcbnew/router/pns_item.cpp:182`).
  pub different_nets_only: bool,
  /// A clearance to use instead of asking the resolver.
  ///
  /// Port of `m_overrideClearance`, whose sentinel is a negative value
  /// (`pcbnew/router/pns_node.h:117`, tested at
  /// `pcbnew/router/pns_item.cpp:214`). It sits **below** the same net,
  /// free pad, keepout and flashing rungs of the ladder, so it overrides
  /// what a rule would have said and not whether a rule applies at all.
  pub override_clearance: Option<i32>,
  /// How many obstacles to collect before giving up.
  ///
  /// Port of `m_limitCount`, sentinel `-1`
  /// (`pcbnew/router/pns_node.h:118`). Read by the visitor
  /// (`pcbnew/router/pns_node.cpp:259`), not here.
  pub limit_count: Option<usize>,
  /// Which kinds of item may be an obstacle.
  ///
  /// Port of `m_kindMask`, sentinel `-1` for "any"
  /// (`pcbnew/router/pns_node.h:119`). Read by the visitor
  /// (`pcbnew/router/pns_node.cpp:243`), not here.
  pub kind_mask: Kind,
  /// Whether the resolver may subtract its epsilon.
  ///
  /// Port of `m_useClearanceEpsilon`, default true
  /// (`pcbnew/router/pns_node.h:120`). The via pushout
  /// (`pcbnew/router/pns_via.cpp:158`) and the walkaround's hull query
  /// (`pcbnew/router/pns_walkaround.cpp:156`) turn it off because they
  /// need the strict rule.
  pub use_clearance_epsilon: bool,
  /// The only items that may be an obstacle, when the search is confined.
  ///
  /// Port of `m_filter`, a `std::function<bool(const ITEM*)>` the visitor
  /// calls (`pcbnew/router/pns_node.h:121`, consumed at
  /// `pcbnew/router/pns_node.cpp:249`), narrowed to its one shape in
  /// KiCad's tree: `WALKAROUND::nearestObstacle` installs a closure that
  /// tests membership of `m_restrictedSet`
  /// (`pcbnew/router/pns_walkaround.cpp:56` to `:64`) and nothing else
  /// ever sets the field. A set is therefore enough, and it keeps the
  /// options struct comparable and copyable.
  ///
  /// [`None`] is KiCad's empty `std::function`, which leaves every
  /// candidate eligible. An empty set is **not** the same thing: it
  /// excludes everything, where KiCad's walkaround guards against that by
  /// only installing the closure when the set is non empty
  /// (`pcbnew/router/pns_walkaround.cpp:56`).
  ///
  /// The set is a [`BTreeSet`] because it is a member of a struct
  /// something may iterate; see `DESIGN.md` section 8.
  pub restricted_set: Option<&'a BTreeSet<ItemId>>,
}

impl Default for CollisionSearchOptions<'_> {
  /// KiCad's member initialisers, `pcbnew/router/pns_node.h:115` to
  /// `:122`.
  fn default() -> Self {
    Self {
      different_nets_only: true,
      override_clearance: None,
      limit_count: None,
      kind_mask: Kind::ANY,
      use_clearance_epsilon: true,
      restricted_set: None,
    }
  }
}

/// One reported collision.
///
/// Port of `OBSTACLE`, `pcbnew/router/pns_node.h:88`, restricted to the
/// fields `collideSimple` fills in (`pcbnew/router/pns_item.cpp:260` to
/// `:266`). KiCad's `m_ipFirst`, `m_pos`, `m_distFirst` and
/// `m_maxFanoutWidth` are zeroed there and written later by
/// `NODE::NearestObstacle` (`pcbnew/router/pns_node.cpp:300`), so they
/// arrive with the node module rather than sitting here as four zeroes.
///
/// The two handles name the items that actually collided, which after a
/// hole expansion is not the pair that was asked about: colliding a pad
/// with a track can report the pad's **hole** against the track.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Obstacle {
  /// The thing being routed. Port of `m_head`. `None` when the head is
  /// not stored in the arena.
  pub head: Option<ItemId>,
  /// The obstacle it hit. Port of `m_item`. `None` when the obstacle is
  /// not stored in the arena.
  pub item: Option<ItemId>,
  /// The clearance the rule resolver asked for, before the `- 1` of the
  /// shape test. Port of `m_clearance`.
  pub clearance: i32,
  /// The gap and the position, when the slow path computed them.
  ///
  /// KiCad computes both on the path that has to know **where** the
  /// collision is (`pcbnew/router/pns_item.cpp:249`) and neither on the
  /// fast path (`:280`), and then keeps neither. They are kept here
  /// because the node's edge exclusion test needs the position; see the
  /// module documentation.
  pub detail: Option<ShapeCollision>,
}

// ---------------------------------------------------------------------
// The head side
// ---------------------------------------------------------------------

/// A [`Line`] as the item level collision test sees it.
///
/// C++ reaches the two `LINE` branches of `ITEM::collideSimple`
/// (`pcbnew/router/pns_item.cpp:133`, `:139`, `:159`) through the `ITEM`
/// base class a `LINE` inherits. A [`Line`] is a value here and not an
/// [`Item`], so the three things the test reads off a `LINE` are gathered
/// into this view instead: its shape (`pcbnew/router/pns_line.h:138`,
/// the bare chain), its width (`:141`) and its via (`:203`).
///
/// Everything else the ladder reads, the net, the layers, the flashing,
/// the marks and the rank, comes from `item`, because every
/// [`RuleResolver`] method takes an [`ItemRef`] and a line has no
/// [`Item`] of its own. [`Line::rule_item`] builds the stand in, so the
/// two never disagree:
///
/// ```
/// use pnsrouter::collide::LineHead;
/// use pnsrouter::line::Line;
/// use pnsrouter::node::World;
/// use pnsrouter::rules::ItemRef;
///
/// fn head_of<'a>(
///   world: &'a World,
///   line: &'a Line,
///   probe: &'a pnsrouter::item::Item,
/// ) -> LineHead<'a> {
///   LineHead::new(ItemRef::unstored(probe), line, line.via_item(world))
/// }
/// ```
#[derive(Clone, Debug)]
pub struct LineHead<'a> {
  /// The item every rule query and every identity test sees.
  item: ItemRef<'a>,
  /// `LINE::Shape( aLayer )` (`pcbnew/router/pns_line.h:138`), which
  /// answers the chain whatever the layer.
  shape: Shape,
  /// `LINE::Width` (`pcbnew/router/pns_line.h:141`), half of which is
  /// folded into the clearance at `pcbnew/router/pns_item.cpp:163`.
  width: i32,
  /// `LINE::Via` (`pcbnew/router/pns_line.h:203`), already resolved
  /// through [`Line::via_item`], so a linked via arrives as the stored
  /// reference the identity tests need.
  via: Option<ItemRef<'a>>,
}

impl<'a> LineHead<'a> {
  /// The head view of a line.
  ///
  /// `item` stands for the line in the rule queries and is normally
  /// [`Line::rule_item`]; `via` is [`Line::via_item`]. The shape and the
  /// width are taken from `line` itself, so a caller cannot pair one
  /// line's chain with another's width. The chain is copied once here,
  /// which is what lets the ladder borrow it across every layer of the
  /// search and both hole recursions.
  pub fn new(item: ItemRef<'a>, line: &Line, via: Option<ItemRef<'a>>) -> Self {
    Self {
      item,
      shape: Shape::LineChain(line.shape().clone()),
      width: line.width(),
      via,
    }
  }
}

/// What the collision test is being asked about.
///
/// The head is KiCad's `const ITEM* aHead`, which may be a `LINE`. This
/// enum is that pointer with the `dyn_cast` at
/// `pcbnew/router/pns_item.cpp:139` turned into a tag, so that one ladder
/// serves both forms.
#[derive(Copy, Clone, Debug)]
enum Head<'a> {
  /// An ordinary item, stored or not.
  Item(ItemRef<'a>),
  /// A line.
  Line(&'a LineHead<'a>),
}

impl<'a> Head<'a> {
  /// The item the rule queries and the identity tests see.
  fn item_ref(self) -> ItemRef<'a> {
    match self {
      Self::Item(item) => item,
      Self::Line(line) => line.item,
    }
  }

  /// The geometry the shape test uses, `ITEM::Shape( aLayer )`
  /// (`pcbnew/router/pns_item.cpp:235`).
  fn shape(self, layer: i32) -> Option<Cow<'a, Shape>> {
    match self {
      Self::Item(item) => item.item().shape(layer),
      Self::Line(line) => Some(Cow::Borrowed(&line.shape)),
    }
  }

  /// `lineWidthH`, the half width folded into the clearance
  /// (`pcbnew/router/pns_item.cpp:163`). Zero for anything but a line,
  /// because every other body carries its width in its shape.
  fn half_width(self) -> i32 {
    match self {
      Self::Item(_) => 0,
      Self::Line(line) => line.width / 2,
    }
  }

  /// The via at a line's last point (`pcbnew/router/pns_item.cpp:140`).
  fn via(self) -> Option<ItemRef<'a>> {
    match self {
      Self::Item(_) => None,
      Self::Line(line) => line.via,
    }
  }
}

// ---------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------

/// Whether two items collide, and how.
///
/// Port of `ITEM::Collide`, `pcbnew/router/pns_item.cpp:305`, which is a
/// wrapper over `collideSimple`, plus the layer loop that KiCad's caller
/// performs through the spatial index; see the module documentation for
/// why the loop is over [`Item::relevant_shape_layers`] here.
///
/// `item` is the candidate obstacle and `head` is the thing being routed.
/// Either may be an unstored [`ItemRef`], which is how the line placer
/// tests a head segment that lives on the stack; KiCad does the same with
/// a local `SEGMENT`.
///
/// The arena is needed to follow the two handles an item can carry:
/// [`Item::hole`], so a pad or via can be expanded into its hole, and
/// [`Item::parent_pad_via`], so a hole can answer for its parent's net
/// and free pad flag the way `HOLE::Net` does
/// (`pcbnew/router/pns_hole.h:56`).
pub fn collide_items(
  arena: &Arena<Item>,
  item: ItemRef<'_>,
  head: ItemRef<'_>,
  resolver: &dyn RuleResolver,
  options: &CollisionSearchOptions,
) -> Option<Obstacle> {
  collide_over_layers(arena, item, Head::Item(head), resolver, options)
}

/// Whether an item and a line collide, and how.
///
/// [`collide_items`] with a [`Line`] on the head side; see [`LineHead`]
/// and the module documentation for what that adds. The obstacle side
/// stays an [`Item`], because a line is never stored and therefore never
/// a candidate.
pub fn collide_line_items(
  arena: &Arena<Item>,
  item: ItemRef<'_>,
  head: &LineHead<'_>,
  resolver: &dyn RuleResolver,
  options: &CollisionSearchOptions,
) -> Option<Obstacle> {
  collide_over_layers(arena, item, Head::Line(head), resolver, options)
}

/// The layer loop both early returning entry points share.
///
/// `ITEM::Collide` (`pcbnew/router/pns_item.cpp:305`) plus the loop
/// KiCad's caller performs through the spatial index; see the module
/// documentation for why the loop is over [`Item::relevant_shape_layers`]
/// here.
fn collide_over_layers(
  arena: &Arena<Item>,
  item: ItemRef<'_>,
  head: Head<'_>,
  resolver: &dyn RuleResolver,
  options: &CollisionSearchOptions,
) -> Option<Obstacle> {
  for layer in item.item().relevant_shape_layers(head.item_ref().item()) {
    let found =
      collide_simple(arena, item, head, layer, resolver, options, None);

    if found.is_some() {
      return found;
    }
  }

  None
}

/// Every collision between two items on one layer, appended to `found`.
///
/// Port of `ITEM::Collide` (`pcbnew/router/pns_item.cpp:305`) in its
/// **collision search context** form, which is the one
/// `NODE::QueryColliding` needs (`pcbnew/router/pns_node.cpp:256`): with a
/// `COLLISION_SEARCH_CONTEXT` the test never returns early, it inserts
/// every obstacle it finds into the context's set and answers only whether
/// it found any (`pcbnew/router/pns_item.cpp:257`, `:282`). See the module
/// documentation, "the accumulating form".
///
/// Three differences from [`collide_items`], all of them KiCad's:
///
/// - one candidate can contribute several obstacles, because a pad and its
///   hole are reported as separate pairs;
/// - both hole recursions run, where the early returning form stops at the
///   first hit (`pcbnew/router/pns_item.cpp:146`, `:154`, whose `|=`
///   evaluates both);
/// - an obstacle the hole recursion appended **stays** appended even when
///   a later test makes this call answer `false`, because the three late
///   exits throw away the local flag and not the set. That is the quirk
///   the module documentation ends on, and it is observable here where it
///   is not in the early returning form.
///
/// `layer` is the layer context KiCad's spatial index stamps onto the
/// visitor (`pcbnew/router/pns_index.h:172`), so the caller passes the
/// layer of the sub index the candidate came out of rather than looping
/// over [`Item::relevant_shape_layers`] itself.
///
/// The return value is KiCad's `collisionsFound`, which the obstacle
/// visitor uses to decide whether to test its limit
/// (`pcbnew/router/pns_node.cpp:255`).
pub fn collide_into(
  arena: &Arena<Item>,
  item: ItemRef<'_>,
  head: ItemRef<'_>,
  layer: i32,
  resolver: &dyn RuleResolver,
  options: &CollisionSearchOptions,
  found: &mut Vec<Obstacle>,
) -> bool {
  collide_simple(
    arena,
    item,
    Head::Item(head),
    layer,
    resolver,
    options,
    Some(found),
  )
  .is_some()
}

/// Every collision between an item and a line on one layer, appended to
/// `found`.
///
/// [`collide_into`] with a [`Line`] on the head side; see [`LineHead`]
/// and the module documentation for what that adds.
pub fn collide_line_into(
  arena: &Arena<Item>,
  item: ItemRef<'_>,
  head: &LineHead<'_>,
  layer: i32,
  resolver: &dyn RuleResolver,
  options: &CollisionSearchOptions,
  found: &mut Vec<Obstacle>,
) -> bool {
  collide_simple(
    arena,
    item,
    Head::Line(head),
    layer,
    resolver,
    options,
    Some(found),
  )
  .is_some()
}

// ---------------------------------------------------------------------
// collideSimple
// ---------------------------------------------------------------------

/// One layer's worth of the item level collision test.
///
/// Port of `ITEM::collideSimple`, `pcbnew/router/pns_item.cpp:104`.
/// `layer` is KiCad's `aLayer`, the layer context, where `-1` means "this
/// item has one shape and no layer decides it".
///
/// `sink` is KiCad's `COLLISION_SEARCH_CONTEXT*`. `None` is the early
/// returning form, which stops at the first hit; `Some` is the
/// accumulating form behind [`collide_into`], which appends every obstacle
/// it finds and runs both hole recursions. The three late exits return
/// `None` in either form, which throws away the **answer** and never what
/// was already appended, exactly as KiCad's `return false` throws away
/// `collisionsFound` and never the set.
///
/// `head` is KiCad's `aHead` with its `LINE` case made explicit; the
/// module documentation says what a line adds. The obstacle side stays an
/// [`ItemRef`], because a line is never stored.
fn collide_simple(
  arena: &Arena<Item>,
  item: ItemRef<'_>,
  head: Head<'_>,
  layer: i32,
  resolver: &dyn RuleResolver,
  options: &CollisionSearchOptions,
  mut sink: Option<&mut Vec<Obstacle>>,
) -> Option<Obstacle> {
  let head_ref = head.item_ref();

  // :119. Nothing collides with itself.
  if item.is_same_as(head_ref) {
    return None;
  }

  // :122
  if !should_consider_hole_collisions(item, head_ref) {
    return None;
  }

  // :127. Physical rules are net blind, so their existence forces the
  // ladder past its same net and free pad short circuits.
  let run_physical_only = resolver.has_user_defined_physical_constraint();

  // :133, the obstacle side of the `dyn_cast`, cannot fire: a line is
  // never stored and therefore never a candidate.

  // :139. A head line's via collides in its own right. KiCad writes it
  // as `line->Via().collideSimple( this, ... )`, so the via takes the
  // obstacle position and this item takes the head position; the
  // obstacle that comes back names them in that order.
  let mut found = match head.via() {
    Some(via) => collide_simple(
      arena,
      via,
      Head::Item(item),
      layer,
      resolver,
      options,
      sink.as_deref_mut(),
    ),
    None => None,
  };

  // :146, :154
  let holes = collide_holes(
    arena,
    item,
    head,
    layer,
    resolver,
    options,
    sink.as_deref_mut(),
  );

  found = found.or(holes);

  // :161. `lineWidthI` is always zero: the obstacle side is never a
  // line.

  // :168. This throws away a hole or via collision just found.
  if !item.item().layers_overlap(head_ref.item()) {
    return None;
  }

  let Some(clearance) = resolve_clearance(
    arena,
    item,
    head_ref,
    resolver,
    options,
    run_physical_only,
  ) else {
    // A negative clearance skips the whole block at :224 and falls
    // through to :301.
    return found;
  };

  // :229. Only the non plated slot half of the trigger is here; the
  // board edge half and the exclusion query itself belong to the node.
  let check_castellation = resolver.is_non_plated_slot(item);
  // :232
  let check_net_tie = resolver.is_in_net_tie(item);

  // :234, :235
  let (Some(shape_item), Some(shape_head)) =
    (item.item().shape(layer), head.shape(layer))
  else {
    // :238, which throws away a hole collision as well.
    return None;
  };

  // :246 to :249. Half the head line's width, and the extra nanometre;
  // see the module documentation for both.
  let distance = clearance + head.half_width() - 1;

  if check_castellation || check_net_tie {
    // The slow path, which needs the position.
    let Some(detail) = collision::collide(&shape_head, &shape_item, distance)
    else {
      return found;
    };

    // :251, the edge exclusion query, is deferred to the node.

    // :254
    if check_net_tie
      && resolver.is_net_tie_exclusion(head_ref, detail.location, item)
    {
      return None;
    }

    // :270
    return Some(report(
      Obstacle {
        head: head_ref.id(),
        item: item.id(),
        clearance,
        detail: Some(detail),
      },
      sink,
    ));
  }

  // :280, the fast path, which asks for a boolean only.
  if collision::collides(&shape_head, &shape_item, distance) {
    // :295
    return Some(report(
      Obstacle {
        head: head_ref.id(),
        item: item.id(),
        clearance,
        detail: None,
      },
      sink,
    ));
  }

  // :301
  found
}

/// Hand one obstacle to the collision search context, if there is one.
///
/// Port of the two `aCtx->obstacles.insert( obs )` sites,
/// `pcbnew/router/pns_item.cpp:265` and `:290`. Without a context KiCad
/// returns the hit instead of recording it, which is what returning the
/// obstacle unchanged stands for.
fn report(obstacle: Obstacle, sink: Option<&mut Vec<Obstacle>>) -> Obstacle {
  if let Some(sink) = sink {
    sink.push(obstacle);
  }

  obstacle
}

/// The two hole recursions of `collideSimple`.
///
/// Port of `pcbnew/router/pns_item.cpp:146` to `:157`.
///
/// KiCad's comment at `:107` claims only the head side needs expanding,
/// because the obstacle is in the node and its hole is therefore in the
/// index as an item of its own, while the head may be under construction
/// and indexed nowhere. The code at `:154` expands the obstacle side all
/// the same. This port follows the code, not the comment: it is also what
/// makes [`collide_items`] correct when a test or a host calls it
/// directly on two stored items with no index in between, and the
/// identity pruning in [`should_consider_hole_collisions`] keeps the
/// extra pass from reporting anything KiCad would not.
///
/// KiCad's `|=` evaluates both recursions whatever the first one
/// answered. Without a sink only the first hit is kept, which is
/// unobservable there: the calls are pure and the caller looks at one
/// obstacle. With a sink both recursions run, because each of them can
/// append an obstacle of its own.
fn collide_holes(
  arena: &Arena<Item>,
  item: ItemRef<'_>,
  head: Head<'_>,
  layer: i32,
  resolver: &dyn RuleResolver,
  options: &CollisionSearchOptions,
  mut sink: Option<&mut Vec<Obstacle>>,
) -> Option<Obstacle> {
  // :127, asked again rather than passed down, so that the argument list
  // stays at seven.
  let run_physical_only = resolver.has_user_defined_physical_constraint();
  let accumulating = sink.is_some();
  let head_ref = head.item_ref();
  let mut found = None;

  // :146. The head's hole against this item. A line has no hole, and the
  // stand in item that speaks for it has none either, so this branch
  // never fires for a line head, exactly as `LINE::Hole()` answers null.
  if let Some(hole) = hole_of(arena, head_ref)
    && should_consider_hole_collisions(item, hole)
    // :150. Skip the net test for hole to hole pairs, and for every pair
    // when a net blind physical rule exists.
    && (item.item().kind() == Kind::HOLE
      || net_of(arena, item) != net_of(arena, hole)
      || run_physical_only)
  {
    found = collide_simple(
      arena,
      item,
      Head::Item(hole),
      layer,
      resolver,
      options,
      sink.as_deref_mut(),
    );
  }

  // :154. This item's hole against the head, which stays whatever the
  // head was, a line included.
  if (found.is_none() || accumulating)
    && let Some(hole) = hole_of(arena, item)
    && should_consider_hole_collisions(hole, head_ref)
  {
    let second =
      collide_simple(arena, hole, head, layer, resolver, options, sink);

    found = found.or(second);
  }

  found
}

// ---------------------------------------------------------------------
// The clearance ladder
// ---------------------------------------------------------------------

/// The clearance between two items, or `None` when they can never
/// collide.
///
/// Port of the `else if` chain at `pcbnew/router/pns_item.cpp:178` to
/// `:222`. First match wins, and the order decides the answer: the free
/// pad rung sits above the keepout rung, and both sit above the clearance
/// override, so an override cannot resurrect a pair a rule exempted.
fn resolve_clearance(
  arena: &Arena<Item>,
  item: ItemRef<'_>,
  head: ItemRef<'_>,
  resolver: &dyn RuleResolver,
  options: &CollisionSearchOptions,
  run_physical_only: bool,
) -> Option<i32> {
  // :178
  let mut different_nets_only = options.different_nets_only;

  // :182. A hole to hole rule has nothing to do with nets.
  if item.item().kind() == Kind::HOLE && head.item().kind() == Kind::HOLE {
    different_nets_only = false;
  }

  let head_net = net_of(arena, head);

  // :188. Same net, so no clearance at all.
  if different_nets_only
    && head_net.is_some()
    && net_of(arena, item) == head_net
    && !run_physical_only
  {
    return None;
  }

  // :193. A pad on a not internally connected pin has no net until it is
  // used.
  if different_nets_only
    && (is_free_pad(arena, item) || is_free_pad(arena, head))
    && !run_physical_only
  {
    return None;
  }

  // :198
  match keepout_between(resolver, item, head) {
    // :202. Keepouts are an exact boundary, with no clearance.
    Keepout::Enforced => return Some(0),
    // :204. A keepout whose rules do not exclude this item is not an
    // obstacle at all.
    Keepout::Present => return None,
    Keepout::None => {}
  }

  // :206
  if !item.item().is_flashed_on_any(head.item().layers()) {
    return None;
  }

  // :210
  if !head.item().is_flashed_on_any(item.item().layers()) {
    return None;
  }

  // :214
  if let Some(override_clearance) = options.override_clearance {
    return Some(override_clearance);
  }

  // :220
  clearance_from_resolver(resolver, item, head, options.use_clearance_epsilon)
}

/// The keepout verdict for a pair, in either direction.
///
/// Port of the short circuiting `||` at
/// `pcbnew/router/pns_item.cpp:198`. The second question is asked only
/// when the first answers [`Keepout::None`], which is exactly what the
/// `||` does, and is only correct because a keepout item is never also a
/// routed item; note 05 section 2.3 makes that point.
fn keepout_between(
  resolver: &dyn RuleResolver,
  item: ItemRef<'_>,
  head: ItemRef<'_>,
) -> Keepout {
  let obstacle_side = resolver.is_keepout(item, head);

  if obstacle_side != Keepout::None {
    return obstacle_side;
  }

  resolver.is_keepout(head, item)
}

/// The resolver's clearance, with the node's two special cases in front.
///
/// Port of `NODE::GetClearance`, `pcbnew/router/pns_node.cpp:143`. A
/// virtual item is given a clearance of zero rather than being exempted
/// (`:148`), so it still collides where it actually overlaps. KiCad's
/// other special case, `100000` when the node has no rule resolver at all
/// (`:145`), cannot happen here because the resolver is not optional.
///
/// The related rule that a virtual item is never the **query** item
/// (`pcbnew/router/pns_node.cpp:273`) sits in `NODE::QueryColliding` and
/// belongs to the node module.
fn clearance_from_resolver(
  resolver: &dyn RuleResolver,
  item: ItemRef<'_>,
  head: ItemRef<'_>,
  use_epsilon: bool,
) -> Option<i32> {
  if item.item().is_virtual() || head.item().is_virtual() {
    return Some(0);
  }

  resolver.clearance(item, Some(head), use_epsilon)
}

// ---------------------------------------------------------------------
// Hole pruning
// ---------------------------------------------------------------------

/// Whether a pair involving a hole is worth testing at all.
///
/// Port of `shouldWeConsiderHoleCollisions`,
/// `pcbnew/router/pns_item.cpp:38`, which prunes the self collisions a
/// hole would otherwise have with the pad or via that drilled it.
///
/// Three cases, in KiCad's order:
///
/// - two holes: the parent identity test at `:71`. The geometric
///   heuristic that sits above it at `:65` is **not** ported; see the
///   module documentation for the fixture that retired it;
/// - one hole: it does not collide with its own parent (`:75`, `:77`);
/// - no hole: always worth testing (`:79`).
fn should_consider_hole_collisions(
  item: ItemRef<'_>,
  head: ItemRef<'_>,
) -> bool {
  let item_is_hole = item.item().kind() == Kind::HOLE;
  let head_is_hole = head.item().kind() == Kind::HOLE;

  if item_is_hole && head_is_hole {
    return consider_hole_to_hole(item, head);
  }

  if item_is_hole {
    // :75
    return !hole_belongs_to(item, head);
  }

  if head_is_hole {
    // :77
    return !hole_belongs_to(head, item);
  }

  // :79
  true
}

/// Whether two holes are worth testing against each other.
///
/// Port of `pcbnew/router/pns_item.cpp:43` to `:72`, minus the geometric
/// heuristic at `:65` to `:69`.
///
/// # The retired heuristic
///
/// KiCad asks, above the identity test, whether the two parents are vias
/// that cannot be told apart: same position, matching padstacks, same net,
/// same drill (`VIA::PadstackMatches`, `pcbnew/router/pns_via.cpp:108`).
/// It exists for one input, a `LINE` holding a clone of a via that is also
/// in the node, and this port cannot produce that input; the module
/// documentation has the argument and `src/node.rs` has the fixture. What
/// is left of the answer it changes is two distinct stored vias sharing a
/// position, which now collide hole to hole.
fn consider_hole_to_hole(item: ItemRef<'_>, head: ItemRef<'_>) -> bool {
  // :48. A hole with no parent is a mechanical hole, and two of those
  // are always worth testing.
  let (Some(parent_item), Some(parent_head)) =
    (item.item().parent_pad_via(), head.item().parent_pad_via())
  else {
    return true;
  };

  // :71
  parent_item != parent_head
}

/// Whether a hole was drilled by a given item.
///
/// Port of the `holeI->ParentPadVia() != aHead` comparisons at
/// `pcbnew/router/pns_item.cpp:75` and `:77`. An unstored item has no
/// handle and can therefore never be a hole's parent, which is the same
/// answer the pointer comparison gives for a hole whose parent is null.
fn hole_belongs_to(hole: ItemRef<'_>, other: ItemRef<'_>) -> bool {
  match (hole.item().parent_pad_via(), other.id()) {
    (Some(parent), Some(id)) => parent == id,
    _ => false,
  }
}

// ---------------------------------------------------------------------
// Following the two item handles
// ---------------------------------------------------------------------

/// The hole a pad or via owns, as a reference.
///
/// Port of `ITEM::Hole` (`pcbnew/router/pns_item.h:305`) followed by the
/// dereference. A handle the arena no longer knows answers `None`, where
/// KiCad would follow a dangling pointer; that is the whole point of the
/// generational id (note 02 section 10.1).
fn hole_of<'a>(
  arena: &'a Arena<Item>,
  item: ItemRef<'_>,
) -> Option<ItemRef<'a>> {
  let id = item.item().hole()?;

  Some(ItemRef::stored(id, arena.get(id)?))
}

/// The pad or via a hole belongs to.
///
/// Port of `ITEM::ParentPadVia` (`pcbnew/router/pns_item.h:293`) followed
/// by the dereference.
fn parent_of<'a>(
  arena: &'a Arena<Item>,
  item: ItemRef<'_>,
) -> Option<&'a Item> {
  arena.get(item.item().parent_pad_via()?)
}

/// An item's net, with a hole answering for its parent.
///
/// Port of `HOLE::Net`, `pcbnew/router/pns_hole.h:56`, which is the one
/// override of `ITEM::Net`. `src/item.rs` cannot implement it, because
/// following the parent handle needs the arena; its documentation says
/// the collision ladder has to do it instead.
fn net_of(arena: &Arena<Item>, item: ItemRef<'_>) -> Option<NetId> {
  if item.item().kind() == Kind::HOLE
    && let Some(parent) = parent_of(arena, item)
  {
    return parent.net();
  }

  item.item().net()
}

/// Whether an item counts as a free pad, its parent included.
///
/// Port of `ITEM::IsFreePad`, `pcbnew/router/pns_item.h:288`, whose
/// second half asks the parent pad or via. `Item::is_free_pad` implements
/// only the first half and says that the caller has to fold in the
/// parent's flag, which is what this does.
fn is_free_pad(arena: &Arena<Item>, item: ItemRef<'_>) -> bool {
  if item.item().is_free_pad() {
    return true;
  }

  parent_of(arena, item).is_some_and(|parent| parent.is_free_pad())
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::seg::Seg;
  use crate::geometry::shape::Shape;
  use crate::geometry::vec2::Vec2;
  use crate::item::{
    Hole, ItemBody, LayerMask, LayerRange, Segment, Solid, Via, ViaType,
  };
  use crate::rules::{Constraint, ConstraintType, FixedClearance};

  /// The clearance every test uses unless it says otherwise.
  const CLEARANCE: i32 = 2000;

  /// A resolver that adds the two switches the ladder needs and that
  /// [`FixedClearance`] deliberately does not have.
  struct TestRules {
    /// The clearance table.
    fixed: FixedClearance,
    /// Which item, by uid, is a keepout.
    keepout_uid: Option<u64>,
    /// What that keepout says.
    keepout: Keepout,
    /// Whether every item is in a net tie, which forces the slow path.
    net_tie: bool,
    /// Whether a net blind physical rule exists.
    physical: bool,
  }

  impl TestRules {
    /// A resolver with one clearance and no switches on.
    fn uniform(clearance: i32) -> Self {
      Self {
        fixed: FixedClearance::uniform(clearance),
        keepout_uid: None,
        keepout: Keepout::None,
        net_tie: false,
        physical: false,
      }
    }
  }

  impl RuleResolver for TestRules {
    fn clearance(
      &self,
      a: ItemRef<'_>,
      b: Option<ItemRef<'_>>,
      use_epsilon: bool,
    ) -> Option<i32> {
      self.fixed.clearance(a, b, use_epsilon)
    }

    fn clearance_epsilon(&self) -> i32 {
      self.fixed.clearance_epsilon()
    }

    fn has_user_defined_physical_constraint(&self) -> bool {
      self.physical
    }

    fn constraint(
      &self,
      constraint_type: ConstraintType,
      a: ItemRef<'_>,
      b: Option<ItemRef<'_>>,
      layer: i32,
    ) -> Option<Constraint> {
      self.fixed.constraint(constraint_type, a, b, layer)
    }

    fn is_keepout(&self, obstacle: ItemRef<'_>, _item: ItemRef<'_>) -> Keepout {
      if self.keepout_uid == Some(obstacle.item().uid()) {
        return self.keepout;
      }

      Keepout::None
    }

    fn is_drilled_hole(&self, item: ItemRef<'_>) -> bool {
      self.fixed.is_drilled_hole(item)
    }

    fn is_non_plated_slot(&self, item: ItemRef<'_>) -> bool {
      self.fixed.is_non_plated_slot(item)
    }

    fn is_in_net_tie(&self, _item: ItemRef<'_>) -> bool {
      self.net_tie
    }

    fn net_code(&self, net: NetId) -> i32 {
      self.fixed.net_code(net)
    }

    fn orphaned_net(&self) -> NetId {
      self.fixed.orphaned_net()
    }
  }

  /// A round pad of a radius, on layer 0, flashed there.
  fn pad(uid: u64, x: i32, radius: i32, net: u32) -> Item {
    let center = Vec2::new(x, 0);
    let shape = Shape::Circle { center, radius };
    let mut item = Item::new(uid, ItemBody::Solid(Solid::new(shape, center)));
    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(Some(NetId(net)));

    item
  }

  /// A horizontal track of a width, on layer 0, flashed there.
  fn segment(uid: u64, x: i32, width: i32, net: u32) -> Item {
    let body =
      ItemBody::Segment(Segment::new(Seg::from_coords(x, 0, x, 10000), width));
    let mut item = Item::new(uid, body);
    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(Some(NetId(net)));

    item
  }

  /// A via spanning layers 0 to 1, flashed on both.
  fn via(uid: u64, x: i32, diameter: i32, drill: i32, net: u32) -> Item {
    let body = ItemBody::Via(Via::new(
      Vec2::new(x, 0),
      diameter,
      drill,
      ViaType::Through,
    ));
    let mut item = Item::new(uid, body);
    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item.set_net(Some(NetId(net)));

    item
  }

  /// A round hole, on layer 0, flashed there.
  fn hole(uid: u64, x: i32, radius: i32) -> Item {
    let body = ItemBody::Hole(Hole::circular(Vec2::new(x, 0), radius));
    let mut item = Item::new(uid, body);
    item.set_layers_and_flash_all(LayerRange::single(0));

    item
  }

  /// Store an item and hand back its handle.
  fn store(arena: &mut Arena<Item>, item: Item) -> ItemId {
    arena.insert(item)
  }

  /// Store a pad or via together with a hole, linked both ways.
  fn store_with_hole(
    arena: &mut Arena<Item>,
    parent: Item,
    mut drilled: Item,
  ) -> (ItemId, ItemId) {
    let parent_id = arena.insert(parent);
    drilled.set_parent_pad_via(Some(parent_id));
    let hole_id = arena.insert(drilled);
    arena
      .get_mut(parent_id)
      .expect("the parent was just inserted")
      .set_hole(Some(hole_id));

    (parent_id, hole_id)
  }

  /// A reference to a stored item.
  fn stored(arena: &Arena<Item>, id: ItemId) -> ItemRef<'_> {
    ItemRef::stored(id, arena.get(id).expect("the item must be stored"))
  }

  /// Collide two stored items with the default options.
  fn collide_stored(
    arena: &Arena<Item>,
    rules: &dyn RuleResolver,
    item: ItemId,
    head: ItemId,
  ) -> Option<Obstacle> {
    collide_items(
      arena,
      stored(arena, item),
      stored(arena, head),
      rules,
      &CollisionSearchOptions::default(),
    )
  }

  // -----------------------------------------------------------------
  // Identity, layers and flashing
  // -----------------------------------------------------------------

  #[test]
  fn an_item_never_collides_with_itself() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let id = store(&mut arena, pad(0, 0, 1000, 1));

    assert_eq!(collide_stored(&arena, &rules, id, id), None);
  }

  #[test]
  fn an_unstored_item_never_collides_with_itself() {
    let arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let item = pad(0, 0, 1000, 1);
    let reference = ItemRef::unstored(&item);

    assert_eq!(
      collide_items(
        &arena,
        reference,
        reference,
        &rules,
        &CollisionSearchOptions::default()
      ),
      None
    );
  }

  #[test]
  fn a_virtual_item_is_given_no_clearance_at_all() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let mut obstacle = pad(0, 0, 1000, 1);
    let head = store(&mut arena, pad(1, 3000, 1000, 2));

    // 1000 nm of copper to copper gap, well inside the 2000 nm rule.
    let colliding = store(&mut arena, obstacle.clone());
    assert!(collide_stored(&arena, &rules, colliding, head).is_some());

    obstacle.set_is_virtual(true);
    let virtual_item = store(&mut arena, obstacle);
    assert_eq!(collide_stored(&arena, &rules, virtual_item, head), None);
  }

  #[test]
  fn items_on_different_layers_never_collide() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let first = store(&mut arena, pad(0, 0, 1000, 1));
    let mut other = pad(1, 0, 1000, 2);
    other.set_layers_and_flash_all(LayerRange::single(1));
    let second = store(&mut arena, other);

    assert_eq!(collide_stored(&arena, &rules, first, second), None);
  }

  #[test]
  fn an_item_not_flashed_on_the_shared_layers_never_collides() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let mut obstacle = pad(0, 0, 1000, 1);
    obstacle.set_flashed_layers(LayerMask::NONE);
    let first = store(&mut arena, obstacle);
    let second = store(&mut arena, pad(1, 0, 1000, 2));

    assert_eq!(collide_stored(&arena, &rules, first, second), None);

    // The same pair collides once the copper is flashed again.
    arena
      .get_mut(first)
      .expect("stored")
      .set_flashed_layers(LayerMask::NONE.with(0));
    assert!(collide_stored(&arena, &rules, first, second).is_some());
  }

  #[test]
  fn the_flashing_test_runs_in_both_directions() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let first = store(&mut arena, pad(0, 0, 1000, 1));
    let mut head = pad(1, 0, 1000, 2);
    head.set_flashed_layers(LayerMask::NONE);
    let second = store(&mut arena, head);

    assert_eq!(collide_stored(&arena, &rules, first, second), None);
  }

  // -----------------------------------------------------------------
  // Nets, free pads and keepouts
  // -----------------------------------------------------------------

  #[test]
  fn a_free_pad_is_skipped_above_the_clearance_override() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let mut free = pad(0, 0, 1000, 1);
    free.set_is_free_pad(true);
    let first = store(&mut arena, free);
    let second = store(&mut arena, pad(1, 0, 1000, 2));

    let options = CollisionSearchOptions {
      override_clearance: Some(CLEARANCE),
      ..CollisionSearchOptions::default()
    };

    // The two pads are concentric, so only the free pad rung of the
    // ladder, which sits above the override, can suppress this.
    assert_eq!(
      collide_items(
        &arena,
        stored(&arena, first),
        stored(&arena, second),
        &rules,
        &options
      ),
      None
    );
  }

  #[test]
  fn a_free_pad_parent_makes_its_hole_a_free_pad_too() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let mut free = pad(0, 0, 1000, 1);
    free.set_is_free_pad(true);
    let (_, hole_id) = store_with_hole(&mut arena, free, hole(1, 0, 300));
    let head = store(&mut arena, pad(2, 0, 1000, 2));

    let options = CollisionSearchOptions {
      override_clearance: Some(CLEARANCE),
      ..CollisionSearchOptions::default()
    };

    assert_eq!(
      collide_items(
        &arena,
        stored(&arena, hole_id),
        stored(&arena, head),
        &rules,
        &options
      ),
      None
    );
  }

  #[test]
  fn the_same_net_is_skipped_only_while_different_nets_only_is_set() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let first = store(&mut arena, pad(0, 0, 1000, 1));
    let second = store(&mut arena, pad(1, 0, 1000, 1));

    assert_eq!(collide_stored(&arena, &rules, first, second), None);

    // With the flag off the ladder reaches the override, which the same
    // net rung would have short circuited.
    let options = CollisionSearchOptions {
      different_nets_only: false,
      override_clearance: Some(CLEARANCE),
      ..CollisionSearchOptions::default()
    };

    assert!(
      collide_items(
        &arena,
        stored(&arena, first),
        stored(&arena, second),
        &rules,
        &options
      )
      .is_some()
    );
  }

  #[test]
  fn items_without_a_net_are_not_on_the_same_net() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let mut first = pad(0, 0, 1000, 1);
    first.set_net(None);
    let mut second = pad(1, 0, 1000, 1);
    second.set_net(None);
    let first = store(&mut arena, first);
    let second = store(&mut arena, second);

    assert!(collide_stored(&arena, &rules, first, second).is_some());
  }

  #[test]
  fn a_physical_rule_defeats_the_same_net_short_circuit() {
    let mut arena = Arena::new();
    let mut rules = TestRules::uniform(CLEARANCE);
    rules.physical = true;
    let first = store(&mut arena, pad(0, 0, 1000, 1));
    let second = store(&mut arena, pad(1, 0, 1000, 1));

    let options = CollisionSearchOptions {
      override_clearance: Some(CLEARANCE),
      ..CollisionSearchOptions::default()
    };

    assert!(
      collide_items(
        &arena,
        stored(&arena, first),
        stored(&arena, second),
        &rules,
        &options
      )
      .is_some()
    );
  }

  #[test]
  fn an_enforced_keepout_collides_at_its_exact_boundary() {
    let mut arena = Arena::new();
    let mut rules = TestRules::uniform(CLEARANCE);
    rules.keepout_uid = Some(0);
    rules.keepout = Keepout::Enforced;
    let keepout = store(&mut arena, pad(0, 0, 1000, 1));
    let overlapping = store(&mut arena, pad(1, 500, 1000, 2));
    let apart = store(&mut arena, pad(2, 3000, 1000, 2));

    let found = collide_stored(&arena, &rules, keepout, overlapping)
      .expect("overlapping copper must collide with an enforced keepout");
    assert_eq!(found.clearance, 0);

    // 1000 nm apart, which would violate the 2000 nm copper rule but is
    // outside a keepout's exact boundary.
    assert_eq!(collide_stored(&arena, &rules, keepout, apart), None);
  }

  #[test]
  fn a_keepout_that_does_not_exclude_the_item_is_not_an_obstacle() {
    let mut arena = Arena::new();
    let mut rules = TestRules::uniform(CLEARANCE);
    rules.keepout_uid = Some(0);
    rules.keepout = Keepout::Present;
    let keepout = store(&mut arena, pad(0, 0, 1000, 1));
    let overlapping = store(&mut arena, pad(1, 500, 1000, 2));

    assert_eq!(collide_stored(&arena, &rules, keepout, overlapping), None);
  }

  #[test]
  fn the_keepout_question_is_asked_in_both_directions() {
    let mut arena = Arena::new();
    let mut rules = TestRules::uniform(CLEARANCE);
    // The head, not the obstacle, is the keepout this time.
    rules.keepout_uid = Some(1);
    rules.keepout = Keepout::Present;
    let first = store(&mut arena, pad(0, 0, 1000, 1));
    let second = store(&mut arena, pad(1, 500, 1000, 2));

    assert_eq!(collide_stored(&arena, &rules, first, second), None);
  }

  // -----------------------------------------------------------------
  // The clearance itself
  // -----------------------------------------------------------------

  #[test]
  fn the_extra_nanometre_makes_the_clearance_distance_safe() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let obstacle = store(&mut arena, pad(0, 0, 1000, 1));

    // Two circles of radius 1000: the edge to edge gap is x - 2000.
    let exactly = store(&mut arena, pad(1, 4000, 1000, 2));
    let one_closer = store(&mut arena, pad(2, 3999, 1000, 2));
    let two_closer = store(&mut arena, pad(3, 3998, 1000, 2));

    // A gap of exactly the clearance is safe at the shape level already,
    // because that comparison is strict. The extra nanometre buys the
    // one after it, which is the whole point of pns_item.cpp:249.
    assert_eq!(collide_stored(&arena, &rules, obstacle, exactly), None);
    assert_eq!(collide_stored(&arena, &rules, obstacle, one_closer), None);
    assert!(collide_stored(&arena, &rules, obstacle, two_closer).is_some());

    let circle = |x: i32| Shape::Circle {
      center: Vec2::new(x, 0),
      radius: 1000,
    };

    assert!(!collision::collides(&circle(0), &circle(4000), CLEARANCE));
    assert!(collision::collides(&circle(0), &circle(3999), CLEARANCE));
  }

  #[test]
  fn the_epsilon_is_asked_for_only_when_the_options_say_so() {
    let mut arena = Arena::new();
    let rules = TestRules {
      fixed: FixedClearance {
        clearance: CLEARANCE,
        hole_clearance: None,
        hole_to_hole_clearance: None,
        clearance_epsilon: 500,
      },
      ..TestRules::uniform(CLEARANCE)
    };
    let obstacle = store(&mut arena, pad(0, 0, 1000, 1));
    // A gap of 1600 nm: inside the 2000 nm rule, outside 2000 - 500.
    let head = store(&mut arena, pad(1, 3600, 1000, 2));

    let with_epsilon = CollisionSearchOptions::default();
    let without = CollisionSearchOptions {
      use_clearance_epsilon: false,
      ..CollisionSearchOptions::default()
    };

    assert_eq!(
      collide_items(
        &arena,
        stored(&arena, obstacle),
        stored(&arena, head),
        &rules,
        &with_epsilon
      ),
      None
    );

    let found = collide_items(
      &arena,
      stored(&arena, obstacle),
      stored(&arena, head),
      &rules,
      &without,
    )
    .expect("the strict rule must be violated");
    assert_eq!(found.clearance, CLEARANCE);
  }

  #[test]
  fn the_override_replaces_what_the_resolver_would_have_said() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let obstacle = store(&mut arena, pad(0, 0, 1000, 1));
    let head = store(&mut arena, pad(1, 6000, 1000, 2));

    assert_eq!(collide_stored(&arena, &rules, obstacle, head), None);

    let options = CollisionSearchOptions {
      override_clearance: Some(10_000),
      ..CollisionSearchOptions::default()
    };
    let found = collide_items(
      &arena,
      stored(&arena, obstacle),
      stored(&arena, head),
      &rules,
      &options,
    )
    .expect("a 10 mm override must reach 4 mm away");
    assert_eq!(found.clearance, 10_000);
  }

  // -----------------------------------------------------------------
  // Holes
  // -----------------------------------------------------------------

  #[test]
  fn a_pads_hole_collides_where_the_pads_copper_does_not() {
    let mut arena = Arena::new();
    let rules = TestRules {
      fixed: FixedClearance {
        clearance: 1000,
        hole_clearance: Some(5000),
        hole_to_hole_clearance: None,
        clearance_epsilon: 0,
      },
      ..TestRules::uniform(1000)
    };
    let (pad_id, hole_id) =
      store_with_hole(&mut arena, pad(0, 0, 1000, 1), hole(1, 0, 300));
    let track = store(&mut arena, segment(2, 4000, 200, 2));

    // Copper gap 2900 nm against a 1000 nm rule, hole gap 3600 nm
    // against a 5000 nm rule.
    let found = collide_stored(&arena, &rules, pad_id, track)
      .expect("the hole must reach the track");

    assert_eq!(found.item, Some(hole_id));
    assert_eq!(found.head, Some(track));
    assert_eq!(found.clearance, 5000);
  }

  #[test]
  fn a_head_vias_hole_collides_with_a_stored_pad() {
    let mut arena = Arena::new();
    let rules = TestRules {
      fixed: FixedClearance {
        clearance: 1000,
        hole_clearance: Some(5000),
        hole_to_hole_clearance: None,
        clearance_epsilon: 0,
      },
      ..TestRules::uniform(1000)
    };
    let obstacle = store(&mut arena, pad(0, 0, 1000, 1));
    let (via_id, hole_id) = store_with_hole(
      &mut arena,
      via(1, 4000, 800, 400, 2),
      hole(2, 4000, 200),
    );

    let found = collide_stored(&arena, &rules, obstacle, via_id)
      .expect("the via's hole must reach the pad");

    assert_eq!(found.item, Some(obstacle));
    assert_eq!(found.head, Some(hole_id));
  }

  #[test]
  fn a_hole_does_not_collide_with_the_pad_that_drilled_it() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let (pad_id, hole_id) =
      store_with_hole(&mut arena, pad(0, 0, 1000, 1), hole(1, 0, 300));

    assert_eq!(collide_stored(&arena, &rules, hole_id, pad_id), None);
    assert_eq!(collide_stored(&arena, &rules, pad_id, hole_id), None);
  }

  /// The one answer retiring the geometric heuristic changes.
  ///
  /// Two **distinct stored** vias at one position, with the same
  /// padstack, net and drill. KiCad prunes their hole pair at
  /// `pcbnew/router/pns_item.cpp:65`; here they collide, because the
  /// pruning existed for a `LINE`'s clone of a via and this port cannot
  /// build that input. See the module documentation.
  #[test]
  fn two_stored_vias_at_one_position_collide_hole_to_hole() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let (first, first_hole) =
      store_with_hole(&mut arena, via(0, 0, 800, 400, 1), hole(1, 0, 200));
    let (second, second_hole) =
      store_with_hole(&mut arena, via(2, 0, 800, 400, 1), hole(3, 0, 200));

    // Every copper pair is same net and exempt, so what is left is the
    // hole pair, which a hole to hole rule tests whatever the nets.
    let found = collide_stored(&arena, &rules, first, second)
      .expect("two vias sharing a position share a hole");

    assert_eq!(found.item, Some(first_hole));
    assert_eq!(found.head, Some(second_hole));
  }

  #[test]
  fn two_via_holes_with_different_padstacks_collide() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let (first, _) =
      store_with_hole(&mut arena, via(0, 0, 800, 400, 1), hole(1, 0, 200));
    let (second, second_hole) =
      store_with_hole(&mut arena, via(2, 0, 900, 400, 1), hole(3, 0, 200));

    let found = collide_stored(&arena, &rules, first, second)
      .expect("the two holes must collide");

    assert_eq!(found.head, Some(second_hole));
  }

  #[test]
  fn two_holes_on_one_net_still_collide() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let mut first = hole(0, 0, 300);
    first.set_net(Some(NetId(1)));
    let mut second = hole(1, 400, 300);
    second.set_net(Some(NetId(1)));
    let first = store(&mut arena, first);
    let second = store(&mut arena, second);

    // The hole to hole rung at pns_item.cpp:182 clears
    // different_nets_only, so the same net rung cannot fire.
    assert!(collide_stored(&arena, &rules, first, second).is_some());
  }

  // -----------------------------------------------------------------
  // Unstored heads
  // -----------------------------------------------------------------

  #[test]
  fn an_unstored_head_collides_with_a_stored_item() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let obstacle = store(&mut arena, pad(0, 0, 1000, 1));
    let head = segment(1, 2500, 200, 2);

    let found = collide_items(
      &arena,
      stored(&arena, obstacle),
      ItemRef::unstored(&head),
      &rules,
      &CollisionSearchOptions::default(),
    )
    .expect("a stack allocated head must be able to collide");

    assert_eq!(found.item, Some(obstacle));
    assert_eq!(found.head, None);
    assert_eq!(found.clearance, CLEARANCE);
  }

  #[test]
  fn an_unstored_head_is_never_a_holes_parent() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let (_, hole_id) =
      store_with_hole(&mut arena, pad(0, 0, 1000, 1), hole(1, 0, 300));
    let head = pad(2, 0, 1000, 2);

    // A copy of the parent that is not the parent still collides with
    // the hole.
    assert!(
      collide_items(
        &arena,
        stored(&arena, hole_id),
        ItemRef::unstored(&head),
        &rules,
        &CollisionSearchOptions::default()
      )
      .is_some()
    );
  }

  // -----------------------------------------------------------------
  // The two shape paths
  // -----------------------------------------------------------------

  /// Every pair the fast and slow path comparison runs over, as
  /// `(obstacle, head)` handles into one arena.
  fn agreement_cases(arena: &mut Arena<Item>) -> Vec<(ItemId, ItemId)> {
    let mut cases = Vec::new();

    let obstacle = store(arena, pad(0, 0, 1000, 1));
    for (uid, x) in [(1, 3998), (2, 3999), (3, 4000), (4, 500)] {
      let head = store(arena, pad(uid, x, 1000, 2));
      cases.push((obstacle, head));
    }

    let same_net = store(arena, pad(10, 500, 1000, 1));
    cases.push((obstacle, same_net));

    let other_layer = {
      let mut item = pad(11, 500, 1000, 2);
      item.set_layers_and_flash_all(LayerRange::single(1));
      store(arena, item)
    };
    cases.push((obstacle, other_layer));

    let (drilled, _) =
      store_with_hole(arena, pad(12, 6000, 1000, 3), hole(13, 6000, 300));
    let track = store(arena, segment(14, 8000, 200, 4));
    cases.push((drilled, track));

    let (first_via, _) =
      store_with_hole(arena, via(15, 0, 800, 400, 5), hole(16, 0, 200));
    let (second_via, _) =
      store_with_hole(arena, via(17, 300, 900, 400, 6), hole(18, 300, 200));
    cases.push((first_via, second_via));

    cases
  }

  #[test]
  fn the_fast_path_agrees_with_the_path_that_reports_a_position() {
    let mut arena = Arena::new();
    let cases = agreement_cases(&mut arena);
    let fast = TestRules::uniform(CLEARANCE);
    let slow = TestRules {
      net_tie: true,
      ..TestRules::uniform(CLEARANCE)
    };

    for (item, head) in cases {
      let by_fast = collide_stored(&arena, &fast, item, head);
      let by_slow = collide_stored(&arena, &slow, item, head);

      assert_eq!(
        by_fast.is_some(),
        by_slow.is_some(),
        "the two paths disagree on {item:?} against {head:?}"
      );

      match (by_fast, by_slow) {
        (Some(fast_hit), Some(slow_hit)) => {
          assert_eq!(fast_hit.item, slow_hit.item);
          assert_eq!(fast_hit.head, slow_hit.head);
          assert_eq!(fast_hit.clearance, slow_hit.clearance);
          // Only the slow path computes the gap and the position.
          assert_eq!(fast_hit.detail, None);
          assert!(slow_hit.detail.is_some());
        }
        (None, None) => {}
        _ => unreachable!("the two paths were just compared"),
      }
    }
  }

  #[test]
  fn a_net_tie_exclusion_suppresses_the_collision_it_names() {
    /// A resolver whose net tie forgives every collision.
    struct Forgiving;

    impl RuleResolver for Forgiving {
      fn clearance(
        &self,
        _a: ItemRef<'_>,
        _b: Option<ItemRef<'_>>,
        _use_epsilon: bool,
      ) -> Option<i32> {
        Some(CLEARANCE)
      }

      fn is_keepout(
        &self,
        _obstacle: ItemRef<'_>,
        _item: ItemRef<'_>,
      ) -> Keepout {
        Keepout::None
      }

      fn is_drilled_hole(&self, item: ItemRef<'_>) -> bool {
        item.item().kind() == Kind::HOLE
      }

      fn is_non_plated_slot(&self, _item: ItemRef<'_>) -> bool {
        false
      }

      fn net_code(&self, net: NetId) -> i32 {
        i32::try_from(net.0).unwrap_or(i32::MAX)
      }

      fn orphaned_net(&self) -> NetId {
        NetId(0)
      }

      fn is_in_net_tie(&self, _item: ItemRef<'_>) -> bool {
        true
      }

      fn is_net_tie_exclusion(
        &self,
        _item: ItemRef<'_>,
        _collision_position: Vec2,
        _colliding_item: ItemRef<'_>,
      ) -> bool {
        true
      }
    }

    let mut arena = Arena::new();
    let first = store(&mut arena, pad(0, 0, 1000, 1));
    let second = store(&mut arena, pad(1, 500, 1000, 2));

    assert_eq!(collide_stored(&arena, &Forgiving, first, second), None);
  }

  // -----------------------------------------------------------------
  // The accumulating form
  // -----------------------------------------------------------------

  /// The early returning form answers with the first obstacle it finds,
  /// the accumulating one with the pad and its hole both.
  #[test]
  fn the_accumulating_form_reports_a_via_and_its_hole_separately() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);
    let (via_id, hole_id) =
      store_with_hole(&mut arena, via(0, 0, 2000, 1000, 1), hole(1, 0, 500));
    let head_id = store(&mut arena, segment(2, 0, 100, 2));

    let mut found = Vec::new();
    let any = collide_into(
      &arena,
      stored(&arena, via_id),
      stored(&arena, head_id),
      0,
      &rules,
      &CollisionSearchOptions::default(),
      &mut found,
    );

    assert!(any);
    assert_eq!(found.len(), 2);
    assert_eq!(found[0].item, Some(hole_id));
    assert_eq!(found[1].item, Some(via_id));

    for obstacle in &found {
      assert_eq!(obstacle.head, Some(head_id));
    }

    // The early returning form answers with one obstacle only, and it is
    // the via: the item's own shape test runs last and returns directly,
    // over the hole the recursion had already found
    // (`pcbnew/router/pns_item.cpp:295` against `:301`).
    let first = collide_stored(&arena, &rules, via_id, head_id);
    assert_eq!(first.map(|obstacle| obstacle.item), Some(Some(via_id)));
  }

  /// The quirk the module documentation ends on, which only the
  /// accumulating form can show: the layer overlap test at
  /// `pcbnew/router/pns_item.cpp:168` throws away the answer and not the
  /// obstacles the hole recursion has already appended.
  #[test]
  fn an_obstacle_found_through_a_hole_survives_the_layer_test() {
    let mut arena = Arena::new();
    let rules = TestRules::uniform(CLEARANCE);

    // A via whose copper is nowhere near the head's layer, drilled by a
    // hole that is.
    let mut drilled = via(0, 0, 2000, 1000, 1);
    drilled.set_layers_and_flash_all(LayerRange::new(5, 6));
    let (via_id, hole_id) =
      store_with_hole(&mut arena, drilled, hole(1, 0, 500));
    let head_id = store(&mut arena, segment(2, 0, 100, 2));

    let mut found = Vec::new();
    let any = collide_into(
      &arena,
      stored(&arena, via_id),
      stored(&arena, head_id),
      0,
      &rules,
      &CollisionSearchOptions::default(),
      &mut found,
    );

    assert!(!any);
    assert_eq!(found.len(), 1);
    assert_eq!(found[0].item, Some(hole_id));
  }
}
