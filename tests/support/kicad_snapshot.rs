// SPDX-License-Identifier: GPL-3.0-or-later

//! Turning a KiCad board and its project file into the two values the
//! engine takes: a [`WorldSnapshot`] and a [`RuleResolver`].
//!
//! This is the Rust side of `PNS_KICAD_IFACE_BASE::SyncWorld`
//! (`pcbnew/router/pns_kicad_iface.cpp:2292`) and of
//! `PNS_PCBNEW_RULE_RESOLVER` (`:865`), written against the neutral
//! reader types in [`super::kicad_pcb`] rather than against KiCad's own
//! board classes. Everything KiCad's sync decides object by object is
//! reproduced here with the citation next to it; note 05 section 3 is the
//! prose version of the same walk.
//!
//! # What becomes what
//!
//! | KiCad object | Snapshot item |
//! | --- | --- |
//! | track segment | [`WorldGeometry::Segment`] on its copper layer |
//! | via | [`WorldGeometry::Via`] plus the hole it drills |
//! | pad | one [`WorldGeometry::Solid`] per distinct padstack layer |
//! | `Edge.Cuts` graphic | zero width solids on every copper layer |
//! | arc track | [`WorldGeometry::Arc`] on its copper layer |
//! | rule area | skipped, counted in [`HostMap::skipped_keepouts`] |
//! | filled zone | never synced, as in KiCad (`:1894`) |
//!
//! # Two places this deviates, and why
//!
//! 1. **A uniform pad becomes one solid, not one per copper layer.**
//!    `syncPad`'s per layer lambda runs under
//!    `Padstack().ForEachUniqueLayer` (`:1743`), and for a `NORMAL`
//!    padstack, which is every pad in the regression corpus, that fires
//!    once and the solid takes the whole span (`:1687`). Splitting a
//!    through hole pad into one solid per layer would give it one joint
//!    per layer instead of one joint spanning the stack, and the pad
//!    would stop connecting the two sides of the board for
//!    [`pnsrouter::topology`]. The hole rides on that one solid, which
//!    also removes the "which layer carries the hole" question.
//! 2. **A custom pad becomes the convex hull of its primitives.** KiCad
//!    takes outline 0 of the effective polygon (`:1735`); a convex hull
//!    of the anchor rectangle and every primitive point is a conservative
//!    superset of that outline and is what [`SimplePolygon`], which
//!    assumes convexity, can hold.
//!
//! A **rounded rectangle** pad used to be the third of these, mapped to
//! its sharp rectangle. It is not an approximation any more:
//! [`rounded_rectangle_shape`] builds the same `ERROR_OUTSIDE` polygon
//! KiCad's polygon branch does (`:1733`), because the difference is
//! visible to the router and not only to the eye. A `SHAPE_RECT` gets an
//! octagonal hull with no chamfer (`pcbnew/router/pns_utils.cpp:488`)
//! where a `SHAPE_SIMPLE` gets `ConvexHull` (`:300`), whose diagonals are
//! pushed in until they touch the outline, and the corpus case
//! `drag-acute-fallback` walks around exactly such a pad.
//!
//! An oval pad is **not** approximated: KiCad builds exactly one
//! `SHAPE_SEGMENT` for it (`pcbnew/pad.cpp:1436`), so the capsule below
//! is the same shape, to the nanometre.
//!
//! # Nets
//!
//! A net is [`NetId`] of its index in [`KicadBoard::nets`], and index 0
//! is KiCad's unconnected net. That reproduces two KiCad behaviours at
//! once: [`RuleResolver::net_code`] answers zero for it, which is what
//! the topology code reads as "no net"
//! (`pcbnew/router/pns_topology.cpp:123`), and two unconnected copper
//! items compare as the same net, because `syncPad` hands the pad's
//! `NETINFO_ITEM` straight through (`:1691`) and the clearance ladder
//! compares handles (`:902`). Only the board outline gets a null net,
//! which is the case KiCad spells `nullptr` (note 05 section 3.8).
//!
//! [`RuleResolver::orphaned_net`], the net a route started in free space
//! carries, is `NetId(u32::MAX)`, an index outside the table, and
//! [`RuleResolver::net_code`] answers `-1` for it. KiCad can reuse the
//! unconnected net's code there because its orphan is a separate
//! `NETINFO_ITEM` and the router compares handles; an index cannot be
//! separate from itself, so the orphan gets an index of its own.

use std::f64::consts::FRAC_1_SQRT_2;

use pnsrouter::geometry::hull::monotone_chain_hull;
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::math::{kiround, kiround_i64};
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{
  HostId, Item, ItemBody, Kind, LayerRange, NetId, ViaType,
};
use pnsrouter::rules::{ItemRef, Keepout, RuleResolver};
use pnsrouter::settings::Sizes;
use pnsrouter::snapshot::{
  WorldGeometry, WorldItem, WorldItemFlags, WorldSnapshot,
};

use super::json::{self, JsonValue};
use super::kicad_dru::{ConstraintKind, DesignRules, ItemType, RuleItem};
use super::kicad_pcb::{
  KicadArc, KicadBoard, KicadGraphic, KicadPad, KicadSegment, KicadVia,
  PadDrill, PadKind, PadShape, Point, rotate_point,
};
use super::pns_log::LogError;

/// Slack added on top of the worst clearance the rules can return.
///
/// KiCad sets `worstClearance + ClearanceEpsilon()` exactly
/// (`pcbnew/router/pns_kicad_iface.cpp:2452`). A tenth of a millimetre is
/// added here because understating
/// [`WorldSnapshot::max_clearance`] loses obstacles in the broad phase
/// without any error, while overstating it only costs candidates the
/// narrow phase throws away.
pub const CLEARANCE_MARGIN_NANOMETRES: i32 = 100_000;

/// The largest sagitta an arc approximation may leave behind, in
/// nanometres.
///
/// Board outline arcs are flattened into inscribed chords, so the
/// approximated outline lies inside the true curve by at most this much
/// and the router is pushed away from the true edge rather than towards
/// it.
pub const ARC_SAGITTA_NANOMETRES: i64 = 10_000;

/// The slack KiCad subtracts from every positive clearance, in
/// nanometres.
///
/// `ADVANCED_CFG::m_DRCEpsilon` defaults to `0.0005` mm
/// (`common/advanced_config.cpp:251`, whose comment calls 0.5 um "small
/// enough not to materially violate any constraints"),
/// `BOARD_DESIGN_SETTINGS::GetDRCEpsilon` converts it to internal units
/// (`pcbnew/board_design_settings.cpp:2067`) and
/// `PNS_PCBNEW_RULE_RESOLVER` reads it once in its constructor
/// (`pcbnew/router/pns_kicad_iface.cpp:338`). No board in the corpus
/// overrides it, and it is an advanced configuration key rather than a
/// board property, so it is a constant here rather than something read
/// out of the `.kicad_pro`.
pub const DRC_EPSILON_NANOMETRES: i32 = 500;

/// The largest deviation a polygonised arc may leave behind, in
/// nanometres.
///
/// `BOARD_DESIGN_SETTINGS::m_MaxError`, whose default is `ARC_HIGH_DEF`,
/// itself `0.005` mm (`pcbnew/board_design_settings.cpp:217`,
/// `include/base_units.h:137`). It is what
/// [`rounded_rectangle_outline`] hands `GetArcToSegmentCount`. Only
/// `walk-with-teardrops` even writes `max_error` into its project file
/// and it writes the default, so this is a constant too.
pub const MAX_ERROR_NANOMETRES: i32 = 5_000;

// ---------------------------------------------------------------------
// The host map
// ---------------------------------------------------------------------

/// What kind of board object a [`HostId`] stands for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostKind {
  /// A track segment.
  Segment,
  /// A curved track segment.
  Arc,
  /// A via.
  Via,
  /// A footprint pad.
  Pad,
  /// One piece of an `Edge.Cuts` graphic.
  BoardOutline,
}

/// One board object the snapshot describes.
#[derive(Debug, Clone)]
pub struct HostEntry {
  /// The handle the snapshot and the commit diff speak in.
  pub host: HostId,
  /// The object's KIID, which is what a `pns.log` event refers to items
  /// by.
  pub uuid: String,
  /// What the object is.
  pub kind: HostKind,
}

/// The two way map between a KIID and the [`HostId`] the snapshot used.
///
/// KiCad's replay resolves an event's KIID to a `BOARD_ITEM*` and then to
/// a `PNS::ITEM*` through `NODE::FindItemByParent`
/// (`qa/tools/pns/pns_log_player.cpp:116`). Here the first half is this
/// map and the second half is [`pnsrouter::snapshot::HostIndex`], which
/// the router owns.
#[derive(Debug, Clone, Default)]
pub struct HostMap {
  /// Every object, in the order the snapshot listed it.
  entries: Vec<HostEntry>,
  /// Arc tracks the conversion had to drop.
  ///
  /// Zero for every board since work item 012 slice 5 gave the crate an
  /// arc body: an arc track becomes a [`WorldGeometry::Arc`] like any
  /// other obstacle. The counter is kept so that a future conversion that
  /// has to drop one has somewhere to say so, and so that the assertion
  /// in `tests/kicad_replay.rs` keeps watching.
  pub skipped_arcs: usize,
  /// Rule areas the conversion had to drop; see
  /// [`snapshot_from_board`].
  pub skipped_keepouts: usize,
  /// Pads with neither copper nor a hole, which `syncPad` also drops
  /// (`pcbnew/router/pns_kicad_iface.cpp:1626`).
  pub skipped_pads: usize,
}

impl HostMap {
  /// The object with this KIID, if the snapshot described one.
  ///
  /// A KIID that names a footprint pad, a track, a via or an outline
  /// piece resolves; one that names anything else does not, which is the
  /// same answer KiCad's `ItemsById` gives for an object the sync never
  /// added (`qa/tools/pns/pns_log_file.cpp:451`).
  pub fn host_of_uuid(&self, uuid: &str) -> Option<HostId> {
    self
      .entries
      .iter()
      .find(|entry| entry.uuid == uuid)
      .map(|entry| entry.host)
  }

  /// The object behind a handle.
  pub fn entry(&self, host: HostId) -> Option<&HostEntry> {
    self.entries.iter().find(|entry| entry.host == host)
  }

  /// Every object, in snapshot order.
  pub fn entries(&self) -> &[HostEntry] {
    &self.entries
  }

  /// How many objects the snapshot described.
  pub fn len(&self) -> usize {
    self.entries.len()
  }

  /// Whether the snapshot described nothing at all.
  pub fn is_empty(&self) -> bool {
    self.entries.is_empty()
  }

  /// Hand out the next handle and record what it stands for.
  ///
  /// Handles start at one so that `HostId(0)` stays available as an
  /// obviously wrong value in a test message.
  fn allocate(&mut self, uuid: &str, kind: HostKind) -> HostId {
    let host = HostId(self.entries.len() as u64 + 1);

    self.entries.push(HostEntry {
      host,
      uuid: uuid.to_string(),
      kind,
    });

    host
  }
}

// ---------------------------------------------------------------------
// Rules
// ---------------------------------------------------------------------

/// One entry of `net_settings.classes` in a `.kicad_pro`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetClass {
  /// The class name, `Default` for the fallback class.
  pub name: String,
  /// KiCad's `priority`, where a **smaller** number wins. The default
  /// class ships `2147483647`.
  pub priority: i64,
  /// Copper to copper clearance in nanometres.
  pub clearance: i32,
  /// Track width in nanometres.
  pub track_width: i32,
  /// Via copper diameter in nanometres.
  pub via_diameter: i32,
  /// Via drill diameter in nanometres.
  pub via_drill: i32,
}

/// The `board.design_settings.rules` block of a `.kicad_pro`.
///
/// These are the board wide minima `ImportSizes`
/// (`pcbnew/router/pns_kicad_iface.cpp:1102`) folds into the sizes with
/// `max`, and the implicit DRC rules the clearance ladder resolves when
/// no user rule applies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BoardRules {
  /// `min_clearance`, KiCad's `bds.m_MinClearance`.
  pub min_clearance: i32,
  /// `min_track_width`, KiCad's `bds.m_TrackMinWidth`.
  pub min_track_width: i32,
  /// `min_hole_clearance`, the hole to copper rule.
  pub min_hole_clearance: i32,
  /// `min_hole_to_hole`.
  pub min_hole_to_hole: i32,
  /// `min_copper_edge_clearance`, the copper to board edge rule.
  pub min_copper_edge_clearance: i32,
  /// `min_via_diameter`, KiCad's `bds.m_ViasMinSize`.
  pub min_via_diameter: i32,
  /// `min_through_hole_diameter`, KiCad's `bds.m_MinThroughDrill`.
  pub min_through_hole_diameter: i32,
}

/// A [`RuleResolver`] over a KiCad project's net classes, board rules and
/// custom design rules.
///
/// The ladder in [`KicadRules::clearance`] is
/// `PNS_PCBNEW_RULE_RESOLVER::Clearance`
/// (`pcbnew/router/pns_kicad_iface.cpp:865` to `:983`) with each
/// `QueryConstraint` call replaced by the value the project files hold:
/// `CT_HOLE_TO_HOLE` by [`BoardRules::min_hole_to_hole`],
/// `CT_HOLE_CLEARANCE` by [`BoardRules::min_hole_clearance`],
/// `CT_CLEARANCE` by the net class clearance folded with
/// [`BoardRules::min_clearance`] through `max` and then overridden by a
/// matching `clearance` rule of the `.kicad_dru`, `CT_EDGE_CLEARANCE` by
/// [`BoardRules::min_copper_edge_clearance`], and the two physical rungs
/// (`:951`, `:960`) by [`super::kicad_dru`].
///
/// # What is not supported
///
/// - **A `.kicad_dru` rule outside the modelled subset**, which is
///   dropped and raises [`KicadRules::has_unsupported_design_rules`]; see
///   [`super::kicad_dru`] for what the subset is.
/// - **The parent class of a hole.** A `.kicad_dru` condition reads the
///   hole's parent pad or via (`HOLE::BoardItem`,
///   `pcbnew/router/pns_hole.h:74`) and [`RuleResolver`] hands a
///   resolver no arena to follow [`pnsrouter::item::Item::parent_pad_via`]
///   with, so a hole answers `Via` here, which is also KiCad's fallback
///   for a parentless one (`getBoardItem`, `:516`). Every pad in the one
///   board that has design rules is surface mount, so no hole on it has
///   a pad for a parent.
/// - **Per pad clearance overrides** (`:2367`), which no board in the
///   corpus sets.
/// - **`IsNonPlatedSlot`**, which needs the parent pad's drill shape
///   (`:494`); it answers false here, so the castellation path of the
///   collision ladder is never taken.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KicadRules {
  /// Every net class of the project, in file order.
  classes: Vec<NetClass>,
  /// Index into [`KicadRules::classes`] of the class named `Default`.
  default_class: usize,
  /// The board wide minima.
  board: BoardRules,
  /// The class each net of [`KicadBoard::nets`] resolves to, by index.
  net_class: Vec<usize>,
  /// The custom design rules, empty for a case that ships no
  /// `.kicad_dru`.
  design_rules: DesignRules,
  /// Whether the `.kicad_dru` held a rule this resolver dropped.
  pub has_unsupported_design_rules: bool,
}

impl KicadRules {
  /// The rules of a project file, resolved against a board's net table.
  ///
  /// `project` is the text of the `.kicad_pro`. [`None`] gives KiCad's
  /// own defaults for a project that ships none, which two cases in the
  /// corpus do.
  ///
  /// # Errors
  ///
  /// When the project file is not JSON, or a value that has to be a
  /// number is not one.
  pub fn from_project(
    project: Option<&str>,
    board: &KicadBoard,
  ) -> Result<Self, LogError> {
    let Some(text) = project else {
      return Ok(Self::fallback(board));
    };

    let document = json::parse(text)?;
    let net_settings = document.member("net_settings");
    let classes = match net_settings.and_then(|value| value.member("classes")) {
      None => default_classes(),
      Some(value) => {
        let mut classes = Vec::new();

        for entry in value.array()? {
          classes.push(read_net_class(entry)?);
        }

        if classes.is_empty() {
          default_classes()
        } else {
          classes
        }
      }
    };

    let default_class = classes
      .iter()
      .position(|class| class.name == "Default")
      .unwrap_or(0);
    let board_rules = match document
      .member("board")
      .and_then(|value| value.member("design_settings"))
      .and_then(|value| value.member("rules"))
    {
      None => BoardRules::default(),
      Some(value) => read_board_rules(value)?,
    };

    let patterns =
      match net_settings.and_then(|value| value.member("netclass_patterns")) {
        None => Vec::new(),
        Some(value) => {
          let mut patterns = Vec::new();

          for entry in value.array()? {
            let pattern = entry.required_member("pattern")?.text()?.to_string();
            let class = entry.required_member("netclass")?.text()?.to_string();

            patterns.push((pattern, class));
          }

          patterns
        }
      };

    let net_class = board
      .nets
      .iter()
      .map(|net| resolve_class(&classes, &patterns, default_class, &net.name))
      .collect();

    Ok(Self {
      classes,
      default_class,
      board: board_rules,
      net_class,
      design_rules: DesignRules::default(),
      has_unsupported_design_rules: false,
    })
  }

  /// Take on the custom design rules of a `.kicad_dru`.
  ///
  /// The engine reads no files, so the case loader reads the text and
  /// hands it over here. KiCad's harness does the same thing in
  /// `PNS_LOG_FILE::Load`, which derives the rules path from the log path
  /// and gives it to `DRC_ENGINE::InitEngine`
  /// (`qa/tools/pns/pns_log_file.cpp:551`); a case with no such file gets
  /// an engine with implicit rules only (`:555`).
  pub fn set_design_rules(&mut self, design_rules: DesignRules) {
    self.has_unsupported_design_rules = design_rules.has_unsupported();
    self.design_rules = design_rules;
  }

  /// The custom design rules this resolver answers with.
  pub const fn design_rules(&self) -> &DesignRules {
    &self.design_rules
  }

  /// KiCad's own defaults, for a case that ships no project file.
  ///
  /// The numbers are `NETCLASS`'s constructor defaults as the settings
  /// framework writes them, which is what a project file with no
  /// `net_settings` block would load.
  fn fallback(board: &KicadBoard) -> Self {
    let classes = default_classes();

    Self {
      default_class: 0,
      net_class: vec![0; board.nets.len()],
      classes,
      board: BoardRules::default(),
      design_rules: DesignRules::default(),
      has_unsupported_design_rules: false,
    }
  }

  /// The board wide minima.
  pub const fn board(&self) -> BoardRules {
    self.board
  }

  /// Every net class, in file order.
  pub fn classes(&self) -> &[NetClass] {
    &self.classes
  }

  /// The class a net resolves to.
  ///
  /// The default class for a net the board does not have, which is what
  /// a null net means here.
  pub fn class_of(&self, net: Option<NetId>) -> &NetClass {
    let index = net
      .and_then(|net| self.net_class.get(net.0 as usize).copied())
      .unwrap_or(self.default_class);

    self
      .classes
      .get(index)
      .unwrap_or_else(|| &self.classes[self.default_class])
  }

  /// One side of a `.kicad_dru` query.
  fn rule_item<'rules>(&'rules self, item: ItemRef<'_>) -> RuleItem<'rules> {
    RuleItem {
      item_type: rule_item_type(item.item()),
      net_class: &self.class_of(item.item().net()).name,
    }
  }

  /// The `min` a `.kicad_dru` constraint of this kind resolves the pair
  /// to, [`None`] when no rule of that kind matches.
  fn design_rule_minimum(
    &self,
    kind: ConstraintKind,
    a: ItemRef<'_>,
    b: Option<ItemRef<'_>>,
  ) -> Option<i32> {
    if self.design_rules.rules.is_empty() {
      return None;
    }

    self
      .design_rules
      .constraint(kind, self.rule_item(a), b.map(|b| self.rule_item(b)))
      .and_then(|constraint| constraint.min)
  }

  /// The copper to copper clearance between two items.
  ///
  /// The maximum over the board minimum and both net classes, which is
  /// what KiCad's DRC engine resolves a `CT_CLEARANCE` query for a pair
  /// to, unless a `clearance` rule of the `.kicad_dru` matches. That rule
  /// **replaces** the net class value rather than joining it: the file's
  /// rules are loaded after the implicit ones and the last match wins
  /// (`pcbnew/drc/drc_engine.cpp:1048`, `:1860`), which is also why the
  /// implicit net class rules are sorted by clearance before they are
  /// added (`:445`) so that the larger of two classes fires last.
  fn copper_clearance(&self, a: ItemRef<'_>, b: Option<ItemRef<'_>>) -> i32 {
    if let Some(min) = self.design_rule_minimum(ConstraintKind::Clearance, a, b)
    {
      return min;
    }

    self
      .board
      .min_clearance
      .max(self.class_of(a.item().net()).clearance)
      .max(self.class_of(b.and_then(|b| b.item().net())).clearance)
  }

  /// The largest clearance this resolver can ever return.
  ///
  /// KiCad accumulates the same number as `worstClearance`
  /// (`pcbnew/router/pns_kicad_iface.cpp:2300`), which is
  /// `BOARD_DESIGN_SETTINGS::GetBiggestClearanceValue`
  /// (`pcbnew/board_design_settings.cpp:1795`) folding in the worst
  /// `clearance`, `physical_clearance` and `physical_hole_clearance` any
  /// rule can give, and hands it to `SetMaxClearance` (`:2452`).
  /// Understating it would lose the pair the physical rule exists for in
  /// the broad phase.
  pub fn worst_clearance(&self) -> i32 {
    let mut worst = self
      .board
      .min_clearance
      .max(self.board.min_hole_clearance)
      .max(self.board.min_hole_to_hole)
      .max(self.board.min_copper_edge_clearance);

    for class in &self.classes {
      worst = worst.max(class.clearance);
    }

    for kind in [
      ConstraintKind::Clearance,
      ConstraintKind::PhysicalClearance,
      ConstraintKind::PhysicalHoleClearance,
    ] {
      worst = worst.max(self.design_rules.worst_minimum(kind));
    }

    worst
  }

  /// What [`WorldSnapshot::max_clearance`] has to be for this board.
  pub fn max_clearance(&self) -> i32 {
    self
      .worst_clearance()
      .saturating_add(CLEARANCE_MARGIN_NANOMETRES)
  }

  /// The sizes a route off `start_net` is placed with.
  ///
  /// Port of `PNS_KICAD_IFACE_BASE::ImportSizes`
  /// (`pcbnew/router/pns_kicad_iface.cpp:1102`), which the replay calls
  /// before every session starting event
  /// (`qa/tools/pns/pns_log_player.cpp:135`), for a board with no
  /// `.kicad_prl`: `m_UseConnectedTrackWidth` is then false and both
  /// `UseNetClassTrack` and `UseNetClassVia` are true, so the width and
  /// the via sizes come from the net class folded with the board minima
  /// through `max` (`:1159`, `:1203`, `:1210`).
  ///
  /// With no start item KiCad falls through to
  /// `bds.GetCurrentTrackWidth()` and `bds.GetCurrentViaSize()`, which at
  /// the default index return the **default** net class values
  /// (`pcbnew/board_design_settings.cpp:1917`, `:1876`); passing
  /// [`None`] here reproduces that, because [`KicadRules::class_of`]
  /// answers with the default class for a null net.
  ///
  /// The `sizes` block a `pns.log` records alongside an event is
  /// deliberately not read, exactly as KiCad's player does not read it
  /// (`qa/tools/pns/pns_log_player.cpp:135`): two of the four routable
  /// cases in the corpus carry `SIZES_SETTINGS`' constructor defaults
  /// there rather than the sizes their session ran with, while the
  /// golden they store was produced with the numbers this routine
  /// computes.
  pub fn import_sizes(
    &self,
    start_net: Option<NetId>,
    copper_layer_count: u8,
  ) -> Sizes {
    let class = self.class_of(start_net);
    let mut sizes = Sizes {
      // :1111, then :1139 where a class clearance at or above the board
      // minimum replaces it.
      clearance: self.board.min_clearance.max(class.clearance),
      min_clearance: self.board.min_clearance,
      // :1186
      track_width: self.board.min_track_width.max(class.track_width),
      // :1188, `!m_UseConnectedTrackWidth || m_TempOverrideTrackWidth`.
      track_width_is_explicit: true,
      // :1187
      board_min_track_width: self.board.min_track_width,
      via_type: ViaType::Through,
      // :1203 and :1210
      via_diameter: self.board.min_via_diameter.max(class.via_diameter),
      via_drill: self.board.min_through_hole_diameter.max(class.via_drill),
      hole_to_hole: self.board.min_hole_to_hole,
      ..Sizes::default()
    };

    // `Sizes::via_layer_range` has no board to ask, so the outermost
    // pair is registered here; see its documentation for the deviation
    // that makes this necessary.
    sizes.add_layer_pair(0, i32::from(copper_layer_count.max(1)) - 1);
    sizes
  }
}

/// Whether an item is a piece of the `Edge.Cuts` outline.
///
/// KiCad asks the parent board item's layer (`isEdge`,
/// `pcbnew/router/pns_kicad_iface.cpp:452`). There is no parent here, so
/// the test is the shape [`snapshot_from_board`] gives an outline piece
/// and gives nothing else: a solid whose copper is a capsule of zero
/// width. `syncGraphicalItem` forces that width to zero for `Edge.Cuts`
/// and only for `Edge.Cuts` (`:2070`).
fn is_board_edge(item: &Item) -> bool {
  let ItemBody::Solid(solid) = item.body() else {
    return false;
  };

  matches!(solid.shape(), Some(Shape::Segment { width: 0, .. }))
}

/// Whether an item counts as copper for the clearance ladder.
///
/// Port of `isCopper` (`pcbnew/router/pns_kicad_iface.cpp:433`), which
/// asks whether the parent board item is on a copper layer and therefore
/// answers **true** for a hole, whose parent is the pad or via that
/// drilled it. Only the board outline is not copper.
fn is_copper(item: &Item) -> bool {
  !is_board_edge(item)
}

/// The class name a `.kicad_dru` condition sees as `A.Type`.
///
/// KiCad gives the DRC engine the item's parent board item, or, for a
/// router temporary that has none, a dummy of the matching board class
/// (`getBoardItem`, `pcbnew/router/pns_kicad_iface.cpp:505`), and
/// `A.Type` is that class's name in `EDA_ITEM_DESC`
/// (`common/eda_item.cpp:557`). The three groupings below are that
/// switch: a segment and a line both become a `PCB_TRACK` (`:524`), an
/// arc a `PCB_ARC` whose name is also `Track`, and a via **or a hole** a
/// `PCB_VIA` (`:516`). See the [`KicadRules`] documentation for why a
/// hole cannot ask its own parent here.
fn rule_item_type(item: &Item) -> ItemType {
  let kind = item.kind();

  if kind == Kind::SEGMENT || kind == Kind::LINE || kind == Kind::ARC {
    ItemType::Track
  } else if kind == Kind::VIA || kind == Kind::HOLE {
    ItemType::Via
  } else if is_board_edge(item) {
    ItemType::Graphic
  } else {
    ItemType::Pad
  }
}

impl RuleResolver for KicadRules {
  /// The ladder of `PNS_PCBNEW_RULE_RESOLVER::Clearance`,
  /// `pcbnew/router/pns_kicad_iface.cpp:865`, with the layer loop
  /// collapsed because every rule here is the same on every layer.
  ///
  /// The two physical rungs (`:951`, `:960`) sit outside the `!sameNet`
  /// guard the copper rung is under, which is the whole point of them: a
  /// physical rule is net blind. Their value then keeps the same net
  /// short circuit at `:968` from firing, because that one only turns a
  /// pair off when the accumulated clearance is still zero.
  fn clearance(
    &self,
    a: ItemRef<'_>,
    b: Option<ItemRef<'_>>,
    use_epsilon: bool,
  ) -> Option<i32> {
    let a_hole = self.is_drilled_hole(a);
    let b_hole = b.is_some_and(|b| self.is_drilled_hole(b));
    // :902. A null net is never the same as anything.
    let same_net = b.is_some_and(|b| {
      a.item().net().is_some() && a.item().net() == b.item().net()
    });
    // :903
    let free_pad =
      b.is_some_and(|b| a.item().is_free_pad() || b.item().is_free_pad());
    let mut result = 0;

    if a_hole && b_hole {
      // :908
      result = result.max(self.board.min_hole_to_hole);
    } else if (a_hole || b_hole) && !same_net {
      // :916
      result = result.max(self.board.min_hole_clearance);
    }

    // :929, an independent `if` and not an `else`, so a plated hole
    // picks up the copper rule on top of its hole rule.
    if is_copper(a.item())
      && b.is_none_or(|b| is_copper(b.item()))
      && !same_net
      && !free_pad
    {
      result = result.max(self.copper_clearance(a, b));
    }

    // :941, net blind: an edge clearance applies whatever the nets are.
    if is_board_edge(a.item()) || b.is_some_and(|b| is_board_edge(b.item())) {
      result = result.max(self.board.min_copper_edge_clearance);
    }

    // :951, the hole half of the net blind pair. `isHole` (`:444`) is
    // the kind test, which is what `is_drilled_hole` answers here too.
    if a_hole || b_hole {
      result = result.max(
        self
          .design_rule_minimum(ConstraintKind::PhysicalHoleClearance, a, b)
          .unwrap_or(0),
      );
    }

    // :960, and this one applies to every pair there is.
    result = result.max(
      self
        .design_rule_minimum(ConstraintKind::PhysicalClearance, a, b)
        .unwrap_or(0),
    );

    if (same_net || free_pad) && result == 0 {
      // :968
      return None;
    }

    if use_epsilon && result > 0 {
      // :971
      result = (result - self.clearance_epsilon()).max(0);
    }

    Some(result)
  }

  /// KiCad's DRC epsilon, [`DRC_EPSILON_NANOMETRES`].
  ///
  /// `PNS_PCBNEW_RULE_RESOLVER`'s constructor takes it from the board
  /// (`pcbnew/router/pns_kicad_iface.cpp:338`) and subtracts it from
  /// every positive clearance the ladder answers with (`:972`), which is
  /// the subtraction above. Note 05 section 7.5 says a host **without**
  /// the notion, LibrePCB among them, leaves
  /// [`RuleResolver::clearance_epsilon`] at its zero default; this
  /// resolver emulates KiCad, which has one.
  fn clearance_epsilon(&self) -> i32 {
    DRC_EPSILON_NANOMETRES
  }

  /// Whether the `.kicad_dru` holds a conditional physical rule.
  ///
  /// Port of `PNS_PCBNEW_RULE_RESOLVER::HasUserDefinedPhysicalConstraint`
  /// (`pcbnew/router/pns_kicad_iface.cpp:851`), which forwards to
  /// `DRC_ENGINE::HasUserDefinedPhysicalConstraint`
  /// (`pcbnew/drc/drc_engine.cpp:2449`). KiCad memoises it because the
  /// collision inner loop asks on every pair; there is nothing to
  /// memoise here, the answer is a walk of at most a handful of rules.
  fn has_user_defined_physical_constraint(&self) -> bool {
    self.design_rules.has_conditional_physical_constraint()
  }

  /// Never a keepout: rule areas are not synced, see
  /// [`snapshot_from_board`].
  fn is_keepout(&self, _obstacle: ItemRef<'_>, _item: ItemRef<'_>) -> Keepout {
    Keepout::None
  }

  /// Every hole. `IsDrilledHole`
  /// (`pcbnew/router/pns_kicad_iface.cpp:464`) also requires the parent
  /// to have a drilled hole, and every hole this conversion creates comes
  /// from a pad drill or a via drill, so the two agree.
  fn is_drilled_hole(&self, item: ItemRef<'_>) -> bool {
    item.item().kind() == Kind::HOLE
  }

  /// Never. See the type documentation.
  fn is_non_plated_slot(&self, _item: ItemRef<'_>) -> bool {
    false
  }

  /// The net's index in the board's net table, so index zero, KiCad's
  /// unconnected net, reads as "no net" to the topology code. The orphan
  /// below is the one index that is not a table entry and answers `-1`.
  fn net_code(&self, net: NetId) -> i32 {
    if net == self.orphaned_net() {
      return -1;
    }

    i32::try_from(net.0).unwrap_or(i32::MAX)
  }

  /// A net index no board table can reach, standing in for KiCad's
  /// `NETINFO_LIST::OrphanedItem()`
  /// (`pcbnew/router/pns_kicad_iface.cpp:3020`).
  ///
  /// KiCad's orphan is a distinct `NETINFO_ITEM` that happens to carry
  /// the same net code as the board's unconnected net, and the router
  /// tells the two apart by comparing handles. A [`NetId`] here **is**
  /// the table index, so the two cannot be told apart that way and the
  /// orphan takes an index instead: `u32::MAX`, which no board of
  /// `u32::MAX` nets could reach and which the reader never assigns.
  /// Its net code has to be its own case, because `net_code` above would
  /// otherwise saturate it to `i32::MAX` and report the orphan as a real
  /// net.
  fn orphaned_net(&self) -> NetId {
    NetId(u32::MAX)
  }
}

/// KiCad's `Default` net class as the settings framework writes it.
fn default_classes() -> Vec<NetClass> {
  vec![NetClass {
    name: "Default".to_string(),
    priority: i64::from(i32::MAX),
    clearance: 200_000,
    track_width: 200_000,
    via_diameter: 600_000,
    via_drill: 300_000,
  }]
}

/// Read one entry of `net_settings.classes`.
fn read_net_class(value: &JsonValue) -> Result<NetClass, LogError> {
  Ok(NetClass {
    name: value.required_member("name")?.text()?.to_string(),
    priority: match value.member("priority") {
      None => i64::from(i32::MAX),
      Some(entry) => entry.integer()?,
    },
    clearance: millimetre_member(value, "clearance", 200_000)?,
    track_width: millimetre_member(value, "track_width", 200_000)?,
    via_diameter: millimetre_member(value, "via_diameter", 600_000)?,
    via_drill: millimetre_member(value, "via_drill", 300_000)?,
  })
}

/// Read `board.design_settings.rules`.
fn read_board_rules(value: &JsonValue) -> Result<BoardRules, LogError> {
  Ok(BoardRules {
    min_clearance: millimetre_member(value, "min_clearance", 0)?,
    min_track_width: millimetre_member(value, "min_track_width", 0)?,
    min_hole_clearance: millimetre_member(value, "min_hole_clearance", 0)?,
    min_hole_to_hole: millimetre_member(value, "min_hole_to_hole", 0)?,
    min_copper_edge_clearance: millimetre_member(
      value,
      "min_copper_edge_clearance",
      0,
    )?,
    min_via_diameter: millimetre_member(value, "min_via_diameter", 0)?,
    min_through_hole_diameter: millimetre_member(
      value,
      "min_through_hole_diameter",
      0,
    )?,
  })
}

/// Read a millimetre valued member as whole nanometres.
///
/// Unlike the `.kicad_pcb` reader, which converts decimal text digit by
/// digit, this rounds an `f64`. It has to: the settings framework writes
/// these numbers back through a `double`, so
/// `simple-shove-1/pns.kicad_pro` really does spell 0.2 mm as
/// `0.19999999999999998`, which no exact decimal conversion can turn into
/// a whole number of nanometres. Rounding to the nearest nanometre
/// recovers the value the user typed.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
fn millimetre_member(
  value: &JsonValue,
  name: &str,
  fallback: i32,
) -> Result<i32, LogError> {
  let Some(entry) = value.member(name) else {
    return Ok(fallback);
  };
  let millimetres = entry.double()?;
  let nanometres = (millimetres * 1_000_000.0).round();

  if !nanometres.is_finite()
    || nanometres < f64::from(i32::MIN)
    || nanometres > f64::from(i32::MAX)
  {
    return Err(LogError::new(
      entry.line(),
      format!("`{name}` is {millimetres} mm, which is not a board value"),
    ));
  }

  Ok(nanometres as i32)
}

/// Which class a net name resolves to.
///
/// KiCad composes an effective class out of every matching pattern
/// (`NET_SETTINGS::GetEffectiveNetClass`); this takes the single matching
/// class with the smallest `priority`, ties going to the first pattern in
/// file order, and falls back to the default class. The only case in the
/// corpus with patterns, `issue22749-shove-weird-drag-track-end`, maps two
/// literal net names onto one class, where the two agree.
fn resolve_class(
  classes: &[NetClass],
  patterns: &[(String, String)],
  default_class: usize,
  net_name: &str,
) -> usize {
  let mut best: Option<usize> = None;

  for (pattern, class_name) in patterns {
    if !matches_pattern(pattern, net_name) {
      continue;
    }

    let Some(index) =
      classes.iter().position(|class| &class.name == class_name)
    else {
      continue;
    };

    if best
      .is_none_or(|current| classes[index].priority < classes[current].priority)
    {
      best = Some(index);
    }
  }

  best.unwrap_or(default_class)
}

/// Whether a net class pattern matches a net name.
///
/// KiCad's patterns are `EDA_COMBINED_MATCHER` globs. Only `*` and `?`
/// are implemented here, because every pattern in the corpus is a literal
/// name; a pattern using anything else silently matches nothing, which
/// leaves the net on the default class.
fn matches_pattern(pattern: &str, name: &str) -> bool {
  let pattern: Vec<char> = pattern.chars().collect();
  let name: Vec<char> = name.chars().collect();
  // Classic two cursor glob match with one backtracking anchor, which
  // needs no recursion and no allocation per step.
  let (mut pattern_index, mut name_index) = (0, 0);
  let mut star: Option<(usize, usize)> = None;

  while name_index < name.len() {
    let matched = pattern_index < pattern.len()
      && (pattern[pattern_index] == '?'
        || pattern[pattern_index] == name[name_index]);

    if matched {
      pattern_index += 1;
      name_index += 1;
    } else if pattern_index < pattern.len() && pattern[pattern_index] == '*' {
      star = Some((pattern_index, name_index));
      pattern_index += 1;
    } else if let Some((star_index, resume)) = star {
      pattern_index = star_index + 1;
      name_index = resume + 1;
      star = Some((star_index, resume + 1));
    } else {
      return false;
    }
  }

  pattern[pattern_index..].iter().all(|glyph| *glyph == '*')
}

// ---------------------------------------------------------------------
// The snapshot
// ---------------------------------------------------------------------

/// Turn a board into the snapshot the engine takes.
///
/// Port of `PNS_KICAD_IFACE_BASE::SyncWorld`
/// (`pcbnew/router/pns_kicad_iface.cpp:2292`), object by object; see the
/// module documentation for the table and for the three approximations.
///
/// What is dropped, each counted on the returned [`HostMap`]: rule areas,
/// because [`RuleResolver::is_keepout`] needs the resolver to recognise
/// the obstacle and nothing on [`pnsrouter::item::Item`] carries that
/// mark, so a keepout could only be added as an unconditional obstacle,
/// which is stronger than KiCad's rule; and pads with neither copper nor
/// a hole, which `syncPad` also drops (`:1626`). Filled zones are not
/// dropped so much as never considered, matching `syncZone` (`:1894`).
pub fn snapshot_from_board(
  board: &KicadBoard,
  rules: &KicadRules,
) -> (WorldSnapshot, HostMap) {
  let layer_count = board.copper_layer_count.max(1);
  let mut snapshot = WorldSnapshot::new(
    u8::try_from(layer_count).unwrap_or(u8::MAX),
    rules.max_clearance(),
  );
  let mut map = HostMap::default();
  let whole_stack = LayerRange::new(0, layer_count as i32 - 1);

  for pad in &board.pads {
    add_pad(&mut snapshot, &mut map, pad, whole_stack);
  }

  for segment in &board.segments {
    add_segment(&mut snapshot, &mut map, segment);
  }

  for arc in &board.arcs {
    add_arc(&mut snapshot, &mut map, arc);
  }

  for via in &board.vias {
    add_via(&mut snapshot, &mut map, via);
  }

  for graphic in &board.board_outline {
    add_outline(&mut snapshot, &mut map, graphic, whole_stack);
  }

  map.skipped_keepouts = board.keepouts.len();

  (snapshot, map)
}

/// A board point as an engine point.
///
/// KiCad's board coordinates are already `int` nanometres; the reader
/// widens them to `i64` so that a malformed file cannot wrap. The clamp
/// only fires on a value no board can hold.
fn to_vec2(point: Point) -> Vec2 {
  Vec2::new(clamp_i32(point.x), clamp_i32(point.y))
}

/// Narrow a board dimension into the engine's `i32` nanometres.
fn clamp_i32(value: i64) -> i32 {
  value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// The net of a copper item.
///
/// An item that names no net at all still belongs to KiCad's unconnected
/// net, which is index zero of the reader's table; see the module
/// documentation for why that is not a null net.
fn copper_net(net: Option<usize>) -> Option<NetId> {
  Some(NetId(net.unwrap_or(0) as u32))
}

/// Add one track segment. `syncTrack`,
/// `pcbnew/router/pns_kicad_iface.cpp:1749`.
fn add_segment(
  snapshot: &mut WorldSnapshot,
  map: &mut HostMap,
  segment: &KicadSegment,
) {
  let host = map.allocate(&segment.uuid, HostKind::Segment);

  snapshot.items.push(WorldItem::new(
    host,
    copper_net(segment.net),
    LayerRange::single(segment.copper_layer as i32),
    WorldGeometry::Segment {
      seg: Seg::new(to_vec2(segment.start), to_vec2(segment.end)),
      width: clamp_i32(segment.width),
    },
  ));
}

/// Add one arc track. `syncArc`,
/// `pcbnew/router/pns_kicad_iface.cpp:1770`, which is four lines of
/// substance: a `SHAPE_ARC` of the `PCB_ARC`'s own start, mid, end and
/// width, the net, the layer and the parent. `PCB_ARC` stores the same
/// three points, so nothing is converted and nothing is rounded.
///
/// The lock marker of `:1779` is not read here for the same reason
/// [`add_segment`] does not read a segment's: the corpus reader carries no
/// lock flag.
fn add_arc(snapshot: &mut WorldSnapshot, map: &mut HostMap, arc: &KicadArc) {
  let host = map.allocate(&arc.uuid, HostKind::Arc);

  snapshot.items.push(WorldItem::new(
    host,
    copper_net(arc.net),
    LayerRange::single(arc.copper_layer as i32),
    WorldGeometry::Arc {
      start: to_vec2(arc.start),
      mid: to_vec2(arc.mid),
      end: to_vec2(arc.end),
      width: clamp_i32(arc.width),
    },
  ));
}

/// Add one via and the hole it drills. `syncVia`,
/// `pcbnew/router/pns_kicad_iface.cpp:1792`, whose hole is a separate
/// item attached with `SetHole` (`:1863`); [`pnsrouter::node::World`]
/// drills it from the drill diameter instead.
fn add_via(snapshot: &mut WorldSnapshot, map: &mut HostMap, via: &KicadVia) {
  let host = map.allocate(&via.uuid, HostKind::Via);

  snapshot.items.push(WorldItem::new(
    host,
    copper_net(via.net),
    LayerRange::new(
      via.copper_layer_top as i32,
      via.copper_layer_bottom as i32,
    ),
    WorldGeometry::Via {
      pos: to_vec2(via.at),
      diameter: clamp_i32(via.size),
      drill: clamp_i32(via.drill),
      via_type: ViaType::Through,
      is_free: via.free,
    },
  ));
}

/// Add one pad. `syncPad`, `pcbnew/router/pns_kicad_iface.cpp:1619`.
///
/// The layer span follows the switch at `:1629`: a plated or non plated
/// through hole pad keeps the whole copper stack, a surface mount or edge
/// connector pad is narrowed to the first copper layer of its layer set
/// and dropped when that leaves it with none.
fn add_pad(
  snapshot: &mut WorldSnapshot,
  map: &mut HostMap,
  pad: &KicadPad,
  whole_stack: LayerRange,
) {
  // :1626, a pad with no copper and no hole is not an obstacle.
  if pad.copper_layers.is_empty() && pad.drill.is_none() {
    map.skipped_pads += 1;

    return;
  }

  let layers = match pad.kind {
    PadKind::ThroughHole | PadKind::NonPlatedThroughHole => whole_stack,
    PadKind::SurfaceMount | PadKind::EdgeConnector => {
      let Some(layer) = pad.copper_layers.first() else {
        // :1646, no copper means no solid at all.
        map.skipped_pads += 1;

        return;
      };

      LayerRange::single(*layer as i32)
    }
  };
  let Some(shape) = pad_shape(pad) else {
    // :1737, a solid with no shape is dropped.
    map.skipped_pads += 1;

    return;
  };
  let host = map.allocate(&pad.uuid, HostKind::Pad);
  let mut item = WorldItem::new(
    host,
    copper_net(pad.net),
    layers,
    WorldGeometry::Solid {
      shape,
      pos: to_vec2(pad.at),
      // No pad in the corpus carries a `(offset ...)`, so the rotated
      // offset of `:1700` is zero and the position is the pad centre.
      offset: Vec2::new(0, 0),
      orientation_degrees: pad.rotation_degrees,
      anchors: Vec::new(),
    },
  );

  item.hole = pad.drill.map(|drill| pad_hole(pad, drill));
  // :1672, which is what makes `Router::is_starting_point_routable`
  // refuse a mounting hole.
  item.flags = WorldItemFlags {
    routable: pad.kind != PadKind::NonPlatedThroughHole,
    ..WorldItemFlags::default()
  };
  snapshot.items.push(item);
}

/// The drilled shape of a pad.
///
/// A round drill is a circle and a slot is a capsule along the longer
/// axis, which is `PAD::GetEffectiveHoleShape` reduced to the two forms
/// `PNS::HOLE` can hold (`pcbnew/router/pns_hole.cpp:131`). The slot is
/// rotated by the pad's own orientation, as the pad's effective hole is.
fn pad_hole(pad: &KicadPad, drill: PadDrill) -> Shape {
  match drill {
    PadDrill::Round { diameter } => {
      Shape::circle(to_vec2(pad.at), clamp_i32(diameter / 2))
    }
    PadDrill::Oval { width, height } => {
      let half_width = width.min(height) / 2;
      let half_length = Point {
        x: width / 2 - half_width,
        y: height / 2 - half_width,
      };
      let rotated = rotate_point(half_length, pad.rotation_degrees);
      let start = Point {
        x: pad.at.x - rotated.x,
        y: pad.at.y - rotated.y,
      };
      let end = Point {
        x: pad.at.x + rotated.x,
        y: pad.at.y + rotated.y,
      };

      Shape::segment(
        Seg::new(to_vec2(start), to_vec2(end)),
        clamp_i32(half_width * 2),
      )
    }
  }
}

/// The copper of a pad.
///
/// Follows `PAD::buildEffectiveShape` (`pcbnew/pad.cpp:1411`) as far as
/// the one shape `syncPad` keeps when the effective shape has a single
/// indexable subshape (`:1720`), and approximates the two cases where it
/// does not; see the module documentation.
fn pad_shape(pad: &KicadPad) -> Option<Shape> {
  let center = to_vec2(pad.at);

  match &pad.shape {
    // `pcbnew/pad.cpp:1432`
    PadShape::Circle => {
      Some(Shape::circle(center, clamp_i32(pad.size.x / 2))).filter(nonzero)
    }
    // `pcbnew/pad.cpp:1436`: a square oval really is a circle.
    PadShape::Oval if pad.size.x == pad.size.y => {
      Some(Shape::circle(center, clamp_i32(pad.size.x / 2))).filter(nonzero)
    }
    PadShape::Oval => {
      let half_width = (pad.size.x / 2).min(pad.size.y / 2);
      let half_length = Point {
        x: pad.size.x / 2 - half_width,
        y: pad.size.y / 2 - half_width,
      };
      let rotated = rotate_point(half_length, pad.rotation_degrees);
      let start = Vec2::new(
        clamp_i32(pad.at.x - rotated.x),
        clamp_i32(pad.at.y - rotated.y),
      );
      let end = Vec2::new(
        clamp_i32(pad.at.x + rotated.x),
        clamp_i32(pad.at.y + rotated.y),
      );

      Some(Shape::segment(
        Seg::new(start, end),
        clamp_i32(half_width * 2),
      ))
      .filter(nonzero)
    }
    // `pcbnew/pad.cpp:1487`, a single `SHAPE_RECT` or `SHAPE_SIMPLE`. A
    // trapezoid loses its taper because the reader does not carry the
    // delta, which leaves a superset of the true copper.
    PadShape::Rectangle | PadShape::Trapezoid => {
      Some(rectangle_shape(pad)).filter(nonzero)
    }
    PadShape::RoundedRectangle { ratio } => {
      Some(rounded_rectangle_shape(pad, *ratio)).filter(nonzero)
    }
    PadShape::Custom { primitives } => custom_shape(pad, primitives),
  }
}

/// Whether a shape has any extent at all.
///
/// A degenerate pad would otherwise reach the index as a shape with an
/// empty bounding box, which `syncPad` refuses at `:1737`.
fn nonzero(shape: &Shape) -> bool {
  shape
    .bbox(0)
    .is_some_and(|bounds| bounds.width() > 0 || bounds.height() > 0)
}

/// The four cornered copper of a rectangular pad.
///
/// `pcbnew/pad.cpp:1487` rotates the corners and then recognises an axis
/// aligned result as a `SHAPE_RECT` rather than a four point polygon.
/// The same test is made here on the quarter turn, because that is the
/// only rotation the corpus uses that keeps a rectangle axis aligned.
fn rectangle_shape(pad: &KicadPad) -> Shape {
  let half = Point {
    x: pad.size.x / 2,
    y: pad.size.y / 2,
  };
  let corners: Vec<Vec2> = [
    Point {
      x: -half.x,
      y: half.y,
    },
    Point {
      x: half.x,
      y: half.y,
    },
    Point {
      x: half.x,
      y: -half.y,
    },
    Point {
      x: -half.x,
      y: -half.y,
    },
  ]
  .into_iter()
  .map(|corner| {
    let rotated = rotate_point(corner, pad.rotation_degrees);

    Vec2::new(
      clamp_i32(pad.at.x + rotated.x),
      clamp_i32(pad.at.y + rotated.y),
    )
  })
  .collect();

  if is_axis_aligned(&corners) {
    let left = corners.iter().map(|corner| corner.x).min().unwrap_or(0);
    let top = corners.iter().map(|corner| corner.y).min().unwrap_or(0);
    let right = corners.iter().map(|corner| corner.x).max().unwrap_or(0);
    let bottom = corners.iter().map(|corner| corner.y).max().unwrap_or(0);

    return Shape::rect(
      Vec2::new(left, top),
      Vec2::new(right - left, bottom - top),
    );
  }

  Shape::simple(LineChain::from_points(corners, true))
}

/// Whether four corners form an axis aligned rectangle.
///
/// The two orderings `pcbnew/pad.cpp:1490` tests for, which between them
/// cover every quarter turn.
fn is_axis_aligned(corners: &[Vec2]) -> bool {
  if corners.len() != 4 {
    return false;
  }

  (corners[0].y == corners[1].y
    && corners[1].x == corners[2].x
    && corners[2].y == corners[3].y
    && corners[3].x == corners[0].x)
    || (corners[0].x == corners[1].x
      && corners[1].y == corners[2].y
      && corners[2].x == corners[3].x
      && corners[3].y == corners[0].y)
}

// ---------------------------------------------------------------------
// The rounded rectangle pad
// ---------------------------------------------------------------------

/// The copper of a rounded rectangle pad.
///
/// A roundrect is the one pad shape whose effective shape is a
/// **compound** of five pieces, the body plus one capsule per side
/// (`pcbnew/pad.cpp:1516` to `:1521`), so
/// `shape->GetIndexableSubshapeCount() == 1` fails and `syncPad` takes
/// its polygon branch instead: `GetEffectivePolygon( aLayer,
/// ERROR_OUTSIDE )`, outline 0, wrapped in a `SHAPE_SIMPLE`
/// (`pcbnew/router/pns_kicad_iface.cpp:1733`). That matters to the
/// router and not only to the picture, because a `SHAPE_SIMPLE` gets
/// `ConvexHull` (`pcbnew/router/pns_utils.cpp:300`), whose four diagonals
/// are pushed inwards until they touch the outline, where a `SHAPE_RECT`
/// gets an octagon with no chamfer at all (`:488`).
///
/// Two shapes short circuit before the polygon, both reproduced here:
///
/// - a **zero radius** roundrect is stored as a plain rectangle, because
///   `PADSTACK`'s assignment rewrites the shape when the ratio is zero
///   (`pcbnew/padstack.cpp:124`);
/// - a roundrect whose straight part has all but vanished is built as a
///   single circle of the corner radius (`pcbnew/pad.cpp:1461` to
///   `:1470`), and one circle is one indexable subshape, so `syncPad`
///   keeps the circle.
fn rounded_rectangle_shape(pad: &KicadPad, ratio: f64) -> Shape {
  // `PADSTACK::RoundRectRadius`, `pcbnew/padstack.cpp:952`.
  let radius = kiround_i64((pad.size.x.min(pad.size.y) as f64) * ratio);

  // `pcbnew/padstack.cpp:124`.
  if radius <= 0 {
    return rectangle_shape(pad);
  }

  // `pcbnew/pad.cpp:1464`, `min_len` of 0.0001 mm.
  const MINIMUM_STRAIGHT_LENGTH_NANOMETRES: i64 = 100;

  let straight = Point {
    x: pad.size.x / 2 - radius,
    y: pad.size.y / 2 - radius,
  };

  if straight.x < MINIMUM_STRAIGHT_LENGTH_NANOMETRES
    && straight.y < MINIMUM_STRAIGHT_LENGTH_NANOMETRES
  {
    return Shape::circle(to_vec2(pad.at), clamp_i32(radius));
  }

  Shape::simple(LineChain::from_points(
    rounded_rectangle_outline(pad, clamp_i32(radius)),
    true,
  ))
}

/// The outline `GetEffectivePolygon( layer, ERROR_OUTSIDE )` builds for a
/// rounded rectangle pad.
///
/// `PAD::TransformShapeToPolygon`'s roundrect arm
/// (`pcbnew/pad.cpp:3040`) calls
/// `TransformRoundChamferedRectToPolygon`
/// (`libs/kimath/src/convert_basic_shapes_to_polygon.cpp:455`) with no
/// clearance, no chamfer and `ERROR_OUTSIDE`, and that hands four corners
/// of equal radius to `CornerListToPolygon` (`:242`). This is that path
/// with the branches a pad's four **right angled** corners can never take
/// removed: no inflation, so `aInflate` is zero throughout; no chamfer,
/// so the corner list stays four long; and both `incoming` and `outgoing`
/// axis aligned at every corner, so the `endAngle = ANGLE_90,
/// tanAngle2 = 1.0` short circuit at `:266` is the only case, which makes
/// `arcTransitionDistance` the radius itself.
///
/// # Why the arc comes out looking trimmed
///
/// `ERROR_OUTSIDE` pushes the polygon outside the true circle by
/// `radiusExtend` (`:319`, `:320`) so that the approximation never
/// understates the copper. That would leave an "ear" sticking out past
/// each straight side, so the loop at `:329` walks the arc from the side
/// inwards and takes the **first** vertex that is within the sharp
/// rectangle, joining it to the side with the chord's intersection point
/// (`:346`). The mirror of that intersection across the corner's diagonal
/// closes the arc at the other end (`:348`).
///
/// The outline is built about the origin, then rotated and moved
/// (`:513` to `:517`), which is the order the pad's own orientation has
/// to be applied in.
fn rounded_rectangle_outline(pad: &KicadPad, radius: i32) -> Vec<Vec2> {
  let half = Vec2::new(clamp_i32(pad.size.x / 2), clamp_i32(pad.size.y / 2));
  // `:477` to `:480`, in that order, which is what makes every turn a
  // left turn in KiCad's y down frame.
  let corners = [
    Vec2::new(-half.x, -half.y),
    Vec2::new(half.x, -half.y),
    Vec2::new(half.x, half.y),
    Vec2::new(-half.x, half.y),
  ];

  // `:300`. Sixteen segments per full turn is the floor, so a pad corner
  // never gets fewer than four vertices' worth of arc.
  let segments = arc_to_segment_count(radius, MAX_ERROR_NANOMETRES, 360.0)
    .max(MINIMUM_SEGMENTS_PER_TURN);
  let angle_delta = 360.0 / f64::from(segments);
  // `:319`, `:320`. `GetCircleToPolyCorrection` is the identity outside
  // a `DISABLE_ARC_RADIUS_CORRECTION` scope
  // (`libs/kimath/src/geometry/geometry_utils.cpp:102`), and nothing on
  // this path opens one.
  let radius_extend = circle_to_end_segment_delta_radius(radius, segments);
  let mut outline: Vec<Vec2> = Vec::new();
  // `:247`
  let mut incoming = corners[0] - corners[3];

  for index in 0..corners.len() {
    let corner = corners[index];
    let outgoing = corners[(index + 1) % corners.len()] - corner;
    // `:266`, the axis aligned short circuit: a right angle to sweep and
    // a transition distance of exactly the radius.
    let mut end_angle = 90.0_f64;
    let plain_start = corner - incoming.resize(radius);
    let centre = plain_start + incoming.perpendicular().resize(radius);
    // `:321`
    let start = plain_start + incoming.perpendicular().resize(-radius_extend);
    let start_origin = start - centre;
    // `:325`. KiCad's comment calls it short and to be treated as an
    // infinite line, which is what `Seg::intersect_lines` does.
    let straight_side = Seg::new(corner - incoming, corner);
    // `:327`, the answer when no arc vertex lands inside the outline.
    let mut end = corner;
    let mut previous = start;
    // `:305` to `:311`: the last segment of the sweep is made the same
    // size as the first, and the first vertex sits half a segment in.
    let mut last_segment = end_angle;

    while last_segment > angle_delta {
      last_segment -= angle_delta;
    }

    let mut angle = if last_segment == 0.0 {
      angle_delta
    } else {
      (angle_delta + last_segment) / 2.0
    };

    // `:329`
    while angle < end_angle {
      let point = rotate_about_origin(start_origin, -angle) + centre;

      angle += angle_delta;

      // `:339`
      if straight_side.side(point) > 0 {
        // `:341`. KiCad asserts the solution exists; the chord runs from
        // outside the side to inside it, so it always does.
        if let Some(crossing) =
          straight_side.intersect_lines(&Seg::new(previous, point))
        {
          append_outline_point(&mut outline, crossing);
          // `:348`
          end = Seg::new(corner, centre).reflect_point(crossing);
        }

        append_outline_point(&mut outline, point);

        break;
      }

      // `:352`, skipping the last vertex as well as the first.
      end_angle -= angle_delta;
      previous = point;
    }

    // `:356`
    while angle < end_angle {
      append_outline_point(
        &mut outline,
        rotate_about_origin(start_origin, -angle) + centre,
      );

      angle += angle_delta;
    }

    // `:363`
    append_outline_point(&mut outline, end);

    incoming = outgoing;
  }

  // `:513` to `:516`
  outline
    .into_iter()
    .map(|point| rotate_about_origin(point, pad.rotation_degrees))
    .map(|point| {
      Vec2::new(
        clamp_i32(i64::from(point.x) + pad.at.x),
        clamp_i32(i64::from(point.y) + pad.at.y),
      )
    })
    .collect()
}

/// The floor `CornerListToPolygon` puts under the segment count.
///
/// `libs/kimath/src/convert_basic_shapes_to_polygon.cpp:300`, whose
/// comment is "Ensure 16+ segments per 360deg".
const MINIMUM_SEGMENTS_PER_TURN: i32 = 16;

/// Append a point unless it repeats the previous one.
///
/// `SHAPE_LINE_CHAIN::Append`
/// (`libs/kimath/include/geometry/shape_line_chain.h:534`) drops a point
/// equal to the chain's last unless duplication is asked for, and nothing
/// on this path asks.
fn append_outline_point(outline: &mut Vec<Vec2>, point: Vec2) {
  if outline.last() != Some(&point) {
    outline.push(point);
  }
}

/// How many segments an arc of that many degrees is approximated by.
///
/// Port of `GetArcToSegmentCount`
/// (`libs/kimath/src/geometry/geometry_utils.cpp:38`). The clamp to
/// `360 / 8` is KiCad's `MIN_SEGCOUNT_FOR_CIRCLE`, which only bites for
/// radii small enough that the error bound allows a coarser step than 45
/// degrees.
fn arc_to_segment_count(radius: i32, max_error: i32, degrees: f64) -> i32 {
  let radius = f64::from(radius.max(1));
  let max_error = f64::from(max_error.max(1));
  let relative_error = max_error / radius;
  let increment =
    (180.0 / std::f64::consts::PI * (1.0 - relative_error).acos() * 2.0)
      .min(360.0 / 8.0);

  kiround(degrees.abs() / increment).max(2)
}

/// How far outside the true circle the ends of an approximating segment
/// sit.
///
/// Port of `CircleToEndSegmentDeltaRadius`
/// (`libs/kimath/src/geometry/geometry_utils.cpp:63`): the radius is that
/// of the circle tangent to the middle of each segment, so the circle
/// through the segment **ends** is larger by this much.
fn circle_to_end_segment_delta_radius(radius: i32, segments: i32) -> i32 {
  let segments = f64::from(segments.max(3));
  let alpha = std::f64::consts::PI / segments;

  kiround((f64::from(radius) * (1.0 - 1.0 / alpha.cos())).abs())
}

/// Rotate a point about the origin, KiCad's way.
///
/// Port of `RotatePoint( int*, int*, const EDA_ANGLE& )`
/// (`libs/kimath/src/trigo.cpp:225`) together with `EDA_ANGLE::Sin` and
/// `Cos` (`libs/kimath/include/geometry/eda_angle.h:178`, `:197`). The
/// exact quarter turns and the exact eighth turns are spelled out rather
/// than left to the library, because that is where KiCad's answer is
/// exact and a `sin` call's would only be nearly so; the polygon of a
/// sixteen segment corner passes through 45 and 315 degrees, so the
/// difference is reachable.
fn rotate_about_origin(point: Vec2, degrees: f64) -> Vec2 {
  let angle = degrees.rem_euclid(360.0);

  if angle == 0.0 {
    return point;
  }

  if angle == 90.0 {
    return Vec2::new(point.y, -point.x);
  }

  if angle == 180.0 {
    return Vec2::new(-point.x, -point.y);
  }

  if angle == 270.0 {
    return Vec2::new(-point.y, point.x);
  }

  let sine = if angle == 45.0 || angle == 135.0 {
    FRAC_1_SQRT_2
  } else if angle == 225.0 || angle == 315.0 {
    -FRAC_1_SQRT_2
  } else {
    angle.to_radians().sin()
  };
  let cosine = if angle == 45.0 || angle == 315.0 {
    FRAC_1_SQRT_2
  } else if angle == 135.0 || angle == 225.0 {
    -FRAC_1_SQRT_2
  } else {
    angle.to_radians().cos()
  };

  Vec2::new(
    kiround(f64::from(point.y) * sine + f64::from(point.x) * cosine),
    kiround(f64::from(point.y) * cosine - f64::from(point.x) * sine),
  )
}

/// The copper of a custom pad, as the convex hull of everything it is
/// made of.
///
/// KiCad unions the anchor shape with the primitives and keeps outline 0
/// of the result (`pcbnew/router/pns_kicad_iface.cpp:1733`). A convex
/// hull contains that union, so the obstacle is never smaller than
/// KiCad's, and unlike the union it satisfies [`SimplePolygon`]'s
/// convexity assumption. Every custom pad in the corpus uses
/// `(anchor rect)`, so the anchor is the pad's own size rectangle.
fn custom_shape(pad: &KicadPad, primitives: &[Vec<Point>]) -> Option<Shape> {
  let mut points: Vec<Vec2> = match rectangle_shape(pad) {
    Shape::Rect { origin, size, .. } => vec![
      origin,
      Vec2::new(origin.x + size.x, origin.y),
      Vec2::new(origin.x + size.x, origin.y + size.y),
      Vec2::new(origin.x, origin.y + size.y),
    ],
    // A rotated anchor comes back as a polygon; take its corners.
    Shape::Simple(polygon) => (0..polygon.point_count())
      .map(|at| polygon.point(at))
      .collect(),
    _ => Vec::new(),
  };

  for primitive in primitives {
    for point in primitive {
      points.push(to_vec2(*point));
    }
  }

  let hull = monotone_chain_hull(&points);

  (hull.point_count() >= 3).then(|| Shape::simple(hull))
}

/// Add one `Edge.Cuts` graphic as zero width, non routable solids on
/// every copper layer.
///
/// `syncGraphicalItem` (`pcbnew/router/pns_kicad_iface.cpp:2047`) makes
/// one solid per effective shape, spans the whole stack and clears the
/// routable flag for `Edge_Cuts` (`:2061`), forces the stroke width to
/// zero for `Edge_Cuts` only (`:2070`), and marks each piece a compound
/// primitive when there is more than one (`:2086`). The position is left
/// at the origin because KiCad never calls `SetPos` here; a non routable
/// solid gets no joint, so nothing reads it.
fn add_outline(
  snapshot: &mut WorldSnapshot,
  map: &mut HostMap,
  graphic: &KicadGraphic,
  whole_stack: LayerRange,
) {
  let (uuid, points, closed) = match graphic {
    KicadGraphic::Line {
      start, end, uuid, ..
    } => (uuid, vec![*start, *end], false),
    KicadGraphic::Rectangle {
      start, end, uuid, ..
    } => (
      uuid,
      vec![
        *start,
        Point {
          x: end.x,
          y: start.y,
        },
        *end,
        Point {
          x: start.x,
          y: end.y,
        },
      ],
      true,
    ),
    KicadGraphic::Polygon { points, uuid, .. } => (uuid, points.clone(), true),
    KicadGraphic::Arc {
      start,
      mid,
      end,
      uuid,
      ..
    } => (uuid, flatten_arc(*start, *mid, *end), false),
  };

  let mut spine: Vec<Vec2> = points.into_iter().map(to_vec2).collect();

  if closed && spine.len() >= 3 {
    spine.push(spine[0]);
  }

  let pieces: Vec<Seg> = spine
    .windows(2)
    .filter(|pair| pair[0] != pair[1])
    .map(|pair| Seg::new(pair[0], pair[1]))
    .collect();

  if pieces.is_empty() {
    return;
  }

  let host = map.allocate(uuid, HostKind::BoardOutline);
  let compound = pieces.len() > 1;

  for piece in pieces {
    let mut item = WorldItem::new(
      host,
      None,
      whole_stack,
      WorldGeometry::Solid {
        shape: Shape::segment(piece, 0),
        pos: Vec2::new(0, 0),
        offset: Vec2::new(0, 0),
        orientation_degrees: 0.0,
        anchors: Vec::new(),
      },
    );

    item.flags = WorldItemFlags {
      routable: false,
      compound_primitive: compound,
      ..WorldItemFlags::default()
    };
    snapshot.items.push(item);
  }
}

/// Flatten a three point arc into inscribed chords.
///
/// The chord count is chosen so that the sagitta stays under
/// [`ARC_SAGITTA_NANOMETRES`], which keeps the approximated outline
/// inside the true curve and therefore pushes the router away from the
/// board edge rather than towards it. The arithmetic is `f64`, which the
/// reader already uses for pad rotations; no board in the corpus has an
/// arc on `Edge.Cuts`, so this runs only for boards outside it.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
fn flatten_arc(start: Point, mid: Point, end: Point) -> Vec<Point> {
  let (ax, ay) = (start.x as f64, start.y as f64);
  let (bx, by) = (mid.x as f64, mid.y as f64);
  let (cx, cy) = (end.x as f64, end.y as f64);
  let determinant = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));

  if determinant.abs() < 1.0 {
    // Three collinear points describe no circle; the two chords are the
    // best answer available.
    return vec![start, mid, end];
  }

  let square = |x: f64, y: f64| x * x + y * y;
  let center_x = (square(ax, ay) * (by - cy)
    + square(bx, by) * (cy - ay)
    + square(cx, cy) * (ay - by))
    / determinant;
  let center_y = (square(ax, ay) * (cx - bx)
    + square(bx, by) * (ax - cx)
    + square(cx, cy) * (bx - ax))
    / determinant;
  let radius = square(ax - center_x, ay - center_y).sqrt();
  let angle_of = |x: f64, y: f64| (y - center_y).atan2(x - center_x);
  let start_angle = angle_of(ax, ay);
  let mid_angle = angle_of(bx, by);
  let end_angle = angle_of(cx, cy);
  let normalise = |angle: f64| {
    let turn = std::f64::consts::TAU;

    ((angle - start_angle) % turn + turn) % turn
  };
  let mid_offset = normalise(mid_angle);
  let mut end_offset = normalise(end_angle);

  // The arc runs the way round that passes through the middle point.
  if mid_offset > end_offset {
    end_offset -= std::f64::consts::TAU;
  }

  let sagitta = ARC_SAGITTA_NANOMETRES as f64;
  let step = if radius <= sagitta {
    std::f64::consts::PI
  } else {
    2.0 * (1.0 - sagitta / radius).clamp(-1.0, 1.0).acos()
  };
  let count = ((end_offset.abs() / step).ceil() as i64).clamp(1, 1024);
  let mut points = Vec::with_capacity(count as usize + 1);

  for index in 0..=count {
    let angle = start_angle + end_offset * (index as f64) / (count as f64);

    points.push(Point {
      x: (center_x + radius * angle.cos()).round() as i64,
      y: (center_y + radius * angle.sin()).round() as i64,
    });
  }

  points
}

#[cfg(test)]
mod tests {
  use std::path::{Path, PathBuf};

  use pnsrouter::item::{Segment, Via};

  use super::super::kicad_dru;
  use super::super::kicad_pcb::read_board;
  use super::*;

  /// A file of the one corpus case that ships design rules.
  fn case_file(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
      .join("tests/fixtures/kicad/pns_regressions")
      .join(name)
  }

  /// Read that case's board, project and design rules into a resolver.
  fn rules_of_the_design_rules_case() -> KicadRules {
    let board_text =
      std::fs::read_to_string(case_file("boards/shove_same_net_via.kicad_pcb"))
        .expect("the corpus board is readable");
    let board = read_board("shove_same_net_via.kicad_pcb", &board_text)
      .expect("it parses");
    let project = std::fs::read_to_string(case_file(
      "issue24132-shove-same-net-via/pns.kicad_pro",
    ))
    .expect("the project file is readable");
    let mut rules =
      KicadRules::from_project(Some(&project), &board).expect("it parses");
    let design_rules = std::fs::read_to_string(case_file(
      "issue24132-shove-same-net-via/pns.kicad_dru",
    ))
    .expect("the design rules are readable");

    rules.set_design_rules(
      kicad_dru::parse(&design_rules).expect("the design rules parse"),
    );
    rules
  }

  /// A track on `net`.
  fn track(uid: u64, net: NetId) -> Item {
    let mut item = Item::new(
      uid,
      ItemBody::Segment(Segment::new(
        Seg::new(Vec2::new(0, 0), Vec2::new(1_000_000, 0)),
        200_000,
      )),
    );

    item.set_net(Some(net));
    item.set_layers_and_flash_all(LayerRange::single(0));
    item
  }

  /// A via on `net`.
  fn via(uid: u64, net: NetId) -> Item {
    let mut item = Item::new(
      uid,
      ItemBody::Via(Via::new(
        Vec2::new(0, 0),
        600_000,
        300_000,
        ViaType::Through,
      )),
    );

    item.set_net(Some(net));
    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item
  }

  /// The corpus's `.kicad_dru` reaches the clearance ladder, and reaches
  /// it net blind.
  ///
  /// Its one rule gives a track and a via 2 mm, and because a physical
  /// constraint sits outside the `!sameNet` guard
  /// (`pcbnew/router/pns_kicad_iface.cpp:960`) the pair keeps that value
  /// even on one net, which is what stops the same net short circuit at
  /// `:968` from turning the pair off. A pair the condition does not name
  /// is untouched and keeps the net class clearance.
  #[test]
  fn the_physical_clearance_rule_applies_to_a_same_net_track_and_via() {
    let rules = rules_of_the_design_rules_case();
    let net = NetId(1);
    let track_item = track(0, net);
    let via_item = via(1, net);
    let other_track = track(2, net);
    let foreign_track = track(3, NetId(2));

    assert!(rules.has_user_defined_physical_constraint());
    // The broad phase has to reach as far as the rule can push.
    assert_eq!(rules.worst_clearance(), 2_000_000);

    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&track_item),
        Some(ItemRef::unstored(&via_item)),
        false
      ),
      Some(2_000_000)
    );
    // And in the order the rule does not spell out.
    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&via_item),
        Some(ItemRef::unstored(&track_item)),
        false
      ),
      Some(2_000_000)
    );
    // Two tracks on one net match no rule, so `:968` still fires.
    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&track_item),
        Some(ItemRef::unstored(&other_track)),
        false
      ),
      None
    );
    // Two tracks on different nets keep the net class clearance, 0.2 mm
    // in this project.
    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&track_item),
        Some(ItemRef::unstored(&foreign_track)),
        false
      ),
      Some(200_000)
    );
    // A one sided query cannot satisfy the rule's `B` term.
    assert_eq!(
      rules.clearance(ItemRef::unstored(&track_item), None, false),
      Some(200_000)
    );
  }

  /// A project with no `.kicad_dru` answers exactly as it did before one
  /// could be read.
  #[test]
  fn a_case_without_design_rules_keeps_the_net_class_ladder() {
    let mut rules = rules_of_the_design_rules_case();

    rules.set_design_rules(DesignRules::default());

    let net = NetId(1);
    let track_item = track(0, net);
    let via_item = via(1, net);

    assert!(!rules.has_user_defined_physical_constraint());
    assert!(!rules.has_unsupported_design_rules);
    assert_eq!(rules.worst_clearance(), 500_000);
    assert_eq!(
      rules.clearance(
        ItemRef::unstored(&track_item),
        Some(ItemRef::unstored(&via_item)),
        false
      ),
      None
    );
  }

  /// A pattern that is a plain name matches only that name.
  #[test]
  fn a_literal_pattern_matches_one_name() {
    assert!(matches_pattern("GND", "GND"));
    assert!(!matches_pattern("GND", "GNDA"));
    assert!(!matches_pattern("GND", "AGND"));
  }

  /// The two glob characters behave.
  #[test]
  fn a_glob_pattern_matches_a_family_of_names() {
    assert!(matches_pattern("*", "anything"));
    assert!(matches_pattern("D?_P", "D0_P"));
    assert!(matches_pattern("/bus/*", "/bus/clk"));
    assert!(!matches_pattern("/bus/*", "/other/clk"));
    assert!(matches_pattern("*_P", "diff_P"));
    assert!(!matches_pattern("*_P", "diff_N"));
  }

  /// A rectangle keeps its rectangle shape through every quarter turn
  /// and becomes a polygon otherwise.
  #[test]
  fn a_quarter_turned_rectangle_is_still_a_rectangle() {
    let mut pad = KicadPad {
      footprint_uuid: String::new(),
      footprint_reference: String::new(),
      number: "1".to_string(),
      kind: PadKind::SurfaceMount,
      shape: PadShape::Rectangle,
      at: Point { x: 0, y: 0 },
      size: Point {
        x: 2_000_000,
        y: 1_000_000,
      },
      rotation_degrees: 0.0,
      layer_names: vec!["F.Cu".to_string()],
      copper_layers: vec![0],
      net: None,
      drill: None,
      uuid: "pad".to_string(),
    };

    for turn in [0.0, 90.0, 180.0, 270.0] {
      pad.rotation_degrees = turn;

      let Shape::Rect { size, .. } = rectangle_shape(&pad) else {
        panic!("a {turn} degree rectangle came out as a polygon");
      };
      let long_axis_is_x = turn == 0.0 || turn == 180.0;

      assert_eq!(size.x > size.y, long_axis_is_x, "at {turn} degrees");
    }

    pad.rotation_degrees = 30.0;
    assert!(matches!(rectangle_shape(&pad), Shape::Simple(_)));
  }

  /// A flattened arc stays on its circle and starts and ends on the
  /// points it was given.
  #[test]
  fn a_flattened_arc_keeps_its_endpoints_and_its_radius() {
    let radius = 10_000_000_i64;
    let start = Point { x: radius, y: 0 };
    let mid = Point {
      x: 7_071_068,
      y: 7_071_068,
    };
    let end = Point { x: 0, y: radius };
    let points = flatten_arc(start, mid, end);

    assert!(points.len() > 2, "the arc was not subdivided");
    assert_eq!(points.first().copied(), Some(start));

    let last = points.last().copied().expect("the arc has points");

    assert!((last.x - end.x).abs() <= 2 && (last.y - end.y).abs() <= 2);

    for point in &points {
      let distance = ((point.x * point.x + point.y * point.y) as f64).sqrt();

      assert!(
        (distance - radius as f64).abs() < 100.0,
        "{point:?} is not on the circle"
      );
    }
  }

  /// A pad the way `drag-acute-fallback` walks around one.
  ///
  /// Pad 2 of `C1` on `boards/drag-walk-optimize.kicad_pcb`: a
  /// `0.9 x 0.95` roundrect with `roundrect_rratio 0.25`, centred at
  /// `(146.225, 105.5)`.
  fn the_acute_fallback_pad() -> KicadPad {
    KicadPad {
      footprint_uuid: String::new(),
      footprint_reference: "C1".to_string(),
      number: "2".to_string(),
      kind: PadKind::SurfaceMount,
      shape: PadShape::RoundedRectangle { ratio: 0.25 },
      at: Point {
        x: 146_225_000,
        y: 105_500_000,
      },
      size: Point {
        x: 900_000,
        y: 950_000,
      },
      rotation_degrees: 0.0,
      layer_names: vec!["F.Cu".to_string()],
      copper_layers: vec![0],
      net: Some(1),
      drill: None,
      uuid: String::new(),
    }
  }

  /// The roundrect polygon has KiCad's five vertices per corner.
  ///
  /// `CornerListToPolygon`'s `ERROR_OUTSIDE` branch
  /// (`libs/kimath/src/convert_basic_shapes_to_polygon.cpp:317`) emits,
  /// per corner, the chord's crossing of the straight side, the three arc
  /// vertices at 22.5, 45 and 67.5 degrees, and the mirror of the
  /// crossing on the other side. Sixteen segments per full turn is the
  /// floor here, because the error bound alone asks for fifteen.
  #[test]
  fn a_roundrect_pad_is_polygonised_the_way_kicad_polygonises_it() {
    let pad = the_acute_fallback_pad();
    let outline = rounded_rectangle_outline(&pad, 225_000);

    assert_eq!(
      arc_to_segment_count(225_000, MAX_ERROR_NANOMETRES, 360.0),
      15
    );
    assert_eq!(circle_to_end_segment_delta_radius(225_000, 16), 4_408);
    assert_eq!(outline.len(), 20);

    // Every vertex is inside the sharp rectangle grown by the outward
    // correction, and none is inside the rectangle shrunk by the radius.
    for point in &outline {
      assert!((point.x - 146_225_000).abs() <= 450_000 + 4_408);
      assert!((point.y - 105_500_000).abs() <= 475_000 + 4_408);
    }
  }

  /// The vertex `PNS::ConvexHull` measures its diagonals against.
  ///
  /// `MoveDiagonal` (`pcbnew/router/pns_utils.cpp:289`) slides each 45
  /// degree line until the **nearest** outline vertex sits at exactly the
  /// clearance, and for a rounded corner that is the arc vertex at 45
  /// degrees, `radius + radiusExtend` from the corner's centre. It is
  /// what cuts `drag-acute-fallback`'s detour column at 45 degrees where
  /// a `Shape::Rect` would leave it square.
  #[test]
  fn the_roundrect_corner_vertex_the_hull_diagonal_touches() {
    let pad = the_acute_fallback_pad();
    let outline = rounded_rectangle_outline(&pad, 225_000);
    let centre = Vec2::new(146_225_000 + 225_000, 105_500_000 + 250_000);
    let corner = outline
      .iter()
      .copied()
      .max_by_key(|point| i64::from(point.x) + i64::from(point.y))
      .expect("the outline has vertices");
    let offset = corner - centre;

    // 229_408 is 225_000 + the 4_408 the outward correction adds, taken
    // at 45 degrees.
    assert_eq!(offset, Vec2::new(162_216, 162_216));
    assert_eq!(corner, Vec2::new(146_612_216, 105_912_216));
  }

  /// A roundrect whose corners eat the whole pad is a circle, and a
  /// roundrect with no radius is a rectangle.
  ///
  /// `pcbnew/pad.cpp:1461` to `:1470` for the circle, whose single
  /// subshape then keeps `syncPad` out of the polygon branch, and
  /// `pcbnew/padstack.cpp:124` for the rectangle.
  #[test]
  fn the_two_roundrects_that_are_not_polygons() {
    let mut pad = the_acute_fallback_pad();

    pad.shape = PadShape::RoundedRectangle { ratio: 0.0 };
    pad.size = Point {
      x: 900_000,
      y: 900_000,
    };

    assert!(matches!(pad_shape(&pad), Some(Shape::Rect { .. })));

    pad.shape = PadShape::RoundedRectangle { ratio: 0.5 };

    assert_eq!(
      pad_shape(&pad),
      Some(Shape::circle(Vec2::new(146_225_000, 105_500_000), 450_000))
    );
  }
}
