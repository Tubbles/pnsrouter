// SPDX-License-Identifier: GPL-3.0-or-later

//! The two settings structs every routing algorithm reads.
//!
//! [`RoutingSettings`] is the port of `PNS::ROUTING_SETTINGS`
//! (`pcbnew/router/pns_routing_settings.h:58`) and [`Sizes`] the port of
//! `PNS::SIZES_SETTINGS` (`pcbnew/router/pns_sizes_settings.h:40`). The
//! first is persistent user configuration, the second is per route
//! geometry the host resolves from the board's design rules.
//!
//! # What is left out, and why
//!
//! KiCad's `ROUTING_SETTINGS` derives from `NESTED_SETTINGS` and loads
//! itself from a JSON file in its constructor
//! (`pcbnew/router/pns_routing_settings.cpp:108`). None of that is here:
//! serialisation is the host's business. The JSON key of every field is
//! quoted in its doc comment so a host can round trip KiCad's file
//! without a second table.
//!
//! Four members of `ROUTING_SETTINGS` are deliberately absent.
//!
//! - `m_snapToTracks` and `m_snapToPads` (`:172`, `:173`) are host only.
//!   Cursor snapping happens in `TOOL_BASE` before the router is called
//!   and the router core never reads either flag; the host even
//!   overwrites both from its magnetic settings on every `checkSnap`
//!   (`pcbnew/router/pns_tool_base.cpp:314` to `:325`). Note 03 section
//!   9.4 says not to put them in the core struct.
//! - `m_suggestFinish` (`:169`) has no consumer anywhere in KiCad's tree.
//! - `m_walkaroundTimeLimit` (`:191`) has no consumer and the constructor
//!   never even assigns it, so it is a default constructed `TIME_LIMIT`.
//!
//! The last two are the dead settings note 03 section 9.5 lists.
//!
//! `SIZES_SETTINGS` loses its four `wxString` provenance fields
//! (`m_clearanceSource`, `m_widthSource`, `m_diffPairWidthSource`,
//! `m_diffPairGapSource`, `pcbnew/router/pns_sizes_settings.h:177` to
//! `:180`). They are strings the host puts in a status bar; no algorithm
//! reads them.
//!
//! There is no port of `SIZES_SETTINGS::Init`, because this revision of
//! KiCad has no such method: the sizes are filled in by
//! `ROUTER_IFACE::ImportSizes` (`pcbnew/router/pns_router.h:112`), a host
//! callback that walks the board's design rules, the net class and the
//! start item. That is host work by definition, so [`Sizes`] is a plain
//! value the host builds.

use std::collections::BTreeMap;

use crate::geometry::direction45::{CornerMode, Direction45, Octant};
use crate::item::{LayerRange, ViaType};

// ---------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------

/// Which of the three head routines the line placer runs.
///
/// Port of `PNS_MODE`, `pcbnew/router/pns_routing_settings.h:39`. The
/// discriminants are KiCad's, because they are what the `mode` key of the
/// settings file holds.
///
/// The variant order below is the numeric order and **not** the order the
/// modes appear in KiCad's user interface.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum RouterMode {
  /// Route straight through and paint what collides.
  /// `RM_MarkObstacles = 0`.
  MarkObstacles = 0,
  /// Push colliding tracks and vias aside. `RM_Shove = 1`.
  Shove = 1,
  /// Bend the head around whatever is in the way. `RM_Walkaround = 2`.
  ///
  /// KiCad's default (`pcbnew/router/pns_routing_settings.cpp:35`).
  #[default]
  Walkaround = 2,
}

/// How hard the optimizer works on a finished head.
///
/// Port of `PNS_OPTIMIZATION_EFFORT`,
/// `pcbnew/router/pns_routing_settings.h:47`.
///
/// [`OptimizerEffort::Medium`] and [`OptimizerEffort::Full`] are the same
/// thing in this revision: the line placer treats them identically
/// (`pcbnew/router/pns_line_placer.cpp:753`, `:973`) and even
/// `SHOVE::runOptimizer`, the only place that branches on them at all,
/// ends up with the same pass count and the same flag set
/// (`pcbnew/router/pns_shove.cpp:2050` to `:2058`). Both variants are
/// kept so that a host's stored setting survives the round trip.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum OptimizerEffort {
  /// Merge obtuse corners only, one pass. `OE_LOW = 0`.
  Low = 0,
  /// Merge segments, two passes. `OE_MEDIUM = 1`, KiCad's default
  /// (`pcbnew/router/pns_routing_settings.cpp:36`).
  #[default]
  Medium = 1,
  /// The same as [`OptimizerEffort::Medium`] in this revision.
  /// `OE_FULL = 2`.
  Full = 2,
}

// ---------------------------------------------------------------------
// RoutingSettings
// ---------------------------------------------------------------------

/// The persistent settings of the router.
///
/// Port of `PNS::ROUTING_SETTINGS`,
/// `pcbnew/router/pns_routing_settings.h:58`. [`RoutingSettings::default`]
/// reproduces the constructor at
/// `pcbnew/router/pns_routing_settings.cpp:32` to `:58` field for field.
///
/// The fields are public because this is configuration and not an
/// invariant carrying type. The two accessors KiCad computes rather than
/// stores, `FollowMouse()` and `AllowDRCViolations()`, are methods here
/// for the same reason they are methods there: both fold
/// [`RoutingSettings::mode`] into the answer, so reading the raw field
/// instead would be a bug.
///
/// A caller never reaches this through a router singleton. It is passed
/// by reference into every algorithm that needs it, which is what
/// `DESIGN.md` section 8 asks for and what note 03 section 9.4 recommends
/// in place of `ROUTER::GetInstance()`.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct RoutingSettings {
  /// Which head routine runs. JSON key `mode`, default
  /// [`RouterMode::Walkaround`]
  /// (`pcbnew/router/pns_routing_settings.cpp:35`).
  ///
  /// Read by `LINE_PLACER::routeHead`
  /// (`pcbnew/router/pns_line_placer.cpp:1022`), by the collision node
  /// choice in `FixRoute` (`:1589`) and `Start` (`:1431`), by
  /// `UnfixRoute` (`:1792`), by `CommitPlacement` (`:1812`), by
  /// `buildInitialLine` (`:2075`) and by both computed accessors below.
  pub mode: RouterMode,
  /// How hard the optimizer works. JSON key `effort`, default
  /// [`OptimizerEffort::Medium`] (`:36`).
  ///
  /// Read by `rhWalkOnly` (`pcbnew/router/pns_line_placer.cpp:747`),
  /// `rhShoveOnly` (`:967`) and `SHOVE::runOptimizer`
  /// (`pcbnew/router/pns_shove.cpp:2028`).
  pub optimizer_effort: OptimizerEffort,
  /// Whether the shove may move vias. JSON key `shove_vias`, default true
  /// (`:39`).
  ///
  /// Read by `SHOVE::onCollidingVia` (`pcbnew/router/pns_shove.cpp:1060`)
  /// and nowhere else.
  pub shove_vias: bool,
  /// Whether a re route deletes the loop it made redundant. JSON key
  /// `remove_loops`, default true (`:37`).
  ///
  /// Read by `LINE_PLACER::Move`
  /// (`pcbnew/router/pns_line_placer.cpp:1545`).
  pub remove_loops: bool,
  /// Whether the optimizer straightens the last hop into a pad. JSON key
  /// `smart_pads`, default true (`:38`).
  ///
  /// Read by `rhWalkOnly` (`pcbnew/router/pns_line_placer.cpp:762`),
  /// `rhShoveOnly` (`:982`) and `SHOVE::runOptimizer`
  /// (`pcbnew/router/pns_shove.cpp:2079`).
  pub smart_pads: bool,
  /// Whether the head follows the cursor rather than stopping short.
  /// JSON key `follow_mouse`, default true (`:41`).
  ///
  /// Read only through [`RoutingSettings::follow_mouse`], which is what
  /// `reduceTail` and the merge stage of `routeStep` consult
  /// (`pcbnew/router/pns_line_placer.cpp:1134`, `:1198`).
  pub follow_mouse: bool,
  /// Whether the first segment of a new trace is diagonal. JSON key
  /// `start_diagonal`, default false (`:42`).
  ///
  /// Read through [`RoutingSettings::initial_direction`] by
  /// `LINE_PLACER::Start` (`pcbnew/router/pns_line_placer.cpp:1393`).
  pub start_diagonal: bool,
  /// Whether the shove may hop a track over an unmovable obstacle. JSON
  /// key `jump_over_obstacles`, default false (`:46`).
  ///
  /// Read by `SHOVE::onCollidingSolid`
  /// (`pcbnew/router/pns_shove.cpp:829`) and nowhere else.
  pub jump_over_obstacles: bool,
  /// Whether a dragged segment is smoothed as it moves. JSON key
  /// `smooth_dragged_segments`, default true (`:47`).
  ///
  /// Read by the two draggers only (`pcbnew/router/pns_dragger.cpp:398`,
  /// `:580`, `:727`, `:818`, `pcbnew/router/pns_multi_dragger.cpp:755`),
  /// which arrive with a later milestone.
  pub smooth_dragged_segments: bool,
  /// Whether the user may commit a route that breaks a rule. JSON key
  /// `can_violate_drc`, default false (`:48`).
  ///
  /// Read only through [`RoutingSettings::allow_drc_violations`], which
  /// folds in the mode.
  pub allow_drc_violations: bool,
  /// Whether the head ignores the 45 degree grid. JSON key
  /// `free_angle_mode`, default false (`:49`).
  ///
  /// Read by `buildInitialLine`
  /// (`pcbnew/router/pns_line_placer.cpp:2075`), and only together with
  /// [`RouterMode::MarkObstacles`].
  pub free_angle_mode: bool,
  /// Whether the whole dragged track is optimized or only the changed
  /// part. JSON key `optimize_dragged_track`, default false (`:52`).
  ///
  /// Read by `DRAGGER::optimizeAndUpdateDraggedLine`
  /// (`pcbnew/router/pns_dragger.cpp:598`) and nowhere else.
  pub optimize_entire_dragged_track: bool,
  /// Whether the posture solver follows the mouse trail. JSON key
  /// `auto_posture`, default true (`:55`).
  ///
  /// Inverted into `MOUSE_TRAIL_TRACER::SetMouseDisabled` by
  /// `LINE_PLACER::initPlacement`
  /// (`pcbnew/router/pns_line_placer.cpp:1427`).
  pub auto_posture: bool,
  /// Whether a click fixes every segment of the head or only the first.
  /// JSON key `fix_all_segments`, default true (`:56`).
  ///
  /// Read by `LINE_PLACER::FixRoute`
  /// (`pcbnew/router/pns_line_placer.cpp:1557`) and by the diff pair
  /// placer (`pcbnew/router/pns_diff_pair_placer.cpp:822`).
  pub fix_all_segments: bool,
  /// Whether the dragger refuses to create non 45 degree corners. JSON
  /// key `restrict_angles`, default false (`:57`).
  ///
  /// Read by `DRAGGER::optimizeAndUpdateDraggedLine`
  /// (`pcbnew/router/pns_dragger.cpp:583`) and by the host's own optimize
  /// action (`pcbnew/router/router_tool.cpp:2339`).
  pub restrict_angles: bool,
  /// Which corners the initial trace and the hulls are built with. JSON
  /// key `corner_mode`, default [`CornerMode::Mitered45`] (`:53`).
  ///
  /// The most widely read setting of the lot: `buildInitialLine`
  /// (`pcbnew/router/pns_line_placer.cpp:2046`), the smart pad gate of
  /// `rhWalkOnly` (`:759`), the hull snapping of `rhMarkObstacles`
  /// (`:825`), `rhShoveOnly` (`:979`), the hull squaring in
  /// `WALKAROUND::singleStep` (`pcbnew/router/pns_walkaround.cpp:129`)
  /// and in `NODE::NearestObstacle` (`pcbnew/router/pns_node.cpp:301`),
  /// the optimizer and the shove.
  ///
  /// KiCad has four modes and this crate has two, because the rounded
  /// ones need an arc type; see [`CornerMode`].
  pub corner_mode: CornerMode,
  /// How many times the walkaround may bend a head before giving up.
  /// JSON key `walkaround_iteration_limit`, default 40 (`:45`).
  ///
  /// Read at every `WALKAROUND` construction site and by the
  /// `WALKAROUND` constructor itself
  /// (`pcbnew/router/pns_walkaround.h:56`); see
  /// [`crate::walkaround::Walkaround::set_iteration_limit`].
  pub walkaround_iteration_limit: u32,
  /// How many times the shove may push before giving up. JSON key
  /// `shove_iteration_limit`, default 250 (`:43`).
  ///
  /// Read by `SHOVE::shoveMainLoop`
  /// (`pcbnew/router/pns_shove.cpp:1890`).
  pub shove_iteration_limit: u32,
  /// KiCad's wall clock budget for one shove, in milliseconds. JSON key
  /// `shove_time_limit`, default 1000 (`:44`).
  ///
  /// **Nothing in this crate reads it.** `DESIGN.md` section 8 forbids
  /// reading a clock inside an algorithm, so the shove's only budget is
  /// [`RoutingSettings::shove_iteration_limit`], which is the same
  /// termination guarantee without the non determinism: KiCad's
  /// `shoveMainLoop` tests both limits in the same condition
  /// (`pcbnew/router/pns_shove.cpp:1890` to `:1891`), so dropping the
  /// clock can only make a run take more iterations, never fewer, and the
  /// iteration limit still stops it.
  ///
  /// The field is kept, rather than dropped like the two dead settings,
  /// so that a host that stores KiCad's settings file can round trip it.
  pub shove_time_limit_ms: u32,
  /// How many times a via pushout may propagate. JSON key
  /// `via_force_prop_iteration_limit`, default 40 (`:58`).
  ///
  /// Read by `buildInitialLine`
  /// (`pcbnew/router/pns_line_placer.cpp:2115`) and by the dragger
  /// (`pcbnew/router/pns_dragger.cpp:69`).
  pub via_force_prop_iteration_limit: u32,
  /// How much longer than the straight path a hugging walkaround may be.
  /// JSON key `walkaround_hug_length_threshold`, default 1.5 (`:54`).
  ///
  /// Read by `rhWalkBase` (`pcbnew/router/pns_line_placer.cpp:590`,
  /// `:592`), which also doubles it for the complete walkaround case.
  /// KiCad stores a `double` here, so this is not a place the crate's "no
  /// floating point where KiCad uses integers" rule applies.
  pub walkaround_hug_length_threshold: f64,
}

impl Default for RoutingSettings {
  /// KiCad's constructor, `pcbnew/router/pns_routing_settings.cpp:32` to
  /// `:58`.
  ///
  /// The serialisation defaults at `:60` to `:107` agree with it field
  /// for field, so there is only one set of numbers to pin.
  fn default() -> Self {
    Self {
      mode: RouterMode::Walkaround,
      optimizer_effort: OptimizerEffort::Medium,
      shove_vias: true,
      remove_loops: true,
      smart_pads: true,
      follow_mouse: true,
      start_diagonal: false,
      jump_over_obstacles: false,
      smooth_dragged_segments: true,
      allow_drc_violations: false,
      free_angle_mode: false,
      optimize_entire_dragged_track: false,
      auto_posture: true,
      fix_all_segments: true,
      restrict_angles: false,
      corner_mode: CornerMode::Mitered45,
      walkaround_iteration_limit: 40,
      shove_iteration_limit: 250,
      shove_time_limit_ms: 1000,
      via_force_prop_iteration_limit: 40,
      walkaround_hug_length_threshold: 1.5,
    }
  }
}

impl RoutingSettings {
  /// Whether the head chases the cursor.
  ///
  /// Port of `FollowMouse()`,
  /// `pcbnew/router/pns_routing_settings.h:100`, which is
  /// `m_followMouse && Mode() != RM_MarkObstacles`. Mark obstacles mode
  /// draws the straight line whatever the flag says.
  pub const fn follow_mouse(&self) -> bool {
    self.follow_mouse && !matches!(self.mode, RouterMode::MarkObstacles)
  }

  /// Whether a colliding route may be committed.
  ///
  /// Port of `AllowDRCViolations()`,
  /// `pcbnew/router/pns_routing_settings.h:117`, which is
  /// `m_routingMode == RM_MarkObstacles && m_allowDRCViolations`. The
  /// flag is inert outside mark obstacles mode, which is why
  /// [`RoutingSettings::allow_drc_violations`] the field and this method
  /// are two different questions; KiCad keeps the raw field reachable as
  /// `GetAllowDRCViolationsSetting()` (`:121`) for its own preferences
  /// dialog.
  pub const fn allow_drc_violations(&self) -> bool {
    matches!(self.mode, RouterMode::MarkObstacles) && self.allow_drc_violations
  }

  /// The direction the first segment of a fresh trace leaves in.
  ///
  /// Port of `InitialDirection()`,
  /// `pcbnew/router/pns_routing_settings.cpp:112`: north east when
  /// [`RoutingSettings::start_diagonal`] is set, north otherwise.
  pub const fn initial_direction(&self) -> Direction45 {
    if self.start_diagonal {
      Direction45::from_octant(Octant::NE)
    } else {
      Direction45::from_octant(Octant::N)
    }
  }
}

// ---------------------------------------------------------------------
// Sizes
// ---------------------------------------------------------------------

/// The geometry one route is placed with.
///
/// Port of `PNS::SIZES_SETTINGS`,
/// `pcbnew/router/pns_sizes_settings.h:40`, minus the four provenance
/// strings; see the module documentation. [`Sizes::default`] reproduces
/// the constructor at `:43` to `:59`, whose numbers are in nanometres.
///
/// KiCad copies this by value into the placer (`LINE_PLACER::m_sizes`)
/// and this crate does the same, which is why the layer pair map is a
/// [`BTreeMap`] and not a shared handle.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Sizes {
  /// The working clearance from the routed net to anything, before per
  /// pair resolution. `Clearance()`, default 0 (`:44`).
  ///
  /// Read by the diff pair gate at `pcbnew/router/pns_router.cpp:229` and
  /// by the host when it builds the preview.
  pub clearance: i32,
  /// The board's absolute minimum clearance. `MinClearance()`, default 0
  /// (`:45`).
  ///
  /// Read by the diff pair gap gate
  /// (`pcbnew/router/pns_router.cpp:229`).
  pub min_clearance: i32,
  /// The track width. `TrackWidth()`, default 155000 (`:46`).
  ///
  /// Pushed into the head and the tail by `LINE_PLACER::initPlacement`
  /// (`pcbnew/router/pns_line_placer.cpp:1452`).
  pub track_width: i32,
  /// Whether the width came from an explicit choice rather than a
  /// default. `TrackWidthIsExplicit()`, default true (`:47`).
  ///
  /// Gates a mid route width change
  /// (`pcbnew/router/pns_line_placer.cpp:2004`).
  pub track_width_is_explicit: bool,
  /// The narrowest track the board allows. `BoardMinTrackWidth()`,
  /// default 0 (`:48`).
  ///
  /// The fallback probe width in `ROUTER::isStartingPointRoutable`
  /// (`pcbnew/router/pns_router.cpp:324`).
  pub board_min_track_width: i32,
  /// Which layers a placed via spans. `ViaType()`, default
  /// [`ViaType::Through`] (`:49`).
  ///
  /// A through via always spans the whole board; every other type takes
  /// the span from [`Sizes::layer_top`] and [`Sizes::layer_bottom`]. The
  /// decision itself lives in `ROUTER_IFACE::GetViaLayerRange`
  /// (`pcbnew/router/pns_router.h:144`), a host method, because only the
  /// host knows which dense layer index is the bottom of the board.
  pub via_type: ViaType,
  /// The copper diameter of a placed via. `ViaDiameter()`, default 600000
  /// (`:50`).
  ///
  /// Read by `LINE_PLACER::makeVia`
  /// (`pcbnew/router/pns_line_placer.cpp:80`).
  pub via_diameter: i32,
  /// The drill diameter of a placed via. `ViaDrill()`, default 250000
  /// (`:51`).
  ///
  /// Read by `LINE_PLACER::makeVia`
  /// (`pcbnew/router/pns_line_placer.cpp:80`).
  pub via_drill: i32,
  /// The width of one track of a differential pair. `DiffPairWidth()`,
  /// default 125000 (`:52`).
  pub diff_pair_width: i32,
  /// The copper gap between the two tracks of a pair. `DiffPairGap()`,
  /// default 180000 (`:53`).
  pub diff_pair_gap: i32,
  /// The copper gap between the two vias of a pair. Default 180000
  /// (`:54`).
  ///
  /// Read through [`Sizes::diff_pair_via_gap`], which returns
  /// [`Sizes::diff_pair_gap`] instead while
  /// [`Sizes::diff_pair_via_gap_same_as_trace_gap`] is set.
  pub diff_pair_via_gap: i32,
  /// Whether the via gap tracks the trace gap. Default true (`:55`).
  pub diff_pair_via_gap_same_as_trace_gap: bool,
  /// The hole to hole clearance. `GetHoleToHole()`, default 0 (`:56`).
  pub hole_to_hole: i32,
  /// The hole to hole clearance inside a pair. `GetDiffPairHoleToHole()`,
  /// default 0 (`:57`).
  pub diff_pair_hole_to_hole: i32,
  /// The copper to hole clearance inside a pair.
  /// `GetDiffPairCopperToHole()`, default 0 (`:58`).
  pub diff_pair_copper_to_hole: i32,
  /// The layer pairs a via may switch between.
  ///
  /// Port of `m_layerPairs` (`:175`), an ordered map from a dense copper
  /// layer index to its partner. [`Sizes::add_layer_pair`] writes both
  /// directions, so a lookup works either way round, and
  /// [`Sizes::layer_top`] depends on the map being ordered.
  ///
  /// It is a [`BTreeMap`] rather than a hash map because it is iterated,
  /// which `DESIGN.md` section 8 only allows for ordered containers, and
  /// because `std::map` is what KiCad uses.
  pub layer_pairs: BTreeMap<i32, i32>,
}

impl Default for Sizes {
  /// KiCad's constructor, `pcbnew/router/pns_sizes_settings.h:43` to
  /// `:59`, in nanometres.
  fn default() -> Self {
    Self {
      clearance: 0,
      min_clearance: 0,
      track_width: 155000,
      track_width_is_explicit: true,
      board_min_track_width: 0,
      via_type: ViaType::Through,
      via_diameter: 600000,
      via_drill: 250000,
      diff_pair_width: 125000,
      diff_pair_gap: 180000,
      diff_pair_via_gap: 180000,
      diff_pair_via_gap_same_as_trace_gap: true,
      hole_to_hole: 0,
      diff_pair_hole_to_hole: 0,
      diff_pair_copper_to_hole: 0,
      layer_pairs: BTreeMap::new(),
    }
  }
}

impl Sizes {
  /// Forget every layer pair.
  ///
  /// Port of `ClearLayerPairs`,
  /// `pcbnew/router/pns_sizes_settings.cpp:30`.
  pub fn clear_layer_pairs(&mut self) {
    self.layer_pairs.clear();
  }

  /// Record that a via may run between two layers.
  ///
  /// Port of `AddLayerPair`,
  /// `pcbnew/router/pns_sizes_settings.cpp:36`, which stores the pair
  /// under **both** keys so that [`Sizes::paired_layer`] answers from
  /// either end, and which normalises the pair so that the smaller index
  /// is the top.
  pub fn add_layer_pair(&mut self, first: i32, second: i32) {
    let top = first.min(second);
    let bottom = first.max(second);

    self.layer_pairs.insert(bottom, top);
    self.layer_pairs.insert(top, bottom);
  }

  /// The layer paired with one, if any.
  ///
  /// Port of `PairedLayer`,
  /// `pcbnew/router/pns_sizes_settings.h:109`, whose
  /// `std::optional<int>` is this [`Option`].
  pub fn paired_layer(&self, layer: i32) -> Option<i32> {
    self.layer_pairs.get(&layer).copied()
  }

  /// The upper layer of the first pair.
  ///
  /// Port of `GetLayerTop`,
  /// `pcbnew/router/pns_sizes_settings.cpp:46`, which returns
  /// `m_layerPairs.begin()->first`.
  ///
  /// # Deviation
  ///
  /// KiCad answers `F_Cu` for an empty map and [`Sizes::layer_bottom`]
  /// answers `B_Cu`, which are **board** layer identifiers returned from a
  /// pair of accessors whose other return path is a dense PNS layer index
  /// (`pcbnew/router/pns_router.h:150` feeds both straight into a
  /// `PNS_LAYER_RANGE`). The two numbering schemes agree at the top by
  /// luck and disagree at the bottom on any board that is not two layers.
  /// This crate has no notion of a board layer, so the empty case is
  /// [`None`] and the caller, which is the host's via layer range helper,
  /// substitutes the span it knows.
  pub fn layer_top(&self) -> Option<i32> {
    self.layer_pairs.first_key_value().map(|(top, _)| *top)
  }

  /// The lower layer of the first pair.
  ///
  /// Port of `GetLayerBottom`,
  /// `pcbnew/router/pns_sizes_settings.cpp:55`, which returns
  /// `m_layerPairs.begin()->second`. See [`Sizes::layer_top`] for why the
  /// empty case is [`None`] here and `B_Cu` in KiCad.
  pub fn layer_bottom(&self) -> Option<i32> {
    self
      .layer_pairs
      .first_key_value()
      .map(|(_, bottom)| *bottom)
  }

  /// The layer span a via placed with these sizes occupies.
  ///
  /// Port of `ROUTER_IFACE::GetViaLayerRange`
  /// (`pcbnew/router/pns_router.h:144`), the one non virtual helper on
  /// the host interface, which note 05 section 7 says belongs on the
  /// router side because it is pure logic over the sizes. Its two callers
  /// are `LINE_PLACER::makeVia`
  /// (`pcbnew/router/pns_line_placer.cpp:78`) and the differential pair
  /// placer's (`pcbnew/router/pns_diff_pair_placer.cpp:76`).
  ///
  /// # Deviation
  ///
  /// KiCad gives a [`ViaType::Through`] via the whole board,
  /// `F_Cu .. B_Cu`, and only lets the layer pair choose the span of a
  /// blind, buried or micro via. This crate has no notion of a board or
  /// of how many copper layers it has, which is the gap
  /// [`Sizes::layer_top`] already documents, so the layer pair decides
  /// the span for every via type here. A host that wants KiCad's through
  /// via rule registers the board's outermost pair.
  ///
  /// [`None`] when no layer pair was registered; the caller substitutes
  /// the span it knows, which for the line placer is the layer it is
  /// routing on.
  pub fn via_layer_range(&self) -> Option<LayerRange> {
    let top = self.layer_top()?;
    let bottom = self.layer_bottom()?;

    Some(LayerRange::new(top, bottom))
  }

  /// The copper gap between the two vias of a differential pair.
  ///
  /// Port of `DiffPairViaGap`,
  /// `pcbnew/router/pns_sizes_settings.h:87`, which returns the trace gap
  /// while [`Sizes::diff_pair_via_gap_same_as_trace_gap`] is set.
  pub const fn diff_pair_via_gap(&self) -> i32 {
    if self.diff_pair_via_gap_same_as_trace_gap {
      self.diff_pair_gap
    } else {
      self.diff_pair_via_gap
    }
  }

  /// The centre to centre spacing of the two lanes of a differential
  /// pair.
  ///
  /// Port of `DIFF_PAIR_PLACER::gap`,
  /// `pcbnew/router/pns_diff_pair_placer.cpp:616`, which is
  /// `DiffPairGap() + DiffPairWidth()`. It lives here rather than on the
  /// placer because it is pure arithmetic over the sizes, and it has a
  /// name of its own because KiCad calls this and
  /// [`Sizes::diff_pair_gap`] both "the gap": the first is the distance
  /// between the two centrelines, the second the distance between the
  /// two copper edges. Everything in [`crate::diff_pair`] that spaces
  /// anchors, and `checkGap`, want this one; the coupling measurements
  /// want the other. See `crate::diff_pair`'s module documentation.
  pub const fn diff_pair_pitch(&self) -> i32 {
    self.diff_pair_gap + self.diff_pair_width
  }

  /// The copper gap between two pair vias once the hole rules are folded
  /// in.
  ///
  /// Port of `EffectiveDiffPairViaGap`,
  /// `pcbnew/router/pns_sizes_settings.h:146`: the largest of the plain
  /// copper gap, the hole to hole rule minus two annular rings, and the
  /// copper to hole rule minus one. The annular ring is
  /// `(diameter - drill) / 2` with KiCad's truncating division.
  pub const fn effective_diff_pair_via_gap(&self) -> i32 {
    let annular_ring = (self.via_diameter - self.via_drill) / 2;
    let from_hole_to_hole = self.diff_pair_hole_to_hole - 2 * annular_ring;
    let from_copper_to_hole = self.diff_pair_copper_to_hole - annular_ring;
    let mut best = self.diff_pair_via_gap();

    if from_hole_to_hole > best {
      best = from_hole_to_hole;
    }

    if from_copper_to_hole > best {
      best = from_copper_to_hole;
    }

    best
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  /// Every default of `ROUTING_SETTINGS`, pinned against the constructor
  /// at `pcbnew/router/pns_routing_settings.cpp:35` to `:58`.
  #[test]
  fn routing_settings_defaults_are_kicads() {
    let settings = RoutingSettings::default();

    assert_eq!(settings.mode, RouterMode::Walkaround);
    assert_eq!(settings.optimizer_effort, OptimizerEffort::Medium);
    assert!(settings.shove_vias);
    assert!(settings.remove_loops);
    assert!(settings.smart_pads);
    assert!(settings.follow_mouse);
    assert!(!settings.start_diagonal);
    assert!(!settings.jump_over_obstacles);
    assert!(settings.smooth_dragged_segments);
    assert!(!settings.allow_drc_violations);
    assert!(!settings.free_angle_mode);
    assert!(!settings.optimize_entire_dragged_track);
    assert!(settings.auto_posture);
    assert!(settings.fix_all_segments);
    assert!(!settings.restrict_angles);
    assert_eq!(settings.corner_mode, CornerMode::Mitered45);
    assert_eq!(settings.walkaround_iteration_limit, 40);
    assert_eq!(settings.shove_iteration_limit, 250);
    assert_eq!(settings.shove_time_limit_ms, 1000);
    assert_eq!(settings.via_force_prop_iteration_limit, 40);
    assert!(
      (settings.walkaround_hug_length_threshold - 1.5).abs() < f64::EPSILON
    );
  }

  /// The numeric values of the two enums are the ones the settings file
  /// stores (`pcbnew/router/pns_routing_settings.h:39`, `:47`).
  #[test]
  fn the_enums_keep_kicads_discriminants() {
    assert_eq!(RouterMode::MarkObstacles as i32, 0);
    assert_eq!(RouterMode::Shove as i32, 1);
    assert_eq!(RouterMode::Walkaround as i32, 2);
    assert_eq!(OptimizerEffort::Low as i32, 0);
    assert_eq!(OptimizerEffort::Medium as i32, 1);
    assert_eq!(OptimizerEffort::Full as i32, 2);
  }

  /// `FollowMouse()` folds in the mode
  /// (`pcbnew/router/pns_routing_settings.h:100`).
  #[test]
  fn follow_mouse_is_off_in_mark_obstacles_mode() {
    let mut settings = RoutingSettings::default();

    assert!(settings.follow_mouse());

    settings.mode = RouterMode::MarkObstacles;
    assert!(!settings.follow_mouse());
    assert!(settings.follow_mouse);

    settings.mode = RouterMode::Shove;
    settings.follow_mouse = false;
    assert!(!settings.follow_mouse());
  }

  /// `AllowDRCViolations()` is inert outside mark obstacles mode
  /// (`pcbnew/router/pns_routing_settings.h:117`).
  #[test]
  fn drc_violations_need_mark_obstacles_mode() {
    let mut settings = RoutingSettings {
      allow_drc_violations: true,
      ..RoutingSettings::default()
    };

    assert!(!settings.allow_drc_violations());

    settings.mode = RouterMode::MarkObstacles;
    assert!(settings.allow_drc_violations());

    settings.allow_drc_violations = false;
    assert!(!settings.allow_drc_violations());
  }

  /// `InitialDirection()`, `pcbnew/router/pns_routing_settings.cpp:112`.
  #[test]
  fn the_initial_direction_follows_the_diagonal_flag() {
    let mut settings = RoutingSettings::default();

    assert_eq!(
      settings.initial_direction(),
      Direction45::from_octant(Octant::N)
    );

    settings.start_diagonal = true;
    assert_eq!(
      settings.initial_direction(),
      Direction45::from_octant(Octant::NE)
    );
  }

  /// Every default of `SIZES_SETTINGS`, pinned against the constructor at
  /// `pcbnew/router/pns_sizes_settings.h:43` to `:59`.
  #[test]
  fn sizes_defaults_are_kicads() {
    let sizes = Sizes::default();

    assert_eq!(sizes.clearance, 0);
    assert_eq!(sizes.min_clearance, 0);
    assert_eq!(sizes.track_width, 155000);
    assert!(sizes.track_width_is_explicit);
    assert_eq!(sizes.board_min_track_width, 0);
    assert_eq!(sizes.via_type, ViaType::Through);
    assert_eq!(sizes.via_diameter, 600000);
    assert_eq!(sizes.via_drill, 250000);
    assert_eq!(sizes.diff_pair_width, 125000);
    assert_eq!(sizes.diff_pair_gap, 180000);
    assert_eq!(sizes.diff_pair_via_gap, 180000);
    assert!(sizes.diff_pair_via_gap_same_as_trace_gap);
    assert_eq!(sizes.hole_to_hole, 0);
    assert_eq!(sizes.diff_pair_hole_to_hole, 0);
    assert_eq!(sizes.diff_pair_copper_to_hole, 0);
    assert!(sizes.layer_pairs.is_empty());
    assert_eq!(sizes.layer_top(), None);
    assert_eq!(sizes.layer_bottom(), None);
  }

  /// `AddLayerPair` writes both directions and normalises the order
  /// (`pcbnew/router/pns_sizes_settings.cpp:36`), and the top and bottom
  /// accessors read the first entry of the ordered map (`:46`, `:55`).
  #[test]
  fn layer_pairs_answer_from_either_end() {
    let mut sizes = Sizes::default();

    sizes.add_layer_pair(3, 1);

    assert_eq!(sizes.paired_layer(1), Some(3));
    assert_eq!(sizes.paired_layer(3), Some(1));
    assert_eq!(sizes.paired_layer(2), None);
    assert_eq!(sizes.layer_top(), Some(1));
    assert_eq!(sizes.layer_bottom(), Some(3));

    // A second pair with a smaller top takes over the first entry,
    // because the map is ordered by key.
    sizes.add_layer_pair(2, 0);
    assert_eq!(sizes.layer_top(), Some(0));
    assert_eq!(sizes.layer_bottom(), Some(2));

    sizes.clear_layer_pairs();
    assert_eq!(sizes.paired_layer(1), None);
    assert_eq!(sizes.layer_top(), None);
  }

  /// `ROUTER_IFACE::GetViaLayerRange` (`pcbnew/router/pns_router.h:144`)
  /// over the layer pair, which is the whole span here whatever the via
  /// type; see the deviation on [`Sizes::via_layer_range`].
  #[test]
  fn the_via_layer_range_comes_from_the_first_layer_pair() {
    let mut sizes = Sizes::default();

    assert_eq!(sizes.via_layer_range(), None);

    sizes.add_layer_pair(3, 1);

    let range = sizes
      .via_layer_range()
      .expect("a registered pair gives a span");

    assert_eq!(range.start(), 1);
    assert_eq!(range.end(), 3);
    assert!(range.contains(2));

    // A blind via answers from the same pair, because this crate has no
    // board layer count to give a through via instead.
    sizes.via_type = ViaType::Blind;
    assert_eq!(sizes.via_layer_range(), Some(range));

    sizes.clear_layer_pairs();
    assert_eq!(sizes.via_layer_range(), None);
  }

  /// `DiffPairViaGap` (`pcbnew/router/pns_sizes_settings.h:87`) hands back
  /// the trace gap while the flag is set.
  #[test]
  fn the_via_gap_tracks_the_trace_gap() {
    let mut sizes = Sizes {
      diff_pair_gap: 100000,
      diff_pair_via_gap: 200000,
      ..Sizes::default()
    };

    assert_eq!(sizes.diff_pair_via_gap(), 100000);

    sizes.diff_pair_via_gap_same_as_trace_gap = false;
    assert_eq!(sizes.diff_pair_via_gap(), 200000);
  }

  /// `DIFF_PAIR_PLACER::gap` (`pns_diff_pair_placer.cpp:616`) is the
  /// centre to centre distance and therefore the copper gap plus one
  /// lane's width, which is the quantity every gateway builder spaces its
  /// anchors by.
  #[test]
  fn the_pair_pitch_is_the_gap_plus_one_width() {
    let sizes = Sizes {
      diff_pair_width: 200000,
      diff_pair_gap: 180000,
      ..Sizes::default()
    };

    assert_eq!(sizes.diff_pair_pitch(), 380000);
    assert_eq!(Sizes::default().diff_pair_pitch(), 125000 + 180000);
  }

  /// `EffectiveDiffPairViaGap` (`:146`) takes the largest of the three
  /// candidates, with a truncating annular ring.
  #[test]
  fn the_effective_via_gap_folds_in_the_hole_rules() {
    let mut sizes = Sizes::default();

    // The plain gap wins with the defaults: both hole rules are zero and
    // the annular ring is (600000 - 250000) / 2 = 175000, so the two
    // candidates are negative.
    assert_eq!(sizes.effective_diff_pair_via_gap(), 180000);

    // Hole to hole minus two annular rings: 900000 - 350000.
    sizes.diff_pair_hole_to_hole = 900000;
    assert_eq!(sizes.effective_diff_pair_via_gap(), 550000);

    // Copper to hole minus one: 800000 - 175000, which does not beat it.
    sizes.diff_pair_copper_to_hole = 800000;
    assert_eq!(sizes.effective_diff_pair_via_gap(), 625000);
  }
}
