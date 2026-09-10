// SPDX-License-Identifier: GPL-3.0-or-later

//! Dragging several traces at once.
//!
//! Port of `PNS::MULTI_DRAGGER` (`pcbnew/router/pns_multi_dragger.h:44`,
//! `pcbnew/router/pns_multi_dragger.cpp`). Every `// :NNNN` citation in
//! this module is a line of `pcbnew/router/pns_multi_dragger.cpp` unless
//! another file is named. The reference note is
//! `doc/reference/kicad/06-dragger.md` section 5.
//!
//! KiCad's own class comment is "Dragging algorithm for multiple
//! segments. Very trival version for demonstration purposes."
//! (`pcbnew/router/pns_multi_dragger.h:44`), and the mechanism is as
//! plain as that says: **drag the line the cursor is attached to, then
//! move every other line to the point at the same perpendicular offset
//! from the primary line's reference segment that it had before the
//! drag.** Lines whose reference segment does not run parallel to the
//! primary's are left where they are.
//!
//! # What is transcribed
//!
//! Every routine of the file that a board without arcs can reach:
//!
//! - `MdragLine`, the per line record (`:127`);
//! - [`MultiDragger::start`] (`:45`), which is four phases: deduplicate
//!   the selected primitives into lines, classify each line as a corner
//!   grab or a mid segment grab, pick the drag mode, and then the corner
//!   mode sanity check, the primary selection and the shove;
//! - [`MultiDragger::drag`] (`:704`) with its `tryPosture` lambda
//!   (`MultiDragger::try_posture`) run for variants 0, 1 and 2;
//! - `multidragMarkObstacles` (`:573`) and `clip_to_other_line`
//!   (`:294`), the only mode that shortens lines against each other
//!   rather than routing around;
//! - `multidragWalkaround` (`:458`) and its `tryWalkaround` (`:384`),
//!   which walks the set in both orders and keeps the cheaper one;
//! - `multidragShove` (`:613`), the one caller in KiCad's tree that
//!   deliberately controls the shove's head order;
//! - `findNewLeaderSegment` (`:407`) and `restoreLeaderSegments` (`:429`),
//!   which are what
//!   [`MultiDragger::last_committed_leader_segments`] answers with so a
//!   host can put the user's selection back on the segments the router
//!   made;
//! - `FixRoute` (`:366`), `CurrentNode` (`:1000`), `Traces` (`:1006`),
//!   `CurrentNets` (`:351`) and `CurrentLayer` (`:1012`).
//!
//! # The errata, transcribed one by one
//!
//! Note 06 section 8.4 lists twelve. Eleven are reproduced as they stand,
//! each with a comment at the line that carries it, because none of them
//! has a fixture asking for a repair:
//!
//! - **E17**, `multidragWalkaround` storing the reverse attempt's results
//!   under the forward attempt's indices (`MultiDragger::multidrag_walkaround`);
//! - **E18**, `Start` reversing lines before the joint test that may
//!   abandon corner mode ([`MultiDragger::start`]);
//! - **E19**, `lastPreDrag` read before it is written in the segment
//!   branch of `tryPosture`, which is a debug shape in KiCad and has no
//!   counterpart here;
//! - **E20**, `Drag` ignoring whether any posture succeeded
//!   ([`MultiDragger::drag`]);
//! - **E21**, `Mode()` always answering `DM_CORNER` and `SetMode` being
//!   an empty body: not ported at all, since [`MultiDragger::mode`]
//!   answers the real mode and there is no mode input (note 06 erratum
//!   E2);
//! - **E22**, `CurrentLayer()` returning `0`
//!   ([`MultiDragger::current_layer`]);
//! - **E23**, `GetForceMarkObstaclesMode` always returning false
//!   ([`MultiDragger::force_mark_obstacles_mode`]);
//! - **E24**, `FixRoute` ignoring `aForceCommit`
//!   ([`MultiDragger::fix_route_node`]);
//! - **E25**, `clipToOtherLine` producing an empty line when the very
//!   first probe collides (`clip_to_other_line`). Its other half, the
//!   `int` narrowing of the chain length, cannot be inherited:
//!   [`crate::geometry::line_chain::LineChain::length`] and
//!   [`crate::geometry::line_chain::LineChain::point_along`] are both
//!   `i64` here;
//! - **E26**, the primary line disagreeing with the drag mode
//!   ([`MultiDragger::start`]);
//! - **E27**, `multidragShove` returning without restoring the leader
//!   segments on failure (`MultiDragger::multidrag_shove`).
//!
//! # The one deviation: E28, the line order
//!
//! `MDRAG_LINE::dragDist` is only ever assigned in the segment branch of
//! `tryPosture` (`:907`), so in corner mode every line compares equal
//! under `compareDragStartDist` and `std::sort` (`:472`, `:629`) is not
//! stable. The walkaround attempt order and the shove head order are
//! therefore **unspecified** in KiCad's corner mode, and both decide the
//! outcome: the first line walked has the free space, and the first head
//! shoved sets the ranks.
//!
//! `DESIGN.md` section 8 forbids an unspecified order, so this port sorts
//! by `(drag_dist, mdrag_index)`, which is the tie break note 06 erratum
//! E28 names. In segment mode, where `dragDist` is assigned, the order is
//! KiCad's; in corner mode it is the order the host listed the selected
//! items in. This is the module's only deliberate behaviour difference.
//!
//! # What is not ported
//!
//! - **The dead members** note 06 erratum E1 names: `MDRAG_LINE`'s
//!   `leaderItem`, `preShoveLine`, `clipDone` and `offset`, none of which
//!   is ever read or written in KiCad's tree, and `isDraggable`, which is
//!   set true at construction (`:81`) and never set false, so the guard
//!   at `:820` always passes.
//! - **`SetMode` and `Mode()`** (`:284`, `:289`). The first has an empty
//!   body and the second answers `DM_CORNER` unconditionally; neither has
//!   a caller (note 06 erratum E1).
//! - **`SetDefaultShovePolicy`** (`:277`, `:647`), a no operation in this
//!   revision of KiCad (note 06 erratum E3); `src/shove.rs` has no
//!   equivalent.
//! - **`SHOVE::ShoveMultiLines`**, which note 06 section 0 shows does not
//!   exist anywhere in KiCad's tree. Both draggers drive the shove
//!   through `ClearHeads` / `AddHeads` / `Run` alone, and
//!   [`Shove::add_head_line`] already takes one head per line, so the
//!   shove needed nothing new for this module.
//! - **Arcs.** `dyn_cast<SEGMENT*>` at `:147` silently skips an arc link,
//!   `l.originalLine.Reverse()` reverses the arc flags, and
//!   `Line::drag_corner` has no arc case; `PLAN.md` has arcs on hold.
//!
//! # The node tree
//!
//! ```text
//! world root                       handed in to MultiDragger::new
//!  +-- pre_shove_node               world.branch(root) in start, shove
//!  |    |   with every dragged line removed, Shove::new stands here
//!  |    +-- last_node               shove.current_node().branch()
//!  +-- pre_walk_node                world.branch(root), walkaround
//!  |    +-- last_node               two attempts, the cheaper one kept
//!  +-- last_node                    world.branch(root), mark obstacles
//! ```
//!
//! `CurrentNode()` is `m_lastNode ? m_lastNode : m_world` (`:1000`), so
//! before the first [`MultiDragger::drag`] the dragger reports the
//! untouched board.
//!
//! KiCad rebuilds `preWalkNode` on every drag (`:475`) and never deletes
//! it, so a long walkaround multi drag leaks one node per mouse move.
//! `MultiDragger::multidrag_walkaround` drops the previous one instead,
//! which takes `m_lastNode` with it and subsumes the `delete m_lastNode`
//! at `:461`.

use crate::algo_base::AlgoContext;
use crate::collide::CollisionSearchOptions;
use crate::dragger::DragMode;
use crate::geometry::direction45::{AngleType, Direction45};
use crate::geometry::line_chain::LineChain;
use crate::geometry::math::sign;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::Vec2;
use crate::item::{ItemBody, ItemId, NetId};
use crate::line::Line;
use crate::node::{NodeId, World};
use crate::settings::RouterMode;
use crate::shove::{Shove, ShovePolicy, ShoveStatus};
use crate::walkaround::{WalkPolicy, Walkaround, WalkaroundStatus};

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

/// The divisor the multi drag applies to the primary line's width to get
/// its corner snap threshold.
///
/// `Settings().SmoothDraggedSegments() ? primaryDragged->Width() / 4 : 0`
/// (`:755`). Unlike the single dragger, which uses a quarter in two paths
/// and a half in the third
/// (`crate::dragger`'s `MARK_OBSTACLES_SNAP_DIVISOR` against its
/// `SHOVE_SNAP_DIVISOR`), the multi dragger computes one threshold before
/// the mode branch and uses it for the primary line (`:765`, `:806`) and
/// for every parallel one (`:914`).
const SNAP_DIVISOR: i32 = 4;

/// How far a multi drag walkaround may stray before it gives up.
///
/// `walkaround.SetLengthLimit( true, 3.0 )` (`:391`), a **tenth** of the
/// single dragger's 30.0 (`pcbnew/router/pns_dragger.cpp:692`). Dragging
/// a bundle is not allowed the enormous detour a single trace is, because
/// a detour that long would cross the lines the drag is moving with it.
const WALKAROUND_LENGTH_LIMIT_FACTOR: f64 = 3.0;

/// The step below which [`clip_to_other_line`]'s binary search stops.
///
/// `constexpr int clipLengthThreshold = 100` (`:299`), 100 nanometres.
const CLIP_LENGTH_THRESHOLD: i64 = 100;

/// How far the perpendicular ray of the segment branch reaches.
///
/// The literal `10000000` of `:897`, 10 mm. It only has to be long enough
/// that [`Seg::line_project`] of the drag start point is numerically
/// sane; note 06 section 5.3 records it as a magic constant.
const PERPENDICULAR_RAY_LENGTH: i32 = 10_000_000;

// ---------------------------------------------------------------------
// The per line record
// ---------------------------------------------------------------------

/// One line of a multi drag, and how the click classified it.
///
/// Port of `MDRAG_LINE` (`pcbnew/router/pns_multi_dragger.h:127`), minus
/// the five fields note 06 erratum E1 shows are dead. One of these exists
/// per **line**, not per selected primitive: several selected segments of
/// one trace collapse into one record with several
/// [`MdragLine::original_leaders`].
#[derive(Clone, Debug)]
struct MdragLine {
  /// Every selected primitive that assembled into this line.
  ///
  /// `originalLeaders` (`:130`). Never read after `Start`, in KiCad or
  /// here, but it is the record of why this line is in the set.
  original_leaders: Vec<ItemId>,
  /// Whether the cursor is within half a track width of this line's
  /// corner or of its leader segment. `isStrict` (`:132`).
  is_strict: bool,
  /// Whether a selected segment of this line was found under the cursor.
  /// `isMidSeg` (`:133`).
  is_mid_seg: bool,
  /// Whether this line offers a corner grab. `isCorner` (`:134`).
  is_corner: bool,
  /// Which segment of the line the user grabbed.
  ///
  /// `leaderSegIndex` (`:137`), whose KiCad default is `-1`. Note 06
  /// section 5.1 calls it "for corner mode a point index, for segment
  /// mode a link index"; without arcs a line has exactly one link per
  /// segment, so both spellings are the same segment index and it is one
  /// here. `:894` then reads it as a **point** index, which is KiCad's
  /// and is transcribed: point `i` is where segment `i` starts.
  ///
  /// [`None`] is the `-1`, which reaches `CSegment( -1 )`, the last
  /// segment. It is unreachable without arcs: the link loop at `:145`
  /// assigns it for every line, because every line was assembled from a
  /// selected segment that is one of its own links.
  leader_seg_index: Option<usize>,
  /// Whether the grabbed corner is the chain's last point.
  /// `cornerIsLast` (`:138`).
  corner_is_last: bool,
  /// The line as assembled, possibly reversed by
  /// [`MultiDragger::start`]. `originalLine` (`:140`).
  original_line: Line,
  /// [`MdragLine::original_line`] at the top of each posture attempt,
  /// possibly with its last point removed by variant 1. `preDragLine`
  /// (`:141`).
  pre_drag_line: Line,
  /// What the drag produced. `draggedLine` (`:142`).
  dragged_line: Line,
  /// Whether this line produced a usable drag this posture. `dragOK`
  /// (`:145`).
  drag_ok: bool,
  /// Whether the cursor is attached to this line. `isPrimaryLine`
  /// (`:146`).
  is_primary_line: bool,
  /// The grabbed segment, in segment mode. `midSeg` (`:149`).
  mid_seg: Seg,
  /// The signed distance this line travels along the perpendicular.
  ///
  /// `dragDist` (`:150`), written only in the segment branch (`:907`), so
  /// in corner mode every line compares equal; see the module
  /// documentation for the deterministic tie break that replaces KiCad's
  /// unstable sort.
  drag_dist: i32,
  /// How far the cursor is from this line's nearer end. `cornerDistance`
  /// (`:151`).
  corner_distance: i32,
  /// How far the cursor is from this line's leader segment, plus half a
  /// track width. `leaderSegDistance` (`:152`).
  leader_seg_distance: i32,
  /// This record's index in [`MultiDragger::mdrag_lines`].
  ///
  /// `mdragIndex` (`:153`), which exists so `multidragShove` can tell
  /// which lines the posture dropped and put them back (`:663`).
  mdrag_index: usize,
}

impl MdragLine {
  /// A record for one freshly assembled line.
  ///
  /// The member initialisers of `MDRAG_LINE`
  /// (`pcbnew/router/pns_multi_dragger.h:127` to `:154`) plus the four
  /// assignments `Start` makes at `:79` to `:82`. KiCad's
  /// `leaderSegDistance = 0` default matters: a line whose every selected
  /// link is an arc keeps it and then wins the `bestSeg` argmin at
  /// `:194`. Without arcs no line keeps it.
  fn new(original_line: Line, leader: ItemId, mdrag_index: usize) -> Self {
    Self {
      original_leaders: vec![leader],
      is_strict: false,
      is_mid_seg: false,
      is_corner: false,
      leader_seg_index: None,
      corner_is_last: false,
      original_line,
      pre_drag_line: Line::new(),
      dragged_line: Line::new(),
      drag_ok: false,
      is_primary_line: false,
      mid_seg: Seg::new(Vec2::new(0, 0), Vec2::new(0, 0)),
      drag_dist: 0,
      corner_distance: 0,
      leader_seg_distance: 0,
      mdrag_index,
    }
  }
}

// ---------------------------------------------------------------------
// The dragger
// ---------------------------------------------------------------------

/// A drag of several traces at once.
///
/// Port of `MULTI_DRAGGER` (`pcbnew/router/pns_multi_dragger.h:44`). One
/// is built per gesture, exactly like [`crate::dragger::Dragger`]:
/// [`MultiDragger::start`] decides what the selection grabbed,
/// [`MultiDragger::drag`] answers every mouse move and
/// [`MultiDragger::fix_route`] commits or refuses.
///
/// `ROUTER::StartDragging` picks between the two from the **shape of the
/// item set** and not from a drag mode (`pcbnew/router/pns_router.cpp:176`
/// to `:190`): more than one segment or arc reaches this one.
pub struct MultiDragger {
  /// The node the drag branches from, KiCad's `m_world`
  /// (`pcbnew/router/pns_drag_algo.h:128`), which
  /// `ROUTER::StartDragging` fills with the router's **root**
  /// (`pcbnew/router/pns_router.cpp:194`).
  world_node: NodeId,
  /// Whether the last drag position is legal. `m_dragStatus`
  /// (`pcbnew/router/pns_multi_dragger.h:157`), which is the answer of
  /// the mode routine and not a collision test: only mark obstacles mode
  /// answers unconditionally true.
  drag_status: bool,
  /// What the selection turned out to grab. `m_dragMode` (`:158`).
  ///
  /// Only [`DragMode::Corner`] and [`DragMode::Segment`] are reachable:
  /// a multi drag never grabs a via, because a set that reaches this
  /// dragger holds more than one segment or arc
  /// (`pcbnew/router/pns_router.cpp:182`).
  drag_mode: DragMode,
  /// One record per line the selection resolved to. `m_mdragLines`
  /// (`:159`).
  mdrag_lines: Vec<MdragLine>,
  /// The segments a host should put the selection back on.
  /// `m_leaderSegments` (`:160`).
  leader_segments: Vec<ItemId>,
  /// The answer of the last [`MultiDragger::drag`]. `m_lastNode`
  /// (`:161`).
  last_node: Option<NodeId>,
  /// The branch every dragged line was removed from, which the shove
  /// stands on. `m_preShoveNode` (`:162`), built by
  /// [`MultiDragger::start`] in [`RouterMode::Shove`] alone (`:267`).
  pre_shove_node: Option<NodeId>,
  /// The branch a walkaround attempt is taken from.
  ///
  /// KiCad's `preWalkNode` is a local of `multidragWalkaround` (`:475`)
  /// that is never deleted, so it leaks one node per mouse move. It is a
  /// member here so that the next drag can drop it, which takes
  /// [`MultiDragger::last_node`] with it.
  pre_walk_node: Option<NodeId>,
  /// The primitives the host selected. `m_origDraggedItems` (`:163`).
  orig_dragged_items: Vec<ItemId>,
  /// What [`MultiDragger::traces`] answers. `m_draggedItems` (`:164`).
  dragged_items: Vec<Line>,
  /// Where the gesture began. `m_dragStartPoint` (`:165`, `:49`), read
  /// only by the segment branch of [`MultiDragger::try_posture`]
  /// (`:898`).
  drag_start_point: Vec2,
  /// The perpendicular ray through the cursor, in segment mode.
  ///
  /// `m_guide` (`:166`), written at `:809` and read only by
  /// [`MultiDragger::find_new_leader_segment`] (`:417`), which
  /// `restoreLeaderSegments` calls only in the non corner branch. So the
  /// stale value a corner mode drag leaves here is never read.
  guide: Seg,
  /// The shove engine, allocated by [`MultiDragger::start`] in
  /// [`RouterMode::Shove`] alone. `m_shove` (`:167`, `:274`).
  shove: Option<Shove>,
}

impl MultiDragger {
  // -----------------------------------------------------------------
  // Construction
  // -----------------------------------------------------------------

  /// A multi dragger over one node.
  ///
  /// The constructor (`:33`) plus `SetWorld`
  /// (`pcbnew/router/pns_drag_algo.h:61`).
  ///
  /// # Panics
  ///
  /// When `node` is not a live node of `world`, which is the same guard
  /// [`crate::dragger::Dragger::new`] applies.
  pub fn new(world: &World, node: NodeId) -> Self {
    assert!(
      world.node(node).is_some(),
      "a multi dragger needs a live node to branch from"
    );

    Self {
      world_node: node,
      // :48
      drag_status: false,
      // KiCad leaves `m_dragMode` indeterminate until phase 3 of `Start`
      // (note 06 section 5.8); a refused start would read it uninitialised
      // there and nothing does. Segment is the safe seed here.
      drag_mode: DragMode::Segment,
      mdrag_lines: Vec::new(),
      leader_segments: Vec::new(),
      last_node: None,
      pre_shove_node: None,
      pre_walk_node: None,
      orig_dragged_items: Vec::new(),
      dragged_items: Vec::new(),
      drag_start_point: Vec2::new(0, 0),
      guide: Seg::new(Vec2::new(0, 0), Vec2::new(0, 0)),
      shove: None,
    }
  }

  // -----------------------------------------------------------------
  // Accessors
  // -----------------------------------------------------------------

  /// What the selection turned out to grab.
  ///
  /// KiCad's `Mode()` (`:289`) answers `DM_CORNER` unconditionally
  /// whatever `m_dragMode` holds, and has no caller anywhere in its tree
  /// (note 06 errata E21 and E1). This answers the real mode, which is
  /// what the tests read.
  pub const fn mode(&self) -> DragMode {
    self.drag_mode
  }

  /// The node holding everything the drag has changed.
  ///
  /// Port of `CurrentNode` (`:1000`): the last drag's node, or the world
  /// the dragger was given when no drag has happened yet.
  pub fn current_node(&self) -> NodeId {
    self.last_node.unwrap_or(self.world_node)
  }

  /// The lines the drag is moving.
  ///
  /// Port of `Traces` (`:1006`), which answers `m_draggedItems`.
  ///
  /// Two things about that set, both KiCad's. The **primary** line is
  /// never in it: `tryPosture` adds only the parallel lines (`:880`, and
  /// nothing adds the primary), so the line under the cursor is not
  /// skipped by `ROUTER::markViolations`
  /// (`pcbnew/router/pns_router.cpp:726`). And in
  /// [`DragMode::Segment`] the set is always **empty**, because the
  /// `m_draggedItems.Add` at `:880` sits in the corner branch alone.
  pub fn traces(&self) -> &[Line] {
    &self.dragged_items
  }

  /// The segments a host should restore the user's selection onto.
  ///
  /// Port of `GetLastCommittedLeaderSegments`
  /// (`pcbnew/router/pns_multi_dragger.h:112`), the one `DRAG_ALGO`
  /// method `MULTI_DRAGGER` overrides and `DRAGGER` does not.
  ///
  /// The router deleted the segments the user had selected and made new
  /// ones, so the host is handed the new ones and selects their parent
  /// objects (`pcbnew/router/router_tool.cpp:3238` to `:3246`). Before
  /// the first drag, and while a posture attempt is running, the list is
  /// the selection as it arrived (`:813`).
  ///
  /// The ids name items of [`MultiDragger::current_node`]. A commit
  /// re homes them into the root rather than replacing them, so they stay
  /// resolvable after [`MultiDragger::fix_route`].
  pub fn last_committed_leader_segments(&self) -> &[ItemId] {
    &self.leader_segments
  }

  /// The nets the drag is on.
  ///
  /// Port of `CurrentNets` (`:351`), which collects
  /// `l.draggedLine.Net()` over every line, skipping the null net.
  ///
  /// # Deviation
  ///
  /// KiCad deduplicates through a `std::set<NET_HANDLE>`, which is
  /// **pointer ordered**; `DESIGN.md` section 8 forbids that. The order
  /// here is `MultiDragger::mdrag_lines` order, first occurrence wins.
  /// Nothing observable depends on it: `ROUTER::GetCurrentNets`'s only
  /// callers are in `pns_dp_meander_placer.cpp`, which never runs during
  /// a drag (note 06 section 1.5).
  ///
  /// `draggedLine` is only assigned by a drag, so this answers nothing
  /// before the first [`MultiDragger::drag`].
  pub fn current_nets(&self) -> Vec<NetId> {
    let mut nets: Vec<NetId> = Vec::new();

    for line in &self.mdrag_lines {
      if let Some(net) = line.dragged_line.net()
        && !nets.contains(&net)
      {
        nets.push(net);
      }
    }

    nets
  }

  /// The layer the drag is on.
  ///
  /// Port of `CurrentLayer` (`:1012`), which is `return 0;` under a
  /// "fixme: should we care?" comment. Note 06 erratum E22; the accessor
  /// is unreachable in KiCad, so the wrong answer is latent there and is
  /// reproduced here rather than quietly repaired.
  pub const fn current_layer(&self) -> i32 {
    0
  }

  /// Whether the drag has fallen back to highlighting, and whether the
  /// last position was legal.
  ///
  /// Port of `GetForceMarkObstaclesMode`
  /// (`pcbnew/router/pns_multi_dragger.h:114`), which writes
  /// `m_dragStatus` out and **always returns false**. Note 06 erratum
  /// E23: the host therefore never shows the "Track violates DRC" hint
  /// during a multi drag, even though [`MultiDragger::fix_route`] will
  /// refuse to commit for exactly that reason.
  pub const fn force_mark_obstacles_mode(&self) -> (bool, bool) {
    (false, self.drag_status)
  }

  // -----------------------------------------------------------------
  // Start
  // -----------------------------------------------------------------

  /// Begin a drag on a set of items at a point.
  ///
  /// Port of `Start` (`:45`), in KiCad's four phases.
  ///
  /// 1. **Deduplicate into lines** (`:60`). A selection of five segments
  ///    of one trace becomes one `MdragLine` with five
  ///    `MdragLine::original_leaders`.
  /// 2. **Classify each line** (`:90`). A line offers a **corner** grab
  ///    when the selection holds a segment with an endpoint exactly at
  ///    one of the line's ends, and a **mid segment** grab for every
  ///    selected link; either becomes *strict* when the cursor is within
  ///    half a track width of it. The link loop keeps overwriting, so the
  ///    **last** selected segment of a line wins and not the nearest to
  ///    the cursor.
  /// 3. **Pick the drag mode** (`:176`). A strict corner anywhere wins,
  ///    then a strict mid segment anywhere, and otherwise the smaller of
  ///    the two argmin distances decides and marks its line primary.
  /// 4. **Corner mode sanity, the primary, and the shove** (`:227`).
  ///
  /// False means the drag never started: an empty set (`:52`), or phase 3
  /// finding neither candidate, which KiCad's own comment marks "can it
  /// really happen?" (`:224`).
  ///
  /// # Two errata live in phase 4
  ///
  /// **E18.** `l.originalLine.Reverse()` (`:234`) runs **before** the
  /// joint test at `:239`, and a missing joint sets `DM_SEGMENT` and
  /// `break`s. So a set that falls back to segment mode is left with the
  /// lines processed so far reversed and the rest not, while
  /// `MdragLine::leader_seg_index` still names the pre reversal
  /// segment order. The `!IsTrivialEndpoint` case at `:247` sets the mode
  /// without breaking, so it keeps reversing.
  ///
  /// **E26.** `anyStrictCornersFound` alone forces `DM_CORNER` (`:176`),
  /// but the primary is the **first** strict line in
  /// `MultiDragger::mdrag_lines` order (`:254`), whatever kind of
  /// strictness it has. A set where one line has a strict corner and an
  /// earlier one a strict mid segment drags in corner mode off a line
  /// classified as a mid segment grab.
  ///
  /// `GetRuleResolver()->ClearCaches()` (`pcbnew/router/pns_router.cpp:174`)
  /// has no counterpart; a [`crate::rules::RuleResolver`] here owns
  /// whatever caching it does.
  pub fn start(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
    items: &[ItemId],
  ) -> bool {
    // :47 to :49
    self.last_node = None;
    self.drag_status = false;
    self.drag_start_point = at;

    // :52
    if items.is_empty() {
      return false;
    }

    // :55
    self.mdrag_lines.clear();
    self.dragged_items.clear();
    self.leader_segments.clear();

    self.assemble_lines(world, items);

    let (any_strict_corners, any_strict_mid_segs) =
      self.classify_lines(world, items, at);

    if !self.pick_drag_mode(any_strict_corners, any_strict_mid_segs) {
      // :224
      return false;
    }

    // :227
    if self.drag_mode == DragMode::Corner {
      self.check_corner_mode(world);
    }

    // :254. The first strict line becomes the primary, whichever kind of
    // strictness it has (erratum E26).
    for line in &mut self.mdrag_lines {
      if (any_strict_corners || any_strict_mid_segs) && line.is_strict {
        line.is_primary_line = true;

        break;
      }
    }

    // :263
    self.orig_dragged_items = items.to_vec();

    // :265
    if context.settings.mode == RouterMode::Shove {
      // :267
      let pre_shove = world.branch(self.world_node);

      // :271. The removal reads the lines' links, which the assembly
      // filled in, so it takes the real board segments out.
      //
      // It runs on a copy, where KiCad removes `l.originalLine` itself
      // and so leaves the record's own line unlinked. Nothing in shove
      // mode reads those links again, because every later use clears them
      // first (`:673`, `:735`, `:860`, `:913`), so the difference is
      // invisible; the copy only keeps the record honest if a session's
      // routing mode ever changes between two moves.
      for line in &self.mdrag_lines {
        let mut original = line.original_line.clone();

        world.remove_line(pre_shove, &mut original);
      }

      self.pre_shove_node = Some(pre_shove);
      // :274. `SetDefaultShovePolicy` at `:277` is a no operation
      // (erratum E3) and has no counterpart.
      self.shove = Some(Shove::new(pre_shove));
    }

    true
  }

  /// Phase 1: turn the selected primitives into one record per line.
  ///
  /// `:60` to `:85`. A primitive whose line is already in the set is
  /// recorded as another leader of it and skipped, which is what makes
  /// five selected segments of one trace a single [`MdragLine`].
  ///
  /// KiCad's `AssembleLine( litem )` takes every default
  /// (`pcbnew/router/pns_node.h:441`), and the line is assembled from the
  /// **root**, not from a branch.
  fn assemble_lines(&mut self, world: &World, items: &[ItemId]) {
    for item in items {
      // :64
      if let Some(line) = self
        .mdrag_lines
        .iter_mut()
        .find(|line| line.original_line.contains_link(*item))
      {
        // :68
        line.original_leaders.push(*item);

        continue;
      }

      // :79
      let assembled =
        world.assemble_line(self.world_node, *item, None, false, false, true);

      self.mdrag_lines.push(MdragLine::new(
        assembled,
        *item,
        self.mdrag_lines.len(),
      ));
    }
  }

  /// Phase 2: decide what each line offers the cursor.
  ///
  /// `:90` to `:174`. The answer is the pair
  /// `(anyStrictCornersFound, anyStrictMidSegsFound)` of `:171` and
  /// `:172`.
  ///
  /// `ITEM_SET::FindVertex` (`pcbnew/router/pns_itemset.cpp:137`) is
  /// [`find_vertex`]: "the user selected a segment that touches this
  /// line's end" is what makes the line a corner candidate.
  fn classify_lines(
    &mut self,
    world: &World,
    items: &[ItemId],
    at: Vec2,
  ) -> (bool, bool) {
    let mut any_strict_corners = false;
    let mut any_strict_mid_segs = false;

    for line in &mut self.mdrag_lines {
      // :92
      let threshold = line.original_line.width() / 2;
      let first = line.original_line.shape().points().first().copied();
      let last = line.original_line.last_point();
      let (Some(first), Some(last)) = (first, last) else {
        continue;
      };
      // :95, :98
      let distance_first = (first - at).euclidean_norm();
      let distance_last = (last - at).euclidean_norm();

      // :100
      line.corner_distance = distance_first.min(distance_last);

      // :103, :104
      let has_last = find_vertex(world, items, last).is_some();
      let has_first = find_vertex(world, items, first).is_some();
      // :106 to :111
      let take_first = if has_last && has_first {
        distance_first < distance_last
      } else {
        has_first
      };

      // :113
      if has_first || has_last {
        if take_first {
          // :117 to :120
          line.corner_is_last = false;
          line.leader_seg_index = Some(0);
          line.corner_distance = distance_first;
          line.is_corner = true;

          // :122
          if distance_first <= threshold {
            line.is_strict = true;
            line.corner_distance = 0;
          }
        } else {
          // :130 to :133
          line.corner_is_last = true;
          line.leader_seg_index =
            line.original_line.segment_count().checked_sub(1);
          line.corner_distance = distance_last;
          line.is_corner = true;

          // :135
          if distance_last <= threshold {
            line.is_strict = true;
            line.corner_distance = 0;
          }
        }
      }

      // :145. The last selected link wins, not the nearest one.
      let links: Vec<ItemId> = line.original_line.links().to_vec();

      for (link_index, link) in links.into_iter().enumerate() {
        // :147, :150
        let Some(item) = world.item(link) else {
          continue;
        };
        let ItemBody::Segment(body) = item.body() else {
          continue;
        };

        if !items.contains(&link) {
          continue;
        }

        // :153 to :158
        let distance = body.seg().distance_to_point(at);

        line.mid_seg = body.seg();
        line.is_mid_seg = true;
        line.leader_seg_index = Some(link_index);
        line.leader_seg_distance = distance + threshold;

        // :160
        if distance < threshold && !line.is_strict {
          line.is_corner = false;
          line.is_strict = true;
          line.leader_seg_distance = 0;
        }
      }

      // :169
      if line.is_strict {
        any_strict_corners |= line.is_corner;
        any_strict_mid_segs |= !line.is_corner;
      }
    }

    (any_strict_corners, any_strict_mid_segs)
  }

  /// Phase 3: choose between a corner drag and a segment drag.
  ///
  /// `:176` to `:225`. False is the `return false` at `:224`, whose own
  /// comment asks "can it really happen?": it needs an
  /// [`MultiDragger::mdrag_lines`] that is empty, which
  /// [`MultiDragger::start`] has already refused.
  fn pick_drag_mode(
    &mut self,
    any_strict_corners: bool,
    any_strict_mid_segs: bool,
  ) -> bool {
    // :176
    if any_strict_corners {
      self.drag_mode = DragMode::Corner;

      return true;
    }

    // :178
    if any_strict_mid_segs {
      self.drag_mode = DragMode::Segment;

      return true;
    }

    // :187 to :199. Both argmins keep the **first** minimum, because the
    // comparison is a strict `<`.
    let mut best_corner: Option<(i32, usize)> = None;
    let mut best_seg: Option<(i32, usize)> = None;

    for (index, line) in self.mdrag_lines.iter().enumerate() {
      if best_corner.is_none_or(|(best, _)| line.corner_distance < best) {
        best_corner = Some((line.corner_distance, index));
      }

      if best_seg.is_none_or(|(best, _)| line.leader_seg_distance < best) {
        best_seg = Some((line.leader_seg_distance, index));
      }
    }

    // :201 to :224
    match (best_corner, best_seg) {
      (Some((corner_distance, corner)), Some((seg_distance, seg))) => {
        if corner_distance < seg_distance {
          self.drag_mode = DragMode::Corner;
          self.mdrag_lines[corner].is_primary_line = true;
        } else {
          self.drag_mode = DragMode::Segment;
          self.mdrag_lines[seg].is_primary_line = true;
        }

        true
      }
      (Some((_, corner)), None) => {
        self.drag_mode = DragMode::Corner;
        self.mdrag_lines[corner].is_primary_line = true;

        true
      }
      (None, Some((_, seg))) => {
        self.drag_mode = DragMode::Segment;
        self.mdrag_lines[seg].is_primary_line = true;

        true
      }
      (None, None) => false,
    }
  }

  /// Phase 4's corner mode check: turn every line so its grabbed corner
  /// is the last point, and fall back to segment mode for a line whose
  /// end is soldered to something.
  ///
  /// `:227` to `:252`. `JOINT::IsTrivialEndpoint`
  /// (`pcbnew/router/pns_joint.h:176`) is "one link and it is a segment",
  /// so the intent of the comment at `:237`, "if it's connected
  /// (non-trivial fanout), disregard it", is to refuse corner mode when a
  /// selected line's end meets a pad, a via or a fan out.
  ///
  /// Erratum E18 is the reversal happening before the test and the
  /// `break` leaving the set half reversed; both are transcribed. See
  /// [`MultiDragger::start`].
  fn check_corner_mode(&mut self, world: &World) {
    for line in &mut self.mdrag_lines {
      // :232. Erratum E18: this runs before the joint test below.
      if !line.corner_is_last {
        line.original_line.reverse();
        line.corner_is_last = true;
      }

      // :239
      let Some(last) = line.original_line.last_point() else {
        continue;
      };
      let joint = world
        .find_joint(
          self.world_node,
          last,
          line.original_line.layers().start(),
          line.original_line.net(),
        )
        .and_then(|reference| world.joint(reference));

      // :241. The `break` leaves the rest of the set un-reversed.
      let Some(joint) = joint else {
        self.drag_mode = DragMode::Segment;

        break;
      };

      // :247. No `break` here, so the reversal keeps going.
      if !joint.is_trivial_endpoint(world.items()) {
        self.drag_mode = DragMode::Segment;
      }
    }
  }

  // -----------------------------------------------------------------
  // Drag
  // -----------------------------------------------------------------

  /// Move the drag to a point.
  ///
  /// Port of `Drag` (`:704`): three posture attempts followed by a
  /// dispatch on the routing mode.
  ///
  /// The three variants are the whole fallback strategy. Variant 0 drags
  /// the lines as they are, variant 1 drops each line's last point first,
  /// and variant 2 keeps the primary's **pre drag** direction as the
  /// reference and tolerates lines that failed to drag (`:945`).
  ///
  /// # Erratum E20: the answer is not consulted
  ///
  /// `res` is assigned at `:970` and never read after the loop, so a drag
  /// that failed all three postures still goes through the shove or the
  /// walkaround with whatever `completed` the last attempt left. That is
  /// transcribed: the variants are a preference and not a precondition,
  /// and refusing here would make a bundle stop following the cursor
  /// wherever the third variant also fails.
  pub fn drag(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    at: Vec2,
  ) -> bool {
    let mut completed = Vec::new();

    // :968 to :974
    for variant in 0..3 {
      let (ok, attempt) = self.try_posture(context, at, variant);

      completed = attempt;

      if ok {
        break;
      }
    }

    // :976
    self.drag_status = match context.settings.mode {
      RouterMode::Walkaround => {
        self.multidrag_walkaround(world, context, &mut completed)
      }
      RouterMode::Shove => self.multidrag_shove(world, context, &mut completed),
      RouterMode::MarkObstacles => {
        self.multidrag_mark_obstacles(world, context, &mut completed)
      }
    };

    // :996
    self.drag_status
  }

  /// One posture attempt.
  ///
  /// Port of the `tryPosture` lambda (`:717` to `:964`). The answer is
  /// KiCad's `bool` plus the `completed` vector the lambda captures by
  /// reference, because the mode dispatch reads that vector whether or
  /// not the attempt succeeded.
  ///
  /// The mechanism, stated plainly: **drag the primary line, then move
  /// every other line to the point at the same perpendicular offset from
  /// the primary's reference segment that it had before the drag.**
  /// `perp` is the perpendicular of the primary's reference segment,
  /// `dist` is the signed line distance of the other line's endpoint from
  /// that segment (`LineDistance( p, true )`,
  /// `libs/kimath/include/geometry/seg.h:155`), and
  /// `aP + perp.Resize( dist )` is where that endpoint should now be.
  /// KiCad's own comment at `:831` calls the algorithm "quite trival".
  ///
  /// A line whose reference segment is not parallel to the primary's is
  /// **skipped entirely**: it keeps its original geometry and never
  /// reaches `completed`, which is what the re-add block of
  /// [`MultiDragger::multidrag_shove`] then has to repair.
  ///
  /// KiCad's debug shape at `:803` reads `lastPreDrag` one line before it
  /// is assigned at `:805` (note 06 erratum E19). It is a debug shape
  /// only and has no counterpart here.
  fn try_posture(
    &mut self,
    context: &AlgoContext<'_>,
    at: Vec2,
    variant: u32,
  ) -> (bool, Vec<MdragLine>) {
    let mut completed: Vec<MdragLine> = Vec::new();
    // :721 to :740
    let Some(primary) = self
      .mdrag_lines
      .iter()
      .position(|line| line.is_primary_line)
    else {
      return (false, completed);
    };

    for line in &mut self.mdrag_lines {
      line.drag_ok = false;
      line.pre_drag_line = line.original_line.clone();
    }

    // :731 to :736. The pre drag copy is what `NODE::Remove` needs to
    // find the segments before the multi drag reshaped them.
    let mut primary_dragged = self.mdrag_lines[primary].original_line.clone();
    let mut primary_pre_drag = self.mdrag_lines[primary].original_line.clone();

    primary_dragged.clear_links();

    // :742
    if variant == 1 && primary_pre_drag.point_count() > 2 {
      remove_last_point(&mut primary_pre_drag);
      remove_last_point(&mut primary_dragged);

      // :747
      for line in &mut self.mdrag_lines {
        remove_last_point(&mut line.pre_drag_line);
      }
    }

    // :755
    let snap_threshold = if context.settings.smooth_dragged_segments {
      primary_dragged.width() / SNAP_DIVISOR
    } else {
      0
    };
    let last_pre_drag;
    let primary_dir;
    let perp;
    let mut primary_last_seg_dir = Direction45::default();

    if self.drag_mode == DragMode::Corner {
      // :762, :763
      let Some(segment) = last_segment(&primary_pre_drag) else {
        return (false, completed);
      };

      last_pre_drag = segment;
      primary_dir = Direction45::from_seg(&last_pre_drag, false);

      // :765, :766
      primary_dragged.set_snap_threshold(snap_threshold);
      primary_dragged.drag_corner(
        at,
        primary_dragged.point_count().saturating_sub(1),
        false,
        Direction45::default(),
      );

      // :769. A collapsed primary is the one hard failure of a posture.
      let Some(last_seg) = last_segment(&primary_dragged) else {
        // :792
        return (false, completed);
      };
      // :771
      let mut last_prim_drag = last_seg;

      // :773. Variant 2 keeps the pre drag direction as the reference.
      if variant == 2 {
        last_prim_drag = last_pre_drag;
      }

      // :777. A short leg that turned is a rounding artefact of the
      // corner drag, not a new direction.
      if Direction45::from_seg(&last_seg, false) != primary_dir
        && last_seg.length() < primary_dragged.width()
      {
        last_prim_drag = last_pre_drag;
      }

      // :785, :786
      perp = (last_prim_drag.b - last_prim_drag.a).perpendicular();
      primary_last_seg_dir = Direction45::from_seg(&last_prim_drag, false);
    } else {
      // :805. The reference is read off the **dragged** copy, before the
      // drag reshapes it.
      let index = self.mdrag_lines[primary].leader_seg_index;
      let Some(index) =
        index.filter(|index| *index < primary_dragged.segment_count())
      else {
        return (false, completed);
      };

      last_pre_drag = primary_dragged.segment(index);
      primary_dir = Direction45::from_seg(&last_pre_drag, false);

      // :806, :807
      primary_dragged.set_snap_threshold(snap_threshold);
      primary_dragged.drag_segment(at, index);

      // :808, :809
      let mid_seg = self.mdrag_lines[primary].mid_seg;

      perp = (mid_seg.b - mid_seg.a).perpendicular();
      self.guide = Seg::new(at, at + perp);
    }

    // :813, :814
    self.leader_segments = self.orig_dragged_items.clone();
    self.dragged_items.clear();

    // :817
    for index in 0..self.mdrag_lines.len() {
      // :822
      self.mdrag_lines[index].drag_ok = false;

      // :826. Reject nulls.
      if self.mdrag_lines[index].pre_drag_line.segment_count() >= 1 {
        let dragged = match self.drag_mode {
          // :837
          DragMode::Corner => self.drag_parallel_corner(
            index,
            at,
            perp,
            last_pre_drag,
            primary_dir,
            primary_last_seg_dir,
          ),
          // :884
          DragMode::Segment => self.drag_parallel_segment(
            index,
            at,
            perp,
            last_pre_drag,
            snap_threshold,
          ),
          // Unreachable: a set that reaches this dragger holds more than
          // one segment or arc (`pcbnew/router/pns_router.cpp:182`).
          DragMode::Via => ParallelDrag::Skipped,
        };

        match dragged {
          // :871, the `continue` that also skips the primary push below.
          ParallelDrag::Collapsed => continue,
          ParallelDrag::Dragged(line) => {
            // :874
            self.mdrag_lines[index].drag_ok = true;

            // :876
            if !self.mdrag_lines[index].is_primary_line {
              self.mdrag_lines[index].dragged_line = (*line).clone();
              completed.push(self.mdrag_lines[index].clone());

              // :880. Corner mode only; the segment branch has no
              // counterpart, so `Traces()` is empty there.
              if self.drag_mode == DragMode::Corner {
                self.dragged_items.push(*line);
              }
            }
          }
          ParallelDrag::Skipped => {}
        }
      }

      // :931
      if self.mdrag_lines[index].is_primary_line {
        self.mdrag_lines[index].dragged_line = primary_dragged.clone();
        self.mdrag_lines[index].drag_ok = true;
        completed.push(self.mdrag_lines[index].clone());
      }
    }

    // :939. Segment mode accepts every posture.
    if self.drag_mode == DragMode::Segment {
      return (true, completed);
    }

    // :943
    for line in &completed {
      // :945. Variant 2 tolerates a line that failed to drag.
      if !line.drag_ok && variant < 2 {
        return (false, completed);
      }

      if line.is_primary_line {
        continue;
      }

      // :953
      let Some(last) = last_segment(&line.dragged_line) else {
        return (false, completed);
      };

      // :958
      if Direction45::from_seg(&last, false) != primary_last_seg_dir {
        return (false, completed);
      }
    }

    (true, completed)
  }

  /// The corner branch of `tryPosture`'s per line loop.
  ///
  /// `:837` to `:882`. The line's last point is projected onto the
  /// perpendicular through the cursor at the offset it had from the
  /// primary's reference segment, and its corner is dragged there with
  /// the primary's own ending direction imposed.
  ///
  /// The angle gate at `:843` is an **equality** against three of the six
  /// angle kinds and not a mask test, unlike the segment branch's.
  fn drag_parallel_corner(
    &self,
    index: usize,
    at: Vec2,
    perp: Vec2,
    last_pre_drag: Seg,
    primary_dir: Direction45,
    primary_last_seg_dir: Direction45,
  ) -> ParallelDrag {
    let line = &self.mdrag_lines[index];
    // :839
    let Some(parallel_seg) = last_segment(&line.pre_drag_line) else {
      return ParallelDrag::Skipped;
    };
    let parallel_dir = Direction45::from_seg(&parallel_seg, false);
    // :841
    let lead_angle = primary_dir.angle(parallel_dir);

    // :843
    if lead_angle != AngleType::OBTUSE
      && lead_angle != AngleType::RIGHT
      && lead_angle != AngleType::STRAIGHT
    {
      return ParallelDrag::Skipped;
    }

    let Some(last_point) = line.pre_drag_line.last_point() else {
      return ParallelDrag::Skipped;
    };
    // :849, the signed overload.
    let distance = last_pre_drag.line_distance_signed(last_point);
    // :852
    let projected = at + perp.resize(distance);
    // :855, :860
    let mut parallel_dragged = line.pre_drag_line.clone();

    parallel_dragged.clear_links();
    // :863
    parallel_dragged.drag_corner(
      projected,
      parallel_dragged.point_count().saturating_sub(1),
      false,
      primary_last_seg_dir,
    );

    // :871. `DragCorner` can collapse a very short secondary line to a
    // single point.
    if parallel_dragged.segment_count() < 1 {
      return ParallelDrag::Collapsed;
    }

    ParallelDrag::Dragged(Box::new(parallel_dragged))
  }

  /// The segment branch of `tryPosture`'s per line loop.
  ///
  /// `:884` to `:926`. The line's leader segment is moved sideways to the
  /// perpendicular offset it had, and [`MdragLine::drag_dist`] records
  /// how far it travelled and in which direction, which is what orders
  /// the walkaround attempts and the shove heads.
  ///
  /// The angle gate at `:891` is a **mask** test against
  /// [`AngleType::HALF_FULL`] and [`AngleType::STRAIGHT`], so a segment
  /// running the same way as the primary's leader or exactly against it
  /// is dragged and everything else is left alone.
  fn drag_parallel_segment(
    &mut self,
    index: usize,
    at: Vec2,
    perp: Vec2,
    last_pre_drag: Seg,
    snap_threshold: i32,
  ) -> ParallelDrag {
    let line = &self.mdrag_lines[index];
    // :886 to :889
    let reference_dir = Direction45::from_seg(&last_pre_drag, false);
    let current_dir = Direction45::from_seg(&line.mid_seg, false);
    let angle = reference_dir.angle(current_dir);

    // :891
    if !angle.intersects(AngleType::HALF_FULL | AngleType::STRAIGHT) {
      return ParallelDrag::Skipped;
    }

    // :893. `leaderSegIndex` is read as a **point** index here, where
    // everywhere else it is a segment index; point `i` is where segment
    // `i` starts.
    let Some(leader) = line
      .leader_seg_index
      .filter(|leader| *leader < line.pre_drag_line.point_count())
    else {
      return ParallelDrag::Skipped;
    };
    let distance =
      last_pre_drag.line_distance_signed(line.pre_drag_line.point(leader));
    // :895
    let projected = at + perp.resize(distance);
    // :897, :898
    let ray = Seg::new(at, at + perp.resize(PERPENDICULAR_RAY_LENGTH));
    let start_projection = ray.line_project(self.drag_start_point);
    // :906, :907
    let travel = projected - start_projection;
    let drag_dist = travel.euclidean_norm() * sign(travel.dot(perp));

    self.mdrag_lines[index].drag_dist = drag_dist;

    // :910
    if self.mdrag_lines[index].is_primary_line {
      // The primary's own geometry comes from `primaryDragged` at `:933`;
      // the branch above only recorded its drag distance.
      return ParallelDrag::Dragged(Box::default());
    }

    // :912 to :915
    let mut dragged = self.mdrag_lines[index].pre_drag_line.clone();

    dragged.clear_links();
    dragged.set_snap_threshold(snap_threshold);

    if leader < dragged.segment_count() {
      dragged.drag_segment(projected, leader);
    }

    ParallelDrag::Dragged(Box::new(dragged))
  }

  // -----------------------------------------------------------------
  // Mark obstacles mode
  // -----------------------------------------------------------------

  /// Move the whole bundle and let whatever it runs into be highlighted.
  ///
  /// Port of `multidragMarkObstacles` (`:573`). It is the only mode that
  /// shortens the lines **against each other** rather than routing
  /// around: every ordered pair is handed to [`clip_to_other_line`], and
  /// then the originals are swapped for the dragged shapes in a fresh
  /// branch of the world.
  ///
  /// Always answers true, so a mark obstacles multi drag never refuses a
  /// position and [`MultiDragger::fix_route`] always commits.
  ///
  /// KiCad's comment at `:583` is the best one line description of the
  /// branch model in its whole tree: "m_lastNode contains the temporary
  /// (post-modification) state. Think of it as of an efficient undo
  /// buffer."
  fn multidrag_mark_obstacles(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    completed: &mut [MdragLine],
  ) -> bool {
    // :577
    self.drop_last_node(world);

    // :587
    let last = world.branch(self.world_node);

    self.last_node = Some(last);

    // :590. Every ordered pair, the earlier line clipping the later one.
    for first in 0..completed.len() {
      for second in (first + 1)..completed.len() {
        let reference = completed[first].dragged_line.clone();
        let mut clipped = completed[second].dragged_line.clone();

        // :597
        if clip_to_other_line(world, context, &reference, &mut clipped) {
          completed[second].dragged_line = clipped;
        }
      }
    }

    // :602 to :606
    for line in completed.iter_mut() {
      world.remove_line(last, &mut line.original_line);
      world.add_line(last, &mut line.dragged_line, false);
    }

    // :608
    self.restore_leader_segments(completed);

    true
  }

  // -----------------------------------------------------------------
  // Walkaround mode
  // -----------------------------------------------------------------

  /// Move the whole bundle and bend every line around what is in its way.
  ///
  /// Port of `multidragWalkaround` (`:458`). A walkaround is order
  /// dependent, because the first line walked has the free space and the
  /// last one has to fit around everything already placed, so the set is
  /// walked in **both** orders and the ordering that bends it least wins.
  /// `totalLength` is the sum of the detours, and a tie goes to the
  /// reverse attempt (`:527`).
  ///
  /// # Erratum E17 is transcribed
  ///
  /// The line walked is `aCompletedLines[attempt ? n - 1 - lidx : lidx]`
  /// (`:501`) and its result is stored at `postWalkLines[lidx]` (`:513`),
  /// then written back as
  /// `aCompletedLines[lidx].draggedLine = postWalkLines[lidx]` (`:555`).
  /// For the reverse attempt the two indexings disagree, so whenever it
  /// wins every record receives some **other** line's geometry. The fix
  /// is one index; it is not applied, because the board is unaffected,
  /// the node holds the right shape for every line and only
  /// [`MultiDragger::restore_leader_segments`] reads the scrambled
  /// records, and nothing asks for the repair.
  ///
  /// # Deviation: the pre walk node is dropped
  ///
  /// `preWalkNode` (`:475`) is a local KiCad never deletes, so it leaks
  /// one node per mouse move. Here it is [`MultiDragger::pre_walk_node`],
  /// dropped at the top of the next drag, which takes
  /// [`MultiDragger::last_node`] with it and subsumes the `delete
  /// m_lastNode` at `:461`.
  fn multidrag_walkaround(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    completed: &mut [MdragLine],
  ) -> bool {
    // :461, and the pre walk node of the previous move with it.
    if let Some(previous) = self.pre_walk_node.take() {
      world.drop_node(previous);
      self.last_node = None;
    }

    self.drop_last_node(world);
    // :467 to :472, with erratum E28's tie break; see the module
    // documentation.
    sort_by_drag_distance(completed);

    // :475
    let pre_walk = world.branch(self.world_node);

    self.pre_walk_node = Some(pre_walk);

    // :477 to :481
    for line in completed.iter_mut() {
      world.remove_line(pre_walk, &mut line.original_line);
    }

    let mut attempts: [Option<WalkAttempt>; 2] = [None, None];

    // :493
    for (attempt, slot) in attempts.iter_mut().enumerate() {
      let node = world.branch(pre_walk);
      let mut state = WalkAttempt {
        node,
        total_length: 0,
        post_walk_lines: vec![Line::new(); completed.len()],
      };
      let mut failed = false;

      for position in 0..completed.len() {
        // :501
        let source = if attempt == 1 {
          completed.len() - 1 - position
        } else {
          position
        };
        // :504
        let Some(mut walked) =
          try_walkaround(world, context, node, &completed[source].dragged_line)
        else {
          // :517
          failed = true;

          break;
        };

        // :511 to :513. Erratum E17: the result is stored under
        // `position`, not under `source`.
        world.add_line(node, &mut walked, false);

        state.total_length += walked.shape().length()
          - completed[source].dragged_line.shape().length();
        state.post_walk_lines[position] = walked;
      }

      if failed {
        world.drop_node(node);
      } else {
        *slot = Some(state);
      }
    }

    // :523 to :543. A tie goes to the reverse attempt.
    let best = match (&attempts[0], &attempts[1]) {
      (Some(forward), Some(reverse)) => {
        usize::from(forward.total_length >= reverse.total_length)
      }
      (Some(_), None) => 0,
      (None, Some(_)) => 1,
      // :545. Both orders failed.
      (None, None) => return false,
    };
    let Some(state) = attempts[best].take() else {
      return false;
    };

    // :553 to :556
    for (position, line) in completed.iter_mut().enumerate() {
      if let Some(walked) = state.post_walk_lines.get(position) {
        line.dragged_line = walked.clone();
      }
    }

    // :558, :559
    self.last_node = Some(state.node);

    if let Some(other) = attempts[1 - best].take() {
      world.drop_node(other.node);
    }

    // :567
    self.restore_leader_segments(completed);

    true
  }

  // -----------------------------------------------------------------
  // Shove mode
  // -----------------------------------------------------------------

  /// Move the whole bundle and push what is in the way aside.
  ///
  /// Port of `multidragShove` (`:613`). Every dragged line becomes a
  /// shove **head**, in drag distance order, and the engine does the
  /// rest. This is the one place in KiCad's tree where a caller
  /// deliberately controls the head order (note 04 section 5.4), and
  /// [`ShovePolicy::DONT_OPTIMIZE`] has **no other user** anywhere, so
  /// the multi dragger is the reason that flag exists.
  ///
  /// # The re-add block is a repair, not an optimization
  ///
  /// `:660` to `:676`. [`MultiDragger::start`] removed **every**
  /// `m_mdragLines` entry from the pre shove node, but a posture only
  /// rebuilds the lines that passed its direction check, so without this
  /// the rejected lines would be silently deleted from the board. It is
  /// also why [`MdragLine::mdrag_index`] exists.
  ///
  /// # Erratum E27
  ///
  /// A failed run returns at `:695` **without** calling
  /// `restoreLeaderSegments`, so [`MultiDragger::leader_segments`] keeps
  /// what `tryPosture` put there at `:813`, the pre drag selection. A
  /// host would then re-select items the commit may have replaced.
  /// Transcribed as it stands.
  fn multidrag_shove(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    completed: &mut [MdragLine],
  ) -> bool {
    // :615
    self.drop_last_node(world);

    // :621
    let Some(mut shove) = self.shove.take() else {
      return false;
    };

    // :629, with erratum E28's tie break.
    sort_by_drag_distance(completed);

    // :647 to :654. `SetDefaultShovePolicy` is a no operation (erratum
    // E3) and has no counterpart.
    shove.clear_heads();

    for line in completed.iter() {
      shove.add_head_line(
        line.dragged_line.clone(),
        ShovePolicy::SHOVE | ShovePolicy::DONT_OPTIMIZE,
      );
    }

    // :656, :658
    let status = shove.run(world, context);
    let last = world.branch(shove.current_node());

    self.last_node = Some(last);

    // :660 to :676. Put back the lines the posture dropped.
    for line in &self.mdrag_lines {
      if completed
        .iter()
        .any(|entry| entry.mdrag_index == line.mdrag_index)
      {
        continue;
      }

      let mut preserved = line.original_line.clone();

      preserved.clear_links();
      world.add_line(last, &mut preserved, false);
    }

    // :678
    if status == ShoveStatus::Ok {
      for (index, line) in completed.iter_mut().enumerate() {
        // :684, :685
        if shove.heads_modified(Some(index))
          && let Some(head) = shove.modified_head(index)
        {
          line.dragged_line = head.clone();
        }

        // :687, :688. "this should not be linked (assert in rt-test)".
        line.dragged_line.clear_links();
        // :690
        world.add_line(last, &mut line.dragged_line, false);
      }
    }

    self.shove = Some(shove);

    // :693
    if status != ShoveStatus::Ok {
      // :695, erratum E27: no leader restoration on this path.
      return false;
    }

    // :698
    self.restore_leader_segments(completed);

    true
  }

  // -----------------------------------------------------------------
  // The selection handoff
  // -----------------------------------------------------------------

  /// Rebuild the list of segments the host should re-select.
  ///
  /// Port of `restoreLeaderSegments` (`:429`). In corner mode the leader
  /// is the segment at the dragged end, which after
  /// [`MultiDragger::check_corner_mode`]'s reversal is the **last** link
  /// (`GetLink( -1 )`, `pcbnew/router/pns_link_holder.h:80`, where a
  /// negative index wraps). In segment mode it is whichever segment
  /// [`MultiDragger::find_new_leader_segment`] matched.
  ///
  /// It runs after the lines have been added to the drag node, because
  /// that is what gave the dragged lines their links.
  fn restore_leader_segments(&mut self, completed: &[MdragLine]) {
    // :431
    self.leader_segments.clear();

    for line in completed {
      // :435
      if !line.drag_ok {
        continue;
      }

      if self.drag_mode == DragMode::Corner {
        // :439 to :442. `GetLink( -1 )` is the last link.
        if line.dragged_line.link_count() > 0
          && let Some(link) = line.dragged_line.link_at(-1)
        {
          self.leader_segments.push(link);
        }
      } else if let Some(index) = self.find_new_leader_segment(line) {
        // :448 to :451. The bounds test is KiCad's own, because
        // `findNewLeaderSegment` counts segments and this indexes links.
        if let Some(link) = line.dragged_line.links().get(index) {
          self.leader_segments.push(*link);
        }
      }
    }
  }

  /// Which segment of a dragged line corresponds to the one the user
  /// grabbed.
  ///
  /// Port of `findNewLeaderSegment` (`:407`): the segment the cursor's
  /// perpendicular ray actually crosses, running the same way the grabbed
  /// one did or exactly against it. [`MultiDragger::guide`] is that ray,
  /// built at `:809`, so this is segment mode only.
  ///
  /// [`None`] is KiCad's `-1`. It is also the answer for a leader index
  /// that is out of range of [`MdragLine::pre_drag_line`], which variant
  /// 1's point removal can produce and which KiCad reads out of bounds.
  fn find_new_leader_segment(&self, line: &MdragLine) -> Option<usize> {
    // :409
    let leader = line
      .leader_seg_index
      .filter(|leader| *leader < line.pre_drag_line.segment_count())?;
    let original = line.pre_drag_line.segment(leader);
    // :410
    let original_dir = Direction45::from_seg(&original, false);

    // :412
    for index in 0..line.dragged_line.segment_count() {
      let current = line.dragged_line.segment(index);
      let current_dir = Direction45::from_seg(&current, false);

      // :417, :419
      let Some(intersection) = current.intersect_lines(&self.guide) else {
        continue;
      };

      if !current.contains_point(intersection) {
        continue;
      }

      // :421
      if current_dir == original_dir || current_dir == original_dir.opposite() {
        return Some(index);
      }
    }

    // :426
    None
  }

  // -----------------------------------------------------------------
  // Fix
  // -----------------------------------------------------------------

  /// Commit the drag, or refuse.
  ///
  /// Port of `FixRoute` (`:366`). `ROUTER::FixRoute` drops the point and
  /// the end item on this branch (`pcbnew/router/pns_router.cpp:928`), so
  /// `force_commit` would be the only argument, and note 06 erratum E24
  /// is that **it is ignored**: `MULTI_DRAGGER::FixRoute` tests
  /// `m_dragStatus || Settings().AllowDRCViolations()` and nothing else,
  /// so Ctrl+click cannot force a multi drag commit where it can force a
  /// single one.
  pub fn fix_route(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    force_commit: bool,
  ) -> bool {
    match self.fix_route_node(context, force_commit) {
      Some(node) => {
        world.commit(node);

        true
      }
      None => false,
    }
  }

  /// Which node a fix would commit, or [`None`] when it refuses.
  ///
  /// The decision half of [`MultiDragger::fix_route`], split out for the
  /// same reason [`crate::dragger::Dragger::fix_route_node`] is: the
  /// facade has to build its host facing diff before the node is folded
  /// away.
  ///
  /// `force_commit` is accepted and never read, which is erratum E24; the
  /// argument stays in the signature because one facade method serves
  /// both draggers.
  pub fn fix_route_node(
    &mut self,
    context: &AlgoContext<'_>,
    force_commit: bool,
  ) -> Option<NodeId> {
    let _ = force_commit;

    // :373
    if !self.drag_status && !context.settings.allow_drc_violations() {
      return None;
    }

    // :377
    Some(self.current_node())
  }

  // -----------------------------------------------------------------
  // Helpers
  // -----------------------------------------------------------------

  /// Throw the previous move's answer away.
  ///
  /// The `if( m_lastNode ) { delete m_lastNode; m_lastNode = nullptr; }`
  /// every mode routine opens with (`:461`, `:577`, `:615`).
  fn drop_last_node(&mut self, world: &mut World) {
    if let Some(last) = self.last_node.take() {
      world.drop_node(last);
    }
  }
}

// ---------------------------------------------------------------------
// The per line drag outcome
// ---------------------------------------------------------------------

/// What one parallel line's drag produced in a posture attempt.
///
/// The three ways out of the per line body of `tryPosture` (`:817` to
/// `:929`): the angle gate rejected the line and it keeps its original
/// geometry, `DragCorner` collapsed it to a point and the whole iteration
/// is skipped with a `continue` (`:872`), or it dragged.
enum ParallelDrag {
  /// The angle gate rejected the line (`:843`, `:891`). `dragOK` stays
  /// false and the line never reaches `completed`.
  Skipped,
  /// `DragCorner` collapsed the line to a single point (`:871`), so the
  /// `continue` skips the primary push as well.
  Collapsed,
  /// The line dragged. For the primary in segment mode the geometry is
  /// unused, because `:933` overwrites it with `primaryDragged`. Boxed
  /// because a [`Line`] is forty times an empty variant.
  Dragged(Box<Line>),
}

// ---------------------------------------------------------------------
// One walkaround attempt
// ---------------------------------------------------------------------

/// The state of one of the two orderings `multidragWalkaround` tries.
///
/// Port of the local `WALK_STATE` struct (`:483`), minus its `fail` flag,
/// which is an [`Option`] around the whole state here.
struct WalkAttempt {
  /// The branch this attempt built.
  node: NodeId,
  /// The sum of the detours, `int` in KiCad and `i64` here because
  /// [`LineChain::length`] is.
  total_length: i64,
  /// The walked shape per line, indexed by **position in the sorted
  /// set** and not by the line it came from; see erratum E17 on
  /// [`MultiDragger::multidrag_walkaround`].
  post_walk_lines: Vec<Line>,
}

// ---------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------

/// Order the set the way the walkaround and the shove want it.
///
/// `std::sort` over `compareDragStartDist` (`:467`, `:629`), which
/// compares [`MdragLine::drag_dist`] alone. The tie break on
/// [`MdragLine::mdrag_index`] is this module's one deviation, note 06
/// erratum E28: `dragDist` is never assigned in corner mode, so KiCad's
/// unstable sort leaves the order of a corner mode bundle unspecified,
/// and both callers' results depend on it.
fn sort_by_drag_distance(lines: &mut [MdragLine]) {
  lines.sort_by_key(|line| (line.drag_dist, line.mdrag_index));
}

/// Drop a line's last point.
///
/// The `Line().Remove( -1 )` of `:744`, `:745` and `:749`. KiCad's
/// `Remove( int )` adds `PointCount()` to a negative index once and then
/// bounds checks, so an empty chain is left alone; that is what
/// [`LineChain::normalize_index`] answers [`None`] for.
fn remove_last_point(line: &mut Line) {
  let chain = line.chain_mut();

  if let Some(index) = chain.normalize_index(-1) {
    chain.remove(index);
  }
}

/// The last segment of a line, or [`None`] when it has none.
///
/// KiCad's `CSegment( -1 )`
/// (`libs/kimath/include/geometry/shape_line_chain.h:447`), which wraps
/// the index once and then reads out of range on a chain with fewer than
/// two points. Every call site here guards instead.
fn last_segment(line: &Line) -> Option<Seg> {
  line
    .shape()
    .segment_count()
    .checked_sub(1)
    .map(|index| line.shape().segment(index))
}

/// The first selected segment with an endpoint exactly at a point.
///
/// Port of `ITEM_SET::FindVertex`
/// (`pcbnew/router/pns_itemset.cpp:137`), whose own comment is "fixme:
/// biconnected concept". It is what makes a line a corner candidate: the
/// user selected a segment that touches this line's end.
fn find_vertex(world: &World, items: &[ItemId], at: Vec2) -> Option<ItemId> {
  items.iter().copied().find(|item| {
    world.item(*item).is_some_and(|stored| match stored.body() {
      ItemBody::Segment(body) => {
        let seg = body.seg();

        seg.a == at || seg.b == at
      }
      _ => false,
    })
  })
}

/// Walk one line around whatever is in its way, or fail.
///
/// Port of `MULTI_DRAGGER::tryWalkaround` (`:384`), which differs from
/// the single dragger's (`pcbnew/router/pns_dragger.cpp:685`) in exactly
/// one number: the length limit factor is
/// [`WALKAROUND_LENGTH_LIMIT_FACTOR`] rather than 30.0.
///
/// [`None`] is KiCad's false, where the caller's `aWalk` is left holding
/// `aOrig` (`:394`) and the caller treats the attempt as failed anyway.
fn try_walkaround(
  world: &mut World,
  context: &AlgoContext<'_>,
  node: NodeId,
  original: &Line,
) -> Option<Line> {
  // :386
  let mut walkaround = Walkaround::new(node, context.settings);

  // :387
  walkaround.set_solids_only(false);
  // :390
  walkaround.set_iteration_limit(context.settings.walkaround_iteration_limit);
  // :391
  walkaround.set_length_limit(true, WALKAROUND_LENGTH_LIMIT_FACTOR);
  // :392
  walkaround.set_allowed_policies(&[WalkPolicy::Shortest]);

  // :396
  let result = walkaround.route(world, context, original);

  // :398 to :402
  (result.status(WalkPolicy::Shortest) == WalkaroundStatus::Done)
    .then(|| result.into_line(WalkPolicy::Shortest))
}

/// Shorten one line until it stops touching another.
///
/// Port of `clipToOtherLine` (`:294`), a binary search on **arc length**
/// for the longest prefix of `clipped` that does not collide with
/// `reference`. It is the only thing mark obstacles mode does about two
/// dragged lines running into each other, and it is the one caller in the
/// dragger of a line against line collision, which
/// [`World::collide_lines`] provides by decomposing the obstacle side
/// into segments.
///
/// The search halves the step every time it changes direction and stops
/// once the step falls to [`CLIP_LENGTH_THRESHOLD`], or immediately when
/// the very first probe is clear (`:338`), which is the common case.
///
/// # Erratum E25 is transcribed
///
/// If the first probe collides and so does every later one, `tightest` is
/// never assigned and `aClipped.SetShape( tightest )` (`:343`) installs
/// an **empty** chain while the answer is still true, so
/// `multidragMarkObstacles` stores an empty line at `:598`. That is
/// reproduced. Its other half, `int curL = l.CLine().Length()` narrowing
/// a `long long` at `:307`, cannot be inherited:
/// [`LineChain::length`] and [`LineChain::point_along`] are both `i64`
/// here.
///
/// KiCad's `Split` answers `-1` for a point it cannot place, and
/// `Slice( 0, -1 )` then names the whole chain; [`LineChain::split`]
/// answers [`None`] for the same case and the fallback below is that
/// whole chain.
///
/// KiCad's `aNode` (`:294`) reaches `LINE::Collide` only so that
/// `collideSimple` can find the rule resolver
/// (`pcbnew/router/pns_item.cpp:133`); the resolver is a parameter of
/// [`World::collide_lines`] here, so there is no node argument.
fn clip_to_other_line(
  world: &World,
  context: &AlgoContext<'_>,
  reference: &Line,
  clipped: &mut Line,
) -> bool {
  // :303, :304
  let mut probe = clipped.clone();
  let mut tightest = LineChain::new();
  // :306 to :308
  let mut did_clip = false;
  let mut current_length = clipped.shape().length();
  let mut step = current_length / 2 - 1;

  // :310
  while step > CLIP_LENGTH_THRESHOLD {
    // :312 to :315
    let mut candidate = clipped.shape().clone();
    let Some(split_at) = candidate.point_along(current_length) else {
      break;
    };
    let end = candidate
      .split(split_at)
      .or_else(|| candidate.point_count().checked_sub(1));
    let Some(end) = end else {
      break;
    };
    let Ok(candidate) = candidate.slice(0, end) else {
      break;
    };

    // :317
    probe.set_shape(candidate.clone());

    // :321. `l.Collide( &aRef, ... )` has the probe as `this` and the
    // reference as the head, which is the way round `World::collide_lines`
    // takes them.
    let hit = world
      .collide_lines(
        &probe,
        reference,
        context.resolver,
        &CollisionSearchOptions {
          limit_count: Some(1),
          ..CollisionSearchOptions::default()
        },
      )
      .is_some();

    if hit {
      // :323 to :325
      did_clip = true;
      current_length -= step;
      step /= 2;
    } else {
      // :329
      tightest = candidate;

      if did_clip {
        // :333, :334
        current_length += step;
        step /= 2;
      } else {
        // :338. The line was clear at full length.
        break;
      }
    }
  }

  // :343, unconditional in KiCad. Erratum E25: an all colliding search
  // never assigned `tightest`, so this installs an **empty** chain and
  // still answers true. On the false path the caller discards the line
  // (`:597`), which is what makes the unconditional write harmless there.
  clipped.set_shape(tightest);

  did_clip
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::seg::Seg;
  use crate::geometry::vec2::Vec2;
  use crate::item::{LayerRange, Segment};
  use crate::rules::FixedClearance;
  use crate::settings::RoutingSettings;

  /// A world holding one segment, whose handle stands in for the
  /// selected primitive a record carries.
  ///
  /// [`ItemId`] has no default and no public constructor, so a unit test
  /// that needs one has to store something.
  fn world_with_one_segment() -> (World, ItemId) {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let body = crate::item::ItemBody::Segment(Segment::new(
      Seg::new(Vec2::new(0, 0), Vec2::new(1_000_000, 0)),
      200_000,
    ));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));

    let id = world
      .add_segment(root, item, false)
      .expect("a fresh segment is neither degenerate nor redundant");

    (world, id)
  }

  /// A chain out of its points.
  fn chain_of(points: &[Vec2]) -> LineChain {
    let mut chain = LineChain::new();

    for point in points {
      chain.append(*point);
    }

    chain
  }

  /// A line with a shape and a width.
  fn line_of(points: &[Vec2], width: i32) -> Line {
    let mut line = Line::new();

    line.set_width(width);
    line.set_shape(chain_of(points));

    line
  }

  /// A record with a shape and a leader index.
  fn record(
    points: &[Vec2],
    leader: Option<usize>,
    index: usize,
    seed: ItemId,
  ) -> MdragLine {
    let line = line_of(points, 200_000);
    let mut mdrag = MdragLine::new(line.clone(), seed, index);

    mdrag.leader_seg_index = leader;
    mdrag.pre_drag_line = line.clone();
    mdrag.dragged_line = line;

    mdrag
  }

  #[test]
  fn removing_the_last_point_leaves_a_short_line_alone() {
    // `Line().Remove( -1 )` at `:744`, whose bounds check is what makes an
    // empty chain safe.
    let mut empty = Line::new();

    remove_last_point(&mut empty);
    assert_eq!(empty.point_count(), 0);

    let mut two = line_of(&[Vec2::new(0, 0), Vec2::new(100, 0)], 100);

    remove_last_point(&mut two);
    assert_eq!(two.point_count(), 1);
  }

  #[test]
  fn the_last_segment_of_a_degenerate_line_is_nothing() {
    // KiCad's `CSegment( -1 )` reads out of range here.
    assert_eq!(last_segment(&Line::new()), None);
    assert_eq!(last_segment(&line_of(&[Vec2::new(0, 0)], 100)), None);
    assert_eq!(
      last_segment(&line_of(
        &[Vec2::new(0, 0), Vec2::new(100, 0), Vec2::new(100, 100)],
        100
      )),
      Some(Seg::new(Vec2::new(100, 0), Vec2::new(100, 100)))
    );
  }

  #[test]
  fn the_sort_ties_on_the_line_index() {
    // Erratum E28's deviation: `dragDist` is never assigned in corner
    // mode, so every line compares equal and the input order has to
    // decide.
    let (_world, seed) = world_with_one_segment();
    let points = [Vec2::new(0, 0), Vec2::new(100, 0)];
    let mut lines = vec![
      record(&points, Some(0), 2, seed),
      record(&points, Some(0), 0, seed),
      record(&points, Some(0), 1, seed),
    ];

    sort_by_drag_distance(&mut lines);

    assert_eq!(
      lines
        .iter()
        .map(|line| line.mdrag_index)
        .collect::<Vec<_>>(),
      vec![0, 1, 2]
    );

    lines[0].drag_dist = 10;
    lines[1].drag_dist = -10;
    sort_by_drag_distance(&mut lines);

    assert_eq!(
      lines
        .iter()
        .map(|line| line.mdrag_index)
        .collect::<Vec<_>>(),
      vec![1, 2, 0]
    );
  }

  #[test]
  fn a_leader_index_out_of_range_finds_no_new_leader() {
    // Variant 1 drops a point, which can leave `leaderSegIndex` past the
    // end; KiCad reads out of bounds there.
    let (world, seed) = world_with_one_segment();
    let mut dragger = MultiDragger::new(&world, world.root());

    dragger.drag_mode = DragMode::Segment;
    dragger.guide = Seg::new(Vec2::new(50, -100), Vec2::new(50, 100));

    let points = [Vec2::new(0, 0), Vec2::new(100, 0)];
    let line = record(&points, Some(4), 0, seed);

    assert_eq!(dragger.find_new_leader_segment(&line), None);

    let line = record(&points, None, 0, seed);

    assert_eq!(dragger.find_new_leader_segment(&line), None);
  }

  #[test]
  fn the_new_leader_is_the_segment_the_guide_crosses() {
    // `:417` to `:422`: the guide has to cross the segment, and the
    // segment has to run the same way as the grabbed one or against it.
    let (world, seed) = world_with_one_segment();
    let mut dragger = MultiDragger::new(&world, world.root());

    dragger.drag_mode = DragMode::Segment;
    // A vertical ray through x = 250, which crosses the second leg only.
    dragger.guide = Seg::new(Vec2::new(250, -1000), Vec2::new(250, 1000));

    let mut line = record(
      &[Vec2::new(0, 0), Vec2::new(100, 0), Vec2::new(200, 100)],
      Some(0),
      0,
      seed,
    );

    line.dragged_line = line_of(
      &[
        Vec2::new(0, 0),
        Vec2::new(100, 100),
        Vec2::new(300, 100),
        Vec2::new(400, 200),
      ],
      200_000,
    );

    // Segment 1 runs due east like the grabbed one and the ray crosses it.
    assert_eq!(dragger.find_new_leader_segment(&line), Some(1));
  }

  #[test]
  fn a_fresh_multi_dragger_reports_the_untouched_board() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let dragger = MultiDragger::new(&world, root);

    assert_eq!(dragger.current_node(), root);
    assert!(dragger.traces().is_empty());
    assert!(dragger.last_committed_leader_segments().is_empty());
    assert!(dragger.current_nets().is_empty());
    assert_eq!(dragger.current_layer(), 0);
    // Erratum E23: the first half is always false.
    assert_eq!(dragger.force_mark_obstacles_mode(), (false, false));
  }

  #[test]
  fn an_empty_selection_refuses_to_start() {
    // `:52`, which `ROUTER::StartDragging` already refuses at `:171`.
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let rules = FixedClearance::uniform(0);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let mut dragger = MultiDragger::new(&world, root);

    assert!(!dragger.start(&mut world, &context, Vec2::new(0, 0), &[]));
  }
}
