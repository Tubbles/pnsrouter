// SPDX-License-Identifier: GPL-3.0-or-later

//! Differential pairs: the coupling geometry, the gateways and the fit.
//!
//! A port of `pcbnew/router/pns_diff_pair.h` and
//! `pcbnew/router/pns_diff_pair.cpp`, the value layer underneath KiCad's
//! `DIFF_PAIR_PLACER`. Nothing here is ever stored in a
//! [`crate::node::World`]: `DIFF_PAIR::Clone()` is `assert( false )`
//! (`pns_diff_pair.h:338`), which is the strongest statement in the file
//! that a pair is a value and not a board object. The placer commits the
//! two [`crate::line::Line`]s a pair holds and throws the pair away.
//!
//! The reference note is `doc/reference/kicad/07-differential-pairs.md`,
//! sections 1 to 4 and errata E1, E4 to E7, E10, E17 and E18.
//!
//! # What is implemented
//!
//! - [`GapConstraint`], KiCad's `RANGED_NUM<int>` (`ranged_num.h:25`).
//! - [`common_parallel_projection`] and the three coupling measurements
//!   [`DiffPair::coupled_segment_pairs`], [`DiffPair::coupled_length`],
//!   [`DiffPair::coupled_length_of_chains`] and [`DiffPair::skew`].
//! - [`DiffPair`] itself: the two chains, the widths, the nets, the two
//!   gap meanings, the end vias and the two [`crate::line::Line`] views.
//! - [`DpGateway`], [`DpPrimitivePair`] and [`DpGateways`] with every
//!   builder KiCad calls: [`DpGateways::build_generic`],
//!   [`DpGateways::build_for_cursor`],
//!   [`DpGateways::build_from_primitive_pair`] (and through it
//!   `buildDpContinuation` and `buildEntries`) and
//!   [`DpGateways::filter_by_orientation`].
//! - The fit: [`DiffPair::build_initial`], [`check_connection_angle`] and
//!   [`fit_gateways`].
//! - [`DiffPair::ending_primitives`], which is what a fixed leg hands the
//!   next one, over the [`DpPrimitive`] that lets a primitive pair name
//!   an object no node holds.
//!
//! # What is not
//!
//! - The placer itself, which is
//!   [`crate::placer::diff_pair_placer::DiffPairPlacer`]. The pair's
//!   optimizer passes live with the rest of the optimizer, at
//!   [`crate::optimizer::Optimizer::optimize_diff_pair`].
//! - `DP_GATEWAYS::BuildOrthoProjections` (`pns_diff_pair.cpp:302`), which
//!   has no caller anywhere in KiCad's tree and whose ortho mode flag the
//!   pair placer stores and never reads (note 07 errata E1 and E13).
//! - The dead members and methods erratum E18 collects: `DIFF_PAIR`'s
//!   `m_viaGap`, `m_maxUncoupledLength` and `m_chamferLimit`, which are
//!   assigned in six places and read in none; `Clear`, `Append`, `Empty`,
//!   `TotalLength`, `CoupledLengthFactor` and the single segment
//!   `CoupledLength( const SEG&, const SEG& )` overload; `DP_GATEWAYS`'s
//!   `Clear` and `CGateways`; `DP_PRIMITIVE_PAIR::dump`.
//! - `DP_PRIMITIVE_PAIR`'s owned clones and therefore its leaking
//!   assignment operator (erratum E17): a pair here holds
//!   [`crate::item::ItemId`] handles into the arena.
//!
//! # The two meanings of the gap
//!
//! The single most confusing thing in the KiCad file, and the reason
//! [`DiffPair::gap`] carries a warning of its own and
//! [`crate::settings::Sizes::diff_pair_pitch`] exists. `DIFF_PAIR::m_gap`
//! holds the **centre to centre** pitch while a route is being fitted
//! (`pns_diff_pair_placer.cpp:731`, and every gateway builder spaces its
//! anchors by it) and the **edge to edge** copper gap once the fit
//! succeeded (`:741`), which is what [`DiffPair::coupled_segment_pairs`]
//! matches against because it subtracts the width first. Both quantities
//! are an `int` called `aGap` in C++; here the centre to centre one is
//! always called a pitch, on [`DpGateways::new`], on [`fit_gateways`] and
//! on [`crate::settings::Sizes::diff_pair_pitch`].

use std::borrow::Cow;
use std::f64::consts::{FRAC_1_SQRT_2, SQRT_2};

use crate::geometry::direction45::{AngleType, CornerMode, Direction45};
use crate::geometry::line_chain::LineChain;
use crate::geometry::math::{kiround, rescale};
use crate::geometry::seg::Seg;
use crate::geometry::shape::Shape;
use crate::geometry::vec2::{Vec2, Vec2L};
use crate::item::{Item, ItemBody, ItemId, Kind, LayerRange, NetId};
use crate::line::Line;
use crate::node::World;

// ---------------------------------------------------------------------
// GapConstraint
// ---------------------------------------------------------------------

/// A gap with an asymmetric tolerance.
///
/// Port of `RANGED_NUM<int>`, `pcbnew/router/ranged_num.h:25`, at the one
/// instantiation the router uses. KiCad's template has a non const
/// `operator T()` that yields the bare value, which is how
/// `checkGap( p, n, m_gapConstraint )` (`pns_diff_pair.cpp:251`) throws
/// the tolerance away at that call site; here that is the explicit
/// [`GapConstraint::value`].
///
/// The two ways a pair acquires one are deliberately different and a port
/// that collapses them changes what counts as coupled:
/// [`DiffPair::with_gap_constraint`] and [`DiffPair::from_chains`] leave
/// both tolerances at zero (`pns_diff_pair.h:277`, `:295`), while
/// [`DiffPair::set_gap`] gives plus or minus 10 micrometres
/// (`pns_diff_pair.h:434`).
///
/// [`GapConstraint::matches`] takes an `i64` because all four of KiCad's
/// call sites hand it an `int64_t` that narrows at the call
/// (`pns_diff_pair.cpp:857`, `:883`, `:931`,
/// `pns_optimizer.cpp:1338`); the narrowing is inert for board sized
/// distances, and comparing in `i64` removes the question.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct GapConstraint {
  /// The gap itself, KiCad's `m_value`.
  value: i32,
  /// How far above [`GapConstraint::value`] still matches, KiCad's
  /// `m_tolerancePlus`.
  tolerance_plus: i32,
  /// How far below [`GapConstraint::value`] still matches, KiCad's
  /// `m_toleranceMinus`.
  tolerance_minus: i32,
}

impl GapConstraint {
  /// The tolerance `SetGap` gives a pair, 10 micrometres each way.
  ///
  /// The literal of `DIFF_PAIR::SetGap`,
  /// `pcbnew/router/pns_diff_pair.h:437`.
  pub const SET_GAP_TOLERANCE: i32 = 10000;

  /// A gap that matches only itself.
  ///
  /// Port of `RANGED_NUM::operator=( const T )`,
  /// `pcbnew/router/ranged_num.h:38`, which writes the value and leaves
  /// both tolerances where the default constructor put them, at zero.
  pub const fn exact(value: i32) -> Self {
    Self {
      value,
      tolerance_plus: 0,
      tolerance_minus: 0,
    }
  }

  /// A gap with a tolerance each way.
  ///
  /// Port of `RANGED_NUM( T, T, T )`,
  /// `pcbnew/router/ranged_num.h:27`.
  pub const fn with_tolerance(
    value: i32,
    tolerance_plus: i32,
    tolerance_minus: i32,
  ) -> Self {
    Self {
      value,
      tolerance_plus,
      tolerance_minus,
    }
  }

  /// The gap itself, tolerance discarded.
  ///
  /// Port of `RANGED_NUM::operator T()`,
  /// `pcbnew/router/ranged_num.h:33`.
  pub const fn value(&self) -> i32 {
    self.value
  }

  /// The upper tolerance.
  pub const fn tolerance_plus(&self) -> i32 {
    self.tolerance_plus
  }

  /// The lower tolerance.
  pub const fn tolerance_minus(&self) -> i32 {
    self.tolerance_minus
  }

  /// Whether a measured gap falls inside the tolerance band.
  ///
  /// Port of `RANGED_NUM::Matches`,
  /// `pcbnew/router/ranged_num.h:44`. The band is closed at both ends.
  pub fn matches(&self, other: i64) -> bool {
    other >= i64::from(self.value) - i64::from(self.tolerance_minus)
      && other <= i64::from(self.value) + i64::from(self.tolerance_plus)
  }
}

// ---------------------------------------------------------------------
// The coupling geometry
// ---------------------------------------------------------------------

/// The parts of two near parallel segments that face each other.
///
/// Port of `commonParallelProjection`,
/// `pcbnew/router/pns_diff_pair.cpp:786`, whose two out parameters become
/// the two segments of the answer: the clip of `p`, then the clip of `n`.
/// `None` is KiCad's `false`, meaning the projection of `n` onto `p`'s
/// line does not overlap `p` at all.
///
/// The routine parameterises `p` by `SEG::TCoef`, projects both
/// endpoints of `n` onto `p`'s line, and takes the two middle order
/// statistics of the four resulting parameters as the overlap. KiCad
/// finds those two by sorting a four element vector with the comment
/// "awful and disgusting way of finding 2 midpoints" (`:810`); a sort of
/// four is kept here because at four elements nothing else is clearer.
///
/// Two behaviours a caller has to know. The early rejections at `:802`
/// and `:805` use swapped copies of the parameters, while the sorted
/// array at `:808` is rebuilt from scratch, so the swaps do not carry.
/// And the clip of `n` is the projection of the clip of `p` onto `n`'s
/// **line**, not onto `n` itself (`:821`), so it can fall outside `n`
/// when the two are not truly parallel; every caller has already tested
/// [`Seg::approx_parallel`], which bounds how far outside.
pub fn common_parallel_projection(p: Seg, n: Seg) -> Option<(Seg, Seg)> {
  // :788
  let n_projected = Seg::new(p.line_project(n.a), p.line_project(n.b));

  // :790 to :794
  let mut t_a = 0;
  let mut t_b = p.t_coef(p.b);
  let mut t_projected_a = p.t_coef(n_projected.a);
  let mut t_projected_b = p.t_coef(n_projected.b);

  // :796, :799
  if t_b < t_a {
    std::mem::swap(&mut t_a, &mut t_b);
  }

  if t_projected_b < t_projected_a {
    std::mem::swap(&mut t_projected_a, &mut t_projected_b);
  }

  // :802, :805
  if t_b <= t_projected_a || t_a >= t_projected_b {
    return None;
  }

  // :808. Rebuilt from the unswapped values, exactly as KiCad rebuilds
  // it.
  let mut t = [
    0,
    p.t_coef(p.b),
    p.t_coef(n_projected.a),
    p.t_coef(n_projected.b),
  ];
  t.sort_unstable();

  // :812 to :819
  let length_squared = p.squared_length();
  let direction = p.b.widening_sub(p.a);

  let along = |parameter: i64| {
    Vec2L::new(
      i64::from(p.a.x) + rescale(direction.x, parameter, length_squared),
      i64::from(p.a.y) + rescale(direction.y, parameter, length_squared),
    )
    .saturating_to_vec2()
  };

  let p_clip = Seg::new(along(t[1]), along(t[2]));

  // :821, :822
  let n_clip = Seg::new(n.line_project(p_clip.a), n.line_project(p_clip.b));

  Some((p_clip, n_clip))
}

/// One record of two segments running alongside each other.
///
/// Port of `DIFF_PAIR::COUPLED_SEGMENTS`,
/// `pcbnew/router/pns_diff_pair.h:236`. The two indices are into the
/// **unsimplified** chains of the pair that produced the record, which is
/// why `DIFF_PAIR::CoupledSegmentPairs` carries the comment "Do not
/// simplify the line chains here, otherwise the indices will be invalid"
/// (`pns_diff_pair.cpp:838`) and why the optimizer's `mergeDpStep`
/// simplifies its own copies instead (`pns_optimizer.cpp:1473`).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct CoupledSegments {
  /// The part of the P segment that faces N.
  pub coupled_p: Seg,
  /// The part of the N segment that faces P.
  pub coupled_n: Seg,
  /// The whole P segment the clip came from.
  pub parent_p: Seg,
  /// The whole N segment the clip came from.
  pub parent_n: Seg,
  /// Which segment of the P chain [`CoupledSegments::parent_p`] is.
  pub index_p: usize,
  /// Which segment of the N chain [`CoupledSegments::parent_n`] is.
  pub index_n: usize,
}

// ---------------------------------------------------------------------
// DiffPair
// ---------------------------------------------------------------------

/// Two coupled line chains, their nets, their width and their gap.
///
/// Port of `DIFF_PAIR`, `pcbnew/router/pns_diff_pair.h:234`. The two
/// chains are the authoritative geometry; [`DiffPair::p_line`] and
/// [`DiffPair::n_line`] are views built from them.
///
/// # The gap
///
/// [`DiffPair::gap`] is whatever the last writer meant by it, and the two
/// writers mean different things. See the module documentation: during a
/// fit it is the centre to centre pitch, after a fit it is the edge to
/// edge copper gap, and only the second is right for
/// [`DiffPair::coupled_segment_pairs`], which subtracts the width from
/// the centreline distance before matching.
///
/// # Deviations from `DIFF_PAIR`
///
/// - `PLine()` and `NLine()` are lazily rebuilt cached members in KiCad
///   (`pns_diff_pair.h:488`, `:496`) whose `IsLinked()` guard never holds,
///   because nothing in the placer ever links them: `NODE::Add` links a
///   copy (`pns_diff_pair_placer.cpp:850`). They are constructions here,
///   which is what they already are in C++, so a caller that needs one
///   twice keeps it rather than asking twice.
/// - The end vias are [`Item`]s rather than `VIA` values, because a via
///   in this crate owns no hole: the hole is a separate arena item. So
///   `AppendVias`'s `SetHole( aVia.Hole()->Clone() )` (`:449`, `:452`)
///   has no counterpart.
/// - `m_viaGap`, `m_maxUncoupledLength` and `m_chamferLimit` are not
///   carried; erratum E18.
#[derive(Clone, Debug, Default)]
pub struct DiffPair {
  /// The P lane. Port of `m_p`, `pcbnew/router/pns_diff_pair.h:558`.
  p: LineChain,
  /// The N lane. Port of `m_n`, `pcbnew/router/pns_diff_pair.h:558`.
  n: LineChain,
  /// The P end via, valid while [`DiffPair::has_vias`] is set. Port of
  /// `m_via_p`, `pcbnew/router/pns_diff_pair.h:560`.
  via_p: Option<Item>,
  /// The N end via. Port of `m_via_n`.
  via_n: Option<Item>,
  /// Whether the pair ends with vias. Port of `m_hasVias`,
  /// `pcbnew/router/pns_diff_pair.h:562`.
  has_vias: bool,
  /// The P net. Port of `m_net_p`,
  /// `pcbnew/router/pns_diff_pair.h:563`.
  net_p: Option<NetId>,
  /// The N net. Port of `m_net_n`.
  net_n: Option<NetId>,
  /// The width of one lane. Port of `m_width`,
  /// `pcbnew/router/pns_diff_pair.h:564`.
  width: i32,
  /// The gap, in whichever of its two meanings the last writer had. Port
  /// of `m_gap`, `pcbnew/router/pns_diff_pair.h:565`.
  gap: i32,
  /// The gap the coupling tests match against. Port of
  /// `m_gapConstraint`, `pcbnew/router/pns_diff_pair.h:569`.
  gap_constraint: GapConstraint,
  /// The copper layers both lanes lie on. Port of `ITEM::m_layers`,
  /// which `DIFF_PAIR` inherits and the placer writes
  /// (`pns_diff_pair_placer.cpp:732`).
  layers: LayerRange,
}

impl DiffPair {
  /// An empty pair.
  ///
  /// Port of `DIFF_PAIR()`, `pcbnew/router/pns_diff_pair.h:259`, which
  /// zeroes every scalar by hand against the static analyser.
  pub fn new() -> Self {
    Self::default()
  }

  /// An empty pair whose coupling tests match one exact value.
  ///
  /// Port of `DIFF_PAIR( int aGap )`,
  /// `pcbnew/router/pns_diff_pair.h:273`. Note what it does **not** do:
  /// the argument reaches [`DiffPair::gap_constraint`] alone, through
  /// `RANGED_NUM::operator=`, so both tolerances stay zero and
  /// [`DiffPair::gap`] stays zero as well. [`fit_gateways`] builds every
  /// candidate this way with the centre to centre pitch, which is what
  /// makes `checkGap` a pitch test rather than a copper gap test.
  pub fn with_gap_constraint(value: i32) -> Self {
    Self {
      gap_constraint: GapConstraint::exact(value),
      ..Self::default()
    }
  }

  /// A pair of two chains, with an exact gap constraint.
  ///
  /// Port of
  /// `DIFF_PAIR( const SHAPE_LINE_CHAIN&, const SHAPE_LINE_CHAIN&, int )`,
  /// `pcbnew/router/pns_diff_pair.h:289`. The same tolerance rule as
  /// [`DiffPair::with_gap_constraint`] applies.
  pub fn from_chains(p: LineChain, n: LineChain, gap: i32) -> Self {
    Self {
      p,
      n,
      gap_constraint: GapConstraint::exact(gap),
      ..Self::default()
    }
  }

  /// A pair of two lines, taking their nets.
  ///
  /// Port of `DIFF_PAIR( const LINE&, const LINE&, int )`,
  /// `pcbnew/router/pns_diff_pair.h:307`, the only constructor that sets
  /// the nets (`:314`, `:315`). It does not take the widths or the
  /// layers, which stay zero and undefined.
  pub fn from_lines(line_p: &Line, line_n: &Line, gap: i32) -> Self {
    Self {
      p: line_p.shape().clone(),
      n: line_n.shape().clone(),
      net_p: line_p.net(),
      net_n: line_n.net(),
      gap_constraint: GapConstraint::exact(gap),
      ..Self::default()
    }
  }

  /// The P lane. Port of `CP`, `pcbnew/router/pns_diff_pair.h:530`.
  pub const fn chain_p(&self) -> &LineChain {
    &self.p
  }

  /// The N lane. Port of `CN`, `pcbnew/router/pns_diff_pair.h:531`.
  pub const fn chain_n(&self) -> &LineChain {
    &self.n
  }

  /// Replace both lanes, optionally swapping which is which.
  ///
  /// Port of
  /// `SetShape( const SHAPE_LINE_CHAIN&, const SHAPE_LINE_CHAIN&, bool )`,
  /// `pcbnew/router/pns_diff_pair.h:399`. Nothing else on the pair
  /// changes, the widths of the chains included.
  pub fn set_shape(&mut self, p: LineChain, n: LineChain, swap_lanes: bool) {
    if swap_lanes {
      self.p = n;
      self.n = p;
    } else {
      self.p = p;
      self.n = n;
    }
  }

  /// Take another pair's two lanes and nothing else.
  ///
  /// Port of `SetShape( const DIFF_PAIR& )`,
  /// `pcbnew/router/pns_diff_pair.h:413`. The gap, the width, the nets
  /// and the vias of the destination survive, which is exactly what
  /// `tryWalkDp` relies on (`pns_diff_pair_placer.cpp:313`).
  pub fn set_shape_from(&mut self, other: &DiffPair) {
    self.p = other.p.clone();
    self.n = other.n.clone();
  }

  /// The nets. Port of `NetP` and `NetN`,
  /// `pcbnew/router/pns_diff_pair.h:479`, `:484`.
  pub const fn nets(&self) -> (Option<NetId>, Option<NetId>) {
    (self.net_p, self.net_n)
  }

  /// Set both nets. Port of `SetNets`,
  /// `pcbnew/router/pns_diff_pair.h:419`.
  pub const fn set_nets(&mut self, net_p: Option<NetId>, net_n: Option<NetId>) {
    self.net_p = net_p;
    self.net_n = net_n;
  }

  /// The width of one lane. Port of `Width`,
  /// `pcbnew/router/pns_diff_pair.h:431`.
  pub const fn width(&self) -> i32 {
    self.width
  }

  /// Set the width of one lane, and of both chains.
  ///
  /// Port of `SetWidth`, `pcbnew/router/pns_diff_pair.h:425`, which
  /// writes the width onto the two chains as well.
  pub fn set_width(&mut self, width: i32) {
    self.width = width;
    self.p.set_width(width);
    self.n.set_width(width);
  }

  /// The gap, in whichever meaning the last writer had. Port of `Gap`,
  /// `pcbnew/router/pns_diff_pair.h:440`.
  pub const fn gap(&self) -> i32 {
    self.gap
  }

  /// Set the gap and give the coupling tests a 10 micrometre tolerance
  /// each way.
  ///
  /// Port of `SetGap`, `pcbnew/router/pns_diff_pair.h:434`. This is the
  /// **only** path that widens the constraint: a pair that was merely
  /// constructed matches its gap exactly. Any port that collapses the two
  /// changes what `CoupledSegmentPairs` answers.
  pub fn set_gap(&mut self, gap: i32) {
    self.gap = gap;
    self.gap_constraint = GapConstraint::with_tolerance(
      gap,
      GapConstraint::SET_GAP_TOLERANCE,
      GapConstraint::SET_GAP_TOLERANCE,
    );
  }

  /// The gap the coupling tests match against.
  ///
  /// Port of `GapConstraint`,
  /// `pcbnew/router/pns_diff_pair.h:541`.
  pub const fn gap_constraint(&self) -> GapConstraint {
    self.gap_constraint
  }

  /// The copper layers. Port of `ITEM::Layers`,
  /// `pcbnew/router/pns_item.h:212`.
  pub const fn layers(&self) -> LayerRange {
    self.layers
  }

  /// Set the copper layers. Port of `ITEM::SetLayers`,
  /// `pcbnew/router/pns_item.h:213`.
  pub const fn set_layers(&mut self, layers: LayerRange) {
    self.layers = layers;
  }

  /// Collapse the pair onto one layer.
  ///
  /// Port of `ITEM::SetLayer`, `pcbnew/router/pns_item.h:215`, which the
  /// placer calls once per move (`pns_diff_pair_placer.cpp:732`).
  pub const fn set_layer(&mut self, layer: i32) {
    self.layers = LayerRange::single(layer);
  }

  // -------------------------------------------------------------------
  // Vias
  // -------------------------------------------------------------------

  /// Whether the pair ends with a via on each lane.
  ///
  /// Port of `EndsWithVias`,
  /// `pcbnew/router/pns_diff_pair.h:461`.
  pub const fn ends_with_vias(&self) -> bool {
    self.has_vias
  }

  /// The P end via, whether or not the pair currently claims to have
  /// vias.
  pub const fn via_p(&self) -> Option<&Item> {
    self.via_p.as_ref()
  }

  /// The N end via, whether or not the pair currently claims to have
  /// vias.
  pub const fn via_n(&self) -> Option<&Item> {
    self.via_n.as_ref()
  }

  /// Give the pair two end vias.
  ///
  /// Port of `AppendVias`, `pcbnew/router/pns_diff_pair.h:445`. KiCad
  /// copies both vias and clones the hole each one owns; a via here owns
  /// no hole, so the two [`Item`]s are simply taken by value.
  ///
  /// # Panics
  ///
  /// When either item is not a via, where KiCad's typed `const VIA&`
  /// would not compile.
  pub fn append_vias(&mut self, via_p: Item, via_n: Item) {
    assert!(
      matches!(via_p.body(), ItemBody::Via(_))
        && matches!(via_n.body(), ItemBody::Via(_)),
      "append_vias needs two via bodies"
    );

    self.has_vias = true;
    self.via_p = Some(via_p);
    self.via_n = Some(via_n);
  }

  /// Stop claiming the pair ends with vias.
  ///
  /// Port of `RemoveVias`, `pcbnew/router/pns_diff_pair.h:454`, which
  /// clears the flag and calls `LINE::RemoveVia` on the two cached lines
  /// but **leaves `m_via_p` and `m_via_n` where they were**. That is
  /// benign in C++ only because `updateLine` guards on the flag, and it
  /// is reproduced here for the same reason: [`DiffPair::p_line`] guards
  /// on the flag too, so a later [`DiffPair::set_via_diameter`] still
  /// finds the vias, as it does in KiCad.
  pub const fn remove_vias(&mut self) {
    self.has_vias = false;
  }

  /// Set the copper diameter of both end vias on every padstack layer.
  ///
  /// Port of `SetViaDiameter`,
  /// `pcbnew/router/pns_diff_pair.h:466`. A complex padstack is forced
  /// down to [`crate::item::StackMode::Normal`] first, as
  /// [`Line::set_via_diameter`] does.
  pub fn set_via_diameter(&mut self, diameter: i32) {
    for via in [self.via_p.as_mut(), self.via_n.as_mut()]
      .into_iter()
      .flatten()
    {
      let layers = via.layers();

      if let ItemBody::Via(body) = via.body_mut() {
        body.set_stack_mode(crate::item::StackMode::Normal);
        body.set_diameter(layers, crate::item::Via::ALL_LAYERS, diameter);
      }
    }
  }

  /// Set the drill of both end vias.
  ///
  /// Port of `SetViaDrill`, `pcbnew/router/pns_diff_pair.h:472`. The hole
  /// a stored via owns is a separate arena item, so a caller that has
  /// already stored the vias resizes those holes itself.
  pub fn set_via_drill(&mut self, drill: i32) {
    for via in [self.via_p.as_mut(), self.via_n.as_mut()]
      .into_iter()
      .flatten()
    {
      if let ItemBody::Via(body) = via.body_mut() {
        body.set_drill(drill);
      }
    }
  }

  // -------------------------------------------------------------------
  // The two line views
  // -------------------------------------------------------------------

  /// The P lane as a line.
  ///
  /// Port of `PLine`, `pcbnew/router/pns_diff_pair.h:488`, through
  /// `updateLine` (`:545`): the shape, the width, the net, the first
  /// layer, and the end via when the pair claims to have one. KiCad's
  /// `SetParent` and `SetSourceItem` have nothing to copy, because
  /// nothing ever gives a `DIFF_PAIR` either.
  pub fn p_line(&self) -> Line {
    self.line_of(&self.p, self.net_p, self.via_p.as_ref())
  }

  /// The N lane as a line. Port of `NLine`,
  /// `pcbnew/router/pns_diff_pair.h:496`.
  pub fn n_line(&self) -> Line {
    self.line_of(&self.n, self.net_n, self.via_n.as_ref())
  }

  /// What a following leg carries on from.
  ///
  /// Port of `EndingPrimitives`,
  /// `pcbnew/router/pns_diff_pair.cpp:764`, which the pair placer assigns
  /// to `m_prevPair` once a leg is fixed
  /// (`pcbnew/router/pns_diff_pair_placer.cpp:856`). A pair that ends
  /// with vias answers with those two vias, anchored at their centres; a
  /// pair that does not answers with the last segment of each lane,
  /// anchored at its far end.
  ///
  /// Neither answer is an arena item, which is why [`DpPrimitive`]
  /// exists: KiCad builds the two segments on the stack and lets
  /// `DP_PRIMITIVE_PAIR`'s constructor clone them, and hands over
  /// pointers to the pair's own two `VIA` members, which are values as
  /// well. Note 07 section 5.10 records the stack objects and section
  /// 12.1 assumes handles, so this is the one place the two do not meet.
  ///
  /// [`None`] where KiCad would read `CSegment( -1 )` of an empty chain,
  /// which its caller keeps unreachable by refusing to fix a pair with an
  /// empty lane (`pns_diff_pair_placer.cpp:813`).
  pub fn ending_primitives(&self) -> Option<DpPrimitivePair> {
    // :766
    if self.has_vias {
      let via_p = self.via_p.as_ref()?;
      let via_n = self.via_n.as_ref()?;
      let primitive = |via: &Item| match via.body() {
        ItemBody::Via(body) => Some(DpPrimitive::Via {
          pos: body.pos(),
          layers: via.layers(),
          diameter: body.diameter(via.layers(), via.layers().start()),
        }),
        _ => None,
      };

      let anchor_p = via_p.anchor(0);
      let anchor_n = via_n.anchor(0);

      return Some(DpPrimitivePair::from_primitives(
        primitive(via_p)?,
        primitive(via_n)?,
        anchor_p,
        anchor_n,
      ));
    }

    // :772 to :778
    if self.p.segment_count() == 0 || self.n.segment_count() == 0 {
      return None;
    }

    let seg_p = self.p.segment(self.p.segment_count() - 1);
    let seg_n = self.n.segment(self.n.segment_count() - 1);

    Some(DpPrimitivePair::from_primitives(
      DpPrimitive::Segment(seg_p),
      DpPrimitive::Segment(seg_n),
      seg_p.b,
      seg_n.b,
    ))
  }

  /// The shared body of [`DiffPair::p_line`] and [`DiffPair::n_line`].
  ///
  /// Port of `updateLine`, `pcbnew/router/pns_diff_pair.h:545`.
  fn line_of(
    &self,
    chain: &LineChain,
    net: Option<NetId>,
    via: Option<&Item>,
  ) -> Line {
    let mut line = Line::new();

    line.set_shape(chain.clone());
    line.set_width(self.width);
    line.set_net(net);
    line.set_layer(self.layers.start());

    if self.has_vias
      && let Some(via) = via
    {
      line.append_via(via.clone());
    }

    line
  }
}

/// The parallelism threshold `CoupledSegmentPairs` asks for, in
/// nanometres.
///
/// The literal `2` at `pcbnew/router/pns_diff_pair.cpp:857`. KiCad asks
/// the same question with three different thresholds: this one,
/// [`Seg::APPROX_DISTANCE_THRESHOLD`] in
/// [`DiffPair::coupled_length_of_chains`] (`:883`), and
/// `DP_PARALLELITY_THRESHOLD = 5` in `TOPOLOGY::AssembleDiffPair`
/// (`pcbnew/router/pns_topology.h:106`).
const COUPLED_SEGMENTS_PARALLELISM_THRESHOLD: i32 = 2;

/// The slack `checkGap` allows below the gap, in nanometres.
///
/// The bare `100` of `pcbnew/router/pns_diff_pair.cpp:184`, which has no
/// name in C++ either.
const CHECK_GAP_SLACK: i32 = 100;

impl DiffPair {
  // -------------------------------------------------------------------
  // Measurements
  // -------------------------------------------------------------------

  /// The difference in length between the two lanes.
  ///
  /// Port of `Skew`, `pcbnew/router/pns_diff_pair.cpp:828`.
  ///
  /// Deviation: KiCad subtracts two `long long` chain lengths into a
  /// `double`. Both inputs are integers and their difference is exact in
  /// `i64`, so the result is the same number without the float, which
  /// `DESIGN.md` section 2 asks for.
  pub fn skew(&self) -> i64 {
    self.p.length() - self.n.length()
  }

  /// Every pair of segments of the two lanes that run alongside each
  /// other at the gap.
  ///
  /// Port of `CoupledSegmentPairs`,
  /// `pcbnew/router/pns_diff_pair.cpp:834`. Three details decide what
  /// comes out. The parallelism threshold is the explicit
  /// `COUPLED_SEGMENTS_PARALLELISM_THRESHOLD` rather than the default
  /// one. The distance compared against [`DiffPair::gap_constraint`] is
  /// the centreline distance **minus the width**, that is the copper edge
  /// to edge gap, so the constraint has to hold the edge to edge value
  /// for this to answer anything; see the module documentation. And the
  /// indices in the records are indices into the chains as they stand,
  /// which is why KiCad's comment at `:838` forbids simplifying them
  /// here.
  ///
  /// KiCad skips arc segments on both sides (`:842`, `:847`). This crate
  /// has no arcs yet, so those two guards have nothing to skip.
  pub fn coupled_segment_pairs(&self) -> Vec<CoupledSegments> {
    let mut pairs = Vec::new();

    for index_p in 0..self.p.segment_count() {
      for index_n in 0..self.n.segment_count() {
        // :850
        let parent_p = self.p.segment(index_p);
        let parent_n = self.n.segment(index_n);

        // :855
        let distance = i64::from(parent_p.distance_to_segment(&parent_n))
          - i64::from(self.width);

        // :857
        if !parent_p
          .approx_parallel(&parent_n, COUPLED_SEGMENTS_PARALLELISM_THRESHOLD)
          || !self.gap_constraint.matches(distance.abs())
        {
          continue;
        }

        if let Some((coupled_p, coupled_n)) =
          common_parallel_projection(parent_p, parent_n)
        {
          // :860
          pairs.push(CoupledSegments {
            coupled_p,
            coupled_n,
            parent_p,
            parent_n,
            index_p,
            index_n,
          });
        }
      }
    }

    pairs
  }

  /// How much of the P lane runs alongside the N lane.
  ///
  /// Port of `double CoupledLength() const`,
  /// `pcbnew/router/pns_diff_pair.cpp:893`, the sum of the P clips of
  /// [`DiffPair::coupled_segment_pairs`]. `tryWalkDp` scores its
  /// candidates with it (`pns_diff_pair_placer.cpp:294`).
  ///
  /// Deviation: an `i64` sum of integer segment lengths where KiCad
  /// accumulates in a `double`.
  pub fn coupled_length(&self) -> i64 {
    self
      .coupled_segment_pairs()
      .iter()
      .map(|pair| i64::from(pair.coupled_p.length()))
      .sum()
  }

  /// The same measurement over two chains the pair does not hold.
  ///
  /// Port of
  /// `int64_t CoupledLength( const SHAPE_LINE_CHAIN&, const SHAPE_LINE_CHAIN& )`,
  /// `pcbnew/router/pns_diff_pair.cpp:868`, which the optimizer uses to
  /// score a hypothetical shape without writing it into the pair
  /// (`pns_optimizer.cpp:1338`).
  ///
  /// It is **not** the same computation as [`DiffPair::coupled_length`]:
  /// there is no arc guard and the parallelism threshold is the default
  /// [`Seg::APPROX_DISTANCE_THRESHOLD`] rather than
  /// `COUPLED_SEGMENTS_PARALLELISM_THRESHOLD`, so the two can disagree
  /// on a shape whose lanes tilt slightly.
  pub fn coupled_length_of_chains(&self, p: &LineChain, n: &LineChain) -> i64 {
    let mut total = 0;

    for index_p in 0..p.segment_count() {
      for index_n in 0..n.segment_count() {
        let segment_p = p.segment(index_p);
        let segment_n = n.segment(index_n);

        let distance = i64::from(segment_p.distance_to_segment(&segment_n))
          - i64::from(self.width);

        if !segment_p
          .approx_parallel(&segment_n, Seg::APPROX_DISTANCE_THRESHOLD)
          || !self.gap_constraint.matches(distance.abs())
        {
          continue;
        }

        if let Some((clip_p, _)) =
          common_parallel_projection(segment_p, segment_n)
        {
          total += i64::from(clip_p.length());
        }
      }
    }

    total
  }

  // -------------------------------------------------------------------
  // The fit
  // -------------------------------------------------------------------

  /// Build the two lanes between one entry gateway and one target
  /// gateway, and say whether the result is acceptable.
  ///
  /// Port of `BuildInitial`,
  /// `pcbnew/router/pns_diff_pair.cpp:208`. Four things a reader has to
  /// know, all of which are behaviour and not tidiness:
  ///
  /// - The three accept tests at `:251`, `:254` and `:257` run on the raw
  ///   middle traces, **not** on the lanes with the entry and target
  ///   leads appended. So a lead that crosses the other lane, or a fan
  ///   lead that doubles back, is accepted.
  /// - The store of the middle traces at `:218` is what the entry angle
  ///   check reads, so it is not the redundant assignment it looks like;
  ///   the `else` branch at `:233` repeats it for the case where the
  ///   check did not run.
  /// - [`check_connection_angle`] is called in two directions: entry lead
  ///   against middle at `:223`, then middle against the **reversed**
  ///   target lead at `:244`.
  /// - The allowed angle mask always gains
  ///   [`AngleType::STRAIGHT`] and [`AngleType::OBTUSE`] on top of
  ///   whatever the gateway asked for.
  ///
  /// `checkGap` compares against [`DiffPair::gap_constraint`]'s bare
  /// value, tolerance discarded (`:251` goes through
  /// `RANGED_NUM::operator T()`), and [`fit_gateways`] fills that value
  /// with the centre to centre pitch.
  pub fn build_initial(
    &mut self,
    entry: &DpGateway,
    target: &DpGateway,
    prefer_diagonal: bool,
  ) -> bool {
    // :211, :213
    let p = build_trace(entry.anchor_p(), target.anchor_p(), prefer_diagonal);
    let n = build_trace(entry.anchor_n(), target.anchor_n(), prefer_diagonal);

    // :216
    let mut mask =
      entry.allowed_angles() | AngleType::STRAIGHT | AngleType::OBTUSE;

    // :218, and see the doc comment: this store feeds the check below.
    self.p = p.clone();
    self.n = n.clone();

    if entry.has_entry_lines() {
      // :223
      if !check_connection_angle(
        entry.entry_p(),
        entry.entry_n(),
        &self.p,
        &self.n,
        mask,
      ) {
        return false;
      }

      // :226 to :229
      self.p = entry.entry_p().clone();
      self.n = entry.entry_n().clone();
      self.p.append_chain(&p);
      self.n.append_chain(&n);
    } else {
      // :233
      self.p = p.clone();
      self.n = n.clone();
    }

    // :237
    mask = target.allowed_angles() | AngleType::STRAIGHT | AngleType::OBTUSE;

    if target.has_entry_lines() {
      // :241, :242
      let mut reversed = target.clone();
      reversed.reverse();

      // :244
      if !check_connection_angle(
        &self.p,
        &self.n,
        reversed.entry_p(),
        reversed.entry_n(),
        mask,
      ) {
        return false;
      }

      // :247, :248
      self.p.append_chain(reversed.entry_p());
      self.n.append_chain(reversed.entry_n());
    }

    // :251, :254, :257. All three on the middle traces.
    check_gap(&p, &n, self.gap_constraint.value())
      && p.self_intersecting().is_none()
      && n.self_intersecting().is_none()
      && !p.intersects_chain(&n)
  }

  /// Whether the last segments of this pair meet the first segments of
  /// another at an allowed angle.
  ///
  /// Port of `CheckConnectionAngle`,
  /// `pcbnew/router/pns_diff_pair.cpp:264`, over the two pairs' chains.
  pub fn check_connection_angle(
    &self,
    other: &DiffPair,
    allowed_angles: AngleType,
  ) -> bool {
    check_connection_angle(&self.p, &self.n, &other.p, &other.n, allowed_angles)
  }
}

/// One `BuildInitialTrace` of [`DiffPair::build_initial`].
///
/// The `DIRECTION_45().BuildInitialTrace( a, b, aPrefDiagonal )` of
/// `pcbnew/router/pns_diff_pair.cpp:211`, on a default constructed
/// direction, so the posture argument decides and the corner mode is the
/// mitered 45 default.
fn build_trace(from: Vec2, to: Vec2, prefer_diagonal: bool) -> LineChain {
  LineChain::from_points(
    Direction45::default().build_initial_trace(
      from,
      to,
      prefer_diagonal,
      CornerMode::Mitered45,
    ),
    false,
  )
}

/// Whether two lanes may be joined to two other lanes.
///
/// Port of `DIFF_PAIR::CheckConnectionAngle`,
/// `pcbnew/router/pns_diff_pair.cpp:264`, as a free function over the
/// four chains, because KiCad's version exists only so that
/// `DP_GATEWAY::Entry()` (`:296`) can wrap two entry chains in a whole
/// throwaway `DIFF_PAIR` to call it. `first` is KiCad's `this` and
/// `second` its `aOther`: the **last** segment of each `first` chain is
/// compared against the **first** segment of the matching `second` one.
///
/// An empty chain on either side of a lane passes that lane, which is the
/// mechanism that makes `buildDpContinuation`'s one sided angled gateways
/// work: they set an entry line on one lane and an empty chain on the
/// other (`pns_diff_pair.cpp:624`, `:632`).
pub fn check_connection_angle(
  first_p: &LineChain,
  first_n: &LineChain,
  second_p: &LineChain,
  second_n: &LineChain,
  allowed_angles: AngleType,
) -> bool {
  check_lane_connection_angle(first_p, second_p, allowed_angles)
    && check_lane_connection_angle(first_n, second_n, allowed_angles)
}

/// One lane of [`check_connection_angle`].
fn check_lane_connection_angle(
  first: &LineChain,
  second: &LineChain,
  allowed_angles: AngleType,
) -> bool {
  // :268, :280. An empty chain on either side passes.
  if first.segment_count() == 0 || second.segment_count() == 0 {
    return true;
  }

  let leaving =
    Direction45::from_seg(&first.segment(first.segment_count() - 1), false);
  let arriving = Direction45::from_seg(&second.segment(0), false);

  leaving.angle(arriving).intersects(allowed_angles)
}

/// Whether no segment of one lane comes closer than the gap to any
/// segment of the other.
///
/// Port of the file static `checkGap`,
/// `pcbnew/router/pns_diff_pair.cpp:182`. It is a **minimum** test and
/// not a coupling test: it rejects a candidate whose lanes pinch, and
/// says nothing about lanes that diverge. The comparison is quadratic in
/// the segment counts and runs inside the innermost loop of
/// [`fit_gateways`].
fn check_gap(p: &LineChain, n: &LineChain, gap: i32) -> bool {
  let threshold = i64::from(gap - CHECK_GAP_SLACK);
  let squared_threshold = threshold * threshold;

  for index_p in 0..p.segment_count() {
    for index_n in 0..n.segment_count() {
      if p
        .segment(index_p)
        .squared_distance_to_segment(&n.segment(index_n))
        < squared_threshold
      {
        return false;
      }
    }
  }

  true
}

// ---------------------------------------------------------------------
// DpGateway
// ---------------------------------------------------------------------

/// A pair of anchor points a route may leave from or arrive at.
///
/// Port of `DP_GATEWAY`, `pcbnew/router/pns_diff_pair.h:43`. The two
/// anchors are [`DpGateways`]'s pitch apart, with one documented
/// exception: the collinear midpoint exit of
/// [`DpGateways::build_generic`] spaces them at half that, which is
/// erratum E4.
///
/// The optional entry lines lead from the board object the gateway sits
/// on to the two anchors. They are set as a pair or not at all, and one
/// of the two may be empty, which is how `buildDpContinuation` builds a
/// gateway that advances one lane and leaves the other alone.
#[derive(Clone, Debug)]
pub struct DpGateway {
  /// The lead in chain of the P lane. Port of `m_entryP`,
  /// `pcbnew/router/pns_diff_pair.h:108`.
  entry_p: LineChain,
  /// The lead in chain of the N lane. Port of `m_entryN`.
  entry_n: LineChain,
  /// Whether the two chains above mean anything. Port of
  /// `m_hasEntryLines`, `pcbnew/router/pns_diff_pair.h:109`.
  has_entry_lines: bool,
  /// The P anchor. Port of `m_anchorP`,
  /// `pcbnew/router/pns_diff_pair.h:110`.
  anchor_p: Vec2,
  /// The N anchor. Port of `m_anchorN`.
  anchor_n: Vec2,
  /// Whether the lead out of this gateway starts diagonal. Port of
  /// `m_isDiagonal`, `pcbnew/router/pns_diff_pair.h:111`.
  ///
  /// KiCad documents it as "the gateway anchors lie on a diagonal line"
  /// (`pns_diff_pair.h:60`), which contradicts what
  /// [`DpGateways::build_for_cursor`] passes: the branch that puts the
  /// anchors on a diagonal passes `false` and the axis aligned branch
  /// passes `true` (`pns_diff_pair.cpp:550` to `:577`). Its only use is
  /// as the `aStartDiagonal` hint to `BuildInitialTrace` in
  /// `buildEntries` (`:590`), under which reading the values are right,
  /// so the values are ported and the documentation is corrected here.
  /// That is erratum E7.
  is_diagonal: bool,
  /// Which angles a route may leave the gateway at. Port of
  /// `m_allowedEntryAngles`, `pcbnew/router/pns_diff_pair.h:112`,
  /// defaulting to [`AngleType::OBTUSE`] (`:47`).
  allowed_entry_angles: AngleType,
  /// The score [`fit_gateways`] sums. Port of `m_priority`,
  /// `pcbnew/router/pns_diff_pair.h:113`, defaulting to 0.
  priority: i32,
}

impl DpGateway {
  /// A gateway with KiCad's default angle mask and priority.
  ///
  /// Port of `DP_GATEWAY( const VECTOR2I&, const VECTOR2I&, bool )` with
  /// both default arguments taken, `pcbnew/router/pns_diff_pair.h:46`.
  pub fn new(anchor_p: Vec2, anchor_n: Vec2, is_diagonal: bool) -> Self {
    Self::with_angles(anchor_p, anchor_n, is_diagonal, AngleType::OBTUSE, 0)
  }

  /// A gateway with an explicit angle mask and priority.
  ///
  /// Port of the full `DP_GATEWAY` constructor,
  /// `pcbnew/router/pns_diff_pair.h:46`, which leaves the entry lines
  /// unset.
  pub fn with_angles(
    anchor_p: Vec2,
    anchor_n: Vec2,
    is_diagonal: bool,
    allowed_entry_angles: AngleType,
    priority: i32,
  ) -> Self {
    Self {
      entry_p: LineChain::new(),
      entry_n: LineChain::new(),
      has_entry_lines: false,
      anchor_p,
      anchor_n,
      is_diagonal,
      allowed_entry_angles,
      priority,
    }
  }

  /// The P anchor. Port of `AnchorP`,
  /// `pcbnew/router/pns_diff_pair.h:66`.
  pub const fn anchor_p(&self) -> Vec2 {
    self.anchor_p
  }

  /// The N anchor. Port of `AnchorN`,
  /// `pcbnew/router/pns_diff_pair.h:68`.
  pub const fn anchor_n(&self) -> Vec2 {
    self.anchor_n
  }

  /// Whether the lead out of the gateway starts diagonal. Port of
  /// `IsDiagonal`, `pcbnew/router/pns_diff_pair.h:60`; see the field for
  /// what the flag really means.
  pub const fn is_diagonal(&self) -> bool {
    self.is_diagonal
  }

  /// The allowed entry angle mask. Port of `AllowedAngles`,
  /// `pcbnew/router/pns_diff_pair.h:73`.
  pub const fn allowed_angles(&self) -> AngleType {
    self.allowed_entry_angles
  }

  /// The matching score. Port of `Priority`,
  /// `pcbnew/router/pns_diff_pair.h:78`.
  pub const fn priority(&self) -> i32 {
    self.priority
  }

  /// Set the matching score. Port of `SetPriority`,
  /// `pcbnew/router/pns_diff_pair.h:83`.
  pub const fn set_priority(&mut self, priority: i32) {
    self.priority = priority;
  }

  /// Whether the gateway carries lead in chains. Port of
  /// `HasEntryLines`, `pcbnew/router/pns_diff_pair.h:101`.
  pub const fn has_entry_lines(&self) -> bool {
    self.has_entry_lines
  }

  /// The P lead in chain, empty when there is none.
  ///
  /// Port of `EntryP`, `pcbnew/router/pns_diff_pair.h:95`.
  pub const fn entry_p(&self) -> &LineChain {
    &self.entry_p
  }

  /// The N lead in chain, empty when there is none.
  ///
  /// Port of `EntryN`, `pcbnew/router/pns_diff_pair.h:96`.
  pub const fn entry_n(&self) -> &LineChain {
    &self.entry_n
  }

  /// Give the gateway both lead in chains.
  ///
  /// Port of `SetEntryLines`,
  /// `pcbnew/router/pns_diff_pair.h:89`. There is no way to set one:
  /// the flag covers both, and either chain may be empty.
  pub fn set_entry_lines(&mut self, entry_p: LineChain, entry_n: LineChain) {
    self.entry_p = entry_p;
    self.entry_n = entry_n;
    self.has_entry_lines = true;
  }

  /// Turn the lead in chains round.
  ///
  /// Port of `Reverse`, `pcbnew/router/pns_diff_pair.cpp:201`, which is
  /// how [`DiffPair::build_initial`] turns a target gateway's leads,
  /// which run from the board object inwards, into leads that run from
  /// the middle out.
  pub fn reverse(&mut self) {
    self.entry_p.reverse();
    self.entry_n.reverse();
  }

  /// The two lead in chains as a pair, for the angle test.
  ///
  /// Port of `Entry`, `pcbnew/router/pns_diff_pair.cpp:296`, which builds
  /// a whole throwaway `DIFF_PAIR` out of the two chains with a gap of
  /// zero purely so that `CheckConnectionAngle` can be called on it, and
  /// does so three times per `BuildInitial`.
  /// [`DiffPair::build_initial`] calls [`check_connection_angle`] on the
  /// chains directly instead; this exists for a caller that wants
  /// KiCad's shape.
  pub fn entry(&self) -> DiffPair {
    DiffPair::from_chains(self.entry_p.clone(), self.entry_n.clone(), 0)
  }
}

// ---------------------------------------------------------------------
// DpPrimitivePair
// ---------------------------------------------------------------------

/// Where a pair route starts or ends: two board objects and two anchors.
///
/// Port of `DP_PRIMITIVE_PAIR`,
/// `pcbnew/router/pns_diff_pair.h:119`, with the two owned `ITEM*`
/// clones replaced by handles into the arena. That removes KiCad's
/// clone in every constructor, the delete in the destructor, and the
/// assignment operator that leaks two items and can leave a pair
/// describing one object while pointing at another (erratum E17).
///
/// Every method that has to look at the objects takes the
/// [`World`] rather than dereferencing a pointer, so a handle that has
/// gone stale answers `None` instead of dangling.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct DpPrimitivePair {
  /// The P object, if the pair started on one. Port of `m_primP`,
  /// `pcbnew/router/pns_diff_pair.h:145`.
  prim_p: DpPrimitive,
  /// The N object. Port of `m_primN`.
  prim_n: DpPrimitive,
  /// The point on the P object a route leaves from. Port of `m_anchorP`,
  /// `pcbnew/router/pns_diff_pair.h:147`.
  anchor_p: Vec2,
  /// The point on the N object. Port of `m_anchorN`.
  anchor_n: Vec2,
}

/// One half of a [`DpPrimitivePair`]: the object a route leaves from.
///
/// KiCad's `m_primP` is an owned `ITEM*` clone, so it does not care
/// whether the object it describes is on the board. Two of the three
/// things it is ever given are **not**: [`DiffPair::ending_primitives`]
/// (`pcbnew/router/pns_diff_pair.cpp:764`) builds two stack `SEGMENT`s
/// out of the last segment of each lane, and, when the pair ends with
/// vias, hands over the pair's own two `VIA` members, which
/// `DIFF_PAIR_PLACER::FixRoute` clones into the node separately
/// (`pns_diff_pair_placer.cpp:838`). Neither is an arena item here, so a
/// bare [`ItemId`] cannot express them.
///
/// [`DpPrimitive::Segment`] and [`DpPrimitive::Via`] carry everything the
/// readers below and [`DpGateways::build_from_primitive_pair`] ask of
/// such an object: its kind, its two anchors, its shape and its layers.
/// A via's diameter is carried only so that its shape is the circle
/// `BuildFromPrimitivePair` dispatches on (`:457`); nothing reads the
/// size.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub enum DpPrimitive {
  /// No object on this side. KiCad's null `m_primP`.
  #[default]
  None,
  /// A board object in the arena.
  Stored(ItemId),
  /// A segment that is in no node.
  Segment(Seg),
  /// A via that is in no node.
  Via {
    /// Where it sits, which is its only anchor.
    pos: Vec2,
    /// The copper layers it spans.
    layers: LayerRange,
    /// Its copper diameter, so that [`DpPrimitive::shape`] is a circle.
    diameter: i32,
  },
}

impl DpPrimitive {
  /// The `ITEM::Kind()` of the object, if there is one.
  pub fn kind(&self, world: &World) -> Option<Kind> {
    match self {
      DpPrimitive::None => None,
      DpPrimitive::Stored(id) => world.item(*id).map(Item::kind),
      DpPrimitive::Segment(_) => Some(Kind::SEGMENT),
      DpPrimitive::Via { .. } => Some(Kind::VIA),
    }
  }

  /// Whether the object is one of the kinds in a mask, `ITEM::OfKind`.
  pub fn of_kind(&self, world: &World, mask: Kind) -> bool {
    self.kind(world).is_some_and(|kind| kind.of_kind(mask))
  }

  /// One of the object's connection points, `ITEM::Anchor`.
  pub fn anchor(&self, world: &World, index: usize) -> Option<Vec2> {
    match self {
      DpPrimitive::None => None,
      DpPrimitive::Stored(id) => world.item(*id).map(|item| item.anchor(index)),
      DpPrimitive::Segment(seg) => Some(if index == 0 { seg.a } else { seg.b }),
      DpPrimitive::Via { pos, .. } => Some(*pos),
    }
  }

  /// The segment the object is, when it is one.
  pub fn seg(&self, world: &World) -> Option<Seg> {
    match self {
      DpPrimitive::Stored(id) => match world.item(*id)?.body() {
        ItemBody::Segment(body) => Some(body.seg()),
        _ => None,
      },
      DpPrimitive::Segment(seg) => Some(*seg),
      DpPrimitive::None | DpPrimitive::Via { .. } => None,
    }
  }

  /// The object's shape over its whole padstack, `Shape( -1 )`.
  pub fn shape<'a>(&self, world: &'a World) -> Option<Cow<'a, Shape>> {
    match self {
      DpPrimitive::None => None,
      DpPrimitive::Stored(id) => world.item(*id)?.shape(-1),
      DpPrimitive::Segment(seg) => Some(Cow::Owned(Shape::Segment {
        seg: *seg,
        width: 0,
      })),
      DpPrimitive::Via { pos, diameter, .. } => {
        Some(Cow::Owned(Shape::circle(*pos, diameter / 2)))
      }
    }
  }

  /// The copper layers the object spans.
  pub fn layers(&self, world: &World) -> Option<LayerRange> {
    match self {
      DpPrimitive::None => None,
      DpPrimitive::Stored(id) => world.item(*id).map(Item::layers),
      DpPrimitive::Segment(_) => None,
      DpPrimitive::Via { layers, .. } => Some(*layers),
    }
  }

  /// Whether there is an object at all, KiCad's `m_primP != nullptr`.
  pub const fn is_some(&self) -> bool {
    !matches!(self, DpPrimitive::None)
  }

  /// The arena handle, when the object is a board object.
  pub const fn stored(&self) -> Option<ItemId> {
    match self {
      DpPrimitive::Stored(id) => Some(*id),
      DpPrimitive::None | DpPrimitive::Segment(_) | DpPrimitive::Via { .. } => {
        None
      }
    }
  }
}

/// The answer of [`DpPrimitivePair::cursor_orientation`].
///
/// KiCad's two out parameters (`pcbnew/router/pns_diff_pair.cpp:120`).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct CursorOrientation {
  /// The point between the two anchors the route grows from.
  pub midpoint: Vec2,
  /// Which way "forward" is, as long as the two anchors are apart.
  pub direction: Vec2,
}

impl DpPrimitivePair {
  /// A pair of two board objects, anchored at each object's first anchor.
  ///
  /// Port of `DP_PRIMITIVE_PAIR( ITEM*, ITEM* )`,
  /// `pcbnew/router/pns_diff_pair.cpp:39`. `None` when either handle is
  /// not in the world, where KiCad would dereference it.
  pub fn from_items(
    world: &World,
    prim_p: ItemId,
    prim_n: ItemId,
  ) -> Option<Self> {
    Some(Self {
      prim_p: DpPrimitive::Stored(prim_p),
      prim_n: DpPrimitive::Stored(prim_n),
      anchor_p: world.item(prim_p)?.anchor(0),
      anchor_n: world.item(prim_n)?.anchor(0),
    })
  }

  /// A pair of two objects that need not be in the arena, at anchors the
  /// caller names.
  ///
  /// The shape [`DiffPair::ending_primitives`] needs: KiCad builds its
  /// `DP_PRIMITIVE_PAIR` from two stack objects and then calls
  /// `SetAnchors` (`pcbnew/router/pns_diff_pair.cpp:773`, `:776`).
  pub const fn from_primitives(
    prim_p: DpPrimitive,
    prim_n: DpPrimitive,
    anchor_p: Vec2,
    anchor_n: Vec2,
  ) -> Self {
    Self {
      prim_p,
      prim_n,
      anchor_p,
      anchor_n,
    }
  }

  /// A pair of two bare points.
  ///
  /// Port of `DP_PRIMITIVE_PAIR( const VECTOR2I&, const VECTOR2I& )`,
  /// `pcbnew/router/pns_diff_pair.cpp:56`.
  pub const fn from_anchors(anchor_p: Vec2, anchor_n: Vec2) -> Self {
    Self {
      prim_p: DpPrimitive::None,
      prim_n: DpPrimitive::None,
      anchor_p,
      anchor_n,
    }
  }

  /// The P object. Port of `PrimP`,
  /// `pcbnew/router/pns_diff_pair.h:136`.
  pub const fn prim_p(&self) -> DpPrimitive {
    self.prim_p
  }

  /// The N object. Port of `PrimN`,
  /// `pcbnew/router/pns_diff_pair.h:137`.
  pub const fn prim_n(&self) -> DpPrimitive {
    self.prim_n
  }

  /// The P anchor. Port of `AnchorP`,
  /// `pcbnew/router/pns_diff_pair.h:130`.
  pub const fn anchor_p(&self) -> Vec2 {
    self.anchor_p
  }

  /// The N anchor. Port of `AnchorN`,
  /// `pcbnew/router/pns_diff_pair.h:131`.
  pub const fn anchor_n(&self) -> Vec2 {
    self.anchor_n
  }

  /// Move both anchors. Port of `SetAnchors`,
  /// `pcbnew/router/pns_diff_pair.cpp:48`.
  pub const fn set_anchors(&mut self, anchor_p: Vec2, anchor_n: Vec2) {
    self.anchor_p = anchor_p;
    self.anchor_n = anchor_n;
  }

  /// Whether the pair sits on objects that have a direction of their own.
  ///
  /// Port of `Directional`,
  /// `pcbnew/router/pns_diff_pair.cpp:97`, which tests the **P** object
  /// only and answers false when there is none.
  pub fn directional(&self, world: &World) -> bool {
    self.prim_p.of_kind(world, Kind::SEGMENT | Kind::ARC)
  }

  /// Which way the P object arrives at its anchor.
  ///
  /// Port of `DirP`, `pcbnew/router/pns_diff_pair.cpp:161`.
  pub fn dir_p(&self, world: &World) -> Direction45 {
    anchor_direction(world, self.prim_p, self.anchor_p)
  }

  /// Which way the N object arrives at its anchor.
  ///
  /// Port of `DirN`, `pcbnew/router/pns_diff_pair.cpp:167`.
  pub fn dir_n(&self, world: &World) -> Direction45 {
    anchor_direction(world, self.prim_n, self.anchor_n)
  }

  /// Where the route grows from, and which way.
  ///
  /// Port of `CursorOrientation`,
  /// `pcbnew/router/pns_diff_pair.cpp:122`, whose two out parameters
  /// become the answer. `None` where KiCad asserts, at `:125`: both
  /// objects have to be there.
  ///
  /// Two branches with a trap each. When both objects are segments and
  /// **parallel**, the direction is whichever way the P segment runs and
  /// the routine returns before the cursor is ever consulted (`:144`), so
  /// the cursor cannot flip it. Otherwise the anchors come from
  /// `Anchor(1)` when both are segments and from `Anchor(0)` when they
  /// are not, and the direction is the perpendicular of the anchor line,
  /// flipped towards the cursor. There is no path that takes `Anchor(0)`
  /// for one object and `Anchor(1)` for the other.
  pub fn cursor_orientation(
    &self,
    world: &World,
    cursor: Vec2,
  ) -> Option<CursorOrientation> {
    // :125
    if !self.prim_p.is_some() || !self.prim_n.is_some() {
      return None;
    }

    let (point_p, point_n) = if self.prim_p.of_kind(world, Kind::SEGMENT)
      && self.prim_n.of_kind(world, Kind::SEGMENT)
    {
      // :131
      let point_p = self.prim_p.anchor(world, 1)?;
      let point_n = self.prim_n.anchor(world, 1)?;

      // :136 to :145
      if let (Some(seg_p), Some(seg_n)) =
        (self.prim_p.seg(world), self.prim_n.seg(world))
        && seg_p.b != seg_p.a
        && seg_n.b != seg_n.a
        && seg_p.approx_parallel(&seg_n, Seg::APPROX_DISTANCE_THRESHOLD)
      {
        return Some(CursorOrientation {
          midpoint: midpoint_of(point_p, point_n),
          direction: (seg_p.b - seg_p.a)
            .resize((point_p - point_n).euclidean_norm()),
        });
      }

      (point_p, point_n)
    } else {
      // :149
      (self.prim_p.anchor(world, 0)?, self.prim_n.anchor(world, 0)?)
    };

    // :153 to :157
    let midpoint = midpoint_of(point_p, point_n);
    let mut direction = (point_p - point_n).perpendicular();

    if direction.dot(cursor - midpoint) < 0 {
      direction = -direction;
    }

    Some(CursorOrientation {
      midpoint,
      direction,
    })
  }
}

/// The midpoint of two points, with C++'s truncating division.
///
/// The `( aP + aN ) / 2` of `pcbnew/router/pns_diff_pair.cpp:141` and
/// `:153`, and the `( p0_p + p0_n ) / 2` of `:676`. `VECTOR2I::operator/`
/// divides each component as an `int`, so it truncates towards zero and
/// not towards minus infinity.
fn midpoint_of(first: Vec2, second: Vec2) -> Vec2 {
  Vec2::new((first.x + second.x) / 2, (first.y + second.y) / 2)
}

/// Which way a board object arrives at one of its anchors.
///
/// Port of `DP_PRIMITIVE_PAIR::anchorDirection`,
/// `pcbnew/router/pns_diff_pair.cpp:110`. The asymmetry is deliberate:
/// at anchor 0 the direction points **away** from the anchor back along
/// the object, and anywhere else it points from anchor 0 towards anchor
/// 1, so in both cases it is the direction the existing track arrives
/// from. An object that is not a segment or an arc, and a handle the
/// world does not know, give an undefined direction.
fn anchor_direction(
  world: &World,
  primitive: DpPrimitive,
  point: Vec2,
) -> Direction45 {
  if !primitive.of_kind(world, Kind::SEGMENT | Kind::ARC) {
    return Direction45::default();
  }

  let (Some(first), Some(second)) =
    (primitive.anchor(world, 0), primitive.anchor(world, 1))
  else {
    return Direction45::default();
  };

  if first == point {
    Direction45::from_vector(first - second, false)
  } else {
    Direction45::from_vector(second - first, false)
  }
}

// ---------------------------------------------------------------------
// DpGateways
// ---------------------------------------------------------------------

/// Why [`DpGateways::build_from_primitive_pair`] built nothing.
///
/// KiCad returns silently in all three cases
/// (`pcbnew/router/pns_diff_pair.cpp:453`), which leaves the placer to
/// report "no gateway fitted" with no idea why. Erratum E10 asks for the
/// reason to survive.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum DpGatewayError {
  /// One of the two handles is not in the world.
  ///
  /// Not reachable in KiCad, which holds cloned items rather than
  /// handles.
  UnknownPrimitive,
  /// The two objects are not the same kind.
  ///
  /// A pad paired with a segment matches neither the solid or via test
  /// at `pcbnew/router/pns_diff_pair.cpp:434` nor the segment or arc test
  /// at `:442`, so KiCad falls through to `:453` and appends nothing.
  /// `FindDpPrimitivePair` keeps it unreachable by requiring equal kinds
  /// (`pns_diff_pair_placer.cpp:559`). Erratum E10.
  ///
  /// A pair with a P object and no N object answers this too, where
  /// KiCad dereferences the null `PrimN()` at `:434`. Only its leaking
  /// assignment operator can build such a pair (erratum E17), which this
  /// port has no counterpart for.
  MixedPrimitiveKinds,
  /// The P object has no shape to fan out of.
  ///
  /// A `SOLID` may hold no shape at all
  /// (`pcbnew/router/pns_solid.h:107`), which is the `shP == nullptr`
  /// return at `pcbnew/router/pns_diff_pair.cpp:452`.
  NoShape,
}

/// The gateway set of one end of a pair route, and the fit over two of
/// them.
///
/// Port of `DP_GATEWAYS`, `pcbnew/router/pns_diff_pair.h:167`.
///
/// Every builder **appends**; none clears first, and
/// [`DpGateways::build_from_primitive_pair`] depends on that when it
/// pushes its fan and then calls [`DpGateways::build_generic`] on top.
///
/// The via fields have a trap KiCad's own call order steps around. A set
/// that never sees [`DpGateways::set_fit_vias`] claims
/// `fit_vias == true` with a via diameter of zero (`:175`, `:176`), and
/// the placer only ever calls it on the target set
/// (`pns_diff_pair_placer.cpp:712`). The entry set therefore runs with
/// that stale default, which is harmless only because
/// [`DpGateways::build_for_cursor`] is the one reader and the entry set
/// never reaches it.
#[derive(Clone, Debug)]
pub struct DpGateways {
  /// The centre to centre spacing of a gateway's two anchors. Port of
  /// `m_gap`, `pcbnew/router/pns_diff_pair.h:198`, which the placer
  /// constructs from `DiffPairGap() + DiffPairWidth()`
  /// (`pns_diff_pair_placer.cpp:616`, `:681`); see the module
  /// documentation for why it is called a pitch here.
  pitch: i32,
  /// The centre to centre spacing of a pair of vias. Port of `m_viaGap`,
  /// `pcbnew/router/pns_diff_pair.h:199`.
  via_gap: i32,
  /// The copper diameter of a via. Port of `m_viaDiameter`,
  /// `pcbnew/router/pns_diff_pair.h:200`.
  via_diameter: i32,
  /// Whether the gateways have to admit a pair of vias. Port of
  /// `m_fitVias`, `pcbnew/router/pns_diff_pair.h:201`.
  fit_vias: bool,
  /// The gateways, in push order. Port of `m_gateways`,
  /// `pcbnew/router/pns_diff_pair.h:203`.
  ///
  /// The order is part of the answer: [`fit_gateways`] keeps the **last**
  /// candidate of an equal score, so a port that reorders a builder
  /// changes the route it picks.
  gateways: Vec<DpGateway>,
}

impl DpGateways {
  /// An empty set at a centre to centre pitch.
  ///
  /// Port of `DP_GATEWAYS( int aGap )`,
  /// `pcbnew/router/pns_diff_pair.h:169`, which starts the via gap at the
  /// pitch, the via diameter at zero and `fit_vias` at **true**.
  pub const fn new(pitch: i32) -> Self {
    Self {
      pitch,
      via_gap: pitch,
      via_diameter: 0,
      fit_vias: true,
      gateways: Vec::new(),
    }
  }

  /// The centre to centre spacing every builder here spaces anchors by.
  pub const fn pitch(&self) -> i32 {
    self.pitch
  }

  /// Whether a pair of vias has to fit in the gateways.
  ///
  /// Port of `SetFitVias`, `pcbnew/router/pns_diff_pair.h:181`. KiCad's
  /// third argument is an `int` whose negative values mean "the same as
  /// the trace gap" (`:186`); it is an [`Option`] here. The one caller
  /// always passes a value (`pns_diff_pair_placer.cpp:712`), so KiCad's
  /// sentinel branch is reachable from nothing that exists.
  pub const fn set_fit_vias(
    &mut self,
    enable: bool,
    diameter: i32,
    via_gap: Option<i32>,
  ) {
    self.fit_vias = enable;
    self.via_diameter = diameter;

    self.via_gap = match via_gap {
      Some(gap) => gap,
      None => self.pitch,
    };
  }

  /// The gateways, in push order. Port of `Gateways`,
  /// `pcbnew/router/pns_diff_pair.h:194`.
  pub fn gateways(&self) -> &[DpGateway] {
    &self.gateways
  }

  /// The gateways, mutably, which is what `FilterByOrientation`'s caller
  /// gets in C++ through the non const `Gateways()`.
  pub fn gateways_mut(&mut self) -> &mut Vec<DpGateway> {
    &mut self.gateways
  }

  /// Drop the gateways whose anchor line makes a listed angle with a
  /// reference direction.
  ///
  /// Port of `FilterByOrientation`,
  /// `pcbnew/router/pns_diff_pair.cpp:391`, which **removes** what
  /// matches the mask, the opposite of what the name suggests. Its one
  /// caller passes `ANG_STRAIGHT | ANG_HALF_FULL` against the direction
  /// of travel (`pns_diff_pair_placer.cpp:724`), that is, "drop every
  /// gateway whose anchor line runs along the way we are going", leaving
  /// the ones that lie across it.
  ///
  /// Two coincident anchors give an undefined direction, whose angle is
  /// [`AngleType::UNDEFINED`] and therefore in no caller's mask, so
  /// degenerate gateways survive.
  pub fn filter_by_orientation(
    &mut self,
    angle_mask: AngleType,
    reference: Direction45,
  ) {
    self.gateways.retain(|gateway| {
      let orientation =
        Direction45::from_vector(gateway.anchor_p - gateway.anchor_n, false);

      !orientation.angle(reference).intersects(angle_mask)
    });
  }

  /// Every 45 degree exit a pair could take from two points.
  ///
  /// Port of `BuildGeneric`,
  /// `pcbnew/router/pns_diff_pair.cpp:649`, the workhorse of the file. It
  /// works entirely through eight 200 nanometre probe segments centred on
  /// the two points, four through each: horizontal, vertical and the two
  /// diagonals. They are only ever used as infinite lines, through
  /// [`Seg::collinear`] and [`Seg::intersect_lines`], so the length is
  /// arbitrary.
  ///
  /// The result is in three families, pushed in this order:
  ///
  /// 1. When the two points already share a horizontal, vertical or
  ///    diagonal line, the midpoint exit and four side by exits (`:673`
  ///    to `:689`). The midpoint exit is the one gateway in the file
  ///    whose anchors are **half** the pitch apart, because
  ///    [`make_gap_vector`] halves what it is given and it is given half
  ///    the pitch (erratum E4); it is spelled out as written, and the
  ///    tests pin that it never wins a fit, since
  ///    [`DiffPair::build_initial`]'s gap test rejects it for any pitch
  ///    above 200 nanometres.
  /// 2. The exits through the intersections of a P probe line and an N
  ///    probe line of the same family, diagonal with diagonal and
  ///    straight with straight (`:710` to `:725`). Two perpendicular legs
  ///    of `pitch / sqrt(2)` put the anchors a pitch apart.
  /// 3. The "8 possibilities of weirder exits" through the intersections
  ///    of a straight probe line with a diagonal one (`:732` to `:755`),
  ///    with legs of `pitch * sqrt(2)` and `pitch` at 45 degrees, which
  ///    again gives a pitch. These are suppressed in via mode.
  ///
  /// `via_mode` suppresses everything that a pair of vias could not sit
  /// in, which is families 1 and 3.
  ///
  /// The collinearity guards at `:702` and `:705` exist because
  /// [`Seg::intersect_lines`] answers two collinear lines with the
  /// midpoint between their start points rather than with "no
  /// intersection", and that answer has to be thrown away. The second
  /// guard tests `st_p[i]` against `st_p[j]` where the line above it
  /// intersected `st_p[i]` with `st_n[j]`; erratum E5 works through the
  /// four index combinations and shows the typo changes no answer,
  /// because two probe lines of different families are never collinear.
  /// It is transcribed as written.
  pub fn build_generic(
    &mut self,
    p0_p: Vec2,
    p0_n: Vec2,
    build_entries: bool,
    via_mode: bool,
  ) {
    /// The multiple of the pitch above which a pad pair counts as far
    /// apart, `padToGapThreshold` at
    /// `pcbnew/router/pns_diff_pair.cpp:655`.
    const PAD_TO_GAP_THRESHOLD: i32 = 3;

    /// Half the length of a probe segment, `pns_diff_pair.cpp:658`.
    const PROBE: i32 = 100;

    // :656
    let pad_distance = (p0_n - p0_p).euclidean_norm();

    // :658 to :665
    let straight_p = [probe(p0_p, PROBE, 0), probe(p0_p, 0, PROBE)];
    let straight_n = [probe(p0_n, PROBE, 0), probe(p0_n, 0, PROBE)];
    let diagonal_p = [probe(p0_p, PROBE, PROBE), probe(p0_p, -PROBE, PROBE)];
    let diagonal_n = [probe(p0_n, PROBE, PROBE), probe(p0_n, -PROBE, PROBE)];

    // :668, the midpoint and side by exits.
    for index in 0..2 {
      // :670, :671
      let straight_collinear = straight_p[index].collinear(&straight_n[index]);
      let diagonal_collinear = diagonal_p[index].collinear(&diagonal_n[index]);

      // :673
      if !(straight_collinear || diagonal_collinear) || via_mode {
        continue;
      }

      // :675 to :682
      let mut direction = make_gap_vector(p0_n - p0_p, self.pitch / 2);
      let middle = midpoint_of(p0_p, p0_n);
      let priority = if pad_distance > PAD_TO_GAP_THRESHOLD * self.pitch {
        2
      } else {
        1
      };

      self.gateways.push(DpGateway::with_angles(
        middle - direction,
        middle + direction,
        diagonal_collinear,
        AngleType::RIGHT,
        priority,
      ));

      // :684 to :688
      direction = make_gap_vector(p0_n - p0_p, 2 * self.pitch);
      let across = direction.perpendicular();

      self.gateways.push(DpGateway::new(
        p0_p - direction,
        p0_p - direction + across,
        diagonal_collinear,
      ));
      self.gateways.push(DpGateway::new(
        p0_p - direction,
        p0_p - direction - across,
        diagonal_collinear,
      ));
      self.gateways.push(DpGateway::new(
        p0_n + direction + across,
        p0_n + direction,
        diagonal_collinear,
      ));
      self.gateways.push(DpGateway::new(
        p0_n + direction - across,
        p0_n + direction,
        diagonal_collinear,
      ));
    }

    // :693
    for i in 0..2 {
      for j in 0..2 {
        // :699 to :706
        let mut intersections = [
          diagonal_n[i].intersect_lines(&diagonal_p[j]),
          straight_p[i].intersect_lines(&straight_n[j]),
        ];

        if diagonal_n[i].collinear(&diagonal_p[j]) {
          intersections[0] = None;
        }

        // Erratum E5: `st_p[i]` against `st_p[j]`, as written.
        if straight_p[i].collinear(&straight_p[j]) {
          intersections[1] = None;
        }

        // :710, the diagonal to diagonal and straight to straight exits.
        for (k, intersection) in intersections.iter().enumerate() {
          let Some(middle) = *intersection else {
            continue;
          };

          // :716
          if middle == p0_p || middle == p0_n {
            continue;
          }

          // :718 to :722
          let priority = if pad_distance > PAD_TO_GAP_THRESHOLD * self.pitch {
            10
          } else {
            20
          };
          let leg = (f64::from(self.pitch) * FRAC_1_SQRT_2).ceil() as i32;

          self.gateways.push(DpGateway::with_angles(
            middle + (p0_p - middle).resize(leg),
            middle + (p0_n - middle).resize(leg),
            k == 0,
            AngleType::OBTUSE,
            priority,
          ));
        }

        // :728, :729
        let intersections = [
          straight_n[i].intersect_lines(&diagonal_p[j]),
          straight_p[i].intersect_lines(&diagonal_n[j]),
        ];

        // :732, the diagonal to straight exits.
        for intersection in intersections {
          let Some(middle) = intersection else {
            continue;
          };

          // :738
          if via_mode || middle == p0_p || middle == p0_n {
            continue;
          }

          let long_leg = (f64::from(self.pitch) * SQRT_2).ceil() as i32;

          // :742 to :746
          let leg_p = (p0_p - middle).resize(long_leg);
          let leg_n = (p0_n - middle).resize(self.pitch);

          if angle_between(leg_p, leg_n) != AngleType::ACUTE {
            self.gateways.push(DpGateway::new(
              middle + leg_p,
              middle + leg_n,
              true,
            ));
          }

          // :748 to :752
          let leg_p = (p0_p - middle).resize(self.pitch);
          let leg_n = (p0_n - middle).resize(long_leg);

          if angle_between(leg_p, leg_n) != AngleType::ACUTE {
            self.gateways.push(DpGateway::new(
              middle + leg_p,
              middle + leg_n,
              true,
            ));
          }
        }
      }
    }

    // :759
    if build_entries {
      self.build_entries(p0_p, p0_n);
    }
  }

  /// Give every gateway that has none a lead from the two starting
  /// points to its two anchors.
  ///
  /// Port of `buildEntries`,
  /// `pcbnew/router/pns_diff_pair.cpp:583`. It walks the whole set,
  /// including gateways an earlier builder appended, and skips the ones
  /// that already have leads, so running it twice fills in the second
  /// batch and leaves the first alone.
  ///
  /// Each lead is built from the **anchor back to the starting point**
  /// and then reversed. `BuildInitialTrace` is not symmetric in its two
  /// endpoints, so the reversal is part of the geometry and a port must
  /// not turn it into a forward build.
  fn build_entries(&mut self, p0_p: Vec2, p0_n: Vec2) {
    for gateway in &mut self.gateways {
      if gateway.has_entry_lines() {
        continue;
      }

      let lead_p =
        build_trace(gateway.anchor_p, p0_p, gateway.is_diagonal).reversed();
      let lead_n =
        build_trace(gateway.anchor_n, p0_n, gateway.is_diagonal).reversed();

      gateway.set_entry_lines(lead_p, lead_n);
    }
  }

  /// Eight gateways around a free cursor position.
  ///
  /// Port of `BuildForCursor`,
  /// `pcbnew/router/pns_diff_pair.cpp:546`. Four have their anchor line
  /// on a diagonal and four have it axis aligned; the flag each one
  /// carries is the opposite of what `DP_GATEWAY::IsDiagonal` is
  /// documented to mean, which is erratum E7 and is explained on
  /// [`DpGateway::is_diagonal`]. Nothing deduplicates them.
  ///
  /// With `fit_vias` set, each of the eight positions is expanded by a
  /// whole [`DpGateways::build_generic`] in via mode instead, and the
  /// spacing is the via pitch `via_gap + via_diameter` rather than the
  /// trace pitch, so the set can reach eight times the generic fan.
  pub fn build_for_cursor(&mut self, cursor: Vec2) {
    // :548
    let gap = if self.fit_vias {
      self.via_gap + self.via_diameter
    } else {
      self.pitch
    };

    // :550
    for diagonal in [false, true] {
      for i in 0..4 {
        let direction = if diagonal {
          // :566 to :571
          let offset = (gap + 1) / 2 * if i % 2 != 0 { -1 } else { 1 };

          if i / 2 == 0 {
            Vec2::new(offset, 0)
          } else {
            Vec2::new(0, offset)
          }
        } else {
          // :556 to :564
          let mut direction = make_gap_vector(Vec2::new(gap, gap), gap);

          if i % 2 == 0 {
            direction.x = -direction.x;
          }

          if i / 2 == 0 {
            direction.y = -direction.y;
          }

          direction
        };

        if self.fit_vias {
          // :575
          self.build_generic(
            cursor + direction,
            cursor - direction,
            true,
            true,
          );
        } else {
          // :577
          self.gateways.push(DpGateway::new(
            cursor + direction,
            cursor - direction,
            diagonal,
          ));
        }
      }
    }
  }
}

impl DpGateways {
  /// Every exit from the two board objects a pair route starts or ends
  /// on.
  ///
  /// Port of `BuildFromPrimitivePair`,
  /// `pcbnew/router/pns_diff_pair.cpp:418`, the dispatcher on what the
  /// pair sits on:
  ///
  /// - No P object at all: straight to [`DpGateways::build_generic`] on
  ///   the two anchors.
  /// - Two segments or arcs: `buildDpContinuation`, which extends the
  ///   existing tracks.
  /// - Two pads or vias: the fan block below, then
  ///   [`DpGateways::build_generic`] on top of it.
  /// - Anything else, which means one of each: nothing, and an error
  ///   rather than KiCad's silent return (erratum E10).
  ///
  /// The fan block is the classic "walk the pair out of two pads side by
  /// side" staircase. It runs only when the two anchors share a
  /// horizontal, vertical or 45 degree line
  /// ([`check_diagonal_alignment`]). `direction` is perpendicular to the
  /// anchor line and half the fan distance long; `dp` along it and `dv`
  /// along the anchor line are each half of `pad_distance - pitch`,
  /// which is exactly how far the two lanes have to converge, so the two
  /// gateway anchors come out a pitch apart with a two segment lead
  /// behind them. The orthogonal fan gets priority 100 and the diagonal
  /// one 99, which beats everything [`DpGateways::build_generic`]
  /// produces.
  ///
  /// Only the **P** object's shape is ever consulted (`:440`, with a
  /// `TODO(JE) padstacks` above it), so a pair of differently shaped pads
  /// is fanned as though both were shaped like P. And a square pad makes
  /// `diag_fan_distance` zero, so its `k == 1` fan collapses onto the pad
  /// centres and produces a degenerate gateway at priority 99, which
  /// only [`DiffPair::build_initial`]'s own tests reject later.
  pub fn build_from_primitive_pair(
    &mut self,
    world: &World,
    pair: &DpPrimitivePair,
    prefer_diagonal: bool,
  ) -> Result<(), DpGatewayError> {
    // The mask of the two kinds the fan block understands, `pvMask` at
    // `pcbnew/router/pns_diff_pair.cpp:432`.
    let pad_or_via = Kind::SOLID | Kind::VIA;
    let segment_or_arc = Kind::SEGMENT | Kind::ARC;

    // :426
    let prim_p = pair.prim_p();
    let prim_n = pair.prim_n();

    if !prim_p.is_some() {
      self.build_generic(pair.anchor_p(), pair.anchor_n(), true, false);
      return Ok(());
    }

    if !prim_n.is_some() {
      return Err(DpGatewayError::MixedPrimitiveKinds);
    }

    let kind_p = prim_p.kind(world).ok_or(DpGatewayError::UnknownPrimitive)?;
    let kind_n = prim_n.kind(world).ok_or(DpGatewayError::UnknownPrimitive)?;

    let shape = if kind_p.of_kind(pad_or_via) && kind_n.of_kind(pad_or_via) {
      // :434 to :440. The "all layers" pseudo layer, with KiCad's
      // padstack TODO above it.
      prim_p.shape(world).ok_or(DpGatewayError::NoShape)?
    } else if kind_p.of_kind(segment_or_arc) && kind_n.of_kind(segment_or_arc) {
      // :442
      self.build_dp_continuation(world, pair, prefer_diagonal);
      return Ok(());
    } else {
      // :453, reached with a null shape. Erratum E10.
      return Err(DpGatewayError::MixedPrimitiveKinds);
    };

    let p0_p = pair.anchor_p();
    let p0_n = pair.anchor_n();

    // :450
    let major_direction = (p0_p - p0_n).perpendicular();

    // :455
    let (ortho_fan_distance, diagonal_fan_distance) = match shape.as_ref() {
      // :457
      Shape::Circle { .. } => {
        self.build_generic(p0_p, p0_n, true, false);
        return Ok(());
      }
      // :461
      Shape::Rect { size, .. } => fan_distances_of_box(size.x, size.y),
      // :474
      Shape::Segment { seg, width } => {
        let length = (seg.b - seg.a).euclidean_norm();

        (*width + length, length)
      }
      // :484
      Shape::Simple(_) | Shape::Compound(_) => {
        let size = shape.bbox(0).ok_or(DpGatewayError::NoShape)?.size();

        fan_distances_of_box(
          size.x.clamp(0, i64::from(i32::MAX)) as i32,
          size.y.clamp(0, i64::from(i32::MAX)) as i32,
        )
      }
      // :499, `wxFAIL_MSG( "Unsupported starting primitive" )`, after
      // which KiCad carries on with both distances at zero. `SH_ARC`
      // lands here too: the switch has no arc case.
      Shape::LineChain(_) | Shape::Arc(_) => (0, 0),
    };

    // :506
    if check_diagonal_alignment(p0_p, p0_n) {
      let pad_distance = (p0_p - p0_n).euclidean_norm();

      for k in 0..2 {
        // :513 to :516
        let fan_distance = if k == 0 {
          ortho_fan_distance
        } else {
          diagonal_fan_distance
        };
        let direction = make_gap_vector(major_direction, fan_distance);

        // :519 to :521
        let convergence = (pad_distance - self.pitch).max(0);
        let along = make_gap_vector(direction, convergence);
        let across = make_gap_vector(p0_n - p0_p, convergence);

        // :523
        for i in 0..2 {
          let sign = if i != 0 { -1 } else { 1 };

          // :527, :528
          let gateway_p = p0_p + (direction + along) * sign + across;
          let gateway_n = p0_n + (direction + along) * sign - across;

          // :530, :531
          let entry_p = LineChain::from_slice(
            &[p0_p, p0_p + direction * sign, gateway_p],
            false,
          );
          let entry_n = LineChain::from_slice(
            &[p0_n, p0_n + direction * sign, gateway_n],
            false,
          );

          // :533 to :538
          let mut gateway = DpGateway::new(gateway_p, gateway_n, false);

          gateway.set_entry_lines(entry_p, entry_n);
          gateway.set_priority(100 - k);
          self.gateways.push(gateway);
        }
      }
    }

    // :542
    self.build_generic(p0_p, p0_n, true, false);

    Ok(())
  }

  /// Carry on from where two existing tracks end.
  ///
  /// Port of `buildDpContinuation`,
  /// `pcbnew/router/pns_diff_pair.cpp:599`. The first gateway is the
  /// identity, leaving the pair exactly where the tracks end, at priority
  /// 100, which is why continuing an existing pair goes straight on by
  /// default.
  ///
  /// The four angled gateways after it let the pair turn 45 degrees
  /// without the inner lane doubling back: stepping **one** anchor
  /// forward by `pitch * sin(22.5)` rotates the anchor line by 22.5
  /// degrees, half of the turn, so the pair enters it already half
  /// rotated. Each of them sets an entry line on one lane and an empty
  /// chain on the other, which [`check_connection_angle`] passes for the
  /// empty side. The second round at `sin(23.5)` and priority 5 is an
  /// admitted fudge, with KiCad's comment at `:642` pointing at issue
  /// 12459.
  ///
  /// The guard at `:638` accepts a vertical anchor line, a horizontal one
  /// and the `+45` diagonal, but not the `-45` one, where
  /// `delta.x == -delta.y` and the third test is `2 * abs( delta.x )`
  /// rather than near zero. So a pair whose anchor line runs at `-45`
  /// degrees cannot make the assisted turn. That is erratum E6, and it is
  /// reproduced as written.
  fn build_dp_continuation(
    &mut self,
    world: &World,
    pair: &DpPrimitivePair,
    is_diagonal: bool,
  ) {
    /// KiCad's `EPSILON`, 5 nanometres, `pns_diff_pair.cpp:612`.
    const EPSILON: i32 = 5;

    /// KiCad's `SIN_22_5`, `pns_diff_pair.cpp:613`.
    const SIN_22_5: f64 = 0.38268;

    /// KiCad's `SIN_23_5`, `pns_diff_pair.cpp:614`.
    const SIN_23_5: f64 = 0.39875;

    // :601 to :603
    let mut identity =
      DpGateway::new(pair.anchor_p(), pair.anchor_n(), is_diagonal);

    identity.set_priority(100);
    self.gateways.push(identity);

    // :605
    if !pair.directional(world) {
      return;
    }

    // :636, :638
    let delta = pair.anchor_p() - pair.anchor_n();

    if delta.x.abs() >= EPSILON
      && delta.y.abs() >= EPSILON
      && (delta.x - delta.y).abs() >= EPSILON
    {
      return;
    }

    // :640, :644
    for (sine, priority) in [(SIN_22_5, 20), (SIN_23_5, 5)] {
      let length = kiround(f64::from(self.pitch) * sine);

      self.add_angled_gateways(world, pair, is_diagonal, length, priority);
    }
  }

  /// The two one sided gateways of one `addAngledGateways` call.
  ///
  /// The lambda at `pcbnew/router/pns_diff_pair.cpp:616`.
  fn add_angled_gateways(
    &mut self,
    world: &World,
    pair: &DpPrimitivePair,
    is_diagonal: bool,
    length: i32,
    priority: i32,
  ) {
    // :618 to :625
    let advanced_p =
      pair.anchor_p() + pair.dir_p(world).to_vector().resize(length);
    let entry_p = LineChain::from_slice(&[pair.anchor_p(), advanced_p], false);
    let mut gateway_p =
      DpGateway::new(advanced_p, pair.anchor_n(), is_diagonal);

    gateway_p.set_priority(priority);
    gateway_p.set_entry_lines(entry_p, LineChain::new());
    self.gateways.push(gateway_p);

    // :627 to :633
    let advanced_n =
      pair.anchor_n() + pair.dir_n(world).to_vector().resize(length);
    let entry_n = LineChain::from_slice(&[pair.anchor_n(), advanced_n], false);
    let mut gateway_n =
      DpGateway::new(pair.anchor_p(), advanced_n, is_diagonal);

    gateway_n.set_priority(priority);
    gateway_n.set_entry_lines(LineChain::new(), entry_n);
    self.gateways.push(gateway_n);
  }
}

/// The two fan distances of a rectangle or a bounding box.
///
/// The `SH_RECT` arithmetic of `pcbnew/router/pns_diff_pair.cpp:461`,
/// shared with the `SH_SIMPLE` and `SH_COMPOUND` branch at `:484`. The
/// sides are sorted so that `w` is the long one, after which the
/// diagonal fan distance is `w - h` and is therefore **zero for a square
/// pad**.
fn fan_distances_of_box(width: i32, height: i32) -> (i32, i32) {
  let (long, short) = if width < height {
    (height, width)
  } else {
    (width, height)
  };

  ((long + 1) * 3 / 2, long - short)
}

/// Whether two points share a horizontal, vertical or 45 degree line.
///
/// Port of `DP_GATEWAYS::checkDiagonalAlignment`,
/// `pcbnew/router/pns_diff_pair.cpp:383`. Note that `dir.x == dir.y` is
/// true when both are zero, so two coincident points pass; nothing else
/// guards against that.
pub fn check_diagonal_alignment(first: Vec2, second: Vec2) -> bool {
  let delta = Vec2::new((first.x - second.x).abs(), (first.y - second.y).abs());

  (delta.x == 0 && delta.y != 0)
    || (delta.x == delta.y)
    || (delta.y == 0 && delta.x != 0)
}

/// A vector along a direction of **half** a requested length.
///
/// Port of the file static `makeGapVector`,
/// `pcbnew/router/pns_diff_pair.cpp:401`. The halving is the whole point:
/// `p + make_gap_vector(d, g)` and `p - make_gap_vector(d, g)` end up `g`
/// apart, not `2 * g`. The loop exists because [`Vec2::resize`] rounds,
/// so doubling `resize(length / 2)` can fall a nanometre short; it
/// lengthens by one until it does not. A zero direction comes straight
/// back, and the caller silently gets a degenerate gateway.
pub fn make_gap_vector(direction: Vec2, length: i32) -> Vec2 {
  if direction.euclidean_norm() == 0 {
    return direction;
  }

  let mut half = length / 2;

  loop {
    let candidate = direction.resize(half);

    half += 1;

    if (candidate * 2).euclidean_norm() >= length {
      return candidate;
    }
  }
}

/// One of `BuildGeneric`'s probe segments.
///
/// The eight `SEG( p + (-dx, -dy), p + (dx, dy) )` of
/// `pcbnew/router/pns_diff_pair.cpp:658` to `:665`. They are used only as
/// infinite lines.
fn probe(point: Vec2, half_x: i32, half_y: i32) -> Seg {
  let half = Vec2::new(half_x, half_y);

  Seg::new(point - half, point + half)
}

/// The kind of angle between two vectors.
///
/// Port of the file static `angle`,
/// `pcbnew/router/pns_diff_pair.cpp:173`.
fn angle_between(first: Vec2, second: Vec2) -> AngleType {
  Direction45::from_vector(first, false)
    .angle(Direction45::from_vector(second, false))
}

// ---------------------------------------------------------------------
// The fit
// ---------------------------------------------------------------------

/// Pick the best route between two gateway sets.
///
/// Port of `DP_GATEWAYS::FitGateways`,
/// `pcbnew/router/pns_diff_pair.cpp:336`. KiCad calls it on an instance
/// and passes that same instance as the first argument
/// (`pns_diff_pair_placer.cpp:734`), so `this` contributes nothing but
/// `m_gap`; it is a free function here, with the pitch explicit.
///
/// The scoring is not a measure of route quality. It is the sum of the
/// two gateway priorities plus a posture term, and geometry contributes
/// nothing:
///
/// - The guard is `score >= best_score`, not `>`, so among equal scoring
///   candidates the **last one that builds** wins, and the push order of
///   every builder is therefore part of the answer.
/// - `preferred` runs false before true and false scores `-3`, so the
///   requested posture is tried second and takes the tie unless it fails
///   to build. The `-3` only bites against priority differences smaller
///   than 3, which is exactly the 100 against 99 of the fan and the 20
///   against 5 of the angled continuation gateways.
/// - [`DiffPair::build_initial`] runs **only** when the score already
///   ties or beats the best, so a high scoring pair that fails to build
///   does not stop a later low scoring one, but an early low scoring one
///   is skipped outright.
///
/// The answer is a fresh pair carrying the fitted lanes and, through
/// [`DiffPair::set_gap`], the pitch with its 10 micrometre tolerance,
/// which is what KiCad writes into the caller's pair at `:374`. A caller
/// that has to keep its own pair's width, nets and vias transplants the
/// lanes with [`DiffPair::set_shape_from`], which is KiCad's other
/// `SetShape` overload.
pub fn fit_gateways(
  pitch: i32,
  entry: &DpGateways,
  target: &DpGateways,
  prefer_diagonal: bool,
) -> Option<DiffPair> {
  let mut best: Option<(LineChain, LineChain)> = None;

  // :341
  let mut best_score = -1000;

  for gateway_entry in entry.gateways() {
    for gateway_target in target.gateways() {
      // :348
      for preferred in [false, true] {
        // :350 to :352
        let score = (if preferred { 0 } else { -3 })
          + gateway_entry.priority()
          + gateway_target.priority();

        // :354
        if score < best_score {
          continue;
        }

        // :356 to :359
        let mut candidate = DiffPair::with_gap_constraint(pitch);
        let posture = if preferred {
          prefer_diagonal
        } else {
          !prefer_diagonal
        };

        if candidate.build_initial(gateway_entry, gateway_target, posture) {
          // :361 to :364
          best = Some((candidate.p.clone(), candidate.n.clone()));
          best_score = score;
        }
      }
    }
  }

  // :372 to :377
  best.map(|(p, n)| {
    let mut fitted = DiffPair::new();

    fitted.set_gap(pitch);
    fitted.set_shape(p, n, false);
    fitted
  })
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::item::{Item, Via, ViaType};

  /// The width of one lane in every measurement test.
  const WIDTH: i32 = 200_000;

  /// The copper gap between the lanes, edge to edge.
  const GAP: i32 = 200_000;

  /// The centre to centre pitch, `GAP + WIDTH`.
  const PITCH: i32 = GAP + WIDTH;

  /// A pair of two straight lanes at a given centre to centre distance,
  /// with the edge to edge gap and its tolerance set the way the placer
  /// sets them after a fit.
  fn pair_of(p: &[Vec2], n: &[Vec2]) -> DiffPair {
    let mut pair = DiffPair::from_chains(
      LineChain::from_slice(p, false),
      LineChain::from_slice(n, false),
      0,
    );

    pair.set_width(WIDTH);
    pair.set_gap(GAP);
    pair
  }

  /// A horizontal lane from `x0` to `x1` at `y`.
  fn lane(x0: i32, x1: i32, y: i32) -> [Vec2; 2] {
    [Vec2::new(x0, y), Vec2::new(x1, y)]
  }

  #[test]
  fn gap_constraint_matches_within_an_asymmetric_band() {
    let exact = GapConstraint::exact(200_000);

    assert_eq!(exact.value(), 200_000);
    assert!(exact.matches(200_000));
    assert!(!exact.matches(199_999));
    assert!(!exact.matches(200_001));

    let ranged = GapConstraint::with_tolerance(200_000, 5, 7);

    assert!(ranged.matches(200_005));
    assert!(!ranged.matches(200_006));
    assert!(ranged.matches(199_993));
    assert!(!ranged.matches(199_992));
  }

  /// `SetGap` is the only path that widens the constraint
  /// (`pns_diff_pair.h:434`); the three gap taking constructors leave the
  /// tolerance at zero (`:277`, `:295`, `:313`).
  #[test]
  fn only_set_gap_widens_the_constraint() {
    let constructed = DiffPair::with_gap_constraint(200_000);

    assert_eq!(constructed.gap_constraint().tolerance_plus(), 0);
    assert_eq!(constructed.gap_constraint().tolerance_minus(), 0);
    assert_eq!(constructed.gap_constraint().value(), 200_000);
    assert_eq!(constructed.gap(), 0, "the constructor writes no gap");

    let mut assigned = DiffPair::new();

    assigned.set_gap(200_000);
    assert_eq!(assigned.gap(), 200_000);
    assert_eq!(assigned.gap_constraint().tolerance_plus(), 10_000);
    assert_eq!(assigned.gap_constraint().tolerance_minus(), 10_000);
  }

  #[test]
  fn parallel_lanes_couple_over_their_whole_length() {
    let pair = pair_of(&lane(0, 1_000_000, 0), &lane(0, 1_000_000, PITCH));
    let coupled = pair.coupled_segment_pairs();

    assert_eq!(coupled.len(), 1);
    assert_eq!(
      coupled[0].coupled_p,
      Seg::new(Vec2::new(0, 0), Vec2::new(1_000_000, 0))
    );
    assert_eq!(
      coupled[0].coupled_n,
      Seg::new(Vec2::new(0, PITCH), Vec2::new(1_000_000, PITCH))
    );
    assert_eq!(coupled[0].index_p, 0);
    assert_eq!(coupled[0].index_n, 0);
    assert_eq!(pair.coupled_length(), 1_000_000);
  }

  #[test]
  fn offset_lanes_couple_over_the_overlap_only() {
    let pair =
      pair_of(&lane(0, 1_000_000, 0), &lane(500_000, 1_500_000, PITCH));
    let coupled = pair.coupled_segment_pairs();

    assert_eq!(coupled.len(), 1);
    assert_eq!(
      coupled[0].coupled_p,
      Seg::new(Vec2::new(500_000, 0), Vec2::new(1_000_000, 0))
    );
    assert_eq!(
      coupled[0].coupled_n,
      Seg::new(Vec2::new(500_000, PITCH), Vec2::new(1_000_000, PITCH))
    );
    assert_eq!(pair.coupled_length(), 500_000);
  }

  #[test]
  fn lanes_that_do_not_face_each_other_do_not_couple() {
    let pair =
      pair_of(&lane(0, 1_000_000, 0), &lane(2_000_000, 3_000_000, PITCH));

    assert!(
      common_parallel_projection(
        pair.chain_p().segment(0),
        pair.chain_n().segment(0),
      )
      .is_none()
    );
    assert!(pair.coupled_segment_pairs().is_empty());
    assert_eq!(pair.coupled_length(), 0);
  }

  /// `SEG::ApproxParallel` is direction blind, so a reversed lane couples
  /// exactly as the forward one does. Easy to break with a direction
  /// test, so it is pinned.
  #[test]
  fn an_antiparallel_lane_still_couples() {
    let pair = pair_of(&lane(0, 1_000_000, 0), &lane(1_000_000, 0, PITCH));
    let coupled = pair.coupled_segment_pairs();

    assert_eq!(coupled.len(), 1);
    assert_eq!(
      coupled[0].coupled_p,
      Seg::new(Vec2::new(0, 0), Vec2::new(1_000_000, 0))
    );
    assert_eq!(pair.coupled_length(), 1_000_000);
  }

  /// The plus or minus 10 micrometre band of `SetGap`, at both ends.
  #[test]
  fn the_gap_tolerance_bounds_what_counts_as_coupled() {
    let inside =
      pair_of(&lane(0, 1_000_000, 0), &lane(0, 1_000_000, PITCH + 10_000));
    let outside =
      pair_of(&lane(0, 1_000_000, 0), &lane(0, 1_000_000, PITCH + 10_001));

    assert_eq!(inside.coupled_segment_pairs().len(), 1);
    assert!(outside.coupled_segment_pairs().is_empty());
  }

  /// A pair on the 45 degree diagonal, which is where `rescale`'s
  /// rounding shows up: the offset of 282843 nanometres per axis is
  /// 400000 apart to the nanometre once `SEG::Distance` truncates its
  /// square root.
  #[test]
  fn a_diagonal_pair_couples_over_its_whole_length() {
    let pair = pair_of(
      &[Vec2::new(0, 0), Vec2::new(1_000_000, 1_000_000)],
      &[Vec2::new(-282_843, 282_843), Vec2::new(717_157, 1_282_843)],
    );
    let coupled = pair.coupled_segment_pairs();

    assert_eq!(coupled.len(), 1);
    assert_eq!(
      coupled[0].coupled_p,
      Seg::new(Vec2::new(0, 0), Vec2::new(1_000_000, 1_000_000))
    );
    assert_eq!(
      coupled[0].coupled_n,
      Seg::new(Vec2::new(-282_843, 282_843), Vec2::new(717_157, 1_282_843))
    );
  }

  #[test]
  fn skew_is_the_difference_of_the_two_lane_lengths() {
    let pair = pair_of(&lane(0, 1_000_000, 0), &lane(0, 700_000, PITCH));

    assert_eq!(pair.skew(), 300_000);

    let same = pair_of(&lane(0, 1_000_000, 0), &lane(0, 1_000_000, PITCH));

    assert_eq!(same.skew(), 0);
  }

  /// The optimizer's overload measures chains the pair does not hold, so
  /// that a hypothetical shape can be scored without writing it in.
  #[test]
  fn coupled_length_of_chains_scores_a_shape_the_pair_does_not_hold() {
    let pair = pair_of(&lane(0, 1_000_000, 0), &lane(0, 1_000_000, PITCH));
    let shorter_p = LineChain::from_slice(&lane(0, 400_000, 0), false);
    let shorter_n = LineChain::from_slice(&lane(0, 400_000, PITCH), false);

    assert_eq!(
      pair.coupled_length_of_chains(&shorter_p, &shorter_n),
      400_000
    );
    assert_eq!(pair.coupled_length(), 1_000_000, "the pair is unchanged");
  }

  /// `checkGap` is a minimum test: it rejects a candidate whose lanes
  /// pinch anywhere, and says nothing about lanes that stay coupled.
  #[test]
  fn check_gap_rejects_a_corner_that_pinches() {
    let outer = LineChain::from_slice(
      &[
        Vec2::new(0, 0),
        Vec2::new(1_000_000, 0),
        Vec2::new(1_000_000, 1_000_000),
      ],
      false,
    );
    let clear = LineChain::from_slice(
      &[
        Vec2::new(0, PITCH),
        Vec2::new(600_000, PITCH),
        Vec2::new(600_000, 1_000_000),
      ],
      false,
    );
    let pinching = LineChain::from_slice(
      &[
        Vec2::new(0, PITCH),
        Vec2::new(800_000, PITCH),
        Vec2::new(800_000, 1_000_000),
      ],
      false,
    );

    assert!(check_gap(&outer, &clear, PITCH));
    assert!(!check_gap(&outer, &pinching, PITCH));
  }

  /// The `- 100` slack of `pns_diff_pair.cpp:184`, at both ends.
  #[test]
  fn check_gap_allows_a_hundred_nanometres_of_slack() {
    let p = LineChain::from_slice(&lane(0, 1_000_000, 0), false);
    let just_inside =
      LineChain::from_slice(&lane(0, 1_000_000, PITCH - 100), false);
    let just_outside =
      LineChain::from_slice(&lane(0, 1_000_000, PITCH - 101), false);

    assert!(check_gap(&p, &just_inside, PITCH));
    assert!(!check_gap(&p, &just_outside, PITCH));
  }

  /// Doubling the answer reaches the requested length and overshoots by
  /// at most two nanometres, whatever the direction.
  #[test]
  fn make_gap_vector_halves_and_rounds_up() {
    for direction in [
      Vec2::new(1, 0),
      Vec2::new(0, 1),
      Vec2::new(1, 1),
      Vec2::new(-3, 7),
      Vec2::new(400_000, 0),
      Vec2::new(282_843, 282_843),
      Vec2::new(-1_000_000, 250_000),
    ] {
      for length in [0, 1, 2, 7, 399, 400_000, 400_001] {
        let doubled = (make_gap_vector(direction, length) * 2).euclidean_norm();

        assert!(
          doubled >= length && doubled <= length + 2,
          "{direction:?} at {length}: doubled to {doubled}"
        );
      }
    }

    assert_eq!(
      make_gap_vector(Vec2::new(0, 0), 400_000),
      Vec2::new(0, 0),
      "a zero direction comes straight back"
    );
  }

  /// An empty chain on either side of a lane passes that lane, which is
  /// what makes `buildDpContinuation`'s one sided gateways work.
  #[test]
  fn an_empty_lane_passes_the_connection_angle() {
    let east = LineChain::from_slice(&lane(0, 1_000_000, 0), false);
    let south = LineChain::from_slice(
      &[Vec2::new(1_000_000, 0), Vec2::new(1_000_000, 1_000_000)],
      false,
    );
    let empty = LineChain::new();
    let mask = AngleType::STRAIGHT | AngleType::OBTUSE;

    assert!(
      !check_connection_angle(&east, &east, &south, &south, mask),
      "a right angle is in neither straight nor obtuse"
    );
    assert!(
      check_connection_angle(&east, &empty, &east, &empty, mask),
      "the empty N lane passes and the P lane runs straight on"
    );
    assert!(
      !check_connection_angle(&east, &empty, &south, &empty, mask),
      "an empty lane does not rescue the other one"
    );
    assert!(
      check_connection_angle(&empty, &empty, &south, &south, mask),
      "two empty lanes pass whatever they are joined to"
    );
  }

  /// The two line views carry the pair's width, nets and layer, and the
  /// end vias only while the pair claims to have them.
  #[test]
  fn the_line_views_carry_the_pairs_properties() {
    let mut pair = pair_of(&lane(0, 1_000_000, 0), &lane(0, 1_000_000, PITCH));

    pair.set_nets(Some(NetId(1)), Some(NetId(2)));
    pair.set_layer(3);

    let line_p = pair.p_line();

    assert_eq!(line_p.width(), WIDTH);
    assert_eq!(line_p.net(), Some(NetId(1)));
    assert_eq!(line_p.layer(), 3);
    assert_eq!(line_p.shape().points(), pair.chain_p().points());
    assert_eq!(pair.n_line().net(), Some(NetId(2)));
    assert!(line_p.via().is_none());
  }

  /// `RemoveVias` clears the flag and leaves the two vias where they
  /// were (`pns_diff_pair.h:454`), which is benign because every reader
  /// guards on the flag.
  #[test]
  fn remove_vias_clears_the_flag_and_keeps_the_vias() {
    let mut pair = pair_of(&lane(0, 1_000_000, 0), &lane(0, 1_000_000, PITCH));

    pair.append_vias(
      via_at(Vec2::new(1_000_000, 0)),
      via_at(Vec2::new(1_000_000, PITCH)),
    );

    assert!(pair.ends_with_vias());
    assert!(pair.p_line().via().is_some());
    assert!(pair.n_line().via().is_some());

    pair.set_via_diameter(700_000);
    pair.set_via_drill(350_000);
    pair.remove_vias();

    assert!(!pair.ends_with_vias());
    assert!(pair.p_line().via().is_none());
    assert!(
      pair.via_p().is_some(),
      "the via itself survives, as it does in C++"
    );

    let ItemBody::Via(body) = pair.via_p().expect("still there").body() else {
      panic!("the P end via is a via");
    };

    assert_eq!(body.drill(), 350_000);
  }

  /// A via item, standing in for KiCad's `VIA` value member.
  fn via_at(pos: Vec2) -> Item {
    let mut item = Item::new(
      1,
      ItemBody::Via(Via::new(pos, 600_000, 300_000, ViaType::Through)),
    );

    item.set_layers_and_flash_all(LayerRange::new(0, 1));
    item
  }

  /// `SetShape( const DIFF_PAIR& )` copies the two lanes and nothing
  /// else, which is what `tryWalkDp` relies on.
  #[test]
  fn set_shape_from_keeps_everything_but_the_lanes() {
    let mut pair = pair_of(&lane(0, 1_000_000, 0), &lane(0, 1_000_000, PITCH));
    let other = pair_of(&lane(0, 500_000, 0), &lane(0, 500_000, PITCH));

    pair.set_nets(Some(NetId(1)), Some(NetId(2)));
    pair.set_shape_from(&other);

    assert_eq!(pair.chain_p().length(), 500_000);
    assert_eq!(pair.gap(), GAP);
    assert_eq!(pair.width(), WIDTH);
    assert_eq!(pair.nets(), (Some(NetId(1)), Some(NetId(2))));
  }

  /// `SetShape( aP, aN, true )` swaps which chain is which.
  #[test]
  fn set_shape_can_swap_the_lanes() {
    let mut pair = DiffPair::new();

    pair.set_shape(
      LineChain::from_slice(&lane(0, 1_000_000, 0), false),
      LineChain::from_slice(&lane(0, 500_000, PITCH), false),
      true,
    );

    assert_eq!(pair.chain_p().length(), 500_000);
    assert_eq!(pair.chain_n().length(), 1_000_000);
  }

  /// Two points are aligned when they share a horizontal, a vertical or
  /// a 45 degree line, and two coincident points pass because
  /// `dir.x == dir.y` holds for two zeroes.
  #[test]
  fn diagonal_alignment_accepts_the_three_regimes_and_a_coincidence() {
    let origin = Vec2::new(0, 0);

    assert!(check_diagonal_alignment(origin, Vec2::new(0, 400_000)));
    assert!(check_diagonal_alignment(origin, Vec2::new(400_000, 0)));
    assert!(check_diagonal_alignment(
      origin,
      Vec2::new(400_000, 400_000)
    ));
    assert!(check_diagonal_alignment(
      origin,
      Vec2::new(-400_000, 400_000)
    ));
    assert!(check_diagonal_alignment(origin, origin));
    assert!(!check_diagonal_alignment(
      origin,
      Vec2::new(400_000, 200_000)
    ));
  }
}
