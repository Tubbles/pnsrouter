// SPDX-License-Identifier: GPL-3.0-or-later

//! Polylines, the container the router lives in.
//!
//! [`LineChain`] is the port of KiCad's `SHAPE_LINE_CHAIN`,
//! `libs/kimath/include/geometry/shape_line_chain.h:77`. This module is
//! part 1 of the port: the container, its editing members and the queries
//! that read points and segments. Intersection, collision, distance,
//! nearest point, point containment and self intersection are part 2 and
//! are deliberately absent.
//!
//! Three structural decisions come from `DESIGN.md` section 3 and
//! `doc/reference/kicad/01-geometry.md` sections 6.8 and 14.3:
//!
//! - There is no bounding box cache. KiCad's `m_bbox` (`slc.h:1007`) is
//!   written by `Append` and `Move` and never invalidated by `Remove`,
//!   `Insert`, `Replace`, `SetPoint`, `Simplify` or `Clear`, while
//!   `BBox()` recomputes from scratch and ignores it, so the two can
//!   disagree and a stale cache turns into a silent false negative in
//!   `PointInside`. [`LineChain::bbox`] recomputes.
//! - Indices are `usize` and are checked. KiCad's `CPoint` (`slc.h:420`)
//!   accepts a negative index and an index equal to `PointCount()` with a
//!   single step wrap, and `ArcIndex`, `Arc` and `CLastPoint` are
//!   unchecked. Callers here use [`LineChain::last_point`],
//!   [`LineChain::segment`] and, where they are transcribing a KiCad call
//!   site that passes a negative index, [`LineChain::normalize_index`].
//! - Milestone 1 has no arcs, so the chain is a plain point vector.
//!
//! KiCad carries two more vectors that are not here yet. `m_shapes`
//! (`slc.h:989`) is parallel to `m_points` and says, per vertex, which arc
//! that vertex belongs to; `m_arcs` (`slc.h:991`) holds the arcs
//! themselves. Note 01 section 14.3 plans them as a `Vec<ArcRef>` parallel
//! to the points plus a `Vec<ShapeArc>`, where `ArcRef` is an enum rather
//! than KiCad's `(ssize_t, ssize_t)` pair with its `-1` sentinel. Every
//! signature in this module is chosen so that adding those two fields is a
//! body change and not an API change: nothing returns or takes a point
//! index that would have to grow an arc companion, [`LineChain::points`]
//! is the only view into the storage, and the members whose KiCad
//! counterparts branch on arcs ([`LineChain::set_closed`],
//! [`LineChain::remove_range`], [`LineChain::slice`],
//! [`LineChain::split`], [`LineChain::simplify`],
//! [`LineChain::simplify2`], [`LineChain::length`]) keep KiCad's control
//! flow so the arc branches can be filled in where KiCad has them.
//!
//! `m_accuracy` (`slc.h:994`) is not ported: every constructor sets it to
//! zero and nothing ever reads it.

use std::fmt;

use crate::geometry::box2::Box2;
use crate::geometry::seg::Seg;
use crate::geometry::vec2::{Vec2, Vec2L};

/// Why [`LineChain::slice`] could not produce a subchain.
///
/// KiCad's `Slice` answers all five of its failure cases with an empty
/// chain (`libs/kimath/src/geometry/shape_line_chain.cpp:1429` to
/// `:1433`), which a caller cannot tell from a legitimately empty result.
/// Note 01 section 14.3 asks for a `Result` instead.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum SliceError {
  /// An endpoint of the range is not a point of the chain.
  IndexOutOfRange {
    /// The offending index.
    index: usize,
    /// The number of points the chain has.
    point_count: usize,
  },
  /// The range runs backwards.
  ///
  /// KiCad requires `aEndIndex >= aStartIndex` after normalisation
  /// (`shape_line_chain.cpp:1433`), which is what makes a slice unable to
  /// cross the seam of a closed chain.
  EndBeforeStart {
    /// The first index of the requested range.
    start: usize,
    /// The last index of the requested range.
    end: usize,
  },
}

impl fmt::Display for SliceError {
  fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      SliceError::IndexOutOfRange { index, point_count } => write!(
        formatter,
        "point index {index} is out of range for {point_count} points"
      ),
      SliceError::EndBeforeStart { start, end } => {
        write!(formatter, "slice end {end} is before slice start {start}")
      }
    }
  }
}

impl std::error::Error for SliceError {}

/// A polyline with an explicit closed flag and a nominal width.
///
/// Port of `SHAPE_LINE_CHAIN`,
/// `libs/kimath/include/geometry/shape_line_chain.h:77`. A closed chain
/// carries one more segment than an open one, the closing `last -> first`,
/// so [`LineChain::segment_count`] equals [`LineChain::point_count`]
/// rather than one less (`slc.h:327`). The width is nominal: it is what
/// [`LineChain::bbox`] inflates by and what the router copies onto the
/// segments it commits, and it takes no part in equality.
///
/// The arc vectors described in the module documentation are not here yet.
/// Milestone 1 routes straight segments only.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct LineChain {
  /// The vertices, in chain order.
  ///
  /// Port of `m_points`, `libs/kimath/include/geometry/shape_line_chain.h:973`.
  points: Vec<Vec2>,
  /// Whether a closing segment joins the last point back to the first.
  ///
  /// Port of `m_closed`, `libs/kimath/include/geometry/shape_line_chain.h:997`.
  closed: bool,
  /// The nominal width in nanometres.
  ///
  /// Port of `m_width`, `libs/kimath/include/geometry/shape_line_chain.h:1004`.
  width: i32,
}

impl LineChain {
  /// The distance below which [`LineChain::split`] accepts a point as
  /// lying on a segment, in nanometres.
  ///
  /// Port of the `min_dist` local in `SHAPE_LINE_CHAIN::Split`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1184`. The test there
  /// is `Distance( aP ) < min_dist`, so only distances of 0 and 1 nm hit.
  /// Note 01 section 13 lists it among the constants that must be
  /// transplanted verbatim.
  pub const SPLIT_HIT_THRESHOLD: i32 = 2;

  /// The colinearity tolerance of [`LineChain::simplify2`], in
  /// nanometres.
  ///
  /// Port of the hard coded `<= 1` at
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2972`. This is the one
  /// nanometre of slack the optimizer's convergence depends on, which is
  /// why both simplifiers exist (note 01 section 6.5).
  pub const SIMPLIFY2_TOLERANCE: i32 = 1;

  // ---------------------------------------------------------------
  // Construction
  // ---------------------------------------------------------------

  /// An empty, open chain of width zero.
  ///
  /// Port of `SHAPE_LINE_CHAIN()`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:148`.
  pub const fn new() -> Self {
    Self {
      points: Vec::new(),
      closed: false,
      width: 0,
    }
  }

  /// A chain over an owned point vector.
  ///
  /// Port of `SHAPE_LINE_CHAIN( const std::vector<VECTOR2I>&, bool )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:78`, which assigns the
  /// vector and then goes through `SetClosed`. Closing therefore drops a
  /// trailing point equal to the first, exactly as
  /// [`LineChain::set_closed`] does. The width starts at zero.
  pub fn from_points(points: Vec<Vec2>, closed: bool) -> Self {
    let mut chain = Self {
      points,
      closed: false,
      width: 0,
    };

    chain.set_closed(closed);
    chain
  }

  /// A chain over a borrowed point slice.
  ///
  /// The borrowing form of [`LineChain::from_points`]; C++ takes the
  /// vector by const reference and copies it.
  pub fn from_slice(points: &[Vec2], closed: bool) -> Self {
    Self::from_points(points.to_vec(), closed)
  }

  /// The two point chain of a segment.
  ///
  /// Port of the `SHAPE_LINE_CHAIN( { m_seg.GetSeg().A, m_seg.GetSeg().B } )`
  /// the router builds for every stored segment,
  /// `pcbnew/router/pns_segment.h:108`. The segment's parent shape index
  /// is not carried: it names a position in the shape the segment came
  /// from, which the new chain is not.
  pub fn from_seg(seg: &Seg) -> Self {
    Self::from_points(vec![seg.a, seg.b], false)
  }

  // ---------------------------------------------------------------
  // Counts and access
  // ---------------------------------------------------------------

  /// The number of vertices.
  ///
  /// Port of `PointCount`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:369`.
  pub fn point_count(&self) -> usize {
    self.points.len()
  }

  /// Whether the chain holds no points at all.
  ///
  /// KiCad spells this `PointCount() == 0`; `SHAPE_LINE_CHAIN` has no
  /// `IsEmpty`.
  pub fn is_empty(&self) -> bool {
    self.points.is_empty()
  }

  /// The number of segments, counting the closing one.
  ///
  /// Port of `SegmentCount`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:327`, which is
  /// `max( 0, PointCount() - 1 + closed )`. A closed chain therefore has
  /// as many segments as points, and a chain of one or no points has none
  /// unless it is closed, in which case a single point yields the
  /// degenerate segment `p0 -> p0`. KiCad reaches the `max( 0, ... )`
  /// through a `size_t` that underflows into an `int`; this computes it in
  /// signed arithmetic, as note 01 section 14.3 requires.
  pub fn segment_count(&self) -> usize {
    let points = self.points.len() as isize;
    let count = points - 1 + isize::from(self.closed);

    count.max(0) as usize
  }

  /// Whether a closing segment joins the last point back to the first.
  ///
  /// Port of `IsClosed`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:297`.
  pub fn is_closed(&self) -> bool {
    self.closed
  }

  /// Open or close the chain.
  ///
  /// Port of `SetClosed`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:288`, which is a
  /// flag assignment followed by `mergeFirstLastPointIfNeeded`
  /// (`shape_line_chain.cpp:214`). Closing a chain of more than one point
  /// whose last point equals its first drops that last point, because the
  /// closing segment already carries it. Opening one is a no operation
  /// until arcs exist: KiCad's other branch only fires for a vertex shared
  /// between two arcs.
  pub fn set_closed(&mut self, closed: bool) {
    self.closed = closed;
    self.merge_first_last_point_if_needed();
  }

  /// The nominal width in nanometres.
  ///
  /// Port of `Width`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:316`.
  pub fn width(&self) -> i32 {
    self.width
  }

  /// Set the nominal width in nanometres.
  ///
  /// Port of `SetWidth`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:306`.
  pub fn set_width(&mut self, width: i32) {
    self.width = width;
  }

  /// The point at an index.
  ///
  /// Deviation from `CPoint`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:420`, which adds or
  /// subtracts `PointCount()` once so that `-1` names the last point and
  /// `PointCount()` names the first, and which indexes out of bounds for
  /// anything further out. This panics on any index that is not a point of
  /// the chain. Transcribe a KiCad call site that relies on the wrap with
  /// [`LineChain::last_point`], [`LineChain::segment`] or
  /// [`LineChain::normalize_index`].
  ///
  /// # Panics
  ///
  /// When `index` is not less than [`LineChain::point_count`].
  pub fn point(&self, index: usize) -> Vec2 {
    assert!(
      index < self.points.len(),
      "point index {index} is out of range for a chain of {} points",
      self.points.len()
    );

    self.points[index]
  }

  /// The last point, or `None` for an empty chain.
  ///
  /// Port of `CLastPoint`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:435`, which indexes
  /// `m_points[PointCount() - 1]` and is therefore undefined on an empty
  /// chain.
  pub fn last_point(&self) -> Option<Vec2> {
    self.points.last().copied()
  }

  /// Every point, in chain order.
  ///
  /// Port of `CPoints`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:428`.
  pub fn points(&self) -> &[Vec2] {
    &self.points
  }

  /// The segment at an index, with [`Seg::index`] filled in.
  ///
  /// Port of `Segment`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1285`. The last
  /// segment of a closed chain runs from the last point back to the first.
  /// KiCad wraps a negative index by `SegmentCount()` and answers a failed
  /// bounds check with the degenerate `SEG( back, back )`; this panics
  /// instead.
  ///
  /// # Panics
  ///
  /// When `index` is not less than [`LineChain::segment_count`].
  pub fn segment(&self, index: usize) -> Seg {
    let segment_count = self.segment_count();

    assert!(
      index < segment_count,
      "segment index {index} is out of range for {segment_count} segments"
    );

    // A chain never holds anywhere near two billion points, so the cast
    // that fills in KiCad's `int` shape index cannot truncate.
    let shape_index = index as i32;

    if index == self.points.len() - 1 && self.closed {
      Seg::with_index(self.points[index], self.points[0], shape_index)
    } else {
      Seg::with_index(self.points[index], self.points[index + 1], shape_index)
    }
  }

  /// KiCad's signed point index turned into a checked `usize`.
  ///
  /// Port of the normalisation the mutators share, for instance
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1086` in `Remove`,
  /// `:1009` in `Replace` and `:1422` in `Slice`: a negative index has
  /// `PointCount()` added to it once, so `-1` is the last point and
  /// `-PointCount()` the first, and anything further out stays negative
  /// and fails the bounds check. Returns `None` for an index that is not a
  /// point of the chain.
  ///
  /// The router passes negative indices at three call sites,
  /// `tail.Slice( -threshold, -1 )`
  /// (`pcbnew/router/pns_line_placer.cpp:1072`),
  /// `tail.Replace( -threshold, -1, ... )` (`:1091`) and
  /// `m_line.Replace( -2, -1, best )` (`pcbnew/router/pns_line.cpp:1392`).
  /// Every index in this API is a `usize`, so those transcribe as
  /// `chain.normalize_index(-1)` rather than as a second signed overload
  /// of each mutator.
  pub fn normalize_index(&self, index: isize) -> Option<usize> {
    let point_count = self.points.len() as isize;
    let normalized = if index < 0 {
      index + point_count
    } else {
      index
    };

    if normalized < 0 || normalized >= point_count {
      return None;
    }

    Some(normalized as usize)
  }

  // ---------------------------------------------------------------
  // Editing
  // ---------------------------------------------------------------

  /// Drop every point and open the chain.
  ///
  /// Port of `Clear`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:274`, which clears
  /// the points and sets `m_closed = false` but leaves `m_width` alone.
  /// The width surviving a `Clear` is note 01 section 6.8 item 7; it is
  /// reproduced because the placer clears and refills the head chain
  /// between mouse moves and expects to keep the track width it set.
  pub fn clear(&mut self) {
    self.points.clear();
    self.closed = false;
  }

  /// Append a point, skipping it when it equals the last one.
  ///
  /// Port of `Append( const VECTOR2I&, bool aAllowDuplication = false )`
  /// with the default argument,
  /// `libs/kimath/include/geometry/shape_line_chain.h:534`. Silently
  /// dropping a duplicate of the last point looks like a wart and is
  /// deliberate: the placer relies on it, so note 01 section 14.3 lists it
  /// among the behaviours to keep exactly. N calls do not imply N points.
  pub fn append(&mut self, point: Vec2) {
    if self.points.last() != Some(&point) {
      self.points.push(point);
    }
  }

  /// Append a point even when it equals the last one.
  ///
  /// Port of `Append( const VECTOR2I&, true )`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:534`. The router
  /// asks for the duplicate at two call sites,
  /// `pcbnew/router/pns_router.cpp:313` and
  /// `pcbnew/router/pns_line_placer.cpp:1155`, both of which build a
  /// degenerate two point chain out of one position.
  pub fn append_allow_duplicate(&mut self, point: Vec2) {
    self.points.push(point);
  }

  /// Append another chain's points at the end.
  ///
  /// Port of `Append( const SHAPE_LINE_CHAIN& )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1546`. Only the
  /// other chain's first point is subject to duplicate suppression
  /// (`:1571`); the rest are copied as they stand, so a chain that
  /// contains a duplicate keeps it. The other chain's closed flag and
  /// width are ignored, and the merge of a trailing point equal to the
  /// first runs afterwards (`:1606`), which matters when this chain is
  /// closed.
  pub fn append_chain(&mut self, other: &LineChain) {
    if other.points.is_empty() {
      return;
    }

    if self.points.is_empty() || self.points.last() != Some(&other.points[0]) {
      self.points.push(other.points[0]);
    }

    self.points.extend_from_slice(&other.points[1..]);
    self.merge_first_last_point_if_needed();
  }

  /// Insert a point before the point at an index.
  ///
  /// Port of `Insert( size_t, const VECTOR2I& )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1637`. An index equal
  /// to [`LineChain::point_count`] delegates to [`LineChain::append`],
  /// which means appending through `insert` suppresses a duplicate while
  /// inserting anywhere else does not. That asymmetry is KiCad's, and it
  /// carries a `todo` at `shape_line_chain.cpp:1650` saying so.
  ///
  /// # Panics
  ///
  /// When `index` is greater than [`LineChain::point_count`].
  pub fn insert(&mut self, index: usize, point: Vec2) {
    if index == self.points.len() {
      self.append(point);
      return;
    }

    assert!(
      index < self.points.len(),
      "insert index {index} is out of range for a chain of {} points",
      self.points.len()
    );

    self.points.insert(index, point);
  }

  /// Remove the point at an index.
  ///
  /// Port of `Remove( int )`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:588`, which is
  /// [`LineChain::remove_range`] over a single index.
  pub fn remove(&mut self, index: usize) {
    self.remove_range(index, index);
  }

  /// Remove the inclusive range of points `[start, end]`.
  ///
  /// Port of `Remove( int, int )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1077`. A range that
  /// runs backwards or reaches past the last point is a silent no
  /// operation, as it is in KiCad (`:1093`).
  ///
  /// KiCad forces the chain open for the duration (`:1083`) and restores
  /// the flag at the end (`:1163`), which runs
  /// `mergeFirstLastPointIfNeeded` twice. Note 01 section 6.8 item 5 calls
  /// that a trick, because in the arc case it can add a trailing point and
  /// then remove it again. What is observable is the restore: removing
  /// from a closed chain until its last point coincides with its first
  /// drops that last point. This reproduces the observable result, and
  /// keeps the open-restore-close shape so that the arc branches have
  /// somewhere to go.
  pub fn remove_range(&mut self, start: usize, end: usize) {
    let closed_state = self.closed;

    self.set_closed(false);

    let point_count = self.points.len();

    if start >= point_count || end >= point_count || start > end {
      self.set_closed(closed_state);
      return;
    }

    self.points.drain(start..=end);
    self.set_closed(closed_state);
  }

  /// Replace the inclusive range of points `[start, end]` with one point.
  ///
  /// Port of `Replace( int, int, const VECTOR2I& )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:999`, which is a
  /// [`LineChain::remove_range`] followed by an [`LineChain::insert`] at
  /// `start`. Both of those keep KiCad's semantics, so replacing the last
  /// point of a chain goes through the append path and is subject to
  /// duplicate suppression, and a range that `remove_range` refuses still
  /// gets the insert.
  ///
  /// # Panics
  ///
  /// When the insert that follows the removal would be out of range, which
  /// needs `start` to be past the end of the shortened chain.
  pub fn replace(&mut self, start: usize, end: usize, point: Vec2) {
    self.remove_range(start, end);
    self.insert(start, point);
  }

  /// Replace the inclusive range of points `[start, end]` with the points
  /// of another chain.
  ///
  /// Port of `Replace( int, int, const SHAPE_LINE_CHAIN& )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1007`. The incoming
  /// chain is trimmed against the boundary points first: a first point
  /// equal to `self.point(start)` is dropped and `start` moves on
  /// (`:1030`), a last point equal to `self.point(end)` is dropped and
  /// `end` moves back, but only when `end` is not already zero
  /// (`:1043`). Either trim can empty the incoming chain, in which case
  /// the call degrades to a plain removal. Unlike
  /// [`LineChain::remove_range`] this does not merge a trailing point
  /// against the first afterwards, which is KiCad's behaviour and not an
  /// oversight in the port.
  ///
  /// The other chain's closed flag and width are ignored; only its points
  /// are taken, so a closed replacement loses its closing segment.
  ///
  /// # Panics
  ///
  /// When `start` is greater than `end`, or `end` is not less than
  /// [`LineChain::point_count`]. KiCad states both as `wxASSERT`
  /// (`:1017`), which is compiled out of a release build; the optimizer
  /// call site at `pcbnew/router/pns_optimizer.cpp:1404` swaps its own
  /// indices rather than relying on that.
  pub fn replace_with_chain(
    &mut self,
    start: usize,
    end: usize,
    other: &LineChain,
  ) {
    assert!(start <= end, "replace start {start} is after end {end}");
    assert!(
      end < self.points.len(),
      "replace end {end} is out of range for a chain of {} points",
      self.points.len()
    );

    let mut start = start;
    let mut end = end;
    let mut new_points = other.points.clone();

    if new_points.is_empty() {
      self.remove_range(start, end);
      return;
    }

    if new_points[0] == self.points[start] {
      start += 1;
      new_points.remove(0);

      if new_points.is_empty() {
        self.remove_range(start, end);
        return;
      }
    }

    if new_points[new_points.len() - 1] == self.points[end] && end > 0 {
      end -= 1;
      new_points.pop();
    }

    self.remove_range(start, end);

    if new_points.is_empty() {
      return;
    }

    self.points.splice(start..start, new_points);
  }

  /// The inclusive range of points `[start, end]` as a new chain.
  ///
  /// Port of `Slice( int, int )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1418`. Two properties
  /// of KiCad's decide how the router's twenty call sites are written and
  /// are kept: the result is always **open**, whatever this chain is, and the
  /// range **cannot cross the seam** of a closed chain, because `end` must
  /// not precede `start`. Slicing across the seam is two slices and a
  /// concatenation, which is what `pcbnew/router/pns_line.cpp` does. The
  /// result also starts at width zero, because KiCad builds it out of a
  /// default constructed chain and never copies the width across.
  ///
  /// KiCad answers every failure with an empty chain, which a caller
  /// cannot tell from an empty result; this returns [`SliceError`].
  /// Negative KiCad indices go through [`LineChain::normalize_index`].
  ///
  /// # Errors
  ///
  /// [`SliceError::IndexOutOfRange`] when either endpoint is not a point
  /// of the chain, [`SliceError::EndBeforeStart`] when the range runs
  /// backwards.
  pub fn slice(
    &self,
    start: usize,
    end: usize,
  ) -> Result<LineChain, SliceError> {
    let point_count = self.points.len();

    if start >= point_count {
      return Err(SliceError::IndexOutOfRange {
        index: start,
        point_count,
      });
    }

    if end >= point_count {
      return Err(SliceError::IndexOutOfRange {
        index: end,
        point_count,
      });
    }

    if end < start {
      return Err(SliceError::EndBeforeStart { start, end });
    }

    Ok(Self {
      points: self.points[start..=end].to_vec(),
      closed: false,
      width: 0,
    })
  }

  /// Insert a vertex at the point of the chain closest to `point`.
  ///
  /// Port of `Split( const VECTOR2I&, bool aExact = false )` with the
  /// default argument,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1181`. The point has
  /// to be within [`LineChain::SPLIT_HIT_THRESHOLD`] of a segment,
  /// strictly, so 0 or 1 nm away; otherwise the answer is `None` and the
  /// chain is untouched. A point that already is a vertex is not
  /// duplicated and its index comes back unchanged. Returns the index of
  /// the vertex that now carries the point.
  ///
  /// KiCad's `aExact` flag short circuits on an exact vertex match. It is
  /// dropped because the router never passes it; the only two argument
  /// `Split` the router calls is the unrelated three way split at
  /// `shape_line_chain.cpp:2877`, which belongs to part 2.
  ///
  /// Two details of KiCad's search are reproduced rather than tidied,
  /// because the walkaround and the optimizer feed it near vertex points.
  /// The candidate segment is only replaced by a nearer one when its index
  /// is below the index of an exactly matching vertex (`:1204`), and a
  /// segment whose own endpoint is the query point is skipped (`:1197`),
  /// which is what stops the split from producing a slightly concave
  /// corner.
  pub fn split(&mut self, point: Vec2) -> Option<usize> {
    let found_index = self.find(point, 0);
    let mut candidate: Option<usize> = None;
    let mut min_dist = Self::SPLIT_HIT_THRESHOLD;

    for index in 0..self.segment_count() {
      let seg = self.segment(index);
      let distance = seg.distance_to_point(point);

      if distance < min_dist && seg.a != point && seg.b != point {
        min_dist = distance;

        match found_index {
          None => candidate = Some(index),
          Some(found) => {
            if index < found {
              candidate = Some(index);
            }
          }
        }
      }
    }

    let index = candidate.or(found_index)?;

    if self.points[index] == point {
      return Some(index);
    }

    let new_index = index + 1;

    self.insert(new_index, point);
    Some(new_index)
  }

  /// Reverse the point order in place.
  ///
  /// KiCad has no in place reverse; `Reverse`
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:910`) copies. The
  /// closed flag and the width survive, as they do in the copy.
  pub fn reverse(&mut self) {
    self.points.reverse();
  }

  /// A copy with the point order reversed.
  ///
  /// Port of `Reverse`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:910`, which copies the
  /// whole chain and reverses the point vector, preserving `m_closed`.
  /// Note that reversing a closed chain moves the seam: the closing
  /// segment of the copy joins the original first point back to the
  /// original last one.
  pub fn reversed(&self) -> LineChain {
    let mut copy = self.clone();

    copy.reverse();
    copy
  }

  /// Translate every point.
  ///
  /// Port of `Move`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:776`.
  pub fn move_by(&mut self, delta: Vec2) {
    for point in &mut self.points {
      *point += delta;
    }
  }

  /// Move one point to a new position.
  ///
  /// Deviation from `SetPoint`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1362`, which wraps a
  /// negative or over range index one step the way `CPoint` does. This
  /// panics instead, for the reason given on [`LineChain::point`]. Once
  /// arcs exist this also has to destroy any arc touching the point, which
  /// KiCad does at `:1371`; there is no operation that moves an arc
  /// endpoint.
  ///
  /// # Panics
  ///
  /// When `index` is not less than [`LineChain::point_count`].
  pub fn set_point(&mut self, index: usize, point: Vec2) {
    assert!(
      index < self.points.len(),
      "point index {index} is out of range for a chain of {} points",
      self.points.len()
    );

    self.points[index] = point;
  }

  /// Reflect every point in an axis.
  ///
  /// Port of `Mirror( const SEG& )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:989`. The router calls
  /// this once, from the meander generator
  /// (`pcbnew/router/pns_meander.cpp:685`). KiCad's other overload, which
  /// mirrors about a horizontal or vertical line through a reference
  /// point (`:974`), has no caller in the router and is not ported.
  pub fn mirror(&mut self, axis: &Seg) {
    for point in &mut self.points {
      *point = axis.reflect_point(*point);
    }
  }

  // ---------------------------------------------------------------
  // Queries
  // ---------------------------------------------------------------

  /// The bounding box of the points, grown by the clearance and the width.
  ///
  /// Port of `BBox`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:457`. Note the
  /// inflation: KiCad grows by `aClearance + m_width`, not by
  /// `aClearance + m_width / 2` as a half width would suggest, and the
  /// growth is skipped entirely when both are zero. Both are reproduced,
  /// because the index queries the router builds from these boxes have to
  /// stay conservative in the same way (note 01 section 6.8 item 6).
  ///
  /// Returns `None` for an empty chain, where KiCad returns its
  /// uninitialised `BOX2I`.
  pub fn bbox(&self, clearance: i32) -> Option<Box2> {
    let (first, rest) = self.points.split_first()?;
    let mut box2 = Box2::from_vec2(*first);

    for point in rest {
      box2 = box2.merge_point(Vec2L::from(*point));
    }

    if clearance != 0 || self.width != 0 {
      box2 = box2.inflate_by(i64::from(clearance) + i64::from(self.width));
    }

    Some(box2)
  }

  /// The total length of the chain, closing segment included.
  ///
  /// Port of `Length`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:956`, a sum of
  /// `SEG::Length` over every segment. Each term is rounded to the nearest
  /// nanometre before it is added, so the sum is not the length of the
  /// exact polyline; that is what the router compares walkaround
  /// candidates by, so it is reproduced.
  pub fn length(&self) -> i64 {
    (0..self.segment_count())
      .map(|index| i64::from(self.segment(index).length()))
      .sum()
  }

  /// The distance from the start of the chain to a point on it.
  ///
  /// Port of `PathLength`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1952`. `index_hint`
  /// names the segment the point is expected to lie on; a hint equal to
  /// [`LineChain::segment_count`] means the last segment, which
  /// compensates for the index aliasing in KiCad's intersection records
  /// (note 01 section 6.8 item 14). A hint past that never matches and the
  /// answer is `None`.
  ///
  /// `None` for the hint reproduces KiCad's `aIndex = -1` default, where
  /// the running match flag starts true and the routine therefore returns
  /// on the very first segment. That silently means "the straight line
  /// distance from point 0", which note 01 section 6.8 item 14 flags as a
  /// trap; the router always passes a hint
  /// (`pcbnew/router/pns_node.cpp:400` and `:419`).
  ///
  /// KiCad accumulates in an `int`. This accumulates in an `i64` so a long
  /// path cannot overflow, and returns `None` where KiCad returns `-1`.
  pub fn path_length(
    &self,
    point: Vec2,
    index_hint: Option<usize>,
  ) -> Option<i64> {
    let segment_count = self.segment_count();
    let mut sum: i64 = 0;

    for index in 0..segment_count {
      let seg = self.segment(index);

      let index_match = match index_hint {
        None => true,
        Some(hint) if hint == segment_count => index == segment_count - 1,
        Some(hint) => index == hint,
      };

      if index_match {
        return Some(sum + i64::from((point - seg.a).euclidean_norm()));
      }

      sum += i64::from(seg.length());
    }

    None
  }

  /// The index of the first vertex within a threshold of a point.
  ///
  /// Port of `Find`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1237`. A threshold of
  /// zero asks for an exact match; anything else compares the Euclidean
  /// norm of the difference with `<=`. The router uses both forms, exact
  /// at most call sites and a threshold of 1 at
  /// `pcbnew/router/pns_line.cpp:369`.
  pub fn find(&self, point: Vec2, threshold: i32) -> Option<usize> {
    self.points.iter().position(|candidate| {
      if threshold == 0 {
        *candidate == point
      } else {
        (*candidate - point).euclidean_norm() <= threshold
      }
    })
  }

  /// Drop vertices that lie on the straight line between their
  /// neighbours.
  ///
  /// Port of `Simplify( int aTolerance = 0 )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2782`, a greedy walk
  /// that keeps pushing the reachable endpoint further out while every
  /// intermediate vertex stays within the tolerance of the chord. Pass a
  /// tolerance of zero for KiCad's default.
  ///
  /// Three details decide the outcome. The reachability test is a distance
  /// to the **segment**, not to the infinite line, so a vertex beyond the
  /// chord's end never qualifies. A tolerance of zero is not exact
  /// colinearity in the algebraic sense: the test bottoms out in
  /// `squared distance < (tolerance + 1)^2`, so a vertex whose squared
  /// distance to the chord rounds down to zero is dropped. And the walk
  /// wraps modulo the point count, so a closed chain can simplify across
  /// its seam and lose its first vertex, while an open one always keeps
  /// its first and last.
  ///
  /// This is where [`LineChain::simplify2`] differs, and the difference is
  /// deliberate; see that method.
  pub fn simplify(&mut self, tolerance: i32) {
    let point_count = self.points.len();

    if point_count < 3 {
      return;
    }

    let mut new_points: Vec<Vec2> = Vec::with_capacity(point_count);
    let mut start_index = 0usize;

    while start_index < point_count {
      new_points.push(self.points[start_index]);

      // An open chain must keep its last two points, so there is nothing
      // left to reach past (`shape_line_chain.cpp:2799`).
      if !self.closed && start_index == point_count - 2 {
        break;
      }

      let mut end_index = (start_index + 2) % point_count;
      let mut can_simplify = true;

      while can_simplify
        && end_index != start_index
        && (end_index > start_index || self.closed)
      {
        let mut test_index = (start_index + 1) % point_count;

        while test_index != end_index {
          if !test_segment_hit(
            self.points[test_index],
            self.points[start_index],
            self.points[end_index],
            tolerance,
          ) {
            can_simplify = false;
            break;
          }

          test_index = (test_index + 1) % point_count;
        }

        if can_simplify {
          end_index = (end_index + 1) % point_count;
        }
      }

      if end_index == (start_index + 2) % point_count {
        start_index += 1;
      } else {
        let new_start_index = (end_index + point_count - 1) % point_count;

        // The walk came all the way round a closed chain.
        if new_start_index <= start_index {
          break;
        }

        start_index = new_start_index;
      }
    }

    // A single point is not a line (`shape_line_chain.cpp:2856`).
    if new_points.len() == 1 {
      new_points.push(self.points[point_count - 1]);
    }

    if !self.closed
      && self.points[point_count - 1] != new_points[new_points.len() - 1]
    {
      new_points.push(self.points[point_count - 1]);
    }

    self.points = new_points;
  }

  /// The legacy simplifier the optimizer runs on.
  ///
  /// Port of `Simplify2( bool aRemoveColinear = true )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2906`. Two stages:
  /// adjacent duplicate vertices go first, then, when `remove_colinear` is
  /// set, runs of vertices whose distance to the chord is at most
  /// [`LineChain::SIMPLIFY2_TOLERANCE`] or whose segments pass
  /// [`Seg::collinear`].
  ///
  /// It is not a worse [`LineChain::simplify`] and it is not redundant.
  /// The measurement is a distance to the **infinite line**, not to the
  /// segment, the tolerance is a hard coded nanometre rather than a
  /// parameter, and it ignores the closed flag entirely, so the closing
  /// segment is never simplified. The header comment at `slc.h:360` asks
  /// for it to stay until the rounding errors that motivated it are
  /// understood, and note 01 section 6.5 records that the optimizer's
  /// convergence depends on that nanometre of slack. Both simplifiers are
  /// therefore ported.
  ///
  /// A chain of three points is a special case in KiCad: only an exact
  /// duplicate of the first point is removed and nothing is checked for
  /// colinearity. The router calls the no colinear form once, from
  /// `pcbnew/router/pns_line.cpp:660`.
  pub fn simplify2(&mut self, remove_colinear: bool) {
    if self.points.len() < 3 {
      return;
    }

    if self.points.len() == 3 {
      if self.points[0] == self.points[1] {
        self.remove(1);
      }

      return;
    }

    // Stage 1, `shape_line_chain.cpp:2928`: collapse runs of equal points.
    let mut unique: Vec<Vec2> = Vec::with_capacity(self.points.len());
    let mut index = 0usize;

    while index < self.points.len() {
      let mut next = index + 1;

      while next < self.points.len() && self.points[index] == self.points[next]
      {
        next += 1;
      }

      unique.push(self.points[index]);
      index = next;
    }

    // Stage 2, `shape_line_chain.cpp:2963`: collapse colinear runs.
    let unique_count = unique.len();
    let limit = unique_count.saturating_sub(2);

    self.points.clear();

    let mut index = 0usize;

    while index < limit {
      let first = unique[index];
      let mut reach = index;

      if remove_colinear {
        while reach < limit
          && (Seg::new(first, unique[reach + 2])
            .line_distance(unique[reach + 1])
            <= Self::SIMPLIFY2_TOLERANCE
            || Seg::new(first, unique[reach + 2])
              .collinear(&Seg::new(first, unique[reach + 1])))
        {
          reach += 1;
        }
      }

      self.points.push(first);

      if reach > index {
        index = reach;
      }

      if reach == limit {
        self.points.push(unique[unique_count - 1]);
        return;
      }

      index += 1;
    }

    if unique_count > 1 {
      self.points.push(unique[unique_count - 2]);
    }

    self.points.push(unique[unique_count - 1]);
  }

  /// Whether two chains describe the same simplified geometry.
  ///
  /// Port of `CompareGeometry( const SHAPE_LINE_CHAIN&, bool
  /// aCyclicalCompare = false, int aEpsilon = 0 )` with both defaults,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2535`. Both chains are
  /// copied, run through [`LineChain::simplify`] with a tolerance of zero
  /// and compared point by point, so a colinear vertex on one side does
  /// not make the chains differ. The closed flag and the width take no
  /// part, but they do reach `simplify`, so an open and a closed chain
  /// over the same points can still compare unequal.
  ///
  /// KiCad's two optional parameters are not ported. `aCyclicalCompare`
  /// sorts both vertex sets by their angle around the centroid, in `f64`,
  /// to make the comparison independent of where the chain starts;
  /// `aEpsilon` loosens the per coordinate comparison. The router calls
  /// only the plain form, from `pcbnew/router/pns_line.cpp:1402` and
  /// `pcbnew/router/pns_shove.cpp:2317`.
  pub fn compare_geometry(&self, other: &LineChain) -> bool {
    let mut simplified_self = self.clone();
    let mut simplified_other = other.clone();

    simplified_self.simplify(0);
    simplified_other.simplify(0);

    simplified_self.points == simplified_other.points
  }

  // ---------------------------------------------------------------
  // Internals
  // ---------------------------------------------------------------

  /// Fold a trailing point that coincides with the first one.
  ///
  /// Port of `mergeFirstLastPointIfNeeded`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:214`. Closing a chain
  /// of more than one point whose last point equals its first drops that
  /// last point. KiCad's other branch, which duplicates point 0 at the end
  /// when an open chain starts on a vertex shared between two arcs
  /// (`:233`), cannot fire without arcs and is where the arc case goes.
  fn merge_first_last_point_if_needed(&mut self) {
    if !self.closed {
      return;
    }

    if self.points.len() > 1
      && self.points[0] == self.points[self.points.len() - 1]
    {
      self.points.pop();
    }
  }
}

/// Whether a point lies within a distance of a segment.
///
/// Port of `TestSegmentHit`, `libs/kimath/src/trigo.cpp:171`, which is the
/// tolerance test inside `SHAPE_LINE_CHAIN::Simplify`
/// (`libs/kimath/src/geometry/shape_line_chain.cpp:2822`). It is not a
/// plain distance comparison: the bounding box rejection comes first, the
/// axis aligned cases short circuit to a single coordinate difference, and
/// the general case compares the **squared** distance with
/// `(distance + 1)^2`, strictly. A distance of zero therefore accepts
/// every point whose squared distance to the segment rounds down to zero,
/// not only the algebraically colinear ones.
///
/// KiCad computes the coordinate differences in a wrapping 32 bit
/// `VECTOR2I`; these are computed in `i64` so a chain that spans the whole
/// coordinate range does not fold a rejection into an acceptance.
fn test_segment_hit(
  reference: Vec2,
  start: Vec2,
  end: Vec2,
  distance: i32,
) -> bool {
  let mut x_min = i64::from(start.x);
  let mut x_max = i64::from(end.x);
  let mut y_min = i64::from(start.y);
  let mut y_max = i64::from(end.y);

  let delta_x = i64::from(start.x) - i64::from(reference.x);
  let delta_y = i64::from(start.y) - i64::from(reference.y);

  if x_max < x_min {
    std::mem::swap(&mut x_max, &mut x_min);
  }

  if y_max < y_min {
    std::mem::swap(&mut y_max, &mut y_min);
  }

  let reference_x = i64::from(reference.x);
  let reference_y = i64::from(reference.y);
  let distance = i64::from(distance);

  if y_min - reference_y > distance || reference_y - y_max > distance {
    return false;
  }

  if x_min - reference_x > distance || reference_x - x_max > distance {
    return false;
  }

  if start.x == end.x && reference_y > y_min && reference_y < y_max {
    return delta_x.abs() <= distance;
  }

  if start.y == end.y && reference_x > x_min && reference_x < x_max {
    return delta_y.abs() <= distance;
  }

  let squared_threshold = (distance + 1) * (distance + 1);

  Seg::new(start, end).squared_distance_to_point(reference) < squared_threshold
}

#[cfg(test)]
mod tests {
  use super::*;

  /// A shorthand for the KiCad test cases, which spell out coordinates.
  fn point(x: i32, y: i32) -> Vec2 {
    Vec2::new(x, y)
  }

  /// The points of a chain, for comparing against a KiCad expectation.
  fn points_of(chain: &LineChain) -> Vec<Vec2> {
    chain.points().to_vec()
  }

  // ---------------------------------------------------------------
  // Construction, equality and the trivial accessors
  // ---------------------------------------------------------------

  #[test]
  fn an_empty_chain_is_open_and_has_nothing_in_it() {
    let chain = LineChain::new();

    assert!(chain.is_empty());
    assert_eq!(chain.point_count(), 0);
    assert_eq!(chain.segment_count(), 0);
    assert!(!chain.is_closed());
    assert_eq!(chain.width(), 0);
    assert_eq!(chain.last_point(), None);
    assert_eq!(chain.bbox(0), None);
    assert_eq!(chain, LineChain::default());
  }

  #[test]
  fn a_chain_from_a_segment_holds_its_two_endpoints() {
    let seg = Seg::with_index(point(0, 0), point(10, 20), 7);
    let chain = LineChain::from_seg(&seg);

    assert_eq!(points_of(&chain), vec![point(0, 0), point(10, 20)]);
    assert!(!chain.is_closed());
    assert_eq!(chain.segment(0).index, 0);
  }

  #[test]
  fn equality_covers_the_points_the_closed_flag_and_the_width() {
    // Deviation from KiCad, which has no `operator==` at all and whose
    // `operator!=` (`shape_line_chain.h:749`) compares only the point
    // count and the points, ignoring both the closed flag and the width.
    // The router never uses it. `compare_geometry` is the KiCad faithful
    // "same shape" test.
    let base = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);

    assert_eq!(base, base.clone());

    let closed = LineChain::from_slice(&[point(0, 0), point(10, 0)], true);

    assert_ne!(base, closed);

    let mut wide = base.clone();

    wide.set_width(250_000);
    assert_ne!(base, wide);
  }

  #[test]
  fn point_and_points_read_the_storage() {
    let chain =
      LineChain::from_slice(&[point(0, 0), point(1, 1), point(2, 2)], false);

    assert_eq!(chain.point(0), point(0, 0));
    assert_eq!(chain.point(2), point(2, 2));
    assert_eq!(chain.last_point(), Some(point(2, 2)));
    assert_eq!(chain.points().len(), 3);
  }

  #[test]
  #[should_panic(expected = "point index 3 is out of range")]
  fn point_panics_instead_of_wrapping_like_c_point() {
    // KiCad's `CPoint` (`shape_line_chain.h:420`) would answer `CPoint(0)`
    // here, and `CPoint(-1)` would answer the last point.
    let chain =
      LineChain::from_slice(&[point(0, 0), point(1, 1), point(2, 2)], false);

    let _ = chain.point(3);
  }

  #[test]
  fn normalize_index_reproduces_kicads_single_step_wrap() {
    let chain =
      LineChain::from_slice(&[point(0, 0), point(1, 1), point(2, 2)], false);

    assert_eq!(chain.normalize_index(0), Some(0));
    assert_eq!(chain.normalize_index(2), Some(2));
    assert_eq!(chain.normalize_index(-1), Some(2));
    assert_eq!(chain.normalize_index(-3), Some(0));
    assert_eq!(chain.normalize_index(-4), None);
    assert_eq!(chain.normalize_index(3), None);
  }

  // ---------------------------------------------------------------
  // Counts
  // ---------------------------------------------------------------

  #[test]
  fn segment_count_for_zero_one_and_two_points_open_and_closed() {
    // KiCad's `SegmentCount` is `max( 0, PointCount() - 1 + closed )`
    // (`shape_line_chain.h:327`), computed here in signed arithmetic so a
    // chain of no points cannot underflow into a huge count.
    let mut chain = LineChain::new();

    assert_eq!(chain.segment_count(), 0);
    chain.set_closed(true);
    assert_eq!(chain.segment_count(), 0);

    let mut one = LineChain::from_slice(&[point(5, 5)], false);

    assert_eq!(one.segment_count(), 0);
    one.set_closed(true);
    assert_eq!(one.segment_count(), 1);
    // The degenerate closing segment of a single point chain,
    // `shape_line_chain.cpp:1295`.
    assert_eq!(one.segment(0), Seg::with_index(point(5, 5), point(5, 5), 0));

    let mut two = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);

    assert_eq!(two.segment_count(), 1);
    two.set_closed(true);
    assert_eq!(two.segment_count(), 2);
  }

  #[test]
  fn the_last_segment_of_a_closed_chain_returns_to_the_first_point() {
    let chain =
      LineChain::from_slice(&[point(0, 0), point(10, 0), point(10, 10)], true);

    assert_eq!(chain.segment_count(), 3);
    assert_eq!(
      chain.segment(2),
      Seg::with_index(point(10, 10), point(0, 0), 2)
    );
  }

  #[test]
  #[should_panic(expected = "segment index 2 is out of range")]
  fn segment_panics_past_the_last_segment() {
    let chain =
      LineChain::from_slice(&[point(0, 0), point(10, 0), point(10, 10)], false);

    let _ = chain.segment(2);
  }

  // ---------------------------------------------------------------
  // Appending
  // ---------------------------------------------------------------

  #[test]
  fn append_drops_a_duplicate_of_the_last_point() {
    // `shape_line_chain.h:539`. The placer depends on this, note 01
    // section 14.3.
    let mut chain = LineChain::new();

    chain.append(point(100, 100));
    chain.append(point(100, 100));
    assert_eq!(chain.point_count(), 1);

    chain.append(point(200, 100));
    chain.append(point(100, 100));
    assert_eq!(chain.point_count(), 3);

    // Only the *last* point is compared, so a point that repeats one
    // further back is kept.
    assert_eq!(
      points_of(&chain),
      vec![point(100, 100), point(200, 100), point(100, 100)]
    );
  }

  #[test]
  fn append_allow_duplicate_keeps_it() {
    // `pcbnew/router/pns_router.cpp:313` builds a degenerate two point
    // chain this way.
    let mut chain = LineChain::new();

    chain.append(point(100, 100));
    chain.append_allow_duplicate(point(100, 100));

    assert_eq!(chain.point_count(), 2);
    assert_eq!(chain.segment_count(), 1);
  }

  #[test]
  fn append_chain_suppresses_only_the_joining_duplicate() {
    // `shape_line_chain.cpp:1571`.
    let mut chain = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);
    let other = LineChain::from_slice(
      &[point(10, 0), point(10, 10), point(10, 10)],
      false,
    );

    chain.append_chain(&other);

    assert_eq!(
      points_of(&chain),
      vec![point(0, 0), point(10, 0), point(10, 10), point(10, 10)]
    );

    // A chain with no points changes nothing.
    let before = chain.clone();

    chain.append_chain(&LineChain::new());
    assert_eq!(chain, before);
  }

  #[test]
  fn append_chain_onto_a_closed_chain_folds_the_returning_point() {
    // The `mergeFirstLastPointIfNeeded` at `shape_line_chain.cpp:1606`.
    let mut chain =
      LineChain::from_slice(&[point(0, 0), point(10, 0), point(10, 10)], true);
    let other = LineChain::from_slice(&[point(0, 10), point(0, 0)], false);

    chain.append_chain(&other);

    assert_eq!(
      points_of(&chain),
      vec![point(0, 0), point(10, 0), point(10, 10), point(0, 10)]
    );
    assert!(chain.is_closed());
  }

  // ---------------------------------------------------------------
  // The closed flag
  // ---------------------------------------------------------------

  #[test]
  fn closing_a_chain_drops_a_last_point_equal_to_the_first() {
    // The point case of `mergeFirstLastPointIfNeeded`,
    // `shape_line_chain.cpp:218`, which KiCad's `SetClosedDuplicatePoint`
    // exercises through arcs.
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(10, 0), point(10, 10), point(0, 0)],
      false,
    );

    assert_eq!(chain.point_count(), 4);

    chain.set_closed(true);

    assert_eq!(chain.point_count(), 3);
    assert_eq!(chain.segment_count(), 3);

    // Reopening does not put it back: only the arc branch adds a point.
    chain.set_closed(false);
    assert_eq!(chain.point_count(), 3);
  }

  #[test]
  fn toggling_closed_keeps_the_point_count_of_the_plain_kicad_cases() {
    // The `OnePoint`, `TwoPoints` and `ThreePoints` rows of KiCad's
    // `ToggleClosed` case table, `test_shape_line_chain.cpp:318` to
    // `:320`, which are the three arc free entries.
    let cases: [Vec<Vec2>; 3] = [
      vec![point(233450000, 228360000)],
      vec![point(0, 0), point(10, 0)],
      vec![point(0, 0), point(10, 0), point(10, 10)],
    ];

    for case in cases {
      let expected = case.len();
      let mut chain = LineChain::from_points(case, false);

      assert!(!chain.is_closed());
      assert_eq!(chain.point_count(), expected);

      chain.set_closed(true);
      assert!(chain.is_closed());
      assert_eq!(chain.point_count(), expected);

      chain.set_closed(false);
      assert!(!chain.is_closed());
      assert_eq!(chain.point_count(), expected);
    }
  }

  #[test]
  fn clear_opens_the_chain_but_keeps_the_width() {
    // `shape_line_chain.h:274`, note 01 section 6.8 item 7.
    let mut chain = LineChain::from_slice(&[point(0, 0), point(1, 1)], true);

    chain.set_width(250_000);
    chain.clear();

    assert!(chain.is_empty());
    assert!(!chain.is_closed());
    assert_eq!(chain.width(), 250_000);
  }

  // ---------------------------------------------------------------
  // Insert, remove, replace
  // ---------------------------------------------------------------

  #[test]
  fn insert_puts_a_point_before_the_index() {
    let mut chain = LineChain::from_slice(&[point(0, 0), point(20, 0)], false);

    chain.insert(1, point(10, 0));
    assert_eq!(
      points_of(&chain),
      vec![point(0, 0), point(10, 0), point(20, 0)]
    );

    // KiCad allows a duplicate here, `shape_line_chain.cpp:1650`.
    chain.insert(1, point(0, 0));
    assert_eq!(
      points_of(&chain),
      vec![point(0, 0), point(0, 0), point(10, 0), point(20, 0)]
    );
  }

  #[test]
  fn insert_at_the_end_goes_through_append_and_dedupes() {
    // `shape_line_chain.cpp:1639`.
    let mut chain = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);

    chain.insert(2, point(10, 0));
    assert_eq!(chain.point_count(), 2);

    chain.insert(2, point(20, 0));
    assert_eq!(chain.point_count(), 3);
  }

  #[test]
  fn remove_range_at_the_ends_and_out_of_range() {
    let base = LineChain::from_slice(
      &[point(0, 0), point(10, 0), point(20, 0), point(30, 0)],
      false,
    );

    let mut front = base.clone();

    front.remove_range(0, 1);
    assert_eq!(points_of(&front), vec![point(20, 0), point(30, 0)]);

    let mut back = base.clone();

    back.remove_range(2, 3);
    assert_eq!(points_of(&back), vec![point(0, 0), point(10, 0)]);

    let mut single = base.clone();

    single.remove(3);
    assert_eq!(single.point_count(), 3);

    // A backwards or over range request is a silent no operation,
    // `shape_line_chain.cpp:1093`.
    let mut untouched = base.clone();

    untouched.remove_range(2, 1);
    assert_eq!(untouched, base);
    untouched.remove_range(1, 4);
    assert_eq!(untouched, base);
    untouched.remove_range(4, 4);
    assert_eq!(untouched, base);
  }

  #[test]
  fn remove_range_on_a_closed_chain_folds_the_new_seam() {
    // The observable half of note 01 section 6.8 item 5: the chain is
    // opened for the removal (`shape_line_chain.cpp:1083`) and closed
    // again afterwards (`:1163`), and the closing runs the first against
    // last merge. Removing the leading point here leaves a last point
    // equal to the new first one, so one removal costs two points.
    let source = vec![
      point(5, 5),
      point(0, 0),
      point(10, 0),
      point(10, 10),
      point(0, 0),
    ];

    let mut closed = LineChain::from_points(source.clone(), true);

    // Nothing folded on construction: the first and last points differ.
    assert_eq!(closed.point_count(), 5);

    closed.remove(0);
    assert_eq!(
      points_of(&closed),
      vec![point(0, 0), point(10, 0), point(10, 10)]
    );
    assert!(closed.is_closed());

    // The same removal on an open chain keeps the trailing duplicate.
    let mut open = LineChain::from_points(source, false);

    open.remove(0);
    assert_eq!(
      points_of(&open),
      vec![point(0, 0), point(10, 0), point(10, 10), point(0, 0)]
    );
  }

  #[test]
  fn replace_a_range_with_one_point() {
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(10, 0), point(20, 0), point(30, 0)],
      false,
    );

    chain.replace(1, 2, point(15, 5));
    assert_eq!(
      points_of(&chain),
      vec![point(0, 0), point(15, 5), point(30, 0)]
    );
  }

  #[test]
  fn replace_with_a_chain_shorter_and_longer_than_the_range() {
    let base = LineChain::from_slice(
      &[point(0, 0), point(10, 0), point(20, 0), point(30, 0)],
      false,
    );

    // Shorter: two points become one.
    let mut shorter = base.clone();

    shorter.replace_with_chain(
      1,
      2,
      &LineChain::from_slice(&[point(15, 5)], false),
    );
    assert_eq!(
      points_of(&shorter),
      vec![point(0, 0), point(15, 5), point(30, 0)]
    );

    // Longer: two points become three.
    let mut longer = base.clone();

    longer.replace_with_chain(
      1,
      2,
      &LineChain::from_slice(
        &[point(12, 3), point(15, 5), point(18, 3)],
        false,
      ),
    );
    assert_eq!(
      points_of(&longer),
      vec![
        point(0, 0),
        point(12, 3),
        point(15, 5),
        point(18, 3),
        point(30, 0)
      ]
    );

    // An empty replacement degrades to a removal,
    // `shape_line_chain.cpp:1024`.
    let mut emptied = base.clone();

    emptied.replace_with_chain(1, 2, &LineChain::new());
    assert_eq!(points_of(&emptied), vec![point(0, 0), point(30, 0)]);
  }

  #[test]
  fn replace_with_a_chain_trims_coincident_boundary_points() {
    // `shape_line_chain.cpp:1030` and `:1043`. Both ends of the incoming
    // chain match, so it empties and the whole call becomes a no
    // operation.
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(10, 0), point(20, 0), point(30, 0)],
      false,
    );
    let before = chain.clone();

    chain.replace_with_chain(
      1,
      2,
      &LineChain::from_slice(&[point(10, 0), point(20, 0)], false),
    );

    assert_eq!(chain, before);
  }

  #[test]
  fn replace_chain_reproduces_the_kicad_crash_case_8949() {
    // Verbatim from `test_shape_line_chain.cpp:1216`.
    let line_points = vec![
      point(206000000, 140110000),
      point(192325020, 140110000),
      point(192325020, 113348216),
      point(192251784, 113274980),
      point(175548216, 113274980),
      point(175474980, 113348216),
      point(175474980, 136694980),
      point(160774511, 121994511),
      point(160774511, 121693501),
      point(160086499, 121005489),
      point(159785489, 121005489),
      point(159594511, 120814511),
      point(160086499, 120814511),
      point(160774511, 120126499),
      point(160774511, 119153501),
      point(160086499, 118465489),
      point(159113501, 118465489),
      point(158425489, 119153501),
      point(158425489, 119645489),
      point(157325020, 118545020),
      point(157325020, 101925020),
      point(208674980, 101925020),
      point(208674980, 145474980),
      point(192325020, 145474980),
      point(192325020, 140110000),
    ];
    let expected_count = line_points.len();

    let mut base = LineChain::from_points(line_points, false);

    base.set_width(250000);
    assert_eq!(base.point_count(), expected_count);

    let replacement =
      LineChain::from_slice(&[point(192325020, 140110000)], false);

    assert_eq!(replacement.point_count(), 1);

    base.replace_with_chain(1, 23, &replacement);
    assert_eq!(base.point_count(), expected_count - (23 - 1));

    // Replacing the last point in a chain is special cased.
    base.replace(
      base.point_count() - 1,
      base.point_count() - 1,
      point(-1, -1),
    );

    assert_eq!(base.last_point(), Some(point(-1, -1)));
  }

  // ---------------------------------------------------------------
  // Slice
  // ---------------------------------------------------------------

  #[test]
  fn slice_at_the_ends_and_over_an_invalid_range() {
    let chain = LineChain::from_slice(
      &[point(0, 0), point(10, 0), point(20, 0), point(30, 0)],
      true,
    );

    let first = chain.slice(0, 0).expect("a single point slice succeeds");

    assert_eq!(points_of(&first), vec![point(0, 0)]);

    let whole = chain.slice(0, 3).expect("the full range must succeed");

    assert_eq!(whole.point_count(), 4);
    // Never closed, `shape_line_chain.cpp:1418`, and the width is not
    // carried across because KiCad builds the result from a default
    // constructed chain.
    assert!(!whole.is_closed());
    assert_eq!(whole.width(), 0);

    let tail = chain.slice(2, 3).expect("the tail must succeed");

    assert_eq!(points_of(&tail), vec![point(20, 0), point(30, 0)]);

    assert_eq!(
      chain.slice(1, 4),
      Err(SliceError::IndexOutOfRange {
        index: 4,
        point_count: 4
      })
    );
    assert_eq!(
      chain.slice(4, 4),
      Err(SliceError::IndexOutOfRange {
        index: 4,
        point_count: 4
      })
    );
    // A slice cannot cross the seam of a closed chain,
    // `shape_line_chain.cpp:1433`.
    assert_eq!(
      chain.slice(3, 1),
      Err(SliceError::EndBeforeStart { start: 3, end: 1 })
    );
  }

  #[test]
  fn slice_transcribes_the_placers_negative_indices() {
    // `tail.Slice( -threshold, -1 )`,
    // `pcbnew/router/pns_line_placer.cpp:1072`, with a threshold of 3.
    let chain = LineChain::from_slice(
      &[point(0, 0), point(10, 0), point(20, 0), point(30, 0)],
      false,
    );

    let start = chain.normalize_index(-3).expect("in range");
    let end = chain.normalize_index(-1).expect("in range");
    let slice = chain.slice(start, end).expect("in range");

    assert_eq!(
      points_of(&slice),
      vec![point(10, 0), point(20, 0), point(30, 0)]
    );
  }

  // ---------------------------------------------------------------
  // Split
  // ---------------------------------------------------------------

  #[test]
  fn split_on_an_existing_vertex_inserts_nothing() {
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(1000, 0), point(2000, 0)],
      false,
    );

    assert_eq!(chain.split(point(1000, 0)), Some(1));
    assert_eq!(chain.point_count(), 3);
  }

  #[test]
  fn split_inside_a_segment_inserts_a_vertex() {
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(1000, 0), point(2000, 0)],
      false,
    );

    assert_eq!(chain.split(point(1500, 0)), Some(2));
    assert_eq!(
      points_of(&chain),
      vec![point(0, 0), point(1000, 0), point(1500, 0), point(2000, 0)]
    );
  }

  #[test]
  fn split_within_the_tolerance_of_a_vertex_still_inserts() {
    // The threshold is `Distance < 2` (`shape_line_chain.cpp:1184`), and
    // a segment whose own endpoint is the query point is skipped, so the
    // point lands on the segment *before* the near vertex.
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(1000, 0), point(2000, 0)],
      false,
    );

    assert_eq!(chain.split(point(999, 1)), Some(1));
    assert_eq!(
      points_of(&chain),
      vec![point(0, 0), point(999, 1), point(1000, 0), point(2000, 0)]
    );
  }

  #[test]
  fn split_misses_a_point_that_is_too_far_away() {
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(1000, 0), point(2000, 0)],
      false,
    );
    let before = chain.clone();

    assert_eq!(chain.split(point(1000, 2)), None);
    assert_eq!(chain, before);
  }

  #[test]
  fn split_on_the_closing_segment_appends() {
    // The closing segment is the last one of a closed chain, so the new
    // index is `PointCount()` and KiCad's `Insert` takes the append path,
    // `shape_line_chain.cpp:1639`.
    let mut chain = LineChain::from_slice(
      &[
        point(0, 0),
        point(1000, 0),
        point(1000, 1000),
        point(0, 1000),
      ],
      true,
    );

    assert_eq!(chain.segment_count(), 4);
    assert_eq!(chain.split(point(0, 500)), Some(4));
    assert_eq!(chain.point_count(), 5);
    assert_eq!(chain.last_point(), Some(point(0, 500)));
  }

  // ---------------------------------------------------------------
  // Reverse, move, mirror, set_point
  // ---------------------------------------------------------------

  #[test]
  fn reverse_keeps_the_closed_flag_and_the_width() {
    // `shape_line_chain.cpp:910` copies the chain and reverses the point
    // vector, so `m_closed` and `m_width` come along.
    let mut chain =
      LineChain::from_slice(&[point(0, 0), point(10, 0), point(10, 10)], true);

    chain.set_width(250_000);

    let reversed = chain.reversed();

    assert_eq!(
      points_of(&reversed),
      vec![point(10, 10), point(10, 0), point(0, 0)]
    );
    assert!(reversed.is_closed());
    assert_eq!(reversed.width(), 250_000);

    chain.reverse();
    assert_eq!(chain, reversed);
  }

  #[test]
  fn move_by_translates_every_point() {
    let mut chain = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);

    chain.move_by(point(5, -5));
    assert_eq!(points_of(&chain), vec![point(5, -5), point(15, -5)]);
  }

  #[test]
  fn mirror_reflects_every_point_in_the_axis() {
    // `shape_line_chain.cpp:989`.
    let mut chain =
      LineChain::from_slice(&[point(0, 10), point(10, 20)], false);

    chain.mirror(&Seg::new(point(0, 0), point(100, 0)));
    assert_eq!(points_of(&chain), vec![point(0, -10), point(10, -20)]);
  }

  #[test]
  fn set_point_moves_one_vertex() {
    let mut chain = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);

    chain.set_point(1, point(10, 10));
    assert_eq!(points_of(&chain), vec![point(0, 0), point(10, 10)]);
  }

  // ---------------------------------------------------------------
  // bbox, length, path_length, find
  // ---------------------------------------------------------------

  #[test]
  fn bbox_inflates_by_the_clearance_plus_the_whole_width() {
    // `shape_line_chain.h:457`. The inflation is `aClearance + m_width`,
    // not a half width, note 01 section 6.8 item 6.
    let mut chain =
      LineChain::from_slice(&[point(0, 0), point(100, 50)], false);

    let plain = chain.bbox(0).expect("a chain with points has a box");

    assert_eq!(plain.left(), 0);
    assert_eq!(plain.top(), 0);
    assert_eq!(plain.right(), 100);
    assert_eq!(plain.bottom(), 50);

    chain.set_width(10);

    let inflated = chain.bbox(5).expect("a chain with points has a box");

    assert_eq!(inflated.left(), -15);
    assert_eq!(inflated.top(), -15);
    assert_eq!(inflated.right(), 115);
    assert_eq!(inflated.bottom(), 65);
  }

  #[test]
  fn length_of_a_closed_chain_includes_the_closing_segment() {
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(300, 0), point(300, 400)],
      false,
    );

    assert_eq!(chain.length(), 300 + 400);

    chain.set_closed(true);
    // The closing 300, 400 leg is a 500 nm hypotenuse.
    assert_eq!(chain.length(), 300 + 400 + 500);
  }

  #[test]
  fn path_length_walks_to_the_named_segment() {
    // `shape_line_chain.cpp:1952`.
    let chain = LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(100, 100)],
      false,
    );

    assert_eq!(chain.path_length(point(50, 0), Some(0)), Some(50));
    assert_eq!(chain.path_length(point(100, 60), Some(1)), Some(160));
    // A hint equal to the segment count means the last segment,
    // `shape_line_chain.cpp:1963`.
    assert_eq!(chain.path_length(point(100, 60), Some(2)), Some(160));
    // Past that nothing matches.
    assert_eq!(chain.path_length(point(100, 60), Some(3)), None);
    // No hint returns on the very first segment, note 01 section 6.8
    // item 14: the answer is the straight line distance from point 0, not
    // a walk along the chain.
    assert_eq!(chain.path_length(point(30, 40), None), Some(50));
    assert_eq!(LineChain::new().path_length(point(0, 0), None), None);
  }

  #[test]
  fn find_matches_exactly_or_within_a_threshold() {
    // `shape_line_chain.cpp:1237`.
    let chain = LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(200, 0)],
      false,
    );

    assert_eq!(chain.find(point(100, 0), 0), Some(1));
    assert_eq!(chain.find(point(101, 0), 0), None);
    assert_eq!(chain.find(point(101, 0), 1), Some(1));
    assert_eq!(chain.find(point(102, 0), 1), None);
  }

  // ---------------------------------------------------------------
  // Simplify and Simplify2
  // ---------------------------------------------------------------

  #[test]
  fn simplify_removes_a_duplicate_point() {
    // KiCad `SimplifyDuplicatePoint`,
    // `test_shape_line_chain.cpp:380`.
    let mut chain = LineChain::new();

    chain.append(point(100, 100));
    chain.append_allow_duplicate(point(100, 100));
    chain.append(point(200, 100));

    assert_eq!(chain.point_count(), 3);

    chain.simplify(0);

    assert_eq!(chain.point_count(), 2);
  }

  #[test]
  fn simplify_keeps_the_end_point_of_a_closed_chain() {
    // KiCad `SimplifyKeepEndPoint`, `test_shape_line_chain.cpp:400`.
    let mut chain = LineChain::from_slice(
      &[
        point(114772424, 90949410),
        point(114767360, 90947240),
        point(114772429, 90947228),
      ],
      true,
    );

    assert_eq!(chain.point_count(), 3);

    chain.simplify(0);

    assert_eq!(chain.point_count(), 3);
  }

  #[test]
  fn simplify_an_open_pns_chain_with_a_tolerance() {
    // KiCad `SimplifyPNSChain`, `test_shape_line_chain.cpp:420`. The
    // chain is open, so the run cannot wrap and the first and last points
    // survive.
    let mut chain = LineChain::from_slice(
      &[
        point(157527820, 223074385),
        point(186541122, 159990156),
        point(186528624, 159977658),
        point(186528624, 159770550),
        point(186528625, 159366691),
        point(186541122, 159354195),
        point(186541122, 155566877),
        point(187291125, 154816872),
        point(187291125, 147807837),
        point(189301788, 145797175),
        point(194451695, 145797175),
        point(195021410, 146366890),
      ],
      false,
    );

    assert_eq!(chain.point_count(), 12);

    chain.simplify(10);

    assert_eq!(chain.point_count(), 11);
  }

  #[test]
  fn simplify_a_complex_chain_open_then_closed() {
    // KiCad `SimplifyComplexChain`, `test_shape_line_chain.cpp:447`. The
    // closed run wraps across the seam and finds one more colinear
    // vertex than the open one.
    let points = vec![
      point(130000, 147320),
      point(125730, 147320),
      point(125730, 150630),
      point(128800, 153700),
      point(150300, 153700),
      point(151500, 152500),
      point(151500, 148900),
      point(149920, 147320),
      point(140000, 147320),
    ];

    let mut chain = LineChain::from_points(points, false);

    assert_eq!(chain.point_count(), 9);

    chain.simplify(0);
    assert_eq!(chain.point_count(), 9);

    chain.set_closed(true);
    chain.simplify(0);
    assert_eq!(chain.point_count(), 8);
  }

  #[test]
  fn simplify_with_tolerance_collapses_a_rotated_rounded_rectangle() {
    // KiCad `SimplifyWithToleranceIssue22597`,
    // `test_shape_line_chain.cpp:1329`: 164 points approximating a
    // rounded rectangle rotated 45 degrees. A 2 mm tolerance has to bring
    // it down to a handful of corners.
    let points = vec![
      point(135095398, 233618441),
      point(135024554, 233546880),
      point(134887999, 233398857),
      point(134757514, 233245455),
      point(134633313, 233086923),
      point(134515595, 232923519),
      point(134404553, 232755507),
      point(134300366, 232583161),
      point(134203203, 232406759),
      point(134113222, 232226587),
      point(134030567, 232042939),
      point(133955377, 231856111),
      point(133887768, 231666408),
      point(133827854, 231474135),
      point(133775731, 231279607),
      point(133731482, 231083137),
      point(133695180, 230885045),
      point(133666884, 230685652),
      point(133646639, 230485281),
      point(133634480, 230284257),
      point(133630425, 230082907),
      point(133634480, 229881557),
      point(133646639, 229680533),
      point(133666884, 229480162),
      point(133695180, 229280769),
      point(133731482, 229082677),
      point(133775731, 228886207),
      point(133827854, 228691679),
      point(133887768, 228499406),
      point(133955377, 228309703),
      point(134030567, 228122875),
      point(134113222, 227939227),
      point(134203203, 227759055),
      point(134300366, 227582653),
      point(134404553, 227410307),
      point(134515595, 227242295),
      point(134633313, 227078891),
      point(134757514, 226920359),
      point(134887999, 226766957),
      point(135024554, 226618934),
      point(135095398, 226547373),
      point(148530427, 213112344),
      point(148601988, 213041500),
      point(148750011, 212904945),
      point(148903413, 212774460),
      point(149061945, 212650259),
      point(149225349, 212532541),
      point(149393361, 212421499),
      point(149565707, 212317312),
      point(149742109, 212220149),
      point(149922281, 212130168),
      point(150105929, 212047514),
      point(150292757, 211972323),
      point(150482460, 211904715),
      point(150674733, 211844800),
      point(150869261, 211792677),
      point(151065731, 211748428),
      point(151263823, 211712126),
      point(151463216, 211683830),
      point(151710655, 211863478),
      point(151864611, 211651426),
      point(152065961, 211647371),
      point(152267311, 211651426),
      point(152468335, 211663586),
      point(152668706, 211683830),
      point(152868099, 211712126),
      point(153066191, 211748428),
      point(153262661, 211792677),
      point(153457189, 211844800),
      point(153649462, 211904715),
      point(153839165, 211972323),
      point(154025993, 212047514),
      point(154209641, 212130168),
      point(154389813, 212220149),
      point(154566215, 212317312),
      point(154738561, 212421499),
      point(154906573, 212532541),
      point(155069977, 212650259),
      point(155228509, 212774460),
      point(155381911, 212904945),
      point(155529934, 213041500),
      point(155601495, 213112344),
      point(160551242, 218062092),
      point(160622086, 218133653),
      point(160758641, 218281676),
      point(160889126, 218435078),
      point(161013327, 218593610),
      point(161131045, 218757014),
      point(161242087, 218925026),
      point(161346274, 219097372),
      point(161443437, 219273774),
      point(161533418, 219453946),
      point(161616072, 219637594),
      point(161691263, 219824422),
      point(161758871, 220014125),
      point(161818786, 220206398),
      point(161870909, 220400926),
      point(161915158, 220597396),
      point(161951460, 220795488),
      point(161979756, 220994881),
      point(162000000, 221195252),
      point(162012160, 221396276),
      point(162016215, 221597626),
      point(162012160, 221798976),
      point(162000000, 222000000),
      point(161979756, 222200371),
      point(161951460, 222399764),
      point(161915158, 222597856),
      point(161870909, 222794326),
      point(161818786, 222988854),
      point(161758871, 223181127),
      point(161691263, 223370830),
      point(161616072, 223557658),
      point(161533418, 223741306),
      point(161443437, 223921478),
      point(161346274, 224097880),
      point(161242087, 224270226),
      point(161131045, 224438238),
      point(161013327, 224601642),
      point(160889126, 224760174),
      point(160758641, 224913576),
      point(160622086, 225061599),
      point(160551242, 225133160),
      point(147116213, 238568188),
      point(147044657, 238639037),
      point(146896633, 238775592),
      point(146743231, 238906077),
      point(146584699, 239030279),
      point(146421295, 239147996),
      point(146253283, 239259039),
      point(146080936, 239363226),
      point(145904534, 239460389),
      point(145724362, 239550371),
      point(145540714, 239633024),
      point(145353886, 239708216),
      point(145164182, 239775824),
      point(144971909, 239835739),
      point(144777380, 239887863),
      point(144580910, 239932111),
      point(144382818, 239968413),
      point(144183424, 239996709),
      point(143983053, 240016953),
      point(143782029, 240029113),
      point(143580679, 240033169),
      point(143379329, 240029113),
      point(143178305, 240016953),
      point(142977934, 239996709),
      point(142778540, 239968413),
      point(142580448, 239932111),
      point(142383978, 239887863),
      point(142189449, 239835739),
      point(141997176, 239775824),
      point(141807472, 239708216),
      point(141620644, 239633024),
      point(141436996, 239550371),
      point(141256824, 239460389),
      point(141080422, 239363226),
      point(140908075, 239259039),
      point(140740063, 239147996),
      point(140576659, 239030279),
      point(140418127, 238906077),
      point(140264725, 238775592),
      point(140116701, 238639037),
      point(140045145, 238568188),
    ];

    let mut chain = LineChain::from_points(points, true);

    assert_eq!(chain.point_count(), 164);

    chain.simplify(2_000_000);

    assert!(chain.point_count() < 164);
    assert!(chain.point_count() <= 20);
  }

  #[test]
  fn simplify_and_simplify2_differ_on_a_one_nanometre_kink() {
    // The reason both simplifiers are ported (note 01 section 6.5). The
    // middle vertex sits one nanometre off the chord: `simplify` measures
    // against the *segment* and refuses to drop it, `simplify2` measures
    // against the infinite line with its hard coded one nanometre of
    // slack and drops it, taking the following colinear vertex with it.
    let kinked = LineChain::from_slice(
      &[point(0, 0), point(1000, 1), point(2000, 0), point(3000, 0)],
      false,
    );

    let mut exact = kinked.clone();

    exact.simplify(0);
    assert_eq!(
      points_of(&exact),
      vec![point(0, 0), point(1000, 1), point(3000, 0)]
    );

    let mut legacy = kinked.clone();

    legacy.simplify2(true);
    assert_eq!(points_of(&legacy), vec![point(0, 0), point(3000, 0)]);

    // Without colinear removal the second stage does nothing.
    let mut kept = kinked;

    kept.simplify2(false);
    assert_eq!(
      points_of(&kept),
      vec![point(0, 0), point(1000, 1), point(2000, 0), point(3000, 0)]
    );
  }

  #[test]
  fn simplify2_ignores_the_closed_flag() {
    // `shape_line_chain.cpp:2906` never looks at `m_closed`, so the
    // closing segment is never simplified. The first vertex here is
    // colinear with the last and the second, and survives.
    let points = vec![
      point(0, 0),
      point(1000, 0),
      point(2000, 0),
      point(2000, 1000),
      point(-1000, 0),
    ];

    let mut chain = LineChain::from_points(points, true);

    chain.simplify2(true);

    // A closed aware simplifier would drop point 0, which is colinear
    // with its neighbour across the seam. This one keeps it and only
    // collapses the run inside the chain.
    assert_eq!(
      points_of(&chain),
      vec![
        point(0, 0),
        point(2000, 0),
        point(2000, 1000),
        point(-1000, 0)
      ]
    );
    assert!(chain.is_closed());
  }

  #[test]
  fn simplify2_special_cases_a_three_point_chain() {
    // `shape_line_chain.cpp:2913`: fewer than three points is left alone,
    // and exactly three points only lose an exact duplicate of the first.
    let mut colinear =
      LineChain::from_slice(&[point(0, 0), point(50, 0), point(100, 0)], false);

    colinear.simplify2(true);
    assert_eq!(colinear.point_count(), 3);

    let mut duplicated =
      LineChain::from_slice(&[point(0, 0), point(0, 0), point(100, 0)], false);

    duplicated.simplify2(true);
    assert_eq!(points_of(&duplicated), vec![point(0, 0), point(100, 0)]);

    let mut two = LineChain::from_slice(&[point(0, 0), point(0, 0)], false);

    two.simplify2(true);
    assert_eq!(two.point_count(), 2);
  }

  // ---------------------------------------------------------------
  // CompareGeometry
  // ---------------------------------------------------------------

  #[test]
  fn compare_geometry_simplifies_before_it_compares() {
    // KiCad `CompareGeometry`, `test_shape_line_chain.cpp:1250`, cases 1,
    // 2 and 6. Cases 3, 4 and 5 exercise the `aEpsilon` and
    // `aCyclicalCompare` parameters, which the router never passes and
    // which are not ported.
    let chain1 = LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(100, 100), point(0, 100)],
      true,
    );

    // 1. Identical chains.
    assert!(chain1.compare_geometry(&chain1.clone()));

    // 2. Different chains.
    let mut chain2 = chain1.clone();

    chain2.set_point(2, point(101, 101));
    assert!(!chain1.compare_geometry(&chain2));

    // 6. A colinear extra vertex does not make a difference, because
    // `CompareGeometry` simplifies both sides first.
    let chain4 = LineChain::from_slice(
      &[
        point(0, 0),
        point(50, 0),
        point(100, 0),
        point(100, 100),
        point(0, 100),
      ],
      true,
    );

    assert!(chain1.compare_geometry(&chain4));
  }

  #[test]
  fn compare_geometry_is_not_direction_agnostic() {
    // KiCad `CompareGeometryReversed`,
    // `test_shape_line_chain.cpp:1299`. The second half of that case
    // needs `aCyclicalCompare`, which is not ported.
    let chain_a = LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(100, 100), point(0, 100)],
      true,
    );
    let chain_b = LineChain::from_slice(
      &[point(0, 0), point(0, 100), point(100, 100), point(100, 0)],
      true,
    );

    assert!(!chain_a.compare_geometry(&chain_b));
  }

  // ---------------------------------------------------------------
  // The helper Simplify bottoms out in
  // ---------------------------------------------------------------

  #[test]
  fn test_segment_hit_is_a_distance_to_the_segment_not_to_the_line() {
    // `libs/kimath/src/trigo.cpp:171`. A point beyond the end of the
    // segment is rejected by the bounding box even though it sits exactly
    // on the infinite line.
    assert!(test_segment_hit(
      point(50, 0),
      point(0, 0),
      point(100, 0),
      0
    ));
    assert!(!test_segment_hit(
      point(150, 0),
      point(0, 0),
      point(100, 0),
      0
    ));

    // A tolerance of zero accepts a point whose squared distance to the
    // segment rounds down to zero, which is not the same as algebraic
    // colinearity.
    assert!(test_segment_hit(
      point(2000, 0),
      point(1000, 1),
      point(3000, 0),
      0
    ));
    assert!(!test_segment_hit(
      point(1000, 1),
      point(0, 0),
      point(2000, 0),
      0
    ));
    assert!(test_segment_hit(
      point(1000, 1),
      point(0, 0),
      point(2000, 0),
      1
    ));
  }
}
