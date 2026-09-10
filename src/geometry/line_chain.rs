// SPDX-License-Identifier: GPL-3.0-or-later

//! Polylines, the container the router lives in.
//!
//! [`LineChain`] is the port of KiCad's `SHAPE_LINE_CHAIN`,
//! `libs/kimath/include/geometry/shape_line_chain.h:77`: the container,
//! its editing members, the queries that read points and segments, and
//! the intersection, collision, distance, nearest point, containment and
//! self intersection queries the router asks of it.
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
//!
//! Members of `SHAPE_LINE_CHAIN` that are not here, and why.
//!
//! - No caller anywhere in the router core: `Intersects( const SEG& )`
//!   (`slc.cpp:1773`), `ClosestPoints` (`slc.cpp:751`), `ClosestSegments`
//!   (`slc.cpp:666`), `ClosestSegmentsFast` (`slc.cpp:505`, which note 01
//!   section 6.8 item 19 shows assumes a closed chain), `FindSegment`
//!   (`slc.cpp:1257`), `OffsetLine` (`slc.cpp:3007`), `Rotate`
//!   (`slc.cpp:495`), `TransformToPolygon` (`slc.cpp:3124`) and the
//!   `POINT_INSIDE_TRACKER` (`slc.cpp:3140`).
//! - Called, but from a part of the router this milestone has not
//!   reached: `PointAlong` (`slc.cpp:2671`), which only the multi item
//!   dragger uses (`pcbnew/router/pns_multi_dragger.cpp:313`).
//! - An editing member, so it belongs with part 1's mutators rather than
//!   with these queries: `RemoveDuplicatePoints` (`slc.cpp:2720`, called
//!   from `pcbnew/router/pns_node.cpp:1204`).
//! - Waiting for the arc vectors: `SelfIntersectingWithArcs`
//!   (`slc.cpp:2234`) and every member that reads `m_arcs`.
//! - Dead or deliberately dropped: `m_accuracy` (`slc.h:994`), which
//!   every constructor sets to zero and nothing reads, and the bounding
//!   box cache with `GenerateBBoxCache` (`slc.h:468`), for the reason
//!   given above.
//! - Debug serialisation, which the crate will grow its own form of:
//!   `Format` and `Parse` (`slc.cpp:2506`, `:2623`), which note 01
//!   section 6.8 item 16 shows are not round trip compatible anyway.

use std::fmt;

use crate::geometry::box2::Box2;
use crate::geometry::math::rescale;
use crate::geometry::seg::{Seg, distance_from_squared};
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

/// Where an intersection landed on one of the two chains.
///
/// Replaces the `index_our` plus `is_corner_our` (and `index_their` plus
/// `is_corner_their`) pairs of `SHAPE_LINE_CHAIN::INTERSECTION`,
/// `libs/kimath/include/geometry/shape_line_chain.h:86`, as
/// `DESIGN.md` section 3 asks for.
///
/// KiCad stores one `int` per chain and a flag beside it. When the hit
/// lands exactly on a segment's `A` endpoint the flag is raised and the
/// index is left alone, so it names the corner already; when it lands on
/// the `B` endpoint the flag is raised **and the index is incremented**
/// (`libs/kimath/src/geometry/shape_line_chain.cpp:1895`, `:1910`,
/// `:1929`, `:1940`), so the index again names the corner. That is the
/// aliasing note 01 section 6.6 warns about: a raw `index_our` can come
/// out equal to `SegmentCount()`, which is out of range as a segment
/// index.
///
/// This type carries the distinction in the tag instead, and
/// [`Hit::Corner`] always holds a **point** index that is in range for
/// its chain. For an open chain the raw KiCad value already is, since
/// `SegmentCount() == PointCount() - 1`. For a closed chain the value
/// `SegmentCount() == PointCount()` means the vertex at index 0, and this
/// type stores the 0. That is why
/// `PNS::HullIntersection`'s compensation, `if( p.index_our >=
/// hull.SegmentCount() ) p.index_our -= hull.SegmentCount();`
/// (`pcbnew/router/pns_utils.cpp:423`), has no counterpart here: a
/// transcription of that routine indexes [`Hit::Corner`] straight into
/// the hull's points and only has to wrap the *predecessor* segment index
/// itself.
///
/// [`Hit::Segment`] holds a segment index, always in range.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Hit {
  /// The intersection lies on the segment at this index and is not one of
  /// its endpoints.
  Segment(usize),
  /// The intersection is exactly the vertex at this point index.
  Corner(usize),
}

impl Hit {
  /// The index this hit carries, whichever kind it is.
  ///
  /// Two router call sites want the bare number that KiCad's `index_our`
  /// and `index_their` hold: `pcbnew/router/pns_line_placer.cpp:128`
  /// picks the intersection with the smallest `index_our`, and
  /// `pcbnew/router/pns_node.cpp:400` feeds `index_their` to
  /// [`LineChain::path_length`] as its segment hint.
  pub fn index(self) -> usize {
    match self {
      Hit::Segment(index) => index,
      Hit::Corner(index) => index,
    }
  }

  /// Whether the intersection landed exactly on a vertex.
  ///
  /// The `is_corner_our` / `is_corner_their` flags,
  /// `libs/kimath/include/geometry/shape_line_chain.h:99` and `:104`.
  pub fn is_corner(self) -> bool {
    matches!(self, Hit::Corner(_))
  }
}

/// One point where a chain meets a segment or another chain.
///
/// Port of `SHAPE_LINE_CHAIN::INTERSECTION`,
/// `libs/kimath/include/geometry/shape_line_chain.h:86`. Two fields of
/// KiCad's record are gone. The index and corner flag of each side are
/// folded into a [`Hit`], and `valid` (`slc.h:109`) is dropped: nothing
/// outside `PNS::HullIntersection` reads it, and that routine writes it
/// itself before deciding whether to keep a record
/// (`pcbnew/router/pns_utils.cpp:414`), so note 01 section 14.3 puts it
/// there rather than here.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Intersection {
  /// The intersection point.
  ///
  /// Port of `INTERSECTION::p`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:89`.
  pub point: Vec2,
  /// Where the point sits on the chain the query was made on.
  ///
  /// Port of `index_our` and `is_corner_our`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:92` and `:99`.
  pub ours: Hit,
  /// Where the point sits on the other chain, or `None` when the query
  /// was made against a bare [`Seg`].
  ///
  /// Port of `index_their` and `is_corner_their`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:96` and `:104`.
  /// `None` reproduces the `index_their = -1` that
  /// [`LineChain::intersect_seg`] writes
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:1758`), where there
  /// is no second chain to index into.
  pub theirs: Option<Hit>,
}

/// A chain came closer to something than the clearance allowed.
///
/// Replaces the `bool` return plus the `int* aActual` and
/// `VECTOR2I* aLocation` out parameters of the two `SHAPE_LINE_CHAIN`
/// collision members, `libs/kimath/include/geometry/shape_line_chain.h:195`
/// and `:210`. KiCad writes both out parameters only when it returns
/// `true`, which is exactly what an `Option` says.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Collision {
  /// The distance between the two shapes in nanometres, zero when they
  /// touch or overlap.
  ///
  /// Port of `*aActual`. KiCad computes it as `sqrt` of the squared
  /// distance in `f64` and truncates
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:474`, `:864`); this
  /// takes the exact integer square root, the choice note 01 section 14.1
  /// records for the whole port.
  pub actual: i32,
  /// A point near the collision.
  ///
  /// Port of `*aLocation`. For a point that is inside a closed chain it
  /// is the query point itself; otherwise it is the point **on the
  /// chain** that is nearest to the other shape
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:471`, `:861`).
  pub location: Vec2,
}

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

  /// The point a given distance along the chain, measured from its start.
  ///
  /// Port of `PointAlong`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2671`. The walk sums
  /// [`Seg::length`] segment by segment and stops at the first segment
  /// the distance falls inside, taking the point
  /// `A + (B - A).Resize( remaining )` on it; a distance of zero is the
  /// first point and a distance past the end is the last one. The sum is
  /// therefore the same rounded per segment sum [`LineChain::length`]
  /// answers, so `point_along( length() )` is the last point exactly.
  ///
  /// `path_length` is an `i64` where KiCad's is an `int`, because
  /// [`LineChain::length`] is an `i64` here and its one caller,
  /// `clipToOtherLine` (`pcbnew/router/pns_multi_dragger.cpp:313`), feeds
  /// that straight in. Note 06 erratum E25 is the narrowing KiCad does at
  /// that call site, which overflows above about 2.147 metres of chain
  /// and which this signature cannot inherit.
  ///
  /// [`None`] for an empty chain, where both of KiCad's returns,
  /// `CPoint( 0 )` and `CLastPoint()`, index out of range. A negative
  /// `path_length` answers the first point, because
  /// [`Vec2::resize`] reverses the direction for a negative length and
  /// the first segment always passes the `total + l >= path_length`
  /// test; that is KiCad's behaviour too, and no caller passes one.
  pub fn point_along(&self, path_length: i64) -> Option<Vec2> {
    // :2675
    if path_length == 0 {
      return self.points.first().copied();
    }

    // :2673
    let mut total: i64 = 0;

    // :2678
    for index in 0..self.segment_count() {
      let seg = self.segment(index);
      let length = i64::from(seg.length());

      // :2683
      if total + length >= path_length {
        let delta = seg.b - seg.a;
        // The remainder is at most this segment's own length, so it fits
        // an `i32` for every reachable `path_length`; the clamp is only
        // there for the unreachable negative one.
        let remaining = (path_length - total)
          .clamp(i64::from(i32::MIN), i64::from(i32::MAX))
          as i32;

        // :2686
        return Some(seg.a + delta.resize(remaining));
      }

      total += length;
    }

    // :2692
    self.points.last().copied()
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

  /// Collapse runs of consecutive equal vertices.
  ///
  /// Port of `RemoveDuplicatePoints`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2720`, which is stage
  /// one of [`LineChain::simplify2`] on its own: no colinear vertex is
  /// touched, and neither is the closed flag, so a closed chain whose last
  /// point equals its first keeps both. The three point special case is
  /// KiCad's: only a duplicate of the first point is removed, and a chain
  /// of fewer than three points is left alone so that it stays a line.
  ///
  /// `NODE::AssembleLine` runs it on every assembled line before handing
  /// it out, with the comment "do NOT remove colinear segments here"
  /// (`pcbnew/router/pns_node.cpp:1204`).
  pub fn remove_duplicate_points(&mut self) {
    if self.points.len() < 3 {
      return;
    }

    if self.points.len() == 3 {
      if self.points[0] == self.points[1] {
        self.remove(1);
      }

      return;
    }

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

    self.points = unique;
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
  // Intersections
  // ---------------------------------------------------------------

  /// Every point where a segment crosses this chain, nearest end first.
  ///
  /// Port of `Intersect( const SEG&, INTERSECTIONS& )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1731`. The results
  /// are ordered by squared distance from `seg.a`
  /// (`shape_line_chain.cpp:1767`), which is what makes the optimizer's
  /// breakout builder able to take entry zero as the nearest hit
  /// (`pcbnew/router/pns_optimizer.cpp:961`).
  ///
  /// Every record comes back as [`Hit::Segment`] with
  /// [`Intersection::theirs`] unset, even when the point is exactly a
  /// vertex: KiCad's `SEG` overload never raises the corner flags
  /// (`shape_line_chain.cpp:1760`), unlike
  /// [`LineChain::intersect_chain`]. A caller that needs to know whether
  /// the hit was a vertex has to ask [`LineChain::find`].
  ///
  /// Two deviations from KiCad, both from note 01 section 6.8 item 10.
  /// KiCad appends to a caller supplied vector and returns its **total**
  /// size, then sorts that whole vector, so entries a caller had already
  /// put there are reordered; this returns a fresh `Vec`. And KiCad sorts
  /// with `std::sort`, which is not stable, so hits at equal distance come
  /// out in an unspecified order; this sorts stably, so equal distances
  /// keep chain order, which `DESIGN.md` section 8 requires.
  pub fn intersect_seg(&self, seg: &Seg) -> Vec<Intersection> {
    let segment_min_x = seg.a.x.min(seg.b.x);
    let segment_max_x = seg.a.x.max(seg.b.x);
    let segment_min_y = seg.a.y.min(seg.b.y);
    let segment_max_y = seg.a.y.max(seg.b.y);

    let mut found: Vec<Intersection> = Vec::new();

    for index in 0..self.segment_count() {
      let candidate = self.segment(index);

      if candidate.a.x.max(candidate.b.x) < segment_min_x
        || candidate.a.x.min(candidate.b.x) > segment_max_x
        || candidate.a.y.max(candidate.b.y) < segment_min_y
        || candidate.a.y.min(candidate.b.y) > segment_max_y
      {
        continue;
      }

      if let Some(point) = candidate.intersect(seg, false, false) {
        found.push(Intersection {
          point,
          ours: Hit::Segment(index),
          theirs: None,
        });
      }
    }

    found.sort_by_key(|intersection| {
      intersection
        .point
        .widening_sub(seg.a)
        .squared_euclidean_norm()
    });

    found
  }

  /// Every point where another chain crosses this one.
  ///
  /// Port of `Intersect( const SHAPE_LINE_CHAIN&, INTERSECTIONS&, bool
  /// aExcludeColinearAndTouching, BOX2I* aChainBBox )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1802`.
  ///
  /// `include_colinear_and_touching` is KiCad's
  /// `aExcludeColinearAndTouching` turned the right way round, which note
  /// 01 section 6.8 item 11 asks for: the branch it guards is written
  /// `if( !aExcludeColinearAndTouching && a.Collinear( b ) )`
  /// (`shape_line_chain.cpp:1882`), so KiCad's default of `false` means
  /// **include**. Pass `true` to transcribe a KiCad call site that uses
  /// the default, which both router call sites do
  /// (`pcbnew/router/pns_line_placer.cpp:125`,
  /// `pcbnew/router/pns_utils.cpp:403`). With it set, a pair of collinear
  /// segments contributes one record per endpoint of either segment that
  /// the other contains, up to four; without it, the pair falls through
  /// to [`Seg::intersect`], which answers collinear overlap with the
  /// single midpoint of the overlap interval.
  ///
  /// The corner rule that [`Hit`] documents applies to both sides. Note
  /// that KiCad reuses one `INTERSECTION` local across the up to four
  /// pushes of the collinear branch without resetting it
  /// (`shape_line_chain.cpp:1884` to `:1945`), so a record pushed later
  /// inherits the corner flag, and the incremented index, of an earlier
  /// one. That is reproduced: the state carries here in the same way.
  ///
  /// Two deviations. KiCad appends to a caller supplied vector and
  /// returns the total size (note 01 section 6.8 item 10); this returns a
  /// fresh `Vec`. And KiCad walks the other chain's segments in an order
  /// sorted by their minimum x, an indexing trick for the `upper_bound`
  /// pruning at `shape_line_chain.cpp:1861`; this walks them in chain
  /// order with the same axis aligned rejection, which yields the same
  /// **set** of records in a deterministic order. No consumer depends on
  /// the order: the placer takes the record with the smallest `index_our`
  /// (`pns_line_placer.cpp:128`), the walkaround splits at every point
  /// (`pcbnew/router/pns_line.cpp:369`) and the node takes the shortest
  /// path length (`pcbnew/router/pns_node.cpp:403`).
  ///
  /// KiCad's `aChainBBox` parameter, a precomputed bounding box for the
  /// other chain, is not ported: no router call site passes it.
  pub fn intersect_chain(
    &self,
    other: &LineChain,
    include_colinear_and_touching: bool,
  ) -> Vec<Intersection> {
    let our_segment_count = self.segment_count();
    let their_segment_count = other.segment_count();

    if our_segment_count == 0 || their_segment_count == 0 {
      return Vec::new();
    }

    // `aChain.BBox()` with its default clearance of zero, which still
    // grows the box by the other chain's width (`slc.h:457`).
    let Some(their_box) = other.bbox(0) else {
      return Vec::new();
    };

    let our_point_count = self.points.len();
    let their_point_count = other.points.len();
    let mut found: Vec<Intersection> = Vec::new();

    for our_index in 0..our_segment_count {
      let ours = self.segment(our_index);

      let our_min_x = i64::from(ours.a.x.min(ours.b.x));
      let our_max_x = i64::from(ours.a.x.max(ours.b.x));
      let our_min_y = i64::from(ours.a.y.min(ours.b.y));
      let our_max_y = i64::from(ours.a.y.max(ours.b.y));

      if our_max_x < their_box.left()
        || our_min_x > their_box.right()
        || our_max_y < their_box.top()
        || our_min_y > their_box.bottom()
      {
        continue;
      }

      for their_index in 0..their_segment_count {
        let theirs = other.segment(their_index);

        // The combined effect of the `upper_bound` cutoff at
        // `shape_line_chain.cpp:1861` and the per entry rejection at
        // `:1873`, which together are a plain box overlap test.
        if i64::from(theirs.a.x.max(theirs.b.x)) < our_min_x
          || i64::from(theirs.a.x.min(theirs.b.x)) > our_max_x
          || i64::from(theirs.a.y.max(theirs.b.y)) < our_min_y
          || i64::from(theirs.a.y.min(theirs.b.y)) > our_max_y
        {
          continue;
        }

        // KiCad's single `INTERSECTION is`, mutated in place across the
        // pushes below (`shape_line_chain.cpp:1877`).
        let mut index_our = our_index;
        let mut index_their = their_index;
        let mut corner_our = false;
        let mut corner_their = false;

        let crossing = ours.intersect(&theirs, false, false);

        if include_colinear_and_touching && ours.collinear(&theirs) {
          if ours.contains_point(theirs.a) {
            corner_their = true;
            found.push(Intersection {
              point: theirs.a,
              ours: hit_at(index_our, corner_our, our_point_count),
              theirs: Some(hit_at(
                index_their,
                corner_their,
                their_point_count,
              )),
            });
          }

          if ours.contains_point(theirs.b) {
            index_their += 1;
            corner_their = true;
            found.push(Intersection {
              point: theirs.b,
              ours: hit_at(index_our, corner_our, our_point_count),
              theirs: Some(hit_at(
                index_their,
                corner_their,
                their_point_count,
              )),
            });
          }

          if theirs.contains_point(ours.a) {
            corner_our = true;
            found.push(Intersection {
              point: ours.a,
              ours: hit_at(index_our, corner_our, our_point_count),
              theirs: Some(hit_at(
                index_their,
                corner_their,
                their_point_count,
              )),
            });
          }

          if theirs.contains_point(ours.b) {
            index_our += 1;
            corner_our = true;
            found.push(Intersection {
              point: ours.b,
              ours: hit_at(index_our, corner_our, our_point_count),
              theirs: Some(hit_at(
                index_their,
                corner_their,
                their_point_count,
              )),
            });
          }
        } else if let Some(point) = crossing {
          if point == ours.a {
            corner_our = true;
          }

          if point == ours.b {
            corner_our = true;
            index_our += 1;
          }

          if point == theirs.a {
            corner_their = true;
          }

          if point == theirs.b {
            corner_their = true;
            index_their += 1;
          }

          found.push(Intersection {
            point,
            ours: hit_at(index_our, corner_our, our_point_count),
            theirs: Some(hit_at(index_their, corner_their, their_point_count)),
          });
        }
      }
    }

    found
  }

  /// Whether another chain touches or crosses this one.
  ///
  /// Port of `Intersects( const SHAPE_LINE_CHAIN& )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2610`, which runs the
  /// full [`LineChain::intersect_chain`] with the colinear and touching
  /// records included and asks whether anything came back. The diff pair
  /// placer uses it to reject a candidate pair whose two lines meet
  /// (`pcbnew/router/pns_diff_pair.cpp:257`).
  pub fn intersects_chain(&self, other: &LineChain) -> bool {
    !self.intersect_chain(other, true).is_empty()
  }

  /// The first place where the chain crosses or touches itself.
  ///
  /// Port of `SelfIntersecting`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2135`. Segment pairs
  /// are tried in the order `s1 < s2` and the first hit wins, so the
  /// answer is the earliest crossing along the chain and not the only
  /// one. Both indices come back as [`Hit::Segment`]: KiCad leaves the
  /// corner flags, and `valid`, false on this record (note 01 section 6.8
  /// item 12).
  ///
  /// Three rules decide what counts. A vertex of the later segment that
  /// merely **lies on** the earlier one is a hit, through
  /// [`Seg::contains_point`] with its squared tolerance of 3, which is
  /// why the axis aligned rejection pads by 2 nm
  /// (`shape_line_chain.cpp:2157`). Adjacent segments are exempt from the
  /// `a2` test only, `s1 + 1 != s2` at `:2181`, because they legitimately
  /// share that vertex. And the closing joint of a closed chain is exempt
  /// from the `b2` test by index, `!( closed && s1 == 0 && s2 ==
  /// segCount - 1 )` at `:2189`, because the last segment ending on the
  /// first segment's start is what closing means. Note that the exemption
  /// is spelled with the *segment* indices, not with a geometric test, so
  /// a chain that returns to its start in the middle is still reported.
  ///
  /// KiCad computes the padded rejection box in wrapping 32 bit
  /// arithmetic; this widens to `i64` first, so a chain that reaches the
  /// coordinate limit cannot fold a rejection into an acceptance.
  ///
  /// The arc exact `SelfIntersectingWithArcs` (`:2234`) is not ported:
  /// there are no arcs yet and the router never calls it.
  pub fn self_intersecting(&self) -> Option<Intersection> {
    let segment_count = self.segment_count();

    if segment_count < 2 {
      return None;
    }

    for first_index in 0..segment_count {
      let first = self.segment(first_index);

      // Expanded by 2 to cover `SEG::Contains`'s squared tolerance of 3.
      let first_min_x = i64::from(first.a.x.min(first.b.x)) - 2;
      let first_max_x = i64::from(first.a.x.max(first.b.x)) + 2;
      let first_min_y = i64::from(first.a.y.min(first.b.y)) - 2;
      let first_max_y = i64::from(first.a.y.max(first.b.y)) + 2;

      for second_index in (first_index + 1)..segment_count {
        let second = self.segment(second_index);

        if first_max_x < i64::from(second.a.x.min(second.b.x))
          || i64::from(second.a.x.max(second.b.x)) < first_min_x
        {
          continue;
        }

        if first_max_y < i64::from(second.a.y.min(second.b.y))
          || i64::from(second.a.y.max(second.b.y)) < first_min_y
        {
          continue;
        }

        let record = |point: Vec2| Intersection {
          point,
          ours: Hit::Segment(first_index),
          theirs: Some(Hit::Segment(second_index)),
        };

        if first_index + 1 != second_index && first.contains_point(second.a) {
          return Some(record(second.a));
        } else if first.contains_point(second.b)
          && !(self.closed
            && first_index == 0
            && second_index == segment_count - 1)
        {
          return Some(record(second.b));
        } else if let Some(point) = first.intersect(&second, true, false) {
          return Some(record(point));
        }
      }
    }

    None
  }

  // ---------------------------------------------------------------
  // Collision
  // ---------------------------------------------------------------

  /// Whether a point comes closer to the chain than a clearance.
  ///
  /// Port of `Collide( const VECTOR2I&, int, int*, VECTOR2I* )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:426`. A point inside
  /// a closed chain collides at distance zero and reports itself as the
  /// location (`:429`); note that the containment test is run with the
  /// **clearance as the accuracy**, so a clearance above 1 also catches a
  /// point sitting on the outline through
  /// [`LineChain::point_on_edge`].
  ///
  /// The comparison is `closest == 0 || closest < clearance` on the
  /// squared values (`:468`), so a point exactly `clearance` away does
  /// **not** collide while a point exactly on the chain always does, at
  /// any clearance including zero. That strictness is what the `- 1` in
  /// `pcbnew/router/pns_item.cpp:249` is written against, and note 01
  /// section 14.6 warns that the walkaround can fail to terminate if the
  /// two disagree.
  ///
  /// Deviation: KiCad breaks out of the scan early when it has a
  /// collision and the caller asked for no actual distance
  /// (`:461`), which leaves `*aLocation` at a segment that is not
  /// necessarily the nearest. This always takes the `aActual != nullptr`
  /// path, so `actual` and `location` always describe the nearest
  /// segment. The boolean outcome is the same either way, because a later
  /// segment can only shrink an already colliding distance.
  pub fn collide_point(
    &self,
    point: Vec2,
    clearance: i32,
  ) -> Option<Collision> {
    if self.closed && self.point_inside(point, clearance) {
      return Some(Collision {
        actual: 0,
        location: point,
      });
    }

    let mut closest_squared = i64::MAX;
    let mut nearest = Vec2::new(0, 0);

    for index in 0..self.segment_count() {
      let segment = self.segment(index);
      let projected = segment.nearest_point_to_point(point);
      let squared = projected.widening_sub(point).squared_euclidean_norm();

      if squared < closest_squared {
        nearest = projected;
        closest_squared = squared;

        if closest_squared == 0 {
          break;
        }
      }
    }

    let clearance_squared = i64::from(clearance) * i64::from(clearance);

    if closest_squared == 0 || closest_squared < clearance_squared {
      return Some(Collision {
        actual: distance_from_squared(closest_squared),
        location: nearest,
      });
    }

    None
  }

  /// Whether a segment comes closer to the chain than a clearance.
  ///
  /// Port of `Collide( const SEG&, int, int*, VECTOR2I* )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:815`. The comparison
  /// and the early exit are the same as in
  /// [`LineChain::collide_point`], and so is the deviation: this always
  /// takes KiCad's `aActual != nullptr` path.
  ///
  /// The closed chain shortcut differs from the point one in a detail
  /// worth keeping. It tests only `seg.a`, not the whole segment, and it
  /// tests it with the **default accuracy of zero** rather than with the
  /// clearance (`:818`), so a segment whose start sits exactly on the
  /// outline of a closed chain does not take the shortcut and is measured
  /// against the segments instead. A segment that passes through a closed
  /// chain without either endpoint inside is likewise not caught by the
  /// shortcut; it is caught by the ordinary scan, since it must cross an
  /// edge.
  ///
  /// The location is `SEG::NearestPoint( const SEG& )` on the winning
  /// chain segment (`:845`), so it lies on the **chain**, not on the
  /// argument.
  pub fn collide_seg(&self, seg: &Seg, clearance: i32) -> Option<Collision> {
    if self.closed && self.point_inside(seg.a, 0) {
      return Some(Collision {
        actual: 0,
        location: seg.a,
      });
    }

    let mut closest_squared = i64::MAX;
    let mut nearest = Vec2::new(0, 0);

    for index in 0..self.segment_count() {
      let segment = self.segment(index);
      let squared = segment.squared_distance_to_segment(seg);

      if squared < closest_squared {
        nearest = segment.nearest_point_to_segment(seg);
        closest_squared = squared;

        if closest_squared == 0 {
          break;
        }
      }
    }

    let clearance_squared = i64::from(clearance) * i64::from(clearance);

    if closest_squared == 0 || closest_squared < clearance_squared {
      return Some(Collision {
        actual: distance_from_squared(closest_squared),
        location: nearest,
      });
    }

    None
  }

  // ---------------------------------------------------------------
  // Distances and nearest points
  // ---------------------------------------------------------------

  /// The squared distance from a point to the chain.
  ///
  /// Port of `SquaredDistance( const VECTOR2I&, bool aOutlineOnly )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1167`. A point inside
  /// a closed chain is at distance zero unless `outline_only` is set,
  /// which is the flag's only effect (`:1171`).
  ///
  /// An empty chain answers `i64::MAX`, which is KiCad's
  /// `VECTOR2I::ECOORD_MAX` sentinel (`math/vector2d.h:72`) reached by a
  /// loop that never runs.
  pub fn squared_distance(&self, point: Vec2, outline_only: bool) -> i64 {
    if self.closed && self.point_inside(point, 0) && !outline_only {
      return 0;
    }

    let mut squared = i64::MAX;

    for index in 0..self.segment_count() {
      squared =
        squared.min(self.segment(index).squared_distance_to_point(point));
    }

    squared
  }

  /// The distance from a point to the chain in nanometres.
  ///
  /// Port of `Distance( const VECTOR2I&, bool aOutlineOnly )`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:886`, the square
  /// root of [`LineChain::squared_distance`]. KiCad takes an `f64` square
  /// root and narrows the result to `int`, which is undefined for the
  /// `ECOORD_MAX` an empty chain produces; this takes the exact integer
  /// square root and saturates, so an empty chain answers `i32::MAX`.
  pub fn distance(&self, point: Vec2, outline_only: bool) -> i32 {
    distance_from_squared(self.squared_distance(point, outline_only))
  }

  /// The point of the chain that is nearest to a point.
  ///
  /// Port of `NearestPoint( const VECTOR2I&, bool
  /// aAllowInternalShapePoints )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2401`. The nearest
  /// **segment** is found first, by [`Seg::distance_to_point`] with ties
  /// going to the earlier segment, and the answer is that segment's
  /// nearest point.
  ///
  /// KiCad's `aAllowInternalShapePoints` is not ported. Its whole body is
  /// inside `if( !aAllowInternalShapePoints )` and every branch of it is
  /// guarded by `IsArcSegment( nearest )` (`:2425` to `:2452`), so it
  /// snaps to arc endpoints and does nothing at all on an arc free chain.
  /// It has to come back with the arc vectors; the router passes both
  /// values (`pcbnew/router/pns_shove.cpp:359` passes `true`,
  /// `pcbnew/router/pns_helpers.cpp:98` passes `false`).
  ///
  /// Returns `None` for an empty chain, where KiCad returns `(0, 0)` with
  /// the comment that the only right answer is not to crash (`:2406`). A
  /// chain of one point answers with that point, which is what KiCad's
  /// failed `wxCHECK` in `Segment` degrades to (`:1293`).
  pub fn nearest_point(&self, point: Vec2) -> Option<Vec2> {
    let first_point = *self.points.first()?;
    let segment_count = self.segment_count();

    if segment_count == 0 {
      return Some(first_point);
    }

    let mut min_distance = i32::MAX;
    let mut nearest = 0usize;

    for index in 0..segment_count {
      let distance = self.segment(index).distance_to_point(point);

      if distance < min_distance {
        min_distance = distance;
        nearest = index;
      }
    }

    Some(self.segment(nearest).nearest_point_to_point(point))
  }

  /// The vertex of the chain that is nearest to the infinite line through
  /// a segment, and its distance to that line.
  ///
  /// Port of `NearestPoint( const SEG&, int& dist )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2459`. Note 01
  /// section 6.8 item 13 flags what this is not: it walks **vertices**,
  /// not segments, and it measures with [`Seg::line_distance`], so it
  /// answers neither the nearest point of the chain to the segment nor
  /// the nearest point to the infinite line. `PNS::MoveDiagonal`
  /// (`pcbnew/router/pns_utils.cpp:293`) is the only caller and depends
  /// on exactly this behaviour, so it is reproduced rather than fixed.
  ///
  /// Ties go to the earlier vertex. Returns `None` for an empty chain,
  /// where KiCad returns `(0, 0)` and leaves `dist` at `INT_MAX`
  /// (`:2464`).
  pub fn nearest_point_to_seg(&self, seg: &Seg) -> Option<(Vec2, i32)> {
    if self.points.is_empty() {
      return None;
    }

    let mut min_distance = i32::MAX;
    let mut nearest = 0usize;

    for (index, point) in self.points.iter().enumerate() {
      let distance = seg.line_distance(*point);

      if distance < min_distance {
        min_distance = distance;
        nearest = index;
      }
    }

    Some((self.points[nearest], min_distance))
  }

  /// The index of the segment nearest to a point.
  ///
  /// Port of `NearestSegment`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2486`, with ties
  /// going to the earlier segment. The walkaround uses it to find where a
  /// collision point sits on a line (`pcbnew/router/pns_line.cpp:693`).
  ///
  /// Returns `None` when the chain has no segments, where KiCad returns
  /// the index 0 it started the scan with.
  pub fn nearest_segment(&self, point: Vec2) -> Option<usize> {
    let segment_count = self.segment_count();

    if segment_count == 0 {
      return None;
    }

    let mut min_distance = i32::MAX;
    let mut nearest = 0usize;

    for index in 0..segment_count {
      let distance = self.segment(index).distance_to_point(point);

      if distance < min_distance {
        min_distance = distance;
        nearest = index;
      }
    }

    Some(nearest)
  }

  // ---------------------------------------------------------------
  // Containment
  // ---------------------------------------------------------------

  /// Whether a point lies inside the closed chain.
  ///
  /// Port of `PointInside( const VECTOR2I&, int aAccuracy, bool
  /// aUseBBoxCache )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1986`, with the cache
  /// flag left off. That is not a simplification: KiCad's bounding box
  /// test is written `if( aUseBBoxCache && GetCachedBBox() && ... )`
  /// (`:1989`), so with the flag false, which is the default and the only
  /// value the router passes, no box test runs at all. There is nothing
  /// to recompute.
  ///
  /// An **open** chain, or one of fewer than three points, is always
  /// outside (`:1994`).
  ///
  /// The rule is a crossing number, not a winding number: a ray is cast
  /// in `+x` and every edge, the closing one included, flips the answer
  /// when it straddles the ray. Two details decide the boundary cases.
  /// The straddle test is the half open `( p1.y >= aPt.y ) != ( p2.y >=
  /// aPt.y )`, so a vertex exactly on the ray belongs to the edge below
  /// it and is counted once, not twice. And the side test is the strict
  /// `aPt.x - p1.x < d` with `d` rounded to nearest by `rescale`
  /// (`src/math/util.cpp:62`), so a point exactly on a non horizontal
  /// edge is **outside** by this rule alone. Horizontal edges are skipped
  /// entirely.
  ///
  /// `accuracy` therefore does not widen the polygon. Up to and including
  /// 1 it is ignored outright (`:2016`); above that the answer is the
  /// crossing number **or** [`LineChain::point_on_edge`] at that
  /// accuracy, which the base class copy of the routine explains as using
  /// "on the edge" as a proxy for "inside" (`:2065`). So a point on the
  /// outline reads as inside only from an accuracy of 2 upwards.
  ///
  /// Deviation: KiCad computes the edge vector and the ray offset in
  /// wrapping 32 bit arithmetic and narrows `d` back to `int`; this
  /// widens to `i64` throughout. The straddle test bounds the numerator
  /// by the denominator, so `d` fits in an `i32` whenever it is used, and
  /// no in range chain can see a difference.
  pub fn point_inside(&self, point: Vec2, accuracy: i32) -> bool {
    let point_count = self.points.len();

    if !self.closed || point_count < 3 {
      return false;
    }

    let mut inside = false;

    for index in 0..point_count {
      let first = self.points[index];
      let second = self.points[if index + 1 == point_count {
        0
      } else {
        index + 1
      }];
      let difference = second.widening_sub(first);

      if difference.y == 0 {
        continue;
      }

      let projected = rescale(
        difference.x,
        i64::from(point.y) - i64::from(first.y),
        difference.y,
      );

      if ((first.y >= point.y) != (second.y >= point.y))
        && (i64::from(point.x) - i64::from(first.x) < projected)
      {
        inside = !inside;
      }
    }

    if accuracy <= 1 {
      inside
    } else {
      inside || self.point_on_edge(point, accuracy)
    }
  }

  /// Whether a point lies on one of the chain's edges.
  ///
  /// Port of `PointOnEdge`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2074`, which is
  /// [`LineChain::edge_containing_point`] answering with an edge. Unlike
  /// [`LineChain::point_inside`] it works on an open chain.
  pub fn point_on_edge(&self, point: Vec2, accuracy: i32) -> bool {
    self.edge_containing_point(point, accuracy).is_some()
  }

  /// The index of the first edge that contains a point.
  ///
  /// Port of `EdgeContainingPoint`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2080`. The tolerance
  /// is `accuracy + 1` compared as a square and inclusively (`:2081`),
  /// one of the constants note 01 section 13 lists, so an accuracy of
  /// zero still accepts a point up to a nanometre off the edge. An exact
  /// match with either endpoint of a segment short circuits ahead of the
  /// distance test (`:2101`), which matters for a chain whose vertex is
  /// further from the segment interior than the tolerance would allow.
  ///
  /// A chain of a single point is a special case: it is contained when
  /// the point is within the same tolerance of that vertex (`:2092`). An
  /// empty chain contains nothing.
  ///
  /// Returns `None` where KiCad returns `-1`. The router never calls this
  /// directly, only through [`LineChain::point_on_edge`].
  pub fn edge_containing_point(
    &self,
    point: Vec2,
    accuracy: i32,
  ) -> Option<usize> {
    let threshold = i64::from(accuracy) + 1;
    let threshold_squared = threshold * threshold;
    let point_count = self.points.len();

    if point_count == 0 {
      return None;
    }

    if point_count == 1 {
      return if self.points[0].squared_distance(point) <= threshold_squared {
        Some(0)
      } else {
        None
      };
    }

    for index in 0..self.segment_count() {
      let segment = self.segment(index);

      if segment.a == point || segment.b == point {
        return Some(index);
      }

      if segment.squared_distance_to_point(point) <= threshold_squared {
        return Some(index);
      }
    }

    None
  }

  /// Whether a point is a vertex of the chain or within a distance of one
  /// of its segments.
  ///
  /// Port of `CheckClearance`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2113`. It reads like
  /// [`LineChain::edge_containing_point`] and is not the same test: the
  /// comparison is on the **unsquared** distance, `s.Distance( aP ) <=
  /// aDist` (`:2127`), so it is subject to the truncation in
  /// [`Seg::distance_to_point`], and a distance of zero accepts anything
  /// less than a nanometre away rather than the `accuracy + 1` band. A
  /// chain of one point matches only that point exactly (`:2118`), and an
  /// empty chain matches nothing.
  ///
  /// The router does not call it; it is ported because it is a public
  /// predicate of the type and the shape level code may reach for it.
  pub fn check_clearance(&self, point: Vec2, distance: i32) -> bool {
    if self.points.is_empty() {
      return false;
    }

    if self.points.len() == 1 {
      return self.points[0] == point;
    }

    for index in 0..self.segment_count() {
      let segment = self.segment(index);

      if segment.a == point || segment.b == point {
        return true;
      }

      if segment.distance_to_point(point) <= distance {
        return true;
      }
    }

    false
  }

  // ---------------------------------------------------------------
  // Area and the three way split
  // ---------------------------------------------------------------

  /// The area enclosed by the closed chain.
  ///
  /// Port of `Area( bool aAbsolute )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:2696`, the trapezoid
  /// form of the shoelace sum. An **open** chain has area zero whatever
  /// its shape (`:2700`).
  ///
  /// The return type is `f64`, as KiCad's is: the sum is accumulated in
  /// `double` and the callers compare it as one, the optimizer picking
  /// the smallest enclosed loop (`pcbnew/router/pns_optimizer.cpp:785`)
  /// and the posture solver comparing the areas of two candidate traces
  /// against the mouse trail (`pcbnew/router/pns_mouse_trail_tracer.cpp:123`).
  /// Values above 2^53 lose exactness, which a board sized polygon in
  /// nanometres reaches, so this is a comparison quantity and not an
  /// exact one.
  ///
  /// With `absolute` the answer is the magnitude. Without, the sign says
  /// which way the chain winds, and the negation at `:2718` is what makes
  /// a clockwise chain in screen coordinates, where y grows downwards,
  /// come out **positive**.
  pub fn area(&self, absolute: bool) -> f64 {
    if !self.closed {
      return 0.0;
    }

    let point_count = self.points.len();
    let mut area = 0.0f64;
    let mut previous = point_count.wrapping_sub(1);

    for index in 0..point_count {
      area += (f64::from(self.points[previous].x)
        + f64::from(self.points[index].x))
        * (f64::from(self.points[previous].y)
          - f64::from(self.points[index].y));
      previous = index;
    }

    if absolute {
      (area * 0.5).abs()
    } else {
      -area * 0.5
    }
  }

  /// Insert a vertex at a point, leaving an existing vertex alone.
  ///
  /// Port of `Split( const VECTOR2I&, true )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1181`, the form with
  /// `aExact` set that part 1 left out. The only difference from
  /// [`LineChain::split`] is the short circuit at `:1188`: a point that
  /// already is a vertex answers with that vertex's index and the search
  /// for a nearer segment never runs. Both forms are needed, because
  /// [`LineChain::split_three_way`] is the exact one's only caller
  /// (`:2885`) while the walkaround, the optimizer and the placer all
  /// call the inexact one.
  pub fn split_exact(&mut self, point: Vec2) -> Option<usize> {
    if let Some(found) = self.find(point, 0) {
      return Some(found);
    }

    self.split(point)
  }

  /// Cut the chain into the part before a point, the part between two
  /// points, and the part after.
  ///
  /// Port of `Split( const VECTOR2I& aStart, const VECTOR2I& aEnd,
  /// SHAPE_LINE_CHAIN& aPre, SHAPE_LINE_CHAIN& aMid, SHAPE_LINE_CHAIN&
  /// aPost )`, `libs/kimath/src/geometry/shape_line_chain.cpp:2877`, the
  /// five argument overload the meander placers use to carve out the
  /// stretch of a line they are about to replace
  /// (`pcbnew/router/pns_meander_placer.cpp:241`,
  /// `pcbnew/router/pns_dp_meander_placer.cpp:262`).
  ///
  /// Neither argument has to be on the chain: each is first snapped with
  /// [`LineChain::nearest_point`] and then inserted as a vertex with
  /// [`LineChain::split_exact`]. If the two land out of order the working
  /// copy is reversed, so `pre` and `post` swap ends relative to the
  /// original chain rather than the range coming back empty.
  ///
  /// The three results are all open and all of width zero, because they
  /// come from [`LineChain::slice`]. `pre` and `post` can be a single
  /// point when the range reaches an end of the chain.
  ///
  /// Returns `None` for an empty chain, and for the case KiCad cannot
  /// express: when a snapped point cannot be located afterwards, where
  /// KiCad's `Find` answers `-1` and the `Slice` calls that follow read
  /// it as a wrapped index. The snap puts both points on the chain, so
  /// the case is not reachable through this API.
  pub fn split_three_way(
    &self,
    start: Vec2,
    end: Vec2,
  ) -> Option<(LineChain, LineChain, LineChain)> {
    let end_on_chain = self.nearest_point(end)?;
    let start_on_chain = self.nearest_point(start)?;

    let mut working = self.clone();

    working.split_exact(end_on_chain);
    working.split_exact(start_on_chain);

    let mut first = working.find(start_on_chain, 0)?;
    let mut last = working.find(end_on_chain, 0)?;

    if first > last {
      working.reverse();
      first = working.find(start_on_chain, 0)?;
      last = working.find(end_on_chain, 0)?;
    }

    let final_index = working.normalize_index(-1)?;
    let pre = working.slice(0, first).ok()?;
    let post = working.slice(last, final_index).ok()?;
    let mid = working.slice(first, last).ok()?;

    Some((pre, mid, post))
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

/// Build a [`Hit`] out of KiCad's index and corner flag pair.
///
/// The corner index is folded back into the chain's point range, which is
/// the compensation `PNS::HullIntersection` writes out by hand at
/// `pcbnew/router/pns_utils.cpp:423`. It only ever fires for a closed
/// chain, where a hit on the last segment's `B` endpoint leaves the
/// incremented index equal to `PointCount()` and the vertex it names is
/// the one at index 0.
fn hit_at(index: usize, corner: bool, point_count: usize) -> Hit {
  if !corner {
    return Hit::Segment(index);
  }

  if point_count > 0 && index >= point_count {
    Hit::Corner(index - point_count)
  } else {
    Hit::Corner(index)
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

// ---------------------------------------------------------------------
// POINT_INSIDE_TRACKER
// ---------------------------------------------------------------------

/// Whether a point is inside a ring assembled from several open chains.
///
/// Port of `SHAPE_LINE_CHAIN::POINT_INSIDE_TRACKER`,
/// `libs/kimath/include/geometry/shape_line_chain.h:126` and
/// `libs/kimath/src/geometry/shape_line_chain.cpp:3131`. It answers the
/// same question as [`LineChain::point_inside`] but for a boundary that
/// arrives in pieces: every [`PointInsideTracker::add_polyline`] call
/// appends one more run of vertices, and
/// [`PointInsideTracker::is_inside`] closes the ring from the last point
/// back to the very first one before answering.
///
/// That is what the shove's direction heuristic needs
/// (`pcbnew/router/pns_shove.cpp:259`): the region it tests is bounded by
/// the obstacle line and by the shoved line walked backwards, two open
/// chains that only form a closed area together.
///
/// The rule is the odd even crossing number of a ray cast in `+x`. A
/// degenerate case (the ray through a vertex, the point exactly on an
/// edge) sets the parity to `-1` and abandons the rest of the polyline
/// being added, which reads as "outside".
///
/// # Not ported
///
/// KiCad's `m_finished` is written by all three degenerate branches and
/// **never read** (`shape_line_chain.cpp:3148`, `:3169`, `:3187`); the
/// only thing the latch actually does is end the current `AddPolyline`
/// early, which is what the `bool` returned by
/// `process_vertex` does here. A later
/// `AddPolyline` resumes as if nothing had happened, in KiCad as here.
///
/// # Deviation
///
/// KiCad computes the cross product as a `double`
/// (`shape_line_chain.cpp:3164`) and the coordinate differences in
/// wrapping 32 bit arithmetic. This widens both to `i64`, which is exact
/// over the whole coordinate range (`DESIGN.md` section 2); the `double`
/// loses the low bits of a product above 2^53 and can disagree there.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct PointInsideTracker {
  /// The point under test. Port of `m_point`.
  point: Vec2,
  /// The very first vertex handed in, which the closing edge runs back
  /// to. Port of `m_firstPoint`.
  first_point: Vec2,
  /// The vertex the next edge starts at. Port of `m_lastPoint`.
  last_point: Vec2,
  /// The crossing parity, or `-1` once a degenerate case has set it.
  /// Port of `m_state`.
  state: i32,
  /// How many vertices have arrived. Port of `m_count`, which is only
  /// read as "is this the first polyline".
  count: usize,
}

impl PointInsideTracker {
  /// A tracker for one point, with no boundary yet.
  ///
  /// Port of the constructor, `shape_line_chain.cpp:3131`.
  pub const fn new(point: Vec2) -> Self {
    Self {
      point,
      first_point: point,
      last_point: point,
      state: 0,
      count: 0,
    }
  }

  /// Append one open run of the boundary.
  ///
  /// Port of `AddPolyline`, `shape_line_chain.cpp:3202`. The first call
  /// also fixes the point the closing edge returns to. Note that KiCad
  /// starts the edge walk at index 1 of every polyline, so the join
  /// between two consecutive polylines is an edge like any other: the
  /// caller is responsible for handing in runs that actually meet.
  ///
  /// An empty chain is skipped; KiCad reads `CPoint( 0 )` unchecked and
  /// would trip its own assertion.
  pub fn add_polyline(&mut self, polyline: &LineChain) {
    if polyline.is_empty() {
      return;
    }

    if self.count == 0 {
      self.last_point = polyline.point(0);
      self.first_point = polyline.point(0);
    }

    self.count += polyline.point_count();

    for index in 1..polyline.point_count() {
      let point = polyline.point(index);

      if !self.process_vertex(self.last_point, point) {
        return;
      }

      self.last_point = point;
    }
  }

  /// Close the ring and answer.
  ///
  /// Port of `IsInside`, `shape_line_chain.cpp:3225`, which processes the
  /// closing edge from the last vertex back to the first and then asks
  /// for a positive parity. It mutates the tracker, so a second call
  /// processes the closing edge again; KiCad has the same shape and no
  /// caller does it twice.
  pub fn is_inside(&mut self) -> bool {
    self.process_vertex(self.last_point, self.first_point);

    self.state > 0
  }

  /// One edge of the boundary.
  ///
  /// Port of `processVertex`, `shape_line_chain.cpp:3140`. It answers
  /// whether the scan should continue; `false` means a degenerate case
  /// has latched the answer to "outside".
  fn process_vertex(&mut self, from: Vec2, to: Vec2) -> bool {
    let point = self.point;

    // :3143. The ray runs through the edge's far vertex.
    if to.y == point.y
      && (to.x == point.x
        || (from.y == point.y && ((to.x > point.x) == (from.x < point.x))))
    {
      self.state = -1;

      return false;
    }

    // :3154. Does the edge straddle the ray at all?
    if (from.y < point.y) == (to.y < point.y) {
      return true;
    }

    // :3156 and :3180, which differ only in the branch that needs no
    // cross product: an edge whose two ends are both to the right of the
    // point always crosses the ray.
    if from.x >= point.x && to.x > point.x {
      self.state = 1 - self.state;

      return true;
    }

    if from.x < point.x && to.x <= point.x {
      return true;
    }

    // :3164 and :3182, the same determinant twice.
    let cross = (i64::from(from.x) - i64::from(point.x))
      * (i64::from(to.y) - i64::from(point.y))
      - (i64::from(to.x) - i64::from(point.x))
        * (i64::from(from.y) - i64::from(point.y));

    // :3167. The point sits exactly on the edge's line.
    if cross == 0 {
      self.state = -1;

      return false;
    }

    if (cross > 0) == (to.y > from.y) {
      self.state = 1 - self.state;
    }

    true
  }
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
  fn point_along_walks_the_chain_by_arc_length() {
    // `shape_line_chain.cpp:2671`.
    let chain = LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(100, 100)],
      false,
    );

    // :2675, the zero case, which never enters the walk.
    assert_eq!(chain.point_along(0), Some(point(0, 0)));
    assert_eq!(chain.point_along(40), Some(point(40, 0)));
    // A distance that lands exactly on a vertex stops on the first
    // segment, because the test at `:2683` is `>=`.
    assert_eq!(chain.point_along(100), Some(point(100, 0)));
    assert_eq!(chain.point_along(160), Some(point(100, 60)));
    // :2692, past the end.
    assert_eq!(chain.point_along(500), Some(point(100, 100)));
    assert_eq!(chain.point_along(chain.length()), Some(point(100, 100)));
  }

  #[test]
  fn point_along_a_diagonal_resizes_the_way_the_hulls_do() {
    // The remainder is handed to `Vec2::resize`, so a 45 degree leg gets
    // KiCad's `sqrt(1/2)` per component rather than a projection.
    let chain = LineChain::from_slice(&[point(0, 0), point(1000, 1000)], false);

    assert_eq!(chain.point_along(1414), Some(point(1000, 1000)));
    assert_eq!(chain.point_along(707), Some(point(500, 500)));
  }

  #[test]
  fn point_along_an_empty_or_single_point_chain() {
    // Both of KiCad's returns index out of range on an empty chain.
    assert_eq!(LineChain::new().point_along(0), None);
    assert_eq!(LineChain::new().point_along(1000), None);

    let dot = LineChain::from_slice(&[point(7, 9)], false);

    assert_eq!(dot.point_along(0), Some(point(7, 9)));
    assert_eq!(dot.point_along(1000), Some(point(7, 9)));
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

  // ---------------------------------------------------------------
  // Intersections
  // ---------------------------------------------------------------

  /// The four unit squares of the walkaround's world, as a closed hull.
  fn closed_square(side: i32) -> LineChain {
    LineChain::from_slice(
      &[
        point(0, 0),
        point(side, 0),
        point(side, side),
        point(0, side),
      ],
      true,
    )
  }

  #[test]
  fn intersect_seg_sorts_by_distance_from_the_segments_start() {
    // `shape_line_chain.cpp:1767` sorts the whole output by squared
    // distance from `aSeg.A`, which is what lets
    // `pcbnew/router/pns_optimizer.cpp:961` read entry zero as the
    // nearest breakout.
    let square = closed_square(10);
    let crossing = Seg::new(point(-5, 5), point(15, 5));
    let hits = square.intersect_seg(&crossing);

    assert_eq!(hits.len(), 2);
    assert_eq!(hits[0].point, point(0, 5));
    assert_eq!(hits[0].ours, Hit::Segment(3));
    assert_eq!(hits[0].theirs, None);
    assert_eq!(hits[1].point, point(10, 5));
    assert_eq!(hits[1].ours, Hit::Segment(1));
  }

  #[test]
  fn intersect_seg_never_reports_a_corner() {
    // The `SEG` overload leaves both corner flags false and
    // `index_their` at `-1` (`shape_line_chain.cpp:1758`), even when the
    // hit is exactly a vertex. Here the segment goes through the corner
    // `(10, 0)`, so both adjoining chain segments report it, as segments.
    let square = closed_square(10);
    let through_corner = Seg::new(point(5, -5), point(15, 5));
    let hits = square.intersect_seg(&through_corner);

    assert_eq!(hits.len(), 2);
    assert!(hits.iter().all(|hit| hit.point == point(10, 0)));
    assert!(hits.iter().all(|hit| hit.theirs.is_none()));
    // Equal distances keep chain order, because the sort is stable.
    assert_eq!(hits[0].ours, Hit::Segment(0));
    assert_eq!(hits[1].ours, Hit::Segment(1));
  }

  #[test]
  fn intersect_chain_on_a_vertex_of_both_chains_reports_two_corners() {
    // The two chains meet only at `(10, 0)`, which is point 1 of each.
    // All four segment pairs see it, and every record names the corner
    // rather than the segment.
    let ours =
      LineChain::from_slice(&[point(0, 0), point(10, 0), point(10, 10)], false);
    let theirs = LineChain::from_slice(
      &[point(0, 10), point(10, 0), point(20, 10)],
      false,
    );

    let hits = ours.intersect_chain(&theirs, true);

    assert_eq!(hits.len(), 4);

    for hit in &hits {
      assert_eq!(hit.point, point(10, 0));
      assert_eq!(hit.ours, Hit::Corner(1));
      assert_eq!(hit.theirs, Some(Hit::Corner(1)));
    }
  }

  #[test]
  fn intersect_chain_on_the_b_endpoint_of_a_closing_segment_wraps_to_corner_zero()
   {
    // The aliasing rule of note 01 section 6.6. The hit on the hull's
    // closing segment (index 3) lands on that segment's `B`, so KiCad
    // increments `index_our` to 4 == `SegmentCount()`. `Hit::Corner`
    // carries the point index instead, so it comes back as corner 0, and
    // `PNS::HullIntersection`'s modulo at
    // `pcbnew/router/pns_utils.cpp:423` has nothing left to do.
    let hull = closed_square(10);
    let line = LineChain::from_slice(&[point(-5, 0), point(5, 0)], false);

    let hits = hull.intersect_chain(&line, true);

    assert_eq!(hits.len(), 3);

    // The first two come from the collinear overlap with the bottom edge.
    assert_eq!(hits[0].point, point(5, 0));
    assert_eq!(hits[0].ours, Hit::Segment(0));
    assert_eq!(hits[0].theirs, Some(Hit::Corner(1)));

    assert_eq!(hits[1].point, point(0, 0));
    assert_eq!(hits[1].ours, Hit::Corner(0));
    // KiCad reuses one record across the pushes of the collinear branch
    // without resetting it (`shape_line_chain.cpp:1884` to `:1945`), so
    // this one inherits the corner flag and the incremented index of the
    // push before it even though the point is not `(5, 0)`.
    assert_eq!(hits[1].theirs, Some(Hit::Corner(1)));

    // And this is the closing segment's `B` endpoint.
    assert_eq!(hits[2].point, point(0, 0));
    assert_eq!(hits[2].ours, Hit::Corner(0));
    assert_eq!(hits[2].theirs, Some(Hit::Segment(0)));
  }

  #[test]
  fn intersect_chain_on_collinear_overlap_depends_on_the_flag() {
    let ours = LineChain::from_slice(&[point(0, 0), point(100, 0)], false);
    let theirs = LineChain::from_slice(&[point(30, 0), point(70, 0)], false);

    // Included: one record per contained endpoint, so the overlap comes
    // back as its two ends (`shape_line_chain.cpp:1884`).
    let included = ours.intersect_chain(&theirs, true);

    assert_eq!(included.len(), 2);
    assert_eq!(included[0].point, point(30, 0));
    assert_eq!(included[0].ours, Hit::Segment(0));
    assert_eq!(included[0].theirs, Some(Hit::Corner(0)));
    assert_eq!(included[1].point, point(70, 0));
    assert_eq!(included[1].ours, Hit::Segment(0));
    assert_eq!(included[1].theirs, Some(Hit::Corner(1)));

    // Excluded: the pair falls through to `SEG::Intersect`, which answers
    // a collinear overlap with the midpoint of the overlap interval.
    let excluded = ours.intersect_chain(&theirs, false);

    assert_eq!(excluded.len(), 1);
    assert_eq!(excluded[0].point, point(50, 0));
    assert_eq!(excluded[0].ours, Hit::Segment(0));
    assert_eq!(excluded[0].theirs, Some(Hit::Segment(0)));
  }

  #[test]
  fn intersect_chain_is_empty_when_either_chain_has_no_segments() {
    let square = closed_square(10);
    let single = LineChain::from_slice(&[point(5, 5)], false);

    assert!(square.intersect_chain(&single, true).is_empty());
    assert!(single.intersect_chain(&square, true).is_empty());
    assert!(!square.intersects_chain(&single));
  }

  #[test]
  fn intersects_chain_answers_the_diff_pair_placers_question() {
    // `pcbnew/router/pns_diff_pair.cpp:257`.
    let positive = LineChain::from_slice(&[point(0, 0), point(100, 0)], false);
    let crossing =
      LineChain::from_slice(&[point(50, -50), point(50, 50)], false);
    let parallel =
      LineChain::from_slice(&[point(0, 20), point(100, 20)], false);

    assert!(positive.intersects_chain(&crossing));
    assert!(!positive.intersects_chain(&parallel));
  }

  // ---------------------------------------------------------------
  // Self intersection
  // ---------------------------------------------------------------

  #[test]
  fn self_intersecting_no_intersection_open_chain() {
    // `SelfIntersecting_NoIntersection_OpenChain`,
    // `qa/tests/libs/kimath/geometry/test_shape_line_chain.cpp:1793`.
    let chain = LineChain::from_slice(
      &[
        point(0, 0),
        point(1000, 0),
        point(2000, 1000),
        point(3000, 0),
      ],
      false,
    );

    assert!(chain.self_intersecting().is_none());
  }

  #[test]
  fn self_intersecting_no_intersection_closed_chain() {
    // `SelfIntersecting_NoIntersection_ClosedChain`, `:1801`. A simple
    // closed square is not self intersecting.
    let chain = LineChain::from_slice(
      &[
        point(0, 0),
        point(10000, 0),
        point(10000, 10000),
        point(0, 10000),
      ],
      true,
    );

    assert!(chain.self_intersecting().is_none());
  }

  #[test]
  fn self_intersecting_crossing_segments() {
    // `SelfIntersecting_CrossingSegments`, `:1810`.
    let chain = LineChain::from_slice(
      &[
        point(0, 0),
        point(10000, 10000),
        point(10000, 0),
        point(0, 10000),
      ],
      false,
    );

    let found = chain.self_intersecting().expect("the chain crosses itself");

    assert_eq!(found.ours, Hit::Segment(0));
    assert_eq!(found.theirs, Some(Hit::Segment(2)));
  }

  #[test]
  fn self_intersecting_closed_figure_eight() {
    // `SelfIntersecting_ClosedFigureEight`, `:1823`. The bow tie.
    let chain = LineChain::from_slice(
      &[
        point(0, 0),
        point(10000, 10000),
        point(10000, 0),
        point(0, 10000),
      ],
      true,
    );

    assert!(chain.self_intersecting().is_some());
  }

  #[test]
  fn self_intersecting_vertex_on_segment() {
    // `SelfIntersecting_VertexOnSegment`, `:1833`. A vertex that merely
    // lies on an earlier segment counts, through `SEG::Contains`.
    let chain = LineChain::from_slice(
      &[
        point(0, 0),
        point(20000, 0),
        point(20000, 10000),
        point(10000, 0),
        point(10000, -10000),
      ],
      false,
    );

    let found = chain
      .self_intersecting()
      .expect("the vertex lies on segment 0");

    assert_eq!(found.point, point(10000, 0));
  }

  #[test]
  fn self_intersecting_two_segments() {
    // `SelfIntersecting_TwoSegments`, `:1846`, and
    // `SelfIntersecting_SinglePoint`, `:1854`: fewer than two segments
    // answers immediately.
    let two = LineChain::from_slice(&[point(0, 0), point(10000, 0)], false);
    let one = LineChain::from_slice(&[point(0, 0)], false);

    assert!(two.self_intersecting().is_none());
    assert!(one.self_intersecting().is_none());
  }

  #[test]
  fn self_intersecting_adjacent_segments_ignored() {
    // `SelfIntersecting_AdjacentSegmentsIgnored`, `:1863`.
    let chain = LineChain::from_slice(
      &[
        point(0, 0),
        point(5000, 10000),
        point(10000, 0),
        point(15000, 10000),
        point(20000, 0),
      ],
      false,
    );

    assert!(chain.self_intersecting().is_none());
  }

  #[test]
  fn self_intersecting_closed_triangle() {
    // `SelfIntersecting_ClosedTriangle`, `:1873`.
    let chain = LineChain::from_slice(
      &[point(0, 0), point(10000, 0), point(5000, 10000)],
      true,
    );

    assert!(chain.self_intersecting().is_none());
  }

  #[test]
  fn self_intersecting_closed_last_first_not_false_positive() {
    // `SelfIntersecting_ClosedLastFirstNotFalsePositive`, `:1882`. The
    // closing joint is exempted by index, `!( closed && s1 == 0 && s2 ==
    // segCount - 1 )` at `shape_line_chain.cpp:2189`, and not by any
    // geometric test.
    let chain = LineChain::from_slice(
      &[
        point(0, 0),
        point(10000, 0),
        point(10000, 10000),
        point(0, 10000),
      ],
      true,
    );

    assert!(chain.self_intersecting().is_none());

    // The same points left open still meet nowhere, because the closing
    // segment does not exist at all.
    let mut open = chain.clone();

    open.set_closed(false);
    assert!(open.self_intersecting().is_none());
  }

  #[test]
  fn self_intersecting_spatially_distant() {
    // `SelfIntersecting_SpatiallyDistant`, `:1893`.
    let chain = LineChain::from_slice(
      &[
        point(0, 0),
        point(1000, 0),
        point(1000, 1000000),
        point(2000, 1000000),
        point(2000, 2000000),
        point(3000, 2000000),
      ],
      false,
    );

    assert!(chain.self_intersecting().is_none());
  }

  #[test]
  fn self_intersecting_large_non_intersecting() {
    // `SelfIntersecting_LargeNonIntersecting`, `:1904`.
    let mut chain = LineChain::new();

    for index in 0..200 {
      chain.append(point(index * 1000, (index % 2) * 5000));
    }

    assert!(chain.self_intersecting().is_none());
  }

  #[test]
  fn self_intersecting_large_with_crossing() {
    // `SelfIntersecting_LargeWithCrossing`, `:1916`.
    let mut chain = LineChain::new();

    for index in 0..50 {
      chain.append(point(index * 1000, 0));
    }

    chain.append(point(5000, 10000));
    chain.append(point(5000, -10000));

    assert!(chain.self_intersecting().is_some());
  }

  // ---------------------------------------------------------------
  // Collision
  // ---------------------------------------------------------------

  #[test]
  fn collide_point_is_strict_at_exactly_the_clearance() {
    // The `closest == 0 || closest < clearance` of
    // `shape_line_chain.cpp:468`. This is the comparison the `- 1` at
    // `pcbnew/router/pns_item.cpp:249` is written against.
    let chain = LineChain::from_slice(&[point(0, 0), point(100, 0)], false);
    let above = point(50, 10);

    assert_eq!(chain.collide_point(above, 10), None);
    assert_eq!(
      chain.collide_point(above, 11),
      Some(Collision {
        actual: 10,
        location: point(50, 0)
      })
    );

    // A point on the chain collides at any clearance, zero included.
    assert_eq!(
      chain.collide_point(point(50, 0), 0),
      Some(Collision {
        actual: 0,
        location: point(50, 0)
      })
    );
  }

  #[test]
  fn collide_point_inside_a_closed_chain_reports_the_point_itself() {
    // `shape_line_chain.cpp:429`. Note that the containment test runs
    // with the clearance as its accuracy, so a clearance above 1 also
    // catches a point sitting on the outline.
    let square = closed_square(10);

    assert_eq!(
      square.collide_point(point(5, 5), 0),
      Some(Collision {
        actual: 0,
        location: point(5, 5)
      })
    );

    // On the outline the containment test says no at accuracy 0 and 1,
    // and the segment scan answers instead, with the same distance but a
    // location that is the projection rather than the query point.
    assert_eq!(
      square.collide_point(point(5, 0), 1),
      Some(Collision {
        actual: 0,
        location: point(5, 0)
      })
    );
  }

  #[test]
  fn collide_seg_reproduces_the_kicad_line_to_line_cases() {
    // `Collide_LineToLine`, `Collide_WithClearance` and
    // `Collide_NoClearance`,
    // `qa/tests/libs/kimath/geometry/test_shape_line_chain_collision.cpp:33`,
    // `:86` and `:105`. KiCad drives them through the shape level
    // `SHAPE::Collide( const SHAPE* )`, which is the collision module's
    // job; the second chain of each is a single segment, so the same
    // expectations hold for this member.
    let line = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);

    let crossing = Seg::new(point(5, 5), point(5, -5));

    assert_eq!(
      line.collide_seg(&crossing, 0),
      Some(Collision {
        actual: 0,
        location: point(5, 0)
      })
    );

    let above = Seg::new(point(5, 6), point(-5, 6));

    assert_eq!(
      line.collide_seg(&above, 7),
      Some(Collision {
        actual: 6,
        location: point(0, 0)
      })
    );
    assert_eq!(line.collide_seg(&above, 0), None);
  }

  #[test]
  fn collide_seg_only_tests_the_segments_start_for_containment() {
    // `shape_line_chain.cpp:818` tests `aSeg.A` and nothing else, and at
    // the default accuracy rather than at the clearance. A segment that
    // starts inside takes the shortcut and reports its own start; the
    // same segment reversed does not, and is measured against the edges.
    let square = closed_square(10);

    assert_eq!(
      square.collide_seg(&Seg::new(point(5, 5), point(50, 5)), 0),
      Some(Collision {
        actual: 0,
        location: point(5, 5)
      })
    );
    assert_eq!(
      square.collide_seg(&Seg::new(point(50, 5), point(5, 5)), 0),
      Some(Collision {
        actual: 0,
        location: point(10, 5)
      })
    );
  }

  // ---------------------------------------------------------------
  // Distances and nearest points
  // ---------------------------------------------------------------

  #[test]
  fn distance_into_a_closed_chain_is_zero_unless_outline_only() {
    // `shape_line_chain.cpp:1171`.
    let square = closed_square(10);
    let inside = point(5, 5);

    assert_eq!(square.squared_distance(inside, false), 0);
    assert_eq!(square.distance(inside, false), 0);
    assert_eq!(square.squared_distance(inside, true), 25);
    assert_eq!(square.distance(inside, true), 5);

    // An open chain over the same points has no inside at all.
    let mut open = square.clone();

    open.set_closed(false);
    assert_eq!(open.squared_distance(inside, false), 25);
  }

  #[test]
  fn distance_from_an_empty_chain_is_the_kicad_sentinel() {
    let empty = LineChain::new();

    assert_eq!(empty.squared_distance(point(0, 0), false), i64::MAX);
    assert_eq!(empty.distance(point(0, 0), false), i32::MAX);
  }

  #[test]
  fn nearest_point_uses_the_nearest_segment() {
    let chain =
      LineChain::from_slice(&[point(0, 0), point(10, 0), point(10, 10)], false);

    assert_eq!(chain.nearest_point(point(20, 5)), Some(point(10, 5)));
    assert_eq!(chain.nearest_point(point(-5, -5)), Some(point(0, 0)));

    // A chain of one point answers with that point, an empty one with
    // nothing.
    let single = LineChain::from_slice(&[point(7, 7)], false);

    assert_eq!(single.nearest_point(point(0, 0)), Some(point(7, 7)));
    assert_eq!(LineChain::new().nearest_point(point(0, 0)), None);
  }

  #[test]
  fn nearest_point_to_seg_walks_vertices_and_measures_to_the_line() {
    // Note 01 section 6.8 item 13. The nearest point of this chain to the
    // segment is `(60, 0)` at distance zero, but the routine only looks
    // at vertices and measures to the infinite line, so it answers the
    // vertex `(50, 0)` at distance 10. `PNS::MoveDiagonal`
    // (`pcbnew/router/pns_utils.cpp:293`) depends on that.
    let chain =
      LineChain::from_slice(&[point(0, 0), point(50, 0), point(100, 0)], false);
    let vertical = Seg::new(point(60, -10), point(60, 10));

    assert_eq!(
      chain.nearest_point_to_seg(&vertical),
      Some((point(50, 0), 10))
    );
    assert_eq!(LineChain::new().nearest_point_to_seg(&vertical), None);
  }

  #[test]
  fn nearest_segment_breaks_ties_towards_the_earlier_segment() {
    let chain =
      LineChain::from_slice(&[point(0, 0), point(10, 0), point(10, 10)], false);

    assert_eq!(chain.nearest_segment(point(20, 5)), Some(1));
    assert_eq!(chain.nearest_segment(point(5, -5)), Some(0));
    // Equidistant from both segments, so the first one wins.
    assert_eq!(chain.nearest_segment(point(10, 0)), Some(0));
    assert_eq!(LineChain::new().nearest_segment(point(0, 0)), None);
  }

  // ---------------------------------------------------------------
  // Containment
  // ---------------------------------------------------------------

  #[test]
  fn point_in_polygon() {
    // `PointInPolygon`,
    // `qa/tests/libs/kimath/geometry/test_shape_line_chain.cpp:349`.
    let mut outline1 = LineChain::from_slice(
      &[
        point(1316455, 913576),
        point(1316455, 901129),
        point(1321102, 901129),
        point(1322152, 901191),
        point(1323055, 901365),
        point(1323830, 901639),
        point(1324543, 902036),
        point(1325121, 902521),
        point(1325581, 903100),
        point(1325914, 903759),
        point(1326120, 904516),
        point(1326193, 905390),
        point(1326121, 906253),
        point(1325915, 907005),
        point(1325581, 907667),
        point(1325121, 908248),
        point(1324543, 908735),
        point(1323830, 909132),
        point(1323055, 909406),
        point(1322153, 909579),
        point(1321102, 909641),
        point(1317174, 909641),
        point(1317757, 909027),
        point(1317757, 913576),
      ],
      false,
    );
    let mut outline2 = LineChain::from_slice(
      &[
        point(1297076, 916244),
        point(1284629, 916244),
        point(1284629, 911597),
        point(1284691, 910547),
        point(1284865, 909644),
        point(1285139, 908869),
        point(1285536, 908156),
        point(1286021, 907578),
        point(1286600, 907118),
        point(1287259, 906785),
        point(1288016, 906579),
        point(1288890, 906506),
        point(1289753, 906578),
        point(1290505, 906784),
        point(1291167, 907118),
        point(1291748, 907578),
        point(1292235, 908156),
        point(1292632, 908869),
        point(1292906, 909644),
        point(1293079, 910546),
        point(1293141, 911597),
        point(1293141, 915525),
        point(1292527, 914942),
        point(1297076, 914942),
      ],
      false,
    );

    outline1.set_closed(true);
    outline2.set_closed(true);

    assert!(outline1.point_inside(point(1317757, 909133), 0));
    assert!(outline2.point_inside(point(1292633, 914942), 0));
  }

  #[test]
  fn point_inside_needs_a_closed_chain_of_at_least_three_points() {
    // `shape_line_chain.cpp:1994`.
    let mut square = closed_square(10);

    assert!(square.point_inside(point(5, 5), 0));
    square.set_closed(false);
    assert!(!square.point_inside(point(5, 5), 0));

    let two = LineChain::from_slice(&[point(0, 0), point(10, 0)], true);

    assert!(!two.point_inside(point(5, 0), 0));
  }

  #[test]
  fn point_inside_puts_the_edge_and_the_vertices_outside() {
    // The strict `aPt.x - p1.x < d` at `shape_line_chain.cpp:2011` leaves
    // a point on a non horizontal edge outside, and the half open
    // `>=` y rule counts a vertex on the ray once. Only from an accuracy
    // of 2 does `PointOnEdge` stand in for "inside" (`:2016`).
    let square = closed_square(10);

    for on_the_outline in
      [point(5, 0), point(10, 5), point(0, 0), point(10, 10)]
    {
      assert!(!square.point_inside(on_the_outline, 0));
      assert!(!square.point_inside(on_the_outline, 1));
      assert!(square.point_inside(on_the_outline, 2));
    }
  }

  #[test]
  fn point_inside_a_concave_polygon() {
    // A U shape with the notch at x in (10, 20), y above 10.
    let outline = LineChain::from_slice(
      &[
        point(0, 0),
        point(30, 0),
        point(30, 30),
        point(20, 30),
        point(20, 10),
        point(10, 10),
        point(10, 30),
        point(0, 30),
      ],
      true,
    );

    assert!(outline.point_inside(point(15, 5), 0));
    assert!(outline.point_inside(point(5, 20), 0));
    assert!(outline.point_inside(point(25, 20), 0));
    // In the notch, so outside despite being within the bounding box.
    assert!(!outline.point_inside(point(15, 20), 0));
    assert!(!outline.point_inside(point(40, 15), 0));
  }

  #[test]
  fn edge_containing_point_uses_an_accuracy_plus_one_band() {
    // `shape_line_chain.cpp:2081`, one of the constants note 01 section
    // 13 lists.
    let chain = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);

    assert_eq!(chain.edge_containing_point(point(5, 1), 0), Some(0));
    assert_eq!(chain.edge_containing_point(point(5, 2), 0), None);
    assert_eq!(chain.edge_containing_point(point(5, 2), 1), Some(0));
    assert!(chain.point_on_edge(point(5, 1), 0));
    assert!(!chain.point_on_edge(point(5, 2), 0));

    // A chain of one point is measured against that point (`:2092`), and
    // an empty one contains nothing.
    let single = LineChain::from_slice(&[point(0, 0)], false);

    assert_eq!(single.edge_containing_point(point(1, 0), 0), Some(0));
    assert_eq!(single.edge_containing_point(point(2, 0), 0), None);
    assert_eq!(
      LineChain::new().edge_containing_point(point(0, 0), 100),
      None
    );
  }

  #[test]
  fn check_clearance_compares_the_unsquared_distance() {
    // `shape_line_chain.cpp:2127` compares `s.Distance( aP ) <= aDist`,
    // so unlike `edge_containing_point` there is no `+ 1` band, and a
    // single point chain wants an exact match (`:2118`).
    let chain = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);

    assert!(!chain.check_clearance(point(5, 1), 0));
    assert!(chain.check_clearance(point(5, 1), 1));
    assert!(chain.check_clearance(point(0, 0), 0));

    let single = LineChain::from_slice(&[point(0, 0)], false);

    assert!(single.check_clearance(point(0, 0), 100));
    assert!(!single.check_clearance(point(1, 0), 100));
    assert!(!LineChain::new().check_clearance(point(0, 0), 100));
  }

  // ---------------------------------------------------------------
  // Area and the three way split
  // ---------------------------------------------------------------

  #[test]
  fn area_is_positive_for_a_clockwise_chain_in_screen_coordinates() {
    // `shape_line_chain.cpp:2718` negates the shoelace sum, so a chain
    // that runs clockwise on a screen where y grows downwards comes out
    // positive.
    let clockwise = closed_square(10);
    let counter_clockwise = clockwise.reversed();

    assert_eq!(clockwise.area(false), 100.0);
    assert_eq!(counter_clockwise.area(false), -100.0);
    assert_eq!(clockwise.area(true), 100.0);
    assert_eq!(counter_clockwise.area(true), 100.0);

    // An open chain encloses nothing whatever its shape (`:2700`).
    let mut open = clockwise.clone();

    open.set_closed(false);
    assert_eq!(open.area(false), 0.0);
    assert_eq!(LineChain::new().area(true), 0.0);
  }

  #[test]
  fn split_exact_leaves_an_existing_vertex_alone() {
    // The only difference from `split`: the short circuit at
    // `shape_line_chain.cpp:1188`.
    let base =
      LineChain::from_slice(&[point(0, 0), point(50, 0), point(100, 0)], false);

    let mut exact = base.clone();

    assert_eq!(exact.split_exact(point(50, 0)), Some(1));
    assert_eq!(exact.point_count(), 3);

    let mut inexact = base.clone();

    assert_eq!(inexact.split_exact(point(60, 0)), Some(2));
    assert_eq!(
      points_of(&inexact),
      vec![point(0, 0), point(50, 0), point(60, 0), point(100, 0)]
    );
  }

  #[test]
  fn split_three_way_cuts_out_the_middle() {
    // `shape_line_chain.cpp:2877`, the form the meander placers use.
    let chain = LineChain::from_slice(&[point(0, 0), point(100, 0)], false);

    let (pre, mid, post) = chain
      .split_three_way(point(20, 0), point(60, 0))
      .expect("both points snap onto the chain");

    assert_eq!(points_of(&pre), vec![point(0, 0), point(20, 0)]);
    assert_eq!(points_of(&mid), vec![point(20, 0), point(60, 0)]);
    assert_eq!(points_of(&post), vec![point(60, 0), point(100, 0)]);
    assert!(!mid.is_closed());
    assert_eq!(mid.width(), 0);
  }

  #[test]
  fn split_three_way_reverses_when_the_points_arrive_backwards() {
    // `shape_line_chain.cpp:2891` reverses the working copy rather than
    // returning an empty middle, so `pre` and `post` swap ends.
    let chain = LineChain::from_slice(&[point(0, 0), point(100, 0)], false);

    let (pre, mid, post) = chain
      .split_three_way(point(60, 0), point(20, 0))
      .expect("both points snap onto the chain");

    assert_eq!(points_of(&pre), vec![point(100, 0), point(60, 0)]);
    assert_eq!(points_of(&mid), vec![point(60, 0), point(20, 0)]);
    assert_eq!(points_of(&post), vec![point(20, 0), point(0, 0)]);
  }

  #[test]
  fn split_three_way_snaps_points_that_are_off_the_chain() {
    let chain = LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(100, 100)],
      false,
    );

    let (pre, mid, post) = chain
      .split_three_way(point(20, 40), point(140, 50))
      .expect("both points snap onto the chain");

    assert_eq!(points_of(&pre), vec![point(0, 0), point(20, 0)]);
    assert_eq!(
      points_of(&mid),
      vec![point(20, 0), point(100, 0), point(100, 50)]
    );
    assert_eq!(points_of(&post), vec![point(100, 50), point(100, 100)]);

    assert_eq!(
      LineChain::new().split_three_way(point(0, 0), point(1, 0)),
      None
    );
  }

  // ---------------------------------------------------------------
  // remove_duplicate_points
  // ---------------------------------------------------------------

  /// Runs of equal vertices collapse, colinear vertices stay.
  #[test]
  fn remove_duplicate_points_keeps_colinear_vertices() {
    let mut chain = LineChain::from_points(
      vec![
        point(0, 0),
        point(0, 0),
        point(5, 0),
        point(10, 0),
        point(10, 0),
        point(10, 0),
        point(10, 5),
      ],
      false,
    );

    chain.remove_duplicate_points();

    assert_eq!(
      chain.points(),
      &[point(0, 0), point(5, 0), point(10, 0), point(10, 5)]
    );
  }

  /// Three points: only a duplicate of the first is removed, a duplicate
  /// of the last is not, and shorter chains are untouched.
  #[test]
  fn remove_duplicate_points_special_cases() {
    let mut first_doubled = LineChain::from_points(
      vec![point(0, 0), point(0, 0), point(9, 0)],
      false,
    );
    first_doubled.remove_duplicate_points();
    assert_eq!(first_doubled.points(), &[point(0, 0), point(9, 0)]);

    let mut last_doubled = LineChain::from_points(
      vec![point(0, 0), point(9, 0), point(9, 0)],
      false,
    );
    last_doubled.remove_duplicate_points();
    assert_eq!(
      last_doubled.points(),
      &[point(0, 0), point(9, 0), point(9, 0)]
    );

    let mut two = LineChain::from_points(vec![point(3, 3), point(3, 3)], false);
    two.remove_duplicate_points();
    assert_eq!(two.points(), &[point(3, 3), point(3, 3)]);
  }

  /// The closed flag is not consulted: first and last stay even when
  /// equal, unlike `set_closed`.
  #[test]
  fn remove_duplicate_points_ignores_the_closed_flag() {
    let mut chain = LineChain::new();
    chain.append(point(0, 0));
    chain.append(point(4, 0));
    chain.append(point(4, 4));
    chain.append(point(0, 4));
    chain.append_allow_duplicate(point(0, 4));
    chain.append(point(0, 0));
    chain.set_closed(false);

    chain.remove_duplicate_points();

    assert_eq!(
      chain.points(),
      &[
        point(0, 0),
        point(4, 0),
        point(4, 4),
        point(0, 4),
        point(0, 0)
      ]
    );
  }
  // ---------------------------------------------------------------
  // POINT_INSIDE_TRACKER
  // ---------------------------------------------------------------

  #[test]
  fn a_tracker_fed_one_square_answers_like_point_inside() {
    // Three sides plus the closing edge the tracker adds itself.
    let ring = LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(100, 100), point(0, 100)],
      false,
    );

    let mut inside = PointInsideTracker::new(point(50, 50));

    inside.add_polyline(&ring);
    assert!(inside.is_inside());

    let mut outside = PointInsideTracker::new(point(150, 50));

    outside.add_polyline(&ring);
    assert!(!outside.is_inside());
  }

  #[test]
  fn a_ring_assembled_from_two_open_chains_closes_on_its_own() {
    // The shove's shape: one chain out and one chain back, meeting at
    // both ends only through the closing edge.
    let there = LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(100, 100)],
      false,
    );
    let back = LineChain::from_slice(&[point(100, 100), point(0, 100)], false);

    let mut tracker = PointInsideTracker::new(point(50, 50));

    tracker.add_polyline(&there);
    tracker.add_polyline(&back);

    assert!(tracker.is_inside());
  }

  #[test]
  fn a_point_on_the_boundary_reads_as_outside() {
    let ring = LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(100, 100), point(0, 100)],
      false,
    );

    // On an edge: the cross product is zero, which sets the parity to -1.
    let mut on_edge = PointInsideTracker::new(point(50, 0));

    on_edge.add_polyline(&ring);
    assert!(!on_edge.is_inside());

    // On a vertex: the ray runs through the far end of an edge.
    let mut on_vertex = PointInsideTracker::new(point(100, 100));

    on_vertex.add_polyline(&ring);
    assert!(!on_vertex.is_inside());
  }

  #[test]
  fn an_empty_polyline_is_skipped() {
    let mut tracker = PointInsideTracker::new(point(50, 50));

    tracker.add_polyline(&LineChain::new());
    tracker.add_polyline(&LineChain::from_slice(
      &[point(0, 0), point(100, 0), point(100, 100), point(0, 100)],
      false,
    ));

    assert!(tracker.is_inside());
  }
}
