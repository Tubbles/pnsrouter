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
//! - A chain carries arcs. [`ArcRef`] replaces KiCad's
//!   `(ssize_t, ssize_t)` pair of arc indices with its `-1` sentinel
//!   (`slc.h:989`), and it carries the vertex's [`PointRole`] so that
//!   [`LineChain::is_arc_start`], [`LineChain::is_arc_end`] and
//!   [`LineChain::is_pt_on_arc`] are field reads rather than point
//!   comparisons against an arc that `amend_arc` and [`LineChain::slice`]
//!   can rebuild. Note 09 section 11.1.
//!
//! The arc model, and the four places it departs from KiCad's, all from
//! `doc/reference/kicad/09-arcs.md` section 11.1.
//!
//! - `shapes` is parallel to `points` and `arcs` holds the arcs in **chain
//!   order**. KiCad relies on that order in `Reverse` (`slc.cpp:926`) and
//!   breaks it in `Replace( int, int, const SHAPE_LINE_CHAIN& )`
//!   (`:1071`, erratum E7); [`LineChain::replace_with_chain`] splices the
//!   incoming arcs into position instead, and the private
//!   `check_invariants` asserts the order after every mutator in a debug
//!   build.
//! - The arcs are only reachable through [`LineChain::live_arcs`], which
//!   walks the shape entries, so an arc that lost its last reference
//!   cannot collide and cannot be drawn. KiCad iterates `ArcCount()`
//!   directly in `Collide` (`:480`, `:873`) and in the preview, which is
//!   erratum E13.
//! - [`LineChain::arc`] and [`LineChain::arc_index`] return [`Option`]
//!   where KiCad's are unchecked (`slc.h:856`, `:864`, erratum E14).
//! - The role in [`ArcRef::On`] removes the geometric point comparison
//!   from the predicates, which is what makes erratum E35's unconditional
//!   wrap in `IsArcEnd( 0 )` (`slc.cpp:3286`) moot.
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
//! - An arc only path with no router caller, so note 09 section 12 leaves
//!   it out of the milestone: `SelfIntersectingWithArcs` (`slc.cpp:2234`,
//!   erratum E16) and `reversedArcIndex` (`slc.h:941`).
//! - Dead or deliberately dropped: `m_accuracy` (`slc.h:994`), which
//!   every constructor sets to zero and nothing reads, and the bounding
//!   box cache with `GenerateBBoxCache` (`slc.h:468`), for the reason
//!   given above.
//! - Debug serialisation, which the crate will grow its own form of:
//!   `Format` and `Parse` (`slc.cpp:2506`, `:2623`), which note 01
//!   section 6.8 item 16 shows are not round trip compatible anyway.

use std::fmt;

use crate::geometry::arc::ShapeArc;
use crate::geometry::box2::Box2;
use crate::geometry::math::{isqrt, rescale};
use crate::geometry::seg::{Seg, distance_from_squared};
use crate::geometry::vec2::{Vec2, Vec2L};

/// Where a vertex sits within the arc it belongs to.
///
/// Change 1 of `doc/reference/kicad/09-arcs.md` section 11.1. KiCad has no
/// counterpart: it answers "is this the arc's first point" by comparing
/// `arc.GetP0()` with the stored point
/// (`libs/kimath/src/geometry/shape_line_chain.cpp:3278`) and "is it the
/// last" by the same against `GetP1()` (`:3299`). Those comparisons are
/// against a value `amendArc` (`:275`) and `Slice`'s re-cut (`:1456`) can
/// move, and they are why `IsArcEnd( 0 )` has to wrap unconditionally
/// (erratum E35). Carrying the role instead makes the three predicates
/// field reads.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum PointRole {
  /// The first vertex of the arc's approximation.
  Start,
  /// A vertex strictly between the arc's two endpoints.
  Interior,
  /// The last vertex of the arc's approximation.
  End,
}

/// What one vertex of a chain belongs to.
///
/// Replaces the `std::pair<ssize_t, ssize_t>` entries of `m_shapes`,
/// `libs/kimath/include/geometry/shape_line_chain.h:989`, with their
/// `SHAPE_IS_PT == -1` sentinel (`:968`). KiCad documents the pair's
/// invariant in prose, "the second element must always be `SHAPE_IS_PT` if
/// the first element is `SHAPE_IS_PT`" (`:987`), and `convertArc` has to
/// re-establish it by hand (`shape_line_chain.cpp:267`). Here it is
/// unrepresentable.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub enum ArcRef {
  /// An ordinary polyline vertex, KiCad's `SHAPES_ARE_PT` (`slc.h:970`).
  #[default]
  Plain,
  /// A vertex of exactly one arc's approximation.
  ///
  /// KiCad's `{ N, SHAPE_IS_PT }`.
  On {
    /// The index into the chain's arcs.
    arc: usize,
    /// Where the vertex sits in that arc.
    role: PointRole,
  },
  /// A vertex that ends one arc and starts the next.
  ///
  /// KiCad's `{ N, N + 1 }`, the "shared point" of `slc.h:975`.
  Shared {
    /// The arc that ends at this vertex, KiCad's `.first`.
    ends: usize,
    /// The arc that starts at this vertex, KiCad's `.second`.
    starts: usize,
  },
}

impl ArcRef {
  /// The arc the segment leaving this vertex lies on.
  ///
  /// `None` at a plain vertex and at an arc's last point, where nothing
  /// continues. KiCad has no such accessor: `ArcIndex` (`slc.h:856`)
  /// answers with the arc **ending** here instead, which is erratum E11's
  /// whole class of bug.
  const fn leaving_arc(self) -> Option<usize> {
    match self {
      ArcRef::Plain
      | ArcRef::On {
        role: PointRole::End,
        ..
      } => None,
      ArcRef::On { arc, .. } => Some(arc),
      ArcRef::Shared { starts, .. } => Some(starts),
    }
  }

  /// The arc the segment arriving at this vertex lies on.
  ///
  /// Port of reading `m_shapes[i].first` directly, which is what
  /// `IsArcSegment` compares against
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:3263`).
  const fn entering_arc(self) -> Option<usize> {
    match self {
      ArcRef::Plain => None,
      ArcRef::On { arc, .. } => Some(arc),
      ArcRef::Shared { ends, .. } => Some(ends),
    }
  }

  /// KiCad's `ArcIndex`, `libs/kimath/include/geometry/shape_line_chain.h:856`:
  /// the second half of a shared entry, the first half otherwise.
  const fn arc_index(self) -> Option<usize> {
    match self {
      ArcRef::Plain => None,
      ArcRef::On { arc, .. } => Some(arc),
      ArcRef::Shared { starts, .. } => Some(starts),
    }
  }

  /// Whether this vertex is the first point of an arc.
  const fn starts_an_arc(self) -> bool {
    matches!(
      self,
      ArcRef::Shared { .. }
        | ArcRef::On {
          role: PointRole::Start,
          ..
        }
    )
  }

  /// Whether this vertex is the last point of an arc.
  const fn ends_an_arc(self) -> bool {
    matches!(
      self,
      ArcRef::Shared { .. }
        | ArcRef::On {
          role: PointRole::End,
          ..
        }
    )
  }

  /// The entry with every arc index at or above `first` shifted up by
  /// `count`.
  ///
  /// Port of the renumbering loops KiCad writes inline with
  /// `alg::run_on_pair`, for instance
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1690`.
  const fn shifted_up_by(self, first: usize, count: usize) -> Self {
    const fn shift(index: usize, first: usize, count: usize) -> usize {
      if index >= first { index + count } else { index }
    }

    match self {
      ArcRef::Plain => ArcRef::Plain,
      ArcRef::On { arc, role } => ArcRef::On {
        arc: shift(arc, first, count),
        role,
      },
      ArcRef::Shared { ends, starts } => ArcRef::Shared {
        ends: shift(ends, first, count),
        starts: shift(starts, first, count),
      },
    }
  }

  /// The entry with every arc index shifted up by `offset`.
  ///
  /// Port of `fixShapeIndices` in `Append( const SHAPE_LINE_CHAIN& )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1558`.
  const fn offset_by(self, offset: usize) -> Self {
    match self {
      ArcRef::Plain => ArcRef::Plain,
      ArcRef::On { arc, role } => ArcRef::On {
        arc: arc + offset,
        role,
      },
      ArcRef::Shared { ends, starts } => ArcRef::Shared {
        ends: ends + offset,
        starts: starts + offset,
      },
    }
  }
}

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
/// Equality compares the points, the shape entries, the arcs, the closed
/// flag and the width. KiCad has no `operator==` at all and its
/// `operator!=` ignores arcs, closedness and width (`slc.h:749`, note 01
/// section 6.8 item 15); nothing in the crate transcribes that comparison.
/// Two chains over the same points with no arcs compare exactly as they
/// did before arcs existed, because every shape entry is then
/// [`ArcRef::Plain`] and both arc vectors are empty.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct LineChain {
  /// The vertices, in chain order.
  ///
  /// Port of `m_points`, `libs/kimath/include/geometry/shape_line_chain.h:973`.
  points: Vec<Vec2>,
  /// What each vertex belongs to, parallel to `points`.
  ///
  /// Port of `m_shapes`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:989`. The lengths
  /// are equal at every observable point, which KiCad asserts in a dozen
  /// places and this module keeps by mutating the two together.
  shapes: Vec<ArcRef>,
  /// The arcs, in chain order.
  ///
  /// Port of `m_arcs`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:991`. Chain order is
  /// an invariant here rather than an accident; see the module
  /// documentation and erratum E7.
  arcs: Vec<ShapeArc>,
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

  /// The accuracy an arc is approximated at when a chain stores it, in
  /// nanometres.
  ///
  /// Port of `getArcPolygonizationMaxError`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:57`, which is
  /// `SHAPE_ARC::DefaultAccuracyForPCB() / 5`. It is the default
  /// [`LineChain::append_arc`] and [`LineChain::slice`] use, matching
  /// KiCad's `Append( SHAPE_ARC )` (`:1614`) and two argument `Slice`
  /// (`:1414`).
  pub const ARC_POLYGONIZATION_MAX_ERROR: i32 =
    ShapeArc::DEFAULT_ACCURACY_FOR_PCB / 5;

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
      shapes: Vec::new(),
      arcs: Vec::new(),
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
    let shapes = vec![ArcRef::Plain; points.len()];
    let mut chain = Self {
      points,
      shapes,
      arcs: Vec::new(),
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
  // Arcs
  // ---------------------------------------------------------------

  /// The number of arcs the chain stores.
  ///
  /// Port of `ArcCount`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:846`. This counts
  /// the storage, so it can exceed the number of arcs any vertex still
  /// refers to; [`LineChain::live_arcs`] is the one that cannot.
  pub fn arc_count(&self) -> usize {
    self.arcs.len()
  }

  /// One stored arc.
  ///
  /// Port of `Arc`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:864`, which is
  /// unchecked and is the second half of erratum E14. Change 4 of note 09
  /// section 11.1 asks for the [`Option`].
  pub fn arc(&self, index: usize) -> Option<ShapeArc> {
    self.arcs.get(index).copied()
  }

  /// The arc the shape leaving a vertex belongs to.
  ///
  /// Port of `ArcIndex`,
  /// `libs/kimath/include/geometry/shape_line_chain.h:856`: the second
  /// half of a shared entry, the first half otherwise. KiCad's is
  /// unchecked, which is the first half of erratum E14.
  ///
  /// Note what this answers at an arc's last point: the arc that **ends**
  /// there, even when the segment leaving it is straight. That is the
  /// reading erratum E11 catches the placer relying on; ask
  /// [`LineChain::is_arc_segment`] instead when the question is "does the
  /// shape leaving this vertex curve".
  pub fn arc_index(&self, index: usize) -> Option<usize> {
    self.shapes.get(index).copied().and_then(ArcRef::arc_index)
  }

  /// Whether a vertex belongs to any arc.
  ///
  /// Port of `IsPtOnArc`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:3240`. True for an
  /// arc's last point even when the segment leaving that point is
  /// straight, and bound checked, both as KiCad's is.
  pub fn is_pt_on_arc(&self, index: usize) -> bool {
    self
      .shapes
      .get(index)
      .is_some_and(|entry| *entry != ArcRef::Plain)
  }

  /// Whether a vertex ends one arc and starts the next.
  ///
  /// Port of `IsSharedPt`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:3232`.
  pub fn is_shared_pt(&self, index: usize) -> bool {
    self
      .shapes
      .get(index)
      .is_some_and(|entry| matches!(entry, ArcRef::Shared { .. }))
  }

  /// Whether the segment from a vertex to the next one lies on an arc.
  ///
  /// Port of `IsArcSegment`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:3246`. Two adjacent
  /// arcs that do not share a vertex have a genuine straight segment
  /// between them, which is why the question cannot be answered by
  /// [`LineChain::is_pt_on_arc`] alone (the comment at `:3248`).
  ///
  /// KiCad compares `ArcIndex( s )` with `m_shapes[s + 1].first`. This
  /// compares the arc leaving this vertex with the arc arriving at the
  /// next one, which is the same answer without the sentinel. Note 09
  /// section 11.1 writes the predicate as a bare equality of those two,
  /// which by itself reports an arc's last point followed by a plain
  /// point as an arc segment, both sides being absent; the comparison has
  /// to require an arc on both sides.
  ///
  /// The wrap onto index 0 for the closing segment of a closed chain is
  /// kept with KiCad's guard (`:3257`), because the role decides what the
  /// next vertex holds but not which vertex is next.
  pub fn is_arc_segment(&self, segment: usize) -> bool {
    let Some(entry) = self.shapes.get(segment) else {
      return false;
    };
    let next = segment + 1;
    let next = if next < self.shapes.len() {
      next
    } else if next == self.shapes.len() && self.closed && self.is_shared_pt(0) {
      0
    } else {
      return false;
    };

    match entry.leaving_arc() {
      None => false,
      Some(arc) => self.shapes[next].entering_arc() == Some(arc),
    }
  }

  /// Whether a vertex is the first point of an arc.
  ///
  /// Port of `IsArcStart`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:3268`, which is
  /// `IsArcSegment` (and so bound checked through it), then shared, then
  /// `arc.GetP0() == m_points[i]`. The last test is the role here.
  pub fn is_arc_start(&self, index: usize) -> bool {
    self.is_arc_segment(index) && self.shapes[index].starts_an_arc()
  }

  /// Whether a vertex is the last point of an arc.
  ///
  /// Port of `IsArcEnd`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:3282`. The look back
  /// from index 0 to the last point is unconditional in KiCad, open chain
  /// or not, which is erratum E35. It is reproduced because it cannot
  /// change an answer: `is_arc_segment` of the last point of an open chain
  /// is always false, there being no segment there.
  pub fn is_arc_end(&self, index: usize) -> bool {
    let previous = if index == 0 {
      match self.points.len().checked_sub(1) {
        Some(last) => last,
        None => return false,
      }
    } else if index > self.points.len() - 1 {
      return false;
    } else {
      index - 1
    };

    self.is_arc_segment(previous) && self.shapes[index].ends_an_arc()
  }

  /// Every arc a vertex still refers to, once each, in chain order.
  ///
  /// Change 3 of note 09 section 11.1, and the answer to erratum E13:
  /// neither `Simplify` nor `Simplify2` nor `RemoveDuplicatePoints` ever
  /// erases from KiCad's `m_arcs`, so an arc that lost every reference
  /// still collides (`shape_line_chain.cpp:480`, `:873`) and still draws
  /// (`pcbnew/router/router_preview_item.cpp:278`). Everything in this
  /// module that consumes arcs goes through here instead.
  pub fn live_arcs(&self) -> impl Iterator<Item = (usize, &ShapeArc)> {
    let mut seen: Option<usize> = None;

    self.shapes.iter().filter_map(move |entry| {
      let index = match entry {
        ArcRef::Plain => return None,
        ArcRef::On { arc, .. } => *arc,
        ArcRef::Shared { ends, starts } => {
          // The arc ending here was opened by an earlier vertex, so only
          // the one starting here can be new.
          let _ = ends;
          *starts
        }
      };

      if seen == Some(index) {
        return None;
      }

      seen = Some(index);
      self.arcs.get(index).map(|arc| (index, arc))
    })
  }

  /// The first vertex of the shape after the one starting at a vertex.
  ///
  /// Port of `NextShape`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1302`. A shape is one
  /// straight segment or one whole arc. `None` is KiCad's `-1`, "no shape
  /// follows"; the walk never wraps past the last point except for the
  /// hidden closing segment of a closed chain (`:1350`).
  ///
  /// KiCad accepts a negative index and adds `PointCount()` to it once;
  /// transcribe that with [`LineChain::normalize_index`].
  pub fn next_shape(&self, index: usize) -> Option<usize> {
    let last_index = self.points.len().checked_sub(1)?;

    // :1305
    if index >= last_index {
      return None;
    }

    let mut walk = index;

    // :1315
    if self.shapes[walk] == ArcRef::Plain {
      if walk == last_index - 1 {
        return if self.closed { Some(last_index) } else { None };
      }

      return Some(walk + 1);
    }

    let arc_start = walk;
    let current = self.shapes[walk].arc_index()?;

    // :1338, skip the rest of the arc
    while walk < last_index && self.arc_index(walk) == Some(current) {
      walk += 1;
    }

    let still_on_arc = match self.shapes[walk] {
      ArcRef::Plain => false,
      ArcRef::On { arc, .. } => arc == current,
      ArcRef::Shared { ends, starts } => ends == current || starts == current,
    };

    // :1345, we want the last vertex of the arc if we started at its first
    if walk - arc_start > 1 && !still_on_arc {
      walk -= 1;
    }

    // :1350
    if walk == last_index {
      if !self.closed || self.is_arc_segment(walk) {
        return None;
      }

      return Some(last_index);
    }

    Some(walk)
  }

  /// The number of shapes, counting a whole arc as one.
  ///
  /// Port of `ShapeCount`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1269`, a walk of
  /// [`LineChain::next_shape`] from vertex zero. A chain of fewer than two
  /// points has no shapes.
  pub fn shape_count(&self) -> usize {
    if self.points.len() < 2 {
      return 0;
    }

    let mut count = 1;
    let mut index = self.next_shape(0);

    while let Some(current) = index {
      count += 1;
      index = self.next_shape(current);
    }

    count
  }

  /// Remove the whole shape a vertex belongs to.
  ///
  /// Port of `RemoveShape`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1380`. On a plain
  /// vertex this is [`LineChain::remove`]; on a vertex of an arc it walks
  /// back to the arc's start and forward through
  /// [`LineChain::next_shape`], so the whole arc goes. That is why the
  /// placer's pullback, `tail.RemoveShape( -1 )`
  /// (`pcbnew/router/pns_line_placer.cpp:244`), drops a whole arc rather
  /// than one approximation segment.
  ///
  /// An index that is not a vertex of the chain is a silent no operation,
  /// as it is in KiCad (`:1385`). Transcribe KiCad's negative indices with
  /// [`LineChain::normalize_index`].
  pub fn remove_shape(&mut self, index: usize) {
    if index >= self.points.len() {
      return;
    }

    if self.shapes[index] == ArcRef::Plain {
      self.remove(index);
      return;
    }

    let mut start = index;
    let mut end = index;
    let Some(arc) = self.arc_index(index) else {
      return;
    };

    // :1398
    if !self.is_arc_start(start) {
      while start > 0 && self.arc_index(start - 1) == Some(arc) {
        start -= 1;
      }
    }

    // :1404
    if !self.is_arc_end(end) || start == end {
      end = match self.next_shape(end) {
        Some(next) => next,
        // KiCad's `-1` here becomes the last point through `Remove`'s
        // negative index normalisation (`:1086`).
        None => self.points.len() - 1,
      };
    }

    self.remove_range(start, end);
  }

  /// Degrade every arc to its polyline.
  ///
  /// Port of `ClearArcs`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:949`, which is
  /// `convertArc` back to front, the one order that needs no renumbering.
  /// The points stay exactly where they are; only the arc references and
  /// the arcs go. No caller in `pcbnew/router/`.
  pub fn clear_arcs(&mut self) {
    for index in (0..self.arcs.len()).rev() {
      self.convert_arc(index);
    }

    self.check_invariants();
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
    self.shapes.clear();
    self.arcs.clear();
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
      self.shapes.push(ArcRef::Plain);
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
    self.shapes.push(ArcRef::Plain);
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
  ///
  /// The other chain's arcs are taken over and its arc indices shifted by
  /// the current arc count (`:1558`). The special case at `:1579` matters:
  /// when the joining point is dropped as a duplicate and the other
  /// chain's first segment is an arc segment, the arc reference is grafted
  /// onto the surviving vertex, which is how two arcs come to share one
  /// point.
  pub fn append_chain(&mut self, other: &LineChain) {
    if other.points.is_empty() {
      return;
    }

    // :1556
    let arc_offset = self.arcs.len();

    self.arcs.extend_from_slice(&other.arcs);

    // :1571
    if self.points.is_empty() || self.points.last() != Some(&other.points[0]) {
      self.points.push(other.points[0]);
      self.shapes.push(other.shapes[0].offset_by(arc_offset));
    } else if other.is_arc_segment(0) {
      // :1579, associate the incoming arc with our existing last point.
      let Some(incoming) = other.shapes[0].entering_arc() else {
        unreachable!("an arc segment has an entering arc");
      };
      let incoming = incoming + arc_offset;
      let last = self.shapes.len() - 1;

      self.shapes[last] = match self.shapes[last] {
        ArcRef::Plain => ArcRef::On {
          arc: incoming,
          role: PointRole::Start,
        },
        ArcRef::On { arc, .. } => ArcRef::Shared {
          ends: arc,
          starts: incoming,
        },
        // KiCad writes `m_shapes.back().second` unconditionally here, so a
        // last point that was already shared silently loses the arc that
        // started at it. That cannot happen: a shared last point means an
        // arc starts here and has no further vertices.
        ArcRef::Shared { ends, .. } => ArcRef::Shared {
          ends,
          starts: incoming,
        },
      };
    }

    // :1588
    for index in 1..other.points.len() {
      self.points.push(other.points[index]);
      self.shapes.push(other.shapes[index].offset_by(arc_offset));
    }

    self.merge_first_last_point_if_needed();
    self.check_invariants();
  }

  /// Append an arc, approximated to a given accuracy.
  ///
  /// Port of `Append( const SHAPE_ARC&, int aMaxError )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1618`. The arc is
  /// polygonised, and the result is only **tagged** as an arc when it has
  /// more than two points (`:1620`): a degenerate or nearly flat arc
  /// silently becomes a plain segment with no entry in the chain's arcs.
  /// The stored copy always has width zero (`:1624`), which the collision
  /// members assert.
  ///
  /// The points then go through [`LineChain::append_chain`], so the
  /// duplicate suppression of `Append` applies to the arc's first point
  /// and the shared point graft at `:1579` applies when this chain already
  /// ends there.
  ///
  /// Pass [`LineChain::ARC_POLYGONIZATION_MAX_ERROR`] for KiCad's one
  /// argument `Append( const SHAPE_ARC& )` (`:1612`).
  pub fn append_arc(&mut self, arc: &ShapeArc, max_error: i32) {
    let mut chain = arc.convert_to_polyline(max_error);

    // :1620
    if chain.points.len() > 2 {
      let mut stored = *arc;

      stored.set_width(0);
      chain.arcs.push(stored);

      let last = chain.points.len() - 1;

      for (index, entry) in chain.shapes.iter_mut().enumerate() {
        *entry = ArcRef::On {
          arc: 0,
          role: if index == 0 {
            PointRole::Start
          } else if index == last {
            PointRole::End
          } else {
            PointRole::Interior
          },
        };
      }
    }

    self.append_chain(&chain);
  }

  /// Insert an arc before the vertex at an index.
  ///
  /// Port of `Insert( size_t aVertex, const SHAPE_ARC&, int aMaxError )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1664`. An insertion
  /// inside an arc splits that arc first, exactly as [`LineChain::insert`]
  /// does, and the new arc takes its place in the arc vector so that chain
  /// order holds.
  ///
  /// Two deviations from KiCad, both erratum E5.
  ///
  /// - KiCad finds the insertion position by walking `m_shapes.rbegin()`
  ///   to `m_shapes.rend() + aVertex` (`:1674`), which is past the reverse
  ///   end for any non zero vertex and reads out of bounds. That is an out
  ///   of bounds read rather than a defined behaviour, so the milestone
  ///   rule says fix it: this scans the vertices from the insertion point
  ///   forward and takes the first arc they still refer to, which is the
  ///   position chain order asks for.
  /// - KiCad skips the "more than two points" demotion that
  ///   [`LineChain::append_arc`] applies (`:1620`), so an arc whose
  ///   polyline is a bare chord still gets an entry and the two arc entry
  ///   points disagree about what counts as an arc. This applies the rule
  ///   in both.
  ///
  /// # Panics
  ///
  /// When `index` is not less than [`LineChain::point_count`], which is
  /// KiCad's `wxCHECK` at `:1666`.
  pub fn insert_arc(&mut self, index: usize, arc: &ShapeArc, max_error: i32) {
    assert!(
      index < self.points.len(),
      "insert index {index} is out of range for a chain of {} points",
      self.points.len()
    );

    let polyline = arc.convert_to_polyline(max_error);

    // :1620's rule, applied here too.
    if polyline.points.len() <= 2 {
      for (offset, point) in polyline.points.iter().enumerate() {
        self.insert(index + offset, *point);
      }

      return;
    }

    // :1671
    if index > 0 && self.is_pt_on_arc(index) {
      self.split_arc(index, false);
    }

    let position = (index..self.shapes.len())
      .find_map(|vertex| self.shapes[vertex].entering_arc())
      .unwrap_or(self.arcs.len());

    for entry in &mut self.shapes {
      *entry = entry.shifted_up_by(position, 1);
    }

    let mut stored = *arc;

    stored.set_width(0);
    self.arcs.insert(position, stored);

    let last = polyline.points.len() - 1;

    for (offset, point) in polyline.points.iter().enumerate() {
      self.points.insert(index + offset, *point);
      self.shapes.insert(
        index + offset,
        ArcRef::On {
          arc: position,
          role: if offset == 0 {
            PointRole::Start
          } else if offset == last {
            PointRole::End
          } else {
            PointRole::Interior
          },
        },
      );
    }

    self.check_invariants();
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
  /// An insertion inside an arc splits that arc first (`:1647`), leaving a
  /// short straight segment between the arc's new end and the inserted
  /// point, and the inserted vertex itself is always plain.
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

    // :1647
    if index > 0 && self.is_pt_on_arc(index) {
      self.split_arc(index, false);
    }

    self.points.insert(index, point);
    self.shapes.insert(index, ArcRef::Plain);
    self.check_invariants();
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
  /// drops that last point.
  ///
  /// Arcs are handled before any point is erased. An arc the range only
  /// partially covers is split at the boundary so the surviving half keeps
  /// its curvature, a boundary index that lands on a shared point is
  /// pulled in by one so the shared point survives, and every arc fully
  /// inside the range is degraded to its polyline before the points go.
  /// Erratum E6 lives in that last step and is reproduced; the comment on
  /// the loop says what it costs.
  pub fn remove_range(&mut self, start: usize, end: usize) {
    let closed_state = self.closed;

    self.set_closed(false);

    let point_count = self.points.len();

    if start >= point_count || end >= point_count || start > end {
      self.set_closed(closed_state);
      return;
    }

    let mut start = start;
    let mut end = end;

    // :1100, cut a partially covered arc free at each end, and step past a
    // shared point rather than deleting it.
    if !self.is_arc_start(start) && self.is_pt_on_arc(start) {
      self.split_arc(start, false);
    }

    if self.is_shared_pt(start) {
      start += 1;
    }

    if !self.is_arc_end(end)
      && self.is_pt_on_arc(end)
      && end < self.points.len() - 1
    {
      self.split_arc(end + 1, true);
    }

    if self.is_shared_pt(end) {
      if end == 0 {
        self.set_closed(closed_state);
        return;
      }

      end -= 1;
    }

    if start > end {
      self.set_closed(closed_state);
      return;
    }

    // :1118, every arc fully inside the range goes. KiCad collects the
    // indices into a `std::set<size_t>` and `convertArc`s them in
    // increasing order while `convertArc` renumbers everything above the
    // one it erased, so the second call works on a renumbered vector with
    // a stale index (erratum E6). The removed point range is contiguous,
    // so the doomed indices are contiguous too, and the net effect is that
    // every other arc of the range is erased and the ones between are left
    // behind with every reference to them gone. That is defined behaviour,
    // so the milestone rule says reproduce it; the ascending order and the
    // stale indices are KiCad's and the test naming the erratum pins them.
    let mut doomed: Vec<usize> = Vec::new();

    for index in start..=end {
      match self.shapes[index] {
        ArcRef::Plain => {}
        ArcRef::On { arc, .. } => doomed.push(arc),
        ArcRef::Shared { ends, starts } => {
          if index == start {
            doomed.push(starts);
          } else if index == end {
            doomed.push(ends);
          } else {
            doomed.push(ends);
            doomed.push(starts);
          }
        }
      }
    }

    doomed.sort_unstable();
    doomed.dedup();

    for arc in doomed {
      self.convert_arc(arc);
    }

    self.points.drain(start..=end);
    self.shapes.drain(start..=end);
    self.set_closed(closed_state);
    self.check_invariants();
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
    let mut incoming = other.clone();

    if incoming.points.is_empty() {
      self.remove_range(start, end);
      return;
    }

    if incoming.points[0] == self.points[start] {
      start += 1;
      incoming.remove(0);

      if incoming.points.is_empty() {
        self.remove_range(start, end);
        return;
      }
    }

    if incoming.points[incoming.points.len() - 1] == self.points[end] && end > 0
    {
      end -= 1;
      incoming.remove(incoming.points.len() - 1);
    }

    self.remove_range(start, end);

    if incoming.points.is_empty() {
      return;
    }

    // Change 2 of note 09 section 11.1. KiCad appends the incoming arcs at
    // the end of `m_arcs` whatever position their points took (`:1071`),
    // which breaks the chain order `Reverse` depends on (erratum E7). The
    // splice position is the number of arcs whose points end up before the
    // insertion point, which is the first arc index any surviving vertex
    // at or after `start` still refers to, or the whole vector when none
    // does.
    let splice_at = (start..self.shapes.len())
      .find_map(|index| self.shapes[index].entering_arc())
      .unwrap_or(self.arcs.len());
    let incoming_count = incoming.arcs.len();

    for entry in &mut self.shapes {
      *entry = entry.shifted_up_by(splice_at, incoming_count);
    }

    let spliced = incoming
      .shapes
      .iter()
      .map(|entry| entry.offset_by(splice_at));

    self.shapes.splice(start..start, spliced);
    self.points.splice(start..start, incoming.points);
    self.arcs.splice(splice_at..splice_at, incoming.arcs);
    self.check_invariants();
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
  /// Arcs survive, and three of KiCad's behaviours around them are worth
  /// stating because a caller cannot guess them.
  ///
  /// - A slice that **starts** inside an arc copies points forward while
  ///   they belong to that arc, with no `end` bound, and rebuilds the arc
  ///   from the new start to the parent's own end point (`:1444`,
  ///   `:1456`). So a range whose two ends are both interior to one arc
  ///   comes back running past `end` to that arc's end. That is erratum
  ///   E10 and it is reproduced.
  /// - A slice that **ends** inside an arc is bounded correctly (`:1491`).
  /// - A whole arc that fits is re-polygonised through
  ///   [`LineChain::append_arc`] (`:1519`), so the interior points of the
  ///   result are not the interior points of the original unless the two
  ///   accuracies agree. Use [`LineChain::slice_with_max_error`] to say
  ///   which accuracy.
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
    self.slice_with_max_error(start, end, Self::ARC_POLYGONIZATION_MAX_ERROR)
  }

  /// The inclusive range of points `[start, end]` as a new chain, naming
  /// the accuracy a whole arc is re-polygonised at.
  ///
  /// Port of `Slice( int, int, int aMaxError )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1417`.
  /// [`LineChain::slice`] is the two argument form, which passes
  /// [`LineChain::ARC_POLYGONIZATION_MAX_ERROR`] as KiCad's does
  /// (`:1414`). Everything else about the two is identical; see
  /// [`LineChain::slice`] for the arc behaviour.
  ///
  /// # Errors
  ///
  /// As [`LineChain::slice`].
  pub fn slice_with_max_error(
    &self,
    start: usize,
    end: usize,
    max_error: i32,
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

    let mut result = LineChain::new();
    let mut start = start;

    // :1437, the slice begins in the middle of an arc.
    if self.is_arc_segment(start) && !self.is_arc_start(start) {
      let Some(parent_index) = self.arc_index(start) else {
        unreachable!("an arc segment has an arc index");
      };
      let Some(parent) = self.arc(parent_index) else {
        unreachable!("an arc index names a stored arc");
      };
      let new_start = self.points[start];
      let mut walk = start;

      // :1444, no `end` bound: erratum E10.
      while walk < self.points.len()
        && self.arc_index(walk) == Some(parent_index)
      {
        result.points.push(self.points[walk]);
        result.shapes.push(ArcRef::On {
          arc: 0,
          role: PointRole::Interior,
        });
        walk += 1;
      }

      // :1456
      result.arcs.push(ShapeArc::from_start_end_center(
        new_start,
        parent.end(),
        parent.center(),
        !parent.is_ccw(),
        0,
      ));
      result.mark_arc_run_roles(0);

      // :1463
      start += result.points.len();
    }

    // :1466
    let mut index = start;

    while index <= end && index < point_count {
      let next_shape = self.next_shape(index);
      let is_last_shape = next_shape.is_none();

      if self.is_arc_start(index) {
        // :1476
        if (is_last_shape && end != point_count - 1)
          || next_shape.is_some_and(|next| next > end)
        {
          if index == end {
            // :1481, a single point of an arc, appended plain.
            result.append(self.points[index]);
            return Ok(result);
          }

          // :1487, the slice ends in the middle of this arc.
          let Some(parent_index) = self.arc_index(index) else {
            unreachable!("an arc start has an arc index");
          };
          let Some(parent) = self.arc(parent_index) else {
            unreachable!("an arc index names a stored arc");
          };
          let first_result_arc = result.arcs.len();

          // :1492
          while index <= end && index < point_count {
            if self.arc_index(index) != Some(parent_index) {
              break;
            }

            result.points.push(self.points[index]);
            result.shapes.push(ArcRef::On {
              arc: first_result_arc,
              role: PointRole::Interior,
            });
            index += 1;
          }

          // :1503
          result.arcs.push(ShapeArc::from_start_end_center(
            parent.start(),
            self.points[end],
            parent.center(),
            !parent.is_ccw(),
            0,
          ));
          result.mark_arc_run_roles(first_result_arc);
          result.check_invariants();

          return Ok(result);
        }

        // :1517, the whole arc fits.
        let Some(parent_index) = self.arc_index(index) else {
          unreachable!("an arc start has an arc index");
        };
        let Some(parent) = self.arc(parent_index) else {
          unreachable!("an arc index names a stored arc");
        };

        result.append_arc(&parent, max_error);

        if is_last_shape {
          result.check_invariants();
          return Ok(result);
        }
      } else {
        // :1526
        if index == start {
          result.append(self.points[index]);
        }

        let next_is_arc =
          next_shape.is_some_and(|next| self.is_arc_segment(next));

        // :1534
        if !next_is_arc && index < self.segment_count() && index < end {
          result.append(self.segment(index).b);
        }
      }

      match next_shape {
        // :1470, `NextShape` reached the end.
        None => {
          result.check_invariants();
          return Ok(result);
        }
        Some(next) => index = next,
      }
    }

    result.check_invariants();
    Ok(result)
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
  ///
  /// Splitting **on an arc segment** does not degrade the arc: the point
  /// is inserted carrying the arc's index and `splitArc` then makes it a
  /// shared vertex (`:1219`), so one arc becomes two that meet there. Both
  /// halves are rebuilt through `ConstructFromStartEndCenter`, which is
  /// lossy (note 09 section 1.2), so neither half reports exactly the
  /// parent's centre.
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

    // :1217
    if self.is_arc_segment(index) {
      let Some(arc) = self.arc_index(index) else {
        unreachable!("an arc segment has an arc index");
      };

      self.points.insert(new_index, point);
      self.shapes.insert(
        new_index,
        ArcRef::On {
          arc,
          role: PointRole::Interior,
        },
      );
      // :1224, make the inserted point a shared point.
      self.split_arc(new_index, true);
      self.check_invariants();
    } else {
      self.insert(new_index, point);
    }

    Some(new_index)
  }

  /// Reverse the point order in place.
  ///
  /// KiCad has no in place reverse; `Reverse`
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:910`) copies. The
  /// closed flag and the width survive, as they do in the copy.
  ///
  /// The shape entries and the arcs reverse with the points, every arc
  /// index is remapped to `arc_count - index - 1`, the two halves of a
  /// shared vertex swap, and each arc is reversed in place (`:940`). The
  /// remap is only a correct reversal while the arcs are in chain order,
  /// which is the invariant this module enforces and KiCad's `Replace`
  /// breaks (erratum E7).
  ///
  /// KiCad reverses each arc with `SHAPE_ARC::Reverse()`, which swaps the
  /// two endpoints in place and leaves the cached centre and bounding box
  /// stale (erratum E4). [`ShapeArc`] caches nothing, so the port has
  /// nothing to go stale and the two forms agree here.
  pub fn reverse(&mut self) {
    self.points.reverse();
    self.shapes.reverse();
    self.arcs.reverse();

    let arc_count = self.arcs.len();
    let remap = |index: usize| arc_count - index - 1;

    for entry in &mut self.shapes {
      *entry = match *entry {
        ArcRef::Plain => ArcRef::Plain,
        ArcRef::On { arc, role } => ArcRef::On {
          arc: remap(arc),
          role: match role {
            PointRole::Start => PointRole::End,
            PointRole::Interior => PointRole::Interior,
            PointRole::End => PointRole::Start,
          },
        },
        // :939, first and second swap, which is what keeps `.first` the
        // arc that ends here.
        ArcRef::Shared { ends, starts } => ArcRef::Shared {
          ends: remap(starts),
          starts: remap(ends),
        },
      };
    }

    for arc in &mut self.arcs {
      arc.reverse();
    }

    self.check_invariants();
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
  /// `libs/kimath/include/geometry/shape_line_chain.h:776`, which
  /// translates the points and the arcs alike. Exact in both.
  pub fn move_by(&mut self, delta: Vec2) {
    for point in &mut self.points {
      *point += delta;
    }

    for arc in &mut self.arcs {
      arc.move_by(delta);
    }
  }

  /// Move one point to a new position.
  ///
  /// Deviation from `SetPoint`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:1362`, which wraps a
  /// negative or over range index one step the way `CPoint` does. This
  /// panics instead, for the reason given on [`LineChain::point`].
  ///
  /// Every arc touching the vertex is destroyed (`:1371`): there is no
  /// operation that moves an arc endpoint, and the router uses `splitArc`
  /// and [`LineChain::slice`] instead. The destruction is a degradation
  /// rather than an erasure, as it is in KiCad: the arc's approximation
  /// points all stay where they are and only the references and the arc
  /// itself go.
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

    // :1371. KiCad runs `convertArc` over both halves of the entry, but
    // the lambda reads the live pair and `convertArc` rewrites it: after
    // the first call the entry's second half is already the sentinel, so
    // the second call does nothing. At a shared vertex only the arc
    // **ending** there is destroyed and the one starting there survives
    // with a start point that no longer matches. That is defined, so it is
    // reproduced rather than tidied.
    match self.shapes[index] {
      ArcRef::Plain => {}
      ArcRef::On { arc, .. } => self.convert_arc(arc),
      ArcRef::Shared { ends, .. } => self.convert_arc(ends),
    }

    self.check_invariants();
  }

  /// Reflect every point in an axis.
  ///
  /// Port of `Mirror( const SEG& )`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:989`. The router calls
  /// this once, from the meander generator
  /// (`pcbnew/router/pns_meander.cpp:685`). KiCad's other overload, which
  /// mirrors about a horizontal or vertical line through a reference
  /// point (`:974`), has no caller in the router and is not ported.
  ///
  /// The arcs are mirrored with the points and the chain is **not**
  /// reversed, so a mirrored closed chain winds the other way round. That
  /// is KiCad's behaviour and note 01 section 12.4 records what depends on
  /// the winding.
  pub fn mirror(&mut self, axis: &Seg) {
    for point in &mut self.points {
      *point = axis.reflect_point(*point);
    }

    for arc in &mut self.arcs {
      arc.mirror(axis);
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
  ///
  /// Segments that lie on an arc are skipped and each arc contributes its
  /// own `f64` length instead, truncated into the running total. This is
  /// the **only** arc aware length in the router (note 09 section 2.7).
  /// KiCad sums over `ArcCount()`, which counts an orphaned arc too
  /// (erratum E13); this sums over [`LineChain::live_arcs`], so an arc no
  /// vertex refers to adds nothing.
  pub fn length(&self) -> i64 {
    let straight: i64 = (0..self.segment_count())
      .filter(|index| !self.is_arc_segment(*index))
      .map(|index| i64::from(self.segment(index).length()))
      .sum();

    let curved: i64 =
      self.live_arcs().map(|(_, arc)| arc.length() as i64).sum();

    straight + curved
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
  ///
  /// **Polyline only, deliberately.** KiCad's is arc unaware and so is this: it
  /// sums straight segment lengths whatever the segments lie on
  /// (`shape_line_chain.cpp:1952`). Note 09 section 2.5.
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
  ///
  /// **Polyline only, deliberately.** KiCad's is arc unaware and so is this: a
  /// point a given distance along a chain with an arc is that distance along the
  /// polyline, which is shorter than the arc by the sagitta error
  /// (`shape_line_chain.cpp:2671`). Note 09 section 2.5.
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
  /// Arcs survive. A run may not step **over** a vertex that belongs to an
  /// arc (`:2816`), so no approximation point is ever dropped, but the
  /// comment at `:2813` is explicit that a run may start or end on one, so
  /// a straight run abutting an arc still collapses. The arcs themselves
  /// are untouched, which means a chain can come out of here with an arc
  /// no vertex refers to; [`LineChain::live_arcs`] is what keeps that from
  /// mattering (erratum E13).
  ///
  /// This is where [`LineChain::simplify2`] differs, and the difference is
  /// deliberate; see that method.
  pub fn simplify(&mut self, tolerance: i32) {
    let point_count = self.points.len();

    if point_count < 3 {
      return;
    }

    let mut new_points: Vec<Vec2> = Vec::with_capacity(point_count);
    let mut new_shapes: Vec<ArcRef> = Vec::with_capacity(point_count);
    let mut start_index = 0usize;

    while start_index < point_count {
      new_points.push(self.points[start_index]);
      new_shapes.push(self.shapes[start_index]);

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
          // :2816, an intermediate vertex on an arc stops the run.
          if self.is_pt_on_arc(test_index) {
            can_simplify = false;
            break;
          }

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
      new_shapes.push(self.shapes[point_count - 1]);
    }

    if !self.closed
      && self.points[point_count - 1] != new_points[new_points.len() - 1]
    {
      new_points.push(self.points[point_count - 1]);
      new_shapes.push(self.shapes[point_count - 1]);
    }

    self.points = new_points;
    self.shapes = new_shapes;
    self.check_invariants();
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
  ///
  /// Two vertices at the same position merge only when their shape entries
  /// agree or one of them is plain, and the surviving entry is the non
  /// plain one (`:2934`). The colinear stage checks that the run's first
  /// two vertices are plain before it starts (`:2968`) but does **not**
  /// re-check as the run advances (`:2971`), so a shallow arc whose
  /// consecutive approximation points sit within a nanometre of the chord
  /// can lose interior points while its arc entry survives, leaving the
  /// chain claiming an arc across a run that no longer approximates it.
  /// That is erratum E12 and it is reproduced.
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
    let (unique, unique_shapes) = self.merged_duplicate_points();

    // Stage 2, `shape_line_chain.cpp:2963`: collapse colinear runs.
    let unique_count = unique.len();
    let limit = unique_count.saturating_sub(2);

    self.points.clear();
    self.shapes.clear();

    let mut index = 0usize;

    while index < limit {
      let first = unique[index];
      let mut reach = index;

      // :2968, both ends of the run have to start out plain. The run
      // extension below never looks at a shape entry again, which is E12.
      if remove_colinear
        && unique_shapes[index] == ArcRef::Plain
        && unique_shapes[index + 1] == ArcRef::Plain
      {
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
      self.shapes.push(unique_shapes[index]);

      if reach > index {
        index = reach;
      }

      if reach == limit {
        self.points.push(unique[unique_count - 1]);
        self.shapes.push(unique_shapes[unique_count - 1]);
        self.check_invariants();
        return;
      }

      index += 1;
    }

    if unique_count > 1 {
      self.points.push(unique[unique_count - 2]);
      self.shapes.push(unique_shapes[unique_count - 2]);
    }

    self.points.push(unique[unique_count - 1]);
    self.shapes.push(unique_shapes[unique_count - 1]);
    self.check_invariants();
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

    let (unique, unique_shapes) = self.merged_duplicate_points();

    self.points = unique;
    self.shapes = unique_shapes;
    self.check_invariants();
  }

  /// Stage one of [`LineChain::simplify2`], shared with
  /// [`LineChain::remove_duplicate_points`].
  ///
  /// Port of `libs/kimath/src/geometry/shape_line_chain.cpp:2928` and the
  /// identical loop at `:2739`. Two vertices at the same position merge
  /// only when their shape entries agree or one of them is plain, and the
  /// entry that survives is the non plain one, so a duplicate of an arc's
  /// endpoint does not cost the arc its reference.
  fn merged_duplicate_points(&self) -> (Vec<Vec2>, Vec<ArcRef>) {
    let mut unique: Vec<Vec2> = Vec::with_capacity(self.points.len());
    let mut unique_shapes: Vec<ArcRef> = Vec::with_capacity(self.points.len());
    let mut index = 0usize;

    while index < self.points.len() {
      let mut next = index + 1;

      while next < self.points.len()
        && self.points[index] == self.points[next]
        && (self.shapes[index] == self.shapes[next]
          || self.shapes[index] == ArcRef::Plain
          || self.shapes[next] == ArcRef::Plain)
      {
        next += 1;
      }

      let mut keep = self.shapes[index];

      if keep == ArcRef::Plain {
        keep = self.shapes[next - 1];
      }

      unique.push(self.points[index]);
      unique_shapes.push(keep);
      index = next;
    }

    (unique, unique_shapes)
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
  ///
  /// **Polyline only, deliberately.** KiCad's is arc unaware and so is this: an
  /// arc reaches it as its stored approximation, which is what
  /// `PNS::HullIntersection` and therefore the whole walkaround sees
  /// (`shape_line_chain.cpp:1741`). Note 09 section 2.5.
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
  ///
  /// **Polyline only, deliberately.** KiCad's is arc unaware and so is this: both
  /// chains reach it as their stored approximations
  /// (`shape_line_chain.cpp:1841`). Note 09 section 2.5.
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
  ///
  /// **Polyline only, deliberately.** KiCad's is arc unaware and so is this: it
  /// is [`LineChain::intersect_chain`] with an early exit
  /// (`shape_line_chain.cpp:2610`). Note 09 section 2.5.
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
  ///
  /// **Polyline only, deliberately.** KiCad's is arc unaware and so is this:
  /// KiCad has an arc aware `SelfIntersectingWithArcs`
  /// (`shape_line_chain.cpp:2234`) and the router never calls it, using this one
  /// at all three of its sites (erratum E16). Note 09 section 2.5.
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

    // :445, the polyline phase skips the approximation of every arc.
    for index in 0..self.segment_count() {
      if self.is_arc_segment(index) {
        continue;
      }

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

    // :479, then the arcs, exactly, and the first one that collides wins.
    // KiCad walks `ArcCount()`, which is what lets an orphaned arc collide
    // (erratum E13); this walks [`LineChain::live_arcs`].
    for (_, arc) in self.live_arcs() {
      debug_assert_eq!(arc.width(), 0, "a chain stores zero width arcs");

      if let Some(collision) = arc.collide_point(point, clearance) {
        return Some(Collision {
          actual: collision.actual,
          location: collision.location,
        });
      }
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

    // :836, the polyline phase skips the approximation of every arc.
    for index in 0..self.segment_count() {
      if self.is_arc_segment(index) {
        continue;
      }

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

    // :870, the polyline's best distance carries into the arc phase so an
    // arc can only improve on it. KiCad keeps it in a `SEG::ecoord`, wide
    // enough to hold the square root of the `ECOORD_MAX` a chain of
    // nothing but arc segments leaves behind; this keeps the same width
    // rather than narrowing to the `i32` the other paths use.
    let mut closest_distance = isqrt(closest_squared.max(0) as u64) as i64;

    // :873, again the live arcs rather than KiCad's `ArcCount()`.
    for (_, arc) in self.live_arcs() {
      debug_assert_eq!(arc.width(), 0, "a chain stores zero width arcs");

      if let Some(collision) = arc.collide_seg(seg, clearance)
        && i64::from(collision.actual) < closest_distance
      {
        closest_distance = i64::from(collision.actual);
        nearest = collision.location;
      }
    }

    // :894
    if closest_distance == 0 || closest_distance < i64::from(clearance) {
      return Some(Collision {
        actual: i32::try_from(closest_distance).unwrap_or(i32::MAX),
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
  /// `allow_internal_shape_points` is KiCad's `aAllowInternalShapePoints`,
  /// which defaults to `true` (`shape_line_chain.h:894`). Clearing it asks
  /// for the answer to be **snapped to an arc endpoint** when the winning
  /// segment lies on an arc: advance to the nearer of that segment's two
  /// endpoints, return it when it is itself an arc start or end, and
  /// otherwise return the nearer of the containing arc's two true
  /// endpoints (`:2425` to `:2452`). On a chain with no arcs the flag
  /// changes nothing.
  ///
  /// Erratum E14 is the `nearest++` at `:2433` reaching `PointCount()`,
  /// after which KiCad calls the unchecked `ArcIndex` and `Arc`. It is
  /// only reachable on a closed chain whose last segment is an arc
  /// segment, which the router does not build. [`LineChain::arc_index`]
  /// and [`LineChain::arc`] return [`Option`] here (change 4 of note 09
  /// section 11.1), so the read is checked and the answer falls back to
  /// the unsnapped nearest point.
  ///
  /// Returns `None` for an empty chain, where KiCad returns `(0, 0)` with
  /// the comment that the only right answer is not to crash (`:2406`). A
  /// chain of one point answers with that point, which is what KiCad's
  /// failed `wxCHECK` in `Segment` degrades to (`:1293`).
  pub fn nearest_point(
    &self,
    point: Vec2,
    allow_internal_shape_points: bool,
  ) -> Option<Vec2> {
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

    let unsnapped = self.segment(nearest).nearest_point_to_point(point);

    // :2424
    if allow_internal_shape_points
      || nearest == 0
      || nearest >= self.points.len()
      || !self.is_arc_segment(nearest)
    {
      return Some(unsnapped);
    }

    let segment = self.segment(nearest);
    let to_start = segment.a.widening_sub(point);
    let to_end = segment.b.widening_sub(point);

    // :2432
    if to_start.euclidean_norm() > to_end.euclidean_norm() {
      nearest += 1;
    }

    // :2435
    if self.is_arc_start(nearest) || self.is_arc_end(nearest) {
      return Some(self.points[nearest]);
    }

    // :2441. E14 is the unchecked read here.
    let Some(arc) = self.arc_index(nearest).and_then(|index| self.arc(index))
    else {
      return Some(unsnapped);
    };

    if arc.start().widening_sub(point).euclidean_norm()
      > arc.end().widening_sub(point).euclidean_norm()
    {
      Some(arc.end())
    } else {
      Some(arc.start())
    }
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
  ///
  /// **Polyline only, deliberately.** KiCad's is arc unaware and so is this:
  /// containment is decided against the stored approximation
  /// (`shape_line_chain.cpp:3060`). Note 09 section 2.5.
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
  ///
  /// **Polyline only, deliberately.** KiCad's is arc unaware and so is this: the
  /// shoelace sum runs over the points (`shape_line_chain.cpp:2696`). Note 09
  /// section 2.5.
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
  ///
  /// Note 09 section 11.3 lists this among the polyline only members, and
  /// that holds of its own body only: it has no arc logic. Everything it
  /// calls is arc aware, in KiCad as here, so a range cut out of an arc
  /// bearing chain comes back carrying arcs. `NearestPoint` is called with
  /// `aAllowInternalShapePoints` cleared (`:2882`), so a point that lands
  /// on an arc is pulled to that arc's nearer end before the chain is cut
  /// there.
  pub fn split_three_way(
    &self,
    start: Vec2,
    end: Vec2,
  ) -> Option<(LineChain, LineChain, LineChain)> {
    // :2882, both snaps clear `aAllowInternalShapePoints`, so a point
    // that lands on an arc is pulled to that arc's nearer end before the
    // chain is cut there.
    let end_on_chain = self.nearest_point(end, false)?;
    let start_on_chain = self.nearest_point(start, false)?;

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
  /// last point, and when that last point was on an arc the arc reference
  /// moves onto vertex zero, which becomes the shared vertex of the seam
  /// (`:220`). Opening a chain whose vertex zero is shared is the mirror
  /// image: vertex zero is duplicated at the end and the two halves are
  /// split between them (`:234`).
  fn merge_first_last_point_if_needed(&mut self) {
    if self.closed {
      if self.points.len() > 1
        && self.points[0] == self.points[self.points.len() - 1]
      {
        let last = self.points.len() - 1;

        // :220
        if let Some(arriving) = self.shapes[last].arc_index() {
          self.shapes[0] = match self.shapes[0] {
            ArcRef::Plain => ArcRef::On {
              arc: arriving,
              role: PointRole::End,
            },
            ArcRef::On { arc, .. } => ArcRef::Shared {
              ends: arriving,
              starts: arc,
            },
            // KiCad writes `.second = .first` and then `.first = arriving`
            // unconditionally, which on an already shared vertex zero
            // drops the arc that used to end there. Vertex zero can only
            // be shared on a chain that is already closed, and this branch
            // runs while closing one, so the case is not reachable.
            ArcRef::Shared { starts, .. } => ArcRef::Shared {
              ends: arriving,
              starts,
            },
          };
        }

        self.points.pop();
        self.shapes.pop();
        self.fix_indices_rotation();
      }

      return;
    }

    // :234
    if self.points.len() > 1 && self.is_shared_pt(0) {
      let ArcRef::Shared { ends, starts } = self.shapes[0] else {
        unreachable!("a shared point holds two arcs");
      };

      self.points.push(self.points[0]);
      self.shapes.push(ArcRef::On {
        arc: ends,
        role: PointRole::End,
      });
      self.shapes[0] = ArcRef::On {
        arc: starts,
        role: PointRole::Start,
      };
    }
  }

  /// Rotate the chain so that no arc straddles the seam.
  ///
  /// Port of `fixIndicesRotation`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:191`, which rotates
  /// right while vertex zero is on an arc without being that arc's start,
  /// with a rotation count guard against an infinite loop on a malformed
  /// chain (`:207`). The rotation is what keeps the chain order invariant
  /// meaningful for a closed chain.
  fn fix_indices_rotation(&mut self) {
    if self.shapes.len() <= 1 {
      return;
    }

    let mut rotations = 0usize;

    while self.arc_index(0).is_some() && !self.is_arc_start(0) {
      self.points.rotate_right(1);
      self.shapes.rotate_right(1);

      rotations += 1;

      if rotations > self.shapes.len() {
        return;
      }
    }
  }

  /// Degrade one arc to its polyline.
  ///
  /// Port of `convertArc`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:246`: every reference
  /// to the arc is cleared, every higher reference is decremented, and the
  /// arc is erased. **The points stay**, so the arc's approximation
  /// survives as an ordinary polyline. An index past the end is a no
  /// operation, as it is in KiCad (`:251`).
  fn convert_arc(&mut self, arc_index: usize) {
    if arc_index >= self.arcs.len() {
      return;
    }

    for entry in &mut self.shapes {
      *entry = match *entry {
        ArcRef::Plain => ArcRef::Plain,
        ArcRef::On { arc, role } => {
          if arc == arc_index {
            ArcRef::Plain
          } else {
            ArcRef::On {
              arc: if arc > arc_index { arc - 1 } else { arc },
              role,
            }
          }
        }
        ArcRef::Shared { ends, starts } => {
          let ends_gone = ends == arc_index;
          let starts_gone = starts == arc_index;
          let shift = |index: usize| {
            if index > arc_index { index - 1 } else { index }
          };

          match (ends_gone, starts_gone) {
            (true, true) => ArcRef::Plain,
            // :267, KiCad re-establishes "second is a point whenever first
            // is" with a swap; the enum does it by construction.
            (true, false) => ArcRef::On {
              arc: shift(starts),
              role: PointRole::Start,
            },
            (false, true) => ArcRef::On {
              arc: shift(ends),
              role: PointRole::End,
            },
            (false, false) => ArcRef::Shared {
              ends: shift(ends),
              starts: shift(starts),
            },
          }
        }
      };
    }

    self.arcs.remove(arc_index);
  }

  /// Rebuild one arc between two new endpoints about its own centre.
  ///
  /// Port of `amendArc`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:275`, which passes the
  /// old arc's centre and handedness to `ConstructFromStartEndCenter`.
  /// Both survive the call only as far as note 09 section 1.2 says they
  /// do: the rebuilt arc keeps nothing but its three points, and the
  /// centre it reports afterwards is recomputed from them.
  fn amend_arc(&mut self, arc_index: usize, start: Vec2, end: Vec2) {
    let Some(arc) = self.arc(arc_index) else {
      return;
    };

    self.arcs[arc_index] = ShapeArc::from_start_end_center(
      start,
      end,
      arc.center(),
      !arc.is_ccw(),
      0,
    );
  }

  /// Cut an arc at one of its interior vertices.
  ///
  /// Port of `splitArc`,
  /// `libs/kimath/src/geometry/shape_line_chain.cpp:292`. With
  /// `coincident` the vertex becomes a shared point and the one arc
  /// becomes two that meet there. Without it the arc is shortened to end
  /// at the **previous** vertex, so a short straight segment is left
  /// between that vertex and this one.
  ///
  /// Nothing to do when the vertex is already an arc start that is not
  /// shared, or when it is not on an arc at all (`:297`, `:300`).
  fn split_arc(&mut self, point_index: usize, coincident: bool) {
    if !self.is_shared_pt(point_index) && self.is_arc_start(point_index) {
      return;
    }

    if !self.is_pt_on_arc(point_index) {
      return;
    }

    if point_index >= self.shapes.len() {
      return;
    }

    // :308, the vertex already ends an arc.
    if self.is_shared_pt(point_index) || self.is_arc_end(point_index) {
      if coincident || point_index == 0 {
        return;
      }

      let Some(first_arc) = self.shapes[point_index].entering_arc() else {
        unreachable!("an arc end has an entering arc");
      };
      let Some(arc) = self.arc(first_arc) else {
        return;
      };
      let new_end = self.points[point_index - 1];

      self.amend_arc(first_arc, arc.start(), new_end);

      self.shapes[point_index] = match self.shapes[point_index] {
        ArcRef::Shared { starts, .. } => ArcRef::On {
          arc: starts,
          role: PointRole::Start,
        },
        _ => ArcRef::Plain,
      };
      self.mark_arc_run_roles(first_arc);

      return;
    }

    // KiCad reads `m_points[aPtIndex - 1]` below without guarding
    // `aPtIndex == 0` (`:345`). The vertex would have to be interior to an
    // arc while being the chain's first point, which no mutator here
    // produces, so the guard costs nothing and removes the underflow.
    if !coincident && point_index == 0 {
      return;
    }

    let Some(current_index) = self.arc_index(point_index) else {
      return;
    };
    let Some(current) = self.arc(current_index) else {
      return;
    };

    // :345
    let first_half_end = if coincident {
      self.points[point_index]
    } else {
      self.points[point_index - 1]
    };
    let second_half_start = self.points[point_index];
    let first_half = ShapeArc::from_start_end_center(
      current.start(),
      first_half_end,
      current.center(),
      !current.is_ccw(),
      0,
    );
    let second_half = ShapeArc::from_start_end_center(
      second_half_start,
      current.end(),
      current.center(),
      !current.is_ccw(),
      0,
    );

    // :352, the first half would have no points of its own.
    if !coincident && self.arc_index(point_index - 1) != Some(current_index) {
      self.arcs[current_index] = second_half;
      self.mark_arc_run_roles(current_index);

      return;
    }

    self.arcs[current_index] = first_half;
    self.arcs.insert(current_index + 1, second_half);

    let mut first_of_second_half = point_index;

    if coincident {
      self.shapes[point_index] = ArcRef::Shared {
        ends: current_index,
        starts: current_index + 1,
      };
      first_of_second_half += 1;
    }

    // :366, only the second half of the point range is renumbered.
    for index in first_of_second_half..self.shapes.len() {
      self.shapes[index] = match self.shapes[index] {
        ArcRef::Plain => ArcRef::Plain,
        ArcRef::On { arc, role } => ArcRef::On { arc: arc + 1, role },
        ArcRef::Shared { ends, starts } => ArcRef::Shared {
          ends: ends + 1,
          starts: starts + 1,
        },
      };
    }

    self.mark_arc_run_roles(current_index);
    self.mark_arc_run_roles(current_index + 1);
  }

  /// Give the run of vertices that refer to one arc their roles.
  ///
  /// There is no KiCad counterpart, because KiCad reads the roles back off
  /// the arc's own endpoints every time it needs them
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:3278`, `:3299`).
  /// Change 1 of note 09 section 11.1 stores them instead, so every mutator
  /// that cuts or renumbers a run has to restate them.
  fn mark_arc_run_roles(&mut self, arc_index: usize) {
    let mut first: Option<usize> = None;
    let mut last: Option<usize> = None;

    for index in 0..self.shapes.len() {
      let touches = match self.shapes[index] {
        ArcRef::Plain => false,
        ArcRef::On { arc, .. } => arc == arc_index,
        ArcRef::Shared { ends, starts } => {
          ends == arc_index || starts == arc_index
        }
      };

      if touches {
        first.get_or_insert(index);
        last = Some(index);
      }
    }

    let (Some(first), Some(last)) = (first, last) else {
      return;
    };

    for index in first..=last {
      if let ArcRef::On { arc, .. } = self.shapes[index]
        && arc == arc_index
      {
        self.shapes[index] = ArcRef::On {
          arc,
          role: if index == first {
            PointRole::Start
          } else if index == last {
            PointRole::End
          } else {
            PointRole::Interior
          },
        };
      }
    }
  }

  /// Check what this module promises about its own storage.
  ///
  /// Change 2 of note 09 section 11.1 asks for this, and every mutator
  /// calls it. It is a debug assertion, so a release build pays nothing.
  ///
  /// Four things are checked: the shape vector is as long as the point
  /// vector, every arc index names a stored arc, the vertices that refer
  /// to one arc form a contiguous run whose ends carry the right roles,
  /// and the arcs a vertex refers to appear in the arc vector in chain
  /// order. The last is the invariant `Reverse` depends on and KiCad's
  /// `Replace( int, int, const SHAPE_LINE_CHAIN& )` breaks (erratum E7).
  ///
  /// Deliberately **not** checked: that an arc's stored endpoints equal
  /// the chain points at the ends of its run. `SetPoint` moves a vertex
  /// out from under an arc that survives (see its documentation), and a
  /// run rebuilt through `ConstructFromStartEndCenter` keeps the points it
  /// was given, so the agreement is not an invariant in KiCad either.
  fn check_invariants(&self) {
    if !cfg!(debug_assertions) {
      return;
    }

    assert_eq!(
      self.points.len(),
      self.shapes.len(),
      "a chain holds one shape entry per point"
    );

    for entry in &self.shapes {
      match *entry {
        ArcRef::Plain => {}
        ArcRef::On { arc, .. } => {
          assert!(arc < self.arcs.len(), "arc index {arc} is out of range");
        }
        ArcRef::Shared { ends, starts } => {
          assert!(
            ends < self.arcs.len() && starts < self.arcs.len(),
            "arc index out of range at a shared point"
          );
        }
      }
    }

    let point_count = self.points.len();
    let mut run_start_of: Vec<Option<usize>> = vec![None; self.arcs.len()];

    // Each arc's vertices form one run, and the run's ends carry the roles
    // the predicates read. A closed chain's run may cross the seam, which
    // is exactly the shape `mergeFirstLastPointIfNeeded` builds when it
    // folds a duplicated endpoint onto vertex zero.
    for (arc, run_start) in run_start_of.iter_mut().enumerate() {
      let touches: Vec<bool> = self
        .shapes
        .iter()
        .map(|entry| match *entry {
          ArcRef::Plain => false,
          ArcRef::On { arc: on, .. } => on == arc,
          ArcRef::Shared { ends, starts } => ends == arc || starts == arc,
        })
        .collect();

      let mut starts: Vec<usize> = Vec::new();

      for index in 0..point_count {
        if !touches[index] {
          continue;
        }

        let previous_touches = if index > 0 {
          touches[index - 1]
        } else {
          self.closed && touches[point_count - 1]
        };

        if !previous_touches {
          starts.push(index);
        }
      }

      if touches.iter().all(|touched| !touched) {
        // An orphan, which `live_arcs` never yields.
        continue;
      }

      if starts.is_empty() {
        // Every vertex is on this arc and the chain is closed, so the run
        // has no beginning; that is a full circle stored as one arc.
        assert!(
          self.closed && touches.iter().all(|touched| *touched),
          "arc {arc} is referred to by a broken run of vertices"
        );
        *run_start = Some(0);
        continue;
      }

      assert_eq!(
        starts.len(),
        1,
        "arc {arc} is referred to by {} separate runs of vertices",
        starts.len()
      );

      let first = starts[0];
      let mut last = first;

      while touches[(last + 1) % point_count]
        && (last + 1) % point_count != first
      {
        last = (last + 1) % point_count;
      }

      assert!(
        self.shapes[first].starts_an_arc(),
        "the first vertex of arc {arc} does not start it"
      );
      // A run of one vertex cannot both start and end its arc, and
      // `splitArc` produces exactly that when it shortens an arc down to
      // its own first point (`shape_line_chain.cpp:352`). KiCad's
      // predicates read the same way there: the vertex is the arc's `P0`
      // and `IsArcSegment` is false, so it is neither a start nor an end.
      assert!(
        first == last || self.shapes[last].ends_an_arc(),
        "the last vertex of arc {arc} does not end it"
      );

      *run_start = Some(first);
    }

    // Chain order: the arcs appear in the arc vector in the order their
    // runs start. Orphans are skipped, nothing being able to reach them.
    let mut previous: Option<usize> = None;

    for first in run_start_of.iter().flatten() {
      if let Some(previous) = previous {
        assert!(
          previous <= *first,
          "the arcs are not in chain order: {previous} then {first}"
        );
      }

      previous = Some(*first);
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
  use proptest::prelude::*;

  use super::*;
  use crate::geometry::math::Degrees;

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

  /// Mirror of `ReplaceChain`, `test_shape_line_chain.cpp:1216`. The
  /// KiCad case carries no arcs; the arc side of the same member is
  /// `replace_with_chain_splices_arcs_into_chain_order_erratum_e7`.
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

    assert_eq!(chain.nearest_point(point(20, 5), true), Some(point(10, 5)));
    assert_eq!(chain.nearest_point(point(-5, -5), true), Some(point(0, 0)));

    // A chain of one point answers with that point, an empty one with
    // nothing.
    let single = LineChain::from_slice(&[point(7, 7)], false);

    assert_eq!(single.nearest_point(point(0, 0), true), Some(point(7, 7)));
    assert_eq!(LineChain::new().nearest_point(point(0, 0), true), None);
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

  // -----------------------------------------------------------------
  // Arcs
  // -----------------------------------------------------------------

  /// The accuracy every arc case of `test_shape_line_chain.cpp` appends
  /// at, `ARC_HIGH_DEF` (`include/base_units.h:137`).
  const ARC_HIGH_DEF: i32 = ShapeArc::DEFAULT_ACCURACY_FOR_PCB;

  /// KiCad's `GEOM_TEST::IsOutlineValid`,
  /// `qa/tests/libs/kimath/geometry/geom_test_utils.h:206`: every vertex
  /// that claims an arc lies on it, each arc is referred to by one
  /// contiguous run, and each run begins and ends on the arc's own
  /// endpoints.
  fn is_outline_valid(chain: &LineChain) -> bool {
    if chain.point_count() > 0 && !chain.is_closed() && chain.is_shared_pt(0) {
      return false;
    }

    let mut previous: Option<usize> = None;
    let mut tested: Vec<usize> = Vec::new();

    for index in 0..chain.point_count() {
      let current = chain.arc_index(index);

      if let Some(arc_index) = current {
        if previous != current && tested.contains(&arc_index) {
          return false;
        }

        let Some(arc) = chain.arc(arc_index) else {
          return false;
        };

        if arc
          .collide_point(chain.point(index), ShapeArc::DEFAULT_ACCURACY_FOR_PCB)
          .is_none()
        {
          return false;
        }

        tested.push(arc_index);
      }

      if previous != current {
        if let Some(previous_index) = previous {
          let to_test = if chain.is_shared_pt(index) {
            chain.point(index)
          } else {
            chain.point(index - 1)
          };
          let Some(arc) = chain.arc(previous_index) else {
            return false;
          };

          if arc.end() != to_test {
            return false;
          }
        }

        if let Some(arc_index) = current {
          let Some(arc) = chain.arc(arc_index) else {
            return false;
          };

          if arc.start() != chain.point(index) {
            return false;
          }
        }
      }

      previous = current;
    }

    true
  }

  /// KiCad's `SLC_CASES` fixture,
  /// `qa/tests/libs/kimath/geometry/test_shape_line_chain.cpp:34` to
  /// `:119`.
  struct SlcCases {
    circle_one_arc: LineChain,
    circle_two_arcs: LineChain,
    arcs_coincident: LineChain,
    arcs_coincident_closed: LineChain,
    arcs_independent: LineChain,
    duplicate_arcs: LineChain,
    arcs_and_seg_mixed: LineChain,
    arc_and_point: LineChain,
    seg_and_arc_coincident: LineChain,
    empty_chain: LineChain,
    one_point: LineChain,
    two_points: LineChain,
    three_points: LineChain,
  }

  impl SlcCases {
    fn new() -> Self {
      let arc_circle = ShapeArc::new(
        point(183_450_000, 128_360_000),
        point(183_850_000, 128_360_000),
        point(183_450_000, 128_360_000),
        0,
      );
      let arc_0a = ShapeArc::new(
        point(183_450_000, 128_360_000),
        point(183_650_000, 128_560_000),
        point(183_850_000, 128_360_000),
        0,
      );
      let arc_0b = ShapeArc::new(
        point(183_850_000, 128_360_000),
        point(183_650_000, 128_160_000),
        point(183_450_000, 128_360_000),
        0,
      );
      let arc_1 = ShapeArc::new(
        point(183_850_000, 128_360_000),
        point(183_638_550, 128_640_305),
        point(183_500_000, 129_204_974),
        0,
      );
      let arc_2 = ShapeArc::new(
        point(283_450_000, 228_360_000),
        point(283_650_000, 228_560_000),
        point(283_850_000, 228_360_000),
        0,
      );
      let arc_3 = ShapeArc::new(
        point(0, 0),
        point(24_142_136, 10_000_000),
        point(0, 20_000_000),
        0,
      );

      let mut circle_one_arc = LineChain::new();

      circle_one_arc.append_arc(&arc_circle, ARC_HIGH_DEF);
      circle_one_arc.set_closed(true);

      let mut circle_two_arcs = LineChain::new();

      circle_two_arcs.append_arc(&arc_0a, ARC_HIGH_DEF);
      circle_two_arcs.append_arc(&arc_0b, ARC_HIGH_DEF);
      circle_two_arcs.set_closed(true);

      let mut arcs_coincident = LineChain::new();

      arcs_coincident.append_arc(&arc_0a, ARC_HIGH_DEF);
      arcs_coincident.append_arc(&arc_1, ARC_HIGH_DEF);

      let mut arcs_coincident_closed = arcs_coincident.clone();

      arcs_coincident_closed.set_closed(true);

      let mut arcs_independent = LineChain::new();

      arcs_independent.append_arc(&arc_0a, ARC_HIGH_DEF);
      arcs_independent.append_arc(&arc_2, ARC_HIGH_DEF);

      let mut duplicate_arcs = arcs_coincident.clone();

      duplicate_arcs.append_arc(&arc_1, ARC_HIGH_DEF);

      let mut arc_and_point = LineChain::new();

      arc_and_point.append_arc(&arc_0a, ARC_HIGH_DEF);
      arc_and_point.append(point(233_450_000, 228_360_000));

      let mut arcs_and_seg_mixed = arc_and_point.clone();

      arcs_and_seg_mixed.append_arc(&arc_2, ARC_HIGH_DEF);

      let mut one_point = LineChain::new();

      one_point.append(point(233_450_000, 228_360_000));

      let mut two_points = one_point.clone();

      two_points.append(point(263_450_000, 258_360_000));

      let mut three_points = two_points.clone();

      three_points.append(point(263_450_000, 308_360_000));

      let mut seg_and_arc_coincident = LineChain::new();

      seg_and_arc_coincident.append(point(0, 20_000_000));
      seg_and_arc_coincident.append_arc(&arc_3, ARC_HIGH_DEF);

      Self {
        circle_one_arc,
        circle_two_arcs,
        arcs_coincident,
        arcs_coincident_closed,
        arcs_independent,
        duplicate_arcs,
        arcs_and_seg_mixed,
        arc_and_point,
        seg_and_arc_coincident,
        empty_chain: LineChain::new(),
        one_point,
        two_points,
        three_points,
      }
    }
  }

  /// Mirror of `ShapeCount`, `test_shape_line_chain.cpp:599`.
  #[test]
  fn shape_count_counts_a_whole_arc_as_one() {
    let cases = SlcCases::new();

    assert_eq!(cases.circle_one_arc.shape_count(), 1);
    assert_eq!(cases.circle_two_arcs.shape_count(), 2);
    assert_eq!(cases.arcs_coincident.shape_count(), 2);
    assert_eq!(cases.arcs_coincident_closed.shape_count(), 3);
    assert_eq!(cases.duplicate_arcs.shape_count(), 4);
    assert_eq!(cases.arc_and_point.shape_count(), 2);
    assert_eq!(cases.arcs_and_seg_mixed.shape_count(), 4);
    assert_eq!(cases.seg_and_arc_coincident.shape_count(), 2);
    assert_eq!(cases.empty_chain.shape_count(), 0);
    assert_eq!(cases.one_point.shape_count(), 0);
    assert_eq!(cases.two_points.shape_count(), 1);
    assert_eq!(cases.three_points.shape_count(), 2);
  }

  /// Mirror of `NextShape`, `test_shape_line_chain.cpp:616`.
  ///
  /// KiCad's rows that pass a negative or out of range index are written
  /// here through [`LineChain::normalize_index`], which is where this port
  /// puts that normalisation.
  #[test]
  fn next_shape_walks_one_whole_shape_at_a_time() {
    let cases = SlcCases::new();

    assert_eq!(cases.circle_one_arc.next_shape(0), None);

    assert_eq!(cases.circle_two_arcs.next_shape(0), Some(8));
    assert_eq!(cases.circle_two_arcs.next_shape(8), None);

    assert_eq!(cases.arcs_coincident.next_shape(0), Some(8));
    assert_eq!(cases.arcs_coincident.next_shape(8), None);

    assert_eq!(cases.arcs_coincident_closed.next_shape(0), Some(8));
    assert_eq!(cases.arcs_coincident_closed.next_shape(8), Some(13));
    assert_eq!(cases.arcs_coincident_closed.next_shape(13), None);

    assert_eq!(cases.arcs_independent.next_shape(0), Some(8));
    assert_eq!(cases.arcs_independent.next_shape(8), Some(9));
    assert_eq!(cases.arcs_independent.next_shape(9), None);

    assert_eq!(cases.duplicate_arcs.next_shape(0), Some(8));
    assert_eq!(cases.duplicate_arcs.next_shape(8), Some(13));
    assert_eq!(cases.duplicate_arcs.next_shape(13), Some(14));
    assert_eq!(cases.duplicate_arcs.next_shape(14), None);

    assert_eq!(cases.arc_and_point.next_shape(0), Some(8));
    assert_eq!(cases.arc_and_point.next_shape(8), None);

    assert_eq!(cases.arcs_and_seg_mixed.next_shape(0), Some(8));
    assert_eq!(cases.arcs_and_seg_mixed.next_shape(8), Some(9));
    assert_eq!(cases.arcs_and_seg_mixed.next_shape(9), Some(10));
    assert_eq!(cases.arcs_and_seg_mixed.next_shape(10), None);
    assert_eq!(cases.arcs_and_seg_mixed.next_shape(20), None);
    assert_eq!(cases.arcs_and_seg_mixed.normalize_index(-50), None);

    assert_eq!(cases.seg_and_arc_coincident.next_shape(0), Some(1));
    assert_eq!(cases.seg_and_arc_coincident.next_shape(1), None);

    assert_eq!(cases.empty_chain.next_shape(0), None);
    assert_eq!(cases.empty_chain.next_shape(1), None);
    assert_eq!(cases.empty_chain.normalize_index(-2), None);

    assert_eq!(cases.one_point.next_shape(0), None);
    assert_eq!(cases.one_point.next_shape(1), None);
    assert_eq!(
      cases
        .one_point
        .normalize_index(-1)
        .map(|index| cases.one_point.next_shape(index)),
      Some(None)
    );

    assert_eq!(cases.two_points.next_shape(0), None);
    assert_eq!(cases.two_points.next_shape(1), None);

    assert_eq!(cases.three_points.next_shape(0), Some(1));
    assert_eq!(cases.three_points.next_shape(1), None);
    assert_eq!(cases.three_points.next_shape(2), None);
  }

  /// Mirror of `AppendArc`, `test_shape_line_chain.cpp:675`.
  ///
  /// The six cases are all about the demotion rule at
  /// `shape_line_chain.cpp:1620`: an approximation of two points or fewer
  /// is not tagged as an arc.
  #[test]
  fn append_arc_demotes_an_arc_that_polygonises_to_two_points() {
    // Case 1: arc mid point nearly collinear.
    let mut chain = LineChain::new();

    chain.append_arc(
      &ShapeArc::new(point(100_000, 0), point(0, 2499), point(-100_000, 0), 0),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&chain));
    assert_eq!(chain.arc_count(), 0);
    assert_eq!(chain.point_count(), 2);
    assert_eq!(chain.point(0), point(100_000, 0));
    assert_eq!(chain.point(1), point(-100_000, 0));

    // Case 2: a large circle.
    let mut chain = LineChain::new();

    chain.append_arc(
      &ShapeArc::new(point(100_000, 0), point(0, 0), point(100_000, 0), 0),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&chain));
    assert_eq!(chain.arc_count(), 1);
    assert_eq!(chain.point_count(), 10);
    assert_eq!(chain.point(0), point(100_000, 0));
    assert_eq!(chain.point(9), point(100_000, 0));

    // Case 3: a circle small enough to approximate to a point.
    let mut chain = LineChain::new();

    chain.append_arc(
      &ShapeArc::new(point(2499, 0), point(0, 0), point(2499, 0), 0),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&chain));
    assert_eq!(chain.arc_count(), 0);
    assert_eq!(chain.point_count(), 1);
    assert_eq!(chain.point(0), point(2499, 0));

    // Case 3 again in KiCad's numbering: a small arc, approximated to a
    // segment.
    let mut chain = LineChain::new();

    chain.append_arc(
      &ShapeArc::new(point(1767, 0), point(2499, 2499), point(0, 1767), 0),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&chain));
    assert_eq!(chain.arc_count(), 0);
    assert_eq!(chain.point_count(), 2);
    assert_eq!(chain.point(0), point(1767, 0));
    assert_eq!(chain.point(1), point(0, 1767));

    // Case 4: a null arc, all three points coincident.
    let mut chain = LineChain::new();

    chain.append_arc(
      &ShapeArc::new(point(2499, 0), point(2499, 0), point(2499, 0), 0),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&chain));
    assert_eq!(chain.arc_count(), 0);
    assert_eq!(chain.point_count(), 1);
    assert_eq!(chain.point(0), point(2499, 0));

    // Case 5: an infinite radius, all three points very close.
    let mut chain = LineChain::new();

    chain.append_arc(
      &ShapeArc::new(point(2499, 0), point(2500, 0), point(2501, 0), 0),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&chain));
    assert_eq!(chain.arc_count(), 0);
    assert_eq!(chain.point_count(), 2);
    assert_eq!(chain.point(0), point(2499, 0));
    assert_eq!(chain.point(1), point(2501, 0));

    // Case 6: a large radius, all three points very close.
    let mut chain = LineChain::new();

    chain.append_arc(
      &ShapeArc::new(point(-100_000, 0), point(0, 1), point(100_000, 0), 0),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&chain));
    assert_eq!(chain.arc_count(), 0);
    assert_eq!(chain.point_count(), 2);
    assert_eq!(chain.point(0), point(-100_000, 0));
    assert_eq!(chain.point(1), point(100_000, 0));
  }

  /// Mirror of `ArcWrappingToStartSharedPoints`,
  /// `test_shape_line_chain.cpp:764`.
  ///
  /// This is the case `fixIndicesRotation` and the closing half of
  /// `mergeFirstLastPointIfNeeded` exist for.
  #[test]
  fn arc_wrapping_to_start_shared_points() {
    let arc_1 = ShapeArc::new(
      point(100_000, 0),
      point(0, 100_000),
      point(-100_000, 0),
      0,
    );
    let arc_2 = ShapeArc::new(
      point(-100_000, 0),
      point(0, -100_000),
      point(100_000, 0),
      0,
    );
    let mut chain = LineChain::new();

    chain.append_arc(&arc_1, ARC_HIGH_DEF);
    chain.append_arc(&arc_2, ARC_HIGH_DEF);
    assert_eq!(chain.point_count(), 13);

    // Open: vertex zero is not shared yet, so it cannot end an arc.
    assert!(!chain.is_shared_pt(0));
    assert!(!chain.is_arc_end(0));
    assert!(chain.is_arc_start(0));

    // Vertex six is the shared point in the middle.
    assert!(chain.is_shared_pt(6));
    assert!(chain.is_arc_end(6));
    assert!(chain.is_arc_start(6));

    let end_index = chain.point_count() - 1;

    assert!(!chain.is_shared_pt(end_index));
    assert!(chain.is_arc_end(end_index));
    assert!(!chain.is_arc_start(end_index));

    for index in 0..chain.point_count() {
      assert!(chain.is_pt_on_arc(index));
    }

    // Closed: the duplicated endpoint folds onto vertex zero, which
    // becomes the seam's shared point.
    chain.set_closed(true);
    assert_eq!(chain.point_count(), 12);

    assert!(chain.is_shared_pt(0));
    assert!(chain.is_arc_end(0));
    assert!(chain.is_arc_start(0));

    assert!(chain.is_shared_pt(6));
    assert!(chain.is_arc_end(6));
    assert!(chain.is_arc_start(6));

    let end_index = chain.point_count() - 1;

    assert!(!chain.is_shared_pt(end_index));
    assert!(!chain.is_arc_end(end_index));
    assert!(!chain.is_arc_start(end_index));
  }

  /// One row of KiCad's `remove_shape_cases` table,
  /// `test_shape_line_chain.cpp:476`.
  struct RemoveShapeCase {
    name: &'static str,
    chain: LineChain,
    shape_count: usize,
    arc_count: usize,
    remove_index: isize,
    expected_shape_count: usize,
    expected_arc_count: usize,
  }

  /// KiCad's `remove_shape_cases`, `test_shape_line_chain.cpp:487` to
  /// `:554`.
  fn remove_shape_cases() -> Vec<RemoveShapeCase> {
    let row = |name: &'static str,
               chain: LineChain,
               shape_count: usize,
               arc_count: usize,
               remove_index: isize,
               expected_shape_count: usize,
               expected_arc_count: usize| RemoveShapeCase {
      name,
      chain,
      shape_count,
      arc_count,
      remove_index,
      expected_shape_count,
      expected_arc_count,
    };
    let case = SlcCases::new;

    vec![
      row(
        "Circle1Arc - 1st arc - index on start",
        case().circle_one_arc,
        1,
        1,
        0,
        0,
        0,
      ),
      row(
        "Circle1Arc - 1st arc - index on mid",
        case().circle_one_arc,
        1,
        1,
        8,
        0,
        0,
      ),
      row(
        "Circle1Arc - 1st arc - index on end",
        case().circle_one_arc,
        1,
        1,
        14,
        0,
        0,
      ),
      row(
        "Circle1Arc - 1st arc - index on -1",
        case().circle_one_arc,
        1,
        1,
        -1,
        0,
        0,
      ),
      row(
        "Circle1Arc - invalid index",
        case().circle_one_arc,
        1,
        1,
        15,
        1,
        1,
      ),
      row(
        "Circle2Arcs - 1st arc - index on start",
        case().circle_two_arcs,
        2,
        2,
        0,
        2,
        1,
      ),
      row(
        "Circle2Arcs - 1st arc - index on mid",
        case().circle_two_arcs,
        2,
        2,
        3,
        2,
        1,
      ),
      row(
        "Circle2Arcs - 1st arc - index on end",
        case().circle_two_arcs,
        2,
        2,
        7,
        2,
        1,
      ),
      row(
        "Circle2Arcs - 2nd arc - index on start",
        case().circle_two_arcs,
        2,
        2,
        8,
        2,
        1,
      ),
      row(
        "Circle2Arcs - 2nd arc - index on mid",
        case().circle_two_arcs,
        2,
        2,
        11,
        2,
        1,
      ),
      row(
        "Circle2Arcs - 2nd arc - index on end",
        case().circle_two_arcs,
        2,
        2,
        15,
        2,
        1,
      ),
      row(
        "Circle2Arcs - 2nd arc - index on -1",
        case().circle_two_arcs,
        2,
        2,
        -1,
        2,
        1,
      ),
      row(
        "Circle2Arcs - invalid index",
        case().circle_two_arcs,
        2,
        2,
        16,
        2,
        2,
      ),
      row(
        "ArcsCoinc. - 1st arc - idx on start",
        case().arcs_coincident,
        2,
        2,
        0,
        1,
        1,
      ),
      row(
        "ArcsCoinc. - 1st arc - idx on mid",
        case().arcs_coincident,
        2,
        2,
        3,
        1,
        1,
      ),
      row(
        "ArcsCoinc. - 1st arc - idx on end",
        case().arcs_coincident,
        2,
        2,
        7,
        1,
        1,
      ),
      row(
        "ArcsCoinc. - 2nd arc - idx on start",
        case().arcs_coincident,
        2,
        2,
        8,
        1,
        1,
      ),
      row(
        "ArcsCoinc. - 2nd arc - idx on mid",
        case().arcs_coincident,
        2,
        2,
        10,
        1,
        1,
      ),
      row(
        "ArcsCoinc. - 2nd arc - idx on end",
        case().arcs_coincident,
        2,
        2,
        13,
        1,
        1,
      ),
      row(
        "ArcsCoinc. - 2nd arc - idx on -1",
        case().arcs_coincident,
        2,
        2,
        -1,
        1,
        1,
      ),
      row(
        "ArcsCoinc. - invalid idx",
        case().arcs_coincident,
        2,
        2,
        14,
        2,
        2,
      ),
      row(
        "A.Co.Closed - 1st arc - idx on start",
        case().arcs_coincident_closed,
        3,
        2,
        1,
        2,
        1,
      ),
      row(
        "A.Co.Closed - 1st arc - idx on mid",
        case().arcs_coincident_closed,
        3,
        2,
        3,
        2,
        1,
      ),
      row(
        "A.Co.Closed - 1st arc - idx on end",
        case().arcs_coincident_closed,
        3,
        2,
        7,
        2,
        1,
      ),
      row(
        "A.Co.Closed - 2nd arc - idx on start",
        case().arcs_coincident_closed,
        3,
        2,
        8,
        2,
        1,
      ),
      row(
        "A.Co.Closed - 2nd arc - idx on mid",
        case().arcs_coincident_closed,
        3,
        2,
        10,
        2,
        1,
      ),
      row(
        "A.Co.Closed - 2nd arc - idx on end",
        case().arcs_coincident_closed,
        3,
        2,
        13,
        2,
        1,
      ),
      row(
        "A.Co.Closed - 2nd arc - idx on -1",
        case().arcs_coincident_closed,
        3,
        2,
        -1,
        2,
        1,
      ),
      row(
        "A.Co.Closed - invalid idx",
        case().arcs_coincident_closed,
        3,
        2,
        14,
        3,
        2,
      ),
      row(
        "ArcsIndep. - 1st arc - idx on start",
        case().arcs_independent,
        3,
        2,
        0,
        1,
        1,
      ),
      row(
        "ArcsIndep. - 1st arc - idx on mid",
        case().arcs_independent,
        3,
        2,
        3,
        1,
        1,
      ),
      row(
        "ArcsIndep. - 1st arc - idx on end",
        case().arcs_independent,
        3,
        2,
        8,
        1,
        1,
      ),
      row(
        "ArcsIndep. - 2nd arc - idx on start",
        case().arcs_independent,
        3,
        2,
        9,
        1,
        1,
      ),
      row(
        "ArcsIndep. - 2nd arc - idx on mid",
        case().arcs_independent,
        3,
        2,
        12,
        1,
        1,
      ),
      row(
        "ArcsIndep. - 2nd arc - idx on end",
        case().arcs_independent,
        3,
        2,
        17,
        1,
        1,
      ),
      row(
        "ArcsIndep. - 2nd arc - idx on -1",
        case().arcs_independent,
        3,
        2,
        -1,
        1,
        1,
      ),
      row(
        "ArcsIndep. - invalid idx",
        case().arcs_independent,
        3,
        2,
        18,
        3,
        2,
      ),
      row(
        "Dup.Arcs - 1st arc - idx on start",
        case().duplicate_arcs,
        4,
        3,
        0,
        3,
        2,
      ),
      row(
        "Dup.Arcs - 1st arc - idx on mid",
        case().duplicate_arcs,
        4,
        3,
        3,
        3,
        2,
      ),
      row(
        "Dup.Arcs - 1st arc - idx on end",
        case().duplicate_arcs,
        4,
        3,
        7,
        3,
        2,
      ),
      row(
        "Dup.Arcs - 2nd arc - idx on start",
        case().duplicate_arcs,
        4,
        3,
        8,
        3,
        2,
      ),
      row(
        "Dup.Arcs - 2nd arc - idx on mid",
        case().duplicate_arcs,
        4,
        3,
        10,
        3,
        2,
      ),
      row(
        "Dup.Arcs - 2nd arc - idx on end",
        case().duplicate_arcs,
        4,
        3,
        13,
        3,
        2,
      ),
      row(
        "Dup.Arcs - 3rd arc - idx on start",
        case().duplicate_arcs,
        4,
        3,
        14,
        2,
        2,
      ),
      row(
        "Dup.Arcs - 3rd arc - idx on mid",
        case().duplicate_arcs,
        4,
        3,
        17,
        2,
        2,
      ),
      row(
        "Dup.Arcs - 3rd arc - idx on end",
        case().duplicate_arcs,
        4,
        3,
        19,
        2,
        2,
      ),
      row(
        "Dup.Arcs - 3rd arc - idx on -1",
        case().duplicate_arcs,
        4,
        3,
        -1,
        2,
        2,
      ),
      row(
        "Dup.Arcs - invalid idx",
        case().duplicate_arcs,
        4,
        3,
        20,
        4,
        3,
      ),
      row(
        "Arcs Mixed - 1st arc - idx on start",
        case().arcs_and_seg_mixed,
        4,
        2,
        0,
        2,
        1,
      ),
      row(
        "Arcs Mixed - 1st arc - idx on mid",
        case().arcs_and_seg_mixed,
        4,
        2,
        3,
        2,
        1,
      ),
      row(
        "Arcs Mixed - 1st arc - idx on end",
        case().arcs_and_seg_mixed,
        4,
        2,
        8,
        2,
        1,
      ),
      row(
        "Arcs Mixed - Straight segment",
        case().arcs_and_seg_mixed,
        4,
        2,
        9,
        3,
        2,
      ),
      row(
        "Arcs Mixed - 2nd arc - idx on start",
        case().arcs_and_seg_mixed,
        4,
        2,
        10,
        2,
        1,
      ),
      row(
        "Arcs Mixed - 2nd arc - idx on mid",
        case().arcs_and_seg_mixed,
        4,
        2,
        14,
        2,
        1,
      ),
      row(
        "Arcs Mixed - 2nd arc - idx on end",
        case().arcs_and_seg_mixed,
        4,
        2,
        18,
        2,
        1,
      ),
      row(
        "Arcs Mixed - 2nd arc - idx on -1",
        case().arcs_and_seg_mixed,
        4,
        2,
        -1,
        2,
        1,
      ),
      row(
        "Arcs Mixed - invalid idx",
        case().arcs_and_seg_mixed,
        4,
        2,
        19,
        4,
        2,
      ),
    ]
  }

  /// KiCad's `RemoveShape( int )` takes a signed index; this port takes a
  /// `usize` and leaves the normalisation to
  /// [`LineChain::normalize_index`], so the table's negative and out of
  /// range rows are translated here.
  fn remove_shape_at(chain: &mut LineChain, index: isize) {
    // An index that names no vertex is a no operation in KiCad too
    // (`shape_line_chain.cpp:1385`).
    if let Some(normalized) = chain.normalize_index(index) {
      chain.remove_shape(normalized);
    }
  }

  /// Mirror of `RemoveShape`, `test_shape_line_chain.cpp:557`.
  #[test]
  fn remove_shape_removes_the_whole_arc() {
    for case in remove_shape_cases() {
      let mut chain = case.chain.clone();

      assert_eq!(chain.shape_count(), case.shape_count, "{}", case.name);
      assert_eq!(chain.arc_count(), case.arc_count, "{}", case.name);
      assert!(is_outline_valid(&chain), "{}", case.name);

      remove_shape_at(&mut chain, case.remove_index);

      assert_eq!(
        chain.shape_count(),
        case.expected_shape_count,
        "{}",
        case.name
      );
      assert_eq!(chain.arc_count(), case.expected_arc_count, "{}", case.name);
      assert!(is_outline_valid(&chain), "{}", case.name);
    }
  }

  /// Mirror of `RemoveShapeAfterSimplify`,
  /// `test_shape_line_chain.cpp:576`: the same table with a
  /// [`LineChain::simplify`] in the middle, which must change neither
  /// count.
  #[test]
  fn remove_shape_after_simplify_removes_the_whole_arc() {
    for case in remove_shape_cases() {
      let mut chain = case.chain.clone();

      assert!(is_outline_valid(&chain), "{}", case.name);
      assert_eq!(chain.shape_count(), case.shape_count, "{}", case.name);
      assert_eq!(chain.arc_count(), case.arc_count, "{}", case.name);

      chain.simplify(0);

      assert!(is_outline_valid(&chain), "{}", case.name);
      assert_eq!(chain.shape_count(), case.shape_count, "{}", case.name);
      assert_eq!(chain.arc_count(), case.arc_count, "{}", case.name);

      remove_shape_at(&mut chain, case.remove_index);

      assert!(is_outline_valid(&chain), "{}", case.name);
      assert_eq!(
        chain.shape_count(),
        case.expected_shape_count,
        "{}",
        case.name
      );
      assert_eq!(chain.arc_count(), case.expected_arc_count, "{}", case.name);
    }
  }

  /// The chain `Split` and `NearestPointPt` share,
  /// `test_shape_line_chain.cpp:824` and `:1188`.
  fn split_case_chain() -> (Seg, Seg, ShapeArc, LineChain) {
    let seg_1 = Seg::new(point(0, 100_000), point(50_000, 0));
    let seg_2 = Seg::new(point(200_000, 0), point(300_000, 0));
    // KiCad's `SHAPE_ARC( VECTOR2I( 200000, 0 ), VECTOR2I( 300000, 0 ),
    // ANGLE_180 )` is the centre, start, angle constructor
    // (`shape_arc.h:57`), not the start, end, angle one.
    let arc = ShapeArc::from_center_start_angle(
      point(200_000, 0),
      point(300_000, 0),
      Degrees::HALF_TURN,
      0,
    );
    let mut chain = LineChain::from_slice(&[seg_1.a, seg_1.b], false);

    chain.append_arc(&arc, ARC_HIGH_DEF);
    chain.append(seg_2.a);
    chain.append(seg_2.b);

    (seg_1, seg_2, arc, chain)
  }

  /// Mirror of `Split`, `test_shape_line_chain.cpp:822`.
  #[test]
  fn split_on_an_arc_segment_makes_two_arcs_sharing_a_point() {
    let (seg_1, _seg_2, arc, chain) = split_case_chain();

    assert_eq!(chain.point_count(), 11);
    assert!(is_outline_valid(&chain));

    // Case 1: a point that is not on the chain.
    let mut copy = chain.clone();

    assert_eq!(copy.split(point(400_000, 0)), None);
    assert_eq!(copy.point_count(), chain.point_count());
    assert_eq!(copy.arc_count(), chain.arc_count());

    // Case 2: close to the start of a segment.
    let mut copy = chain.clone();
    let split_point = seg_1.a + point(5, -10);

    assert_eq!(copy.split(split_point), Some(1));
    assert!(is_outline_valid(&copy));
    assert_eq!(copy.point(1), split_point);
    assert_eq!(copy.point_count(), chain.point_count() + 1);
    assert_eq!(copy.arc_count(), chain.arc_count());

    // Case 3: exactly on the segment.
    let mut copy = chain.clone();

    assert_eq!(copy.split(seg_1.b), Some(1));
    assert!(is_outline_valid(&copy));
    assert_eq!(copy.point(1), seg_1.b);
    assert_eq!(copy.point_count(), chain.point_count());
    assert_eq!(copy.arc_count(), chain.arc_count());

    // Case 4: exactly at the arc's start.
    let mut copy = chain.clone();

    assert_eq!(copy.split(arc.start()), Some(2));
    assert!(is_outline_valid(&copy));
    assert_eq!(copy.point(2), arc.start());
    assert_eq!(copy.point_count(), chain.point_count());
    assert_eq!(copy.arc_count(), chain.arc_count());

    // Case 5: close to the arc's start, which cuts the arc in two.
    let mut copy = chain.clone();
    let split_point = arc.start() + point(-10, 130);

    assert_eq!(copy.split(split_point), Some(3));
    assert!(is_outline_valid(&copy));
    assert_eq!(copy.point(3), split_point);
    assert!(copy.is_shared_pt(3));
    assert_eq!(copy.point_count(), chain.point_count() + 1);
    assert_eq!(copy.arc_count(), chain.arc_count() + 1);
  }

  /// Mirror of `NearestPointPt`, `test_shape_line_chain.cpp:1186`, which
  /// is the case KiCad's `aAllowInternalShapePoints` exists for.
  #[test]
  fn nearest_point_snaps_to_an_arc_endpoint_when_asked() {
    let (_seg_1, _seg_2, arc, chain) = split_case_chain();

    assert_eq!(chain.point_count(), 11);
    assert!(is_outline_valid(&chain));

    let near_start = point(297_553, 31_697);
    let near_end = point(139_709, 82_983);

    assert_eq!(chain.nearest_point(near_start, true), Some(near_start));
    assert_eq!(chain.nearest_point(near_start, false), Some(arc.start()));

    assert_eq!(chain.nearest_point(near_end, true), Some(near_end));
    assert_eq!(chain.nearest_point(near_end, false), Some(arc.end()));
  }

  /// Mirror of `Slice`, `test_shape_line_chain.cpp:896`, all ten cases.
  #[test]
  fn slice_cuts_whole_and_partial_arcs() {
    let target_segment = Seg::new(point(200_000, 0), point(300_000, 0));
    let first_arc = ShapeArc::from_center_start_angle(
      point(200_000, 0),
      point(300_000, 0),
      Degrees::HALF_TURN,
      0,
    );
    let second_arc = ShapeArc::from_center_start_angle(
      point(-200_000, -200_000),
      point(-300_000, -100_000),
      -Degrees::HALF_TURN,
      0,
    );
    let tolerance = ShapeArc::DEFAULT_ACCURACY_FOR_PCB;

    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(0, 100_000), point(100_000, 0)],
      false,
    );

    assert_eq!(chain.point_count(), 3);
    chain.append_arc(&first_arc, ARC_HIGH_DEF);
    assert_eq!(chain.point_count(), 10);
    chain.append(target_segment.a);
    chain.append(target_segment.b);
    assert_eq!(chain.point_count(), 12);
    chain.append_arc(&second_arc, ARC_HIGH_DEF);
    assert_eq!(chain.point_count(), 20);
    assert!(is_outline_valid(&chain));

    // Case 1: start at an arc endpoint, finish in the middle of an arc.
    let sliced = chain.slice_with_max_error(9, 18, ARC_HIGH_DEF).unwrap();

    assert!(is_outline_valid(&sliced));
    assert_eq!(sliced.arc_count(), 1);

    let expected = ShapeArc::from_start_end_center(
      second_arc.start(),
      chain.point(18),
      second_arc.center(),
      !second_arc.is_ccw(),
      0,
    );
    let sliced_arc = sliced.arc(0).unwrap();

    assert_eq!(sliced_arc.start(), expected.start());
    assert!(
      sliced_arc
        .collide_point(expected.arc_mid(), tolerance)
        .is_some()
    );
    assert!(
      sliced_arc
        .collide_point(expected.end(), tolerance)
        .is_some()
    );
    assert_eq!(sliced.point_count(), 10);
    assert_eq!(sliced.point(0), first_arc.end());
    assert_eq!(sliced.point(1), target_segment.a);
    assert_eq!(sliced.point(2), target_segment.b);
    assert_eq!(sliced.point(3), expected.start());
    assert!(sliced.is_arc_start(3));

    for index in 4..=8 {
      assert!(!sliced.is_arc_start(index));
    }

    for index in 3..=7 {
      assert!(!sliced.is_arc_end(index));
    }

    assert!(sliced.is_arc_end(9));
    assert_eq!(sliced.point(9), expected.end());

    // Case 2: start in the middle of an arc, finish at an arc start point.
    let sliced = chain.slice_with_max_error(5, 12, ARC_HIGH_DEF).unwrap();

    assert!(is_outline_valid(&sliced));
    assert_eq!(sliced.arc_count(), 1);

    let expected = ShapeArc::from_start_end_center(
      chain.point(5),
      first_arc.end(),
      first_arc.center(),
      !first_arc.is_ccw(),
      0,
    );
    let sliced_arc = sliced.arc(0).unwrap();

    assert_eq!(sliced_arc.end(), expected.end());
    assert!(
      sliced_arc
        .collide_point(expected.arc_mid(), tolerance)
        .is_some()
    );
    assert!(
      sliced_arc
        .collide_point(expected.start(), tolerance)
        .is_some()
    );
    assert_eq!(sliced.point_count(), 8);
    assert_eq!(sliced.point(0), expected.start());
    assert!(sliced.is_arc_start(0));

    for index in 1..=4 {
      assert!(!sliced.is_arc_start(index));
    }

    for index in 0..=3 {
      assert!(!sliced.is_arc_end(index));
    }

    assert!(sliced.is_arc_end(4));
    assert_eq!(sliced.point(4), expected.end());
    assert_eq!(sliced.point(5), target_segment.a);
    assert_eq!(sliced.point(6), target_segment.b);
    assert_eq!(sliced.point(7), second_arc.start());

    // Case 3: a whole arc and nothing else.
    let sliced = chain.slice_with_max_error(3, 9, ARC_HIGH_DEF).unwrap();

    assert!(is_outline_valid(&sliced));
    assert_eq!(sliced.arc_count(), 1);

    let sliced_arc = sliced.arc(0).unwrap();

    assert_eq!(first_arc.end(), sliced_arc.end());
    assert_eq!(first_arc.arc_mid(), sliced_arc.arc_mid());
    assert_eq!(sliced.point_count(), 7);
    assert_eq!(sliced.point(0), sliced_arc.start());
    assert!(sliced.is_arc_start(0));

    for index in 1..=6 {
      assert!(!sliced.is_arc_start(index));
    }

    for index in 0..=5 {
      assert!(!sliced.is_arc_end(index));
    }

    assert!(sliced.is_arc_end(6));
    assert_eq!(sliced.point(6), sliced_arc.end());

    // Case 4: a whole arc and the straight segments up to the next arc.
    let sliced = chain.slice_with_max_error(3, 12, ARC_HIGH_DEF).unwrap();

    assert!(is_outline_valid(&sliced));
    assert_eq!(sliced.arc_count(), 1);

    let sliced_arc = sliced.arc(0).unwrap();

    assert_eq!(first_arc.end(), sliced_arc.end());
    assert_eq!(first_arc.arc_mid(), sliced_arc.arc_mid());
    assert_eq!(sliced.point_count(), 10);
    assert_eq!(sliced.point(0), sliced_arc.start());
    assert!(sliced.is_arc_start(0));
    assert!(sliced.is_arc_end(6));
    assert_eq!(sliced.point(6), sliced_arc.end());
    assert_eq!(sliced.point(7), target_segment.a);
    assert_eq!(sliced.point(8), target_segment.b);
    assert_eq!(sliced.point(9), second_arc.start());

    // Case 5: a chain that ends in an arc and then a point.
    let mut copy = chain.clone();

    copy.append(point(400_000, 400_000));

    let last = copy.normalize_index(-1).unwrap();
    let sliced = copy.slice_with_max_error(11, last, ARC_HIGH_DEF).unwrap();

    assert!(is_outline_valid(&sliced));
    assert_eq!(sliced.last_point(), Some(point(400_000, 400_000)));

    // Case 6: a whole chain of one point.
    let one_point = SlcCases::new().one_point;
    let last = one_point.normalize_index(-1).unwrap();
    let sliced = one_point
      .slice_with_max_error(0, last, ARC_HIGH_DEF)
      .unwrap();

    assert_eq!(sliced.point_count(), 1);
    assert_eq!(sliced.point(0), point(233_450_000, 228_360_000));

    // Case 7: a whole chain of two points.
    let two_points = SlcCases::new().two_points;
    let last = two_points.normalize_index(-1).unwrap();
    let sliced = two_points
      .slice_with_max_error(0, last, ARC_HIGH_DEF)
      .unwrap();

    assert_eq!(sliced.point_count(), 2);
    assert_eq!(sliced.point(0), point(233_450_000, 228_360_000));
    assert_eq!(sliced.point(1), point(263_450_000, 258_360_000));

    // Case 8: the whole second arc and nothing else.
    let sliced = chain.slice_with_max_error(12, 19, ARC_HIGH_DEF).unwrap();

    assert!(is_outline_valid(&sliced));
    assert_eq!(sliced.arc_count(), 1);

    let sliced_arc = sliced.arc(0).unwrap();

    assert_eq!(second_arc.end(), sliced_arc.end());
    assert_eq!(second_arc.arc_mid(), sliced_arc.arc_mid());
    assert_eq!(sliced.point_count(), 8);
    assert_eq!(sliced.point(0), sliced_arc.start());
    assert!(sliced.is_arc_start(0));
    assert!(sliced.is_arc_end(7));
    assert_eq!(sliced.point(7), sliced_arc.end());

    // Case 9: start in the middle of the second arc, finish at the end.
    let sliced = chain.slice_with_max_error(16, 19, ARC_HIGH_DEF).unwrap();

    assert!(is_outline_valid(&sliced));
    assert_eq!(sliced.arc_count(), 1);

    let expected = ShapeArc::from_start_end_center(
      chain.point(16),
      second_arc.end(),
      second_arc.center(),
      !second_arc.is_ccw(),
      0,
    );
    let sliced_arc = sliced.arc(0).unwrap();

    assert_eq!(sliced_arc.end(), expected.end());
    assert!(
      sliced_arc
        .collide_point(expected.arc_mid(), tolerance)
        .is_some()
    );
    assert!(
      sliced_arc
        .collide_point(expected.start(), tolerance)
        .is_some()
    );
    assert_eq!(sliced.point_count(), 4);
    assert_eq!(sliced.point(0), expected.start());
    assert!(sliced.is_arc_start(0));
    assert!(sliced.is_arc_end(3));
    assert_eq!(sliced.point(3), expected.end());

    // Case 10: a fresh chain of one arc, sliced from its middle to its end.
    let mut chain_10 = LineChain::new();

    chain_10.append_arc(&first_arc, ARC_HIGH_DEF);

    let sliced = chain_10.slice_with_max_error(3, 6, ARC_HIGH_DEF).unwrap();

    assert!(is_outline_valid(&sliced));
    assert_eq!(sliced.arc_count(), 1);

    let expected = ShapeArc::from_start_end_center(
      chain_10.point(3),
      first_arc.end(),
      first_arc.center(),
      !first_arc.is_ccw(),
      0,
    );
    let sliced_arc = sliced.arc(0).unwrap();

    assert_eq!(sliced_arc.end(), expected.end());
    assert!(
      sliced_arc
        .collide_point(expected.arc_mid(), tolerance)
        .is_some()
    );
    assert_eq!(sliced.point_count(), 4);
    assert_eq!(sliced.point(0), expected.start());
    assert!(sliced.is_arc_start(0));
    assert!(sliced.is_arc_end(3));
    assert_eq!(sliced.point(3), expected.end());
  }

  /// Mirror of `SimplifyWithArcs`, `test_shape_line_chain.cpp:1524`, all
  /// ten contexts.
  #[test]
  fn simplify_keeps_every_arc_and_still_collapses_straight_runs() {
    let hump = |start_x: i32, mid_x: i32, end_x: i32, mid_y: i32| {
      ShapeArc::new(point(start_x, 0), point(mid_x, mid_y), point(end_x, 0), 0)
    };

    // 1 segment, arc, 2 collinear segments.
    let mut original = LineChain::new();

    original.append(point(0, 0));
    original.append_arc(
      &hump(2_000_000, 2_500_000, 3_000_000, 500_000),
      ARC_HIGH_DEF,
    );
    original.append(point(4_000_000, 0));
    original.append(point(5_000_000, 0));
    assert!(is_outline_valid(&original));

    let before = original.point_count();
    let mut simplified = original.clone();

    simplified.simplify(0);
    assert_eq!(simplified.arc_count(), original.arc_count());
    assert!(simplified.point_count() < before);
    assert_eq!(simplified.arc(0).unwrap().start(), point(2_000_000, 0));
    assert_eq!(simplified.arc(0).unwrap().end(), point(3_000_000, 0));
    assert_eq!(simplified.find(point(4_000_000, 0), 0), None);
    assert!(simplified.find(point(3_000_000, 0), 0).is_some());

    // Arc, two collinear segments.
    let mut original = LineChain::new();

    original.append_arc(&hump(0, 1_000_000, 2_000_000, 500_000), ARC_HIGH_DEF);
    original.append(point(3_000_000, 0));
    original.append(point(4_000_000, 0));
    assert!(is_outline_valid(&original));

    let before = original.point_count();
    let mut simplified = original.clone();

    simplified.simplify(0);
    assert_eq!(simplified.arc_count(), 1);
    assert!(simplified.point_count() < before);
    assert!(is_outline_valid(&simplified));
    assert_eq!(simplified.find(point(3_000_000, 0), 0), None);
    assert_eq!(simplified.arc(0).unwrap().start(), point(0, 0));
    assert_eq!(simplified.arc(0).unwrap().end(), point(2_000_000, 0));

    // 2 collinear segments, arc, 2 collinear segments.
    let mut original = LineChain::new();

    original.append(point(0, 0));
    original.append(point(1_000_000, 0));
    original.append_arc(
      &hump(2_000_000, 2_500_000, 3_000_000, 500_000),
      ARC_HIGH_DEF,
    );
    original.append(point(4_000_000, 0));
    original.append(point(5_000_000, 0));
    assert!(is_outline_valid(&original));

    let before = original.point_count();
    let before_arcs = original.arc_count();
    let mut simplified = original.clone();

    simplified.simplify(0);
    assert_eq!(simplified.arc_count(), before_arcs);
    assert!(simplified.point_count() < before);
    assert!(is_outline_valid(&simplified));
    assert_eq!(simplified.arc(0).unwrap().start(), point(2_000_000, 0));
    assert_eq!(simplified.arc(0).unwrap().end(), point(3_000_000, 0));
    assert_eq!(simplified.find(point(4_000_000, 0), 0), None);
    assert!(simplified.find(point(5_000_000, 0), 0).is_some());

    // 2 collinear segments, arc.
    let mut original = LineChain::new();

    original.append(point(0, 0));
    original.append(point(1_000_000, 0));
    original.append_arc(
      &hump(2_000_000, 2_500_000, 3_000_000, 500_000),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&original));

    let before = original.point_count();
    let mut simplified = original.clone();

    simplified.simplify(0);
    assert_eq!(simplified.arc_count(), 1);
    assert!(simplified.point_count() < before);
    assert_eq!(simplified.find(point(1_000_000, 0), 0), None);
    assert!(is_outline_valid(&simplified));
    assert_eq!(simplified.arc(0).unwrap().start(), point(2_000_000, 0));
    assert_eq!(simplified.arc(0).unwrap().end(), point(3_000_000, 0));

    // Arc at the start, two collinear segments after it.
    let mut original = LineChain::new();

    original.append_arc(&hump(0, 1_000_000, 2_000_000, 500_000), ARC_HIGH_DEF);
    original.append(point(3_000_000, 0));
    original.append(point(4_000_000, 0));
    assert!(is_outline_valid(&original));

    let before = original.point_count();
    let before_arcs = original.arc_count();
    let mut simplified = original.clone();

    simplified.simplify(0);
    assert!(
      simplified.arc_count() == before_arcs
        || simplified.arc_count() == before_arcs - 1
    );
    assert!(simplified.point_count() < before);
    assert!(is_outline_valid(&simplified));

    // Tolerance semantics, zero against a small positive.
    let mut original = LineChain::new();

    original.append(point(0, 0));
    original.append(point(1_000_000, 1));
    original.append_arc(
      &hump(2_000_000, 2_500_000, 3_000_000, 500_000),
      ARC_HIGH_DEF,
    );
    original.append(point(4_000_000, 0));
    original.append(point(5_000_000, 0));
    assert!(is_outline_valid(&original));

    let before = original.point_count();
    let mut at_zero = original.clone();

    at_zero.simplify(0);
    assert_eq!(at_zero.point_count(), before - 1);

    let mut at_one = original.clone();

    at_one.simplify(1);
    assert_eq!(at_one.point_count(), before - 2);
    assert_eq!(at_one.arc_count(), original.arc_count());

    // Two adjacent arcs.
    let mut original = LineChain::new();

    original.append_arc(&hump(0, 1_000_000, 2_000_000, 500_000), ARC_HIGH_DEF);
    original.append_arc(
      &hump(2_000_000, 3_000_000, 4_000_000, 500_000),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&original));

    let before_arcs = original.arc_count();
    let mut simplified = original.clone();

    simplified.simplify(0);
    assert_eq!(simplified.arc_count(), before_arcs);
    assert!(is_outline_valid(&simplified));
    assert_eq!(
      simplified.arc(0).unwrap().end(),
      simplified.arc(1).unwrap().start()
    );
    assert_eq!(simplified.arc(0).unwrap().start(), point(0, 0));
    assert_eq!(simplified.arc(1).unwrap().end(), point(4_000_000, 0));

    // A segment, two arcs of opposite bulge, two collinear segments.
    let mut original = LineChain::new();

    original.append(point(-1_000_000, 0));
    original.append_arc(&hump(0, 500_000, 1_000_000, 500_000), ARC_HIGH_DEF);
    original.append_arc(
      &hump(1_000_000, 1_500_000, 2_000_000, -500_000),
      ARC_HIGH_DEF,
    );
    original.append(point(3_000_000, 0));
    original.append(point(4_000_000, 0));
    assert!(is_outline_valid(&original));

    let before = original.point_count();
    let before_arcs = original.arc_count();
    let mut simplified = original.clone();

    simplified.simplify(0);
    assert_eq!(simplified.arc_count(), before_arcs);
    assert!(simplified.point_count() < before);
    assert!(is_outline_valid(&simplified));
    assert!(simplified.find(point(-1_000_000, 0), 0).is_some());
    assert!(simplified.find(point(4_000_000, 0), 0).is_some());

    // Arc, a collinear point, arc.
    let mut original = LineChain::new();

    original.append_arc(&hump(0, 1_000_000, 2_000_000, 500_000), ARC_HIGH_DEF);
    original.append(point(2_500_000, 0));
    original.append_arc(
      &hump(3_000_000, 3_500_000, 4_000_000, 500_000),
      ARC_HIGH_DEF,
    );
    assert!(is_outline_valid(&original));

    let before = original.point_count();
    let before_arcs = original.arc_count();
    let mut simplified = original.clone();

    simplified.simplify(0);
    assert_eq!(simplified.arc_count(), before_arcs);
    assert!(simplified.point_count() < before);
    assert!(is_outline_valid(&simplified));
    assert_eq!(simplified.arc(0).unwrap().end(), point(2_000_000, 0));
    assert_eq!(simplified.arc(1).unwrap().start(), point(3_000_000, 0));
  }

  /// A chain shaped plain point, arc, plain point, arc, plain point, the
  /// smallest shape that lets one removal range swallow two whole arcs.
  fn two_arcs_between_plain_points() -> LineChain {
    let mut chain = LineChain::new();

    chain.append(point(-1_000_000, 0));
    chain.append_arc(
      &ShapeArc::new(
        point(0, 0),
        point(500_000, 500_000),
        point(1_000_000, 0),
        0,
      ),
      ARC_HIGH_DEF,
    );
    chain.append(point(2_000_000, 0));
    chain.append_arc(
      &ShapeArc::new(
        point(3_000_000, 0),
        point(3_500_000, 500_000),
        point(4_000_000, 0),
        0,
      ),
      ARC_HIGH_DEF,
    );
    chain.append(point(5_000_000, 0));
    chain
  }

  /// Erratum E6: `Remove`'s arc dropping iterates its index set in
  /// increasing order while `convertArc` renumbers everything above the
  /// index it just erased
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:1118` to `:1157`).
  ///
  /// Reproduced, because the behaviour is defined. A range covering two
  /// whole arcs erases the first and leaves the second behind with every
  /// reference to it gone, so the arc count drops by one rather than two.
  #[test]
  fn remove_range_erases_every_other_arc_of_the_range_erratum_e6() {
    let mut chain = two_arcs_between_plain_points();

    assert_eq!(chain.arc_count(), 2);
    assert_eq!(chain.live_arcs().count(), 2);

    let last = chain.point_count() - 1;

    chain.remove_range(1, last - 1);

    assert_eq!(chain.point_count(), 2);
    // Both arcs were inside the range. One entry survives, referred to by
    // nothing: that is the erratum.
    assert_eq!(chain.arc_count(), 1);
    assert_eq!(chain.live_arcs().count(), 0);
  }

  /// Erratum E13: an arc no vertex refers to still collides and still
  /// contributes length in KiCad, because `Collide` and the preview walk
  /// `ArcCount()` directly (`shape_line_chain.cpp:480`, `:873`).
  ///
  /// Fixed by construction: every consumer in this module goes through
  /// [`LineChain::live_arcs`], so the orphan erratum E6 leaves behind is
  /// invisible.
  #[test]
  fn an_orphaned_arc_neither_collides_nor_adds_length_erratum_e13() {
    let mut chain = two_arcs_between_plain_points();
    let last = chain.point_count() - 1;

    chain.remove_range(1, last - 1);

    assert_eq!(chain.arc_count(), 1);
    assert_eq!(chain.live_arcs().count(), 0);

    // What is left is the straight run from the first plain point to the
    // last, and nothing else.
    assert_eq!(chain.point_count(), 2);
    assert_eq!(chain.length(), 6_000_000);

    // The orphan's own geometry is around (3.5 mm, 0.5 mm), far from the
    // surviving segment, and a query there finds nothing.
    assert_eq!(chain.collide_point(point(3_500_000, 500_000), 1000), None);
    assert_eq!(
      chain.collide_seg(
        &Seg::new(point(3_500_000, 400_000), point(3_600_000, 600_000)),
        1000
      ),
      None
    );
  }

  /// Erratum E7: `Replace( int, int, const SHAPE_LINE_CHAIN& )` appends
  /// the incoming arcs at the end of `m_arcs` whatever position their
  /// points took (`libs/kimath/src/geometry/shape_line_chain.cpp:1071`),
  /// which breaks the chain order `Reverse` needs.
  ///
  /// Fixed: [`LineChain::replace_with_chain`] splices them into position.
  /// The incoming arc here lands before the arc that was already there, so
  /// KiCad would have numbered the two the other way round.
  #[test]
  fn replace_with_chain_splices_arcs_into_chain_order_erratum_e7() {
    let trailing_arc = ShapeArc::new(
      point(2_000_000, 0),
      point(2_500_000, 500_000),
      point(3_000_000, 0),
      0,
    );
    let incoming_arc = ShapeArc::new(
      point(500_000, 500_000),
      point(1_000_000, 1_000_000),
      point(1_500_000, 500_000),
      0,
    );
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(1_000_000, 0), point(2_000_000, 0)],
      false,
    );

    chain.append_arc(&trailing_arc, ARC_HIGH_DEF);
    assert_eq!(chain.arc_count(), 1);
    assert_eq!(chain.arc(0).unwrap().start(), trailing_arc.start());

    let mut incoming = LineChain::new();

    incoming.append_arc(&incoming_arc, ARC_HIGH_DEF);
    chain.replace_with_chain(1, 1, &incoming);

    assert_eq!(chain.arc_count(), 2);
    // Chain order: the arc whose points come first is the arc at index 0.
    assert_eq!(chain.arc(0).unwrap().start(), incoming_arc.start());
    assert_eq!(chain.arc(1).unwrap().start(), trailing_arc.start());
    assert!(is_outline_valid(&chain));

    // Which is what makes reversing twice an identity. With KiCad's order
    // the index remap at `:926` would have swapped the two arcs.
    let before = chain.clone();

    chain.reverse();
    chain.reverse();
    assert_eq!(chain, before);
  }

  /// Erratum E10: `Slice` copies points forward from a start inside an arc
  /// with no `aEndIndex` bound
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:1444`), so a range
  /// whose two ends are interior to the same arc runs on to that arc's
  /// end.
  ///
  /// Reproduced, because the behaviour is defined and
  /// `LINE::restoreUntouchedArcs` reaches it (`pns_line.cpp:285`).
  #[test]
  fn slice_inside_one_arc_runs_past_its_end_erratum_e10() {
    let arc = ShapeArc::from_center_start_angle(
      point(200_000, 0),
      point(300_000, 0),
      Degrees::HALF_TURN,
      0,
    );
    let mut chain = LineChain::new();

    chain.append_arc(&arc, ARC_HIGH_DEF);
    assert_eq!(chain.point_count(), 7);

    // Both ends of the range are interior to the one arc.
    let sliced = chain.slice_with_max_error(2, 4, ARC_HIGH_DEF).unwrap();

    // A bounded slice would have answered three points; this answers the
    // five from index two to the arc's end.
    assert_eq!(sliced.point_count(), 5);
    assert_eq!(sliced.point(0), chain.point(2));
    assert_eq!(sliced.last_point(), Some(chain.point(6)));
    assert_eq!(sliced.arc_count(), 1);
    assert_eq!(sliced.arc(0).unwrap().end(), arc.end());
  }

  /// Erratum E12: `Simplify2`'s colinear run checks the shape entries of
  /// the run's first two vertices and then never looks again
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:2968` to `:2974`), so
  /// a shallow arc can lose interior points while its entry survives.
  ///
  /// Reproduced, because the behaviour is defined. The arc below has a
  /// sagitta of three nanometres and is stored at an accuracy of one, so
  /// its approximation points sit within `SIMPLIFY2_TOLERANCE` of the
  /// chord and the run walks straight through them.
  #[test]
  fn simplify2_can_drop_an_arcs_interior_points_erratum_e12() {
    let mut chain = LineChain::new();

    chain.append(point(-2_000_000, 0));
    chain.append(point(-1_000_000, 0));
    chain.append_arc(
      &ShapeArc::new(point(0, 0), point(1_000_000, 3), point(2_000_000, 0), 0),
      1,
    );

    let before = chain.point_count();

    assert!(before > 4, "the shallow arc must survive the demotion rule");
    assert_eq!(chain.arc_count(), 1);

    chain.simplify2(true);

    // The arc entry is still there while its run has lost vertices.
    assert_eq!(chain.arc_count(), 1);
    assert!(
      chain.point_count() < before,
      "the colinear run should have eaten interior points of the arc"
    );
  }

  /// Erratum E14: the arc snapping in `NearestPoint` can advance `nearest`
  /// to `PointCount()` and then read the unchecked `ArcIndex` and `Arc`
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:2427` to `:2450`).
  ///
  /// Fixed by change 4 of note 09 section 11.1: both accessors return
  /// [`Option`], so the read is checked and the answer falls back to the
  /// unsnapped nearest point. The chain below is the only shape that
  /// reaches it, a closed chain whose last segment is an arc segment.
  #[test]
  fn nearest_point_snapping_stays_in_range_erratum_e14() {
    let chain = SlcCases::new().circle_one_arc;
    let last_segment = chain.segment_count() - 1;

    assert!(chain.is_closed());
    assert!(chain.is_arc_segment(last_segment));

    // A point just outside the circle, nearest to the last segment and
    // nearer to that segment's second endpoint than to its first, which is
    // what makes KiCad step one past the last vertex.
    let segment = chain.segment(last_segment);
    let probe = segment.b + point(2000, 2000);
    let nearest = chain.nearest_point(probe, false);

    assert_eq!(nearest, Some(segment.nearest_point_to_point(probe)));
  }

  /// Erratum E35: `IsArcEnd( 0 )` looks back at the last vertex whether
  /// the chain is closed or not
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:3286`).
  ///
  /// Reproduced, and it cannot change an answer. The look back asks
  /// `is_arc_segment` of the last vertex, and on an open chain there is no
  /// segment there, so the predicate is false whatever the last shape is.
  #[test]
  fn is_arc_end_at_vertex_zero_never_wraps_erratum_e35() {
    let chain = SlcCases::new().arcs_coincident;
    let last = chain.point_count() - 1;

    assert!(!chain.is_closed());
    // The chain ends on an arc, and its first vertex starts one.
    assert!(chain.is_arc_end(last));
    assert!(chain.is_arc_start(0));

    // The unconditional look back reaches the last vertex and finds no
    // segment leaving it.
    assert!(!chain.is_arc_segment(last));
    assert!(!chain.is_arc_end(0));
  }

  /// Erratum E5: KiCad's `Insert( size_t, const SHAPE_ARC&, int )` scans
  /// `m_shapes.rbegin()` to `m_shapes.rend() + aVertex` for the insertion
  /// position, which is past the reverse end for any non zero vertex
  /// (`libs/kimath/src/geometry/shape_line_chain.cpp:1674`).
  ///
  /// Fixed, an out of bounds read being outside what the milestone rule
  /// asks to reproduce. The insertion position is found by scanning
  /// forward from the insertion point, which puts the new arc in chain
  /// order for every vertex.
  #[test]
  fn insert_arc_finds_its_position_without_reading_past_the_end_erratum_e5() {
    let trailing_arc = ShapeArc::new(
      point(2_000_000, 0),
      point(2_500_000, 500_000),
      point(3_000_000, 0),
      0,
    );
    let inserted_arc = ShapeArc::new(
      point(500_000, 500_000),
      point(1_000_000, 1_000_000),
      point(1_500_000, 500_000),
      0,
    );
    let mut chain = LineChain::from_slice(
      &[point(0, 0), point(1_000_000, 0), point(2_000_000, 0)],
      false,
    );

    chain.append_arc(&trailing_arc, ARC_HIGH_DEF);
    chain.insert_arc(1, &inserted_arc, ARC_HIGH_DEF);

    assert_eq!(chain.arc_count(), 2);
    assert_eq!(chain.arc(0).unwrap().start(), inserted_arc.start());
    assert_eq!(chain.arc(1).unwrap().start(), trailing_arc.start());
    assert!(is_outline_valid(&chain));

    // The demotion rule of `Append( SHAPE_ARC )` applies here too, which
    // KiCad's `Insert` skips.
    let mut chain = LineChain::from_slice(&[point(0, 0), point(10, 0)], false);

    chain.insert_arc(
      1,
      &ShapeArc::new(point(2499, 0), point(2500, 0), point(2501, 0), 0),
      ARC_HIGH_DEF,
    );
    assert_eq!(chain.arc_count(), 0);
  }

  /// Two arcs that meet at one shared vertex.
  fn chain_with_two_arcs_sharing_a_point() -> LineChain {
    let mut chain = LineChain::new();

    chain.append_arc(
      &ShapeArc::new(
        point(0, 0),
        point(500_000, 500_000),
        point(1_000_000, 0),
        0,
      ),
      ARC_HIGH_DEF,
    );
    chain.append_arc(
      &ShapeArc::new(
        point(1_000_000, 0),
        point(1_500_000, -500_000),
        point(2_000_000, 0),
        0,
      ),
      ARC_HIGH_DEF,
    );
    chain
  }

  /// Two arcs with a straight run between them.
  fn chain_with_a_straight_run_between_two_arcs() -> LineChain {
    let mut chain = LineChain::new();

    chain.append_arc(
      &ShapeArc::new(
        point(0, 0),
        point(500_000, 500_000),
        point(1_000_000, 0),
        0,
      ),
      ARC_HIGH_DEF,
    );
    chain.append(point(2_000_000, 0));
    chain.append(point(3_000_000, 1_000_000));
    chain.append_arc(
      &ShapeArc::new(
        point(4_000_000, 0),
        point(4_500_000, 500_000),
        point(5_000_000, 0),
        0,
      ),
      ARC_HIGH_DEF,
    );
    chain
  }

  /// An arc at each end of an open chain, with plain vertices between.
  fn chain_with_an_arc_at_each_end() -> LineChain {
    let mut chain = chain_with_a_straight_run_between_two_arcs();

    // The helper above already starts and ends on an arc; make the
    // difference explicit by checking it here rather than in every caller.
    assert!(chain.is_arc_start(0));
    assert!(chain.is_arc_end(chain.point_count() - 1));
    chain.set_width(250_000);
    chain
  }

  /// One named edit for `every_mutator_leaves_the_arc_invariants_intact`.
  type Mutator = Box<dyn Fn(&mut LineChain)>;

  /// The three arc bearing shapes the invariant test walks.
  fn invariant_fixtures() -> Vec<(&'static str, LineChain)> {
    vec![
      (
        "two arcs sharing a point",
        chain_with_two_arcs_sharing_a_point(),
      ),
      (
        "a straight run between two arcs",
        chain_with_a_straight_run_between_two_arcs(),
      ),
      ("an arc at each end", chain_with_an_arc_at_each_end()),
    ]
  }

  /// The private `check_invariants` holds after every mutator.
  ///
  /// Change 2 of note 09 section 11.1 asks for exactly this: the shape
  /// vector stays parallel to the points, every arc index names a stored
  /// arc, each arc's vertices form one run with the right roles at its
  /// ends, and the arcs stay in chain order.
  #[test]
  fn every_mutator_leaves_the_arc_invariants_intact() {
    let extra = ShapeArc::new(
      point(9_000_000, 0),
      point(9_500_000, 500_000),
      point(10_000_000, 0),
      0,
    );

    for (name, original) in invariant_fixtures() {
      let last = original.point_count() - 1;
      let middle = original.point_count() / 2;

      let mutators: Vec<(&str, Mutator)> = vec![
        (
          "append",
          Box::new(|chain: &mut LineChain| {
            chain.append(point(20_000_000, 20_000_000));
          }),
        ),
        (
          "append_allow_duplicate",
          Box::new(|chain: &mut LineChain| {
            let last_point = chain.last_point().unwrap();

            chain.append_allow_duplicate(last_point);
          }),
        ),
        (
          "append_chain",
          Box::new(move |chain: &mut LineChain| {
            let mut other = LineChain::new();

            other.append_arc(&extra, ARC_HIGH_DEF);
            chain.append_chain(&other);
          }),
        ),
        (
          "append_arc",
          Box::new(move |chain: &mut LineChain| {
            chain.append_arc(&extra, ARC_HIGH_DEF);
          }),
        ),
        (
          "append_arc onto its own last point",
          Box::new(|chain: &mut LineChain| {
            let tail = chain.last_point().unwrap();
            let arc = ShapeArc::new(
              tail,
              tail + point(500_000, 500_000),
              tail + point(1_000_000, 0),
              0,
            );

            chain.append_arc(&arc, ARC_HIGH_DEF);
          }),
        ),
        (
          "insert at the middle",
          Box::new(move |chain: &mut LineChain| {
            chain.insert(middle, point(-5_000_000, -5_000_000));
          }),
        ),
        (
          "insert_arc at the middle",
          Box::new(move |chain: &mut LineChain| {
            chain.insert_arc(middle, &extra, ARC_HIGH_DEF);
          }),
        ),
        (
          "remove the middle vertex",
          Box::new(move |chain: &mut LineChain| {
            chain.remove(middle);
          }),
        ),
        (
          "remove_range over the middle",
          Box::new(move |chain: &mut LineChain| {
            chain.remove_range(1, middle);
          }),
        ),
        (
          "remove_range over everything",
          Box::new(move |chain: &mut LineChain| {
            chain.remove_range(0, last);
          }),
        ),
        (
          "remove_shape at the middle",
          Box::new(move |chain: &mut LineChain| {
            chain.remove_shape(middle);
          }),
        ),
        (
          "remove_shape at the last vertex",
          Box::new(move |chain: &mut LineChain| {
            chain.remove_shape(last);
          }),
        ),
        (
          "replace the middle vertex",
          Box::new(move |chain: &mut LineChain| {
            chain.replace(middle, middle, point(-5_000_000, -5_000_000));
          }),
        ),
        (
          "replace_with_chain",
          Box::new(move |chain: &mut LineChain| {
            let mut other = LineChain::new();

            other.append_arc(&extra, ARC_HIGH_DEF);
            chain.replace_with_chain(1, middle, &other);
          }),
        ),
        (
          "set_point at the middle",
          Box::new(move |chain: &mut LineChain| {
            chain.set_point(middle, point(-5_000_000, -5_000_000));
          }),
        ),
        (
          "set_point at the first vertex",
          Box::new(|chain: &mut LineChain| {
            chain.set_point(0, point(-5_000_000, -5_000_000));
          }),
        ),
        (
          "set_closed",
          Box::new(|chain: &mut LineChain| {
            chain.set_closed(true);
          }),
        ),
        (
          "set_closed then open again",
          Box::new(|chain: &mut LineChain| {
            chain.set_closed(true);
            chain.set_closed(false);
          }),
        ),
        ("reverse", Box::new(|chain: &mut LineChain| chain.reverse())),
        (
          "mirror",
          Box::new(|chain: &mut LineChain| {
            chain.mirror(&Seg::new(point(0, 0), point(0, 1_000_000)));
          }),
        ),
        (
          "move_by",
          Box::new(|chain: &mut LineChain| {
            chain.move_by(point(1234, -4321));
          }),
        ),
        (
          "simplify",
          Box::new(|chain: &mut LineChain| chain.simplify(0)),
        ),
        (
          "simplify with a tolerance",
          Box::new(|chain: &mut LineChain| {
            chain.simplify(1000);
          }),
        ),
        (
          "simplify2",
          Box::new(|chain: &mut LineChain| chain.simplify2(true)),
        ),
        (
          "simplify2 without colinear removal",
          Box::new(|chain: &mut LineChain| {
            chain.simplify2(false);
          }),
        ),
        (
          "remove_duplicate_points",
          Box::new(|chain: &mut LineChain| {
            chain.remove_duplicate_points();
          }),
        ),
        (
          "split on an arc",
          Box::new(move |chain: &mut LineChain| {
            let on_arc = chain.point(1);

            chain.split(on_arc + point(1, 0));
          }),
        ),
        (
          "split_exact on a vertex",
          Box::new(move |chain: &mut LineChain| {
            let on_arc = chain.point(1);

            chain.split_exact(on_arc);
          }),
        ),
        (
          "clear_arcs",
          Box::new(|chain: &mut LineChain| chain.clear_arcs()),
        ),
        ("clear", Box::new(|chain: &mut LineChain| chain.clear())),
      ];

      for (what, mutate) in mutators {
        let mut chain = original.clone();

        mutate(&mut chain);
        chain.check_invariants();

        assert_eq!(
          chain.point_count(),
          chain.shapes.len(),
          "{name}: {what} left the shape vector out of step"
        );

        // Every live arc is reachable exactly once and in order.
        let live: Vec<usize> =
          chain.live_arcs().map(|(index, _)| index).collect();
        let mut sorted = live.clone();

        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(
          live, sorted,
          "{name}: {what} left the live arcs out of order or duplicated"
        );
      }
    }
  }

  /// An arc whose approximation survives the demotion rule, taken from a
  /// centre, a start point and a sweep so that the geometry is always
  /// well formed.
  fn arc_strategy() -> impl Strategy<Value = ShapeArc> {
    (
      -10_000_000i32..10_000_000,
      -10_000_000i32..10_000_000,
      100_000i32..5_000_000,
      -350.0f64..350.0,
    )
      .prop_filter_map(
        "the arc must not be degenerate",
        |(x, y, radius, sweep)| {
          if sweep.abs() < 10.0 {
            return None;
          }

          let center = Vec2::new(x, y);
          let arc = ShapeArc::from_center_start_angle(
            center,
            center + Vec2::new(radius, 0),
            Degrees::new(sweep),
            0,
          );

          if arc.is_effective_line() {
            None
          } else {
            Some(arc)
          }
        },
      )
  }

  proptest! {
    /// Appending an arc and then slicing the whole chain gives the arc
    /// back, which is the exit criterion of slice 3 of note 09 section 12.
    ///
    /// The slice re-polygonises at the accuracy it is given
    /// (`shape_line_chain.cpp:1519`), so this passes the same accuracy the
    /// chain was built at. With a different one the arc still comes back
    /// identical and only the interior points move.
    #[test]
    fn append_arc_then_slicing_the_whole_chain_preserves_the_arc(
      arc in arc_strategy()
    ) {
      let mut chain = LineChain::new();

      chain.append_arc(&arc, LineChain::ARC_POLYGONIZATION_MAX_ERROR);
      prop_assume!(chain.arc_count() == 1);

      let last = chain.point_count() - 1;
      let sliced = chain.slice(0, last).unwrap();

      prop_assert_eq!(sliced.arc_count(), 1);
      prop_assert_eq!(sliced.arc(0), chain.arc(0));
      prop_assert_eq!(sliced.point_count(), chain.point_count());
      prop_assert_eq!(sliced.points(), chain.points());
    }

    /// Reversing twice is the identity on the points, the shape entries
    /// and the arcs.
    ///
    /// This is what the chain order invariant buys: the index remap at
    /// `shape_line_chain.cpp:926` is only a reversal while the arcs are in
    /// chain order, which is why erratum E7 is fixed rather than
    /// reproduced.
    #[test]
    fn reversing_twice_is_the_identity(
      first in arc_strategy(),
      second in arc_strategy(),
      tail_x in -10_000_000i32..10_000_000,
      tail_y in -10_000_000i32..10_000_000,
    ) {
      let mut chain = LineChain::new();

      chain.append_arc(&first, LineChain::ARC_POLYGONIZATION_MAX_ERROR);
      chain.append(Vec2::new(tail_x, tail_y));
      chain.append_arc(&second, LineChain::ARC_POLYGONIZATION_MAX_ERROR);

      let before = chain.clone();

      chain.reverse();
      chain.reverse();

      prop_assert_eq!(&chain, &before);
    }
  }
}
