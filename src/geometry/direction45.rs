// SPDX-License-Identifier: GPL-3.0-or-later

//! Directions in the 45 degree routing regime.
//!
//! Port of `DIRECTION_45`,
//! `libs/kimath/include/geometry/direction45.h:36`, the type that makes the
//! router a 45 degree router. A direction is one of eight octants plus an
//! undefined state, together with a flag saying whether turns step by 45 or
//! by 90 degrees.
//!
//! North is up on the screen, which is negative y in world space
//! (`direction45.h:45`). Every construction from a vector or a segment
//! therefore flips y before classifying (`direction45.h:96`, `:107`), and
//! [`Direction45::to_vector`] flips it back.
//!
//! KiCad spells the undefined state as the enumerator `UNDEFINED = -1`
//! (`direction45.h:59`) and that value does reach arithmetic in two places:
//!
//! - `Opposite()` indexes its lookup table with `m_dir`
//!   (`direction45.h:173`), so an undefined direction reads one element
//!   before the array. [`Direction45::opposite`] returns undefined instead,
//!   which is the only sensible reading of that expression.
//! - `Mask()` computes `1 << -1` (`direction45.h:307`). It is not ported,
//!   the router never calls it.
//!
//! Everywhere else the `-1` is either guarded (`Angle`, `Left`, `Right`) or
//! well defined: `IsDiagonal()` is `( m_dir % 2 ) == 1`
//! (`direction45.h:215`) and C++ gives `-1 % 2 == -1`, so an undefined
//! direction is not diagonal, which [`Direction45::is_diagonal`] keeps.
//!
//! Left out of this port:
//!
//! - `Mask()` (`direction45.h:305`), which the router never calls.

use crate::geometry::arc::{ShapeArc, resize_f64, truncate_f64_pair};
use crate::geometry::line_chain::LineChain;
use crate::geometry::math::{Degrees, kiround, rotate_point_f64, sign};
use crate::geometry::seg::Seg;
use crate::geometry::shape::Shape;
use crate::geometry::vec2::{Vec2, Vec2L};
use std::fmt;
use std::ops::BitOr;

/// The number of octants, the port of `Directions::LAST`,
/// `libs/kimath/include/geometry/direction45.h:58`. KiCad uses it as the
/// modulus of `Left` and `Right` (`direction45.h:258`, `:276`).
const OCTANT_COUNT: i8 = 8;

/// One of the eight directions of a rectilinear map, north being up on the
/// screen.
///
/// Port of the defined values of `DIRECTION_45::Directions`,
/// `libs/kimath/include/geometry/direction45.h:48`. The numeric order is
/// KiCad's and the angle classification depends on it: neighbouring
/// discriminants are 45 degrees apart, going clockwise on the screen.
///
/// KiCad's `UNDEFINED = -1` is not a variant here, it is the `None` of the
/// `Option<Octant>` inside [`Direction45`]. `LAST = 8` is not a direction
/// either, it is the modulus of the turns, see `Left` and `Right`.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
#[repr(i8)]
pub enum Octant {
  /// Up on the screen, negative y in world space.
  N = 0,
  /// Up and to the right.
  NE = 1,
  /// Right, positive x.
  E = 2,
  /// Down and to the right.
  SE = 3,
  /// Down on the screen, positive y in world space.
  S = 4,
  /// Down and to the left.
  SW = 5,
  /// Left, negative x.
  W = 6,
  /// Up and to the left.
  NW = 7,
}

impl Octant {
  /// The eight octants in KiCad's numeric order.
  ///
  /// KiCad iterates the enum by integer arithmetic instead
  /// (`direction45.h:258`), this is the spelling that does not need a cast
  /// back from an integer.
  pub const ALL: [Octant; 8] = [
    Octant::N,
    Octant::NE,
    Octant::E,
    Octant::SE,
    Octant::S,
    Octant::SW,
    Octant::W,
    Octant::NW,
  ];

  /// The numeric value KiCad gives this direction.
  ///
  /// Port of the implicit conversion of `Directions` to `int`, which
  /// `Angle` (`libs/kimath/include/geometry/direction45.h:186`),
  /// `IsDiagonal` (`:215`) and `Left` and `Right` (`:258`, `:276`) all
  /// rely on.
  pub const fn index(self) -> i8 {
    self as i8
  }

  /// The octant with this numeric value, or `None` when the value is not
  /// one of the eight.
  ///
  /// Port of `static_cast<DIRECTION_45::Directions>( int( angle ) )`,
  /// `pcbnew/router/pns_line_placer.cpp:1417`, which turns a pad
  /// orientation in degrees into a direction. That cast can produce
  /// `LAST = 8` for an orientation of 337.5 degrees or more, which is not
  /// a direction at all and which KiCad then carries around as if it were
  /// one. Here the caller gets a `None` and has to decide.
  pub const fn from_index(index: i32) -> Option<Octant> {
    match index {
      0 => Some(Octant::N),
      1 => Some(Octant::NE),
      2 => Some(Octant::E),
      3 => Some(Octant::SE),
      4 => Some(Octant::S),
      5 => Some(Octant::SW),
      6 => Some(Octant::W),
      7 => Some(Octant::NW),
      _ => None,
    }
  }
}

/// The kind of angle formed by two directions.
///
/// Port of `DIRECTION_45::AngleType`,
/// `libs/kimath/include/geometry/direction45.h:77`, with KiCad's values.
/// They are powers of two because the router ORs them into masks and tests
/// a result against the mask with `&`: see
/// `pcbnew/router/pns_optimizer.cpp:1114` and
/// `pcbnew/router/pns_line_placer.cpp:325`, which both build a
/// `ForbiddenAngles` mask, and `pns_optimizer.cpp:314`.
///
/// A single [`Direction45::angle`] result always carries exactly one bit.
/// A mask carries several, so KiCad's `angle & mask` idiom is
/// [`AngleType::intersects`], not [`AngleType::contains`].
///
/// This is a newtype rather than a `bitflags` crate type because the crate
/// carries no runtime dependencies, see `DESIGN.md` section 10.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct AngleType(u8);

impl AngleType {
  /// 135 degrees between the two directions, one octant apart.
  ///
  /// Port of `ANG_OBTUSE`,
  /// `libs/kimath/include/geometry/direction45.h:79`.
  pub const OBTUSE: AngleType = AngleType(0x01);

  /// 90 degrees, two octants apart.
  ///
  /// Port of `ANG_RIGHT`,
  /// `libs/kimath/include/geometry/direction45.h:80`.
  pub const RIGHT: AngleType = AngleType(0x02);

  /// 45 degrees, three octants apart.
  ///
  /// Port of `ANG_ACUTE`,
  /// `libs/kimath/include/geometry/direction45.h:81`.
  pub const ACUTE: AngleType = AngleType(0x04);

  /// The same direction.
  ///
  /// Port of `ANG_STRAIGHT`,
  /// `libs/kimath/include/geometry/direction45.h:82`.
  pub const STRAIGHT: AngleType = AngleType(0x08);

  /// Opposite directions, four octants apart.
  ///
  /// Port of `ANG_HALF_FULL`,
  /// `libs/kimath/include/geometry/direction45.h:83`.
  pub const HALF_FULL: AngleType = AngleType(0x10);

  /// At least one of the two directions is undefined.
  ///
  /// Port of `ANG_UNDEFINED`,
  /// `libs/kimath/include/geometry/direction45.h:84`.
  pub const UNDEFINED: AngleType = AngleType(0x20);

  /// The raw bits, for a caller that has to talk to KiCad's numbering.
  pub const fn bits(self) -> u8 {
    self.0
  }

  /// Every bit of both, as a `const fn`.
  ///
  /// The same value the [`BitOr`] implementation below produces. It
  /// exists because a trait method cannot be called while building a
  /// `const`, and the one mask KiCad spells inline is worth naming once:
  /// `pcbnew/router/pns_optimizer.cpp:1114` builds
  /// [`crate::optimizer::FORBIDDEN_ANGLES`] this way.
  pub const fn union(self, other: AngleType) -> AngleType {
    AngleType(self.0 | other.0)
  }

  /// Whether every bit of `other` is set here.
  pub const fn contains(self, other: AngleType) -> bool {
    (self.0 & other.0) == other.0
  }

  /// Whether any bit is set in both.
  ///
  /// This is the port of the router's `angle & mask` test, for instance
  /// `pcbnew/router/pns_optimizer.cpp:1159` and
  /// `pcbnew/router/pns_dragger.cpp:633`.
  pub const fn intersects(self, other: AngleType) -> bool {
    (self.0 & other.0) != 0
  }
}

/// Port of the `|` that builds the angle masks, for instance
/// `pcbnew/router/pns_optimizer.cpp:1114`.
impl BitOr for AngleType {
  type Output = AngleType;

  fn bitor(self, other: AngleType) -> AngleType {
    AngleType(self.0 | other.0)
  }
}

/// How the corner between the two segments of an initial trace is built.
///
/// Port of `DIRECTION_45::CORNER_MODE`,
/// `libs/kimath/include/geometry/direction45.h:66`, with KiCad's four
/// discriminants. The two rounded modes replace the corner with an arc,
/// which is the only thing they change: nothing in the walkaround, the
/// shove or the optimizer treats `Rounded45` differently from
/// `Mitered45`, or `Rounded90` differently from `Mitered90`, and every
/// site that branches asks "is this a 90 degree mode" or "is this a 45
/// degree mode" (note 09 section 3.4). KiCad's `ROUTING_SETTINGS`
/// defaults to `MITERED_45` (`pcbnew/router/pns_routing_settings.cpp:53`).
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum CornerMode {
  /// Horizontal, vertical and 45 degree segments, mitered corners.
  Mitered45 = 0,
  /// Horizontal, vertical and 45 degree segments, the corner replaced by
  /// a 45 degree arc.
  Rounded45 = 1,
  /// Horizontal and vertical segments only, mitered corners.
  Mitered90 = 2,
  /// Horizontal and vertical segments only, the corner replaced by a
  /// quarter turn arc.
  Rounded90 = 3,
}

impl CornerMode {
  /// Whether the mode lays down right angles rather than 45 degree ones.
  ///
  /// Port of the `is90mode` local of `BuildInitialTrace`,
  /// `libs/kimath/src/geometry/direction_45.cpp:40`, which every other
  /// corner mode branch in the router spells out inline:
  /// `pcbnew/router/pns_node.cpp:326`,
  /// `pcbnew/router/pns_walkaround.cpp:162`,
  /// `pcbnew/router/pns_line_placer.cpp:827` and
  /// `pcbnew/router/pns_optimizer.cpp:859`.
  pub fn is_90_degree(self) -> bool {
    self == Self::Mitered90 || self == Self::Rounded90
  }

  /// Whether the mode lays down 45 degree corners rather than right
  /// angles.
  ///
  /// The complement of [`CornerMode::is_90_degree`], written out because
  /// KiCad writes the test that way at the two smart pad gates
  /// (`pcbnew/router/pns_line_placer.cpp:763`, `:983`) and at the post
  /// shove one (`pcbnew/router/pns_shove.cpp:2080`).
  pub fn is_45_degree(self) -> bool {
    self == Self::Mitered45 || self == Self::Rounded45
  }

  /// Whether the corner is an arc rather than a mitre.
  ///
  /// KiCad has no such predicate; it is the one distinction the router
  /// never makes, and it exists here so that a host boundary can refuse
  /// the two rounded modes in one test. LibrePCB's trace format carries
  /// no angle (`libs/librepcb/core/geometry/trace.cpp:236`), so its
  /// integration refuses them; see `doc/work/012-arcs.md`.
  pub fn is_rounded(self) -> bool {
    self == Self::Rounded45 || self == Self::Rounded90
  }
}

/// A routing direction: an octant, or undefined, plus the turn step.
///
/// Port of `DIRECTION_45`,
/// `libs/kimath/include/geometry/direction45.h:36`. `dir` is `m_dir`
/// (`:346`) with `UNDEFINED` spelled as `None`, and `ninety_deg` is
/// `m_90deg` (`:349`), which only affects [`Direction45::left`] and
/// [`Direction45::right`].
///
/// The default is the undefined direction with 45 degree turns, KiCad's
/// default constructor at `direction45.h:87`, which the router spells
/// `DIRECTION_45()` at every `BuildInitialTrace` call site that has no
/// direction to impose.
#[derive(Copy, Clone, Eq, Debug, Default)]
pub struct Direction45 {
  /// The octant, or `None` for KiCad's `UNDEFINED`.
  dir: Option<Octant>,
  /// Whether [`Direction45::left`] and [`Direction45::right`] step by two
  /// octants instead of one.
  ninety_deg: bool,
}

impl Direction45 {
  /// A direction from an octant, with 45 degree turns.
  ///
  /// Port of `DIRECTION_45( Directions aDir )`,
  /// `libs/kimath/include/geometry/direction45.h:87`, which leaves
  /// `m_90deg` false.
  pub const fn from_octant(octant: Octant) -> Self {
    Self {
      dir: Some(octant),
      ninety_deg: false,
    }
  }

  /// The direction of a vector in world space, rounded to the nearest
  /// octant.
  ///
  /// Port of `DIRECTION_45( const VECTOR2I&, bool )`,
  /// `libs/kimath/include/geometry/direction45.h:92`, which flips y and
  /// calls `construct_`. The zero vector is the only input that gives an
  /// undefined direction (`direction45.h:321`).
  ///
  /// Deviation: KiCad negates y in `int`, which wraps for `i32::MIN`. The
  /// negation happens in `i64` here, so a vector pointing at the very
  /// bottom of the coordinate range classifies as south rather than north.
  pub fn from_vector(vec: Vec2, ninety_deg: bool) -> Self {
    Self {
      dir: construct(i64::from(vec.x), i64::from(vec.y)),
      ninety_deg,
    }
  }

  /// The direction of a segment, from its first point to its second.
  ///
  /// Port of `DIRECTION_45( const SEG&, bool )`,
  /// `libs/kimath/include/geometry/direction45.h:103`.
  ///
  /// Deviation: KiCad takes `B - A` in `int`, which wraps for a segment
  /// longer than the coordinate range. The difference is widened here, see
  /// `Vec2::widening_sub`.
  pub fn from_seg(seg: &Seg, ninety_deg: bool) -> Self {
    let delta = seg.b.widening_sub(seg.a);

    Self {
      dir: construct(delta.x, delta.y),
      ninety_deg,
    }
  }

  /// The direction of an arc, taken from its **chord**.
  ///
  /// Port of `DIRECTION_45( const SHAPE_ARC&, bool )`,
  /// `libs/kimath/include/geometry/direction45.h:116`, which classifies
  /// `aArc.GetP1() - aArc.GetP0()` with the same y flip every other
  /// construction applies.
  ///
  /// The chord is not either tangent. A 45 degree arc's chord bisects the
  /// two tangents, so an arc whose tangents are axis aligned answers a
  /// diagonal octant and one whose tangents are diagonal answers an axis
  /// aligned octant. That is what the placer reads when the head or the
  /// tail ends on an arc (`pcbnew/router/pns_line_placer.cpp:200`,
  /// `:207`, `:232`, `:355`, `:365`, `:380`); see note 09 section 3.3.
  ///
  /// An arc whose two endpoints coincide, which is a whole turn, gives
  /// the undefined direction, the same answer a zero length segment
  /// gives.
  pub fn from_arc(arc: &ShapeArc, ninety_deg: bool) -> Self {
    let delta = arc.end().widening_sub(arc.start());

    Self {
      dir: construct(delta.x, delta.y),
      ninety_deg,
    }
  }

  /// The octant, or `None` when the direction is undefined.
  ///
  /// KiCad has no accessor for `m_dir`, it compares whole directions
  /// instead (`direction45.h:238`).
  pub const fn octant(self) -> Option<Octant> {
    self.dir
  }

  /// Whether the direction is one of the eight octants.
  ///
  /// Port of `IsDefined`,
  /// `libs/kimath/include/geometry/direction45.h:218`. The router never
  /// calls it, it compares against `DIRECTION_45::UNDEFINED` instead
  /// (`pcbnew/router/pns_line.cpp:1277`), which this crate spells as this
  /// method.
  pub const fn is_defined(self) -> bool {
    self.dir.is_some()
  }

  /// Whether the direction is one of NE, SE, SW, NW.
  ///
  /// Port of `IsDiagonal`,
  /// `libs/kimath/include/geometry/direction45.h:213`, which is
  /// `( m_dir % 2 ) == 1`. An undefined direction is not diagonal, because
  /// C++ evaluates `-1 % 2` to `-1`.
  pub fn is_diagonal(self) -> bool {
    self.dir.is_some_and(|octant| octant.index() % 2 == 1)
  }

  /// The kind of angle between this direction and another.
  ///
  /// Port of `Angle`, `libs/kimath/include/geometry/direction45.h:181`.
  /// The classification is the octant distance `d = |a - b|`: 0 straight,
  /// 1 and 7 obtuse, 2 and 6 right, 3 and 5 acute, 4 half full, and
  /// undefined as soon as either direction is undefined. KiCad's final
  /// `else` covers only `d == 0`, since `d` cannot exceed 7.
  pub fn angle(self, other: Self) -> AngleType {
    let (Some(own), Some(other)) = (self.dir, other.dir) else {
      return AngleType::UNDEFINED;
    };

    match (own.index() - other.index()).abs() {
      1 | 7 => AngleType::OBTUSE,
      2 | 6 => AngleType::RIGHT,
      3 | 5 => AngleType::ACUTE,
      4 => AngleType::HALF_FULL,
      _ => AngleType::STRAIGHT,
    }
  }

  /// Whether the two directions form a 135 degree angle.
  ///
  /// Port of `IsObtuse`,
  /// `libs/kimath/include/geometry/direction45.h:203`.
  pub fn is_obtuse(self, other: Self) -> bool {
    self.angle(other) == AngleType::OBTUSE
  }

  /// The direction turned right by 45 degrees, or by 90 in 90 degree mode.
  ///
  /// Port of `Right`, `libs/kimath/include/geometry/direction45.h:251`.
  /// Right is clockwise on the screen, the direction of increasing octant
  /// numbers. An undefined direction stays undefined.
  ///
  /// KiCad builds the result with the default constructor and only assigns
  /// `m_dir` (`direction45.h:253`, `:258`), so the 90 degree flag is lost:
  /// `d.Right().Right()` on a 90 degree direction turns by 90 and then by
  /// 45. That is reproduced here, `pcbnew/router/pns_optimizer.cpp:1592`
  /// chains two turns.
  pub fn right(self) -> Self {
    let step = if self.ninety_deg { 2 } else { 1 };

    Self {
      dir: self.dir.and_then(|octant| {
        Octant::from_index(i32::from((octant.index() + step) % OCTANT_COUNT))
      }),
      ninety_deg: false,
    }
  }

  /// The direction turned left by 45 degrees, or by 90 in 90 degree mode.
  ///
  /// Port of `Left`, `libs/kimath/include/geometry/direction45.h:269`.
  /// The 90 degree flag is lost the same way [`Direction45::right`]
  /// loses it.
  pub fn left(self) -> Self {
    let step = if self.ninety_deg { 2 } else { 1 };

    Self {
      dir: self.dir.and_then(|octant| {
        Octant::from_index(i32::from(
          (octant.index() + OCTANT_COUNT - step) % OCTANT_COUNT,
        ))
      }),
      ninety_deg: false,
    }
  }

  /// The direction turned by 180 degrees.
  ///
  /// Port of `Opposite`,
  /// `libs/kimath/include/geometry/direction45.h:170`, whose lookup table
  /// also loses the 90 degree flag, since the result is built through the
  /// converting constructor.
  ///
  /// Deviation: KiCad indexes that table with `m_dir`, so an undefined
  /// direction reads `OppositeMap[-1]`, one element before the array. This
  /// returns undefined.
  pub fn opposite(self) -> Self {
    Self {
      dir: self.dir.map(|octant| match octant {
        Octant::N => Octant::S,
        Octant::NE => Octant::SW,
        Octant::E => Octant::W,
        Octant::SE => Octant::NW,
        Octant::S => Octant::N,
        Octant::SW => Octant::NE,
        Octant::W => Octant::E,
        Octant::NW => Octant::SE,
      }),
      ninety_deg: false,
    }
  }

  /// A vector of one nanometre per axis pointing this way, in world
  /// coordinates.
  ///
  /// Port of `ToVector`,
  /// `libs/kimath/include/geometry/direction45.h:287`. The screen to world
  /// y flip is already applied, so north is `(0, -1)`. A diagonal is a
  /// corner of the unit square, `(1, 1)` for south east, not a unit
  /// vector: its length is the square root of two. An undefined direction
  /// gives the zero vector.
  pub fn to_vector(self) -> Vec2 {
    match self.dir {
      Some(Octant::N) => Vec2::new(0, -1),
      Some(Octant::NE) => Vec2::new(1, -1),
      Some(Octant::E) => Vec2::new(1, 0),
      Some(Octant::SE) => Vec2::new(1, 1),
      Some(Octant::S) => Vec2::new(0, 1),
      Some(Octant::SW) => Vec2::new(-1, 1),
      Some(Octant::W) => Vec2::new(-1, 0),
      Some(Octant::NW) => Vec2::new(-1, -1),
      None => Vec2::new(0, 0),
    }
  }

  /// The two or three shape trace between two points that obeys the 45
  /// degree routing regime.
  ///
  /// Port of `BuildInitialTrace`,
  /// `libs/kimath/src/geometry/direction_45.cpp:24`. It is the single most
  /// important routine of the 45 degree regime: the placer, the line, the
  /// optimizer and the posture solver all build their candidate paths with
  /// it, and in the two rounded corner modes it is the **only** producer
  /// of arcs in the whole router (note 09 section 3.4).
  ///
  /// The result is the chain KiCad builds, with the duplicate suppression
  /// of `SHAPE_LINE_CHAIN::Append`
  /// (`libs/kimath/include/geometry/shape_line_chain.h:534`), which drops
  /// a point equal to the one before it. So a mitered result holds:
  ///
  /// - one point when `p0` and `p1` are the same point, because the second
  ///   `Append` is suppressed. Consumers that take segment zero of the
  ///   result have to cope, as `pcbnew/router/pns_optimizer.cpp:1155`
  ///   does;
  /// - two points when the pair is axis aligned, or an exact diagonal in
  ///   45 degree mode (`direction_45.cpp:46`);
  /// - three points otherwise, `p0`, the corner and `p1`.
  ///
  /// A rounded result holds at most one arc, plus at most one plain
  /// vertex before or after it. Which of the two, and where the arc's
  /// endpoints land, is the whole of the private `rounded_45` and
  /// `rounded_90` below, whose doc comments carry KiCad's case tables.
  ///
  /// `start_diagonal` is KiCad's `aStartDiagonal`: it asks for the diagonal
  /// leg first, and in 90 degree mode for the shorter leg first
  /// (`direction45.h:230`). It is only consulted when this direction is
  /// undefined; a defined direction imposes its own posture through
  /// [`Direction45::is_diagonal`] (`direction_45.cpp:31`).
  ///
  /// # `Simplify` is run for the rounded modes and not for the mitered ones
  ///
  /// KiCad ends with `pl.Simplify()` (`direction_45.cpp:310`) on every
  /// path but the `ROUNDED_90` `w == h` early return (`:259`). The two
  /// rounded branches below run it, where the two mitered ones keep this
  /// port's older decision not to.
  ///
  /// The reason for the asymmetry is that `Simplify` is **not** a no
  /// operation on a mitered result, contrary to what this doc comment
  /// used to claim: a tolerance of zero bottoms out in
  /// `squared distance < ( tolerance + 1 )^2`
  /// ([`LineChain::simplify`]), so a corner less than a nanometre off its
  /// own chord is dropped, which a trace a few nanometres long can
  /// produce. Every recorded session and every KiCad replay golden was
  /// taken against the current behaviour, so changing it is not slice
  /// 6's to make. On a rounded result `Simplify` genuinely cannot change
  /// anything, because a run may not step over a vertex an arc claims
  /// (`shape_line_chain.cpp:2816`) and the one or two plain vertices a
  /// rounded result carries sit at the ends with no intermediate vertex
  /// to drop; it is run anyway so that the branch is KiCad's line for
  /// line.
  ///
  /// Deviation: the coordinate differences are taken in `i64`, where KiCad
  /// takes them in `int` (`direction_45.cpp:36`) and wraps for a pair
  /// spanning more than the coordinate range. Every vertex the mitered
  /// branches produce lies inside the bounding box of `p0` and `p1`, so
  /// the narrowing back to `i32` is exact; the rounded branches narrow
  /// the two corner vectors once and saturate, where KiCad wraps.
  pub fn build_initial_trace(
    self,
    p0: Vec2,
    p1: Vec2,
    start_diagonal: bool,
    corner_mode: CornerMode,
  ) -> LineChain {
    let start_diagonal = if self.is_defined() {
      self.is_diagonal()
    } else {
      start_diagonal
    };

    let delta = p1.widening_sub(p0);
    let width = delta.x.abs();
    let height = delta.y.abs();
    let sign_x = i64::from(sign(delta.x));
    let sign_y = i64::from(sign(delta.y));

    // `is90mode`, direction_45.cpp:40.
    let is_90_mode = corner_mode.is_90_degree();

    // The shortcut of direction_45.cpp:46, which keeps the single segment
    // cases away from the corner arithmetic. It is also what makes
    // `ROUNDED_45`'s own `w == h` block at `:123` unreachable, erratum
    // E19.
    if width == 0 || height == 0 || (!is_90_mode && height == width) {
      return chain_of(&[p0, p1]);
    }

    // `mp0`, `mp1` and `tangentLength` of direction_45.cpp:52 to :81.
    // `mp0` is the corner offset of a straight first trace, `mp1` that of
    // a diagonal first one; the 90 degree mode has only one corner and
    // leaves `mp1` at zero. `tangentLength` is the straight leg's length
    // less the diagonal leg's, and only the rounded 45 branch reads it.
    let mut tangent_length: i64 = 0;
    let (corner_straight, corner_diagonal) = if is_90_mode {
      let corner = if start_diagonal == (height >= width) {
        Vec2L::new(width * sign_x, 0)
      } else {
        Vec2L::new(0, sign_y * height)
      };

      (corner, Vec2L::new(0, 0))
    } else if width > height {
      let diagonal = Vec2L::new(height * sign_x, height * sign_y);

      tangent_length = (width - height) - diagonal.euclidean_norm();

      (Vec2L::new((width - height) * sign_x, 0), diagonal)
    } else {
      let diagonal = Vec2L::new(sign_x * width, sign_y * width);

      tangent_length = (height - width) - diagonal.euclidean_norm();

      (Vec2L::new(0, sign_y * (height - width)), diagonal)
    };

    let extents = Extents {
      width,
      height,
      sign_x,
      sign_y,
    };

    match corner_mode {
      // direction_45.cpp:86.
      CornerMode::Mitered45 => {
        let corner = if start_diagonal {
          corner_diagonal
        } else {
          corner_straight
        };

        chain_of(&[p0, (Vec2L::from(p0) + corner).saturating_to_vec2(), p1])
      }
      // direction_45.cpp:224.
      CornerMode::Mitered90 => chain_of(&[
        p0,
        (Vec2L::from(p0) + corner_straight).saturating_to_vec2(),
        p1,
      ]),
      // direction_45.cpp:105.
      CornerMode::Rounded45 => Self::rounded_45(
        p0,
        p1,
        start_diagonal,
        extents,
        corner_straight.saturating_to_vec2(),
        corner_diagonal.saturating_to_vec2(),
        tangent_length,
      ),
      // direction_45.cpp:239.
      CornerMode::Rounded90 => Self::rounded_90(
        p0,
        p1,
        start_diagonal,
        extents,
        corner_straight.saturating_to_vec2(),
      ),
    }
  }

  /// The `ROUNDED_45` branch of [`Direction45::build_initial_trace`].
  ///
  /// Port of `direction_45.cpp:105` to `:222`. Four sub-cases on the
  /// posture and the sign of `tangentLength`, three of which derive the
  /// arc from two endpoints and an angle and one of which derives it from
  /// a centre:
  ///
  /// | posture | `tangentLength` | arc | rest | line |
  /// | --- | --- | --- | --- | --- |
  /// | diagonal | `>= 0` | `p0` to `p1 - mp0.resize( t )`, `+45 * s` | then `p1` | `:143` |
  /// | diagonal | `< 0` | `p0 + mp1.resize( |t| )` to `p1`, `+45 * s` | `p0` first | `:156` |
  /// | straight | `>= 0` | `p0 + mp0.resize( t )` to `p1`, `-45 * s` | `p0` first | `:177` |
  /// | straight | `< 0` | centre `p0 + centre_dir.resize( r )`, start `p0`, `-45 * s` | then `p1` | `:190` |
  ///
  /// `s` is `rotation_sign`, `( w > h ) ? -sw * sh : sw * sh`, computed
  /// identically in both halves (`:141`, `:172`). `centre_dir` is `mp0`
  /// rotated by `90 * s` in floating point (`:173`).
  ///
  /// Every sub-case guards against a zero length arc and appends the
  /// point instead (`:149`, `:164`, `:185`, `:213`), because
  /// [`LineChain::append_arc`] would otherwise demote it to a plain
  /// segment and leave the chain a shape short.
  ///
  /// # Erratum E17, the arc radius
  ///
  /// KiCad computes `diag2`, `diagLength` and `arcRadius` unconditionally
  /// at `:130` to `:132` and reads `arcRadius` only in the fourth
  /// sub-case; `diagLength` only ever feeds `arcRadius`. The three are
  /// computed here inside the branch that needs them. Nothing is
  /// observable either way, the expressions being pure, and hoisting them
  /// out would put a `KiROUND` of an unbounded `double` on three paths
  /// that never look at it. See
  /// `rounded_45_computes_the_arc_radius_only_where_it_is_used_erratum_e17`.
  ///
  /// # Erratum E18, the re-snap
  ///
  /// The fourth sub-case builds its arc from a centre, which loses the
  /// endpoint, so `:197` to `:211` pulls the endpoint back onto `p1` in
  /// whichever axis it is within `SHAPE_ARC::MIN_PRECISION_IU` of
  /// ([`Shape::MIN_PRECISION_IU`], four nanometres) and rebuilds the arc
  /// through `ConstructFromStartEndAngle`. The rebuild passes
  /// `+45 * rotation_sign` where the original passed `-45 * rotation_sign`
  /// (`:194` versus `:205`, `:210`), and
  /// [`ShapeArc::from_start_end_angle`] places the centre from the
  /// angle's sign, so **the corrected arc bulges the other way**: it is
  /// the mirror image of the fillet it was correcting, tangent to
  /// neither leg. Reproduced; see
  /// `rounded_45_re_snap_flips_the_arc_bulge_erratum_e18`.
  #[allow(clippy::too_many_arguments)]
  fn rounded_45(
    p0: Vec2,
    p1: Vec2,
    start_diagonal: bool,
    extents: Extents,
    corner_straight: Vec2,
    corner_diagonal: Vec2,
    tangent_length: i64,
  ) -> LineChain {
    let mut chain = LineChain::new();
    // :141 and :172, the same expression in both halves.
    let rotation_sign = f64::from(extents.rotation_sign());
    let tangent = saturate_i32(tangent_length);

    if start_diagonal {
      if tangent_length >= 0 {
        // :143, a positive tangent length puts the arc at the start.
        let arc_endpoint = p1 - corner_straight.resize(tangent);
        let arc = ShapeArc::from_start_end_angle(
          p0,
          arc_endpoint,
          Degrees::EIGHTH_TURN * rotation_sign,
          0,
        );

        append_arc_or_point(&mut chain, &arc, p0);
        chain.append(p1);
      } else {
        // :156, a negative one puts it at the end.
        let arc_endpoint =
          p0 + corner_diagonal.resize(saturate_i32(tangent_length.abs()));
        let arc = ShapeArc::from_start_end_angle(
          arc_endpoint,
          p1,
          Degrees::EIGHTH_TURN * rotation_sign,
          0,
        );

        chain.append(p0);
        append_arc_or_point(&mut chain, &arc, p1);
      }
    } else if tangent_length >= 0 {
      // :177, the arc goes at the end.
      let arc_endpoint = p0 + corner_straight.resize(tangent);
      let arc = ShapeArc::from_start_end_angle(
        arc_endpoint,
        p1,
        -Degrees::EIGHTH_TURN * rotation_sign,
        0,
      );

      chain.append(p0);
      append_arc_or_point(&mut chain, &arc, p1);
    } else {
      // :190, the arc goes at the start and is built from a centre.
      let arc = Self::rounded_45_from_centre(
        p0,
        p1,
        extents,
        corner_straight,
        rotation_sign,
      );

      append_arc_or_point(&mut chain, &arc, p0);
      chain.append(p1);
    }

    // :310
    chain.simplify(0);
    chain
  }

  /// The fourth sub-case of [`Direction45::rounded_45`], the one built
  /// from a centre.
  ///
  /// Port of `direction_45.cpp:190` to `:211`, including erratum E17's
  /// radius and erratum E18's re-snap; both are documented on
  /// [`Direction45::rounded_45`].
  fn rounded_45_from_centre(
    p0: Vec2,
    p1: Vec2,
    extents: Extents,
    corner_straight: Vec2,
    rotation_sign: f64,
  ) -> ShapeArc {
    // :173, `mp0` turned a quarter turn, in floating point.
    let centre_direction = rotate_point_f64(
      (f64::from(corner_straight.x), f64::from(corner_straight.y)),
      (0.0, 0.0),
      Degrees::QUARTER_TURN * rotation_sign,
    );
    // :130 to :132. A negative tangent length takes `mp0`'s squared
    // length; the chord of a 135 degree sweep of that radius is the
    // arc's chord, and an arc of 45 degrees with that chord has radius
    // `chord / ( 2 * cos( 67.5 ) )`.
    //
    // The two cosines are `std::cos` of a `double`, not `EDA_ANGLE::Cos`,
    // and the difference is real: `EDA_ANGLE( 135 ).Cos()` answers the
    // exact `-1 / sqrt( 2 )` from its own table
    // (`libs/kimath/include/geometry/eda_angle.h:197`) where
    // `std::cos( 3 * M_PI_4 )` answers the double one unit in the last
    // place above it. KiCad writes the `std::cos` form, so this does.
    let diagonal_squared = extents.straight_leg_squared() as f64;
    let three_quarter_pi = 3.0 * std::f64::consts::FRAC_PI_4;
    let diagonal_length = ((2.0 * diagonal_squared)
      - (2.0 * diagonal_squared * three_quarter_pi.cos()))
    .sqrt();
    let arc_radius = kiround(
      diagonal_length / (2.0 * (67.5 * std::f64::consts::PI / 180.0).cos()),
    );

    // :192, the centre, in floating point and then truncated.
    let offset = resize_f64(
      centre_direction.0,
      centre_direction.1,
      f64::from(arc_radius),
    );
    let arc_centre = truncate_f64_pair((
      f64::from(p0.x) + offset.0,
      f64::from(p0.y) + offset.1,
    ));
    // :194
    let arc = ShapeArc::from_center_start_angle(
      arc_centre,
      p0,
      -Degrees::EIGHTH_TURN * rotation_sign,
      0,
    );
    let endpoint = arc.end();

    // :200 and :206, erratum E18. The rebuild angle is positive where
    // the construction above was negative, which mirrors the arc about
    // its chord.
    //
    // Deviation: the two differences are taken in `i64`, where KiCad
    // subtracts in `int` and wraps for a pair more than the coordinate
    // range apart.
    let gap = endpoint.widening_sub(p1);
    let tolerance = i64::from(Shape::MIN_PRECISION_IU);
    let fixed_end = if gap.y.abs() < tolerance {
      Some(Vec2::new(endpoint.x, p1.y))
    } else if gap.x.abs() < tolerance {
      Some(Vec2::new(p1.x, endpoint.y))
    } else {
      None
    };

    match fixed_end {
      Some(end) => ShapeArc::from_start_end_angle(
        arc.start(),
        end,
        Degrees::EIGHTH_TURN * rotation_sign,
        0,
      ),
      None => arc,
    }
  }

  /// The `ROUNDED_90` branch of [`Direction45::build_initial_trace`].
  ///
  /// Port of `direction_45.cpp:239` to `:307`. The radius is the shorter
  /// of the two extents and every centre is an exact integer point, so
  /// this is the better behaved of the two rounded branches: the only
  /// floating point in it is
  /// [`ShapeArc::from_start_end_center`]'s own mid point rotation.
  ///
  /// | case | arc | rest | line |
  /// | --- | --- | --- | --- |
  /// | `w == h` | `p0` to `p1` about `p1 - mp0`, clockwise `( sh == sw ) != diagonal` | **returns at once** | `:255` |
  /// | diagonal, `h > w` | `p0` to `( p1.x, y )` about `( p0.x, y )`, clockwise `sh != sw` | then `p1` | `:267` |
  /// | diagonal, `h <= w` | `p0` to `( x, p1.y )` about `( x, p0.y )`, clockwise `sh == sw` | then `p1` | `:276` |
  /// | straight, `w > h` | `( x, p0.y )` to `p1` about `( x, p1.y )`, clockwise `sh != sw` | `p0` first | `:288` |
  /// | straight, `w <= h` | `( p0.x, y )` to `p1` about `( p1.x, y )`, clockwise `sh == sw` | `p0` first | `:297` |
  ///
  /// with `y = p0.y + w * sh` or `p1.y - w * sh` and `x = p0.x + h * sw`
  /// or `p1.x - h * sw` as the table's rows need them.
  ///
  /// The `w == h` case is the one path out of `BuildInitialTrace` that
  /// does not run `Simplify` (`:259`), which is reproduced here. Unlike
  /// [`Direction45::rounded_45`] this branch has no degenerate guard;
  /// KiCad has none either, and an arc `append_arc` demotes for having a
  /// two point polyline leaves a plain segment behind rather than
  /// nothing.
  fn rounded_90(
    p0: Vec2,
    p1: Vec2,
    start_diagonal: bool,
    extents: Extents,
    corner_straight: Vec2,
  ) -> LineChain {
    let mut chain = LineChain::new();
    let Extents {
      width,
      height,
      sign_x,
      sign_y,
    } = extents;
    let same_sign = sign_y == sign_x;

    // :255, the quarter turn that needs no straight leg at all.
    if width == height {
      let arc = ShapeArc::from_start_end_center(
        p0,
        p1,
        p1 - corner_straight,
        same_sign != start_diagonal,
        0,
      );

      chain.append_arc(&arc, LineChain::ARC_POLYGONIZATION_MAX_ERROR);

      // :259, the one return that skips `Simplify`.
      return chain;
    }

    let arc = if start_diagonal {
      if height > width {
        // :267, the arc then a vertical line.
        let y = saturate_i32(i64::from(p0.y) + width * sign_y);

        ShapeArc::from_start_end_center(
          p0,
          Vec2::new(p1.x, y),
          Vec2::new(p0.x, y),
          !same_sign,
          0,
        )
      } else {
        // :276, the arc then a horizontal line.
        let x = saturate_i32(i64::from(p0.x) + height * sign_x);

        ShapeArc::from_start_end_center(
          p0,
          Vec2::new(x, p1.y),
          Vec2::new(x, p0.y),
          same_sign,
          0,
        )
      }
    } else if width > height {
      // :288, a horizontal line then the arc.
      let x = saturate_i32(i64::from(p1.x) - height * sign_x);

      chain.append(p0);
      ShapeArc::from_start_end_center(
        Vec2::new(x, p0.y),
        p1,
        Vec2::new(x, p1.y),
        !same_sign,
        0,
      )
    } else {
      // :297, a vertical line then the arc.
      let y = saturate_i32(i64::from(p1.y) - width * sign_y);

      chain.append(p0);
      ShapeArc::from_start_end_center(
        Vec2::new(p0.x, y),
        p1,
        Vec2::new(p1.x, y),
        same_sign,
        0,
      )
    };

    chain.append_arc(&arc, LineChain::ARC_POLYGONIZATION_MAX_ERROR);

    if start_diagonal {
      chain.append(p1);
    }

    // :310
    chain.simplify(0);
    chain
  }
}

/// The four quantities `BuildInitialTrace` derives from its two points.
///
/// `w`, `h`, `sw` and `sh` of `direction_45.cpp:35` to `:38`, widened to
/// `i64` for the reason given on [`Direction45::build_initial_trace`].
/// They travel together because the rounded branches read all four.
#[derive(Copy, Clone, Debug)]
struct Extents {
  /// `w`, the absolute horizontal extent.
  width: i64,
  /// `h`, the absolute vertical extent.
  height: i64,
  /// `sw`, the sign of the horizontal step.
  sign_x: i64,
  /// `sh`, the sign of the vertical step.
  sign_y: i64,
}

impl Extents {
  /// `rotationSign`, `direction_45.cpp:141` and `:172`, which are the
  /// same expression.
  fn rotation_sign(self) -> i32 {
    let product = self.sign_x * self.sign_y;

    saturate_i32(if self.width > self.height {
      -product
    } else {
      product
    })
  }

  /// `mp0.SquaredEuclideanNorm()`, the `tangentLength < 0` half of
  /// `diag2` at `direction_45.cpp:130`. `mp0` is axis aligned and its one
  /// non zero component is the difference of the two extents.
  fn straight_leg_squared(self) -> i64 {
    let leg = (self.width - self.height).abs();

    leg * leg
  }
}

/// Append an arc, or the point it collapses to.
///
/// The `if( arc.GetP0() == arc.GetP1() )` guard of `direction_45.cpp:149`,
/// `:164`, `:185` and `:213`. A zero length arc would otherwise enter the
/// chain as a two point polyline, which [`LineChain::append_arc`] demotes
/// to a plain segment (`shape_line_chain.cpp:1620`), leaving the chain one
/// shape short of what the caller asked for.
fn append_arc_or_point(chain: &mut LineChain, arc: &ShapeArc, fallback: Vec2) {
  if arc.start() == arc.end() {
    chain.append(fallback);
  } else {
    chain.append_arc(arc, LineChain::ARC_POLYGONIZATION_MAX_ERROR);
  }
}

/// A chain of plain points, collected the way `SHAPE_LINE_CHAIN::Append`
/// collects them.
///
/// Port of `Append( const VECTOR2I&, bool )`,
/// `libs/kimath/include/geometry/shape_line_chain.h:534`, with the default
/// `aAllowDuplication`, which skips a point equal to the last one already
/// in the chain. `doc/reference/kicad/01-geometry.md` section 14.1 flags
/// that suppression as behaviour the placer depends on.
fn chain_of(points: &[Vec2]) -> LineChain {
  let mut chain = LineChain::new();

  for point in points {
    chain.append(*point);
  }

  chain
}

/// An `i64` narrowed the way KiCad's `int` arithmetic would have carried
/// it, except that this saturates where KiCad wraps.
fn saturate_i32(value: i64) -> i32 {
  value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// Two directions are equal when their octants are, whatever their turn
/// step.
///
/// Port of `operator==`,
/// `libs/kimath/include/geometry/direction45.h:238`, which compares `m_dir`
/// alone. Two undefined directions are equal, which is how the router tests
/// for undefined (`pcbnew/router/pns_line.cpp:1277`).
impl PartialEq for Direction45 {
  fn eq(&self, other: &Self) -> bool {
    self.dir == other.dir
  }
}

/// Port of `Format`, `libs/kimath/include/geometry/direction45.h:129`,
/// which the router uses in its debug messages, for instance
/// `pcbnew/router/pns_line_placer.cpp:1421`. KiCad's `<Error>` case is
/// unreachable here, since `None` is the only value outside the eight
/// octants.
impl fmt::Display for Direction45 {
  fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
    let name = match self.dir {
      Some(Octant::N) => "north",
      Some(Octant::NE) => "north-east",
      Some(Octant::E) => "east",
      Some(Octant::SE) => "south-east",
      Some(Octant::S) => "south",
      Some(Octant::SW) => "south-west",
      Some(Octant::W) => "west",
      Some(Octant::NW) => "north-west",
      None => "undefined",
    };

    formatter.write_str(name)
  }
}

/// Classify a world space vector into an octant.
///
/// Port of `construct_`,
/// `libs/kimath/include/geometry/direction45.h:317`, together with the
/// `vec.y = -vec.y` its callers apply first (`direction45.h:96`). The
/// screen angle is measured with `atan2` and rounded to the nearest octant,
/// so no vector is ever rejected for not being on a 45 degree line: only
/// the zero vector is undefined.
///
/// The float classification is deliberate rather than an integer one over
/// `( sign( x ), sign( y ), |x| vs |y| )`. The two agree on every exact
/// octant vector and differ in how near octant vectors round, and
/// `MOUSE_TRAIL_TRACER` feeds this raw cursor deltas
/// (`pcbnew/router/pns_mouse_trail_tracer.cpp:166`), so the posture the
/// user sees depends on the rounding.
fn construct(world_x: i64, world_y: i64) -> Option<Octant> {
  if world_x == 0 && world_y == 0 {
    return None;
  }

  // KiCad's callers flip y before classifying, north being up on the
  // screen. The flip happens here, in i64, so that i32::MIN does not wrap.
  let x = world_x as f64;
  let y = (-world_y) as f64;

  let mut magnitude =
    360.0 - (180.0 / std::f64::consts::PI * y.atan2(x)) + 90.0;

  if magnitude >= 360.0 {
    magnitude -= 360.0;
  }

  if magnitude < 0.0 {
    magnitude += 360.0;
  }

  // C++ truncates the double towards zero on the conversion to int, and so
  // does the `as` cast.
  let mut index = ((magnitude + 22.5) / 45.0) as i32;

  if index >= i32::from(OCTANT_COUNT) {
    index -= i32::from(OCTANT_COUNT);
  }

  if index < 0 {
    index += i32::from(OCTANT_COUNT);
  }

  Octant::from_index(index)
}

#[cfg(test)]
mod tests {
  use super::*;

  /// The vertex list of an initial trace.
  ///
  /// The tables below predate arcs and compare point lists, which is
  /// still the whole content of a trace in the two mitered modes.
  /// `build_initial_trace_table` is where the four mode contract lives.
  fn trace_points(
    direction: Direction45,
    p0: Vec2,
    p1: Vec2,
    start_diagonal: bool,
    corner_mode: CornerMode,
  ) -> Vec<Vec2> {
    direction
      .build_initial_trace(p0, p1, start_diagonal, corner_mode)
      .points()
      .to_vec()
  }

  /// Shorthand for a defined direction with 45 degree turns.
  fn dir(octant: Octant) -> Direction45 {
    Direction45::from_octant(octant)
  }

  /// The undefined direction, KiCad's `DIRECTION_45()`.
  fn undefined() -> Direction45 {
    Direction45::default()
  }

  /// `ToVector`, `Left`, `Right` and `Opposite` for every octant, written
  /// out from `direction45.h:172`, `:258`, `:276` and `:287`. Left is
  /// counter clockwise on the screen, right is clockwise.
  #[test]
  fn octant_table() {
    let table = [
      (
        Octant::N,
        Vec2::new(0, -1),
        Octant::NW,
        Octant::NE,
        Octant::S,
      ),
      (
        Octant::NE,
        Vec2::new(1, -1),
        Octant::N,
        Octant::E,
        Octant::SW,
      ),
      (
        Octant::E,
        Vec2::new(1, 0),
        Octant::NE,
        Octant::SE,
        Octant::W,
      ),
      (
        Octant::SE,
        Vec2::new(1, 1),
        Octant::E,
        Octant::S,
        Octant::NW,
      ),
      (
        Octant::S,
        Vec2::new(0, 1),
        Octant::SE,
        Octant::SW,
        Octant::N,
      ),
      (
        Octant::SW,
        Vec2::new(-1, 1),
        Octant::S,
        Octant::W,
        Octant::NE,
      ),
      (
        Octant::W,
        Vec2::new(-1, 0),
        Octant::SW,
        Octant::NW,
        Octant::E,
      ),
      (
        Octant::NW,
        Vec2::new(-1, -1),
        Octant::W,
        Octant::N,
        Octant::SE,
      ),
    ];

    for (octant, vector, left, right, opposite) in table {
      assert_eq!(dir(octant).to_vector(), vector, "to_vector of {octant:?}");
      assert_eq!(dir(octant).left(), dir(left), "left of {octant:?}");
      assert_eq!(dir(octant).right(), dir(right), "right of {octant:?}");
      assert_eq!(
        dir(octant).opposite(),
        dir(opposite),
        "opposite of {octant:?}"
      );
    }
  }

  /// The undefined direction has no vector and no turns,
  /// `direction45.h:255`, `:273`, `:300`. `Opposite` is the one place
  /// where KiCad reads out of bounds instead, see the module
  /// documentation.
  #[test]
  fn the_undefined_direction_stays_undefined() {
    assert_eq!(undefined().to_vector(), Vec2::new(0, 0));
    assert_eq!(undefined().left(), undefined());
    assert_eq!(undefined().right(), undefined());
    assert_eq!(undefined().opposite(), undefined());
    assert!(!undefined().is_defined());
    assert!(!undefined().is_diagonal());
    assert!(dir(Octant::N).is_defined());
  }

  /// Only the four odd octants are diagonal, `direction45.h:215`.
  #[test]
  fn diagonal_octants() {
    for octant in Octant::ALL {
      let expected =
        matches!(octant, Octant::NE | Octant::SE | Octant::SW | Octant::NW);

      assert_eq!(dir(octant).is_diagonal(), expected, "{octant:?}");
    }
  }

  /// In 90 degree mode a turn steps by two octants,
  /// `direction45.h:258` and `:276`.
  #[test]
  fn turns_in_ninety_degree_mode() {
    let north = Direction45::from_vector(Vec2::new(0, -1), true);

    assert_eq!(north, dir(Octant::N));
    assert_eq!(north.right(), dir(Octant::E));
    assert_eq!(north.left(), dir(Octant::W));

    let north_east = Direction45::from_vector(Vec2::new(1, -1), true);

    assert_eq!(north_east.right(), dir(Octant::SE));
    assert_eq!(north_east.left(), dir(Octant::NW));
  }

  /// KiCad builds the turned direction with the default constructor and
  /// only assigns `m_dir` (`direction45.h:253`), so the second turn of a
  /// chain steps by 45 degrees even in 90 degree mode. The router chains
  /// two turns at `pcbnew/router/pns_optimizer.cpp:1592`.
  #[test]
  fn a_turn_drops_the_ninety_degree_flag() {
    let north = Direction45::from_vector(Vec2::new(0, -1), true);

    assert_eq!(north.right(), dir(Octant::E));
    assert_eq!(north.right().right(), dir(Octant::SE));
    assert_eq!(north.left().left(), dir(Octant::SW));
  }

  /// KiCad's angle table, `direction45.h:186`, written out by hand from
  /// the rule `d = |a - b|`: 0 straight, 1 and 7 obtuse, 2 and 6 right,
  /// 3 and 5 acute, 4 half full. Rows are the first direction, columns the
  /// second, both in octant order N, NE, E, SE, S, SW, W, NW.
  #[test]
  fn angle_between_every_ordered_pair_of_octants() {
    const S: AngleType = AngleType::STRAIGHT;
    const O: AngleType = AngleType::OBTUSE;
    const R: AngleType = AngleType::RIGHT;
    const A: AngleType = AngleType::ACUTE;
    const H: AngleType = AngleType::HALF_FULL;

    let table: [[AngleType; 8]; 8] = [
      [S, O, R, A, H, A, R, O],
      [O, S, O, R, A, H, A, R],
      [R, O, S, O, R, A, H, A],
      [A, R, O, S, O, R, A, H],
      [H, A, R, O, S, O, R, A],
      [A, H, A, R, O, S, O, R],
      [R, A, H, A, R, O, S, O],
      [O, R, A, H, A, R, O, S],
    ];

    for (row, first) in Octant::ALL.into_iter().enumerate() {
      for (column, second) in Octant::ALL.into_iter().enumerate() {
        let expected = table[row][column];

        assert_eq!(
          dir(first).angle(dir(second)),
          expected,
          "angle of {first:?} to {second:?}"
        );
        assert_eq!(
          dir(first).is_obtuse(dir(second)),
          expected == AngleType::OBTUSE,
          "is_obtuse of {first:?} to {second:?}"
        );
      }
    }
  }

  /// Either direction undefined gives an undefined angle,
  /// `direction45.h:183`.
  #[test]
  fn angle_with_an_undefined_direction() {
    assert_eq!(undefined().angle(dir(Octant::N)), AngleType::UNDEFINED);
    assert_eq!(dir(Octant::N).angle(undefined()), AngleType::UNDEFINED);
    assert_eq!(undefined().angle(undefined()), AngleType::UNDEFINED);
    assert!(!undefined().is_obtuse(dir(Octant::NE)));
  }

  /// The bit values of `direction45.h:79` and the mask idiom the optimizer
  /// uses, `pcbnew/router/pns_optimizer.cpp:1114`.
  #[test]
  fn angle_type_masks() {
    assert_eq!(AngleType::OBTUSE.bits(), 0x01);
    assert_eq!(AngleType::RIGHT.bits(), 0x02);
    assert_eq!(AngleType::ACUTE.bits(), 0x04);
    assert_eq!(AngleType::STRAIGHT.bits(), 0x08);
    assert_eq!(AngleType::HALF_FULL.bits(), 0x10);
    assert_eq!(AngleType::UNDEFINED.bits(), 0x20);

    let forbidden = AngleType::ACUTE
      | AngleType::RIGHT
      | AngleType::HALF_FULL
      | AngleType::UNDEFINED;

    assert_eq!(forbidden.bits(), 0x36);
    assert!(forbidden.contains(AngleType::RIGHT));
    assert!(!forbidden.contains(AngleType::OBTUSE));
    assert!(AngleType::RIGHT.intersects(forbidden));
    assert!(!AngleType::OBTUSE.intersects(forbidden));
    // A single angle does not contain a multi bit mask, which is why the
    // router's `angle & mask` test is `intersects`.
    assert!(!AngleType::RIGHT.contains(forbidden));

    // The `const fn` form answers the same as the operator, which is the
    // whole reason it exists.
    const CONST_FORBIDDEN: AngleType = AngleType::ACUTE
      .union(AngleType::RIGHT)
      .union(AngleType::HALF_FULL)
      .union(AngleType::UNDEFINED);

    assert_eq!(CONST_FORBIDDEN, forbidden);
  }

  /// The eight exact octant vectors, at unit length and scaled,
  /// `direction45.h:317`.
  #[test]
  fn construction_from_exact_octant_vectors() {
    for octant in Octant::ALL {
      let vector = dir(octant).to_vector();

      assert_eq!(
        Direction45::from_vector(vector, false),
        dir(octant),
        "unit vector of {octant:?}"
      );
      assert_eq!(
        Direction45::from_vector(vector * 1_000_000, false),
        dir(octant),
        "scaled vector of {octant:?}"
      );
    }
  }

  /// An independent reference for the octant of a world vector: the screen
  /// angle clockwise from north, rounded to the nearest octant. KiCad
  /// spells the same angle as `360 - atan2( -y, x ) + 90`
  /// (`direction45.h:324`), which is this one modulo a full turn.
  fn reference_octant(vec: Vec2) -> Option<Octant> {
    if vec.x == 0 && vec.y == 0 {
      return None;
    }

    let mut degrees = f64::from(vec.x).atan2(-f64::from(vec.y)).to_degrees();

    if degrees < 0.0 {
      degrees += 360.0;
    }

    Octant::from_index((((degrees + 22.5) / 45.0).floor() as i32) % 8)
  }

  /// Just below and just above each of the eight 22.5 degree boundaries.
  /// The vectors are a million nanometres long and a thousandth of a
  /// degree off the boundary, which is about 17 nm across the boundary,
  /// far more than the nanometre the rounding to integer coordinates can
  /// move them.
  #[test]
  fn construction_rounds_at_the_octant_boundaries() {
    const RADIUS: f64 = 1_000_000.0;
    const EPSILON_DEGREES: f64 = 0.001;

    for boundary in 0..8 {
      let degrees = 22.5 + 45.0 * f64::from(boundary);

      for (offset, expected_index) in [
        (-EPSILON_DEGREES, boundary),
        (EPSILON_DEGREES, boundary + 1),
      ] {
        let radians = (degrees + offset).to_radians();
        // North is up on the screen, so a positive screen angle turns from
        // (0, -1) towards (1, 0).
        let vector = Vec2::new(
          (RADIUS * radians.sin()).round() as i32,
          (-RADIUS * radians.cos()).round() as i32,
        );
        let expected = Octant::from_index(expected_index % 8);

        assert_eq!(
          Direction45::from_vector(vector, false).octant(),
          expected,
          "{degrees} degrees offset by {offset}, vector {vector:?}"
        );
        assert_eq!(
          Direction45::from_vector(vector, false).octant(),
          reference_octant(vector),
          "atan2 reference at {degrees} degrees offset by {offset}"
        );
      }
    }
  }

  /// Only the zero vector is undefined, `direction45.h:321`. Even a vector
  /// one nanometre off an axis classifies.
  #[test]
  fn construction_from_the_zero_vector_and_its_neighbours() {
    assert_eq!(
      Direction45::from_vector(Vec2::new(0, 0), false),
      undefined()
    );
    assert_eq!(
      Direction45::from_vector(Vec2::new(1, 0), false),
      dir(Octant::E)
    );
    assert_eq!(
      Direction45::from_vector(Vec2::new(0, -1), false),
      dir(Octant::N)
    );
    assert_eq!(
      Direction45::from_vector(Vec2::new(-1, 0), false),
      dir(Octant::W)
    );
  }

  /// The 90 degree flag does not reach `construct_`, it only changes the
  /// turns (`direction45.h:92`).
  #[test]
  fn construction_in_ninety_degree_mode_classifies_the_same() {
    for octant in Octant::ALL {
      let vector = dir(octant).to_vector() * 12_345;

      assert_eq!(
        Direction45::from_vector(vector, true),
        Direction45::from_vector(vector, false),
        "{octant:?}"
      );
    }

    // A vector between two octants rounds the same way in both modes.
    let between = Vec2::new(1000, -300);

    assert_eq!(
      Direction45::from_vector(between, true),
      Direction45::from_vector(between, false)
    );
  }

  /// A segment classifies as the vector from its first to its second
  /// point, `direction45.h:103`.
  #[test]
  fn construction_from_a_segment() {
    let seg = Seg::from_coords(100, 100, 1100, -900);

    assert_eq!(Direction45::from_seg(&seg, false), dir(Octant::NE));
    assert_eq!(
      Direction45::from_seg(&seg.reversed(), false),
      dir(Octant::SW)
    );
    assert_eq!(
      Direction45::from_seg(&Seg::from_coords(5, 5, 5, 5), false),
      undefined()
    );

    // KiCad takes `B - A` in `int`, which would wrap here.
    let wide = Seg::from_coords(i32::MIN, 0, i32::MAX, 0);

    assert_eq!(Direction45::from_seg(&wide, false), dir(Octant::E));
  }

  /// The numeric round trip of `direction45.h:48`, and the out of range
  /// value `pcbnew/router/pns_line_placer.cpp:1417` can produce.
  #[test]
  fn octant_indices() {
    for (index, octant) in Octant::ALL.into_iter().enumerate() {
      assert_eq!(i32::from(octant.index()), index as i32);
      assert_eq!(Octant::from_index(index as i32), Some(octant));
    }

    assert_eq!(Octant::from_index(8), None);
    assert_eq!(Octant::from_index(-1), None);
  }

  /// `operator==` compares the octant alone, `direction45.h:238`.
  #[test]
  fn equality_ignores_the_turn_step() {
    let north_45 = Direction45::from_vector(Vec2::new(0, -1), false);
    let north_90 = Direction45::from_vector(Vec2::new(0, -1), true);

    assert_eq!(north_45, north_90);
    assert_ne!(north_45, dir(Octant::NE));
    assert_ne!(north_45, undefined());
    assert_eq!(undefined(), undefined());
  }

  /// The names of `Format`, `direction45.h:129`.
  #[test]
  fn display_matches_the_kicad_names() {
    let names = [
      "north",
      "north-east",
      "east",
      "south-east",
      "south",
      "south-west",
      "west",
      "north-west",
    ];

    for (octant, name) in Octant::ALL.into_iter().zip(names) {
      assert_eq!(dir(octant).to_string(), name);
    }

    assert_eq!(undefined().to_string(), "undefined");
  }

  /// The single segment shortcut of `direction_45.cpp:46`: an axis aligned
  /// pair in either mode, an exact diagonal in 45 degree mode, and the
  /// degenerate pair whose second `Append` is suppressed.
  #[test]
  fn build_initial_trace_single_segment_cases() {
    let origin = Vec2::new(0, 0);

    for corner_mode in [CornerMode::Mitered45, CornerMode::Mitered90] {
      for start_diagonal in [false, true] {
        assert_eq!(
          trace_points(
            undefined(),
            origin,
            Vec2::new(1000, 0),
            start_diagonal,
            corner_mode
          ),
          vec![origin, Vec2::new(1000, 0)]
        );
        assert_eq!(
          trace_points(
            undefined(),
            origin,
            Vec2::new(0, -1000),
            start_diagonal,
            corner_mode
          ),
          vec![origin, Vec2::new(0, -1000)]
        );
        assert_eq!(
          trace_points(
            undefined(),
            origin,
            origin,
            start_diagonal,
            corner_mode
          ),
          vec![origin],
          "a degenerate pair collapses to one point"
        );
      }

      // The exact diagonal is a single segment in 45 degree mode only.
      let diagonal = trace_points(
        undefined(),
        origin,
        Vec2::new(500, 500),
        false,
        corner_mode,
      );

      if corner_mode == CornerMode::Mitered45 {
        assert_eq!(diagonal, vec![origin, Vec2::new(500, 500)]);
      } else {
        assert_eq!(
          diagonal,
          vec![origin, Vec2::new(0, 500), Vec2::new(500, 500)]
        );
      }
    }
  }

  /// The mitered 45 degree corner, `direction_45.cpp:69` and `:100`, in
  /// both postures and in all four quadrants. The wide leg is horizontal
  /// when the pair is wider than it is tall, vertical otherwise.
  #[test]
  fn build_initial_trace_mitered_45_in_every_quadrant() {
    let origin = Vec2::new(0, 0);

    // (end point, straight first corner, diagonal first corner)
    let wide = [
      (Vec2::new(1000, 300), Vec2::new(700, 0), Vec2::new(300, 300)),
      (
        Vec2::new(-1000, 300),
        Vec2::new(-700, 0),
        Vec2::new(-300, 300),
      ),
      (
        Vec2::new(1000, -300),
        Vec2::new(700, 0),
        Vec2::new(300, -300),
      ),
      (
        Vec2::new(-1000, -300),
        Vec2::new(-700, 0),
        Vec2::new(-300, -300),
      ),
    ];

    // The same four with the axes swapped, so the straight leg is vertical.
    let tall = [
      (Vec2::new(300, 1000), Vec2::new(0, 700), Vec2::new(300, 300)),
      (
        Vec2::new(-300, 1000),
        Vec2::new(0, 700),
        Vec2::new(-300, 300),
      ),
      (
        Vec2::new(300, -1000),
        Vec2::new(0, -700),
        Vec2::new(300, -300),
      ),
      (
        Vec2::new(-300, -1000),
        Vec2::new(0, -700),
        Vec2::new(-300, -300),
      ),
    ];

    for (end, straight, diagonal) in wide.into_iter().chain(tall) {
      assert_eq!(
        trace_points(undefined(), origin, end, false, CornerMode::Mitered45),
        vec![origin, straight, end],
        "straight first to {end:?}"
      );
      assert_eq!(
        trace_points(undefined(), origin, end, true, CornerMode::Mitered45),
        vec![origin, diagonal, end],
        "diagonal first to {end:?}"
      );
    }
  }

  /// The trace does not start at the origin, `direction_45.cpp:101` adds
  /// the corner to `aP0`.
  #[test]
  fn build_initial_trace_is_relative_to_the_start_point() {
    let start = Vec2::new(-5000, 7000);
    let end = Vec2::new(-4000, 7300);

    assert_eq!(
      trace_points(undefined(), start, end, false, CornerMode::Mitered45),
      vec![start, Vec2::new(-4300, 7000), end]
    );
    assert_eq!(
      trace_points(undefined(), start, end, true, CornerMode::Mitered45),
      vec![start, Vec2::new(-4700, 7300), end]
    );
  }

  /// A defined direction imposes the posture and ignores the flag,
  /// `direction_45.cpp:31`.
  #[test]
  fn build_initial_trace_follows_a_defined_direction() {
    let origin = Vec2::new(0, 0);
    let end = Vec2::new(1000, 300);

    for start_diagonal in [false, true] {
      assert_eq!(
        trace_points(
          dir(Octant::N),
          origin,
          end,
          start_diagonal,
          CornerMode::Mitered45
        ),
        vec![origin, Vec2::new(700, 0), end],
        "a straight direction always starts straight"
      );
      assert_eq!(
        trace_points(
          dir(Octant::NE),
          origin,
          end,
          start_diagonal,
          CornerMode::Mitered45
        ),
        vec![origin, Vec2::new(300, 300), end],
        "a diagonal direction always starts diagonal"
      );
    }
  }

  /// The mitered 90 degree corner, `direction_45.cpp:56` and `:234`. The
  /// flag asks for the shorter leg first (`direction45.h:230`), so it
  /// picks the horizontal leg exactly when it is the shorter one.
  #[test]
  fn build_initial_trace_mitered_90_in_every_quadrant() {
    let origin = Vec2::new(0, 0);

    // (end point, corner without the flag, corner with the flag)
    let table = [
      (Vec2::new(1000, 300), Vec2::new(1000, 0), Vec2::new(0, 300)),
      (
        Vec2::new(-1000, -300),
        Vec2::new(-1000, 0),
        Vec2::new(0, -300),
      ),
      (Vec2::new(300, 1000), Vec2::new(0, 1000), Vec2::new(300, 0)),
      (
        Vec2::new(-300, -1000),
        Vec2::new(0, -1000),
        Vec2::new(-300, 0),
      ),
      (
        Vec2::new(1000, -300),
        Vec2::new(1000, 0),
        Vec2::new(0, -300),
      ),
      (
        Vec2::new(-300, 1000),
        Vec2::new(0, 1000),
        Vec2::new(-300, 0),
      ),
    ];

    for (end, straight, diagonal) in table {
      assert_eq!(
        trace_points(undefined(), origin, end, false, CornerMode::Mitered90),
        vec![origin, straight, end],
        "long leg first to {end:?}"
      );
      assert_eq!(
        trace_points(undefined(), origin, end, true, CornerMode::Mitered90),
        vec![origin, diagonal, end],
        "short leg first to {end:?}"
      );
    }
  }

  /// The mitered corner is exact integer arithmetic, which is why the
  /// `SHAPE_ARC::MIN_PRECISION_IU` endpoint re-snap of
  /// `direction_45.cpp:200` to `:211` has no counterpart here: that
  /// re-snap only repairs an arc built from a centre, in the `ROUNDED_45`
  /// branch. The endpoints come back untouched and the corner lies exactly
  /// on the axis through the start point, with the diagonal leg at exactly
  /// 45 degrees.
  #[test]
  fn build_initial_trace_keeps_its_endpoints_and_corner_exact() {
    let start = Vec2::new(123_457, -987_659);
    let end = Vec2::new(4_567_891, -1_234_567);
    let trace =
      trace_points(undefined(), start, end, false, CornerMode::Mitered45);

    assert_eq!(trace.len(), 3);
    assert_eq!(trace[0], start);
    assert_eq!(trace[2], end);
    assert_eq!(trace[1].y, start.y, "the straight leg stays on the axis");

    let diagonal = trace[2].widening_sub(trace[1]);

    assert_eq!(diagonal.x.abs(), diagonal.y.abs());
    assert_eq!(
      Direction45::from_seg(&Seg::new(trace[1], trace[2]), false),
      dir(Octant::NE)
    );
  }

  /// KiCad takes the coordinate differences in `int`
  /// (`direction_45.cpp:36`), which wraps for a pair spanning the whole
  /// coordinate range. The widened difference gets it right, and every
  /// vertex still fits in `i32`.
  #[test]
  fn build_initial_trace_spanning_the_coordinate_range() {
    let start = Vec2::new(i32::MIN, 0);
    let end = Vec2::new(i32::MAX, 100);

    assert_eq!(
      trace_points(undefined(), start, end, false, CornerMode::Mitered45),
      vec![start, Vec2::new(i32::MAX - 100, 0), end]
    );
    assert_eq!(
      trace_points(undefined(), start, end, true, CornerMode::Mitered45),
      vec![start, Vec2::new(i32::MIN + 100, 100), end]
    );
  }

  // ===============================================================
  // The four mode `build_initial_trace` contract
  // ===============================================================
  //
  // KiCad has no test for `BuildInitialTrace` at all (note 01 section
  // 15, note 09 section 8.4 item 6), so the four tables below are the
  // port's own contract rather than a mirror of one.
  //
  // How the expected values were derived. Every row was computed from
  // `libs/kimath/src/geometry/direction_45.cpp` by hand, through a
  // transcription of the C++ into a throwaway script, never by running
  // this port. The steps, for a row with end point `(dx, dy)` taken from
  // the start point:
  //
  //   w  = |dx|,  h  = |dy|,  sw = sign( dx ),  sh = sign( dy )   :35
  //
  // `w == 0`, `h == 0`, or `w == h` outside a 90 degree mode answers
  // `[ p0, p1 ]` and stops (`:46`). Otherwise, for the 45 degree modes
  // (`:69` to `:79`):
  //
  //   w > h:   mp0 = ( (w-h) * sw, 0 )      mp1 = ( h * sw, h * sh )
  //   w <= h:  mp0 = ( 0, (h-w) * sh )      mp1 = ( w * sw, w * sh )
  //   tangentLength = |w - h| - KiROUND( |mp1| )
  //
  // `|mp1|` is an exact diagonal, so `EuclideanNorm` takes
  // `KiROUND( leg * sqrt( 2 ) )` without a `hypot`
  // (`libs/kimath/include/math/vector2d.h:279`). For the 90 degree modes
  // `mp0` is `( w * sw, 0 )` when `startDiagonal == ( h >= w )` and
  // `( 0, h * sh )` otherwise (`:58` to `:66`), and `mp1` stays zero.
  //
  // `MITERED_45` appends `p0`, `p0 + ( startDiagonal ? mp1 : mp0 )`,
  // `p1` (`:97`). `MITERED_90` appends `p0`, `p0 + mp0`, `p1` (`:232`).
  //
  // `ROUNDED_45` picks one of four sub-cases on the posture and the sign
  // of `tangentLength`; `Direction45::rounded_45` has the table with the
  // line numbers. Three of them build the arc with
  // `ConstructFromStartEndAngle( start, end, +/- 45 * rotationSign )`
  // (`shape_arc.cpp:197`), which places the centre with
  // `CalcArcCenter( start, end, angle )` (`trigo.cpp:329`), truncates it
  // into `i32`, and takes the mid point as the start rotated about that
  // centre by half the angle. The fourth builds
  // `SHAPE_ARC( centre, p0, -45 * rotationSign )` (`shape_arc.cpp:41`)
  // with `centre = p0 + centreDir.Resize( arcRadius )` in floating point,
  // truncated, and then applies the `MIN_PRECISION_IU` re-snap.
  //
  // `ROUNDED_90` has five cases, all built with
  // `ConstructFromStartEndCenter( start, end, centre, clockwise )`
  // (`shape_arc.cpp:216`), whose centres are exact integer points;
  // `Direction45::rounded_90` has that table.
  //
  // What a row states, and what it deliberately does not. A row lists the
  // **shapes** the chain has to hold, in order: plain vertices and, for
  // the rounded modes, the arc with its three points. The runner also
  // compares the full vertex list against the one those shapes produce
  // through `LineChain::append_arc`, so nothing may be added or dropped,
  // but the interior vertices of an arc's approximation are not written
  // out. Those belong to `ShapeArc::convert_to_polyline` and
  // `LineChain::append_arc`, both mirrored against KiCad's own tests in
  // slices 1 and 3; repeating them here would pin the accuracy constant
  // in a table about corner geometry.
  //
  // Every table is run twice, from the origin and from an offset start,
  // because the whole routine is relative to `aP0`.
  // ---------------------------------------------------------------
  // The four mode `build_initial_trace` table
  // ---------------------------------------------------------------

  /// One shape of an expected trace.
  ///
  /// A trace is a sequence of these: `BuildInitialTrace` appends plain
  /// points and, in the two rounded modes, at most one arc.
  #[derive(Copy, Clone, Debug, PartialEq, Eq)]
  enum Step {
    /// `pl.Append( p )`.
    Point(Vec2),
    /// `pl.Append( arc )`, with the arc's start, mid and end.
    Arc(Vec2, Vec2, Vec2),
  }

  /// `Step::Point`, short enough for a table row.
  const fn p(x: i32, y: i32) -> Step {
    Step::Point(Vec2::new(x, y))
  }

  /// `Step::Arc`, short enough for a table row.
  const fn a(sx: i32, sy: i32, mx: i32, my: i32, ex: i32, ey: i32) -> Step {
    Step::Arc(Vec2::new(sx, sy), Vec2::new(mx, my), Vec2::new(ex, ey))
  }

  /// A table row: the end point relative to the start, in the quadrant
  /// where both coordinates are positive or zero, the posture, and the
  /// shapes the trace must hold.
  type Row = (i32, i32, bool, &'static [Step]);

  const MITERED_45_TABLE: &[Row] = &[
    // |dx| > |dy|, a positive tangent length
    (
      3_000_000,
      1_000_000,
      false,
      &[p(0, 0), p(2_000_000, 0), p(3_000_000, 1_000_000)],
    ),
    (
      3_000_000,
      1_000_000,
      true,
      &[p(0, 0), p(1_000_000, 1_000_000), p(3_000_000, 1_000_000)],
    ),
    // |dx| < |dy|, a positive tangent length
    (
      1_000_000,
      3_000_000,
      false,
      &[p(0, 0), p(0, 2_000_000), p(1_000_000, 3_000_000)],
    ),
    (
      1_000_000,
      3_000_000,
      true,
      &[p(0, 0), p(1_000_000, 1_000_000), p(1_000_000, 3_000_000)],
    ),
    // |dx| > |dy|, a negative tangent length
    (
      3_000_000,
      2_000_000,
      false,
      &[p(0, 0), p(1_000_000, 0), p(3_000_000, 2_000_000)],
    ),
    (
      3_000_000,
      2_000_000,
      true,
      &[p(0, 0), p(2_000_000, 2_000_000), p(3_000_000, 2_000_000)],
    ),
    // |dx| < |dy|, a negative tangent length
    (
      2_000_000,
      3_000_000,
      false,
      &[p(0, 0), p(0, 1_000_000), p(2_000_000, 3_000_000)],
    ),
    (
      2_000_000,
      3_000_000,
      true,
      &[p(0, 0), p(2_000_000, 2_000_000), p(2_000_000, 3_000_000)],
    ),
    // |dx| == |dy|
    (
      2_000_000,
      2_000_000,
      false,
      &[p(0, 0), p(2_000_000, 2_000_000)],
    ),
    (
      2_000_000,
      2_000_000,
      true,
      &[p(0, 0), p(2_000_000, 2_000_000)],
    ),
    // dy == 0
    (2_000_000, 0, false, &[p(0, 0), p(2_000_000, 0)]),
    (2_000_000, 0, true, &[p(0, 0), p(2_000_000, 0)]),
    // dx == 0
    (0, 2_000_000, false, &[p(0, 0), p(0, 2_000_000)]),
    (0, 2_000_000, true, &[p(0, 0), p(0, 2_000_000)]),
  ];

  const ROUNDED_45_TABLE: &[Row] = &[
    // |dx| > |dy|, a positive tangent length
    (
      3_000_000,
      1_000_000,
      false,
      &[
        p(0, 0),
        a(585_786, 0, 1_892_349, 259_892, 3_000_000, 1_000_000),
      ],
    ),
    (
      3_000_000,
      1_000_000,
      true,
      &[
        a(0, 0, 1_107_651, 740_108, 2_414_214, 1_000_000),
        p(3_000_000, 1_000_000),
      ],
    ),
    // |dx| < |dy|, a positive tangent length
    (
      1_000_000,
      3_000_000,
      false,
      &[
        p(0, 0),
        a(0, 585_786, 259_892, 1_892_349, 1_000_000, 3_000_000),
      ],
    ),
    (
      1_000_000,
      3_000_000,
      true,
      &[
        a(0, 0, 740_108, 1_107_651, 1_000_000, 2_414_214),
        p(1_000_000, 3_000_000),
      ],
    ),
    // |dx| > |dy|, a negative tangent length
    (
      3_000_000,
      2_000_000,
      false,
      &[
        a(0, 0, 923_880, 183_771, 1_707_107, 707_107),
        p(3_000_000, 2_000_000),
      ],
    ),
    (
      3_000_000,
      2_000_000,
      true,
      &[
        p(0, 0),
        a(
          1_292_893, 1_292_893, 2_076_120, 1_816_229, 3_000_000, 2_000_000,
        ),
      ],
    ),
    // |dx| < |dy|, a negative tangent length
    (
      2_000_000,
      3_000_000,
      false,
      &[
        a(0, 0, 183_771, 923_880, 707_107, 1_707_107),
        p(2_000_000, 3_000_000),
      ],
    ),
    (
      2_000_000,
      3_000_000,
      true,
      &[
        p(0, 0),
        a(
          1_292_893, 1_292_893, 1_816_229, 2_076_120, 2_000_000, 3_000_000,
        ),
      ],
    ),
    // |dx| == |dy|
    (
      2_000_000,
      2_000_000,
      false,
      &[p(0, 0), p(2_000_000, 2_000_000)],
    ),
    (
      2_000_000,
      2_000_000,
      true,
      &[p(0, 0), p(2_000_000, 2_000_000)],
    ),
    // dy == 0
    (2_000_000, 0, false, &[p(0, 0), p(2_000_000, 0)]),
    (2_000_000, 0, true, &[p(0, 0), p(2_000_000, 0)]),
    // dx == 0
    (0, 2_000_000, false, &[p(0, 0), p(0, 2_000_000)]),
    (0, 2_000_000, true, &[p(0, 0), p(0, 2_000_000)]),
  ];

  const MITERED_90_TABLE: &[Row] = &[
    // |dx| > |dy|, a positive tangent length
    (
      3_000_000,
      1_000_000,
      false,
      &[p(0, 0), p(3_000_000, 0), p(3_000_000, 1_000_000)],
    ),
    (
      3_000_000,
      1_000_000,
      true,
      &[p(0, 0), p(0, 1_000_000), p(3_000_000, 1_000_000)],
    ),
    // |dx| < |dy|, a positive tangent length
    (
      1_000_000,
      3_000_000,
      false,
      &[p(0, 0), p(0, 3_000_000), p(1_000_000, 3_000_000)],
    ),
    (
      1_000_000,
      3_000_000,
      true,
      &[p(0, 0), p(1_000_000, 0), p(1_000_000, 3_000_000)],
    ),
    // |dx| > |dy|, a negative tangent length
    (
      3_000_000,
      2_000_000,
      false,
      &[p(0, 0), p(3_000_000, 0), p(3_000_000, 2_000_000)],
    ),
    (
      3_000_000,
      2_000_000,
      true,
      &[p(0, 0), p(0, 2_000_000), p(3_000_000, 2_000_000)],
    ),
    // |dx| < |dy|, a negative tangent length
    (
      2_000_000,
      3_000_000,
      false,
      &[p(0, 0), p(0, 3_000_000), p(2_000_000, 3_000_000)],
    ),
    (
      2_000_000,
      3_000_000,
      true,
      &[p(0, 0), p(2_000_000, 0), p(2_000_000, 3_000_000)],
    ),
    // |dx| == |dy|
    (
      2_000_000,
      2_000_000,
      false,
      &[p(0, 0), p(0, 2_000_000), p(2_000_000, 2_000_000)],
    ),
    (
      2_000_000,
      2_000_000,
      true,
      &[p(0, 0), p(2_000_000, 0), p(2_000_000, 2_000_000)],
    ),
    // dy == 0
    (2_000_000, 0, false, &[p(0, 0), p(2_000_000, 0)]),
    (2_000_000, 0, true, &[p(0, 0), p(2_000_000, 0)]),
    // dx == 0
    (0, 2_000_000, false, &[p(0, 0), p(0, 2_000_000)]),
    (0, 2_000_000, true, &[p(0, 0), p(0, 2_000_000)]),
  ];

  const ROUNDED_90_TABLE: &[Row] = &[
    // |dx| > |dy|, a positive tangent length
    (
      3_000_000,
      1_000_000,
      false,
      &[
        p(0, 0),
        a(2_000_000, 0, 2_707_107, 292_893, 3_000_000, 1_000_000),
      ],
    ),
    (
      3_000_000,
      1_000_000,
      true,
      &[
        a(0, 0, 292_893, 707_107, 1_000_000, 1_000_000),
        p(3_000_000, 1_000_000),
      ],
    ),
    // |dx| < |dy|, a positive tangent length
    (
      1_000_000,
      3_000_000,
      false,
      &[
        p(0, 0),
        a(0, 2_000_000, 292_893, 2_707_107, 1_000_000, 3_000_000),
      ],
    ),
    (
      1_000_000,
      3_000_000,
      true,
      &[
        a(0, 0, 707_107, 292_893, 1_000_000, 1_000_000),
        p(1_000_000, 3_000_000),
      ],
    ),
    // |dx| > |dy|, a negative tangent length
    (
      3_000_000,
      2_000_000,
      false,
      &[
        p(0, 0),
        a(1_000_000, 0, 2_414_214, 585_786, 3_000_000, 2_000_000),
      ],
    ),
    (
      3_000_000,
      2_000_000,
      true,
      &[
        a(0, 0, 585_786, 1_414_214, 2_000_000, 2_000_000),
        p(3_000_000, 2_000_000),
      ],
    ),
    // |dx| < |dy|, a negative tangent length
    (
      2_000_000,
      3_000_000,
      false,
      &[
        p(0, 0),
        a(0, 1_000_000, 585_786, 2_414_214, 2_000_000, 3_000_000),
      ],
    ),
    (
      2_000_000,
      3_000_000,
      true,
      &[
        a(0, 0, 1_414_214, 585_786, 2_000_000, 2_000_000),
        p(2_000_000, 3_000_000),
      ],
    ),
    // |dx| == |dy|
    (
      2_000_000,
      2_000_000,
      false,
      &[a(0, 0, 585_786, 1_414_214, 2_000_000, 2_000_000)],
    ),
    (
      2_000_000,
      2_000_000,
      true,
      &[a(0, 0, 1_414_214, 585_786, 2_000_000, 2_000_000)],
    ),
    // dy == 0
    (2_000_000, 0, false, &[p(0, 0), p(2_000_000, 0)]),
    (2_000_000, 0, true, &[p(0, 0), p(2_000_000, 0)]),
    // dx == 0
    (0, 2_000_000, false, &[p(0, 0), p(0, 2_000_000)]),
    (0, 2_000_000, true, &[p(0, 0), p(0, 2_000_000)]),
  ];
  /// Rebuild the chain a row describes, so that the point list a row
  /// implies can be compared against the real one.
  fn chain_of_steps(steps: &[Step], origin: Vec2) -> LineChain {
    let mut chain = LineChain::new();

    for step in steps {
      match *step {
        Step::Point(point) => chain.append(point + origin),
        Step::Arc(start, mid, end) => chain.append_arc(
          &ShapeArc::new(start + origin, mid + origin, end + origin, 0),
          LineChain::ARC_POLYGONIZATION_MAX_ERROR,
        ),
      }
    }

    chain
  }

  /// The shapes a chain holds, in order, as a row describes them.
  fn steps_of(chain: &LineChain, origin: Vec2) -> Vec<Step> {
    let mut steps = Vec::new();
    let mut last_arc: Option<usize> = None;

    for index in 0..chain.point_count() {
      match chain.arc_index(index) {
        Some(arc) => {
          if last_arc == Some(arc) {
            continue;
          }

          let shape = chain.arc(arc).expect("the index came from the chain");

          steps.push(Step::Arc(
            shape.start() - origin,
            shape.arc_mid() - origin,
            shape.end() - origin,
          ));
          last_arc = Some(arc);
        }
        None => steps.push(Step::Point(chain.point(index) - origin)),
      }
    }

    steps
  }

  /// The start point every table is written against.
  const TABLE_ORIGIN: Vec2 = Vec2::new(0, 0);

  /// A second start point, to check that the routine is relative to
  /// `aP0`.
  const TABLE_OFFSET: Vec2 = Vec2::new(-1_234_567, 890_123);

  /// How far a translated arc point may land from the translated
  /// expectation.
  ///
  /// `ConstructFromStartEndAngle` truncates its centre into `i32`
  /// (`shape_arc.cpp:205` through the narrowing conversion at
  /// `libs/kimath/include/math/vector2d.h:85`), and truncation towards
  /// zero is not a translation invariant rounding: the same arc built
  /// where the centre has negative coordinates rounds the other way, and
  /// the mid point can land one nanometre off.
  /// `a_rounded_45_trace_is_not_quite_translation_invariant` pins the
  /// case this number exists for. The endpoints are exact either way,
  /// being stored rather than computed.
  const TRANSLATION_SLACK: i32 = 1;

  /// One expected shape reflected about the axes.
  fn mirrored(step: Step, sign_x: i32, sign_y: i32) -> Step {
    let flip = |point: Vec2| Vec2::new(point.x * sign_x, point.y * sign_y);

    match step {
      Step::Point(point) => Step::Point(flip(point)),
      Step::Arc(start, mid, end) => {
        Step::Arc(flip(start), flip(mid), flip(end))
      }
    }
  }

  /// Check one table against `build_initial_trace`.
  ///
  /// Each row is run twelve times: in all four sign combinations of the
  /// end point, and each of those from the origin and from an offset
  /// start.
  ///
  /// The table states the row for the quadrant where both coordinates
  /// are positive, and the other three are its exact reflections. That
  /// is an assertion in its own right, not a shortcut: `w`, `h` and the
  /// two signs enter `BuildInitialTrace` separately, and every vertex it
  /// builds is a sum of terms each carrying one of the two signs, so a
  /// reflection of the input has to give the reflection of the output to
  /// the nanometre. Writing all four quadrants out would have said the
  /// same thing four times as long and would have hidden a broken
  /// symmetry inside a wall of literals.
  fn check_table(table: &[Row], corner_mode: CornerMode) {
    for &(dx, dy, start_diagonal, steps) in table {
      for sign_x in [1, -1] {
        for sign_y in [1, -1] {
          let delta = Vec2::new(dx * sign_x, dy * sign_y);
          let expected: Vec<Step> = steps
            .iter()
            .map(|step| mirrored(*step, sign_x, sign_y))
            .collect();

          check_row(corner_mode, delta, start_diagonal, &expected);
        }
      }
    }
  }

  /// One row of [`check_table`], in one sign combination.
  fn check_row(
    corner_mode: CornerMode,
    delta: Vec2,
    start_diagonal: bool,
    steps: &[Step],
  ) {
    let context =
      format!("{corner_mode:?} to {delta:?} diagonal {start_diagonal}");
    let chain = undefined().build_initial_trace(
      TABLE_ORIGIN,
      TABLE_ORIGIN + delta,
      start_diagonal,
      corner_mode,
    );

    assert_eq!(steps_of(&chain, TABLE_ORIGIN), steps, "shapes, {context}");
    assert_eq!(
      chain.points(),
      chain_of_steps(steps, TABLE_ORIGIN).points(),
      "points, {context}"
    );
    assert_eq!(chain.point(0), TABLE_ORIGIN, "start, {context}");
    assert_eq!(
      chain.last_point(),
      Some(TABLE_ORIGIN + delta),
      "end, {context}"
    );

    let arcs = steps
      .iter()
      .filter(|step| matches!(step, Step::Arc(..)))
      .count();

    assert_eq!(chain.arc_count(), arcs, "arc count, {context}");
    assert!(!chain.is_closed(), "closed, {context}");

    // The same row from a start point that is not the origin.
    let moved = undefined().build_initial_trace(
      TABLE_OFFSET,
      TABLE_OFFSET + delta,
      start_diagonal,
      corner_mode,
    );

    assert_eq!(moved.point(0), TABLE_OFFSET, "moved start, {context}");
    assert_eq!(
      moved.last_point(),
      Some(TABLE_OFFSET + delta),
      "moved end, {context}"
    );
    assert_eq!(moved.arc_count(), arcs, "moved arc count, {context}");

    for (expected, actual) in steps.iter().zip(steps_of(&moved, TABLE_OFFSET)) {
      match (*expected, actual) {
        (Step::Point(want), Step::Point(got)) => {
          assert_eq!(want, got, "moved point, {context}");
        }
        (
          Step::Arc(want_start, want_mid, want_end),
          Step::Arc(got_start, got_mid, got_end),
        ) => {
          for (want, got) in [
            (want_start, got_start),
            (want_mid, got_mid),
            (want_end, got_end),
          ] {
            assert!(
              (want.x - got.x).abs() <= TRANSLATION_SLACK
                && (want.y - got.y).abs() <= TRANSLATION_SLACK,
              "moved arc point {got:?} is not {want:?}, {context}"
            );
          }
        }
        (want, got) => {
          panic!("moved shape {got:?} is not {want:?}, {context}")
        }
      }
    }
  }

  #[test]
  fn build_initial_trace_table_mitered_45() {
    check_table(MITERED_45_TABLE, CornerMode::Mitered45);
  }

  #[test]
  fn build_initial_trace_table_rounded_45() {
    check_table(ROUNDED_45_TABLE, CornerMode::Rounded45);
  }

  #[test]
  fn build_initial_trace_table_mitered_90() {
    check_table(MITERED_90_TABLE, CornerMode::Mitered90);
  }

  #[test]
  fn build_initial_trace_table_rounded_90() {
    check_table(ROUNDED_90_TABLE, CornerMode::Rounded90);
  }

  // ---------------------------------------------------------------
  // The `ROUNDED_45` errata
  // ---------------------------------------------------------------

  /// The only arc a rounded trace holds, or nothing.
  fn only_arc(chain: &LineChain) -> ShapeArc {
    assert_eq!(chain.arc_count(), 1, "the trace holds exactly one arc");
    chain.arc(0).expect("the count says there is one")
  }

  /// Erratum E17. `ROUNDED_45` computes `diag2`, `diagLength` and
  /// `arcRadius` unconditionally (`direction_45.cpp:130` to `:132`) and
  /// reads `arcRadius` in exactly one of its four sub-cases, the
  /// `!startDiagonal && tangentLength < 0` one at `:193`.
  ///
  /// The port computes them inside that sub-case. Nothing is observable
  /// either way, which is what this test says: the three sub-cases that
  /// do not use the radius are reproduced exactly by
  /// `ConstructFromStartEndAngle` of their two endpoints, and the fourth
  /// is reproduced exactly by the centre the radius places.
  #[test]
  fn rounded_45_computes_the_arc_radius_only_where_it_is_used_erratum_e17() {
    let origin = Vec2::new(0, 0);
    let quarter = Degrees::EIGHTH_TURN;

    // `!startDiagonal`, `tangentLength = 585786 >= 0` (`:177`). The arc
    // runs from `p0 + mp0.Resize( tangentLength )` to `p1`, sweeping
    // `-45 * rotationSign` with `rotationSign = -1` because `w > h`.
    let straight = undefined().build_initial_trace(
      origin,
      Vec2::new(3_000_000, 1_000_000),
      false,
      CornerMode::Rounded45,
    );

    assert_eq!(
      only_arc(&straight),
      ShapeArc::from_start_end_angle(
        Vec2::new(585_786, 0),
        Vec2::new(3_000_000, 1_000_000),
        quarter,
        0,
      )
    );

    // `startDiagonal`, `tangentLength >= 0` (`:143`). From `p0` to
    // `p1 - mp0.Resize( tangentLength )`, sweeping `+45 * rotationSign`.
    let diagonal = undefined().build_initial_trace(
      origin,
      Vec2::new(3_000_000, 1_000_000),
      true,
      CornerMode::Rounded45,
    );

    assert_eq!(
      only_arc(&diagonal),
      ShapeArc::from_start_end_angle(
        origin,
        Vec2::new(2_414_214, 1_000_000),
        -quarter,
        0,
      )
    );

    // `startDiagonal`, `tangentLength = -1828427 < 0` (`:156`). From
    // `p0 + mp1.Resize( |tangentLength| )` to `p1`.
    let diagonal_negative = undefined().build_initial_trace(
      origin,
      Vec2::new(3_000_000, 2_000_000),
      true,
      CornerMode::Rounded45,
    );

    assert_eq!(
      only_arc(&diagonal_negative),
      ShapeArc::from_start_end_angle(
        Vec2::new(1_292_893, 1_292_893),
        Vec2::new(3_000_000, 2_000_000),
        -quarter,
        0,
      )
    );

    // The one sub-case that reads the radius, `:190`. `mp0` is
    // `( 1000000, 0 )`, so `diag2` is `1e12`, `diagLength` is
    // `sqrt( 2 * diag2 * ( 1 - cos 135 ) )` and
    // `arcRadius = KiROUND( diagLength / ( 2 cos 67.5 ) )` is 2414214.
    // `centreDir` is `mp0` turned by `90 * rotationSign = -90`, which is
    // `( 0, 1000000 )`, so the centre is `( 0, 2414214 )`.
    let from_centre = undefined().build_initial_trace(
      origin,
      Vec2::new(3_000_000, 2_000_000),
      false,
      CornerMode::Rounded45,
    );
    let arc_radius = 2_414_214;

    assert_eq!(
      only_arc(&from_centre),
      ShapeArc::from_center_start_angle(
        Vec2::new(0, arc_radius),
        origin,
        quarter,
        0,
      )
    );
    // And the radius is the one the formula gives, not a coincidence of
    // the endpoints: a centre one nanometre further out is a different
    // arc.
    assert_ne!(
      only_arc(&from_centre),
      ShapeArc::from_center_start_angle(
        Vec2::new(0, arc_radius + 1),
        origin,
        quarter,
        0,
      )
    );
  }

  /// Erratum E18. The `MIN_PRECISION_IU` re-snap of
  /// `direction_45.cpp:197` to `:211` rebuilds the arc with
  /// `+ANGLE_45 * rotationSign` where the construction at `:194` used
  /// `-ANGLE_45 * rotationSign`, and
  /// `ConstructFromStartEndAngle` places the centre from the angle's
  /// sign. **Reproduced**, the behaviour being defined and reachable
  /// rather than undefined or a crash.
  ///
  /// What the sign flip does to the arc: it mirrors it about its own
  /// chord. The raw arc is the fillet, bulging towards the corner it
  /// cuts; the corrected one bulges the other way, away from the corner
  /// and out into the space the mitre would have occupied, tangent to
  /// neither leg. The correction it was making was one nanometre of
  /// endpoint.
  ///
  /// The case: `p0` at the origin, `p1` at
  /// `( w, w + KiROUND( w * sqrt 2 ) - 1 )`, which makes
  /// `tangentLength` exactly `-1`, the arc from the centre land one
  /// nanometre short of `p1` in y, and the y branch at `:200` fire.
  #[test]
  fn rounded_45_re_snap_flips_the_arc_bulge_erratum_e18() {
    let origin = Vec2::new(0, 0);
    let end = Vec2::new(1_000_000, 2_414_213);
    let chain = undefined().build_initial_trace(
      origin,
      end,
      false,
      CornerMode::Rounded45,
    );
    // `:194`, before the re-snap. `mp0` is `( 0, 1414213 )`, turned by
    // `90 * rotationSign = 90` it is `( 1414213, 0 )`, and the radius is
    // 3414212.
    let raw = ShapeArc::from_center_start_angle(
      Vec2::new(3_414_212, 0),
      origin,
      -Degrees::EIGHTH_TURN,
      0,
    );

    assert_eq!(raw.start(), origin);
    assert_eq!(raw.arc_mid(), Vec2::new(259_891, 1_306_562));
    assert_eq!(
      raw.end(),
      Vec2::new(1_000_000, 2_414_212),
      "the centre construction lands one nanometre short in y"
    );
    assert!(
      (raw.end().y - end.y).abs() < Shape::MIN_PRECISION_IU,
      "which is what arms the re-snap at :200"
    );

    // What the trace actually holds: the rebuild, exact at both ends and
    // bulging the other way.
    let fixed = only_arc(&chain);

    assert_eq!(
      fixed,
      ShapeArc::from_start_end_angle(
        origin,
        Vec2::new(raw.end().x, end.y),
        Degrees::EIGHTH_TURN,
        0,
      )
    );
    assert_eq!(fixed.start(), origin);
    assert_eq!(fixed.end(), end);
    assert_eq!(fixed.arc_mid(), Vec2::new(740_108, 1_107_650));

    // The flip, stated twice. The handedness is the exact one
    // (`shape_arc.h:310`), and the mid point crosses the chord.
    assert_ne!(
      raw.is_ccw(),
      fixed.is_ccw(),
      "the corrected arc turns the other way"
    );

    let chord = Seg::new(origin, end);
    let corner = Vec2::new(0, 1_414_213);

    assert_eq!(
      chord.side(raw.arc_mid()),
      chord.side(corner),
      "the raw arc is a fillet: it bulges towards the corner it cuts"
    );
    assert_eq!(
      chord.side(fixed.arc_mid()),
      -chord.side(corner),
      "the corrected one bulges away from it"
    );

    // The trace still starts and ends where it was asked to, and the
    // trailing `Append( aP1 )` at `:218` is suppressed as a duplicate.
    assert_eq!(chain.point(0), origin);
    assert_eq!(chain.last_point(), Some(end));
  }

  /// Erratum E19. `ROUNDED_45`'s own `w == h` block
  /// (`direction_45.cpp:123` to `:128`) is unreachable, because the
  /// single segment shortcut at `:46` returns for `!is90mode && h == w`
  /// and `ROUNDED_45` is not a 90 degree mode. **Not ported.**
  ///
  /// Nothing is lost by leaving it out, which is the other half of what
  /// this test says: the dead block appends `aP0` and `aP1` and breaks,
  /// which is exactly what the shortcut already returned. So the answer
  /// is the same two point chain whichever of the two produced it, and
  /// it holds no arc in either reading.
  #[test]
  fn rounded_45_never_reaches_its_own_equal_extent_block_erratum_e19() {
    for extent in [1, 4, 2_000_000, i32::MAX / 2] {
      for sign_x in [1, -1] {
        for sign_y in [1, -1] {
          for start_diagonal in [false, true] {
            let origin = Vec2::new(0, 0);
            let end = Vec2::new(extent * sign_x, extent * sign_y);
            let chain = undefined().build_initial_trace(
              origin,
              end,
              start_diagonal,
              CornerMode::Rounded45,
            );

            assert_eq!(chain.points(), [origin, end], "to {end:?}");
            assert_eq!(chain.arc_count(), 0, "to {end:?}");
          }
        }
      }
    }
  }

  /// A rounded trace whose arc is flatter than the chain's own
  /// approximation error carries no arc at all.
  ///
  /// `SHAPE_LINE_CHAIN::Append( const SHAPE_ARC& )` tags the result as an
  /// arc only when the polyline has more than two points
  /// (`shape_line_chain.cpp:1620`), and `ConvertToPolyline` answers a
  /// bare chord when the arc's mid point is within half the maximum error
  /// of the chord (`shape_arc.cpp:592`). At the chain's 1000 nanometre
  /// accuracy that is any 45 degree corner of a few micrometres, which
  /// the placer does produce when the user is zoomed in. The trace is
  /// then the mitre's two legs by another route, and everything
  /// downstream sees a line with no arcs in it.
  #[test]
  fn a_rounded_trace_too_small_for_its_own_approximation_carries_no_arc() {
    let origin = Vec2::new(0, 0);
    let small = Vec2::new(3000, 1000);

    for corner_mode in [CornerMode::Rounded45, CornerMode::Rounded90] {
      let chain =
        undefined().build_initial_trace(origin, small, false, corner_mode);

      assert_eq!(chain.arc_count(), 0, "{corner_mode:?}");
      assert_eq!(chain.point(0), origin, "{corner_mode:?}");
      assert_eq!(chain.last_point(), Some(small), "{corner_mode:?}");
      assert_eq!(chain.point_count(), 3, "{corner_mode:?}");
    }

    // The same shape a thousand times larger does carry one.
    let large = Vec2::new(3_000_000, 1_000_000);

    for corner_mode in [CornerMode::Rounded45, CornerMode::Rounded90] {
      assert_eq!(
        undefined()
          .build_initial_trace(origin, large, false, corner_mode)
          .arc_count(),
        1,
        "{corner_mode:?}"
      );
    }
  }

  /// A rounded trace is relative to `aP0`, but not to the nanometre.
  ///
  /// `ConstructFromStartEndAngle` truncates its centre into `i32`
  /// (`shape_arc.cpp:205` through `libs/kimath/include/math/vector2d.h:85`),
  /// and truncation towards zero rounds the other way on the negative
  /// side of each axis, so the same trace built around a centre with a
  /// negative coordinate can have a mid point one nanometre away from the
  /// translation of the one built at the origin. The endpoints are exact
  /// either way, being stored and not computed. KiCad has the same
  /// property; it is why the tables above allow
  /// [`TRANSLATION_SLACK`] on their second pass.
  #[test]
  fn a_rounded_45_trace_is_not_quite_translation_invariant() {
    let delta = Vec2::new(3_000_000, 1_000_000);
    let at = |origin: Vec2| {
      undefined()
        .build_initial_trace(
          origin,
          origin + delta,
          false,
          CornerMode::Rounded45,
        )
        .arc(0)
        .expect("the trace holds an arc")
    };
    let origin = Vec2::new(0, 0);
    let offset = Vec2::new(-1_234_567, 890_123);
    let here = at(origin);
    let there = at(offset);

    assert_eq!(there.start(), here.start() + offset);
    assert_eq!(there.end(), here.end() + offset);
    assert_eq!(there.arc_mid(), here.arc_mid() + offset - Vec2::new(0, 1));
  }

  // ---------------------------------------------------------------
  // `Direction45::from_arc`
  // ---------------------------------------------------------------

  /// `DIRECTION_45( const SHAPE_ARC&, bool )`,
  /// `libs/kimath/include/geometry/direction45.h:116`, classifies the
  /// arc's **chord** and nothing else. A 45 degree arc whose tangents are
  /// axis aligned therefore answers a diagonal octant, and one whose
  /// tangents are diagonal answers an axis aligned octant.
  #[test]
  fn direction_from_an_arc_is_the_direction_of_its_chord() {
    let origin = Vec2::new(0, 0);
    // The fillet of `( 3000000, 1000000 )` straight first leaves its
    // start heading east and arrives at `p1` heading south east, so its
    // chord is the bisector of the two, 22.5 degrees off each. That is
    // not an octant at all, and `construct_` rounds it to the nearer
    // one. The answer here is east by a hair: the chord is
    // `( 2414214, 1000000 )`, whose screen angle is a hundredth of a
    // degree short of the 22.5 degree boundary, so a nanometre of noise
    // in either endpoint could answer south east instead. Slice 7 reads
    // this to pick a posture (`pcbnew/router/pns_line_placer.cpp:200`),
    // and this is the knife edge it stands on.
    let straight = undefined()
      .build_initial_trace(
        origin,
        Vec2::new(3_000_000, 1_000_000),
        false,
        CornerMode::Rounded45,
      )
      .arc(0)
      .expect("the trace holds an arc");

    assert_eq!(
      straight.chord().b - straight.chord().a,
      Vec2::new(2_414_214, 1_000_000)
    );
    assert_eq!(
      Direction45::from_arc(&straight, false),
      dir(Octant::E),
      "neither tangent: the chord rounds to the nearer octant"
    );

    // A quarter turn is the clean case: the chord is 45 degrees off each
    // tangent, so an arc between two axis aligned tangents lands exactly
    // on a diagonal octant. This one starts north of the centre heading
    // east and ends east of it heading south, and its chord runs from up
    // to right, which is south east on screen.
    let quarter = ShapeArc::from_start_end_center(
      Vec2::new(0, -1_000_000),
      Vec2::new(1_000_000, 0),
      origin,
      true,
      0,
    );

    assert_eq!(Direction45::from_arc(&quarter, false), dir(Octant::SE));
    assert_eq!(
      Direction45::from_arc(&quarter, true).octant(),
      Some(Octant::SE),
      "the 90 degree flag only changes how turns step"
    );

    // The chord and not the tangent: the reversed arc answers the
    // opposite octant, where reversing an arc leaves both tangent lines
    // where they were.
    assert_eq!(
      Direction45::from_arc(&quarter.reversed(), false),
      dir(Octant::NW)
    );

    // A whole turn has no chord and no direction, the answer a zero
    // length segment gives.
    let whole = ShapeArc::new(origin, Vec2::new(0, -1_000_000), origin, 0);

    assert_eq!(Direction45::from_arc(&whole, false), undefined());
  }
}
