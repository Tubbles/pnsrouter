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
//! - The `SHAPE_ARC` constructor (`direction45.h:116`) and the `ROUNDED_45`
//!   and `ROUNDED_90` corner modes, because milestone 1 has no arcs. See
//!   [`CornerMode`].
//! - `Mask()` (`direction45.h:305`), which the router never calls.

use crate::geometry::math::sign;
use crate::geometry::seg::Seg;
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
/// `libs/kimath/include/geometry/direction45.h:66`. KiCad has four modes,
/// `MITERED_45 = 0`, `ROUNDED_45 = 1`, `MITERED_90 = 2` and
/// `ROUNDED_90 = 3`; the two rounded ones replace the corner with an arc
/// and are left out of this port, because milestone 1 has no arc type.
/// They will slot into the numbering gaps when arcs arrive. KiCad's
/// `ROUTING_SETTINGS` defaults to `MITERED_45`
/// (`pcbnew/router/pns_routing_settings.cpp:53`).
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum CornerMode {
  /// Horizontal, vertical and 45 degree segments, mitered corners.
  Mitered45 = 0,
  /// Horizontal and vertical segments only, mitered corners.
  Mitered90 = 2,
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

  /// The two segment trace between two points that obeys the 45 degree
  /// routing regime.
  ///
  /// Port of `BuildInitialTrace`,
  /// `libs/kimath/src/geometry/direction_45.cpp:24`, restricted to the two
  /// mitered corner modes. It is the single most important routine of the
  /// 45 degree regime: the placer, the line, the optimizer and the posture
  /// solver all build their candidate paths with it.
  ///
  /// The return value is the list of vertices KiCad appends to its
  /// `SHAPE_LINE_CHAIN`, in order, with the duplicate suppression of
  /// `SHAPE_LINE_CHAIN::Append`
  /// (`libs/kimath/include/geometry/shape_line_chain.h:534`), which drops a
  /// point equal to the one before it. So the result holds:
  ///
  /// - one point when `p0` and `p1` are the same point, because the second
  ///   `Append` is suppressed. Consumers that take segment zero of the
  ///   result have to cope, as `pcbnew/router/pns_optimizer.cpp:1155`
  ///   does;
  /// - two points when the pair is axis aligned, or an exact diagonal in
  ///   45 degree mode (`direction_45.cpp:46`);
  /// - three points otherwise, `p0`, the corner and `p1`.
  ///
  /// KiCad ends with `pl.Simplify()` (`direction_45.cpp:310`), which drops
  /// exactly collinear vertices. It cannot change a mitered result: the
  /// early return has already removed every case where a leg has zero
  /// length, and the two remaining legs are a straight and a diagonal in 45
  /// degree mode, or a horizontal and a vertical in 90 degree mode, so they
  /// are never collinear. It is therefore not ported.
  ///
  /// `start_diagonal` is KiCad's `aStartDiagonal`: it asks for the diagonal
  /// leg first, and in 90 degree mode for the shorter leg first
  /// (`direction45.h:230`). It is only consulted when this direction is
  /// undefined; a defined direction imposes its own posture through
  /// [`Direction45::is_diagonal`] (`direction_45.cpp:31`).
  ///
  /// The endpoint re-snap against `SHAPE_ARC::MIN_PRECISION_IU`
  /// (`direction_45.cpp:200` to `:211`) belongs to the `ROUNDED_45` branch
  /// and to nothing else. It exists because that branch builds its arc from
  /// a centre, which loses the endpoint, so the code pulls the endpoint back
  /// onto `p1` when it is within four nanometres of it. The mitered corners
  /// below are exact integers, so there is nothing to re-snap. The re-snap
  /// arrives with the arc port.
  ///
  /// Deviation: the coordinate differences are taken in `i64`, where KiCad
  /// takes them in `int` (`direction_45.cpp:36`) and wraps for a pair
  /// spanning more than the coordinate range. Every vertex the routine
  /// produces lies inside the bounding box of `p0` and `p1`, so the
  /// narrowing back to `i32` is exact.
  pub fn build_initial_trace(
    self,
    p0: Vec2,
    p1: Vec2,
    start_diagonal: bool,
    corner_mode: CornerMode,
  ) -> Vec<Vec2> {
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

    // `is90mode`, direction_45.cpp:41. ROUNDED_90 is not ported.
    let is_90_mode = corner_mode == CornerMode::Mitered90;

    // The shortcut of direction_45.cpp:46, which keeps the single segment
    // cases away from the corner arithmetic.
    if width == 0 || height == 0 || (!is_90_mode && height == width) {
      return append_all(&[p0, p1]);
    }

    // `mp0` and `mp1` of direction_45.cpp:56 to :81. `mp0` is the corner of
    // a straight first trace, `mp1` the corner of a diagonal first one; the
    // 90 degree mode has only one corner and leaves `mp1` at zero.
    let (corner_straight, corner_diagonal) = if is_90_mode {
      let corner = if start_diagonal == (height >= width) {
        Vec2L::new(width * sign_x, 0)
      } else {
        Vec2L::new(0, sign_y * height)
      };

      (corner, Vec2L::new(0, 0))
    } else if width > height {
      (
        Vec2L::new((width - height) * sign_x, 0),
        Vec2L::new(height * sign_x, height * sign_y),
      )
    } else {
      (
        Vec2L::new(0, sign_y * (height - width)),
        Vec2L::new(sign_x * width, sign_y * width),
      )
    };

    let corner = match corner_mode {
      // direction_45.cpp:100.
      CornerMode::Mitered45 => {
        if start_diagonal {
          corner_diagonal
        } else {
          corner_straight
        }
      }
      // direction_45.cpp:234.
      CornerMode::Mitered90 => corner_straight,
    };

    append_all(&[p0, (Vec2L::from(p0) + corner).saturating_to_vec2(), p1])
  }
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

/// Collect points the way `SHAPE_LINE_CHAIN::Append` collects them.
///
/// Port of `Append( const VECTOR2I&, bool )`,
/// `libs/kimath/include/geometry/shape_line_chain.h:534`, with the default
/// `aAllowDuplication`, which skips a point equal to the last one already
/// in the chain. `doc/reference/kicad/01-geometry.md` section 14.1 flags
/// that suppression as behaviour the placer depends on.
fn append_all(points: &[Vec2]) -> Vec<Vec2> {
  let mut chain: Vec<Vec2> = Vec::with_capacity(points.len());

  for point in points {
    if chain.last() != Some(point) {
      chain.push(*point);
    }
  }

  chain
}

#[cfg(test)]
mod tests {
  use super::*;

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
          undefined().build_initial_trace(
            origin,
            Vec2::new(1000, 0),
            start_diagonal,
            corner_mode
          ),
          vec![origin, Vec2::new(1000, 0)]
        );
        assert_eq!(
          undefined().build_initial_trace(
            origin,
            Vec2::new(0, -1000),
            start_diagonal,
            corner_mode
          ),
          vec![origin, Vec2::new(0, -1000)]
        );
        assert_eq!(
          undefined().build_initial_trace(
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
      let diagonal = undefined().build_initial_trace(
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
        undefined().build_initial_trace(
          origin,
          end,
          false,
          CornerMode::Mitered45
        ),
        vec![origin, straight, end],
        "straight first to {end:?}"
      );
      assert_eq!(
        undefined().build_initial_trace(
          origin,
          end,
          true,
          CornerMode::Mitered45
        ),
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
      undefined().build_initial_trace(start, end, false, CornerMode::Mitered45),
      vec![start, Vec2::new(-4300, 7000), end]
    );
    assert_eq!(
      undefined().build_initial_trace(start, end, true, CornerMode::Mitered45),
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
        dir(Octant::N).build_initial_trace(
          origin,
          end,
          start_diagonal,
          CornerMode::Mitered45
        ),
        vec![origin, Vec2::new(700, 0), end],
        "a straight direction always starts straight"
      );
      assert_eq!(
        dir(Octant::NE).build_initial_trace(
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
        undefined().build_initial_trace(
          origin,
          end,
          false,
          CornerMode::Mitered90
        ),
        vec![origin, straight, end],
        "long leg first to {end:?}"
      );
      assert_eq!(
        undefined().build_initial_trace(
          origin,
          end,
          true,
          CornerMode::Mitered90
        ),
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
      undefined().build_initial_trace(start, end, false, CornerMode::Mitered45);

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
      undefined().build_initial_trace(start, end, false, CornerMode::Mitered45),
      vec![start, Vec2::new(i32::MAX - 100, 0), end]
    );
    assert_eq!(
      undefined().build_initial_trace(start, end, true, CornerMode::Mitered45),
      vec![start, Vec2::new(i32::MIN + 100, 100), end]
    );
  }
}
