// SPDX-License-Identifier: GPL-3.0-or-later

//! Scalar arithmetic and angles shared by the geometry layer.
//!
//! Ported from `libs/kimath/include/math/util.h`,
//! `libs/kimath/src/math/util.cpp`, the integer square root in
//! `libs/kimath/src/geometry/seg.cpp`, and, for [`Degrees`] and
//! [`rotate_point`], `libs/kimath/include/geometry/eda_angle.h` and
//! `libs/kimath/src/trigo.cpp`.
//!
//! The angle part arrived with the arcs (`doc/reference/kicad/09-arcs.md`
//! section 11.2): nothing below an arc needs a trigonometric angle, and
//! everything an arc derives from its three points does.

use std::f64::consts::{FRAC_1_SQRT_2, PI, SQRT_2};
use std::ops::{Add, Div, Mul, Neg, Sub};

use crate::geometry::vec2::{Vec2, Vec2L};

/// Round to the nearest integer with halfway cases going away from zero,
/// then saturate into the `i32` range.
///
/// Port of `KiROUND<double, int>`, `libs/kimath/include/math/util.h:98`.
/// KiCad rounds with `std::llround`, which is half away from zero, then
/// clamps with `std::clamp` and reports the clamp through `wxFAIL_MSG`,
/// which trips an assertion in a debug build and is silent otherwise.
/// `f64::round` has the same tie rule and a saturating `as` cast reproduces
/// the clamp.
///
/// A NaN input returns 0, which is what KiCad returns when it is built as
/// C++23 (`util.h:102`); on a C++20 build the NaN reaches `std::llround`
/// with an unspecified result. Here it trips a debug assertion first,
/// mirroring the `wxFAIL_MSG` KiCad emits on the same path.
pub fn kiround(value: f64) -> i32 {
  debug_assert!(!value.is_nan(), "kiround: value is NaN");
  value.round() as i32
}

/// Round to the nearest integer with halfway cases going away from zero,
/// then saturate into the `i64` range.
///
/// Port of `KiROUND<double, int64_t>`,
/// `libs/kimath/include/math/util.h:98`, the widened instantiation.
/// `SEG::SquaredDistance` (`libs/kimath/src/geometry/seg.cpp:738`) and
/// `BOX2::Distance` (`libs/kimath/include/math/box2.h:799`) both round an
/// `ecoord` result through it, and an `i32` return would truncate those.
///
/// The tie rule, the clamp and the NaN behaviour are the same as
/// [`kiround`], see its documentation.
pub fn kiround_i64(value: f64) -> i64 {
  debug_assert!(!value.is_nan(), "kiround_i64: value is NaN");
  value.round() as i64
}

/// Compute `numerator * value / denominator` through a 128 bit intermediate,
/// rounding to nearest with halfway cases away from zero.
///
/// Port of the `int64_t` specialization of `rescale`,
/// `libs/kimath/src/math/util.cpp:76`, specifically the `__int128` branch at
/// `util.cpp:106` that every non MSVC build takes. The generic template
/// (`util.h:135`) is a plain truncating `num * val / den` and is not what
/// the geometry code gets.
///
/// Deviation: KiCad narrows the 128 bit quotient back to `int64_t` by an
/// implementation defined conversion, this returns a saturated value
/// instead. The quotient only leaves the `i64` range when the caller asks
/// for a rescaling that cannot be represented, which the geometry code
/// never does.
///
/// # Panics
///
/// When `denominator` is zero, the same input that divides by zero in the
/// C++ original.
pub fn rescale(numerator: i64, value: i64, denominator: i64) -> i64 {
  let product = i128::from(numerator) * i128::from(value);
  // KiCad halves the denominator in 64 bit before adding it, so keep the
  // truncation of `denominator / 2` where it is.
  let half = i128::from(denominator / 2);
  let biased = if (product < 0) != (denominator < 0) {
    product - half
  } else {
    product + half
  };
  let quotient = biased / i128::from(denominator);

  quotient.clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i64
}

/// Exact integer square root, the largest `r` with `r * r <= value`.
///
/// KiCad rolls its own at `libs/kimath/src/geometry/seg.cpp:57`: seed `r`
/// from `std::sqrt` on the value converted to `double`, walk `r` up while
/// `r * r < value`, then walk it down while `r * r > value`, with `r`
/// capped at `ct_sqrt(INT64_MAX) == 3037000499`. That loop lands on the
/// exact floor for every non negative `int64_t`, and the cap never changes
/// the result because `3037000500 * 3037000500` is already above
/// `INT64_MAX`. So `u64::isqrt` and KiCad's routine agree on the whole
/// range KiCad can pass, `0 ..= i64::MAX`, which the test below checks;
/// above that KiCad has no defined behaviour to agree with, since its
/// argument is signed and a negative input returns the cap as a sentinel.
///
/// KiCad's callers still round differently from each other: `SEG::Distance`
/// (`seg.cpp:698`) truncates through this function while
/// `shape_collisions.cpp` takes `std::sqrt` on a `double` and rounds. That
/// difference is a property of the callers, not of this function.
pub fn isqrt(value: u64) -> u64 {
  value.isqrt()
}

/// The sign of a value as `-1`, `0` or `1`.
///
/// Port of `sign`, `libs/kimath/include/math/util.h:141`, which is
/// `( T( 0 ) < val ) - ( val < T( 0 ) )`. `Default` stands in for `T( 0 )`.
pub fn sign<T: Default + PartialOrd>(value: T) -> i32 {
  let zero = T::default();
  i32::from(zero < value) - i32::from(value < zero)
}

/// The number of radians in one degree.
///
/// Port of `EDA_ANGLE::DEGREES_TO_RADIANS`,
/// `libs/kimath/include/geometry/eda_angle.h:122`. Both conversions go
/// through this one constant and KiCad divides by it to go from radians to
/// degrees rather than multiplying by its reciprocal, which is reproduced
/// in [`Degrees::from_radians`] because the two differ in the last bit.
pub const DEGREES_TO_RADIANS: f64 = PI / 180.0;

/// An angle in degrees.
///
/// Port of `EDA_ANGLE`, `libs/kimath/include/geometry/eda_angle.h:36`,
/// which stores degrees in a `double` and offers the tenths of a degree
/// and radians forms as conversions. Only the parts the arc geometry
/// reaches are ported: the vector constructor, the two normalisations, the
/// sine and cosine with their exact quadrant cases, and the arithmetic.
///
/// The type is deliberately not normalised on construction, as KiCad's is
/// not: `Normalize` is something callers ask for, and several routines
/// depend on an angle staying outside `[0, 360)`.
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Degrees(f64);

impl Degrees {
  /// Zero degrees, KiCad's `ANGLE_0` (`eda_angle.h:422`).
  pub const ZERO: Degrees = Degrees(0.0);

  /// 45 degrees, KiCad's `ANGLE_45` (`eda_angle.h:423`).
  pub const EIGHTH_TURN: Degrees = Degrees(45.0);

  /// 90 degrees, KiCad's `ANGLE_90` (`eda_angle.h:424`).
  pub const QUARTER_TURN: Degrees = Degrees(90.0);

  /// 180 degrees, KiCad's `ANGLE_180` (`eda_angle.h:426`).
  pub const HALF_TURN: Degrees = Degrees(180.0);

  /// 270 degrees, KiCad's `ANGLE_270` (`eda_angle.h:427`).
  pub const THREE_QUARTER_TURN: Degrees = Degrees(270.0);

  /// 360 degrees, KiCad's `ANGLE_360` (`eda_angle.h:428`).
  pub const FULL_TURN: Degrees = Degrees(360.0);

  /// An angle from a value already in degrees.
  ///
  /// Port of `EDA_ANGLE( double aAngleInDegrees )`,
  /// `libs/kimath/include/geometry/eda_angle.h:68`.
  pub const fn new(degrees: f64) -> Self {
    Self(degrees)
  }

  /// An angle from a value in radians.
  ///
  /// Port of `EDA_ANGLE( double, RADIANS_T )`,
  /// `libs/kimath/include/geometry/eda_angle.h:46`, which divides by
  /// [`DEGREES_TO_RADIANS`] rather than multiplying by `180 / pi`.
  pub fn from_radians(radians: f64) -> Self {
    Self(radians / DEGREES_TO_RADIANS)
  }

  /// The angle of a vector measured from the positive x axis, with the y
  /// axis pointing up.
  ///
  /// Port of `EDA_ANGLE( const VECTOR2D& )`,
  /// `libs/kimath/include/geometry/eda_angle.h:72`. The five exact cases
  /// come before the `atan2`, so an axis aligned or exactly diagonal
  /// vector produces a whole number of degrees and the quadrant cases of
  /// [`Degrees::sin`] and [`Degrees::cos`] then fire. The result is not
  /// normalised: the negative x axis is `-180`, not `180`.
  pub fn from_vector(x: f64, y: f64) -> Self {
    if x == 0.0 && y == 0.0 {
      Self(0.0)
    } else if y == 0.0 {
      if x >= 0.0 { Self(0.0) } else { Self(-180.0) }
    } else if x == 0.0 {
      if y >= 0.0 { Self(90.0) } else { Self(-90.0) }
    } else if x == y {
      if x >= 0.0 { Self(45.0) } else { Self(-135.0) }
    } else if x == -y {
      if x >= 0.0 { Self(-45.0) } else { Self(135.0) }
    } else {
      Self::from_radians(y.atan2(x))
    }
  }

  /// The value in degrees.
  ///
  /// Port of `AsDegrees`,
  /// `libs/kimath/include/geometry/eda_angle.h:116`.
  pub const fn as_degrees(self) -> f64 {
    self.0
  }

  /// The value in radians.
  ///
  /// Port of `AsRadians`,
  /// `libs/kimath/include/geometry/eda_angle.h:120`.
  pub fn as_radians(self) -> f64 {
    self.0 * DEGREES_TO_RADIANS
  }

  /// The same angle brought into `[0, 360)`.
  ///
  /// Port of `Normalize`,
  /// `libs/kimath/include/geometry/eda_angle.h:229`, which mutates in
  /// place and returns itself. The repeated addition and subtraction of
  /// 360 is kept rather than a remainder, because the two do not agree in
  /// the last bit and the centre and sweep of an arc are compared exactly
  /// in several places (note 09 section 1.10).
  ///
  /// # Panics
  ///
  /// In a debug build for a value that is not finite, where KiCad's loop
  /// would not terminate either.
  pub fn normalized(self) -> Self {
    debug_assert!(self.0.is_finite(), "Degrees::normalized: not finite");

    let mut value = self.0;

    while value < 0.0 {
      value += 360.0;
    }

    while value >= 360.0 {
      value -= 360.0;
    }

    Self(value)
  }

  /// The same angle brought into `(-180, 180]`.
  ///
  /// Port of `Normalize180`,
  /// `libs/kimath/include/geometry/eda_angle.h:269`. Note the asymmetric
  /// bounds: `-180` is pushed up to `180`, and `180` itself stays.
  ///
  /// # Panics
  ///
  /// In a debug build for a value that is not finite, as
  /// [`Degrees::normalized`] does.
  pub fn normalized_180(self) -> Self {
    debug_assert!(self.0.is_finite(), "Degrees::normalized_180: not finite");

    let mut value = self.0;

    while value <= -180.0 {
      value += 360.0;
    }

    while value > 180.0 {
      value -= 360.0;
    }

    Self(value)
  }

  /// The sine of the angle.
  ///
  /// Port of `Sin`, `libs/kimath/include/geometry/eda_angle.h:178`. The
  /// eight exact multiples of 45 degrees are answered from a table so that
  /// a quarter turn is exactly one and a diagonal is exactly
  /// `sqrt(1/2)`; everything else goes through the library `sin` of the
  /// **unnormalised** value, which is what KiCad passes.
  pub fn sin(self) -> f64 {
    let test = self.normalized().0;

    if test == 0.0 || test == 180.0 {
      0.0
    } else if test == 45.0 || test == 135.0 {
      FRAC_1_SQRT_2
    } else if test == 225.0 || test == 315.0 {
      -FRAC_1_SQRT_2
    } else if test == 90.0 {
      1.0
    } else if test == 270.0 {
      -1.0
    } else {
      self.as_radians().sin()
    }
  }

  /// The cosine of the angle.
  ///
  /// Port of `Cos`, `libs/kimath/include/geometry/eda_angle.h:197`, with
  /// the same exact cases and the same unnormalised fallback as
  /// [`Degrees::sin`].
  pub fn cos(self) -> f64 {
    let test = self.normalized().0;

    if test == 0.0 {
      1.0
    } else if test == 180.0 {
      -1.0
    } else if test == 90.0 || test == 270.0 {
      0.0
    } else if test == 45.0 || test == 315.0 {
      FRAC_1_SQRT_2
    } else if test == 135.0 || test == 225.0 {
      -FRAC_1_SQRT_2
    } else {
      self.as_radians().cos()
    }
  }
}

impl Neg for Degrees {
  type Output = Degrees;

  /// Port of `EDA_ANGLE::Invert`,
  /// `libs/kimath/include/geometry/eda_angle.h:173`.
  fn neg(self) -> Degrees {
    Degrees(-self.0)
  }
}

impl Add for Degrees {
  type Output = Degrees;

  /// Port of `operator+( const EDA_ANGLE&, const EDA_ANGLE& )`,
  /// `libs/kimath/include/geometry/eda_angle.h:340`.
  fn add(self, other: Degrees) -> Degrees {
    Degrees(self.0 + other.0)
  }
}

impl Sub for Degrees {
  type Output = Degrees;

  /// Port of `operator-( const EDA_ANGLE&, const EDA_ANGLE& )',
  /// `libs/kimath/include/geometry/eda_angle.h:334`.
  fn sub(self, other: Degrees) -> Degrees {
    Degrees(self.0 - other.0)
  }
}

impl Mul<f64> for Degrees {
  type Output = Degrees;

  /// Port of `operator*( const EDA_ANGLE&, double )`,
  /// `libs/kimath/include/geometry/eda_angle.h:346`.
  fn mul(self, factor: f64) -> Degrees {
    Degrees(self.0 * factor)
  }
}

impl Div<f64> for Degrees {
  type Output = Degrees;

  /// Port of `operator/( const EDA_ANGLE&, double )`,
  /// `libs/kimath/include/geometry/eda_angle.h:352`.
  fn div(self, divisor: f64) -> Degrees {
    Degrees(self.0 / divisor)
  }
}

/// The Euclidean norm of a floating point vector.
///
/// Port of `VECTOR2<double>::EuclideanNorm`,
/// `libs/kimath/include/math/vector2d.h:279`, the floating point
/// instantiation: the exact diagonal takes `|x| * sqrt(2)` and an axis
/// aligned vector takes the absolute value of its one non zero component,
/// so that both come out without a `hypot` rounding step.
///
/// [`crate::geometry::vec2::Vec2::distance`] runs the same three cases on
/// the integer types; this is the entry point the arc centre needs, where
/// the operands are already `f64` and never were coordinates.
pub fn euclidean_norm_f64(x: f64, y: f64) -> f64 {
  if x.abs() == y.abs() {
    return x.abs() * SQRT_2;
  }

  if x == 0.0 {
    return y.abs();
  }

  if y == 0.0 {
    return x.abs();
  }

  x.hypot(y)
}

/// Rotate a point about a centre, clockwise on screen for a positive
/// angle.
///
/// Port of `RotatePoint( int*, int*, int, int, const EDA_ANGLE& )`,
/// `libs/kimath/src/trigo.cpp:263`, which subtracts the centre, calls the
/// two argument form (`:225`) and adds the centre back. The four cardinal
/// angles are exact swaps of the two coordinates, so a quarter turn never
/// loses a nanometre; every other angle rounds the trigonometric product
/// with [`kiround`].
///
/// Deviation: the offset from the centre is carried in `i64` and the
/// result saturates into `i32`, where KiCad computes in `int` throughout
/// and wraps on a pair of points more than the coordinate range apart.
pub fn rotate_point(point: Vec2, center: Vec2, angle: Degrees) -> Vec2 {
  let rotated = rotate_offset(point.widening_sub(center), angle);

  Vec2L::new(
    rotated.x.saturating_add(i64::from(center.x)),
    rotated.y.saturating_add(i64::from(center.y)),
  )
  .saturating_to_vec2()
}

/// Rotate a floating point position about a centre, clockwise on screen
/// for a positive angle.
///
/// Port of `RotatePoint( double*, double*, double, double, const EDA_ANGLE& )`,
/// `libs/kimath/src/trigo.cpp:277` and the two argument form at `:291`.
/// Pass `(0.0, 0.0)` as the centre for the latter, which is what it does.
/// Nothing is rounded here, so this is the form the constructors that take
/// a centre use before they round once at the end.
pub fn rotate_point_f64(
  point: (f64, f64),
  center: (f64, f64),
  angle: Degrees,
) -> (f64, f64) {
  let (x, y) = rotate_offset_f64(point.0 - center.0, point.1 - center.1, angle);

  (x + center.0, y + center.1)
}

/// The rotation about the origin that [`rotate_point`] is built on,
/// `libs/kimath/src/trigo.cpp:225`.
fn rotate_offset(offset: Vec2L, angle: Degrees) -> Vec2L {
  let angle = angle.normalized();

  if angle == Degrees::ZERO {
    offset
  } else if angle == Degrees::QUARTER_TURN {
    Vec2L::new(offset.y, -offset.x)
  } else if angle == Degrees::HALF_TURN {
    Vec2L::new(-offset.x, -offset.y)
  } else if angle == Degrees::THREE_QUARTER_TURN {
    Vec2L::new(-offset.y, offset.x)
  } else {
    let sinus = angle.sin();
    let cosinus = angle.cos();
    let x = offset.x as f64;
    let y = offset.y as f64;

    Vec2L::new(
      kiround_i64(y * sinus + x * cosinus),
      kiround_i64(y * cosinus - x * sinus),
    )
  }
}

/// The rotation about the origin that [`rotate_point_f64`] is built on,
/// `libs/kimath/src/trigo.cpp:291`.
fn rotate_offset_f64(x: f64, y: f64, angle: Degrees) -> (f64, f64) {
  let angle = angle.normalized();

  if angle == Degrees::ZERO {
    (x, y)
  } else if angle == Degrees::QUARTER_TURN {
    (y, -x)
  } else if angle == Degrees::HALF_TURN {
    (-x, -y)
  } else if angle == Degrees::THREE_QUARTER_TURN {
    (-y, x)
  } else {
    let sinus = angle.sin();
    let cosinus = angle.cos();

    (y * sinus + x * cosinus, y * cosinus - x * sinus)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  /// KiCad's `isqrt<int64_t>` from `libs/kimath/src/geometry/seg.cpp:57`,
  /// transcribed so the port can be compared against it.
  fn kicad_isqrt(x: i64) -> i64 {
    /// `sqrt_max_typed<int64_t>`, `seg.cpp:54`.
    const SQRT_MAX: i64 = 3037000499;

    if x < 0 {
      return SQRT_MAX;
    }

    let mut r = (x as f64).sqrt() as i64;

    while r < SQRT_MAX && r * r < x {
      r += 1;
    }

    while r > SQRT_MAX || r * r > x {
      r -= 1;
    }

    r
  }

  /// Every case from `qa/tests/libs/kimath/math/test_util.cpp:46`, in the
  /// same order, with KiCad's expected values verbatim.
  #[test]
  fn rescale_matches_kicad_test_util() {
    // Order: numerator, value, denominator, result.
    let cases: [(i64, i64, i64, i64); 48] = [
      (10, 10, 1, 100),
      (10, 10, -1, -100),
      (10, -10, 1, -100),
      (10, -10, -1, 100),
      (1, 9, 1, 9),
      (1, 9, -1, -9),
      (1, -9, 1, -9),
      (1, -9, -1, 9),
      (10, 10, 2, 50),
      (10, 10, -2, -50),
      (10, -10, 2, -50),
      (10, -10, -2, 50),
      (1, 9, 2, 5),
      (1, 9, -2, -5),
      (1, -9, 2, -5),
      (1, -9, -2, 5),
      (1, 17, 4, 4),
      (1, 17, -4, -4),
      (1, -17, 4, -4),
      (1, -17, -4, 4),
      (1, 19, 4, 5),
      (1, 19, -4, -5),
      (1, -19, 4, -5),
      (1, -19, -4, 5),
      (1, 0, 4, 0),
      (1, 0, -4, 0),
      (-1, 0, 4, 0),
      (-1, 0, -4, 0),
      // sqrt(2^63) = 3037000499.98..
      (3037000499, 3037000499, 1, 9223372030926249001),
      (3037000499, 3037000499, -1, -9223372030926249001),
      (3037000499, -3037000499, 1, -9223372030926249001),
      (3037000499, -3037000499, -1, 9223372030926249001),
      // sqrt(2^63 * 10) = 9603838834.99..
      (9603838834, 9603838834, 10, 9223372034944647956),
      (9603838834, 9603838834, -10, -9223372034944647956),
      (9603838834, -9603838834, 10, -9223372034944647956),
      (9603838834, -9603838834, -10, 9223372034944647956),
      // INT64_MAX = 9223372036854775807
      (i64::MAX, 10, 10, i64::MAX),
      (i64::MAX, 10, -10, -i64::MAX),
      (i64::MAX, -10, 10, -i64::MAX),
      (i64::MAX, -10, -10, i64::MAX),
      (i64::MAX, 10, i64::MAX, 10),
      (i64::MAX, 10, -i64::MAX, -10),
      (i64::MAX, -10, i64::MAX, -10),
      (i64::MAX, -10, -i64::MAX, 10),
      (i64::MAX, i64::MAX, i64::MAX, i64::MAX),
      (i64::MAX, i64::MAX, -i64::MAX, -i64::MAX),
      (i64::MAX, -i64::MAX, i64::MAX, -i64::MAX),
      (i64::MAX, -i64::MAX, -i64::MAX, i64::MAX),
    ];

    for (numerator, value, denominator, expected) in cases {
      assert_eq!(
        rescale(numerator, value, denominator),
        expected,
        "rescale({numerator}, {value}, {denominator})"
      );
    }
  }

  /// Halfway quotients go away from zero for every sign combination.
  #[test]
  fn rescale_rounds_halfway_away_from_zero() {
    assert_eq!(rescale(1, 1, 2), 1);
    assert_eq!(rescale(-1, 1, 2), -1);
    assert_eq!(rescale(1, -1, 2), -1);
    assert_eq!(rescale(1, 1, -2), -1);
    assert_eq!(rescale(-1, -1, 2), 1);
    assert_eq!(rescale(-1, 1, -2), 1);
    assert_eq!(rescale(1, -1, -2), 1);
    assert_eq!(rescale(-1, -1, -2), -1);
    assert_eq!(rescale(3, 1, 2), 2);
    assert_eq!(rescale(-3, 1, 2), -2);
  }

  /// The 128 bit intermediate keeps a product that overflows `i64`.
  #[test]
  fn rescale_survives_an_i64_product() {
    // 2^62 * 4 / 8 is 2^61, but 2^62 * 4 does not fit in an i64.
    assert_eq!(rescale(1i64 << 62, 4, 8), 1i64 << 61);
  }

  /// A quotient outside `i64` saturates instead of being narrowed.
  #[test]
  fn rescale_saturates_an_out_of_range_quotient() {
    assert_eq!(rescale(i64::MAX, 4, 2), i64::MAX);
    assert_eq!(rescale(i64::MAX, -4, 2), i64::MIN);
  }

  /// Halfway cases round away from zero, like `std::llround`.
  #[test]
  fn kiround_rounds_halfway_away_from_zero() {
    assert_eq!(kiround(0.5), 1);
    assert_eq!(kiround(-0.5), -1);
    assert_eq!(kiround(1.5), 2);
    assert_eq!(kiround(-1.5), -2);
    assert_eq!(kiround(2.5), 3);
    assert_eq!(kiround(-2.5), -3);
    assert_eq!(kiround(0.49), 0);
    assert_eq!(kiround(-0.49), 0);
    assert_eq!(kiround(0.0), 0);
  }

  /// Out of range values clamp to the `i32` limits.
  #[test]
  fn kiround_saturates() {
    assert_eq!(kiround(3.0e9), i32::MAX);
    assert_eq!(kiround(-3.0e9), i32::MIN);
    assert_eq!(kiround(f64::INFINITY), i32::MAX);
    assert_eq!(kiround(f64::NEG_INFINITY), i32::MIN);
    assert_eq!(kiround(f64::from(i32::MAX)), i32::MAX);
    assert_eq!(kiround(f64::from(i32::MIN)), i32::MIN);
    assert_eq!(kiround(2147483647.4), i32::MAX);
    assert_eq!(kiround(-2147483647.6), i32::MIN);
  }

  /// NaN returns 0 once the debug assertion is out of the way.
  #[test]
  #[cfg(not(debug_assertions))]
  fn kiround_of_nan_is_zero() {
    assert_eq!(kiround(f64::NAN), 0);
    assert_eq!(kiround_i64(f64::NAN), 0);
  }

  /// The widened rounding keeps values that would not survive an `i32`,
  /// and it clamps at the `i64` limits.
  #[test]
  fn kiround_i64_rounds_and_saturates() {
    assert_eq!(kiround_i64(0.5), 1);
    assert_eq!(kiround_i64(-0.5), -1);
    assert_eq!(kiround_i64(2.5), 3);
    assert_eq!(kiround_i64(-2.5), -3);
    assert_eq!(kiround_i64(3.0e9), 3_000_000_000);
    assert_eq!(kiround_i64(-3.0e9), -3_000_000_000);
    assert_eq!(kiround_i64(f64::INFINITY), i64::MAX);
    assert_eq!(kiround_i64(f64::NEG_INFINITY), i64::MIN);
    assert_eq!(kiround_i64(1.0e30), i64::MAX);
  }

  /// The first few values, including the non squares.
  #[test]
  fn isqrt_of_small_values() {
    assert_eq!(isqrt(0), 0);
    assert_eq!(isqrt(1), 1);
    assert_eq!(isqrt(2), 1);
    assert_eq!(isqrt(3), 1);
    assert_eq!(isqrt(4), 2);
    assert_eq!(isqrt(8), 2);
    assert_eq!(isqrt(9), 3);
  }

  /// Perfect squares and the value just below each of them.
  #[test]
  fn isqrt_of_perfect_squares() {
    for root in [1u64, 2, 3, 7, 16, 1000, 65536, 3037000499, 4294967295] {
      assert_eq!(isqrt(root * root), root);
      assert_eq!(isqrt(root * root - 1), root - 1);
    }
  }

  /// The top of the range, where KiCad's cap sits.
  #[test]
  fn isqrt_of_extremes() {
    assert_eq!(isqrt(u64::MAX), 4294967295);
    assert_eq!(isqrt(i64::MAX as u64), 3037000499);
    assert_eq!(isqrt(1u64 << 62), 1u64 << 31);
  }

  /// The port and KiCad's loop agree over the whole signed range.
  #[test]
  fn isqrt_agrees_with_the_kicad_algorithm() {
    let mut values: Vec<i64> = vec![
      0,
      1,
      2,
      3,
      4,
      5,
      99,
      100,
      101,
      1_000_000,
      1_000_000_000_000,
      i64::MAX,
      i64::MAX - 1,
      3037000499 * 3037000499,
      3037000499 * 3037000499 + 1,
      3037000499 * 3037000499 - 1,
    ];

    // A deterministic spread over the exponent range.
    for shift in 0..63 {
      values.push(1i64 << shift);
      values.push((1i64 << shift) - 1);
    }

    for value in values {
      assert_eq!(
        isqrt(value as u64),
        kicad_isqrt(value) as u64,
        "isqrt({value})"
      );
    }
  }

  /// `Normalize` lands in `[0, 360)` from either side.
  #[test]
  fn degrees_normalize_into_a_full_turn() {
    assert_eq!(Degrees::new(0.0).normalized(), Degrees::ZERO);
    assert_eq!(Degrees::new(360.0).normalized(), Degrees::ZERO);
    assert_eq!(Degrees::new(-360.0).normalized(), Degrees::ZERO);
    assert_eq!(
      Degrees::new(-90.0).normalized(),
      Degrees::THREE_QUARTER_TURN
    );
    assert_eq!(Degrees::new(450.0).normalized(), Degrees::QUARTER_TURN);
    assert_eq!(Degrees::new(-720.5).normalized(), Degrees::new(359.5));
  }

  /// `Normalize180`'s bounds are asymmetric: `-180` is pushed up to `180`
  /// and `180` itself stays.
  #[test]
  fn degrees_normalize_180_keeps_the_half_turn() {
    assert_eq!(Degrees::new(180.0).normalized_180(), Degrees::HALF_TURN);
    assert_eq!(Degrees::new(-180.0).normalized_180(), Degrees::HALF_TURN);
    assert_eq!(Degrees::new(181.0).normalized_180(), Degrees::new(-179.0));
    assert_eq!(Degrees::new(-179.0).normalized_180(), Degrees::new(-179.0));
    assert_eq!(Degrees::new(540.0).normalized_180(), Degrees::HALF_TURN);
  }

  /// The five exact cases of the vector constructor, which is what keeps
  /// an axis aligned or diagonal arc on whole degrees.
  #[test]
  fn degrees_from_vector_has_exact_cases() {
    assert_eq!(Degrees::from_vector(0.0, 0.0), Degrees::ZERO);
    assert_eq!(Degrees::from_vector(5.0, 0.0), Degrees::ZERO);
    assert_eq!(Degrees::from_vector(-5.0, 0.0), Degrees::new(-180.0));
    assert_eq!(Degrees::from_vector(0.0, 5.0), Degrees::QUARTER_TURN);
    assert_eq!(Degrees::from_vector(0.0, -5.0), Degrees::new(-90.0));
    assert_eq!(Degrees::from_vector(5.0, 5.0), Degrees::EIGHTH_TURN);
    assert_eq!(Degrees::from_vector(-5.0, -5.0), Degrees::new(-135.0));
    assert_eq!(Degrees::from_vector(5.0, -5.0), Degrees::new(-45.0));
    assert_eq!(Degrees::from_vector(-5.0, 5.0), Degrees::new(135.0));
    // Everything else is an `atan2`.
    assert!(
      (Degrees::from_vector(1.0, 2.0).as_degrees() - 63.434_948_822_922).abs()
        < 1e-9
    );
  }

  /// The eight exact multiples of 45 degrees answer from the table, so a
  /// quarter turn is exactly one and a diagonal exactly `sqrt(1/2)`.
  #[test]
  fn degrees_sin_and_cos_have_exact_cases() {
    assert_eq!(Degrees::ZERO.sin(), 0.0);
    assert_eq!(Degrees::HALF_TURN.sin(), 0.0);
    assert_eq!(Degrees::QUARTER_TURN.sin(), 1.0);
    assert_eq!(Degrees::THREE_QUARTER_TURN.sin(), -1.0);
    assert_eq!(Degrees::EIGHTH_TURN.sin(), FRAC_1_SQRT_2);
    assert_eq!(Degrees::new(225.0).sin(), -FRAC_1_SQRT_2);

    assert_eq!(Degrees::ZERO.cos(), 1.0);
    assert_eq!(Degrees::HALF_TURN.cos(), -1.0);
    assert_eq!(Degrees::QUARTER_TURN.cos(), 0.0);
    assert_eq!(Degrees::THREE_QUARTER_TURN.cos(), 0.0);
    assert_eq!(Degrees::EIGHTH_TURN.cos(), FRAC_1_SQRT_2);
    assert_eq!(Degrees::new(135.0).cos(), -FRAC_1_SQRT_2);

    // The special cases are chosen on the normalised value but the
    // fallback uses the unnormalised one, as KiCad's does.
    assert_eq!(Degrees::new(-90.0).sin(), -1.0);
    assert_eq!(Degrees::new(450.0).cos(), 0.0);
  }

  /// Radians go out through the same constant they came in through.
  #[test]
  fn degrees_round_trip_through_radians() {
    assert_eq!(Degrees::from_radians(PI).as_degrees(), 180.0);
    assert_eq!(Degrees::HALF_TURN.as_radians(), PI);
    assert_eq!(DEGREES_TO_RADIANS, PI / 180.0);
  }

  /// The four cardinal rotations are exact swaps, so a quarter turn of a
  /// board coordinate never loses a nanometre.
  #[test]
  fn rotate_point_is_exact_on_the_cardinals() {
    let point = Vec2::new(1_234_567, -7_654_321);
    let center = Vec2::new(1000, 2000);

    assert_eq!(rotate_point(point, center, Degrees::ZERO), point);
    assert_eq!(rotate_point(point, center, Degrees::FULL_TURN), point);
    assert_eq!(
      rotate_point(point, center, Degrees::HALF_TURN),
      Vec2::new(2 * 1000 - 1_234_567, 2 * 2000 + 7_654_321)
    );

    // A quarter turn is clockwise on screen, where y grows downwards.
    assert_eq!(
      rotate_point(Vec2::new(100, 0), Vec2::new(0, 0), Degrees::QUARTER_TURN),
      Vec2::new(0, -100)
    );
    assert_eq!(
      rotate_point(
        Vec2::new(100, 0),
        Vec2::new(0, 0),
        Degrees::THREE_QUARTER_TURN
      ),
      Vec2::new(0, 100)
    );

    // Four quarter turns are the identity.
    let mut walked = point;

    for _ in 0..4 {
      walked = rotate_point(walked, center, Degrees::QUARTER_TURN);
    }

    assert_eq!(walked, point);
  }

  /// A non cardinal rotation rounds once, half away from zero.
  #[test]
  fn rotate_point_rounds_a_diagonal() {
    assert_eq!(
      rotate_point(Vec2::new(100, 0), Vec2::new(0, 0), Degrees::EIGHTH_TURN),
      Vec2::new(71, -71)
    );
    assert_eq!(
      rotate_point_f64((100.0, 0.0), (0.0, 0.0), Degrees::QUARTER_TURN),
      (0.0, -100.0)
    );
  }

  /// The diagonal and axis aligned cases avoid `hypot`, so they come out
  /// on the exact constant.
  #[test]
  fn euclidean_norm_f64_has_exact_cases() {
    assert_eq!(euclidean_norm_f64(3.0, 0.0), 3.0);
    assert_eq!(euclidean_norm_f64(0.0, -3.0), 3.0);
    assert_eq!(euclidean_norm_f64(2.0, -2.0), 2.0 * SQRT_2);
    assert_eq!(euclidean_norm_f64(3.0, 4.0), 5.0);
  }

  /// `sign` over integers and floats.
  #[test]
  fn sign_of_values() {
    assert_eq!(sign(-5i32), -1);
    assert_eq!(sign(0i32), 0);
    assert_eq!(sign(5i32), 1);
    assert_eq!(sign(i64::MIN), -1);
    assert_eq!(sign(i64::MAX), 1);
    assert_eq!(sign(-0.5f64), -1);
    assert_eq!(sign(0.0f64), 0);
    assert_eq!(sign(0.5f64), 1);
  }
}
