// SPDX-License-Identifier: GPL-3.0-or-later

//! Two dimensional integer vectors.
//!
//! [`Vec2`] is the port of KiCad's `VECTOR2I`
//! (`libs/kimath/include/math/vector2d.h:66`, instantiated at `:683`) and
//! [`Vec2L`] the port of `VECTOR2L` (`:684`). A `Vec2` holds nanometre
//! coordinates, a `Vec2L` holds the widened differences and intermediate
//! points that `SEG` and `BOX2` work with.
//!
//! Arithmetic follows KiCad exactly: every product, determinant and norm
//! widens its operands before multiplying, while `+` and `-` between two
//! vectors stay in the coordinate type. KiCad lets that addition wrap
//! silently (`vector2d.h:437`, `:466`); here it panics in a debug build and
//! wraps in a release build, which is Rust's default integer behaviour.
//! [`Vec2::widening_sub`] is the way to take a difference that cannot
//! overflow.
//!
//! Deliberate differences from `vector2d.h`:
//!
//! - No `PartialOrd` or `Ord`. KiCad has two contradictory orders on the
//!   same type: `operator<` compares squared magnitudes (`vector2d.h:566`)
//!   while `std::less<VECTOR2I>`, which is what its `std::map` and
//!   `std::set` keys use (`vector2d.h:715`, defined at
//!   `libs/kimath/src/math/vector2.cpp:22`), is lexicographic. A derived
//!   `Ord` would silently pick the second one, so callers have to say which
//!   order they mean.
//! - `Hash` is derived even though KiCad deletes `std::hash<VECTOR2I>`
//!   (`vector2d.h:708`) to keep callers out of hash containers. The
//!   determinism rules in `DESIGN.md` section 8 already forbid iterating
//!   one.
//! - The dot product operator `operator*(VECTOR2, VECTOR2)`
//!   (`vector2d.h:502`) is not ported, [`Vec2::dot`] is the spelling.
//! - Scalar `+` and `-` on a whole vector (`vector2d.h:452`, `:481`) are
//!   not ported, nothing in the ported code needs them.

use crate::geometry::math::{kiround, kiround_i64, rescale, sign};
use std::f64::consts::{FRAC_1_SQRT_2, SQRT_2};
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

/// Clamp an `i64` into the `i32` range.
///
/// Port of the clamping cross type constructor,
/// `libs/kimath/include/math/vector2d.h:85`, whose integral branch
/// (`:100`) clamps through `int64_t`.
fn saturate_i32(value: i64) -> i32 {
  value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// The floating point branch of `EuclideanNorm`,
/// `libs/kimath/include/math/vector2d.h:279`, with the exact diagonal and
/// axis aligned special cases but without the final rounding.
fn euclidean_norm_f64(x: f64, y: f64) -> f64 {
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

/// A point or displacement in nanometres.
///
/// Port of `VECTOR2I`, `libs/kimath/include/math/vector2d.h:66` and `:683`.
/// The fields are public in KiCad and stay public here.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct Vec2 {
  /// Horizontal coordinate, nanometres.
  pub x: i32,
  /// Vertical coordinate, nanometres, growing downwards on screen.
  pub y: i32,
}

impl Vec2 {
  /// Build a vector from its components.
  ///
  /// Port of `VECTOR2( T aX, T aY )`,
  /// `libs/kimath/include/math/vector2d.h:271`. The zero vector is
  /// `Vec2::default()`, KiCad's `VECTOR2()` at `:265`.
  pub const fn new(x: i32, y: i32) -> Self {
    Self { x, y }
  }

  /// The squared length, `x * x + y * y`.
  ///
  /// Port of `SquaredEuclideanNorm`,
  /// `libs/kimath/include/math/vector2d.h:303`. Both operands widen before
  /// the multiplication, so the result is exact for every `Vec2` except the
  /// single corner `(i32::MIN, i32::MIN)`, whose true value is
  /// `i64::MAX + 1`. KiCad wraps there, this panics in a debug build.
  pub fn squared_euclidean_norm(self) -> i64 {
    let x = i64::from(self.x);
    let y = i64::from(self.y);

    x * x + y * y
  }

  /// The length, rounded to the nearest nanometre.
  ///
  /// Port of `EuclideanNorm`,
  /// `libs/kimath/include/math/vector2d.h:279`. The exact diagonal case
  /// takes `|x| * sqrt(2)` and an axis aligned vector takes the other
  /// absolute value, both without touching `hypot`, because KiCad boards
  /// are full of exact diagonals and `hypot` would round them
  /// inconsistently.
  ///
  /// Deviation: KiCad calls `std::abs` on the `int` coordinate, which is
  /// undefined for `INT_MIN`. This widens first and saturates the two
  /// absolute value paths at `i32::MAX`.
  pub fn euclidean_norm(self) -> i32 {
    let abs_x = i64::from(self.x).abs();
    let abs_y = i64::from(self.y).abs();

    if abs_x == abs_y {
      return kiround(abs_x as f64 * SQRT_2);
    }

    if self.x == 0 {
      return saturate_i32(abs_y);
    }

    if self.y == 0 {
      return saturate_i32(abs_x);
    }

    kiround(f64::from(self.x).hypot(f64::from(self.y)))
  }

  /// The vector rotated by 90 degrees, `(-y, x)`.
  ///
  /// Port of `Perpendicular`,
  /// `libs/kimath/include/math/vector2d.h:310`. That is counter clockwise
  /// in mathematical coordinates and clockwise on screen, where y grows
  /// downwards.
  ///
  /// # Panics
  ///
  /// In a debug build when `y` is `i32::MIN`, which has no negation in
  /// `i32`.
  pub fn perpendicular(self) -> Self {
    Self::new(-self.y, self.x)
  }

  /// A vector with the same direction and the given length.
  ///
  /// Port of `Resize`, `libs/kimath/include/math/vector2d.h:381`. The zero
  /// vector resizes to itself, an exact diagonal takes
  /// `|new_length| * sqrt(1/2)` per component, and everything else takes
  /// `sqrt(rescale(new_length^2, x^2, l^2))` per component with the sign of
  /// the original component. A negative `new_length` reverses the
  /// direction, because KiCad multiplies the result by
  /// [`sign`] of the requested length.
  ///
  /// This is the workhorse of every hull builder, so its rounding has to
  /// match KiCad to the nanometre.
  ///
  /// Deviation: the absolute values are taken after widening to `i64` and
  /// `f64`, where KiCad calls `std::abs` on the `int`, which is undefined
  /// for `INT_MIN`.
  pub fn resize(self, new_length: i32) -> Self {
    if self.x == 0 && self.y == 0 {
      return Self::new(0, 0);
    }

    let new_x;
    let new_y;

    if i64::from(self.x).abs() == i64::from(self.y).abs() {
      new_x = f64::from(new_length).abs() * FRAC_1_SQRT_2;
      new_y = new_x;
    } else {
      let x = i64::from(self.x);
      let y = i64::from(self.y);
      let length = i64::from(new_length);
      let x_squared = x * x;
      let y_squared = y * y;
      let length_squared = x_squared + y_squared;
      let new_length_squared = length * length;

      new_x =
        (rescale(new_length_squared, x_squared, length_squared) as f64).sqrt();
      new_y =
        (rescale(new_length_squared, y_squared, length_squared) as f64).sqrt();
    }

    let resized = Self::new(
      if self.x < 0 {
        -kiround(new_x)
      } else {
        kiround(new_x)
      },
      if self.y < 0 {
        -kiround(new_y)
      } else {
        kiround(new_y)
      },
    );

    resized * sign(new_length)
  }

  /// The z component of the cross product, `x * other.y - y * other.x`.
  ///
  /// Port of `Cross`, `libs/kimath/include/math/vector2d.h:534`. Every
  /// operand widens before the multiplication. The result is the
  /// determinant that the collinearity and side tests are built on.
  pub fn cross(self, other: Self) -> i64 {
    i64::from(self.x) * i64::from(other.y)
      - i64::from(self.y) * i64::from(other.x)
  }

  /// The dot product, `x * other.x + y * other.y`.
  ///
  /// Port of `Dot`, `libs/kimath/include/math/vector2d.h:542`. Every
  /// operand widens before the multiplication.
  pub fn dot(self, other: Self) -> i64 {
    i64::from(self.x) * i64::from(other.x)
      + i64::from(self.y) * i64::from(other.y)
  }

  /// The distance to another point, as a `f64` because it is rarely a whole
  /// nanometre.
  ///
  /// Port of `Distance`, `libs/kimath/include/math/vector2d.h:549`, which
  /// takes the widened vector type, computes the difference in `i64` and
  /// then runs the floating point `EuclideanNorm`, special cases included.
  pub fn distance(self, other: Vec2L) -> f64 {
    let dx = other.x - i64::from(self.x);
    let dy = other.y - i64::from(self.y);

    euclidean_norm_f64(dx as f64, dy as f64)
  }

  /// The squared distance to another point.
  ///
  /// Port of `SquaredDistance`,
  /// `libs/kimath/include/math/vector2d.h:556`. The differences are taken
  /// in `i64` before they are squared, which is exact as long as the
  /// squared distance itself fits in `i64`, so up to a separation of about
  /// 3.03e9 nanometres, three metres. Two points at opposite ends of the
  /// full coordinate envelope are further apart than that: KiCad wraps
  /// there, this panics in a debug build.
  pub fn squared_distance(self, other: Self) -> i64 {
    let dx = i64::from(self.x) - i64::from(other.x);
    let dy = i64::from(self.y) - i64::from(other.y);

    dx * dx + dy * dy
  }

  /// The difference of two points, widened so that it cannot overflow.
  ///
  /// KiCad's `operator-` stays in `int` and wraps
  /// (`libs/kimath/include/math/vector2d.h:466`), which is a trap for
  /// coordinates near the ends of the range. `DESIGN.md` section 2 asks for
  /// differences of coordinates in `i64` before any product, and this is
  /// how to take one.
  pub fn widening_sub(self, other: Self) -> Vec2L {
    Vec2L::new(
      i64::from(self.x) - i64::from(other.x),
      i64::from(self.y) - i64::from(other.y),
    )
  }
}

/// Port of `operator+`, `libs/kimath/include/math/vector2d.h:437`.
impl Add for Vec2 {
  type Output = Self;

  fn add(self, other: Self) -> Self {
    Self::new(self.x + other.x, self.y + other.y)
  }
}

/// Port of `operator-`, `libs/kimath/include/math/vector2d.h:466`.
impl Sub for Vec2 {
  type Output = Self;

  fn sub(self, other: Self) -> Self {
    Self::new(self.x - other.x, self.y - other.y)
  }
}

/// Port of `operator-()`, `libs/kimath/include/math/vector2d.h:495`.
impl Neg for Vec2 {
  type Output = Self;

  fn neg(self) -> Self {
    Self::new(-self.x, -self.y)
  }
}

/// Port of `operator*( VECTOR2, scalar )`,
/// `libs/kimath/include/math/vector2d.h:510`, which multiplies in the
/// coordinate type.
impl Mul<i32> for Vec2 {
  type Output = Self;

  fn mul(self, scalar: i32) -> Self {
    Self::new(self.x * scalar, self.y * scalar)
  }
}

/// Port of `operator/`, `libs/kimath/include/math/vector2d.h:524`.
///
/// KiCad only divides a vector by a `double` and rounds each component with
/// `KiROUND`, so a midpoint such as `( a + b ) / 2` rounds half away from
/// zero rather than truncating. The divisor is a `f64` here to keep that
/// visible at every call site.
impl Div<f64> for Vec2 {
  type Output = Self;

  fn div(self, factor: f64) -> Self {
    Self::new(
      kiround(f64::from(self.x) / factor),
      kiround(f64::from(self.y) / factor),
    )
  }
}

/// Port of `operator+=`, `libs/kimath/include/math/vector2d.h:327`.
impl AddAssign for Vec2 {
  fn add_assign(&mut self, other: Self) {
    self.x += other.x;
    self.y += other.y;
  }
}

/// Port of `operator-=`, `libs/kimath/include/math/vector2d.h:363`.
impl SubAssign for Vec2 {
  fn sub_assign(&mut self, other: Self) {
    self.x -= other.x;
    self.y -= other.y;
  }
}

/// A point or displacement in the widened coordinate type.
///
/// Port of `VECTOR2L`, `libs/kimath/include/math/vector2d.h:66` and `:684`.
/// KiCad uses it for the results of `SEG::NearestPoint` and friends, which
/// are differences and rescalings of `VECTOR2I` coordinates.
///
/// Note that KiCad's `VECTOR2_TRAITS` widening only kicks in for `int`
/// (`vector2d.h:47`), so the `i64` products below are computed in `i64` just
/// like KiCad computes them, and overflow for operands beyond about
/// `3.03e9`. The router only ever feeds it differences of `i32`
/// coordinates.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub struct Vec2L {
  /// Horizontal coordinate, nanometres.
  pub x: i64,
  /// Vertical coordinate, nanometres, growing downwards on screen.
  pub y: i64,
}

impl Vec2L {
  /// Build a vector from its components.
  ///
  /// Port of `VECTOR2( T aX, T aY )`,
  /// `libs/kimath/include/math/vector2d.h:271`.
  pub const fn new(x: i64, y: i64) -> Self {
    Self { x, y }
  }

  /// The squared length, `x * x + y * y`.
  ///
  /// Port of `SquaredEuclideanNorm`,
  /// `libs/kimath/include/math/vector2d.h:303`.
  pub fn squared_euclidean_norm(self) -> i64 {
    self.x * self.x + self.y * self.y
  }

  /// The length, rounded to the nearest nanometre.
  ///
  /// Port of `EuclideanNorm`,
  /// `libs/kimath/include/math/vector2d.h:279` instantiated over
  /// `int64_t`, with the same three special cases as
  /// [`Vec2::euclidean_norm`]: an exact diagonal takes `|x| * sqrt(2)`, an
  /// axis aligned vector takes the absolute value of its one non zero
  /// component without touching `hypot`, and everything else goes through
  /// `hypot`. The rounding is half away from zero.
  ///
  /// `CIRCLE::IntersectLine` (`libs/kimath/src/geometry/circle.cpp:349`)
  /// is the collision routine that needs the `i64` width: the vector it
  /// measures is the difference of two `i32` points, which KiCad widens
  /// before measuring.
  ///
  /// Deviation: KiCad calls `std::abs` on the coordinate, which is
  /// undefined for `INT64_MIN`. The absolute values here saturate at
  /// `i64::MAX`.
  pub fn euclidean_norm(self) -> i64 {
    let abs_x = self.x.saturating_abs();
    let abs_y = self.y.saturating_abs();

    if abs_x == abs_y {
      return kiround_i64(abs_x as f64 * SQRT_2);
    }

    if self.x == 0 {
      return abs_y;
    }

    if self.y == 0 {
      return abs_x;
    }

    kiround_i64((self.x as f64).hypot(self.y as f64))
  }

  /// The vector rotated by 90 degrees, `(-y, x)`.
  ///
  /// Port of `Perpendicular`,
  /// `libs/kimath/include/math/vector2d.h:310`.
  pub fn perpendicular(self) -> Self {
    Self::new(-self.y, self.x)
  }

  /// The z component of the cross product.
  ///
  /// Port of `Cross`, `libs/kimath/include/math/vector2d.h:534`.
  pub fn cross(self, other: Self) -> i64 {
    self.x * other.y - self.y * other.x
  }

  /// The dot product.
  ///
  /// Port of `Dot`, `libs/kimath/include/math/vector2d.h:542`.
  pub fn dot(self, other: Self) -> i64 {
    self.x * other.x + self.y * other.y
  }

  /// The squared distance to another point.
  ///
  /// Port of `SquaredDistance`,
  /// `libs/kimath/include/math/vector2d.h:556`.
  pub fn squared_distance(self, other: Self) -> i64 {
    let dx = self.x - other.x;
    let dy = self.y - other.y;

    dx * dx + dy * dy
  }

  /// Narrow to nanometre coordinates, clamping each component.
  ///
  /// Port of the cross type constructor,
  /// `libs/kimath/include/math/vector2d.h:85`, whose integral branch
  /// (`:100`) clamps through `int64_t` instead of truncating. Every KiCad
  /// call site that builds a `VECTOR2I` from a `VECTOR2L` gets this
  /// clamping.
  pub fn saturating_to_vec2(self) -> Vec2 {
    Vec2::new(saturate_i32(self.x), saturate_i32(self.y))
  }
}

/// Port of `operator+`, `libs/kimath/include/math/vector2d.h:437`.
impl Add for Vec2L {
  type Output = Self;

  fn add(self, other: Self) -> Self {
    Self::new(self.x + other.x, self.y + other.y)
  }
}

/// Port of `operator-`, `libs/kimath/include/math/vector2d.h:466`.
impl Sub for Vec2L {
  type Output = Self;

  fn sub(self, other: Self) -> Self {
    Self::new(self.x - other.x, self.y - other.y)
  }
}

/// Port of `operator-()`, `libs/kimath/include/math/vector2d.h:495`.
impl Neg for Vec2L {
  type Output = Self;

  fn neg(self) -> Self {
    Self::new(-self.x, -self.y)
  }
}

/// Port of `operator*( VECTOR2, scalar )`,
/// `libs/kimath/include/math/vector2d.h:510`.
impl Mul<i64> for Vec2L {
  type Output = Self;

  fn mul(self, scalar: i64) -> Self {
    Self::new(self.x * scalar, self.y * scalar)
  }
}

/// Port of `operator+=`, `libs/kimath/include/math/vector2d.h:327`.
impl AddAssign for Vec2L {
  fn add_assign(&mut self, other: Self) {
    self.x += other.x;
    self.y += other.y;
  }
}

/// Port of `operator-=`, `libs/kimath/include/math/vector2d.h:363`.
impl SubAssign for Vec2L {
  fn sub_assign(&mut self, other: Self) {
    self.x -= other.x;
    self.y -= other.y;
  }
}

/// Widening a `Vec2` is exact, so it is the one conversion that needs no
/// clamping. Port of the cross type constructor,
/// `libs/kimath/include/math/vector2d.h:85`.
impl From<Vec2> for Vec2L {
  fn from(value: Vec2) -> Self {
    Self::new(i64::from(value.x), i64::from(value.y))
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  /// The runtime half of `Constexpr`,
  /// `qa/tests/libs/kimath/math/test_vector2.cpp:34`. The ordering
  /// assertions of that case are left out, see the module documentation on
  /// KiCad's two contradictory orders.
  #[test]
  fn algebra_from_the_kicad_constexpr_case() {
    let vi_1_2 = Vec2::new(1, 2);

    assert_eq!(vi_1_2, Vec2::new(1, 2));
    assert_eq!(vi_1_2.x, 1);
    assert_eq!(vi_1_2.y, 2);

    let vi_3_5 = vi_1_2 + Vec2::new(3, 4) - Vec2::new(1, 1);

    assert_eq!(vi_3_5, Vec2::new(3, 5));
    assert_eq!(vi_3_5.squared_euclidean_norm(), 9 + 25);
    assert_ne!(vi_1_2, vi_3_5);
    assert_eq!(vi_1_2.squared_distance(vi_3_5), 4 + 9);
  }

  /// `test_cross_product`,
  /// `qa/tests/libs/kimath/math/test_vector2.cpp:73`.
  #[test]
  fn cross_product() {
    let v1 = Vec2::new(0, 1);
    let v2 = Vec2::new(1, 0);

    assert_eq!(v2.cross(v1), 1);
  }

  /// `test_dot_product`, `qa/tests/libs/kimath/math/test_vector2.cpp:81`.
  #[test]
  fn dot_product() {
    let v1 = Vec2::new(0, 1);
    let v2 = Vec2::new(1, 0);

    assert_eq!(v2.dot(v1), 0);
  }

  /// `test_resize`, `qa/tests/libs/kimath/math/test_vector2.cpp:89`, with
  /// KiCad's expected values verbatim.
  #[test]
  fn resize_matches_kicad_test_vector2() {
    assert_eq!(Vec2::new(4, 3).resize(8), Vec2::new(6, 5));
    assert_eq!(Vec2::new(5, -1).resize(10), Vec2::new(10, -2));
    assert_eq!(Vec2::new(-2, 1).resize(4), Vec2::new(-4, 2));
    assert_eq!(Vec2::new(1, 1).resize(1), Vec2::new(1, 1));
    assert_eq!(Vec2::new(-70, -70).resize(100), Vec2::new(-71, -71));
  }

  /// A negative length keeps the axis and reverses the direction,
  /// `libs/kimath/include/math/vector2d.h:405`.
  #[test]
  fn resize_with_a_negative_length_reverses_the_direction() {
    assert_eq!(Vec2::new(4, 3).resize(-8), Vec2::new(-6, -5));
    assert_eq!(Vec2::new(-2, 1).resize(-4), Vec2::new(4, -2));
    assert_eq!(Vec2::new(1, 1).resize(-1), Vec2::new(-1, -1));
    assert_eq!(Vec2::new(-70, -70).resize(-100), Vec2::new(71, 71));
  }

  /// A zero vector has no direction to keep.
  #[test]
  fn resize_of_the_zero_vector() {
    assert_eq!(Vec2::new(0, 0).resize(10), Vec2::new(0, 0));
    assert_eq!(Vec2::new(0, 0).resize(-10), Vec2::new(0, 0));
  }

  /// A zero length collapses the vector, since `sign( 0 )` is 0.
  #[test]
  fn resize_to_zero_length() {
    assert_eq!(Vec2::new(4, 3).resize(0), Vec2::new(0, 0));
    assert_eq!(Vec2::new(7, 7).resize(0), Vec2::new(0, 0));
  }

  /// The three branches of `EuclideanNorm`.
  #[test]
  fn euclidean_norm_special_cases() {
    // The general case, through hypot.
    assert_eq!(Vec2::new(3, 4).euclidean_norm(), 5);
    // Exact diagonals, through |x| * sqrt(2).
    assert_eq!(Vec2::new(1, 1).euclidean_norm(), 1);
    assert_eq!(Vec2::new(-5, 5).euclidean_norm(), 7);
    // Axis aligned, the other absolute value.
    assert_eq!(Vec2::new(7, 0).euclidean_norm(), 7);
    assert_eq!(Vec2::new(0, -7).euclidean_norm(), 7);
    // The zero vector goes through the diagonal branch.
    assert_eq!(Vec2::new(0, 0).euclidean_norm(), 0);
  }

  /// The norm of the extreme coordinates stays inside `i32`.
  #[test]
  fn euclidean_norm_saturates() {
    assert_eq!(Vec2::new(0, i32::MIN).euclidean_norm(), i32::MAX);
    assert_eq!(Vec2::new(i32::MIN, 0).euclidean_norm(), i32::MAX);
    assert_eq!(Vec2::new(i32::MAX, i32::MAX).euclidean_norm(), i32::MAX);
  }

  /// The squared norm and the squared distance are exact at the ends of the
  /// coordinate range.
  #[test]
  fn squared_quantities_are_exact() {
    let corner = Vec2::new(i32::MAX, i32::MAX);
    let expected = 2 * i64::from(i32::MAX) * i64::from(i32::MAX);

    assert_eq!(corner.squared_euclidean_norm(), expected);

    // A separation of 2^31 nanometres, a bit over two metres, still fits.
    // A separation of the whole coordinate envelope would not, see the
    // documentation of the method.
    assert_eq!(
      Vec2::new(-(1 << 30), 0).squared_distance(Vec2::new(1 << 30, 0)),
      (1i64 << 31) * (1i64 << 31)
    );
  }

  /// A 90 degree rotation, counter clockwise in mathematical coordinates.
  #[test]
  fn perpendicular_rotates_by_90_degrees() {
    assert_eq!(Vec2::new(1, 0).perpendicular(), Vec2::new(0, 1));
    assert_eq!(Vec2::new(0, 1).perpendicular(), Vec2::new(-1, 0));
    assert_eq!(Vec2::new(3, -4).perpendicular(), Vec2::new(4, 3));
    assert_eq!(Vec2L::new(3, -4).perpendicular(), Vec2L::new(4, 3));
  }

  /// Negation, scalar multiplication and the compound assignments.
  #[test]
  fn scalar_operators() {
    assert_eq!(-Vec2::new(3, -4), Vec2::new(-3, 4));
    assert_eq!(Vec2::new(3, -4) * 3, Vec2::new(9, -12));
    assert_eq!(Vec2::new(3, -4) * -1, Vec2::new(-3, 4));

    let mut accumulator = Vec2::new(1, 2);
    accumulator += Vec2::new(3, 4);
    assert_eq!(accumulator, Vec2::new(4, 6));
    accumulator -= Vec2::new(10, 10);
    assert_eq!(accumulator, Vec2::new(-6, -4));
  }

  /// Division rounds each component half away from zero, it does not
  /// truncate the way an integer division would.
  #[test]
  fn division_rounds_rather_than_truncates() {
    assert_eq!(Vec2::new(3, -3) / 2.0, Vec2::new(2, -2));
    assert_eq!(Vec2::new(5, 7) / 2.0, Vec2::new(3, 4));
    assert_eq!(Vec2::new(10, -10) / 4.0, Vec2::new(3, -3));
  }

  /// The floating point distance, with the same special cases as the
  /// integer norm.
  #[test]
  fn distance_between_points() {
    let origin = Vec2::new(0, 0);

    assert!((origin.distance(Vec2L::new(3, 4)) - 5.0).abs() < 1e-12);
    assert!(
      (Vec2::new(1, 1).distance(Vec2L::new(2, 2)) - SQRT_2).abs() < 1e-12
    );
    assert!((origin.distance(Vec2L::new(0, -7)) - 7.0).abs() < 1e-12);

    // The difference is taken in i64, so the extremes do not wrap.
    let span =
      Vec2::new(i32::MIN, 0).distance(Vec2L::from(Vec2::new(i32::MAX, 0)));
    assert!((span - 4_294_967_295.0).abs() < 1e-3);
  }

  /// Widening is exact, narrowing clamps, which is what KiCad's cross type
  /// constructor does, `test_vector2.cpp:212` covers the same clamp.
  #[test]
  fn conversions_between_the_two_widths() {
    assert_eq!(Vec2L::from(Vec2::new(-4, 3)), Vec2L::new(-4, 3));
    assert_eq!(
      Vec2L::new(i64::MAX, i64::MIN).saturating_to_vec2(),
      Vec2::new(i32::MAX, i32::MIN)
    );
    assert_eq!(
      Vec2L::new(2_147_483_648, -2_147_483_649).saturating_to_vec2(),
      Vec2::new(i32::MAX, i32::MIN)
    );
  }

  /// The widened difference does not wrap where KiCad's `operator-` would.
  #[test]
  fn widening_sub_does_not_wrap() {
    assert_eq!(
      Vec2::new(i32::MAX, i32::MIN).widening_sub(Vec2::new(i32::MIN, i32::MAX)),
      Vec2L::new(4_294_967_295, -4_294_967_295)
    );
  }

  /// The `Vec2L` arithmetic the segment code will lean on.
  #[test]
  fn vec2l_algebra() {
    let a = Vec2L::new(3, 5);
    let b = Vec2L::new(-2, 7);

    assert_eq!(a + b, Vec2L::new(1, 12));
    assert_eq!(a - b, Vec2L::new(5, -2));
    assert_eq!(-a, Vec2L::new(-3, -5));
    assert_eq!(a * 4, Vec2L::new(12, 20));
    assert_eq!(a.dot(b), -6 + 35);
    assert_eq!(a.cross(b), 21 + 10);
    assert_eq!(a.squared_euclidean_norm(), 9 + 25);
    assert_eq!(a.squared_distance(b), 25 + 4);

    let mut accumulator = a;
    accumulator += b;
    accumulator -= Vec2L::new(1, 2);
    assert_eq!(accumulator, Vec2L::new(0, 10));
  }

  /// The widened `EuclideanNorm` keeps the three special cases of the
  /// narrow one and rounds half away from zero,
  /// `libs/kimath/include/math/vector2d.h:279`. `CIRCLE::IntersectLine`
  /// is the collision routine that needs it
  /// (`libs/kimath/src/geometry/circle.cpp:349`).
  #[test]
  fn vec2l_euclidean_norm() {
    assert_eq!(Vec2L::new(0, 0).euclidean_norm(), 0);
    assert_eq!(Vec2L::new(0, -17).euclidean_norm(), 17);
    assert_eq!(Vec2L::new(-17, 0).euclidean_norm(), 17);
    assert_eq!(Vec2L::new(3, 4).euclidean_norm(), 5);

    // The exact diagonal takes |x| * sqrt(2), rounded.
    assert_eq!(Vec2L::new(100, -100).euclidean_norm(), 141);
    assert_eq!(Vec2L::new(-1, 1).euclidean_norm(), 1);

    // Half away from zero: 4 * sqrt(2) is 5.657.
    assert_eq!(Vec2L::new(4, 4).euclidean_norm(), 6);

    // The width is what makes it worth having: this overflows an i32.
    assert_eq!(
      Vec2L::new(3_000_000_000, 4_000_000_000).euclidean_norm(),
      5_000_000_000
    );

    // The two Vec2 widths agree wherever both can represent the answer.
    for (x, y) in [(0, 0), (7, 0), (0, -7), (5, 5), (-5, 5), (3, 4), (11, 37)] {
      assert_eq!(
        Vec2L::new(i64::from(x), i64::from(y)).euclidean_norm(),
        i64::from(Vec2::new(x, y).euclidean_norm()),
        "at ({x}, {y})"
      );
    }
  }
}
