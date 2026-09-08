// SPDX-License-Identifier: GPL-3.0-or-later

//! Scalar arithmetic shared by the geometry layer.
//!
//! Ported from `libs/kimath/include/math/util.h`,
//! `libs/kimath/src/math/util.cpp` and the integer square root in
//! `libs/kimath/src/geometry/seg.cpp`.

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
