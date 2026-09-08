// SPDX-License-Identifier: GPL-3.0-or-later

//! Pure value geometry, ported from KiCad's `libs/kimath`.
//!
//! The layer knows nothing about nets, layers or items. Coordinates are
//! `i32` nanometres, which covers about plus or minus 2.1 metres, and every
//! product, norm or determinant of two coordinates is computed in `i64` from
//! operands that were widened first. See `DESIGN.md` section 2 and
//! `doc/reference/kicad/01-geometry.md` sections 2 and 14.1.
//!
//! Modules are added as the port progresses. So far:
//!
//! - [`math`]: rounding, rational rescaling and integer square root.
//! - [`vec2`]: the two integer vector types.

pub mod math;
pub mod vec2;

pub use vec2::{Vec2, Vec2L};
