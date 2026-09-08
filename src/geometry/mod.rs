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
//! - [`seg`]: line segments, with the distance, intersection and
//!   collinearity tolerances the router depends on.
//! - [`box2`]: axis aligned bounding boxes, always normalised, with `i64`
//!   coordinates so a clearance inflation cannot overflow.
//! - [`direction45`]: the octant directions of the 45 degree routing
//!   regime, their angle classification and the initial trace builder.

pub mod box2;
pub mod direction45;
pub mod math;
pub mod seg;
pub mod vec2;

pub use box2::Box2;
pub use direction45::{AngleType, CornerMode, Direction45, Octant};
pub use seg::{NearestPoints, Seg, SegCollision};
pub use vec2::{Vec2, Vec2L};
