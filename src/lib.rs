// SPDX-License-Identifier: GPL-3.0-or-later

//! Interactive push and shove PCB router.
//!
//! The crate is a design level port of KiCad's PNS router. See `DESIGN.md`
//! in the repository for the architecture and `PLAN.md` for the
//! implementation order.

#![cfg_attr(feature = "fail-on-warnings", deny(warnings))]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

pub mod algo_base;
pub mod arena;
pub mod collide;
pub mod debug;
pub mod geometry;
pub mod index;
pub mod item;
pub mod joint;
pub mod line;
pub mod mouse_trail;
pub mod node;
pub mod optimizer;
pub mod placer;
pub mod rules;
pub mod settings;
pub mod topology;
pub mod walkaround;
