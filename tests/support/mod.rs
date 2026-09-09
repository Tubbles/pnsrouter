// SPDX-License-Identifier: GPL-3.0-or-later

//! Readers for KiCad's PNS regression fixtures.
//!
//! The fixtures live under `tests/fixtures/kicad/pns_regressions` and are
//! copied verbatim from KiCad, see the README next to them. Three file
//! kinds matter:
//!
//! - `boards/*.kicad_pcb`, the board pool the cases route on, read by
//!   [`kicad_pcb`] on top of the s-expression reader in [`sexpr`].
//! - `<case>/*.log`, the recorded input event stream plus the golden
//!   commit result, read by [`pns_log`] on top of the JSON reader in
//!   [`json`].
//! - `<case>/*.settings`, the router settings for the case, also JSON and
//!   also read by [`pns_log`].
//! - `<case>/*.kicad_dru`, the custom design rules one case ships, read
//!   by [`kicad_dru`] on the same s-expression reader.
//!
//! Everything here stops at a neutral intermediate representation: rows of
//! numbers, strings and enums with no dependency on the router crate.
//! Turning that into a `WorldSnapshot` is a separate step, so that the
//! file format knowledge and the engine model can change independently.
//!
//! The whole module is hand written on purpose. The crate takes no
//! dependencies, dev dependencies included, and both dialects here are
//! small enough that a reader is cheaper than a parser generator.

// The readers are a fixture library: they expose the whole of both file
// formats so that the WorldSnapshot conversion can be written against
// them, while `tests/kicad_fixtures.rs` only reads the parts it asserts
// on. Fields that no test touches yet are still part of the contract.
#![allow(dead_code)]

pub mod json;
pub mod kicad_dru;
pub mod kicad_pcb;
pub mod kicad_replay;
pub mod kicad_snapshot;
pub mod pns_log;
pub mod sexpr;
