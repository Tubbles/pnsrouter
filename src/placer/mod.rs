// SPDX-License-Identifier: GPL-3.0-or-later

//! The interactive placement algorithms.
//!
//! KiCad's `PLACEMENT_ALGO` (`pcbnew/router/pns_placement_algo.h:44`) is
//! the abstract base the router holds one of: the single track placer,
//! the differential pair placer and the three meander placers. Only the
//! single track placer is ported, as [`line_placer::LinePlacer`], and its
//! interface is a plain `impl` rather than a trait. Note 03 section 9.2
//! asks for that: the set of placers is closed by the router mode, so an
//! enum wrapping the five is a better fit than a trait object, and it
//! removes the downcast KiCad's `Finish` and `ContinueFromEnd` perform on
//! `Traces()[0]` (`pcbnew/router/pns_router.cpp:579`, `:624`). The enum
//! belongs to the session facade of milestone 5.
//!
//! `fixed_tail.rs`, the undo stack of note 03 section 3.1, is part 2 of
//! this work item; [`line_placer::FixStage`] is its placeholder.

pub mod line_placer;
