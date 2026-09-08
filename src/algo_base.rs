// SPDX-License-Identifier: GPL-3.0-or-later

//! What every routing algorithm is handed instead of a global router.
//!
//! Port of `PNS::ALGO_BASE` (`pcbnew/router/pns_algo_base.h:42`), the
//! base class the walkaround, the shove, the optimizer, the line placer
//! and both draggers derive from. It exists in KiCad to give all of them
//! one route to three ambient things: the routing settings, the debug
//! decorator and the logger, each reached through a `ROUTER*` that is in
//! practice the process wide `ROUTER::GetInstance()`.
//!
//! # Why this is a struct and not a base class
//!
//! Inheriting three fields is not a reason for a class hierarchy, and the
//! singleton behind them is exactly what `DESIGN.md` section 8 rules out.
//! Note 03 section 9.4 spells the replacement out: the four uses of
//! `ROUTER::GetInstance()` all become fields of a context passed by
//! reference. [`AlgoContext`] is that context.
//!
//! # What is in it, and what is not
//!
//! - `Settings()` (`pcbnew/router/pns_algo_base.cpp:28`) becomes
//!   [`AlgoContext::settings`].
//! - `Dbg()` (`pcbnew/router/pns_algo_base.h:78`) becomes
//!   [`AlgoContext::debug`], a value rather than a nullable pointer.
//! - The rule resolver is not on `ALGO_BASE` at all: KiCad reaches it
//!   through `m_world->GetRuleResolver()`, a pointer the node holds. This
//!   crate's [`crate::node::World`] deliberately does not store one
//!   (`DESIGN.md` section 4.5), so it travels here as
//!   [`AlgoContext::resolver`].
//! - `Router()` (`:54`) is the singleton itself and has no counterpart.
//! - `Logger()` / `SetLogger` (`:63`, `:65`) belong to the event log of
//!   note 03 section 7, which is milestone 5.
//! - `VisibleViewArea()` (`:83`) has exactly one consumer in KiCad's
//!   tree, `SHOVE::runOptimizer` (`pcbnew/router/pns_shove.cpp:2040`),
//!   and note 03 section 5.5 confirms the walkaround never restricts
//!   itself to a viewport. It arrives with the shove, as an optimizer
//!   parameter rather than as ambient state.

use crate::debug::{DebugDecorator, NO_DEBUG};
use crate::rules::RuleResolver;
use crate::settings::RoutingSettings;

/// Everything an algorithm needs that is not the world it works on.
///
/// The replacement for `ALGO_BASE`'s three inherited fields; see the
/// module documentation. It is deliberately cheap to build and holds only
/// borrows, so a caller can make one per call rather than storing it.
///
/// The world is **not** here. It is passed as `&mut World` beside the
/// context, because the algorithms mutate the node arena and the caches
/// while everything in here is read only.
#[derive(Copy, Clone)]
pub struct AlgoContext<'a> {
  /// The rule oracle every clearance query goes through.
  ///
  /// KiCad reaches this through `NODE::GetRuleResolver`
  /// (`pcbnew/router/pns_node.h:139`); this crate passes it explicitly so
  /// that a world is a pure data structure.
  pub resolver: &'a dyn RuleResolver,
  /// The settings `ALGO_BASE::Settings()` returns
  /// (`pcbnew/router/pns_algo_base.cpp:28`).
  pub settings: &'a RoutingSettings,
  /// The hook `ALGO_BASE::Dbg()` returns
  /// (`pcbnew/router/pns_algo_base.h:78`), never null.
  pub debug: &'a dyn DebugDecorator,
}

impl<'a> AlgoContext<'a> {
  /// A context that draws nothing.
  ///
  /// The headless case, which is every test and every replay: KiCad's
  /// `m_debugDecorator` is null there
  /// (`pcbnew/router/pns_algo_base.h:46`) and this installs
  /// [`crate::debug::NoDebug`] instead.
  pub fn new(
    resolver: &'a dyn RuleResolver,
    settings: &'a RoutingSettings,
  ) -> Self {
    Self {
      resolver,
      settings,
      debug: &NO_DEBUG,
    }
  }

  /// The same context with a trace hook attached.
  ///
  /// Port of `SetDebugDecorator`,
  /// `pcbnew/router/pns_algo_base.h:73`.
  #[must_use]
  pub fn with_debug(self, debug: &'a dyn DebugDecorator) -> Self {
    Self { debug, ..self }
  }
}

#[cfg(test)]
mod tests {
  use std::cell::RefCell;

  use super::*;
  use crate::rules::FixedClearance;
  use crate::settings::RouterMode;

  /// A decorator that counts what it was told.
  #[derive(Default)]
  struct Counter {
    /// How many messages arrived.
    messages: RefCell<usize>,
  }

  impl DebugDecorator for Counter {
    fn message(&self, text: &str) {
      let _ = text;
      *self.messages.borrow_mut() += 1;
    }
  }

  #[test]
  fn a_default_context_traces_nothing_and_carries_the_settings() {
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);

    assert_eq!(context.settings.mode, RouterMode::Walkaround);
    context.debug.message("dropped on the floor");
  }

  #[test]
  fn a_trace_hook_replaces_the_no_op_one() {
    let rules = FixedClearance::uniform(1000);
    let settings = RoutingSettings::default();
    let counter = Counter::default();
    let context = AlgoContext::new(&rules, &settings).with_debug(&counter);

    context.debug.message("counted");
    context.debug.message("counted again");

    assert_eq!(*counter.messages.borrow(), 2);
  }
}
