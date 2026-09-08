// SPDX-License-Identifier: GPL-3.0-or-later

//! The hook an algorithm draws its own internals through.
//!
//! Port of `PNS::DEBUG_DECORATOR`
//! (`pcbnew/router/pns_debug_decorator.h:37`), reduced to the calls the
//! walkaround makes. The full decorator, the logger and the JSON event
//! log of note 03 section 7 belong to milestone 5; this module exists so
//! that the algorithms can be written with their trace points in place
//! instead of having them retrofitted later.
//!
//! # What is here and what is not
//!
//! Every method is defaulted to a no op, so an implementation only
//! overrides what it cares about and adding a method later does not break
//! a host.
//!
//! KiCad's calls carry a colour, an override width and a source location
//! (`pcbnew/router/pns_debug_decorator.h:66` to `:91`). None of those are
//! here: they are rendering decisions the host owns, and this crate has
//! no colour type. The message that KiCad formats into every call is
//! kept, because that string is what a trace is actually read for.
//!
//! KiCad's `SetIteration`, `NewStage` and `Clear` (`:64`, `:69`, `:110`)
//! are not here either; the walkaround never calls them, and the router
//! facade that does arrives with milestone 5.
//!
//! # Why a trait and not a struct
//!
//! `ALGO_BASE` holds a `DEBUG_DECORATOR*` that is null in a headless run
//! (`pcbnew/router/pns_algo_base.h:86`). [`NoDebug`] is that null, spelled
//! as a value so that no call site has to test for absence.

use crate::geometry::line_chain::LineChain;
use crate::item::ItemId;
use crate::line::Line;

/// Where an algorithm sends the geometry it is reasoning about.
///
/// Port of `PNS::DEBUG_DECORATOR`,
/// `pcbnew/router/pns_debug_decorator.h:37`. See the module documentation
/// for what was dropped.
///
/// Nothing an implementation does may change an answer: the router is a
/// pure function of its inputs (`DESIGN.md` section 8), and a trace that
/// steered the algorithm would break that.
pub trait DebugDecorator {
  /// Whether anything is listening.
  ///
  /// Port of the `dbg && dbg->IsDebugEnabled()` guard of the `PNS_DBG`
  /// macro (`pcbnew/router/pns_debug_decorator.h:127`), which is what
  /// keeps a headless run from formatting the message strings at all.
  /// Every call site tests it before building the text it would pass in,
  /// so a decorator that answers false costs one branch per trace point.
  ///
  /// It defaults to true, so that an implementation only has to write the
  /// methods it wants. [`NoDebug`] overrides it to false.
  fn is_enabled(&self) -> bool {
    true
  }

  /// A line of prose about what just happened.
  ///
  /// Port of `Message`, `pcbnew/router/pns_debug_decorator.h:66`.
  fn message(&self, text: &str) {
    let _ = text;
  }

  /// Open a nested group of trace entries.
  ///
  /// Port of `BeginGroup`, `pcbnew/router/pns_debug_decorator.h:72`.
  /// `level` is KiCad's `aLevel`, the nesting depth the host indents by.
  fn begin_group(&self, name: &str, level: i32) {
    let _ = (name, level);
  }

  /// Close the group [`DebugDecorator::begin_group`] opened.
  ///
  /// Port of `EndGroup`, `pcbnew/router/pns_debug_decorator.h:75`.
  fn end_group(&self) {}

  /// A stored item worth looking at.
  ///
  /// Port of the `AddItem( const ITEM* )` overload,
  /// `pcbnew/router/pns_debug_decorator.h:81`, for the case where the
  /// item lives in a node. The walkaround uses it for the obstacle it
  /// found and for each member of the obstacle's cluster.
  fn add_item(&self, item: ItemId, text: &str) {
    let _ = (item, text);
  }

  /// A line an algorithm is carrying around.
  ///
  /// The other half of `AddItem( const ITEM* )`,
  /// `pcbnew/router/pns_debug_decorator.h:81`: KiCad passes a `LINE*`
  /// through the same overload because a line is an item there, and it is
  /// not one here (`DESIGN.md` section 4.2).
  fn add_line(&self, line: &Line, text: &str) {
    let _ = (line, text);
  }

  /// A bare chain, which for the walkaround is always a hull.
  ///
  /// Port of the `AddShape( const SHAPE* )` overload,
  /// `pcbnew/router/pns_debug_decorator.h:86`, narrowed to the one shape
  /// kind the walkaround passes it.
  fn add_shape(&self, shape: &LineChain, text: &str) {
    let _ = (shape, text);
  }
}

/// A decorator that draws nothing.
///
/// This is `ALGO_BASE`'s null `m_debugDecorator`
/// (`pcbnew/router/pns_algo_base.h:86`) as a value, so that an algorithm
/// can call the hook unconditionally. It is what
/// [`crate::algo_base::AlgoContext::new`] installs.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct NoDebug;

impl DebugDecorator for NoDebug {
  /// Nothing is listening, so no call site formats a message.
  fn is_enabled(&self) -> bool {
    false
  }
}

/// The one [`NoDebug`] every default context borrows.
///
/// A `'static` value rather than a temporary, so that
/// [`crate::algo_base::AlgoContext::new`] can hand out a reference with
/// the caller's lifetime.
pub static NO_DEBUG: NoDebug = NoDebug;

#[cfg(test)]
mod tests {
  use std::cell::RefCell;

  use super::*;
  use crate::geometry::vec2::Vec2;

  /// A decorator that records what it was told, to show that the default
  /// methods can be overridden one at a time.
  #[derive(Default)]
  struct Recorder {
    /// Every message, in the order it arrived.
    messages: RefCell<Vec<String>>,
  }

  impl DebugDecorator for Recorder {
    fn message(&self, text: &str) {
      self.messages.borrow_mut().push(text.to_string());
    }
  }

  #[test]
  fn the_default_methods_are_no_ops() {
    let debug = NoDebug;
    let line = Line::new();
    let chain = LineChain::from_slice(&[Vec2::new(0, 0)], false);

    assert!(!debug.is_enabled());
    debug.message("nothing happens");
    debug.begin_group("group", 1);
    debug.add_line(&line, "a line");
    debug.add_shape(&chain, "a hull");
    debug.end_group();
  }

  #[test]
  fn an_implementation_may_override_one_method() {
    let debug = Recorder::default();

    assert!(debug.is_enabled());
    debug.message("first");
    debug.end_group();
    debug.message("second");

    assert_eq!(debug.messages.borrow().as_slice(), ["first", "second"]);
  }
}
