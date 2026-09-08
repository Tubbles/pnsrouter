// SPDX-License-Identifier: GPL-3.0-or-later

//! The undo stack of the line placer.
//!
//! Port of `PNS::FIXED_TAIL` (`pcbnew/router/pns_line_placer.h:48`), the
//! stack of fix points a placement can be rolled back through. Note 03
//! section 3.1 lists it as part of the placer's state and section 3.11
//! describes the roll back itself, which is
//! [`crate::placer::line_placer::LinePlacer::undo_last_segment`].
//!
//! # One point per stage
//!
//! KiCad's `STAGE` holds a `NODE*` plus a `std::vector<FIX_POINT>`
//! (`pcbnew/router/pns_line_placer.h:61`), and its constructor takes a
//! line count that it ignores (`pcbnew/router/pns_line_placer.cpp:2158`).
//! Both `AddStage` call sites push exactly one point (`:1436`, `:1720`)
//! and the only reader indexes `pts[0]` (`:1773` and following), so the
//! vector is flattened into [`FixStage`] here. A placer that fixes
//! several lines at once, which is what the vector was meant for, does
//! not exist in KiCad's tree.

use crate::geometry::direction45::Direction45;
use crate::geometry::vec2::Vec2;
use crate::node::NodeId;

/// One undo step: where a leg started and what the placement looked like
/// there.
///
/// `FIXED_TAIL::STAGE` (`pcbnew/router/pns_line_placer.h:61`) with its
/// one `FIX_POINT` (`:54`) folded in; see the module documentation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FixStage {
  /// Where the leg this stage undoes to started. Port of `FIX_POINT::p`
  /// (`pcbnew/router/pns_line_placer.h:58`), which
  /// `LINE_PLACER::FixRoute` fills from `m_fixStart` (`:1720`).
  pub pos: Vec2,
  /// The layer the placement ran on. Port of `FIX_POINT::layer` (`:56`).
  pub layer: i32,
  /// Whether a via was pending. Port of `FIX_POINT::placingVias` (`:57`).
  pub placing_via: bool,
  /// The routing direction to restore. Port of `FIX_POINT::direction`
  /// (`:59`).
  pub direction: Direction45,
  /// The node the fix before this one committed into. Port of
  /// `STAGE::commit` (`pcbnew/router/pns_line_placer.h:93`), a `NODE*`
  /// there and a handle into the world's arena here.
  pub node: NodeId,
}

/// The stack of fix points a placement can be rolled back through.
///
/// Port of `PNS::FIXED_TAIL` (`pcbnew/router/pns_line_placer.h:48`). Its
/// one non obvious rule is in [`FixedTail::pop_stage`]: the bottom stage
/// is handed out but never removed, which is what makes
/// `HasPlacedAnything`'s `StageCount() > 1` mean "at least one fix
/// happened" (`pcbnew/router/pns_line_placer.cpp:1804`).
#[derive(Clone, Debug, Default)]
pub struct FixedTail {
  /// The stages, oldest first. Port of `m_stages`
  /// (`pcbnew/router/pns_line_placer.h:104`).
  stages: Vec<FixStage>,
}

impl FixedTail {
  /// An empty stack.
  ///
  /// Port of `FIXED_TAIL::FIXED_TAIL( int aLineCount )`
  /// (`pcbnew/router/pns_line_placer.cpp:2158`), whose body is empty and
  /// whose argument is never read; the default argument of one is the
  /// only value any caller passes: `LINE_PLACER` declares its
  /// `m_fixedTail` without one (`pcbnew/router/pns_line_placer.h:415`).
  pub const fn new() -> Self {
    Self { stages: Vec::new() }
  }

  /// Forget every stage.
  ///
  /// Port of `Clear`, `pcbnew/router/pns_line_placer.cpp:2170`.
  pub fn clear(&mut self) {
    self.stages.clear();
  }

  /// Push one fix point.
  ///
  /// Port of `AddStage`,
  /// `pcbnew/router/pns_line_placer.cpp:2176`. The parameters are
  /// KiCad's, in KiCad's order.
  pub fn add_stage(
    &mut self,
    pos: Vec2,
    layer: i32,
    placing_via: bool,
    direction: Direction45,
    node: NodeId,
  ) {
    self.stages.push(FixStage {
      pos,
      layer,
      placing_via,
      direction,
      node,
    });
  }

  /// Take the newest stage back, keeping the oldest one.
  ///
  /// Port of `PopStage`,
  /// `pcbnew/router/pns_line_placer.cpp:2194`. Note that it copies the
  /// last stage and only removes it when more than one is left (`:2201`),
  /// so undoing past the first fix keeps answering with the point the
  /// placement started at rather than failing.
  ///
  /// # Deviation
  ///
  /// KiCad writes through a `STAGE&` out parameter and answers a `bool`,
  /// which works there because `STAGE` is default constructible. A
  /// [`FixStage`] carries a [`NodeId`], which has no meaningful default,
  /// so the answer is an [`Option`]. `DESIGN.md` section 11 asks for that
  /// substitution across the crate. The semantics are unchanged.
  pub fn pop_stage(&mut self) -> Option<FixStage> {
    let last = *self.stages.last()?;

    // :2201
    if self.stages.len() > 1 {
      self.stages.pop();
    }

    Some(last)
  }

  /// How many stages are on the stack.
  ///
  /// Port of `StageCount`,
  /// `pcbnew/router/pns_line_placer.cpp:2208`.
  pub fn stage_count(&self) -> usize {
    self.stages.len()
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::node::World;

  /// Shorthand for a point.
  fn at(x: i32, y: i32) -> Vec2 {
    Vec2::new(x, y)
  }

  #[test]
  fn a_fresh_fixed_tail_holds_nothing_and_pops_nothing() {
    let mut tail = FixedTail::new();

    assert_eq!(tail.stage_count(), 0);
    assert!(tail.pop_stage().is_none());
  }

  #[test]
  fn a_stage_comes_back_with_every_field_it_was_pushed_with() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let mut tail = FixedTail::new();
    let direction = Direction45::from_seg(
      &crate::geometry::seg::Seg::new(at(0, 0), at(1000, 0)),
      false,
    );

    tail.add_stage(at(10, 20), 3, true, direction, root);

    assert_eq!(tail.stage_count(), 1);

    let stage = tail.pop_stage().expect("one stage was pushed");

    assert_eq!(stage.pos, at(10, 20));
    assert_eq!(stage.layer, 3);
    assert!(stage.placing_via);
    assert_eq!(stage.direction, direction);
    assert_eq!(stage.node, root);
  }

  #[test]
  fn the_bottom_stage_is_handed_out_but_never_removed() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let mut tail = FixedTail::new();

    tail.add_stage(at(0, 0), 0, false, Direction45::default(), root);
    tail.add_stage(at(1000, 0), 1, true, Direction45::default(), root);

    assert_eq!(tail.stage_count(), 2);

    // The newest stage comes back and leaves the stack.
    let second = tail.pop_stage().expect("two stages were pushed");

    assert_eq!(second.pos, at(1000, 0));
    assert_eq!(tail.stage_count(), 1);

    // The oldest one comes back for ever.
    for _ in 0..3 {
      let first = tail.pop_stage().expect("the bottom stage stays");

      assert_eq!(first.pos, at(0, 0));
      assert_eq!(tail.stage_count(), 1);
    }
  }

  #[test]
  fn clearing_drops_every_stage() {
    let world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let mut tail = FixedTail::new();

    tail.add_stage(at(0, 0), 0, false, Direction45::default(), root);
    tail.add_stage(at(1, 1), 0, false, Direction45::default(), root);
    tail.clear();

    assert_eq!(tail.stage_count(), 0);
    assert!(tail.pop_stage().is_none());
  }
}
