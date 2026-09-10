// SPDX-License-Identifier: GPL-3.0-or-later

//! Moving a via out of what it collides with.
//!
//! The iterative half of `PNS::VIA::PushoutForce`
//! (`pcbnew/router/pns_via.cpp:143`) and the two one line movers that go
//! with it. The geometric half, the minimum translation vector between
//! one via and one other item, is [`crate::item::Via::pushout_force`] on
//! the body itself; this is the loop around it that walks a via until
//! nothing is in the way.
//!
//! # Why it is its own module
//!
//! It has two callers that have nothing else in common. The line placer
//! runs it when a head that ends with a via cannot put that via where the
//! cursor is (`pcbnew/router/pns_line_placer.cpp:2115`), and the dragger
//! runs it from `propagateViaForces` when the user drags a via
//! (`pcbnew/router/pns_dragger.cpp:69`). It lived in
//! `src/placer/line_placer.rs` while the placer was the only caller.
//!
//! `src/item.rs` cannot host it, because it needs
//! [`crate::node::World`], a [`crate::node::NodeId`] and an
//! [`AlgoContext`], all of which sit above `item` in the module order of
//! `DESIGN.md` section 9. `src/shove.rs` could, but then the placer and
//! the dragger's mark obstacles path, neither of which shoves anything,
//! would depend on the shove for a helper that has nothing to do with
//! shoving. A leaf module above `node` and `collide` and below every
//! algorithm keeps the arrows pointing one way.

use crate::algo_base::AlgoContext;
use crate::collide::{CollisionSearchOptions, Obstacle};
use crate::geometry::vec2::Vec2;
use crate::item::{Item, ItemBody, Kind};
use crate::node::{NodeId, World};
use crate::rules::ItemRef;

/// Move a via item to a point.
///
/// The `v.SetPos( walkFull.CLastPoint() )` of
/// `pcbnew/router/pns_line_placer.cpp:719`. KiCad's `VIA::SetPos` also
/// recentres the hole the via owns (`pcbnew/router/pns_via.h:208`); a
/// preview via is not in a node and therefore has no hole item, which is
/// what [`via_pushout_force`] documents. A non via item is left alone,
/// where KiCad's typed `VIA&` makes the case unrepresentable.
pub fn move_via_to(item: &mut Item, pos: Vec2) {
  if let ItemBody::Via(body) = item.body_mut() {
    body.set_pos(pos);
  }
}

/// Move a via item by a vector.
///
/// The `mv.SetPos( mv.Pos() + force )` of
/// `pcbnew/router/pns_via.cpp:196` and `:216`, and the
/// `v.SetPos( v.Pos() + force )` of
/// `pcbnew/router/pns_line_placer.cpp:2130`.
pub fn move_via_by(item: &mut Item, delta: Vec2) {
  if let ItemBody::Via(body) = item.body_mut() {
    let pos = body.pos();

    body.set_pos(pos + delta);
  }
}

/// The translation vector that frees a via from one obstacle.
///
/// The `mv.PushoutForce( aNode, obs->m_item, force )` of
/// `pcbnew/router/pns_via.cpp:166`, which is the three argument overload
/// at `:126`; the geometry of it is
/// [`crate::item::Via::pushout_force`].
///
/// KiCad asks the node for `GetClearance( this, aOther, false )`
/// (`:128`). That cannot go through [`World::clearance_between`] here,
/// because a preview via has no [`crate::item::ItemId`] to key the cache
/// on, so the resolver is asked directly with the virtual item rule of
/// `pcbnew/router/pns_node.cpp:148` in front of it.
///
/// `None` when the obstacle handle is stale, when the moving item is not
/// a via, and when the two shapes give a zero translation vector.
fn single_step_force(
  world: &World,
  context: &AlgoContext<'_>,
  moving: &Item,
  obstacle: &Obstacle,
) -> Option<Vec2> {
  let other_id = obstacle.item?;
  let other = world.item(other_id)?;
  let ItemBody::Via(body) = moving.body() else {
    return None;
  };

  // :128, `NODE::GetClearance`.
  let clearance = if moving.is_virtual() || other.is_virtual() {
    0
  } else {
    context
      .resolver
      .clearance(
        ItemRef::unstored(moving),
        Some(ItemRef::stored(other_id, other)),
        false,
      )
      .unwrap_or(-1)
  };

  body.pushout_force(moving.layers(), other, clearance)
}

/// Walk a via out of everything it collides with.
///
/// Port of `VIA::PushoutForce( NODE*, const VECTOR2I&, VECTOR2I&, int,
/// int )`, `pcbnew/router/pns_via.cpp:143`, the iterative overload the
/// line placer needs. A copy of the via is stepped along the minimum
/// translation vector of the first obstacle it meets until nothing is in
/// the way; the answer is the accumulated displacement. Past half the
/// iteration budget an oversized force is taken as a sign that the
/// barycentric direction is wrong, and the step follows `direction`, the
/// lead vector, instead.
///
/// `None` is KiCad's `false`: either the budget ran out (`:224`), or an
/// obstacle was reported whose translation vector came out zero (`:174`),
/// which the comment there calls a failure of force propagation.
///
/// # A KiCad no op, reproduced
///
/// The cap on the translation vector at `pcbnew/router/pns_via.cpp:207`
/// reads `force.Resize( threshold )`, and `VECTOR2::Resize` is `const`
/// (`libs/kimath/include/math/vector2d.h:186`), so its result is thrown
/// away and the step is **not** capped on that branch. Only the lead
/// vector branch, which assigns, really limits its step. Note 03 section
/// 9.6 asks for the code to be reproduced rather than the comment's
/// intent, because a regression suite built from KiCad logs pins the
/// code, so the cap is left out here as well and the `threshold` only
/// decides which of the two branches runs.
///
/// # No hole
///
/// The via handed in is the placer's preview via: an [`Item`] that is in
/// no node, so it carries no hole item and the pushout resolves copper
/// clearances only. KiCad's `VIA` always owns a hole, so its pushout also
/// answers to hole to hole and copper to hole rules. Giving a preview via
/// a hole needs an arena item for it, which is a change to
/// [`crate::line::LineVia`].
///
/// The shove does not have the same gap: a head that ends with a via has
/// that via **stored** for the duration of the run
/// (`pcbnew/router/pns_shove.cpp:2505`), and [`World::add_via`] drills a
/// hole alongside it, so everything the shove measures against the head's
/// via answers to the hole rules as well.
#[allow(clippy::too_many_arguments)]
pub fn via_pushout_force(
  world: &World,
  context: &AlgoContext<'_>,
  node: NodeId,
  via: &Item,
  direction: Vec2,
  collision_mask: Kind,
  max_iterations: u32,
) -> Option<Vec2> {
  let layers = via.layers();
  let ItemBody::Via(body) = via.body() else {
    return None;
  };

  // :181. KiCad calls it "another stupid heuristic": a quarter of the
  // via's own copper diameter.
  let threshold = body.diameter(layers, body.effective_layer(layers, 0)) / 4;
  let mut moving = via.clone();
  let mut total = Vec2::new(0, 0);
  let mut iteration: u32 = 0;

  // :155
  let options = CollisionSearchOptions {
    limit_count: Some(1),
    kind_mask: collision_mask,
    use_clearance_epsilon: false,
    ..CollisionSearchOptions::default()
  };

  // :153
  while iteration < max_iterations {
    // :160
    let Some(obstacle) = world.check_colliding(
      node,
      ItemRef::unstored(&moving),
      context.resolver,
      &options,
    ) else {
      break;
    };

    // :166
    let Some(force) = single_step_force(world, context, &moving, &obstacle)
    else {
      // :174
      return None;
    };

    let magnitude = force.euclidean_norm();

    // :187
    let step = if iteration > max_iterations / 2 && magnitude > threshold {
      direction.resize(threshold)
    } else {
      // :200, whose cap at `:207` is the no op the doc comment says
      // KiCad throws away.
      force
    };

    total += step;

    move_via_by(&mut moving, step);

    iteration += 1;
  }

  // :224
  if iteration == max_iterations {
    return None;
  }

  Some(total)
}
