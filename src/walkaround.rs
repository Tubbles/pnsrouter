// SPDX-License-Identifier: GPL-3.0-or-later

//! Bending a whole line around everything in its way.
//!
//! Port of `PNS::WALKAROUND` (`pcbnew/router/pns_walkaround.h:36`,
//! `pcbnew/router/pns_walkaround.cpp`), the second of the three head
//! routines. Where [`Line::walkaround`] bends a line around **one** hull,
//! this drives that primitive to a fixed point: find the nearest
//! obstacle, gather everything touching it, bend the line around the
//! whole group, and repeat until the line is clear or the iteration
//! budget runs out.
//!
//! # The three policies
//!
//! One [`Walkaround::route`] call runs up to three independent state
//! machines over the same starting line: clockwise, counter clockwise and
//! shortest. Each keeps its own line and its own status, and the caller
//! picks a winner from [`WalkaroundResult`]. The line placer enables the
//! two windings and compares their lengths itself
//! (`pcbnew/router/pns_line_placer.cpp:567`); the shove and both draggers
//! enable only [`WalkPolicy::Shortest`], which decides per obstacle
//! instead of per route (`pcbnew/router/pns_shove.cpp:817`,
//! `pcbnew/router/pns_dragger.cpp:693`).
//!
//! Winding is not a traversal rule. A hull is clockwise by construction,
//! so walking the other way round is expressed as walking a reversed hull
//! (`pcbnew/router/pns_line.cpp:397`); see `crate::geometry::hull`.
//!
//! # What is not ported
//!
//! Note 03 section 9.5 lists the dead members, and none of them are here:
//! `SetForceWinding` / `m_forceWinding` / `m_forceCw`,
//! `SetPickShortestPath` / `m_useShortestPath`, `m_forceLongerPath`,
//! `m_cursorPos`, `m_lastP`, and the declared but never defined
//! `STATUS Route( const LINE&, LINE&, bool )` overload
//! (`pcbnew/router/pns_walkaround.h:112` to `:158`). Nothing in KiCad's
//! tree writes or reads any of them.
//!
//! Three more members are dead without being on that list, and are left
//! out for the same reason. `m_currentObstacle` and `m_currentCluster`
//! (`pcbnew/router/pns_walkaround.h:159`, `:160`) are never written:
//! `singleStep` keeps its clusters in a local array. `m_restrictedVertices`
//! (`:153`) is filled by `RestrictToCluster` with the cluster's solid
//! anchors and read nowhere. `m_initialLength` (`:163`) is written by
//! `Route` and read only inside two debug messages, so it is computed at
//! the trace point here instead of stored.
//!
//! The per cluster wall clock bail out
//! (`pcbnew/router/pns_walkaround.cpp:136` to `:154`) is not ported
//! either: `DESIGN.md` section 8 forbids reading a clock inside an
//! algorithm. Termination does not depend on it, because a cluster is a
//! finite item list and each item costs one [`Line::walkaround`] call,
//! which carries its own 1000 step cap
//! (`crate::line::WALKAROUND_ITERATION_LIMIT`). Dropping it makes a
//! pathological cluster slower, never unbounded, and it removes a source
//! of run to run disagreement that a golden file could not survive.

use std::collections::BTreeSet;

use crate::algo_base::AlgoContext;
use crate::collide::CollisionSearchOptions;
use crate::item::{ItemId, Kind};
use crate::line::Line;
use crate::node::{NodeId, World, simplified_hull};
use crate::settings::RoutingSettings;

/// How many policies run side by side.
///
/// Port of `MaxWalkPolicies`, `pcbnew/router/pns_walkaround.h:38`.
pub const MAX_WALK_POLICIES: usize = 3;

/// How much longer than the initial path a walk may get before it is
/// abandoned.
///
/// Port of the `m_lengthExpansionFactor` member initialiser,
/// `pcbnew/router/pns_walkaround.h:55`. The line placer never calls
/// [`Walkaround::set_length_limit`], so this is the factor it routes
/// with. The two draggers do call it, with 30.0 and 3.0
/// (`pcbnew/router/pns_dragger.cpp:692`,
/// `pcbnew/router/pns_multi_dragger.cpp:391`), which is the opposite
/// direction from what the names suggest: a bigger factor is a looser
/// limit.
pub const DEFAULT_LENGTH_EXPANSION_FACTOR: f64 = 10.0;

// ---------------------------------------------------------------------
// Policies, statuses and results
// ---------------------------------------------------------------------

/// Which way round the obstacles a walk goes.
///
/// Port of `WALK_POLICY`, `pcbnew/router/pns_walkaround.h:70`. The
/// discriminants are KiCad's, because they are the indices into the
/// result's two parallel arrays.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub enum WalkPolicy {
  /// Always leave every obstacle on the left. `WP_CW = 0`.
  Clockwise = 0,
  /// Always leave every obstacle on the right. `WP_CCW = 1`.
  CounterClockwise = 1,
  /// Decide per obstacle, preferring the shorter detour that does not
  /// collide. `WP_SHORTEST = 2`.
  Shortest = 2,
}

impl WalkPolicy {
  /// The three policies in KiCad's numeric order.
  pub const ALL: [WalkPolicy; MAX_WALK_POLICIES] = [
    WalkPolicy::Clockwise,
    WalkPolicy::CounterClockwise,
    WalkPolicy::Shortest,
  ];

  /// The index of this policy's slot in a [`WalkaroundResult`].
  const fn index(self) -> usize {
    self as usize
  }
}

/// How far one policy got.
///
/// Port of `WALKAROUND::STATUS`,
/// `pcbnew/router/pns_walkaround.h:61`.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum WalkaroundStatus {
  /// Still walking. `ST_IN_PROGRESS = 0`. Never leaves
  /// [`Walkaround::route`]: the final classification turns it into
  /// [`WalkaroundStatus::AlmostDone`]
  /// (`pcbnew/router/pns_walkaround.cpp:375`).
  InProgress,
  /// A path exists but it does not reach the requested endpoint.
  /// `ST_ALMOST_DONE`.
  AlmostDone,
  /// A collision free path from the original start to the original end.
  /// `ST_DONE`.
  Done,
  /// No path. `ST_STUCK`.
  Stuck,
  /// Nothing was attempted. `ST_NONE`, the value a fresh `RESULT` carries
  /// (`pcbnew/router/pns_walkaround.h:82`).
  #[default]
  None,
}

/// What one [`Walkaround::route`] call produced.
///
/// Port of `WALKAROUND::RESULT`,
/// `pcbnew/router/pns_walkaround.h:77`, whose two parallel arrays are
/// reached here through [`WalkaroundResult::status`] and
/// [`WalkaroundResult::line`].
///
/// # Read only the policies you asked for
///
/// [`Walkaround::route`]'s final classification loop runs over **every**
/// slot, enabled or not (`pcbnew/router/pns_walkaround.cpp:369`), so a
/// disabled policy comes back as [`WalkaroundStatus::AlmostDone`]
/// carrying an untouched copy of the initial path, or as
/// [`WalkaroundStatus::Stuck`] when that path had no segments. A caller
/// must therefore key off its own policy selection and never off "this
/// slot is not [`WalkaroundStatus::None`]". See
/// [`Walkaround::set_allowed_policies`] for why the loop is reproduced.
#[derive(Clone, PartialEq, Debug, Default)]
pub struct WalkaroundResult {
  /// One status per policy, indexed by [`WalkPolicy`].
  status: [WalkaroundStatus; MAX_WALK_POLICIES],
  /// One line per policy, indexed by [`WalkPolicy`].
  lines: [Line; MAX_WALK_POLICIES],
}

impl WalkaroundResult {
  /// How far one policy got.
  pub const fn status(&self, policy: WalkPolicy) -> WalkaroundStatus {
    self.status[policy.index()]
  }

  /// The line one policy produced.
  pub const fn line(&self, policy: WalkPolicy) -> &Line {
    &self.lines[policy.index()]
  }

  /// The line one policy produced, taken out of the result.
  pub fn into_line(self, policy: WalkPolicy) -> Line {
    let [clockwise, counter_clockwise, shortest] = self.lines;

    match policy {
      WalkPolicy::Clockwise => clockwise,
      WalkPolicy::CounterClockwise => counter_clockwise,
      WalkPolicy::Shortest => shortest,
    }
  }
}

// ---------------------------------------------------------------------
// Walkaround
// ---------------------------------------------------------------------

/// The three policy state machines over one node.
///
/// Port of `PNS::WALKAROUND`, `pcbnew/router/pns_walkaround.h:36`. Build
/// one, tell it which policies to run and what may count as an obstacle,
/// then call [`Walkaround::route`] once per head.
///
/// # What it holds and what it is handed
///
/// The node is stored, because it is the one thing KiCad's `SetWorld`
/// (`:89`) changes between routes and because it names the branch every
/// query goes to. Everything else an algorithm needs, the rule resolver,
/// the settings and the debug hook, arrives per call in an
/// [`AlgoContext`] instead of being reachable through a router singleton;
/// see `DESIGN.md` section 8 and note 03 section 9.4.
///
/// `ALGO_BASE`'s own contribution is described in [`crate::algo_base`].
#[derive(Clone, Debug)]
pub struct Walkaround {
  /// The branch every query runs against. Port of `m_world` (`:143`),
  /// which `SetWorld` (`:89`) replaces.
  node: NodeId,
  /// How many rounds have been walked. Port of `m_iteration` (`:145`).
  iteration: u32,
  /// How many rounds are allowed. Port of `m_iterationLimit` (`:146`).
  iteration_limit: u32,
  /// Which kinds of item may stop the line. Port of `m_itemMask` (`:147`).
  item_mask: Kind,
  /// The only items that may stop the line, when the search is confined.
  /// Port of `m_restrictedSet` (`:152`).
  restricted_set: BTreeSet<ItemId>,
  /// Whether a runaway walk is abandoned. Port of `m_lengthLimitOn`
  /// (`:155`).
  length_limit_on: bool,
  /// How far a walk may run away. Port of `m_lengthExpansionFactor`
  /// (`:157`).
  length_expansion_factor: f64,
  /// Which policies run. Port of `m_enabledPolicies` (`:158`).
  enabled_policies: [bool; MAX_WALK_POLICIES],
  /// Every item resolved in an earlier round, for the shortest policy's
  /// back off. Port of `m_processedItems` (`:161`).
  processed_items: BTreeSet<ItemId>,
  /// The three lines and statuses under construction. Port of
  /// `m_currentResult` (`:162`).
  result: WalkaroundResult,
}

impl Walkaround {
  /// A walkaround over one branch, with KiCad's iteration budget.
  ///
  /// Port of the constructor at `pcbnew/router/pns_walkaround.h:41`,
  /// which reads [`RoutingSettings::walkaround_iteration_limit`] through
  /// `Settings()` (`:56`). The settings are read here and not stored, so
  /// a later change to them does not silently retune a live walkaround;
  /// [`Walkaround::set_iteration_limit`] is how the shove and the
  /// draggers re impose it (`pcbnew/router/pns_shove.cpp:818`).
  ///
  /// # No policy is enabled
  ///
  /// KiCad's constructor leaves `m_enabledPolicies` **uninitialised**,
  /// and every one of its five call sites happens to call
  /// `SetAllowedPolicies` before `Route`. This starts with all three off,
  /// which is the safe reading of an indeterminate value: a caller that
  /// forgets gets three untouched copies of its input rather than a walk
  /// it did not ask for.
  pub fn new(node: NodeId, settings: &RoutingSettings) -> Self {
    Self {
      node,
      iteration: 0,
      iteration_limit: settings.walkaround_iteration_limit,
      // `:46`
      item_mask: Kind::ANY,
      restricted_set: BTreeSet::new(),
      // `:53`
      length_limit_on: true,
      // `:55`
      length_expansion_factor: DEFAULT_LENGTH_EXPANSION_FACTOR,
      enabled_policies: [false; MAX_WALK_POLICIES],
      processed_items: BTreeSet::new(),
      result: WalkaroundResult::default(),
    }
  }

  /// The branch the queries run against.
  pub const fn world(&self) -> NodeId {
    self.node
  }

  /// Route against a different branch.
  ///
  /// Port of `SetWorld`, `pcbnew/router/pns_walkaround.h:89`.
  pub const fn set_world(&mut self, node: NodeId) {
    self.node = node;
  }

  /// How many rounds a route may take.
  ///
  /// Port of `SetIterationLimit`,
  /// `pcbnew/router/pns_walkaround.h:94`. Every caller passes
  /// [`RoutingSettings::walkaround_iteration_limit`], which the
  /// constructor already installed; the shove's call carries a "fixme:
  /// make configurable" of its own (`pcbnew/router/pns_shove.cpp:818`).
  pub const fn set_iteration_limit(&mut self, limit: u32) {
    self.iteration_limit = limit;
  }

  /// Whether only solids may stop the line.
  ///
  /// Port of `SetSolidsOnly`,
  /// `pcbnew/router/pns_walkaround.h:99`, which is
  /// [`Walkaround::set_allowed_kinds`] with [`Kind::SOLID`] or
  /// [`Kind::ANY`]. The line placer walks around solids only while it is
  /// placing a via (`pcbnew/router/pns_line_placer.cpp:711`) and the
  /// shove turns it off explicitly (`pcbnew/router/pns_shove.cpp:815`).
  pub const fn set_solids_only(&mut self, solids_only: bool) {
    self.item_mask = if solids_only { Kind::SOLID } else { Kind::ANY };
  }

  /// Which kinds of item may stop the line.
  ///
  /// Port of `SetItemMask`,
  /// `pcbnew/router/pns_walkaround.h:107`. It becomes the collision
  /// search's kind mask (`pcbnew/router/pns_walkaround.cpp:54`), so a
  /// kind that is masked out is not an obstacle at all rather than an
  /// obstacle that is ignored.
  pub const fn set_allowed_kinds(&mut self, mask: Kind) {
    self.item_mask = mask;
  }

  /// Whether a runaway walk is abandoned, and how long it may get.
  ///
  /// Port of `SetLengthLimit`,
  /// `pcbnew/router/pns_walkaround.h:130`. The factor is compared against
  /// the ratio of the current line's length to the initial path's
  /// (`pcbnew/router/pns_walkaround.cpp:344`), and a policy over it is
  /// marked [`WalkaroundStatus::AlmostDone`] rather than left to burn
  /// through the iteration limit. See
  /// [`DEFAULT_LENGTH_EXPANSION_FACTOR`].
  pub const fn set_length_limit(&mut self, enabled: bool, factor: f64) {
    self.length_limit_on = enabled;
    self.length_expansion_factor = factor;
  }

  /// Which of the three policies run.
  ///
  /// Port of `SetAllowedPolicies`,
  /// `pcbnew/router/pns_walkaround.cpp:396`, which clears all three flags
  /// and then sets the ones it was given. A repeated policy is harmless,
  /// as it is there.
  pub fn set_allowed_policies(&mut self, policies: &[WalkPolicy]) {
    self.enabled_policies = [false; MAX_WALK_POLICIES];

    for policy in policies {
      self.enabled_policies[policy.index()] = true;
    }
  }

  /// Whether a policy is enabled.
  pub const fn policy_enabled(&self, policy: WalkPolicy) -> bool {
    self.enabled_policies[policy.index()]
  }

  /// Confine the search to one cluster of items.
  ///
  /// Port of `RestrictToCluster`,
  /// `pcbnew/router/pns_walkaround.cpp:71`, which fills `m_restrictedSet`
  /// with the cluster's items **and their holes** so that
  /// `nearestObstacle` sees nothing else. Its only caller is the shove
  /// (`pcbnew/router/pns_shove.cpp:816`), which has already decided which
  /// obstacle it is trying to get past and does not want the walk
  /// wandering off to a different one.
  ///
  /// `enabled` is KiCad's `aEnabled`: false clears the restriction and
  /// ignores the cluster, which is how a reused walkaround is unconfined.
  ///
  /// Holes are added here rather than by the caller because
  /// [`crate::collide`] reports a pad's hole as the obstacle when the
  /// hole is what the clearance was measured against, and a restriction
  /// that named only the pad would then filter out the collision that
  /// motivated it.
  ///
  /// KiCad also fills `m_restrictedVertices` with the cluster's solid
  /// anchors (`:87` to `:91`); nothing reads that vector, so it is not
  /// ported.
  pub fn restrict_to_cluster(
    &mut self,
    world: &World,
    enabled: bool,
    cluster: &[ItemId],
  ) {
    self.restricted_set.clear();

    if !enabled {
      return;
    }

    for id in cluster {
      self.restricted_set.insert(*id);

      if let Some(hole) = world.item(*id).and_then(crate::item::Item::hole) {
        self.restricted_set.insert(hole);
      }
    }
  }

  /// The items the search is confined to, empty when it is not.
  pub const fn restricted_set(&self) -> &BTreeSet<ItemId> {
    &self.restricted_set
  }

  // -----------------------------------------------------------------
  // Routing
  // -----------------------------------------------------------------

  /// Walk the line around everything in its way.
  ///
  /// Port of `WALKAROUND::Route( const LINE& )`,
  /// `pcbnew/router/pns_walkaround.cpp:303`. The loop is
  /// the private `single_step` until every enabled policy has stopped
  /// making progress or the iteration budget is spent, followed by one
  /// classification pass.
  ///
  /// The `#if 0` block at `:311` to `:323`, a sketch for placing a via in
  /// the middle of a track, is not ported.
  ///
  /// # The classification pass
  ///
  /// `:369` to `:390`, in KiCad's order, which matters because the three
  /// tests overwrite each other:
  ///
  /// 1. a policy still in progress becomes
  ///    [`WalkaroundStatus::AlmostDone`], since the loop above it ran out
  ///    of budget rather than out of obstacles;
  /// 2. a line with no segments, or one that no longer starts where the
  ///    input did, is [`WalkaroundStatus::Stuck`];
  /// 3. a line that does not end where the input did is
  ///    [`WalkaroundStatus::AlmostDone`], **even if the test above just
  ///    called it stuck**.
  ///
  /// The loop runs over disabled slots as well; see
  /// [`WalkaroundResult`].
  pub fn route(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    initial_path: &Line,
  ) -> WalkaroundResult {
    let initial_length = initial_path.shape().length();

    // :325
    self.start(initial_path);

    // :327
    self.processed_items.clear();

    if context.debug.is_enabled() {
      context.debug.add_line(initial_path, "initial-path");
    }

    // :331
    while self.iteration < self.iteration_limit {
      self.single_step(world, context);

      let mut still_in_progress = false;

      // :337
      for policy in WalkPolicy::ALL {
        let slot = policy.index();

        if !self.enabled_policies[slot] {
          continue;
        }

        // :344. KiCad divides two `double`s, so an initial path of zero
        // length gives an infinity or a NaN and the comparison below
        // answers exactly as it does there.
        let length_factor = self.result.lines[slot].shape().length() as f64
          / initial_length as f64;

        // :349 to :353
        if self.length_limit_on
          && self.result.status[slot] != WalkaroundStatus::Done
          && length_factor > self.length_expansion_factor
        {
          self.result.status[slot] = WalkaroundStatus::AlmostDone;
        }

        if context.debug.is_enabled() {
          context.debug.message(&format!(
            "check-wp iter {} st {:?} i {slot} lf {length_factor:.1}",
            self.iteration, self.result.status[slot]
          ));
        }

        // :357
        if self.result.status[slot] == WalkaroundStatus::InProgress {
          still_in_progress = true;
        }
      }

      // :362
      if !still_in_progress {
        break;
      }

      self.iteration += 1;
    }

    // :369
    let first = initial_path.shape().point_count() > 0;
    let last = initial_path.last_point();

    for policy in WalkPolicy::ALL {
      let slot = policy.index();
      let status = &mut self.result.status[slot];
      let line = &mut self.result.lines[slot];

      // :374
      line.clear_links();

      // :375
      if *status == WalkaroundStatus::InProgress {
        *status = WalkaroundStatus::AlmostDone;
      }

      // :378. KiCad's `||` short circuits, so `CPoint( 0 )` is only
      // reached for a line that has a segment and therefore two points;
      // `aInitialPath.CPoint( 0 )` on an empty input is undefined there
      // and is the `first` guard here.
      if line.segment_count() < 1
        || !first
        || line.point(0) != initial_path.point(0)
      {
        *status = WalkaroundStatus::Stuck;
      }

      // :383. Not an `else`: a line that is stuck by the test above and
      // ends somewhere other than the input did comes out as almost done.
      if line.point_count() > 0 && line.last_point() != last {
        *status = WalkaroundStatus::AlmostDone;
      }
    }

    self.result.clone()
  }

  /// Put every policy back to the start of the line.
  ///
  /// Port of `start`, `pcbnew/router/pns_walkaround.cpp:38`. It seeds
  /// **all three** slots regardless of
  /// [`Walkaround::set_allowed_policies`], which is the asymmetry note 03
  /// section 5.2 asks to preserve: a disabled slot ends the route holding
  /// an untouched copy of the input.
  ///
  /// The links are cleared (`:45`) because the walked line is a new
  /// geometry that no stored segment corresponds to any more.
  fn start(&mut self, initial_path: &Line) {
    self.iteration = 0;

    for policy in WalkPolicy::ALL {
      let slot = policy.index();

      self.result.status[slot] = WalkaroundStatus::InProgress;
      self.result.lines[slot] = initial_path.clone();
      self.result.lines[slot].clear_links();
    }
  }

  /// The obstacle one policy's line runs into first.
  ///
  /// Port of `nearestObstacle`,
  /// `pcbnew/router/pns_walkaround.cpp:50`: the item mask, the restricted
  /// set as the search filter (`:56` to `:64`), and the clearance epsilon
  /// left on (`:66`).
  ///
  /// The handle is unwrapped here rather than by the caller. An obstacle
  /// with no handle is unreachable, because every candidate a collision
  /// search offers came out of the spatial index and is therefore stored,
  /// and folding the two cases together keeps the caller from having to
  /// invent a status for one that cannot happen.
  fn nearest_obstacle(
    &self,
    world: &mut World,
    context: &AlgoContext<'_>,
    line: &Line,
  ) -> Option<ItemId> {
    let options = CollisionSearchOptions {
      // :54
      kind_mask: self.item_mask,
      // :56. An empty set means "no filter", not "nothing is eligible".
      restricted_set: (!self.restricted_set.is_empty())
        .then_some(&self.restricted_set),
      // :66
      use_clearance_epsilon: true,
      ..CollisionSearchOptions::default()
    };

    world
      .nearest_obstacle(
        self.node,
        line,
        context.resolver,
        &options,
        context.settings.corner_mode,
      )
      .and_then(|obstacle| obstacle.item)
  }

  /// One round of all three policies.
  ///
  /// Port of `singleStep`, `pcbnew/router/pns_walkaround.cpp:94`, in its
  /// two halves: gather one cluster per policy still in progress, then
  /// bend each policy's line around its cluster.
  ///
  /// # The discarded return value
  ///
  /// KiCad's `singleStep` is declared `bool` and returns `ST_IN_PROGRESS`
  /// (`:299`), which is the enumerator `0`, which is `false`; `Route`
  /// ignores it (`:333`). Note 03 section 9.6 item 5 lists it as a
  /// behaviour to decide on deliberately. The decision here: the function
  /// returns nothing. A `bool` that is a constant `false` under a name
  /// that reads as "did something happen" is not a behaviour a caller can
  /// depend on, since no caller looks at it and the only value it can
  /// take is the one that means "no"; keeping it would be keeping a
  /// misleading signature, not a semantic.
  fn single_step(&mut self, world: &mut World, context: &AlgoContext<'_>) {
    let mut clusters: [Vec<ItemId>; MAX_WALK_POLICIES] =
      [Vec::new(), Vec::new(), Vec::new()];

    // :99, half one.
    for policy in WalkPolicy::ALL {
      let slot = policy.index();

      if !self.enabled_policies[slot] {
        continue;
      }

      if context.debug.is_enabled() {
        context.debug.add_line(
          &self.result.lines[slot],
          &format!(
            "current (policy {slot}, stat {:?})",
            self.result.status[slot]
          ),
        );
      }

      // :109
      if self.result.status[slot] != WalkaroundStatus::InProgress {
        continue;
      }

      // :112
      let Some(obstacle) =
        self.nearest_obstacle(world, context, &self.result.lines[slot])
      else {
        // :117. Nothing is in the way any more.
        self.result.status[slot] = WalkaroundStatus::Done;
        continue;
      };

      // :124. The cluster, not the single item, is what one round
      // clears.
      let line = &self.result.lines[slot];

      clusters[slot] = world.assemble_cluster(
        self.node,
        obstacle,
        line.layer(),
        None,
        line.net(),
        context.resolver,
      );

      if context.debug.is_enabled() {
        context.debug.add_item(
          obstacle,
          &format!("col-item cl-items={}", clusters[slot].len()),
        );
      }
    }

    // :199 and :206, the two plain windings.
    for (policy, clockwise) in [
      (WalkPolicy::Clockwise, true),
      (WalkPolicy::CounterClockwise, false),
    ] {
      let slot = policy.index();

      if !self.enabled_policies[slot] {
        continue;
      }

      let mut line = std::mem::take(&mut self.result.lines[slot]);
      let walked =
        process_cluster(world, context, &clusters[slot], &mut line, clockwise);

      self.result.lines[slot] = line;

      if !walked {
        self.result.status[slot] = WalkaroundStatus::Stuck;
      }
    }

    // :213, the shortest path policy.
    if self.enabled_policies[WalkPolicy::Shortest.index()] {
      self.single_step_shortest(world, context, &clusters);
    }
  }

  /// One round of the shortest path policy.
  ///
  /// Port of the `WP_SHORTEST` block of `singleStep`,
  /// `pcbnew/router/pns_walkaround.cpp:213` to `:297`. Unlike the two
  /// plain windings it tries **both** directions from the same starting
  /// line every round and picks between them:
  ///
  /// - when both walks succeeded and either both or neither of them still
  ///   collides, the shorter one wins and the other is kept as the
  ///   alternate (`:237`);
  /// - otherwise the one that does not collide wins, with no alternate
  ///   (`:250`);
  /// - when only one walk succeeded, it wins (`:256`).
  ///
  /// The winner is then checked against every item resolved in an earlier
  /// round, and a winner that collides with one of them is dropped in
  /// favour of the alternate (`:267` to `:284`). That back off is what
  /// stops the policy from ping ponging: a winding that undoes the
  /// clearance won in an earlier round is refused.
  ///
  /// # Deviation: how the back off asks
  ///
  /// KiCad calls `LINE::Collide( item, m_world, layer, ctx )` directly on
  /// each processed item (`:271`), which is the item level test with the
  /// spatial index bypassed. This asks
  /// [`World::check_colliding_line`] with the processed items as the
  /// search filter, which is the same question through the node: same
  /// clearance ladder, same hole expansion, plus the branch's override
  /// filter, which can only remove an item the branch has deleted. The
  /// reason is that a [`Line`] is never the obstacle side of a collision
  /// in this crate (note 02, milestone 2 closing entry), so the direct
  /// call has no counterpart with the arguments that way round.
  fn single_step_shortest(
    &mut self,
    world: &mut World,
    context: &AlgoContext<'_>,
    clusters: &[Vec<ItemId>; MAX_WALK_POLICIES],
  ) {
    let slot = WalkPolicy::Shortest.index();
    let cluster = &clusters[slot];
    let mut path_cw = self.result.lines[slot].clone();
    let mut path_ccw = self.result.lines[slot].clone();

    // :218
    let walked_cw =
      process_cluster(world, context, cluster, &mut path_cw, true);
    let walked_ccw =
      process_cluster(world, context, cluster, &mut path_ccw, false);

    // :221. A walk that failed is not asked whether it collides.
    let options = CollisionSearchOptions::default();
    let collides_cw = walked_cw
      && world
        .check_colliding_line(self.node, &path_cw, context.resolver, &options)
        .is_some();
    let collides_ccw = walked_ccw
      && world
        .check_colliding_line(self.node, &path_ccw, context.resolver, &options)
        .is_some();

    // :231 to :259
    let (shortest, alternate) = match (walked_cw, walked_ccw) {
      (true, true) => {
        if collides_cw == collides_ccw {
          // :239. The strict `>` sends a tie to the clockwise walk.
          if path_cw.shape().length() > path_ccw.shape().length() {
            (Some(path_ccw), Some(path_cw))
          } else {
            (Some(path_cw), Some(path_ccw))
          }
        } else if collides_cw {
          (Some(path_ccw), None)
        } else {
          (Some(path_cw), None)
        }
      }
      (false, true) => (Some(path_ccw), None),
      (true, false) => (Some(path_cw), None),
      (false, false) => (None, None),
    };

    // :261 to :284. The back off against the earlier rounds.
    let shortest = match shortest {
      Some(line) if self.collides_with_processed(world, context, &line) => {
        alternate
      }
      other => other,
    };

    // :286
    match shortest {
      None => self.result.status[slot] = WalkaroundStatus::Stuck,
      Some(line) => self.result.lines[slot] = line,
    }

    // :295
    self.processed_items.extend(cluster.iter().copied());
  }

  /// Whether a candidate undoes an earlier round's clearance.
  ///
  /// The loop at `pcbnew/router/pns_walkaround.cpp:267`; see
  /// [`Walkaround::single_step_shortest`] for why it is asked through the
  /// node. An empty processed set answers false, which is what KiCad's
  /// loop over an empty `std::set` does.
  fn collides_with_processed(
    &self,
    world: &World,
    context: &AlgoContext<'_>,
    line: &Line,
  ) -> bool {
    if self.processed_items.is_empty() {
      return false;
    }

    let options = CollisionSearchOptions {
      restricted_set: Some(&self.processed_items),
      ..CollisionSearchOptions::default()
    };

    world
      .check_colliding_line(self.node, line, context.resolver, &options)
      .is_some()
  }
}

/// Bend one line around every member of one cluster.
///
/// Port of the `processCluster` lambda,
/// `pcbnew/router/pns_walkaround.cpp:131`. Each member contributes its
/// hull at the clearance this line needs from it, and the line is walked
/// around them one after another, so the result clears the whole group
/// and not only the item that was hit.
///
/// `false` is KiCad's `false`: one member's [`Line::walkaround`] failed,
/// and the calling policy goes [`WalkaroundStatus::Stuck`]. The line is
/// left holding whatever the members before the failure produced, exactly
/// as KiCad leaves `aLine`, and the caller overwrites the status rather
/// than the geometry.
///
/// # The two hull keys
///
/// The clearance is asked for **without** the epsilon (`:156`), because
/// the hull has to be the strict rule, and the line's width goes in as
/// the walkaround thickness (`:157`) where
/// [`World::nearest_obstacle`] folds half of it into the clearance
/// instead. Both reach the same geometry through `cl = clearance +
/// thickness / 2` inside the builders, and both therefore occupy their
/// own entry in the hull cache. The two call shapes are kept, so the two
/// entries are kept.
///
/// The `Simplify2` at `:177` runs on the line before every member, not
/// once per cluster, because the previous member's walk can leave
/// duplicate or nearly colinear points that the next one's graph would
/// trip over.
///
/// No node is needed. KiCad reaches `m_world` twice here, for
/// `GetClearance` and for `GetRuleResolver()->HullCache` (`:156`,
/// `:157`), and both of those are world wide rather than per branch in
/// this crate: a clearance is a rule question and a hull is a property of
/// one item.
fn process_cluster(
  world: &mut World,
  context: &AlgoContext<'_>,
  cluster: &[ItemId],
  line: &mut Line,
  clockwise: bool,
) -> bool {
  if context.debug.is_enabled() {
    context.debug.begin_group(
      &format!("cluster-details [cw {}]", i32::from(clockwise)),
      1,
    );
  }

  for item in cluster {
    // :156
    let clearance =
      world.clearance_for_line(*item, line, false, context.resolver);

    // :157
    let Some(cached_hull) =
      world.hull_of(*item, clearance, line.width(), line.layer())
    else {
      // KiCad cannot reach this: `HullCache` is asked about an item it
      // holds a live pointer to. A handle can go stale here, and a
      // cluster member with no hull is one the line cannot be walked
      // around.
      if context.debug.is_enabled() {
        context.debug.end_group();
      }

      return false;
    };

    // :162
    let hull = simplified_hull(cached_hull, context.settings.corner_mode);

    // :177
    line.chain_mut().simplify2(true);

    // :179
    let Some(walked) = line.walkaround(&hull, clockwise) else {
      if context.debug.is_enabled() {
        context.debug.add_shape(&hull, "hull stat 0");
        context.debug.end_group();
      }

      return false;
    };

    // :191
    line.set_shape(walked);

    if context.debug.is_enabled() {
      context.debug.add_shape(&hull, "hull stat 1");
      context.debug.add_item(*item, "item stat 1");
    }
  }

  if context.debug.is_enabled() {
    context.debug.end_group();
  }

  true
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::geometry::line_chain::LineChain;
  use crate::geometry::shape::Shape;
  use crate::geometry::vec2::Vec2;
  use crate::item::{ItemBody, LayerRange, NetId, Solid};
  use crate::rules::FixedClearance;

  /// The clearance every unit test routes to.
  const CLEARANCE: i32 = 100000;

  /// The width of every head.
  const TRACK_WIDTH: i32 = 200000;

  /// The pad's copper radius.
  const PAD_RADIUS: i32 = 400000;

  /// Where the pad sits.
  const PAD_AT: Vec2 = Vec2::new(1000000, 0);

  /// The net the pad is on.
  const PAD_NET: Option<NetId> = Some(NetId(1));

  /// The net every head is on, so that the pad never exempts it.
  const HEAD_NET: Option<NetId> = Some(NetId(2));

  /// A world holding one round pad on layer 0.
  fn one_pad() -> World {
    let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
    let root = world.root();
    let shape = Shape::circle(PAD_AT, PAD_RADIUS);
    let mut item = world.make_item(ItemBody::Solid(Solid::new(shape, PAD_AT)));

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(PAD_NET);
    world.add_solid(root, item, None);

    world
  }

  /// A straight head running east past the pad, north of its centre.
  fn head() -> Line {
    let mut line = Line::new();

    line.set_width(TRACK_WIDTH);
    line.set_layer(0);
    line.set_net(HEAD_NET);
    line.set_shape(LineChain::from_slice(
      &[Vec2::new(0, -300000), Vec2::new(2000000, -300000)],
      false,
    ));

    line
  }

  /// The rule oracle every unit test uses.
  fn rules() -> FixedClearance {
    FixedClearance::uniform(CLEARANCE)
  }

  #[test]
  fn a_fresh_walkaround_takes_kicads_budget_and_runs_no_policy() {
    let world = one_pad();
    let settings = RoutingSettings::default();
    let walkaround = Walkaround::new(world.root(), &settings);

    assert_eq!(walkaround.world(), world.root());
    assert!(walkaround.restricted_set().is_empty());
    for policy in WalkPolicy::ALL {
      assert!(!walkaround.policy_enabled(policy));
    }
  }

  #[test]
  fn set_allowed_policies_replaces_the_whole_selection() {
    let world = one_pad();
    let settings = RoutingSettings::default();
    let mut walkaround = Walkaround::new(world.root(), &settings);

    // Duplicates are harmless, as they are in KiCad's loop
    // (`pcbnew/router/pns_walkaround.cpp:401`).
    walkaround.set_allowed_policies(&[
      WalkPolicy::Shortest,
      WalkPolicy::Shortest,
      WalkPolicy::Clockwise,
    ]);

    assert!(walkaround.policy_enabled(WalkPolicy::Shortest));
    assert!(walkaround.policy_enabled(WalkPolicy::Clockwise));
    assert!(!walkaround.policy_enabled(WalkPolicy::CounterClockwise));

    // A second call replaces rather than adds.
    walkaround.set_allowed_policies(&[WalkPolicy::CounterClockwise]);

    assert!(!walkaround.policy_enabled(WalkPolicy::Shortest));
    assert!(walkaround.policy_enabled(WalkPolicy::CounterClockwise));
  }

  /// Note 03 section 9.6 item 6: the final classification loop runs over
  /// disabled slots too, so they come back as almost done over the input.
  #[test]
  fn a_disabled_policy_still_gets_a_status_and_the_untouched_input() {
    let mut world = one_pad();
    let settings = RoutingSettings::default();
    let rules = rules();
    let context = AlgoContext::new(&rules, &settings);
    let path = head();
    let mut walkaround = Walkaround::new(world.root(), &settings);

    walkaround.set_allowed_policies(&[WalkPolicy::Shortest]);

    let result = walkaround.route(&mut world, &context, &path);

    assert_eq!(result.status(WalkPolicy::Shortest), WalkaroundStatus::Done);
    assert_ne!(
      result.line(WalkPolicy::Shortest).shape().points(),
      path.shape().points()
    );

    for policy in [WalkPolicy::Clockwise, WalkPolicy::CounterClockwise] {
      assert_eq!(
        result.status(policy),
        WalkaroundStatus::AlmostDone,
        "a disabled slot is classified, not left as `None`"
      );
      assert_eq!(result.line(policy).shape().points(), path.shape().points());
    }
  }

  /// An empty input has no segments, so every slot is stuck
  /// (`pcbnew/router/pns_walkaround.cpp:378`).
  #[test]
  fn an_empty_head_is_stuck_in_every_slot() {
    let mut world = one_pad();
    let settings = RoutingSettings::default();
    let rules = rules();
    let context = AlgoContext::new(&rules, &settings);
    let path = Line::new();
    let mut walkaround = Walkaround::new(world.root(), &settings);

    walkaround.set_allowed_policies(&WalkPolicy::ALL);

    let result = walkaround.route(&mut world, &context, &path);

    for policy in WalkPolicy::ALL {
      assert_eq!(result.status(policy), WalkaroundStatus::Stuck);
    }
  }

  #[test]
  fn the_length_limit_abandons_a_walk_that_grows() {
    let mut world = one_pad();
    let settings = RoutingSettings::default();
    let rules = rules();
    let context = AlgoContext::new(&rules, &settings);
    let path = head();

    // Any detour is longer than the straight line, so a factor of one
    // rejects every walk the moment it is made.
    let mut tight = Walkaround::new(world.root(), &settings);

    tight.set_allowed_policies(&WalkPolicy::ALL);
    tight.set_length_limit(true, 1.0);

    let bailed = tight.route(&mut world, &context, &path);

    for policy in WalkPolicy::ALL {
      assert_eq!(bailed.status(policy), WalkaroundStatus::AlmostDone);
    }

    // Turning the limit off lets the same walk finish.
    let mut loose = Walkaround::new(world.root(), &settings);

    loose.set_allowed_policies(&WalkPolicy::ALL);
    loose.set_length_limit(false, 1.0);

    let finished = loose.route(&mut world, &context, &path);

    for policy in WalkPolicy::ALL {
      assert_eq!(finished.status(policy), WalkaroundStatus::Done);
    }
  }

  #[test]
  fn a_result_hands_out_the_line_of_one_policy() {
    let mut world = one_pad();
    let settings = RoutingSettings::default();
    let rules = rules();
    let context = AlgoContext::new(&rules, &settings);
    let path = head();
    let mut walkaround = Walkaround::new(world.root(), &settings);

    walkaround.set_allowed_policies(&WalkPolicy::ALL);

    let result = walkaround.route(&mut world, &context, &path);

    for policy in WalkPolicy::ALL {
      let borrowed = result.line(policy).clone();

      assert_eq!(result.clone().into_line(policy), borrowed);
    }
  }

  #[test]
  fn a_walkaround_can_be_pointed_at_another_branch() {
    let mut world = one_pad();
    let root = world.root();
    let branch = world.branch(root);
    let settings = RoutingSettings::default();
    let rules = rules();
    let context = AlgoContext::new(&rules, &settings);
    let path = head();

    // The pad is removed in the branch, so the same head that has to
    // walk in the root meets nothing there.
    let pad = world.all_items_in_net(root, PAD_NET, Kind::SOLID)[0];
    world.remove(branch, pad);

    let mut walkaround = Walkaround::new(root, &settings);

    walkaround.set_allowed_policies(&[WalkPolicy::Shortest]);

    let walked = walkaround.route(&mut world, &context, &path);

    assert_ne!(
      walked.line(WalkPolicy::Shortest).shape().points(),
      path.shape().points()
    );

    walkaround.set_world(branch);
    assert_eq!(walkaround.world(), branch);

    let clear = walkaround.route(&mut world, &context, &path);

    assert_eq!(clear.status(WalkPolicy::Shortest), WalkaroundStatus::Done);
    assert_eq!(
      clear.line(WalkPolicy::Shortest).shape().points(),
      path.shape().points()
    );
  }
}
