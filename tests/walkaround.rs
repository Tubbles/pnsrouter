// SPDX-License-Identifier: GPL-3.0-or-later

//! Walkaround scenarios on a two layer board, through the public API
//! only.
//!
//! `doc/work/003-walkaround-router.md` asks for the walkaround to be
//! exercised on a board with pads and existing traces rather than only in
//! unit tests, so this file builds one out of plain data the way
//! `tests/world.rs` does and drives
//! [`pnsrouter::walkaround::Walkaround`] over it.
//!
//! Every scenario that claims a route asserts that the route is actually
//! clear, through `World::check_colliding_line`, because a walkaround
//! that reports `Done` over a line that still collides is the failure
//! mode worth catching.

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::collide::CollisionSearchOptions;
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{ItemBody, ItemId, LayerRange, NetId, Segment, Solid};
use pnsrouter::line::Line;
use pnsrouter::node::World;
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::RoutingSettings;
use pnsrouter::walkaround::{WalkPolicy, Walkaround, WalkaroundStatus};

/// The clearance every scenario routes to, in nanometres.
const CLEARANCE: i32 = 100000;

/// The width of the head and of the board's track.
const TRACK_WIDTH: i32 = 200000;

/// The net everything on the board is on.
const BOARD_NET: Option<NetId> = Some(NetId(1));

/// The net the head is on, so that nothing on the board exempts it.
const HEAD_NET: Option<NetId> = Some(NetId(2));

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 400000;

/// How far a head's centreline has to stay from a pad's centre: the
/// pad's copper, the clearance and half the head's width.
const KEEP_OUT: i32 = PAD_RADIUS + CLEARANCE + TRACK_WIDTH / 2;

/// A pad of the board, as a host would hand it over: position and layer.
///
/// The two on layer 0 are far enough apart that their hulls do not touch,
/// so a head crossing both meets two clusters and therefore needs two
/// rounds. The third is the only thing on layer 1, and it sits where no
/// layer 0 obstacle is, which is what makes the board a two layer one for
/// the purposes of these tests.
const PADS: [(i32, i32, i32); 3] =
  [(1000000, 0, 0), (3000000, 0, 0), (1000000, -1500000, 1)];

/// The board's one existing trace: both endpoints and the layer. It runs
/// well south of the layer 0 pads so that a head can meet it on its own.
const TRACK: (i32, i32, i32, i32, i32) =
  (2000000, -2000000, 2000000, -1000000, 0);

/// Everything the builder stored.
struct Board {
  /// The three pads, in the order [`PADS`] lists them.
  pads: Vec<ItemId>,
}

/// Turn the plain data into a world.
fn build() -> (World, Board) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let mut pads = Vec::new();

  for (x, y, layer) in PADS {
    let at = Vec2::new(x, y);
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(BOARD_NET);
    pads.push(world.add_solid(root, item, None));
  }

  let (from_x, from_y, to_x, to_y, layer) = TRACK;
  let seg = Seg::new(Vec2::new(from_x, from_y), Vec2::new(to_x, to_y));
  let body = ItemBody::Segment(Segment::new(seg, TRACK_WIDTH));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::single(layer));
  item.set_net(BOARD_NET);
  world
    .add_segment(root, item, false)
    .expect("the board's track is neither degenerate nor redundant");

  (world, Board { pads })
}

/// A head the placer could be dragging, on the given layer.
fn head_on(layer: i32, points: &[Vec2]) -> Line {
  let mut line = Line::new();

  line.set_width(TRACK_WIDTH);
  line.set_layer(layer);
  line.set_net(HEAD_NET);
  line.set_shape(LineChain::from_slice(points, false));

  line
}

/// A head on the routing layer.
fn head(points: &[Vec2]) -> Line {
  head_on(0, points)
}

/// A straight head that runs east past the first pad only, offset north
/// of its centre so that the two windings are not the same length.
fn one_pad_head() -> Line {
  head(&[Vec2::new(0, -300000), Vec2::new(2000000, -300000)])
}

/// The same head, run far enough east to meet both layer 0 pads.
fn two_pad_head() -> Line {
  head(&[Vec2::new(0, -300000), Vec2::new(4000000, -300000)])
}

/// A head that starts on the first pad's centre, and therefore strictly
/// inside its hull, running south to where nothing else is.
fn trapped_head() -> Line {
  head(&[Vec2::new(1000000, 0), Vec2::new(1000000, -2500000)])
}

/// A head that crosses the board's track and nothing else.
fn track_crossing_head(layer: i32) -> Line {
  head_on(
    layer,
    &[Vec2::new(0, -1500000), Vec2::new(3000000, -1500000)],
  )
}

/// Whether a line is clear of everything the root holds.
fn is_clear(world: &World, line: &Line) -> bool {
  world
    .check_colliding_line(
      world.root(),
      line,
      &FixedClearance::uniform(CLEARANCE),
      &CollisionSearchOptions::default(),
    )
    .is_none()
}

/// Whether a line keeps its distance from a pad's centre, which is what
/// [`is_clear`] says geometrically for a circular pad.
fn clears_pad(line: &Line, pad: usize) -> bool {
  let (x, y, _) = PADS[pad];
  let centre = Vec2::new(x, y);

  (0..line.segment_count())
    .all(|index| line.segment(index).distance_to_point(centre) >= KEEP_OUT)
}

/// A walkaround over the root with every policy enabled.
fn all_policies(world: &World, settings: &RoutingSettings) -> Walkaround {
  let mut walkaround = Walkaround::new(world.root(), settings);

  walkaround.set_allowed_policies(&WalkPolicy::ALL);
  walkaround
}

#[test]
fn one_pad_is_walked_around_in_both_windings_and_the_shortest_is_picked() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let settings = RoutingSettings::default();
  let context = AlgoContext::new(&rules, &settings);
  let path = one_pad_head();

  assert!(
    !is_clear(&world, &path),
    "the straight head crosses the pad"
  );

  let mut walkaround = all_policies(&world, &settings);
  let result = walkaround.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    let line = result.line(policy);

    assert_eq!(
      result.status(policy),
      WalkaroundStatus::Done,
      "policy {policy:?} did not finish"
    );
    assert!(is_clear(&world, line), "policy {policy:?} still collides");
    assert!(
      clears_pad(line, 0),
      "policy {policy:?} cuts the pad's corner"
    );
    // A walk keeps both endpoints, which is what separates a done route
    // from an almost done one.
    assert_eq!(line.point(0), path.point(0));
    assert_eq!(line.last_point(), path.last_point());
    // And it is not the straight line any more.
    assert!(line.point_count() > path.point_count());
  }

  // The head passes north of the pad's centre, so one winding hugs the
  // near side and the other goes the long way round. The shortest policy
  // is the one that has to notice.
  let clockwise = result.line(WalkPolicy::Clockwise).shape().length();
  let counter = result.line(WalkPolicy::CounterClockwise).shape().length();
  let shortest = result.line(WalkPolicy::Shortest).shape().length();

  assert_ne!(
    clockwise, counter,
    "the two windings should differ in length"
  );
  assert_eq!(shortest, clockwise.min(counter));
}

#[test]
fn two_pads_are_cleared_one_round_each() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let settings = RoutingSettings::default();
  let context = AlgoContext::new(&rules, &settings);
  let path = two_pad_head();

  let mut walkaround = all_policies(&world, &settings);
  let result = walkaround.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    let line = result.line(policy);

    assert_eq!(result.status(policy), WalkaroundStatus::Done);
    assert!(is_clear(&world, line), "policy {policy:?} still collides");
    assert!(clears_pad(line, 0));
    assert!(clears_pad(line, 1));
    assert_eq!(line.point(0), path.point(0));
    assert_eq!(line.last_point(), path.last_point());
  }
}

#[test]
fn a_head_that_meets_nothing_comes_back_unchanged() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let settings = RoutingSettings::default();
  let context = AlgoContext::new(&rules, &settings);
  // South of the layer 0 pads and of the track's southern end.
  let path = head(&[Vec2::new(0, -3000000), Vec2::new(3000000, -3000000)]);

  assert!(is_clear(&world, &path));

  let mut walkaround = all_policies(&world, &settings);
  let result = walkaround.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    assert_eq!(result.status(policy), WalkaroundStatus::Done);
    assert_eq!(result.line(policy).shape().points(), path.shape().points());
  }
}

/// A head only ever meets what shares its layer, which is what makes the
/// same straight path two different routes on the two layers.
#[test]
fn the_two_layers_hold_different_obstacles() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let settings = RoutingSettings::default();
  let context = AlgoContext::new(&rules, &settings);
  // On layer 0 this path meets the track and nothing else; on layer 1 it
  // meets the pad that sits under the track's west side and nothing
  // else.
  let on_zero = track_crossing_head(0);
  let on_one = track_crossing_head(1);

  assert!(!is_clear(&world, &on_zero));
  assert!(!is_clear(&world, &on_one));

  let mut first = all_policies(&world, &settings);
  let zero = first.route(&mut world, &context, &on_zero);
  let mut second = all_policies(&world, &settings);
  let one = second.route(&mut world, &context, &on_one);

  for policy in WalkPolicy::ALL {
    assert_eq!(zero.status(policy), WalkaroundStatus::Done);
    assert_eq!(one.status(policy), WalkaroundStatus::Done);
    assert!(is_clear(&world, zero.line(policy)));
    assert!(is_clear(&world, one.line(policy)));
    // The layer 1 head is the only one that has to clear the layer 1
    // pad, and the layer 0 head runs straight over it.
    assert!(clears_pad(one.line(policy), 2));
    assert!(!clears_pad(zero.line(policy), 2));
  }
}

#[test]
fn a_head_that_starts_inside_an_obstacle_is_stuck() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let settings = RoutingSettings::default();
  let context = AlgoContext::new(&rules, &settings);
  // The first point sits on the pad's centre, so it is strictly inside
  // the pad's hull. That is the precondition `LINE::Walkaround` refuses
  // (`pcbnew/router/pns_line.cpp:308`), and short of a graph failure it
  // is the only way a policy reports stuck: a walk can only ever leave
  // from outside the obstacle it is walking around. It is what the
  // placer's `splitHeadTail` exists to prevent.
  let path = trapped_head();

  let mut walkaround = all_policies(&world, &settings);
  let result = walkaround.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    assert_eq!(
      result.status(policy),
      WalkaroundStatus::Stuck,
      "policy {policy:?} should be stuck"
    );
    // The pad is the only obstacle and the walk failed on it, so the
    // line is the one that went in.
    assert_eq!(result.line(policy).shape().points(), path.shape().points());
  }
}

#[test]
fn the_iteration_limit_stops_a_walk_short() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let settings = RoutingSettings::default();
  let context = AlgoContext::new(&rules, &settings);
  let path = two_pad_head();

  // One round clears one cluster, and the two pads are far enough apart
  // to be two clusters, so a budget of one round cannot finish.
  let mut walkaround = all_policies(&world, &settings);

  walkaround.set_iteration_limit(1);

  let stopped = walkaround.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    let line = stopped.line(policy);

    assert_eq!(
      stopped.status(policy),
      WalkaroundStatus::AlmostDone,
      "policy {policy:?} should have run out of rounds"
    );
    // It got past the first pad and no further, so the answer is a real
    // partial route rather than the input.
    assert!(clears_pad(line, 0));
    assert!(!clears_pad(line, 1));
    assert!(!is_clear(&world, line));
  }

  // The same walk with KiCad's default budget finishes.
  assert_eq!(settings.walkaround_iteration_limit, 40);

  let mut unlimited = all_policies(&world, &settings);
  let finished = unlimited.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    assert_eq!(finished.status(policy), WalkaroundStatus::Done);
  }
}

#[test]
fn a_restricted_walkaround_ignores_everything_outside_the_cluster() {
  let (mut world, board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let settings = RoutingSettings::default();
  let context = AlgoContext::new(&rules, &settings);
  let path = two_pad_head();

  let mut walkaround = all_policies(&world, &settings);

  walkaround.restrict_to_cluster(&world, true, &board.pads[0..1]);
  assert_eq!(walkaround.restricted_set().len(), 1);

  let result = walkaround.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    let line = result.line(policy);

    // Nothing inside the restriction is left to walk around, so the
    // policy reports itself done even though the line is not clear.
    assert_eq!(result.status(policy), WalkaroundStatus::Done);
    assert!(
      clears_pad(line, 0),
      "the pad in the cluster was walked around"
    );
    assert!(!clears_pad(line, 1), "the pad outside it was ignored");
    assert!(!is_clear(&world, line));
  }

  // Turning the restriction off again clears both.
  walkaround.restrict_to_cluster(&world, false, &board.pads[0..1]);
  assert!(walkaround.restricted_set().is_empty());

  let unrestricted = walkaround.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    assert_eq!(unrestricted.status(policy), WalkaroundStatus::Done);
    assert!(is_clear(&world, unrestricted.line(policy)));
  }
}

#[test]
fn solids_only_walks_past_a_track() {
  let (mut world, _board) = build();
  let rules = FixedClearance::uniform(CLEARANCE);
  let settings = RoutingSettings::default();
  let context = AlgoContext::new(&rules, &settings);
  let path = track_crossing_head(0);

  assert!(
    !is_clear(&world, &path),
    "the straight head crosses the track"
  );

  let mut every_kind = all_policies(&world, &settings);
  let walked = every_kind.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    assert_eq!(walked.status(policy), WalkaroundStatus::Done);
    assert!(is_clear(&world, walked.line(policy)));
    assert_ne!(walked.line(policy).shape().points(), path.shape().points());
  }

  // With the mask narrowed to solids the track is not an obstacle at
  // all, so there is nothing to walk around and the head comes back as
  // it went in, still colliding.
  let mut solids = all_policies(&world, &settings);

  solids.set_solids_only(true);

  let ignored = solids.route(&mut world, &context, &path);

  for policy in WalkPolicy::ALL {
    assert_eq!(ignored.status(policy), WalkaroundStatus::Done);
    assert_eq!(ignored.line(policy).shape().points(), path.shape().points());
  }

  assert!(!is_clear(&world, ignored.line(WalkPolicy::Shortest)));
}

/// The whole set of scenarios is a pure function of the plain data, down
/// to the geometry of all three policies (`DESIGN.md` section 8).
#[test]
fn the_scenarios_answer_identically_twice() {
  fn run() -> Vec<(WalkaroundStatus, Vec<Vec2>)> {
    let (mut world, board) = build();
    let rules = FixedClearance::uniform(CLEARANCE);
    let settings = RoutingSettings::default();
    let context = AlgoContext::new(&rules, &settings);
    let mut answers = Vec::new();
    let record =
      |walkaround: &mut Walkaround,
       world: &mut World,
       path: &Line,
       answers: &mut Vec<(WalkaroundStatus, Vec<Vec2>)>| {
        let result = walkaround.route(world, &context, path);

        for policy in WalkPolicy::ALL {
          answers.push((
            result.status(policy),
            result.line(policy).shape().points().to_vec(),
          ));
        }
      };

    for path in [
      two_pad_head(),
      track_crossing_head(0),
      track_crossing_head(1),
      trapped_head(),
    ] {
      let mut walkaround = all_policies(&world, &settings);

      record(&mut walkaround, &mut world, &path, &mut answers);
    }

    // Once more with a restriction covering both layer 0 pads, which is
    // the path through the shortest policy's back off against the items
    // of an earlier round.
    let mut restricted = all_policies(&world, &settings);

    restricted.restrict_to_cluster(&world, true, &board.pads[0..2]);
    record(&mut restricted, &mut world, &two_pad_head(), &mut answers);

    answers
  }

  assert_eq!(run(), run());
}
