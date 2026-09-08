// SPDX-License-Identifier: GPL-3.0-or-later

//! Optimizer scenarios on a two layer board, through the public API
//! only.
//!
//! `doc/work/003-walkaround-router.md` asks for the optimizer to be
//! exercised against a real world rather than only in unit tests, so this
//! file builds one out of plain data the way `tests/world.rs` and
//! `tests/walkaround.rs` do and drives
//! [`pnsrouter::optimizer::Optimizer`] over it.
//!
//! Every expected shape below is hand computed from KiCad's algorithm and
//! written out in full, because the point of pinning them is to catch a
//! port that drifts, not to record whatever the code happens to do. Every
//! scenario that produces a line also asserts that the line is actually
//! clear, through `World::check_colliding_line`: an optimizer that
//! shortens a route into a pad is the failure mode worth catching.

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::collide::CollisionSearchOptions;
use pnsrouter::geometry::box2::Box2;
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::{Vec2, Vec2L};
use pnsrouter::item::{
  ItemBody, LayerRange, NetId, Segment, Solid, Via, ViaType,
};
use pnsrouter::line::Line;
use pnsrouter::node::World;
use pnsrouter::optimizer::{
  Candidate, Constraint, CostEstimator, EffortFlags, Optimizer,
  compute_breakouts, find_pad_or_via, rect_breakouts,
};
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::RoutingSettings;

/// The clearance every scenario routes to, in nanometres.
const CLEARANCE: i32 = 50000;

/// The width of the head and of the board's track.
const TRACK_WIDTH: i32 = 100000;

/// The copper radius of every pad.
const PAD_RADIUS: i32 = 100000;

/// How far a head's centreline has to stay from a pad's centre: the
/// pad's copper, the clearance and half the head's width.
const KEEP_OUT: i32 = PAD_RADIUS + CLEARANCE + TRACK_WIDTH / 2;

/// The net everything on the board is on.
const BOARD_NET: Option<NetId> = Some(NetId(1));

/// The net the head is on, so that nothing on the board exempts it.
const HEAD_NET: Option<NetId> = Some(NetId(2));

/// A pad of the board, as a host would hand it over: position and layer.
///
/// Pad 0 sits exactly on the diagonal that joins the ends of
/// [`STAIRCASE`], so it is the obstacle that stops the staircase from
/// collapsing on layer 0 and leaves it free to collapse on layer 1. Pad 1
/// sits inside the triangle between [`ELBOW`] and its diagonal shortcut,
/// which is what [`EffortFlags::KEEP_TOPOLOGY`] is there to notice, and
/// is far enough from both to collide with neither. Pad 2 only exists so
/// that layer 1 is not empty.
const PADS: [(i32, i32, i32); 3] = [
  (1500000, -1500000, 0),
  (3000000, -7000000, 0),
  (5000000, -5000000, 1),
];

/// The board's one existing trace: both endpoints and the layer. It runs
/// east of everything the scenarios touch.
const TRACK: (i32, i32, i32, i32, i32) = (6000000, 0, 6000000, -3000000, 0);

/// A staircase of three right angle corners.
///
/// The two middle points are chosen so that the span from the first point
/// to the fourth is an exact diagonal: the bypass across it is then a
/// single segment, which is what lets `mergeFull` land on a two segment
/// answer. It cannot reach one otherwise, because its step is clamped to
/// `SegmentCount() - 2` and it therefore never spans a whole line.
const STAIRCASE: [Vec2; 5] = [
  Vec2::new(0, 0),
  Vec2::new(1000000, 0),
  Vec2::new(1000000, -2000000),
  Vec2::new(2000000, -2000000),
  Vec2::new(2000000, -3000000),
];

/// A long east run, a short jog, a long diagonal run and a short east
/// run.
///
/// Segment 0 runs east and segment 2 runs north east, which is the obtuse
/// pair `mergeObtuse` looks for; their infinite lines cross at
/// `(1100000, 0)`, and both halves of that corner are obtuse as well, so
/// the jog between them collapses into it.
const JOG: [Vec2; 5] = [
  Vec2::new(0, 0),
  Vec2::new(1000000, 0),
  Vec2::new(1200000, -100000),
  Vec2::new(2000000, -900000),
  Vec2::new(3000000, -900000),
];

/// An east run, a north run and a short east run, well south of
/// everything else.
///
/// The bypass across the first two segments is the exact diagonal from
/// the first point to the third, and pad 1 sits inside the triangle that
/// bypass encloses.
const ELBOW: [Vec2; 4] = [
  Vec2::new(0, -6000000),
  Vec2::new(4000000, -6000000),
  Vec2::new(4000000, -10000000),
  Vec2::new(5000000, -10000000),
];

// -----------------------------------------------------------------
// The pad pass fixtures
//
// Everything below sits at a positive y, well away from the four merge
// scenarios above, which all live at y <= 0. The heads that use it are
// on `BOARD_NET` rather than on `HEAD_NET`, because both pad passes
// start from a joint lookup keyed on the line's own net: a head on a net
// nothing on the board belongs to has no pad at either end, which is
// what `the_pad_passes_leave_a_head_with_no_pads_alone` covers.
// -----------------------------------------------------------------

/// The net the obstacle that blocks a breakout is on, so that it is not
/// exempt from the head the way the pads are.
const BLOCKER_NET: Option<NetId> = Some(NetId(3));

/// The rectangular pad the unobstructed smart pads scenario leaves from:
/// the copper centre and the copper size, on layer 0.
const RECT_PAD: (Vec2, Vec2) =
  (Vec2::new(0, 5000000), Vec2::new(600000, 200000));

/// The same pad again for the obstructed scenario, three millimetres
/// east so that the blocker cannot reach the first one.
const BLOCKED_RECT_PAD: (Vec2, Vec2) =
  (Vec2::new(3000000, 5000000), Vec2::new(600000, 200000));

/// The obstacle that rules out the two cheapest exits of
/// [`BLOCKED_RECT_PAD`]: it sits exactly on the east breakout's diagonal
/// and 141421 nanometres from the south east one's, both inside the
/// 200000 nanometre keep out, while the exit that wins passes no closer
/// than 300000.
const BREAKOUT_BLOCKER: (Vec2, i32) = (Vec2::new(3700000, 5300000), 100000);

/// The round pad the second smart pads scenario leaves from, on layer 0.
const ROUND_PAD: (Vec2, i32) = (Vec2::new(0, 8000000), PAD_RADIUS);

/// The two pads the fanout scenario runs between, on layer 0. They are
/// close enough that the line between them is under ten track widths
/// long, which is what arms the cleanup.
const FANOUT_PADS: [(Vec2, i32); 2] = [
  (Vec2::new(0, 11000000), PAD_RADIUS),
  (Vec2::new(500000, 11300000), PAD_RADIUS),
];

/// The via the third smart pads scenario leaves from: position, copper
/// diameter and drill. It spans both layers.
const SMART_PADS_VIA: (Vec2, i32, i32) =
  (Vec2::new(0, 14000000), 2 * PAD_RADIUS, PAD_RADIUS);

/// Turn the plain data into a world.
fn build() -> World {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();

  for (centre, size) in [RECT_PAD, BLOCKED_RECT_PAD] {
    let origin = centre - Vec2::new(size.x / 2, size.y / 2);
    let body = ItemBody::Solid(Solid::new(Shape::rect(origin, size), centre));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(BOARD_NET);
    world.add_solid(root, item, None);
  }

  for (centre, radius) in
    [ROUND_PAD, FANOUT_PADS[0], FANOUT_PADS[1], BREAKOUT_BLOCKER]
  {
    let body =
      ItemBody::Solid(Solid::new(Shape::circle(centre, radius), centre));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(if (centre, radius) == BREAKOUT_BLOCKER {
      BLOCKER_NET
    } else {
      BOARD_NET
    });
    world.add_solid(root, item, None);
  }

  let (at, diameter, drill) = SMART_PADS_VIA;
  let body = ItemBody::Via(Via::new(at, diameter, drill, ViaType::Through));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::new(0, 1));
  item.set_net(BOARD_NET);
  world.add_via(root, item);

  for (x, y, layer) in PADS {
    let at = Vec2::new(x, y);
    let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(layer));
    item.set_net(BOARD_NET);
    world.add_solid(root, item, None);
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

  world
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

/// A head on the board's own net, which is what a trace leaving one of
/// the board's pads is. The pads it touches are then exempt from it, as
/// they are for any real route.
fn head_on_board_net(layer: i32, points: &[Vec2]) -> Line {
  let mut line = head_on(layer, points);

  line.set_net(BOARD_NET);
  line
}

/// The rule oracle every scenario queries through.
fn rules() -> FixedClearance {
  FixedClearance::uniform(CLEARANCE)
}

/// Whether a line is clear of everything the root holds.
fn is_clear(world: &World, line: &Line) -> bool {
  world
    .check_colliding_line(
      world.root(),
      line,
      &rules(),
      &CollisionSearchOptions::default(),
    )
    .is_none()
}

/// Run one optimizer over the root, with the caller setting whatever
/// constraints the scenario needs.
///
/// Returns KiCad's "did anything change" answer and the optimized line.
fn optimize(
  world: &World,
  settings: &RoutingSettings,
  effort: EffortFlags,
  line: &Line,
  setup: impl FnOnce(&mut Optimizer),
) -> (bool, Line) {
  let rules = rules();
  let context = AlgoContext::new(&rules, settings);
  let mut optimizer = Optimizer::new(world.root());
  let mut result = Line::new();

  optimizer.set_effort_level(effort);
  setup(&mut optimizer);

  let changed = optimizer.optimize(world, &context, line, &mut result, None);

  (changed, result)
}

/// The points of a line, for comparing against a hand computed answer.
fn points(line: &Line) -> Vec<Vec2> {
  line.shape().points().to_vec()
}

/// Whether a point lies on a line's geometry, to the tolerance the
/// preserve vertex constraint uses.
fn lies_on(line: &Line, point: Vec2) -> bool {
  (0..line.segment_count())
    .any(|index| line.segment(index).squared_distance_to_point(point) <= 1)
}

/// Whether a line keeps its distance from a pad's centre, which is what
/// [`is_clear`] says geometrically for a circular pad.
fn clears_pad(line: &Line, pad: usize) -> bool {
  let (x, y, _) = PADS[pad];
  let centre = Vec2::new(x, y);

  (0..line.segment_count())
    .all(|index| line.segment(index).distance_to_point(centre) >= KEEP_OUT)
}

/// The layer 1 staircase, which nothing is in the way of.
#[test]
fn a_staircase_merges_to_two_segments_when_nothing_is_in_the_way() {
  let world = build();
  let settings = RoutingSettings::default();
  let line = head_on(1, &STAIRCASE);

  assert!(is_clear(&world, &line));

  let (changed, optimized) = optimize(
    &world,
    &settings,
    EffortFlags::MERGE_SEGMENTS,
    &line,
    |_| {},
  );

  // The first pass spans segments 0 and 2, whose outer endpoints are an
  // exact diagonal apart, so the bypass is one segment and the three
  // right angles become one 45 degree corner. The second pass has only
  // two segments left and its step clamps to zero.
  assert!(changed);
  assert_eq!(
    points(&optimized),
    vec![
      Vec2::new(0, 0),
      Vec2::new(2000000, -2000000),
      Vec2::new(2000000, -3000000),
    ]
  );
  assert_eq!(optimized.segment_count(), 2);
  assert!(is_clear(&world, &optimized));
  // The optimizer never moves an endpoint.
  assert_eq!(optimized.point(0), line.point(0));
  assert_eq!(optimized.last_point(), line.last_point());
}

/// The same staircase on the layer pad 0 is on. The diagonal shortcut
/// runs straight through the pad, so the optimizer has to settle for
/// less.
#[test]
fn a_staircase_does_not_merge_into_an_obstacle() {
  let world = build();
  let settings = RoutingSettings::default();
  let line = head_on(0, &STAIRCASE);

  assert!(is_clear(&world, &line), "the staircase itself is clear");

  // The answer the layer 1 scenario gets is not available here.
  let shortcut = head_on(
    0,
    &[
      Vec2::new(0, 0),
      Vec2::new(2000000, -2000000),
      Vec2::new(2000000, -3000000),
    ],
  );

  assert!(!is_clear(&world, &shortcut), "the shortcut hits pad 0");

  let (changed, optimized) = optimize(
    &world,
    &settings,
    EffortFlags::MERGE_SEGMENTS,
    &line,
    |_| {},
  );

  // Both postures of the long span collide, so the pass falls back to
  // the span from segment 1 to segment 3 and takes the cheaper of its
  // two bypasses.
  assert!(changed);
  assert_eq!(
    points(&optimized),
    vec![
      Vec2::new(0, 0),
      Vec2::new(1000000, 0),
      Vec2::new(2000000, -1000000),
      Vec2::new(2000000, -3000000),
    ]
  );
  assert!(is_clear(&world, &optimized));
  assert!(clears_pad(&optimized, 0));
  assert_eq!(optimized.point(0), line.point(0));
  assert_eq!(optimized.last_point(), line.last_point());
}

#[test]
fn merge_obtuse_removes_a_short_jog() {
  let world = build();
  let settings = RoutingSettings::default();
  let line = head_on(1, &JOG);

  assert!(is_clear(&world, &line));

  let (changed, optimized) =
    optimize(&world, &settings, EffortFlags::MERGE_OBTUSE, &line, |_| {});

  // Segments 0 and 2 are 45 degrees apart, their infinite lines cross at
  // (1100000, 0), and both halves of that corner are obtuse too, so the
  // two points between them collapse into it.
  assert!(changed);
  assert_eq!(
    points(&optimized),
    vec![
      Vec2::new(0, 0),
      Vec2::new(1100000, 0),
      Vec2::new(2000000, -900000),
      Vec2::new(3000000, -900000),
    ]
  );
  assert!(is_clear(&world, &optimized));
  assert_eq!(optimized.point(0), line.point(0));
  assert_eq!(optimized.last_point(), line.last_point());
}

#[test]
fn merge_colinear_joins_two_collinear_segments() {
  let world = build();
  let settings = RoutingSettings::default();
  let line = head_on(
    1,
    &[
      Vec2::new(0, 0),
      Vec2::new(1000000, 0),
      Vec2::new(2000000, 0),
      Vec2::new(2000000, -1000000),
    ],
  );

  let (changed, optimized) = optimize(
    &world,
    &settings,
    EffortFlags::MERGE_COLINEAR,
    &line,
    |_| {},
  );

  assert!(changed);
  assert_eq!(
    points(&optimized),
    vec![
      Vec2::new(0, 0),
      Vec2::new(2000000, 0),
      Vec2::new(2000000, -1000000),
    ]
  );
  assert!(is_clear(&world, &optimized));
}

/// The anchor the dragger would pass survives the optimization, and the
/// unconstrained answer does not keep it.
///
/// KiCad's constraint is a distance test and not a vertex test
/// (`pcbnew/router/pns_optimizer.cpp:247`), so what it guarantees is that
/// the point still lies **on** the result, not that it is still a corner
/// of it. The assertions say exactly that.
#[test]
fn the_preserve_vertex_constraint_keeps_the_anchor_on_the_line() {
  let world = build();
  let settings = RoutingSettings::default();
  let line = head_on(1, &STAIRCASE);
  let anchor = Vec2::new(1000000, -2000000);

  let (_, free) = optimize(
    &world,
    &settings,
    EffortFlags::MERGE_SEGMENTS,
    &line,
    |_| {},
  );

  assert!(!lies_on(&free, anchor), "the free answer drops the anchor");

  let (changed, kept) = optimize(
    &world,
    &settings,
    EffortFlags::MERGE_SEGMENTS,
    &line,
    |optimizer| optimizer.set_preserve_vertex(anchor),
  );

  assert!(changed);
  assert!(lies_on(&kept, anchor));
  assert_eq!(
    points(&kept),
    vec![
      Vec2::new(0, 0),
      Vec2::new(0, -1000000),
      Vec2::new(2000000, -3000000),
    ]
  );
  assert!(is_clear(&world, &kept));
  assert_eq!(kept.point(0), line.point(0));
  assert_eq!(kept.last_point(), line.last_point());
}

/// A box that holds only the tail of the staircase refuses every span,
/// so the corners outside it survive untouched.
///
/// The `strict` flag is passed both ways and makes no difference, which
/// is KiCad's behaviour: `AREA_CONSTRAINT`'s constructor takes the flag
/// and does not store it (`pcbnew/router/pns_optimizer.h:269`).
#[test]
fn the_restrict_area_constraint_keeps_a_corner_outside_the_box() {
  let world = build();
  let settings = RoutingSettings::default();
  let line = head_on(1, &STAIRCASE);
  let area = Box2::from_corners(
    Vec2L::new(1500000, -3500000),
    Vec2L::new(2500000, -1500000),
  );

  for strict in [false, true] {
    let (changed, optimized) = optimize(
      &world,
      &settings,
      EffortFlags::MERGE_SEGMENTS,
      &line,
      |optimizer| optimizer.set_restrict_area(area, strict),
    );

    assert!(!changed, "strict = {strict}");
    assert_eq!(points(&optimized), STAIRCASE.to_vec(), "strict = {strict}");
    assert!(is_clear(&world, &optimized));
  }
}

/// The topology constraint on its own, with the polygon and the pad both
/// hand placed.
#[test]
fn the_keep_topology_constraint_refuses_a_bypass_that_hops_a_pad() {
  let world = build();
  let root = world.root();
  let path = LineChain::from_slice(&ELBOW[0..3], false);
  let replacement = LineChain::from_slice(&[ELBOW[0], ELBOW[2]], false);
  let on_layer_zero = head_on(0, &ELBOW[0..3]);

  fn span<'a>(
    line: &'a Line,
    path: &'a LineChain,
    replacement: &'a LineChain,
  ) -> Candidate<'a> {
    Candidate {
      vertex1: 0,
      vertex2: 2,
      origin_line: line,
      current_path: path,
      replacement,
    }
  }

  // Pad 1 is inside the triangle the bypass would enclose.
  assert!(!Constraint::KeepTopology.check(
    &world,
    root,
    &span(&on_layer_zero, &path, &replacement)
  ));

  // The same geometry on layer 1 sees no solid at all, because
  // `QueryJoints` is filtered by the line's layers.
  let on_layer_one = head_on(1, &ELBOW[0..3]);

  assert!(Constraint::KeepTopology.check(
    &world,
    root,
    &span(&on_layer_one, &path, &replacement)
  ));

  // And a head on the pad's own net is allowed to hop it, because the
  // check skips joints of the line's net.
  let mut same_net = head_on(0, &ELBOW[0..3]);

  same_net.set_net(BOARD_NET);
  assert!(Constraint::KeepTopology.check(
    &world,
    root,
    &span(&same_net, &path, &replacement)
  ));
}

/// End to end: the flag changes what the optimizer settles for, and the
/// corner that keeps the pad on the outside of the route survives.
#[test]
fn keep_topology_keeps_the_joint_structure() {
  let world = build();
  let settings = RoutingSettings::default();
  let line = head_on(0, &ELBOW);

  assert!(is_clear(&world, &line));

  let (free_changed, free) = optimize(
    &world,
    &settings,
    EffortFlags::MERGE_SEGMENTS,
    &line,
    |_| {},
  );

  // Without the constraint the elbow collapses onto its diagonal, which
  // puts pad 1 on the other side of the route.
  assert!(free_changed);
  assert_eq!(
    points(&free),
    vec![ELBOW[0], Vec2::new(4000000, -10000000), ELBOW[3]]
  );
  assert!(is_clear(&world, &free), "the diagonal clears pad 1");

  let (kept_changed, kept) = optimize(
    &world,
    &settings,
    EffortFlags::MERGE_SEGMENTS | EffortFlags::KEEP_TOPOLOGY,
    &line,
    |_| {},
  );

  // With it, the long span is refused and only the tail is tidied, so
  // the corner at the elbow stays where it was.
  // The shape changed but the segment count did not, and KiCad's answer
  // is literally `current_path.SegmentCount() < segs_pre`
  // (`pcbnew/router/pns_optimizer.cpp:623`), so this reports "unchanged"
  // over a line it did rewrite. That is worth pinning: a caller that
  // skips writing the result back when the answer is false, as
  // `SHOVE::runOptimizer` does (`pcbnew/router/pns_shove.cpp:2121`),
  // silently drops this improvement.
  assert!(!kept_changed);
  assert_eq!(
    points(&kept),
    vec![
      ELBOW[0],
      Vec2::new(4000000, -6000000),
      Vec2::new(5000000, -7000000),
      Vec2::new(5000000, -10000000),
    ]
  );
  assert_ne!(points(&kept), points(&free));
  assert!(is_clear(&world, &kept));
  assert!(clears_pad(&kept, 1));
}

/// The static convenience overload optimizes in place and answers the
/// same as the long form.
#[test]
fn the_static_overload_optimizes_in_place() {
  let world = build();
  let settings = RoutingSettings::default();
  let rules = rules();
  let context = AlgoContext::new(&rules, &settings);
  let mut line = head_on(1, &STAIRCASE);

  let (_, expected) = optimize(
    &world,
    &settings,
    EffortFlags::MERGE_SEGMENTS,
    &line,
    |_| {},
  );

  let changed = Optimizer::optimize_line(
    &world,
    &context,
    world.root(),
    &mut line,
    EffortFlags::MERGE_SEGMENTS,
    Vec2::new(0, 0),
  );

  assert!(changed);
  assert_eq!(points(&line), points(&expected));
  assert!(is_clear(&world, &line));
}

/// A head whose ends carry no pad passes through both pad passes
/// untouched, and the two inert flags change nothing either.
///
/// The staircase is on the head's own net, which no joint on the board
/// belongs to, so [`find_pad_or_via`] answers nothing at both ends. That
/// is the common case for a head in flight and it has to be a no
/// operation on the geometry.
#[test]
fn the_pad_passes_leave_a_head_with_no_pads_alone() {
  let world = build();
  let settings = RoutingSettings::default();
  let line = head_on(1, &STAIRCASE);
  let effort = EffortFlags::MERGE_SEGMENTS
    | EffortFlags::SMART_PADS
    | EffortFlags::FANOUT_CLEANUP
    | EffortFlags::LIMIT_CORNER_COUNT
    | EffortFlags::RESTRICT_VERTEX_RANGE;

  let (changed, optimized) = optimize(&world, &settings, effort, &line, |_| {});
  let (_, plain) = optimize(
    &world,
    &settings,
    EffortFlags::MERGE_SEGMENTS,
    &line,
    |_| {},
  );

  assert!(changed);
  assert_eq!(points(&optimized), points(&plain));
  assert!(is_clear(&world, &optimized));

  // And `SMART_PADS` alone reports "changed" over a line it did not
  // touch, because `runSmartPads` returns true unconditionally
  // (`pcbnew/router/pns_optimizer.cpp:1254`).
  let (smart_changed, untouched) =
    optimize(&world, &settings, EffortFlags::SMART_PADS, &line, |_| {});

  assert!(smart_changed);
  assert_eq!(points(&untouched), STAIRCASE.to_vec());
}

#[test]
fn the_scenarios_answer_identically_twice() {
  let settings = RoutingSettings::default();
  let effort = EffortFlags::MERGE_SEGMENTS
    | EffortFlags::MERGE_OBTUSE
    | EffortFlags::MERGE_COLINEAR;

  let run = || {
    let world = build();
    let mut answers = Vec::new();

    for (layer, path) in [
      (0, STAIRCASE.as_slice()),
      (1, STAIRCASE.as_slice()),
      (1, JOG.as_slice()),
      (0, ELBOW.as_slice()),
    ] {
      let line = head_on(layer, path);
      let (changed, optimized) =
        optimize(&world, &settings, effort, &line, |_| {});

      assert!(is_clear(&world, &optimized));
      answers.push((changed, points(&optimized)));
    }

    answers
  };

  assert_eq!(run(), run());
}

// ---------------------------------------------------------------------
// The pad passes
// ---------------------------------------------------------------------

/// A right angle exit from the oblong pad, which is what the smart pads
/// pass exists to remove.
///
/// The line leaves the pad centre straight along the pad's **short**
/// axis and then turns 90 degrees, which costs `COST_RIGHT`, 30.
fn awkward_exit_from(centre: Vec2) -> [Vec2; 3] {
  [
    centre,
    centre + Vec2::new(0, 600000),
    centre + Vec2::new(1000000, 600000),
  ]
}

/// The eight exits of the unobstructed rectangular pad, hand computed
/// from `pcbnew/router/pns_optimizer.cpp:1003` and `:1013` for a 600000
/// by 200000 pad and a 100000 wide track: `size / 2 + width` along each
/// axis, then `d_offset = 200000` out along the long axis followed by a
/// 45 degree leg of `width + min(size) / 2 = 200000`.
fn rect_pad_breakout_ends(centre: Vec2) -> Vec<Vec2> {
  [
    Vec2::new(400000, 0),
    Vec2::new(-400000, 0),
    Vec2::new(0, 200000),
    Vec2::new(0, -200000),
    Vec2::new(400000, 200000),
    Vec2::new(400000, -200000),
    Vec2::new(-400000, 200000),
    Vec2::new(-400000, -200000),
  ]
  .into_iter()
  .map(|offset| centre + offset)
  .collect()
}

/// The breakout list the world's own rectangular pad offers, through the
/// public entry point.
fn breakouts_of(world: &World, at: Vec2, layer: i32) -> Vec<Vec2> {
  let id = find_pad_or_via(world, world.root(), layer, BOARD_NET, at)
    .expect("the fixture puts a pad here");
  let item = world.item(id).expect("the joint's link is a live item");

  compute_breakouts(TRACK_WIDTH, item, true)
    .iter()
    .map(|chain| chain.last_point().expect("a breakout has points"))
    .collect()
}

/// What a chain of points costs in corners, for comparing candidates.
fn cost_of(points: &[Vec2]) -> i32 {
  CostEstimator::corner_cost_of_chain(&LineChain::from_slice(points, false))
}

/// The exits the fixture's rectangular pad offers are the hand computed
/// eight, and the joint lookup that finds the pad answers the pad.
#[test]
fn a_rectangular_pad_offers_the_hand_computed_breakouts() {
  let world = build();
  let (centre, _) = RECT_PAD;

  assert_eq!(
    breakouts_of(&world, centre, 0),
    rect_pad_breakout_ends(centre)
  );

  // The pad is on layer 0 only, so the same lookup on layer 1 finds
  // nothing and the pass would decline the line.
  assert_eq!(
    find_pad_or_via(&world, world.root(), 1, BOARD_NET, centre),
    None
  );
}

/// The unobstructed rectangular pad: the right angle exit is replaced by
/// the cheapest collision free candidate, which leaves along the pad's
/// long axis.
///
/// The winner is breakout 0, the east exit, joined to the line's last
/// point by a single 45 degree segment: cost 10 against the 30 the user
/// drew. Every other candidate that survives the forbidden angle filter
/// costs 20 or more, and the two nearest ones are checked below.
#[test]
fn a_rectangular_pad_gets_the_cheapest_collision_free_breakout() {
  let world = build();
  let settings = RoutingSettings::default();
  let (centre, _) = RECT_PAD;
  let path = awkward_exit_from(centre);
  let line = head_on_board_net(0, &path);

  assert!(is_clear(&world, &line));
  assert_eq!(cost_of(&path), 30);

  let (changed, optimized) =
    optimize(&world, &settings, EffortFlags::SMART_PADS, &line, |_| {});

  let winner = vec![
    centre,
    centre + Vec2::new(400000, 0),
    centre + Vec2::new(1000000, 600000),
  ];

  assert!(changed);
  assert_eq!(points(&optimized), winner);
  assert_eq!(cost_of(&winner), 10);
  assert!(is_clear(&world, &optimized));

  // The winner starts on the pad's own east breakout.
  assert_eq!(optimized.point(1), rect_pad_breakout_ends(centre)[0]);
  // And the pass never moves either end of the line.
  assert_eq!(optimized.point(0), line.point(0));
  assert_eq!(optimized.last_point(), line.last_point());

  // The two runner up candidates are clear as well, so the pass really
  // did pick on cost and not on availability. They come from breakout 2
  // (the short axis exit) and breakout 4 (the south east diagonal).
  for runner_up in [
    vec![
      centre,
      centre + Vec2::new(0, 200000),
      centre + Vec2::new(400000, 600000),
      centre + Vec2::new(1000000, 600000),
    ],
    vec![
      centre,
      centre + Vec2::new(200000, 0),
      centre + Vec2::new(800000, 600000),
      centre + Vec2::new(1000000, 600000),
    ],
  ] {
    assert_eq!(cost_of(&runner_up), 20);
    assert!(is_clear(&world, &head_on_board_net(0, &runner_up)));
  }
}

/// The same pad with an obstacle across its two cheapest exits: the pass
/// settles for the short axis exit instead of taking the blocked one.
#[test]
fn a_blocked_breakout_is_not_chosen() {
  let world = build();
  let settings = RoutingSettings::default();
  let (centre, _) = BLOCKED_RECT_PAD;
  let path = awkward_exit_from(centre);
  let line = head_on_board_net(0, &path);

  assert!(is_clear(&world, &line), "the awkward exit itself is clear");

  // What won on the unobstructed pad, and the diagonal behind it, both
  // run into the blocker here.
  let east_exit = vec![
    centre,
    centre + Vec2::new(400000, 0),
    centre + Vec2::new(1000000, 600000),
  ];
  let diagonal_exit = vec![
    centre,
    centre + Vec2::new(200000, 0),
    centre + Vec2::new(800000, 600000),
    centre + Vec2::new(1000000, 600000),
  ];

  assert_eq!(cost_of(&east_exit), 10);
  assert!(!is_clear(&world, &head_on_board_net(0, &east_exit)));
  assert!(!is_clear(&world, &head_on_board_net(0, &diagonal_exit)));

  let (changed, optimized) =
    optimize(&world, &settings, EffortFlags::SMART_PADS, &line, |_| {});

  let winner = vec![
    centre,
    centre + Vec2::new(0, 200000),
    centre + Vec2::new(400000, 600000),
    centre + Vec2::new(1000000, 600000),
  ];

  assert!(changed);
  assert_eq!(points(&optimized), winner);
  assert_eq!(cost_of(&winner), 20);
  assert!(is_clear(&world, &optimized));
  assert_eq!(optimized.point(1), centre + Vec2::new(0, 200000));
  assert_eq!(optimized.point(0), line.point(0));
  assert_eq!(optimized.last_point(), line.last_point());
}

/// The round pad. Its eight exits are the hand computed rays of
/// `radius * sqrt(2)`, and the winner is the east one followed by a
/// straight run and a 45 degree turn, which the candidate's own
/// `Simplify2` folds into two segments.
#[test]
fn a_round_pad_gets_a_breakout_too() {
  let world = build();
  let settings = RoutingSettings::default();
  let (centre, _) = ROUND_PAD;
  let path = awkward_exit_from(centre);
  let line = head_on_board_net(0, &path);

  assert!(is_clear(&world, &line));
  assert_eq!(
    breakouts_of(&world, centre, 0),
    vec![
      centre + Vec2::new(141421, 0),
      centre + Vec2::new(100000, 100000),
      centre + Vec2::new(0, 141421),
      centre + Vec2::new(-100000, 100000),
      centre + Vec2::new(-141421, 0),
      centre + Vec2::new(-100000, -100000),
      centre + Vec2::new(0, -141421),
      centre + Vec2::new(100000, -100000),
    ]
  );

  let (changed, optimized) =
    optimize(&world, &settings, EffortFlags::SMART_PADS, &line, |_| {});

  // Breakout 0 runs east to (141421, 0); the straight first connection
  // to the far point continues east to (400000, 0) and then turns 45
  // degrees, so the breakout's own endpoint is collinear and `Simplify2`
  // drops it.
  let winner = vec![
    centre,
    centre + Vec2::new(400000, 0),
    centre + Vec2::new(1000000, 600000),
  ];

  assert!(changed);
  assert_eq!(points(&optimized), winner);
  assert_eq!(cost_of(&winner), 10);
  assert!(is_clear(&world, &optimized));
  assert_eq!(optimized.point(0), line.point(0));
  assert_eq!(optimized.last_point(), line.last_point());
}

/// A via offers the same eight exits as a round pad of its copper
/// radius, and the pass refuses to use any of them.
///
/// `smartPadsSingle` returns before it looks at the breakouts when the
/// item is a via (`pcbnew/router/pns_optimizer.cpp:1128`), with the
/// reason written out there: a via is round, so no exit is better than
/// another, and rewriting one would only destroy the posture the placer
/// chose. The pass still reports "changed", which is the unconditional
/// `return true` at `:1254`.
#[test]
fn a_via_offers_breakouts_and_the_pass_refuses_them() {
  let world = build();
  let settings = RoutingSettings::default();
  let (centre, diameter, _) = SMART_PADS_VIA;
  let path = awkward_exit_from(centre);
  let line = head_on_board_net(0, &path);
  let radius = diameter / 2;

  assert!(is_clear(&world, &line));
  assert_eq!(
    breakouts_of(&world, centre, 0),
    vec![
      centre + Vec2::new(141421, 0),
      centre + Vec2::new(radius, radius),
      centre + Vec2::new(0, 141421),
      centre + Vec2::new(-radius, radius),
      centre + Vec2::new(-141421, 0),
      centre + Vec2::new(-radius, -radius),
      centre + Vec2::new(0, -141421),
      centre + Vec2::new(radius, -radius),
    ]
  );

  let (changed, optimized) =
    optimize(&world, &settings, EffortFlags::SMART_PADS, &line, |_| {});

  assert!(changed);
  assert_eq!(points(&optimized), path.to_vec());
  assert!(is_clear(&world, &optimized));
}

/// No caller in KiCad's tree ever passes `aPermitDiagonal = false`:
/// `smartPadsSingle` is the only caller of `computeBreakouts` and it
/// hard codes `true` (`pcbnew/router/pns_optimizer.cpp:1131`). The
/// parameter is still ported and still works, so a host that wants
/// orthogonal only exits has it.
#[test]
fn the_diagonal_breakouts_are_dropped_when_they_are_not_permitted() {
  let world = build();
  let (centre, size) = RECT_PAD;
  let id = find_pad_or_via(&world, world.root(), 0, BOARD_NET, centre)
    .expect("the fixture puts a pad here");
  let item = world.item(id).expect("the joint's link is a live item");
  let origin = centre - Vec2::new(size.x / 2, size.y / 2);
  let shape = Shape::rect(origin, size);

  assert_eq!(compute_breakouts(TRACK_WIDTH, item, true).len(), 8);

  let orthogonal = rect_breakouts(TRACK_WIDTH, &shape, false);

  assert_eq!(orthogonal.len(), 4);
  assert_eq!(
    orthogonal
      .iter()
      .map(|chain| chain.last_point().expect("a breakout has points"))
      .collect::<Vec<_>>(),
    rect_pad_breakout_ends(centre)[0..4].to_vec()
  );

  // Every orthogonal exit is a single segment; the diagonals are the
  // only two segment ones.
  for breakout in &orthogonal {
    assert_eq!(breakout.segment_count(), 1);
  }
}

/// Two pads a short hop apart, joined by a right angle: the cleanup
/// redraws the whole thing as the straight leg first posture.
#[test]
fn fanout_cleanup_straightens_a_two_segment_fanout() {
  let world = build();
  let settings = RoutingSettings::default();
  let (start, _) = FANOUT_PADS[0];
  let (end, _) = FANOUT_PADS[1];
  let path = [start, Vec2::new(start.x, end.y), end];
  let line = head_on_board_net(0, &path);

  assert!(is_clear(&world, &line));
  assert_eq!(cost_of(&path), 30);
  // The guard is `length < 10 * width`, so the fixture has to be short.
  assert!(line.shape().length() < i64::from(TRACK_WIDTH) * 10);

  let (changed, optimized) = optimize(
    &world,
    &settings,
    EffortFlags::FANOUT_CLEANUP,
    &line,
    |_| {},
  );

  // `BuildInitialTrace` with the straight leg first: 500000 across and
  // 300000 down means 200000 of straight run and then the diagonal.
  let winner = vec![start, Vec2::new(start.x + 200000, start.y), end];

  assert!(changed);
  assert_eq!(points(&optimized), winner);
  assert_eq!(cost_of(&winner), 10);
  assert!(is_clear(&world, &optimized));
  assert_eq!(optimized.point(0), line.point(0));
  assert_eq!(optimized.last_point(), line.last_point());
}

/// The cleanup declines a line that is too long, and one whose start is
/// not on a pad at all.
#[test]
fn fanout_cleanup_declines_what_it_is_not_for() {
  let world = build();
  let settings = RoutingSettings::default();
  let (start, _) = FANOUT_PADS[0];
  let (end, _) = FANOUT_PADS[1];

  // A route between the same two pads that wanders far enough to break
  // the `10 * width` budget is left alone.
  let long_way = [
    start,
    Vec2::new(start.x, start.y + 900000),
    Vec2::new(end.x, start.y + 900000),
    end,
  ];
  let wandering = head_on_board_net(0, &long_way);

  assert!(wandering.shape().length() >= i64::from(TRACK_WIDTH) * 10);

  let (changed, untouched) = optimize(
    &world,
    &settings,
    EffortFlags::FANOUT_CLEANUP,
    &wandering,
    |_| {},
  );

  assert!(!changed);
  assert_eq!(points(&untouched), long_way.to_vec());

  // And a line whose far end is on a pad but whose near end is in open
  // copper is refused at `pcbnew/router/pns_optimizer.cpp:1288`, even
  // though the reverse case falls back to `EndsWithVia`.
  let backwards =
    [Vec2::new(300000, 11100000), Vec2::new(end.x, 11100000), end];
  let from_nowhere = head_on_board_net(0, &backwards);

  let (changed, untouched) = optimize(
    &world,
    &settings,
    EffortFlags::FANOUT_CLEANUP,
    &from_nowhere,
    |_| {},
  );

  assert!(!changed);
  assert_eq!(points(&untouched), backwards.to_vec());
}

/// Both pad passes at once over every pad fixture, twice, with the same
/// answers and no clearance violation either time.
#[test]
fn the_pad_scenarios_answer_identically_twice() {
  let settings = RoutingSettings::default();
  let effort = EffortFlags::MERGE_SEGMENTS
    | EffortFlags::MERGE_OBTUSE
    | EffortFlags::MERGE_COLINEAR
    | EffortFlags::SMART_PADS
    | EffortFlags::FANOUT_CLEANUP;

  let run = || {
    let world = build();
    let mut answers = Vec::new();
    let (start, _) = FANOUT_PADS[0];
    let (end, _) = FANOUT_PADS[1];

    for path in [
      awkward_exit_from(RECT_PAD.0).to_vec(),
      awkward_exit_from(BLOCKED_RECT_PAD.0).to_vec(),
      awkward_exit_from(ROUND_PAD.0).to_vec(),
      awkward_exit_from(SMART_PADS_VIA.0).to_vec(),
      vec![start, Vec2::new(start.x, end.y), end],
    ] {
      let line = head_on_board_net(0, &path);
      let (changed, optimized) =
        optimize(&world, &settings, effort, &line, |_| {});

      assert!(is_clear(&world, &optimized));
      assert_eq!(optimized.point(0), line.point(0));
      assert_eq!(optimized.last_point(), line.last_point());
      answers.push((changed, points(&optimized)));
    }

    answers
  };

  assert_eq!(run(), run());
}
