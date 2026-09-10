// SPDX-License-Identifier: GPL-3.0-or-later

//! Multi drag scenarios on a two layer board, through the public API only.
//!
//! `doc/work/009-dragging.md` and note
//! `doc/reference/kicad/06-dragger.md` section 5 ask for the multi
//! dragger to be exercised over a board with real stored traces rather
//! than only in unit tests. The board is built out of plain data the way
//! `tests/dragger.rs` builds its own, because `tests/support/` is the
//! KiCad fixture reader and nothing else.
//!
//! All three routing modes are here, plus a corner mode drag, a bundle
//! with unequal spacing and a determinism check. The facade side, which
//! is `Router::start_dragging` picking the multi dragger from the shape
//! of the item set, is in `tests/router.rs`.

#![forbid(unsafe_code)]

use pnsrouter::algo_base::AlgoContext;
use pnsrouter::dragger::DragMode;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{ItemBody, ItemId, LayerRange, NetId, Segment, Solid};
use pnsrouter::multi_dragger::MultiDragger;
use pnsrouter::node::{NodeId, World};
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RouterMode, RoutingSettings};

/// The clearance every scenario drags to, in nanometres.
const CLEARANCE: i32 = 100_000;

/// The width of every trace in the bundle.
const TRACK_WIDTH: i32 = 200_000;

/// The copper radius of the pad a walkaround has to bend around.
const PAD_RADIUS: i32 = 400_000;

/// Where every trace of a bundle starts.
const LEFT: i32 = 0;

/// Where the middle segment of every trace starts.
const MIDDLE_LEFT: i32 = 2_000_000;

/// Where the middle segment of every trace ends.
const MIDDLE_RIGHT: i32 = 18_000_000;

/// Where every trace ends.
const RIGHT: i32 = 20_000_000;

/// The x the cursor grabs a bundle at, in the middle of its span so that
/// the 45 degree legs a drag builds leave the leader segment under the
/// cursor.
const GRAB_X: i32 = 10_000_000;

/// A bundle of parallel traces, by handle.
struct Bundle {
  /// The three segments of each trace, in trace order.
  traces: Vec<[ItemId; 3]>,
  /// The y each trace started at, in the same order.
  offsets: Vec<i32>,
}

impl Bundle {
  /// The middle segment of each trace, which is what a scenario selects.
  fn middle_segments(&self) -> Vec<ItemId> {
    self.traces.iter().map(|trace| trace[1]).collect()
  }
}

/// The net of the trace at index `index` of a bundle.
fn net_of(index: usize) -> Option<NetId> {
  #[expect(
    clippy::cast_possible_truncation,
    reason = "a scenario never builds more than a handful of traces"
  )]
  Some(NetId(index as u32 + 1))
}

/// The net nothing in a bundle is allowed to touch.
const OBSTACLE_NET: Option<NetId> = Some(NetId(90));

/// Add one three segment trace at a y offset and answer its handles.
fn add_trace(world: &mut World, y: i32, net: Option<NetId>) -> [ItemId; 3] {
  let root = world.root();
  let points = [
    Vec2::new(LEFT, y),
    Vec2::new(MIDDLE_LEFT, y),
    Vec2::new(MIDDLE_RIGHT, y),
    Vec2::new(RIGHT, y),
  ];
  let mut ids = Vec::new();

  for pair in points.windows(2) {
    let body =
      ItemBody::Segment(Segment::new(Seg::new(pair[0], pair[1]), TRACK_WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);

    ids.push(
      world
        .add_segment(root, item, false)
        .expect("a board track is neither degenerate nor redundant"),
    );
  }

  [ids[0], ids[1], ids[2]]
}

/// A round pad on layer 0, on the obstacle net.
fn add_pad(world: &mut World, at: Vec2) -> ItemId {
  let root = world.root();
  let body = ItemBody::Solid(Solid::new(Shape::circle(at, PAD_RADIUS), at));
  let mut item = world.make_item(body);

  item.set_layers_and_flash_all(LayerRange::single(0));
  item.set_net(OBSTACLE_NET);

  world.add_solid(root, item, None)
}

/// A board holding one parallel trace per offset, each on its own net.
fn build(offsets: &[i32]) -> (World, Bundle) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let mut traces = Vec::new();

  for (index, y) in offsets.iter().enumerate() {
    traces.push(add_trace(&mut world, *y, net_of(index)));
  }

  (
    world,
    Bundle {
      traces,
      offsets: offsets.to_vec(),
    },
  )
}

/// Two traces a millimetre and a half apart, the default bundle.
fn build_pair() -> (World, Bundle) {
  build(&[0, 1_500_000])
}

/// The rule oracle every scenario uses.
fn rules() -> FixedClearance {
  FixedClearance::uniform(CLEARANCE)
}

/// Settings in one routing mode, with everything else at KiCad's
/// defaults, which includes `smooth_dragged_segments`.
fn settings_for(mode: RouterMode) -> RoutingSettings {
  RoutingSettings {
    mode,
    ..RoutingSettings::default()
  }
}

/// The segments of a node's delta, as geometry, in the order
/// `World::get_updated_items` answers.
fn delta_segments(world: &World, node: NodeId) -> (Vec<Seg>, Vec<Seg>) {
  let (added, removed) = world.get_updated_items(node);
  let seg_of = |ids: Vec<ItemId>| -> Vec<Seg> {
    ids
      .into_iter()
      .filter_map(|id| match world.item(id)?.body() {
        ItemBody::Segment(body) => Some(body.seg()),
        _ => None,
      })
      .collect()
  };

  (seg_of(added), seg_of(removed))
}

/// The net and the geometry of each leader segment the drag handed back.
///
/// `MULTI_DRAGGER::GetLastCommittedLeaderSegments`
/// (`pcbnew/router/pns_multi_dragger.h:112`) is how a host puts the
/// user's selection back; reading it is also the tidiest way for a test
/// to say "this is where each trace's grabbed segment ended up".
fn leader_segments(
  world: &World,
  dragger: &MultiDragger,
) -> Vec<(Option<NetId>, Seg)> {
  dragger
    .last_committed_leader_segments()
    .iter()
    .filter_map(|id| {
      let item = world.item(*id)?;
      let ItemBody::Segment(body) = item.body() else {
        return None;
      };

      Some((item.net(), body.seg()))
    })
    .collect()
}

/// The y a horizontal leader segment sits at, per trace index.
///
/// The nets are handed out in bundle order by [`net_of`], so the answer
/// is indexed the way [`Bundle::offsets`] is however the drag reordered
/// the set.
fn leader_y_per_trace(
  world: &World,
  dragger: &MultiDragger,
  bundle: &Bundle,
) -> Vec<Option<i32>> {
  let leaders = leader_segments(world, dragger);

  (0..bundle.traces.len())
    .map(|index| {
      leaders.iter().find_map(|(net, seg)| {
        (*net == net_of(index) && seg.a.y == seg.b.y).then_some(seg.a.y)
      })
    })
    .collect()
}

/// Whether anything of a bundle's nets collides in a node.
fn bundle_collides(
  world: &World,
  node: NodeId,
  bundle: &Bundle,
  rules: &FixedClearance,
) -> bool {
  let options = pnsrouter::collide::CollisionSearchOptions {
    limit_count: Some(1),
    ..pnsrouter::collide::CollisionSearchOptions::default()
  };

  (0..bundle.traces.len()).any(|index| {
    world
      .all_items_in_net(node, net_of(index), pnsrouter::item::Kind::SEGMENT)
      .into_iter()
      .any(|id| {
        world.item(id).is_some_and(|item| {
          world
            .check_colliding(
              node,
              pnsrouter::rules::ItemRef::stored(id, item),
              rules,
              &options,
            )
            .is_some()
        })
      })
  })
}

// ---------------------------------------------------------------------
// Mark obstacles mode
// ---------------------------------------------------------------------

#[test]
fn a_pair_dragged_in_mark_obstacles_mode_keeps_its_spacing() {
  // `pcbnew/router/pns_multi_dragger.cpp:849` to `:915`: the primary line
  // goes to the cursor and every other line to the point at the same
  // signed perpendicular offset from the primary's leader segment that it
  // had before the drag.
  let (mut world, bundle) = build_pair();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = MultiDragger::new(&world, world.root());
  let grab = Vec2::new(GRAB_X, bundle.offsets[0]);
  let travel = 4_000_000;

  assert!(dragger.start(&mut world, &context, grab, &bundle.middle_segments()));
  // The cursor sits on the first trace's middle segment, which is a
  // strict mid segment grab and nothing else is, so the mode is segment.
  assert_eq!(dragger.mode(), DragMode::Segment);

  assert!(dragger.drag(
    &mut world,
    &context,
    Vec2::new(GRAB_X, bundle.offsets[0] + travel)
  ));

  let node = dragger.current_node();
  let (added, removed) = delta_segments(&world, node);

  // Both traces were taken apart and rebuilt.
  assert_eq!(removed.len(), 6, "{removed:?}");
  assert!(added.len() >= 6, "{added:?}");

  // Both leader segments moved, by the same amount, so the spacing is
  // what it was.
  let leaders = leader_y_per_trace(&world, &dragger, &bundle);

  assert_eq!(
    leaders,
    vec![
      Some(bundle.offsets[0] + travel),
      Some(bundle.offsets[1] + travel)
    ],
    "{leaders:?}"
  );
}

#[test]
fn a_bundle_with_unequal_spacing_keeps_every_offset() {
  // Nothing in `tryPosture` assumes a constant pitch: each line carries
  // its own signed `LineDistance` from the primary's leader segment
  // (`:893`).
  let offsets = [0, 1_000_000, 2_500_000];
  let (mut world, bundle) = build(&offsets);
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = MultiDragger::new(&world, world.root());
  let travel = 3_000_000;

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(GRAB_X, offsets[0]),
    &bundle.middle_segments()
  ));
  assert_eq!(dragger.mode(), DragMode::Segment);
  assert!(dragger.drag(
    &mut world,
    &context,
    Vec2::new(GRAB_X, offsets[0] + travel)
  ));

  let leaders = leader_y_per_trace(&world, &dragger, &bundle);

  assert_eq!(
    leaders,
    offsets
      .iter()
      .map(|offset| Some(offset + travel))
      .collect::<Vec<_>>(),
    "{leaders:?}"
  );
}

// ---------------------------------------------------------------------
// Walkaround mode
// ---------------------------------------------------------------------

#[test]
fn a_pair_dragged_in_walkaround_mode_bends_around_a_pad() {
  // `multidragWalkaround` (`:458`) walks the set in both orders and keeps
  // the ordering that bends it least.
  let (mut world, bundle) = build_pair();
  let travel = 4_000_000;
  let pad_at = Vec2::new(GRAB_X, bundle.offsets[0] + travel);

  add_pad(&mut world, pad_at);

  let rules = rules();
  let settings = settings_for(RouterMode::Walkaround);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = MultiDragger::new(&world, world.root());

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(GRAB_X, bundle.offsets[0]),
    &bundle.middle_segments()
  ));
  assert!(dragger.drag(
    &mut world,
    &context,
    Vec2::new(GRAB_X, bundle.offsets[0] + travel)
  ));

  let node = dragger.current_node();
  let (added, removed) = delta_segments(&world, node);

  assert_eq!(removed.len(), 6, "{removed:?}");
  // The walk bends the dragged trace around the pad, so the first trace
  // comes back with more pieces than the straight drag would have made.
  assert!(added.len() > 6, "the walkaround added {added:?}");

  // Nothing the bundle owns is left lying on the pad.
  assert!(
    !bundle_collides(&world, node, &bundle, &rules),
    "the walked bundle still collides"
  );

  // The second trace is clear of the pad to begin with, so it kept its
  // offset while the first one detoured.
  let leaders = leader_y_per_trace(&world, &dragger, &bundle);

  assert_eq!(leaders[1], Some(bundle.offsets[1] + travel), "{leaders:?}");
}

// ---------------------------------------------------------------------
// Shove mode
// ---------------------------------------------------------------------

#[test]
fn a_pair_dragged_in_shove_mode_pushes_a_third_track_aside() {
  // `multidragShove` (`:613`) turns every dragged line into a shove head,
  // in drag distance order, and lets the engine move whatever is in the
  // way.
  let (mut world, bundle) = build_pair();
  let travel = 4_000_000;
  let victim_y = bundle.offsets[0] + travel + 200_000;
  let victim = add_trace(&mut world, victim_y, OBSTACLE_NET);
  let rules = rules();
  let settings = settings_for(RouterMode::Shove);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = MultiDragger::new(&world, world.root());

  assert!(dragger.start(
    &mut world,
    &context,
    Vec2::new(GRAB_X, bundle.offsets[0]),
    &bundle.middle_segments()
  ));
  assert!(dragger.drag(
    &mut world,
    &context,
    Vec2::new(GRAB_X, bundle.offsets[0] + travel)
  ));

  let node = dragger.current_node();
  let (_, removed) = world.get_updated_items(node);

  // The third track was in the way at 200 um where 300 um is needed, so
  // the shove replaced it.
  assert!(
    removed.contains(&victim[1]),
    "the shove left the third track where it was"
  );
  assert!(
    !bundle_collides(&world, node, &bundle, &rules),
    "the shoved bundle still collides"
  );

  let leaders = leader_y_per_trace(&world, &dragger, &bundle);

  assert_eq!(
    leaders,
    vec![
      Some(bundle.offsets[0] + travel),
      Some(bundle.offsets[1] + travel)
    ],
    "{leaders:?}"
  );
}

// ---------------------------------------------------------------------
// Corner mode
// ---------------------------------------------------------------------

#[test]
fn a_pair_grabbed_at_their_ends_drags_in_corner_mode() {
  // `Start`'s phase 2 (`:113`): a selected segment whose endpoint is the
  // line's own end makes the line a corner candidate, and a cursor within
  // half a track width of that end makes it strict, which forces
  // `DM_CORNER` at `:176`.
  let (mut world, bundle) = build_pair();
  let rules = rules();
  let settings = settings_for(RouterMode::MarkObstacles);
  let context = AlgoContext::new(&rules, &settings);
  let mut dragger = MultiDragger::new(&world, world.root());
  // The last segment of each trace, whose far endpoint is the trace's own
  // end and whose joint there is trivial.
  let ends: Vec<ItemId> = bundle.traces.iter().map(|trace| trace[2]).collect();
  let grab = Vec2::new(RIGHT, bundle.offsets[0]);

  assert!(dragger.start(&mut world, &context, grab, &ends));
  assert_eq!(dragger.mode(), DragMode::Corner);

  let target = Vec2::new(RIGHT, bundle.offsets[0] - 3_000_000);

  assert!(dragger.drag(&mut world, &context, target));

  let node = dragger.current_node();
  let (added, removed) = delta_segments(&world, node);

  assert_eq!(removed.len(), 6, "{removed:?}");
  assert!(!added.is_empty(), "the corner drag added nothing");

  // The primary line's end followed the cursor exactly.
  let ends_now: Vec<Vec2> = added
    .iter()
    .flat_map(|seg| [seg.a, seg.b])
    .filter(|point| point.x == RIGHT)
    .collect();

  assert!(
    ends_now.contains(&target),
    "the grabbed end is not at the cursor: {ends_now:?}"
  );

  // `Traces()` holds the parallel lines only (`:880`), never the primary,
  // so a pair answers with one line.
  assert_eq!(dragger.traces().len(), 1);
  // Corner mode takes the last link of each dragged line as its leader
  // (`:441`), so both traces contribute one.
  assert_eq!(dragger.last_committed_leader_segments().len(), 2);
}

// ---------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------

#[test]
fn two_identical_multi_drags_answer_identically() {
  // `DESIGN.md` section 8, and note 06 erratum E28: KiCad's sort over
  // `dragDist` is unstable and leaves a corner mode bundle's order
  // unspecified, so this port ties on the line index.
  let run = || {
    let (mut world, bundle) = build(&[0, 1_000_000, 2_500_000]);
    let rules = rules();
    let settings = settings_for(RouterMode::Shove);
    let context = AlgoContext::new(&rules, &settings);
    let mut dragger = MultiDragger::new(&world, world.root());

    assert!(dragger.start(
      &mut world,
      &context,
      Vec2::new(GRAB_X, 0),
      &bundle.middle_segments()
    ));

    let mut frames = Vec::new();

    for step in 1..=4 {
      dragger.drag(&mut world, &context, Vec2::new(GRAB_X, step * 750_000));

      let node = dragger.current_node();

      frames.push((
        delta_segments(&world, node),
        leader_segments(&world, &dragger),
      ));
    }

    frames
  };

  let first = run();
  let second = run();

  assert_eq!(first, second);
  assert!(first.iter().any(|(delta, _)| !delta.0.is_empty()));
}
