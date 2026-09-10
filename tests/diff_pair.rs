// SPDX-License-Identifier: GPL-3.0-or-later

//! The differential pair gateway builders and the fit, pinned in full.
//!
//! `doc/reference/kicad/07-differential-pairs.md` section 12.5 names the
//! push order of every gateway builder as part of the router's answer:
//! `DP_GATEWAYS::FitGateways` keeps the **last** candidate of an equal
//! score (`pcbnew/router/pns_diff_pair.cpp:354`), so a port that reorders
//! a builder, or drops a gateway that never wins, picks a different
//! route. Section 14 step 4 therefore asks for the complete ordered
//! gateway list of five inputs, and step 5 for the chains
//! [`pnsrouter::diff_pair::fit_gateways`] fits from them.
//!
//! # Where the expected values come from
//!
//! They are worked out from KiCad's own code rather than recorded from
//! this crate. Each list was derived by hand from
//! `pcbnew/router/pns_diff_pair.cpp` and the `libs/kimath` routines it
//! calls (`VECTOR2I::Resize`, `SEG::Collinear`, `SEG::IntersectLines`,
//! `DIRECTION_45::BuildInitialTrace`), with a throwaway model of those
//! same C++ routines used as the calculator for the rounding heavy steps,
//! `Resize` and the `ceil( gap * sqrt(2) )` legs above all. The five
//! inputs are:
//!
//! 1. Two round pads on a common horizontal line, a pitch apart. The
//!    anchor line lies along the direction of travel, which is the case
//!    that has to fan out sideways.
//! 2. Two round pads on a common 45 degree line.
//! 3. Two rectangular pads on a common vertical line, further apart than
//!    the pitch, which is the case the pad fan block of
//!    `BuildFromPrimitivePair` was written for. They are deliberately not
//!    square, so that the diagonal fan distance `w - h` is not zero.
//! 4. Two existing tracks arriving from the west, which takes the
//!    `buildDpContinuation` branch.
//! 5. A bare cursor, which is `BuildForCursor` with via fitting off.

#![forbid(unsafe_code)]

use pnsrouter::diff_pair::{
  DiffPair, DpGatewayError, DpGateways, DpPrimitivePair, fit_gateways,
};
use pnsrouter::geometry::direction45::{AngleType, Direction45};
use pnsrouter::geometry::line_chain::LineChain;
use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{ItemBody, ItemId, LayerRange, NetId, Segment, Solid};
use pnsrouter::node::World;

/// The width of one lane, `Sizes::diff_pair_width`.
const WIDTH: i32 = 200_000;

/// The copper gap between the lanes, `Sizes::diff_pair_gap`.
const GAP: i32 = 200_000;

/// The centre to centre spacing every builder spaces anchors by,
/// `Sizes::diff_pair_pitch`.
const PITCH: i32 = WIDTH + GAP;

/// Where the target gateways sit, well east of every entry fixture.
const TARGET: Vec2 = Vec2::new(4_000_000, 0);

/// The P net of every fixture.
const NET_P: Option<NetId> = Some(NetId(1));

/// The N net of every fixture.
const NET_N: Option<NetId> = Some(NetId(2));

/// One gateway of a set, in the form the expected lists are written in.
///
/// The complete state a builder decides, minus the entry chains
/// themselves: the two anchors, the diagonal hint, the allowed entry
/// angle mask as KiCad's raw bits, the priority and whether the gateway
/// carries entry chains at all.
fn summarise(gateways: &DpGateways) -> Vec<String> {
  gateways
    .gateways()
    .iter()
    .map(|gateway| {
      format!(
        "p=({}, {}) n=({}, {}) diag={} allowed=0x{:02x} prio={} entries={}",
        gateway.anchor_p().x,
        gateway.anchor_p().y,
        gateway.anchor_n().x,
        gateway.anchor_n().y,
        gateway.is_diagonal(),
        gateway.allowed_angles().bits(),
        gateway.priority(),
        gateway.has_entry_lines(),
      )
    })
    .collect()
}

/// Assert the whole ordered gateway list, one line per gateway.
///
/// The comparison is on the joined text so that a failure prints the two
/// lists as blocks rather than as one long debug line.
#[track_caller]
fn assert_gateways(gateways: &DpGateways, expected: &[&str]) {
  assert_eq!(summarise(gateways).join("\n"), expected.join("\n"));
}

/// The points of a chain, for comparing a fitted lane.
fn points(chain: &LineChain) -> Vec<Vec2> {
  chain.points().to_vec()
}

/// A world holding one pair of round pads, and the primitive pair over
/// them.
fn round_pads(anchor_p: Vec2, anchor_n: Vec2) -> (World, DpPrimitivePair) {
  pad_pair(anchor_p, anchor_n, |at| Shape::circle(at, 150_000))
}

/// A world holding one pair of 600 by 300 micrometre rectangular pads.
fn rectangular_pads(
  anchor_p: Vec2,
  anchor_n: Vec2,
) -> (World, DpPrimitivePair) {
  pad_pair(anchor_p, anchor_n, |at| {
    let size = Vec2::new(600_000, 300_000);

    Shape::rect(at - Vec2::new(size.x / 2, size.y / 2), size)
  })
}

/// The shared body of [`round_pads`] and [`rectangular_pads`].
fn pad_pair(
  anchor_p: Vec2,
  anchor_n: Vec2,
  shape: impl Fn(Vec2) -> Shape,
) -> (World, DpPrimitivePair) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let mut add = |at: Vec2, net: Option<NetId>| -> ItemId {
    let body = ItemBody::Solid(Solid::new(shape(at), at));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);
    world.add_solid(root, item, None)
  };

  let prim_p = add(anchor_p, NET_P);
  let prim_n = add(anchor_n, NET_N);
  let pair = DpPrimitivePair::from_items(&world, prim_p, prim_n)
    .expect("both pads were just added");

  (world, pair)
}

/// A world holding two tracks that end at the two anchors and run west,
/// and the primitive pair over them.
fn track_pair(anchor_p: Vec2, anchor_n: Vec2) -> (World, DpPrimitivePair) {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let mut add = |at: Vec2, net: Option<NetId>| -> ItemId {
    let seg = Seg::new(at, Vec2::new(at.x - 1_000_000, at.y));
    let body = ItemBody::Segment(Segment::new(seg, WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);
    world
      .add_segment(root, item, false)
      .expect("the track is neither degenerate nor redundant")
  };

  let prim_p = add(anchor_p, NET_P);
  let prim_n = add(anchor_n, NET_N);
  let pair = DpPrimitivePair::from_items(&world, prim_p, prim_n)
    .expect("both tracks were just added");

  (world, pair)
}

/// The gateway set of one of the five inputs, by name.
fn entry_set(input: &str) -> DpGateways {
  let mut gateways = DpGateways::new(PITCH);

  let (world, pair) = match input {
    "round pads on a horizontal" => {
      round_pads(Vec2::new(-200_000, 0), Vec2::new(200_000, 0))
    }
    "round pads on a diagonal" => {
      round_pads(Vec2::new(0, 0), Vec2::new(282_843, 282_843))
    }
    "rectangular pads" => {
      rectangular_pads(Vec2::new(0, -300_000), Vec2::new(0, 300_000))
    }
    "segment continuation" => {
      track_pair(Vec2::new(0, -200_000), Vec2::new(0, 200_000))
    }
    "bare cursor" => {
      gateways.set_fit_vias(false, 0, None);
      gateways.build_for_cursor(Vec2::new(0, 0));

      return gateways;
    }
    other => panic!("no such input: {other}"),
  };

  gateways
    .build_from_primitive_pair(&world, &pair, false)
    .expect("every fixture is a matched pair of board objects");

  gateways
}

/// The target gateways every fit in this file aims at: a bare cursor
/// well to the east.
fn target_set() -> DpGateways {
  let mut gateways = DpGateways::new(PITCH);

  gateways.set_fit_vias(false, 0, None);
  gateways.build_for_cursor(TARGET);

  gateways
}

/// The fit of one input against [`target_set`].
fn fit(input: &str) -> DiffPair {
  fit_gateways(PITCH, &entry_set(input), &target_set(), false)
    .expect("every fixture fits")
}

#[test]
fn gateways_from_round_pads_on_a_horizontal() {
  assert_gateways(
    &entry_set("round pads on a horizontal"),
    &[
      "p=(-100000, 0) n=(100000, 0) diag=false allowed=0x02 prio=1 entries=true",
      "p=(-600000, 0) n=(-600000, 400000) diag=false allowed=0x01 prio=0 entries=true",
      "p=(-600000, 0) n=(-600000, -400000) diag=false allowed=0x01 prio=0 entries=true",
      "p=(600000, 400000) n=(600000, 0) diag=false allowed=0x01 prio=0 entries=true",
      "p=(600000, -400000) n=(600000, 0) diag=false allowed=0x01 prio=0 entries=true",
      "p=(-200000, 0) n=(200000, 0) diag=true allowed=0x01 prio=20 entries=true",
      "p=(-200000, 0) n=(200000, 0) diag=true allowed=0x01 prio=20 entries=true",
      "p=(-200000, 0) n=(200000, 0) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-82843, 117157) n=(200000, -165686) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-200000, 165686) n=(82843, -117157) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-200000, 0) n=(200000, 0) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-200000, 0) n=(200000, 0) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-82843, -117157) n=(200000, 165686) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-200000, -165686) n=(82843, 117157) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-200000, 0) n=(200000, 0) diag=true allowed=0x01 prio=0 entries=true",
    ],
  );
}

#[test]
fn gateways_from_round_pads_on_a_diagonal() {
  assert_gateways(
    &entry_set("round pads on a diagonal"),
    &[
      "p=(70710, 70710) n=(212132, 212132) diag=true allowed=0x02 prio=1 entries=true",
      "p=(-282843, -282843) n=(-565686, 0) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-282843, -282843) n=(0, -565686) diag=true allowed=0x01 prio=0 entries=true",
      "p=(282843, 848529) n=(565686, 565686) diag=true allowed=0x01 prio=0 entries=true",
      "p=(848529, 282843) n=(565686, 565686) diag=true allowed=0x01 prio=0 entries=true",
      "p=(0, 0) n=(282843, 282843) diag=false allowed=0x01 prio=20 entries=true",
      "p=(117157, -117157) n=(117157, 282843) diag=true allowed=0x01 prio=0 entries=true",
      "p=(0, 0) n=(282843, 282843) diag=true allowed=0x01 prio=0 entries=true",
      "p=(0, 0) n=(282843, 282843) diag=true allowed=0x01 prio=0 entries=true",
      "p=(165686, 0) n=(165686, 400000) diag=true allowed=0x01 prio=0 entries=true",
      "p=(0, 0) n=(282843, 282843) diag=false allowed=0x01 prio=20 entries=true",
      "p=(-117157, 117157) n=(282843, 117157) diag=true allowed=0x01 prio=0 entries=true",
      "p=(0, 0) n=(282843, 282843) diag=true allowed=0x01 prio=0 entries=true",
      "p=(0, 0) n=(282843, 282843) diag=true allowed=0x01 prio=0 entries=true",
      "p=(0, 165686) n=(400000, 165686) diag=true allowed=0x01 prio=0 entries=true",
    ],
  );
}

#[test]
fn gateways_from_rectangular_pads() {
  assert_gateways(
    &entry_set("rectangular pads"),
    &[
      "p=(550001, -200000) n=(550001, 200000) diag=false allowed=0x01 prio=100 entries=true",
      "p=(-550001, -200000) n=(-550001, 200000) diag=false allowed=0x01 prio=100 entries=true",
      "p=(250000, -200000) n=(250000, 200000) diag=false allowed=0x01 prio=99 entries=true",
      "p=(-250000, -200000) n=(-250000, 200000) diag=false allowed=0x01 prio=99 entries=true",
      "p=(0, -100000) n=(0, 100000) diag=false allowed=0x02 prio=1 entries=true",
      "p=(0, -700000) n=(-400000, -700000) diag=false allowed=0x01 prio=0 entries=true",
      "p=(0, -700000) n=(400000, -700000) diag=false allowed=0x01 prio=0 entries=true",
      "p=(-400000, 700000) n=(0, 700000) diag=false allowed=0x01 prio=0 entries=true",
      "p=(400000, 700000) n=(0, 700000) diag=false allowed=0x01 prio=0 entries=true",
      "p=(200000, -100000) n=(200000, 300000) diag=true allowed=0x01 prio=0 entries=true",
      "p=(317157, 17157) n=(34314, 300000) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-34314, -300000) n=(-317157, -17157) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-200000, -300000) n=(-200000, 100000) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-100000, -200000) n=(-100000, 200000) diag=true allowed=0x01 prio=20 entries=true",
      "p=(-200000, -100000) n=(-200000, 300000) diag=true allowed=0x01 prio=0 entries=true",
      "p=(-317157, 17157) n=(-34314, 300000) diag=true allowed=0x01 prio=0 entries=true",
      "p=(34314, -300000) n=(317157, -17157) diag=true allowed=0x01 prio=0 entries=true",
      "p=(200000, -300000) n=(200000, 100000) diag=true allowed=0x01 prio=0 entries=true",
      "p=(100000, -200000) n=(100000, 200000) diag=true allowed=0x01 prio=20 entries=true",
    ],
  );
}

#[test]
fn gateways_from_a_segment_continuation() {
  assert_gateways(
    &entry_set("segment continuation"),
    &[
      "p=(0, -200000) n=(0, 200000) diag=false allowed=0x01 prio=100 entries=false",
      "p=(153072, -200000) n=(0, 200000) diag=false allowed=0x01 prio=20 entries=true",
      "p=(0, -200000) n=(153072, 200000) diag=false allowed=0x01 prio=20 entries=true",
      "p=(159500, -200000) n=(0, 200000) diag=false allowed=0x01 prio=5 entries=true",
      "p=(0, -200000) n=(159500, 200000) diag=false allowed=0x01 prio=5 entries=true",
    ],
  );
}

#[test]
fn gateways_from_a_bare_cursor() {
  assert_gateways(
    &entry_set("bare cursor"),
    &[
      "p=(-141422, -141422) n=(141422, 141422) diag=false allowed=0x01 prio=0 entries=false",
      "p=(141422, -141422) n=(-141422, 141422) diag=false allowed=0x01 prio=0 entries=false",
      "p=(-141422, 141422) n=(141422, -141422) diag=false allowed=0x01 prio=0 entries=false",
      "p=(141422, 141422) n=(-141422, -141422) diag=false allowed=0x01 prio=0 entries=false",
      "p=(200000, 0) n=(-200000, 0) diag=true allowed=0x01 prio=0 entries=false",
      "p=(-200000, 0) n=(200000, 0) diag=true allowed=0x01 prio=0 entries=false",
      "p=(0, 200000) n=(0, -200000) diag=true allowed=0x01 prio=0 entries=false",
      "p=(0, -200000) n=(0, 200000) diag=true allowed=0x01 prio=0 entries=false",
    ],
  );
}

#[test]
fn fit_from_round_pads_on_a_horizontal() {
  let fitted = fit("round pads on a horizontal");

  assert_eq!(
    points(fitted.chain_p()),
    vec![
      Vec2::new(-200000, 0),
      Vec2::new(200000, -400000),
      Vec2::new(600000, -400000),
      Vec2::new(3882844, -400000),
      Vec2::new(4141422, -141422),
    ],
  );
  assert_eq!(
    points(fitted.chain_n()),
    vec![
      Vec2::new(200000, 0),
      Vec2::new(600000, 0),
      Vec2::new(3717156, 0),
      Vec2::new(3858578, 141422),
    ],
  );
}

#[test]
fn fit_from_round_pads_on_a_diagonal() {
  let fitted = fit("round pads on a diagonal");

  assert_eq!(
    points(fitted.chain_p()),
    vec![
      Vec2::new(0, 0),
      Vec2::new(200000, -200000),
      Vec2::new(4000000, -200000),
    ],
  );
  assert_eq!(
    points(fitted.chain_n()),
    vec![
      Vec2::new(282843, 282843),
      Vec2::new(365686, 200000),
      Vec2::new(4000000, 200000),
    ],
  );
}

#[test]
fn fit_from_rectangular_pads() {
  let fitted = fit("rectangular pads");

  assert_eq!(
    points(fitted.chain_p()),
    vec![
      Vec2::new(0, -300000),
      Vec2::new(450001, -300000),
      Vec2::new(550001, -200000),
      Vec2::new(4000000, -200000),
    ],
  );
  assert_eq!(
    points(fitted.chain_n()),
    vec![
      Vec2::new(0, 300000),
      Vec2::new(450001, 300000),
      Vec2::new(550001, 200000),
      Vec2::new(4000000, 200000),
    ],
  );
}

#[test]
fn fit_from_a_segment_continuation() {
  let fitted = fit("segment continuation");

  assert_eq!(
    points(fitted.chain_p()),
    vec![Vec2::new(0, -200000), Vec2::new(4000000, -200000),],
  );
  assert_eq!(
    points(fitted.chain_n()),
    vec![Vec2::new(0, 200000), Vec2::new(4000000, 200000),],
  );
}

#[test]
fn fit_from_a_bare_cursor() {
  let fitted = fit("bare cursor");

  assert_eq!(
    points(fitted.chain_p()),
    vec![Vec2::new(0, -200000), Vec2::new(4000000, -200000),],
  );
  assert_eq!(
    points(fitted.chain_n()),
    vec![Vec2::new(0, 200000), Vec2::new(4000000, 200000),],
  );
}

/// Erratum E4: the collinear midpoint exit of `BuildGeneric` spaces its
/// two anchors at **half** the pitch, because `makeGapVector` halves and
/// it is handed `m_gap / 2` (`pns_diff_pair.cpp:675`). Every other
/// gateway in the file is a whole pitch apart. A route built from it
/// would have its lanes half a pitch apart at the anchors, which
/// `BuildInitial`'s own gap test rejects for any pitch above 200
/// nanometres, so it is present, ported as written, and unreachable.
#[test]
fn the_collinear_midpoint_gateway_is_half_spaced_and_never_fits() {
  for input in [
    "round pads on a horizontal",
    "round pads on a diagonal",
    "rectangular pads",
  ] {
    let set = entry_set(input);
    let midpoint = set
      .gateways()
      .iter()
      .find(|gateway| gateway.allowed_angles() == AngleType::RIGHT)
      .unwrap_or_else(|| panic!("{input} has a collinear midpoint gateway"));
    let spacing = (midpoint.anchor_p() - midpoint.anchor_n()).euclidean_norm();

    assert!(
      (spacing - PITCH / 2).abs() <= 2,
      "{input}: the midpoint anchors are {spacing} apart, not {}",
      PITCH / 2
    );

    let mut alone = DpGateways::new(PITCH);

    alone.set_fit_vias(false, 0, None);
    alone.gateways_mut().push(midpoint.clone());

    assert!(
      fit_gateways(PITCH, &alone, &target_set(), false).is_none(),
      "{input}: the midpoint gateway fitted something"
    );
  }

  for input in ["segment continuation", "bare cursor"] {
    assert!(
      entry_set(input)
        .gateways()
        .iter()
        .all(|gateway| gateway.allowed_angles() != AngleType::RIGHT),
      "{input} goes nowhere near BuildGeneric's collinear block"
    );
  }
}

/// `FilterByOrientation` **removes** what matches the mask, which is the
/// opposite of what its name suggests. Its one caller drops every
/// gateway whose anchor line runs along the direction of travel
/// (`pns_diff_pair_placer.cpp:724`), leaving the ones that lie across
/// it.
#[test]
fn filter_by_orientation_drops_the_gateways_along_the_way_we_are_going() {
  let mut gateways = entry_set("bare cursor");

  assert_eq!(gateways.gateways().len(), 8);

  gateways.filter_by_orientation(
    AngleType::STRAIGHT | AngleType::HALF_FULL,
    Direction45::from_vector(Vec2::new(1, 0), false),
  );

  assert_gateways(
    &gateways,
    &[
      "p=(-141422, -141422) n=(141422, 141422) diag=false allowed=0x01 prio=0 entries=false",
      "p=(141422, -141422) n=(-141422, 141422) diag=false allowed=0x01 prio=0 entries=false",
      "p=(-141422, 141422) n=(141422, -141422) diag=false allowed=0x01 prio=0 entries=false",
      "p=(141422, 141422) n=(-141422, -141422) diag=false allowed=0x01 prio=0 entries=false",
      "p=(0, 200000) n=(0, -200000) diag=true allowed=0x01 prio=0 entries=false",
      "p=(0, -200000) n=(0, 200000) diag=true allowed=0x01 prio=0 entries=false",
    ],
  );
}

/// Erratum E10: a pad paired with a track matches neither branch of
/// `BuildFromPrimitivePair`, so KiCad appends nothing and returns
/// silently with a null shape (`pns_diff_pair.cpp:453`). Here the reason
/// survives.
#[test]
fn a_mixed_primitive_pair_is_an_error_rather_than_a_silent_nothing() {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();

  let at = Vec2::new(0, -200_000);
  let body = ItemBody::Solid(Solid::new(Shape::circle(at, 150_000), at));
  let mut pad = world.make_item(body);

  pad.set_layers_and_flash_all(LayerRange::single(0));
  pad.set_net(NET_P);

  let prim_p = world.add_solid(root, pad, None);

  let seg = Seg::new(Vec2::new(0, 200_000), Vec2::new(-1_000_000, 200_000));
  let body = ItemBody::Segment(Segment::new(seg, WIDTH));
  let mut track = world.make_item(body);

  track.set_layers_and_flash_all(LayerRange::single(0));
  track.set_net(NET_N);

  let prim_n = world
    .add_segment(root, track, false)
    .expect("the track is neither degenerate nor redundant");

  let pair = DpPrimitivePair::from_items(&world, prim_p, prim_n)
    .expect("both objects were just added");
  let mut gateways = DpGateways::new(PITCH);

  assert_eq!(
    gateways.build_from_primitive_pair(&world, &pair, false),
    Err(DpGatewayError::MixedPrimitiveKinds)
  );
  assert!(gateways.gateways().is_empty());
}

/// `CursorOrientation`'s parallel segment branch returns before the
/// cursor is ever consulted (`pns_diff_pair.cpp:144`), so the direction
/// is whichever way the P segment runs even when the cursor is the other
/// way. Both tracks here run west, and the cursor is far to the east.
#[test]
fn cursor_orientation_of_two_parallel_tracks_ignores_the_cursor() {
  let (world, pair) = track_pair(Vec2::new(0, -200_000), Vec2::new(0, 200_000));

  for cursor in [TARGET, Vec2::new(-4_000_000, 0)] {
    let orientation = pair
      .cursor_orientation(&world, cursor)
      .expect("both primitives are there");

    assert_eq!(orientation.midpoint, Vec2::new(-1_000_000, 0));
    assert_eq!(orientation.direction, Vec2::new(-400_000, 0));
  }
}

/// Two segments that are not parallel fall through to the perpendicular
/// of the anchor line, taken at `Anchor(1)` (`pns_diff_pair.cpp:131`),
/// and that one is flipped towards the cursor.
#[test]
fn cursor_orientation_of_two_crossing_tracks_follows_the_cursor() {
  let mut world = World::new(World::DEFAULT_MAX_CLEARANCE);
  let root = world.root();
  let mut add = |seg: Seg, net: Option<NetId>| -> ItemId {
    let body = ItemBody::Segment(Segment::new(seg, WIDTH));
    let mut item = world.make_item(body);

    item.set_layers_and_flash_all(LayerRange::single(0));
    item.set_net(net);
    world
      .add_segment(root, item, false)
      .expect("the track is neither degenerate nor redundant")
  };

  let prim_p = add(
    Seg::new(Vec2::new(0, -200_000), Vec2::new(-1_000_000, -200_000)),
    NET_P,
  );
  let prim_n = add(
    Seg::new(Vec2::new(0, 200_000), Vec2::new(-1_000_000, -800_000)),
    NET_N,
  );
  let pair = DpPrimitivePair::from_items(&world, prim_p, prim_n)
    .expect("both tracks were just added");

  let east = pair
    .cursor_orientation(&world, TARGET)
    .expect("both primitives are there");

  assert_eq!(east.midpoint, Vec2::new(-1_000_000, -500_000));
  assert_eq!(east.direction, Vec2::new(600_000, 0));

  let west = pair
    .cursor_orientation(&world, Vec2::new(-4_000_000, 0))
    .expect("both primitives are there");

  assert_eq!(west.direction, Vec2::new(-600_000, 0));
}

/// When the two objects are not both segments the anchors come from
/// `Anchor(0)` instead (`pns_diff_pair.cpp:149`), and the direction is
/// again flipped towards the cursor.
#[test]
fn cursor_orientation_of_two_pads_uses_the_first_anchor() {
  let (world, pair) = round_pads(Vec2::new(0, -200_000), Vec2::new(0, 200_000));

  let east = pair
    .cursor_orientation(&world, TARGET)
    .expect("both primitives are there");

  assert_eq!(east.midpoint, Vec2::new(0, 0));
  assert_eq!(east.direction, Vec2::new(400_000, 0));

  let west = pair
    .cursor_orientation(&world, Vec2::new(-4_000_000, 0))
    .expect("both primitives are there");

  assert_eq!(west.direction, Vec2::new(-400_000, 0));
}

/// The direction an existing track arrives from, which
/// `buildDpContinuation` extends: at `Anchor(0)` it points **away** from
/// the anchor back along the track (`pns_diff_pair.cpp:114`).
#[test]
fn the_anchor_direction_points_back_along_the_track() {
  let (world, pair) = track_pair(Vec2::new(0, -200_000), Vec2::new(0, 200_000));

  assert!(pair.directional(&world));
  assert_eq!(pair.dir_p(&world).to_vector(), Vec2::new(1, 0));
  assert_eq!(pair.dir_n(&world).to_vector(), Vec2::new(1, 0));

  let (pad_world, pads) =
    round_pads(Vec2::new(0, -200_000), Vec2::new(0, 200_000));

  assert!(!pads.directional(&pad_world));
  assert!(!pads.dir_p(&pad_world).is_defined());
}

/// A pad fan whose two pads are exactly a pitch apart is unusable, and
/// nothing in KiCad says so.
///
/// The fan lead is the three point chain
/// `{ p0, p0 + sign * dir, gateway }` (`pns_diff_pair.cpp:530`), and the
/// convergence `max( 0, padDist - m_gap )` that separates the second
/// point from the third is zero when the pads are a pitch apart. The
/// chain's vector constructor does not suppress the duplicate, so the
/// lead ends with a zero length segment, whose `DIRECTION_45` is
/// undefined, whose angle is therefore `ANG_UNDEFINED`, and which no
/// allowed angle mask contains. Every candidate through such a gateway
/// is rejected by the entry angle test of `BuildInitial`, whatever its
/// priority of 100 or 99 promised.
#[test]
fn a_pad_fan_at_exactly_the_pitch_has_a_degenerate_lead() {
  let (world, pair) =
    rectangular_pads(Vec2::new(0, -PITCH / 2), Vec2::new(0, PITCH / 2));
  let mut gateways = DpGateways::new(PITCH);

  gateways
    .build_from_primitive_pair(&world, &pair, false)
    .expect("two pads of the same kind");

  let fan: Vec<_> = gateways
    .gateways()
    .iter()
    .filter(|gateway| gateway.priority() >= 99)
    .cloned()
    .collect();

  assert_eq!(fan.len(), 4, "two fan distances, two signs");

  for gateway in fan {
    let lead = gateway.entry_p();

    assert_eq!(lead.point_count(), 3);
    assert_eq!(
      lead.point(1),
      lead.point(2),
      "the convergence is zero, so the lead repeats its last point"
    );

    let mut alone = DpGateways::new(PITCH);

    alone.set_fit_vias(false, 0, None);
    alone.gateways_mut().push(gateway);

    assert!(
      fit_gateways(PITCH, &alone, &target_set(), false).is_none(),
      "the undefined angle of the zero length segment rejects everything"
    );
  }
}
