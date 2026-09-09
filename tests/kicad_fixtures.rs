// SPDX-License-Identifier: GPL-3.0-or-later

//! Fixture tests for the KiCad PNS regression corpus.
//!
//! The corpus under `tests/fixtures/kicad/pns_regressions` is KiCad's own
//! router regression suite, copied verbatim; see the README next to it.
//! Replaying it against this crate is a later step
//! (`doc/work/005-session-api-and-event-log.md`). What is tested here is
//! only the reading: every board and every case has to come out of
//! `tests/support` with the shape it actually has on disk.
//!
//! Every expected number below was produced by a second, independent
//! reader written in Python (`tmp/verify_boards.py` at the time of
//! writing, not committed) rather than by the code under test, so a bug in
//! the Rust reader cannot quietly define its own expectations. Where a
//! number is cheap to reproduce by hand it is spelled out in a comment,
//! for example `grep -c '^\t(segment' boards/backspace1.kicad_pcb` gives
//! the 15 segments pinned for that board.

#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

/// The readers under test.
mod support;

use std::path::{Path, PathBuf};

use support::kicad_pcb::{
  self, KicadBoard, PadKind, PadShape, Point, read_board,
};
use support::pns_log::{
  self, CornerMode, EventKind, LogShape, RouterMode, RoutingMode, TestCaseType,
};
use support::sexpr;

/// How far outside the `Edge.Cuts` outline a pad is still allowed to sit.
///
/// Not zero: `boards/ultrasound.kicad_pcb` genuinely parks fourteen pads
/// off the board edge, the furthest of them 19.15 mm out. The exact count
/// is pinned per board in [`BoardExpectation::pads_outside_outline`]; this
/// margin only rules out a transform bug that throws a pad across the
/// county.
const OUTLINE_MARGIN_NANOMETRES: i64 = 25_000_000;

/// The root of the copied corpus.
fn corpus_root() -> PathBuf {
  Path::new(env!("CARGO_MANIFEST_DIR"))
    .join("tests/fixtures/kicad/pns_regressions")
}

/// Read a fixture file, failing the test with the path when it is
/// missing.
fn read_fixture(path: &Path) -> String {
  std::fs::read_to_string(path)
    .unwrap_or_else(|error| panic!("cannot read {}: {error}", path.display()))
}

/// What one board in `boards/` must parse to.
struct BoardExpectation {
  /// File name inside `boards/`.
  name: &'static str,
  /// The `(version ...)` stamp.
  file_version: i64,
  /// Copper layers in the stack.
  copper_layer_count: usize,
  /// Entries in the net table, synthesised or declared.
  nets: usize,
  /// Straight track segments.
  segments: usize,
  /// Curved track segments.
  arcs: usize,
  /// Vias.
  vias: usize,
  /// Pads over all footprints.
  pads: usize,
  /// Pads whose shape is `custom`.
  custom_pads: usize,
  /// Graphic items on `Edge.Cuts`.
  outline_items: usize,
  /// Keepout polygons, footprint local ones included.
  keepouts: usize,
  /// Pads whose absolute position coincides exactly with an endpoint of a
  /// track on the same net.
  ///
  /// This is the assertion that pins the footprint transform. A wrong
  /// rotation direction, a spurious mirror for a back side footprint or a
  /// misread pad angle all drive this number to nearly zero, because a
  /// coincidence to the nanometre does not survive any of them.
  pads_on_track_ends: usize,
  /// Pads that sit outside the `Edge.Cuts` bounding box.
  pads_outside_outline: usize,
}

/// Every board in the pool, with the numbers a second reader produced.
const BOARDS: &[BoardExpectation] = &[
  BoardExpectation {
    name: "backspace1.kicad_pcb",
    file_version: 20_260_206,
    copper_layer_count: 2,
    nets: 4,
    segments: 15,
    arcs: 0,
    vias: 0,
    pads: 83,
    custom_pads: 0,
    outline_items: 4,
    keepouts: 0,
    pads_on_track_ends: 4,
    pads_outside_outline: 0,
  },
  BoardExpectation {
    name: "dp_test.kicad_pcb",
    file_version: 20_250_907,
    copper_layer_count: 2,
    nets: 4,
    segments: 15,
    arcs: 0,
    vias: 0,
    pads: 83,
    custom_pads: 0,
    outline_items: 4,
    keepouts: 0,
    pads_on_track_ends: 4,
    pads_outside_outline: 0,
  },
  BoardExpectation {
    name: "drag-walk-optimize-fix-corners.kicad_pcb",
    file_version: 20_260_728,
    copper_layer_count: 2,
    nets: 2,
    segments: 3,
    arcs: 0,
    vias: 0,
    pads: 4,
    custom_pads: 0,
    outline_items: 1,
    keepouts: 0,
    pads_on_track_ends: 2,
    pads_outside_outline: 0,
  },
  BoardExpectation {
    name: "drag-walk-optimize.kicad_pcb",
    file_version: 20_260_728,
    copper_layer_count: 2,
    nets: 2,
    segments: 6,
    arcs: 0,
    vias: 0,
    pads: 4,
    custom_pads: 0,
    outline_items: 1,
    keepouts: 0,
    pads_on_track_ends: 2,
    pads_outside_outline: 0,
  },
  BoardExpectation {
    name: "pic_programmer.kicad_pcb",
    file_version: 20_260_206,
    copper_layer_count: 2,
    nets: 90,
    segments: 370,
    arcs: 0,
    vias: 6,
    pads: 219,
    custom_pads: 2,
    outline_items: 5,
    keepouts: 0,
    pads_on_track_ends: 102,
    pads_outside_outline: 0,
  },
  BoardExpectation {
    name: "shove_same_net_via.kicad_pcb",
    file_version: 20_260_513,
    copper_layer_count: 2,
    nets: 3,
    segments: 6,
    arcs: 0,
    vias: 1,
    pads: 4,
    custom_pads: 0,
    outline_items: 1,
    keepouts: 0,
    pads_on_track_ends: 2,
    pads_outside_outline: 0,
  },
  BoardExpectation {
    name: "simple.kicad_pcb",
    file_version: 20_220_914,
    copper_layer_count: 2,
    nets: 28,
    segments: 107,
    arcs: 0,
    vias: 15,
    pads: 77,
    custom_pads: 0,
    outline_items: 5,
    keepouts: 0,
    pads_on_track_ends: 52,
    pads_outside_outline: 0,
  },
  BoardExpectation {
    name: "stickhub-extra-via.kicad_pcb",
    file_version: 20_260_206,
    copper_layer_count: 2,
    nets: 48,
    segments: 1113,
    arcs: 180,
    vias: 88,
    pads: 278,
    custom_pads: 2,
    outline_items: 20,
    keepouts: 0,
    pads_on_track_ends: 201,
    pads_outside_outline: 0,
  },
  BoardExpectation {
    name: "ultrasound.kicad_pcb",
    file_version: 20_260_206,
    copper_layer_count: 4,
    nets: 506,
    segments: 2214,
    arcs: 0,
    vias: 244,
    pads: 1525,
    custom_pads: 1,
    outline_items: 1,
    keepouts: 16,
    pads_on_track_ends: 810,
    pads_outside_outline: 14,
  },
  BoardExpectation {
    name: "video-v10.kicad_pcb",
    file_version: 20_260_206,
    copper_layer_count: 4,
    nets: 589,
    segments: 7932,
    arcs: 0,
    vias: 808,
    pads: 2118,
    custom_pads: 0,
    outline_items: 22,
    keepouts: 0,
    pads_on_track_ends: 767,
    pads_outside_outline: 0,
  },
];

/// What one regression case must parse to.
struct CaseExpectation {
  /// The case directory name.
  name: &'static str,
  /// The log file inside it. Eight cases call it `pns.log`, three do not.
  log_file: &'static str,
  /// Total recorded events.
  events: usize,
  /// `EVT_START_ROUTE` events.
  start_route: usize,
  /// `EVT_START_DRAG` events.
  start_drag: usize,
  /// `EVT_FIX` events.
  fix: usize,
  /// `EVT_MOVE` events.
  moves: usize,
  /// `EVT_UNFIX` events.
  unfix: usize,
  /// Items in the golden added set.
  added: usize,
  /// KIIDs in the golden removed set.
  removed: usize,
  /// Head items, which KiCad records and never reads back.
  head: usize,
  /// Whether the log names its board by content hash.
  has_board_hash: bool,
  /// The recorded test case type, when there is one.
  test_case_type: Option<TestCaseType>,
  /// The routing algorithm the `.settings` sidecar selects.
  routing_mode: RoutingMode,
  /// Whether the sidecar restricts angles.
  restrict_angles: bool,
  /// Keys in the sidecar that today's KiCad no longer defines.
  unrecognised_setting_keys: usize,
}

/// Every regression case, with the numbers a second reader produced.
const CASES: &[CaseExpectation] = &[
  CaseExpectation {
    name: "backspace1",
    log_file: "pns.log",
    events: 334,
    start_route: 1,
    start_drag: 0,
    fix: 10,
    moves: 314,
    unfix: 9,
    added: 2,
    removed: 0,
    head: 1,
    has_board_hash: true,
    test_case_type: Some(TestCaseType::StrictGeometry),
    routing_mode: RoutingMode::Shove,
    restrict_angles: false,
    unrecognised_setting_keys: 0,
  },
  CaseExpectation {
    name: "drag-acute-fallback",
    log_file: "pns.log",
    events: 858,
    start_route: 0,
    start_drag: 1,
    fix: 0,
    moves: 857,
    unfix: 0,
    added: 11,
    removed: 6,
    head: 1,
    has_board_hash: true,
    test_case_type: None,
    routing_mode: RoutingMode::Walkaround,
    restrict_angles: true,
    unrecognised_setting_keys: 1,
  },
  CaseExpectation {
    name: "drag-walk-optimize-a",
    log_file: "drag-walk-optimize-a.log",
    events: 50,
    start_route: 0,
    start_drag: 1,
    fix: 0,
    moves: 49,
    unfix: 0,
    added: 9,
    removed: 6,
    head: 1,
    has_board_hash: true,
    test_case_type: None,
    routing_mode: RoutingMode::Walkaround,
    restrict_angles: true,
    unrecognised_setting_keys: 1,
  },
  CaseExpectation {
    name: "drag-walk-optimize-fix-corners",
    log_file: "drag-walk-optimize-fix-corners.log",
    events: 83,
    start_route: 0,
    start_drag: 1,
    fix: 0,
    moves: 82,
    unfix: 0,
    added: 9,
    removed: 3,
    head: 1,
    has_board_hash: true,
    test_case_type: None,
    routing_mode: RoutingMode::Walkaround,
    restrict_angles: true,
    unrecognised_setting_keys: 1,
  },
  CaseExpectation {
    name: "issue22749-shove-weird-drag-track-end",
    log_file: "pns.log",
    events: 54,
    start_route: 1,
    start_drag: 0,
    fix: 0,
    moves: 53,
    unfix: 0,
    added: 11,
    removed: 5,
    head: 1,
    has_board_hash: true,
    test_case_type: None,
    routing_mode: RoutingMode::Shove,
    restrict_angles: false,
    unrecognised_setting_keys: 0,
  },
  CaseExpectation {
    name: "issue23449-shove-lone-via-drag-crash",
    log_file: "pns.log",
    events: 103,
    start_route: 0,
    start_drag: 1,
    fix: 0,
    moves: 102,
    unfix: 0,
    added: 0,
    removed: 0,
    head: 0,
    has_board_hash: true,
    test_case_type: None,
    routing_mode: RoutingMode::Shove,
    restrict_angles: false,
    unrecognised_setting_keys: 0,
  },
  CaseExpectation {
    name: "issue24132-shove-same-net-via",
    log_file: "pns.log",
    events: 344,
    start_route: 1,
    start_drag: 0,
    fix: 1,
    moves: 342,
    unfix: 0,
    added: 4,
    removed: 1,
    head: 1,
    has_board_hash: true,
    test_case_type: Some(TestCaseType::StrictGeometry),
    routing_mode: RoutingMode::Shove,
    restrict_angles: false,
    unrecognised_setting_keys: 0,
  },
  CaseExpectation {
    name: "simple-drag-shove-singlelayer",
    log_file: "pns.log",
    events: 35,
    start_route: 0,
    start_drag: 1,
    fix: 0,
    moves: 34,
    unfix: 0,
    added: 134,
    removed: 139,
    head: 1,
    has_board_hash: true,
    test_case_type: None,
    routing_mode: RoutingMode::Shove,
    restrict_angles: false,
    unrecognised_setting_keys: 0,
  },
  CaseExpectation {
    name: "simple-shove-1",
    log_file: "pns.log",
    events: 27,
    start_route: 1,
    start_drag: 0,
    fix: 0,
    moves: 26,
    unfix: 0,
    added: 28,
    removed: 13,
    head: 1,
    has_board_hash: true,
    test_case_type: Some(TestCaseType::StrictGeometry),
    routing_mode: RoutingMode::Shove,
    restrict_angles: false,
    unrecognised_setting_keys: 0,
  },
  CaseExpectation {
    name: "walk-with-teardrops",
    log_file: "pns-no-hug-2.log",
    events: 18,
    start_route: 0,
    start_drag: 1,
    fix: 0,
    moves: 17,
    unfix: 0,
    added: 17,
    removed: 19,
    head: 1,
    has_board_hash: false,
    test_case_type: Some(TestCaseType::StrictGeometry),
    routing_mode: RoutingMode::Walkaround,
    restrict_angles: false,
    unrecognised_setting_keys: 0,
  },
  CaseExpectation {
    name: "walk_drag_seg_against_board_edge",
    log_file: "pns.log",
    events: 18,
    start_route: 0,
    start_drag: 1,
    fix: 0,
    moves: 17,
    unfix: 0,
    added: 3,
    removed: 3,
    head: 1,
    has_board_hash: true,
    test_case_type: Some(TestCaseType::StrictGeometry),
    routing_mode: RoutingMode::Shove,
    restrict_angles: false,
    unrecognised_setting_keys: 0,
  },
];

/// Read every board in the pool once, in the order of [`BOARDS`].
fn read_all_boards() -> Vec<KicadBoard> {
  let directory = corpus_root().join("boards");
  BOARDS
    .iter()
    .map(|expected| {
      let path = directory.join(expected.name);
      read_board(expected.name, &read_fixture(&path)).unwrap_or_else(|error| {
        panic!("cannot read {}: {error}", path.display())
      })
    })
    .collect()
}

#[test]
fn sexpr_reader_round_trips_a_hand_written_board() {
  // A miniature of the real thing: an unquoted symbol, a quoted string
  // with an escaped quote inside it, negative and fractional numbers, an
  // empty list and three levels of nesting.
  let input = "(kicad_pcb\n\
     \t(version 20260206)\n\
     \t(generator \"pcb\\\"new\")\n\
     \t(segment\n\
     \t\t(start -1.5 0.075)\n\
     \t\t(end 0 -0.0625)\n\
     \t\t(layer \"F.Cu\")\n\
     \t\t(empty)\n\
     \t)\n\
     )\n";

  let first = sexpr::parse(input).expect("the hand written board parses");
  assert_eq!(first.tag(), Some("kicad_pcb"));
  assert_eq!(
    first
      .required_child("generator")
      .unwrap()
      .value_str(0)
      .unwrap(),
    "pcb\"new",
    "the escaped quote survives"
  );

  let rewritten = first.write_to_string();
  let second = sexpr::parse(&rewritten).expect("the rewritten text parses");
  assert!(
    first.structurally_equal(&second),
    "round trip changed the tree: {rewritten}"
  );
  assert!(
    second.structurally_equal(&first),
    "structural equality is not symmetric"
  );

  // A one line rewrite moves everything onto line 1, which is exactly why
  // structural equality has to ignore line numbers.
  let segment = first.required_child("segment").unwrap();
  assert_eq!(segment.line(), 4, "line numbers follow the original text");
  assert_eq!(second.required_child("segment").unwrap().line(), 1);

  // An error names the line the bad token is on, not the line the reader
  // happened to stop at.
  let error = sexpr::parse("(a\n(b\n(c 1 2)\n)\n").unwrap_err();
  assert_eq!(error.line, 1, "the unterminated list is the outer one");
  assert!(error.message.contains("unterminated"), "{error}");
}

#[test]
fn millimetres_convert_to_nanometres_exactly() {
  let cases: &[(&str, i64)] = &[
    ("0", 0),
    ("-0", 0),
    ("1", 1_000_000),
    ("0.075", 75_000),
    ("-6.35", -6_350_000),
    ("117.95", 117_950_000),
    ("122.143198", 122_143_198),
    // Trailing zeros past the sixth decimal are not extra precision.
    ("1.0000000", 1_000_000),
    (".5", 500_000),
  ];
  for (text, expected) in cases {
    assert_eq!(
      sexpr::millimetres_to_nanometres(text, 1),
      Ok(*expected),
      "`{text}` should be {expected} nanometres"
    );
  }

  // 0.2083333333 is a real value in the corpus, but only ever as a
  // roundrect ratio, never as a coordinate. Reading it as a length is a
  // reader bug and has to say so rather than round.
  assert!(sexpr::millimetres_to_nanometres("0.2083333333", 7).is_err());
  assert!(sexpr::millimetres_to_nanometres("nope", 1).is_err());
  assert!(sexpr::millimetres_to_nanometres("1.2.3", 1).is_err());
  assert_eq!(
    sexpr::millimetres_to_nanometres("x", 42).unwrap_err().line,
    42
  );
}

#[test]
fn every_board_parses_with_the_expected_counts() {
  for (expected, board) in BOARDS.iter().zip(read_all_boards()) {
    let name = expected.name;
    assert_eq!(board.file_version, expected.file_version, "{name} version");
    assert_eq!(
      board.copper_layer_count, expected.copper_layer_count,
      "{name} copper layers"
    );
    assert_eq!(board.nets.len(), expected.nets, "{name} nets");
    assert_eq!(board.segments.len(), expected.segments, "{name} segments");
    assert_eq!(board.arcs.len(), expected.arcs, "{name} arcs");
    assert_eq!(board.vias.len(), expected.vias, "{name} vias");
    assert_eq!(board.pads.len(), expected.pads, "{name} pads");
    assert_eq!(
      board.board_outline.len(),
      expected.outline_items,
      "{name} outline items"
    );
    assert_eq!(board.keepouts.len(), expected.keepouts, "{name} keepouts");

    let custom = board
      .pads
      .iter()
      .filter(|pad| matches!(pad.shape, PadShape::Custom { .. }))
      .count();
    assert_eq!(custom, expected.custom_pads, "{name} custom pads");

    // Net 0 is the unconnected net in every board, whether the table was
    // read from the file or synthesised from inline names.
    assert_eq!(board.nets[0].number, 0, "{name} first net number");
    assert!(board.nets[0].name.is_empty(), "{name} first net name");

    // The copper index is the PNS layer index, so the outer layers sit at
    // the two ends of the range whatever the file numbers them.
    assert_eq!(board.copper_index("F.Cu"), Some(0), "{name} F.Cu index");
    assert_eq!(
      board.copper_index("B.Cu"),
      Some(expected.copper_layer_count - 1),
      "{name} B.Cu index"
    );
    assert_eq!(
      board.copper_index("F.Mask"),
      None,
      "{name} F.Mask is not copper"
    );

    assert_eq!(
      board.has_unsupported_items(),
      expected.arcs > 0,
      "{name} unsupported item flag follows the arc count"
    );
  }
}

#[test]
fn every_pad_lands_where_the_footprint_transform_puts_it() {
  for (expected, board) in BOARDS.iter().zip(read_all_boards()) {
    let name = expected.name;

    // Collect the track endpoints per net. A pad centre that coincides
    // with one to the nanometre is a pad the transform placed correctly:
    // KiCad snapped that track to the pad when the board was drawn.
    let mut endpoints: Vec<(Option<usize>, Point)> = Vec::new();
    for segment in &board.segments {
      endpoints.push((segment.net, segment.start));
      endpoints.push((segment.net, segment.end));
    }
    endpoints.sort_unstable_by_key(|(net, point)| (*net, point.x, point.y));
    endpoints.dedup();

    let hits = board
      .pads
      .iter()
      .filter(|pad| {
        endpoints
          .binary_search_by_key(&(pad.net, pad.at.x, pad.at.y), |(net, p)| {
            (*net, p.x, p.y)
          })
          .is_ok()
      })
      .count();
    assert_eq!(
      hits, expected.pads_on_track_ends,
      "{name}: pads coinciding with a track end on their own net"
    );

    let Some(outline) = board.outline_bounds() else {
      panic!("{name} has no Edge.Cuts outline");
    };
    let outside = board
      .pads
      .iter()
      .filter(|pad| !outline.contains(pad.at))
      .count();
    assert_eq!(
      outside, expected.pads_outside_outline,
      "{name}: pads outside the board outline"
    );

    let generous = outline.grown(OUTLINE_MARGIN_NANOMETRES);
    for pad in &board.pads {
      assert!(
        generous.contains(pad.at),
        "{name}: pad {} of {} is at ({}, {}), far outside {outline:?}",
        pad.number,
        pad.footprint_reference,
        pad.at.x,
        pad.at.y
      );
    }
  }
}

#[test]
fn the_pad_transform_matches_a_worked_example() {
  // boards/shove_same_net_via.kicad_pcb has two resistors, both written
  // `(at <x> <y> 90)`, whose pads sit at local (-0.9125 0) and
  // (0.9125 0). A 90 degree rotation therefore has to send local
  // (-0.9125, 0) to (0, +0.9125), which is the whole of the rotation
  // convention in one board. The file confirms it independently: a track
  // on net B ends at (114.7 74.9625), which is the first footprint's
  // origin plus that offset.
  let path = corpus_root().join("boards/shove_same_net_via.kicad_pcb");
  let board = read_board("shove_same_net_via.kicad_pcb", &read_fixture(&path))
    .expect("the board parses");

  let mut placed: Vec<(&str, i64, i64)> = board
    .pads
    .iter()
    .map(|pad| {
      (
        board.net_name(pad.net.expect("every pad here has a net")),
        pad.at.x,
        pad.at.y,
      )
    })
    .collect();
  placed.sort_unstable();
  assert_eq!(
    placed,
    vec![
      ("A", 114_700_000, 73_137_500),
      ("A", 124_000_000, 73_087_500),
      ("B", 114_700_000, 74_962_500),
      ("B", 124_000_000, 74_912_500),
    ]
  );

  let pad = board
    .pads
    .iter()
    .find(|pad| {
      pad.at
        == Point {
          x: 114_700_000,
          y: 74_962_500,
        }
    })
    .expect("the pad the net B track ends on");
  assert_eq!(pad.number, "1");
  assert!(
    (pad.rotation_degrees - 90.0).abs() < f64::EPSILON,
    "the pad angle in the file is the absolute one"
  );
  assert_eq!(pad.kind, PadKind::SurfaceMount);
  assert!(matches!(pad.shape, PadShape::RoundedRectangle { .. }));
  assert_eq!(pad.drill, None);
  assert_eq!(pad.copper_layers, vec![0], "an SMD pad on F.Cu only");

  assert!(
    board.segments.iter().any(|segment| {
      segment.net == pad.net
        && (segment.start == pad.at || segment.end == pad.at)
    }),
    "a track on net B ends on the pad"
  );

  let via = &board.vias[0];
  assert_eq!(via.copper_layer_top, 0);
  assert_eq!(via.copper_layer_bottom, 1);
  assert_eq!(via.size, 600_000);
  assert_eq!(via.drill, 300_000);
  assert!(via.free, "the board's single via carries `(free yes)`");
}

#[test]
fn custom_pad_primitives_come_out_in_board_coordinates() {
  // boards/stickhub-extra-via.kicad_pcb: a `connect custom` pad whose
  // three primitive triangles span exactly the same interval in X as its
  // rectangular anchor, which is what shows the primitive points are
  // relative to the pad and not to the footprint origin.
  let path = corpus_root().join("boards/stickhub-extra-via.kicad_pcb");
  let board = read_board("stickhub-extra-via.kicad_pcb", &read_fixture(&path))
    .expect("the board parses");

  let pad = board
    .pads
    .iter()
    .find(|pad| matches!(pad.shape, PadShape::Custom { .. }))
    .expect("the board has a custom pad");
  assert_eq!(pad.kind, PadKind::EdgeConnector);

  let PadShape::Custom { primitives } = &pad.shape else {
    unreachable!("just matched");
  };
  assert_eq!(primitives.len(), 3, "three triangles");
  for polygon in primitives {
    assert_eq!(polygon.len(), 3);
  }

  let left = primitives
    .iter()
    .flatten()
    .map(|point| point.x)
    .min()
    .expect("the polygons have points");
  let right = primitives
    .iter()
    .flatten()
    .map(|point| point.x)
    .max()
    .expect("the polygons have points");
  assert_eq!(
    (left, right),
    (pad.at.x - pad.size.x / 2, pad.at.x + pad.size.x / 2),
    "the primitives span the anchor rectangle exactly"
  );
}

#[test]
fn keepouts_carry_their_flags_and_absolute_outlines() {
  // boards/ultrasound.kicad_pcb is the only board with rule areas: one
  // inside a footprint that forbids tracks, and fifteen placement areas
  // that forbid nothing.
  let path = corpus_root().join("boards/ultrasound.kicad_pcb");
  let board = read_board("ultrasound.kicad_pcb", &read_fixture(&path))
    .expect("the board parses");

  assert_eq!(board.keepouts.len(), 16);
  let blocking: Vec<_> = board
    .keepouts
    .iter()
    .filter(|area| !area.tracks_allowed)
    .collect();
  assert_eq!(blocking.len(), 1, "one area actually forbids tracks");

  let area = blocking[0];
  assert!(!area.footprints_allowed);
  assert!(area.vias_allowed);
  assert_eq!(area.layer_names, vec!["F.Cu".to_string()]);
  assert_eq!(area.copper_layers, vec![0]);
  assert_eq!(
    area.outline.first(),
    Some(&Point {
      x: 93_200_000,
      y: 62_675_000
    }),
    "the outline of a footprint's rule area is already absolute"
  );

  // Every outline point of every area is inside the board, which it would
  // not be if a footprint transform had been applied to it by mistake.
  let outline = board.outline_bounds().expect("the board has an outline");
  let generous = outline.grown(OUTLINE_MARGIN_NANOMETRES);
  for area in &board.keepouts {
    for point in &area.outline {
      assert!(
        generous.contains(*point),
        "keepout point {point:?} is off board"
      );
    }
  }
}

#[test]
fn the_corpus_holds_exactly_the_expected_cases() {
  let cases =
    pns_log::discover_cases(&corpus_root()).expect("the corpus is readable");
  let found: Vec<(String, String)> = cases
    .iter()
    .map(|case| {
      (
        case.name.clone(),
        case
          .log_path
          .file_name()
          .expect("a log has a file name")
          .to_string_lossy()
          .into_owned(),
      )
    })
    .collect();
  let expected: Vec<(String, String)> = CASES
    .iter()
    .map(|case| (case.name.to_string(), case.log_file.to_string()))
    .collect();
  assert_eq!(found, expected);

  // Exactly one case carries its own board rather than naming one from
  // the pool by hash, and exactly one ships custom design rules.
  let with_dump: Vec<&str> = cases
    .iter()
    .filter(|case| case.board_dump_path.is_some())
    .map(|case| case.name.as_str())
    .collect();
  assert_eq!(with_dump, vec!["walk-with-teardrops"]);
  let with_rules: Vec<&str> = cases
    .iter()
    .filter(|case| case.design_rules_path.is_some())
    .map(|case| case.name.as_str())
    .collect();
  assert_eq!(with_rules, vec!["issue24132-shove-same-net-via"]);
  assert!(cases.iter().all(|case| case.settings_path.is_some()));
}

#[test]
fn every_case_log_parses_with_the_expected_events() {
  let cases =
    pns_log::discover_cases(&corpus_root()).expect("the corpus is readable");
  for (expected, case) in CASES.iter().zip(&cases) {
    let name = expected.name;
    let log = pns_log::read_log(&read_fixture(&case.log_path))
      .unwrap_or_else(|error| panic!("{name}: {error}"));

    assert_eq!(
      log.router_mode,
      RouterMode::RouteSingle,
      "{name}: the whole corpus is single track routing"
    );
    assert_eq!(log.test_case_type, expected.test_case_type, "{name} type");
    assert_eq!(
      log.board_hash.is_some(),
      expected.has_board_hash,
      "{name} board hash"
    );
    assert_eq!(log.events.len(), expected.events, "{name} events");

    let count = |kind: EventKind| {
      log.events.iter().filter(|event| event.kind == kind).count()
    };
    assert_eq!(count(EventKind::StartRoute), expected.start_route, "{name}");
    assert_eq!(count(EventKind::StartDrag), expected.start_drag, "{name}");
    assert_eq!(count(EventKind::Fix), expected.fix, "{name}");
    assert_eq!(count(EventKind::Move), expected.moves, "{name}");
    assert_eq!(count(EventKind::Unfix), expected.unfix, "{name}");
    assert_eq!(
      count(EventKind::Abort)
        + count(EventKind::ToggleVia)
        + count(EventKind::StartMultiDrag),
      0,
      "{name}: the corpus exercises none of these three"
    );

    assert_eq!(log.added_items.len(), expected.added, "{name} added");
    assert_eq!(log.removed_items.len(), expected.removed, "{name} removed");
    assert_eq!(log.head_items.len(), expected.head, "{name} head");

    // Every event carries the decorative sizes block, and the golden
    // items name their nets by name rather than by number.
    assert!(
      log.events.iter().all(|event| event.sizes.is_some()),
      "{name}"
    );
    for item in &log.added_items {
      assert!(
        !item.net.is_empty(),
        "{name}: an added item has no net name"
      );
    }
  }
}

#[test]
fn every_case_settings_file_parses() {
  let cases =
    pns_log::discover_cases(&corpus_root()).expect("the corpus is readable");
  for (expected, case) in CASES.iter().zip(&cases) {
    let name = expected.name;
    let path = case
      .settings_path
      .as_ref()
      .expect("every case has settings");
    let settings = pns_log::read_settings(&read_fixture(path))
      .unwrap_or_else(|error| panic!("{name}: {error}"));

    assert_eq!(settings.mode, expected.routing_mode, "{name} routing mode");
    assert_eq!(
      settings.restrict_angles, expected.restrict_angles,
      "{name} restrict_angles"
    );
    assert_eq!(
      settings.unrecognised_keys.len(),
      expected.unrecognised_setting_keys,
      "{name} unrecognised keys: {:?}",
      settings.unrecognised_keys
    );
    assert_eq!(
      settings.corner_mode,
      CornerMode::Mitered45,
      "{name}: no case changes the corner mode"
    );
    assert!(
      !settings.can_violate_drc,
      "{name}: no case allows a rule violation"
    );
  }

  // The one obsolete key in the corpus is `pad_pushout`, dropped from
  // PNS::ROUTING_SETTINGS but still written into three of the sidecars.
  let path = corpus_root().join("drag-acute-fallback/pns.settings");
  let settings = pns_log::read_settings(&read_fixture(&path)).unwrap();
  assert_eq!(settings.unrecognised_keys, vec!["pad_pushout".to_string()]);
}

#[test]
fn the_golden_of_the_via_case_holds_a_via_and_its_hole() {
  // issue24132-shove-same-net-via is the only case whose golden contains
  // anything but segments, and the only one shipping a .kicad_dru.
  let path = corpus_root().join("issue24132-shove-same-net-via/pns.log");
  let log = pns_log::read_log(&read_fixture(&path)).expect("the log parses");

  let kinds: Vec<&str> = log
    .added_items
    .iter()
    .map(|item| item.kind.as_str())
    .collect();
  assert_eq!(kinds, vec!["segment", "via", "hole", "segment"]);

  let via = &log.added_items[1];
  assert!(via.drill.is_some(), "a logged via carries its drill");
  let Some(LogShape::Circle { radius, .. }) = via.shape else {
    panic!("a logged via is a circle, got {:?}", via.shape);
  };
  assert!(radius > 0);

  let hole = &log.added_items[2];
  assert_eq!(hole.drill, None, "a hole logs no drill of its own");
  assert!(matches!(hole.shape, Some(LogShape::Circle { .. })));

  // Head items are `line` kind and carry no shape at all, which is why
  // KiCad never reads them back.
  assert_eq!(log.head_items[0].kind, "line");
  assert!(log.head_items[0].shape.is_none());
}

#[test]
fn the_legacy_log_grammar_parses() {
  // No file in the corpus uses it, so the only way to test the fallback
  // is a hand written input in the shape LOGGER::ParseEvent writes.
  let text = "\
# a comment line, which the grammar ignores
mode 1
event 111800000 93600000 0 0 1 ebb07c9c-0000-4000-8000-000000000000
event 111900000 93700000 3 0 0
event 0 0 6 0 0
added segment net 3 layers 0 0 shape 4 1000 2000 3000 4000 250000
added via net 7 layers 0 1 shape 2 5000 6000 300000 drill 200000
added zone net 1 layers 0 0
removed 4dbd2b6d-0000-4000-8000-000000000000
";
  let log = pns_log::read_legacy_log(text).expect("the legacy log parses");

  assert_eq!(log.router_mode, RouterMode::RouteSingle);
  assert_eq!(log.board_hash, None);
  assert_eq!(log.test_case_type, None);
  assert_eq!(log.events.len(), 3);
  assert_eq!(log.events[0].kind, EventKind::StartRoute);
  assert_eq!(
    log.events[0].position,
    Point {
      x: 111_800_000,
      y: 93_600_000
    }
  );
  assert_eq!(log.events[0].uuids.len(), 1);
  assert_eq!(log.events[1].kind, EventKind::Move);
  assert!(log.events[1].uuids.is_empty());
  assert_eq!(log.events[2].kind, EventKind::Unfix);
  assert!(
    log.events[0].sizes.is_none(),
    "the legacy event line has no sizes block"
  );

  // `zone` is an item kind the legacy grammar cannot express, so it is
  // dropped rather than guessed at.
  assert_eq!(log.added_items.len(), 2);
  assert_eq!(
    log.added_items[0].shape,
    Some(LogShape::Segment {
      start: Point { x: 1000, y: 2000 },
      end: Point { x: 3000, y: 4000 },
      width: 250_000,
    })
  );
  assert_eq!(
    log.added_items[0].net, "3",
    "the legacy format names a net by netcode, so it stays as text"
  );
  assert_eq!(log.added_items[1].drill, Some(200_000));
  assert_eq!(log.removed_items.len(), 1);

  // The dispatcher tries JSON first and only then the legacy grammar,
  // which is the order PNS_LOG_FILE::Load uses.
  assert_eq!(pns_log::read_log(text).unwrap().events.len(), 3);
  assert!(pns_log::read_log("not a log at all").is_err());
}

#[test]
fn board_coordinates_keep_the_files_downward_y_axis() {
  // A track that runs from a smaller Y to a larger one in the file has to
  // come out running downwards, not upwards: nothing here flips the axis.
  let path = corpus_root().join("boards/drag-walk-optimize.kicad_pcb");
  let board = read_board("drag-walk-optimize.kicad_pcb", &read_fixture(&path))
    .expect("the board parses");

  let segment = board
    .segments
    .iter()
    .find(|segment| {
      segment.start
        == Point {
          x: 137_500_000,
          y: 110_000_000,
        }
    })
    .expect("the (137.5 110) -> (146.5 101) track is in the file");
  assert_eq!(
    segment.end,
    Point {
      x: 146_500_000,
      y: 101_000_000
    }
  );
  assert_eq!(segment.width, 500_000);
  assert_eq!(segment.copper_layer, 0);

  // The two test point pads sit at the two ends of that track run, in
  // footprints written with the newer (transform ...) spelling.
  let mut anchors: Vec<Point> = board
    .pads
    .iter()
    .filter(|pad| pad.shape == PadShape::Circle)
    .map(|pad| pad.at)
    .collect();
  anchors.sort_unstable_by_key(|point| (point.x, point.y));
  assert_eq!(
    anchors,
    vec![
      Point {
        x: 137_500_000,
        y: 110_000_000
      },
      Point {
        x: 169_000_000,
        y: 101_000_000
      },
    ]
  );
}

#[test]
fn the_board_bounding_box_helper_behaves() {
  let mut bounds = kicad_pcb::BoundingBox::around(Point { x: 10, y: -20 });
  assert!(bounds.contains(Point { x: 10, y: -20 }));
  bounds.include(Point { x: -5, y: 40 });
  assert_eq!(bounds.left, -5);
  assert_eq!(bounds.top, -20);
  assert_eq!(bounds.right, 10);
  assert_eq!(bounds.bottom, 40);
  assert!(bounds.contains(Point { x: 0, y: 0 }));
  assert!(!bounds.contains(Point { x: 11, y: 0 }));
  assert!(bounds.grown(1).contains(Point { x: 11, y: 41 }));
}
