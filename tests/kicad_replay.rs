// SPDX-License-Identifier: GPL-3.0-or-later

//! Replaying KiCad's PNS regression corpus against this crate.
//!
//! Each case under `tests/fixtures/kicad/pns_regressions` is one recorded
//! routing session: a `.log` of input events plus the added and removed
//! items KiCad's own router answered with. `tests/support/kicad_snapshot`
//! turns the board into a [`pnsrouter::snapshot::WorldSnapshot`] and a rule resolver,
//! `tests/support/kicad_replay` drives a [`pnsrouter::router::Router`]
//! from the events, and the tests below judge the result in tiers. Note
//! 05 section 6.10 proposes exactly this and names the tiers:
//!
//! 1. **The session terminates and does not panic**, and nothing it left
//!    behind violates the rules it ran under. Asserted for every case
//!    whose events this crate can replay.
//! 2. **The counts of added and removed items equal the golden stored in
//!    the log.** All four replayable cases reach it. Should one stop, the
//!    convention is to `#[ignore]` its tier 2 test with the measured and
//!    the expected numbers in the reason string, so
//!    `cargo test -- --ignored` still shows the difference instead of the
//!    suite quietly agreeing with itself.
//!
//! Exact geometry, note 05's tier 3, is deliberately not attempted. The
//! golden is a multiset of vertices produced by a different
//! implementation running under KiCad's own DRC engine; two shove routers
//! that both answer correctly will not agree on it.
//!
//! # Which board each case routes on
//!
//! A `pns.log` names its board by content hash and not by name: the
//! producer stamps `board_hash` with `IO_UTILS::fileHashMMH3` over the
//! saved board (`pcbnew/router/router_tool.cpp:894`) and KiCad's harness
//! hashes every `boards/*.kicad_pcb` at startup to match
//! (`qa/tools/pns/qa_pns_regressions_main.cpp:166`). That hash function
//! is outside the reference checkout, so the mapping below was recovered
//! from the corpus itself instead: for every case, the set of KIIDs its
//! events and its `removedItems` refer to was intersected with the KIIDs
//! of every board in the pool. Nine of the eleven cases have exactly one
//! board that contains all of theirs.
//!
//! Two need a word.
//!
//! - `backspace1` refers to a single KIID, a pad of footprint `U1`, and
//!   both `boards/backspace1.kicad_pcb` and `boards/dp_test.kicad_pcb`
//!   contain it. The two files are the same board saved in two format
//!   eras: identical KIID sets, identical counts of segments, vias,
//!   footprints and pads, and the same footprint origin, so either
//!   produces the same world. The file whose name is the case's name is
//!   used.
//! - `walk-with-teardrops` ships its own board next to the log as
//!   `pns-no-hug-2.dump`, a `.kicad_pcb` payload under a `.dump`
//!   extension, and carries no `board_hash` at all
//!   (`qa/tools/pns/pns_log_file.cpp:484`). It maps to that file.
//!
//! Every routable case asserts its own half of this: a mapping that
//! resolves every KIID the log refers to is what
//! `support::kicad_replay::unresolved_uuids` checks, and a case matched
//! to the wrong board would fail it.
//!
//! # What is not replayed
//!
//! Seven of the eleven cases open with `EVT_START_DRAG` or
//! `EVT_START_MULTIDRAG`. The crate has no dragger yet (`PLAN.md`
//! milestone 8), so those tests are `#[ignore]`d with that reason and
//! their bodies check only that the case loads and that its board mapping
//! resolves.
//!
//! # The rules a case is replayed under
//!
//! `support::kicad_snapshot::KicadRules` resolves the `.kicad_pro`'s net
//! classes and board minima, and, for the one case that ships a
//! `.kicad_dru`, the subset of KiCad's rule language
//! `support::kicad_dru` models. A rule outside that subset is dropped and
//! raises `KicadRules::has_unsupported_design_rules`, which is the flag
//! an ignore reason would cite; nothing in the corpus trips it today.

#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

/// The fixture readers and the conversion under test.
mod support;

use std::path::{Path, PathBuf};

use pnsrouter::eventlog::{SessionRecording, assert_replay_matches};
use pnsrouter::rules::RuleResolver;

use support::kicad_replay::{
  EventOutcome, LoadedCase, ReplayReport, replay_case, unresolved_uuids,
};
use support::pns_log::{RegressionCase, discover_cases};

/// The root of the copied corpus.
fn corpus_root() -> PathBuf {
  Path::new(env!("CARGO_MANIFEST_DIR"))
    .join("tests/fixtures/kicad/pns_regressions")
}

/// Where the recorded session fixtures live.
fn session_root() -> PathBuf {
  Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/sessions")
}

/// The board each case routes on, relative to the corpus root.
///
/// Recovered by KIID intersection; see the module documentation.
const BOARDS: &[(&str, &str)] = &[
  ("backspace1", "boards/backspace1.kicad_pcb"),
  ("drag-acute-fallback", "boards/drag-walk-optimize.kicad_pcb"),
  (
    "drag-walk-optimize-a",
    "boards/drag-walk-optimize.kicad_pcb",
  ),
  (
    "drag-walk-optimize-fix-corners",
    "boards/drag-walk-optimize-fix-corners.kicad_pcb",
  ),
  (
    "issue22749-shove-weird-drag-track-end",
    "boards/pic_programmer.kicad_pcb",
  ),
  (
    "issue23449-shove-lone-via-drag-crash",
    "boards/stickhub-extra-via.kicad_pcb",
  ),
  (
    "issue24132-shove-same-net-via",
    "boards/shove_same_net_via.kicad_pcb",
  ),
  (
    "simple-drag-shove-singlelayer",
    "boards/video-v10.kicad_pcb",
  ),
  ("simple-shove-1", "boards/simple.kicad_pcb"),
  (
    "walk-with-teardrops",
    "walk-with-teardrops/pns-no-hug-2.dump",
  ),
  (
    "walk_drag_seg_against_board_edge",
    "boards/ultrasound.kicad_pcb",
  ),
];

/// The regression case of that name, as `discover_cases` found it.
fn find_case(name: &str) -> RegressionCase {
  let root = corpus_root();
  let cases = discover_cases(&root)
    .unwrap_or_else(|error| panic!("cannot walk {}: {error}", root.display()));

  cases
    .into_iter()
    .find(|case| case.name == name)
    .unwrap_or_else(|| panic!("the corpus has no case called `{name}`"))
}

/// Load a case together with the board the table above maps it to.
fn load(name: &str) -> LoadedCase {
  let case = find_case(name);
  let board = BOARDS
    .iter()
    .find(|(case_name, _)| *case_name == name)
    .map(|(_, board)| corpus_root().join(board))
    .unwrap_or_else(|| panic!("no board is mapped to `{name}`"));

  LoadedCase::load(&case, &board)
    .unwrap_or_else(|error| panic!("cannot load `{name}`: {error}"))
}

/// Every KIID the log refers to has to resolve against the mapped board.
///
/// This is the mapping assertion: a case pointed at the wrong board would
/// start its route from nothing and every tier below it would be
/// meaningless.
fn assert_board_mapping(case: &LoadedCase) {
  let (_, host_map) =
    support::kicad_snapshot::snapshot_from_board(&case.board, &case.rules);
  let missing = unresolved_uuids(case, &host_map);

  assert!(
    missing.is_empty(),
    "`{}` refers to {} object(s) that `{}` does not hold, the first being \
     {}; the board mapping is wrong",
    case.name,
    missing.len(),
    case.board_source,
    missing[0]
  );
}

/// Tier 1: the session ran to the end and left nothing colliding.
fn assert_tier_one(report: &ReplayReport) {
  assert!(
    report.unsupported_events.is_empty(),
    "`{}` needs event kinds this harness cannot replay: {:?}",
    report.case,
    report.unsupported_events
  );

  // A refused start would leave every later event with nothing to do and
  // an empty delta, which passes the two collision checks below for
  // entirely the wrong reason.
  for outcome in &report.outcomes {
    assert!(
      !matches!(
        outcome,
        EventOutcome::StartRefused { .. } | EventOutcome::Ignored
      ),
      "`{}` did not keep a session running: {outcome:?}. Events: {}",
      report.case,
      report.outcome_summary()
    );
  }

  assert!(
    report.pending_collisions.is_empty(),
    "`{}` left {} colliding segment(s) in the live session: {}",
    report.case,
    report.pending_collisions.len(),
    report.pending_collisions[0]
  );
  assert!(
    report.committed_collisions.is_empty(),
    "`{}` committed {} colliding segment(s): {}",
    report.case,
    report.committed_collisions.len(),
    report.committed_collisions[0]
  );
}

/// Tier 2: the counts agree with the golden stored in the log.
///
/// The comparison is spelled out in the message whether it passes or
/// fails, which is what makes an `#[ignore]`d case's reason string
/// checkable against a run.
fn assert_tier_two(report: &ReplayReport) {
  assert!(
    report.matches_golden(),
    "{}. Events: {}",
    report.golden_summary(),
    report.outcome_summary()
  );
}

/// A case that cannot be replayed still has to load and to map.
fn assert_loads_and_maps(name: &str) {
  let case = load(name);

  assert_board_mapping(&case);
  assert!(
    !case.is_replayable(),
    "`{name}` is ignored as a drag case but holds no drag event"
  );
}

// ---------------------------------------------------------------------
// The four cases this crate can replay
// ---------------------------------------------------------------------
//
// Two tests per case. The tier 1 test always runs; the tier 2 test is
// `#[ignore]`d, with the numbers in its reason string, wherever the
// counts do not agree with the golden yet.

/// Backspace during routing: one start, 314 moves, ten fixes and nine
/// undos, in shove mode. The only case in the corpus that exercises
/// `UndoLastSegment`.
#[test]
fn backspace1_replays_without_a_violation() {
  let case = load("backspace1");

  assert_board_mapping(&case);
  assert_tier_one(&replay_case(&case));
}

/// Tier 2 for the case above.
#[test]
fn backspace1_matches_the_golden() {
  assert_tier_two(&replay_case(&load("backspace1")));
}

/// GitLab issue 22749: a shove that produced a malformed track end. One
/// start on layer 1 and 53 moves, with no fix at all, so the golden is
/// the live delta and a host would commit nothing.
#[test]
fn issue22749_shove_weird_drag_track_end_replays_without_a_violation() {
  let case = load("issue22749-shove-weird-drag-track-end");

  assert_board_mapping(&case);
  assert_tier_one(&replay_case(&case));
}

/// Tier 2 for the case above.
#[test]
fn issue22749_shove_weird_drag_track_end_matches_the_golden() {
  assert_tier_two(&replay_case(&load("issue22749-shove-weird-drag-track-end")));
}

/// GitLab issue 24132: shoving a via on the same net. The only case with
/// a via and a hole in its golden, and the only one shipping a
/// `.kicad_dru`.
#[test]
fn issue24132_shove_same_net_via_replays_without_a_violation() {
  let case = load("issue24132-shove-same-net-via");

  assert_board_mapping(&case);
  assert_tier_one(&replay_case(&case));
}

/// Tier 2 for the case above, and the only one that depends on a
/// `.kicad_dru`.
///
/// The case ships one, and its single rule declares a net blind
/// `physical_clearance` of 2 mm between a track and a via. That is what
/// makes the golden four added items and one removed: the rule reaches
/// the pair even though the route is on the via's own net, so the shove
/// moves the board's one via aside and the commit is the moved via, the
/// hole that came with it and two segments, against the via it replaced.
/// Two things have to line up for that, and this test is where both are
/// checked end to end: the ladder folds the physical rung in outside its
/// same net guard (`pcbnew/router/pns_kicad_iface.cpp:960`, `:968`), and
/// `RuleResolver::has_user_defined_physical_constraint` answers true so
/// the collision code does not take the same net short circuit first
/// (`pcbnew/router/pns_item.cpp:127`, `:188`).
#[test]
fn issue24132_shove_same_net_via_matches_the_golden() {
  assert_tier_two(&replay_case(&load("issue24132-shove-same-net-via")));
}

/// A route started in free space, so the first event carries no uuid at
/// all and the layer comes from the event rather than from an item.
#[test]
fn simple_shove_1_replays_without_a_violation() {
  let case = load("simple-shove-1");

  assert_board_mapping(&case);
  assert_tier_one(&replay_case(&case));
}

/// Tier 2 for the case above, and the only replayable case that
/// exercises the net a free space route carries.
///
/// `LINE_PLACER::Start` gives such a route
/// `ROUTER_IFACE::GetOrphanedNetHandle()`
/// (`pcbnew/router/pns_line_placer.cpp:1386`), a real handle whose net
/// code is not positive, which
/// [`pnsrouter::rules::RuleResolver::orphaned_net`] ports. It is what
/// makes the head and the tail this session has already fixed "same net"
/// (`pcbnew/router/pns_item.cpp:188`), so the shove pushes the tracks
/// in the way instead of refusing. A null net there collides with the
/// route's own tail, the shove refuses, and the placer falls back to the
/// walkaround (`:1012`), which walks around the obstacles and commits
/// nothing at all.
#[test]
fn simple_shove_1_matches_the_golden() {
  assert_tier_two(&replay_case(&load("simple-shove-1")));
}

// ---------------------------------------------------------------------
// The seven drag cases
// ---------------------------------------------------------------------

/// The longest log in the corpus, 858 events of one drag.
#[test]
#[ignore = "dragging is milestone 8"]
fn drag_acute_fallback() {
  assert_loads_and_maps("drag-acute-fallback");
}

/// Post drag walkaround optimiser behaviour.
#[test]
#[ignore = "dragging is milestone 8"]
fn drag_walk_optimize_a() {
  assert_loads_and_maps("drag-walk-optimize-a");
}

/// The corner fixing variant of the case above.
#[test]
#[ignore = "dragging is milestone 8"]
fn drag_walk_optimize_fix_corners() {
  assert_loads_and_maps("drag-walk-optimize-fix-corners");
}

/// GitLab issue 23449, a crash dragging an isolated via. Its golden is
/// empty, so it is a free win at tier 1 once the dragger exists.
#[test]
#[ignore = "dragging is milestone 8"]
fn issue23449_shove_lone_via_drag_crash() {
  assert_loads_and_maps("issue23449-shove-lone-via-drag-crash");
}

/// A heavy single layer shove cascade across ten nets, by far the largest
/// golden in the corpus.
#[test]
#[ignore = "dragging is milestone 8"]
fn simple_drag_shove_singlelayer() {
  assert_loads_and_maps("simple-drag-shove-singlelayer");
}

/// Walking around teardrop pads with hugging disabled. The only self
/// contained case, shipping its own board.
#[test]
#[ignore = "dragging is milestone 8"]
fn walk_with_teardrops() {
  assert_loads_and_maps("walk-with-teardrops");
}

/// Dragging a segment into the board outline.
#[test]
#[ignore = "dragging is milestone 8"]
fn walk_drag_seg_against_board_edge() {
  assert_loads_and_maps("walk_drag_seg_against_board_edge");
}

// ---------------------------------------------------------------------
// Golden session fixtures
// ---------------------------------------------------------------------

/// The two cases recorded into `tests/fixtures/sessions` and the file
/// each is written to.
///
/// These are the two cleanest routing sessions in the corpus: the only
/// one that backspaces and the only one that places a via. Storing them
/// as recordings gives the engine a regression fixture that does not
/// depend on the KiCad readers, which is what `DESIGN.md` section 8 asks
/// for.
const SESSION_FIXTURES: &[(&str, &str)] = &[
  ("backspace1", "kicad_backspace1.txt"),
  (
    "issue24132-shove-same-net-via",
    "kicad_shove_same_net_via.txt",
  ),
];

/// A resolver factory over one case's rules, which
/// [`assert_replay_matches`] needs because a `Box<dyn RuleResolver>`
/// cannot be cloned.
fn resolver_of(case: &LoadedCase) -> impl Fn() -> Box<dyn RuleResolver> + '_ {
  move || Box::new(case.rules.clone())
}

/// Every stored session replays to the commits stored beside it.
///
/// This is the strict tier of `tests/fixtures/sessions/README.md`, and it
/// is the drift detector for the engine: the recording carries its own
/// [`pnsrouter::snapshot::WorldSnapshot`], so it keeps answering the same way whatever the
/// KiCad readers do, and a changed commit means a changed routing
/// behaviour.
#[test]
fn the_kicad_session_fixtures_replay_to_the_commits_stored_in_them() {
  for (case_name, file) in SESSION_FIXTURES {
    let path = session_root().join(file);
    let text = std::fs::read_to_string(&path).unwrap_or_else(|error| {
      panic!("cannot read {}: {error}", path.display())
    });
    let recording = SessionRecording::from_text(&text)
      .unwrap_or_else(|error| panic!("{} does not parse: {error}", file));
    let case = load(case_name);

    assert_replay_matches(&recording, resolver_of(&case));
  }
}

/// Rewrite the session fixtures from what this crate does today.
///
/// Ignored on purpose. A changed commit diff is a change of routing
/// behaviour, not a test that needs fixing; read
/// `tests/fixtures/sessions/README.md` before running this.
///
/// ```text
/// dev/in-container.sh cargo test --test kicad_replay -- --ignored \
///   regenerate_the_kicad_session_fixtures
/// ```
#[test]
#[ignore = "rewrites golden fixtures, run it deliberately"]
fn regenerate_the_kicad_session_fixtures() {
  for (case_name, file) in SESSION_FIXTURES {
    let case = load(case_name);
    let report = replay_case(&case);
    let path = session_root().join(file);

    std::fs::write(&path, report.recording.to_text()).unwrap_or_else(|error| {
      panic!("cannot write {}: {error}", path.display())
    });
  }
}

/// Print what every replayable case measured, for the report a human
/// reads after a behaviour change.
///
/// Ignored because it asserts nothing; it exists so that the numbers in
/// the `#[ignore]` reason strings above can be refreshed from one run.
#[test]
#[ignore = "reports numbers, asserts nothing"]
fn report_the_measured_numbers_of_every_replayable_case() {
  for (name, _) in BOARDS {
    let case = load(name);

    if !case.is_replayable() {
      println!("{name}: not replayable, drag events");

      continue;
    }

    let report = replay_case(&case);

    println!("{}", report.golden_summary());
    println!("  board {}", report.board_source);
    println!("  events {}", report.outcome_summary());
    println!(
      "  committed added {} updated {} removed {}",
      report.commit.added.len(),
      report.commit.updated.len(),
      report.commit.removed.len()
    );
    println!(
      "  collisions pending {} committed {}",
      report.pending_collisions.len(),
      report.committed_collisions.len()
    );
    println!(
      "  snapshot skipped {} arc(s), {} keepout(s), {} pad(s); rules \
       unsupported {:?}",
      report.host_map.skipped_arcs,
      report.host_map.skipped_keepouts,
      report.host_map.skipped_pads,
      report.settings.unsupported
    );
  }
}

/// The snapshot of a routable case describes every kind of obstacle the
/// board holds.
///
/// A cheap guard on the conversion itself: a board with pads, tracks, a
/// via and an outline has to reach the world with all four, or a replay
/// would run in a world that is missing obstacles and pass tier 1 for the
/// wrong reason.
#[test]
fn the_snapshot_of_the_via_case_holds_every_obstacle_kind() {
  let case = load("issue24132-shove-same-net-via");
  let (snapshot, host_map) =
    support::kicad_snapshot::snapshot_from_board(&case.board, &case.rules);

  assert_eq!(host_map.skipped_arcs, 0);
  assert_eq!(host_map.skipped_keepouts, 0);
  assert_eq!(host_map.skipped_pads, 0);
  assert_eq!(
    host_map.len(),
    case.board.pads.len()
      + case.board.segments.len()
      + case.board.vias.len()
      + case.board.board_outline.len(),
    "one host object per board object"
  );
  assert_eq!(snapshot.copper_layer_count, 2);
  assert!(
    snapshot.max_clearance >= case.rules.worst_clearance(),
    "the broad phase would lose obstacles the narrow phase never sees"
  );

  // The one `(gr_rect)` outline becomes four zero width capsules, so the
  // snapshot holds more items than it does host objects.
  assert!(snapshot.items.len() > host_map.len());
}
