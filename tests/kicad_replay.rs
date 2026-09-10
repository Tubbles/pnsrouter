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
//!    whose events this crate can replay. The two halves are separate
//!    assertions, because the drag cases reached the first before they
//!    reached the second; every case reaches both now.
//! 2. **The counts of added and removed items equal the golden stored in
//!    the log.** All four routing cases reach it, and six of the seven
//!    drag cases. Should one stop, the convention is to `#[ignore]` its
//!    tier 2 test with the measured and the expected numbers in the
//!    reason string, so `cargo test -- --ignored` still shows the
//!    difference instead of the suite quietly agreeing with itself.
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
//! # The one drag case that misses its golden
//!
//! Seven of the eleven cases open with `EVT_START_DRAG`. All seven replay,
//! all seven leave nothing colliding, and six of the seven match their
//! golden.
//!
//! Two of them used to miss because of the **snapshot the harness
//! builds** rather than because of the drag, and both are fixed in
//! `support::kicad_snapshot`: a rounded rectangle pad is polygonised the
//! way `syncPad` polygonises it instead of being flattened to a sharp
//! rectangle, which is what `drag-acute-fallback` detours around, and
//! `KicadRules::clearance_epsilon` answers KiCad's 500 nm DRC epsilon
//! instead of zero, which is what sent `walk-with-teardrops` round the
//! board outline.
//!
//! The one that misses is `issue23449-shove-lone-via-drag-crash`, whose
//! golden is empty on all three counts. It matched while the via drag
//! moved nothing and stopped matching when it started to; the difference
//! is in the shove's lone via head, not in the dragger, and the case's
//! own tier 2 test says where it first appears.
//!
//! `EVT_START_MULTIDRAG` is still refused, and no case in the corpus
//! emits one.
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
  EventOutcome, LoadedCase, ReplayReport, replay_case,
  replay_case_with_parallelism, unresolved_uuids,
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

/// Tier 1, first half: every event was replayed with a session alive.
///
/// A refused start would leave every later event with nothing to do and
/// an empty delta, which passes [`assert_collision_free`] and sometimes
/// even the golden for entirely the wrong reason.
fn assert_replays(report: &ReplayReport) {
  assert!(
    report.unsupported_events.is_empty(),
    "`{}` needs event kinds this harness cannot replay: {:?}",
    report.case,
    report.unsupported_events
  );

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
}

/// Tier 1, second half: nothing the session left behind collides.
fn assert_collision_free(report: &ReplayReport) {
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

/// Tier 1: the session ran to the end and left nothing colliding.
fn assert_tier_one(report: &ReplayReport) {
  assert_replays(report);
  assert_collision_free(report);
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

// ---------------------------------------------------------------------
// The four routing cases
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

// Three tests per case. All three run wherever the drag reaches them;
// where a golden does not agree yet the tier 2 test is `#[ignore]`d with
// the measured numbers in its reason string. Every one of the seven is a
// mid segment or via drag: not one corpus case is a corner drag (note 06
// section 10.1).

/// The longest log in the corpus, 858 events of one drag, in walkaround
/// mode with `restrict_angles`.
#[test]
fn drag_acute_fallback_replays() {
  let case = load("drag-acute-fallback");

  assert_board_mapping(&case);
  assert_replays(&replay_case(&case));
}

/// Tier 1's collision half for the case above.
#[test]
fn drag_acute_fallback_leaves_nothing_colliding() {
  assert_collision_free(&replay_case(&load("drag-acute-fallback")));
}

/// Tier 2 for the case above, the golden the roundrect pad hull unlocked.
///
/// The drag detours around pad 2 of `C1`, a `0.9 x 0.95` roundrect. While
/// `support::kicad_snapshot` mapped it to a
/// [`pnsrouter::geometry::shape::Shape::Rect`] the detour hugged a square
/// corner, reached the drag anchor in one segment where KiCad needs two,
/// and came out at 10 added against the golden's 11. With the pad
/// polygonised the way `syncPad` polygonises it the two agree.
#[test]
fn drag_acute_fallback_matches_the_golden() {
  assert_tier_two(&replay_case(&load("drag-acute-fallback")));
}

/// Post drag walkaround optimiser behaviour, on the same board.
#[test]
fn drag_walk_optimize_a_replays() {
  let case = load("drag-walk-optimize-a");

  assert_board_mapping(&case);
  assert_replays(&replay_case(&case));
}

/// Tier 1's collision half for the case above.
#[test]
fn drag_walk_optimize_a_leaves_nothing_colliding() {
  assert_collision_free(&replay_case(&load("drag-walk-optimize-a")));
}

/// Tier 2 for the case above, the first golden `REQUIRE_OBTUSE_ANGLES`
/// unlocked.
#[test]
fn drag_walk_optimize_a_matches_the_golden() {
  assert_tier_two(&replay_case(&load("drag-walk-optimize-a")));
}

/// The corner fixing variant of the case above.
#[test]
fn drag_walk_optimize_fix_corners_replays() {
  let case = load("drag-walk-optimize-fix-corners");

  assert_board_mapping(&case);
  assert_replays(&replay_case(&case));
}

/// Tier 1's collision half for the case above.
#[test]
fn drag_walk_optimize_fix_corners_leaves_nothing_colliding() {
  assert_collision_free(&replay_case(&load("drag-walk-optimize-fix-corners")));
}

/// Tier 2 for the case above, and the one that names
/// `OPTIMIZER::dragFixCorners` in its own title.
#[test]
fn drag_walk_optimize_fix_corners_matches_the_golden() {
  assert_tier_two(&replay_case(&load("drag-walk-optimize-fix-corners")));
}

/// GitLab issue 23449, a crash dragging an isolated via.
#[test]
fn issue23449_shove_lone_via_drag_crash_replays_without_a_violation() {
  let case = load("issue23449-shove-lone-via-drag-crash");

  assert_board_mapping(&case);
  assert_tier_one(&replay_case(&case));
}

/// Tier 2 for the case above, which the via drag turned from an accident
/// into a real disagreement.
///
/// The log holds one `EVT_START_DRAG` on a lone via and 102 moves, and
/// KiCad answered with **nothing**: `addedItems`, `removedItems` and
/// `headItems` are all empty. It matched while the via drag was a stub
/// that moved nothing, which note 06 section 10.2 step 9 predicted would
/// not last ("at tier 2 only if the crate's drag also ends with an empty
/// delta"). It does not: this crate now drags the via.
///
/// Where the two diverge, in one sentence: every one of the 102 moves
/// succeeds here, `Shove::run` answering `Ok` with the head via moved,
/// and the last cursor position `(142.4, 80.0)` leaves the via at
/// `(142.352108, 79.897157)`, clear of everything under the case's own
/// rules. An empty delta on KiCad's side needs the opposite: either the
/// last `Drag` failed and the restore branch re-branched a clean node
/// (`pcbnew/router/pns_dragger.cpp:1039`, and `m_lastDragSolution` is a
/// default constructed `LINE` in `DM_VIA`, so the restore adds nothing),
/// or `dragViaWalkaround` found an empty fanout and returned true at
/// `:498` having changed nothing. Both are failure shapes, which fits a
/// case whose name ends in `crash`. The disagreement is in the shove's
/// handling of a lone "stitching" via head
/// (`pcbnew/router/pns_shove.cpp:1120`), not in the dragger, so it is not
/// loosened here.
///
/// Its two tier 1 tests still run and are the real regression guard.
#[test]
#[ignore = "the via drag moves the via where KiCad's recorded session \
            moved nothing: measured added 2 versus golden 0, measured \
            removed 1 versus golden 0"]
fn issue23449_shove_lone_via_drag_crash_matches_the_golden() {
  assert_tier_two(&replay_case(&load("issue23449-shove-lone-via-drag-crash")));
}

/// A heavy single layer shove cascade across ten nets, by far the largest
/// golden in the corpus.
#[test]
fn simple_drag_shove_singlelayer_replays() {
  let case = load("simple-drag-shove-singlelayer");

  assert_board_mapping(&case);
  assert_replays(&replay_case(&case));
}

/// Tier 1's collision half for the case above.
#[test]
fn simple_drag_shove_singlelayer_leaves_nothing_colliding() {
  assert_collision_free(&replay_case(&load("simple-drag-shove-singlelayer")));
}

/// Tier 2 for the case above, by far the largest golden in the corpus.
///
/// 134 added and 139 removed across ten nets, all of it the shove cascade
/// the drag head pushes, so it is the real test of `dragShove`: the
/// dragged line goes in as one head under
/// `SHP_SHOVE | SHP_DONT_LOCK_ENDPOINTS` and everything else in the
/// answer is what the shove engine did with it.
#[test]
fn simple_drag_shove_singlelayer_matches_the_golden() {
  assert_tier_two(&replay_case(&load("simple-drag-shove-singlelayer")));
}

/// Walking around teardrop pads with hugging disabled. The only self
/// contained case, shipping its own board.
#[test]
fn walk_with_teardrops_replays() {
  let case = load("walk-with-teardrops");

  assert_board_mapping(&case);
  assert_replays(&replay_case(&case));
}

/// Tier 1's collision half for the case above.
#[test]
fn walk_with_teardrops_leaves_nothing_colliding() {
  assert_collision_free(&replay_case(&load("walk-with-teardrops")));
}

/// Tier 2 for the case above, the golden the DRC epsilon unlocked.
///
/// One pair of 0.2 mm tracks on this board sits at exactly the 0.2 mm net
/// class clearance. While `KicadRules::clearance_epsilon` answered zero
/// the pair collided here and not in KiCad, so the very first move ran a
/// walkaround it never needed and that walk circled the board outline,
/// giving 15 added against the golden's 17. With the 500 nm epsilon
/// `PNS_PCBNEW_RULE_RESOLVER` subtracts, the two agree.
#[test]
fn walk_with_teardrops_matches_the_golden() {
  assert_tier_two(&replay_case(&load("walk-with-teardrops")));
}

/// Dragging a segment into the board outline.
#[test]
fn walk_drag_seg_against_board_edge_replays() {
  let case = load("walk_drag_seg_against_board_edge");

  assert_board_mapping(&case);
  assert_replays(&replay_case(&case));
}

/// Tier 1's collision half for the case above.
#[test]
fn walk_drag_seg_against_board_edge_leaves_nothing_colliding() {
  assert_collision_free(&replay_case(&load(
    "walk_drag_seg_against_board_edge",
  )));
}

/// Tier 2 for the case above, which agrees with the golden vertex for
/// vertex.
///
/// The drag is one 73 mm segment of a 1 mm track moved sideways into the
/// board outline. The counts agreed before the shove drag landed as well,
/// because three segments went out and three came back either way; the
/// geometry did not, and it does now. Tier 3 is not asserted anywhere in
/// this file, so that agreement is recorded here rather than checked.
#[test]
fn walk_drag_seg_against_board_edge_matches_the_golden() {
  assert_tier_two(&replay_case(&load("walk_drag_seg_against_board_edge")));
}

// ---------------------------------------------------------------------
// The thread count does not reach the answer
// ---------------------------------------------------------------------

/// Every case in the corpus, all of which replay.
const REPLAYABLE: &[&str] = &[
  "backspace1",
  "drag-acute-fallback",
  "drag-walk-optimize-a",
  "drag-walk-optimize-fix-corners",
  "issue22749-shove-weird-drag-track-end",
  "issue23449-shove-lone-via-drag-crash",
  "issue24132-shove-same-net-via",
  "simple-drag-shove-singlelayer",
  "simple-shove-1",
  "walk-with-teardrops",
  "walk_drag_seg_against_board_edge",
];

/// Every case answers the same on one thread and on eight.
///
/// `pnsrouter::node::World::nearest_obstacle` cuts its candidate scan
/// into blocks and runs them on `std::thread::scope` threads, so the
/// block boundaries move with the thread count. `DESIGN.md` section 8
/// says the engine is a pure function of (snapshot, settings, event
/// sequence), and a thread count is none of those. The comparison is the
/// recorder's own text, which is the commit diffs vertex for vertex plus
/// everything the recording carries.
///
/// `tests/parallelism.rs` runs the same check over the stored session
/// fixtures and says what neither set reaches: the largest obstacle set
/// in this corpus is 9 candidates, so these sessions stay under the
/// block threshold and never cut a block. The split itself is pinned by
/// unit tests in `src/node.rs`.
#[test]
fn every_replayable_case_replays_the_same_at_every_thread_count() {
  for name in REPLAYABLE {
    let case = load(name);
    let sequential = replay_case_with_parallelism(&case, Some(1));
    let parallel = replay_case_with_parallelism(&case, Some(8));

    assert!(
      sequential.recording.to_text() == parallel.recording.to_text(),
      "`{name}` replays differently on one thread and on eight"
    );
    assert_eq!(
      sequential.measured_added, parallel.measured_added,
      "`{name}` adds a different set of items on eight threads"
    );
    assert_eq!(
      sequential.measured_removed, parallel.measured_removed,
      "`{name}` removes a different number of items on eight threads"
    );
  }
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
