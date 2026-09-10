// SPDX-License-Identifier: GPL-3.0-or-later

//! The obstacle query answers the same on one thread and on many.
//!
//! [`pnsrouter::node::World::nearest_obstacle`] scans its candidates on
//! [`std::thread::scope`] threads once there are enough of them, and the
//! block boundaries fall wherever the candidate count and the thread
//! count put them. `DESIGN.md` section 8 says the engine is a pure
//! function of (snapshot, settings, event sequence); the thread count is
//! none of the three, so it must not appear in the answer. That is the
//! whole of what this file checks, and it checks it the only way that
//! means anything: by replaying real sessions and comparing every commit.
//!
//! The four KiCad regression cases are the other half of the same check
//! and live in `tests/kicad_replay.rs`, because that is where the corpus
//! reader is.
//!
//! # What these tests do not reach
//!
//! Neither the fixtures here nor KiCad's corpus produces a query big
//! enough to be cut into blocks. Measured on the recordings in the tree:
//! the largest obstacle set a fixture here reaches is 6 candidates and
//! the largest in the corpus is 9, against a block of 32. So what these
//! tests pin end to end is that the knob changes nothing, not that the
//! blocks are cut correctly. The blocks themselves are pinned by
//! `the_candidate_scan_answers_the_same_at_every_block_count` and
//! `a_distance_tie_goes_to_the_lowest_uid_at_every_thread_count` in
//! `src/node.rs`, which build a candidate list big enough on purpose.
//!
//! They are still worth running: a denser recording dropped into
//! `tests/fixtures/sessions/` reaches the split without anything else
//! having to change, and they cover the plumbing between
//! `Router::set_parallelism` and the query.
//!
//! # Why the resolver does not have to be the recorded one
//!
//! A recording carries no rule table, so `tests/librepcb_sessions.rs`
//! rebuilds one from [`Sizes::min_clearance`]. Here it does not matter
//! whether that is the table the session was recorded under: both
//! replays use the same resolver, so a difference between them can only
//! come from the thread count. The commits a fixture replays to under
//! these rules are pinned by the other test files, not by this one.

#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

use std::path::{Path, PathBuf};

use pnsrouter::eventlog::{SessionRecording, replay, replay_with_parallelism};
use pnsrouter::rules::FixedClearance;

/// The thread count that stands for "sequential".
const SEQUENTIAL: usize = 1;

/// The thread count that stands for "as many blocks as the query allows".
///
/// Eight is [`pnsrouter::node::World::MAX_USEFUL_PARALLELISM`], the most
/// a host has any reason to ask for.
const PARALLEL: usize = 8;

/// Where the recorded session fixtures live.
///
/// Located off `CARGO_MANIFEST_DIR` the way `tests/eventlog.rs`,
/// `tests/librepcb_sessions.rs` and `tests/kicad_replay.rs` locate the
/// same directory, so that the test runs from any working directory.
fn session_root() -> PathBuf {
  Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/sessions")
}

/// The file name of every `*.txt` fixture, sorted.
///
/// Sorted because `read_dir` yields the directory's own order and a
/// directory driven test has to fail on the same file twice in a row.
fn fixture_names() -> Vec<String> {
  let root = session_root();
  let entries = std::fs::read_dir(&root)
    .unwrap_or_else(|error| panic!("cannot read {}: {error}", root.display()));
  let mut names: Vec<String> = Vec::new();

  for entry in entries {
    let entry = entry.unwrap_or_else(|error| {
      panic!("cannot walk {}: {error}", root.display())
    });
    let name = entry.file_name().to_string_lossy().into_owned();

    if name.ends_with(".txt") {
      names.push(name);
    }
  }

  names.sort();
  names
}

/// Read and parse one fixture by file name.
fn load(name: &str) -> SessionRecording {
  let path = session_root().join(name);
  let text = std::fs::read_to_string(&path)
    .unwrap_or_else(|error| panic!("cannot read {}: {error}", path.display()));

  SessionRecording::from_text(&text)
    .unwrap_or_else(|error| panic!("{name} does not parse: {error}"))
}

/// Every commit of a replay, written out the way a fixture writes them.
///
/// The comparison is over this text rather than over the
/// [`pnsrouter::router::CommitDiff`] values because a byte for byte
/// comparison is what the fixture format promises and because a mismatch
/// then names the line that moved. [`SessionRecording::to_text`] writes
/// the whole file, so the recording is rebuilt with the replayed commits
/// in place of the stored ones and the events cut away, leaving the
/// snapshot, the settings, the sizes and the results.
fn commits_of(recording: &SessionRecording, threads: usize) -> String {
  let outcome = replay_with_parallelism(
    recording,
    Box::new(FixedClearance::uniform(recording.sizes.min_clearance)),
    threads,
  );
  let mut written = SessionRecording::new(
    recording.snapshot.clone(),
    recording.settings,
    recording.sizes.clone(),
  );

  written.results = outcome.diffs;

  format!("frames {}\n{}", outcome.frames_count, written.to_text())
}

/// The directory is not an empty glob.
///
/// Without this a renamed directory turns the test below into a silent
/// pass.
#[test]
fn at_least_one_session_fixture_is_in_the_tree() {
  assert!(
    !fixture_names().is_empty(),
    "no *.txt fixture in {}",
    session_root().display()
  );
}

/// Every recorded session replays to the same commits at both settings.
#[test]
fn every_session_fixture_replays_the_same_on_one_thread_and_on_eight() {
  for name in fixture_names() {
    let recording = load(&name);
    let sequential = commits_of(&recording, SEQUENTIAL);
    let parallel = commits_of(&recording, PARALLEL);

    assert!(
      sequential == parallel,
      "{name} replays differently on {SEQUENTIAL} thread and on \
       {PARALLEL}"
    );
  }
}

/// A world built the ordinary way answers the same as a sequential one.
///
/// The default is one thread today, so this passes trivially. It is here
/// so that changing the default cannot change an answer without a test
/// saying so.
#[test]
fn the_default_thread_count_replays_what_one_thread_replays() {
  for name in fixture_names() {
    let recording = load(&name);
    let sequential = commits_of(&recording, SEQUENTIAL);
    let outcome = replay(
      &recording,
      Box::new(FixedClearance::uniform(recording.sizes.min_clearance)),
    );
    let mut written = SessionRecording::new(
      recording.snapshot.clone(),
      recording.settings,
      recording.sizes.clone(),
    );

    written.results = outcome.diffs;

    assert!(
      sequential
        == format!("frames {}\n{}", outcome.frames_count, written.to_text()),
      "{name} replays differently at the default thread count than on one"
    );
  }
}
