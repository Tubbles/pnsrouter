// SPDX-License-Identifier: GPL-3.0-or-later

//! Every session LibrePCB recorded, replayed.
//!
//! LibrePCB writes a recording of each push and shove session it runs to
//! the directory named by `LIBREPCB_PNS_RECORD_DIR`, in the text format
//! of `src/eventlog.rs`. Dropping such a file into
//! `tests/fixtures/sessions/` under the `librepcb_` prefix is the whole
//! of adding a regression case: this test discovers the files at run
//! time, so nothing has to be written per recording.
//! `tests/fixtures/sessions/README.md` states the convention and the one
//! kind of board it does not cover.
//!
//! # The rules a recording is replayed under
//!
//! A recording carries the board, the settings, the sizes and the events,
//! but no rule table, so the resolver has to be reconstructed. The one
//! clearance a recording does carry is [`Sizes::min_clearance`], the
//! board minimum, and [`FixedClearance::uniform`] over it reproduces a
//! session exactly as long as no net class on the board overrode that
//! minimum. Serialising the rules is a `TODO.md` item, and until it
//! happens a board with net class overrides does not belong under this
//! prefix.

#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

use std::panic::AssertUnwindSafe;
use std::path::{Path, PathBuf};

use pnsrouter::eventlog::{
  SessionRecording, assert_replay_is_collision_free, assert_replay_matches,
};
use pnsrouter::rules::{FixedClearance, RuleResolver};
use pnsrouter::settings::Sizes;

/// The prefix that marks a fixture as one LibrePCB recorded.
const PREFIX: &str = "librepcb_";

/// Where the recorded session fixtures live.
///
/// Located off `CARGO_MANIFEST_DIR` the way `tests/eventlog.rs` and
/// `tests/kicad_replay.rs` locate the same directory, so that the test
/// runs from any working directory.
fn session_root() -> PathBuf {
  Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/sessions")
}

/// The file name of every `librepcb_*.txt` fixture, sorted.
///
/// Sorted because `read_dir` yields the directory's own order, and a
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

    if name.starts_with(PREFIX) && name.ends_with(".txt") {
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

/// The resolver a recording is replayed under, as a factory.
///
/// [`assert_replay_matches`] replays twice and a `Box<dyn RuleResolver>`
/// cannot be cloned, so it asks for a factory rather than a resolver.
fn resolver_of(sizes: &Sizes) -> impl Fn() -> Box<dyn RuleResolver> + '_ {
  move || Box::new(FixedClearance::uniform(sizes.min_clearance))
}

/// Run one assertion and name the fixture if it fails.
///
/// The two replay assertions panic with the detail of the mismatch but
/// not with the file it came from, and a directory driven test that does
/// not name the file leaves the reader to guess which one broke. The
/// original message is still printed, by the panic hook, before this one.
fn named<Body>(name: &str, what: &str, body: Body)
where
  Body: FnOnce(),
{
  let outcome = std::panic::catch_unwind(AssertUnwindSafe(body));

  assert!(outcome.is_ok(), "{name}: {what}");
}

/// The prefix is not a dead glob.
///
/// Without this, an empty directory or a renamed prefix turns every test
/// below into a silent pass.
#[test]
fn at_least_one_librepcb_recording_is_in_the_tree() {
  assert!(
    !fixture_names().is_empty(),
    "no {PREFIX}*.txt fixture in {}",
    session_root().display()
  );
}

/// A recording survives a round trip through its own text format.
///
/// The files are written on the LibrePCB side and read here, so the
/// writer and the reader agreeing on every field is what makes the
/// replay below mean anything.
#[test]
fn every_librepcb_recording_round_trips_through_its_text_format() {
  for name in fixture_names() {
    let recording = load(&name);
    let text = recording.to_text();
    let read = SessionRecording::from_text(&text).unwrap_or_else(|error| {
      panic!("{name} does not parse after being written back: {error}")
    });

    assert!(
      read == recording,
      "{name} does not survive a write and a read"
    );
  }
}

/// Every recording replays to the commits stored in it, and cleanly.
///
/// Tiers 2 and 3 of `tests/fixtures/sessions/README.md`, both at the
/// board minimum clearance the file itself carries.
#[test]
fn every_librepcb_recording_replays_to_the_commits_stored_in_it() {
  for name in fixture_names() {
    let recording = load(&name);
    let rules = resolver_of(&recording.sizes);

    named(
      &name,
      "the replay does not reproduce the recorded commits",
      || {
        assert_replay_matches(&recording, &rules);
      },
    );
    named(
      &name,
      "the replay leaves a clearance violation behind",
      || {
        assert_replay_is_collision_free(&recording, &rules);
      },
    );
  }
}
