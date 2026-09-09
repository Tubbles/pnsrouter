# Session fixtures

Each file here is one recorded routing session in the text format of `src/eventlog.rs`: the board snapshot, the settings, the sizes, the events a host sent, and the commit diff the engine answered with. `tests/eventlog.rs` reads them, replays them against a fresh `Router`, and compares.

`DESIGN.md` section 8 says the engine is a pure function of (snapshot, settings, event sequence). These files are what makes that statement testable, and the stored diff is the golden of note 05 section 6.6.

## What the tests assert

Three tiers, from note 05 section 6.10:

1. The replay terminates and does not panic.
2. `assert_replay_is_collision_free`: every segment the replay committed is clear under the rules it was replayed with. A host with its own clearances gets different geometry and must still pass this one.
3. `assert_replay_matches`: the commit diffs equal the ones stored in the file, vertex for vertex, and a second replay equals the first.

## Regenerating a fixture

A fixture only changes when the engine's answer changes, so regenerating one is a deliberate act:

    dev/in-container.sh cargo test --test eventlog -- --ignored regenerate_the_golden_fixture

Then read the diff. **A changed commit diff is a change of routing behaviour**, not a test that needs fixing. Before committing the new file, be able to say which change caused it and why the new geometry is the better answer. Record the reasoning in `doc/log/`, the way every other deliberate deviation from KiCad is recorded.

A changed *event* list means the session the test drives was edited, which is ordinary and needs no such justification.

KiCad's own harness regenerates its goldens with `qa_pns_regressions --update-golden` and its log viewer rewrites them on every save (`qa/tools/pns/pns_log_viewer_frame.cpp:365`), so a golden there always encodes whatever the code did on the day it was refreshed. The ignored test above is the same button with the safety catch on: it is never part of a normal `cargo test` run, and `the_golden_fixture_is_the_session_this_crate_records_today` fails loudly when the two drift apart.

## Adding a fixture

Record a session in `tests/eventlog.rs`, write it here with `SessionRecording::to_text`, and add a test that reads it back and calls `assert_replay_matches`. Keep one file to one scenario; the file name says what the scenario is.

## The files

- `two_layer_via.txt`: a route out of a pad on layer 0, a via, a leg on layer 1, one fix undone again, and a terminal fix on a pad on layer 1. The board is the two layer fixture `tests/router.rs` and `tests/placer.rs` use.
- `kicad_backspace1.txt`: KiCad's `backspace1` regression case, replayed. One start on a pad of footprint `U1`, 314 moves, ten fixes and nine backspaces, in shove mode. It is the only case in KiCad's corpus that exercises `UndoLastSegment`.
- `kicad_shove_same_net_via.txt`: KiCad's `issue24132-shove-same-net-via` regression case, replayed. One start, 342 moves and one fix, in shove mode, on the corpus's smallest board.

### Where the two KiCad fixtures come from

They are produced by `tests/kicad_replay.rs`, which reads a case out of `tests/fixtures/kicad/pns_regressions`, turns its board into a snapshot with `tests/support/kicad_snapshot.rs`, drives a `Router` from the recorded events, and writes the recorder's answer here. Regenerate them with

    dev/in-container.sh cargo test --test kicad_replay -- --ignored \
      regenerate_the_kicad_session_fixtures

The same warning applies as above: a changed commit diff is a change of routing behaviour.

Their value is that the recording carries its own `WorldSnapshot`, so once the file exists the engine is pinned against a real board independently of the KiCad readers. A change to `tests/support/kicad_snapshot.rs` moves the tier assertions in `tests/kicad_replay.rs` and leaves these two alone; a change to the engine moves these two.

Neither fixture reproduces KiCad's own answer for its case. `backspace1` matches the golden counts stored in the log; `issue24132-shove-same-net-via` does not, because that case ships a `pns.kicad_dru` whose net blind 2 mm physical clearance between a track and a via is what makes KiCad's shove move the board's via, and this crate reads no `.kicad_dru`. The difference is recorded in the `#[ignore]` reason of that case's tier 2 test.
