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
