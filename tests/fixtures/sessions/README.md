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

Record a session in `tests/eventlog.rs`, write it here with `SessionRecording::to_text`, and add a test that reads it back and calls `assert_replay_matches`. Keep one file to one scenario, and let the file name say what the scenario is.

### The `librepcb_` prefix

A file named `librepcb_*.txt` was recorded by LibrePCB itself, through the directory named by `LIBREPCB_PNS_RECORD_DIR`, and needs no named test. `tests/librepcb_sessions.rs` reads the directory at run time, and for every file it finds it checks the text round trip, `assert_replay_matches` and `assert_replay_is_collision_free`, all with `FixedClearance::uniform(sizes.min_clearance)`, the board minimum clearance the file carries. Dropping a recording in here is the whole of adding a case.

The known limitation is the resolver. A recording carries `max_clearance` and the sizes but no rule table, so the uniform board minimum is the best resolver that can be reconstructed from the file. That is exact only while no net class on the board overrides the board minimum. A board that does override it will replay to different geometry than LibrePCB produced and must not be added under this prefix until recordings carry their rules, which is a `TODO.md` item. Such a board can still live here under another name, with a named test that hands `replay` a resolver of its own.

## The files

- `two_layer_via.txt`: a route out of a pad on layer 0, a via, a leg on layer 1, one fix undone again, and a terminal fix on a pad on layer 1. The board is the two layer fixture `tests/router.rs` and `tests/placer.rs` use.
- `kicad_backspace1.txt`: KiCad's `backspace1` regression case, replayed. One start on a pad of footprint `U1`, 314 moves, ten fixes and nine backspaces, in shove mode. It is the only case in KiCad's corpus that exercises `UndoLastSegment`.
- `kicad_shove_same_net_via.txt`: KiCad's `issue24132-shove-same-net-via` regression case, replayed. One start, 342 moves and one fix, in shove mode, on the corpus's smallest board.
- `librepcb_gerber_test.txt`: the first recording made by a host. LibrePCB's `BoardPnsRouterTest` routed one leg out of a footprint pad into free space on its `Gerber Test` project, in walkaround mode, 236 items on the board and one commit of three segments. Its origin comment is added by hand, everything below it is what LibrePCB wrote.

### Where the two KiCad fixtures come from

They are produced by `tests/kicad_replay.rs`, which reads a case out of `tests/fixtures/kicad/pns_regressions`, turns its board into a snapshot with `tests/support/kicad_snapshot.rs`, drives a `Router` from the recorded events, and writes the recorder's answer here. Regenerate them with

    dev/in-container.sh cargo test --test kicad_replay -- --ignored \
      regenerate_the_kicad_session_fixtures

The same warning applies as above: a changed commit diff is a change of routing behaviour.

Their value is that the recording carries its own `WorldSnapshot`, so once the file exists the engine is pinned against a real board independently of the board reader. The rule resolver is the exception: `the_kicad_session_fixtures_replay_to_the_commits_stored_in_them` builds one from the live case, so a change to what `tests/support/kicad_snapshot.rs` or `tests/support/kicad_dru.rs` resolve moves these files too, and the recorded `max-clearance` line moves with it.

Both fixtures now reproduce KiCad's own added and removed counts for their case. `issue24132-shove-same-net-via` reaches its four added and one removed only because the case's `pns.kicad_dru` is read: its net blind 2 mm physical clearance between a track and a via is what makes the shove move the board's via even on the via's own net.
