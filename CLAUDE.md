# pnsrouter: instructions for LLM agents

## What this is

A Rust library implementing an interactive push and shove PCB router, ported at the design level from KiCad's PNS router. First consumer: LibrePCB (local checkout at `~/dev/librepcb`, a fork of LibrePCB/LibrePCB with `upstream` remote). Read PLAN.md and DESIGN.md before touching code.

## Reference sources on this machine

- KiCad, sparse shallow clone: `~/dev/ref/kicad` (pcbnew/router, libs/kimath, qa/tools/pns, qa/tests/pcbnew, qa/data/pcbnew/pns_regressions). Commit 302b2ba1014b2f116ab38d69ffa8c6d1c633ed85 (2026-09-07).
- Horizon EDA, shallow clone: `~/dev/ref/horizon`. `3rd_party/router` is their vendored KiCad 6.0.4 router, `src/router/pns_horizon_iface.cpp` is their glue.
- rnestler/pns-router, the 2018 C++ extraction attempt: `~/dev/ref/pns-router`.
- Digested notes with file:line citations: `doc/reference/kicad/`. Read the relevant note before porting a module; they list dead code not to port and behaviours to reproduce deliberately.

## Conventions

- Formatting: rustfmt with the repository's `rustfmt.toml` (2 spaces, 80 columns, matching LibrePCB's rust-core).
- `#![forbid(unsafe_code)]`, `#![warn(missing_docs)]`, clippy clean with `--all-targets --features fail-on-warnings -- -D warnings`.
- Coordinates are i32 nanometres, products of coordinates are i64, see DESIGN.md. Do not use f64 where KiCad uses integers.
- Determinism: never iterate a HashMap or HashSet, never read the wall clock inside an algorithm, tie-break on (distance, item uid). See DESIGN.md.
- When porting a KiCad routine, cite the KiCad file:line in a doc comment. Record deliberate deviations from KiCad behaviour in `doc/log/`.
- Tests: unit tests next to the code, scenario tests under `tests/`. Mirror KiCad's qa tests where they exist (see doc/reference/kicad/01-geometry.md section 15).
- Commits: imperative title of at most 72 characters, body wrapped at 72 columns, `Assisted-by: Claude:<model-id>` trailer, no dashes as prose punctuation.

## Documents

- `PLAN.md`: milestones. `DESIGN.md`: architecture. `TODO.md`: user inbox. `SUGGESTIONS.md`: ideas found during work.
- `doc/log/YYYY-MM-DD.md`: decision log, write once. `doc/work/`: work items with status (todo, in progress, implemented, verified).
- `work/` and `tmp/` are not committed.

## Commands

Nothing is installed on the host. Every cargo invocation goes through the podman wrapper (build the image once with `podman build -t pnsrouter-dev -f dev/Containerfile dev`):

    dev/in-container.sh cargo build
    dev/in-container.sh cargo test
    dev/in-container.sh cargo clippy --all-targets --features fail-on-warnings -- -D warnings
    dev/in-container.sh cargo fmt --all --check
    dev/in-container.sh cargo doc --no-deps --document-private-items

LibrePCB builds against this crate through `dev/librepcb-in-container.sh` (mounts `~/dev/librepcb` too, working directory there); the integration lives on the fork's `pns-router` branch, design in `doc/librepcb-integration.md`. LibrePCB takes the crate from crates.io (`pnsrouter = "0.1"` in `libs/librepcb/rust-core/Cargo.toml`, since 2026-09-24), so a crate change reaches the LibrePCB build only through a published release: bump the version here, publish with `dev/publish.sh` (runbook in `doc/work/007-hardening-and-release.md`, the user approves every release), then raise the requirement in rust-core's `Cargo.toml` if the version is not semver compatible and run `cargo update -p pnsrouter` on its lock file. To try an unpublished crate change from LibrePCB, point rust-core at a path dependency on this checkout locally and never commit that.

The LibrePCB side checks, all from this repository (the build directory is already configured with Ninja and tests on):

    dev/librepcb-in-container.sh cargo fmt --manifest-path libs/librepcb/rust-core/Cargo.toml --check
    dev/librepcb-in-container.sh cargo clippy --manifest-path libs/librepcb/rust-core/Cargo.toml --lib --features ffi -- -D warnings
    dev/librepcb-in-container.sh cargo test --manifest-path libs/librepcb/rust-core/Cargo.toml --quiet
    dev/librepcb-in-container.sh cargo test --manifest-path libs/librepcb/rust-core/Cargo.toml --features ffi --lib --quiet
    dev/librepcb-in-container.sh sh -c 'cd build && ninja -j16 librepcb_unittests'
    dev/librepcb-in-container.sh sh -c 'xvfb-run -a ./build/tests/unittests/librepcb-unittests --gtest_filter="BoardPns*"'

The fork carries feature branches that stay separate until upstreamed: `pns-router` (this crate's integration), `fix-selection-rect-performance` (an upstream regression fix), `diff-pairs`, `compare-trace-lengths`, `pns-router-pairs` (`pns-router` plus `diff-pairs` plus the pair and tuning UI) and `touchpad-navigation` (FreeCAD's touchpad navigation style as a workspace setting, unrelated to the router). The branch `integration` is upstream master plus every feature branch merged, for the user to test everything at once. It is disposable: when a feature branch changes, recreate it from master (`git branch -D integration`, then `git worktree add tmp/integration -b integration <master sha>`), merge each feature branch (order: `fix-selection-rect-performance`, `compare-trace-lengths`, `pns-router-pairs`, `touchpad-navigation`), then cherry pick every commit of `integration-fixups` (the durable home of commits that only make sense on top of all feature branches, today the route tool following the touchpad style's snap override; it is fast forward only like the feature branches), push with `git push --force-with-lease origin integration`, remove the worktree. This is the one branch the user has authorised a force push for (2026-09-11); feature branches stay fast forward only. Small additions can also be merged into the existing branch.

Plain `--lib` does not compile the `ffi` module and `--all-targets` fails on pre-existing test lints in rust-core, so `--lib --features ffi` is the clippy check that matters. The same goes for the tests: the unit tests inside the `ffi` module only run with `--features ffi --lib`, which is why both test lines are listed. `clang-format` is not in the image yet.
