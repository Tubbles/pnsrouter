# 007 Hardening and release

Status: todo

## Goal

Confidence and a first crates.io release.

## Tasks

- [x] Fixtures from real LibrePCB boards through the recorder. LibrePCB writes one file per routing session when `LIBREPCB_PNS_RECORD_DIR` names an existing directory, in this crate's own recorded session format. See [../librepcb-integration.md](../librepcb-integration.md) section 3.1.1 and the note there on picking a resolver for the replay.
- [ ] Fuzzing of line chain, collision and hull code (cargo-fuzz), property tests for geometry invariants.
- [x] Performance profiling on a large board, budget tuning. See [../performance.md](../performance.md): the `examples/latency.rs` harness, the numbers, the profile and what the shove budget buys.
- [x] `cargo doc` clean, README examples, CHANGELOG. The crate level documentation in `src/lib.rs` is a compiling doctest of a whole session plus a map of the modules, `README.md` carries the same example trimmed, and [../../CHANGELOG.md](../../CHANGELOG.md) says what 0.1.0 contains and what it deliberately leaves out.
- [ ] 0.1.0 on crates.io. Packaging is ready: `Cargo.toml` has the metadata crates.io asks for and an `exclude` that keeps the 12 MB KiCad corpus, the reference notes and the repository only directories out, which brings the package down to roughly 0.65 MB compressed. `documentation` is left unset on purpose, since crates.io links to docs.rs by itself. The two integration tests that read the corpus (`tests/kicad_fixtures.rs`, `tests/kicad_replay.rs`) and the readers under `tests/support/` are excluded with it; `cargo test` inside `target/package/pnsrouter-0.1.0` passes on what is left. The runbook is below.

## Publishing 0.1.0

Everything runs in the container, from the repository root. `cargo publish` refuses a dirty tree, so the CHANGELOG date is committed first.

1. Green on all five checks:

        dev/in-container.sh cargo fmt --all --check
        dev/in-container.sh cargo clippy --all-targets --features fail-on-warnings -- -D warnings
        dev/in-container.sh cargo test
        dev/in-container.sh sh -c 'RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --document-private-items'
        dev/in-container.sh cargo package

2. Read the file list once more and check that nothing under `tests/fixtures/kicad/` is in it:

        dev/in-container.sh cargo package --list

3. Replace `0.1.0 - unreleased` in `CHANGELOG.md` with the release date and commit that on its own.

4. Rehearse the upload. This one reaches the crates.io index but needs no token, since it stops before the upload:

        dev/in-container.sh cargo publish --dry-run

5. Upload. `dev/in-container.sh` forwards no environment and the container's `CARGO_HOME` is not a mounted volume, so a `cargo login` would not survive the run and the token has to go on the command line. Substitute wherever the crates.io token is kept:

        dev/in-container.sh cargo publish --token "$(cat <path to the token>)"

6. Tag the commit that was published and push both:

        git tag -a v0.1.0 -m "pnsrouter 0.1.0"
        git push origin main
        git push origin v0.1.0

7. Check that docs.rs built the crate, at https://docs.rs/pnsrouter/0.1.0. It builds on its own toolchain, so a doctest or an intra doc link that only passes locally shows up there.

## Acceptance

Release published, CI green on stable and MSRV.
