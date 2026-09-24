#!/usr/bin/env bash
# Run cargo publish in the dev container with the host's crates.io login
# mounted read only into the container's cargo home (/.cargo in the
# image), so the token stays in ~/.cargo/credentials.toml, written by
# `cargo login` on the host, and never touches a command line.
#
#   dev/publish.sh dry       # cargo publish --dry-run
#   dev/publish.sh upload    # cargo publish
#
# Mirrors dev/in-container.sh's mounts and user mapping; the runbook is
# doc/work/007-hardening-and-release.md.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${PNSROUTER_DEV_IMAGE:-localhost/pnsrouter-dev:latest}"
case "${1:-}" in
  dry) args=(cargo publish --dry-run) ;;
  upload) args=(cargo publish) ;;
  *) echo "usage: $0 dry|upload" >&2; exit 2 ;;
esac
exec podman run --rm -i \
  --userns=keep-id \
  --security-opt label=disable \
  -v "$repo_root:$repo_root" \
  -v pnsrouter-cargo-registry:/.cargo/registry:U \
  -v pnsrouter-cargo-git:/.cargo/git:U \
  -v "$HOME/.cargo/credentials.toml:/.cargo/credentials.toml:ro" \
  -w "$repo_root" \
  "$image" "${args[@]}"
