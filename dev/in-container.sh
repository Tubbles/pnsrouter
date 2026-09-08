#!/usr/bin/env bash
# Run a command inside the pnsrouter development container.
#
#   dev/in-container.sh cargo test
#   dev/in-container.sh cargo clippy --all-targets --features fail-on-warnings -- -D warnings
#
# The repository is bind mounted at the same absolute path as on the host and
# the container runs with the host user id, so files written by the build are
# owned by the host user. The cargo registry and git caches live in named
# podman volumes so they survive between runs. The image is built from
# dev/Containerfile; build it once with:
#
#   podman build -t pnsrouter-dev -f dev/Containerfile dev
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${PNSROUTER_DEV_IMAGE:-localhost/pnsrouter-dev:latest}"

tty_flags=(-i)
if [ -t 0 ] && [ -t 1 ]; then
  tty_flags=(-it)
fi

exec podman run --rm "${tty_flags[@]}" \
  --userns=keep-id \
  --security-opt label=disable \
  -v "$repo_root:$repo_root" \
  -v pnsrouter-cargo-registry:/.cargo/registry:U \
  -v pnsrouter-cargo-git:/.cargo/git:U \
  -w "$repo_root" \
  "$image" "$@"
