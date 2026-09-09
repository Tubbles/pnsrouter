#!/usr/bin/env bash
# Run a command inside the pnsrouter development container with both this
# repository and a LibrePCB checkout mounted.
#
#   dev/librepcb-in-container.sh cmake --version
#   dev/librepcb-in-container.sh ninja -C build -j16
#
# Both repositories are bind mounted at the same absolute path as on the
# host and the container runs with the host user id, so files written by
# the build are owned by the host user. The working directory is the
# LibrePCB checkout, because that is where the build happens; pnsrouter is
# mounted so that the path dependency in
# libs/librepcb/rust-core/Cargo.toml resolves.
#
# The cargo registry and the ccache directory live in named podman volumes
# so they survive between runs. The image is dev/Containerfile, which is
# LibrePCB's own CI image plus clippy and rustfmt; build it once with:
#
#   podman build -t pnsrouter-dev -f dev/Containerfile dev
#
# Set LIBREPCB_REPO to point at a checkout other than ~/dev/librepcb.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
librepcb_root="${LIBREPCB_REPO:-$HOME/dev/librepcb}"
image="${PNSROUTER_DEV_IMAGE:-localhost/pnsrouter-dev:latest}"

if [ ! -d "$librepcb_root" ]; then
  echo "No LibrePCB checkout at $librepcb_root, set LIBREPCB_REPO" >&2
  exit 1
fi

tty_flags=(-i)
if [ -t 0 ] && [ -t 1 ]; then
  tty_flags=(-it)
fi

# The image sets no ccache variable, so ccache falls back to its built in
# default of $HOME/.cache/ccache. Under --userns=keep-id podman rewrites
# HOME to the passwd entry of the mapped uid, which is /home/ubuntu in this
# image, so that is where the volume goes. CCACHE_DIR is exported to the
# same path so the location cannot silently drift if the home changes.
ccache_dir="/home/ubuntu/.cache/ccache"

exec podman run --rm "${tty_flags[@]}" \
  --userns=keep-id \
  --security-opt label=disable \
  -v "$repo_root:$repo_root" \
  -v "$librepcb_root:$librepcb_root" \
  -v pnsrouter-cargo-registry:/.cargo/registry:U \
  -v pnsrouter-cargo-git:/.cargo/git:U \
  -v librepcb-ccache:"$ccache_dir":U \
  -e CCACHE_DIR="$ccache_dir" \
  -w "$librepcb_root" \
  "$image" "$@"
