#!/usr/bin/env bash
set -e

IMAGE_NAME="ghcr.io/mavtech-srl/px4-nuttx-simulation-jammy:dev"
CCACHE_DIR="${HOME}/.ccache"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "${CCACHE_DIR}"

docker pull "${IMAGE_NAME}"

if [ -n "${DISPLAY:-}" ]; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

if [ "$#" -eq 0 ]; then
  set -- bash
fi

docker run --init -it --rm --privileged \
  --env=LOCAL_USER_ID="$(id -u)" \
  --env=LOCAL_GROUP_ID="$(id -g)" \
  --env=GZ_PARTITION=sim \
  --env=CCACHE_DIR="${CCACHE_DIR}" \
  -v "${SCRIPT_DIR}:/PX4-Autopilot:rw" \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${CCACHE_DIR}:${CCACHE_DIR}:rw" \
  -e DISPLAY="${DISPLAY}" \
  --network=host \
  -w /PX4-Autopilot \
  "${IMAGE_NAME}" \
  "$@"
