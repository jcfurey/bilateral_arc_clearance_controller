#!/usr/bin/env bash
# Build and test the complete package in a clean ROS 2 / Nav2 container.
#
#   ./docker/nav2/verify.sh            # ROS 2 Lyrical (the default)
#   ./docker/nav2/verify.sh jazzy      # ROS 2 Jazzy
#   BAC_ROS_DISTRO=jazzy ./docker/nav2/verify.sh
set -euo pipefail

readonly SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
readonly PACKAGE_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)
readonly ROS_DISTRO=${1:-${BAC_ROS_DISTRO:-lyrical}}
readonly IMAGE_NAME=${BAC_NAV2_IMAGE:-bac-nav2-${ROS_DISTRO}-verification}

case "${ROS_DISTRO}" in
  jazzy|lyrical) ;;
  *)
    echo "Unsupported ROS 2 distribution '${ROS_DISTRO}': expected jazzy or lyrical" >&2
    exit 2
    ;;
esac

docker build \
  --tag "${IMAGE_NAME}" \
  --build-arg "ROS_DISTRO=${ROS_DISTRO}" \
  --file "${SCRIPT_DIR}/Dockerfile" \
  "${PACKAGE_ROOT}"

docker run --rm \
  --mount "type=bind,src=${PACKAGE_ROOT},dst=/source,readonly" \
  "${IMAGE_NAME}" \
  bash /source/docker/nav2/test_package.sh
