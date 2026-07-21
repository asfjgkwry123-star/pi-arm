#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_root}"

colcon build --symlink-install --event-handlers console_direct+
echo "构建完成。运行：source ${workspace_root}/install/setup.bash"
