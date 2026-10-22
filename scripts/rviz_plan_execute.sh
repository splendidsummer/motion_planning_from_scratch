#!/usr/bin/env bash
# 在 MoveIt 中规划 Gento Skye 的桌面抓取，在 RViz 里显示，并在模型上回放。
#
# 回放会发布 /display_planned_path 和 /joint_states。RViz 里的 Plan /
# Execute 不会驱动真机；这个软件栈没有连到硬件的 FollowJointTrajectory。
#
# 用法：
#   ./scripts/rviz_plan_execute.sh
#   ./scripts/rviz_plan_execute.sh --planner RRT --time 8
#   ./scripts/rviz_plan_execute.sh --planner RRTstar --time 8
#
# 手动做同样的事时，先 source ROS，再开两个终端。
# 请用 /usr/bin/python3：conda 环境激活后，PATH 上排在前面的是 Python 3.13，
# 而 Humble 的 rclpy 扩展是按 CPython 3.10 编译的。
#   ros2 launch gento_skye_moveit_config demo.launch.py
#   /usr/bin/python3 docs/scene_gento_skye.py --planner RRT --time 8
#
# 停止本脚本启动的演示：
#   pkill -INT -f 'ros2 launch gento_skye_moveit_config'

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLANNER="RRT"
TIME="8"
STARTED_LAUNCH=0
LAUNCH_PID=""

usage() {
  cat <<'EOF'
用法: rviz_plan_execute.sh [--planner 名称] [--time 秒数]

若 MoveIt 和 RViz（demo.launch.py）尚未运行，则先启动它们，
然后运行 docs/scene_gento_skye.py。该脚本为 both_arms 规划绕过
桌子、方块和球的轨迹，把轨迹发布给 RViz，并在模型上回放关节。

默认规划器是 RRT。其他名称：RRTConnect、RRTstar、PRM、PRMstar、
EST、SBL、KPIECE、BKPIECE、LBKPIECE、TRRT、BiTRRT、AnytimePathShortening。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --planner)
      PLANNER="${2:?--planner 需要一个名称}"
      shift 2
      ;;
    --time)
      TIME="${2:?--time 需要秒数}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ -z "${DISPLAY:-}" ]]; then
  echo "未设置 DISPLAY。RViz 需要图形会话。" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${HOME}/.local/opt/ros-humble-user/local_setup.bash"
set -u

cd "${ROOT}"

move_group_up() {
  # 进程先起来，服务要晚几秒。规划必须等服务出现。
  ros2 service list 2>/dev/null | grep -q '/plan_kinematic_path'
}

rviz_up() {
  # 节点列表会留下已崩溃进程的名字。只认还活着的 rviz2 进程。
  pgrep -f '/opt/ros/humble/lib/rviz2/rviz2' >/dev/null 2>&1
}

stop_demo() {
  # 被暂停的 joint_state_publisher 在继续运行之前会忽略 INT 和 TERM。
  pkill -CONT -f '/joint_state_publisher/joint_state_publisher' 2>/dev/null || true
  sleep 0.3
  pkill -TERM -f '/joint_state_publisher/joint_state_publisher .*gento_skye' 2>/dev/null || true
  pkill -INT -f 'ros2 launch gento_skye_moveit_config' 2>/dev/null || true
  sleep 2
  pkill -TERM -f 'ros2 launch gento_skye_moveit_config' 2>/dev/null || true
  pkill -TERM -f '/moveit_ros_move_group/move_group' 2>/dev/null || true
  pkill -TERM -f '/opt/ros/humble/lib/rviz2/rviz2' 2>/dev/null || true
  sleep 1
  pkill -KILL -f '/joint_state_publisher/joint_state_publisher .*gento_skye' 2>/dev/null || true
}

start_demo() {
  LOG="${TMPDIR:-/tmp}/gento_skye_demo_launch.log"
  echo "正在启动 MoveIt 和 RViz: ros2 launch gento_skye_moveit_config demo.launch.py"
  echo "启动日志: ${LOG}"
  ros2 launch gento_skye_moveit_config demo.launch.py >"${LOG}" 2>&1 &
  LAUNCH_PID=$!
  STARTED_LAUNCH=1
  disown "${LAUNCH_PID}" || true

  echo "正在等待 /plan_kinematic_path 和 rviz2 ..."
  ready=0
  for _ in $(seq 1 90); do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      echo "demo.launch.py 已退出。请查看 ${LOG}" >&2
      exit 1
    fi
    if move_group_up && rviz_up; then
      ready=1
      break
    fi
    sleep 1
  done
  if [[ "${ready}" -ne 1 ]]; then
    echo "等待 move_group 和 RViz 超时。请查看 ${LOG}" >&2
    exit 1
  fi
  # 给 MotionPlanning 显示留出订阅 /display_planned_path 的时间。
  sleep 3
}

place_rviz() {
  /usr/bin/python3 - <<'PY'
import sys
import time
import gi
gi.require_version("Wnck", "3.0")
from gi.repository import GLib, Wnck

screen = Wnck.Screen.get_default()
screen.force_update()
anchor = None
target = None
for window in screen.get_windows():
    name = window.get_name() or ""
    klass = (window.get_class_group_name() or "").lower()
    if "RViz" in name and target is None:
        target = window
    if klass == "cursor":
        anchor = window
if target is None:
    print("错误: 没有 RViz 窗口。rviz2 没在运行。", file=sys.stderr)
    raise SystemExit(1)
target.unmaximize()
for _ in range(40):
    while GLib.main_context_default().iteration(False):
        pass
    time.sleep(0.05)
    screen.force_update()
    if not target.is_maximized():
        break
ax, ay = (80, 80)
if anchor is not None:
    ax, ay, _width, _height = anchor.get_geometry()
mask = (
    Wnck.WindowMoveResizeMask.X
    | Wnck.WindowMoveResizeMask.Y
    | Wnck.WindowMoveResizeMask.WIDTH
    | Wnck.WindowMoveResizeMask.HEIGHT
)
target.set_geometry(Wnck.WindowGravity.STATIC, mask, ax + 80, ay + 80, 1400, 900)
target.activate(1)
print("已把 RViz 窗口放到 Cursor 所在的屏幕。")
PY
}

if move_group_up && rviz_up; then
  echo "move_group 和 rviz2 进程都在。"
else
  if ! rviz_up; then
    echo "警告: RViz 没有在运行。正在重新启动 MoveIt 和 RViz。" >&2
  fi
  if move_group_up; then
    stop_demo
  fi
  start_demo
  if ! rviz_up; then
    echo "错误: RViz 启动后仍然没有 rviz2 进程。请查看 ${TMPDIR:-/tmp}/gento_skye_demo_launch.log" >&2
    exit 1
  fi
fi

# Humble 自带的 rclpy 面向 CPython 3.10。PATH 上的 python3 常常是 conda 的 3.13。
ROS_PYTHON="/usr/bin/python3"
if ! "${ROS_PYTHON}" -c 'import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)'; then
  echo "${ROS_PYTHON} 不是 Python 3.10，而 ROS Humble 的 rclpy 需要 3.10。" >&2
  exit 1
fi

place_rviz
echo "正在规划，随后在 RViz 中回放轨迹"
echo "${ROS_PYTHON} docs/scene_gento_skye.py --planner ${PLANNER} --time ${TIME}"
"${ROS_PYTHON}" docs/scene_gento_skye.py --planner "${PLANNER}" --time "${TIME}"

if ! rviz_up; then
  echo "警告: 回放结束后 RViz 已经不在运行，窗口可能已经崩溃。" >&2
  exit 1
fi

if [[ "${STARTED_LAUNCH}" -eq 1 ]]; then
  echo "RViz 仍开着（pid ${LAUNCH_PID}）。"
  echo "停止命令: pkill -INT -f 'ros2 launch gento_skye_moveit_config'"
fi
