#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-xtdrone_sim}"
LAUNCH_FILE="${LAUNCH_FILE:-indoor1.launch}"
XT_LAUNCH_DIR="${XT_LAUNCH_DIR:-}"
GUI="${GUI:-true}"
VERBOSE="${VERBOSE:-false}"
PAUSED="${PAUSED:-false}"
SIM_TIMEOUT="${SIM_TIMEOUT:-0}"
INTERACTIVE="${INTERACTIVE:-1}"

if ! sudo docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "错误: 容器未运行 -> $CONTAINER_NAME"
  echo "请先执行: ./start_xtdrone_container.sh"
  exit 1
fi

# 说明：该脚本不加载额外下洗/风场插件，使用 XTDrone 默认 PX4+Gazebo 动力学配置。
if [ -z "$XT_LAUNCH_DIR" ]; then
  for base in /root/XTDrone /home/xtdrone/XTDrone /workspace/XTDrone; do
    if sudo docker exec "$CONTAINER_NAME" bash -lc "test -d '$base/sitl_config/launch'"; then
      XT_LAUNCH_DIR="$base/sitl_config/launch"
      break
    fi
  done
fi

if [ -z "$XT_LAUNCH_DIR" ]; then
  echo "错误: 无法自动探测容器内 XTDrone launch 目录"
  echo "请手动设置 XT_LAUNCH_DIR，例如: /root/XTDrone/sitl_config/launch"
  exit 1
fi

LAUNCH_PATH="$XT_LAUNCH_DIR/$LAUNCH_FILE"

if ! sudo docker exec "$CONTAINER_NAME" bash -lc "test -f '$LAUNCH_PATH'"; then
  echo "错误: 容器内未找到 launch 文件 -> $LAUNCH_PATH"
  echo "可通过环境变量指定: XT_LAUNCH_DIR 或 LAUNCH_FILE"
  exit 1
fi

INNER_CMD="
set -e
source /opt/ros/noetic/setup.bash
if [ -f ~/catkin_ws/devel/setup.bash ]; then source ~/catkin_ws/devel/setup.bash; fi

# PX4 + Gazebo package environment (required for mavlink_sitl_gazebo)
if [ -f /root/PX4_Firmware/Tools/setup_gazebo.bash ]; then
  source /root/PX4_Firmware/Tools/setup_gazebo.bash /root/PX4_Firmware /root/PX4_Firmware/build/px4_sitl_default
  export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/PX4_Firmware
  export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/PX4_Firmware/Tools/sitl_gazebo
fi

cd ~/XTDrone
roslaunch ${LAUNCH_PATH} gui:=${GUI} verbose:=${VERBOSE} paused:=${PAUSED}
"

if [[ "$SIM_TIMEOUT" =~ ^[0-9]+$ ]] && [ "$SIM_TIMEOUT" -gt 0 ]; then
  echo "[INFO] 启动 Gazebo 3D（无下洗气流）并在 ${SIM_TIMEOUT}s 后自动退出..."
  sudo docker exec "$CONTAINER_NAME" bash -lc "timeout ${SIM_TIMEOUT}s bash -lc '$INNER_CMD'"
else
  echo "[INFO] 启动 Gazebo 3D（无下洗气流），按 Ctrl+C 结束..."
  if [ "$INTERACTIVE" = "1" ]; then
    exec sudo docker exec -it "$CONTAINER_NAME" bash -lc "$INNER_CMD"
  else
    exec sudo docker exec "$CONTAINER_NAME" bash -lc "$INNER_CMD"
  fi
fi
