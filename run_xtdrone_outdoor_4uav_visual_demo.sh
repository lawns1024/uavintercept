#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-xtdrone_sim}"
GUI="${GUI:-true}"
VERBOSE="${VERBOSE:-false}"
PAUSED="${PAUSED:-false}"
STARTUP_WAIT="${STARTUP_WAIT:-14}"
LAUNCH_FILE="${LAUNCH_FILE:-multi_vehicle.launch}"
LAUNCH_DIR="${LAUNCH_DIR:-}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "错误: 容器未运行 -> $CONTAINER_NAME"
  echo "请先执行: ./start_xtdrone_container.sh"
  exit 1
fi

XT_ROOT=""
for p in /root/XTDrone /home/xtdrone/XTDrone /workspace/XTDrone; do
  if docker exec "$CONTAINER_NAME" bash -lc "test -d '$p'"; then
    XT_ROOT="$p"
    break
  fi
done

if [ -z "$XT_ROOT" ]; then
  echo "错误: 未找到容器内 XTDrone 根目录"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_XT_ROOT="$SCRIPT_DIR/XTDrone"
HOST_LAUNCH_PATH="$HOST_XT_ROOT/coordination/launch_generator/$LAUNCH_FILE"
HOST_VISUAL_DEMO="$HOST_XT_ROOT/coordination/four_uav_visual_demo_gazebo.py"

if [ -z "$LAUNCH_DIR" ]; then
  LAUNCH_DIR="$XT_ROOT/coordination/launch_generator"
fi
LAUNCH_PATH="$LAUNCH_DIR/$LAUNCH_FILE"
VISUAL_DEMO_PATH="$XT_ROOT/coordination/four_uav_visual_demo_gazebo.py"

if [ -f "$HOST_LAUNCH_PATH" ]; then
  docker exec "$CONTAINER_NAME" bash -lc "mkdir -p '$LAUNCH_DIR'"
  docker cp "$HOST_LAUNCH_PATH" "$CONTAINER_NAME:$LAUNCH_PATH"
fi
if [ -f "$HOST_VISUAL_DEMO" ]; then
  docker exec "$CONTAINER_NAME" bash -lc "mkdir -p '$XT_ROOT/coordination'"
  docker cp "$HOST_VISUAL_DEMO" "$CONTAINER_NAME:$VISUAL_DEMO_PATH"
fi

INNER_CMD="
set -e
source /opt/ros/noetic/setup.bash
if [ -f ~/catkin_ws/devel/setup.bash ]; then source ~/catkin_ws/devel/setup.bash; fi
if [ -f /root/PX4_Firmware/Tools/setup_gazebo.bash ]; then
  source /root/PX4_Firmware/Tools/setup_gazebo.bash /root/PX4_Firmware /root/PX4_Firmware/build/px4_sitl_default
  export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/PX4_Firmware
  export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/PX4_Firmware/Tools/sitl_gazebo
fi
if [ ! -f '$LAUNCH_PATH' ]; then echo '错误: launch 不存在 -> $LAUNCH_PATH'; exit 1; fi
if [ ! -f '$VISUAL_DEMO_PATH' ]; then echo '错误: 视觉demo不存在 -> $VISUAL_DEMO_PATH'; exit 1; fi

killall -9 roslaunch rosmaster gzserver gzclient px4 mavros_node 2>/dev/null || true
sleep 2

cd $XT_ROOT
chmod +x '$VISUAL_DEMO_PATH'

roslaunch '$LAUNCH_PATH' gui:=${GUI} verbose:=${VERBOSE} paused:=${PAUSED} &
LAUNCH_PID=\$!

echo '[INFO] 等待室外仿真启动...'
sleep ${STARTUP_WAIT}

echo '[INFO] 启动四机视觉示意（各飞各的，最终编队外观）...'
python3 '$VISUAL_DEMO_PATH'

echo '[INFO] 视觉示意完成，仿真保持运行便于录屏。'
wait \$LAUNCH_PID
"

exec docker exec -it "$CONTAINER_NAME" bash -lc "$INNER_CMD"
