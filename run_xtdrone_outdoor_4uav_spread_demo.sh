#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-xtdrone_sim}"
GUI="${GUI:-true}"
VERBOSE="${VERBOSE:-false}"
PAUSED="${PAUSED:-false}"
STARTUP_WAIT="${STARTUP_WAIT:-18}"
START_COMM_BRIDGE="${START_COMM_BRIDGE:-true}"
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
HOST_DEMO_PATH="$HOST_XT_ROOT/coordination/four_uav_spread_demo.py"
HOST_COMM_SH="$HOST_XT_ROOT/communication/multi_vehicle_communication.sh"

if [ -z "$LAUNCH_DIR" ]; then
  LAUNCH_DIR="$XT_ROOT/coordination/launch_generator"
fi
LAUNCH_PATH="$LAUNCH_DIR/$LAUNCH_FILE"
DEMO_PATH="$XT_ROOT/coordination/four_uav_spread_demo.py"
COMM_SH="$XT_ROOT/communication/multi_vehicle_communication.sh"

# 同步宿主机最新文件到容器（避免镜像内旧文件）
if [ -f "$HOST_DEMO_PATH" ]; then
  docker exec "$CONTAINER_NAME" bash -lc "mkdir -p '$XT_ROOT/coordination'"
  docker cp "$HOST_DEMO_PATH" "$CONTAINER_NAME:$DEMO_PATH"
fi

if [ -f "$HOST_LAUNCH_PATH" ]; then
  docker exec "$CONTAINER_NAME" bash -lc "mkdir -p '$LAUNCH_DIR'"
  docker cp "$HOST_LAUNCH_PATH" "$CONTAINER_NAME:$LAUNCH_PATH"
fi

if [ -f "$HOST_COMM_SH" ]; then
  docker exec "$CONTAINER_NAME" bash -lc "mkdir -p '$XT_ROOT/communication'"
  docker cp "$HOST_COMM_SH" "$CONTAINER_NAME:$COMM_SH"
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
if [ ! -f '$LAUNCH_PATH' ]; then
  echo '错误: launch 不存在 -> $LAUNCH_PATH'
  exit 1
fi
if [ ! -f '$DEMO_PATH' ]; then
  echo '错误: demo 脚本不存在 -> $DEMO_PATH'
  exit 1
fi

# 清理上一轮残留进程，避免模型重名与通信异常
killall -9 roslaunch rosmaster gzserver gzclient px4 mavros_node 2>/dev/null || true
sleep 2

cd $XT_ROOT
chmod +x '$DEMO_PATH'

roslaunch '$LAUNCH_PATH' gui:=${GUI} verbose:=${VERBOSE} paused:=${PAUSED} &
LAUNCH_PID=\$!

echo '[INFO] 等待仿真启动...'
sleep ${STARTUP_WAIT}

if [ "${START_COMM_BRIDGE}" = "true" ]; then
  if [ -f '$COMM_SH' ]; then
    echo '[INFO] 启动官方通信桥接 multi_vehicle_communication.sh (IRIS_NUM=4)...'
    cd '$XT_ROOT/communication'
    chmod +x '$COMM_SH'
    IRIS_NUM=4 bash '$COMM_SH' &
    COMM_PID=\$!
    sleep 2
    cd '$XT_ROOT'
  else
    echo '[WARN] 未找到通信桥脚本，跳过。'
  fi
else
  echo '[INFO] 未启用通信桥接（START_COMM_BRIDGE=false）。当前 demo 走 MAVROS 直控。'
fi

echo '[INFO] 启动四机自动示意飞行脚本（起飞->前飞->散开）...'
python3 '$DEMO_PATH'

echo '[INFO] 自动示意已完成，仿真保持运行便于录屏。'
wait \$LAUNCH_PID
"

exec docker exec -it "$CONTAINER_NAME" bash -lc "$INNER_CMD"
