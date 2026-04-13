#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-xtdrone:1.3}"
CONTAINER_NAME="${CONTAINER_NAME:-xtdrone_sim}"
HOST_WORKSPACE="${HOST_WORKSPACE:-$HOME/XTDrone_ws}"

mkdir -p "$HOST_WORKSPACE"

if ! sudo docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "错误: 镜像不存在 -> $IMAGE"
  echo "请先执行: ./import_xtdrone_image.sh <镜像tar路径>"
  exit 1
fi

# Gazebo/X11 显示
if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

if sudo docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "容器已在运行，直接进入: $CONTAINER_NAME"
  exec sudo docker exec -it "$CONTAINER_NAME" bash
fi

if sudo docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "发现已停止容器，正在启动: $CONTAINER_NAME"
  sudo docker start "$CONTAINER_NAME" >/dev/null
  exec sudo docker exec -it "$CONTAINER_NAME" bash
fi

echo "创建并启动容器: $CONTAINER_NAME"
exec sudo docker run -it \
  --name "$CONTAINER_NAME" \
  --gpus all \
  --network host \
  --ipc host \
  --privileged \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOST_WORKSPACE":/workspace \
  -w /workspace \
  "$IMAGE" \
  bash
