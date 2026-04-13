#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${1:-xtdrone_sim}"

upsert_block() {
  local file_path="$1"
  local ros_master_uri="$2"
  local ros_ip="$3"

  if [[ ! -f "$file_path" ]]; then
    touch "$file_path"
  fi

  sed -i '/# XTDRONE_ROS_BEGIN/,/# XTDRONE_ROS_END/d' "$file_path"
  cat >> "$file_path" <<EOF
# XTDRONE_ROS_BEGIN
export ROS_MASTER_URI=$ros_master_uri
export ROS_IP=$ros_ip
export ROS_HOSTNAME=$ros_ip
# XTDRONE_ROS_END
EOF
}

if ! sudo docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "错误: 容器未运行 -> $CONTAINER_NAME"
  echo "请先执行 ./start_xtdrone_container.sh"
  exit 1
fi

HOST_IP="$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++){if($i=="src"){print $(i+1); exit}}}')"
HOST_IP="${HOST_IP:-127.0.0.1}"

DOCKER_NET_MODE="$(sudo docker inspect -f '{{.HostConfig.NetworkMode}}' "$CONTAINER_NAME")"
DOCKER_IP="$(sudo docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$CONTAINER_NAME")"

if [[ -z "$DOCKER_IP" ]]; then
  if [[ "$DOCKER_NET_MODE" == "host" ]]; then
    DOCKER_IP="127.0.0.1"
  else
    echo "错误: 无法获取容器IP，请检查容器网络。"
    exit 1
  fi
fi

echo "检测到:"
echo "  CONTAINER_NAME = $CONTAINER_NAME"
echo "  DOCKER_IP      = $DOCKER_IP"
echo "  HOST_IP        = $HOST_IP"

upsert_block "$HOME/.bashrc" "http://$DOCKER_IP:11311" "$HOST_IP"

# 容器内 .bashrc
sudo docker exec \
  -e XT_ROS_MASTER_URI="http://$DOCKER_IP:11311" \
  -e XT_ROS_IP="$DOCKER_IP" \
  "$CONTAINER_NAME" \
  bash -lc 'set -euo pipefail; FILE="$HOME/.bashrc"; sed -i "/# XTDRONE_ROS_BEGIN/,/# XTDRONE_ROS_END/d" "$FILE"; { echo "# XTDRONE_ROS_BEGIN"; echo "export ROS_MASTER_URI=$XT_ROS_MASTER_URI"; echo "export ROS_IP=$XT_ROS_IP"; echo "export ROS_HOSTNAME=$XT_ROS_IP"; echo "# XTDRONE_ROS_END"; } >> "$FILE"'

echo "已写入宿主机和容器的 .bashrc（如需生效请执行: source ~/.bashrc）"
