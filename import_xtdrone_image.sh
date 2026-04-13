#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "用法: $0 <xtdrone镜像tar路径>"
  exit 1
fi

TAR_PATH="$1"
if [[ ! -f "$TAR_PATH" ]]; then
  echo "错误: 文件不存在 -> $TAR_PATH"
  exit 1
fi

echo "[1/2] 导入镜像: $TAR_PATH"
sudo docker load -i "$TAR_PATH"

echo "[2/2] 当前与 xtdrone 相关的镜像:"
sudo docker images --format '{{.Repository}}:{{.Tag}}\t{{.Size}}' | grep -i xtdrone || echo "未发现 xtdrone 标签，请手动确认 load 输出并执行 docker tag"
