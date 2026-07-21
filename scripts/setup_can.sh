#!/usr/bin/env bash
set -euo pipefail

mode="${1:-real}"
interface="${2:-can0}"
bitrate="${3:-1000000}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "请以 root 运行，例如：sudo bash $0 ${mode} ${interface}" >&2
  exit 1
fi

case "${mode}" in
  real)
    ip link set "${interface}" down 2>/dev/null || true
    if ! [[ "${bitrate}" =~ ^[1-9][0-9]*$ ]]; then
      echo "bitrate 必须是正整数" >&2
      exit 2
    fi
    ip link set "${interface}" type can bitrate "${bitrate}" restart-ms 100
    ip link set "${interface}" txqueuelen 1000
    ip link set "${interface}" up
    ;;
  vcan)
    modprobe vcan
    if ! ip link show "${interface}" >/dev/null 2>&1; then
      ip link add dev "${interface}" type vcan
    fi
    ip link set "${interface}" up
    ;;
  down)
    ip link set "${interface}" down
    ;;
  *)
    echo "用法：$0 real|vcan|down [can0|vcan0] [bitrate]" >&2
    exit 2
    ;;
esac

ip -details -statistics link show "${interface}"
