#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" != "0" ]]; then
  echo "run as root on the K230 Ubuntu board" >&2
  exit 1
fi

isp_pids() {
  ps -eo pid=,args= | awk '
    $2 == "/usr/bin/isp_media_server" || $2 == "/usr/bin/isp_media_server_debian" {print $1}
  '
}

pids="$(isp_pids)"
if [[ -n "$pids" ]]; then
  kill -TERM $pids >/dev/null 2>&1 || true
  sleep 1
fi

pids="$(isp_pids)"
if [[ -n "$pids" ]]; then
  kill -KILL $pids >/dev/null 2>&1 || true
  sleep 1
fi

for module in vvcam_video vvcam_isp_subdev vvcam_vb vvcam_mipi vvcam_isp; do
  if lsmod | awk '{print $1}' | grep -qx "$module"; then
    rmmod "$module"
  fi
done

modprobe vvcam_isp
modprobe vvcam_mipi
modprobe vvcam_vb
modprobe vvcam_isp_subdev
modprobe vvcam_video

lsmod | grep vvcam || true
