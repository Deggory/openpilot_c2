#!/usr/bin/env bash
set -euo pipefail

ROOT="${K230_OPENPILOT_ROOT:-/root/openpilot_c2_k230}"
TIMEOUT_SEC="${K230_CAMERAD_SMOKE_TIMEOUT:-10}"
REQUIRED_FRAMES="${K230_CAMERAD_SMOKE_FRAMES:-5}"

cd "$ROOT"

mkdir -p /root/k230_diag 2>/dev/null || true
tmpdir="$(mktemp -d /root/k230_diag/k230_camerad_smoke.XXXXXX)"
echo "$tmpdir" >/root/k230_diag/latest_camerad_smoke 2>/dev/null || true
camerad_log="$tmpdir/camerad.log"

cleanup() {
  if [[ -n "${camerad_pid:-}" ]]; then
    kill -TERM -- "-$camerad_pid" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL -- "-$camerad_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

isp_media_server_pids() {
  ps -eo pid=,args= | awk '
    $2 == "/usr/bin/isp_media_server" || $2 == "/usr/bin/isp_media_server_debian" {print $1}
  '
}

if [[ "${K230_SKIP_ISP_START:-0}" == "1" ]]; then
  echo "K230_SKIP_ISP_START=1: leaving ISP/media server untouched"
elif [[ "${K230_FORCE_ISP_START:-0}" == "1" || -z "$(isp_media_server_pids)" ]]; then
  scripts/k230_start_isp_media_server.sh >/tmp/k230_start_isp_media_server.last.log 2>&1 || {
    cat /tmp/k230_start_isp_media_server.last.log >&2
    exit 1
  }
else
  echo "using existing isp_media_server pid(s): $(isp_media_server_pids | tr '\n' ' ')"
fi

rm -f /tmp/visionipc_camerad
setsid scripts/k230_run_openpilot.sh ./selfdrive/camerad/camerad >"$camerad_log" 2>&1 &
camerad_pid=$!

sleep 2
set +e
scripts/k230_run_openpilot.sh python3 - "$TIMEOUT_SEC" "$REQUIRED_FRAMES" <<'PY'
import sys
import time

import cereal.messaging as messaging
from cereal.visionipc.visionipc_pyx import VisionIpcClient, VisionStreamType

timeout_sec = float(sys.argv[1])
required_frames = int(sys.argv[2])

client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
deadline = time.monotonic() + timeout_sec
connected = False
while time.monotonic() < deadline:
  if client.connect(False):
    connected = True
    break
  time.sleep(0.1)

sm = messaging.SubMaster(["roadCameraState"])
frames = 0
states = 0
first_len = None
last_state_id = None

while time.monotonic() < deadline:
  if connected:
    dat = client.recv(100)
    if dat is not None:
      frames += 1
      if first_len is None:
        first_len = len(dat)
  sm.update(0)
  if sm.updated["roadCameraState"]:
    states += 1
    last_state_id = sm["roadCameraState"].frameId
  if frames >= required_frames and states >= required_frames:
    break

print(f"vipc_connected={connected} width={client.width} height={client.height} stride={client.stride}")
print(f"vipc_frames={frames} first_len={first_len} roadCameraState_updates={states} last_state_id={last_state_id}")
raise SystemExit(0 if connected and frames >= required_frames and states >= required_frames else 2)
PY
status=$?
set -e

echo "camerad_log=$camerad_log"
sed -n '1,160p' "$camerad_log"
exit "$status"
