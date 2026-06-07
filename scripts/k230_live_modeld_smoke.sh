#!/usr/bin/env bash
set -euo pipefail

ROOT="${K230_OPENPILOT_ROOT:-/root/openpilot_c2_k230}"
SAMPLES="${K230_LIVE_MODELD_SMOKE_SAMPLES:-20}"
TIMEOUT_SEC="${K230_LIVE_MODELD_SMOKE_TIMEOUT:-60}"

cd "$ROOT"

if [[ -n "${K230_LIVE_MODELD_SMOKE_DIR:-}" ]]; then
  tmpdir="$K230_LIVE_MODELD_SMOKE_DIR"
  mkdir -p "$tmpdir"
else
  tmpdir="$(mktemp -d /tmp/k230_live_modeld_smoke.XXXXXX)"
fi
mkdir -p /root/k230_diag 2>/dev/null || true
echo "$tmpdir" >/root/k230_diag/latest_live_modeld_smoke 2>/dev/null || true
camerad_log="$tmpdir/camerad.log"
modeld_log="$tmpdir/modeld.log"
carstate_log="$tmpdir/carstate.log"

cleanup() {
  for pid_var in modeld_pid camerad_pid carstate_pid; do
    pid="${!pid_var:-}"
    if [[ -n "$pid" ]]; then
      kill -KILL -- "-$pid" >/dev/null 2>&1 || true
    fi
  done
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

setsid scripts/k230_run_openpilot.sh python3 - >"$carstate_log" 2>&1 <<'PY' &
import signal
import os
import time
import math

import cereal.messaging as messaging
from common.transformations.camera import get_view_frame_from_road_frame
from common.transformations.model import model_height

running = True
publish_calibration = os.getenv("K230_LIVE_MODELD_PUBLISH_CALIBRATION", "0") == "1"
services = ["carState"] + (["liveCalibration"] if publish_calibration else [])
roll = math.radians(float(os.getenv("K230_LIVE_MODELD_CALIB_ROLL_DEG", "0")))
pitch = math.radians(float(os.getenv("K230_LIVE_MODELD_CALIB_PITCH_DEG", "0")))
yaw = math.radians(float(os.getenv("K230_LIVE_MODELD_CALIB_YAW_DEG", "0")))
extrinsic = get_view_frame_from_road_frame(roll, pitch, yaw, model_height).flatten().tolist()

def stop(_signum, _frame):
  global running
  running = False

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

pm = messaging.PubMaster(services)
last_calib = 0.0
while running:
  msg = messaging.new_message("carState")
  msg.carState.vEgo = 0.0
  msg.carState.vEgoRaw = 0.0
  msg.carState.standstill = True
  pm.send("carState", msg)

  now = time.monotonic()
  if publish_calibration and now - last_calib >= 0.25:
    calib = messaging.new_message("liveCalibration")
    calib.liveCalibration.validBlocks = 100
    calib.liveCalibration.calStatus = 1
    calib.liveCalibration.calPerc = 100
    calib.liveCalibration.extrinsicMatrix = extrinsic
    calib.liveCalibration.rpyCalib = [roll, pitch, yaw]
    calib.liveCalibration.rpyCalibSpread = [0.0, 0.0, 0.0]
    pm.send("liveCalibration", calib)
    last_calib = now

  time.sleep(0.02)
PY
carstate_pid=$!

sleep 2
setsid scripts/k230_run_openpilot.sh selfdrive/modeld/modeld >"$modeld_log" 2>&1 &
modeld_pid=$!

set +e
scripts/k230_run_openpilot.sh python3 - "$SAMPLES" "$TIMEOUT_SEC" <<'PY'
import statistics
import sys
import time

import cereal.messaging as messaging

samples = int(sys.argv[1])
timeout_sec = float(sys.argv[2])
sm = messaging.SubMaster(["roadCameraState", "modelV2", "carState"])
deadline = time.monotonic() + timeout_sec
road_updates = 0
car_updates = 0
exec_times = []
model_frames = []
road_last = None

while time.monotonic() < deadline and len(exec_times) < samples:
  sm.update(1000)
  if sm.updated["roadCameraState"]:
    road_updates += 1
    road_last = int(sm["roadCameraState"].frameId)
  if sm.updated["carState"]:
    car_updates += 1
  if sm.updated["modelV2"]:
    msg = sm["modelV2"]
    exec_times.append(float(msg.modelExecutionTime))
    model_frames.append(int(msg.frameId))

print(f"roadCameraState_updates={road_updates} lastFrame={road_last}")
print(f"carState_updates={car_updates}")
if exec_times:
  print(f"modelV2_count={len(exec_times)} firstFrame={model_frames[0]} lastFrame={model_frames[-1]}")
  print(f"modelExecutionTime_s avg={statistics.mean(exec_times):.4f} min={min(exec_times):.4f} max={max(exec_times):.4f}")
else:
  print("modelV2_count=0")
raise SystemExit(0 if len(exec_times) >= samples else 2)
PY
status=$?
set -e

ps -p "$camerad_pid" -o pid=,pcpu=,rss=,vsz=,comm= | awk '{printf "camerad_ps pid=%s cpu_pct=%s rss_kb=%s vsz_kb=%s comm=%s\n", $1, $2, $3, $4, $5}' || true
ps -p "$modeld_pid" -o pid=,pcpu=,rss=,vsz=,comm= | awk '{printf "modeld_ps pid=%s cpu_pct=%s rss_kb=%s vsz_kb=%s comm=%s\n", $1, $2, $3, $4, $5}' || true
echo "camerad_log=$camerad_log"
echo "modeld_log=$modeld_log"
echo "carstate_log=$carstate_log"
exit "$status"
