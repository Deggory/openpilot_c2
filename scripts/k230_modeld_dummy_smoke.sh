#!/usr/bin/env bash
set -euo pipefail

ROOT="${K230_OPENPILOT_ROOT:-/root/openpilot_c2_k230}"
SAMPLES="${K230_MODELD_SMOKE_SAMPLES:-5}"
FPS="${K230_MODELD_SMOKE_FPS:-2}"
WIDTH="${K230_MODELD_SMOKE_WIDTH:-1928}"
HEIGHT="${K230_MODELD_SMOKE_HEIGHT:-1208}"
TIMEOUT_SEC="${K230_MODELD_SMOKE_TIMEOUT:-90}"

cd "$ROOT"

tmpdir="$(mktemp -d /tmp/k230_modeld_smoke.XXXXXX)"
pub_log="$tmpdir/dummy_vipc.log"
modeld_log="$tmpdir/modeld.log"

cleanup() {
  if [[ -n "${modeld_pid:-}" ]]; then
    kill -TERM -- "-$modeld_pid" >/dev/null 2>&1 || true
  fi
  if [[ -n "${pub_pid:-}" ]]; then
    kill -TERM -- "-$pub_pid" >/dev/null 2>&1 || true
  fi
  sleep 1
  if [[ -n "${modeld_pid:-}" ]]; then
    kill -KILL -- "-$modeld_pid" >/dev/null 2>&1 || true
  fi
  if [[ -n "${pub_pid:-}" ]]; then
    kill -KILL -- "-$pub_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

setsid scripts/k230_run_openpilot.sh python3 - "$WIDTH" "$HEIGHT" "$FPS" >"$pub_log" 2>&1 <<'PY' &
import os
import sys
import time
import math

import cereal.messaging as messaging
from common.transformations.camera import get_view_frame_from_road_frame
from common.transformations.model import model_height
from cereal.visionipc.visionipc_pyx import VisionIpcServer, VisionStreamType

width = int(sys.argv[1])
height = int(sys.argv[2])
fps = float(sys.argv[3])
period = 1.0 / fps
publish_calibration = os.getenv("K230_MODELD_SMOKE_PUBLISH_CALIBRATION", "0") == "1"
services = ["roadCameraState", "carState"] + (["liveCalibration"] if publish_calibration else [])
roll = math.radians(float(os.getenv("K230_MODELD_SMOKE_CALIB_ROLL_DEG", "0")))
pitch = math.radians(float(os.getenv("K230_MODELD_SMOKE_CALIB_PITCH_DEG", "0")))
yaw = math.radians(float(os.getenv("K230_MODELD_SMOKE_CALIB_YAW_DEG", "0")))
extrinsic = get_view_frame_from_road_frame(roll, pitch, yaw, model_height).flatten().tolist()

frame = (bytes([64]) * (width * height)) + (bytes([128]) * (width * height // 2))
server = VisionIpcServer("camerad")
server.create_buffers(VisionStreamType.VISION_STREAM_ROAD, 8, False, width, height)
server.start_listener()
pm = messaging.PubMaster(services)

frame_id = 0
last_calib = 0.0
while True:
  ts = int(time.monotonic() * 1e9)
  server.send(VisionStreamType.VISION_STREAM_ROAD, frame, frame_id, ts, ts)
  msg = messaging.new_message("roadCameraState")
  msg.roadCameraState = {
    "frameId": frame_id,
    "transform": [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0],
  }
  pm.send("roadCameraState", msg)

  car_state = messaging.new_message("carState")
  car_state.carState.vEgo = 0.0
  car_state.carState.vEgoRaw = 0.0
  car_state.carState.standstill = True
  pm.send("carState", car_state)

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

  frame_id += 1
  time.sleep(period)
PY
pub_pid=$!

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
sm = messaging.SubMaster(["modelV2", "carState"])
deadline = time.monotonic() + timeout_sec
exec_times = []
frame_ids = []
car_state_count = 0

while time.monotonic() < deadline and (len(exec_times) < samples or car_state_count < samples):
  sm.update(1000)
  if sm.updated["modelV2"]:
    msg = sm["modelV2"]
    exec_times.append(float(msg.modelExecutionTime))
    frame_ids.append(int(msg.frameId))
  if sm.updated["carState"]:
    car_state_count += 1

if not exec_times:
  print("modelV2_count=0")
  raise SystemExit(2)

print(f"modelV2_count={len(exec_times)} firstFrame={frame_ids[0]} lastFrame={frame_ids[-1]}")
print(f"carState_count={car_state_count}")
print(f"modelExecutionTime_s avg={statistics.mean(exec_times):.4f} min={min(exec_times):.4f} max={max(exec_times):.4f}")
raise SystemExit(0 if len(exec_times) >= samples and car_state_count >= samples else 2)
PY
status=$?
set -e

ps -p "$modeld_pid" -o pid=,pcpu=,rss=,vsz=,comm= | awk '{printf "modeld_ps pid=%s cpu_pct=%s rss_kb=%s vsz_kb=%s comm=%s\n", $1, $2, $3, $4, $5}' || true
echo "modeld_log=$modeld_log"
echo "dummy_vipc_log=$pub_log"
exit "$status"
