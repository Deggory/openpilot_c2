#!/usr/bin/env bash
set -euo pipefail

ROOT="${K230_OPENPILOT_ROOT:-/root/openpilot_c2_k230}"
DURATION_SEC="${K230_MANAGER_SMOKE_DURATION:-300}"
REQUIRED_UPDATES="${K230_MANAGER_SMOKE_UPDATES:-5}"
BLOCK="${BLOCK:-modeld,dmonitoringmodeld,camerad,controlsd,plannerd,radard}"
FAKE_STARTED="${K230_MANAGER_FAKE_STARTED:-0}"
EXPECT_SERVICES="${K230_MANAGER_EXPECT_SERVICES:-}"
SKIP_UI="${K230_MANAGER_SKIP_UI:-0}"
GRACEFUL_CLEANUP="${K230_MANAGER_GRACEFUL_CLEANUP:-0}"
NOBOARD_FLAG="${K230_MANAGER_NOBOARD:-1}"

cd "$ROOT"

tmpdir="$(mktemp -d /tmp/k230_manager_smoke.XXXXXX)"
manager_log="$tmpdir/manager.log"
fake_started_log="$tmpdir/fake_started.log"

cleanup() {
  if [[ -n "${fake_started_pid:-}" ]]; then
    kill -TERM -- "-$fake_started_pid" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL -- "-$fake_started_pid" >/dev/null 2>&1 || true
  fi
  if [[ -n "${manager_pid:-}" ]]; then
    if [[ "$GRACEFUL_CLEANUP" == "1" ]]; then
      kill -TERM -- "-$manager_pid" >/dev/null 2>&1 || true
      sleep 1
    fi
    kill -KILL -- "-$manager_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

autostart_isp="${K230_AUTOSTART_ISP:-0}"
if [[ "$FAKE_STARTED" == "1" && -n "$EXPECT_SERVICES" ]]; then
  SKIP_UI="${K230_MANAGER_SKIP_UI:-1}"
fi
if [[ "$SKIP_UI" == "1" && ",$BLOCK," != *",ui,"* ]]; then
  BLOCK="${BLOCK},ui"
fi

manager_env=(NO_WATCHDOG=1 BLOCK="$BLOCK" K230_AUTOSTART_ISP="$autostart_isp" K230_MANAGER_SKIP_UI="$SKIP_UI")
if [[ "$NOBOARD_FLAG" == "1" ]]; then
  manager_env+=(NOBOARD=1)
fi

setsid env "${manager_env[@]}" \
  scripts/k230_run_openpilot.sh python3 selfdrive/manager/manager.py >"$manager_log" 2>&1 &
manager_pid=$!

if [[ "$FAKE_STARTED" == "1" ]]; then
  fake_env=(NO_WATCHDOG=1 BLOCK="$BLOCK")
  if [[ "$NOBOARD_FLAG" == "1" ]]; then
    fake_env+=(NOBOARD=1)
  fi
  setsid env "${fake_env[@]}" \
    scripts/k230_run_openpilot.sh python3 - >"$fake_started_log" 2>&1 <<'PY' &
import signal
import time

import cereal.messaging as messaging

running = True

def stop(_signum, _frame):
  global running
  running = False

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

pm = messaging.PubMaster(["deviceState"])
while running:
  msg = messaging.new_message("deviceState")
  msg.deviceState.started = True
  pm.send("deviceState", msg)
  time.sleep(0.05)
PY
  fake_started_pid=$!
fi

sleep 6
set +e
sub_env=(NO_WATCHDOG=1 BLOCK="$BLOCK")
if [[ "$NOBOARD_FLAG" == "1" ]]; then
  sub_env+=(NOBOARD=1)
fi
env "${sub_env[@]}" \
  scripts/k230_run_openpilot.sh python3 - "$DURATION_SEC" "$REQUIRED_UPDATES" "$EXPECT_SERVICES" <<'PY'
import sys
import time

import cereal.messaging as messaging

duration_sec = float(sys.argv[1])
required_updates = int(sys.argv[2])
expect_services = [s for s in sys.argv[3].split(",") if s]
services = ["managerState"] + [s for s in expect_services if s != "managerState"]
sm = messaging.SubMaster(services)
deadline = time.monotonic() + duration_sec
updates = {s: 0 for s in services}
last_running = []
last_count = 0
last_frames = {}

while time.monotonic() < deadline:
  sm.update(1000)
  if sm.updated["managerState"]:
    updates["managerState"] += 1
    state = sm["managerState"]
    last_count = len(state.processes)
    last_running = [p.name for p in state.processes if p.running]
  for service in expect_services:
    if sm.updated[service]:
      updates[service] += 1
      frame_id = getattr(sm[service], "frameId", None)
      if frame_id is not None:
        last_frames[service] = frame_id

print(f"managerState_updates={updates['managerState']} processes={last_count} running={','.join(last_running)}")
for service in expect_services:
  suffix = f" lastFrame={last_frames[service]}" if service in last_frames else ""
  print(f"{service}_updates={updates[service]}{suffix}")
ok = updates["managerState"] >= required_updates and all(updates[s] >= 1 for s in expect_services)
raise SystemExit(0 if ok else 2)
PY
status=$?
set -e

echo "manager_log=$manager_log"
if [[ "$FAKE_STARTED" == "1" ]]; then
  echo "fake_started_log=$fake_started_log"
fi
exit "$status"
