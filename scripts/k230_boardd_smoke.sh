#!/usr/bin/env bash
set -euo pipefail

ROOT="${K230_OPENPILOT_ROOT:-/root/openpilot_c2_k230}"
TIMEOUT_SEC="${K230_BOARDD_SMOKE_TIMEOUT:-15}"
REQUIRED_PANDA_STATES="${K230_BOARDD_SMOKE_PANDA_STATES:-3}"

cd "$ROOT"

tmpdir="$(mktemp -d /tmp/k230_boardd_smoke.XXXXXX)"
boardd_log="$tmpdir/boardd.log"

cleanup() {
  if [[ -n "${boardd_pid:-}" ]]; then
    kill -KILL -- "-$boardd_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

serials="$(scripts/k230_run_openpilot.sh python3 - <<'PY'
from panda import Panda
print(" ".join(Panda.list()))
PY
)"

if [[ -z "$serials" ]]; then
  echo "panda_serials="
  exit 2
fi
echo "panda_serials=$serials"

setsid scripts/k230_run_openpilot.sh selfdrive/boardd/boardd $serials >"$boardd_log" 2>&1 &
boardd_pid=$!

set +e
scripts/k230_run_openpilot.sh python3 - "$TIMEOUT_SEC" "$REQUIRED_PANDA_STATES" <<'PY'
import sys
import time

import cereal.messaging as messaging

timeout_sec = float(sys.argv[1])
required_panda_states = int(sys.argv[2])
sm = messaging.SubMaster(["pandaStates", "can"])
deadline = time.monotonic() + timeout_sec
panda_state_updates = 0
can_updates = 0
last = None

while time.monotonic() < deadline and panda_state_updates < required_panda_states:
  sm.update(1000)
  if sm.updated["pandaStates"]:
    panda_state_updates += 1
    last = sm["pandaStates"]
  if sm.updated["can"]:
    can_updates += 1

print(f"pandaStates_updates={panda_state_updates} can_updates={can_updates}")
if last is not None and len(last) > 0:
  p = last[0]
  print(f"pandaType={p.pandaType} safetyModel={p.safetyModel} ignitionLine={p.ignitionLine} ignitionCan={p.ignitionCan}")
  print(f"heartbeatLost={p.heartbeatLost} harness={p.harnessStatus} canRxErrs={p.canRxErrs} canSendErrs={p.canSendErrs}")
raise SystemExit(0 if panda_state_updates >= required_panda_states else 2)
PY
status=$?
set -e

ps -p "$boardd_pid" -o pid=,pcpu=,rss=,vsz=,comm= | awk '{printf "boardd_ps pid=%s cpu_pct=%s rss_kb=%s vsz_kb=%s comm=%s\n", $1, $2, $3, $4, $5}' || true
echo "boardd_log=$boardd_log"
sed -n '1,120p' "$boardd_log"
exit "$status"
