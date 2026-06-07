#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" != "0" ]]; then
  echo "run as root on the K230 Ubuntu board" >&2
  exit 1
fi

DEFAULT_SERVER="/usr/bin/isp_media_server"
if [[ -x "/usr/bin/isp_media_server_debian" ]]; then
  DEFAULT_SERVER="/usr/bin/isp_media_server_debian"
fi

SERVER="${K230_ISP_MEDIA_SERVER:-$DEFAULT_SERVER}"
DRIVER="${K230_ISP_SENSOR_DRIVER:-/usr/lib/riscv64-linux-gnu/libvvcam.so}"
PROC_NODE="${K230_ISP_PROC_NODE:-/proc/vsi/isp_subdev0}"
LOG_OUT="${K230_ISP_LOG_OUT:-/tmp/isp_media_server.log}"
LOG_ERR="${K230_ISP_LOG_ERR:-/tmp/isp.err.log}"
SERVER_NAME="$(basename "$SERVER")"

if [[ ! -x "$SERVER" ]]; then
  echo "missing executable ISP media server: $SERVER" >&2
  echo "install the K230 vvcam prebuilt isp_media_server first" >&2
  exit 1
fi
if [[ ! -r "$DRIVER" ]]; then
  echo "missing vvcam sensor driver: $DRIVER" >&2
  exit 1
fi

current_sensor() {
  awk -F: '/sensor[[:space:]]*:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' "$PROC_NODE" 2>/dev/null || true
}

current_mode() {
  awk -F: '/mode[[:space:]]*:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' "$PROC_NODE" 2>/dev/null || true
}

SENSOR="${K230_ISP_SENSOR:-$(current_sensor)}"
SENSOR="${SENSOR:-gc2093}"
MODE="${K230_ISP_MODE:-0}"

case "$SENSOR" in
  gc2093)
    XML="${K230_ISP_XML:-/etc/vvcam/gc2093-1920x1080.xml}"
    MANU_JSON="${K230_ISP_MANU_JSON:-/etc/vvcam/gc2093-1920x1080_manual.json}"
    AUTO_JSON="${K230_ISP_AUTO_JSON:-/etc/vvcam/gc2093-1920x1080_auto.json}"
    ;;
  gc2053)
    XML="${K230_ISP_XML:-/etc/vvcam/gc2053-1920x1080.xml}"
    MANU_JSON="${K230_ISP_MANU_JSON:-/etc/vvcam/gc2053-1920x1080_manual.json}"
    AUTO_JSON="${K230_ISP_AUTO_JSON:-/etc/vvcam/gc2053-1920x1080_auto.json}"
    ;;
  imx335)
    XML="${K230_ISP_XML:-/etc/vvcam/imx335-1920x1080.xml}"
    MANU_JSON="${K230_ISP_MANU_JSON:-/etc/vvcam/imx335-1920x1080_manual.json}"
    AUTO_JSON="${K230_ISP_AUTO_JSON:-/etc/vvcam/imx335-1920x1080_auto.json}"
    ;;
  ov5647)
    XML="${K230_ISP_XML:-/etc/vvcam/ov5647.xml}"
    MANU_JSON="${K230_ISP_MANU_JSON:-/etc/vvcam/ov5647.manual.json}"
    AUTO_JSON="${K230_ISP_AUTO_JSON:-/etc/vvcam/ov5647.auto.json}"
    ;;
  bf3238)
    XML="${K230_ISP_XML:-/etc/vvcam/bf3238-1920x1080.xml}"
    MANU_JSON="${K230_ISP_MANU_JSON:-/etc/vvcam/bf3238-1920x1080_manual.json}"
    AUTO_JSON="${K230_ISP_AUTO_JSON:-/etc/vvcam/bf3238-1920x1080_auto.json}"
    ;;
  *)
    echo "unknown K230_ISP_SENSOR: $SENSOR" >&2
    exit 1
    ;;
esac

for path in "$XML" "$MANU_JSON" "$AUTO_JSON"; do
  if [[ ! -r "$path" ]]; then
    echo "missing ISP config file: $path" >&2
    exit 1
  fi
done

server_pids() {
  ps -eo pid=,args= | awk -v server="$SERVER" '
    $2 == server || $2 == "/usr/bin/isp_media_server" || $2 == "/usr/bin/isp_media_server_debian" {print $1}
  '
}

stop_server() {
  local pids
  pids="$(server_pids)"
  if [[ -z "$pids" ]]; then
    return
  fi

  kill -TERM $pids >/dev/null 2>&1 || true
  for _ in {1..30}; do
    if [[ -z "$(server_pids)" ]]; then
      return
    fi
    sleep 0.1
  done

  pids="$(server_pids)"
  if [[ -n "$pids" ]]; then
    kill -KILL $pids >/dev/null 2>&1 || true
  fi
  for _ in {1..10}; do
    if [[ -z "$(server_pids)" ]]; then
      return
    fi
    sleep 0.1
  done

  echo "failed to stop existing $SERVER_NAME processes: $(server_pids | tr '\n' ' ')" >&2
  exit 1
}

pid_count="$(server_pids | wc -l | tr -d ' ')"
if [[ "${K230_RESTART_ISP:-0}" == "1" || "$pid_count" -gt 1 ]]; then
  stop_server
fi

started_server=0
if [[ -z "$(server_pids)" ]]; then
  : >"$LOG_OUT"
  : >"$LOG_ERR"
  ISP_MEDIA_SENSOR_DRIVER="$DRIVER" "$SERVER" >"$LOG_OUT" 2>"$LOG_ERR" &
  started_server=1
  sleep "${K230_ISP_START_DELAY:-1}"
fi

need_config=0
if [[ "$started_server" == "1" || "${K230_FORCE_ISP_CONFIG:-0}" == "1" ]]; then
  need_config=1
elif [[ "$(current_sensor)" != "$SENSOR" || "$(current_mode)" != "$MODE" ]]; then
  need_config=1
fi

if [[ "$need_config" == "1" ]]; then
  if [[ -w "$PROC_NODE" ]]; then
    echo "0 sensor=$SENSOR" >"$PROC_NODE"
    echo "0 mode=$MODE" >"$PROC_NODE"
    echo "0 xml=$XML" >"$PROC_NODE"
    echo "0 manu_json=$MANU_JSON" >"$PROC_NODE"
    echo "0 auto_json=$AUTO_JSON" >"$PROC_NODE"
  else
    echo "cannot write ISP proc node: $PROC_NODE" >&2
    exit 1
  fi
  sleep "${K230_ISP_READY_DELAY:-1}"
fi

if [[ -z "$(server_pids)" ]]; then
  echo "$SERVER_NAME exited during sensor setup" >&2
  echo "--- $LOG_OUT ---" >&2
  tail -40 "$LOG_OUT" >&2 || true
  echo "--- $LOG_ERR ---" >&2
  tail -40 "$LOG_ERR" >&2 || true
  exit 1
fi

pid="$(server_pids | head -1 || true)"
echo "isp_media_server_pid=$pid sensor=$SENSOR mode=$MODE configured=$need_config"
echo "isp_media_server_log=$LOG_OUT"
echo "isp_media_server_err=$LOG_ERR"
sed -n '1,16p' "$PROC_NODE" || true
