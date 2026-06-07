#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BOARD="${K230_BOARD:-root@192.168.219.106}"
ACADOS_TAG="${K230_ACADOS_TAG:-v0.1.8}"
SRC="${K230_ACADOS_SRC:-$ROOT/build/k230_acados_${ACADOS_TAG#v}}"
TERA_SRC="${K230_TERA_RENDERER_SRC:-$SRC/interfaces/acados_template/tera_renderer}"
REMOTE_DIR="${K230_T_RENDERER_REMOTE_DIR:-/tmp/k230_t_renderer}"
OUT="$ROOT/third_party/acados/riscv64/t_renderer"
PEST_VERSION="${K230_T_RENDERER_PEST_VERSION:-2.7.15}"
BACKTRACE_VERSION="${K230_T_RENDERER_BACKTRACE_VERSION:-0.3.74}"
SSH_OPTS=(-o PreferredAuthentications=password -o PubkeyAuthentication=no -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null)
SSH=(ssh "${SSH_OPTS[@]}")
SCP=(scp "${SSH_OPTS[@]}")
RSYNC=(rsync)

if [[ -n "${K230_PASSWORD:-}" ]]; then
  SSH=(sshpass -p "$K230_PASSWORD" ssh "${SSH_OPTS[@]}")
  SCP=(sshpass -p "$K230_PASSWORD" scp "${SSH_OPTS[@]}")
  RSYNC=(sshpass -p "$K230_PASSWORD" rsync)
fi

if [[ ! -d "$TERA_SRC" ]]; then
  if [[ ! -d "$SRC/.git" ]]; then
    rm -rf "$SRC"
    git clone \
      --branch "$ACADOS_TAG" \
      --depth 1 \
      --recurse-submodules \
      --shallow-submodules \
      https://github.com/acados/acados.git \
      "$SRC"
  else
    git -C "$SRC" fetch --depth 1 origin "refs/tags/$ACADOS_TAG:refs/tags/$ACADOS_TAG"
    git -C "$SRC" checkout --detach "$ACADOS_TAG"
    git -C "$SRC" submodule update --init --recursive --depth 1
  fi
fi

if [[ ! -d "$TERA_SRC" ]]; then
  echo "missing acados tera_renderer source: $TERA_SRC" >&2
  exit 1
fi

"${SSH[@]}" "$BOARD" 'set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends ca-certificates cargo rustc
rm -rf '"$REMOTE_DIR"'
mkdir -p '"$REMOTE_DIR"'
'

"${RSYNC[@]}" -a --delete -e "ssh ${SSH_OPTS[*]}" "$TERA_SRC/" "$BOARD:$REMOTE_DIR/"

"${SSH[@]}" "$BOARD" bash -s -- "$REMOTE_DIR" "$PEST_VERSION" "$BACKTRACE_VERSION" <<'REMOTE'
set -euo pipefail
REMOTE_DIR="$1"
PEST_VERSION="$2"
BACKTRACE_VERSION="$3"

cd "$REMOTE_DIR"
cargo update -p pest_derive --precise "$PEST_VERSION"
cargo update -p pest_generator --precise "$PEST_VERSION"
cargo update -p pest_meta --precise "$PEST_VERSION"
cargo update -p pest --precise "$PEST_VERSION"
cargo update -p backtrace --precise "$BACKTRACE_VERSION"
cargo build --release
file target/release/t_renderer
REMOTE

mkdir -p "$(dirname "$OUT")"
"${SCP[@]}" "$BOARD:$REMOTE_DIR/target/release/t_renderer" "$OUT"
chmod +x "$OUT"
file "$OUT"
