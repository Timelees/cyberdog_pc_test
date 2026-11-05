#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_BIN="${WS_DIR}/install/lib/motion_manager/motion_manager"
LOCAL_SHARE="${WS_DIR}/install/share/motion_manager"
REMOTE_HOST="${REMOTE_HOST:-192.168.44.1}"
REMOTE_USER="${REMOTE_USER:-mi}"
REMOTE_ROOT="/opt/ros2/cyberdog"

for required in "$LOCAL_BIN" "$LOCAL_SHARE"; do
  if [[ ! -e "$required" ]]; then
    printf '缺少编译产物：%s\n' "$required" >&2
    exit 1
  fi
done

command -v sshpass >/dev/null || {
  printf '未安装 sshpass。请安装后重试。\n' >&2
  exit 1
}

read -r -s -p "机器人 SSH 密码 (${REMOTE_USER}@${REMOTE_HOST}): " SSHPASS
printf '\n'
export SSHPASS

SSH=(sshpass -e ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)
SCP=(sshpass -e scp -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)
STAMP="$(date +%Y%m%d_%H%M%S)"
REMOTE_STAGE="/tmp/motion_manager_deploy_${STAMP}"
LOCAL_TAR="$(mktemp "${TMPDIR:-/tmp}/motion_manager_share.XXXXXX.tar.gz")"
trap 'rm -f "$LOCAL_TAR"' EXIT

tar -C "$(dirname "$LOCAL_SHARE")" -czf "$LOCAL_TAR" motion_manager

"${SSH[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p '$REMOTE_STAGE'"
"${SCP[@]}" "$LOCAL_BIN" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_STAGE}/motion_manager"
"${SCP[@]}" "$LOCAL_TAR" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_STAGE}/share.tar.gz"

"${SSH[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "bash -s -- '$STAMP' '$REMOTE_STAGE' '$REMOTE_ROOT'" <<'REMOTE_SCRIPT'
set -euo pipefail
stamp="$1"
stage="$2"
root="$3"
lib_dir="$root/lib/motion_manager"
share_dir="$root/share/motion_manager"
backup="$root/backup/motion_manager-$stamp"

test -x "$stage/motion_manager"
test -f "$stage/share.tar.gz"
mkdir -p "$backup" "$lib_dir" "$root/share"

if [ -e "$lib_dir/motion_manager" ]; then
  cp -a "$lib_dir/motion_manager" "$backup/motion_manager"
fi
if [ -e "$share_dir" ]; then
  cp -a "$share_dir" "$backup/share_motion_manager"
fi

install -m 0755 "$stage/motion_manager" "$lib_dir/motion_manager"
rm -rf "$share_dir"
tar -C "$root/share" -xzf "$stage/share.tar.gz"
rm -rf "$stage"

printf '部署成功。备份目录：%s\n' "$backup"
printf '新二进制：%s\n' "$lib_dir/motion_manager"
printf '服务未重启；请在确认机器人安全且控制服务允许后单独重启。\n'
REMOTE_SCRIPT
