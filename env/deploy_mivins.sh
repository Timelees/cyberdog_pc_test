#!/usr/bin/env bash
set -euo pipefail

WS=/home/lee/code/cyberdog2_ws
ROBOT=mi@192.168.44.1
ROBOT_PASS=123
STAGE=/home/mi/mivins_deploy

export SSHPASS=$ROBOT_PASS
SSH_OPTS="-o StrictHostKeyChecking=no"
ssh_cmd()  { sshpass -e ssh  $SSH_OPTS "$@"; }
rsync_cmd() { sshpass -e rsync -avz -e "ssh $SSH_OPTS" "$@"; }

# Docker symlink-install: /home/builder/... -> 宿主机 $WS
resolve_src() {
    local path="$1"
    if [[ ! -L "$path" ]]; then
        echo "$path"
        return
    fi
    if [[ -e "$path" ]]; then
        echo "$path"
        return
    fi
    local target
    target=$(readlink "$path")
    target="${target//\/home\/builder\/cyberdog2_ws/$WS}"
    if [[ -e "$target" ]]; then
        echo "$target"
        return
    fi
    echo "错误: 无法解析符号链接 $path -> $(readlink "$path")" >&2
    return 1
}

stage_file() {
    local src="$1"
    local dst="$2"
    local real
    real=$(resolve_src "$src")
    mkdir -p "$(dirname "$dst")"
    cp -a "$real" "$dst"
}

stage_dir_files() {
  local src_dir="$1"
  local dst_dir="$2"
  mkdir -p "$dst_dir"
  local f
  for f in "$src_dir"/*; do
    [[ -e "$f" || -L "$f" ]] || continue
    local name
    name=$(basename "$f")
    if [[ -d "$f" && ! -L "$f" ]]; then
      stage_dir_files "$f" "$dst_dir/$name"
    else
      stage_file "$f" "$dst_dir/$name"
    fi
  done
}

LOCAL_STAGE=$(mktemp -d)
trap 'rm -rf "$LOCAL_STAGE"' EXIT

echo "==> 准备本地暂存（修复 Docker 符号链接）..."
mkdir -p "$LOCAL_STAGE/lib/vins" \
         "$LOCAL_STAGE/share/vins" \
         "$LOCAL_STAGE/share/ament_index/resource_index/"{packages,package_run_dependencies,parent_prefix_path}

# lib/vins：mivins_node 可执行文件
stage_dir_files "$WS/install/lib/vins" "$LOCAL_STAGE/lib/vins"

# libvins_lib.so
stage_file "$WS/install/lib/libvins_lib.so" "$LOCAL_STAGE/lib/libvins_lib.so"

# share/vins：launch + param + package.xml（跳过 environment/cmake 等编译期链接）
stage_dir_files "$WS/install/share/vins/launch" "$LOCAL_STAGE/share/vins/launch"
find "$LOCAL_STAGE/share/vins/launch" -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true
stage_dir_files "$WS/install/share/vins/param" "$LOCAL_STAGE/share/vins/param"
stage_file "$WS/install/share/vins/package.xml" "$LOCAL_STAGE/share/vins/package.xml" 2>/dev/null \
    || cp "$WS/src/cyberdog_mivins/mivins_ros2/src/package.xml" "$LOCAL_STAGE/share/vins/package.xml"

# ament_index
for f in \
    packages/vins \
    package_run_dependencies/vins \
    parent_prefix_path/vins
do
    stage_file "$WS/install/share/ament_index/resource_index/$f" \
               "$LOCAL_STAGE/share/ament_index/resource_index/$f"
done

echo "==> 上传到机器狗 $ROBOT ..."
ssh_cmd "$ROBOT" "rm -rf $STAGE"
rsync_cmd "$LOCAL_STAGE/" "$ROBOT:$STAGE/"

echo "==> 安装到 /opt/ros2/cyberdog ..."
ssh_cmd "$ROBOT" bash -s <<EOF
set -e
PASS='$ROBOT_PASS'
STAGE=/home/mi/mivins_deploy
DEST=/opt/ros2/cyberdog
sudo_cmd() { echo "\$PASS" | sudo -S -p '' "\$@"; }

# 正在运行的可执行文件无法被 cp 覆盖（Text file busy），部署前先停掉 mivins 节点
echo "==> 停止 mivins 相关节点..."
sudo_cmd pkill -f '/opt/ros2/cyberdog/lib/vins/mivins_node' 2>/dev/null || true
sleep 1
if sudo_cmd pgrep -f '/opt/ros2/cyberdog/lib/vins/mivins_node' >/dev/null; then
    echo "警告: mivins_node 仍在运行，强制结束..." >&2
    sudo_cmd pkill -9 -f '/opt/ros2/cyberdog/lib/vins/mivins_node' 2>/dev/null || true
    sleep 1
fi

sudo_cmd rm -rf "\$DEST/lib/vins.deploy_tmp"
sudo_cmd cp -r "\$STAGE/lib/vins" "\$DEST/lib/vins.deploy_tmp"
sudo_cmd rm -rf "\$DEST/lib/vins"
sudo_cmd mv "\$DEST/lib/vins.deploy_tmp" "\$DEST/lib/vins"

sudo_cmd cp "\$STAGE/lib/libvins_lib.so" "\$DEST/lib/libvins_lib.so.deploy_tmp"
sudo_cmd mv "\$DEST/lib/libvins_lib.so.deploy_tmp" "\$DEST/lib/libvins_lib.so"

sudo_cmd mkdir -p "\$DEST/share/vins"
sudo_cmd cp -r "\$STAGE/share/vins/launch"     "\$DEST/share/vins/"
sudo_cmd cp -r "\$STAGE/share/vins/param"      "\$DEST/share/vins/"
sudo_cmd cp    "\$STAGE/share/vins/package.xml" "\$DEST/share/vins/"

for sub in packages package_run_dependencies parent_prefix_path; do
    if [ -f "\$STAGE/share/ament_index/resource_index/\$sub/vins" ]; then
        sudo_cmd mkdir -p "\$DEST/share/ament_index/resource_index/\$sub"
        sudo_cmd cp "\$STAGE/share/ament_index/resource_index/\$sub/vins" \
                    "\$DEST/share/ament_index/resource_index/\$sub/"
    fi
done

rm -rf "\$STAGE"
echo "mivins 部署完成"
EOF

echo "==> 完成"
