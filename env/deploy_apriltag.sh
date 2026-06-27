#!/usr/bin/env bash
set -euo pipefail

WS=/home/lee/code/cyberdog2_ws
ROBOT=mi@192.168.44.1
ROBOT_PASS=123
STAGE=/home/mi/apriltag_deploy

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
mkdir -p "$LOCAL_STAGE/lib" \
         "$LOCAL_STAGE/share/apriltag_ros" \
         "$LOCAL_STAGE/share/apriltag_msgs/msg" \
         "$LOCAL_STAGE/share/ament_index/resource_index/"{packages,rosidl_interfaces,package_run_dependencies,parent_prefix_path,rclcpp_components}

# lib
stage_dir_files "$WS/install/lib/apriltag_ros" "$LOCAL_STAGE/lib/apriltag_ros"
stage_file "$WS/install/lib/libAprilTagNode.so" "$LOCAL_STAGE/lib/libAprilTagNode.so"
for lib in "$WS"/install/lib/libapriltag.so*; do
    [[ -e "$lib" || -L "$lib" ]] || continue
    stage_file "$lib" "$LOCAL_STAGE/lib/$(basename "$lib")"
done
for lib in "$WS"/install/lib/libapriltag_msgs__rosidl*.so; do
    [[ -e "$lib" || -L "$lib" ]] || continue
    stage_file "$lib" "$LOCAL_STAGE/lib/$(basename "$lib")"
done
# Python 绑定：ros2 topic echo / ros2 interface show 等 CLI 需要
stage_file "$WS/install/lib/libapriltag_msgs__python.so" "$LOCAL_STAGE/lib/libapriltag_msgs__python.so"
if [[ -d "$WS/install/lib/python3.6/site-packages/apriltag_msgs" ]]; then
    stage_dir_files "$WS/install/lib/python3.6/site-packages/apriltag_msgs" \
                    "$LOCAL_STAGE/lib/python3.6/site-packages/apriltag_msgs"
    find "$LOCAL_STAGE/lib/python3.6/site-packages/apriltag_msgs" \
         -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true
fi

# share/apriltag_ros：launch + cfg + package.xml（跳过 __pycache__）
stage_dir_files "$WS/install/share/apriltag_ros/launch" "$LOCAL_STAGE/share/apriltag_ros/launch"
find "$LOCAL_STAGE/share/apriltag_ros/launch" -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true
stage_dir_files "$WS/install/share/apriltag_ros/cfg" "$LOCAL_STAGE/share/apriltag_ros/cfg"
stage_file "$WS/install/share/apriltag_ros/package.xml" "$LOCAL_STAGE/share/apriltag_ros/package.xml" 2>/dev/null \
    || cp "$WS/src/apriltag_ros/package.xml" "$LOCAL_STAGE/share/apriltag_ros/package.xml"

# share/apriltag_msgs：msg + package.xml（跳过 environment/cmake 等编译期链接）
stage_dir_files "$WS/install/share/apriltag_msgs/msg" "$LOCAL_STAGE/share/apriltag_msgs/msg"
stage_file "$WS/install/share/apriltag_msgs/package.xml" "$LOCAL_STAGE/share/apriltag_msgs/package.xml" 2>/dev/null \
    || cp "$WS/src/apriltag_msgs/package.xml" "$LOCAL_STAGE/share/apriltag_msgs/package.xml"

# ament_index
for f in \
    packages/apriltag_msgs packages/apriltag_ros \
    rosidl_interfaces/apriltag_msgs \
    package_run_dependencies/apriltag_msgs package_run_dependencies/apriltag_ros \
    parent_prefix_path/apriltag_msgs parent_prefix_path/apriltag_ros \
    rclcpp_components/apriltag_ros
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
STAGE=/home/mi/apriltag_deploy
DEST=/opt/ros2/cyberdog
sudo_cmd() { echo "\$PASS" | sudo -S "\$@"; }

sudo_cmd cp -r "\$STAGE/lib/apriltag_ros"              "\$DEST/lib/"
sudo_cmd cp    "\$STAGE/lib/libAprilTagNode.so"        "\$DEST/lib/"
sudo_cmd cp -P "\$STAGE/lib/libapriltag.so"*           "\$DEST/lib/"
sudo_cmd cp    "\$STAGE/lib/libapriltag_msgs__rosidl"*.so "\$DEST/lib/"
sudo_cmd cp    "\$STAGE/lib/libapriltag_msgs__python.so"   "\$DEST/lib/"
sudo_cmd mkdir -p "\$DEST/lib/python3.6/site-packages"
sudo_cmd cp -r "\$STAGE/lib/python3.6/site-packages/apriltag_msgs" \
            "\$DEST/lib/python3.6/site-packages/"

sudo_cmd cp -r "\$STAGE/share/apriltag_ros"           "\$DEST/share/"
sudo_cmd cp -r "\$STAGE/share/apriltag_msgs"          "\$DEST/share/"

for sub in packages rosidl_interfaces package_run_dependencies parent_prefix_path rclcpp_components; do
    if [ -d "\$STAGE/share/ament_index/resource_index/\$sub" ]; then
        sudo_cmd mkdir -p "\$DEST/share/ament_index/resource_index/\$sub"
        sudo_cmd cp "\$STAGE/share/ament_index/resource_index/\$sub"/* \
                    "\$DEST/share/ament_index/resource_index/\$sub/"
    fi
done

rm -rf "\$STAGE"
echo "apriltag 部署完成"
EOF

echo "==> 完成"
