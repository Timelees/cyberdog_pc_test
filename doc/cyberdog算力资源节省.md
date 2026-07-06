# 算力资源节省

关闭不必要的节点，避免需要使用的功能节点被杀死

## base

### **源码修改**

修改cyberdog_bringup/automatic/launch.yaml中base项为以下内容

```yaml
base:
    file_name: base.launch.py
    nodes: [connector, cyberdog_audio, device_manager, sensor_manager, motion_manager, realsense2_camera, cyberdog_manager, odom_out_publisher, motor_bridge,  bes_transmit]
```

然后编译修改cyberdog_bringup部署到实机





## navigation

修改 cyberdog_nav2/navigation_bringup/launch/navigation.launch.py的以下内容

```yaml
node_lists = [
        'state_publisher',
        # 'vision_manager',
        # 'camera_server',
        # 'tracking',
        'realsense',
        'realsense_align',
        # 'mcr_uwb',
        # 'mcr_voice',
        'velocity_adaptor',
        # 'nav2_base',
        # 'map_label_server',
        # 'report_dog_pose',
        # 'laser_mapping',
        #'laser_localization',
        'mivins_localization',
        # 'mivins_mapping',
        #'mivins_vo',
        # 'miloc',
        # 'stereo_camera',
        # 'occmap',
        # 'algorithm_manager',
        # 'elevation_mapping_odom',
        'head_tof_pc_publisher',
        # 'stair_align',
        # 'charging_localization',
        # 'seat_adjust_server',
        # 'tracking_indication',
        'rosbag_recorder',
        # 'tracking_base',
        'emergency_stop'
        ]
```

## 回充坐姿调整关闭

```bash
# 停止当前进程
sudo systemctl stop cyberdog_autodock.service
# 禁止开机自启
sudo systemctl disable cyberdog_autodock.service
```