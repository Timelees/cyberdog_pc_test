**修改自启动配置**
进入cyberdog的NX板，修改以下文件中的DepsLifecycleNodes为以下内容

/opt/ros2/cyberdog/share/algorithm_manager/config/Task.toml

```
[[task]]
TaskName = "VisionLocalization"
Id = 7
OutDoor = true
DepsNav2LifecycleNodes = []
DepsLifecycleNodes = ["camera/camera", "stereo_camera", "mivinslocalization"]
```

**部署编译的包**
```
scp -r install/lib/vins mi@192.168.44.1:/home/mi/
sudo cp -rf /home/mi/vins /opt/ros2/cyberdog/lib/
sudo rm -rf /home/mi/vins

scp -r install/lib/libvins_lib.so mi@192.168.44.1:/home/mi/
sudo cp -rf /home/mi/libvins_lib.so /opt/ros2/cyberdog/lib/
sudo rm -rf /home/mi/libvins_lib.so

scp -r install/share/vins mi@192.168.44.1:/home/mi/
sudo cp -rf /home/mi/vins /opt/ros2/cyberdog/share/
sudo rm -rf /home/mi/vins

sudo reboot
```


前置要求
realsense相机驱动必须启动，这个有开机自启动节点，设置生命周期时需要加namespace

```
ros2 launch realsense2_camera on_dog.py # 开机自启动

ros2 lifecycle set /camera/camera_align configure

ros2 lifecycle set /camera/camera_align activate
```

启动
```
ros2 launch vins dog_d430i_stereo_odometry_localization.py use_miloc:=false # 开机自启动

ros2 lifecycle set /vinslocalization configure

ros2 lifecycle set /vinslocalization activate
```

