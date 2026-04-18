本机可视化cyberdog2实机话题工具

当前支持激光雷达数据和里程计话题的显示，转换后的话题为config/topic.yaml中的output_topic的内容

构建

```bash
cd /cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-up-to topic_visualization
```

运行

```bash
ros2 launch topic_visualization visualiza.launch.py
```