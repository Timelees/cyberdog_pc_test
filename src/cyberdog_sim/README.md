```bash
cd /home/lee/code/cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash

colcon build --packages-select ros_bridge sim_base cyber_guide --symlink-install

source install/setup.bash
```