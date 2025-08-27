## Usage
1. Download the [SDK](https://cloud.tsinghua.edu.cn/f/3dcd648bd16e4a54a6a9/) and unzip somewhere
2. Modify config in `tac3d_ros2/config/tac3d_leap_config`
    - change `server_url` to your SDK install location
    - change the config `tac3d/<finger_name>` according to your setup
3. Clone and build `tac3d_ros2` in a ROS2 workspace
4. Launch
   ```
   ros2 launch tac3d_ros2 launch_tac3d_client.launch.py
   ```