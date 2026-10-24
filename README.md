NIM anggota :
080
068

How to run (UTS.py)
Terminal 1

cd ~/ros2_ws
ros2 launch robin_bringup my_robot_gazebo.launch.py

Terminal 2

cd ~/ros2_ws
ros2 run ros_gz_bridge parameter_bridge '/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' --ros-args -r /world/empty/dynamic_pose/info:=/world_pose

Terminal 3

cd ~/ros2_ws
python3 maze_runner.py
