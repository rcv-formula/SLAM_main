source /home/rcv/SLAM/install/setup.bash

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/rcv/SLAM/0320.pbstream'}"
ros2 run nav2_map_server map_saver_cli -f /home/rcv/SLAM/0320

