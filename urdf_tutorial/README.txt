visualize robot
https://velog.io/@gaebalsebal/ROS2-URDF-%EC%9E%91%EC%84%B1%ED%95%98%EC%97%AC-%EA%B0%80%EC%83%81%ED%99%98%EA%B2%BD%EC%97%90-%EB%A1%9C%EB%B4%87-%EB%82%98%ED%83%80%EB%82%B4%EA%B8%B0

sudo apt install ros-humble-xacro
ros2 pkg create --build-type ament_python urdf_tutorial
cd urdf_tutorial

rm -rf install/urdf_tutorial/ build/urdf_tutorial/
colcon build --packages-select urdf_tutorial
source install/local_setup.bash
ros2 launch urdf_tutorial robot.launch.py

urdf Tutorials Building a Visual Robot Model with URDF from Scratch
https://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch

urdf_tutorial
git clone https://github.com/ros/urdf_tutorial.git