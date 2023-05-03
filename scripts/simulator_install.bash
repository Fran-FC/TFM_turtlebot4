sudo apt-get update && sudo apt-get install wget
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress -y

sudo apt install ros-humble-ros2-control -y


mkdir -p tb4_ws/src
cd tb4_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator -b humble
cd ..
rosdep install --from-path src -yi --rosdistro humble
source /opt/ros/humble/setup.bash

sudo apt update; sudo apt upgrade -y

colcon build --symlink-install
source install/setup.bash
