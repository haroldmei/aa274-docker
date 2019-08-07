# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install -y ros-kinetic-ros-base
sudo rosdep init
rosdep update

# Prepare ROS workspace
mkdir -p ~/catkin_ws/src

# Download ROS packages
cd ~/catkin_ws/src
sudo apt-get install -y git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/StanfordASL/asl_turtlebot.git

# Initialize ROS workspace
cd ~/catkin_ws
source /opt/ros/kinetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

source ~/.bashrc

catkin_make && catkin_make install

sudo apt-get install -y wget expect
tmp="$(mktemp)" && \
	wget -O "$tmp" https://svwh.dl.sourceforge.net/project/virtualgl/2.6.2/virtualgl_2.6.2_amd64.deb && \
	sudo dpkg -i "$tmp" && \
	rm -f $tmp
tmp="$(mktemp)" && \
	wget -O "$tmp" https://svwh.dl.sourceforge.net/project/turbovnc/2.2.2/turbovnc_2.2.2_amd64.deb && \
	sudo dpkg -i "$tmp" && \
	rm -f $tmp

echo "export PATH=${PATH}:/opt/VirtualGL/bin:/opt/TurboVNC/bin" >> ~/.bashrc
