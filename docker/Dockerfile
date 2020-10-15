FROM osrf/ros:noetic-desktop-full

# Set up Gazebo source
RUN apt-get update && apt-get install -y wget
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Install dependent packages for TurtleBot3 control
RUN apt-get update && apt-get install -y ros-noetic-joy \
									ros-noetic-teleop-twist-joy \
									ros-noetic-teleop-twist-keyboard \
									ros-noetic-laser-proc \
									ros-noetic-rgbd-launch \
									#ros-noetic-depthimage-to-laserscan \
									ros-noetic-rosserial-arduino \
									ros-noetic-rosserial-python \
									#ros-noetic-rosserial-server \
									ros-noetic-rosserial-client \
									ros-noetic-rosserial-msgs \
									ros-noetic-amcl \
									ros-noetic-map-server \
									ros-noetic-move-base \
									ros-noetic-urdf \
									ros-noetic-xacro \
									ros-noetic-compressed-image-transport \
									ros-noetic-rqt-image-view \
									ros-noetic-gmapping \
									ros-noetic-navigation \
									ros-noetic-interactive-markers \
									ros-noetic-gazebo-ros \libignition-cmake2-dev \
                     				libignition-common3-dev \
                     				libignition-common3-graphics-dev \
                     				libignition-math6-dev \
                     				libignition-math6-eigen3-dev \
                     				libignition-plugin-dev \
									libdart6-collision-ode \
                     				#libdart6-dev \
                     				#libdart6-collision-ode-dev \
                     				#libdart6-utils-urdf-dev \
                     				#libsdformat8-dev \
									libgazebo11 \
									gazebo11-common \
									gazebo11-plugin-base \
									gazebo11 \
									gfortran

# Install TurboVNC
RUN apt-get install -y expect x11-xkb-utils xauth xfonts-base xkb-data
RUN tmp="$(mktemp)" && \
	wget -O "$tmp" https://downloads.sourceforge.net/project/virtualgl/2.6.2/virtualgl_2.6.2_amd64.deb && \
	dpkg -i "$tmp" && \
	rm -f $tmp
RUN tmp="$(mktemp)" && \
	wget -O "$tmp" https://downloads.sourceforge.net/project/turbovnc/2.2.2/turbovnc_2.2.2_amd64.deb && \
	dpkg -i "$tmp" && \
	rm -f $tmp
ENV PATH ${PATH}:/opt/VirtualGL/bin:/opt/TurboVNC/bin

# Create user
ARG uid
ARG user
ARG gid
RUN groupadd -g $gid -o $user
RUN useradd -m -u $uid -g $gid $user
RUN usermod -aG video $user
RUN chmod 755 /root
RUN apt-get install -y sudo
RUN echo "\n$user ALL=(ALL) NOPASSWD:ALL\n" >> /etc/sudoers

# Copy the current directory contents into the container at /
COPY ros_env.sh /
COPY ros_entrypoint.sh /
RUN sed -i "s/\${HOST_USER}/$user/g" /ros_entrypoint.sh
RUN echo "\nsource /ros_env.sh\nsource /home/$user/catkin_ws/.bashrc" >> /home/$user/.bashrc
RUN ln -s /home/$user/catkin_ws /root/catkin_ws

# Install pip libraries
RUN apt-get install -y python3-pip
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install numpy==1.16 scipy matplotlib==2.2 jupyter scikits.bvp_solver dubins==0.9.2
# RUN dpkg --remove --force-depends python-enum34  # Installed by ros, newer version required by tensorflow
# RUN python -m pip install tensorflow==1.15

# Install additional apt packages
RUN apt-get -f install -y
RUN apt-get install -y net-tools iputils-ping vim nano less tmux libpcap-dev

# Clear apt cache
RUN apt-get autoremove -y
RUN rm -rf /var/lib/apt/lists/*
