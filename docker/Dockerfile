FROM osrf/ros:kinetic-desktop-full

# Install dependent packages for TurtleBot3 control
RUN apt-get update && apt-get install -y ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

# Install TurboVNC
RUN apt-get install -y wget expect x11-xkb-utils xauth xfonts-base xkb-data
RUN tmp="$(mktemp)" && \
	wget -O "$tmp" https://svwh.dl.sourceforge.net/project/virtualgl/2.6.2/virtualgl_2.6.2_amd64.deb && \
	dpkg -i "$tmp" && \
	rm -f $tmp
RUN tmp="$(mktemp)" && \
	wget -O "$tmp" https://svwh.dl.sourceforge.net/project/turbovnc/2.2.2/turbovnc_2.2.2_amd64.deb && \
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
RUN apt-get install -y python-pip
RUN pip install numpy==1.16 scipy==1.2 matplotlib tornado==5.1.1 ipython==5.8 ipykernel==4.10 notebook==5.7 jupyter==1.0 opencv-python scikits.bvp_solver

# Install additional apt packages
RUN apt-get install -y net-tools

# Clear apt cache
RUN rm -rf /var/lib/apt/lists/*
