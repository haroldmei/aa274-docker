FROM osrf/ros:kinetic-desktop-full

# Install dependent packages for TurtleBot3 control
RUN apt-get update && apt-get install -y ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

# Install Nvidia OpenGL
RUN apt-get install -y autoconf
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
RUN git clone --branch="v1.0.0" https://github.com/NVIDIA/libglvnd.git /opt/libglvnd && \
	cd /opt/libglvnd && \
	./autogen.sh && \
	./configure --prefix=/usr/local --libdir=/usr/local/lib/x86_64-linux-gnu && \
	make -j"$(nproc)" install-strip && \
	find /usr/local/lib/x86_64-linux-gnu -type f -name 'lib*.la' -delete

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
RUN useradd -m -u $uid -g $gid $user
RUN usermod -aG video $user
RUN chmod 755 /root
RUN apt-get install -y sudo
RUN echo "\n$user ALL=(ALL) NOPASSWD:ALL\n" >> /etc/sudoers

# Copy the current directory contents into the container at /
COPY ros_env.sh /
COPY ros_entrypoint.sh /
RUN sed -i "s/\${HOST_USER}/$user/g" /ros_entrypoint.sh
RUN echo "\nsource /ros_env.sh\n" >> /home/$user/.bashrc
RUN ln -s /home/$user/catkin_ws /root/catkin_ws
