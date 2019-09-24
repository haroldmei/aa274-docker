# Installation

1. Install Docker.
```
./install_docker.sh
```
2. Restart computer.
3. Create the Catkin workspace (`aa274-docker/catkin_ws`) and Docker network
   (`aa274_net`). Other ROS packages can be put into `aa274/catkin_ws` as well.
```
./init_aa274.sh
```
4. Build the Docker image. This should be run any time `docker/Dockerfile` is changed.
```
./build_docker.sh
```
5. Build the Catkin workspace. This should be run any time a new ROS package is added.
```
./rosdep_install.sh
```
6. Whenever you make changes to your own ROS package, compile it with the
   following command:
```
./run.sh catkin_make
```

# Running ROS

Roscore needs to be running before any other nodes. In one terminal window, run:
```
./run.sh roscore
```

Nodes can now be run in separate terminal windows.
```
./run.sh <ros_command>
```

To choose the ROS master URI, call `run.sh` with `--rosmaster <hostname>` and/or
`--rosport <port>`:
```
./run.sh --rosmaster master --rosport 11311
```

If you are using Docker remotely and need to view the GUI, call the command with
`--display <display_id>`. Using display ID >= 1 will use the host computer's
hardware acceleration to render the graphics, while ID == 0 will use software
rendering.
```
./run.sh --display 1 --vncport 5901 roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
The optional `vncport` parameter can be manually specified to avoid port
collisions with other nodes. Otherwise, the port number will default to
`5900 + <display_id>`.

You can connect to this VNC instance by connecting to the host's IP address with
the VNC port number, e.g., `192.168.1.10:5901`.
