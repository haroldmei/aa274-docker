#!/bin/bash

directory=$(pwd)
network=aa274_net

# Base Docker command
cmd=( \
	docker run -it --rm --init \
	--mount type=bind,src=$directory/catkin_ws,dst=/home/$USER/catkin_ws \
	--mount type=bind,src=$directory/.ros,dst=/home/$USER/.ros \
	--net $network \
	--user $USER)

# Parse arguments
while [[ $# -ge 2 && $1 == -* ]]; do
	if [[ $1 == "--display" ]]; then
		# Display argument is not passed to Docker
		display=$2
		shift
		shift
		continue
	elif [[ $1 == "--vncport" ]]; then
		# vncport argument is not passed to Docker
		vncport=$2
		shift
		shift
		continue
	elif [[ $1 == "--rosmaster" ]]; then
		# rosmaster argument is not passed to Docker
		rosmaster=$2
		shift
		shift
		continue
	elif [[ $1 == "--rosport" ]]; then
		# rosport argument is not passed to Docker
		rosport=$2
		shift
		shift
		continue
	fi

	# Add arguments to Docker command
	cmd+=($1 $2)
	shift
	shift
done

# Display options
if [[ -n $display ]]; then

	# Set vncport to default (display + 5900) if display > 0
	if [[ -z $vncport ]]; then
		vncport=$(($display + 5900))
	fi

	# Set VNC settings
	if [[ -n $vncport ]]; then
		cmd+=(-p $vncport:$vncport)

		# Prompt VNC password
		printf "\nRunning a VNC instance at $(hostname --ip-address):${vncport} with the password created below.\n\n"
		while [[ ${#password} -lt 6 ]]; do
			read -p "VNC password (6-8 characters): " -s password
		done
		printf "\n\n"

		cmd+=( \
			--env "VNCPASSWD=$password" \
			--env "VNCPORT=$vncport")
	fi
	cmd+=(--env "DISPLAY=:$display")
elif [[ -n $DISPLAY ]]; then
	cmd+=(--env "DISPLAY=$DISPLAY")
fi

# Set rosport to default (11311)
if [[ -z $rosport ]]; then
	rosport=11311
fi

# Set rosmaster to default (master)
if [[ -z $rosmaster ]]; then
	rosmaster="master"
fi

cmd+=(--env "ROS_MASTER_URI=http://$rosmaster:$rosport")

if [[ -n $display || -n $DISPLAY ]]; then
	# Set environment variables
	cmd+=(--env "QT_GRAPHICSSYSTEM=native")

	# Give Docker container access to host X11 for hardware acceleration,
	# unless vncport is defined and display == 0
	if [[ $display -gt 0 || -z $vncport ]]; then
		# Run Nvidia driver
		cmd+=(--runtime nvidia)

		XSOCK=/tmp/.X11-unix/X0
		XAUTH=/tmp/.docker.xauth
		touch $XAUTH
		xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
		cmd+=(--volume=$XSOCK:$XSOCK:rw \
		  	  --volume=$XAUTH:$XAUTH:rw)
	fi

	# Prepend vglrun to command
	set -- vglrun "$@"
fi


# Application specific options
if [[ $* == *roscore* ]]; then
	# Name the roscore container 'master' so that other nodes can reach it
	cmd+=(--name $rosmaster)
elif [[ $* == *rvizweb* ]]; then
	# Port forwarding for rvizweb
	cmd+=(-p 8001:8001 -p 9090:9090)
fi

# Run command
echo ${cmd[@]}
${cmd[@]} aa274 $@
