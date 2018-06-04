# NTRIP password
if [ -z $1 ]; then
	echo "Please enter the password as first argument in order to have RTK corrections from the NTRIP server."
	exit
fi

# ROS_IP
if [ ! -z $2 ]; then
	export ROS_IP=$2
else
	export ROS_IP=$(hostname -I)
	# Check if there is one or several IP returned by hostname
	test `echo $ROS_IP | wc -m` -lt 17
	if [ ! $? -eq 0 ]; then
		echo "IP address cannot be determined, please enter it as second argument."
		export ROS_IP=
		exit
	fi
fi
echo "Your IP address is : "$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:3128
. ~/ros_ws/devel/setup.bash
roscore

# Rosbag
if [ ! -d ~/ros_ws/rosbags ]; then
    mkdir ~/ros_ws/rosbags
fi
rosbag record -a -o ~/ros_ws/rosbags

# Roslaunch
roslaunch zodiac_launchers zodiac.launch

# RTK Corrections
# The password must be entered as first argument of this script
sh ~/.str2str -in ntrip://ENSTABRE:$1 @78.24.131.136:2101/MAC30 -out serial://gps -n 1000 -p 48.418 -4.472 150.0

