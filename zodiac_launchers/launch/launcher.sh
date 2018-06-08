# This file must be `source` before any launch.
# The PIDs variable contains every PID of the process launched by this file.

export now=`date +"%F-%T"`

# NTRIP password
if [ -z $1 ]; then
	echo "Please enter the password as first argument in order to have RTK corrections from the NTRIP server."
	return
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
		return
	fi
fi
echo "Your IP address is : "$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:11311
. ~/ros_ws/devel/setup.bash


# Rosbag
if [ ! -d ~/ros_ws/recorded_data ]; then
    mkdir ~/ros_ws/recorded_data
fi
mkdir ~/ros_ws/recorded_data/$now
# nohup roscore > ~/ros_ws/recorded_data/$now/roscore.out &
# PIDs=$!
# sleep 1
# nohup rosbag record -a -o ~/ros_ws/recorded_data/$now/rosbag > ~/ros_ws/recorded_data/$now/rosbag.out &
# PIDs+=" $!"
cp ~/ros_ws/src/ZodiacAuto/zodiac_launchers/config/*.yaml ~/ros_ws/recorded_data/$now/
cp -r `readlink ~/.ros/log/latest` ~/ros_ws/recorded_data/$now/

# RTK Corrections
# The password must be entered as first argument of this script
nohup ~/str2str -in "ntrip://ENSTABRE:$1@78.24.131.136:2101/MAC30" -out serial://gps -n 1000 -p 48.418 -4.472 150.0 > ~/ros_ws/recorded_data/$now/str2str.out &
PIDs=$!

# PIDS+=" `pidof record`"

# Roslaunch
roslaunch zodiac_launchers zodiac.launch &
PIDs+=" $!"

