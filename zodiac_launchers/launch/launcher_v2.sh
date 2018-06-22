# This file must be `source` before any launch.
# The PIDs variable contains every PID of the process launched by this file.

export now=`date +"%F-%T"`
pwd_path=$PWD

# NTRIP password
use_rtk=true
if [ -z $1 ]; then
	echo "Please enter the password as first argument in order to have RTK corrections from the NTRIP server."
	use_rtk=false
else
	echo "RTK corrections will be sent to /dev/gps"
fi

# ROS_IP
export ROS_IP=10.42.0.1
echo "Your IP address is : "$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:11311

# cd ${0%*launcher.sh}/../../../..
cd ~/ros_ws
. ./devel/setup.${SHELL##*/}

# Rosbag
if [ ! -d ./recorded_data ] ; then
    mkdir recorded_data
fi
mkdir recorded_data/$now
save_path=$PWD/recorded_data/$now

roscd zodiac_launchers
cp ./config/*.yaml $save_path
cp -r `readlink ~/.ros/log/latest` $save_path

# RTK Corrections
# The password must be entered as first argument of this script
if [ "$use_rtk" = true ] ; then
    nohup ~/str2str -in "ntrip://ENSTABRE:$1@78.24.131.136:2101/MAC30" -out serial://gps -n 1000 -p 48.418 -4.472 150.0 > $save_path/str2str.out &
    PIDs=$!
fi

sleep 1

# Roslaunch
roslaunch zodiac_launchers zodiac.launch &
PIDs+=" $!"

# Return to initial config
cd $pwd_path
unset pwd_path
unset use_rtk
unset save_path
unset IP_nb

fg
