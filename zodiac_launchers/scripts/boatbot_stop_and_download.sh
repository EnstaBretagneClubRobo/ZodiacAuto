#!/bin/sh
echo "Please wait..."
ssh zodiac@10.42.0.1 "killall -INT roslaunch; sleep 20; killall roslaunch bash; sudo killall socat str2str TCPSplit UxVSim UxVCtrl python; sleep 5; killall roslaunch bash ; sudo killall socat str2str TCPSplit UxVSim UxVCtrl python"
sleep 2
mkdir -p ~/Desktop/recorded_data/rosloglatest && scp -r zodiac@10.42.0.1:~/.ros/log/latest/* ~/Desktop/recorded_data/rosloglatest && scp -r zodiac@10.42.0.1:~/ros_ws/recorded_data/* ~/Desktop/recorded_data
exit
