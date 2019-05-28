#!/bin/sh
echo "Please wait..."
ssh zodiac@10.42.0.1 "killall -INT roslaunch; sleep 20; killall roslaunch bash; sudo killall socat str2str TCPSplit UxVSim UxVCtrl python; sleep 5; killall roslaunch bash ; sudo killall socat str2str TCPSplit UxVSim UxVCtrl python"
exit
