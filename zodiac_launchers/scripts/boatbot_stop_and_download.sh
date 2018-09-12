#!/bin/sh
ssh zodiac@10.42.0.1 "killall -INT roslaunch"
echo "Please wait..."
sleep 20
ssh zodiac@10.42.0.1 "killall roslaunch ; sudo killall socat"
sleep 5
mkdir -p ~/Desktop/recorded_data && scp -r zodiac@10.42.0.1:~/ros_ws/recorded_data/* ~/Desktop/recorded_data
exit

