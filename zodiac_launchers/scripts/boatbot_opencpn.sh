#!/bin/bash
sudo killall socat
bash -c "sleep 2 ; sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB0,mode=666 tcp:10.42.0.1:5001 & (sleep 5 ; opencpn)"
exit
