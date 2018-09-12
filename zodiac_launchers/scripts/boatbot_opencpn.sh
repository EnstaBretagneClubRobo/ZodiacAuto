#!/bin/bash
sudo killall socat
bash -c "sleep 2 ; sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB0,mode=666 tcp:10.42.0.1:5001 & sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB3,mode=666 tcp:10.42.0.1:5011 & (sleep 5 ; opencpn)"
exit

