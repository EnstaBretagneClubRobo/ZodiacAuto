This document explains how to set up the embedded computer (__Zotac__) and the laptop (__Toughbook__). README.txt should be read once you want to launch the automatization of the system.

* Copy `launcher.sh` in the home directory of the __Zotac__ (_/home/zodiac_)
* In "Startup Applications" (gnome-session-properties) on the __Zotac__, add : 
** bash -c "sleep 5 ; cd ~ && . ~/launcher.sh rtkpassword"
* Copy `README.txt` as well as the `boatbot_XXX.sh` on the Desktop of the __Toughbook__ (_/home/user/Desktop_)
* In Startup Applications (gnome-session-properties) on the __Toughbook__, add : 
** bash -c "sleep 10 ; sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB0,mode=666 tcp:10.42.0.1:5001 & sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB3,mode=666 tcp:10.42.0.1:5011 & (sleep 5 ; opencpn)"
