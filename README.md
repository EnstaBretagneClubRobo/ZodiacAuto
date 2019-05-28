# ZodiacAuto
This document explains how to set up the embedded computer (__Zotac__) and the laptop (__Toughbook__) :
* Copy `launcher.sh` (or create a link : `ln -s ~/ros_ws/src/ZodiacAuto/zodiac_launchers/scripts/launcher.sh ~/launcher.sh`) in the home directory of the __Zotac__ (_/home/zodiac_)
* Copy `mux_server.py` (or create a link : `ln -s ~/ros_ws/src/ZodiacAuto/mux_serial/mux_server.py ~/mux_server.py`) in the home directory of the __Zotac__ (_/home/zodiac_)
* Build and copy `str2str` (or create a link : `ln -s ~/ros_ws/src/ZodiacAuto/RTKLIB/app/str2str/gcc/str2str ~/str2str`) in the home directory of the __Zotac__ (_/home/zodiac_)
* Check `ROS_IP`, `ROS_WS`, `RTK_RTCM_SERVER_IP` in `launcher.sh, run `catkin_make clean && catkin_make` in ROS workspace (`~/ros_ws`)
* Check the devices `.yaml` and `.launch` files, create udev rules to match the required serial port names, add respawn options in launch files if needed, check if you need to tweak `serial.Serial(..., rtscts=True, dsrdtr=True)` in some nodes due to a problem with virtual serial ports in Python 2.7, etc.
* In "Startup Applications" (gnome-session-properties) on the __Zotac__, add : 
** bash -c "sleep 5; cd ~ && . ~/launcher.sh rtkpassword"
* In Startup Applications (gnome-session-properties) on the __Toughbook__, add : 
** bash -c "sleep 10; sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB0,mode=666 tcp:10.42.0.1:5001 & (sleep 5 ; opencpn)"
* From `zodiac_launchers/scripts`, copy `README.txt` as well as the `boatbot_XXX.sh` on the Desktop of the __Toughbook__ (_/home/user/Desktop_)
zodiac_launchers/scripts/README.txt should be read once you want to launch the automatization of the system.
