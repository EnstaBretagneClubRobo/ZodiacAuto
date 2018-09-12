Welcome to BoatBot
==================


Normal operation
-----

First plug everything, ensure the switch on the electronic box is off (manual mode) and start the zodiac battery. After around 2 min restart the Toughbook and wait for OpenCPN to launch automatically. The boat should appear in red in OpenCPN (click the "Auto Follow" icon in the toolbar to show it if needed, as well as "Enable Tracking" icon to show its trajectory). Create a route using the "Create Route" icon, right-click on the route (not on the waypoint) and select "Send to GPS (Serial:/dev/ttyVUSB0)" and finally move the switch on the electronic box to start an autonomous mission (you still need to control the throttle manually, but you should see the wheel moving automatically). You can move the switch to off at any time to get manual control, and then move it back to on to resume the current route in autonomous mode.


End of the mission
-----

Please double-click on boatbot_stop_and_download.sh and wait for the logs to be downloaded through Wi-Fi. This may take several minutes. Ensure the Toughbook is as close as possible to the electronic box.
Check that the recorded_data folder received all the expected data. If you are sure, double-click on boatbot_erase.sh to erase the originals (the memory of the embedded computer is limited), or be sure to bring back the electronic box so that someone can check.
Finally, double-click on boatbot_poweroff.sh to shut down the embedded computer. You can now safely stop the zodiac battery and unplug everything.


Troubleshooting
-----

If e.g. the boat appears in black in OpenCPN and the different values do not seem to update or the behavior of the boat is unexpected, power off the Toughbook (if possible, try before to stop cleanly the embedded computer by double-clicking on boatbot_stop.sh and then boatbot_poweroff.sh), unplug the power cable from the electronic box, wait 10 s, check everything and start all over again.


Advanced
-----

* Open a terminal (Ctrl+Alt+T) and connect to the embedded computer using `ssh zodiac@10.42.0.1`
* The main configuration file can be modified using the command `nano /home/zodiac/ros_ws/src/ZodiacAuto/zodiac_launchers/launch/zodiac.launch`
* If you want to stop the sending of RTK corrections, you can enter `pkill str2str`
