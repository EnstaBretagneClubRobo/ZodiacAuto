Welcome to BoatBot
==================

Setup
-----

In order to launch the automatization of the zodiac, just follow the following steps (once the embedded computer is booted, which is automatic once the power wire is plugged):

* Open a terminal (Ctrl+Alt+T)
* Connect to the embedded computer via ssh : `$ ssh zodiac@10.42.0.1`
* Enter the embedded computer password : `caidoz`
* Launch the script _launcher.sh_ : `$ . launcher.sh 03071508` (the second argument is the password in order to have RTK corrections, if you don't enter it, there will be no corrections, but it will work fine as well)
* Open an other terminal
* launch the _boatbot.sh_ script : `$ . boatbot.sh` and enter the toughbook password : `Workstation`


Interface & mission planner
---------------------------

Now, you can launch OpenCPN.
To start the mission right clic on the waypoint path (not on the waypoint) and click on "Send to GPS (/dev/...)".


Stopping or relaunching the scripts
-----------------------------------

You can stop the scripts in each terminals by doing Ctrl+C.

If you want to stop the sending of RTK corrections, you can enter the following command line (in the terminal 
connected to the embedded computer, with the ssh):

		pkill str2str

You can relaunch the scripts with the same commands in each terminal.

Please shutdown the embedded computer before unplugging it with this command line (in the terminal 
connected to the embedded computer, with the ssh):

		sudo poweroff

The password may be asked, in this case enter `caidoz` and press Enter.
