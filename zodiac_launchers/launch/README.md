Welcome to BoatBot
==================

Setup
-----

In order to launch the automatization of the zodiac, just follow the following steps (once the embedded computer is booted):

* Open a terminal (Ctrl+Alt+T)
* Connect to the embedded computer via ssh : `$ ssh zodiac@10.42.0.1`
* Enter the embedded computer password : `caidoz`
* Launch the script _launcher.sh_ : `$ . launcher.sh 03071508` (the second argument is the password in order to have RTK corrections, if you don't enter it, there will be no corrections, but it will work fine as well)
* Open an other terminal
* launch the _boatbot.sh_ script : `$ . boatbot.sh`and enter the toughbook password : `Workstation`


Interface & mission planner
---------------------------

Now, you can launch OpenCPN.
To start the mission right clic on the waypoint path (not on the waypoint).

