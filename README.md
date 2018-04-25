# ZodiacAuto
Some ROS packets usefull to the automatization of the zodiac.
For now, there is just one packet which contains 2 nodes :
* _localisation_, which listens to fix messages from a GPS node and publishes a position,
* _pololu_, which listends to cmd messages and modify the position of a servomotor connected to a Pololu Maestro.

The GPS node is [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver), and you need to install it from source in order to parse correctly the NMEA trames sent by a RTK GPS.
You can send the RTK correction with _str2str_ from [RTKLIB](http://www.rtklib.com/) with a command like this one :

        ./str2str -in ntrip://<user>:<password>@<address>:<port>/<mountpoint> -out serial://ttyACM0 -n 1000 -p <lat> <lon> <hgt>

