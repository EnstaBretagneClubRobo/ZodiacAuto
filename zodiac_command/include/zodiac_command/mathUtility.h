#ifndef mathUtility_h
#define mathUtility_h

#include <stdint.h>
#include <vector>
#include <cmath>



class mathUtility {
    public:
    	static double degreeToRadian(double degrees);
	    static double radianToDegree(double radians);
        static double calculateDTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat);
};

#endif