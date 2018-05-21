#ifndef mathUtility_h
#define mathUtility_h

#include <stdint.h>
#include <vector>
#include <cmath>
#include <array>


class mathUtility {
    public:
    	static double degreeToRadian(double degrees);
	    static double radianToDegree(double radians);
        static double limitRadianAngleRange(double angle);
        
        static double calculateDTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat);
	    static double calculateSignedDistanceToLine(const double nextLon, const double nextLat, const double prevLon, const double prevLat,
					const double gpsLon, const double gpsLat);
	    static double calculateWaypointsOrthogonalLine(const double nextLon, const double nextLat, const double prevLon, const double prevLat,
					const double gpsLon, const double gpsLat);
};

#endif