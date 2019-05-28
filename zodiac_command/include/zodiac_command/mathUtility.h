#ifndef mathUtility_h
#define mathUtility_h

#include <stdint.h>
#include <vector>
#include <cmath>
#include <array>


class mathUtility {
    public:
		static int sgn(double value);

    	static double degreeToRadian(double degrees);
	    static double radianToDegree(double radians);
		static double limitAngleRange(double angle);
		static double limitAngleRange180(double angle);
        static double limitRadianAngleRange(double angle);
		static double linearFunctionBetweenAngle(double x, double x1, double x2, double angle1, double angle2);

        static double calculateDTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat);
	    static double calculateSignedDistanceToLine(const double nextLon, const double nextLat, const double prevLon, const double prevLat,
					const double gpsLon, const double gpsLat);
	    static double calculateWaypointsOrthogonalLine(const double nextLon, const double nextLat, const double prevLon, const double prevLat,
					const double gpsLon, const double gpsLat);
};

#endif