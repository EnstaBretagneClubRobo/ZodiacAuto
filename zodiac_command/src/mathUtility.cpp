#include <zodiac_command/mathUtility.h>

double mathUtility::degreeToRadian(double degrees)
{
	return degrees * M_PI / 180;
}


double mathUtility::radianToDegree(double radians)
{
	return radians / M_PI * 180;
}

double mathUtility::calculateDTW(double gpsLon, double gpsLat, double waypointLon, double waypointLat)
{
	const double radiusOfEarth = 6371.0;

	double deltaLatitudeRadians = mathUtility::degreeToRadian(waypointLat - gpsLat);
	double boatLatitudeInRadian = mathUtility::degreeToRadian(gpsLat);
	double waypointLatitudeInRadian = mathUtility::degreeToRadian(waypointLat);
	double deltaLongitudeRadians = mathUtility::degreeToRadian(waypointLon - gpsLon);

	double a = sin(deltaLatitudeRadians/2)
			* sin(deltaLatitudeRadians/2)
			+ cos(boatLatitudeInRadian)
			* cos(waypointLatitudeInRadian)
			* sin(deltaLongitudeRadians/2)
			* sin(deltaLongitudeRadians/2); 			

	double b = 2 * atan2(sqrt(a), sqrt(1 - a));
	double distanceToWaypoint = radiusOfEarth * b * 1000;
	
	return distanceToWaypoint;
}