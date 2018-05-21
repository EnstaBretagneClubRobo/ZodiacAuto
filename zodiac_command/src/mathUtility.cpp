#include <zodiac_command/mathUtility.h>

double mathUtility::degreeToRadian(double degrees)
{
	return degrees * M_PI / 180;
}

double mathUtility::radianToDegree(double radians)
{
	return radians / M_PI * 180;
}

double mathUtility::limitRadianAngleRange(double angle)
{
	const double fullRevolution = 2 * M_PI;
	const double minAngle = 0;

	while (angle < minAngle)
		angle += fullRevolution;

	while (angle >= minAngle + fullRevolution)
		angle -= fullRevolution;

	return angle;

	// NOTE - Maël: An other possibility to set the angle in ]0, 360[ is to use a sawtooth function.
	// return 2*atan(tan((angle - M_PI)/2)) + M_PI;
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

double mathUtility::calculateSignedDistanceToLine(const double nextLon, const double nextLat, const double prevLon, const double prevLat,
					const double gpsLon, const double gpsLat)
{
    int earthRadius = 6371000;

    std::array<double, 3> prevWPCoord = //a
     {  earthRadius * cos(degreeToRadian(prevLat)) * cos(degreeToRadian(prevLon)),
        earthRadius * cos(degreeToRadian(prevLat)) * sin(degreeToRadian(prevLon)),
        earthRadius * sin(degreeToRadian(prevLat))};
    std::array<double, 3> nextWPCoord = //b
     {  earthRadius * cos(degreeToRadian(nextLat)) * cos(degreeToRadian(nextLon)),
        earthRadius * cos(degreeToRadian(nextLat)) * sin(degreeToRadian(nextLon)),
        earthRadius * sin(degreeToRadian(nextLat))};
        std::array<double, 3> boatCoord = //m
     {  earthRadius * cos(degreeToRadian(gpsLat)) * cos(degreeToRadian(gpsLon)),
        earthRadius * cos(degreeToRadian(gpsLat)) * sin(degreeToRadian(gpsLon)),
        earthRadius * sin(degreeToRadian(gpsLat))};

    std::array<double, 3> oab = //vector normal to plane
    {   (prevWPCoord[1]*nextWPCoord[2] - prevWPCoord[2]*nextWPCoord[1]),       //Vector product: A^B divided by norm ||a^b||     a^b / ||a^b||
        (prevWPCoord[2]*nextWPCoord[0] - prevWPCoord[0]*nextWPCoord[2]),
        (prevWPCoord[0]*nextWPCoord[1] - prevWPCoord[1]*nextWPCoord[0])};

    double normOAB =  sqrt(pow(oab[0],2)+ pow(oab[1],2) + pow(oab[2],2));

    oab[0] = oab[0]/normOAB;
    oab[1] = oab[1]/normOAB;
    oab[2] = oab[2]/normOAB;

    double signedDistance = boatCoord[0]*oab[0] + boatCoord[1]*oab[1] + boatCoord[2]*oab[2];

    return signedDistance;
}

double mathUtility::calculateWaypointsOrthogonalLine(const double nextLon, const double nextLat, const double prevLon, const double prevLat,
					const double gpsLon, const double gpsLat)
{    /* Check to see if boat has passed the orthogonal to the line
     * otherwise the boat will continue to follow old line if it passed the waypoint without entering the radius
     */
    int earthRadius = 6371000;

    std::array<double, 3> prevWPCoord = //a
     {  earthRadius * cos(degreeToRadian(prevLat)) * cos(degreeToRadian(prevLon)),
        earthRadius * cos(degreeToRadian(prevLat)) * sin(degreeToRadian(prevLon)),
        earthRadius * sin(degreeToRadian(prevLat))};
    std::array<double, 3> nextWPCoord = //b
     {  earthRadius * cos(degreeToRadian(nextLat)) * cos(degreeToRadian(nextLon)),
        earthRadius * cos(degreeToRadian(nextLat)) * sin(degreeToRadian(nextLon)),
        earthRadius * sin(degreeToRadian(nextLat))};
        std::array<double, 3> boatCoord = //m
     {  earthRadius * cos(degreeToRadian(gpsLat)) * cos(degreeToRadian(gpsLon)),
        earthRadius * cos(degreeToRadian(gpsLat)) * sin(degreeToRadian(gpsLon)),
        earthRadius * sin(degreeToRadian(gpsLat))};

    std::array<double, 3> oab = //vector normal to plane
    {   (prevWPCoord[1]*nextWPCoord[2] - prevWPCoord[2]*nextWPCoord[1]),       //Vector product: A^B divided by norm ||a^b||     a^b / ||a^b||
        (prevWPCoord[2]*nextWPCoord[0] - prevWPCoord[0]*nextWPCoord[2]),
        (prevWPCoord[0]*nextWPCoord[1] - prevWPCoord[1]*nextWPCoord[0])};

    double normOAB =  sqrt(pow(oab[0],2)+ pow(oab[1],2) + pow(oab[2],2));

    oab[0] = oab[0]/normOAB;
    oab[1] = oab[1]/normOAB;
    oab[2] = oab[2]/normOAB;

    //compute if boat is after waypointModel
    std::array<double, 3> orthogonal_to_AB_from_B = //C the point such as  BC is orthogonal to AB
    {  nextWPCoord[0]+oab[0],
       nextWPCoord[1]+oab[1],
       nextWPCoord[2]+oab[2]
    };

    std::array<double, 3> obc = //vector normal to plane
    {   (orthogonal_to_AB_from_B[1]*nextWPCoord[2] - orthogonal_to_AB_from_B[2]*nextWPCoord[1]) ,       //Vector product: C^B divided by norm ||c^b||     c^b / ||c^b||
        (orthogonal_to_AB_from_B[2]*nextWPCoord[0] - orthogonal_to_AB_from_B[0]*nextWPCoord[2]) ,
        (orthogonal_to_AB_from_B[0]*nextWPCoord[1] - orthogonal_to_AB_from_B[1]*nextWPCoord[0])};

    double normOBC =  sqrt(pow(obc[0],2)+ pow(obc[1],2) + pow(obc[2],2));

	double orthogonalLine;
    //float temp = boatCoord[0]*obc[0] + boatCoord[1]*obc[1] + boatCoord[2]*obc[2];
    orthogonalLine = boatCoord[0]*obc[0]/normOBC + boatCoord[1]*obc[1]/normOBC + boatCoord[2]*obc[2]/normOBC;

    return orthogonalLine;
}