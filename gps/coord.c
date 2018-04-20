#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define deg_to_rad(deg) (deg * M_PI / 180.0)
#define rad_to_deg(rad) (rad * 180.0 / M_PI)

struct coord
{
	double lat;
	double lon;
};

double coord_distance(struct coord origin, struct coord destination)
{
	/* convert coordinates to radians and change east longitude positive by formula sign convention */
	origin.lat=deg_to_rad(origin.lat);
	origin.lon=-deg_to_rad(origin.lon);
	destination.lat=deg_to_rad(destination.lat);
	destination.lon=-deg_to_rad(destination.lon);
	
	/* formula from http://www.edwilliams.org/avform.htm#Dist */
	double distance = 2*asin(sqrt(pow(sin((origin.lat-destination.lat)/2.0),2.0)+cos(origin.lat)*cos(destination.lat)*pow(sin((origin.lon-destination.lon)/2.0),2.0)));
	
	/* convert radial distance to meters (1 deg = 60 nmi, 1 nmi = 1852 m) */
	distance = rad_to_deg(distance) * 60.0 * 1852.0;
	
	/* return result */
	return distance;
}


struct coord coord_dist_radial(struct coord origin, double distance, double radial)
{
	/* convert parameters to radians and change west longitude positive by formula sign convention */
	origin.lat = deg_to_rad(origin.lat);
	origin.lon = -deg_to_rad(origin.lon);
		
	/* invert radial to use positive clockwise angle and convert to radians */ 
	radial = deg_to_rad(-radial);
	
	/* convert distance to angular distance (1 deg = 60 nmi, 1 m = 0.000539957 nmi) */
	distance = deg_to_rad(distance * 0.000539957 / 60.0);
	
	/* create struct for result coordinate */
	struct coord result;

	/* formulas from http://www.edwilliams.org/avform.htm#LL */
	result.lat = asin(sin(origin.lat)*cos(distance)+cos(origin.lat)*sin(distance)*cos(radial));
	result.lon = fmod(origin.lon-atan2(sin(radial)*sin(distance)*cos(origin.lat),cos(distance)-sin(origin.lat)*sin(result.lat))+M_PI,2.0*M_PI)-M_PI;
  
    /* convert result coordinate to degrees and switch to west longitude negative */
	result.lat = rad_to_deg(result.lat);
	result.lon = -rad_to_deg(result.lon);
  
    /* return result pointer */
	return result;
}


double coord_course(struct coord origin, struct coord destination)
{
	/* convert coordinates to radians and east longitude positive by formula sign convention */
	origin.lat=deg_to_rad(origin.lat);
	origin.lon=-deg_to_rad(origin.lon);
	destination.lat=deg_to_rad(destination.lat);
	destination.lon=-deg_to_rad(destination.lon);
	
	/* formula from http://www.edwilliams.org/avform.htm#Crs */
	double course = fmod(atan2(sin(origin.lon-destination.lon)*cos(destination.lat),cos(origin.lat)*sin(destination.lat)-sin(origin.lat)*cos(destination.lat)*cos(origin.lon-destination.lon)),2*M_PI);
	
	/* convert course to degrees and add offset for standard course range*/
	course = -rad_to_deg(course);
	
	// flip negative values around
	if (course < 0.0)
		course = course + 360.0;
	
	/* return result */
	return course;
}


void coord_print(struct coord coord)
{
	printf("%f, %f\n", coord.lat, coord.lon);
}


int main(void)
{
	/* create struct for origin coordinate */
	struct coord origin;
	
	/* set origin coordinate */
	origin.lat = 42.039603;
	origin.lon = -93.641755;
	
	/* set distance and radial from the origin coordinate */
	double distance = 100.0;
	double radial = 90.0;
	
	/* calculate target coordinate */
	struct coord result = coord_dist_radial(origin, distance, radial);
	
	/* print coordinates */
	coord_print(origin);
	coord_print(result);
	
	/* calculate distance and course */
	printf("distance: %f\n", coord_distance(origin, result));
	printf("course: %f\n", coord_course(origin, result));
	
	return 0;
}
