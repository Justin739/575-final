#include <stdio.h>
#include <stdlib.h>
#include <math.h>

struct geocoord
{
	double lat;
	double lon;
};

struct geocoord* coord_dist_from_bearing(struct geocoord* origin, double distance, double bearing)
{
	/* convert parameters to radians */
	double lat = origin->lat * (M_PI / 180.0);
	double lon = origin->lon * (M_PI / 180.0);
	
	/* invert bearing to use positive clockwise angle and convert to radians */ 
	bearing = (-M_PI / 180.0) * bearing;
	
	/* convert distance to angular distance (1 deg = 60 nmi, 1 m = 0.000539957 nmi) */
	distance = (M_PI / 180.0) * (distance * 0.000539957 / 60.0);
	
	/* allocate memory for result coordinate */
	struct geocoord* result = (struct geocoord*)malloc(sizeof(struct geocoord));

	/* formulas from http://www.edwilliams.org/avform.htm#LL */
	result->lat = asin(sin(lat)*cos(distance)+cos(lat)*sin(distance)*cos(bearing));
	result->lon = fmod(lon-atan2(sin(bearing)*sin(distance)*cos(lat),cos(distance)-sin(lat)*sin(result->lat))+M_PI,2*M_PI)-M_PI;
  
    /* convert result coordinate to degrees */
	result->lat = (180.0 / M_PI) * result->lat;
	result->lon = (180.0 / M_PI) * result->lon;
  
    /* return result pointer */
	return result;
}

void print_geocoord(struct geocoord* coord)
{
	printf("%f, %f\n", coord->lat, coord->lon);
}

int main(void)
{
	/* allocate memory for origin coordinate */
	struct geocoord* origin = (struct geocoord*)malloc(sizeof(struct geocoord));
	
	/* set origin coordinate */
	origin->lat = 42.039603;
	origin->lon = -93.641755;
	
	/* set distance and bearing from the origin coordinate */
	double distance = 100.0;
	double bearing = 270.0;
	
	/* calculate target coordinate */
	struct geocoord* result = coord_dist_from_bearing(origin, distance, bearing);
	
	/* print coordinates */
	print_geocoord(origin);
	print_geocoord(result);
	
	/* free memory */
	free(origin);
	free(result);
	return 0;
}
