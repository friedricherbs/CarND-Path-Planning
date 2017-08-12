#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include <math.h>

using namespace std;

class Tools {

public:

	/**
	* Constructor.
	*/
	Tools();

	/**
	* Destructor.
	*/
	virtual ~Tools();

    // For converting back and forth between radians and degrees.
    static inline constexpr double pi() { return M_PI; }
    static inline double deg2rad(double x) { return x * pi() / 180; }
    static inline double rad2deg(double x) { return x * 180 / pi(); }

    // Velocity conversion functions
    static inline double mph2mps(const double x) { return x * 1.60934/3.6; }
    static inline double mps2mph(const double x) { return x * 3.6/1.60934; }

    // Calculate squared euclidean distance between two points -> no square root!
    static inline double distanceSquared(const double x1, const double y1, const double x2, const double y2)
    {
      return ((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }

    // Calculate euclidean distance between two points
    static inline double distance(const double x1, const double y1, const double x2, const double y2)
    {
      return sqrt(distanceSquared(x1,y1,x2,y2));
    }

    // Transform from global world coordinates to local vehicle coordinates
    static void global2Vehicle(
      const double xRef,
      const double yRef,
      const double cosYaw,
      const double sinYaw,
      const double xGlob,
      const double yGlob,
      double&      xVeh,
      double&      yVeh);

    // Transform from local vehicle to global world coordinates
    static void vehicle2Global(
      const double xRef,
      const double yRef,
      const double cosYaw,
      const double sinYaw,
      const double xVeh,
      const double yVeh,
      double&      xGlob,
      double&      yGlob);

    // Return closest waypoint index towards the ego vehicle
    static int closestWaypoint(
      const double          x, 
      const double          y, 
      const vector<double>& maps_x, 
      const vector<double>& maps_y);

    // Return next waypoint index
    static int nextWaypoint(
      const double          x, 
      const double          y, 
      const double          theta, 
      const vector<double>& maps_x, 
      const vector<double>& maps_y );

    // Transform from Frenet s,d coordinates to Cartesian x,y
    static vector<double> getXY(
      const double          s, 
      const double          d, 
      const vector<double>& maps_s, 
      const vector<double>& maps_x, 
      const vector<double>& maps_y);
};

#endif // TOOLS_H