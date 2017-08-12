#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "spline.h"

using namespace std;

class PathPlanner
{
public:

  // Current vehicle state
  struct VehicleState
  {
    double x; // longitudinal world coordinate
    double y; // lateral world coordinate

    double refX; // longitudinal reference point
    double refY; // lateral reference point

    double s; // longitudinal frenet coordinate
    double d; // lateral frenet coordinate

    double yawDeg; // orientation in degrees
    double yawRad; // orientation in rad

    double speedMph; // velocity in miles per hour

    double endS; // end frenet s coordinate from last sent path
    double endD; // end frenet d coordinate from last sent path
  };

  // Map containing all waypoints
  struct WpMap
  {
    vector<double> x;  // waypoint longitudinal world coordinates
    vector<double> y;  // waypoint lateral world coordinates
    vector<double> s;  // waypoint longitudinal frenet coordinates
    vector<double> dx; // waypoint lane offset vector x component
    vector<double> dy; // waypoint lane offset vector y component
  };

  // Constructor
  PathPlanner(const WpMap& map);

  // Destructor
  virtual ~PathPlanner();

  // Calc next path
  void calcPath(
    const VehicleState&           state,
    const vector<double>&         previous_path_x,
    const vector<double>&         previous_path_y,
    const vector<vector<double>>& sensorFusion,
    vector<double>&               next_x_vals,
    vector<double>&               next_y_vals);

private:

  // Check if lane change is feasible
  bool checkLaneChange(
    const int newLane ) const;

  // Update state information
  void updateState();

  // Determine new reference velocity
  void calcTargetSpeed();

  // Try to change lane if vehicle in front is too slow
  void tryChangeLanes();

  // Setup next waypoints for ego path
  void setupLocalNextPath(vector<double>& ptsx, vector<double>& ptsy) const;

  // Transform points from global to vehicle coordinates
  void transformGlobal2Vehicle(
    const vector<double>& ptsxGlobal, 
    const vector<double>& ptsyGlobal,
    vector<double>&       ptsxVehicle, 
    vector<double>&       ptsyVehicle ) const;

  // Append next points to path 
  void appendNewPoints(
    const tk::spline&  s,
    vector<double>&    next_x_vals, 
    vector<double>&    next_y_vals);

  vector<vector<double>> m_sensor_fusion; // Other vehicles on the track
  VehicleState           m_state;         // Current ego vehicle information

  WpMap                  m_wpMap;         // Map information
  int                    m_numWps;        // Number of waypoints

  vector<double>         m_previous_path_x; // Previous path in x direction
  vector<double>         m_previous_path_y; // Previous path in y direction

  int                    m_lane;            // Lane index of ego vehicle
  int                    m_lane_change_wp;  // Waypoint index for lane change

  double                 m_targetSpeed;     // Target speed in mph
  int                    m_prevSize;        // Number of unused waypoint in previous path

  int                    m_nextWp;          // Next waypoint index along our path

  double                 m_cosYawRad;       // Cosine of current yaw angle in rad
  double                 m_sinYawRad;       // Sine of current yaw angle in rad

  bool                   m_changeLanes;     // Flag indicating if we plan to change lane
};

#endif