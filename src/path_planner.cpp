#include "path_planner.h"
#include "tools.h"
#include "constants.h"
#include <limits>

PathPlanner::PathPlanner(const WpMap& map)
  : m_wpMap(map),
    m_lane(1),
    m_lane_change_wp(0),
    m_numWps(map.x.size())
{
}

PathPlanner::~PathPlanner() {}

void PathPlanner::calcPath(
  const VehicleState&           state,
  const vector<double>&         previous_path_x,
  const vector<double>&         previous_path_y,
  const vector<vector<double>>& sensorFusion,
  vector<double>&               next_x_vals,
  vector<double>&               next_y_vals)
{
  // Clear output first
  next_x_vals.clear();
  next_y_vals.clear();

  // Copy new information
  m_state           = state;
  m_previous_path_x = previous_path_x;
  m_previous_path_y = previous_path_y;
  m_sensor_fusion   = sensorFusion;
  m_targetSpeed     = TARGET_SPEED;
  m_changeLanes     = false;
  m_prevSize        = previous_path_x.size();

  // Update state data
  updateState();

  // Determine the new target speed
  calcTargetSpeed();

  //try to change lanes if too close to car in front
  if(m_changeLanes && ((m_nextWp-m_lane_change_wp)%m_numWps > 2U))
  {
    tryChangeLanes();
  }

  // Setup curve through next a set of points in global coordinates
  vector<double> ptsxGlobal, ptsyGlobal;
  setupNextPath(ptsxGlobal, ptsyGlobal);

  // Transform points to vehicle coordinates
  vector<double> ptsxLocal, ptsyLocal;
  transformGlobal2Vehicle(ptsxGlobal, ptsyGlobal, ptsxLocal, ptsyLocal);

  // Re-use old path
  for(int i = 0; i < m_prevSize; ++i)
  {
    next_x_vals.push_back(m_previous_path_x[i]);
    next_y_vals.push_back(m_previous_path_y[i]);
  }

  // Setup spline through waypoints
  tk::spline s;
  s.set_points(ptsxLocal,ptsyLocal);

  // Append new waypoints along path
  appendNewPoints(s, next_x_vals, next_y_vals);
}

bool PathPlanner::checkLaneChange(
  const int newLane ) const
{
  bool lane_safe = true;
  for(unsigned int i = 0; i < m_sensor_fusion.size(); ++i)
  {
    //car is in left lane
    const vector<double>& vehicle = m_sensor_fusion[i];
    const double d                = vehicle[6];
    const double leftLaneD        = static_cast<double>(2+4*newLane-2);
    const double rightLaneD       = static_cast<double>(2+4*newLane+2);
    if( (d < rightLaneD) && (d > leftLaneD) )
    {
      const double check_vx    = vehicle[3];
      const double check_vy    = vehicle[4];
      const double check_speed = Tools::distance(check_vx,check_vy,0.0,0.0);

      const double check_s_pred  = vehicle[5] + static_cast<double>(m_prevSize)*DT*check_speed;
      const double dist_s = check_s_pred-m_state.s;
      if( (dist_s < MAX_DIST_S_CLOSE) && (dist_s > -MAX_DIST_S_CLOSE) )
      {
        lane_safe = false;
      }
    }
  }
  return lane_safe;
}

void PathPlanner::updateState()
{
  m_state.refX   = m_state.x;
  m_state.refY   = m_state.y;
  m_state.yawRad = Tools::deg2rad(m_state.yawDeg);

  if(m_prevSize < 2)
  {
    m_nextWp = Tools::nextWaypoint(m_state.refX, m_state.refY, m_state.yawRad, m_wpMap.x, m_wpMap.y);
  }
  else
  {
    m_state.refX = m_previous_path_x[m_prevSize-1];
    const double ref_x_prev = m_previous_path_x[m_prevSize-2];
    m_state.refY = m_previous_path_y[m_prevSize-1];
    const double ref_y_prev = m_previous_path_y[m_prevSize-2];
    m_state.yawRad = atan2(m_state.refY-ref_y_prev,m_state.refX-ref_x_prev);
    m_nextWp = Tools::nextWaypoint(m_state.refX,m_state.refY,m_state.yawRad,m_wpMap.x, m_wpMap.y);

    m_state.s = m_state.endS;

    m_state.speedMph = Tools::mps2mph(FPS*Tools::distance(m_state.refX, m_state.refY, ref_x_prev, ref_y_prev));
  }

  m_cosYawRad = cos(m_state.yawRad);
  m_sinYawRad = sin(m_state.yawRad);
}

void PathPlanner::calcTargetSpeed()
{
  //find reference velocity from next car in front
  double closestDist_s = numeric_limits<double>::max();
  m_changeLanes = false;
  for(int i = 0; i < m_sensor_fusion.size(); ++i)
  {
    //car is in my lane
    const vector<double>& vehicle = m_sensor_fusion[i];
    const double d                = vehicle[6];
    const double leftLaneD        = static_cast<double>(2+4*m_lane-2);
    const double rightLaneD       = static_cast<double>(2+4*m_lane+2);
    if( (d < rightLaneD) && (d > leftLaneD) )
    {
      const double check_vx    = vehicle[3];
      const double check_vy    = vehicle[4];
      const double check_speed = Tools::distance(check_vx, check_vy, 0.0, 0.0);
      const double check_s     = vehicle[5] + static_cast<double>(m_prevSize)*DT*check_speed;

      //check s values greater than mine and s gap
      const double dist_s      = check_s-m_state.s;
      if((dist_s > 0.0) && (dist_s < MAX_DIST_S) && (dist_s < closestDist_s) )
      {
        closestDist_s = dist_s;
        m_changeLanes = true;

        if(dist_s > MAX_DIST_S_CLOSE)
        {
          //match that cars speed
          m_targetSpeed = Tools::mps2mph(check_speed);
        }
        else
        {
          //go slightly slower than the cars speed
          m_targetSpeed = Tools::mps2mph(check_speed)-SPEED_DECCEL;
        }
      }
    }
  }
}

void PathPlanner::tryChangeLanes()
{
  bool changed_lanes = false;
  bool lane_safe     = false;

  //first try to change to left lane
  if (m_lane != 0)
    lane_safe = checkLaneChange(m_lane-1);

  if(lane_safe)
  {
    changed_lanes = true;
    m_lane -= 1;
    m_lane_change_wp = m_nextWp;
  }

  //next try to change to right lane
  if(m_lane != 2 && !changed_lanes)
  {
    if( checkLaneChange(m_lane+1) )
    {
      changed_lanes = true;
      m_lane += 1;
      m_lane_change_wp = m_nextWp;
    }
  }
}

void PathPlanner::setupNextPath(vector<double>& ptsx, vector<double>& ptsy) const
{
  // Setup next wps
  ptsx.clear();
  ptsy.clear();

  // First obtain last two points
  if(m_prevSize < 2)
  {
    const double prev_car_x = m_state.x - cos(m_state.yawDeg);
    const double prev_car_y = m_state.y - sin(m_state.yawDeg);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(m_state.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(m_state.y);
  }
  else
  {
    ptsx.push_back(m_previous_path_x[m_prevSize-2]);
    ptsx.push_back(m_previous_path_x[m_prevSize-1]);

    ptsy.push_back(m_previous_path_y[m_prevSize-2]);
    ptsy.push_back(m_previous_path_y[m_prevSize-1]);
  }

  // Obtain next three points
  const vector<double> next_wp0 = Tools::getXY(m_state.s+30,(2+4*m_lane),m_wpMap.s,m_wpMap.x,m_wpMap.y);
  const vector<double> next_wp1 = Tools::getXY(m_state.s+60,(2+4*m_lane),m_wpMap.s,m_wpMap.x,m_wpMap.y);
  const vector<double> next_wp2 = Tools::getXY(m_state.s+90,(2+4*m_lane),m_wpMap.s,m_wpMap.x,m_wpMap.y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
}

void PathPlanner::transformGlobal2Vehicle(
  const vector<double>& ptsxGlobal, 
  const vector<double>& ptsyGlobal,
  vector<double>&       ptsxVehicle, 
  vector<double>&       ptsyVehicle ) const
{
  ptsxVehicle.clear();
  ptsyVehicle.clear();
  for (int i = 0; i < ptsxGlobal.size(); ++i)
  {
    double xVeh,yVeh;
    Tools::global2Vehicle(
      m_state.refX,  m_state.refY,
      m_cosYawRad,   m_sinYawRad,
      ptsxGlobal[i], ptsyGlobal[i],
      xVeh,          yVeh          );

    ptsxVehicle.push_back(xVeh);
    ptsyVehicle.push_back(yVeh);
  }
}

void PathPlanner::appendNewPoints(
  const tk::spline&  s,
  vector<double>&    next_x_vals, 
  vector<double>&    next_y_vals)
{
  const double target_y    = s(TARGET_X);
  const double target_dist = Tools::distance(TARGET_X,target_y,0.0,0.0);

  double x_add_on = 0;

  for (int i = 1; i <= 50-m_prevSize; ++i) {

    if(m_targetSpeed > m_state.speedMph)
    {
      m_state.speedMph += SPEED_INCREASE;
    }
    else if(m_targetSpeed < m_state.speedMph)
    {
      m_state.speedMph -= SPEED_INCREASE;
    }


    const double N = (target_dist/Tools::mph2mps(DT*m_state.speedMph));
    double x_point = x_add_on+TARGET_X/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    const double x_ref = x_point;
    const double y_ref = y_point;

    Tools::vehicle2Global(
      m_state.refX, m_state.refY,
      m_cosYawRad,  m_sinYawRad,
      x_ref,        y_ref,
      x_point,      y_point  );

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

