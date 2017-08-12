#include "tools.h"
#include <limits>

Tools::Tools() {}

Tools::~Tools() {}

void Tools::global2Vehicle(
  const double xRef,
  const double yRef,
  const double cosYaw,
  const double sinYaw,
  const double xGlob,
  const double yGlob,
  double&      xVeh,
  double&      yVeh)
{
  const double x_trans = xGlob - xRef;
  const double y_trans = yGlob - yRef;

  xVeh =  x_trans * cosYaw + y_trans * sinYaw;
  yVeh = -x_trans * sinYaw + y_trans * cosYaw;
}

void Tools::vehicle2Global(
  const double xRef,
  const double yRef,
  const double cosYaw,
  const double sinYaw,
  const double xVeh,
  const double yVeh,
  double&      xGlob,
  double&      yGlob)
{
  xGlob = xRef + xVeh * cosYaw - yVeh * sinYaw;
  yGlob = yRef + xVeh * sinYaw + yVeh * cosYaw;
}

int Tools::closestWaypoint(
  const double          x, 
  const double          y, 
  const vector<double>& maps_x, 
  const vector<double>& maps_y)
{
  double closestLen = numeric_limits<double>::max(); //large number
  int closestWp = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    const double map_x = maps_x[i];
    const double map_y = maps_y[i];
    const double distSquared = distanceSquared(x,y,map_x,map_y);
    if(distSquared < closestLen)
    {
      closestLen = distSquared;
      closestWp  = i;
    }
  }
  return closestWp;
}

int Tools::nextWaypoint(
  const double          x, 
  const double          y, 
  const double          theta, 
  const vector<double>& maps_x, 
  const vector<double>& maps_y)
{
  int closestWp = closestWaypoint(x,y,maps_x,maps_y);

  const double map_x = maps_x[closestWp];
  const double map_y = maps_y[closestWp];

  const double heading = atan2( (map_y-y),(map_x-x) );

  const double angle = fabs(theta-heading);

  if(angle > pi()/4)
  {
    closestWp++;
  }
  return closestWp;
}

std::vector<double> Tools::getXY(
  const double          s, 
  const double          d, 
  const vector<double>& maps_s, 
  const vector<double>& maps_x, 
  const vector<double>& maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  const int wp2 = (prev_wp+1)%maps_x.size();

  const double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  const double seg_s = (s-maps_s[prev_wp]);

  const double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  const double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  const double perp_heading = heading-pi()/2;

  const double x = seg_x + d*cos(perp_heading);
  const double y = seg_y + d*sin(perp_heading);

  return {x,y};
}
