#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "tools.h"
#include <string>

// Parameters
static const string map_file_              = "../data/highway_map.csv";  // map file location
static const double TARGET_X               = 30.0;                       // local x coordinate of local wp spline
static const double DT                     = 0.02;                       // time elapsed between two cycles
static const double FPS                    = 1.0/DT;                     // frames per second
static const double MAX_DIST_S             = 30.0;                       // maximum front distance between cars in frenet s coordinates
static const double MAX_DIST_S_CLOSE       = 20.0;                       // maximum front distance between cars in frenet s coordinates for matching speed
static const double SPEED_DECCEL           = 5.0;                        // Reduce speed by 5.0 m/s if closer than MAX_DIST_S_CLOSE to car in front
static const double DIST_INCREASE          = Tools::mps2mph(5.0*DT);     // maximum increase or decrease in frenet s coordinate per frame
static const double TARGET_SPEED           = 49.5;                       // Target velocity in mph

#endif // CONSTANTS_H