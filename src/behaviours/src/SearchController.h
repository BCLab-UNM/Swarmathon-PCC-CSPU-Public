#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

#include <math.h>

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : public virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();

  void patternDeterminator(Point pre_point);
  Point calGoalPos(float a, float b, float theta, Point pre_point);

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
//  Point subGoalPos;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;

  //sammi: pattern walk
//  void setNextGoalPos();

  //sammi: pattern walk
  Point startPoint;
  float diffX = 0, diffY = 0;
  const float initialSpinSize = 0.05; //in meters aka 10cm
  const float spinSizeIncrement = 0.50; //in meters
  float spinSizeIncrease = 0;
  float depthFromHome = 2.5;//meter
  float spinner = 0;
  Point nextSpinPoint;

};

#endif /* SEARCH_CONTROLLER */
