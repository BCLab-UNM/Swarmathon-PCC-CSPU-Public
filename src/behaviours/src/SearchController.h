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

protected:

  void ProcessData();
  void setNextGoalPos();
  void setIntialPos();
  Point calculateTempPoint();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;

  //sammi: pattern walk
  Point nextGoalPos;
  Point tempPoint;//a point in the map with no angel drifting
  const float INCREASE_x_y = 0.75/sqrt(2);//the increasing x and y, since the pattern walk is 1/4 division of the arena with origin at home
  const float START_LEN = 1.3;//meter
  float THETA_TRAN;
  float r_patterWalk;
  
  //localization 
  float testingDistance = 0;
  Point testing_last_pos;
};

#endif /* SEARCH_CONTROLLER */
