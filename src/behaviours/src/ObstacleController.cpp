#include "ObstacleController.h"
#include <ros/ros.h>

ObstacleController::ObstacleController()
{
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  result.PIDMode = CONST_PID; //use the const PID to turn at a constant speed
}


//note, not a full reset as this could cause a bad state
//resets the interupt and knowledge of an obstacle or obstacle avoidance only.
void ObstacleController::Reset() {
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  delay = current_time;
}

// Avoid crashing into objects detected by the ultraound
void ObstacleController::avoidObstacle() {
    //always turn left to avoid obstacles
    if (right < 0.9 || center < 0.9 || left < 0.9) {
        if(!initialized){
            //ROS_INFO_STREAM("IN Avoidance: AVOIDING is initialized");
            sonar_startTime = getROSTimeInMilliSecs_S();//for reducing sonar fake noise
        }
        if(obstacle_counter >= obstacle_constrain){
            //ROS_INFO_STREAM("IN Avoidance: time:" <<  getROSTimeInMilliSecs_S() << " - " << sonar_startTime);
            if(getROSTimeInMilliSecs_S() - sonar_startTime <= SONAR_DURATION){
                //ROS_INFO_STREAM("IN Avoidance: Logic says do avoiding!!!!!!!!!!!!!");
                /*result.type = precisionDriving;

                result.pd.cmdAngular = -K_angular;

                result.pd.setPointVel = 0.0;
                result.pd.cmdVel = 0.0;
                result.pd.setPointYaw = 0;
*/
		    if(center > left && center > right){
			//ROS_INFO_STREAM("IN AVOIDING: FORWARD");
			result.pd.cmdAngularError = 0.0;
			result.pd.cmdVel = 0.3;
		    }else if(left > right){
			//ROS_INFO_STREAM("IN AVOIDING: TURN LEFT");
			result.pd.cmdAngular = K_angular-0.2;
			result.pd.cmdVel = 0.0;
		    }else{
			//ROS_INFO_STREAM("IN AVOIDING: TURN RIGHT");
			result.pd.cmdAngular = -K_angular+0.2;
			result.pd.cmdVel = 0.0;
		    }
		      result.type = precisionDriving;

		      result.pd.setPointVel = 0.0;
		      result.pd.setPointYaw = 0;
            }
           // ROS_INFO_STREAM("IN Avoidance: AVOIDING RESETTING");
            sonar_startTime = getROSTimeInMilliSecs_S();
            obstacle_counter = 0;
            initialized = false;
        }else if(getROSTimeInMilliSecs_S() - sonar_startTime > SONAR_DURATION){
            obstacle_counter = 0;
            initialized = false;
        }else{
            obstacle_counter++;
            //ROS_INFO_STREAM("IN Avoidance: AVOIDING: " << obstacle_counter);
            initialized = true;
        }

    }
}

int backup_duration = 3;
int spin_duration = 5;
int total_avoid_time = backup_duration + spin_duration;

// A collection zone was seen in front of the rover and we are not carrying a target
// so avoid running over the collection zone and possibly pushing cubes out.
void ObstacleController::avoidCollectionZone() {
  
    

    long int Tdifference = current_time - timeSinceTags;
    float Td = Tdifference/1e3;

    result.type = precisionDriving;
     
    if(Td < backup_duration)// first 5 seconds, back up
    {
      result.pd.cmdVel = -0.2;
      result.pd.cmdAngular = 0;
    }
    else // next 5 seconds, spin around
    {
      result.pd.cmdVel = 0.0;
      result.pd.cmdAngular = K_angular;
    } 

}


Result ObstacleController::DoWork() {

  clearWaypoints = true;
  set_waypoint = true;
  result.PIDMode = CONST_PID;

  // The obstacle is an april tag marking the collection zone
  if(collection_zone_seen){
    avoidCollectionZone();
  }
  else {
    avoidObstacle();
  }

  //if an obstacle has been avoided
  if (can_set_waypoint) {

    can_set_waypoint = false; //only one waypoint is set
    set_waypoint = false;
    clearWaypoints = false;

    result.type = waypoint; 
    result.PIDMode = FAST_PID; //use fast pid for waypoints
    Point forward;            //waypoint is directly ahead of current heading
    forward.x = currentLocation.x + (0.5 * cos(currentLocation.theta));
    forward.y = currentLocation.y + (0.5 * sin(currentLocation.theta));
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(forward);
    //ROS_INFO_STREAM("IN AVOIDING: getting waypoint");
  }

  return result;
}


void ObstacleController::setSonarData(float sonarleft, float sonarcenter, float sonarright) {
  left = sonarleft;
  right = sonarright;
  center = sonarcenter;
  ProcessData();
}

void ObstacleController::setCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void ObstacleController::ProcessData() {

  //timeout timer for no tag messages
  //this is used to set collection zone seen to false beacuse
  //there is no report of 0 tags seen
  long int Tdifference = current_time - timeSinceTags;
  float Td = Tdifference/1e3;
  if ((Td >= total_avoid_time && collection_zone_seen == true) || (Td >= 0.5 && collection_zone_seen == false)) {
    collection_zone_seen = false;
    phys= false;
    if (!obstacleAvoided)
    {
      can_set_waypoint = true;
    }
  }

  //If we are ignoring the center sonar
  if(ignore_center_sonar){
    //If the center distance is longer than the reactivation threshold 
    if(center > reactivate_center_sonar_threshold){
      //currently do not re-enable the center sonar instead ignore it till the block is dropped off
      //ignore_center_sonar = false; //look at sonar again beacuse center ultrasound has gone long
    }
    else{
      //set the center distance to "max" to simulated no obstacle
      center = 3;
    }
  }
  else {
    //this code is to protect against a held block causing a false short distance
    //currently pointless due to above code
    if (center < 3.0) {
      result.wristAngle = 0.7;
    }
    else {
      result.wristAngle = -1;
    }
  }

  //if any sonar is below the trigger distance set physical obstacle true
  if (left < triggerDistance || right < triggerDistance || center < triggerDistance)
  {
    phys = true;
    timeSinceTags = current_time;
  }

  //if physical obstacle or collection zone visible
  if (collection_zone_seen || phys)
  {
    obstacleDetected = true;
    obstacleAvoided = false;
    can_set_waypoint = false;
  }
  else
  {
    obstacleAvoided = true;
  }
}


// Report April tags seen by the rovers camera so it can avoid
// the collection zone
// Added relative pose information so we know whether the
// top of the AprilTag is pointing towards the rover or away.
// If the top of the tags are away from the rover then treat them as obstacles. 
void ObstacleController::setTagData(vector<Tag> tags){
  //collection_zone_seen = false;
  count_left_collection_zone_tags = 0;
  count_right_collection_zone_tags = 0;


  // this loop is to get the number of center tags
  if (!targetHeld) {
    for (int i = 0; i < tags.size(); i++) { //redundant for loop
      if (tags[i].getID() == 256) {
	//collection_zone_seen = checkForCollectionZoneTags( tags );
        collection_zone_seen = true;
        timeSinceTags = current_time;
        
      }
    }
  }

}

bool ObstacleController::checkForCollectionZoneTags( vector<Tag> tags ) {

  for ( auto & tag : tags ) { 

    // Check the orientation of the tag. If we are outside the collection zone the yaw will be positive so treat the collection zone as an obstacle. 
    //If the yaw is negative the robot is inside the collection zone and the boundary should not be treated as an obstacle. 
    //This allows the robot to leave the collection zone after dropping off a target.
    if ( tag.calcYaw() > 0 ) 
      {
	// checks if tag is on the right or left side of the image
	if (tag.getPositionX() + camera_offset_correction > 0) {
	  count_right_collection_zone_tags++;
	  
	} else {
	  count_left_collection_zone_tags++;
	}
      }
    
  }


  // Did any tags indicate that the robot is inside the collection zone?
  return count_left_collection_zone_tags + count_right_collection_zone_tags > 0;

}

//obstacle controller should inrerupt is based upon the transition from not seeing and obstacle to seeing an obstacle
bool ObstacleController::ShouldInterrupt() {

  //if we see and obstacle and havent thrown an interrupt yet
  if(obstacleDetected && !obstacleInterrupt)
  {
    obstacleInterrupt = true;
    return true;
  }
  else
  {
    //if the obstacle has been avoided and we had previously detected one interrupt to change to waypoints
    if(obstacleAvoided && obstacleDetected)
    {
      Reset();
      return true;
    } else {
      return false;
    }
  }
}

bool ObstacleController::HasWork() {
  //there is work if a waypoint needs to be set or the obstacle hasnt been avoided
  if (can_set_waypoint && set_waypoint)
  {
    return true;
  }

  return !obstacleAvoided;
}

//ignore center ultrasound
void ObstacleController::setIgnoreCenterSonar(){
  ignore_center_sonar = true; 
}

void ObstacleController::setCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

void ObstacleController::setTargetHeld() {
  targetHeld = true;

  //adjust current state on transition from no cube held to cube held
  if (previousTargetState == false) {
    obstacleAvoided = true;
    obstacleInterrupt = false;
    obstacleDetected = false;
    previousTargetState = true;
  }
}

void ObstacleController::setTargetHeldClear()
{
  //adjust current state on transition from cube held to cube not held
  if (targetHeld)
  {
    Reset();
    targetHeld = false;
    previousTargetState = false;
    ignore_center_sonar = false;
  }
}

long int ObstacleController::getROSTimeInMilliSecs_S()
{
    // Get the current time according to ROS (will be zero for simulated clock until the first time message is recieved).
    ros::Time t = ros::Time::now();

    // Convert from seconds and nanoseconds to milliseconds.
    return t.sec*1e3 + t.nsec/1e6;
}

