#include "SearchController.h"
#include <angles/angles.h>

#include <ros/ros.h>
extern char currentState;
int counter = 0;//for testing
const double RADIUS = 0.7;//TODO: set a proper radius based on the home detection capability
double cur_rad = 1;
int ATTEMPTS = 4;

SearchController::SearchController() {
    rng = new random_numbers::RandomNumberGenerator();
    currentLocation.x = 0;
    currentLocation.y = 0;
    currentLocation.theta = 0;

    centerLocation.x = 0;
    centerLocation.y = 0;
    centerLocation.theta = 0;
    result.PIDMode = FAST_PID;

    result.fingerAngle = M_PI/2;
    result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
    result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
    currentState = '1';
    //ROS_INFO_STREAM("count: " << counter << " attemp: " << attemptCount << " IN SEARCH");
    counter ++;//for testing pattern walk
    //initialize pattern walk
    if (!result.wpts.waypoints.empty()) {
       // ROS_INFO_STREAM("IN SEARCH: waypoint is not empty");
        if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
            attemptCount = 0;
         //   ROS_INFO_STREAM("IN SEARCH: I am very close to the last search location attemptCount = 0");
         //   ROS_INFO_STREAM("IN SEARCH: cur loc: " << currentLocation.x << ", " << currentLocation.y);
          //  ROS_INFO_STREAM("IN SEARCH: wpt loc: " << result.wpts.waypoints[0].x << ", " <<  result.wpts.waypoints[0].y);
        }
    }else{
        //for fixing the interuption by obstacle avoicance after waypoint gets cleared up but not yet assigning a new one
        attemptCount = 0;
    }

    if (attemptCount > 0 && attemptCount < ATTEMPTS) {
        attemptCount++;
        if (succesfullPickup) {
          //  ROS_INFO_STREAM("IN SEARCH: successfulPickup");
            succesfullPickup = false;
            attemptCount = 1;
        }

        return result;
    }
    else if (attemptCount >= ATTEMPTS || attemptCount == 0)
    {
        attemptCount = 1;
        result.type = waypoint;

        if(first_waypoint){
            //set the intial searchPoint
            this->nextSpinPoint.theta = currentLocation.theta + M_PI;
            this->nextSpinPoint.x = currentLocation.x + depthFromHome*cos(this->nextSpinPoint.theta);
            this->nextSpinPoint.y = currentLocation.y + depthFromHome*sin(this->nextSpinPoint.theta);
            first_waypoint = false;
            startPoint = nextSpinPoint;
        }else{
            //sets a goal that is 60cm from the initial location and spinner
            //radians counterclockwise from being purly along the x-axis.
            do{
                this->nextSpinPoint.x = startPoint.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
                this->nextSpinPoint.y = startPoint.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
                this->nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);

                spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
                if (spinner > 2*M_PI) {
                    spinner -= 2*M_PI;
                }
                spinSizeIncrease += spinSizeIncrement/8;
                searchLocation.x -= diffX;////TODO: test if it is needed
                searchLocation.y -= diffY;//// TODO:test if it is needed
            }while(hypot(nextSpinPoint.x - centerLocation.x, nextSpinPoint.y- centerLocation.y) < 2);

        }
        searchLocation = this->nextSpinPoint;

        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

       // ROS_INFO_STREAM("IN SEARCH: cur loc: " << currentLocation.x << ", " << currentLocation.y <<"|" << currentLocation.theta);
       // ROS_INFO_STREAM("IN SEARCH: new loc: " << searchLocation.x << ", " << searchLocation.y << "|" << searchLocation.theta);


        return result;
    }
}

void SearchController::SetCenterLocation(Point centerLocation) {
    ////todo: set heading
    diffX = this->centerLocation.x - centerLocation.x;
    diffY = this->centerLocation.y - centerLocation.y;
    this->centerLocation = centerLocation;

    if (!result.wpts.waypoints.empty())
    {
        result.wpts.waypoints.back().x -= diffX;
        result.wpts.waypoints.back().y -= diffY;
    }

    //drift the next goal location
    nextSpinPoint.x -= diffX;
    nextSpinPoint.y -= diffY;

    //ROS_INFO_STREAM("IN SEARCH: CENTER: " << centerLocation.x << ", " << centerLocation.y);
}

void SearchController::SetCurrentLocation(Point currentLocation) {
    this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
    ProcessData();

    return false;
}

bool SearchController::HasWork() {
    return true;
}

void SearchController::SetSuccesfullPickup() {
    succesfullPickup = true;
}

//void SearchController::setSubGoalPos(){
//    //float dnt = hypot(subGoalPos.x - nextGoalPos.x, subGoalPos.y - nextGoalPos.y);
////    ROS_INFO_STREAM("IN SUB: hypot: " << dnt);
////    if(dnt >= subInterval){
//    ROS_INFO_STREAM("IN SUB: subDistanceTotal: " << subDistanceTotal);
//    if(subDistanceTotal < this->c){
//        subDistanceTotal += subInterval;
//        ROS_INFO_STREAM("IN SUB <0.8: sub.x - next.x: " << subGoalPos.x << "-" << nextGoalPos.x << " ;" <<
//                                                          subGoalPos.y << "-" << nextGoalPos.y);
//        subGoalPos.x += subInterval * cos(nextGoalPos.theta);
//        subGoalPos.y += subInterval * sin(nextGoalPos.theta);
//        ROS_INFO_STREAM("IN SUB: subGoal: " << subGoalPos.x << ", " << subGoalPos.y);
//    }else{
//        subDistanceTotal = 0;
//        subGoalPos = nextGoalPos;
//        ROS_INFO_STREAM("IN SUB SUB: old next Goal: " << nextGoalPos.x << ", " << nextGoalPos.y);
//        nextGoalPos = setNextGoalPos();
//        ROS_INFO_STREAM("IN SUB: subGoal          : " << subGoalPos.x << ", " << subGoalPos.y);
//        ROS_INFO_STREAM("IN SUB: new next Goal    : " << nextGoalPos.x << ", " << nextGoalPos.y);

//    }
//    ROS_INFO_STREAM("\n");
//}



