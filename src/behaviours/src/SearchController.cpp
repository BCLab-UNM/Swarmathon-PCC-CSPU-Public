#include "SearchController.h"
#include <angles/angles.h>

#include <ros/ros.h>
extern char currentState;
int counter = 0;//for testing
const double RADIUS = 0.7;//TODO: set a proper radius based on the home detection capability
double cur_rad = 2;

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
    ROS_INFO_STREAM("count: " << counter << " attemp: " << attemptCount << " IN SEARCH");
//    counter ++;//for testing pattern walk
    //initialize pattern walk
    if (!result.wpts.waypoints.empty()) {
	testingDistance += hypot(currentLocation.x - testing_last_pos.x, currentLocation.y - testing_last_pos.y);
        ROS_INFO_STREAM("IN SEARCH testingDISSSSSSSSS: " << testingDistance);
	
	testing_last_pos = currentLocation;
	 
        if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
            attemptCount = 0;
            ROS_INFO_STREAM("IN SEARCH: I am very close to the last search location attemptCount = 0");
            ROS_INFO_STREAM("IN SEARCH: cur loc: " << currentLocation.x << ", " << currentLocation.y);
            ROS_INFO_STREAM("IN SEARCH: wpt loc: " << result.wpts.waypoints[0].x << ", " <<  result.wpts.waypoints[0].y);
        }
    }else{
	//localization testing
	testing_last_pos = currentLocation;
    }

    if (attemptCount > 0 && attemptCount < 1) {
        attemptCount++;
        //ROS_INFO_STREAM("IN SEARCH: attemp > 0 and < 5");
        if (succesfullPickup) {
            ROS_INFO_STREAM("IN SEARCH: successfulPickup");
            succesfullPickup = false;
            attemptCount = 1;
        }

        return result;
    }

    else if (attemptCount >= 1 || attemptCount == 0)
    {
        ROS_INFO_STREAM("IN SEARCH: attemp >= 5 || == 0");
        attemptCount = 1;


        result.type = waypoint;
        //        Point  searchLocation;


        ////             back and forth pattern
        //back and forth from home to a meter away
        //     localization testing
        if(counter%2){
            searchLocation.theta = this->centerLocation.theta;
            searchLocation.x = this->centerLocation.x;
            searchLocation.y = this->centerLocation.y;
            ROS_INFO_STREAM("IN SEARCH: GOING HOME: " << searchLocation.x << ", " << searchLocation.y);
        }else{
            searchLocation.theta = currentLocation.theta + M_PI;
            //                searchLocation.x = currentLocation.x + (1 * cos(searchLocation.theta));
            //                searchLocation.y = currentLocation.y + (1 * sin(searchLocation.theta));

//            cur_rad += RADIUS;
            searchLocation.x = currentLocation.x + (cur_rad * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + (cur_rad * sin(searchLocation.theta));
            ROS_INFO_STREAM("IN SEARCH: GOING OUT: " << searchLocation.x << ", " << searchLocation.y);
        }
        counter ++;



        //end of back and forth

        //select new position 50 cm from current location
        //    if (first_waypoint)
        //    {
        //      first_waypoint = false;
        //      searchLocation.theta = currentLocation.theta + M_PI;
        //      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
        //      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
        //    }
        //    else
        //    {
        ////      select new heading from Gaussian distribution around current heading
        //      searchLocation.theta = currentLocation.theta + M_PI;; //45 degrees in radians
        //      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
        //      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));

        //        spirial walk for looking for home.
        //        if(counter%2){
        //            cur_rad += RADIUS;
        //        }
        //        searchLocation.theta = currentLocation.theta + M_PI/2;
        //        searchLocation.x = currentLocation.x + cur_rad*cos(searchLocation.theta);
        //        searchLocation.y = currentLocation.y + cur_rad*sin(searchLocation.theta);
        //        counter++;
        //        //end of spirial walk
        //    }


        ////pattern walk
/**
        if(!initialized){
            THETA_TRAN = currentLocation.theta + M_PI/2;//get the very first theta
            //set the intial searchPoint
            setIntialPos();
            searchLocation = nextGoalPos;
            //set temp point
            float r = sqrt(pow(searchLocation.x, 2) + pow(searchLocation.y, 2));
            float cur_theta = atan2(this->searchLocation.y , this->searchLocation.x);
            this->tempPoint.theta = cur_theta - THETA_TRAN;
            this->tempPoint.x = r * cos(this->tempPoint.theta);
            this->tempPoint.y = r * sin(this->tempPoint.theta);
            initialized = true;
            ROS_INFO_STREAM("IN SEARCH: THETA_TRAN: " << THETA_TRAN);
            ROS_INFO_STREAM("IN SEARCH: INCREASE_x_y: " << INCREASE_x_y);
            ROS_INFO_STREAM("IN SEARCH: cur loc: " << currentLocation.x << ", " << currentLocation.y);
            ROS_INFO_STREAM("IN SEARCH: searchLoc: " << searchLocation.x << "," <<searchLocation.y );
            ROS_INFO_STREAM("IN SEARCH: initialized tempPoint: " << tempPoint.x << ", " << tempPoint.y);
        }else{
            setNextGoalPos();
            searchLocation = nextGoalPos;
            ROS_INFO_STREAM("IN SEARCH: cur loc: " << currentLocation.x << ", " << currentLocation.y);
            ROS_INFO_STREAM("IN SEARCH: new loc: " << searchLocation.x << ", " << searchLocation.y);
        }
        **/

        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

        return result;
    }

}

void SearchController::SetCenterLocation(Point centerLocation) {

    float diffX = this->centerLocation.x - centerLocation.x;
    float diffY = this->centerLocation.y - centerLocation.y;
    this->centerLocation = centerLocation;

    if (!result.wpts.waypoints.empty())
    {
        result.wpts.waypoints.back().x -= diffX;
        result.wpts.waypoints.back().y -= diffY;

        //for pattern walk
        tempPoint.x -= diffX;
        tempPoint.y -= diffY;
    }
    ROS_INFO_STREAM("IN SEARCH: CENTER: " << centerLocation.x << ", " << centerLocation.y);
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

/////////////////////////////////
///////// pattern walk /////////
///////////////////////////////
void SearchController::setNextGoalPos(){
    this->r_patterWalk = sqrt(pow(tempPoint.x, 2) + pow(tempPoint.y, 2));
    float cur_theta = atan2(this->tempPoint.y , this->tempPoint.x);

    this->nextGoalPos.theta = cur_theta - THETA_TRAN;
    this->nextGoalPos.x = r_patterWalk * cos(this->nextGoalPos.theta);
    this->nextGoalPos.y = r_patterWalk * sin(this->nextGoalPos.theta);

    tempPoint = calculateTempPoint();
}

void SearchController::setIntialPos(){
    this->nextGoalPos.theta = currentLocation.theta + M_PI/2;
    this->nextGoalPos.x = currentLocation.x + START_LEN*cos(this->nextGoalPos.theta);
    this->nextGoalPos.y = currentLocation.y + START_LEN*sin(this->nextGoalPos.theta);
}

Point SearchController::calculateTempPoint(){
    Point newTemp;
    if (this->tempPoint.x < 0){
        newTemp.x = -(this->tempPoint.x /*- this->INCREASE_x_y*/);
    }else{
        newTemp.x = -(this->tempPoint.x + this->INCREASE_x_y);
    }
    newTemp.y += (this->tempPoint.y - this->INCREASE_x_y);

    ROS_INFO_STREAM("IN SEARCH: new temp: " << newTemp.x << " ," << newTemp.y);

    return newTemp;
}
