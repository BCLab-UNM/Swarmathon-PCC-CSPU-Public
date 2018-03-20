#include "PickUpController.h"
#include <limits> // For numeric limits
#include <cmath> // For hypot


// TODO: Remove these includes and remove all debugging messages
// for debugging 
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
extern ros::Publisher infoLogPublisher;
extern ros::Publisher fingerAnglePublish;

/* NOTES
  search controller constructor
  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
*/


int closeClawOffsetSim 			= 0; // TODO: Set this value to 0, this is only for simulation.  //2
float extraTimeOffsetSim 		= 0; //TODO: Set this value to 0. this is only for simulation.  //1.3


float fingerAngleOpen		 	= M_PI_2;
float fingerAngleClose 			= 0;
float wristAngleDown 			= 1; // optimal angle for picking up cube from the center (1.25 is all the way down)
float wristAngleUp 			= 0; //wrist angle all the way up

float grasp_time_begin 			= 0.7 + extraTimeOffsetSim;
float raise_time_begin 			= 1.6 + extraTimeOffsetSim;
float check_time_begin 			= 2.6 + closeClawOffsetSim + extraTimeOffsetSim; 
float lower_gripper_time_begin 		= 4.6 + closeClawOffsetSim + extraTimeOffsetSim;
float target_reaquire_begin		= 4.8 + closeClawOffsetSim + extraTimeOffsetSim;
float target_pickup_task_time_limit 	= 5.6 + closeClawOffsetSim + extraTimeOffsetSim;


float last_rotation_before_being_lost = 0; // this variable keeps track of the rotation of the rover the instant it loses track of the cube,
					   // (it is useful to rotate the rover the opposite direction when the cube is lost)

int frame_counter = 0; // This variable keeps track of the number of frames that the rover thinks the cube is in, using only opencv
int frame_write_counter = 0; // TODO: Remove this variable, this is only for debugging the filtered frames.

PickUpController::PickUpController()
{
  lockTarget = false;
  timeOut = false;
  nTargetsSeen = 0;
  blockYawError = 0;
  blockDistance = 0;

  targetFound = false;

  result.type = precisionDriving;
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError= 0;
  result.fingerAngle = -1;
  result.wristAngle = -1;
  result.PIDMode = SLOW_PID;

  frame_counter = 0;
  last_rotation_before_being_lost = 0;
}

PickUpController::~PickUpController() { /*Destructor*/  }

void PickUpController::SetTagData(vector<Tag> tags)
{


  if (tags.size() > 0)
  {
    
    nTargetsSeen = tags.size();

    //we saw a target, set target_timer
    target_timer = current_time;

    double closest = std::numeric_limits<double>::max();
    int target  = 0;

    //this loop selects the closest visible block to makes goals for it
    for (int i = 0; i < tags.size(); i++)
    {

      if (tags[i].getID() == 0)
      {

        last_rotation_before_being_lost = result.pd.cmdAngularError;

        targetFound = true;

        //absolute distance to block from camera lens
        double test = hypot(hypot(tags[i].getPositionX(), tags[i].getPositionY()), tags[i].getPositionZ());

        if (closest > test)
        {
          target = i;
          closest = test;
        }
      }
      else
      {

        if(tags[i].getID() == 256)
        {

          Reset();

          if (has_control)
          {
            cout << "pickup reset return interupt free" << endl;
            release_control = true;
          }

          return;
        }
      }
    }

    float cameraOffsetCorrection = 0.023; //meters;

    ///TODO: Explain the trig going on here- blockDistance is c, 0.195 is b; find a
    blockDistanceFromCamera = hypot(hypot(tags[target].getPositionX(), tags[target].getPositionY()), tags[target].getPositionZ());

    if ( (blockDistanceFromCamera*blockDistanceFromCamera - 0.195*0.195) > 0 )
    {
      blockDistance = sqrt(blockDistanceFromCamera*blockDistanceFromCamera - 0.195*0.195);
    }
    else
    {
      float epsilon = 0.00001; // A small non-zero positive number
      blockDistance = epsilon;
    }

    //cout << "blockDistance  TAGDATA:  " << blockDistance << endl;

    blockYawError = atan((tags[target].getPositionX() + cameraOffsetCorrection)/blockDistance)*1.05; //angle to block from bottom center of chassis on the horizontal.

    cout << "blockYawError TAGDATA:  " << blockYawError << endl;

  }

}

bool PickUpController::SetSonarData(float r)
{

  rangeCenter = r;
  if (rangeCenter < 0.12 && targetFound)
  {
    result.type = behavior;
    result.b = nextProcess;
    result.reset = true; 
    targetHeld = true;
    return true;
  }

  return false;

}

void PickUpController::ProcessData()
{

  if(!targetFound)
  {
    //cout << "PICKUP No Target Seen!" << endl;

    // Do nothing
    return;
  }

  //diffrence between current time and millisecond time
  long int Tdiff = current_time - millTimer;
  float Td = Tdiff/1e3;

  //cout << "PICKUP Target Seen!" << endl;

  //cout << "distance : " << blockDistanceFromCamera << " time is : " << Td << endl;

  

  
  if(Td > check_time_begin && Td < lower_gripper_time_begin && ProcessImage())
    frame_counter ++;

  stringstream ss;
  ss << "\nPickupController::ProcessData" << "\n";
  ss << "Td = " << Td << "\n";
  ss << "frameCounter = " << frame_counter << "\n";
  ss << "blockDistanceFromCamera = " << blockDistanceFromCamera << "\n";
  std_msgs::String msg;
  msg.data = ss.str();
  //infoLogPublisher.publish(msg);

  
  if(Td > check_time_begin && Td < lower_gripper_time_begin && ProcessImage())
    frame_counter ++;

  if ((Td > check_time_begin + 1.5 && Td < target_pickup_task_time_limit + 0.1 && blockDistanceFromCamera < 0.16 ) || frame_counter > 8)
  {
    frame_counter = 0;
    result.type = behavior;
    result.b = nextProcess;
    result.reset = true;
    targetHeld = true;

  }
  //Lower wrist and open fingures if no locked target
  else if (!lockTarget )
  {

    frame_counter = 0;

    //set gripper;
    result.fingerAngle = M_PI_2;
    result.wristAngle = wristAngleDown;//1.25
  }

}

// Alex C uses opencv to help locate the home base
bool PickUpController::ProcessImage(){

  if(img.data == NULL)
  {
    return false;
  }

  stringstream ss;
  ss << "\nPickupController::ProcessImage" << "\n";

  bool return_me = false;

  int morph_operation = 3;
  int morph_size = 10;
  int morph_element = 2;

  int blur_factor = 3;

  int thresh = 185; // 225

  cv::Mat morph, morph_gray, blur, threshold;
  
  cv::Mat element = getStructuringElement(morph_element, cv::Size(2*morph_size+1, 2*morph_size+1), cv::Point(morph_size, morph_size));
  
  // erode and dilate image
  cv::morphologyEx(img, morph, morph_operation, element);

  // morphologyEx will output an image with color
  cv::cvtColor(morph, morph_gray, CV_BGR2GRAY);

  cv::blur(morph_gray, blur, cv::Size(blur_factor,blur_factor));

  cv::threshold(blur,threshold, thresh, 255, cv::THRESH_BINARY);


  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  cv::findContours(threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );
  
  // get moments - this is just to get every object detected on the image based on the contours
  vector<cv::Moments> mu( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
  {
    mu[i] = cv::moments( contours[i], false); 
  }

  // get mass center (the area of the objects detected)
  vector<cv::Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
  {
    mc[i] = cv::Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
  }

  // draw contours
  cv::Mat drawing = cv::Mat::zeros( threshold.size(), CV_8UC3 );
  // draw each object detected with different colors
  for( int i = 0; i < contours.size(); i++ )
  {
    stringstream text;
    text << mu[i].m00;
    
    ss << "Area = " << mu[i].m00 << "\n";
    
    // if the area of the contours is greater than 6000 (arbitrary number to make sure we are looking at home base)
    if( mu[i].m00 > 8000 )
    {
      int b,g,r;
      b = rand()%256;
      g = rand()%256;  
      r = rand()%256;
      
      cv::drawContours( drawing, contours, i, cv::Scalar(b,g,r), 2, 8, hierarchy, 0, cv::Point(0,0) );
      cv::circle(drawing, mc[i], 4, cv::Scalar(255-b,255-g,255-r), -1, 8, 0 );
      cv::putText(drawing, text.str(), cv::Point(mc[i].x,mc[i].y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));

      if(mc[i].y < 240/2.0)
      {
        cv::putText(drawing, "Not A Cube", cv::Point(mc[i].x,mc[i].y+20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      }
      else
      {
        cv::putText(drawing, "Cube", cv::Point(mc[i].x,mc[i].y+20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
        return_me = true;
      }
      
    }
    else
    {
      cv::drawContours( drawing, contours, i, cv::Scalar(0,255,0), 2, 8, hierarchy, 0, cv::Point(0,0) );
      cv::circle(drawing, mc[i], 4, cv::Scalar(255,0,0), -1, 8, 0 );
      cv::putText(drawing, text.str(), cv::Point(mc[i].x,mc[i].y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
    }
  }


  std_msgs::String msg;
  msg.data = ss.str();
  //infoLogPublisher.publish(msg);

  /*
  cv::imshow("ProcessImage::morph",morph);
  cv::imshow("ProcessImage::drawing",drawing);
  cv::imshow("ProcessImage::raw",img);
  cv::waitKey(1);
  */
 

  /*
  frame_write_counter++;
  stringstream dPath;
  dPath << "../tempImages/drawing"<<frame_write_counter<<".jpg";
  stringstream iPath;
  iPath << "../tempImages/img"<<frame_write_counter<<".jpg";
  stringstream mPath;
  mPath << "../tempImages/morph"<<frame_write_counter<<".jpg";

  cv::imwrite(dPath.str(),drawing);
  cv::imwrite(iPath.str(),img);
  cv::imwrite(mPath.str(),morph);
  */
  
  return return_me;
}


bool PickUpController::ShouldInterrupt(){

  ProcessData();

  if (release_control)
  {
    release_control = false;
    has_control = false;
    return true;
  }

  if ((targetFound && !interupted) || targetHeld)
  {
    interupted = true;
    has_control = false;
    return true;
  }
  else if (!targetFound && interupted)
  {
    interupted = false;
    has_control = false;
    return true;
  }
  else
  {
    return false;
  }
}


Result PickUpController::DoWork()
{

  // alex c Debug
  stringstream ss;
  ss << "\nPickupController::DoWork" << "\n";

  has_control = true;
  
  if (!targetHeld)
  {

    //threshold distance to be from the target block before attempting pickup
    float targetDistance = 0.2; // 0.15//meters

    // -----------------------------------------------------------
    // millisecond time = current time if not in a counting state
    //     when timeOut is true, we are counting towards a time out
    //     when timeOut is false, we are not counting towards a time out
    //
    // In summary, when timeOut is true, the robot is executing a pre-programmed time-based block pickup
    // I routine. <(@.@)/"
    // !!!!! AND/OR !!!!!
    // The robot has started a timer so it doesn't get stuck trying to pick up a cube that doesn't exist.
    //
    // If the robot does not see a block in its camera view within the time out period, the pickup routine
    // is considered to have FAILED.
    //
    // During the pre-programmed pickup routine, a current value of "Td" is used to progress through
    // the routine. "Td" is defined below...
    // -----------------------------------------------------------
    if (!timeOut) millTimer = current_time;

    //difference between current time and millisecond time
    long int Tdifference = current_time - millTimer;

    // converts from a millisecond difference to a second difference
    // Td = [T]ime [D]ifference IN SECONDS
    float Td = Tdifference/1e3;

    // The following nested if statement implements a time based pickup routine.
    // The sequence of events is:
    // 1. Target aquisition phase: Align the robot with the closest visible target cube, if near enough to get a target lock then start the pickup timer (Td)
    // 2. Approach Target phase: until *grasp_time_begin* seconds
    // 3. Stop and close fingers (hopefully on a block - we won't be able to see it remember): at *grasp_time_begin* seconds 
    // 4. Raise the gripper - does the rover see a block or did it miss it: at *raise_time_begin* seconds 
    // 5. If we get here the target was not seen in the robots gripper so drive backwards and and try to get a new lock on a target: at *target_reaquire_begin* seconds
    // 6. If we get here we give up and release control with a task failed flag: for *target_pickup_task_time_limit* seconds
    
    // If we don't see any blocks or cubes turn towards the location of the last cube we saw.
    // I.E., try to re-aquire the last cube we saw.

    

    //cout << "blockDistance DOWORK:  " << blockDistance << endl;

    //Calculate time difference between last seen tag
    float target_timeout = (current_time - target_timer)/1e3;

    //delay between the camera refresh and rover runtime is 6/10's of a second
    float target_timeout_limit = 0.61;

    //Timer to deal with delay in refresh from camera and the runtime of rover code
    if( target_timeout >= target_timeout_limit )
    {
        //Has to be set back to 0
        nTargetsSeen = 0;
    }

    // first line of defence, stand still
    if (nTargetsSeen == 0 && !lockTarget)
    {
      // This if statement causes us to time out if we don't re-aquire a block within the time limit.
      if(!timeOut)
      {
        result.pd.cmdVel = 0.0;
        result.pd.cmdAngularError= 0;

        result.wristAngle = wristAngleDown; //1.25
        // result.fingerAngle does not need to be set here

        // We are getting ready to start the pre-programmed pickup routine now! Maybe? <(^_^)/"
        // This is a guard against being stuck permanently trying to pick up something that doesn't exist.
        timeOut = true;

      }
      // second line of defence, look in the opposite direction that the rover was turning before it lost sight of cube
      else if (Td > 0.7 && Td < 2.0)
      {
        if(blockDistance > 0.35)
        {
          // The rover will slowly go towards the cube's last position
          result.pd.cmdVel = 0.05;// -0.15 Alex C
        }
        else
        {
          // The rover will reverse angling towards the direction of the last seen cube.
          result.pd.cmdVel = -0.05;// -0.15 Alex C
        }

        if(blockYawError != 0) 
        {

          if(last_rotation_before_being_lost == 0)// make sure not dividng by 0
          {
            last_rotation_before_being_lost =  blockYawError/2.0;
          } 

          float newAngle = -0.07 * (last_rotation_before_being_lost / fabs(last_rotation_before_being_lost) ); // basically turn towards the opposite direction of what the rover 
													     // was turning at when it lost sight of the cube at a rotational speed of 0.07.

          // 0.1 * (-blockYawError / abs(blockYawError) ); // turn at an anglular speed of 0.1 in the direction of the last seen spot of the cube
          result.pd.cmdAngularError= newAngle; // -blockYawError
        }

      }
      // last line of defence, turn towards the direction of the last location of the cube on the frame.
      else if (Td >= 2.0 && Td < target_pickup_task_time_limit)
      {
        if(blockDistance > 0.35)
        {
          // The rover will slowly go towards the cube's last position
          result.pd.cmdVel = 0.05;// -0.15 Alex C
        }
        else
        {
          // The rover will reverse angling towards the direction of the last seen cube.
          result.pd.cmdVel = -0.05;// -0.15 Alex C
        }

        result.pd.cmdAngularError = -blockYawError/2.0; 
                     
      }

        
        ss << "I know i just saw a cube somewhere" << "\n";
        ss << "-blockYawError = " << -blockYawError << "\n";
        ss << "LRBBL = " << last_rotation_before_being_lost << "\n";
        ss << "Block Distance = " << blockDistance << "\n";
        ss << "cmdAngularError = " << result.pd.cmdAngularError << "\n";
        std_msgs::String msg;
        msg.data = ss.str();
        //infoLogPublisher.publish(msg);
    }
    else if (blockDistance > targetDistance && !lockTarget) //if a target is detected but not locked, and not too close.
    {
      // this is a 3-line P controller, where Kp = 0.20
      float vel = blockDistance * 0.20;
      if (vel < 0.1) vel = 0.1;
      if (vel > 0.2) vel = 0.2;

      result.pd.cmdVel = vel;
      float dist = blockDistance;

      //if(dist == 0){dist = 0.00001;} // not necessary because parent if condition states blockDistance must be greater than 0.15
      
      result.pd.cmdAngularError = -blockYawError/(0.6/dist);// alex c, the closer the rover gets to the target, the less drastic it turns
      timeOut = false;

      return result;
    }
    else if (!lockTarget) //if a target hasn't been locked lock it and enter a counting state while slowly driving forward.
    {
      
      lockTarget = true;
      result.pd.cmdVel = 0.2;
      result.pd.cmdAngularError= 0.0;
      timeOut = true;
      ignoreCenterSonar = true;
    }
    else if (Td > check_time_begin)//Alex C
    {
      result.pd.cmdVel = -0.15;
      result.pd.cmdAngularError= 0.0;
      result.fingerAngle = fingerAngleClose; // alex c
      result.wristAngle = wristAngleUp;// do not raies wrist up all the way, so we can isolate the cube to the bottom half of the frame
    }
    else if (Td > raise_time_begin) //raise the wrist
    {
      result.pd.cmdVel = 0; // -0.15
      result.pd.cmdAngularError= 0.0;
      result.fingerAngle = fingerAngleClose; // alex c
      result.wristAngle = wristAngleUp;// do not raies wrist up all the way, so we can isolate the cube to the bottom half of the frame
      return result;
    }
    else if (Td > grasp_time_begin) //close the fingers and stop driving
    {
      result.pd.cmdVel = 0.0;
      result.pd.cmdAngularError= 0.0;
      result.fingerAngle = fingerAngleClose;
      return result;  
    }


    // the magic numbers compared to Td must be in order from greater(top) to smaller(bottom) numbers
    if (Td > target_reaquire_begin && timeOut)
    {
      lockTarget = false;
      ignoreCenterSonar = true;
    }

    //if enough time has passed enter a recovery state to re-attempt a pickup
    else if (Td > lower_gripper_time_begin && timeOut)
    {
      result.pd.cmdVel = -0.15;
      result.pd.cmdAngularError= 0.0;
      //set gripper to open and down
      result.fingerAngle = M_PI_2;
      result.wristAngle = wristAngleUp;
    }

    //if no targets are found after too long a period go back to search pattern
    if (Td > target_pickup_task_time_limit && timeOut)
    {
      Reset();
      interupted = true;
      result.pd.cmdVel = 0.0;
      result.pd.cmdAngularError= 0.0;
      ignoreCenterSonar = true;
    }
  }

  return result;
}

bool PickUpController::IsHoldingCube()
{
  return (blockDistanceFromCamera < 0.16) || (rangeCenter < 0.12 && targetFound);
}

bool PickUpController::HasWork()
{
  return targetFound;
}

void PickUpController::Reset() {

  result.type = precisionDriving;
  result.PIDMode = SLOW_PID;
  lockTarget = false;
  timeOut = false;
  nTargetsSeen = 0;
  blockYawError = 0;
  blockDistance = 0;

  targetFound = false;
  interupted = false;
  targetHeld = false;

  result.pd.cmdVel = 0;
  result.pd.cmdAngularError= 0;
  result.fingerAngle = -1;
  result.wristAngle = -1;
  result.reset = false;

  frame_counter = 0;
  last_rotation_before_being_lost = 0;

  ignoreCenterSonar = false;
}

void PickUpController::SetUltraSoundData(bool blockBlock){
  this->blockBlock = blockBlock;
}

void PickUpController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

void PickUpController::UpdateFrame(const cv::Mat image){
  img = image.clone();
}

