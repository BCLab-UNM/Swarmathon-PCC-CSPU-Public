#include "DropOffController.h"

DropOffController::DropOffController() {

  reachedCollectionPoint = false;

  result.type = behavior;
  result.b = wait;
  result.wristAngle = 0.7;
  result.reset = false;
  interrupt = false;

  circularCenterSearching = false;
  spinner = 0;
  centerApproach = false;
  seenEnoughCenterTags = false;
  prevCount = 0;

  countLeft = 0;
  countRight = 0;

  isPrecisionDriving = false;
  startWaypoint = false;
  timerTimeElapsed = -1;

}

DropOffController::~DropOffController() {

}

Result DropOffController::DoWork() {

  cout << "8" << endl;

  int count = countLeft + countRight;

  if(timerTimeElapsed > -1) {

    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
  //to resart our search.
  if(reachedCollectionPoint)
  {
    cout << "2" << endl;
    if (timerTimeElapsed >= 5)
    {
      if (finalInterrupt)
      {
        result.type = behavior;
        result.b = nextProcess;
        result.reset = true;
        return result;
      }
      else
      {
        finalInterrupt = true;
        cout << "1" << endl;
      }
    }
    else if (timerTimeElapsed >= 0.1)
    {
      isPrecisionDriving = true;
      result.type = precisionDriving;

      result.fingerAngle = M_PI_2; //open fingers
      result.wristAngle = 0; //raise wrist

      result.pd.cmdVel = -0.3;
      result.pd.cmdAngularError = 0.0;
    }

    return result;
  }

  double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);

  //check to see if we are driving to the center location or if we need to drive in a circle and look.
  if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && (count == 0)) {

    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(this->centerLocation);
    startWaypoint = false;
    isPrecisionDriving = false;

    timerTimeElapsed = 0;

    return result;

  }
  else if (timerTimeElapsed >= 2)//spin search for center
  {
    Point nextSpinPoint;

    //sets a goal that is 60cm from the centerLocation and spinner
    //radians counterclockwise from being purly along the x-axis.
    nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
    nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
    nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);

    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(nextSpinPoint);

    spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
    if (spinner > 2*M_PI) {
      spinner -= 2*M_PI;
    }
    spinSizeIncrease += spinSizeIncrement/8;
    circularCenterSearching = true;
    //safety flag to prevent us trying to drive back to the
    //center since we have a block with us and the above point is
    //greater than collectionPointVisualDistance from the center.

    returnTimer = current_time;
    timerTimeElapsed = 0;

  }

  bool left = (countLeft > 0);
  bool right = (countRight > 0);
  bool centerSeen = (right || left);
  
  /*
  //if(centerSeen == false)
  //{
    if(cv_see_home == true)
    {
      result.type = precisionDriving;
      result.pd.cmdVel = 0;
      return result;
    } 
 
  //}
  */
  //reset lastCenterTagThresholdTime timout timer to current time
  if ((!centerApproach && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {

    lastCenterTagThresholdTime = current_time;

  }

  if (count > 0 || seenEnoughCenterTags || prevCount > 0) //if we have a target and the center is located drive towards it.
  {

    cout << "9" << endl;
    centerSeen = true;

    if (first_center && isPrecisionDriving)
    {
      first_center = false;
      result.type = behavior;
      result.reset = false;
      result.b = nextProcess;
      return result;
    }
    isPrecisionDriving = true;

    if (seenEnoughCenterTags) //if we have seen enough tags
    {
      if ((countLeft-5) > countRight) //and there are too many on the left
      {
        right = false; //then we say none on the right to cause us to turn right
      }
      else if ((countRight-5) > countLeft)
      {
        left = false; //or left in this case
      }
    }

    float turnDirection = 1;
    //reverse tag rejection when we have seen enough tags that we are on a
    //trajectory in to the square we dont want to follow an edge.
    if (seenEnoughCenterTags) turnDirection = -3;

    result.type = precisionDriving;

    //otherwise turn till tags on both sides of image then drive straight
    if (left && right) {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }
    else if (right) {
      result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = -centeringTurnRate*turnDirection;
    }
    else if (left){
      result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = centeringTurnRate*turnDirection;
    }
    else
    {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }

    //must see greater than this many tags before assuming we are driving into the center and not along an edge.
    if (count > centerTagThreshold)
    {
      seenEnoughCenterTags = true; //we have driven far enough forward to be in and aligned with the circle.
      lastCenterTagThresholdTime = current_time;
    }
    if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
    {
      lastCenterTagThresholdTime = current_time;
    }
    //time since we dropped below countGuard tags
    long int elapsed = current_time - lastCenterTagThresholdTime;
    float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds

    //we have driven far enough forward to have passed over the circle.
    if (count < 1 && seenEnoughCenterTags && timeSinceSeeingEnoughCenterTags > dropDelay) {
      centerSeen = false;
    }
    centerApproach = true;
    prevCount = count;
    count = 0;
    countLeft = 0;
    countRight = 0;
  }

  //was on approach to center and did not seenEnoughCenterTags
  //for lostCenterCutoff seconds so reset.
  else if (centerApproach) {

    long int elapsed = current_time - lastCenterTagThresholdTime;
    float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds
    if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff)
    {
      cout << "4" << endl;
      //go back to drive to center base location instead of drop off attempt
      reachedCollectionPoint = false;
      seenEnoughCenterTags = false;
      centerApproach = false;

      result.type = waypoint;
      result.wpts.waypoints.push_back(this->centerLocation);
      if (isPrecisionDriving) {
        result.type = behavior;
        result.b = prevProcess;
        result.reset = false;
      }
      isPrecisionDriving = false;
      interrupt = false;
      precisionInterrupt = false;
      
    }
    else
    {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }

    return result;

  }

  if (!centerSeen && seenEnoughCenterTags)
  {
    reachedCollectionPoint = true;
    centerApproach = false;
    returnTimer = current_time;
  }

  return result;
}

void DropOffController::Reset() {
  result.type = behavior;
  result.b = wait;
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError = 0;
  result.fingerAngle = -1;
  result.wristAngle = 0.7;
  result.reset = false;
  result.wpts.waypoints.clear();
  spinner = 0;
  spinSizeIncrease = 0;
  prevCount = 0;
  timerTimeElapsed = -1;

  countLeft = 0;
  countRight = 0;


  //reset flags
  reachedCollectionPoint = false;
  seenEnoughCenterTags = false;
  circularCenterSearching = false;
  isPrecisionDriving = false;
  finalInterrupt = false;
  precisionInterrupt = false;
  targetHeld = false;
  startWaypoint = false;
  first_center = true;
  cout << "6" << endl;

}

void DropOffController::SetTargetData(vector<Tag> tags) {
  countRight = 0;
  countLeft = 0;

  if(targetHeld) {
    // if a target is detected and we are looking for center tags
    if (tags.size() > 0 && !reachedCollectionPoint) {

      // this loop is to get the number of center tags
      for (int i = 0; i < tags.size(); i++) {
        if (tags[i].getID() == 256) {

          // checks if tag is on the right or left side of the image
          if (tags[i].getPositionX() + cameraOffsetCorrection > 0) {
            countRight++;

          } else {
            countLeft++;
          }
        }
      }
    }
  }

}

// Alex C uses opencv to help locate the home base
void DropOffController::ProcessImage(){
  cv_see_home = false;
  if(img.data == NULL)
  {
    return;
  }

  int morph_operation = 3;
  int morph_size = 10;
  int morph_element = 2;

  int blur_factor = 3;

  int thresh = 225;

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

    
    // if the area of the contours is greater than 6000 (arbitrary number to make sure we are looking at home base)
    if( mu[i].m00 > 3000 && mc[i].y < 240/2.0)
    {
      int b,g,r;
      b = rand()%256;
      g = rand()%256;  
      r = rand()%256;
      
      cv::drawContours( drawing, contours, i, cv::Scalar(b,g,r), 2, 8, hierarchy, 0, cv::Point(0,0) );
      cv::circle(drawing, mc[i], 4, cv::Scalar(255-b,255-g,255-r), -1, 8, 0 );
      cv::putText(drawing, text.str(), cv::Point(mc[i].x,mc[i].y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      cv::putText(drawing, "Home Base", cv::Point(mc[i].x,mc[i].y+20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
      cv_see_home = true;
    }
    else
    {
      cv::drawContours( drawing, contours, i, cv::Scalar(0,255,0), 2, 8, hierarchy, 0, cv::Point(0,0) );
      cv::circle(drawing, mc[i], 4, cv::Scalar(255,0,0), -1, 8, 0 );
      cv::putText(drawing, text.str(), cv::Point(mc[i].x,mc[i].y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255));
    }
  }

  /*
  cv::imshow("ProcessImage::morph",morph);
  cv::imshow("ProcessImage::drawing",drawing);
  cv::imshow("ProcessImage::raw",img);
  cv::waitKey(1);
  */
}


void DropOffController::ProcessData() {
  ProcessImage();
  

  if((countLeft + countRight) > 0) {
    isPrecisionDriving = true;
  } else {
    startWaypoint = true;
  }
}

bool DropOffController::ShouldInterrupt() {
  ProcessData();
  if (startWaypoint && !interrupt) {
    interrupt = true;
    precisionInterrupt = false;
    return true;
  }
  else if (isPrecisionDriving && !precisionInterrupt) {
    precisionInterrupt = true;
    return true;
  }
  if (finalInterrupt) {
    return true;
  }
}

bool DropOffController::HasWork() {

  if(timerTimeElapsed > -1) {
    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  if (circularCenterSearching && timerTimeElapsed < 2 && !isPrecisionDriving) {
    return false;
  }

  return ((startWaypoint || isPrecisionDriving));
}

bool DropOffController::IsChangingMode() {
  return isPrecisionDriving;
}

void DropOffController::SetCenterLocation(Point center) {
  centerLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
  currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
  targetHeld = true;
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
  targetHeld = targetHeld || blockBlock;
}

void DropOffController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

// Alex c
void DropOffController::UpdateFrame(const cv::Mat image){
  img = image.clone();
}
