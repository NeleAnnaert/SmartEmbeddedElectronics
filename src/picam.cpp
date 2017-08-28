#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include <raspicam/raspicam_cv.h>

static const std::string CAMERA_TOPIC = "/camera/image";
static const std::string MOVING_TOPIC = "/camera/moving";

using namespace std; 
using namespace cv;

bool checkMoving(Mat img1, Mat img2);

int main(int argc, char **argv)
{
  int looprate=2;
  float scale=1;

  /// Init ros and become node
  ros::init(argc, argv, "raspicam");
  ros::NodeHandle nh;

  /// Create a publisher object for camera frames
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pubCam = it.advertise(CAMERA_TOPIC, 1);

  /// Create and init a camera object
  raspicam::RaspiCam_Cv Camera;
  Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
  Camera.set( CV_CAP_PROP_WHITE_BALANCE_RED_V, 0.015 );
  Camera.set( CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 0.017 );
  Camera.set( CV_CAP_PROP_EXPOSURE, 3 );
  Camera.set( CV_CAP_PROP_FRAME_WIDTH, 800 );
  Camera.set( CV_CAP_PROP_FRAME_HEIGHT, 600 );

  /// Create a publisher object to see if the robot is moving
  ros::Publisher pubMov = nh.advertise<std_msgs::Bool>(MOVING_TOPIC,1);

  Mat img, imgPrev;

  /// Parse Arguments
  if (argc > 1)
  {
    looprate = atoi(argv[1]);
  }
  
  if (argc > 2)
  {
    scale = atof(argv[2]);
  }

  /// Open Camera
  if (!Camera.open())
  {
    ROS_ERROR_STREAM("Could not open Camera!");
    ros::shutdown();
    exit(1);
  }

  /// Grab first image
  Camera.grab();
  Camera.retrieve(img);

  /// Main Loop
  ros::Rate loop_rate(looprate);
  while (ros::ok())
  {
    // Copy image to previous image
    img.copyTo(imgPrev);

    // Get image
    Camera.grab();
    Camera.retrieve(img);
    
    // Resize Image
    if (scale != 1)
      resize(img.clone(), img, Size(), scale,scale);

    // Check if moving
    bool moving = checkMoving(imgPrev, img);

    // Publish Image
    sensor_msgs::ImagePtr msgCam = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pubCam.publish(msgCam);

    // Publish bool moving
    std_msgs::Bool msgMov;
    msgMov.data = moving;
    pubMov.publish(msgMov);

    ros::spinOnce();
    loop_rate.sleep();
  }

  Camera.release();
  return 0;
}

bool checkMoving(Mat img1, Mat img2)
{
  // Convert to hsv Colors
  Mat img1HSV;
  Mat img2HSV;
  cvtColor(img1,img1HSV,COLOR_BGR2HSV);
  cvtColor(img2,img2HSV,COLOR_BGR2HSV);
  
  // Compare images
  Mat diff;
  absdiff(img1HSV,img2HSV,diff);
  
  // Check if the camera is moving
  double avgH;
  double avgS;
  avgH = mean(diff)[0];
  avgS = mean(diff)[1];
  int t = 15;  // threshold value
  bool mov; // 1->moving, 0->standing still
  if(avgH > t || avgS > t){
    mov = true;
  }
  else{
    mov = false;
  }

  return(mov);
}
