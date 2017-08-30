#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "aruco/Position.h"
#include "ArucoDetector.h"

static const std::string CAMERA_TOPIC = "/camera/image";

using namespace std;
namespace aruco_detect
{
	class ArucoNode
	{
		ros::NodeHandle nh;
		image_transport::ImageTransport it_;
		image_transport::Subscriber img_sub_;
		ros::Publisher res_pub;

		Detector det;

		public:
		/// Constructor
		ArucoNode(String xml) : it_(nh), det(xml)
		{

		/// Create topic to publish results
		res_pub = nh.advertise<aruco::Position>("position", 1);

		/// Subscribe to the camera topic
		img_sub_ = it_.subscribe(CAMERA_TOPIC, 1, &ArucoNode::imageCb, this);
		//ROS_INFO_STREAM("Subscribed to camera: " << CAMERA_TOPIC);

		}

		/// Process Image Topic Callback
		void imageCb(const sensor_msgs::ImageConstPtr &msg)
		{
		cv_bridge::CvImagePtr cv_ptr;
		aruco::Position resMsg;

		// Convert to OpenCV image
		try
		{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
		}

		// OpenCV calculations
		if (!det.detectAruco(cv_ptr->image))
		{
			ROS_DEBUG_STREAM("Position detection failed");
        		resMsg.X_coord = -1;
			resMsg.Y_coord = -1;
			resMsg.Robot_Rotation = -666;
		}
		else
		{
			resMsg.X_coord = det.out_X;
			resMsg.Y_coord = det.out_Y;
			resMsg.Robot_Rotation = det.out_Rotation;
		}

		// Publish message
		
		//cout << "Message: X = " << resMsg.X_coord << ", Y = " << resMsg.Y_coord << ", rotation = " << resMsg.Robot_Rotation << endl << endl;
		//cout << "/////////////////////////////////////////////////////////" << endl << endl;
		res_pub.publish(resMsg);
		}
	};
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arucodetector");
  if(argc >= 2)
  {
  	aruco_detect::ArucoNode an(argv[1]);
  }
else
{
	aruco_detect::ArucoNode an("/home/nicholas/Desktop/Docu_SEE/Callib/new_cam.yml");
}
  ros::spin();
}
