#ifndef ARUCODETECTOR_H
#define ARUCODETECTOR_H

#define DEBUG

/// Sides of the board
#define LEFT 0
#define UP  1
#define RIGHT 2
#define DOWN  3

#define UNKNOWN 999
#define DEGR 180/M_PI

/// Dimensions of the board
#define WIDTH 96
#define HEIGHT 159
#define UNKNOWN 999
#define THRES_ROT 3.0
#define THRES_X 3.0
#define THRES_Y 3.0

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <math.h>

using namespace std;
using namespace cv;
using namespace aruco;

namespace aruco_detect
{
  class Detector
  {
    public:

	/// Public variables
	float out_X; // Horizontal position of the robot
	float out_Y; // Vertical position of the robot
	float out_Rotation; // Rotation of the robot, relative to the axes
        int valuePoint;
		
	/// Public Methods
	Detector(String xml);
	~Detector();
	std::vector<bool> detectAruco(cv::Mat img);
	
	
    private:
	
	/// Marker Structure
	struct Mark {
		float X; // X-coordinate
		float Y; // Y-coordinate
		int side; // Side the markers is attached to (UP, DOWN, LEFT, RIGHT)	

		Mark(): X(UNKNOWN), Y(UNKNOWN), side(UNKNOWN) {}
		Mark(float x, float y, int s): X(x), Y(y), side(s) {}		
	};
	
	std::map<int, Mark> markerMap; // Overview of the markers of the board with their coordinates
	
	/// Result Structure
	struct Result {
		int id; // Marker ID
		float X; // X-coordinate of the robot according to the marker
		float Y; // Y-coordinate of the robot according to the marker
		float rot; // Rotation of the robot according to the marker

		Result(): id (UNKNOWN), X(UNKNOWN), Y(UNKNOWN), rot(UNKNOWN) {}
		Result(float x, float y, float r, int id): X(x), Y(y), rot(r), id(id) {}		
	};
	
	/// Private variables
	aruco::MarkerDetector MDet; // Instance of the MarkerDetector class
	aruco::CameraParameters CamParam; // Parameter for camera calibration
	float MarkerSize; // Size of the markers to detect	
	
	float x_coord, z_coord, dist, depth_rel, border_rel; // Variables used for the position calculation
	
	// Variables used for the rotation calculation
	Mat R;
	float yaw, pitch, roll;
	float gamma, omega;
	
	float cur_out_X, cur_out_Y, cur_out_Rotation;
	Result res;
	
	// Variables used for outlier rejection and end result calculation (averaging)
	int outliers;
	float diff;
	float max_diff;
	float res1, res2;
	float sum;
	float median, mean;
	
	float X_sum;
	float Y_sum;
	float rot_sum;
	
	
	/// Private methods
	float calc_median(vector<Result> R, int type); // Used to calculate median
      
  };
}

#endif
