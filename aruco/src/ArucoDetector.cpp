#include "ArucoDetector.h"

namespace aruco_detect
{
  /// Constructor
  Detector::Detector(String xml)
  {

    //namedWindow(OPENCV_WINDOW);
	MarkerSize = 0.07; /// Size of the markers in meters
	MDet.setThresholdParams(7,7);
	MDet.setThresholdParamRange(2,0);
	MDet.setDictionary( "ARUCO" ,0.f); /// Choice of used dictionary
	CamParam.readFromXMLFile(xml); /// XML file with calibration info of the camera --> Got to be a better one
	
	out_X = 0.0;
	out_Y = 0.0;
	out_Rotation = 0.0;

	/// Down markers
	markerMap[100] = Mark(144.0,  96.0, DOWN);
	markerMap[101] = Mark(131.5,  96.0, DOWN);
	markerMap[102] = Mark(117.9,  96.0, DOWN);
	markerMap[103] = Mark(104.4,  96.0, DOWN);
	markerMap[104] = Mark(91.4,  96.0, DOWN);
	markerMap[105] = Mark(78.5,  96.0, DOWN);
	markerMap[106] = Mark(64.9,  96.0, DOWN);
	markerMap[107] = Mark(53.6,  96.0, DOWN);
	markerMap[108] = Mark(40.4,  96.0, DOWN);
	markerMap[109] = Mark(26.4,  96.0, DOWN);
	markerMap[110] = Mark(12.8,  96.0, DOWN);

	/// Left markers
	markerMap[111] = Mark(0.0,  78.7, LEFT);
	markerMap[112] = Mark(0.0,  58.5, LEFT);
	markerMap[113] = Mark(0.0,  39.2, LEFT);
	markerMap[114] = Mark(0.0,  21.4, LEFT);

	/// Up markers
	markerMap[115] = Mark(14.0, 0.0, UP);
	markerMap[116] = Mark(26.7, 0.0, UP);
	markerMap[117] = Mark(40.2, 0.0, UP);
	markerMap[118] = Mark(52.9, 0.0, UP);
	markerMap[119] = Mark(66.2, 0.0, UP);
	markerMap[120] = Mark(79.4, 0.0, UP);
	markerMap[121] = Mark(92.9, 0.0, UP);
	markerMap[122] = Mark(107.0, 0.0, UP);
	markerMap[123] = Mark(120.6, 0.0, UP);
	markerMap[124] = Mark(132.8, 0.0, UP);
	markerMap[125] = Mark(146.0, 0.0, UP);

	/// Right markers
	markerMap[126] = Mark(159.0,  20.5, RIGHT);
	markerMap[127] = Mark(159.0,  38.7, RIGHT);
	markerMap[128] = Mark(159.0,  57.6, RIGHT);
	markerMap[129] = Mark(159.0,  77.9, RIGHT);
	

  }

  Detector::~Detector()
  {
   // destroyWindow(OPENCV_WINDOW);
  }

  /// Public Search Method
  bool Detector::detectAruco(Mat img)
  {
	vector< Result >  Results; // Vector for calculated X, Y and rotation according to markers
	vector< Marker >  Markers; // Vector for found markers  
	  
    //printf("PROCESSING IMAGE...\n\n");
	MDet.detect(img, Markers, CamParam, MarkerSize); // Actual detection
	
	if(Markers.size() == 0) // Test e.g. wrong dictionary
	{
		return false;
	}
	
	#ifdef DEBUG
	
	/// Visualizing the detection

	for (unsigned int i = 0; i < Markers.size(); i++) 
	{
		Markers[i].draw(img, Scalar(0, 255, 255), 5); // Draw the borders of the markers
		CvDrawingUtils::draw3dAxis(img, Markers[i], CamParam); // Draw the axes of the markers
	}		
	
	#endif
	/// Calculating the position and rotation
	
	for (unsigned int i = 0; i < Markers.size(); i++) 
	{
		x_coord = Markers[i].Tvec.at<Vec3f>(0,0)[0]; // Extract the x coordinate from the translation vector
		z_coord = Markers[i].Tvec.at<Vec3f>(0,0)[2]; // Extract the z coordinate from the translation vector
		dist = 100*sqrt((x_coord*x_coord)+(z_coord*z_coord)); // Calculate the distance between robot and marker in cm
		
		Rodrigues(Markers[i].Rvec, R); // Conversion from rotation vector to rotation matrix
		roll = atan2(R.at<float>(1,0), R.at<float>(0,0)); 
		
		if(roll > 0)
		{
			yaw = atan2(R.at<float>(2,1), R.at<float>(2,2)); 
		}
		else
		{
			yaw = -atan2(R.at<float>(2,1), R.at<float>(2,2)); 
		}
		
				
		omega = atan2(x_coord,z_coord);
		
		if((yaw > 0 && x_coord > 0) || (yaw < 0 && x_coord < 0))
		{
			gamma = M_PI/2-(M_PI-abs(yaw)+abs(omega)); // Calculating the angle between robot and marker
		}
		else
		{
			gamma = M_PI/2-(M_PI-abs(yaw)-abs(omega));
		}
		
		//cout << Markers[i].id << ": x = " << x_coord << ", z = " << z_coord << ", dist = " << dist << ", yaw = " << DEGR*yaw << endl << endl;
		//cout << "\tYaw = " << DEGR*yaw << ", omega = " << DEGR*abs(omega) << ", gamma = " << DEGR*gamma << endl << endl;
		//cout << Markers[i].id << ": Yaw = " << DEGR*yaw << ", roll = " << DEGR*roll << ", pitch = " << DEGR*pitch << endl << endl;
		
		border_rel = abs(dist*cos(gamma));
		depth_rel = abs(dist*sin(gamma));
		
		Mark det_mark = markerMap[Markers[i].id];
		
		switch(det_mark.side)
		{
		case LEFT:
			 cur_out_Rotation = -yaw;
			 cur_out_X     = det_mark.X + depth_rel;
			 if(yaw > 0)
				cur_out_Y  = det_mark.Y + border_rel;
			 else
				cur_out_Y  = det_mark.Y - border_rel;
			 break;

		case UP:
			 if(yaw > 0)
			 {
				cur_out_Rotation = M_PI/2-yaw;
			 }
			 else
			 {	 
				cur_out_Rotation = yaw;
			 }
			 cur_out_Y     = det_mark.Y + depth_rel;
			 if(yaw > 0)
				cur_out_X  = det_mark.X - border_rel;
			 else
				cur_out_X  = det_mark.X + border_rel;
			 break;

		case RIGHT:
			 if(yaw > 0)
			 {
				cur_out_Rotation = M_PI-yaw;
			 }
			 else
			 {	 
				cur_out_Rotation = -yaw-M_PI;
			 }
			 cur_out_X     = det_mark.X - depth_rel;
			 if(yaw > 0)
				cur_out_Y  = det_mark.Y - border_rel;
			 else
				cur_out_Y  = det_mark.Y + border_rel;
			 break;

		case DOWN:
			 if(yaw > 0)
			 {
				cur_out_Rotation = 3*M_PI/2-yaw;
			 }
			 else
			 {	 
				cur_out_Rotation = -yaw-M_PI/2;
			 }
			 cur_out_Y     = det_mark.Y - depth_rel;
			 if(yaw > 0)
				cur_out_X  = det_mark.X + border_rel;
			 else
				cur_out_X  = det_mark.X - border_rel;
			 break;

		default: return false;
		}
		//cout << Markers[i].id << ": Depthrel = " << depth_rel << ", Borderrel = " << border_rel << endl << endl;
		
		//cout << Markers[i].id << ":Rotatie = " << DEGR*cur_out_Rotation << ", X = " << cur_out_X << ", Y = " << cur_out_Y << endl << endl;
		
		res.X = cur_out_X;
		res.Y = cur_out_Y;
		res.rot = DEGR*cur_out_Rotation;
		res.id = Markers[i].id;
		
		Results.push_back(res);
	}
	
	/// Printing the results
	#ifdef DEBUG
	for(unsigned int i = 0; i < Results.size(); i++)
	{
		cout << Results[i].id << ": X = " << Results[i].X << ", Y = " << Results[i].Y << ", Rotation = " << Results[i].rot << endl << endl;	
	}
	#endif
	/// Outlier Rejection based on the rotation

	outliers = UNKNOWN;
	max_diff = 0;
	sum = 0;
		
	if(Results.size()>2)
	{
		//cout << "-- OUTLIER REJECTION --" << endl << endl;
		
		median = calc_median(Results, 1);

		while(outliers > 0)
		{
			outliers = 0;
			sum = 0;
			for(unsigned int i = 0; i < Results.size(); i++)
			{
				sum +=  Results[i].rot;
				for(unsigned int j = i+1; j < Results.size(); j++)
				{
					diff = abs(Results[i].rot - Results[j].rot);
					if(diff > THRES_ROT)
					{
						outliers++;
						if(diff > max_diff)
						{
							max_diff = diff;	
							res1 = i;
							res2 = j;						
						}
					}
				}
				
			}
			
			mean = sum/Results.size();
			
			if(outliers > 0)
			{
				if(abs(Results[res1].rot-median)>abs(Results[res2].rot-median))
				{
					Results.erase(Results.begin()+res1);		
				}
				else
				{
					Results.erase(Results.begin()+res2);
				}
			
			
				//cout << "median rotation = " << median << endl;
				//cout << "mean rotation = " << mean << endl << endl;
			 
				for(unsigned int i = 0; i < Results.size(); i++)
				{
					//cout << Results[i].id << ": X = " << Results[i].X << ", Y = " << Results[i].Y << ", Rotation = " << Results[i].rot << endl << endl;	
				}
			}
			
		}
		
		//cout << "Final mean rotation = " << mean << endl << endl;
		
		out_Rotation = mean;
		
		/// Outlier Rejection based on X-coordinate
		
		outliers = UNKNOWN;
		median = calc_median(Results, 2);
		while(outliers > 0)
		{
			outliers = 0;
			sum = 0;
			for(unsigned int i = 0; i < Results.size(); i++)
			{
				sum +=  Results[i].X;
				for(unsigned int j = i+1; j < Results.size(); j++)
				{
					diff = abs(Results[i].X - Results[j].X);
					if(diff > THRES_X)
					{
						outliers++;
						if(diff > max_diff)
						{
							max_diff = diff;	
							res1 = i;
							res2 = j;						
						}
					}
				}
				
			}
			
			mean = sum/Results.size();
			
			if(outliers > 0)
			{
				if(abs(Results[res1].X-median)>abs(Results[res2].X-median))
				{
					Results.erase(Results.begin()+res1);		
				}
				else
				{
					Results.erase(Results.begin()+res2);
				}

			
				//cout << "median X = " << median << endl;
				//cout << "mean X = " << mean << endl;
				 
				/*for(unsigned int i = 0; i < Results.size(); i++)
				{
					cout << Results[i].id << ": X = " << Results[i].X << ", Y = " << Results[i].Y << ", Rotation = " << Results[i].rot << endl << endl;	
				}*/
			}
			
		}
		
		//cout << "Final mean X = " << mean << endl << endl;
		
		out_X = mean;
		
		/// Outlier Rejection based on Y-coordinate
		
		outliers = UNKNOWN;
		median = calc_median(Results, 3);
		while(outliers > 0)
		{
			outliers = 0;
			sum = 0;
			for(unsigned int i = 0; i < Results.size(); i++)
			{
				sum +=  Results[i].Y;
				for(unsigned int j = i+1; j < Results.size(); j++)
				{
					diff = abs(Results[i].Y - Results[j].Y);
					if(diff > THRES_X)
					{
						outliers++;
						if(diff > max_diff)
						{
							max_diff = diff;	
							res1 = i;
							res2 = j;						
						}
					}
				}
				
			}
			
			mean = sum/Results.size();
			
			if(outliers > 0)
			{
				if(abs(Results[res1].Y-median)>abs(Results[res2].Y-median))
				{
					Results.erase(Results.begin()+res1);		
				}
				else
				{
					Results.erase(Results.begin()+res2);
				}
			
			
				//cout << "median Y = " << median << endl;
				//cout << "mean Y = " << mean << endl;
				 /*
				for(unsigned int i = 0; i < Results.size(); i++)
				{
					cout << Results[i].id << ": X = " << Results[i].X << ", Y = " << Results[i].Y << ", Rotation = " << Results[i].rot << endl << endl;	
				}*/
			}
			
		}
		
		//cout << "Final mean Y = " << mean << endl << endl;
		
		out_Y = mean;
	}
	else
	{
		X_sum = 0;
		Y_sum = 0;
		rot_sum = 0;
		for(unsigned int i = 0; i < Results.size(); i++)
			{
				X_sum += Results[i].X;
				Y_sum += Results[i].Y;
				rot_sum += Results[i].rot;
			}
			
		out_X = X_sum/Results.size();
		out_Y = Y_sum/Results.size();
		out_Rotation = rot_sum/Results.size();
	}
	
	//cout << "Detector: X = " << out_X << ", Y = " << out_Y << ", rotation = " << out_Rotation << endl << endl;
	
	//cout << "/////////////////////////////////////////////////////////" << endl << endl;
	resize(img,img,Size(),0.3,0.3);
	//imshow("Detectie", img);

    return true;
  }
  
  float Detector::calc_median(vector< Result > R, int type)
  {
		vector<float> dummy;
		
		switch(type)
		{
		case 1:
			for(unsigned int i = 0; i < R.size(); i++)
			{	
				dummy.push_back(R[i].rot);
			}
			 
			 break;

		case 2:
			for(unsigned int i = 0; i < R.size(); i++)
			{
				dummy.push_back(R[i].X);
			}
			 break;

		case 3:
			for(unsigned int i = 0; i < R.size(); i++)
			{
				dummy.push_back(R[i].Y);
			}
			 
			 break;

		default: return 0.0;
		}
		
		/*for(unsigned int i = 0; i < dummy.size(); i++)
			{
				cout << "Waarde" << i+1 << " = " << dummy[i] << "\t";
			}*/
			
		//cout << endl << endl;
		
		sort(dummy.begin(),dummy.end());
		
		int index;
		if(dummy.size() % 2 == 0)
		{
				index = dummy.size()/2;
				return (dummy[index]+dummy[index-1])/2;
		}
		else
		{
				index = dummy.size()/2;
				return dummy[index];
		}
  }
}
