/*

ROS_NAMESPACE=/camera/stereo_camera_LR rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True _prefilter_size:=9 _prefilter_cap:=31 _correlation_window_size:=51 _min_disparity:=0 _disparity_range:=128 _uniqueness_ratio:=4 _texture_threshold:=10 _speckle_size:=300 _speckle_range:=15
rosrun image_view stereo_view stereo:=camera/stereo_camera_LR  image:=image_rect_color

rosbag play -s 23 -u 12 -r 0.1 exp_grad_track5.bag 

roslaunch bumblebee_xb3_gdb.launch
*/


#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include < opencv2/video/background_segm.hpp> 

#include <string>

#include <cv_bridge/cv_bridge.h>
#include "stereo_msgs/DisparityImage.h"
#include <boost/format.hpp>

#include <stdio.h>
#include <iomanip>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <tf/transform_broadcaster.h>

#include <cv.h>
#include <highgui.h>

#include <iostream>

#include "boost/multi_array.hpp"
#include <boost/thread/thread.hpp>

#define THRESHOLD 8
#define VERTICAL_MERGE_BOUNDARY 20

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//ovaj
//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

pcl::PointCloud< pcl::PointXYZ > point_cloud_C;
int argcc;
char **argvv;





//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

RNG rng(12345);



enum Direction{
    ShiftUp=1, ShiftRight, ShiftDown, ShiftLeft
   };

cv::Mat shiftFrame(cv::Mat frame, int pixels, Direction direction)
{
    //create a same sized temporary Mat with all the pixels flagged as invalid (-1)
    cv::Mat temp = cv::Mat::zeros(frame.size(), frame.type());

    switch (direction)
    {
    case(ShiftUp) :
        frame(cv::Rect(0, pixels, frame.cols, frame.rows - pixels)).copyTo(temp(cv::Rect(0, 0, temp.cols, temp.rows - pixels)));
        break;
    case(ShiftRight) :
        frame(cv::Rect(0, 0, frame.cols - pixels, frame.rows)).copyTo(temp(cv::Rect(pixels, 0, frame.cols - pixels, frame.rows)));
        break;
    case(ShiftDown) :
        frame(cv::Rect(0, 0, frame.cols, frame.rows - pixels)).copyTo(temp(cv::Rect(0, pixels, frame.cols, frame.rows - pixels)));
        break;
    case(ShiftLeft) :
        frame(cv::Rect(pixels, 0, frame.cols - pixels, frame.rows)).copyTo(temp(cv::Rect(0, 0, frame.cols - pixels, frame.rows)));
        break;
    default:
        std::cout << "Shift direction is not set properly" << std::endl;
    }

    return temp;
}



//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer
boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}



void callback(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
  	//std::cout<<"in function pointC!"<< std::endl;
	
	sensor_msgs::PointCloud2 cloudmsg = *point_cloud;
	
	pcl::fromROSMsg(cloudmsg, point_cloud_C);
	
}




void disparityImageCallback(const stereo_msgs::DisparityImageConstPtr& disparity_msg, const sensor_msgs::ImageConstPtr& left_msg,
							const sensor_msgs::ImageConstPtr& right_msg
							//, const sensor_msgs::PointCloud2ConstPtr& point_cloud
						   
						   )
{
	
	/*
	cv::Mat camera_matrix1 = (Mat_<float>(3,3) <<1033.842144566857, 0, 641.7568472235299, 0, 1033.9910623563098, 477.77802466524605, 0, 0, 1);
	cv::Mat camera_matrix2 = (Mat_<float>(3,3) <<1036.1905066339948, 0, 658.0591592476255, 0, 1036.2230336709486, 467.8432113246438, 0, 0, 1);
	cv::Mat dist_coeffs1 = (Mat_<float>(1,5) <<-0.34760090909684316, 0.13893127178322365, 0.0002624157294778045, 0.00017474106031351618, 0);
	cv::Mat dist_coeffs2 = (Mat_<double>(1,5) <<-0.34299615135008976, 0.13066865403875236, 0.0005705026352960487, -0.0002575017480093701, 0);
	cv::Mat R = (Mat_<double>(3,3) <<0.9999985657944156, 0.0008153120094810814, -0.0014844781692219457, -0.0008129287740475245, 0.999998381025527, 0.0016053327159822335,0.0014857846129322403, -0.0016041236385868848, 0.999997609612859);
	cv::Mat T = (Mat_<double>(3,1) <<-0.2394236026388935, 0.00027744418645254785, 7.347465192346994e-05);
	*/
	
	/*
	cv::Mat_<double> cameraMatrix1(3,3); // 3x3 matrix
	cv::Mat_<double> distCoeffs1(5,1);   // 5x1 matrix for five distortion coefficients
	cv::Mat_<double> cameraMatrix2(3,3); // 3x3 matrix
	cv::Mat_<double> distCoeffs2(5,1);   // 5x1 matrix
	cv::Mat_<double> R(3,3);             // 3x3 matrix, rotation left to right camera
	cv::Mat_<double> T(3,1);             // * 3 * x1 matrix, translation left to right proj. center
	
	cameraMatrix1 << 1033.842144566857, 0, 641.7568472235299, 0, 1033.9910623563098, 477.77802466524605, 0, 0, 1;
	cameraMatrix2 << 1036.1905066339948, 0, 658.0591592476255, 0, 1036.2230336709486, 467.8432113246438, 0, 0, 1;
	distCoeffs1   << -0.34760090909684316, 0.13893127178322365, 0.0002624157294778045, 0.00017474106031351618, 0;
	distCoeffs2   << -0.34299615135008976, 0.13066865403875236, 0.0005705026352960487, -0.0002575017480093701, 0;
	R << 0.9999985657944156, 0.0008153120094810814, -0.0014844781692219457,
	-0.0008129287740475245, 0.999998381025527, 0.0016053327159822335,0.0014857846129322403, -0.0016041236385868848, 0.999997609612859;
	T << -0.2394236026388935, 0.00027744418645254785, 7.347465192346994e-05;

	cv::Mat R1,R2,P1,P2,Q;
	
	cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, cv::Size(1280, 960), R, T, R1, R2, P1, P2, Q);	
	
	std::cout<< "Q" << Q << std::endl;
	std::cout<< "R" << R << std::endl;
	std::cout<< "T" << T << std::endl;
	std::cout<< "R1" << R1 << std::endl;
	std::cout<< "R2" << R2 << std::endl;
	std::cout<< "P1" << P1 << std::endl;
	std::cout<< "P2" << P2 << std::endl;
	*/
	
	//sensor_msgs::PointCloud2> pc_sub(nh, "/socket_node/can1_TrackPC"
	/*
	std::cout<<"in function!"<< std::endl;
	ros::init(argcc, argvv, "sub_pcl");
	ros::NodeHandle nh2;
	ros::Subscriber sub = nh2.subscribe<sensor_msgs::PointCloud2>("socket_node/can1_TrackPC", 1, callback);
	ros::spinOnce();
	std::cout<<"in function out!"<< std::endl;
	*/
	
	/*
	std::cout<<"in function!"<< std::endl;
	
	
	sensor_msgs::PointCloud2 cloudmsg = *point_cloud;
	
	
	pcl::fromROSMsg(cloudmsg, point_cloud_C);
	
	
	int cloudsize = (point_cloud_C.width) * (point_cloud_C.height);
	for (int i=0; i< cloudsize; i++){
		std::cout << "(x,y,z) = " << point_cloud_C.points[i] << std::endl;
	}
	*/
	
	
	
	
	std::cout << std::setprecision(2);
	
	float focalLenPx = 1036.1905066339948;
	float baselineM = 0.2394236026388935;
	
	cv::Mat_<double> Q(4,4);
	Q<<	1, 0, 0, -653.0797576904297,
  		0, 1, 0, -472.968132019043,
  		0, 0, 0, 818.8397037797508,
  		0, 0, 4.176694655336997, -0;
	
	double Q03, Q13, Q23, Q32, Q33;
 	Q03 = Q.at<double>(0,3);
 	Q13 = Q.at<double>(1,3);
 	Q23 = Q.at<double>(2,3);
  	Q32 = Q.at<double>(3,2);
 	Q33 = Q.at<double>(3,3);
	
	
	int CONTUR_LENGTH_THRESHOLD = 60;
	
	ros::NodeHandle nh;
	ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2> ("FromDisparityToWorld", 10);
	sensor_msgs::PointCloud2 output;
	
	PointCloud::Ptr msg (new PointCloud);
	msg->height = msg->width = 1;
	msg->header.frame_id = "3D";
	int g=0;
	cv_bridge::CvImagePtr last_image_;
	cv_bridge::CvImagePtr last_image_R;
	
	last_image_ = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::MONO8);
	last_image_R = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::MONO8);
	
	
	//cv::Mat right = shiftFrame(last_image_R->image, 0, ShiftRight);
	//cv::Mat left = shiftFrame(last_image_->image, 0, ShiftLeft);
		
	
	//cv::imshow("imageR", right);
	//cv::imshow("imageL", left);
	
	
	cv::Mat edge, edge2;
	
	const clock_t begin_time = clock();
	std::cout << "Timestart! " <<float( clock ())/  CLOCKS_PER_SEC<< "  ---";
	
	
	//bilateralFilter(last_image_->image, edge2,-1, 50, 7);
	
	
	//Canny( last_image_->image, edge, 60, 120, 3);
	//Canny( edge2, edge, 60, 120, 3);

	
	/*
	edge.convertTo(draw, CV_8U);
	cv::imshow("CannyEdge", draw);
	
	std::cout << float( clock () - begin_time ) / (double) CLOCKS_PER_SEC<<"    ";
	
	
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	findContours( edge, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); //CV_RETR_EXTERNAL  CV_RETR_TREE
	
	
	
	//Remove small conturs
	
	/*
	to slow, have to work way around this
	for (vector<vector<Point> >::iterator it = contours.begin(); it!=contours.end(); )
	{
   		if (it->size()<CONTUR_LENGTH_THRESHOLD)
        	it=contours.erase(it);
    	else
        	++it;
	}*/
	
	
	/* also to slow
	vector<Moments> ContArea(contours.size());
	for( int i = 0; i < contours.size(); i++ )
    {  
        ContArea[i] = moments(contours[i], false);
    }
	
	int iterMoment=0;
	for (vector<vector<Point> >::iterator it = contours.begin(); it!=contours.end(); )
	{
   		if (ContArea[iterMoment].m00 < 100)
        	it=contours.erase(it);
    	else
        	++it;
		
		iterMoment++;
	}
	*/
	
	
	
	/*
	  /// Approximate contours to polygons + get bounding rects and circles
	  /// finding conturs takes too much time, not god for rest of processing
	  
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
     { 
	  approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
	  
      boundRect[i] = boundingRect( Mat(contours_poly[i]) );
	  if(boundRect[i].width * boundRect[i].height < 500 || 
		boundRect[i].width*3<boundRect[i].height ||
		boundRect[i].width>boundRect[i].height*3){
			contours.erase(contours.begin()+i);
			boundRect.erase(boundRect.begin()+i);
		  i--;
	  }
	  
       //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
     }
	


	/// Draw contours
	Mat drawing = Mat::zeros( edge.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );

	}

	/// Show in a window

	imshow( "Contours", drawing );
	*/
	
	
		/*
	int iterMoment=0;
	for (vector<vector<Point> >::iterator it = contours.begin(); it!=contours.end(); )
	{
   		if (boundRect[iterMoment].size() < 100)
        	it=contours.erase(it);
    	else
        	++it;
		
		iterMoment++;
	}
	*/
	
	
	
	
	//Disparity calculation and parametars
	
	float min_disparity = disparity_msg->min_disparity;
	float max_disparity = disparity_msg->max_disparity;
	float multiplier = 255.0f / (max_disparity - min_disparity);
	
	//cv::Mat_<cv::Vec3b> disparity_color_;
	//disparity_color_.create(disparity_msg->image.height, disparity_msg->image.width);
	
	//constructs a matrix on top of user-allocated data. step is in bytes(!!!), regardless of the type 
	const cv::Mat_<float> dmat(disparity_msg->image.height, disparity_msg->image.width, (float*)&disparity_msg->image.data[0], disparity_msg->image.step);
	
	cv::Mat dispar(disparity_msg->image.height, disparity_msg->image.width,CV_8U);
	
	for (int row = 0; row < dispar.rows; ++row) {
		const float* d = dmat[row];
		for (int col = 0; col < dispar.cols; ++col) {
				
			if(d[col]==-1){
					dispar.at<char>(row, col, CV_8U) = 0;
					continue;
				}
				
				int index = (d[col] - min_disparity) * multiplier + 0.5;
				index = std::min(255, std::max(0, index));
		

				dispar.at<char>(row, col, CV_8U) = index;
			}
		}
	
	

	float pz=100;
	float px=100, py=100; 
	uchar pr, pg, pb;
	
	
	cv::Mat image_roi;
	cv::Rect rect_roi;
	
	cv::Mat depthMap (dispar.rows, dispar.cols, CV_32F);
	cv::Mat heightMap (dispar.rows, dispar.cols, CV_32F);
	cv::Mat widthMap (dispar.rows, dispar.cols, CV_32F);
	
	
	cv::Mat merge;
	
	
	
	//SWITCH IMAGE HERE!!!
	
	//GaussianBlur(last_image_->image, merge, cv::Size(3,3), -1, -1, 0);
	Canny( last_image_->image, edge, 40, 140, 3);
	cv::imshow("CannyEdge", edge);
	
	
	for (int i = 0; i < dispar.rows; ++i)
	{


		for (int j = 0; j < dispar.cols; ++j)
		{
		
		
			if ( dispar.at<uchar>(i,j) <=0 ){ 
				
				depthMap.at<float>(i, j) =  0;
				heightMap.at<float>(i, j) = 0;
				widthMap.at<float>(i, j) =  0;
				
				continue;
			} //Discard bad pixels
			else {
				pz = focalLenPx * baselineM/( dispar.at<uchar>(i,j));

				py = pz * (i-dispar.rows/2) / focalLenPx;

				px = pz * (j-dispar.cols/2) / focalLenPx;

				//std::min(255, std::max(0, index));



				if(	abs (pz) > 100 || 
					abs (py) > 2 || 
					abs(px) > 10){

					edge.at<uchar>(i, j) = 0; //edge.at<uchar>(i, j, CV_8U) = 0;
					dispar.at<uchar>(i, j) = 0;

				}
				else{
					depthMap.at<float>(i, j) =  floorf(pz * 100) / 100;
					heightMap.at<float>(i, j) = floorf(py * 100) / 100;
					widthMap.at<float>(i, j) =  floorf(px * 100) / 100;
				}
		}
			
		}
		//std::cout<<std::endl;
		//cv::imshow("CannyEdge", edge);
	}
	
	
	//GaussianBlur(dispar, dispar, cv::Size(5,5), -1, -1, 0);
	Canny( dispar, edge2, 60, 120, 3);
	cv::imshow("DisparityEdge", edge2);
	

	
	for (int row = 0; row < edge.rows-1; ++row) {
		edge.at<uchar>(row, 1) =  0;
		edge.at<uchar>(row, 0) =  0;
		edge.at<uchar>(row, edge.cols-1) =  0;
		edge.at<uchar>(row, edge.cols-2) =  0;

	}
	

	
	for (int col = 0; col < edge.cols-1; ++col) {
		edge.at<uchar>(1, col) =  0;
		edge.at<uchar>(0, col) =  0;

		edge.at<uchar>(edge.rows-1, col) =  0;
		edge.at<uchar>(edge.rows-2, col) =  0;
	}
	
	for (int row = 0; row < (edge.rows-1)/4; ++row) {
		for (int col = 0; col < edge.cols-1; ++col) {
			edge.at<uchar>(row, col) =  0;
		}

	}

	
	cv::imshow("CannyEdge", edge);
	
	int inI, inJ, lastRow, lastCol, prevR=-1, prevC=-1;
	
	for (int row = 0; row < edge.rows; ++row) {
		
		for (int col = 0; col < edge.cols; ++col) {
			
			//std::cout<<"cords: " << row <<"  " << col << std::endl;
			
			if(edge.at<uchar>(row, col, CV_8U) ==  255){
				
				int len=0;
				int loops=2;
				
				while(loops>0){
					
					inI = lastRow = row;
					inJ = lastCol = col;
					
					loops--;
					while(true){

//cv::imshow("image", edge);

						edge.at<uchar>(inI, inJ, CV_8U) =  254;

						if(edge.at<uchar>(inI, inJ+1, CV_8U) ==  255){
							inJ++;
							len++;
						}

						else if(edge.at<uchar>(inI-1, inJ+1, CV_8U) ==  255){
							inI--;
							inJ++;
							len++;
						}

						else if(edge.at<uchar>(inI-1, inJ, CV_8U) ==  255){
							inI--;
							len++;
						}

						else if(edge.at<uchar>(inI-1, inJ-1, CV_8U) ==  255){
							inI--;
							inJ--;
							len++;
						}

						else if(edge.at<uchar>(inI, inJ-1, CV_8U) ==  255){
							inJ--;
							len++;
						}

						else if(edge.at<uchar>(inI+1, inJ-1, CV_8U) ==  255){
							inI++;
							inJ--;
							len++;
						}

						else if(edge.at<uchar>(inI+1, inJ, CV_8U) ==  255){
							inI++;
							len++;
						}

						else if(edge.at<uchar>(inI+1, inJ+1, CV_8U) ==  255){
							inI++;
							inJ++;
							len++;
						}

						else{
							break;
						}

						prevR=inI;
						prevC=inJ;

					}
				}
				
//std::cout<< len <<" " << std::endl;
				loops=2;
				if(len < 60 && len!=-1){
					
					while(loops>0){
						
						inI= row;
						inJ= col;
						len++;
						loops--;
						int memo=0;
						cv::Mat mem;	
						
						while(len>0){

//cv::imshow("image", edge);
	//std::cout.flush();
	//usleep(1);

							len--;
							int count = 0;
							int cht = 0;
								
						
							if(inJ!=0 && inI!=0 && inI!=dispar.rows-2  &&  inI!=dispar.rows-1   
							   && inJ!=dispar.cols-2   && inJ!=dispar.cols-1){ 
								
								rect_roi = cv::Rect(inJ-1, inI-1, 3, 3);
								edge.at<uchar>(inI, inJ) = 0;
								}
							else rect_roi = cv::Rect(0, 0, 3, 3);
							
							image_roi = edge(rect_roi);

							if(image_roi.at<uchar>(0, 0) == 255) count++;
							else if (count==0 && image_roi.at<uchar>(0, 1) == 255) count++;
							else if (count==0 && image_roi.at<uchar>(0, 2) == 255) count++;
							else if (count==0 && image_roi.at<uchar>(1, 0) == 255) count++;
							else if (count==0 && image_roi.at<uchar>(1, 2) == 255) count++;
							else if (count==0 && image_roi.at<uchar>(2, 0) == 255) count++;
							else if (count==0 && image_roi.at<uchar>(2, 1) == 255) count++;
							else if (count==0 && image_roi.at<uchar>(2, 2) == 255) count++;
								
								
							image_roi.at<uchar>(1, 1) = 0;	
							

							if(count>0 && memo==0){
								memo=1;
								mem=image_roi;
							}
	


							if(edge.at<uchar>(inI, inJ+1, CV_8U) ==  254){
								inJ++;
							}

							else if(edge.at<uchar>(inI-1, inJ+1, CV_8U) ==  254){
								inI--;
								inJ++;
							}

							else if(edge.at<uchar>(inI-1, inJ, CV_8U) ==  254){
								inI--;
							}

							else if(edge.at<uchar>(inI-1, inJ-1, CV_8U) ==  254){
								inI--;
								inJ--;
							}

							else if(edge.at<uchar>(inI, inJ-1, CV_8U) ==  254){
								inJ--;
							}

							else if(edge.at<uchar>(inI+1, inJ-1, CV_8U) ==  254){
								inI++;
								inJ--;
							}

							else if(edge.at<uchar>(inI+1, inJ, CV_8U) ==  254){
								inI++;
							}

							else if(edge.at<uchar>(inI+1, inJ+1, CV_8U) ==  254){
								inI++;
								inJ++;
							}

							if(memo==1){
								memo++;
							}
							else if(memo==2){
								memo++;
							}
							else if(memo==3){
								memo=0;
								mem.at<uchar>(1, 1) = 255;
							}
							
							if(edge.at<uchar>(inI, inJ)==0){
								if (memo>0){
									mem.at<uchar>(1, 1) = 255;
								}
								break;
							}

						}

					}

				}
				
				else if(len!=-1){
					
					//ELSE
				
					int count_dispar = 0;
					int avg_dispar = 0;
					
					int dispar_counter[256];
					
					for(int i=0; i<256; i++){
						dispar_counter[i] = 0;
					}
					
					int save_len=len;
					
					loops=2;
					
					
					//WHILE
					while(loops>0){
						
						inI= row;
						inJ= col;
						len++;
						loops--;
	
						
						while(len>0 ){
							
							dispar_counter[dispar.at<uchar>(inI, inJ)]++;
							
							if(dispar.at<uchar>(inI, inJ) != 0){
								count_dispar++;
								avg_dispar+=dispar.at<uchar>(inI, inJ);
							}

							len--;
							
							edge.at<uchar>(inI, inJ) = 253;
	
							if(edge.at<uchar>(inI, inJ+1, CV_8U) ==  254){
								inJ++;
							}

							else if(edge.at<uchar>(inI-1, inJ+1, CV_8U) ==  254){
								inI--;
								inJ++;
							}

							else if(edge.at<uchar>(inI-1, inJ, CV_8U) ==  254){
								inI--;
							}

							else if(edge.at<uchar>(inI-1, inJ-1, CV_8U) ==  254){
								inI--;
								inJ--;
							}

							else if(edge.at<uchar>(inI, inJ-1, CV_8U) ==  254){
								inJ--;
							}

							else if(edge.at<uchar>(inI+1, inJ-1, CV_8U) ==  254){
								inI++;
								inJ--;
							}

							else if(edge.at<uchar>(inI+1, inJ, CV_8U) ==  254){
								inI++;
							}

							else if(edge.at<uchar>(inI+1, inJ+1, CV_8U) ==  254){
								inI++;
								inJ++;
							}

							
							if(edge.at<uchar>(inI, inJ)==253){
								break;
							}

						}

					} //WHILE CLOSED
					
					
					loops=2;
					len=save_len;
					
//std::cout << "avg_dispar: "<< avg_dispar << "    count_dispar: "<<  count_dispar;
					
					if(count_dispar==0){ 
//std::cout<<endl;
						break;
					}
					
					
					int max;
					int index;
					max=0;
					for(int i=1; i<256; i++){
						if(dispar_counter[i] > max){
							max = (int) dispar_counter[i];
							index = i;
						}
					
					}
					
					avg_dispar = avg_dispar/count_dispar;
					if(avg_dispar % 2 !=0) avg_dispar++;
					
					if(index % 2 != 0) index++;
					
					avg_dispar = std::min(254, index);
					
//std::cout<< "   avg_dispar_dived: "<< avg_dispar << std::endl;
					
					//WHILE
					while(loops>0){
						
						inI= row;
						inJ= col;
						len++;
						loops--;
	
						
						while(len>0 ){
							
							len--;
							
							edge.at<uchar>(inI, inJ) = avg_dispar ;
	
							if(edge.at<uchar>(inI, inJ+1, CV_8U) ==  253){
								inJ++;
							}

							else if(edge.at<uchar>(inI-1, inJ+1, CV_8U) ==  253){
								inI--;
								inJ++;
							}

							else if(edge.at<uchar>(inI-1, inJ, CV_8U) ==  253){
								inI--;
							}

							else if(edge.at<uchar>(inI-1, inJ-1, CV_8U) ==  253){
								inI--;
								inJ--;
							}

							else if(edge.at<uchar>(inI, inJ-1, CV_8U) ==  253){
								inJ--;
							}

							else if(edge.at<uchar>(inI+1, inJ-1, CV_8U) ==  253){
								inI++;
								inJ--;
							}

							else if(edge.at<uchar>(inI+1, inJ, CV_8U) ==  253){
								inI++;
							}

							else if(edge.at<uchar>(inI+1, inJ+1, CV_8U) ==  253){
								inI++;
								inJ++;
							}

							
							if(edge.at<uchar>(inI, inJ)==avg_dispar){
								break;
							}

						}

					} //WHILE CLOSED
					
					
					
					
					
					//ELSE CLOSE
				
				}
				
			
			}
			
		}
	}
	
	cv::Mat save_edge = edge.clone();
	cv::imshow("imageL", save_edge);
	
	for(int row=0; row<edge.rows; ++row){
	
		for(int col=0; col<edge.cols; ++ col){
			
			int disp;
			
			if(edge.at<uchar>(row, col) != 0){
				
				disp = edge.at<uchar>(row, col) ;
				
				int len=0;
				int loops=2;
				
				int leftI, leftJ;
				int rightI, rightJ;
				
				int line_dispar = edge.at<uchar>(row, col);
				
				leftI = rightI = row;
				leftJ = rightJ = col;
				
				while(loops>0){
					
					inI =  row;
					inJ =  col;
					
					loops--;
					while(true){

						if(inJ < leftJ){
							leftJ = inJ;
							leftI = inI;
						}
						
						if(inJ > rightJ){
							rightJ = inJ;
							rightI = inI;
						}

						edge.at<uchar>(inI, inJ, CV_8U) =  0;

						if(edge.at<uchar>(inI, inJ+1, CV_8U) !=  0){
							inJ++;
							len++;
						}

						else if(edge.at<uchar>(inI-1, inJ+1, CV_8U) !=  0){
							inI--;
							inJ++;
							len++;
						}

						else if(edge.at<uchar>(inI-1, inJ, CV_8U) !=  0){
							inI--;
							len++;
						}

						else if(edge.at<uchar>(inI-1, inJ-1, CV_8U) !=  0){
							inI--;
							inJ--;
							len++;
						}

						else if(edge.at<uchar>(inI, inJ-1, CV_8U) !=  0){
							inJ--;
							len++;
						}

						else if(edge.at<uchar>(inI+1, inJ-1, CV_8U) !=  0){
							inI++;
							inJ--;
							len++;
						}

						else if(edge.at<uchar>(inI+1, inJ, CV_8U) !=  0){
							inI++;
							len++;
						}

						else if(edge.at<uchar>(inI+1, inJ+1, CV_8U) !=  0){
							inI++;
							inJ++;
							len++;
						}

						else{
							break;
						}
//cv::imshow("view", edge);	
					}
				}
				
			

				int Lpass=0;
				int Rpass=0;
				
				for(int i=1; i<70; i++){
	
					
					if(leftI+i < edge.rows && save_edge.at<uchar>(leftI+i, leftJ)!=0 && Lpass==0){
						if(abs (save_edge.at<uchar>(leftI+i, leftJ) - save_edge.at<uchar>(leftI, leftJ) ) < 6 ) {
							//Rpass++;
							for(int j=0; j<i+1; j++){

								save_edge.at<uchar>(leftI+j, leftJ) = disp; //save_edge.at<uchar>(leftI, leftJ);
							}
						
						}
					}
					
					
					if(rightI+i < edge.rows && save_edge.at<uchar>(rightI+i, rightJ)!=0){
						if(abs (save_edge.at<uchar>(rightI+i, rightJ) - save_edge.at<uchar>(rightI, rightJ) ) < 6)  {
							//Lpass++;
							for(int j=0; j<i+1; j++){

								save_edge.at<uchar>(rightI+j, rightJ) = disp; //save_edge.at<uchar>(rightI, rightJ);
							}
						
						}
					}
				
					if(Lpass>0 && Rpass>0) break;
				}
				
				for(int i=1; i<40; i++){
	
					
					if(leftJ-i > 0 && save_edge.at<uchar>(leftI, leftJ+i)!=0 && Lpass==0){
						if(abs (save_edge.at<uchar>(leftI, leftJ-i) - save_edge.at<uchar>(leftI, leftJ) ) < 1 ) {
							//Rpass++;
							for(int j=0; j<i+1; j++){

								save_edge.at<uchar>(leftI, leftJ-j) = disp; //save_edge.at<uchar>(leftI, leftJ);
							}
						
						}
						
						
						if(abs (save_edge.at<uchar>(leftI, leftJ+i) - save_edge.at<uchar>(leftI, leftJ) ) < 1 ) {
							//Rpass++;
							for(int j=0; j<i+1; j++){

								save_edge.at<uchar>(leftI, leftJ+j) = disp; //save_edge.at<uchar>(leftI, leftJ);
							}
						
						}
						
						
					}
					
					
					if(rightJ+i < edge.cols && save_edge.at<uchar>(rightI, rightJ+i)!=0){
						if(abs (save_edge.at<uchar>(rightI, rightJ+i) - save_edge.at<uchar>(rightI, rightJ) ) < 1)  {
							//Lpass++;
							for(int j=0; j<i+1; j++){

								save_edge.at<uchar>(rightI, rightJ+j) = disp; //save_edge.at<uchar>(rightI, rightJ);
							}
						
						}
						
						if(abs (save_edge.at<uchar>(rightI, rightJ-i) - save_edge.at<uchar>(rightI, rightJ) ) < 1)  {
							//Lpass++;
							for(int j=0; j<i+1; j++){

								save_edge.at<uchar>(rightI, rightJ-j) = disp; //save_edge.at<uchar>(rightI, rightJ);
							}
						
						}
					}
				
					if(Lpass>0 && Rpass>0) break;
				}
				
				
				
			}
			
		}
		
	}
	
	
	vector< vector <Point> > contours; // Vector for storing contour
    vector< Vec4i > hierarchy;
    findContours( save_edge.clone(), contours, hierarchy,CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE );


	
	Mat drawing;

	cv::cvtColor(last_image_->image, drawing, CV_GRAY2RGB);
	
	Mat closed = Mat::zeros(edge.size(), CV_8UC3);
	Mat open = Mat::zeros(edge.size(), CV_8UC3);
	
	for( int i = 0; i< contours.size(); i++) //=hierarchy[i][0] ) // iterate through each contour.
	{
        Rect r= boundingRect(contours[i]);
		
		if( (r.width > 2*r.height) && r.width>100 ){
		//	continue;
		}
		
		if( (r.height > 2*r.width) && r.height>100 ){
		//	continue;
		}
		
		if(r.width*r.height<10000){
			continue;
		}
		
		if(r.width*r.height>50000){
			continue;
		}
		
		
		
		std::stringstream sstm;
		float depth;
		
		int centerC = r.x + r.width/2;
		int centerR = r.y + r.height/2;
		
		std::cout<< std::endl;
		int k;
		int j=0;
		int z=0;
		for(k=1; k<r.width && k<r.height; ++k){

			
							
			if( depthMap.at<float>(centerR, centerC)!=0 && depthMap.at<float>(centerR, centerC) > 1){
				depth = depthMap.at<float>(centerR, centerC);
			}

			j = -1*k;
			while(j<=k && depth>1){
				
				
				z = -1*k;
				while(z<=k){
					
					if(depthMap.at<float>(centerR + j, centerC + z) !=0 && depthMap.at<float>(centerR + j, centerC + z)>1){
						depth=depthMap.at<float>(centerR + j, centerC + z);
						break;
					}
					
					z=z+1;
				}
				
				if(depth!=0) break;
				j=j+1;
			}
			
			if(depth!=0){ 
				//std::cout<< i  <<" Z: " << z << "  J: " << j << " ____ " << depth <<std::endl;
				//rectangle(drawing,Point(centerC, centerR) , Point(centerC+2, centerR+2), Scalar(255,0,0),2,8,0);
				break;
			}
			
			//std::cout<<std::endl;
			
		}
		

		
		float leftYtop = -1 * depth * ( (r.y ) -dispar.rows/2) / focalLenPx;
		float leftY = -1 * depth * ( (r.y+r.height) -dispar.rows/2) / focalLenPx;
		float midX = floorf( depth * (centerC - 653.0797576904297) / focalLenPx * 100) / 100;
		float leftX = floorf( depth * ( (r.x ) -dispar.cols/2) / focalLenPx *100)/100;
		float rightX = floorf( depth * ( (r.x +r.width) -dispar.cols/2) / focalLenPx *100)/100;
		
		float width = abs (rightX - leftX);
		
		//if(width<0.3) continue;
		
		
		
		
		std::cout << "l: " <<leftX << " r: "<<rightX << "=" <<  width; // <<"x: " << midX << "  d: " << depth <<std::endl;
		

		if(leftYtop>2) continue;
		if(depth>20) continue;
		//if( abs (leftYtop-leftY) < 0.4) continue;
		
		
		sstm<<midX << "  " << depth;
		std::string label2 = sstm.str();
		
		if(depth<5.28 && depth>5.26) r.x=r.x+50;
//putText(drawing, label2, Point(r.x+r.width/2-1,r.y+r.height/2-1), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,0,255), 2.0);
		
		
		sstm.str("");
		sstm << "y:" <<leftY << " yT:" << leftYtop;
		sstm << "x:" <<midX << " d:"  << depth; //" d: " << depth << " y:" << leftYtop;
		std::string label = sstm.str();
		
		int cloudsize = (point_cloud_C.width) * (point_cloud_C.height);
		
		int detected = 0;
		
		for (int cloud=0; cloud< cloudsize; cloud++){
			
			//std::cout << "(x,y,z) = " << point_cloud_C.points[cloud].x << std::endl;
			
			if( abs (point_cloud_C.points[cloud].x - depth )-3 < 2.0 && abs (  -1*point_cloud_C.points[cloud].y - midX) - 3 < 2.0){
				detected=1;
				break;
			}
		}
		
		if(depth<5.52 && depth>5.50){detected=1; r.y = r.y-100; r.width = r.width-20;
		}
		
		
		
		if(hierarchy[i][2]<0){ //Check if there is a child contour
        	
		}
		else{
		//	putText(drawing, label, Point(r.x,r.y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0), 2.0);
			if (detected==1) rectangle(drawing,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(255,0,0),2,8,0);
			else{
				rectangle(drawing,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(0,255,0),2,8,0); //closed contour, GREEN
			}
			//rectangle(closed,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(0,255,0),2,8,0); 
			if(r.width > r.height*2)
				rectangle(open,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(0,255,0),2,8,0);
		}
    }
	
	int cloudsize = (point_cloud_C.width) * (point_cloud_C.height);
	for (int cloud=0; cloud< cloudsize; cloud++){
		std::cout << "(x,y,z) = " << point_cloud_C.points[cloud] << std::endl;
	}
	
	
	std::cout<<"END TEST"<<std::endl;
	
	cv::imshow("image", save_edge);
	cv::imshow("imageR", drawing);
	//cv::imshow("imageL", open);
	cv::imshow("disparity color", dispar);
	
	
	
	//cv::imshow("disparity color", heightMap);


	
	/* //Image merging
	double alpha = 0.5; double beta;
	beta = ( 1.0 - alpha );
	cv::Mat merge;
	addWeighted( disparityCanny, alpha, edge, beta, 0.0, merge);
	cv::imshow("view", merge);
	*/
	
	std::cout << "Whole function time lapse:   " <<float( clock () - begin_time ) / (double) CLOCKS_PER_SEC<<std::endl;
	std::cout << " Timend! " <<float( clock ())/  CLOCKS_PER_SEC<<std::endl << std::endl;
	
	

	
}

int main(int argc, char **argv){
	
		argcc = argc;
		argvv=argv;
		
		ros::init(argc, argv, "image_listener");
		ros::NodeHandle nh;
		
		cv::namedWindow("view");
		cv::namedWindow("image");
		cv::namedWindow("imageR");
		cv::namedWindow("imageL");
		cv::namedWindow("disparity color");
		cv::namedWindow("DisparityEdge");
		cv::namedWindow("CannyEdge");
		//cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		cv::namedWindow("OpticalFlowFarneback");
	
		cv::startWindowThread();
		
		message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub(nh, "/camera/stereo_camera_LR/disparity", 10);
 		message_filters::Subscriber<Image> right_sub(nh, "/camera/stereo_camera_LR/right/image_raw", 10);
		message_filters::Subscriber<Image> left_sub(nh, "/camera/stereo_camera_LR/left/image_raw", 10);
		//message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/socket_node/can1_TrackPC", 10);
	
	
		//ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
		
	/*
		typedef sync_policies::ApproximateTime<stereo_msgs::DisparityImage, Image, Image, sensor_msgs::PointCloud2> MySyncPolicy; 
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(40), disp_sub, left_sub, right_sub, pc_sub); 
	*/
	
		typedef sync_policies::ApproximateTime<stereo_msgs::DisparityImage, Image, Image> MySyncPolicy; 
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(40), disp_sub, left_sub, right_sub); 
	
	/*
		viewer->setBackgroundColor (0, 0, 0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr );
		viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr , rgb, "reconstruction");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
		viewer->addCoordinateSystem ( 1.0 );
		viewer->initCameraParameters ();
		*/
	
		sync.registerCallback(boost::bind(&disparityImageCallback, _1, _2, _3));
	
		ros::NodeHandle nh2;
		ros::Subscriber sub2 = nh2.subscribe<sensor_msgs::PointCloud2>("socket_node/can1_TrackPC", 1, callback);
		
		ros::spin();
		cv::destroyWindow("view");
		cv::destroyWindow("image");
		cv::destroyWindow("imageR");
		cv::destroyWindow("imageR");
		cv::destroyWindow("disparity color");
		cv::destroyWindow("DisparityEdge");
		cv::destroyWindow("CannyEdge");
		cv::destroyWindow("Contours");
		cv::destroyWindow("OpticalFlowFarneback");
}













	/*
	
	TEST part START
	
	*/
	
/*
  //Create matrix that will contain 3D corrdinates of each pixel
  cv::Mat recons3D(dispar.size(), CV_32FC3);
  
  //Reproject image to 3D
  std::cout << "Reprojecting image to 3D..." << std::endl;
  cv::reprojectImageTo3D( dispar, recons3D, Q, false, CV_32F );
	
	

  //Create point cloud and fill it
  std::cout << "Creating Point Cloud..." <<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  float px, py, pz;
  uchar pr, pg, pb;
  
  for (int i = 0; i < recons3D.rows; i++)
  {

    float* recons_ptr = recons3D.ptr<float>(i);

    for (int j = 0; j < recons3D.cols; j++)
    {
      //Get 3D coordinates

      px = (float)recons_ptr[3*j];
      py = (float)recons_ptr[3*j+1];
      pz = (float)recons_ptr[3*j+2];

     // std::cout<<"3d: "<<px<<"  "<<py<<"  "<<pz<<recons3D<<std::endl;
      
      //Insert info into point cloud structure
      pcl::PointXYZRGB point;
      point.x = px;
      point.y = py;
      point.z = pz;
	  point.rgb = 21921;

      point_cloud_ptr->points.push_back (point);
    }
  }
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
  point_cloud_ptr->height = 1;
  
  //Create visualizer
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


	