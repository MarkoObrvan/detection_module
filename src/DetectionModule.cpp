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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

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





//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

RNG rng(12345);

static unsigned char colormap[768] =
{ 150, 150, 150,	107, 0, 12,		106, 0, 18,		105, 0, 24,		103, 0, 30,	102, 0, 36,		101, 0, 42,		99, 0, 48,	98, 0, 54,	97, 0, 60,	96, 0, 66,	94, 0, 72,	93, 0, 78,		92, 0, 84,		91, 0, 90,	89, 0, 96,		88, 0, 102,		87, 0, 108,	85, 0, 114,
84, 0, 120,
83, 0, 126,
82, 0, 131,
80, 0, 137,
79, 0, 143,
78, 0, 149,
77, 0, 155,
75, 0, 161,
74, 0, 167,
73, 0, 173,
71, 0, 179,
70, 0, 185,
69, 0, 191,
68, 0, 197,
66, 0, 203,
65, 0, 209,
64, 0, 215,
62, 0, 221,
61, 0, 227,
60, 0, 233,
59, 0, 239,
57, 0, 245,
56, 0, 251,
55, 0, 255,
54, 0, 255,
52, 0, 255,
51, 0, 255,
50, 0, 255,
48, 0, 255,
47, 0, 255,
46, 0, 255,
45, 0, 255,
43, 0, 255,
42, 0, 255,
41, 0, 255,
40, 0, 255,
38, 0, 255,
37, 0, 255,
36, 0, 255,
34, 0, 255,
33, 0, 255,
32, 0, 255,
31, 0, 255,
29, 0, 255,
28, 0, 255,
27, 0, 255,
26, 0, 255,
24, 0, 255,
23, 0, 255,
22, 0, 255,
20, 0, 255,
19, 0, 255,
18, 0, 255,
17, 0, 255,
15, 0, 255,
14, 0, 255,
13, 0, 255,
11, 0, 255,
10, 0, 255,
9, 0, 255,
8, 0, 255,
6, 0, 255,
5, 0, 255,
4, 0, 255,
3, 0, 255,
1, 0, 255,
0, 4, 255,
0, 10, 255,
0, 16, 255,
0, 22, 255,
0, 28, 255,
0, 34, 255,
0, 40, 255,
0, 46, 255,
0, 52, 255,
0, 58, 255,
0, 64, 255,
0, 70, 255,
0, 76, 255,
0, 82, 255,
0, 88, 255,
0, 94, 255,
0, 100, 255,
0, 106, 255,
0, 112, 255,
0, 118, 255,
0, 124, 255,
0, 129, 255,
0, 135, 255,
0, 141, 255,
0, 147, 255,
0, 153, 255,
0, 159, 255,
0, 165, 255,
0, 171, 255,
0, 177, 255,
0, 183, 255,
0, 189, 255,
0, 195, 255,
0, 201, 255,
0, 207, 255,
0, 213, 255,
0, 219, 255,
0, 225, 255,
0, 231, 255,
0, 237, 255,
0, 243, 255,
0, 249, 255,
0, 255, 255,
0, 255, 249,
0, 255, 243,
0, 255, 237,
0, 255, 231,
0, 255, 225,
0, 255, 219,
0, 255, 213,
0, 255, 207,
0, 255, 201,
0, 255, 195,
0, 255, 189,
0, 255, 183,
0, 255, 177,
0, 255, 171,
0, 255, 165,
0, 255, 159,
0, 255, 153,
0, 255, 147,
0, 255, 141,
0, 255, 135,
0, 255, 129,
0, 255, 124,
0, 255, 118,
0, 255, 112,
0, 255, 106,
0, 255, 100,
0, 255, 94,
0, 255, 88,
0, 255, 82,
0, 255, 76,
0, 255, 70,
0, 255, 64,
0, 255, 58,
0, 255, 52,
0, 255, 46,
0, 255, 40,
0, 255, 34,
0, 255, 28,
0, 255, 22,
0, 255, 16,
0, 255, 10,
0, 255, 4,
2, 255, 0,
8, 255, 0,
14, 255, 0,
20, 255, 0,
26, 255, 0,
32, 255, 0,
38, 255, 0,
44, 255, 0,
50, 255, 0,
56, 255, 0,
62, 255, 0,
68, 255, 0,
74, 255, 0,
80, 255, 0,
86, 255, 0,
92, 255, 0,
98, 255, 0,
104, 255, 0,
110, 255, 0,
116, 255, 0,
122, 255, 0,
128, 255, 0,
133, 255, 0,
139, 255, 0,
145, 255, 0,
151, 255, 0,
157, 255, 0,
163, 255, 0,
169, 255, 0,
175, 255, 0,
181, 255, 0,
187, 255, 0,
193, 255, 0,
199, 255, 0,
205, 255, 0,
211, 255, 0,
217, 255, 0,
223, 255, 0,
229, 255, 0,
235, 255, 0,
241, 255, 0,
247, 255, 0,
253, 255, 0,
255, 251, 0,
255, 245, 0,
255, 239, 0,
255, 233, 0,
255, 227, 0,
255, 221, 0,
255, 215, 0,
255, 209, 0,
255, 203, 0,
255, 197, 0,
255, 191, 0,
255, 185, 0,
255, 179, 0,
255, 173, 0,
255, 167, 0,
255, 161, 0,
255, 155, 0,
255, 149, 0,
255, 143, 0,
255, 137, 0,
255, 131, 0,
255, 126, 0,
255, 120, 0,
255, 114, 0,
255, 108, 0,
255, 102, 0,
255, 96, 0,
255, 90, 0,
255, 84, 0,
255, 78, 0,
255, 72, 0,
255, 66, 0,
255, 60, 0,
255, 54, 0,
255, 48, 0,
255, 42, 0,
255, 36, 0,
255, 30, 0,
255, 24, 0,
255, 18, 0,
255, 12, 0,
255, 6, 0,
255, 0, 0,
};




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



void disparityImageCallback(const stereo_msgs::DisparityImageConstPtr& disparity_msg, const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg)
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
	
	
	cv::Mat edge, edge2, draw;
	
	const clock_t begin_time = clock();
	std::cout << "Timestart! " <<float( clock ())/  CLOCKS_PER_SEC<< "  ---";
	
//	GaussianBlur(last_image_->image, edge2, cv::Size(7,7), 4, 4, 0);
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
	*(
	
	
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
	
	cv::Mat_<cv::Vec3b> disparity_color_;
	disparity_color_.create(disparity_msg->image.height, disparity_msg->image.width);
	
	//constructs a matrix on top of user-allocated data. step is in bytes(!!!), regardless of the type 
	const cv::Mat_<float> dmat(disparity_msg->image.height, disparity_msg->image.width, (float*)&disparity_msg->image.data[0], disparity_msg->image.step);
	
	cv::Mat dispar(disparity_msg->image.height, disparity_msg->image.width,CV_8U);
	
	for (int row = 0; row < disparity_color_.rows; ++row) {
		const float* d = dmat[row];
		for (int col = 0; col < disparity_color_.cols; ++col) {
				
			if(d[col]==-1){
					//disparity_color_(row, col)[2]=0;
					//disparity_color_(row, col)[1]=0;
					//disparity_color_(row, col)[0]=0;
					dispar.at<char>(row, col, CV_8U) = 0;
					continue;
				}
				
				int index = (d[col] - min_disparity) * multiplier + 0.5;
				index = std::min(255, std::max(0, index));
				// Fill as BGR
			/*
				disparity_color_(row, col)[2] = colormap[3*index + 0];
				disparity_color_(row, col)[1] = colormap[3*index + 1];
				disparity_color_(row, col)[0] = colormap[3*index + 2];
				*/

				dispar.at<char>(row, col, CV_8U) = index;
			}
		}
	
	
	cv::Mat left = shiftFrame(last_image_R->image, 0, ShiftLeft);
	cv::Mat leftZ (dispar.rows, dispar.cols,CV_8U);
	
	
	int count=0;
	for (int row = 1; row < dispar.rows-1; ++row) {
		
		const char* cMid = left.ptr<char>(row);
		const char* cTop = left.ptr<char>(row-1);
		const char* cBot = left.ptr<char>(row+1);
		
		const char* dMid = dispar.ptr<char>(row);
		const char* dTop = dispar.ptr<char>(row-1);
		const char* dBot = dispar.ptr<char>(row+1);
		
		for (int col = 1; col < dispar.cols-1; ++col) {
			
			if(cMid[col] == 0) continue;
			
			double step = dMid[col] - (dMid[col-1]+dMid[col+1]+dTop[col]+dTop[col-1]+dTop[col+1] + dBot[col]+ dBot[col-1]+ dBot[col+1])/((double) 8);
			
			

			
			if(step<2 && step>-2 && dMid[col]!=0){
			//if(dMid[col] != 0){	
				count++;
				
				/*				
				leftZ.at<char>(row, col, CV_8U) = 	(left.at<char>(row, col, CV_8U) +
													left.at<char>(row-1, col-1, CV_8U) +
													left.at<char>(row-1, col+1, CV_8U) +
													left.at<char>(row-1, col, CV_8U) +

													left.at<char>(row, col+1, CV_8U) +
													left.at<char>(row, col-1, CV_8U) +

													left.at<char>(row+1, col-1, CV_8U) +
													left.at<char>(row+1, col+1, CV_8U) +
													left.at<char>(row+1, col, CV_8U)  ) /9;
													*/
					
				
				/*
				left.at<char>(row, col, CV_8U) = 	( 4*left.at<char>(row, col, CV_8U) +
													left.at<char>(row-1, col-1, CV_8U) +
													left.at<char>(row-1, col+1, CV_8U) +
													2*left.at<char>(row-1, col, CV_8U) +

													2*left.at<char>(row, col+1, CV_8U) +
													2*left.at<char>(row, col-1, CV_8U) +

													left.at<char>(row+1, col-1, CV_8U) +
													left.at<char>(row+1, col+1, CV_8U) +
													2*left.at<char>(row+1, col, CV_8U)  ) /16;
													*/
													
				
				//leftZ.at<char>(row, col, CV_8U) =  (dMid[col] + 4*left.at<char>(row, col, CV_8U))/5;
				
				/*				
				leftZ.at<char>(row, col, CV_8U) = 	( 2*left.at<char>(row, col, CV_8U) +
													left.at<char>(row-1, col-1, CV_8U) +
													left.at<char>(row-1, col+1, CV_8U) +
													2*left.at<char>(row-1, col, CV_8U) +

													2*left.at<char>(row, col+1, CV_8U) +
													2*left.at<char>(row, col-1, CV_8U) +

													left.at<char>(row+1, col-1, CV_8U) +
													left.at<char>(row+1, col+1, CV_8U) +
													2*left.at<char>(row+1, col, CV_8U)  ) /14;
													*/
												
				
				/*
				leftZ.at<char>(row-1, col-1, CV_8U) = left.at<char>(row-1, col-1, CV_8U) - (left.at<char>(row-1, col-1, CV_8U) - left.at<char>(row, col, CV_8U))/2;
				leftZ.at<char>(row-1, col+1, CV_8U) = left.at<char>(row-1, col+1, CV_8U) - (left.at<char>(row-1, col+1, CV_8U) - left.at<char>(row, col, CV_8U))/2;
				leftZ.at<char>(row-1, col, CV_8U) = left.at<char>(row-1, col, CV_8U) - (left.at<char>(row-1, col, CV_8U) - left.at<char>(row, col, CV_8U))/2;
				
				leftZ.at<char>(row, col+1, CV_8U) = left.at<char>(row, col+1, CV_8U) - (left.at<char>(row, col+1, CV_8U) - left.at<char>(row, col, CV_8U))/2;
				leftZ.at<char>(row, col-1, CV_8U) = left.at<char>(row, col-1, CV_8U) - (left.at<char>(row, col-1, CV_8U) - left.at<char>(row, col, CV_8U))/2;
				leftZ.at<char>(row, col, CV_8U) = left.at<char>(row, col, CV_8U);
				
				leftZ.at<char>(row+1, col-1, CV_8U) = left.at<char>(row+1, col-1, CV_8U) - (left.at<char>(row+1, col-1, CV_8U) - left.at<char>(row, col, CV_8U))/2;
				leftZ.at<char>(row+1, col+1, CV_8U) = left.at<char>(row+1, col+1, CV_8U) - (left.at<char>(row+1, col+1, CV_8U) - left.at<char>(row, col, CV_8U))/2;
				leftZ.at<char>(row+1, col, CV_8U)   = left.at<char>(row+1, col, CV_8U)   - (left.at<char>(row+1, col, CV_8U)   - left.at<char>(row, col, CV_8U))/2;
				*/
				
				}
			
			else
			{
				leftZ.at<char>(row, col, CV_8U) = left.at<char>(row-1, col, CV_8U); //leftZ.at<char>(row-1, col, CV_8U);
			}
				leftZ.at<char>(row, col, CV_8U) = left.at<char>(row, col, CV_8U) ;
			}
		}
	std::cout << "Blurs: " <<count<<std::endl;
	
	cv::imshow("view", leftZ);
	cv::imshow("image", last_image_->image);

	
	double alpha = 0.7; double beta;
	beta = ( 1.0 - alpha );
	cv::Mat merge, mergeb;
	
	//GaussianBlur(dispar, dispar, cv::Size(5,5), 4, 4, 0);
	cv::addWeighted( last_image_->image, alpha, last_image_R->image, beta, 0.0, merge);
	
	alpha=0.8;
	//cv::addWeighted( last_image_->image, alpha, dispar, beta, 0.0, merge);
//	GaussianBlur(merge, merge, cv::Size(7,7), 4, 4, 0);
	bilateralFilter(merge, mergeb, 4, 40, 7);
	//cv::imshow("image", mergeb);

	
	//GaussianBlur(merge, merge, cv::Size(7,7), 4, 4, 0);
	//bilateralFilter(merge, mergeb,60, 50, 7);
	
//	Canny( mergeb, edge, 50, 150, 3);
	Canny( leftZ, edge, 50, 150, 3);
	cv::imshow("CannyEdge", edge);
	
	
	
	cv::imshow("disparity color", dispar);
	
	
	cv::Mat disparityCanny;
	Canny( dispar, disparityCanny, 20, 60, 3);
	cv::imshow("DisparityEdge", disparityCanny);
	
	/* //Image merging
	double alpha = 0.5; double beta;
	beta = ( 1.0 - alpha );
	cv::Mat merge;
	addWeighted( disparityCanny, alpha, edge, beta, 0.0, merge);
	cv::imshow("view", merge);
	*/
	
	std::cout << "Whole function time lapse:   " <<float( clock () - begin_time ) / (double) CLOCKS_PER_SEC<<std::endl;
	std::cout << " Timend! " <<float( clock ())/  CLOCKS_PER_SEC<<std::endl;
	
	
	
	/*

working 3D reconstruction for testing purposes

  //Create matrix that will contain 3D corrdinates of each pixel
  cv::Mat recons3D(dispar.size(), CV_32FC3);
  
  //Reproject image to 3D
  std::cout << "Reprojecting image to 3D..." << std::endl;
  cv::reprojectImageTo3D( dispar, recons3D, Q, false, CV_32F );

  //Create point cloud and fill it
  std::cout << "Creating Point Cloud..." <<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  double px, py, pz;
  uchar pr, pg, pb;
  
  for (int i = 0; i < dispar.rows; i++)
  {
    char* rgb_ptr = dispar.ptr<char>(i);

    char* disp_ptr = dispar.ptr<char>(i);

    for (int j = 0; j < dispar.cols; j++)
    {
      //Get 3D coordinates
		
      uchar d = disp_ptr[j];
		
		
      if ( d == 0 ) continue; //Discard bad pixels
		
		
      double pw = -1.0 * static_cast<double>(d) * Q32 + Q33; 
      px = static_cast<double>(j) + Q03;
      py = static_cast<double>(i) + Q13;
      pz = Q23;
	  
      
      px = -1*px/pw;
      py = py/pw;
      pz = pz/pw;

      
      //Get RGB info
      pb = 255;
      pg = 255;
      pr = 255;
      
      //Insert info into point cloud structure
      pcl::PointXYZRGB point;
      point.x = px;
      point.y = py;
      point.z = pz;
      uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
              static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
  }
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
  point_cloud_ptr->height = 1;

	
    viewer.showCloud (point_cloud_ptr);
	
	*/
	
	
}

int main(int argc, char **argv){
	
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
		
		message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub(nh, "/camera/stereo_camera_LR/disparity", 1);
 		message_filters::Subscriber<Image> right_sub(nh, "/camera/stereo_camera_LR/right/image_raw", 1);
		message_filters::Subscriber<Image> left_sub(nh, "/camera/stereo_camera_LR/left/image_raw", 1);
		
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

	
	viewer.showCloud (point_cloud_ptr);
*/
		/*
	
	TEST part END
	
	*/
	
