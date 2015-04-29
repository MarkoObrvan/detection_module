#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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
#include <pcl/io/pcd_io.h>

#include <iostream>

#include "boost/multi_array.hpp"

#define THRESHOLD 8
#define VERTICAL_MERGE_BOUNDARY 20

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace sensor_msgs;
using namespace message_filters;

static unsigned char colormap[768] =
{ 150, 150, 150,
107, 0, 12,
106, 0, 18,
105, 0, 24,
103, 0, 30,
102, 0, 36,
101, 0, 42,
99, 0, 48,
98, 0, 54,
97, 0, 60,
96, 0, 66,
94, 0, 72,
93, 0, 78,
92, 0, 84,
91, 0, 90,
89, 0, 96,
88, 0, 102,
87, 0, 108,
85, 0, 114,
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

void disparityImageCallback(const stereo_msgs::DisparityImageConstPtr& disparity_msg, const sensor_msgs::ImageConstPtr& left_msg)
{
	
	std::cout <<"In callback"<<std::endl;
	
	ros::NodeHandle nh;
	ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2> ("FromDisparityToWorld", 10);
	sensor_msgs::PointCloud2 output;
	
	PointCloud::Ptr msg (new PointCloud);
	msg->height = msg->width = 1;
	msg->header.frame_id = "3D";
	int g=0;
	cv_bridge::CvImagePtr last_image_;
	
	last_image_ = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::MONO8);
	
    cv::Mat edge, draw;
 
    Canny( last_image_->image, edge, 60, 140, 3);
 
	
	
	
    edge.convertTo(draw, CV_8U);
    cv::imshow("CannyEdge", draw);
	
	
	try
	{
		
		float min_disparity = disparity_msg->min_disparity;
		float max_disparity = disparity_msg->max_disparity;
		float multiplier = 255.0f / (max_disparity - min_disparity);

		std::cout<< "minimal disparity: " << min_disparity << "  maximal disparity: " << max_disparity << std::endl;
		std::cout<< "focal length pixels: " << disparity_msg->f << "  focal baseline world units: " << disparity_msg->T << std::endl;
		
		cv::Mat_<cv::Vec3b> disparity_color_;
		disparity_color_.create(disparity_msg->image.height, disparity_msg->image.width);
                
        cv::Mat_<cv::Vec3b> edges_;
		edges_.create(disparity_msg->image.height, disparity_msg->image.width);
		
		//constructs a matrix on top of user-allocated data. step is in bytes(!!!), regardless of the type 
		const cv::Mat_<float> dmat(disparity_msg->image.height, disparity_msg->image.width,
		(float*)&disparity_msg->image.data[0], disparity_msg->image.step);
		
		typedef boost::multi_array<float, 3> array_type;
		typedef array_type::index index;
		array_type mat3DfromDisparity(boost::extents[disparity_msg->image.height][disparity_msg->image.width][3]);
		
		int centerCol = disparity_msg->image.width/2;
		int centerRow = disparity_msg->image.height/2;
		int u,v;
		
		/*
                Run trough disparity map and calculate depth map
		*/
		for (int row = 0; row < disparity_msg->image.height; ++row) {
			
			if(row== disparity_msg->image.height-1) std::cout<< "Run this many times:P : "<< g+1 << std::endl;
			
			const float* d = dmat[row];
			
			for (int col = 0; col < disparity_msg->image.width; ++col) {

				if(d[col]!=-1){
				
//std::cout<<d[col]<<" "<<std::endl;
					
					float Z = disparity_msg->f * disparity_msg->T / d[col];
					
					u = col - centerCol;
					v = row - centerRow;
					
					mat3DfromDisparity[row][col][0]= Z; //Z
					mat3DfromDisparity[row][col][1]= u * Z/disparity_msg->f;   //X = uZ/f
					mat3DfromDisparity[row][col][2]= v * Z/disparity_msg->f;   //Y=vZ/f

					msg->points.push_back (pcl::PointXYZ(Z, mat3DfromDisparity[row][col][1], mat3DfromDisparity[row][col][2]));
				}
			}
		}
		
                /*
                 Publish in ROS point cloud (and kill my computer -.-)
                 */		
		msg->height = 0;
		msg->width = 0;
		msg->header.stamp =  g;
		pcl::toROSMsg(*msg, output);	
		g++;
		publisher.publish (output);
		msg->points.clear();
		
		/*
			Run trough disparity map and calculate edges using: http://www.vision.caltech.edu/VisionWiki/images/9/92/Franke96fast.pdf
			something like given in example. TODO mod it to calculate smooth changes in disparity, aka. sides of car where one side of disparity
			is ~constant and other one is smooth decreasing using neighbourghs at distance z:
			
			modified matrix					
						i
						-
						-							usual edge detector matrix
						-								i
		j	-	-	-	x	-	-	-	-	j		j	x	j
						-								i
						-
						-
						-
						i
						
		usual edge detector g(x) - j>threshold ---> x is vertical edge and belongs to object (same position is disparity of object)
							g(x) - i>threshold ----> x is horizontal edge and belongs to object
			
		*/
		
		
		//unmodified matrix test
		
		for (int row = 1; row < disparity_msg->image.height - 1; ++row) {
			
			const float* up = dmat[row-1];
			const float* mid = dmat[row];
                        const float* bot = dmat[row+1];
                        
			for (int col = 1; col < disparity_msg->image.width - 1; ++col) {
                            
                            //check horizontal
                            if(mid[col]-up[col]>THRESHOLD || mid[col]-bot[col]>THRESHOLD){
                                edges_(row, col)[0] = 255;
                                continue;
                            }
                            
                            if(mid[col]-mid[col-1]>THRESHOLD || mid[col]-mid[col+1]>THRESHOLD){
                                edges_(row, col)[1] = 255;
                                continue;
                            }
                            
                            edges_(row, col)[0] = edges_(row, col)[1] = edges_(row, col)[2] = 0;
                            
                        }
                    }
		
                cv::imshow("Edge map", edges_);
		
		
		/******THIS IS TEST FOR ADVANCE EDGE DETECTOR******/
			for (int row = 1; row < disparity_msg->image.height - VERTICAL_MERGE_BOUNDARY ; ++row) {

				for (int col = 1; col < disparity_msg->image.width - 1; ++col) {
                            
					for(int i=VERTICAL_MERGE_BOUNDARY; i>0; i--){
						if (edges_(row-i, col)[1] == 255  && edges_(row, col)[1] == 255){
							while(i!=0){
								edges_(row-i, col)[2]=255;
								i--;
							}
						}
					}
                            
				}
			}
		
         	cv::imshow("Edge map", edges_);
		
		
		
                
                /* Tesrun, no disparity during this */
		/*DISPLAY DISPARITY WITH COLOR MAP*/
		for (int row = 0; row < disparity_color_.rows; ++row) {
			const float* d = dmat[row];
			for (int col = 0; col < disparity_color_.cols; ++col) {
				
				if(d[col]==-1){
					disparity_color_(row, col)[2]=disparity_color_(row, col)[1]=disparity_color_(row, col)[0]=0;
					
					continue;
				}
				
				int index = (d[col] - min_disparity) * multiplier + 0.5;
				index = std::min(255, std::max(0, index));
				// Fill as BGR
				disparity_color_(row, col)[2] = colormap[3*index + 0];
				disparity_color_(row, col)[1] = colormap[3*index + 1];
				disparity_color_(row, col)[0] = colormap[3*index + 2];
			}
		}

		cv::imshow("view", disparity_color_);
                
                
		//std::cout<<msg->T<<endl;
		//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e)
	{
	   // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv){
	
		ros::init(argc, argv, "image_listener");
		ros::NodeHandle nh;
		
		cv::namedWindow("view");
		cv::namedWindow("Edge map");
		cv::namedWindow("CannyEdge");
		cv::startWindowThread();
		
		message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub(nh, "/camera/stereo_camera_LR/disparity", 1);
 		message_filters::Subscriber<Image> left_sub(nh, "/camera/stereo_camera_LR/left/image_raw", 1);
		
		typedef sync_policies::ApproximateTime<stereo_msgs::DisparityImage, Image> MySyncPolicy; //ovo dodao
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), disp_sub, left_sub); /// ovo dodao
	
		//TimeSynchronizer<stereo_msgs::DisparityImage, Image> sync(disp_sub, left_sub, 10);  //ovo maknuo
		
		sync.registerCallback(boost::bind(&disparityImageCallback, _1, _2));
		
		ros::spin();
		cv::destroyWindow("view");
}

