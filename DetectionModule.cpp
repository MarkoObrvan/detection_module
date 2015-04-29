




#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "stereo_msgs/DisparityImage.h"

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image, const stereo_msgs::DisparityImageConstPtr& disp)
{
  // Solve all of perception here...
	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<stereo_msgs::DisparityImage> info_sub(nh, "camera_info", 1);
  TimeSynchronizer<Image, stereo_msgs::DisparityImage> sync(image_sub, info_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}