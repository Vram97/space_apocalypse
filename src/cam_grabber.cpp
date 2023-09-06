#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam_grabber");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/rgb_img", 1);

  cv::VideoCapture cap(0);

  if(!cap.isOpened())
  {
    ROS_ERROR("Failed to open camera.");
    return -1;
  }
  
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(10);

  cv::Mat frame;

  while (nh.ok()) {
    

    bool suc=cap .read(frame);

    if(!frame.empty())
    {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
    }
    else
    {
      ROS_ERROR("Failed to capture frame.");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  cap.release();
  return 0;
}
