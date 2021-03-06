#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <math.h>
#include <sstream>

#include <mavros_msgs/CommandLong.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

using namespace cv;
using namespace ros;
using namespace std;
using namespace sensor_msgs;
namespace aksan_percept {
  // OpenCV Window Name for imshow
  static const std::string OPENCV_WINDOW = "Vision Detected";

  // Topics
  static const std::string IMAGE_TOPIC = "/camera/image";
  

  class AksanPercept {
    public: 
      image_transport::Subscriber itSubscriber;
      ros::ServiceClient commandClient;

      // Save video variable
      cv::VideoWriter writer;
      

    public:
      AksanPercept(ros::NodeHandle* node);
      ~AksanPercept();

      void improCB(const sensor_msgs::ImageConstPtr& msg);
      void doServoMove(int channel, int pwm);
  };
  
}