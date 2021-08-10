#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <math.h>

using namespace cv;
using namespace ros;
using namespace std;
using namespace sensor_msgs;

namespace aksan_percept {
  class AksanPercept {
    public: 
      image_transport::Subscriber itSubscriber;
      std::ofstream myfile;

    public:
      AksanPercept(ros::NodeHandle* node);
      ~AksanPercept();

      void improCB(const sensor_msgs::ImageConstPtr& msg);
  };
}