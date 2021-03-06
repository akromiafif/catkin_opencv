#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <aksan_percept.hpp>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <cstdlib>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;


namespace aksan_percept {
  AksanPercept::AksanPercept(ros::NodeHandle* node) {
    cv::namedWindow(OPENCV_WINDOW, WINDOW_AUTOSIZE);

    cv::startWindowThread(); 

    //Define source of image
    image_transport::ImageTransport it(*node); 

    // VARIABLE BUAT SIMPEN VIDEO
    cv::Size size(
      (int) 640,
      (int) 360
    );

    writer.open("vision_detected.avi", VideoWriter::fourcc('M','J','P','G'), 30, size);
    // VARIABLE BUAT SIMPEN VIDEO

    //Image subscriber to "camera/image" topic
    itSubscriber = it.subscribe(IMAGE_TOPIC, 1, &AksanPercept::improCB, this); 
    commandClient = node->serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");  
  }

  AksanPercept::~AksanPercept() {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void AksanPercept::improCB(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    
    try {      
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

      //Convert image from msg to BGR format
      Mat BGR = cv_ptr->image;

      //Make copy of image for processing
      Mat orig_image = BGR.clone(); 
      
      //Introduce blur for image flitering and converting
      cv::medianBlur(BGR, BGR, 7); 
      
      // Convert input image to HSV
      Mat HSV;

      //Convert BGR image to HSV image
      cv::cvtColor(BGR, HSV, COLOR_BGR2HSV); 

      vector<Vec3f> circles;

      // CUSTOM FOR RED RANGE //
      // Threshold the HSV image, keep only the red pixels
      Mat lower_hue;
      Mat upper_hue;

      //Lower hue threshold range
      inRange(HSV, Scalar(0, 170, 160), Scalar(10, 255, 255), lower_hue); 
      
      //Upper hue threshold range
      inRange(HSV, Scalar(165, 100, 100), Scalar(180, 255, 255), upper_hue); 
      
      Mat hue_image;
      
      cv::addWeighted(lower_hue, 1.0, upper_hue, 1.0, 0.0, hue_image); 
      
      cv::GaussianBlur(hue_image, hue_image, Size(9, 9), 2, 2); 
      
      cv::HoughCircles(hue_image, circles, CV_HOUGH_GRADIENT, 1, hue_image.rows/4, 100, 25, 25, 100); 
      // CUSTOM FOR RED RANGE //


      // ============== CUSTOM FOR PINK RANGE ============== //
      Mat lower_pink;

      cv::inRange(HSV, cv::Scalar(135, 40, 160), cv::Scalar(160, 255, 255), lower_pink);

      cv::GaussianBlur(lower_pink, lower_pink, Size(9, 9), 2, 2);

      cv::HoughCircles(lower_pink, circles, CV_HOUGH_GRADIENT, 1, lower_pink.rows/4, 100, 25, 25, 100); 
      // ============== CUSTOM FOR PINK RANGE ============== //

      if (circles.size() != 0) {
        // doServoMove(8, 500);
        ROS_INFO("Red Circle Detected");
        // sleep(3);
        // doServoMove(8, 1300);
        // sleep(5);
        // doServoMove(9, 700);
        // sleep(3);
        // doServoMove(9, 1600);
      }

      // Highlight detected object
      for(size_t cur = 0; cur < circles.size(); ++cur) {
        //Define centre point of detected circle
        Point center(circles[cur][0], circles[cur][1]);

        //Define radius of detected circle
        int radius = circles[cur][2]; 
        
        //Overlay detected cricle outline onto origional image
        circle(orig_image, center, radius, Scalar(239, 152, 38), 2); 

        //allow for display of image for given milliseconds (Image overlay refreshrate)
        waitKey(10);  
      }

      // DISABLE KALO MODE FLIGHT
      // cv::imshow(OPENCV_WINDOW, orig_image);
      // cv::imshow("Red Hue", hue_image);
      // cv::imshow("Pink Hue", lower_pink);
      // DISABLE KALO MODE FLIGHT

      writer << orig_image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void AksanPercept::doServoMove(int channel, int pwm) {
    ros::Rate rate(30.0);

    mavros_msgs::CommandLong::Request request;
    mavros_msgs::CommandLong::Response response;

    request.command = 183;
    request.confirmation = 1;
    request.param1 = channel;
    request.param2 = pwm;
    request.param3 = 0;
    request.param4 = 0;
    request.param5 = 0;
    request.param6 = 0;
    request.param7 = 0;
    bool succeed = commandClient.call(request, response);

    if (succeed) {
      ROS_INFO("Payload Dropped");
    } else {
      ROS_INFO("Payload not Dropped");
    }

    ros::spinOnce();
    rate.sleep();
  }
}
