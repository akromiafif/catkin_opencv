#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <aksan_percept.hpp>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;


namespace aksan_percept {
  AksanPercept::AksanPercept(ros::NodeHandle* node) {
    //Create viewable window - Image overlay output
    cv::namedWindow(OPENCV_WINDOW, WINDOW_AUTOSIZE);

    //Begin window view and image display
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
      
      // Threshold the HSV image, keep only the red pixels
      Mat lower_hue;
      Mat upper_hue;
      
      //Lower hue threshold range
      inRange(HSV, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_hue); 
      
      //Upper hue threshold range
      inRange(HSV, Scalar(165, 100, 100), Scalar(179, 255, 255), upper_hue); 
      
      // Combine the above two images
      Mat hue_image;
      
      //Combine hue images for object recognition
      cv::addWeighted(lower_hue, 1.0, upper_hue, 1.0, 0.0, hue_image); 
      
      //Introduce interference
      //Introduce noise
      cv::GaussianBlur(hue_image, hue_image, Size(9, 9), 2, 2); 
      
      //Hough transform to detect circles
      //Detected circles array
      vector<Vec3f> circles;

      //HOUGH CIRCLE TRANSFORNATION
      cv::HoughCircles(hue_image, circles, CV_HOUGH_GRADIENT, 1, hue_image.rows/4, 100, 25, 1, 100); 
      //imshow("Original", orig_image);

      if (circles.size() != 0) {
        ROS_INFO("Red Circle Detected");
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
      // DISABLE KALO MODE FLIGHT

      writer << orig_image;


    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}

