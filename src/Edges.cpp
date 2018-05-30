#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string RAW_WINDOW = "Raw Image";
static const std::string EDGES_WINDOW = "Edge Detection";

class Edge_Detector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  Edge_Detector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &Edge_Detector::imageCb, this);
    image_pub_ = it_.advertise("/edge_detector/raw_image", 1);

  }

  ~Edge_Detector()
  {
    //cv::destroyWindow(RAW_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600){	     
      cv_ptr->image = detect_edges(cv_ptr->image);
      image_pub_.publish(cv_ptr->toImageMsg());
	  }
  }

  cv::Mat detect_edges(cv::Mat img)
  {
    cv::Mat img_hsv, img_gray, mask_y, mask_w, bitwise_yw, mask_imyw;
  	int rows_win = 720, cols_win = 402;
    //cv::namedWindow(RAW_WINDOW, cv::WINDOW_NORMAL);
    //cv::namedWindow(EDGES_WINDOW, cv::WINDOW_NORMAL);

  	//cv::resizeWindow(RAW_WINDOW, rows_win, cols_win);
  	//cv::resizeWindow(EDGES_WINDOW, rows_win, cols_win);

    //cv::imshow(RAW_WINDOW, img);

	  cv::cvtColor( img, img_gray, CV_BGR2GRAY);
    cv::cvtColor( img, img_hsv, CV_BGR2HSV);

    //Value thresholds for the HSV representation of yellow 
    int am_low_h = 16, am_low_s = 90, am_low_v = 50;
    int am_high_h = 45, am_high_s = 255, am_high_v = 255;
    cv::Scalar am_low = cv::Scalar(am_low_h, am_low_s, am_low_v);
    cv::Scalar am_high = cv::Scalar(am_high_h, am_high_s, am_high_v);
    
    //Get just the pixels that comply with the yellow or white threshold
    cv::inRange(img_hsv, am_low, am_high, mask_y);
    cv::inRange(img_gray, 180, 255, mask_w);
    
    //Join both thresholds in one mask so we get the places where it's either white or yellow
    cv::bitwise_or(mask_y, mask_w, bitwise_yw);
    //Apply that mask to the grayscale image
    cv::bitwise_and(bitwise_yw, img_gray, mask_imyw);
    //Eliminate everything above the horizon



    //cv::rectangle(mask_imyw, cv::Point(0, 0), cv::Point(mask_imyw.cols-1, ya), cv::Scalar(0), -1, 8);
    


    //Smooth the image for the coming Morphology transformations
    //cv::imshow(EDGES_WINDOW, mask_imyw);
    //Define the Canny thresholds using a ratio of 1:2 and a Kernel of 5x5
    int cannyLowT = 50, cannyHighT = 90, canny_kernel = 5;
    
    //Apply Canny Edge detection
    cv::Canny(mask_imyw, img, cannyLowT, cannyHighT, canny_kernel);
    
    
    cv::waitKey(3);
    cv::Mat imgEdges;
    //Convert to 3 channel image for image transport
    cv::cvtColor(img, imgEdges, CV_GRAY2BGR);
    return imgEdges;

  }	
 
};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Edges_Node");
  ROS_INFO("Edge detector online");
  	Edge_Detector ic;
  	ros::spin();
	return 0;
}