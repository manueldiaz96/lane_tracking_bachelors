#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

static const std::string EDGES_WINDOW = "Line Detection";

class Line_Detector
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

  public:
    Line_Detector()
      : it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/edge_detector/raw_image", 1, &Line_Detector::imageCb, this);
      image_pub_ = it_.advertise("/line_detector/raw_image", 1);
      int rows_win = 720, cols_win = 402;
      cv::namedWindow(EDGES_WINDOW, cv::WINDOW_NORMAL);
      cv::resizeWindow(EDGES_WINDOW, rows_win, cols_win);

    }

    ~Line_Detector()
    {
      cv::destroyWindow(EDGES_WINDOW);
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

      // Draw an example circle on the video stream
      if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600) {
        cv_ptr->image=detect_lines(cv_ptr->image);
        image_pub_.publish(cv_ptr->toImageMsg());
      }
    }

    cv::Mat detect_lines(cv::Mat img)
    { 

      cv::Mat lines;
      std::vector<int> clasif;
      std::vector<int> vecxR, vecyR;
      std::vector<int> vecxL, vecyL;
      int LeftLine[4] = {0, 0, 0, 0};
      int RightLine[4] = {0, 0, 0, 0};
      int LastLeftLine[4] = {0, 0, 0, 0};
      int LastRightLine[4] = {0, 0, 0, 0};



      int ya = 0;
      double theta = CV_PI / 180, rho = 1; //Standard values
      double min_line_len = 10; //The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
      double max_line_gap = 3; //The maximum gap between two points to be considered in the same line.
      int threshold = 45; //The minimum number of intersections to “detect” a line

      //Apply Probabilistic Hough Lines Transform to the image
      cv::cvtColor(img, img, CV_BGR2GRAY);
      int cannyLowT = 50, cannyHighT = 90, canny_kernel = 5;
      cv::Canny(img, img, cannyLowT, cannyHighT, canny_kernel);
      cv::HoughLinesP(img, lines, rho, theta, threshold, min_line_len, max_line_gap );

      double yb = img.rows;
      //Prepare variables for line drawing.
      double m1, b1_int, xa1, xb1, xc1, xc2;
      double m2, b2_int, xa2, xb2;

      //Classify each one of the lines
      for ( int i = 0; i < lines.rows; i++ )
      {

        //Set variables as double
        double x1 = lines.row(i).at<int>(0) * 1.0;
        double y1 = lines.row(i).at<int>(1) * 1.0;
        double x2 = lines.row(i).at<int>(2) * 1.0;
        double y2 = lines.row(i).at<int>(3) * 1.0;

        double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        //For each line that isn't vertical and is below the horizon
        if ((x2 - x1) != 0 && ((y1 >= ya) && (y2 >= ya))) {

          //Find the slope for the line
          double slope = (y2 - y1) / (x2 - x1);

          if (slope > 0.5 ) {
            //classify line as right
            clasif.push_back(0);
          }

          else if (slope < -0.5 ) {
            //classify line as left
            clasif.push_back(1);
          }

          else {
            //If the line is neither left or right, it is null
            clasif.push_back(2);
          }
        }

        else {
          //If the line is vertical, it is null
          clasif.push_back(2);
        }

      }
      //Initialize the arrays
      for ( int i = 0; i < lines.rows; i++ )
      {
        double x1 = lines.row(i).at<int>(0) * 1.0;
        double y1 = lines.row(i).at<int>(1) * 1.0;
        double x2 = lines.row(i).at<int>(2) * 1.0;
        double y2 = lines.row(i).at<int>(3) * 1.0;
        if (clasif[i] == 0) {
          RightLine[0] = x1;
          RightLine[1] = y1;
          RightLine[2] = x2;
          RightLine[3] = y2;
          break;
        }
      }
      for ( int i = 0; i < lines.rows; i++ )
      {
        double x1 = lines.row(i).at<int>(0) * 1.0;
        double y1 = lines.row(i).at<int>(1) * 1.0;
        double x2 = lines.row(i).at<int>(2) * 1.0;
        double y2 = lines.row(i).at<int>(3) * 1.0;
        if (clasif[i] == 1) {
          LeftLine[0] = x1;
          LeftLine[1] = y1;
          LeftLine[2] = x2;
          LeftLine[3] = y2;
          break;
        }
      }

      //Find the mean value for each line
      //and store all x and y values for the median

      for ( int i = 0; i < lines.rows; i++ )
      {
        double x1 = lines.row(i).at<int>(0) * 1.0;
        double y1 = lines.row(i).at<int>(1) * 1.0;
        double x2 = lines.row(i).at<int>(2) * 1.0;
        double y2 = lines.row(i).at<int>(3) * 1.0;
        if (clasif[i] == 0) {

          RightLine[0] = (x1 + RightLine[0]) / 2;
          RightLine[1] = (y1 + RightLine[1]) / 2;
          RightLine[2] = (x2 + RightLine[2]) / 2;
          RightLine[3] = (y2 + RightLine[3]) / 2;
          vecxR.push_back(x1);
          vecxR.push_back(x2);
          vecyR.push_back(y1);
          vecyR.push_back(y2);
        }
        else if (clasif[i] == 1) {
          LeftLine[0] = (x1 + LeftLine[0]) / 2;
          LeftLine[1] = (y1 + LeftLine[1]) / 2;
          LeftLine[2] = (x2 + LeftLine[2]) / 2;
          LeftLine[3] = (y2 + LeftLine[3]) / 2;
          vecxL.push_back(x1);
          vecxL.push_back(x2);
          vecyL.push_back(y1);
          vecyL.push_back(y2);
        }
      }

      if( (LeftLine[0]+ LeftLine[1]+ LeftLine[2]+ LeftLine[3])==0){
        LeftLine[0] = LastLeftLine[0];
        LeftLine[1] = LastLeftLine[1];
        LeftLine[2] = LastLeftLine[2];
        LeftLine[3] = LastLeftLine[3];
      }

      if((RightLine[0]+ RightLine[1]+ RightLine[2]+ RightLine[3])==0){
        RightLine[0] = LastRightLine[0];
        RightLine[1] = LastRightLine[1];
        RightLine[2] = LastRightLine[2];
        RightLine[3] = LastRightLine[3];
      }

      m1 = (LeftLine[3]-LeftLine[1])/(LeftLine[2]-LeftLine[0]);
      b1_int = LeftLine[1] - (m1*LeftLine[0]);

      m2 = (RightLine[3]-RightLine[1])/(RightLine[2]-RightLine[0]);
      b2_int = RightLine[1] - (m2*RightLine[0]);

      int x_h = (b2_int - b1_int)/(m1 - m2);
      ya = m1*x_h + b1_int;

      xa1 = x_h;
      xb1 = (yb - b1_int) / m1;

      LeftLine[0] = xa1;
      LeftLine[1] = ya;
      LeftLine[2] = xb1;
      LeftLine[3] = yb;

      //Then, the right line
      xa2 = x_h;
      xb2 = (yb - b2_int) / m2;

      RightLine[0] = xa2;
      RightLine[1] = ya;
      RightLine[2] = xb2;
      RightLine[3] = yb;

      xc1=LeftLine[0]+(RightLine[0]-LeftLine[0])/2;
      xc2=LeftLine[2]+(RightLine[2]-LeftLine[2])/2;

      cv::cvtColor(img, img, CV_GRAY2BGR);

      cv::line( img, cv::Point(LeftLine[0],LeftLine[1]), cv::Point(LeftLine[2],LeftLine[3]), cv::Scalar(0,255,0), 2, 8 );
      //Draw right lane line
      cv::line( img, cv::Point(RightLine[0],RightLine[1]), cv::Point(RightLine[2],RightLine[3]), cv::Scalar(0,255,0), 2, 8 );
      //Draw center lane line
      cv::line( img, cv::Point(xc1, ya), cv::Point(xc2,yb), cv::Scalar(255,0,0), 2, 8 );
      //Draw horizon line
      cv::line( img, cv::Point(0, ya), cv::Point(img.cols,ya), cv::Scalar(0,0,255), 2, 8 );
      //Draw camera's vertical center line
      cv::line( img, cv::Point(img.cols/2, 0), cv::Point(img.cols/2, yb), cv::Scalar(0,0,255), 2, 8 );


      return img;


    }

};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Lines_Node");
  ROS_INFO("Line detector online");
  Line_Detector ic;
  ros::spin();
  return 0;
}