#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageSkeleton{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher skeleton_slope_pub_;

public:
  ImageSkeleton()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image_raw", 1, &ImageSkeleton::skeletonCallback, this);
    image_pub_ = it_.advertise("/image_skeleton", 1);
    skeleton_slope_pub_ = nh_.advertise<std_msgs::Float64>("/slope", 1);


    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageSkeleton()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void skeletonCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      ROS_INFO_STREAM(msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::normalize(cv_ptr->image, cv_ptr->image, 1, 0, cv::NORM_MINMAX);

    cv::blur(cv_ptr->image,cv_ptr->image,cv::Size(10,10));
   // cv::Mat img;
    cv::threshold(cv_ptr->image, cv_ptr->image, 0.1, 1, cv::THRESH_BINARY); 

    
    cv::Mat skel(cv_ptr->image.size(), CV_32FC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;
 
    cv::Mat dilateelement = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::Mat erodeelement = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(4 , 4));

    bool done;    
    do
    {
     cv::erode(cv_ptr->image, eroded, dilateelement);
     cv::dilate(eroded, temp, dilateelement); // temp = open(img)
     cv::subtract(cv_ptr->image, temp, temp);
     cv::bitwise_or(skel, temp, skel);
     eroded.copyTo(cv_ptr->image);
     done = (cv::countNonZero(cv_ptr->image) == 0);

   } while (!done);


    //*/

    // Draw an example circle on the video stream
  //  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  //    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    
    cv::Mat skel_line;
    cv::Mat skel_line_;
    cv::Mat skel_line_t;

    skel.convertTo(skel_line, CV_8UC1,255.0);
 //   cv::Mat cropped_skel = skel_line(cv::Rect(50,50,200,200));    
    //cv::medianBlur(skel_line,skel_line_,5);
    //cv::dilate(skel_line, skel_line_, dilateelement);
    //cv::erode(skel_line_, skel_line_t, erodeelement);
 /*    
    cv::Mat lines;
   cv::HoughLines(skel_line, lines, 2, CV_PI/180,200,0,0);
   
    for(int i = 0; i < lines.rows; i++)
  {
    const double* Mi = lines.ptr<double>(i);
    for(int j = 0; j < lines.cols; j= j+2){

     double rho = Mi[j];
     double theta = Mi[j+1];
     cv::Point pt1;
     cv::Point pt2;
     float a = cos(theta);
     float b = sin(theta);
     float x0 = a*rho;
     float y0 = b*rho;
     cv::line( skel_line, cv::Point(x0 + 1000*(-b),y0 + 1000*(a)), cv::Point(x0 - 1000*(-b),y0 - 1000*(a)), cv::Scalar(255,0,0), 3, CV_AA);
    ROS_INFO("Lines printed");

    }
  }

  //*/
   
    cv::Vec4f lines;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > prunedContours;

   cv::findContours(skel_line,contours,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);

    for (std::size_t i = 0; i< contours.size(); i++)
     {
         if (cv::contourArea(contours[i]) > 10)
         {
           prunedContours.push_back(contours[i]);
         }
     }
   cv::fitLine(cv::Mat(prunedContours[0]),lines,CV_DIST_L2,0,0.01,0.01);

    //const double* M = lines.ptr<double>(0);

    //lefty = int((-x*vy/vx) + y)
    //righty = int(((gray.shape[1]-x)*vy/vx)+y)
    //int lefty = (-M[2]*M[1]/M[0])+M[3];
    //int righty = ((skel_line.cols-M[2])*M[1]/M[0])+M[3];
    int lefty = (-lines[2]*lines[1]/lines[0])+lines[3];
    int righty = ((skel_line.cols-lines[2])*lines[1]/lines[0])+lines[3];

    cv::line(skel_line,cv::Point(skel_line.cols-1,righty),cv::Point(0,lefty),cv::Scalar(255,0,0),2);
   // cv::line(skel_line,cv::Point(lines[3] + 5*lines[0],lines[4] + 5*lines[1]),cv::Point(lines[3] ,lines[4]),cv::Scalar(255,0,0),2);

/*    float rho = lin[0][j];
     float theta = lin[0][j];;
     cv::Point pt1;
     cv::Point pt2;

     float a = cos(theta);

     float b = sin(theta);
     float x0 = a*rho;
     float y0 = b*rho;
     cv::line( skel_line, cv::Point(x0 + 1000*(-b),y0 + 1000*(a)), cv::Point(x0 - 1000*(-b),y0 - 1000*(a)), cv::Scalar(0,0,255), 3, CV_AA);
 // }
    
//*/
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, skel_line);
    cv::waitKey(3);

    std_msgs::Float64 slope;
    slope.data = -lines[1]/lines[0];
    

    skeleton_slope_pub_.publish(slope);

    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skeleton_tracker_node");
  ImageSkeleton ic;
  ros::spin();
  return 0;
}