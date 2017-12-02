#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "Histogram.h"

static const std::string OPENCV_WINDOW = "Depth Image window";
// define a class
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/zed/depth/depth_registered", 1,
      &ImageConverter::depthCb, this);
    image_pub_ = it_.advertise("/output", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

private:
int window_size = 200;
void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    double min; double max;
    cv::minMaxIdx(cv_ptr->image,&min,&max);
    cv::Mat adjMap;
    cv::convertScaleAbs(cv_ptr->image,adjMap,255/max); 

    // histogram processing
    cv::Mat depth_img1=cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());

    //cv::Mat depth_img2=cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
    //  cv::threshold(cv_ptr->image,depth_img1,2000,2000,cv::THRESH_TRUNC);

    cv::threshold(cv_ptr->image,depth_img1,2000,1,cv::THRESH_BINARY_INV);
    cv::Mat imageROI;
	for(int i=0;i < ((depth_img1.cols)/320)-1;i++){
	   for(int j=0;j<((depth_img1.rows)/240)-1;j++){
    		start_time = ros::Time::now();
		cv::Rect region = cv::Rect(i*320,j*240,320,240);
		imageROI = depth_img1(region);
		double s = cv::sum(imageROI)[0];
		//std::cout<< i+1<<","<<j+1<<":"<<s<< " ";
		//std::cout<<depth_img1.rows<<" ";
		}
	}
	//ros::Duration elapsed_time = ros::Time::now()-start_time;
	//double secs= elapsed_time.toSec();
	//std::cout<<secs<<" ";
	//Histogram1D h1; Histogram1D h2;
	//normalize
	/*double min_2m; double max_2m;
        cv::minMaxIdx(depth_img1,&min_2m,&max_2m);
        cv::Mat adjMap_2m;
        cv::convertScaleAbs(depth_img1,adjMap_2m,255/max_2m); 


*/  
    
    //std::cout<<std::endl; 
    //cv::imshow(OPENCV_WINDOW, adjMap);
    cv::imshow("2m distance depth image", depth_img1);
	//cv::imshow("5m distance depth image", depth_img2);
	//cv::imshow("histogram 2000 mm",	h1.getHistogramImage(adjMap_2m));
	//cv::imshow("histogram 5000 mm",	h2.getHistogramImage(depth_img1));
    cv::waitKey(3);

    // Output modified video stream depth_img1
    //image_pub_.publish(depth_img1.toImageMsg());
  }
};

int main(int argc,char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0; 
}
