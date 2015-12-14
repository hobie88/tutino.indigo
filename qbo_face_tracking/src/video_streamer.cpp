#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <sensor_msgs/image_encodings.h>

/******* CALLBACK TO TEST VIDEO STREAMING *************
void viewer_callback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
	cv::namedWindow("Image viewer", cv::WINDOW_AUTOSIZE);
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);
	imshow("Image viewer",cv_ptr->image);
}
*******************************************/

int main(int argc, char** argv)
{
//  if(argv[1]==NULL) return 1;
  
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/stereo/left/image_raw", 1);
  ros::Publisher infoPub = nh.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info",1);
//  ros::Subscriber subImage = nh.subscribe("/stereo/left/image_raw",1,viewer_callback); // test video stream


//  std::istringstream video_sourceCmd(argv[1]);
//  int video_source;
//  if(!(video_sourceCmd >> video_source)) return 1;
  
  cv::VideoCapture cap;
  cap.open(0);
  
  if(!cap.isOpened())
  {
     std::cout<<"video capture not opened"<<std::endl;
     return 1;
  }
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;
  sensor_msgs::CameraInfo info;

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    cap>>frame;
    if(!frame.empty())
    {   
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        info.header.stamp = ros::Time::now();
        infoPub.publish(info);
        pub.publish(msg);
        cv::waitKey(1);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}


