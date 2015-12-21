#include <vector>
#include <stdio.h>
#include <string>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <video_capture/snapshot.h>

using namespace cv;

int count = 0;

namespace patch
{
    template < typename T > std::string to_string(const T& n)
    {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
}

bool take_picture(video_capture::snapshot::Request& req,
                    video_capture::snapshot::Response& res)
{
    //take picture and save as .pgm
    VideoCapture cap(0);
    if(!cap.isOpened())
    {
        return false;
    }
    std::cout << "Capture is open" << std::endl;
    count++;
    Mat frame, edges;
    cap >> frame;
    cvtColor(frame, edges, CV_BGR2GRAY);
    
    std::string under=patch::to_string(req.id) + "_";
    std::string path="/opt/ros/indigo/catkin_ws/src/video_capture/src/data/" + under + (string)req.name + patch::to_string(count) + ".pgm"; 
    std::cout << "writing image in: " << path << std::endl;
    if (imwrite(path,edges))
    {
        std::cout << "image written" << std::endl;
        res.result=true;
        return res.result;
    }
    res.result=false;
    return res.result;
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"video_capture_server");
    ros::NodeHandle nh;
    ros::ServiceServer service=nh.advertiseService("take_picture",take_picture);
    namedWindow("take a picture",1);
    ROS_INFO("Ready to take picture");
    ros::spin();
    
    return 0;
}

/********************************************
int main(int argc, char** argv)
{
    ros::init(argc,argv,"video_capture");
    ros::NodeHandle nh;
    ros::Publisher video_pub=nh.advertise<sensor_msgs::Image>("/video_stream", 10);
    ros::Rate loop_rate(10);
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("image",1);
    cv_bridge::CvImage out_msg;
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, edges, CV_BGR2GRAY);
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC4;
        out_msg.image = edges;
        video_pub.publish(out_msg);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        //imshow("edges", edges);
        imshow("image",edges);
        
        if(waitKey(30) >= 0) break;
    }
}
***********************************************/
