#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <video_capture/snapshot.h>



int main(int argc, char* argv[])
{
    ros::init(argc,argv,"save_image");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<video_capture::snapshot>("take_picture");
    video_capture::snapshot srv;
    int num=0;
    std::cout<<"Inserisci il nome da registrare: "<<std::endl;
    std::cin >> srv.request.name;
    std::cout<<"E il numero di foto da salvare"<<std::endl;
    std::cin >> num;
    for (int i=0; i<num ; i++)
    {
        if (client.call(srv))
        {
            if (srv.response.result)
                    ROS_INFO("Taking picture");
            else ROS_ERROR("Servive Error");
        }
        else
        {
         ROS_ERROR("Impossibile comunicare con il server");
            return 1;
        }
        ros::Duration(0.5).sleep();
    }
    
    return 0;
}
