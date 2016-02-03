#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
//#include "curl/curl.h"

/*************************************************
//CURL * curl;
//ros::Publisher listen_pub_;
//ros::Subscriber listen_sub_;
std::string recording="/opt/ros/indigo/stacks/src/qbo_listener/src/temp.flac";
std::string key="AIzaSyCmwVC1h5TasF6NhCV2CwZUq4bFLRrCx9A";
std::string alt_key="AIzaSyAcalCzUvPmmJ7CZBFOEWx2Z1ZSn4Vs1gg";
std::string lang="it-IT";

void callback(const std_msgs::String::ConstPtr& msg)
{
	curl = curl_easy_init();
    
    ROS_INFO("Recording audio");
    ROS_INFO("Audio recorded\nCalling google service...");
    std::string ss="wget -q --post-file ";
    ss += recording;

    try
    {
//    	r=RestClient::get("http://www.google.com/speech-api/v2/recognize?client=chromium&lang="+lang+"&key="+key, headers);
        //resource = r.get(ss + " --haeder='Content-Type: audio/x-flac; rate=16000' -O - 'http://www.google.com/speech-api/v2/recognize?client=chromium&lang="+lang+"&key="+key+"'");

    }
    catch (std::exception e)
    {
    	std::cout << "An exception occurred. Exception Nr. " << e.what() << '\n';
    	return;
    }

    
    //result=system("wget -q --post-file /opt/ros/indigo/stacks/src/qbo_listener/src/temp.flac --header='Content-Type: audio/x-flac; rate=16000' -O - 'http://www.google.com/speech-api/v2/recognize?client=chromium&lang=it-IT&key=AIzaSyCmwVC1h5TasF6NhCV2CwZUq4bFLRrCx9A' ");
    //std::cout<<"cout result: "<<result<<std::endl;
    //ROS_INFO("RESULT= %s",result.c_str());
    ROS_INFO("Exiting callback");
}

int main(int argc, char**argv)
{
	curl_global_init(CURL_GLOBAL_ALL);
    ros::init(argc, argv, "qbo_listener");
    ros::NodeHandle n;

//    listen_sub_=n.subscribe("key_pressed",1,callback);
    ros::Subscriber listen_sub=n.subscribe("key_pressed",1,callback);
    ros::spin();
    curl_global_cleanup();
//    listen_pub_=n.advertise<std_msgs::String>("listened",1);
//    ros::Rate loop_rate(10);

}


**********************************************/



/**************************************
RESULT=`wget -q --post-file $INFILE --header="Content-Type: audio/x-flac; rate=$SRATE" -O - "http://www.google.com/speech-api/v2/recognize?client=chromium&lang=$LANGUAGE&key=$KEY"`
 
FILTERED=`echo "$RESULT" | grep "transcript.*}" | sed 's/,/\n/g;s/[{,},"]//g;s/\[//g;s/\]//g;s/:/: /g' | grep -o -i -e "transcript.*" -e "confidence:.*"`

rec -V output.wav silence 0 1 0:00:03 2% silence 0 1 00:00:05 0% //stop registrazione dopo 3 secondi di silenzio

wget -q --post-file /opt/ros/indigo/stacks/src/qbo_listener/src/temp.flac --header="Content-Type: audio/x-flac; rate=16000" -O /opt/ros/indigo/stacks/src/qbo_listener/src/output.txt - "http://www.google.com/speech-api/v2/recognize?client=chromium&lang=it-IT&key=AIzaSyCmwVC1h5TasF6NhCV2CwZUq4bFLRrCx9A"



********************************/
