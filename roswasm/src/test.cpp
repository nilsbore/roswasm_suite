#include <stdio.h>

#include <iostream>
#include <roswasm/roswasm.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <rosapi/TopicType.h>

roswasm::NodeHandle* nh; 
roswasm::Subscriber string_sub;
roswasm::Subscriber gps_sub;
roswasm::Publisher string_pub;
roswasm::ServiceCallbackClient service;
roswasm::Timer timer;
roswasm::Time previous;

void string_callback(const std_msgs::String& msg)
{
    printf("Got string message in callback: %s\n", msg.data.c_str());
}

void gps_callback(const sensor_msgs::NavSatFix& msg)
{
    printf("Got string message in callback: %s\n", msg.header.frame_id.c_str());
}

void service_callback(const rosapi::TopicType::Response& res, bool result)
{
    printf("Got service response with value: %s\n", res.type.c_str());
}

void timer_callback(const ros::TimerEvent& ev)
{
    printf("Got timer callback!\n");
    roswasm::Time t = roswasm::Time::now();
    printf("Got time: %d sec, %d, nsec\n", t.sec, t.nsec);
    roswasm::Duration diff = t - previous;
    printf("Time since last: %f\n", diff.toSec());
    previous = t;
}

void loop()
{
    std_msgs::String msg;
    msg.data = "LOOPING";
    string_pub.publish(msg);
}

extern "C" int main(int argc, char** argv)
{
    roswasm::init(argc, argv, "test");
    //nh = roswasm::NodeHandle("test");
    nh = new roswasm::NodeHandle();
    string_sub = nh->subscribe("test", 1000, string_callback);
    gps_sub = nh->subscribe("test2", 1000, gps_callback);
    string_pub = nh->advertise<std_msgs::String>("test", 1000);
    //service = nh->serviceCallbackClient<rosapi::TopicType>("/rosapi/topic_type", service_callback);
    service = roswasm::createServiceCallbackClient<rosapi::TopicType>(*nh, "/rosapi/topic_type");
    rosapi::TopicType::Request req;
    req.topic = "/connected_clients";
    service.call<rosapi::TopicType>(req, service_callback);
    previous = roswasm::Time::now();
    timer = nh->createTimer(roswasm::Duration(5.), timer_callback);

    roswasm::Duration loop_rate(1.);
    roswasm::spinLoop(loop, loop_rate);

/*
#ifdef ROSWASM_NATIVE
    ros::spin();
#else
    emscripten_set_main_loop(loop, 1, 1);
#endif
*/

    return 0;
}
