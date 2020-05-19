#include <stdio.h>

#include <emscripten.h>

#include <iostream>
#include <roswasm/roswasm.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <rosapi/TopicType.h>

roswasm::NodeHandle* nh; 
roswasm::Subscriber* string_sub;
roswasm::Subscriber* gps_sub;
roswasm::Publisher* string_pub;
roswasm::ServiceClient* service;
roswasm::Timer* timer;

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
}

void loop()
{
    std_msgs::String msg;
    msg.data = "LOOPING";
    string_pub->publish(msg);
}

extern "C" int main(int argc, char** argv)
{
  nh = new roswasm::NodeHandle();
  string_sub = nh->subscribe<std_msgs::String>("test", string_callback);
  gps_sub = nh->subscribe<sensor_msgs::NavSatFix>("test2", gps_callback);
  string_pub = nh->advertise<std_msgs::String>("test");
  service = nh->serviceClient<rosapi::TopicType>("/rosapi/topic_type", service_callback);
  rosapi::TopicType::Request req;
  req.topic = "/connected_clients";
  service->call<rosapi::TopicType>(req);
  timer = nh->createTimer(5., timer_callback);


  emscripten_set_main_loop(loop, 1, 1);

  return 0;
}
