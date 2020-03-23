#include <stdio.h>

#include <emscripten.h>

#include <iostream>
#include <roswasm/roswasm.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>

roswasm::NodeHandle* nh; 
roswasm::Subscriber* string_sub;
roswasm::Subscriber* gps_sub;
roswasm::Publisher* string_pub;


void string_callback(const std_msgs::String& msg)
{
    printf("Got string message in callback: %s\n", msg.data.c_str());
}

void gps_callback(const sensor_msgs::NavSatFix& msg)
{
    printf("Got string message in callback: %s\n", msg.header.frame_id.c_str());
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

  emscripten_set_main_loop(loop, 1, 1);

  return 0;
}
