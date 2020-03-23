#include <stdio.h>

#include <emscripten.h>

#include <iostream>
#include <roswasm/roswasm.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>

roswasm::NodeHandle* nh; 
roswasm::Subscriber* string_sub;
roswasm::Subscriber* gps_sub;

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

}

extern "C" int main(int argc, char** argv)
{
  nh = new roswasm::NodeHandle();
  string_sub = nh->subscribe<std_msgs::String>("test", string_callback);
  gps_sub = nh->subscribe<sensor_msgs::NavSatFix>("test2", gps_callback);

  emscripten_set_main_loop(loop, 60, 1);

  return 0;
}
