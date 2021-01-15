#ifndef ROSWASM_NATIVE_H
#define ROSWASM_NATIVE_H

#include <ros/ros.h>

namespace roswasm {

    using NodeHandle = ros::NodeHandle;
    using Publisher = ros::Publisher;
    using Subscriber = ros::Subscriber;
    using Timer = ros::Timer;
    using Time = ros::Time;
    using Duration = ros::Duration;
    //using init = ros::init;
    void init(int argc, char** argv, const std::string& arg);

} // namespace roswasm

#endif // ROSWASM_NATIVE_H
