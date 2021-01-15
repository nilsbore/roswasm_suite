#include <roswasm/roswasm.h>

#ifdef ROSWASM_NATIVE

#include <ros/ros.h>

namespace roswasm {

void init(int argc, char** argv, const std::string& arg)
{
    ros::init(argc, argv, arg);
}

} // namespace roswasm
    
#else
#include <roswasm/roswasm.hpp>
#include <roswasm/timer.hpp>
#endif
