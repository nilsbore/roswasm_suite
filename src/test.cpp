#include <roscpp_json/serialize.h>
#include <sensor_msgs/NavSatFix.h>
#include <rosmon_msgs/State.h>

void print_navsat()
{
    sensor_msgs::NavSatFix msg;
    msg.header.frame_id = "base_link";
    roscpp_json::JSONStream stream;
    ros::message_operations::Printer<sensor_msgs::NavSatFix>::stream(stream, "", msg);
    std::cout << stream.str() << std::endl;
    std::stringstream sstream;
    ros::message_operations::Printer<sensor_msgs::NavSatFix>::stream(sstream, "", msg);
    std::cout << sstream.str() << std::endl;
}

void print_rosmon()
{
    rosmon_msgs::State msg;
    msg.nodes.resize(2);
    roscpp_json::JSONStream stream;
    ros::message_operations::Printer<rosmon_msgs::State>::stream(stream, "", msg);
    std::cout << stream.str() << std::endl;
    std::stringstream sstream;
    ros::message_operations::Printer<rosmon_msgs::State>::stream(sstream, "", msg);
    std::cout << sstream.str() << std::endl;
}

int main(int argc, char** argv)
{
    print_navsat();
    print_rosmon();
    return 0;
}
