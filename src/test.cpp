#include <roscpp_json/serialize.h>
#include <roscpp_json/deserialize.h>
#include <sensor_msgs/NavSatFix.h>
#include <rosmon_msgs/State.h>

bool service_list = false;

template <typename MSG>
void test_serialization(const MSG& msg)
{
    std::string json_msg = roscpp_json::serialize(msg, service_list);
    std::cout << json_msg << std::endl;

    std::stringstream sstream;
    ros::message_operations::Printer<MSG>::stream(sstream, "", msg);
    std::string original_def = sstream.str();
    std::cout << original_def << std::endl;

    MSG parsed_msg = roscpp_json::deserialize<MSG>(json_msg, service_list);

    sstream.str("");
    ros::message_operations::Printer<MSG>::stream(sstream, "", parsed_msg);
    std::string parsed_def = sstream.str();
    std::cout << parsed_def << std::endl;

    assert(original_def == parsed_def);
}

void print_navsat()
{
    sensor_msgs::NavSatFix msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    msg.position_covariance[5] = 3.;
    msg.status.status = -1;
    test_serialization(msg);
}

void print_rosmon()
{
    rosmon_msgs::State msg;
    msg.header.frame_id = "base_link";
    msg.nodes.resize(2);
    msg.nodes[0].ns = "test1";
    msg.nodes[1].name = "test2";
    test_serialization(msg);
}

int main(int argc, char** argv)
{
    print_navsat();
    print_rosmon();
    return 0;
}
