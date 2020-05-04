#include <roscpp_json_serialize/serialize.h>
#include <roscpp_json_serialize/deserialize.h>
#include <sensor_msgs/NavSatFix.h>
#include <rosmon_msgs/State.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

template <typename MSG>
std::pair<std::string, std::string> test_serialization(const MSG& msg)
{
    bool service_list = false;

    std::string json_msg = roscpp_json::serialize(msg, service_list);

    std::stringstream sstream;
    ros::message_operations::Printer<MSG>::stream(sstream, "", msg);
    std::string original_def = sstream.str();

    MSG parsed_msg = roscpp_json::deserialize<MSG>(json_msg, service_list);

    sstream.str("");
    ros::message_operations::Printer<MSG>::stream(sstream, "", parsed_msg);
    std::string parsed_def = sstream.str();

    //assert(original_def == parsed_def);
    return make_pair(original_def, parsed_def);
}

TEST(JsonSerialization, navSatFix)
{
    sensor_msgs::NavSatFix msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    msg.position_covariance[5] = 3.;
    msg.status.status = -1;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, rosmonState)
{
    rosmon_msgs::State msg;
    msg.header.frame_id = "base_link";
    msg.nodes.resize(2);
    msg.nodes[0].ns = "test1";
    msg.nodes[1].name = "test2";
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
