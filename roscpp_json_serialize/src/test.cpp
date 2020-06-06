#include <roscpp_json_serialize/serialize.h>
#include <roscpp_json_serialize/deserialize.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <gtest/gtest.h>

/*
#include <rosmon_msgs/State.h>

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
*/

template <typename MSG>
std::pair<std::string, std::string> test_serialization(const MSG& msg)
{
    bool service_list = false;

    std::string json_msg = roscpp_json::serialize(msg, service_list);

    std::stringstream sstream;
    sstream.precision(17);
    ros::message_operations::Printer<MSG>::stream(sstream, "", msg);
    std::string original_def = sstream.str();

    //std::cout << original_def << std::endl;
    //std::cout << json_msg << std::endl;

    MSG parsed_msg = roscpp_json::deserialize<MSG>(json_msg, service_list);

    sstream.str("");
    ros::message_operations::Printer<MSG>::stream(sstream, "", parsed_msg);
    std::string parsed_def = sstream.str();

    //assert(original_def == parsed_def);
    return make_pair(original_def, parsed_def);
}

TEST(JsonSerialization, stdString)
{
    std_msgs::String msg;
    msg.data = "testing";
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, stdFloat32)
{
    std_msgs::Float32 msg;
    msg.data = 3.14f;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, precisionFloat32)
{
    std_msgs::Float32 msg;
    msg.data = 3.14159265359f;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, nanFloat32)
{
    std_msgs::Float32 msg;
    msg.data = std::numeric_limits<float>::quiet_NaN();
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, infFloat32)
{
    std_msgs::Float32 msg;
    msg.data = std::numeric_limits<float>::infinity();
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, negInfFloat32)
{
    std_msgs::Float32 msg;
    msg.data = -std::numeric_limits<float>::infinity();
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, nanFloat64)
{
    std_msgs::Float64 msg;
    msg.data = std::numeric_limits<double>::quiet_NaN();
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, infFloat64)
{
    std_msgs::Float64 msg;
    msg.data = std::numeric_limits<double>::infinity();
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, negInfFloat64)
{
    std_msgs::Float64 msg;
    msg.data = -std::numeric_limits<double>::infinity();
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, stdFloat64)
{
    std_msgs::Float64 msg;
    msg.data = 3.14;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, precisionFloat64)
{
    std_msgs::Float64 msg;
    msg.data = 3.14159265359f;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, stdBool)
{
    std_msgs::Bool msg;
    msg.data = true;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, stdUint8)
{
    std_msgs::UInt8 msg;
    msg.data = 255;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, stdInt32)
{
    std_msgs::Int32 msg;
    msg.data = -1;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
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

TEST(JsonSerialization, image)
{
    sensor_msgs::Image msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    msg.data.resize(100);
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, pointCloud2)
{
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    msg.fields.resize(3);
    msg.data.resize(100);
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, emptyPointCloud2)
{
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    msg.fields.resize(3);
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, imu)
{
    sensor_msgs::Imu msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    msg.orientation_covariance[5] = 3.;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, fluidPressure)
{
    sensor_msgs::FluidPressure msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    msg.fluid_pressure = 3.14;
    msg.variance = 100.;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, poseWithCovarianceStamped)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, accelWithCovarianceStamped)
{
    geometry_msgs::AccelWithCovarianceStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, occupancyGrid)
{
    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    msg.data.resize(100);
    msg.data[23] = 255;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, odometry)
{
    nav_msgs::Odometry msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, path)
{
    nav_msgs::Path msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    msg.poses.resize(100);
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

TEST(JsonSerialization, emptyPath)
{
    nav_msgs::Path msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp.nsec = 4;
    msg.header.stamp.sec = 3;
    std::string original_def, parsed_def;
    tie(original_def, parsed_def) = test_serialization(msg);
    EXPECT_EQ(original_def, parsed_def);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
