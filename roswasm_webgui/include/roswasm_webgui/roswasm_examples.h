#ifndef ROSWASM_EXAMPLES_H
#define ROSWASM_EXAMPLES_H

#include <roswasm_webgui/roswasm_widget.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>

namespace roswasm_webgui {

class ExampleActuatorWidget {
private:
    TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>* thruster_angles;
    TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>* thruster_rpms;
    roswasm::Publisher rpm_pub;
    roswasm::Timer pub_timer;
    bool rpm_pub_enabled;
    TopicWidget<std_msgs::Float32>* lcg_actuator;
    TopicWidget<std_msgs::Bool>* lcg_control_enable;
    TopicWidget<std_msgs::Float32>* lcg_control_setpoint;
    TopicWidget<std_msgs::Float32>* vbs_actuator;
    TopicWidget<std_msgs::Bool>* vbs_control_enable;
    TopicWidget<std_msgs::Float32>* vbs_control_setpoint;
    TopicWidget<std_msgs::Float32>* tcg_actuator;
    TopicWidget<std_msgs::Bool>* tcg_control_enable;
    TopicWidget<std_msgs::Float32>* tcg_control_setpoint;

public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_actuator_window);
    ExampleActuatorWidget(roswasm::NodeHandle& nh);
};

class ExampleDashboardWidget {
private:
    bool was_leak;
    TopicBuffer<std_msgs::Bool>* leak;
    TopicBuffer<sensor_msgs::NavSatFix>* gps;
    TopicBuffer<sensor_msgs::BatteryState>* battery;
    TopicBuffer<nav_msgs::Odometry>* odom;
    TopicBuffer<std_msgs::Float32>* vbs;
    TopicBuffer<std_msgs::Float32>* lcg;
    TopicBuffer<std_msgs::Float64>* depth;
    TopicBuffer<std_msgs::Float64>* pitch;
    TopicBuffer<std_msgs::Float64>* roll;
    TopicBuffer<std_msgs::Float64>* yaw;
public:
    bool is_emergency() { return was_leak; }
    void show_window(bool& show_dashboard_window);
    ExampleDashboardWidget(roswasm::NodeHandle& nh);
};

class ExampleTeleopWidget {
private:
    bool enabled;
    geometry_msgs::Pose2D angles_msg;
    geometry_msgs::Pose2D rpm_msg;
    roswasm::Publisher rpm_pub;
    roswasm::Publisher angle_pub;
    roswasm::Timer pub_timer;
public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_teleop_window);
    ExampleTeleopWidget(roswasm::NodeHandle& nh);
};

} // namespace roswasm_webgui

#endif // ROSWASM_EXAMPLES_H
