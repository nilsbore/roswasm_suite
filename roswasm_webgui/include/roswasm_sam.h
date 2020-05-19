#ifndef ROSWASM_SAM_H
#define ROSWASM_SAM_H

#include <roswasm_widget.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>

#include <sam_msgs/PercentStamped.h>
#include <sam_msgs/ThrusterRPMs.h>
#include <sam_msgs/ThrusterAngles.h>
#include <sam_msgs/BallastAngles.h>
#include <sam_msgs/Leak.h>

namespace roswasm_webgui {

bool draw_ballast_angles(sam_msgs::BallastAngles& msg, roswasm::Publisher* pub);
bool draw_percent(sam_msgs::PercentStamped& msg, roswasm::Publisher* pub);
bool draw_thruster_rpms(sam_msgs::ThrusterRPMs& msg, roswasm::Publisher* pub);
bool draw_thruster_angles(sam_msgs::ThrusterAngles& msg, roswasm::Publisher* pub);

class SamActuatorWidget {
private:
    TopicWidget<sam_msgs::ThrusterAngles>* thruster_angles;
    TopicWidget<sam_msgs::ThrusterRPMs>* thruster_rpms;
    roswasm::Publisher* rpm_pub;
    roswasm::Timer* pub_timer;
    bool rpm_pub_enabled;
    TopicWidget<sam_msgs::PercentStamped>* lcg_actuator;
    TopicWidget<std_msgs::Bool>* lcg_control_enable;
    TopicWidget<std_msgs::Float64>* lcg_control_setpoint;
    TopicWidget<sam_msgs::PercentStamped>* vbs_actuator;
    TopicWidget<std_msgs::Bool>* vbs_control_enable;
    TopicWidget<std_msgs::Float64>* vbs_control_setpoint;
    TopicWidget<sam_msgs::BallastAngles>* tcg_actuator;
    TopicWidget<std_msgs::Bool>* tcg_control_enable;
    TopicWidget<std_msgs::Float64>* tcg_control_setpoint;

public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_actuator_window);
    SamActuatorWidget(roswasm::NodeHandle* nh);
};

class SamDashboardWidget {
private:
    bool was_leak;
    TopicBuffer<sam_msgs::Leak>* leak;
    TopicBuffer<sensor_msgs::NavSatFix>* gps;
    TopicBuffer<sensor_msgs::BatteryState>* battery;
    TopicBuffer<nav_msgs::Odometry>* odom;
    TopicBuffer<sam_msgs::PercentStamped>* vbs;
    TopicBuffer<sam_msgs::PercentStamped>* lcg;
    TopicBuffer<sam_msgs::ThrusterRPMs>* rpms;
    TopicBuffer<std_msgs::Float64>* depth;
    TopicBuffer<std_msgs::Float64>* pitch;
    TopicBuffer<std_msgs::Float64>* roll;
    TopicBuffer<std_msgs::Float64>* yaw;
public:
    bool is_emergency() { return was_leak; }
    void show_window(bool& show_dashboard_window);
    SamDashboardWidget(roswasm::NodeHandle* nh);
};

class SamTeleopWidget {
private:
    bool enabled;
    sam_msgs::ThrusterAngles angles_msg;
    sam_msgs::ThrusterRPMs rpm_msg;
    roswasm::Publisher* rpm_pub;
    roswasm::Publisher* angle_pub;
    roswasm::Timer* pub_timer;
public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_teleop_window);
    SamTeleopWidget(roswasm::NodeHandle* nh);
};

} // namespace roswasm_webgui

#endif // ROSWASM_EXAMPLES_H
