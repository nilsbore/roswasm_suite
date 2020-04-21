#ifndef ROSWASM_EXAMPLES_H
#define ROSWASM_EXAMPLES_H

#include <roswasm_widget.h>

namespace roswasm_webgui {

class ExampleActuatorWidget {
private:
    TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>* thruster_angles;
    TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>* thruster_rpms;
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
    void show_window(bool& show_actuator_window);
    ExampleActuatorWidget(roswasm::NodeHandle* nh);
};

} // namespace roswasm_webgui

#endif // ROSWASM_EXAMPLES_H
