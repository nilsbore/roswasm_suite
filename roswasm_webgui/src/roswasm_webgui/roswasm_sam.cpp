#include <roswasm_sam.h>

#include "imgui.h"

namespace roswasm_webgui {

bool draw_ballast_angles(sam_msgs::BallastAngles& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("Angles cmd slider");
    ImGui::SliderFloat("", &msg.weight_1_offset_radians, -1.6f, 1.6f, "%.2frad");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.weight_2_offset_radians = msg.weight_1_offset_radians;
        pub->publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("Angles cmd input", &msg.weight_1_offset_radians, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.weight_2_offset_radians = msg.weight_1_offset_radians;
        pub->publish(msg);
    }

    return false;
}

bool draw_percent(sam_msgs::PercentStamped& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("Percent cmd slider");
    ImGui::SliderFloat("", &msg.value, 0.0f, 100.0f, "%.2f%%");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    bool lock = ImGui::IsItemActive();

    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("Percent cmd input", &msg.value, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    return lock;
}

bool draw_thruster_rpms(sam_msgs::ThrusterRPMs& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("First cmd slider");
    ImGui::Text("Thruster 1");
    ImGui::SameLine();
    ImGui::SliderInt("", &msg.thruster_1_rpm, -1000, 1000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    bool lock = ImGui::IsItemActive();
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("First cmd input", &msg.thruster_1_rpm);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    ImGui::PushID("Second cmd slider");
    ImGui::Text("Thruster 2");
    ImGui::SameLine();
    ImGui::SliderInt("", &msg.thruster_2_rpm, -1000, 1000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    lock = lock || ImGui::IsItemActive();
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("Second cmd input", &msg.thruster_2_rpm);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    return lock;
}

bool draw_thruster_angles(sam_msgs::ThrusterAngles& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("First cmd slider");
    ImGui::Text("Hori (rad)");
    ImGui::SameLine();
    ImGui::SliderFloat("", &msg.thruster_horizontal_radians, -0.1f, 0.18f, "%.2frad");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("First cmd input", &msg.thruster_horizontal_radians, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    ImGui::PushID("Second cmd slider");
    ImGui::Text("Vert (rad)");
    ImGui::SameLine();
    ImGui::SliderFloat("", &msg.thruster_vertical_radians, -0.1f, 0.15f, "%.2frad");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("Second cmd input", &msg.thruster_vertical_radians, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    return false;
}

SamActuatorWidget::SamActuatorWidget(roswasm::NodeHandle* nh) : rpm_pub_enabled(false), pub_timer(nullptr)
{
    //thruster_angles = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Hori (rad)", -0.1, 0.18, "Vert (rad)", -0.1, 0.15), "core/thrust_vector_cmd", "core/thrust_fb1", "core/thrust_fb2");
    //thruster_rpms = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Thruster 1", -1000., 1000., "Thruster 2", -1000., 1000.), "core/rpm_cmd", "core/rpm_fb1", "core/rpm_fb2");
    thruster_angles = new TopicWidget<sam_msgs::ThrusterAngles>(nh, &draw_thruster_angles, "core/thrust_vector_cmd");
    //thruster_rpms = new TopicWidget<sam_msgs::ThrusterRPMs>(nh, &draw_thruster_rpms, "core/rpm_cmd", "core/rpm_fb");
    thruster_rpms = new TopicWidget<sam_msgs::ThrusterRPMs>(nh, &draw_thruster_rpms, "core/rpm_cmd", "core/rpm_cmd");
    rpm_pub = nh->advertise<sam_msgs::ThrusterRPMs>("core/rpm_cmd");

    lcg_actuator = new TopicWidget<sam_msgs::PercentStamped>(nh, &draw_percent, "core/lcg_cmd", "core/lcg_fb");
    lcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "ctrl/lcg/pid_enable");
    lcg_control_setpoint = new TopicWidget<std_msgs::Float64>(nh, DrawFloat64(-1.6, 1.6), "ctrl/lcg/setpoint"); //, -1.6, 1.6)

    vbs_actuator = new TopicWidget<sam_msgs::PercentStamped>(nh, &draw_percent, "core/vbs_cmd", "core/vbs_fb");
    vbs_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "ctrl/vbs/pid_enable");
    vbs_control_setpoint = new TopicWidget<std_msgs::Float64>(nh, DrawFloat64(0., 5.), "ctrl/vbs/setpoint"); //, 0 5)
        
    tcg_actuator = new TopicWidget<sam_msgs::BallastAngles>(nh, &draw_ballast_angles, "core/tcg_cmd");
    tcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "ctrl/tcg/pid_enable");
    tcg_control_setpoint = new TopicWidget<std_msgs::Float64>(nh, DrawFloat64(-1.6, 1.6), "ctrl/tcg/setpoint"); //, -1.6, 1.6)

}

void SamActuatorWidget::pub_callback(const ros::TimerEvent& e)
{
    if (rpm_pub_enabled) {
        rpm_pub->publish(thruster_rpms->get_msg());
    }
}

void SamActuatorWidget::show_window(bool& show_actuator_window)
{
    ImGui::SetNextWindowSize(ImVec2(500, 478), ImGuiCond_FirstUseEver);
    ImGui::Begin("Actuator controls", &show_actuator_window);

    if (ImGui::CollapsingHeader("Thruster Angles", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("Angles");
        thruster_angles->show_widget();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Thruster RPMs", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("RPMs");
        thruster_rpms->show_widget();
        ImGui::Checkbox("Publish RPMs at 10hz", &rpm_pub_enabled);
        ImGui::PopID();
        if (rpm_pub_enabled && pub_timer == nullptr) {
            pub_timer = new roswasm::Timer(0.08, std::bind(&SamActuatorWidget::pub_callback, this, std::placeholders::_1));
        }
        else if (!rpm_pub_enabled && pub_timer != nullptr) {
            delete pub_timer;
            pub_timer = nullptr;
        }
    }

    if (ImGui::CollapsingHeader("Longitudinal Centre of Gravity (LCG)", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("LCG");
        ImGui::Text("Pos (%%)");
        ImGui::SameLine();
        ImGui::PushID("Actuator");
        lcg_actuator->show_widget();
        ImGui::PopID();
        lcg_control_enable->show_widget();
        ImGui::Text("Pitch (rad)");
        ImGui::SameLine();
        ImGui::PushID("Control");
        lcg_control_setpoint->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Variable Buoyancy System (VBS)", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("VBS");
        ImGui::Text("Pos (%%)");
        ImGui::SameLine();
        ImGui::PushID("Actuator");
        vbs_actuator->show_widget();
        ImGui::PopID();
        vbs_control_enable->show_widget();
        ImGui::Text("Depth (m)");
        ImGui::SameLine();
        ImGui::PushID("Control");
        vbs_control_setpoint->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Transversal Centre of Gravity (TCG)", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("TCG");
        ImGui::Text("Pos (rad)");
        ImGui::SameLine();
        ImGui::PushID("Actuator");
        tcg_actuator->show_widget();
        ImGui::PopID();
        tcg_control_enable->show_widget();
        ImGui::Text("Roll (rad)");
        ImGui::SameLine();
        ImGui::PushID("Control");
        tcg_control_setpoint->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }

    ImGui::End();
}

SamDashboardWidget::SamDashboardWidget(roswasm::NodeHandle* nh) : was_leak(false)
{
    leak = new TopicBuffer<sam_msgs::Leak>(nh, "core/leak_fb");
    gps = new TopicBuffer<sensor_msgs::NavSatFix>(nh, "core/gps");
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery_fb");
    odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    vbs = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
    lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
    rpms = new TopicBuffer<sam_msgs::ThrusterRPMs>(nh, "core/rpm_fb", 1000);
    depth = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/depth_feedback", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/pitch_feedback", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/roll_feedback", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/yaw_feedback", 1000);
}

void SamDashboardWidget::show_window(bool& show_dashboard_window)
{
    ImGui::SetNextWindowSize(ImVec2(500, 243), ImGuiCond_FirstUseEver);
    ImGui::Begin("Status dashboard", &show_dashboard_window);

    if (ImGui::CollapsingHeader("Critical Info", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        was_leak = was_leak || leak->get_msg().value;

        float sz = ImGui::GetTextLineHeight();
        std::string status_text;
        ImColor status_color;
        if (!was_leak) {
            status_text = "No leaks!";
            status_color = ImColor(0, 255, 0);
        }
        else {
            status_text = "Leak!!!!!";
            status_color = ImColor(255, 0, 0);
        }
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
        ImGui::Dummy(ImVec2(sz, sz));
        ImGui::SameLine();
        ImGui::Text("%s", status_text.c_str());

        ImGui::SameLine(150);
        ImGui::Text("Battery: %.0f%%", battery->get_msg().percentage);
    }

    if (ImGui::CollapsingHeader("GPS and depth", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Lat: %.5f", gps->get_msg().latitude);
        ImGui::SameLine(150);
        ImGui::Text("Lon: %.5f", gps->get_msg().longitude);
        ImGui::SameLine(300);
        ImGui::Text("Depth: %.2fm", depth->get_msg().data);
    }

    if (ImGui::CollapsingHeader("DR translation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("X: %.2fm", odom->get_msg().pose.pose.position.x);
        ImGui::SameLine(150);
        ImGui::Text("Y: %.2fm", odom->get_msg().pose.pose.position.y);
        ImGui::SameLine(300);
        ImGui::Text("Z: %.2fm", odom->get_msg().pose.pose.position.z);
    }

    if (ImGui::CollapsingHeader("DR rotation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Roll: %.2fdeg", 180./M_PI*roll->get_msg().data);
        ImGui::SameLine(150);
        ImGui::Text("Pitch: %.2fdeg", 180./M_PI*pitch->get_msg().data);
        ImGui::SameLine(300);
        ImGui::Text("Yaw: %.2fdeg", 180./M_PI*yaw->get_msg().data);
    }

    if (ImGui::CollapsingHeader("Actuator feedback", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("VBS pos: %.2f%%", vbs->get_msg().value);
        ImGui::SameLine(150);
        ImGui::Text("LCG pos: %.2f%%", lcg->get_msg().value);
        ImGui::SameLine(300);
        ImGui::Text("RPMs: %d, %d rpm", rpms->get_msg().thruster_1_rpm, rpms->get_msg().thruster_2_rpm);
    }

    ImGui::End();
}

SamTeleopWidget::SamTeleopWidget(roswasm::NodeHandle* nh) : enabled(false), pub_timer(nullptr)
{
    angle_pub = nh->advertise<sam_msgs::ThrusterAngles>("core/thrust_vector_cmd");
    rpm_pub = nh->advertise<sam_msgs::ThrusterRPMs>("core/rpm_cmd");
}

void SamTeleopWidget::pub_callback(const ros::TimerEvent& e)
{
    if (enabled) {
        angle_pub->publish(angles_msg);
        rpm_pub->publish(rpm_msg);
    }
}

void SamTeleopWidget::show_window(bool& show_teleop_window)
{
    ImGuiIO& io = ImGui::GetIO();
    ImVec4 col = ImGui::GetStyle().Colors[23];

    ImGui::SetNextWindowSize(ImVec2(472, 80), ImGuiCond_FirstUseEver);
    ImGui::Begin("Keyboard teleop", &show_teleop_window);

    angles_msg.thruster_vertical_radians = angles_msg.thruster_horizontal_radians = rpm_msg.thruster_1_rpm = rpm_msg.thruster_2_rpm = 0.0f;

    float sz = ImGui::GetTextLineHeight();
    ImGui::BeginGroup();
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 0.75f*sz));
    ImGui::Checkbox("Teleop enabled", &enabled);
    if (enabled && pub_timer == nullptr) {
        pub_timer = new roswasm::Timer(0.08, std::bind(&SamTeleopWidget::pub_callback, this, std::placeholders::_1));
    }
    else if (!enabled && pub_timer != nullptr) {
        delete pub_timer;
        pub_timer = nullptr;
    }
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 1.5f*sz));
    bool key_down = enabled && io.KeysDownDuration[263] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##left", ImGuiDir_Left) || key_down) {
        angles_msg.thruster_horizontal_radians = -0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    key_down = enabled && io.KeysDownDuration[265] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##up", ImGuiDir_Up) || key_down) {
        angles_msg.thruster_vertical_radians = 0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    key_down = enabled && io.KeysDownDuration[264] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##down", ImGuiDir_Down) || key_down) {
        angles_msg.thruster_vertical_radians = -0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 1.5f*sz));
    key_down = enabled && io.KeysDownDuration[262] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##right", ImGuiDir_Right) || key_down) {
        angles_msg.thruster_horizontal_radians = 0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, sz));
    ImGui::Text("Thrust Vector");
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    key_down = enabled && io.KeysDownDuration[87] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::Button("w") || key_down) {
        rpm_msg.thruster_1_rpm = 500;
        rpm_msg.thruster_2_rpm = 500;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    key_down = enabled && io.KeysDownDuration[83] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::Button("s") || key_down) {
        rpm_msg.thruster_1_rpm = -500;
        rpm_msg.thruster_2_rpm = -500;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Text("Forward");
    ImGui::Dummy(ImVec2(sz, 0.5f*sz));
    ImGui::Text("Reverse");
    ImGui::EndGroup();
    ImGui::EndGroup();

    ImGui::End();

    //ImGui::Text("Keys down:");      for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++) if (io.KeysDownDuration[i] >= 0.0f)     { ImGui::SameLine(); ImGui::Text("%d (%.02f secs)", i, io.KeysDownDuration[i]); }
}


} // namespace roswasm_webgui
