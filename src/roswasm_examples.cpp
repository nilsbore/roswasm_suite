#include <roswasm_examples.h>

#include "imgui.h"

namespace roswasm_webgui {

ExampleActuatorWidget::ExampleActuatorWidget(roswasm::NodeHandle* nh)
{
    thruster_angles = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Hori (rad)", -0.1, 0.18, "Vert (rad)", -0.1, 0.15), "/sam/core/thrust_vector_cmd", "/sam/core/thrust_fb1", "/sam/core/thrust_fb2");
//                                                      [{"name": "Hori.", "member": "thruster_horizontal_radians", "min": -0.1, "max": 0.18},
//                                                       {"name": "Vert.", "member": "thruster_vertical_radians", "min": -0.1, "max": 0.15}])
    thruster_rpms = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Thruster 1", -1000., 1000., "Thruster 2", -1000., 1000.), "/sam/core/rpm_cmd", "/sam/core/rpm_fb1", "/sam/core/rpm_fb2");
//                                                    [{"name": "Front", "member": "thruster_1_rpm", "min": -1000, "max": 1000, "type": "int"},
//                                                     {"name": "Back", "member": "thruster_2_rpm", "min": -1000, "max": 1000, "type": "int"}])

//            self.leak_button = flx.Button(text="No leaks...", style="background: #008000;", disabled=True)

    lcg_actuator = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 100.), "/sam/core/lcg_cmd", "/sam/core/lcg_fb");
    lcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "/sam/ctrl/lcg/pid_enable");
    lcg_control_setpoint = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-1.6, 1.6), "/sam/ctrl/lcg/setpoint"); //, -1.6, 1.6)

    vbs_actuator = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 100.), "/sam/core/vbs_cmd", "/sam/core/vbs_fb");
    vbs_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "/sam/ctrl/vbs/pid_enable");
    vbs_control_setpoint = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 5.), "/sam/ctrl/vbs/setpoint"); //, 0 5)
        
    tcg_actuator = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-1.6, 1.6), "/sam/core/tcg_cmd");
    tcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "/sam/ctrl/tcg/pid_enable");
    tcg_control_setpoint = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-1.6, 1.6), "/sam/ctrl/tcg/setpoint"); //, -1.6, 1.6)

}

void ExampleActuatorWidget::show_window(bool& show_actuator_window)
{
    ImGui::SetNextWindowSize(ImVec2(500, 454), ImGuiCond_FirstUseEver);
    ImGui::Begin("Actuator controls", &show_actuator_window);

    if (ImGui::CollapsingHeader("Thruster Angles", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("Angles");
        thruster_angles->show_widget();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Thruster RPMs", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("RPMs");
        thruster_rpms->show_widget();
        ImGui::PopID();
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

ExampleDashboardWidget::ExampleDashboardWidget(roswasm::NodeHandle* nh) : was_leak(false)
{
    leak = new TopicBuffer<std_msgs::Bool>(nh, "/sam/core/leak_fb");
    gps = new TopicBuffer<sensor_msgs::NavSatFix>(nh, "/sam/core/gps");
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "/sam/core/battery_fb");
    odom = new TopicBuffer<nav_msgs::Odometry>(nh, "/sam/dr/odom", 1000);
    vbs = new TopicBuffer<std_msgs::Float64>(nh, "/sam/core/vbs_fb", 1000);
    lcg = new TopicBuffer<std_msgs::Float64>(nh, "/sam/core/lcg_fb", 1000);
    depth = new TopicBuffer<std_msgs::Float64>(nh, "/sam/ctrl/depth_feedback", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "/sam/ctrl/pitch_feedback", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "/sam/ctrl/roll_feedback", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "/sam/ctrl/yaw_feedback", 1000);
}

void ExampleDashboardWidget::show_window(bool& show_dashboard_window)
{
    ImGui::SetNextWindowSize(ImVec2(500, 243), ImGuiCond_FirstUseEver);
    ImGui::Begin("Status dashboard", &show_dashboard_window);

    if (ImGui::CollapsingHeader("Critical Info", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        was_leak = was_leak || leak->get_msg().data;

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
        ImGui::Text("VBS pos: %.2f%%", vbs->get_msg().data);
        ImGui::SameLine(150);
        ImGui::Text("LCG pos: %.2f%%", lcg->get_msg().data);
    }

    ImGui::End();
}

ExampleTeleopWidget::ExampleTeleopWidget(roswasm::NodeHandle* nh)
{
    angle_pub = nh->advertise<geometry_msgs::Pose2D>("/sam/core/thrust_vector_cmd");
    rpm_pub = nh->advertise<geometry_msgs::Pose2D>("/sam/core/rpm_cmd");
}

void ExampleTeleopWidget::pub_callback(const ros::TimerEvent& e)
{

}

void ExampleTeleopWidget::show_window(bool& show_teleop_window)
{
    ImGui::SetNextWindowSize(ImVec2(472, 243), ImGuiCond_FirstUseEver);
    ImGui::Begin("Keyboard teleop", &show_teleop_window);
    float sz = ImGui::GetTextLineHeight();
    ImGui::BeginGroup();
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, sz));
    ImGui::ArrowButton("##left", ImGuiDir_Left);
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::ArrowButton("##up", ImGuiDir_Up);
    ImGui::ArrowButton("##down", ImGuiDir_Down);
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, sz));
    ImGui::ArrowButton("##right", ImGuiDir_Right);
    ImGui::EndGroup();
    ImGui::EndGroup();
    ImGui::End();
    //ImGui::Text("Keys down:");      for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++) if (io.KeysDownDuration[i] >= 0.0f)     { ImGui::SameLine(); ImGui::Text("%d (%.02f secs)", i, io.KeysDownDuration[i]); }
}


} // namespace roswasm_webgui
