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
    ImGui::SetNextWindowSize(ImVec2(550,680), ImGuiCond_FirstUseEver);
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

} // namespace roswasm_webgui
