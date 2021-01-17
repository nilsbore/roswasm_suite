#include <roswasm_webgui/roswasm_widget.h>

#include "roswasm_webgui/imgui/imgui.h"

namespace roswasm_webgui {

bool draw_float(std_msgs::Float32& msg, roswasm::Publisher& pub)
{
    ImGui::PushID("Float");
    ImGui::SliderFloat("Float slider", &msg.data, 0.0f, 100.0f);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    ImGui::SameLine();
    ImGui::InputFloat("Float value", &msg.data);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    ImGui::PopID();

    return false;
}

bool draw_bool(std_msgs::Bool& msg, roswasm::Publisher& pub)
{
    ImGui::PushID("Bool");
    bool value = msg.data;
    ImGui::Checkbox("Controller active", &value);
    msg.data = value;
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    ImGui::PopID();

    return false;
}

bool draw_pose2d(std_msgs::Float64& msg1, std_msgs::Float64& msg2, roswasm::Publisher& pub)
{
    geometry_msgs::Pose2D msg;
    msg.x = msg1.data;
    msg.y = msg2.data;

    double min_value = 0.0;
    double max_value = 100.0;

    ImGui::PushID("Pose2D");

    ImGui::SliderScalar("First cmd slider", ImGuiDataType_Double, &msg1.data, &min_value, &max_value, "%.10f grams");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.x = msg1.data;
        pub.publish(msg);
    }
    ImGui::SameLine();
    ImGui::InputScalar("First cmd input",  ImGuiDataType_Double, &msg1.data);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.x = msg1.data;
        pub.publish(msg);
    }

    ImGui::SliderScalar("Second cmd slider", ImGuiDataType_Double, &msg2.data, &min_value, &max_value, "%.10f grams");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.y = msg2.data;
        pub.publish(msg);
    }
    ImGui::SameLine();
    ImGui::InputScalar("Second cmd input",  ImGuiDataType_Double, &msg2.data);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.y = msg2.data;
        pub.publish(msg);
    }

    ImGui::PopID();

    return false;
}

bool DrawFloat32::operator()(std_msgs::Float32& msg, roswasm::Publisher& pub)
{
    ImGui::SliderFloat("", &msg.data, minv, maxv, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    ImGui::SameLine();
    ImGui::InputFloat("Float value", &msg.data, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }

    return false;
}

bool DrawFloat64::operator()(std_msgs::Float64& msg, roswasm::Publisher& pub)
{
    ImGui::PushID("Cmd slider");
    ImGui::SliderScalar("", ImGuiDataType_Double, &msg.data, &minv, &maxv, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputScalar("Cmd input",  ImGuiDataType_Double, &msg.data, NULL, NULL, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }

    return false;
}

bool DrawFloatPair::operator()(std_msgs::Float64& msg1, std_msgs::Float64& msg2, roswasm::Publisher& pub)
{
    geometry_msgs::Pose2D msg;
    msg.x = msg1.data;
    msg.y = msg2.data;

    ImGui::PushID("First cmd slider");
    ImGui::Text("%s", name1.c_str());
    ImGui::SameLine();
    ImGui::SliderScalar("", ImGuiDataType_Double, &msg1.data, &minv1, &maxv1, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.x = msg1.data;
        pub.publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputScalar("First cmd input",  ImGuiDataType_Double, &msg1.data, NULL, NULL, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.x = msg1.data;
        pub.publish(msg);
    }

    ImGui::PushID("Second cmd slider");
    ImGui::Text("%s", name2.c_str());
    ImGui::SameLine();
    ImGui::SliderScalar("", ImGuiDataType_Double, &msg2.data, &minv1, &maxv1, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.y = msg2.data;
        pub.publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputScalar("Second cmd input",  ImGuiDataType_Double, &msg2.data, NULL, NULL, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.y = msg2.data;
        pub.publish(msg);
    }

    return false;
}

} // namespace roswasm_webgui
