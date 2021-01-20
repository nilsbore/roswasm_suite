#include <roswasm_webgui/roswasm_monlaunch.h>
#include <roswasm_webgui/imgui/imgui.h>

namespace roswasm_webgui {

// ros stuff

void LaunchState::service_callback(const rosmon_msgs::StartStop::Response& res, bool result)
{
    printf("Got StartStop service response!\n");
}

void LaunchState::start_stop_launch_node()
{
    if (start_stop_launch_node_idx == -1 || start_stop_launch_node_idx >= msg.nodes.size()) {
        return;
    }

    uint8_t desired_state;
    if (start_stop_launch_action == rosmon_msgs::StartStop::Request::START) {
        desired_state = rosmon_msgs::NodeState::RUNNING;
    }
    else if (start_stop_launch_action == rosmon_msgs::StartStop::Request::STOP) {
        desired_state = rosmon_msgs::NodeState::IDLE;
    }

    uint8_t state = msg.nodes[start_stop_launch_node_idx].state;
    if (state == rosmon_msgs::NodeState::CRASHED) {
        state = rosmon_msgs::NodeState::IDLE;
    }
    while (state == desired_state) {
        ++start_stop_launch_node_idx;
        start_stop_launch_attempts = 0;
        if (start_stop_launch_node_idx >= msg.nodes.size()) {
            start_stop_launch_node_idx = -1;
            break;
        }
        state = msg.nodes[start_stop_launch_node_idx].state;
        if (state == rosmon_msgs::NodeState::CRASHED) {
            state = rosmon_msgs::NodeState::IDLE;
        }
    }
    if (start_stop_launch_attempts > max_launch_attempts) {
        start_stop_launch_node_idx = -1;
    }
    if (start_stop_launch_node_idx != -1) {
        start_stop(start_stop_launch_action, msg.nodes[start_stop_launch_node_idx].name, msg.nodes[start_stop_launch_node_idx].ns);
        ++start_stop_launch_attempts;
    }
}

void LaunchState::callback(const rosmon_msgs::State& new_msg)
{
    msg = new_msg;
    start_stop_launch_node();
    printf("Got quite simple msg\n");
    printf("Nodes len: %zu\n", new_msg.nodes.size());
}

void LaunchState::start_stop_launch(uint8_t action)
{
    start_stop_launch_node_idx = 0;
    start_stop_launch_attempts = 0;
    start_stop_launch_action = action;
    start_stop_launch_node();
}

void LaunchState::start_stop(uint8_t action, const std::string& name, const std::string& ns)
{
    rosmon_msgs::StartStop::Request req;
    req.node = name;
    req.action = action;
    req.ns = ns;
    client.call<rosmon_msgs::StartStop>(req, std::bind(&LaunchState::service_callback, this, std::placeholders::_1, std::placeholders::_2));
}

float LaunchState::get_progress()
{
    if (start_stop_launch_node_idx == -1) {
        float sum = 0.;
        for (const rosmon_msgs::NodeState& node : msg.nodes) {
            sum += float(node.state == rosmon_msgs::NodeState::RUNNING);
        }
        return sum / float(msg.nodes.size());
    }
    else {
        return float(start_stop_launch_node_idx)/float(msg.nodes.size());
    }
}

LaunchState::LaunchState(roswasm::NodeHandle& node_handle, const std::string& topic, int max_launch_attempts) : selected(-1), start_stop_launch_node_idx(-1), max_launch_attempts(max_launch_attempts)
{
    //sub = node_handle.subscribe(topic, 1000, std::bind(&LaunchState::callback, this, std::placeholders::_1));
    sub = node_handle.subscribe(topic, 1000, &LaunchState::callback, this);
    //topics_service = nh->serviceClient<rosapi::TopicsForType>("/rosapi/topics_for_type", service_callback);
    size_t ns_pos = topic.find('/', 1);
    name = topic.substr(1, ns_pos-1);
    std::string service_name = std::string("/") + name + "/start_stop";
    printf("Service name: %s\n", service_name.c_str());
    //client = node_handle->serviceClient<rosmon_msgs::StartStop>(service_name, std::bind(&LaunchState::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    client = roswasm::createServiceCallbackClient<rosmon_msgs::StartStop>(node_handle, service_name); //, std::bind(&LaunchState::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    //sub = node_handle->subscribe<rosmon_msgs::State>(topic, simple_callback);
}

LaunchState::LaunchState() {} // : sub(nullptr) {}

const char* LaunchState::status[] = { "IDLE", "RUNNING", "CRASHED", "WAITING" };

void MonlaunchWidget::service_callback(const rosapi::TopicsForType::Response& res, bool result)
{
    for (int i = 0; i < res.topics.size(); ++i) {
        if (launch_states.count(res.topics[i]) == 0) {
            launch_states[res.topics[i]] = new LaunchState(nh, res.topics[i]);
        }
    }
}

void MonlaunchWidget::timer_callback(const ros::TimerEvent& event)
{
    rosapi::TopicsForType::Request req;
    req.type = "rosmon_msgs/State";
    topics_service.call<rosapi::TopicsForType>(req, std::bind(&MonlaunchWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
}

MonlaunchWidget::MonlaunchWidget(roswasm::NodeHandle& n) : nh(n)
{
    //nh = n;
    //topics_service = nh->serviceClient<rosapi::TopicsForType>("/rosapi/topics_for_type", std::bind(&MonlaunchWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    topics_service = roswasm::createServiceCallbackClient<rosapi::TopicsForType>(nh, "/rosapi/topics_for_type"); //, std::bind(&MonlaunchWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    rosapi::TopicsForType::Request req;
    req.type = "rosmon_msgs/State";
    topics_service.call<rosapi::TopicsForType>(req, std::bind(&MonlaunchWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    timer = nh.createTimer(roswasm::Duration(5.), std::bind(&MonlaunchWidget::timer_callback, this, std::placeholders::_1));
}

void MonlaunchWidget::show_window(bool& show_another_window)
{
      ImGui::Begin("Launch control", &show_another_window);
      for (std::pair<const std::string, LaunchState*>& state : launch_states) {
          if (state.second->msg.nodes.empty()) {
             continue;
          } 
          if (ImGui::CollapsingHeader(state.second->name.c_str(), ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
              ImGui::PushID(state.second->name.c_str());
              bool nodes_expanded = ImGui::TreeNodeEx("Nodes", ImGuiTreeNodeFlags_AllowItemOverlap | ImGuiTreeNodeFlags_FramePadding);
              ImGui::SameLine(); //(0, 0);
              if (ImGui::Button("Start"))
                  state.second->start_stop_launch(rosmon_msgs::StartStop::Request::START);
              ImGui::SameLine();
              if (ImGui::Button("Stop"))
                  state.second->start_stop_launch(rosmon_msgs::StartStop::Request::STOP);
              ImGui::SameLine();
              char buf[32];
              float progress = state.second->get_progress();
              sprintf(buf, "%d/%d", int(progress*state.second->msg.nodes.size()), int(state.second->msg.nodes.size()));
              ImGui::ProgressBar(progress, ImVec2(0.f,0.f), buf);
              if (nodes_expanded) {
                    ImGui::Columns(5, "mycolumns"); // 4-ways, with border
                    ImGui::Separator();
                    ImGui::Text("Name"); ImGui::NextColumn();
                    ImGui::Text("State"); ImGui::NextColumn();
                    ImGui::Text("CPU"); ImGui::NextColumn();
                    ImGui::Text("Memory"); ImGui::NextColumn();
                    ImGui::Text("Restarts"); ImGui::NextColumn();
                    ImGui::Separator();
                    for (int i = 0; i < state.second->msg.nodes.size(); i++)
                    {
                        const std::string& name = state.second->msg.nodes[i].name;
                        const std::string& ns = state.second->msg.nodes[i].ns;
                        if (ImGui::Selectable(name.c_str(), state.second->selected == i, ImGuiSelectableFlags_SpanAllColumns))
                            state.second->selected = i;
                        if (ImGui::BeginPopupContextItem(name.c_str()))
                        {
                            if (ImGui::Selectable("START")) state.second->start_stop(rosmon_msgs::StartStop::Request::START, name, ns);
                            if (ImGui::Selectable("STOP")) state.second->start_stop(rosmon_msgs::StartStop::Request::STOP, name, ns);
                            if (ImGui::Selectable("RESTART")) state.second->start_stop(rosmon_msgs::StartStop::Request::RESTART, name, ns);
                            ImGui::EndPopup();
                        }
                        bool hovered = ImGui::IsItemHovered();
                        ImGui::NextColumn();
                        ImGui::Text("%s", LaunchState::status[state.second->msg.nodes[i].state]); ImGui::NextColumn();
                        ImGui::Text("%.2f %%", 100.*state.second->msg.nodes[i].user_load); ImGui::NextColumn();
                        ImGui::Text("%.2f MB", 1e-6*state.second->msg.nodes[i].memory); ImGui::NextColumn();
                        ImGui::Text("%u", state.second->msg.nodes[i].restart_count); ImGui::NextColumn();
                    }
                    ImGui::Columns(1);
                    ImGui::Separator();
                    ImGui::TreePop();
              }
              ImGui::PopID();
          }
      }
      ImGui::End();
}

} // namespace roswasm_webgui
