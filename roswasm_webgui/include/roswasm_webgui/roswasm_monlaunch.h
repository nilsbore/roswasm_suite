#ifndef ROSWASM_MONLAUNCH_H
#define ROSWASM_MONLAUNCH_H

#include <roswasm/roswasm.h>
#include <rosapi/TopicsForType.h>
#include <rosmon_msgs/State.h>
#include <rosmon_msgs/StartStop.h>

namespace roswasm_webgui {

struct LaunchState {

    static const char* status[];

    roswasm::Subscriber sub;
    roswasm::ServiceCallbackClient client;
    rosmon_msgs::State msg;
    std::string name;
    int selected;
    //size_t nbr_nodes;
    
    int start_stop_launch_node_idx;
    int start_stop_launch_attempts;
    uint8_t start_stop_launch_action;
    int max_launch_attempts;

    void service_callback(const rosmon_msgs::StartStop::Response& res, bool result);

    void start_stop_launch_node();

    void callback(const rosmon_msgs::State& new_msg);

    void start_stop_launch(uint8_t action);

    void start_stop(uint8_t action, const std::string& name, const std::string& ns);

    float get_progress();

    LaunchState(roswasm::NodeHandle& node_handle, const std::string& topic, int max_launch_attempts = 10);

    LaunchState();

};

class MonlaunchWidget {
private:
    roswasm::ServiceCallbackClient topics_service;
    std::map<std::string, LaunchState*> launch_states;
    roswasm::NodeHandle& nh;
    roswasm::Timer timer;
public:
    void service_callback(const rosapi::TopicsForType::Response& res, bool result);
    void timer_callback(const ros::TimerEvent& event);
    void show_window(bool& show_another_window);
    MonlaunchWidget(roswasm::NodeHandle& n);
};

} // namespace roswasm_webgui

#endif // ROSWASM_MONLAUNCH_H
