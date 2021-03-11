#include <roswasm/roswasm.h>

#ifdef ROSWASM_NATIVE

#include <ros/ros.h>

namespace roswasm {

void init(int argc, char** argv, const std::string& arg)
{
    ros::init(argc, argv, arg);
}

void spinLoop(void(*loop)())
{
    while (ros::ok()) {
        ros::spinOnce();
        (*loop)();
    }
}

void spinLoop(void(*loop)(), roswasm::Duration loop_rate)
{
    ros::Rate rate(1./(1./20.)); //loop_rate.toSec()); // 10 hz
    while (ros::ok()) {
        ros::spinOnce();
        (*loop)();
        rate.sleep();
    }
}

void shutdown()
{
    ros::shutdown();
}

void ServiceCallbackClient::timerCallback(const ros::TimerEvent& ev)
{
    
    ROS_INFO("Timer callback going through %zu running calls", calls.size());
    //std::cout << "Size: " << calls.size() << std::endl;
    calls.erase(std::remove_if(calls.begin(), calls.end(),
                [](ServiceCallbackCallBase* c)
                {
                    bool done = c->is_done();
                    if (done) {
                        delete c;
                    }
                    return done;
                }), calls.end());

    if (calls.empty()) {
        timer.stop();
    }
}

//std::unique_ptr<ServiceCallbackClientImplBase> impl;

ServiceCallbackClient::ServiceCallbackClient(ros::NodeHandle& nh, const std::string& service_name) : nh(&nh), service_name(service_name) //, impl_callback(impl_callback)
{
    calls.clear();
    timer = this->nh->createTimer(ros::Duration(.05), &ServiceCallbackClient::timerCallback, this);
    timer.stop();
}

ServiceCallbackClient::ServiceCallbackClient(ServiceCallbackClient&& other) noexcept : nh(other.nh), calls(std::move(other.calls)), service_name(std::move(other.service_name))
{
    timer = this->nh->createTimer(ros::Duration(.05), &ServiceCallbackClient::timerCallback, this);
    timer.stop();
}

ServiceCallbackClient& ServiceCallbackClient::operator=(ServiceCallbackClient&& other)
{
    nh = other.nh;
    calls = std::move(other.calls);
    //timer = std::move(other.timer);
    timer = this->nh->createTimer(ros::Duration(.05), &ServiceCallbackClient::timerCallback, this);
    timer.stop();
    service_name = std::move(other.service_name);
    return *this;
}

} // namespace roswasm
    
#else
#include <roswasm/roswasm.hpp>
#include <roswasm/timer.hpp>

namespace roswasm {

template <>
void ServiceCallbackClientImpl<std_srvs::SetBool>::call(const std_srvs::SetBool::Request& req, CallbT cb)
{
    impl_callback = cb;
    std::string json_msg = roscpp_json::serialize(req, false, true);
    std::string message = "\"op\":\"call_service\", \"service\":\"" + service_name + "\", \"args\":" + json_msg + ", \"id\":\"" + id + "\"";
    message = std::string("{ ") + message + " }";
    nh->send_message(message);
}

template <>
void ServiceCallbackClientImpl<std_srvs::SetBool>::callback(const std::string& buffer, bool result)
{
    std::cout << "Got service response: " << buffer << std::endl;
    typename std_srvs::SetBool::Response res;
    if (result) {
        res = roscpp_json::deserialize<std_srvs::SetBool::Response, true>(buffer);
    }
    impl_callback(res, result);
}

}
#endif
