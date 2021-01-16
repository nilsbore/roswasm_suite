#include <roswasm/roswasm.h>

#ifdef ROSWASM_NATIVE

#include <ros/ros.h>

namespace roswasm {

void init(int argc, char** argv, const std::string& arg)
{
    ros::init(argc, argv, arg);
}

void ServiceCallbackClient::timerCallback(const ros::TimerEvent& ev)
{
    
    ROS_INFO("Timer callback going through %zu running calls", calls.size());
    //std::cout << "Size: " << calls.size() << std::endl;
    calls.erase(std::remove_if(calls.begin(), calls.end(),
                [](ServiceCallbackCallBase* c)
                {
                    return c->is_done();
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
#endif
