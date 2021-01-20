#ifndef ROSWASM_NATIVE_H
#define ROSWASM_NATIVE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <future>
#include <thread>
#include <chrono>

namespace roswasm {

using NodeHandle = ros::NodeHandle;
using Publisher = ros::Publisher;
using Subscriber = ros::Subscriber;
using Timer = ros::Timer;
using Time = ros::Time;
using Duration = ros::Duration;
//using init = ros::init;
void init(int argc, char** argv, const std::string& arg);
void spinLoop(void(*loop)());
void spinLoop(void(*loop)(), roswasm::Duration loop_rate);
void shutdown();

class ServiceCallbackCallBase
{
    public:
    virtual bool is_done() = 0;
};

template <typename SRV>
class ServiceCallbackCall : public ServiceCallbackCallBase
{
    private:
    SRV srv;
    using CallbT = std::function<void(const typename SRV::Response&, bool)>;
    //typedef void(*CallbT)(const typename SRV::Response&, bool);
    CallbT impl_callback;
    std::future<bool> future;
    std::string service_name;
    //std::thread spinner_thread;
    public:
    ServiceCallbackCall(const std::string& _service_name, const typename SRV::Request& req, CallbT impl_callback) : service_name(_service_name), impl_callback(impl_callback)
    {
        srv.request = req;

        ROS_INFO("Calling service %s...", service_name.c_str());
        future = std::async(std::launch::async, [this] {
            ros::NodeHandle nh_service;
            ros::ServiceClient client = nh_service.serviceClient<SRV>(service_name);
            return client.call(srv);
        });
    }

    bool is_done()
    {
        auto status = future.wait_for(std::chrono::milliseconds(0));

        // Print status.
        if (status == std::future_status::ready) {
            ROS_INFO("Thread finished");
        } else {
            ROS_INFO("Thread still running");
            return false;
        }
        bool success = future.get(); // Get result.
        if (success) {
            ROS_INFO("Succcessfully called service %s", service_name.c_str());
        }
        else {
            ROS_ERROR("Failed to call service %s", service_name.c_str());
        }
        //impl_callback(srv.response, success);
        ROS_INFO("Calling callback with results...");
        //(*impl_callback)(srv.response, success);
        impl_callback(srv.response, success);
        ROS_INFO("Done calling callback with results...");
        return true;
    }
};

class ServiceCallbackClient {
    private:

    std::vector<ServiceCallbackCallBase*> calls;
    //ros::CallbackQueue callback_queue_service;
    ros::Timer timer;
    std::string service_name;
    //void (*impl_callback)();
    roswasm::NodeHandle* nh;

    public:

    template <typename SRV>
    void call(const typename SRV::Request& req, std::function<void(const typename SRV::Response&, bool result)> impl_callback)
    {
        calls.push_back(new ServiceCallbackCall<SRV>(service_name, req, impl_callback));
        timer.start();
    }

    void timerCallback(const ros::TimerEvent& ev);

    ServiceCallbackClient(roswasm::NodeHandle& nh, const std::string& service_name); //, void (*impl_callback)());

    ServiceCallbackClient() = default; // : impl(nullptr) {}

    ServiceCallbackClient(ServiceCallbackClient&& other) noexcept;
    ServiceCallbackClient& operator=(ServiceCallbackClient&& other);

};

template <typename SRV>
ServiceCallbackClient createServiceCallbackClient(roswasm::NodeHandle& nh, const std::string& service_name) //, void(*callback)(const typename SRV::Response&, bool))
//ServiceCallbackClient createServiceCallbackClient(roswasm::NodeHandle& nh, const std::string& service_name, std::function<void(const typename SRV::Response&, bool result)> callback)
{
    return ServiceCallbackClient(nh, service_name); //, reinterpret_cast<void(*)()>(callback)); //, callback);
}

} // namespace roswasm

#endif // ROSWASM_NATIVE_H
