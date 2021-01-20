#ifndef ROSWASM_H
#define ROSWASM_H

#ifdef ROSWASM_NATIVE
#include <roswasm/roswasm_native.h>
#else

#ifdef ROSCONSOLE_BACKEND_LOG4CXX
  #undef ROSCONSOLE_BACKEND_LOG4CXX
#endif

#ifdef ROS_BUILD_SHARED_LIBS
  //#define ROS_BUILD_SHARED_LIBS 0
  #undef ROS_BUILD_SHARED_LIBS
#endif

// assumes we're in an emscripten environment
#include <emscripten.h>
#include <emscripten/websocket.h>

#include <roswasm/publisher.h>
#include <roswasm/subscriber.h>
#include <roswasm/service_client.h>
#include <roswasm/timer.h>
#include <roswasm/time.h>

#include <unordered_map>
#include <list>

namespace roswasm {

// dummy init
void init(int argc, const char* const* argv, const std::string& arg);
void spinLoop(void(*loop)());
void spinLoop(void(*loop)(), roswasm::Duration loop_rate);
void shutdown();

class NodeHandleImpl
{
public:

    EMSCRIPTEN_WEBSOCKET_T socket;
    std::list<std::string> message_queue;
    bool socket_open;
    std::unordered_map<std::string, SubscriberImplBase*> subscribers;
    std::unordered_map<std::string, PublisherImplBase*> publishers;
    std::unordered_map<std::string, ServiceCallbackClientImplBase*> service_clients;
    std::string rosbridge_ip;
    std::string rosbridge_port;
    Timer timer;
    static bool debug_print;

    bool ok()
    {
        return socket_open;
    }

    Timer createTimer(roswasm::Duration duration, std::function<void(const ros::TimerEvent&)> cb)
    {
        return Timer(duration, cb);
    }

    template <typename MSG>
    //Subscriber subscribe(const std::string& topic, void(*callback)(const MSG&), int throttle_rate = -1, int queue_length = -1, int fragment_size = -1);
    Subscriber subscribe(const std::string& topic, std::function<void(const MSG&)> callback, int throttle_rate = -1, int queue_length = -1, int fragment_size = -1);

    void unsubscribe(const std::string& id);

    template <typename MSG>
    Publisher advertise(const std::string& topic, const std::string& id="");

    template <typename SRV>
    ServiceCallbackClient serviceCallbackClient(const std::string& service_name); //, std::function<void(const typename SRV::Response&, bool result)> callback);

    void try_websocket_connect();
    void websocket_open();
    void websocket_close();
    std::string get_websocket_url();

    static EM_BOOL WebSocketOpen(int eventType, const EmscriptenWebSocketOpenEvent *e, void *userData);
    static EM_BOOL WebSocketClose(int eventType, const EmscriptenWebSocketCloseEvent *e, void *userData);
    static EM_BOOL WebSocketError(int eventType, const EmscriptenWebSocketErrorEvent *e, void *userData);
    static EM_BOOL WebSocketMessage(int eventType, const EmscriptenWebSocketMessageEvent *e, void *userData);

    void handle_bytes(const EmscriptenWebSocketMessageEvent* e);
    void handle_string(const EmscriptenWebSocketMessageEvent* e);

    void send_message(const std::string& message);

    NodeHandleImpl(const std::string& rosbridge_ip="127.0.0.1", const std::string& rosbridge_port="9090");

    /*
    ~NodeHandle()
    {
        printf("Running NodeHandle destructor!");
        emscripten_websocket_close(socket, 0, 0);
        emscripten_websocket_delete(socket);
    }
    */
};

class NodeHandle {
private:
    std::unique_ptr<NodeHandleImpl> impl;
public:
    bool ok()
    {
        return impl->ok();
    }

    std::string get_websocket_url()
    {
        return impl->get_websocket_url();
    }

    Timer createTimer(roswasm::Duration duration, std::function<void(const ros::TimerEvent&)> cb)
    {
        return impl->createTimer(duration, cb);
    }

    template <typename MSG>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*callback)(const MSG&), int throttle_rate = -1, int queue_length = -1, int fragment_size = -1)
    {
        return impl->subscribe<MSG>(topic, std::function<void(const MSG&)>(callback), throttle_rate, queue_length, fragment_size);
    }

    template <typename MSG, typename T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*callback)(const MSG&), T* obj, int throttle_rate = -1, int queue_length = -1, int fragment_size = -1)
    {
        return impl->subscribe<MSG>(topic, std::function<void(const MSG&)>(std::bind(callback, obj, std::placeholders::_1)), throttle_rate, queue_length, fragment_size);
    }

    void unsubscribe(const std::string& id)
    {
        impl->unsubscribe(id);
    }

    template <typename MSG>
    Publisher advertise(const std::string& topic, uint32_t queue_size, const std::string& id="")
    {
        return impl->advertise<MSG>(topic, id);
    }

    template <typename SRV>
    ServiceCallbackClient serviceCallbackClient(const std::string& service_name)
    {
        return impl->serviceCallbackClient<SRV>(service_name);
    }
    
    //NodeHandle() = default; // : impl(nullptr) {}

    NodeHandle(const std::string& rosbridge_ip="127.0.0.1", const std::string& rosbridge_port="9090") : impl(new NodeHandleImpl(rosbridge_ip, rosbridge_port))
    {

    }

    /*
    virtual ~NodeHandle()
    {
        //delete impl;
    }
    */

    NodeHandle(NodeHandle&& other) noexcept : impl(std::move(other.impl))
    {
    }

    NodeHandle& operator=(NodeHandle&& other)
    {
        impl = std::move(other.impl);
        return *this;
    }
};

template <typename SRV>
ServiceCallbackClient createServiceCallbackClient(roswasm::NodeHandle& nh, const std::string& service_name)
{
    return nh.serviceCallbackClient<SRV>(service_name);
}

template <typename MSG>
Subscriber NodeHandleImpl::subscribe(const std::string& topic, std::function<void(const MSG&)> callback, int throttle_rate, int queue_length, int fragment_size)
{
    SubscriberImplBase* impl = new SubscriberImpl<MSG>(callback, this, topic, throttle_rate, queue_length, fragment_size);

    if (NodeHandleImpl::socket_open) {
        std::string message = impl->json_subscribe_message();
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }

    subscribers[impl->get_id()] = impl;
    return Subscriber(impl);
}

template <typename MSG>
Publisher NodeHandleImpl::advertise(const std::string& topic, const std::string& id)
{
    PublisherImplBase* impl = new PublisherImpl<MSG>(this, topic);

    if (NodeHandleImpl::socket_open) {
        std::string message = impl->json_advertise_message();
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }

    publishers[impl->get_id()] = impl;
    return Publisher(impl);
}

template <typename SRV>
ServiceCallbackClient NodeHandleImpl::serviceCallbackClient(const std::string& service_name) //, std::function<void(const typename SRV::Response&, bool result)> callback)
{
    ServiceCallbackClientImplBase* impl = new ServiceCallbackClientImpl<SRV>(this, service_name);

    service_clients[impl->get_id()] = impl;
    return ServiceCallbackClient(impl);
}

} // namespace wasmros

#include "publisher.hpp"
#include "subscriber.hpp"
#include "service_client.hpp"

#ifdef ROSWASM_IMPL
#include "roswasm.hpp"
#include "timer.hpp"
#endif

#endif // !ROSWASM_NATIVE

#endif // ROSWASM_H
