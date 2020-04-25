#ifndef ROSWASM_H
#define ROSWASM_H

// assumes we're in an emscripten environment
#include <emscripten.h>
#include <emscripten/websocket.h>

#include <roswasm/publisher.h>
#include <roswasm/subscriber.h>
#include <roswasm/service_client.h>
#include <roswasm/timer.h>

#include <unordered_map>
#include <list>

namespace roswasm {

class NodeHandle
{
public:

    EMSCRIPTEN_WEBSOCKET_T socket;
    std::list<std::string> message_queue;
    bool socket_open;
    std::unordered_map<std::string, Subscriber*> subscribers;
    std::unordered_map<std::string, Publisher*> publishers;
    std::unordered_map<std::string, ServiceClient*> service_clients;
    std::string rosbridge_ip;
    std::string rosbridge_port;
    Timer* timer;

    bool ok()
    {
        return socket_open;
    }

    Timer* createTimer(double seconds, std::function<void(const ros::TimerEvent&)> cb)
    {
        return new Timer(seconds, cb);
    }

    template <typename MSG>
    Subscriber* subscribe(const std::string& topic, std::function<void(const MSG&)> callback, int throttle_rate = -1, int queue_length = -1, int fragment_size = -1);

    void unsubscribe(const std::string& id);

    template <typename MSG>
    Publisher* advertise(const std::string& topic, const std::string& id="");

    template <typename SRV>
    ServiceClient* serviceClient(const std::string& service_name, std::function<void(const typename SRV::Response&, bool result)> callback);

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

    NodeHandle(const std::string& rosbridge_ip="127.0.0.1", const std::string& rosbridge_port="9090");

    /*
    ~NodeHandle()
    {
        printf("Running NodeHandle destructor!");
        emscripten_websocket_close(socket, 0, 0);
        emscripten_websocket_delete(socket);
    }
    */
};

template <typename MSG>
Subscriber* NodeHandle::subscribe(const std::string& topic, std::function<void(const MSG&)> callback, int throttle_rate, int queue_length, int fragment_size)
{
    Subscriber* subscriber = new Subscriber(new SubscriberImpl<MSG>(callback, this), topic, throttle_rate, queue_length, fragment_size);

    if (NodeHandle::socket_open) {
        std::string message = subscriber->json_subscribe_message();
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }

    subscribers[subscriber->get_id()] = subscriber;
    return subscriber;
}

template <typename MSG>
Publisher* NodeHandle::advertise(const std::string& topic, const std::string& id)
{
    Publisher* publisher = new Publisher(new PublisherImpl<MSG>(this), topic);

    if (NodeHandle::socket_open) {
        std::string message = publisher->json_advertise_message();
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }

    publishers[publisher->get_id()] = publisher;
    return publisher;
}

template <typename SRV>
ServiceClient* NodeHandle::serviceClient(const std::string& service_name, std::function<void(const typename SRV::Response&, bool result)> callback)
{
    ServiceClient* service_client = new ServiceClient(new ServiceClientImpl<SRV>(callback, this), service_name);

    service_clients[service_client->get_id()] = service_client;
    return service_client;
}

} // namespace wasmros

#include "publisher.hpp"
#include "subscriber.hpp"
#include "service_client.hpp"

#ifdef ROSWASM_IMPL
#include "roswasm.hpp"
#include "timer.hpp"
#endif

#endif // ROSWASM_H
