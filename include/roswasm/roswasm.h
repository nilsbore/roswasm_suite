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
    Timer* timer;

    Timer* createTimer(double seconds, std::function<void(const ros::TimerEvent&)> cb)
    {
        return new Timer(seconds, cb);
    }

    template <typename MSG>
    Subscriber* subscribe(const std::string& topic, std::function<void(const MSG&)> callback, int throttle_rate = -1, int queue_length = -1, int fragment_size = -1);

    template <typename MSG>
    Publisher* advertise(const std::string& topic, const std::string& id="");

    template <typename SRV>
    ServiceClient* serviceClient(const std::string& service_name, std::function<void(const typename SRV::Response&, bool result)> callback);

    void try_websocket_connect();
    void websocket_open();
    void websocket_close();

    static EM_BOOL WebSocketOpen(int eventType, const EmscriptenWebSocketOpenEvent *e, void *userData);
    static EM_BOOL WebSocketClose(int eventType, const EmscriptenWebSocketCloseEvent *e, void *userData);
    static EM_BOOL WebSocketError(int eventType, const EmscriptenWebSocketErrorEvent *e, void *userData);
    static EM_BOOL WebSocketMessage(int eventType, const EmscriptenWebSocketMessageEvent *e, void *userData);

    void handle_bytes(const EmscriptenWebSocketMessageEvent* e);
    void handle_string(const EmscriptenWebSocketMessageEvent* e);

    void send_message(const std::string& message);

    NodeHandle();

    /*
    ~NodeHandle()
    {
        printf("Running NodeHandle destructor!");
        emscripten_websocket_close(socket, 0, 0);
        emscripten_websocket_delete(socket);
    }
    */
};


} // namespace wasmros

#include "roswasm.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "service_client.hpp"
#include "timer.hpp"

#endif // ROSWASM_H
