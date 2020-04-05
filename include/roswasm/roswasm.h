#ifndef ROSWASM_H
#define ROSWASM_H

// assumes we're in an emscripten environment
#include <emscripten.h>
#include <emscripten/websocket.h>

#include <unordered_map>
#include <ros/serialization.h>
#include <cbor-lite/codec.h>
#include <roscpp_json/serialize.h>
#include <roscpp_json/deserialize.h>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

namespace ros
{
namespace serialization
{
void throwStreamOverrun()
{
  throw StreamOverrunException("Buffer Overrun");
}
}
}

using namespace std::placeholders;

namespace roswasm {

class NodeHandle;

class SubscriberImplBase {
    public:

    virtual void callback(std::vector<uint8_t>& buffer) = 0;
    virtual ~SubscriberImplBase() {}

};

template <typename MSG>
class SubscriberImpl : public SubscriberImplBase {
    public:

    std::function<void(const MSG&)> impl_callback;
    
    void callback(std::vector<uint8_t>& buffer)
    {
        namespace ser = ros::serialization;
        MSG msg;
        uint32_t serial_size = ros::serialization::serializationLength(msg);
        // Fill buffer with a serialized UInt32
        ser::IStream stream(&buffer[0], buffer.size());
        ser::deserialize(stream, msg);
        impl_callback(msg);
    }

    SubscriberImpl(std::function<void(const MSG&)> cb) : impl_callback(cb)
    {

    }
    ~SubscriberImpl() {}

};

class Subscriber {
    public:

    SubscriberImplBase* impl;

    void callback(std::vector<uint8_t>& buffer)
    {
        if (impl != nullptr) {
            impl->callback(buffer);
        }
        else {
            printf("Subscriber callback not initialized!\n");
        }
    }

    Subscriber(SubscriberImplBase* new_impl) : impl(new_impl) {}

    Subscriber() : impl(nullptr) {}

};

class ServiceClientImplBase {
    public:

    virtual void callback(const std::string& buffer, bool result) = 0;
    virtual ~ServiceClientImplBase() {}

};

template <typename SRV>
class ServiceClientImpl : public ServiceClientImplBase {
private:

    NodeHandle* nh;

public:

    std::function<void(const typename SRV::Response&, bool)> impl_callback;

    void call(const typename SRV::Request& req, const std::string& service_name);
    
    void callback(const std::string& buffer, bool result)
    {
        std::cout << "Got service response: " << buffer << std::endl;
        typename SRV::Response res;
        if (result) {
           res = roscpp_json::deserialize<typename SRV::Response>(buffer);
        }
        impl_callback(res, result);
    }

    ServiceClientImpl(std::function<void(const typename SRV::Response&, bool)> cb, NodeHandle* nh) : impl_callback(cb), nh(nh)
    {

    }
    ~ServiceClientImpl() {}

};

class ServiceClient {
    public:

    ServiceClientImplBase* impl;
    std::string service_name;

    void callback(const std::string& buffer, bool result)
    {
        if (impl != nullptr) {
            impl->callback(buffer, result);
        }
        else {
            printf("ServiceClient callback not initialized!\n");
        }
    }

    template <typename SRV>
    void call(const typename SRV::Request& req)
    {
        ServiceClientImpl<SRV>* value = dynamic_cast<ServiceClientImpl<SRV>*>(impl);
        value->call(req, service_name);
    }

    ServiceClient(ServiceClientImplBase* new_impl, const std::string& new_service_name) : impl(new_impl), service_name(new_service_name) {}

    ServiceClient() : impl(nullptr) {}

};

class PublisherImplBase
{
    public:
    virtual ~PublisherImplBase() {}
};

template <typename MSG>
class PublisherImpl : public PublisherImplBase
{
private:
    NodeHandle* nh;
public:
    void publish(const MSG& msg, const std::string& topic);

    PublisherImpl(NodeHandle* nh) : nh(nh) {}
    ~PublisherImpl() {}
};

class Publisher {
    public:

    PublisherImplBase* impl;
    std::string topic;

    template <typename MSG>
    void publish(const MSG& msg)
    {
        // this basically allows us to check that advertise was called with correct template
        PublisherImpl<MSG>* value = dynamic_cast<PublisherImpl<MSG>*>(impl);
        value->publish(msg, topic);
    }

    Publisher(PublisherImplBase* new_impl, const std::string& new_topic) : impl(new_impl), topic(new_topic) {}

    Publisher() : impl(nullptr) {}
};

struct Timer {

    long id;
    std::function<void(const ros::TimerEvent&)> callback;

    static void impl_callback(void* user_data)
    {
        Timer* timer = (Timer*)(user_data);
        ros::TimerEvent ev;
        timer->callback(ev);
    }

    void stop()
    {
        if (id != 0) {
            emscripten_clear_interval(id);
        }
        id = 0;
    }

    Timer(double seconds, std::function<void(const ros::TimerEvent&)> cb) : callback(cb)
    {
        id = emscripten_set_interval(&Timer::impl_callback, 1e3*seconds, (void*)(this));
    }

    ~Timer()
    {
        if (id != 0) {
            emscripten_clear_interval(id);
        }
    }
};

struct NodeHandle
{
    EMSCRIPTEN_WEBSOCKET_T socket;
    std::list<std::string> message_queue;
    bool socket_open;
    std::unordered_map<std::string, Subscriber*> subscribers;
    std::unordered_map<std::string, Publisher*> publishers;
    std::unordered_map<std::string, ServiceClient*> service_clients;

    Timer* createTimer(double seconds, std::function<void(const ros::TimerEvent&)> cb)
    {
        return new Timer(seconds, cb);
    }

    template <typename MSG>
    Subscriber* subscribe(const std::string& topic, std::function<void(const MSG&)> callback, const std::string& id = "", int throttle_rate = -1, int queue_length = -1, int fragment_size = -1, const std::string& compression = "")
    {
        std::string message = "\"op\":\"subscribe\", \"topic\":\"" + topic + "\", \"compression\":\"cbor-raw\"";
        message += ", \"type\": \"" + std::string(ros::message_traits::DataType<MSG>::value()) + "\"";

        if (id.compare("") != 0)
        {
            message += ", \"id\":\"" + id + "\"";
        }
        if (throttle_rate > -1)
        {
            message += ", \"throttle_rate\":" + std::to_string(throttle_rate);
        }
        if (queue_length > -1)
        {
            message += ", \"queue_length\":" + std::to_string(queue_length);
        }
        if (fragment_size > -1)
        {
            message += ", \"fragment_size\":" + std::to_string(fragment_size);
        }
        message = "{" + message + "}";

        send_message(message);

        Subscriber* subscriber = new Subscriber(new SubscriberImpl<MSG>(callback));

        subscribers[topic] = subscriber;
        return subscriber;

    }

    template <typename MSG>
    Publisher* advertise(const std::string& topic, const std::string& id="")
    {
        std::string message = "\"op\":\"advertise\", \"topic\":\"" + topic + "\"";
        message += ", \"type\": \"" + std::string(ros::message_traits::DataType<MSG>::value()) + "\"";
        if (id.compare("") != 0)
        {
            message += ", \"id\":\"" + id + "\"";
        }
        message = "{" + message + "}";

        send_message(message);

        Publisher* publisher = new Publisher(new PublisherImpl<MSG>(this), topic);

        publishers[topic] = publisher;
        return publisher;
    }

    template <typename SRV>
    ServiceClient* serviceClient(const std::string& service_name, std::function<void(const typename SRV::Response&, bool result)> callback)
    {
        ServiceClient* service_client = new ServiceClient(new ServiceClientImpl<SRV>(callback, this), service_name);

        service_clients[service_name] = service_client;
        return service_client;
    }

    void websocket_open()
    {
        socket_open = true;

        for (const std::string& message : message_queue)
        {
            printf("Sending: %s\n", message.c_str());
            emscripten_websocket_send_utf8_text(socket, message.c_str());
        }
        message_queue.clear();
    }

    void websocket_close()
    {
        socket_open = false;
    }

    static EM_BOOL WebSocketOpen(int eventType, const EmscriptenWebSocketOpenEvent *e, void *userData)
    {
        printf("open(eventType=%d, userData=%d)\n", eventType, (int)userData);

        NodeHandle* nh = (NodeHandle*)(userData);
        nh->websocket_open();

        return 0;
    }

    static EM_BOOL WebSocketClose(int eventType, const EmscriptenWebSocketCloseEvent *e, void *userData)
    {
        printf("close(eventType=%d, wasClean=%d, code=%d, reason=%s, userData=%d)\n", eventType, e->wasClean, e->code, e->reason, (int)userData);

        NodeHandle* nh = (NodeHandle*)(userData);
        nh->websocket_close();

        return 0;
    }

    static EM_BOOL WebSocketError(int eventType, const EmscriptenWebSocketErrorEvent *e, void *userData)
    {
        printf("error(eventType=%d, userData=%d)\n", eventType, (int)userData);
        return 0;
    }

    static EM_BOOL WebSocketMessage(int eventType, const EmscriptenWebSocketMessageEvent *e, void *userData)
    {
        printf("message(eventType=%d, userData=%d, data=%p, numBytes=%d, isText=%d)\n", eventType, (int)userData, e->data, e->numBytes, e->isText);
        NodeHandle* nh = (NodeHandle*)(userData);
        if (e->isText) {
            printf("text data: \"%s\"\n", e->data);
            nh->handle_string(e);
        }
        else {
            nh->handle_bytes(e);
        }
        return 0;
    }

    void handle_bytes(const EmscriptenWebSocketMessageEvent* e)
    {
        // to be filled in
        std::vector<uint8_t> buffer;
        std::string topic;

        printf("binary data:");
        for(int i = 0; i < e->numBytes; ++i)
            printf(" %02X", e->data[i]);
        printf("\n");

        CborLite::Flags flags = CborLite::Flag::none;
        std::vector<uint8_t> buff_vec(e->data, e->data+e->numBytes);
        auto vpos = buff_vec.begin();
        auto vend = buff_vec.end();
        size_t nItems;
        CborLite::Tag tag, additional;
        size_t len = CborLite::decodeMapSize(vpos, vend, nItems, flags);

        printf("Number items: %zu\n", nItems);
        for (int i = 0; i < nItems; ++i) {
            std::string key;
            len += CborLite::decodeText(vpos, vend, key, flags);
            printf("Key: %s\n", key.c_str());
            auto rpos = vpos;
            CborLite::decodeTagAndAdditional(vpos, vend, tag, additional, flags);
            vpos = rpos;
            if (tag == CborLite::Major::textString) {
                std::string text;
                len += CborLite::decodeText(vpos, vend, text, flags);
                printf("Tag: %llu\n", tag);
                printf("Additional: %llu\n", additional);
                printf("Text: %s\n", text.c_str());
                if (key == "topic") {
                    topic = text;
                }
            }
            else if (tag == CborLite::Major::map) {
                size_t mItems;
                len += CborLite::decodeMapSize(vpos, vend, mItems, flags);
                printf("Number items: %zu\n", nItems);
                for (int j = 0; j < mItems; ++j) {
                    std::string mkey;
                    len += CborLite::decodeText(vpos, vend, mkey, flags);
                    printf("Key: %s\n", mkey.c_str());
                    rpos = vpos;
                    CborLite::decodeTagAndAdditional(vpos, vend, tag, additional, flags);
                    vpos = rpos;
                    if (tag == CborLite::Major::textString) {
                        std::string text;
                        len += CborLite::decodeText(vpos, vend, text, flags);
                        printf("Tag: %llu\n", tag);
                        printf("Additional: %llu\n", additional);
                        printf("Text: %s\n", text.c_str());
                    }
                    else if (tag == CborLite::Major::byteString) {
                        len += CborLite::decodeBytes(vpos, vend, buffer, flags);
                        printf("Tag: %llu\n", tag);
                        printf("Additional: %llu\n", additional);
                        printf("Bytes len: %zu\n", buffer.size());
                    }
                    else if (tag == CborLite::Major::unsignedInteger) {
                        unsigned int value;
                        len += CborLite::decodeUnsigned(vpos, vend, value, flags);
                        printf("Value: %u\n", value);
                    }
                    else {
                        printf("Unknown tag: %llu\n", tag);
                    }
                }

            }
            else {
                printf("Unknown tag: %llu\n", tag);
            }

        }
        printf("Total len: %zu\n", len);
        if (subscribers.count(topic)) {
            printf("Buffer size: %zu, calling subscriber callback\n", buffer.size());
            subscribers[topic]->callback(buffer);
        }
    }

    void handle_string(const EmscriptenWebSocketMessageEvent* e)
    {
        rapidjson::Document document;
        document.Parse((const char*)e->data);
        assert(document.IsObject());
        assert(document.HasMember("op"));
        assert(document["op"].IsString());
        std::string op = document["op"].GetString();
        if (op == "publish") {
            printf("Got string message!\n");
        }
        else if (op == "service_response") {
            printf("Got string service response!\n");
            assert(document.HasMember("values"));
            assert(document.HasMember("service"));
            assert(document["service"].IsString());
            assert(document.HasMember("result"));
            assert(document["result"].IsBool());
            std::string service_name = document["service"].GetString();
            bool result = document["result"].GetBool();
            rapidjson::StringBuffer sb;
            rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
            document["values"].Accept(writer);
            std::string json_values = sb.GetString();
            if (service_clients.count(service_name)) {
                printf("Response: %s, calling service client callback\n", json_values.c_str());
                service_clients[service_name]->callback(json_values, result);
            }
        }
        else {
            printf("Unknown operation!\n");
        }
    }

    void send_message(const std::string& message)
    {
        if (NodeHandle::socket_open) {
            emscripten_websocket_send_utf8_text(socket, message.c_str());
        }
        else {
            message_queue.push_back(message);
        }
    }

    NodeHandle()
    {
        socket_open = false;

        if (!emscripten_websocket_is_supported())
        {
            printf("WebSockets are not supported, cannot continue!\n");
            exit(1);
        }

        EmscriptenWebSocketCreateAttributes attr;
        emscripten_websocket_init_create_attributes(&attr);

        attr.url = "ws://127.0.0.1:9090/";
        attr.createOnMainThread = true;

        socket = emscripten_websocket_new(&attr);
        if (socket <= 0)
        {
            printf("WebSocket creation failed, error code %d!\n", (EMSCRIPTEN_RESULT)socket);
            exit(1);
        }

        int urlLength = 0;
        EMSCRIPTEN_RESULT res = emscripten_websocket_get_url_length(socket, &urlLength);
        assert(res == EMSCRIPTEN_RESULT_SUCCESS);
        assert(urlLength == strlen(attr.url));

        emscripten_websocket_set_onopen_callback(socket, (void*)(this), NodeHandle::WebSocketOpen);
        emscripten_websocket_set_onclose_callback(socket, (void*)(this), NodeHandle::WebSocketClose);
        emscripten_websocket_set_onerror_callback(socket, (void*)44, NodeHandle::WebSocketError);
        emscripten_websocket_set_onmessage_callback(socket, (void*)(this), NodeHandle::WebSocketMessage);
    }

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
void PublisherImpl<MSG>::publish(const MSG& msg, const std::string& topic)
{
    roscpp_json::JSONStream stream;
    ros::message_operations::Printer<MSG>::stream(stream, "", msg);
    std::string message = "\"op\":\"publish\", \"topic\":\"" + topic + "\", \"msg\":" + stream.str();
    message = std::string("{ ") + message + " }";
    nh->send_message(message);
    /*
    if (NodeHandle::socket_open) {
        emscripten_websocket_send_utf8_text(NodeHandle::socket, message.c_str());
    }
    else {
        NodeHandle::message_queue.push_back(message);
    }
    */
}

template <typename SRV>
void ServiceClientImpl<SRV>::call(const typename SRV::Request& req, const std::string& service_name)
{
    roscpp_json::JSONStream stream(false);
    ros::message_operations::Printer<typename SRV::Request>::stream(stream, "", req);
    std::string message = "\"op\":\"call_service\", \"service\":\"" + service_name + "\", \"args\":" + stream.str();
    message = std::string("{ ") + message + " }";
    nh->send_message(message);
    /*
    if (NodeHandle::socket_open) {
        emscripten_websocket_send_utf8_text(NodeHandle::socket, message.c_str());
    }
    else {
        NodeHandle::message_queue.push_back(message);
    }
    */
}

/*
std::list<std::string> NodeHandle::message_queue = {};
bool NodeHandle::socket_open = false;
EMSCRIPTEN_WEBSOCKET_T NodeHandle::socket = NULL;
std::unordered_map<std::string, Subscriber*> NodeHandle::subscribers;
std::unordered_map<std::string, Publisher*> NodeHandle::publishers;
std::unordered_map<std::string, ServiceClient*> NodeHandle::service_clients;
*/

} // namespace wasmros

#endif // ROSWASM_H
