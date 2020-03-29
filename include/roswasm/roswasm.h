#ifndef ROSWASM_H
#define ROSWASM_H

// assumes we're in an emscripten environment
#include <emscripten.h>
#include <emscripten/websocket.h>

#include <unordered_map>
#include <ros/serialization.h>
#include <cbor-lite/codec.h>
#include <roscpp_json/serialize.h>

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

    virtual void callback(const std::string& buffer) = 0;
    virtual ~ServiceClientImplBase() {}

};

template <typename SRV>
class ServiceClientImpl : public ServiceClientImplBase {
    public:

    std::function<void(const typename SRV::Response&)> impl_callback;

    void call(const typename SRV::Request& req, const std::string& service_name);
    
    void callback(const std::string& buffer)
    {
        std::cout << "Got service response: " << buffer << std::endl;
    }

    ServiceClientImpl(std::function<void(const typename SRV::Response&)> cb) : impl_callback(cb)
    {

    }
    ~ServiceClientImpl() {}

};

class ServiceClient {
    public:

    ServiceClientImplBase* impl;
    std::string service_name;

    void callback(const std::string& buffer)
    {
        if (impl != nullptr) {
            impl->callback(buffer);
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
    public:
    void publish(const MSG& msg, const std::string& topic);

    PublisherImpl() {}
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

struct NodeHandle
{
    static EMSCRIPTEN_WEBSOCKET_T socket;
    static std::list<std::string> message_queue;
    static bool socket_open;
    static std::unordered_map<std::string, Subscriber*> subscribers;
    static std::unordered_map<std::string, Publisher*> publishers;
    static std::unordered_map<std::string, ServiceClient*> service_clients;

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

        if (socket_open) {
            emscripten_websocket_send_utf8_text(socket, message.c_str());
        }
        else {
            message_queue.push_back(message);
        }

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

        if (socket_open) {
            emscripten_websocket_send_utf8_text(socket, message.c_str());
        }
        else {
            message_queue.push_back(message);
        }

        Publisher* publisher = new Publisher(new PublisherImpl<MSG>(), topic);

        publishers[topic] = publisher;
        return publisher;
    }

    template <typename SRV>
    ServiceClient* serviceClient(const std::string& service_name, std::function<void(const typename SRV::Response&)> callback)
    {
        ServiceClient* service_client = new ServiceClient(new ServiceClientImpl<SRV>(callback), service_name);

        service_clients[service_name] = service_client;
        return service_client;
    }

    static EM_BOOL WebSocketOpen(int eventType, const EmscriptenWebSocketOpenEvent *e, void *userData)
    {
        printf("open(eventType=%d, userData=%d)\n", eventType, (int)userData);
        socket_open = true;

        for (const std::string& message : message_queue)
        {
            printf("Sending: %s\n", message.c_str());
            emscripten_websocket_send_utf8_text(socket, message.c_str());
        }
        message_queue.clear();

        return 0;
    }

    static EM_BOOL WebSocketClose(int eventType, const EmscriptenWebSocketCloseEvent *e, void *userData)
    {
        printf("close(eventType=%d, wasClean=%d, code=%d, reason=%s, userData=%d)\n", eventType, e->wasClean, e->code, e->reason, (int)userData);
        socket_open = false;
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
        if (e->isText)
            printf("text data: \"%s\"\n", e->data);
        else
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
        return 0;
    }

    void send_message(const std::string& msg)
    {
        emscripten_websocket_send_utf8_text(socket, msg.c_str());
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

        emscripten_websocket_set_onopen_callback(socket, (void*)42, NodeHandle::WebSocketOpen);
        emscripten_websocket_set_onclose_callback(socket, (void*)43, NodeHandle::WebSocketClose);
        emscripten_websocket_set_onerror_callback(socket, (void*)44, NodeHandle::WebSocketError);
        emscripten_websocket_set_onmessage_callback(socket, (void*)45, NodeHandle::WebSocketMessage);
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
    if (NodeHandle::socket_open) {
        emscripten_websocket_send_utf8_text(NodeHandle::socket, message.c_str());
    }
    else {
        NodeHandle::message_queue.push_back(message);
    }
}

template <typename SRV>
void ServiceClientImpl<SRV>::call(const typename SRV::Request& req, const std::string& service_name)
{
    roscpp_json::JSONStream stream(false);
    ros::message_operations::Printer<typename SRV::Request>::stream(stream, "", req);
    std::string message = "\"op\":\"call_service\", \"service\":\"" + service_name + "\", \"args\":" + stream.str();
    message = std::string("{ ") + message + " }";
    if (NodeHandle::socket_open) {
        emscripten_websocket_send_utf8_text(NodeHandle::socket, message.c_str());
    }
    else {
        NodeHandle::message_queue.push_back(message);
    }
}

std::list<std::string> NodeHandle::message_queue = {};
bool NodeHandle::socket_open = false;
EMSCRIPTEN_WEBSOCKET_T NodeHandle::socket = NULL;
std::unordered_map<std::string, Subscriber*> NodeHandle::subscribers;
std::unordered_map<std::string, Publisher*> NodeHandle::publishers;
std::unordered_map<std::string, ServiceClient*> NodeHandle::service_clients;

} // namespace wasmros

#endif // ROSWASM_H
