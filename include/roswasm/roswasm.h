#ifndef ROSWASM_H
#define ROSWASM_H

// assumes we're in an emscripten environment
#include <emscripten.h>
#include <emscripten/websocket.h>

#include <unordered_map>
#include <ros/serialization.h>
#include "cbor-lite/include/cbor-lite/codec.h"

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

    std::function<void(const MSG& msg)> impl_callback;
    
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

    SubscriberImpl(std::function<void(const MSG& msg)> cb) : impl_callback(cb)
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

class Publisher;

struct NodeHandle
{
    static EMSCRIPTEN_WEBSOCKET_T socket;
    static std::list<std::string> message_queue;
    static bool socket_open;
    static std::unordered_map<std::string, Subscriber*> subscribers;
    //static std::unordered_map<std::string, Publisher*> publishers;

    /*
    void advertise(const std::string& client_name, const std::string& topic, const std::string& type, const std::string& id = "")
    {
        std::unordered_map<std::string, std::shared_ptr<WsClient>>::iterator it = client_map.find(client_name);
        if (it != client_map.end())
        {
        std::string message = "\"op\":\"advertise\", \"topic\":\"" + topic + "\", \"type\":\"" + type + "\"";

        if (id.compare("") != 0)
        {
        message += ", \"id\":\"" + id + "\"";
        }
        message = "{" + message + "}";

        start(client_name, it->second, message);
        }
#ifdef DEBUG
        else
        {
        std::cerr << client_name << "has not been created" << std::endl;
        }
#endif
    }
    */

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
        std::string message = "\"op\":\"advertise\", \"topic\":\"" + topic + "\", \"compression\":\"cbor-raw\"";
        message += ", \"type\": \"" + std::string(ros::message_traits::DataType<MSG>::value()) + "\"";
        if (id.compare("") != 0)
        {
            message += ", \"id\":\"" + id + "\"";
        }
        message = "{" + message + "}";

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
        //emscripten_websocket_send_utf8_text(socket, "hello on the other side\r\n\r\n");
        //emscripten_websocket_send_utf8_text(socket, "\r\n\r\n");


        //char data[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
        //emscripten_websocket_send_binary(e->socket, data, sizeof(data));

        //emscripten_websocket_close(e->socket, 0, 0);
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
            size_t nItems = 3; //0;
            //uint8_t* pos = e->data;
            //uint8_t* end = e->data+e->numBytes;
            std::vector<uint8_t> buff_vec(e->data, e->data+e->numBytes);
            auto vpos = buff_vec.begin();
            auto vend = buff_vec.end();
            CborLite::Tag tag, additional;
            //len += CborLite::decodeTagAndAdditional(vpos, vend, tag, additional, flags);
            //assert(tag == CborLite::Major::map);
            size_t len = CborLite::decodeMapSize(vpos, vend, nItems, flags);

            //size_t len = 0;
            //auto len = CborLite::decodeArraySize(vpos, vend, nItems, flags);
            //if (nItems != 4) throw Exception("not the right number of items");
            printf("Number items: %zu\n", nItems);
            //vector<pair<string, string> > tag_values;
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
                            //std::string text;
                            len += CborLite::decodeBytes(vpos, vend, buffer, flags);
                            printf("Tag: %llu\n", tag);
                            printf("Additional: %llu\n", additional);
                            //printf("Text: %s\n", text.c_str());
                            printf("Bytes len: %zu\n", buffer.size());
                            /*
                            namespace ser = ros::serialization;

                            std_msgs::String msg;
                            uint32_t serial_size = ros::serialization::serializationLength(msg);
                            // Fill buffer with a serialized UInt32
                            ser::IStream stream(&buffer[0], buffer.size());
                            ser::deserialize(stream, msg);
                            printf("Message deserialized: %s\n", msg.data.c_str());
                            */
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
            /*
            unsigned long type;
            len += CborLite::decodeUnsigned(pos, end, type, flags);
            m.type = static_cast<Message::Type>(type);
            len += CborLite::decodeBool(pos, end, m.accepted, flags);
            len += CborLite::decodeBytes(pos, end, m.id);
            len += CborLite::decodeText(pos, end, m.payload, flags);
            return len;
            */

            //emscripten_websocket_delete(e->socket);
            //exit(0);
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

        //attr.url = "ws://localhost:9090/";
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
        //printf("Urllength: %d, Str len: %zu", urlLength, strlen(attr.url));
        assert(res == EMSCRIPTEN_RESULT_SUCCESS);
        assert(urlLength == strlen(attr.url));

        //a = A(std::bind(&B::function, this, _1));
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

class PublisherImplBase
{

};

template <typename MSG>
class PublisherImpl : PublisherImplBase
{
    void publish(const MSG& msg)
    {
        std::string message;
        emscripten_websocket_send_utf8_text(NodeHandle::socket, message.c_str());
    }
};

class Publisher {
    public:

    PublisherImplBase* impl;

    template <typename MSG>
    void publish(const MSG& msg)
    {
        // this basically allows us to check that advertise was called with correct template
        PublisherImpl<MSG>* value = dynamic_cast<PublisherImpl<MSG>*>(impl);
        value->publish(msg);
    }
};

std::list<std::string> NodeHandle::message_queue = {};
bool NodeHandle::socket_open = false;
EMSCRIPTEN_WEBSOCKET_T NodeHandle::socket = NULL;
std::unordered_map<std::string, Subscriber*> NodeHandle::subscribers;

} // namespace wasmros

#endif // ROSWASM_H
