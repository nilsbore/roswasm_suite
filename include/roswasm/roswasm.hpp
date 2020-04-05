#include <roswasm/roswasm.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <cbor-lite/codec.h>

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

namespace roswasm {

template <typename MSG>
Subscriber* NodeHandle::subscribe(const std::string& topic, std::function<void(const MSG&)> callback, int throttle_rate, int queue_length, int fragment_size)
{
    Subscriber* subscriber = new Subscriber(new SubscriberImpl<MSG>(callback), topic);
    std::string id = subscriber->get_id();

    std::string message = "\"op\":\"subscribe\", \"topic\":\"" + topic + "\", \"compression\":\"cbor-raw\", \"id\":\"" + id + "\"";
    //std::string message = "\"op\":\"subscribe\", \"topic\":\"" + topic + "\", \"id\":\"" + id + "\"";
    message += ", \"type\": \"" + std::string(ros::message_traits::DataType<MSG>::value()) + "\"";

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

    subscribers[id] = subscriber;
    return subscriber;
}

template <typename MSG>
Publisher* NodeHandle::advertise(const std::string& topic, const std::string& id)
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

void NodeHandle::websocket_open()
{
    socket_open = true;

    for (const std::string& message : message_queue)
    {
        printf("Sending: %s\n", message.c_str());
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }
    message_queue.clear();
}

void NodeHandle::websocket_close()
{
    socket_open = false;
}

EM_BOOL NodeHandle::WebSocketOpen(int eventType, const EmscriptenWebSocketOpenEvent *e, void *userData)
{
    printf("open(eventType=%d, userData=%d)\n", eventType, (int)userData);

    NodeHandle* nh = (NodeHandle*)(userData);
    nh->websocket_open();

    return 0;
}

EM_BOOL NodeHandle::WebSocketClose(int eventType, const EmscriptenWebSocketCloseEvent *e, void *userData)
{
    printf("close(eventType=%d, wasClean=%d, code=%d, reason=%s, userData=%d)\n", eventType, e->wasClean, e->code, e->reason, (int)userData);

    NodeHandle* nh = (NodeHandle*)(userData);
    nh->websocket_close();

    return 0;
}

EM_BOOL NodeHandle::WebSocketError(int eventType, const EmscriptenWebSocketErrorEvent *e, void *userData)
{
    printf("error(eventType=%d, userData=%d)\n", eventType, (int)userData);
    return 0;
}

EM_BOOL NodeHandle::WebSocketMessage(int eventType, const EmscriptenWebSocketMessageEvent *e, void *userData)
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

void NodeHandle::handle_bytes(const EmscriptenWebSocketMessageEvent* e)
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
    printf("Buffer size: %zu, calling subscriber callback\n", buffer.size());
    //subscribers[id]->callback(buffer);
    for (std::pair<const std::string, Subscriber*>& sub : subscribers) {
        if (sub.second->topic == topic) {
            sub.second->callback(buffer);
        }
    }
}

void NodeHandle::handle_string(const EmscriptenWebSocketMessageEvent* e)
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
        assert(document.HasMember("id"));
        assert(document["id"].IsString());
        assert(document.HasMember("result"));
        assert(document["result"].IsBool());
        std::string service_name = document["service"].GetString();
        bool result = document["result"].GetBool();
        rapidjson::StringBuffer sb;
        rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
        document["values"].Accept(writer);
        std::string json_values = sb.GetString();
        std::string id = document["id"].GetString();
        if (service_clients.count(id)) {
            printf("Response: %s, calling service client callback\n", json_values.c_str());
            service_clients[id]->callback(json_values, result);
        }
    }
    else {
        printf("Unknown operation!\n");
    }
}

void NodeHandle::send_message(const std::string& message)
{
    if (NodeHandle::socket_open) {
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }
    else {
        message_queue.push_back(message);
    }
}

NodeHandle::NodeHandle()
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

} // namespace roswasm
