#include <roswasm/roswasm.h>
#include <roscpp_json_serialize/local_rapidjson.h>
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

std::ostream& operator<<(std::ostream& os, const Time &rhs)
{
    os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    return os;
}

void normalizeSecNSec(uint64_t& sec, uint64_t& nsec)
{
    uint64_t nsec_part = nsec % 1000000000UL;
    uint64_t sec_part = nsec / 1000000000UL;

    if (sec + sec_part > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("Time is out of dual 32-bit range");

    sec += sec_part;
    nsec = nsec_part;
}

void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
{
    uint64_t sec64 = sec;
    uint64_t nsec64 = nsec;

    normalizeSecNSec(sec64, nsec64);

    sec = (uint32_t)sec64;
    nsec = (uint32_t)nsec64;
}

void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
{
    int64_t nsec_part = nsec % 1000000000L;
    int64_t sec_part = sec + nsec / 1000000000L;
    if (nsec_part < 0)
    {
        nsec_part += 1000000000L;
        --sec_part;
    }

    if (sec_part < 0 || sec_part > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("Time is out of dual 32-bit range");

    sec = sec_part;
    nsec = nsec_part;
}


void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec)
{
    int64_t nsec_part = nsec % 1000000000L;
    int64_t sec_part = sec + nsec / 1000000000L;
    if (nsec_part < 0)
    {
        nsec_part += 1000000000L;
        --sec_part;
    }

    if (sec_part < std::numeric_limits<int32_t>::min() || sec_part > std::numeric_limits<int32_t>::max())
        throw std::runtime_error("Duration is out of dual 32-bit range");

    sec = sec_part;
    nsec = nsec_part;
}

void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec)
{
    int64_t sec64 = sec;
    int64_t nsec64 = nsec;

    normalizeSecNSecSigned(sec64, nsec64);

    sec = (int32_t)sec64;
    nsec = (int32_t)nsec64;
}

}

namespace roswasm {

void init(int argc, const char* const* argv, const std::string& arg)
{
}

void spinLoop(void(*loop)())
{
    emscripten_set_main_loop(loop, -1, 1);
}

void spinLoop(void(*loop)(), roswasm::Duration loop_rate)
{
    emscripten_set_main_loop(loop, int(1./loop_rate.toSec()), 1);
}

void shutdown()
{
    emscripten_exit_with_live_runtime();
}

bool NodeHandleImpl::debug_print = false;

void NodeHandleImpl::unsubscribe(const std::string& id)
{
    if (subscribers.count(id) == 0) {
        return;
    }
    if (NodeHandleImpl::socket_open) {
        std::string message = subscribers[id]->json_unsubscribe_message();
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }
    subscribers.erase(id);
}

void NodeHandleImpl::websocket_open()
{
    socket_open = true;

    printf("Connection open, clearing message queue!\n");

    for (std::pair<const std::string, SubscriberImplBase*>& sub : subscribers) {
        std::string message = sub.second->json_subscribe_message();
        printf("To rosbridge sending: %s\n", message.c_str());
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }

    for (std::pair<const std::string, PublisherImplBase*>& pub : publishers) {
        std::string message = pub.second->json_advertise_message();
        printf("To rosbridge sending: %s\n", message.c_str());
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }

    for (const std::string& message : message_queue) {
        printf("To rosbridge sending: %s\n", message.c_str());
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }
    message_queue.clear();
}

void NodeHandleImpl::websocket_close()
{
    socket_open = false;
    emscripten_websocket_delete(socket);
    // try again in 5 seconds
    timer.start();
}

std::string NodeHandleImpl::get_websocket_url()
{
    return std::string("ws://") + rosbridge_ip + ":" + rosbridge_port + "/";
}

void NodeHandleImpl::try_websocket_connect()
{
    timer.stop();

    EmscriptenWebSocketCreateAttributes attr;
    emscripten_websocket_init_create_attributes(&attr);

    std::string url = get_websocket_url();
    attr.url = url.c_str(); // "ws://127.0.0.1:9090/";
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
    // assert(urlLength == strlen(attr.url));

    emscripten_websocket_set_onopen_callback(socket, (void*)(this), NodeHandleImpl::WebSocketOpen);
    emscripten_websocket_set_onclose_callback(socket, (void*)(this), NodeHandleImpl::WebSocketClose);
    emscripten_websocket_set_onerror_callback(socket, (void*)44, NodeHandleImpl::WebSocketError);
    emscripten_websocket_set_onmessage_callback(socket, (void*)(this), NodeHandleImpl::WebSocketMessage);
}

EM_BOOL NodeHandleImpl::WebSocketOpen(int eventType, const EmscriptenWebSocketOpenEvent *e, void *userData)
{
    printf("open(eventType=%d, userData=%d)\n", eventType, (int)userData);

    NodeHandleImpl* nh = (NodeHandleImpl*)(userData);
    nh->websocket_open();

    return 0;
}

EM_BOOL NodeHandleImpl::WebSocketClose(int eventType, const EmscriptenWebSocketCloseEvent *e, void *userData)
{
    printf("close(eventType=%d, wasClean=%d, code=%d, reason=%s, userData=%d)\n", eventType, e->wasClean, e->code, e->reason, (int)userData);

    NodeHandleImpl* nh = (NodeHandleImpl*)(userData);
    nh->websocket_close();

    return 0;
}

EM_BOOL NodeHandleImpl::WebSocketError(int eventType, const EmscriptenWebSocketErrorEvent *e, void *userData)
{
    printf("error(eventType=%d, userData=%d)\n", eventType, (int)userData);
    return 0;
}

EM_BOOL NodeHandleImpl::WebSocketMessage(int eventType, const EmscriptenWebSocketMessageEvent *e, void *userData)
{
    if (debug_print) printf("message(eventType=%d, userData=%d, data=%p, numBytes=%d, isText=%d)\n", eventType, (int)userData, e->data, e->numBytes, e->isText);
    NodeHandleImpl* nh = (NodeHandleImpl*)(userData);
    if (e->isText) {
        if (debug_print) printf("text data: \"%s\"\n", e->data);
        nh->handle_string(e);
    }
    else {
        nh->handle_bytes(e);
    }
    return 0;
}

void NodeHandleImpl::handle_bytes(const EmscriptenWebSocketMessageEvent* e)
{
    // to be filled in
    std::vector<uint8_t> buffer;
    std::string topic;

    CborLite::Flags flags = CborLite::Flag::none;
    std::vector<uint8_t> buff_vec(e->data, e->data+e->numBytes);
    auto vpos = buff_vec.begin();
    auto vend = buff_vec.end();
    size_t nItems;
    CborLite::Tag tag, additional;
    size_t len = CborLite::decodeMapSize(vpos, vend, nItems, flags);

    if (debug_print) printf("Number items: %zu\n", nItems);
    for (int i = 0; i < nItems; ++i) {
        std::string key;
        len += CborLite::decodeText(vpos, vend, key, flags);
        if (debug_print) printf("Key: %s\n", key.c_str());
        auto rpos = vpos;
        CborLite::decodeTagAndAdditional(vpos, vend, tag, additional, flags);
        vpos = rpos;
        if (tag == CborLite::Major::textString) {
            std::string text;
            len += CborLite::decodeText(vpos, vend, text, flags);
            if (debug_print) {
                printf("Tag: %llu\n", tag);
                printf("Additional: %llu\n", additional);
                printf("Text: %s\n", text.c_str());
            }
            if (key == "topic") {
                topic = text;
            }
        }
        else if (tag == CborLite::Major::map) {
            size_t mItems;
            len += CborLite::decodeMapSize(vpos, vend, mItems, flags);
            if (debug_print) printf("Number items: %zu\n", nItems);
            for (int j = 0; j < mItems; ++j) {
                std::string mkey;
                len += CborLite::decodeText(vpos, vend, mkey, flags);
                if (debug_print) printf("Key: %s\n", mkey.c_str());
                rpos = vpos;
                CborLite::decodeTagAndAdditional(vpos, vend, tag, additional, flags);
                vpos = rpos;
                if (tag == CborLite::Major::textString) {
                    std::string text;
                    len += CborLite::decodeText(vpos, vend, text, flags);
                    if (debug_print) {
                        printf("Tag: %llu\n", tag);
                        printf("Additional: %llu\n", additional);
                        printf("Text: %s\n", text.c_str());
                    }
                }
                else if (tag == CborLite::Major::byteString) {
                    len += CborLite::decodeBytes(vpos, vend, buffer, flags);
                    if (debug_print) {
                        printf("Tag: %llu\n", tag);
                        printf("Additional: %llu\n", additional);
                        printf("Bytes len: %zu\n", buffer.size());
                    }
                }
                else if (tag == CborLite::Major::unsignedInteger) {
                    unsigned int value;
                    len += CborLite::decodeUnsigned(vpos, vend, value, flags);
                    if (debug_print) printf("Value: %u\n", value);
                }
                else {
                    if (debug_print) printf("Unknown tag: %llu\n", tag);
                }
            }

        }
        else {
            if (debug_print) printf("Unknown tag: %llu\n", tag);
        }

    }
    if (debug_print) printf("Total len: %zu\n", len);
    if (debug_print) printf("Buffer size: %zu, calling subscriber callback\n", buffer.size());
    //subscribers[id]->callback(buffer);
    for (std::pair<const std::string, SubscriberImplBase*>& sub : subscribers) {
        if (sub.second->get_topic() == topic) {
            sub.second->callback(buffer);
        }
    }
}

void NodeHandleImpl::handle_string(const EmscriptenWebSocketMessageEvent* e)
{
    rapidjson::Document document;
    document.Parse((const char*)e->data);
    assert(document.IsObject());
    assert(document.HasMember("op"));
    assert(document["op"].IsString());
    std::string op = document["op"].GetString();
    if (op == "service_response") {
        printf("Rosbridge sent service response!\n");
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
            //printf("Response: %s, calling service client callback\n", json_values.c_str());
            service_clients[id]->callback(json_values, result);
        }
    }
    else if (op == "publish") {
        printf("ERROR: Rosbridge sent string message!\n");
    }
    else {
        printf("ERROR: Rosbridge sent unknown operation %s!\n", op.c_str());
    }
}

void NodeHandleImpl::send_message(const std::string& message)
{
    if (NodeHandleImpl::socket_open) {
        emscripten_websocket_send_utf8_text(socket, message.c_str());
    }
    else {
        message_queue.push_back(message);
    }
}

NodeHandleImpl::NodeHandleImpl(const std::string& rosbridge_ip, const std::string& rosbridge_port)
    : rosbridge_ip(rosbridge_ip), rosbridge_port(rosbridge_port)
{
    debug_print = false;
    socket_open = false;

    if (!emscripten_websocket_is_supported())
    {
        printf("WebSockets are not supported, cannot continue!\n");
        exit(1);
    }

    timer = createTimer(roswasm::Duration(5.), std::bind(&NodeHandleImpl::try_websocket_connect, this));
    timer.stop();

    /*
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

    emscripten_websocket_set_onopen_callback(socket, (void*)(this), NodeHandleImpl::WebSocketOpen);
    emscripten_websocket_set_onclose_callback(socket, (void*)(this), NodeHandleImpl::WebSocketClose);
    emscripten_websocket_set_onerror_callback(socket, (void*)44, NodeHandleImpl::WebSocketError);
    emscripten_websocket_set_onmessage_callback(socket, (void*)(this), NodeHandleImpl::WebSocketMessage);
    */
    try_websocket_connect();
}

} // namespace roswasm

