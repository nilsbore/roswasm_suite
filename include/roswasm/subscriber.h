#ifndef ROSWASM_SUBSCRIBER_H
#define ROSWASM_SUBSCRIBER_H

#include <string>
#include <functional>

namespace roswasm {

class SubscriberImplBase {
    public:

    virtual void callback(std::vector<uint8_t>& buffer) = 0;
    virtual void shutdown(const std::string& id) = 0;
    virtual std::string msg_type() = 0;
    virtual ~SubscriberImplBase() {}

};

template <typename MSG>
class SubscriberImpl : public SubscriberImplBase {
private:
    NodeHandle* nh;
public:
    void shutdown(const std::string& id);

    std::function<void(const MSG&)> impl_callback;
    
    void callback(std::vector<uint8_t>& buffer);
    std::string msg_type();

    SubscriberImpl(std::function<void(const MSG&)> cb, roswasm::NodeHandle* nh) : impl_callback(cb), nh(nh)
    {

    }
    ~SubscriberImpl() {}

};

class Subscriber {
public:
    SubscriberImplBase* impl;
    std::string topic;
    std::string id;
    int throttle_rate;
    int queue_length;
    int fragment_size;


    void callback(std::vector<uint8_t>& buffer)
    {
        if (impl != nullptr) {
            impl->callback(buffer);
        }
        else {
            printf("Subscriber callback not initialized!\n");
        }
    }

    std::string json_subscribe_message()
    {
        std::string message = "\"op\":\"subscribe\", \"topic\":\"" + topic + "\", \"compression\":\"cbor-raw\", \"id\":\"" + id + "\"";
        message += ", \"type\": \"" + impl->msg_type() + "\"";

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

        return std::string("{") + message + "}";
    }

    std::string json_unsubscribe_message()
    {
        std::string message = "\"op\":\"unsubscribe\", \"topic\":\"" + topic + "\", \"id\":\"" + id + "\"";
        return std::string("{") + message + "}";
    }

    std::string get_id()
    {
        return id;
    }

    Subscriber(SubscriberImplBase* new_impl, const std::string& new_topic, int throttle_rate, int queue_length, int fragment_size)
        : impl(new_impl), topic(new_topic), throttle_rate(throttle_rate), queue_length(queue_length), fragment_size(fragment_size)
    {
        id = std::to_string(size_t(impl));
    }

    Subscriber() : impl(nullptr) {}

    ~Subscriber()
    {
        impl->shutdown(id);
        delete impl;
        impl = nullptr;
    }

};

} // namespace roswasm

#endif // ROSWASM_SUBSCRIBER_H
