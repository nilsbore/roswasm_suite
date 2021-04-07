#ifndef ROSWASM_SUBSCRIBER_H
#define ROSWASM_SUBSCRIBER_H

#include <string>
#include <functional>

namespace roswasm {

class SubscriberImplBase {
    public:

    virtual void callback(std::vector<uint8_t>& buffer) = 0;
    virtual void shutdown() = 0;
    virtual std::string json_subscribe_message() = 0;
    virtual std::string json_unsubscribe_message() = 0;
    virtual std::string msg_type() = 0;
    virtual std::string get_id() = 0;
    virtual std::string get_topic() = 0;
    virtual ~SubscriberImplBase() {}

};

template <typename MSG>
class SubscriberImpl : public SubscriberImplBase {
private:
    NodeHandleImpl* nh;
    std::string topic;
    std::string id;
    int throttle_rate;
    int queue_length;
    int fragment_size;
public:
    void shutdown();

    std::function<void(const MSG&)> impl_callback;
    
    void callback(std::vector<uint8_t>& buffer);
    std::string msg_type();

    std::string json_subscribe_message()
    {
        std::string message = "\"op\":\"subscribe\", \"topic\":\"" + topic + "\", \"compression\":\"cbor-raw\", \"id\":\"" + id + "\"";
        message += ", \"type\": \"" + msg_type() + "\"";

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
    
    std::string get_topic()
    {
        return topic;
    }

    SubscriberImpl(std::function<void(const MSG&)> cb, roswasm::NodeHandleImpl* nh, const std::string& topic, int throttle_rate, int queue_length, int fragment_size) : impl_callback(cb), nh(nh), topic(topic), throttle_rate(throttle_rate), queue_length(queue_length), fragment_size(fragment_size)
    {
        id = std::to_string(size_t(this));
    }
    ~SubscriberImpl()
    {
       shutdown(); 
    }

};

class Subscriber {
public:
    //SubscriberImplBase* impl;
    std::unique_ptr<SubscriberImplBase> impl;

    void shutdown()
    {
        impl.reset();
    }

    std::string getTopic()
    {
        if (impl) {
            return impl->get_topic();
        }
        else {
            return "";
        }
    }

    Subscriber(Subscriber&& other) noexcept : impl(std::move(other.impl))
    {
    }

    Subscriber& operator=(Subscriber&& other)
    {
        impl = std::move(other.impl);
        return *this;
    }

    Subscriber(SubscriberImplBase* impl) : impl(impl)
    {
    }

    Subscriber() = default;
    //Subscriber() : impl() {}

    ~Subscriber()
    {
        //delete impl;
        //impl = nullptr;
    }

};

} // namespace roswasm

#endif // ROSWASM_SUBSCRIBER_H
