#ifndef ROSWASM_SUBSCRIBER_H
#define ROSWASM_SUBSCRIBER_H

#include <string>
#include <functional>

namespace roswasm {

class SubscriberImplBase {
    public:

    virtual void callback(std::vector<uint8_t>& buffer) = 0;
    virtual std::string msg_type() = 0;
    virtual ~SubscriberImplBase() {}

};

template <typename MSG>
class SubscriberImpl : public SubscriberImplBase {
    public:

    std::function<void(const MSG&)> impl_callback;
    
    void callback(std::vector<uint8_t>& buffer);
    std::string msg_type();

    SubscriberImpl(std::function<void(const MSG&)> cb) : impl_callback(cb)
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

    std::string json_subscribe_message();

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

};

} // namespace roswasm

#endif // ROSWASM_SUBSCRIBER_H
