#ifndef ROSWASM_SUBSCRIBER_H
#define ROSWASM_SUBSCRIBER_H

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
    
    void callback(std::vector<uint8_t>& buffer);

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

    void callback(std::vector<uint8_t>& buffer)
    {
        if (impl != nullptr) {
            impl->callback(buffer);
        }
        else {
            printf("Subscriber callback not initialized!\n");
        }
    }

    std::string get_id()
    {
        return id;
    }

    Subscriber(SubscriberImplBase* new_impl, const std::string& new_topic) : impl(new_impl), topic(new_topic)
    {
        id = std::to_string(size_t(impl));
    }

    Subscriber() : impl(nullptr) {}

};

} // namespace roswasm

#endif // ROSWASM_SUBSCRIBER_H
