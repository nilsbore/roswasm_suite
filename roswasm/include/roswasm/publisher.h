#ifndef ROSWASM_PUBLISHER_H
#define ROSWASM_PUBLISHER_H

#include <string>

namespace roswasm {

class NodeHandleImpl;

class PublisherImplBase
{
    public:
    virtual ~PublisherImplBase() {}
    virtual std::string msg_type() = 0;
    virtual std::string json_advertise_message() = 0;
    virtual std::string get_id() = 0;
};

template <typename MSG>
class PublisherImpl : public PublisherImplBase
{
private:
    NodeHandleImpl* nh;
    std::string topic;
    std::string id;
public:
    void publish(const MSG& msg);
    std::string msg_type();

    std::string json_advertise_message()
    {
        std::string message = "\"op\":\"advertise\", \"topic\":\"" + topic + "\", \"id\":\"" + id + "\"";
        message += ", \"type\": \"" + msg_type() + "\"";
        return std::string("{") + message + "}";
    }

    std::string get_id()
    {
        return id;
    }

    PublisherImpl(NodeHandleImpl* nh, const std::string& topic) : nh(nh), topic(topic)
    {
        id = std::to_string(size_t(this));
    }

    ~PublisherImpl() {}
};

class Publisher {
    public:

    std::unique_ptr<PublisherImplBase> impl;

    template <typename MSG>
    void publish(const MSG& msg)
    {
        // this basically allows us to check that advertise was called with correct template
        PublisherImpl<MSG>* value = dynamic_cast<PublisherImpl<MSG>*>(impl.get());
        value->publish(msg);
    }

    Publisher(PublisherImplBase* impl) : impl(impl)
    {
    }

    Publisher() = default; // : impl(nullptr) {}

    Publisher(Publisher&& other) noexcept : impl(std::move(other.impl))
    {
    }

    Publisher& operator=(Publisher&& other)
    {
        impl = std::move(other.impl);
        return *this;
    }
};

} // namespace roswasm

#endif // ROSWASM_PUBLISHER_H
