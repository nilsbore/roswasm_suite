#ifndef ROSWASM_PUBLISHER_H
#define ROSWASM_PUBLISHER_H

#include <string>

namespace roswasm {

class NodeHandle;

class PublisherImplBase
{
    public:
    virtual ~PublisherImplBase() {}
    virtual std::string msg_type() = 0;
};

template <typename MSG>
class PublisherImpl : public PublisherImplBase
{
private:
    NodeHandle* nh;
public:
    void publish(const MSG& msg, const std::string& topic);
    std::string msg_type();

    PublisherImpl(NodeHandle* nh) : nh(nh) {}
    ~PublisherImpl() {}
};

class Publisher {
    public:

    PublisherImplBase* impl;
    std::string topic;
    std::string id;

    template <typename MSG>
    void publish(const MSG& msg)
    {
        // this basically allows us to check that advertise was called with correct template
        PublisherImpl<MSG>* value = dynamic_cast<PublisherImpl<MSG>*>(impl);
        value->publish(msg, topic);
    }

    std::string json_advertise_message()
    {
        std::string message = "\"op\":\"advertise\", \"topic\":\"" + topic + "\", \"id\":\"" + id + "\"";
        message += ", \"type\": \"" + impl->msg_type() + "\"";
        return std::string("{") + message + "}";
    }

    std::string get_id()
    {
        return id;
    }

    Publisher(PublisherImplBase* new_impl, const std::string& new_topic) : impl(new_impl), topic(new_topic)
    {
        id = std::to_string(size_t(impl));
    }

    Publisher() : impl(nullptr) {}
};

} // namespace roswasm

#endif // ROSWASM_PUBLISHER_H
