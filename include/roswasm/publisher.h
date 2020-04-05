#ifndef ROSWASM_PUBLISHER_H
#define ROSWASM_PUBLISHER_H

namespace roswasm {

class NodeHandle;

class PublisherImplBase
{
    public:
    virtual ~PublisherImplBase() {}
};

template <typename MSG>
class PublisherImpl : public PublisherImplBase
{
private:
    NodeHandle* nh;
public:
    void publish(const MSG& msg, const std::string& topic);

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
