#include <roswasm/roswasm.h>
#include <roswasm/subscriber.h>
#include <ros/serialization.h>

namespace roswasm {

template <typename MSG>
void SubscriberImpl<MSG>::shutdown()
{
    nh->unsubscribe(id);
}

template <typename MSG>
void SubscriberImpl<MSG>::callback(std::vector<uint8_t>& buffer)
{
    namespace ser = ros::serialization;
    MSG msg;
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    // Fill buffer with a serialized UInt32
    ser::IStream stream(&buffer[0], buffer.size());
    ser::deserialize(stream, msg);
    impl_callback(msg);
}

template <typename MSG>
std::string SubscriberImpl<MSG>::msg_type()
{
    return std::string(ros::message_traits::DataType<MSG>::value());
}

} // namespace roswasm
