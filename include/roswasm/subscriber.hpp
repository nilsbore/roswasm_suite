#include <ros/serialization.h>

namespace roswasm {

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

std::string Subscriber::json_subscribe_message()
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

} // namespace roswasm
