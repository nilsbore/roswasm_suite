#include <roswasm/roswasm.h>
#include <roswasm/publisher.h>
#include <roscpp_json/serialize.h>

namespace roswasm {

template <typename MSG>
void PublisherImpl<MSG>::publish(const MSG& msg, const std::string& topic)
{
    std::string json_msg = roscpp_json::serialize(msg);
    std::string message = "\"op\":\"publish\", \"topic\":\"" + topic + "\", \"msg\":" + json_msg;
    message = std::string("{ ") + message + " }";
    nh->send_message(message);
}

template <typename MSG>
std::string PublisherImpl<MSG>::msg_type()
{
    return std::string(ros::message_traits::DataType<MSG>::value());
}

std::string Publisher::json_advertise_message()
{
    std::string message = "\"op\":\"advertise\", \"topic\":\"" + topic + "\", \"id\":\"" + id + "\"";
    message += ", \"type\": \"" + impl->msg_type() + "\"";
    return std::string("{") + message + "}";
}

} // namespace roswasm
