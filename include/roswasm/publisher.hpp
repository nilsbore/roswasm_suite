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

} // namespace roswasm
