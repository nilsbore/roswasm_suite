#include <roswasm/roswasm.h>
#include <roswasm/service_client.h>
#include <roscpp_json_serialize/serialize.h>
#include <roscpp_json_serialize/deserialize.h>
#include <std_srvs/SetBool.h>

namespace roswasm {

template <typename SRV>
void ServiceCallbackClientImpl<SRV>::call(const typename SRV::Request& req, CallbT cb)
{
    impl_callback = cb;
    std::string json_msg = roscpp_json::serialize(req, false);
    std::string message = "\"op\":\"call_service\", \"service\":\"" + service_name + "\", \"args\":" + json_msg + ", \"id\":\"" + id + "\"";
    message = std::string("{ ") + message + " }";
    nh->send_message(message);
}

template <>
void ServiceCallbackClientImpl<std_srvs::SetBool>::call(const std_srvs::SetBool::Request& req, CallbT cb);

template <typename SRV>
void ServiceCallbackClientImpl<SRV>::callback(const std::string& buffer, bool result)
{
    std::cout << "Got service response: " << buffer << std::endl;
    typename SRV::Response res;
    if (result) {
        res = roscpp_json::deserialize<typename SRV::Response>(buffer);
    }
    impl_callback(res, result);
}

template <>
void ServiceCallbackClientImpl<std_srvs::SetBool>::callback(const std::string& buffer, bool result);

} // namespace roswasm
