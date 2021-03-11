#include <roswasm/roswasm.h>
#include <std_srvs/SetBool.h>

roswasm::NodeHandle* n;
roswasm::ServiceCallbackClient service;

void service_callback(const std_srvs::SetBool::Response& res, bool result)
{
    printf("Got service response with value: %s\n", res.message.c_str());
}

void loop()
{

}

extern "C" int main(int argc, char** argv)
{
    roswasm::init(argc, argv, "bool_client");

    n = new roswasm::NodeHandle();

    service = roswasm::createServiceCallbackClient<std_srvs::SetBool>(*n, "/test_bool");
    std_srvs::SetBool::Request req;
    req.data = true;
    service.call<std_srvs::SetBool>(req, service_callback);

    roswasm::Duration loop_rate(1.);
    roswasm::spinLoop(loop, loop_rate);

    return 0;
}
