#ifndef ROSWASM_SERVICE_CLIENT_H
#define ROSWASM_SERVICE_CLIENT_H

#include <string>
#include <functional>

namespace roswasm {

class NodeHandleImpl;

class ServiceCallbackClientImplBase {
    public:
    std::string id;

    virtual void callback(const std::string& buffer, bool result) = 0;

    std::string get_id()
    {
        return id;
    }

    ServiceCallbackClientImplBase()
    {
        id = std::to_string(size_t(this));
    }

    virtual ~ServiceCallbackClientImplBase() {}

};

template <typename SRV>
class ServiceCallbackClientImpl : public ServiceCallbackClientImplBase {
private:

    NodeHandleImpl* nh;
    std::string service_name;
    using CallbT = std::function<void(const typename SRV::Response&, bool)>;
    CallbT impl_callback;

public:

    void call(const typename SRV::Request& req, CallbT cb);
    
    void callback(const std::string& buffer, bool result);

    //ServiceCallbackClientImpl(std::function<void(const typename SRV::Response&, bool)> cb, NodeHandleImpl* nh, const std::string& service_name) : impl_callback(cb), nh(nh), service_name(service_name)
    ServiceCallbackClientImpl(NodeHandleImpl* nh, const std::string& service_name) : nh(nh), service_name(service_name)
    {
    }
    ~ServiceCallbackClientImpl() {}

};

class ServiceCallbackClient {
    public:

    std::unique_ptr<ServiceCallbackClientImplBase> impl;

    template <typename SRV>
    void call(const typename SRV::Request& req, std::function<void(const typename SRV::Response&, bool)> cb)
    {
        ServiceCallbackClientImpl<SRV>* value = dynamic_cast<ServiceCallbackClientImpl<SRV>*>(impl.get());
        value->call(req, cb);
    }

    ServiceCallbackClient(ServiceCallbackClientImplBase* impl) : impl(impl)
    {
    }

    ServiceCallbackClient() = default; // : impl(nullptr) {}

    ServiceCallbackClient(ServiceCallbackClient&& other) noexcept : impl(std::move(other.impl))
    {
    }

    ServiceCallbackClient& operator=(ServiceCallbackClient&& other)
    {
        impl = std::move(other.impl);
        return *this;
    }

};

} // namespace roswasm

#endif // ROSWASM_SERVICE_CLIENT_H
