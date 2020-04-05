#ifndef ROSWASM_SERVICE_CLIENT_H
#define ROSWASM_SERVICE_CLIENT_H

namespace roswasm {

class NodeHandle;

class ServiceClientImplBase {
    public:
    std::string id;

    virtual void callback(const std::string& buffer, bool result) = 0;

    ServiceClientImplBase()
    {
        id = std::to_string(size_t(this));
    }

    virtual ~ServiceClientImplBase() {}

};

template <typename SRV>
class ServiceClientImpl : public ServiceClientImplBase {
private:

    NodeHandle* nh;

public:

    std::function<void(const typename SRV::Response&, bool)> impl_callback;

    void call(const typename SRV::Request& req, const std::string& service_name);
    
    void callback(const std::string& buffer, bool result);

    ServiceClientImpl(std::function<void(const typename SRV::Response&, bool)> cb, NodeHandle* nh) : impl_callback(cb), nh(nh)
    {
    }
    ~ServiceClientImpl() {}

};

class ServiceClient {
    public:

    ServiceClientImplBase* impl;
    std::string service_name;

    void callback(const std::string& buffer, bool result)
    {
        if (impl != nullptr) {
            impl->callback(buffer, result);
        }
        else {
            printf("ServiceClient callback not initialized!\n");
        }
    }

    template <typename SRV>
    void call(const typename SRV::Request& req)
    {
        ServiceClientImpl<SRV>* value = dynamic_cast<ServiceClientImpl<SRV>*>(impl);
        value->call(req, service_name);
    }

    std::string get_id()
    {
        return impl->id;
    }

    ServiceClient(ServiceClientImplBase* new_impl, const std::string& new_service_name) : impl(new_impl), service_name(new_service_name)
    {
    }

    ServiceClient() : impl(nullptr) {}

};

} // namespace roswasm

#endif // ROSWASM_SERVICE_CLIENT_H
