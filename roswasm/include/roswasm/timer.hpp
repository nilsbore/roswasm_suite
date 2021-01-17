#include <roswasm/timer.h>
#include <emscripten.h>

namespace roswasm {

TimerImpl::TimerImpl(double seconds, std::function<void(const ros::TimerEvent&)> cb) : callback(cb)
{
    id = emscripten_set_interval(&TimerImpl::impl_callback, 1e3*seconds, (void*)(this));
}

TimerImpl::~TimerImpl()
{
    if (id != 0) {
        emscripten_clear_interval(id);
    }
}

void Timer::stop()
{
    if (impl != nullptr) {
        delete impl;
        impl = nullptr;
    }
}

void Timer::start()
{
    if (impl == nullptr) {
        impl = new TimerImpl(sec, cb);
    }
}

Timer::Timer(roswasm::Duration duration, std::function<void(const ros::TimerEvent&)> cb) : sec(duration.toSec()), cb(cb), impl(new TimerImpl(duration.toSec(), cb))
{
}

Timer::Timer() : impl(nullptr)
{

}

Timer::~Timer()
{
    delete impl;
    impl = nullptr;
}

} // namespace roswasm

