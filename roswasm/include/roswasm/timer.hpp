#include <roswasm/timer.h>
#include <emscripten.h>

namespace roswasm {

void Timer::stop()
{
    if (id != 0) {
        emscripten_clear_interval(id);
    }
    id = 0;
}

Timer::Timer(double seconds, std::function<void(const ros::TimerEvent&)> cb) : callback(cb)
{
    id = emscripten_set_interval(&Timer::impl_callback, 1e3*seconds, (void*)(this));
}

Timer::~Timer()
{
    if (id != 0) {
        emscripten_clear_interval(id);
    }
}

} // namespace roswasm

