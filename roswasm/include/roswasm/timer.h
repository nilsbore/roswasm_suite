#ifndef ROSWASM_TIMER_H
#define ROSWASM_TIMER_H

#include <ros/ros.h>
#include <roswasm/time.h>

namespace roswasm {

struct TimerImpl {

    long id;
    std::function<void(const ros::TimerEvent&)> callback;

    static void impl_callback(void* user_data)
    {
        TimerImpl* timer = (TimerImpl*)(user_data);
        ros::TimerEvent ev;
        timer->callback(ev);
    }

    TimerImpl(double seconds, std::function<void(const ros::TimerEvent&)> cb);

    ~TimerImpl();

};

struct Timer {

    TimerImpl* impl;
    double sec;
    std::function<void(const ros::TimerEvent&)> cb;

    void stop();
    void start();

    Timer();
    Timer(roswasm::Duration duration, std::function<void(const ros::TimerEvent&)> cb);

    ~Timer();
};

} // namespace roswasm

#endif // ROSWASM_TIMER_H
