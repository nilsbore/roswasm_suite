#ifndef ROSWASM_TIMER_H
#define ROSWASM_TIMER_H

#include <ros/ros.h>

namespace roswasm {

struct Timer {

    long id;
    std::function<void(const ros::TimerEvent&)> callback;

    static void impl_callback(void* user_data)
    {
        Timer* timer = (Timer*)(user_data);
        ros::TimerEvent ev;
        timer->callback(ev);
    }

    void stop();

    Timer(double seconds, std::function<void(const ros::TimerEvent&)> cb);

    ~Timer();
};

} // namespace roswasm

#endif // ROSWASM_TIMER_H
