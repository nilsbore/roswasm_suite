#ifndef ROSWASM_TIME_H
#define ROSWASM_TIME_H

#include <chrono>
#include <ros/time.h>
#include <ros/duration.h>
#include <ros/impl/time.h>
#include <ros/impl/duration.h>

namespace roswasm {

class WallDuration : public ros::DurationBase<WallDuration>
{
public:
    WallDuration() : DurationBase<WallDuration>() {}
    WallDuration(int32_t _sec, int32_t _nsec) : DurationBase<WallDuration>(_sec, _nsec) {}
    explicit WallDuration(double t) { fromSec(t); }
};

class WallTime : public ros::TimeBase<WallTime, WallDuration>
{
public:
    WallTime(): ros::TimeBase<WallTime, WallDuration>() {}
    WallTime(uint32_t _sec, uint32_t _nsec) : ros::TimeBase<WallTime, WallDuration>(_sec, _nsec) {}
    explicit WallTime(double t) { fromSec(t); }
    static WallTime now()
    {
        using namespace std::chrono;

        auto now = system_clock::now();
        auto duration = now.time_since_epoch();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);

        WallTime t;
        t.fromNSec(nanoseconds.count());
        return t;
    }
};

using Time = WallTime;
using Duration = WallDuration; 

} // namespace roswasm

#endif // ROSWASM_TIME_H
