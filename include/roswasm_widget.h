#ifndef ROSWASM_WIDGET_H
#define ROSWASM_WIDGET_H

#include <roswasm/roswasm.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>

namespace roswasm_webgui {

template <typename MSG>
class TopicBuffer
{
private:
    roswasm::Subscriber* sub;
    MSG msg;
public:
    void callback(const MSG& new_msg)
    {
        msg = new_msg;
    }

    const MSG& get_msg()
    {
        return msg;
    }

    TopicBuffer(roswasm::NodeHandle* nh, const std::string& topic, int throttle_rate=-1)
    {
        sub = nh->subscribe<MSG>(topic, std::bind(&TopicBuffer::callback, this, std::placeholders::_1), throttle_rate);
    }

};

template <typename MSG, typename FB_MSG=MSG>
class TopicWidget {
private:
    std::function<void(FB_MSG&, roswasm::Publisher*)> draw_cb;
    roswasm::Publisher* pub;
    roswasm::Subscriber* sub;
    FB_MSG msg;
public:

    void show_widget()
    {
        draw_cb(msg, pub);
    }

    void callback(const FB_MSG& new_msg)
    {
        msg = new_msg;
    }

    TopicWidget(roswasm::NodeHandle* nh, std::function<void(FB_MSG&, roswasm::Publisher*)> draw_cb, const std::string& topic, const std::string& fb_topic="") : draw_cb(draw_cb)
    {
        pub = nh->advertise<MSG>(topic);
        if (fb_topic.empty()) {
            sub = nh->subscribe<FB_MSG>(topic, std::bind(&TopicWidget::callback, this, std::placeholders::_1));
        }
        else {
            sub = nh->subscribe<FB_MSG>(fb_topic, std::bind(&TopicWidget::callback, this, std::placeholders::_1));
        }
    }
};

template <typename MSG, typename FB_MSG>
class TopicPairWidget {
private:
    std::function<void(FB_MSG&, FB_MSG&, roswasm::Publisher*)> draw_cb;
    roswasm::Publisher* pub;
    roswasm::Subscriber* sub1;
    roswasm::Subscriber* sub2;
    FB_MSG msg1;
    FB_MSG msg2;
public:

    void show_widget()
    {
        draw_cb(msg1, msg2, pub);
    }

    void callback1(const FB_MSG& new_msg)
    {
        msg1 = new_msg;
    }

    void callback2(const FB_MSG& new_msg)
    {
        msg2 = new_msg;
    }

    TopicPairWidget(roswasm::NodeHandle* nh, std::function<void(FB_MSG&, FB_MSG&, roswasm::Publisher*)> draw_cb, const std::string& topic, const std::string& fb_topic1, const std::string& fb_topic2) : draw_cb(draw_cb)
    {
        pub = nh->advertise<MSG>(topic);
        sub1 = nh->subscribe<FB_MSG>(fb_topic1, std::bind(&TopicPairWidget::callback1, this, std::placeholders::_1));
        sub2 = nh->subscribe<FB_MSG>(fb_topic2, std::bind(&TopicPairWidget::callback2, this, std::placeholders::_1));
    }
};

struct DrawFloat32
{
    float minv, maxv;
    DrawFloat32(float minv, float maxv) : minv(minv), maxv(maxv) {}
    void operator()(std_msgs::Float32& msg, roswasm::Publisher* pub);
};

struct DrawFloatPair
{
    double minv1, maxv1, minv2, maxv2;
    std::string name1, name2;
    DrawFloatPair(const std::string name1, double minv1, double maxv1, const std::string& name2, double minv2, double maxv2) : name1(name1), minv1(minv1), maxv1(maxv1), name2(name2), minv2(minv2), maxv2(maxv2) {}
    void operator()(std_msgs::Float64& msg1, std_msgs::Float64& msg2, roswasm::Publisher* pub);
};

void draw_float(std_msgs::Float32& msg, roswasm::Publisher* pub);

void draw_bool(std_msgs::Bool& msg, roswasm::Publisher* pub);

void draw_pose2d(std_msgs::Float64& msg1, std_msgs::Float64& msg2, roswasm::Publisher* pub);

} // namespace roswasm_webgui

#endif // ROSWASM_WIDGET_H
