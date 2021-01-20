#ifndef ROSWASM_IMAGE_H
#define ROSWASM_IMAGE_H

//#include <GLES3/gl3.h>
#ifdef ROSWASM_NATIVE
#include <GL/glew.h> 
#else
#include <GLES3/gl3.h>
#endif
#include <roswasm/roswasm.h>
#include <rosapi/TopicsForType.h>
#include <sensor_msgs/CompressedImage.h>

namespace roswasm_webgui {

class ImageWidget {
private:
    roswasm::ServiceCallbackClient topics_service;
    roswasm::NodeHandle& nh;
    std::vector<std::string> topics;
    roswasm::Subscriber sub;
    roswasm::Timer timer;

    int image_width;
    int image_height;
    GLuint image_texture;
public:
    void load_texture(std::vector<uint8_t>& buffer, const std::string& format);
    void service_callback(const rosapi::TopicsForType::Response& res, bool result);
    void timer_callback(const ros::TimerEvent& event);
    ImageWidget(roswasm::NodeHandle& n);
    void callback(const sensor_msgs::CompressedImage& msg);
    void show_window(bool& show_another_window);
};

} // namsespace roswasm_webgui

#endif // ROSWASM_IMAGE_H
