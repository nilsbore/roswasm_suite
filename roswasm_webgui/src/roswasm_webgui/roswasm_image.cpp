#include <roswasm_webgui/roswasm_image.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include "roswasm_webgui/imgui/imgui.h"

namespace roswasm_webgui {

void ImageWidget::load_texture(std::vector<uint8_t>& buffer, const std::string& format)
{
    //GLuint textureID;
    //glGenTextures(1, &textureID);

    //SDL_Surface* surface = IMG_Load("resources/test.jpg");
    std::cout << "Buffer size: " << buffer.size() << std::endl;
    SDL_RWops* src = SDL_RWFromMem(&buffer[0], buffer.size());
    //SDL_Surface* surface = IMG_LoadTyped_RW(src, 1, "png");
    SDL_Surface* surface;
    if (format.find("jpeg") != std::string::npos) {
        surface = IMG_LoadTyped_RW(src, 1, "jpg");
    }
    else if (format.find("png") != std::string::npos) {
        surface = IMG_LoadTyped_RW(src, 1, "png");
    }
    else {
        printf("Invalid compression format!\n");
        return;
    }
    //surface = IMG_LoadTyped_RW(src, 1, "png");
    std::cout << "Image width: " << surface->w << std::endl;
    std::cout << "Image height: " << surface->h << std::endl;

    if (image_width != surface->w || image_height != surface->h) {
        image_width = surface->w;
        image_height = surface->h;

        glGenTextures(1, &image_texture);
        glBindTexture(GL_TEXTURE_2D, image_texture);

        // Setup filtering parameters for display
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Upload pixels into texture
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, surface->pixels);
        //stbi_image_free(image_data);
    }
    else {
        glBindTexture(GL_TEXTURE_2D, image_texture);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width, image_height, GL_RGB, GL_UNSIGNED_BYTE, surface->pixels);
    };
    SDL_FreeSurface(surface);

    //glGenTextures(1, &textureID);
    //glBindTexture(GL_TEXTURE_2D, textureID);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, surface->w, surface->h, 0, GL_RGB, GL_UNSIGNED_BYTE, surface->pixels);
}

void ImageWidget::service_callback(const rosapi::TopicsForType::Response& res, bool result)
{
    //topics = res.topics;
    topics.clear();
    std::copy_if(res.topics.begin(), res.topics.end(), std::back_inserter(topics), [](const std::string& s){return s.find("compressedDepth") == std::string::npos;});
}

void ImageWidget::timer_callback(const ros::TimerEvent& event)
{
    rosapi::TopicsForType::Request req;
    req.type = "sensor_msgs/CompressedImage";
    topics_service.call<rosapi::TopicsForType>(req, std::bind(&ImageWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
}

ImageWidget::ImageWidget(roswasm::NodeHandle& n) : nh(n)
{
    image_width = 0;
    image_height = 0;
    image_texture = 0;
    //nh = n;
    //sub = nullptr;
    //topics_service = nh->serviceClient<rosapi::TopicsForType>("/rosapi/topics_for_type", std::bind(&ImageWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    topics_service = roswasm::createServiceCallbackClient<rosapi::TopicsForType>(nh, "/rosapi/topics_for_type"); // std::bind(&ImageWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2)
    rosapi::TopicsForType::Request req;
    req.type = "sensor_msgs/CompressedImage";
    topics_service.call<rosapi::TopicsForType>(req, std::bind(&ImageWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    timer = nh.createTimer(roswasm::Duration(5.), std::bind(&ImageWidget::timer_callback, this, std::placeholders::_1));
}

void ImageWidget::callback(const sensor_msgs::CompressedImage& msg)
{
    printf("Got callback with compresed image!\n");
    std::vector<uint8_t> bytes = msg.data;
    load_texture(bytes, msg.format);
}

void ImageWidget::show_window(bool& show_another_window)
{
    ImGui::Begin("Image topic", &show_another_window);
    std::string choices;
    if (topics.empty()) {
        choices = " \0";
    }
    else {
        for (const std::string& topic : topics) choices += topic + '\0';
    }
    static int style_idx = -1;
    if (ImGui::Combo("Published topics", &style_idx, choices.c_str()))
    {
        if (style_idx != -1 && !topics.empty()) {

            // TODO: this needs to get fixed
            if (sub.getTopic() != topics[style_idx]) {
                sub.shutdown();
                sub = nh.subscribe(topics[style_idx], 1000, &ImageWidget::callback, this);
            }
            //sub = nh.subscribe(topics[style_idx], 1000, std::bind(&ImageWidget::callback, this, std::placeholders::_1), 1000);
        }
    }
    if (image_width != 0 && image_height != 0) {
        //ImGui::Image((void*)(intptr_t)image_texture, ImVec2(image_width, image_height));
        ImVec2 size = ImGui::GetWindowSize();
        ImGui::Image((void*)(intptr_t)image_texture, ImVec2(size.x, int(float(size.x)/float(image_width)*image_height)));
    }
    ImGui::End();
}


} // namespace roswasm_webgui
