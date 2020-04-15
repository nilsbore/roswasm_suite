#include <stdio.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#define GLFW_INCLUDE_ES3
#include <GLES3/gl3.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <roswasm/roswasm.h>
#include <roswasm_monlaunch.h>
//#include <iostream>

#include <unordered_set>
#include <sensor_msgs/CompressedImage.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

roswasm::NodeHandle* nh; 

namespace roswasm_image {

roswasm::ServiceClient* topics_service;
roswasm::NodeHandle* nh;
std::vector<std::string> topics;
roswasm::Subscriber* sub;
roswasm::Timer* timer;
SDL_Surface* surface;

int image_width;
int image_height;
GLuint image_texture;

void load_texture(std::vector<uint8_t>& buffer, const std::string& format)
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

void service_callback(const rosapi::TopicsForType::Response& res, bool result)
{
    topics = res.topics;
}

void timer_callback(const ros::TimerEvent& event)
{
    rosapi::TopicsForType::Request req;
    req.type = "sensor_msgs/CompressedImage";
    topics_service->call<rosapi::TopicsForType>(req);
}

void init_image(roswasm::NodeHandle* n)
{
    image_width = 0;
    image_height = 0;
    image_texture = 0;
    nh = n;
    topics_service = nh->serviceClient<rosapi::TopicsForType>("/rosapi/topics_for_type", service_callback);
    rosapi::TopicsForType::Request req;
    req.type = "sensor_msgs/CompressedImage";
    topics_service->call<rosapi::TopicsForType>(req);
    timer = nh->createTimer(5., timer_callback);
}

void callback(const sensor_msgs::CompressedImage& msg)
{
    printf("Got callback with compresed image!\n");
    std::vector<uint8_t> bytes = msg.data;
    load_texture(bytes, msg.format);
}

void show_image_window(bool& show_another_window)
{
    ImGui::SetNextWindowSize(ImVec2(550,680), ImGuiCond_FirstUseEver);
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
            sub = nh->subscribe<sensor_msgs::CompressedImage>(topics[style_idx], callback, 1000);
        }
    }
    if (image_width != 0 && image_height != 0) {
        ImGui::Image((void*)(intptr_t)image_texture, ImVec2(image_width, image_height));
    }
    ImGui::End();
}

} // namespace roswasm_image

GLFWwindow* g_window;
//ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
ImVec4 clear_color = ImVec4(0.25f, 0.45f, 0.55f, 1.00f);
ImGuiContext* imgui = 0;
bool show_demo_window = false;
bool show_monlaunch_window = true;
bool show_image_window = true;

EM_JS(int, canvas_get_width, (), {
  return Module.canvas.width;
});

EM_JS(int, canvas_get_height, (), {
  return Module.canvas.height;
});

EM_JS(void, resizeCanvas, (), {
  js_resizeCanvas();
});

void loop()
{
  int width = canvas_get_width();
  int height = canvas_get_height();

  glfwSetWindowSize(g_window, width, height);

  ImGui::SetCurrentContext(imgui);

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // 1. Show a simple window.
  // Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets automatically appears in a window called "Debug".
  {
      float sz = ImGui::GetTextLineHeight();
      std::string status_text;
      ImColor status_color;
      if (nh->ok()) {
          status_text = "Connected to " + nh->get_websocket_url();
          status_color = ImColor(0, 255, 0);
      }
      else {
          status_text = "Disconnected from " + nh->get_websocket_url();
          status_color = ImColor(255, 0, 0);
      }
      ImVec2 p = ImGui::GetCursorScreenPos();
      ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
      ImGui::Dummy(ImVec2(sz, sz));
      ImGui::SameLine();
      ImGui::Text("%s", status_text.c_str());


      ImGui::Checkbox("Mon launch instances", &show_monlaunch_window);
      ImGui::Checkbox("Image topic", &show_image_window);
      ImGui::Checkbox("Demo widget window", &show_demo_window);      // Edit bools storing our windows open/close state

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::ColorEdit3("Background", (float*)&clear_color); // Edit 3 floats representing a color
  }

  //std::cout << "2nd window" << std::endl;

  // 2. Show another simple window. In most cases you will use an explicit Begin/End pair to name your windows.
  if (show_monlaunch_window)
  {
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      roswasm_monlaunch::show_monlaunch_window(show_monlaunch_window);
  }

  if (show_image_window)
  {
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      roswasm_image::show_image_window(show_image_window);
  }

  // 3. Show the ImGui demo window. Most of the sample code is in ImGui::ShowDemoWindow(). Read its code to learn more about Dear ImGui!
  if (show_demo_window)
  {
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      ImGui::ShowDemoWindow(&show_demo_window);
  }

  ImGui::Render();

  int display_w, display_h;
  glfwMakeContextCurrent(g_window);
  glfwGetFramebufferSize(g_window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwMakeContextCurrent(g_window);
}


int init()
{
  if( !glfwInit() )
  {
      fprintf( stderr, "Failed to initialize GLFW\n" );
      return 1;
  }

  //glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

  // Open a window and create its OpenGL context
  int canvasWidth = 800;
  int canvasHeight = 600;
  g_window = glfwCreateWindow( canvasWidth, canvasHeight, "WebGui Demo", NULL, NULL);
  if( g_window == NULL )
  {
      fprintf( stderr, "Failed to open GLFW window.\n" );
      glfwTerminate();
      return -1;
  }
  glfwMakeContextCurrent(g_window); // Initialize GLEW

  // Create game objects
  // Setup Dear ImGui binding
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();

  ImGui_ImplGlfw_InitForOpenGL(g_window, false);
  ImGui_ImplOpenGL3_Init();

  // Setup style
  ImGui::StyleColorsLight();
  //ImGui::StyleColorsDark();
  //ImGui::StyleColorsClassic();

  // Load Fonts
  io.Fonts->AddFontDefault();
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 23.0f);
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 18.0f);
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 26.0f);
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 32.0f);

  imgui =  ImGui::GetCurrentContext();

  // Cursor callbacks
  glfwSetMouseButtonCallback(g_window, ImGui_ImplGlfw_MouseButtonCallback);
  glfwSetScrollCallback(g_window, ImGui_ImplGlfw_ScrollCallback);
  glfwSetKeyCallback(g_window, ImGui_ImplGlfw_KeyCallback);
  glfwSetCharCallback(g_window, ImGui_ImplGlfw_CharCallback);

  resizeCanvas();

  return 0;
}


void quit()
{
  glfwTerminate();
}


extern "C" int main(int argc, char** argv)
{
  if (init() != 0) return 1;

  if (argc < 3) {
      printf("Rosbridge server ip and port not provided!");
      return 1;
  }

  std::string rosbridge_ip(argv[1]);
  std::string rosbridge_port(argv[2]);

  nh = new roswasm::NodeHandle(rosbridge_ip, rosbridge_port);
  roswasm_monlaunch::init_monlaunch(nh);
  roswasm_image::init_image(nh);

  #ifdef __EMSCRIPTEN__
  emscripten_set_main_loop(loop, 20, 1);
  #endif

  quit();

  return 0;
}
