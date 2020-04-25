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
#include <roswasm_image.h>
#include <roswasm_examples.h>
//#include <iostream>

#include <unordered_set>

roswasm::NodeHandle* nh; 
roswasm_webgui::MonlaunchWidget* monlaunch_widget;
roswasm_webgui::ImageWidget* image_widget;
roswasm_webgui::ExampleActuatorWidget* actuator_widget;
roswasm_webgui::ExampleDashboardWidget* dashboard_widget;
roswasm_webgui::ExampleTeleopWidget* teleop_widget;

GLFWwindow* g_window;
//ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
ImVec4 clear_color = ImVec4(0.25f, 0.45f, 0.55f, 1.00f);
ImGuiContext* imgui = 0;
bool show_demo_window = false;
bool show_monlaunch_window = true;
bool show_image_window = true;
bool show_actuator_window = true;
bool show_dashboard_window = true;
bool show_teleop_window = true;

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
      ImGui::SetNextWindowPos(ImVec2(30, 30), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(482, 210), ImGuiCond_FirstUseEver);
      ImGui::Begin("Roswasm webgui"); //, &show_another_window);
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


      ImGui::Checkbox("Launch control", &show_monlaunch_window);
      ImGui::Checkbox("Image topic", &show_image_window);
      ImGui::Checkbox("Actuator controls", &show_actuator_window);
      ImGui::Checkbox("Status dashboard", &show_dashboard_window);
      ImGui::Checkbox("Keyboard teleop", &show_teleop_window);
      ImGui::Checkbox("Demo widgets", &show_demo_window);      // Edit bools storing our windows open/close state

      //ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::ColorEdit3("Background", (float*)&clear_color); // Edit 3 floats representing a color
      ImGui::End();
  }

  //std::cout << "2nd window" << std::endl;

  // 2. Show another simple window. In most cases you will use an explicit Begin/End pair to name your windows.
  if (show_monlaunch_window)
  {
      ImGui::SetNextWindowPos(ImVec2(30, 270), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      monlaunch_widget->show_window(show_monlaunch_window);
  }

  if (show_image_window)
  {
      ImGui::SetNextWindowPos(ImVec2(1072, 30), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      image_widget->show_window(show_image_window);
  }

  // 3. Show the ImGui demo window. Most of the sample code is in ImGui::ShowDemoWindow(). Read its code to learn more about Dear ImGui!
  if (show_demo_window)
  {
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      ImGui::ShowDemoWindow(&show_demo_window);
  }

  if (show_actuator_window)
  {
      ImGui::SetNextWindowPos(ImVec2(542, 303), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      /*
        ImGui::SetNextWindowSize(ImVec2(550,680), ImGuiCond_FirstUseEver);
        ImGui::Begin("Topic setpoints", &show_topic_window);
        float_widget->show_widget();
        pose2d_widget->show_widget();
        ImGui::End();
        */
      actuator_widget->show_window(show_actuator_window);

  }

  if (show_dashboard_window)
  {
      ImGui::SetNextWindowPos(ImVec2(542, 30), ImGuiCond_FirstUseEver);
      dashboard_widget->show_window(show_dashboard_window);

  }

  if (show_teleop_window)
  {
      ImGui::SetNextWindowPos(ImVec2(1072, 487), ImGuiCond_FirstUseEver);
      teleop_widget->show_window(show_teleop_window);

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
  monlaunch_widget = new roswasm_webgui::MonlaunchWidget(nh);
  image_widget = new roswasm_webgui::ImageWidget(nh);
  //float_widget = new roswasm_webgui::TopicWidget<std_msgs::Float32>(nh, &roswasm_webgui::draw_float, "/test_float");
  //pose2d_widget = new roswasm_webgui::TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, &roswasm_webgui::draw_pose2d, "/pose2d", "/pose2d_fb1", "/pose2d_fb2");
  actuator_widget = new roswasm_webgui::ExampleActuatorWidget(nh);
  dashboard_widget = new roswasm_webgui::ExampleDashboardWidget(nh);
  teleop_widget = new roswasm_webgui::ExampleTeleopWidget(nh);

  #ifdef __EMSCRIPTEN__
  emscripten_set_main_loop(loop, 20, 1);
  #endif

  quit();

  return 0;
}
