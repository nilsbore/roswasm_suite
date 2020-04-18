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
//#include <iostream>

#include <unordered_set>
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

void draw_float(std_msgs::Float32& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("Float");
    ImGui::SliderFloat("Float slider", &msg.data, 0.0f, 100.0f);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    ImGui::SameLine();
    ImGui::InputFloat("Float value", &msg.data);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    ImGui::PopID();
}

void draw_bool(std_msgs::Bool& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("Bool");
    bool value = msg.data;
    ImGui::Checkbox("Controller active", &value);
    msg.data = value;
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    ImGui::PopID();
}

void draw_pose2d(std_msgs::Float64& msg1, std_msgs::Float64& msg2, roswasm::Publisher* pub)
{
    geometry_msgs::Pose2D msg;
    msg.x = msg1.data;
    msg.y = msg2.data;

    double min_value = 0.0;
    double max_value = 100.0;

    ImGui::PushID("Pose2D");

    ImGui::SliderScalar("First cmd slider", ImGuiDataType_Double, &msg1.data, &min_value, &max_value, "%.10f grams");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.x = msg1.data;
        pub->publish(msg);
    }
    ImGui::SameLine();
    ImGui::InputScalar("First cmd input",  ImGuiDataType_Double, &msg1.data);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.x = msg1.data;
        pub->publish(msg);
    }

    ImGui::SliderScalar("Second cmd slider", ImGuiDataType_Double, &msg2.data, &min_value, &max_value, "%.10f grams");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.y = msg2.data;
        pub->publish(msg);
    }
    ImGui::SameLine();
    ImGui::InputScalar("Second cmd input",  ImGuiDataType_Double, &msg2.data);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.y = msg2.data;
        pub->publish(msg);
    }

    ImGui::PopID();
}

struct DrawFloat32
{
    float minv, maxv;
    DrawFloat32(float minv, float maxv) : minv(minv), maxv(maxv) {}
    void operator()(std_msgs::Float32& msg, roswasm::Publisher* pub)
    {
        ImGui::PushID("Float");
        ImGui::SliderFloat("Float slider", &msg.data, minv, maxv);
        if (ImGui::IsItemDeactivatedAfterChange()) {
            pub->publish(msg);
        }
        ImGui::SameLine();
        ImGui::InputFloat("Float value", &msg.data);
        if (ImGui::IsItemDeactivatedAfterChange()) {
            pub->publish(msg);
        }
        ImGui::PopID();
    }
};

struct DrawFloatPair
{
    double minv1, maxv1, minv2, maxv2;
    DrawFloatPair(double minv1, double maxv1, double minv2, double maxv2) : minv1(minv1), maxv1(maxv1), minv2(minv2), maxv2(maxv2) {}
    void operator()(std_msgs::Float64& msg1, std_msgs::Float64& msg2, roswasm::Publisher* pub)
    {
        geometry_msgs::Pose2D msg;
        msg.x = msg1.data;
        msg.y = msg2.data;

        ImGui::PushID("Pose2D");

        ImGui::SliderScalar("First cmd slider", ImGuiDataType_Double, &msg1.data, &minv1, &maxv1, "%.10f grams");
        if (ImGui::IsItemDeactivatedAfterChange()) {
            msg.x = msg1.data;
            pub->publish(msg);
        }
        ImGui::SameLine();
        ImGui::InputScalar("First cmd input",  ImGuiDataType_Double, &msg1.data);
        if (ImGui::IsItemDeactivatedAfterChange()) {
            msg.x = msg1.data;
            pub->publish(msg);
        }

        ImGui::SliderScalar("Second cmd slider", ImGuiDataType_Double, &msg2.data, &minv1, &maxv1, "%.10f grams");
        if (ImGui::IsItemDeactivatedAfterChange()) {
            msg.y = msg2.data;
            pub->publish(msg);
        }
        ImGui::SameLine();
        ImGui::InputScalar("Second cmd input",  ImGuiDataType_Double, &msg2.data);
        if (ImGui::IsItemDeactivatedAfterChange()) {
            msg.y = msg2.data;
            pub->publish(msg);
        }

        ImGui::PopID();
    }
};

class ExampleActuatorWidget {
private:
    TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>* thruster_angles;
    TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>* thruster_rpms;
    TopicWidget<std_msgs::Float32>* lcg_actuator;
    TopicWidget<std_msgs::Bool>* lcg_control_enable;
    TopicWidget<std_msgs::Float32>* lcg_control_setpoint;
    TopicWidget<std_msgs::Float32>* vbs_actuator;
    TopicWidget<std_msgs::Bool>* vbs_control_enable;
    TopicWidget<std_msgs::Float32>* vbs_control_setpoint;
    TopicWidget<std_msgs::Float32>* tcg_actuator;
    TopicWidget<std_msgs::Bool>* tcg_control_enable;
    TopicWidget<std_msgs::Float32>* tcg_control_setpoint;

public:
    void show_window(bool& show_actuator_window)
    {
        ImGui::SetNextWindowSize(ImVec2(550,680), ImGuiCond_FirstUseEver);
        ImGui::Begin("Actuator controls", &show_actuator_window);

        if (ImGui::CollapsingHeader("Thruster Angles", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            thruster_angles->show_widget();
        }
        if (ImGui::CollapsingHeader("Thruster RPMs", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            thruster_rpms->show_widget();
        }

        if (ImGui::CollapsingHeader("LCG", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            lcg_actuator->show_widget();
            lcg_control_enable->show_widget();
            lcg_control_setpoint->show_widget();
        }
        if (ImGui::CollapsingHeader("VBS", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            vbs_actuator->show_widget();
            vbs_control_enable->show_widget();
            vbs_control_setpoint->show_widget();
        }
        if (ImGui::CollapsingHeader("TCG", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            tcg_actuator->show_widget();
            tcg_control_enable->show_widget();
            tcg_control_setpoint->show_widget();
        }

        ImGui::End();
    }

    ExampleActuatorWidget(roswasm::NodeHandle* nh)
    {
        thruster_angles = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair(-0.1, 0.18, -0.1, 0.15), "/sam/core/thrust_vector_cmd", "/sam/core/thrust_fb1", "/sam/core/thrust_fb2");
//                                                      [{"name": "Hori.", "member": "thruster_horizontal_radians", "min": -0.1, "max": 0.18},
//                                                       {"name": "Vert.", "member": "thruster_vertical_radians", "min": -0.1, "max": 0.15}])
        thruster_rpms = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair(-1000., 1000., -1000., 1000.), "/sam/core/rpm_cmd", "/sam/core/rpm_fb1", "/sam/core/rpm_fb2");
//                                                    [{"name": "Front", "member": "thruster_1_rpm", "min": -1000, "max": 1000, "type": "int"},
//                                                     {"name": "Back", "member": "thruster_2_rpm", "min": -1000, "max": 1000, "type": "int"}])

//            self.leak_button = flx.Button(text="No leaks...", style="background: #008000;", disabled=True)

        lcg_actuator = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 100.), "/sam/core/lcg_cmd", "/sam/core/lcg_fb");
        lcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "/sam/ctrl/lcg/pid_enable");
        lcg_control_setpoint = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-1.6, 1.6), "/sam/ctrl/lcg/setpoint"); //, -1.6, 1.6)

        vbs_actuator = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 100.), "/sam/core/vbs_cmd", "/sam/core/vbs_fb");
        vbs_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "/sam/ctrl/vbs/pid_enable");
        vbs_control_setpoint = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 5.), "/sam/ctrl/vbs/setpoint"); //, 0 5)
            
        tcg_actuator = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-1.6, 1.6), "/sam/core/tcg_cmd");
        tcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "/sam/ctrl/tcg/pid_enable");
        tcg_control_setpoint = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-1.6, 1.6), "/sam/ctrl/tcg/setpoint"); //, -1.6, 1.6)

    }
};

} // namespace roswasm_webgui

roswasm::NodeHandle* nh; 
roswasm_webgui::MonlaunchWidget* monlaunch_widget;
roswasm_webgui::ImageWidget* image_widget;
roswasm_webgui::TopicWidget<std_msgs::Float32>* float_widget;
roswasm_webgui::TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>* pose2d_widget;
roswasm_webgui::ExampleActuatorWidget* actuator_widget;

GLFWwindow* g_window;
//ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
ImVec4 clear_color = ImVec4(0.25f, 0.45f, 0.55f, 1.00f);
ImGuiContext* imgui = 0;
bool show_demo_window = false;
bool show_monlaunch_window = true;
bool show_image_window = true;
bool show_actuator_window = true;

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
      ImGui::Checkbox("Actuators", &show_actuator_window);
      ImGui::Checkbox("Demo widget window", &show_demo_window);      // Edit bools storing our windows open/close state

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::ColorEdit3("Background", (float*)&clear_color); // Edit 3 floats representing a color
  }

  //std::cout << "2nd window" << std::endl;

  // 2. Show another simple window. In most cases you will use an explicit Begin/End pair to name your windows.
  if (show_monlaunch_window)
  {
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      monlaunch_widget->show_window(show_monlaunch_window);
  }

  if (show_image_window)
  {
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
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
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      /*
        ImGui::SetNextWindowSize(ImVec2(550,680), ImGuiCond_FirstUseEver);
        ImGui::Begin("Topic setpoints", &show_topic_window);
        float_widget->show_widget();
        pose2d_widget->show_widget();
        ImGui::End();
        */
      actuator_widget->show_window(show_actuator_window);

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

  #ifdef __EMSCRIPTEN__
  emscripten_set_main_loop(loop, 20, 1);
  #endif

  quit();

  return 0;
}
