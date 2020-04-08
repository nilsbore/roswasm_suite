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
#include <rosapi/TopicsForType.h>
#include <rosmon_msgs/State.h>
#include <rosmon_msgs/StartStop.h>
#include <iostream>

namespace roswasm_webgui {

struct LaunchState {

    static const char* status[];

    roswasm::Subscriber* sub;
    roswasm::ServiceClient* client;
    rosmon_msgs::State msg;
    std::string name;
    int selected;
    //size_t nbr_nodes;
    
    int start_stop_launch_node_idx;
    int start_stop_launch_attempts;
    uint8_t start_stop_launch_action;
    int max_launch_attempts;

    void service_callback(const rosmon_msgs::StartStop::Response& res, bool result)
    {
        printf("Got StartStop service response!\n");
    }

    void start_stop_launch_node()
    {
        if (start_stop_launch_node_idx == -1 || start_stop_launch_node_idx >= msg.nodes.size()) {
            return;
        }

        uint8_t desired_state;
        if (start_stop_launch_action == rosmon_msgs::StartStop::Request::START) {
            desired_state = rosmon_msgs::NodeState::RUNNING;
        }
        else if (start_stop_launch_action == rosmon_msgs::StartStop::Request::STOP) {
            desired_state = rosmon_msgs::NodeState::IDLE;
        }

        uint8_t state = msg.nodes[start_stop_launch_node_idx].state;
        if (state == rosmon_msgs::NodeState::CRASHED) {
            state = rosmon_msgs::NodeState::IDLE;
        }
        while (state == desired_state) {
            ++start_stop_launch_node_idx;
            start_stop_launch_attempts = 0;
            if (start_stop_launch_node_idx >= msg.nodes.size()) {
                start_stop_launch_node_idx = -1;
                break;
            }
            state = msg.nodes[start_stop_launch_node_idx].state;
            if (state == rosmon_msgs::NodeState::CRASHED) {
                state = rosmon_msgs::NodeState::IDLE;
            }
        }
        if (start_stop_launch_attempts > max_launch_attempts) {
            start_stop_launch_node_idx = -1;
        }
        if (start_stop_launch_node_idx != -1) {
            start_stop(start_stop_launch_action, msg.nodes[start_stop_launch_node_idx].name, msg.nodes[start_stop_launch_node_idx].ns);
            ++start_stop_launch_attempts;
        }
    }

    void callback(const rosmon_msgs::State& new_msg)
    {
        msg = new_msg;
        start_stop_launch_node();
        printf("Got quite simple msg\n");
        printf("Nodes len: %zu\n", new_msg.nodes.size());
    }

    void start_stop_launch(uint8_t action)
    {
        start_stop_launch_node_idx = 0;
        start_stop_launch_attempts = 0;
        start_stop_launch_action = action;
        start_stop_launch_node();
    }

    void start_stop(uint8_t action, const std::string& name, const std::string& ns)
    {
        rosmon_msgs::StartStop::Request req;
        req.node = name;
        req.action = action;
        req.ns = ns;
        client->call<rosmon_msgs::StartStop>(req);
    }

    float get_progress()
    {
        if (start_stop_launch_node_idx == -1) {
            float sum = 0.;
            for (const rosmon_msgs::NodeState& node : msg.nodes) {
                sum += float(node.state == rosmon_msgs::NodeState::RUNNING);
            }
            return sum / float(msg.nodes.size());
        }
        else {
            return float(start_stop_launch_node_idx)/float(msg.nodes.size());
        }
    }

    LaunchState(roswasm::NodeHandle* node_handle, const std::string& topic, int max_launch_attempts = 10) : selected(-1), start_stop_launch_node_idx(-1), max_launch_attempts(max_launch_attempts)
    {
        sub = node_handle->subscribe<rosmon_msgs::State>(topic, std::bind(&LaunchState::callback, this, std::placeholders::_1));
        //topics_service = nh->serviceClient<rosapi::TopicsForType>("/rosapi/topics_for_type", service_callback);
        size_t ns_pos = topic.find('/', 1);
        name = topic.substr(1, ns_pos-1);
        std::string service_name = std::string("/") + name + "/start_stop";
        printf("Service name: %s\n", service_name.c_str());
        client = node_handle->serviceClient<rosmon_msgs::StartStop>(service_name, std::bind(&LaunchState::service_callback, this, std::placeholders::_1, std::placeholders::_2));
        //sub = node_handle->subscribe<rosmon_msgs::State>(topic, simple_callback);
    }

    LaunchState() : sub(nullptr) {}

};

const char* LaunchState::status[] = { "IDLE", "RUNNING", "CRASHED", "WAITING" };

// ros stuff
roswasm::ServiceClient* topics_service;
std::unordered_map<std::string, LaunchState*> launch_states;
roswasm::NodeHandle* nh;
roswasm::Timer* timer;

void service_callback(const rosapi::TopicsForType::Response& res, bool result)
{
    for (int i = 0; i < res.topics.size(); ++i) {
        if (launch_states.count(res.topics[i]) == 0) {
            launch_states[res.topics[i]] = new LaunchState(nh, res.topics[i]);
        }
    }
}

void timer_callback(const ros::TimerEvent& event)
{
    rosapi::TopicsForType::Request req;
    req.type = "rosmon_msgs/State";
    topics_service->call<rosapi::TopicsForType>(req);
}

void init_monlaunch(roswasm::NodeHandle* n)
{
    nh = n;
    topics_service = nh->serviceClient<rosapi::TopicsForType>("/rosapi/topics_for_type", service_callback);
    rosapi::TopicsForType::Request req;
    req.type = "rosmon_msgs/State";
    topics_service->call<rosapi::TopicsForType>(req);
    timer = nh->createTimer(5., timer_callback);
}

void show_monlaunch_window(bool& show_another_window)
{
      ImGui::SetNextWindowSize(ImVec2(550,680), ImGuiCond_FirstUseEver);
      ImGui::Begin("Mon launch instances", &show_another_window);
      for (std::pair<const std::string, LaunchState*>& state : launch_states) {
          if (state.second->msg.nodes.empty()) {
             continue;
          } 
          if (ImGui::CollapsingHeader(state.second->name.c_str(), ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
              ImGui::PushID(state.second->name.c_str());
              bool nodes_expanded = ImGui::TreeNodeEx("Nodes", ImGuiTreeNodeFlags_AllowItemOverlap | ImGuiTreeNodeFlags_FramePadding);
              ImGui::SameLine(); //(0, 0);
              if (ImGui::Button("Start"))
                  state.second->start_stop_launch(rosmon_msgs::StartStop::Request::START);
              ImGui::SameLine();
              if (ImGui::Button("Stop"))
                  state.second->start_stop_launch(rosmon_msgs::StartStop::Request::STOP);
              ImGui::SameLine();
              char buf[32];
              float progress = state.second->get_progress();
              sprintf(buf, "%d/%d", int(progress*state.second->msg.nodes.size()), int(state.second->msg.nodes.size()));
              ImGui::ProgressBar(progress, ImVec2(0.f,0.f), buf);
              if (nodes_expanded) {
                    ImGui::Columns(5, "mycolumns"); // 4-ways, with border
                    ImGui::Separator();
                    ImGui::Text("Name"); ImGui::NextColumn();
                    ImGui::Text("State"); ImGui::NextColumn();
                    ImGui::Text("CPU"); ImGui::NextColumn();
                    ImGui::Text("Memory"); ImGui::NextColumn();
                    ImGui::Text("Restarts"); ImGui::NextColumn();
                    ImGui::Separator();
                    for (int i = 0; i < state.second->msg.nodes.size(); i++)
                    {
                        const std::string& name = state.second->msg.nodes[i].name;
                        const std::string& ns = state.second->msg.nodes[i].ns;
                        if (ImGui::Selectable(name.c_str(), state.second->selected == i, ImGuiSelectableFlags_SpanAllColumns))
                            state.second->selected = i;
                        if (ImGui::BeginPopupContextItem(name.c_str()))
                        {
                            if (ImGui::Selectable("START")) state.second->start_stop(rosmon_msgs::StartStop::Request::START, name, ns);
                            if (ImGui::Selectable("STOP")) state.second->start_stop(rosmon_msgs::StartStop::Request::STOP, name, ns);
                            if (ImGui::Selectable("RESTART")) state.second->start_stop(rosmon_msgs::StartStop::Request::RESTART, name, ns);
                            ImGui::EndPopup();
                        }
                        bool hovered = ImGui::IsItemHovered();
                        ImGui::NextColumn();
                        ImGui::Text("%s", LaunchState::status[state.second->msg.nodes[i].state]); ImGui::NextColumn();
                        ImGui::Text("%.2f %%", 100.*state.second->msg.nodes[i].user_load); ImGui::NextColumn();
                        ImGui::Text("%.2f MB", 1e-6*state.second->msg.nodes[i].memory); ImGui::NextColumn();
                        ImGui::Text("%u", state.second->msg.nodes[i].restart_count); ImGui::NextColumn();
                    }
                    ImGui::Columns(1);
                    ImGui::Separator();
                    ImGui::TreePop();
              }
              ImGui::PopID();
          }
      }
      ImGui::End();
}

} // namespace roswasm_webgui

roswasm::NodeHandle* nh; 

GLFWwindow* g_window;
//ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
ImVec4 clear_color = ImVec4(0.25f, 0.45f, 0.55f, 1.00f);
ImGuiContext* imgui = 0;
bool show_demo_window = false;
bool show_another_window = true;

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
          status_text = "Connected";
          status_color = ImColor(0, 255, 0);
      }
      else {
          status_text = "Disconnected";
          status_color = ImColor(255, 0, 0);
      }
      ImVec2 p = ImGui::GetCursorScreenPos();
      ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
      ImGui::Dummy(ImVec2(sz, sz));
      ImGui::SameLine();
      ImGui::Text("%s", status_text.c_str());


      ImGui::Checkbox("Mon launch instances", &show_another_window);
      ImGui::Checkbox("Demo widget window", &show_demo_window);      // Edit bools storing our windows open/close state

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::ColorEdit3("Background", (float*)&clear_color); // Edit 3 floats representing a color
  }

  //std::cout << "2nd window" << std::endl;

  // 2. Show another simple window. In most cases you will use an explicit Begin/End pair to name your windows.
  if (show_another_window)
  {
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
      roswasm_webgui::show_monlaunch_window(show_another_window);
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

  nh = new roswasm::NodeHandle();
  roswasm_webgui::init_monlaunch(nh);

  #ifdef __EMSCRIPTEN__
  emscripten_set_main_loop(loop, 20, 1);
  #endif

  quit();

  return 0;
}
