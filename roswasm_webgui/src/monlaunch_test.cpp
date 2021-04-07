#include <stdio.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <roswasm_webgui/imgui/imgui.h>
#include <roswasm_webgui/imgui/imgui_impl_glfw.h>
#include <roswasm_webgui/imgui/imgui_impl_opengl3.h>

#ifdef ROSWASM_NATIVE
#include <GL/glew.h> 
#else
#define GLFW_INCLUDE_ES3
#include <GLES3/gl3.h>
#endif
#include <GLFW/glfw3.h>

#include <roswasm/roswasm.h>
#include <roswasm_webgui/roswasm_monlaunch.h>
#include <roswasm_webgui/roswasm_image.h>
#include <roswasm_webgui/roswasm_examples.h>

roswasm::NodeHandle* nh; 
roswasm_webgui::MonlaunchWidget* monlaunch_widget;

GLFWwindow* g_window;
ImVec4 clear_color = ImVec4(0.25f, 0.45f, 0.55f, 1.00f);
ImGuiContext* imgui = 0;
bool show_demo_window = false;
bool show_monlaunch_window = true;

#ifndef ROSWASM_NATIVE
EM_JS(int, canvas_get_width, (), {
    return Module.canvas.width;
});

EM_JS(int, canvas_get_height, (), {
    return Module.canvas.height;
});

EM_JS(void, resizeCanvas, (), {
    js_resizeCanvas();
});
#endif

void loop()
{
    if (glfwWindowShouldClose(g_window)) {
        roswasm::shutdown();
        return;
    }

#ifndef ROSWASM_NATIVE
    glfwSetWindowSize(g_window, canvas_get_width(), canvas_get_height());
#endif
    glfwPollEvents();

    //ImGui::SetCurrentContext(imgui);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    {
        ImGui::SetNextWindowPos(ImVec2(30, 30), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(482, 100), ImGuiCond_FirstUseEver);
        ImGui::Begin("Roswasm webgui"); //, &show_another_window);
        float sz = ImGui::GetTextLineHeight();
        std::string status_text;
        ImColor status_color;
        if (nh->ok()) {
#ifdef ROSWASM_NATIVE
            status_text = "ROS OK";
#else
            status_text = "Connected to " + nh->get_websocket_url();
#endif
            status_color = ImColor(0, 255, 0);
        }
        else {
#ifdef ROSWASM_NATIVE
            status_text = "ROS NOT OK!";
#else
            status_text = "Disconnected from " + nh->get_websocket_url();
#endif
            status_color = ImColor(255, 0, 0);
        }
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
        ImGui::Dummy(ImVec2(sz, sz));
        ImGui::SameLine();
        ImGui::Text("%s", status_text.c_str());

        ImGui::Checkbox("Launch control", &show_monlaunch_window);

        //ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::ColorEdit3("Background", (float*)&clear_color); // Edit 3 floats representing a color
        ImGui::End();
    }

    if (show_monlaunch_window) {
        ImGui::SetNextWindowPos(ImVec2(30, 160), ImGuiCond_FirstUseEver);
        monlaunch_widget->show_window(show_monlaunch_window);
    }

    ImGui::Render();

    int display_w, display_h;
    //glfwMakeContextCurrent(g_window);
    glfwGetFramebufferSize(g_window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    //glfwMakeContextCurrent(g_window);
    glfwSwapBuffers(g_window);
}

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int init()
{
    glfwSetErrorCallback(glfw_error_callback);
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        return 1;
    }

    //glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Open a window and create its OpenGL context
    int canvasWidth = 1280;
    int canvasHeight = 720;
    g_window = glfwCreateWindow(canvasWidth, canvasHeight, "ROSWASM Example", NULL, NULL);
    if( g_window == NULL )
    {
        fprintf( stderr, "Failed to open GLFW window.\n" );
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(g_window); // Initialize GLEW


#ifdef ROSWASM_NATIVE
    glfwSwapInterval(1); // Enable vsync

    // Initialize OpenGL loader
    bool err = glewInit() != GLEW_OK;
    if (err)
    {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return 1;
    }
#endif

    // Create game objects
    // Setup Dear ImGui binding
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Setup style
    ImGui::StyleColorsLight();
    //ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    //ImGui_ImplGlfw_InitForOpenGL(g_window, false);
    ImGui_ImplGlfw_InitForOpenGL(g_window, true);
    //ImGui_ImplOpenGL3_Init();
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load Fonts
    io.Fonts->AddFontDefault();
    /*
    io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 23.0f);
    io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 18.0f);
    io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 26.0f);
    io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 32.0f);
    */

    //imgui =  ImGui::GetCurrentContext();

    // Cursor callbacks
    /*
    glfwSetMouseButtonCallback(g_window, ImGui_ImplGlfw_MouseButtonCallback);
    glfwSetScrollCallback(g_window, ImGui_ImplGlfw_ScrollCallback);
    glfwSetKeyCallback(g_window, ImGui_ImplGlfw_KeyCallback);
    glfwSetCharCallback(g_window, ImGui_ImplGlfw_CharCallback);
    */

#ifndef ROSWASM_NATIVE
    resizeCanvas();
#endif

    return 0;
}

void quit()
{
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(g_window);
    glfwTerminate();
}


extern "C" int main(int argc, char** argv)
{
    if (init() != 0) return 1;

    roswasm::init(argc, argv, "roswasm_webgui");

#ifdef ROSWASM_NATIVE
    nh = new roswasm::NodeHandle();
#else
    if (argc < 3) {
        printf("Rosbridge server ip and port not provided!");
        return 1;
    }

    std::string rosbridge_ip(argv[1]);
    std::string rosbridge_port(argv[2]);
    nh = new roswasm::NodeHandle(rosbridge_ip, rosbridge_port);
#endif

  monlaunch_widget = new roswasm_webgui::MonlaunchWidget(*nh);

  roswasm::Duration loop_rate(1./20.);
  roswasm::spinLoop(loop, loop_rate);

  quit();

  return 0;
}
