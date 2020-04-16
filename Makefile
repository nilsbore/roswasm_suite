#
# Cross Platform Makefile
# Compatible with MSYS2/MINGW, Ubuntu 14.04.1 and Mac OS X
#
# You will need GLFW (http://www.glfw.org):
# Linux:
#   apt-get install libglfw-dev
# Mac OS X:
#   brew install glfw
# MSYS2:
#   pacman -S --noconfirm --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-glfw
#

CXX = emcc
OUTPUT = www/imgui.js

SOURCES = src/main.cpp
SOURCES += src/imgui_impl_glfw.cpp src/imgui_impl_opengl3.cpp
SOURCES += src/imgui.cpp src/imgui_draw.cpp src/imgui_demo.cpp
SOURCES += external/roswasm/src/roswasm.cpp
SOURCES += src/roswasm_monlaunch.cpp src/roswasm_image.cpp

LIBS = -lGL -lwebsocket.js
WEBGL_VER = -s USE_WEBGL2=1 -s USE_GLFW=3 -s FULL_ES3=1
#WEBGL_VER = USE_GLFW=2
USE_WASM = -s WASM=1

all: $(SOURCES) $(OUTPUT)

$(OUTPUT): $(SOURCES)
	$(CXX)  $(SOURCES) -std=c++11 -o $(OUTPUT) $(LIBS) $(WEBGL_VER) -O2 --preload-file data $(USE_WASM) -I include -I /opt/ros/$(ROS_DISTRO)/include -I external/roswasm/include -I external/roswasm/external/cbor-lite/include -I external/roswasm/external/roscpp_json_serialize/include -I external/roswasm/external/roscpp_json_serialize/external/rapidjson/include -s USE_BOOST_HEADERS=1 -s USE_SDL=2 -s USE_SDL_IMAGE=2 -s SDL2_IMAGE_FORMATS='["png", "jpg"]'

clean:
	rm -f $(OUTPUT)
