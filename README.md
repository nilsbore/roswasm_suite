# roswasm_suite
![Build Status](https://github.com/nilsbore/roswasm_suite/workflows/CI/badge.svg)[![license](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/nilsbore/roswasm_suite/blob/master/LICENSE.md)

Libraries for compiling C++ ROS nodes to Webassembly using Emscripten. Allows you to write C++ ROS nodes similar to how you would otherwise, and to then have them run in a webbrowser and communicate with ROS through `rosbridge_websocket`.

## Dependencies

* [Emscripten](https://emscripten.org/docs/getting_started/downloads.html) tested with version `1.39.10` but latest should do
* [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) after [the commit adding cbor-raw compression](https://github.com/RobotWebTools/rosbridge_suite/commit/dc7fcb282d1326d573abe83579cc7d989ae71739), latest `develop` should do:
  ```bash
  git clone https://github.com/RobotWebTools/rosbridge_suite -b develop # in workspace src folder
  ```

## Building

`catkin build` is recommended, since `catkin_make` might leak configurations to other packages.
Make sure to [source the Emscripten environment](https://emscripten.org/docs/getting_started/downloads.html#installation-instructions)
before building the package:
```bash
source /path/to/emsdk/emsdk_env.sh
catkin build
```

## Packages

* [`roscpp_json_serialize`](https://github.com/nilsbore/roswasm_suite/tree/master/roscpp_json_serialize) - library that serializes ROS messages to and from json
* [`roswasm`](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm) - contains the `roswasm` client library and configures cmake to build dependent packages using Emscripten. It supports [publishers](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm#publisher), [subscribers](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm#subscriber), [service clients](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm#serviceclient) and [timers](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm#timer)
* [`roswasm_tutorials`](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm_tutorials) - contains examples corresponding to `talker`, `listener` and `timers` of [`roscpp`](https://github.com/ros/ros_tutorials/tree/noetic-devel/roscpp_tutorials)
* [`roswasm_webgui`](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm_webgui) - a proof of concept implementation of a ROS GUI library based on `roswasm` and [`imgui`](https://github.com/ocornut/imgui)

## Writing a roswasm node

The `roswasm` client library presents an API similar to `roscpp`, with the
main differences being that most interfaces are heap allocated, and that Emscripten
manages the event loop. Below is a shortened version of the corresponding
`listener` example implementation.

```cpp
#include <emscripten.h>
#include <roswasm/roswasm.h>
#include <std_msgs/String.h>

roswasm::NodeHandle* n;
roswasm::Subscriber* sub;

void chatterCallback(const std_msgs::String& msg)
{
    printf("I heard: [%s]\n", msg.data.c_str());
}

void loop() {}

extern "C" int main(int argc, char** argv)
{
    n = new roswasm::NodeHandle();
    sub = n->subscribe<std_msgs::String>("chatter", chatterCallback);
    emscripten_set_main_loop(loop, 10, 1);
    return 0;
}

```
For more complete examples, see the [`roswasm_tutorials` package](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm_tutorials).

## Building a roswasm package

The `roswasm` library uses `catkin` to build an emscripten project.
All you have to do in your package that is using `roswasm` is to add
it as a dependency in your `cmake` file and link against
`${roswasm_LIBRARIES}`. It will automatically set the Emscripten
`em++` compiler as the default for building and linking nodes.
Note that you have to install and source the emscripten SDK before
building your workspace. The following minimal `cmake` file includes
a guard to check that Emscripten is present.
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(listener)

find_package(catkin REQUIRED COMPONENTS roscpp roswasm std_msgs)
catkin_package()

if (DEFINED ENV{EMSDK})

include_directories(${catkin_INCLUDE_DIRS})
add_executable(listener src/listener.cpp)
set_target_properties(listener PROPERTIES OUTPUT_NAME "listener.js")
target_link_libraries(listener ${roswasm_LIBRARIES})
configure_file(www/listener.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/listener.html COPYONLY)

endif()
```

## Running an Emscripten node

### Run node separately

Before starting any of the roswasm nodes, you need to have one instance of `rosbridge_websocket` running:
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
After building your node, you can run it similar to the following command:
```bash
rosrun roswasm run.py _pkg:=roswasm_tutorials _node:=listener.html _display_port:=8080
```
This will start a webserver and allow you to view the page at `localhost:8080`.
If you want to see the output from the node, open the browser debug console.

### Combined launch file (alternative)

There is a also a convenience launch file to run both `rosbridge_websocket`
as well as your `roswasm` node at the same time:
```bash
roslaunch roswasm run_server.launch pkg:=roswasm_tutorials node:=listener.html
```
