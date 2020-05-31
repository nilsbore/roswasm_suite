# roswasm_webgui

![Example GUI](https://raw.githubusercontent.com/nilsbore/roswasm_suite/master/roswasm_webgui/media/example_gui.png)

roswasm_webgui is a library for writing web GUIs for ROS using [roswasm](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm).
It also provides example web interfaces, one of which is used for our [underwater robot](https://github.com/nilsbore/sam_webgui).
The resulting web pages are built using wasm and have been tested using Chrome and Firefox.

The code has been adapted from the [WebGui](https://github.com/jnmaloney/WebGui) example by jnmaloney.

The latest [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) is needed
to get the effective [`cbor-raw` encoding](https://github.com/RobotWebTools/rosbridge_suite/commit/dc7fcb282d1326d573abe83579cc7d989ae71739) when subscribing to messages in the browser.

## Library details

[The example main file](https://github.com/nilsbore/roswasm_suite/blob/master/roswasm_webgui/src/main.cpp)
serves as an example of how to set up imgui and the roswasm connections.

Apart from this, there are two main components to the library:

### Imgui headers and library

By linking to this package in your node's cmake file using `${roswasm_webgui_LIBRARIES}`, you will link the built imgui sources.
In addition, by including `${catkin_INCLUDE_DIRS}`, you get access to the corresponding headers in `roswasm_webgui/imgui/*`.

### Topic Buffer and Topic Widget

There are two example classes that facilitate writing ROS GUIs using imgui.

* [`TopicBuffer`](https://github.com/nilsbore/roswasm_suite/blob/master/roswasm_webgui/include/roswasm_webgui/roswasm_widget.h#L13) subscribes to your message and keeps the latest one in a buffer. This is handy when you just need to display the value of a topic
* [`TopicWidget`](https://github.com/nilsbore/roswasm_suite/blob/master/roswasm_webgui/include/roswasm_webgui/roswasm_widget.h#L37) subscribes to a feedback topic and keeps it in a buffer. In addition, it takes a `std::function` argument that should do the imgui draw call and that can also publish any control updates to another topic. This is convenient when you have a control that should reflect the value of a topic and allow publishing to that or another topic

There are plenty of examples of the usage of both in [the examples library](https://github.com/nilsbore/roswasm_suite/blob/master/roswasm_webgui/include/roswasm_webgui/roswasm_examples.h).

### Implemented Widgets

![Monlaunch GUI](https://raw.githubusercontent.com/nilsbore/roswasm_suite/master/roswasm_webgui/media/monlaunch_gui.png)

* [ImageWidget](https://github.com/nilsbore/roswasm_suite/blob/master/roswasm_webgui/include/roswasm_webgui/roswasm_image.h) allows you to subscribe to and display images published as `sensor_msgs/CompressedImage` (see example in GUI at page start)
* [MonlaunchWidget](https://github.com/nilsbore/roswasm_suite/blob/master/roswasm_webgui/include/roswasm_webgui/roswasm_monlaunch.h) allows displaying all launch instances launched using `mon launch` and startup and restart nodes together or individually (see image above)

## Running the example with std_msgs

First clone the latest rosbridge_suite to your catkin workspace:
```bash
git clone https://github.com/RobotWebTools/rosbridge_suite
```
Then compile that aswell as this package within the catkin workspace, source and launch using
```bash
roslaunch roswasm_webgui example_gui.launch
```
Then navigate to `localhost:8080` to the see the webpage.

If you want to get some more interesting stuff to play with,
try launching with [mon launch](http://wiki.ros.org/rosmon) instead.
roswasm_webgui has good facilities for remote launching using this tool.
The example [tmux startup script](https://github.com/nilsbore/roswasm_webgui/blob/master/scripts/example.sh)
demonstrates how the tool can be used to remotely start up a system.
