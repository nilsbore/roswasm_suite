# roswasm
ROS C++ client library for emscripten WASM

## Intro

Under the hood, roswasm uses both native ROS binary serialization and [json serialization](https://github.com/nilsbore/roswasm_suite/tree/master/roscpp_json_serialize)
to provide an interface that looks like native roscpp when communcating through
`rosbridge_websocket` (see details below).

roswasm supports the following primitives, mirroring the ones in roscpp.
For complete example nodes, see [roswasm_tutorials](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm_tutorials).

### NodeHandle

You can create several NodeHandles. They can not be configured with a namespace.
To achieve this, namespace the `rosbridge_websocket` instance instead.
```cpp
#include <roswasm/roswasm.h>
...
roswasm::NodeHandle* nh = new roswasm::NodeHandle();
```

### Subscriber

Subscribers receive data through the new [cbor-raw](https://github.com/RobotWebTools/rosbridge_suite/commit/dc7fcb282d1326d573abe83579cc7d989ae71739)
compression protocol. In practice, this means `rosbridge_websocket` transmits the binary
serialized ROS messages, making overhead minimal.
```cpp
void string_callback(const std_msgs::String& msg) {}
...
roswasm::Subscriber* sub = nh->subscribe<std_msgs::String>("test", callback);
```
To throttle the rate of messages to at most 1 message every 100 ms (10Hz), change the call to
```
roswasm::Subscriber* sub = nh->subscribe<std_msgs::String>("test", callback, 100);
```

### Publisher

Publishers use the `json` compression protocol, since it is expected that the
bandwidth of publishers will be low.
```cpp
roswasm::Publisher* pub = nh->advertise<std_msgs::String>("test");
std_msgs::String msg;
msg.data = "TEST";
pub->publish(msg);
```

### ServiceClient

Similarly, ServiceClients also send and receive data using json serialization.
```cpp
roswasm::ServiceClient* service = nh->serviceClient<rosapi::TopicType>("/rosapi/topic_type", service_callback);
rosapi::TopicType::Request req;
req.topic = "/connected_clients";
service->call<rosapi::TopicType>(req);
```

### Timer

Timers adhere to the roscpp timer callback interface. However, the `TimerEvent` will
not contain any meaningful data.
```cpp
void callback(const ros::TimerEvent& ev) {}
...
roswasm::Timer* timer = nh->createTimer(5., callback);
```

## Building

Any nodes that are to use roswasm and compile using Emscripten
just need to depend on `roswasm` in the catkin build configuration.
This will automatically pull in the build configuration by
importing a [`CFG_EXTRAS` Cmake file](https://github.com/nilsbore/roswasm_suite/blob/master/roswasm/cmake/roswasm-extras.cmake.in).
Note that some configurations still are necessary and that the
produced Webassembly and JavaScript files need to be imported
by an html file in order to run in the web browser.
See the [tutorials package](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm_tutorials)
for examples on the necessary configurations.

## Running


Before starting any of the roswasm nodes, you need to have one instance of `rosbridge_websocket` running:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
After building your node, you can run it similar to the following command:
```
rosrun roswasm run.py _pkg:=roswasm_tutorials _node:=listener.html _display_port:=8080
```
This will start a webserver and allow you to view the page at `localhost:8080`.
If you want to see the output from the node, open the browser debug console.
