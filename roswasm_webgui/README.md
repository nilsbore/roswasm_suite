# roswasm_webgui

roswasm_webgui is a library for writing web GUIs for ROS using [roswasm](https://github.com/nilsbore/roswasm_suite/tree/master/roswasm).
It also provides example web interfaces, one of which is used for our [underwater robot](https://github.com/nilsbore/sam_webgui).
The resulting web pages are built using wasm and have been tested using Chrome and Firefox.

The code has been adapted from the [WebGui](https://github.com/jnmaloney/WebGui) example by jnmaloney.

The latest [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) is needed
to get the effective [`cbor-raw` encoding](https://github.com/RobotWebTools/rosbridge_suite/commit/dc7fcb282d1326d573abe83579cc7d989ae71739) when subscribing to messages in the browser.

## Running the example with std_msgs

First clone the latest rosbridge_suite to your catkin workspace:
```
git clone https://github.com/RobotWebTools/rosbridge_suite
```
Then compile that aswell as this package within the catkin workspace, source and launch using
```
roslaunch roswasm_webgui example.launch
```
Then navigate to `localhost:8080` to the see the webpage.

If you want to get some more interesting stuff to play with,
try launching with [mon launch](http://wiki.ros.org/rosmon) instead.
roswasm_webgui has good facilities for remote launching using this tool.
The example [tmux startup script](https://github.com/nilsbore/roswasm_webgui/blob/master/scripts/example.sh)
demonstrates how the tool can be used to remotely start up a system.
