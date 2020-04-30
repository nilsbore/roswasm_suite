# roswasm_webgui

roswasm_webgui is a library for writing web GUIs for ROS using [roswasm](https://github.com/nilsbore/roswasm).
It also provides example web interfaces, one of which is used for our [underwater robot](https://github.com/smarc-project).
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

## Running with SAM custom ROS msgs (for sim and AUV)


First clone the latest rosbridge_suite to your catkin workspace:
```
git clone https://github.com/RobotWebTools/rosbridge_suite
```
Then compile that aswell as this package within the catkin workspace, source and launch using
```
roslaunch roswasm_webgui sam_webgui.launch
```
Then navigate to `localhost:8080` to the see the webpage.
If you want the page to be accessible from another computer
with IP 1.2.3.4, you have to provide another parameter:
```
roslaunch roswasm_webgui sam_webgui.launch rosbridge_ip:=1.2.3.4
```
You will then be able to access the page from another computer on
the network at the address `1.2.3.4:8080`.

## *NOT NEEDED FOR RUNNING!* Building the packages

First off, you need to get the roswasm submodule by typing `git submodule init --update` within this folder.
If you just want to build the example with `std_msgs`, simply
build it inside this folder by typing `make`. For the AUV
specific page, you need to also clone [sam_common](https://github.com/smarc-project/sam_common)
and build the package, and also replace the hardcoded include
path in the [make file](https://github.com/nilsbore/roswasm_webgui/blob/master/Makefile).
