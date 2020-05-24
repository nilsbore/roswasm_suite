# roscpp_json_serialize
Header library for off-the-shelf JSON serialization for ROS messages in C++

The following example illustrates the functionality:
```cpp
#include <roscpp_json_serialize/serialize.h>
#include <roscpp_json_serialize/deserialize.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv)
{
    // initialize and fill in a few elements of a msg object
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp.sec = 3;
    msg.header.stamp.nsec = 4;
    msg.header.frame_id = "base_link";
    
    // serialize to json using roscpp_json_serialize
    std::string json_msg = roscpp_json::serialize(msg);

    // std::cout << json_msg << std::endl;
    /* prints
    { 
      "header": { "seq": 0, "stamp": { "secs": 3, "nsecs": 4 }, "frame_id": "base_link" },
      "pose": { "pose": { "position": { "x": 0, "y": 0, "z": 0 },
      "orientation": { "x": 0, "y": 0, "z": 0, "w": 0 } },
      "covariance": [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ] }
    } */

    // parse a new message object with the same values as msg
    geometry_msgs::PoseWithCovarianceSta mped parsed_msg =
        roscpp_json::deserialize<geometry_msgs::PoseWithCovarianceStamped>(json_msg);
    
    return 0;
}
```
