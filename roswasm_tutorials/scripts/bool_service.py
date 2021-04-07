#!/usr/bin/env python

from __future__ import print_function

from std_srvs.srv import SetBool, SetBoolResponse
import rospy

def handle_bool(req):
    return SetBoolResponse(True, "Here it comes!")

def bool_server():
    rospy.init_node('bool_server')
    s = rospy.Service('/test_bool', SetBool, handle_bool)
    print("Ready to take bool.")
    rospy.spin()

if __name__ == "__main__":
    bool_server()
