#!/usr/bin/env python

from __future__ import print_function

from pracownia_badawcza.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    #print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    s_factors = [req.s1_x, req.s1_y, req.s1_yaw, req.s2_x, req.s2_y, req.s2_yaw]
    return AddTwoIntsResponse(s_factors)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Server working")
    rospy.spin()

if __name__ == "__main__":
    #print("EEEEEEEEE")
    add_two_ints_server()
