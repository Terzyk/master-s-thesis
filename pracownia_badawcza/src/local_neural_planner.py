#!/usr/bin/env python

from __future__ import print_function

from pracownia_badawcza.srv import LNP,LNPResponse
import rospy

def handle_LNP(req):
    #print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    s_factors = [req.s1_x, req.s1_y, req.s1_yaw, req.s2_x, req.s2_y, req.s2_yaw]
    return LNPResponse(s_factors)

def LNP_server():
    rospy.init_node('local_neural_planner_server')
    s = rospy.Service('LNP', LNP, handle_LNP)
    print("Server working")
    rospy.spin()

if __name__ == "__main__":
    LNP_server()
