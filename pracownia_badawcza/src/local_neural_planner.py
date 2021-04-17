#!/usr/bin/env python3

from __future__ import print_function

from python_neural_pkg.utils.test import run_and_plot,run_and_plot_white


from pracownia_badawcza.srv import LNP,LNPResponse
import rospy
import math
import cv2
import numpy as np


map_white_small_path = r'/home/karol/Studia/mgr/catkin_ws/src/pracownia_badawcza/maps/map_white_small.png'


def calculate_factors(x0, y0, th0, x1, y1, th1):

    ff = y0
    ee = math.tan(th0)
    dd = 0.0
    w = y1 - ee*x1
    z = math.tan(th1) - ee
    aa = (4.0*w*z - x1*x1*z + 3*z*x1)/(4.0*pow(x1,5))
    cc = (4.0*w*z + 16.0*w - x1*x1*z - x1*z)/(4.0*x1*x1*x1)
    bb = (z - 5*aa*pow(x1,4) - 3.0*cc*x1*x1)/(4.0*x1*x1*x1)

    return [aa,bb,cc,dd,ee,ff]




def handle_LNP(req):


    grid_map = req.grid_map

    img = np.zeros(shape = (128,128), dtype = np.int8)



    for i in range(0,int(len(grid_map)/128)):
        img[i] = grid_map[i*128:(i+1)*128]
        

    # print(img.shape)


    # cv2.imshow('image',img)
    # cv2.waitKey(0)

    # threshold = 90
    # binarized = 1.0 * (img > threshold)
    binarized = img 
    # cv2.imshow('image',binarized)
    # cv2.waitKey(0)

    s_factors = [req.s_x, req.s_y, req.s_yaw]
    res_list = []

    if len(s_factors) > 0:
        x,y,th = run_and_plot(binarized, req.s_x, req.s_y, req.s_yaw)

        for i in range(0,len(x)):
            # print('x first: ' + str(x[i][0]) + '| y first: ' + str(y[i][0]) + '| th first: ' + str(th[i][0]))
            # print('x last: ' + str(x[i][-1]) + '| y last: ' + str(y[i][-1]) + '| th last: ' + str(th[i][-1]))
            factors = calculate_factors(x[i][0], y[i][0], th[i][0], x[i][-1], y[i][-1], th[i][-1])
            for j in factors:
                res_list.append(j*10000.0)
            # print(res_list)
        return LNPResponse(res_list)

def handle_LNP_white(req):

    s_factors = [req.s_x, req.s_y, req.s_yaw]
    res_list = []

    if len(s_factors) > 0:

        x,y,th = run_and_plot_white(map_white_small_path, req.s_x, req.s_y, req.s_yaw)

            
        for i in range(0,len(x)):
            print('x first: ' + str(x[i][0]) + '| y first: ' + str(y[i][0]) + '| th first: ' + str(th[i][0]))
            print('x last: ' + str(x[i][-1]) + '| y last: ' + str(y[i][-1]) + '| th last: ' + str(th[i][-1]))
            factors = calculate_factors(x[i][0], y[i][0], th[i][0], x[i][-1], y[i][-1], th[i][-1])
            for j in factors:
                res_list.append(j*10000.0)
            print(res_list)
        return LNPResponse(res_list)
    

def LNP_server():
    global map_white_small_path
    use_white_map = False
    rospy.init_node('local_neural_planner_server')
    if not use_white_map:
        print("not white")
        s = rospy.Service('LNP', LNP, handle_LNP)
    else:
        print("white")
        s = rospy.Service('LNP', LNP, handle_LNP_white)
    print("Server working")




    #input()
    rospy.spin()

if __name__ == "__main__":
    #print("Waiting for server working")
    #rospy.wait_for_service('local_neural_planner_server')
    LNP_server()
