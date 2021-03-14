#!/usr/bin/env python

from __future__ import print_function

from models.planner import PlanningNetworkMP, plan_loss
from utils.crucial_points import calculate_car_crucial_points
import tensorflow as tf
import numpy as np

from pracownia_badawcza.srv import AddTwoInts,AddTwoIntsResponse
import rospy


def run_and_plot(model_path, map_path, as_path, xd, yd, thd, ax):
    bs = 128
    map = read_map(map_path)[tf.newaxis]
    p0 = np.array([0.4, 0., 0., 0.], dtype=np.float32)[np.newaxis]
    pk = np.array([xd, yd, thd, 0.], dtype=np.float32)[np.newaxis]
    path = np.stack([p0, pk], axis=1)
    ddy0 = np.array([0.], dtype=np.float32)
    data = (map, path, ddy0)

    # 2. Define model
    model = PlanningNetworkMP(7, (bs, 6))

    # 3. Optimization

    optimizer = tf.keras.optimizers.Adam(1e-4)

    # 4. Restore, Log & Save
    experiment_handler = ExperimentHandler(".", "", 1, model, optimizer)
    experiment_handler.restore(model_path)

    output, last_ddy = model(data, None, training=True)
    model_loss, invalid_loss, overshoot_loss, curvature_loss, non_balanced_loss, _, x_path, y_path, th_path = plan_loss(
        output, data, last_ddy)


def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    print("EEEEEEEEE")
    add_two_ints_server()
