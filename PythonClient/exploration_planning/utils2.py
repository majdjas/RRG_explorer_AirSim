import airsim
import sys
import math
import time
import argparse
import pprint
import numpy
from graph_manager import *
from map_manager import *
import numpy as np

def collect_lidar_data(client, vehicle, sensor_list, position, map: voxelMap):
    readings = []
    for sensor in sensor_list:

        sensor_data = client.getLidarData(lidar_name=sensor, vehicle_name=vehicle)
        orientation = sensor_data.pose.orientation
        q0, q1, q2, q3 = orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val
        rotation_matrix = np.array(([1-2*(q2*q2+q3*q3),2*(q1*q2-q3*q0),2*(q1*q3+q2*q0)],
                                    [2*(q1*q2+q3*q0),1-2*(q1*q1+q3*q3),2*(q2*q3-q1*q0)],
                                    [2*(q1*q3-q2*q0),2*(q2*q3+q1*q0),1-2*(q1*q1+q2*q2)]))

        for i in range(0, len(sensor_data.point_cloud), 3):
            xyz = sensor_data   .point_cloud[i:i+3]
            corrected_x, corrected_y, corrected_z = np.matmul(rotation_matrix, np.asarray(xyz))
            final_x = corrected_x + position[0]
            final_y = corrected_y + position[1]
            final_z = corrected_z + position[2]
            point_pos = (final_x, final_y, final_z)
            readings.append(point_pos)

    for r in readings:
        map.add_sensor_reading(quantize_coordinates(position), quantize_coordinates(r))
    return


def move_on_path(client, path, vel, timeout):
    client.moveOnPathAsync(path, vel, timeout).join()
    print("Completed local path")


def sense_loop(client, vehicle, sensor_list, destination, map): # TODO communicate end of path
    state = client.getMultirotorState()
    drone_position = tuple(state.kinematics_estimated.position)
    while calc_dist(destination, drone_position) > 1:
        print("ON LOCAL PATH")
        collect_lidar_data(client, vehicle, sensor_list, drone_position, map)
        state = client.getMultirotorState()
        drone_position = tuple(state.kinematics_estimated.position)
    return