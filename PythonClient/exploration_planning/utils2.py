import airsim
import sys
import math
import time
import argparse
import pprint
from airsim.client import Vector2r
from airsim.types import Vector3r
from networkx import all_pairs_bellman_ford_path
import numpy
from graph_manager import *
from map_manager import *
from gps_manager import *
from utils import *
import numpy as np
import pymap3d
import open3d as o3d

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
            xyz = sensor_data.point_cloud[i:i+3]
            corrected_x, corrected_y, corrected_z = np.matmul(rotation_matrix, np.asarray(xyz))
            final_x = corrected_x + position[0]
            final_y = corrected_y + position[1]
            final_z = corrected_z + position[2]
            point_pos = (final_x, final_y, final_z)
            #client.simPlotPoints([Vector3r(point_pos[0], point_pos[1], point_pos[2])], color_rgba=[1,1,0,0.1], size=5, is_persistent=True, duration=1) ###
            #map.add_sensor_reading_short(quantize_coordinates(position), quantize_coordinates(point_pos), False)
            map.add_sphere_obstacle(quantize_coordinates(point_pos))
    return


def collect_los_data(client, vehicle, pos, map:voxelMap, gps:GPSManager):
    map.add_sensor_readings_los(quantize_coordinates(pos))
    #add_sensor_readings_los_with_callback(quantize_coordinates(pos), check_los_callback, (client, vehicle, gps))


def check_los_callback(pos_ned, args: tuple): #TODO type check tuple of...
    client = args[0]
    vehicle = args[1]
    gps = args[2]

    pos_geo_p = airsim.GeoPoint()
    pos_geo = gps.ned_to_geo(pos_ned)
    pos_geo_p.latitude = pos_geo[0]
    pos_geo_p.longitude = pos_geo[1]
    pos_geo_p.altitude = pos_geo[2]
    return client.simTestLineOfSightToPoint(pos_geo_p, vehicle) # TODO gps manager singleton?


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


def test_los_callback(voxel, args: tuple):
    client = args[0]
    vehicle = args[1]

    client.simTestLineOfSightToPoint(voxel, vehicle)


def show_rrg(client: airsim.VehicleClient, rrg: RRG):
    for v in rrg.graph.nodes:
        client.simPlotPoints([Vector3r(v[0], v[1], v[2])], color_rgba=[1,0,1,1], size=10, is_persistent=True) 
    #for e in rrg.graph.edges:
        #client.simPlotLineStrip([Vector3r(e[0][0], e[0][1], e[0][2]), Vector3r(e[1][0], e[1][1], e[1][2])], color_rgba=[1,1,0,1], is_persistent=True, thickness=1)