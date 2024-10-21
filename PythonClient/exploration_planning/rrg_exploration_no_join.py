from socket import gethostbyaddr
import airsim
import sys
import math
import time
import argparse
import pprint
from airsim.types import Vector3r
from networkx import voterank
import numpy
from graph_manager import *
from map_manager import *
from utils2 import *
from gps_manager import *
import numpy as np
import pymap3d

# TODO
# DONT make dense graph (dont append every local graph)
# Remake los check
# Check window and VG
# Why does robot stop after completing path
# quantize random sampling
# check window sliding
# i am overwriting too much free voxels
# implement c-space by shortening lidar readings

if __name__=="__main__":
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    print("arming the drone...")
    client.armDisarm(True)
    # client2 = airsim.MultirotorClient()
    landed = client.getMultirotorState().landed_state
    if landed == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync().join()
        print("...took off")

    vehicle = "Drone1"
    sensor_list = ["L_front", "L_back", "L_left", "L_right", "L_up", "L_down"]
    #sensor_list = ["L"]

    state = client.getMultirotorState()
    drone_position = tuple(state.kinematics_estimated.position)
    home_node = drone_position
    drone_orientation = tuple(state.kinematics_estimated.orientation)

    map = voxelMap(((-400, -400, -400), (400, 400, 400)), set(), set(), set(), dict(), 10) #TODO chage name
    gps = GPSManager((state.gps_location.latitude, state.gps_location.longitude, state.gps_location.altitude))

    print(state.can_arm)
    print(state.collision)
    print(state.ready)
    print(state.ready_message)

    #print(client.simTestLineOfSightToPoint(state.gps_location, vehicle))###

    #plot_voxels = [Vector3r(v[0], v[1], v[2]) for v in map.window_set]
    #client.simPlotPoints(plot_voxels, color_rgba=[0,0,1,0.1], size=10, is_persistent=True, duration=1)
    

    rrg = RRG(home_node, 10, 2, 25, 500, 1, 1, 1)

    collect_lidar_data(client, vehicle, sensor_list, drone_position, map)
    collect_los_data(client, vehicle, drone_position, map, gps)

    for _ in range(10): # TODO sync data with ..
        state = client.getMultirotorState()
        drone_position = tuple(state.kinematics_estimated.position)
        local_path = rrg.plan_local_path(map, quantize_coordinates(drone_position))
        local_path_ref = [airsim.Vector3r(p[0], p[1], p[2]) for p in local_path]
        destination = local_path[-1]

        print(local_path)
        client.moveOnPathAsync(local_path_ref, 1, 20)

        state = client.getMultirotorState()
        drone_position = tuple(state.kinematics_estimated.position)

        while len(client.client._request_table) > 0:
            collect_lidar_data(client, vehicle, sensor_list, drone_position, map)
            collect_los_data(client, vehicle, drone_position, map, gps)

            state = client.getMultirotorState()
            drone_position = tuple(state.kinematics_estimated.position)
            client.simPlotPoints([state.kinematics_estimated.position], color_rgba=[1,1,1,1], size=10, is_persistent=True)
            client.simPlotStrings(["{}".format(drone_position)], positions=[state.kinematics_estimated.position],duration=0.25, scale=1)

        ###
        #for v in map.hollow_set:
        #    client.simPlotPoints([Vector3r(v[0], v[1], v[2])], color_rgba=[0,0,1,0.2], size=5, is_persistent=True, duration=1) 
        ###

    print("DONE")

    show_rrg(client, rrg)

    for v in map.known_map:
        if map.known_map[v] == voxelType.OCCUPIED:
            client.simPlotPoints([Vector3r(v[0],v[1],v[2])], color_rgba=[1,0,0,1], size=30, is_persistent=True)
        #elif map.known_map[v] == voxelType.FREE:
            #client.simPlotPoints([Vector3r(v[0],v[1],v[2])], color_rgba=[0,0,1,0.2], size=6, is_persistent=True)

    print("DONE 2")

    while True:
        continue

            