import airsim
import sys
import math
import time
import argparse
import pprint
import numpy
from graph_manager import *
from map_manager import *
from utils2 import *
import numpy as np

if __name__=='__main__':
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    print("arming the drone...")
    client.armDisarm(True)
    client2 = airsim.MultirotorClient()
    landed = client.getMultirotorState().landed_state
    if landed == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync().join()
        print("...took off")

    vehicle = 'Drone1'
    sensor_list = ['L_front', 'L_back', 'L_left', 'L_right', 'L_up', 'L_down']

    state = client.getMultirotorState()
    drone_position = tuple(state.kinematics_estimated.position)
    home_node = drone_position
    drone_orientation = tuple(state.kinematics_estimated.orientation)

    map = voxelMap(((-400, -400, -400), (400, 400, 400)), set(), set(), set(), dict(), 10)
    rrg = RRG(home_node, 10, 2, 25, 200, 1, 1, 1)

    collect_lidar_data(client, vehicle, sensor_list, drone_position, map)

    while True: # TODO sync data with ..
        state = client.getMultirotorState()
        drone_position = tuple(state.kinematics_estimated.position)
        local_path = rrg.plan_local_path(map, quantize_coordinates(drone_position))
        local_path_ref = [airsim.Vector3r(p[0], p[1], p[2]) for p in local_path]
        destination = local_path[-1]

        print(local_path)
        client.moveOnPathAsync(local_path_ref, 1, 45)

        state = client.getMultirotorState()
        drone_position = tuple(state.kinematics_estimated.position)

        while calc_dist(drone_position, destination) > 5:
            collect_lidar_data(client, vehicle, sensor_list, drone_position, map)
            state = client.getMultirotorState()
            drone_position = tuple(state.kinematics_estimated.position)
            client.simPlotPoints([state.kinematics_estimated.position], color_rgba=[1,0,0,1], size=10, is_persistent=True)
            client.simPlotStrings(["{}".format(drone_position)], positions=[state.kinematics_estimated.position],duration=0.1, scale=1)