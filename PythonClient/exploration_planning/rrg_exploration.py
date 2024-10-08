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
import threading

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

    #time.sleep(5)

    vehicle = 'Drone1'
    sensor_list = ['L_front', 'L_back', 'L_left', 'L_right', 'L_up', 'L_down']

    state = client.getMultirotorState()
    drone_position = tuple(state.kinematics_estimated.position)
    home_node = drone_position
    drone_orientation = tuple(state.kinematics_estimated.orientation)

    #client.moveToPositionAsync(drone_position[0], drone_position[1], 10, 3).join()

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
        t1 = threading.Thread(target=move_on_path, args=(client, local_path_ref, 1, 300))
        t2 = threading.Thread(target=sense_loop, args=(client2, vehicle, sensor_list, destination, map))

        t1.start()
        t2.start()
        print("started threads")

        t1.join()
        t2.join()
        print("ended threads")
        