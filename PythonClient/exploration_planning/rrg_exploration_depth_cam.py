import airsim
import time
import pprint
from airsim.types import Vector3r
from graph_manager import *
from map_manager import *
from utils2 import *
from gps_manager import *
from camera_manager import *
from sensor_manager import *
import numpy as np
import pymap3d
import argparse
import json
import os
import re
import open3d as o3d
import pandas as pd
import PIL.Image
from tqdm import tqdm

# TODO
# DONT make dense graph (dont append every local graph)
# Remake los check
# Check window and VG
# Why does robot stop after completing path
# quantize random sampling
# i am overwriting too much free voxels
# leaf vertices
# think about surface normal in sensor
# drivetrain setting for variable orientation
# deal w/ unkown voxels not observable: ommit vg or test los
# compare bresenahms
# more points vs voxel
# geometry shadow
# tree collision detection
# voxel occlusion culling cone tracing
# should bounds affect vol gain or only map?
# instead of random sampling: exclude occ voxs in radius once, index free space
# how many voxels are missed in bresenhams
# implement float line segments

# capture pose as close as possible to reads
# implement queue
# care mutable defaults/ statics /idk
# quantize coordinates only in map class (check all refs to quantize_coords)
# wait complete stop before planning
# warn if values are not compatible eg num new vertices too large for window size

if __name__=="__main__":
    '''airsim_path = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim')

    # Load the settings file
    with open(os.path.join(airsim_path, 'settings.json'), 'r') as fp:
        data = json.load(fp)

    # Get the camera intrinsics
    capture_settings = data['CameraDefaults']['CaptureSettings'][0]
    img_width = capture_settings['Width']
    img_height = capture_settings['Height']
    img_fov = capture_settings['FOV_Degrees']

    # Compute the focal length
    fov_rad = img_fov * np.pi/180
    fd = (img_width/2.0) / np.tan(fov_rad/2.0)

    # Create the camera intrinsic object
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(img_width, img_height, fd, fd, img_width/2 - 0.5, img_height/2 - 0.5)

    #camera_list = ["C_front", "C_back", "C_left", "C_right", "C_up", "C_down"]'''

    vehicle = "Drone1"
    sensor_list = ["L_front", "L_back", "L_left", "L_right", "L_up", "L_down"]
    #sensor_list = ["L"]

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    print("arming the drone...")
    client.armDisarm(True)
    client.takeoffAsync().join()
    drone_position = client.getMultirotorState().kinematics_estimated.position
    client.moveToPositionAsync(drone_position.x_val, drone_position.y_val, -4, 1).join()

    state = client.getMultirotorState()
    drone_position = tuple(state.kinematics_estimated.position)
    home_node = drone_position
    drone_orientation = tuple(state.kinematics_estimated.orientation)

    vmap = voxelMap(((-400, -400, -400), (400, 400, 400)), dict(), 20, 1)
    gps = GPSManager((state.gps_location.latitude, state.gps_location.longitude, state.gps_location.altitude))
    rrg = RRG(home_node, 20, 10, 50, 500, 0.01, 1, 1)
    sens = SensorManager(vmap, client, vehicle, sensor_list, 5)
    #cam = CamManager(intrinsic, 25, vmap, client, camera_list)

    #print(client.simTestLineOfSightToPoint(state.gps_location, vehicle))
    #plot_voxels = [Vector3r(v[0], v[1], v[2]) for v in vmap.window_set]
    #client.simPlotPoints(plot_voxels, color_rgba=[0,0,1,0.1], size=10, is_persistent=True, duration=1)
    
    ###
    #for v in vmap.known_map:
    #    if vmap.known_map[v] == voxelType.OCCUPIED:
    #        client.simPlotPoints([Vector3r(v[0],v[1],v[2])], color_rgba=[1,0,0,1], size=30, is_persistent=True)
    ###
    local_path = [drone_position]
    for _ in range(10): # TODO sync data with ..
        #collect_lidar_data(client, vehicle, sensor_list, drone_position, vmap)
        #collect_los_data(client, vehicle, drone_position, vmap, gps)
        sens.collect_data()
        sens.process_data()
        sens.process_los()

        state = client.getMultirotorState()
        drone_position = tuple(state.kinematics_estimated.position)

        for p in local_path:
            collect_los_data(client, vehicle, p, vmap, gps)
        sens.pos_history.clear()
        #drone_position = cam.point_cloud()

        local_path = rrg.plan_local_path(vmap, quantize_coordinates(drone_position))
        local_path_ref = [airsim.Vector3r(p[0], p[1], p[2]) for p in local_path]
        print(local_path)
        client.moveOnPathAsync(local_path_ref, 2, 20)

        while len(client.client._request_table) > 0:
            #collect_lidar_data(client, vehicle, sensor_list, drone_position, vmap)
            #collect_los_data(client, vehicle, drone_position,vmap, gps)
            sens.collect_data()
            sens.process_data()

            #state = client.getMultirotorState()
            #drone_position = tuple(state.kinematics_estimated.position)
            ##drone_position = cam.point_cloud()
            print("read")
            
            #client.simPlotPoints([state.kinematics_estimated.position], color_rgba=[1,1,1,1], size=10, is_persistent=True)
            #client.simPlotStrings(["{}".format(drone_position)], positions=[state.kinematics_estimated.position],duration=0.25, scale=1)
            #im = client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.SurfaceNormals, True, False)])

        ###
        #for v in vmap.hollow_set:
        #    client.simPlotPoints([Vector3r(v[0], v[1], v[2])], color_rgba=[0,0,1,0.2], size=5, is_persistent=True, duration=1) 
        ###

    print("DONE")

    show_rrg(client, rrg)

    for v in vmap.known_map:
        if vmap.known_map[v] == voxelType.OCCUPIED:
            client.simPlotPoints([Vector3r(v[0],v[1],v[2])], color_rgba=[1,0,0,1], size=10, is_persistent=True)
        elif vmap.known_map[v] == voxelType.OCCUPIED_C_SPACE:
            client.simPlotPoints([Vector3r(v[0],v[1],v[2])], color_rgba=[1,0.05,0,1], size=10, is_persistent=True)
        elif vmap.known_map[v] == voxelType.FREE:
            client.simPlotPoints([Vector3r(v[0],v[1],v[2])], color_rgba=[0,0.5,1,0.05], size=2, is_persistent=True)

    print("DONE 2")

    while True:
        continue

            