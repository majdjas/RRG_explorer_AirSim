import networkx as nx
import numpy as np
import pyclustering.container
import pyclustering.container.kdtree
from scipy.spatial import KDTree
import pyclustering

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
from camera_manager import *
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

def test_3d():
    airsim_path = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim')

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

    T = np.eye(4)
    T[:3,3] = [0, 0, 0]
    
    R = np.eye(4)
    R[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion((1,0,0,0))
    
    C = np.array([[ 1,  0,  0,  0],
                  [ 0,  0, -1,  0],
                  [ 0,  1,  0,  0],
                  [ 0,  0,  0,  1]])

    F = R.T @ T @ C

    depth_array = np.array([[1,1],[0,0]])
    p1 = np.linalg.inv(intrinsic.intrinsic_matrix) @ np.array([0,0,1])
    p2 = F @ np.append(depth_array[0,0] * p1, 1)
    print("S")


test_3d()

G = nx.complete_graph(20)

print(G.nodes)

T = KDTree(((0,0,0) , (1,1,1), (2,1,1)))
d, i = T.query(((0,0,1),))

print(f"Distance to NN is {d}")
print(f"Index of NN is {i}")

print(T.data[i])

l = T.query_ball_point([(0.5, 0.5, 0.5), ], [1, ])

print(l)

for k in l:
    print(T.data[k])

kd_tree = pyclustering.container.kdtree.kdtree([(0,0,0)])

nearest = kd_tree.find_nearest_dist_node((0,0,11), 12)
print(nearest)
print(nearest.data)

kd_tree.insert((2,1,1))
found = kd_tree.find_node((999,999,999))
print(found)
found = kd_tree.find_node((2,1,1))
print(found)
