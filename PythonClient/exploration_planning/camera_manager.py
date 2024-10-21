import airsim
from numpy.lib import index_tricks
from utils import *
from map_manager import *
import numpy as np
import open3d as o3d
import time

class CamManager:
    def __init__(self, intrinsic, radius, voxel_map: voxelMap, client, cameras):
        self.intrinsic = intrinsic
        self.radius = radius
        self.voxel_map = voxel_map
        self.client = client
        self.image_requests = [airsim.ImageRequest(cam, 1, True, False) for cam in cameras]


    def point_cloud(self):
        responses = self.client.simGetImages(self.image_requests)
        drone_pos = tuple(self.client.getMultirotorState().kinematics_estimated.position)

        for res in responses:
            # === Create the transformation matrix ===
            ###
            start_time = time.time()
            ###
            pos = res.camera_position
            T = np.eye(4)
            T[:3,3] = [-pos.y_val, -pos.z_val, -pos.x_val]
    
            orq = res.camera_orientation
            R = np.eye(4)
            R[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion((orq.w_val, orq.x_val, orq.y_val, orq.z_val))
    
            C = np.array([[ 1,  0,  0,  0],
                          [ 0,  0, -1,  0],
                          [ 0,  1,  0,  0],
                          [ 0,  0,  0,  1]])

            F = R.T @ T @ C

            # === Load the images ===
            depth_array = airsim.utils.get_pfm_array(res)
            depth_image = o3d.geometry.Image(depth_array)
            ###
            #o3d.visualization.draw_geometries([depth_image])
            ###
            point_cloud = o3d.geometry.PointCloud.create_from_depth_image(depth_image, self.intrinsic, extrinsic=F)
            pc_points = np.asarray(point_cloud.points)
            ###
            exectime = time.time() - start_time
            print("PCD took {}".format(exectime))
            ###


            ###
            start_time = time.time()
            ###
            for p in pc_points:
                if calc_dist(drone_pos, tuple(p)) > self.radius:
                    ###
                    #start_time = time.time()
                    ###
                    v = scale_vector(sub_3d_tuples(tuple(p),drone_pos), self.voxel_map.window_radius)
                    self.voxel_map.add_sensor_reading_short(quantize_coordinates(drone_pos), quantize_coordinates(add_3d_tuples(drone_pos, v)), True)
                    ###
                    #exectime = time.time() - start_time
                    #print("free vox took {}".format(exectime))
                    ###
                else:
                    ###
                    #start_time = time.time()
                    ###
                    self.voxel_map.add_sphere_obstacle(tuple(p))
                    ###
                    #exectime = time.time() - start_time
                    #print("occ vox took {}".format(exectime))
                    ###
            ###
            exectime = time.time() - start_time
            print("map building took {}".format(exectime))
            ###


        return drone_pos