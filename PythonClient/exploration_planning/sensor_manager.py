import airsim
import numpy as np
from map_manager import *

# SRATS
# 1 collect in loop process outside with pose history
# 2 collect and process in loop with pose history
# 3 collect in loop process outside with los check at end
# 4 collect and process in loop with los check at end

class SensorManager:
    def __init__(self, voxel_map: voxelMap, client: airsim.VehicleClient, vehicle: str, sensor_list: list, num_samples):
        self.voxel_map = voxel_map
        self.client = client
        self.vehicle = vehicle
        self.sensor_list = sensor_list
        self.data = []
        self.pos_history = []
        self.num_samples = num_samples

    def collect_data(self):
        for s in self.sensor_list:
            self.data.append(self.client.getLidarData(s, self.vehicle))

    def process_data(self): # TODO return pos?
        #idx = 0
        #pose_history = []
        for r in self.data:
            x, y, z = r.pose.position.x_val, r.pose.position.y_val, r.pose.position.z_val
            q0, q1, q2, q3 = r.pose.orientation.w_val, r.pose.orientation.x_val, r.pose.orientation.y_val, r.pose.orientation.z_val
            rotation_matrix = np.array(([1-2*(q2*q2+q3*q3),2*(q1*q2-q3*q0),2*(q1*q3+q2*q0)],
                                        [2*(q1*q2+q3*q0),1-2*(q1*q1+q3*q3),2*(q2*q3-q1*q0)],
                                        [2*(q1*q3-q2*q0),2*(q2*q3+q1*q0),1-2*(q1*q1+q2*q2)]))

            for i in range(0, len(r.point_cloud), 3):
                p = np.array(r.point_cloud[i:i+3])
                v = np.matmul(rotation_matrix, p)
                point_pos = (v[0] + x, v[1] + y, v[2] + z)
                self.voxel_map.add_sphere_obstacle(quantize_coordinates(point_pos))

            #if idx % len(self.sensor_list) == (len(self.sensor_list) - 1):
            #    pose_history.append((x,y,z))

            #idx += 1
        
        #for p in pose_history:
        #    self.voxel_map.add_sensor_readings_los(quantize_coordinates(p))

        #self.voxel_map.add_sensor_readings_los(quantize_coordinates((x,y,z)))
        #self.pos_history.append((x,y,z))
        keep_ordered_set(self.pos_history, quantize_coordinates((x,y,z)))

        #self.clear_data()

    def collect_and_process_data(self):
        self.collect_data()
        self.process_data()


    def process_los(self):
        for p in sample_array_uniformly(self.pos_history, self.num_samples):
            self.voxel_map.add_sensor_readings_los(p)

        self.clear_data()


    def clear_data(self):
        self.data.clear()
        self.pos_history.clear()