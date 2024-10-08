from enum import Enum
import numpy as np
from utils import *

class voxelType(Enum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2

class voxelMap:

    def __init__(self, corners=((0, 0, 0), (100, 100, 100)), unknown_set=None, free_set=None, occupied_set=None, known_map=None, window_radius=10):
        self.set_map_bounds(corners[0], corners[1])
        self.unkown_set = unknown_set
        self.free_set = free_set
        self.occupied_set = occupied_set
        self.known_map = known_map
        self.window_radius = window_radius
        self.preprocess_sphere_set(window_radius)

    def verify_set(self, type):
        if type == voxelType.UNKNOWN:
            voxel_set = self.unkown_set
        elif type == voxelType.FREE:
            voxel_set = self.free_set
        elif type == voxelType.OCCUPIED:
            voxel_set = self.occupied_set
        for voxel in voxel_set:
            if self.get_voxel_type != type:
                voxel_set.remove(voxel)

    def set_map_bounds(self, corner_a, corner_b):
        left_down_far = (min((corner_a[0], corner_b[0])), min((corner_a[1], corner_b[1])), min((corner_a[2], corner_b[2])))
        right_up_near = (max((corner_a[0], corner_b[0])), max((corner_a[1], corner_b[1])), max((corner_a[2], corner_b[2])))
        self.map_bounds = (left_down_far, right_up_near)

    def check_if_witihin_bounds(self, coords):
        return (compare_coordinates(coords, self.map_bounds[0]) and compare_coordinates(self.map_bounds[1], coords)) ## account for voxel size

    def add_voxel(self, coords, type):
        if self.check_if_witihin_bounds(coords):
            voxel_coords = quantize_coordinates(coords)
            if self.get_voxel_type(voxel_coords) == voxelType.OCCUPIED:
                return False
            
            self.known_map[voxel_coords] = type
            if type == voxelType.FREE:
                self.free_set.add(voxel_coords)
            elif type == voxelType.OCCUPIED:
                self.occupied_set.add(voxel_coords)
            return True
        return False
    
    def force_add_voxel(self, coords, type):
        voxel_coords = quantize_coordinates(coords)
        self.known_map[voxel_coords] = type
        if type == voxelType.FREE:
            self.free_set.add(voxel_coords)
        elif type == voxelType.OCCUPIED:
            self.occupied_set.add(voxel_coords)
        elif type == voxelType.UNKNOWN:
            self.unkown_set.add(voxel_coords)

    #def scan_map(self, callback, args):
    #    for x in range(self.map_bounds[0][0], self.map_bounds[1][0] + 1):
    #        for y in range(self.map_bounds[0][1], self.map_bounds[1][1] + 1):
    #            for z in range(self.map_bounds[0][2], self.map_bounds[1][2] + 1):
    #                callback((x,y,z), args)


    def get_voxel_type(self, voxel_coords):
        if voxel_coords in self.known_map:
            return self.known_map[voxel_coords]
        else:
            return voxelType.UNKNOWN
        
    def check_if_voxel_is_known(self, voxel_coords):
        if voxel_coords in self.known_map:
            return True
        else:
            return False

    def detect_voxel_collision(self, voxel):
        if (voxel in self.known_map and self.known_map[voxel] == voxelType.OCCUPIED):
            return True
        else:
            return False

    def detect_point_collision(self, point):
        quantized_point = quantize_coordinates(point)
        return self.detect_voxel_collision(quantized_point)

    def detect_voxel_freedom(self, voxel):
        if (voxel in self.known_map and self.known_map[voxel] == voxelType.FREE):
            return True
        else:
            return False

    def detect_point_freedom(self, point):
        quantized_point = quantize_coordinates(point)
        return self.detect_voxel_freedom(quantized_point)

    def add_sensor_reading(self, start_voxel, end_voxel): 
        if start_voxel == end_voxel:
            return
        v = np.array((end_voxel[0] - start_voxel[0], end_voxel[1] - start_voxel[1], end_voxel[2] - start_voxel[2]))

        i = np.argmax(np.abs(v)) # assert >=1
        j = (i+1) % 3
        k = (i+2) % 3

        S = (np.sign(v[j]), np.sign(v[k]), np.sign(v[i]))
        D = [2*v[j]*S[0]*S[2] - v[i], 2*v[k]*S[1]*S[2] - v[i]]
        C = [start_voxel[j], start_voxel[k]]

        voxel_curr = list(start_voxel)
        for t in range(start_voxel[i], end_voxel[i], np.sign(v[i])):
            voxel_curr[k] = C[1]
            voxel_curr[j] = C[0]
            voxel_curr[i] = t
            self.add_voxel(tuple(voxel_curr), voxelType.FREE)
            for n in range(2):
                if D[n]*S[2] > 0:
                    C[n] += S[n]
                    D[n] -= 2*v[i]
                D[n] += 2*v[(i+n+1)%3]*S[n]*S[2]
        self.add_voxel(end_voxel, voxelType.OCCUPIED)

    def detect_line_segment_collision(self, start_voxel, end_voxel):
        if start_voxel == end_voxel: # change
            return True
        v = np.array((end_voxel[0] - start_voxel[0], end_voxel[1] - start_voxel[1], end_voxel[2] - start_voxel[2]))
        indices = np.argsort(np.abs(v))

        S = (np.sign(v[indices[1]]), np.sign(v[indices[0]]), np.sign(v[indices[2]]))
        D = [2*v[indices[1]] - v[indices[2]], 2*v[indices[0]] - v[indices[2]]]
        C = [start_voxel[indices[1]], start_voxel[indices[0]]]
        
        voxel_curr = list(start_voxel)
        has_collided = False
        for t in range(start_voxel[indices[2]], end_voxel[indices[2]] + S[2], S[2]):
            voxel_curr[indices[0]] = C[1]
            voxel_curr[indices[1]] = C[0]
            voxel_curr[indices[2]] = t
            has_collided = self.detect_voxel_collision(tuple(voxel_curr))
            for n in range(2):
                if D[n]*S[2] > 0:
                    C[n] += S[n]
                    voxel_curr[indices[1-n]] = C[n]
                    has_collided = self.detect_voxel_collision(tuple(voxel_curr))
                    D[n] -= 2*v[indices[1-n]]
                D[n] += 2*v[indices[2]]
            if has_collided:
                return True
        return has_collided



    """ def detect_line_segment_collision(self, point_a, point_b):
        voxel_a = quantize_coordinates(point_a)
        voxel_b = quantize_coordinates(point_b)
        is_colliding = False
        colliding_voxel_set = set()

        vector = np.array((voxel_b[0] - voxel_a[0], voxel_b[1] - voxel_a[1], voxel_b[2] - voxel_a[2]))
        vector = vector / np.max(vector)

        independent_axis = np.argmax(vector)
        step = int(vector[independent_axis])

        prev = [voxel_a[(independent_axis + 1) % 3], voxel_a[(independent_axis + 2) % 3]]
        for i in range(abs(voxel_b[independent_axis] - voxel_a[independent_axis])):
            pt = quantize_coordinates(np.array(voxel_a) + (i+1) * step * vector)
            colliding_voxel_set.add(pt)
            for k in range(2):
                if pt[(independent_axis + k+1) % 3] != prev[k]:
                    new_pt = list(pt)
                    new_pt[independent_axis] -= step
                    colliding_voxel_set.add(new_pt)
                    prev[k] = pt[(independent_axis + k+1) % 3]


        return (is_colliding, colliding_voxel_set)"""

    '''def detect_sphere_submap(self, center, radius):
        center_voxel = quantize_coordinates(center)
        submap = voxelMap(((center_voxel[0] - radius, center_voxel[1] - radius, center_voxel[2] - radius), (center_voxel[0] + radius, center_voxel[1] + radius, center_voxel[2] + radius)),
                          known_map=self.known_map)

        for x in range(submap.map_bounds[0][0], submap.map_bounds[1][0] + 1):
            for y in range(submap.map_bounds[0][1], submap.map_bounds[1][1] + 1):
                for z in range(submap.map_bounds[0][2], submap.map_bounds[1][2] + 1):
                    curr_voxel = (x,y,z)
                    if np.abs(np.array(curr_voxel) - np.array(center)) < radius:
                        submap.add_voxel(curr_voxel, self.known_map[curr_voxel])

        return submap'''
    
    '''def compute_volumetric_gain(self):
        return len(self.unkown_set)'''
    
    def preprocess_sphere_set(self, radius):
        self.window_center = (0,0,0)
        self.window_set = set()
        for x in range(-radius, radius+1):
            for y in range(-radius, radius+1):
                for  z in range(-radius, radius+1):
                    if np.linalg.norm(np.array((x,y,z))) < radius:
                        self.window_set.add((x,y,z))

    def slide_window(self, center_point):
        center_voxel = quantize_coordinates(center_point)
        displacement = sub_3d_tuples(center_voxel, self.window_center)
        self.window_center = center_voxel
        for voxel in self.window_set.copy():
            self.window_set.remove(voxel)
            self.window_set.add(add_3d_tuples(voxel, displacement))

    def compute_volumetric_gain(self):
        volumetric_gain = 0
        for voxel in self.window_set:
            if voxel not in self.known_map or self.known_map[voxel] == voxelType.UNKNOWN:
                volumetric_gain += 1
        return volumetric_gain
    
    def find_volumetric_gain(self, center_point):
        self.slide_window(center_point)
        return self.compute_volumetric_gain()







#def scan_sphere_callback(voxel_coords, args):
#    center
#    radius = args[1]





#    for voxel in voxel_map:
#        if voxel_map[voxel] == type:
#            voxel_set.add(voxel)