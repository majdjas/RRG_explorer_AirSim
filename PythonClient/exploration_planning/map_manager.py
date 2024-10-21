from enum import Enum
import pyclustering.container.kdtree
import numpy as np
from utils import *


class voxelType(Enum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED_C_SPACE = 2
    OCCUPIED = 3

class voxelMap:

    def __init__(self, corners=((0, 0, 0), (100, 100, 100)), known_map=None, window_radius=10, brush_radius=0):
        self.set_map_bounds(corners[0], corners[1])
        self.set_radii(window_radius, brush_radius)
        self.known_map = known_map


    def set_radii(self, window_radius, brush_radius):
        self.window_radius = window_radius
        self.brush_radius = brush_radius
        self.preprocess_sphere_sets()


    def set_map_bounds(self, corner_a, corner_b):
        left_down_far = (min((corner_a[0], corner_b[0])), min((corner_a[1], corner_b[1])), min((corner_a[2], corner_b[2])))
        right_up_near = (max((corner_a[0], corner_b[0])), max((corner_a[1], corner_b[1])), max((corner_a[2], corner_b[2])))
        self.map_bounds = (left_down_far, right_up_near)


    def is_in_bounds(self, point):
        return (compare_coordinates(point, self.map_bounds[0]) and compare_coordinates(self.map_bounds[1], point)) ## TODO change fn name and account for voxel size


    def add_voxel(self, point, voxel_type):# how to use bool here TODO separte add voxel and point
        if self.is_in_bounds(point):
            voxel = quantize_coordinates(point)
            if self.get_voxel_type(voxel) == voxelType.OCCUPIED:
                return True
            elif self.get_voxel_type(voxel) == voxelType.OCCUPIED_C_SPACE:
                if voxel_type == voxelType.OCCUPIED:
                    self.known_map[voxel] = voxel_type
            elif self.get_voxel_type(voxel) == voxelType.FREE:
                if voxel_type == voxelType.OCCUPIED or voxel_type == voxelType.OCCUPIED_C_SPACE:
                    self.known_map[voxel] = voxel_type
            else:
                self.known_map[voxel] = voxel_type
            return True

        else:
            return False


    def add_sphere_obstacle(self, center_point):
        self.add_voxel(center_point, voxelType.OCCUPIED)
        for v in self.brush_set:
            self.add_voxel(add_3d_tuples(center_point, v), voxelType.OCCUPIED_C_SPACE)

    
    def force_add_voxel(self, point, voxel_type):
        voxel = quantize_coordinates(point)
        self.known_map[voxel] = voxel_type


    def get_voxel_type(self, voxel):
        if voxel in self.known_map:
            return self.known_map[voxel]
        else:
            return voxelType.UNKNOWN
        

    def is_known(self, voxel):
        if voxel in self.known_map:
            return True
        else:
            return False


    def check_voxel_collision(self, voxel):
        if self.get_voxel_type(voxel) == voxelType.OCCUPIED or self.get_voxel_type(voxel) == voxelType.OCCUPIED_C_SPACE:
            return True
        else:
            return False


    def check_point_collision(self, point):
        voxel = quantize_coordinates(point)
        return self.check_voxel_collision(voxel)


    def check_voxel_freedom(self, voxel):
        if self.get_voxel_type(voxel) == voxelType.FREE:
            return True
        else:
            return False


    def check_point_freedom(self, point):
        voxel = quantize_coordinates(point)
        return self.check_voxel_freedom(voxel)


    def add_sensor_reading(self, start_voxel, end_voxel, is_last_free: bool): 
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
        if is_last_free:
            self.add_voxel(end_voxel, voxelType.FREE)
        else:
            self.add_sphere_obstacle(end_voxel)


    def add_sensor_reading_short(self, start_voxel, end_voxel, is_last_free: bool): 
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
            if self.check_voxel_collision(tuple(voxel_curr)):
                return
            self.add_voxel(tuple(voxel_curr), voxelType.FREE)
            for n in range(2):
                if D[n]*S[2] > 0:
                    C[n] += S[n]
                    D[n] -= 2*v[i]
                D[n] += 2*v[(i+n+1)%3]*S[n]*S[2]
        if is_last_free:
            self.add_voxel(end_voxel, voxelType.FREE)
        else:
            self.add_sphere_obstacle(end_voxel)
        

    def add_sensor_readings_los(self, center_voxel):
        for v in self.hollow_set:
            self.add_sensor_reading_short(center_voxel, add_3d_tuples(center_voxel, v), True)


    def add_sensor_readings_los_with_callback(self, center_voxel, callback, args: tuple):
        for v in self.hollow_set:
            if callback(v, args):
                self.add_sensor_reading(center_voxel, add_3d_tuples(center_voxel, v), True)


    def detect_line_segment_collision(self, start_voxel, end_voxel):
        if start_voxel == end_voxel: # change
            return True
        v = np.array((end_voxel[0] - start_voxel[0], end_voxel[1] - start_voxel[1], end_voxel[2] - start_voxel[2]))
        indices = np.argsort(np.abs(v))

        S = (np.sign(v[indices[0]]), np.sign(v[indices[1]]), np.sign(v[indices[2]]))
        D = [2*v[indices[0]]*S[0]*S[2] - v[indices[2]], 2*v[indices[1]]*S[1]*S[2] - v[indices[2]]]
        C = [start_voxel[indices[0]], start_voxel[indices[1]]]
        
        voxel_curr = list(start_voxel)
        has_collided = False
        for t in range(start_voxel[indices[2]], end_voxel[indices[2]] + S[2], S[2]):
            voxel_curr[indices[0]] = C[0]
            voxel_curr[indices[1]] = C[1]
            voxel_curr[indices[2]] = t
            has_collided = self.check_voxel_collision(tuple(voxel_curr))
            for n in [1,0]:
                if D[n]*S[2] > 0:
                    C[n] += S[n]
                    voxel_curr[indices[n]] = C[n]
                    has_collided = self.check_voxel_collision(tuple(voxel_curr))
                    D[n] -= 2*v[indices[2]]
                D[n] += 2*v[indices[n]]*S[n]*S[2]
            if has_collided:
                return True
        return has_collided #LAST voxel is occupied?

    
    def preprocess_sphere_sets(self):
        self.window_set = []
        for x in range(-self.window_radius, self.window_radius+1):
            for y in range(-self.window_radius, self.window_radius+1):
                for  z in range(-self.window_radius, self.window_radius+1):
                    if np.linalg.norm(np.array((x,y,z))) <= self.window_radius:
                        self.window_set.append((x,y,z))

        self.hollow_set = [] # TODO bresenhams circle
        for v in self.window_set:
            if np.linalg.norm(v) > self.window_radius - 1:#np.sqrt(3) - 0.0001:
                self.hollow_set.append(v)
        print(len(self.hollow_set))
        print(len(self.window_set))

        self.brush_set = []
        for x in range(-self.brush_radius, self.brush_radius+1):
            for y in range(-self.brush_radius, self.brush_radius+1):
                for  z in range(-self.brush_radius, self.brush_radius+1):
                    if np.linalg.norm((x,y,z)) <= self.brush_radius:
                        if (x,y,z) != (0,0,0):
                            self.brush_set.append((x,y,z))

    
    def find_volumetric_gain(self, center_point): # TODO kd tree scan sphere subtract known voxs maybe?
        center_voxel = quantize_coordinates(center_point)
        gain = 0
        for v in self.window_set:
            if add_3d_tuples(center_voxel, v) not in self.known_map:
                gain += 1

        return gain


    def find_volumetric_gain_los(self, center_point):
        start_voxel = quantize_coordinates(center_point)
        gain = 0
        unknown_set = set()
        for voxel in self.hollow_set:
            ###
            if voxel == (0,0,0):
                continue
            ###
            end_voxel = add_3d_tuples(start_voxel, voxel) # assert end != start, and change bresenhams, and check 1st voxel
            v = np.array((end_voxel[0] - start_voxel[0], end_voxel[1] - start_voxel[1], end_voxel[2] - start_voxel[2]))
            indices = np.argsort(np.abs(v))

            S = (np.sign(v[indices[0]]), np.sign(v[indices[1]]), np.sign(v[indices[2]]))
            D = [2*v[indices[0]]*S[0]*S[2] - v[indices[2]], 2*v[indices[1]]*S[1]*S[2] - v[indices[2]]]
            C = [start_voxel[indices[0]], start_voxel[indices[1]]]
            
            voxel_curr = list(start_voxel)
            has_collided = False
            for t in range(start_voxel[indices[2]], end_voxel[indices[2]] + S[2], S[2]):
                if has_collided: # have to double break instaed of return
                    break
                voxel_curr[indices[0]] = C[0]
                voxel_curr[indices[1]] = C[1]
                voxel_curr[indices[2]] = t
                has_collided = self.check_voxel_collision(tuple(voxel_curr))
                if has_collided:
                    break
                elif (not self.is_known(tuple(voxel_curr))) and (tuple(sub_3d_tuples(tuple(voxel_curr), start_voxel)) in self.window_set):
                    if tuple(voxel_curr) not in unknown_set:
                        unknown_set.add(tuple(voxel_curr))
                        gain += 1
                for n in [1,0]:
                    if D[n]*S[2] > 0:
                        C[n] += S[n]
                        voxel_curr[indices[n]] = C[n]
                        has_collided = self.check_voxel_collision(tuple(voxel))
                        if has_collided:
                            break
                        elif (not self.is_known(tuple(voxel_curr))) and (tuple(sub_3d_tuples(tuple(voxel_curr), start_voxel)) in self.window_set):
                            if tuple(voxel_curr) not in unknown_set:
                                unknown_set.add(tuple(voxel_curr))
                                gain += 1
                        D[n] -= 2*v[indices[2]]
                    D[n] += 2*v[indices[n]]*S[n]*S[2]
            if (not has_collided) and (not self.is_known(end_voxel)) and (sub_3d_tuples(end_voxel, start_voxel) in self.window_set):# if broken and possible bresenhams goes out of window set
                if tuple(end_voxel) not in unknown_set:
                    unknown_set.add(tuple(end_voxel))
                    gain += 1
                        
        return gain #,unknown_set

        '''start_voxel = quantize_coordinates(center_point)
        gain = 0
        unknown_tree = pyclustering.container.kdtree.kdtree()
        for voxel in self.window_set:
            ###
            if voxel == (0,0,0):
                continue
            ###
            has_broken = False
            end_voxel = add_3d_tuples(start_voxel, voxel) # assert end != start, and change bresenhams, and check 1st voxel
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
                if self.check_voxel_collision(tuple(voxel_curr)):
                    has_broken = True
                    break
                elif not self.is_known(tuple(voxel_curr)):
                    if unknown_tree.find_node(tuple(voxel_curr)) == None:
                        unknown_tree.insert(tuple(voxel_curr))
                        gain += 1
                for n in range(2):
                    if D[n]*S[2] > 0:
                        C[n] += S[n]
                        D[n] -= 2*v[i]
                    D[n] += 2*v[(i+n+1)%3]*S[n]*S[2]
            if not has_broken and not self.is_known(end_voxel):# if broken
                if unknown_tree.find_node(end_voxel) == None:
                    unknown_tree.insert(end_voxel)
                    gain += 1

        return gain'''

    #def scan_map(self, callback, args):
    #    for x in range(self.map_bounds[0][0], self.map_bounds[1][0] + 1):
    #        for y in range(self.map_bounds[0][1], self.map_bounds[1][1] + 1):
    #            for z in range(self.map_bounds[0][2], self.map_bounds[1][2] + 1):
    #                callback((x,y,z), args)

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



# TODO heirachy unk free occ when doing los using kdtree scan