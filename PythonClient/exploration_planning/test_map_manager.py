from map_manager import *
import matplotlib.pyplot as plt
import numpy as np


def detect_wall():
    bounds = ((0,0,0), (20,20,20))
    pose = (10,0,0)
    sample = (1,0,0)
    wall = [(5,0,0), (5,0,1), (5,0,2), (5,0,3), (5,1,0), (5,1,1), (5,1,2), (5,1,3), (5,2,0), (5,2,1), (5,2,2), (5,2,3), (5,3,0), (5,3,1), (5,3,2), (5,3,3)] + \
           [(15,0,5), (15,0,6), (15,0,7), (15,0,8), (15,1,5), (15,1,6), (15,1,7), (15,1,8), (15,2,5), (15,2,6), (15,2,7), (15,2,8), (15,3,5), (15,3,6), (15,3,7), (15,3,8)]

    map = voxelMap(bounds, window_radius=2, known_map={}, unknown_set=set(), free_set=set(), occupied_set=set())

    for v in wall:
        map.add_sensor_reading(pose, v)

    print(map.detect_line_segment_collision(pose, sample))

    occ_array = np.full((bounds[1][0]+1, bounds[1][1]+1, bounds[1][2]+1), False)
    fre_array = np.full((bounds[1][0]+1, bounds[1][1]+1, bounds[1][2]+1), False)

    for h in map.known_map:
        if map.known_map[h] == voxelType.OCCUPIED:
            occ_array[h] = True
        elif map.known_map[h] == voxelType.FREE:
            fre_array[h] = True


    bx = plt.figure(1).add_subplot(projection='3d')
    bx.voxels(occ_array, facecolors='red', edgecolor='k')
    bx.voxels(fre_array, facecolors=[0,0,1,0.2], edgecolor='k')
    plt.show()

def detect_sparse_points():
    bounds = ((0,0,0), (20,20,20))
    pose = (10,10,10)
    sample = (1,0,0)
    wall = [(5,2,0), (5,0,10), (10,0,10), (15,0,3), (2,1,0), (0,0,0)]

    map = voxelMap(bounds, window_radius=2, known_map={}, unknown_set=set(), free_set=set(), occupied_set=set())

    for v in wall:
        map.add_sensor_reading(pose, v)

    print(map.detect_line_segment_collision(pose, sample))

    occ_array = np.full((bounds[1][0]+1, bounds[1][1]+1, bounds[1][2]+1), False)
    fre_array = np.full((bounds[1][0]+1, bounds[1][1]+1, bounds[1][2]+1), False)

    for h in map.known_map:
        if map.known_map[h] == voxelType.OCCUPIED:
            occ_array[h] = True
        elif map.known_map[h] == voxelType.FREE:
            fre_array[h] = True


    bx = plt.figure(2).add_subplot(projection='3d')
    bx.voxels(occ_array, facecolors='red', edgecolor='k')
    bx.voxels(fre_array, facecolors=[0,0,1,0.2], edgecolor='k')
    plt.show()

if __name__=='__main__':
    detect_wall()
    detect_sparse_points()


