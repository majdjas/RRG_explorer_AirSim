from map_manager import *
import matplotlib.pyplot as plt
import numpy as np
import time


def detect_wall():
    bounds = ((0,0,0), (20,20,20))
    pose = (10,0,0)
    sample = (1,0,0)
    wall = [(5,0,0), (5,0,1), (5,0,2), (5,0,3), (5,1,0), (5,1,1), (5,1,2), (5,1,3), (5,2,0), (5,2,1), (5,2,2), (5,2,3), (5,3,0), (5,3,1), (5,3,2), (5,3,3)] + \
           [(15,0,5), (15,0,6), (15,0,7), (15,0,8), (15,1,5), (15,1,6), (15,1,7), (15,1,8), (15,2,5), (15,2,6), (15,2,7), (15,2,8), (15,3,5), (15,3,6), (15,3,7), (15,3,8)]

    map = voxelMap(bounds, {})

    for v in wall:
        map.add_sensor_reading(pose, v, False)

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

def hollow_sphere():
    bounds = ((-20,-20,-20), (20,20,20))
    pose = (0,0,0)

    map = voxelMap(bounds, window_radius=5, known_map={}, unknown_set=set(), free_set=set(), occupied_set=set())

    hollow_set = map.window_set.copy()
    for v in map.window_set:
        if calc_dist(v, pose) < map.window_radius - 1:
            hollow_set.remove(v)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for x,y,z in hollow_set:
        if x >= 0:
            ax.bar3d(x, y, z, 1, 1, 1, shade=True, color='blue', edgecolor='k')
    plt.show()


def bres():
    vmap = voxelMap(known_map={})
    pose = (50,50,50)

    reads = [(20,10,10),(-20,10,10),(20,-10,10),(-20,-10,10),(20,10,-10),(-20,10,-10),(20,-10,-10),(-20,-10,-10)]

    for r in reads:
        vmap.add_sensor_reading(pose, add_3d_tuples(pose,r), True)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for v in vmap.known_map:
        ax.bar3d(v[0], v[1], v[2], 1, 1, 1, shade=True, color=[0,0,1,0.1], edgecolor='k')
    plt.show()

    vmap = voxelMap(known_map={})
    pose = (50,50,50)

    reads = [(10,20,10),(-10,20,10),(10,-20,10),(-10,-20,10),(10,20,-10),(-10,20,-10),(10,-20,-10),(-10,-20,-10)]

    for r in reads:
        vmap.add_sensor_reading(pose, add_3d_tuples(pose,r), True)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for v in vmap.known_map:
        ax.bar3d(v[0], v[1], v[2], 1, 1, 1, shade=True, color=[0,0,1,0.1], edgecolor='k')
    plt.show()

    vmap = voxelMap(known_map={})
    pose = (50,50,50)

    reads = [(10,10,20),(-10,10,20),(10,-10,20),(-10,-10,20),(10,10,-20),(-10,10,-20),(10,-10,-20),(-10,-10,-20)]

    for r in reads:
        vmap.add_sensor_reading(pose, add_3d_tuples(pose,r), True)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for v in vmap.known_map:
        ax.bar3d(v[0], v[1], v[2], 1, 1, 1, shade=True, color=[0,0,1,0.1], edgecolor='k')
    plt.show()


def vol_gain():
    pose = (50,50,50)
    vmap = voxelMap(known_map={(add_3d_tuples((0,0,1),pose)):voxelType.OCCUPIED}, window_radius=5)
    #vmap = voxelMap(known_map={},window_radius=5)


    print("Size of window set: {}".format(len(vmap.window_set)))
    start_time = time.time()
    VG1 = vmap.find_volumetric_gain(pose)
    exectime = time.time() - start_time
    print("Occupancy vol. gain: {} | {}".format(VG1, exectime))
    
    start_time = time.time()
    VG2 = vmap.find_volumetric_gain_los(pose)
    exectime = time.time() - start_time
    print("LOS vol. gain: {} | {}".format(VG2, exectime))

    '''fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for x,y,z in vmap.window_set:
        ax.bar3d(x + pose[0], y + pose[1], z + pose[2], 1, 1, 1, shade=True, color='blue', edgecolor='k')
    plt.show()

    fig = plt.figure()
    hx = fig.add_subplot(111, projection='3d')
    for x,y,z in vmap.hollow_set:
        hx.bar3d(x + pose[0], y + pose[1], z + pose[2], 1, 1, 1, shade=True, color=[0,0,1,0.2], edgecolor='k')
    plt.show()'''

    vmap.add_sensor_readings_los(pose)

    #rem = [(0,-5,0),(-2,-4,2),(-1,-4,2),(0,-4,2),(1,-4,2),(2,-4,2)]
    #for r in rem:
    #    del vmap.known_map[add_3d_tuples(r,pose)]
    #vmap.add_voxel(add_3d_tuples((-1,-3,2),pose), voxelType.OCCUPIED)

    fig = plt.figure()
    bx = fig.add_subplot(111, projection='3d')
    for v in vmap.known_map:
        if vmap.get_voxel_type(v) == voxelType.OCCUPIED:
            bx.bar3d(v[0], v[1], v[2], 1, 1, 1, shade=True, color=[1,0.5,0,1], edgecolor='k')
        else:
            bx.bar3d(v[0], v[1], v[2], 1, 1, 1, shade=True, color=[0,0.5,1,1], edgecolor='k')
    plt.show()


def collision_detection():
    vmap = voxelMap(corners=((-100,-100,-100),(100,100,100)),known_map={}, window_radius=5)
    vmap.add_sensor_reading((0,0,0),(0,0,1), False)

    reads = [(10,0,0), (-10,0,0), (0,10,0), (0,-10,0), (0,0,10),(0,0,-10)]
    for r in reads:
        print("{}: {}".format(r,vmap.detect_line_segment_collision((0,0,0), r)))

    print(len(vmap.window_set))

    g = 0
    for v in vmap.window_set:
        if not vmap.detect_line_segment_collision((0,0,0),v):
            g += 1
    print(g)

    g = 0
    for v in vmap.hollow_set:
        if not vmap.detect_line_segment_collision((0,0,0),v):
            g += 1
    print(g)

    q,p = vmap.find_volumetric_gain_los((0,0,0))
    print(q)
    
    fig = plt.figure()
    bx = fig.add_subplot(111, projection='3d')
    for v in p:
        bx.bar3d(v[0], v[1], v[2], 1, 1, 1, shade=True, color=[0,0.5,1,1], edgecolor='k')
    plt.show()


if __name__=='__main__':
    #detect_wall()
    #detect_sparse_points()
    #hollow_sphere()
    #bres()
    vol_gain()
    collision_detection()

    


