import numpy as np

def quantize_coordinates(coordinates):
    return (int(np.round(coordinates[0])), int(np.round(coordinates[1])), int(np.round(coordinates[2])))

def compare_coordinates(point_a, point_b):
    for index in range(len(point_a)):
        if point_a[index] < point_b[index]:
            return False
    return True

def add_3d_tuples(a, b):
    return((a[0] + b[0], a[1] + b[1], a[2] + b[2]))

def sub_3d_tuples(a, b):
    return((a[0] - b[0], a[1] - b[1], a[2] - b[2]))

def convert_spherical_to_cartesian(r, theta, phi):
    x = r * np.sin(phi) * np.cos(theta)
    y = r * np.sin(phi) * np.sin(theta)
    z = r * np.cos(phi)
    return (x,y,z)

def sample_random_point(radius):
    r = np.random.uniform(np.sqrt(3) + 0.0001, radius) # assert radius > sqrt3
    theta = np.random.uniform(0,2*np.pi)
    phi = np.random.uniform(0, np.pi)
    return convert_spherical_to_cartesian(r, theta, phi)

def calc_dist(a, b):
    return np.linalg.norm(sub_3d_tuples(b, a))

def convert_coords(p):
    return (p[0]/100, p[1] / 100, p[2] / 100)

def scale_vector(v, length):
    norm = np.linalg.norm(v)
    if norm < 0.0001:
        return (0,0,0)
    else:
        return (v[0] * length / norm, v[1] * length / norm, v[2] * length / norm)


def interpolate_path(ref, d):
    new_path = []
    length = len(ref)
    if length == 1:
        return [ref[0]]
    else:
        for i in range(0, len(ref) - 1):
            v = scale_vector(sub_3d_tuples(ref[i+1], ref[i]), d)
            n = int(np.floor(np.linalg.norm(v) / d))
            for j in range(n+1):
                new_path.append(add_3d_tuples(ref[i], v))

    return new_path

def compute_straight_path(p, v, n, d):
    path = []
    for i in range(n):
        path.append(add_3d_tuples(p, scale_vector(v, i*d)))
    return path


def sample_array_uniformly(arr, n):
    if n >= len(arr):
        return arr
    
    srr = []
    step = (len(arr) - 1) / (n-1) # assert n > 1

    for i in range(n):
        srr.append(arr[int(np.round(step * i))])

    return srr

def keep_ordered_set(l: list, a):
    if a not in l:
        l.append(a)
