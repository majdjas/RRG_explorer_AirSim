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
    r = np.random.uniform(np.sqrt(3), radius) # assert radius > sqrt3
    theta = np.random.uniform(0,2*np.pi)
    phi = np.random.uniform(0, np.pi)
    return convert_spherical_to_cartesian(r, theta, phi)

def calc_dist(a, b):
    return np.linalg.norm(sub_3d_tuples(b, a))

def convert_coords(p):
    return (p[0]/100, p[1] / 100, p[2] / 100)