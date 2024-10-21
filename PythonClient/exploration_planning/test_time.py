import numpy as np
import time
#import math as m 

# Define the custom function to add two lists element-wise
def add_lists_elementwise(a):
    return np.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])

# Create small lists and NumPy arrays of size 3
py_list_a = (1, 2, 3)
np_array_a = np.array([1, 2, 3])

# Element-wise addition using the custom function for Python lists
start = time.time()
py_result = add_lists_elementwise(py_list_a)
print("Python list result:", py_result)
print("Python list time:", time.time() - start)

# Element-wise addition using NumPy arrays
start = time.time()
np_result = np.linalg.norm(np_array_a)
print("NumPy array result:", np_result)
print("NumPy array time:", time.time() - start)


start = time.time()
np_result = np.sqrt(np.sum(np_array_a * np_array_a))
print("NumPy array result:", np_result)
print("NumPy array time:", time.time() - start)

