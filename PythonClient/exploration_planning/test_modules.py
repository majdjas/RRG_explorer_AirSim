import networkx as nx
import numpy as np
from scipy.spatial import KDTree

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