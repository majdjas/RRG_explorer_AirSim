from graph_manager import *
import matplotlib.pyplot as plt

gloabal_graph = RRG((0,0,0))
print(gloabal_graph.graph[(0,0,0)])

'''subax1 = plt.subplot()
nx.draw(gloabal_graph.graph, with_labels=True, font_weight='bold')
plt.show()'''

tree = KDTree([(0,0,0)])
print(tree.query([(0,0,1)]))
print(tree.query_ball_point([(0,0,1)], [2]))

map = voxelMap(((-9,-9,-9), (9,9,9)), known_map={}, unknown_set=set(), free_set=set(), occupied_set=set())
for x in range(-9,10):
    for y in range(-9,10):
        for z in range(-9,10):
            map.force_add_voxel((x,y,z), voxelType.FREE)

for x in range(-9,10):
    for z in range(-9,10):
        map.force_add_voxel((x,6,z), voxelType.OCCUPIED)

for x in range(-9,10):
    for y in range(-9,10):
        map.force_add_voxel((x,y,-6), voxelType.OCCUPIED)

for x in range(-9,10):
    for y in range(-9,10):
        map.force_add_voxel((x,y,6), voxelType.OCCUPIED)

for y in range(-9,10):
    for z in range(-9,20):
        map.force_add_voxel((-6,y,z), voxelType.OCCUPIED)

for y in range(-9,10):
    for z in range(-9,10):
        map.force_add_voxel((6,y,z), voxelType.OCCUPIED)

rrg = RRG(max_local_nodes=20)
loc = rrg.build_local_graph(map, (0,0,0))


pos = {node: node for node in loc.nodes()}

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Draw edges
for edge in loc.edges():
    x_vals = [pos[edge[0]][0], pos[edge[1]][0]]
    y_vals = [pos[edge[0]][1], pos[edge[1]][1]]
    z_vals = [pos[edge[0]][2], pos[edge[1]][2]]
    ax.plot(x_vals, y_vals, z_vals, color='blue')

# Draw nodes
x_nodes = [pos[node][0] for node in loc.nodes()]
y_nodes = [pos[node][1] for node in loc.nodes()]
z_nodes = [pos[node][2] for node in loc.nodes()]

ax.scatter(x_nodes, y_nodes, z_nodes, color='red', s=100)

for x,y,z in map.known_map:
    if map.known_map[(x,y,z)] == voxelType.OCCUPIED:
        ax.bar3d(x, y, z, 1, 1, 1, shade=True, color='blue', edgecolor='k')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show plot
plt.show()

subax2 = plt.subplot()
nx.draw(loc, with_labels=True, font_weight='bold')
plt.show()