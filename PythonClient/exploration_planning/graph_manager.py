import networkx as nx
import numpy as np
from scipy.spatial import KDTree
from utils import *
from map_manager import *


class RRG:
    def __init__(self, home_node=(0,0,0), expanding_radius=10, pathing_radius=2, max_local_nodes=10, max_local_edges=100, w_d=1, w_s=1, e_d=1):
        self.graph = nx.Graph()
        self.home_node = home_node
        self.graph.add_node(home_node)
        self.expanding_radius = expanding_radius
        self.pathing_radius = pathing_radius
        self.max_local_nodes = max_local_nodes
        self.max_local_edges = max_local_edges
        self.w_d = w_d
        self.w_s = w_s
        self.e_d = e_d

        self.frontier_nodes = []


    def sample_free_point(self, map: voxelMap, root_point):
        pt = add_3d_tuples(root_point, sample_random_point(self.expanding_radius))
        while not map.detect_point_freedom(pt):
            pt = add_3d_tuples(root_point, sample_random_point(self.expanding_radius))
        return pt


    def build_local_graph(self, map: voxelMap, root_node):
        local_graph = nx.Graph()
        local_graph.add_node(root_node)
        tree_data = [root_node]
        n_v = 0
        n_e = 0

        while n_v < self.max_local_nodes and n_e < self.max_local_edges: # TODO make function
            rand_node = quantize_coordinates(self.sample_free_point(map, root_node))
            kd_tree = KDTree(tree_data)
            _, nearest_node_idx = kd_tree.query([rand_node])
            if not map.detect_line_segment_collision(rand_node, tree_data[nearest_node_idx[0]]):
                local_graph.add_node(rand_node, VG=map.find_volumetric_gain(rand_node))
                n_v += 1

                local_graph.add_edge(rand_node, tree_data[nearest_node_idx[0]], weight=calc_dist(rand_node, tree_data[nearest_node_idx[0]]))
                n_e += 1

                nearest_nodes_witihin_delta_idx = kd_tree.query_ball_point(rand_node, self.pathing_radius) # TODO MAKE FUNCTION
                for near_node_idx in nearest_nodes_witihin_delta_idx:
                    if not map.detect_line_segment_collision(rand_node, tree_data[near_node_idx]):
                        local_graph.add_edge(rand_node, tree_data[near_node_idx], weight=calc_dist(rand_node, tree_data[near_node_idx]))
                        n_e += 1
                        if n_e >= self.max_local_edges:
                            break

                tree_data.append(rand_node)
        return local_graph
    

    def plan_local_path(self, map: voxelMap, pose_curr):
        local_graph = self.build_local_graph(map, pose_curr)
        target_nodes = list(local_graph.nodes) # TODO remove curr?

        paths = [] # TODO set size
        for node in target_nodes:
            paths.append(nx.dijkstra_path(local_graph, pose_curr, node))

        gain_best = 0
        path_best = []

        for path in paths:
            gain = RRG.compute_exploration_gain(map, local_graph, path, self.w_d, self.w_s)
            if gain > gain_best:
                gain_best = gain
                path_best = path

        # TODO improve path

        return path_best


    def compute_exploration_gain(map: voxelMap, local_graph, path, w_d, w_s):
        gain = map.find_volumetric_gain(path[0])
        cumulative_euclidean_distance = 0
        for idx in range(1, len(path)): #TODO check this 
            cumulative_euclidean_distance += calc_dist(path[idx-1], path[idx]) # how to not repeat this
            gain += local_graph.nodes[path[idx]]['VG'] * np.exp(-w_d * cumulative_euclidean_distance)
        #gain *= similarity_function TODO
        return gain
    
    def compute_global_exploration_gain(map: voxelMap, path, RET, epsilon_d): # TODO RET nd ETA
        return map.find_volumetric_gain(path[-1]) * np.exp(-epsilon_d * RRG.compute_cumulative_euclidean_distance(path))
    
    def compute_cumulative_euclidean_distance(path):
        dist = 0
        for idx in range(len(path) - 1):
            dist += calc_dist(path[idx+1], path[idx])

        return dist
        

    def plan_global_path(self, map: voxelMap, pose_curr):
        paths_from_curr = [] # TODO set size
        paths_from_home = []
        for idx, node in enumerate(list(self.graph.nodes())): # TODO add 2 graph directly without list
            paths_from_curr[idx] = nx.dijkstra_path(self.graph, pose_curr, node)
            paths_from_home[idx] = nx.dijkstra_path(self.graph, self.home_node, node)

        frontier_paths = []
        for idx, node in enumerate(self.frontier_nodes):
            frontier_paths[idx] = nx.dijkstra_path(self.graph, pose_curr, node)

        gain_best = 0
        path_best = []
        for path in frontier_paths:
            gain = RRG.compute_global_exploration_gain(map, path, 0, self.e_d) # TODO RET AND ETA
            if gain > gain_best:
                gain_best = gain
                path_best = path
        
        # TODO improve path
        return path_best
    

    def build_global_graph(self, map: voxelMap, pose_curr):
        local_path_best = self.plan_local_path(map, pose_curr)

        self.add_path_to_graph(local_path_best)

        for path in self.find_principal_paths(pose_curr): # TODO implemet clustering
            self.add_path_to_graph(path)


    def add_path_to_graph(self, path):
        tree_data = list(self.graph.nodes())
        for node in path:
            kd_tree = KDTree(tree_data) # TODO change 2 pyclustering kdtree
            _, nearest_node_idx = kd_tree.query([node])
            if not map.detect_line_segment_collision(node, tree_data[nearest_node_idx[0]]):
                self.graph.add_node(node)

                nearest_nodes_within_delta_idx = kd_tree.query_ball_point(node, self.pathing_radius)
                for near_node_idx in nearest_nodes_within_delta_idx:
                    if not map.detect_line_segment_collision(node, tree_data[near_node_idx]):
                        self.graph.add_edge(node, tree_data[near_node_idx])

                tree_data.append(node)

    def find_principal_paths(self, pose_curr):
        for node in self.frontier_nodes:
            path = nx.dijkstra_path(self.graph, pose_curr, node)
            self.add_path_to_graph(path)

