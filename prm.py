from cmath import inf
from operator import index
from platform import node
from shapely.geometry import LineString, Point

import random
from math import dist

from draw import draw_map
from models import Node, Edge, Graph
from a_star import a_star


class PRM:
    def __init__(self, n, r, start, goal, x_range, y_range, obstacels) -> None:
        self.n = n
        self.r = r
        self.start = start
        self.goal = goal
        self.x_range = x_range
        self.y_range = y_range
        self.obstacles = obstacels

    def point_is_valid(self, point):
        point_shapely = Point(point[0], point[1])
        for obs in self.obstacles:
            if obs.contains(point_shapely) or obs.intersection(point_shapely):
                return False

        return True

    def sample(self):
        xy_samples = []
        for _ in range(self.n):
            x_sampled = round(random.uniform(
                self.x_range[0], self.x_range[1]), 3)
            y_sampled = round(random.uniform(
                self.y_range[0], self.y_range[1]), 3)
            sampled_point = (x_sampled, y_sampled)
            if self.point_is_valid(sampled_point):
                xy_samples.append(sampled_point)
        return xy_samples

    def edge_is_valid(self, node1, node2, smooth_mode = False):
        if not smooth_mode and dist(node1.point, node2.point) > self.r:
            return False

        line = LineString([[node1.point[0], node1.point[1]],
                          [node2.point[0], node2.point[1]]])
        for obs in self.obstacles:
            if line.intersects(obs):
                return False

        return True

    def find_closest_node(self, selected_node, nodes):
        closest_node = None
        shortest_dist = inf
        for node in nodes:
            if node.id != selected_node.id:
                distance = dist(selected_node.point, node.point)
                if distance < shortest_dist:
                    shortest_dist = distance
                    closest_node = node
        return closest_node

    def form_graph(self, sampled_points):
        nodes = []
        id_gen = 0
        for point in sampled_points:
            nodes.append(Node(id_gen, point))
            id_gen += 1

        edges = []
        for node1 in nodes:
            for node2 in nodes:
                if node1.id != node2.id:
                    if self.edge_is_valid(node1, node2):
                        edges.append(Edge(node1, node2))

        start_node = Node(id_gen, self.start)
        goal_node = Node(id_gen + 1, self.goal)
        close_to_start = self.find_closest_node(start_node, nodes)
        close_to_goal = self.find_closest_node(goal_node, nodes)

        edges.append(Edge(start_node, close_to_start))
        edges.append(Edge(close_to_goal, goal_node))
        nodes.append(start_node)
        nodes.append(goal_node)

        graph = Graph(nodes, edges)
        return graph, start_node, goal_node

    def print_details(self):
        print("path is: ", end='')
        for node in self.path:
            print(node.point, end=", ")
        print()
        print(f"path length: {len(self.path) - 1}")
        print(f"computation time is: {self.computation_time}")

    def draw_map(self):
        draw_map(self.graph, self.start_node, self.goal_node, self.x_range, self.y_range, self.obstacles, self.path)
    
    def smooth_pathing(self):
        several_times = round(len(self.path)/2)
        for _ in range(0, several_times):
            indexes = [i for i in range(0, len(self.path))]
            i_j = random.sample(indexes, k = 2)
            i, j = i_j[0], i_j[1]
            node_i, node_j = self.path[i], self.path[j]
            if self.edge_is_valid(node_i, node_j, smooth_mode=True):
                if i < j:
                    self.path = self.path[0:i+1] + self.path[j:]
                else:
                    self.path = self.path[0:j+1] + self.path[i:]
        return self.path


    def path_planning(self):
        xy_samples = self.sample()
        self.graph, self.start_node, self.goal_node = self.form_graph(xy_samples)
        self.path, self.iterations, self.computation_time, self.found_solution = a_star(self.graph, self.start_node, self.goal_node)
        return self.path, self.iterations, self.computation_time, self.graph, self.start_node, self.goal_node, self.found_solution
