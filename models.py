from math import inf


class Graph:
    def __init__(self, nodes, edges, weight = None):
        self.nodes = nodes
        self.edges = edges
        self.weight = weight


class Node:
    def __init__(self, id, point):
        self.id = id
        self.point = point
        self.parent = None
        self.adjacent_nodes = list()
        self.edges = list()
        self.seen = False

    def add_neighbour(self, neighbour, edge):
        self.adjacent_nodes.append(neighbour)
        self.edges.append(edge)


class Edge:
    def __init__(self, node_a, node_b):
        self.node_a = node_a
        self.node_b = node_b
        self.node_a.add_neighbour(node_b, self)
