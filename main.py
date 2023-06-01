# import module to plot graph ...

class BasicGraph:
    def __init__(self, nodes_dict=None, adj_dict=None, directed=False):
        """Construct a graph from the dicts for geometry and topology."""

        # node positions, colors, labels, attributes ...
        self.nodes_dict = nodes_dict if nodes_dict else {}
        # adjacency info only
        self.adj_dict = adj_dict if adj_dict else {}
        # flags directed graph or not
        self.directed = directed

    def __str__(self):
        """Returns adjacency matrix of a graph, as a string."""
        mat = self.get_adj_matrix()
        mat_str = \
            '\n'.join([''.join([f'{item:4}' for item in row]) for row in mat])
        return "\n"+mat_str+"\n"

    def __iter__(self):
        """Return an iterators; delegate to iter over vertices."""
        self._iter_obj = iter(self.nodes_dict)
        return self._iter_obj

    def __next__(self):
        """Iterate over the vertices."""
        return next(self._iter_obj)

    def get_edge_weight(self, node_a, node_b):
        """Return weight of the edge from node_idA to node_idB."""
        return self.adj_dict.get(node_a).get(node_b, 0)

    def is_adjacent(self, node_a, node_b):
        """"Return T|F edge(a, b) exist in graph."""
        return node_b in self.adj_dict[node_a]

    def get_adj_matrix(self):
        """Return adjacency matrix for this graph."""
        mat = [[self.get_edge_weight(i, j) for j in self.nodes()]
               for i in self.nodes()]
        return mat

    def add_node(self, node_id, node_dict=None):
        """Add new node to graph."""
        self.nodes_dict[node_id] = node_dict if node_dict else {}
        self.adj_dict[node_id] = {}

    def nodes(self):
        """Returns a list of nodes keys."""
        ### BEGIN SOLUTION
        return list(self.adj_dict.keys())
        ### END SOLUTION

    def neighbors(self, node):
        """Return list of neighbors (keys) connected to node."""
        ### BEGIN SOLUTION
        return list(self.adj_dict[node].keys())
        ### END SOLUTION

    def all_edges(self):
        """Return a list of all unique edge tuples (from, to, weight).

        If graph is directed, return a list of all edges found in
            the adjacency dictionary.
        If graph is undirected, return all unique edge tuples:
            `(vi, vk, w)` for which `i < k`

        Observe;
            for undirected graphs, edge `vi-vk` is identical to edge `vk-vi`
            hence: `(vi, vk, w)` is identical to `(vk, vi, w)`. Include
            edge only once.
        """
        all_edges = []
        for edge_from in self.nodes():
            for edge_to in self.neighbors(edge_from):
                ### BEGIN SOLUTION
                if not self.directed and edge_to < edge_from:
                    continue
                all_edges.append(
                    (edge_from, edge_to, self.adj_dict[edge_from][edge_to]))
                ### END SOLUTION
        return all_edges

    def add_edge(self, node_a, node_b, weight=1):
        """Add edge and update adjacencies directed or undirected."""

        def _create_edge(node_a, node_b, weight):
            """Create a new one-way weighted edge."""
            node_a_connections = self.adj_dict.get(node_a)
            node_a_connections.update({node_b: weight})
            self.adj_dict.update({node_a: node_a_connections})

        ### BEGIN SOLUTION
        _create_edge(node_a, node_b, weight)
        if not self.directed:
            _create_edge(node_b, node_a, weight)
        ### END SOLUTION


