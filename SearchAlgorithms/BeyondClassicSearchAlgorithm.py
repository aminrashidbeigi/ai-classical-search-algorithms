import sys


class BeyondClassicSearchAlgorithm(object):
    def __init__(self, problem):
        self.problem = problem
        self.parent = {}
        self.memory = 0

    def a_star(self, start_state):
        def find_node_with_minimum_cost_to_expand(nodes):
            min_cost = sys.maxsize
            min_node = ()
            for node in nodes:
                if min_cost > node[1]:
                    min_cost = node[1]
                    min_node = node
            return min_node

        path = []
        path_cost = 0
        visited_nodes = []
        nodes_to_expand = [(start_state, path_cost)]

        while nodes_to_expand:
            current_state = find_node_with_minimum_cost_to_expand(nodes_to_expand)
            path.append(list(current_state[0]))
            nodes_to_expand.pop(nodes_to_expand.index(current_state))
            path_cost = current_state[1]
            if self.problem.isGoalTest(current_state[0]):
                print("Algorithm: Graph Uniform Cost Search")
                print("Cost: " + str(path_cost))
                print("Memory: " + str(self.memory))
                print("Last State: " + str(current_state))
                self.print_path(current_state[0])
                return path
            visited_nodes.append(current_state[0])
            states = self.problem.results(self.problem.actions(current_state[0]), current_state[0])
            for state in states:
                if state not in visited_nodes:
                    nodes_to_expand.append((state, path_cost + self.problem.step_cost(current_state[0], state) + self.problem.heuristic(state)))
                    self.parent[state] = current_state[0]
                    self.memory = self.memory + 1

    def print_path(self, leaf_node):
        path = []
        while leaf_node:
            path.append(leaf_node)
            if leaf_node in self.parent:
                leaf_node = self.parent[leaf_node]
            else:
                leaf_node = None
        self.problem.print_path(list(reversed(path)))

