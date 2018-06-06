import sys


class ClassicSearchAlgorithm(object):
    def __init__(self, problem):
        self.problem = problem
        self.parent = {}
        self.memory = 0

    def graph_depth_first_search(self, start_state):
        path = []
        visited_nodes = []
        nodes_to_expand = [start_state]

        number_of_visited_nodes = 1
        number_of_expanded_nodes = 0

        while nodes_to_expand:
            current_state = nodes_to_expand.pop()
            path.append(current_state)
            number_of_expanded_nodes = number_of_expanded_nodes + 1
            if self.problem.isGoalTest(current_state):
                print("Algorithm: Graph DFS")
                print("Number Of Visited Nodes: " + str(number_of_visited_nodes))
                print("Number Of Expanded Nodes: " + str(number_of_expanded_nodes))
                print("Memory: " + str(self.memory))
                print("Last State: " + str(current_state))
                self.print_path(current_state)
                return path
            visited_nodes.append(current_state)
            states = self.problem.results(self.problem.actions(current_state), current_state)
            for state in states:
                if state not in visited_nodes:
                    number_of_visited_nodes = number_of_visited_nodes + 1
                    nodes_to_expand.append(state)
                    self.parent[state] = current_state
                    self.memory = self.memory + 1

    def graph_breadth_first_search(self, start_state):
        path = []
        visited_nodes = []
        nodes_to_expand = [start_state]
        number_of_visited_nodes = 1
        number_of_expanded_nodes = 0

        while nodes_to_expand:
            current_state = nodes_to_expand.pop(0)
            path.append(current_state)
            number_of_expanded_nodes = number_of_expanded_nodes + 1
            if self.problem.isGoalTest(current_state):
                print("Algorithm: Graph BFS")
                print("Number Of Visited Nodes: " + str(number_of_visited_nodes))
                print("Number Of Expanded Nodes: " + str(number_of_expanded_nodes))
                print("Memory: " + str(self.memory))
                print("Last State: " + str(current_state))
                self.print_path(current_state)
                return path
            visited_nodes.append(current_state)
            states = self.problem.results(self.problem.actions(current_state), current_state)
            for state in states:
                if state not in visited_nodes:
                    number_of_visited_nodes = number_of_visited_nodes + 1
                    nodes_to_expand.append(state)
                    self.parent[state] = current_state
                    self.memory = self.memory + 1


    def graph_bidirectional_search(self, start_state, goal_state):
        path = []
        path_from_start = []
        path_from_goal = []
        visited_nodes = []
        nodes_to_expand_from_start = [start_state]
        nodes_to_expand_from_goal = [goal_state]

        while nodes_to_expand_from_goal or nodes_to_expand_from_start:
            current_state_from_start = nodes_to_expand_from_start.pop(0)
            path_from_start.append(current_state_from_start)
            current_state_from_goal = nodes_to_expand_from_goal.pop(0)
            path_from_goal.append(current_state_from_goal)
            if current_state_from_goal == current_state_from_start:
                print("Algorithm: Graph bidirectional")
                print("Memory: " + str(self.memory))
                for p1 in path_from_start:
                    path.append(p1)
                for p2 in reversed(path_from_goal):
                    path.append(p2)
                return path
            visited_nodes.append(current_state_from_goal)
            visited_nodes.append(current_state_from_start)
            states_from_start = self.problem.results(self.problem.actions(current_state_from_start), current_state_from_start)
            states_from_goal = self.problem.results(self.problem.actions(current_state_from_goal), current_state_from_goal)
            for state_from_start in states_from_start:
                nodes_to_expand_from_start.append(state_from_start)
                self.memory = self.memory + 1
            for state_from_goal in states_from_goal:
                nodes_to_expand_from_goal.append(state_from_goal)
                self.memory = self.memory + 1

    def graph_uniform_cost_search(self, start_state):
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
                    nodes_to_expand.append((state, path_cost + self.problem.step_cost(current_state[0], state)))
                    self.parent[state] = current_state[0]
                    self.memory = self.memory + 1

    def graph_depth_limited_search(self, start_state, depth):
        path = []
        current_depth = 0
        visited_nodes = []
        nodes_to_expand = [(start_state, 0)]

        number_of_visited_nodes = 1
        number_of_expanded_nodes = 0

        while nodes_to_expand and current_depth <= depth:
            current_state = nodes_to_expand.pop()
            path.append(current_state)
            number_of_expanded_nodes = number_of_expanded_nodes + 1
            if self.problem.isGoalTest(current_state):
                print("Algorithm: Graph DFS")
                print("Number Of Visited Nodes: " + str(number_of_visited_nodes))
                print("Number Of Expanded Nodes: " + str(number_of_expanded_nodes))
                print("Memory: " + str(self.memory))
                print("Last State: " + str(current_state))
                print("Solution found in Depth " + str(current_depth))
                self.print_path(current_state)
                return path
            visited_nodes.append(current_state[0])
            states = self.problem.results(self.problem.actions(current_state[0]), current_state[0])
            current_depth = current_depth + 1
            for state in states:
                if state not in visited_nodes:
                    number_of_visited_nodes = number_of_visited_nodes + 1
                    nodes_to_expand.append(state)
                    self.parent[state] = current_state
                    self.memory = self.memory + 1

        print("No solution in depth " + str(depth))
        return None

    def graph_iterative_deepening_search(self, start_state, depth=0):
        path = None
        while path is None:
            path = self.graph_depth_limited_search(start_state, depth)
            depth = depth + 1

    def tree_depth_first_search(self, start_state):
        path = []
        nodes_to_expand = [start_state]

        number_of_visited_nodes = 1
        number_of_expanded_nodes = 0

        while nodes_to_expand:
            current_state = nodes_to_expand.pop()
            path.append(current_state)
            number_of_expanded_nodes = number_of_expanded_nodes + 1
            if self.problem.isGoalTest(current_state):
                print("Algorithm: Graph DFS")
                print("Number Of Visited Nodes: " + str(number_of_visited_nodes))
                print("Number Of Expanded Nodes: " + str(number_of_expanded_nodes))
                print("Memory: " + str(self.memory))
                print("Last State: " + str(current_state))
                self.print_path(current_state)
                return path
            visited_nodes.append(current_state)
            states = self.problem.results(self.problem.actions(current_state), current_state)
            for state in states:
                    number_of_visited_nodes = number_of_visited_nodes + 1
                    nodes_to_expand.append(state)
                    self.parent[state] = current_state
                    self.memory = self.memory + 1

    def tree_breadth_first_search(self, start_state):
        path = []
        visited_nodes = []
        nodes_to_expand = [start_state]
        number_of_visited_nodes = 1
        number_of_expanded_nodes = 0

        while nodes_to_expand:
            current_state = nodes_to_expand.pop(0)
            path.append(current_state)
            number_of_expanded_nodes = number_of_expanded_nodes + 1
            if self.problem.isGoalTest(current_state):
                print("Algorithm: Graph BFS")
                print("Number Of Visited Nodes: " + str(number_of_visited_nodes))
                print("Number Of Expanded Nodes: " + str(number_of_expanded_nodes))
                print("Memory: " + str(self.memory))
                print("Last State: " + str(current_state))
                self.print_path(current_state)
                return path
            visited_nodes.append(current_state)
            states = self.problem.results(self.problem.actions(current_state), current_state)
            for state in states:
                if state not in visited_nodes:
                    number_of_visited_nodes = number_of_visited_nodes + 1
                    nodes_to_expand.append(state)
                    self.parent[state] = current_state
                    self.memory = self.memory + 1

    def tree_bidirectional_search(self, start_state, goal_state):
        path = []
        path_from_start = []
        path_from_goal = []
        visited_nodes = []
        nodes_to_expand_from_start = [start_state]
        nodes_to_expand_from_goal = [goal_state]

        while nodes_to_expand_from_goal or nodes_to_expand_from_start:
            current_state_from_start = nodes_to_expand_from_start.pop(0)
            path_from_start.append(current_state_from_start)
            current_state_from_goal = nodes_to_expand_from_goal.pop(0)
            path_from_goal.append(current_state_from_goal)
            if current_state_from_goal == current_state_from_start:
                print("Algorithm: Graph bidirectional")
                print("Memory: " + str(self.memory))
                for p1 in path_from_start:
                    path.append(p1)
                for p2 in reversed(path_from_goal):
                    path.append(p2)
                return path
            visited_nodes.append(current_state_from_goal)
            visited_nodes.append(current_state_from_start)
            states_from_start = self.problem.results(self.problem.actions(current_state_from_start), current_state_from_start)
            states_from_goal = self.problem.results(self.problem.actions(current_state_from_goal), current_state_from_goal)
            for state_from_start in states_from_start:
                nodes_to_expand_from_start.append(state_from_start)
                self.memory = self.memory + 1
            for state_from_goal in states_from_goal:
                nodes_to_expand_from_goal.append(state_from_goal)
                self.memory = self.memory + 1

    def tree_uniform_cost_search(self, start_state):
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
                    nodes_to_expand.append((state, path_cost + self.problem.step_cost(current_state[0], state)))
                    self.parent[state] = current_state[0]
                    self.memory = self.memory + 1

    def tree_depth_limited_search(self, start_state, depth):
        path = []
        current_depth = 0
        visited_nodes = []
        nodes_to_expand = [(start_state, 0)]

        number_of_visited_nodes = 1
        number_of_expanded_nodes = 0

        while nodes_to_expand and current_depth <= depth:
            current_state = nodes_to_expand.pop()
            path.append(current_state)
            number_of_expanded_nodes = number_of_expanded_nodes + 1
            if self.problem.isGoalTest(current_state):
                print("Algorithm: Graph DFS")
                print("Number Of Visited Nodes: " + str(number_of_visited_nodes))
                print("Number Of Expanded Nodes: " + str(number_of_expanded_nodes))
                print("Memory: " + str(self.memory))
                print("Last State: " + str(current_state))
                print("Solution found in Depth " + str(current_depth))
                self.print_path(current_state)
                return path
            visited_nodes.append(current_state[0])
            states = self.problem.results(self.problem.actions(current_state[0]), current_state[0])
            current_depth = current_depth + 1
            for state in states:
                if state not in visited_nodes:
                    number_of_visited_nodes = number_of_visited_nodes + 1
                    nodes_to_expand.append(state)
                    self.parent[state] = current_state
                    self.memory = self.memory + 1

        print("No solution in depth " + str(depth))
        return None

    def tree_iterative_deepening_search(self, start_state, depth=0):
        path = None
        while path is None:
            path = self.graph_depth_limited_search(start_state, depth)
            depth = depth + 1

    def print_path(self, leaf_node):
        path = []
        while leaf_node:
            path.append(leaf_node)
            if leaf_node in self.parent:
                leaf_node = self.parent[leaf_node]
            else:
                leaf_node = None
        self.problem.print_path(list(reversed(path)))

