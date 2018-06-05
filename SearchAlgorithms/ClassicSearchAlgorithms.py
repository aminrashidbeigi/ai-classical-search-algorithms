class ClassicSearchAlgorithm(object):
    def __init__(self, problem):
        self.problem = problem

    def GraphDFS(self, startState):
        visited_nodes = []
        nodes_to_expand = [startState]

        number_of_visited_nodes = 1
        number_of_expanded_nodes = 0

        while nodes_to_expand:
            state_to_check = nodes_to_expand.pop()
            number_of_expanded_nodes = number_of_expanded_nodes + 1
            if self.problem.isGoalTest(state_to_check):
                print("Algorithm: Graph DFS")
                print("Number Of Visited Nodes: " + str(number_of_visited_nodes))
                print("Number Of Expanded Nodes: " + str(number_of_expanded_nodes))
                return state_to_check
            visited_nodes.append(state_to_check)
            states = self.problem.results(self.problem.actions(state_to_check), state_to_check)
            # print(states)
            for state in states:
                if state not in visited_nodes:
                    number_of_visited_nodes = number_of_visited_nodes + 1
                    nodes_to_expand.append(state)

    def GraphBFS(self, start_state):
        visited_nodes = []
        nodes_to_expand = [start_state]

        number_of_visited_nodes = 1
        number_of_expanded_nodes = 0

        while nodes_to_expand:
            state_to_check = nodes_to_expand.pop(0)
            number_of_expanded_nodes = number_of_expanded_nodes + 1
            if self.problem.isGoalTest(state_to_check):
                print("Algorithm: Graph BFS")
                print("Number Of Visited Nodes: " + str(number_of_visited_nodes))
                print("Number Of Expanded Nodes: " + str(number_of_expanded_nodes))
                return state_to_check
            visited_nodes.append(state_to_check)
            states = self.problem.results(self.problem.actions(state_to_check), state_to_check)
            # print(states)
            for state in states:
                if state not in visited_nodes:
                    number_of_visited_nodes = number_of_visited_nodes + 1
                    nodes_to_expand.append(state)

    # def UCS(self, start_node):

    def bidirectional(self, start_state, goal_state):
        visited_nodes = []
        nodes_to_expand_from_start = [start_state]
        nodes_to_expand_from_goal = [goal_state]

        while nodes_to_expand_from_goal or nodes_to_expand_from_start:
            state_to_check_from_start = nodes_to_expand_from_start.pop(0)
            state_to_check_from_goal = nodes_to_expand_from_goal.pop(0)
            if state_to_check_from_goal == state_to_check_from_start:
                print("Algorithm: Graph bidirectional")
                print(state_to_check_from_goal)
                return state_to_check_from_start
            visited_nodes.append(state_to_check_from_goal)
            visited_nodes.append(state_to_check_from_start)
            states_from_start = self.problem.results(self.problem.actions(state_to_check_from_start), state_to_check_from_start)
            states_from_goal = self.problem.results(self.problem.actions(state_to_check_from_goal), state_to_check_from_goal)
            for state_from_start in states_from_start:
                nodes_to_expand_from_start.append(state_from_start)
            for state_from_goal in states_from_goal:
                nodes_to_expand_from_goal.append(state_from_goal)

