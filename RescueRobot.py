from SearchAlgorithms.ClassicSearchAlgorithms import *
from SearchAlgorithms.BeyondClassicSearchAlgorithm import *
from copy import deepcopy


class Problem():
    n = 5
    m = 5
    size = [n, m]
    number_of_blocks = 4
    list_of_blocks = [
        ((3, 2), (4, 2)),
        ((3, 3), (4, 3)),
        ((2, 3), (2, 4)),
        ((3, 3), (3, 4)),
    ]

    def initialState(self):
        return (1, 1)

    def goal(self):
        return (self.n, self.m)

    def isGoalTest(self, state):
        return state == self.goal()

    def actions(self, state):
        actions = []
        if state[0] > 1:
            actions.append('U')
        if state[0] < self.n:
            actions.append('D')
        if state[1] > 1:
            actions.append('L')
        if state[1] < self.m:
            actions.append('R')
        return actions

    def results(self, actions, state):
        def is_safe_to_move(direction, location, block_list):
            for block_row in block_list:
                if list(block_row[0]) == location:
                    if 'U' in direction and block_row[0][0] == block_row[1][0] + 1:
                        return False
                    elif 'D' in direction and block_row[0][0] == block_row[1][0] - 1:
                        return False
                    elif 'R' in direction and block_row[0][1] == block_row[1][1] - 1:
                        return False
                    elif 'L' in direction and block_row[0][1] == block_row[1][1] + 1:
                        return False
                if list(block_row[1]) == location:
                    if 'U' in direction and block_row[0][0] == block_row[1][0] - 1:
                        return False
                    elif 'D' in direction and block_row[0][0] == block_row[1][0] + 1:
                        return False
                    elif 'R' in direction and block_row[0][1] == block_row[1][1] + 1:
                        return False
                    elif 'L' in direction and block_row[0][1] == block_row[1][1] - 1:
                        return False
            return True

        states = []
        for action in actions:
            temp_state = deepcopy(state)
            temp_state = list(temp_state)
            state = list(state)
            if is_safe_to_move(action, state, self.list_of_blocks):
                if 'U' in action:
                    temp_state[0], temp_state[1] = state[0] - 1, state[1]
                elif 'D' in action:
                    temp_state[0], temp_state[1] = state[0] + 1, state[1]
                elif 'L' in action:
                    temp_state[0], temp_state[1] = state[0], state[1] - 1
                elif 'R' in action:
                    temp_state[0], temp_state[1] = state[0], state[1] + 1
            states.append(tuple(temp_state))
        return states

    def step_cost(self, current_state, next_state):
        return 1

    def heuristic(self, state):
        return list(state)[0] + list(state)[1] - 2

    def print_path(self, path):
        # TODO: this function has bug in some algorithms
        def find_direction(current_state, next_state):
            if current_state[0] == next_state[0] and current_state[1] > next_state[1]:
                print('L', end=" ")
            elif current_state[0] == next_state[0] and current_state[1] < next_state[1]:
                print('R', end=" ")
            elif current_state[0] > next_state[0] and current_state[1] == next_state[1]:
                print('U', end=" ")
            elif current_state[0] < next_state[0] and current_state[1] == next_state[1]:
                print('D', end=" ")

        print("Path:", end=" ")
        for current_state, next_state in zip(path, path[1:]):
            find_direction(current_state, next_state)


p = Problem()
csa = ClassicSearchAlgorithm(p)
bcsa = BeyondClassicSearchAlgorithm(p)
# csa.graph_depth_first_search(p.initialState())
# csa.graph_uniform_cost_search(p.initialState())
# csa.graph_bidirectional_search(p.initialState(), p.goal())
# bcsa.a_star(p.initialState())
