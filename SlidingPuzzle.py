from SearchAlgorithms.ClassicSearchAlgorithms import *
from SearchAlgorithms.BeyondClassicSearchAlgorithm import *
from copy import deepcopy


class Problem():
    def initialState(self):
        # return [
        #     [4, 5, 2],
        #     [1, 7, 3],
        #     [0, 6, 8]
        # ]

        return (
            (3, 1, 2),
            (6, 4, 5),
            (0, 7, 8)
        )

    def goal(self):
        return (
            (0, 1, 2),
            (3, 4, 5),
            (6, 7, 8)
        )

    def isGoalTest(self, state):
        return state == self.goal()

    def actions(self, state):
        zero_row = -1
        zero_col = -1
        actions = []
        for row in state:
            for col in row:
                if col == 0:
                    zero_row = state.index(row)
                    zero_col = row.index(col)
        if zero_col > 0:
            actions.append('L')
        if zero_col < 2:
            actions.append('R')
        if zero_row > 0:
            actions.append('U')
        if zero_row < 2:
            actions.append('D')
        return actions

    def results(self, actions, state):
        zero_row = -1
        zero_col = -1
        states = []
        for row in state:
            for col in row:
                if col == 0:
                    zero_row = state.index(row)
                    zero_col = row.index(col)
        for action in actions:
            state = list(list(item) for item in state)
            temp_state = deepcopy(state)
            if 'U' in action:
                temp_state[zero_row][zero_col], temp_state[zero_row - 1][zero_col] = state[zero_row - 1][zero_col], \
                                                                                     state[zero_row][zero_col]
            if 'D' in action:
                temp_state[zero_row][zero_col], temp_state[zero_row + 1][zero_col] = state[zero_row + 1][zero_col], \
                                                                                     state[zero_row][zero_col]
            if 'L' in action:
                temp_state[zero_row][zero_col], temp_state[zero_row][zero_col - 1] = state[zero_row][zero_col - 1], \
                                                                                     state[zero_row][zero_col]
            if 'R' in action:
                temp_state[zero_row][zero_col], temp_state[zero_row][zero_col + 1] = state[zero_row][zero_col + 1], \
                                                                                     state[zero_row][zero_col]
            temp_state = tuple(tuple(item) for item in temp_state)
            states.append(temp_state)
        return states

    def step_cost(self, current_state, next_state):
        return 1

    def heuristic(self, state):
        zero_row = -1
        zero_col = -1
        for row in state:
            for col in row:
                if col == 0:
                    zero_row = state.index(row)
                    zero_col = row.index(col)
        return zero_col + zero_row

    def print_path(self, path):
        def find_direction(current_state, next_state):
            current_zero_row = -1
            current_zero_col = -1
            for row in current_state:
                for col in row:
                    if col == 0:
                        current_zero_row = current_state.index(row)
                        current_zero_col = row.index(col)
            next_zero_row = -1
            next_zero_col = -1
            for row in next_state:
                for col in row:
                    if col == 0:
                        next_zero_row = next_state.index(row)
                        next_zero_col = row.index(col)
            if next_zero_col == current_zero_col and next_zero_row > current_zero_row:
                print('D', end=" ")
            if next_zero_col == current_zero_col and next_zero_row < current_zero_row:
                print('U', end=" ")
            if next_zero_col > current_zero_col and next_zero_row == current_zero_row:
                print('R', end=" ")
            if next_zero_col < current_zero_col and next_zero_row == current_zero_row:
                print('L', end=" ")

        print("Path:", end=" ")
        for current_state, next_state in zip(path, path[1:]):
            find_direction(current_state, next_state)
        print()

p = Problem()
csa = ClassicSearchAlgorithm(p)
bcsa = BeyondClassicSearchAlgorithm(p)

# csa.graph_depth_first_search(p.initialState())
# csa.graph_bidirectional_search(p.initialState(), p.goal())
# csa.graph_uniform_cost_search(p.initialState())
# bcsa.graph_a_star(p.initialState())
