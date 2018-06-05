from SearchAlgorithms.ClassicSearchAlgorithms import *
from copy import deepcopy

class Problem():
    def initialState(self):
        # return [
        #     [4, 5, 2],
        #     [1, 7, 3],
        #     [0, 6, 8]
        # ]

        return [
            [3, 1, 2],
            [6, 4, 5],
            [7, 0, 8]
        ]

    def goal(self):
        return [
            [0, 1, 2],
            [3, 4, 5],
            [6, 7, 8]
        ]

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
            actions.append('Left')
        if zero_col < 2:
            actions.append('Right')
        if zero_row > 0:
            actions.append('Up')
        if zero_row < 2:
            actions.append('Down')
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
            temp_state = deepcopy(state)
            if 'Up' in action:
                temp_state[zero_row][zero_col], temp_state[zero_row - 1][zero_col] = state[zero_row - 1][zero_col], \
                                                                                     state[zero_row][zero_col]
            if 'Down' in action:
                temp_state[zero_row][zero_col], temp_state[zero_row + 1][zero_col] = state[zero_row + 1][zero_col], \
                                                                                     state[zero_row][zero_col]
            if 'Left' in action:
                temp_state[zero_row][zero_col], temp_state[zero_row][zero_col - 1] = state[zero_row][zero_col - 1], \
                                                                                     state[zero_row][zero_col]
            if 'Right' in action:
                temp_state[zero_row][zero_col], temp_state[zero_row][zero_col + 1] = state[zero_row][zero_col + 1], \
                                                                                     state[zero_row][zero_col]
            states.append(temp_state)
            # print(action)
            # print(temp_state)

        return states


p = Problem()
csa = ClassicSearchAlgorithm(p)
# csa.GraphDFS(p.initialState())
# csa.GraphBFS(p.initialState())
csa.bidirectional(p.initialState(), p.goal())
