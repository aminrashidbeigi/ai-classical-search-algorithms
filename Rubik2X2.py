from SearchAlgorithms.ClassicSearchAlgorithms import *
from SearchAlgorithms.BeyondClassicSearchAlgorithm import *
from copy import deepcopy


class Problem():
    def initialState(self):
        return ('', 'y', 'b', 'y', 'b', 'g', 'y', 'g', 'y', 'w', 'g', 'w', 'g', 'b', 'w', 'b', 'w', 'r', 'r', 'r', 'r',
                'o', 'o', 'o', 'o')

    def isGoalTest(self, state):
        color_of_square = ''
        for i in range(0, 25):
            if i % 4 == 1:
                color_of_square = state[i]
            elif color_of_square != state[i]:
                return False
        return True

    def actions(self, state):
        return ['T', 'TC', 'F', 'FC', 'R', 'RC']

    def results(self, actions, state):
        states = []
        for action in actions:
            state = list(state)
            temp_state = deepcopy(state)
            if 'F' is action:
                temp_state[7], temp_state[5], temp_state[6], temp_state[8] = state[5], state[6], state[8], state[7]
                temp_state[20], temp_state[18], temp_state[3], temp_state[4] = state[3], state[4], state[21], state[23]
                temp_state[21], temp_state[23], temp_state[10], temp_state[9] = state[10], state[9], state[20], state[18]
            elif 'FC' is action:
                temp_state[6], temp_state[8], temp_state[7], temp_state[5] = state[5], state[6], state[8], state[7]
                temp_state[21], temp_state[23], temp_state[10], temp_state[9] = state[3], state[4], state[21], state[23]
                temp_state[20], temp_state[18], temp_state[3], temp_state[4] = state[10], state[9], state[20], state[18]
            elif 'T' is action:
                temp_state[3], temp_state[1], temp_state[2], temp_state[4] = state[1], state[2], state[4], state[3]
                temp_state[18], temp_state[17], temp_state[15], temp_state[16] = state[15], state[16], state[22], state[24]
                temp_state[22], temp_state[24], temp_state[6], temp_state[5] = state[6], state[5], state[18], state[17]

            elif 'TC' is action:
                temp_state[2], temp_state[4], temp_state[3], temp_state[1] = state[1], state[2], state[4], state[3]
                temp_state[22], temp_state[24], temp_state[6], temp_state[5] = state[15], state[16], state[22], state[24]
                temp_state[18], temp_state[17], temp_state[15], temp_state[16] = state[6], state[5], state[18], state[17]
            elif 'R' is action:
                temp_state[23], temp_state[21], temp_state[22], temp_state[24] = state[21], state[22], state[24], state[
                    23]
                temp_state[8], temp_state[6], temp_state[4], temp_state[2] = state[4], state[2], state[16], state[14]
                temp_state[16], temp_state[14], temp_state[12], temp_state[10] = state[12], state[10], state[8], state[6]
            elif 'RC' is action:
                temp_state[22], temp_state[24], temp_state[23], temp_state[21] = state[21], state[22], state[24], state[
                    23]
                temp_state[16], temp_state[14], temp_state[12], temp_state[10] = state[4], state[2], state[16], state[14]
                temp_state[8], temp_state[6], temp_state[4], temp_state[2] = state[12], state[10], state[8], state[6]
            temp_state = tuple(temp_state)
            states.append(temp_state)
        return states

    def results_actions(self, state):
        states = []
        state = state[0]
        actions = self.actions(state)
        for action in actions:
            state = list(state)
            temp_state = deepcopy(state)
            if 'FC' is action:
                temp_state[7], temp_state[5], temp_state[6], temp_state[8] = state[5], state[6], state[8], state[7]
                temp_state[20], temp_state[18], temp_state[3], temp_state[4] = state[3], state[4], state[21], state[23]
                temp_state[21], temp_state[23], temp_state[10], temp_state[9] = state[10], state[9], state[20], state[18]
            elif 'F' is action:
                temp_state[6], temp_state[8], temp_state[7], temp_state[5] = state[5], state[6], state[8], state[7]
                temp_state[21], temp_state[23], temp_state[10], temp_state[9] = state[3], state[4], state[21], state[23]
                temp_state[20], temp_state[18], temp_state[3], temp_state[4] = state[10], state[9], state[20], state[18]
            elif 'TC' is action:
                temp_state[3], temp_state[1], temp_state[2], temp_state[4] = state[1], state[2], state[4], state[3]
                temp_state[18], temp_state[17], temp_state[15], temp_state[16] = state[15], state[16], state[22], state[24]
                temp_state[22], temp_state[24], temp_state[6], temp_state[5] = state[6], state[5], state[18], state[17]

            elif 'T' is action:
                temp_state[2], temp_state[4], temp_state[3], temp_state[1] = state[1], state[2], state[4], state[3]
                temp_state[22], temp_state[24], temp_state[6], temp_state[5] = state[15], state[16], state[22], state[24]
                temp_state[18], temp_state[17], temp_state[15], temp_state[16] = state[6], state[5], state[18], state[17]
            elif 'RC' is action:
                temp_state[23], temp_state[21], temp_state[22], temp_state[24] = state[21], state[22], state[24], state[
                    23]
                temp_state[8], temp_state[6], temp_state[4], temp_state[2] = state[4], state[2], state[16], state[14]
                temp_state[16], temp_state[14], temp_state[12], temp_state[10] = state[12], state[10], state[8], state[6]
            elif 'R' is action:
                temp_state[22], temp_state[24], temp_state[23], temp_state[21] = state[21], state[22], state[24], state[
                    23]
                temp_state[16], temp_state[14], temp_state[12], temp_state[10] = state[4], state[2], state[16], state[14]
                temp_state[8], temp_state[6], temp_state[4], temp_state[2] = state[12], state[10], state[8], state[6]
            states.append([temp_state, action])
        return states

    def print_path(self, path):
        print('Path:', end=' ')
        for current_state, next_state in zip(path, path[1:]):
            states = self.results_actions(current_state)
            for state in states:
                if state[0] == list(next_state):
                    print(state[1])


p = Problem()
csa = ClassicSearchAlgorithm(p)
bcsa = BeyondClassicSearchAlgorithm(p)
path = csa.graph_breadth_first_search(p.initialState())
# path = csa.graph_depth_limited_search(p.initialState(), 14)
# path = csa.graph_iterative_deepening_search(p.initialState(), 0)
