class Problem:

    def initialState(self):
        return [
            [4, 5, 2],
            [1, 7, 3],
            [0, 6, 8]
        ]

    def goal(self):
        return [
            [0, 1, 2],
            [3, 4, 5],
            [6, 7, 8]
        ]

    def isGoalTest(self, state):
        return state == self.goal()

