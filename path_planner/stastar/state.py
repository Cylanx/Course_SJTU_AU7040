import numpy as np

class State:

    def __init__(self, id: int, time: int, g_score: int, h_score: int):
        self.id = id
        self.time = time
        self.g_score = g_score
        self.f_score = g_score + h_score

    def __hash__(self) -> int:
        concat = str(self.id) + '0' + str(self.time)
        return int(concat)

    def equal_to(self, goal: int) -> bool:
        return self.id == goal

    def __lt__(self, other: 'State') -> bool:
        return self.f_score < other.f_score

    def __eq__(self, other: 'State') -> bool:
        return self.__hash__() == other.__hash__()

    def __str__(self):
        return 'State(id=[' + str(self.id) + '], ' \
               + 'time=' + str(self.time) + ', fscore=' + str(self.f_score) + ')'

    def __repr__(self):
        return self.__str__()
