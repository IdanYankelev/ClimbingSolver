from human_state import HumanState


class Search_Node:
    def __init__(self, human_state: HumanState, g, h, prev=None):
        self.prev: Search_Node = prev
        self.g = g
        self.h = h
        self.f = g + h
        self.human_state: HumanState = human_state

    def generate_possible_moves(self, global_holds) -> set['HumanState']:
        return self.human_state.generate_possible_moves(global_holds)

    def __lt__(self, other):
        return (self.f < other.f) or (self.f == other.f and self.h < other.h)

    def __gt__(self, other):
        return (self.f > other.f) or (self.f == other.f and self.h > other.h)

    # weight heuristic
    # def __lt__(self, other):
    #     return (self.f < other.f) or (self.f == other.f and (0.9 * self.h + 0.1 * self.g) < (0.9 * other.h + 0.1 * other.g))

    # def __gt__(self, other):
    #     return (self.f > other.f) or (self.f == other.f and (0.9 * self.h + 0.1 * self.g) > (0.9 * other.h + 0.1 * other.g))
