from numpy import ndarray

from hold import Hold
from leg_state import LegState
from utils import *

ARM_LENGTH = 0.0


def set_arm_length(value: np.float64):
    global ARM_LENGTH
    ARM_LENGTH = value


class ArmState(IState):
    def __init__(self, side: Side, hand_hold: Hold, shoulder_position: np.array):
        self.side = side
        self.hand_position: np.array = hand_hold.coordinates
        self.shoulder_position: np.array = shoulder_position

    def is_possible_state(self) -> bool:
        if self.shoulder_position is None:
            return False
        if self.hand_position is None:
            return False
        # Check if hand length is less than arm length
        return np.linalg.norm(self.hand_position - self.shoulder_position) <= ARM_LENGTH

    def generate_possible_moves(self, global_holds) -> list['ArmState']:
        legal_arm_states: 'ArmState' = []
        for hold in global_holds:
            if np.array_equal(hold.coordinates, self.hand_position):
                continue
            arm_state = ArmState(self.side, hold, self.shoulder_position)
            if arm_state.is_possible_state():
                legal_arm_states.append(arm_state)
        return legal_arm_states


def line_intersection(point1: np.array, point2: np.array, point3: np.array, point4: np.array) -> np.array:
    f1 = np.poly1d(np.polyfit([point1[0], point2[0]], [point1[1], point2[1]], 1))
    f2 = np.poly1d(np.polyfit([point3[0], point4[0]], [point3[1], point4[1]], 1))
    x = (f2 - f1).roots[0]
    y = f1(x)
    return np.array([x, y])


def calculate_length_from_feet_to_center(left_leg_state: LegState, right_leg_state: LegState) -> float | ndarray:
    # calculate the length from the feet to the center of mass
    return np.linalg.norm(
        line_intersection(left_leg_state.knee_position, left_leg_state.hip_position,
                          right_leg_state.knee_position,right_leg_state.hip_position) - left_leg_state.knee_position)
