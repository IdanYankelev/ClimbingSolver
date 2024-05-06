from hold import Hold
from utils import *

LEG_LENGTH = 0.0


def set_leg_length(foot_hold: Hold, knee_position: np.array, hip_position: np.array):
    global LEG_LENGTH
    LEG_LENGTH = np.linalg.norm(knee_position - foot_hold.coordinates) + np.linalg.norm(hip_position - knee_position)
    

class LegState(IState):
    def __init__(self, side: Side, foot_hold: Hold, knee_position: np.array, hip_position: np.array):
        self.side = side
        self.foot_position = foot_hold.coordinates
        self.knee_position = knee_position
        self.hip_position = hip_position

    def __lt__(self, other):
        #epsilon = np.float64(0.0000001)
        
        return (self.foot_position[0] < other.foot_position[0] and self.knee_position[0] < other.knee_position[0] and
                self.hip_position[0] < other.hip_position[0])

    def __gt__(self, other):
        #epsilon = np.float64(0.0000001)
        
        return (self.foot_position[0] > other.foot_position[0] and self.knee_position[0] > other.knee_position[0] and
                self.hip_position[0] > other.hip_position[0])

    def is_possible_state(self) -> bool:

        if self.foot_position is None:
            return False

        if self.hip_position is None:
            return False

        if self.knee_position is None:
            return False

        # Check if feet above hip
        if self.foot_position[1] >= self.hip_position[1]:
            return False
        # Check if knee is below feet
        if self.foot_position[1] >= self.knee_position[1]:
            return False
        # Check if angle between hip, knee and feet is less than 180 degrees
        hip_knee_vector = self.hip_position - self.knee_position
        feet_knee_vector = self.foot_position - self.knee_position
        
        
        if calculate_angle_between_two_vectors(hip_knee_vector, feet_knee_vector) > np.pi:
            return False
        #Check if leg length is less than the maximum leg length
        
        
        if np.linalg.norm(self.foot_position - self.knee_position) + np.linalg.norm(self.hip_position - self.knee_position) > LEG_LENGTH:
            return False
        return True

    # generate possible moves and return list of possible leg states
    def generate_possible_moves(self, global_holds: list[Hold]) -> list['LegState']:
        legal_leg_states = []
        for hold in global_holds:
            if np.array_equal(hold.coordinates, self.foot_position):
                continue
            knee_position = self.calaculate_knee_position(hold)
            if knee_position is None:
                continue
            leg_state = LegState(self.side, hold, knee_position, self.hip_position)
            if leg_state.is_possible_state():
                legal_leg_states.append(leg_state)
        return legal_leg_states

    def calaculate_knee_position(self, new_feet_position: np.array) -> np.array:
        intersections = get_intersections(self.hip_position,
                                          np.linalg.norm(self.hip_position - self.knee_position),
                                          new_feet_position,
                                          np.linalg.norm(self.foot_position - self.knee_position))
        if intersections is None:
            return None
        return max(intersections, key=lambda x: x[1])
