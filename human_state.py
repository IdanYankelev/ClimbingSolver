import copy

from arm_state import *
from leg_state import *
import leg_state
import arm_state
LEG_SPREAD = 0.0
# LENGTH_FROM_FEET_TO_CENTER = 0.0


def set_leg_spread(value: np.float64):
    global LEG_SPREAD
    LEG_SPREAD = value


# def set_length_from_feet_to_center(left_leg_state, right_leg_state):
#     global LENGTH_FROM_FEET_TO_CENTER
#     LENGTH_FROM_FEET_TO_CENTER = calculate_length_from_feet_to_center(left_leg_state, right_leg_state)


class HumanState(IState):
    def __init__(self, left_leg_state: LegState, right_leg_state: LegState,
                 left_arm_state: ArmState, right_arm_state: ArmState):
        self.left_leg_state: LegState = left_leg_state
        self.right_leg_state: LegState = right_leg_state
        self.left_arm_state: ArmState = left_arm_state
        self.right_arm_state: ArmState = right_arm_state
    def __eq__(self, other):
        if not isinstance(other, type(self)):
            return NotImplemented

        return (np.array_equal(self.left_leg_state.foot_position, other.left_leg_state.foot_position) and
                np.array_equal(self.right_leg_state.foot_position, other.right_leg_state.foot_position) and
                np.array_equal(self.left_arm_state.hand_position, other.left_arm_state.hand_position) and
                np.array_equal(self.right_arm_state.hand_position, other.right_arm_state.hand_position))
    
    def __hash__(self):
        left_leg_foot_position_tuple = tuple(self.left_leg_state.foot_position)
        right_leg_foot_position_tuple = tuple(self.right_leg_state.foot_position)
        left_arm_hand_position_tuple = tuple(self.left_arm_state.hand_position)
        right_arm_hand_position_tuple = tuple(self.right_arm_state.hand_position)
        
        # Return the hash of a tuple containing all the position tuples
        return hash((left_leg_foot_position_tuple, right_leg_foot_position_tuple,
                    left_arm_hand_position_tuple, right_arm_hand_position_tuple))

    def get_arm_length(self) -> np.float64:
        return arm_state.ARM_LENGTH

    def get_leg_length(self) -> np.float64:
        return leg_state.LEG_LENGTH

        
    def is_possible_state(self) -> bool:
        # check if arm is not none
        if self.left_arm_state is None or self.right_arm_state is None:
            return False
        
        # check if leg is not none
        if self.left_leg_state is None or self.right_leg_state is None:
            return False

        # Check if the arms are possible
        if not self.left_arm_state.is_possible_state() or not self.right_arm_state.is_possible_state():
            return False

        # Check if the legs are possible
        if not self.left_leg_state.is_possible_state() or not self.right_leg_state.is_possible_state():
            return False

        # Check if the legs are not intersecting
        if self.left_leg_state > self.right_leg_state:
            return False

        # Check if both hands are above the legs
        if self.left_arm_state.hand_position[1] < self.left_leg_state.foot_position[1] or \
                self.right_arm_state.hand_position[1] < self.right_leg_state.foot_position[1]:
            return False

        # Check if the angle between the legs is less than LEG_SPREAD
        if self.calculate_angle_between_legs(self.left_leg_state, self.right_leg_state) > LEG_SPREAD:
            return False

        return True

    def calculate_angle_between_legs(self, left_leg_state: LegState, right_leg_state: LegState) -> np.float64:
        # calculate the angle between the legs
        left_leg_vector = left_leg_state.hip_position - left_leg_state.knee_position
        right_leg_vector = right_leg_state.hip_position - right_leg_state.knee_position
        return calculate_angle_between_two_vectors(left_leg_vector, right_leg_vector)

   

    
    def generate_possible_moves(self, global_holds: list[Hold]) -> set['HumanState']:
        """
        Generates all possible human states by moving only a single arm/leg.
        """

        # filtered_global_holds = list(filter(lambda hold: hold.coordinates[1] >= self.left_leg_state.foot_position[1] and hold.coordinates[1] >= self.right_leg_state.foot_position[1], global_holds))

        human_states: HumanState = [copy.deepcopy(self)]

        # Find all possible states by straightening the arms.
        
        # Straighten the left arm.
        human_state = copy.deepcopy(self)
        if(human_state.straighten_arm(Side.LEFT)):
            human_states.append(human_state)
        
        # Straighten the right arm.
        human_state = copy.deepcopy(self)
        if(human_state.straighten_arm(Side.RIGHT)):
            human_states.append(human_state)
        # --------------------------------------------
        
        # Straighten up the human.
        human_state = copy.deepcopy(self)
        if(human_state.straighten_up()):
            human_states.append(human_state)
        # --------------------------------------------
        
        # Straighten the left and right arms.
        human_state = copy.deepcopy(self)
        if(human_state.straighten_arm(Side.LEFT) and human_state.straighten_arm(Side.RIGHT)):
            human_states.append(human_state)
        # --------------------------------------------
        
        # Straighten the left arm and straighten up the human.
        human_state = copy.deepcopy(self)
        if(human_state.straighten_arm(Side.LEFT) and human_state.straighten_up()):
            human_states.append(human_state)
        # --------------------------------------------
        
        # Straighten the right arm and straighten up the human.
        human_state = copy.deepcopy(self)
        if(human_state.straighten_arm(Side.RIGHT) and human_state.straighten_up()):
            human_states.append(human_state)
        
        # --------------------------------------------
        
        # Straighten the both arms and straighten up the human.
        human_state = copy.deepcopy(self)
        if(human_state.straighten_arm(Side.LEFT) and 
            human_state.straighten_arm(Side.RIGHT) and
            human_state.straighten_up()):
            
            human_states.append(human_state)

        

        legal_human_states: HumanState = []

        for human_state in human_states:
            left_leg_states = human_state.left_leg_state.generate_possible_moves(global_holds)
            right_leg_states = human_state.right_leg_state.generate_possible_moves(global_holds)
            left_arm_states = human_state.left_arm_state.generate_possible_moves(global_holds)
            right_arm_states = human_state.right_arm_state.generate_possible_moves(global_holds)

            for left_leg_state in left_leg_states:
                new_human_state = HumanState(left_leg_state, human_state.right_leg_state, human_state.left_arm_state, human_state.right_arm_state)
                if new_human_state.is_possible_state():
                    legal_human_states.append(new_human_state)
                
            for right_leg_state in right_leg_states:
                new_human_state = HumanState(human_state.left_leg_state, right_leg_state, human_state.left_arm_state, human_state.right_arm_state)
                if new_human_state.is_possible_state():
                    legal_human_states.append(new_human_state)

            for left_arm_state in left_arm_states:
                new_human_state = HumanState(human_state.left_leg_state, human_state.right_leg_state, left_arm_state, human_state.right_arm_state)
                if new_human_state.is_possible_state():
                    legal_human_states.append(new_human_state)

            for right_arm_state in right_arm_states:
                new_human_state = HumanState(human_state.left_leg_state, human_state.right_leg_state, human_state.left_arm_state, right_arm_state)
                if new_human_state.is_possible_state():
                    legal_human_states.append(new_human_state)

        return set(legal_human_states)
    
    def straighten_arm(self, side: Side) -> bool:
        distance_between_sholders = np.linalg.norm(self.left_arm_state.shoulder_position - self.right_arm_state.shoulder_position)
        
        if side == Side.LEFT:
            
            # left sholder
            length_between_hip_and_shoulder = np.linalg.norm(self.left_arm_state.shoulder_position - self.left_leg_state.hip_position)
            left_shoulder_positions = get_intersections(self.left_leg_state.hip_position,
                                            length_between_hip_and_shoulder,
                                            self.left_arm_state.hand_position,
                                            arm_state.ARM_LENGTH)
            if left_shoulder_positions is None:
                return False
            
            if np.array_equal(left_shoulder_positions[0][1], left_shoulder_positions[1][1]):
                # take the right most point
                left_shoulder: np.array = max(left_shoulder_positions, key=lambda x: x[0])
            else:
                left_shoulder: np.array = max(left_shoulder_positions, key=lambda x: x[1])
            if np.array_equal(self.left_arm_state.shoulder_position, left_shoulder):
                return False
            
            old_left_shoulder = self.left_arm_state.shoulder_position

            self.left_arm_state.shoulder_position = left_shoulder
            
            
            # right sholder
            length_between_hip_and_shoulder = np.linalg.norm(self.right_arm_state.shoulder_position - self.right_leg_state.hip_position)
            right_shoulder_positions = get_intersections(self.right_leg_state.hip_position,
                                            length_between_hip_and_shoulder,
                                            self.left_arm_state.shoulder_position,
                                            distance_between_sholders)
            if right_shoulder_positions is None:
                return False
    
            right_shoulder: np.array = max(right_shoulder_positions, key=lambda x: x[1])

            old_right_shoulder = self.right_arm_state.shoulder_position

            self.right_arm_state.shoulder_position = right_shoulder

            old_left_hip = self.left_leg_state.hip_position
            old_right_hip = self.right_leg_state.hip_position
            self.update_hips_position_upon_straighten_arm(old_left_shoulder, old_right_shoulder)
            if not self.update_knees_position_upon_straighten_arm(old_left_hip, old_right_hip):
                return False
            
            
            
        else:
            # right sholder
            length_between_hip_and_shoulder = np.linalg.norm(self.right_arm_state.shoulder_position - self.right_leg_state.hip_position)
            right_shoulder_posistions = get_intersections(self.right_leg_state.hip_position,
                                            length_between_hip_and_shoulder,
                                            self.right_arm_state.hand_position,
                                            arm_state.ARM_LENGTH)
            if right_shoulder_posistions is None:
                return False
            if np.array_equal(right_shoulder_posistions[0][1], right_shoulder_posistions[1][1]):
                # take the left most point
                right_shoulder: np.array = min(right_shoulder_posistions, key=lambda x: x[0])
            else:
                right_shoulder: np.array = max(right_shoulder_posistions, key=lambda x: x[1])
            
            if np.array_equal(self.right_arm_state.shoulder_position, right_shoulder):
                return False

            old_right_shoulder = self.right_arm_state.shoulder_position

            self.right_arm_state.shoulder_position = right_shoulder
            
            
            # left sholder
            length_between_hip_and_shoulder = np.linalg.norm(self.left_arm_state.shoulder_position - self.left_leg_state.hip_position)
            left_sholder_posistions = get_intersections(self.left_leg_state.hip_position,
                                            length_between_hip_and_shoulder,
                                            self.right_arm_state.shoulder_position,
                                            distance_between_sholders)
            if left_sholder_posistions is None:
                return False

            left_sholder: np.array = max(left_sholder_posistions, key=lambda x: x[1])

            old_left_shoulder = self.left_arm_state.shoulder_position

            self.left_arm_state.shoulder_position = left_sholder

            old_left_hip = self.left_leg_state.hip_position
            old_right_hip = self.right_leg_state.hip_position
            self.update_hips_position_upon_straighten_arm(old_left_shoulder, old_right_shoulder)
            if not self.update_knees_position_upon_straighten_arm(old_left_hip, old_right_hip):
                return False

        return True
    
    
    def update_hips_position_upon_straighten_arm(self, old_left_shoulder: np.array, old_right_shoulder: np.array) -> None:
        # update the hips position
        self.left_leg_state.hip_position = self.left_leg_state.hip_position + (
                self.left_arm_state.shoulder_position - old_left_shoulder)
        self.right_leg_state.hip_position = self.right_leg_state.hip_position + (
                self.right_arm_state.shoulder_position - old_right_shoulder)
    
    def update_knees_position_upon_straighten_arm(self, old_left_hip: np.array, old_right_hip: np.array) -> None:
        # update the knees position
        knee_length = np.linalg.norm(self.left_leg_state.knee_position - self.left_leg_state.foot_position)
        knee_to_hip_length = np.linalg.norm(old_left_hip - self.left_leg_state.knee_position)

        intersections = get_intersections(self.left_leg_state.foot_position,
                                          knee_length,
                                          self.left_leg_state.hip_position,
                                          knee_to_hip_length)
        if intersections is not None:
            self.left_leg_state.knee_position = max(intersections, key=lambda x: x[1])
        else:
            return False

        knee_length = np.linalg.norm(self.right_leg_state.knee_position - self.right_leg_state.foot_position)
        knee_to_hip_length = np.linalg.norm(old_right_hip - self.right_leg_state.knee_position)

        intersections = get_intersections(self.right_leg_state.foot_position,
                                          knee_length,
                                          self.right_leg_state.hip_position,
                                          knee_to_hip_length)
        if intersections is not None:
            self.right_leg_state.knee_position = max(intersections, key=lambda x: x[1])
        else:
            return False
        return True


    # def straighten_up1(self) -> bool:
    #     # straighten up the human
    #     old_left_hip: np.array = self.left_leg_state.hip_position
    #     old_right_hip: np.array = self.right_leg_state.hip_position
    #     center_of_mass: np.array = self.calculate_center_of_mass()
    #     if center_of_mass is None:
    #         return False
    #     self.update_hips_position_upon_straighten_up(center_of_mass)
    #     self.update_shoulders_position_upon_straighten_up(old_left_hip, old_right_hip)
    #     self.update_hands_position_upon_straighten_up()
    #     self.update_knees_position_upon_straighten_up(center_of_mass)

    #     return True
    # def update_knees_position_upon_straighten_up(self, center_of_mass: np.array) -> None:
    #     # update the knees position
    #     knee_length = np.linalg.norm(self.left_leg_state.knee_position - self.left_leg_state.foot_position)

    #     new_center_of_mass = get_intersections(self.left_leg_state.foot_position,
    #                                            LENGTH_FROM_FEET_TO_CENTER + knee_length,
    #                                            self.right_leg_state.foot_position,
    #                                            LENGTH_FROM_FEET_TO_CENTER + knee_length)
    #     # left leg
    #     vector_left_feet_to_center_of_mass = new_center_of_mass - self.left_leg_state.foot_position
    #     self.left_leg_state.knee_position = vector_left_feet_to_center_of_mass * (
    #             knee_length / (LENGTH_FROM_FEET_TO_CENTER + knee_length)) + self.left_leg_state.foot_position

    #     # right leg
    #     vector_right_feet_to_center_of_mass = new_center_of_mass - self.right_leg_state.foot_position
    #     self.right_leg_state.knee_position = vector_right_feet_to_center_of_mass * (
    #             knee_length / (LENGTH_FROM_FEET_TO_CENTER + knee_length)) + self.right_leg_state.foot_position

    def update_shoulders_position_upon_straighten_up(self, old_left_hip: np.array, old_right_hip: np.array) -> None:
        # update the shoulders position
        self.left_arm_state.shoulder_position = self.left_arm_state.shoulder_position + (
                self.left_leg_state.hip_position - old_left_hip)
        self.right_arm_state.shoulder_position = self.right_arm_state.shoulder_position + (
                self.right_leg_state.hip_position - old_right_hip)

    def update_hands_position_upon_straighten_up(self) -> bool:
        # Check if the hands position is to far from the shoulders

        epsilon = np.float64(0.0001)
        # Check if both hands position are to far from the shoulders
        if np.linalg.norm(self.left_arm_state.hand_position - self.left_arm_state.shoulder_position) > arm_state.ARM_LENGTH + epsilon and \
                np.linalg.norm(self.right_arm_state.hand_position - self.right_arm_state.shoulder_position) > arm_state.ARM_LENGTH + epsilon:
            return False
        if np.linalg.norm(self.left_arm_state.hand_position - self.left_arm_state.shoulder_position) > arm_state.ARM_LENGTH + epsilon:
            self.left_arm_state.hand_position = self.right_arm_state.hand_position

        elif np.linalg.norm(self.right_arm_state.hand_position - self.right_arm_state.shoulder_position) > arm_state.ARM_LENGTH + epsilon:
            self.right_arm_state.hand_position = self.left_arm_state.hand_position
        return True
    # def update_hips_position_upon_straighten_up(self, center_of_mass: np.array) -> None:
    #     # update the hips position

    #     vector_left_feet_to_center_of_mass = center_of_mass - self.left_leg_state.foot_position
    #     self.left_leg_state.hip_position = vector_left_feet_to_center_of_mass * (
    #             LEG_LENGTH / LENGTH_FROM_FEET_TO_CENTER) + self.left_leg_state.foot_position

    #     vector_right_feet_to_center_of_mass = center_of_mass - self.right_leg_state.foot_position
    #     self.right_leg_state.hip_position = vector_right_feet_to_center_of_mass * (
    #             LEG_LENGTH / LENGTH_FROM_FEET_TO_CENTER) + self.right_leg_state.foot_position

    # def calculate_center_of_mass1(self) -> np.array:
    #     intersections = get_intersections(self.left_leg_state.foot_position,
    #                                       LENGTH_FROM_FEET_TO_CENTER,
    #                                       self.right_leg_state.foot_position,
    #                                       LENGTH_FROM_FEET_TO_CENTER)
    #     if intersections is None:
    #         return None
    #     return max(intersections, key=lambda x: x[1])

    # def calculate_center_of_mass(self) -> np.array:
    #     intersections = get_intersections(self.left_leg_state.foot_position,
    #                                       LENGTH_FROM_FEET_TO_CENTER,
    #                                       self.right_leg_state.foot_position,
    #                                       LENGTH_FROM_FEET_TO_CENTER)
    #     if intersections is None:
    #         return None
    #     return max(intersections, key=lambda x: x[1])

    def straighten_up(self) -> bool:
        # straighten up the human
        old_left_hip: np.array = self.left_leg_state.hip_position
        old_right_hip: np.array = self.right_leg_state.hip_position

        center_of_mass: np.array = self.find_center_of_mass()
        if center_of_mass is None:
            return False
        if not np.array_equal(center_of_mass, np.array([-1,-1])):    
            self.update_hips(center_of_mass)
            self.update_knees(center_of_mass)
        else:
            self.update_hips_and_knees_special_case()

        self.update_shoulders_position_upon_straighten_up(old_left_hip, old_right_hip)
        
        return self.update_hands_position_upon_straighten_up()
    """
    This function is a special case for the straighten up function
    This only happens when the distance between the feet and the hips are the same
    And we cannot use our similar triangles formula to calculate the center of mass
    We can simply update the knees and the hips using the orthogonal feet vector
    
    """    
    def update_hips_and_knees_special_case(self) -> None:
        feet_vector = self.left_leg_state.foot_position - self.right_leg_state.foot_position
        feet_vector = feet_vector / np.linalg.norm(feet_vector)
        knee_length = np.linalg.norm(self.left_leg_state.knee_position - self.left_leg_state.foot_position)
        #orthogonal vectors going up
        orthogonal_vectors = [np.array([-feet_vector[1], feet_vector[0]]), np.array([feet_vector[1], -feet_vector[0]])]
        # the ortogonal vector with max y value is the one going up
        orthogonal_vector = max(orthogonal_vectors, key=lambda x: x[1])
        #left leg
        self.left_leg_state.hip_position = self.left_leg_state.foot_position + orthogonal_vector * leg_state.LEG_LENGTH
        self.left_leg_state.knee_position = self.left_leg_state.foot_position + orthogonal_vector * knee_length
        
        #right leg
        self.right_leg_state.hip_position = self.right_leg_state.foot_position + orthogonal_vector * leg_state.LEG_LENGTH
        self.right_leg_state.knee_position = self.right_leg_state.foot_position + orthogonal_vector * knee_length
        
    # The formula is based on simiarity between triangles
    def calculate_distance_between_foot_and_center_of_mass(self) -> np.float64:
        distance_between_feet: np.float64 = np.linalg.norm(self.left_leg_state.foot_position - self.right_leg_state.foot_position)
        distance_between_hips: np.float64 = np.linalg.norm(self.left_leg_state.hip_position - self.right_leg_state.hip_position)
        
        if(np.isclose(distance_between_hips, distance_between_feet)):
            return -1
        return leg_state.LEG_LENGTH + (leg_state.LEG_LENGTH * distance_between_hips)/(distance_between_feet - distance_between_hips)
    
    def find_center_of_mass(self) -> np.array:
        distance_between_foot_and_center_of_mass = self.calculate_distance_between_foot_and_center_of_mass()
        if distance_between_foot_and_center_of_mass == -1:
            return np.array([-1,-1])
        # find the center of mass
        intersections = get_intersections(self.left_leg_state.foot_position,
                                          distance_between_foot_and_center_of_mass,
                                          self.right_leg_state.foot_position,
                                          distance_between_foot_and_center_of_mass)
        if intersections is None:
            return None
        return max(intersections, key=lambda x: x[1])
    
    #upon straighten up, we need to update the hips positions
    def update_hips(self, center_of_mass: np.array) -> None:
        distance_between_foot_and_center_of_mass = self.calculate_distance_between_foot_and_center_of_mass()
        
        ratio_to_hip = leg_state.LEG_LENGTH / distance_between_foot_and_center_of_mass
        
        #left leg hip
        left_leg_vector = center_of_mass - self.left_leg_state.foot_position        
        self.left_leg_state.hip_position = left_leg_vector * ratio_to_hip + self.left_leg_state.foot_position

        # right leg hip
        right_leg_vector = center_of_mass - self.right_leg_state.foot_position
        self.right_leg_state.hip_position = right_leg_vector * ratio_to_hip + self.right_leg_state.foot_position
        
    def update_knees(self, center_of_mass: np.array) -> None:
        distance_between_foot_and_center_of_mass = self.calculate_distance_between_foot_and_center_of_mass()

        distance_between_foot_to_knee = np.linalg.norm(self.left_leg_state.knee_position - self.left_leg_state.foot_position)

        ratio_to_knee = distance_between_foot_to_knee / distance_between_foot_and_center_of_mass
        
        #left leg knee
        left_leg_vector = center_of_mass - self.left_leg_state.foot_position
        self.left_leg_state.knee_position = left_leg_vector * ratio_to_knee + self.left_leg_state.foot_position
        
        # right leg knee
        right_leg_vector = center_of_mass - self.right_leg_state.foot_position
        self.right_leg_state.knee_position = right_leg_vector * ratio_to_knee + self.right_leg_state.foot_position

    # def update_hips_and_knees_special_case1(self) -> None:
    #     m = (self.left_leg_state.foot_position[1] - self.right_leg_state.foot_position[1]) / (
    #             self.left_leg_state.foot_position[0] - self.right_leg_state.foot_position[0])
    #     m_ortogonal = -1 / m
    #     p_left = Point(self.left_leg_state.foot_position[0], self.left_leg_state.foot_position[1])
    #     l_left = LineString([(self.left_leg_state.foot_position[0], self.left_leg_state.foot_position[1]), (0, -m_ortogonal*self.left_leg_state.foot_position[0] + self.left_leg_state.foot_position[1])])
        
    #     c = p_left.buffer(LEG_LENGTH).boundary
    #     i = c.intersection(l_left)
    #     if i.geoms[0].coords[0][1] < i.geoms[1].coords[0][1]:
    #         self.left_leg_state.hip_position = np.array(i.geoms[0].coords[0])
    #     else:
    #         self.left_leg_state.hip_position = np.array(i.geoms[1].coords[0])
        
    #     c = p_left.buffer(np.linalg.norm(self.left_leg_state.knee_position - self.left_leg_state.foot_position)).boundary
    #     i = c.intersection(l_left)
    #     if i.geoms[0].coords[0][1] < i.geoms[1].coords[0][1]:
    #         self.left_leg_state.knee_position = np.array(i.geoms[0].coords[0])
    #     else:
    #         self.left_leg_state.knee_position = np.array(i.geoms[1].coords[0])

    #     p_right = Point(self.right_leg_state.foot_position[0], self.right_leg_state.foot_position[1])
    #     l_right = LineString([(self.right_leg_state.foot_position[0], self.right_leg_state.foot_position[1]), (0, -m_ortogonal*self.right_leg_state.foot_position[0] + self.right_leg_state.foot_position[1])])

    #     c = p_right.buffer(LEG_LENGTH).boundary
    #     i = c.intersection(l_right)
    #     if i.geoms[0].coords[0][1] < i.geoms[1].coords[0][1]:
    #         self.right_leg_state.hip_position = np.array(i.geoms[0].coords[0])
    #     else:
    #         self.right_leg_state.hip_position = np.array(i.geoms[1].coords[0])

    #     c = p_right.buffer(np.linalg.norm(self.right_leg_state.knee_position - self.right_leg_state.foot_position)).boundary
    #     i = c.intersection(l_right)
    #     if i.geoms[0].coords[0][1] < i.geoms[1].coords[0][1]:
    #         self.right_leg_state.knee_position = np.array(i.geoms[0].coords[0])
    #     else:
    #         self.right_leg_state.knee_position = np.array(i.geoms[1].coords[0])
        
# j = {
#     "left_arm": {
#       "hand_position": [
#         5,
#         17
#       ],
#       "shoulder_position": [
#         4.000000000000001,
#         15.975335199598758
#       ]
#     },
#     "right_arm": {
#       "hand_position": [
#         5,
#         17
#       ],
#       "shoulder_position": [
#         5.999999999999999,
#         15.975335199598758
#       ]
#     },
#     "left_leg": {
#       "foot_position": [
#         7,
#         13
#       ],
#       "knee_position": [
#         5.600589463506928,
#         14.450568905758072
#       ],
#       "hip_position": [
#         4.000000000000001,
#         14.475335199598758
#       ]
#     },
#     "right_leg": {
#       "foot_position": [
#         9,
#         13
#       ],
#       "knee_position": [
#         7.600589463506926,
#         14.45056890575807
#       ],
#       "hip_position": [
#         5.999999999999999,
#         14.475335199598758
#       ]
#     }
# }

# leg_state.set_leg_length(Hold(np.array(j["right_leg"]["foot_position"])), np.array(j["right_leg"]["knee_position"]), np.array(j["right_leg"]["hip_position"]))
# arm_state.set_arm_length(np.float64(2.5))
# set_leg_spread(np.float64(2.0943951))
# right_leg_state = LegState(Side.RIGHT, Hold(np.array(j["right_leg"]["foot_position"])), np.array(j["right_leg"]["knee_position"]), np.array(j["right_leg"]["hip_position"]))
# left_leg_state = LegState(Side.LEFT, Hold(np.array(j["left_leg"]["foot_position"])), np.array(j["left_leg"]["knee_position"]), np.array(j["left_leg"]["hip_position"]))
# right_arm_state = ArmState(Side.RIGHT, Hold(np.array(j["right_arm"]["hand_position"])), np.array(j["right_arm"]["shoulder_position"]))
# left_arm_state = ArmState(Side.LEFT, Hold(np.array(j["left_arm"]["hand_position"])), np.array(j["left_arm"]["shoulder_position"]))
# human_state = HumanState(left_leg_state, right_leg_state, left_arm_state, right_arm_state)
# print(human_state.straighten_up())