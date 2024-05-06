import numpy as np
import json
from visualization import create_animation
from algorithm import a_star_algorithm
from arm_state import ArmState, set_arm_length
from hold import Hold
from human_state import HumanState, set_leg_spread
from leg_state import LegState, set_leg_length
from utils import Side
from search_node import Search_Node
if __name__ == '__main__':
    # Based on https://youtu.be/87RfgQMyd30?t=236
    # with cm units.
    wall_holds: list[Hold] = [
        Hold(x,y) for x in range(1, 11, 2) for y in range(1, 21, 2)
     ]
    # Define wall holds and goal state positions.
    goal_state_positions = [np.array([9,19]), np.array([9,19])]

    # Define starting left leg configuration.
    starting_left_foot_hold = Hold(3, 1)
    starting_left_knee_position = np.array([2,2.75])
    starting_left_hip_position = np.array([3, 4])

    # Define starting right leg configuration.
    starting_right_foot_hold = Hold(5, 1)
    starting_right_knee_position = np.array([6, 2.75])
    starting_right_hip_position = np.array([5, 4])

    # Define starting left arm configuration.
    starting_left_hand_hold = Hold(1, 7)
    starting_left_shoulder_position = np.array([3, 5.5])

    # Define starting right arm configuration.
    starting_right_hand_hold = Hold(5,7)
    starting_right_shoulder_position = np.array([5, 5.5])

    # Define initial human state based on starting body configurations.
    starting_left_leg_state = LegState(Side.LEFT, starting_left_foot_hold, starting_left_knee_position,
                                       starting_left_hip_position)
    starting_right_leg_state = LegState(Side.RIGHT, starting_right_foot_hold, starting_right_knee_position,
                                        starting_right_hip_position)
    starting_left_arm_state = ArmState(Side.LEFT, starting_left_hand_hold, starting_left_shoulder_position)
    starting_right_arm_state = ArmState(Side.RIGHT, starting_right_hand_hold, starting_right_shoulder_position)
    starting_human_state = HumanState(starting_left_leg_state, starting_right_leg_state, starting_left_arm_state,
                                      starting_right_arm_state)

    # Define body physique constants.
    set_leg_spread(np.float_(np.float64(2.0943951)))
    set_arm_length(np.float_(np.float64(2.5)))
    set_leg_length(starting_right_foot_hold, starting_right_knee_position, starting_right_hip_position)
    # set_length_from_feet_to_center(starting_left_leg_state, starting_right_leg_state)

    # Search path using a* algorithm.
    path:list[Search_Node] | None = a_star_algorithm(starting_human_state, goal_state_positions, wall_holds)

    # If path exists, print the solution for the climbing problem.
    # Simple print of the path.
    if path:
        print("Path found!")
        for node in path:
            print(
                f" LH: {node.human_state.left_arm_state.hand_position},"
                f" RH: {node.human_state.right_arm_state.hand_position},"
                f" LF: {node.human_state.left_leg_state.foot_position},"
                f" RF: {node.human_state.right_leg_state.foot_position}")
    else:
        print("No path found.")
    
    # full detailed print of the path
    path_json = []
    for node in path:
        human_state = {
            "left_arm": {
                "hand_position": node.human_state.left_arm_state.hand_position.tolist(),
                "shoulder_position": node.human_state.left_arm_state.shoulder_position.tolist(),
            },
            "right_arm": {
                "hand_position": node.human_state.right_arm_state.hand_position.tolist(),
                "shoulder_position": node.human_state.right_arm_state.shoulder_position.tolist(),
            },
            "left_leg": {
                "foot_position": node.human_state.left_leg_state.foot_position.tolist(),
                "knee_position": node.human_state.left_leg_state.knee_position.tolist(),
                "hip_position": node.human_state.left_leg_state.hip_position.tolist(),
            },
            "right_leg": {
                "foot_position": node.human_state.right_leg_state.foot_position.tolist(),
                "knee_position": node.human_state.right_leg_state.knee_position.tolist(),
                "hip_position": node.human_state.right_leg_state.hip_position.tolist(),
            }
        }
        path_json.append(human_state)

    # Convert to JSON string (you can also write this to a file if needed)
    path_json_str = json.dumps(path_json, indent=2)
    print(path_json_str)
    
    # save the path to a json file
    with open("climber_path.json", "w") as file:
        file.write(path_json_str)
    print("Path has been saved to climber_path.json.")
    
    # Create the animation
    #create_animation(path_json)
        
    
