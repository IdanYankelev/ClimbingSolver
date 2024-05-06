import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import PillowWriter, FuncAnimation
import json
import math
from utils import get_intersections
# # Define the Hold class
# class Hold:
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y

# # Create the wall holds
# wall_holds = [Hold(x, y) for x in range(1, 11, 2) for y in range(1, 21, 2)]

# climber_path = [
#     {'LH': [1, 7], 'RH': [5, 7], 'LF': [3, 1], 'RF': [5, 1]},
#     {'LH': [1, 7], 'RH': [5, 7], 'LF': [3, 1], 'RF': [5, 3]},
#     {'LH': [1, 7], 'RH': [1, 7], 'LF': [3, 3], 'RF': [5, 3]},
#     {'LH': [1, 7], 'RH': [1, 7], 'LF': [3, 3], 'RF': [5, 5]},
#     {'LH': [1, 7], 'RH': [1, 7], 'LF': [3, 5], 'RF': [5, 5]},
#     {'LH': [1, 7], 'RH': [1, 7], 'LF': [3, 5], 'RF': [5, 7]},
#     {'LH': [1, 7], 'RH': [3, 13], 'LF': [3, 5], 'RF': [5, 7]},
#     {'LH': [1, 7], 'RH': [3, 13], 'LF': [1, 7], 'RF': [5, 7]},
#     {'LH': [3, 13], 'RH': [3, 13], 'LF': [3, 9], 'RF': [5, 7]},
#     {'LH': [3, 13], 'RH': [3, 13], 'LF': [3, 9], 'RF': [5, 9]},
#     {'LH': [3, 13], 'RH': [3, 13], 'LF': [3, 9], 'RF': [7, 11]},
#     {'LH': [3, 13], 'RH': [5, 17], 'LF': [3, 9], 'RF': [7, 11]},
#     {'LH': [3, 13], 'RH': [5, 17], 'LF': [3, 11], 'RF': [7, 11]},
#     {'LH': [5, 17], 'RH': [5, 17], 'LF': [3, 11], 'RF': [9, 13]},
#     {'LH': [5, 17], 'RH': [5, 17], 'LF': [7, 13], 'RF': [9, 13]},
#     {'LH': [5, 17], 'RH': [9, 19], 'LF': [7, 13], 'RF': [9, 13]},
#     {'LH': [9, 19], 'RH': [9, 19], 'LF': [7, 13], 'RF': [9, 13]}
# ]
# def plot_frame(step):
#     plt.clf()
#     for hold in wall_holds:
#         plt.plot(hold.x, hold.y, 'ko', markersize=10)  # Wall holds as black circles
#     current_positions = climber_path[step]
#     for limb, position in current_positions.items():
#         plt.plot(position[0], position[1], 'ro', markersize=15)  # Limb positions as red circles
#     plt.xlim(0, 12)
#     plt.ylim(0, 22)
#     plt.gca().set_aspect('equal', adjustable='box')
#     #make large title size with the text Step {step+1}
#     plt.title(f"Step {step+1}", fontsize=24)
    
    



# # Create the animation
# fig = plt.figure(figsize=(8, 16))
# ani = FuncAnimation(fig, plot_frame, frames=len(climber_path), interval=1000) # 1000ms = 1s per frame

# # Save the animation as a GIF
# ani.save('climbing_animation.gif', writer=PillowWriter(fps=1))

# print("GIF has been created.")


# Extended path with more detailed positions for demonstration
# extended_path_json = [
#     {
#         "left_arm": {"hand_position": [1, 7], "shoulder_position": [1, 6]},
#         "right_arm": {"hand_position": [5, 7], "shoulder_position": [5, 6]},
#         "left_leg": {"foot_position": [3, 1], "knee_position": [1.5, 2], "hip_position": [3, 3]},
#         "right_leg": {"foot_position": [5, 1], "knee_position": [3.5, 2], "hip_position": [5, 3]}
#     },
#     # Adding more steps with interpolated or assumed positions
#     {
#         "left_arm": {"hand_position": [2, 9], "shoulder_position": [2, 8]},
#         "right_arm": {"hand_position": [4, 9], "shoulder_position": [4, 8]},
#         "left_leg": {"foot_position": [3, 4], "knee_position": [3, 5], "hip_position": [3, 6]},
#         "right_leg": {"foot_position": [5, 4], "knee_position": [5, 5], "hip_position": [5, 6]}
#     },
#     {
#         "left_arm": {"hand_position": [3, 11], "shoulder_position": [3, 10]},
#         "right_arm": {"hand_position": [3, 11], "shoulder_position": [3, 10]},
#         "left_leg": {"foot_position": [4, 7], "knee_position": [4, 8], "hip_position": [4, 9]},
#         "right_leg": {"foot_position": [6, 7], "knee_position": [6, 8], "hip_position": [6, 9]}
#     },
#     # Continue with more detailed steps as needed...
# ]
# To enhance the visualization with large dots at the end of each line (for right foot, right knee, right hip, etc.),
# we'll modify the plotting function to include these dots for each significant point (hand, shoulder, foot, knee, hip).

# Adjusted function to plot a single frame with distinct colors for each connection and large dots at key points

extended_path_json = json.load(open("C:\\Users\\Shust\\OneDrive\\Documents\\Second Degree\\coursses\\Research methods\\Code\\RM-Climbing\\climber_path.json"))

def get_elbow_position(shoulder: np.array, hand: np.array, arm_length: np.float64):
    distance_hand_elbow = 0.5 * arm_length
    distance_shoulder_elbow = 0.5 * arm_length

    intersections = get_intersections(shoulder, distance_shoulder_elbow, hand, distance_hand_elbow)
    return max(intersections, key=lambda x: x[1])


def plot_frame_with_full_connections(step, ax):
    human_state = extended_path_json[step]
    ax.clear()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 20)
    
    # Plotting connections for the arms with distinct colors and large dots
    left_hand_pos = human_state["left_arm"]["hand_position"]
    left_shoulder_pos = human_state["left_arm"]["shoulder_position"]
    right_hand_pos = human_state["right_arm"]["hand_position"]
    right_shoulder_pos = human_state["right_arm"]["shoulder_position"]
    
    
    left_elbow = get_elbow_position(np.array(left_shoulder_pos), np.array(left_hand_pos), 2.5)
    right_elbow = get_elbow_position(np.array(right_shoulder_pos), np.array(right_hand_pos), 2.5)
    
    ax.plot([left_hand_pos[0], left_elbow[0]], [left_hand_pos[1], left_elbow[1]], 'm-', linewidth=4)
    ax.plot([right_hand_pos[0], right_elbow[0]], [right_hand_pos[1], right_elbow[1]], 'c-', linewidth=4)
    
    ax.plot([left_elbow[0], left_shoulder_pos[0]], [left_elbow[1], left_shoulder_pos[1]], 'm-', linewidth=5)
    ax.plot([right_elbow[0], right_shoulder_pos[0]], [right_elbow[1], right_shoulder_pos[1]], 'c-', linewidth=5)
    
    
    # # Connect left hand to left shoulder and right hand to right shoulder
    # ax.plot([left_hand_pos[0], left_shoulder_pos[0]], [left_hand_pos[1], left_shoulder_pos[1]], 'm-', linewidth=2)
    # ax.plot([right_hand_pos[0], right_shoulder_pos[0]], [right_hand_pos[1], right_shoulder_pos[1]], 'c-', linewidth=2)
    
    # shoulder vector
    shoulder_vector = np.array(right_shoulder_pos) - np.array(left_shoulder_pos)
    mid_point_left = np.array(left_shoulder_pos) + shoulder_vector / 3
    mid_point_right = np.array(left_shoulder_pos) + 2 * shoulder_vector / 3
    shoulder_vector = shoulder_vector / np.linalg.norm(shoulder_vector)
    # find the perpendicular vector
    perpendicular_vectors = [np.array([shoulder_vector[1], -shoulder_vector[0]]), np.array([-shoulder_vector[1], shoulder_vector[0]])]
    
    #shoulders perpendicular vector
    shoulders_perpendicular_vector = max(perpendicular_vectors, key=lambda x: x[1])
    mid_point_left = mid_point_left + shoulders_perpendicular_vector * 0.3
    mid_point_right = mid_point_right + shoulders_perpendicular_vector * 0.3

    mid_points_vector = mid_point_right - mid_point_left

    
    # find the middle point
    middle_point = (np.array(mid_point_left) + np.array(mid_point_right)) / 2
    
    #head position
    head_position = middle_point + shoulders_perpendicular_vector * 0.75
    #Plot the head
    ax.plot(head_position[0], head_position[1], 'ro', markersize=28)
    
    #Plot the neck, from the middle point to the head
    ax.plot([middle_point[0], head_position[0]], [middle_point[1], head_position[1]], 'r-', linewidth=6)
    
    
    # Plotting large dots for hands and shoulders and mids points
    ax.plot(left_hand_pos[0], left_hand_pos[1], 'mo', markersize=10)
    ax.plot(left_shoulder_pos[0], left_shoulder_pos[1], 'mo', markersize=10)
    ax.plot(right_hand_pos[0], right_hand_pos[1], 'co', markersize=10)
    ax.plot(right_shoulder_pos[0], right_shoulder_pos[1], 'co', markersize=10)


    # Plotting dots for the elbows
    ax.plot(left_elbow[0], left_elbow[1], 'mo', markersize=10)
    ax.plot(right_elbow[0], right_elbow[1], 'co', markersize=10)
    
    # Connect left shoulder to mid point left and right shoulder to mid point right
    ax.plot([left_shoulder_pos[0], mid_point_left[0]], [left_shoulder_pos[1], mid_point_left[1]], 'k-', linewidth=4)
    ax.plot([right_shoulder_pos[0], mid_point_right[0]], [right_shoulder_pos[1], mid_point_right[1]], 'k-', linewidth=4)

    # Connect mid point left to mid point right
    ax.plot([mid_point_left[0], mid_point_right[0]], [mid_point_left[1], mid_point_right[1]], 'k-', linewidth=4)
    # ax.plot([left_shoulder_pos[0], right_shoulder_pos[0]], [left_shoulder_pos[1], right_shoulder_pos[1]], 'k-', linewidth=4)

    # Connect shoulders to hips left side
    ax.plot([left_shoulder_pos[0], human_state["left_leg"]["hip_position"][0]], [left_shoulder_pos[1], human_state["left_leg"]["hip_position"][1]], 'k-', linewidth=4)
    
    # Connect shoulders to hips right side
    ax.plot([right_shoulder_pos[0], human_state["right_leg"]["hip_position"][0]], [right_shoulder_pos[1], human_state["right_leg"]["hip_position"][1]], 'k-', linewidth=4)
    
    # Plotting connections for the legs with distinct colors and large dots
    for leg, color in (("left_leg", "b"), ("right_leg", "y")):
        foot_pos = human_state[leg]["foot_position"]
        knee_pos = human_state[leg]["knee_position"]
        hip_pos = human_state[leg]["hip_position"]
        
        # Connect foot to knee and knee to hip
        ax.plot([foot_pos[0], knee_pos[0]], [foot_pos[1], knee_pos[1]], f'{color}-', linewidth=6)
        ax.plot([knee_pos[0], hip_pos[0]], [knee_pos[1], hip_pos[1]], f'{color}-', linewidth=8)
        
        # Plot large dots for foot, knee, and hip
        ax.plot(foot_pos[0], foot_pos[1], f'{color}o', markersize=10)
        ax.plot(knee_pos[0], knee_pos[1], f'{color}o', markersize=10)
        ax.plot(hip_pos[0], hip_pos[1], f'{color}o', markersize=10)
    
    # Connect both hips
    left_hip_pos = human_state["left_leg"]["hip_position"]
    right_hip_pos = human_state["right_leg"]["hip_position"]
    ax.plot([left_hip_pos[0], right_hip_pos[0]], [left_hip_pos[1], right_hip_pos[1]], 'k-', linewidth=4)
    
    
    # Add the wall holds as black circles, of the set of right and left foot and hand positions
    for human_state in extended_path_json:
        for limb in ("left_arm", "right_arm", "left_leg", "right_leg"):
            position = human_state[limb]["hand_position"] if "arm" in limb else human_state[limb]["foot_position"]
            ax.plot(position[0], position[1], 'ko', markersize=10)
    
    # Add some high holds from the 
    wall_holds = [(x, y) for x in range(1, 11, 2) for y in range(1, 21, 2)]
    
    #sample some high holds
    np.random.seed(420)
    # Filter indices where the second element (y) is between 16 and 20
    filtered_indices1 = np.array([i for i, hold in enumerate(wall_holds) if 16 < hold[1] < 21])
    filtered_indices2 = np.array([i for i, hold in enumerate(wall_holds) if  (1 <= hold[0] <= 3 and 1 < hold[1] < 10)])

    # Sample 5 indices from the filtered indices
    sampled_indices1 = np.random.choice(filtered_indices1, 5, replace=False)
    sampled_indices2 = np.random.choice(filtered_indices2, 4, replace=False)

    # Retrieve the sampled wall_holds using the sampled indices
    sampled_holds1 = [wall_holds[i] for i in sampled_indices1]
    sampled_holds2 = [wall_holds[i] for i in sampled_indices2]

    for sampled_hold1 in sampled_holds1:
        ax.plot(sampled_hold1[0], sampled_hold1[1], 'ko', markersize=10)
    
    for sampled_hold2 in sampled_holds2:    
        ax.plot(sampled_hold2[0], sampled_hold2[1], 'ko', markersize=10)
    ax.set_aspect('equal')
    ax.set_title(f"Step {step+1}", fontsize=24)

# Create the animation with full connections
def create_animation(extended_path_json):
 
    fig, ax = plt.subplots(figsize=(5, 10))
    ani = FuncAnimation(fig, plot_frame_with_full_connections, frames=len(extended_path_json), fargs=(ax,))

    # Save the animation as a GIF with full connections
    full_connections_gif_path = "climber_movement_new.gif"
    ani.save(full_connections_gif_path, writer=PillowWriter(fps=1))

create_animation(extended_path_json)