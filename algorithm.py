import heapq
import numpy as np
from search_node import Search_Node


def create_open_set():
    """
    Creates heap for maintaining order by f and dict for fast recovery.
    """
    return [], {}  # heap and dict


def create_closed_set():
    """
    Creates dict for fast recovery.
    """
    return {}


def add_to_open(node, open_set):
    """
    Adds new node to the open_set.
    """
    heapq.heappush(open_set[0], node)
    open_set[1][node.human_state] = node


def add_to_closed(node, closed_set):
    """
    Adds new node to the close_set.
    """
    closed_set[node.human_state] = node


def get_best(open_set):
    """
    Returns the node with the lowest f-score from the open_set.
    """
    while open_set[0]:
        best_node = heapq.heappop(open_set[0])
        if best_node.human_state in open_set[1]:
            open_set[1].pop(best_node.human_state)
            return best_node
    return None


def duplicate_in_open(state, open_set):
    """
    returns False if curr_neighbor state not in open_set or has a lower g from the node in open_set
    remove the node with the higher g from open_set (if exists).
    """
    if state in open_set[1]:
        existing_state = open_set[1][state]
        if existing_state.g > state.g:
            open_set[1].pop(existing_state)
            return False
        else:
            return True
    return False


def duplicate_in_closed(state, closed_set):
    """
    returns False if curr_neighbor state not in closed_set or has a lower g from the node in closed_set
    remove the node with the higher g from closed_set (if exists)
    """
    if state in closed_set:
        existing_state = closed_set[state]
        if existing_state.g > state.g:
            closed_set.pop(existing_state)
            return False
        else:
            return True
    return False

def calculate_heuristic(node: Search_Node, goal_state_positions):
    """
    Calculates heuristic for the node base on ...
    """
    return np.linalg.norm(node.human_state.left_arm_state.hand_position - goal_state_positions[0]) / node.human_state.get_arm_length() + \
        np.linalg.norm(node.human_state.right_arm_state.hand_position - goal_state_positions[1]) / node.human_state.get_arm_length() 


def a_star_algorithm(starting_human_state, goal_state_positions, wall_holds, calculate_heuristic=calculate_heuristic, generated_nodes=[]) -> list[Search_Node] | None:
    """
    Applies the A* search algorithm to find solution for the climbing problem.
    """
    # Initialize open_set and closed_set.
    open_set = create_open_set()
    closed_set = create_closed_set()

    # Initialize initial state.
    initial_node = Search_Node(starting_human_state, g=0, h=0)
    initial_node.h = calculate_heuristic(initial_node, goal_state_positions)
    # Add initial_state to open_set to start the search process.
    add_to_open(initial_node, open_set)

    # While heap is not empty, search for goal state nodes.
    while len(open_set[0]) > 0:
        # Get node with minimal f from the open_set.
        current_node: Search_Node = get_best(open_set)
        # If no node found, break the loop.
        if current_node is None:
            break
        # Check if current_node depicts a goal state.
        if is_goal_state(current_node, goal_state_positions):
            return reconstruct_path(current_node)

        # Add current node to the close_set.
        add_to_closed(current_node, closed_set)

        # Add all possible continuation states (by moving a single arm\leg) if not explored yet.
        generated_nodes.append(current_node)
        for new_state in current_node.generate_possible_moves(wall_holds):
            # if new_state already in closed_set, no need to create search node.
            if new_state in closed_set:
                continue

            # Create new search node and calculate its heuristic.
            new_node: Search_Node = Search_Node(new_state, current_node.g + 1, 0, prev=current_node)
            new_node.h = calculate_heuristic(new_node, goal_state_positions)

            # Add new node to open_set only if it does not exist already
            # in open_set/close_set.
            if not duplicate_in_open(new_node, open_set) and \
                    not duplicate_in_closed(new_node, closed_set):
                add_to_open(new_node, open_set)

    return None  # Path not found

def is_goal_state(node: Search_Node, goal_state_positions):
    """
    Checks if the hand positions in the depicted state are the goal positions.
    """
    left_hand_position = node.human_state.left_arm_state.hand_position
    right_hand_position = node.human_state.right_arm_state.hand_position
    return left_hand_position[0] == goal_state_positions[0][0] and \
        left_hand_position[1] == goal_state_positions[0][1] and \
        right_hand_position[0] == goal_state_positions[1][0] and \
        right_hand_position[1] == goal_state_positions[1][1]


def reconstruct_path(node: Search_Node):
    """
    Reconstruct path from state to start
    """
    path = []
    while node is not None:
        path.append(node)
        node: Search_Node = node.prev
    path.reverse()
    return path
