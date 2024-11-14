import heapq

# Node class to represent each state
class Node:
    def __init__(self, state, parent=None, g_cost=0, h_cost=0):
        self.state = state
        self.parent = parent
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost

    def __lt__(self, other):
        return self.f_cost < other.f_cost

# Heuristic function (can be modified as needed)
def heuristic(current_state, goal_state):
    # Example heuristic: number of misplaced tiles (Manhattan distance can also be used)
    return sum(1 for c, g in zip(current_state, goal_state) if c != g and c != 0)

# Function to generate neighbors for the 8-puzzle game
def generate_neighbors(state):
    neighbors = []
    zero_index = state.index(0)  # Find the index of the blank (0)
    row, col = divmod(zero_index, 3)  # Convert index to (row, col)

    # Possible move directions: up, down, left, right
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for dr, dc in moves:
        new_row, new_col = row + dr, col + dc
        if 0 <= new_row < 3 and 0 <= new_col < 3:  # Check if the move is within bounds
            new_index = new_row * 3 + new_col
            new_state = state[:]
            new_state[zero_index], new_state[new_index] = new_state[new_index], new_state[zero_index]  # Swap
            neighbors.append((new_state, 1))  # Append neighbor with move cost of 1

    return neighbors

# A* algorithm implementation
def a_star_algorithm(initial_state, goal_state):
    open_list = []
    closed_list = set()
    start_node = Node(initial_state, g_cost=0, h_cost=heuristic(initial_state, goal_state))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        # Check if the goal state is reached
        if current_node.state == goal_state:
            path = []
            total_cost = current_node.g_cost
            while current_node:
                path.append(current_node.state)
                current_node = current_node.parent
            return path[::-1], total_cost  # Return the path in correct order

        closed_list.add(tuple(current_node.state))

        # Generate neighbors dynamically
        for neighbor, move_cost in generate_neighbors(current_node.state):
            if tuple(neighbor) in closed_list:
                continue

            g_cost = current_node.g_cost + move_cost
            h_cost = heuristic(neighbor, goal_state)
            neighbor_node = Node(neighbor, parent=current_node, g_cost=g_cost, h_cost=h_cost)

            heapq.heappush(open_list, neighbor_node)

    return None, None  # Return None if no path is found

# Read state from a file
def read_state_from_file(filename):
    try:
        with open(filename, 'r') as file:
            state = []
            for line in file:
                state.extend(map(int, line.strip().split(',')))
        return state
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        exit()
    except ValueError:
        print(f"Error: Invalid format in file '{filename}'. Ensure all values are integers.")
        exit()

# Function to print a state in 3x3 format
def print_state(state):
    for i in range(0, 9, 3):
        print(state[i:i+3])
    print()  # Print a blank line for better readability

# Get file input from the user
initial_state_file = input("Enter file location for initial state: ")
goal_state_file = input("Enter file location for goal state: ")

# Read initial and goal states from the specified files
initial_state = read_state_from_file(initial_state_file)
goal_state = read_state_from_file(goal_state_file)

# Print the initial and goal states for verification
print("Initial State:")
print_state(initial_state)
print("Goal State:")
print_state(goal_state)

# Run the updated A* algorithm
path, cost = a_star_algorithm(initial_state, goal_state)

# Output the result
if path:
    print("Path found:")
    for state in path:
        print_state(state)
    print("Total cost:", cost)
else:
    print("No path found.")
