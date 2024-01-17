import heapq

class Node:
    def __init__(self, x, y, cost, heuristic):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

def heuristic(node, goal):
    return abs(node.x - goal[0]) + abs(node.y - goal[1])

def is_valid(x, y, grid):
    return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != "obstacle"

def get_neighbors(node, grid, goal):
    neighbors = []
    moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Possible moves: right, left, down, up

    for move in moves:
        new_x, new_y = node.x + move[0], node.y + move[1]
        if is_valid(new_x, new_y, grid):
            cost = node.cost + 1  # Assuming equal cost for all moves
            neighbors.append(Node(new_x, new_y, cost, heuristic(Node(new_x, new_y, 0, 0), goal)))

    return neighbors

def a_star(grid, start, goal):
    start_node = Node(start[0], start[1], 0, heuristic(Node(start[0], start[1], 0, 0), goal))
    start_node.parent = None  # Set the parent attribute for the start node
    goal_node = Node(goal[0], goal[1], 0, 0)

    open_set = [start_node]
    closed_set = set()

    while open_set:
        current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) == (goal[0], goal[1]):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add((current_node.x, current_node.y))

        for neighbor in get_neighbors(current_node, grid, goal):
            if (neighbor.x, neighbor.y) not in closed_set:
                neighbor.parent = current_node
                heapq.heappush(open_set, neighbor)

    return None  # No path found


def main():
    grid = [
        ["empty", "empty", "empty", "empty", "empty"],
        ["empty", "obstacle", "empty", "obstacle", "empty"],
        ["empty", "empty", "trash", "empty", "empty"],
        ["empty", "obstacle", "empty", "obstacle", "empty"],
        ["empty", "empty", "empty", "empty", "empty"]
    ]

    start = (0, 0)
    trash_location = [(2, 2)]  # You can have multiple trash locations
    goal = trash_location[0]

    for trash in trash_location:
        path = a_star(grid, start, trash)
        if path:
            print(f"Path to trash at {trash}: {path}")
            start = trash  # Update the start position to the trash location
        else:
            print(f"No path to trash at {trash}")

if __name__ == "__main__":
    main()
