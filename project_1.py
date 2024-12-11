import heapq
import time

class Puzzle:
    def __init__(self, state, parent=None, move=None, depth=0, cost=0):
        self.state = state  # Current state of the puzzle
        self.parent = parent  # Parent node
        self.move = move  # Move that led to this state
        self.depth = depth  # Depth of the node in the search tree
        self.cost = cost  # Path cost for UCS; g(n) for A*

    def __lt__(self, other):
        return self.cost < other.cost

    def generate_successors(self):
        def swap(state, i, j):
            new_state = list(state)
            new_state[i], new_state[j] = new_state[j], new_state[i]
            return tuple(new_state)

        successors = []
        blank = self.state.index(0)  # Find the blank tile (0 represents the blank)
        row, col = divmod(blank, 3)

        # Possible moves: up, down, left, right
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for dr, dc in moves:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < 3 and 0 <= new_col < 3:
                new_blank = new_row * 3 + new_col
                new_state = swap(self.state, blank, new_blank)
                successors.append(Puzzle(new_state, self, (dr, dc), self.depth + 1, self.cost + 1))

        return successors


def uniform_cost_search(start, goal):
    frontier = []
    heapq.heappush(frontier, (0, Puzzle(start)))
    explored = set()
    start_time = time.time()

    while frontier:
        _, current = heapq.heappop(frontier)
        
        if current.state == goal:
            end_time = time.time()
            return current, len(explored), end_time - start_time

        explored.add(current.state)

        for successor in current.generate_successors():
            if successor.state not in explored:
                heapq.heappush(frontier, (successor.cost, successor))

    return None, len(explored), time.time() - start_time


def a_star_search(start, goal):
    def heuristic(state, goal):
        # Manhattan distance
        distance = 0
        for i in range(1, 9):  # Skip the blank tile
            xi, yi = divmod(state.index(i), 3)
            xg, yg = divmod(goal.index(i), 3)
            distance += abs(xi - xg) + abs(yi - yg)
        return distance

    frontier = []
    heapq.heappush(frontier, (0, Puzzle(start)))
    explored = set()
    start_time = time.time()

    while frontier:
        _, current = heapq.heappop(frontier)

        if current.state == goal:
            end_time = time.time()
            return current, len(explored), end_time - start_time

        explored.add(current.state)

        for successor in current.generate_successors():
            if successor.state not in explored:
                h = heuristic(successor.state, goal)
                successor.cost += h  # f(n) = g(n) + h(n)
                heapq.heappush(frontier, (successor.cost, successor))

    return None, len(explored), time.time() - start_time


def print_solution(solution):
    path = []
    current = solution
    while current:
        path.append(current.state)
        current = current.parent

    path.reverse()
    for state in path:
        for i in range(0, 9, 3):
            print(state[i:i+3])
        print()


if __name__ == "__main__":
    start_state = (1, 2, 3, 4, 0, 5, 6, 7, 8)  # Initial state
    goal_state = (1, 2, 3, 4, 5, 6, 7, 8, 0)  # Goal state

    print("\n--- Uniform Cost Search ---")
    ucs_solution, ucs_explored, ucs_time = uniform_cost_search(start_state, goal_state)
    if ucs_solution:
        print("Solution found:")
        print_solution(ucs_solution)
        print(f"Nodes explored: {ucs_explored}")
        print(f"Time taken: {ucs_time:.4f} seconds")
    else:
        print("No solution found.")

    print("\n--- A* Search ---")
    astar_solution, astar_explored, astar_time = a_star_search(start_state, goal_state)
    if astar_solution:
        print("Solution found:")
        print_solution(astar_solution)
        print(f"Nodes explored: {astar_explored}")
        print(f"Time taken: {astar_time:.4f} seconds")
    else:
        print("No solution found.")
