from collections import deque
import heapq
import time

graph = {
    'A': {'B': 1, 'D': 4},
    'B': {'A': 1, 'C': 3},
    'C': {'B': 3, 'E': 2},
    'D': {'A': 4, 'E': 2},
    'E': {'D': 2, 'C': 2}
}

packages = {
    'P1': 'B',
    'P2': 'D',
    'P3': 'E'
}

start_state = ('A', (False, False, False)) 
goal_status = (True, True, True)


def is_goal(state):
    return state[1] == goal_status


def get_neighbors(state):
    location, delivered = state
    neighbors = []

   
    for nxt, cost in graph[location].items():
        neighbors.append(((nxt, delivered), cost))

    new_delivered = list(delivered)
    for idx, (_, destination) in enumerate(packages.items()):
        if not new_delivered[idx] and location == destination:
            new_delivered[idx] = True
            neighbors.append(((location, tuple(new_delivered)), 0))

    return neighbors


def bfs():
    queue = deque([(start_state, [])])
    visited = set()

    nodes_expanded = 0
    start_time = time.time()

    while queue:
        state, path = queue.popleft()
        nodes_expanded += 1

        if is_goal(state):
            return path, nodes_expanded, time.time() - start_time

        if state in visited:
            continue

        visited.add(state)

        for nxt, _ in get_neighbors(state):
            queue.append((nxt, path + [nxt]))

    return None


def heuristic(state):
    """Heuristic based on min distance to an undelivered destination."""
    location, delivered = state
    remaining = [destination for i, (_, destination) in enumerate(packages.items()) if not delivered[i]]

    if not remaining:
        return 0

    return min(graph[location].get(dest, 3) for dest in remaining) + len(remaining)


def astar():
    pq = []
    heapq.heappush(pq, (0, start_state, []))
    visited = set()

    g_cost = {start_state: 0}
    nodes_expanded = 0
    start_time = time.time()

    while pq:
        cost, state, path = heapq.heappop(pq)
        nodes_expanded += 1

        if is_goal(state):
            return path, nodes_expanded, time.time() - start_time

        if state in visited:
            continue

        visited.add(state)

        for nxt, step_cost in get_neighbors(state):
            new_cost = g_cost[state] + step_cost

            if nxt not in g_cost or new_cost < g_cost[nxt]:
                g_cost[nxt] = new_cost
                f_cost = new_cost + heuristic(nxt)
                heapq.heappush(pq, (f_cost, nxt, path + [nxt]))

    return None



def run():
    bfs_path, bfs_nodes, bfs_time = bfs()
    a_path, a_nodes, a_time = astar()

    print("=== RESULTS ===")
    print("\n--- BFS ---")
    print("Path:", bfs_path)
    print("Nodes expanded:", bfs_nodes)
    print("Execution time:", round(bfs_time, 5))

    print("\n--- A* ---")
    print("Path:", a_path)
    print("Nodes expanded:", a_nodes)
    print("Execution time:", round(a_time, 5))


if __name__ == "__main__":
    run()
