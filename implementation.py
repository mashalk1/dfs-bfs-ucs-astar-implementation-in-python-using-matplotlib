import networkx as nx
import matplotlib.pyplot as plt
import heapq
from collections import deque

# Define nodes and create the graph
nodes = ['A', 'B', 'C', 'D', 'E', 'F', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'H', 'G', 'R', 'Q', 'S']
G = nx.Graph()
G.add_nodes_from(nodes)

# Define edges with weights
edges_with_weights = [
    ('A', 'B', 10), ('A', 'C', 20), ('B', 'D', 15),
    ('C', 'E', 25), ('C', 'F', 30), ('F', 'I', 12),
    ('I', 'L', 35), ('L', 'P', 40), ('P', 'O', 22),
    ('E', 'H', 18), ('H', 'K', 28), ('K', 'N', 45),
    ('N', 'O', 50), ('D', 'G', 17), ('G', 'J', 8),
    ('J', 'M', 33), ('M', 'O', 27), ('O', 'R', 14),
    ('O', 'Q', 9), ('Q', 'S', 19), ('R', 'S', 23)
]
G.add_weighted_edges_from(edges_with_weights)

# Define static obstacle edges
obstacle_edges = [
    ('A', 'D', 40), ('B', 'E', 35), ('C', 'G', 30),
    ('F', 'H', 25), ('I', 'J', 45), ('L', 'K', 50),
    ('M', 'P', 15), ('N', 'Q', 20), ('O', 'S', 12),
    ('R', 'P', 18)
]
G.add_weighted_edges_from(obstacle_edges)

# Define dynamic obstacle edges
dynamic_obstacle_edges = [
    ('A', 'E'), ('B', 'F'), ('C', 'H'), ('D', 'I'), ('J', 'M')
]

# Precompute node positions for consistent layout
pos = nx.spring_layout(G, k=0.5, iterations=50)

# Function to draw the graph with obstacles and paths
def draw_graph_with_obstacles(ax, dynamic_obstacles_on, highlight_edges=None):
    ax.clear()
    # Draw nodes
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=300, font_size=10, ax=ax)
    # Draw normal edges
    normal_edges = [edge for edge in G.edges() if edge not in obstacle_edges and edge not in dynamic_obstacle_edges]
    nx.draw_networkx_edges(G, pos, edgelist=normal_edges, edge_color='black', ax=ax)
    # Draw static obstacle edges
    nx.draw_networkx_edges(G, pos, edgelist=obstacle_edges, edge_color='red', width=2, ax=ax)
    # Draw dynamic obstacle edges if they are on
    if dynamic_obstacles_on:
        nx.draw_networkx_edges(G, pos, edgelist=dynamic_obstacle_edges, edge_color='green', style='dashed', width=2, ax=ax)
    # Draw edge labels
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, ax=ax)
    # Highlight specific edges if provided
    if highlight_edges:
        nx.draw_networkx_edges(G, pos, edgelist=highlight_edges, edge_color='blue', width=2, style='dotted', ax=ax)

# Depth-First Search (DFS) algorithm
def dfs(G, start, target):
    stack = [(start, [start])]
    visited = set()
    obstacle_paths = set()
    while stack:
        (vertex, path) = stack.pop()
        if vertex in visited:
            continue
        visited.add(vertex)
        for neighbor in G.neighbors(vertex):
            if neighbor in visited:
                continue
            edge = (vertex, neighbor) if (vertex, neighbor) in G.edges() else (neighbor, vertex)
            if edge in obstacle_edges:
                obstacle_paths.add(edge)
                continue
            if neighbor == target:
                return path + [neighbor], obstacle_paths
            stack.append((neighbor, path + [neighbor]))
    return None, obstacle_paths

# Breadth-First Search (BFS) algorithm
def bfs(G, start, target):
    queue = deque([(start, [start])])
    visited = set()
    obstacle_paths = set()
    while queue:
        (vertex, path) = queue.popleft()
        if vertex in visited:
            continue
        visited.add(vertex)
        for neighbor in G.neighbors(vertex):
            if neighbor in visited:
                continue
            edge = (vertex, neighbor) if (vertex, neighbor) in G.edges() else (neighbor, vertex)
            if edge in obstacle_edges:
                obstacle_paths.add(edge)
                continue
            if neighbor == target:
                return path + [neighbor], obstacle_paths
            queue.append((neighbor, path + [neighbor]))
    return None, obstacle_paths

# A* Search algorithm
def a_star(G, start, target):
    def heuristic(a, b):
        return abs(ord(a) - ord(b))
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, target), 0, start, []))
    visited = set()
    obstacle_paths = set()
    while open_set:
        _, cost, vertex, path = heapq.heappop(open_set)
        if vertex in visited:
            continue
        visited.add(vertex)
        new_path = path + [vertex]
        if vertex == target:
            return new_path, cost, obstacle_paths
        for neighbor in G.neighbors(vertex):
            if neighbor in visited:
                continue
            edge = (vertex, neighbor) if (vertex, neighbor) in G.edges() else (neighbor, vertex)
            if edge in obstacle_edges:
                obstacle_paths.add(edge)
                continue
            total_cost = cost + G[vertex][neighbor]['weight']
            heapq.heappush(open_set, (total_cost + heuristic(neighbor, target), total_cost, neighbor, new_path))
    return None, None, obstacle_paths

# Uniform Cost Search (UCS) algorithm
def ucs(G, start, target):
    open_set = []
    heapq.heappush(open_set, (0, start, []))
    visited = set()
    obstacle_paths = set()
    while open_set:
        cost, vertex, path = heapq.heappop(open_set)
        if vertex in visited:
            continue
        visited.add(vertex)
        new_path = path + [vertex]
        if vertex == target:
            return new_path, cost, obstacle_paths
        for neighbor in G.neighbors(vertex):
            if neighbor in visited:
                continue
            edge = (vertex, neighbor) if (vertex, neighbor) in G.edges() else (neighbor, vertex)
            if edge in obstacle_edges:
                obstacle_paths.add(edge)
                continue
            total_cost = cost + G[vertex][neighbor]['weight']
            heapq.heappush(open_set, (total_cost, neighbor, new_path))
    return None, None, obstacle_paths

# Run all search algorithms and return their results
def run_all_searches(start, target):
    results = {}
    path_dfs, obstacle_paths_dfs = dfs(G, start, target)
    cost_dfs = sum(G[path_dfs[i]][path_dfs[i+1]]['weight'] for i in range(len(path_dfs) - 1)) if path_dfs else None
    results['dfs'] = {'path': path_dfs, 'cost': cost_dfs, 'obstacles': obstacle_paths_dfs}
    path_bfs, obstacle_paths_bfs = bfs(G, start, target)
    cost_bfs = sum(G[path_bfs[i]][path_bfs[i+1]]['weight'] for i in range(len(path_bfs) - 1)) if path_bfs else None
    results['bfs'] = {'path': path_bfs, 'cost': cost_bfs, 'obstacles': obstacle_paths_bfs}
    path_a_star, cost_a_star, obstacle_paths_a_star = a_star(G, start, target)
    results['a_star'] = {'path': path_a_star, 'cost': cost_a_star, 'obstacles': obstacle_paths_a_star}
    path_ucs, cost_ucs, obstacle_paths_ucs = ucs(G, start, target)
    results['ucs'] = {'path': path_ucs, 'cost': cost_ucs, 'obstacles': obstacle_paths_ucs}
    return results

# Visualize all results and paths in subplots
def visualize_all(start, target):
    fig, axs = plt.subplots(3, 2, figsize=(15, 15))
    results = run_all_searches(start, target)
    draw_graph_with_obstacles(axs[0, 0], dynamic_obstacles_on=True)
    axs[0, 0].set_title('Original Graph with Obstacles')
    draw_graph_with_obstacles(axs[0, 1], dynamic_obstacles_on=True)
    axs[0, 1].set_title('Original Graph with Dynamic Obstacles On')
    if results['dfs']['path']:
        dfs_path_edges = list(zip(results['dfs']['path'], results['dfs']['path'][1:]))
        draw_graph_with_obstacles(axs[1, 0], dynamic_obstacles_on=True, highlight_edges=dfs_path_edges)
        path_str = ' -> '.join(results['dfs']['path'])
        axs[1, 0].set_title(f'DFS Path (Cost: {results["dfs"]["cost"]})\nPath: {path_str}')
    if results['bfs']['path']:
        bfs_path_edges = list(zip(results['bfs']['path'], results['bfs']['path'][1:]))
        draw_graph_with_obstacles(axs[1, 1], dynamic_obstacles_on=True, highlight_edges=bfs_path_edges)
        path_str = ' -> '.join(results['bfs']['path'])
        axs[1, 1].set_title(f'BFS Path (Cost: {results["bfs"]["cost"]})\nPath: {path_str}')
    if results['a_star']['path']:
        a_star_path_edges = list(zip(results['a_star']['path'], results['a_star']['path'][1:]))
        draw_graph_with_obstacles(axs[2, 0], dynamic_obstacles_on=True, highlight_edges=a_star_path_edges)
        path_str = ' -> '.join(results['a_starA']['path'])
        axs[2, 0].set_title(f'A* Path (Cost: {results["a_star"]["cost"]})\nPath: {path_str}')
    if results['ucs']['path']:
        ucs_path_edges = list(zip(results['ucs']['path'], results['ucs']['path'][1:]))
        draw_graph_with_obstacles(axs[2, 1], dynamic_obstacles_on=True, highlight_edges=ucs_path_edges)
        path_str = ' -> '.join(results['ucs']['path'])
        axs[2, 1].set_title(f'UCS Path (Cost: {results["ucs"]["cost"]})\nPath: {path_str}')
    plt.tight_layout()
    plt.show()

# Get start and target nodes from user
start_node = input("Enter the start node: ")
target_node = input("Enter the target node: ")
visualize_all(start_node, target_node)
