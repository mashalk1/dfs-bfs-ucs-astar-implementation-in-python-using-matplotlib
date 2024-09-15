# Pathfinding Search Algorithm

## Overview

This project visualizes pathfinding algorithms on a graph with various types of obstacles. It provides an interactive way to compare how different algorithms perform in finding paths between nodes while navigating static and dynamic obstacles. All algorithms are executed concurrently to provide a comparative view of their performance.

## Features

- **Concurrent Pathfinding Algorithms**: The project simultaneously runs Depth-First Search (DFS), Breadth-First Search (BFS), A* Search, and Uniform Cost Search (UCS) to find paths between nodes.
- **Dynamic and Static Obstacles**: The graph includes both static and dynamic obstacles that affect the pathfinding algorithms.
- **Interactive Visualization**: Displays multiple graphs to show the original graph, paths taken by different algorithms, and obstacles encountered.

## Graphs Displayed

1. **Original Graph with Obstacles**:
   - Displays the entire graph with all nodes, edges, static obstacles, and dynamic obstacles.
   - Static obstacle edges are highlighted in red, and dynamic obstacle edges are highlighted in green.

2. **Pathfinding Results**:
   - **DFS Path**: Shows the path found using Depth-First Search with its total cost and nodes followed.
   - **BFS Path**: Shows the path found using Breadth-First Search with its total cost and nodes followed.
   - **A* Path**: Shows the path found using A* Search with its total cost and nodes followed.
   - **UCS Path**: Shows the path found using Uniform Cost Search with its total cost and nodes followed.

3. **Original Graph without Paths**:
   - Displays the original graph with static and dynamic obstacles but without any paths highlighted.

## How to Use

1. **Input**:
   - Enter the start node and the target node when prompted.
   - The code will automatically run all four algorithms (DFS, BFS, A*, UCS) concurrently.

2. **Visualization**:
   - The application generates and displays five graphs:
     1. Original graph with all obstacles.
     2. Original graph with dynamic obstacles.
     3. Pathfinding results for DFS.
     4. Pathfinding results for BFS.
     5. Pathfinding results for A*.
     6. Pathfinding results for UCS.

3. **Results**:
   - Each algorithm's graph shows the path taken, total cost, and nodes followed. Obstacles avoided are highlighted in orange.

## Installation

To run the visualization, ensure you have the following Python libraries installed:
- `networkx`
- `matplotlib`

Install the required libraries using pip:
```bash
pip install networkx matplotlib
```

## Example

Here's a sample input and output:

**Input**:

Enter the start node: A
Enter the target node: S
```

**Output**:
- Displays graphs with paths for DFS, BFS, A*, and UCS, showing the routes, costs, and nodes followed