# Path Planning {#t20176}


# Plan for today

-   Learn about our path planning system, the Rapidly-Exploring Random Tree (RRT)
-   Learn about other path planning algorithms
-   Download, compile, and run our RRT repository


# High Level Overview

![img](https://i.imgur.com/4Y3wCJH.jpg)


# What is Path Planning

-   Our robots need to know how to get from a start point to an end point
-   IGVC uses A\*
-   RoboCup uses an RRT (Rapidly-Exploring Random Tree)
-   STOx's Planner
-   Path network generation


# A\*

-   Search space
    -   Discrete network of nodes
    -   Traversable edges between nodes
-   Generalized Dijkstra's algorithm
    -   Generalized breadth-first search


# BFS

![img](https://i.imgur.com/FYNyt8T.gif)


# Dijkstra's algorithm

-   Like BFS, but acknowledges costs with each edge
-   Associates each edge with a distance cost, and assigns each node with a tentative distance cost

-   At each iteration, update the distance to the nodes neighboring the current node
-   Select the unvisited node with the smallest tentative distance at the next iteration
-   BFS is Dijkstra's algorithm with equal edge weights


# A\*

-   Like Dijkstra's but with a heuristic function h(n)
-   Cost function f(n) = g(n) + h(n)
    -   g(n): cost of path from start to n
    -   h(n): estimate cost of cheapest path from n to goal
-   Heuristic must be admissible (no overestimating)


# RRT

-   Continuous search space
-   Randomly extend a space-filling tree throughout the search space until we connect the start and goal states


# How the RRT works

1.  Start building our tree by placing a root node at the destination
2.  Randomly select some coordinate in the position space
3.  Identify existing node in the tree that is nearest to that coordinate
4.  Add new node in tree branching from nearest node to random coordinate
5.  Repeat 2-4 until a node is created at our destination. The series of connected nodes from root to destination forms the path that the robot will follow
6.  Smooth out path with overlaid Bézier curve


# Sounds really inefficient

-   Why waste computation time branching out in random directions?
-   What advantages could there be in random branching?
-   Why not use a less computationally intense algorithm like A\*?


## Advantages of RRT

-   Specialized for continuous configuration spaces
-   Can handle kinodynamic constraints
-   Algorithm can be modified for various needs and preferences


# How to construct a Bézier curve


## Linear curve

-   Bézier curve B(t)
-   The t in the function for a linear Bézier curve can be thought of as describing how far B(t) is from P0 to P1
-   For example, when t=0.25, B(t) is one quarter of the way from point P0 to P1. As t varies from 0 to 1, B(t) describes a straight line from P0 to P1

![img](https://upload.wikimedia.org/wikipedia/commons/0/00/B%C3%A9zier_1_big.gif)


## Quadratic curve

-   For quadratic Bézier curves one can construct intermediate points Q0 and Q1 such that as t increases from 0 to 1
    -   Point Q0(t) varies from P0 to P1 and describes a linear Bézier curve.
    -   Point Q1(t) varies from P1 to P2 and describes a linear Bézier curve.
    -   Point B(t) is interpolated linearly between Q0(t) to Q1(t) and describes a quadratic Bézier curve.

![img](https://upload.wikimedia.org/wikipedia/commons/3/3d/B%C3%A9zier_2_big.gif)


# Repository Code

-   rrt-viewer
-   rrt


## rrt-viewer

-   Displays window for running RRT
-   Uses QT for graphics


## rrt

-   Contains RRT implementation
-   Defines state space
-   Bi-RRT
    -   We execute two RRTs, one rooted at the start node and the other at the end node


# Download RRT

-   Our rrt repository is located at <http://github.com/RoboJackets/rrt>
-   On your terminal, type in:

```shell
git clone https://github.com/RoboJackets/rrt.git
```

-   DO NOT execute this command in an existing git repository


# Compile and run RRT

```shell
cd rrt
git clone http://github.com/RoboJackets/rrt rrt
make
./build/rrt-viewer
```


# How to use RRT

-   Drag start and end points to desired locations
-   Drag around the plane space to draw and remove obstacles
-   Click "run" to run until the rrt finds a valid path, or "step" to execute a single rrt iteration
-   Click "reset" once to delete the tree, twice to delete the previously calculated path


# Tweaking parameters


## Biases

-   Increasing Goal Bias
    -   Random branching has tendency to branch directly towards goal instead
-   Increasing Waypoint Bias
    -   Random branching has tendency to branch towards Bézier curve waypoints of previous paths
-   Goal Bias + Waypoing Bias must sum to at most 1.0


## Adaptive Stepsize Control

-   Stepsize now dynamically changes based on whether there are obstacles nearby
-   Requires extra computation time to locate nearby obstacles
-   Having larger stepsizes when possible reduces total iteration count, which reduces overall computation time
-   Obstacle-light environments benefit the most from this enhancement


# STOx's Planner

-   Generate a straight line from the start state to end state
-   As long as the path intersects an obstacle:
    -   Generate a subgoal state next to the obstacle
    -   Now divided into two smaller subproblems
    -   Recurse!


## STOx's Planner

![img](https://i.imgur.com/Ea040em.png)


## STOx's Planner

-   Very fast when obstacle count is low
-   Not very flexible


# Path Network

-   Transform continuous space into discrete space
-   Invisible network of waypoints
-   Obstacles represented as polygons


## Path Network

![img](https://i.imgur.com/lM67O8P.png)


# Navigation Mesh

![img](https://i.imgur.com/uoD7ARv.png)


## Navigation Mesh

![img](https://i.imgur.com/ADl3xa3.png)


## Navigation Mesh

-   For each point
    -   Pick two other points
    -   See if they form a triangle through traversable space
    -   See if the triangle does not cross an existing triangle in the mesh
    -   If yes, add triangle to nav mesh


## Navigation Mesh

-   For any 2 triangles with a shared edge
    -   If the merged polygon is convex, replace them with the new polygon
-   Repeat for higher order polygons


# Generating a path network from a nav mesh

-   For each polygon in the nav mesh, place a path node in its center

![img](https://i.imgur.com/BQ2I4lH.png)


## Generating a path network from a nav mesh

-   Alternatively, place a path node at midpoint of each edge between two adjacent polygons

![img](https://i.imgur.com/EyNSpgk.png)


# Any questions?
