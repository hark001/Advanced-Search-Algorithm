# Advanced-Search-Algorithm
Implementation of PRM (for various sampling methods), RRT and RRT* Search Algorithms.

For **PRM**, you are required to implement 4 different sampling methods - **uniform sampling**, **random sampling**, **gaussian sampling** and **bridge sampling**. These three algorithms are the basic ones for sampling-based planning.

Packages required for installation:
**Bresenham** : pip install bresenham (This package is used for returning all the points between two points in a 2D space.)

Files included:

**PRM.py** is the file where the PRM class is implemented with four different sampling methods.

**RRT.py** is the file wherethe RRT class for RRT and RRT* is implemented.

**main.py** is the scrip that provides helper functions that load the map from an image and call the classes and functions from **PRM.py** and **RRT.py**.

**WPI_map.jpg** is a binary WPI map image with school buildings.

## Instruction

`python main.py`

The **main.py** loads the map image **WPI_map.jpg** and calls classes and functions to run planning tasks.