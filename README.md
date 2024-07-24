# 3D Pathfinding and Collision Avoidance for Multiple Projectiles

## Project Overview

This project demonstrates the implementation of a 3D pathfinding algorithm with obstacle and collision avoidance for multiple projectiles. The program calculates paths between starting and ending coordinates, detects obstacles, and computes alternate routes if needed. Additionally, it ensures projectiles avoid colliding with each other during their journey.



## Features

- **3D Pathfinding:** Calculates straight-line paths between starting and ending points for each projectile.
- **Obstacle Avoidance:** Detects spherical obstacles and computes alternate paths around them.
- **Collision Avoidance:** Prevents projectiles from colliding with each other by dynamically adjusting their paths.
- **Visualization:** Uses Matplotlib to create a 3D plot visualizing the paths of the projectiles, obstacles, and start/end points.

## Requirements

- Python 3.x
- NumPy
- Matplotlib

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/3D-Pathfinding-and-Collision-Avoidance-for-Multiple-Projectiles.git
    ```
2. Navigate to the project directory:
    ```bash
    cd 3D-Pathfinding-and-Collision-Avoidance-for-Multiple-Projectiles
    ```
3. Install the required packages:
    ```bash
    pip install numpy matplotlib
    ```

## Usage

Run the main script to start the simulation:
```bash
python main.py
```

### Example Input

- Number of projectiles: 4
- Starting points: (0, 0, 0), (10, 0, 0), (0, 10, 0), (10, 10, 0)
- Ending points: (10, 10, 10), (0, 10, 10), (10, 0, 10), (0, 0, 10)
- Obstacles: (5, 5, 5, 3), (8, 8, 8, 2)

### Expected Output

A 3D plot showing the paths of the projectiles, the obstacles, and the start/end points.

## Code Overview

### Main Components

- **Projectile Class:** Represents a projectile with attributes like start and end points, position, radius, and path.
- **Pathfinding Functions:** Compute paths, detect obstacles, and find alternate routes.
- **Collision Avoidance Functions:** Ensure projectiles do not collide with each other.
- **Visualization:** Plot the paths of projectiles and obstacles using Matplotlib.

### Key Functions

- `plot_line(x1, y1, z1, x2, y2, z2, style='b-')`: Plots a line between two points.
- `is_obstacle_in_path(p1, p2, obstacles)`: Checks if there is an obstacle in the path.
- `find_alternate_path(start, end, obstacles)`: Finds an alternate path around obstacles.
- `plot_path(path, style='b-')`: Plots the path of a projectile.
- `update_positions(projectiles, obstacles, step_size=0.1)`: Updates the positions of projectiles.
- `check_collisions(projectiles, step_size)`: Checks and adjusts for collisions between projectiles.

## Future Improvements

1. **Advanced Pathfinding Algorithms:**
   - Implement A* or Dijkstra's algorithm for more efficient path planning.

2. **Dynamic Obstacles:**
   - Introduce moving obstacles and update pathfinding in real-time.

3. **User Interface:**
   - Add a graphical user interface (GUI) using libraries like Tkinter or PyQt.

4. **Parameters:**
   - **Fuel:** Implement fuel constraints affecting the maximum travel distance.
   - **Speed:** Vary the speed of projectiles and incorporate it into path calculations.
   - **Time:** Calculate and display the time taken for each projectile to reach its destination.

5. **Performance Optimization:**
   - Optimize the code for better performance, especially for a larger number of projectiles and obstacles.

6. **Enhanced Visualization:**
   - Improve the visualization with more detailed graphics and interactive elements.

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue if you have any suggestions or improvements.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.


