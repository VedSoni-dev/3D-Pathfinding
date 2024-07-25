import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class Projectile:
    def __init__(self, start, end, radius=0.5):
        self.start = np.array(start)
        self.end = np.array(end)
        self.position = np.array(start)
        self.radius = radius
        self.path = [start]
        self.finished = False

def plot_line(x1, y1, z1, x2, y2, z2, style='b-'):
    ax.plot([x1, x2], [y1, y2], [z1, z2], style)

def is_obstacle_in_path(p1, p2, obstacles):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    for (ox, oy, oz, radius) in obstacles:
        if min(x1, x2) <= ox <= max(x1, x2) and min(y1, y2) <= oy <= max(y1, y2) and min(z1, z2) <= oz <= max(z1, z2):
            # Check if the line intersects the sphere
            dx = x2 - x1
            dy = y2 - y1
            dz = z2 - z1
            a = dx**2 + dy**2 + dz**2
            b = 2 * (dx * (x1 - ox) + dy * (y1 - oy) + dz * (z1 - oz))
            c = (x1 - ox)**2 + (y1 - oy)**2 + (z1 - oz)**2 - radius**2
            det = b**2 - 4 * a * c
            if det >= 0:
                return True
    return False

def find_alternate_path(start, end, obstacles):
    x1, y1, z1 = start
    x2, y2, z2 = end
    for (ox, oy, oz, radius) in obstacles:
        if min(x1, x2) <= ox <= max(x1, x2) and min(y1, y2) <= oy <= max(y1, y2) and min(z1, z2) <= oz <= max(z1, z2):
            # Calculate direction vector
            direction_vector = np.array([x2 - x1, y2 - y1, z2 - z1])
            direction_vector = direction_vector / np.linalg.norm(direction_vector)
            
            # Offset around the obstacle
            offset_distance = radius + 1  # 1 meter buffer around the obstacle
            
            # Calculate two perpendicular vectors
            perp_vector_1 = np.cross(direction_vector, np.array([1, 0, 0]))
            if np.linalg.norm(perp_vector_1) == 0:
                perp_vector_1 = np.cross(direction_vector, np.array([0, 1, 0]))
            perp_vector_1 = perp_vector_1 / np.linalg.norm(perp_vector_1)
            
            perp_vector_2 = np.cross(direction_vector, perp_vector_1)
            perp_vector_2 = perp_vector_2 / np.linalg.norm(perp_vector_2)
            
            # Determine new path points
            new_point_1 = np.array([ox, oy, oz]) + perp_vector_1 * offset_distance
            new_point_2 = np.array([ox, oy, oz]) + perp_vector_2 * offset_distance
            
            # Choose the best offset point to add to the path
            if np.linalg.norm(new_point_1 - np.array([x1, y1, z1])) < np.linalg.norm(new_point_2 - np.array([x1, y1, z1])):
                return new_point_1
            else:
                return new_point_2
    return end

def plot_path(path, style='b-'):
    for i in range(len(path) - 1):
        x1, y1, z1 = path[i]
        x2, y2, z2 = path[i + 1]
        plot_line(x1, y1, z1, x2, y2, z2, style)

def update_positions(projectiles, obstacles, step_size=0.1):
    for proj in projectiles:
        if not proj.finished:
            direction_vector = proj.end - proj.position
            distance_to_target = np.linalg.norm(direction_vector)
            if distance_to_target < step_size:
                proj.position = proj.end
                proj.path.append(tuple(proj.position))
                proj.finished = True
                continue
            direction_vector = direction_vector / distance_to_target
            new_position = proj.position + direction_vector * step_size

            if is_obstacle_in_path(proj.position, new_position, obstacles):
                new_position = find_alternate_path(proj.position, proj.end, obstacles)
            
            proj.position = new_position
            proj.path.append(tuple(proj.position))

def check_collisions(projectiles, step_size):
    for i, proj1 in enumerate(projectiles):
        for j, proj2 in enumerate(projectiles):
            if i != j and not proj1.finished and not proj2.finished:
                distance = np.linalg.norm(proj1.position - proj2.position)
                if distance < proj1.radius + proj2.radius:
                    proj1.finished = True
                    proj2.finished = True
                elif distance < 2 * (proj1.radius + proj2.radius):
                    # Adjust paths to avoid collision
                    direction_vector = proj2.position - proj1.position
                    direction_vector = direction_vector / np.linalg.norm(direction_vector)
                    avoidance_vector = np.cross(direction_vector, np.array([0, 0, 1])) * step_size
                    proj1.position -= avoidance_vector
                    proj2.position += avoidance_vector

def main():
    # Number of projectiles
    n_projectiles = int(input("Enter number of projectiles: "))
    projectiles = []

    for i in range(n_projectiles):
        print(f"Enter details for projectile {i+1}:")
        start = tuple(map(float, input("Enter starting point coordinates (x1 y1 z1) in meters: ").split()))
        end = tuple(map(float, input("Enter ending point coordinates (x2 y2 z2) in meters: ").split()))
        projectiles.append(Projectile(start, end))

    # Obstacle input
    obstacles = []
    has_obstacles = input("Are there any obstacles? (yes/no): ").strip().lower()
    if has_obstacles == 'yes':
        n_obstacles = int(input("How many obstacles? "))
        for _ in range(n_obstacles):
            ox, oy, oz, radius = map(float, input("Enter obstacle coordinates and radius (ox oy oz radius) in meters: ").split())
            obstacles.append((ox, oy, oz, radius))

    fig = plt.figure()
    global ax
    ax = fig.add_subplot(111, projection='3d')
    
    # Simulation loop
    while any(not proj.finished for proj in projectiles):
        update_positions(projectiles, obstacles)
        check_collisions(projectiles, step_size=0.1)
    
    for proj in projectiles:
        plot_path(proj.path, style='b-')
    
    # Plot obstacles
    for ox, oy, oz, radius in obstacles:
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = ox + radius * np.cos(u) * np.sin(v)
        y = oy + radius * np.sin(u) * np.sin(v)
        z = oz + radius * np.cos(v)
        ax.plot_wireframe(x, y, z, color='black')
        ax.scatter(ox, oy, oz, color='black')  # Obstacle centers

    # Plot projectile paths and start/end points
    for proj in projectiles:
        ax.scatter(*zip(*proj.path), color='blue')
        ax.scatter(*proj.start, color='green')  # Start points
        ax.scatter(*proj.end, color='red')  # End points

    ax.set_xlabel('X-axis (meters)')
    ax.set_ylabel('Y-axis (meters)')
    ax.set_zlabel('Z-axis (meters)')
    ax.set_title('Paths of Multiple Projectiles with Obstacles')
    plt.show()

if __name__ == "__main__":
    main()
