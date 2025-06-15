import numpy as np
import csv

def get_grid_coordinates(x, y, map_origin, map_resolution):
        """
        Convert world coordinates to grid coordinates
        Args:
            x (float): x coordinate in world frame
            y (float): y coordinate in world frame
            map_origin (tuple): origin of the map in world frame
            map_resolution (float): resolution of the map
        Returns:
            (int, int): x and y coordinates in grid frame
        """
        grid_x = int((x - map_origin[0]) / map_resolution)
        grid_y = int((y - map_origin[1]) / map_resolution)

        return grid_x, grid_y
    
def get_world_coordinates(grid_x, grid_y, map_origin, map_resolution):
    """
    Convert grid coordinates to world coordinates
    Args:
        grid_x (int): x coordinate in grid frame
        grid_y (int): y coordinate in grid frame
    Returns:
        (float, float): x and y coordinates in world frame
    """
    world_x = grid_x * map_resolution + map_origin[0]
    world_y = grid_y * map_resolution + map_origin[1]

    return world_x, world_y

def update_grid_with_ray(
    grid: np.ndarray,
    start_x: int,
    start_y: int,
    angle: float,  # Input in radians
    distance: float,
    map_resolution: float,
    map_width: int,
    map_height: int,
    area_size: int
) -> np.ndarray:
    """
    Updates an occupancy grid by tracing a ray from (start_x, start_y) to a point at 
    `distance` meters away along `angle`, marking free cells along the ray and occupied 
    cells in a square area around the endpoint.

    Args:
        grid (np.ndarray): 2D occupancy grid (0=free, 100=occupied).
        start_x (int): Starting x-coordinate in grid cells.
        start_y (int): Starting y-coordinate in grid cells.
        angle (float): Ray angle in radians.
        distance (float): Maximum ray distance in meters.
        map_resolution (float): Grid resolution (meters/cell).
        map_width (int): Grid width in cells.
        map_height (int): Grid height in cells.
        area_size (int): Half-size of the square area around the endpoint to mark as occupied.

    Returns:
        np.ndarray: Updated occupancy grid.
    """
    # Validate inputs
    if not (0 <= start_x < map_width and 0 <= start_y < map_height):
        raise ValueError("Start position out of grid bounds.")
    if map_resolution <= 0:
        raise ValueError("map_resolution must be positive.")

    # Calculate endpoint
    range_in_cells = int(round(distance / map_resolution))
    end_x = int(round(start_x + range_in_cells * np.cos(angle)))
    end_y = int(round(start_y + range_in_cells * np.sin(angle)))

    # Bresenham's line algorithm
    x, y = start_x, start_y
    dx = abs(end_x - x)
    dy = -abs(end_y - y)
    sx = 1 if end_x > x else -1
    sy = 1 if end_y > y else -1
    err = dx + dy

    max_x = map_width - 1
    max_y = map_height - 1

    while True:
        if 0 <= x <= max_x and 0 <= y <= max_y:
            grid[y, x] = 0  # Free cell

        if x == end_x and y == end_y:
            break

        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy

    # Mark occupied area around endpoint
    for dx in range(-area_size, area_size):
        for dy in range(-area_size, area_size):
            new_x = end_x + dx
            new_y = end_y + dy
            if 0 <= new_x < map_width and 0 <= new_y < map_height:
                grid[new_y, new_x] = 100  # Occupied cell

    return grid

def load_waypoints(filepath):
    """
    Load waypoints from a CSV file.
    Args:
        filepath (str): Path to the CSV file.
    Returns:
        list of tuples: List of (x, y) waypoints.
    """
    waypoints = []
    with open(filepath, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            x = float(row[0])
            y = float(row[1])
            waypoints.append((x, y))
    return waypoints    