import numpy as np
import re
import os


class WbtObject:
    def __init__(self, name, translation=None, size=None):
        self.name = name
        self.translation = translation or [0, 0, 0]
        self.size = size or [0, 0, 0]


def parse_wbt_file(filename):
    """
    Parse a Webots .wbt file and extract relevant object information
    
    Args:
        filename (str): Path to .wbt file
    
    Returns:
        list: List of WbtObject instances
    """
    objects = []

    with open(filename, 'r') as f:
        content = f.read()

    # Find all object blocks
    object_pattern = r'(\w+)\s*{[^}]*?translation\s+([-\d.\s]+)[^}]*?(?:size\s+([-\d.\s]+)[^}]*?)?}'
    matches = re.finditer(object_pattern, content, re.DOTALL)

    for match in matches:
        obj_type = match.group(1)

        # Only process relevant objects
        if obj_type not in ['Wall', 'CardboardBox', 'WoodenBox', 'ConveyorBelt']:
            continue

        # Parse translation
        translation = [float(x) for x in match.group(2).split()]

        # Parse size if present
        size = None
        if match.group(3):
            size = [float(x) for x in match.group(3).split()]

        if not size: print(f"Size field for object type {obj_type} not found!")

        objects.append(WbtObject(obj_type, translation, size))

    return objects


def grid_to_world(pixel_x, pixel_y, world_size_x, world_size_y, resolution):
    """
    Convert grid coordinates (pixels) to world coordinates (meters)
    
    Args:
        pixel_x (int): X coordinate in pixels
        pixel_y (int): Y coordinate in pixels
        world_size_x (float): World size in X dimension (meters)
        world_size_y (float): World size in Y dimension (meters)
        resolution (float): Resolution in meters per pixel
    
    Returns:
        tuple: (x, y) coordinates in meters
    """
    x_offset = world_size_x / 2
    y_offset = world_size_y / 2

    # Convert pixel coordinates to world coordinates
    world_x = (pixel_x * resolution) - x_offset
    # Flip Y coordinate since grid Y increases downward but world Y increases upward
    world_y = y_offset - (pixel_y * resolution)

    return world_x, world_y


def create_pgm_map(objects, world_size_x, world_size_y, resolution):
    """
    Create a PGM map from parsed Webots objects
    
    Args:
        objects (list): List of WbtObject instances
        world_size_x (float): World size in X dimension (meters)
        world_size_y (float): World size in Y dimension (meters)
        resolution (float): Resolution in meters per pixel
    
    Returns:
        numpy.ndarray: Binary occupancy grid
    """
    # Calculate grid dimensions
    grid_size_x = int(np.round(world_size_x / resolution))
    grid_size_y = int(np.round(world_size_y / resolution))

    # Create empty grid (255 = white = free space)
    grid = np.full((grid_size_y, grid_size_x), 255, dtype=np.uint8)

    # Convert world coordinates to grid coordinates
    def world_to_grid(x, y):
        # Convert from Webots coordinates to grid coordinates
        grid_x = int(np.round((x + world_size_x / 2) / resolution))
        # Flip Y axis since grid Y increases downward
        grid_y = int(np.round((world_size_y / 2 - y) / resolution))
        return grid_x, grid_y

    def meters_to_pixels(meters):
        """Convert a distance in meters to number of pixels"""
        return int(np.round(meters / resolution))

    # Function to add rectangular obstacle to grid
    def add_rectangle(x, y, width, height):
        # Convert dimensions from meters to pixels
        width_px = meters_to_pixels(width)
        height_px = meters_to_pixels(height)

        # Get center point in grid coordinates
        center_x, center_y = world_to_grid(x, y)

        # Calculate rectangle bounds
        start_x = center_x - width_px // 2
        end_x = start_x + width_px
        start_y = center_y - height_px // 2
        end_y = start_y + height_px

        # Ensure coordinates are within grid bounds
        start_x = max(0, min(start_x, grid_size_x))
        start_y = max(0, min(start_y, grid_size_y))
        end_x = max(0, min(end_x, grid_size_x))
        end_y = max(0, min(end_y, grid_size_y))

        # Ensure start coordinates are less than end coordinates
        # start_x, end_x = min(start_x, end_x), max(start_x, end_x)
        # start_y, end_y = min(start_y, end_y), max(start_y, end_y)

        if start_x < end_x and start_y < end_y:
            grid[start_y:end_y, start_x:end_x] = 0

    # Process each object
    for obj in objects:
        x, y = obj.translation[0], obj.translation[1]

        if obj.name == 'Wall' and obj.size:
            add_rectangle(x, y, obj.size[0], obj.size[1])

        elif obj.name == 'CardboardBox':
            add_rectangle(x, y, obj.size[0], obj.size[1])

        elif obj.name == 'WoodenBox':
            # Size not present in world file
            add_rectangle(x, y, 0.6, 0.6)

        elif obj.name == 'ConveyorBelt' and obj.size:
            add_rectangle(x, y, obj.size[0], obj.size[1])

    return grid


def save_pgm(grid, resolution, filename="map.pgm"):
    """
    Save the occupancy grid as a PGM file
    
    Args:
        grid (numpy.ndarray): Binary occupancy grid
        filename (str): Output filename
        resolution (float): Resolution in meters per pixel
    """
    height, width = grid.shape

    # Calculate world dimensions
    world_size_x = width * resolution
    world_size_y = height * resolution

    # Calculate center point in pixel coordinates
    center_pixel_x = width // 2
    center_pixel_y = height // 2

    # Convert to world coordinates
    world_x, world_y = grid_to_world(center_pixel_x, center_pixel_y,
                                     world_size_x, world_size_y, resolution)

    print(f"Grid dimensions: {width}x{height} pixels")
    print(f"World dimensions: {world_size_x:.2f}x{world_size_y:.2f} meters")
    print(f"Center pixel: ({center_pixel_x}, {center_pixel_y})")
    print(f"Center point in world coordinates: ({world_x:.2f}, {world_y:.2f}) meters")

    with open(filename, 'wb') as f:
        # Write PGM header
        f.write(f"P5\n{width} {height}\n255\n".encode())
        # Write image data
        grid.tofile(f)


def main(wbt_file, output_file="map.pgm"):
    """
    Main function to process a .wbt file and generate a PGM map
    
    Args:
        wbt_file (str): Input .wbt file path
        output_file (str): Output PGM file path
    """
    # Parse the WBT file
    objects = parse_wbt_file(wbt_file)

    if int(os.getenv("DEBUG", "0")) >= 1:
        print(f"Parsed objects:")
        [print(f"{o.name} -> size: [{o.size[0]}, {o.size[1]}]") for o in objects]

    # Create the grid
    grid = create_pgm_map(objects, 14.2, 7.2, 0.05)

    # Save the map
    save_pgm(grid, 0.05, output_file)
    print(f"Map created with dimensions: {grid.shape}")


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python script.py <input_wbt_file> [output_pgm_file]")
        sys.exit(1)

    wbt_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "map.pgm"
    main(wbt_file, output_file)
