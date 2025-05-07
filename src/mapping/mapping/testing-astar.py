def visualize_grid_and_path(grid, path=None, start=None, end=None):
    """Visualizes the occupancy grid and an optional path."""
    # grid = np.transpose(grid)
    # grid = np.fliplr(grid)
    # grid = np.rot90(grid)
    plt.figure(figsize=(20, 20))
    plt.imshow(grid, cmap='gray', origin='lower')
    plt.colorbar(label='Occupancy')

    if path:
        path_y = [node[0] for node in path]
        path_x = [node[1] for node in path]
        plt.plot(path_x, path_y, color='red', linestyle='-', linewidth=2, label='Planned Path')

    if start:
        plt.plot(start[0], start[1], marker='*', color='green', markersize=15, label='Start')

    if end:
        plt.plot(end[0], end[1], marker='X', color='blue', markersize=15, label='End')

    plt.title('Occupancy Grid and Planned Path')
    plt.xlabel('Column')
    plt.ylabel('Row')
    plt.legend()
    plt.grid(True)
    plt.show()

from mapping.map import Map
from project_interfaces.msg import ObjectList, Object

def read_workspace(filename, skip_header=False):
    workspace_vertices = []
    try:
        with open(filename, "r") as file:
            if skip_header:
                next(file)
            for line in file:
                line = line.strip()
                if line:
                    parts = line.split("\t")
                    if len(parts) == 2:
                        x, y = parts
                        workspace_vertices.append((float(x)/100, float(y)/100))
        return workspace_vertices
    except FileNotFoundError:
        return []

def read_map_file(filename): #ALL
    object_list = []
    try:
        with open(filename, 'r') as file:
            lines = file.readlines()

        for line in lines:
            parts = line.strip().split(',')
            if len(parts) < 4:
                continue

            if parts[0] == "1":
                object_type = Object.CUBE
            elif parts[0] == "2":
                object_type = Object.SPHERE
            elif parts[0] == "3":
                object_type = Object.PLUSHIE
            elif parts[0] == "B":
                object_type = Object.BOX
            else:
                object_type = parts[0]

            x = float(parts[1])/100
            y = float(parts[2])/100
            angle = float(parts[3])

            # Create new object message
            object_msg = Object()
            object_msg.x = x
            object_msg.y = y
            object_msg.angle = angle
            object_msg.object_type = object_type

            object_list.append(object_msg)
        return object_list
    except FileNotFoundError:
        return []

def main():
    map = Map(0.05)
    workspace_file = "workspaces/large_workspace.tsv"
    map_file = "maps/map.csv"
    workspace_vertices = read_workspace(workspace_file, skip_header=True)
    object_list = read_map_file(map_file)
    map.initialise_grid(workspace_vertices)
    for object_msg in object_list:
        map.add_object(object_msg.object_type, object_msg.x, object_msg.y, object_msg.angle)
    map.inflate_grid(0.35)
    occupancy_grid = map.grid
    grid_rows = map.grid_height
    grid_columns = map.grid_width

    # Define start and end nodes (make sure they are within bounds and not completely blocked)
    # point = (1.0, 1.7)
    point = (9.1, 3.0)
    start_node = map.world_to_grid(point[0], point[1])
    safe_point = map.get_safe_point(point[0], point[1], 3, 75)
    if safe_point is None:
        safe_point = (0.0, 0.0)

    end_node = map.world_to_grid(safe_point[0], safe_point[1])
    # end_node = map.world_to_grid(3.1, 8.8)

    start_free = map.are_adjacent_free(start_node[0], start_node[1], 1, 75, world=False)
    end_free = map.are_adjacent_free(end_node[0], end_node[1], 1, 75, world=False)

    print(f"Start free: {start_free}; End free: {end_free}")

    grid_origin_x, grid_origin_y = map.world_to_grid(map.origin_x, map.origin_y)
    print(f"Origin ({map.origin_x}, {map.origin_y}) = ({grid_origin_x}, {grid_origin_y})")

    # Ensure start and end are valid
    if not map.is_free(start_node[0], start_node[1], 100, world=False):
        print("Start node in occupied cell")
        return
    if not map.is_free(end_node[0], end_node[1], 100, world=False):
        print("End node in occupied cell")
        return

    path_planner = AdaptiveAStar(occupancy_grid)
    path = path_planner.plan_path((start_node[1], start_node[0]), (end_node[1], end_node[0]))

    # Visualize the grid and path
    if path:
        print("Path found!")
        visualize_grid_and_path(occupancy_grid, path, start_node, end_node)
    else:
        print("No path found.")
        visualize_grid_and_path(occupancy_grid, start=start_node, end=end_node)

if __name__ == "__main__":
    main()

