from enum import Enum
from queue import PriorityQueue
import numpy as np
from bresenham import bresenham


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]

            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    # Diagonal actions
    WE_NOR = (-1, -1, np.sqrt(2))
    WE_SOU = (1, -1, np.sqrt(2))
    EA_NOR = (-1, 1, np.sqrt(2))
    EA_SOU = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    
    # filter diagonal action feasibility
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.WE_NOR)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.WE_SOU)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.EA_NOR)
    if x + 1 > n or y +1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.EA_SOU)

    return valid_actions
def prune_func(grid, path):
    """
	Trim  the points
    """
    pp_init = [p for p in path]

    n = 0
    while n < len(pp_init) - 2:
        waypoint_uno = pp_init[n]
        waypoint_dos = pp_init[n+1]
        waypoint_tres = pp_init[n+2]

        point_counter = bresenham(waypoint_uno[0], waypoint_uno[1], waypoint_tres[0], waypoint_tres[1])
        if all(grid[count[0], count[1]] == 0 for count in point_counter):
            pp_init.remove(waypoint_dos)
        else:
            n += 1

    return pp_init

def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]
            
        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for direction_vec in valid_actions(grid, current_node):
                # get the tuple representation
                delta_action_dir = direction_vec.delta
                next_node = (current_node[0] + delta_action_dir[0], current_node[1] + delta_action_dir[1])
                branch_cost = current_cost + direction_vec.cost
                queue_cost = branch_cost + h(next_node, goal)
                

                if next_node not in visited:

                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, direction_vec)
                    queue.put((queue_cost, next_node))


    if found:
        # retrace steps
        goal_step = goal

        path_cost = branch[goal_step][0]
        path.append(goal)
        while branch[goal_step][1] != start:
            path.append(branch[goal_step][1])
            goal_step = branch[goal_step][1]
        path.append(branch[goal_step][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    """ 
             find the herustic for the postion by normalizing
    """
    nump_pos=np.array(position)
    nump_goal=np.array(goal_position)
    return np.linalg.norm(nump_pos - nump_goal)

