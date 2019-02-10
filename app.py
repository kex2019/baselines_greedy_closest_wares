import sys
import os
import time
import random
import collections

# Hacky but tacky
# sys.path.append(os.getcwd())

import robotic_warehouse.robotic_warehouse as rw
import robotic_warehouse_utils.path_finder as pf

MOVE_DOWN = 0
MOVE_LEFT = 1
MOVE_UP = 2
MOVE_RIGHT = 3
PICKUP_PACKAGE = 4
DROP_PACKAGE = 5
def dir_to_string(dr):
    if dr == 0:
        return "MOVE_DOWN"
    if dr == 1:
        return "MOVE_LEFT"
    if dr == 2:
        return "MOVE_UP"
    if dr == 3:
        return "MOVE_RIGHT"
    return ""

def position_equal(p1, p2):
    return p1[0] == p2[0] and p1[1] == p2[1]

# Finds the closest robot for every package
def robot_for_package(gym):
    print(gym.map)
    print(gym.packages)
    print(gym.robots)
    # N^2 to find shortest robot for every package
    shortest_robot = {}
    for packageId, package in gym.packages.items():
        shortest = 1e6
        for i in range(len(gym.robots)):
            robot = gym.robots[i]
            if len(robot[1]) >= gym.capacity:
                continue
            pos = robot[0]
            distance = abs(pos[0] - package[0][0]) + abs(pos[1] - package[1][1])
            if distance < shortest:
                shortest = distance
                shortest_robot[packageId] = i
    return shortest_robot

# Finds the closest package for every robot, or none if the robot is full
def package_for_robot(gym):
    robots = [None] * len(gym.robots)
    for i in range(len(gym.robots)):
        shortest = 1e6
        robot = gym.robots[i]
        if len(robot[1]) >= gym.capacity:
            # We can't carry any more packages, set goal to random drop position instead
            robots[i] = random.choice(gym.drop_positions)
            continue

        robotPos = robot[0]
        for packageId, package in gym.packages.items():
            packagePos = package[0]
            distance = abs(robotPos[0]-packagePos[0]) + abs(robotPos[1]-packagePos[1])
            if distance < shortest:
                shortest = distance
                robots[i] = packageId
    return robots

# Gets the closest path _from, _to
def path_to(gym, _from, _to):
    astar = pf.Astar(gym)
    newto = astar.available_pos_near(_to)
    print(_from, ", ", _to, ", newto: ", newto)

    instructions = astar(_from, newto).get_instructions()
    return instructions
    
    if _to is None:
        return []
    queue = [_from]
    path = []
    prev = [[None for _ in range((len(gym.map)) + 1)] for _ in range(len(gym.map[0]) + 1)]
    while len(queue) > 0:
        cur = queue[0]
        queue.pop(0)
        
        if position_equal(cur, _to):
            break
        nextdxdy = [[1,0],[-1,0],[0,1],[0,-1]]
        nextxy = [[pos[0]+cur[0],pos[1]+cur[1]] for pos in nextdxdy]
        for i in range(len(nextxy)):
            x = nextxy[i][0]
            y = nextxy[i][1]
            if x < 0 or x >= len(gym.map) or y < 0 or y >= len(gym.map[0]):
                continue

            if prev[y][x] is None:
                queue.append([x,y])
                prev[y][x] = cur
    # Retrace the path
    cur = _to
    while(not position_equal(cur, _from)):
        path.append(cur)
        cur = prev[cur[1]][cur[0]]
    path.append(cur)
    path.reverse()
    return path

def deltatodir(delta):
    if delta[0] == -1:
        return MOVE_LEFT
    if delta[0] == 1:
        return MOVE_RIGHT
    if delta[1] == -1:
        return MOVE_UP
    if delta[1] == 1:
        return MOVE_DOWN
    return -1

# Gets the direction for the first element in the path
def dir_in_path(gym, path):
    if len(path) < 2:
        return MOVE_DOWN
    dx = path[0][0] - path[1][0]
    dy = path[0][1] - path[1][1]
    dr = deltatodir([dx,dy])
    return dr

timestamp = time.time()
gym = rw.RoboticWarehouse(
    robots=1,
    capacity=1,
    spawn=5,
#    spawn_rate=0.1,
    shelve_length=8,
    shelve_height=4,
    shelve_width=4,
    shelve_throughput=1,
    cross_throughput=5)
print("Setup Time: {}".format(time.time() - timestamp))
print("A robot for every package!")
print(robot_for_package(gym))
print(package_for_robot(gym))
steps = 0
timestamp = time.time()

def get_goal_position(positionDict, goal, currentPosition):
    if hasattr(goal, "__len__"):
        return goal
    if goal is None or positionDict[goal] is None:
        return currentPosition
    return positionDict[goal][0]

try:
    while True:
        print("STEPS: ",steps)
        gym.render()
        package_assignments = package_for_robot(gym)
        # For each robot get a path
        robot_paths = [path_to(gym, gym.robots[i][0], get_goal_position(gym.packages, package_assignments[i], gym.robots[i][0])) for i in range(len(gym.robots))]
        dirs = [None for _ in range(len(robot_paths))]
        for i in range(len(robot_paths)):
            if len(robot_paths[i]) == 0: # Going to have to fix this logic
                 # If we have a package drop it, otherwise pick it up
                if len(gym.robots[i][1]) > 0:
                    print("DROP PACKAGE", len(gym.robots[i][1]))
                    dirs[i] = DROP_PACKAGE
                else:
                    dirs[i] = PICKUP_PACKAGE
                    print("Robot ", i, " picking package")
            else:
                dirs[i] = robot_paths[i][0]

        gym.step(dirs)
        #gym.step(gym.action_space.sample())
        steps += 1
        # time.sleep(0.1)
except KeyboardInterrupt:
    print("Number of steps: {}, average step per second: {}".format(
        steps, steps / (time.time() - timestamp)))

