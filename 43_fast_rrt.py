import numpy as np
import cv2
from random import randint, uniform
import time

start_time = time.time() 

def map_to_bottom_left(point, height):
    # Maps a point to the bottom-left coordinate system
    return (point[0], height - point[1])

def is_valid(point, obstacle_map):
    # Checks if a point is valid (within the obstacle map)
    x, y = map_to_bottom_left(point, 600)
    if 0 <= x < obstacle_map.shape[1] and 0 <= y < obstacle_map.shape[0]:
        return obstacle_map[y, x, 0] == 255
    return False

class Node:
    def __init__(self, point, parent=None):
        # Represents a node in the RRT tree
        self.point = point
        self.parent = parent

def distance(p1, p2):
    # Calculates the Euclidean distance between two points
    return np.linalg.norm(np.array(p1) - np.array(p2))

def steer(towards, from_point, step_size, obstacle_map):
    # Steers towards a target point from a given point
    direction = np.array(towards) - np.array(from_point)
    length = np.linalg.norm(direction)
    direction = direction / length if length > 0 else np.random.rand(2) - 0.5

    new_point = np.array(from_point) + direction * min(step_size, length)
    new_point = tuple(new_point.astype(int))
    if is_valid(new_point, obstacle_map):
        return new_point
    return None

def fast_sample(obstacle_map, nodes, threshold):
    # Generates a random point that is not too close to existing explored nodes
    while True:
        point = (randint(0, obstacle_map.shape[1] - 1), randint(0, obstacle_map.shape[0] - 1))
        if not is_in_explored_area(point, nodes, threshold):
            return point

def is_in_explored_area(point, nodes, threshold):
    # Checks if a point is within the explored area (close to existing nodes)
    for node in nodes:
        if distance(point, node.point) < threshold:
            return True
    return False

def random_steer(nearest_node, step_size, obstacle_map):
    # Steers randomly from the nearest node
    theta = uniform(0, 2 * np.pi)
    new_point = (nearest_node.point[0] + int(step_size * np.cos(theta)),
                 nearest_node.point[1] + int(step_size * np.sin(theta)))
    if is_valid(new_point, obstacle_map):
        return new_point
    return None

def improved_rrt(start, goal, obstacle_map, step_size, max_iter=10000):
    # Performs the improved RRT algorithm to find a path from start to goal
    x, y = map_to_bottom_left(goal, 600)
    goal = (x, y)
    x, y = map_to_bottom_left(start, 600)
    start = (x, y)
    explored = np.zeros_like(obstacle_map[:, :, 0], dtype=bool)
    nodes = [Node(start)]
    explored[start[1], start[0]] = True

    for _ in range(max_iter):
        random_point = fast_sample(obstacle_map, nodes, step_size)
        nearest_node = min(nodes, key=lambda node: distance(node.point, random_point))
        new_point = steer(random_point, nearest_node.point, step_size, obstacle_map)

        if new_point is None:
            new_point = random_steer(nearest_node, step_size, obstacle_map)

        if new_point and not explored[new_point[1], new_point[0]]:
            new_node = Node(new_point, nearest_node)
            nodes.append(new_node)
            explored[new_point[1], new_point[0]] = True

            if (obstacle_map[map_to_bottom_left(new_point, 600)[1], map_to_bottom_left(new_point, 600)[0]] == [255, 0, 0]).all():
                return trace_path(new_node)

            cv2.line(obstacle_map, map_to_bottom_left(nearest_node.point, obstacle_map.shape[0]),
                    map_to_bottom_left(new_point, obstacle_map.shape[0]), (0, 255, 0), 1)
            cv2.circle(obstacle_map, map_to_bottom_left(new_point, obstacle_map.shape[0]), 2, (0, 0, 255), -1)

    return None

def trace_path(node):
    # Traces the path from the goal node to the start node
    path = []
    while node:
        path.append(node.point)
        node = node.parent
    return path[::-1]

def fast_rrt(start, goal, obstacle_map, step_size):
    # Performs the fast RRT algorithm to find a path from start to goal
    path = improved_rrt(start, goal, obstacle_map, step_size)
    if path:
        return path
    if not path:
        return None

def main():
    width, height = 1400, 600
    clearance = 180
    obstacle_map = np.ones((height, width, 3), dtype=np.uint8) * 255
    start = (100, height - 200)
    goal = (1340, height - 80)
    step_size = 30

    obstacles = [
        {'shape': 'rectangle', 'vertices': [(int((600-clearance)/5) + 100, int((400-clearance)/5)), (int((600-clearance)/5) + 100, int((700+clearance)/5)), (int((900+clearance)/5) + 100, int((700+clearance)/5)), (int((900+clearance)/5) + 100, int((400-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(600/5) + 100, int(400/5)), (int(600/5) + 100, int(700/5)), (int(900/5) + 100, int(700/5)), (int(900/5) + 100, int(400/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int((1500-clearance)/5) + 100, int((1200-clearance)/5)), (int((1500-clearance)/5) + 100, int((3000+clearance)/5)), (int((1520+clearance)/5) + 100, int((3000+clearance)/5)), (int((1520+clearance)/5) + 100, int((1200-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(1500/5) + 100, int(1200/5)), (int(1500/5) + 100, int(3000/5)), (int(1520/5) + 100, int(3000/5)), (int(1520/5) + 100, int(1200/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int((2100-clearance)/5) + 100, int((400-clearance)/5)), (int((2100-clearance)/5) + 100, int((700+clearance)/5)), (int((2400+clearance)/5) + 100, int((700+clearance)/5)), (int((2400+clearance)/5) + 100, int((400-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(2100/5) + 100, int(400/5)), (int(2100/5) + 100, int(700/5)), (int(2400/5) + 100, int(700/5)), (int(2400/5) + 100, int(400/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int((2100-clearance)/5) + 100, int((2100-clearance)/5)), (int((2100-clearance)/5) + 100, int((2400+clearance)/5)), (int((2400+clearance)/5) + 100, int((2400+clearance)/5)), (int((2400+clearance)/5) + 100, int((2100-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(2100/5) + 100, int(2100/5)), (int(2100/5) + 100, int(2400/5)), (int(2400/5) + 100, int(2400/5)), (int(2400/5) + 100, int(2100/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int((3000-clearance)/5) + 100, int((0-clearance)/5)), (int((3000-clearance)/5) + 100, int((1800+clearance)/5)), (int((3020+clearance)/5) + 100, int((1800+clearance)/5)), (int((3020+clearance)/5) + 100, int((0-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(3000/5) + 100, int(0/5)), (int(3000/5) + 100, int(1800/5)), (int(3020/5) + 100, int(1800/5)), (int(3020/5) + 100, int(0/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int((3600-clearance)/5) + 100, int((400-clearance)/5)), (int((3600-clearance)/5) + 100, int((700+clearance)/5)), (int((3900+clearance)/5) + 100, int((700+clearance)/5)), (int((3900+clearance)/5) + 100, int((400-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(3600/5) + 100, int(400/5)), (int(3600/5) + 100, int(700/5)), (int(3900/5) + 100, int(700/5)), (int(3900/5) + 100, int(400/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int((3600-clearance)/5) + 100, int((2100-clearance)/5)), (int((3600-clearance)/5) + 100, int((2400+clearance)/5)), (int((3900+clearance)/5) + 100, int((2400+clearance)/5)), (int((3900+clearance)/5) + 100, int((2100-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(3600/5) + 100, int(2100/5)), (int(3600/5) + 100, int(2400/5)), (int(3900/5) + 100, int(2400/5)), (int(3900/5) + 100, int(2100/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int((5100-clearance)/5) + 100, int((400-clearance)/5)), (int((5100-clearance)/5) + 100, int((700+clearance)/5)), (int((5400+clearance)/5) + 100, int((700+clearance)/5)), (int((5400+clearance)/5) + 100, int((400-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(5100/5) + 100, int(400/5)), (int(5100/5) + 100, int(700/5)), (int(5400/5) + 100, int(700/5)), (int(5400/5) + 100, int(400/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int((5100-clearance)/5) + 100, int((2100-clearance)/5)), (int((5100-clearance)/5) + 100, int((2400+clearance)/5)), (int((5400+clearance)/5) + 100, int((2400+clearance)/5)), (int((5400+clearance)/5) + 100, int((2100-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(5100/5) + 100, int(2100/5)), (int(5100/5) + 100, int(2400/5)), (int(5400/5) + 100, int(2400/5)), (int(5400/5) + 100, int(2100/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int((4500-clearance)/5) + 100, int((1200-clearance)/5)), (int((4500-clearance)/5) + 100, int((3000+clearance)/5)), (int((4520+clearance)/5) + 100, int((3000+clearance)/5)), (int((4520+clearance)/5) + 100, int((1200-clearance)/5))], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(int(4500/5) + 100, int(1200/5)), (int(4500/5) + 100, int(3000/5)), (int(4520/5) + 100, int(3000/5)), (int(4520/5) + 100, int(1200/5))], 'color': (0, 0, 255), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(0, 0), (0, int(clearance/5)), (1400, int(clearance/5)), (1400, 0)], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(0, 0), (0, 600), (int(clearance/5), 600), (int(clearance/5), 0)], 'color': (128, 128, 128), 'thickness': 1},  
        {'shape': 'rectangle', 'vertices': [(1400-int(clearance/5), 0), (1400-int(clearance/5), 600), (1400, 600), (1400, 0)], 'color': (128, 128, 128), 'thickness': 1}, 
        {'shape': 'rectangle', 'vertices': [(0, 600-int(clearance/5)), (0, 600), (1400, 600), (1400, 600-int(clearance/5))], 'color': (128, 128, 128), 'thickness': 1},
        {'shape': 'circle', 'vertices': [(1340, 80), 30], 'color': (255, 0, 0), 'thickness': 2},
    ]

    for obstacle in obstacles:
        if obstacle['shape'] == 'rectangle':
            pts = np.array([map_to_bottom_left((x, y), height) for x, y in obstacle['vertices']], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.fillPoly(obstacle_map, [pts], obstacle['color'])
        elif obstacle['shape'] == 'circle':
            center, radius = obstacle['vertices']
            cv2.circle(obstacle_map, map_to_bottom_left(center, height), radius, obstacle['color'], -1)

    path = fast_rrt(start, goal, obstacle_map, step_size)
    
    end_time = time.time()
    
    execution_time = end_time - start_time
    print(f"Execution time: {execution_time:.6f} seconds")
    
    if path:
        for i in range(len(path) - 1):
            cv2.line(obstacle_map, map_to_bottom_left(path[i], height), map_to_bottom_left(path[i+1], height), (255, 0, 0), 2)

    cv2.imshow("Path Found", obstacle_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
