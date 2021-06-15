import random as r
import cv2
import numpy as np
import time
from Node import Node

class RRT:
    def __init__(self, mmap, start_point, goal_point, actual_map_size, number_of_nodes=2000, max_length=20, goal_radius=20, collision_dist=25):
        # max_length=20, goal_radius=20, collision_dist=25
        # max_length=5, goal_radius=5, collision_dist=5
        self.mmap = mmap
        self.max_length = max_length
        self.goal_radius = goal_radius
        self.kernel_size = collision_dist
        self.actual_map_size = actual_map_size
        self.area_map = None
        self.width, self.height = self.mmap.shape[1], self.mmap.shape[0]
        self.number_of_nodes = number_of_nodes
        self.goal = Node(goal_point)
        self.start = Node(start_point)
        self.node_list = [self.start]
        self.solution = []
        self.modified_solution = []

    def run(self):
        start = time.time()
        self.preprocess_map()
        while self.number_of_nodes >= 1:
            new_node = self.get_random_node()
            nearest_node = self.find_nearest_node(new_node)
            steered_node = self.steer(nearest_node, new_node)
            if Node.distance(steered_node, self.goal) <= self.goal_radius:
                self.goal.parent = nearest_node
                self.goal.cost = nearest_node.cost + Node.distance(self.goal, nearest_node)
                self.node_list.append(self.goal)
                self.formulate_path(self.goal)
                self.optimize_path()
                self.draw_graph()
                self.draw_solution()
                self.draw_nodes()
                break
            if self.are_not_obstructed(steered_node, nearest_node):
                steered_node.parent = nearest_node
                nearest_node.children.append(steered_node)
                steered_node.cost = steered_node.parent.cost + Node.distance(steered_node, nearest_node)
                self.node_list.append(steered_node)
                self.number_of_nodes -= 1
        if self.number_of_nodes < 1:
            print("No Solution Found..!!")

    def preprocess_map(self):
        self.area_map = cv2.cvtColor(self.mmap, cv2.COLOR_BGR2GRAY)
        self.area_map = cv2.erode(self.area_map, np.ones((self.kernel_size, self.kernel_size)))

    def get_random_node(self):
        return Node((r.randint(0, self.width - 1), r.randint(0, self.height - 1)))

    def find_nearest_node(self, check_node):
        nearest_node = None
        dist = None
        for i, node in enumerate(self.node_list):
            _dist = np.hypot(check_node.x-node.x, check_node.y-node.y)
            if dist is None or _dist < dist:
                dist = _dist
                nearest_node = node
        return nearest_node

    def are_not_obstructed(self, node1, node2):
        mask = np.zeros_like(self.area_map)
        mask = cv2.line(mask, (node1.x, node1.y), (node2.x, node2.y), 255, 1)
        return (mask == cv2.bitwise_and(self.area_map, self.area_map, mask=mask)).all()

    def steer(self, near, new):
        m, n = self.max_length, np.hypot(new.x-near.x, new.y-near.y) - self.max_length
        if n < 0:
            return new
        x, y = int((m * new.x + n * near.x) / (m + n)), int((m * new.y + n * near.y) / (m + n))
        return Node((x, y))

    def formulate_path(self, end_node):
        current_node = end_node
        while current_node != self.start:
            self.solution.append(current_node)
            current_node = current_node.parent
        self.solution.append(self.start)

    def optimize_path(self):
        length = len(self.solution)
        if length >= 3:
            i = 2
            self.modified_solution.append(self.solution[0])
            current_node = self.modified_solution[0]
            while i < length:
                if not self.are_not_obstructed(self.solution[i], current_node) or i == length-1:
                    self.modified_solution.append(self.solution[i - 1])
                    current_node = self.solution[i - 1]
                i += 1

    def draw_nodes(self, node_size=1):
        for node in self.node_list:
            self.mmap = cv2.circle(self.mmap, node(), node_size, (0, 0, 255), -1)

    def draw_solution(self):
        l = len(self.solution)
        for i in range(l-1):
            cv2.line(self.mmap, self.solution[i](), self.solution[i+1](), (0, 255, 0), 2)

    def draw_modified_solution(self):
        l = len(self.modified_solution)
        for i in range(l-1):
            cv2.line(self.mmap, self.modified_solution[i](), self.modified_solution[i+1](), (255, 255, 0), 2)

    def draw_graph(self):
        cv2.circle(self.mmap, self.start(), 10, (0, 255, 0), 3)
        cv2.putText(self.mmap, "Start", (self.start.x-10, self.start.y+35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.circle(self.mmap, self.goal(), 10, (0, 255, 0), 3)
        cv2.putText(self.mmap, "Goal", (self.goal.x+12, self.goal.y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        for node in self.node_list[1:]:
            cv2.line(self.mmap, node(), node.parent(), (255, 255, 0), 1)
            
            
if __name__ == "__main__":
    print("RRT Class")
