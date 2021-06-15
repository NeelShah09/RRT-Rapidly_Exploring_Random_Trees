import numpy as np

class Node:
    def __init__(self, point):
        self.x, self.y = point
        self.parent = None
        self.children = []
        self.cost = 0.0

    def __ne__(self, other):
        return (self.x != other.x) or (self.y != other.y)

    def __call__(self):
        return (self.x, self.y)

    def __str__(self):
        return "X : "+str(self.x)+" Y : "+str(self.y)

    @staticmethod
    def distance(node1, node2):
        return np.hypot(node1.x-node2.x, node1.y-node2.y)


if __name__ == "__main__":
    print("Node Class")
