# escape_robot.py
import random

class EscapeRobot:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y

    def move(self):
        # For now, just move randomly for demonstration
        self.x += random.choice([-1, 0, 1])
        self.y += random.choice([-1, 0, 1])

    def get_position(self):
        return self.x, self.y
