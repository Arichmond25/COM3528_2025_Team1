# patrol_robot.py
import random

class PatrolRobot:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
    
    def move(self):
        # Patrol robot moves in a predefined pattern (for now, a simple random movement)
        self.x += random.choice([-1, 0, 1])
        self.y += random.choice([-1, 0, 1])

    def get_position(self):
        return self.x, self.y
    
    def detect_intruder(self, intruder):
        # Check if the intruder is within range (simple example: distance of 2 units)
        intruder_x, intruder_y = intruder.get_position()
        distance = ((self.x - intruder_x)**2 + (self.y - intruder_y)**2)**0.5
        if distance <= 2:
            return True
        return False
    
    def alert_intruder(self, intruder):
        # Make a sound or alert the system that an intruder has been detected
        print(f"Cop robot {self.id} detected {intruder.id} at position {intruder.get_position()}")
        self.make_sound()

    def make_sound(self):
        # This is where you can implement the sound alert
        print("Cop robot makes a sound: Intruder detected!")
