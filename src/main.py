# main.py
import time
from patrol_robot import PatrolRobot
from escape_robot import EscapeRobot

def main():
    # Initialize the patrol robot (cop)
    patrol_robot = PatrolRobot(id="cop", x=0, y=0)
    
    # Initialize the intruder robots
    intruder1 = EscapeRobot(id="intruder_1", x=5, y=5)
    intruder2 = EscapeRobot(id="intruder_2", x=7, y=8)
    
    # Add robots to a list for easy management
    robots = [patrol_robot, intruder1, intruder2]
    
    # Simulate the patrolling and detection
    while True:
        for robot in robots:
            robot.move()  # Move each robot according to its behavior
        
        # Check if the patrol robot detects any intruder
        for robot in robots:
            if isinstance(robot, EscapeRobot) and patrol_robot.detect_intruder(robot):
                patrol_robot.alert_intruder(robot)
        
        # Add a delay for the simulation loop
        time.sleep(1)

if __name__ == "__main__":
    main()
