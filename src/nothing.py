#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from miro2.lib import wheel_speed2cmd_vel

class MiRoPatrolPID:
    def __init__(self):
        rospy.init_node("miro_patrol_pid", anonymous=True)
        self.robot_name = "/" + rospy.get_param("~robot_name", "miro02")

        # Movement parameters
        self.LINEAR_SPEED = 0.2
        self.WHEEL_BASE = 0.14  # meters
        self.TURN_ANGLE_RAD = math.radians(90)
        self.FORWARD_DURATION = 10.0  # seconds to move one side of the square

        # PID parameters
        self.pid_kp = 2.0
        self.pid_ki = 0.0
        self.pid_kd = 0.2
        self.pid_integral = 0.0
        self.pid_last_error = 0.0

        # Robot state
        self.yaw = 0.0
        self.yaw_target = None
        self.pose_received = False

        # ROS setup
        self.cmd_pub = rospy.Publisher(self.robot_name + "/control/cmd_vel", TwistStamped, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.robot_name + "/sensors/odom", Odometry, self.callback_pose)

        # Wait for odometry
        rospy.loginfo("Waiting for odometry...")
        while not self.pose_received and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Odometry received. Starting patrol...")

        self.patrol_loop()

    def callback_pose(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)
        self.pose_received = True

    def pid_correction(self, target, current, dt):
        error = math.atan2(math.sin(target - current), math.cos(target - current))
        self.pid_integral += error * dt
        derivative = (error - self.pid_last_error) / dt if dt > 0 else 0.0
        self.pid_last_error = error
        return self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * derivative

    def publish_movement(self, speed_l, speed_r, duration):
        twist_msg = TwistStamped()
        (v, w) = wheel_speed2cmd_vel([speed_l, speed_r])
        twist_msg.twist.linear.x = v
        twist_msg.twist.angular.z = w
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(20)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist_msg)
            rate.sleep()

    def stop(self):
        self.publish_movement(0.0, 0.0, 0.1)

    def move_straight_pid(self, duration):
        self.yaw_target = self.yaw
        rate = rospy.Rate(50)
        start_time = rospy.Time.now()
        last_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now
            correction = self.pid_correction(self.yaw_target, self.yaw, dt)
            speed_l = self.LINEAR_SPEED - correction * self.WHEEL_BASE / 2.0
            speed_r = self.LINEAR_SPEED + correction * self.WHEEL_BASE / 2.0
            self.publish_movement(speed_l, speed_r, 0.02)
            rate.sleep()
        self.stop()

    def turn_pid(self, angle_rad):
        self.yaw_target = self.yaw + angle_rad
        self.yaw_target = math.atan2(math.sin(self.yaw_target), math.cos(self.yaw_target))  # normalize
        rate = rospy.Rate(50)
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now
            correction = self.pid_correction(self.yaw_target, self.yaw, dt)
            if abs(self.yaw - self.yaw_target) < math.radians(1.5):
                break
            speed_l = -correction * self.WHEEL_BASE / 2.0
            speed_r = correction * self.WHEEL_BASE / 2.0
            self.publish_movement(speed_l, speed_r, 0.02)
            rate.sleep()
        self.stop()

    def patrol_loop(self):
        while not rospy.is_shutdown():
            for i in range(4):  # square path
                rospy.loginfo(f"Moving straight (leg {i+1})")
                self.move_straight_pid(self.FORWARD_DURATION)
                rospy.sleep(0.5)
                rospy.loginfo("Turning 90 degrees")
                self.turn_pid(self.TURN_ANGLE_RAD)
                rospy.sleep(0.5)

if __name__ == "__main__":
    try:
        MiRoPatrolPID()
    except rospy.ROSInterruptException:
        pass
