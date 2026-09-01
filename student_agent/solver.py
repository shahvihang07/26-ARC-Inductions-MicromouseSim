"""
Micromouse Solver
Right-wall-hugging maze solver.

Priority:
    1. If front is blocked -> turn left
    2. If right side is open -> turn right
    3. Otherwise -> follow the right wall
"""

import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# ============================================================
# THESE FOUR PARAMETERS MUST ADD UP TO EXACTLY 30
# ============================================================

TOP_SPEED = 7
ACCELARATION = 7
TURN_SPEED = 7
SENSOR_RANGE = 9

# 7 + 7 + 7 + 9 = 30


# ============================================================
# WALL FOLLOWING SETTINGS
# ============================================================

# Desired distance from the right wall
RIGHT_WALL_DISTANCE = 0.55

# If front distance is below this -> front is blocked
FRONT_BLOCKED = 0.65

# If right distance is above this -> there is a right opening
RIGHT_OPEN = 0.90

# Forward speed
FORWARD_SPEED = 0.45

# Turning speed
TURN_SPEED_CMD = 1.5

# How long to turn at a corner
LEFT_TURN_TIME = 0.85
RIGHT_TURN_TIME = 0.85

# U-turn time
U_TURN_TIME = 1.7

# Small forward movement after completing a turn
FORWARD_AFTER_TURN = 0.35


class StudentSolver(Node):

    def __init__(self):

        super().__init__('student_solver')

        # ====================================================
        # SUBSCRIBER
        # ====================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/mouse/scan',
            self.scan_callback,
            10
        )

        # ====================================================
        # PUBLISHER
        # ====================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/mouse/cmd_vel',
            10
        )

        # ====================================================
        # TURN STATE
        # ====================================================

        self.turning = False

        self.turn_direction = 0.0

        self.turn_end_time = 0.0

        # ====================================================
        # AFTER TURN STATE
        # ====================================================

        self.forward_after_turn = False

        self.forward_end_time = 0.0

        # ====================================================
        # PREVENT REPEATED RIGHT TURNS
        # ====================================================

        self.last_turn = 0.0

        # ====================================================
        # INFORMATION
        # ====================================================

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "Right Wall Hugging Solver Started"
        )

        self.get_logger().info(
            f"TOP_SPEED = {TOP_SPEED}"
        )

        self.get_logger().info(
            f"ACCELARATION = {ACCELARATION}"
        )

        self.get_logger().info(
            f"TURN_SPEED = {TURN_SPEED}"
        )

        self.get_logger().info(
            f"SENSOR_RANGE = {SENSOR_RANGE}"
        )

        self.get_logger().info(
            f"TOTAL = "
            f"{TOP_SPEED + ACCELARATION + TURN_SPEED + SENSOR_RANGE}"
        )

        self.get_logger().info(
            "=========================================="
        )

    # ========================================================
    # START TURN
    # ========================================================

    def start_turn(self, direction, duration):

        self.turning = True

        self.turn_direction = direction

        self.turn_end_time = time.time() + duration

        self.last_turn = time.time()

    # ========================================================
    # SENSOR CALLBACK
    # ========================================================

    def scan_callback(self, msg):

        # ====================================================
        # READ LASER SENSORS
        #
        # [0] = LEFT
        # [1] = FRONT
        # [2] = RIGHT
        # ====================================================

        if len(msg.ranges) < 3:
            return

        d_left = msg.ranges[0]
        d_front = msg.ranges[1]
        d_right = msg.ranges[2]

        # ====================================================
        # CLEAN INVALID VALUES
        # ====================================================

        if d_left <= 0:
            d_left = SENSOR_RANGE

        if d_front <= 0:
            d_front = SENSOR_RANGE

        if d_right <= 0:
            d_right = SENSOR_RANGE

        # Limit sensor readings
        d_left = min(d_left, SENSOR_RANGE)
        d_front = min(d_front, SENSOR_RANGE)
        d_right = min(d_right, SENSOR_RANGE)

        # ====================================================
        # CREATE COMMAND
        # ====================================================

        cmd = Twist()

        now = time.time()

        # ====================================================
        # 1. CURRENTLY TURNING
        # ====================================================

        if self.turning:

            cmd.linear.x = 0.0

            cmd.angular.z = self.turn_direction

            if now >= self.turn_end_time:

                self.turning = False

                cmd.angular.z = 0.0

                # Move straight for a short time after turning
                self.forward_after_turn = True

                self.forward_end_time = (
                    now + FORWARD_AFTER_TURN
                )

            self.cmd_pub.publish(cmd)

            return

        # ====================================================
        # 2. MOVE FORWARD AFTER TURN
        # ====================================================

        if self.forward_after_turn:

            if now < self.forward_end_time:

                # Only move if front is clear
                if d_front > FRONT_BLOCKED:

                    cmd.linear.x = FORWARD_SPEED
                    cmd.angular.z = 0.0

                    self.cmd_pub.publish(cmd)

                    return

                else:

                    # Something is blocking us
                    self.forward_after_turn = False

            else:

                self.forward_after_turn = False

        # ====================================================
        # 3. FRONT BLOCKED
        #
        # Highest priority.
        # If we cannot go forward, turn LEFT.
        # ====================================================

        if d_front < FRONT_BLOCKED:

            # If right is also blocked, left turn is useful
            self.start_turn(
                TURN_SPEED_CMD,
                LEFT_TURN_TIME
            )

            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_direction

            self.cmd_pub.publish(cmd)

            return

        # ====================================================
        # 4. RIGHT OPEN
        #
        # Right-hand wall following:
        # If there is an opening on the right,
        # take it.
        #
        # But don't immediately repeat right turns.
        # ====================================================

        if d_right > RIGHT_OPEN:

            # Prevent another right turn immediately
            if now - self.last_turn > 0.8:

                self.start_turn(
                    -TURN_SPEED_CMD,
                    RIGHT_TURN_TIME
                )

                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_direction

                self.cmd_pub.publish(cmd)

                return

        # ====================================================
        # 5. FOLLOW RIGHT WALL
        #
        # Right wall too far -> steer RIGHT
        # Right wall too close -> steer LEFT
        # Otherwise -> straight.
        # ====================================================

        error = d_right - RIGHT_WALL_DISTANCE

        # Steering strength
        steering = error * 1.2

        # Limit steering
        if steering > 0.7:
            steering = 0.7

        if steering < -0.7:
            steering = -0.7

        # ====================================================
        # MOVE FORWARD
        # ====================================================

        cmd.linear.x = FORWARD_SPEED

        # Positive angular velocity = LEFT
        # Negative angular velocity = RIGHT
        #
        # If wall is too far:
        # error positive -> turn right
        #
        # If wall is too close:
        # error negative -> turn left
        cmd.angular.z = -steering

        # ====================================================
        # PUBLISH
        # ====================================================

        self.cmd_pub.publish(cmd)


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = StudentSolver()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Stop the robot
        stop_cmd = Twist()

        node.cmd_pub.publish(stop_cmd)

        node.destroy_node()

        rclpy.shutdown()


# ============================================================
# RUN
# ============================================================

if __name__ == '__main__':

    main()