"""
Micromouse Solver

Right-hand wall-following maze solver.

Priority:
    RIGHT TURN > FORWARD > LEFT TURN > U-TURN
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# ==========================================
# These four parameters MUST add up to exactly 30!
# ==========================================

TOP_SPEED = 6
ACCELARATION = 8
TURN_SPEED = 6
SENSOR_RANGE = 10


# ==========================================
# Solver settings
# ==========================================

WALL_THRESHOLD = 0.80
FRONT_BLOCKED = 0.65

FORWARD_SPEED = 0.5

# Angular velocity
TURN_SPEED_CMD = 1.5

# At 1.5 rad/s:
# 1 second ≈ 86 degrees
RIGHT_TURN_TIME = 1.05
LEFT_TURN_TIME = 1.05
U_TURN_TIME = 2.10

# Small straight movement after turning
FORCED_FORWARD_TIME = 0.30


class StudentSolver(Node):

    def __init__(self):

        super().__init__('student_solver')

        # ==========================================
        # Subscriber
        # ==========================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/mouse/scan',
            self.scan_callback,
            10
        )

        # ==========================================
        # Publisher
        # ==========================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/mouse/cmd_vel',
            10
        )

        # ==========================================
        # Turn state
        # ==========================================

        self.turning = False
        self.turn_direction = 0.0
        self.turn_end_time = 0.0

        # ==========================================
        # Forward state
        # ==========================================

        self.forced_forward_until = 0.0

        self.get_logger().info(
            "Student Solver Node initialized successfully."
        )

        self.get_logger().info(
            f"Stats -> Speed: {TOP_SPEED}, "
            f"Accel: {ACCELARATION}, "
            f"Turn: {TURN_SPEED}, "
            f"Range: {SENSOR_RANGE}"
        )


    # ==========================================
    # Start a turn
    # ==========================================

    def start_turn(self, direction, duration):

        self.turning = True

        self.turn_direction = direction

        self.turn_end_time = time.time() + duration


    # ==========================================
    # Sensor callback
    # ==========================================

    def scan_callback(self, msg):

        # ------------------------------------------
        # Read sensors
        # ------------------------------------------

        d_left = msg.ranges[0]
        d_front = msg.ranges[1]
        d_right = msg.ranges[2]

        self.get_logger().info(
            f"L={d_left:.2f} F={d_front:.2f} R={d_right:.2f}"
)

        now = time.time()

        cmd = Twist()


        # ==========================================
        # 1. Currently turning
        # ==========================================

        if self.turning:

            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_direction

            # Turn finished
            if now >= self.turn_end_time:

                self.turning = False
                self.turn_direction = 0.0

                # Force a short straight movement
                self.forced_forward_until = (
                    now + FORCED_FORWARD_TIME
                )

                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            return


        # ==========================================
        # 2. Forced forward after a turn
        # ==========================================

        if now < self.forced_forward_until:

            # Safety: don't drive into a wall
            if d_front > FRONT_BLOCKED:

                cmd.linear.x = FORWARD_SPEED
                cmd.angular.z = 0.0

            else:

                self.forced_forward_until = 0.0

            self.cmd_pub.publish(cmd)

            return


        # ==========================================
        # 3. RIGHT-HAND WALL FOLLOWING
        # ==========================================

        # ------------------------------------------
        # Right open AND front blocked
        # -> Turn right
        # ------------------------------------------

        if d_right > WALL_THRESHOLD and d_front <= FRONT_BLOCKED:

            self.start_turn(
                -TURN_SPEED_CMD,
                RIGHT_TURN_TIME
            )

            cmd.linear.x = 0.0
            cmd.angular.z = -TURN_SPEED_CMD


        # ------------------------------------------
        # Front open
        # -> Go forward
        # ------------------------------------------

        elif d_front > FRONT_BLOCKED:

            cmd.linear.x = FORWARD_SPEED
            cmd.angular.z = 0.0


        # ------------------------------------------
        # Front blocked
        # Right blocked
        # Left open
        # -> Turn left
        # ------------------------------------------

        elif d_left > WALL_THRESHOLD:

            self.start_turn(
                TURN_SPEED_CMD,
                LEFT_TURN_TIME
            )

            cmd.linear.x = 0.0
            cmd.angular.z = TURN_SPEED_CMD


        # ------------------------------------------
        # Everything blocked
        # -> U-turn
        # ------------------------------------------

        else:

            self.start_turn(
                -TURN_SPEED_CMD,
                U_TURN_TIME
            )

            cmd.linear.x = 0.0
            cmd.angular.z = -TURN_SPEED_CMD


        # ==========================================
        # Publish command
        # ==========================================

        self.cmd_pub.publish(cmd)


# ==========================================
# Main
# ==========================================

def main(args=None):

    rclpy.init(args=args)

    node = StudentSolver()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


# ==========================================
# Run
# ==========================================

if __name__ == '__main__':
    main()