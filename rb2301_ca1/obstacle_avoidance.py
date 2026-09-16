import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

# Gate tuning. Distances in metres, tick counts at the 20 Hz timer rate.
FRONT_BLOCK = 0.55  # front nearer than this -> start dodging
FRONT_CLEAR = 0.65  # front further than this -> episode is over (hysteresis)
SIDE_BLOCK = 0.25  # the side we are strafing into counts as closed
SIDE_OPEN = 0.35  # a side this open is enough to unfreeze us
FLIP_MARGIN = 0.15  # opposite side must beat the chosen side by this to flip
TRAP_TICKS = 5  # ~0.25 s of a blocked side before we believe we are trapped
FREEZE_RETRY_TICKS = 60  # ~3 s frozen -> re-arm one flip and retry. None = freeze forever

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message

        self.reset_episode()  # Sets up the dodge state machine

        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        self.last_scan = np.array(msg.ranges)[::20] # Slices the 721 scan array to return only 36 scans. Feel free to edit

    def reset_episode(self):
        """Back to cruising, with the flip gate closed until the front blocks again."""
        self.state = "CRUISE"
        self.dodge = 0  # +1 strafes left, -1 strafes right, 0 means "not dodging"
        self.flips_left = 0  # the gate: one direction change per blocked episode
        self.trap_ticks = 0
        self.freeze_ticks = 0
        self.freeze_dir = 0  # the direction we were strafing into when we froze

    def enter_dodge(self, left, right):
        """Commit to the freer side and hand out one flip."""
        self.state = "DODGE"
        self.dodge = 1 if left > right else -1
        self.flips_left = 1
        self.trap_ticks = 0
        self.freeze_ticks = 0

    def drive_dodge(self, front, left, right):
        """Creep forward while sidestepping so the obstacle leaves the front
        sector sooner. Back out instead if we are already very close, or if
        both sides are too tight to slip through."""
        forward = -0.15 if (front < 0.20 or max(left, right) < 0.25) else 0.10
        self.move_2D(forward, 0.30 * self.dodge, 0.0)

    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        # --- 1. Clean up the scan --------------------------------------------
        # inf/nan mean the beam hit nothing. Anything below the lidar's 0.05 m
        # minimum range is a bogus reading (the URDF warns the last beam can
        # come back as a phantom 0.05). Treat both as "far away".
        scan = np.array(self.last_scan, dtype=float)
        scan[~np.isfinite(scan) | (scan < 0.06)] = 10.0

        # Helper function to set as the detection as angles instead of just indexing
        # The lidar is mounted rotated 180 deg (laser_joint rpy is 0 0 3.1416),
        # so index 0 points straight ahead and the index grows anticlockwise.
        deg = 360.0 / len(scan)  # degrees between beams (~10)

        def arc(lo, hi):
            """Nearest range within [lo, hi] degrees. 0 is front, +ve is left."""
            i = np.arange(round(lo / deg), round(hi / deg) + 1)
            return scan.take(i, mode="wrap").min()  # wrap handles -ve indices

        front = arc(-25, 25)
        left = arc(20, 80)  # the beams we would strafe into, not just +-90 deg
        right = arc(-80, -20)  # narrow these to +-30..80 if freezing is too eager

        # --- 2. Dodge state machine ------------------------------------------
        # CRUISE -> DODGE when the front blocks. The dodge direction is
        # committed so the robot does not flip-flop every tick, but the gate
        # (flips_left) buys exactly one reversal: if the side we chose closes
        # up while the front is still blocked and the opposite side is clearly
        # more open, we turn around and run that way instead. Spend the gate
        # and get boxed in again -> FROZEN.
        if self.state == "CRUISE":
            if front < FRONT_BLOCK:
                self.enter_dodge(left, right)
                self.drive_dodge(front, left, right)
            else:  # clear ahead: cruise
                self.move_2D(0.25, 0.0, 0.0)

        elif self.state == "DODGE":
            if front > FRONT_CLEAR:  # we slipped past, forget the old dodge
                self.reset_episode()
                self.move_2D(0.25, 0.0, 0.0)
            else:
                chosen = left if self.dodge > 0 else right
                other = right if self.dodge > 0 else left
                # Debounced so one noisy beam cannot trigger a reversal.
                self.trap_ticks = self.trap_ticks + 1 if chosen < SIDE_BLOCK else 0

                if self.trap_ticks < TRAP_TICKS:
                    self.drive_dodge(front, left, right)
                elif self.flips_left > 0 and other > chosen + FLIP_MARGIN:
                    self.dodge = -self.dodge
                    self.flips_left -= 1
                    self.trap_ticks = 0
                    self.get_logger().info(
                        f"Trapped ({chosen:.2f} m) - flipping to "
                        f"{'left' if self.dodge > 0 else 'right'} ({other:.2f} m)"
                    )
                    self.drive_dodge(front, left, right)
                else:
                    # Gate already spent, or neither side is any better.
                    self.state = "FROZEN"
                    self.freeze_ticks = 0
                    self.freeze_dir = self.dodge
                    self.get_logger().info(
                        f"Boxed in (front {front:.2f}, left {left:.2f}, "
                        f"right {right:.2f}) - freezing"
                    )
                    self.move_2D(0.0, 0.0, 0.0)

        else:  # FROZEN
            # Only the side that trapped us counts. The side we came from is
            # still open by definition, so testing max(left, right) here would
            # unfreeze instantly and oscillate.
            blocked_side = left if self.freeze_dir > 0 else right
            if front > FRONT_CLEAR or blocked_side > SIDE_OPEN:
                self.get_logger().info("Path opened up - resuming")
                self.reset_episode()
                self.move_2D(0.0, 0.0, 0.0)
            elif FREEZE_RETRY_TICKS is not None and self.freeze_ticks >= FREEZE_RETRY_TICKS:
                self.get_logger().info("Freeze timed out - retrying the dodge")
                self.enter_dodge(left, right)  # re-arms the gate
                self.drive_dodge(front, left, right)
            else:
                self.freeze_ticks += 1
                self.move_2D(0.0, 0.0, 0.0)  # keep publishing zeros, do not just stop

        self.get_logger().debug(
            f"{self.state:6s} front {front:.2f} left {left:.2f} right {right:.2f} "
            f"dodge {self.dodge:+d} flips {self.flips_left} | "
            f"nearest beam at {np.argmin(scan) * deg:.0f} deg"
        )

        ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
