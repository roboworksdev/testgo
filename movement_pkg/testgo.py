#!/usr/bin/env python3
# Note: the file will only save edits in the designated locations
# editable locations are denoted by comments. DO NOT remove the
# generated comments
import rclpy
from rclpy.node import Node
from booster_interface.msg import LowState, LowCmd, MotorCmd


# add custom imports below
import math
import os
import time
# end custom imports

class Movement(Node):

    def __init__(self):
        super().__init__('movement')
        self.low_state_sub = self.create_subscription(LowState, '/low_state', self.low_state_callback, 10)
        self.low_cmd_pub = self.create_publisher(LowCmd, 'test_control', 10)

        # customize init function below
        # Clear any leftover pause flag from a previous run
        if os.path.exists('/tmp/movement_pause'):
            os.remove('/tmp/movement_pause')

        self.low_state = None
        self.last_state_time = None

        self.forward_speed = 0.30       # m/s (vx)
        self.backward_speed = 0.10      # m/s
        self.turn_speed = 0.40          # rad/s (vyaw)
        self.turn_cw_deg = 90.0         # degrees clockwise
        self.turn_acw_deg = 90.0        # degrees anti-clockwise

        # Non-blocking action tracking
        self._action_active = False
        self._action_type = None
        self._action_start_time = None
        self._action_duration = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Movement node started.')
        # end init customization

    def low_state_callback(self, msg):
        # add low_state_callback implementation below
        self.low_state = msg
        self.last_state_time = self.get_clock().now()
        # end low_state_callback implementation


    # add custom methods to the node below
    def get_imu_rpy(self):
        """Return (roll, pitch, yaw) from the latest LowState IMU, or None if unavailable."""
        if self.low_state is None:
            return None
        rpy = self.low_state.imu_state.rpy
        return (rpy[0], rpy[1], rpy[2])

    def control_loop(self):
        """Main control loop running at 10 Hz."""
        if os.path.exists('/tmp/movement_pause'):
            return

        # Stop if no recent state data
        state_stale = (self.last_state_time is None or
            (self.get_clock().now() - self.last_state_time).nanoseconds / 1e9 > 1.0)
        if state_stale:
            return

        # Non-blocking action in progress — let it finish
        if self._action_active:
            self._continue_action()
            return

        # user control_loop logic below
        pass
        # end user control_loop logic

    def _continue_action(self):
        """Continue a non-blocking timed rotation via LowCmd."""
        elapsed = time.time() - self._action_start_time
        if elapsed >= self._action_duration:
            self._action_active = False
            return

        cmd = LowCmd()
        cmd.cmd_type = 0  # CMD_TYPE_PARALLEL
        # Still rotating — placeholder for low-level joint commands
        self.low_cmd_pub.publish(cmd)

    def turn_cw(self, degrees=None):
        """Start a non-blocking clockwise turn."""
        if degrees is None:
            degrees = self.turn_cw_deg
        self._action_active = True
        self._action_type = 'cw'
        self._action_start_time = time.time()
        self._action_duration = math.radians(degrees) / self.turn_speed if self.turn_speed > 0 else 0
        self.get_logger().info(f'Turning {degrees} degrees clockwise.')

    def turn_acw(self, degrees=None):
        """Start a non-blocking anti-clockwise turn."""
        if degrees is None:
            degrees = self.turn_acw_deg
        self._action_active = True
        self._action_type = 'acw'
        self._action_start_time = time.time()
        self._action_duration = math.radians(degrees) / self.turn_speed if self.turn_speed > 0 else 0
        self.get_logger().info(f'Turning {degrees} degrees anti-clockwise.')

    def stop(self):
        """Publish a zero LowCmd to stop motion."""
        cmd = LowCmd()
        cmd.cmd_type = 0
        self.low_cmd_pub.publish(cmd)
        self._action_active = False

    def set_speed(self, speed):
        """Update the turn_speed parameter."""
        self.turn_speed = speed
    # end custom methods

def main(args=None):
    # customize main function below
    # Note: editing any of the generated code here could break the node.
    # take precaution when editing
    rclpy.init(args=args)
    movement = Movement()
    # Do not edit code above here

    try:
        rclpy.spin(movement)
    except KeyboardInterrupt:
        pass
    finally:
        movement.stop()
        time.sleep(0.1)  # let the stop message transmit before node teardown

    # Do not delete this
    movement.destroy_node()
    rclpy.shutdown()
    # end main customization


if __name__ == '__main__':
    main()
