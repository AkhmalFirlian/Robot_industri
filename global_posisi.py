import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class fab_vel_global_position(Node):

    def __init__(self):
        super().__init__('fab_vel_global_position')

        # Enable this for More Debug Info (Wheel Speed, Linear Speed, Angular Speed, DT, PosX, PosY, Tetha)
        self.ShowMoreDebugInfo = False

        self.wheel_radius = 0.03
        self.wheel_separation = 0.17

        # Declaration DeltaTime Variables
        self.DT = 0.0
        self.CurrentTime = 0.0
        self.PrevTime = 0.0

        # Declaration Robin Transforms Variable
        self.PosX = 0.0
        self.PosY = 0.0
        self.Tetha = 0.0

        # Wheel Velocity Variables
        self.Vel_WL = Float64()
        self.Vel_WR = Float64()

        # Gathering Data From input_ik and Wheels
        self.create_subscription(
            Twist,
            '/input_ik',
            self.global_position_calculation,
            10
        )
        self.create_subscription(
            Float64,
            '/left_wheel/command',
            self.LeftWheelCallback,
            10
        )
        self.create_subscription(
            Float64,
            '/right_wheel/command',
            self.RightWheelCallback,
            10
        )

        #self.create_timer(0.1, self.publish_sheels)

        self.get_logger().info(
            f'ri_global_position/fab_vel_global_position.py is running...'
        )

    def LeftWheelCallback(self, msg):
        self.Vel_WL.data = msg.data
        if self.ShowMoreDebugInfo:
            self.get_logger().info(
                f'Left Wheel Velocity: {self.Vel_WL.data} | Right Wheel Velocity: {self.Vel_WR}'
            )
        

    def RightWheelCallback(self, msg):
        self.Vel_WR.data = msg.data
        if self.ShowMoreDebugInfo:
            self.get_logger().info(
                f'Right Wheel Velocity: {self.Vel_WR.data} | Left Wheel Velocity: {self.Vel_WL}'
            )

    def global_position_calculation(self, msg):

        self.PrevTime = self.CurrentTime
        self.CurrentTime = self.get_clock().now().nanoseconds * 1e-9
        self.DT = self.CurrentTime - self.PrevTime

        if self.CurrentTime == 0:
            return

        # Speed Calculations
        ThisLinearSpeed = (self.Vel_WL.data + self.Vel_WR.data) * (self.wheel_radius / 2)
        ThisAngularSpeed = msg.angular.z

        if self.ShowMoreDebugInfo:
            self.get_logger().info(
                f'[LOGGER] Linear Speed: {ThisLinearSpeed} m/s'
            )
            self.get_logger().info(
                f'[LOGGER] Angular: {msg.angular.z} rad/s'
            )

        ThisXDot = ThisLinearSpeed * math.cos(self.Tetha)
        ThisYDot = ThisLinearSpeed * math.sin(self.Tetha)
        self.Tetha += ThisAngularSpeed * self.DT
        self.Tetha = math.atan2(math.sin(self.Tetha), math.cos(self.Tetha)) # Normalize

        formatDT = "{:.2f}".format(self.DT)
        formatVelX = "{:.2f}".format(ThisXDot)
        formatVelY = "{:.2f}".format(ThisYDot)
        formatAngularSpeed = "{:.2f}".format(ThisAngularSpeed)

        self.get_logger().info(
            f'[LOGGER] DeltaTime: {formatDT}: '
            f'VelX: {formatVelX} m/s, VelY: {formatVelY} m/s '
            f'Omega: {formatAngularSpeed} rad/s'
        )

def main(args=None):
    rclpy.init(args=args)

    node = fab_vel_global_position()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
