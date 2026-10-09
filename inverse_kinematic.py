import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class InverseKinematics(Node):
    def _init_(self):
        super()._init_('inverse_kinematics')
        
        self.wheel_radius = 0.03      
        self.wheel_separation = 0.17
        
        self.create_subscription(
            Twist, 
            '/input_ik', 
            self.velocity_callback, 
            10
        )

        self.left_publisher = self.create_publisher(
            Float64, 
            '/left_wheel/command', 
            10
        )
        self.right_publisher = self.create_publisher(
            Float64, 
            '/right_wheel/command', 
            10
        )
        
        self.get_logger().info('Inverse Kinematics aktif.')

    def velocity_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        
        v_left = v - (omega * self.wheel_separation) / 2.0
        v_right = v + (omega * self.wheel_separation) / 2.0
        
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius
        
        left_msg = Float64()
        left_msg.data = float(omega_left)
        self.left_publisher.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = float(omega_right)
        self.right_publisher.publish(right_msg)
        
        self.get_logger().info(f'Kiri: {omega_left:.2f} rad/s | Kanan: {omega_right:.2f} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if _name_ == '_main_':
    main()
