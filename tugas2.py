import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoverNode(Node):
    def __init__(self):
        super().__init__('mover_node')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()

        self.t_maju_panjang = 5.0   
        self.t_maju_pendek  = 2.0   
        self.t_putar        = 4.1

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        t1 = self.t_maju_panjang
        t2 = t1 + self.t_putar
        t3 = t2 + self.t_maju_pendek
        t4 = t3 + self.t_putar
        t5 = t4 + self.t_maju_panjang
        t6 = t5 + self.t_putar
        t7 = t6 + self.t_maju_pendek
        t8 = t7 + self.t_putar

        if elapsed_time < t1:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Maju (Sisi Panjang 1)...')
        elif elapsed_time < t2:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Putar 90 Derajat (1)...')
        elif elapsed_time < t3:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Maju (Sisi Pendek 1)...')
        elif elapsed_time < t4:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Putar 90 Derajat (2)...')
        elif elapsed_time < t5:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Maju (Sisi Panjang 2)...')
        elif elapsed_time < t6:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Putar 90 Derajat (3)...')
        elif elapsed_time < t7:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Maju (Sisi Pendek 2)...')
        elif elapsed_time < t8:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Putar 90 Derajat (4)...')
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Persegi Panjang Selesai. Berhenti.')
            self.publisher_.publish(msg)
            
            self.timer.cancel()
            rclpy.shutdown()
            return

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
