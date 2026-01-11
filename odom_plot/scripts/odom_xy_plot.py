import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class OdometryPlot(Node):
    def __init__(self):
        super().__init__('odom_plot')
        self.x_data = []
        self.y_data = []
        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Robot durduğunda grafiği çiz
        plt.plot(node.x_data, node.y_data, marker='o')
        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")
        plt.title("Robot Trajectory")
        plt.grid(True)
        plt.show()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

