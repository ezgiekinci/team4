import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class LiveOdomPlot(Node):
    def __init__(self):
        super().__init__('live_odom_plot')
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
    node = LiveOdomPlot()

    plt.ion()  # Interactive mode
    fig, ax = plt.subplots()
    line, = ax.plot([], [], '-', color='r', label="Trajectory")
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.set_title("Robot Live Trajectory")
    ax.grid(True)
    ax.legend()

    # Sabit eksen limitleri
    ax.set_xlim(11.750, 12.05)
    ax.set_ylim(7.25, 7.55)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # Yeni mesajları oku

            if node.x_data and node.y_data:
                line.set_data(node.x_data, node.y_data)
                plt.pause(0.01)  # Grafik güncellenir

    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

