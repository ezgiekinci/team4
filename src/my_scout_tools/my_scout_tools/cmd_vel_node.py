#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_node')

        # ---- Parameters ----
        # Hedef hızlar (ÖDEV ŞARTI)
        self.declare_parameter('vx', 1.0)        # m/s (linear x)
        self.declare_parameter('wz', 0.5)        # rad/s (angular z)
        self.declare_parameter('rate_hz', 10)    # Hz
        # Ramp süresi: tam hıza kaç saniyede çıksın?
        # 0.0 veya negatif verirsen RAMP KAPANIR (direkt hedef hıza geçer)
        self.declare_parameter('ramp_time', 0.0)  # s

        # Param değerlerini oku
        self.vx_target = float(self.get_parameter('vx').value)
        self.wz_target = float(self.get_parameter('wz').value)
        rate_hz = int(self.get_parameter('rate_hz').value)
        self.ramp_time = float(self.get_parameter('ramp_time').value)

        # Anlık hızlar
        self.vx_current = 0.0
        self.wz_current = 0.0

        # Zaman adımı
        self.dt = 1.0 / rate_hz

        # Ramp adımı
        if self.ramp_time > 0.0:
            # ramp_time sonunda target’a ulaşacak şekilde step hesapla
            self.vx_step = abs(self.vx_target) * self.dt / self.ramp_time
            self.wz_step = abs(self.wz_target) * self.dt / self.ramp_time
        else:
            # ramp yok: current'ı baştan target’a kur, step'e gerek yok
            self.vx_step = 0.0
            self.wz_step = 0.0
            self.vx_current = self.vx_target
            self.wz_current = self.wz_target

        # Publisher & timer
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.publish_cmd)

        self.get_logger().info(
            f'CmdVelNode started. Target: vx={self.vx_target} m/s, '
            f'wz={self.wz_target} rad/s, rate={rate_hz} Hz, '
            f'ramp_time={self.ramp_time} s'
        )

    def _ramp_value(self, current, target, step):
        """current değerini target’a doğru step kadar yaklaştır."""
        dv = target - current
        if abs(dv) <= step:
            return target
        if dv > 0.0:
            return current + step
        else:
            return current - step

    def publish_cmd(self):
        # Ramp açıksa yumuşak git, kapalıysa direk target
        if self.ramp_time > 0.0:
            self.vx_current = self._ramp_value(
                self.vx_current, self.vx_target, self.vx_step
            )
            self.wz_current = self._ramp_value(
                self.wz_current, self.wz_target, self.wz_step
            )
        else:
            self.vx_current = self.vx_target
            self.wz_current = self.wz_target

        msg = Twist()
        msg.linear.x = self.vx_current
        msg.angular.z = self.wz_current
        # diğer bileşenleri sıfırla (garanti olsun)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

