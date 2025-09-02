# encoder_odom_bridge.py
import math, serial, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# '/dev/ttyUSB0' (tüm esplerin uzun adı aynı)
# SERIAL_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0' 
SERIAL_PORT = '/dev/ttyUSB1' 
BAUD = 115200

# Kalibrasyon (güncelle)
COUNTS_PER_REV = 1440.0        # 360 PPR × 4x
WHEEL_DIAMETER_M_L = 0.065
WHEEL_DIAMETER_M_R = 0.065
GEAR_REDUCTION_L = 1.0
GEAR_REDUCTION_R = 1.0
TRACK_WIDTH_M = 0.22

CPM_L = (COUNTS_PER_REV * GEAR_REDUCTION_L) / (math.pi * WHEEL_DIAMETER_M_L)
CPM_R = (COUNTS_PER_REV * GEAR_REDUCTION_R) / (math.pi * WHEEL_DIAMETER_M_R)

def wrap_pi(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class EncoderOdomBridge(Node):
    def __init__(self):
        super().__init__('esp32_encoder_bridge')
        self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
        self.pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.x = self.y = self.th = 0.0
        self.create_timer(0.001, self.tick)

    def tick(self):
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line:
            return

        # Ham satırı logla
        # self.get_logger().info(f"Raw data: {line}")

        # Format kontrolü
        if not line.startswith('CNT,'):
            return

        try:
            _, dL_s, dR_s, dt_ms_s = line.split(',', 3)
            dL = int(dL_s)
            dR = int(dR_s)
            dt_ms = float(dt_ms_s)
            dt = max(1e-3, dt_ms / 1000.0)

            # Parsed değerleri logla
            # self.get_logger().info(
            #     f"Parsed counts → Left: {dL}, Right: {dR}, Δt: {dt_ms} ms"
            # )
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}, line={line}")
            return

        # Adım uzunluklarını metreye çevir
        dsL = dL / CPM_L
        dsR = dR / CPM_R
        ds   = 0.5 * (dsL + dsR)
        v    = ds / dt
        omega = ((dsR - dsL) / TRACK_WIDTH_M) / dt
        dth   = omega * dt

        # Konum güncelle
        self.x  += ds * math.cos(self.th + 0.5 * dth)
        self.y  += ds * math.sin(self.th + 0.5 * dth)
        self.th = wrap_pi(self.th + dth)

        # Mesaj oluştur
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.th/2.0)
        msg.pose.pose.orientation.w = math.cos(self.th/2.0)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = EncoderOdomBridge()   # <-- doğru sınıf adı
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()