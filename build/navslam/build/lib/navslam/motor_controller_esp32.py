#!/usr/bin/env python3
import math, serial, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistToSerial(Node):
    def __init__(self):
        super().__init__('twist_to_serial')

        # Parametreler
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('track_width', 0.50)    # W (m)
        self.declare_parameter('max_lin', 1.0)         # m/s (tam gaz ileri)
        self.declare_parameter('max_ang', 2.0)         # rad/s (tam zıt paletler)
        self.declare_parameter('rate_limit', 2.0)      # %/ms; 0=kapalı
        self.declare_parameter('deadzone', 0.01)       # m/s ve rad/s için

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.W = float(self.get_parameter('track_width').value)
        self.vmax = float(self.get_parameter('max_lin').value)
        self.wmax = float(self.get_parameter('max_ang').value)
        self.rate_limit = float(self.get_parameter('rate_limit').value)
        self.deadzone = float(self.get_parameter('deadzone').value)

        # Seri
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
        time.sleep(0.5)

        self.last_cmd = (0.0, 0.0)  # son gönderilen (L%, R%) -100..100
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

        # Watchdog: cmd_vel gelmezse periyodik "STOP"
        self.timer = self.create_timer(0.1, self.keepalive)

        self.get_logger().info(f'[twist_to_serial] {port} @ {baud}, W={self.W}m Vmax={self.vmax} Wmax={self.wmax}')

    def saturate(self, x, lo, hi):
        return max(lo, min(hi, x))

    def rate_limit_percent(self, target, current, dt_ms):
        if self.rate_limit <= 0:
            return target
        delta = self.rate_limit * dt_ms
        return self.saturate(target, current - delta, current + delta)

    def keepalive(self):
        # küçük keepalive: ESP32 watchdog'u rahatlatır
        try:
            self.ser.write(b"PING\n")
        except Exception:
            pass

    def cb(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z

        # küçük gürültüleri öldür
        if abs(vx) < self.deadzone: vx = 0.0
        if abs(wz) < self.deadzone: wz = 0.0

        # Normalizasyon: verilen cmd'leri belirtilen maximumlara göre ölçekle
        vx = self.saturate(vx, -self.vmax, self.vmax)
        wz = self.saturate(wz, -self.wmax, self.wmax)

        # Skid-steer kinematik
        vL = vx - wz * (self.W/2.0)
        vR = vx + wz * (self.W/2.0)

        # Maksimuma göre yüzdeye çevir (tam ileri = +100, tam geri = -100)
        # Referans: tam zıt paletlerle dönmede |v| = vmax kabulü için ölçekleme
        scale = self.vmax
        pL = self.saturate( (vL/scale)*100.0, -100.0, 100.0 )
        pR = self.saturate( (vR/scale)*100.0, -100.0, 100.0 )

        # Oran limit (yumuşaklık)
        now = self.get_clock().now()
        dt_ms = 100.0  # kaba; daha hassas için zaman damgası tut
        pL = self.rate_limit_percent(pL, self.last_cmd[0], dt_ms)
        pR = self.rate_limit_percent(pR, self.last_cmd[1], dt_ms)
        self.last_cmd = (pL, pR)

        # Seri paket: "L:<-100..100> R:<-100..100>\n"
        cmd = f"L:{int(round(pL))} R:{int(round(pR))}\n"
        
        # Terminale Yazdırma (Eklenen satır)
        self.get_logger().info(f"Gönderilen komut: {cmd.strip()}")

        try:
            self.ser.write(cmd.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f'serial write err: {e}')

def main():
    rclpy.init()
    n = TwistToSerial()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.ser.write(b"L:0 R:0\n")
    n.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
