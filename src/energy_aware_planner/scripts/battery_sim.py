#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import select
import tty
import termios

class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_sim')
        self.publisher_ = self.create_publisher(Float32, '/battery', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.current_battery = 100.0
        print("\n" + "="*30)
        print("ENERGY-AWARE BATTERY SIMULATOR")
        print("="*30)
        print("CONTROLS:")
        print("  [d] -> Drop battery by 10%")
        print("  [r] -> Reset battery to 100%")
        print("  [q] -> Quit")
        print("-" * 30)

    def timer_callback(self):
        msg = Float32()
        msg.data = self.current_battery
        self.publisher_.publish(msg)
        
        # Non-blocking keyboard check
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 'd':
                self.current_battery = max(0.0, self.current_battery - 10.0)
                print(f"DEBUG: Battery dropped to {self.current_battery}%")
            elif key == 'r':
                self.current_battery = 100.0
                print("DEBUG: Battery reset to 100%")
            elif key == 'q':
                sys.exit(0)

def main(args=None):
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rclpy.init(args=args)
        node = BatterySim()
        rclpy.spin(node)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()