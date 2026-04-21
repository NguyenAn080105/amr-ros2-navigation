#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial

# [F0, F1, F2, L, R, B0, B1] → sensor fusion
INDEX_TO_SENSOR = {
    0: 'us_top_left',
    1: 'us_top_right',
    2: 'us_mid_1_left',
    3: 'us_mid_1_right',
    4: 'us_mid_2_left',
    5: 'us_bot_left',
    6: 'us_bot_right',
}

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        self.pubs = {
            name: self.create_publisher(Range, f'/ultrasonic/{name}', 10)
            for name in INDEX_TO_SENSOR.values()
        }

        self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.ser.reset_input_buffer()
        self.create_timer(0.02, self.read_serial)

    def make_range(self, frame_id: str, dist_cm: int) -> Range:
        msg = Range()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = 0.26
        msg.min_range       = 0.02
        msg.max_range       = 1.0
        msg.range           = float('inf') if dist_cm == 999 else dist_cm / 100.0
        return msg

    def read_serial(self):
        if not self.ser.in_waiting:
            return
        #self.get_logger().info('Got data from serial') 
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        #self.get_logger().info(f'Raw: {line}') 
        if not line.startswith('$') or '*' not in line:
            #self.get_logger().warn(f'Bad frame: {line}')
            return

        try:
            data, chk_str = line[1:].split('*')
            vals = [int(x) for x in data.split(',')]

            if len(vals) != 7:
                return

            chk_calc = vals[0] & 0xFF
            for v in vals[1:]:
                chk_calc ^= (v & 0xFF)

            if chk_calc != int(chk_str):
                return

            for idx, dist_cm in enumerate(vals):
                name = INDEX_TO_SENSOR[idx]
                self.pubs[name].publish(self.make_range(name, dist_cm))
                #self.get_logger().info(
                #f'F0={vals[0]} F1={vals[1]} F2={vals[2]} L={vals[3]} R={vals[4]} B0={vals[5]} B1={vals[6]}'
#)

        except (ValueError, IndexError):
            return

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()