import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialWriterNode(Node):
    def __init__(self):
        super().__init__('serial_writer')

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("subscribe_to", "/serial_data")

        port, baud_rate, subscribe_topic = self.read_parameters()

        self.serial_comm = serial.Serial()
        self.close_serial_port()  # Ensure the serial port is closed before opening
        self.connect_serial_port(serial_port=port, baud_rate=baud_rate)

        self.data_subscriber = self.create_subscription(String, subscribe_topic, self.write_serial_data, 10)

        self.get_logger().info(f"Serial writer node has started. Subscribing to: {subscribe_topic}")

    def read_parameters(self):
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        subscribe_topic = self.get_parameter("subscribe_to").get_parameter_value().string_value

        self.get_logger().info(f"Port: {serial_port}, Baud Rate: {baud_rate}, Subscribeing To: {subscribe_topic}")

        return serial_port, baud_rate, subscribe_topic

    def connect_serial_port(self, serial_port, baud_rate):
        self.serial_comm.port = serial_port
        self.serial_comm.baudrate = baud_rate
        self.serial_comm.timeout = 1
        self.serial_comm.open()

    def close_serial_port(self):
        if self.serial_comm.is_open:
            self.serial_comm.close()
            self.get_logger().info("Serial port closed.")

    def write_serial_data(self, msg: String):
        try:
            msg_data = str(msg.data)
            self.serial_comm.write(msg_data.encode("utf-8"))
        except Exception as e:
            self.get_logger().warn("Serial communication error - Writing")
            self.get_logger().warn(repr(e))

def main(args=None):
    rclpy.init(args=args)

    node = SerialWriterNode()
    
    try:
        rclpy.spin(node)
    finally:
        node.close_serial_port()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
