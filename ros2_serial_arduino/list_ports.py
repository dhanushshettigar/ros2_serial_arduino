import rclpy
from rclpy.node import Node
from serial.tools import list_ports

class SerialNode(Node):
    """
    ROS 2 Node for handling serial ports.
    """

    def __init__(self):
        """
        Constructor for SerialNode class.
        """
        super().__init__('serial_node')
        self.port_list = []

    def find_ports(self):
        """
        Find available serial ports.
        """
        self.port_list = [str(port) for port in list_ports.comports()]

    def print_ports(self):
        """
        Print the list of available serial ports.
        """
        self.get_logger().info("Found ports:")
        for port in self.port_list:
            self.get_logger().info(port)

    def main(self):
        """
        Main function to run the node.
        """
        self.find_ports()
        self.print_ports()
        rclpy.spin_once(self)

def main():
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init()
    serial_node = SerialNode()
    serial_node.main()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
