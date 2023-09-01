import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class VolumeDispPublisher(Node):
    def __init__(self, floater_names):
        super().__init__('volume_disp_publisher')
        self.floater_publishers = {}  # Dictionary to store publishers for each floater
        self.value_toggle = False
        

        for floater_name in floater_names:
            topic_name = f'/{floater_name}/volume_disp'
            self.floater_publishers[floater_name] = self.create_publisher(Float32MultiArray, topic_name, 10)

        self.timer = self.create_timer(60.0, self.publish_data)
        
                                                                                     
    def publish_data(self):
        value_1, value_2 = 0.0, 20.0

        for floater_name, publisher in self.floater_publishers.items():
            value_to_publish = value_1 if self.value_toggle else value_2

            self.get_logger().info(f"Publishing to {floater_name}: {value_to_publish}")

            volume_msg = Float32MultiArray()
            volume_msg.data = [value_to_publish]

            publisher.publish(volume_msg)

        self.value_toggle = not self.value_toggle

def main(args=None):
    rclpy.init(args=args)

    floater_names = ['floater', 'floater_2']  # Add more floater names as needed
    node = VolumeDispPublisher(floater_names)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
