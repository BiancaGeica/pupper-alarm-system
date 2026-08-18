import rclpy # ROS Client Library for Python
from rclpy.node import Node # to make a ROS 2 node
from std_msgs.msg import String # to send and receive string messages
from tf2_ros import Buffer, TransformListener # to read the pose of the robot in the map

# tf = transform frames, keeps track of where is every frame in the map, reported to the old ones

TRESHOLD = 50 # Celsius 

class ThermalMapNode(Node):
    def __init__(self):
        super().__init__('thermal_map_node') # give a name to the ROS2 node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # to read the pose of the robot

        self.subscription = self.create_subscription(String, '/temperature_data', self.temperature_callback, 10)
        self.alert_publisher = self.create_publisher(String, '/alarm_event', 10)
        self.thermal_data = []

    def temperature_callback(self, msg): # call herself when a new message is received
        try:
            temp = float(msg.data)

            # read the position of the robot in the map
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # save the touple (x, y, temp)
            self.thermal_data.append((x, y, temp))
            self.get_logger().info(f'Temp: {temp}°C la pozitia: ({x:.2f}, {y:.2f})')

            if temp >= TRESHOLD:
                alert = String()
                alert.data = f'ALERTA TEMPERATURA! {temp}°C la pozitia ({x:.2f}, {y:.2f})'
                self.alert_publisher.publish(alert)

        except Exception as e:
            pass

def main(args=None):
    rclpy.init()
    node = ThermalMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()