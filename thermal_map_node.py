import rclpy # ROS Client Library for Python
from rclpy.node import Node # to make a ROS 2 node
from std_msgs.msg import String # to send and receive string messages
from visualization_msgs.msg import Marker, MarkerArray # for RViz
from std_msgs.msg import ColorRGBA # for marker colors
from tf2_ros import Buffer, TransformListener # to read the pose of the robot in the map
import math

# tf = transform frames, keeps track of where is every frame in the map, reported to the old ones

TRESHOLD = 50 # Celsius 

class ThermalMapNode(Node):
    def __init__(self):
        super().__init__('thermal_map_node') # give a name to the ROS2 node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # to read the pose of the robot

        self.subscription = self.create_subscription(String, '/temperature_data', self.temperature_callback, 10)
        self.alert_publisher = self.create_publisher(String, '/alarm_event', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/thermal_map_markers', 10) # publisher for RVizz
        self.thermal_data = [] # list of tuples: (x, y, temp)

        self.gas_subscription = self.create_subscription(String, '/alarm_event', self.gas_callback, 10) # listen for gas readings
        self.gas_marker_publisher = self.create_publisher(MarkerArray, '/gas_map_markers', 10) # publish the clouds in rviz
        self.gas_data = [] # list of positions of gas clouds (x, y)


    def get_color(self, temp):
        color = ColorRGBA()
        color.a = 0.3 # semitransparent to mix on the map
        
        if temp < 45:    # Green = Safe zone
            color.r = 0.0
            color.g = 1.0
            color.b = 0.0
        elif temp <= 55: # Yellow = Potentially dangerous
            color.r = 1.0
            color.g = 1.0
            color.b = 0.0
        else:            # Red = Dangerous zone
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
        return color

    def temperature_callback(self, msg): # call herself when a new message is received
        try:
            temp = float(msg.data)

            # read the position of the robot in the map
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # save the touple (x, y, temp)
            should_save = True
            if len(self.thermal_data) > 0:
                last_x, last_y, _ = self.thermal_data[-1]
                distance = math.sqrt((x - last_x)**2 + (y - last_y)**2)
                if distance < 0.2: # 20 centimetri
                    should_save = False
            if should_save:
                self.thermal_data.append((x, y, temp))
                self.get_logger().info(f'Temp: {temp}°C salvata la: ({x:.2f}, {y:.2f})')

            if temp >= TRESHOLD:
                alert = String()
                alert.data = f'ALERTA TEMPERATURA! {temp}°C la pozitia ({x:.2f}, {y:.2f})'
                self.alert_publisher.publish(alert)

            marker_array = MarkerArray()

            for i, data_point in enumerate(self.thermal_data):
                x, y, temp = data_point

                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'thermal_map'
                marker.id = i

                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.01

                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.01

                marker.color = self.get_color(temp)

                marker_array.markers.append(marker)

            self.marker_publisher.publish(marker_array)

        except Exception as e:
            pass

    def gas_callback(self, msg):
        if "POSIBILA SCURGERE DE GAZE" in msg.data:
            try:
                gas_level_str = msg.data.split("Nivel gaz: ")[1]
                gas_level = float(gas_level_str)

                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                x = transform.transform.translation.x
                y = transform.transform.translation.y

                should_save = True
                if len(self.gas_data) > 0:
                    last_x, last_y, _ = self.gas_data[-1]
                    distance = math.sqrt((x - last_x)**2 + (y - last_y)**2)
                    if distance < 0.4:
                        should_save = False
                
                if should_save:
                    self.gas_data.append((x, y, gas_level))
                    self.get_logger().info(f'Gaz detectat ({gas_level})! Nor creat la: ({x:.2f}, {y:.2f})')
                    
                marker_array = MarkerArray()
                for i, data_point in enumerate(self.gas_data):
                    px, py, pgas = data_point
                    
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'gas_map'
                    marker.id = i
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    marker.pose.position.x = px
                    marker.pose.position.y = py
                    marker.pose.position.z = 0.5
                    
                    marker.scale.x = 1.0
                    marker.scale.y = 1.0
                    marker.scale.z = 1.0
                    
                    marker.color.r = 0.6
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    
                    opacity = 0.3 + ((pgas - 300) / 723.0) * 0.7
                    
                    if opacity > 1.0: opacity = 1.0
                    if opacity < 0.1: opacity = 0.1
                    
                    marker.color.a = float(opacity)
                    
                    marker_array.markers.append(marker)

                self.gas_marker_publisher.publish(marker_array)
                
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