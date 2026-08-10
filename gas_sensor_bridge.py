import rclpy # ROS Client Library for Python
from rclpy.node import Node # to make a ROS 2 node
from std_msgs.msg import String # to send and receive string messages
import serial # to communicate with the gas sensor via serial port

SERIAL_PORT = '/dev/ttyUSB0' # TODO: connect the arduino via USB and check the port type
BAUD_RATE = 9600
TRESHOLD = 300 # TODO: modify the threshold value based on the gas sensor calibration

class GasSensorNode(Node):
    def __init__(self): # self is the instance of the class
        super().__init__('gas_sensor_node') # give a name to the ROS2 node
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)  # timeout = no. of sec. to wait before closing the program if nothing comes

        self.gas_value_publisher = self.create_publisher(String, '/alarm_event', 10) # max 10 slots on message queue
        self.timer_for_publishing = self.create_timer(0.5, self.check_gas) # 0.5 = how often to check, self.check_gas = what to check

    def check_gas(self):
        try:
            line = self.ser.readline().decode('utf-8').strip() # strip removes all newlines and spaces
            extracted_value = int(line.split(":")[1])
            if extracted_value >= TRESHOLD:
                msg = String()
                msg.data = f"POSIBILA SCURGERE DE GAZE! Nivel gaz: {extracted_value}"
                self.gas_value_publisher.publish(msg)
        except:
            pass # if the gas sensor send an invalid value, ignore it and continue

def main(args=None):
    rclpy.init()
    gas_sensor_node = GasSensorNode()
    try:
        rclpy.spin(gas_sensor_node) # continues to run the node until interrupted
    except KeyboardInterrupt:
        pass
    finally:
        gas_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()