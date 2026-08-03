import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import telebot

TELEGRAM_TOKEN = '8991496997:AAGl5nM5Oj5s8xEcbW0tGPnjVzrdugFimco'
CHAT_ID = '7254358750'

bot = telebot.TeleBot(TELEGRAM_TOKEN)
ros_node_instance = None

class TelegramAlarmNode(Node):

  def __init__(self):
    super().__init__('telegram_alarm_node')
    self.subscription = self.create_subscription(
        String, '/alarm_event', self.alarm_callback, 10
    )
    self.command_publisher = self.create_publisher(String, '/robot_commands', 10)
    self.get_logger().info('Nodul de alertare Telegram a pornit')
    self.send_telegram_alert('Sistem de alarma Pupper: Conexiune ROS 2 stabilita.')

  def alarm_callback(self, msg):
    self.get_logger().info(f'Alerta primita: {msg.data}')
    self.send_telegram_alert(f'ALERTA ROBOT:\n{msg.data}')

  def send_telegram_alert(self, message_text):
    try:
      bot.send_message(CHAT_ID, message_text)
    except Exception as e:
      self.get_logger().error(f'Eroare trimitere Telegram: {e}')

@bot.message_handler(commands=['status'])
def handle_status(message):
  bot.reply_to(message, 'Status Pupper: Sistem functional.')

@bot.message_handler(commands=['stop'])
def handle_stop(message):
  bot.reply_to(message, 'Comanda de oprire trimisa.')
  if ros_node_instance:
      cmd_msg = String()
      cmd_msg.data = "STOP_ROBOT"
      ros_node_instance.command_publisher.publish(cmd_msg)

@bot.message_handler(commands=['intercom'])
def handle_intercom(message):
  bot.reply_to(message, 'Modul audio activat.')
  if ros_node_instance:
      cmd_msg = String()
      cmd_msg.data = "ACTIVATE_AUDIO_LINK"
      ros_node_instance.command_publisher.publish(cmd_msg)

def start_telegram_polling():
  bot.infinity_polling()

def main(args=None):
  global ros_node_instance
  rclpy.init(args=args)
  ros_node_instance = TelegramAlarmNode()
  
  telegram_thread = threading.Thread(target=start_telegram_polling, daemon=True)
  telegram_thread.start()

  try:
    rclpy.spin(ros_node_instance)
  except KeyboardInterrupt:
    pass
  finally:
    ros_node_instance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
