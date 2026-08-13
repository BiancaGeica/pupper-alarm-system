import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import telebot

TELEGRAM_TOKEN = 'INSEREAZA_TOKEN_AICI'
CHAT_ID = '7254358750'

bot = telebot.TeleBot(TELEGRAM_TOKEN)

class TelegramAlarmNode(Node):

    def __init__(self):
        super().__init__('telegram_alarm_node')
        
        self.subscription = self.create_subscription(
            String, '/alarm_event', self.alarm_callback, 10
        )
        
        self.photo_subscription = self.create_subscription(
            String, '/camera/poze_salvate', self.photo_callback, 10
        )

        self.get_logger().info('Nodul de alertare Telegram a pornit')
        self.send_telegram_alert('Sistem de alarma Pupper: Conexiune ROS 2 stabilita.')

    def alarm_callback(self, msg):
        self.get_logger().info(f'Alerta primita: {msg.data}')
        self.send_telegram_alert(f'ALERTA ROBOT:\n{msg.data}')

    def photo_callback(self, msg):
        self.get_logger().info(f'Trimitere harta PGM de la: {msg.data}')
        try:
            with open(msg.data, 'rb') as pgm_file:
                bot.send_document(CHAT_ID, pgm_file)
        except Exception as e:
            self.get_logger().error(f'Eroare trimitere harta: {e}')

    def send_telegram_alert(self, message_text):
        try:
            bot.send_message(CHAT_ID, message_text)
        except Exception as e:
            self.get_logger().error(f'Eroare trimitere Telegram: {e}')


@bot.message_handler(commands=['status'])
def handle_status(message):
    bot.reply_to(message, 'Status Pupper: Sistem functional.')

@bot.message_handler(commands=['harta'])
def handle_harta(message):
    bot.reply_to(message, 'Se trimite harta generata...')
    try:
        with open('map.pgm', 'rb') as pgm_file:
            bot.send_document(message.chat.id, pgm_file)
    except Exception as e:
        bot.reply_to(message, f'Eroare la citirea hartii: {e}')

def start_telegram_polling():
    bot.infinity_polling()

def main(args=None):
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