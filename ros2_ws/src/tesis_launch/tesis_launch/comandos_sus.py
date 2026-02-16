import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(
            String,
            'audio_text',  # El mismo nombre del tópico en el publicador
            self.listener_callback,
            10)
        self.subscription  # Evita que la suscripción sea eliminada por el recolector de basura
        self.get_logger().info("Nodo suscriptor de audio inicializado, esperando mensajes...")

    def listener_callback(self, msg):
        # Imprime el texto que recibe
        self.get_logger().info(f"Texto recibido: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    audio_subscriber = AudioSubscriber()

    try:
        rclpy.spin(audio_subscriber)
    except KeyboardInterrupt:
        audio_subscriber.get_logger().info("Nodo detenido por el usuario.")
    finally:
        audio_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
