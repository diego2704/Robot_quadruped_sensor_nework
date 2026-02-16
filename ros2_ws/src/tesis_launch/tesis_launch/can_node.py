import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class CanNode(Node):
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        super().__init__('can_node')  # Nombre del nodo

        # Configuración del puerto serial
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(f'Conectado al puerto serial: {port} a {baudrate} bps')
        except serial.SerialException as e:
            self.get_logger().error(f'Error al abrir el puerto serial: {e}')
            return

        # Publicador para enviar datos leídos por serial
        self.publisher_ = self.create_publisher(String, 'serial_data', 10)

        # Temporizador para leer el serial periódicamente
        self.timer = self.create_timer(0.01, self.read_serial_callback)  # Cada 10 ms

    def read_serial_callback(self):
        if self.serial_port.is_open:
            try:
                # Leer todos los datos disponibles del puerto serial
                while self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8')
                    lines = data.splitlines()  # Divide el contenido en líneas
                    
                    for line in lines:
                        if line.strip():  # Procesa solo líneas no vacías
                            msg = String()
                            msg.data = line
                            self.publisher_.publish(msg)  # Publica el mensaje en el tópico
                            self.get_logger().info(f'Publicado: "{line}"')
            except Exception as e:
                self.get_logger().error(f'Error al leer el puerto serial: {e}')

    def destroy_node(self):
        # Cerrar el puerto serial al destruir el nodo
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Puerto serial cerrado')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanNode(port='/dev/ttyUSB1', baudrate=115200)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido manualmente')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
