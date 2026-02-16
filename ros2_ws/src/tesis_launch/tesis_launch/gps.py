import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        # Crear un publicador en el tópico '/gps/fix'
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # Configurar el puerto serial
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
            self.get_logger().info("Puerto serie abierto en /dev/ttyUSB2.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error al abrir el puerto serie: {e}")
            self.ser = None

        # Crear un temporizador para leer los datos periódicamente
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.ser is not None and self.ser.is_open:
            try:
                # Leer una línea desde el puerto serie
                raw_data = self.ser.readline()

                try:
                    sentence = raw_data.decode('ascii').strip()  # Intentar decodificar como ASCII
                except UnicodeDecodeError:
                    sentence = None
                    self.get_logger().warn(f"Error de decodificación. Datos en bruto (hex): {raw_data.hex()}")

                if sentence and sentence.startswith('$'):
                    self.decode_nmea_sentence(sentence)
                else:
                    self.get_logger().warn(f"Trama no válida recibida: {raw_data.decode('latin-1', errors='ignore').strip()}")
                    self.get_logger().warn(f"Datos en bruto (hex): {raw_data.hex()}")

            except serial.SerialException as e:
                self.get_logger().warn(f"Error al leer del puerto serie: {e}")
    
    def decode_nmea_sentence(self, sentence):
        try:
            # Decodificar la trama NMEA
            msg = pynmea2.parse(sentence)

            # Verificar si la trama es de tipo GGA (Fix information)
            if isinstance(msg, pynmea2.GGA):
                latitude = msg.latitude
                longitude = msg.longitude
                altitude = msg.altitude

                # Validar los datos antes de publicarlos
                if latitude and longitude and altitude:
                    # Crear un mensaje NavSatFix
                    navsat_msg = NavSatFix()
                    navsat_msg.header.stamp = self.get_clock().now().to_msg()
                    navsat_msg.header.frame_id = "gps"

                    # Asignar los valores al mensaje NavSatFix
                    navsat_msg.latitude = float(latitude)
                    navsat_msg.longitude = float(longitude)
                    navsat_msg.altitude = float(altitude)
                    navsat_msg.status.status = 0  # Status: GPS fix
                    navsat_msg.status.service = 1  # Service: GPS

                    # Publicar el mensaje
                    self.publisher_.publish(navsat_msg)
                    self.get_logger().info(f"Publicado: Latitud={latitude}, Longitud={longitude}, Altitud={altitude}")
                else:
                    self.get_logger().warn("Trama incompleta: faltan datos de latitud, longitud o altitud.")

        except pynmea2.ParseError as e:
            self.get_logger().warn(f"Error al decodificar la trama NMEA: {e}")
        except ValueError as e:
            self.get_logger().warn(f"Error en los datos de la trama: {e}")
        except Exception as e:
            self.get_logger().warn(f"Error inesperado: {e}")

    def destroy(self):
        # Cerrar el puerto serie cuando el nodo se destruya
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Puerto serie cerrado.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    gps_node = GPSNode()
    rclpy.spin(gps_node)

    gps_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
