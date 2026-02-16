#!/usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # Suscribirse al tópico de la cámara
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)
        self.subscription  # evitar advertencias de variable no utilizada

        # Crear un publicador para enviar los frames procesados
        self.publisher_ = self.create_publisher(Image, 'video_detections', 10)

        self.bridge = CvBridge()

        # Cargar el modelo YOLO
        try:
            self.model = YOLO('yolov5nu.pt')
            self.get_logger().info("Modelo YOLO cargado correctamente.")
        except Exception as e:
            self.get_logger().error(f"Error al cargar el modelo YOLO: {e}")

    def listener_callback(self, msg):
        try:
            # Convertir el mensaje de ROS a un frame de OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Realizar la detección de objetos
            results = self.model(frame)
            
            # Dibuja las cajas detectadas
            annotated_frame = results[0].plot()

            # Publicar el frame procesado en el nuevo tópico
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.publisher_.publish(annotated_msg)

            # Mostrar el frame procesado localmente
            #cv2.imshow("Video (procesado)", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Salir con 'q'
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error procesando frame: {e}")

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()  # Cerrar ventanas al finalizar

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        camera_subscriber.get_logger().info("Nodo detenido por el usuario.")
    finally:
        camera_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
