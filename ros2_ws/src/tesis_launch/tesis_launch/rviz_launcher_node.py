import subprocess
import rclpy
from rclpy.node import Node

class RvizLauncherNode(Node):
    def __init__(self):
        super().__init__('rviz_launcher_node')
        self.get_logger().info('Launching RViz2...')

        # Ejecuta rviz2 directamente sin configuración
        subprocess.Popen(['rviz2'])

def main(args=None):
    rclpy.init(args=args)
    node = RvizLauncherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

