from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener la ruta del launch del otro paquete
    rplidar_launch_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')
    rplidar_launch_file = os.path.join(rplidar_launch_dir, 'rplidar.launch.py')
    rviz_config_dir = os.path.join(get_package_share_directory('tesis_launch'), 'rviz', 'my_rviz_config.rviz')

    # Incluye el launch file del otro paquete
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file)
    )
    
    return LaunchDescription([

        #rplidar_launch,

        # Nodo para lanzar RViz
        #Node(
        #    name='rviz2',
        #    package='rviz2',
        #    executable='rviz2',
        #    output='screen',
        #    arguments=['-d', rviz_config_dir] 
        #),

        # Nodo de la cámara
        Node(
            package='tesis_launch',  # El nombre de tu paquete
            executable='img_publisher',  # El nombre del ejecutable
            output='screen',  # Muestra la salida en la terminal
            parameters=[{
                'video_frames': '/camera/image_raw',  # Parámetros adicionales si es necesario
            }]
        ),
        
        # Nodo que procesa la imagen (image subscriber)
        Node(
             package='tesis_launch',  # El nombre de tu paquete
             executable='img_sus',  # El nombre del ejecutable
             output='screen',  # Muestra la salida en la terminal
         ),

        Node(
            package='tesis_launch',  # El nombre de tu paquete
            executable='interfaz',  # El nombre del ejecutable
            output='screen',  # Muestra la salida en la terminal
        ),

        # Nodo de publicación de comandos
        Node(
             package='tesis_launch',  # El nombre de tu paquete
            executable='coman_pub',  # El nombre del ejecutable
            output='screen',  # Muestra la salida en la terminal
        ),
        
        # Nodo de suscripción de comandos
        Node(
            package='tesis_launch',  # El nombre de tu paquete
            executable='coman_sus',  # El nombre del ejecutable
            output='screen',  # Muestra la salida en la terminal
        ),

        Node(
            package='tesis_launch',  # El nombre de tu paquete
            executable='can',  # El nombre del ejecutable
            output='screen',  # Muestra la salida en la terminal
        ),

        #Node(
        #    package='tesis_launch',  # El nombre de tu paquete
        #    executable='gps',  # El nombre del ejecutable
        #    output='screen',  # Muestra la salida en la terminal
        #),



    ])
