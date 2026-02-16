#!/usr/bin/env python3
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tesis_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # Encuentra automáticamente los subpaquetes
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Incluir todos los archivos de la carpeta launch
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Asegúrate de que la ruta es correcta
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',  # Cambia esto a tu nombre
    maintainer_email='tu_email@example.com',  # Cambia esto a tu email
    description='Paquete de lanzamiento para mi robot',
    license='MIT',
    tests_require=['pytest',
                   'opencv-python',
                   'rplidar'],
    entry_points={
        'console_scripts': [
            'img_publisher = tesis_launch.image_publisher:main',
            'img_sus = tesis_launch.image_sus:main',
            'interfaz = tesis_launch.interfaz:main', 
            'rplidar.launch = tesis_launch.rplidar.launch:main',
            'rviz_launcher_node = tesis_launch.rviz_launcher_node:main',
            'coman_pub =  tesis_launch.comandos_pub:main',
            'coman_sus =  tesis_launch.comandos_sus:main',
            'can =  tesis_launch.can_node:main',
            'gps =  tesis_launch.gps:main',
            

                    ],
    },
)
