import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import customtkinter as ctk
import tkinter as tk
from tkintermapview import TkinterMapView
import cv2
import threading
import numpy as np
from PIL import Image as PILImage, ImageTk

import open3d as o3d
import numpy as np
from collections import deque
from statistics import mean
from math import radians
import time

import math
import matplotlib.pyplot as plt
import subprocess
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"


from sensor_msgs.msg import NavSatFix

import time



class SerialDataProcessor(Node):
    def __init__(self, gui_app):
        super().__init__('serial_data_processor')
        
        # Referencia a la interfaz gráfica
        self.gui_app = gui_app
        
        # Suscripción al tópico de datos seriales
        self.subscription = self.create_subscription(
            String,
            'serial_data',
            self.serial_callback,
            10
        )
        
        # Valores actuales de los sensores
        self.current_values = {
            'voltage': 0.0,
            'current': 0.0,
            'mq4': 0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'rate_roll': 0.0,
            'rate_pitch': 0.0,
            'rate_yaw': 0.0,
            'f1': 0.0,
            'f2': 0.0,
            'f3': 0.0,
            'f4': 0.0
        }
    
    def parse_serial_data(self, data):

        """Parsea los datos seriales usando expresiones regulares"""
        # Añadir debug
        self.get_logger().info(f'Parseando datos: {data}')

        # Patrones para extraer roll, pitch y yaw
        roll_pattern = r'Roll=([-\d.]+)'
        pitch_pattern = r'Pitch=([-\d.]+)'
        yaw_pattern = r'Yaw=([-\d.]+)'

        # Buscar coincidencias
        roll_match = re.search(roll_pattern, data)
        pitch_match = re.search(pitch_pattern, data)
        yaw_match = re.search(yaw_pattern, data)
        
        # Actualizar valores si se encuentran coincidencias
        if roll_match:
            self.current_values['roll'] = float(roll_match.group(1))
            print(f"Roll extraído: {self.current_values['roll']}")
        
        if pitch_match:
            self.current_values['pitch'] = float(pitch_match.group(1))
            print(f"Pitch extraído: {self.current_values['pitch']}")
        
        if yaw_match:
            self.current_values['yaw'] = float(yaw_match.group(1))
            print(f"Yaw extraído: {self.current_values['yaw']}")
        
        # Patrón para los estados táctiles en formato [xxxx]
        tactile_pattern = r'Estados Táctiles: \[(\d{4})\]'
        tactile_match = re.search(tactile_pattern, data)
        
        if tactile_match:
            tactile_states = tactile_match.group(1)  # Obtiene los 4 dígitos
            self.get_logger().info(f'Estados táctiles encontrados: {tactile_states}')
            # Actualizar cada sensor individualmente
            self.current_values['f1'] = int(tactile_states[0])
            self.current_values['f2'] = int(tactile_states[1])
            self.current_values['f3'] = int(tactile_states[2])
            self.current_values['f4'] = int(tactile_states[3])
            return
        

        # Patrones para cada tipo de dato
        patterns = {
            'voltage': r'Voltaje=([-\d.]+)',
            'current': r'Corriente=([-\d.]+)',
            'mq4': r'MQ-4=(\d+)',
            'roll': r'Roll=([-\d.]+)',
            'pitch': r'Pitch=([-\d.]+)',
            'yaw': r'Yaw=([-\d.]+)',
            'rate_roll': r'RateRoll=([-\d.]+)',
            'rate_pitch': r'RatePitch=([-\d.]+)',
            'rate_yaw': r'RateYaw=([-\d.]+)',

        }


        # Buscar cada patrón en los datos
        for key, pattern in patterns.items():
            match = re.search(pattern, data)
            if match:
                value = match.group(1)
                # Convertir a float
                self.current_values[key] = float(value)
        """
        # Buscar cada patrón en los datos
        for key, pattern in patterns.items():
            match = re.search(pattern, data)
            if match:
                value = match.group(1)
                # Convertir todos los valores a float
                self.current_values[key] = float(value)
    """
    def serial_callback(self, msg):
        """Callback para procesar los datos seriales recibidos"""
        # Añadir logging para depuración
        self.get_logger().info(f'Datos seriales recibidos: {msg.data}')
        
        # Procesar los datos
        self.parse_serial_data(msg.data)
        
        
        # Actualizar la interfaz
       # self.update_gui()
        # Actualizar la interfaz en el hilo principal
        self.gui_app.after(0, self.update_gui)
    
    def update_gui(self):
        """Actualiza todos los elementos de la interfaz con los nuevos valores"""
        try:
            # Actualizar sensores táctiles
            states = {
                (self.gui_app.f1_label, 'f1'),
                (self.gui_app.f2_label, 'f2'),
                (self.gui_app.f3_label, 'f3'),
                (self.gui_app.f4_label, 'f4')
            }
            
            for label, key in states:
                value = self.current_values[key]
                state_text = "ACTIVO" if value == 1 else "INACTIVO"
                color = "green" if value == 1 else "red"
                
                # Forzar actualización inmediata
                label.configure(
                    text=f"{key.upper()}: {state_text}",
                    text_color=color
                )
                self.gui_app.update_idletasks()  # Forzar actualización de la interfaz
                
                # Logging para debug
                self.get_logger().info(f'Actualizando {key}: {state_text} (Color: {color})')


            # Actualizar etiquetas de IMU
            self.gui_app.roll_label.configure(text=f"Roll: {self.current_values['roll']:.2f}")
            self.gui_app.pitch_label.configure(text=f"Pitch: {self.current_values['pitch']:.2f}")
            self.gui_app.yaw_label.configure(text=f"Yaw: {self.current_values['yaw']:.2f}")
            
            # Actualizar tasas
            self.gui_app.rateRoll_label.configure(text=f"RateRoll: {self.current_values['rate_roll']:.2f}")
            self.gui_app.ratePitch_label.configure(text=f"RatePitch: {self.current_values['rate_pitch']:.2f}")
            self.gui_app.rateYaw_label.configure(text=f"RateYaw: {self.current_values['rate_yaw']:.2f}")
            
            # Actualizar sensores generales
            self.gui_app.gas_sensor_label.configure(text=f"Gas: {self.current_values['mq4']}")
            self.gui_app.voltage_sensor_label.configure(text=f"Voltaje: {self.current_values['voltage']:.2f}V")
            self.gui_app.current_sensor_label.configure(text=f"Corriente: {self.current_values['current']:.2f}A")
            
            print(f"Roll: {self.current_values['roll']}, Pitch: {self.current_values['pitch']}, Yaw: {self.current_values['yaw']}")
            self.gui_app.viewer.update_stl(self.current_values['roll'], self.current_values['pitch'], self.current_values['yaw'])


            # Actualizar visualización 3D si está disponible
            #if hasattr(self.gui_app, 'viewer'):
             #   self.gui_app.viewer.apply_rotation(
              #      self.current_values['roll'],
               #     self.current_values['pitch'],
               #     self.current_values['yaw']
                #)
            
        except Exception as e:
            self.get_logger().error(f'Error al actualizar la interfaz: {e}')
            # Mostrar el traceback completo para debugging
            import traceback
            self.get_logger().error(traceback.format_exc())

class STLViewer:
    def __init__(self, filename):
        print(f"Iniciando STL Viewer...")
        
        
        # Load STL mesh
        self.stl_path = filename
        print(f"Cargando STL desde: {filename}")
        self.mesh = o3d.io.read_triangle_mesh(filename)
        self.mesh.compute_vertex_normals()
        print("STL cargado exitosamente")
        
        # Variables for angles
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # Buffer for smooth movement
        self.angle_buffer_size = 5
        self.roll_buffer = deque(maxlen=self.angle_buffer_size)
        self.pitch_buffer = deque(maxlen=self.angle_buffer_size)
        self.yaw_buffer = deque(maxlen=self.angle_buffer_size)
        
        # Amplification factor for angles
        self.amplification_factor = 1.0
        
        # Time and update control
        self.target_fps = 60
        self.frame_time = 1.0 / self.target_fps
        self.last_update = time.time()
        
        # Reference to visualizer
        self.vis = None
        
        # Previous angles for change detection
        self.previous_angles = (0, 0, 0)

    def get_smoothed_angles(self):
        roll = mean(self.roll_buffer) if self.roll_buffer else 0
        pitch = mean(self.pitch_buffer) if self.pitch_buffer else 0
        yaw = mean(self.yaw_buffer) if self.yaw_buffer else 0
        return roll, pitch, yaw

    def update_stl(self, roll, pitch, yaw):
        try:
            # Add new angles to buffers
            self.roll_buffer.append(roll)
            self.pitch_buffer.append(pitch)
            self.yaw_buffer.append(yaw)
            
            # Get smoothed angles
            roll, pitch, yaw = self.get_smoothed_angles()
            
            # Check if angles have changed significantly
            if (abs(roll - self.previous_angles[0]) > 1 or 
                abs(pitch - self.previous_angles[1]) > 1 or 
                abs(yaw - self.previous_angles[2]) > 1):
                
                # Convert to radians
                rx = radians(roll)
                ry = radians(pitch)
                rz = radians(yaw)
                
                # Reset mesh to original orientation
                self.mesh = o3d.io.read_triangle_mesh(self.stl_path)
                self.mesh.compute_vertex_normals()
                
                # Create combined rotation matrix
                R = self.mesh.get_rotation_matrix_from_xyz((rx, ry, rz))
                
                # Apply rotation using center
                self.mesh.rotate(R, center=self.mesh.get_center())
                
                # Update visualization
                if self.vis is not None:
                    self.vis.clear_geometries()
                    self.vis.add_geometry(self.mesh)
                    self.vis.poll_events()
                    self.vis.update_renderer()
                
                # Save current angles
                self.previous_angles = (roll, pitch, yaw)
            
        except Exception as e:
            print(f"Error updating STL: {e}")
            import traceback
            traceback.print_exc()

    def show(self):
        import open3d as o3d
        import numpy as np
        
        try:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(window_name="STL Viewer")
            self.vis.add_geometry(self.mesh)
            
            # Configure view
            view_control = self.vis.get_view_control()
            view_control.set_zoom(0.8)
            
            # Configure render options
            opt = self.vis.get_render_option()
            opt.background_color = np.asarray([0.5, 0.5, 0.5])
            opt.point_size = 1.0
            opt.show_coordinate_frame = True
            
            print("Visualización iniciada")
            self.vis.run()
            
        except Exception as e:
            print(f"Error in visualization: {e}")
        finally:
            if self.vis is not None:
                self.vis.destroy_window()

    def start_update_thread(self):
        # This method is kept for compatibility but can be empty
        pass


class SensorInterface(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.viewer = STLViewer("/home/anderson/camara/src/tesis_launch/tesis_launch/POCHITA_v30.stl")  # Inicializa el visualizador STL
        self.viewer.start_update_thread()
        self.img_rst = None
        self.camera_window = None

        # Variables para el mapa
        self.map_window = None
        self.map_widget = None
        self.current_position = (3.353703, -75.522254)  # Posición inicial

        # Configuración de la ventana principal
        self.title("Monitor de Sensores del Perro Robótico")
        self.geometry("1000x800")

        # Tema y colores
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # Sección de título principal
        title_label = ctk.CTkLabel(self, text="Monitor de Sensores", font=ctk.CTkFont(size=24, weight="bold"))
        title_label.pack(pady=10)

        # Contenedor principal
        container_frame = ctk.CTkFrame(self)
        container_frame.pack(pady=10, padx=20, fill="both", expand=True)

        # Menú desplegable para los botones
        self.create_side_buttons(container_frame)

        # Sección del IMU
        self.create_imu(container_frame)

        # Sección de monitoreo de sensores
        self.create_sensor_sections(container_frame)

        # Sección para los sensores de fuerza
        self.create_force(container_frame)

        # Sección de comandos de voz
        self.create_voice_section()
        


    

    def create_side_buttons(self, parent):
        """Crea un menú desplegable para Cámara, Mapa y Simulación"""
        self.side_frame = ctk.CTkFrame(parent)
        self.side_frame.pack(side="top", anchor="nw", padx=(10, 0), pady=(10, 0), fill="x")

        # Botón para expandir/colapsar el menú (icono de tres rayitas)
        self.toggle_button = ctk.CTkButton(self.side_frame, text="☰", command=self.toggle_menu, width=10)
        self.toggle_button.pack(padx=10, pady=(5, 0))

        # Frame para los botones, inicialmente oculto
        self.button_frame = ctk.CTkFrame(self.side_frame)
        self.button_frame.pack(padx=10, pady=(5, 10))

        # Cargar y redimensionar la imagen
        camara_image_path = "/home/anderson/camara/src/tesis_launch/tesis_launch/camara-reflex-digital.png"  # Reemplaza con la ruta de tu imagen
        camara_image = PILImage.open(camara_image_path)
        camara_image = camara_image.resize((50, 50), PILImage.ANTIALIAS)  # Ajustar el tamaño de la imagen según sea necesario
        self.camara_image = ImageTk.PhotoImage(camara_image)  # Guardar una referencia para evitar que sea recolectada por el GC

        # Cargar y redimensionar la imagen
        mapa_image_path = "/home/anderson/camara/src/tesis_launch/tesis_launch/mapas.png"  # Reemplaza con la ruta de tu imagen
        mapa_image = PILImage.open(mapa_image_path)
        mapa_image = mapa_image.resize((50, 50), PILImage.ANTIALIAS)  # Ajustar el tamaño de la imagen según sea necesario
        self.mapa_image = ImageTk.PhotoImage(mapa_image)  # Guardar una referencia para evitar que sea recolectada por el GC

        # Cargar y redimensionar la imagen
        rviz_image_path = "/home/anderson/camara/src/tesis_launch/tesis_launch/rviz.png"  # Reemplaza con la ruta de tu imagen
        rviz_image = PILImage.open(rviz_image_path)
        rviz_image = rviz_image.resize((50, 50), PILImage.ANTIALIAS)  # Ajustar el tamaño de la imagen según sea necesario
        self.rviz_image = ImageTk.PhotoImage(rviz_image)  # Guardar una referencia para evitar que sea recolectada por el GC

        # Botones para abrir cámara, mapa y simulación con imágenes
        camera_button = ctk.CTkButton(self.button_frame, text="Cámara", command=self.open_camera_window, image=self.camara_image, compound="right")
        camera_button.pack(padx=30, pady=5)

        map_button = ctk.CTkButton(self.button_frame, text="Mapa", command=self.open_map_window, image=self.mapa_image, compound="right")
        map_button.pack(padx=10, pady=5)

        sim_button = ctk.CTkButton(self.button_frame, text="Simulación", command=self.open_simulation_window, image=self.rviz_image, compound="right")
        sim_button.pack(padx=10, pady=5)

        # Ocultar el marco de botones inicialmente
        self.button_frame.pack_forget()

    def toggle_menu(self):
        """Alterna la visibilidad del marco de botones"""
        if self.button_frame.winfo_ismapped():
            self.button_frame.pack_forget()
        else:
            self.button_frame.pack(padx=10, pady=(5, 10))

    def create_imu(self, parent):
        """Crea la sección del IMU"""
        imu_frame = ctk.CTkFrame(parent)
        imu_frame.pack(side="left", anchor="sw", padx=(10, 10), pady=(10, 0), fill="both", expand=True)

        imu_label = ctk.CTkLabel(imu_frame, text="IMU", font=ctk.CTkFont(size=50, weight="bold"))
        imu_label.pack(pady=10)

        # Botón para abrir la visualización del IMU
        imu_button = ctk.CTkButton(imu_frame, text="Visualizar IMU", command=self.open_imu_visualization)
        imu_button.pack(side="left", padx=20)



        # Frame para la orientación
        orientation_frame = ctk.CTkFrame(imu_frame)
        orientation_frame.pack(side=ctk.LEFT, padx=(10, 10))  # Alinea a la izquierda

        # Sensores de orientación
        self.roll_label = ctk.CTkLabel(orientation_frame, text="roll:   9.6")
        self.roll_label.pack(padx=(50),pady=(30, 30))

        self.pitch_label = ctk.CTkLabel(orientation_frame, text="pitch:   5.6")
        self.pitch_label.pack(padx=(10, 10), pady=(10, 30))

        self.yaw_label = ctk.CTkLabel(orientation_frame, text="yaw:   5.4")
        self.yaw_label.pack(padx=(10, 10), pady=(10, 30))
        # Frame para la orientación
        rate_frame = ctk.CTkFrame(imu_frame)
        rate_frame.pack(side=ctk.LEFT, padx=(10, 10))  # Alinea a la izquierda

        # Sensores de tasas
        self.rateRoll_label = ctk.CTkLabel(rate_frame, text="rateRoll:   2.5")
        self.rateRoll_label.pack(padx=(50),pady=(30, 30))

        self.ratePitch_label = ctk.CTkLabel(rate_frame, text="ratePitch:   22")
        self.ratePitch_label.pack(padx=(10, 10), pady=(10, 30))

        self.rateYaw_label = ctk.CTkLabel(rate_frame, text="rateYaw:   554")
        self.rateYaw_label.pack(padx=(10, 10) ,pady=(10, 30))


    

    
    def open_imu_visualization(self):
        try:
            # Aplicar las rotaciones deseadas
            self.viewer.show()
        except Exception as e:
            print(f"Error al abrir la visualización del IMU: {e}")

    def create_sensor_sections(self, parent):
        """Crea la sección de sensores (Gas, Voltaje, Corriente, Touch)"""
        sensor_frame = ctk.CTkFrame(parent)
        sensor_frame.pack(side="top", fill="both", padx=(5, 10), pady=(10, 0))

        sensor_label = ctk.CTkLabel(sensor_frame, text="Datos de Sensores", font=ctk.CTkFont(size=18, weight="bold"))
        sensor_label.pack(pady=(50, 0))

        # Sensores individuales
        self.gas_sensor_label = ctk.CTkLabel(sensor_frame, text="Gas: ...")
        self.gas_sensor_label.pack(padx=5 ,pady=5)

        self.voltage_sensor_label = ctk.CTkLabel(sensor_frame, text="Voltaje: ...")
        self.voltage_sensor_label.pack(pady=5)

        self.current_sensor_label = ctk.CTkLabel(sensor_frame, text="Corriente: ...")
        self.current_sensor_label.pack(pady=5)



    
    def create_force(self, parent):
        """Crea la sección de IMU y sensores de fuerza"""
        force_frame = ctk.CTkFrame(parent)
        force_frame.pack(side="bottom", anchor="se", fill="both", padx=5, pady=(5, 5))

        force_label = ctk.CTkLabel(force_frame, text="Sensores de Fuerza", font=ctk.CTkFont(size=18, weight="bold"))
        force_label.pack(pady=5)

        # Cargar y redimensionar la imagen
        image_path = "/home/anderson/camara/src/tesis_launch/tesis_launch/perro.png"
        image = PILImage.open(image_path)
        image = image.resize((200, 200), PILImage.ANTIALIAS)
        self.image = ImageTk.PhotoImage(image)

        # Crear un contenedor para la imagen y los valores
        content_frame = ctk.CTkFrame(force_frame)
        content_frame.pack(padx=(5, 0), pady=5)

        # Crear un label para la imagen sin texto
        image_label = ctk.CTkLabel(content_frame, image=self.image, text='')
        image_label.pack(side=ctk.LEFT, padx=(10))

        # Frame para los valores de sensores táctiles
        values_frame = ctk.CTkFrame(content_frame)
        values_frame.pack(side=ctk.RIGHT)

        # Crear labels para sensores táctiles con valores iniciales
        self.f1_label = ctk.CTkLabel(values_frame, text="F1: INACTIVO", text_color="red")
        self.f1_label.pack(padx=50, pady=2)

        self.f2_label = ctk.CTkLabel(values_frame, text="F2: INACTIVO", text_color="red")
        self.f2_label.pack(pady=2)

        self.f3_label = ctk.CTkLabel(values_frame, text="F3: INACTIVO", text_color="red")
        self.f3_label.pack(pady=2)

        self.f4_label = ctk.CTkLabel(values_frame, text="F4: INACTIVO", text_color="red")
        self.f4_label.pack(pady=2)

    def create_voice_section(self):
        """Crea la sección de comandos de voz"""
        voice_frame = ctk.CTkFrame(self)
        voice_frame.pack(pady=10, padx=20, fill="both", expand=True)

        voice_label = ctk.CTkLabel(voice_frame, text="Comandos de Voz", font=ctk.CTkFont(size=18, weight="bold"))
        voice_label.pack(pady=10)

        # Crear un Textbox donde se mostrarán los comandos de voz
        self.voice_command_display = ctk.CTkTextbox(voice_frame, width=800, height=100)
        self.voice_command_display.pack(pady=10)

    def update_audio_text(self, audio_text):
        """Actualiza la caja de texto con el nuevo comando de voz"""
        # Insertar el comando de voz recibido en la caja de texto
        self.voice_command_display.insert("end", audio_text + "\n")
        # Hacer scroll automáticamente hacia el final para ver los comandos más recientes
        self.voice_command_display.see("end")



    def update_img_rst(self, new_img_rst):
        """Función para actualizar la imagen procesada"""
        self.img_rst = new_img_rst

    def open_camera_window(self):
        """Abrir la cámara y mostrar la transmisión en una ventana con OpenCV."""
        if not hasattr(self, 'img_rst') or self.img_rst is None:
            print("No se ha recibido ninguna imagen para mostrar.")
            return

        print("Activando la cámara...")

        # Crear una ventana para la cámara
        cv2.namedWindow("Transmisión de la cámara", cv2.WINDOW_NORMAL)

        def update_camera_window():
            """Función para actualizar la ventana de la cámara."""
            try:
                if self.img_rst is not None:
                    # Mostrar la imagen en la ventana
                    cv2.imshow("Transmisión de la cámara", self.img_rst)

                # Esperar 1 ms y verificar si se presionó la tecla 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Cerrando la ventana de la cámara.")
                    cv2.destroyWindow("Transmisión de la cámara")
                    return

                # Programar la próxima actualización
                self.after(30, update_camera_window)  # Actualizar cada 30 ms
            except Exception as e:
                print(f"Error al mostrar la imagen: {e}")
                cv2.destroyWindow("Transmisión de la cámara")

        # Iniciar la actualización de la ventana de la cámara
        self.after(0, update_camera_window)




    def open_map_window(self):
        """Abre una nueva ventana para mostrar el mapa en tiempo real"""
        if self.map_window is None or not self.map_window.winfo_exists():
            self.map_window = ctk.CTkToplevel(self)
            self.map_window.title("Mapa en tiempo real")
            self.map_window.geometry("800x600")

            # Crear el widget del mapa
            self.map_widget = TkinterMapView(self.map_window, width=800, height=600, corner_radius=0)
            self.map_widget.pack(fill="both", expand=True)
            
            # Establecer posición inicial
            self.map_widget.set_position(*self.current_position)
            self.map_widget.set_zoom(15)
            
            # Agregar un marcador inicial
            self.current_marker = self.map_widget.set_marker(*self.current_position, text="Robot")


    def update_gps_position(self, lat, lon):
        """Actualiza la posición en el mapa"""
        if self.map_widget and self.map_window and self.map_window.winfo_exists():
            self.current_position = (lat, lon)
            # Actualizar el marcador
            if hasattr(self, 'current_marker'):
                self.current_marker.delete()
            self.current_marker = self.map_widget.set_marker(lat, lon, text="Robot")
            # Centrar el mapa en la nueva posición
            self.map_widget.set_position(lat, lon)



    def open_simulation_window(self):
        """Execute ROS 2 LIDAR launch and RViz"""
        try:
            # Paths
            rviz_config_path = "/home/anderson/camara/src/tesis_launch/tesis_launch/rviz/my_rviz_config.rviz"
            
            # Validate configuration file
            if not os.path.exists(rviz_config_path):
                print(f"RViz configuration not found: {rviz_config_path}")
                return
            
            # Launch LIDAR 
            lidar_cmd = ['ros2', 'launch', 'rplidar_ros', 'rplidar.launch.py']
            
            # RViz launch
            rviz_cmd = ['rviz2', '-d', rviz_config_path]
            
            # Use subprocess to launch in separate processes
            lidar_process = subprocess.Popen(lidar_cmd)
            rviz_process = subprocess.Popen(rviz_cmd)
            
            # Optional: Wait for processes to complete if needed
            # lidar_process.wait()
            # rviz_process.wait()
            
        except Exception as e:
            print(f"Simulation launch error: {e}")
            # Optionally log more detailed error
            import traceback
            traceback.print_exc()





"""
    def show_video(self,img_rst):

        cv2.imshow("camera", self.img_rst)

        cv2.waitKey(1)
"""

class GPSSubscriber(Node):
    def __init__(self, gui_app):
        super().__init__('gps_subscriber')
        self.gui_app = gui_app
        
        # Suscripción al tópico GPS
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
    
    def gps_callback(self, msg):
        """Callback para procesar los datos GPS recibidos"""
        # Actualizar la posición en la interfaz
        self.gui_app.update_gps_position(msg.latitude, msg.longitude)
        self.get_logger().info(f'GPS Update - Lat: {msg.latitude}, Lon: {msg.longitude}')

class interfazSuscriber(Node):
    def __init__(self, gui_app):
        super().__init__('interfaz_suscriber')

        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(Image, 'video_detections', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

        # Suscripción al tópico 'audio_text' (tipo de mensaje: String)
        self.subscription_audio = self.create_subscription(
        String, 'audio_text', self.listener_audio_callback, 10)

         # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
    

        
        # Referencia a la aplicación gráfica
        self.gui_app = gui_app
        

    def listener_callback(self, data):
        bridge = CvBridge()
        
        try:
            # Convertir la imagen ROS a una imagen de OpenCV
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.update_img_rst(cv_image)
        except Exception as e:
            print(f"Error al convertir la imagen: {e}")

            self.get_logger().info('Receiving video frame')
  
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)


        # Display image
        kernel = np.array([[1.0, 1.0, 1.0], 
                    [1.0, 1.0, 1.0],
                    [1.0, 1.0, 1.0]])

        kernel = kernel/(np.sum(kernel) if np.sum(kernel)!=0 else 1)

        img_rst = cv2.filter2D(current_frame,-1,kernel)

        # Actualizar la imagen en la interfaz gráfica
        self.gui_app.update_img_rst(img_rst)


        #cv2.imshow("camera", img_rst)

        #cv2.waitKey(1)

    # Callback para recibir el texto del tópico 'audio_text'
    def listener_audio_callback(self, msg):
        audio_text = msg.data
        self.get_logger().info(f"Texto recibido del audio: {audio_text}")

        # Actualizar la interfaz gráfica con el nuevo comando de voz
        self.gui_app.update_audio_text(audio_text)

def main(args=None):
    rclpy.init(args=args)
    
    gui_app = SensorInterface()
    ros_node = interfazSuscriber(gui_app)
    serial_processor = SerialDataProcessor(gui_app)
    gps_subscriber = GPSSubscriber(gui_app)  # Añadir el suscriptor GPS
    
    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0)
            rclpy.spin_once(serial_processor, timeout_sec=0)
            rclpy.spin_once(gps_subscriber, timeout_sec=0)  # Añadir el spin del GPS
            gui_app.update()  # Actualizar la GUI regularmente
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    gui_app.mainloop()
    
    gps_subscriber.destroy_node()
    ros_node.destroy_node()
    serial_processor.destroy_node()
    rclpy.shutdown()

"""
def main(args=None):
    # Inicializar ROS 2
    rclpy.init(args=args)

    # Crear la interfaz gráfica
    gui_app = SensorInterface()

    # Crear los nodos ROS 2
    ros_node = interfazSuscriber(gui_app)
    serial_processor = SerialDataProcessor(gui_app)

    # Asignar los nodos a la aplicación gráfica
    gui_app.ros_node = ros_node
    gui_app.serial_processor = serial_processor

    # Iniciar ROS 2 en un hilo separado
    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(ros_node)
            rclpy.spin_once(serial_processor)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # Ejecutar la interfaz gráfica
    gui_app.mainloop()

    # Limpiar
    ros_node.destroy_node()
    serial_processor.destroy_node()
    rclpy.shutdown()
"""

"""
def main(args=None):
    # Inicializar ROS 2
    rclpy.init(args=args)

    # Crear la interfaz gráfica
    gui_app = SensorInterface()

    # Crear el nodo ROS 2
    ros_node = interfazSuscriber(gui_app)

    # Asignar el nodo ROS 2 a la aplicación gráfica para futuras interacciones
    gui_app.ros_node = ros_node

    # Iniciar ROS 2 en un hilo separado para que la interfaz gráfica no se bloquee
    def ros_spin():
        rclpy.spin(ros_node)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # Ejecutar la interfaz gráfica
    gui_app.mainloop()

    # Destruir el nodo ROS y limpiar recursos al finalizar
    ros_node.destroy_node()
    rclpy.shutdown()
"""
if __name__ == '__main__':
    main()