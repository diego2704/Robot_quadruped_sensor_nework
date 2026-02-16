import open3d as o3d
import numpy as np
import serial
import threading
import time
import re
from math import radians
from collections import deque
from statistics import mean

class STLViewer:
    def __init__(self, stl_path, port='/dev/ttyUSB0', baudrate=115200):
        print(f"Iniciando STL Viewer...")
        
        # Cargar el modelo STL
        self.stl_path = stl_path
        print(f"Cargando STL desde: {stl_path}")
        self.mesh = o3d.io.read_triangle_mesh(stl_path)
        self.mesh.compute_vertex_normals()
        print("STL cargado exitosamente")
        
        # Variables para los ángulos
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # Buffer para suavizar el movimiento
        self.angle_buffer_size = 5
        self.roll_buffer = deque(maxlen=self.angle_buffer_size)
        self.pitch_buffer = deque(maxlen=self.angle_buffer_size)
        self.yaw_buffer = deque(maxlen=self.angle_buffer_size)
        
        # Factor de amplificación para los ángulos
        self.amplification_factor = 1.0
        
        # Control de tiempo y actualización
        self.target_fps = 60
        self.frame_time = 1.0 / self.target_fps
        self.last_update = time.time()
        
        # Configuración serial
        print(f"Conectando al puerto serial {port} a {baudrate} baudios...")
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print("Puerto serial conectado exitosamente")
        except Exception as e:
            print(f"Error al abrir puerto serial: {e}")
            raise
        
        # Limpiar buffers
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        
        # Iniciar thread de lectura serial
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        print("Thread de lectura serial iniciado")
        
        # Referencia al visualizador
        self.vis = None

    def read_serial(self):
        print("Iniciando lectura de datos seriales...")
        while self.running:
            try:
                if self.serial_port.in_waiting:
                    raw_line = self.serial_port.readline()
                    
                    try:
                        line = raw_line.decode('ascii', errors='ignore').strip()
                        print(f"Datos recibidos: {line}")  # Debug
                        
                        # Procesar Roll y Pitch
                        roll_pitch_match = re.search(r'Enviado Roll=([-\d.]+) Pitch=([-\d.]+)', line)
                        if roll_pitch_match:
                            try:
                                roll = float(roll_pitch_match.group(1)) * self.amplification_factor
                                pitch = float(roll_pitch_match.group(2)) * self.amplification_factor
                                self.roll_buffer.append(roll)
                                self.pitch_buffer.append(pitch)
                                print(f"Roll: {roll}, Pitch: {pitch}")  # Debug
                            except ValueError as e:
                                print(f"Error al convertir Roll/Pitch: {e}")
                        
                        # Procesar Yaw
                        yaw_match = re.search(r'Enviado Yaw=([-\d.]+)', line)
                        if yaw_match:
                            try:
                                yaw = float(yaw_match.group(1)) * self.amplification_factor
                                self.yaw_buffer.append(yaw)
                                print(f"Yaw: {yaw}")  # Debug
                            except ValueError as e:
                                print(f"Error al convertir Yaw: {e}")
                                
                    except UnicodeDecodeError as e:
                        print(f"Error al decodificar línea: {e}")
                
            except serial.SerialException as e:
                print(f"Error en lectura serial: {e}")
                time.sleep(0.1)
            except Exception as e:
                print(f"Error inesperado en lectura serial: {e}")
                time.sleep(0.1)

    def get_smoothed_angles(self):
        roll = mean(self.roll_buffer) if self.roll_buffer else 0
        pitch = mean(self.pitch_buffer) if self.pitch_buffer else 0
        yaw = mean(self.yaw_buffer) if self.yaw_buffer else 0
        print("datos roll: ", roll, "pitch: ", pitch, "yaw: ", yaw)
        return roll, pitch, yaw

    def update_visualization(self, vis):
        current_time = time.time()
        elapsed = current_time - self.last_update
        
        if elapsed >= self.frame_time:
            try:
                # Obtener ángulos suavizados
                roll, pitch, yaw = self.get_smoothed_angles()
                print("datos de update_visualization roll: ", roll, "pitch: ", pitch, "yaw: ", yaw)
                
                # Convertir a radianes
                rx = radians(roll)
                ry = radians(pitch)
                rz = radians(yaw)
                
                print(f"Ángulos (rad) - Roll: {rx:.2f}, Pitch: {ry:.2f}, Yaw: {rz:.2f}")  # Debug
                
                # Resetear el mesh a su orientación original
                self.mesh = o3d.io.read_triangle_mesh(self.stl_path)
                self.mesh.compute_vertex_normals()
                
                # Crear una matriz de rotación combinada
                R = self.mesh.get_rotation_matrix_from_xyz((rx, ry, rz))
                
                # Aplicar la rotación usando el centro como numpy array
                self.mesh.rotate(R, center=self.mesh.get_center())
                
                # Forzar actualización de geometría
                vis.clear_geometries()
                vis.add_geometry(self.mesh)
                vis.poll_events()
                vis.update_renderer()
                
                print("Geometría actualizada")  # Debug
                
                self.last_update = current_time
                
            except Exception as e:
                print(f"Error en actualización visual: {e}")
                import traceback
                traceback.print_exc()
        
        return True

    def run(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="STL Viewer")
        self.vis.add_geometry(self.mesh)
        
        view_control = self.vis.get_view_control()
        view_control.set_zoom(0.8)
        
        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0.5, 0.5, 0.5])
        opt.point_size = 1.0
        opt.show_coordinate_frame = True
        
        print("Visualización iniciada. Presiona Ctrl+C para salir.")
        print("Esperando datos seriales...")
        
        try:
            while True:
                if not self.update_visualization(self.vis):
                    break
                time.sleep(0.001)  # Pequeña pausa para no saturar la CPU
                
        except KeyboardInterrupt:
            print("\nCerrando visualización...")
        finally:
            self.running = False
            self.serial_port.close()
            self.vis.destroy_window()

def main():
    stl_path = "/home/anderson/camara/src/tesis_launch/tesis_launch/POCHITA_v30.stl"
    port = '/dev/ttyUSB0'
    baudrate = 115200
    
    try:
        viewer = STLViewer(stl_path, port, baudrate)
        viewer.run()
    except serial.SerialException as e:
        print(f"Error al abrir el puerto serial: {e}")
        print(f"Verifica que:")
        print(f"1. El dispositivo está conectado al puerto {port}")
        print(f"2. Tienes permisos para acceder al puerto (sudo chmod 666 {port})")
        print(f"3. El puerto no está siendo usado por otro programa")
    except Exception as e:
        print(f"Error inesperado: {e}")

if __name__ == "__main__":
    main()