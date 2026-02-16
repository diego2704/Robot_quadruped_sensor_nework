import serial
import pyvista as pv
import numpy as np
import re
import time  # Para agregar una pausa en el bucle

# Configuración del puerto serial
serial_port = '/dev/ttyUSB0'  # Cambiar por el puerto correspondiente
baud_rate = 115200

# Cargar el modelo STL
filename = "/home/anderson/camara/src/tesis_launch/tesis_launch/POCHITA_v30.stl"  # Cambiar por la ruta de tu archivo STL
mesh = pv.read(filename)

# Configuración de la ventana interactiva
plotter = pv.Plotter()
plotter.add_mesh(mesh, color="white")
plotter.show_axes()

# Función para crear una matriz de rotación
def rotation_matrix(roll, pitch, yaw):
    # Convertir ángulos de grados a radianes
    roll, pitch, yaw = np.radians([roll, pitch, yaw])

    # Matrices de rotación individuales
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combinar las matrices de rotación (orden ZYX)
    return Rz @ Ry @ Rx

# Conectar al puerto serial
try:
    ser = serial.Serial(serial_port, baud_rate)
    print(f"Conectado al puerto {serial_port}")
except Exception as e:
    print(f"Error al conectar al puerto serial: {e}")
    exit()

# Función para extraer los valores de Roll, Pitch y Yaw del nuevo formato de datos
def extract_rpy_from_string(data_lines):
    roll, pitch, yaw = None, None, None

    for line in data_lines:
        if "Enviado Roll=" in line and "Pitch=" in line:
            match = re.search(r"Enviado Roll=(-?\d+\.\d+)\s*Pitch=(-?\d+\.\d+)", line)
            if match:
                roll = float(match.group(1))
                pitch = float(match.group(2))
        elif "Enviado Yaw=" in line:
            match = re.search(r"Enviado Yaw=(-?\d+\.\d+)", line)
            if match:
                yaw = float(match.group(1))

    if roll is not None and pitch is not None and yaw is not None:
        print(f"Extraído: Roll={roll}, Pitch={pitch}, Yaw={yaw}")
        return roll, pitch, yaw
    else:
        print("No se pudieron extraer todos los valores de Roll, Pitch y Yaw.")
        return None, None, None

# Función para actualizar la posición del modelo
def update_mesh():
    previous_angles = (0, 0, 0)  # Para almacenar los valores anteriores de roll, pitch, yaw
    original_points = mesh.points.copy()  # Guardar las coordenadas originales del modelo
    data_buffer = []  # Almacena líneas temporales para procesar los datos

    while True:
        try:
            if ser.in_waiting > 0:
                # Leer línea del puerto serial
                line = ser.readline().decode('utf-8').strip()
                print(f"Datos crudos recibidos: {line}")

                # Acumular las líneas en el buffer
                data_buffer.append(line)

                # Procesar los datos cuando tengamos suficientes líneas
                if len(data_buffer) >= 3:  # Se necesitan al menos 3 líneas para Roll, Pitch y Yaw
                    roll, pitch, yaw = extract_rpy_from_string(data_buffer)
                    data_buffer.clear()  # Limpiar el buffer después de procesar

                    if roll is not None and pitch is not None and yaw is not None:
                        # Solo actualizar si los valores han cambiado significativamente
                        if abs(roll - previous_angles[0]) > 1 or \
                           abs(pitch - previous_angles[1]) > 1 or \
                           abs(yaw - previous_angles[2]) > 1:

                            # Calcular la nueva matriz de rotación
                            R = rotation_matrix(roll, pitch, yaw)

                            # Aplicar la rotación a las coordenadas originales
                            transformed_points = (R @ original_points.T).T

                            # Actualizar las coordenadas del modelo STL
                            mesh.points[:] = transformed_points

                            # Actualizar la visualización
                            plotter.update_coordinates(mesh.points)
                            plotter.render()

                            # Guardar los valores actuales para la siguiente comparación
                            previous_angles = (roll, pitch, yaw)
                        else:
                            print("Sin cambios significativos en los valores de Roll, Pitch, Yaw. No actualizando rotación.")

            # Agregar una pequeña pausa para evitar actualizaciones demasiado frecuentes
            time.sleep(0.01)

        except KeyboardInterrupt:
            print("Cerrando conexión...")
            ser.close()
            break
        except Exception as e:
            print(f"Error: {e}")

# Ejecutar la visualización
plotter.show(interactive_update=True)
update_mesh()
