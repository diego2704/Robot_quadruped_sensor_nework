import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
from pydub import AudioSegment
import speech_recognition as sr
import numpy as np

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')
        self.publisher_ = self.create_publisher(String, 'audio_text', 10)

        # Updated parameters to match ALC295 Analog
        self.sample_rate = 48000
        self.duration = 5
        self.channels = 2  # ALC295 requires 2 channels
        self.device_id = 4  # Device index for ALC295 Analog

        self.recognizer = sr.Recognizer()
        self.timer = self.create_timer(6, self.timer_callback)
        self.get_logger().info("Nodo de audio inicializado. Grabando cada 5 segundos.")

    def timer_callback(self):
        try:
            self.get_logger().info(f"Grabando durante {self.duration} segundos...")

            # Record audio with 2 channels
            audio_data = sd.rec(
                int(self.sample_rate * self.duration),
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='int16',
                device=self.device_id
            )
            sd.wait()

            # Convert stereo to mono for speech recognition
            mono_data = np.mean(audio_data, axis=1).astype('int16')

            # Convert to AudioSegment (using mono data)
            audio_segment = AudioSegment(
                mono_data.tobytes(),
                frame_rate=self.sample_rate,
                sample_width=2,  # 16-bit audio
                channels=1  # Mono for speech recognition
            )
            audio_segment.export("temp.wav", format="wav")

            with sr.AudioFile("temp.wav") as source:
                audio_text = self.recognizer.record(source)
                text = self.recognizer.recognize_google(audio_text)
                self.get_logger().info(f"Texto reconocido: {text}")

                msg = String()
                msg.data = text
                self.publisher_.publish(msg)

        except sr.UnknownValueError:
            self.get_logger().info("No se pudo entender el audio.")
        except sr.RequestError as e:
            self.get_logger().error(f"Error en la solicitud de reconocimiento de habla: {e}")
        except Exception as e:
            self.get_logger().error(f"Error inesperado: {e}")

def main(args=None):
    rclpy.init(args=args)
    audio_publisher = AudioPublisher()
    try:
        rclpy.spin(audio_publisher)
    except KeyboardInterrupt:
        audio_publisher.get_logger().info("Nodo detenido por el usuario.")
    finally:
        audio_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


