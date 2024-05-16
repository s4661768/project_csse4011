import serial 
from gesture import Gesture
from mqtt import MQTT

class Thingy52_Receiver:
    def __init__(self, mqtt: MQTT, com_port: str, baud_rate: int):
        self.mqtt = mqtt
        # Initialize serial port
        print("Thingy52_Receiver")
        try:
            self.ser = serial.Serial(com_port, baud_rate)
            self.ser.write("blecon -s 0c:0c:18:4c:b7:f1\n".encode('ascii'))
            print("Connected to", com_port)

            self.handle_serial()
        except serial.SerialException as e:
            print("Error connecting to serial port:", e)
            self.ser = None



    def handle_serial(self):
        """ Handle serial data reading and immediately publish to MQTT in a single thread. """
        try:
            while True:
                if self.ser.in_waiting > 0:
                    received_data = self.ser.readline().decode('utf-8').strip()
                    # print("Received:", received_data)

                    if (received_data == ""):
                        continue

                    if (len(received_data) == 0):
                        continue      

                    if (len(received_data) == 1): 
                        print(f"Direction Command ------------------------------- {received_data}")
                        self.mqtt.publish_gesture_data_thingy52(received_data)
                        continue
                    else:
                        print(received_data)
                    
        except serial.SerialException as e:
            print("Serial port error:", e)
        finally:
            if self.ser.is_open:
                self.ser.close()
                print("Serial port closed")