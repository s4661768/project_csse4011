""" This file contains the Thingy52_Receiver class. It is used to read serial data from the DK
board and send the data to the MQTT class to be published."""


import serial 
from mqtt import MQTT

class Thingy52_Receiver:
    def __init__(self, mqtt: MQTT):
        self.mqtt = mqtt
        # Initialize serial port

    def open_serial_port(self, com_port: str, baud_rate: int):
        try:
            self.ser = serial.Serial(com_port, baud_rate)

            # Tell the DK board to connect to the Thingy52 with tis specific MAC address
            self.ser.write("blecon -s 0c:0c:18:4c:b7:f1\n".encode('ascii')) 
            print("Connected to", com_port)

            return True, self.handle_serial() # Start listening for serial data
        except serial.SerialException as e:
            print("Error connecting to serial port:", e)
            self.ser = None
            return False, None



    def handle_serial(self):
        """ Handle serial data reading and immediately send data to MQTT class to publish. """
        try:
            while True:
                if self.ser.in_waiting > 0:
                    received_data = self.ser.readline().decode('utf-8').strip()

                    if (received_data == "") or (len(received_data) == 0):
                        continue    

                    if (len(received_data) == 1): # If we received a single character, it is a gesture command
                        self.mqtt.publish_gesture_data_thingy52(received_data)
                        continue
                    else: # If we received more than a single character, it is a terminal or debug message from the DK board
                        print(received_data)
                    
        except serial.SerialException as e:
            print("Serial port error:", e)

        finally:
            if self.ser.is_open:
                self.ser.close()
                print("Serial port closed")