""" This file is the main file for the project. It initialises and start 
all of the required classes to recognise hand gesture with the webcam, and
read serial messages from the Thingy52 and DK board. It also publishes the 
data using the MQTT class. """


import threading
import serial

from gui import  GUI
from gesture_recogniser import Gesture_Recogniser
from mqtt import MQTT
from thingy52 import Thingy52_Receiver

def main():
    print("Starting...")
    com_baud = input("Enter COM port and baud rate (e.g. COM11 115200): ")
    try:
        com_port, baud_rate = com_baud.split(" ")
        baud = int(baud_rate)
        if baud < 9600:
            raise ValueError("Invalid baud rate: Too low")
        print(com_port, baud_rate)
        # self.ser = serial.Serial(com_port, baud_rate)
    except ValueError:
        print("Invalid input")
        exit(1)

    try:
        # Testing serial port
        ser = serial.Serial(com_port, baud_rate)
        ser.close()
    except serial.SerialException as e:    
        print("Error connecting to serial port:", e)
        exit(1)

    """ If the serial port opened successfully we can continue with the rest of the program. """

    mqtt = MQTT() # Make MQTT object
    
    # # Make the start the Thingy52_Receiver in a separate thread
    threading.Thread(target=Thingy52_Receiver, args=(mqtt, com_port, baud,), daemon=True).start()
    
    # # Make the start the GUI in a separate thread
    threading.Thread(target=GUI, args=(mqtt,), daemon=True).start()

    # # Make the start the Gesture_Recogniser in a separate thread
    # # Must by called in main thread to use the webcam properly
    gr = Gesture_Recogniser(mqtt)
    

if __name__ == "__main__":
    main()