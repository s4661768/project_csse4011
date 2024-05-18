""" This file is the main file for the project. It initialises and start 
all of the required classes to recognise hand gesture with the webcam, and
read serial messages from the Thingy52 and DK board. It also publishes the 
data using the MQTT class. """


import threading

from gui import  GUI
from gesture_recogniser import Gesture_Recogniser
from mqtt import MQTT
from thingy52 import Thingy52_Receiver

def main():
    mqtt = MQTT() # Make MQTT object
    
    # Make the start the Thingy52_Receiver in a separate thread
    threading.Thread(target=Thingy52_Receiver, args=(mqtt, "COM11", 115200,), daemon=True).start()
    
    # Make the start the GUI in a separate thread
    threading.Thread(target=GUI, args=(mqtt,), daemon=True).start()

    # Make the start the Gesture_Recogniser in a separate thread
    # Must by called in main thread to use the webcam properly
    gr = Gesture_Recogniser(mqtt)
    

if __name__ == "__main__":
    main()