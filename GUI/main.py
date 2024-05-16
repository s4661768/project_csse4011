import threading

from gui import  GUI
from gesture import Gesture
from gesture_recogniser import Gesture_Recogniser
from mqtt import MQTT
from thingy52 import Thingy52_Receiver

def main():
    m = MQTT()
    # print(m)
    threading.Thread(target=Thingy52_Receiver, args=(m, 'COM11', 115200,), daemon=True).start()
    
    threading.Thread(target=GUI, args=(m,), daemon=True).start()
    gr = Gesture_Recogniser(m) # Must by called in main thread
    

if __name__ == "__main__":
    main()