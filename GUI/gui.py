import threading
from mqtt import MQTT 
import tkinter as tk
# from gesture_recogniser import Gesture_Recogniser
from gesture import Gesture
from time import sleep

class GUI:
    def __init__(self, mqtt: MQTT):
        self.mqtt = mqtt
        # Create the main window
        self._gesture_input = 0 # 0 -> Hand recognition, 1 -> IMU data
        self.root = tk.Tk()
        self.root.title("GUI with Button and Label")

        
        # Create a button
        self.gesture_source_button = tk.Button(self.root, text="Hand Recognition", bg='yellow', command=self._gesture_source_button_click)
        self.gesture_source_button.pack()
        # Create a label
        self._cmd_label = tk.Label(self.root, text="Current Command: STOP")
        self._cmd_label.pack()
        # self._gest_rec = threading.Thread(target=Gesture_Recogniser, args=(self,), daemon=True)
        # self._gest_rec.start()

        self._hand_rec_gesture = Gesture.STOP
        self._imu_gesture = Gesture.STOP
        
        threading.Thread(target=self.update_cmd_label,  daemon=True).start()

        # Start GUI
        self.root.mainloop()
        

    def _gesture_source_button_click(self):
        # Define what happens when the button is clicked
        self._gesture_input = (self._gesture_input + 1) % 2
        if (self._gesture_input == 0):
            gesture_source = "Hand Recognition"
            self.gesture_source_button.config(text=gesture_source, bg='yellow')
            self.mqtt.set_input_source(0)
        else:
            gesture_source = "IMU Data" 
            self.gesture_source_button.config(text=gesture_source, bg='grey')
            self.mqtt.set_input_source(1)
        
        # self._gesture_source_label.config(text="Movement Command: "  + gesture_source)  # Update the label text

    def update_cmd_label(self) -> None:
        while True:
            if (self.mqtt.get_input_source() == 0):
                self._cmd_label.config(text=f"Current Command: {self.mqtt.get_last_hand_gesture().__str__().split('.')[1]}")
                sleep(0.01)
            elif (self.mqtt.get_input_source() == 1):
                self._cmd_label.config(text=f"Current Command: {self.mqtt.get_last_thingy52_gesture().__str__().split('.')[1]}")
                sleep(0.1)
