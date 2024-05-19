import threading
from mqtt import MQTT 
import tkinter as tk
from time import sleep

from gesture import Gesture
from thingy52 import Thingy52_Receiver


class GUI:
    def __init__(self, mqtt: MQTT):
        self.mqtt = mqtt

        # Create the main window
        self._gesture_input = 0 # 0 -> Hand recognition, 1 -> IMU data
        self.root = tk.Tk()
        self.root.title("Gesture Controlled Turtle Bot")
        
        # Create a toggle for input mode
        self.gesture_source_button = tk.Button(self.root, text="Hand Recognition", bg='yellow',font=('Arial', 30), command=self._gesture_source_button_click)
        self.gesture_source_button.pack()

        # Create a toggle for publishing
        self.publish_toggle = tk.Button(self.root, text="Not Publishing", bg='red', font=('Arial', 30), command=self._publish_toggle_callback)
        self.publish_toggle.pack()
        # Create a label
        self._cmd_label = tk.Label(self.root, text="Current Command: STOP", font=('Arial', 30))
        self._cmd_label.pack()

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
            self.gesture_source_button.config(text=gesture_source, bg='yellow', font=('Arial', 30))
            self.mqtt.set_input_source(0)
        else:
            gesture_source = "IMU Data" 
            self.gesture_source_button.config(text=gesture_source, bg='grey', font=('Arial', 30))
            self.mqtt.set_input_source(1)

        
    def _publish_toggle_callback(self):
        self.mqtt.publish_toggle()
        print(self.mqtt.get_publish())
        if (self.mqtt.get_publish() == 1):
            self.publish_toggle.config(text="Publishing", bg='green',font=('Arial', 30))
        else:
            self.publish_toggle.config(text="Not Publishing", bg='red', font=('Arial', 30))


    def update_cmd_label(self) -> None:
        while True:
            if (self.mqtt.get_input_source() == 0):
                self._cmd_label.config(text=f"Current Command: {self.mqtt.get_last_hand_gesture().__str__().split('.')[1]}" ,font=('Arial', 30))
                sleep(0.01)
            elif (self.mqtt.get_input_source() == 1):
                self._cmd_label.config(text=f"Current Command: {self.mqtt.get_last_thingy52_gesture().__str__().split('.')[1]}", font=('Arial', 30))
                sleep(0.1)
