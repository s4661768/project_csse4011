import threading
import numpy as np
import cv2
import tensorflow as tf
from google.protobuf.internal import builder as _builder
from cvzone.HandTrackingModule import HandDetector
import paho.mqtt.client as mqtt
import json
import tkinter as tk
from PIL import Image, ImageTk
from enum import Enum
import serial
import time

class Gesture(Enum):
    STOP = 0
    FORWARD = 1
    BACK = 2
    LEFT = 3
    RIGHT = 4   
    HOME = 5


class MQTT:

    def __init__(self):
            # MQTT STUFF
            self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            self.mqttc.on_connect = self.on_connect
            self.mqttc.on_message = self.on_message
            self.mqttc.on_subscribe = self.on_subscribe
            self.mqttc.on_unsubscribe = self.on_unsubscribe
            self.mqttc.user_data_set([])
            self.mqttc.connect("csse4011-iot.zones.eait.uq.edu.au")
            self.mqtt_thread = threading.Thread(target=self.mqtt_process,daemon=True).start()

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
            if reason_code_list[0].is_failure:
                print(f"Broker rejected your subscription: {reason_code_list[0]}")
            else:
                print(f"Broker granted the following QoS: {reason_code_list[0].value}")

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            print("Unsubscribe succeeded.")
        else:
            print(f"Broker replied with failure: {reason_code_list[0]}")
        client.disconnect()

    def on_message(self, client, userdata, message):
        inval = message.payload.decode()
        # print("got message", inval)
        self.last_rec_command = inval

    def on_connect(self, client, userdata, flags, reason_code, properties):
            if reason_code.is_failure:
                print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
            else:
                client.subscribe("un44779195")

    def publish_odometry_data(self, odom_data):
        self.mqttc.publish("un44779195", json.dumps(odom_data))

    def publish_gesture_data(self, movement_data: int):
        self.mqttc.publish("un44779195", json.dumps(movement_data))

    def mqtt_process(self):
        self.mqttc.loop_forever()



class GUI:
    def __init__(self):
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
        self._gest_rec = threading.Thread(target=Gesture_Recogniser, args=(self,), daemon=True)
        self._gest_rec.start()

        self._hand_rec_gesture = Gesture.STOP
        self._imu_gesture = Gesture.STOP

    def _gesture_source_button_click(self):
        # Define what happens when the button is clicked
        self._gesture_input = (self._gesture_input + 1) % 2
        if (self._gesture_input == 0):
            gesture_source = "Hand Recognition"
            self.gesture_source_button.config(text=gesture_source, bg='yellow')
        else:
            gesture_source = "IMU Data" 
            self.gesture_source_button.config(text=gesture_source, bg='grey')
        
        # self._gesture_source_label.config(text="Movement Command: "  + gesture_source)  # Update the label text

    def update_cmd_label(self, gest) -> None:
        while True:
            if (self._gesture_input == 0):
                self._cmd_label.config(text=f"Current Command: {gest.__str__().split('.')[1]}")
            elif (self._gesture_input == 1):
                self._cmd_label.config(text=f"Current Command: {gest.__str__().split('.')[1]}")



# Hands array is [thumb index middle ring pinky]

  





class Gesture_Recogniser:
    def __init__(self, gui: GUI, mqtt: MQTT_class):
        self.gui = gui
        self.mqtt = mqtt
        cv2.namedWindow("test")
        
        self.cam = cv2.VideoCapture(0)
        
        self._handDetector = HandDetector(detectionCon = 0.5, maxHands = 1)
        while True:  
            ret, frame = self.cam.read()
            frame = cv2.flip(frame, 1)

            if not ret:
                print("Failed to get frame")
                break

            hands, frame = self._handDetector.findHands(frame)
            
            if hands:
                cv2.imshow("test", frame)
                hands1 = hands[0]
                fingers = self._handDetector.fingersUp(hands1)
                # print(fingers)
                print(self.get_gesture(fingers))
                code = self.get_gest_code(self.get_gesture(fingers))
                self.mqtt.publish_gesture_data(code)
                # self.gui.update_cmd_label(self.get_gesture(fingers))
                    
            else:        
                ret, frame1 = self.cam.read()
                cv2.imshow("test", frame1)
        # self.recognise_hands()

    def get_gesture(self, fingers: list) -> Gesture:
        gest = None
        # Check for invalid arrays and set their gesture to Gesture.STOP
        # if (fingers[0] != 0) or (fingers.count(0) == 5):
        #     print("invalid arr")
        #     return 
        
        num = -1
        for i in range(0, len(fingers)):
            num += (2 ** i) * fingers[i]
        

        if num == 29:
            gest = Gesture.FORWARD
        elif num == 27:
            gest = Gesture.BACK
        elif num == 5:
            gest = Gesture.LEFT
        elif num == 1:
            gest = Gesture.RIGHT
        else:
            gest = Gesture.STOP

        return gest

    def get_gest_code(self, gest: Gesture) -> int:
        code = 0
        if gest == Gesture.FORWARD:
            code = 1
        elif gest == Gesture.BACK:
            code = 2
        elif gest == Gesture.LEFT:
            code = 3
        elif gest == Gesture.RIGHT:
            code = 4

        return code


    def close_window(self):
        self.cam.release()
        cv2.destroyAllWindows()



# Connors Code from his "test.py"
def on_publish(client, userdata, mid, reason_code_list, properties):
    # print("Message {} published successfully.".format(mid))
    pass

def handle_serial_and_publish(ser, client, topic):
    """ Handle serial data reading and immediately publish to MQTT in a single thread. """
    try:
        while True:
            if ser.in_waiting > 0:
                received_data = ser.readline().decode('utf-8').strip()
                # print("Received:", received_data)

                if (received_data == ""):
                    continue

                if (len(received_data) == 0):
                    continue      

                if (len(received_data) == 1): 
                    print(f"Direction Command ------------------------------- {received_data}")
                    # payload = json.dumps({"data": received_data})
                    payload = json.dumps(received_data)
                    msg_info = client.publish(topic, payload, qos=0)
                    msg_info.wait_for_publish()
                    # print("Published:", received_data)
                    continue
                else:
                    print(received_data)
                
    except serial.SerialException as e:
        print("Serial port error:", e)
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")

# com_port = 'COM11'  # Change this to the appropriate COM port
# baud_rate = 115200         # Change this to match the baud rate of your device

# # Initialize MQTT client
# mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
# mqttc.on_publish = on_publish
# mqttc.connect("csse4011-iot.zones.eait.uq.edu.au")
# mqttc.loop_start()

# topic = "un44779195"

# # Initialize serial port
# try:
#     ser = serial.Serial(com_port, baud_rate)
#     ser.write("blecon -s 26:e9:ec:20:ff:6f\n".encode('ascii'))
#     print("Connected to", com_port)
# except serial.SerialException as e:
#     print("Error connecting to serial port:", e)
#     ser = None

# # Start a single thread for handling both reading from serial and publishing to MQTT
# if ser:
#     threading.Thread(target=handle_serial_and_publish, args=(ser, mqttc, topic), daemon=True).start()


# try:
#     while True:  # Keep the main thread running
#         time.sleep(10)
# except KeyboardInterrupt:
#     print("Program terminated by user")

# # Cleanup
# mqttc.disconnect()
# mqttc.loop_stop()
# if ser and ser.is_open:
#     ser.close()


# g = GUI()
# g.root.mainloop()
m = MQTT_class()
gesture_rec = Gesture_Recogniser(1, m)


# recognise_hands()



# root.mainloop()

# self.mqtt_thread = threading.Thread(target=self.mqtt_process,daemon=True).start()

# Run the Tkinter event loop

# root.mainloop()