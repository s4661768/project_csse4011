import threading
import numpy as np
import cv2
import tensorflow as tf
from google.protobuf.internal import builder as _builder
from cvzone.HandTrackingModule import HandDetector
import paho.mqtt.client as mqtt
import json

class MQTT_class:

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

    def publish_gesture_data(self, gesture_data: list):
        self.mqttc.publish("un44779195", json.dumps(gesture_data))

    def mqtt_process(self):
        self.mqttc.loop_forever()

    


print(cv2.__version__)
cam = cv2.VideoCapture(0)

cv2.namedWindow("test")
handDetector = HandDetector(detectionCon = 0.5, maxHands = 1)
faceDetector = FaceDetector()

img_counter = 0

m = MQTT_class()


while True:
    ret, frame0 = cam.read()
    if not ret:
            print("Failed to get frame")
            break
    hands, frame0 = handDetector.findHands(frame0)
    cv2.imshow("test", frame0)

    if hands:
        hands1 = hands[0]
        fingers = handDetector.fingersUp(hands1)
        print(fingers)
        m.publish_gesture_data(fingers)
        

    k = cv2.waitKey(1)

    if k%256 == 27: # esc key
          print("Closing Window")
          break

cam.release()
cam.destroyAllWindows()


