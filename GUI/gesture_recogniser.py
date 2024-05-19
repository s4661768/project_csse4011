import cv2
import tensorflow as tf
from google.protobuf.internal import builder as _builder
from cvzone.HandTrackingModule import HandDetector
from gui import GUI
from gesture import Gesture
from mqtt import MQTT
from time import sleep

class Gesture_Recogniser:
    def __init__(self, mqtt: MQTT):
        self.mqtt = mqtt
        self.cam = cv2.VideoCapture(0)
        cv2.namedWindow("CSSE4011 Project Gesture recogniser")
        self._handDetector = HandDetector(detectionCon = 0.5, maxHands = 1)
        
        while True:  
            k = cv2.waitKey(1)    
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
                code = self.get_gest_code(self.get_gesture(fingers))
                self.mqtt.publish_gesture_data_hand(code)
                    
            else:        
                ret, frame1 = self.cam.read()
                cv2.imshow("test", cv2.flip(frame1, 1))
            sleep(0.05)
      

        

    def get_gesture(self, fingers: list) -> Gesture:
        gest = None
        
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
        elif num == 2:
            gest = Gesture.HOME
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
        elif gest == Gesture.HOME:
            code = 5

        return code


    def close_window(self):
        self.cam.release()
        cv2.destroyAllWindows()
