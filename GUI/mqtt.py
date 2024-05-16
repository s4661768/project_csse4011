import paho.mqtt.client as mqtt
import threading
import json
import gesture as g
from gesture import Gesture
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

            # Gesture Stuff
            self.input_source = 0 # 0 -> Hand recognition, 1 -> IMU data
            self.last_hand_gesture = Gesture.STOP
            self.last_thingy52_gesture = Gesture.STOP

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

    def publish_gesture_data_hand(self, movement_data: int):
        self.last_hand_gesture = g.get_gest_value(movement_data)
        if self.input_source == 0:
            
            self.mqttc.publish("un44779195", json.dumps(movement_data))

    def publish_gesture_data_thingy52(self, movement_data: int):
        i = int(movement_data)
        self.last_thingy52_gesture = g.get_gest_value(i)
        if self.input_source == 1:
        
            print(f"This is the number ------------------ {i}")
            self.mqttc.publish("un44779195", json.dumps(i))

    def mqtt_process(self):
        self.mqttc.loop_forever()

    def set_input_source(self, input_source):
        self.input_source = input_source

    def get_input_source(self):
        return self.input_source
    
    def get_last_hand_gesture(self):
        return self.last_hand_gesture
    
    def get_last_thingy52_gesture(self):
        print(self.last_thingy52_gesture)
        return self.last_thingy52_gesture
