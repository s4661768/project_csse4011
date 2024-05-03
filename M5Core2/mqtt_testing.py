import time
import json
import paho.mqtt.client as mqtt

def on_publish(client, userdata, mid):
    print("Message {} published successfully.".format(mid))

unacked_publish = set()
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# Set the on_publish callback function explicitly
mqttc.message_callback_add("$SYS/broker/log/M", on_publish)

mqttc.user_data_set(unacked_publish)
mqttc.connect("mqtt.eclipseprojects.io")
# mqttc.connect("csse4011-iot.zones.eait.uq.edu.au")
mqttc.loop_start()

# Hardcoded topic name
topic = "un46591300"
# Hardcoded QoS level
qos = 0

while True:
    message = input("Enter the message to publish in the format 'x,y,yaw' (or 'exit' to quit): ")
    if message.lower() == "exit":
        break
    
    # Split the input into x, y, and yaw
    try:
        x, y, yaw = map(float, message.split(','))
        # Create a JSON object with x, y, and yaw
        payload = json.dumps({"position": {"x": x, "y": y, "yaw": yaw}})
        msg_info = mqttc.publish(topic, payload, qos=qos)
        unacked_publish.add(msg_info.mid)
        # Wait for the message to be published
        msg_info.wait_for_publish()
    except ValueError:
        print("Invalid input format. Please enter in the format 'x,y,yaw'.")

# Disconnect from the MQTT broker and stop the loop
mqttc.disconnect()
mqttc.loop_stop()
