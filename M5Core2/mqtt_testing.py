import time
import paho.mqtt.client as mqtt

def on_publish(client, userdata, mid):
    print("Message {} published successfully.".format(mid))

unacked_publish = set()
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# Set the on_publish callback function explicitly
mqttc.message_callback_add("$SYS/broker/log/M", on_publish)

mqttc.user_data_set(unacked_publish)
mqttc.connect("mqtt.eclipseprojects.io")
mqttc.loop_start()

# Hardcoded topic name
topic = "46591300_General"
# Hardcoded QoS level
qos = 0

while True:
    message = input("Enter the message to publish (or 'exit' to quit): ")
    if message.lower() == "exit":
        break
    
    msg_info = mqttc.publish(topic, message, qos=qos)
    unacked_publish.add(msg_info.mid)

    # Wait for the message to be published
    msg_info.wait_for_publish()

# Disconnect from the MQTT broker and stop the loop
mqttc.disconnect()
mqttc.loop_stop()
