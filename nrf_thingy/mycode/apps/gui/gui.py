import serial
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
mqttc.connect("csse4011-iot.zones.eait.uq.edu.au")
# mqttc.connect("csse4011-iot.zones.eait.uq.edu.au")
mqttc.loop_start()

# Hardcoded topic name0
topic = "un46591300"
# Hardcoded QoS level


com_port = '/dev/ttyACM0'  # Change this to the appropriate COM port
baud_rate = 115200   # Change this to match the baud rate of your device

qos = 0

x = 0.0
y = 0.0
yaw = -3.14

# Increment x, y, and yaw values in each iteration
while yaw <= 3.14:
    # Create a JSON object with x, y, and yaw
    payload = json.dumps({"position": {"x": x, "y": y, "yaw": yaw}})
    msg_info = mqttc.publish(topic, payload, qos=qos)
    unacked_publish.add(msg_info.mid)
    # Wait for the message to be published
    msg_info.wait_for_publish()
    
    # Increment x, y, and yaw values
    x += 0.01
    y += 0.01
    yaw += 0.01
    print(x)
    print(y)
    print(yaw)
    time.sleep(0.01)

try:
    ser = serial.Serial(com_port, baud_rate)
    print("Connected to", com_port)

    while True:
        if ser.in_waiting > 0:
            received_data = ser.readline().decode('utf-8').strip()
            print("Received:", received_data)

except serial.SerialException as e:
    print("Serial port error:", e)
finally:
    if ser.is_open:
        ser.close()
        print("Serial port closed")



# Disconnect from the MQTT broker and stop the loop
mqttc.disconnect()
mqttc.loop_stop()