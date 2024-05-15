import serial
import threading
import time
import json
import paho.mqtt.client as mqtt

def on_publish(client, userdata, mid, reason_code_list, properties):
    print("Message {} published successfully.".format(mid))

def handle_serial_and_publish(ser, client, topic):
    """ Handle serial data reading and immediately publish to MQTT in a single thread. """
    try:
        while True:
            if ser.in_waiting > 0:
                received_data = ser.readline().decode('utf-8').strip()
                print("Received:", received_data)
                # payload = json.dumps({"data": received_data})
                payload = json.dumps(received_data)
                msg_info = client.publish(topic, payload, qos=0)
                msg_info.wait_for_publish()
                print("Published:", received_data)
    except serial.SerialException as e:
        print("Serial port error:", e)
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")

com_port = '/dev/ttyACM0'  # Change this to the appropriate COM port
baud_rate = 115200         # Change this to match the baud rate of your device

# Initialize MQTT client
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_publish = on_publish
mqttc.connect("csse4011-iot.zones.eait.uq.edu.au")
mqttc.loop_start()

topic = "un44779195"

# Initialize serial port
try:
    ser = serial.Serial(com_port, baud_rate)
    print("Connected to", com_port)
except serial.SerialException as e:
    print("Error connecting to serial port:", e)
    ser = None

# Start a single thread for handling both reading from serial and publishing to MQTT
if ser:
    threading.Thread(target=handle_serial_and_publish, args=(ser, mqttc, topic), daemon=True).start()

try:
    while True:  # Keep the main thread running
        time.sleep(10)
except KeyboardInterrupt:
    print("Program terminated by user")

# Cleanup
mqttc.disconnect()
mqttc.loop_stop()
if ser and ser.is_open:
    ser.close()
