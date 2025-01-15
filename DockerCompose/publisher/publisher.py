import paho.mqtt.client as mqtt
import time
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect

mqttc.connect("localhost", 1883, 60)

mqttc.loop_start()

while True:
    temperature = 25.0
    mqttc.publish("sensors/temperature", temperature)
    time.sleep(1)
    print("Published: ", temperature)

mqttc.loop_stop()