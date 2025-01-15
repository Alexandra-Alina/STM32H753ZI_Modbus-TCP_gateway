import paho.mqtt.client as mqtt
import logging

logging.warning("Start subscriber")

def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    client.subscribe("sensors/#", qos=0)

def on_message(client, userdata, msg):
    logging.warning(msg.topic + " " + str(msg.payload))

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect("host.docker.internal", 1883, 60)
mqttc.loop_forever()
