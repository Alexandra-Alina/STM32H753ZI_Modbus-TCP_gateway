import socket
import paho.mqtt.client as mqtt
import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host = '192.168.0.110'
port = 7

s.connect((host, port))

mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
mqtt_client.connect("host.docker.internal", 1883, 60)

while True:
    s.sendall(bytes("AA0300010001CC11", 'utf-8'))
    data = s.recv(1024).decode('utf-8')
    mqtt_client.publish("sensors/temperature", data)
    print(data)
    time.sleep(2);
