import paho.mqtt.client as mqtt

class MQTTClient():
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client = mqtt.Client()
        
        self.topics = ['sensor/DHT11/humedad', 'sensor/DHT11/temperatura', 
                       'sensor/HC-SR04/uno', 'sensor/HC-SR04/dos', 
                       'sensor/VL53L0X/distancia', 'sensor/MPU6050/angulos']
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(self.host, self.port, 60)
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected successfully " + str(rc))
        for topic in self.topics:
            self.client.subscribe(topic)
    
    def on_message(self, client, userdata, msg):
        print(f"Received Message: {msg.payload.decode()}")

if __name__ == '__main__':
    mqtt_client = MQTTClient('192.168.100.216', 1883) # 192.168.100.69