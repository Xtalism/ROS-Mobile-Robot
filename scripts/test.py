import paho.mqtt.client as mqtt
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

        self.data = {topic: [] for topic in self.topics}

        self.client.connect(self.host, self.port, 60)
        self.client.loop_start()

        self.setup_plot()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected successfully " + str(rc))
        for topic in self.topics:
            self.client.subscribe(topic)
    
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        print(f"Received Message: {payload} on topic: {topic}")
        self.data[topic].append(float(payload))

    def setup_plot(self):
        self.fig, self.ax = plt.subplots()
        self.lines = {topic: self.ax.plot([], [], label=topic)[0] for topic in self.topics}
        self.ax.legend()
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=1000)
        plt.show()

    def update_plot(self, frame):
        for topic, line in self.lines.items():
            line.set_data(range(len(self.data[topic])), self.data[topic])
        self.ax.relim()
        self.ax.autoscale_view()

if __name__ == '__main__':
    mqtt_client = MQTTClient('192.168.100.216', 1883)