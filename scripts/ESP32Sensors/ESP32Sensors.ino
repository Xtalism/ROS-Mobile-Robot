#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <VL53L0X.h>

#define DHTTYPE DHT11
#define DHTPIN 19
DHT dht(DHTPIN, DHTTYPE);

const int trigPin = 12;
const int echoPin = 13;
const int trigPin2 = 27;
const int echoPin2 = 26;
#define SOUND_SPEED 0.034

VL53L0X sensor;

const char* ssid = "MotherBase";
const char* password = "@6830135@";
const char* mqtt_server = "192.168.100.216";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  dht.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  float distanceCm = measureDistance(trigPin, echoPin);
  float distanceCm2 = measureDistance(trigPin2, echoPin2);

  int distanciaVL53L0X = sensor.readRangeContinuousMillimeters();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Error leyendo el sensor DHT11");
  } else {
    char humidityStr[16];
    char temperatureStr[16];
    sprintf(humidityStr, "%.2f", humidity);
    sprintf(temperatureStr, "%.2f", temperature);
    client.publish("sensor/DHT11/humedad", humidityStr);
    client.publish("sensor/DHT11/temperatura", temperatureStr);
    
    Serial.printf("Humedad: %.2f%%, Temperatura: %.2f°C\n", humidity, temperature);
  }

  client.publish("sensor/HC-SR04/uno", String(distanceCm).c_str());
  client.publish("sensor/HC-SR04/dos", String(distanceCm2).c_str());
  
  if (sensor.timeoutOccurred()) {
    Serial.println("Error de tiempo de espera del sensor VL53L0X");
  } else {
    char distanciaVL53L0XStr[16];
    sprintf(distanciaVL53L0XStr, "%d", distanciaVL53L0X);
    client.publish("sensor/VL53L0X/distancia", distanciaVL53L0XStr);
    
    Serial.printf("Distancia VL53L0X: %d mm\n", distanciaVL53L0X);
  }

  delay(1000);  
}

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  return duration * SOUND_SPEED / 2;
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conectar a MQTT...");
    if (client.connect("ArduinoClient")) {
      Serial.println("conectado");
    } else {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intentamos de nuevo en 5 segundos");
      delay(5000);
    }
  }
}