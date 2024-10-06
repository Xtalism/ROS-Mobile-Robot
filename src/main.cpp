#include <WiFi.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>

const int firstMotorOne = 32; 
const int firstMotorTwo = 33; 
const int speedMotorOne = 23;

const int secondMotorOne = 4;
const int secondMotorTwo = 2; 
const int speedMotorTwo = 15; 

const char* ssid = "MotherBase";
const char* password = "@6830135@";

IPAddress server(192, 168, 100, 213);  // 192.168.100.213
const uint16_t serverPort = 11411; 

WiFiClient client;
ros::NodeHandle nh; 

void stopFirstMotor() {
    digitalWrite(firstMotorOne, LOW);
    digitalWrite(firstMotorTwo, LOW);
    analogWrite(speedMotorOne, 0);
}

void stopSecondMotor() {
    digitalWrite(secondMotorOne, LOW);
    digitalWrite(secondMotorTwo, LOW);
    analogWrite(speedMotorTwo, 0);
}

void joyCallback(const sensor_msgs::Joy &joy_msg) {
    float leftStickX = joy_msg.axes[0];  // horizontal
    float leftStickY = joy_msg.axes[4];  // vertical

    float rightStickX = joy_msg.axes[3]; // horizontal
    float rightStickY = joy_msg.axes[1]; // vertical

    const int leftStickForward = 1;
    const int leftStickBackward = -1;

    const int rightStickForward = 1; 
    const int rightStickBackward = -1;

    int firstMotorState = (leftStickY > 0.1) ? leftStickForward : (leftStickY < -0.1) ? leftStickBackward : 0;

    int secondMotorState = (rightStickY > 0.1) ? rightStickForward : (rightStickY < -0.1) ? rightStickBackward : 0;
    
    switch (firstMotorState) {
        case leftStickForward: 
            digitalWrite(firstMotorOne, HIGH);
            digitalWrite(firstMotorTwo, LOW);
            analogWrite(speedMotorOne, abs(leftStickY * 255));
            break;
        case leftStickBackward:
            digitalWrite(firstMotorOne, LOW);
            digitalWrite(firstMotorTwo, HIGH);
            analogWrite(speedMotorOne, abs(leftStickY * 255));
            break;
        default: 
            stopFirstMotor();
            break;
    }

    switch (secondMotorState) {
        case rightStickForward:
            digitalWrite(secondMotorOne, HIGH);
            digitalWrite(secondMotorTwo, LOW);
            analogWrite(speedMotorTwo, abs(rightStickY * 255));
            break;
        case rightStickBackward:
            digitalWrite(secondMotorOne, LOW);
            digitalWrite(secondMotorTwo, HIGH);
            analogWrite(speedMotorTwo, abs(rightStickY * 255));
            break;
        default:
            stopSecondMotor(); 
            break;
    }
}

ros::Subscriber<sensor_msgs::Joy> sub("/joy", joyCallback);

void setup() {
    pinMode(firstMotorOne, OUTPUT);
    pinMode(firstMotorTwo, OUTPUT);
    pinMode(speedMotorOne, OUTPUT);

    pinMode(secondMotorOne, OUTPUT);
    pinMode(secondMotorTwo, OUTPUT);
    pinMode(speedMotorTwo, OUTPUT);

    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");

    nh.getHardware()->setConnection(server, serverPort); // TCP
    nh.initNode();
    nh.subscribe(sub);
    
    stopFirstMotor();
    stopSecondMotor();
}

void loop() {
    nh.spinOnce();
    delay(10);
}
