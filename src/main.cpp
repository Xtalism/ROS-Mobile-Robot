#include <WiFi.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>

const int firstMotorOne = 32; 
const int firstMotorTwo = 33; 
const int speedMotorOne = 23;

const int secondMotorOne = 4;
const int secondMotorTwo = 2; 
const int speedMotorTwo = 15; 

const char* ssid = "SSID";
const char* password = "PASSWORD";

IPAddress server(192, 168, 0, 0);
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

void controlMotor(int motorOnePin, int motorTwoPin, int motorSpeedPin, float stickY) {
    const int stickForward = 1;
    const int stickBackward = -1;

    int motorState = (stickY > 0.1) ? stickForward : (stickY < -0.1) ? stickBackward : 0;

    switch (motorState) {
        case stickForward:
            digitalWrite(motorOnePin, HIGH);
            digitalWrite(motorTwoPin, LOW);
            analogWrite(motorSpeedPin, abs(stickY * 255));
            break;
        case stickBackward:
            digitalWrite(motorOnePin, LOW);
            digitalWrite(motorTwoPin, HIGH);
            analogWrite(motorSpeedPin, abs(stickY * 255));
            break;
        default:
            digitalWrite(motorOnePin, LOW);
            digitalWrite(motorTwoPin, LOW);
            analogWrite(motorSpeedPin, 0);
            break;
    }
}

void joyCallback(const sensor_msgs::Joy &joy_msg) {
    float leftStickY = joy_msg.axes[4];  // vertical 
    float rightStickY = joy_msg.axes[1]; // vertical

    controlMotor(firstMotorOne, firstMotorTwo, speedMotorOne, leftStickY);
    controlMotor(secondMotorOne, secondMotorTwo, speedMotorTwo, rightStickY);
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
