#include <WiFi.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>

const int firstMotorOne = 17;  // forward
const int firstMotorTwo = 16;  // backward
const int speedMotorOne = 5;   // enable

const int secondMotorOne = 4;  // forward
const int secondMotorTwo = 0;  // backward
const int speedMotorTwo = 2;   // enable

const int fourthMotorOne = 27;  // forward
const int fourthMotorTwo = 26;  // backward
const int speedMotorFour = 25; // enable

const int thirdMotorOne = 12;  // forward
const int thirdMotorTwo = 14;  // backward
const int speedMotorThree = 13;  // enable

const char* ssid = "MotherBase"; 
const char* password = "@6830135@"; 

IPAddress server(192, 168, 100, 213); 
const uint16_t serverPort = 11411; 

WiFiClient client;
ros::NodeHandle nh; 

// Motor stop functions
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

void stopThirdMotor() {
    digitalWrite(thirdMotorOne, LOW);
    digitalWrite(thirdMotorTwo, LOW);
    analogWrite(speedMotorThree, 0);
}

void stopFourthMotor() {
    digitalWrite(fourthMotorOne, LOW);
    digitalWrite(fourthMotorTwo, LOW);
    analogWrite(speedMotorFour, 0);
}

void controlMotorForwardBackward(int motorOnePin, int motorTwoPin, int motorSpeedPin, float stickY) {
    const int stickForward = 1;
    const int stickBackward = -1;

    int motorStateFB = (stickY > 0.1) ? stickForward : (stickY < -0.1) ? stickBackward : 0;

    switch (motorStateFB) {
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
    float leftStickY = joy_msg.axes[4];  // vertical left y
    float rightStickY = joy_msg.axes[1]; // vertical right y

    controlMotorForwardBackward(firstMotorOne, firstMotorTwo, speedMotorOne, leftStickY);
    controlMotorForwardBackward(secondMotorOne, secondMotorTwo, speedMotorTwo, rightStickY);
    controlMotorForwardBackward(thirdMotorOne, thirdMotorTwo, speedMotorThree, rightStickY);
    controlMotorForwardBackward(fourthMotorOne, fourthMotorTwo, speedMotorFour, leftStickY);
}

ros::Subscriber<sensor_msgs::Joy> sub("/joy", joyCallback);

void setup() {
    // Initialize motor pins
    pinMode(firstMotorOne, OUTPUT);
    pinMode(firstMotorTwo, OUTPUT);
    pinMode(speedMotorOne, OUTPUT);

    pinMode(secondMotorOne, OUTPUT);
    pinMode(secondMotorTwo, OUTPUT);
    pinMode(speedMotorTwo, OUTPUT);

    pinMode(thirdMotorOne, OUTPUT);
    pinMode(thirdMotorTwo, OUTPUT);
    pinMode(speedMotorThree, OUTPUT);

    pinMode(fourthMotorOne, OUTPUT);
    pinMode(fourthMotorTwo, OUTPUT);
    pinMode(speedMotorFour, OUTPUT);

    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");

    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.subscribe(sub);
    
    stopFirstMotor();
    stopSecondMotor();
    stopThirdMotor();
    stopFourthMotor();
}

void loop() {
    nh.spinOnce();
    delay(10);
}