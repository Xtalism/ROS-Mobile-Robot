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

IPAddress server(192, 168, 100, 156); 
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

void controlMotorForwardBackward(int motorOnePin, int motorTwoPin, int motorSpeedPin, float stickY, float stickX, float RT, float LT) {
    const int stickForward = 1;
    const int stickBackward = -1;

    const int stickLeft = 1;
    const int stickRight = -1;

    int motorStateFB = (stickY > 0.1) ? stickForward : (stickY < -0.1) ? stickBackward : 0;
    int motorStateLR = (stickX > 0.1) ? stickLeft : (stickX < -0.1) ? stickRight : 0;

    switch (motorStateFB) {
        case stickForward:
            digitalWrite(motorOnePin, HIGH);
            digitalWrite(motorTwoPin, LOW);
            analogWrite(motorSpeedPin, (RT * -128) + (LT * 128)); // break and speed
            break;
        case stickBackward:
            digitalWrite(motorOnePin, LOW);
            digitalWrite(motorTwoPin, HIGH);
            analogWrite(motorSpeedPin, (RT * -128) + (LT * 128)); // break and speed
            break;
        default:
            digitalWrite(motorOnePin, LOW);
            digitalWrite(motorTwoPin, LOW);
            analogWrite(motorSpeedPin, 0);
            break;
    }

    switch (motorStateLR) {
        case stickLeft:
            digitalWrite(motorOnePin, LOW);
            digitalWrite(motorTwoPin , HIGH);
            analogWrite(motorSpeedPin, (RT * -128) + (LT * 128)); // break and speed
            break;
        case stickRight:
            digitalWrite(motorOnePin, HIGH);
            digitalWrite(motorTwoPin, LOW);
            analogWrite(motorSpeedPin, (RT * -128) + (LT * 128)); // break and speed
            break;
        default:
            digitalWrite(motorOnePin, LOW);
            digitalWrite(motorTwoPin, LOW);
            analogWrite(motorSpeedPin, 0);
            break;
    }
}

void joyCallback(const sensor_msgs::Joy &joy_msg) {
    float leftStickY = joy_msg.axes[4];
    float rightStickY = joy_msg.axes[3];
    
    float rightStickX = joy_msg.axes[0];
    float leftStickX = joy_msg.axes[1];

    float RT = joy_msg.axes[5];
    float LT =  joy_msg.axes[2];

    controlMotorForwardBackward(firstMotorOne, firstMotorTwo, speedMotorOne, leftStickY, leftStickX, RT, LT);
    controlMotorForwardBackward(secondMotorOne, secondMotorTwo, speedMotorTwo, rightStickY, rightStickY, RT, LT);
    controlMotorForwardBackward(thirdMotorOne, thirdMotorTwo, speedMotorThree, rightStickY, rightStickX, RT, LT);
    controlMotorForwardBackward(fourthMotorOne, fourthMotorTwo, speedMotorFour, leftStickY, leftStickY, RT, LT);
}

ros::Subscriber<sensor_msgs::Joy> sub("/joy", joyCallback);

void setup() {
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