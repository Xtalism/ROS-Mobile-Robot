#include <WiFi.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>

const int firstMotorOne = 16;  // forward
const int firstMotorTwo = 17;  // backward
const int speedMotorOne = 5;   // enable

const int secondMotorOne = 0;  
const int secondMotorTwo = 4;  
const int speedMotorTwo = 2;   

const int thirdMotorOne = 14;  
const int thirdMotorTwo = 12;  
const int speedMotorThree = 13;

const int fourthMotorOne = 26;  
const int fourthMotorTwo = 27;  
const int speedMotorFour = 25;

int motorOnePins[] = {firstMotorOne, secondMotorOne, thirdMotorOne, fourthMotorOne};
int motorTwoPins[] = {firstMotorTwo, secondMotorTwo, thirdMotorTwo, fourthMotorTwo};
int motorSpeedPins[] = {speedMotorOne, speedMotorTwo, speedMotorThree, speedMotorFour};

const char* ssid = "MotherBase"; 
const char* password = "@6830135@"; 

IPAddress server(192, 168, 100, 156); 
const uint16_t serverPort = 11411; 

WiFiClient client;
ros::NodeHandle nh; 

void stopMotors(int motorOnePins[], int motorTwoPins[], int motorSpeedPins[]) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(motorOnePins[i], LOW);
        digitalWrite(motorTwoPins[i], LOW);
        digitalWrite(motorSpeedPins[i], 0);
    }
}

void controlMotor(int motorOnePins[], int motorTwoPins[], int motorSpeedPins[], float stickY, float RT, float LT) {
    const int stickValuePositive = 1;
    const int stickValueNegative = -1;

    int motorState = (stickY > 0.1) ? stickValuePositive : (stickY < -0.1) ? stickValueNegative : 0;

    for (int i = 0; i < 4; i++) {
        switch (motorState) {
            case stickValuePositive:
                digitalWrite(motorOnePins[i], HIGH);
                digitalWrite(motorTwoPins[i], LOW);
                analogWrite(motorSpeedPins[i], (RT * -255) + (LT * 255)); // break and speed
                break;
            case stickValueNegative:
                digitalWrite(motorOnePins[i], LOW);
                digitalWrite(motorTwoPins[i], HIGH);
                analogWrite(motorSpeedPins[i], (RT * -255) + (LT * 255)); // break and speed
                break;
            default:
                stopMotors(motorOnePins, motorTwoPins, motorSpeedPins);
                break;
        }
    }
}

void joyCallback(const sensor_msgs::Joy &joy_msg) {
    float leftStickY = joy_msg.axes[1];
    float rightStickY = joy_msg.axes[4];

    float leftStickX = joy_msg.axes[0]; 
    float rightStickX = joy_msg.axes[3];

    float LT =  joy_msg.axes[2];
    float RT = joy_msg.axes[5];

    controlMotor(motorOnePins, motorTwoPins, motorSpeedPins, leftStickY, RT, LT);
}

ros::Subscriber<sensor_msgs::Joy> sub("/joy", joyCallback);

void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(motorOnePins[i], OUTPUT);
        pinMode(motorTwoPins[i], OUTPUT);
        pinMode(motorSpeedPins[i], OUTPUT);
    }

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
    
    stopMotors(motorOnePins, motorTwoPins, motorSpeedPins);
}

void loop() {
    nh.spinOnce();
    delay(10);
}