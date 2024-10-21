#include <WiFi.h>
#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>

int motorOnePins[] = {16, 0, 14, 26};
int motorTwoPins[] = {17, 4, 12, 27};
int motorSpeedPins[] = {5, 2, 13, 25};

const char* ssid = ""; 
const char* password = ""; 

IPAddress server(192, 168, 100, 213); 
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

void controlMotor(int motorOnePins[], int motorTwoPins[], int motorSpeedPins[], float stickY, float RT, float LT, float angle) {
    const int stickValuePositive = 1;
    const int stickValueNegative = -1;

    if (angle < 0) {
        angle += 360;
    }

    int nearestMultipleOf45 = round(angle / 45.0) * 45.0;

    Serial.println(nearestMultipleOf45);

    for (int i = 0; i < 4; i++) {
        switch (nearestMultipleOf45) {
            case -0:
            case 360: // left
                stopMotors(motorOnePins, motorTwoPins, motorSpeedPins);
                break;
            case 45: // top-left
                digitalWrite(motorOnePins[0], HIGH);
                digitalWrite(motorTwoPins[0], LOW);
                digitalWrite(motorOnePins[2], HIGH);
                digitalWrite(motorTwoPins[2], LOW);
                analogWrite(motorSpeedPins[i], (RT * -255) + (LT * 255));
                break;
            case 90: // top
                digitalWrite(motorOnePins[i], HIGH);
                digitalWrite(motorTwoPins[i], LOW);
                analogWrite(motorSpeedPins[i], (RT * -255) + (LT * 255));
                break;
            case 135: // top-right
                digitalWrite(motorOnePins[1], HIGH);
                digitalWrite(motorTwoPins[1], LOW);
                digitalWrite(motorOnePins[3], HIGH);
                digitalWrite(motorTwoPins[3], LOW);
                analogWrite(motorSpeedPins[i], (RT * -255) + (LT * 255));
                break;
            case 180: // right
                stopMotors(motorOnePins, motorTwoPins, motorSpeedPins);
                break;
            case 225: // bottom-right
                digitalWrite(motorOnePins[0], LOW);
                digitalWrite(motorTwoPins[0], HIGH);
                digitalWrite(motorOnePins[2], LOW);
                digitalWrite(motorTwoPins[2], HIGH);
                analogWrite(motorSpeedPins[i], (RT * -255) + (LT * 255));
                break;
            case 270: // bottom
                digitalWrite(motorOnePins[i], LOW);
                digitalWrite(motorTwoPins[i], HIGH);
                analogWrite(motorSpeedPins[i], (RT * -255) + (LT * 255));
                break;
            case 315: // bottom-left
                digitalWrite(motorOnePins[1], LOW);
                digitalWrite(motorTwoPins[1], HIGH);
                digitalWrite(motorOnePins[3], LOW);
                digitalWrite(motorTwoPins[3], HIGH);
                analogWrite(motorSpeedPins[i], (RT * -255) + (LT * 255));
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

    float angle = atan2(leftStickY, leftStickX) * 180.0 / PI;

    controlMotor(motorOnePins, motorTwoPins, motorSpeedPins, leftStickY, RT, LT, angle);
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