
////////////////////////////////////////////////////////////////////
// Define the Pins connected to the arduineo
//

/*
// Setup for the Arduino
#define MOTOR_RIGHT 1
#define MOTOR_LEFT 2
#define SERVO_STEER_C1 5
#define SERVO_STEER_C2 6
#define SONAR_TRIGGER_PIN 9 //yellow
#define SONAR_ECHO_PIN 8 // orange
#define SONAR_MAX_DISTANCE 200
#define PICKUPSYSTEM_PIN 4
*/

// Setup for the Feather
// Chassis Pins
#define SERVO_FRONT_RIGHT 0
#define SERVO_FRONT_LEFT 1
#define SERVO_BACK_RIGHT 6
#define SERVO_BACK_LEFT 7
#define SERVO_BACKBONE 3

// Sonar Pins
#define SONAR_TRIGGER_PIN 10 //yellow
#define SONAR_ECHO_PIN 9 // orange

// Pickup Pins
#define PICKUPSYSTEM_PIN 13

// Communicaiton Pins
#define COMMUNICAION_CS 8
#define COMMUNICAION_IRQ 7
#define COMMUNICAION_RST 4
#define COMMUNICAION_EN 2
#define LISTEN_PORT 80

// Vision Pins
#define VISION_I2C_SCL 21
#define VISION_I2C_SDA 20

// WiFi setPins
#define WIFI_CS 8
#define WIFI_IRQ 7
#define WIFI_RST 4
#define WIFI_EN 2
#define WIFI_LISTEN_PORT 80

////////////////////////////////////////////////////////////////////
// General Igel specific Configuraiton (Strategy)
//
#define SONAR_MAX_DISTANCE 1000
#define MIN_OBJECT_DISTANCE_CM 7
// Target Dectcion
#define TARGET_DEVIATION_TOLARANCE 200
#define TARGET_DISTANCE_TOLARANCE 1000
// Naviagion Controll
#define TARGET_NAV_TOLARANCE 10
#define TARGET_CONTROL_P 1.59 // Linear transformation from deviation in pixesl (max=160) to servo frequency (max=255)
