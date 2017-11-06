
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

// Vision Pins
#define VISION_SS 19

////////////////////////////////////////////////////////////////////
// General Igel specific Configuraiton (Strategy)
//
#define SONAR_MAX_DISTANCE 200
#define MIN_OBJECT_DISTANCE_CM 10
#define TARGET_DEVIATION_TOLARANCE 100
#define TARGET_NAV_TOLARANCE 5
#define TARGET_DISTANCE_TOLARANCE 190
#define TARGET_CONTROL_P 1.59 // Linear transformation from deviation in pixesl (max=160) to servo frequency (max=255)
