#ifndef MOTOR_AND_BUZZER_FUNCTIONS_H
#define MOTOR_AND_BUZZER_FUNCTIONS_H
#include <string>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <U8g2lib.h>


// Motor 1 control pins
#define ENA_PIN 19
#define IN1_PIN 17
#define IN2_PIN 16

#define BUZZER_PIN 14  // Define the buzzer pin

// Ultrasound Sensor pins
#define TRIG_PIN 32
#define ECHO_PIN 33

// Motor 2 control pins
#define ENB_PIN 21
#define IN3_PIN 22
#define IN4_PIN 2

//  SH1106 SPI JK-130-12864-B OLED setup using SPI
#define OLED_SCK 18   // Connect to SPI SCK of OLED out
#define OLED_MOSI 23  // Connect to SDA of OLED out
#define OLED_RST 5   // Connect to RST of OLED out
#define OLED_DC  12   // Connect to DC of OLED out
#define OLED_CS  15   // Connect to CS of OLED out

#define LINE_HEIGHT 10  // Height of a line on the OLED
#define MAX_LINES 5     // Assuming the OLED can display 5 lines. Adjust as needed.
#define MESSAGE_LENGTH 20

void displayOnOLED(const char* message);
void forwardMotorA();
void backwardMotorA();
void stopMotorA();
void forwardMotorB();
void backwardMotorB();
void stopMotorB();
void beepOnce();
void beepTwice();
float getDistance();
void distanceBuzzerAlert();
void setupMotorPins();
void BLEsetup();
void OLEDsetup();
void updateFirmware(const std::string& rxValue);


void handleBLECommand(const std::string &command);
void handleLoopOperations();

#endif
