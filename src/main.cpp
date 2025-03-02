#include <U8g2lib.h>
#include "motor_and_buzzer_functions.h"

U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE work!");
    
    OLEDsetup(); // Setting up OLED display

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);  
    digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off at the start

    BLEsetup(); // Setting up BLE
    setupMotorPins(); // Setting up motor pins
}

void loop() {
    handleLoopOperations();  // Handling loop operations defined in the "motor_and_buzzer_functions.cpp" file
}