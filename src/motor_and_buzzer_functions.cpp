#include <Arduino.h>
#include <Update.h>
#include <U8g2lib.h>
#include "motor_and_buzzer_functions.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
int receivedChecksum = 0; // To store the received checksum
int calculatedChecksum = 0; // To store the calculated checksum
bool isReceiving = false; 

extern U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2;  // Declare the OLED object as external as it's assumed to be defined elsewhere

static int currentLine = 0;  // Current line on the OLED
static char messages[MAX_LINES][MESSAGE_LENGTH];


class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();

        Serial.print("Received rxValue: ");
        Serial.println(rxValue.c_str());
        displayOnOLED(rxValue.c_str());
        handleBLECommand(rxValue);  // Handle the BLE command

        if (rxValue == "START OTA UPDATE") {
          Serial.println("Received Start OTA Update command.");
          displayOnOLED("OTA START");
          isReceiving = true; // Set the flag to true to start receiving packets
          return;
        }

        if(isReceiving) updateFirmware(rxValue);
    }
};


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("BLE connected");
        beepOnce();
        displayOnOLED("BLE connected");
    }

    void onDisconnect(BLEServer* pServer) {
        Serial.println("BLE disconnected");
        beepTwice(); 
        displayOnOLED("BLE disconnected");
        BLEDevice::startAdvertising(); // start advertising when client disconnects
    }
};


 void updateFirmware(const std::string& rxValue) {
    if(rxValue == "END_OF_TRANSMISSION") {
      Serial.println("Received End of Transmission Signal.");
      isReceiving = false;
      displayOnOLED("Firmware Download Done!");

      if(receivedChecksum != calculatedChecksum) {
        Serial.println("Checksum mismatch. Update aborted.");
        Update.abort();
      } else {
        Serial.println("Checksum matched!! Update running..");
        displayOnOLED("Checksum Matched");
        
        if(Update.end(true)) {
          displayOnOLED("Firmware Update Done");
          Serial.println("Update Success: Rebooting...");
          ESP.restart();
        } else {
          Serial.println("Update Error Occurred: Error #: " + String(Update.getError()));
        }
      }

      
    } else if(rxValue.substr(0,8) == "CHECKSUM") {
      receivedChecksum = std::stoi(rxValue.substr(9));
      Serial.println("Received checksum: " + String(receivedChecksum));


      
    } else {
      if(!Update.isRunning() && !Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Serial.println("Not enough space to begin OTA");
        Update.abort();
        return;
      }

      for(const auto& c: rxValue) calculatedChecksum += (unsigned char)c;

      if(Update.write((uint8_t*)&rxValue[0], rxValue.length()) != rxValue.length()) {
        Serial.println("Write failed");
        Update.abort();
      } else {
        Serial.println("More data needed.");
      }
    }
    
}



void displayOnOLED(const char* message) {
    if (currentLine >= MAX_LINES) {
        currentLine = 0;
    }

    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_ncenB08_tr);
        // Display previous messages
        for (int i = 0; i < currentLine; i++) {
            u8g2.drawStr(0, i * LINE_HEIGHT + LINE_HEIGHT, messages[i]);
        }
        // Display the new message
        u8g2.drawStr(0, currentLine * LINE_HEIGHT + LINE_HEIGHT, message);
    } while (u8g2.nextPage());

    // Store the new message in the array
    strncpy(messages[currentLine], message, sizeof(messages[currentLine]) - 1);
    currentLine++;
}

void OLEDsetup() {
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.firstPage();
    do {
        u8g2.drawStr(0, 30, "Hello SAMSUNG");
    } while (u8g2.nextPage());
}


void BLEsetup() {
    Serial.println("Starting BLE work!");
    BLEDevice::init("ESP32_BLE");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); // Set server callbacks

    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

    pCharacteristic->setCallbacks(new MyCallbacks());
  
    pCharacteristic->setValue("Hello World, this is JY's BLE");
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);

    BLEAdvertisementData advertisementData = BLEAdvertisementData();
    advertisementData.setName("ESP32_BLE");

    uint8_t payload[5] = { 0xFF, 0xFF, 0x33, 0x44, 0x55 };
    advertisementData.setManufacturerData(std::string(reinterpret_cast<char*>(payload), 5));
    pAdvertising->setAdvertisementData(advertisementData);

    BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now you can read/write from your phone!");
}

void forwardMotorA() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, 255);
}

void backwardMotorA() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  analogWrite(ENA_PIN, 255);
}

void stopMotorA() {
  analogWrite(ENA_PIN, 0);
}

void forwardMotorB() {
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENB_PIN, 255);
}

void backwardMotorB() {
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  analogWrite(ENB_PIN, 255);
}

void stopMotorB() {
  analogWrite(ENB_PIN, 0);
}

void breakMotorA() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void breakMotorB() {
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}


void beepOnce() {
  int frequency = 2000;  // Frequency in Hz, adjust as needed for your buzzer
  int duration = 100;  // Duration in milliseconds

  // Generate a tone on BUZZER_PIN at the specified frequency and duration
  ledcSetup(0, frequency, 8);  // 8-bit resolution
  ledcAttachPin(BUZZER_PIN, 0);
  ledcWrite(0, 128);  // 50% duty cycle
  delay(duration);
  ledcWrite(0, 0);  // Stop the tone
}

void beepTwice() {
  beepOnce();
  delay(200);  // Delay between beeps
  beepOnce();
}

float getDistance() {
  // Clear the trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  // Send a 10 microsecond pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pulse duration in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);  // 25ms timeout

  // Calculate the distance (in centimeters) based on the speed of sound.
  float distance = (duration / 2.0) * 0.0343;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

void setupMotorPins() {
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
}

void handleBLECommand(const std::string &rxValue) {
    if (rxValue == "start") {
        forwardMotorA();
        forwardMotorB();
    } else if (rxValue == "m1f") {
        forwardMotorA();
        delay(1000);
        stopMotorA();
    } else if (rxValue == "m1b") {
        backwardMotorA();
        delay(1000);
        stopMotorA();
    } else if (rxValue == "m2f") {
        forwardMotorB();
        delay(1000);
        stopMotorB();
    } else if (rxValue == "m2b") {
        backwardMotorB();
        delay(1000);
        stopMotorB();
    } else if (rxValue == "slow") {
        analogWrite(ENA_PIN, 100);  // 10% speed
        analogWrite(ENB_PIN, 100);  // 10% speed
        delay(1000);
        stopMotorB();
        stopMotorA();
    } else if (rxValue == "fast") {
        analogWrite(ENA_PIN, 255);  // 100% speed
        analogWrite(ENB_PIN, 255);  // 100% speed
        delay(500);
        stopMotorB();
        stopMotorA();
    } else if (rxValue == "dist") {
        float distance = getDistance(); 
        char message[MESSAGE_LENGTH];
        snprintf(message, sizeof(message), "dist = %.2f cm", distance);  
        displayOnOLED(message);  
    }


    
}

void handleLoopOperations() {
    float distance = getDistance();

    if (distance >= 0 && distance < 10) {
        beepOnce();
        breakMotorA();
        breakMotorB();
        delay(50);  // Reduced this delay
    } else if (distance >= 10) {
        delay(20);
    }
}
