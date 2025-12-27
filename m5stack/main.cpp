#include <M5Unified.h>
#include "Config.h"
#include "BleMonitor.h"
#include "UserInterface.h"

BeaconData currentDisplay;

void setup(){
    auto cfg = M5.config();
    M5.begin(cfg);

    Serial.begin(9600);  // Add this for debugging
    delay(100);
    // Direct test without canvas
    M5.Display.setRotation(1);
    M5.Display.setBrightness(255);
    M5.Display.fillScreen(TFT_RED);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(3);
    M5.Display.setCursor(50, 100);
    M5.Display.println("HARDWARE OK");
    Serial.println("M5Stack Starting...");
    delay(2000);

    initUI();
    // Initialize currentDisplay explicitly
    currentDisplay.active = false;
    currentDisplay.isDanger = false;
    currentDisplay.rssi = -100;
    currentDisplay.mask = 0;
    currentDisplay.lastPacketTime = millis();
    initBLE();

    updateUI(currentDisplay);
}

void loop() {
    M5.update();
    uint32_t now = millis();
    
    // Timeout check
    if (currentDisplay.active && (now - latestData.lastPacketTime > SIGNAL_TIMEOUT)) {
        portENTER_CRITICAL(&stateMux);
        latestData.active = false;
        latestData.isDanger = false;
        latestData.rssi = -100;
        portEXIT_CRITICAL(&stateMux);
        
        currentDisplay = latestData;
        Serial.println("No connection!");
        if((now - latestData.lastPacketTime > SIGNAL_TIMEOUT)){
            Serial.println("Time out!");
        }
        updateUI(currentDisplay);
    }
    
    // New data? Just update! (BLE callback already checked if it changed)
    if (newDataAvailable.exchange(false)) {
        portENTER_CRITICAL(&stateMux);
        currentDisplay = latestData;
        portEXIT_CRITICAL(&stateMux);
        Serial.println("State changed");
        updateUI(currentDisplay);
    }
    
    delay(20);
}