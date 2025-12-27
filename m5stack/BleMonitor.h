#pragma once
#include "Config.h"
#include <BLEDevice.h>
#include <atomic>

static std::atomic<bool> newDataAvailable;
static BeaconData latestData;
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

class ScanCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) override {
        if(!dev.haveManufacturerData()) return;
        const std::string &data = dev.getManufacturerData();
        
        if(data.length()<3) return;
        
        uint16_t mfgId = ((uint8_t)data[0]) | (((uint8_t)data[1])<<8);
        if(mfgId != MANUFACTURER_ID) return;
        
        if(dev.haveName()){
            std::string name = dev.getName();
            if(name != BEACON_NAME) return;
        }
        
        if(MY_TRACK_ID < 1 || MY_TRACK_ID > 8){
            Serial.println("ERROR: Invalid MY_TRACK_ID (must be 1-8)");
            return;
        }
        
        uint8_t mask = (uint8_t)data[2];
        bool newDanger = (mask & (1<<(MY_TRACK_ID-1))) != 0;
        int newRSSI = dev.getRSSI();
        
        bool dataChanged = false;
        
        portENTER_CRITICAL(&stateMux);
        if (latestData.isDanger != newDanger || 
            latestData.mask != mask || 
            latestData.active != true) {  // Only update if RSSI changed significantly
            
            latestData.isDanger = newDanger;
            latestData.rssi = newRSSI;
            latestData.mask = mask;
            latestData.active = true;
            dataChanged = true;
        }
        latestData.lastPacketTime = millis();  // Always update timestamp
        
        if (dataChanged) {
            newDataAvailable.store(true);
        }
        portEXIT_CRITICAL(&stateMux);
        
        if (dataChanged) {
            Serial.printf("BLE RX: RSSI=%d, Mask=0x%02X, MyBit=%d, Danger=%s\n",
                newRSSI, mask, MY_TRACK_ID, newDanger ? "YES" : "NO");
        }
    }
};

void initBLE(){
    BLEDevice::init("M5 Monitor");
    BLEScan *scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
    scan->setActiveScan(true);
    scan->setInterval(SCAN_INTERVAL);
    scan->setWindow(SCAN_WINDOW);
    scan->start(0, nullptr, false);

    Serial.println("BLE scan started");
    Serial.println("Listening for beacon");
}