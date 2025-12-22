#include <M5Unified.h>
#include <NimBLEDevice.h>
#include "SafetyUI.h"
#include "Positioning.h"

// --- CONFIGURATION ---
#define MY_TRACK_ID 1
#define SERVICE_UUID        "0000abcd-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00001234-0000-1000-8000-00805f9b34fb"
const unsigned long RASPI_TIMEOUT = 3000;

// --- OBJECTS ---
NimBLEScan* pBLEScan;
SafetyUI ui;
Positioning pos;

// --- STATE ---
bool raspiDanger = false;
unsigned long lastRaspiSeen = 0;

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  
  // Init UI (Sprite)
  ui.init();
  
  // Init BLE
  NimBLEDevice::init("WORKER_M5");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(50); // Faster Interval
  pBLEScan->setWindow(50);   // Faster Window
  
  // M5.Display.setTextSize(2); // Handled by SafetyUI now
}

void loop() {
  M5.update();
  
  // 1. **High Speed Scan** (0.2s)
  // This reduces latency significantly.
  pBLEScan->clearResults(); 
  NimBLEScanResults foundDevices = pBLEScan->start(0.2, false);
  
  for(int i=0; i<foundDevices.getCount(); i++) {
    NimBLEAdvertisedDevice d = foundDevices.getDevice(i);
    std::string addr = d.getAddress().toString();
    
    // A. Check Anchors (Partial Update)
    int anchorIdx = pos.getAnchorIndex(addr);
    if (anchorIdx != -1) {
        pos.updateSingle(anchorIdx, d.getRSSI());
    }
    
    // B. Check RasPi (Wall Warning)
    if (d.isAdvertisingService(NimBLEUUID(SERVICE_UUID))) {
        // Connect & Check
        NimBLEClient* pClient = NimBLEDevice::createClient();
        if (pClient->connect(&d)) {
            NimBLERemoteService* pSvc = pClient->getService(SERVICE_UUID);
            if (pSvc) {
                NimBLERemoteCharacteristic* pChr = pSvc->getCharacteristic(CHARACTERISTIC_UUID);
                if (pChr) {
                    std::string val = pChr->readValue();
                    if (val.length() > 0) {
                        int mask = (int)val[0];
                        if (mask & (1 << (MY_TRACK_ID - 1))) {
                            raspiDanger = true;
                            lastRaspiSeen = millis();
                        } else {
                            raspiDanger = false; 
                        }
                    }
                }
            }
            pClient->disconnect();
        }
        NimBLEDevice::deleteClient(pClient);
    }
  }

  // 2. Process Logic
  if (millis() - lastRaspiSeen > RASPI_TIMEOUT) {
      raspiDanger = false;
  }
  
  // Update Position Math
  pos.compute();
  
  bool positionDanger = false;
  if (pos.hasPosition()) {
      positionDanger = ui.isDanger((int)(pos.getX()/0.5), (int)(pos.getY()/0.5));
  }

  // 3. Update UI (Flicker Free)
  if (raspiDanger) {
      ui.drawWallAlert();
  } else if (positionDanger) {
      ui.drawZoneAlert();
  } else {
      ui.drawPosition(pos.getX(), pos.getY(), MY_TRACK_ID, pos.hasPosition());
  }
}