#pragma once
#include <Arduino.h>

#define MY_TRACK_ID     1
#define MANUFACTURER_ID 0xCCCC
#define BEACON_NAME     "DANGER"

// timing
static const uint32_t SIGNAL_TIMEOUT = 10000;
static const uint32_t SCAN_INTERVAL = 100;
static const uint32_t SCAN_WINDOW = 100;

struct BeaconData {
    bool isDanger = false;
    uint8_t mask = 0;
    int rssi = -100;
    bool active = false;
    uint32_t lastPacketTime = 0;
};