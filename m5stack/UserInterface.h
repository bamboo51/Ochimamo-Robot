#pragma once
#include <M5Unified.h>
#include "Config.h"

static void initUI(){
    M5.Display.setRotation(1);
    M5.Display.setBrightness(255);
    M5.Display.fillScreen(TFT_BLACK);
    Serial.println("Display initialized");
}

static void updateUI(const BeaconData &data){
    int centerX = M5.Display.width() / 2;
    int centerY = M5.Display.height() / 2;
    
    // 1. Background color
    if(!data.active){
        M5.Display.fillScreen(TFT_DARKGRAY);
    } else if (data.isDanger){
        M5.Display.fillScreen(TFT_RED);
    } else {
        M5.Display.fillScreen(TFT_GREEN);
    }
    
    // 2. Main message (center)
    M5.Display.setTextDatum(middle_center);
    if(!data.active){
        M5.Display.setTextColor(TFT_ORANGE, TFT_DARKGRAY);
        M5.Display.setTextSize(3);
        M5.Display.drawString("WAITING...", centerX, centerY);
    } else if(data.isDanger){
        M5.Display.setTextColor(TFT_WHITE, TFT_RED);
        M5.Display.setTextSize(5);
        M5.Display.drawString("DANGER", centerX, centerY);
    } else {
        M5.Display.setTextColor(TFT_BLACK, TFT_GREEN);
        M5.Display.setTextSize(5);
        M5.Display.drawString("SAFE", centerX, centerY);
    }
    
    // 3. Info box (top-left)
    M5.Display.fillRect(0, 0, 140, 45, TFT_BLACK);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(5, 5);
    M5.Display.printf("Track ID: %d", MY_TRACK_ID);
    M5.Display.setCursor(5, 18);
    M5.Display.printf("RSSI: %d dBm", data.rssi);
    M5.Display.setCursor(5, 31);
    if (data.active) M5.Display.printf("Mask: 0x%02X", data.mask);
    else M5.Display.print("Mask: --");
    
    Serial.println("UI updated");
}