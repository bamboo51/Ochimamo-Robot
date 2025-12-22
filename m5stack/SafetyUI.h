#ifndef SAFETY_UI_H
#define SAFETY_UI_H

#include <M5Unified.h>

class SafetyUI {
private:
    M5Canvas canvas;

    // Map Config
    static const int MAP_W = 8;
    static const int MAP_H = 8;
    static const int MAP_PIXELS = 200;
    static const int MAP_OFFSET_X = 10;
    static const int MAP_OFFSET_Y = 20;
    
    // Danger Map Data (0: Safe, 1: Danger)
    // Simple 8x8 fixed map
    const uint8_t dangerMap[8][8] = {
      {0,0,0,0,0,0,0,1},
      {0,0,0,0,0,0,0,1},
      {0,0,0,0,0,0,0,1},
      {0,0,0,0,0,0,0,1},
      {0,0,0,0,0,0,0,1},
      {0,0,0,0,0,0,0,1},
      {0,0,0,0,0,0,0,1},
      {1,1,1,1,1,1,1,1} 
    };

    int CELL_PX;

public:
    SafetyUI() : canvas(&M5.Display) {
        CELL_PX = MAP_PIXELS / MAP_W;
    }

    void init() {
        canvas.createSprite(320, 240);
        canvas.setTextSize(2);
    }

    bool isDanger(int gx, int gy) {
        if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return true;
        int mapY = MAP_H - 1 - gy;
        return dangerMap[mapY][gx] == 1;
    }

    void drawWallAlert() {
        // Red Background
        canvas.fillScreen(TFT_RED);
        canvas.setTextColor(TFT_YELLOW);
        canvas.setTextSize(3);
        canvas.setCursor(20, 80);
        canvas.print("WALL ALERT");
        
        // Push to hardware
        canvas.pushSprite(0, 0);
        M5.Speaker.tone(6000, 100);
    }

    void drawZoneAlert() {
        canvas.fillScreen(TFT_RED);
        canvas.setTextColor(TFT_WHITE);
        canvas.setTextSize(3);
        canvas.setCursor(20, 80);
        canvas.print("ZONE ALERT");
        
        canvas.pushSprite(0, 0);
        M5.Speaker.tone(4000, 100);
    }

    void drawPosition(float x, float y, int id, bool hasPos) {
        canvas.fillScreen(TFT_BLACK);

        if (!hasPos) {
            canvas.setTextColor(TFT_YELLOW);
            canvas.setCursor(10, 50);
            canvas.setTextSize(2);
            canvas.println("Searching Anchors...");
            canvas.pushSprite(0, 0);
            return;
        }

        // Draw Map Grid
        for (int y0=0; y0<MAP_H; y0++) {
            for (int x0=0; x0<MAP_W; x0++) {
                int px = MAP_OFFSET_X + x0 * CELL_PX;
                int py = MAP_OFFSET_Y + y0 * CELL_PX;
                
                if (dangerMap[y0][x0]) {
                    canvas.fillRect(px, py, CELL_PX, CELL_PX, TFT_MAROON);
                } else {
                    canvas.drawRect(px, py, CELL_PX, CELL_PX, TFT_DARKGREY);
                }
            }
        }

        // Draw Player
        float cellSize = 0.5;
        int gx = x / cellSize;
        int gy = y / cellSize;
        
        int mapY = MAP_H - 1 - gy; 
        
        int px = MAP_OFFSET_X + gx * CELL_PX + CELL_PX/2;
        int py = MAP_OFFSET_Y + mapY * CELL_PX + CELL_PX/2;
        
        canvas.fillCircle(px, py, 6, TFT_CYAN);

        // Text Info
        canvas.setTextColor(TFT_WHITE);
        canvas.setTextSize(1);
        canvas.setCursor(220, 10);
        canvas.printf("ID: %d", id);
        
        canvas.setCursor(220, 30);
        canvas.printf("X: %.2f", x);
        canvas.setCursor(220, 50);
        canvas.printf("Y: %.2f", y);
        
        // Push Frame
        canvas.pushSprite(0, 0);
    }
};

#endif
