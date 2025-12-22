#ifndef POSITIONING_H
#define POSITIONING_H

#include <math.h>
#include <vector>
#include <string>

// Configuration
#define NUM_ANCHORS 3
#define AVG_WINDOW 5
// Signal Loss Timeout (if an anchor isn't seen for 5 seconds, ignore it)
#define ANCHOR_TIMEOUT 5000 

class Positioning {
private:
    const char* anchor_addrs[NUM_ANCHORS] = {
      "84:CC:A8:60:72:F6",
      "b8:f0:09:c5:1f:1a",
      "b8:f0:09:c5:1f:6e"
    };
    
    float x_anchor[NUM_ANCHORS] = {0.25, 3.25, 3.25};
    float y_anchor[NUM_ANCHORS] = {0.75, 0.75, 3.75};
    
    float txPower[NUM_ANCHORS] = {-65, -65, -65};
    float nValue[NUM_ANCHORS]  = {2.0, 2.0, 2.0};

    int rssiBuf[NUM_ANCHORS][AVG_WINDOW];
    int head[NUM_ANCHORS]; // Circular buffer head per anchor
    unsigned long lastUpdate[NUM_ANCHORS]; // Time last seen

    // Results
    float posX = 0;
    float posY = 0;
    bool valid = false;

    float avg(int anchorIdx) {
      float s = 0;
      int count = 0;
      for (int i = 0; i < AVG_WINDOW; i++) {
          if (rssiBuf[anchorIdx][i] > -900) { 
             s += rssiBuf[anchorIdx][i];
             count++;
          }
      }
      if (count == 0 || (millis() - lastUpdate[anchorIdx] > ANCHOR_TIMEOUT)) return -100;
      return s / count;
    }

    float rssiToDist(int id, float rssi) {
      return pow(10, (txPower[id] - rssi) / (10.0 * nValue[id]));
    }

    bool computeTrilateration(float* d) {
      const float lr = 0.01;
      float x = 2.0; 
      float y = 2.0; 
    
      for (int it = 0; it < 100; it++) {
        float dx = 0, dy = 0;
        for (int i = 0; i < 3; i++) {
           // Skip lost anchors? 
           // For simple trilateration we need 3 points. 
           // If signal is weak/lost, distance will be large, pushing us away.
           // Ideally we need at least 3 valid distances.
           if (d[i] > 100) continue; 

           float vx = x - x_anchor[i];
           float vy = y - y_anchor[i];
           float dist = sqrt(vx*vx + vy*vy) + 1e-6;
           float err = dist - d[i];
           dx += err * vx / dist;
           dy += err * vy / dist;
        }
        x -= lr * dx;
        y -= lr * dy;
        if (fabs(dx) + fabs(dy) < 1e-5) break; 
      }
      
      posX = x;
      posY = y;
      return true;
    }

public:
    Positioning() {
       for(int i=0; i<NUM_ANCHORS; i++) {
        for(int j=0; j<AVG_WINDOW; j++) rssiBuf[i][j] = -1000;
        head[i] = 0;
        lastUpdate[i] = 0;
       }
       valid = false;
    }

    int getAnchorIndex(std::string addr) {
        for(int i=0; i<NUM_ANCHORS; i++) {
            if (addr == anchor_addrs[i] || addr == std::string(anchor_addrs[i])) {
                return i;
            }
        }
        return -1;
    }

    // Update ONE anchor's RSSI
    void updateSingle(int anchorIdx, int rssi) {
        if (anchorIdx < 0 || anchorIdx >= NUM_ANCHORS) return;
        
        rssiBuf[anchorIdx][head[anchorIdx]] = rssi;
        head[anchorIdx] = (head[anchorIdx] + 1) % AVG_WINDOW;
        lastUpdate[anchorIdx] = millis();
    }
    
    // Call periodically to re-calculate position
    void compute() {
        float d[NUM_ANCHORS];
        int validCount = 0;
        
        for(int i=0; i<NUM_ANCHORS; i++) {
            float r = avg(i);
            if (r > -90) { // arbitrary valid threshold
                d[i] = rssiToDist(i, r);
                validCount++;
            } else {
                d[i] = 999; // Far away
            }
        }
        
        if (validCount >= 3) {
            computeTrilateration(d);
            valid = true;
        } else {
            // Signal lost or not enough anchors
            valid = false; 
        }
    }

    bool hasPosition() { return valid; }
    float getX() { return posX; }
    float getY() { return posY; }
};
#endif
