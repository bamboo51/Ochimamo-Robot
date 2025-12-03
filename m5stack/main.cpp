#include <NimBLEDevice.h>
#include <M5Unified.h>

// Python側と同じUUIDを使用
#define SERVICE_UUID        "0000abcd-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00001234-0000-1000-8000-00805f9b34fb"

NimBLEServer *pServer = nullptr;
NimBLECharacteristic *pCharacteristic = nullptr;
bool isWarningReceived = false;
unsigned long warningStartTime = 0;
const unsigned long WARNING_DURATION = 1000;
int messageCount = 0;

// サーバー接続/切断のコールバック
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
        Serial.println("=============================");
        Serial.println("✓ Client Connected!");
        Serial.print("Address: ");
        Serial.println(connInfo.getAddress().toString().c_str());
        Serial.println("=============================");
        
        M5.Display.fillScreen(TFT_BLACK);
        M5.Display.setCursor(0, 0);
        M5.Display.setTextSize(2);
        M5.Display.setTextColor(TFT_GREEN);
        M5.Display.println("Python Connected!");
        M5.Display.setTextColor(TFT_WHITE);
        M5.Display.println("Ready for warnings...");
        M5.Display.println("");
        M5.Display.print("Messages: ");
        M5.Display.println(messageCount);
        
        // 接続パラメータを更新（安定性向上）
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 60);
    }
    
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
        Serial.println("=============================");
        Serial.println("✗ Client Disconnected!");
        Serial.print("Reason: ");
        Serial.println(reason);
        Serial.println("=============================");
        
        M5.Display.fillScreen(TFT_BLACK);
        M5.Display.setCursor(0, 0);
        M5.Display.setTextSize(2);
        M5.Display.setTextColor(TFT_YELLOW);
        M5.Display.println("Connection Lost");
        M5.Display.setTextColor(TFT_WHITE);
        M5.Display.println("Restarting advertising...");
        
        // アドバタイジング再開
        NimBLEDevice::startAdvertising();
        Serial.println("Advertising restarted");
    }
    
    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) {
        Serial.print("MTU updated: ");
        Serial.println(MTU);
    }
};

// データ受信のコールバック
class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) {
        std::string value = pCharacteristic->getValue();
        
        messageCount++;
        
        Serial.println("=============================");
        Serial.print("📨 Message #");
        Serial.println(messageCount);
        Serial.print("Length: ");
        Serial.println(value.length());
        
        if (value.length() > 0) {
            Serial.print("First byte (HEX): 0x");
            Serial.println(value[0], HEX);
            Serial.print("First byte (DEC): ");
            Serial.println((int)value[0]);
            Serial.print("First byte (CHAR): '");
            Serial.print(value[0]);
            Serial.println("'");
            
            // 全バイトを表示
            Serial.print("All bytes: ");
            for(size_t i = 0; i < value.length(); i++) {
                Serial.print("0x");
                Serial.print(value[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
            
            // "1" (0x31) または 0x01 を受信したら警告フラグを立てる
            if (value[0] == '1' || value[0] == 0x01 || value[0] == 1) {
                isWarningReceived = true;
                Serial.println("⚠️  WARNING FLAG SET TO TRUE!");
            } else {
                Serial.println("⚠️  Value did NOT match warning condition");
                Serial.println("    Expected: '1' (0x31) or 0x01 or 1");
            }
        } else {
            Serial.println("⚠️  Empty message received!");
        }
        Serial.println("=============================");
        
        // 画面に受信カウントを表示
        if (warningStartTime == 0) {
            M5.Display.fillRect(0, 40, 320, 20, TFT_BLACK);
            M5.Display.setCursor(0, 40);
            M5.Display.setTextColor(TFT_WHITE);
            M5.Display.print("Messages: ");
            M5.Display.println(messageCount);
        }
    }
};

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    
    // シリアルが準備できるまで少し待つ
    delay(1000);
    
    // スピーカーの初期化
    M5.Speaker.setVolume(255);

    // 初期画面表示
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.println("BLE Initializing...");
    Serial.println("=============================");
    Serial.println("🚀 M5Stack BLE System Starting...");
    Serial.println("=============================");

    // BLEデバイス初期化
    NimBLEDevice::init("WORKER_M5");
    
    // セキュリティ設定（ペアリング不要に設定）
    NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    
    // BLEサーバー作成
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // BLEサービス作成
    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    // BLE特性作成
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | 
        NIMBLE_PROPERTY::WRITE | 
        NIMBLE_PROPERTY::WRITE_NR |
        NIMBLE_PROPERTY::NOTIFY
    );

    pCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();

    // アドバタイジング設定
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
    
    Serial.println("✓ Advertising started");
    
    Serial.println("✓ BLE Service Started");
    Serial.println("✓ Advertising Started");
    Serial.print("Device Name: WORKER_M5");
    Serial.println();
    Serial.print("Service UUID: ");
    Serial.println(SERVICE_UUID);
    Serial.print("Characteristic UUID: ");
    Serial.println(CHARACTERISTIC_UUID);
    Serial.print("Server address: ");
    Serial.println(NimBLEDevice::getAddress().toString().c_str());
    Serial.println("=============================");
    Serial.println("Waiting for connection...");
    Serial.println("=============================");
    
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(TFT_CYAN);
    M5.Display.println("BLE Ready!");
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.println("Device: WORKER_M5");
    M5.Display.println("");
    M5.Display.println("Waiting for Python...");
    M5.Display.println("");
    M5.Display.setTextSize(1);
    M5.Display.print("Addr: ");
    M5.Display.println(NimBLEDevice::getAddress().toString().c_str());
}

void loop() {
    M5.update();
    
    // 接続状態を定期的にチェック（デバッグ用）
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 5000) {
        lastCheck = millis();
        Serial.print("Connected clients: ");
        Serial.println(pServer->getConnectedCount());
    }

    // 警告が届いたら警告表示を開始
    if (isWarningReceived && warningStartTime == 0) {
        Serial.println("=============================");
        Serial.println("🚨 WARNING ACTION START!");
        Serial.println("=============================");
        
        warningStartTime = millis();
        
        // 画面を赤で塗りつぶして警告表示
        M5.Display.fillScreen(TFT_RED);
        M5.Display.setTextColor(TFT_WHITE);
        M5.Display.setCursor(20, 60);
        M5.Display.setTextSize(3);
        M5.Display.print("DANGER!");
        
        // 警告音を鳴らす
        M5.Speaker.tone(4000, WARNING_DURATION);
        
        isWarningReceived = false;  // フラグをクリア
    }
    
    // 警告表示時間が経過したら画面をリセット
    if (warningStartTime > 0 && millis() - warningStartTime >= WARNING_DURATION) {
        Serial.println("✓ WARNING ACTION END");
        Serial.println("=============================");
        
        // 画面を元に戻す
        M5.Display.fillScreen(TFT_BLACK);
        M5.Display.setCursor(0, 0);
        M5.Display.setTextSize(2);
        M5.Display.setTextColor(TFT_WHITE);
        
        if (pServer->getConnectedCount() > 0) {
            M5.Display.setTextColor(TFT_GREEN);
            M5.Display.println("Connected");
            M5.Display.setTextColor(TFT_WHITE);
            M5.Display.println("Ready for warnings...");
            M5.Display.println("");
            M5.Display.print("Messages: ");
            M5.Display.println(messageCount);
        } else {
            M5.Display.setTextColor(TFT_YELLOW);
            M5.Display.println("Waiting for Python...");
        }
        
        warningStartTime = 0;  // タイマーをリセット
    }
    
    delay(20);
}