#include <Arduino.h>
#include <WiFi.h>

#include "car_bridge.h"

#define STM32_UART_BAUD   115200
#define STM32_UART_RX     21
#define STM32_UART_TX     47

/* Bộ đệm gom dữ liệu STM32 -> PC.
 * TRƯỚC ĐÂY: mỗi byte đọc được từ Serial1 bị gửi bằng MỘT lệnh
 * bridgeClient.write() riêng -> mỗi byte tạo ra một gói TCP nhỏ. Khi luồng
 * video MJPEG đang chạy song song, hàng loạt gói tin nhỏ này cạnh tranh
 * băng thông/CPU với luồng video, làm telemetry đến trễ/rớt và đôi khi làm
 * rớt luôn cả kết nối TCP điều khiển (lỗi "connection forcibly closed" bên
 * PC là hệ quả thường gặp của kiểu nghẽn này).
 * BÂY GIỜ: gom byte cho tới khi gặp '\n' (mọi dòng STM32 gửi đều kết thúc
 * bằng '\n', xem UART_SendString ở main.c) rồi gửi MỘT LẦN bằng một lệnh
 * write() -> giảm số gói TCP đi rất nhiều. */
#define BRIDGE_TX_BUF_SIZE 160

WiFiServer bridgeServer(8080);
WiFiClient bridgeClient;
static bool clientWasConnected = false;

static uint8_t txBuf[BRIDGE_TX_BUF_SIZE];
static size_t txLen = 0;

static void flushTxBuf()
{
    if (txLen == 0) {
        return;
    }
    if (bridgeClient && bridgeClient.connected()) {
        bridgeClient.write(txBuf, txLen);
    }
    txLen = 0;
}

static void sendBridgeInfo(const char *message)
{
    if (bridgeClient && bridgeClient.connected()) {
        flushTxBuf();   // đẩy nốt dữ liệu telemetry đang gom dở để giữ đúng thứ tự
        bridgeClient.print("[ESP32-S3] ");
        bridgeClient.println(message);
    }
}

static void stopStm32Car()
{
    /* STM32 treats S as a global emergency-stop command. */
    Serial1.print("S\n");
}

void carBridgeBegin()
{
    /* ESP32 RX receives STM32 PA9/TX; ESP32 TX drives STM32 PA10/RX. */
    Serial1.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_UART_RX, STM32_UART_TX);

    bridgeServer.begin();
    clientWasConnected = false;
    txLen = 0;
}

void carBridgeLoop()
{
    bool clientConnected = bridgeClient && bridgeClient.connected();

    /* Keep a single controlling PC. Replacing a disconnected client first
     * stops the car so a lost TCP link cannot leave AUTO mode running. */
    if (bridgeServer.hasClient()) {
        if (!clientConnected) {
            if (clientWasConnected) {
                stopStm32Car();
            }
            if (bridgeClient) {
                bridgeClient.stop();
            }
            txLen = 0;
            bridgeClient = bridgeServer.available();
            clientConnected = bridgeClient && bridgeClient.connected();
            if (clientConnected) {
                /* QUAN TRỌNG: setNoDelay() phải gọi TRÊN CLIENT vừa accept.
                 * Bản cũ gọi bridgeServer.setNoDelay(true) - không có tác
                 * dụng gì tới socket dữ liệu thật sự, Nagle vẫn bật khiến
                 * lệnh lái bị delay thêm vài chục ms không cần thiết. */
                bridgeClient.setNoDelay(true);
                sendBridgeInfo("TCP connected; UART bridge ready (RX=21, TX=47, 115200).");
            }
        } else {
            WiFiClient rejectedClient = bridgeServer.available();
            rejectedClient.stop();
        }
    }

    if (clientWasConnected && !clientConnected) {
        stopStm32Car();
        txLen = 0;
    }
    clientWasConnected = clientConnected;

    /* TCP -> UART: GUI commands to STM32. */
    if (clientConnected) {
        while (bridgeClient.available()) {
            char c = (char)bridgeClient.read();
            Serial1.write((uint8_t)c);
        }
    }

    /* UART -> TCP: STM32 telemetry to GUI, gom theo dòng thay vì gửi từng byte. */
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (clientConnected) {
            if (txLen < BRIDGE_TX_BUF_SIZE) {
                txBuf[txLen++] = (uint8_t)c;
            }
            if (c == '\n' || txLen >= BRIDGE_TX_BUF_SIZE) {
                flushTxBuf();
            }
        }
    }
}

bool carBridgeHasClient()
{
    return bridgeClient && bridgeClient.connected();
}
