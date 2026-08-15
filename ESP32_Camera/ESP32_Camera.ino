#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "car_bridge.h"

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM      11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

const char *ssid = "Ngo5C6";
const char *password = "12345678a@";

void startCameraServer();
void setupLedFlash();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  // GIỮ NGUYÊN 20MHz: từng thử tăng lên 24MHz nhưng bị nhiễu hình (artifact),
  // đã revert. Muốn tăng chất lượng thì nên chỉnh jpeg_quality/sharpness
  // (bên dưới) thay vì đụng vào xclk.
  config.xclk_freq_hz = 20000000;

  config.frame_size = FRAMESIZE_CIF;   // 400x296 - tăng 1 bước từ QVGA (320x240)
                                        // thay vì nhảy thẳng lên VGA, để tránh
                                        // lặp lại rủi ro nóng máy như jpeg_quality
                                        // trước đó (ảnh to hơn = ESP32 nén nặng
                                        // hơn + tốn băng thông WiFi hơn).
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_DRAM;
  // jpeg_quality: 0-63, SỐ CÀNG THẤP ẢNH CÀNG NÉT nhưng ESP32 phải nén
  // NẶNG HƠN mỗi frame (tốn CPU + dòng điện hơn). ĐÃ RÚT LẠI về 20 (mặc
  // định gốc) sau khi ghi nhận camera OV3660 nóng >60°C kèm mất kết nối -
  // giảm quality (12) là nghi phạm hàng đầu vì đúng hướng "bắt ESP32 làm
  // việc nặng hơn liên tục", CHƯA kiểm chứng được trên phần cứng thật
  // trước khi đổi. Muốn nét hơn, nên test lại giá trị TỪ TỪ (vd 18 -> 16)
  // sau khi đã xác nhận hết nóng máy, không nhảy thẳng xuống 12 nữa.
  config.jpeg_quality = 20;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.fb_count = 2;
        config.jpeg_quality = 20;
        config.grab_mode = CAMERA_GRAB_LATEST;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_CIF);
  }

  // ĐÃ RÚT LẠI set_sharpness()/set_denoise() đã thêm lượt trước: đây là
  // 2 thay đổi CHƯA kiểm chứng được trên phần cứng thật (không có thiết
  // bị để test), và trùng thời điểm ghi nhận camera OV3660 nóng >60°C +
  // mất kết nối timeout. Không có bằng chứng chắc chắn 2 dòng này là thủ
  // phạm, nhưng khi nghi ngờ ảnh hưởng tới an toàn phần cứng thì rút lại
  // trước, xác nhận hết nóng máy với cấu hình an toàn này đã rồi mới thử
  // bật lại TỪNG DÒNG MỘT (không bật cả 2 cùng lúc) để cô lập nguyên nhân
  // nếu vẫn muốn ảnh nét hơn sau này.

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  startCameraServer();
  Serial.println("Camera Server Started!");

  // Mở cầu nối STM32 (Bắt buộc dùng carBridgeBegin để cấu hình chân 21, 47)
  carBridgeBegin();
}

void loop() {
  carBridgeLoop();
  yield(); // <-- Đã sửa: Không block CPU, chống tràn bộ đệm UART
}
