#include "mode2_obstacle.h"
#include "hardware.h"

/* PID line follower with obstacle avoidance. The 10 ms control update runs in
 * TIM3_IRQHandler; sensor measurements themselves run in the foreground. */

static float Kp = 42.0f;
static float Ki = 0.0f;
static float Kd = 120.0f;

#define AUTO_MIN_BASE_SPEED      300
#define AUTO_MAX_BASE_SPEED      600
#define DEFAULT_BASE_SPEED       350

#define MAX_LOST_CYCLES          15
#define PIVOT_SPEED              300

#define OBSTACLE_DISTANCE_X10    150     /* 15.0 cm */

/* Thời gian quay né vật cản (mỗi lần AVOID_TURN1/AVOID_TURN2).
 * TRƯỚC ĐÂY 40 tick (400ms) - góc né bị nhận xét là "quay hơi nhanh, chưa
 * đủ rộng". Tăng lên 60 tick (600ms) để góc quay rộng hơn ~50%. Đây là
 * tham số có thể tinh chỉnh thêm tuỳ khung xe/tốc độ motor thực tế: tăng
 * nữa nếu vẫn thấy chưa đủ rộng, giảm nếu thấy né quá đà. */
#define AVOID_TURN_CYCLES        60      /* 600 ms */
#define AVOID_FORWARD_CYCLES     60      /* 600 ms */
#define AVOID_SAFETY_TIMEOUT     300     /* 3 seconds */

#define SERVO_CENTER_DEG         90
#define SERVO_LEFT_DEG           150
#define SERVO_RIGHT_DEG          30

/* ------------------------------------------------------------------
 * Timing cho pha quét 2 bên (AVOID_SCAN) - ĐÃ SỬA LỖI:
 * Bản cũ trigger đo HC-SR04 bên PHẢI ngay lúc vừa ra lệnh servo xoay từ
 * trái (150°) sang phải (30°) - một cú xoay 120° - rồi chỉ đợi 200ms là
 * đọc kết quả. Servo (SG90 và tương tự) xoay 120° thường cần ~250-400ms
 * mới ổn định, nghĩa là phép đo bên phải gần như chắc chắn được thực hiện
 * khi servo CÒN ĐANG XOAY DỞ, chưa hướng hẳn sang phải -> dữ liệu quét sai
 * lệch -> xe chọn nhầm hướng né / góc né trông "không đủ rộng".
 *
 * Sửa: chờ đủ thời gian servo ổn định THEO ĐÚNG GÓC XOAY của từng bước
 * (bước 120° chờ gần gấp đôi bước 60°) trước khi trigger đo, rồi mới chờ
 * thêm một khoảng để HC-SR04 chắc chắn đo xong (bản thân phép đo thường
 * xong trong <30ms, nhưng vẫn để dư cho chắc) trước khi đọc kết quả. */
#define SCAN_SETTLE_SMALL_TICKS   35   /* ~350ms: servo xoay ~60° (giữa <-> trái) */
#define SCAN_SETTLE_WIDE_TICKS    70   /* ~700ms: servo xoay ~120° (trái <-> phải), gấp đôi góc nên cần gần gấp đôi thời gian */
#define SCAN_HCSR04_WAIT_TICKS    15   /* ~150ms chờ sau khi trigger, đủ dư dả để HC-SR04 đo xong trước khi đọc kết quả */

#define SCAN_LEFT_TRIGGER_TICK   (SCAN_SETTLE_SMALL_TICKS)
#define SCAN_LEFT_READ_TICK      (SCAN_LEFT_TRIGGER_TICK + SCAN_HCSR04_WAIT_TICKS)
#define SCAN_RIGHT_TRIGGER_TICK  (SCAN_LEFT_READ_TICK + SCAN_SETTLE_WIDE_TICKS)
#define SCAN_RIGHT_READ_TICK     (SCAN_RIGHT_TRIGGER_TICK + SCAN_HCSR04_WAIT_TICKS)

/* Nhịp xin đo khoảng cách PHÍA TRƯỚC (dùng để phát hiện vật cản khi đang
 * bám line bình thường, và là giá trị hiển thị "Khoảng cách" trên GUI).
 * TRƯỚC ĐÂY mỗi 6 tick (~60ms). Giảm còn 4 tick (~40ms) cho GUI bám sát
 * thực tế hơn một chút. KHÔNG hạ thấp hơn nữa: HC-SR04 cần một khoảng
 * nghỉ tối thiểu giữa 2 lần đo để tiếng vọng (echo) của lần đo trước kịp
 * tắt hẳn, đo quá dồn dập dễ đọc nhầm tiếng vọng cũ thành vật cản ảo. */
#define FRONT_PING_DIV            4

/* TRƯỚC ĐÂY: PublishMotorLog() set telemetry_ready = 1 ở MỌI tick điều
 * khiển (mỗi 10ms -> 100 lần/giây), trong khi Mode 1 (mode1.c) đã giới hạn
 * xuống còn 1/5 tick (~50ms -> 20Hz) từ trước. Gửi dữ liệu dồn dập gấp 5
 * lần cần thiết này, cộng dồn với luồng video chạy song song, là nguyên
 * nhân khiến dữ liệu cảm biến hiển thị trên GUI bị trễ. Giảm về cùng nhịp
 * ~20Hz như Mode 1 (khớp luôn với TELEMETRY_POLL_MS=50 phía GUI). */
#define AUTO_TELE_DIV            5
static volatile int auto_tele_count = 0;

extern volatile int error;
extern volatile int last_error;
extern volatile int log_pwm_l;
extern volatile int log_pwm_r;
extern volatile int log_distance_cm_x10;
extern volatile int telemetry_ready;

static volatile float I_term = 0.0f;
static volatile int lost_count = 0;
static volatile int line_detected = 0;
static volatile int last_turn_direction = 0;
static volatile int Base_Speed = DEFAULT_BASE_SPEED;

typedef enum {
    AVOID_NONE = 0,
    AVOID_SCAN,
    AVOID_TURN1,
    AVOID_FORWARD1,
    AVOID_TURN2,
    AVOID_FORWARD2
} AvoidState_t;

static volatile AvoidState_t avoid_state = AVOID_NONE;
static volatile int avoid_timer = 0;
static volatile int avoid_turn_dir = 1;
static int ping_counter = 0;
static int16_t last_distance_cm_x10 = -1;
static int16_t scan_distance_left_cm_x10 = -1;
static int16_t scan_distance_right_cm_x10 = -1;

void Mode2_Obstacle_SetSpeedPercent(uint8_t speed_pct)
{
    if (speed_pct > 100U) {
        speed_pct = 100U;
    }

    if (speed_pct == 0U) {
        Base_Speed = 0;
        return;
    }

    Base_Speed = AUTO_MIN_BASE_SPEED +
                 ((int)speed_pct * (AUTO_MAX_BASE_SPEED - AUTO_MIN_BASE_SPEED)) / 100;
}

void Mode2_Obstacle_Init(void)
{
    line_detected = 0;
    I_term = 0.0f;
    error = 0;
    last_error = 0;
    lost_count = 0;
    last_turn_direction = 0;
    avoid_state = AVOID_NONE;
    avoid_timer = 0;
    avoid_turn_dir = 1;
    ping_counter = 0;
    auto_tele_count = 0;
    last_distance_cm_x10 = -1;
    scan_distance_left_cm_x10 = -1;
    scan_distance_right_cm_x10 = -1;
    log_distance_cm_x10 = HCSR04_GetDistance_cm_x10();
    Servo_SetAngle(SERVO_CENTER_DEG);
    Set_Motor_Outputs(0, 0);
}

static void PublishMotorLog(int pwm_l, int pwm_r)
{
    log_pwm_l = pwm_l;
    log_pwm_r = pwm_r;
    if (++auto_tele_count >= AUTO_TELE_DIV) {
        auto_tele_count = 0;
        telemetry_ready = 1;
    }
}

static void Run_Line_PID(uint8_t raw_state, uint8_t side_left, uint8_t side_right)
{
    int sensor_high_count = 0;

    for (int i = 0; i < 5; i++) {
        if ((raw_state >> i) & 1U) {
            sensor_high_count++;
        }
    }

    if (!line_detected) {
        if (sensor_high_count >= 1 && sensor_high_count <= 3) {
            line_detected = 1;
        } else {
            Set_Motor_Outputs(0, 0);
            PublishMotorLog(0, 0);
            return;
        }
    }

    if (sensor_high_count == 0 || sensor_high_count == 5) {
        int pwm_l;
        int pwm_r;

        lost_count++;
        if (lost_count > MAX_LOST_CYCLES) {
            Set_Motor_Outputs(0, 0);
            error = 0;
            last_error = 0;
            I_term = 0.0f;
            line_detected = 0;
            lost_count = 0;
            last_turn_direction = 0;
            PublishMotorLog(0, 0);
            return;
        }

        if (sensor_high_count == 0) {
            if (side_right && !side_left) {
                pwm_l = PIVOT_SPEED;
                pwm_r = -PIVOT_SPEED;
            } else if (side_left && !side_right) {
                pwm_l = -PIVOT_SPEED;
                pwm_r = PIVOT_SPEED;
            } else if (last_turn_direction >= 0) {
                pwm_l = PIVOT_SPEED;
                pwm_r = -PIVOT_SPEED;
            } else {
                pwm_l = -PIVOT_SPEED;
                pwm_r = PIVOT_SPEED;
            }
        } else {
            pwm_l = Base_Speed;
            pwm_r = Base_Speed;
        }

        Set_Motor_Outputs(pwm_l, pwm_r);
        PublishMotorLog(pwm_l, pwm_r);
        return;
    }

    lost_count = 0;

    switch (raw_state) {
        case 0x04: error =  0; break;
        case 0x0C: error =  1; break;
        case 0x08: error =  2; break;
        case 0x18: error =  3; break;
        case 0x10: error =  4; break;
        case 0x06: error = -1; break;
        case 0x02: error = -2; break;
        case 0x03: error = -3; break;
        case 0x01: error = -4; break;
        default:   error = last_error; break;
    }

    if (error > 0) {
        last_turn_direction = 1;
    } else if (error < 0) {
        last_turn_direction = -1;
    }

    float p_term = Kp * error;
    I_term += Ki * error;
    if (I_term > 200.0f) {
        I_term = 200.0f;
    } else if (I_term < -200.0f) {
        I_term = -200.0f;
    }
    float d_term = Kd * (error - last_error);
    last_error = error;

    int pid_value = (int)(p_term + I_term + d_term);
    int abs_error = (error >= 0) ? error : -error;
    int base_pwm = Base_Speed - (abs_error * 25);
    if (base_pwm < AUTO_MIN_BASE_SPEED) {
        base_pwm = AUTO_MIN_BASE_SPEED;
    }

    int pwm_l = base_pwm + pid_value;
    int pwm_r = base_pwm - pid_value;
    if (pwm_l > 999) pwm_l = 999;
    if (pwm_l < -999) pwm_l = -999;
    if (pwm_r > 999) pwm_r = 999;
    if (pwm_r < -999) pwm_r = -999;

    Set_Motor_Outputs(pwm_l, pwm_r);
    PublishMotorLog(pwm_l, pwm_r);
}

void Mode2_Obstacle_Update(uint8_t raw_state, uint8_t side_left, uint8_t side_right)
{
    int16_t newest_distance_cm_x10 = HCSR04_GetDistance_cm_x10();
    if (newest_distance_cm_x10 > 0) {
        last_distance_cm_x10 = newest_distance_cm_x10;
    } else if (newest_distance_cm_x10 < 0) {
        /* Do not use an old "obstacle detected" reading after a timeout. */
        last_distance_cm_x10 = -1;
    }
    log_distance_cm_x10 = newest_distance_cm_x10;

    if (Base_Speed <= 0) {
        Set_Motor_Outputs(0, 0);
        PublishMotorLog(0, 0);
        return;
    }

    if (avoid_state == AVOID_NONE) {
        if (++ping_counter >= FRONT_PING_DIV) {
            ping_counter = 0;
            HCSR04_RequestMeasurement();
        }

        if (last_distance_cm_x10 > 0 &&
            last_distance_cm_x10 < OBSTACLE_DISTANCE_X10) {
            Set_Motor_Outputs(0, 0);
            PublishMotorLog(0, 0);
            avoid_state = AVOID_SCAN;
            avoid_timer = 0;
            return;
        }

        Run_Line_PID(raw_state, side_left, side_right);
        return;
    }

    switch (avoid_state) {
        case AVOID_SCAN:
            if (avoid_timer == 0) {
                Servo_SetAngle(SERVO_LEFT_DEG);        /* bắt đầu xoay ~60° sang trái */
            } else if (avoid_timer == SCAN_LEFT_TRIGGER_TICK) {
                HCSR04_RequestMeasurement();           /* servo đã ổn định ở bên trái, đo */
            } else if (avoid_timer == SCAN_LEFT_READ_TICK) {
                scan_distance_left_cm_x10 = HCSR04_GetDistance_cm_x10();
                Servo_SetAngle(SERVO_RIGHT_DEG);       /* bắt đầu xoay ~120° sang phải */
            } else if (avoid_timer == SCAN_RIGHT_TRIGGER_TICK) {
                HCSR04_RequestMeasurement();           /* servo đã ổn định ở bên phải, đo */
            } else if (avoid_timer == SCAN_RIGHT_READ_TICK) {
                scan_distance_right_cm_x10 = HCSR04_GetDistance_cm_x10();
                Servo_SetAngle(SERVO_CENTER_DEG);

                if (scan_distance_left_cm_x10 <= 0) {
                    avoid_turn_dir = 1;
                } else if (scan_distance_right_cm_x10 <= 0) {
                    avoid_turn_dir = -1;
                } else {
                    avoid_turn_dir = (scan_distance_right_cm_x10 > scan_distance_left_cm_x10) ? 1 : -1;
                }

                avoid_state = AVOID_TURN1;
                avoid_timer = 0;
                break;
            }
            avoid_timer++;
            break;

        case AVOID_TURN1: {
            int pwm_l = (avoid_turn_dir > 0) ? PIVOT_SPEED : -PIVOT_SPEED;
            int pwm_r = -pwm_l;
            Set_Motor_Outputs(pwm_l, pwm_r);
            PublishMotorLog(pwm_l, pwm_r);

            if (++avoid_timer >= AVOID_TURN_CYCLES) {
                avoid_state = AVOID_FORWARD1;
                avoid_timer = 0;
            }
            break;
        }

        case AVOID_FORWARD1:
            Set_Motor_Outputs(Base_Speed, Base_Speed);
            PublishMotorLog(Base_Speed, Base_Speed);
            if (++avoid_timer >= AVOID_FORWARD_CYCLES) {
                avoid_state = AVOID_TURN2;
                avoid_timer = 0;
            }
            break;

        case AVOID_TURN2: {
            int pwm_l = (avoid_turn_dir > 0) ? -PIVOT_SPEED : PIVOT_SPEED;
            int pwm_r = -pwm_l;
            Set_Motor_Outputs(pwm_l, pwm_r);
            PublishMotorLog(pwm_l, pwm_r);

            if (++avoid_timer >= AVOID_TURN_CYCLES) {
                avoid_state = AVOID_FORWARD2;
                avoid_timer = 0;
            }
            break;
        }

        case AVOID_FORWARD2: {
            int sensor_high_count = 0;
            Set_Motor_Outputs(Base_Speed, Base_Speed);
            PublishMotorLog(Base_Speed, Base_Speed);

            for (int i = 0; i < 5; i++) {
                if ((raw_state >> i) & 1U) {
                    sensor_high_count++;
                }
            }

            if (sensor_high_count >= 1 && sensor_high_count <= 3) {
                avoid_state = AVOID_NONE;
                avoid_timer = 0;
                line_detected = 1;
                error = 0;
                last_error = 0;
                I_term = 0.0f;
            } else if (++avoid_timer >= AVOID_SAFETY_TIMEOUT) {
                avoid_state = AVOID_NONE;
                avoid_timer = 0;
                line_detected = 0;
            }
            break;
        }

        default:
            avoid_state = AVOID_NONE;
            avoid_timer = 0;
            Set_Motor_Outputs(0, 0);
            PublishMotorLog(0, 0);
            break;
    }
}
