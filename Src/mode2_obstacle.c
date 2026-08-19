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

/* Khoảng cách kích hoạt né vật cản (đã giữ nguyên theo test thực tế) */
#define OBSTACLE_DISTANCE_X10    160

#define AVOID_SAFETY_TIMEOUT     300     /* 3 seconds - dò lại line sau khi né xong */

/* ==================================================================
 * THÔNG SỐ NÉ VẬT CẢN (giữ nguyên 100% theo test thực tế)
 * ================================================================== */
#define AVOID_PIVOT_SPEED         500
#define AVOID_SPEED               350

#define AVOID_PIVOT_OUT_CYCLES    30
#define AVOID_PIVOT_BACK_CYCLES   40

#define AVOID_WAIT_CYCLES         100
#define AVOID_FORWARD_SIDE_CYCLES 200
/* ================================================================== */

#define SERVO_CENTER_DEG          90
#define SERVO_LEFT_DEG            170
#define SERVO_RIGHT_DEG           10

#define SCAN_SETTLE_TICKS         100
#define SCAN_HCSR04_WAIT_TICKS    15

#define SCAN_LEFT_TRIGGER_TICK   (SCAN_SETTLE_TICKS)
#define SCAN_LEFT_READ_TICK      (SCAN_LEFT_TRIGGER_TICK + SCAN_HCSR04_WAIT_TICKS)
#define SCAN_RIGHT_TRIGGER_TICK  (SCAN_LEFT_READ_TICK + SCAN_SETTLE_TICKS)
#define SCAN_RIGHT_READ_TICK     (SCAN_RIGHT_TRIGGER_TICK + SCAN_HCSR04_WAIT_TICKS)

/* Bổ sung nhịp chờ 1 giây cho Servo về giữa hút xong dòng điện trước khi motor bẻ lái */
#define SCAN_CENTER_SETTLE_TICK  (SCAN_RIGHT_READ_TICK + SCAN_SETTLE_TICKS)

#define FRONT_PING_DIV            4
#define AUTO_TELE_DIV             5
static volatile int auto_tele_count = 0;

#define AIRBORNE_CONFIRM_TICKS    3
static volatile int airborne_count = 0;

/* ==================================================================
 * BẮT GÓC VUÔNG (line giao lộ)
 * ------------------------------------------------------------------
 * Ý đồ: mắt biên (side_left/side_right) phát hiện lệch trước, chờ xe
 * tiến thêm một đoạn ngắn tới gần tâm ngã tư rồi mới bẻ 90 độ.
 *
 * LỊCH SỬ SỬA LỖI (quan trọng): bản đầu dùng điều kiện chốt bẻ là
 * "sensor_high_count >= 1" ngay tick kế tiếp sau khi ARM. Vấn đề: khi
 * đang bám line bình thường, sensor_high_count LUÔN nằm trong 1-3 (xem
 * switch-case error bên dưới, các pattern hợp lệ chỉ có tối đa 2 bit
 * sáng) - tức điều kiện đó gần như luôn đúng ngay từ tick đầu tiên, nên
 * xe bẻ cua gần như tức thì lúc vừa ARM, không kịp tiến vào giữa ngã tư
 * như ý đồ ban đầu.
 *
 * Thử đổi sang yêu cầu 4 mắt giữa sáng cùng lúc (vạch ngang phủ rộng) để
 * chốt bẻ, nhưng đo thực tế trên xe cho thấy khoảng cách vật lý giữa mắt
 * biên và cụm 5 mắt giữa CHỈ ~3cm - quãng đường quá ngắn, xe thường chỉ
 * kịp lấn qua 3/5 mắt (không tới 4) trước khi lố qua khỏi vạch ngang, mid
 * -array dễ bắt trượt hoàn toàn (0 mắt sáng) nếu chờ ngưỡng 4.
 *
 * GIẢI PHÁP HIỆN TẠI: kết hợp 3 lớp bảo vệ, không phụ thuộc hoàn toàn 1
 * loại cảm biến:
 *   1) CORNER_MIN_ADVANCE_TICKS: chờ tối thiểu, tuyệt đối không bẻ ngay
 *      tick vừa ARM, cho xe kịp nhích vào ngã tư.
 *   2) Xác nhận chính xác: >=3/5 mắt giữa sáng (khớp đúng số đo thực tế
 *      3cm ở trên) - không lẫn với trạng thái bám line thường.
 *   3) CORNER_MAX_ADVANCE_TICKS: lưới an toàn theo thời gian (200ms, quy
 *      đổi từ quãng 3cm ở tốc độ hiện tại) - nếu mid-array bắt trượt hoàn
 *      toàn (xe đi lố), vẫn bẻ đúng lúc thay vì bỏ lỡ hẳn góc cua.
 * ================================================================== */
#define CORNER_CONFIRM_TICKS       3    /* debounce ARM: side_left != side_right phải giữ liên tục 30ms mới tính, tránh nhiễu/rung ở khúc cong thường */
#define CORNER_MIN_ADVANCE_TICKS   5    /* 50ms - chờ tối thiểu sau ARM trước khi cho phép bẻ */
#define CORNER_MAX_ADVANCE_TICKS   20   /* 200ms - timeout tối đa nếu mid-array không xác nhận được, ước theo quãng ~3cm giữa 2 cụm cảm biến */
#define CORNER_ARM_CANCEL_TICKS    60   /* 600ms - hủy ARM nếu quá lâu vẫn chưa chốt bẻ (nghi ARM giả), lưới an toàn cuối cùng */
#define CORNER_TURN_TICKS         60

/* PWM xoay tại chỗ để bẻ góc vuông.
 * SỬA LỖI TIỀM ẨN: bản trước dùng 320 - chỉ nhỉnh hơn chút so với
 * MANUAL_MIN_PWM (300, ngưỡng tối thiểu để bánh xe CHẠY THẲNG). Xoay tại
 * chỗ (2 bánh ngược chiều) cần thắng ma sát tĩnh của CẢ HAI bánh cùng lúc,
 * nặng hơn hẳn so với đi thẳng, đặc biệt lúc vào ngã tư xe đã giảm tốc,
 * không còn đà tiến hỗ trợ. Đồng bộ về đúng AVOID_PIVOT_SPEED (500) - giá
 * trị xoay tại chỗ ĐÃ ĐƯỢC KIỂM CHỨNG THỰC TẾ ở pha né vật cản, thay vì
 * dùng một con số riêng (320) chưa rõ đã test độc lập cho corner hay chưa. */
#define CORNER_PIVOT_SPEED        500

/* "Breakaway kick": vài tick đầu tiên của cú bẻ bơm PWM CAO HƠN mức duy
 * trì, rồi mới hạ về CORNER_PIVOT_SPEED. Lý do: ma sát TĨNH (lúc bánh còn
 * đứng yên) luôn cao hơn ma sát ĐỘNG (lúc bánh đã lăn) - PWM vừa đủ để
 * DUY TRÌ xoay chưa chắc đủ để BẮT ĐẦU xoay từ trạng thái đứng yên. Chỉ
 * áp dụng trong thời gian ngắn để tránh xoay quá đà/lố góc.
 * LƯU Ý: các con số dưới đây là điểm khởi đầu hợp lý theo nguyên lý vật
 * lý, KHÔNG thay thế cho việc tune lại trên xe thật - ma sát thực tế phụ
 * thuộc bề mặt sàn, tải trọng, độ mòn bánh xe cụ thể của bạn. */
#define CORNER_BREAKAWAY_TICKS      5    /* 50ms đầu tiên dùng PWM kick */
#define CORNER_BREAKAWAY_SPEED    650    /* > CORNER_PIVOT_SPEED, dưới xa PWM_MAX=999 */

/* ==================================================================
 * THỜI GIAN MÙ (COOLDOWN) SIÊU ÂM
 * ================================================================== */
#define AVOID_COOLDOWN_TICKS      150    /* 1.5 giây "mù" siêu âm sau khi về vạch */
static volatile int avoid_cooldown_timer = 0;

typedef enum {
    CORNER_NONE = 0,
    CORNER_ARMED,        /* Đã nhận diện góc cua từ mắt biên, đang tiến vào tâm */
    CORNER_TURNING       /* Đang thực hiện xoay xe */
} CornerState_t;
static volatile CornerState_t corner_state = CORNER_NONE;
static volatile int corner_dir = 0;
static volatile int corner_timer = 0;
static volatile int corner_confirm_count = 0;

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

/* Biến đếm lọc nhiễu cho pha tái bám line (cảm biến đơn lướt qua nhanh) */
static volatile int reacquire_confirm_count = 0;

typedef enum {
    AVOID_NONE = 0,
    AVOID_SCAN,
    AVOID_PIVOT_OUT,
    AVOID_WAIT1,
    AVOID_FORWARD_SIDE,
    AVOID_WAIT2,
    AVOID_PIVOT_BACK,
    AVOID_FORWARD_REACQUIRE
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
    avoid_cooldown_timer = 0;
    reacquire_confirm_count = 0;
    ping_counter = 0;
    auto_tele_count = 0;
    airborne_count = 0;
    corner_state = CORNER_NONE;
    corner_dir = 0;
    corner_timer = 0;
    corner_confirm_count = 0;
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

    if (raw_state == 0U && side_left == 0U && side_right == 0U) {
        if (++airborne_count >= AIRBORNE_CONFIRM_TICKS) {
            Set_Motor_Outputs(0, 0);
            PublishMotorLog(0, 0);
            line_detected = 0;
            error = 0;
            last_error = 0;
            I_term = 0.0f;
            lost_count = 0;
            last_turn_direction = 0;
            return;
        }
    } else {
        airborne_count = 0;
    }

    for (int i = 0; i < 5; i++) {
        if ((raw_state >> i) & 1U) {
            sensor_high_count++;
        }
    }

    /* ---- 1. THỰC THI BẺ GÓC VUÔNG ---- */
    if (corner_state == CORNER_TURNING) {
        /* Vài tick đầu dùng PWM "kick" cao hơn để thắng ma sát tĩnh lúc
         * bánh xe còn đứng yên, sau đó hạ về PWM duy trì - xem giải thích
         * ở CORNER_BREAKAWAY_TICKS/CORNER_BREAKAWAY_SPEED phía trên. */
        int turn_speed = (corner_timer < CORNER_BREAKAWAY_TICKS)
                              ? CORNER_BREAKAWAY_SPEED
                              : CORNER_PIVOT_SPEED;
        int pwm_l = (corner_dir > 0) ?  turn_speed : -turn_speed;
        int pwm_r = -pwm_l;
        Set_Motor_Outputs(pwm_l, pwm_r);
        PublishMotorLog(pwm_l, pwm_r);

        if (sensor_high_count >= 1 && sensor_high_count <= 3) {
            corner_state = CORNER_NONE;
            corner_timer = 0;
            error = 0;
            last_error = 0;
            I_term = 0.0f;
            lost_count = 0;
        } else if (++corner_timer >= CORNER_TURN_TICKS) {
            corner_state = CORNER_NONE;
            corner_timer = 0;
        }
        return;
    }

    /* ---- 2. KIỂM TRA TRẠNG THÁI CHỜ BẺ GÓC (ARMED) ---- */
    if (corner_state == CORNER_ARMED) {
        corner_timer++;

        /* Bình thường khi bám line, sensor_high_count chỉ rơi vào 1 hoặc 2
         * bit sáng (xem switch-case error bên dưới: 0x04,0x0C,0x08,0x18,
         * 0x10,0x06,0x02,0x03,0x01 đều tối đa 2 bit). >=3 bit sáng gần như
         * chắc chắn là vạch ngang giao lộ, khớp với thực tế đo được (3/5
         * mắt sáng lúc xe tới đúng ngã tư) - không lẫn với lúc bám line
         * bình thường. */
        int wide_line = (sensor_high_count >= 3);

        if (corner_timer >= CORNER_MIN_ADVANCE_TICKS &&
            (wide_line || corner_timer >= CORNER_MAX_ADVANCE_TICKS)) {
            corner_state = CORNER_TURNING;
            corner_timer = 0;
            return; /* Bắt đầu bẻ, bỏ qua PID bên dưới */
        }

        /* Hủy ARM nếu quá lâu vẫn chưa chốt bẻ (nghi ARM giả do nhiễu ở
         * khúc cong, không phải ngã tư thật) - lưới an toàn cuối, canh dư
         * hẳn so với CORNER_MAX_ADVANCE_TICKS. */
        if (corner_timer > CORNER_ARM_CANCEL_TICKS) {
            corner_state = CORNER_NONE;
            corner_timer = 0;
        }
        /* LƯU Ý: Không return ở nhánh này. Khi đang ARMED mà chưa đủ điều
         * kiện chốt bẻ, xe VẪN PHẢI CHẠY PID bình thường để tiếp tục bám
         * line, tiến dần vào ngã tư cho tới lúc chốt bẻ hoặc bị hủy. */
    }

    /* ---- 3. BẮT TÍN HIỆU GÓC VUÔNG TỪ MẮT BIÊN (CÓ LỌC NHIỄU) ---- */
    if (corner_state == CORNER_NONE && line_detected && (side_left != side_right)) {
        if (++corner_confirm_count >= CORNER_CONFIRM_TICKS) {
            corner_state = CORNER_ARMED;
            corner_dir = side_right ? 1 : -1;
            corner_timer = 0;
            corner_confirm_count = 0;
        }
    } else {
        corner_confirm_count = 0;
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

    /* ---- CHẠY PID BÌNH THƯỜNG ---- */
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
        last_distance_cm_x10 = -1;
    }
    log_distance_cm_x10 = newest_distance_cm_x10;

    if (Base_Speed <= 0) {
        Set_Motor_Outputs(0, 0);
        PublishMotorLog(0, 0);
        return;
    }

    if (avoid_state == AVOID_NONE) {
        /* Trừ dần thời gian mù siêu âm (nếu có) */
        if (avoid_cooldown_timer > 0) {
            avoid_cooldown_timer--;
        }

        if (++ping_counter >= FRONT_PING_DIV) {
            ping_counter = 0;
            HCSR04_RequestMeasurement();
        }

        /* CHỈ cho phép né vật cản nếu ĐÃ HẾT thời gian mù */
        if (avoid_cooldown_timer == 0 &&
            last_distance_cm_x10 > 0 &&
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
                Servo_SetAngle(SERVO_LEFT_DEG);
            } else if (avoid_timer == SCAN_LEFT_TRIGGER_TICK) {
                HCSR04_RequestMeasurement();
            } else if (avoid_timer == SCAN_LEFT_READ_TICK) {
                scan_distance_left_cm_x10 = HCSR04_GetDistance_cm_x10();
                Servo_SetAngle(SERVO_RIGHT_DEG);
            } else if (avoid_timer == SCAN_RIGHT_TRIGGER_TICK) {
                HCSR04_RequestMeasurement();
            } else if (avoid_timer == SCAN_RIGHT_READ_TICK) {
                scan_distance_right_cm_x10 = HCSR04_GetDistance_cm_x10();
                Servo_SetAngle(SERVO_CENTER_DEG);
            } else if (avoid_timer == SCAN_CENTER_SETTLE_TICK) {
                if (scan_distance_left_cm_x10 <= 0) {
                    avoid_turn_dir = 1;
                } else if (scan_distance_right_cm_x10 <= 0) {
                    avoid_turn_dir = -1;
                } else {
                    avoid_turn_dir = (scan_distance_right_cm_x10 > scan_distance_left_cm_x10) ? 1 : -1;
                }

                avoid_state = AVOID_PIVOT_OUT;
                avoid_timer = 0;
                break;
            }
            avoid_timer++;
            break;

        case AVOID_PIVOT_OUT: {
            int pwm_l = (avoid_turn_dir > 0) ?  AVOID_PIVOT_SPEED : -AVOID_PIVOT_SPEED;
            int pwm_r = -pwm_l;

            Set_Motor_Outputs(pwm_l, pwm_r);
            PublishMotorLog(pwm_l, pwm_r);

            if (++avoid_timer >= AVOID_PIVOT_OUT_CYCLES) {
                avoid_state = AVOID_WAIT1;
                avoid_timer = 0;
            }
            break;
        }

        case AVOID_WAIT1: {
            Set_Motor_Outputs(0, 0);
            PublishMotorLog(0, 0);

            if (++avoid_timer >= AVOID_WAIT_CYCLES) {
                avoid_state = AVOID_FORWARD_SIDE;
                avoid_timer = 0;
            }
            break;
        }

        case AVOID_FORWARD_SIDE:
            Set_Motor_Outputs(AVOID_SPEED, AVOID_SPEED);
            PublishMotorLog(AVOID_SPEED, AVOID_SPEED);

            if (++avoid_timer >= AVOID_FORWARD_SIDE_CYCLES) {
                avoid_state = AVOID_WAIT2;
                avoid_timer = 0;
            }
            break;

        case AVOID_WAIT2: {
            Set_Motor_Outputs(0, 0);
            PublishMotorLog(0, 0);

            if (++avoid_timer >= AVOID_WAIT_CYCLES) {
                avoid_state = AVOID_PIVOT_BACK;
                avoid_timer = 0;
            }
            break;
        }

        case AVOID_PIVOT_BACK: {
            int pwm_l = (avoid_turn_dir > 0) ? -AVOID_PIVOT_SPEED : AVOID_PIVOT_SPEED;
            int pwm_r = -pwm_l;

            Set_Motor_Outputs(pwm_l, pwm_r);
            PublishMotorLog(pwm_l, pwm_r);

            if (++avoid_timer >= AVOID_PIVOT_BACK_CYCLES) {
                avoid_state = AVOID_FORWARD_REACQUIRE;
                avoid_timer = 0;
                reacquire_confirm_count = 0; /* Reset bộ lọc nhiễu cho pha bám line */
            }
            break;
        }

        case AVOID_FORWARD_REACQUIRE: {
            int sensor_high_count = 0;

            Set_Motor_Outputs(AVOID_SPEED, AVOID_SPEED);
            PublishMotorLog(AVOID_SPEED, AVOID_SPEED);

            for (int i = 0; i < 5; i++) {
                if ((raw_state >> i) & 1U) {
                    sensor_high_count++;
                }
            }

            /* Lọc nhiễu: phải nhận line 2 nhịp liên tiếp (20ms) mới thoát để chống xóc nảy */
            if (sensor_high_count >= 1 || side_left || side_right) {
                if (++reacquire_confirm_count >= 2) {
                    avoid_state = AVOID_NONE;
                    avoid_timer = 0;
                    line_detected = 1;
                    error = 0;
                    last_error = 0;
                    I_term = 0.0f;

                    /* KÍCH HOẠT THỜI GIAN MÙ VÀ XÓA DỮ LIỆU CŨ */
                    avoid_cooldown_timer = AVOID_COOLDOWN_TICKS;
                    last_distance_cm_x10 = -1;
                    reacquire_confirm_count = 0;
                }
            } else {
                reacquire_confirm_count = 0; /* Có nhiễu hoặc mất line -> đếm lại */
            }

            if (++avoid_timer >= AVOID_SAFETY_TIMEOUT) {
                avoid_state = AVOID_NONE;
                avoid_timer = 0;
                line_detected = 0;

                avoid_cooldown_timer = AVOID_COOLDOWN_TICKS;
                last_distance_cm_x10 = -1;
                reacquire_confirm_count = 0;
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
