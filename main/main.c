#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

static const char *TAG = "ROBOT";

#define RCCHECK(fn)        { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ ESP_LOGE(TAG,"rcl fail %s L%d",#fn,__LINE__); }}
#define RCSOFTCHECK(fn)    { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ ESP_LOGW(TAG,"rcl soft %s",#fn); }}
#define constrain(v,lo,hi) ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))

/* ===========================================================
   TOF — UART2
   TOF TX  →  ESP32 D16  (ESP32 RX2)  TOF_RX_PIN = 16
   TOF RX  →  ESP32 D17  (ESP32 TX2)  TOF_TX_PIN = 17
=========================================================== */
#define TOF_UART_NUM    UART_NUM_2
#define TOF_UART_BAUD   921600
#define TOF_RX_PIN      16          /* ESP32 RX2 ← TOF TX */
#define TOF_TX_PIN      17          /* ESP32 TX2 → TOF RX */
#define TOF_BUF_SIZE    1024
#define TOF_FRAME_LEN   16
#define TOF_FRAME_HDR   0x57
#define TOF_FUNC_MARK   0x00

/* ===========================================================
   IMU — GY-87 MPU6050
   SDA → D32 (yellow wire)
   SCL → D33 (white wire)
=========================================================== */
#define IMU_I2C_NUM       I2C_NUM_0
#define IMU_SDA_PIN       32
#define IMU_SCL_PIN       33
#define IMU_FREQ_HZ       400000
#define MPU6050_ADDR      0x68
#define MPU_REG_PWR       0x6B
#define MPU_REG_GYRO_CFG  0x1B
#define MPU_REG_ACCEL_CFG 0x1C
#define MPU_REG_ACCEL_H   0x3B
#define MPU_REG_GYRO_H    0x43
#define ACCEL_SCALE       16384.0f
#define GYRO_SCALE        131.0f
#define G_TO_MS2          9.80665f
#define DEG_TO_RAD        0.017453292519943f

/* ===========================================================
   ENCODERS — PCNT quadrature decoder
   Left  Motor Encoder:  A → D14 (green wire)
                         B → D13 (yellow wire)
   Right Motor Encoder:  A → D27
                         B → D26 (yellow wire)

   *** SET ENCODER_CPR TO YOUR MOTOR SPEC ***
   How to measure: spin one wheel exactly 1 full turn by hand,
   read /left_ticks before and after → difference = your CPR
=========================================================== */
#define LEFT_ENC_A      14
#define LEFT_ENC_B      13
#define RIGHT_ENC_A     27
#define RIGHT_ENC_B     26
#define ENCODER_CPR     1320        /* ADJUST to match your motor */
#define PCNT_HIGH_LIM   30000
#define PCNT_LOW_LIM   -30000

/* ===========================================================
   MOTORS — TB6612FNG
   PWMA → D19    AIN1 → D5     AIN2 → D18
   PWMB → D23    BIN1 → D21    BIN2 → D22
   STBY → D4
=========================================================== */
#define PIN_ENA     19              /* PWMA */
#define PIN_ENB     23              /* PWMB */
#define PIN_IN1     18              /* AIN2 */
#define PIN_IN2      5              /* AIN1 */
#define PIN_IN3     21              /* BIN1 */
#define PIN_IN4     22              /* BIN2 */
#define PIN_STBY     4
#define LED_BUILTIN  2
#define PWM_MODE    LEDC_LOW_SPEED_MODE
#define PWM_TIMER   LEDC_TIMER_0
#define PWM_CH_L    LEDC_CHANNEL_0
#define PWM_CH_R    LEDC_CHANNEL_1
#define PWM_FREQ    5000
#define PWM_RES     LEDC_TIMER_12_BIT
#define PWM_MIN     400
#define PWM_MAX     4095

/* ===========================================================
   GLOBALS
=========================================================== */
static EventGroupHandle_t wifi_eg;
#define WIFI_GOT_IP BIT0

static SemaphoreHandle_t tof_mutex;
static int32_t           g_tof_mm = 0;

static SemaphoreHandle_t imu_mutex;
typedef struct { float ax, ay, az, gx, gy, gz; } ImuData_t;
static ImuData_t g_imu = {0};

static SemaphoreHandle_t enc_mutex;
static int32_t g_left_ticks  = 0;
static int32_t g_right_ticks = 0;

static pcnt_unit_handle_t enc_left_unit  = NULL;
static pcnt_unit_handle_t enc_right_unit = NULL;

/* ===========================================================
   MPU6050 I2C HELPERS
=========================================================== */
static esp_err_t mpu_write(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(IMU_I2C_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r;
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(IMU_I2C_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r;
}

static void imu_init(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = IMU_SDA_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = IMU_SCL_PIN,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = IMU_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(IMU_I2C_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(IMU_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_ERROR_CHECK(mpu_write(MPU_REG_PWR,       0x00));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(mpu_write(MPU_REG_GYRO_CFG,  0x00));
    ESP_ERROR_CHECK(mpu_write(MPU_REG_ACCEL_CFG, 0x00));
    ESP_LOGI(TAG, "MPU6050 ready SDA=D%d SCL=D%d", IMU_SDA_PIN, IMU_SCL_PIN);
}

/* ===========================================================
   ENCODER INIT — PCNT quadrature (full 4x resolution)
=========================================================== */
static void encoder_init_unit(pcnt_unit_handle_t *unit, int enc_a, int enc_b) {
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HIGH_LIM,
        .low_limit  = PCNT_LOW_LIM,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, unit));

    pcnt_glitch_filter_config_t filt = { .max_glitch_ns = 3000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*unit, &filt));

    /* Channel A counts edges, direction from B */
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = enc_a,
        .level_gpio_num = enc_b,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*unit, &chan_a_cfg, &chan_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Channel B counts edges, direction from A */
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = enc_b,
        .level_gpio_num = enc_a,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*unit, &chan_b_cfg, &chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(*unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*unit));
    ESP_ERROR_CHECK(pcnt_unit_start(*unit));
}

static void encoders_init(void) {
    encoder_init_unit(&enc_left_unit,  LEFT_ENC_A,  LEFT_ENC_B);
    encoder_init_unit(&enc_right_unit, RIGHT_ENC_A, RIGHT_ENC_B);
    ESP_LOGI(TAG, "Encoders ready  L=A%d/B%d  R=A%d/B%d  CPR=%d",
             LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B, ENCODER_CPR);
}

/* ===========================================================
   ENCODER READER TASK — 50 Hz
   Reads PCNT hardware counter, accumulates into 32-bit total
   to handle the 16-bit PCNT overflow automatically
=========================================================== */
static void encoder_reader_task(void *arg) {
    int     left_raw  = 0, right_raw  = 0;
    int     left_prev = 0, right_prev = 0;
    int32_t left_total  = 0;
    int32_t right_total = 0;

    while (1) {
        pcnt_unit_get_count(enc_left_unit,  &left_raw);
        pcnt_unit_get_count(enc_right_unit, &right_raw);

        left_total  += (left_raw  - left_prev);
        right_total += (right_raw - right_prev);
        left_prev  = left_raw;
        right_prev = right_raw;

        xSemaphoreTake(enc_mutex, portMAX_DELAY);
        g_left_ticks  = left_total;
        g_right_ticks = right_total;
        xSemaphoreGive(enc_mutex);

        vTaskDelay(pdMS_TO_TICKS(20));  /* 50 Hz */
    }
}

/* ===========================================================
   IMU READER TASK — 50 Hz
=========================================================== */
static void imu_reader_task(void *arg) {
    uint8_t buf[14];
    while (1) {
        if (mpu_read(MPU_REG_ACCEL_H, buf, 14) == ESP_OK) {
            int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
            int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
            int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
            int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
            int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
            int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
            ImuData_t d;
            d.ax = (float)ax / ACCEL_SCALE * G_TO_MS2;
            d.ay = (float)ay / ACCEL_SCALE * G_TO_MS2;
            d.az = (float)az / ACCEL_SCALE * G_TO_MS2;
            d.gx = (float)gx / GYRO_SCALE  * DEG_TO_RAD;
            d.gy = (float)gy / GYRO_SCALE  * DEG_TO_RAD;
            d.gz = (float)gz / GYRO_SCALE  * DEG_TO_RAD;
            xSemaphoreTake(imu_mutex, portMAX_DELAY);
            g_imu = d;
            xSemaphoreGive(imu_mutex);
        } else {
            ESP_LOGW(TAG, "MPU6050 read fail");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ===========================================================
   TOF UART INIT + READER TASK
=========================================================== */
static void tof_uart_init(void) {
    uart_config_t cfg = {
        .baud_rate  = TOF_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(TOF_UART_NUM,
                                        TOF_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(TOF_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(TOF_UART_NUM,
                                  TOF_TX_PIN, TOF_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "TOFSense UART2  ESP32-RX=D%d(←TOF-TX)  ESP32-TX=D%d(→TOF-RX)",
             TOF_RX_PIN, TOF_TX_PIN);
}

static bool tof_decode(uint8_t *buf, int32_t *mm) {
    if (buf[0] != TOF_FRAME_HDR || buf[1] != TOF_FUNC_MARK) return false;
    uint8_t chk = 0;
    for (int i = 0; i < 15; i++) chk += buf[i];
    if (chk != buf[15]) return false;
    if (buf[11] == 0)   return false;
    *mm = (int32_t)((long)(((uint32_t)buf[10] << 24) |
                            ((uint32_t)buf[9]  << 16) |
                            ((uint32_t)buf[8]  <<  8)) / 256);
    return true;
}

static void tof_reader_task(void *arg) {
    uint8_t byte, frame[TOF_FRAME_LEN];
    int idx = 0;
    while (1) {
        int len = uart_read_bytes(TOF_UART_NUM, &byte, 1, pdMS_TO_TICKS(10));
        if (len <= 0) continue;
        if (idx == 0 && byte != TOF_FRAME_HDR) continue;
        frame[idx++] = byte;
        if (idx == TOF_FRAME_LEN) {
            int32_t d = 0;
            if (tof_decode(frame, &d)) {
                xSemaphoreTake(tof_mutex, portMAX_DELAY);
                g_tof_mm = d;
                xSemaphoreGive(tof_mutex);
            }
            idx = 0;
        }
    }
}

/* ===========================================================
   MOTOR CONTROL — TB6612FNG
=========================================================== */
static void motors_init(void) {
    gpio_config_t io = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_IN1)  | (1ULL << PIN_IN2)  |
                        (1ULL << PIN_IN3)  | (1ULL << PIN_IN4)  |
                        (1ULL << PIN_STBY) | (1ULL << LED_BUILTIN),
    };
    gpio_config(&io);
    gpio_set_level(PIN_STBY, 1);

    ledc_timer_config_t t = {
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ,
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .clk_cfg         = LEDC_USE_APB_CLK,
    };
    ledc_timer_config(&t);

    ledc_channel_config_t cl = {
        .channel    = PWM_CH_L, .duty = 0,
        .gpio_num   = PIN_ENA,  .speed_mode = PWM_MODE,
        .hpoint     = 0,        .timer_sel  = PWM_TIMER,
    };
    ledc_channel_config(&cl);

    ledc_channel_config_t cr = {
        .channel    = PWM_CH_R, .duty = 0,
        .gpio_num   = PIN_ENB,  .speed_mode = PWM_MODE,
        .hpoint     = 0,        .timer_sel  = PWM_TIMER,
    };
    ledc_channel_config(&cr);

    ESP_LOGI(TAG, "TB6612 ready  ENA=D%d ENB=D%d IN1=D%d IN2=D%d IN3=D%d IN4=D%d STBY=D%d",
             PIN_ENA, PIN_ENB, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_STBY);
}

static float fmap(float v, float imin, float imax, float omin, float omax) {
    return (v - imin) * (omax - omin) / (imax - imin) + omin;
}

static void motors_set(float left, float right) {
    left  = constrain(left,  -1.0f, 1.0f);
    right = constrain(right, -1.0f, 1.0f);

    uint16_t pwl = (fabsf(left)  < 0.05f) ? 0
                 : (uint16_t)fmap(fabsf(left),  0, 1, PWM_MIN, PWM_MAX);
    uint16_t pwr = (fabsf(right) < 0.05f) ? 0
                 : (uint16_t)fmap(fabsf(right), 0, 1, PWM_MIN, PWM_MAX);

    if      (left  >  0.05f) { gpio_set_level(PIN_IN1, 1); gpio_set_level(PIN_IN2, 0); }
    else if (left  < -0.05f) { gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 1); }
    else                     { gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 0); }

    if      (right >  0.05f) { gpio_set_level(PIN_IN3, 1); gpio_set_level(PIN_IN4, 0); }
    else if (right < -0.05f) { gpio_set_level(PIN_IN3, 0); gpio_set_level(PIN_IN4, 1); }
    else                     { gpio_set_level(PIN_IN3, 0); gpio_set_level(PIN_IN4, 0); }

    ledc_set_duty(PWM_MODE, PWM_CH_L, pwl); ledc_update_duty(PWM_MODE, PWM_CH_L);
    ledc_set_duty(PWM_MODE, PWM_CH_R, pwr); ledc_update_duty(PWM_MODE, PWM_CH_R);
}

static void test_motors(void) {
    ESP_LOGI(TAG, "=== MOTOR TEST: SPINNING FORWARD 50%% ===");
    motors_set(0.5f, 0.5f);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "=== MOTOR TEST: STOP ===");
    motors_set(0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
}

/* ===========================================================
   WIFI
=========================================================== */
static void wifi_event_handler(void *a, esp_event_base_t b,
                                int32_t id, void *d) {
    if      (b == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (b == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        motors_set(0, 0);
        esp_wifi_connect();
    } else if (b == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi IP: " IPSTR,
                 IP2STR(&((ip_event_got_ip_t *)d)->ip_info.ip));
        xEventGroupSetBits(wifi_eg, WIFI_GOT_IP);
    }
}

static void wifi_init(void) {
    wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t h1, h2;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         wifi_event_handler, NULL, &h1);
    esp_event_handler_instance_register(IP_EVENT,   IP_EVENT_STA_GOT_IP,
                                         wifi_event_handler, NULL, &h2);
    wifi_config_t wc = { .sta = {
        .ssid     = CONFIG_ESP_WIFI_SSID,
        .password = CONFIG_ESP_WIFI_PASSWORD,
    }};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    xEventGroupWaitBits(wifi_eg, WIFI_GOT_IP, pdFALSE, pdTRUE,
                         pdMS_TO_TICKS(30000));
}

/* ===========================================================
   micro-ROS
=========================================================== */
static geometry_msgs__msg__Twist cmd_msg;
static std_msgs__msg__Int32      tof_msg;
static std_msgs__msg__Int32      left_ticks_msg;
static std_msgs__msg__Int32      right_ticks_msg;
static sensor_msgs__msg__Imu     imu_msg;

static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *t =
        (const geometry_msgs__msg__Twist *)msgin;
    float lin = constrain((float)t->linear.x,  -1.0f, 1.0f);
    float ang = constrain((float)t->angular.z, -1.0f, 1.0f);
    
    ESP_LOGI(TAG, "CMD_VEL received: lin=%.2f ang=%.2f", lin, ang);
    
    motors_set(lin - ang, lin + ang);
}

static void timer_callback(rcl_timer_t *t, int64_t lct) {
    (void)lct;
    if (!t) return;
    gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
}

static void micro_ros_task(void *arg) {
    rcl_allocator_t    alloc   = rcl_get_default_allocator();
    rclc_support_t     support = {0};
    rcl_init_options_t opts    = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&opts, alloc));

#ifdef CONFIG_MICRO_ROS_ESP_NETIF_WLAN
    rmw_init_options_t *rmw = rcl_init_options_get_rmw_init_options(&opts);
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT, rmw));
#endif

    esp_err_t agent_connected = ESP_FAIL;
    while (agent_connected != ESP_OK) {
        rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &opts, &alloc);
        if (rc == RCL_RET_OK) {
            agent_connected = ESP_OK;
            ESP_LOGI(TAG, "Connected to micro-ROS agent successfully!");
        } else {
            ESP_LOGW(TAG, "Failed to connect to agent, retrying in 2 seconds...");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "esp32_robot", "", &support));

    rcl_publisher_t tof_pub   = rcl_get_zero_initialized_publisher();
    rcl_publisher_t imu_pub   = rcl_get_zero_initialized_publisher();
    rcl_publisher_t left_pub  = rcl_get_zero_initialized_publisher();
    rcl_publisher_t right_pub = rcl_get_zero_initialized_publisher();

    RCCHECK(rclc_publisher_init_default(&tof_pub,   &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,    msg, Int32), "tof_distance"));
    RCCHECK(rclc_publisher_init_default(&imu_pub,   &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),   "imu/data"));
    RCCHECK(rclc_publisher_init_default(&left_pub,  &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,    msg, Int32), "left_ticks"));
    RCCHECK(rclc_publisher_init_default(&right_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,    msg, Int32), "right_ticks"));

    rcl_subscription_t cmd_sub = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default(&timer, &support,
                                     RCL_MS_TO_NS(500), timer_callback));

    rclc_executor_t ex = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&ex, &support.context, 2, &alloc));
    RCCHECK(rclc_executor_add_subscription(&ex, &cmd_sub, &cmd_msg,
                                            &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&ex, &timer));

    imu_msg.header.frame_id.data     = "imu_link";
    imu_msg.header.frame_id.size     = 8;
    imu_msg.header.frame_id.capacity = 9;
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i]         = -1;
        imu_msg.angular_velocity_covariance[i]    =  0;
        imu_msg.linear_acceleration_covariance[i] =  0;
    }
    imu_msg.angular_velocity_covariance[0]    = 0.002f;
    imu_msg.angular_velocity_covariance[4]    = 0.002f;
    imu_msg.angular_velocity_covariance[8]    = 0.002f;
    imu_msg.linear_acceleration_covariance[0] = 0.017f;
    imu_msg.linear_acceleration_covariance[4] = 0.017f;
    imu_msg.linear_acceleration_covariance[8] = 0.017f;
    imu_msg.orientation.w = 1.0;

    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, " ESP32 ROBOT v4.0 READY");
    ESP_LOGI(TAG, " TOF  : RX=D%d(←TOFTX) TX=D%d(→TOFRX)", TOF_RX_PIN, TOF_TX_PIN);
    ESP_LOGI(TAG, " IMU  : SDA=D%d SCL=D%d", IMU_SDA_PIN, IMU_SCL_PIN);
    ESP_LOGI(TAG, " ENC-L: A=D%d B=D%d", LEFT_ENC_A,  LEFT_ENC_B);
    ESP_LOGI(TAG, " ENC-R: A=D%d B=D%d", RIGHT_ENC_A, RIGHT_ENC_B);
    ESP_LOGI(TAG, " CPR  : %d", ENCODER_CPR);
    ESP_LOGI(TAG, " PUB  : /tof_distance /imu/data /left_ticks /right_ticks");
    ESP_LOGI(TAG, " SUB  : /cmd_vel");
    ESP_LOGI(TAG, "==============================");

    while (1) {
        int64_t ts_ns = (int64_t)(esp_timer_get_time() * 1000LL);

        /* TOF */
        xSemaphoreTake(tof_mutex, portMAX_DELAY);
        tof_msg.data = g_tof_mm;
        xSemaphoreGive(tof_mutex);
        if (tof_msg.data > 0)
            RCSOFTCHECK(rcl_publish(&tof_pub, &tof_msg, NULL));

        /* IMU */
        xSemaphoreTake(imu_mutex, portMAX_DELAY);
        ImuData_t imu = g_imu;
        xSemaphoreGive(imu_mutex);
        imu_msg.header.stamp.sec     = (int32_t)(ts_ns / 1000000000LL);
        imu_msg.header.stamp.nanosec = (uint32_t)(ts_ns % 1000000000LL);
        imu_msg.angular_velocity.x   = imu.gx;
        imu_msg.angular_velocity.y   = imu.gy;
        imu_msg.angular_velocity.z   = imu.gz;
        imu_msg.linear_acceleration.x = imu.ax;
        imu_msg.linear_acceleration.y = imu.ay;
        imu_msg.linear_acceleration.z = imu.az;
        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));

        /* Encoder ticks */
        xSemaphoreTake(enc_mutex, portMAX_DELAY);
        left_ticks_msg.data  = g_left_ticks;
        right_ticks_msg.data = g_right_ticks;
        xSemaphoreGive(enc_mutex);
        RCSOFTCHECK(rcl_publish(&left_pub,  &left_ticks_msg,  NULL));
        RCSOFTCHECK(rcl_publish(&right_pub, &right_ticks_msg, NULL));

        rclc_executor_spin_some(&ex, RCL_MS_TO_NS(10));
        usleep(20000);  /* 50 Hz */
    }
}

/* ===========================================================
   app_main
=========================================================== */
void app_main(void) {
    ESP_LOGI(TAG, "Booting ESP32 Robot v4.0");
    ESP_ERROR_CHECK(nvs_flash_init());

    tof_mutex = xSemaphoreCreateMutex();
    imu_mutex = xSemaphoreCreateMutex();
    enc_mutex = xSemaphoreCreateMutex();

    tof_uart_init();
    imu_init();
    encoders_init();
    motors_init();
    test_motors();
    wifi_init();

    xTaskCreatePinnedToCore(tof_reader_task,     "tof",     4096, NULL, 7, NULL, 0);
    xTaskCreatePinnedToCore(imu_reader_task,     "imu",     4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(encoder_reader_task, "encoder", 4096, NULL, 6, NULL, 0);

    ESP_LOGI(TAG, "Waiting 3s for micro-ROS agent...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    xTaskCreatePinnedToCore(micro_ros_task, "micro_ros", 12288, NULL, 5, NULL, 1);
}
