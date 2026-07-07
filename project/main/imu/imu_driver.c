/**
 * IMU驱动模块 — I2C通信 (独立I2C0总线, 不与其他设备共享)
 *
 * 姿态解算: 2状态卡尔曼滤波器 (角度 + 陀螺仪零偏), 每轴独立
 *
 * IMU模块 I2C地址: 0x23
 * 寄存器自动递增, 支持连续读取
 * 关键寄存器 (小端字节序):
 *   0x04-0x0F: ACCEL_X/Y/Z, GYRO_X/Y/Z (int16)
 *   0x16-0x25: 四元数 QUAT_W/X/Y/Z (float)
 *   0x26-0x31: 欧拉角 ROLL/PITCH/YAW (float, 弧度)
 */

#include "imu_driver.h"
#include "config.h"
#include <math.h>
#include <string.h>

#if IMU_ENABLED
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

static const char *TAG = "imu";

#define IMU_I2C_ADDR    0x23
#define IMU_REG_EULER   0x26    // 欧拉角起始地址
#define IMU_REG_RAW     0x04    // 原始传感器起始地址

static i2c_master_bus_handle_t  g_imu_bus = NULL;
static i2c_master_dev_handle_t  g_imu_dev = NULL;

/* ========================================================================
 * 2状态卡尔曼滤波器 — 每轴独立 (角度 + 陀螺仪零偏)
 *
 * 状态向量: X = [θ, b]^T
 * 预测步:  θ⁻ = θ + (ω - b)·Δt   (陀螺仪驱动)
 *           b⁻ = b                  (随机游走)
 * 更新步:  z = θ_accel             (加速度计反算角度)
 *
 * 相比旧版1D卡尔曼:
 *   1. 有过程模型 — 陀螺仪角速度直接驱动状态转移, 响应零延迟
 *   2. 在线估计零偏 — b 状态自动追踪温漂, 长期不累积误差
 *   3. 自适应增益 — 动态时 K→0 信任陀螺仪, 静态时 K→1 信任加速度计
 *   4. 统一数据路径 — 不再有"读欧拉角"和"互补滤波回退"两条割裂路径
 * ======================================================================== */
typedef struct {
    float theta;            // 状态: 角度 (度)
    float bias;             // 状态: 陀螺仪零偏 (度/秒)
    float p00, p01, p11;    // 误差协方差 (对称, 只存上三角)
    float q_theta;          // 过程噪声协方差: 角度
    float q_bias;           // 过程噪声协方差: 零偏随机游走
    float r;                // 观测噪声协方差
} kalman_2d_t;

static kalman_2d_t kf_pitch, kf_roll;
static uint64_t   g_last_time;

/* ---- 卡尔曼初始化 ---- */
static void kalman2d_init(kalman_2d_t *kf, float q_theta, float q_bias, float r)
{
    kf->theta   = 0;
    kf->bias    = 0;
    kf->p00     = 1.0f;
    kf->p01     = 0;
    kf->p11     = 1.0f;
    kf->q_theta = q_theta;
    kf->q_bias  = q_bias;
    kf->r       = r;
}

/**
 * 单步 预测+更新 (每轴独立)
 *
 * @param kf     滤波器实例
 * @param gyro   陀螺仪该轴角速度 (度/秒), 原始值, 内部会扣除零偏
 * @param meas   观测角度 (度), 来自加速度计反算 或 模块欧拉角
 * @param dt     距上次更新的时间 (秒)
 * @return 滤波后的角度 (度)
 */
static float kalman2d_step(kalman_2d_t *kf, float gyro, float meas, float dt)
{
    /* ---- 预测步: X⁻ = F·X,  P⁻ = F·P·Fᵀ + Q ---- */
    kf->theta = kf->theta + (gyro - kf->bias) * dt;
    // bias 不变: b⁻ = b

    /* P⁻ = F·P·Fᵀ + Q
     * F = [[1, -dt], [0, 1]]
     * F·P·Fᵀ = [[p00 - dt*(p01+p10) + dt²*p11,  p01 - dt*p11],
     *           [p10 - dt*p11,                   p11          ]]
     * 加上 Q = [[q_theta, 0], [0, q_bias]]
     */
    float p00 = kf->p00 - 2.0f * dt * kf->p01 + dt * dt * kf->p11 + kf->q_theta;
    float p01 = kf->p01 - dt * kf->p11;
    float p11 = kf->p11 + kf->q_bias;

    /* ---- 更新步: K = P⁻·Hᵀ / (H·P⁻·Hᵀ + R) ---- */
    /* H = [1, 0]  →  S = p00 + R,  K = [p00/S, p01/S] */
    float s = p00 + kf->r;
    float k0 = p00 / s;       // 角度增益
    float k1 = p01 / s;       // 零偏增益

    float innov = meas - kf->theta;
    kf->theta += k0 * innov;
    kf->bias  += k1 * innov;

    /* P = (I - K·H)·P⁻,  I-KH = [[1-k0, 0], [-k1, 1]] */
    kf->p00 = (1.0f - k0) * p00;
    kf->p01 = (1.0f - k0) * p01;
    float p10_new = p01 - k1 * p00;
    kf->p11 = p11 - k1 * p01;
    // 强制对称 — 抑制浮点累积不对称, 代价几乎为零
    kf->p01 = (kf->p01 + p10_new) * 0.5f;

    return kf->theta;
}

/* ---- 最新数据 (线程安全) ---- */
static imu_raw_data_t g_latest_raw;
static imu_attitude_t g_latest_att;
static bool           g_raw_valid = false;
static bool           g_att_valid = false;
static portMUX_TYPE   g_data_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---- 故障计数 (避免日志泛滥) ---- */
static int        g_fail_count   = 0;
static uint64_t   g_fail_logged  = 0;
#define FAIL_LOG_INTERVAL_US  5000000   /* 每5秒最多打印一次故障摘要 */

/* ---- I2C 读取寄存器块 ---- */
static esp_err_t imu_read_regs(uint8_t reg_addr, uint8_t *buf, size_t len)
{
    if (!g_imu_dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(g_imu_dev, &reg_addr, 1, buf, len, 100);
}

/* ---- 读取欧拉角 (0x26-0x31, 3×float 小端, 弧度) ---- */
static esp_err_t imu_read_euler(imu_attitude_t *att)
{
    uint8_t buf[12];
    esp_err_t ret = imu_read_regs(IMU_REG_EULER, buf, 12);
    if (ret != ESP_OK) return ret;

    float roll, pitch, yaw;
    memcpy(&roll,  buf,      4);
    memcpy(&pitch, buf + 4,  4);
    memcpy(&yaw,   buf + 8,  4);

    att->roll  = roll  * 180.0f / M_PI;
    att->pitch = pitch * 180.0f / M_PI;
    return ESP_OK;
}

/* ---- 读取原始传感器数据 (0x04-0x0F, 6×int16 小端) ---- */
static esp_err_t imu_read_raw_sensors(imu_raw_data_t *data)
{
    uint8_t buf[12];
    esp_err_t ret = imu_read_regs(IMU_REG_RAW, buf, 12);
    if (ret != ESP_OK) return ret;

    int16_t ax = (int16_t)(buf[0]  | (buf[1]  << 8));
    int16_t ay = (int16_t)(buf[2]  | (buf[3]  << 8));
    int16_t az = (int16_t)(buf[4]  | (buf[5]  << 8));
    int16_t gx = (int16_t)(buf[6]  | (buf[7]  << 8));
    int16_t gy = (int16_t)(buf[8]  | (buf[9]  << 8));
    int16_t gz = (int16_t)(buf[10] | (buf[11] << 8));

    // 量程: ACCEL=±16g/32767, GYRO=±2000°/s/32767
    data->accel_x = ax * 16.0f / 32767.0f * 9.8f;
    data->accel_y = ay * 16.0f / 32767.0f * 9.8f;
    data->accel_z = az * 16.0f / 32767.0f * 9.8f;
    data->gyro_x  = gx * 2000.0f / 32767.0f;
    data->gyro_y  = gy * 2000.0f / 32767.0f;
    data->gyro_z  = gz * 2000.0f / 32767.0f;
    return ESP_OK;
}

/* ---- 从加速度计反算角度 (度) ---- */
static float accel_to_angle(float ax, float ay, float az, int is_pitch)
{
    if (is_pitch) {
        // pitch = atan2(-ax, sqrt(ay² + az²))
        return atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    } else {
        // roll = atan2(ay, az)
        return atan2f(ay, az) * 180.0f / M_PI;
    }
}

/* ---- 读取版本号校验I2C通信 ---- */
static esp_err_t imu_check_version(void)
{
    uint8_t ver_h, ver_m, ver_l;
    esp_err_t ret;
    ret = imu_read_regs(0x01, &ver_h, 1);
    if (ret != ESP_OK) return ret;
    ret = imu_read_regs(0x02, &ver_m, 1);
    if (ret != ESP_OK) return ret;
    ret = imu_read_regs(0x03, &ver_l, 1);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "IMU firmware v%d.%d.%d", ver_h, ver_m, ver_l);
    return ESP_OK;
}

/* ========================================================================
 * 公开 API
 * ======================================================================== */

esp_err_t imu_init(void)
{
    ESP_LOGI(TAG, "Initializing IMU on dedicated I2C0 (SCL=GPIO%d, SDA=GPIO%d)...",
             IMU_I2C_SCL_IO, IMU_I2C_SDA_IO);

    // 1. 创建独立的IMU I2C总线 (I2C0, 不与触摸屏/相机共享)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port    = I2C_NUM_0,
        .sda_io_num  = IMU_I2C_SDA_IO,
        .scl_io_num  = IMU_I2C_SCL_IO,
        .clk_source  = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &g_imu_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C0 bus create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 2. 添加IMU设备到总线
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = IMU_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    ret = i2c_master_bus_add_device(g_imu_bus, &dev_cfg, &g_imu_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
        i2c_del_master_bus(g_imu_bus);
        g_imu_bus = NULL;
        return ret;
    }

    // 3. 验证I2C通信（读版本号, 重试最多3次）
    for (int attempt = 0; attempt < 3; attempt++) {
        ret = imu_check_version();
        if (ret == ESP_OK) break;
        ESP_LOGW(TAG, "Version check attempt %d/3: %s", attempt + 1,
                 esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU not responding at 0x%02X on I2C0 (SCL=%d,SDA=%d)",
                 IMU_I2C_ADDR, IMU_I2C_SCL_IO, IMU_I2C_SDA_IO);
        i2c_master_bus_rm_device(g_imu_dev);
        i2c_del_master_bus(g_imu_bus);
        g_imu_dev = NULL;
        g_imu_bus = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    // 4. 初始化2状态卡尔曼滤波器
    kalman2d_init(&kf_pitch, 0.001f, 0.003f, 0.03f);
    kalman2d_init(&kf_roll,  0.001f, 0.003f, 0.03f);
    g_last_time = esp_timer_get_time();

    ESP_LOGI(TAG, "IMU init complete (2-state Kalman filter)");
    return ESP_OK;
}

esp_err_t imu_read_raw(imu_raw_data_t *data)
{
    imu_raw_data_t raw;
    esp_err_t ret = imu_read_raw_sensors(&raw);
    if (ret != ESP_OK) return ret;

    portENTER_CRITICAL(&g_data_lock);
    g_latest_raw = raw;
    g_raw_valid = true;
    portEXIT_CRITICAL(&g_data_lock);

    *data = raw;
    return ESP_OK;
}

esp_err_t imu_get_attitude(imu_attitude_t *attitude)
{
    uint64_t now = esp_timer_get_time();
    float dt = (now - g_last_time) / 1000000.0f;
    g_last_time = now;
    if (dt <= 0 || dt > 1.0f) dt = 0.001f;

    /* 1. 优先读模块自带的融合欧拉角 (最可靠, 兼容旧行为) */
    imu_attitude_t euler;
    esp_err_t euler_ret = imu_read_euler(&euler);

    /* 2. 尝试读原始传感器 (为卡尔曼预测提供陀螺仪, 失败不致命) */
    imu_raw_data_t raw;
    esp_err_t raw_ret = imu_read_raw_sensors(&raw);
    bool raw_valid = (raw_ret == ESP_OK);

    /* 3. 确定观测值 */
    float meas_pitch, meas_roll;
    if (euler_ret == ESP_OK) {
        meas_pitch = euler.pitch;
        meas_roll  = euler.roll;
    } else if (raw_valid) {
        meas_pitch = accel_to_angle(raw.accel_x, raw.accel_y, raw.accel_z, 1);
        meas_roll  = accel_to_angle(raw.accel_x, raw.accel_y, raw.accel_z, 0);
    } else {
        /* 完全无法读取 */
        g_fail_count++;
        uint64_t now2 = esp_timer_get_time();
        if (now2 - g_fail_logged > FAIL_LOG_INTERVAL_US) {
            ESP_LOGW(TAG, "I2C dead for %d reads (euler=%s, raw=%s, dev=%s)",
                     g_fail_count,
                     esp_err_to_name(euler_ret),
                     esp_err_to_name(raw_ret),
                     g_imu_dev ? "ok" : "NULL");
            g_fail_logged = now2;
            if (g_fail_count > 100 && g_imu_bus) {
                i2c_master_bus_reset(g_imu_bus);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            g_fail_count = 0;
        }
        portENTER_CRITICAL(&g_data_lock);
        *attitude = g_latest_att;
        portEXIT_CRITICAL(&g_data_lock);
        return g_att_valid ? ESP_OK : ESP_FAIL;
    }

    g_fail_count = 0;

    /* 4. 卡尔曼滤波 */
    imu_attitude_t att;
    if (raw_valid) {
        att.pitch = kalman2d_step(&kf_pitch, raw.gyro_x, meas_pitch, dt);
        att.roll  = kalman2d_step(&kf_roll,  raw.gyro_y, meas_roll,  dt);
    } else {
        /* 无陀螺仪 → 仅观测更新 (退化为自适应低通) */
        att.pitch = kalman2d_step(&kf_pitch, 0, meas_pitch, dt);
        att.roll  = kalman2d_step(&kf_roll,  0, meas_roll,  dt);
    }

    portENTER_CRITICAL(&g_data_lock);
    g_latest_att = att;
    g_att_valid = true;
    portEXIT_CRITICAL(&g_data_lock);

    *attitude = att;
    return ESP_OK;
}

esp_err_t imu_get_tilt_accel(float *ax, float *ay)
{
    imu_attitude_t att;
    esp_err_t ret = imu_get_attitude(&att);
    if (ret != ESP_OK) return ret;

    float r = fminf(fmaxf(att.roll,  -45.0f), 45.0f);
    float p = fminf(fmaxf(att.pitch, -45.0f), 45.0f);

    *ax = sinf(r * M_PI / 180.0f) * GRAVITY_MAGNITUDE;
    *ay = sinf(p * M_PI / 180.0f) * GRAVITY_MAGNITUDE;
    return ESP_OK;
}

void imu_deinit(void)
{
    if (g_imu_dev) {
        i2c_master_bus_rm_device(g_imu_dev);
        g_imu_dev = NULL;
    }
    if (g_imu_bus) {
        i2c_del_master_bus(g_imu_bus);
        g_imu_bus = NULL;
    }
    g_raw_valid = false;
    g_att_valid = false;
}

#else  /* !IMU_ENABLED */

/* ---- 占位实现 (模拟数据) ---- */
esp_err_t imu_init(void) { return ESP_OK; }

esp_err_t imu_read_raw(imu_raw_data_t *data)
{
    static float t = 0; t += 0.01f;
    data->accel_x = sinf(t * 0.5f) * 2.0f;
    data->accel_y = sinf(t * 0.3f) * 1.5f;
    data->accel_z = 9.8f;
    data->gyro_x = sinf(t * 0.5f) * 10.0f;
    data->gyro_y = sinf(t * 0.3f) * 8.0f;
    data->gyro_z = 0;
    return ESP_OK;
}

esp_err_t imu_get_attitude(imu_attitude_t *attitude)
{
    static float t = 0; t += 0.01f;
    attitude->pitch = sinf(t * 0.3f) * 15.0f;
    attitude->roll  = sinf(t * 0.5f) * 10.0f;
    return ESP_OK;
}

esp_err_t imu_get_tilt_accel(float *ax, float *ay)
{
    static float t = 0; t += 0.01f;
    *ax = sinf(t * 0.5f) * 500.0f;
    *ay = sinf(t * 0.3f) * 300.0f;
    return ESP_OK;
}

void imu_deinit(void) {}

#endif /* IMU_ENABLED */
