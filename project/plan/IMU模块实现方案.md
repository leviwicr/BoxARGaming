# IMU模块实现方案（占位 + MPU6050接口预留）

## 模块目标

读取MPU6050六轴IMU传感器数据，解算设备倾斜角度，输出重力在地图平面上的加速度分量。

**当前优先级：低（优先完成视觉和显示部分）**

## 硬件连接

- **芯片**：MPU6050（六轴：三轴加速度计 + 三轴陀螺仪）
- **接口**：I2C（独立于触摸屏的I2C总线）
- **I2C地址**：0x68（AD0引脚接地）或 0x69（AD0接VCC）
- **引脚**：需确认实际硬件连接（暂用占位宏定义）

```c
#define IMU_I2C_SCL_IO      GPIO_NUM_XX    // 根据实际连接修改
#define IMU_I2C_SDA_IO      GPIO_NUM_XX    // 根据实际连接修改
#define IMU_I2C_PORT        I2C_NUM_1      // 独立I2C端口
#define IMU_I2C_ADDR        0x68           // MPU6050默认地址
#define IMU_I2C_FREQ_HZ     400000         // 400kHz Fast Mode
```

## 接口定义

```c
// imu_driver.h

#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** IMU原始数据 */
typedef struct {
    float accel_x, accel_y, accel_z;    // 加速度 (m/s²)
    float gyro_x, gyro_y, gyro_z;       // 角速度 (deg/s)
} imu_raw_data_t;

/** 倾斜角度 */
typedef struct {
    float pitch;    // 俯仰角 (度)，前后倾斜
    float roll;     // 横滚角 (度)，左右倾斜
} imu_attitude_t;

/**
 * @brief 初始化IMU（MPU6050）
 * @return ESP_OK 成功
 */
esp_err_t imu_init(void);

/**
 * @brief 读取IMU原始数据
 */
esp_err_t imu_read_raw(imu_raw_data_t *data);

/**
 * @brief 获取设备倾斜角度（经过卡尔曼滤波）
 * @param attitude 输出倾斜角度
 * @return ESP_OK 成功
 */
esp_err_t imu_get_attitude(imu_attitude_t *attitude);

/**
 * @brief 获取重力在平面上的加速度分量（用于物理引擎）
 * @param ax, ay 输出加速度分量（像素/s²）
 * @note 此函数组合了IMU读取+姿态解算+坐标映射
 */
esp_err_t imu_get_tilt_accel(float *ax, float *ay);

/**
 * @brief 反初始化IMU
 */
void imu_deinit(void);

#ifdef __cplusplus
}
#endif
```

## MPU6050 驱动实现要点

### 初始化步骤

```c
esp_err_t imu_init(void)
{
    // 1. 初始化I2C总线
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = IMU_I2C_PORT,
        .scl_io_num = IMU_I2C_SCL_IO,
        .sda_io_num = IMU_I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&i2c_cfg, &bus_handle);

    // 2. 添加MPU6050设备
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);

    // 3. 唤醒MPU6050（退出睡眠模式）
    uint8_t data[2];
    data[0] = 0x6B;  // PWR_MGMT_1 寄存器
    data[1] = 0x00;  // 退出睡眠，选择内部振荡器
    i2c_master_transmit(dev_handle, data, 2, 100);

    // 4. 配置加速度计量程 ±2g
    data[0] = 0x1C;  // ACCEL_CONFIG 寄存器
    data[1] = 0x00;  // ±2g
    i2c_master_transmit(dev_handle, data, 2, 100);

    // 5. 配置陀螺仪量程 ±250°/s
    data[0] = 0x1B;  // GYRO_CONFIG 寄存器
    data[1] = 0x00;  // ±250°/s
    i2c_master_transmit(dev_handle, data, 2, 100);

    return ESP_OK;
}
```

### 读取数据

```c
esp_err_t imu_read_raw(imu_raw_data_t *data)
{
    // MPU6050 从 ACCEL_XOUT_H (0x3B) 开始连续读取 14 字节
    // [0:1]=AX, [2:3]=AY, [4:5]=AZ, [6:7]=TEMP
    // [8:9]=GX, [10:11]=GY, [12:13]=GZ

    uint8_t reg = 0x3B;
    uint8_t buf[14];
    i2c_master_transmit_receive(dev_handle, &reg, 1, buf, 14, 100);

    // 解析（大端字节序，有符号16位）
    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];

    // 转换为物理单位
    // ±2g → 16384 LSB/g, ±250°/s → 131 LSB/(°/s)
    data->accel_x = ax / 16384.0f * 9.8f;
    data->accel_y = ay / 16384.0f * 9.8f;
    data->accel_z = az / 16384.0f * 9.8f;
    data->gyro_x = gx / 131.0f;
    data->gyro_y = gy / 131.0f;
    data->gyro_z = gz / 131.0f;

    return ESP_OK;
}
```

## 姿态解算（简化版互补滤波）

```c
// 从加速度计估算角度
static void accel_to_angle(float ax, float ay, float az,
                            float *pitch, float *roll)
{
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    *roll  = atan2f(ay, az) * 180.0f / M_PI;
}

// 互补滤波：融合加速度计（低频可靠）和陀螺仪（高频响应快）
static float pitch_angle = 0, roll_angle = 0;

esp_err_t imu_get_attitude(imu_attitude_t *attitude)
{
    imu_raw_data_t raw;
    imu_read_raw(&raw);

    // 加速度计角度
    float accel_pitch, accel_roll;
    accel_to_angle(raw.accel_x, raw.accel_y, raw.accel_z,
                    &accel_pitch, &accel_roll);

    // 互补滤波 (α=0.98 偏重陀螺仪)
    static uint64_t last_time = 0;
    uint64_t now = esp_timer_get_time();
    float dt = (now - last_time) / 1000000.0f;
    last_time = now;

    const float alpha = 0.98f;
    pitch_angle = alpha * (pitch_angle + raw.gyro_x * dt) + (1 - alpha) * accel_pitch;
    roll_angle  = alpha * (roll_angle + raw.gyro_y * dt) + (1 - alpha) * accel_roll;

    attitude->pitch = pitch_angle;
    attitude->roll = roll_angle;
    return ESP_OK;
}
```

## 重力分量到物理引擎加速度的映射

```c
esp_err_t imu_get_tilt_accel(float *ax, float *ay)
{
    imu_attitude_t att;
    ESP_ERROR_CHECK(imu_get_attitude(&att));

    // 倾斜角度 → 重力在地图平面上的分量
    // x: 左右倾斜 (roll)
    // y: 前后倾斜 (pitch)
    // 限制角度范围
    float clamped_roll = fminf(fmaxf(att.roll, -45.0f), 45.0f);
    float clamped_pitch = fminf(fmaxf(att.pitch, -45.0f), 45.0f);

    // sinθ ≈ θ/45 在±45°范围内近似（或直接用sin）
    float grav_x = sinf(clamped_roll * M_PI / 180.0f);
    float grav_y = sinf(clamped_pitch * M_PI / 180.0f);

    // 映射到像素加速度
    *ax = grav_x * GRAVITY_MAGNITUDE;
    *ay = grav_y * GRAVITY_MAGNITUDE;

    return ESP_OK;
}
```

## 卡尔曼滤波（可选升级）

互补滤波简单有效，适合大多数场景。如需更平滑的响应，可升级为一维卡尔曼滤波：

```c
typedef struct {
    float q;  // 过程噪声协方差
    float r;  // 测量噪声协方差
    float x;  // 状态估计值
    float p;  // 估计误差协方差
    float k;  // 卡尔曼增益
} kalman_1d_t;

static float kalman_update(kalman_1d_t *kf, float measurement)
{
    // 预测
    kf->p = kf->p + kf->q;
    // 更新
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;
    return kf->x;
}
```

## 阶段6前的占位实现

在IMU未连接时，提供模拟数据用于调试其他模块：

```c
// 编译开关
#if IMU_ENABLED

esp_err_t imu_get_tilt_accel(float *ax, float *ay)
{
    // 真实IMU实现
    ...
}

#else

// 占位：模拟正弦变化，用于测试
esp_err_t imu_get_tilt_accel(float *ax, float *ay)
{
    static float t = 0;
    t += 0.01f;
    *ax = sinf(t * 0.5f) * 500.0f;
    *ay = sinf(t * 0.3f) * 300.0f;
    return ESP_OK;
}

#endif
```

## 调试方法

1. **原始数据检查**：
```c
imu_raw_data_t raw;
imu_read_raw(&raw);
ESP_LOGI("IMU", "accel: %.2f %.2f %.2f | gyro: %.2f %.2f %.2f",
         raw.accel_x, raw.accel_y, raw.accel_z,
         raw.gyro_x, raw.gyro_y, raw.gyro_z);
```

2. **静止检测**：平放时 accel_z ≈ 9.8, accel_x ≈ 0, accel_y ≈ 0

3. **倾斜角度可视化**：在屏幕上显示实时倾斜角度值

4. **I2C通信检查**：读取 WHO_AM_I 寄存器 (0x75)，应返回 0x68

## MPU6050 关键寄存器速查表

| 寄存器 | 地址 | 说明 |
|--------|------|------|
| WHO_AM_I | 0x75 | 设备ID，应为0x68 |
| PWR_MGMT_1 | 0x6B | 电源管理，写0x00唤醒 |
| ACCEL_CONFIG | 0x1C | 加速度计量程配置 |
| GYRO_CONFIG | 0x1B | 陀螺仪量程配置 |
| ACCEL_XOUT_H | 0x3B | 加速度数据起始（共6字节） |
| GYRO_XOUT_H | 0x43 | 陀螺仪数据起始（共6字节） |

## config.h 中的 IMU 配置

```c
// ========== IMU 配置 ==========
#define IMU_ENABLED             0    // 是否启用真实IMU（阶段6设为1）
#define IMU_I2C_SCL_IO         -1    // 占位，实际连接后修改
#define IMU_I2C_SDA_IO         -1
#define IMU_I2C_PORT           1    // I2C_NUM_1 (独立于触摸的I2C_NUM_0)
#define IMU_I2C_ADDR           0x68
#define IMU_I2C_FREQ_HZ        400000
#define IMU_READ_RATE_HZ       100  // IMU读取频率
#define IMU_FILTER_ALPHA       0.98f // 互补滤波系数
```
