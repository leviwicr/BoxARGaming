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
 * @brief 初始化IMU（I2C共享触摸屏总线）
 * @return ESP_OK 成功
 */
esp_err_t imu_init(void);

/**
 * @brief 读取IMU原始数据
 */
esp_err_t imu_read_raw(imu_raw_data_t *data);

/**
 * @brief 获取设备倾斜角度（经过互补滤波）
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
