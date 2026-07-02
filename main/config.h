#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 全局项目配置 —— 智绘灵境 AR Interactive Sandbox
 * ======================================================================== */

/* ------------------------------ 调试开关 ------------------------------ */
#define AR_DEBUG_ENABLE         1       // 总调试开关
#define AR_DEBUG_PHYSICS        1       // 物理引擎调试日志
#define AR_DEBUG_DETECTION      1       // 目标检测调试日志
#define AR_DEBUG_RENDER         0       // 渲染调试日志

/* ------------------------------ 相机配置 ------------------------------ */
#define CAMERA_H_RES            800     // 相机水平分辨率
#define CAMERA_V_RES            640     // 相机垂直分辨率
/* 注: MIPI lane数、比特率、ISP时钟、LDO、SCCB地址等硬件参数
 *     现在由 esp_video 框架和 Kconfig 统一管理，不再需要在此硬编码 */

/* ------------------------------ 检测模块配置 ------------------------------ */
#define DETECTION_MODE          3       // 0=固定测试数据, 1=简单颜色检测, 3=ESP-DL
#define DETECTION_MAX_OBJECTS   10      // 最多检测物体数
#define DETECTION_CONFIDENCE    0.5f    // 置信度阈值(ESP-DL模式)

/* ------------------------------ 游戏地图配置 ------------------------------ */
#define MAP_SIZE                640     // 游戏地图实际尺寸(像素,正方形)
#define MAP_DISPLAY_SIZE        640     // 地图在屏幕上的显示尺寸
#define MAP_DISPLAY_X           ((720 - MAP_DISPLAY_SIZE) / 2)  // X偏移,居中
#define MAP_DISPLAY_Y           20      // 上方边距(像素)

/* ------------------------------ 弹珠配置 ------------------------------ */
#define MARBLE_RADIUS           12      // 弹珠半径(地图像素)
#define MARBLE_INIT_X           (MAP_SIZE / 2.0f)  // 初始X(地图中心)
#define MARBLE_INIT_Y           (MAP_SIZE * 0.2f)  // 初始Y(地图上方1/5处)

/* ------------------------------ 物理引擎配置 ------------------------------ */
#define PHYSICS_UPDATE_HZ       100     // 物理更新频率
#define PIXELS_PER_METER        1067    // 像素/米换算(桌面约0.6m宽→640px)
#define GRAVITY_MAGNITUDE       (9.8f * PIXELS_PER_METER)  // 重力加速度(像素/s²)
#define GLOBAL_FRICTION         0.05f   // 全局摩擦系数(大理石桌面)
#define GOAL_THRESHOLD          20.0f   // 到达终点的判定距离(像素)
#define MARBLE_MAX_SPEED        2000.0f // 弹珠最大速度限制(像素/s)

/* ------------------------------ IMU配置 ------------------------------ */
#define IMU_ENABLED             0       // 是否启用真实IMU (阶段6设为1)
#define IMU_I2C_SCL_IO          -1      // 占位, 实际连接后修改
#define IMU_I2C_SDA_IO          -1      // 占位
#define IMU_I2C_PORT            0       // 独立I2C端口
#define IMU_I2C_ADDR            0x68    // MPU6050默认地址
#define IMU_I2C_FREQ_HZ         400000  // 400kHz
#define IMU_FILTER_ALPHA        0.98f   // 互补滤波系数

#ifdef __cplusplus
}
#endif
