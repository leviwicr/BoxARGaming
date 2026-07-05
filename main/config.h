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
#define MARBLE_MAX_SPEED        5000.0f // 弹珠最大速度限制(像素/s)

/* ------------------------------ IMU配置 ------------------------------ */
#define IMU_ENABLED             1       // 是否启用真实IMU
#define IMU_I2C_SCL_IO          4       // IMU独立I2C总线 SCL引脚 (I2C0)
#define IMU_I2C_SDA_IO          5       // IMU独立I2C总线 SDA引脚 (I2C0)
#define IMU_FILTER_ALPHA        0.98f   // 互补滤波系数

/* ------------------------------ 图像预处理配置 ------------------------------ */
#define PREPROC_FLAG_NONE        0
#define PREPROC_FLAG_HIST_EQ     (1 << 0)   // 直方图均衡化
#define PREPROC_FLAG_SHARPEN     (1 << 1)   // 锐化 (3x3 Laplacian)
#define PREPROC_FLAG_CONTRAST    (1 << 2)   // 对比度拉伸 (2%-98% 百分位)
#define PREPROC_FLAG_DENOISE     (1 << 3)   // 去噪 (3x3 中值滤波)
#define PREPROC_FLAG_GAMMA       (1 << 4)   // Gamma 校正 (γ=1.8 暗区提亮)

/* Gamma 校正值 (>1 提亮暗区, <1 压暗) */
#define PREPROC_GAMMA_VALUE      1.8f

/* 初始预处理模式 (运行时可通过按钮切换) */
#define PREPROC_DEFAULT_FLAGS    PREPROC_FLAG_GAMMA

/* 预处理预设模式 (按钮循环切换) */
#define PREPROC_PRESET_COUNT     6
#define PREPROC_PRESET_0         PREPROC_FLAG_NONE                                 /* OFF          */
#define PREPROC_PRESET_1         PREPROC_FLAG_GAMMA                                /* Gamma        */
#define PREPROC_PRESET_2         (PREPROC_FLAG_GAMMA | PREPROC_FLAG_DENOISE)       /* Gamma+DN     */
#define PREPROC_PRESET_3         PREPROC_FLAG_DENOISE                              /* DN           */
#define PREPROC_PRESET_4         (PREPROC_FLAG_HIST_EQ | PREPROC_FLAG_DENOISE)     /* HE+DN        */
#define PREPROC_PRESET_5         (PREPROC_FLAG_GAMMA | PREPROC_FLAG_DENOISE |      \
                                  PREPROC_FLAG_SHARPEN | PREPROC_FLAG_CONTRAST)     /* ALL */

/* 预处理管线执行顺序: 去噪 → Gamma 校正 → 直方图均衡化 → 锐化 → 对比度拉伸 */

/* ------------------------------ 相机 ISP 配置 ----------------------------- */
#define CAMERA_AE_LEVEL         4       // AE 目标亮度偏移 (EV), 范围通常 -5~+5, 正值更亮
#define CAMERA_ISP_GAMMA_ENABLE 1       // 是否启用 ISP 硬件 Gamma 校正
#define CAMERA_ISP_GAMMA_VALUE  1.8f    // ISP Gamma 值 (>1 提亮暗区)

/* ------------------------------ 边缘检测配置 ------------------------------ */
#define EDGE_DETECT_ENABLED     1
#define EDGE_DOWNSCALE_W        400     // Canny 处理分辨率 (宽度, 原始800的一半)
#define EDGE_DOWNSCALE_H        320     // Canny 处理分辨率 (高度, 原始640的一半)
#define CANNY_LOW_THRESHOLD     30      // Canny 低阈值 (弱边缘)
#define CANNY_HIGH_THRESHOLD    90      // Canny 高阈值 (强边缘)
#define EDGE_COLOR_RGB565       0x07FF  // 边缘线条颜色 (Cyan: R=0,G=63,B=31)

#ifdef __cplusplus
}
#endif
