# FreeRTOS 多任务架构与低功耗/启动方案

## 目录
1. [当前架构问题分析](#一当前架构问题分析)
2. [目标多任务架构](#二目标多任务架构)
3. [任务详细设计](#三任务详细设计)
4. [任务间通信设计](#四任务间通信设计)
5. [低功耗深度集成](#五低功耗深度集成)
6. [启动与唤醒流程](#六启动与唤醒流程)
7. [内存与资源管理](#七内存与资源管理)
8. [实施路线](#八实施路线)
9. [代码框架示例](#九代码框架示例)

---

## 一、当前架构问题分析

### 1.1 现状

当前所有逻辑挤压在 `app_main()` 的单一 `while(1)` 循环中：

```
app_main() {
    初始化: display → camera → preprocessing → IMU → physics
    while(1) {
        switch(game_state) { ... }          // 游戏状态机
        if (detect_triggered) { ... }       // 目标检测
        if (edge_view) { ... }              // 边缘检测+游戏模式
        if (live_view) { ... }              // 实时预览
        vTaskDelay(50ms);                   // 空闲延时
    }
}
```

### 1.2 核心缺陷

| 问题 | 影响 | 严重程度 |
|------|------|----------|
| **单线程阻塞** | `VIDIOC_DQBUF` 阻塞整个循环 0-2000ms，期间无法处理其他事件 | 高 |
| **无法并行** | ESP32-P4 双核完全闲置一核，相机 ISP 和 AI 推理串行执行 | 高 |
| **时序不稳** | 物理引擎和IMU采样没有固定周期，帧率随检测耗时波动 | 中 |
| **无法睡眠** | 循环始终"就绪"，FreeRTOS Tickless Idle 永远不会触发 Auto Light-sleep | **致命(功耗)** |
| **耦合严重** | 所有模块生命周期绑定，无法独立启停、仅按需运行 | 中 |
| **状态管理困难** | goto 语句、多层嵌套 if-else，难以增删状态或插入新逻辑 | 中 |
| **功耗无感知** | 无空闲超时机制，背光/相机/IMU永远在线 | 高 |

### 1.3 为什么 vTaskDelay 不等于"省电"

```c
vTaskDelay(pdMS_TO_TICKS(50));   // 只是放弃CPU 50ms
// FreeRTOS 仍然认为"有任务可运行"(本任务50ms后就要被唤醒)
// → Tickless Idle 无法进入 Light-sleep
// → CPU时钟继续跑, 只是执行了 Idle Hook 中的空指令
```

**关键认知**: 要让 Auto Light-sleep 工作，**所有任务必须同时阻塞在队列/信号量/事件组上**，即系统处于"真正无事可做"的状态。

---

## 二、目标多任务架构

### 2.1 设计原则

1. **按功能拆分**: 每个独立硬件外设或逻辑模块对应一个任务
2. **按需运行**: 任务在不需要时阻塞或挂起，不轮询
3. **解耦通信**: 任务间通过队列/事件组异步通信，避免共享状态
4. **功耗原生**: 任务阻塞 → 系统空闲 → Auto Light-sleep，无需刻意"进入低功耗"
5. **双核利用**: 相机流和AI检测并行，渲染和物理并行

### 2.2 任务总览

```
┌──────────────────────────────────────────────────────────────────┐
│                       ESP32-P4 双核 SMP                           │
├────────────────────────────┬─────────────────────────────────────┤
│ 优先级 5: Camera Task      │ 优先级 3: Detection Task            │
│  - V4L2 帧捕获/出队         │  - ESP-DL AI 推理                   │
│  - 帧按需分发到下游          │  - 图像预处理                      │
│  - 阻塞于 VIDIOC_DQBUF      │  - 阻塞于 detect_request_q          │
│  - 按需 STREAMON/OFF        │  - 仅检测时运行, 否则挂起            │
├────────────────────────────┼─────────────────────────────────────┤
│ 优先级 4: Display Task      │ 优先级 3: Game Render Task          │
│  - lv_timer_handler()       │  - 像素游戏地图渲染                 │
│  - 预览/游戏缓冲交换         │  - 弹珠/物体/特效叠加              │
│  - LVGL UI事件分发           │  - HUD信息更新                     │
│  - 阻塞于 LVGL timer         │  - 仅 PLAYING 时运行               │
├────────────────────────────┼─────────────────────────────────────┤
│ 优先级 4: IMU Task          │ 优先级 2: Main Control Task         │
│  - 200Hz 周期采样            │  - 游戏状态机                      │
│  - 卡尔曼滤波姿态解算        │  - 按钮/触摸事件处理               │
│  - 发布姿态到物理引擎        │  - 任务生命周期管理                │
│  - 阻塞于 vTaskDelayUntil    │  - 阻塞于 event_q                  │
├────────────────────────────┼─────────────────────────────────────┤
│ 优先级 3: Physics Task      │ 优先级 1: Power Management Task     │
│  - 100Hz 固定步长物理模拟    │  - 空闲计时器管理                  │
│  - 弹珠碰撞检测              │  - 分层睡眠决策                    │
│  - 仅 PLAYING 时运行         │  - PM锁与时钟管理                  │
│  - 阻塞于 imu_attitude_q     │  - 周期性检查(1Hz)                 │
└────────────────────────────┴─────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│                   IDLE Task (FreeRTOS 自动)                       │
│  - 所有8个任务都阻塞时运行                                        │
│  - 触发 Tickless Idle → Auto Light-sleep                         │
│  - CPU时钟门控 + 数字外设断电                                     │
│  - <1ms 唤醒延迟, 对用户完全透明                                   │
└──────────────────────────────────────────────────────────────────┘
```

### 2.3 功耗自然收敛原理

核心机制：**任务阻塞 = 系统空闲 = 自动睡眠**

```
所有任务状态:
  Camera:   阻塞于 xQueueReceive — 等捕获指令      ┐
  Detection: 挂起(suspended) — 不在检测中           │
  Display:  阻塞于 vTaskDelayUntil — 等下次刷新     │
  IMU:      阻塞于 vTaskDelayUntil — 等下次采样     ├─→ 全部阻塞
  Physics:  挂起 — 不在游戏中                       │   → Tickless Idle
  Render:   挂起 — 不在游戏中                       │   → Auto Light-sleep
  Main:     阻塞于 xQueueReceive — 等用户事件       │   → 芯片自动进入低功耗
  Power:    阻塞于 vTaskDelay — 等下次检查          ┘
```

**对比当前**: 主循环一直轮询 `display_is_live_view()`, `display_detect_triggered()` 等标志 → FreeRTOS 认为"有任务要跑" → 永不睡眠

---

## 三、任务详细设计

### 3.1 Camera Task (优先级 5, 核心0)

```
职责: V4L2相机帧捕获与分发
优先级理由: 最高优先级确保CSI DMA不被饿死

运行模式:
  IDLE模式: 阻塞等待 capture_request_sem
  STREAMING模式: 连续 DQBUF → 发布帧 → QQBUF

生命周期:
  - 初始化: camera_init() 在任务内执行
  - 工作: 收到 CAPTURE 信号 → 捕获一帧 → 分发到目标队列
  - 流模式: 收到 STREAM_START → 连续捕获 → 队列满时丢弃旧帧(overwrite)
  - 挂起: 收到 STREAM_STOP → 停止流 → 回到 IDLE
  - 销毁: 收到 DEINIT → camera_deinit() → 任务退出
```

```c
void camera_task(void *pvParams)
{
    camera_init();  // V4L2 open, mmap, STREAMON
    camera_warmup(5);

    while (1) {
        // 等待捕获指令 或 流模式已开启
        CameraCmd cmd;
        if (xQueueReceive(g_camera_cmd_q, &cmd, 
                g_streaming ? 0 : portMAX_DELAY) == pdTRUE) {
            handle_camera_cmd(cmd);
        }

        if (!g_streaming) continue;

        // 阻塞等待V4L2帧就绪 (DQBUF阻塞)
        camera_frame_t frame;
        if (camera_capture_frame(&frame, 500) != ESP_OK) continue;

        // 分发到下游: 优先检测, 其次预览
        if (g_detect_pending) {
            // 非阻塞发送, 队列满则丢弃旧帧 (检测不需要每帧)
            xQueueOverwrite(g_frame_detect_q, &frame);
        }
        if (g_preview_active) {
            // 预览只需要最新帧 (overwrite模式)
            xQueueOverwrite(g_frame_preview_q, &frame);
        }
    }
}
```

### 3.2 Display/LVGL Task (优先级 4)

```
职责: LVGL定时器、显示刷新、UI事件分发
优先级理由: 用户交互的感知核心, 需要及时响应

周期: ~30ms (33fps LVGL刷新)

关键点:
  - lv_timer_handler() 必须在同一个线程中定期调用
  - bsp_display_lock/unlock 保护MIPI DSI通信
  - 触摸事件在LVGL回调中处理, 转换为内部事件发送给Main Task
```

```c
void display_task(void *pvParams)
{
    display_init();  // BSP显示启动, LVGL初始化, UI构建

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33);  // ~30fps

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        bsp_display_lock(portMAX_DELAY);
        lv_timer_handler();  // 驱动LVGL定时器和动画
        bsp_display_unlock();

        // 处理待渲染的帧 (如果有)
        process_pending_frame();
    }
}
```

### 3.3 IMU Task (优先级 4)

```
职责: 固定周期IMU采样 + 卡尔曼滤波
优先级理由: 需要稳定200Hz采样率, 不能被检测或渲染阻塞
周期: 200Hz (5ms)

输出: imu_attitude_q → 供Physics Task和HUD消费

功耗设计:
  - 仅在游戏进行中或倾斜预览时全速运行
  - 空闲时降频至10Hz (仅用于姿态日志)
  - 超时无使用 → 挂起任务(imu_deinit)
```

```c
void imu_task(void *pvParams)
{
    imu_init();  // I2C0总线, 设备探测, 卡尔曼初始化

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5);  // 200Hz
    int sample_count = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        // 读取+滤波
        imu_attitude_t att;
        if (imu_get_attitude(&att) == ESP_OK) {
            // 发布给物理引擎 (非阻塞, 覆盖旧数据)
            xQueueOverwrite(g_imu_attitude_q, &att);
        }

        // 动态降频: 每20次采样(100ms)检查是否还在游戏中
        // 非游戏时降到10Hz
        if (++sample_count >= 20) {
            sample_count = 0;
            if (!g_game_active && !g_preview_active) {
                // 不在此直接改周期, 而是通过命令队列接收
            }
        }
    }
}
```

### 3.4 Physics Task (优先级 3)

```
职责: 固定步长100Hz物理模拟 (弹珠运动+碰撞)
优先级理由: 略低于IMU, 确保姿态数据先于物理消费

运行条件: 仅 GAME_STATE_PLAYING 时激活; 其余时间挂起

设计要点:
  - vTaskDelayUntil 保证固定步长, 不被渲染干扰
  - 从 imu_attitude_q 读取最新姿态 (非阻塞, 无数据则用上次值)
  - 计算结果发布到 marble_state_q
  - 检测 win/lose → 发送事件到 Main Task
```

```c
void physics_task(void *pvParams)
{
    // 等待游戏开始的信号
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 初始挂起

    marble_physics_init();
    pixel_physics_start();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    while (g_game_active) {
        vTaskDelayUntil(&last_wake, period);

        // 1. 读最新IMU姿态 (非阻塞)
        imu_attitude_t att;
        if (xQueuePeek(g_imu_attitude_q, &att, 0) != pdTRUE) {
            // 无新数据, 重用上次
        }

        // 2. 物理步进
        marble_physics_step(att.roll, att.pitch);

        // 3. 发布弹珠状态
        marble_state_t ms;
        marble_physics_get_state(&ms);
        xQueueOverwrite(g_marble_state_q, &ms);

        // 4. 检查游戏结束条件
        pixel_world_t *world = pixel_world_get();
        if (world && world->goal_reached) {
            send_event(EVT_GAME_WIN);
        } else if (world && world->player_dead) {
            send_event(EVT_GAME_LOSE);
        }
    }

    pixel_physics_stop();
    vTaskDelete(NULL);
}
```

### 3.5 Detection Task (优先级 3)

```
职责: ESP-DL AI目标检测 (COCO YOLO11n)
优先级理由: 低于Camera和Display, 可被抢占; 但需要双核并行优势

生命周期: 按需创建/销毁 或 常驻挂起
  - 收到 detect_request_q → 处理帧 → 发布结果 → 回到阻塞
  - 空闲时: 阻塞在 detect_request_q → 贡献Light-sleep

内存: ESP-DL模型加载在PSRAM, 模型仅在任务创建后初始化
```

```c
void detection_task(void *pvParams)
{
    // 按需初始化ESP-DL模型 (约2-3MB PSRAM)
    detection_init();

    while (1) {
        // 阻塞等待检测请求
        detection_request_t req;
        if (xQueueReceive(g_detect_request_q, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // 预处理 (可选: 复用相机任务已做的预处理)
        if (req.preproc_flags != PREPROC_FLAG_NONE) {
            preprocess_frame(req.frame, req.preproc_flags);
        }

        // AI推理 (耗时操作, 可被抢占)
        detection_result_t results[DETECTION_MAX_OBJECTS];
        int count = DETECTION_MAX_OBJECTS;
        esp_err_t ret = detection_run_internal(req.frame, results, &count);

        // 发布结果
        detection_response_t resp = {
            .results = results,
            .count = count,
            .ret = ret,
        };
        xQueueSend(g_detect_result_q, &resp, pdMS_TO_TICKS(100));
    }
}
```

### 3.6 Game Render Task (优先级 3)

```
职责: 游戏画面渲染 (640×640像素地图 + 弹珠 + HUD)
优先级理由: 与Detection同级, 双核可并行

运行条件: 仅 GAME_STATE_PLAYING 时激活

周期: GAME_FPS (30fps = 33ms)

与Display Task的分工:
  Game Render Task: 绘制游戏内容到后端缓冲 (纯CPU渲染)
  Display Task: 缓冲交换 + LVGL刷新 + 送显
```

### 3.7 Main Control Task (优先级 2)

```
职责: 游戏状态机、用户交互响应、任务生命周期管理
优先级理由: 最低功能性优先级, 不抢占时序敏感任务

这是 app_main 演化的任务, 是整个系统的"大脑":

状态管理:
  IDLE → CAPTURING → PLAYING → WIN/LOSE → IDLE
  
  每个状态转换时:
    1. 发送命令到相关任务 (启动/挂起/恢复)
    2. 更新PM锁 (游戏时需要全速, 空闲时释放)
    3. 更新UI状态文本

事件处理:
  - 按钮事件 (Live View / Edge / Game / Preproc / Detect)
  - 检测完成事件 → 更新预览
  - 游戏胜利/失败事件 → 显示结束画面
  - 电源管理事件 → 准备睡眠
```

```c
void main_control_task(void *pvParams)
{
    // --- 冷启动或唤醒分支 ---
    esp_sleep_wake_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    BootMode boot_mode = (wake_cause == ESP_SLEEP_WAKEUP_UNDEFINED) 
                         ? BOOT_COLD : BOOT_WARM;

    // 按需创建子系统任务
    create_all_tasks(boot_mode);

    // 通知所有任务: 系统就绪
    broadcast_event(EVT_SYSTEM_READY);

    // --- 主事件循环 ---
    while (1) {
        event_t evt;
        // 阻塞等待事件 (无事件时此任务阻塞 → 贡献Light-sleep!)
        if (xQueueReceive(g_event_q, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (g_game_state) {
        case GAME_STATE_IDLE:
            handle_idle_event(&evt);
            break;
        case GAME_STATE_CAPTURING:
            handle_capturing_event(&evt);
            break;
        case GAME_STATE_PLAYING:
            handle_playing_event(&evt);
            break;
        case GAME_STATE_WIN:
        case GAME_STATE_LOSE:
            handle_game_end_event(&evt);
            break;
        }
    }
}
```

### 3.8 Power Management Task (优先级 1)

```
职责: 空闲检测、分级睡眠决策、外设启停
优先级理由: 最低优先级, 仅在真正无事可做时运行

空闲超时层级:
  用户活跃 ──30秒──→ IDLE_DIM (背光20%)
     │                   │
     │    ←──触摸事件─────┘
     │
   IDLE_DIM ──30秒──→ IDLE_STANDBY (背光OFF+相机STREAMOFF)
     │                     │
     │    ←──触摸事件───────┘
     │
   IDLE_STANDBY ──5分钟──→ DEEP_STANDBY (相机deinit+显示休眠+IMU停止)
     │                         │
     │    ←──触摸事件───────────┘
     │
   DEEP_STANDBY ──30分钟──→ DEEP_SLEEP (esp_deep_sleep_start)
     │                         │
     └────触摸INT唤醒───────────┘ (GPIO唤醒 → RTC持久化 → 快速启动)
```

```c
void power_mgmt_task(void *pvParams)
{
    PowerState state = POWER_ACTIVE;
    uint32_t idle_ticks = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // 每秒检查一次

        // 获取距离上次用户操作的时间
        uint32_t elapsed = get_idle_elapsed_seconds();

        switch (state) {
        case POWER_ACTIVE:
            if (elapsed > 30) {
                state = POWER_IDLE_DIM;
                send_cmd(CMD_BACKLIGHT_DIM);       // 背光20%
                send_cmd(CMD_CAMERA_STREAM_STOP);  // 关相机流
            }
            break;
        case POWER_IDLE_DIM:
            if (elapsed > 60) {
                state = POWER_IDLE_STANDBY;
                send_cmd(CMD_BACKLIGHT_OFF);       // 完全关背光
                send_cmd(CMD_DISPLAY_SLEEP);       // MIPI DCS休眠
                send_cmd(CMD_IMU_STOP);            // 停止IMU采样
            }
            break;
        case POWER_IDLE_STANDBY:
            if (elapsed > 360) {  // 6分钟(30s+30s+5min)
                state = POWER_DEEP_STANDBY;
                send_cmd(CMD_CAMERA_DEINIT);       // 释放相机资源
                // 配置触摸INT唤醒
                configure_touch_wakeup();
            }
            break;
        case POWER_DEEP_STANDBY:
            if (elapsed > 2160) {  // 36分钟
                state = POWER_DEEP_SLEEP;
                // 通知所有任务准备睡眠
                broadcast_event(EVT_ENTER_DEEP_SLEEP);
                vTaskDelay(pdMS_TO_TICKS(500));  // 等待各任务清理
                // 进入Deep-sleep (不复返)
                esp_deep_sleep_start();
            }
            break;
        }
    }
}
```

---

## 四、任务间通信设计

### 4.1 通信通道汇总

```
┌──────────────┬──────────────┬─────────┬──────────────────────────────┐
│   生产者      │   消费者      │  通道    │  语义                         │
├──────────────┼──────────────┼─────────┼──────────────────────────────┤
│ Camera       │ Detection    │ Queue(2)│ 非阻塞发送, 满则丢弃旧帧        │
│ Camera       │ Display      │ Queue(1)│ Overwrite, 预览只需要最新帧     │
│ IMU          │ Physics      │ Queue(1)│ Overwrite, 物理只关心最新姿态    │
│ Physics      │ Game Render  │ Queue(1)│ Overwrite, 渲染只需要最新弹珠状态│
│ Main         │ Camera       │ Queue(5)│ 命令: CAPTURE/STREAM/STOP/DEINIT│
│ Main         │ Detection    │ Queue(3)│ 检测请求+帧指针                 │
│ Detection    │ Main/Display │ Queue(5)│ 检测结果(物体列表+边界框)        │
│ LVGL回调     │ Main         │ Queue(10)│ 按钮/触摸事件                  │
│ Power        │ 各任务        │ Queue(3)│ 睡眠/唤醒命令                  │
│ 各任务       │ Power        │ Queue(3)│ 用户活动通知(重置空闲计时器)     │
│ Main         │ Physics      │ TaskNotify│ 开始/停止物理模拟              │
│ Main         │ Game Render  │ TaskNotify│ 开始/停止渲染                 │
└──────────────┴──────────────┴─────────┴──────────────────────────────┘
```

### 4.2 关键队列设计细节

**相机帧队列 — 使用 Ring Buffer 而非 FreeRTOS Queue**:

FreeRTOS Queue 在元素大小较大时(帧指针+元数据=16字节)有内存碎片问题。对于相机→检测的帧传递，建议使用**有锁环形缓冲区**:

```c
#define FRAME_RING_SIZE 3

typedef struct {
    camera_frame_t frames[FRAME_RING_SIZE];
    volatile int write_idx;
    volatile int read_idx;
    SemaphoreHandle_t data_ready;   // 有新帧可读
    SemaphoreHandle_t space_ready;  // 有空位可写
    portMUX_TYPE lock;
} frame_ring_t;
```

**检测结果队列 — 传递指针而非拷贝**:

检测结果包含最多10个物体信息(~1KB)。用指针传递避免拷贝:

```c
// Queue元素是指针, 而非结构体本身
QueueHandle_t g_detect_result_q = xQueueCreate(5, sizeof(detection_result_t*));

// 生产者
detection_result_t *results = malloc(sizeof(detection_result_t) * DETECTION_MAX_OBJECTS);
// ... 填充结果
xQueueSend(g_detect_result_q, &results, portMAX_DELAY);

// 消费者
detection_result_t *results;
xQueueReceive(g_detect_result_q, &results, portMAX_DELAY);
// ... 使用结果
free(results);  // 消费者负责释放
```

### 4.3 事件定义

```c
typedef enum {
    // === 用户交互事件 ===
    EVT_BTN_LIVE_VIEW,
    EVT_BTN_EDGE_VIEW,
    EVT_BTN_GAME,
    EVT_BTN_DETECT,
    EVT_BTN_PREPROC,
    EVT_TOUCH_SCREEN,     // 任意触摸(用于唤醒)

    // === 游戏事件 ===
    EVT_GAME_CAPTURE_DONE,
    EVT_GAME_WIN,
    EVT_GAME_LOSE,
    EVT_GAME_EXIT,

    // === 检测事件 ===
    EVT_DETECTION_COMPLETE,
    EVT_DETECTION_FAILED,

    // === 系统事件 ===
    EVT_SYSTEM_READY,
    EVT_ENTER_DEEP_SLEEP,
    EVT_WOKE_FROM_SLEEP,

    // === 电源事件 ===
    EVT_POWER_IDLE_DIM,
    EVT_POWER_IDLE_STANDBY,
    EVT_POWER_DEEP_STANDBY,
    EVT_USER_ACTIVITY,    // 任何用户操作(重置空闲计时器)
} event_type_t;

typedef struct {
    event_type_t type;
    void *data;           // 可选附加数据
    uint32_t timestamp;
} event_t;
```

---

## 五、低功耗深度集成

### 5.1 PM锁策略

不同运行时状态持有不同的PM锁，精确控制DFS和Light-sleep:

```c
// 锁创建 (在app_main早期)
esp_pm_lock_handle_t g_pm_lock_high;       // 禁止Light-sleep, CPU高频
esp_pm_lock_handle_t g_pm_lock_medium;     // 允许Light-sleep, CPU可降频

void pm_locks_init(void)
{
    // 高功耗锁: 游戏/检测时持有
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "high_perf", &g_pm_lock_high);
    // 中功耗锁: 预览时持有(允许Light-sleep但保CPU不降太低)
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_max", &g_pm_lock_medium);
}
```

| 状态 | PM锁 | CPU频率 | Light-sleep | 典型功耗 |
|------|------|---------|-------------|----------|
| 游戏中 | `NO_LIGHT_SLEEP` | 360MHz | 禁止 | ~770mA |
| 检测中 | `NO_LIGHT_SLEEP` | 360MHz | 禁止 | ~700mA |
| 预览(Live/Edge) | `CPU_FREQ_MAX` | 360MHz | 允许(帧间) | ~500mA |
| 空闲(30s) | 无锁 | 40-360MHz自动 | 自动 | ~150mA |
| 浅待机(1min) | 无锁 | 40MHz | 持续 | ~30mA |
| 深待机(5min+) | N/A | 0 | Deep-sleep | ~5mA |

### 5.2 空闲检测机制

不依赖轮询，而是通过"最后活跃时间戳"机制:

```c
static volatile uint32_t g_last_activity_tick = 0;

// 每个用户交互点调用
void notify_user_activity(void)
{
    g_last_activity_tick = xTaskGetTickCount();
    // 非阻塞发送给Power Task (重置状态机)
    event_t evt = { .type = EVT_USER_ACTIVITY };
    xQueueSend(g_power_event_q, &evt, 0);
}

uint32_t get_idle_elapsed_seconds(void)
{
    return (xTaskGetTickCount() - g_last_activity_tick) * portTICK_PERIOD_MS / 1000;
}

// 在以下位置插入 notify_user_activity():
// - LVGL按钮回调
// - 触摸屏INT中断
// - 按键扫描
```

### 5.3 Auto Light-sleep 透明工作原理

```
时间轴: ═══ 任务运行 ═══ | ... 全部阻塞 ... | ═══ 唤醒 ═══

1. 所有任务都阻塞 (等队列/等信号量/等延时)
2. FreeRTOS Idle Task 开始运行
3. Idle Task 检测到 Tickless Idle 可触发
4. 计算下一个任务的唤醒时间
5. 配置RTC定时器到该时间
6. 进入 Light-sleep (CPU停, 外设时钟门控, RAM保持)
7. RTC定时器触发 → PMU唤醒
8. 时钟恢复 → CPU从休眠点继续执行
9. FreeRTOS唤醒对应任务

总延迟: <1ms (约300μs PMU开销)
用户感知: 完全透明, 操作即时响应
```

### 5.4 相机按需启停流程

这是除了背光之外最大的功耗优化点:

```
需要拍照 → Main Task发 CMD_CAMERA_CAPTURE
            → Camera Task: VIDIOC_STREAMON (若已停止)
            → 预热3帧 (~300ms)
            → DQBUF 获取一帧
            → 分发给检测/预览
            → 等待超时(如5秒无新请求)
            → VIDIOC_STREAMOFF

需要预览 → Main Task发 CMD_CAMERA_STREAM_START
            → Camera Task: VIDIOC_STREAMON
            → 连续 DQBUF → 发布到预览队列
            → 直到收到 CMD_CAMERA_STREAM_STOP
```

---

## 六、启动与唤醒流程

### 6.1 冷启动 (上电/复位)

```
上电
  │
  ▼
ROM 1st-stage Bootloader
  │
  ▼
2nd-stage Bootloader (SPI Flash)
  ├── 跳过固件校验 (CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP=n
  │                  时才会校验, 冷启动必须校验)
  │
  ▼
FreeRTOS 初始化 → app_main()
  │
  ├─ 1. esp_pm_configure()                     // 启用DFS+Auto Light-sleep
  ├─ 2. pm_locks_init()                        // 创建PM锁
  ├─ 3. 创建Event Group、所有Queue
  ├─ 4. 创建Power Management Task              // 开始监控空闲
  ├─ 5. 创建Display/LVGL Task                  // 显示初始化(耗时~1s)
  │      └─ 内部: bsp_display_start()
  │               bsp_display_backlight_on()
  │               构建LVGL UI
  ├─ 6. 创建Camera Task                        // 相机初始化(耗时~2s)
  │      └─ 内部: esp_video_init()
  │               open("/dev/video0")
  │               设置格式+缓冲区
  │               VIDIOC_STREAMON
  │               camera_warmup(10)
  ├─ 7. 创建IMU Task                           // IMU初始化(~0.5s)
  │      └─ 内部: I2C总线, 设备探测, 卡尔曼
  ├─ 8. 创建Detection Task (暂时挂起)           // 仅创建, 不加载模型
  ├─ 9. 创建Physics Task (挂起, 等待游戏)       // 不分配资源
  ├─ 10. 创建Game Render Task (挂起)            // 不分配缓冲
  │
  └─ 11. app_main 演化为 Main Control Task
         └─ 进入主事件循环
              └─ 广播 EVT_SYSTEM_READY
              └─ 设置状态: "Ready"
              └─ 阻塞等待事件 ←─ Light-sleep由此开始
```

**冷启动总耗时: ~3-4秒** (显示1s + 相机2s + IMU 0.5s, 部分可并行)

### 6.2 触摸唤醒 (Deep-sleep → 快速恢复)

```
触摸面板检测到手指 → GT911 INT引脚拉低
  │
  ▼
GPIO唤醒PMU → 芯片退出Deep-sleep
  │
  ▼
ROM Bootloader
  ├── 检查 RTC_CNTL_STORE6_REG → Wake Stub地址
  ├── 若有Wake Stub → 跳转到RTC FAST Memory执行
  │   └── 恢复GPIO状态 (可选)
  │
  ▼
2nd-stage Bootloader
  ├── CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP=y → 跳过校验
  │   → 节省 ~500ms
  │
  ▼
app_main()
  ├── esp_sleep_get_wakeup_cause() = ESP_SLEEP_WAKEUP_GPIO
  ├── RTC_DATA_ATTR 读取保留状态
  │   ├── boot_count++
  │   ├── calibration_done = true → 跳过ISP校准
  │   └── last_state = DEEP_STANDBY
  │
  ├── esp_pm_configure() (同样需要)
  ├── 创建Power Mgmt Task
  ├── 创建Display Task
  │   └── display_wakeup() (仅从DCS Sleep恢复, 不复位IC)
  ├── 创建Camera Task
  │   └── RTC状态指示之前已校准 → 仅 warmup(3)
  ├── 创建IMU Task (跳过版本检查, 直接恢复)
  ├── 按需创建其他任务
  │
  └── Main Control → 广播 EVT_WOKE_FROM_SLEEP

快速恢复总耗时: ~1-2秒
  (跳过ISP校准、IMU版本探测、模型重新加载等重复操作)
```

### 6.3 RTC持久化状态结构

```c
// rtc_state.h
#include "esp_attr.h"

typedef struct {
    uint32_t magic;             // 0xA5F3C017 — 验证内存内容有效
    uint32_t boot_count;        // 累计启动次数
    uint32_t deep_sleep_count;  // Deep-sleep唤醒次数

    // 校准状态 (避免每次唤醒重新校准)
    struct {
        bool camera_isp_calibrated : 1;
        bool imu_calibrated        : 1;
        bool display_initialized   : 1;
        uint8_t reserved           : 5;
    } flags;

    // 上次状态 (决定恢复策略)
    uint8_t last_game_state;    // 游戏状态机快照
    uint8_t last_power_state;   // 电源状态快照

    // 预留: 可存储少量游戏进度(如关卡、分数)
    uint8_t reserved[56];
} rtc_persist_t;

RTC_DATA_ATTR static rtc_persist_t g_rtc;
```

### 6.4 Light-sleep 恢复 (无感知)

Light-sleep 恢复不需要任何特殊处理:

```
所有任务阻塞 → Auto Light-sleep → GPIO/定时器唤醒
  │
  ▼
CPU从休眠继续执行 (PC寄存器不变, RAM内容不变)
  │
  ▼
FreeRTOS调度器恢复 → 唤醒等待队列的任务
  │
  ▼
任务继续运行, 仿佛什么都没发生

总延迟: <1ms (仅PMU数字上电 + PLL稳定时间)
```

---

## 七、内存与资源管理

### 7.1 PSRAM使用预算

| 资源 | 大小 | 常驻/按需 | 所属任务 |
|------|------|-----------|----------|
| 相机帧缓冲 (V4L2 mmap) | 800×640×2 × 2 = 2MB | 按需 | Camera |
| 预览双缓冲 (前端+后端) | 640×512×2 × 2 = 1.25MB | 常驻 | Display |
| 游戏双缓冲 (前端+后端) | 640×640×2 × 2 = 1.56MB | 按需 | Game Render |
| 边缘检测缓冲 (降采样+边缘图) | ~250KB | 按需 | Detection |
| ESP-DL 模型 (YOLO11n) | ~2-3MB | 按需 | Detection |
| LVGL 显示缓冲 | ~300KB | 常驻 | Display |
| 预处理临时缓冲 | ~1MB | 按需(临时) | Detection |
| FreeRTOS 任务栈 | 8×4KB = 32KB | 常驻 | 所有任务 |
| **总计(峰值)** | **~8-9MB** | | |
| **总计(空闲)** | **~1.6MB** | | |

ESP32-P4 开发板通常有 8MB 或 16MB PSRAM，峰值内存可接受。关键是**空闲时释放大块缓冲区**。

### 7.2 任务栈大小建议

| 任务 | 栈大小 | 理由 |
|------|--------|------|
| Camera | 4096 | V4L2 ioctl调用栈浅, 无递归 |
| Display/LVGL | 8192 | LVGL内部有嵌套绘制调用 |
| IMU | 2048 | 浮点运算, 栈浅 |
| Physics | 3072 | 碰撞检测有中等嵌套 |
| Detection | 8192 | ESP-DL内部栈消耗大 |
| Game Render | 4096 | 像素渲染循环, 栈浅 |
| Main Control | 4096 | 状态机+事件处理 |
| Power Mgmt | 2048 | 逻辑简单 |
| **总计** | **36KB** | DRAM可承受 |

### 7.3 关键内存管理原则

1. **按需分配, 按需释放**: 不在 GAME_STATE_IDLE 时持有游戏缓冲
2. **分配归属清晰**: 哪个任务分配, 哪个任务释放 (通过消息通知例外)
3. **PSRAM优先**: 大缓冲用 `heap_caps_calloc(..., MALLOC_CAP_SPIRAM)`
4. **DMA对齐**: Camera缓冲已由V4L2内核驱动管理对齐
5. **Deep-sleep前全部释放**: 发送 EVT_ENTER_DEEP_SLEEP → 各任务清理 → 最后检查 `heap_caps_get_free_size()`

---

## 八、实施路线

### Phase 1: 基础重构 (1-2天) — 无功耗变化，但为功耗铺路

**目标**: 将当前单一循环拆分为多任务，功能完全等价，确保稳定

1. 创建 `main/tasks/` 目录结构
2. 提取 Camera Task (封装当前的 camera_capture_frame)
3. 提取 Display/LVGL Task (封装当前的 display_init + lv_timer_handler)
4. 提取 Main Control Task (封装当前的状态机逻辑)
5. 创建 event_q 和基础事件定义
6. sdkconfig: 启用 `CONFIG_FREERTOS_USE_TICKLESS_IDLE`
7. **不引入任何功耗优化** — 仅架构重构

**验证**: 编译通过, 所有功能与原版一致, 无回归

### Phase 2: 功耗基础 (1天) — 引入Auto Light-sleep

**目标**: 空闲时系统能够自动进入Light-sleep

1. 添加 `esp_pm_configure()` 到 app_main
2. 创建 Power Management Task (仅空闲计时)
3. 添加 PM锁: 游戏/检测时持有 `NO_LIGHT_SLEEP`
4. 在各任务中插入 `notify_user_activity()`
5. sdkconfig: 添加 PM + Light-sleep 配置
6. **测量**: 空闲态电流应有显著下降

**验证**: 空闲30秒后电流明显降低, 触摸操作即时响应

### Phase 3: 按需启停 (1天) — 关闭不用的外设

**目标**: 空闲时关闭背光、相机流、IMU

1. 实现背光分级控制 (100% → 50% → 20% → OFF)
2. 实现 Camera Task 的 stream start/stop
3. 实现 IMU Task 的采样率调整和挂起
4. Power Mgmt Task 增加分层超时逻辑
5. **测量**: 空闲1分钟后电流进一步下降

### Phase 4: 深度优化 (1-2天) — Deep-sleep + 快速唤醒

**目标**: 长期空闲进入Deep-sleep, 触摸能快速唤醒

1. 添加 `rtc_state.h` RTC持久化状态
2. 实现显示休眠 (MIPI DCS Sleep In)
3. 实现 GT911 触摸唤醒 (GPIO wakeup)
4. 实现快速恢复路径 (RTC_DATA_ATTR)
5. sdkconfig: 添加 `SKIP_VALIDATE_IN_DEEP_SLEEP`
6. **测量**: Deep-sleep电流 < 10mA, 唤醒 < 2秒

### Phase 5: 性能优化 (可选)

1. 双核亲和性调优 (Camera/IMU pin Core0, Detection/Render pin Core1)
2. 环形缓冲替代部分Queue
3. 帧时间戳对齐, 减少不必要的帧处理

---

## 九、代码框架示例

### 9.1 目录结构

```
main/
├── CMakeLists.txt
├── idf_component.yml
├── config.h                      // 全局配置(保持)
├── main.c                        // app_main入口, 创建所有任务
├── rtc_state.h                   // RTC持久化状态定义
├── tasks/
│   ├── camera_task.c/h           // 相机捕获任务
│   ├── display_task.c/h          // LVGL显示任务
│   ├── imu_task.c/h              // IMU采样任务
│   ├── physics_task.c/h          // 物理引擎任务
│   ├── detection_task.c/h        // AI检测任务
│   ├── game_render_task.c/h      // 游戏渲染任务
│   ├── main_control_task.c/h     // 主控状态机
│   └── power_mgmt_task.c/h       // 电源管理任务
├── ipc/
│   └── ipc.h                     // 事件/命令/队列定义
├── camera/                       // (保持不变)
├── display/                      // (调整)
├── imu/                          // (保持不变)
├── detection/                    // (保持不变)
├── pixel_game/                   // (保持不变)
├── physics/                      // (小调)
├── game/                         // (小调)
├── edge_detection/               // (保持不变)
├── track/                        // (保持不变)
└── image_processing/             // (保持不变)
```

### 9.2 核心IPC定义 (`ipc.h`)

```c
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "config.h"
#include "camera/camera_driver.h"
#include "detection/detection_driver.h"
#include "imu/imu_driver.h"
#include "physics/marble_physics.h"

/* ---- 事件类型 ---- */
typedef enum {
    EVT_BTN_LIVE_VIEW = 1,
    EVT_BTN_EDGE_VIEW,
    EVT_BTN_GAME,
    EVT_BTN_DETECT,
    EVT_BTN_PREPROC,
    EVT_TOUCH_SCREEN,
    EVT_GAME_CAPTURE_DONE,
    EVT_GAME_WIN,
    EVT_GAME_LOSE,
    EVT_GAME_EXIT,
    EVT_DETECTION_COMPLETE,
    EVT_DETECTION_FAILED,
    EVT_SYSTEM_READY,
    EVT_ENTER_DEEP_SLEEP,
    EVT_WOKE_FROM_SLEEP,
    EVT_POWER_STATE_CHANGE,
    EVT_USER_ACTIVITY,
} event_type_t;

typedef struct {
    event_type_t type;
    void *data;
    uint32_t timestamp;
} event_t;

/* ---- 命令类型 ---- */
typedef enum {
    CMD_CAMERA_CAPTURE,         // 捕获单帧
    CMD_CAMERA_STREAM_START,    // 开始连续流
    CMD_CAMERA_STREAM_STOP,     // 停止流
    CMD_CAMERA_DEINIT,          // 反初始化
    CMD_DISPLAY_SLEEP,          // 显示休眠
    CMD_DISPLAY_WAKEUP,         // 显示唤醒
    CMD_BACKLIGHT_SET,          // 设置背光亮度 (0-100)
    CMD_IMU_START,              // 开始IMU采样
    CMD_IMU_STOP,               // 停止IMU采样
    CMD_IMU_SET_RATE,           // 设置采样率
    CMD_DETECTION_RUN,          // 运行检测
    CMD_PHYSICS_START,          // 开始物理模拟
    CMD_PHYSICS_STOP,           // 停止物理模拟
    CMD_RENDER_START,           // 开始游戏渲染
    CMD_RENDER_STOP,            // 停止游戏渲染
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    void *data;
} cmd_t;

/* ---- 全局IPC句柄 (main.c中初始化) ---- */
extern QueueHandle_t g_event_q;
extern QueueHandle_t g_camera_cmd_q;
extern QueueHandle_t g_detect_request_q;
extern QueueHandle_t g_detect_result_q;
extern QueueHandle_t g_frame_detect_q;
extern QueueHandle_t g_frame_preview_q;
extern QueueHandle_t g_imu_attitude_q;
extern QueueHandle_t g_marble_state_q;
extern QueueHandle_t g_power_cmd_q;

extern EventGroupHandle_t g_sys_events;
#define SYS_EVT_GAME_ACTIVE     (1 << 0)
#define SYS_EVT_PREVIEW_ACTIVE  (1 << 1)
#define SYS_EVT_IDLE            (1 << 2)
#define SYS_EVT_SLEEP_PREPARE   (1 << 3)

/* ---- 便捷发送宏 ---- */
static inline void send_event(event_type_t type, void *data)
{
    event_t evt = { .type = type, .data = data, 
                    .timestamp = xTaskGetTickCount() };
    xQueueSend(g_event_q, &evt, 0);
}

static inline void send_cmd(QueueHandle_t q, cmd_type_t type, void *data)
{
    cmd_t cmd = { .type = type, .data = data };
    xQueueSend(q, &cmd, pdMS_TO_TICKS(100));
}
```

### 9.3 新的 main.c

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "config.h"
#include "rtc_state.h"
#include "ipc.h"

static const char *TAG = "main";

// ---- 全局IPC句柄定义 ----
QueueHandle_t g_event_q           = NULL;
QueueHandle_t g_camera_cmd_q      = NULL;
QueueHandle_t g_detect_request_q  = NULL;
QueueHandle_t g_detect_result_q   = NULL;
QueueHandle_t g_frame_detect_q    = NULL;
QueueHandle_t g_frame_preview_q   = NULL;
QueueHandle_t g_imu_attitude_q    = NULL;
QueueHandle_t g_marble_state_q    = NULL;
QueueHandle_t g_power_cmd_q       = NULL;
EventGroupHandle_t g_sys_events   = NULL;

// ---- PM锁 ----
static esp_pm_lock_handle_t g_pm_lock_high;
static esp_pm_lock_handle_t g_pm_lock_medium;

// ---- 外部任务入口 ----
extern void camera_task(void *);
extern void display_task(void *);
extern void imu_task(void *);
extern void physics_task(void *);
extern void detection_task(void *);
extern void game_render_task(void *);
extern void main_control_task(void *);
extern void power_mgmt_task(void *);

static void create_all_ipc(void)
{
    g_event_q          = xQueueCreate(10, sizeof(event_t));
    g_camera_cmd_q     = xQueueCreate(5,  sizeof(cmd_t));
    g_detect_request_q = xQueueCreate(3,  sizeof(detection_request_t));
    g_detect_result_q  = xQueueCreate(5,  sizeof(detection_result_t*));
    g_frame_detect_q   = xQueueCreate(2,  sizeof(camera_frame_t));
    g_frame_preview_q  = xQueueCreate(1,  sizeof(camera_frame_t));
    g_imu_attitude_q   = xQueueCreate(1,  sizeof(imu_attitude_t));
    g_marble_state_q   = xQueueCreate(1,  sizeof(marble_state_t));
    g_power_cmd_q      = xQueueCreate(5,  sizeof(cmd_t));
    g_sys_events       = xEventGroupCreate();
}

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  AR Sandbox - Multi-Task Architecture");
    ESP_LOGI(TAG, "==============================================");

    // === Step 1: Power Management ===
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 360,
        .min_freq_mhz = 40,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "high_perf", &g_pm_lock_high);
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_max", &g_pm_lock_medium);

    // === Step 2: Wake Cause Detection ===
    esp_sleep_wake_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    bool is_warm_boot = (wake_cause != ESP_SLEEP_WAKEUP_UNDEFINED)
                     && (g_rtc.magic == RTC_MAGIC);

    if (!is_warm_boot) {
        memset(&g_rtc, 0, sizeof(g_rtc));
        g_rtc.magic = RTC_MAGIC;
    }
    g_rtc.boot_count++;
    ESP_LOGI(TAG, "Boot #%lu (%s)", g_rtc.boot_count,
             is_warm_boot ? "warm" : "cold");

    // === Step 3: Create IPC ===
    create_all_ipc();

    // === Step 4: Create Tasks ===
    // Power task first (starts monitoring immediately)
    xTaskCreate(power_mgmt_task, "power", 2048, NULL, 1, NULL);

    // Display task (user feedback ASAP)
    xTaskCreate(display_task, "display", 8192, NULL, 4, NULL);

    // Camera task
    xTaskCreate(camera_task, "camera", 4096, NULL, 5, NULL);

    // IMU task
    xTaskCreate(imu_task, "imu", 2048, NULL, 4, NULL);

    // Detection task (created suspended, loaded on demand)
    TaskHandle_t detect_handle;
    xTaskCreate(detection_task, "detect", 8192, NULL, 3, &detect_handle);
    
    // Physics task (suspended, started when game starts)
    TaskHandle_t physics_handle;
    xTaskCreate(physics_task, "physics", 3072, NULL, 3, &physics_handle);

    // Game render task (suspended, started when game starts)
    TaskHandle_t render_handle;
    xTaskCreate(game_render_task, "render", 4096, NULL, 3, &render_handle);

    // === Step 5: Become Main Control Task ===
    // Pass task handles so main control can manage lifecycle
    main_control_params_t params = {
        .detect_handle  = detect_handle,
        .physics_handle = physics_handle,
        .render_handle  = render_handle,
        .is_warm_boot   = is_warm_boot,
    };
    main_control_task(&params);

    // Never reached - main_control_task runs forever
}
```

### 9.4 Main Control Task 核心逻辑

```c
void main_control_task(void *pvParams)
{
    main_control_params_t *p = (main_control_params_t *)pvParams;

    // --- 初始化 ---
    if (p->is_warm_boot) {
        ESP_LOGI(TAG, "Main: Warm boot — skip full init");
        display_set_status("Ready! (warm boot)", 0x00FF00);
    } else {
        ESP_LOGI(TAG, "Main: Cold boot — full init");
        display_set_status("Ready! Press DETECT or GAME", 0x00FF00);
    }

    // --- 主事件循环 ---
    while (1) {
        event_t evt;
        // 阻塞等待 — 无事件时Main Task阻塞 → 贡献Light-sleep!
        if (xQueueReceive(g_event_q, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // 任何事件都视为用户活动
        notify_power_user_activity();

        switch (evt.type) {

        case EVT_BTN_GAME:
            if (g_game_state == GAME_STATE_IDLE) {
                // 进入捕获阶段
                g_game_state = GAME_STATE_CAPTURING;
                esp_pm_lock_acquire(g_pm_lock_high);  // 禁止睡眠
                send_cmd(g_camera_cmd_q, CMD_CAMERA_CAPTURE, NULL);
                display_set_status("Game: Capturing...", 0x00FF00);
            } else if (g_game_state == GAME_STATE_PLAYING ||
                       g_game_state == GAME_STATE_WIN ||
                       g_game_state == GAME_STATE_LOSE) {
                // 退出游戏
                send_cmd_to_physics(CMD_PHYSICS_STOP);
                send_cmd_to_render(CMD_RENDER_STOP);
                display_exit_game_mode();
                pixel_world_destroy();
                g_game_state = GAME_STATE_IDLE;
                esp_pm_lock_release(g_pm_lock_high);   // 允许睡眠
                display_set_status("Ready!", 0x00FF00);
            }
            break;

        case EVT_BTN_DETECT:
            if (g_game_state == GAME_STATE_IDLE) {
                esp_pm_lock_acquire(g_pm_lock_high);
                send_cmd(g_camera_cmd_q, CMD_CAMERA_CAPTURE, NULL);
                display_set_status("Status: Capturing...", 0x00FF00);
            }
            break;

        case EVT_DETECTION_COMPLETE:
            // 检测完成 → 更新预览 → 释放锁
            update_preview_from_results(evt.data);
            esp_pm_lock_release(g_pm_lock_high);
            break;

        case EVT_GAME_WIN:
            g_game_state = GAME_STATE_WIN;
            display_show_game_end(true);
            break;

        case EVT_GAME_LOSE:
            g_game_state = GAME_STATE_LOSE;
            display_show_game_end(false);
            break;

        case EVT_TOUCH_SCREEN:
            // 从任何睡眠状态唤醒 → 恢复显示
            display_wakeup();
            notify_power_user_activity();
            break;

        default:
            break;
        }
    }
}
```

### 9.5 状态机对比

**旧架构 (单线程)**:
```
while(1) {
    switch(state) { ... }           // ← 状态机
    if (display_detect_triggered()) // ← 轮询
        { ...大量内联处理... }
    if (display_is_edge_view())     // ← 轮询
        { ...大量内联处理... }
    else if (display_is_live_view())// ← 轮询
        { ...大量内联处理... }
    vTaskDelay(50ms);              // ← 纯延时
}
// 问题: 状态机逻辑、IO处理、渲染全部耦合
```

**新架构 (事件驱动+多任务)**:
```
Main Task:  while(1) { xQueueReceive(&evt); process(evt); }
Camera:     while(1) { xQueueReceive(&cmd);  handle(cmd);  }
Display:    while(1) { vTaskDelayUntil();    lv_timer_handler(); }
// 各任务独立阻塞 → 无事可做 → Auto Light-sleep
```

---

## 十、关键Kconfig配置汇总

在项目根目录 `sdkconfig.defaults` 中添加:

```ini
# ===== Power Management =====
CONFIG_PM_ENABLE=y
CONFIG_PM_DFS_INIT_AUTO=y
CONFIG_PM_PROFILING=n

# ===== FreeRTOS Tickless Idle =====
CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
CONFIG_FREERTOS_IDLE_TIME_BEFORE_SLEEP=3

# ===== Light-sleep Optimization =====
CONFIG_PM_SLP_IRAM_OPT=y
CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP=y
CONFIG_ESP_SLEEP_POWER_DOWN_FLASH=n
CONFIG_ESP_SLEEP_FLASH_LEAKAGE_WORKAROUND=y

# ===== RTC Clock =====
CONFIG_RTC_CLK_SRC_EXT_CRYS=y
CONFIG_RTC_CLK_CAL_CYCLES=1024

# ===== Bootloader (快速唤醒) =====
CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP=y
CONFIG_BOOTLOADER_LOG_LEVEL_NONE=y

# ===== Compiler (减少固件体积, 加快加载) =====
CONFIG_COMPILER_OPTIMIZATION_SIZE=y

# ===== FreeRTOS (任务支持) =====
CONFIG_FREERTOS_HZ=1000
CONFIG_FREERTOS_NUMBER_OF_CORES=2
```

---

## 十一、总结

### 核心思路

| 维度 | 旧架构 | 新架构 |
|------|--------|--------|
| **线程模型** | 单线程 while(1) 轮询 | 8个任务, 事件驱动, 按需运行 |
| **功耗** | CPU永不休眠, ~770mA | Auto Light-sleep, 空闲 ~30mA |
| **双核利用** | 仅用1核 | 相机/渲染/检测并行 |
| **时序** | 帧率随检测耗时波动 | 物理100Hz, IMU 200Hz, 渲染30fps各自独立 |
| **可维护性** | 630行main.c, goto, 嵌套if | 按模块拆分, 独立增删 |
| **启动** | 冷启动3-4秒 | 冷启动相同; Warm boot 1-2秒 |

### 实施风险与缓解

| 风险 | 缓解 |
|------|------|
| 重构引入回归Bug | Phase 1仅重构不改逻辑, 功能对比测试 |
| 多任务竞态条件 | Queue/EventGroup天然线程安全; 共享状态用互斥锁 |
| 内存峰值增加 | 按需分配+释放; 峰值仅增加8个任务栈(~36KB) |
| 调试复杂度 | 每个任务独立日志TAG, FreeRTOS trace工具 |
| LVGL线程安全 | 所有LVGL操作集中在Display Task, bsp_display_lock保护 |

### 后续优化空间

1. **LP Core**: 在Deep-sleep中用LP RISC-V核心监控GT911触摸, 主CPU完全掉电
2. **相机ISP降频**: 非检测场景(纯预览)可降低ISP时钟
3. **模型量化**: ESP-DL int8量化模型比float32省50%推理时间和内存带宽
4. **动态背光**: 利用环境光传感器(若板载)自动调节亮度
5. **分区表优化**: 模型放在OTA分区, 支持在线更新

---

*文档生成: 2026-07-06*
*基于 ESP-IDF v5.5.4 + ESP32-P4 + Waveshare DEV-KIT*
