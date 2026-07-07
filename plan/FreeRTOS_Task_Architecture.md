# 智绘灵境 (AR Interactive Sandbox) — FreeRTOS 任务架构与通信总结

## 项目概述

本项目 "智绘灵境" 是一个基于 ESP32-P4 的 AR 互动沙盘系统。通过摄像头捕获真实桌面画面，利用边缘检测和目标检测（COCO/ESP-DL）构建游戏世界，玩家倾斜设备（IMU）控制弹珠在有物理碰撞的赛道中滚动。项目采用 FreeRTOS 多任务架构，将工作负载分配到 ESP32-P4 的两个 HP 核上。

---

## 双核任务分配

```
Core 0 (外设 I/O 核)              Core 1 (计算内核)
┌──────────────────────┐          ┌──────────────────────────┐
│ Camera Task  (prio 5)│          │ Main Control Task (prio 2)│
│ Display Task (prio 4)│          │ Marble Physics   (prio 3)│
│ IMU Task     (prio 4)│          │                          │
│ Power Mgmt   (prio 1)│          │                          │
└──────────────────────┘          └──────────────────────────┘
```

---

## 一、任务详细说明

### 1. Camera Task — 相机帧捕获任务

| 属性 | 值 |
|------|-----|
| 文件 | `main/tasks/camera_task.c` |
| 入口函数 | `camera_task()` |
| 运行核心 | Core 0 |
| 优先级 | 5 (最高) |
| 栈大小 | 4096 bytes |

**实现内容：**
- 阻塞等待 `g_capture_request` 二元信号量（由 Main Control Task 发出捕获请求）
- 调用 `camera_capture_frame()` 执行 V4L2 DQBUF 阻塞式帧捕获（超时 2000ms）
- 将捕获到的帧通过 `g_frame_response_q` 队列发送给 Main Control Task
- 相机初始化在 `app_main` 中完成，此任务仅负责运行时帧捕获

**作用：** 将 V4L2 阻塞式帧捕获操作隔离在独立任务中，避免阻塞主控任务的游戏逻辑循环。

---

### 2. Display Task — LVGL 显示心跳任务

| 属性 | 值 |
|------|-----|
| 文件 | `main/tasks/display_task.c` |
| 入口函数 | `display_task()` |
| 运行核心 | Core 0 |
| 优先级 | 4 |
| 栈大小 | 8192 bytes |

**实现内容：**
- 以约 30fps（33ms 周期）调用 `lv_timer_handler()` 驱动 LVGL 图形库
- 驱动内容包括：LVGL 定时器（动画）、显示刷新、触摸输入处理（GT911）
- 启动后通过 `g_sys_events` 事件组设置 `SYS_EVT_DISPLAY_READY` 标志，通知 `app_main` 显示已就绪
- 使用 `bsp_display_lock/unlock` 保护 LVGL 上下文，允许其他任务安全调用 `display_*()` 函数

**作用：** 为 LVGL 提供稳定的心跳节拍，确保 UI 动画和触摸响应正常工作。显示初始化在 `app_main` 中完成，此任务仅负责周期性 LVGL 维护。

---

### 3. IMU Task — 姿态采样与发布任务

| 属性 | 值 |
|------|-----|
| 文件 | `main/tasks/imu_task.c` |
| 入口函数 | `imu_task()` |
| 运行核心 | Core 0 |
| 优先级 | 4 |
| 栈大小 | 4096 bytes |

**实现内容：**
- 以固定频率（默认 200Hz，周期 5ms）通过 I2C 读取 IMU 传感器数据
- 运行卡尔曼滤波姿态解算（互补滤波，系数 0.98）
- 将最新姿态数据通过 `g_imu_attitude_q` 队列以 overwrite 模式发布（队列深度为 1，始终保留最新数据）
- 支持动态采样周期调整：当 Power Management Task 修改 `g_imu_period_ms` 时，自动适配新频率（省电模式下可降至 10Hz）
- 启动后通过 `g_sys_events` 设置 `SYS_EVT_IMU_READY` 标志

**作用：** 以固定高频采集姿态数据，为弹珠物理引擎和 HUD 显示提供实时的 roll/pitch 角度。IMU 初始化在 `app_main` 中完成，此任务仅负责运行时数据采集。

---

### 4. Power Management Task — 电源管理任务

| 属性 | 值 |
|------|-----|
| 文件 | `main/tasks/power_mgmt_task.c` |
| 入口函数 | `power_mgmt_task()` |
| 运行核心 | Core 0 |
| 优先级 | 1 (最低) |
| 栈大小 | 2048 bytes |

**实现内容：**
- 以 10Hz（100ms 周期）轮询用户空闲时间（距上次用户活动的时间）
- 实现三层省电状态机：

| 状态 | 空闲时间 | 动作 |
|------|---------|------|
| `PM_STATE_ACTIVE` | < 30s | 全速运行，背光 100% |
| `PM_STATE_DIM` | 30s ~ 60s | 背光降至 20% |
| `PM_STATE_DEEP_SLEEP` | >= 60s | 背光关闭 + 相机停流（STREAMOFF）+ IMU 降至 10Hz |

- 提供 `notify_user_activity()` 函数，由按钮回调和主控状态转换调用，重置空闲计时器
- 提供 `pm_resume_all()` 函数，快速恢复所有外设（背光 100%、相机重新 STREAMON、IMU 恢复 200Hz）
- 相机暂停前设置 `g_camera_paused = true`，等待 100ms 让进行中的 DQBUF 完成后再 STREAMOFF
- 输出阶段性空闲日志（30s / 60s / 120s）

**作用：** 在用户不操作时自动降低系统功耗，延长电池续航。空闲 < 5s 时自动恢复正常状态。

---

### 5. Main Control Task — 主控任务

| 属性 | 值 |
|------|-----|
| 文件 | `main/tasks/main_control_task.c` |
| 入口函数 | `main_control_task()` |
| 运行核心 | Core 1 |
| 优先级 | 2 |
| 栈大小 | 12288 bytes |

**实现内容：** 这是项目最核心的任务，管理游戏的完整生命周期。

#### 5.1 游戏状态机

```
IDLE → CAPTURING → PLAYING → WIN/LOSE → IDLE
```

- **IDLE**: 空闲状态，响应用户按钮操作（Live View / Edge View / Detect / Game）
- **CAPTURING**: 像素游戏捕获阶段——拍照 → 边缘检测（Canny）→ COCO 目标检测 → 构建像素世界 → 初始化游戏模式显示 → 启动物理引擎
- **PLAYING**: 游戏运行阶段——每帧渲染像素世界（30fps）→ 更新 HUD（弹珠位置/速度/姿态/穿墙Buff/弹力系数）→ 检测胜负条件
- **WIN/LOSE**: 游戏结束，显示结果画面 3 秒后自动回到 IDLE

#### 5.2 用户交互模式

| 模式 | 触发方式 | 功能 |
|------|---------|------|
| Live View | 按钮 | 实时相机预览 + 赛道/弹珠叠加渲染 |
| Edge View | 按钮 | 边缘检测预览模式，Canny 灰度图显示 |
| Track Capture | 按钮 | 在 Edge View 中重新捕获赛道（边缘检测 + 轨道构建 + 物体轮廓提取） |
| Detect | 按钮 | 单次目标检测：拍照 → 可选预处理 → COCO 检测 → 结果叠加显示 |
| Game Capture | 按钮 | 切换到像素游戏模式（IDLE → CAPTURING） |

#### 5.3 业务编排

Main Control Task 负责协调所有子模块：
- **相机帧请求**: 通过 `camera_capture_via_task()` → Camera Task
- **IMU 姿态获取**: 通过 `imu_get_attitude_via_task()` → IMU Task
- **边缘检测**: Canny 算法 (`edge_detect_run()`)，降采样到 400×320
- **目标检测**: ESP-DL COCO 模型推理 (`detection_run()`)
- **赛道构建**: 从边缘图构建轨道碰撞结构 (`track_build_from_edges()`)
- **像素世界构建**: 从边缘图 + 检测结果构建游戏世界 (`pixel_world_build()`)
- **渲染**: 调用 `game_render_pixel_frame()` / `game_render_frame()` 绘制到显示缓冲区
- **预处理管线**: 支持去噪/直方图均衡/Gamma/锐化/对比度拉伸等预处理组合

**作用：** 整个系统的"大脑"，管理游戏状态机、响应用户交互、编排所有子模块完成检测/渲染/物理模拟流程。

---

### 6. Marble Physics Task — 弹珠物理引擎任务（隐式创建）

| 属性 | 值 |
|------|-----|
| 文件 | `main/physics/marble_physics.c` |
| 入口函数 | `physics_task()` (内部 static) |
| 创建方式 | `marble_physics_init()` 内部通过 `xTaskCreatePinnedToCore` 创建 |
| 运行核心 | Core 1 |
| 优先级 | 3 |
| 栈大小 | 12288 bytes |

**实现内容：**
- 以 100Hz（10ms 周期）运行半隐式欧拉积分器
- 从 `g_imu_attitude_q` 非阻塞读取最新姿态（xQueuePeek），将 roll/pitch 角度映射为加速度
- 5 度死区过滤微小倾斜
- 摩擦衰减：每帧速度乘以 `(1 - friction * dt)`
- 24 点圆周采样进行赛道墙壁碰撞检测，速度反射 + 位置推开
- 书墙破坏检测：速度超过阈值时摧毁书墙瓦片
- 地图边界反弹保护
- 速度上限钳制（`MARBLE_MAX_SPEED`）
- 3D 金属球渲染（带光照、高光、旋转纹理、投影）
- 通过 `portMUX_TYPE` 自旋锁保护弹珠状态
- 支持游戏回调注册：每物理 tick 调用注册的回调函数（用于像素游戏物体交互）

**作用：** 独立的物理模拟循环，以固定 100Hz 运行，不依赖渲染帧率。与 Main Control Task 均在 Core 1 运行，确保计算密集型操作集中在同一核心。

---

## 二、任务间通信机制

### 通信拓扑图

```
                    ┌─────────────────────────────────────────────┐
                    │              Main Control Task               │
                    │              (Core 1, Prio 2)                │
                    └──┬──────┬──────────┬──────────┬─────────────┘
                       │      │          │          │
          camera_capture_via_task()      │          │
           │ give         │ recv         │          │
           ▼              ▲              │          │
    ┌──────────┐    ┌──────────┐         │          │
    │g_capture │    │g_frame   │         │          │
    │_request  │    │_response │         │          │
    │(Binary   │    │_q (Queue │         │          │
    │ Semaphore│    │  depth 2)│         │          │
    └────┬─────┘    └────┬─────┘         │          │
         ▼               ▲              │          │
    ┌─────────────────────────┐          │          │
    │     Camera Task          │          │          │
    │   (Core 0, Prio 5)       │          │          │
    └─────────────────────────┘          │          │
                                         │          │
              imu_get_attitude_via_task()│          │
                       │                 │          │
                       │ xQueuePeek      │          │
                       ▼                 │          │
                ┌──────────────┐         │          │
                │ g_imu_attitude│◄────────┤          │
                │ _q (Queue 1, │         │          │
                │  overwrite)  │         │          │
                └──────┬───────┘         │          │
                       ▲                 │          │
                       │ xQueueOverwrite │          │
                       │                 │          │
                ┌──────────────┐         │          │
                │   IMU Task    │         │          │
                │ (Core 0,Prio 4)│        │          │
                └──────────────┘         │          │
                                         │          │
             marble_physics_init()       │          │
              creates internally ────────┘          │
                       │                            │
                       │ xQueuePeek                  │
                       ▼                            │
                ┌──────────────┐                    │
                │ g_imu_attitude│                    │
                │ _q (shared)  │                    │
                └──────────────┘                    │
                       │                            │
                       ▼                            │
                ┌──────────────────┐                │
                │ Marble Physics    │                │
                │ (Core 1, Prio 3)  │                │
                └──────────────────┘                │
                                                    │
    ┌──────────────────┐                            │
    │  Power Mgmt Task  │────────────────────────────┘
    │  (Core 0, Prio 1) │  volatile globals:
    └──────────────────┘  g_camera_paused, g_imu_period_ms
                                                    │
    ┌──────────────────┐                            │
    │  Display Task     │  g_sys_events:             │
    │  (Core 0, Prio 4) │  SYS_EVT_DISPLAY_READY     │
    └──────────────────┘  SYS_EVT_IMU_READY          │
                          SYS_EVT_GAME_ACTIVE         │
```

### 通信机制详解

#### 2.1 `g_capture_request` — 二元信号量

| 属性 | 值 |
|------|-----|
| 类型 | `SemaphoreHandle_t` (Binary Semaphore) |
| 创建 | `xSemaphoreCreateBinary()` |

**通信方向：** Main Control Task → Camera Task

**工作流程：**
1. Main Control Task 调用 `camera_capture_via_task()` → 内部执行 `xSemaphoreGive(g_capture_request)`
2. Camera Task 阻塞在 `xSemaphoreTake(g_capture_request, portMAX_DELAY)`，收到信号后调用 `camera_capture_frame()` 捕获一帧
3. 检查 `g_camera_paused` 标志：若为 true（省电模式），拒绝捕获请求并返回 `ESP_ERR_INVALID_STATE`

---

#### 2.2 `g_frame_response_q` — 帧数据队列

| 属性 | 值 |
|------|-----|
| 类型 | `QueueHandle_t` |
| 深度 | 2 |
| 元素类型 | `camera_frame_t` |

**通信方向：** Camera Task → Main Control Task

**工作流程：**
1. Camera Task 捕获帧后，通过 `xQueueSend(g_frame_response_q, &frame, 100ms)` 发送
2. Main Control Task 的 `camera_capture_via_task()` 调用 `xQueueReceive(g_frame_response_q, &frame, timeout_ms)` 阻塞等待
3. 若队列满，Camera Task 丢弃帧并打印警告

---

#### 2.3 `g_imu_attitude_q` — IMU 姿态队列

| 属性 | 值 |
|------|-----|
| 类型 | `QueueHandle_t` |
| 深度 | 1 (overwrite 模式) |
| 元素类型 | `imu_attitude_t` |

**通信方向：** IMU Task → Main Control Task / Marble Physics Task (多消费者)

**工作流程：**
1. IMU Task 以 200Hz 频率调用 `xQueueOverwrite(g_imu_attitude_q, &att)` 写入最新姿态（队列满时覆盖旧数据）
2. Main Control Task 通过 `xQueuePeek(g_imu_attitude_q, att, 0)` 非阻塞读取最新姿态
3. Marble Physics Task 同样通过 `xQueuePeek` 非阻塞读取（两个消费者各自独立 Peek，不消费数据）
4. `xQueueOverwrite` 配合 `xQueuePeek` 实现"发布-订阅"模式，消费者获取最新值而不移除

---

#### 2.4 `g_sys_events` — 系统事件组

| 属性 | 值 |
|------|-----|
| 类型 | `EventGroupHandle_t` |
| 事件位 | 见下表 |

| 位定义 | 含义 | 设置者 | 等待者 |
|--------|------|--------|--------|
| `SYS_EVT_DISPLAY_READY` (bit 0) | 显示就绪 | Display Task | app_main |
| `SYS_EVT_IMU_READY` (bit 2) | IMU 就绪 | IMU Task | app_main |
| `SYS_EVT_GAME_ACTIVE` (bit 3) | 游戏进行中 | Main Control | Power Management (状态感知) |

**工作流程：**
- `app_main` 创建所有任务后，调用 `xEventGroupWaitBits()` 等待 IMU 和 Display 就绪（超时 3s）
- `SYS_EVT_GAME_ACTIVE` 用于标识游戏运行状态（进入 PLAYING 时置位，退出时清除）

---

#### 2.5 `g_event_q` — 通用事件队列

| 属性 | 值 |
|------|-----|
| 类型 | `QueueHandle_t` |
| 深度 | 20 |
| 元素类型 | `event_t` (type + data + timestamp) |

**通信方向：** UI 按钮回调 → Main Control Task（定义但当前代码中主要通过 `display_*_triggered()` 函数直接轮询按钮状态）

**事件类型（已定义）：**
```
EVT_BTN_LIVE_VIEW, EVT_BTN_EDGE_VIEW, EVT_BTN_GAME, EVT_BTN_DETECT,
EVT_BTN_PREPROC, EVT_TOUCH_SCREEN, EVT_GAME_CAPTURE_DONE,
EVT_GAME_WIN, EVT_GAME_LOSE, EVT_GAME_EXIT,
EVT_DETECTION_COMPLETE, EVT_DETECTION_FAILED,
EVT_SYSTEM_READY, EVT_DISPLAY_READY, EVT_CAMERA_READY,
EVT_IMU_READY, EVT_USER_ACTIVITY
```

---

#### 2.6 volatile 全局变量 — 省电控制

| 变量 | 类型 | 作用 |
|------|------|------|
| `g_camera_paused` | `volatile bool` | true 时 Camera Task 拒绝捕获请求；Power Mgmt 控制 |
| `g_imu_period_ms` | `volatile int` | IMU 采样周期，默认 5（200Hz），省电时改为 100（10Hz） |

**通信方向：** Power Management Task → Camera Task / IMU Task

这些变量绕过了 FreeRTOS IPC 机制，直接通过内存共享通信，适合简单的标志位/配置值传递。

---

#### 2.7 portMUX_TYPE 自旋锁 — 弹珠状态保护

| 变量 | 作用 |
|------|------|
| `g_lock` (static, marble_physics.c) | 保护 `g_marble` 结构体和穿墙 Buff 状态的临界区 |

Marble Physics Task 更新状态时持锁，Main Control Task 通过 `marble_physics_get_state()` 读状态时也持锁。

---

## 三、函数封装：跨任务调用

为简化跨任务通信，IPC 模块封装了两个便捷函数：

```c
// 通过 Camera Task 捕获一帧（阻塞等待）
esp_err_t camera_capture_via_task(camera_frame_t *frame, uint32_t timeout_ms);

// 获取最新 IMU 姿态（非阻塞）
esp_err_t imu_get_attitude_via_task(imu_attitude_t *att);
```

这些函数隐藏了底层的信号量/队列操作，使 Main Control Task 的代码保持简洁。

---

## 四、任务启动流程

`app_main` 中的初始化与任务创建顺序：

```
1. 创建所有 IPC 对象 (队列/信号量/事件组)
2. 初始化显示 (display_init)
3. 初始化相机 (camera_init)
4. 初始化预处理模块 (preprocessing_init)
5. 相机预热 (camera_warmup × 10 帧)
6. 初始化 IMU + 弹珠物理 (imu_init → marble_physics_init)
7. 创建 FreeRTOS 任务:
   └─ Camera Task    → Core 0, Prio 5
   └─ Display Task   → Core 0, Prio 4
   └─ IMU Task       → Core 0, Prio 4
   └─ Power Mgmt     → Core 0, Prio 1
8. 等待 Display + IMU 就绪 (超时 3s)
9. notify_user_activity() 初始化空闲计时器
10. 创建 Main Control Task → Core 1, Prio 2
    (Marble Physics Task 在 marble_physics_init() 中已创建 → Core 1, Prio 3)
```

---

## 五、关键设计要点

1. **双核分离**：Core 0 处理所有外设 I/O（相机 DMA、I2C/IMU、SPI/DSI 显示），Core 1 处理所有计算密集型任务（物理模拟、渲染、检测、边缘检测），避免 I/O 等待阻塞计算。

2. **发布-订阅模式**：IMU 数据使用 `xQueueOverwrite` + `xQueuePeek` 实现多消费者共享，Main Control Task 和 Marble Physics Task 各自独立获取最新姿态，无需同步。

3. **信号量同步捕获**：Camera Task 通过信号量按需触发帧捕获，而非持续轮询，避免不必要的 CPU 和总线占用。

4. **分层省电**：通过 `g_camera_paused` 和 `g_imu_period_ms` 两个 volatile 全局变量实现轻量级省电控制，避免引入额外的 FreeRTOS IPC 开销。

5. **固定物理步长**：Marble Physics Task 以固定 100Hz 运行，与渲染帧率（30fps）解耦，保证物理模拟的稳定性。

6. **游戏回调机制**：Marble Physics Task 支持注册游戏回调函数（`marble_physics_register_game_cb`），像素游戏物理模块（`pixel_physics`）通过此机制在每物理 tick 检测弹珠与游戏物体的交互（传送门、道具拾取、伤害区域等）。
