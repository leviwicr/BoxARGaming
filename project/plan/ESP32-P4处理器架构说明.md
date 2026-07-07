# ESP32-P4 处理器架构说明

## 一、芯片定位

ESP32-P4 是乐鑫 2024 年发布的高性能**边缘计算 SoC**，与以往 ESP32 系列有本质区别：

| 特性 | ESP32-S3 | ESP32-P4 |
|------|----------|----------|
| CPU 架构 | Xtensa LX7 双核 | **RISC-V HP 双核** 400MHz |
| 无线 | Wi-Fi 4 + BLE 5.0 | **无** (纯算力芯片) |
| NPU | 无 | 无 (依赖 CPU 推理) |
| PSRAM | 8/16MB OPI | **16/32MB XIP** (代码可在 PSRAM 执行) |
| 显示 | 并行 RGB | **MIPI-DSI** 1024×800 |
| 相机 | DVP 8-bit | **MIPI-CSI** 2-lane |
| 定位 | IoT 连接 | 边缘 AI + HMI 交互 |

P4 是乐鑫首款**去掉无线连接**、全力堆算力和多媒体接口的芯片，对标 NXP i.MX RT 系列。

---

## 二、CPU 核心架构

### 2.1 三核结构

```
┌─────────────────────────────────────────────────────┐
│                   ESP32-P4                          │
│                                                     │
│  ┌──────────────┐  ┌──────────────┐                 │
│  │  HP Core 0   │  │  HP Core 1   │  400MHz RISC-V │
│  │  (PRO_CPU)   │  │  (APP_CPU)   │  RV32IMAFV     │
│  │  16KB L1-I   │  │  16KB L1-I   │                 │
│  │  64KB L1-D   │  │  64KB L1-D   │                 │
│  └──────┬───────┘  └──────┬───────┘                 │
│         └────────┬────────┘                         │
│                  ▼                                  │
│  ┌──────────────────────────────┐                   │
│  │  共享 L2 Cache (128-512KB)   │  8-way            │
│  └──────────────────────────────┘                   │
│                                                     │
│  ┌──────────────────────────────┐                   │
│  │  LP Core (RISC-V 40MHz)      │  超低功耗协处理器  │
│  │  32KB LP SRAM                │  (deep-sleep 期间  │
│  └──────────────────────────────┘   可独立运行)       │
└─────────────────────────────────────────────────────┘
```

### 2.2 HP 双核详细信息

- **指令集**: RV32IMAFV — 32 位 RISC-V，支持整数(I)、乘除(M)、原子(A)、单精度浮点(F)、矢量(V)扩展
- **频率**: 最高 400MHz，支持 DVFS 动态调频 (40-400MHz)
- **对称性**: 两个 HP 核完全相同 (SMP)，任何代码可在任一核执行

**Core 0 的特殊职责**:
- 系统 Tick 中断仅由 Core 0 维护 —— 所有超时、延时、时间片轮转的时钟源
- Core 0 调度器长时间挂起会导致系统时间漂移

**Core 1**: 与 Core 0 完全对称的计算能力，无额外职责。

### 2.3 LP Core（低功耗协处理器）

- RISC-V 32 位 @ 40MHz
- 32KB 专用 SRAM
- 在 HP 双核 deep-sleep 期间可独立运行
- 典型用途：触摸传感器轮询、GPIO 唤醒检测、简单传感器数据采集

---

## 三、内存层级

```
┌────────────┐  延迟    │  容量      │  说明
├────────────┼──────────┼────────────┼──────────────────────
│ TCM        │  0 周期  │  8KB       │  紧耦合内存，零等待
│ L1 I-Cache │  ~1 周期 │  16KB×2    │  每核独立，4-way
│ L1 D-Cache │  ~2 周期 │  64KB×2    │  每核独立，2-way
│ L2 Cache   │  ~5 周期 │  128-512KB │  双核共享，8-way
│ HP SRAM    │  ~10 周期│  768KB     │  片上 SRAM (L2MEM)
│ PSRAM      │  ~50 周期│  16/32MB   │  片外 XIP
│ LP SRAM    │  -       │  32KB      │  LP Core 专用
│ HP ROM     │  -       │  128KB     │  启动 ROM
└────────────┘
```

**关键特性**:
- PSRAM 支持 **XIP** (Execute In Place)：代码可直接在 PSRAM 中运行，不需要先加载到 SRAM
- L2 Cache 可配置 128/256/512KB（本项目配置 256KB）
- L2 Cache Line 可配置 64/128B（本项目配置 128B）

---

## 四、Cache 一致性

**P4 没有硬件 Cache 一致性协议** (无 MESI/MOESI)。

两个 HP 核各有一套独立的 L1 Cache，共享 L2。当两核同时写同一块内存时：

| 场景 | 风险 |
|------|------|
| L1 D-Cache write-back | 核 A 修改在 L1 中未写回，核 B 读到 L2 中的旧值 |
| L1 D-Cache write-through | 写操作直达 L2，核 B 能读到新值 |

**安全做法**:
1. 使用 `portMUX_TYPE` spinlock 保护所有跨核共享的可变数据
2. 不要依赖 "benign race" 或假设硬件会帮你维护一致性
3. ESP-IDF 提供的 spinlock (`taskENTER_CRITICAL` / `taskEXIT_CRITICAL`) 同时禁用本核中断 + 获取自旋锁

---

## 五、SMP 调度特性 (ESP-IDF FreeRTOS)

### 5.1 调度规则

- **固定优先级抢占** + 同优先级时间片轮转
- 每个核独立从就绪队列中选取最高优先级、兼容亲和性的任务
- Tick 中断两核都触发，但**仅 Core 0 递增 tick 计数器**

### 5.2 SMP 特有的陷阱

| 陷阱 | 说明 |
|------|------|
| **同优先级跨核饥饿** | 如果两个同优先级任务分别钉在 Core 0 和 Core 1，调度器的共享就绪链表指针可能导致其中一个核始终跳过该任务，造成饥饿。解决：同优先级任务放在同一核，或错开优先级 |
| **vTaskSuspendAll 只影响本核** | 它不是跨核互斥机制，不能用来保护共享数据 |
| **FPU 懒惰上下文切换** | 使用 `float` 的任务会被自动钉在首次运行的核上。应显式 `xTaskCreatePinnedToCore()` 避免意外 |
| **double 是软浮点** | P4 只支持单精度(float)硬件浮点，double 走软件模拟，性能极差 |
| **vTaskDelay 不能跨核同步** | 两核的 tick 同频但不同相，delay 到期时间有微小偏差。跨核同步用队列或信号量 |
| **跨核删除任务** | 被删任务如果持有互斥锁，可能死锁。避免跨核 `vTaskDelete` |
| **Core 0 调度器禁用的连锁反应** | Core 0 长时间禁用调度会导致系统时钟漂移 |

### 5.3 跨核通信推荐方式

```c
// 队列: 跨核传递数据，线程安全，支持阻塞等待
QueueHandle_t q = xQueueCreate(10, sizeof(data_t));
xQueueSend(q, &data, 0);              // Core 0 发送，非阻塞
xQueueReceive(q, &data, portMAX_DELAY); // Core 1 接收，阻塞等待

// Spinlock: 保护共享内存
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
taskENTER_CRITICAL(&mux);
shared_var = new_value;  // 本核中断禁止 + 另一核 spin 等待
taskEXIT_CRITICAL(&mux);

// 信号量: 跨核事件通知
SemaphoreHandle_t sem = xSemaphoreCreateBinary();
xSemaphoreGive(sem);  // 一个核通知
xSemaphoreTake(sem, portMAX_DELAY);  // 另一个核等待
```

---

## 六、多媒体外设

### 6.1 MIPI-CSI (相机接口)

- 2-lane MIPI CSI-2，最高 800Mbps/lane
- 通过 esp_video 框架封装为 V4L2 设备 (`/dev/video0`)
- CSI 控制器 + ISP (Image Signal Processor) 由硬件 DMA 管理，**不占 CPU 周期**
- ISP 管线: RAW8 → RGB565，支持 AE (自动曝光)、AWB (自动白平衡)、Gamma 校正

### 6.2 MIPI-DSI (显示接口)

- 支持最高 1024×800 分辨率
- 通过 LVGL 库驱动，硬件 SPI/DSI DMA 传输

### 6.3 其他

- **I2C**: 多组 I2C 总线，连接触摸 (GT911)、IMU (0x23)、背光 PWM (0x45) 等
- **SD/MMC**: 4-bit SDIO
- **USB**: USB 2.0 OTG
- **Ethernet**: RMII 接口

---

## 七、功耗特性

| 状态 | CPU | 外设 | 典型电流 | 退出延迟 |
|------|-----|------|----------|----------|
| 全速运行 | 400MHz 双核 | 全部开启 | ~770mA | - |
| DFS 降频 | 40MHz | 全部开启 | ~500mA | <10μs |
| Light-sleep | 时钟门控 | 可选择性保持 | ~30mA | ~300μs |
| Deep-sleep | 完全断电 | 仅 RTC + LP Core | ~5mA | ~1-2s |

**注意**: ESP-IDF v5.5 的 SMP FreeRTOS 模式下 `esp_pm_configure()` 的 Auto Light-sleep 支持仍有限制。本项目采用**外设级省电**代替 CPU 级省电（见 Phase 3 实现）。

---

## 八、编程建议总结

| 类别 | 建议 |
|------|------|
| **任务分配** | Core 0: 外设 I/O (Camera/Display/IMU)；Core 1: 计算 (推理/渲染/物理) |
| **浮点代码** | 用 `float` 而非 `double`；FPU 任务显式钉核 |
| **共享数据** | 始终用 spinlock 保护；不依赖 cache 一致性 |
| **同优先级** | 避免钉在不同核；要么同核，要么错开优先级 |
| **跨核同步** | 用队列/信号量，不用 vTaskDelay |
| **关键区** | 尽量短，内部不调用阻塞 API |
| **内存** | 大缓冲放 PSRAM (heap_caps_calloc)，小对象放 SRAM |
| **编译优化** | `CONFIG_COMPILER_OPTIMIZATION_PERF=y` 开启 -O2 |

---

*基于 ESP-IDF v5.5.4 + ESP32-P4 调研整理，2026-07-06*
