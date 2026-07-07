# 芯片音乐音频系统设计方案

> 版本: v1.0 | 日期: 2026-07-07

---

## 1. 硬件概况

| 组件 | 型号/引脚 | 说明 |
|------|----------|------|
| 音频 Codec | ES8311 (I2C: 0x18) | 单声道 DAC+ADC |
| 功放 | NS4150B, PA使能 GPIO 53 | 3W Class-D |
| I2S | SCLK=12, MCLK=13, LCLK=10, DOUT=9, DSIN=11 | I2S #1, Master, 22050Hz/16bit/Mono |
| I2C | SCL=8, SDA=7 | 与触摸控制器共享 |

BSP 已提供 `bsp_audio_init()`, `bsp_audio_codec_speaker_init()` 封装。

---

## 2. 整体架构

```
Core 0 (外设核)                          Core 1 (计算核)
┌──────────────────┐                    ┌─────────────────────┐
│  Audio Task      │◄──g_audio_cmd_q───│  Main Control Task   │
│  (prio 3, 4KB)   │                   │  Pixel Physics       │
│                  │                   │  (发送音效事件)       │
│  ┌────────────┐  │                    └─────────────────────┘
│  │ PSG Synth   │  │
│  │ 4-channel   │  │
│  │ Ch1: Pulse  │  │
│  │ Ch2: Pulse  │  │
│  │ Ch3: Tri    │  │
│  │ Ch4: Noise  │  │
│  └─────┬──────┘  │
│        │          │
│  ┌─────▼──────┐  │
│  │ DMA Buffer  │──┼──► I2S ──► ES8311 ──► Speaker
│  │ (双缓冲)    │  │
│  └────────────┘  │
└──────────────────┘
```

### 任务分配

| 核 | 已有任务 | 新增 Audio 后 |
|----|---------|---------------|
| Core 0 | Camera(5), Display(4), IMU(4), Power(1) | + Audio(3) |
| Core 1 | MainCtrl(2), Physics(3) | 不变 |

Audio Task 优先级 3，高于 Power(1) 低于 IMU/Display(4)，保证音频实时性。

---

## 3. PSG 合成器设计 (4通道)

参考 NES APU 风格，每个通道独立生成样本，最终混音输出。

### 3.1 通道定义

| 通道 | 波形 | 参数 | 用途 |
|------|------|------|------|
| CH1 Pulse1 | 方波 | freq, duty(12.5/25/50/75%), ADSR vol | 主旋律 / SFX |
| CH2 Pulse2 | 方波 | freq, duty(12.5/25/50/75%), ADSR vol | 和弦 / SFX |
| CH3 Triangle | 三角波 | freq, ADSR vol | 低音 / BGM 伴奏 |
| CH4 Noise | LFSR噪声 | mode(短/长周期), ADSR vol | 碰撞/爆炸/鼓点 |

### 3.2 ADSR 包络

```
 vol ^
     |   A    D
     |   /\   /\____ S (sustain level)
     |  /  \ /  \   \____
     | /    \    \        \  R
     |/      \    \        \_____
     +---------------------------> time
```

每个通道有独立的 A(Attack), D(Decay), S(Sustain), R(Release) 参数 (单位: ms)。

### 3.3 输出配置

- **采样率**: 22050 Hz
- **位深**: 16 bit signed
- **声道**: Mono
- **缓冲区**: 双缓冲, 每块 512 samples (~23ms延迟)
- **DMA 回调**: 填充下一个缓冲块, 触发合成器渲染

---

## 4. 音效定义

| 音效 ID | 触发事件 | 参数 |
|---------|---------|------|
| SFX_WALL_BOUNCE | 弹珠碰墙 | CH4 短噪声(30ms) + CH1 降频 sweep |
| SFX_FRUIT_PICKUP | 捡到水果 | CH1 上行琶音 C-E-G-C (各50ms) |
| SFX_PORTAL | 传送门传送 | CH1 频率滑升(200ms) + CH4 wide noise |
| SFX_DEATH | 触碰死亡物体 | CH1 下行滑音(300ms) |
| SFX_WIN | 到达目标 | CH1+CH2+CH3 上行大三和弦(500ms) |
| SFX_LOSE | 游戏结束 | CH1 下行小三和弦(500ms) |

---

## 5. BGM 设计

游戏 PLAYING 状态下循环播放的 8-bit 风格旋律。

### 5.1 音符格式

```c
// 音高: 用 MIDI 音符号 (0-127, 0=静音)
// 时值: 1/16 拍为一个 tick, bpm ~120 → 31.25ms/tick
typedef struct {
    uint8_t note;   // MIDI note, 0=silence
    uint8_t ticks;  // duration in 1/16 beats
} bgm_note_t;

typedef struct {
    const bgm_note_t *notes;
    uint16_t length;     // total notes count
    uint16_t loop_start; // loop point (0=no loop)
} bgm_track_t;
```

### 5.2 BGM 结构

四轨并行:
- **Track1 (Pulse1)**: 主旋律, duty=50%
- **Track2 (Pulse2)**: 和声, duty=25%, 音量略低
- **Track3 (Triangle)**: 低音线, 八度低于旋律
- **Track4 (Noise)**: 简单鼓点 (kick/snare/hihat)

初始 BGM 为 16 小节的循环段，约 64 个音符/轨道。

---

## 6. IPC: 音频命令队列

### 6.1 队列

```c
// ipc.h 新增
extern QueueHandle_t g_audio_cmd_q;  // depth 16, audio_cmd_t
```

### 6.2 命令类型

```c
typedef enum {
    AUDIO_CMD_PLAY_SFX,       // 播放音效, 参数: sfx_type_t
    AUDIO_CMD_BGM_START,      // 开始/恢复 BGM
    AUDIO_CMD_BGM_STOP,       // 停止 BGM
    AUDIO_CMD_BGM_PAUSE,      // 暂停 BGM (电力管理)
    AUDIO_CMD_SET_MASTER_VOL, // 设置主音量 0-100
    AUDIO_CMD_MUTE,           // 静音
    AUDIO_CMD_UNMUTE,         // 取消静音
} audio_cmd_type_t;

typedef enum {
    SFX_WALL_BOUNCE,
    SFX_FRUIT_PICKUP,
    SFX_PORTAL,
    SFX_DEATH,
    SFX_WIN,
    SFX_LOSE,
    SFX_COUNT
} sfx_type_t;

typedef struct {
    audio_cmd_type_t cmd;
    sfx_type_t      sfx;   // for AUDIO_CMD_PLAY_SFX
    uint8_t         value; // for volume etc
} audio_cmd_t;
```

---

## 7. Audio Task 主循环

```
初始化:
  1. bsp_audio_init(NULL)         → I2S + ES8311 初始化
  2. bsp_audio_codec_speaker_init() → esp_codec_dev 句柄
  3. esp_codec_dev_open()          → 打开 22050Hz/16bit/Mono 输出
  4. synth_init()                  → PSG 合成器初始化
  5. 设置 I2S DMA 回调 → synth_render_block()

主循环:
  while(1) {
    xQueueReceive(g_audio_cmd_q, &cmd, portMAX_DELAY)
    switch(cmd.cmd) {
      AUDIO_CMD_PLAY_SFX   → synth_trigger_sfx(cmd.sfx)
      AUDIO_CMD_BGM_START  → bgm_start()
      AUDIO_CMD_BGM_STOP   → bgm_stop()
      AUDIO_CMD_BGM_PAUSE  → bgm_pause()
      AUDIO_CMD_SET_MASTER_VOL → synth_set_master_vol(cmd.value)
      AUDIO_CMD_MUTE       → esp_codec_dev_set_out_vol(handle, 0)
      AUDIO_CMD_UNMUTE     → esp_codec_dev_set_out_vol(handle, master_vol)
    }
  }

DMA回调 (I2S 需要数据时):
  synth_render_block(buf_16bit, 512 samples):
    for i in 0..511:
      sample = 0
      sample += ch1_render()  // Pulse1
      sample += ch2_render()  // Pulse2
      sample += ch3_render()  // Triangle
      sample += ch4_render()  // Noise
      buf[i] = clamp(sample, -32768, 32767)
```

---

## 8. 文件清单

### 新建文件

| 文件 | 说明 |
|------|------|
| `main/audio/audio_driver.h` | 音频硬件抽象: init, deinit, set_volume |
| `main/audio/audio_driver.c` | ES8311+I2S 初始化, I2S DMA 回调安装 |
| `main/audio/audio_synth.h` | PSG 合成器: 通道定义, ADSR, 混音 |
| `main/audio/audio_synth.c` | 4通道波形生成, sfx触发, 实时采样渲染 |
| `main/audio/bgm_data.h` | BGM 音符序列数据 (C 数组) |
| `main/audio/bgm_player.h` | BGM 播放器: 音符序列播放, 循环控制 |
| `main/audio/bgm_player.c` | BGM 播放引擎实现 |
| `main/audio/sfx_data.h` | 音效参数数据 |
| `main/tasks/audio_task.h` | Audio Task 声明 |
| `main/tasks/audio_task.c` | Audio Task 入口, 命令处理, DMA回调 |

### 修改文件

| 文件 | 修改内容 |
|------|---------|
| `main/ipc/ipc.h` | 添加 `audio_cmd_t`, `sfx_type_t`, `g_audio_cmd_q` 声明 |
| `main/ipc/ipc.c` | 添加 `g_audio_cmd_q` 定义 |
| `main/main.c` | 添加 audio init + Audio Task 创建, 新 IPC 队列创建 |
| `main/pixel_game/pixel_physics.c` | FRUIT/DEATH/GOAL/PORTAL/WALL 事件触发音效 |
| `main/tasks/main_control_task.c` | CAPTURING→PLAYING 触发 BGM; WIN/LOSE 触发音效 |
| `main/power_mgmt_task.c` | 省电时发送 MUTE/UNMUTE + BGM_PAUSE |

---

## 9. 实现步骤

| 步骤 | 内容 | 预计产出 |
|------|------|---------|
| **Step 1** | 创建 Audio Driver: I2S + ES8311 初始化, 能播放正弦波测试音 | `audio_driver.h/c` |
| **Step 2** | 实现 PSG 合成器: 4通道波形生成 + ADSR + 混音 + DMA回调 | `audio_synth.h/c` |
| **Step 3** | 创建 Audio Task + IPC: 命令队列, 任务主循环, 集成到 main.c | `audio_task.h/c`, 修改 ipc 和 main |
| **Step 4** | 实现音效系统: 6种SFX参数 + SFX触发逻辑 | `sfx_data.h`, 修改 pixel_physics |
| **Step 5** | 实现 BGM 播放器: 音符引擎 + BGM数据 + 循环控制 | `bgm_player.h/c`, `bgm_data.h` |
| **Step 6** | 集成游戏事件: 各状态转换触发 BGM/SFX, 省电MUTE | 修改 main_control_task, power_mgmt |
| **Step 7** | 调音与测试: 调整音量平衡, ADSR参数, 音效听感 | 所有参数微调 |

---

## 10. 扩展预留

- SD 卡 BGM 乐谱热加载 (留 API 接口)
- 麦克风语音指令 (留 g_audio_cmd_q 扩展能力)
- 更多 SFX 槽位 (数组管理, 易于添加)
- 多首 BGM 切换 (bgm_player 支持多 track)
