# 项目音频系统实现总结

> 版本: v1.1 | 日期: 2026-07-07 | 状态: 已实现, 噪音问题已修复

---

## 一、硬件与驱动层

### 1.1 硬件配置

| 组件 | 型号 | 接口 | 参数 |
|------|------|------|------|
| Codec | ES8311 | I2C 0x18 | 单声道 DAC+ADC |
| 通信 | I2S #1 | MCLK=13, SCLK=12, LCLK=10, DOUT=9 | 22050Hz / 16bit / Mono |
| 功放 | NS4150B | PA_EN=GPIO53 | 3W Class-D |

### 1.2 驱动封装 (`audio_driver.h/c`)

```
audio_driver_init()   → bsp_i2c_init() → bsp_audio_init() → bsp_audio_codec_speaker_init()
audio_driver_start()  → esp_codec_dev_open()    打开 22050Hz/16bit/Mono 播放
audio_driver_stop()   → esp_codec_dev_close()   关闭播放
audio_driver_write()  → esp_codec_dev_write()   阻塞写入 PCM 数据
audio_driver_set_volume() → esp_codec_dev_set_out_vol()
audio_driver_mute/unmute() → 硬件音量 0 / 恢复
```

**关键处理 — ES8311 侧音抑制**: 初始化时向 REG44 写 0x08 (禁用 DAC2ADC 回环)、REG17 写 0x00 (静音 ADC 音量), 防止麦克风信号串入喇叭。

---

## 二、合成器层

### 2.1 通道架构 (`audio_synth.h/c`)

采用 NES-APU 风格的 4 通道 PSG 合成器, 每样本独立生成后混音:

```
                     ┌─ CH1: Pulse 方波 (duty 12/25/50/75%) ─┐
                     ├─ CH2: Pulse 方波 (duty 12/25/50/75%) ─┤
MIDI Note → freq ───├─ CH3: Triangle 三角波 ──────────────────├── mix → clamp → vol → int16
                     └─ CH4: LFSR 噪声 (15-bit, 长/短周期) ──┘
                              │
                          ADSR 包络 (每通道独立)
```

| 通道 | 波形 | 默认 ADSR | 主要用途 |
|------|------|-----------|---------|
| CH1 Pulse1 | 方波 | A=20 D=50 S=10/15 R=100ms | 主旋律 / SFX 音调 |
| CH2 Pulse2 | 方波 | A=20 D=50 S=10/15 R=100ms | 和弦 / SFX 叠加 |
| CH3 Tri | 三角波 | A=20 D=50 S=10/15 R=100ms | BGM 低音线 |
| CH4 Noise | LFSR噪声 | A=2 D=30 S=0 R=20ms | 打击乐 / 碰撞音效 |

### 2.2 ADSR 包络状态机

```
IDLE ──note_on──► ATTACK ──(level≥1.0)──► DECAY ──(level≤sustain)──► SUSTAIN
  ▲                                                                      │
  └──────────── RELEASE ◄──── note_off ──────────────────────────────────┘
                       └──(level≤0.0)──► IDLE
```

速率公式: `attack_rate = 1.0 / (attack_ms × 0.001 × sample_rate)`, 在 `adsr_tick()` 中每样本更新。

### 2.3 噪声通道特殊设计

使用 15-bit LFSR, 反馈 `bit14 XOR bit13`:

- **长周期**: 分频比 = note_freq, 输出 bit0, 模拟 kick/低频轰鸣
- **短周期**: 分频比 = note_freq × 4, 输出 bit6, 模拟 snare/hihat 金属音

关键: LFSR 并非每样本推进, 而是通过频率分频器控制推进速率, 使输出保持在可听范围。

### 2.4 渲染流程 (`audio_synth_render`)

```
每样本循环:
  1. sfx_tick(elapsed_ms)      — 更新 SFX 参数 (音高滑变)
  2. bgm_tick()                — 更新 BGM 步进
  3. channel_render() × 4      — 4 通道独立生成 ±1.0 样本
  4. 混音 → clamp[-1,1] → ×(master_vol/100) → ×16384 → int16
```

输出: 每次调用生成 512 samples (≈23.2ms), 16-bit signed mono。

---

## 三、音效系统

### 3.1 音效参数表

```c
// 每 SFX 定义: 时长 + 每通道 (起始音高, 滑变速率, 占空比, 音量)
g_sfx_defs[SFX_COUNT] = {
    [SFX_WALL_BOUNCE] = { 60ms,  Pulse1(note=48,sweep=-8,vol=8) + Noise(note=1,vol=12) },
    [SFX_FRUIT_PICKUP]= { 200ms, Pulse1(C5,vol=12) + Pulse2(E5,vol=8) },
    [SFX_PORTAL]      = { 250ms, Pulse1(note=36,sweep=+12,vol=10) + Noise(note=1,vol=8) },
    [SFX_DEATH]       = { 400ms, Pulse1(note=72,sweep=-6,vol=12) + Noise(note=1,vol=6) },
    [SFX_WIN]         = { 600ms, Pulse1(C5,vol=14) + Pulse2(E5,vol=10) + Tri(C4,vol=12) },
    [SFX_LOSE]        = { 500ms, Pulse1(A4,sweep=-3,vol=12) + Pulse2(E4,sweep=-3,vol=8) },
};
```

`note_sweep` 为非零时, 每 50ms 音高变化 sweep 个半音, 实现频率滑变效果。

### 3.2 SFX 生命周期

```
触发 (trigger_sfx):
  1. 保存 4 通道当前音量 → saved_vol[]
  2. SFX 使用的通道: ADSR 重置为快速参数 (A=2 D=30 S=10 R=50ms), 触发 note_on
  3. g_sfx.active = true, 记录起始样本号

播放中 (sfx_tick):
  每样本计算 elapsed_ms, 按 sweep 速率更新 note → freq → phase_inc

结束:
  1. 恢复 saved_vol[] 到各通道
  2. 仅对 SFX 使用的通道: midi_note=0, phase_inc=0, note_off
  3. 未使用的通道保持 BGM 状态不变
  4. 噪声通道恢复长周期模式
```

---

## 四、BGM 系统

### 4.1 数据格式 (`bgm_data.h`)

4 轨独立数据, 每轨 32 step 循环 (2 小节 4/4, tempo=120BPM, 每 step=31.25ms):

```
pulse1_data[64]: NOTE_C5,1, SILENCE,1, NOTE_E5,1, ...   — 主旋律
pulse2_data[64]: NOTE_C4,1, SILENCE,1, SILENCE,1, ...    — 和弦伴奏
tri_data[64]:    NOTE_C3,1, SILENCE,1, SILENCE,1, ...    — 低音线
noise_data[64]:  0x01,1, 0x04,1, 0x00,1, ...             — 鼓点
                                                0x01=kick  0x02=snare  0x04=hihat
```

### 4.2 播放逻辑

```
bgm_tick() 每样本调用:
  sample_counter++
  if sample_counter >= samples_per_step:
      → 4 通道 note_off (结束上一步音符)
      → step++, step %= 32 (循环)
      → bgm_advance_step(): 读取当前 step 的 4 轨数据, 非零音符触发 note_on

BGM 状态: STOPPED ⇄ PLAYING ⇄ PAUSED
```

### 4.3 音量分配

| 轨道 | 通道 | duty | 音量 |
|------|------|------|------|
| 主旋律 | CH1 Pulse1 | 50% | 10/15 |
| 和弦 | CH2 Pulse2 | 25% | 8/15 |
| 低音 | CH3 Tri | — | 12/15 |
| 鼓 | CH4 Noise | — | 14/15 (kick), 12/15 (snare), 5/15 (hihat) |

---

## 五、任务间通信

### 5.1 音频命令队列

```
g_audio_cmd_q: QueueHandle_t, 深度 16, 元素 audio_cmd_t

发送方 (非阻塞 xQueueSend, timeout=0):
  Marble Physics Task  → SFX_WALL_BOUNCE  (100Hz 碰撞检测)
  Pixel Physics 回调    → SFX_FRUIT_PICKUP / DEATH / WIN / PORTAL
  Main Control Task    → BGM_START / BGM_STOP / SFX_LOSE
  Power Mgmt Task      → MUTE / UNMUTE

接收方:
  Audio Task 主循环 (非阻塞轮询, 每 ~23ms 处理一批命令)
```

### 5.2 命令类型

```c
typedef enum {
    AUDIO_CMD_PLAY_SFX,       // 参数: sfx (sfx_type_t)
    AUDIO_CMD_BGM_START,      // 开始/恢复 BGM 循环
    AUDIO_CMD_BGM_STOP,       // 停止 BGM, 所有通道静音
    AUDIO_CMD_BGM_PAUSE,      // 暂停 BGM (预留)
    AUDIO_CMD_SET_MASTER_VOL, // 参数: value (0-100)
    AUDIO_CMD_MUTE,           // 硬件静音
    AUDIO_CMD_UNMUTE,         // 硬件取消静音
} audio_cmd_type_t;
```

---

## 六、Audio Task 主循环

```
audio_task() 运行于 Core 0, 优先级 3:

初始化:
  audio_driver_init()     I2S + ES8311 初始化 + 侧音抑制
  audio_driver_start()    打开 codec 播放通道
  audio_synth_init()      PSG 合成器初始化, BGM=STOPPED

主循环 (while 1):
  ┌─ 非阻塞排空命令队列 (xQueueReceive timeout=0)
  │    AUDIO_CMD_PLAY_SFX    → audio_synth_trigger_sfx()
  │    AUDIO_CMD_BGM_START   → audio_synth_bgm_start()
  │    AUDIO_CMD_BGM_STOP    → audio_synth_bgm_stop()
  │    AUDIO_CMD_BGM_PAUSE   → audio_synth_bgm_pause()
  │    AUDIO_CMD_SET_MASTER_VOL → audio_synth_set_master_vol() + audio_driver_set_volume()
  │    AUDIO_CMD_MUTE/UNMUTE → audio_driver_mute()/unmute()
  │
  ├─ audio_synth_render(buf, 512)   渲染 512 samples (~23.2ms)
  └─ audio_driver_write(buf, 1024)  阻塞写入 I2S → ES8311 (自然流控)
```

---

## 七、游戏事件到音效的映射

| 游戏事件 | 触发位置 | 音效 |
|---------|---------|------|
| 弹珠碰撞赛道墙壁 | `marble_physics.c` track collision | SFX_WALL_BOUNCE |
| 弹珠到达地图边界 | `marble_physics.c` boundary check | SFX_WALL_BOUNCE |
| 捡到水果 (激活穿墙) | `pixel_physics.c` FRUIT case | SFX_FRUIT_PICKUP |
| 触碰剪刀 (死亡) | `pixel_physics.c` DEATH case | SFX_DEATH |
| 到达瓶子 (胜利) | `pixel_physics.c` GOAL case | SFX_WIN |
| 进入传送门 | `pixel_physics.c` PORTAL case | SFX_PORTAL |
| 进入游戏 PLAYING 状态 | `main_control_task.c` CAPTURING→PLAYING | BGM_START |
| 游戏胜利 | `main_control_task.c` goal_reached | BGM_STOP |
| 游戏失败 | `main_control_task.c` player_dead | BGM_STOP + SFX_LOSE |
| 系统空闲 >60s | `power_mgmt_task.c` deep sleep | MUTE |
| 用户活动恢复 | `power_mgmt_task.c` resume | UNMUTE |

---

## 八、Bug 修复: 碰撞导致持续噪音

### 8.1 问题描述

- **现象 A**: 像素游戏中小球到达地图边缘 → 喇叭持续发出噪音
- **现象 B**: 启动界面 (无小球显示) → 同样持续噪音

### 8.2 根因分析

```
物理任务 100Hz ──每 10ms──► SFX_WALL_BOUNCE ──► audio_synth_trigger_sfx()
                                                        │
                                          重置 g_sfx.start_sample (每次!)
                                                        │
                                          60ms 音效永远无法结束 ◄──┘
                                                        │
                                          CH4 噪声通道 note=1 (≈8.6Hz)
                                          ADSR 卡在 Attack/Sustain
                                                        │
                                                  持续低频噪音
```

启动界面同理: IMU 上电初期可能输出未校准数据 → 小球被推向地图边界 → 同一循环。

### 8.3 修复方案

**marble_physics.c — 边缘触发 + 冷却**:

```c
// 地图边界: 仅在"未接触→接触"的上升沿触发, 配合 200ms 冷却
static bool g_was_on_boundary = false;
static int  g_boundary_sfx_cooldown = 0;

if (on_boundary && !g_was_on_boundary && g_boundary_sfx_cooldown <= 0) {
    xQueueSend(g_audio_cmd_q, &cmd, 0);
    g_boundary_sfx_cooldown = 20;  // 200ms
}
g_was_on_boundary = on_boundary;

// 赛道墙壁: 150ms 冷却
static int g_track_sfx_cooldown = 0;
if (vn < 0 && g_track_sfx_cooldown <= 0) {
    xQueueSend(g_audio_cmd_q, &cmd, 0);
    g_track_sfx_cooldown = 15;  // 150ms
}
```

**audio_synth.c — SFX 结束精确恢复**:

```c
// 修复前: 无条件 note_off 全部 4 通道 (切断 BGM)
// 修复后: 仅 note_off SFX 实际使用的通道
for (int i = 0; i < SYNTH_CH_COUNT; i++) {
    g_ch[i].volume = g_sfx.saved_vol[i];
    if (g_sfx_defs[type].ch[i].vol > 0) {
        // 仅清理 SFX 使用的通道
        g_ch[i].midi_note = 0;
        g_ch[i].phase_inc = 0.0f;
        g_ch[i].phase     = 0.0f;
        adsr_note_off(&g_ch[i].adsr);
    }
    // 未使用通道保持 BGM 状态不变
}
```

### 8.4 效果对比

| 场景 | 修复前 | 修复后 |
|------|--------|--------|
| 小球卡在地图边缘 | 100Hz 连续触发 → 持续噪音 | 仅首次接触触发 1 次 60ms 音效 |
| IMU 启动垃圾数据 | 同上 → 启动即噪音 | 同上 → 无持续噪音 |
| 赛道墙壁连续弹跳 | 每帧触发 (100Hz) | 最多 ~6.7 次/秒 (150ms CD) |
| SFX 结束后 BGM 被切断 | 4 通道全 note_off | 仅受影响通道静音 |

---

## 九、文件结构

```
main/
├── audio/
│   ├── audio_driver.h       # 硬件抽象层 API
│   ├── audio_driver.c       # ES8311 + I2S 驱动实现
│   ├── audio_synth.h        # PSG 合成器 API (通道/ADSR/SFX/BGM)
│   ├── audio_synth.c        # 合成器实现 (波形/包络/音效/BGM)
│   └── bgm_data.h           # BGM 乐谱数据 (4轨 × 32步)
├── tasks/
│   ├── audio_task.h         # Audio Task 声明
│   └── audio_task.c         # Audio Task 入口与主循环
├── ipc/
│   ├── ipc.h                # sfx_type_t, audio_cmd_t, g_audio_cmd_q
│   └── ipc.c                # g_audio_cmd_q 定义
├── physics/
│   └── marble_physics.c     # 碰撞音效触发 + 防抖
├── pixel_game/
│   └── pixel_physics.c      # 游戏物体事件音效触发
├── tasks/
│   ├── main_control_task.c  # 游戏状态 → BGM/SFX 控制
│   └── power_mgmt_task.c    # 省电 MUTE/UNMUTE
└── main.c                   # IPC 队列创建 + Audio Task 创建
```
