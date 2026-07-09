# 智绘灵境 (BoxAR Gaming) — AR Interactive Sandbox on ESP32-P4

> Point a camera at your desk, and everyday objects become a pixel-art marble maze game. Tilt to play.

**智绘灵境** is an augmented reality interactive sandbox that transforms a real-world desktop scene into a retro pixel-art game — entirely on-device, no cloud, no Wi-Fi. A camera captures your desk, AI detects objects, edges become walls, and a physics-simulated marble rolls under IMU tilt control. Everything runs on a single ESP32-P4 microcontroller.

---

## How It Works

```
┌──────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  OV5647      │    │  ESP-DL (CNN)    │    │  Pixel Game      │
│  MIPI-CSI    │───▶│  COCO 80-Class   │───▶│  World Builder   │
│  Camera      │    │  Object Detector │    │  (Tilemap+Sprites│
└──────────────┘    └──────────────────┘    └────────┬─────────┘
                                                     │
┌──────────────┐    ┌──────────────────┐             │
│  Canny Edge  │    │  Marble Physics  │◀────────────┘
│  Detection   │───▶│  100Hz Sim       │
│  (Walls)     │    │  (Tilt via IMU)  │
└──────────────┘    └────────┬─────────┘
                             │
┌──────────────┐             │
│  NES-APU     │◀────────────┘
│  Audio Synth │
│  (SFX + BGM) │
└──────────────┘
```

1. **Capture** — Press "Game" to freeze the camera frame
2. **Detect** — Canny edge detection builds stone walls; COCO object detection maps real items to game elements
3. **Play** — Tilt the device to roll the marble through the course toward the goal
4. **Interact** — Each detected object has unique gameplay behavior (see below)

---

## Real-World Objects → Game Elements

| Real Object | COCO Class | Game Element | Behavior |
|:---|:---|:---|:---|
| Apple / Orange / Banana | 47 / 49 / 46 | **Fruit** (pickup) | +100 pts, 5s wall-pass buff |
| Mouse | 64 | **Portal** (teleport) | Pairs warp the marble, +25 pts |
| Scissors | 76 | **Death Trap** | Instant death, lose a life |
| Bottle | 39 | **Goal** | Win! +500 pts + time bonus |
| Book | 73 | **Destructible Wall** | 3 hits to destroy, +50 pts |
| Cup | 41 | **Capture & Launch** | Freeze → aim by tilting → launch (600 px/s) |
| Spoon | 44 | **Sticky Wall** | Low-bounce surface (0.15) |
| Keyboard | 66 | **Medium Bounce** | Bounce coefficient 0.40 |
| Cell Phone | 67 | **High Bounce** | Trampoline-like (0.85) |

---

## Hardware

| Component | Part |
|:---|:---|
| **MCU** | ESP32-P4 (dual RISC-V HP @ 400MHz + LP @ 40MHz) |
| **Dev Board** | Waveshare ESP32-P4-Module-DEV-KIT |
| **Camera** | OV5647 (5MP) via MIPI-CSI 2-lane |
| **Display** | MIPI-DSI LCD with GT911 capacitive touch |
| **Audio** | ES8311 codec + NS4150B 3W amplifier via I2S |
| **IMU** | I2C accelerometer/gyroscope |
| **RAM** | 16MB Octal PSRAM |

---

## Software Architecture

```
┌─────────────────────────────────────────────────────┐
│                   FreeRTOS SMP                       │
├───────────────────────┬─────────────────────────────┤
│       Core 0 (I/O)    │      Core 1 (Compute)       │
├───────────────────────┼─────────────────────────────┤
│ Camera Task    (prio 5)│ Main Control Task (prio 2)  │
│ Display Task   (prio 4)│ Marble Physics   (prio 3)  │
│ IMU Task       (prio 4)│                             │
│ Audio Task     (prio 3)│                             │
│ Power Mgmt     (prio 1)│                             │
└───────────────────────┴─────────────────────────────┘
```

- **6 FreeRTOS tasks** distributed across dual HP cores, plus a dynamic physics task
- **IPC via queues, semaphores, and event groups** — `portMUX_TYPE` spinlocks protect shared state (no hardware cache coherence on ESP32-P4)
- **Power management** with 3-level idle state machine (Active → Dim → Deep Sleep)

---

## Key Features

### Camera & Image Processing
- V4L2-based MIPI-CSI camera driver (OV5647)
- 800×640 capture resolution
- 6-switchable image preprocessing presets: Denoise, Gamma, Histogram Equalization, Sharpen, Contrast Stretch
- Hardware ISP with auto-exposure control

### Edge Detection (Walls)
- Full Canny pipeline: RGB565-to-gray → 5×5 Gaussian blur → 3×3 Sobel gradient → non-maximum suppression → hysteresis thresholding
- All fixed-point integer math, optimized for RISC-V

### Object Detection (AI)
- ESP-DL v3.3.6 with COCO 80-class model
- Single-shot inference on 640×640 input
- Bounding box + confidence score output

### Marble Physics Engine
- Semi-implicit Euler integration at **100 Hz** fixed timestep
- 24-point circumferential collision sampling against the track map
- Sub-step anti-tunneling (auto subdivides when displacement > 6px)
- Wall normals computed from Sobel gradients for accurate bounce angles
- Speed cap, friction, wall-pass buff, and destructible wall logic

### Game World
- Pixel-art tilemap (40×40 tiles at 16px each)
- Procedurally generated sprites (3D-shaded marble with Phong lighting and rotation)
- 64-particle system with 5 effect types (spark, fruit, portal, death, win)
- 3 difficulty levels (Easy/Normal/Hard) with different time/lives/friction

### Audio (NES-APU Style)
- 4-channel PSG synthesizer: 2× Pulse, Triangle, LFSR Noise
- Independent ADSR envelope per channel
- 6 sound effects + 4-track 32-step BGM sequencer at 120 BPM

### UI (LVGL)
- Real-time camera preview with touch controls
- Mode switching: Live View / Edge View / Detect / Game
- Detection results overlay with bounding boxes
- Game score, lives, timer HUD

---

## Getting Started

### Prerequisites

- **ESP-IDF v5.5.4** — [Installation Guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32p4/get-started/index.html)
- ESP32-P4 development board (Waveshare ESP32-P4-Module-DEV-KIT recommended)
- OV5647 MIPI-CSI camera module
- MIPI-DSI LCD display

### Build & Flash

```bash
# Clone the repository
git clone https://github.com/leviwicr/BoxARGaming.git
cd BoxARGaming

# Set ESP32-P4 as target
idf.py set-target esp32p4

# Configure project (camera pins, LCD type, etc.)
idf.py menuconfig

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

### Hardware Setup

1. Connect OV5647 camera to the MIPI-CSI connector
2. Connect MIPI-DSI LCD display
3. Connect I2C IMU sensor (SCL: GPIO4, SDA: GPIO5, configurable)
4. Ensure the ES8311 audio codec is connected to I2S pins
5. Power via USB-C (5V/2A recommended)

---

## Project Structure

```
├── main/
│   ├── main.c                    # Entry point, system init
│   ├── config.h                  # Global configuration
│   ├── CMakeLists.txt            # Build rules
│   ├── ipc/                      # Inter-process communication
│   │   └── ipc.c/h               # Queues, semaphores, event groups
│   ├── tasks/                    # FreeRTOS tasks
│   │   ├── camera_task.c         # Camera capture loop
│   │   ├── display_task.c        # LVGL UI + preview
│   │   ├── imu_task.c            # IMU sensor polling
│   │   ├── audio_task.c          # Audio synthesis + playback
│   │   ├── main_control_task.c   # Game state machine + orchestration
│   │   └── power_mgmt_task.c     # Idle detection + power saving
│   ├── camera/                   # OV5647 MIPI-CSI driver
│   ├── display/                  # LVGL + display driver (C++)
│   ├── detection/                # ESP-DL COCO object detector (C++)
│   ├── edge_detection/           # Canny edge detection pipeline
│   ├── image_processing/         # ISP + preprocessing (C++)
│   ├── imu/                      # I2C IMU + complementary filter
│   ├── physics/                  # Marble physics engine
│   ├── track/                    # Collision map + wall management
│   ├── game/                     # Game renderer + particle system
│   ├── pixel_game/               # Pixel world builder + physics
│   └── audio/                    # ES8311 driver + NES-APU synth
└── plan/
    └── game_plan.md              # Game design document (Chinese)
```

---

## Dependencies

| Component | Version | Purpose |
|:---|:---|:---|
| ESP-IDF | v5.5.4 | Core framework |
| LVGL | v9.x | Graphics UI |
| ESP-DL | v3.3.6 | Neural network inference |
| COCO Detect | v0.4.0 | Object detection model |
| ESP Video | v2.x | V4L2 camera framework |
| ESP Cam Sensor | * | Camera sensor HAL |
| ESP Codec Dev | * | Audio codec abstraction |
| ESP New JPEG | v1.x | JPEG decoding |

See `main/idf_component.yml` for the full managed component list.

---

## Version History

| Version | Description |
|:---|:---|
| **v0.0.6** | Audio module: NES-APU style chip-music SFX + BGM |
| **v0.0.5** | FreeRTOS multi-task architecture, dual-core distribution, power management |
| **v0.0.4** | Pixel-art game UI and interactive game logic |
| **v0.0.3** | IMU driver, marble physics engine, track collision |
| **v0.0.2** | Canny edge detection for wall generation |
| **v0.0.1** | Camera + display drivers, COCO object detection baseline |

---

## License

This project is for educational and personal use.

---

## Acknowledgements

Built with [ESP-IDF](https://github.com/espressif/esp-idf), [LVGL](https://lvgl.io/), and [ESP-DL](https://github.com/espressif/esp-dl) by Espressif.
