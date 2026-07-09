#!/usr/bin/env python3
"""
Generate a minimal CJK TTF subset for the Zhihui-Lingjing project.

Takes any Chinese-capable TrueType font and extracts only the characters
needed by the UI, producing a tiny embedded font file.

Requirements:
    pip install fonttools brotli

Usage:
    # From any CJK font file:
    python generate_font.py /path/to/NotoSansSC-Regular.ttf

    # This produces: cjk_subset.ttf (~30-60 KB for ~50 chars)
"""

import sys
import os
import subprocess

# ASCII printable characters (so CJK font can render English text too)
ASCII_CHARS = (
    " !\"#$%&'()*+,-./"
    "0123456789"
    ":;<=>?@"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "[\\]^_`"
    "abcdefghijklmnopqrstuvwxyz"
    "{|}~"
)

# All Chinese characters used in the UI (add new ones here as needed)
CHINESE_CHARS = (
    # App title
    "智绘灵境"
    # Mode buttons
    "实时边缘待机"
    # Action buttons
    "检测开始游戏"
    # Difficulty
    "难度简单普通困难"
    # Status
    "就绪摄像头初始化预热中捕获失败已找到物体无检测"
    # Game HUD
    "分数生命时间速度弹力穿墙剩余秒重生"
    # Win/Lose
    "胜利失败游戏结束目标达成生命耗尽"
    # Scene objects
    "苹果橙子香蕉老鼠剪刀书本瓶子杯子勺子键盘手机"
    # Misc
    "退出模式切换选择当前未知已拾取消失"
    "穿墙激活已到达按下按键操作"
)

# Combined: all characters to include in the font subset
ALL_CHARS = ASCII_CHARS + CHINESE_CHARS

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <source_font.ttf>")
        print("\nExample: python generate_font.py ~/fonts/NotoSansSC-Regular.ttf")
        sys.exit(1)

    src = sys.argv[1]
    if not os.path.exists(src):
        print(f"Error: font file not found: {src}")
        sys.exit(1)

    out = os.path.join(os.path.dirname(__file__) or ".", "cjk_subset.ttf")

    # Deduplicate and sort characters
    chars = "".join(sorted(set(ALL_CHARS)))
    print(f"Subsetting {len(chars)} unique characters:")
    print(chars)

    # Use pyftsubset from fonttools
    # --no-subset-tables+=* keeps all OpenType tables
    # --text=... specifies the exact characters to keep
    cmd = [
        sys.executable, "-m", "fontTools.subset",
        src,
        f"--text={chars}",
        "--no-subset-tables+=cmap,glyf,loca,head,hhea,hmtx,maxp,name,post,OS/2",
        "--output-file=" + out,
        "--flavor=woff2",  # smallest output
        "--no-hinting",
        "--desubroutinize",
    ]

    print(f"\nRunning: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print("pyftsubset failed. Trying without woff2 compression...")
        # Fallback: plain TTF
        cmd_ttf = [
            sys.executable, "-m", "fontTools.subset",
            src,
            f"--text={chars}",
            "--output-file=" + out,
            "--no-hinting",
            "--desubroutinize",
        ]
        result = subprocess.run(cmd_ttf, capture_output=True, text=True)

    if result.returncode != 0:
        print("ERROR:", result.stderr)
        sys.exit(1)

    size_kb = os.path.getsize(out) / 1024
    print(f"\nDone! Generated: {out} ({size_kb:.1f} KB)")

    # Also print the C hex dump for direct embedding
    hex_out = out.replace(".ttf", ".h")
    with open(out, "rb") as f:
        data = f.read()

    with open(hex_out, "w") as hf:
        hf.write("/* Auto-generated CJK font subset */\n")
        hf.write(f"/* {len(chars)} characters, {len(data)} bytes */\n\n")
        hf.write("#pragma once\n")
        hf.write("#include <stdint.h>\n\n")
        hf.write(f"static const uint8_t cjk_font_data[{len(data)}] = {{\n")
        for i in range(0, len(data), 16):
            chunk = data[i:i+16]
            hex_bytes = ", ".join(f"0x{b:02X}" for b in chunk)
            hf.write(f"    {hex_bytes},\n")
        hf.write("};\n")
        hf.write(f"static const size_t cjk_font_size = {len(data)};\n")

    print(f"Also generated C header: {hex_out} ({os.path.getsize(hex_out) / 1024:.0f} KB)")
    print("\nNext: rebuild the project. The font will be embedded automatically.")

if __name__ == "__main__":
    main()
