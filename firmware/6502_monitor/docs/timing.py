#!/usr/bin/env python3
"""Timing diagram of one Atari bus cycle as seen by the nodemcu-32s firmware:
waveforms of PHI2, address, selects and data, with the ESP32 sample points.
Writes timing_atari.svg.  Times in ns from the PHI2 falling edge (NTSC,
559 ns cycle); the firmware values are BUS_ADDR_SETTLE_NS = 200 and
BUS_WR_SAMPLE_NS = 120 (after the PHI2 rise).
"""
import os

T_CYCLE, T_RISE = 559, 280          # PHI2 low (PHI1) 0-280, high 280-559
T_ADS, T_SEL = 140, 170             # address / select valid
T_SAMPLE = 200                      # ESP32 PHI1 sample
T_DECODE = 320                      # byte in GPIO.out
T_DRIVE = 360                       # D0-D7 on the bus (read)
T_DSU = T_CYCLE - 100               # 6502 read setup
T_MDS = T_RISE + 100                # 6502 write data valid
T_WR = T_RISE + 120                 # ESP32 write sample
T_END = 640

X0, PX = 230, 1.45                  # ns -> px
ROW_H, Y0 = 58, 110
W = int(X0 + T_END * PX + 40)

ROWS = ["PHI2", "A0-A7, R/W", "~D1XX / ~CCTL", "ESP32", "D0-D7 read", "D0-D7 write"]
H = Y0 + len(ROWS) * ROW_H + 120
o = []


def x(t):
    return X0 + t * PX


def row_y(i):
    return Y0 + i * ROW_H


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text(xx, yy, s, size=12, anchor="start", fill="#222", weight="normal"):
    o.append(f'<text x="{xx}" y="{yy}" font-size="{size}" text-anchor="{anchor}" '
             f'fill="{fill}" font-weight="{weight}">{esc(s)}</text>')


def wave(i, points, color="#1f4e9c"):
    """points: list of (t, level) with level 0 / 1; step waveform."""
    y_hi, y_lo = row_y(i) + 8, row_y(i) + 34
    d, prev = [], None
    for t, lv in points:
        y = y_hi if lv else y_lo
        if prev is None:
            d.append(f"M{x(t)},{y}")
        else:
            d.append(f"L{x(t)},{prev}")
            d.append(f"L{x(t)},{y}")
        prev = y
    d.append(f"L{x(T_END)},{prev}")
    o.append(f'<path d="{" ".join(d)}" fill="none" stroke="{color}" stroke-width="2.5"/>')


def bus(i, segments):
    """segments: (t0, t1, kind, label) with kind valid / invalid / z."""
    yt, yb = row_y(i) + 8, row_y(i) + 34
    ym = (yt + yb) / 2
    for t0, t1, kind, label in segments:
        a, b = x(t0), x(t1)
        if kind == "z":
            o.append(f'<line x1="{a}" y1="{ym}" x2="{b}" y2="{ym}" stroke="#888" stroke-width="2" stroke-dasharray="6 4"/>')
        else:
            fill = {"valid": "#d6ecd2", "drive": "#cfe0fb", "invalid": "url(#hatch)"}[kind]
            stroke = {"valid": "#2e7d32", "drive": "#1f4e9c", "invalid": "#999"}[kind]
            o.append(f'<polygon points="{a},{ym} {a+6},{yt} {b-6},{yt} {b},{ym} {b-6},{yb} {a+6},{yb}" '
                     f'fill="{fill}" stroke="{stroke}" stroke-width="1.6"/>')
        if label:
            text((a + b) / 2, ym + 4, label, 11, "middle", "#222")


def marker(t, i0, i1, label, color, dy=0):
    o.append(f'<line x1="{x(t)}" y1="{row_y(i0)}" x2="{x(t)}" y2="{row_y(i1)+42}" '
             f'stroke="{color}" stroke-width="1.6" stroke-dasharray="4 3"/>')
    text(x(t) + 4, row_y(i1) + 54 + dy, label, 11, "start", color, "bold")


o.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
         f'font-family="DejaVu Sans, Arial, sans-serif">')
o.append('<defs><pattern id="hatch" width="6" height="6" patternUnits="userSpaceOnUse" '
         'patternTransform="rotate(45)"><rect width="6" height="6" fill="#f2f2f2"/>'
         '<line x1="0" y1="0" x2="0" y2="6" stroke="#bbb" stroke-width="2"/></pattern></defs>')
o.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
text(W / 2, 34, "Atari bus cycle, 6502_monitor firmware (env nodemcu-32s)", 20, "middle", "#000", "bold")
text(W / 2, 58, "ns from the PHI2 falling edge, NTSC 559 ns cycle; read = the device answers, write = the 6502 writes", 12, "middle", "#555")

# time grid
for t in range(0, T_END + 1, 50):
    o.append(f'<line x1="{x(t)}" y1="{Y0-14}" x2="{x(t)}" y2="{row_y(len(ROWS))}" stroke="#eee"/>')
    text(x(t), Y0 - 20, str(t), 10, "middle", "#888")
for i, name in enumerate(ROWS):
    text(X0 - 14, row_y(i) + 26, name, 13, "end", "#000", "bold")
    o.append(f'<line x1="{X0-6}" y1="{row_y(i)+48}" x2="{x(T_END)}" y2="{row_y(i)+48}" stroke="#f4f4f4"/>')

wave(0, [(0, 0), (T_RISE, 1), (T_CYCLE, 0)])
text(x(T_RISE / 2), row_y(0) + 50, "PHI1 (PHI2 low)", 11, "middle", "#555")
text(x((T_RISE + T_CYCLE) / 2), row_y(0) + 4, "PHI2 high", 11, "middle", "#555")
bus(1, [(0, T_ADS, "invalid", ""), (T_ADS, T_CYCLE + 30, "valid", "address, R/W valid (t_ADS ~140)"),
        (T_CYCLE + 30, T_END, "invalid", "")])
wave(2, [(0, 1), (T_SEL, 0), (T_CYCLE + 20, 1)], "#b03a8a")
text(x(T_SEL) + 6, row_y(2) + 50, "MMU / PLA decode", 11, "start", "#b03a8a")
bus(3, [(T_SAMPLE, T_SAMPLE + 15, "drive", ""), (T_SAMPLE + 15, T_DECODE, "valid", "decode, byte to GPIO.out"),
        (T_WR, T_WR + 10, "drive", "")])
bus(4, [(0, T_DRIVE, "z", ""), (T_DRIVE, T_CYCLE + 25, "drive", "ESP32 drives D0-D7"),
        (T_CYCLE + 25, T_END, "z", "")])
bus(5, [(0, T_MDS, "z", ""), (T_MDS, T_CYCLE + 10, "valid", "6502 data (t_MDS)"), (T_CYCLE + 10, T_END, "z", "")])

marker(T_SAMPLE, 1, 3, "sample 200 ns (BUS_ADDR_SETTLE_NS)", "#c62828")
marker(T_DSU, 4, 4, "6502 setup: data valid by 459", "#1f4e9c")
marker(T_WR, 5, 5, "write sample 400 ns (rise + BUS_WR_SAMPLE_NS)", "#c62828", 14)

text(40, H - 30, "Values to be measured on the real bus with a scope; tune with -D BUS_ADDR_SETTLE_NS / BUS_WR_SAMPLE_NS.", 12, "start", "#333")
o.append("</svg>")
here = os.path.dirname(os.path.abspath(__file__))
open(os.path.join(here, "timing_atari.svg"), "w").write("\n".join(o))
print("timing_atari.svg")
