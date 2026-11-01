#!/usr/bin/env python3
"""Audio wiring of the sidboard: DAC1 (GPIO25) taken from the NodeMCU module
pin, isolated from the TXS0108E, through the RC filter to the Atari 130XE
ECI AUDIO IN.  Writes audio_wiring.svg.

From the board netlist: /D25 = J2 pin 8 (module socket) and U3 pin 6 (TXS0108E
A side); the B side, U3 pin 15, goes to J3 pin 13.  J2: pin 1 VIN (USB end)
... pin 15 EN (antenna end), the left header of the 30-pin DevKit V1.
"""
import os

W, H = 1180, 880
o = []


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text(x, y, s, size=13, anchor="start", fill="#222", weight="normal", rot=None):
    tr = f' transform="rotate({rot} {x} {y})"' if rot is not None else ""
    o.append(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" fill="{fill}" '
             f'font-weight="{weight}"{tr}>{esc(s)}</text>')


def wire(pts, color, width=5):
    d = " ".join(f"{x},{y}" for x, y in pts)
    o.append(f'<polyline points="{d}" fill="none" stroke="#00000055" stroke-width="{width + 2.5}" '
             f'stroke-linejoin="round" stroke-linecap="round"/>')
    o.append(f'<polyline points="{d}" fill="none" stroke="{color}" stroke-width="{width}" '
             f'stroke-linejoin="round" stroke-linecap="round"/>')
    for x, y in (pts[0], pts[-1]):
        o.append(f'<circle cx="{x}" cy="{y}" r="5" fill="{color}" stroke="#333" stroke-width="1.2"/>')


def cross(x, y, r=14, color="#d32f2f"):
    o.append(f'<line x1="{x-r}" y1="{y-r}" x2="{x+r}" y2="{y+r}" stroke="{color}" stroke-width="5" stroke-linecap="round"/>')
    o.append(f'<line x1="{x-r}" y1="{y+r}" x2="{x+r}" y2="{y-r}" stroke="{color}" stroke-width="5" stroke-linecap="round"/>')


o.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
         f'font-family="DejaVu Sans, Arial, sans-serif">')
o.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
text(W / 2, 36, "sidboard audio: DAC1 (GPIO25) from the module pin, isolated from the TXS0108E", 20, "middle", "#000", "bold")
text(W / 2, 60, "J2 pin 8 (D25) is also wired to U3 pin 6: the level translator loads the DAC, so the pin must not reach the socket", 12, "middle", "#555")

# ---- board -------------------------------------------------------------------
BX, BY, BW, BH = 300, 100, 430, 560
o.append(f'<rect x="{BX+5}" y="{BY+5}" width="{BW}" height="{BH}" rx="12" fill="#000" opacity="0.15"/>')
o.append(f'<rect x="{BX}" y="{BY}" width="{BW}" height="{BH}" rx="12" fill="#1d6b3c" stroke="#0f4626" stroke-width="2"/>')
text(BX + BW - 16, BY + 28, "ESP32-BOBOARD-5V", 15, "end", "#fff", "bold")

# J2 socket: pin 15 (EN) at the top, pin 1 (VIN) at the bottom
labels = {15: "EN", 14: "VP", 13: "VN", 12: "D34", 11: "D35", 10: "D32", 9: "D33", 8: "D25",
          7: "D26", 6: "D27", 5: "D14", 4: "D12", 3: "D13", 2: "GND", 1: "VIN"}
SX, SY, P = BX + 70, BY + 70, 30
o.append(f'<rect x="{SX-16}" y="{SY-18}" width="32" height="{15*P+6}" rx="3" fill="#222"/>')
text(SX, SY - 28, "J2", 13, "middle", "#fff", "bold")
hole = {}
for i, pin in enumerate(range(15, 0, -1)):
    y = SY + i * P
    hole[pin] = (SX, y)
    o.append(f'<rect x="{SX-6}" y="{y-6}" width="12" height="12" fill="#111" stroke="#c9a24a" stroke-width="2"/>')
    text(SX + 24, y + 5, f"{pin} {labels[pin]}", 12, "start", "#fff" if pin != 8 else "#ffd54f",
         "bold" if pin in (8, 2) else "normal")

# U3 TXS0108E
UX, UY = BX + 260, BY + 210
o.append(f'<rect x="{UX}" y="{UY}" width="90" height="200" rx="4" fill="#2b2b2b"/>')
text(UX + 45, UY + 95, "U3", 14, "middle", "#fff", "bold")
text(UX + 45, UY + 115, "TXS0108E", 11, "middle", "#ddd")
for k in range(10):
    y = UY + 14 + k * 19
    o.append(f'<rect x="{UX-8}" y="{y-3}" width="8" height="6" fill="#bbb"/>')
    o.append(f'<rect x="{UX+90}" y="{y-3}" width="8" height="6" fill="#bbb"/>')
u3_a6 = (UX - 8, UY + 14 + 5 * 19)          # pin 6 (A side)
u3_b15 = (UX + 98, UY + 14 + 5 * 19)        # pin 15 (B side), pins 11-20 up the right
text(u3_a6[0] - 4, u3_a6[1] - 8, "6 (A)", 10, "end", "#fff")
text(u3_b15[0] + 4, u3_b15[1] - 8, "15 (B)", 10, "start", "#fff")
x8, y8 = hole[8]
o.append(f'<polyline points="{x8},{y8+6} {x8},{y8+14} {x8+90},{y8+14} {x8+90},{u3_a6[1]} {u3_a6[0]},{u3_a6[1]}" '
         f'fill="none" stroke="#e0a85a" stroke-width="4"/>')
cross(x8 + 150, u3_a6[1])
text(x8 + 150, u3_a6[1] + 34, "trace D25: J2-8 to U3-6", 11, "middle", "#fff")
o.append(f'<polyline points="{u3_b15[0]},{u3_b15[1]} {BX+BW-40},{u3_b15[1]} {BX+BW-40},{BY+BH-40}" '
         f'fill="none" stroke="#e0a85a" stroke-width="4"/>')
o.append(f'<rect x="{BX+BW-60}" y="{BY+BH-40}" width="40" height="22" rx="3" fill="#222"/>')
text(BX + BW - 40, BY + BH - 24, "J3", 11, "middle", "#fff", "bold")
text(BX + BW - 40, BY + BH - 52, "J3 pin 13: leave free", 11, "end", "#ffd54f")

# ---- module pin bent out of the socket ------------------------------------
o.append(f'<rect x="{SX-40}" y="{SY-40}" width="80" height="{15*P+50}" rx="6" fill="#3a3a3a" opacity="0.35" '
         f'stroke="#111" stroke-dasharray="6 4"/>')
text(BX - 220, SY + 20, "dashed: the NodeMCU-32S module", 12, "start", "#333")
text(BX - 220, SY + 36, "(DevKit V1, 30 pins) in J2 / J1", 12, "start", "#333")
leg_end = (BX - 70, y8 - 44)
o.append(f'<polyline points="{x8},{y8} {x8-26},{y8} {leg_end[0]+40},{leg_end[1]} {leg_end[0]},{leg_end[1]}" '
         f'fill="none" stroke="#c0c0c0" stroke-width="6" stroke-linecap="round"/>')
cross(x8, y8, 9, "#ff5252")
text(BX - 220, y8 + 20, "1. D25 pin of the module bent out", 13, "start", "#c62828", "bold")
text(BX - 220, y8 + 38, "   (or cut): it must not enter J2-8", 12, "start", "#c62828")
text(BX - 220, y8 + 62, "   alternative: cut the D25 trace", 12, "start", "#555")
text(BX - 220, y8 + 78, "   between J2-8 and U3-6 (red X)", 12, "start", "#555")

# ---- RC filter -------------------------------------------------------------
TOP = 690
gx, gy = hole[2]
# audio wire: bent leg -> around the board -> C1
C1X, R1X, NX, OUTX = 790, 900, 1000, 1090
wire([leg_end, (leg_end[0] - 30, leg_end[1]), (leg_end[0] - 30, 84), (C1X - 40, 84), (C1X - 40, 220), (C1X - 12, 220)], "#ffd54f")
text(leg_end[0] - 40, 100, "2. audio wire", 13, "end", "#b8860b", "bold")
# C1 10 uF
o.append(f'<line x1="{C1X-12}" y1="196" x2="{C1X-12}" y2="244" stroke="#111" stroke-width="4"/>')
o.append(f'<path d="M{C1X+6},196 Q{C1X-2},220 {C1X+6},244" fill="none" stroke="#111" stroke-width="4"/>')
text(C1X - 24, 200, "+", 14, "middle", "#111", "bold")
text(C1X - 3, 270, "C1 10 µF", 12, "middle", "#111", "bold")
text(C1X - 3, 286, "(+ to the DAC)", 10, "middle", "#555")
# R1 100k zigzag
zig = [(C1X + 6, 220), (R1X - 30, 220)]
xs = R1X - 30
pts = [(xs, 220)]
for k in range(6):
    pts.append((xs + 5 + k * 10, 210 if k % 2 == 0 else 230))
pts.append((xs + 65, 220))
o.append(f'<line x1="{C1X+4}" y1="220" x2="{xs}" y2="220" stroke="#111" stroke-width="3"/>')
o.append('<polyline points="' + " ".join(f"{x},{y}" for x, y in pts) + '" fill="none" stroke="#111" stroke-width="3"/>')
text(R1X + 2, 200, "R1 100 kΩ", 12, "middle", "#111", "bold")
o.append(f'<line x1="{xs+65}" y1="220" x2="{OUTX}" y2="220" stroke="#111" stroke-width="3"/>')
o.append(f'<circle cx="{NX}" cy="220" r="5" fill="#111"/>')
# C2 10 nF to GND
o.append(f'<line x1="{NX}" y1="220" x2="{NX}" y2="300" stroke="#111" stroke-width="3"/>')
o.append(f'<line x1="{NX-22}" y1="300" x2="{NX+22}" y2="300" stroke="#111" stroke-width="4"/>')
o.append(f'<line x1="{NX-22}" y1="312" x2="{NX+22}" y2="312" stroke="#111" stroke-width="4"/>')
o.append(f'<line x1="{NX}" y1="312" x2="{NX}" y2="380" stroke="#111" stroke-width="3"/>')
text(NX + 30, 310, "C2 10 nF", 12, "start", "#111", "bold")
text(NX + 30, 326, "low-pass, fc ≈ 160 Hz", 10, "start", "#555")
text(NX + 30, 340, "with R1 (see note)", 10, "start", "#555")
o.append(f'<line x1="{NX-26}" y1="380" x2="{NX+26}" y2="380" stroke="#111" stroke-width="3"/>')
o.append(f'<line x1="{NX-16}" y1="388" x2="{NX+16}" y2="388" stroke="#111" stroke-width="3"/>')
o.append(f'<line x1="{NX-7}" y1="396" x2="{NX+7}" y2="396" stroke="#111" stroke-width="3"/>')
# Atari ECI
EX, EY = 960, 470
o.append(f'<rect x="{EX}" y="{EY}" width="190" height="200" rx="8" fill="#fbf7ef" stroke="#888"/>')
text(EX + 95, EY + 24, "Atari 130XE ECI", 14, "middle", "#000", "bold")
for k, (lab, on) in enumerate((("1 ~EXSEL", 0), ("2 ~RST", 0), ("3 ~D1XX", 0), ("4 ~MPD", 0),
                               ("5 AUDIO IN", 1), ("6 ~REF", 0), ("7 +5V", 0))):
    y = EY + 50 + k * 20
    o.append(f'<circle cx="{EX+22}" cy="{y}" r="6" fill="{"#ffd54f" if on else "#ddd"}" stroke="#666"/>')
    text(EX + 34, y + 4, lab, 11, "start", "#000" if on else "#999", "bold" if on else "normal")
o.append(f'<circle cx="{EX+120}" cy="{EY+170}" r="6" fill="#404040" stroke="#666"/>')
text(EX + 132, EY + 174, "H GND", 11, "start", "#000", "bold")
wire([(OUTX, 220), (OUTX + 40, 220), (OUTX + 40, 440), (EX - 30, 440), (EX - 30, EY + 130), (EX + 16, EY + 130)], "#ffd54f")
text(OUTX + 30, 210, "3. to AUDIO IN", 13, "end", "#b8860b", "bold")
# ground: J2-2 GND -> C2 ground -> ECI H (already common through J3 30 -> ECI H)
wire([(gx, gy), (gx - 30, gy), (gx - 30, TOP + 30), (NX, TOP + 30), (NX, 396)], "#404040")
wire([(NX, TOP + 30), (EX + 120, TOP + 30), (EX + 120, EY + 176)], "#404040")
text(gx - 40, TOP + 56, "4. GND from J2-2 (module GND) to C2 and ECI H", 13, "start", "#333", "bold")

notes = [
    "Why: U3 pin 6 (A side of the TXS0108E) shares the net with D25.  The TXS0108E has internal pull-ups (4 kΩ / 40 kΩ, to 3.3 V on A and 5 V on B)",
    "and one-shot edge accelerators while OE is high: on an analog DAC signal they shift the level and distort it.  Taking the signal from the module",
    "pin is not enough while the pin is still in the socket: it is the same copper.  GPIO26 (DAC2) has the same problem (J2-7, U3-7).",
    "Values from the sidboard README; with 100 kΩ and 10 nF the low-pass sits at about 160 Hz: use 1 nF (≈ 1.6 kHz) for brighter sound.",
]
for k, n in enumerate(notes):
    text(40, H - 84 + k * 18, n, 12, "start", "#333")
o.append("</svg>")
here = os.path.dirname(os.path.abspath(__file__))
open(os.path.join(here, "audio_wiring.svg"), "w").write("\n".join(o))
print("audio_wiring.svg")
