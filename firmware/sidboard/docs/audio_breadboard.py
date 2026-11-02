#!/usr/bin/env python3
"""Audio wiring of the sidboard in the style of the Fritzing breadboard view:
DAC1 (GPIO25) from the NodeMCU module pin, bent out of J2 so that the
TXS0108E (U3 pin 6) does not load it, through the filter on a breadboard to
the computer.

- Atari 130XE: C1 10 uF, R1 100 kOhm, C2 10 nF to the ECI AUDIO IN (pin 5),
  GND to ECI H: the sound comes out mixed with POKEY.
- Commodore 64: line-level network C1 10 uF, R1 4.7 kOhm / R2 2.2 kOhm,
  C2 47 nF, then C3 1 uF to the A/V socket pin 5 (AUDIO IN = SID EXT IN),
  GND to pin 2: the sound comes out mixed with the internal SID.

Values from the sidboard README (Audio Circuit, Amplifier).  Writes
audio_breadboard_atari.svg and audio_breadboard_c64.svg.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 1360, 830

YELLOW, BLACK, GREY = "#ffd54f", "#404040", "#c0c0c0"


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def darker(c, f=0.6):
    r, g, b = (int(c[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % (int(r * f), int(g * f), int(b * f))


class Svg:
    def __init__(self):
        self.o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
                  f'viewBox="0 0 {W} {H}" font-family="DejaVu Sans, Arial, sans-serif">',
                  f'<rect width="{W}" height="{H}" fill="#ffffff"/>']

    def add(self, s):
        self.o.append(s)

    def text(self, x, y, s, size=12, anchor="start", fill="#222", weight="normal", rot=None, bg=False):
        tr = f' transform="rotate({rot} {x} {y})"' if rot is not None else ""
        if bg:
            w = len(s) * size * 0.62 + 6
            x0 = {"start": x - 3, "middle": x - w / 2, "end": x - w + 3}[anchor]
            self.add(f'<rect x="{x0}" y="{y-size}" width="{w}" height="{size+5}" rx="3" '
                     f'fill="#ffffff" opacity="0.9"/>')
        self.add(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" fill="{fill}" '
                 f'font-weight="{weight}"{tr}>{esc(s)}</text>')

    def wire(self, pts, c):
        """Fritzing wire: darker outline, colour, round ends."""
        d = " ".join(f"{x},{y}" for x, y in pts)
        self.add(f'<polyline points="{d}" fill="none" stroke="{darker(c)}" stroke-width="7" '
                 f'stroke-linejoin="round" stroke-linecap="round"/>')
        self.add(f'<polyline points="{d}" fill="none" stroke="{c}" stroke-width="4.5" '
                 f'stroke-linejoin="round" stroke-linecap="round"/>')
        for x, y in (pts[0], pts[-1]):
            self.add(f'<circle cx="{x}" cy="{y}" r="4.5" fill="{c}" stroke="{darker(c)}" stroke-width="1.5"/>')

    def leg(self, x1, y1, x2, y2):
        self.add(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="#9a9a9a" '
                 f'stroke-width="2.5" stroke-linecap="round"/>')

    def cross(self, x, y, r=9, c="#ff5252"):
        for dx in (-r, r):
            self.add(f'<line x1="{x-dx}" y1="{y-r}" x2="{x+dx}" y2="{y+r}" stroke="{c}" '
                     f'stroke-width="4" stroke-linecap="round"/>')

    def save(self, name):
        self.o.append("</svg>")
        open(os.path.join(HERE, name), "w").write("\n".join(self.o))


# ---- resistor colour code ----------------------------------------------------
BAND = {0: "#000000", 1: "#8b4513", 2: "#e02020", 3: "#ff8c00", 4: "#f2d000",
        5: "#2e9d3a", 6: "#2060d0", 7: "#8a2be2", 8: "#808080", 9: "#ffffff"}


def bands(ohm):
    s = str(int(ohm))
    mult = len(s) - 2
    return [BAND[int(s[0])], BAND[int(s[1])], BAND[mult], "#c9a24a"]   # gold 5 %


def resistor(g, p1, p2, ohm, label):
    (x1, y1), (x2, y2) = p1, p2
    g.leg(x1, y1, x2, y2)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    g.add(f'<rect x="{cx-24}" y="{cy-8}" width="48" height="16" rx="7" fill="#e8d3a2" stroke="#a58b52"/>')
    for k, c in enumerate(bands(ohm)):
        bx = cx - 15 + k * 8 + (6 if k == 3 else 0)
        g.add(f'<rect x="{bx}" y="{cy-8}" width="4" height="16" fill="{c}"/>')
    for x, y in (p1, p2):
        g.add(f'<circle cx="{x}" cy="{y}" r="3" fill="#7a7a7a"/>')
    g.text(cx, cy - 14, label, 11, "middle", "#111", "bold", bg=True)


def electrolytic(g, plus, minus, label):
    """Lying radial electrolytic: blue body, light stripe on the minus side."""
    (x1, y1), (x2, y2) = plus, minus
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    g.leg(x1, y1, cx - 18, cy)
    g.leg(cx + 18, cy, x2, y2)
    g.add(f'<rect x="{cx-22}" y="{cy-13}" width="44" height="26" rx="6" fill="#1f3f8f" stroke="#10245a"/>')
    g.add(f'<rect x="{cx+8}" y="{cy-13}" width="9" height="26" fill="#9fb4e0"/>')
    g.text(cx + 12.5, cy + 4, "−", 11, "middle", "#10245a", "bold")
    g.text(cx - 8, cy + 4, "+", 11, "middle", "#ffffff", "bold")
    for x, y in (plus, minus):
        g.add(f'<circle cx="{x}" cy="{y}" r="3" fill="#7a7a7a"/>')
    g.text(cx, cy - 18, label, 11, "middle", "#111", "bold", bg=True)


def ceramic(g, p1, p2, code, label, label_pos=None, anchor="start", body=None):
    (x1, y1), (x2, y2) = p1, p2
    cx, cy = body if body else ((x1 + x2) / 2, (y1 + y2) / 2)
    g.leg(x1, y1, cx, cy)
    g.leg(cx, cy, x2, y2)
    g.add(f'<ellipse cx="{cx}" cy="{cy}" rx="13" ry="11" fill="#e8a33d" stroke="#9c6416"/>')
    g.text(cx, cy + 3.5, code, 8, "middle", "#4a2c00", "bold")
    for x, y in (p1, p2):
        g.add(f'<circle cx="{x}" cy="{y}" r="3" fill="#7a7a7a"/>')
    lx, ly = label_pos if label_pos else (cx + 18, cy + 4)
    g.text(lx, ly, label, 11, anchor, "#111", "bold", bg=True)


# ---- the ESP32-BOBOARD-5V with the module ----------------------------------
BX, BY, BW, BH = 40, 110, 400, 470
MX, MY, MW, MH = 110, 135, 230, 380
J2X = MX + 16
J2LAB = {15: "EN", 14: "VP", 13: "VN", 12: "D34", 11: "D35", 10: "D32", 9: "D33", 8: "D25",
         7: "D26", 6: "D27", 5: "D14", 4: "D12", 3: "D13", 2: "GND", 1: "VIN"}


def j2y(pin):
    return MY + 40 + (15 - pin) * 22


def j3xy(p):
    x = 72 + ((p - 1) // 2) * 21
    return x, (557 if p % 2 else 541)


def board(g, computer):
    g.add(f'<rect x="{BX+5}" y="{BY+5}" width="{BW}" height="{BH}" rx="10" fill="#000" opacity="0.18"/>')
    g.add(f'<rect x="{BX}" y="{BY}" width="{BW}" height="{BH}" rx="10" fill="#1d6b3c" stroke="#0f4626" stroke-width="2"/>')
    g.text(BX + BW - 12, BY + 20, "ESP32-BOBOARD-5V", 13, "end", "#fff", "bold")
    # NodeMCU-32S module in J2 / J1
    g.add(f'<rect x="{MX}" y="{MY}" width="{MW}" height="{MH}" rx="5" fill="#202020"/>')
    g.add(f'<rect x="{MX+70}" y="{MY+14}" width="110" height="130" rx="3" fill="#b8bcc0" stroke="#8a8e92"/>')
    g.text(MX + 125, MY + 84, "ESP32-WROOM-32", 9, "middle", "#333")
    g.text(MX + 125, MY + 200, "NodeMCU-32S", 12, "middle", "#ddd")
    g.text(MX + 125, MY + 218, "(J2 left, J1 right)", 10, "middle", "#aaa")
    for pin in range(1, 16):
        y = j2y(pin)
        for x in (J2X, MX + MW - 16):
            if not (x == J2X and pin == 8):
                g.add(f'<rect x="{x-5}" y="{y-5}" width="10" height="10" fill="#d4b04c" stroke="#8a6d1f"/>')
        hot = pin in (8, 2)
        g.text(J2X + 12, y + 4, f"{pin} {J2LAB[pin]}", 10, "start", YELLOW if pin == 8 else "#fff",
               "bold" if hot else "normal")
    # D25 bent out of the socket
    y8 = j2y(8)
    g.add(f'<rect x="{J2X-5}" y="{y8-5}" width="10" height="10" fill="#111" stroke="#8a6d1f"/>')
    g.add(f'<polyline points="{J2X},{y8} {J2X-22},{y8} {BX+40},{y8-26} {BX+12},{y8-26}" fill="none" '
          f'stroke="{GREY}" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/>')
    g.cross(J2X, y8, 8)
    # J3 2x16 header (odd pins lower row, pin 1 on the left)
    g.add(f'<rect x="{BX+20}" y="{530}" width="{16*21+10}" height="38" rx="2" fill="#2b2b2b"/>')
    for p in range(1, 33):
        x, y = j3xy(p)
        g.add(f'<rect x="{x-3.5}" y="{y-3.5}" width="7" height="7" fill="#d4b04c" stroke="#8a6d1f"/>')
    g.text(BX + 20, 524, "J3", 11, "start", "#fff", "bold")
    g.text(BX + 44, 524, "pin 13 (U3-15, D25 B side): leave free", 10, "start", YELLOW)
    # bus cable to the computer
    g.add(f'<rect x="{BX+30}" y="{574}" width="{16*21-10}" height="100" fill="#9aa3ad" opacity="0.55"/>')
    for k in range(12):
        x = BX + 40 + k * 27
        g.add(f'<line x1="{x}" y1="574" x2="{x}" y2="674" stroke="#7c858f" stroke-width="2"/>')
    dest = ("J3 to the cartridge slot and the ECI: 6502_monitor/docs/breadboard_atari.svg"
            if computer == "atari" else
            "J3 to the C64 expansion port: 6502_monitor/docs/breadboard_c64.svg")
    g.text(BX + 30, 694, dest, 11, "start", "#555")
    return (BX + 12, y8 - 26), (J2X, j2y(2))


# ---- breadboard --------------------------------------------------------------
BBX, BBY, P, NC = 500, 150, 18, 24
ROWS = "abcdefghij"


def hx(col):
    return BBX + 30 + (col - 1) * P


def hy(row):
    k = ROWS.index(row)
    return BBY + 70 + k * P + (22 if k >= 5 else 0)


RAIL_PLUS, RAIL_MINUS = BBY + 18, BBY + 36          # top rails
CHANNEL = (hy("e") + hy("f")) / 2


def breadboard(g):
    w, h = 30 * 2 + (NC - 1) * P, 330
    g.add(f'<rect x="{BBX+4}" y="{BBY+4}" width="{w}" height="{h}" rx="6" fill="#000" opacity="0.12"/>')
    g.add(f'<rect x="{BBX}" y="{BBY}" width="{w}" height="{h}" rx="6" fill="#f4f4f2" stroke="#c8c8c4"/>')
    g.add(f'<rect x="{BBX+10}" y="{CHANNEL-4}" width="{w-20}" height="8" fill="#e2e2de"/>')
    # top rails: + (red line) and - (blue line), the - rail is the GND
    g.add(f'<line x1="{BBX+20}" y1="{RAIL_PLUS-10}" x2="{BBX+w-20}" y2="{RAIL_PLUS-10}" stroke="#d33" stroke-width="2"/>')
    g.add(f'<line x1="{BBX+20}" y1="{RAIL_MINUS+10}" x2="{BBX+w-20}" y2="{RAIL_MINUS+10}" stroke="#36c" stroke-width="2"/>')
    g.text(BBX + 10, RAIL_PLUS + 4, "+", 12, "middle", "#d33", "bold")
    g.text(BBX + 10, RAIL_MINUS + 4, "−", 12, "middle", "#36c", "bold")
    for c in range(1, NC + 1):
        for y in (RAIL_PLUS, RAIL_MINUS):
            g.add(f'<rect x="{hx(c)-3}" y="{y-3}" width="6" height="6" fill="#4a4a4a"/>')
        for r in ROWS:
            g.add(f'<rect x="{hx(c)-3}" y="{hy(r)-3}" width="6" height="6" fill="#4a4a4a"/>')
        if c in (1, 5, 10, 15, 20):
            g.text(hx(c), hy("a") - 10, str(c), 8, "middle", "#888")
    for r in ROWS:
        g.text(BBX + 12, hy(r) + 3, r, 8, "middle", "#888")
    g.text(BBX + w / 2, BBY + h + 18, "breadboard: each column a-e (and f-j) is one node; − rail = GND",
           11, "middle", "#555")
    return BBX + w


def hole(col, row):
    return hx(col), hy(row)


def rail(col):
    return hx(col), RAIL_MINUS


# ---- drawings ----------------------------------------------------------------
def header(g, title, subtitle):
    g.text(W / 2, 36, title, 20, "middle", "#000", "bold")
    g.text(W / 2, 60, subtitle, 12, "middle", "#555")


def notes(g, lines):
    for k, n in enumerate(lines):
        g.add(f'<text x="40" y="{H - 96 + k * 19}" font-size="12" fill="#333">{n}</text>')


def common(g, computer):
    leg_end, gnd = board(g, computer)
    breadboard(g)
    # audio wire: bent D25 leg -> over the board -> column 3
    g.wire([leg_end, (leg_end[0], 88), (hx(3), 88), hole(3, "a")], YELLOW)
    g.text(leg_end[0] + 6, 82, "D25 pin of the module bent out of J2 (or cut)", 11, "start", "#b8860b", "bold")
    # GND: module J2 pin 2 -> - rail
    g.wire([gnd, (BX + 26, gnd[1]), (BX + 26, 102), (hx(1) - 14, 102), (hx(1) - 14, RAIL_MINUS), rail(1)], BLACK)
    g.text(BX + 20, gnd[1] - 6, "GND from J2 pin 2", 11, "start", "#fff", "bold", rot=-90)
    # C1 10 uF: + on column 3 (DAC), - on column 7
    electrolytic(g, hole(3, "c"), hole(7, "c"), "C1 10 µF")


def atari():
    g = Svg()
    header(g, "sidboard audio on the Atari 130XE, breadboard view",
           "DAC1 (GPIO25) through C1 / R1 / C2 to the ECI AUDIO IN: the SID comes out of the TV mixed with POKEY")
    common(g, "atari")
    resistor(g, hole(7, "b"), hole(12, "b"), 100e3, "R1 100 kΩ")
    ceramic(g, hole(12, "a"), rail(14), "103", "C2 10 nF", (hx(13) + 8, RAIL_PLUS + 4))
    # ECI connector (rear of the 130XE), contacts as 6502_monitor breadboard_atari
    cx0, CY, PC = 1030, 440, 36
    top = ["1 ~EXSEL", "2 ~RST", "3 ~D1XX", "4 ~MPD", "5 AUDIO IN", "6 ~REF", "7 +5V"]
    bot = ["A reserved", "B ~IRQ", "C ~HALT", "D A13", "E A14", "F A15", "H GND"]
    g.add(f'<rect x="{cx0}" y="{CY-14}" width="{7*PC+PC/2}" height="62" rx="5" fill="#3a3a3a" stroke="#1e1e1e" stroke-width="2"/>')
    g.add(f'<rect x="{cx0+6}" y="{CY+10}" width="{7*PC+PC/2-12}" height="6" fill="#141414"/>')
    pos = {}
    for r, row in enumerate((top, bot)):
        for i, lab in enumerate(row):
            x = cx0 + PC / 2 + i * PC + (24 if r else 12)
            y = CY + (26 if r else 0)
            on = lab.startswith(("5 ", "H "))
            g.add(f'<rect x="{x-5}" y="{y-6}" width="10" height="12" rx="1" '
                  f'fill="{"#e6c35c" if on else "#9c8a52"}" stroke="#7a5e14"/>')
            g.add(f'<text x="{x+4}" y="{CY+60}" font-size="10" fill="{"#000" if on else "#aaa"}" '
                  f'font-weight="{"bold" if on else "normal"}" transform="rotate(90 {x+4} {CY+60})">{esc(lab)}</text>')
            pos[lab[0]] = (x, y)
    g.text(cx0 + 7 * PC / 2 + 9, CY + 165, "Atari 130XE ECI", 14, "middle", "#000", "bold")
    g.text(cx0 + 7 * PC / 2 + 9, CY + 183, "upper row 1-7, lower row A-H (drawn offset)", 10, "middle", "#555")
    # OUT: column 12 -> along the channel -> ECI 5
    x5, y5 = pos["5"]
    g.wire([hole(12, "e"), (hx(12), CHANNEL), (990, CHANNEL), (990, 400), (x5, 400), (x5, y5)], YELLOW)
    g.text(hx(12) + 12, CHANNEL + 22, "OUT to ECI 5 AUDIO IN", 11, "start", "#b8860b", "bold", bg=True)
    xh, yh = pos["H"]
    g.wire([rail(22), (xh, RAIL_MINUS), (xh, yh)], BLACK)
    g.text(hx(22) + 4, RAIL_MINUS - 10, "GND to ECI H", 11, "start", "#333", "bold", bg=True)
    notes(g, [
        "1. Bend the D25 pin of the NodeMCU module out of J2 (pin 8) or cut it: U3 pin 6 (TXS0108E) shares that net and would load the DAC.",
        "2. C1 10 µF (+ towards the DAC), R1 100 kΩ, C2 10 nF to GND: low-pass about 160 Hz (1 nF: about 1.6 kHz).  GND from J2 pin 2.",
        "3. The ECI AUDIO IN has a high impedance: the 100 kΩ filter is right here.  Plug the board in, power on: the sound joins POKEY on the TV.",
        "Values and reasons: sidboard README, Audio Circuit.  Works with every build (sid-atari-cctl, sid-atari-pbi, sid-demo).",
    ])
    g.save("audio_breadboard_atari.svg")


def c64():
    g = Svg()
    header(g, "sidboard audio on the Commodore 64, breadboard view",
           "DAC1 (GPIO25) at line level through C3 1 µF to the A/V socket pin 5 (AUDIO IN = SID EXT IN): mixed with the internal SID")
    common(g, "c64")
    resistor(g, hole(7, "b"), hole(12, "b"), 4.7e3, "R1 4.7 kΩ")
    ceramic(g, hole(12, "a"), rail(14), "473", "C2 47 nF", (hx(13) + 8, RAIL_PLUS + 4))
    resistor(g, hole(12, "d"), hole(17, "d"), 2.2e3, "R2 2.2 kΩ")
    g.wire([hole(17, "a"), (hx(17), RAIL_MINUS + 8), rail(17)], BLACK)
    # C3 1 uF across the channel: column 12 (a-e) -> column 12 (f-j)
    ceramic(g, hole(12, "e"), hole(12, "f"), "105", "C3 1 µF", (hx(12) + 48, CHANNEL + 4),
            body=(hx(12) + 26, CHANNEL))
    # A/V socket, as a DIN plug breakout
    AX, AY = 1040, 360
    av = [("1", "LUM / SYNC"), ("2", "GND"), ("3", "AUDIO OUT"), ("4", "VIDEO"), ("5", "AUDIO IN"),
          ("6", "CHROMA (8-pin)")]
    g.add(f'<rect x="{AX}" y="{AY}" width="250" height="190" rx="8" fill="#fbf7ef" stroke="#888"/>')
    g.text(AX + 125, AY + 26, "C64 A/V socket (DIN, rear)", 14, "middle", "#000", "bold")
    g.text(AX + 125, AY + 44, "through a DIN plug or breakout", 10, "middle", "#555")
    pos = {}
    for k, (n, lab) in enumerate(av):
        y = AY + 66 + k * 22
        on = n in ("2", "5")
        g.add(f'<circle cx="{AX+22}" cy="{y}" r="6" fill="{"#e6c35c" if on else "#ddd"}" stroke="#666"/>')
        g.text(AX + 36, y + 4, f"{n}  {lab}", 11, "start", "#000" if on else "#999", "bold" if on else "normal")
        pos[n] = (AX + 22, y)
    # OUT after C3: column 12 row h -> A/V 5
    x5, y5 = pos["5"]
    g.wire([hole(12, "h"), (1000, hy("h")), (1000, y5), (x5, y5)], YELLOW)
    g.text(hx(12) + 12, hy("h") + 22, "OUT to A/V 5 AUDIO IN", 11, "start", "#b8860b", "bold", bg=True)
    x2, y2 = pos["2"]
    g.wire([rail(22), (1015, RAIL_MINUS), (1015, y2), (x2, y2)], BLACK)
    g.text(hx(22) + 4, RAIL_MINUS - 10, "GND to A/V 2", 11, "start", "#333", "bold", bg=True)
    g.text(AX + 125, AY + 216, "check the pin numbers printed on the plug:", 10, "middle", "#555")
    g.text(AX + 125, AY + 230, "pin 2 is the shield / GND", 10, "middle", "#555")
    notes(g, [
        "1. Bend the D25 pin of the NodeMCU module out of J2 (pin 8) or cut it: U3 pin 6 (TXS0108E) shares that net and would load the DAC.",
        "2. Line level: C1 10 µF (+ towards the DAC), R1 4.7 kΩ / R2 2.2 kΩ (3.3 Vpp to about 1 Vpp), C2 47 nF (low-pass about 2.3 kHz).",
        "3. C3 1 µF (non-polarised) keeps the DC bias of the SID EXT IN off the network.  Set the internal SID volume above 0 (POKE 54296,15):",
        "    EXT IN goes through it.  The sound comes out of the TV / monitor.  Works with sid-c64-io1, sid-c64-io2 and sid-demo.",
    ])
    g.save("audio_breadboard_c64.svg")


if __name__ == "__main__":
    atari()
    c64()
    print("audio_breadboard_atari.svg, audio_breadboard_c64.svg")
