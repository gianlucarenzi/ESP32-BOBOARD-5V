#!/usr/bin/env python3
"""Wiring infographics for the 6502_monitor firmware: J3 of the
ESP32-BOBOARD-5V to the Atari 130XE (cartridge slot + ECI) and to the
Commodore 64 expansion port.  Writes wiring_atari.svg and wiring_c64.svg.

J3 pins come from the board netlist (5 V side of the TXS0108E).  The
computer connector pinouts were checked online: Atari cartridge slot and
130XE ECI against atarimania.com (Atari 8-bit FAQ) and computersmuseum.com,
C64 expansion port against c64-wiki.com and retroisle.com.
"""
import os

# J3 (5 V side): pin -> (signal, note)
J3 = {
    1: "PHI2", 2: "D0", 3: "D1", 4: "n.c. (GPIO12)", 5: "D2", 6: "D3",
    7: "R/W", 8: "D6", 9: "D7", 10: "A6", 11: "~D1XX", 12: "~CCTL",
    13: "n.c. (GPIO25)", 14: "n.c. (GPIO26)", 15: "A7", 16: "A4", 17: "A5",
    18: "A0", 19: "A1", 20: "A2", 21: "A3", 22: "D4", 23: "D5",
    24: "n.c.", 25: "n.c.", 26: "EN - do not connect", 27: "+3V3 - do not connect",
    28: "+3V3 - do not connect", 29: "GND", 30: "GND", 31: "+5V", 32: "+5V",
}

COL = {"D": "#2f6fdf", "A": "#1f9d55", "CTL": "#e07b00", "SEL": "#d6336c",
       "GND": "#333333", "5V": "#c92a2a"}


def group(sig):
    if sig.startswith("D") and sig[1:].isdigit():
        return "D"
    if sig.startswith("A") and sig[1:].isdigit():
        return "A"
    if sig in ("PHI2", "R/W"):
        return "CTL"
    if sig == "GND":
        return "GND"
    if sig == "+5V":
        return "5V"
    return "SEL"


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def draw(title, subtitle, connectors, links, notes, fname):
    """connectors: list of (name, short, x, y, rows), rows = list of pin rows,
    each a list of (pin label, signal).  links: (J3 pin, signal, connector
    name, pin label).  Left: J3 with an arrow per used pin to its
    destination; right: the connectors, each used pin marked with its J3
    pin number."""
    W, P = 1180, 24
    j3x, j3y = 40, 130
    dest = {l[0]: l for l in links}
    short = {c[0]: c[1] for c in connectors}
    bottom = max(c[3] + len(c[4][0]) * P for c in connectors)
    H = max(j3y + 32 * P, bottom) + 70 + 20 * len(notes)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'viewBox="0 0 {W} {H}" font-family="DejaVu Sans, Arial, sans-serif">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{W/2}" y="40" font-size="24" font-weight="bold" text-anchor="middle">{esc(title)}</text>',
           f'<text x="{W/2}" y="68" font-size="14" fill="#555" text-anchor="middle">{esc(subtitle)}</text>']

    # J3 and the arrows
    out.append(f'<rect x="{j3x}" y="{j3y-34}" width="230" height="{32*P+44}" rx="8" fill="#f4f6f8" stroke="#888"/>')
    out.append(f'<text x="{j3x+115}" y="{j3y-12}" font-size="15" font-weight="bold" text-anchor="middle">ESP32-BOBOARD-5V  J3</text>')
    for p in range(1, 33):
        y = j3y + (p - 1) * P + 12
        sig = J3[p]
        l = dest.get(p)
        fill = COL[group(l[1])] if l else "#bbbbbb"
        out.append(f'<circle cx="{j3x+210}" cy="{y}" r="7" fill="{fill}" stroke="#333"/>')
        out.append(f'<text x="{j3x+194}" y="{y+5}" font-size="13" text-anchor="end" '
                   f'fill="{"#000" if l else "#999"}">{p:2d}  {esc(sig)}</text>')
        if l:
            c = COL[group(l[1])]
            x1, x2 = j3x + 218, j3x + 330
            out.append(f'<line x1="{x1}" y1="{y}" x2="{x2}" y2="{y}" stroke="{c}" stroke-width="3"/>')
            out.append(f'<polygon points="{x2},{y-6} {x2+10},{y} {x2},{y+6}" fill="{c}"/>')
            lab = f"{short[l[2]]} {l[3]}  {l[1]}"
            out.append(f'<rect x="{x2+14}" y="{y-10}" width="190" height="20" rx="10" fill="{c}" opacity="0.12" stroke="{c}"/>')
            out.append(f'<text x="{x2+24}" y="{y+5}" font-size="13" fill="#000">{esc(lab)}</text>')

    # connectors
    for name, sh, x, y, rows in connectors:
        n = len(rows[0])
        w = 150 * len(rows)
        out.append(f'<rect x="{x}" y="{y-34}" width="{w+20}" height="{n*P+44}" rx="8" fill="#fbf7ef" stroke="#888"/>')
        out.append(f'<text x="{x+(w+20)/2}" y="{y-12}" font-size="15" font-weight="bold" text-anchor="middle">{esc(name)}</text>')
        for r, row in enumerate(rows):
            cx = x + 26 + 150 * r
            for i, (lab, sig) in enumerate(row):
                if not lab:
                    continue
                cy = y + i * P + 12
                j3 = [l[0] for l in links if l[2] == name and l[3] == lab]
                if j3:
                    c = COL[group(sig)]
                    out.append(f'<circle cx="{cx}" cy="{cy}" r="10" fill="{c}" stroke="#333"/>')
                    out.append(f'<text x="{cx}" y="{cy+4}" font-size="10" font-weight="bold" fill="#fff" text-anchor="middle">{j3[0]}</text>')
                else:
                    out.append(f'<circle cx="{cx}" cy="{cy}" r="6" fill="#e2e2e2" stroke="#aaa"/>')
                out.append(f'<text x="{cx+16}" y="{cy+5}" font-size="13" fill="{"#000" if j3 else "#aaa"}">'
                           f'{esc(lab)}  {esc(sig)}</text>')
        out.append(f'<text x="{x+(w+20)/2}" y="{y+n*P+26}" font-size="11" fill="#666" text-anchor="middle">'
                   f'number in the circle = J3 pin</text>')

    # legend and notes
    ly = H - 30 - 20 * len(notes)
    lx = 40
    for g, lab in (("D", "data D0-D7"), ("A", "address A0-A7"), ("CTL", "PHI2, R/W"),
                   ("SEL", "selects"), ("GND", "GND"), ("5V", "+5V")):
        out.append(f'<rect x="{lx}" y="{ly-11}" width="22" height="8" fill="{COL[g]}"/>')
        out.append(f'<text x="{lx+28}" y="{ly-3}" font-size="13">{lab}</text>')
        lx += 170
    for i, n in enumerate(notes):
        out.append(f'<text x="40" y="{ly+24+20*i}" font-size="13" fill="#333">{esc(n)}</text>')
    out.append('</svg>')
    open(fname, "w").write("\n".join(out))


here = os.path.dirname(os.path.abspath(__file__))

# ---- Atari 130XE: cartridge slot (top 1-15, bottom A-S) + ECI ------------
cart_top = [("1", "~S4"), ("2", "A3"), ("3", "A2"), ("4", "A1"), ("5", "A0"),
            ("6", "D4"), ("7", "D5"), ("8", "D2"), ("9", "D1"), ("10", "D0"),
            ("11", "D6"), ("12", "~S5"), ("13", "+5V"), ("14", "RD5"), ("15", "~CCTL")]
cart_bot = [("A", "RD4"), ("B", "GND"), ("C", "A4"), ("D", "A5"), ("E", "A6"),
            ("F", "A7"), ("H", "A8"), ("J", "A9"), ("K", "A12"), ("L", "D3"),
            ("M", "D7"), ("N", "A11"), ("P", "A10"), ("R", "R/W"), ("S", "PHI2")]
eci_top = [("1", "~EXSEL"), ("2", "~RST"), ("3", "~D1XX"), ("4", "~MPD"),
           ("5", "AUDIO"), ("6", "~REF"), ("7", "+5V")]
eci_bot = [("A", "reserved"), ("B", "~IRQ"), ("C", "~HALT"), ("D", "A13"),
           ("E", "A14"), ("F", "A15"), ("H", "GND")]
CART, ECI = "Atari cartridge slot", "Atari 130XE ECI"
sig2cart = {s: l for l, s in cart_top + cart_bot}
ATARI_LINKS = []
for p, s in J3.items():
    if s in sig2cart and s not in ("GND", "+5V"):
        ATARI_LINKS.append((p, s, CART, sig2cart[s]))
ATARI_LINKS += [(11, "~D1XX", ECI, "3"), (29, "GND", CART, "B"), (30, "GND", ECI, "H"),
                (31, "+5V", CART, "13")]
ATARI_NOTES = [
    "Pinouts checked against atarimania.com, computersmuseum.com and the RetroBitLab KiCad library",
    "(ATARI-CARTRIDGE / ECIBUS symbols; footprints: pins 1-15 / 1-7 on the back, A-S / A-H on the front, 1 opposite A).",
    "800XL: the PBI connector carries no ~D1XX; $D1xx must be decoded from A8-A15 (external logic, not shown).",
    "Never connect J3 26 (EN) and 27-28 (+3V3).  Program the ESP32 only disconnected from the computer."]

# ---- Commodore 64 expansion port (top 1-22, bottom A-Z) ------------------
c64_top = [("1", "GND"), ("2", "+5V"), ("3", "+5V"), ("4", "~IRQ"), ("5", "R/W"),
           ("6", "DOTCLK"), ("7", "~IO1"), ("8", "~GAME"), ("9", "~EXROM"), ("10", "~IO2"),
           ("11", "~ROML"), ("12", "BA"), ("13", "~DMA"), ("14", "D7"), ("15", "D6"),
           ("16", "D5"), ("17", "D4"), ("18", "D3"), ("19", "D2"), ("20", "D1"),
           ("21", "D0"), ("22", "GND")]
c64_bot = [("A", "GND"), ("B", "~ROMH"), ("C", "~RESET"), ("D", "~NMI"), ("E", "PHI2"),
           ("F", "A15"), ("H", "A14"), ("J", "A13"), ("K", "A12"), ("L", "A11"),
           ("M", "A10"), ("N", "A9"), ("P", "A8"), ("R", "A7"), ("S", "A6"),
           ("T", "A5"), ("U", "A4"), ("V", "A3"), ("W", "A2"), ("X", "A1"),
           ("Y", "A0"), ("Z", "GND")]
C64 = "C64 expansion port"
sig2c64 = {s: l for l, s in c64_top + c64_bot}
C64_LINKS = []
for p, s in J3.items():
    if s in sig2c64 and s not in ("GND", "+5V"):
        C64_LINKS.append((p, s, C64, sig2c64[s]))
C64_LINKS += [(12, "~IO1", C64, "7"), (11, "~IO2", C64, "10"),
              (29, "GND", C64, "1"), (30, "GND", C64, "Z"), (31, "+5V", C64, "2")]
C64_NOTES = [
    "Pinout checked against c64-wiki.com, retroisle.com and the Programmer's Reference Guide tables.",
    "J3 11 / 12 are labelled ~D1XX / ~CCTL on the board: with the c64 build they read /IO2 / /IO1.",
    "Never connect J3 26 (EN) and 27-28 (+3V3).  Program the ESP32 only disconnected from the computer."]

if __name__ == "__main__":
    draw("6502_monitor - Atari 130XE wiring (env nodemcu-32s)",
         "J3 of the ESP32-BOBOARD-5V to the cartridge slot (~CCTL, bus) and the ECI (~D1XX)",
         [(CART, "CART", 840, 130, [cart_top, cart_bot]), (ECI, "ECI", 840, 590, [eci_top, eci_bot])],
         ATARI_LINKS, ATARI_NOTES, os.path.join(here, "wiring_atari.svg"))
    draw("6502_monitor - Commodore 64 GeoRAM wiring (env c64)",
         "J3 of the ESP32-BOBOARD-5V to the C64 expansion port: /IO1 on J3 12, /IO2 on J3 11",
         [(C64, "C64", 840, 130, [c64_top, c64_bot])],
         C64_LINKS, C64_NOTES, os.path.join(here, "wiring_c64.svg"))
    print("wiring_atari.svg, wiring_c64.svg")
