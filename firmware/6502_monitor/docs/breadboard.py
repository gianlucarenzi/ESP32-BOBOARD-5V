#!/usr/bin/env python3
"""Wiring drawings in the style of the Fritzing breadboard view: the
ESP32-BOBOARD-5V with its J3 header, the computer edge connector with gold
contacts and thick coloured wires routed in straight segments.  Same data
as wiring.py (J3 from the netlist, connector pinouts checked online and
against the RetroBitLab KiCad library).

Writes breadboard_atari.svg and breadboard_c64.svg.
"""
import os

import wiring as W

HERE = os.path.dirname(os.path.abspath(__file__))

# Fritzing wire colours
WIRE = {"D": "#418dd9",      # blue
        "A": "#47cc79",      # green
        "PHI2": "#ffe24d",   # yellow
        "R/W": "#ff7033",    # orange
        "SEL": "#b673e6",    # purple
        "GND": "#404040",    # black
        "5V": "#cc1414"}     # red


def wire_col(sig):
    g = W.group(sig)
    if g == "CTL":
        return WIRE["PHI2"] if sig == "PHI2" else WIRE["R/W"]
    return WIRE[g]


def darker(c, f=0.6):
    r, g, b = (int(c[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % (int(r * f), int(g * f), int(b * f))


def esc(t):
    return W.esc(t)


def draw(title, subtitle, connectors, links, notes, fname):
    """connectors: list of (name, short, rows) with rows = [numbered row,
    lettered row], each a list of (pin label, signal)."""
    # One pitch for J3 and the connectors; the connector rows sit 12 / 24 px
    # off the J3 grid, so no vertical wire run overlaps another one.
    PJ = PC = 36
    X0 = 60
    WIDTH = X0 * 2 + 32 * PJ + 40
    BOARD_Y, BOARD_H = 90, 240
    HDR_Y = BOARD_Y + BOARD_H - 72        # J3 header body top (2 rows)
    LANE_Y = BOARD_Y + BOARD_H + 40
    LANE_DY = 12
    CONN_Y = LANE_Y + len(links) * LANE_DY + 50
    H = CONN_Y + 250 + 20 * len(notes)
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{H}" '
         f'viewBox="0 0 {WIDTH} {H}" font-family="DejaVu Sans, Arial, sans-serif">',
         f'<rect width="{WIDTH}" height="{H}" fill="#ffffff"/>',
         f'<text x="{WIDTH/2}" y="36" font-size="22" font-weight="bold" text-anchor="middle">{esc(title)}</text>',
         f'<text x="{WIDTH/2}" y="60" font-size="13" fill="#555" text-anchor="middle">{esc(subtitle)}</text>']

    # ---- the board -----------------------------------------------------
    bw = 32 * PJ + 40
    o.append(f'<rect x="{X0-4}" y="{BOARD_Y+4}" width="{bw}" height="{BOARD_H}" rx="10" fill="#000" opacity="0.18"/>')
    o.append(f'<rect x="{X0-8}" y="{BOARD_Y}" width="{bw}" height="{BOARD_H}" rx="10" fill="#1d6b3c" stroke="#0f4626" stroke-width="2"/>')
    for hx in (X0 + 4, X0 + bw - 20):            # mounting holes
        for hy in (BOARD_Y + 12, BOARD_Y + BOARD_H - 12):
            o.append(f'<circle cx="{hx}" cy="{hy}" r="6" fill="#ffffff" stroke="#c9a24a" stroke-width="3"/>')
    # NodeMCU module silhouette
    mx, my = X0 + 330, BOARD_Y + 14
    o.append(f'<rect x="{mx}" y="{my}" width="300" height="78" rx="4" fill="#202020"/>')
    o.append(f'<rect x="{mx+150}" y="{my+8}" width="120" height="62" rx="3" fill="#b8bcc0" stroke="#8a8e92"/>')
    o.append(f'<text x="{mx+210}" y="{my+44}" font-size="11" text-anchor="middle" fill="#333">ESP32-WROOM-32</text>')
    o.append(f'<text x="{mx+70}" y="{my+44}" font-size="12" text-anchor="middle" fill="#ddd">NodeMCU-32S</text>')
    o.append(f'<text x="{X0+60}" y="{BOARD_Y+40}" font-size="15" font-weight="bold" fill="#ffffff">ESP32-BOBOARD-5V</text>')
    o.append(f'<text x="{X0+60}" y="{BOARD_Y+58}" font-size="11" fill="#d8f0d8">3 x TXS0108E  3.3 V / 5 V</text>')
    # J3 header: 2x16, numbered as the PCB footprint pads (PinHeader_2x16
    # odd/even: 1 opposite 2 ... 31 opposite 32), seen from the top with
    # pin 1 on the left: odd pins on the lower row, even pins on the upper.
    # An even pin's wire leaves sideways by half a column, so its vertical
    # run passes between the odd pins of the lower row.
    o.append(f'<rect x="{X0+14}" y="{HDR_Y}" width="{32*PJ-24}" height="50" rx="2" fill="#2b2b2b"/>')
    o.append(f'<text x="{X0+60}" y="{BOARD_Y+76}" font-size="11" fill="#d8f0d8">'
             f'<tspan font-weight="bold" fill="#ffffff">J3</tspan> 2x16: odd pins lower row '
             f'(pin 1 left), even pins upper row</text>')
    jx = {}
    jexit = {}
    used = {l[0] for l in links}
    for p in range(1, 33):
        cx = X0 + 8 + PJ / 2 + ((p - 1) // 2) * 2 * PJ
        cy = HDR_Y + (37 if p % 2 else 13)
        jx[p] = (cx, cy)
        jexit[p] = [] if p % 2 else [(cx + PJ, cy)]
        o.append(f'<rect x="{cx-4}" y="{cy-4}" width="8" height="8" fill="#d4b04c" stroke="#8a6d1f"/>')
        sig = W.J3[p]
        col = "#ffffff" if p in used else "#9fc7a9"
        lab = f"{p} {sig.split(' ')[0]}"
        tx = cx - 4 if p % 2 else cx + 12
        o.append(f'<text x="{tx}" y="{HDR_Y-6}" font-size="10" fill="{col}" '
                 f'transform="rotate(-90 {tx} {HDR_Y-6})">{esc(lab)}</text>')

    # ---- the computer connectors ----------------------------------------
    cpos = {}
    cx0 = X0 + 8 + PJ                     # on the J3 grid
    for name, short, rows in connectors:
        n = len(rows[0])
        w = n * PC
        o.append(f'<rect x="{cx0}" y="{CONN_Y-14}" width="{w+PC/2}" height="62" rx="5" fill="#3a3a3a" stroke="#1e1e1e" stroke-width="2"/>')
        o.append(f'<rect x="{cx0+6}" y="{CONN_Y+10}" width="{w+PC/2-12}" height="6" fill="#141414"/>')
        o.append(f'<text x="{cx0+w/2}" y="{CONN_Y+150}" font-size="14" font-weight="bold" text-anchor="middle">{esc(name)}</text>')
        for r, row in enumerate(rows):
            for i, (lab, sig) in enumerate(row):
                x = cx0 + PC / 2 + i * PC + (24 if r else 12)
                y = CONN_Y + (0 if r == 0 else 26)
                on = any(l[2] == name and l[3] == lab for l in links)
                o.append(f'<rect x="{x-5}" y="{y-6}" width="10" height="12" rx="1" '
                         f'fill="{"#e6c35c" if on else "#9c8a52"}" stroke="#7a5e14"/>')
                cpos[(name, lab)] = (x, y)
                ty = CONN_Y + 60 + (0 if r == 0 else 0)
                tx = x
                o.append(f'<text x="{tx+4}" y="{ty}" font-size="10" fill="{"#000" if on else "#aaa"}" '
                         f'transform="rotate(90 {tx+4} {ty})">{esc(lab)} {esc(sig)}</text>')
        o.append(f'<text x="{cx0+w/2}" y="{CONN_Y+168}" font-size="11" fill="#555" text-anchor="middle">'
                 f'upper row {rows[0][0][0]}-{rows[0][-1][0]}, lower row {rows[1][0][0]}-{rows[1][-1][0]} '
                 f'(drawn offset; on the edge 1 is opposite A)</text>')
        cx0 += w + 2 * PC

    # ---- the wires -------------------------------------------------------
    order = sorted(links, key=lambda l: cpos[(l[2], l[3])][0])
    segs = []
    for k, (p, sig, cname, lab) in enumerate(order):
        start = [jx[p]] + jexit[p]
        x1 = start[-1][0]
        x2, y2 = cpos[(cname, lab)]
        ly = LANE_Y + k * LANE_DY
        pts = start + [(x1, ly), (x2, ly), (x2, y2)]
        segs.append((pts, wire_col(sig)))
    for pts, c in segs:                   # outline, then colour, Fritzing style
        d = " ".join(f"{x},{y}" for x, y in pts)
        o.append(f'<polyline points="{d}" fill="none" stroke="{darker(c)}" stroke-width="7" '
                 f'stroke-linejoin="round" stroke-linecap="round"/>')
        o.append(f'<polyline points="{d}" fill="none" stroke="{c}" stroke-width="4.5" '
                 f'stroke-linejoin="round" stroke-linecap="round"/>')
    for pts, c in segs:                   # wire ends
        for x, y in (pts[0], pts[-1]):
            o.append(f'<circle cx="{x}" cy="{y}" r="4.5" fill="{c}" stroke="{darker(c)}" stroke-width="1.5"/>')

    # ---- legend and notes -------------------------------------------------
    ly = H - 30 - 20 * len(notes)
    lx = X0
    for key, lab in (("D", "D0-D7"), ("A", "A0-A7"), ("PHI2", "PHI2"), ("R/W", "R/W"),
                     ("SEL", "selects"), ("GND", "GND"), ("5V", "+5V")):
        c = WIRE[key]
        o.append(f'<line x1="{lx}" y1="{ly-6}" x2="{lx+30}" y2="{ly-6}" stroke="{darker(c)}" stroke-width="7" stroke-linecap="round"/>')
        o.append(f'<line x1="{lx}" y1="{ly-6}" x2="{lx+30}" y2="{ly-6}" stroke="{c}" stroke-width="4.5" stroke-linecap="round"/>')
        o.append(f'<text x="{lx+40}" y="{ly-1}" font-size="13">{lab}</text>')
        lx += 140
    for i, n in enumerate(notes):
        o.append(f'<text x="{X0}" y="{ly+24+20*i}" font-size="12" fill="#333">{esc(n)}</text>')
    o.append('</svg>')
    open(fname, "w").write("\n".join(o))


if __name__ == "__main__":
    draw("6502_monitor - Atari 130XE wiring, breadboard view (env nodemcu-32s)",
         "J3 of the ESP32-BOBOARD-5V to the cartridge slot (~CCTL, bus) and the ECI (~D1XX)",
         [(W.CART, "CART", [W.cart_top, W.cart_bot]), (W.ECI, "ECI", [W.eci_top, W.eci_bot])],
         W.ATARI_LINKS, W.ATARI_NOTES, os.path.join(HERE, "breadboard_atari.svg"))
    draw("6502_monitor - Commodore 64 GeoRAM wiring, breadboard view (env c64)",
         "J3 of the ESP32-BOBOARD-5V to the C64 expansion port: /IO1 on J3 12, /IO2 on J3 11",
         [(W.C64, "C64", [W.c64_top, W.c64_bot])],
         W.C64_LINKS, W.C64_NOTES, os.path.join(HERE, "breadboard_c64.svg"))
    print("breadboard_atari.svg, breadboard_c64.svg")
