# GEODISK.SYS: D9: RAM disk (Atari 130XE, MyDOS 4.5x)

`GEODISK.SYS` turns the 6502_monitor paged RAM (200+ KB, 256-byte window at
`$D500`, registers at `$D1FB-$D1FF`) into a RAM disk on **D9:**, one 256-byte
sector per bank: about 850 sectors, the exact number is the bank count of
the device.

## How it works

The device has no PBI ROM, so the OS does not know it as a disk.  The
installer, run once at load time (INITAD), then back to DOS:

1. finds the device: `$80` to `$D1FF`, bank count at `$D1FB-$D1FC`;
2. copies the resident handler (423 bytes, relocated) to MEMLO, page
   aligned, and raises MEMLO;
3. copies the OS ROM (`$C000-$CFFF`, `$D800-$FFFF`) to the RAM under it,
   switches the ROM off (PORTB bit 0 = 0) and points the SIO vector **SIOV**
   (`$E459`) at the handler;
4. hooks **DOSINI**: after RESET the OS copy is switched on again and MEMLO
   stays above the handler.

Every SIO call for device `$31` unit 9 (D9:) is then served by the handler;
the others go to the original SIO.

| SIO command | Answer |
|---|---|
| `R` read, `W` / `P` write | sector n = bank n-1; the length the DOS asks for (128 bytes for sectors 1-3 of a double density disk, 256 for the others) |
| `S` status | double density (`$20`), 4 bytes |
| `$4E` read PERCOM | 1 track of *count* sectors, 256 bytes per sector, MFM |
| `$4F` write PERCOM | accepted, ignored |
| `!` / `"` format | every sector zeroed, no bad sectors |
| other | NAK (`$8B`) |

Sector 0 or beyond the count: error `$90`.  The device is selected
(`$80` to `$D1FF`, and the `SHPDVS` shadow, as the PBI convention wants)
only during a call and deselected at the end.  The RAM disk survives RESET
(the device does not see it) but not an ESP32 reset or a power cycle.

## Build

```sh
make            # GEODISK.SYS (mkgeodisk.py: resident part at $2000 and $2100,
                # relocation table from the bytes that differ, installer at
                # $6000, Atari executable with INITAD)
make test       # py65: fake OS + device model, two MEMLO values
```

## Use with MyDOS 4.5x

1. Copy `GEODISK.SYS` to the MyDOS disk and load it with DUP `L` (or rename
   it `AUTORUN.SYS`, or chain it from your own `AUTORUN.SYS`).  It prints
   `D9: <n> SECTORS OF 256 BYTES`, or `NO DEVICE` if the board does not
   answer (then nothing is changed).
2. In DUP, configure **drive 9** as a double density, high capacity drive of
   *n* sectors (option `O`; the handler also answers the PERCOM query).
   Do **not** give D9: to the MyDOS internal 130XE RAM disk.
3. Format D9: with `I` (initialize), then use it as any drive.
4. Save the MyDOS configuration (`H`, write DOS files) if you want D9: set
   up at every boot; `GEODISK.SYS` has to be loaded at every boot anyway.

The exact DUP dialogue depends on the MyDOS version: check its manual for
the drive configuration of high capacity drives.

## Limits

- **XL / XE only** (OS RAM under the ROM, PORTB bit 0).  The OS runs from
  RAM while GEODISK is installed: a program that switches the OS ROM back on
  (PORTB bit 0 = 1) disables D9: until the next RESET, and the MyDOS
  130XE RAM disk or other programs that rewrite PORTB must keep bit 0 = 0.
- Cartridges that bank-switch through CCTL (`$D5xx`) cannot be used.
- Tested on py65 with a fake OS only: still to be tried on a real 130XE
  with MyDOS (the OS copy, the RESET path and the DUP configuration).
