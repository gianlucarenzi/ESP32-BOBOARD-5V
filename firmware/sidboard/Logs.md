# Project Logs & Architectural Decisions - SIDBOARD

## 2026-05-06 (rev 2): cSIDLight — DAC nativo, nessuna dipendenza esterna

### Motivazione
La dipendenza da FabGL è stata rimossa. L'engine audio è ora completamente
autonomo e integrato nella classe `cSIDLight`. Il motivo principale è la
**garanzia del campionamento del bus Atari**: FabGL imponeva l'uso di
`xQueueSend()` nel path critico di `MonitorTask`, introducendo critical sections
FreeRTOS da 1–5 µs incompatibili con il ciclo PHI2 di ~565 ns PAL.

### Architettura cSIDLight (`include/csidlight.h`)

**Uscita audio**: DAC ESP32 su **GPIO 25** via `dacWrite()`.  
**Engine**: `esp_timer` periodico a **22050 Hz** (~45 µs/campione), task su Core 0.  
**Tre voci**: oscillatori a phase accumulator a 32 bit, onda quadra, formula PAL:
```
phaseInc = (freq_hz × 2³²) / 22050
mix = 128 + Σᵢ (phaseᵢ[31] ? +volᵢ : −volᵢ)   →  clamp [0, 255]  →  DAC
```

#### Sicurezza timing bus Atari

| Metodo | Core | Costo | Note |
|:---|:---|:---|:---|
| `write(reg, val)` | 1 – MonitorTask | **~3 cicli CPU** | singola store volatile, zero FreeRTOS |
| `read(reg)` | 1 – MonitorTask | ~3 cicli CPU | singola load volatile |
| `process()` | 1 – loop() | 32 confronti byte | snapshot `_prevRegs[]`, nessuna queue |
| `_tick()` | 0 – esp_timer | ~50 ns | legge `_phaseInc[]`/`_voiceVol[]`, scrive DAC |

`write()` è ridotto a una singola istruzione `ST` — nessuna critical section,
nessun blocco, nessuna interferenza con la finestra di campionamento PHI2.

`process()` rileva i cambiamenti comparando `_regs[]` con `_prevRegs[]` e aggiorna
i parametri di sintesi (`_phaseInc[]`, `_voiceVol[]`); viene sempre preemptato da
`MonitorTask` (priorità 10) se necessario.

#### Stato condiviso cross-core

`_phaseInc[3]` e `_voiceVol[3]` sono scritti da `process()` (Core 1) e letti da
`_tick()` (Core 0). Dichiarati `volatile uint32_t`/`int32_t`: su ESP32 la DRAM
non è cachata per-core (Xtensa LX6), quindi gli accessi a 32 bit allineati sono
atomici e immediatamente visibili sull'altro core senza barriere aggiuntive.

### Mappa indirizzi $D1xx

| Indirizzo | Accesso | Descrizione |
|:---|:---|:---|
| **$D100–$D11F** | R/W (solo se abilitato) | Registri SID (cSIDLight) |
| **$D120–$D1FE** | R/W (solo se abilitato) | Registri interni ESP32 |
| **$D1FF** | W | Bit 0 = 1: abilita device; Bit 0 = 0: disabilita + reset SID |

L'abilitazione avviene scrivendo `1` nel bit 0 di `$D1FF` (identificazione
device). Tutti gli accessi a `$D100–$D1FE` sono gated su questo flag.
Quando il bit torna a `0`, `cSIDLight::reset()` silenzia le voci e azzera lo stato.

### Mappa registri SID ($D100–$D11F)

| Offset | Registro | Descrizione |
|:---|:---|:---|
| $00–$01 | V1 Freq Lo/Hi | Frequenza Voce 1 (PAL: freq_hz = reg × 985248 / 16777216) |
| $02–$03 | V1 PW Lo/Hi | Pulse Width V1 (non emulato in Light) |
| $04 | V1 CTRL | Bit 0 = GATE (on/off voce), altri bit non emulati |
| $05–$06 | V1 A/D, S/R | ADSR V1 (non emulato in Light) |
| $07–$0D | V2 | Come V1 |
| $0E–$14 | V3 | Come V1 |
| $15–$17 | FC Lo/Hi, Res/Filt | Filtro (non emulato in Light) |
| $18 | MODE/VOL | Bit 3:0 = volume master (0–15 → ±0–120 DAC) |

### Handler PBI `I:` lato Atari

Il ROM handler in `6502/src/pbi-driver.s` registra la periferica Atari **`I:`**.
Le routine `GETBYT` e `PUTBYT` usano l'**accumulatore** per il byte trasferito e
`ICAX1,X` come **offset del registro SID**.

| Routine | Parametri | Comportamento |
|:---|:---|:---|
| `PUTBYT` | `A` = valore, `ICAX1,X` = offset registro | scrive `A` in `$D100 + offset` |
| `GETBYT` | `ICAX1,X` = offset registro | legge `$D100 + offset` e restituisce il valore in `A` |
| `CLOSE` | nessuno | silenzia il SID azzerando l'intera finestra `$D100-$D11F` |

La routine `SILENCE` azzera **32 registri** (`$00-$1F`) per allinearsi alla
finestra completa esposta dal firmware ESP32 (`SID_NUM_REGS = $20`).

### Formato dump compatibile con `sid-arduino-lib`

I file dump generati per l'esempio
`daitangio/sid-arduino-lib/examples/sid_player/sid_player.ino` non contengono
32 byte per frame ma 25.

1. scrive solo i registri **`0..24`**
2. avanza di **25 byte per frame**
3. attende circa **50 Hz** (`delay(19)`) tra un frame e il successivo

Quindi il formato corretto dei dump `_dmp.h`/binari compatibili e':

- **1 frame = 25 byte**
- **registri scritti = 0..24**
- **timing = 1 frame ogni tick video circa**

Per questo motivo gli esempi in:

- `examples/atari-basic/sid_dump_player.bas`
- `examples/cc65/sid_dump_player.c`

trattano il file come una sequenza di frame da **25 byte**, impostano
`ICAX1`/`aux1` con il registro `0..24` e aspettano un tick video tra i frame.

---

## 2026-05-06 (rev 1): Trasformazione in Sidboard

### Overview
Il progetto è stato evoluto da `6502_monitor` a `sidboard`. Obiettivo: integrare
l'emulazione **SID (MOS 6581/8580)** mantenendo il bus monitor PBI/ECI ad alta velocità.

### Key Changes
1. **Pin Reconfiguration**:
   - **Data Bus D5** spostato su **GPIO 3** (ex RX/DEV_SEL).
   - **GPIO 25 (DAC1)** liberato per l'uscita audio.
   - Rimosso `PIN_DEV_SEL`.
2. **Audio Engine**: `cSIDLight` autonoma su DAC GPIO 25 + `esp_timer`.
3. **Multicore**: `MonitorTask` su Core 1 (timing bus); `esp_timer`/audio su Core 0.

### Hardware Mapping

#### ESP32 Pinout (Sidboard)
| Signal | GPIO | Dir | Descrizione |
|:---|:---|:---|:---|
| Data Bus D0–D4 | 18, 19, 21, 22, 23 | Bidirezionale | Bus dati Atari |
| Data Bus D5 | **3** | Bidirezionale | Spostato da 25 per liberare DAC1 |
| Data Bus D6–D7 | 26, 27 | Bidirezionale | Bus dati Atari |
| Audio Output | **25 (DAC1)** | Output | SID Mono (3 voci mixate) |
| Address A0–A5 | 32–36, 39 | Input | Bus indirizzi |
| Address A6–A10 | 16, 17, 14, 12, 13 | Input | Bus indirizzi |
| PHI2 | 2 | Input | Clock 6502 (trigger sync) |
| R/W | 15 | Input | High = Read, Low = Write |
| D1XX | 5 | Input | Selezione pagina device (Active Low) |
| ROM_SEL | 4 | Input | Active Low da 74HC138 ($D800–$DFFF) |
| EXTSEL | 0 | Output | External Select (Active Low) verso Atari |
| Serial TX | 1 | Output | Debug seriale TX-Only |

### Performance & Timing
- **Bus latency**: < 50 ns per l'iniezione dati.
- **write() overhead**: ~3 cicli CPU (singola store) — nessun impatto sul PHI2.
- **Audio sample rate**: 22050 Hz via `esp_timer` su Core 0.
- **Multicore**: MonitorTask fisso su Core 1; audio e system task su Core 0.

### Circuito Audio

**Nota hardware**: GPIO 25 è connesso direttamente al TXS0108 (level translator).
Il TXS0108 è incompatibile con segnali analogici multi-livello (pull-up interni ~4 kΩ +
circuito auto-direzione). È necessario tagliare la traccia PCB tra GPIO 25 e il
pin A del TXS0108, e prelevare il segnale audio direttamente dal lato ESP32.
Dopo il taglio, il pin A del TXS0108 va pullato a GND con un resistore da 10 kΩ
per evitare che il lato B (verso l'Atari) fluttui alto.

#### Stadio 1 — Prototipo: amplificatore esterno

```
GPIO25 ──→ 10µF (DC block) ──→ 270Ω ──+──→ Amplificatore (PAM8403 o equiv.)
                                       |
                                      10nF
                                       |
                                      GND   ← LPF fc ≈ 59 kHz
```

Segnale a ~1.5 Vpp AC dopo il condensatore — sufficiente per qualsiasi ingresso
line-level o ad alta impedenza.

#### Stadio 2 — Versione finale: ECI AUDIO IN → mixer POKEY

```
GPIO25 ──→ 10µF (DC block) ──→ 100kΩ ──+──→ ECI AUDIO IN (POKEY pin 3)
                                        |
                                    10kΩ ╪ 10nF   ← in parallelo, fc ≈ 1.75 kHz
                                        |
                                       GND
```

Il 10nF in parallelo al 10kΩ forma un filtro passa-basso insieme alla sorgente
(100kΩ): fc = (100k+10k)/(2π × 100k×10k × 10nF) ≈ **1.75 kHz**.

Il segnale SID si somma direttamente alle voci POKEY nel mixer interno;
l'uscita audio dell'Atari (TV/monitor) porta SID + POKEY senza amplificatori
esterni. Nessuna traduzione di livello necessaria: AUDIO IN è un ingresso
analogico ad alta impedenza, non un segnale bus digitale.
