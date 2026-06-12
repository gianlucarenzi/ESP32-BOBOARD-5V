#pragma once

#include <Arduino.h>
#include <cstring>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

// SID register offsets (relative to $D500 base)
#define SID_V1_FREQ_LO 0x00
#define SID_V1_FREQ_HI 0x01
#define SID_V1_PW_LO   0x02
#define SID_V1_PW_HI   0x03
#define SID_V1_CTRL    0x04
#define SID_V1_AD      0x05
#define SID_V1_SR      0x06
#define SID_V2_FREQ_LO 0x07
#define SID_V2_FREQ_HI 0x08
#define SID_V2_PW_LO   0x09
#define SID_V2_PW_HI   0x0A
#define SID_V2_CTRL    0x0B
#define SID_V2_AD      0x0C
#define SID_V2_SR      0x0D
#define SID_V3_FREQ_LO 0x0E
#define SID_V3_FREQ_HI 0x0F
#define SID_V3_PW_LO   0x10
#define SID_V3_PW_HI   0x11
#define SID_V3_CTRL    0x12
#define SID_V3_AD      0x13
#define SID_V3_SR      0x14
#define SID_FC_LO      0x15
#define SID_FC_HI      0x16
#define SID_RES_FILT   0x17
#define SID_MODE_VOL   0x18

// Registers 0x00-0x1E are SID; 0x1F is the latch control ($D5FF decoded).
#define SID_NUM_REGS   0x1F

// Voice control register bits
#define SID_CTRL_GATE 0x01
#define SID_CTRL_SYNC 0x02
#define SID_CTRL_RING 0x04
#define SID_CTRL_TEST 0x08
#define SID_CTRL_TRI  0x10
#define SID_CTRL_SAW  0x20
#define SID_CTRL_SQR  0x40
#define SID_CTRL_NOI  0x80

// PAL SID clock frequency (Hz)
#define SID_PAL_CLOCK 985248.0f

// Audio output — GPIO 25 (ESP32 DAC1), freed from address bus A9
#define SID_DAC_PIN     25
#define SID_SAMPLE_RATE 22050
#define SID_TIMER_US    (1000000 / SID_SAMPLE_RATE)  // 45 µs (integer truncation)
#define SID_ACTUAL_RATE (1000000 / SID_TIMER_US)     // 22222 Hz — real timer rate

/**
 * cSIDLight - Lightweight SID 6581/8580 emulator.
 *
 * Registers 0x00-0x1E mapped at $D500-$D51E via CCTL/SEL_N.
 * Register 0x1F ($D5FF decoded) is the latch control — not part of SID.
 * Audio output via ESP32 DAC1 on GPIO 25 at 22050 Hz.
 *
 * Bus-timing safety:
 *  write() is a single volatile byte store — no FreeRTOS primitives, no
 *  critical sections. Safe to call on every PHI2 cycle from MonitorTask
 *  (Core 1, IRAM context).
 *  process() detects register changes via snapshot comparison and updates
 *  synthesis parameters; call periodically from loop() (Core 0).
 *
 * Cross-core shared state (_phaseInc[], _voiceVol[]):
 *  Written by process() on Core 0, read by _tick() on Core 0 (same core,
 *  same esp_timer task). _regs[] written by write() on Core 1, read by
 *  process() on Core 0 — volatile uint8_t is sufficient on Xtensa LX6.
 */
class cSIDLight
{
   public:
    cSIDLight()
    {
        memset((void *)_regs, 0, sizeof(_regs));
        memset(_prevRegs, 0, sizeof(_prevRegs));
        memset(_gate, 0, sizeof(_gate));
        for (int i = 0; i < 3; i++)
        {
            _phase[i]    = 0;
            _phaseInc[i] = 0;
            _voiceVol[i] = 0;
        }

        dacWrite(SID_DAC_PIN, 128);  // DC centre

        esp_timer_create_args_t ta = {};
        ta.callback        = &_timerCb;
        ta.arg             = this;
        ta.dispatch_method = ESP_TIMER_TASK;
        ta.name            = "sid_audio";
        esp_timer_create(&ta, &_timer);
        esp_timer_start_periodic(_timer, SID_TIMER_US);
    }

    ~cSIDLight()
    {
        esp_timer_stop(_timer);
        esp_timer_delete(_timer);
        dacWrite(SID_DAC_PIN, 128);
    }

    /**
     * Write a SID register (0x00-0x1E).
     * Single volatile byte store — safe from MonitorTask (Core 1, IRAM).
     */
    void IRAM_ATTR write(uint8_t reg, uint8_t val)
    {
        if (reg >= SID_NUM_REGS) return;
        _regs[reg] = val;
    }

    /**
     * Read a SID register shadow value.
     * Safe from Core 1 (IRAM context).
     */
    uint8_t IRAM_ATTR read(uint8_t reg) const
    {
        if (reg >= SID_NUM_REGS) return 0xFF;
        return _regs[reg];
    }

    /**
     * Scan _regs[] for changes and update synthesis parameters.
     * O(SID_NUM_REGS) byte comparisons — call from loop() (Core 0).
     */
    void process()
    {
        for (uint8_t i = 0; i < SID_NUM_REGS; i++)
        {
            uint8_t v = _regs[i];
            if (v != _prevRegs[i])
            {
                _prevRegs[i] = v;
                _applyReg(i);
            }
        }
    }

    /**
     * Silence all voices and clear state.
     * Call from loop() when latch is disabled.
     */
    void reset()
    {
        memset((void *)_regs, 0, sizeof(_regs));
        memset(_prevRegs, 0, sizeof(_prevRegs));
        for (int i = 0; i < 3; i++)
        {
            _gate[i]     = false;
            _phaseInc[i] = 0;
            _voiceVol[i] = 0;
        }
    }

   private:
    esp_timer_handle_t _timer;

    volatile uint8_t  _regs[SID_NUM_REGS];
    uint8_t           _prevRegs[SID_NUM_REGS];
    bool              _gate[3];

    uint32_t _phase[3];
    uint32_t _phaseInc[3];
    int32_t  _voiceVol[3];

    static void _timerCb(void *arg)
    {
        static_cast<cSIDLight *>(arg)->_tick();
    }

    void _tick()
    {
        int mix = 128;
        for (int i = 0; i < 3; i++)
        {
            _phase[i] += _phaseInc[i];
            if (_voiceVol[i] == 0) continue;
            mix += (_phase[i] & 0x80000000U) ? _voiceVol[i] : -_voiceVol[i];
        }
        if (mix < 0)   mix = 0;
        if (mix > 255) mix = 255;
        dacWrite(SID_DAC_PIN, (uint8_t)mix);
    }

    static uint32_t _phaseIncFor(int hz)
    {
        return (uint32_t)(((uint64_t)hz << 32) / SID_ACTUAL_RATE);
    }

    static int _sidFreqHz(uint16_t fval)
    {
        int hz = (int)((fval * SID_PAL_CLOCK) / 16777216.0f);
        return hz < 1 ? 1 : hz;
    }

    int _masterVol() const
    {
        return (_regs[SID_MODE_VOL] & 0x0F) * 128 / (3 * 16);
    }

    void _updateVoice(int v)
    {
        const int base = v * 7;
        uint16_t  fval = (uint16_t)_regs[base] | ((uint16_t)_regs[base + 1] << 8);
        uint8_t   ctrl = _regs[base + 4];
        bool      gate = (ctrl & SID_CTRL_GATE) != 0;

        _phaseInc[v] = _phaseIncFor(_sidFreqHz(fval));

        if (gate)
            _voiceVol[v] = _masterVol();
        else if (_gate[v] && !gate)
            _voiceVol[v] = 0;

        _gate[v] = gate;
    }

    void _applyReg(uint8_t reg)
    {
        if (reg < 0x07)
            _updateVoice(0);
        else if (reg < 0x0E)
            _updateVoice(1);
        else if (reg < 0x15)
            _updateVoice(2);
        else if (reg == SID_MODE_VOL)
        {
            int vol = _masterVol();
            for (int i = 0; i < 3; i++)
                if (_gate[i]) _voiceVol[i] = vol;
        }
        // Filter/resonance ($15-$17) not emulated
    }
};
