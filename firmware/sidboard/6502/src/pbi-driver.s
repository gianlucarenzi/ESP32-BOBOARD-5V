; PBI Device Driver ROM (MPD) skeleton
; based on Earl Rice ANTIC Magazine
;
; (C) 2025-26 RetroBit Lab
; written by Gianluca Renzi

; Generic OS Parallel Device handler vectors
.define NEWDEV $E486
.define GENDEV $E48F

; Parallel Device Mask
.define PDVMSK $0247

; Activated PBI device
.define PNDEVREQ $248

; Parallel Interrupt Mask
.define PDIMSK $0249

; Generic OS Parallel Device Vector
.define GPDVV $E48F

; Device Handler table
.define HATABS $031A

; Critical code section flag
.define CRITIC $42

; Device Name S[I]D Board
.define DEVNAM 'I'

; OS Equates
.define DDEVIC     $0300
.define DUNIT      $0301
.define DCOMND     $0302
.define DSTATS     $0303
.define DBUFLO     $0304
.define DBUFHI     $0305
.define DTIMLO     $0306
.define DUNUSE     $0307
.define DBYTLO     $0308
.define DBYTHI     $0309
.define DAUX1      $030A
.define DAUX2      $030B

.define IOCB       $0340    ;128-byte I/O control blocks area
.define ICHID      $0340    ;1-byte handler ID ($FF = free)
.define ICDNO      $0341    ;1-byte device number
.define ICCOM      $0342    ;1-byte command code
.define ICSTA      $0343    ;1-byte status of last action
.define ICBAL      $0344    ;1-byte low buffer address
.define ICBAH      $0345    ;1-byte high buffer address
.define ICPTL      $0346    ;1-byte low PUT-BYTE routine address-1
.define ICPTH      $0347    ;1-byte high PUT-BYTE routine address-1
.define ICBLL      $0348    ;1-byte low buffer length
.define ICBLH      $0349    ;1-byte high buffer length
.define ICAX1      $034A    ;1-byte first auxiliary information
.define ICAX2      $034B    ;1-byte second auxiliary information
.define ICSPR      $034C    ;4-byte work area

; The best approach should be using CIO/IOCB routines.
; Having setting the index offset somewhere in the IOCB block so the
; GET/PUT/STATUS vectors will retreive them from the call mechanism.

; Device SID REGISTERS are mapped $D100-$D11F
.define PBI_ADDR        $D100

; SID Register Map Definition
.define SID_BASE		PBI_ADDR

; Voice 1
.define V1_FREQ_LO    SID_BASE + 0   ; Frequency Low
.define V1_FREQ_HI    SID_BASE + 1   ; Frequency High
.define V1_PW_LO      SID_BASE + 2   ; Pulse Width Low
.define V1_PW_HI      SID_BASE + 3   ; Pulse Width High
.define V1_CTRL       SID_BASE + 4   ; Control Register (Waveform, Gate)
.define V1_ADSR       SID_BASE + 5   ; Attack/Decay
.define V1_SUSREL     SID_BASE + 6   ; Sustain/Release

; Voice 2
.define V2_FREQ_LO    SID_BASE + 7
.define V2_FREQ_HI    SID_BASE + 8
.define V2_PW_LO      SID_BASE + 9
.define V2_PW_HI      SID_BASE + 10
.define V2_CTRL       SID_BASE + 11
.define V2_ADSR       SID_BASE + 12
.define V2_SUSREL     SID_BASE + 13

; Voice 3
.define V3_FREQ_LO    SID_BASE + 14
.define V3_FREQ_HI    SID_BASE + 15
.define V3_PW_LO      SID_BASE + 16
.define V3_PW_HI      SID_BASE + 17
.define V3_CTRL       SID_BASE + 18
.define V3_ADSR       SID_BASE + 19
.define V3_SUSREL     SID_BASE + 20

; Filter & Volume
.define FILTER_LO     SID_BASE + 21  ; Filter Cutoff Frequency Low
.define FILTER_HI     SID_BASE + 22  ; Filter Cutoff Frequency High
.define FILTER_RES    SID_BASE + 23  ; Resonance/Voice Routing
.define MODE_VOL      SID_BASE + 24  ; Filter Mode/Volume

; Voice 3 Random/Osc
.define V3_OSC        SID_BASE + 25  ; OSC3 Readout
.define V3_ENV        SID_BASE + 26  ; ENV3 Readout
.define SID_UNUSED_27 SID_BASE + 27
.define SID_UNUSED_28 SID_BASE + 28
.define SID_UNUSED_29 SID_BASE + 29
.define SID_UNUSED_30 SID_BASE + 30
.define SID_UNUSED_31 SID_BASE + 31
.define SID_NUM_REGS   $20

; OS ROM (MPD) $D800 Vector Table

.code
.org $D800 ; OS ROM Vector Table

    .word 0     ; ROM checksum LSB/MSB (optional)
    .byte 0     ; ROM Revision number (optional)

    ; PBI Mandatory ID Number
    .byte $80

    .byte 0     ; Optional Name or Type

    ; LOW LEVEL I/O Vector
    jmp IOVECTOR

    ; INTERRUPT VECTOR
    jmp IRQVECTOR

    ; Mandatory ID Number
    .byte $91

    ; Device Name
    .byte DEVNAM

    ; OPEN VECTOR
    .word NONEED-1

    ; CLOSE VECTOR
    .word CLOSE-1

    ; GET BYTE VECTOR
    .word GETBYT-1

    ; PUT BYTE VECTOR
    .word PUTBYT-1

    ; GET STATUS VECTOR
    .word GETSTA-1

    ; SPECIAL VECTOR
    .word NONEED-1

    ; INIT VECTOR @ PowerUp or RESET
    jmp INIT

    .byte 0     ; NOT USED

; Code starts here

; Not implementing now
IOVECTOR:
    clc
    rts

; no IRQ handling yet
IRQVECTOR:
    rts

; Initialize device and device handler
INIT:
    lda PDVMSK   ; Get Enabled devices PBI flags
    ora PNDEVREQ ; OR in the current device request bit
    sta PDVMSK   ; store the device back

; Put device name in Handler Table HATABS Earl Rice (ANTIC JAN-APR 1985)
;    ldx #0
;SEARCH:
;    lda HATABS,X    ; Get a byte from table
;    beq FNDIT       ; 0? Then we found space
;    inx
;    inx
;    inx
;    cpx #36         ; Length of HATABS
;    bcc SEARCH      ; Still looking
;    rts             ; No room in HATABS device not initialized
;
; We found a spot
;FNDIT:
;    lda #DEVNAM     ; Get Device Name
;    sta HATABS,X    ; Put it in blank spot
;    inx
;    lda #<GPDVV     ; Get LOW BYTE of a vector GPDVV
;    sta HATABS+1,X
;    lda #>GPDVV     ; Get HIGH BYTE of a vector GPDVV
;    sta HATABS+2,X
;    rts

    ; roland scholz/fjc method using the NEWDEV routine in the OS
    ldx #DEVNAM
    lda #>GENDEV
    ldy #<GENDEV
    jsr NEWDEV        ; returns: N = 1 - failed, C = 0 - success, C = 1 - entry already exists
        
    ; TODO: device-specifi init
    jsr SILENCE
    rts

SILENCE:
    lda #0
    ldy #0
clr_sid:
    sta SID_BASE,y
    iny
    cpy #SID_NUM_REGS  ; 32 registers
    bne clr_sid
    rts

CLOSE:
    lda #0
    sta CRITIC                ; Enable deferred vertical blank
    jsr SILENCE
    ldy #1
    sec
    rts

; GET BYTE ROUTINE
GETBYT:
    lda #0
    sta CRITIC                ; Enable deferred vertical blank
    ldy ICAX1,x
    lda SID_BASE,y

    ldy #1
    sec                       ; Indicate we handled it
                              ; Register 'A' holds the value to be read
    rts

; PUT BYTE ROUTINE
PUTBYT:
    pha
    lda #0
    sta CRITIC                ; Enable deferred vertical blank
    ldy ICAX1,x
    pla
    sta SID_BASE,y

    ldy #1
    sec                       ; Indicate we handled it
    rts

; GET STATUS ROUTINE
GETSTA:
    lda #0
    sta CRITIC              ; Enable deferred vertical blank

    ldy #1
    sec                     ; Indicate we handled it
                            ; Register 'A' holds the value to be read
    rts

; DO NOTHING ROUTINE
NONEED:
    ldy #1
    sec                     ; Indicate we handled it
    rts
