;
; geodisk_res.s -- resident part of GEODISK.SYS: the D9: RAM disk handler
;
; Hooked into the SIO vector (SIOV, $E459, in the OS copied to RAM): calls
; for device $31 unit 9 (D9:) are served from the 6502_monitor paged RAM,
; everything else goes to the original SIO.  One sector = one 256-byte bank
; of the $D500 window (sector n = bank n-1); sectors 1-3 are transferred
; with the length the DOS asks for (128 bytes on a double density disk).
;
; Device: $D1FF PBI latch (bit 7), $D1FB-$D1FC bank count, $D1FD-$D1FE bank
; register, $D500-$D5FF window.
;
; Assembled twice (at $2000 and $2100) by mkgeodisk.py, which builds the
; relocation table from the bytes that differ: the code is copied to a
; page-aligned address at MEMLO, so only high bytes of addresses change.
; Do not use <label on resident labels in a way that depends on the page.
;

DDEVIC  = $0300
DUNIT   = $0301
DCOMND  = $0302
DSTATS  = $0303
DBUFLO  = $0304
DBUFHI  = $0305
DBYTLO  = $0308
DBYTHI  = $0309
DAUX1   = $030A
DAUX2   = $030B
SHPDVS  = $0248                 ; shadow of the PBI select register
MEMLO   = $02E7
PORTB   = $D301

DEVSEL  = $D1FF                 ; PBI select latch
BANKL   = $D1FD                 ; bank register
BANKH   = $D1FE
WIN     = $D500                 ; RAM window
DEV_ID  = $80
DISK    = $31                   ; SIO device: disk drive
UNIT    = 9                     ; D9:

        .segment "CODE"

; ---- header read by the installer (all words relocated) -------------------
res_start:
        .word   handler         ; +0  new SIOV target
        .word   reinit          ; +2  new DOSINI
        .word   old_siov+1      ; +4  where the original SIOV target goes
        .word   old_dosini+1    ; +6  where the original DOSINI goes
        .word   nsect           ; +8  number of sectors
        .word   percom          ; +10 PERCOM block
        .word   res_end         ; +12 new MEMLO
        .byte   "GEOD"          ; +14 signature

nsect:  .word   0
; PERCOM: tracks, step, sectors/track hi, lo, sides-1, density (4 = MFM),
; bytes/sector hi, lo, drive online, 3 unused
percom: .byte   1, 0, 0, 0, 0, 4, 1, 0, $FF, 0, 0, 0
; status: double density, controller status, timeout, unused
status: .byte   $20, $FF, $E0, $00
len:    .byte   0               ; bytes to transfer (0 = 256)
snum:   .word   0               ; sector number

; ---- SIO entry ------------------------------------------------------------
handler:
        lda     DDEVIC
        cmp     #DISK
        bne     old_siov
        lda     DUNIT
        cmp     #UNIT
        beq     mine
old_siov:
        jmp     $FFFF           ; original SIO, patched by the installer

mine:
        lda     DBUFLO          ; buffer address into the copy loops
        sta     rd_dst+1
        sta     wr_src+1
        sta     tb_dst+1
        sta     fm_dst+1
        lda     DBUFHI
        sta     rd_dst+2
        sta     wr_src+2
        sta     tb_dst+2
        sta     fm_dst+2
        lda     DBYTHI          ; 256 bytes (0) or DBYTLO
        beq     @lo
        lda     #0
        beq     @len
@lo:    lda     DBYTLO
@len:   sta     len
        lda     DCOMND
        cmp     #$52            ; 'R' read sector
        beq     cmd_read
        cmp     #$57            ; 'W' write with verify
        beq     cmd_write
        cmp     #$50            ; 'P' put (write)
        beq     cmd_write
        cmp     #$53            ; 'S' status
        beq     cmd_status
        cmp     #$4E            ; read PERCOM
        beq     cmd_percom
        cmp     #$4F            ; write PERCOM: accepted, ignored
        beq     cmd_ok
        cmp     #$21            ; '!' format
        beq     cmd_format
        cmp     #$22            ; '"' format medium density
        beq     cmd_format
        ldy     #$8B            ; NAK: command not supported
        jmp     done

cmd_ok:
        ldy     #1
        jmp     done

bad_sector:
        ldy     #$90            ; device error
        jmp     done

cmd_read:
        jsr     set_sector
        bcs     bad_sector
        ldx     len
        ldy     #0
rd_loop:  lda     WIN,y
rd_dst: sta     $FFFF,y
        iny
        dex
        bne     rd_loop
        ldy     #1
        jmp     done

cmd_write:
        jsr     set_sector
        bcs     bad_sector
        ldx     len
        ldy     #0
wr_loop:
wr_src: lda     $FFFF,y
        sta     WIN,y
        iny
        dex
        bne     wr_loop
        ldy     #1
        jmp     done

cmd_status:
        lda     #<status
        sta     tb_src+1
        lda     #>status
        sta     tb_src+2
        jmp     copy_tab

cmd_percom:
        lda     #<percom
        sta     tb_src+1
        lda     #>percom
        sta     tb_src+2
copy_tab:
        ldx     len
        ldy     #0
tb_loop:
tb_src: lda     $FFFF,y
tb_dst: sta     $FFFF,y
        iny
        dex
        bne     tb_loop
        ldy     #1
        jmp     done

; Format: every sector to $00, then the bad sector list (none: $FF).
cmd_format:
        jsr     select
        lda     #0
        sta     snum
        sta     snum+1
@bank:  lda     snum
        cmp     nsect
        bne     @clear
        lda     snum+1
        cmp     nsect+1
        beq     @list
@clear: lda     snum
        sta     BANKL
        lda     snum+1
        sta     BANKH
        lda     #0
        tay
@zero:  sta     WIN,y
        iny
        bne     @zero
        inc     snum
        bne     @bank
        inc     snum+1
        jmp     @bank
@list:  ldx     len
        ldy     #0
        lda     #$FF
fm_loop:
fm_dst: sta     $FFFF,y
        iny
        dex
        bne     fm_loop
        ldy     #1
        jmp     done

; set_sector: sector DAUX1/DAUX2 must be 1..nsect; selects the device and
; the bank (sector - 1).  C = 1 if out of range.
set_sector:
        lda     DAUX1
        sta     snum
        lda     DAUX2
        sta     snum+1
        ora     snum
        beq     @bad            ; sector 0
        lda     nsect           ; nsect - sector < 0: out of range
        cmp     snum
        lda     nsect+1
        sbc     snum+1
        bcc     @bad
        jsr     select
        lda     snum
        sec
        sbc     #1
        sta     BANKL
        lda     snum+1
        sbc     #0
        sta     BANKH
        clc
        rts
@bad:   sec
        rts

; select: the device answers at $D1xx / $D5xx.  SHPDVS follows the PBI
; convention, so that an IRQ from another PBI device restores our select.
select:
        lda     #DEV_ID
        sta     SHPDVS
        sta     DEVSEL
        rts

; done: deselect, status in Y and DSTATS, N flag set on an error.
done:
        lda     #0
        sta     DEVSEL
        sta     SHPDVS
        sty     DSTATS
        ldy     DSTATS
        rts

; ---- DOSINI: after RESET -------------------------------------------------
reinit:
old_dosini:
        jsr     $FFFF           ; original DOSINI, patched by the installer
        lda     PORTB           ; the OS copy in RAM again (and our hook)
        and     #$FE
        sta     PORTB
        lda     #<res_end       ; keep the handler below MEMLO
        sta     MEMLO
        lda     #>res_end
        sta     MEMLO+1
        rts

res_end:
