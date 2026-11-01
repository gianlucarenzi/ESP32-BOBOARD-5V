;
; banked_ram_test.s -- Atari test program for the 6502_monitor banked RAM
;
; The device (GeoRAM style, fixed roles):
;   $D1FB-$D1FC  number of banks (R, 16 bit)
;   $D1FD-$D1FE  bank register (R/W, 16 bit)
;   $D1FF        PBI select latch (W): bit 7 = device ID $80
;   $D500-$D5FF  256 bytes of the current bank (R/W), while selected
;
; Tests:
;   1. COUNT     number of banks (printed), not 0, read-only
;   2. BANK REG  write / read back the 16-bit bank register: low byte
;                $00-$FF with high byte $00, $55, $AA, $FF
;   3. PROTECT   deselected ($7F: bit 7 = 0, other bits set) the device
;                ignores RAM and bank register writes; reselected ($81) the
;                old values are back
;   4. MARCH     fill every bank, verify; then the complement, verify.
;                Bytes 0-1 of each bank hold the bank number, the others
;                offset EOR bank lo EOR bank hi EOR key; all 256 offsets,
;                $D5FF included
;   5. LIMIT     writes to bank = count (no RAM behind it) do not land in
;                the first or the last bank
;   6. KEEP      deselect and reselect: every bank still holds the last
;                pattern
;
; Errors are counted on 24 bits; the first four of each test are printed as
;   ERR D5 B=bbbb O=oo E=ee G=gg   (page, bank, offset, expected, got).
; At the end the device is deselected.  Run time grows with the number of
; banks: about 20 s for 220 KB.
;
; Do not run with a cartridge that uses CCTL ($D5xx) for bank switching
; (XEGS, AtariMax, SpartaDOS X, ...).
;
; Build:  cl65 -t atari -C atari-asm.cfg -o banked_ram_test.xex banked_ram_test.s
; Style: ca65, Atari OS CIO for the screen output (E:).
;

        .include "atari.inc"

        .export start

REGS       = $D100              ; register page
RAMWIN     = $D500              ; RAM window
COUNT_L    = $FB                ; number of banks
COUNT_H    = $FC
BANK_L     = $FD                ; bank register
BANK_H     = $FE
LATCH      = $FF                ; select latch
DEV_ID     = $80                ; PBI device ID bit
MAX_REPORT = 4                  ; detailed error lines per test

; ---------------------------------------------------------------------------
        .segment "ZEROPAGE"

ptr:      .res 2                ; register page $D100
dptr:     .res 2                ; RAM window $D500
sptr:     .res 2                ; string pointer for print
bank:     .res 2                ; current bank
count:    .res 2                ; number of banks
top:      .res 1                ; last RAM offset ($FF)
bk:       .res 1                ; bank lo EOR bank hi EOR key
key:      .res 1                ; pattern key
epage:    .res 1                ; page printed by report
errs:     .res 3                ; total errors (24 bit)
terrs:    .res 3                ; errors at the start of the current test
reported: .res 1                ; detailed error lines printed in this test
expect:   .res 1
got:      .res 1
offs:     .res 1
tmp:      .res 1
hi:       .res 1                ; BANK REG: high byte under test

; ---------------------------------------------------------------------------
        .segment "CODE"

.macro  PRINT   addr
        lda     #<addr
        ldx     #>addr
        jsr     print
.endmacro

start:
        lda     #0
        sta     errs
        sta     errs+1
        sta     errs+2
        lda     #<REGS
        sta     ptr
        lda     #>REGS
        sta     ptr+1
        lda     #<RAMWIN
        sta     dptr
        lda     #>RAMWIN
        sta     dptr+1
        lda     #$FF
        sta     top

        PRINT   s_title
        jsr     select

        ; 1. COUNT
        PRINT   s_count
        jsr     test_reg
        jsr     read_count
        lda     count+1
        jsr     phex
        lda     count
        jsr     phex
        lda     #' '
        jsr     putc
        lda     #0
        sta     bank
        sta     bank+1
        lda     count
        ora     count+1
        bne     @cnt_nz
        sta     expect                  ; no banks: A = 0
        sta     got
        lda     #COUNT_L
        sta     offs
        jsr     report
@cnt_nz:
        lda     count                   ; read-only: writes ignored
        eor     #$FF
        ldy     #COUNT_L
        sta     (ptr),y
        lda     count+1
        eor     #$FF
        ldy     #COUNT_H
        sta     (ptr),y
        ldy     #COUNT_L
        lda     (ptr),y
        cmp     count
        bne     @cnt_err_l
        ldy     #COUNT_H
        lda     (ptr),y
        cmp     count+1
        beq     @cnt_ro
        ldx     count+1
        jmp     @cnt_err
@cnt_err_l:
        ldx     count
@cnt_err:
        sta     got
        stx     expect
        sty     offs
        jsr     report
@cnt_ro:
        jsr     test_end
        lda     count
        ora     count+1
        bne     @bankreg
        jmp     @done                   ; nothing else to test

        ; 2. BANK REG: lo $00-$FF with hi $00, $55, $AA, $FF.
@bankreg:
        PRINT   s_bankreg
        jsr     test_reg
        ldx     #3
@br_hi:
        lda     hi_values,x
        sta     hi
        lda     #0
        sta     bank
@br_lo:
        lda     hi
        sta     bank+1
        jsr     write_bank
        txa
        pha
        jsr     check_bank
        pla
        tax
        inc     bank
        bne     @br_lo
        dex
        bpl     @br_hi
        jsr     test_end

        ; 3. PROTECT
        PRINT   s_protect
        jsr     test_reg
        lda     #0                      ; bank 0, offset 0 = $11
        sta     bank
        sta     bank+1
        jsr     write_bank
        lda     #$11
        ldy     #0
        sta     (dptr),y
        lda     #$7F                    ; deselect: bit 7 = 0
        ldy     #LATCH
        sta     (ptr),y
        lda     #$EE                    ; must be ignored
        ldy     #0
        sta     (dptr),y
        lda     #$07                    ; must be ignored
        ldy     #BANK_L
        sta     (ptr),y
        ldy     #BANK_H
        sta     (ptr),y
        lda     #DEV_ID | $01           ; select: bit 7 = 1
        ldy     #LATCH
        sta     (ptr),y
        jsr     check_bank              ; bank still 0
        jsr     write_bank              ; force bank 0 for the RAM check
        lda     dptr+1
        sta     epage
        ldy     #0                      ; offset 0 still $11
        lda     (dptr),y
        cmp     #$11
        beq     @pr_ram_ok
        sta     got
        lda     #$11
        sta     expect
        lda     #0
        sta     offs
        jsr     report
@pr_ram_ok:
        jsr     test_end

        ; 4. MARCH: pattern, then complement.
        PRINT   s_march
        jsr     test_ram
        lda     #0
        sta     key
        jsr     fill_all
        jsr     verify_all
        lda     #$FF
        sta     key
        jsr     fill_all
        jsr     verify_all
        jsr     test_end

        ; 5. LIMIT: bank = count has no RAM behind it.
        PRINT   s_limit
        jsr     test_ram
        lda     count
        sta     bank
        lda     count+1
        sta     bank+1
        jsr     write_bank
        lda     #$5A
        ldy     #0
        sta     (dptr),y
        ldy     #1
        sta     (dptr),y
        ldy     top
        sta     (dptr),y
        lda     #0                      ; first bank intact
        sta     bank
        sta     bank+1
        jsr     verify_bank
        sec                             ; last bank intact
        lda     count
        sbc     #1
        sta     bank
        lda     count+1
        sbc     #0
        sta     bank+1
        jsr     verify_bank
        jsr     test_end

        ; 6. KEEP: deselect, reselect, the complement pattern is still there.
        PRINT   s_keep
        jsr     test_ram
        jsr     deselect
        jsr     select
        jsr     verify_all
        jsr     test_end

@done:
        jsr     deselect

        lda     errs
        ora     errs+1
        ora     errs+2
        bne     @failed
        PRINT   s_pass
        jmp     halt
@failed:
        PRINT   s_fail1
        lda     errs+2
        jsr     phex
        lda     errs+1
        jsr     phex
        lda     errs
        jsr     phex
        PRINT   s_fail2
halt:
        jmp     halt

hi_values: .byte $00, $55, $AA, $FF

; ---------------------------------------------------------------------------
; Bank patterns.  Bank b, key k:  byte 0 = b lo EOR k, byte 1 = b hi EOR k,
; byte n (2-top) = n EOR b lo EOR b hi EOR k.
; ---------------------------------------------------------------------------
; set_bank: bank register = bank, bk = bank lo EOR bank hi EOR key.
set_bank:
        jsr     write_bank
        lda     bank
        eor     bank+1
        eor     key
        sta     bk
        rts

write_bank:
        lda     bank
        ldy     #BANK_L
        sta     (ptr),y
        lda     bank+1
        ldy     #BANK_H
        sta     (ptr),y
        rts

; check_bank: the bank register reads back bank.  Clobbers A, X, Y.
check_bank:
        lda     ptr+1
        sta     epage
        ldy     #BANK_L
        lda     (ptr),y
        cmp     bank
        bne     @err_l
        ldy     #BANK_H
        lda     (ptr),y
        cmp     bank+1
        beq     @ok
        sta     got
        lda     bank+1
        jmp     @rep
@err_l:
        sta     got
        lda     bank
@rep:
        sta     expect
        sty     offs
        jmp     report
@ok:
        rts

; fill_bank: write the pattern of the current bank.
fill_bank:
        jsr     set_bank
        ldy     top
@byte:
        tya
        eor     bk
        sta     (dptr),y
        dey
        cpy     #2
        bcs     @byte
        lda     bank+1
        eor     key
        sta     (dptr),y                ; y = 1
        dey
        lda     bank
        eor     key
        sta     (dptr),y                ; y = 0
        rts

; verify_bank: check the pattern of the current bank.
verify_bank:
        jsr     set_bank
        ldy     top
@byte:
        tya
        eor     bk
        cmp     (dptr),y
        beq     @next
        jsr     vb_report
@next:
        dey
        cpy     #2
        bcs     @byte
        lda     bank+1                  ; y = 1
        eor     key
        cmp     (dptr),y
        beq     @byte0
        jsr     vb_report
@byte0:
        ldy     #0
        lda     bank
        eor     key
        cmp     (dptr),y
        beq     @done
        jsr     vb_report
@done:
        rts

; vb_report: A = expected, Y = offset; keeps Y.
vb_report:
        sta     expect
        lda     (dptr),y
        sta     got
        sty     offs
        jsr     report
        ldy     offs
        rts

; fill_all / verify_all: banks 0 .. count-1.
fill_all:
        lda     #0
        sta     bank
        sta     bank+1
@loop:
        jsr     fill_bank
        jsr     next_bank
        bne     @loop
        rts

verify_all:
        lda     #0
        sta     bank
        sta     bank+1
@loop:
        jsr     verify_bank
        jsr     next_bank
        bne     @loop
        rts

; next_bank: bank = bank + 1; Z = 1 when bank == count.
next_bank:
        inc     bank
        bne     @cmp
        inc     bank+1
@cmp:
        lda     bank
        cmp     count
        bne     @done
        lda     bank+1
        cmp     count+1
@done:
        rts

read_count:
        ldy     #COUNT_L
        lda     (ptr),y
        sta     count
        ldy     #COUNT_H
        lda     (ptr),y
        sta     count+1
        rts

select:
        lda     #DEV_ID
        ldy     #LATCH
        sta     (ptr),y
        rts

deselect:
        lda     #0
        ldy     #LATCH
        sta     (ptr),y
        rts

; ---------------------------------------------------------------------------
; Error bookkeeping.
; ---------------------------------------------------------------------------
; test_reg / test_ram: test_begin, errors reported on the register / RAM page.
test_reg:
        lda     ptr+1
        sta     epage
        jmp     test_begin

test_ram:
        lda     dptr+1
        sta     epage
        ; fall through

test_begin:
        ldx     #2
@copy:
        lda     errs,x
        sta     terrs,x
        dex
        bpl     @copy
        lda     #0
        sta     reported
        rts

; Prints OK, or FAIL and the number of errors of this test.
test_end:
        ldx     #2
@cmp:
        lda     errs,x
        cmp     terrs,x
        bne     @fail
        dex
        bpl     @cmp
        PRINT   s_ok
        rts
@fail:
        PRINT   s_failn
        sec                             ; terrs = errs - terrs
        ldx     #0
@sub:
        lda     errs,x
        sbc     terrs,x
        sta     terrs,x
        inx
        txa
        eor     #3
        bne     @sub
        lda     terrs+2
        jsr     phex
        lda     terrs+1
        jsr     phex
        lda     terrs
        jsr     phex
        lda     #ATEOL
        jmp     putc

; report: one error at epage / bank / offs, expected / got.  Counts it and
; prints the first MAX_REPORT of the test.  Keeps bank, offs; clobbers A,
; X, Y.
report:
        inc     errs
        bne     @counted
        inc     errs+1
        bne     @counted
        inc     errs+2
@counted:
        lda     reported
        cmp     #MAX_REPORT
        bcs     @done
        lda     reported
        bne     @line                   ; first error of the test: new line
        lda     #ATEOL
        jsr     putc
@line:
        inc     reported
        PRINT   s_err
        lda     epage
        jsr     phex
        PRINT   s_b
        lda     bank+1
        jsr     phex
        lda     bank
        jsr     phex
        PRINT   s_o
        lda     offs
        jsr     phex
        PRINT   s_e
        lda     expect
        jsr     phex
        PRINT   s_g
        lda     got
        jsr     phex
        lda     #ATEOL
        jsr     putc
@done:
        rts

; ---------------------------------------------------------------------------
; Screen output through CIO, IOCB 0 (E:).
; ---------------------------------------------------------------------------
; print: zero-terminated string at A (low) / X (high).
print:
        sta     sptr
        stx     sptr+1
        ldy     #0
@loop:
        lda     (sptr),y
        beq     @done
        sty     tmp
        jsr     putc
        ldy     tmp
        iny
        bne     @loop
@done:
        rts

; phex: A as two hex digits.
phex:
        pha
        lsr     a
        lsr     a
        lsr     a
        lsr     a
        jsr     @nib
        pla
        and     #$0F
@nib:
        cmp     #10
        bcc     @dig
        adc     #6                      ; C = 1: +7, 'A'-'0'-10
@dig:
        adc     #'0'
        ; fall through

; putc: one character in A (CIO PUT CHARACTERS, zero length).
putc:
        ldx     #0                      ; IOCB 0
        ldy     #PUTCHR
        sty     ICCOM
        ldy     #0
        sty     ICBLL
        sty     ICBLH
        jmp     CIOV

; ---------------------------------------------------------------------------
        .segment "RODATA"

s_title:   .byte ATCLR, "BANKED RAM TEST (GEORAM STYLE)", ATEOL
           .byte "RAM $D500-$D5FF, COUNT $D1FB", ATEOL
           .byte "BANK $D1FD, SELECT $D1FF", ATEOL, ATEOL, 0
s_count:   .byte "COUNT    $", 0
s_bankreg: .byte "BANK REG ", 0
s_protect: .byte "PROTECT  ", 0
s_march:   .byte "MARCH    ", 0
s_limit:   .byte "LIMIT    ", 0
s_keep:    .byte "KEEP     ", 0
s_ok:      .byte "OK", ATEOL, 0
s_failn:   .byte "FAIL $", 0
s_err:     .byte "ERR ", 0
s_b:       .byte " B=", 0
s_o:       .byte " O=", 0
s_e:       .byte " E=", 0
s_g:       .byte " G=", 0
s_pass:    .byte ATEOL, "ALL TESTS PASSED", ATEOL, 0
s_fail1:   .byte ATEOL, "ERRORS: $", 0
s_fail2:   .byte ATEOL, 0
