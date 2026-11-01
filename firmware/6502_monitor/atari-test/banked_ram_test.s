;
; banked_ram_test.s -- Atari test program for the 6502_monitor banked RAM
;
; Tests both windows of the ESP32-BOBOARD-5V banked RAM peripheral:
;   $xx00-$xx7F  RAM window (128 bytes of the current bank)
;   $xxFB-$xxFC  number of banks of the window (R, 16 bit)
;   $xxFD-$xxFE  bank register (R/W, 16 bit)
;   $xxFF        select latch (W, bit 7 = device ID $80)
; with xx = $D1 (PBI, ~D1XX) and $D5 (CCTL, ~CCTL).  The number of banks
; depends on the ESP32 heap and is read from $xxFB-$xxFC.
;
; Per window:
;   1. COUNT     number of banks (printed), not 0, read-only
;   2. BANK REG  write / read back the 16-bit bank register: low byte
;                $00-$FF with high byte $00, $55, $AA, $FF
;   3. PROTECT   deselected (latch written with $7F: bit 7 = 0, other bits
;                set) the device must ignore RAM and bank register writes;
;                reselected with $81 (bit 7 = 1) the old values are back
;   4. MARCH     fill every bank, verify; then the complement, verify.
;                Bytes 0-1 of each bank hold the bank number, the others
;                offset EOR bank lo EOR bank hi: stuck data bits, address
;                aliasing and banks that alias each other
;   5. LIMIT     writes to bank = count (beyond the RAM) must not land in
;                the first or the last bank
; Then:
;   6. KEEP      window $D1 again, after the $D5 tests and a deselect: its
;                last pattern must still be there (windows independent,
;                RAM kept while deselected)
;
; Errors are counted on 24 bits; the first four of each test are printed as
;   ERR D1 B=bbbb O=oo E=ee G=gg   (window, bank, offset, expected, got).
; At the end both windows are deselected.  Run time grows with the number
; of banks: a few seconds per 64 KB.
;
; Do not run with a cartridge that uses CCTL ($D5xx) for bank switching
; (XEGS, AtariMax, SpartaDOS X, ...): set TEST_CCTL = 0 to skip $D5xx.
;
; Build:  cl65 -t atari -C atari-asm.cfg -o banked_ram_test.xex banked_ram_test.s
; Style: ca65, Atari OS CIO for the screen output (E:).
;

        .include "atari.inc"

        .export start

TEST_CCTL  = 1                  ; 0 = test only the $D1xx window

RAM_SIZE   = $80                ; RAM window $xx00-$xx7F
COUNT_L    = $FB                ; number of banks
COUNT_H    = $FC
BANK_L     = $FD                ; bank register
BANK_H     = $FE
LATCH      = $FF                ; select latch
DEV_ID     = $80                ; PBI device ID bit
MAX_REPORT = 4                  ; detailed error lines per test

; ---------------------------------------------------------------------------
        .segment "ZEROPAGE"

ptr:      .res 2                ; window base: $D100 / $D500
sptr:     .res 2                ; string pointer for print
bank:     .res 2                ; current bank
count:    .res 2                ; banks of the window
bk:       .res 1                ; bank lo EOR bank hi EOR key
key:      .res 1                ; window key EOR pass inversion
wkey:     .res 1                ; window key
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

        PRINT   s_title

        lda     #$D1
        ldx     #$00
        jsr     test_window

.if TEST_CCTL
        lda     #$D5
        ldx     #$A5
        jsr     test_window
.endif

        ; 6. KEEP: $D1 window still holds its complemented pattern.
        PRINT   s_keep
        jsr     test_begin
        lda     #$D1
        sta     ptr+1
        lda     #0
        sta     ptr
        sta     wkey
        jsr     select
        jsr     read_count
        lda     #$FF
        eor     wkey
        sta     key
        lda     count
        ora     count+1
        beq     @keep_done
        jsr     verify_all
@keep_done:
        jsr     deselect
        jsr     test_end

        ; Summary.
.if TEST_CCTL
        lda     #$D5
        sta     ptr+1
        jsr     deselect
.endif
        lda     #$D1
        sta     ptr+1
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

; ---------------------------------------------------------------------------
; test_window: A = page ($D1 / $D5), X = pattern key of the window.
; ---------------------------------------------------------------------------
test_window:
        sta     ptr+1
        lda     #0
        sta     ptr
        stx     wkey
        jsr     select

        ; 1. COUNT
        jsr     print_win
        PRINT   s_count
        jsr     test_begin
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
        jsr     print_win
        PRINT   s_bankreg
        jsr     test_begin
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
        ldy     #BANK_L
        lda     (ptr),y
        cmp     bank
        bne     @br_err_l
        ldy     #BANK_H
        lda     (ptr),y
        cmp     bank+1
        beq     @br_ok
        sta     got
        lda     bank+1
        jmp     @br_rep
@br_err_l:
        sta     got
        lda     bank
@br_rep:
        sta     expect
        sty     offs
        txa
        pha
        jsr     report
        pla
        tax
@br_ok:
        inc     bank
        bne     @br_lo
        dex
        bpl     @br_hi
        jsr     test_end

        ; 3. PROTECT
        jsr     print_win
        PRINT   s_protect
        jsr     test_begin
        lda     #0                      ; bank 0, offset 0 = $11
        sta     bank
        sta     bank+1
        jsr     write_bank
        lda     #$11
        ldy     #0
        sta     (ptr),y
        lda     #$7F                    ; deselect: bit 7 = 0
        ldy     #LATCH
        sta     (ptr),y
        lda     #$EE                    ; must be ignored
        ldy     #0
        sta     (ptr),y
        lda     #$07                    ; must be ignored
        ldy     #BANK_L
        sta     (ptr),y
        ldy     #BANK_H
        sta     (ptr),y
        lda     #DEV_ID | $01           ; select: bit 7 = 1
        ldy     #LATCH
        sta     (ptr),y
        ldy     #BANK_L                 ; bank still 0
        lda     (ptr),y
        bne     @pr_bank_err
        ldy     #BANK_H
        lda     (ptr),y
        beq     @pr_bank_ok
@pr_bank_err:
        sta     got
        sty     offs
        lda     #0
        sta     expect
        jsr     report
@pr_bank_ok:
        jsr     write_bank              ; force bank 0 for the RAM check
        ldy     #0                      ; offset 0 still $11
        lda     (ptr),y
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
        jsr     print_win
        PRINT   s_march
        jsr     test_begin
        lda     wkey
        sta     key
        jsr     fill_all
        jsr     verify_all
        lda     #$FF
        eor     wkey
        sta     key
        jsr     fill_all
        jsr     verify_all
        jsr     test_end

        ; 5. LIMIT: bank = count has no RAM behind it.
        jsr     print_win
        PRINT   s_limit
        jsr     test_begin
        lda     count
        sta     bank
        lda     count+1
        sta     bank+1
        jsr     write_bank
        lda     #$5A
        ldy     #0
        sta     (ptr),y
        ldy     #1
        sta     (ptr),y
        ldy     #RAM_SIZE - 1
        sta     (ptr),y
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

@done:
        jsr     deselect
        rts

hi_values: .byte $00, $55, $AA, $FF

; ---------------------------------------------------------------------------
; Bank patterns.  Bank b, key k:  byte 0 = b lo EOR k, byte 1 = b hi EOR k,
; byte n (2-$7F) = n EOR b lo EOR b hi EOR k.
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

; fill_bank: write the pattern of the current bank.
fill_bank:
        jsr     set_bank
        ldy     #RAM_SIZE - 1
@byte:
        tya
        eor     bk
        sta     (ptr),y
        dey
        cpy     #2
        bcs     @byte
        lda     bank+1
        eor     key
        sta     (ptr),y                 ; y = 1
        dey
        lda     bank
        eor     key
        sta     (ptr),y                 ; y = 0
        rts

; verify_bank: check the pattern of the current bank.
verify_bank:
        jsr     set_bank
        ldy     #RAM_SIZE - 1
@byte:
        tya
        eor     bk
        cmp     (ptr),y
        beq     @next
        jsr     vb_report
@next:
        dey
        cpy     #2
        bcs     @byte
        lda     bank+1                  ; y = 1
        eor     key
        cmp     (ptr),y
        beq     @byte0
        jsr     vb_report
@byte0:
        ldy     #0
        lda     bank
        eor     key
        cmp     (ptr),y
        beq     @done
        jsr     vb_report
@done:
        rts

; vb_report: A = expected, Y = offset; keeps Y.
vb_report:
        sta     expect
        lda     (ptr),y
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

; report: one error at bank / offs, expected / got.  Counts it and prints
; the first MAX_REPORT of the test.  Keeps bank, offs; clobbers A, X, Y.
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
        lda     ptr+1
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
; print_win: "D1 " / "D5 "
print_win:
        lda     ptr+1
        jsr     phex
        lda     #' '
        jmp     putc

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

s_title:   .byte ATCLR, "BANKED RAM TEST", ATEOL
           .byte "RAM $xx00-$xx7F, COUNT $xxFB", ATEOL
           .byte "BANK $xxFD, SELECT $xxFF", ATEOL, ATEOL, 0
s_count:   .byte "COUNT    $", 0
s_bankreg: .byte "BANK REG ", 0
s_protect: .byte "PROTECT  ", 0
s_march:   .byte "MARCH    ", 0
s_limit:   .byte "LIMIT    ", 0
s_keep:    .byte "D1 KEEP     ", 0
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
