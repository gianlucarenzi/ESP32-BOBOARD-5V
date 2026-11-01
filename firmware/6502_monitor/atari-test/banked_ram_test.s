;
; banked_ram_test.s -- Atari test program for the 6502_monitor banked RAM
;
; Tests both windows of the ESP32-BOBOARD-5V banked RAM peripheral:
;   $xx00-$xx7F  RAM window (128 bytes of the current bank)
;   $xxFD        bank register (R/W, 256 banks)
;   $xxFF        select latch (W, bit 7 = device ID $80)
; with xx = $D1 (PBI, ~D1XX) and $D5 (CCTL, ~CCTL).
;
; Per window:
;   1. BANK REG  write / read back the bank register, values $00-$FF
;   2. PROTECT   deselected (latch written with $7F: bit 7 = 0, other bits
;                set) the device must ignore RAM and bank register writes;
;                reselected with $81 (bit 7 = 1) the old values are back
;   3. MARCH     fill all 256 banks with  offset EOR bank EOR key,  verify;
;                then the complement, verify.  Catches stuck data bits,
;                offset / address aliasing and banks that alias each other
; Then:
;   4. KEEP      window $D1 again, after the $D5 tests and a deselect: its
;                last pattern must still be there (windows independent,
;                RAM kept while deselected)
;
; The first errors are printed as  ERR D1 B=bb O=oo E=ee G=gg  (bank,
; offset, expected, got).  At the end both windows are deselected.
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
BANK_REG   = $FD                ; bank register offset
LATCH      = $FF                ; select latch offset
DEV_ID     = $80                ; PBI device ID bit
MAX_REPORT = 4                  ; detailed error lines per test

; ---------------------------------------------------------------------------
        .segment "ZEROPAGE"

ptr:      .res 2                ; window base: $D100 / $D500
sptr:     .res 2                ; string pointer for print
bank:     .res 1                ; current bank in the loops
bk:       .res 1                ; bank EOR key (pattern of the current bank)
key:      .res 1                ; window key EOR pass inversion
wkey:     .res 1                ; window key
errs:     .res 3                ; total errors (24 bit)
terrs:    .res 3                ; errors at the start of the current test
reported: .res 1                ; detailed error lines printed in this test
expect:   .res 1
got:      .res 1
offs:     .res 1
tmp:      .res 1

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

        ; 4. KEEP: $D1 window still holds its complemented pattern.
        PRINT   s_keep
        jsr     test_begin
        lda     #$D1
        sta     ptr+1
        lda     #0
        sta     ptr
        sta     wkey
        jsr     select
        lda     #$FF
        eor     wkey
        sta     key
        jsr     verify_all
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

        ; 1. BANK REG
        jsr     print_win
        PRINT   s_bankreg
        jsr     test_begin
        jsr     select
        lda     #0
        sta     bank
@br_loop:
        lda     bank
        ldy     #BANK_REG
        sta     (ptr),y
        lda     (ptr),y
        cmp     bank
        beq     @br_ok
        sta     got
        lda     bank
        sta     expect
        lda     #BANK_REG
        sta     offs
        jsr     report
@br_ok:
        inc     bank
        bne     @br_loop
        jsr     test_end

        ; 2. PROTECT
        jsr     print_win
        PRINT   s_protect
        jsr     test_begin
        lda     #0                      ; bank 0, offset 0 = $11
        ldy     #BANK_REG
        sta     (ptr),y
        sta     bank
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
        ldy     #BANK_REG
        sta     (ptr),y
        lda     #DEV_ID | $01           ; select: bit 7 = 1
        ldy     #LATCH
        sta     (ptr),y
        ldy     #BANK_REG               ; bank still 0
        lda     (ptr),y
        beq     @pr_bank_ok
        sta     got
        lda     #0
        sta     expect
        lda     #BANK_REG
        sta     offs
        jsr     report
@pr_bank_ok:
        ldy     #BANK_REG               ; force bank 0 for the RAM check
        lda     #0
        sta     (ptr),y
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

        ; 3. MARCH: pattern, then complement.
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

        jsr     deselect
        rts

; ---------------------------------------------------------------------------
; fill_all / verify_all: every bank, every offset, value = offset EOR bank
; EOR key.
; ---------------------------------------------------------------------------
fill_all:
        lda     #0
        sta     bank
@bank:
        jsr     set_bank
        ldy     #RAM_SIZE - 1
@byte:
        tya
        eor     bk
        sta     (ptr),y
        dey
        bpl     @byte
        inc     bank
        bne     @bank
        rts

verify_all:
        lda     #0
        sta     bank
@bank:
        jsr     set_bank
        ldy     #RAM_SIZE - 1
@byte:
        tya
        eor     bk
        cmp     (ptr),y
        bne     @err
@next:
        dey
        bpl     @byte
        inc     bank
        bne     @bank
        rts
@err:
        sta     expect
        lda     (ptr),y
        sta     got
        sty     offs
        jsr     report
        ldy     offs
        jmp     @next

; set_bank: bank register = bank, bk = bank EOR key.
set_bank:
        lda     bank
        ldy     #BANK_REG
        sta     (ptr),y
        eor     key
        sta     bk
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
; the first MAX_REPORT.  Keeps bank, offs; clobbers A, X, Y.
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
           .byte "WINDOW $xx00-$xx7F, BANK $xxFD", ATEOL, ATEOL, 0
s_bankreg: .byte "BANK REG ", 0
s_protect: .byte "PROTECT  ", 0
s_march:   .byte "MARCH    ", 0
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
