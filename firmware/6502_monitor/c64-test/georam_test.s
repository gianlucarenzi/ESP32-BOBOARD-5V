;
; georam_test.s -- Commodore 64 test program for the 6502_monitor GeoRAM
;                  (PlatformIO env c64)
;
; GeoRAM map:
;   $DE00-$DEFF  256-byte window on the RAM (IO1)
;   $DFFE        page within the 16 KB block, bits 0-5 (W, IO2)
;   $DFFF        16 KB block (W, IO2)
; The registers are write-only: the program keeps a copy.
;
; Tests:
;   0. PRESENT   block 0 page 0 offset 1 keeps $A5 and $5A, else stop
;   1. SIZE      blocks 255..0 get their number at page 0 offset 0; the
;                first block that does not read back its own number is the
;                size (the block register is masked to the size, as on a
;                real GeoRAM).
;                EXPECT_BLOCKS (16 = 256 KB) is checked, 0 = any size.
;   2. ALIAS     block = size aliases block 0 (skipped with 256 blocks)
;   3. PAGE REG  64 pages of block 0 marked, read back with the page
;                register bits 6-7 set: only bits 0-5 count
;   4. MARCH     every page of every block filled and verified, then the
;                complement: byte 0 = page EOR key, byte 1 = block EOR key,
;                byte n = n EOR page EOR block EOR key (all 256 offsets,
;                $DEFF included).  Stuck data bits, address aliasing, pages
;                or blocks that alias each other
;
; Errors are counted on 24 bits; the first four of each test are printed as
;   ERR B=bb P=pp O=oo E=ee G=gg   (block, page, offset, expected, got).
; The test overwrites the whole GeoRAM.  Run time: about 30 s for 256 KB.
; It ends in an endless loop: RUN/STOP + RESTORE or reset to leave.
;
; Build:  cl65 -t c64 -C c64-asm.cfg -u __EXEHDR__ -o georam_test.prg georam_test.s
; LOAD"GEORAM_TEST",8 : RUN
; Strings are written in lower case: ca65 turns them into PETSCII that the
; C64 shows in upper case.
;

        .include "cbm_kernal.inc"

EXPECT_BLOCKS = 16              ; 256 KB; 0 = accept any size

GEO_WIN    = $DE00
GEO_PAGE   = $DFFE
GEO_BLOCK  = $DFFF
MAX_REPORT = 4                  ; detailed error lines per test
CR         = 13

; ---------------------------------------------------------------------------
        .segment "ZEROPAGE"

sptr:     .res 2                ; string pointer for print
nblk:     .res 1                ; blocks (0 = 256)
page:     .res 1                ; copy of $DFFE
block:    .res 1                ; copy of $DFFF
bk:       .res 1                ; page EOR block EOR key
key:      .res 1
errs:     .res 3                ; total errors (24 bit)
terrs:    .res 3                ; errors at the start of the current test
reported: .res 1
expect:   .res 1
got:      .res 1
offs:     .res 1
tmp:      .res 1
hi:       .res 1

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

        ; 0. Is there a GeoRAM?  Block 0 page 0 offset 1 holds $A5 and $5A.
        lda     #0
        jsr     set_page
        sta     block
        jsr     set_block
        lda     #$A5
        sta     GEO_WIN+1
        cmp     GEO_WIN+1
        bne     @none
        lda     #$5A
        sta     GEO_WIN+1
        cmp     GEO_WIN+1
        beq     @present
@none:
        PRINT   s_none
        jmp     halt

        ; 1. SIZE
@present:
        PRINT   s_size
        jsr     test_begin
        ldx     #$FF                    ; blocks 255..0: block 0 last
@sz_w:
        stx     GEO_BLOCK
        stx     GEO_WIN
        dex
        cpx     #$FF
        bne     @sz_w
        ldx     #0                      ; first block not reading itself
@sz_r:
        stx     GEO_BLOCK
        cpx     GEO_WIN
        bne     @sz_found
        inx
        bne     @sz_r
@sz_found:
        stx     nblk                    ; 0: all 256 blocks
        lda     #'$'
        jsr     putc
        lda     nblk
        jsr     phex
        PRINT   s_blocks
.if EXPECT_BLOCKS
        lda     nblk
        cmp     #<EXPECT_BLOCKS
        beq     @sz_ok
        sta     got
        lda     #<EXPECT_BLOCKS
        sta     expect
        lda     #0
        sta     offs
        sta     block
        sta     page
        jsr     report
@sz_ok:
.endif
        jsr     test_end

        ; 2. ALIAS: block nblk is block 0.
        lda     nblk
        beq     @page_reg
        PRINT   s_alias
        jsr     test_begin
        lda     #0
        jsr     set_page
        sta     block
        jsr     set_block
        lda     #$A5
        sta     GEO_WIN+2
        lda     nblk
        sta     block
        jsr     set_block
        lda     #$5A
        sta     GEO_WIN+2
        lda     #0
        sta     block
        jsr     set_block
        lda     GEO_WIN+2
        cmp     #$5A
        beq     @al_ok
        sta     got
        lda     #$5A
        sta     expect
        lda     #2
        sta     offs
        jsr     report
@al_ok:
        jsr     test_end

        ; 3. PAGE REG: bits 6-7 of the page register are ignored.
@page_reg:
        PRINT   s_pagereg
        jsr     test_begin
        lda     #0
        sta     block
        jsr     set_block
        ldx     #63
@pr_w:
        txa
        jsr     set_page
        eor     #$3C
        sta     GEO_WIN+3
        dex
        bpl     @pr_w
        lda     #$C0
        sta     hi
@pr_hi:
        ldx     #63
@pr_r:
        txa
        ora     hi
        jsr     set_page
        txa
        eor     #$3C
        cmp     GEO_WIN+3
        beq     @pr_next
        sta     expect
        lda     GEO_WIN+3
        sta     got
        lda     #3
        sta     offs
        txa
        pha
        jsr     report
        pla
        tax
@pr_next:
        dex
        bpl     @pr_r
        lda     hi
        sec
        sbc     #$40
        sta     hi
        bcs     @pr_hi                  ; $C0, $80, $40, $00
        jsr     test_end

        ; 4. MARCH: pattern, then complement.
        PRINT   s_march
        jsr     test_begin
        lda     #0
        sta     key
        jsr     fill_all
        jsr     verify_all
        lda     #$FF
        sta     key
        jsr     fill_all
        jsr     verify_all
        jsr     test_end

        ; Summary.
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
        lda     #CR
        jsr     putc
halt:
        jmp     halt

; ---------------------------------------------------------------------------
; Registers (write-only: copies in page / block).
; ---------------------------------------------------------------------------
; set_page: page = A, $DFFE = A.  Keeps A, X, Y.
set_page:
        sta     page
        sta     GEO_PAGE
        rts

; set_block: $DFFF = block.  Keeps A, X, Y.
set_block:
        pha
        lda     block
        sta     GEO_BLOCK
        pla
        rts

; ---------------------------------------------------------------------------
; Page patterns.  Block b, page p, key k:  byte 0 = p EOR k, byte 1 =
; b EOR k, byte n (2-$FF) = n EOR p EOR b EOR k.
; ---------------------------------------------------------------------------
; select: registers = block / page, bk = page EOR block EOR key.
select:
        jsr     set_block
        lda     page
        sta     GEO_PAGE
        eor     block
        eor     key
        sta     bk
        rts

fill_page:
        jsr     select
        ldy     #$FF
@byte:
        tya
        eor     bk
        sta     GEO_WIN,y
        dey
        cpy     #2
        bcs     @byte
        lda     block
        eor     key
        sta     GEO_WIN+1
        lda     page
        eor     key
        sta     GEO_WIN
        rts

verify_page:
        jsr     select
        ldy     #$FF
@byte:
        tya
        eor     bk
        cmp     GEO_WIN,y
        beq     @next
        jsr     vp_report
@next:
        dey
        cpy     #2
        bcs     @byte
        lda     block                   ; y = 1
        eor     key
        cmp     GEO_WIN,y
        beq     @byte0
        jsr     vp_report
@byte0:
        ldy     #0
        lda     page
        eor     key
        cmp     GEO_WIN
        beq     @done
        jsr     vp_report
@done:
        rts

; vp_report: A = expected, Y = offset; keeps Y.
vp_report:
        sta     expect
        lda     GEO_WIN,y
        sta     got
        sty     offs
        jsr     report
        ldy     offs
        rts

; fill_all / verify_all: every page of blocks 0 .. nblk-1.
fill_all:
        jsr     first_page
@loop:
        jsr     fill_page
        jsr     next_page
        bne     @loop
        rts

verify_all:
        jsr     first_page
@loop:
        jsr     verify_page
        jsr     next_page
        bne     @loop
        rts

first_page:
        lda     #0
        sta     page
        sta     block
        rts

; next_page: page + 1, then block + 1; Z = 1 after the last page.
next_page:
        inc     page
        lda     page
        cmp     #64
        bne     @more
        lda     #0
        sta     page
        inc     block
        lda     block
        cmp     nblk                    ; nblk 0 = 256: block wraps to 0
        rts
@more:
        lda     #1                      ; Z = 0
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
        lda     #CR
        jmp     putc

; report: one error at block / page / offs, expected / got.  Clobbers A, X,
; Y; keeps block, page, offs.
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
        bne     @line
        lda     #CR
        jsr     putc
@line:
        inc     reported
        PRINT   s_err
        lda     block
        jsr     phex
        PRINT   s_p
        lda     page
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
        lda     #CR
        jsr     putc
@done:
        rts

; ---------------------------------------------------------------------------
; Screen output through the KERNAL (CHROUT).
; ---------------------------------------------------------------------------
print:
        sta     sptr
        stx     sptr+1
        ldy     #0
@loop:
        lda     (sptr),y
        beq     @done
        jsr     putc
        iny
        bne     @loop
@done:
        rts

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
        adc     #6                      ; C = 1: +7 -> $41 'A' (PETSCII)
@dig:
        adc     #$30
        ; fall through

; putc: CHROUT keeps X and Y.
putc:
        jmp     CHROUT

; ---------------------------------------------------------------------------
        .segment "RODATA"

s_title:   .byte 147, "georam test", CR
           .byte "window $de00, page $dffe, block $dfff", CR, CR, 0
s_size:    .byte "size     ", 0
s_blocks:  .byte " blocks ", 0
s_none:    .byte "no georam: $de01 does not keep $a5 / $5a", CR, 0
s_alias:   .byte "alias    ", 0
s_pagereg: .byte "page reg ", 0
s_march:   .byte "march    ", 0
s_ok:      .byte "ok", CR, 0
s_failn:   .byte "fail $", 0
s_err:     .byte "err b=", 0
s_p:       .byte " p=", 0
s_o:       .byte " o=", 0
s_e:       .byte " e=", 0
s_g:       .byte " g=", 0
s_pass:    .byte CR, "all tests passed", CR, 0
s_fail1:   .byte CR, "errors: $", 0
