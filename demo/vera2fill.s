; vera2fill.s -- switch to 8bpp, fill the WHOLE screen fast with the blit
; (doubling copy from a 16-colour seed), wait for a key, return to BASIC.
; This is the assembly proof of the technique the BASIC example in vera_2.md uses.
;
; Build: ca65 --cpu 65C02 vera2fill.s -o vera2fill.o
;        ld65 -C vera2demo.cfg vera2fill.o -o VERA2FILL.PRG
; Run:   x16emu -bitmap2 -prg VERA2FILL.PRG -run

.setcpu "65C02"

BMP_CTRL   = $9F60
BMP_ID     = $9F61
BMP_ADDRL  = $9F62
BMP_ADDRM  = $9F63
BMP_ADDRH  = $9F64
BMP_DATA   = $9F65
BMP_PALADR = $9F66
BMP_PALLO  = $9F67
BMP_PALHI  = $9F68
BLIT_DSTL  = $9F69
BLIT_DSTM  = $9F6A
BLIT_DSTH  = $9F6B
BLIT_LENL  = $9F6C
BLIT_LENM  = $9F6D
BLIT_LENH  = $9F6E
BLIT_CTRL  = $9F6F

CHROUT = $FFD2
GETIN  = $FFE4

; one doubling copy: bytes [0,len) -> [dst,dst+len)  (blit src = ADDR = 0)
; NOTE: ADDR_H ($9F64) is {incr[3:0], ptr[19:16]} -- the upper nibble selects
; the DATA auto-increment stride, so a bank byte stored there must be masked
; to $0F.  Here the pointer is always 0, so a plain stz gives ptr 0 / stride +1.
.macro BLITC dst, len
    lda #<(dst)
    sta BLIT_DSTL
    lda #>(dst)
    sta BLIT_DSTM
    lda #^(dst)
    sta BLIT_DSTH
    lda #<(len)
    sta BLIT_LENL
    lda #>(len)
    sta BLIT_LENM
    lda #^(len)
    sta BLIT_LENH
    stz BMP_ADDRL
    stz BMP_ADDRM
    stz BMP_ADDRH
    lda #1
    sta BLIT_CTRL
    jsr wait_blit
.endmacro

.segment "LOADADDR"
    .word $0801
.segment "CODE"
    .word basic_next
    .word 10
    .byte $9E, "2061", $00
basic_next:
    .word 0

start:
    lda BMP_ID
    cmp #$B5
    beq have
    ldx #0
@nd:
    lda msg_nodev,x
    beq @ndx
    jsr CHROUT
    inx
    bne @nd
@ndx:
    rts

have:
    jsr pal16
    lda #$03                   ; enable 8bpp
    sta BMP_CTRL

    ; seed the first 16 pixels = colours 0..15
    stz BMP_ADDRL
    stz BMP_ADDRM
    stz BMP_ADDRH
    ldx #0
@seed:
    stx BMP_DATA
    inx
    cpx #16
    bne @seed

    ; fill the visible 307,200 bytes by doubling the seed (16 -> ... -> 307200)
    BLITC     16,     16
    BLITC     32,     32
    BLITC     64,     64
    BLITC    128,    128
    BLITC    256,    256
    BLITC    512,    512
    BLITC   1024,   1024
    BLITC   2048,   2048
    BLITC   4096,   4096
    BLITC   8192,   8192
    BLITC  16384,  16384
    BLITC  32768,  32768
    BLITC  65536,  65536
    BLITC 131072, 131072
    BLITC 262144,  45056       ; 262144 + 45056 = 307200

    ; drain the pending RUN <CR>, then wait for a real key
@flush:
    jsr GETIN
    bne @flush
@wk:
    jsr GETIN
    beq @wk
    stz BMP_CTRL               ; bitmap off -> BASIC screen returns
    rts

wait_blit:
    lda BLIT_CTRL
    and #1
    bne wait_blit
    rts

; 16-colour ramp: entry i -> R=i, G=15-i, B=i  (green -> magenta)
pal16:
    stz BMP_PALADR
    ldx #0
@l:
    txa
    eor #$0F                   ; 15 - i
    asl a
    asl a
    asl a
    asl a                      ; (15-i) << 4  = G nibble
    sta tmp
    txa
    ora tmp                    ; {G, B} = ((15-i)<<4) | i
    sta BMP_PALLO
    txa
    sta BMP_PALHI              ; R = i  (commit, cursor++)
    inx
    cpx #16
    bne @l
    rts

.segment "RODATA"
msg_nodev:
    .byte $93
    .byte "VERA_2 BITMAP LAYER NOT FOUND.", $0D
    .byte "LAUNCH THE EMULATOR WITH -BITMAP2", $0D, $00

.segment "BSS"
tmp: .res 1
