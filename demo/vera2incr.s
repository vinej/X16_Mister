; vera2incr.s -- VERA_2 bitmap: DATA auto-increment STRIDE showcase.
;
;   * Self-tests the stride field first and prints a message if the machine is
;     running an OLD bitstream/emulator (where $9F64[7:4] was ignored).
;   * 15 full-height VERTICAL lines drawn with stride +640 -- one `sta` per
;     pixel instead of a 24-bit add and three pointer stores.
;   * A rectangle outline drawn by WALKING THE PERIMETER: the pointer is loaded
;     ONCE and each edge only changes the stride (+1, +640, -1, -640), reading
;     $9F64 back to keep ptr[19:16] intact.  Ends exactly where it started.
;   * Any key returns to BASIC.
;
; Build: ca65 --cpu 65C02 vera2incr.s -o vera2incr.o
;        ld65 -C vera2demo.cfg vera2incr.o -o VERA2INCR.PRG
; Run:   x16emu -bitmap2 -prg VERA2INCR.PRG -run

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

; stride selects -- ADDR_H[7:4] (see vera_2.md section 3.1)
INC_1      = $00               ; +1    linear streaming (reset default)
INC_HOLD   = $10               ;  0    pointer does not move
INC_640    = $B0               ; +640  8bpp: one pixel DOWN
INC_M1     = $C0               ; -1    reverse
INC_M640   = $F0               ; -640  8bpp: one pixel UP

CHROUT = $FFD2
GETIN  = $FFE4

; rectangle: (120,120), 200 x 120
RX0      = 120
RY0      = 120
RW       = 200
RH       = 120
RECT_OFF = RY0 * 640 + RX0
RCOL     = 15                  ; white

; load the 20-bit pointer AND the stride.  ADDR_H[3:0] is ptr[19:16], so the
; bank byte MUST be masked to $0F before the stride nibble is OR'd in.
.macro SETPTR pv, strd
    lda #<(pv)
    sta BMP_ADDRL
    lda #>(pv)
    sta BMP_ADDRM
    lda #((((^(pv)) & $0F) | (strd)))
    sta BMP_ADDRH
.endmacro

; declared before use so ca65 knows `sp` is zero page for `lda (sp),y`
.zeropage
sp:    .res 2                  ; print() string pointer

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
    ldx #<msg_nodev
    ldy #>msg_nodev
    jmp print

have:
    jsr stride_test
    beq @ok
    ldx #<msg_old              ; stride field not honoured -> old build
    ldy #>msg_old
    jmp print
@ok:
    jsr pal16
    lda #$03                   ; enable 8bpp
    sta BMP_CTRL
    jsr clear0
    jsr stripes
    jsr rect

    ; drain the pending RUN <CR>, then wait for a real key
@flush:
    jsr GETIN
    bne @flush
@wk:
    jsr GETIN
    beq @wk
    stz BMP_CTRL               ; bitmap off -> BASIC screen returns
    rts

; ---- print the NUL-terminated string at Y:X, then return to BASIC ----
print:
    stx sp
    sty sp+1
    ldy #0
@l:
    lda (sp),y
    beq @done
    jsr CHROUT
    iny
    bne @l
@done:
    rts

; ============================================================================
; Stride self-test -- A = 0 on pass, else the number of failures.
; Every check uses the ADDR_L/M/H READBACK, so it verifies the pointer itself
; rather than what landed in the framebuffer.  Runs before the screen is
; cleared, so the bytes it scribbles are wiped by clear0.
; ============================================================================
stride_test:
    stz fails

    ; +1 (default): four writes must leave the pointer four bytes on
    SETPTR $40000, INC_1
    stz BMP_DATA
    stz BMP_DATA
    stz BMP_DATA
    stz BMP_DATA
    lda BMP_ADDRL
    cmp #4
    beq @s1
    inc fails
@s1:
    ; hold: a write then a read must both leave the pointer put (4bpp RMW case)
    SETPTR $40000, INC_HOLD
    lda #$5A
    sta BMP_DATA
    lda BMP_DATA
    cmp #$5A
    beq @s2
    inc fails
@s2:
    lda BMP_ADDRL
    beq @s3
    inc fails
@s3:
    ; +640: one write steps exactly one row down
    SETPTR $40000, INC_640
    stz BMP_DATA
    lda BMP_ADDRL
    cmp #<640
    beq @s4
    inc fails
@s4:
    lda BMP_ADDRM
    cmp #>640
    beq @s5
    inc fails
@s5:
    ; -1 from 0 wraps to the top of the 1 MB space ($FFFFF)
    SETPTR 0, INC_M1
    stz BMP_DATA
    lda BMP_ADDRL
    cmp #$FF
    beq @s6
    inc fails
@s6:
    lda BMP_ADDRH
    and #$0F
    cmp #$0F
    beq @s7
    inc fails
@s7:
    lda fails
    rts

; ============================================================================
; Clear the visible 307,200 bytes to colour 0 -- the default +1 stride, which
; is already the fastest thing a CPU loop can do (1200 * 256 bytes).
; ============================================================================
clear0:
    SETPTR 0, INC_1
    lda #<1200
    sta cnt
    lda #>1200
    sta cnt+1
@outer:
    ldx #0
@inner:
    stz BMP_DATA
    inx
    bne @inner
    lda cnt                    ; 16-bit decrement
    bne @dl
    dec cnt+1
@dl:
    dec cnt
    lda cnt
    ora cnt+1
    bne @outer
    rts

; ============================================================================
; 15 vertical lines, x = 20, 60, ... 580, full 480-pixel height, colour = index.
; Row 0 means the offset is just x, so ptr[19:16] is 0 and ADDR_H = the stride.
; ============================================================================
stripes:
    lda #<20
    sta colx
    lda #>20
    sta colx+1
    ldy #1                     ; colour, and the line counter
@line:
    lda colx
    sta BMP_ADDRL
    lda colx+1
    sta BMP_ADDRM
    lda #INC_640               ; ptr[19:16] = 0, stride = +640
    sta BMP_ADDRH
    tya                        ; A = colour, held across both halves
    ldx #240
@a:
    sta BMP_DATA               ; ONE store per pixel, straight down the screen
    dex
    bne @a
    ldx #240
@b:
    sta BMP_DATA
    dex
    bne @b
    clc                        ; x += 40
    lda colx
    adc #40
    sta colx
    lda colx+1
    adc #0
    sta colx+1
    iny
    cpy #16
    bne @line
    rts

; ============================================================================
; Rectangle outline -- ONE pointer load, then four strides.  Each edge leaves
; the pointer on the next corner, so the walk closes on the starting pixel.
; ============================================================================
rect:
    SETPTR RECT_OFF, INC_1
    lda #RCOL
    ldx #RW
@top:                          ; -> right along the top edge
    sta BMP_DATA
    dex
    bne @top
    ldy #INC_640
    jsr set_incr
    ldx #RH
@right:                        ; -> down the right edge
    sta BMP_DATA
    dex
    bne @right
    ldy #INC_M1
    jsr set_incr
    ldx #RW
@bottom:                       ; -> left along the bottom edge
    sta BMP_DATA
    dex
    bne @bottom
    ldy #INC_M640
    jsr set_incr
    ldx #RH
@left:                         ; -> up the left edge, back to the start
    sta BMP_DATA
    dex
    bne @left
    rts

; Change ONLY the stride (Y = stride select), leaving the pointer where it is.
; ADDR_H is readable, so ptr[19:16] is read back and the new nibble merged in.
; A is preserved (it holds the drawing colour).
set_incr:
    pha
    lda BMP_ADDRH
    and #$0F
    sta tmp
    tya
    ora tmp
    sta BMP_ADDRH
    pla
    rts

; ============================================================================
; Palette: 0 = black background, 1..14 = colour ramp, 15 = white (the rectangle)
; ============================================================================
pal16:
    stz BMP_PALADR
    stz BMP_PALLO
    stz BMP_PALHI              ; entry 0 = black, cursor -> 1
    ldx #1
@l:
    txa
    eor #$0F                   ; 15 - i
    asl a
    asl a
    asl a
    asl a                      ; (15-i) << 4 = G nibble
    sta tmp
    txa
    ora tmp                    ; {G = 15-i, B = i}
    sta BMP_PALLO
    txa
    sta BMP_PALHI              ; R = i -> commit, cursor++
    inx
    cpx #15
    bne @l
    lda #15
    sta BMP_PALADR
    lda #$FF
    sta BMP_PALLO
    lda #$0F
    sta BMP_PALHI              ; entry 15 = white
    rts

.segment "RODATA"
msg_nodev:
    .byte $93
    .byte "VERA_2 BITMAP LAYER NOT FOUND.", $0D
    .byte "LAUNCH THE EMULATOR WITH -BITMAP2", $0D, $00
msg_old:
    .byte $93
    .byte "STRIDE SELF-TEST FAILED.", $0D
    .byte "$9F64 BIT 7-4 IS NOT HONOURED --", $0D
    .byte "THIS BITSTREAM/EMULATOR PREDATES THE", $0D
    .byte "AUTO-INCREMENT STRIDE. UPDATE IT.", $0D, $00

.segment "BSS"
tmp:   .res 1
fails: .res 1
cnt:   .res 2
colx:  .res 2
