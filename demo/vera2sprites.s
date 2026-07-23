; vera2sprites.s -- VERA_2 bitmap + sprites + mouse + write/read-back proof.
;
;   * 8bpp 640x480 diagonal gradient in the SDRAM bitmap layer.
;   * WRITE/READ-BACK SELF-TEST: writes a known pattern off-screen through
;     $9F65, reads it back through $9F65, compares.  Draws a full-width status
;     bar at the top -- GREEN = every byte matched (write + read-back proven),
;     RED = a mismatch.
;   * CTRL passthru ON: VERA hardware sprites composite OVER the bitmap --
;     16 randomly-placed diamonds + the KERNAL mouse pointer float on the
;     gradient (proving sprites work with the new mode).  VERA layers are
;     disabled so only sprites show through.
;   * Any key exits.
;
; Build: ca65 --cpu 65C02 vera2sprites.s -o vera2sprites.o
;        ld65 -C vera2demo.cfg vera2sprites.o -o VERA2SPRITES.PRG
; Run:   x16emu -bitmap2 -prg VERA2SPRITES.PRG -run

.setcpu "65C02"

; ---- VERA_2 bitmap registers ----
BMP_CTRL   = $9F60
BMP_ID     = $9F61
BMP_ADDRL  = $9F62
BMP_ADDRM  = $9F63
BMP_ADDRH  = $9F64
BMP_DATA   = $9F65
BMP_PALADR = $9F66
BMP_PALLO  = $9F67
BMP_PALHI  = $9F68

; ---- VERA ----
VERA_ADDR_L   = $9F20
VERA_ADDR_M   = $9F21
VERA_ADDR_H   = $9F22
VERA_DATA0    = $9F23
VERA_CTRL     = $9F25
VERA_DC_VIDEO = $9F29          ; DCSEL=0
SPRITES_EN    = $40            ; VERA_DC_VIDEO bit 6
LAYERS_EN     = $30            ; bits 5:4 = layer1|layer0 enable

; ---- KERNAL ----
CHROUT       = $FFD2
GETIN        = $FFE4
MOUSE_CONFIG = $FF68
MOUSE_SCAN   = $FF71

NSPR = 16                      ; sprites 1..16 (sprite 0 = mouse)

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
    ldx #0                     ; device absent -> message
@nd:
    lda msg_nodev,x
    beq @ndx
    jsr CHROUT
    inx
    bne @nd
@ndx:
    rts

have:
    jsr pal8                   ; 256-colour gradient palette
    jsr fill8                  ; draw the gradient (full-screen, including the top)

    jsr make_sprite_img        ; a 16x16 diamond in VRAM $10000
    jsr seed_init
    jsr make_sprites           ; NSPR sprites at random positions
    jsr setup_video            ; VERA layers off, sprite plane on
    jsr show_mouse             ; KERNAL mouse pointer (sprite 0)

    lda #$0B                   ; enable | mode 1 (8bpp) | passthru
    sta BMP_CTRL
@loop:
    wai                        ; wait for the 60 Hz vsync IRQ -> scan ONCE/frame
    jsr MOUSE_SCAN             ; (a tight loop re-applies the SMC delta = too fast)
    jsr GETIN
    beq @loop
    stz BMP_CTRL               ; a key -> bitmap off
    jsr restore_video          ; text layer back on, sprites + mouse off
    rts

; ===== 8bpp gradient: pixel = (x+y) & 255 =====
fill8:
    stz BMP_ADDRL
    stz BMP_ADDRM
    stz BMP_ADDRH
    stz ycnt
    stz ycnt+1
@row:
    lda ycnt
    sta val
    lda #$80
    sta xcnt
    lda #$02
    sta xcnt+1
@in:
    lda val
    sta BMP_DATA
    inc val
    lda xcnt
    bne @dl
    dec xcnt+1
@dl:
    dec xcnt
    lda xcnt
    ora xcnt+1
    bne @in
    inc ycnt
    bne @cy
    inc ycnt+1
@cy:
    lda ycnt+1
    cmp #>480
    bcc @row
    lda ycnt
    cmp #<480
    bcc @row
    rts

; ===== write/read-back self-test at off-screen ptr $50000 =====
; returns A = number of mismatched bytes (0 = pass)
selftest:
    stz BMP_ADDRL
    stz BMP_ADDRM
    lda #$05                   ; ptr = $50000 (past the 307200-byte visible area)
    sta BMP_ADDRH
    ldx #0
@w:
    txa
    eor #$5A
    sta BMP_DATA               ; write pattern byte, ptr++
    inx
    bne @w
    stz BMP_ADDRL
    stz BMP_ADDRM
    lda #$05
    sta BMP_ADDRH
    stz mism
    ldx #0
@r:
    lda BMP_DATA               ; read-back, ptr++
    sta tmp
    txa
    eor #$5A
    cmp tmp
    beq @ok
    inc mism
@ok:
    inx
    bne @r
    lda mism
    rts

; ===== full-width status bar (top 16 rows) in colour A =====
status_bar:
    sta col
    stz BMP_ADDRL
    stz BMP_ADDRM
    stz BMP_ADDRH              ; ptr = 0
    lda #<10240                ; 640 * 16
    sta cnt
    lda #>10240
    sta cnt+1
@l:
    lda col
    sta BMP_DATA
    lda cnt
    bne @dl
    dec cnt+1
@dl:
    dec cnt
    lda cnt
    ora cnt+1
    bne @l
    rts

; ===== gradient palette (R=G=hi nibble of index, B=lo nibble) =====
pal8:
    stz BMP_PALADR
    ldx #0
@l:
    txa
    sta BMP_PALLO
    txa
    lsr a
    lsr a
    lsr a
    lsr a
    sta BMP_PALHI
    inx
    bne @l
    rts

; ===== indicator colours: 254=green, 255=red =====
pal_indic:
    lda #254
    sta BMP_PALADR
    lda #$F0                   ; {G=15, B=0}
    sta BMP_PALLO
    lda #$00                   ; {R=0} -> green ; cursor -> 255
    sta BMP_PALHI
    lda #$00                   ; {G=0, B=0}
    sta BMP_PALLO
    lda #$0F                   ; {R=15} -> red
    sta BMP_PALHI
    rts

; ===== 16x16 diamond sprite image -> VRAM $10000 (index 1, 0=transparent) =====
make_sprite_img:
    stz VERA_CTRL              ; DCSEL=0, ADDRSEL=0
    stz VERA_ADDR_L
    stz VERA_ADDR_M
    lda #$11                   ; incr index 1 (bit4) | VRAM bit16 (bit0) -> $10000
    sta VERA_ADDR_H
    ldy #0                     ; py
@row:
    ldx #0                     ; px
@col:
    txa
    sec
    sbc #8
    bpl @dxp
    eor #$FF
    inc a
@dxp:
    sta dxabs
    tya
    sec
    sbc #8
    bpl @dyp
    eor #$FF
    inc a
@dyp:
    clc
    adc dxabs                  ; |px-8| + |py-8|
    cmp #7
    bcc @insd
    lda #0                     ; outside -> transparent
    bra @put
@insd:
    lda #1                     ; inside -> index 1
@put:
    sta VERA_DATA0
    inx
    cpx #16
    bne @col
    iny
    cpy #16
    bne @row
    rts

; ===== NSPR sprite attributes at $1FC00 + i*8, random X/Y, varied colour =====
make_sprites:
    ldx #1
@sp:
    txa                        ; VERA addr = $1FC00 + i*8
    asl a
    asl a
    asl a
    sta VERA_ADDR_L            ; ($FC00 low = 0) + i*8
    lda #$FC
    sta VERA_ADDR_M
    lda #$11                   ; VRAM bit16 | incr 1
    sta VERA_ADDR_H
    stz VERA_DATA0             ; b0: addr[12:5] = 0
    lda #$88                   ; b1: 8bpp | addr[16:13]=8  (image $10000)
    sta VERA_DATA0
    jsr rand8
    sta VERA_DATA0             ; b2: X[7:0]
    jsr rand8
    and #1
    sta VERA_DATA0             ; b3: X[9:8] (0..1 -> X 0..511)
    jsr rand8
    sta ylo
    jsr rand8
    and #1
    sta yhi
    beq @ync
    lda ylo                    ; keep Y < 464 when Y[8]=1
    and #$CF
    sta ylo
@ync:
    lda ylo
    sta VERA_DATA0             ; b4: Y[7:0]
    lda yhi
    sta VERA_DATA0             ; b5: Y[9:8]
    lda #$0C                   ; b6: Z-depth 3 (in front)
    sta VERA_DATA0
    txa                        ; b7: 16x16 | palette offset = i&15 (varied colour)
    and #$0F
    ora #$50
    sta VERA_DATA0
    inx
    cpx #NSPR+1
    bne @sp
    rts

; 8-bit maximal LFSR (period 255); A = next value, X/Y preserved
rand8:
    lda seed
    asl a
    bcc @nf
    eor #$1D
@nf:
    sta seed
    rts

seed_init:
    lda $9F04                  ; VIA1 T1 counter low -- varies at startup
    ora #1
    sta seed
    rts

; ===== VERA: layers off, sprite plane on =====
setup_video:
    stz VERA_CTRL
    lda #LAYERS_EN
    trb VERA_DC_VIDEO          ; layer0/1 off (only sprites over the bitmap)
    lda #SPRITES_EN
    tsb VERA_DC_VIDEO          ; sprite plane on
    rts

; ---- undo setup_video/show_mouse: return to the normal text screen ----
restore_video:
    stz VERA_CTRL
    lda #SPRITES_EN
    trb VERA_DC_VIDEO          ; sprite plane off (removes sprites + mouse)
    lda #$20                   ; layer 1 (text) back on
    tsb VERA_DC_VIDEO
    lda #0
    jsr MOUSE_CONFIG           ; stop the mouse driver
    rts

; ===== KERNAL mouse pointer (sprite 0), 640x480 field =====
show_mouse:
    stz VERA_CTRL
    lda #SPRITES_EN
    tsb VERA_DC_VIDEO
    lda #1                     ; default arrow
    ldx #80                    ; 80x60 eight-pixel cells = 640x480
    ldy #60
    jsr MOUSE_CONFIG
    rts

.segment "RODATA"
msg_nodev:
    .byte $93
    .byte "VERA_2 BITMAP LAYER NOT FOUND.", $0D
    .byte "LAUNCH THE EMULATOR WITH -BITMAP2", $0D, $00

.segment "BSS"
ycnt:  .res 2
xcnt:  .res 2
val:   .res 1
cnt:   .res 2
col:   .res 1
mism:  .res 1
tmp:   .res 1
dxabs: .res 1
ylo:   .res 1
yhi:   .res 1
seed:  .res 1
