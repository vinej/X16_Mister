; vera2blit.s -- VERA_2 bitmap: blit SAVE-UNDER demo.
;
;   * 8bpp 640x480 gradient + random VERA sprites + KERNAL mouse (passthru).
;   * LEFT-CLICK on the gradient  -> save-under (blit the covered band to an
;     off-screen scratch region) then draw a "message" box over it.
;   * LEFT-CLICK on the message   -> restore (blit the band back), box vanishes
;     and the ORIGINAL gradient underneath reappears exactly -- proving the
;     SDRAM->SDRAM blit save-under works.
;   * Any key exits (restoring first if a box is open).
;
; Build: ca65 --cpu 65C02 vera2blit.s -o vera2blit.o
;        ld65 -C vera2demo.cfg vera2blit.o -o VERA2BLIT.PRG
; Run:   x16emu -bitmap2 -prg VERA2BLIT.PRG -run

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

VERA_ADDR_L   = $9F20
VERA_ADDR_M   = $9F21
VERA_ADDR_H   = $9F22
VERA_DATA0    = $9F23
VERA_CTRL     = $9F25
VERA_DC_VIDEO = $9F29
SPRITES_EN    = $40
LAYERS_EN     = $30

CHROUT       = $FFD2
GETIN        = $FFE4
MOUSE_CONFIG = $FF68
MOUSE_GET    = $FF6B
MOUSE_SCAN   = $FF71

NSPR = 16
MZP  = $02                     ; MOUSE_GET buffer (r0/r1): X @ $02/03, Y @ $04/05

; save-under band (rows 200..279, full width) and its scratch copy
BAND_OFF = 128000              ; 200 * 640
BAND_LEN = 51200               ; 80  * 640
SCRATCH  = 307200              ; first off-screen byte (8bpp shows 0..307199)
BOX_FIRST = 128200             ; box top-left byte: 200*640 + 200
FILL_COL = 252
BORD_COL = 253

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
    stz dlg_open
    stz lprev
    jsr pal8
    jsr pal_extra              ; dialog colours (252..255)
    jsr fill8
    jsr make_sprite_img
    jsr seed_init
    jsr make_sprites
    jsr setup_video
    jsr show_mouse

    lda #$0B                   ; enable | 8bpp | passthru
    sta BMP_CTRL
@loop:
    wai
    jsr MOUSE_SCAN
    ldx #MZP
    jsr MOUSE_GET              ; $02/03=X, $04/05=Y, A=buttons
    and #1                     ; left button
    sta curr
    cmp lprev
    beq @nopress
    lda curr
    beq @nopress               ; release edge -> ignore
    jsr on_click               ; press edge
@nopress:
    lda curr
    sta lprev
    jsr GETIN
    beq @loop
    lda dlg_open               ; key -> restore (if open) then exit
    beq @noopen
    jsr restore_under
@noopen:
    stz BMP_CTRL
    jsr restore_video
    rts

; ---- left click handler ----
on_click:
    lda dlg_open
    bne @isopen
    jsr save_under             ; closed -> open: save band, draw box
    jsr draw_dialog
    lda #1
    sta dlg_open
    rts
@isopen:
    jsr in_box                 ; open -> click on the box restores it
    bcc @out
    jsr restore_under
    stz dlg_open
@out:
    rts

; ---- mouse (r0=X, r1=Y) inside the box [200,440) x [200,280) ? C=1 if inside
in_box:
    lda $02
    cmp #<200
    lda $03
    sbc #>200
    bcc @no                    ; X < 200
    lda $02
    cmp #<440
    lda $03
    sbc #>440
    bcs @no                    ; X >= 440
    lda $04
    cmp #<200
    lda $05
    sbc #>200
    bcc @no                    ; Y < 200
    lda $04
    cmp #<280
    lda $05
    sbc #>280
    bcs @no                    ; Y >= 280
    sec
    rts
@no:
    clc
    rts

; ---- blit helpers ----
; ADDR_H ($9F64) is {incr[3:0], ptr[19:16]}: the upper nibble is the DATA
; auto-increment stride select, so every bank byte stored there is masked to
; $0F -- leaving stride 0 = +1, the linear default these routines want.
save_under:                    ; band -> scratch
    lda #<BAND_OFF
    sta BMP_ADDRL
    lda #>BAND_OFF
    sta BMP_ADDRM
    lda #(^BAND_OFF) & $0F
    sta BMP_ADDRH
    lda #<SCRATCH
    sta BLIT_DSTL
    lda #>SCRATCH
    sta BLIT_DSTM
    lda #^SCRATCH
    sta BLIT_DSTH
    jmp blit_band
restore_under:                 ; scratch -> band
    lda #<SCRATCH
    sta BMP_ADDRL
    lda #>SCRATCH
    sta BMP_ADDRM
    lda #(^SCRATCH) & $0F
    sta BMP_ADDRH
    lda #<BAND_OFF
    sta BLIT_DSTL
    lda #>BAND_OFF
    sta BLIT_DSTM
    lda #^BAND_OFF
    sta BLIT_DSTH
    ; falls into blit_band
blit_band:
    lda #<BAND_LEN
    sta BLIT_LENL
    lda #>BAND_LEN
    sta BLIT_LENM
    lda #^BAND_LEN
    sta BLIT_LENH
    lda #1
    sta BLIT_CTRL              ; start
@wait:
    lda BLIT_CTRL
    and #1
    bne @wait                  ; busy
    rts

; ---- draw the message box (240x80) with a 2px border ----
draw_dialog:
    lda #<BOX_FIRST
    sta rp
    lda #>BOX_FIRST
    sta rp+1
    lda #^BOX_FIRST
    sta rp+2
    ldx #0
@row:
    lda rp
    sta BMP_ADDRL
    lda rp+1
    sta BMP_ADDRM
    lda rp+2
    and #$0F                   ; keep the stride nibble clear -> +1
    sta BMP_ADDRH
    cpx #2
    bcc @border
    cpx #78
    bcs @border
    lda #BORD_COL              ; interior: 2 border | 236 fill | 2 border
    sta BMP_DATA
    sta BMP_DATA
    ldy #236
    lda #FILL_COL
@fl:
    sta BMP_DATA
    dey
    bne @fl
    lda #BORD_COL
    sta BMP_DATA
    sta BMP_DATA
    bra @adv
@border:
    ldy #240
    lda #BORD_COL
@bl:
    sta BMP_DATA
    dey
    bne @bl
@adv:
    clc
    lda rp
    adc #<640
    sta rp
    lda rp+1
    adc #>640
    sta rp+1
    lda rp+2
    adc #0
    sta rp+2
    inx
    cpx #80
    bne @row
    rts

; ===== 8bpp gradient =====
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

; ===== write/read-back self-test (off-screen) -> A = mismatches =====
selftest:
    stz BMP_ADDRL
    stz BMP_ADDRM
    lda #$06                   ; ptr = $60000 (off-screen, clear of scratch)
    sta BMP_ADDRH
    ldx #0
@w:
    txa
    eor #$5A
    sta BMP_DATA
    inx
    bne @w
    stz BMP_ADDRL
    stz BMP_ADDRM
    lda #$06
    sta BMP_ADDRH
    stz mism
    ldx #0
@r:
    lda BMP_DATA
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

; ===== top status bar (colour A) =====
status_bar:
    sta col
    stz BMP_ADDRL
    stz BMP_ADDRM
    stz BMP_ADDRH
    lda #<10240
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

; ===== palettes =====
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

pal_extra:                     ; 252=fill(blue) 253=border(white) 254=grn 255=red
    lda #252
    sta BMP_PALADR
    lda #$4C                   ; 252: {G=4, B=12}
    sta BMP_PALLO
    lda #$02                   ; R=2 -> blue
    sta BMP_PALHI
    lda #$FF
    sta BMP_PALLO
    lda #$0F                   ; 253 -> white
    sta BMP_PALHI
    lda #$F0
    sta BMP_PALLO
    lda #$00                   ; 254 -> green
    sta BMP_PALHI
    lda #$00
    sta BMP_PALLO
    lda #$0F                   ; 255 -> red
    sta BMP_PALHI
    rts

; ===== 16x16 diamond sprite -> VRAM $10000 =====
make_sprite_img:
    stz VERA_CTRL
    stz VERA_ADDR_L
    stz VERA_ADDR_M
    lda #$11
    sta VERA_ADDR_H
    ldy #0
@row:
    ldx #0
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
    adc dxabs
    cmp #7
    bcc @insd
    lda #0
    bra @put
@insd:
    lda #1
@put:
    sta VERA_DATA0
    inx
    cpx #16
    bne @col
    iny
    cpy #16
    bne @row
    rts

make_sprites:
    ldx #1
@sp:
    txa
    asl a
    asl a
    asl a
    sta VERA_ADDR_L
    lda #$FC
    sta VERA_ADDR_M
    lda #$11
    sta VERA_ADDR_H
    stz VERA_DATA0
    lda #$88
    sta VERA_DATA0
    jsr rand8
    sta VERA_DATA0
    jsr rand8
    and #1
    sta VERA_DATA0
    jsr rand8
    sta ylo
    jsr rand8
    and #1
    sta yhi
    beq @ync
    lda ylo
    and #$CF
    sta ylo
@ync:
    lda ylo
    sta VERA_DATA0
    lda yhi
    sta VERA_DATA0
    lda #$0C
    sta VERA_DATA0
    txa
    and #$0F
    ora #$50
    sta VERA_DATA0
    inx
    cpx #NSPR+1
    bne @sp
    rts

rand8:
    lda seed
    asl a
    bcc @nf
    eor #$1D
@nf:
    sta seed
    rts

seed_init:
    lda $9F04
    ora #1
    sta seed
    rts

setup_video:
    stz VERA_CTRL
    lda #LAYERS_EN
    trb VERA_DC_VIDEO
    lda #SPRITES_EN
    tsb VERA_DC_VIDEO
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

show_mouse:
    stz VERA_CTRL
    lda #SPRITES_EN
    tsb VERA_DC_VIDEO
    lda #1
    ldx #80
    ldy #60
    jsr MOUSE_CONFIG
    rts

.segment "RODATA"
msg_nodev:
    .byte $93
    .byte "VERA_2 BITMAP LAYER NOT FOUND.", $0D
    .byte "LAUNCH THE EMULATOR WITH -BITMAP2", $0D, $00

.segment "BSS"
ycnt:    .res 2
xcnt:    .res 2
val:     .res 1
cnt:     .res 2
col:     .res 1
mism:    .res 1
tmp:     .res 1
dxabs:   .res 1
ylo:     .res 1
yhi:     .res 1
seed:    .res 1
dlg_open:.res 1
lprev:   .res 1
curr:    .res 1
rp:      .res 3
