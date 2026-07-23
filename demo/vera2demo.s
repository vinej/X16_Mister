; vera2demo.s -- VERA_2 SDRAM bitmap layer demo (640x480 8bpp + 4bpp).
; Auto-cycles: 8bpp diagonal 256-colour gradient  <->  4bpp 16-colour bands.
;
; Build: ca65 --cpu 65C02 vera2demo.s -o vera2demo.o
;        ld65 -C vera2demo.cfg vera2demo.o -o VERA2DEMO.PRG
; Run:   x16emu -bitmap2 -prg VERA2DEMO.PRG -run

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
CHROUT     = $FFD2
GETIN      = $FFE4

.segment "LOADADDR"
    .word $0801

.segment "CODE"
    ; ---- BASIC stub: 10 SYS 2061 ----
    .word basic_next
    .word 10
    .byte $9E, "2061", $00
basic_next:
    .word 0

; ---- entry (SYS 2061 = $080D) ----
start:
    lda BMP_ID
    cmp #$B5
    beq have_dev
    ldx #0                  ; device absent -> message, back to BASIC
@nd:
    lda msg_nodev,x
    beq @ndx
    jsr CHROUT
    inx
    bne @nd
@ndx:
    rts

have_dev:
    jsr print_title
@flush:                    ; drain the pending RUN <CR> so we don't exit at once
    jsr GETIN
    bne @flush
main_loop:
    ; --- 8bpp ---
    stz BMP_CTRL            ; layer off (title text shows while filling)
    jsr fill8
    jsr pal8
    lda #$03               ; enable, mode 1 (640x480x8bpp)
    sta BMP_CTRL
    jsr delay
    bne @quit              ; key -> back to BASIC
    ; --- 4bpp ---
    stz BMP_CTRL
    jsr fill4
    jsr pal4
    lda #$05               ; enable, mode 2 (640x480x4bpp)
    sta BMP_CTRL
    jsr delay
    bne @quit
    jmp main_loop
@quit:
    stz BMP_CTRL           ; bitmap off
    lda #$93
    jsr CHROUT             ; clear screen -> clean BASIC prompt
    rts

; ===== 8bpp framebuffer: diagonal (x+y) gradient over 256 colours =====
fill8:
    stz BMP_ADDRL
    stz BMP_ADDRM
    stz BMP_ADDRH
    stz ycnt
    stz ycnt+1
@row:
    lda ycnt               ; pixel value at x=0 is (y & 255)
    sta val
    lda #$80               ; 640 bytes per row
    sta xcnt
    lda #$02
    sta xcnt+1
@in:
    lda val
    sta BMP_DATA           ; DATA auto-increments the pointer
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

; ===== 4bpp framebuffer: 16-colour diagonal bands (2 px/byte) =====
fill4:
    stz BMP_ADDRL
    stz BMP_ADDRM
    stz BMP_ADDRH
    stz ycnt
    stz ycnt+1
@row:
    lda ycnt
    and #$0F
    sta color              ; band colour base = (y & 15)
    stz grp
    lda #$40               ; 320 bytes per row
    sta xcnt
    lda #$01
    sta xcnt+1
@in:
    lda color
    asl a
    asl a
    asl a
    asl a
    ora color              ; both nibbles = colour -> 2 px same colour
    sta BMP_DATA
    inc grp
    lda grp
    cmp #8                 ; new colour every 8 bytes (16 px band)
    bne @nx
    stz grp
    inc color
    lda color
    and #$0F
    sta color
@nx:
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

; ===== palettes (RGB444) =====
pal8:                       ; 256 colours: R=G=hi nibble of index, B=lo nibble
    stz BMP_PALADR
    ldx #0
@l:
    txa
    sta BMP_PALLO          ; {G = i>>4, B = i&15}
    txa
    lsr a
    lsr a
    lsr a
    lsr a
    sta BMP_PALHI          ; R = i>>4  (commit, cursor++)
    inx
    bne @l
    rts

pal4:                       ; 16 colours: R ramps up, G ramps down, B ramps up
    stz BMP_PALADR
    ldx #0
@l:
    stx tmpb               ; B = i
    lda #15
    sec
    sbc tmpb               ; G = 15 - i
    asl a
    asl a
    asl a
    asl a
    ora tmpb               ; {G, B}
    sta BMP_PALLO
    txa
    sta BMP_PALHI          ; R = i  (commit, cursor++)
    inx
    cpx #16
    bne @l
    rts

print_title:
    ldx #0
@l:
    lda msg_title,x
    beq @done
    jsr CHROUT
    inx
    bne @l
@done:
    rts

delay:                      ; ~2 s wait; abort early on a key.  Z=0 key / Z=1 timeout
    lda #48
    sta dcnt
@o:
    ldx #0
@x:
    ldy #0
@y:
    dey
    bne @y
    inx
    bne @x
    jsr GETIN
    bne @done              ; key pressed -> return (Z=0)
    dec dcnt
    bne @o
    lda #0                 ; timeout -> Z=1
@done:
    rts

.segment "RODATA"
msg_title:
    .byte $93                                     ; clear screen
    .byte "VERA_2 BITMAP LAYER DEMO", $0D, $0D
    .byte "640X480  8BPP (256 COL)", $0D
    .byte "         4BPP (16 COL)", $0D, $0D
    .byte "SDRAM LAYER AT $9F60-$9F6F", $0D, $0D
    .byte "AUTO-CYCLING...", $0D, $00
msg_nodev:
    .byte $93
    .byte "VERA_2 BITMAP LAYER NOT FOUND.", $0D
    .byte "LAUNCH THE EMULATOR WITH -BITMAP2", $0D, $00

.segment "BSS"
ycnt:  .res 2
xcnt:  .res 2
val:   .res 1
color: .res 1
grp:   .res 1
tmpb:  .res 1
dcnt:  .res 1
