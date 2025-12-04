    PROCESSOR   18F87J50
    
  ; === Configuration for PIC18F87J50 clicker2 ===
    ; Tell XC8 which PIC18 device we are using
    #define __18F87J50    1

    ; Core assembler definitions
    #include <xc.inc>

    ;Device-specific SFR definitions for PIC18F87J50
     #include <proc/pic18f87j50.inc>

SD_CS_LOW   MACRO
    bcf     LATD, 3, A      ; RD3 = 0, select card
ENDM

SD_CS_HIGH  MACRO
    bsf     LATD, 3, A      ; RD3 = 1, deselect card
ENDM


    ; CONFIG1L
    ;CONFIG  RETEN = ON
    ;CONFIG  INTOSCSEL = HIGH
    ;CONFIG  SOSCSEL = DIG
    CONFIG  STVREN = ON
    CONFIG  XINST  = OFF
    CONFIG  WDTEN  = OFF
    CONFIG  PLLDIV = 2
    CONFIG  DEBUG  = ON        ; OFF if just programming, not debugging

    ; CONFIG1H
    CONFIG  CP0    = OFF
    CONFIG  CPUDIV = OSC1      ; 8 MHz from crystal straight to CPU

    ; CONFIG2L
    CONFIG  IESO   = OFF
    CONFIG  FCMEN  = OFF
    CONFIG  FOSC   = HS        ; matches the clicker2 crystal

    ; CONFIG2H
    CONFIG  WDTPS  = 32768

    ; CONFIG3L
    CONFIG  EASHFT = ON
    CONFIG  BW     = 16
    CONFIG  WAIT   = OFF
    CONFIG  MODE   = MM

    ; CONFIG3H
    CONFIG  CCP2MX  = DEFAULT
    CONFIG  ECCPMX  = DEFAULT
    CONFIG  MSSPMSK = 1
    CONFIG  PMPMX   = DEFAULT
    
;==== Constants (thresholds in ADRESH units) ==============================

SIGNAL_HI   EQU 0x0D      ; ? 50 counts (from Arduino SIGNAL_THRESHOLD)
RESET_HI    EQU 0x07      ; ? 25 counts (from Arduino RESET_THRESHOLD)

;--------------------------------------
; Reset vector
;--------------------------------------
    PSECT   resetVec, class=CODE, abs
    ORG     0x0000
reset_vector:
    goto    start    ; on reset, CPU starts at 0x0000 and goes to start

;--------------------------------------
; Main code
;--------------------------------------
    PSECT   code, class=CODE
    ORG     0x0100   ; places main code at memory address 0x0100

start:
    ;-----------------------------
    ; Configure LED on RD4
    ;-----------------------------
    bcf     TRISD, 4, A        ; RD4 as output
    bcf     LATD, 4, A         ; LED off initially

    ;-----------------------------
    ; Initialise SD card
    ;-----------------------------
    call    spi_init
    call    sd_init

    ;-----------------------------
    ; Check sd_r1 (R1 response to CMD0)
    ;   Expected: 0x01 (idle state)
    ;-----------------------------
    movf    sd_r1, W, A        ; W = sd_r1
    xorlw   0x01               ; compare with 0x01
    btfsc   STATUS, 2, A       ; Z flag set if equal
    bra     sd_ok              ; if sd_r1 == 0x01 ? OK

    ; anything else = fail
    bra     sd_fail

    ;-----------------------------
    ; main muon loop here
    ;-----------------------------
main_loop:
    bra     main_loop          ; placeholder

    ;-----------------------------
    ; SPI init for microSD (MSSP1)
    ; Socket 1: SCK=RC3, SDO=RC5, SDI=RC4, CS=RD3
    ;-----------------------------
spi_init:
    ; Pin directions
    bcf     TRISC, 3, A        ; RC3/SCK  output
    bcf     TRISC, 5, A        ; RC5/SDO  output
    bsf     TRISC, 4, A        ; RC4/SDI  input

    bcf     TRISD, 3, A        ; RD3/CS   output
    SD_CS_HIGH                 ; CS high (deselect card)

    ; SPI mode 0, master, Fosc/64
    movlw   0x40               ; SSPSTAT: SMP=0, CKE=1
    movwf   SSP1STAT, A

    movlw   0x22               ; SSPCON1: SSPEN=1, CKP=0, Fosc/64
    movwf   SSP1CON1, A

    bcf     PIR1, 3, A         ; clear SSPIF
    return

;------------------------------------------------
; spi_xfer: full-duplex SPI byte transfer
;   Input:  W  = byte to send
;   Output: W  = byte received
;   Clobbers: SSPBUF, STATUS bits, etc.
;------------------------------------------------
spi_xfer:
    movwf   SSP1BUF, A          ; start transfer
spi_wait:
    btfss   PIR1, 3, A         ; SSPIF set when done?
    bra     spi_wait
    bcf     PIR1, 3, A
    movf    SSP1BUF, W, A       ; read received byte
    return

;------------------------------------------------
; sd_clock_80: send 80 clock cycles with CS high
;------------------------------------------------
sd_clock_80:
SD_CS_HIGH               ; deselect card
    movlw   0x0A
    movwf   sd_tmp, A        ; 10 bytes * 8 clocks = 80 clocks
clk80_loop:
    movlw   0xFF
    call    spi_xfer         ; send dummy, ignore W
    decfsz  sd_tmp, F, A
    bra     clk80_loop
    return
    
;------------------------------------------------
; sd_send_cmd0: send CMD0 (GO_IDLE_STATE)
;  - arg = 0x00000000
;  - CRC = 0x95 (required for CMD0)
;------------------------------------------------
sd_send_cmd0:
    movlw   0x40            ; 0x40 | 0 (CMD0)
    call    spi_xfer
    movlw   0x00            ; arg[31:24]
    call    spi_xfer
    movlw   0x00            ; arg[23:16]
    call    spi_xfer
    movlw   0x00            ; arg[15:8]
    call    spi_xfer
    movlw   0x00            ; arg[7:0]
    call    spi_xfer
    movlw   0x95            ; CRC for CMD0 (with end bit)
    call    spi_xfer
    return

;------------------------------------------------
; sd_init: basic SD card init
; 1) 80 clocks with CS high
; 2) CMD0, read R1 response into sd_r1
;------------------------------------------------
sd_init:
    ; 1) Power-up clocks
    call    sd_clock_80

    ; 2) Select card
SD_CS_LOW

    ; 3) Send CMD0
    call    sd_send_cmd0

    ; 4) Read R1 response (up to 8 bytes)
    movlw   0x08
    movwf   sd_retry, A

sd_wait_r1:
    movlw   0xFF
    call    spi_xfer        ; clock in response
    movwf   sd_r1, A

    ; If response != 0xFF, break
    movf    sd_r1, W, A
    xorlw   0xFF
    btfss   STATUS, 2, A
    bra     sd_got_r1

    ; still 0xFF, retry until sd_retry==0
    decfsz  sd_retry, F, A
    bra     sd_wait_r1

    ; Out of retries, fall through with sd_r1 still 0xFF
sd_got_r1:
    SD_CS_HIGH             ; deselect card again
    return

;------------------------------------------------
; sd_ok: slow blink = CMD0 success
;------------------------------------------------
sd_ok:
sd_ok_loop:
    bsf     LATD, 4, A         ; LED on
    call    delay_ms
    bcf     LATD, 4, A         ; LED off
    call    delay_ms
    bra     sd_ok_loop         ; loop forever


;------------------------------------------------
; sd_fail: fast blink = CMD0 failed / no card
;------------------------------------------------
sd_fail:
sd_fail_loop:
    bsf     LATD, 4, A
    call    delay_ms_short
    bcf     LATD, 4, A
    call    delay_ms_short
    bra     sd_fail_loop

;------------------------------------------------
; delay_ms: crude longer delay
;------------------------------------------------
delay_ms:
    movlw   0x40
    movwf   delay1, A
d1_loop:
    movlw   0xFF
    movwf   delay2, A
d2_loop:
    decfsz  delay2, F, A
    bra     d2_loop
    decfsz  delay1, F, A
    bra     d1_loop
    return

;------------------------------------------------
; delay_ms_short: crude shorter delay
;------------------------------------------------
delay_ms_short:
    movlw   0x10
    movwf   delay1, A
d1s_loop:
    movlw   0xFF
    movwf   delay2, A
d2s_loop:
    decfsz  delay2, F, A
    bra     d2s_loop
    decfsz  delay1, F, A
    bra     d1s_loop
    return

; variables:
    PSECT   udata_acs
sd_tmp:     ds  1
sd_r1:      ds  1
sd_retry:   ds  1
delay1:     ds  1
delay2:     ds  1

    END     reset_vector