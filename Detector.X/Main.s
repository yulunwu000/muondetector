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
    xorlw   0xFF               ; compare with 0xFF
    btfsc   STATUS, 2, A       ; Z = 1 if sd_r1 == 0xFF
    bra     sd_fail            ; if no response, fast blink

    movf    sd_r1, W, A
    xorlw   0x00
    btfsc   STATUS, 2, A
    bra     sd_ok              ; OK path (slow blink)
    
    bra     sd_fail_other

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

    ; SPI mode , master, Fosc/64
    movlw   0x40      
    movwf   SSP1STAT, A

    movlw   0x22
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
; sd_send_cmd
;   Input:
;       W      = command index (e.g. 0, 8, 55, 41, 58)
;       sd_arg3..sd_arg0 = 32-bit argument (MSB..LSB)
;       sd_crc = CRC (only really needed for CMD0 & CMD8)
;   Output:
;       sd_r1  = R1 response
;   Uses:
;       sd_retry, sd_tmp, W
;   Note:
;       Assumes CS is already LOW (SD_CS_LOW done by caller)
;------------------------------------------------
sd_send_cmd:
    ; save command index from W
    movwf   sd_tmp, A          ; sd_tmp = cmd index
    ; 1 dummy byte before command
    movlw   0xFF
    call    spi_xfer

    ; send command byte: 0x40 | cmd
    movf    sd_tmp, W, A       ; restore cmd index
    iorlw   0x40        ; W = cmd | 0x40
    call    spi_xfer

    ; send 4 arg bytes (MSB first)
    movf    sd_arg3, W, A
    call    spi_xfer
    movf    sd_arg2, W, A
    call    spi_xfer
    movf    sd_arg1, W, A
    call    spi_xfer
    movf    sd_arg0, W, A
    call    spi_xfer

    ; send CRC
    movf    sd_crc, W, A
    call    spi_xfer

    ; now wait for R1 (non-0xFF) up to 8 bytes
    movlw   0x20
    movwf   sd_retry, A

sd_cmd_resp_loop:
    movlw   0xFF
    call    spi_xfer
    movwf   sd_r1, A           ; sd_r1 = response

    movf    sd_r1, W, A
    xorlw   0xFF
    btfsc   STATUS, 2, A       ; Z=1 if sd_r1 == 0xFF
    bra     sd_cmd_resp_next   ; still waiting

    return                     ; got a valid R1

sd_cmd_resp_next:
    decfsz  sd_retry, F, A
    bra     sd_cmd_resp_loop

    ; timeout, sd_r1 is still 0xFF
    return

;------------------------------------------------
; sd_init: SD/SDHC/SDXC init in SPI mode
;  1) 80 clocks with CS high
;  2) CMD0  (GO_IDLE_STATE) loop until R1=0x01
;  3) CMD8  (SEND_IF_COND)   for SDHC/SDXC
;  4) ACMD41 (CMD55 + CMD41 with HCS) until R1=0x00
;  5) CMD58 (READ_OCR) to clear OCR and confirm ready
;  Result:
;      sd_r1 = last R1 (should be 0x00 on success)
;------------------------------------------------
sd_init:
    ; 1) Power-up clocks (80 clocks, CS high)
    ; call    delay_ms
    call    sd_clock_80

    ; --- CMD0 loop: go idle ---
    movlw   0x32                ; up to 50 tries
    movwf   sd_retry, A

sd_init_cmd0_loop:
    SD_CS_LOW

    ; arg = 0x00000000
    clrf    sd_arg3, A
    clrf    sd_arg2, A
    clrf    sd_arg1, A
    clrf    sd_arg0, A

    ; CRC for CMD0 is 0x95 (with end bit)
    movlw   0x95
    movwf   sd_crc, A

    movlw   0x00                ; CMD0 index
    call    sd_send_cmd         ; result in sd_r1

    SD_CS_HIGH

    ; check for R1 = 0x01 (in idle state)
    movf    sd_r1, W, A
    xorlw   0x01
    btfsc   STATUS, 2, A
    bra     sd_after_cmd0       ; success

    ; else try again until sd_retry==0
    decfsz  sd_retry, F, A
    bra     sd_init_cmd0_loop

    ; CMD0 failed: leave sd_r1 as last response (likely 0xFF)
    return

sd_after_cmd0:
    ; --- CMD8: SEND_IF_COND (for SDHC/SDXC) ---
    SD_CS_LOW

    ; arg = 0x000001AA (2.7?3.6V, check pattern 0xAA)
    clrf    sd_arg3, A          ; 0x00
    clrf    sd_arg2, A          ; 0x00
    movlw   0x01
    movwf   sd_arg1, A
    movlw   0xAA
    movwf   sd_arg0, A

    ; CRC for CMD8 is 0x87
    movlw   0x87
    movwf   sd_crc, A

    movlw   0x08                ; CMD8 index
    call    sd_send_cmd         ; R1 in sd_r1

    ; read the rest of R7 (4 bytes) and ignore
    movlw   0x04
    movwf   sd_tmp, A
sd_read_r7:
    movlw   0xFF
    call    spi_xfer
    decfsz  sd_tmp, F, A
    bra     sd_read_r7

    SD_CS_HIGH

    ; --- ACMD41 loop: initialise (HCS set) ---
    movlw   0xFF                ; many tries
    movwf   sd_retry, A

sd_acmd41_loop:
    ; CMD55: APP_CMD
    SD_CS_LOW

    clrf    sd_arg3, A
    clrf    sd_arg2, A
    clrf    sd_arg1, A
    clrf    sd_arg0, A

    movlw   0xFF                ; dummy CRC (CRC off after CMD0/CMD8)
    movwf   sd_crc, A

    movlw   55                  ; CMD55 index
    call    sd_send_cmd         ; R1 in sd_r1

    SD_CS_HIGH

    ; CMD41: SD_SEND_OP_COND with HCS (bit30=1)
    SD_CS_LOW

    movlw   0x40                ; 0x40000000 (HCS)
    movwf   sd_arg3, A
    clrf    sd_arg2, A
    clrf    sd_arg1, A
    clrf    sd_arg0, A

    movlw   0xFF
    movwf   sd_crc, A

    movlw   41                  ; CMD41 index
    call    sd_send_cmd         ; R1 in sd_r1

    SD_CS_HIGH

    ; check if card is ready: R1 == 0x00
    movf    sd_r1, W, A
    xorlw   0x00
    btfsc   STATUS, 2, A
    bra     sd_acmd41_done      ; ready!

    ; else try again until retry == 0
    decfsz  sd_retry, F, A
    bra     sd_acmd41_loop

    ; ACMD41 timed out; sd_r1 holds last code
    return

sd_acmd41_done:
    ; --- CMD58: READ_OCR ---
    SD_CS_LOW

    clrf    sd_arg3, A
    clrf    sd_arg2, A
    clrf    sd_arg1, A
    clrf    sd_arg0, A

    movlw   0xFF
    movwf   sd_crc, A

    movlw   58                  ; CMD58 index
    call    sd_send_cmd         ; R1 in sd_r1 (should be 0x00)

    ; read OCR (4 bytes) and ignore or store if you like
    movlw   0x04
    movwf   sd_tmp, A
sd_read_ocr:
    movlw   0xFF
    call    spi_xfer
    decfsz  sd_tmp, F, A
    bra     sd_read_ocr

    SD_CS_HIGH

    ; If we got here, sd_r1 should be 0x00 (card ready)
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
; sd_fail_other:
;  Blink pattern encodes sd_r1 low nibble:
;  - Extract low 4 bits of sd_r1
;  - Blink that many times, then long pause, repeat
;------------------------------------------------
sd_fail_other:
    ; Get low nibble of sd_r1 into err_cnt
    movf    sd_r1, W, A
    andlw   0x0F              ; keep only low nibble
    bz      sd_fail_other_zero ; if 0, treat as 16 blinks to be visible
    movwf   err_cnt, A
    bra     sd_fail_other_loop_start

sd_fail_other_zero:
    movlw   0x10              ; 0x0 -> 16 blinks
    movwf   err_cnt, A

sd_fail_other_loop_start:
sd_fail_other_loop:
    ; copy err_cnt to sd_tmp as loop counter
    movf    err_cnt, W, A
    movwf   sd_tmp, A

sd_fail_other_blink_loop:
    bsf     LATD, 4, A        ; LED on
    call    delay_ms_short
    bcf     LATD, 4, A        ; LED off
    call    delay_ms_short
    decfsz  sd_tmp, F, A
    bra     sd_fail_other_blink_loop

    ; long pause between groups
    call    delay_ms
    call    delay_ms

    bra     sd_fail_other_loop

;------------------------------------------------
; sd_wait_not_busy
;   Polls MISO by sending 0xFF until card responds
;   with 0xFF (not busy) or timeout.
;   Output:
;       sd_tmp = last byte read
;   On timeout, sd_tmp != 0xFF.
;------------------------------------------------
sd_wait_not_busy:
    movlw   0xFF
    movwf   sd_retry, A       ; number of polls

sd_wait_nb_loop:
    movlw   0xFF
    call    spi_xfer          ; read response into W
    movwf   sd_tmp, A

    movf    sd_tmp, W, A
    xorlw   0xFF
    btfsc   STATUS, 2, A      ; Z=1 if 0xFF (not busy)
    return                    ; done

    decfsz  sd_retry, F, A
    bra     sd_wait_nb_loop

    ; timeout, still busy
    return
    
;------------------------------------------------
; sd_write_block:
;   Write one 512-byte block from sd_buf.
;   Input:
;       sd_arg3..sd_arg0 = block number (SDHC/SDXC)
;       sd_buf[512]      = data to write
;   Output:
;       sd_r1 = 0x00 on success (CMD24 R1)
;              non-zero on error
;------------------------------------------------
sd_write_block:
    ; 1) Select card
    SD_CS_LOW

    ; 2) CRC can be dummy for CMD24 once CRC disabled
    movlw   0xFF
    movwf   sd_crc, A

    ; 3) Send CMD24 (index 24)
    movlw   24                  ; CMD24 index
    call    sd_send_cmd         ; R1 -> sd_r1

    ; 4) Check R1 (must be 0x00)
    movf    sd_r1, W, A
    xorlw   0x00
    btfsc   STATUS, 2, A        ; Z=1 if R1 == 0
    bra     sd_write_cmd_ok

    ; command rejected or card not ready
    bra     sd_write_block_fail

sd_write_cmd_ok:
    ; 5) Send one dummy 0xFF before data token (optional but common)
    movlw   0xFF
    call    spi_xfer

    ; 6) Send data token 0xFE
    movlw   0xFE
    call    spi_xfer

    ; 7) Send 512 data bytes from sd_buf
    ;    Use FSR0 to walk the buffer
    lfsr    0, sd_buf

    ; set 16-bit count = 512 (0x0200)
    movlw   low 512            ; 0x00
    movwf   sd_cntL, A
    movlw   high 512           ; 0x02
    movwf   sd_cntH, A

sd_wr_loop:
    movf    POSTINC0, W, A     ; W = *FSR0; FSR0++
    call    spi_xfer           ; send data byte

    ; dec 16-bit counter
    decfsz  sd_cntL, F, A
    bra     sd_wr_loop
    decfsz  sd_cntH, F, A
    bra     sd_wr_loop

    ; 8) Send 2-byte CRC (dummy)
    movlw   0xFF
    call    spi_xfer
    movlw   0xFF
    call    spi_xfer

    ; 9) Read data response token
    movlw   0xFF
    call    spi_xfer
    movwf   sd_token, A

    ; lower 5 bits:
    movf    sd_token, W, A
    andlw   0x1F               ; mask to xxxxx
    xorlw   0x05               ; 0b00101 = "data accepted"
    btfss   STATUS, 2, A
    bra     sd_write_block_fail ; not accepted

    ; 10) Wait until card not busy
    call    sd_wait_not_busy
    movf    sd_tmp, W, A
    xorlw   0xFF
    btfss   STATUS, 2, A       ; Z=1 if sd_tmp == 0xFF (ready)
    bra     sd_write_block_fail

    ; 11) Deselect card, one extra 0xFF clocks
    SD_CS_HIGH
    movlw   0xFF
    call    spi_xfer

    ; indicate success: R1 already 0x00
    return

sd_write_block_fail:
    ; clean up chip select
    SD_CS_HIGH
    movlw   0xFF
    call    spi_xfer           ; extra clocks

    ; set a non-zero error code if you like
    movlw   0x01
    movwf   sd_r1, A           ; "generic write error"
    return

;------------------------------------------------
; delay_ms: crude longer delay
;------------------------------------------------
delay_ms:
    movlw   0xFF
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
    movlw   0x04
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
err_cnt:    ds  1

sd_arg0:    ds  1     ; arg[7:0]
sd_arg1:    ds  1     ; arg[15:8]
sd_arg2:    ds  1     ; arg[23:16]
sd_arg3:    ds  1     ; arg[31:24]
sd_crc:     ds  1     ; CRC byte for command

sd_cntL:    ds  1     ; 16-bit byte counter L
sd_cntH:    ds  1     ; 16-bit byte counter H
sd_token:   ds  1     ; data response / token

; --- big SD sector buffer in linear RAM ---
    PSECT   sd_buf_ram, class=BIGRAM, space=1, noexec
sd_buf:     ds  512

    END     reset_vector