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

SIGNAL_HI	    EQU 0x0D      ; ? 50 counts (from Arduino SIGNAL_THRESHOLD)
RESET_HI	    EQU 0x07      ; ? 25 counts (from Arduino RESET_THRESHOLD)
LOG_REC_SIZE        EQU 8
LOG_RECS_PER_SECTOR EQU (512 / LOG_REC_SIZE)    ; = 64

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
    
;--------------------------------------
; Back to CODE
;--------------------------------------
    PSECT   code, class=CODE    

start:
    
    bcf     TRISD, 4, A        ; RD4 as output
    bcf     LATD, 4, A         ; LED off

    ; Timer0 setup
    movlw   0x87
    movwf   T0CON, A
    bsf     T0CON, 7, A

    ; --- SD + FAT32 init ---
    call    spi_init
    call    sd_init

    ; check sd_init result
    movf    sd_r1, W, A
    xorlw   0xFF
    btfsc   STATUS, 2, A
    bra     sd_fail            ; no response

    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    bra     sd_fail_other      ; some R1 error

    ; --- FAT32 mount (read MBR + boot sector) ---
    call    fat32_mount
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    bra     sd_fail_other      ; fat32 error code in sd_r1

    ; --- Start a new log session ---
    call    fat32_start_new_log  ; open/prepare file for logging
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    bra     sd_fail_other

    ; --- init logger state ---
    clrf    log_index, A

main_loop:
    ; fill log_buf with one 8-byte fake event
    call    make_fake_event

    ; check if 64*8 bytes are filled; if so, flush sector
    movf    log_index, W, A
    xorlw   LOG_RECS_PER_SECTOR ; 64
    btfss   STATUS, 2, A
    bra     main_loop           ; not full yet

    call    log_flush_sector    ; replace main_flush_sector
    bra     main_loop

main_write_error:
    ; for now just go to error blink
    bra     sd_fail_other

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

    movlw   0x01
    movwf   sd_r1, A           ;  write error
    return

;------------------------------------------------
; sd_test_write:
;   Fill sd_buf with pattern and write 1 block.
;   On return:
;       sd_r1 = 0x00 if write succeeded
;             = non-zero on error (from sd_write_block)
;------------------------------------------------
sd_test_write:
    ; 1) Fill sd_buf[512] with 0xAA
    lfsr    0, sd_buf

    movlw   low 512            ; 512 = 0x0200
    movwf   sd_cntL, A         ; = 0x00
    movlw   high 512
    movwf   sd_cntH, A         ; = 0x02

sd_fill_loop:
    movlw   0xAA               ; test pattern byte
    movwf   POSTINC0, A        ; store and FSR0++

    ; dec 16-bit counter
    decfsz  sd_cntL, F, A
    bra     sd_fill_loop
    decfsz  sd_cntH, F, A
    bra     sd_fill_loop
    ; now sd_buf is full of 0xAA

    ; 2) Choose a test block number (LBA) for SDHC/SDXC
    ;    Example: LBA = 0x00001000
    ;    arg = 0x00001000 ? [31:24]=0x00, [23:16]=0x00, [15:8]=0x10, [7:0]=0x00
    clrf    sd_arg3, A         ; 0x00
    clrf    sd_arg2, A         ; 0x00
    movlw   0x10
    movwf   sd_arg1, A
    clrf    sd_arg0, A

    ; 3) Call sd_write_block
    call    sd_write_block     ; sd_r1 = 0x00 on success (per our implementation)

    return

;------------------------------------------------
; sd_read_block:
;   Read one 512-byte block into sd_buf.
;   Input:
;       sd_arg3..sd_arg0 = block number (SDHC/SDXC, LBA)
;   Output:
;       sd_r1 = 0x00 on success (CMD17 R1)
;              non-zero on error
;------------------------------------------------
sd_read_block:
    ; 1) Select card
    SD_CS_LOW

    ; 2) CRC can be dummy for CMD17 once CRC disabled
    movlw   0xFF
    movwf   sd_crc, A

    ; 3) Send CMD17 (index 17)
    movlw   17                  ; CMD17 index
    call    sd_send_cmd         ; R1 -> sd_r1

    ; 4) Check R1 (must be 0x00)
    movf    sd_r1, W, A
    xorlw   0x00
    btfsc   STATUS, 2, A        ; Z=1 if R1 == 0
    bra     sd_read_cmd_ok

    ; command rejected or card not ready
    bra     sd_read_block_fail

sd_read_cmd_ok:
    ; 5) Wait for data token 0xFE
    movlw   0xFF
    movwf   sd_retry, A         ; retry counter

sd_wait_token:
    movlw   0xFF
    call    spi_xfer            ; read a byte
    movwf   sd_tmp, A

    movf    sd_tmp, W, A
    xorlw   0xFE                ; token?
    btfsc   STATUS, 2, A
    bra     sd_got_token

    ; not 0xFE; if 0xFF card may be busy; loop with timeout
    decfsz  sd_retry, F, A
    bra     sd_wait_token

    ; token timeout
    bra     sd_read_block_fail

sd_got_token:
    ; 6) Read 512 data bytes into sd_buf
    lfsr    0, sd_buf

    ; set 16-bit count = 512 (0x0200)
    movlw   low 512            ; 0x00
    movwf   sd_cntL, A
    movlw   high 512           ; 0x02
    movwf   sd_cntH, A

sd_rd_loop:
    movlw   0xFF
    call    spi_xfer           ; get data byte into W
    movwf   POSTINC0, A        ; store into *FSR0, FSR0++

    ; dec 16-bit counter
    decfsz  sd_cntL, F, A
    bra     sd_rd_loop
    decfsz  sd_cntH, F, A
    bra     sd_rd_loop

    ; 7) Read 2-byte CRC and ignore
    movlw   0xFF
    call    spi_xfer
    movlw   0xFF
    call    spi_xfer

    ; 8) Deselect card, extra clocks
    SD_CS_HIGH
    movlw   0xFF
    call    spi_xfer

    ; success: sd_r1 already 0x00 from CMD17
    return

sd_read_block_fail:
    SD_CS_HIGH
    movlw   0xFF
    call    spi_xfer           ; extra clocks

    movlw   0x02               ; "generic read error"
    movwf   sd_r1, A
    return

;------------------------------------------------
; sd_test_verify:
;   Read back test block and verify all 0xAA.
;   Uses the same LBA as sd_test_write (0x00001000).
;   On return:
;       sd_r1 = 0x00 if data matches
;             = 0x03 on mismatch
;             = other non-zero if sd_read_block failed
;------------------------------------------------
sd_test_verify:
    ; 1) Set same test block number: LBA = 0x00001000
    clrf    sd_arg3, A         ; 0x00
    clrf    sd_arg2, A         ; 0x00
    movlw   0x10
    movwf   sd_arg1, A
    clrf    sd_arg0, A

    ; 2) Read block into sd_buf
    call    sd_read_block      ; sd_r1 = 0x00 on success
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A       ; if not zero, read error
    return                     ; sd_r1 already holds error

    ; 3) Verify all 512 bytes == 0xAA
    lfsr    0, sd_buf

    movlw   low 512            ; 0x00
    movwf   sd_cntL, A
    movlw   high 512           ; 0x02
    movwf   sd_cntH, A

sd_ver_loop:
    movf    POSTINC0, W, A     ; W = *FSR0

    xorlw   0xAA               ; compare with 0xAA
    btfss   STATUS, 2, A       ; Z=1 if equal
    bra     sd_ver_fail

    ; dec 16-bit counter
    decfsz  sd_cntL, F, A
    bra     sd_ver_loop
    decfsz  sd_cntH, F, A
    bra     sd_ver_loop

    ; if we get here, all 512 bytes matched
    movlw   0x00
    movwf   sd_r1, A
    return

sd_ver_fail:
    movlw   0x03               ; "verify mismatch"
    movwf   sd_r1, A
    return
    
;------------------------------------------------
; make_fake_event:
;   - Builds one 8-byte log record in log_buf
;   - Uses:
;       log_index (0..63)
;       timestampL/timestampH (from TMR0L/H)
;   Record format (for now):
;       byte 0: log_index       (easy to recognise)
;       byte 1: 0xAA            (marker)
;       byte 2: timestampL
;       byte 3: timestampH
;       byte 4: 0x55            (marker)
;       byte 5: 0x00
;       byte 6: 0x00
;       byte 7: 0x00
;------------------------------------------------
make_fake_event:
    ; 1) Sample Timer0 into timestampL/H
    movf    TMR0L, W, A
    movwf   timestampL, A
    movf    TMR0H, W, A
    movwf   timestampH, A

    ; 2) Compute offset = log_index * LOG_REC_SIZE (8)
    movf    log_index, W, A
    mullw   LOG_REC_SIZE      ; W * 8 -> PRODL (L), PRODH (H)

    ; 3) FSR0 = log_buf + offset
    movlw   low(log_buf)
    addwf   PRODL, W, A       ; W = low(base) + offset L
    movwf   FSR0L, A

    movlw   high(log_buf)
    addwfc  PRODH, W, A       ; W = high(base) + carry + offset H
    movwf   FSR0H, A

    ; 4) Write 8-byte record
    ; byte 0: log_index
    movf    log_index, W, A
    movwf   POSTINC0, A

    ; byte 1: 0xAA
    movlw   0xAA
    movwf   POSTINC0, A

    ; byte 2: timestampL
    movf    timestampL, W, A
    movwf   POSTINC0, A

    ; byte 3: timestampH
    movf    timestampH, W, A
    movwf   POSTINC0, A

    ; byte 4: 0x55
    movlw   0x55
    movwf   POSTINC0, A

    ; byte 5,6,7: zero for now
    clrf    POSTINC0, A
    clrf    POSTINC0, A
    clrf    POSTINC0, A

    ; 5) Increment log_index
    incf    log_index, F, A

    return    

fat32_mount:
    ; LBA = 0
    clrf    sd_arg3, A
    clrf    sd_arg2, A
    clrf    sd_arg1, A
    clrf    sd_arg0, A

    call    sd_read_block
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    return                  ; sd_r1 = read error

    ; check MBR signature 0x55AA at offset 510
    lfsr    0, sd_buf
    movlw   510 & 0xFF      ; 0xFE
    addwf   FSR0L, F, A
    movf    INDF0, W, A     ; sd_buf[510]
    xorlw   0x55
    btfss   STATUS, 2, A
    bra     fat_mbr_err
    incf    FSR0L, F, A
    movf    INDF0, W, A     ; sd_buf[511]
    xorlw   0xAA
    btfss   STATUS, 2, A
    bra     fat_mbr_err

    ; first partition entry at offset 0x1BE (446)
    lfsr    0, sd_buf
    movlw   0xBE
    addwf   FSR0L, F, A

    ; Skip status(1), chs_first(3), type(1), chs_last(3)
    movlw   8
    addwf   FSR0L, F, A

    ; now FSR0 points to starting LBA (4 bytes little-endian)
    movf    POSTINC0, W, A  ; LBA0
    movwf   fat_PartStart0, A
    movf    POSTINC0, W, A  ; LBA1
    movwf   fat_PartStart1, A
    movf    POSTINC0, W, A  ; LBA2
    movwf   fat_PartStart2, A
    movf    INDF0, W, A     ; LBA3
    movwf   fat_PartStart3, A

    ; now read FAT32 boot sector at PartStart LBA
    movf    fat_PartStart0, W, A
    movwf   sd_arg0, A
    movf    fat_PartStart1, W, A
    movwf   sd_arg1, A
    movf    fat_PartStart2, W, A
    movwf   sd_arg2, A
    movf    fat_PartStart3, W, A
    movwf   sd_arg3, A

    call    sd_read_block
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    return                  ; error reading boot sector

    ; parse FAT32 BPB in sd_buf

    ; BytesPerSec @ offset 0x0B (word, 0x0200)
    lfsr    0, sd_buf
    movlw   0x0B
    addwf   FSR0L, F, A
    movf    POSTINC0, W, A        ; low
    movwf   fat_BytsPerSec0, A
    movf    INDF0, W, A           ; high
    movwf   fat_BytsPerSec1, A
    xorlw   0x02                  ; expect 0x0200
    btfss   STATUS, 2, A
    bra     fat_bs_err

    ; SecPerClus @ 0x0D
    lfsr    0, sd_buf
    movlw   0x0D
    addwf   FSR0L, F, A
    movf    INDF0, W, A
    movwf   fat_SecPerClus, A

    ; compute log2(SecPerClus) -> fat_SecPerClusShift
    movf    fat_SecPerClus, W, A
    movwf   sd_tmp, A          ; tmp = SecPerClus
    clrf    fat_SecPerClusShift, A

fspc_loop:
    movf    sd_tmp, W, A
    xorlw   1
    btfsc   STATUS, 2, A       ; tmp == 1 ?
    bra     fspc_done

    rrncf   sd_tmp, F, A       ; tmp >>= 1
    incf    fat_SecPerClusShift, F, A
    bra     fspc_loop

fspc_done:
    ; RsvdSecCnt @ 0x0E (word)
    lfsr    0, sd_buf
    movlw   0x0E
    addwf   FSR0L, F, A
    movf    POSTINC0, W, A        ; low
    movwf   sd_cntL, A
    movf    INDF0, W, A           ; high
    movwf   sd_cntH, A            ; RsvdSecCnt in sd_cntH:L

    ; NumFATs @ 0x10 (byte)
    lfsr    0, sd_buf
    movlw   0x10
    addwf   FSR0L, F, A
    movf    INDF0, W, A
    movwf   sd_tmp, A             ; NumFATs

    ; FATSz32 @ 0x24 (dword)
    lfsr    0, sd_buf
    movlw   0x24
    addwf   FSR0L, F, A
    movf    POSTINC0, W, A        ; size0
    movwf   sd_arg0, A
    movf    POSTINC0, W, A        ; size1
    movwf   sd_arg1, A
    movf    POSTINC0, W, A        ; size2
    movwf   sd_arg2, A
    movf    INDF0, W, A           ; size3
    movwf   sd_arg3, A

    ; RootClus @ 0x2C (dword)
    lfsr    0, sd_buf
    movlw   0x2C
    addwf   FSR0L, F, A
    movf    POSTINC0, W, A
    movwf   fat_RootClus0, A
    movf    POSTINC0, W, A
    movwf   fat_RootClus1, A
    movf    POSTINC0, W, A
    movwf   fat_RootClus2, A
    movf    INDF0, W, A
    movwf   fat_RootClus3, A

    ; now compute FATStartLBA = PartStart + RsvdSecCnt
    ; using 32-bit add: fat_FATStart = PartStart + RsvdSecCnt
    movf    fat_PartStart0, W, A
    addwf   sd_cntL, W, A
    movwf   fat_FATStart0, A
    movf    fat_PartStart1, W, A
    addwfc  sd_cntH, W, A
    movwf   fat_FATStart1, A
    movf    fat_PartStart2, W, A
    addwfc  0x00, W, A
    movwf   fat_FATStart2, A
    movf    fat_PartStart3, W, A
    addwfc  0x00, W, A
    movwf   fat_FATStart3, A

    ; compute DataStartLBA:
    ;   FAT region: NumFATs * FATSz32
    ;   DataStart = PartStart + RsvdSecCnt + NumFATs*FATSz32
    ;   we already have PartStart + RsvdSecCnt = fat_FATStart
    ;   so do: DataStart = fat_FATStart + NumFATs*FATSz32

    ; For simplicity, assume NumFATs=2 or 1 but small, and do a loop:
    ; put FATSz32 in a temp 32-bit and add it NumFATs times

    ; init DataStart = fat_FATStart
    movf    fat_FATStart0, W, A
    movwf   fat_DataStart0, A
    movf    fat_FATStart1, W, A
    movwf   fat_DataStart1, A
    movf    fat_FATStart2, W, A
    movwf   fat_DataStart2, A
    movf    fat_FATStart3, W, A
    movwf   fat_DataStart3, A

fat_add_fats_loop:
    movf    sd_tmp, W, A
    bz      fat_done_add_fats

    ; DataStart += FATSz32
    movf    fat_DataStart0, W, A
    addwf   sd_arg0, W, A
    movwf   fat_DataStart0, A
    movf    fat_DataStart1, W, A
    addwfc  sd_arg1, W, A
    movwf   fat_DataStart1, A
    movf    fat_DataStart2, W, A
    addwfc  sd_arg2, W, A
    movwf   fat_DataStart2, A
    movf    fat_DataStart3, W, A
    addwfc  sd_arg3, W, A
    movwf   fat_DataStart3, A

    decf    sd_tmp, F, A
    bra     fat_add_fats_loop

fat_done_add_fats:
    movlw   0x00
    movwf   sd_r1, A
    return

fat_mbr_err:
    movlw   0xE0
    movwf   sd_r1, A
    return

fat_bs_err:
    movlw   0xE1
    movwf   sd_r1, A
    return
    
;------------------------------------------------
; fat_ClusterToLBA
;  Input:
;    fat_FileClus3..0 = cluster number
;  Output:
;    sd_arg3..0 = first LBA of that cluster
;  Uses:
;    sd_arg3..0, sd_tmp
;------------------------------------------------
fat_ClusterToLBA:
    ; sd_arg = fat_FileClus
    movf    fat_FileClus0, W, A
    movwf   sd_arg0, A
    movf    fat_FileClus1, W, A
    movwf   sd_arg1, A
    movf    fat_FileClus2, W, A
    movwf   sd_arg2, A
    movf    fat_FileClus3, W, A
    movwf   sd_arg3, A

    ; subtract 2 -> (cluster - 2)
    movlw   2
    subwf   sd_arg0, F, A
    btfss   STATUS, 0, A       ; if borrow, propagate
    decf    sd_arg1, F, A
    btfsc   STATUS, 0, A
    bra     fc_no_borrow2
    decf    sd_arg2, F, A
    btfsc   STATUS, 0, A
    bra     fc_no_borrow2
    decf    sd_arg3, F, A
fc_no_borrow2:

    ; multiply by SecPerClus via left shifts
    movf    fat_SecPerClusShift, W, A
    movwf   sd_tmp, A
fc_shift_loop:
    movf    sd_tmp, W, A
    bz      fc_shift_done

    rlncf   sd_arg0, F, A
    rlncf   sd_arg1, F, A
    rlncf   sd_arg2, F, A
    rlncf   sd_arg3, F, A

    decf    sd_tmp, F, A
    bra     fc_shift_loop
fc_shift_done:

    ; sd_arg = (cluster-2)*SecPerClus
    ; add DataStartLBA -> sd_arg = LBA
    movf    fat_DataStart0, W, A
    addwf   sd_arg0, F, A
    movf    fat_DataStart1, W, A
    addwfc  sd_arg1, F, A
    movf    fat_DataStart2, W, A
    addwfc  sd_arg2, F, A
    movf    fat_DataStart3, W, A
    addwfc  sd_arg3, F, A

    return
;------------------------------------------------
; fat32_start_new_log
;  Finds MUONLOG.BIN in root dir, clears its size,
;  sets up fat_FileClus* and fat_FileLBA*.
;  On success: sd_r1 = 0x00
;  On fail   : sd_r1 = 0xE2 (not found)
;              sd_r1 = 0xE3 (bad start cluster)
;------------------------------------------------
fat32_start_new_log:
    ; 1) Root directory cluster -> fat_FileClus
    movf    fat_RootClus0, W, A
    movwf   fat_FileClus0, A
    movf    fat_RootClus1, W, A
    movwf   fat_FileClus1, A
    movf    fat_RootClus2, W, A
    movwf   fat_FileClus2, A
    movf    fat_RootClus3, W, A
    movwf   fat_FileClus3, A

    ; 2) Get LBA of root dir cluster
    call    fat_ClusterToLBA        ; sd_arg = LBA of root cluster
    movf    sd_arg0, W, A
    movwf   fat_RootLBA0, A
    movf    sd_arg1, W, A
    movwf   fat_RootLBA1, A
    movf    sd_arg2, W, A
    movwf   fat_RootLBA2, A
    movf    sd_arg3, W, A
    movwf   fat_RootLBA3, A

    ; sector index within root cluster
    clrf    sd_cntL, A              ; sector index 0

; ================== scan sectors in root cluster ==================
root_sector_loop:
    ; LBA = RootLBA + sector_index
    movf    fat_RootLBA0, W, A
    addwf   sd_cntL, W, A
    movwf   sd_arg0, A

    movf    fat_RootLBA1, W, A
    addwfc  0x00, W, A
    movwf   sd_arg1, A

    movf    fat_RootLBA2, W, A
    addwfc  0x00, W, A
    movwf   sd_arg2, A

    movf    fat_RootLBA3, W, A
    addwfc  0x00, W, A
    movwf   sd_arg3, A

    call    sd_read_block
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    return                          ; read error in sd_r1

    ; ---- scan 16 entries in this 512-byte sector ----
    clrf    fat_DirIndex, A         ; 0..15

root_entry_loop:
    ; Compute byte offset = fat_DirIndex * 32
    movf    fat_DirIndex, W, A
    mullw   32                      ; PRODL = offset (0..0x1F0)
    lfsr    0, sd_buf
    movf    PRODL, W, A
    addwf   FSR0L, F, A

    ; first byte of entry
    movf    INDF0, W, A
    movwf   sd_tmp, A               ; save first byte

    ; 0x00 -> end of directory
    movf    sd_tmp, W, A
    bz      root_not_found

    ; 0xE5 -> deleted, skip
    movf    sd_tmp, W, A
    xorlw   0xE5
    btfsc   STATUS, 2, A
    bra     root_next_entry

    ; ATTR at offset +11: check for LFN (0x0F)
    lfsr    0, sd_buf
    movf    PRODL, W, A             ; go back to start of entry
    addwf   FSR0L, F, A
    movlw   11
    addwf   FSR0L, F, A
    movf    INDF0, W, A
    xorlw   0x0F
    btfsc   STATUS, 2, A
    bra     root_next_entry         ; LFN entry, skip

    ; ---------- Compare NAME[11] to "MUONLOG BIN" ----------
    ; pointer back to start of entry
    lfsr    0, sd_buf
    movf    PRODL, W, A
    addwf   FSR0L, F, A

    ; remember this offset for later (size/cluster fields)
    movf    PRODL, W, A
    movwf   fat_DirEntOfs, A

    ; expected: 'M','U','O','N','L','O','G',' ','B','I','N'

    movlw   'M'
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   'U'
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   'O'
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   'N'
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   'L'
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   'O'
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   'G'
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   ' '
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   'B'
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   'I'
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    movlw   'N'
    movwf   sd_tmp, A
    incf    FSR0L, F, A
    xorwf   INDF0, W, A
    btfss   STATUS, 2, A
    bra     root_next_entry

    ; ================== FOUND MUONLOG.BIN ==================
root_found:
    ; Save directory sector LBA
    movf    sd_arg0, W, A
    movwf   fat_DirEntLBA0, A
    movf    sd_arg1, W, A
    movwf   fat_DirEntLBA1, A
    movf    sd_arg2, W, A
    movwf   fat_DirEntLBA2, A
    movf    sd_arg3, W, A
    movwf   fat_DirEntLBA3, A

    ; ---- read FstClusHI at +20 ----
    lfsr    0, sd_buf
    movf    fat_DirEntOfs, W, A
    addwf   FSR0L, F, A
    movlw   20
    addwf   FSR0L, F, A

    movf    INDF0, W, A            ; high word low byte
    movwf   fat_FileClus2, A
    incf    FSR0L, F, A
    movf    INDF0, W, A
    movwf   fat_FileClus3, A

    ; ---- FstClusLO at +26 ----
    lfsr    0, sd_buf
    movf    fat_DirEntOfs, W, A
    addwf   FSR0L, F, A
    movlw   26
    addwf   FSR0L, F, A

    movf    INDF0, W, A
    movwf   fat_FileClus0, A
    incf    FSR0L, F, A
    movf    INDF0, W, A
    movwf   fat_FileClus1, A

    ; fat_FileSize = 0
    clrf    fat_FileSize0, A
    clrf    fat_FileSize1, A
    clrf    fat_FileSize2, A
    clrf    fat_FileSize3, A

    ; Ensure file has a valid start cluster (non-zero)
    movf    fat_FileClus0, W, A
    iorwf   fat_FileClus1, W, A
    iorwf   fat_FileClus2, W, A
    iorwf   fat_FileClus3, W, A
    bz      root_bad_start_cluster

    ; ---- zero FileSize in directory entry at +28 ----
    lfsr    0, sd_buf
    movf    fat_DirEntOfs, W, A
    addwf   FSR0L, F, A
    movlw   28
    addwf   FSR0L, F, A

    clrf    INDF0, A           ; size[0]
    incf    FSR0L, F, A
    clrf    INDF0, A           ; size[1]
    incf    FSR0L, F, A
    clrf    INDF0, A           ; size[2]
    incf    FSR0L, F, A
    clrf    INDF0, A           ; size[3]

    ; ---- write directory sector back ----
    movf    fat_DirEntLBA0, W, A
    movwf   sd_arg0, A
    movf    fat_DirEntLBA1, W, A
    movwf   sd_arg1, A
    movf    fat_DirEntLBA2, W, A
    movwf   sd_arg2, A
    movf    fat_DirEntLBA3, W, A
    movwf   sd_arg3, A

    call    sd_write_block
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    return                      ; write error

    ; ---- compute starting LBA of file data ----
    call    fat_ClusterToLBA    ; sd_arg = LBA of first data cluster
    movf    sd_arg0, W, A
    movwf   fat_FileLBA0, A
    movf    sd_arg1, W, A
    movwf   fat_FileLBA1, A
    movf    sd_arg2, W, A
    movwf   fat_FileLBA2, A
    movf    sd_arg3, W, A
    movwf   fat_FileLBA3, A

    ; start at sector 0 inside this cluster
    clrf    fat_SectorInClus, A

    movlw   0x00
    movwf   sd_r1, A
    return

; ---------- continue scanning / error exits ----------

root_next_entry:
    ; increment entry index in this sector
    incf    fat_DirIndex, F, A
    movf    fat_DirIndex, W, A
    xorlw   16                  ; done 16 entries?
    btfsc   STATUS, 2, A
    bra     root_next_sector    ; yes -> next sector
    bra     root_entry_loop     ; no  -> next entry

root_next_sector:
    ; move to next sector in root cluster
    incf    sd_cntL, F, A
    movf    sd_cntL, W, A
    xorwf   fat_SecPerClus, W, A
    btfsc   STATUS, 2, A
    bra     root_not_found      ; no more sectors in root cluster
    bra     root_sector_loop

root_bad_start_cluster:
    movlw   0xE3                ; "MUONLOG.BIN has no start cluster"
    movwf   sd_r1, A
    return

root_not_found:
    movlw   0xE2                ; "MUONLOG.BIN not found"
    movwf   sd_r1, A
    return
   
;------------------------------------------------
; fat32_write_dir_size
;  Writes fat_FileSize* into the MUONLOG.BIN
;  directory entry (previously located).
;------------------------------------------------
fat32_write_dir_size:
    ; read directory sector
    movf    fat_DirEntLBA0, W, A
    movwf   sd_arg0, A
    movf    fat_DirEntLBA1, W, A
    movwf   sd_arg1, A
    movf    fat_DirEntLBA2, W, A
    movwf   sd_arg2, A
    movf    fat_DirEntLBA3, W, A
    movwf   sd_arg3, A

    call    sd_read_block
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    return                      ; read error

    ; pointer = sd_buf + fat_DirEntOfs + 28
    lfsr    0, sd_buf
    movf    fat_DirEntOfs, W, A
    addwf   FSR0L, F, A
    movlw   28
    addwf   FSR0L, F, A

    ; write 4-byte FileSize
    movf    fat_FileSize0, W, A
    movwf   INDF0, A
    incf    FSR0L, F, A
    movf    fat_FileSize1, W, A
    movwf   INDF0, A
    incf    FSR0L, F, A
    movf    fat_FileSize2, W, A
    movwf   INDF0, A
    incf    FSR0L, F, A
    movf    fat_FileSize3, W, A
    movwf   INDF0, A

    ; write directory sector back
    movf    fat_DirEntLBA0, W, A
    movwf   sd_arg0, A
    movf    fat_DirEntLBA1, W, A
    movwf   sd_arg1, A
    movf    fat_DirEntLBA2, W, A
    movwf   sd_arg2, A
    movf    fat_DirEntLBA3, W, A
    movwf   sd_arg3, A

    call    sd_write_block
    return
    
;------------------------------------------------
; log_flush_sector
;  - Copies log_buf[512] -> sd_buf
;  - Writes to current file LBA (fat_FileLBA*)
;  - Increments file LBA and file size
;  - Updates directory entry size
;------------------------------------------------
log_flush_sector:
    ; 1) copy 512 bytes log_buf -> sd_buf
    lfsr    0, log_buf
    lfsr    1, sd_buf

    movlw   low 512            ; 0x00
    movwf   sd_cntL, A
    movlw   high 512           ; 0x02
    movwf   sd_cntH, A

lf_copy_loop:
    movf    POSTINC0, W, A
    movwf   POSTINC1, A

    decfsz  sd_cntL, F, A
    bra     lf_copy_loop
    decfsz  sd_cntH, F, A
    bra     lf_copy_loop

    ; 2) write sd_buf to current file LBA
    movf    fat_FileLBA0, W, A
    movwf   sd_arg0, A
    movf    fat_FileLBA1, W, A
    movwf   sd_arg1, A
    movf    fat_FileLBA2, W, A
    movwf   sd_arg2, A
    movf    fat_FileLBA3, W, A
    movwf   sd_arg3, A

    call    sd_write_block
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    bra     log_flush_error

    ; 3) advance file LBA by 1
    movlw   1
    addwf   fat_FileLBA0, F, A
    clrf    sd_tmp, A
    addwfc  fat_FileLBA1, F, A
    addwfc  fat_FileLBA2, F, A
    addwfc  fat_FileLBA3, F, A
    
        ; 3b) advance sector index inside the cluster
    incf    fat_SectorInClus, F, A
    movf    fat_SectorInClus, W, A
    xorwf   fat_SecPerClus, W, A
    btfss   STATUS, 2, A
    bra     lf_not_cluster_end

    ; we reached the end of the first cluster:
    ; for now, stop with an error so we don't run into other data
    movlw   0xE4               ; "cluster full (demo limit)"
    movwf   sd_r1, A
    bra     log_flush_error

lf_not_cluster_end:

    ; 4) file_size += 512 (0x0200)
    movlw   low 512            ; 0x00
    addwf   fat_FileSize0, F, A
    movlw   high 512           ; 0x02
    addwfc  fat_FileSize1, F, A
    clrf    sd_tmp, A
    addwfc  fat_FileSize2, F, A
    addwfc  fat_FileSize3, F, A

    ; 5) write updated size into directory entry
    call    fat32_write_dir_size

    ; 6) reset log_index and blink LED
    clrf    log_index, A

    bsf     LATD, 4, A
    call    delay_ms_short
    bcf     LATD, 4, A

    return

log_flush_error:
    bra     sd_fail_other    
    
;------------------------------------------------
; delay_ms: crude longer delay
;------------------------------------------------
delay_ms:
    movlw   0x05
    movwf   delay1, A      ; outer loop
delay_outer:
    movlw   0xFF
    movwf   delay2, A
delay_mid:
    movlw   0xFF
    movwf   sd_tmp, A
delay_inner:
    decfsz  sd_tmp, F, A
    bra     delay_inner

    decfsz  delay2, F, A
    bra     delay_mid

    decfsz  delay1, F, A
    bra     delay_outer

    return

;------------------------------------------------
; delay_ms_short: crude shorter delay
;------------------------------------------------
delay_ms_short:
    movlw   0x08
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

;==================================================================
; FAST VARIABLES (ACCESS RAM)
;   - small, frequently used
;   - no bank select needed
;==================================================================
    GLOBAL  sd_r1          ; still export this if other files need it
    PSECT   udata_acs

sd_tmp:             ds 1
sd_r1:              ds 1
sd_retry:           ds 1

sd_cntL:            ds 1   ; 16-bit byte counter L
sd_cntH:            ds 1   ; 16-bit byte counter H
sd_token:           ds 1   ; data token / response

delay1:             ds 1
delay2:             ds 1
err_cnt:            ds 1

timestampL:         ds 1
timestampH:         ds 1

fat_SecPerClus:     ds 1   ; sectors per cluster
fat_SecPerClusShift:ds 1   ; log2(SecPerClus)
fat_SectorInClus:   ds 1   ; 0 .. (SecPerClus-1)

fat_DirEntOfs:      ds 1   ; offset of MUONLOG.BIN entry in sector

log_index:          ds 1   ; 0..64 (# of 8-byte records in log_buf)
fat_DirIndex:       ds 1     ; 0..15 index within sector


;==================================================================
; GENERAL VARIABLES (BANKED RAM ? normal data memory)
;==================================================================
    PSECT   appvars, class=RAM, space=1, noexec

; --- logging state ---
log_blk0:           ds 1   ; future use if you want block # again
log_blk1:           ds 1
log_blk2:           ds 1
log_blk3:           ds 1

; --- SD command args / CRC ---
sd_arg0:            ds 1   ; arg[7:0]
sd_arg1:            ds 1   ; arg[15:8]
sd_arg2:            ds 1   ; arg[23:16]
sd_arg3:            ds 1   ; arg[31:24]
sd_crc:             ds 1   ; CRC byte for command

;==================================================================
; FAT32 VOLUME LAYOUT
;==================================================================
fat_PartStart0:     ds 1   ; partition start LBA (little-endian)
fat_PartStart1:     ds 1
fat_PartStart2:     ds 1
fat_PartStart3:     ds 1

fat_FATStart0:      ds 1   ; FAT start LBA
fat_FATStart1:      ds 1
fat_FATStart2:      ds 1
fat_FATStart3:      ds 1

fat_DataStart0:     ds 1   ; first data cluster LBA (cluster 2)
fat_DataStart1:     ds 1
fat_DataStart2:     ds 1
fat_DataStart3:     ds 1

fat_RootClus0:      ds 1   ; root directory first cluster
fat_RootClus1:      ds 1
fat_RootClus2:      ds 1
fat_RootClus3:      ds 1

fat_BytsPerSec0:    ds 1   ; 512 = 0x0200
fat_BytsPerSec1:    ds 1

; current log file cluster
fat_FileClus0:      ds 1
fat_FileClus1:      ds 1
fat_FileClus2:      ds 1
fat_FileClus3:      ds 1

; current log file size (bytes)
fat_FileSize0:      ds 1   ; LSB
fat_FileSize1:      ds 1
fat_FileSize2:      ds 1
fat_FileSize3:      ds 1   ; MSB

; where MUONLOG.BIN directory entry lives
fat_DirEntLBA0:     ds 1
fat_DirEntLBA1:     ds 1
fat_DirEntLBA2:     ds 1
fat_DirEntLBA3:     ds 1

; base LBA of root dir cluster
fat_RootLBA0:       ds 1
fat_RootLBA1:       ds 1
fat_RootLBA2:       ds 1
fat_RootLBA3:       ds 1

; current file write position (absolute LBA)
fat_FileLBA0:       ds 1
fat_FileLBA1:       ds 1
fat_FileLBA2:       ds 1
fat_FileLBA3:       ds 1


;==================================================================
; BIG BUFFERS (LINEAR RAM / BIGRAM)
;   - keep each buffer in its own PSECT so they don't overlap
;==================================================================

    PSECT   log_buf_ram, class=BIGRAM, space=1, noexec
log_buf:            ds 512      ; your 8-byte records, 64 per sector

    PSECT   sd_buf_ram, class=BIGRAM, space=1, noexec
sd_buf:             ds 512      ; raw SD sector buffer


    END     reset_vector