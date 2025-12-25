    PROCESSOR   18F87J50
    #define __18F87J50    1

    #include <xc.inc>
    #include <proc/pic18f87j50.inc>

; =========================
; SD macros
; =========================
SD_CS_LOW   MACRO
    bcf     LATD, 3, A      ; RD3 = 0, select card
ENDM
SD_CS_HIGH  MACRO
    bsf     LATD, 3, A      ; RD3 = 1, deselect card
ENDM

; =========================
; Config bits (keep ONE set)
; =========================
    CONFIG  STVREN = ON
    CONFIG  XINST  = OFF
    CONFIG  WDTEN  = OFF
    CONFIG  PLLDIV = 2
    CONFIG  DEBUG  = ON
    CONFIG  CP0    = OFF
    CONFIG  CPUDIV = OSC1
    CONFIG  IESO   = OFF
    CONFIG  FCMEN  = OFF
    CONFIG  FOSC   = HS
    CONFIG  WDTPS  = 32768
    CONFIG  EASHFT = ON
    CONFIG  BW     = 16
    CONFIG  WAIT   = OFF
    CONFIG  MODE   = MM
    CONFIG  CCP2MX  = DEFAULT
    CONFIG  ECCPMX  = DEFAULT
    CONFIG  MSSPMSK = 1
    CONFIG  PMPMX   = DEFAULT

; =========================
; Event detection constants
; =========================
ST_IDLE      EQU 0
ST_INPULSE   EQU 1
ST_DEAD      EQU 2

; Use ONE set of thresholds. Keep ADC branch values (tune later).
SIGNAL_HI    EQU 0x10
RESET_HI     EQU 0x08
DEAD_SAMPLES EQU 20

LOG_REC_SIZE         EQU 8
LOG_RECS_PER_SECTOR  EQU (512 / LOG_REC_SIZE)   ; =64
  
; =========================
; Coincidence / ADC channel defs
; =========================
CH_TOP      EQU 0              ; top detector
CH_BOT      EQU 1              ; bottom detector

; ADC channels (PIC18F87J50 CHS codes)
; CHS=0b0010 -> AN2, CHS=0b0011 -> AN3
ADC_CHS_AN2 EQU 0x08           ; (CHS<<2) = (2<<2)=0x08
ADC_CHS_AN3 EQU 0x0C           ; (3<<2)=0x0C

COINC_WIN_SAMPLES EQU 2

; ============================================================
; Reset vector
; ============================================================
    PSECT   resetVec, class=CODE, abs
    ORG     0x0000
reset_vector:
    goto    start

; ============================================================
; Interrupt vector (single priority mode -> 0x0008)
; ============================================================
    PSECT   intVec, class=CODE, abs
    ORG     0x0008
irq_high:
    ; Save minimal context
    movff   WREG,   saveW
    movff   STATUS, saveSTATUS
    movff   BSR,    saveBSR

    ; ADC interrupt?
    btfss   PIR1, 6, A
    bra     irq_exit

    ; Clear ADIF
    bcf     PIR1, 6, A

    ; Read sample
    movf    ADRESH, W, A
    movwf   adc_hi, A

    ; Handle state machine (may append event)
    call    adc_sample_handler

    ; Start next conversion
    ; advance sample index (16-bit)
    incf    sampL, F, A
    btfsc   STATUS, 2, A
    incf    sampH, F, A

    ; toggle channel: 0->1 or 1->0
    movf    adc_ch, W, A
    xorlw   0x01
    movwf   adc_ch, A

    ; set ADC channel select bits in ADCON0
    ; keep ADON bit0 as-is, set CHS bits (5:2)
    movf    ADCON0, W, A
    andlw   0x03              ; keep bits1:0 (GO/DONE,ADON)
    movwf   ADCON0, A

    movf    adc_ch, W, A
    bz      set_an2
set_an3:
    movlw   ADC_CHS_AN3
    bra     set_chs_done
set_an2:
    movlw   ADC_CHS_AN2
set_chs_done:
    iorwf   ADCON0, F, A

    ; start next conversion
    bsf     ADCON0, 1, A


irq_exit:
    movff   saveBSR,    BSR
    movff   saveSTATUS, STATUS
    movff   saveW,      WREG
    retfie  0

; ============================================================
; Main code
; ============================================================
    PSECT   code, class=CODE
    ORG     0x0100

start:
    ; -----------------------------
    ; Single interrupt priority mode
    ; -----------------------------
    bcf     RCON, 7, A        ; IPEN = 0

    ; clear/disable interrupts
    clrf    INTCON, A
    clrf    PIR1, A
    clrf    PIE1, A
        ; init ADC mux state: start with TOP/AN2
    clrf    adc_ch, A

    ; sample index = 0
    clrf    sampL, A
    clrf    sampH, A

    ; init per-channel detection vars
    movlw   ST_IDLE
    movwf   state0, A
    movwf   state1, A
    clrf    peak0, A
    clrf    peak1, A
    clrf    dead0, A
    clrf    dead1, A
    clrf    evt0_valid, A
    clrf    evt1_valid, A

    ; -----------------------------
    ; GPIO directions
    ; -----------------------------
    ; ADC input RA2 / AN2
    bsf     TRISA, 2, A
    bsf     TRISA, 3, A

    ; SD: RC3=SCK out, RC5=SDO out, RC4=SDI in
    bcf     TRISC, 3, A
    bcf     TRISC, 5, A
    bsf     TRISC, 4, A

    ; SD CS = RD3 output, LED = RD4 output
    bcf     TRISD, 3, A
    bcf     TRISD, 4, A
    SD_CS_HIGH
    bcf     LATD, 4, A        ; LED off

    ; -----------------------------
    ; Timer0 free-running timestamp
    ; (used for timestampL/H sampled at event end)
    ; -----------------------------
    movlw   0x87              ; 16-bit, Fosc/4, prescale 1:256, TMR0ON=0
    movwf   T0CON, A
    bsf     T0CON, 7, A       ; TMR0ON=1

    ; -----------------------------
    ; Configure analog on AN2 only
    ; -----------------------------
    bsf     WDTCON, 4, A      ; ADSHR = 1 (ANCON access)
    movlw   0xFB              ; AN2 analog (bit2=0), others digital
    movwf   ANCON0, A
    movlw   0xFF
    movwf   ANCON1, A
    bcf     WDTCON, 4, A      ; ADSHR = 0

    ; -----------------------------
    ; ADC setup
    ; -----------------------------
    movlw   0xF3              ; AN2 & AN3 analog (bits2,3=0), others digital
    movwf   ANCON0, A
    movlw   0x82
    movwf   ADCON1, A
    ; ADCON0: ADON=1, GO=0, CHS=AN2 initially (CHS=2 -> bits5:2 = 0x08)
    movlw   (ADC_CHS_AN2 | 0x01)   ; 0x08 | 0x01 = 0x09
    movwf   ADCON0, A


    ; init event detection vars
    movlw   ST_IDLE
    movwf   adc_state, A
    clrf    pulse_max, A
    clrf    dead_count, A
    clrf    led_flash, A

    ; -----------------------------
    ; Logging init
    ; -----------------------------
    clrf    log_index, A
    clrf    log_full, A
    clrf    drop_count, A

    ; pick a safe LBA start (your SD code uses 0x00001010)
    clrf    log_blk3, A
    clrf    log_blk2, A
    movlw   0x10
    movwf   log_blk1, A
    movlw   0x10
    movwf   log_blk0, A

    ; -----------------------------
    ; SPI + SD init
    ; -----------------------------
    call    spi_init
    call    sd_init

    ; if sd_r1 != 0 -> blink error
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    bra     sd_fail_other
    
    ; -----------------------------
    ; Enable ADC interrupt + global interrupts
    ; -----------------------------
    bsf     PIE1, 6, A        ; ADIE
    bsf     INTCON, 6, A      ; PEIE
    bsf     INTCON, 7, A      ; GIE

    ; start first conversion
    bsf     ADCON0, 1, A

main_loop:
    ; If a sector is full, flush it to SD
    movf    log_full, F, A
    btfsc   STATUS, 2, A          ; Z=1 -> log_full==0 (not full)
    bra     main_loop

    ; flush one sector
    call    main_flush_sector

    ; reset flags/counters for next sector
    clrf    log_index, A
    clrf    log_full, A
    clrf    drop_count, A

    bra     main_loop

; ============================================================
; adc_sample_handler_dual
;  Uses:
;    adc_hi = current sample (ADRESH)
;    adc_ch = 0(top/AN2) or 1(bot/AN3)
;    sampL/H = global sample index
; ============================================================
adc_sample_handler:
    ; select pointers/vars based on adc_ch
    movf    adc_ch, W, A
    bz      use_ch0

; ---------- channel 1 (bottom) ----------
use_ch1:
    ; state in state1, peak in peak1, dead in dead1
    movf    state1, W, A
    xorlw   ST_IDLE
    btfsc   STATUS, 2, A
    bra     ch1_idle

    movf    state1, W, A
    xorlw   ST_INPULSE
    btfsc   STATUS, 2, A
    bra     ch1_inpulse

    bra     ch1_dead

ch1_idle:
    movlw   SIGNAL_HI
    subwf   adc_hi, W, A
    btfss   STATUS, 0, A
    return
    movlw   ST_INPULSE
    movwf   state1, A
    movf    adc_hi, W, A
    movwf   peak1, A
    return

ch1_inpulse:
    ; peak track
    movf    adc_hi, W, A
    subwf   peak1, W, A
    btfsc   STATUS, 0, A
    bra     ch1_check_end
    movf    adc_hi, W, A
    movwf   peak1, A

ch1_check_end:
    movlw   RESET_HI
    subwf   adc_hi, W, A
    btfsc   STATUS, 0, A
    return

    ; pulse end -> latch event for ch1
    movlw   ST_DEAD
    movwf   state1, A
    movlw   DEAD_SAMPLES
    movwf   dead1, A

    movlw   1
    movwf   evt1_valid, A
    movf    peak1, W, A
    movwf   evt1_peak, A
    movf    sampL, W, A
    movwf   evt1_sampL, A
    movf    sampH, W, A
    movwf   evt1_sampH, A

    ; check coincidence
    bra     check_coinc

ch1_dead:
    movf    dead1, F, A
    btfsc   STATUS, 2, A
    bra     ch1_dead_done
    decfsz  dead1, F, A
    return
ch1_dead_done:
    movlw   ST_IDLE
    movwf   state1, A
    return


; ---------- channel 0 (top) ----------
use_ch0:
    movf    state0, W, A
    xorlw   ST_IDLE
    btfsc   STATUS, 2, A
    bra     ch0_idle

    movf    state0, W, A
    xorlw   ST_INPULSE
    btfsc   STATUS, 2, A
    bra     ch0_inpulse

    bra     ch0_dead
    
ch0_dead:
    movf    dead0, F, A
    btfsc   STATUS, 2, A
    bra     ch0_dead_done
    decfsz  dead0, F, A
    return

ch0_dead_done:
    movlw   ST_IDLE
    movwf   state0, A
    return

ch0_idle:
    movlw   SIGNAL_HI
    subwf   adc_hi, W, A
    btfss   STATUS, 0, A
    return
    movlw   ST_INPULSE
    movwf   state0, A
    movf    adc_hi, W, A
    movwf   peak0, A
    return

ch0_inpulse:
    movf    adc_hi, W, A
    subwf   peak0, W, A
    btfsc   STATUS, 0, A
    bra     ch0_check_end
    movf    adc_hi, W, A
    movwf   peak0, A

ch0_check_end:
    movlw   RESET_HI
    subwf   adc_hi, W, A
    btfsc   STATUS, 0, A
    return

    ; pulse end -> latch event for ch0
    movlw   ST_DEAD
    movwf   state0, A
    movlw   DEAD_SAMPLES
    movwf   dead0, A

    movlw   1
    movwf   evt0_valid, A
    movf    peak0, W, A
    movwf   evt0_peak, A
    movf    sampL, W, A
    movwf   evt0_sampL, A
    movf    sampH, W, A
    movwf   evt0_sampH, A

    ; check coincidence
    ; fallthrough


; ---------- coincidence decision ----------
check_coinc:
    ; need both valid
    movf    evt0_valid, F, A
    btfsc   STATUS, 2, A
    return
    movf    evt1_valid, F, A
    btfsc   STATUS, 2, A
    return

    ; compute abs(evt0 - evt1) into diffH:diffL
    ; diff = |evt0_sampL - evt1_sampL|
    movf    evt0_sampL, W, A
    subwf   evt1_sampL, W, A      ; W = evt1L - evt0L
    btfsc   STATUS, 0, A          ; C=1 means no borrow => evt1>=evt0
    bra     got_diff
    ; else take two's complement to abs
    negf    WREG, A               ; W = -(evt1L-evt0L) = evt0L-evt1L
got_diff:
    ; now W = abs diff (low byte)
    sublw   COINC_WIN_SAMPLES     ; compare: COINC_WIN_SAMPLES - diff
    btfss   STATUS, 0, A          ; if borrow => diff > window
    bra     not_coinc

    ; --- coincident! ---
    call    log_append_event
    clrf    evt0_valid, A
    clrf    evt1_valid, A
    return

not_coinc:
    ; keep the most recent event only:
    ; if evt0_sampL > evt1_sampL -> clear evt1, else clear evt0
    movf    evt0_sampL, W, A
    subwf   evt1_sampL, W, A      ; W = evt1L - evt0L
    btfsc   STATUS, 0, A          ; C=1 => evt1>=evt0, so evt0 older
    bra     clear_evt0
    clrf    evt1_valid, A         ; evt1 older
    return
clear_evt0:
    clrf    evt0_valid, A
    return

; ============================================================
; ---- SD / SPI / logging routines ----
; ============================================================

; --- flush one full sector to SD ---
main_flush_sector:
    
    ; copy 512 bytes from log_buf -> sd_buf
    lfsr    0, log_buf
    lfsr    1, sd_buf
    movlw   low 512
    movwf   sd_cntL, A
    movlw   high 512
    movwf   sd_cntH, A
copy_loop:
    movf    POSTINC0, W, A
    movwf   POSTINC1, A
    decfsz  sd_cntL, F, A
    bra     copy_loop
    decfsz  sd_cntH, F, A
    bra     copy_loop

    ; set sd_arg3..0 from log_blk3..0
    movf    log_blk3, W, A
    movwf   sd_arg3, A
    movf    log_blk2, W, A
    movwf   sd_arg2, A
    movf    log_blk1, W, A
    movwf   sd_arg1, A
    movf    log_blk0, W, A
    movwf   sd_arg0, A

    call    sd_write_block

    ; if fail -> blink error
    movf    sd_r1, W, A
    xorlw   0x00
    btfss   STATUS, 2, A
    bra     sd_fail_other

    ; increment block number
    incf    log_blk0, F, A
    btfsc   STATUS, 2, A
    incf    log_blk1, F, A
    btfsc   STATUS, 2, A
    incf    log_blk2, F, A
    btfsc   STATUS, 2, A
    incf    log_blk3, F, A

    return
 

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
; log_append_event (call from ISR when an event completes)
; Inputs (example):
;   pulse_max already in pulse_max
;   timestampL/H already sampled into timestampL/H
; Uses:
;   log_index, log_full, log_buf
;------------------------------------------------
log_append_event:
    ; if log_full != 0 -> drop event quickly
    movf    log_full, F, A
    btfss   STATUS, 2, A
    bra     log_drop

    ; offset = log_index * 8
    movf    log_index, W, A
    mullw   8

    ; FSR0 = log_buf + offset
    movlw   low(log_buf)
    addwf   PRODL, W, A
    movwf   FSR0L, A
    movlw   high(log_buf)
    addwfc  PRODH, W, A
    movwf   FSR0H, A

      ; byte0 = peak_top
    movf    evt0_peak, W, A
    movwf   POSTINC0, A

    ; byte1 = peak_bottom
    movf    evt1_peak, W, A
    movwf   POSTINC0, A

    ; byte2-3 = top sample index
    movf    evt0_sampL, W, A
    movwf   POSTINC0, A
    movf    evt0_sampH, W, A
    movwf   POSTINC0, A

    ; byte4-5 = bottom sample index
    movf    evt1_sampL, W, A
    movwf   POSTINC0, A
    movf    evt1_sampH, W, A
    movwf   POSTINC0, A

    ; byte6 = flags
    movlw   0xC0
    movwf   POSTINC0, A

    ; byte7 = marker
    movlw   0xA5
    movwf   POSTINC0, A

    ; -----------------------------------------------

    ; log_index++
    incf    log_index, F, A

    ; if log_index == 64 -> log_full = 1
    movf    log_index, W, A
    xorlw   LOG_RECS_PER_SECTOR
    btfss   STATUS, 2, A
    return

    movlw   1
    movwf   log_full, A
    return

log_drop:
    incf    drop_count, F, A
    return

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
; ============================================================
; Variables (ONE udata_acs + your BIGRAM buffers)
; ============================================================
    GLOBAL  sd_r1

    PSECT   udata_acs
; --- ISR save
saveW:          ds 1
saveSTATUS:     ds 1
saveBSR:        ds 1

; --- ADC vars
adc_hi:         ds 1
adc_state:      ds 1
pulse_max:      ds 1
dead_count:     ds 1
led_flash:      ds 1

; --- logging vars
log_index:      ds 1
log_blk0:       ds 1
log_blk1:       ds 1
log_blk2:       ds 1
log_blk3:       ds 1
log_full:       ds 1
drop_count:     ds 1
timestampL:     ds 1
timestampH:     ds 1

; --- SD vars
sd_tmp:         ds 1
sd_r1:          ds 1
sd_retry:       ds 1
err_cnt:        ds 1
delay1:         ds 1
delay2:         ds 1

sd_arg0:        ds 1
sd_arg1:        ds 1
sd_arg2:        ds 1
sd_arg3:        ds 1
sd_crc:         ds 1
sd_cntL:        ds 1
sd_cntH:        ds 1
sd_token:       ds 1
    
; --- ADC mux / sample index
adc_ch:         ds 1          ; 0=top(AN2), 1=bot(AN3)
sampL:          ds 1          ; 16-bit sample index
sampH:          ds 1

; --- per-channel states
state0:         ds 1
state1:         ds 1
peak0:          ds 1
peak1:          ds 1
dead0:          ds 1
dead1:          ds 1

; --- latched event end info (pending for coincidence)
evt0_valid:     ds 1
evt1_valid:     ds 1
evt0_peak:      ds 1
evt1_peak:      ds 1
evt0_sampL:     ds 1
evt0_sampH:     ds 1
evt1_sampL:     ds 1
evt1_sampH:     ds 1

; --- scratch for abs diff
diffL:          ds 1
diffH:          ds 1

; --- log buffer in BIGRAM
    PSECT   log_buf_ram, class=BIGRAM, space=1, noexec
log_buf:        ds 512

; --- sd sector buffer in BIGRAM
    PSECT   sd_buf_ram, class=BIGRAM, space=1, noexec
sd_buf:         ds 512

    END     reset_vector
