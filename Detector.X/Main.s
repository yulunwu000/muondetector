    PROCESSOR   18F87J50
    
  ; === Configuration for PIC18F87J50 clicker2 ===
    ; Tell XC8 which PIC18 device we are using
    #define __18F87J50    1

    ; Core assembler definitions
    #include <xc.inc>

    ;Device-specific SFR definitions for PIC18F87J50
     #include <proc/pic18f87j50.inc>

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

;==== Event detection states =====================================
ST_IDLE     EQU 0      ; waiting for pulse
ST_INPULSE  EQU 1      ; currently in a pulse (tracking max)
ST_DEAD     EQU 2      ; in dead-time, ignore new pulses

;==== Thresholds in ADRESH units =================================
SIGNAL_HI   EQU 0x10  ; ~0.32 V
RESET_HI    EQU 0x08   ; ~0.16 V

;==== Dead-time in number of samples  ============================
DEAD_SAMPLES EQU 20    ; 20 samples 

;--------------------------------------
; Reset vector
;--------------------------------------
    PSECT   resetVec, class=CODE, abs
    ORG     0x0000
reset_vector:
    goto    start    ; on reset, CPU starts at 0x0000 and goes to start

;--------------------------------------
; Interrupt vector
;--------------------------------------
PSECT   intVec, class=CODE, abs
ORG     0x0008
irq_high:
    ; Save context
    movff   WREG,   saveW
    movff   STATUS, saveSTATUS
    movff   BSR,    saveBSR

    ; Check ADC interrupt flag
    btfss   PIR1, 6, A      ; ADC conversion complete?
    bra     irq_other       ; (handle other IRQs if any)

    ; Clear ADC interrupt flag
    bcf     PIR1, 6, A
    
    movf    ADRESH, W, A    ; W = ADRESH (top 8 bits of ADC)
    movwf   LATD, A         ; put it on PORTD LEDs

    ; Read ADC results
    movf    ADRESH, W, A
    movwf   adc_hi, A

    ; Call state-machine handler
     call    adc_sample_handler

    ; Start next conversion (continuous sampling)
    bsf     ADCON0, 1, A    ; GO/DONE = 1

    bra     irq_exit

irq_other:
    ; handle other interrupt sources here

irq_exit:
    ; Restore context
    movff   saveBSR,    BSR
    movff   saveSTATUS, STATUS
    movff   saveW,      WREG
    retfie  0
    
;--------------------------------------
; Main code
;--------------------------------------
    PSECT   code, class=CODE
    ORG     0x0100   ; places main code at memory address 0x0100

start:
    ;------------------------------------------------
    ; PORT setup
    ;------------------------------------------------
    bsf	    TRISA, 2, A            ; RA2 input
    clrf    TRISD, A           ; all PORTD as outputs
    bsf     TRISB, 2, A        ; RB2 as input (UVP / INT)
    bcf     LATD, 4, A         ; LD1 (RD4) off initially
    
    ;------------------------------------------------
    ; Force single interrupt priority mode, clean flags
    ;------------------------------------------------
    bcf     RCON, 7, A        ; IPEN = 0  ? single priority, vector @ 0x0008

    clrf    INTCON, A         ; disable all core interrupts + clear TMR0IF, INT0IF, RBIF
    clrf    PIR1, A           ; clear all peripheral interrupt flags (incl. ADIF)
    clrf    PIE1, A           ; disable all peripheral interrupts
    clrf    led_flash, A
    
    ;------------------------------------------------
    ; Make RA2 (AN2) analog, others digital
    ; bit = 0 to analog, 1 to digital
    ;------------------------------------------------
    bsf     WDTCON, 4, A       ; ADSHR = 1 -> access ANCON0/ANCON
    
    movlw   0xFB		       ; AN2 analog
    movwf   ANCON0, A
    
    movlw   0xFF            ; all upper analog pins digital (AN8?AN12)
    movwf   ANCON1, A
    
    bcf     WDTCON, 4, A       ; ADSHR = 0 -> back to ADCON0/ADCON1

    ;-----------------------------
    ; ADC setup for PIC18F87J50
    ;-----------------------------
    ; ADCON0: select AN2, Vref = Vdd/Vss, ADC on
    movlw 0x09      ; 0000 1001b
    movwf ADCON0, A

    ; ADCON1: ADFM=1 (right justify),
    ;         ACQT2:0 = 111  (20 Tad),
    ;         ADCS2:0 = 110  (Fosc/64)
    movlw   0x82       ; ADFM=1, ADCAL=0, ACQT=111, ADCS=110
    movwf   ADCON1, A

    ; --- Initialise event detection state ---
    movlw   ST_IDLE
    movwf   adc_state, A
    clrf    adc_state, A     ; ST_IDLE
    clrf    pulse_max, A
    clrf    dead_count, A
    clrf    adc_state, A
    clrf    pulse_max, A
    clrf    dead_count, A
    clrf    irqCountL, A
    clrf    irqCountH, A

    ; --- Enable ADC interrupt ---
    bsf     PIE1, 6, A        ; enable ADIE (bit 6)

    ; --- Enable global + peripheral interrupts ---
    bsf     INTCON, 6, A      ; PEIE (bit 6)
    bsf     INTCON, 7, A      ; GIE  (bit 7)
    
    ; --- Start first conversion ---
    bsf     ADCON0, 1, A     ; GO/DONE = 1

main_loop:
    ; ADC + LED logic is driven by interrupts
    ; add:
    ;   - service SD card
    ;   - SPI, UART, etc.
    bra     main_loop

;;================================================
; ADC sample handler - LED latch + peak tracking
;   - adc_hi     = latest ADRESH
;   - pulse_max  = max ADRESH seen during this pulse
;    UPDATED: CHANGING HYSTERISIS REARMING ENTIRELY TO FIXED DEADTIME
;================================================
    
adc_sample_handler:
    movf    led_flash, F, A
    btfsc   STATUS, 2, A            ; if led_flash == 0
    bra     adc_sm                  ; skip

    decfsz  led_flash, F, A
    bra     adc_sm

    ; reached 0 -> turn LED off
    bcf     LATD, 4, A

adc_sm:
    ;----------------------------------------
    ; 2) State machine: IDLE / INPULSE / DEAD
    ;----------------------------------------
    movf    adc_state, W, A
    xorlw   ST_IDLE
    btfsc   STATUS, 2, A
    bra     st_idle

    movf    adc_state, W, A
    xorlw   ST_INPULSE
    btfsc   STATUS, 2, A
    bra     st_inpulse

    ; otherwise ST_DEAD
    bra     st_dead


;----------------------------------------
; ST_IDLE: wait for threshold crossing
;----------------------------------------
st_idle:
    ; if adc_hi < SIGNAL_HI -> stay idle
    movlw   SIGNAL_HI
    subwf   adc_hi, W, A            ; W = adc_hi - SIGNAL_HI
    btfss   STATUS, 0, A            ; C=1 if adc_hi >= SIGNAL_HI
    return

    ; Event detected: enter INPULSE and init peak
    movlw   ST_INPULSE
    movwf   adc_state, A

    movf    adc_hi, W, A
    movwf   pulse_max, A

    ; visible LED flash
    movlw   200
    movwf   led_flash, A
    bsf     LATD, 4, A

    return
    
;----------------------------------------
; ST_INPULSE: track peak until pulse ends,
; then start fixed dead-time
;----------------------------------------
st_inpulse:
    ; Update peak: if adc_hi > pulse_max, pulse_max = adc_hi
    movf    adc_hi, W, A
    subwf   pulse_max, W, A         ; W = pulse_max - adc_hi
    btfsc   STATUS, 0, A            ; C=1 if pulse_max >= adc_hi
    bra     inpulse_check_end

    movf    adc_hi, W, A
    movwf   pulse_max, A

inpulse_check_end:
    ; End-of-pulse condition (below RESET_HI)
    movlw   RESET_HI
    subwf   adc_hi, W, A            ; W = adc_hi - RESET_HI
    btfsc   STATUS, 0, A            ; C=1 if adc_hi >= RESET_HI
    return                          ; still in pulse

    ; Pulse finished -> enter fixed dead-time
    movlw   ST_DEAD
    movwf   adc_state, A

    movlw   DEAD_SAMPLES
    movwf   dead_count, A

    ; At this point, pulse_max holds the captured peak.
    ; (Push to buffer / log later in main or another routine.)
    return

;----------------------------------------
; ST_DEAD: ignore events for DEAD_SAMPLES
;----------------------------------------
st_dead:
    ; dead_count--
    movf    dead_count, F, A
    btfsc   STATUS, 2, A            ; already 0? (safety)
    bra     dead_done

    decfsz  dead_count, F, A
    return                          ; still dead

dead_done:
    ; Dead-time complete -> go back to idle
    movlw   ST_IDLE
    movwf   adc_state, A
    return

;------------------------------------------------
; RAM variables (in access bank)
;------------------------------------------------
    PSECT   udata_acs
delay1:     ds  1
delay2:     ds  1
saveW:          ds 1
saveSTATUS:     ds 1
saveBSR:        ds 1

adc_hi:         ds 1   ; last ADRESH
adc_state:      ds 1   ; 0/1/2 = IDLE / INPULSE / DEAD
pulse_max:      ds 1   ; max ADRESH seen in this pulse
dead_count:     ds 1   ; down-counter for dead-time

irqCountL:      ds 1
irqCountH:      ds 1
led_flash:      ds 1
    
    END     reset_vector