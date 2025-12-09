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
SIGNAL_HI   EQU 0x10   ; ~0.32 V
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

    ; Read ADC results
    movf    ADRESH, W, A
    movwf   adc_hi, A
    
    ; DEBUG: slow blink ? toggle LED only when 16-bit counter overflows
    incfsz  irqCountL, F, A
    bra     no_toggle
    incfsz  irqCountH, F, A
    bra     no_toggle

    ; If we get here, both bytes just rolled over (overflow)
    btg     LATD, 4, A

no_toggle:

    ; Call state-machine handler
    ;call    adc_sample_handler

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
    retfie  1
    
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
    movlw   0xBE       ; ADFM=1, ADCAL=0, ACQT=111, ADCS=110
    movwf   ADCON1, A

    ; --- Initialize event detection state ---
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

;================================================
; ADC sample handler - one detector on AN2
;   - adc_hi holds ADRESH of latest sample
;   - states: ST_IDLE / ST_INPULSE / ST_DEAD
;   - controls LED on RD4
;================================================
adc_sample_handler:

    ; Decide which state we?re in
    movf    adc_state, W, A
    sublw   ST_IDLE           ; Z=1 if adc_state == ST_IDLE
    btfsc   STATUS, 2, A      ; Z bit (2)
    bra     adc_idle

    movf    adc_state, W, A
    sublw   ST_INPULSE        ; Z=1 if adc_state == ST_INPULSE
    btfsc   STATUS, 2, A
    bra     adc_inpulse

    ; Otherwise treat as ST_DEAD
    bra     adc_dead

;---------------- IDLE state ----------------
; Waiting for adc_hi >= SIGNAL_HI
adc_idle:
    ; Check if above SIGNAL_HI
    movlw   SIGNAL_HI
    subwf   adc_hi, W, A      ; W = adc_hi - SIGNAL_HI
    btfss   STATUS, 0, A      ; C=1 if adc_hi >= SIGNAL_HI
    return                    ; below threshold ? stay IDLE

    ; Crossed SIGNAL_HI ? pulse start
    movf    adc_hi, W, A
    movwf   pulse_max, A      ; start tracking peak

    movlw   ST_INPULSE
    movwf   adc_state, A

    ; LED ON (RD4)
    bsf     LATD, 4, A

    return

;---------------- IN_PULSE state ----------------
; Track peak and look for drop below RESET_HI
adc_inpulse:
    ; 1) Track maximum
    movf    adc_hi, W, A
    subwf   pulse_max, W, A   ; W = pulse_max - adc_hi
    btfsc   STATUS, 0, A      ; C=1 if pulse_max >= adc_hi
    bra     adc_inpulse_check_tail

    ; adc_hi > pulse_max ? update peak
    movf    adc_hi, W, A
    movwf   pulse_max, A

adc_inpulse_check_tail:
    ; 2) Check if signal has fallen below RESET_HI
    movlw   RESET_HI
    subwf   adc_hi, W, A      ; W = adc_hi - RESET_HI
    btfsc   STATUS, 0, A      ; C=1 if adc_hi >= RESET_HI
    return                    ; still above reset threshold ? stay IN_PULSE

    ; Now adc_hi < RESET_HI ? end of pulse
    ; LED OFF
    bcf     LATD, 4, A

    ; Start dead-time
    movlw   DEAD_SAMPLES
    movwf   dead_count, A

    movlw   ST_DEAD
    movwf   adc_state, A

    ; Here is where you'd eventually:
    ; - copy pulse_max into an event buffer for SD logging
    ; - maybe latch a "new event" flag
    return

;---------------- DEAD state ----------------
; Simple dead-time based on sample count; then wait
; until clearly back below RESET_HI before going IDLE.
adc_dead:
    ; If dead_count > 0, just count down
    movf    dead_count, F, A
    btfsc   STATUS, 2, A      ; Z=1 if dead_count == 0
    bra     adc_dead_check_reset

    ; dead_count != 0 ? decrement and stay in DEAD
    decfsz  dead_count, F, A
    return                    ; not yet zero ? stay DEAD

    ; If decfsz made it zero, fall through to reset check

adc_dead_check_reset:
    ; Only leave DEAD if we are clearly below RESET_HI
    movlw   RESET_HI
    subwf   adc_hi, W, A      ; W = adc_hi - RESET_HI
    btfsc   STATUS, 0, A      ; C=1 if adc_hi >= RESET_HI
    return                    ; still above reset threshold ? remain DEAD

    ; Below RESET_HI and dead_count == 0 ? go back to IDLE
    clrf    adc_state, A      ; ST_IDLE
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
    END     reset_vector