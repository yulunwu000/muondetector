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
    
;==== Constants (thresholds in ADRESH units) ==============================

SIGNAL_HI   EQU 0x0D      ; approx 50 counts (from Arduino SIGNAL_THRESHOLD)
RESET_HI    EQU 0x07      ; approx 25 counts (from Arduino RESET_THRESHOLD)

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
    ;------------------------------------------------
    ; PORT setup
    ;------------------------------------------------
    bsf	    TRISA, 2, A            ; RA2 input
    clrf    TRISD, A           ; all PORTD as outputs
    bsf     TRISB, 2, A        ; RB2 as input (UVP / INT)
    bcf     LATD, 4, A         ; LD1 (RD4) off initially

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

main_loop:
    ;===== 1) Wait for pulse above SIGNAL_HI =============================

wait_for_pulse:
    ; start conversion
    bsf     ADCON0, 1, A       ; GO/DONE = 1
adc_wait1:
    btfsc   ADCON0, 1, A       ; wait while GO/DONE = 1
    bra     adc_wait1

    ; ADRESH now holds the high 8 bits of the 10-bit result
    movlw   SIGNAL_HI
    subwf   ADRESH, W, A       ; W = ADRESH - SIGNAL_HI

    btfss   STATUS, 0, A       ; C=1 if ADRESH >= SIGNAL_HI
    bra     wait_for_pulse     ; if below threshold, keep waiting

    ;===== Pulse detected! Flash LED =====================================

    bsf     LATD, 4, A         ; LED ON

    ; crude visible delay (~ few ms)
    movlw   0x40
    movwf   delay1, A
delay1_loop:
    movlw   0xFF
    movwf   delay2, A
delay2_loop:
    decfsz  delay2, F, A
    bra     delay2_loop
    decfsz  delay1, F, A
    bra     delay1_loop

    bcf     LATD, 4, A         ; LED OFF

    ;===== 2) Deadtime: wait until signal < RESET_HI =====================

wait_reset:
    bsf     ADCON0, 1, A       ; new conversion
adc_wait2:
    btfsc   ADCON0, 1, A
    bra     adc_wait2

    movlw   RESET_HI
    subwf   ADRESH, W, A       ; W = ADRESH - RESET_HI

    btfsc   STATUS, 0, A       ; C=1 -> ADRESH >= RESET_HI
    bra     wait_reset         ; still above reset threshold, keep waiting

    ; back to waiting for next pulse
    bra     main_loop
;------------------------------------------------
; RAM variables (in access bank)
;------------------------------------------------
    PSECT   udata_acs
delay1:     ds  1
delay2:     ds  1

    END     reset_vector