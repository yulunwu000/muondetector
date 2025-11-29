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
    bsf     TRISA, 0, A        ; configure RA0 (AN0) as input for ADC
    clrf    TRISD, A           ; all PORTD as outputs
    bcf     LATD, 4, A         ; LD1 (RD4) off initially

    ;------------------------------------------------
    ; Make RA0 analog, others digital
    ; bit = 0 to analog, 1 to digital
    ;------------------------------------------------
    movlw   0xFE               ; 1111 1110b: AN0 analog, AN1?AN7 digital since bit0=0, bits1-7=1
    movwf   ANCON0, A
    movlw   0xFF               ; AN8?AN12 all digital
    movwf   ANCON1, A

    ;-----------------------------
    ; ADC setup for PIC18F87J50
    ;-----------------------------
    ; ADCON0: select AN0, Vref = Vdd/Vss, ADC on
    movlw   0x01     ; VCFG1:VCFG0 = 00, CHS3:0 = 0000 (AN0), ADON = 1
    movwf   ADCON0, A

    ; ADCON1: ADFM=1 (right justify),
    ;         ACQT2:0 = 111  (20 Tad),
    ;         ADCS2:0 = 110  (Fosc/64)
    movlw   0xBE       ; ADFM=1, ADCAL=0, ACQT=111, ADCS=110
    movwf   ADCON1, A
  
main_loop:
    ;------------------------------------------------
    ; Start conversion on AN0
    ;------------------------------------------------
    bsf     ADCON0, 1, A       ; GO/DONE = 1

wait_conv:
    btfsc   ADCON0, 1, A       ; wait while GO/DONE = 1
    bra     wait_conv

    ;------------------------------------------------
    ; Compare ADRESH with threshold
    ;------------------------------------------------
    movlw   0x10               ; adjust threshold as needed
    subwf   ADRESH, W, A       ; W = ADRESH minus 0x20

    btfss   STATUS, 0, A       ; C=1 if ADRESH <= 0x20
    bra     below_thresh       ; if C=0, below threshold

; === pulse detected ===
pulse_detected:
    bsf     LATD, 4, A         ; turn LD1 on

    ; short visible delay so you can see the blink
    movlw   0x20
    movwf   delay1, A
delay1_loop:
    movlw   0xFF
    movwf   delay2, A
delay2_loop:
    decfsz  delay2, F, A
    bra     delay2_loop
    decfsz  delay1, F, A
    bra     delay1_loop

    bcf     LATD, 4, A         ; LED off again
    bra     main_loop

below_thresh:
    bcf     LATD, 4, A         ; make sure LED is off
    bra     main_loop


;------------------------------------------------
; RAM variables (in access bank)
;------------------------------------------------
    PSECT   udata_acs
delay1:     ds  1
delay2:     ds  1

    END     reset_vector