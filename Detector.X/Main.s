    PROCESSOR   18F87J50
    
  ; === Configuration for PIC18F87J50 clicker2 ===
    #include <xc.inc>

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
    goto    start

;--------------------------------------
; Main code
;--------------------------------------
    PSECT   code, class=CODE
    ORG     0x0100

start:
    ;-----------------------------
    ; I/O setup
    ;-----------------------------
    clrf    TRISB, A          ; all PORTB as outputs (LED/debug)
    bsf     TRISA, 0, A       ; RA0 = input (AN0)

    ;-----------------------------
    ; Configure analog pins
    ;-----------------------------
    movlw   0xFE              ; AN0 analog, AN1?AN7 digital
    movwf   ANCON0, A
    movlw   0xFF              ; AN8?AN12 digital
    movwf   ANCON1, A

    ;-----------------------------
    ; ADC setup
    ;-----------------------------
    clrf    ADCON1, A         ; Vref+ = Vdd, Vref- = Vss

    ; ADCON2: right-justified, ACQT & ADCS set
    movlw   0xBE        ; ADFM=1, ACQT=111, ADCS=110 (Fosc/64)
    movwf   ADCON2, A

    ; ADCON0: select AN0, turn ADC on
    movlw   0x1        ; CHS=0000 (AN0), ADON=1
    movwf   ADCON0, A

main_loop:
    ; Small acquisition delay
    nop
    nop

    ; Start conversion (set GO/DONE)
    bsf     ADCON0, 1, A

wait_conv:
    btfsc   ADCON0, 1, A      ; wait while GO/DONE = 1
    bra     wait_conv

    ;-----------------------------
    ; Threshold test on ADRESH
    ;-----------------------------
    movlw   0x20              ; threshold

    cpfslt  ADRESH, A         ; skip next if ADRESH < 0x20
    bra     above_thresh      ; executes only if ADRESH >= 0x20

    ; below threshold
    bcf     LATB, 0, A        ; LED off
    bra     main_loop

above_thresh:
    bsf     LATB, 0, A        ; LED on
    bra     main_loop

    END     reset_vector
