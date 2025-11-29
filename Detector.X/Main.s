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

;------------------------------------------------
; Main program
;------------------------------------------------
PSECT code, class=CODE
ORG 0x0100

start:
    ;------------------------------------------------
    ; Make everything digital
    ;------------------------------------------------
    movlw   0xFF
    movwf   ANCON0, A      ; AN0?AN7 digital
    movwf   ANCON1, A      ; AN8?AN12 digital

    ; LED on RD4
    clrf    TRISD, A       ; all PORTD outputs
    bcf     LATD, 4, A     ; LED off

    ; RB2 as input
    bsf     TRISB, 2, A    ; RB2 input (INT pin on mikroBUS 2)

main_loop:
    ; read RB2 and mirror it to LED

    bcf     LATD, 4, A     ; default LED off

    btfsc   PORTB, 2, A    ; if RB2 is HIGH, skip next
    bsf     LATD, 4, A     ; turn LED on when RB2=1

    bra     main_loop
