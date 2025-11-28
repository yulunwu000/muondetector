    PROCESSOR   18F87J50
    
  ; === Configuration for PIC18F87J50 clicker2 ===
    ; Tell XC8 which PIC18 device we are using
    #define __18F87J50    1

    ; Core assembler definitions (required)
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
    goto    start

;--------------------------------------
; Main code
;--------------------------------------
    PSECT   code, class=CODE
    ORG     0x0100

start:
    ;------------------------------------------------
    ; Make PORTD digital and set RD4 as output
    ;------------------------------------------------
    movlw   0xFF
    movwf   ANCON0, A        ; all AN0?AN7 digital
    movwf   ANCON1, A        ; all AN8?AN12 digital

    clrf    TRISD, A         ; all PORTD pins outputs
    bcf     LATD, 4, A       ; start with LED off on RD4

blink_loop:
    ; Toggle RD4 LED
    btg     LATD, 4, A

    ;------------------------------------------------
    ; Crude software delay
    ;------------------------------------------------
    movlw   0xFF
    movwf   delay1, A
delay_loop1:
    movlw   0xFF
    movwf   delay2, A
delay_loop2:
    decfsz  delay2, F, A
    bra     delay_loop2
    decfsz  delay1, F, A
    bra     delay_loop1

    bra     blink_loop       ; repeat forever


;------------------------------------------------
; RAM variables (in access bank)
;------------------------------------------------
    PSECT   udata_acs
delay1:
    ds 1
delay2:
    ds 1

    END     reset_vector
