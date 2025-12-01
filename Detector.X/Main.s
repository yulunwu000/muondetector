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

;------------------------------------------------
; Reset vector
;------------------------------------------------
PSECT resetVec, class=CODE, abs
ORG 0x0000
    goto start


;------------------------------------------------
; Interrupt vector
;------------------------------------------------
PSECT intVec, class=CODE, abs
ORG 0x0008
    goto isr

;------------------------------------------------
; Main program
;------------------------------------------------
PSECT code, class=CODE
ORG 0x0100

start:
    ;------------------------------------------------
    ; Disable all analog inputs: make ports digital
    ;------------------------------------------------
    movlw   0xFF
    movwf   ANCON0, A      ; AN0?AN7 all digital
    movwf   ANCON1, A      ; AN8?AN12 all digital
 
    ; LED on RD4
    clrf TRISD, A
    bcf  LATD, 4, A

    ; ==== Configure RB2 as input ====
    bsf TRISB, 2, A        ; RB2 = input

    ; ==== Configure INT2 (RB2) ====
    bsf INTCON2, 4, A      ; INTEDG2=1 ? interrupt on rising edge !!!IMPORTANT-- DOES COMPARATOR OUTPUT RISING OR FALLING EDGE? !!!
    bcf INTCON3, 1, A      ; INT2IF = 0 (clear any pending flag)
    bsf INTCON3, 4, A      ; enable INT2 external interrupt
    bsf INTCON, 7, A       ; GIE = 1 (global interrupt enable)
main_loop:
    bra     main_loop          ; do nothing, wait for interrupts

;------------------------------------------------
; Interrupt Service Routine
;------------------------------------------------
isr:
    ; Make sure it's really INT2
    btfss   INTCON3, 1, A      ; INT2IF set?
    retfie

    bcf     INTCON3, 1, A      ; clear INT2IF

    ; Turn LED on
    bsf     LATD, 4, A

    ; ----- visible delay -----
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
    ; -------------------------

    ; Turn LED off
    bcf     LATD, 4, A
    retfie

;------------------------------------------------
; RAM variables (in access bank)
;------------------------------------------------
    PSECT   udata_acs          ; access RAM section

delay1:     ds  1              ; reserve 1 byte for delay1
delay2:     ds  1              ; reserve 1 byte for delay2
    
    END