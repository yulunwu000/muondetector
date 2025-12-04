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

SIGNAL_HI   EQU 0x0D      ; ? 50 counts (from Arduino SIGNAL_THRESHOLD)
RESET_HI    EQU 0x07      ; ? 25 counts (from Arduino RESET_THRESHOLD)

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
    ;-----------------------------
    ; SPI init for microSD (MSSP1)
    ; Socket 1: SCK=RC3, SDO=RC5, SDI=RC4, CS=RD3
    ;-----------------------------

    ; Pin directions
    bcf     TRISC, 3, A        ; RC3/SCK  as output
    bcf     TRISC, 5, A        ; RC5/SDO  as output
    bsf     TRISC, 4, A        ; RC4/SDI  as input

    bcf     TRISD, 3, A        ; RD3/CS as output
    bsf     LATD, 3, A         ; CS high (deselected)

    ; SPI mode 0, master, Fosc/64
    movlw   0x40               ; SSPSTAT: SMP=0, CKE=1 (data valid on rising edge)
    movwf   SSP1STAT, A

    movlw   0x22        ; SSPCON1:
                               ;   SSPEN=1 (bit5) enable SPI
                               ;   CKP=0
                               ;   SSPM3:0=0010 -> Fosc/64
    movwf   SSP1CON1, A

    bcf     PIR1, 3, A         ; clear SSPIF

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
