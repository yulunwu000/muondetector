    PROCESSOR   18F87J50
    #include <xc.inc>

    CONFIG  XINST = OFF      ; disable extended instruction set

    PSECT   resetVec, class=CODE, abs
    ORG     0x0000
reset_vector:
    goto    start

    PSECT   code, class=CODE
    ORG     0x0100
start:
    clrf    TRISB, A
loop:
    incf    PORTB, F, A
    bra     loop

    END     reset_vector
