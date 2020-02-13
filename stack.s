; https://www.microchip.com/forums/m966141.aspx
.ifdef __dsPIC33F
.include "p33fxxxx.inc"
.endif

.ifdef __PIC24H
.include "p24hxxxx.inc"
.endif

 .global _fill_stack
 .global _check_stack
 .section .text

_fill_stack:
    mov #0xA5A5,w3 ;This is the fill Word
    mov SPLIM,w4 ;This is the top of the stack
LOOP1:
    mov w3,[w4--] ;Write the fill word into the stack, post decrement the address
    sub w4,w15,w5 ;Subtract w4 from the stack pointer w15, and store in w5
    bra NZ,LOOP1 ;Loop around until we reach the stack pointer
    return
    
_check_stack:
    mov SPLIM,w4 ;This is the top of the stack
    clr w6 ;w6 will store the number of unused stack words
    mov #0xA5A5,w7
LOOP2:
    inc2 w6,w6 ;Increment w6
    mov [w4--],w5 ;Move data pointed to by w4, into w5. Post decrement w4
    sub w7,w5,w5 ;Subtract the fill word (0xA5A5) from w5, storing it in w5
    bra Z,LOOP2 ;Loop around until 0xA5A5 isn't found
    
    dec2 w6,w6 ;It gets incremented each time, so it should be decremented once to be accurate
    mov w6,_UnusedStackBytes;Put w6 into UnusedStackBytes
    mov w4,_MaxStackPointer ;Put the value into StackPointer
    mov SPLIM,w4 ;This is the top of the stack
    mov w4,_StackLimit ;Put the Stack limit into StackLimit
    return


