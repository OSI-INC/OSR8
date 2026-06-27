; ---------------------------------------------------------------
; Add N bytes pointed to by IY to N bytes pointed to by IX. Both 
; IX and IY point to the least significant byte of both operands, 
; and will be decremented to get to the remaining bytes of the 
; operands. We pass N in A. The routine over-writes the bytes 
; pointed to by IY with the result of the addision, so the effect
; is like (IY) := (IX) + (IY). Registers A, IX, and IY are returned 
; unchanged. The routine ignores the incoming carry flag, but sets 
; the outgoing carry flag. The routine takes 29 + 15N clock cycles.

add_8n:

; Save A, B, C, IX, and IY on the stack.

push A
push B
push C
push IX
push IY

; We use C to count the bytes.

push A
pop C

; Clear the flags.

clrf

; The subtraction loop.

add_8n_loop:
ld A,(IY)
push A
pop B
ld A,(IX)
adc A,B
ld (IY),A
dec IX
dec IY
dec C
jp nz,add_8n_loop

; Recover registers.

pop IY
pop IX
pop C
pop B
pop A
ret

