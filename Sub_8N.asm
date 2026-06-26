; ---------------------------------------------------------------
; Subtract N bytes pointed to by IY from N bytes pointed to by
; IX. Both IX and IY point to the least significant byte of
; both operands, and will be decremented to get to the remaining
; bytes of the operands. We pass N in A. The routine over-writes
; the bytes pointed to by IY with the result of the subtraction.
; Registers A, IX, and IY are returned unchanged. The routine
; ignores the incoming carry flag, but sets the outgoing carry 
; flag. The routine takes 29 + 15N clock cycles.

sub_8n:

; Save A and B on the stack.

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

sub_8n_loop:
ld A,(IY)
push A
pop B
ld A,(IX)
sbc A,B
ld (IY),A
dec IX
dec IY
dec C
jp nz,sub_8n_loop

; Recover registers.

pop IY
pop IX
pop C
pop B
pop A
ret

