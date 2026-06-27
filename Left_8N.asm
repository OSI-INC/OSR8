; ---------------------------------------------------------------
; Shift N bytes one bit to the left, starting with the byte 
; pointed to by IX and decrementing IX to get to each successive
; byte, as we would when starting with the least significant 
; byte of a multi-byte value in big-endian byte ordering. We
; pass the number of bytes to be rotated in A. The routine 
; shifts the carry flag into the bottom bit of the first byte
; and shifts the top bit of the last byte into the carry flag
; at the end. Pointer IX and Register A are returned with their 
; original values intact. The routine takes 21 + 9N clock cycles 
; to complete, including the call.
;
; This routine is re-entrant and runs in both slow and boost modes.

left_8n:

; Save A, C, and IX on the stack.

push A
push C
push IX

; We use C to count the bytes.

push A
pop C

; The shift left loop.

left_8n_loop:
ld A,(IX)
rl A
ld (IX),A
dec IX
dec C
jp nz,left_8n_loop

; Recover registers.

pop IX
pop C
pop A
ret

