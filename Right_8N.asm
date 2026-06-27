; ---------------------------------------------------------------
; Shift N bytes one bit to the right, starting with the byte 
; pointed to by IX and incrementing IX to get to each successive
; byte, as we would when starting with the most significant 
; byte of a multi-byte value in big-endian byte ordering. We
; pass the number of bytes to be rotated in A. The routine 
; shifts the carry flag into the top bit of the first byte
; and shifts the bottom bit of the last byte into the carry 
; flag at the end. Pointer IX and Register A are returned with 
; their original values intact. The routine takes 21 + 9N clock 
; cycles to complete, including the call.

right_8n:

; Save A, C, and IX on the stack.

push A
push C
push IX

; We use C to count the bytes.

push A
pop C

; The shift right loop.

right_8n_loop:
ld A,(IX)
rr A
ld (IX),A
inc IX
dec C
jp nz,right_8n_loop

; Recover registers.

pop IX
pop C
pop A
ret

