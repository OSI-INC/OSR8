; ---------------------------------------------------------------
; Copy N bytes pointed to by IX to the area pointed to by IY.
; Both IX and IY point to the first byte to by copied, and both
; are incremented thereafter. At the end of the routine, IX
; and IY point to the locations immediately after the final byte
; copied. Register A remains unchanged. The routine takes 16 + 9N 
; clock cycles.

copy_8n:

; Save A and C on the stack.

push A
push C


; We use C to count the bytes.

push A
pop C

; The copy loop.

copy_8n_loop:
ld A,(IX)
ld (IY),A
inc IX
inc IY
dec C
jp nz,copy_8n_loop

; Recover registers.

pop C
pop A
ret

