; ---------------------------------------------------------------
; Obtain the product of two sixteen-bit operands. We pass in IX and
; IY pointers to the least significant bytes of both operands. The
; IY operand will play the role of the multiplicand. The IX operand
; will play the role of the multiplier. The routine assumes big-endian 
; byte ordering. When the calculation is complete, the routine writes
; the product into the operand locations, placing the least significant 
; bytes at the locations pointed to by IY, in big-endian order, and the 
; most significant bytes at the locations pointed to by IX, in big-endian 
; order.
;
; The routine uses the stack as its work space. We store in HL the 
; address of the least significant byte of our multiplicand, and refer 
; to other work space addresses as HL+k, where k is an offset -3 to +4 as 
; shown in the workspace table below.
;
; HL+5:  stack pointer base during operation
; HL+4:  product byte 0
; HL+3:  product byte 1
; HL+2:  product byte 2
; HL+1:  product byte 3
; HL+0:  multiplicand byte 0, address stored in HL
; HL-1:  multiplicand byte 1
; HL-2:  multiplicand byte 2
; HL-3:  multiplicand byte 3
; HL-4:  calling routine program counter byte 0
; HL-5:  calling routine program counter byte 1
;
; This routine is re-entrant and runs in both slow and boost modes.

mult_16:

; Save registers and flags on the stack.

push F
push A
push B
push C
push D
push E
push L
push H
push IX
push IY

; Push eight zeros onto the stack. After we push
; the third zero, save the stack pointer to HL.

ld A,0
push A    ; multiplicand byte 3, HL-3
push A    ; multiplicand byte 2, HL-2
push A    ; multiplicand byte 1, HL-1
ld HL,SP  ; SP now points to unused byte on top of stack.
push A    ; multiplicand byte 0, HL+0
push A    ; product byte 3, HL+1
push A    ; product byte 2, HL+2
push A    ; product byte 1, HL+3
push A    ; product byte 0, HL+4

; Copy the IY operand into multiplicand bytes zero and one.
; We use IX as the destination pointer, so we push IX, set
; it to point to the multiplicand, copy, then restore IY
; and pop IX back again.

push IX
push H
push L
pop IX
ld A,(IY)
ld (IX),A
dec IX
dec IY
ld A,(IY)
ld (IX),A
inc IY
pop IX

; We use D to count down from sixteen to zero.

ld A,16
push A
pop D

; Save the value of IY on the stack. We will need it later
; when we write the least significant bytes of the product
; to the IY location. During our calculation, we will use
; IY to point to our workspace.

push IY

; Point IY to the least significant byte of the multiplicand.
; We already have IX pointing to the least significant byte
; of the multiplier. Our product is zero and our multiplicand
; upper two bytes are zero. Its lower two bytes consist of
; the copied IY operand.

push H
push L
pop IY

; We execute the multiplication loop sixteen times. Each
; execution consults the least significant bit of the 
; multiplier. If set, we add the multiplicand to the product.
; Otherwise we do not add anything to the product. In either
; case, we shift the four-byte multiplicand left by one bit
; and we shift the two-byte multiplier right by one bit.

mult_16_loop:

ld A,(IX)
and A,0x01
jp z,mult_16_noadd

; Use IY to point to the product and IX to point to the
; multiplicand. We are going to perform a four byte
; addition that does (IY) := (IX) + (IY). We have to 
; save the IX multiplier pointer, set IX equal to IY
; and then move IY up to point to the product least 
; significant byte.

push IX
push IY
pop IX
inc IY
inc IY
inc IY
inc IY
ld A,4
call add_8n
pop IX

mult_16_noadd:

; Make sure IY is pointed to the least significant byte
; of the multiplicand.

push H
push L
pop IY

; Shift the multiplicand left, making sure we shift a zero
; in for the least significant bit. We have to use IX to
; point to the multiplicand in order to call the left_8n
; routine.

push IX
push IY
pop IX
clrf
ld A,4
call left_8n
pop IX

; Shift the multiplier right. We do not care what the carry
; bit is going in because these added bits will never be
; tested. We have to move IX to the most significant byte
; of the multiplier before calling right_8N.

dec IX
ld A,2
call right_8n
inc IX

; Decrement our counter. If it is not yet zero, run the 
; loop again.

dec D
jp nz,mult_16_loop

; Copy the most significant two bytes of the product to the
; IX locations. We have IY pointing at the least significant
; byte of the multiplicand, so incrementing IY points to the
; most significant product byte. We have IX pointing to the
; least significant multiplier byte, so decrementing points 
; to the most significant multiplier byte.

inc IY      ; product byte 3, HL+1
dec IX
ld A,(IY)
ld (IX),A
inc IY      ; product byte 2, HL+2
inc IX
ld A,(IY)
ld (IX),A

; Now we pop the value of the original IY, which points to 
; the least significant byte of the operand we used for our 
; multiplicand, into IX and decrement to point to the most
; significant byte of that operand. We copy the least 
; significant bytes of the product into the IX locations.

pop IX
dec IX
inc IY      ; product byte 1, HL+3
ld A,(IY)
ld (IX),A
inc IY      ; product byte 0, HL+4
inc IX
ld A,(IY)
ld (IX),A

; Free the workspace.

pop A
pop A
pop A
pop A
pop A
pop A
pop A
pop A

; Recover registers and flags.

pop IY
pop IX
pop H
pop L
pop E
pop D
pop C
pop B
pop A
pop F
ret

