; ---------------------------------------------------------------
; Obtain the square root of a 32-bit operand, returning the 16-bit 
; square root in the first two bytes of the operand location. The 
; IX register points to the least significant byte of the operand
; upon entry into the routine, and upon exit as well, when it is 
; then pointing to the least significant byte of the root. Byte 
; ordering is assumed big-endian. The algorithm shifts two bits at 
; time from the top byte of the operand. During operation, the 
; stack looks like this, from the top down. We store the original
; value of the stack pointer in HL just after pushing registers, so
; in our stack drawing HL+k means address HL plus k.
;
; HL+12: stack pointer base during operation
; HL+11: root byte 0
; HL+10: root byte 1
; HL+9:  root byte 2
; HL+8:  root byte 3
; HL+7:  trial byte 0
; HL+6:  trial byte 1
; HL+5:  trial byte 2
; HL+4:  trial byte 3
; HL+3:  remainder byte 0
; HL+2:  remainder byte 1
; HL+1:  remainder byte 2
; HL+0:  remainder byte 3, address stored in HL
; HL-1:  calling routine program counter byte 0
; HL-2:  calling routine program counter byte 1
;
; The algorithm is build upon two principles. First, when we add
; one to our root, its square increases by twice the root plus
; one. Second, if we divide the operand by 4^m, divide the root
; by 2^m, and ignore the bits after the decimal point, we obtain
; a value for the root that, when multiplied by 2^m, is less than
; or equal to the correct root.
;
; The algorithm starts by looking at the operand divided by 4^15,
; so that there are only two bits to look at, and the root divided
; by 2^15, although the root is at first zero. Our remainder we
; initialize as the top two bits from the operand, so it starts 
; between 00 and 11 (0 and 3). Our root is zero, so if we add one
; its square increases by one. We subtract one from the remainder,
; and if the result is positive, we set our root to one and our
; remainder to the difference. Otherwise, we leave the remainder
; equal to zero. In the next iteration, we multiply the remainder
; by four and add the top two bits from the operand. We multiply
; our root by two. We are now looking at the operand divided 
; by 4^14 and the root divided by 2^14. We take our root, multiply
; by two, and add one, to make the amount by which the square of
; the root will increase if we add one to its current value. If
; this is less than our remainder, we subtract from the remainder
; and add one to our root. Otherwise, we do nothing, we proceed
; to the next iteration, where we do the same thing: multiply the
; remainder by four, add the top two bits of the operand, and
; multiply the root by two. We are operating with the operand 
; divided by 4^13 and the root divided by 2^13.
;
; The algorithm consumes the operand two bits at a time from the
; top end. We start with a remainder and root that are both zero.
; In each iteration of the loop, we shift two bits out of the
; operant and into the remainder. Thus we multiply the remainder
; by four and add to it the top two bits of the operand. We also
; shift our root to the left by one, multiplying it by two. Now
; we take our root and multiply it by two and add one to get the
; amount by which our root squared will increase if we add one 
; to it now. If this trial value is less than our remainder, we
; go ahead and add one to our root, and we subtract the trial
; from our remainder. We keep going like this until we have 
; consumed all 32 bits of the operand, which is sixteen steps.
; We now have a sixteen bit root, which we transfer into the
; least signficant two bytes of the operand.
;
; The algorithm makes use of three external routines: left_8n,
; copy_8n, and sub_8n. In each case, we call these routines 
; with A loaded with 4, so that the routines will apply
; themselves to 4 bytes. 
; 
; To left_8n we must pass in IX the address of the least 
; significant byte of the four-byte operand that is to be 
; shifted left. The carry bit is shifted into the least 
; significant byte and set by the bit coming out of the most 
; significant byte. Both IX and A are unchanged. The routine
; descends from IX to successively more significant bytes in
; accordance with our big-endian byte ordering.
; 
; To copy_8n we must pass in IX the address of the first byte
; to be copied from and in IY the address of the first byte to
; be copied to. The routine increments IX and IY to copy in the
; upward direction and leaves IX and IY pointing to the byte
; above the final byte read and written respectively. Register
; A is unchanged.
;
; To sub_n we must pass in IX the address of the least
; significant byte of the operand from which we are going to
; subtract, and in IY the address of the least significant
; byte of the operand that we are going to subtract. The
; routine over-writes the second operand with the result of
; subtraction, using the same big-endian byte ordering. During
; subtraction, the routine descents from IX and IY to 
; successively more significant byte in accordance with our 
; big-endian byte ordering. The routine returns A, IX, and IY 
; unchanged.
;

sqrt_32:

; Save registers and flags on the stack.

push F
push A
push C
push IX
push IY

; Each iteration of our root-finding routine shifts two bits out
; of the 32-bit operand, which is sixteen iterations in all. We
; will use C to keep count.

ld A,16
push A
pop C

; Store the stack pointer in HL.

ld HL,SP

; We need space to perform this calculation, so we move the stack
; pointer up by twelve bytes so we can place three 32-bit local
; variables on the stack. We clear the locations as we go.

ld A,0
push A
push A
push A
push A
push A
push A
push A
push A
push A
push A
push A
push A

; We push IX, which points to the four-byte operand somewhere
; in memory. Any time we want to get IX back, we will pop and
; push it.

push IX

; The sqrt loop. In each run through the loop, we multipy our 
; root by two and see if we can set the new least significant
; bit to zero. 

sqrt_32n_loop:

; Shift the root left by one. We set IX = HL+11, clear the carry
; flag and call the shift-left routine for four bytes. We clear
; the carry flag so that the new least significant bit in our
; root will be zero.

push L
pop A
add A,11
push A
pop B
push H
pop A
adc A,0
push A
push B
pop IX
ld A,4
clrf
call left_8n

; Store HL+11 in IY. This saves time later.

push IX
pop IY

; Restore IX to point to the least significant byte of the operand
; and shift the operand by one to the left.

pop IX
push IX
call left_8n

; Set IX to HL and shift from the operand into the remainder.

push H
push L
pop IX
call left_8n

; Restore IX and shift the operand left by one again.

pop IX
push IX
call left_8n

; Set IX to HL and shift from the operand into the remainder.
; Once complete, the remainder has been multiplied by four and
; its bottom two bits are equal to the former top two bits of the
; operand.

push H
push L
pop IX
call left_8n

; We want to copy our root value into the trial variable. We take
; IY, which is currently HL+11 and decrement by three to get root
; byte 3.

push IY
pop IX  ; HL+11
dec IX  ; HL+10
dec IX  ; HL+9
dec IX  ; HL+8 = root byte 3

; We set IY to HL and increment by four to get to trial byte 3.

push H
push L
pop IY  ; HL+0
inc IY  ; HL+1
inc IY  ; HL+2
inc IY  ; HL+3
inc IY  ; HL+4 = trial byte 3

; Copy the root to the trial value.

call copy_8n

; Right now, IY has been incremented by four by the copy 
; routine, so it is at HL+8. Move it down to HL+7, which 
; is trial byte 0, and copy into IX.

dec IY
push IY
pop IX

; Shift the trial value to the left and set its least 
; significant bit to one. The trial value is now equal 
; to the increase in the square of our root that will 
; arise if we add one to the root. Pointer IX remains 
; at HL+7 after the shift. We set the least significant 
; bit of the trial value, then we restore A to 4.

call left_8n
ld A,(IX)
or A,0x01
ld (IX),A
ld A,4

; We have IX = HL+7, the least significant byte of the trial
; value. We move to IY. Then we set IX = HL and increment 
; three times to get to HL+3, the least significant byte of 
; the remainder.

push IX ; HL+7
pop IY  ; HL+7
push H
push L
pop IX  ; HL+0
inc IX  ; HL+1
inc IX  ; HL+2
inc IX  ; HL+3 = remainder byte 0

; Subtract the trial value from the remainder, leaving the
; difference in the trial value locations.

call sub_8n

; A negative result sets the carry bit, in which case we will
; do nothing more: the remainder was insufficient to allow us
; to add a one to the least significant bit of our root.

jp c,sqrt_32n_dec

; The trial value was less than or equal to the remainder, so 
; we are going to copy the difference into our remainder and add 
; one to our root. Right now IX is HL+3, remainder byte 0. We move 
; up to trial byte 3.

inc IX  ; HL+4 = trial byte 3

; Point IY to remainder byte 3, which is at HL.

push H
push L
pop IY  ; HL+0 = remainder byte 3

; Copy four bytes from trial to remainder.

call copy_8n

; We want to set the least significant bit of our root to one. 
; The copy from trial to remainder leaves IY = HL+4 and IX = HL+8. 
; We want to get to HL+11, so we increment IX three times.

inc IX  ; HL+9
inc IX  ; HL+10
inc IX  ; HL+11 = root byte 0

; Now set the least significant bit.

ld A,(IX)
or A,0x01
ld (IX),A

; We are done with this run through the loop. When we have done 
; this sixteen times, we should have the root byte 1 in HL+10 
; root byte 0 in HL+11. 

sqrt_32n_dec:
dec C
jp nz,sqrt_32n_loop

; Pop the address of the operand byte zero into IY, but restore
; on the stack, because we are going to pop IX later.

pop IY
push IY

; Right now IX will be IX = HL+11 if we added one to the root,
; but otherwise it will be IX = HL+3. We need it to be HL+11
; to point to root byte 0. So we make sure it is HL+11.

push L
pop A
add A,11
push A
pop B
push H
pop A
adc A,0
push A
push B
pop IX

; Copy the root into the least significant bytes of the operand

ld A,(IX)
ld (IY),A
dec IX
dec IY
ld A,(IX)
ld (IY),A

; Pop IX off the stack so it points to operand byte 0, as it did
; at the start of the routine.

pop IX

; Move the stack pointer back down again, leaving our intermediate
; variables behind.

pop A
pop A
pop A
pop A
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
pop C
pop A
pop F
ret

