.data
.text

#program that sorts data in memory in an ascending order

main:

        ori $v0, $zero, 0x16
        ori $v1, $zero, 0x21
        ori $a0, $zero, 0x32
        ori $a1, $zero, 0x46
        ori $a2, $zero, 0x54
        ori $a3, $zero, 0x66
        ori $t0, $zero, 0x7
        ori $t1, $zero, 0x8
        ori $t2, $zero, 0x9
        ori $t3, $zero, 0xa
        ori $t4, $zero, 0xb
        ori $t5, $zero, 0xc
        ori $t6, $zero, 0xd
        ori $t7, $zero, 0xe
        ori $s0, $zero, 0xf
        ori $s1, $zero, 0x10
        ori $s2, $zero, 0x11
        ori $s3, $zero, 0x12
        ori $s4, $zero, 0x13
        ori $s5, $zero, 0x14
        ori $s6, $zero, 0x15
        ori $s7, $zero, 0x16
        ori $t8, $zero, 0x17
        ori $t9, $zero, 0x18
        ori $k0, $zero, 0x19
        ori $k1, $zero, 0x1a
        addi $ra, $zero, -1

        ori $s0, $zero ,0x40
        sw $s0, 16($gp)

        ori $s0, $zero , 0x7
        sw $s0, 0($gp)

        ori $s0, $zero , 0x15
        sw $s0, 4($gp)

        ori $s0, $zero , 0x2000
        sw $s0, 8($gp)

        ori $s0, $zero , 0x100
        sw $s0, 12($gp)

        ori $s0, $zero , 0x85
        sw $s0, 36($gp)

        ori $s0, $zero , 0x201
        sw $s0, 20($gp)

        ori $s0, $zero , 0x329
        sw $s0, 24($gp)

        ori $s0, $zero , 0x753
        sw $s0, 28($gp)

        ori $s0, $zero , 0x451
        sw $s0, 32($gp)

        ori $s0, $zero , 0x94
        sw $s0, 40($gp)

        ori $s0, $zero , 0x07
        sw $s0, 44($gp)

        ori $s0, $zero , 0x591
        sw $s0, 48($gp)

        ori $s0, $zero , 0x36
        sw $s0, 52($gp)

        ori $s0, $zero , 0x04
        sw $s0, 56($gp)

        ori $s0, $zero , 0x14
        sw $s0, 60($gp)

        ori $s0, $zero , 0x78
        sw $s0, 64($gp)

        ori $s0, $zero , 0x76
        sw $s0, 68($gp)

        ori $s0, $zero , 0x30
        sw $s0, 72($gp)

        ori $s0, $zero , 0x93
        sw $s0, 76($gp)

        ori $s0, $zero , 0x89
        sw $s0, 80($gp)

        or $s0, $zero , $gp

        ori $s5, $zero, 0x15

        addi $s1, $zero, -1

        or $s2, $zero, $gp #first pointer
        or $s3, $zero, $gp #last pointer

        addi $s2, $s2, -4

loop1:

        addi $s2, $s2, 4
        and $s3, $s1, $s2
        addi $s3, $s3, 4
        and $s4, $s5, $s1
        addi $s5, $s5, -1
        addi $s4, $s4, -1
        beq $s5, $zero, loop4

loop2:

        lw $t0, 0($s2)
        lw $t1, 0($s3)
        add $zero, $zero, $zero
        slt $t3, $t1, $t0
        beq $t3, $zero, loop3

        addi $s3, $s3, 0x4
        addi $s4, $s4, -1
        beq $s4, $zero, loop1

        beq $zero, $zero, loop2

loop3:

        sw $t1, 0($s2)
        sw $t0, 0($s3)
        addi $s3, $s3, 0x4
        addi $s4, $s4, -1
        beq $s4, $zero, loop1

        beq $zero, $zero, loop2

loop4:

        or $s0, $zero , $gp
        ori $s5, $zero, 0x15

        addi $s1, $zero, -1

        or $s2, $zero, $gp #first pointer
        or $s3, $zero, $gp #last pointer
        addi $s2, $s2, -4

loop5:

        addi $s2, $s2, 4
        and $s3, $s1, $s2
        addi $s3, $s3, 4
        and $s4, $s5, $s1
        addi $s5, $s5, -1
        addi $s4, $s4, -1
        beq $s5, $zero, loop8

loop6:

        lw $t0, 0($s2)
        lw $t1, 0($s3)
        add $zero, $zero, $zero
        slt $t3, $t0, $t1
        beq $t3, $zero, loop7

        addi $s3, $s3, 0x4
        addi $s4, $s4, -1
        beq $s4, $zero, loop5

        beq $zero, $zero, loop6

loop7:

        sw $t1, 0($s2)
        sw $t0, 0($s3)
        addi $s3, $s3, 0x4
        addi $s4, $s4, -1
        beq $s4, $zero, loop5

        beq $zero, $zero, loop6

loop8:
        sll $zero, $zero, 0
        li  $v0, 10
        syscall
