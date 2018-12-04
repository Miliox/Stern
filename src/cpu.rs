use std::fmt;

/// StatusFlags contain the flags that are stored in the Status Register (sr).
#[allow(dead_code)]
#[derive(Debug)]
#[repr(u8)]
pub enum StatusFlags {
    /// Carry
    C = 0b1000_0000,

    /// Zero
    Z = 0b0100_0000,

    /// IRQ Disable
    I = 0b0010_0000,

    /// Decimal
    D = 0b0001_0000,

    /// Index Size
    X = 0b0000_1000,

    // IGNORED
    // _ = 0b0000_0100,

    /// Overflow
    V = 0b0000_0010,

    /// Negative
    N = 0b0000_0001
}

/// Registers of 6507/6502
#[allow(dead_code)]
pub struct Registers {
    /// Accumulator
    a : u8,

    /// Index X
    x : u8,

    /// Index Y
    y : u8,

    /// Status
    sr : u8,

    /// Stack Pointer
    sp : u8,

    /// Program Counter
    pc : u16
}


impl Registers {
    pub fn new() -> Registers {
        Registers { a: 0, x: 0, y: 0, sr: 0, sp: 0, pc: 0 }
    }
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "a:{:02x} x:{:02x} y:{:02x} sr:{:02x} sp:{:02x} pc:{:04x}",
               self.a, self.x, self.y, self.sr, self.sp, self.pc)
    }
}

/// Cpu
#[allow(dead_code)]
pub struct Cpu {
    r : Registers,
    clock : u64,
    room: Vec<u8>
}

impl fmt::Debug for Cpu {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "r:({:?}) clock:{:?} room_size:{:?}", self.r, self.clock, self.room.len())
    }
}

#[allow(dead_code)]
impl Cpu {
    pub fn new() -> Cpu {
        Cpu { r: Registers::new(), clock: 0, room: Vec::new()}
    }

    pub fn load(&mut self, room: Vec<u8>) {
        self.room = room;
    }

    pub fn step(&mut self) {

    }

    // Add Memory to Accumulator with Carry
    fn adc(&mut self) {
    }

    // AND Memory with Accumulator
    fn and(&mut self) {
    }

    // Shift Left One Bit (Memory or Accumulator)
    fn asl(&mut self) {
    }

    // Branch on Carry Clear
    fn bcc(&mut self) {
    }

    // Branch on Carry Set
    fn bcs(&mut self) {
    }

    // Branch on Result Zero
    fn beq(&mut self) {
    }


    // Test Bits in Memory with Accumulator
    fn bit(&mut self) {
    }

    // Branch on Result Minus
    fn bmi(&mut self) {
    }

    // Branch on Result not Zero
    fn bne(&mut self) {
    }

    // Branch on Result Plus
    fn bpl(&mut self) {
    }

    // Force Break
    fn brk(&mut self) {
    }

    // Branch on Overflow Clear
    fn bvc(&mut self) {
    }

    // Branch on Overflow Set
    fn bvs(&mut self) {
    }

    // Clear Carry Flag
    fn clc(&mut self) {
    }

    // Clear Decimal Mode
    fn cld(&mut self) {
    }

    // Clear Interrupt Disable Bit
    fn cli(&mut self) {
    }

    // Clear Overflow Flag
    fn clv(&mut self) {
    }

    // Compare Memory with Accumulator
    fn cmp(&mut self) {
    }

    // Compare Memory and Index X
    fn cpx(&mut self) {
    }

    // Compare Memory and Index Y
    fn cpy(&mut self) {
    }

    // Decrement Memory by One
    fn dec(&mut self) {
    }

    // Decrement Index X by One
    fn dex(&mut self) {
    }

    // Decrement Index Y by One
    fn dey(&mut self) {
    }

    // Exclusive-OR Memory with Accumulator
    fn eor(&mut self) {
    }

    // Increment Memory by One
    fn inc(&mut self) {
    }

    // Increment Index X by One
    fn inx(&mut self) {
    }

    // Increment Index Y by One
    fn iny(&mut self) {
    }

    // Jump to New Location
    fn jmp(&mut self) {
    }

    // Jump to New Location Saving Return Address
    fn jsr(&mut self) {
    }

    // Load Accumulator with Memory
    fn lda(&mut self) {
    }

    // Load Index X with Memory
    fn ldx(&mut self) {
    }

    // Load Index Y with Memory
    fn ldy(&mut self) {
    }

    // Shift One Bit Right (Memory or Accumulator)
    fn lsr(&mut self) {
    }

    // No Operation
    fn nop(&mut self) {
    }

    // OR Memory with Accumulator
    fn ora(&mut self) {
    }

    // Push Accumulator on Stack
    fn pha(&mut self) {
    }

    // Push Processor Status on Stack
    fn php(&mut self) {
    }

    // Pull Accumulator from Stack
    fn pla(&mut self) {
    }

    // Pull Processor Status from Stack
    fn plp(&mut self) {
    }

    // Rotate One Bit Left (Memory or Accumulator)
    fn rol(&mut self) {
    }

    // Rotate One Bit Right (Memory or Accumulator)
    fn ror(&mut self) {
    }

    // Return from Interrupt
    fn rti(&mut self) {
    }

    // Return from Subroutine
    fn rts(&mut self) {
    }

    // Subtract Memory from Accumulator with Borrow
    fn sbc(&mut self) {
    }

    // Set Carry Flag
    fn sec(&mut self) {
    }

    // Set Decimal Flag
    fn sed(&mut self) {
    }

    // Set Interrupt Disable Status
    fn sei(&mut self) {
    }

    // Store Accumulator in Memory
    fn sta(&mut self) {
    }

    // Store Index X in Memory
    fn stx(&mut self) {
    }

    // Store Index Y in Memory
    fn sty(&mut self) {
    }

    // Transfer Accumulator to Index X
    fn tax(&mut self) {
    }

    // Transfer Accumulator to Index Y
    fn tay(&mut self) {
    }

    // Transfer Stack Pointer to Index X
    fn tsx(&mut self) {
    }

    // Transfer Index X to Accumulator
    fn txa(&mut self) {
    }

    // Transfer Index X to Stack Register
    fn txs(&mut self) {
    }

    // Transfer Index Y to Accumulator
    fn tya(&mut self) {
    }

}
