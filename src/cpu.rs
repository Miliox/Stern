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
        let opcode = self.room[self.r.pc as usize];
        self.r.pc += 1;

        let elapsed = match opcode {
            // BRK impl
            0x00 => {
                self.brk();
                7
            }

            // ORA X,ind
            0x01 => {
                let offset = (self.fetch() as usize) + (self.r.x as usize);

                let addr = self.read_address(offset);

                let value = self.room[addr];

                self.ora(value);
                6
            }

            // ORA zpg
            0x05 => {
                let addr = self.fetch() as usize;

                let value = self.room[addr];

                self.ora(value);
                3
            }

            // ORA #
            0x09 => {
                let value = self.fetch();

                self.ora(value);
                2
            }

            // ORA abs
            0x0d => {
                let addr = self.fetch_address();

                let value = self.room[addr];

                self.ora(value);
                4
            }

            // ORA ind,Y
            0x11 => {
                let offset = self.fetch() as usize;

                let addr = self.read_address(offset) + (self.r.y as usize);

                let value = self.room[addr];

                self.ora(value);
                5 // TODO: Increment by one if page boundary crossed
            }

            // ORA zpg,X
            0x15 => {
                let addr = (self.fetch() as usize) + (self.r.x as usize);

                let value = self.room[addr];

                self.ora(value);
                4
            }

            // ORA abs,Y
            0x19 => {
                let addr = self.fetch_address() + (self.r.y as usize);

                let value = self.room[addr];

                self.ora(value);
                4 // TODO: Increment by one if page boundary crossed
            }

            // ORA abs,X
            0x1d => {
                let addr = self.fetch_address() + (self.r.x as usize);

                let value = self.room[addr];

                self.ora(value);
                4 // TODO: Increment by one if page boundary crossed
            }
            _ => panic!("opcode {:x} not implemented yet!", opcode)
        };

        self.clock += elapsed;
    }

    /// Fetchs the next byte indexed by pc register and increment pc by one
    fn fetch(&mut self) -> u8 {
        let b = self.room[self.r.pc as usize];
        self.r.pc += 1;
        b
    }

    // Fetchs the immediate address indexed by pc register and increment pc by two
    fn fetch_address(&mut self) -> usize {
        let ll = self.fetch() as usize;
        let hh = self.fetch() as usize;
        (hh << 8) + ll
    }

    // Read the address from memory by a given memory address
    fn read_address(&mut self, addr: usize) -> usize {
        let ll = self.room[addr] as usize;
        let hh = self.room[addr + 1] as usize;
        (hh << 8) + ll
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
        self.r.pc += 1;
        self.r.sr |= StatusFlags::I as u8;
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
    fn ora(&mut self, value: u8) {
        self.r.a |= value;
        if self.r.a == 0 {
            self.r.sr |= StatusFlags::Z as u8;
        }
        if (self.r.a & 0b1000_0000) != 0  {
            self.r.sr |= StatusFlags::N as u8;
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_fetch_sequence() {
        let mut cpu = Cpu::new();
        cpu.load([0x00, 0x01, 0x02, 0x03, 0x4, 0x5].to_vec());
        assert!(cpu.r.pc == 0x00);

        assert!(cpu.fetch() == 0x00);
        assert!(cpu.r.pc == 0x01);

        assert!(cpu.fetch() == 0x01);
        assert!(cpu.r.pc == 0x02);

        assert!(cpu.fetch() == 0x02);
        assert!(cpu.r.pc == 0x03);

        assert!(cpu.fetch() == 0x03);
        assert!(cpu.r.pc == 0x04);

        assert!(cpu.fetch() == 0x04);
        assert!(cpu.r.pc == 0x05);

        assert!(cpu.fetch() == 0x05);
        assert!(cpu.r.pc == 0x06);
    }

    #[test]
    fn cpu_ora_zero() {
        let mut cpu = Cpu::new();
        cpu.ora(0x00);
        assert!(cpu.r.pc == 0);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == StatusFlags::Z as u8);
        assert!(cpu.clock == 0);
    }

    #[test]
    fn cpu_ora_neg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1010_0101;
        cpu.ora(  0b1100_0011);
        assert!(cpu.r.pc == 0);
        assert!(cpu.r.a == 0b1110_0111);
        assert!(cpu.r.sr == StatusFlags::N as u8);
        assert!(cpu.clock == 0);
    }

    #[test]
    fn cpu_ora_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b0010_0101;
        cpu.ora(  0b0100_0011);
        assert!(cpu.r.pc == 0);
        assert!(cpu.r.a == 0b0110_0111);
        assert!(cpu.r.sr == 0);
        assert!(cpu.clock == 0);
    }

    #[test]
    fn cpu_instruction_brk() {
        let mut cpu = Cpu::new();
        cpu.load([0x00, 0x00, 0x00].to_vec());
        cpu.step();
        assert!(cpu.r.pc == 0x02);
        assert!(cpu.r.sr == StatusFlags::I as u8);
        assert!(cpu.clock == 7);
    }

    #[test]
    fn cpu_instruction_ora_immidiate() {
        let mut cpu = Cpu::new();
        cpu.load([0x09, 0x00].to_vec());
        cpu.step();
        assert!(cpu.r.pc == 0x02);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == StatusFlags::Z as u8);
        assert!(cpu.clock == 2);
    }

    #[test]
    fn cpu_instruction_ora_x_ind() {
        let mut cpu = Cpu::new();
        cpu.r.x = 1;
        cpu.load([0x01, 0x01, 0x04, 0x00, 0b0011_1001].to_vec());
        cpu.step();
        // execution:
        //   offset = 0x01(op) + 0x01(x) => 0x02
        //   mem[offset] = 0x0004
        //   value[mem[offset]] => 0b0011_1001
        assert!(cpu.r.pc == 0x02);
        assert!(cpu.r.a == 0b0011_1001);
        assert!(cpu.r.sr == 0);
        assert!(cpu.clock == 6);
    }

    #[test]
    fn cpu_instruction_ora_ind_y() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1001_0110;
        cpu.r.y = 1;
        cpu.load([0x11, 0x02, 0x04, 0x00, 0x00, 0xff].to_vec());
        cpu.step();
        // execution:
        //   offset = 0x02(op)
        //   mem[offset] = 0x0004
        //   value[mem[offset] + 0x01(x)] = 0xff
        assert!(cpu.r.pc == 0x02);
        assert!(cpu.r.a == 0xff);
        assert!(cpu.r.sr == StatusFlags::N as u8);
        assert!(cpu.clock == 5);
    }

    #[test]
    fn cpu_instruction_ora_zpg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b0110_0000;
        cpu.load([0x05, 0x03, 0x00, 0b1111_0000, 0b0000_1111].to_vec());
        cpu.step();
        assert!(cpu.r.pc == 0x02);
        assert!(cpu.r.a == 0b1111_0000);
        assert!(cpu.r.sr == StatusFlags::N as u8);
        assert!(cpu.clock == 3);
    }

    #[test]
    fn cpu_instruction_ora_zpg_x() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b0110_0000;
        cpu.r.x = 1;
        cpu.load([0x15, 0x03, 0x00, 0b1111_0000, 0b0000_1111].to_vec());
        cpu.step();
        assert!(cpu.r.pc == 0x02);
        assert!(cpu.r.a == 0b0110_1111);
        assert!(cpu.r.sr == 0);
        assert!(cpu.clock == 4);
    }

    #[test]
    fn cpu_instruction_ora_abs() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b0110_0000;
        cpu.load([0x0d, 0x03, 0x00, 0b0000_1111].to_vec());
        cpu.step();
        assert!(cpu.r.pc == 0x03);
        assert!(cpu.r.a == 0b0110_1111);
        assert!(cpu.r.sr == 0);
        assert!(cpu.clock == 4);
    }

    #[test]
    fn cpu_instruction_ora_abs_x() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b0001_0000;
        cpu.r.x = 1;
        cpu.load([0x1d, 0x03, 0x00, 0b1111_0000, 0b0000_1111].to_vec());
        cpu.step();
        assert!(cpu.r.pc == 0x03);
        assert!(cpu.r.a == 0b0001_1111);
        assert!(cpu.r.sr == 0);
        assert!(cpu.clock == 4);
    }

    #[test]
    fn cpu_instruction_ora_abs_y() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b0001_0000;
        cpu.r.y = 1;
        cpu.load([0x19, 0x03, 0x00, 0b1111_0000, 0b0000_1111].to_vec());
        cpu.step();
        assert!(cpu.r.pc == 0x03);
        assert!(cpu.r.a == 0b0001_1111);
        assert!(cpu.r.sr == 0);
        assert!(cpu.clock == 4);
    }

}