use std::fmt;

/// StatusFlags contain the flags that are stored in the Status Register (sr).
#[allow(dead_code)]
pub mod status_flags {
    /// Carry
    pub const CARRY : u8 = 0b0000_0001;

    /// Zero
    pub const ZERO : u8 = 0b0000_0010;

    /// IRQ Disable
    pub const IRQD : u8 = 0b0000_0100;

    /// Decimal
    pub const DEC : u8 = 0b0000_1000;

    /// BRK Interrupt
    pub const BRK : u8 = 0b0001_0000;

    // UNUSED (Should Always Be Set)
    pub const UNUSED : u8 = 0b0010_0000;

    /// Overflow
    pub const OVER : u8 = 0b0100_0000;

    /// Negative
    pub const NEG : u8 = 0b1000_0000;
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
                let value = self.fetch_x_ind();
                self.ora(value);
                6
            }

            // ORA zpg
            0x05 => {
                let value = self.fetch_zpg();
                self.ora(value);
                3
            }

            // ASL zpg
            0x06 => {
                let value = self.fetch_zpg();
                self.asl(value);
                5
            }

            // ORA #
            0x09 => {
                let value = self.fetch();
                self.ora(value);
                2
            }

            // ASL A
            0x0a => {
                let value = self.r.a;
                self.asl(value);
                2
            }

            // ORA abs
            0x0d => {
                let value = self.fetch_abs();
                self.ora(value);
                4
            }

            // ASL abs
            0x0e => {
                let value = self.fetch_abs();
                self.asl(value);
                6
            }

            // ORA ind,Y
            0x11 => {
                let value = self.fetch_ind_y();
                self.ora(value);
                5 // TODO: Increment by one if page boundary crossed
            }

            // ORA zpg,X
            0x15 => {
                let value = self.fetch_zpg_x();
                self.ora(value);
                4
            }

            // ASL zpg,X
            0x16 => {
                let value = self.fetch_zpg_x();
                self.asl(value);
                6
            }

            // CLC
            0x18 => {
                self.clc();
                2
            }

            // ORA abs,Y
            0x19 => {
                let value = self.fetch_abs_y();
                self.ora(value);
                4 // TODO: Increment by one if page boundary crossed
            }

            // ORA abs,X
            0x1d => {
                let value = self.fetch_abs_x();
                self.ora(value);
                4 // TODO: Increment by one if page boundary crossed
            }


            // ASL abs,X
            0x1e => {
                let value = self.fetch_abs_x();
                self.asl(value);
                7
            }

            // AND X,ind
            0x21 => {
                let value = self.fetch_x_ind();
                self.and(value);
                6
            }

            // BIT zpg
            0x24 => {
                let value = self.fetch_zpg();
                self.bit(value);
                3
            }

            // AND zpg
            0x25 => {
                let value = self.fetch_zpg();
                self.and(value);
                3
            }

            // ROL zpg
            0x26 => {
                let value = self.fetch_zpg();
                self.rol(value);
                5
            }

            // AND #
            0x29 => {
                let value = self.fetch();
                self.and(value);
                2
            }

            // ROL A
            0x2a => {
                let value = self.r.a;
                self.rol(value);
                2
            }

            // BIT abs
            0x2c => {
                let value = self.fetch_abs();
                self.bit(value);
                4
            }

            // AND abs
            0x2d => {
                let value = self.fetch_abs();
                self.and(value);
                4
            }

            // ROL abs
            0x2e => {
                let value = self.fetch_abs();
                self.rol(value);
                6
            }

            // AND ind,Y
            0x31 => {
                let value = self.fetch_ind_y();
                self.and(value);
                5
            }

            // AND zpg,X
            0x35 => {
                let value = self.fetch_zpg_x();
                self.and(value);
                4
            }

            // ROL zpg,X
            0x36 => {
                let value = self.fetch_zpg_x();
                self.rol(value);
                6
            }

            // AND abs,Y
            0x39 => {
                let value = self.fetch_abs_y();
                self.and(value);
                4
            }

            // AND abs,X
            0x3d => {
                let value = self.fetch_abs_x();
                self.and(value);
                4
            }

            // ROL abs,X
            0x3e => {
                let value = self.fetch_abs_x();
                self.rol(value);
                7
            }

            // EOR X,ind
            0x41 => {
                let value = self.fetch_x_ind();
                self.eor(value);
                6
            }

            // EOR zpg
            0x45 => {
                let value = self.fetch_zpg();
                self.eor(value);
                3
            }

            // LSR zpg
            0x46 => {
                let value = self.fetch_zpg();
                self.lsr(value);
                5
            }

            // EOR #
            0x49 => {
                let value = self.fetch();
                self.eor(value);
                2
            }

            // LSR A
            0x4a => {
                let value = self.r.a;
                self.lsr(value);
                2
            }

            // EOR abs
            0x4d => {
                let value = self.fetch_abs();
                self.eor(value);
                4
            }

            // LSR abs
            0x4e => {
                let value = self.fetch_abs();
                self.lsr(value);
                6
            }

            // EOR ind,Y
            0x51 => {
                let value = self.fetch_ind_y();
                self.eor(value);
                5
            }

            // EOR zpg,X
            0x55 => {
                let value = self.fetch_zpg_x();
                self.eor(value);
                4
            }

            // LSR zpg,X
            0x56 => {
                let value = self.fetch_zpg_x();
                self.lsr(value);
                6
            }

            // CLI
            0x58 => {
                self.cli();
                2
            }

            // EOR abs,Y
            0x59 => {
                let value = self.fetch_abs_y();
                self.eor(value);
                4
            }

            // EOR abs,X
            0x5d => {
                let value = self.fetch_abs_x();
                self.eor(value);
                4
            }

            // LSR abs,X
            0x5e => {
                let value = self.fetch_abs_x();
                self.lsr(value);
                7
            }

            // ROR zpg
            0x66 => {
                let value = self.fetch_zpg();
                self.ror(value);
                5
            }

            // ROR A
            0x6a => {
                let value = self.r.a;
                self.ror(value);
                2
            }

            // ROR abs
            0x6e => {
                let value = self.fetch_abs();
                self.ror(value);
                6
            }

            // ROR zpg,X
            0x76 => {
                let value = self.fetch_zpg_x();
                self.ror(value);
                6
            }

            // ROR abs,X
            0x7e => {
                let value = self.fetch_abs_x();
                self.ror(value);
                7
            }

            // CLV
            0xb8 => {
                self.clv();
                2
            }

            // CPY #
            0xc0 => {
                let value = self.fetch();
                self.cpy(value);
                2
            }

            // CMP X,ind
            0xc1 => {
                let value = self.fetch_x_ind();
                self.cmp(value);
                6
            }

            // CPY zpg
            0xc4 => {
                let value = self.fetch_zpg();
                self.cpy(value);
                3
            }

            // CMP zpg
            0xc5 => {
                let value = self.fetch_zpg();
                self.cmp(value);
                3
            }

            // CMP #
            0xc9 => {
                let value = self.fetch();
                self.cmp(value);
                2
            }

            // CPY abs
            0xcc => {
                let value = self.fetch_abs();
                self.cpy(value);
                4
            }

            // CMP abs
            0xcd => {
                let value = self.fetch_abs();
                self.cmp(value);
                4
            }

            // CMP ind,Y
            0xd1 => {
                let value = self.fetch_ind_y();
                self.cmp(value);
                5
            }

            // CMP zpg,X
            0xd5 => {
                let value = self.fetch_zpg_x();
                self.cmp(value);
                4
            }

            // CLD
            0xd8 => {
                self.cld();
                2
            }

            // CMP abs,Y
            0xd9 => {
                let value = self.fetch_abs_y();
                self.cmp(value);
                4
            }

            // CMP abs,X
            0xdd => {
                let value = self.fetch_abs_x();
                self.cmp(value);
                4
            }

            // CPX #
            0xe0 => {
                let value = self.fetch();
                self.cpx(value);
                2
            }

            // CPX zpg
            0xe4 => {
                let value = self.fetch_zpg();
                self.cpx(value);
                3
            }

            // CPX abs
            0xec => {
                let value = self.fetch_abs();
                self.cpx(value);
                4
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

    // Fetchs the absolute address from the program and resolve his memory value
    fn fetch_abs(&mut self) -> u8 {
        let addr = self.fetch_address();
        self.room[addr]
    }

    // Fetchs the absolute address + x from the program and resolve his memory value
    fn fetch_abs_x(&mut self) -> u8 {
        let addr = self.fetch_address() + self.r.x as usize;
        self.room[addr]
    }

    // Fetchs the absolute address + y from the program and resolve his memory value
    fn fetch_abs_y(&mut self) -> u8 {
        let addr = self.fetch_address() + self.r.y as usize;
        self.room[addr]
    }

    // Fetchs the zeropage address from the program and resolve his memory value
    fn fetch_zpg(&mut self) -> u8 {
        let addr = self.fetch() as usize;
        self.room[addr]
    }

    // Fetchs the zeropage address + x from the program and resolve his memory value
    fn fetch_zpg_x(&mut self) -> u8 {
        let addr = self.fetch() as usize + self.r.x as usize;
        self.room[addr]
    }

    // Fetchs the zeropage address + y from the program and resolve his memory value
    fn fetch_zpg_y(&mut self) -> u8 {
        let addr = self.fetch() as usize + self.r.y as usize;
        self.room[addr]
    }

    // Fetchs an indirect address in zero page + x and resolve his memory value
    fn fetch_x_ind(&mut self) -> u8 {
        let zpg_addr = (self.fetch() as usize) + (self.r.x as usize);
        let addr = self.read_address(zpg_addr);
        self.room[addr]
    }

    // Fetchs an indirect address in zero page, increment by y and resolve his memory value
    fn fetch_ind_y(&mut self) -> u8 {
        let zpg_addr = self.fetch() as usize;
        let addr = self.read_address(zpg_addr) + (self.r.y as usize);
        self.room[addr]
    }

    // Read the address from memory by a given memory address
    fn read_address(&mut self, addr: usize) -> usize {
        let ll = self.room[addr] as usize;
        let hh = self.room[addr + 1] as usize;
        (hh << 8) + ll
    }

    // Set status flag
    fn flag_set(&mut self, flag: u8) {
        self.r.sr |= flag;
    }

    // Conditional set status flag
    fn flag_set_if(&mut self, flag: u8, cond: bool) {
        if cond {
            self.r.sr |= flag;
        } else {
            self.r.sr &= !flag;
        }
    }

    // Reset status flag
    fn flag_reset(&mut self, flag: u8) {
        self.r.sr &= !flag;
    }

    // Add Memory to Accumulator with Carry
    fn adc(&mut self) {
    }

    // AND Memory with Accumulator
    fn and(&mut self, value : u8) {
        self.r.a &= value;
        self.flag_set_if(status_flags::NEG, self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
    }

    // Shift Left One Bit (Memory or Accumulator)
    fn asl(&mut self, value : u8) {
        self.r.a = value << 1;

        self.flag_set_if(status_flags::CARRY, value & 0x80 != 0);
        self.flag_set_if(status_flags::NEG,  self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
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
    fn bit(&mut self, value: u8) {
        self.flag_set_if(status_flags::NEG, value & 0x80 != 0);
        self.flag_set_if(status_flags::OVER, value & 0x40 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == value);
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
        self.r.sr |= status_flags::BRK;
    }

    // Branch on Overflow Clear
    fn bvc(&mut self) {
    }

    // Branch on Overflow Set
    fn bvs(&mut self) {
    }

    // Clear Carry Flag
    fn clc(&mut self) {
        self.flag_reset(status_flags::CARRY);
    }

    // Clear Decimal Mode
    fn cld(&mut self) {
        self.flag_reset(status_flags::DEC);
    }

    // Clear Interrupt Disable Bit
    fn cli(&mut self) {
        self.flag_reset(status_flags::IRQD);
    }

    // Clear Overflow Flag
    fn clv(&mut self) {
        self.flag_reset(status_flags::OVER);
    }

    // Compare Memory with Accumulator
    fn cmp(&mut self, value: u8) {
        self.flag_set_if(status_flags::NEG, self.r.a < value);
        self.flag_set_if(status_flags::ZERO, self.r.a == value);
        self.flag_set_if(status_flags::CARRY, self.r.a >= value);
    }

    // Compare Memory and Index X
    fn cpx(&mut self, value: u8) {
        self.flag_set_if(status_flags::NEG, self.r.x < value);
        self.flag_set_if(status_flags::ZERO, self.r.x == value);
        self.flag_set_if(status_flags::CARRY, self.r.x >= value);
    }

    // Compare Memory and Index Y
    fn cpy(&mut self, value: u8) {
        self.flag_set_if(status_flags::NEG, self.r.y < value);
        self.flag_set_if(status_flags::ZERO, self.r.y == value);
        self.flag_set_if(status_flags::CARRY, self.r.y >= value);
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
    fn eor(&mut self, value : u8) {
        self.r.a ^= value;
        self.flag_set_if(status_flags::NEG, self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
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
    fn lsr(&mut self, value : u8) {
        self.r.a = value >> 1;

        self.flag_set_if(status_flags::CARRY, value & 0x01 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
        self.flag_reset(status_flags::NEG);
    }

    // No Operation
    fn nop(&mut self) {
    }

    // OR Memory with Accumulator
    fn ora(&mut self, value: u8) {
        self.r.a |= value;
        self.flag_set_if(status_flags::NEG, self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
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
    fn rol(&mut self, value: u8) {
        self.r.a = value << 1;
        if self.r.sr & status_flags::CARRY != 0 {
            self.r.a |= 0x01;
        }

        self.flag_set_if(status_flags::CARRY, value & 0x80 != 0);
        self.flag_set_if(status_flags::NEG,  self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
    }

    // Rotate One Bit Right (Memory or Accumulator)
    fn ror(&mut self, value: u8) {
        self.r.a = value >> 1;
        if self.r.sr & status_flags::CARRY != 0 {
            self.r.a |= 0x80;
        }

        self.flag_set_if(status_flags::CARRY, value & 0x01 != 0);
        self.flag_set_if(status_flags::NEG,  self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
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
        assert!(cpu.r.sr == status_flags::ZERO);
        assert!(cpu.clock == 0);
    }

    #[test]
    fn cpu_ora_neg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1010_0101;
        cpu.ora(  0b1100_0011);
        assert!(cpu.r.pc == 0);
        assert!(cpu.r.a == 0b1110_0111);
        assert!(cpu.r.sr == status_flags::NEG);
        assert!(cpu.clock == 0);
    }

    #[test]
    fn cpu_asl_noflags() {
        let mut cpu = Cpu::new();
        cpu.asl(0b0001_1000);
        assert!(cpu.r.a == 0b0011_0000);
        assert!(cpu.r.sr == 0);
    }

    #[test]
    fn cpu_asl_neg() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.asl(0b0101_0101);
        assert!(cpu.r.a == 0b1010_1010);
        assert!(cpu.r.sr == status_flags::NEG);
    }

    #[test]
    fn cpu_asl_zero_carry() {
        let mut cpu = Cpu::new();
        cpu.asl(0b1000_0000);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::CARRY);
    }

    #[test]
    fn cpu_lsr_carry() {
        let mut cpu = Cpu::new();
        cpu.lsr(0b1001_1001);
        assert!(cpu.r.a == 0b0100_1100);
        assert!(cpu.r.sr == status_flags::CARRY);
    }

    #[test]
    fn cpu_lsr_zero() {
        let mut cpu = Cpu::new();
        cpu.lsr(0b0000_0000);
        assert!(cpu.r.a == 0b0000_0000);
        assert!(cpu.r.sr == status_flags::ZERO);
    }

    #[test]
    fn cpu_lsr_noflags() {
        let mut cpu = Cpu::new();
        cpu.lsr(0b1001_1000);
        assert!(cpu.r.a == 0b0100_1100);
        assert!(cpu.r.sr == 0);
    }

    #[test]
    fn cpu_rol_set_carry() {
        let mut cpu = Cpu::new();
        cpu.rol(0b1000_0001);
        assert!(cpu.r.a == 0b0000_0010);
        assert!(cpu.r.sr == status_flags::CARRY);
    }

    #[test]
    fn cpu_rol_reset_carry() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.rol(0b0000_0001);
        assert!(cpu.r.a == 0b0000_0011);
        assert!(cpu.r.sr == 0);
    }

    #[test]
    fn cpu_rol_neg() {
        let mut cpu = Cpu::new();
        cpu.rol(0b0100_0000);
        assert!(cpu.r.a == 0b1000_0000);
        assert!(cpu.r.sr == status_flags::NEG);
    }

    #[test]
    fn cpu_rol_keep_carry() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.rol(0b1000_0000);
        print!("{:?}", cpu);
        assert!(cpu.r.a == 0b0000_0001);
        assert!(cpu.r.sr == status_flags::CARRY);
    }

    #[test]
    fn cpu_rol_zero() {
        let mut cpu = Cpu::new();
        cpu.rol(0b0000_0000);
        assert!(cpu.r.a == 0b0000_0000);
        assert!(cpu.r.sr == status_flags::ZERO);
    }

    #[test]
    fn cpu_ror_set_carry() {
        let mut cpu = Cpu::new();
        cpu.ror(0b1000_0001);
        assert!(cpu.r.a == 0b0100_0000);
        assert!(cpu.r.sr == status_flags::CARRY);
    }

    #[test]
    fn cpu_ror_reset_carry() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.ror(0b0000_0000);
        assert!(cpu.r.a == 0b1000_0000);
        assert!(cpu.r.sr == status_flags::NEG);
    }

    #[test]
    fn cpu_ror_keep_carry() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.ror(0b1000_0000);
        print!("{:?}", cpu);
        assert!(cpu.r.a == 0b1100_0000);
        assert!(cpu.r.sr == status_flags::NEG);
    }

    #[test]
    fn cpu_ror_zero() {
        let mut cpu = Cpu::new();
        cpu.ror(0b0000_0000);
        assert!(cpu.r.a == 0b0000_0000);
        assert!(cpu.r.sr == status_flags::ZERO);
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
    fn cpu_and_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1010_0101;
        cpu.and(0b0010_0100);
        assert!(cpu.r.a == 0b0010_0100);
        assert!(cpu.r.sr == 0);
    }

    #[test]
    fn cpu_and_neg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1010_0101;
        cpu.and(0b1010_0100);
        assert!(cpu.r.a == 0b1010_0100);
        assert!(cpu.r.sr == status_flags::NEG);
    }

    #[test]
    fn cpu_and_zero() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1111_1111;
        cpu.and(0b0000_0000);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO);
    }

    #[test]
    fn cpu_eor_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1010_0101;
        cpu.eor(0b1100_1100);
        assert!(cpu.r.a == 0b0110_1001);
        assert!(cpu.r.sr == 0);
    }

    #[test]
    fn cpu_eor_neg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1111_1111;
        cpu.eor(0b0110_1001);
        assert!(cpu.r.a == 0b1001_0110);
        assert!(cpu.r.sr == status_flags::NEG);
    }

    #[test]
    fn cpu_eor_zero() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1111_1111;
        cpu.eor(0b01111_1111);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO);
    }

    #[test]
    fn cpu_cmp_less() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0;
        cpu.cmp(1);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::NEG);
    }

    #[test]
    fn cpu_cmp_equal() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0;
        cpu.cmp(0);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == (status_flags::ZERO | status_flags::CARRY));
    }

    #[test]
    fn cpu_cmp_greater() {
        let mut cpu = Cpu::new();
        cpu.r.a = 1;
        cpu.cmp(0);
        assert!(cpu.r.a == 1);
        assert!(cpu.r.sr == status_flags::CARRY);
    }

    #[test]
    fn cpu_cpx_less() {
        let mut cpu = Cpu::new();
        cpu.r.x = 0;
        cpu.cpx(1);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.sr == status_flags::NEG);
    }

    #[test]
    fn cpu_cpx_equal() {
        let mut cpu = Cpu::new();
        cpu.r.x = 0;
        cpu.cpx(0);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.sr == (status_flags::ZERO | status_flags::CARRY));
    }

    #[test]
    fn cpu_cpx_greater() {
        let mut cpu = Cpu::new();
        cpu.r.x = 1;
        cpu.cpx(0);
        assert!(cpu.r.x == 1);
        assert!(cpu.r.sr == status_flags::CARRY);
    }

    #[test]
    fn cpu_cpy_less() {
        let mut cpu = Cpu::new();
        cpu.r.y = 0;
        cpu.cpy(1);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::NEG);
    }

    #[test]
    fn cpu_cpy_equal() {
        let mut cpu = Cpu::new();
        cpu.r.y = 0;
        cpu.cpy(0);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == (status_flags::ZERO | status_flags::CARRY));
    }

    #[test]
    fn cpu_cpy_greater() {
        let mut cpu = Cpu::new();
        cpu.r.y = 1;
        cpu.cpy(0);
        assert!(cpu.r.y == 1);
        assert!(cpu.r.sr == status_flags::CARRY);
    }

    #[test]
    fn cpu_bit_equal() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1000_0000;
        cpu.bit(0b1000_0000);
        assert!(cpu.r.a == 0b1000_0000);
        assert!(cpu.r.sr == (status_flags::NEG | status_flags::ZERO));
    }

    #[test]
    fn cpu_bit_flags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0;
        cpu.bit(0b1100_0000);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == (status_flags::NEG | status_flags::OVER));
    }

    #[test]
    fn cpu_clc() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY | status_flags::UNUSED;
        cpu.clc();
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_cld() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::DEC | status_flags::UNUSED;
        cpu.cld();
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_cli() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::IRQD | status_flags::UNUSED;
        cpu.cli();
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_clv() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::OVER | status_flags::UNUSED;
        cpu.clv();
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_instruction_brk() {
        let mut cpu = Cpu::new();
        cpu.load([0x00, 0x00, 0x00].to_vec());
        cpu.step();
        assert!(cpu.r.pc == 0x02);
        assert!(cpu.r.sr == status_flags::BRK);
        assert!(cpu.clock == 7);
    }

    #[test]
    fn cpu_instruction_ora_immidiate() {
        let mut cpu = Cpu::new();
        cpu.load([0x09, 0x00].to_vec());
        cpu.step();
        assert!(cpu.r.pc == 0x02);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO);
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
        assert!(cpu.r.sr == status_flags::NEG);
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
        assert!(cpu.r.sr == status_flags::NEG);
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