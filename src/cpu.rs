use std::fmt;

use crate::mmu::Mmu;

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
        Registers { a: 0, x: 0, y: 0, sr: status_flags::UNUSED, sp: 0, pc: 0xf000 }
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
    pub clock : u64,
}

impl fmt::Debug for Cpu {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "r:({:?}) clock:{:?}", self.r, self.clock)
    }
}

#[allow(dead_code)]
impl Cpu {
    pub fn new() -> Cpu {
        Cpu { r: Registers::new(), clock: 0 }
    }

    pub fn step(&mut self, mmu: &mut Mmu) {
        let opcode = mmu.read(self.r.pc);
        self.r.pc = self.r.pc.wrapping_add(1);

        let elapsed = match opcode {
            // BRK impl
            0x00 => {
                self.brk(mmu);
                7
            }

            // ORA X,ind
            0x01 => {
                let value = self.fetch_x_ind(mmu);
                self.ora(value);
                6
            }

            // ORA zpg
            0x05 => {
                let value = self.fetch_zpg(mmu);
                self.ora(value);
                3
            }

            // ASL zpg
            0x06 => {
                let value = self.fetch_zpg(mmu);
                self.asl(value);
                5
            }

            // PHP
            0x08 => {
                self.php(mmu);
                3
            }

            // ORA #
            0x09 => {
                let value = self.fetch(mmu);
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
                let value = self.fetch_abs(mmu);
                self.ora(value);
                4
            }

            // ASL abs
            0x0e => {
                let value = self.fetch_abs(mmu);
                self.asl(value);
                6
            }

            // BPL
            0x10 => {
                self.bpl(mmu);
                2
            }

            // ORA ind,Y
            0x11 => {
                let value = self.fetch_ind_y(mmu);
                self.ora(value);
                5 // TODO: Increment by one if page boundary crossed
            }

            // ORA zpg,X
            0x15 => {
                let value = self.fetch_zpg_x(mmu);
                self.ora(value);
                4
            }

            // ASL zpg,X
            0x16 => {
                let value = self.fetch_zpg_x(mmu);
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
                let value = self.fetch_abs_y(mmu);
                self.ora(value);
                4 // TODO: Increment by one if page boundary crossed
            }

            // ORA abs,X
            0x1d => {
                let value = self.fetch_abs_x(mmu);
                self.ora(value);
                4 // TODO: Increment by one if page boundary crossed
            }


            // ASL abs,X
            0x1e => {
                let value = self.fetch_abs_x(mmu);
                self.asl(value);
                7
            }

            // JSR
            0x20 => {
                self.jsr(mmu);
                6
            }

            // AND X,ind
            0x21 => {
                let value = self.fetch_x_ind(mmu);
                self.and(value);
                6
            }

            // BIT zpg
            0x24 => {
                let value = self.fetch_zpg(mmu);
                self.bit(value);
                3
            }

            // AND zpg
            0x25 => {
                let value = self.fetch_zpg(mmu);
                self.and(value);
                3
            }

            // ROL zpg
            0x26 => {
                let value = self.fetch_zpg(mmu);
                self.rol(value);
                5
            }

            // PLP
            0x28 => {
                self.plp(mmu);
                4
            }

            // AND #
            0x29 => {
                let value = self.fetch(mmu);
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
                let value = self.fetch_abs(mmu);
                self.bit(value);
                4
            }

            // AND abs
            0x2d => {
                let value = self.fetch_abs(mmu);
                self.and(value);
                4
            }

            // ROL abs
            0x2e => {
                let value = self.fetch_abs(mmu);
                self.rol(value);
                6
            }

            // BMI
            0x30 => {
                self.bmi(mmu);
                2
            }

            // AND ind,Y
            0x31 => {
                let value = self.fetch_ind_y(mmu);
                self.and(value);
                5
            }

            // AND zpg,X
            0x35 => {
                let value = self.fetch_zpg_x(mmu);
                self.and(value);
                4
            }

            // ROL zpg,X
            0x36 => {
                let value = self.fetch_zpg_x(mmu);
                self.rol(value);
                6
            }

            // SEC
            0x38 => {
                self.sec();
                2
            }

            // AND abs,Y
            0x39 => {
                let value = self.fetch_abs_y(mmu);
                self.and(value);
                4
            }

            // AND abs,X
            0x3d => {
                let value = self.fetch_abs_x(mmu);
                self.and(value);
                4
            }

            // ROL abs,X
            0x3e => {
                let value = self.fetch_abs_x(mmu);
                self.rol(value);
                7
            }

            // RTI
            0x40 => {
                self.rti(mmu);
                6
            }

            // EOR X,ind
            0x41 => {
                let value = self.fetch_x_ind(mmu);
                self.eor(value);
                6
            }

            // EOR zpg
            0x45 => {
                let value = self.fetch_zpg(mmu);
                self.eor(value);
                3
            }

            // LSR zpg
            0x46 => {
                let value = self.fetch_zpg(mmu);
                self.lsr(value);
                5
            }

            // PHA
            0x48 => {
                self.pha(mmu);
                3
            }

            // EOR #
            0x49 => {
                let value = self.fetch(mmu);
                self.eor(value);
                2
            }

            // LSR A
            0x4a => {
                let value = self.r.a;
                self.lsr(value);
                2
            }

            // JMP abs
            0x4c => {
                let addr = self.fetch_abs_address(mmu);
                self.jmp(addr);
                3
            }

            // EOR abs
            0x4d => {
                let value = self.fetch_abs(mmu);
                self.eor(value);
                4
            }

            // LSR abs
            0x4e => {
                let value = self.fetch_abs(mmu);
                self.lsr(value);
                6
            }

            // BVC
            0x50 => {
                self.bvc(mmu);
                2
            }

            // EOR ind,Y
            0x51 => {
                let value = self.fetch_ind_y(mmu);
                self.eor(value);
                5
            }

            // EOR zpg,X
            0x55 => {
                let value = self.fetch_zpg_x(mmu);
                self.eor(value);
                4
            }

            // LSR zpg,X
            0x56 => {
                let value = self.fetch_zpg_x(mmu);
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
                let value = self.fetch_abs_y(mmu);
                self.eor(value);
                4
            }

            // EOR abs,X
            0x5d => {
                let value = self.fetch_abs_x(mmu);
                self.eor(value);
                4
            }

            // LSR abs,X
            0x5e => {
                let value = self.fetch_abs_x(mmu);
                self.lsr(value);
                7
            }

            // RTS
            0x60 => {
                self.rts(mmu);
                6
            }

            // ADC X,ind
            0x61 => {
                let value = self.fetch_x_ind(mmu);
                self.adc(value);
                6
            }

            // ADC zpg
            0x65 => {
                let value = self.fetch_zpg(mmu);
                self.adc(value);
                3
            }

            // ROR zpg
            0x66 => {
                let value = self.fetch_zpg(mmu);
                self.ror(value);
                5
            }

            // PLA
            0x68 => {
                self.pla(mmu);
                4
            }

            // ADC #
            0x69 => {
                let value = self.fetch(mmu);
                self.adc(value);
                2
            }

            // ROR A
            0x6a => {
                let value = self.r.a;
                self.ror(value);
                2
            }

            // JMP ind
            0x6c => {
                let ind_addr = self.fetch_abs_address(mmu);
                let addr = self.read_address(mmu, ind_addr);
                self.jmp(addr);
                5
            }

            // ADC abs
            0x6d => {
                let value = self.fetch_abs(mmu);
                self.adc(value);
                4
            }

            // ROR abs
            0x6e => {
                let value = self.fetch_abs(mmu);
                self.ror(value);
                6
            }

            // BVS
            0x70 => {
                self.bvs(mmu);
                2
            }

            // ADC ind,Y
            0x71 => {
                let value = self.fetch_ind_y(mmu);
                self.adc(value);
                5
            }

            // ADC zpg,X
            0x75 => {
                let value = self.fetch_zpg_x(mmu);
                self.adc(value);
                4
            }

            // ROR zpg,X
            0x76 => {
                let value = self.fetch_zpg_x(mmu);
                self.ror(value);
                6
            }

            // SEI
            0x78 => {
                self.sei();
                2
            }

            // ADC abs,Y
            0x79 => {
                let value = self.fetch_abs_y(mmu);
                self.adc(value);
                4
            }

            // ADC abs,X
            0x7d => {
                let value = self.fetch_abs_x(mmu);
                self.adc(value);
                4
            }

            // ROR abs,X
            0x7e => {
                let value = self.fetch_abs_x(mmu);
                self.ror(value);
                7
            }

            // STA X,ind
            0x81 => {
                let addr = self.fetch_x_ind_address(mmu);
                self.sta(mmu, addr);
                6
            }

            // STY zpg
            0x84 => {
                let addr = self.fetch_zpg_address(mmu);
                self.sty(mmu, addr);
                3
            }

            // STA zpg
            0x85 => {
                let addr = self.fetch_zpg_address(mmu);
                self.sta(mmu, addr);
                3
            }

            // STX zpg
            0x86 => {
                let addr = self.fetch_zpg_address(mmu);
                self.stx(mmu, addr);
                3
            }

            // DEY
            0x88 => {
                self.dey();
                2
            }

            // TXA
            0x8a => {
                self.txa();
                2
            }

            // STY abs
            0x8c => {
                let addr = self.fetch_abs_address(mmu);
                self.sty(mmu, addr);
                4
            }

            // STA abs
            0x8d => {
                let addr = self.fetch_abs_address(mmu);
                self.sta(mmu, addr);
                4
            }

            // STX abs
            0x8e => {
                let addr = self.fetch_abs_address(mmu);
                self.stx(mmu, addr);
                4
            }

            // BCC
            0x90 => {
                self.bcc(mmu);
                2
            }

            // STA ind,Y
            0x91 => {
                let addr = self.fetch_ind_y_address(mmu);
                self.sta(mmu, addr);
                6
            }

            // STY zpg,X
            0x94 => {
                let addr = self.fetch_zpg_x_address(mmu);
                self.sty(mmu, addr);
                4
            }

            // STA zpg,X
            0x95 => {
                let addr = self.fetch_zpg_x_address(mmu);
                self.sta(mmu, addr);
                4
            }

            // STX zpg,Y
            0x96 => {
                let addr = self.fetch_zpg_y_address(mmu);
                self.stx(mmu, addr);
                4
            }

            // TYA
            0x98 => {
                self.tya();
                2
            }

            // STA abs,Y
            0x99 => {
                let addr = self.fetch_abs_y_address(mmu);
                self.sta(mmu, addr);
                5
            }

            // TXS
            0x9a => {
                self.txs();
                2
            }

            // STA abs,X
            0x9d => {
                let addr = self.fetch_abs_x_address(mmu);
                self.sta(mmu, addr);
                5
            }

            // LDY #
            0xa0 => {
                let value = self.fetch(mmu);
                self.ldy(value);
                2
            }

            // LDA X,ind
            0xa1 => {
                let value = self.fetch_x_ind(mmu);
                self.lda(value);
                6
            }

            // LDX #
            0xa2 => {
                let value = self.fetch(mmu);
                self.ldx(value);
                2
            }

            // LDY zpg
            0xa4 => {
                let value = self.fetch_zpg(mmu);
                self.ldy(value);
                3
            }

            // LDA zpg
            0xa5 => {
                let value = self.fetch_zpg(mmu);
                self.lda(value);
                3
            }

            // LDX zpg
            0xa6 => {
                let value = self.fetch_zpg(mmu);
                self.ldx(value);
                3
            }

            // TAY
            0xa8 => {
                self.tay();
                2
            }

            // LDA #
            0xa9 => {
                let value = self.fetch(mmu);
                self.lda(value);
                2
            }

            // TAX
            0xaa => {
                self.tax();
                2
            }

            // LDY abs
            0xac => {
                let value = self.fetch_abs(mmu);
                self.ldy(value);
                4
            }

            // LDA abs
            0xad => {
                let value = self.fetch_abs(mmu);
                self.lda(value);
                4
            }

            // LDX abs
            0xae => {
                let value = self.fetch_abs(mmu);
                self.ldx(value);
                4
            }

            // BCS
            0xb0 => {
                self.bcs(mmu);
                2
            }

            // LDA ind,Y
            0xb1 => {
                let value = self.fetch_ind_y(mmu);
                self.lda(value);
                5
            }

            // LDY zpg,X
            0xb4 => {
                let value = self.fetch_zpg_x(mmu);
                self.ldy(value);
                4
            }

            // LDA zpg,X
            0xb5 => {
                let value = self.fetch_zpg_x(mmu);
                self.lda(value);
                4
            }

            // LDX zpg,y
            0xb6 => {
                let value = self.fetch_zpg_y(mmu);
                self.ldx(value);
                4
            }

            // CLV
            0xb8 => {
                self.clv();
                2
            }

            // LDA abs,Y
            0xb9 => {
                let value = self.fetch_abs_y(mmu);
                self.lda(value);
                4
            }

            // TSX
            0xba => {
                self.tsx();
                2
            }

            // LDY abs,X
            0xbc => {
                let value = self.fetch_abs_x(mmu);
                self.ldy(value);
                4
            }

            // LDA abs,X
            0xbd => {
                let value = self.fetch_abs_x(mmu);
                self.lda(value);
                4
            }

            // LDX abs,Y
            0xbe => {
                let value = self.fetch_abs_y(mmu);
                self.ldx(value);
                4
            }

            // CPY #
            0xc0 => {
                let value = self.fetch(mmu);
                self.cpy(value);
                2
            }

            // CMP X,ind
            0xc1 => {
                let value = self.fetch_x_ind(mmu);
                self.cmp(value);
                6
            }

            // CPY zpg
            0xc4 => {
                let value = self.fetch_zpg(mmu);
                self.cpy(value);
                3
            }

            // CMP zpg
            0xc5 => {
                let value = self.fetch_zpg(mmu);
                self.cmp(value);
                3
            }

            // DEC zpg
            0xc6 => {
                let addr = self.fetch_zpg_address(mmu);
                let value = self.dec(mmu.read(addr));
                mmu.write(addr, value);
                5
            }

            // INY
            0xc8 => {
                self.iny();
                2
            }

            // CMP #
            0xc9 => {
                let value = self.fetch(mmu);
                self.cmp(value);
                2
            }

            // DEX
            0xca => {
                self.dex();
                2
            }

            // CPY abs
            0xcc => {
                let value = self.fetch_abs(mmu);
                self.cpy(value);
                4
            }

            // CMP abs
            0xcd => {
                let value = self.fetch_abs(mmu);
                self.cmp(value);
                4
            }

            // DEC abs
            0xce => {
                let addr = self.fetch_abs_address(mmu);
                let value = self.dec(mmu.read(addr));
                mmu.write(addr, value);
                3
            }

            // BNE
            0xd0 => {
                self.bne(mmu);
                2
            }

            // CMP ind,Y
            0xd1 => {
                let value = self.fetch_ind_y(mmu);
                self.cmp(value);
                5
            }

            // CMP zpg,X
            0xd5 => {
                let value = self.fetch_zpg_x(mmu);
                self.cmp(value);
                4
            }

            // DEC zpg,X
            0xd6 => {
                let addr = self.fetch_zpg_x_address(mmu);
                let value = self.dec(mmu.read(addr));
                mmu.write(addr, value);
                6
            }

            // CLD
            0xd8 => {
                self.cld();
                2
            }

            // CMP abs,Y
            0xd9 => {
                let value = self.fetch_abs_y(mmu);
                self.cmp(value);
                4
            }

            // CMP abs,X
            0xdd => {
                let value = self.fetch_abs_x(mmu);
                self.cmp(value);
                4
            }

            // DEC abs,X
            0xde => {
                let addr = self.fetch_abs_x_address(mmu);
                let value = self.dec(mmu.read(addr));
                mmu.write(addr, value);
                7
            }

            // CPX #
            0xe0 => {
                let value = self.fetch(mmu);
                self.cpx(value);
                2
            }

            // SBC X,ind
            0xe1 => {
                let value = self.fetch_x_ind(mmu);
                self.sbc(value);
                6
            }

            // CPX zpg
            0xe4 => {
                let value = self.fetch_zpg(mmu);
                self.cpx(value);
                3
            }

            // SBC zpg
            0xe5 => {
                let value = self.fetch_zpg(mmu);
                self.sbc(value);
                3
            }

            // INC zpg
            0xe6 => {
                let addr = self.fetch_zpg_address(mmu);
                let value = self.inc(mmu.read(addr));
                mmu.write(addr, value);
                5
            }

            // INX
            0xe8 => {
                self.inx();
                2
            }

            // SBC #
            0xe9 => {
                let value = self.fetch(mmu);
                self.sbc(value);
                2
            }

            // NOP
            0xea => {
                self.nop();
                2
            }

            // CPX abs
            0xec => {
                let value = self.fetch_abs(mmu);
                self.cpx(value);
                4
            }

            // INC abs
            0xee => {
                let addr = self.fetch_abs_address(mmu);
                let value = self.inc(mmu.read(addr));
                mmu.write(addr, value);
                6
            }

            // SBC abs
            0xed => {
                let value = self.fetch_abs(mmu);
                self.sbc(value);
                4
            }

            // BEQ
            0xf0 => {
                self.beq(mmu);
                2
            }

            // SBC ind,Y
            0xf1 => {
                let value = self.fetch_ind_y(mmu);
                self.sbc(value);
                5
            }

            // SBC zpg,X
            0xf5 => {
                let value = self.fetch_zpg_x(mmu);
                self.sbc(value);
                4
            }

            // INC zpg,X
            0xf6 => {
                let addr = self.fetch_zpg_x_address(mmu);
                let value = self.inc(mmu.read(addr));
                mmu.write(addr, value);
                6
            }

            // SED
            0xf8 => {
                self.sed();
                2
            }

            // SBC abs,Y
            0xf9 => {
                let value = self.fetch_abs_y(mmu);
                self.sbc(value);
                4
            }

            // SBC abs,X
            0xfd => {
                let value = self.fetch_abs_x(mmu);
                self.sbc(value);
                4
            }

            // INC abs,X
            0xfe => {
                let addr = self.fetch_abs_x_address(mmu);
                let value = self.inc(mmu.read(addr));
                mmu.write(addr, value);
                7
            }

            _ => panic!("opcode {:x} not implemented yet!", opcode)
        };

        self.clock += elapsed;
    }

    /// Fetchs the next byte indexed by pc register and increment pc by one
    fn fetch(&mut self, mmu: &Mmu) -> u8 {
        let b = mmu.read(self.r.pc);
        self.r.pc = self.r.pc.wrapping_add(1);
        b
    }

    // Fetchs the immediate address indexed by pc register and increment pc by two
    fn fetch_address(&mut self, mmu: &Mmu) -> u16 {
        let ll = self.fetch(mmu) as u16;
        let hh = self.fetch(mmu) as u16;
        (hh << 8) + ll
    }

    // Fetchs the absolute address from the program and resolve his memory value
    fn fetch_abs(&mut self, mmu: &Mmu) -> u8 {
        let addr = self.fetch_address(mmu);
        mmu.read(addr)
    }

    // Fetchs the absolute address + x from the program and resolve his memory value
    fn fetch_abs_x(&mut self, mmu: &Mmu) -> u8 {
        let addr = self.fetch_abs_x_address(mmu);
        mmu.read(addr)
    }

    // Fetchs the absolute address + y from the program and resolve his memory value
    fn fetch_abs_y(&mut self, mmu: &Mmu) -> u8 {
        let addr = self.fetch_abs_y_address(mmu);
        mmu.read(addr)
    }

    // Fetch the absolute address
    fn fetch_abs_address(&mut self, mmu: &Mmu) -> u16 {
        self.fetch_address(mmu)
    }

    // Fetch the absolute address + x
    fn fetch_abs_x_address(&mut self, mmu: &Mmu) -> u16 {
        let addr = self.fetch_address(mmu);
        let offset = self.r.x as u16;

        if addr & 0xff + offset > 0xff {
            self.clock += 1;
        }

        addr + offset
    }

    // Fetch the absolute address + y
    fn fetch_abs_y_address(&mut self, mmu: &Mmu) -> u16 {
        let addr = self.fetch_address(mmu);
        let offset = self.r.y as u16;

        if addr & 0xff + offset > 0xff {
            self.clock += 1;
        }

        addr + offset
    }

    // Fetchs the zeropage address from the program and resolve his memory value
    fn fetch_zpg(&mut self, mmu: &Mmu) -> u8 {
        let addr = self.fetch_zpg_address(mmu);
        mmu.read(addr)
    }

    // Fetchs the zeropage address + x from the program and resolve his memory value
    fn fetch_zpg_x(&mut self, mmu: &Mmu) -> u8 {
        let addr = self.fetch_zpg_x_address(mmu);
        mmu.read(addr)
    }

    // Fetchs the zeropage address + y from the program and resolve his memory value
    fn fetch_zpg_y(&mut self, mmu: &Mmu) -> u8 {
        let addr = self.fetch_zpg_y_address(mmu);
        mmu.read(addr)
    }

    // Fetch zeropage address
    fn fetch_zpg_address(&mut self, mmu: &Mmu) -> u16 {
        self.fetch(mmu) as u16
    }

    // Fetch zeropage address + x from the program
    fn fetch_zpg_x_address(&mut self, mmu: &Mmu) -> u16 {
        self.fetch_zpg_address(mmu).wrapping_add(self.r.x as u16)
    }

    // Fetch zeropage address + y from the program
    fn fetch_zpg_y_address(&mut self, mmu: &Mmu) -> u16 {
        self.fetch_zpg_address(mmu).wrapping_add(self.r.y as u16)
    }

    // Fetchs an indirect address in zero page + x and resolve his memory value
    fn fetch_x_ind(&mut self, mmu: &Mmu) -> u8 {
        let addr = self.fetch_x_ind_address(mmu);
        mmu.read(addr)
    }

    // Fetchs an indirect address in zero page + x
    fn fetch_x_ind_address(&mut self, mmu: &Mmu) -> u16 {
        let zpg_addr = self.fetch_zpg_address(mmu).wrapping_add(self.r.x as u16);
        self.read_address(mmu, zpg_addr & 0xff)
    }

    // Fetchs an indirect address in zero page, then increment by y and resolve his memory value
    fn fetch_ind_y(&mut self, mmu: &Mmu) -> u8 {
        let zpg_addr = self.fetch(mmu) as u16;
        let addr = self.read_address(mmu, zpg_addr).wrapping_add(self.r.y as u16);
        mmu.read(addr)
    }

    // Fetchs an indirect address in zero page, then increment by y
    fn fetch_ind_y_address(&mut self, mmu: &Mmu) -> u16 {
        let zpg_addr = self.fetch(mmu) as u16;
        let addr = self.read_address(mmu, zpg_addr);
        let offset = self.r.y as u16;

        if (addr & 0xff) + offset > 0xff {
            self.clock += 1;
        }

        addr + offset
    }

    // Read the address from memory by a given memory address
    fn read_address(&mut self, mmu: &Mmu, addr: u16) -> u16 {
        let ll = mmu.read(addr) as u16;
        let hh = mmu.read(addr + 1) as u16;
        (hh << 8) + ll
    }

    // Set status flag
    fn flag_set(&mut self, flag: u8) {
        self.r.sr |= flag;
        self.r.sr |= status_flags::UNUSED;
    }

    // Conditional set status flag
    fn flag_set_if(&mut self, flag: u8, cond: bool) {
        if cond {
            self.r.sr |= flag;
        } else {
            self.r.sr &= !flag;
        }
        self.r.sr |= status_flags::UNUSED;
    }

    // Reset status flag
    fn flag_reset(&mut self, flag: u8) {
        self.r.sr &= !flag;
        self.r.sr |= status_flags::UNUSED;
    }

    // Push on Stack
    fn stack_push(&mut self, mmu: &mut Mmu, value: u8) {
        mmu.write(0x100 + self.r.sp as u16, value);
        self.r.sp = self.r.sp.wrapping_sub(1);
    }

    // Pull from Stack
    fn stack_pull(&mut self, mmu: &Mmu) -> u8 {
        self.r.sp = self.r.sp.wrapping_add(1);
        mmu.read(0x100 + self.r.sp as u16)
    }

    // Add Memory to Accumulator with Carry
    fn adc(&mut self, value: u8) {
        let mut result: u16 = self.r.a as u16 + value as u16;

        if self.r.sr & status_flags::CARRY != 0 {
            result += 1;
        }

        if self.r.sr & status_flags::DEC != 0 {
            // BCD Mode
            if result & 0x0f > 0x09 {
                result += 0x06;
            }

            if result & 0xf0 > 0x90 {
                result += 0x60;
            }
        }

        self.r.a = result as u8;
        self.flag_set_if(status_flags::NEG, self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
        self.flag_set_if(status_flags::CARRY, result > 0xff);
        self.flag_set_if(status_flags::OVER, result >= 128);
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

    // Branch If Condition is Set
    fn branch_if(&mut self, mmu: &Mmu, cond: bool) {
        let offset = self.fetch(mmu) as u16;

        if cond {
            self.clock += 1;

            if ((self.r.pc & 0xff) + offset) > 0xff {
                self.clock += 1;
            }

            self.r.pc = self.r.pc.wrapping_add(offset);
        }
    }

    // Branch on Carry Clear
    fn bcc(&mut self, mmu: &Mmu) {
        self.branch_if(mmu, self.r.sr & status_flags::CARRY == 0);
    }

    // Branch on Carry Set
    fn bcs(&mut self, mmu: &Mmu) {
        self.branch_if(mmu, self.r.sr & status_flags::CARRY != 0);
    }

    // Branch on Result Zero
    fn beq(&mut self, mmu: &Mmu) {
        self.branch_if(mmu, self.r.sr & status_flags::ZERO != 0);
    }

    // Test Bits in Memory with Accumulator
    fn bit(&mut self, value: u8) {
        self.flag_set_if(status_flags::NEG, value & 0x80 != 0);
        self.flag_set_if(status_flags::OVER, value & 0x40 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == value);
    }

    // Branch on Result Minus
    fn bmi(&mut self, mmu: &Mmu) {
        self.branch_if(mmu, self.r.sr & status_flags::NEG != 0);
    }

    // Branch on Result not Zero
    fn bne(&mut self, mmu: &Mmu) {
        self.branch_if(mmu, self.r.sr & status_flags::ZERO == 0);
    }

    // Branch on Result Plus
    fn bpl(&mut self, mmu: &Mmu) {
        self.branch_if(mmu, self.r.sr & status_flags::NEG == 0);
    }

    // Force Break
    fn brk(&mut self, mmu: &mut Mmu) {
        self.r.pc = self.r.pc.wrapping_add(1);
        self.flag_set(status_flags::BRK);

        let ret_addr = self.r.pc.wrapping_sub(1);
        let ll = ret_addr as u8;
        let hh = (ret_addr > 8) as u8;

        self.stack_push(mmu, hh);
        self.stack_push(mmu, ll);
        self.stack_push(mmu, self.r.sr);

        self.r.pc = self.read_address(mmu, 0xfffe) as u16;
    }

    // Branch on Overflow Clear
    fn bvc(&mut self, mmu: &Mmu) {
        self.branch_if(mmu, self.r.sr & status_flags::OVER == 0);
    }

    // Branch on Overflow Set
    fn bvs(&mut self, mmu: &Mmu) {
        self.branch_if(mmu, self.r.sr & status_flags::OVER != 0);
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
    fn dec(&mut self, value: u8) -> u8 {
        let result = value.wrapping_sub(1);
        self.flag_set_if(status_flags::NEG, result & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, result == 0);
        result
    }

    // Decrement Index X by One
    fn dex(&mut self) {
        self.r.x = self.dec(self.r.x);
    }

    // Decrement Index Y by One
    fn dey(&mut self) {
        self.r.y = self.dec(self.r.y);
    }

    // Exclusive-OR Memory with Accumulator
    fn eor(&mut self, value : u8) {
        self.r.a ^= value;
        self.flag_set_if(status_flags::NEG, self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
    }

    // Increment Memory by One
    fn inc(&mut self, value: u8) -> u8 {
        let result = value.wrapping_add(1);
        self.flag_set_if(status_flags::NEG, result & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, result == 0);
        result
    }

    // Increment Index X by One
    fn inx(&mut self) {
        self.r.x = self.inc(self.r.x);
    }

    // Increment Index Y by One
    fn iny(&mut self) {
        self.r.y = self.inc(self.r.y);
    }

    // Jump to New Location
    fn jmp(&mut self, addr: u16) {
        self.r.pc = addr as u16;
    }

    // Jump to New Location Saving Return Address
    fn jsr(&mut self, mmu: &mut Mmu) {
        let jmp_addr = self.fetch_abs_address(mmu);

        let ret_addr = self.r.pc.wrapping_sub(1);
        let ll = ret_addr as u8;
        let hh = (ret_addr >> 8) as u8;

        self.stack_push(mmu, hh);
        self.stack_push(mmu, ll);

        self.r.pc = jmp_addr as u16;
    }

    // Load Accumulator with Memory
    fn lda(&mut self, value: u8) {
        self.r.a = value;
    }

    // Load Index X with Memory
    fn ldx(&mut self, value: u8) {
        self.r.x = value;
    }

    // Load Index Y with Memory
    fn ldy(&mut self, value: u8) {
        self.r.y = value;
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
    fn pha(&mut self, mmu: &mut Mmu) {
        self.stack_push(mmu, self.r.a);
    }

    // Push Processor Status on Stack
    fn php(&mut self, mmu: &mut Mmu) {
        self.stack_push(mmu, self.r.sr);
    }

    // Pull Accumulator from Stack
    fn pla(&mut self, mmu: &Mmu) {
        self.r.a = self.stack_pull(mmu);

        self.flag_set_if(status_flags::NEG,  self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
    }

    // Pull Processor Status from Stack
    fn plp(&mut self, mmu: &Mmu) {
        self.r.sr = self.stack_pull(mmu) | status_flags::UNUSED;
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
    fn rti(&mut self, mmu: &Mmu) {
        self.r.sr = self.stack_pull(mmu) | status_flags::UNUSED;

        let ll = self.stack_pull(mmu) as u16;
        let hh = self.stack_pull(mmu) as u16;
        self.r.pc = (hh << 8) + ll;
    }

    // Return from Subroutine
    fn rts(&mut self, mmu: &Mmu) {
        let ll = self.stack_pull(mmu) as u16;
        let hh = self.stack_pull(mmu) as u16;

        self.r.pc = (hh << 8) + ll;
        self.r.pc = self.r.pc.wrapping_add(1);
    }

    // Subtract Memory from Accumulator with Borrow
    fn sbc(&mut self, value: u8) {
        let mut result: i16 = self.r.a as i16 - value as i16;

        if self.r.sr & status_flags::CARRY != 0 {
            result -= 1;
        }

        if self.r.sr & status_flags::DEC != 0 {
            // BCD Mode
            if result & 0xf0 > 0x90 {
                result -= 0x60;
            }

            if result & 0x0f > 0x09 {
                result -= 0x06;
            }
        }

        self.r.a = result as u8;
        self.flag_set_if(status_flags::NEG, self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
        self.flag_set_if(status_flags::CARRY, result < 0);
        self.flag_set_if(status_flags::OVER, result < -128 || result >= 128);
    }

    // Set Carry Flag
    fn sec(&mut self) {
        self.flag_set(status_flags::CARRY);
    }

    // Set Decimal Flag
    fn sed(&mut self) {
        self.flag_set(status_flags::DEC);
    }

    // Set Interrupt Disable Status
    fn sei(&mut self) {
        self.flag_set(status_flags::IRQD);
    }

    // Store Accumulator in Memory
    fn sta(&mut self, mmu: &mut Mmu, addr: u16) {
        let value = self.r.a;
        mmu.write(addr, value);
    }

    // Store Index X in Memory
    fn stx(&mut self, mmu: &mut Mmu, addr: u16) {
        let value = self.r.x;
        mmu.write(addr, value);
    }

    // Store Index Y in Memory
    fn sty(&mut self, mmu: &mut Mmu, addr: u16) {
        let value = self.r.y;
        mmu.write(addr, value);
    }

    // Transfer Accumulator to Index X
    fn tax(&mut self) {
        self.r.x = self.r.a;
        self.flag_set_if(status_flags::NEG, self.r.x & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.x == 0);
    }

    // Transfer Accumulator to Index Y
    fn tay(&mut self) {
        self.r.y = self.r.a;
        self.flag_set_if(status_flags::NEG, self.r.y & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.y == 0);
    }

    // Transfer Stack Pointer to Index X
    fn tsx(&mut self) {
        self.r.x = self.r.sp;
        self.flag_set_if(status_flags::NEG, self.r.x & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.x == 0);
    }

    // Transfer Index X to Accumulator
    fn txa(&mut self) {
        self.r.a = self.r.x;
        self.flag_set_if(status_flags::NEG, self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
    }

    // Transfer Index X to Stack Pointer
    fn txs(&mut self) {
        self.r.sp = self.r.x;
    }

    // Transfer Index Y to Accumulator
    fn tya(&mut self) {
        self.r.a = self.r.y;
        self.flag_set_if(status_flags::NEG, self.r.a & 0x80 != 0);
        self.flag_set_if(status_flags::ZERO, self.r.a == 0);
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_fetch_sequence() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0;
        room[1] = 1;
        room[2] = 2;
        room[3] = 3;
        room[4] = 4;
        room[5] = 5;

        let mut mmu = Mmu::new();
        let mut cpu = Cpu::new();

        mmu.load_room(&room);
        assert!(cpu.r.pc == 0xf000);

        assert!(cpu.fetch(&mut mmu) == 0x00);
        assert!(cpu.r.pc == 0xf001);

        assert!(cpu.fetch(&mut mmu) == 0x01);
        assert!(cpu.r.pc == 0xf002);

        assert!(cpu.fetch(&mut mmu) == 0x02);
        assert!(cpu.r.pc == 0xf003);

        assert!(cpu.fetch(&mut mmu) == 0x03);
        assert!(cpu.r.pc == 0xf004);

        assert!(cpu.fetch(&mut mmu) == 0x04);
        assert!(cpu.r.pc == 0xf005);

        assert!(cpu.fetch(&mut mmu) == 0x05);
        assert!(cpu.r.pc == 0xf006);
    }

    #[test]
    fn cpu_ora_zero() {
        let mut cpu = Cpu::new();
        cpu.ora(0x00);
        assert!(cpu.r.pc == 0xf000);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
        assert!(cpu.clock == 0);
    }

    #[test]
    fn cpu_ora_neg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1010_0101;
        cpu.ora(  0b1100_0011);
        assert!(cpu.r.pc == 0xf000);
        assert!(cpu.r.a == 0b1110_0111);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
        assert!(cpu.clock == 0);
    }

    #[test]
    fn cpu_asl_noflags() {
        let mut cpu = Cpu::new();
        cpu.asl(0b0001_1000);
        assert!(cpu.r.a == 0b0011_0000);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_asl_neg() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.asl(0b0101_0101);
        assert!(cpu.r.a == 0b1010_1010);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_asl_zero_carry() {
        let mut cpu = Cpu::new();
        cpu.asl(0b1000_0000);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_lsr_carry() {
        let mut cpu = Cpu::new();
        cpu.lsr(0b1001_1001);
        assert!(cpu.r.a == 0b0100_1100);
        assert!(cpu.r.sr == status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_lsr_zero() {
        let mut cpu = Cpu::new();
        cpu.lsr(0b0000_0000);
        assert!(cpu.r.a == 0b0000_0000);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_lsr_noflags() {
        let mut cpu = Cpu::new();
        cpu.lsr(0b1001_1000);
        assert!(cpu.r.a == 0b0100_1100);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_rol_set_carry() {
        let mut cpu = Cpu::new();
        cpu.rol(0b1000_0001);
        assert!(cpu.r.a == 0b0000_0010);
        assert!(cpu.r.sr == status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_rol_reset_carry() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.rol(0b0000_0001);
        assert!(cpu.r.a == 0b0000_0011);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_rol_neg() {
        let mut cpu = Cpu::new();
        cpu.rol(0b0100_0000);
        assert!(cpu.r.a == 0b1000_0000);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_rol_keep_carry() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.rol(0b1000_0000);
        print!("{:?}", cpu);
        assert!(cpu.r.a == 0b0000_0001);
        assert!(cpu.r.sr == status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_rol_zero() {
        let mut cpu = Cpu::new();
        cpu.rol(0b0000_0000);
        assert!(cpu.r.a == 0b0000_0000);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_ror_set_carry() {
        let mut cpu = Cpu::new();
        cpu.ror(0b1000_0001);
        assert!(cpu.r.a == 0b0100_0000);
        assert!(cpu.r.sr == status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_ror_reset_carry() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.ror(0b0000_0000);
        assert!(cpu.r.a == 0b1000_0000);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_ror_keep_carry() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.ror(0b1000_0000);
        print!("{:?}", cpu);
        assert!(cpu.r.a == 0b1100_0000);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_ror_zero() {
        let mut cpu = Cpu::new();
        cpu.ror(0b0000_0000);
        assert!(cpu.r.a == 0b0000_0000);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_ora_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b0010_0101;
        cpu.ora(  0b0100_0011);
        assert!(cpu.r.pc == 0xf000);
        assert!(cpu.r.a == 0b0110_0111);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.clock == 0);
    }

    #[test]
    fn cpu_and_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1010_0101;
        cpu.and(0b0010_0100);
        assert!(cpu.r.a == 0b0010_0100);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_and_neg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1010_0101;
        cpu.and(0b1010_0100);
        assert!(cpu.r.a == 0b1010_0100);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_and_zero() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1111_1111;
        cpu.and(0b0000_0000);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_eor_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1010_0101;
        cpu.eor(0b1100_1100);
        assert!(cpu.r.a == 0b0110_1001);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_eor_neg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1111_1111;
        cpu.eor(0b0110_1001);
        assert!(cpu.r.a == 0b1001_0110);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_eor_zero() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1111_1111;
        cpu.eor(0b01111_1111);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_cmp_less() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0;
        cpu.cmp(1);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_cmp_equal() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0;
        cpu.cmp(0);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_cmp_greater() {
        let mut cpu = Cpu::new();
        cpu.r.a = 1;
        cpu.cmp(0);
        assert!(cpu.r.a == 1);
        assert!(cpu.r.sr == status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_cpx_less() {
        let mut cpu = Cpu::new();
        cpu.r.x = 0;
        cpu.cpx(1);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_cpx_equal() {
        let mut cpu = Cpu::new();
        cpu.r.x = 0;
        cpu.cpx(0);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_cpx_greater() {
        let mut cpu = Cpu::new();
        cpu.r.x = 1;
        cpu.cpx(0);
        assert!(cpu.r.x == 1);
        assert!(cpu.r.sr == status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_cpy_less() {
        let mut cpu = Cpu::new();
        cpu.r.y = 0;
        cpu.cpy(1);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_cpy_equal() {
        let mut cpu = Cpu::new();
        cpu.r.y = 0;
        cpu.cpy(0);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_cpy_greater() {
        let mut cpu = Cpu::new();
        cpu.r.y = 1;
        cpu.cpy(0);
        assert!(cpu.r.y == 1);
        assert!(cpu.r.sr == status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_bit_equal() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0b1000_0000;
        cpu.bit(0b1000_0000);
        assert!(cpu.r.a == 0b1000_0000);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_bit_flags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0;
        cpu.bit(0b1100_0000);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::OVER | status_flags::UNUSED);
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
    fn cpu_sec() {
        let mut cpu = Cpu::new();
        cpu.sec();
        assert!(cpu.r.sr == status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_sed() {
        let mut cpu = Cpu::new();
        cpu.sed();
        assert!(cpu.r.sr == status_flags::DEC | status_flags::UNUSED);
    }

    #[test]
    fn cpu_sei() {
        let mut cpu = Cpu::new();
        cpu.sei();
        assert!(cpu.r.sr == status_flags::IRQD | status_flags::UNUSED);
    }

    #[test]
    fn cpu_adc_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0x3a;
        cpu.adc(0x21);
        assert!(cpu.r.a == 0x5b);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_adc_with_carry_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::CARRY;
        cpu.r.a = 0x3a;
        cpu.adc(0x21);
        assert!(cpu.r.a == 0x5c);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_adc_carry_overflow_and_zero() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0xf0;
        cpu.adc(0x10);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::OVER | status_flags::CARRY |
            status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_adc_bcd_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::DEC | status_flags::UNUSED;
        cpu.r.a = 0x49;
        cpu.adc(0x12);
        assert!(cpu.r.a == 0x61);
        assert!(cpu.r.sr == status_flags::DEC | status_flags::UNUSED);
    }

    #[test]
    fn cpu_adc_bcd_with_carry_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::DEC | status_flags::CARRY | status_flags::UNUSED;
        cpu.r.a = 0x49;
        cpu.adc(0x12);
        assert!(cpu.r.a == 0x62);
        assert!(cpu.r.sr == status_flags::DEC | status_flags::UNUSED);
    }

    #[test]
    fn cpu_adc_bcd_carry_overflow_and_zero() {
        let mut cpu = Cpu::new();
        cpu.r.sr = status_flags::DEC | status_flags::UNUSED;
        cpu.r.a = 0x93;
        cpu.adc(0x07);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::CARRY |status_flags::ZERO |
            status_flags::OVER | status_flags::DEC | status_flags::UNUSED);
    }

    #[test]
    fn cpu_sbc_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0x74;
        cpu.sbc(0x32);
        assert!(cpu.r.a == 0x42);
        assert!(cpu.r.sr == status_flags::UNUSED, "{:x}", cpu.r.sr);
    }

    #[test]
    fn cpu_sbc_with_carry_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0x5e;
        cpu.sbc(0x29);
        assert!(cpu.r.a == 0x35);
        assert!(cpu.r.sr == status_flags::UNUSED, "{:x}", cpu.r.sr);
    }

    #[test]
    fn cpu_sbc_with_carry_zero() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0x15;
        cpu.r.sr = status_flags::CARRY;
        cpu.sbc(0x14);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_sbc_bcd_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0x24;
        cpu.r.sr = status_flags::DEC;
        cpu.sbc(0x15);
        assert!(cpu.r.a == 0x09);
        assert!(cpu.r.sr == status_flags::DEC | status_flags::UNUSED);
    }

    #[test]
    fn cpu_sbc_bcd_carry_overflow() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0x15;
        cpu.r.sr = status_flags::DEC | status_flags::CARRY;
        cpu.sbc(0x24);
        assert!(cpu.r.a == 0x90);
        assert!(cpu.r.sr == status_flags::DEC | status_flags::NEG | status_flags::CARRY | status_flags::UNUSED);
    }

    #[test]
    fn cpu_inc_noflags() {
        let mut cpu = Cpu::new();
        assert!(cpu.inc(0x49) == 0x4a);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_inc_neg() {
        let mut cpu = Cpu::new();
        assert!(cpu.inc(0x80) == 0x81);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_inc_zero() {
        let mut cpu = Cpu::new();
        assert!(cpu.inc(0xff) == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_dec_noflags() {
        let mut cpu = Cpu::new();
        assert!(cpu.dec(0x80) == 0x7f);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_dec_zero() {
        let mut cpu = Cpu::new();
        assert!(cpu.dec(0x01) == 0x00);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
    }

    #[test]
    fn cpu_dec_neg() {
        let mut cpu = Cpu::new();
        assert!(cpu.dec(0x00) == 0xff);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
    }

    #[test]
    fn cpu_lda() {
        let mut cpu = Cpu::new();
        cpu.lda(0x88);
        assert!(cpu.r.a == 0x88);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_ldx() {
        let mut cpu = Cpu::new();
        cpu.ldx(0x3a);
        assert!(cpu.r.x == 0x3a);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_ldy() {
        let mut cpu = Cpu::new();
        cpu.ldy(0xab);
        assert!(cpu.r.y == 0xab);
        assert!(cpu.r.sr == status_flags::UNUSED);
    }

    #[test]
    fn cpu_tax_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0x14;
        cpu.tax();
        assert!(cpu.r.a == 0x14);
        assert!(cpu.r.x == 0x14);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tax_neg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0xb1;
        cpu.tax();
        assert!(cpu.r.a == 0xb1);
        assert!(cpu.r.x == 0xb1);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tax_zero() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0;
        cpu.tax();
        assert!(cpu.r.a == 0);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tay_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0x41;
        cpu.tay();
        assert!(cpu.r.a == 0x41);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.y == 0x41);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tay_neg() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0xc4;
        cpu.tay();
        assert!(cpu.r.a == 0xc4);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.y == 0xc4);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tay_zero() {
        let mut cpu = Cpu::new();
        cpu.r.a = 0;
        cpu.tay();
        assert!(cpu.r.a == 0);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tsx_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.sp = 0x5e;
        cpu.tsx();
        assert!(cpu.r.a == 0);
        assert!(cpu.r.x == 0x5e);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.r.sp == 0x5e);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tsx_neg() {
        let mut cpu = Cpu::new();
        cpu.r.sp = 0xee;
        cpu.tsx();
        assert!(cpu.r.a == 0);
        assert!(cpu.r.x == 0xee);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
        assert!(cpu.r.sp == 0xee);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tsx_zero() {
        let mut cpu = Cpu::new();
        cpu.r.sp = 0;
        cpu.tsx();
        assert!(cpu.r.a == 0);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_txa_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.x = 0x11;
        cpu.txa();
        assert!(cpu.r.a == 0x11);
        assert!(cpu.r.x == 0x11);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_txa_neg() {
        let mut cpu = Cpu::new();
        cpu.r.x = 0x91;
        cpu.txa();
        assert!(cpu.r.a == 0x91);
        assert!(cpu.r.x == 0x91);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_txa_zero() {
        let mut cpu = Cpu::new();
        cpu.r.x = 0;
        cpu.txa();
        assert!(cpu.r.a == 0);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::ZERO|status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_txs() {
        let mut cpu = Cpu::new();
        cpu.r.x = 0xa1;
        cpu.txs();
        assert!(cpu.r.a == 0);
        assert!(cpu.r.x == 0xa1);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.r.sp == 0xa1);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tya_noflags() {
        let mut cpu = Cpu::new();
        cpu.r.y = 0x3c;
        cpu.tya();
        assert!(cpu.r.a == 0x3c);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.y == 0x3c);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tya_neg() {
        let mut cpu = Cpu::new();
        cpu.r.y = 0xbb;
        cpu.tya();
        assert!(cpu.r.a == 0xbb);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.y == 0xbb);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_tya_zero() {
        let mut cpu = Cpu::new();
        cpu.r.y = 0;
        cpu.tya();
        assert!(cpu.r.a == 0);
        assert!(cpu.r.x == 0);
        assert!(cpu.r.y == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
        assert!(cpu.r.sp == 0);
        assert!(cpu.r.pc == 0xf000);
    }

    #[test]
    fn cpu_sta() {
        let mut mmu = Mmu::new();

        let mut cpu = Cpu::new();
        cpu.r.a = 0xea;
        cpu.r.x = 0x1f;
        cpu.r.y = 0x80;
        cpu.sta(&mut mmu, 0x100);

        assert!(mmu.read(0x100) == 0xea);
    }

    #[test]
    fn cpu_stx() {
        let mut mmu = Mmu::new();

        let mut cpu = Cpu::new();
        cpu.r.a = 0xea;
        cpu.r.x = 0x1f;
        cpu.r.y = 0x80;
        cpu.stx(&mut mmu, 0x100);
        assert!(mmu.read(0x100) == 0x1f);
    }

    #[test]
    fn cpu_sty() {
        let mut mmu = Mmu::new();

        let mut cpu = Cpu::new();
        cpu.r.a = 0xea;
        cpu.r.x = 0x1f;
        cpu.r.y = 0x80;
        cpu.sty(&mut mmu, 0x100);

        assert!(mmu.read(0x100) == 0x80);
    }

    #[test]
    fn cpu_jmp() {
        let mut cpu = Cpu::new();
        cpu.jmp(0x3489);
        assert!(cpu.r.pc == 0x3489);
    }

    #[test]
    fn cpu_instruction_brk() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x00;
        room[1] = 0x00;
        room[2] = 0x00;
        room[2046] = 0x11;
        room[2047] = 0x99;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.step(&mut mmu);

        assert!(cpu.r.pc == 0x9911);
        assert!(cpu.r.sr == status_flags::BRK | status_flags::UNUSED);
        assert!(cpu.clock == 7);
    }

    #[test]
    fn cpu_instruction_ora_immidiate() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x09;
        room[1] = 0x00;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.step(&mut mmu);

        assert!(cpu.r.pc == 0xf002);
        assert!(cpu.r.a == 0);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
        assert!(cpu.clock == 2);
    }

    #[test]
    fn cpu_instruction_ora_x_ind() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x01;
        room[1] = 0x81;
        room[2] = 0x00;
        room[3] = 0x00;
        room[4] = 0b0011_1001;

        let mut mmu = Mmu::new();
        mmu.write(0x82, 0x04);
        mmu.write(0x83, 0xf0);
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.x = 1;
        cpu.step(&mut mmu);

        // execution:
        //   offset = 0x01(op) + 0x01(x) => 0x02
        //   mem[offset] = 0x0004
        //   value[mem[offset]] => 0b0011_1001
        assert!(cpu.r.pc == 0xf002);
        assert!(cpu.r.a == 0b0011_1001);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.clock == 6);
    }

    #[test]
    fn cpu_instruction_ora_ind_y() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x11;
        room[1] = 0x02;
        room[2] = 0x04;
        room[3] = 0xf0;
        room[4] = 0x00;
        room[5] = 0xff;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.a = 0b1001_0110;
        cpu.r.y = 1;
        cpu.step(&mut mmu);
        // execution:
        //   offset = 0x02(op)
        //   mem[offset] = 0x0004
        //   value[mem[offset] + 0x01(x)] = 0xff
        assert!(cpu.r.pc == 0xf002);
        assert!(cpu.r.a == 0xff);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
        assert!(cpu.clock == 5);
    }

    #[test]
    fn cpu_instruction_ora_zpg() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x05;
        room[1] = 0x80;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);
        mmu.write(0x80, 0b1111_0000);
        mmu.write(0x81, 0b0000_1111);

        let mut cpu = Cpu::new();
        cpu.r.a = 0b0110_0000;
        cpu.step(&mut mmu);

        assert!(cpu.r.pc == 0xf002);
        assert!(cpu.r.a == 0b1111_0000);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
        assert!(cpu.clock == 3);
    }

    #[test]
    fn cpu_instruction_ora_zpg_x() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x15;
        room[1] = 0x80;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);
        mmu.write(0x83, 0b1111_0000);
        mmu.write(0x84, 0b0000_1111);

        let mut cpu = Cpu::new();
        cpu.r.a = 0b0110_0000;
        cpu.r.x = 3;
        cpu.step(&mut mmu);
        assert!(cpu.r.pc == 0xf002);
        assert!(cpu.r.a == 0b1111_0000);
        assert!(cpu.r.sr == status_flags::UNUSED | status_flags::NEG);
        assert!(cpu.clock == 4);
    }

    #[test]
    fn cpu_instruction_ora_abs() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x0d;
        room[1] = 0x03;
        room[2] = 0xf0;
        room[3] = 0b0000_1111;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.a = 0b0110_0000;
        cpu.step(&mut mmu);

        assert!(cpu.r.pc == 0xf003);
        assert!(cpu.r.a == 0b0110_1111);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.clock == 4);
    }

    #[test]
    fn cpu_instruction_ora_abs_x() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x1d;
        room[1] = 0x03;
        room[2] = 0xf0;
        room[3] = 0b1111_0000;
        room[4] = 0b0000_1111;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.a = 0b0001_0000;
        cpu.r.x = 1;
        cpu.step(&mut mmu);

        assert!(cpu.r.pc == 0xf003);
        assert!(cpu.r.a == 0b0001_1111);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.clock == 4);
    }

    #[test]
    fn cpu_instruction_ora_abs_y() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x19;
        room[1] = 0x03;
        room[2] = 0xf0;
        room[3] = 0b1111_0000;
        room[4] = 0b0000_1111;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.a = 0b0001_0000;
        cpu.r.y = 1;
        cpu.step(&mut mmu);

        assert!(cpu.r.pc == 0xf003);
        assert!(cpu.r.a == 0b0001_1111);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(cpu.clock == 4);
    }

    #[test]
    fn cpu_instruction_jmp_abs() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x4c;
        room[1] = 0x21;
        room[2] = 0x43;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.step(&mut mmu);

        assert!(cpu.r.pc == 0x4321);
    }

    #[test]
    fn cpu_instruction_jmp_ind() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x6c;
        room[1] = 0x03;
        room[2] = 0xf0;
        room[3] = 0xef;
        room[4] = 0xbe;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.step(&mut mmu);

        assert!(cpu.r.pc == 0xbeef);
    }

    #[test]
    fn cpu_instruction_jsr() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0] = 0x20;
        room[1] = 0xef;
        room[2] = 0xbe;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.sp = 0xff;
        cpu.step(&mut mmu);

        assert!(cpu.r.sp == 0xfd);
        assert!(cpu.r.pc == 0xbeef);
        assert!(mmu.read(0x1ff) == 0xf0);
        assert!(mmu.read(0x1fe) == 0x02);
    }

    #[test]
    fn cpu_instruction_pha() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0x00] = 0x48;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.a = 0x34;
        cpu.r.sr = 0x91;
        cpu.r.sp = 0xff;
        cpu.step(&mut mmu);

        assert!(cpu.r.sp == 0xfe);
        assert!(mmu.read(0x1ff) == 0x34);
    }

    #[test]
    fn cpu_instruction_php() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0x00] = 0x08;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.a = 0x34;
        cpu.r.sr = 0x91;
        cpu.r.sp = 0xff;
        cpu.step(&mut mmu);

        assert!(cpu.r.sp == 0xfe);
        assert!(mmu.read(0x1ff) == 0x91);
    }

    #[test]
    fn cpu_instruction_pla_noflags() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0x00] = 0x68;

        let mut mmu = Mmu::new();
        mmu.load_room(&room);
        mmu.write(0x1ff, 0x39);

        let mut cpu = Cpu::new();
        cpu.r.sp = 0xfe;
        cpu.step(&mut mmu);

        assert!(cpu.r.sp == 0xff);
        assert!(cpu.r.sr == status_flags::UNUSED);
        assert!(mmu.read(0x1ff) == 0x39);
    }

    #[test]
    fn cpu_instruction_pla_neg() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0x00] = 0x68;

        let mut mmu = Mmu::new();
        mmu.write(0x1ff, 0xab);
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.sp = 0xfe;
        cpu.step(&mut mmu);

        assert!(cpu.r.sp == 0xff);
        assert!(cpu.r.sr == status_flags::NEG | status_flags::UNUSED);
        assert!(mmu.read(0x1ff) == 0xab);
    }

    #[test]
    fn cpu_instruction_pla_zero() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0x00] = 0x68;

        let mut mmu = Mmu::new();
        mmu.write(0x1ff, 0);
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.sp = 0xfe;
        cpu.step(&mut mmu);

        assert!(cpu.r.sp == 0xff);
        assert!(cpu.r.sr == status_flags::ZERO | status_flags::UNUSED);
        assert!(mmu.read(0x1ff) == 0x00);
    }

    #[test]
    fn cpu_instruction_plp() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0x00] = 0x28;

        let mut mmu = Mmu::new();
        mmu.write(0x1ff, 0x91);
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.sp = 0xfe;
        cpu.step(&mut mmu);

        assert!(cpu.r.sp == 0xff);
        assert!(cpu.r.sr == 0x91 | status_flags::UNUSED);
        assert!(mmu.read(0x1ff) == 0x91);
    }

    #[test]
    fn cpu_instruction_rti() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0x00] = 0x40;

        let mut mmu = Mmu::new();
        mmu.write(0x1ff, 0xbe); // PCH
        mmu.write(0x1fe, 0xef); // PCL
        mmu.write(0x1fd, 0x91); // SR
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.sp = 0xfc;
        cpu.step(&mut mmu);

        assert!(cpu.r.sp == 0xff);
        assert!(cpu.r.sr == 0x91 | status_flags::UNUSED);
        assert!(cpu.r.pc == 0xbeef);
    }

    #[test]
    fn cpu_instruction_rts() {
        let mut room : Vec<u8> = Vec::new();
        room.resize(2048, 0xff);
        room[0x00] = 0x60;
        room[0x1ff] = 0xbe; // PCH
        room[0x1fe] = 0xee; // PCL

        let mut mmu = Mmu::new();
        mmu.write(0x1ff, 0xbe); // PCH
        mmu.write(0x1fe, 0xee); // PCL
        mmu.load_room(&room);

        let mut cpu = Cpu::new();
        cpu.r.sp = 0xfd;
        cpu.step(&mut mmu);

        assert!(cpu.r.sp == 0xff);
        assert!(cpu.r.pc == 0xbeef);
    }
}
