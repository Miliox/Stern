
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
#[derive(Debug)]
pub struct Registers {
    /// Program Counter
    pc : u16,

    /// Accumulator
    ac : u8,

    /// Index X
    x : u8,

    /// Index Y
    y : u8,

    /// Status
    sr : u8,

    /// Stack Pointer
    sp : u8
}

impl Registers {
    pub fn new() -> Registers {
        Registers { pc: 0, ac: 0, x: 0, y: 0, sr: 0, sp: 0 }
    }
}
