use std::fmt;

const TIA_SIZE  : usize = 0x30;
const RIOT_SIZE : usize = 0x20;
const ZERO_SIZE : usize = 0x80;

const BANK_SIZE : usize = 4096;
const HALF_BANK : usize = BANK_SIZE / 2;

const TIA_RIOT_MASK : u16 = 0x3ff;

const ADDR_SIZE : usize = 1 << 13;
const ADDR_MASK : u16 = ADDR_SIZE as u16 - 1;

// Allocated a u8 vector with size
fn vec_with_size(size: usize, value: u8) -> Vec<u8> {
    std::iter::repeat(value).take(size).collect::<Vec<u8>>()
}

/// MMU
#[allow(dead_code)]
pub struct Mmu {
    /// Data from the Current Room Bank
    bank: Vec<u8>,

    /// Index of the Current Room Bank
    bank_number: u8,

    /// RIOT: 6532 RAM-I/O-Timer (RIOT) Registers
    riot: Vec<u8>,

    /// Full Carthridge Room Data
    room: Vec<u8>,

    /// TIA Television Interface Adaptor Registers
    tia: Vec<u8>,

    /// Zero Page: RAM Primary Image
    zero_page: Vec<u8>,
}
impl fmt::Debug for Mmu {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "room_size:{:?}", self.room.len())
    }
}

/// Memory Management Unit
#[allow(dead_code)]
impl Mmu {
    pub fn new() -> Mmu {
        Mmu {
            bank: vec_with_size(BANK_SIZE, 0xff),
            bank_number: 0,
            riot: vec_with_size(RIOT_SIZE, 0xff),
            room: Vec::new(),
            tia: vec_with_size(TIA_SIZE, 0xff),
            zero_page: vec_with_size(ZERO_SIZE, 0xff),
        }
    }

    /// Load room to memory
    pub fn load_room(&mut self, room: &[u8]) {
        if room.len() == HALF_BANK || room.len() / BANK_SIZE >= 1 && room.len() % BANK_SIZE == 0 {
            self.room = room.to_vec();

            self.bank_number = 0;
            if room.len() == HALF_BANK {
                // $1000-17FF Lower 2K Cartridge ROM (2K carts mirrored here)
                self.bank[..HALF_BANK].clone_from_slice(&room[..HALF_BANK]);
                // $1800-1FFF Upper 2K Cartridge ROM (2K carts go here)
                self.bank[HALF_BANK..BANK_SIZE].clone_from_slice(&room[..HALF_BANK]);
            } else {
                // $1000-1FFF Lower 2K Cartridge ROM (4K carts start here)
                // $1800-1FFF Upper 2K Cartridge ROM (2K carts go here)
                self.bank[..BANK_SIZE].clone_from_slice(&room[..BANK_SIZE]);
            }
        } else {
            panic!("Room must be at least {} or multiple of {}, but was {}", HALF_BANK, BANK_SIZE, room.len());
        }
    }

    /// Read byte from a given address
    pub fn read(&self, mut addr: u16) -> u8 {
        addr &= ADDR_MASK;

        if addr >= 0x1000 {
            self.bank[(addr - 0x1000) as usize]
        } else {
            addr &= TIA_RIOT_MASK;

            // Memory Map
            if addr <= 0x200 {
                // only last by matters
                addr &= 0xff;

                if addr <= 0x2f {
                    // $0000-002F TIA Primary Image
                    // $0100-012F [shadow] TIA
                    self.tia[addr as usize]
                } else if addr <= 0x5f {
                    // $0030-005F [shadow] TIA
                    // $0130-005F [shadow] TIA
                    self.tia[(addr - 0x30) as usize]
                } else if addr <= 0x7f {
                    // 0060-007F [shadow-partial] TIA
                    // $0160-017F [shadow-partial] TIA
                    self.tia[(addr - 0x60) as usize]
                } else /* if addr <= 0xff */ {
                    // $0080-00FF 128 bytes of RAM Primary Image (zero page image)
                    // $0180-01FF [shadow] 128 bytes of RAM (CPU stack image)
                    self.zero_page[(addr - 0x80) as usize]
                }
            } else if addr <= 0x3ff {
                addr &= 0xff;

                if addr <= 0x2f {
                    // $0200-022F [shadow] TIA
                    // $0300-032F [shadow] TIA
                    self.tia[addr as usize]
                } else if addr <= 0x5f {
                    // $0230-025F [shadow] TIA
                    // $0330-035F [shadow] TIA
                    self.tia[(addr - 0x30) as usize]
                } else if addr <= 0x7f {
                    // $0260-027F [shadow-partial] TIA
                    // $0360-037F [shadow-partial] TIA
                    self.tia[(addr - 0x60) as usize]
                } else if addr <= 0x9f {
                    // $0280-029F 6532-PIA I/O ports and timer Primary image
                    self.riot[(addr - 0x80) as usize]
                } else if addr <= 0xbf {
                    // $02A0-02BF [shadow] 6532-PIA
                    self.riot[(addr - 0xa0) as usize]
                } else if addr <= 0xdf {
                    // $02C0-02DF [shadow] 6532-PIA
                    self.riot[(addr - 0xa0) as usize]
                } else /* if addr <= 0xff */ {
                    // $02D0-02FF [shadow] 6532-PIA
                    self.riot[(addr - 0xd0) as usize]
                }
            } else {
                panic!("Something went very wrong! addr={:x}", addr)
            }
        }
    }

    /// Write a byte on a given address
    pub fn write(&mut self, mut addr: u16, value: u8) {
        addr &= ADDR_MASK;

        if addr < 0x1000 {
            addr &= TIA_RIOT_MASK;

            // Memory Map
            if addr <= 0x200 {
                // only last by matters
                addr &= 0xff;

                if addr <= 0x2f {
                    // $0000-002F TIA Primary Image
                    // $0100-012F [shadow] TIA
                    self.tia[addr as usize] = value;
                } else if addr <= 0x5f {
                    // $0030-005F [shadow] TIA
                    // $0130-005F [shadow] TIA
                    self.tia[(addr - 0x30) as usize] = value;
                } else if addr <= 0x7f {
                    // 0060-007F [shadow-partial] TIA
                    // $0160-017F [shadow-partial] TIA
                    self.tia[(addr - 0x60) as usize] = value;
                } else /* if addr <= 0xff */ {
                    // $0080-00FF 128 bytes of RAM Primary Image (zero page image)
                    // $0180-01FF [shadow] 128 bytes of RAM (CPU stack image)
                    self.zero_page[(addr - 0x80) as usize] = value;
                }
            } else if addr <= 0x3ff {
                addr &= 0xff;

                if addr <= 0x2f {
                    // $0200-022F [shadow] TIA
                    // $0300-032F [shadow] TIA
                    self.tia[addr as usize] = value;
                } else if addr <= 0x5f {
                    // $0230-025F [shadow] TIA
                    // $0330-035F [shadow] TIA
                    self.tia[(addr - 0x30) as usize] = value;
                } else if addr <= 0x7f {
                    // $0260-027F [shadow-partial] TIA
                    // $0360-037F [shadow-partial] TIA
                    self.tia[(addr - 0x60) as usize] = value;
                } else if addr <= 0x9f {
                    // $0280-029F 6532-PIA I/O ports and timer Primary image
                    self.riot[(addr - 0x80) as usize]= value;
                } else if addr <= 0xbf {
                    // $02A0-02BF [shadow] 6532-PIA
                    self.riot[(addr - 0xa0) as usize] = value;
                } else if addr <= 0xdf {
                    // $02C0-02DF [shadow] 6532-PIA
                    self.riot[(addr - 0xa0) as usize] = value;
                } else /* if addr <= 0xff */ {
                    // $02D0-02FF [shadow] 6532-PIA
                    self.riot[(addr - 0xd0) as usize] = value;
                }
            }
        }
    }
}