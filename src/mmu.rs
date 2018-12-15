use std::fmt;

/// MMU
#[allow(dead_code)]
pub struct Mmu {
    memory: Vec<u8>,
    room_size: usize
}
impl fmt::Debug for Mmu {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "room_size:{:?}", self.room_size)
    }
}

#[allow(dead_code)]
impl Mmu {
    pub fn new() -> Mmu {
        Mmu {
            memory: {
                let mut v = Vec::new();
                v.resize(u16::max_value() as usize + 1, 0xff);
                v
            },
            room_size: 0
        }
    }

    /// Load room to memory
    pub fn load(&mut self, room: &[u8]) {
        self.memory[..room.len()].clone_from_slice(&room[..room.len()]);
        self.room_size = room.len();
    }

    /// Read byte from a given address
    pub fn read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    /// Write a byte on a given address
    pub fn write(&mut self, addr: u16, value: u8) {
        self.memory[addr as usize] = value;
    }
}