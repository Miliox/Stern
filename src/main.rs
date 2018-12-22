pub mod emulator;
pub mod cpu;
pub mod mmu;

use std::env;

fn main() {
    if let Some(filename) = env::args().nth(1) {
        println!("Room: {:?}", filename);

        let mut emulator = emulator::Emulator::new();
        emulator.set_room_filepath(filename);
        emulator.run();
    }

}
