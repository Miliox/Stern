mod cpu;
mod mmu;

use std::env;
use std::fs::File;
use std::io::Read;
use std::{thread, time};

const FRAME_RATE : u64 = 60;
const FRAME_DURATION : time::Duration = time::Duration::from_nanos(1_000_000_000 / FRAME_RATE);

const CRYSTAL_TICKS_PER_SECOND : u64 = 14_318_180;
const CPU_TICKS_PER_SECOND     : u64 = CRYSTAL_TICKS_PER_SECOND / 12;
const CPU_TICKS_PER_FRAME      : u64 = CPU_TICKS_PER_SECOND / FRAME_RATE;



fn main() {
    if let Some(filename) = env::args().nth(1) {
        println!("Room: {:?}", filename);

        let mut room = Vec::new();
        let mut file = File::open(filename).unwrap();
        file.read_to_end(&mut room).unwrap();

        println!("Size: {:?} bytes", room.len());

        let mut cpu = cpu::Cpu::new();
        cpu.load_room(&room);
        println!("{:?}", cpu);

        loop {
            cpu.step();
            println!("{:?}", cpu);

            if cpu.clock >= CPU_TICKS_PER_FRAME {
                cpu.clock -= CPU_TICKS_PER_FRAME;
                thread::sleep(FRAME_DURATION);
            }
        }
    }

}
