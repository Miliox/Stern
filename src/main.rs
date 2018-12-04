mod cpu;

use std::env;
use std::fs::File;
use std::io::Read;

fn main() {
    if let Some(filename) = env::args().nth(1) {
        println!("Room: {:?}", filename);

        let mut room = Vec::new();
        let mut file = File::open(filename).unwrap();
        file.read_to_end(&mut room).unwrap();

        println!("Size: {:?} bytes", room.len());

        let mut cpu = cpu::Cpu::new();
        cpu.load(room);
        println!("{:?}", cpu);
    }

}
