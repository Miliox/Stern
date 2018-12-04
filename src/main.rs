mod cpu;

use std::env;

fn main() {
    if let Some(room) = env::args().nth(1) {
        println!("Room: {:?}", room);
    }

    let r = cpu::Cpu::new();
    println!("{:?}", r);
}
