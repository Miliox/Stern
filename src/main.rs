mod cpu;

fn main() {
    let r = cpu::Registers::new();
    println!("{:?}", r);
}
