mod cpu;

fn main() {
    let r = cpu::Cpu::new();
    println!("{:?}", r);
}
