mod registers;

fn main() {
    let r = registers::Registers::new();
    println!("{:?}", r);
}
