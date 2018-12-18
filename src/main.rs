mod cpu;
mod mmu;

use std::env;
use std::fs::File;
use std::io::Read;
use std::{thread, time};

const FRAME_RATE : u64 = 60;
const FRAME_DURATION : time::Duration = time::Duration::from_nanos(1_000_000_000 / FRAME_RATE);

const ZERO_DURATION : time::Duration = time::Duration::from_secs(0);

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
        // println!("{:?}", cpu);

        let mut oversleep = ZERO_DURATION;
        let mut beg_frame = time::Instant::now();
        loop {
            cpu.step();
            // println!("{:?}", cpu);

            // Match emulator speed to real 6507
            if cpu.clock >= CPU_TICKS_PER_FRAME {
                cpu.clock -= CPU_TICKS_PER_FRAME;

                let busy_duration = time::Instant::now() - beg_frame;
                let sleep_duration = FRAME_DURATION - busy_duration - oversleep;

                // The current processors are much faster than the original 6507, so
                // to try match the speed we put the thread to sleep lot!
                thread::sleep(sleep_duration);

                let end_frame = time::Instant::now();
                let frame_duration = end_frame - beg_frame;
                beg_frame = end_frame;

                // Sleep is not a precise function, it only guaratee that the thread will sleep
                // for at least the given time and awake as soon as possible. But the accumulate
                // difference will slowdown the produced frame rate.
                //
                // So an ammortization is necessary to bring the frame rate as close as possible
                // to the target one that is done by compensating the over time that the thread
                // slept on the next frame.
                oversleep = frame_duration - (busy_duration + sleep_duration);

                //println!("{}\t{}\t{}",
                //    frame_duration.subsec_nanos(),
                //    oversleep.subsec_nanos(),
                //    FRAME_DURATION.subsec_nanos());
            }
        }
    }

}
