use std::fs::File;
use std::io::Read;
use std::{thread, time};

use crate::cpu::Cpu;
use crate::mmu::Mmu;
use crate::debugger::Debugger;

const FRAME_RATE : u64 = 60;
const FRAME_DURATION : time::Duration = time::Duration::from_nanos(1_000_000_000 / FRAME_RATE);

const ZERO_DURATION : time::Duration = time::Duration::from_secs(0);

const CRYSTAL_TICKS_PER_SECOND : u64 = 14_318_180;
const CPU_TICKS_PER_SECOND     : u64 = CRYSTAL_TICKS_PER_SECOND / 12;
const CPU_TICKS_PER_FRAME      : u64 = CPU_TICKS_PER_SECOND / FRAME_RATE;

#[allow(dead_code)]
pub struct Emulator {
    cpu: Cpu,
    mmu: Mmu,
    debugger: Debugger,
}

#[allow(dead_code)]
impl Emulator {
    pub fn new() -> Emulator {
        Emulator {
            cpu: Cpu::new(),
            mmu: Mmu::new(),
            debugger: Debugger::new()
        }
    }

    pub fn set_room_filepath(&mut self, filepath: String) {
        let mut room = Vec::new();
        let mut file = File::open(filepath).unwrap();
        file.read_to_end(&mut room).unwrap();

        println!("Size: {:?} bytes", room.len());
        self.mmu.load_room(&room);
    }

    pub fn run(&mut self) {
        let mut oversleep = ZERO_DURATION;
        let mut beg_frame = time::Instant::now();
        loop {
            self.cpu.step(&mut self.mmu);
            // println!("{:?}", self.cpu);

            // Match emulator speed to real 6507
            if self.cpu.clock >= CPU_TICKS_PER_FRAME {
                self.cpu.clock -= CPU_TICKS_PER_FRAME;

                self.debugger.refresh();

                let busy_duration = time::Instant::now() - beg_frame;

                oversleep = match FRAME_DURATION.checked_sub(busy_duration + oversleep) {
                    Some(sleep_duration) => {
                        // The modern cpus are ridiculously faster than the 6507.
                        // So to emulate the speed original the process has to sleep a lot!
                        thread::sleep(sleep_duration);

                        let end_frame = time::Instant::now();
                        let frame_duration = end_frame - beg_frame;
                        beg_frame = end_frame;

                        // Sleep is not precise, it always takes a bit longer that the specified
                        // duration. This slowdown the produced frame rate overtime.
                        //
                        // So an ammortization is necessary to bring the frame rate up.
                        // This is done on the next frame.
                        let oversleep = frame_duration - (busy_duration + sleep_duration);

                        // println!("{}.{:0>9}\t{}.{:0>9}\t{}.{:0>9}\t{}.{:0>9}\t{}.{:0>9}",
                        //    frame_duration.as_secs(), frame_duration.subsec_nanos(),
                        //    busy_duration.as_secs(), busy_duration.subsec_nanos(),
                        //    sleep_duration.as_secs(), sleep_duration.subsec_nanos(),
                        //    oversleep.as_secs(), oversleep.subsec_nanos(),
                        //    FRAME_DURATION.as_secs(), FRAME_DURATION.subsec_nanos());

                        oversleep
                    }
                    None => ZERO_DURATION
                }


            }
        }
    }
}
